# debug_view.py
import cv2
import time
import math
import threading
import queue
import numpy as np
from mission_types import UISnapshot, Event, event_bus
from config import (DEBUG_WINDOW_WIDTH, DEBUG_WINDOW_HEIGHT,
                    OBJECT_CENTER_DOT_RADIUS, OBJECT_TO_IMAGE_CENTER_LINE_COLOR,
                    OBJECT_TO_IMAGE_CENTER_LINE_THICKNESS, DISTANCE_ESTIMATION_GAIN)

FONT = cv2.FONT_HERSHEY_SIMPLEX


class DebugView:
    """
    Handles OpenCV drawing and telemetry overlays. Camera feed occupies ~70% of
    the combined canvas width; the remaining ~30% is a dark, panel-based
    mission dashboard (Mission / Vehicle / Vision / Servo / Payload / Health /
    Diagnostics / Timeline / Mission Map), topped by a full-width safety banner.
    """

    # Dark theme palette (BGR)
    COL_BG          = (22, 22, 24)
    COL_PANEL_BG    = (34, 34, 38)
    COL_PANEL_BORDER = (58, 58, 64)
    COL_HEADER_BG   = (48, 48, 54)
    COL_ACCENT      = (235, 178, 60)    # amber
    COL_TEXT        = (232, 232, 232)
    COL_TEXT_DIM    = (150, 150, 156)
    COL_GOOD        = (110, 220, 120)
    COL_WARN        = (60, 210, 235)
    COL_BAD         = (70, 70, 235)
    COL_CYAN        = (235, 210, 70)

    # Final display upscale factor applied to the whole combined canvas right
    # before imshow (see draw_dashboard). Was previously shown at raw camera
    # resolution (no scaling at all), which read as too small on most
    # monitors with panel text at the edge of legibility.
    DISPLAY_SCALE = 1.35

    def __init__(self):
        self.prev_time = time.time()
        self.frame_count = 0
        self.fps = 0.0
        self.window_initialized = False

    # ------------------------------------------------------------------
    # small drawing helpers
    # ------------------------------------------------------------------
    def _text(self, img, text, x, y, color=None, scale=0.5, thick=1):
        cv2.putText(img, text, (x, y), FONT, scale, color or self.COL_TEXT, thick, cv2.LINE_AA)

    def _panel(self, img, x0, x1, y, height, title):
        """Draws a bordered, titled panel box and returns the y-coordinate
        where the panel's body content should start."""
        cv2.rectangle(img, (x0, y), (x1, y + height), self.COL_PANEL_BG, -1)
        cv2.rectangle(img, (x0, y), (x1, y + height), self.COL_PANEL_BORDER, 1)
        cv2.rectangle(img, (x0, y), (x1, y + 22), self.COL_HEADER_BG, -1)
        self._text(img, title, x0 + 10, y + 16, self.COL_ACCENT, 0.55, 1)
        return y + 40

    def draw_dashboard(self, snapshot: UISnapshot, events: list, target_markers: dict = None, path_history: list = None):
        if snapshot.frame_bgr is None:
            return

        frame = snapshot.frame_bgr.copy()
        h, w = frame.shape[:2]

        # ---------------------------------------------------------
        # CAMERA PANEL (~70% of the combined canvas width)
        # ---------------------------------------------------------
        cx, cy = w // 2, h // 2

        cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 1, cv2.LINE_AA)

        if snapshot.target_data:
            tx, ty = snapshot.target_data["center"]

            cv2.line(frame, (cx, cy), (tx, ty), (0, 255, 255), 2, cv2.LINE_AA)
            cv2.circle(frame, (tx, ty), 6, (0, 0, 255), -1, cv2.LINE_AA)

            # Bounding box is stored as absolute (x1, y1, x2, y2) corners -- NOT
            # (x, y, w, h). Drawing it as (x, y, x+w, y+h) previously rendered a
            # box roughly 2x too large, skewed toward the bottom-right corner.
            # Still used for label placement below even when a polygon is drawn.
            x1, y1, x2, y2 = snapshot.target_data.get("bounding_box", (0, 0, 0, 0))

            # Trace the actual detected outline (the hexagon's 6 edges, the
            # triangle's 3 edges -- HSVContourDetectorBackend's own
            # cv2.approxPolyDP result, carried through unchanged) instead of
            # a generic axis-aligned green box, so the overlay demonstrates
            # the algorithm found the shape's real edges, not just its
            # extent. Falls back to the box only for detections that never
            # had a polygon (e.g. a box-only backend like YOLO).
            polygon = snapshot.target_data.get("polygon")
            if polygon and len(polygon) >= 3:
                pts = np.array(polygon, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], True, (0, 255, 0), 3, cv2.LINE_AA)
            elif x2 > x1 and y2 > y1:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2, cv2.LINE_AA)

            label = f"{snapshot.target_data.get('color', '').upper()} {snapshot.target_data.get('name', '').upper()}"
            cv2.putText(frame, label, (x1, max(0, y1 - 8)), FONT, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Conf: {snapshot.target_data.get('confidence', 0):.2f}",
                        (x1, min(h - 5, y2 + 20)), FONT, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

            dx, dy = snapshot.target_data.get("pixel_error", (0, 0))
            cv2.putText(frame, f"ERR: {dx:.2f}, {dy:.2f}", (cx + 20, cy + 20),
                        FONT, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

        # Slim glance-strip over the top of the camera feed
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 28), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)
        self._text(frame, f"{snapshot.mission_state}  |  ALT {snapshot.alt_rel:.1f}m  |  {snapshot.flight_mode}",
                    10, 19, (255, 255, 255), 0.55, 1)

        # ---------------------------------------------------------
        # DASHBOARD PANEL (~30% width, dark theme)
        # ---------------------------------------------------------
        dash_w = max(420, int(w * 3 / 7))
        dash_h = h
        dashboard = np.full((dash_h, dash_w, 3), self.COL_BG, dtype=np.uint8)

        # ---- Safety banner (full width of dashboard column) ----
        banner_h = 34
        is_emergency = "EMERGENCY" in snapshot.safety_banner
        banner_color = self.COL_BAD if is_emergency else (0, 120, 0)
        cv2.rectangle(dashboard, (0, 0), (dash_w, banner_h), banner_color, -1)
        self._text(dashboard, f"{snapshot.safety_banner}  |  LOG:{'ON' if snapshot.is_logging else 'OFF'}",
                    14, 23, (255, 255, 255), 0.6, 2)

        # ---- Proportionally-sized panels filling the rest ----
        margin = 6
        x0, x1 = margin, dash_w - margin
        weights = [
            ("MISSION",     3.0),
            ("VEHICLE",     3.0),
            ("VISION",      2.6),
            ("SERVO",       2.0),
            ("PAYLOAD",     2.0),
            ("HEALTH",      1.6),
            ("DIAGNOSTICS", 1.6),
            ("TIMELINE",    3.2),
            ("MISSION MAP", 5.0),
        ]
        total_weight = sum(wgt for _, wgt in weights)
        available_h = dash_h - banner_h - margin * (len(weights) + 1)
        heights = {name: max(56, int(available_h * wgt / total_weight)) for name, wgt in weights}

        y = banner_h + margin

        # 1. MISSION
        body_y = self._panel(dashboard, x0, x1, y, heights["MISSION"], "MISSION")
        self._text(dashboard, f"STATE:  {snapshot.mission_state}", x0 + 10, body_y, self.COL_GOOD, 0.6, 2)
        self._text(dashboard, f"PHASE:  {snapshot.mission_phase}", x0 + 10, body_y + 22)
        self._text(dashboard, f"T+{snapshot.mission_time:.1f}s", x0 + 10, body_y + 44, self.COL_TEXT_DIM)
        self._text(dashboard, f"INTENT: {snapshot.flight_intent}", x0 + 190, body_y + 44, self.COL_CYAN)
        y += heights["MISSION"] + margin

        # 2. VEHICLE
        body_y = self._panel(dashboard, x0, x1, y, heights["VEHICLE"], "VEHICLE")
        armed_color = self.COL_GOOD if snapshot.is_armed else self.COL_TEXT_DIM
        self._text(dashboard, f"MODE: {snapshot.flight_mode}", x0 + 10, body_y, scale=0.53)
        self._text(dashboard, f"ARMED: {'YES' if snapshot.is_armed else 'NO'}", x1 - 110, body_y, armed_color)
        self._text(dashboard, f"ALT: {snapshot.alt_rel:.2f} m", x0 + 10, body_y + 22, self.COL_CYAN)
        self._text(dashboard, f"YAW: {snapshot.yaw_deg:.1f} deg", x0 + 220, body_y + 22)
        self._text(dashboard, f"NED: {snapshot.ned[0]:.1f}, {snapshot.ned[1]:.1f}, {snapshot.ned[2]:.1f}",
                    x0 + 10, body_y + 44, self.COL_TEXT_DIM, 0.53)
        self._text(dashboard, f"VEL: {snapshot.velocity[0]:.1f}, {snapshot.velocity[1]:.1f}, {snapshot.velocity[2]:.1f}",
                    x0 + 10, body_y + 64, self.COL_TEXT_DIM, 0.53)
        y += heights["VEHICLE"] + margin

        # 3. VISION
        body_y = self._panel(dashboard, x0, x1, y, heights["VISION"], "VISION")
        filter_str = ", ".join(snapshot.active_filter) if snapshot.active_filter else "NONE"
        self._text(dashboard, f"FILTER: {filter_str}", x0 + 10, body_y, self.COL_CYAN, 0.5)
        if snapshot.target_data:
            td = snapshot.target_data
            self._text(dashboard, f"LOCK: {td.get('color','').upper()} {td.get('name','').upper()} "
                                    f"({td.get('confidence',0):.2f})", x0 + 10, body_y + 22, self.COL_GOOD)
        else:
            self._text(dashboard, "LOCK: none", x0 + 10, body_y + 22, self.COL_TEXT_DIM)
        comp_str = ", ".join(snapshot.completed_targets) if snapshot.completed_targets else "NONE"
        self._text(dashboard, f"COMPLETED: {comp_str}", x0 + 10, body_y + 44, self.COL_GOOD, 0.53)
        y += heights["VISION"] + margin

        # 4. SERVO
        body_y = self._panel(dashboard, x0, x1, y, heights["SERVO"], "SERVO")
        self._text(dashboard, f"GATE: {snapshot.descent_gate:.1f} m", x0 + 10, body_y, self.COL_CYAN)
        self._text(dashboard, f"ALIGNED FRAMES: {snapshot.aligned_frames}", x0 + 200, body_y)
        y += heights["SERVO"] + margin

        # 5. PAYLOAD
        body_y = self._panel(dashboard, x0, x1, y, heights["PAYLOAD"], "PAYLOAD")
        self._text(dashboard, f"STATUS: {snapshot.payload_status}", x0 + 10, body_y)
        self._text(dashboard, f"PICKUP REQ: {snapshot.pickup_target or 'UNKNOWN'}", x0 + 10, body_y + 22)
        y += heights["PAYLOAD"] + margin

        # 6. HEALTH
        body_y = self._panel(dashboard, x0, x1, y, heights["HEALTH"], "HEALTH")
        self._text(dashboard, f"DET: {snapshot.detector_fps:.1f}Hz  TRK: {snapshot.tracker_fps:.1f}Hz",
                    x0 + 10, body_y, scale=0.53)
        self._text(dashboard, f"PIPE: {snapshot.pipeline_latency:.1f}ms  UI: {snapshot.ui_latency_ms:.1f}ms",
                    x0 + 10, body_y + 20, scale=0.53)
        y += heights["HEALTH"] + margin

        # 7. DIAGNOSTICS
        body_y = self._panel(dashboard, x0, x1, y, heights["DIAGNOSTICS"], "DIAGNOSTICS")
        self._text(dashboard, f"FRAME: {snapshot.frame_id}  DROPPED: {snapshot.dropped_snapshot_count}",
                    x0 + 10, body_y, self.COL_TEXT_DIM, 0.53)
        y += heights["DIAGNOSTICS"] + margin

        # 8. TIMELINE (uses the UIWorker's accumulated event log, not just the
        # short-lived recent_events on the snapshot, so operators see real history)
        panel_h = heights["TIMELINE"]
        body_y = self._panel(dashboard, x0, x1, y, panel_h, "EVENT TIMELINE")
        source_events = events if events else snapshot.recent_events
        max_lines = max(1, (panel_h - 40) // 18)
        y_off = body_y
        for ev in source_events[-max_lines:]:
            self._text(dashboard, f"[{ev.timestamp % 10000:6.1f}] {ev.name}", x0 + 10, y_off, self.COL_CYAN, 0.46)
            y_off += 18
        y += panel_h + margin

        # 9. MISSION MAP (local NED, fills remaining space)
        # ALT_GZ: auto-fits to every known mission point (Home, vehicle,
        # Drop1, Drop2, plus any Mission-3 TARGET_VERIFIED markers) with a
        # single equal-aspect scale for both axes. The previous version
        # hardcoded a mission-specific -5..+40m / ±15m box tied to this
        # world's exact spawn layout, AND used independent N/E scales that
        # stretched the map out of real-world proportion (a genuine shape
        # drawn on this map would not look like the same shape in reality).
        # Auto-fitting means the map stays correct even if drop/target
        # positions differ from the assumed layout, and a single scale
        # means on-screen distances actually match the real geometry.
        panel_h = heights["MISSION MAP"]
        vn, ve, _ = snapshot.ned
        start_n, start_e = (snapshot.start_ned[0], snapshot.start_ned[1]) if snapshot.start_ned else (0.0, 0.0)
        map_points = [(start_n, start_e), (vn, ve)]
        for (pn, pe) in (path_history or []):
            map_points.append((pn, pe))
        if snapshot.drop1_ned:
            map_points.append((snapshot.drop1_ned[0], snapshot.drop1_ned[1]))
        if snapshot.drop2_ned:
            map_points.append((snapshot.drop2_ned[0], snapshot.drop2_ned[1]))
        for (tn, te) in (target_markers or {}).values():
            map_points.append((tn, te))

        n_vals = [p[0] for p in map_points]
        e_vals = [p[1] for p in map_points]
        pad = 5.0
        n_min, n_max = min(n_vals) - pad, max(n_vals) + pad
        e_min, e_max = min(e_vals) - pad, max(e_vals) + pad
        # Minimum visible span so the map doesn't zoom in absurdly tight
        # right after boot, when only Home and the vehicle (both near the
        # origin) are known.
        MIN_SPAN = 40.0
        if n_max - n_min < MIN_SPAN:
            mid = (n_max + n_min) / 2
            n_min, n_max = mid - MIN_SPAN / 2, mid + MIN_SPAN / 2
        if e_max - e_min < MIN_SPAN:
            mid = (e_max + e_min) / 2
            e_min, e_max = mid - MIN_SPAN / 2, mid + MIN_SPAN / 2

        body_y = self._panel(dashboard, x0, x1, y, panel_h, "MISSION MAP")
        self._text(dashboard, f"N:{n_min:.0f}..{n_max:.0f}m  E:{e_min:.0f}..{e_max:.0f}m",
                   x0 + 10, body_y, self.COL_TEXT_DIM, 0.4)
        map_top, map_bottom = body_y + 8, y + panel_h - 5
        map_left, map_right = x0 + 15, x1 - 15
        scale = min(max(1, map_bottom - map_top) / (n_max - n_min),
                    max(1, map_right - map_left) / (e_max - e_min))
        map_cx = (map_left + map_right) // 2
        map_cy = (map_top + map_bottom) // 2
        n_mid, e_mid = (n_min + n_max) / 2, (e_min + e_max) / 2

        def ned_to_px(n, e):
            return (int(map_cx + (e - e_mid) * scale), int(map_cy - (n - n_mid) * scale))

        # Reference grid, labeled in meters, spaced to roughly quarter the
        # larger visible span (rounded to a clean 5m step).
        grid_step = max(5.0, round((max(n_max - n_min, e_max - e_min) / 4) / 5.0) * 5.0)
        gn = math.ceil(n_min / grid_step) * grid_step
        while gn <= n_max:
            _, py = ned_to_px(gn, e_mid)
            if map_top <= py <= map_bottom:
                cv2.line(dashboard, (map_left, py), (map_right, py), (48, 48, 54), 1)
                self._text(dashboard, f"{gn:.0f}", map_left + 2, max(map_top + 10, py - 2), self.COL_TEXT_DIM, 0.32)
            gn += grid_step
        ge = math.ceil(e_min / grid_step) * grid_step
        while ge <= e_max:
            px, _ = ned_to_px(n_mid, ge)
            if map_left <= px <= map_right:
                cv2.line(dashboard, (px, map_top), (px, map_bottom), (48, 48, 54), 1)
                self._text(dashboard, f"{ge:.0f}", px + 2, map_top + 12, self.COL_TEXT_DIM, 0.32)
            ge += grid_step

        # Permanent flight trajectory -- the actual path flown so far (see
        # UIWorker.path_history), so the map shows where the vehicle has
        # been, not just where it is right now.
        if path_history and len(path_history) >= 2:
            pts = np.array([ned_to_px(pn, pe) for pn, pe in path_history], dtype=np.int32)
            cv2.polylines(dashboard, [pts], False, (150, 90, 60), 1, cv2.LINE_AA)

        # Return-to-start line: only drawn once RETURN_HOME/LANDING has
        # actually begun, from the vehicle's current position straight to
        # the recorded start position -- makes the commanded maneuver visible,
        # not just its endpoint.
        if snapshot.mission_state in ("RETURN_HOME", "LANDING") and snapshot.start_ned:
            cv2.line(dashboard, ned_to_px(vn, ve), ned_to_px(start_n, start_e), (0, 220, 255), 1, cv2.LINE_AA)

        # START -- the recorded mission start position (MissionMemory.start_position),
        # the actual RETURN_HOME/LANDING target. Drawn dim/gray with a "?" until
        # mission.py has actually captured it (see _state_takeoff).
        sx, sy = ned_to_px(start_n, start_e)
        if snapshot.start_ned:
            cv2.drawMarker(dashboard, (sx, sy), (60, 220, 60), cv2.MARKER_TRIANGLE_UP, 16, 2)
            self._text(dashboard, "START", sx + 8, sy - 6, (60, 220, 60), 0.42)
        else:
            cv2.drawMarker(dashboard, (sx, sy), (120, 120, 120), cv2.MARKER_SQUARE, 12, 1)
            self._text(dashboard, "START?", sx + 8, sy - 6, (120, 120, 120), 0.4)

        # Drop1 / Drop2 -- actual recorded drop locations, colored by the
        # real payload color, so the map reflects what actually happened.
        drop_color_bgr = {"red": (0, 0, 255), "blue": (255, 120, 0)}
        for label, dned, dcolor in (("D1", snapshot.drop1_ned, snapshot.drop1_color),
                                     ("D2", snapshot.drop2_ned, snapshot.drop2_color)):
            if not dned:
                continue
            dx, dy = ned_to_px(dned[0], dned[1])
            color = drop_color_bgr.get(dcolor, (200, 200, 200))
            cv2.drawMarker(dashboard, (dx, dy), color, cv2.MARKER_TRIANGLE_UP, 12, 2)
            self._text(dashboard, label, dx + 7, dy + 4, color, 0.4)

        # Permanent target markers (ALT_GZ LIVE MAP FEATURES): populated once
        # per shape via TARGET_VERIFIED events (see UIWorker._on_event) and
        # never cleared, so they stay visible for the rest of the mission.
        marker_style = {
            "blue_hexagon": ((255, 120, 0), "HEXAGON"),   # BGR blue
            "red_triangle": ((0, 0, 255), "TRIANGLE"),    # BGR red
        }
        for shape_key, (tn, te) in (target_markers or {}).items():
            color, label = marker_style.get(shape_key, ((200, 200, 200), shape_key.upper()))
            tx, ty = ned_to_px(tn, te)
            cv2.circle(dashboard, (tx, ty), 7, color, -1, cv2.LINE_AA)
            cv2.circle(dashboard, (tx, ty), 7, (255, 255, 255), 1, cv2.LINE_AA)
            self._text(dashboard, label, tx + 8, ty + 4, color, 0.42)

        # Vehicle + heading
        vx, vy = ned_to_px(vn, ve)
        cv2.circle(dashboard, (vx, vy), 6, self.COL_GOOD, -1, cv2.LINE_AA)
        yaw_rad = math.radians(snapshot.yaw_deg)
        hx_end = int(vx + math.sin(yaw_rad) * 15)
        hy_end = int(vy - math.cos(yaw_rad) * 15)
        cv2.line(dashboard, (vx, vy), (hx_end, hy_end), self.COL_GOOD, 2, cv2.LINE_AA)

        # ---------------------------------------------------------
        # Combine camera (~70%) + dashboard (~30%) into one canvas
        # ---------------------------------------------------------
        combined = np.hstack((frame, dashboard))

        # Upscale before display: the window previously opened at the raw
        # camera-stream resolution (1:1 pixels), which reads as "too small"
        # on most monitors and leaves panel text right at the edge of
        # legibility. INTER_CUBIC upscaling here (rather than just calling
        # resizeWindow on the small image, which the OS would stretch
        # blurrily) actually adds pixel detail before the resize.
        ch, cw = combined.shape[:2]
        disp_w, disp_h = int(cw * self.DISPLAY_SCALE), int(ch * self.DISPLAY_SCALE)
        combined = cv2.resize(combined, (disp_w, disp_h), interpolation=cv2.INTER_CUBIC)

        if not self.window_initialized:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, disp_w, disp_h)
            self.window_initialized = True

        cv2.imshow(self.window_name, combined)

    def close(self):
        cv2.destroyAllWindows()


class UIWorker:
    """Dedicated thread for rendering OpenCV UI to prevent blocking the flight loop."""
    def __init__(self):
        self.running = False
        self.thread = None
        self.snapshot_queue = queue.Queue(maxsize=1)
        self.events = []
        self.dropped_snapshots = 0
        self.last_render_time_ms = 0.0
        self.debug_view = DebugView()
        self.debug_view.window_name = "Mission Dashboard (ALT_GZ)"

        # ALT_GZ LIVE MAP FEATURES: permanent {shape_key: (n, e)} markers,
        # populated from TARGET_VERIFIED events and never cleared.
        self.target_markers = {}

        # Permanent flight trajectory: (n, e) points, appended in update()
        # (called every main-loop tick) at a fixed min-distance spacing so it
        # doesn't grow unbounded over a long mission.
        self.path_history = []
        self._last_path_point = None

        # Subscribe to Event Bus natively
        event_bus.subscribe(self._on_event)

    def _on_event(self, event: Event):
        self.events.append(event)
        if len(self.events) > 50:
            self.events.pop(0)
        if event.name == "TARGET_VERIFIED":
            shape = event.payload.get("shape")
            ned = event.payload.get("ned")
            if shape and ned:
                self.target_markers[shape] = ned

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print("[UI_WORKER] Started Dashboard thread.")

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        self.debug_view.close()
        print("[UI_WORKER] Stopped Dashboard thread.")

    def update(self, snapshot: UISnapshot):
        """Called by main loop at 33Hz. Drops frames if UI is lagging."""
        n, e, _ = snapshot.ned
        if self._last_path_point is None or math.hypot(n - self._last_path_point[0], e - self._last_path_point[1]) >= 0.5:
            self.path_history.append((n, e))
            self._last_path_point = (n, e)
            if len(self.path_history) > 3000:
                self.path_history.pop(0)

        try:
            self.snapshot_queue.put(snapshot, block=False)
        except queue.Full:
            self.dropped_snapshots += 1
            pass # Drop old snapshot to maintain 33Hz flight loop performance

    def _run(self):
        while self.running:
            try:
                # 20Hz refresh rate (50ms timeout)
                snapshot = self.snapshot_queue.get(timeout=0.05)
                t0 = time.time()
                self.debug_view.draw_dashboard(snapshot, self.events, self.target_markers, self.path_history)
                self.last_render_time_ms = (time.time() - t0) * 1000
            except queue.Empty:
                pass

            if cv2.waitKey(1) & 0xFF == ord('q'):
                pass
