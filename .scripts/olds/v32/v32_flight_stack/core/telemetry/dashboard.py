"""
ADR-004 §13 (Mission Operations Center Architecture) / §16 (60-second rule) /
ADR-005 (migration). This REPLACES the old `Dashboard` class -- a
synchronous `cv2.imshow` HUD called directly inside Gorev2Orchestrator's
hot loop (ADR-005 §0/§6: no thread isolation, no backpressure, no crash
isolation, and blocking-to-the-mission by construction).

MissionOpsDashboard runs on its own dedicated thread (ADR-005 §8.1,
restoring UIWorker's proven pattern) and polls two independent, non-blocking
sources at a fixed cadence:
  - RuntimeStateAggregator.snapshot() for structured mission/vehicle/health/
    watchdog/event telemetry (ADR-004 §16's "Blocking Reason Panel").
  - FrameChannel.latest() for the live camera + detection overlay -- the
    vision pipeline runs on this same GCS machine (Görev 2 architecture
    mandate), so this is a local frame handoff, never a call across the
    vehicle's MAVLink/telemetry link, and never a call back into the
    mission runtime.

Nothing it does can block or crash the mission coroutine: every cv2 call is
wrapped, and a render failure degrades to headless (log-only) mode instead
of raising. Layout mirrors V31's DebugView (camera feed + a dark telemetry
column, combined side by side) -- restoring that operator-facing layout,
rebuilt to read from the new MissionSnapshot/FrameChannel sources instead
of the old UISnapshot.
"""
import logging
import sys
import threading
import time
from typing import Optional

import numpy as np

from core.config.parameters import (CENTERING_TOLERANCE_X_NORM, CENTERING_TOLERANCE_Y_NORM,
                                    RELEASED_OVERLAY_DURATION_S)
from core.detection.camera_intrinsics import default_camera_intrinsics
from core.mission.blocking import BlockingKind
from core.mission.phase import TERMINAL_PHASES
from core.telemetry.aggregator import RuntimeStateAggregator
from core.telemetry.paint_bridge import MAIN_THREAD_PAINT
from core.telemetry.events import Severity
from core.telemetry.frame_channel import FrameChannel, FrameSample
from core.telemetry.snapshot import MissionSnapshot

logger = logging.getLogger("telemetry.dashboard")

try:
    import cv2
    _CV2_AVAILABLE = True
except Exception:  # noqa: BLE001 -- headless environments without an OpenCV GUI build
    _CV2_AVAILABLE = False

FONT = cv2.FONT_HERSHEY_SIMPLEX if _CV2_AVAILABLE else None

COL_BG = (22, 22, 24)
COL_PANEL_BG = (34, 34, 38)
COL_PANEL_BORDER = (58, 58, 64)
COL_HEADER_BG = (48, 48, 54)
COL_ACCENT = (235, 178, 60)
COL_TEXT = (232, 232, 232)
COL_TEXT_DIM = (150, 150, 156)
COL_GOOD = (110, 220, 120)
COL_WARN = (60, 210, 235)
COL_BAD = (70, 70, 235)
COL_CYAN = (235, 210, 70)
# Lock indicator (operator request, 2026-08-16). BGR: the target->crosshair
# vector is yellow while a pursuit is centring and black once the
# CenteringController reports converged. Black reads as "settled/locked"
# against this dashboard's dark-but-never-black panels and against the
# camera feed's own bright arena colours; every other overlay element stays
# a saturated hue, so it is unambiguous at a glance.
COL_VECTOR = (0, 255, 255)
COL_LOCKED = (0, 0, 0)
# ADR-010 P5: every detected shape is stroked in the same green, whether it
# came from a contour or (adapter fallback) a bounding box. Per-shape colours
# are gone with the rectangles -- the shape's own outline already says which
# shape it is, and the class label says it in words.
COL_CONTOUR = (0, 255, 0)


def _contour_points(detection):
    """The detection's polygon as an int32 (N,2) array for cv2.polylines, or
    None when the detector supplied no contour (see the adapter contract in
    core/detection/types.py). Malformed or degenerate polygons return None
    so the caller falls back to the bbox rather than raising inside a paint
    loop -- the display must degrade, never crash."""
    pts = getattr(detection, "contour_px", None)
    if not pts or len(pts) < 3:
        return None
    try:
        return np.array([[int(round(x)), int(round(y))] for x, y in pts], dtype=np.int32)
    except (TypeError, ValueError):
        return None

_SEVERITY_COLOR = {
    Severity.DEBUG.value: COL_TEXT_DIM,
    Severity.INFO.value: COL_TEXT,
    Severity.WARN.value: COL_WARN,
    Severity.CRITICAL.value: COL_BAD,
    Severity.FATAL.value: COL_BAD,
}
_HEALTH_COLOR = {
    "HEALTHY": COL_GOOD, "DEGRADED": COL_WARN, "STALE": COL_WARN,
    "DOWN": COL_BAD, "UNKNOWN": COL_TEXT_DIM,
}
_SHAPE_COLOR = {
    "MAVI_ALTIGEN": (255, 120, 0), "MAVI_DIKDORTGEN": (255, 120, 0),
    "KIRMIZI_UCGEN": (0, 0, 255), "KIRMIZI_DIKDORTGEN": (0, 0, 255),
}

CAMERA_PLACEHOLDER_SIZE = (640, 480)  # (w, h), used until the first frame arrives
TELEMETRY_COLUMN_WIDTH = 460
DISPLAY_SCALE = 1.2


class MissionOpsDashboard:
    def __init__(
        self,
        aggregator: RuntimeStateAggregator,
        frame_channel: Optional[FrameChannel] = None,
        mission_id: str = "",
        window_name: str = "Mission Operations Center",
        refresh_hz: float = 10.0,
        telemetry_col_width: int = TELEMETRY_COLUMN_WIDTH,
    ):
        self.aggregator = aggregator
        self.frame_channel = frame_channel
        self.mission_id = mission_id
        self.window_name = window_name
        self.refresh_interval_s = 1.0 / max(1.0, refresh_hz)
        self.telemetry_col_width = telemetry_col_width
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._window_initialized = False
        self._headless = not _CV2_AVAILABLE

        # macOS: compose here, paint on the main thread (ADR-006, implemented).
        #
        # Cocoa requires every cv2 GUI call on the process MAIN thread. ADR-005
        # §3 requires this dashboard's state/composition/lifecycle to stay on
        # its own dedicated thread, and its §8 table forbids "a direct cv2 call
        # on this thread" for the MISSION thread. main_gz.py now runs the
        # mission coroutine on a WORKER thread, so the main thread is free and
        # is no longer the mission thread -- painting there satisfies Cocoa
        # without putting cv2 anywhere near the mission. Everything this class
        # does is unchanged; only the final imshow/waitKey is delegated, via a
        # single-slot drop-oldest bridge so the dashboard never blocks on the
        # painter.
        #
        # Linux/Windows are untouched: no bridge, paint on this thread as before.
        self._delegate_paint = (not self._headless) and sys.platform == "darwin"
        if self._delegate_paint:
            MAIN_THREAD_PAINT.enable()
            logger.info(
                "[DASHBOARD] macOS: composing on dashboard thread, painting on main thread "
                "(ADR-006); mission runs off the main thread"
            )

    # ------------------------------------------------------------------
    def start(self) -> None:
        """Auto-launched by main_*.py immediately on mission start -- no
        operator action, per ADR-004 §13's auto-open requirement."""
        self._running = True
        self._thread = threading.Thread(target=self._run, name="MissionOpsDashboard", daemon=True)
        self._thread.start()
        logger.info("Mission Operations Center dashboard thread started.")

    def stop(self, timeout_s: float = 2.0) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
        logger.info("Mission Operations Center dashboard thread stopped.")

    # ------------------------------------------------------------------
    def _run(self) -> None:
        while self._running:
            t0 = time.time()
            try:
                snap = self.aggregator.snapshot()
                sample = self.frame_channel.latest() if self.frame_channel else None
                self._render(snap, sample)
            except Exception as e:  # noqa: BLE001 -- this thread must never take the mission down
                logger.error("Dashboard render failed, continuing headless: %s", e)
                self._headless = True
            sleep_s = self.refresh_interval_s - (time.time() - t0)
            if sleep_s > 0:
                time.sleep(sleep_s)
        # Window teardown must happen on the same thread that created it
        # (highgui/Qt has thread affinity -- calling this from stop()'s
        # caller thread instead produces a benign but noisy Qt warning).
        if not self._headless and self._window_initialized:
            try:
                cv2.destroyAllWindows()
            except Exception:  # noqa: BLE001
                pass

    def _render(self, snap: MissionSnapshot, sample: Optional[FrameSample]) -> None:
        if self._headless:
            return

        camera_img = self._build_camera_panel(sample, snap)
        telemetry_img = self._build_telemetry_column(snap, height=camera_img.shape[0])
        combined = np.hstack((camera_img, telemetry_img))

        if DISPLAY_SCALE != 1.0:
            h, w = combined.shape[:2]
            combined = cv2.resize(combined, (int(w * DISPLAY_SCALE), int(h * DISPLAY_SCALE)),
                                   interpolation=cv2.INTER_LINEAR)

        # macOS (ADR-006): composition is done -- hand the finished image to
        # the main thread and return. Non-blocking by construction; if the
        # painter is behind, the previous frame is simply dropped.
        if self._delegate_paint:
            MAIN_THREAD_PAINT.publish(self.window_name, combined)
            return

        try:
            if not self._window_initialized:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, combined.shape[1], combined.shape[0])
                self._window_initialized = True
            cv2.imshow(self.window_name, combined)
            cv2.waitKey(1)
        except Exception as e:  # noqa: BLE001 -- no $DISPLAY / Qt plugin missing etc.
            # ERROR, not WARNING: this is an unexpected display failure (the
            # one EXPECTED case, macOS/Cocoa, is handled up front in
            # __init__). Swallowing it at WARNING is how a dead camera panel
            # previously looked identical to a working one. Include the
            # exception text and thread name so it is attributable.
            logger.error(
                "cv2 display unavailable on thread %r, switching dashboard to headless mode: %s: %s",
                threading.current_thread().name, type(e).__name__, e,
                exc_info=True,
            )
            self._headless = True

    # ------------------------------------------------------------------
    # camera + detection overlay panel (V31-style)
    # ------------------------------------------------------------------
    def _build_camera_panel(self, sample: Optional[FrameSample], snap: MissionSnapshot) -> np.ndarray:
        if sample is None or sample.frame_bgr is None or sample.frame_bgr.size == 0:
            w, h = CAMERA_PLACEHOLDER_SIZE
            img = np.full((h, w, 3), COL_BG, dtype=np.uint8)
            self._text(img, "WAITING FOR CAMERA FEED...", w // 2 - 150, h // 2, COL_TEXT_DIM, 0.6, 1)
            self._draw_flight_mode_badge(img, snap)
            return img

        frame = sample.frame_bgr.copy()
        h, w = frame.shape[:2]

        # Operator request (2026-08-16): the crosshair must sit at the EXACT
        # image centre the controller centres on -- CenteringController uses
        # res_w/2.0, res_h/2.0 (a float), not an integer floor, and it
        # reports that centre in CENTERING_STEP. Prefer the reported value
        # so the two can never disagree; fall back to this frame's own
        # centre before the first step arrives.
        centre = snap.centering.center_px or (w / 2.0, h / 2.0)
        cx, cy = int(round(centre[0])), int(round(centre[1]))

        cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 1, cv2.LINE_AA)

        # Convergence tolerance, drawn as the TRUE ellipse (operator
        # request, 2026-08-17 -- explicitly not a circle approximation).
        # The tolerance is +/-0.01 NORMALIZED PER AXIS, and each axis
        # normalizes by its own half-extent, so the region is an ellipse
        # with semi-axes tol_x*(w/2) and tol_y*(h/2) -- 6.4 x 4.8 px at
        # 1280x960. Derived from the frame's own size, so it scales with
        # whatever resolution the camera delivers.
        semi_x = max(1, int(round(CENTERING_TOLERANCE_X_NORM * (w / 2.0))))
        semi_y = max(1, int(round(CENTERING_TOLERANCE_Y_NORM * (h / 2.0))))
        cv2.ellipse(frame, (cx, cy), (semi_x, semi_y), 0, 0, 360, COL_ACCENT, 1, cv2.LINE_AA)

        intrinsics = default_camera_intrinsics()
        agl_m = snap.vehicle.position[2] if snap.vehicle.position else None

        for d in sample.detections:
            # ADR-010 P5: stroke the shape's OWN outline -- a triangle draws
            # as a triangle, a hexagon as a hexagon. Bounding rectangles are
            # gone entirely: they said nothing the contour does not say, and
            # at low altitude a frame-filling box is pure noise. The polygon
            # is the detector's own approxPolyDP result (the one its
            # vertex-count gate accepted), so what is drawn is literally
            # what was detected rather than a redrawing of it.
            color = COL_CONTOUR
            contour = _contour_points(d)
            if contour is not None:
                cv2.polylines(frame, [contour], True, color, 2, cv2.LINE_AA)
                label_anchor = tuple(contour[contour[:, 1].argmin()])
            else:
                # Adapter fallback (see core/detection/types.py): a detector
                # that supplies no polygon gets its bbox stroked in the SAME
                # green -- the overlay never implies "this one is different"
                # when the only difference is what the detector reported.
                x1, y1, x2, y2 = d.bbox_px
                if x2 > x1 and y2 > y1:
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2, cv2.LINE_AA)
                label_anchor = (int(x1), int(y1))
            tx, ty = int(d.center_px[0]), int(d.center_px[1])
            cv2.circle(frame, (tx, ty), 5, color, -1, cv2.LINE_AA)

            # Lock indicator: YELLOW while this target is being centred,
            # BLACK once the controller itself reports convergence. The flag
            # is taken verbatim from CENTERING_STEP -- the overlay never
            # re-decides "locked" against a threshold of its own, so losing
            # lock (any later step with converged=False, or a timeout)
            # reverts it to yellow automatically.
            locked = snap.centering.converged and snap.centering.shape_type == d.shape_type
            vector_color = COL_LOCKED if locked else COL_VECTOR
            cv2.line(frame, (cx, cy), (tx, ty), vector_color, 2 if locked else 1, cv2.LINE_AA)

            # ADR-010 P5: label sits just above the TOPMOST contour vertex,
            # so it tracks the shape's actual outline rather than a corner
            # of a box that no longer exists.
            label = f"{d.shape_type} {d.confidence:.2f}"
            cv2.putText(frame, label, (int(label_anchor[0]), max(12, int(label_anchor[1]) - 8)),
                        FONT, 0.55, color, 2, cv2.LINE_AA)

            self._draw_ground_distance(frame, d, snap, intrinsics, agl_m,
                                       (cx, cy), (tx, ty), w, h)

            # W4.4: for RELEASED_OVERLAY_DURATION_S after the servo fires,
            # tag the target that was just dropped on. Anchored to the
            # contour, not the frame, so it reads as "this shape" rather than
            # a global banner -- and it is time-boxed so it can never be
            # mistaken for a live state.
            rel_at = snap.payload.released_at
            if rel_at and snap.payload.released_shape == d.shape_type \
                    and (time.time() - rel_at) <= RELEASED_OVERLAY_DURATION_S:
                rx, ry = int(label_anchor[0]), max(28, int(label_anchor[1]) - 28)
                cv2.putText(frame, "RELEASED", (rx, ry), FONT, 0.6, COL_GOOD, 2, cv2.LINE_AA)

        # Slim glance-strip across the top of the feed, mirrors V31's overlay.
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 26), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)
        age_s = time.time() - sample.ts
        stale_note = "  [FRAME STALE]" if age_s > 1.0 else ""
        # ADR-008 B1: a stale DETECTION feed is reported separately from a
        # stale frame, and loudly -- these are different failures with
        # opposite meanings (frozen video vs. live video with a dead
        # detector), and conflating them is how the 2026-08-16 run read as
        # healthy on screen for 82 seconds while vision was DOWN. The
        # producer already withholds the boxes themselves; this says why the
        # overlay went empty instead of leaving it looking like "no targets
        # in view".
        if sample.detections_stale:
            age = f" {sample.detections_age_s:.1f}s" if sample.detections_age_s is not None else ""
            self._text(frame, f"VISION FEED STALE{age} -- detections not drawn",
                       8, 18, COL_BAD, 0.5, 1)
        else:
            self._text(frame, f"{len(sample.detections)} detection(s){stale_note}", 8, 18, COL_TEXT, 0.5, 1)

        self._draw_flight_mode_badge(frame, snap)
        return frame

    def _draw_ground_distance(self, frame, detection, snap: MissionSnapshot, intrinsics,
                              agl_m, centre, target, w: int, h: int) -> None:
        """`<TARGET> d=X.X m` at the lower-right of the target->crosshair
        vector (operator request, 2026-08-16).

        X is the horizontal ground distance from the vehicle's nadir point
        to the target, from pixel offset + current AGL + the camera's own
        FOV (parsed from the mono_cam SDF -- see
        core/detection/camera_intrinsics.py; nothing here hardcodes a lens).

        For the target currently being centred, the value published in
        CENTERING_STEP is reused verbatim so the label and the log agree
        exactly; other detections are computed here with the same
        intrinsics. If neither the intrinsics nor a usable AGL are
        available, the label is omitted rather than showing a fabricated
        number."""
        cx, cy = centre
        tx, ty = target

        distance_m = None
        if snap.centering.shape_type == detection.shape_type and snap.centering.ground_distance_m is not None:
            distance_m = snap.centering.ground_distance_m
        elif intrinsics is not None:
            distance_m = intrinsics.scaled_to(w, h).ground_distance_m(tx - cx, ty - cy, agl_m)

        if distance_m is None:
            return

        text = f"{detection.shape_type} d={distance_m:.1f} m"
        (tw, th), _ = cv2.getTextSize(text, FONT, 0.5, 1)

        # Lower-right of the vector: anchor to the lower-right END of the
        # line's bounding box, then clamp so it stays inside the frame when
        # the target is near an edge.
        anchor_x = max(cx, tx) + 8
        anchor_y = max(cy, ty) + th + 10
        x = min(anchor_x, w - tw - 4)
        y = min(anchor_y, h - 4)

        # Plate + light text regardless of the vector colour: when the lock
        # indicator turns the vector black, drawing the label in the same
        # colour would make it vanish against its own backing plate.
        cv2.rectangle(frame, (x - 4, y - th - 4), (x + tw + 4, y + 4), (0, 0, 0), -1)
        cv2.putText(frame, text, (x, y), FONT, 0.5, COL_TEXT, 1, cv2.LINE_AA)

    def _draw_flight_mode_badge(self, img: np.ndarray, snap: MissionSnapshot) -> None:
        """Onboard (PX4 flying its own Mission plan) vs Offboard (this
        codebase actively commanding it toward a target) is the single most
        useful at-a-glance signal for "what is actually flying the vehicle
        right now" -- drawn directly on the camera feed, not buried in the
        telemetry column, per the operator's own framing of what matters
        here."""
        w = img.shape[1]
        mode = snap.vehicle.flight_mode or "UNKNOWN"
        is_offboard = mode == "OFFBOARD"
        badge_text = f"OFFBOARD  ({mode})" if is_offboard else f"ONBOARD  ({mode})"
        badge_color = (0, 200, 255) if is_offboard else (120, 220, 120)  # BGR: amber vs green
        badge_w = 250
        x0, y0 = w - badge_w - 10, 10
        cv2.rectangle(img, (x0, y0), (x0 + badge_w, y0 + 30), (20, 20, 20), -1)
        cv2.rectangle(img, (x0, y0), (x0 + badge_w, y0 + 30), badge_color, 2)
        self._text(img, badge_text, x0 + 10, y0 + 21, badge_color, 0.55, 2)

    # ------------------------------------------------------------------
    # telemetry column (right side)
    # ------------------------------------------------------------------
    def _build_telemetry_column(self, snap: MissionSnapshot, height: int) -> np.ndarray:
        img = np.full((height, self.telemetry_col_width, 3), COL_BG, dtype=np.uint8)
        y = self._draw_header(img, snap, 0)
        y = self._draw_blocking(img, snap, y)
        y = self._draw_interlock(img, snap, y)
        y = self._draw_payload(img, snap, y)
        y = self._draw_health_and_watchdogs(img, snap, y)
        self._draw_timeline(img, snap, y, height)
        return img

    # ------------------------------------------------------------------
    # panel primitives
    # ------------------------------------------------------------------
    def _text(self, img, text, x, y, color=COL_TEXT, scale=0.5, thick=1):
        cv2.putText(img, text, (x, y), FONT, scale, color, thick, cv2.LINE_AA)

    def _panel(self, img, x0, x1, y, height, title, accent=COL_ACCENT):
        cv2.rectangle(img, (x0, y), (x1, y + height), COL_PANEL_BG, -1)
        cv2.rectangle(img, (x0, y), (x1, y + height), COL_PANEL_BORDER, 1)
        cv2.rectangle(img, (x0, y), (x1, y + 22), COL_HEADER_BG, -1)
        self._text(img, title, x0 + 10, y + 16, accent, 0.5, 1)
        return y + 36

    def _draw_header(self, img, snap: MissionSnapshot, y: int) -> int:
        w = img.shape[1]
        h = 46
        is_terminal_failure = snap.phase in TERMINAL_PHASES and snap.phase.value != "MISSION_COMPLETE"
        banner_color = COL_BAD if is_terminal_failure else (0, 120, 0) if snap.phase.value == "MISSION_COMPLETE" else COL_HEADER_BG
        cv2.rectangle(img, (0, y), (w, y + h), banner_color, -1)
        self._text(img, snap.mission_id or "(no mission id)", 10, y + 17, COL_TEXT_DIM, 0.42)

        # QGC connection status (operator-requested, ADR: MAVSDK exposes no
        # API for this -- heuristic UDP-port-bound check, see qgc_monitor.py).
        if snap.qgc_connected is True:
            qgc_text, qgc_color = "QGC: CONNECTED", COL_GOOD
        elif snap.qgc_connected is False:
            qgc_text, qgc_color = "QGC: NOT DETECTED", COL_WARN
        else:
            qgc_text, qgc_color = "QGC: UNKNOWN", COL_TEXT_DIM
        (tw, _), _ = cv2.getTextSize(qgc_text, FONT, 0.42, 1)
        self._text(img, qgc_text, w - tw - 10, y + 17, qgc_color, 0.42, 1)

        self._text(img, f"{snap.phase.value}", 10, y + 36, COL_TEXT, 0.5, 1)
        timeout_txt = f"T+{snap.elapsed_s:.0f}s"
        if snap.timeout_remaining_s is not None:
            timeout_txt += f" / timeout {snap.timeout_remaining_s:.0f}s"
        self._text(img, timeout_txt, w - 190, y + 36, COL_TEXT_DIM, 0.42)
        return y + h + 4

    def _draw_blocking(self, img, snap: MissionSnapshot, y: int) -> int:
        w = img.shape[1]
        h = 56
        body_y = self._panel(img, 6, w - 6, y, h, "BLOCKING REASON")
        if snap.blocking is None:
            self._text(img, "No active block.", 16, body_y, COL_GOOD, 0.46)
        else:
            b = snap.blocking
            kind_color = COL_WARN if b.kind == BlockingKind.EXPECTED_WAIT else COL_BAD
            remaining = b.remaining_s()
            remaining_txt = f"  {remaining:.0f}s to timeout" if remaining is not None else "  no timeout"
            self._text(img, f"[{b.kind.value}]", 16, body_y, kind_color, 0.42)
            self._text(img, f"{b.waiting_on}", 16, body_y + 16, COL_TEXT, 0.44)
            self._text(img, f"owner={b.owning_subsystem}  {b.elapsed_s():.0f}s{remaining_txt}",
                      16, body_y + 32, COL_TEXT_DIM, 0.36)
        return y + h + 4

    def _draw_interlock(self, img, snap: MissionSnapshot, y: int) -> int:
        w = img.shape[1]
        h = 30
        body_y = self._panel(img, 6, w - 6, y, h, "PAYLOAD INTERLOCK")
        p1_color = COL_GOOD if snap.payload.payload_1_released else COL_TEXT_DIM
        p2_color = COL_GOOD if snap.payload.payload_2_released else COL_TEXT_DIM
        self._text(img, f"P1 MAVI: {'OK' if snap.payload.payload_1_released else '--'}", 16, body_y, p1_color, 0.44)
        self._text(img, f"P2 KIRMIZI: {'OK' if snap.payload.payload_2_released else '--'}",
                  self.telemetry_col_width // 2 + 10, body_y, p2_color, 0.44)
        return y + h + 4

    def _draw_payload(self, img, snap: MissionSnapshot, y: int) -> int:
        """ADR-010 P2: what the active drop is doing, right now.

        Deliberately shows the release altitude AND whether it landed in the
        commanded band: V1''' dropped payload 1 at 1.587 m against a
        commanded 0.45 m and nothing on screen said so. The band check is the
        producer's (PAYLOAD_STATE.within_tolerance), not re-derived here --
        the same rule the lock indicator follows."""
        w = img.shape[1]
        h = 78
        p = snap.payload
        body_y = self._panel(img, 6, w - 6, y, h, "PAYLOAD")

        if not p.active_index:
            self._text(img, "no drop in progress", 16, body_y, COL_TEXT_DIM, 0.42)
            return y + h + 4

        released = p.released_alt_m is not None
        head_color = COL_GOOD if released else COL_ACCENT
        self._text(img, f"PAYLOAD {p.active_index}  {p.active_shape}", 16, body_y, head_color, 0.46)

        alt_txt = f"{p.current_alt_m:.2f}" if p.current_alt_m is not None else "--"
        tgt_txt = f"{p.target_alt_m:.2f}" if p.target_alt_m is not None else "--"
        self._text(img, f"alt {alt_txt} -> {tgt_txt} m   step {p.descent_step or '--'}",
                   16, body_y + 17, COL_TEXT, 0.4)

        # "vision committed" is the detector's commit, not "is the camera
        # working" -- below LOW_ALT_VISION_LIMIT_M a False here is EXPECTED
        # and is what hands the descent to the open-loop path (ADR-010 P1),
        # so it is drawn dim rather than red.
        vis = p.vision_committed
        vis_txt = "yes" if vis else ("no" if vis is not None else "--")
        vis_color = COL_GOOD if vis else COL_TEXT_DIM
        off_txt = f"{p.last_offset_cm:.1f} cm" if p.last_offset_cm is not None else "--"
        self._text(img, f"vision {vis_txt}", 16, body_y + 33, vis_color, 0.4)
        self._text(img, f"last offset {off_txt}", self.telemetry_col_width // 2 + 10,
                   body_y + 33, COL_TEXT, 0.4)

        if released:
            # W4: green, with the wall-clock time of the servo, and it stays
            # up for the rest of the flight -- disarm is exactly when an
            # operator goes looking for "did it actually drop, and where".
            ok = p.released_within_tolerance
            rel_color = COL_GOOD if ok else COL_WARN
            band = "" if ok else "  OUT OF BAND"
            stamp = time.strftime("%H:%M:%S", time.localtime(p.released_at)) if p.released_at else "--:--:--"
            self._text(img, f"RELEASED at {p.released_alt_m:.2f} m{band}",
                       16, body_y + 51, rel_color, 0.44)
            self._text(img, stamp, self.telemetry_col_width - 78, body_y + 51, rel_color, 0.42)
        return y + h + 4

    def _draw_health_and_watchdogs(self, img, snap: MissionSnapshot, y: int) -> int:
        w = img.shape[1]
        h = 96
        body_y = self._panel(img, 6, w - 6, y, h, "HEALTH")
        entries = sorted(snap.health.items(), key=lambda kv: kv[1].state == "HEALTHY")
        yy = body_y
        for name, entry in entries[:3]:
            color = _HEALTH_COLOR.get(entry.state, COL_TEXT_DIM)
            self._text(img, f"{name}: {entry.state}", 16, yy, color, 0.4)
            yy += 15
        if not entries:
            self._text(img, "no registered subsystems yet", 16, yy, COL_TEXT_DIM, 0.4)
        yy += 6
        for name, wd in list(snap.watchdogs.items())[:2]:
            remaining = f"{wd.remaining_s:.0f}s" if wd.remaining_s is not None else "-"
            color = COL_WARN if (wd.remaining_s is not None and wd.remaining_s < (wd.threshold_s or 1) * 0.2) else COL_TEXT_DIM
            self._text(img, f"WD {name}: {remaining}", 16, yy, color, 0.4)
            yy += 15
        return y + h + 4

    def _draw_timeline(self, img, snap: MissionSnapshot, y: int, total_height: int) -> None:
        w = img.shape[1]
        h = max(60, total_height - y - 6)
        body_y = self._panel(img, 6, w - 6, y, h, "EVENT TIMELINE")
        max_lines = max(1, (h - 30) // 15)
        yy = body_y
        for ev in snap.recent_events[-max_lines:]:
            color = _SEVERITY_COLOR.get(ev.severity.value if hasattr(ev.severity, "value") else ev.severity, COL_TEXT)
            t_rel = ev.ts - snap.started_at
            self._text(img, f"[{t_rel:6.1f}s] {ev.code}", 14, yy, color, 0.36)
            yy += 15
