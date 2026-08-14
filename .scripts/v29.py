#!/usr/bin/env python3
# v30.py
# Gazebo cam + OpenCV + MAVSDK Offboard
# FIXES:
# - Single MAVSDK connection (no multiple System() chaos)
# - HEX priority always
# - TRI mark only before HEX; after HEX -> goto saved NE -> TRI drop
# - 20m hold 5s (countdown) -> 8m hold 5s (countdown) -> 2m hold 3s -> release
# - Vision loss tolerance = 5s (keep last error)
# - NO CLIMB during MID_RECENTER / HOLD / DESCENT (prevents 8m bounce)
# - Strong mission resume with flight_mode verification

import os
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import cv2
import numpy as np
import threading
import time
import argparse
import math
import asyncio
from typing import Optional, Tuple, Dict

# ============== Gazebo Transport (Harmonic) ==============
from gz.transport13 import Node
try:
    from gz.msgs.image_pb2 import Image as ImageMsg
except Exception:
    from gz.msgs10.image_pb2 import Image as ImageMsg
# =========================================================

# ============== MAVSDK / PX4 =============================
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, PositionNedYaw
from mavsdk.telemetry import FlightMode
# =========================================================

latest_frame: Optional[np.ndarray] = None
frame_lock = threading.Lock()


# =========================
# Gazebo Image Reader
# =========================
class GzCameraReader:
    def __init__(self, topic: str):
        self.topic = topic
        self.node = Node()

    def _cb(self, msg: ImageMsg):
        global latest_frame
        try:
            if not msg.data or msg.width == 0 or msg.height == 0:
                return
            W, H = msg.width, msg.height
            step = msg.step if getattr(msg, "step", 0) else W * 3
            rowb = W * 3
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if buf.size < H * step:
                return
            img_step = buf.reshape((H, step))
            rgb = img_step[:, :rowb].reshape((H, W, 3))
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            with frame_lock:
                latest_frame = bgr
        except Exception:
            pass

    def start(self):
        if not self.node.subscribe(topic=self.topic, callback=self._cb, msg_type=ImageMsg):
            raise RuntimeError(f"Subscribe failed: {self.topic}")

    def stop(self):
        pass


# =======================
# Detection: only RED TRI + BLUE HEX
# (ignore blue squares by never classifying square)
# =======================
class Detector:
    def __init__(self):
        self.RED_LO_1 = (0, 40, 40);   self.RED_HI_1 = (15, 255, 255)
        self.RED_LO_2 = (165, 40, 40); self.RED_HI_2 = (180, 255, 255)
        self.BLUE_LO  = (90, 80, 40);  self.BLUE_HI  = (140, 255, 255)

        self.MIN_AREA_TRI_BASE = 390
        self.MIN_AREA_HEX_BASE = 800

        self.EPS_TRI_MIN = 0.03
        self.EPS_TRI_MAX = 0.09
        self.EPS_HEX     = 0.026

        self.COLOR_FRAC_TRI_BASE = 0.35
        self.COLOR_FRAC_HEX      = 0.45

        self.STREAK_FRAMES = 3
        self.STREAK_DIST_PX = 60

        # per-shape streak
        self._last_center = {"triangle": None, "hexagon": None}
        self._streak = {"triangle": 0, "hexagon": 0}

        self.ignore_hexagon = False
        self.ignore_triangle = False

        self.best_tri = None
        self.best_hex = None
        self.largest_hex_contour = None

    def _preprocess(self, frame_bgr):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = cv2.createCLAHE(2.0, (8, 8)).apply(v)
        return cv2.merge([h, s, v])

    def _color_masks(self, hsv):
        red1 = cv2.inRange(hsv, self.RED_LO_1, self.RED_HI_1)
        red2 = cv2.inRange(hsv, self.RED_LO_2, self.RED_HI_2)
        red = cv2.bitwise_or(red1, red2)
        blue = cv2.inRange(hsv, self.BLUE_LO, self.BLUE_HI)

        k = np.ones((5, 5), np.uint8)
        red = cv2.morphologyEx(red, cv2.MORPH_OPEN, k, 1)
        red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, k, 2)
        blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, k, 1)
        blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, k, 2)
        return red, blue

    @staticmethod
    def _center_of(poly):
        M = cv2.moments(poly)
        if M["m00"] != 0:
            return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
        x, y, w, h = cv2.boundingRect(poly)
        return (x + w//2, y + h//2)

    @staticmethod
    def _min_side_len(poly):
        pts = [poly[i][0] for i in range(len(poly))]
        lens = [np.linalg.norm(pts[i]-pts[(i+1) % len(pts)]) for i in range(len(pts))]
        return min(lens) if lens else 0.0

    def _update_streak(self, name: str, center: Tuple[int, int]) -> bool:
        last = self._last_center[name]
        if last is not None:
            if np.hypot(center[0] - last[0], center[1] - last[1]) <= self.STREAK_DIST_PX:
                self._streak[name] += 1
            else:
                self._streak[name] = 1
        else:
            self._streak[name] = 1
        self._last_center[name] = center
        return self._streak[name] >= self.STREAK_FRAMES

    def detect(self, frame_bgr: np.ndarray):
        self.best_tri = None
        self.best_hex = None
        self.largest_hex_contour = None

        blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
        hsv = self._preprocess(blurred)
        red_mask, blue_mask = self._color_masks(hsv)

        h, w = frame_bgr.shape[:2]
        area_scale = (w * h) / float(1280 * 960)

        best_tri = None
        best_tri_score = -1.0
        if not self.ignore_triangle:
            contours_r, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours_r:
                if cv2.contourArea(cnt) < (self.MIN_AREA_TRI_BASE * area_scale):
                    continue
                peri = cv2.arcLength(cnt, True)
                for eps in np.linspace(self.EPS_TRI_MIN, self.EPS_TRI_MAX, 6):
                    approx = cv2.approxPolyDP(cnt, eps * peri, True)
                    if len(approx) != 3 or not cv2.isContourConvex(approx):
                        continue
                    if self._min_side_len(approx) < 8:
                        continue
                    tri_area = abs(cv2.contourArea(approx))
                    if tri_area <= 0:
                        continue

                    poly_mask = np.zeros(red_mask.shape, dtype=np.uint8)
                    cv2.fillPoly(poly_mask, [approx], 255)
                    colored = cv2.countNonZero(cv2.bitwise_and(red_mask, red_mask, mask=poly_mask))
                    area_px = cv2.countNonZero(poly_mask)
                    frac = 0.0 if area_px == 0 else colored / float(area_px)
                    frac_thr = max(0.20, self.COLOR_FRAC_TRI_BASE - 0.10*np.log10(max(1.0, tri_area/1500.0)))
                    if frac < frac_thr:
                        continue

                    c = self._center_of(approx)
                    score = tri_area * (0.7 * frac + 0.3 * 0.5)
                    if score > best_tri_score:
                        best_tri_score = score
                        best_tri = {"name": "triangle", "color": "red", "center": c, "contour": approx, "score": score}

        best_hex = None
        best_hex_score = -1.0
        if not self.ignore_hexagon:
            contours_b, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours_b:
                if cv2.contourArea(cnt) < (self.MIN_AREA_HEX_BASE * area_scale):
                    continue
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, self.EPS_HEX * peri, True)
                if len(approx) != 6 or not cv2.isContourConvex(approx):
                    continue
                if self._min_side_len(approx) < 8:
                    continue

                poly_mask = np.zeros(blue_mask.shape, dtype=np.uint8)
                cv2.fillPoly(poly_mask, [approx], 255)
                colored = cv2.countNonZero(cv2.bitwise_and(blue_mask, blue_mask, mask=poly_mask))
                area_px = cv2.countNonZero(poly_mask)
                frac = 0.0 if area_px == 0 else colored / float(area_px)
                if frac < self.COLOR_FRAC_HEX:
                    continue

                c = self._center_of(approx)
                area = cv2.contourArea(approx)
                score = area * frac
                if score > best_hex_score:
                    best_hex_score = score
                    best_hex = {"name": "hexagon", "color": "blue", "center": c, "contour": approx, "score": score}

        tri_committed = None
        hex_committed = None
        if best_tri and self._update_streak("triangle", best_tri["center"]):
            tri_committed = best_tri
        if best_hex and self._update_streak("hexagon", best_hex["center"]):
            hex_committed = best_hex
            self.largest_hex_contour = best_hex["contour"]

        self.best_tri = tri_committed
        self.best_hex = hex_committed
        return tri_committed, hex_committed

    def draw(self, frame, fps=None):
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.drawMarker(frame, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 16, 2)

        if self.best_hex:
            cv2.drawContours(frame, [self.best_hex["contour"]], -1, (0, 255, 0), 2)
            x, y, ww, hh = cv2.boundingRect(self.best_hex["contour"])
            cv2.putText(frame, "blue hexagon", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        if self.best_tri:
            cv2.drawContours(frame, [self.best_tri["contour"]], -1, (0, 255, 0), 2)
            x, y, ww, hh = cv2.boundingRect(self.best_tri["contour"])
            cv2.putText(frame, "red triangle", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if fps is not None:
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, time.strftime("%H:%M:%S"), (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


# =========================
# Bus: vision -> ctrl (per-shape)
# =========================
class TargetBus:
    def __init__(self):
        self.lock = threading.Lock()
        self.hex = {"seen": False, "dx": 0.0, "dy": 0.0, "ts": 0.0}
        self.tri = {"seen": False, "dx": 0.0, "dy": 0.0, "ts": 0.0}
        self.frame_wh = (1280, 960)

    def update(self, shape: str, seen: bool, dx: float, dy: float, wh: Tuple[int, int]):
        now = time.monotonic()
        with self.lock:
            self.frame_wh = wh
            if shape == "hexagon":
                self.hex["seen"] = bool(seen)
                if seen:
                    self.hex["dx"] = float(dx); self.hex["dy"] = float(dy); self.hex["ts"] = now
            elif shape == "triangle":
                self.tri["seen"] = bool(seen)
                if seen:
                    self.tri["dx"] = float(dx); self.tri["dy"] = float(dy); self.tri["ts"] = now

    def snapshot(self):
        with self.lock:
            return dict(self.hex), dict(self.tri), self.frame_wh


# =========================
# Single MAVSDK connection manager
# =========================
class Mav:
    def __init__(self, url: str):
        self.url = url
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        self.drone: Optional[System] = None
        self.connected = threading.Event()

        self.alt_rel = 0.0
        self.alt_ts = 0.0
        self.ned = (0.0, 0.0, 0.0)
        self.yaw_deg = 0.0
        self.flight_mode = None

        self.submit(self._connect())

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    async def _connect(self):
        self.drone = System()
        await self.drone.connect(system_address=self.url)
        async for st in self.drone.core.connection_state():
            if st.is_connected:
                print("[mavsdk] Connected (single System).")
                self.connected.set()
                break

        self.loop.create_task(self._tel_position())
        self.loop.create_task(self._tel_ned())
        self.loop.create_task(self._tel_att())
        self.loop.create_task(self._tel_fm())

    async def _tel_position(self):
        try:
            async for pos in self.drone.telemetry.position():
                self.alt_rel = float(pos.relative_altitude_m)
                self.alt_ts = time.monotonic()
        except Exception as e:
            print(f"[mavsdk] position telemetry ended: {e}")

    async def _tel_ned(self):
        try:
            async for pv in self.drone.telemetry.position_velocity_ned():
                self.ned = (float(pv.position.north_m), float(pv.position.east_m), float(pv.position.down_m))
        except Exception as e:
            print(f"[mavsdk] ned telemetry ended: {e}")

    async def _tel_att(self):
        try:
            async for e in self.drone.telemetry.attitude_euler():
                self.yaw_deg = float(e.yaw_deg)
        except Exception as e:
            print(f"[mavsdk] att telemetry ended: {e}")

    async def _tel_fm(self):
        try:
            async for fm in self.drone.telemetry.flight_mode():
                self.flight_mode = fm
        except Exception as e:
            print(f"[mavsdk] fm telemetry ended: {e}")

    def alt(self) -> float:
        return self.alt_rel

    def get_ned(self) -> Tuple[float, float, float]:
        return self.ned

    async def strong_resume_mission(self, note: str = ""):
        # stop offboard must have been called before this
        try:
            await self.drone.mission.start_mission()
        except Exception as e:
            print(f"[mavsdk] start_mission err: {e}")

        # wait a bit for flightmode
        t0 = time.monotonic()
        ok = False
        while (time.monotonic() - t0) < 2.5:
            if self.flight_mode == FlightMode.MISSION:
                ok = True
                break
            await asyncio.sleep(0.05)

        if ok:
            print(f"[mavsdk] Mission resumed. {note}")
            return True

        # fallback: try RAW
        try:
            await self.drone.mission_raw.start_mission()
        except Exception as e:
            print(f"[mavsdk] mission_raw.start_mission err: {e}")

        t0 = time.monotonic()
        while (time.monotonic() - t0) < 2.0:
            if self.flight_mode == FlightMode.MISSION:
                print(f"[mavsdk] Mission resumed (raw). {note}")
                return True
            await asyncio.sleep(0.05)

        print(f"[mavsdk] Mission resume failed. {note}")
        return False


# =========================
# Offboard controller (single thread, continuous offboard)
# =========================
class OffboardTask(threading.Thread):
    """
    Runs 30Hz BODY velocity offboard.
    Modes:
      - MARK_TRI: 20m hold 5s then save NE and finish (no descent)
      - DROP_HEX / DROP_TRI: 20m hold 5s -> 8m hold 5s -> 2m hold 3s -> release
    Key fix:
      - During MID/HOLD/DESCENT we never command climb (v_down<0 => clamp to 0)
      - Vision loss tolerated up to 5s (keep last dx,dy)
    """
    def __init__(self, mav: Mav, bus: TargetBus, shape: str,
                 alt_top=20.0, alt_mid=8.0, alt_low=2.0,
                 hold_top=5.0, hold_mid=5.0, hold_low=3.0,
                 loss_tol=5.0,
                 kp_xy=2.0, vmax_xy=1.6,
                 kp_z=0.8, vmax_z=1.3,
                 actuator_slot=6, pulse_val=1.0):
        super().__init__(daemon=True)
        self.mav = mav
        self.bus = bus
        self.shape = shape  # "hexagon" or "triangle"
        self.alt_top = float(alt_top)
        self.alt_mid = float(alt_mid)
        self.alt_low = float(alt_low)
        self.hold_top = float(hold_top)
        self.hold_mid = float(hold_mid)
        self.hold_low = float(hold_low)
        self.loss_tol = float(loss_tol)

        self.kp_xy = float(kp_xy)
        self.vmax_xy = float(vmax_xy)
        self.kp_z = float(kp_z)
        self.vmax_z = float(vmax_z)

        self.actuator_slot = int(actuator_slot)
        self.pulse_val = float(pulse_val)

        self.stop_evt = threading.Event()
        self.ok_drop = False
        self.saved_ne = None

        # internal
        self._last_seen_ts = None
        self._dx = 0.0
        self._dy = 0.0

    def stop(self):
        self.stop_evt.set()

    def _read_err(self):
        hex_s, tri_s, _wh = self.bus.snapshot()
        now = time.monotonic()
        src = hex_s if self.shape == "hexagon" else tri_s

        if src["seen"]:
            self._dx = float(src["dx"])
            self._dy = float(src["dy"])
            self._last_seen_ts = now
            return True

        # no detection now
        if self._last_seen_ts is None:
            return False
        if (now - self._last_seen_ts) <= self.loss_tol:
            # keep last dx/dy
            return True
        return False

    def _clamp(self, x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    async def _set_vel(self, v_fwd, v_right, v_down):
        try:
            await self.mav.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(v_fwd, v_right, v_down, 0.0)
            )
            return True
        except OffboardError:
            try:
                await self.mav.drone.offboard.start()
                return True
            except Exception:
                return False

    async def _offboard_start(self):
        try:
            await self.mav.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
            await self.mav.drone.offboard.start()
            return True
        except OffboardError as e:
            print(f"[offboard] start err: {e}")
            return False

    async def _offboard_stop(self):
        try:
            await self.mav.drone.offboard.stop()
        except Exception:
            pass

    async def _countdown_log(self, label: str, remaining: float, last_tick: Dict[str, float]):
        # print once per second: 5..4..3..2..1
        now = time.monotonic()
        if "t" not in last_tick:
            last_tick["t"] = now - 999
        if (now - last_tick["t"]) >= 1.0:
            sec = int(math.ceil(max(0.0, remaining)))
            print(f"[{label}] {sec}...")
            last_tick["t"] = now

    async def _run_flight(self, mark_only: bool):
        if not await self._offboard_start():
            return

        dt = 1.0 / 30.0

        # Phase A: reach TOP altitude (allow climb only here)
        phase = "ALT_TOP"
        t_phase = time.monotonic()
        hold_deadline = None
        countdown_tick = {}

        while not self.stop_evt.is_set():
            now = time.monotonic()

            ok_seen = self._read_err()
            if not ok_seen:
                print(f"[{self.shape}] LOST > {self.loss_tol:.0f}s => ABORT")
                return

            # XY velocities
            v_right = self._clamp(self.kp_xy * self._dx, -self.vmax_xy, self.vmax_xy)
            v_fwd   = self._clamp(self.kp_xy * self._dy, -self.vmax_xy, self.vmax_xy)

            alt = self.mav.alt()

            if phase == "ALT_TOP":
                alt_err = (alt - self.alt_top)
                # allow climb/descend to reach top
                v_down = self._clamp(self.kp_z * alt_err, -self.vmax_z, self.vmax_z)

                # enter hold when near enough OR after 8s force hold
                if abs(alt_err) <= 0.35 or (now - t_phase) >= 8.0:
                    phase = "HOLD_TOP"
                    hold_deadline = now + self.hold_top
                    countdown_tick.clear()
                    print(f"[{self.shape}] Reached ~{self.alt_top:.0f}m → HOLD_TOP ({self.hold_top:.0f}s).")
                    v_down = 0.0

            elif phase == "HOLD_TOP":
                # NO CLIMB here: only down or 0
                alt_err = (alt - self.alt_top)
                v_down = self._clamp(0.5 * self.kp_z * alt_err, -0.25, 0.25)
                if v_down < 0.0:
                    v_down = 0.0

                await self._countdown_log(f"{self.shape} HOLD_TOP", hold_deadline - now, countdown_tick)

                if now >= hold_deadline:
                    if mark_only:
                        n, e, _d = self.mav.get_ned()
                        self.saved_ne = (float(n), float(e))
                        print(f"[TRI] MARKED NE={self.saved_ne} ✅")
                        return
                    phase = "DESC_TO_MID"
                    print(f"[{self.shape}] HOLD_TOP done → DESC_TO_MID ({self.alt_mid:.0f}m).")

            elif phase == "DESC_TO_MID":
                alt_err = (alt - self.alt_mid)
                v_down = self._clamp(self.kp_z * alt_err, 0.35, self.vmax_z) if alt_err > 0 else 0.0
                # never climb
                if v_down < 0.0:
                    v_down = 0.0
                if alt <= (self.alt_mid + 0.25):
                    phase = "HOLD_MID"
                    hold_deadline = now + self.hold_mid
                    countdown_tick.clear()
                    print(f"[{self.shape}] Reached ~{self.alt_mid:.0f}m → HOLD_MID ({self.hold_mid:.0f}s).")
                    v_down = 0.0

            elif phase == "HOLD_MID":
                # NO CLIMB (this is the big fix for your 8m bounce)
                alt_err = (alt - self.alt_mid)
                v_down = self._clamp(0.5 * self.kp_z * alt_err, -0.20, 0.20)
                if v_down < 0.0:
                    v_down = 0.0

                await self._countdown_log(f"{self.shape} HOLD_MID", hold_deadline - now, countdown_tick)

                if now >= hold_deadline:
                    phase = "DESC_TO_LOW"
                    print(f"[{self.shape}] HOLD_MID done → DESC_TO_LOW ({self.alt_low:.0f}m).")

            elif phase == "DESC_TO_LOW":
                alt_err = (alt - self.alt_low)
                v_down = self._clamp(self.kp_z * alt_err, 0.35, self.vmax_z) if alt_err > 0 else 0.0
                if v_down < 0.0:
                    v_down = 0.0
                if alt <= (self.alt_low + 0.20):
                    phase = "HOLD_LOW"
                    hold_deadline = now + self.hold_low
                    countdown_tick.clear()
                    print(f"[{self.shape}] Reached ~{self.alt_low:.0f}m → HOLD_LOW ({self.hold_low:.0f}s).")
                    v_down = 0.0

            elif phase == "HOLD_LOW":
                # tiny corrections, NO CLIMB
                alt_err = (alt - self.alt_low)
                v_down = self._clamp(0.4 * self.kp_z * alt_err, -0.12, 0.12)
                if v_down < 0.0:
                    v_down = 0.0

                # 3..2..1
                await self._countdown_log(f"{self.shape} HOLD_LOW", hold_deadline - now, countdown_tick)

                if now >= hold_deadline:
                    phase = "RELEASE"
                    v_down = 0.0
                    print(f"[{self.shape}] HOLD_LOW done → RELEASE.")

            elif phase == "RELEASE":
                try:
                    print(f"[payload] set_actuator(slot={self.actuator_slot}, value={self.pulse_val})")
                    await self.mav.drone.action.set_actuator(self.actuator_slot, self.pulse_val)
                    await asyncio.sleep(1.0)
                    print(f"[payload] set_actuator(slot={self.actuator_slot}, value=0.0)")
                    await self.mav.drone.action.set_actuator(self.actuator_slot, 0.0)
                    await asyncio.sleep(1.5)
                    self.ok_drop = True
                    print(f"[{self.shape}] DROP DONE ✅")
                except Exception as e:
                    print(f"[payload] actuator err: {e}")
                return

            ok = await self._set_vel(v_fwd, v_right, v_down)
            if not ok:
                print("[offboard] set_velocity failed hard.")
                return

            await asyncio.sleep(dt)

    def run(self):
        async def _runner():
            try:
                # Determine mark_only: triangle mark task uses pulse_val=None in orchestrator
                mark_only = (self.pulse_val == 9999.0)
                await self._run_flight(mark_only=mark_only)
            finally:
                await self._offboard_stop()
        self.mav.submit(_runner())


# =========================
# GoTo (single System) position offboard
# =========================
async def goto_ne_at_alt(mav: Mav, target_ne: Tuple[float, float], alt_target: float = 20.0, arrive_m: float = 0.8):
    yaw = mav.yaw_deg
    try:
        await mav.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw))
        await mav.drone.offboard.start()
        print("[goto] offboard position started.")
    except OffboardError as e:
        print(f"[goto] start err: {e}")
        return False

    try:
        n0, e0, d0 = mav.get_ned()
        alt_now = mav.alt()
        down_target = float(d0 - (alt_target - alt_now))
        print(f"[goto] down_target: alt_now={alt_now:.2f} d={d0:.2f} -> d_target={down_target:.2f}")

        while True:
            n, e, d = mav.get_ned()
            dist = math.hypot(target_ne[0] - n, target_ne[1] - e)
            await mav.drone.offboard.set_position_ned(PositionNedYaw(target_ne[0], target_ne[1], down_target, yaw))
            if dist <= arrive_m:
                print("[goto] arrived near saved NE.")
                return True
            await asyncio.sleep(0.05)
    finally:
        try:
            await mav.drone.offboard.stop()
            print("[goto] offboard position stopped.")
        except Exception:
            pass


# =========================
# Orchestrator (HEX priority)
# =========================
class Orchestrator:
    def __init__(self, mav: Mav, bus: TargetBus, detector: Detector):
        self.mav = mav
        self.bus = bus
        self.detector = detector

        self.hex_done = False
        self.tri_done = False
        self.tri_saved_ne: Optional[Tuple[float, float]] = None

        self.active: Optional[OffboardTask] = None
        self.lock = threading.Lock()

        # servo pulse values
        self.HEX_PULSE = 1.0
        self.TRI_PULSE = -1.0

        # logic params
        self.ALT_TOP = 20.0
        self.ALT_MID = 8.0
        self.ALT_LOW = 2.0

    def busy(self):
        return self.active is not None and self.active.is_alive()

    def _clear_task(self):
        if self.active and not self.active.is_alive():
            self.active = None

    def step(self):
        self._clear_task()
        if self.busy():
            return

        hex_s, tri_s, _wh = self.bus.snapshot()
        hex_seen = bool(hex_s["seen"])
        tri_seen = bool(tri_s["seen"])

        # 1) HEX always first
        if (not self.hex_done) and hex_seen:
            print("[HEX] START")
            self.active = OffboardTask(self.mav, self.bus, "hexagon",
                                      alt_top=self.ALT_TOP, alt_mid=self.ALT_MID, alt_low=self.ALT_LOW,
                                      hold_top=5.0, hold_mid=5.0, hold_low=3.0,
                                      loss_tol=5.0,
                                      actuator_slot=6, pulse_val=self.HEX_PULSE)
            self.active.start()
            return

        # 2) TRI first (before HEX): MARK only (20m hold 5s then save NE)
        if (not self.hex_done) and (self.tri_saved_ne is None) and tri_seen:
            print("[TRI] MARK START (no drop)")
            # mark task uses pulse_val=9999 sentinel to mean "mark_only"
            self.active = OffboardTask(self.mav, self.bus, "triangle",
                                      alt_top=self.ALT_TOP, alt_mid=self.ALT_MID, alt_low=self.ALT_LOW,
                                      hold_top=5.0, hold_mid=5.0, hold_low=3.0,
                                      loss_tol=5.0,
                                      actuator_slot=6, pulse_val=9999.0)
            self.active.start()
            return

        # 3) After HEX done: TRI drop (prefer saved NE -> goto -> drop)
        if self.hex_done and (not self.tri_done):
            # if we have saved NE, go there then wait for TRI detection (same pipeline)
            if self.tri_saved_ne is not None:
                print(f"[TRI] DROP FROM SAVED NE {self.tri_saved_ne}")
                self.mav.submit(self._tri_drop_saved())
                return
            # else direct TRI if visible
            if tri_seen:
                print("[TRI] DIRECT DROP")
                self.active = OffboardTask(self.mav, self.bus, "triangle",
                                          alt_top=self.ALT_TOP, alt_mid=self.ALT_MID, alt_low=self.ALT_LOW,
                                          hold_top=5.0, hold_mid=5.0, hold_low=3.0,
                                          loss_tol=5.0,
                                          actuator_slot=6, pulse_val=self.TRI_PULSE)
                self.active.start()
                return

        # else nothing: stay mission

    async def _tri_drop_saved(self):
        # wait if currently in offboard (should not)
        try:
            # pause mission is optional
            try:
                await self.mav.drone.mission.pause_mission()
            except Exception:
                pass

            ok = await goto_ne_at_alt(self.mav, self.tri_saved_ne, alt_target=self.ALT_TOP)
            if not ok:
                await self.mav.strong_resume_mission("(tri goto failed)")
                return

            # start TRI drop controller thread
            self.active = OffboardTask(self.mav, self.bus, "triangle",
                                      alt_top=self.ALT_TOP, alt_mid=self.ALT_MID, alt_low=self.ALT_LOW,
                                      hold_top=5.0, hold_mid=5.0, hold_low=3.0,
                                      loss_tol=5.0,
                                      actuator_slot=6, pulse_val=self.TRI_PULSE)
            self.active.start()
        except Exception as e:
            print(f"[TRI] drop_saved exception: {e}")

    def poll_task_results(self):
        # called in main loop to harvest results and resume mission cleanly
        if self.active is None:
            return
        if self.active.is_alive():
            return

        # finished
        t = self.active
        self.active = None

        # mark-only?
        if t.pulse_val == 9999.0:
            if t.saved_ne is not None:
                self.tri_saved_ne = t.saved_ne
            # IMPORTANT: ignore TRI until HEX done? (your rule)
            print("[TRI] MARK DONE -> mission resume")
            self.detector.ignore_triangle = False  # allow detection still, but orchestrator won't drop before HEX
            self.mav.submit(self.mav.strong_resume_mission("(tri marked)"))
            return

        # drop done?
        if t.ok_drop:
            if t.shape == "hexagon":
                self.hex_done = True
                print("[HEX] DONE -> unlock TRI")
                self.detector.ignore_hexagon = True  # don’t re-trigger hex
                self.mav.submit(self.mav.strong_resume_mission("(hex done)"))
                return
            if t.shape == "triangle":
                self.tri_done = True
                self.tri_saved_ne = None
                self.detector.ignore_triangle = True
                print("[TRI] DONE -> mission resume")
                self.mav.submit(self.mav.strong_resume_mission("(tri done)"))
                return

        # aborted -> mission resume
        print(f"[{t.shape}] task aborted -> mission resume")
        self.mav.submit(self.mav.strong_resume_mission(f"({t.shape} abort)"))


# =========================
# Args + main
# =========================
def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", type=str, required=False,
                    help="GZ image topic. Empty => default x500 down cam.")
    ap.add_argument("--mavsdk-url", type=str, default="udpin://0.0.0.0:14540")
    ap.add_argument("--no-display", action="store_true")
    return ap.parse_args()


def main():
    args = parse_args()
    if not args.topic or not args.topic.strip():
        args.topic = "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
        print("[auto] --topic verilmedi, default kullanılıyor:")
        print(f"       {args.topic}")

    reader = GzCameraReader(args.topic)
    reader.start()
    print(f"[gz] Subscribed: {args.topic}")

    detector = Detector()
    bus = TargetBus()

    mav = Mav(args.mavsdk_url)
    if not mav.connected.wait(timeout=10.0):
        print("[mavsdk] connect timeout!")
        return

    orch = Orchestrator(mav, bus, detector)

    prev_time = time.time()
    frame_count = 0
    fps_val = 0.0

    try:
        while True:
            with frame_lock:
                if latest_frame is None:
                    time.sleep(0.005)
                    continue
                frame = latest_frame.copy()

            tri, hexg = detector.detect(frame)

            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2

            # update bus for both shapes independently
            if hexg is not None:
                tx, ty = hexg["center"]
                dx = (tx - cx) / float(cx)
                dy = (cy - ty) / float(cy)
                bus.update("hexagon", True, dx, dy, (w, h))
            else:
                bus.update("hexagon", False, 0.0, 0.0, (w, h))

            if tri is not None:
                tx, ty = tri["center"]
                dx = (tx - cx) / float(cx)
                dy = (cy - ty) / float(cy)
                bus.update("triangle", True, dx, dy, (w, h))
            else:
                bus.update("triangle", False, 0.0, 0.0, (w, h))

            # orchestrator + harvest finished tasks
            orch.poll_task_results()
            orch.step()

            # fps
            frame_count += 1
            now = time.time()
            if now - prev_time >= 1.0:
                fps_val = frame_count / (now - prev_time)
                prev_time = now
                frame_count = 0

            if not args.no_display:
                detector.draw(frame, fps=fps_val)
                cv2.imshow("Tracking", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    finally:
        reader.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
