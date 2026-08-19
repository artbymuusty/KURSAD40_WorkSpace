import logging
import asyncio
import math
import os
import time
from core.interfaces.i_flight_backend import IFlightBackend, TelemetryStale
from core.interfaces.i_camera_source import ICameraSource
from core.detection.camera_intrinsics import default_camera_intrinsics
from core.detection.detection_feed import DetectionFeed
from core.detection.types import Detection
from core.config.parameters import (
    MISSION_ALTITUDE_M, HOVER_DURATION_S, MAX_CENTERING_SPEED_M_S,
    OFFBOARD_SETPOINT_INTERVAL_S, OFFBOARD_MODE_CONFIRM_TIMEOUT_S,
    CENTERING_TOLERANCE_X_NORM, CENTERING_TOLERANCE_Y_NORM,
    CENTERING_ALTITUDE_CHANGE_ATTEMPTS,
    KP_ALTITUDE, ALTITUDE_CONVERGENCE_TOLERANCE_M, CENTERING_LATERAL_TIMEOUT_S,
    GPS_POSITION_CONVERGENCE_TOLERANCE_M, GPS_POSITION_VELOCITY_TOLERANCE_M_S,
    GLOBAL_POSITION_NAV_TIMEOUT_S,
    CENTERING_LATERAL_TIMEOUT_ENV, CENTERING_MIN_CMD_SPEED_M_S,
    CENTERING_FLOOR_TOL_FRACTION,
    LOW_ALT_VISION_LIMIT_M, LOW_ALT_BBOX_CENTER, LOW_ALT_OPEN_LOOP_TIMEOUT_S,
    PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M, LOW_ALT_OPEN_LOOP_MIN_DESCENT_M_S,
    TARGET_LOSS_GRACE_FRAMES,
    MOUNT_TRANSLATE_BUDGET_S, MOUNT_TRANSLATE_TOLERANCE_M,
    CAMERA_LEVER_ARM_BODY_M,
)
from core.navigation.setpoint_limiter import SetpointLimiter
from core.navigation.geo import gps_to_ned_delta, haversine_distance_m
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger(__name__)

# ADR-008 B1: how often the per-iteration centering telemetry is echoed to
# the console. Every iteration is published as a structured CENTERING_STEP
# event (that is what the dashboard and the post-run analysis read); the
# INFO line is throttled so a 10 Hz loop does not bury the log.
_CENTERING_LOG_INTERVAL_S = 0.5


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def ground_tolerance_m(tolerance_norm: float, half_axis_px: float,
                       alt_m: float, focal_px: float) -> float:
    """How wide the centering tolerance actually is ON THE GROUND at `alt_m`.

    The control law works in normalized pixel error, which is an ANGULAR
    quantity -- so the same tolerance means 0.18m of ground at 15m and
    3.6mm at 0.30m. Every altitude-dependent decision about the command
    floor has to start here."""
    tol_px = tolerance_norm * half_axis_px
    return alt_m * tol_px / focal_px


def floor_speed_m_s(tolerance_norm: float, half_axis_px: float,
                    alt_m, focal_px) -> float:
    """ADR-009 S2: the command floor for one axis at the current altitude.

    Capped so a single OFFBOARD_SETPOINT_INTERVAL_S step covers at most
    CENTERING_FLOOR_TOL_FRACTION of the ground tolerance band -- i.e. the
    vehicle cannot be commanded straight across the band and out the other
    side, which is exactly the bang-bang limit cycle V1' produced at 0.30m.

    Falls back to the flat S1 floor when altitude or intrinsics are
    unavailable: without them there is nothing to scale by, and the flat
    floor is still better than the 9 mm/s dead zone it replaced."""
    if alt_m is None or focal_px is None or alt_m <= 0.0:
        return CENTERING_MIN_CMD_SPEED_M_S
    band_m = ground_tolerance_m(tolerance_norm, half_axis_px, alt_m, focal_px)
    return min(CENTERING_MIN_CMD_SPEED_M_S,
               CENTERING_FLOOR_TOL_FRACTION * band_m / OFFBOARD_SETPOINT_INTERVAL_S)


def _with_min_speed(cmd_m_s: float, error_norm: float, tolerance: float,
                    floor_m_s: float = CENTERING_MIN_CMD_SPEED_M_S) -> float:
    """ADR-009 S1 (+S2 floor). Pure proportional control asymptotes: as the
    error shrinks the command shrinks with it, and below a few cm/s PX4
    simply cannot act on it against drift. V1 pursuit 6 sat at |ey|=0.0146
    (~7px, 0.20m) for a full 15s budget commanding 0.009 m/s and the error
    never moved.

    So: outside tolerance, command at least `floor_m_s` in the direction the
    error says. Inside tolerance, command exactly 0 -- the floor must not
    push the vehicle around once it has arrived, which is what would turn it
    into a limit cycle."""
    if abs(error_norm) < tolerance:
        return 0.0
    if abs(cmd_m_s) >= floor_m_s:
        return cmd_m_s
    direction = 1.0 if cmd_m_s > 0 else (-1.0 if cmd_m_s < 0 else 0.0)
    if direction == 0.0:
        return 0.0
    return direction * floor_m_s


class CenteringController:
    def __init__(self, flight: IFlightBackend, detection_feed: DetectionFeed, camera: ICameraSource,
                 publisher: EventPublisher = NULL_PUBLISHER):
        self.flight = flight
        # ADR-008 B1: was `detector: IDetector`, and go_to_and_center() used
        # to run its own camera.get_frame() + detector.detect() every
        # iteration -- a second, independently-scheduled consumer of the one
        # detector instance. See core/detection/detection_feed.py's module
        # docstring for the full failure this caused. Centering is now a
        # pure CONSUMER of the orchestrator's single detection loop.
        self.detection_feed = detection_feed
        self.camera = camera
        self.publisher = publisher
        # ADR-010 P4: setpoint-stage smoothing. Lives on the controller (not
        # per-call) so a ramp is not restarted by every go_to_and_center(),
        # but reset() is called at the start of each session -- see there.
        self._limiter = SetpointLimiter()
        # ADR-010 P1: last known yaw, refreshed each centering session. The
        # pixel->ground back-projection needs it to rotate body offsets into
        # north/east, and the open-loop descent needs it to steer at a GPS
        # point in body frame. 0.0 until the first session reads it, which
        # matches the yaw the vehicle takes off with.
        self._last_yaw_deg = 0.0
        # D3: the last committed target fix, carried out of go_to_and_center
        # so descend_to_release can translate it by the mount vector.
        self._last_frozen_estimate = None
        # TODO[KONTROL]: Gerçek kapalı döngü kazanç (gain) değerleri fiziksel testlerle
        # ayarlanacaktır (bkz. real_system.yaml / gz_system.yaml config parametreleri
        # Kp_yatay, Kp_dikey). Bu değerler artık GERÇEKTEN kullanılıyor (bkz.
        # go_to_and_center) -- daha önce tanımlı olup hiçbir yerde okunmuyorlardı.
        self.kp_horizontal = 0.5
        self.kp_vertical = 0.3
        # Operator-specified precision (2026-08-13 revision): ±0.01 normalized
        # in both axes, config-injected per real_system.yaml/gz_system.yaml
        # exactly like kp_horizontal/kp_vertical above.
        self.tolerance_x = CENTERING_TOLERANCE_X_NORM
        self.tolerance_y = CENTERING_TOLERANCE_Y_NORM
        self.kp_altitude = KP_ALTITUDE
        # BUG FIX (operator-reported, 2026-08-13): the lateral-only (no
        # altitude change) branch of go_to_and_center's max_attempts used to
        # be a fixed 30 (3s), sized for the old 20px tolerance -- too short
        # for the new, much tighter ±0.01 normalized precision to reliably
        # converge against real detector/control noise. Overridable per-call
        # like tolerance_x/y (tests can shrink this for speed).
        # ADR-009 D3: env-overridable so a validation run can shorten the
        # budget while keeping the loop's real pacing. This replaces the
        # ADR-008 instant-fail hook, which returned False before the loop
        # even started -- that made every failure free, so failed pursuits
        # re-engaged ~3x/second and wedged PX4 (2026-08-16 23:06).
        self.lateral_timeout_s = float(
            os.environ.get(CENTERING_LATERAL_TIMEOUT_ENV, CENTERING_LATERAL_TIMEOUT_S))
        if self.lateral_timeout_s != CENTERING_LATERAL_TIMEOUT_S:
            logger.warning("[TEST] %s=%.1fs -- merkezleme butcesi kisaltildi (varsayilan %.1fs).",
                           CENTERING_LATERAL_TIMEOUT_ENV, self.lateral_timeout_s,
                           CENTERING_LATERAL_TIMEOUT_S)

    def _publish(self, code, message="", severity=Severity.INFO, data=None):
        self.publisher.publish(Event(
            code=code, subsystem="CenteringController", category=Category.NAVIGATION,
            severity=severity, message=message, data=data or {},
        ))

    async def switch_to_offboard(self) -> bool:
        """Mission modu durur, Offboard'a geçilir (Bölüm 8).

        BUG FIX (operator-reported): previously this called
        switch_to_offboard_from_mission() + start_offboard() and returned
        None unconditionally -- nothing ever confirmed PX4 actually
        accepted the mode change, and an OffboardError from PX4 rejecting
        it would propagate uncaught all the way out of
        Gorev2Orchestrator.run(), aborting the entire Görev 2 mission over
        a single failed engagement attempt. Now returns bool: the caller
        must check it and fall back to SEARCHING instead of blindly
        proceeding into go_to_and_center() while still in Mission mode."""
        logger.info("Mission modu durduruluyor, Offboard'a geciliyor...")
        await self.flight.switch_to_offboard_from_mission()

        try:
            await self.flight.start_offboard()
        except Exception as e:
            logger.error(f"Offboard baslatma reddedildi: {e}")
            self._publish("OFFBOARD_SWITCH_FAILED", str(e), severity=Severity.CRITICAL, data={"error": str(e)})
            return False

        # Verify PX4 actually reports OFFBOARD instead of trusting
        # start_offboard()'s mere absence of an exception -- PX4 can accept
        # the command and still not be in OFFBOARD a moment later for
        # reasons the MAVSDK call itself won't surface.
        deadline = asyncio.get_event_loop().time() + OFFBOARD_MODE_CONFIRM_TIMEOUT_S
        while asyncio.get_event_loop().time() < deadline:
            mode = await self.flight.get_flight_mode()
            if mode == "OFFBOARD":
                self._publish("OFFBOARD_SWITCH_CONFIRMED", data={"flight_mode": mode})
                return True
            await asyncio.sleep(0.2)

        logger.error("PX4 OFFBOARD modunu onaylamadi (timeout).")
        self._publish("OFFBOARD_SWITCH_FAILED", "PX4 did not report OFFBOARD before timeout",
                      severity=Severity.CRITICAL, data={"timeout_s": OFFBOARD_MODE_CONFIRM_TIMEOUT_S})
        return False

    def budget_s(self, start_alt_m: float, altitude_m: float = MISSION_ALTITUDE_M) -> float:
        """ADR-008 B1: the REAL wall-clock budget go_to_and_center() will
        spend, so the dashboard's WAITING_CENTERING_CONVERGENCE banner can
        report it instead of the fictional CENTERING_CONVERGENCE_TIMEOUT_S
        (5.0s, while the loop actually ran 77s and 82s on 2026-08-16).

        Same numbers the loop itself uses -- this is the single source of
        truth both now read, not a second estimate that can drift."""
        return self._max_attempts(start_alt_m, altitude_m) * OFFBOARD_SETPOINT_INTERVAL_S

    def _max_attempts(self, start_alt_m: float, altitude_m: float) -> int:
        # self.lateral_timeout_s (not the module constant) so a per-call
        # override -- tests shrink it for speed -- is reflected in the
        # reported budget too, instead of the banner and the loop
        # disagreeing.
        lateral_only = abs(start_alt_m - altitude_m) < ALTITUDE_CONVERGENCE_TOLERANCE_M
        return (int(self.lateral_timeout_s / OFFBOARD_SETPOINT_INTERVAL_S) if lateral_only
                else CENTERING_ALTITUDE_CHANGE_ATTEMPTS)

    def _aim_offset_px(self, aim_offset_body_m, current_alt_m: float,
                       res_w: int, res_h: int):
        """A2: the mount vector expressed in image pixels.

        No heading rotation is involved here and none is needed: the camera
        is rigidly mounted to the airframe, so image axes ARE body axes
        (image +x = body right, image +y = body aft, the same convention
        the velocity commands use). Heading only enters once a position
        leaves the body frame, which happens in _freeze_target_estimate --
        and because the residual error computed here is what gets frozen,
        that rotation is applied there exactly once, for free.

        Returns (0, 0) whenever the geometry cannot be resolved, so a
        missing intrinsic degrades to camera-centred aim rather than to a
        wrong aim point."""
        if not aim_offset_body_m or not current_alt_m or current_alt_m <= 0.0:
            return (0.0, 0.0)
        intrinsics = default_camera_intrinsics()
        if intrinsics is None:
            return (0.0, 0.0)
        focal = intrinsics.scaled_to(res_w, res_h).focal_px
        if not focal:
            return (0.0, 0.0)
        forward_m, right_m = aim_offset_body_m
        px_per_m = focal / current_alt_m
        return (right_m * px_per_m, -forward_m * px_per_m)

    async def descend_to_release(self, shape_type: str, altitude_m: float,
                                 mount_body_m) -> float:
        """PHASE 13 D3: put the PAYLOAD over the target, then descend.

        The vision loop has just converged with the target CENTRED, which
        is the regime where the measurement is least corrupted by clipping.
        Its last committed estimate is therefore the best fix on the target
        this flight will get. The mount offset is applied here, once, as a
        pure translation of that point -- the vehicle holds at
        `target - mount` so the payload hangs over the target -- and the
        existing open-loop descent flies it down while holding position on
        GPS the whole way.

        This replaces biasing the vision error (the first A2), which
        corrupted the measurement it depended on and produced no net
        improvement: 40.1 / 32.5 cm against a 33.7-37.3 cm baseline.

        Returns the altitude actually reached."""
        estimate = self._last_frozen_estimate
        if estimate is None:
            logger.warning("[AIM_OFFSET_APPLIED] %s: dondurulmus kestirim yok -- "
                           "montaj otelemesi uygulanamiyor.", shape_type)
            return await self._descend_without_estimate(altitude_m)

        forward_m, right_m = mount_body_m or (0.0, 0.0)
        yaw_rad = math.radians(self._last_yaw_deg or 0.0)
        # Body -> NED, then SUBTRACTED: the vehicle must sit mount-vector
        # opposite the target so the payload ends up on it.
        north_m = -(forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad))
        east_m = -(forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad))
        held = dict(estimate)
        held["lat"] = estimate["lat"] + north_m / 111320.0
        held["lon"] = estimate["lon"] + east_m / (
            111320.0 * max(0.1, math.cos(math.radians(estimate["lat"]))))

        logger.info("[AIM_OFFSET_APPLIED] %s: montaj (ileri=%.2f, sag=%.2f) m, yon=%.1f deg "
                    "-> tutma noktasi (%.7f, %.7f) olarak otelendi (kuzey %+.2f, dogu %+.2f m).",
                    shape_type, forward_m, right_m, self._last_yaw_deg or 0.0,
                    held["lat"], held["lon"], north_m, east_m)
        self._publish("AIM_OFFSET_APPLIED", shape_type,
                      data={"shape_type": shape_type,
                            "payload_mount_body_m": list(mount_body_m or ()),
                            "heading_deg": round(self._last_yaw_deg or 0.0, 1),
                            "target_estimate": {"lat": estimate["lat"], "lon": estimate["lon"]},
                            "held_point": {"lat": held["lat"], "lon": held["lon"]},
                            "translation_ned_m": [round(north_m, 3), round(east_m, 3)],
                            "measured_from_alt_m": estimate.get("from_alt_m")})

        # T1: translate FIRST, holding altitude, on its own budget. Then
        # descend. Doing both at once let the asymptotic lateral move eat the
        # descent timeout and drop the release out of band (0.385 / 0.159 m).
        await self._mount_translate(shape_type, held)

        try:
            _, _, alt = await self.flight.get_global_position()
        except TelemetryStale:
            alt = altitude_m
        # No lateral tolerance here on purpose: the translation is already
        # done, and re-imposing one would hand the descent the same way to
        # exhaust its budget. A1's altitude band is the only exit condition.
        return await self._open_loop_descend(shape_type, held, altitude_m, alt)

    async def _mount_translate(self, shape_type: str, held: dict) -> float:
        """T1: fly the mount translation at constant altitude.

        Same lateral P-law and the same CENTERING_MIN_CMD_SPEED floor the
        centering loop uses -- the floor matters here, because pure
        proportional control on a 0.28 m error crawls the last few
        centimetres and that crawl is what previously consumed the descent.
        Bounded by MOUNT_TRANSLATE_BUDGET_S; on timeout it logs the residual
        and hands over anyway, because a slightly-off translation is a far
        better outcome than a release at the wrong altitude.

        Returns the residual distance to the translated hold, in metres."""
        started = time.monotonic()
        residual = float("inf")
        while time.monotonic() - started < MOUNT_TRANSLATE_BUDGET_S:
            try:
                lat, lon, _alt = await self.flight.get_global_position()
            except TelemetryStale as e:
                self._abort_on_stale("MOUNT_TRANSLATE", shape_type, e)
                break
            north_m, east_m = gps_to_ned_delta(lat, lon, held["lat"], held["lon"])
            residual = math.hypot(north_m, east_m)
            if residual <= MOUNT_TRANSLATE_TOLERANCE_M:
                break
            yaw_rad = math.radians(self._last_yaw_deg or 0.0)
            forward_m = north_m * math.cos(yaw_rad) + east_m * math.sin(yaw_rad)
            right_m = -north_m * math.sin(yaw_rad) + east_m * math.cos(yaw_rad)
            forward_m_s = _clamp(forward_m * self.kp_horizontal, MAX_CENTERING_SPEED_M_S)
            right_m_s = _clamp(right_m * self.kp_horizontal, MAX_CENTERING_SPEED_M_S)
            if abs(forward_m_s) < CENTERING_MIN_CMD_SPEED_M_S and abs(forward_m) > 0.01:
                forward_m_s = math.copysign(CENTERING_MIN_CMD_SPEED_M_S, forward_m)
            if abs(right_m_s) < CENTERING_MIN_CMD_SPEED_M_S and abs(right_m) > 0.01:
                right_m_s = math.copysign(CENTERING_MIN_CMD_SPEED_M_S, right_m)
            # down = 0: this phase holds altitude, full stop.
            await self._send_setpoint(forward_m_s, right_m_s, 0.0)
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)

        await self._send_setpoint(0.0, 0.0, 0.0)
        elapsed = time.monotonic() - started
        converged = residual <= MOUNT_TRANSLATE_TOLERANCE_M
        logger.info("[MOUNT_TRANSLATE_DONE] %s: kalan %.1f cm, sure %.2f s (%s).",
                    shape_type, residual * 100.0, elapsed,
                    "yakinsadi" if converged else "SURE DOLDU")
        self._publish("MOUNT_TRANSLATE_DONE", shape_type,
                      severity=Severity.INFO if converged else Severity.WARN,
                      data={"shape_type": shape_type,
                            "residual_cm": round(residual * 100.0, 1),
                            "elapsed_s": round(elapsed, 2),
                            "tolerance_cm": MOUNT_TRANSLATE_TOLERANCE_M * 100.0,
                            "budget_s": MOUNT_TRANSLATE_BUDGET_S,
                            "converged": converged})
        return residual

    async def _descend_without_estimate(self, altitude_m: float) -> float:
        """Degenerate path: nothing was ever committed, so there is no point
        to translate. Descend in place rather than inventing a hold point."""
        return await self.climb_to_altitude(altitude_m)

    async def go_to_and_center(self, target_shape_type: str, altitude_m: float = MISSION_ALTITUDE_M,
                               alt_tolerance_m: float = None,
                               aim_offset_body_m=None) -> bool:
        """Aracı hedefe yönlendirir, irtifayı korur, şekli kare merkezine getirir.
        Merkezleme tamamlanınca True döner.

        BUG FIX (operator-reported): this used to compute pixel error and
        just check it against a threshold -- it never called
        set_velocity_body() at all, so nothing ever drove the vehicle
        toward the target, and PX4 auto-exits Offboard after ~500ms without
        a new setpoint regardless. Now streams a real proportional-control
        velocity setpoint every iteration (well under PX4's Offboard
        timeout), and always ends on an explicit zero-velocity stop so no
        residual drift carries into hover_and_confirm()."""
        # A1: the altitude band is per-call. Staged approach steps keep the
        # loose ALTITUDE_CONVERGENCE_TOLERANCE_M (landing anywhere near 10 m
        # or 5 m is fine); the FINAL step is handed the release band. Before
        # this, the tight band only existed on the open-loop path, so the
        # release-altitude guarantee silently depended on whether vision had
        # been lost -- measured: payload 2 released at 0.564 m because its
        # triangle stayed visible and the loose 0.30 m band accepted it.
        alt_tolerance_m = (ALTITUDE_CONVERGENCE_TOLERANCE_M if alt_tolerance_m is None
                           else alt_tolerance_m)
        logger.info(f"{target_shape_type} hedefine merkezleniyor (irtifa hedefi: {altitude_m}m, "
                    f"irtifa bandi +/-{alt_tolerance_m:.2f}m)...")
        self._publish("CENTERING_STARTED", target_shape_type,
                      data={"shape_type": target_shape_type, "altitude_m": altitude_m,
                            "alt_tolerance_m": alt_tolerance_m,
                            "aim_offset_body_m": list(aim_offset_body_m) if aim_offset_body_m else None})

        # GAP FIX (operator revision, 2026-08-13): `altitude_m` was a dead
        # parameter -- nothing in this loop ever read it or commanded a
        # vertical setpoint, so every call centered laterally at whatever
        # altitude the vehicle already happened to be at. This is what makes
        # the staged payload approach (15m -> 10m -> 5m -> 0.30m, each step
        # re-centered) possible: this same loop now also closes the
        # altitude loop every iteration.
        #
        # max_attempts: the altitude loop is pure proportional control
        # (down_m_s = kp_altitude * alt_error), which only *asymptotically*
        # approaches the target -- a naive "distance / max_speed" time
        # estimate covers the initial clamped-speed phase but badly
        # underestimates the long decaying tail as alt_error shrinks toward
        # ALTITUDE_CONVERGENCE_TOLERANCE_M (verified: with kp_altitude=0.5,
        # a 5m descent needs ~55-60 iterations, not the ~25 a linear
        # estimate would suggest). Rather than an exact analytical settling-
        # time formula that's fragile to get right for arbitrary future
        # kp_altitude values, use a large fixed budget for any real altitude
        # change (200x0.1s=20s, comfortably covers even the largest jump --
        # the post-drop 0.30m -> MISSION_ALTITUDE_M climb-back) and
        # self.lateral_timeout_s (default 15s -- see its own BUG FIX comment
        # in __init__) for same-altitude lateral-only calls, i.e. the FIRST
        # lock-on pass at mission altitude.
        try:
            _, _, start_alt = await self.flight.get_global_position()
        except TelemetryStale as e:
            return self._abort_on_stale("CENTERING", target_shape_type, e)
        max_attempts = self._max_attempts(start_alt, altitude_m)

        # ADR-010 P4: start every session from a known-zero commanded state.
        # Without this the first tick of a new call is measured against the
        # PREVIOUS call's last command, and the rate limit would either be a
        # no-op (previous was already high) or ramp from a stale value. The
        # measured 2.50 m/s single-tick jumps were exactly these boundaries.
        self._limiter.reset()
        # ADR-010 P1: yaw for the pixel->ground back-projection.
        try:
            self._last_yaw_deg = await self.flight.get_yaw_deg()
        except Exception:  # noqa: BLE001 -- keep the last known yaw
            pass

        if aim_offset_body_m:
            logger.info("[AIM_OFFSET_APPLIED] %s: yuk montaj vektoru (ileri=%.2f, sag=%.2f) m, "
                        "yon=%.1f deg -- kamera degil YUK hedefe ortalanacak.",
                        target_shape_type, aim_offset_body_m[0], aim_offset_body_m[1],
                        self._last_yaw_deg or 0.0)
            self._publish("AIM_OFFSET_APPLIED", target_shape_type,
                          data={"shape_type": target_shape_type,
                                "payload_mount_body_m": list(aim_offset_body_m),
                                "heading_deg": round(self._last_yaw_deg or 0.0, 1),
                                "altitude_m": altitude_m})

        converged = False
        last_log = 0.0
        # ADR-010 P1: the most recent COMMITTED target position, kept so the
        # descent can continue on it after the detector stops committing.
        frozen_estimate = None
        last_seen_alt_m = None
        # PHASE 12 Q1: consecutive frames with no committed target.
        # Drives the transient-loss deceleration; reset the moment
        # the target comes back, so a dropout only ever costs the
        # frames it actually lasted.
        lost_frames = 0
        for attempt in range(1, max_attempts + 1):
            # ADR-008 B1: reads the orchestrator's single detection loop
            # instead of running a competing detect() of its own. `get()`
            # returns None both when the target is not in the newest frame
            # and when the feed has gone stale (producer quiet longer than
            # DETECTION_STALE_AFTER_S) -- a control loop must treat "I do
            # not know where it is" the same either way, but the two are
            # reported distinctly below so a dead loop is never mistaken
            # for an out-of-frame target again.
            target = self.detection_feed.get(target_shape_type)

            res_w, res_h = self.camera.get_resolution()
            center_x, center_y = res_w / 2.0, res_h / 2.0

            if not target:
                feed_stale = self.detection_feed.is_stale()
                feed_age_s = self.detection_feed.age_s()
                now = time.monotonic()
                if now - last_log >= _CENTERING_LOG_INTERVAL_S:
                    last_log = now
                    if not feed_stale:
                        why = " (feed canli, hedef bu karede yok)"
                    elif feed_age_s is None:
                        # The detection loop has never published at all --
                        # a different failure from "it went quiet", and the
                        # more alarming of the two.
                        why = " -- VISION FEED HIC VERI URETMEDI"
                    else:
                        why = f" -- VISION FEED STALE (age={feed_age_s:.2f}s)"
                    logger.warning(
                        f"[CENTERING] {target_shape_type} {attempt}/{max_attempts} hedef kayboldu{why}")
                # ADR-010 P1: below LOW_ALT_VISION_LIMIT_M, losing the
                # target is EXPECTED (the shape has grown past what the
                # detector's fixed-vertex-count gates can accept) and
                # holding altitude here is what stranded payload 1 at
                # 1.587 m. If a confirmed estimate exists, finish the
                # descent on it instead. Above the limit nothing changes:
                # a lost target up there means something is actually wrong,
                # and descending blind would be unsafe.
                if frozen_estimate is not None and last_seen_alt_m is not None \
                        and last_seen_alt_m < LOW_ALT_VISION_LIMIT_M \
                        and altitude_m < last_seen_alt_m:
                    reached = await self._open_loop_descend(
                        target_shape_type, frozen_estimate, altitude_m, last_seen_alt_m)
                    # Treat "reached the commanded release altitude" as
                    # success: the vertical goal was met and the lateral
                    # position is the best that is knowable without vision.
                    # Reporting failure here would send the caller into a
                    # retry that cannot possibly see anything either.
                    return abs(reached - altitude_m) <= PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M

                # Keep streaming a hold setpoint rather than going silent --
                # a silent gap here is exactly what lets PX4 fall back out of
                # Offboard mid-pursuit.
                #
                # PHASE 12 Q1: the hold is now DECELERATED under the rate
                # limit for the first TARGET_LOSS_GRACE_FRAMES frames rather
                # than commanded to zero on the spot. V1'''' hard-braked from
                # ~1.3 m/s to zero four separate times, every one of them a
                # single-frame dropout, and then had to re-accelerate from
                # standstill when the target reappeared on the very next
                # frame. A convergence stop still zeroes immediately (that is
                # a different event, and coasting past a converged target is
                # exactly the post-lock drift we measure) -- only this
                # transient-loss path ramps.
                lost_frames += 1
                await self._send_setpoint(
                    0.0, 0.0, 0.0,
                    immediate_stop=lost_frames > TARGET_LOSS_GRACE_FRAMES)
                self._publish_centering_step(
                    target_shape_type, attempt, max_attempts, target_seen=False,
                    feed_stale=feed_stale, feed_age_s=feed_age_s, altitude_m=altitude_m,
                    lost_frames=lost_frames)
                # ADR-008 B1: was asyncio.sleep(0.5) -- a slower cadence in
                # exactly the situation that most needs a fast one. It also
                # put the setpoint stream at 2x PX4's ~500ms Offboard
                # timeout while the target was missing, and (because this
                # branch never called get_global_position()) starved the
                # flight heartbeat into a DEGRADED<->STALE flap for the
                # whole 77s of the 2026-08-16 failed centering.
                await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)
                continue

            intrinsics = default_camera_intrinsics()
            # ADR-010 P1: centre source is altitude-dependent -- moment
            # centre normally, bbox centre below LOW_ALT_VISION_LIMIT_M.
            # Read the altitude BEFORE choosing, so the choice is made on
            # this tick's altitude rather than the previous one's.
            try:
                cur_lat, cur_lon, current_alt = await self.flight.get_global_position()
            except TelemetryStale as e:
                return self._abort_on_stale("CENTERING", target_shape_type, e)
            target_cx, target_cy = self.target_center_px(target, current_alt)
            # A2: the aim point is the PAYLOAD's position in frame, not the
            # frame centre. Everything downstream -- the P-law, the frozen
            # estimate, the reported offsets -- then works on the residual
            # between the target and where the payload actually is, which is
            # the error that decides where the body lands.
            # PHASE 13 D3: the aim offset NO LONGER biases the vision error.
            # Biasing it held the target 0.28 m off-centre through the whole
            # low-altitude descent, which pushed a shape that already fills
            # the frame further into the edge and corrupted the very
            # measurement the correction depends on -- measured: the frozen
            # estimate landed at east -0.640 m where the aim point was
            # -0.28 m, a 0.36 m perception bias, and the net offset did not
            # improve at all. The target is kept CENTRED while it is being
            # measured; the mount offset is applied afterwards, as a pure
            # translation of the hold point (see descend_to_release).
            aim_dx, aim_dy = 0.0, 0.0
            error_x = target_cx - (center_x + aim_dx)
            error_y = target_cy - (center_y + aim_dy)

            # Downward-facing camera (x500_mono_cam_down): image "up" is
            # aligned with body-forward. Target below center (error_y > 0)
            # is physically behind the vehicle -> negative forward_m_s;
            # target right of center (error_x > 0) is physically to the
            # vehicle's right -> positive right_m_s. Sign convention to be
            # confirmed against the real camera mount during physical
            # testing (see kp_horizontal/kp_vertical TODO above) -- the
            # part that was actually missing is that a command is sent at
            # all, every iteration, not just computed and discarded.
            error_x_norm = error_x / center_x if center_x else 0.0
            error_y_norm = error_y / center_y if center_y else 0.0

            alt_error = current_alt - altitude_m
            lost_frames = 0

            # ADR-010 P1: this tick HAS a committed detection, so record
            # where the target actually is. Only committed samples are
            # frozen -- an extrapolation would be exactly the kind of guess
            # an open-loop descent must not be built on.
            estimate = self._freeze_target_estimate(
                error_x, error_y, current_alt, res_w, res_h, cur_lat, cur_lon)
            if estimate is not None:
                frozen_estimate = estimate
                last_seen_alt_m = current_alt
                # D3: kept so the caller can translate it by the mount
                # vector after this loop converges, without re-deriving it.
                self._last_frozen_estimate = estimate

            # Operator precision requirement (2026-08-13 revision): ±0.01
            # normalized in both axes, replacing the previous raw-pixel
            # threshold -- and now also gated on altitude, since this loop
            # actually commands descent/climb.
            if (abs(error_x_norm) < self.tolerance_x and abs(error_y_norm) < self.tolerance_y
                    and abs(alt_error) < alt_tolerance_m):
                logger.info(f"Merkezleme tamamlandi. (attempt {attempt}/{max_attempts}, "
                            f"dx={error_x:+.0f}px dy={error_y:+.0f}px "
                            f"ex={error_x_norm:+.4f} ey={error_y_norm:+.4f} "
                            f"alt_err={alt_error:+.2f}m)")
                self._publish_centering_step(
                    target_shape_type, attempt, max_attempts, target_seen=True,
                    feed_stale=False, feed_age_s=self.detection_feed.age_s(), altitude_m=altitude_m,
                    dx_px=error_x, dy_px=error_y, ex_norm=error_x_norm, ey_norm=error_y_norm,
                    alt_error_m=alt_error, converged=True,
                    target_px=target.center_px, center_px=(center_x, center_y),
                    ground_distance_m=self._ground_distance_m(error_x, error_y, current_alt, res_w, res_h),
                    offsets=self._offset_measurements(target, error_x, error_y, current_alt, res_w, res_h))
                converged = True
                break

            right_m_s = _clamp(error_x_norm * self.kp_horizontal * MAX_CENTERING_SPEED_M_S, MAX_CENTERING_SPEED_M_S)
            forward_m_s = _clamp(-error_y_norm * self.kp_vertical * MAX_CENTERING_SPEED_M_S, MAX_CENTERING_SPEED_M_S)
            # ADR-009 S1/S2 -- applied to the two LATERAL axes only.
            # Altitude keeps pure proportional control: its own tolerance
            # (0.3m) is 20x looser than the lateral one, so it never reached
            # the dead zone that stalled the lateral axes. Each axis gets
            # its own floor because they normalize by different half-axes
            # (640px vs 480px), so their ground tolerance bands differ.
            focal_px = intrinsics.scaled_to(res_w, res_h).focal_px if intrinsics else None
            floor_x = floor_speed_m_s(self.tolerance_x, center_x, current_alt, focal_px)
            floor_y = floor_speed_m_s(self.tolerance_y, center_y, current_alt, focal_px)
            right_m_s = _with_min_speed(right_m_s, error_x_norm, self.tolerance_x, floor_x)
            forward_m_s = _with_min_speed(forward_m_s, error_y_norm, self.tolerance_y, floor_y)
            # NED down is positive-downward; alt_error > 0 means "too high"
            # (current > target) so a positive down_m_s (descend) is correct.
            down_m_s = _clamp(alt_error * self.kp_altitude, MAX_CENTERING_SPEED_M_S)

            # ADR-008 B1: per-iteration observability. Before this the loop
            # logged only "hedefine merkezleniyor" / "hedef kayboldu" /
            # "tamamlandi" -- there was no way at all to see whether the
            # offsets were shrinking, how many attempts had been spent, or
            # what was actually being commanded, which is why an 82s
            # centering was indistinguishable from a hung one.
            ground_distance_m = self._ground_distance_m(error_x, error_y, current_alt, res_w, res_h)
            offsets = self._offset_measurements(target, error_x, error_y, current_alt, res_w, res_h)

            # ADR-010 P4: the distance cap needs the ground distance, so the
            # send moved below its computation. What is published is what
            # was ACTUALLY commanded after limiting -- CENTERING_STEP would
            # otherwise report a setpoint the vehicle never received, which
            # is the same class of lie as the frozen detection boxes.
            forward_m_s, right_m_s, down_m_s = await self._send_setpoint(
                forward_m_s, right_m_s, down_m_s)
            self._publish_centering_step(
                target_shape_type, attempt, max_attempts, target_seen=True,
                feed_stale=False, feed_age_s=self.detection_feed.age_s(), altitude_m=altitude_m,
                dx_px=error_x, dy_px=error_y, ex_norm=error_x_norm, ey_norm=error_y_norm,
                alt_error_m=alt_error, forward_m_s=forward_m_s, right_m_s=right_m_s, down_m_s=down_m_s,
                target_px=target.center_px, center_px=(center_x, center_y),
                ground_distance_m=ground_distance_m, offsets=offsets)
            now = time.monotonic()
            if now - last_log >= _CENTERING_LOG_INTERVAL_S:
                last_log = now
                dist_note = f" d={ground_distance_m:.1f}m" if ground_distance_m is not None else ""
                logger.info(
                    f"[CENTERING] {target_shape_type} {attempt}/{max_attempts} "
                    f"dx={error_x:+.0f}px dy={error_y:+.0f}px{dist_note} "
                    f"ex={error_x_norm:+.4f} ey={error_y_norm:+.4f} (tol +/-{self.tolerance_x:.3f}) "
                    f"alt_err={alt_error:+.2f}m -> fwd={forward_m_s:+.2f} right={right_m_s:+.2f} "
                    f"down={down_m_s:+.2f} m/s")

            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)

        # Explicit stop -- always sent, converged or not, so no residual
        # velocity command carries into whatever runs next. ADR-010 P4: an
        # explicit zero is exempt from the rate limit (see SetpointLimiter),
        # so this still stops on the tick it is issued.
        await self._send_setpoint(0.0, 0.0, 0.0)

        if converged:
            self._publish("CENTERING_CONVERGED", target_shape_type, data={"shape_type": target_shape_type, "altitude_m": altitude_m})
            return True

        logger.error(f"Merkezleme zaman asimina ugradi! ({max_attempts} deneme / "
                     f"{self.budget_s(start_alt, altitude_m):.1f}s butcesi doldu)")
        self._publish("CENTERING_TIMED_OUT", target_shape_type, severity=Severity.WARN,
                      data={"shape_type": target_shape_type, "altitude_m": altitude_m,
                            "max_attempts": max_attempts,
                            "budget_s": self.budget_s(start_alt, altitude_m)})
        return False

    # ------------------------------------------------------------------
    # ADR-010 P1: low-altitude vision limit, centre source, hybrid descent
    # ------------------------------------------------------------------
    @staticmethod
    def target_center_px(target, current_alt_m) -> tuple:
        """Which point on the detection the controller steers at.

        Above LOW_ALT_VISION_LIMIT_M: the detector's own centre (HSV's
        contour-moment centre) -- unchanged, and what every prior run used.

        Below it: the BOUNDING-BOX centre. Measured divergence between the
        two grows with apparent size (3.51 px mean at 15 m, 77.99 px mean /
        163.5 px max at 0.45 m, where the shape spans 814 px of a 1280 px
        frame). The moment centre is the unstable one: once the blob clips
        the frame edge, its moments are taken over the visible part only, so
        the "centre" slides toward whatever is still in view while the true
        centre has not moved. The bbox centre degrades far more gracefully
        under the same clipping.

        This is a change of MEASUREMENT, not of control: the same P-law and
        the same tolerances act on whatever this returns."""
        if (not LOW_ALT_BBOX_CENTER or current_alt_m is None
                or current_alt_m >= LOW_ALT_VISION_LIMIT_M or not target.bbox_px):
            return tuple(target.center_px)
        x1, y1, x2, y2 = target.bbox_px
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    def _freeze_target_estimate(self, dx_px: float, dy_px: float, current_alt_m: float,
                                res_w: int, res_h: int, lat: float, lon: float):
        """The last CONFIRMED ground position to descend on, as GPS.

        Vehicle GPS + the pixel offset back-projected to metres on the
        ground. Only ever called from a tick where the detector actually
        committed a detection, so it is a measured position and not an
        extrapolation.

        A2: `dx_px`/`dy_px` are the RESIDUAL to the aim point, so with a
        payload mount offset in play this returns where the VEHICLE should
        be for the payload to sit on the target, not the target's own
        centre. The body->NED rotation below is therefore also what carries
        the mount vector into the ground frame -- applied once, here, and
        inherited by the open-loop descent for free. Returns None when the intrinsics or altitude are
        unusable -- in which case there is nothing to descend on and the
        caller must keep holding, which is the old behaviour."""
        intrinsics = default_camera_intrinsics()
        if intrinsics is None or current_alt_m is None or current_alt_m <= 0.0:
            return None
        focal = intrinsics.scaled_to(res_w, res_h).focal_px
        if not focal:
            return None
        # Downward camera: image +y is body-forward, image +x is body-right
        # (the same convention the velocity commands below use). Metres of
        # ground per pixel is alt/focal.
        m_per_px = current_alt_m / focal
        # T2: the pixel offset is measured from the CAMERA, which sits
        # CAMERA_LEVER_ARM_BODY_M forward of the body origin the GPS
        # reports. Adding the offset straight to the vehicle's GPS put every
        # frozen estimate 0.35 m aft of the real target, and the payload
        # duly landed 0.25-0.31 m south on every flight measured.
        lever_forward, lever_right = CAMERA_LEVER_ARM_BODY_M
        forward_m = -dy_px * m_per_px + lever_forward
        right_m = dx_px * m_per_px + lever_right
        yaw_rad = math.radians(self._last_yaw_deg or 0.0)
        # Body forward/right -> north/east.
        north_m = forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)
        east_m = forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)
        d_lat = north_m / 111320.0
        d_lon = east_m / (111320.0 * max(0.1, math.cos(math.radians(lat))))
        return {
            "lat": lat + d_lat, "lon": lon + d_lon,
            "from_alt_m": round(current_alt_m, 3),
            "dx_px": round(dx_px, 1), "dy_px": round(dy_px, 1),
            "offset_cm": round(math.hypot(dx_px, dy_px) * m_per_px * 100.0, 2),
        }

    async def _open_loop_descend(self, shape_type: str, estimate: dict, altitude_m: float,
                                 lost_at_alt_m: float,
                                 hold_tolerance_m: float = None) -> float:
        """ADR-010 P1: finish the descent without vision.

        Why this exists -- V1''' payload 1: the hexagon was lost at 1.63 m,
        the target-lost branch commanded zero velocity to hold, and the
        descent simply stopped. The servo fired at 1.587 m instead of
        0.45 m. Payload 2 only reached 0.407 m because its triangle
        happened to stay visible to 0.47 m. Release altitude was decided by
        which shape it was, not by the mission.

        "Open loop" is only true of VISION. Position is still closed-loop:
        the frozen estimate is a fixed GPS point and every tick steers at it
        using live GPS, so drift is corrected the whole way down. What is no
        longer corrected is the target's position estimate itself, which
        stopped improving the moment the detector stopped committing.

        Returns the altitude actually reached."""
        logger.warning(
            "[LOW_ALT_OPEN_LOOP_DESCENT] %s: goruntu %.2fm'de kayboldu -- dondurulmus "
            "kestirime (%.7f, %.7f, offset=%.1fcm @ %.2fm) gore %.2fm'ye alcalmaya devam.",
            shape_type, lost_at_alt_m, estimate["lat"], estimate["lon"],
            estimate["offset_cm"], estimate["from_alt_m"], altitude_m)
        self._publish("LOW_ALT_OPEN_LOOP_DESCENT", shape_type, severity=Severity.WARN,
                      data={"shape_type": shape_type, "vision_lost_at_alt_m": round(lost_at_alt_m, 3),
                            "release_target_alt_m": altitude_m, "estimate": estimate,
                            "low_alt_vision_limit_m": LOW_ALT_VISION_LIMIT_M})

        deadline = time.monotonic() + LOW_ALT_OPEN_LOOP_TIMEOUT_S
        reached = lost_at_alt_m
        while time.monotonic() < deadline:
            try:
                lat, lon, alt = await self.flight.get_global_position()
            except TelemetryStale as e:
                self._abort_on_stale("OPEN_LOOP_DESCENT", shape_type, e)
                return reached
            reached = alt
            alt_error = alt - altitude_m
            # D3 fix: the position error has to be known BEFORE the exit
            # test. Exiting on altitude alone meant that when this method
            # was handed a vehicle already at the release altitude -- which
            # is exactly the case after a mount-offset translation -- it
            # returned on its first iteration having never commanded the
            # lateral move. The translation was computed, logged, and never
            # flown: measured, 0 open-loop steps after AIM_OFFSET_APPLIED.
            north_m, east_m = gps_to_ned_delta(lat, lon, estimate["lat"], estimate["lon"])
            hold_error_m = math.hypot(north_m, east_m)
            # ADR-010 P1 (V1'''' fix): the exit band is the RELEASE tolerance
            # (0.05 m), NOT ALTITUDE_CONVERGENCE_TOLERANCE_M (0.30 m). The
            # 0.30 m figure is sized for staged approach steps, where landing
            # anywhere near 10 m or 5 m is fine. Used here it ended the
            # descent the moment the vehicle was within 0.30 m of the release
            # altitude: V1'''' payload 1 stopped at 0.744 m (0.294 m error,
            # just inside 0.30) and released out of band. The whole point of
            # this method is to hit 0.45 +/- 0.05.
            alt_ok = abs(alt_error) <= PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M
            hold_ok = hold_tolerance_m is None or hold_error_m <= hold_tolerance_m
            if alt_ok and hold_ok:
                break

            # Position hold on the frozen estimate, in NED.
            yaw_rad = math.radians(self._last_yaw_deg or 0.0)
            forward_m = north_m * math.cos(yaw_rad) + east_m * math.sin(yaw_rad)
            right_m = -north_m * math.sin(yaw_rad) + east_m * math.cos(yaw_rad)
            forward_m_s = _clamp(forward_m * self.kp_horizontal, MAX_CENTERING_SPEED_M_S)
            right_m_s = _clamp(right_m * self.kp_horizontal, MAX_CENTERING_SPEED_M_S)
            down_m_s = _clamp(alt_error * self.kp_altitude, MAX_CENTERING_SPEED_M_S)
            # Pure proportional control asymptotes, and the band is now
            # 0.05 m: at alt_error = 0.10 m it would ask for 0.05 m/s and
            # crawl the last stretch past the timeout. Same reasoning as the
            # ADR-009 S1 lateral floor, applied to the vertical axis for
            # this method only -- the staged-approach descent keeps pure
            # proportional control exactly as before.
            if alt_ok:
                # D3: already in the release band and only the lateral hold
                # is outstanding. Holding altitude while translating is the
                # point; the descent floor below would otherwise drive the
                # vehicle into the ground during a purely lateral move.
                down_m_s = 0.0
            elif abs(down_m_s) < LOW_ALT_OPEN_LOOP_MIN_DESCENT_M_S:
                down_m_s = math.copysign(LOW_ALT_OPEN_LOOP_MIN_DESCENT_M_S, alt_error)
            await self._send_setpoint(forward_m_s, right_m_s, down_m_s)
            self._publish("LOW_ALT_OPEN_LOOP_STEP", shape_type, severity=Severity.DEBUG,
                          data={"shape_type": shape_type, "altitude_m": round(alt, 3),
                                "alt_error_m": round(alt_error, 3),
                                "hold_error_m": round(hold_error_m, 3),
                                "hold_tolerance_m": hold_tolerance_m})
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)

        await self._send_setpoint(0.0, 0.0, 0.0)
        logger.info("[LOW_ALT_OPEN_LOOP_DESCENT] %s: %.3fm'ye ulasildi (hedef %.2fm).",
                    shape_type, reached, altitude_m)
        self._publish("LOW_ALT_OPEN_LOOP_DESCENT_DONE", shape_type,
                      data={"shape_type": shape_type, "reached_alt_m": round(reached, 3),
                            "release_target_alt_m": altitude_m,
                            "within_tolerance": abs(reached - altitude_m) <= PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M})
        return reached

    async def _send_setpoint(self, forward_m_s: float, right_m_s: float, down_m_s: float,
                             immediate_stop: bool = True) -> tuple:
        """ADR-010 P4: the ONE place a velocity setpoint leaves this
        controller, so the rate limit cannot be bypassed by a branch that
        forgets it. Returns what was actually commanded, which is what
        CENTERING_STEP reports -- the telemetry must show the commanded
        value, not the requested one.

        `immediate_stop=False` marks a TRANSIENT-LOSS hold, which is
        decelerated under the rate limit rather than zeroed on the spot;
        see SetpointLimiter.limit()."""
        f, r, d = self._limiter.limit(forward_m_s, right_m_s, down_m_s,
                                      immediate_stop=immediate_stop)
        await self.flight.set_velocity_body(f, r, d, 0.0)
        return f, r, d

    def _ground_distance_m(self, dx_px: float, dy_px: float, current_alt_m: float,
                           res_w: int, res_h: int):
        """Horizontal ground distance from the vehicle's nadir point to the
        target, for the CENTERING_STEP payload and the dashboard's `d=`
        label (operator request, 2026-08-16). Derived from the mono_cam
        SDF's own FOV -- see core/detection/camera_intrinsics.py.

        Read-only telemetry: nothing in the control law consumes it, and
        the loop still converges on normalized pixel error exactly as
        before. None when the intrinsics could not be resolved or the
        altitude is unusable, so the label is omitted rather than showing a
        number derived from a guess."""
        intrinsics = default_camera_intrinsics()
        if intrinsics is None:
            return None
        return intrinsics.scaled_to(res_w, res_h).ground_distance_m(dx_px, dy_px, current_alt_m)

    def _abort_on_stale(self, phase: str, subject: str, error: Exception) -> bool:
        """ADR-009 D1: telemetry is dead -- stop commanding immediately.

        Deliberately does NOT send a final zero-velocity setpoint: with the
        link down that command cannot arrive, and pretending otherwise would
        just delay the caller's fallback. The caller treats False the same
        way it treats any other non-convergence, so the existing abort ->
        return-to-start -> land-in-place chain takes over -- the difference
        is that it happens within TELEMETRY_STALE_AFTER_S rather than after
        a full navigation timeout spent flying on frozen numbers."""
        logger.error("[%s] %s: telemetri bayat -- komut gonderimi durduruldu: %s", phase, subject, error)
        self._publish("TELEMETRY_STALE_ABORT", str(error), severity=Severity.CRITICAL,
                      data={"phase": phase, "subject": subject, "error": str(error)})
        return False

    def _offset_measurements(self, target, dx_px: float, dy_px: float,
                             alt_m: float, res_w: int, res_h: int) -> dict:
        """Operator request (2026-08-17): per-axis offset in cm, plus the
        divergence between the two candidate target-centre definitions.

        The controller centres on HSVContourDetector's contour-MOMENT
        centre. The bounding-box centre is the obvious alternative and they
        are not the same point for an asymmetric or partially-occluded
        blob -- and the gap grows in pixels as the shape fills more of the
        frame on descent. Reporting both on the same frame is what makes
        that measurable instead of assumed. Pure telemetry: the control law
        keeps using the moment centre exactly as before."""
        intrinsics = default_camera_intrinsics()
        if intrinsics is None or alt_m is None or alt_m <= 0:
            return {}
        focal = intrinsics.scaled_to(res_w, res_h).focal_px
        px_to_cm = alt_m * 100.0 / focal

        x1, y1, x2, y2 = target.bbox_px
        bbox_cx, bbox_cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        sep_x = bbox_cx - target.center_px[0]
        sep_y = bbox_cy - target.center_px[1]
        sep_px = (sep_x ** 2 + sep_y ** 2) ** 0.5
        bbox_w, bbox_h = abs(x2 - x1), abs(y2 - y1)

        return {
            "dx_cm": round(dx_px * px_to_cm, 2),
            "dy_cm": round(dy_px * px_to_cm, 2),
            "offset_cm": round((dx_px ** 2 + dy_px ** 2) ** 0.5 * px_to_cm, 2),
            "px_to_cm": round(px_to_cm, 4),
            # target-centre source comparison (moment vs bbox)
            "moment_center_px": [round(target.center_px[0], 1), round(target.center_px[1], 1)],
            "bbox_center_px": [round(bbox_cx, 1), round(bbox_cy, 1)],
            "center_sep_px": round(sep_px, 2),
            "center_sep_cm": round(sep_px * px_to_cm, 2),
            "bbox_w_px": round(bbox_w, 1), "bbox_h_px": round(bbox_h, 1),
        }

    def _publish_centering_step(self, shape_type: str, attempt: int, max_attempts: int, *,
                                target_seen: bool, feed_stale: bool, feed_age_s,
                                altitude_m: float,
                                dx_px: float = None, dy_px: float = None,
                                ex_norm: float = None, ey_norm: float = None,
                                alt_error_m: float = None,
                                forward_m_s: float = 0.0, right_m_s: float = 0.0, down_m_s: float = 0.0,
                                converged: bool = False,
                                target_px=None, center_px=None,
                                ground_distance_m=None, offsets=None,
                                lost_frames: int = 0) -> None:
        """One structured event per control iteration (ADR-008 B1). DEBUG
        severity: it belongs in the JSONL timeline and on the dashboard, not
        on the console -- the throttled INFO line in the loop covers that.

        `converged` is the controller's OWN convergence flag (the same
        expression that ends the loop) -- the dashboard's lock indicator
        reads it directly rather than re-deciding "locked" against a
        threshold of its own, which would be a second, drifting definition
        of the same thing. `center_px` is the exact image centre the
        controller used, so the overlay's crosshair cannot land anywhere
        else."""
        self._publish("CENTERING_STEP", severity=Severity.DEBUG, data={
            "shape_type": shape_type,
            "attempt": attempt,
            "max_attempts": max_attempts,
            "target_altitude_m": altitude_m,
            "target_seen": target_seen,
            "feed_stale": feed_stale,
            "feed_age_s": round(feed_age_s, 3) if feed_age_s is not None else None,
            "dx_px": dx_px, "dy_px": dy_px,
            "ex_norm": ex_norm, "ey_norm": ey_norm,
            "alt_error_m": alt_error_m,
            "ground_distance_m": round(ground_distance_m, 2) if ground_distance_m is not None else None,
            "target_px": list(target_px) if target_px is not None else None,
            "center_px": list(center_px) if center_px is not None else None,
            "setpoint": {"forward_m_s": forward_m_s, "right_m_s": right_m_s, "down_m_s": down_m_s},
            "converged": converged,
            # PHASE 12 Q1: consecutive frames without a committed target, so
            # a post-run analysis can tell a one-frame dropout (decelerated)
            # from a real loss (hard stop) instead of inferring it.
            "lost_frames": lost_frames,
            **(offsets or {}),
        })

    async def nudge_forward(self, distance_m: float, speed_m_s: float = 0.3) -> None:
        """Sabit bir hızda kısa bir süre ileri hareket ederek yaklaşık
        `distance_m` kadar yol alır (Görev 2 Rapor: yük bırakma öncesi
        '10 cm ileri hareket'). Süre-tabanlı bir tahmindir -- gerçek mesafe
        rüzgar/gecikme nedeniyle sapabilir; kritik değildir çünkü bu son
        adım zaten SERVO tetiklemesinden hemen önce gelir ve yük bırakma
        pozisyonu yalnızca kabaca bu kadar ileride olmalıdır.

        Diğer tüm metodlarla aynı sürekli-akış güvenliği: PX4 ~500ms
        setpoint'siz kalırsa Offboard'dan çıkar, bu yüzden tek seferlik bir
        komut değil, süre boyunca tekrarlanan bir akış gönderilir."""
        if distance_m <= 0 or speed_m_s <= 0:
            return
        duration_s = distance_m / speed_m_s
        logger.info(f"{distance_m}m ileri hareket ediliyor ({speed_m_s} m/s, {duration_s:.1f}s)...")
        self._publish("NUDGE_FORWARD_STARTED", data={"distance_m": distance_m, "speed_m_s": speed_m_s})

        deadline = asyncio.get_event_loop().time() + duration_s
        while asyncio.get_event_loop().time() < deadline:
            await self._send_setpoint(speed_m_s, 0.0, 0.0)
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)

        await self._send_setpoint(0.0, 0.0, 0.0)
        self._publish("NUDGE_FORWARD_DONE", data={"distance_m": distance_m})

    async def climb_to_altitude(self, target_altitude_m: float, timeout_s: float = 20.0) -> bool:
        """Görüntüden bağımsız dikey hareket -- go_to_and_center()'ın aksine
        hedefin görüntüde olmasını GEREKTİRMEZ. Yük bırakma sonrası (araç
        yere yakın, az önce ileri kaydı, hedef artık kare içinde
        olmayabilir) MISSION_ALTITUDE_M'e geri tırmanmak için kullanılır --
        go_to_and_center burada kullanılsaydı hedef görünmediği sürece
        sadece beklerdi ve zaman aşımına uğrardı, hiç tırmanmadan."""
        logger.info(f"{target_altitude_m}m irtifasina tirmaniliyor (goruntuden bagimsiz)...")
        self._publish("CLIMB_STARTED", data={"target_altitude_m": target_altitude_m})

        deadline = asyncio.get_event_loop().time() + timeout_s
        converged = False
        while asyncio.get_event_loop().time() < deadline:
            try:
                _, _, current_alt = await self.flight.get_global_position()
            except TelemetryStale as e:
                return self._abort_on_stale("CLIMB", f"{target_altitude_m}m", e)
            alt_error = current_alt - target_altitude_m
            if abs(alt_error) < ALTITUDE_CONVERGENCE_TOLERANCE_M:
                converged = True
                break
            down_m_s = _clamp(alt_error * self.kp_altitude, MAX_CENTERING_SPEED_M_S)
            await self._send_setpoint(0.0, 0.0, down_m_s)
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)

        await self._send_setpoint(0.0, 0.0, 0.0)
        self._publish("CLIMB_DONE" if converged else "CLIMB_TIMED_OUT",
                      data={"target_altitude_m": target_altitude_m},
                      severity=Severity.INFO if converged else Severity.WARN)
        return converged

    async def goto_global_position_and_wait(self, target_lat: float, target_lon: float,
                                              target_alt_m: float,
                                              timeout_s: float = GLOBAL_POSITION_NAV_TIMEOUT_S) -> bool:
        """Görev 2 Rapor (operatör revizyonu, 2026-08-13 "Mission Lifecycle"
        yeniden yapılandırması): Search tamamlandığında Payload Mission 1/2
        için kaydedilen GPS konumuna dönüş -- araç o an ikinci hedefin
        yakınında olabilir, kaydedilen konumda DEĞİL. Görüntüden bağımsız
        (climb_to_altitude gibi): hedef şeklin o an kamerada görünmesi
        gerekmez, yalnızca GPS mesafesi kullanılır.

        BUG FIX (regression investigation, 2026-08-13): this used to
        recompute gps_to_ned_delta(CURRENT, target) fresh every iteration
        and send that -- a value that shrinks toward (0,0) as the vehicle
        approaches -- straight into goto_position_ned(), which sends an
        ABSOLUTE local-NED setpoint (proven earlier this session: repeated
        identical PositionNedYaw setpoints hold the vehicle at one fixed
        point, not a moving one). Feeding a moving relative delta into an
        absolute-position API means the vehicle chases a different,
        physically meaningless point every iteration -- proven via live
        instrumentation to produce a chaotic, non-monotonic trajectory
        (distance swinging between ~1.5m and ~26m repeatedly) rather than a
        smooth approach. Compounding this, the old convergence check only
        looked at position, not velocity -- proven to fire while the
        vehicle was moving at ~11 m/s mid-flight through the target's 2m
        radius, coasting ~10-25m past afterward with nothing correcting it.

        Fixed by restoring the two principles the prior (pre-Mission-
        Lifecycle-revision) codebase's proven-working equivalent always
        used (.scripts/olds/v32/mission.py::_state_return_home, used only
        as a behavioral reference -- not copied, not reintroduced as a
        dependency): (1) the target's absolute local-NED position is
        computed ONCE, by adding a GPS-derived delta to a single
        get_position_ned() snapshot, and sent unchanged on every
        iteration -- not recomputed from a moving current position; (2)
        convergence requires 3D velocity magnitude to also be below
        GPS_POSITION_VELOCITY_TOLERANCE_M_S, not position alone. No new
        navigation framework introduced -- still goto_position_ned(),
        still this same function's existing structure."""
        logger.info(f"Kayitli GPS konumuna gidiliyor: lat={target_lat}, lon={target_lon}, alt={target_alt_m}m")
        self._publish("GLOBAL_POSITION_NAV_STARTED",
                      data={"target_lat": target_lat, "target_lon": target_lon, "target_alt_m": target_alt_m})

        # Fixed absolute local-NED target, computed once -- see BUG FIX note
        # above. start_n/start_e are already in PX4's true local-NED frame
        # (get_position_ned()); adding the GPS-derived delta translates
        # that frame's origin-relative coordinates to the target's position
        # in the SAME frame, without ever needing to know the EKF origin's
        # own GPS coordinate directly.
        try:
            start_lat, start_lon, _ = await self.flight.get_global_position()
            start_n, start_e, _ = await self.flight.get_position_ned()
        except TelemetryStale as e:
            return self._abort_on_stale("GLOBAL_POSITION_NAV", f"{target_lat},{target_lon}", e)
        delta_n, delta_e = gps_to_ned_delta(start_lat, start_lon, target_lat, target_lon)
        target_n, target_e = start_n + delta_n, start_e + delta_e

        deadline = asyncio.get_event_loop().time() + timeout_s
        converged = False
        while asyncio.get_event_loop().time() < deadline:
            try:
                current_lat, current_lon, current_alt = await self.flight.get_global_position()
                vel_n, vel_e, vel_d = await self.flight.get_velocity_ned()
            except TelemetryStale as e:
                # ADR-009 D1: this is the exact loop that flew its full 60s
                # timeout against a frozen position on 2026-08-16. It must
                # now stop within one stale-guard period instead.
                return self._abort_on_stale("GLOBAL_POSITION_NAV", f"{target_lat},{target_lon}", e)
            distance_m = haversine_distance_m(current_lat, current_lon, target_lat, target_lon)
            alt_error = current_alt - target_alt_m
            speed = (vel_n ** 2 + vel_e ** 2 + vel_d ** 2) ** 0.5

            if (distance_m < GPS_POSITION_CONVERGENCE_TOLERANCE_M
                    and abs(alt_error) < ALTITUDE_CONVERGENCE_TOLERANCE_M
                    and speed < GPS_POSITION_VELOCITY_TOLERANCE_M_S):
                converged = True
                break

            try:
                yaw = await self.flight.get_yaw_deg()
            except TelemetryStale as e:
                return self._abort_on_stale("GLOBAL_POSITION_NAV", f"{target_lat},{target_lon}", e)
            await self.flight.goto_position_ned(target_n, target_e, -target_alt_m, yaw)
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)

        self._publish("GLOBAL_POSITION_NAV_CONVERGED" if converged else "GLOBAL_POSITION_NAV_TIMED_OUT",
                      data={"target_lat": target_lat, "target_lon": target_lon},
                      severity=Severity.INFO if converged else Severity.WARN)
        return converged

    async def hover_and_confirm(self, duration_s: float = HOVER_DURATION_S) -> None:
        """flight.hold_position(duration_s) çağırır — GPS/görüntü stabilizasyonu ve konum
        doğrulaması için (Bölüm 9). hold_position() artık süre boyunca setpoint
        akışını KENDİSİ sürdürüyor (bkz. MavsdkBackendBase.hold_position) -- bu yüzden
        burada ayrıca sessizce uyumaya gerek yok; o da PX4'ün Offboard'dan
        düşmesine yol açan sessiz bir boşluktu."""
        logger.info(f"{duration_s} saniye hover yapiliyor (Konum dogrulama)...")
        self._publish("HOVER_STARTED", data={"duration_s": duration_s})
        await self.flight.hold_position(duration_s)
        self._publish("HOVER_CONFIRMED", data={"duration_s": duration_s})
