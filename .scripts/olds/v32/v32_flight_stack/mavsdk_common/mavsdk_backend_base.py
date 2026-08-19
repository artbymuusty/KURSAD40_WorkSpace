import asyncio
import logging
import time
from typing import Optional, Tuple
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw
from mavsdk.mission import MissionItem, MissionPlan
from core.interfaces.i_flight_backend import IFlightBackend, TelemetryStale
from core.config.parameters import (
    OFFBOARD_SETPOINT_INTERVAL_S,
    TELEMETRY_FIRST_SAMPLE_TIMEOUT_S,
    TELEMETRY_HEARTBEAT_PUBLISH_INTERVAL_S,
    TELEMETRY_STALE_AFTER_FLIGHT_MODE_S,
    TELEMETRY_STALE_AFTER_S,
    TELEMETRY_STREAM_RATE_HZ,
    TELEMETRY_STREAM_RATE_REPORT_INTERVAL_S,
)
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger(__name__)

# ADR-004 §9.1: MAVSDK exposes drone.telemetry.speed_m_s()-equivalent cruise
# speed for MissionItem generation; a fixed default is used until the team
# fills NORMAL_MISSION_SPEED_M_S in parameters.py (still None/TODO there).
_DEFAULT_MISSION_SPEED_M_S = 5.0


class _StreamCache:
    """ADR-008 B0: one background-subscribed MAVSDK telemetry stream's latest
    value, plus enough bookkeeping to prove the stream is actually running at
    the rate we asked PX4 for (`observed_hz`).

    `value` is replaced by whole-object assignment from a single producer
    coroutine and only ever read (never mutated) by consumers -- all on the
    same single-threaded asyncio loop, so no lock is needed: assignment and
    read never interleave mid-instruction, only at await points.
    """

    __slots__ = ("name", "value", "ts", "samples", "_window_start", "_window_samples", "observed_hz")

    def __init__(self, name: str):
        self.name = name
        self.value = None
        self.ts: float = 0.0
        self.samples: int = 0
        self._window_start: float = 0.0
        self._window_samples: int = 0
        self.observed_hz: float = 0.0

    def update(self, value, now: Optional[float] = None) -> None:
        now = now or time.time()
        self.value = value
        self.ts = now
        self.samples += 1
        if self._window_start == 0.0:
            self._window_start = now
        self._window_samples += 1
        elapsed = now - self._window_start
        if elapsed >= TELEMETRY_STREAM_RATE_REPORT_INTERVAL_S:
            self.observed_hz = self._window_samples / elapsed
            self._window_start = now
            self._window_samples = 0

    @property
    def has_sample(self) -> bool:
        return self.ts > 0.0

    def age_s(self, now: Optional[float] = None) -> float:
        return (now or time.time()) - self.ts if self.has_sample else float("inf")

class MavsdkBackendBase(IFlightBackend):
    """
    Hem real_system hem gz_system için ortak MAVSDK uçuş kontrol mantığı.
    core/ bu sınıftan habersizdir, yalnızca adaptörler kullanır.

    `publisher` is optional (defaults to a no-op) so existing callers/tests
    that construct this without one keep working unchanged (ADR-004 §14).
    """
    def __init__(self, connection_string: str, publisher: EventPublisher = NULL_PUBLISHER):
        self.connection_string = connection_string
        self.drone = System()
        self.publisher = publisher
        # BUG FIX (runtime investigation, 2026-08-13): is_mission_finished()
        # used to open a brand-new `async for ... in
        # self.drone.mission.mission_progress()` subscription on every
        # single call, then take the first pushed value. Unlike
        # position/attitude/flight_mode (which PX4 streams continuously at
        # flight-control rate, so a fresh subscription's first value
        # arrives almost immediately), mission_progress is event-driven --
        # MAVSDK/PX4 only pushes a new value when the mission actually
        # advances (e.g. a waypoint is reached). A fresh subscription
        # started between two such events can block for many seconds
        # waiting for the next one. _search_and_engage_loop's own while
        # condition calls is_mission_finished() every iteration, so this
        # silently throttled the ENTIRE search loop -- including
        # TargetValidator's consecutive-frame counting -- down to the rate
        # of PX4 mission-progress events instead of the intended ~10Hz
        # detection-processing rate. Proven via a real Gazebo run: 633
        # VISION_FRAME_PROCESSED events (~10Hz, correct) against only 3
        # TRACK_STATE_UPDATED events over the same 46s search window --
        # continuous detections were never being fed to the validator often
        # enough to reach its 5-consecutive-frame threshold before the
        # underlying PX4 mission ended. Fixed the same way every other
        # telemetry field in this class should be (see get_flight_mode
        # etc., a broader but lower-severity instance of the same pattern
        # not touched here since it's not what caused the proven failure):
        # subscribe ONCE in the background, cache the latest value, let
        # is_mission_finished() return the cache instantly.
        self._mission_finished_cache: bool = False
        # ADR-010 R2: the live mission item index, so a resume can point PX4
        # explicitly at where the route actually is instead of relying on
        # start_mission()'s implicit resume state -- which PX4 silently
        # declined to act on on 2026-08-17.
        self._mission_current_index: int = 0

        # ADR-008 B0: the same "subscribe once in the background, serve from
        # cache" treatment, finally applied to the four streams the comment
        # above explicitly deferred ("a broader but lower-severity instance
        # of the same pattern not touched here"). It turned out NOT to be
        # lower-severity: `async for pos in self.drone.telemetry.position()`
        # per call returns the first value the stream pushes, and PX4's
        # default position rate is 1 Hz -- so every get_global_position()
        # blocked ~1s. CenteringController.go_to_and_center() calls it once
        # per iteration, which throttled the whole centering loop from its
        # designed 10 Hz (OFFBOARD_SETPOINT_INTERVAL_S) to ~1 Hz. Proven on
        # the 2026-08-16 21:04 run: 81 VEHICLE_TELEMETRY events at exactly
        # 1.000s spacing across an 82.0s centering window, i.e. one loop
        # iteration per second. At 1 Hz the setpoint stream is also 2x past
        # PX4's ~500ms Offboard timeout, and the HSV detector's
        # N-consecutive-frame streak can never hold across ~1s frame gaps.
        self._position = _StreamCache("position")
        self._position_velocity_ned = _StreamCache("position_velocity_ned")
        self._flight_mode = _StreamCache("flight_mode")
        self._attitude_euler = _StreamCache("attitude_euler")
        self._watchers: list = []
        self._last_heartbeat_publish: float = 0.0
        # ADR-009 D1: streams already reported stale, so the CRITICAL is
        # published once per episode rather than once per cache read.
        self._stale_reported: set = set()

    def _publish(self, code, message="", severity=Severity.INFO, category=Category.TELEMETRY, data=None):
        self.publisher.publish(Event(
            code=code, subsystem="MavsdkBackendBase", category=category, severity=severity,
            message=message, data=data or {},
        ))

    async def connect(self) -> None:
        logger.info(f"Drone'a bağlanılıyor: {self.connection_string}")
        await self.drone.connect(system_address=self.connection_string)

        logger.info("Drone bağlantısı bekleniyor...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                logger.info("Drone bağlandı!")
                self._publish("CONNECTED", f"connected to {self.connection_string}", category=Category.LIFECYCLE,
                              data={"connected": True})
                break

        # See _mission_finished_cache's own comment (__init__) for why this
        # runs once in the background instead of is_mission_finished()
        # subscribing fresh on every call.
        self._watchers.append(asyncio.ensure_future(self._mission_progress_watcher()))

        # ADR-008 B0. Rates are requested BEFORE subscribing so the very
        # first samples already arrive at the fast cadence, and the achieved
        # rate is logged/published later by _stream_rate_reporter() -- a
        # set_rate_* call PX4 silently declines must not look identical to
        # one it honoured (that is exactly how the 1 Hz position stream hid
        # for so long).
        await self._request_stream_rates()

        self._watchers.append(asyncio.ensure_future(self._position_watcher()))
        self._watchers.append(asyncio.ensure_future(self._position_velocity_ned_watcher()))
        self._watchers.append(asyncio.ensure_future(self._flight_mode_watcher()))
        self._watchers.append(asyncio.ensure_future(self._attitude_watcher()))
        self._watchers.append(asyncio.ensure_future(self._stream_rate_reporter()))

        # Bounded wait for the first samples that a caller could otherwise
        # read as a placeholder:
        #   position    -- the very first get_global_position() is the
        #                  CHECKPOINT_SAVE read; a (0,0,0) cache miss there
        #                  would silently record a garbage start/finish
        #                  checkpoint.
        #   flight_mode -- has no set_rate_* and is change-driven, so its
        #                  first push can lag the others. Measured on the
        #                  live SITL: position arrived in 1.26s while
        #                  flight_mode still read "UNKNOWN". Anything
        #                  branching on the mode straight after connect
        #                  (MasterMissionController._ensure_offboard, the
        #                  dashboard badge) would act on that placeholder.
        await self._await_first_sample(self._position, TELEMETRY_FIRST_SAMPLE_TIMEOUT_S)
        await self._await_first_sample(self._flight_mode, TELEMETRY_FIRST_SAMPLE_TIMEOUT_S)

    async def _request_stream_rates(self) -> None:
        """Ask PX4 for TELEMETRY_STREAM_RATE_HZ on every stream the centering
        loop depends on. Each is attempted independently: some PX4/MAVSDK
        combinations reject an individual set_rate_* while happily honouring
        the others, and that must degrade to "this one stream stays slow",
        not "no rate was requested at all"."""
        for name, setter in (
            ("position", "set_rate_position"),
            ("position_velocity_ned", "set_rate_position_velocity_ned"),
            ("attitude_euler", "set_rate_attitude_euler"),
        ):
            try:
                await getattr(self.drone.telemetry, setter)(TELEMETRY_STREAM_RATE_HZ)
                logger.info(f"[TELEMETRY] {name} rate {TELEMETRY_STREAM_RATE_HZ:.1f} Hz istendi.")
            except Exception as e:  # noqa: BLE001 -- a declined rate is degraded, not fatal
                logger.warning(f"[TELEMETRY] {setter}({TELEMETRY_STREAM_RATE_HZ}) reddedildi: {e}")
                self._publish("TELEMETRY_RATE_REJECTED", f"{name}: {e}", severity=Severity.WARN,
                              data={"stream": name, "requested_hz": TELEMETRY_STREAM_RATE_HZ, "error": str(e)})
        # flight_mode has no set_rate_* in MAVSDK -- PX4 pushes it on change,
        # which is exactly the right cadence for a mode field. Cached the
        # same way so get_flight_mode() stops costing a stream round-trip.

    async def _await_first_sample(self, cache: _StreamCache, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if cache.has_sample:
                return True
            await asyncio.sleep(0.05)
        logger.warning(f"[TELEMETRY] {cache.name} akisindan {timeout_s:.0f}s icinde ilk ornek gelmedi.")
        self._publish("TELEMETRY_STREAM_SILENT", f"{cache.name}: no sample within {timeout_s:.0f}s",
                      severity=Severity.CRITICAL, data={"stream": cache.name, "timeout_s": timeout_s})
        return False

    async def _stream_watcher(self, cache: _StreamCache, stream_factory, on_sample=None) -> None:
        """Shared body for every background telemetry subscription: consume
        forever, cache the latest value, and reconnect after a stream hiccup
        instead of dying silently (same discipline as
        _mission_progress_watcher)."""
        while True:
            try:
                async for sample in stream_factory():
                    cache.update(sample)
                    if on_sample is not None:
                        on_sample(sample)
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001 -- a stream hiccup must never take the mission down
                logger.warning(f"[MAVSDK] {cache.name} stream hatasi, 1s icinde yeniden baglaniliyor: {e}")
                await asyncio.sleep(1.0)

    async def _position_watcher(self) -> None:
        await self._stream_watcher(self._position, lambda: self.drone.telemetry.position(),
                                   on_sample=self._publish_heartbeat_from_position)

    async def _position_velocity_ned_watcher(self) -> None:
        await self._stream_watcher(self._position_velocity_ned,
                                   lambda: self.drone.telemetry.position_velocity_ned())

    async def _flight_mode_watcher(self) -> None:
        await self._stream_watcher(self._flight_mode, lambda: self.drone.telemetry.flight_mode())

    async def _attitude_watcher(self) -> None:
        await self._stream_watcher(self._attitude_euler, lambda: self.drone.telemetry.attitude_euler())

    def _publish_heartbeat_from_position(self, pos) -> None:
        """ADR-008 B0: the Flight Backend heartbeat now originates from the
        position STREAM, not from whoever happens to call
        get_global_position(). Previously it was published inside that
        getter, so the backend's health depended entirely on the shape of
        the current mission loop -- and go_to_and_center()'s "target lost"
        branch, which never calls it, starved the heartbeat into a
        DEGRADED<->STALE flap for the whole 77s of the failed 2026-08-16
        centering. Throttled to TELEMETRY_HEARTBEAT_PUBLISH_INTERVAL_S so a
        10 Hz stream does not multiply the event log tenfold; that interval
        stays comfortably inside HealthMonitor's own
        FLIGHT_TELEMETRY_HEARTBEAT_INTERVAL_S window."""
        now = time.time()
        if now - self._last_heartbeat_publish < TELEMETRY_HEARTBEAT_PUBLISH_INTERVAL_S:
            return
        self._last_heartbeat_publish = now
        self._publish("VEHICLE_TELEMETRY", severity=Severity.DEBUG, data={
            "connected": True,
            "position": (pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m),
            "flight_mode": self._cached_flight_mode(),
        })

    async def _stream_rate_reporter(self) -> None:
        """Publishes what each stream ACTUALLY achieved, so a set_rate_*
        that PX4 accepted but did not honour is visible instead of being
        assumed."""
        caches = (self._position, self._position_velocity_ned, self._flight_mode, self._attitude_euler)
        while True:
            await asyncio.sleep(TELEMETRY_STREAM_RATE_REPORT_INTERVAL_S)
            rates = {c.name: round(c.observed_hz, 2) for c in caches}
            self._publish("TELEMETRY_STREAM_RATES", severity=Severity.DEBUG,
                          data={"requested_hz": TELEMETRY_STREAM_RATE_HZ, "observed_hz": rates})
            if self._position.observed_hz and self._position.observed_hz < TELEMETRY_STREAM_RATE_HZ * 0.5:
                logger.warning(f"[TELEMETRY] position akisi {self._position.observed_hz:.1f} Hz -- "
                               f"istenen {TELEMETRY_STREAM_RATE_HZ:.1f} Hz'in cok altinda.")

    def _cached_flight_mode(self) -> str:
        return str(self._flight_mode.value) if self._flight_mode.has_sample else "UNKNOWN"

    async def _mission_progress_watcher(self) -> None:
        while True:
            try:
                async for progress in self.drone.mission.mission_progress():
                    self._publish("MISSION_PROGRESS", severity=Severity.DEBUG,
                                  data={"progress_current": progress.current, "progress_total": progress.total})
                    # total == 0 means "no mission was ever uploaded", not
                    # "the mission is complete" -- treating them the same
                    # is exactly the bug class ADR-005 §5 closed. A real
                    # mission is "finished" only once it has actually had
                    # items and completed.
                    self._mission_current_index = progress.current
                    self._mission_finished_cache = (
                        progress.total > 0 and progress.current == progress.total
                    )
            except Exception as e:  # noqa: BLE001 -- a stream hiccup must never take the mission down
                logger.warning(f"[MAVSDK] mission_progress stream hatasi, 1s icinde yeniden baglaniliyor: {e}")
                await asyncio.sleep(1.0)
                
    async def get_health_summary(self) -> dict:
        """ADR-004 §9.1: EKF/GPS-fix/pre-arm-check status is available from
        MAVSDK today (drone.telemetry.health()) and was previously never
        read anywhere in this codebase -- meaning a PX4 arm rejection
        surfaced only as a bare 'COMMAND_DENIED' string with zero indication
        of *which* pre-arm check actually failed. This closes that gap."""
        async for health in self.drone.telemetry.health():
            return {
                "is_gyrometer_calibration_ok": health.is_gyrometer_calibration_ok,
                "is_accelerometer_calibration_ok": health.is_accelerometer_calibration_ok,
                "is_magnetometer_calibration_ok": health.is_magnetometer_calibration_ok,
                "is_local_position_ok": health.is_local_position_ok,
                "is_global_position_ok": health.is_global_position_ok,
                "is_home_position_ok": health.is_home_position_ok,
                "is_armable": health.is_armable,
            }
        return {}

    async def arm(self) -> None:
        logger.info("Drone arm ediliyor...")
        try:
            await self.drone.action.arm()
        except Exception as e:
            health = await self.get_health_summary()
            failing_checks = [k for k, v in health.items() if k != "is_armable" and v is False]
            logger.error(f"Arm reddedildi: {e} -- health: {health}")
            self._publish("ARM_REJECTED", f"{e} -- failing pre-arm checks: {failing_checks or 'unknown (see health data)'}",
                          severity=Severity.CRITICAL, category=Category.LIFECYCLE,
                          data={"error": str(e), "health": health, "failing_checks": failing_checks})
            raise
        self._publish("ARMED", "vehicle armed", category=Category.LIFECYCLE, data={"armed": True})

    async def takeoff(self, target_altitude_m: float) -> None:
        logger.info(f"Kalkış yapılıyor (Hedef: {target_altitude_m}m)...")
        await self.drone.action.set_takeoff_altitude(target_altitude_m)
        await self.drone.action.takeoff()
        self._publish("TAKEOFF_ISSUED", f"target_altitude_m={target_altitude_m}", category=Category.LIFECYCLE,
                      data={"target_altitude_m": target_altitude_m})
        
    async def land(self) -> None:
        logger.info("İniş yapılıyor...")
        await self.drone.action.land()

    async def send_status_text(self, text: str, severity: str = "INFO") -> bool:
        """W4: surface an operator-visible line in QGC's message bubble.

        Best-effort by contract (see IFlightBackend.send_status_text): the
        ServerUtility plugin is not available on every MAVSDK build or every
        autopilot, and a payload release must never fail because a cosmetic
        message could not be delivered. Every failure path returns False and
        logs at DEBUG -- the same text is always written to the mission log
        and published as an event by the caller, so nothing is lost if this
        does not land."""
        try:
            from mavsdk.server_utility import StatusTextType
            level = {
                "INFO": StatusTextType.INFO,
                "WARNING": StatusTextType.WARNING,
                "CRITICAL": StatusTextType.CRITICAL,
            }.get(severity.upper(), StatusTextType.INFO)
            # MAVLink STATUSTEXT is capped at 50 characters.
            await self.drone.server_utility.send_status_text(level, text[:50])
            return True
        except Exception as e:  # noqa: BLE001 -- cosmetic channel, never fatal
            logger.debug("STATUSTEXT gonderilemedi (yoksayiliyor): %s", e)
            return False
        
    async def start_offboard(self) -> None:
        logger.info("Offboard başlatılıyor...")
        initial_setpoint = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        await self.drone.offboard.set_velocity_body(initial_setpoint)
        await self.drone.offboard.start()
        
    async def stop_offboard(self) -> None:
        logger.info("Offboard durduruluyor...")
        await self.drone.offboard.stop()
        
    async def goto_position_ned(self, north_m: float, east_m: float, down_m: float, yaw_deg: float) -> None:
        setpoint = PositionNedYaw(north_m, east_m, down_m, yaw_deg)
        await self.drone.offboard.set_position_ned(setpoint)

    async def goto_position_ned_and_hold(self, north_m: float, east_m: float, down_m: float,
                                          yaw_deg: float, duration_s: float) -> None:
        """GAP FIX (operator revision, 2026-08-13): Görev 3 fazları
        (pickup/transport/redrop/finish) previously called goto_position_ned()
        once and then asyncio.sleep() -- the exact single-shot-then-silence
        bug already fixed elsewhere in this file for hold_position(), just
        never applied here because these phases were, until now, entirely
        simulated placeholders. Streams the same setpoint at
        OFFBOARD_SETPOINT_INTERVAL_S for the whole duration so PX4 never
        sees a setpoint gap long enough to auto-exit Offboard."""
        setpoint = PositionNedYaw(north_m, east_m, down_m, yaw_deg)
        deadline = asyncio.get_event_loop().time() + duration_s
        while asyncio.get_event_loop().time() < deadline:
            await self.drone.offboard.set_position_ned(setpoint)
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)


    async def set_velocity_body(self, forward_m_s: float, right_m_s: float, down_m_s: float, yaw_rate_deg_s: float) -> None:
        setpoint = VelocityBodyYawspeed(forward_m_s, right_m_s, down_m_s, yaw_rate_deg_s)
        await self.drone.offboard.set_velocity_body(setpoint)
        
    async def hold_position(self, duration_s: float) -> None:
        """Offboard modunda sıfır hız vererek konum koruma.

        BUG FIX (operator-reported): this used to send exactly ONE
        zero-velocity setpoint and return immediately -- the caller
        (CenteringController.hover_and_confirm) then did a bare
        asyncio.sleep(duration_s) with nothing streamed to PX4 during it.
        PX4 auto-exits Offboard after ~500ms without a new setpoint, so the
        vehicle was falling out of Offboard mid-hover on every single
        engagement, regardless of whether go_to_and_center() had just
        fixed its own streaming gap. This now owns the full duration
        itself, streaming a hold setpoint at OFFBOARD_SETPOINT_INTERVAL_S
        the whole time."""
        deadline = asyncio.get_event_loop().time() + duration_s
        while asyncio.get_event_loop().time() < deadline:
            await self.set_velocity_body(0.0, 0.0, 0.0, 0.0)
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)
        
    def _fresh(self, cache: _StreamCache, stale_after_s: float = TELEMETRY_STALE_AFTER_S):
        """ADR-009 D1: the one place every cached getter checks freshness.

        Raises TelemetryStale rather than returning the frozen sample (see
        that exception's contract) and publishes it once per transition, so
        a wedged link is visible on the dashboard timeline instead of only
        inferable from a stalled mission."""
        if not cache.has_sample:
            self._publish_stale(cache, None)
            raise TelemetryStale(f"{cache.name}: no sample received yet")
        age = cache.age_s()
        if age > stale_after_s:
            self._publish_stale(cache, age)
            raise TelemetryStale(
                f"{cache.name}: last sample {age:.1f}s old (limit {stale_after_s:.1f}s) -- "
                "vehicle link is not delivering telemetry")
        self._stale_reported.discard(cache.name)
        return cache.value

    def _publish_stale(self, cache: _StreamCache, age) -> None:
        # Once per stale episode, not once per call: a 10Hz control loop
        # reading a dead cache would otherwise emit thousands of identical
        # CRITICALs while the operator is trying to read the timeline.
        if cache.name in self._stale_reported:
            return
        self._stale_reported.add(cache.name)
        logger.error("[TELEMETRY] %s akisi bayat (%s) -- komut gonderimi durduruluyor.",
                     cache.name, f"{age:.1f}s" if age is not None else "hic ornek yok")
        self._publish("TELEMETRY_STALE", f"{cache.name} stale", severity=Severity.CRITICAL,
                      data={"stream": cache.name, "age_s": round(age, 2) if age is not None else None,
                            "samples": cache.samples})

    async def get_position_ned(self) -> Tuple[float, float, float]:
        """ADR-008 B0: served from _position_velocity_ned_watcher's cache.
        Used once per goto_global_position_and_wait() to fix the absolute
        NED target, so a ~1s stall here delayed the start of every return-
        to-position navigation."""
        pos = self._fresh(self._position_velocity_ned)
        return (pos.position.north_m, pos.position.east_m, pos.position.down_m)

    async def get_velocity_ned(self) -> Tuple[float, float, float]:
        """BUG FIX (regression investigation, 2026-08-13): position_velocity_ned()
        already carries velocity alongside position, but get_position_ned()
        above discards it. Added so goto_global_position_and_wait() can
        gate its convergence condition on 3D speed, not position alone --
        see that method's own BUG FIX comment for the full root-cause
        proof (a position-only convergence check fired while the vehicle
        was moving at ~11 m/s through the target).

        ADR-008 B0: served from _position_velocity_ned_watcher's cache --
        this is read EVERY iteration of goto_global_position_and_wait(),
        the same per-call-subscription cost get_global_position() had."""
        pos = self._fresh(self._position_velocity_ned)
        return (pos.velocity.north_m_s, pos.velocity.east_m_s, pos.velocity.down_m_s)

    async def get_flight_mode(self) -> str:
        """ONBOARD (Mission) vs OFFBOARD -- the operator's single most
        useful at-a-glance signal for "is the vehicle following its
        pre-planned route right now, or is Gorev2Orchestrator actively
        flying it toward a target." Read from real MAVSDK telemetry, not
        inferred from which function we last called, so it stays correct
        even if PX4 rejects a requested mode change.

        ADR-008 B0: served from _flight_mode_watcher's cache. PX4 pushes
        this stream on change, so a fresh per-call subscription could block
        until the NEXT mode change -- and switch_to_offboard()'s
        confirmation loop polls it in a tight loop, which is precisely when
        no mode change is happening."""
        # Looser bound: flight_mode is change-driven, so quiet is normal.
        return str(self._fresh(self._flight_mode, TELEMETRY_STALE_AFTER_FLIGHT_MODE_S))

    async def get_global_position(self) -> Tuple[float, float, float]:
        """ADR-008 B0 (root cause 3 of the 2026-08-16 investigation): served
        from _position_watcher's cache instead of opening a fresh
        `telemetry.position()` subscription per call. The heartbeat publish
        moved to the watcher (see _publish_heartbeat_from_position) -- it
        was never really "the orchestrator's tick", it was "the position
        stream's tick", and tying it to the caller is what let it starve."""
        pos = self._fresh(self._position)
        return (pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m)

    async def get_yaw_deg(self) -> float:
        """ADR-008 B0: served from _attitude_watcher's cache -- read every
        iteration of goto_global_position_and_wait()."""
        return self._fresh(self._attitude_euler).yaw_deg

    def telemetry_stream_rates(self) -> dict:
        """Observed (not requested) Hz per cached stream -- for validation
        reporting and the dashboard, so "we asked for 10 Hz" is never
        mistaken for "we got 10 Hz"."""
        return {c.name: round(c.observed_hz, 2) for c in (
            self._position, self._position_velocity_ned, self._flight_mode, self._attitude_euler)}

    @staticmethod
    def _to_mission_items(waypoints: list, speed_m_s: float = _DEFAULT_MISSION_SPEED_M_S) -> list:
        """waypoints: list of (lat, lon, alt_rel_m). Builds real MAVSDK
        MissionItems -- BUG FIX (ADR-005 §5): the previous implementation
        ignored this argument entirely and always uploaded an empty
        MissionPlan([]), which made PX4 report the mission "finished"
        (0==0) before ever flying anywhere."""
        items = []
        for lat, lon, alt in waypoints:
            items.append(MissionItem(
                lat, lon, alt, speed_m_s,
                True,               # is_fly_through
                float("nan"), float("nan"),  # gimbal pitch/yaw -- unused, no gimbal
                MissionItem.CameraAction.NONE,
                float("nan"),       # loiter_time_s
                float("nan"),       # camera_photo_interval_s
                float("nan"),       # acceptance_radius_m -- PX4 default
                float("nan"),       # yaw_deg -- no forced heading
                float("nan"),       # camera_photo_distance_m
                MissionItem.VehicleAction.NONE,
            ))
        return items

    async def upload_mission(self, waypoints: list) -> None:
        requested_count = len(waypoints)
        self._publish("MISSION_UPLOAD_REQUESTED", f"{requested_count} waypoints",
                      category=Category.LIFECYCLE, data={"requested_item_count": requested_count})

        mission_items = self._to_mission_items(waypoints)
        mission_plan = MissionPlan(mission_items)
        await self.drone.mission.upload_mission(mission_plan)

        # Round-trip verification (ADR-004 §7.2): read the plan back instead
        # of trusting the upload call's mere absence of an exception -- this
        # is the assertion that would have caught the empty-mission bug at
        # the source instead of it surfacing as "the drone just hovers."
        uploaded_plan = await self.drone.mission.download_mission()
        uploaded_count = len(uploaded_plan.mission_items)

        if uploaded_count != requested_count:
            self._publish("MISSION_UPLOAD_MISMATCH",
                          f"requested={requested_count} uploaded={uploaded_count}",
                          severity=Severity.CRITICAL, category=Category.LIFECYCLE,
                          data={"requested_item_count": requested_count, "uploaded_item_count": uploaded_count})
            raise RuntimeError(
                f"MISSION_UPLOAD_MISMATCH: requested {requested_count} waypoints, "
                f"PX4 reports {uploaded_count} uploaded. Refusing to start_mission() "
                f"on an unverified upload."
            )

        self._publish("MISSION_UPLOAD_CONFIRMED", f"{uploaded_count} waypoints confirmed",
                      category=Category.LIFECYCLE,
                      data={"requested_item_count": requested_count, "uploaded_item_count": uploaded_count})

    async def confirm_existing_mission(self) -> int:
        """The operator defines the search route in QGroundControl before
        flight -- this system must never generate and upload its own route
        over it (that silently overwrites whatever the operator planned,
        which is exactly what was happening before this fix). This only
        reads back whatever mission is already on the vehicle and reports
        how many items it has; it never uploads anything."""
        mission_plan = await self.drone.mission.download_mission()
        item_count = len(mission_plan.mission_items)

        if item_count == 0:
            self._publish("MISSION_ROUTE_MISSING", "no mission found on vehicle -- operator must define "
                          "waypoints in QGroundControl before RUN MISSION",
                          severity=Severity.CRITICAL, category=Category.LIFECYCLE,
                          data={"item_count": 0})
        else:
            self._publish("MISSION_ROUTE_CONFIRMED", f"{item_count} items already on vehicle (from QGroundControl)",
                          category=Category.LIFECYCLE, data={"item_count": item_count})

        return item_count

    async def start_mission(self) -> None:
        await self.drone.mission.start_mission()
        self._publish("MISSION_STARTED_ONBOARD", category=Category.LIFECYCLE)

    # ------------------------------------------------------------------
    # ADR-007: raw mission introspection/control.
    #
    # The high-level drone.mission API abstracts items away (it reported 3
    # items for a route whose raw form is 4, hiding the TAKEOFF/LAND
    # entries), so route VALIDATION and start-index selection must read the
    # raw items. Thin pass-throughs only -- no route is ever modified here.
    # ------------------------------------------------------------------
    async def get_raw_mission_items(self) -> list:
        """Raw MAVLink mission items currently on the vehicle.

        Returns a list of objects exposing .seq/.command/.x/.y/.z; empty
        list if the vehicle has no route or the download fails.
        """
        try:
            result = await self.drone.mission_raw.download_mission()
        except Exception as e:  # noqa: BLE001 -- caller decides how to react
            logger.warning(f"Raw mission indirilemedi: {e}")
            return []
        # MAVSDK versions differ: some return (items, fence, rally).
        items = result[0] if isinstance(result, tuple) else result
        return list(items)

    async def set_current_mission_item(self, index: int) -> None:
        """Point PX4 at `index` before starting, without touching the route.

        Used to skip a NAV_TAKEOFF at seq 0 when this system has already
        performed its own takeoff -- otherwise PX4 would re-run the takeoff
        item against an already-airborne vehicle.
        """
        await self.drone.mission_raw.set_current_mission_item(index)
        self._publish("MISSION_CURRENT_ITEM_SET", f"start index {index}",
                      category=Category.LIFECYCLE, data={"index": index})

    async def get_current_mission_index(self) -> int:
        """ADR-010 R2: where PX4 currently is in the route, from the same
        background mission_progress subscription is_mission_finished()
        reads. Used to resume with an explicit set_current_mission_item()
        rather than a bare start_mission()."""
        return self._mission_current_index

    async def is_mission_finished(self) -> bool:
        """See _mission_finished_cache's own comment (__init__) -- this
        used to subscribe fresh to mission_progress() on every call, which
        is what actually throttled the whole search loop. Now just reads
        the value _mission_progress_watcher() keeps current in the
        background."""
        return self._mission_finished_cache
        
    async def switch_to_offboard_from_mission(self) -> None:
        logger.info("Mission'dan Offboard'a geçiş yapılıyor...")
        await self.drone.mission.pause_mission()
        # Offboard başlatma kısmı orchestrator'da ayrıca çağrılacak
