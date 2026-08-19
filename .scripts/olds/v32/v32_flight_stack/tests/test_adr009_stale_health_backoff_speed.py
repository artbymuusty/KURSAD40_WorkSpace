"""
ADR-009 regression guards.

D1  A cached telemetry getter must RAISE when its sample is stale, and every
    control loop must stop commanding within one guard period instead of
    flying its full timeout on frozen numbers.
D2  HealthMonitor must not treat its own HEALTH_STATE_CHANGED events as
    heartbeats, so a silent subsystem actually reaches DOWN.
D3  A failed pursuit must back off (cooldown) and eventually give up
    (attempt cap) instead of re-engaging until PX4 stops answering.
S1  Outside tolerance the commanded lateral speed has a floor; inside
    tolerance it is exactly zero.

All four exist because of measured failures on 2026-08-16 -- see ADR-009.
"""
import asyncio
import time

import pytest

from mocks.mock_camera_source import MockCameraSource
from mocks.mock_flight_backend import MockFlightBackend

from core.config.parameters import (
    CENTERING_MAX_ATTEMPTS_PER_TARGET,
    CENTERING_MIN_CMD_SPEED_M_S,
    CENTERING_RETRY_COOLDOWN_S,
    GLOBAL_POSITION_NAV_TIMEOUT_S,
    TELEMETRY_STALE_AFTER_S,
)
from core.detection.detection_feed import DetectionFeed
from core.detection.types import Detection
from core.interfaces.i_flight_backend import TelemetryStale
from core.navigation.centering_controller import CenteringController, _with_min_speed
from core.telemetry.events import Category, Event, Severity
from core.telemetry.health import DOWN, HealthMonitor


# ======================================================================
# D1 -- freshness guard
# ======================================================================
class _WedgeableFlight(MockFlightBackend):
    """A backend whose telemetry link can be wedged mid-flight, exactly as
    the real one did on 2026-08-16 when pause_mission() timed out."""

    def __init__(self):
        super().__init__()
        self.wedged = False
        self.commands_after_wedge = 0

    def _guard(self):
        if self.wedged:
            raise TelemetryStale("position: last sample 9.9s old (limit 1.0s) -- test wedge")

    async def get_global_position(self):
        self._guard()
        return await super().get_global_position()

    async def get_velocity_ned(self):
        self._guard()
        return await super().get_velocity_ned()

    async def get_position_ned(self):
        self._guard()
        return await super().get_position_ned()

    async def get_yaw_deg(self):
        self._guard()
        return await super().get_yaw_deg()

    async def goto_position_ned(self, *a, **kw):
        if self.wedged:
            self.commands_after_wedge += 1
        return await super().goto_position_ned(*a, **kw)

    async def set_velocity_body(self, *a, **kw):
        if self.wedged:
            self.commands_after_wedge += 1
        return await super().set_velocity_body(*a, **kw)


@pytest.mark.asyncio
async def test_goto_stops_within_one_guard_period_when_the_link_wedges():
    """THE D1 regression. On 2026-08-16 this loop flew its full 60s timeout
    computing distance and speed from a position that had not changed in
    66.8 seconds. It must now bail out almost immediately."""
    flight = _WedgeableFlight()
    flight._global_pos = (41.0, 29.0, 15.0)
    controller = CenteringController(flight, DetectionFeed(), MockCameraSource())

    async def wedge_soon():
        await asyncio.sleep(0.3)
        flight.wedged = True

    started = time.monotonic()
    converged, _ = await asyncio.gather(
        controller.goto_global_position_and_wait(41.01, 29.01, 15.0), wedge_soon())
    elapsed = time.monotonic() - started

    assert converged is False
    # The whole point: bounded by the guard, not by the navigation timeout.
    assert elapsed < 0.3 + TELEMETRY_STALE_AFTER_S + 0.5
    assert elapsed < GLOBAL_POSITION_NAV_TIMEOUT_S / 10
    assert flight.commands_after_wedge == 0, "must not keep commanding on dead telemetry"


@pytest.mark.asyncio
async def test_centering_stops_commanding_when_the_link_wedges():
    flight = _WedgeableFlight()
    feed = DetectionFeed()
    feed.publish([Detection(shape_type="MAVI_ALTIGEN", confidence=0.9,
                            center_px=(500.0, 240.0), bbox_px=(0, 0, 10, 10))])
    controller = CenteringController(flight, feed, MockCameraSource())
    controller.lateral_timeout_s = 5.0
    flight.wedged = True

    started = time.monotonic()
    converged = await controller.go_to_and_center("MAVI_ALTIGEN")

    assert converged is False
    assert time.monotonic() - started < 1.0, "must not spend the centering budget"
    assert flight.commands_after_wedge == 0


@pytest.mark.asyncio
async def test_stale_abort_is_published_so_the_operator_can_see_why():
    published = []

    class _Rec:
        def publish(self, event):
            published.append(event)

    flight = _WedgeableFlight()
    flight.wedged = True
    controller = CenteringController(flight, DetectionFeed(), MockCameraSource(), publisher=_Rec())

    await controller.goto_global_position_and_wait(41.01, 29.01, 15.0)

    aborts = [e for e in published if e.code == "TELEMETRY_STALE_ABORT"]
    assert aborts and aborts[0].severity == Severity.CRITICAL


def test_backend_getters_raise_rather_than_serving_a_frozen_sample():
    """The contract itself: stale must raise, never return the old value."""
    from mavsdk_common.mavsdk_backend_base import MavsdkBackendBase, _StreamCache

    backend = MavsdkBackendBase.__new__(MavsdkBackendBase)
    backend.publisher = type("_N", (), {"publish": staticmethod(lambda e: None)})()
    backend._stale_reported = set()

    cache = _StreamCache("position")
    cache.update("SAMPLE", now=time.time() - (TELEMETRY_STALE_AFTER_S + 0.5))
    with pytest.raises(TelemetryStale):
        backend._fresh(cache)

    cache.update("SAMPLE")  # fresh again
    assert backend._fresh(cache) == "SAMPLE"

    empty = _StreamCache("attitude_euler")
    with pytest.raises(TelemetryStale):
        backend._fresh(empty)


# ======================================================================
# D2 -- HealthMonitor self-feed
# ======================================================================
class _LoopbackBus:
    """Mirrors the real wiring: HealthMonitor publishes to the same bus it
    is subscribed to (ops_center.build_ops_center does exactly this)."""

    def __init__(self):
        self.monitor = None
        self.events = []

    def publish(self, event):
        self.events.append(event)
        if self.monitor is not None:
            self.monitor.on_event(event)


def test_a_silent_subsystem_reaches_down_and_does_not_oscillate():
    """On 2026-08-16 MavsdkBackendBase oscillated HEALTHY<->DEGRADED<->STALE
    for 66.8s of completely dead telemetry and never reached DOWN, because
    each state-change event it published counted as its own heartbeat."""
    bus = _LoopbackBus()
    monitor = HealthMonitor(publisher=bus)
    bus.monitor = monitor
    monitor.register("MavsdkBackendBase", expected_interval_s=1.0, grace_multiplier=3.0)

    t0 = 1000.0
    monitor.on_event(Event(code="VEHICLE_TELEMETRY", subsystem="MavsdkBackendBase",
                           category=Category.TELEMETRY, ts=t0))

    # The subsystem now goes completely silent. Tick the monitor as the
    # supervisor loop would, and watch the states it settles through.
    states = [monitor.check(now=t0 + i)["MavsdkBackendBase"] for i in range(1, 12)]

    assert states[-1] == DOWN
    # Once DOWN it must STAY down -- no recovery without a real heartbeat.
    assert all(s == DOWN for s in states[-5:]), f"oscillated instead of settling: {states}"
    assert "HEALTHY" not in states[1:], f"a dead subsystem must not read HEALTHY again: {states}"


def test_a_real_heartbeat_still_recovers_health():
    """The fix must not make health one-way."""
    bus = _LoopbackBus()
    monitor = HealthMonitor(publisher=bus)
    bus.monitor = monitor
    monitor.register("MavsdkBackendBase", expected_interval_s=1.0, grace_multiplier=3.0)

    t0 = 2000.0
    monitor.on_event(Event(code="VEHICLE_TELEMETRY", subsystem="MavsdkBackendBase",
                           category=Category.TELEMETRY, ts=t0))
    assert monitor.check(now=t0 + 10)["MavsdkBackendBase"] == DOWN

    monitor.on_event(Event(code="VEHICLE_TELEMETRY", subsystem="MavsdkBackendBase",
                           category=Category.TELEMETRY, ts=t0 + 10.5))
    assert monitor.check(now=t0 + 10.6)["MavsdkBackendBase"] == "HEALTHY"


def test_dashboard_renders_down_in_red():
    from core.telemetry.dashboard import COL_BAD, _HEALTH_COLOR
    assert _HEALTH_COLOR["DOWN"] == COL_BAD


# ======================================================================
# D3 -- pursuit backoff
# ======================================================================
def _orchestrator_with_backoff():
    from core.mission.gorev2_orchestrator import Gorev2Orchestrator
    from core.telemetry.event_bus import NULL_PUBLISHER
    orch = Gorev2Orchestrator.__new__(Gorev2Orchestrator)
    orch.publisher = NULL_PUBLISHER
    orch._centering_attempts = {}
    orch._centering_cooldown_until = {}
    orch._centering_abandoned = set()
    return orch


def test_failed_pursuits_cool_down_then_give_up():
    orch = _orchestrator_with_backoff()

    for _ in range(1, CENTERING_MAX_ATTEMPTS_PER_TARGET):
        before = time.time()
        orch._note_centering_failure("MAVI_ALTIGEN")
        assert "MAVI_ALTIGEN" not in orch._centering_abandoned
        # Held off for the cooldown -- this is what stops the pause/resume
        # storm that wedged PX4. Measured from NOW, not from whatever
        # timestamp the caller happened to be holding: the V1' run showed a
        # retry 0.7s after a failure because the loop passed in a `now` it
        # had captured 15s earlier, before the attempt even started.
        cooldown = orch._centering_cooldown_until["MAVI_ALTIGEN"]
        assert cooldown >= before + CENTERING_RETRY_COOLDOWN_S
        assert cooldown <= time.time() + CENTERING_RETRY_COOLDOWN_S

    orch._note_centering_failure("MAVI_ALTIGEN")
    assert "MAVI_ALTIGEN" in orch._centering_abandoned
    # The other target is untouched -- search continues for it.
    assert "KIRMIZI_UCGEN" not in orch._centering_abandoned


def test_backoff_publishes_target_seen_but_not_centered():
    published = []

    class _Rec:
        def publish(self, event):
            published.append(event)

    orch = _orchestrator_with_backoff()
    orch.publisher = _Rec()
    for _ in range(CENTERING_MAX_ATTEMPTS_PER_TARGET):
        orch._note_centering_failure("KIRMIZI_UCGEN")

    codes = [e.code for e in published]
    assert codes.count("CENTERING_RETRY_SCHEDULED") == CENTERING_MAX_ATTEMPTS_PER_TARGET - 1
    assert "TARGET_SEEN_BUT_NOT_CENTERED" in codes


# ======================================================================
# S1 -- minimum command speed
# ======================================================================
def test_min_command_speed_removes_the_dead_zone_outside_tolerance():
    """V1 pursuit 6 sat at |ey|=0.0146 commanding 0.009 m/s for a whole
    15s budget without the error moving."""
    tol = 0.01
    stalled_cmd = 0.0146 * 0.3 * 2.0  # the exact V1 command, ~0.0088 m/s
    assert stalled_cmd < CENTERING_MIN_CMD_SPEED_M_S

    boosted = _with_min_speed(stalled_cmd, error_norm=0.0146, tolerance=tol)
    assert boosted == pytest.approx(CENTERING_MIN_CMD_SPEED_M_S)
    # Direction is preserved.
    assert _with_min_speed(-stalled_cmd, -0.0146, tol) == pytest.approx(-CENTERING_MIN_CMD_SPEED_M_S)


def test_inside_tolerance_commands_exactly_zero():
    """The floor must not become a limit cycle: once inside tolerance the
    vehicle is told to stop, not to keep nudging at the minimum speed."""
    assert _with_min_speed(0.05, error_norm=0.005, tolerance=0.01) == 0.0
    assert _with_min_speed(-0.05, error_norm=-0.005, tolerance=0.01) == 0.0
    assert _with_min_speed(0.0, error_norm=0.0, tolerance=0.01) == 0.0


def test_commands_above_the_floor_are_untouched():
    big = 1.2
    assert _with_min_speed(big, error_norm=0.4, tolerance=0.01) == big


@pytest.mark.asyncio
async def test_centering_still_converges_and_stops_with_the_floor_active():
    """End-to-end sanity for S1: a target inside tolerance must still
    converge and end on an explicit zero-velocity stop."""
    class _FixedFeed(DetectionFeed):
        def __init__(self, center_px):
            super().__init__()
            self.publish([Detection(shape_type="MAVI_ALTIGEN", confidence=0.9,
                                    center_px=center_px, bbox_px=(0, 0, 10, 10))])

        def fresh(self, now=None):
            return self.latest()

    flight = MockFlightBackend()
    controller = CenteringController(flight, _FixedFeed((322.0, 241.0)), MockCameraSource())

    assert await controller.go_to_and_center("MAVI_ALTIGEN") is True
    last = [c for c in flight.calls if c[0] == "set_velocity_body"][-1][1]
    assert last == {"forward_m_s": 0.0, "right_m_s": 0.0, "down_m_s": 0.0, "yaw_rate_deg_s": 0.0}


# ======================================================================
# S2 -- altitude-aware command floor
# ======================================================================
# A multirotor does not achieve a commanded velocity instantly -- it
# accelerates toward it. That lag is the whole reason a coarse command
# floor limit-cycles: by the time the error is inside the tolerance band
# and the command drops to zero, the vehicle is still moving and carries
# straight through to the other side.
#
# The first version of this harness integrated the commanded velocity
# directly (perfect, lag-free tracking) and consequently PASSED with S1's
# flat floor as well as with S2 -- i.e. it proved nothing. Verified after
# adding the lag: with the flat floor these tests fail at 0.30/0.45m and
# pass at 15m, which is exactly the measured V1' behaviour.
_VELOCITY_TAU_S = 0.3


class _ClosedLoopWorld:
    """Kinematic closed loop with first-order velocity lag: the vehicle
    accelerates toward whatever lateral velocity the controller commands,
    and the target's pixel position follows from its remaining ground
    offset and the current altitude.

    This is what makes a limit cycle observable in a unit test -- with a
    static feed the error never responds to the command, so bang-bang
    behaviour is invisible."""

    def __init__(self, alt_m, ground_x_m, ground_y_m, focal_px, res=(1280, 960)):
        self.alt_m = alt_m
        self.gx, self.gy = ground_x_m, ground_y_m
        self.focal_px = focal_px
        self.res = res
        self.vx = 0.0   # actual vehicle velocity, lagging the command
        self.vy = 0.0
        self.err_x_history = []
        self.err_y_history = []

    # -- camera ------------------------------------------------------
    def get_resolution(self):
        return self.res

    async def get_frame(self):
        return None

    # -- detection feed ----------------------------------------------
    def get(self, shape_type, now=None):
        cx, cy = self.res[0] / 2.0, self.res[1] / 2.0
        px = cx + self.gx * self.focal_px / self.alt_m
        py = cy + self.gy * self.focal_px / self.alt_m
        self.err_x_history.append(px - cx)
        self.err_y_history.append(py - cy)
        return Detection(shape_type=shape_type, confidence=0.9,
                         center_px=(px, py), bbox_px=(px - 5, py - 5, px + 5, py + 5))

    def is_stale(self, now=None):
        return False

    def age_s(self, now=None):
        return 0.0

    def detections(self, now=None):
        return []

    # -- flight backend ----------------------------------------------
    async def get_global_position(self):
        return (41.0, 29.0, self.alt_m)

    async def set_velocity_body(self, forward_m_s, right_m_s, down_m_s, yaw_rate_deg_s):
        dt = 0.1
        alpha = dt / _VELOCITY_TAU_S
        self.vx += (right_m_s - self.vx) * alpha
        self.vy += (forward_m_s - self.vy) * alpha
        self.gx -= self.vx * dt
        self.gy += self.vy * dt

    def zero_crossings(self):
        def cross(v):
            return sum(1 for i in range(1, len(v)) if v[i - 1] * v[i] < 0)
        return cross(self.err_x_history), cross(self.err_y_history)


def _steady_state_crossings(alt_m, floor_fraction, iterations=150, start_offset_bands=1.5):
    """Drive the REAL control law for a fixed number of iterations near the
    target and count how often the error changes sign.

    Deliberately does NOT go through go_to_and_center(): that loop breaks
    the instant one sample lands inside the tolerance band, so in a
    noise-free model it always exits before any limit cycle can develop --
    an earlier version of this test did exactly that and consequently
    passed with S1's flat floor too, proving nothing. What "limit cycle"
    actually means is the STEADY-STATE behaviour once the vehicle is near
    the target, so that is what is measured here: no early exit, count the
    sign changes.
    """
    from core.config.parameters import (
        CENTERING_TOLERANCE_X_NORM, MAX_CENTERING_SPEED_M_S, OFFBOARD_SETPOINT_INTERVAL_S,
    )
    from core.detection.camera_intrinsics import default_camera_intrinsics
    import core.navigation.centering_controller as cc

    focal = default_camera_intrinsics().focal_px
    half_axis = 640.0
    dt = OFFBOARD_SETPOINT_INTERVAL_S
    band_m = cc.ground_tolerance_m(CENTERING_TOLERANCE_X_NORM, half_axis, alt_m, focal)

    saved = cc.CENTERING_FLOOR_TOL_FRACTION
    cc.CENTERING_FLOOR_TOL_FRACTION = floor_fraction
    try:
        floor = cc.floor_speed_m_s(CENTERING_TOLERANCE_X_NORM, half_axis, alt_m, focal)
        g = start_offset_bands * band_m     # ground offset, metres
        v = 0.0                             # vehicle velocity, lagging the command
        errs = []
        for _ in range(iterations):
            err_px = g * focal / alt_m
            err_norm = err_px / half_axis
            errs.append(err_norm)
            cmd = cc._clamp(err_norm * 0.5 * MAX_CENTERING_SPEED_M_S, MAX_CENTERING_SPEED_M_S)
            cmd = cc._with_min_speed(cmd, err_norm, CENTERING_TOLERANCE_X_NORM, floor)
            v += (cmd - v) * (dt / _VELOCITY_TAU_S)
            g -= v * dt
    finally:
        cc.CENTERING_FLOOR_TOL_FRACTION = saved

    crossings = sum(1 for i in range(1, len(errs)) if errs[i - 1] * errs[i] < 0)
    peak_bands = max(abs(e) for e in errs[len(errs) // 2:]) / CENTERING_TOLERANCE_X_NORM
    return crossings, peak_bands


@pytest.mark.asyncio
async def test_s2_bounds_steady_state_oscillation_at_045m():
    """The 0.45m release step. Under S1's flat floor one iteration covered
    2.8x the tolerance band, so the error was slammed across zero and back
    indefinitely."""
    s2_cross, s2_peak = _steady_state_crossings(0.45, floor_fraction=0.5)
    s1_cross, s1_peak = _steady_state_crossings(0.45, floor_fraction=1e9)  # flat 0.15 m/s

    assert s2_cross <= 2, f"S2 still limit-cycling at 0.45m: {s2_cross} crossings"
    assert s2_peak <= 1.5, f"S2 steady-state error {s2_peak:.1f} bands at 0.45m"
    # And the guard is meaningful: the flat floor DOES limit-cycle here.
    assert s1_cross > 10, f"harness not reproducing the S1 limit cycle ({s1_cross})"


@pytest.mark.asyncio
async def test_s2_bounds_steady_state_oscillation_at_030m():
    """0.30m is Görev 3's descent altitude (GOREV3_DESCENT_ALTITUDE_M),
    deliberately unchanged -- so the floor has to cope with it."""
    s2_cross, s2_peak = _steady_state_crossings(0.30, floor_fraction=0.5)
    s1_cross, _ = _steady_state_crossings(0.30, floor_fraction=1e9)

    assert s2_cross <= 2, f"S2 still limit-cycling at 0.30m: {s2_cross} crossings"
    assert s2_peak <= 1.5, f"S2 steady-state error {s2_peak:.1f} bands at 0.30m"
    assert s1_cross > 10, f"harness not reproducing the S1 limit cycle ({s1_cross})"


@pytest.mark.asyncio
async def test_s2_leaves_mission_altitude_behaviour_unchanged():
    """At 15m the absolute 0.15 m/s cap still binds, so S2 must not have
    slowed the search-phase lock-on that S1 bought us -- and 15m never
    oscillated under S1 either, so both must look the same."""
    from core.navigation.centering_controller import floor_speed_m_s
    from core.detection.camera_intrinsics import default_camera_intrinsics

    focal = default_camera_intrinsics().focal_px
    assert floor_speed_m_s(0.01, 640, 15.0, focal) == pytest.approx(CENTERING_MIN_CMD_SPEED_M_S)

    s2_cross, s2_peak = _steady_state_crossings(15.0, floor_fraction=0.5)
    s1_cross, s1_peak = _steady_state_crossings(15.0, floor_fraction=1e9)
    assert s2_cross == s1_cross, "15m behaviour must be identical under S1 and S2"
    assert s2_peak == pytest.approx(s1_peak)
    assert s2_cross <= 2


def test_s2_floor_step_never_exceeds_half_the_tolerance_band():
    """The invariant the whole of S2 exists to hold, checked directly."""
    from core.config.parameters import (
        CENTERING_FLOOR_TOL_FRACTION, CENTERING_TOLERANCE_X_NORM, OFFBOARD_SETPOINT_INTERVAL_S,
    )
    from core.detection.camera_intrinsics import default_camera_intrinsics
    from core.navigation.centering_controller import floor_speed_m_s, ground_tolerance_m

    focal = default_camera_intrinsics().focal_px
    for alt in (15.0, 10.0, 5.0, 1.0, 0.45, 0.30, 0.10):
        band = ground_tolerance_m(CENTERING_TOLERANCE_X_NORM, 640, alt, focal)
        step = floor_speed_m_s(CENTERING_TOLERANCE_X_NORM, 640, alt, focal) * OFFBOARD_SETPOINT_INTERVAL_S
        assert step / band <= CENTERING_FLOOR_TOL_FRACTION + 1e-9, f"alt={alt}: {step/band:.2f}x band"


def test_s2_falls_back_to_the_flat_floor_without_altitude_or_intrinsics():
    from core.navigation.centering_controller import floor_speed_m_s
    assert floor_speed_m_s(0.01, 640, None, 540.0) == CENTERING_MIN_CMD_SPEED_M_S
    assert floor_speed_m_s(0.01, 640, 15.0, None) == CENTERING_MIN_CMD_SPEED_M_S
    assert floor_speed_m_s(0.01, 640, 0.0, 540.0) == CENTERING_MIN_CMD_SPEED_M_S


# ======================================================================
# PX4 STABILITY -- Mission resume spacing + confirmation
# ======================================================================
def _resume_orchestrator(flight, publisher=None):
    from core.mission.gorev2_orchestrator import Gorev2Orchestrator
    from core.telemetry.event_bus import NULL_PUBLISHER
    orch = Gorev2Orchestrator.__new__(Gorev2Orchestrator)
    orch.flight = flight
    orch.publisher = publisher or NULL_PUBLISHER
    orch._search_complete = False
    orch._last_resume_at = 0.0
    return orch


class _ResumeFlight(MockFlightBackend):
    """Records resume timing and can emulate PX4 ignoring start_mission()."""

    def __init__(self, enter_mission=True):
        super().__init__()
        self.start_times = []
        self._enter_mission = enter_mission
        self._flight_mode = "HOLD"

    async def stop_offboard(self):
        self._flight_mode = "HOLD"

    async def start_mission(self):
        self.start_times.append(time.monotonic())
        if self._enter_mission:
            self._flight_mode = "MISSION"
        # else: PX4 accepts the command without error and stays in HOLD --
        # exactly what the 4th resume did on 2026-08-17.


@pytest.mark.asyncio
async def test_resume_confirms_px4_actually_entered_mission():
    flight = _ResumeFlight(enter_mission=True)
    orch = _resume_orchestrator(flight)
    await orch._resume_mission_route()
    assert len(flight.start_times) == 1
    assert await flight.get_flight_mode() == "MISSION"


@pytest.mark.asyncio
async def test_resume_retries_then_reports_when_px4_ignores_it(monkeypatch):
    """The 2026-08-17 failure: start_mission() returned cleanly, PX4 stayed
    in HOLD, the route froze at 3/4 and nothing noticed."""
    import core.mission.gorev2_orchestrator as go
    monkeypatch.setattr(go, "MISSION_MODE_CONFIRM_TIMEOUT_S", 0.3)

    published = []

    class _Rec:
        def publish(self, event):
            published.append(event)

    flight = _ResumeFlight(enter_mission=False)
    orch = _resume_orchestrator(flight, _Rec())
    await orch._resume_mission_route()

    assert len(flight.start_times) == 2, "must retry once before giving up"
    codes = [e.code for e in published]
    assert "MISSION_RESUME_NOT_CONFIRMED" in codes
    assert "MISSION_ROUTE_RESUMED" not in codes, "must not claim success"
    crit = [e for e in published if e.code == "MISSION_RESUME_NOT_CONFIRMED"][0]
    assert crit.severity == Severity.CRITICAL


@pytest.mark.asyncio
async def test_consecutive_resumes_are_spaced_out(monkeypatch):
    """Measured cause of the PX4 deaths: 5.04s spacing killed it, 14.39s
    survived, 0.11s wedged it outright."""
    import core.mission.gorev2_orchestrator as go
    monkeypatch.setattr(go, "MISSION_RESUME_MIN_INTERVAL_S", 0.4)

    flight = _ResumeFlight(enter_mission=True)
    orch = _resume_orchestrator(flight)

    await orch._resume_mission_route()
    await orch._resume_mission_route()
    await orch._resume_mission_route()

    gaps = [flight.start_times[i] - flight.start_times[i - 1]
            for i in range(1, len(flight.start_times))]
    assert all(g >= 0.4 * 0.95 for g in gaps), f"resumes not spaced: {gaps}"


@pytest.mark.asyncio
async def test_first_resume_is_not_delayed(monkeypatch):
    """The throttle must not add latency to the first pursuit's recovery."""
    import core.mission.gorev2_orchestrator as go
    monkeypatch.setattr(go, "MISSION_RESUME_MIN_INTERVAL_S", 5.0)

    flight = _ResumeFlight(enter_mission=True)
    orch = _resume_orchestrator(flight)

    started = time.monotonic()
    await orch._resume_mission_route()
    assert time.monotonic() - started < 1.0


@pytest.mark.asyncio
async def test_resume_stays_blocked_after_search_completes():
    """The ADR-008 one-way guard must survive this rework."""
    flight = _ResumeFlight(enter_mission=True)
    orch = _resume_orchestrator(flight)
    orch._search_complete = True
    await orch._resume_mission_route()
    assert flight.start_times == []
