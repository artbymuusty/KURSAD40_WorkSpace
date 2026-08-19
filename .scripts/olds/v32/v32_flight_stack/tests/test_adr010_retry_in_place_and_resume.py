"""
ADR-010 regression guards.

R1  A failed centering attempt must retry the SAME target IN PLACE -- stay
    in Offboard, hold over it, try again -- instead of resuming the route
    and flying away from it.
R2  A resume must point PX4 at the route index it is actually on, and a
    pursuit must not ask PX4 to leave MISSION immediately after one.
R4  Ctrl-C/SIGTERM must reach the cancel path however the process was
    launched.

All three come from measured failures, see ADR-010.
"""
import asyncio
import signal
import time

import pytest

from mocks.mock_camera_source import MockCameraSource
from mocks.mock_flight_backend import MockFlightBackend

from core.config.parameters import (
    CENTERING_MAX_ATTEMPTS_PER_TARGET,
    CENTERING_RETRY_COOLDOWN_S,
    OFFBOARD_AFTER_RESUME_SETTLE_S,
)
from core.detection.detection_feed import DetectionFeed
from core.detection.types import Detection
from core.telemetry.event_bus import NULL_PUBLISHER


# ======================================================================
# R1 -- retry in place
# ======================================================================
class _ScriptedCentering:
    """Returns a scripted sequence of converge/fail results and records
    when each attempt happened."""

    def __init__(self, results):
        self._results = list(results)
        self.attempt_times = []

    async def go_to_and_center(self, shape_type, altitude_m=None,
                               alt_tolerance_m=None, aim_offset_body_m=None):
        self.attempt_times.append(time.monotonic())
        return self._results.pop(0) if self._results else False

    def _ground_distance_m(self, dx, dy, alt, w, h):
        return 0.42


def _orch(centering, feed=None, flight=None, publisher=None):
    from core.mission.gorev2_orchestrator import Gorev2Orchestrator
    orch = Gorev2Orchestrator.__new__(Gorev2Orchestrator)
    orch.centering = centering
    orch.flight = flight or MockFlightBackend()
    orch.camera = MockCameraSource()
    orch.detection_feed = feed or DetectionFeed()
    orch.publisher = publisher or NULL_PUBLISHER
    orch._centering_attempts = {}
    orch._centering_cooldown_until = {}
    orch._centering_abandoned = set()
    orch._last_resume_at = 0.0
    return orch


@pytest.mark.asyncio
async def test_retry_in_place_never_resumes_the_route_between_attempts(monkeypatch):
    """THE R1 regression. On V1'' the vehicle resumed the route after a
    failed attempt and 68.3s later the target was not in a single frame of
    the next 150; it was never recorded and the payload phase never ran."""
    import core.mission.gorev2_orchestrator as go
    monkeypatch.setattr(go, "CENTERING_RETRY_COOLDOWN_S", 0.05)

    flight = MockFlightBackend()
    centering = _ScriptedCentering([False, True])
    orch = _orch(centering, flight=flight)

    assert await orch._center_with_retries("MAVI_ALTIGEN", 15.0) is True
    assert len(centering.attempt_times) == 2
    # The vehicle held position rather than flying the route.
    assert any(c[0] == "hold_position" for c in flight.calls)
    assert not any(c[0] == "start_mission" for c in flight.calls), \
        "must NOT resume the route between attempts"


@pytest.mark.asyncio
async def test_retry_in_place_holds_for_the_cooldown_between_attempts(monkeypatch):
    import core.mission.gorev2_orchestrator as go
    monkeypatch.setattr(go, "CENTERING_RETRY_COOLDOWN_S", 0.2)

    flight = MockFlightBackend()
    centering = _ScriptedCentering([False, True])
    orch = _orch(centering, flight=flight)
    await orch._center_with_retries("KIRMIZI_UCGEN", 15.0)

    holds = [c for c in flight.calls if c[0] == "hold_position"]
    assert holds and holds[0][1]["duration_s"] == pytest.approx(0.2)


@pytest.mark.asyncio
async def test_retry_in_place_gives_up_at_the_cap_and_reports(monkeypatch):
    import core.mission.gorev2_orchestrator as go
    monkeypatch.setattr(go, "CENTERING_RETRY_COOLDOWN_S", 0.01)

    published = []

    class _Rec:
        def publish(self, event):
            published.append(event)

    centering = _ScriptedCentering([False] * CENTERING_MAX_ATTEMPTS_PER_TARGET)
    orch = _orch(centering, publisher=_Rec())

    assert await orch._center_with_retries("MAVI_ALTIGEN", 15.0) is False
    assert len(centering.attempt_times) == CENTERING_MAX_ATTEMPTS_PER_TARGET
    assert "MAVI_ALTIGEN" in orch._centering_abandoned

    codes = [e.code for e in published]
    # One RETRY_IN_PLACE per gap between attempts, then the give-up.
    assert codes.count("RETRY_IN_PLACE") == CENTERING_MAX_ATTEMPTS_PER_TARGET - 1
    assert "TARGET_SEEN_BUT_NOT_CENTERED" in codes


@pytest.mark.asyncio
async def test_retry_in_place_reports_distance_to_target(monkeypatch):
    """RETRY_IN_PLACE must say where the target is, so a run log shows
    whether the retry had anything to aim at."""
    import core.mission.gorev2_orchestrator as go
    monkeypatch.setattr(go, "CENTERING_RETRY_COOLDOWN_S", 0.01)

    published = []

    class _Rec:
        def publish(self, event):
            published.append(event)

    feed = DetectionFeed()
    feed.publish([Detection(shape_type="MAVI_ALTIGEN", confidence=0.9,
                            center_px=(340.0, 250.0), bbox_px=(0, 0, 10, 10))])
    orch = _orch(_ScriptedCentering([False, True]), feed=feed, publisher=_Rec())
    await orch._center_with_retries("MAVI_ALTIGEN", 15.0)

    retry = [e for e in published if e.code == "RETRY_IN_PLACE"][0]
    assert retry.data["target_visible"] is True
    assert retry.data["ground_distance_m"] == pytest.approx(0.42)
    assert retry.data["attempt"] == 1
    assert retry.data["max_attempts"] == CENTERING_MAX_ATTEMPTS_PER_TARGET


@pytest.mark.asyncio
async def test_first_attempt_success_does_not_hold_at_all():
    flight = MockFlightBackend()
    orch = _orch(_ScriptedCentering([True]), flight=flight)
    assert await orch._center_with_retries("MAVI_ALTIGEN", 15.0) is True
    assert not any(c[0] == "hold_position" for c in flight.calls)


# ======================================================================
# R2 -- robust resume
# ======================================================================
@pytest.mark.asyncio
async def test_resume_points_px4_at_the_live_mission_index(monkeypatch):
    """A bare start_mission() relied on PX4's implicit resume state, which
    it silently declined to act on on 2026-08-17."""
    import core.mission.gorev2_orchestrator as go
    monkeypatch.setattr(go, "MISSION_RESUME_MIN_INTERVAL_S", 0.0)

    flight = MockFlightBackend()
    flight._current_mission_item = 2
    orch = _orch(_ScriptedCentering([]), flight=flight)
    orch._search_complete = False

    await orch._resume_mission_route()

    names = [c[0] for c in flight.calls]
    assert "set_current_mission_item" in names
    idx = [c for c in flight.calls if c[0] == "set_current_mission_item"][0][1]["index"]
    assert idx == 2
    assert names.index("set_current_mission_item") < names.index("start_mission")


@pytest.mark.asyncio
async def test_resume_still_starts_mission_if_setting_the_index_fails(monkeypatch):
    """Setting the index is best-effort; the MISSION-mode confirm is the
    real gate."""
    import core.mission.gorev2_orchestrator as go
    monkeypatch.setattr(go, "MISSION_RESUME_MIN_INTERVAL_S", 0.0)

    class _NoIndex(MockFlightBackend):
        async def set_current_mission_item(self, index):
            raise RuntimeError("unsupported")

    flight = _NoIndex()
    orch = _orch(_ScriptedCentering([]), flight=flight)
    orch._search_complete = False

    await orch._resume_mission_route()
    assert any(c[0] == "start_mission" for c in flight.calls)


@pytest.mark.asyncio
async def test_offboard_request_waits_for_the_post_resume_settle(monkeypatch):
    """PX4 refused OFFBOARD 4 times in V1'' when a pursuit asked to pause
    the Mission ~1s after a resume started it."""
    import core.mission.gorev2_orchestrator as go
    monkeypatch.setattr(go, "OFFBOARD_AFTER_RESUME_SETTLE_S", 0.3)

    orch = _orch(_ScriptedCentering([]))
    orch._last_resume_at = time.time()

    started = time.monotonic()
    await orch._settle_after_resume()
    assert time.monotonic() - started >= 0.3 * 0.9


@pytest.mark.asyncio
async def test_no_settle_when_no_resume_has_happened():
    orch = _orch(_ScriptedCentering([]))
    started = time.monotonic()
    await orch._settle_after_resume()
    assert time.monotonic() - started < 0.05


# ======================================================================
# R4 -- signal delivery
# ======================================================================
def test_signal_handlers_override_an_inherited_sig_ign():
    """ROOT CAUSE of V3's failure: a process launched with `&` from a
    non-interactive shell inherits SIGINT=SIG_IGN (POSIX background-job
    behaviour). Python then never installs default_int_handler, so
    KeyboardInterrupt can never be raised and `kill -INT` is silently
    discarded -- main_gz's `except KeyboardInterrupt` was unreachable and
    the vehicle would be left airborne on any background-launched mission.
    Installing our own handler overrides that inherited disposition."""
    from gz_system.main_gz import _install_signal_handlers

    previous_int = signal.getsignal(signal.SIGINT)
    previous_term = signal.getsignal(signal.SIGTERM)
    try:
        # Simulate exactly what a backgrounded launch hands us.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        assert signal.getsignal(signal.SIGINT) == signal.SIG_IGN

        called = []
        _install_signal_handlers(lambda: called.append(True))

        handler = signal.getsignal(signal.SIGINT)
        assert handler not in (signal.SIG_IGN, signal.SIG_DFL), \
            "SIG_IGN must be overridden or Ctrl-C can never work"
        assert signal.getsignal(signal.SIGTERM) not in (signal.SIG_IGN, signal.SIG_DFL)

        # And the handler actually triggers the stop request.
        handler(signal.SIGINT, None)
        assert called == [True]
    finally:
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)
