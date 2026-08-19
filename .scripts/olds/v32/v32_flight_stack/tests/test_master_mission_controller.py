"""
Regression guard for a bug found during end-to-end verification: a
failure in Görev 3 (or in the landing call itself) must always still
attempt a landing, and the mission's final phase must accurately reflect
the failure -- not get silently overwritten to MISSION_COMPLETE just
because the landing call itself happened to succeed afterward.

ADR-008 B2 extends that to WHERE it lands. Every terminal path must first
fly back to the recorded start/finish checkpoint; only a run that failed
before the checkpoint was ever recorded may land in place. Two paths did
not land at all before this: Ctrl-C/window-close (asyncio.CancelledError
is a BaseException, so the `except Exception` guards below never saw it)
and the mandatory 10-minute MISSION_TIMEOUT watchdog (it only relabelled
the phase).
"""
import asyncio
import pytest

from core.mission.context import MissionContext
from core.mission.phase import MissionPhase
from core.navigation.checkpoint import MissionCheckpoint
from core.telemetry.event_bus import EventBus
from core.mission.master_fsm import MasterMissionController

CHECKPOINT = (41.0, 29.0, 15.0)


class _StubFlight:
    def __init__(self, flight_mode="OFFBOARD", position=(41.001, 29.001, 15.0)):
        self.land_called = False
        self.holds: list = []
        self._flight_mode = flight_mode
        self._position = position

    async def land(self):
        self.land_called = True

    async def get_global_position(self):
        return self._position

    async def get_flight_mode(self):
        return self._flight_mode

    async def hold_position(self, duration_s: float):
        self.holds.append(duration_s)


class _StubCentering:
    """Records the return-to-checkpoint navigation without re-exercising
    CenteringController's own convergence physics."""

    def __init__(self, converges=True, offboard_ok=True):
        self.goto_calls: list = []
        self.switch_calls = 0
        self._converges = converges
        self._offboard_ok = offboard_ok

    async def goto_global_position_and_wait(self, lat, lon, alt_m):
        self.goto_calls.append((lat, lon, alt_m))
        return self._converges

    async def switch_to_offboard(self):
        self.switch_calls += 1
        return self._offboard_ok


class _StubGorev2:
    def __init__(self, flight, raises=None, checkpoint_saved=True, centering=None, runs_forever=False):
        self.flight = flight
        self.centering = centering or _StubCentering()
        self.checkpoint = MissionCheckpoint()
        if checkpoint_saved:
            self.checkpoint.save(*CHECKPOINT)
        self._raises = raises
        self._runs_forever = runs_forever

    async def run(self):
        if self._raises:
            raise self._raises
        if self._runs_forever:
            await asyncio.sleep(3600)


class _StubGorev3:
    def __init__(self, raises=None, success=True):
        self._raises = raises
        self._success = success

    async def run(self):
        if self._raises:
            raise self._raises
        return self._success


def _assert_returned_to_checkpoint(gorev2):
    assert gorev2.centering.goto_calls, "must fly to the checkpoint before landing"
    lat, lon, _alt = gorev2.centering.goto_calls[-1]
    assert (lat, lon) == CHECKPOINT[:2]
    assert gorev2.flight.holds, "must settle before descending"


@pytest.mark.asyncio
async def test_gorev3_exception_still_lands_and_reports_failure():
    flight = _StubFlight()
    bus = EventBus()
    ctx = MissionContext(publisher=bus, mission_id="m1")
    gorev2 = _StubGorev2(flight)
    gorev3 = _StubGorev3(raises=RuntimeError("GOREV3_TRANSIT_SPEED_M_S not configured"))
    master = MasterMissionController(gorev2, gorev3, context=ctx, publisher=bus)

    await master.run()

    assert flight.land_called is True
    _assert_returned_to_checkpoint(gorev2)
    assert ctx.current_phase == MissionPhase.MISSION_FAILED


@pytest.mark.asyncio
async def test_gorev3_failure_result_still_lands_and_reports_failure():
    flight = _StubFlight()
    ctx = MissionContext(mission_id="m2")
    gorev2 = _StubGorev2(flight)
    gorev3 = _StubGorev3(success=False)
    master = MasterMissionController(gorev2, gorev3, context=ctx)

    await master.run()

    assert flight.land_called is True
    _assert_returned_to_checkpoint(gorev2)
    assert ctx.current_phase == MissionPhase.MISSION_FAILED


@pytest.mark.asyncio
async def test_full_success_lands_and_reports_mission_complete():
    flight = _StubFlight()
    ctx = MissionContext(mission_id="m3")
    gorev2 = _StubGorev2(flight)
    gorev3 = _StubGorev3(success=True)
    master = MasterMissionController(gorev2, gorev3, context=ctx)

    await master.run()

    assert flight.land_called is True
    _assert_returned_to_checkpoint(gorev2)
    assert ctx.current_phase == MissionPhase.MISSION_COMPLETE


@pytest.mark.asyncio
async def test_gorev2_exception_skips_gorev3_but_still_lands():
    flight = _StubFlight()
    ctx = MissionContext(mission_id="m4")
    gorev2 = _StubGorev2(flight, raises=RuntimeError("upload mismatch"))
    gorev3_called = {"ran": False}

    class _TrackingGorev3(_StubGorev3):
        async def run(self):
            gorev3_called["ran"] = True
            return await super().run()

    master = MasterMissionController(gorev2, _TrackingGorev3(), context=ctx)
    await master.run()

    assert flight.land_called is True
    assert gorev3_called["ran"] is False
    _assert_returned_to_checkpoint(gorev2)
    assert ctx.current_phase == MissionPhase.MISSION_FAILED


@pytest.mark.asyncio
async def test_landing_failure_itself_reports_mission_failed():
    class _FailingFlight(_StubFlight):
        async def land(self):
            raise RuntimeError("actuator fault")

    ctx = MissionContext(mission_id="m5")
    gorev2 = _StubGorev2(_FailingFlight())
    gorev3 = _StubGorev3(success=True)
    master = MasterMissionController(gorev2, gorev3, context=ctx)

    await master.run()

    assert ctx.current_phase == MissionPhase.MISSION_FAILED


# ----------------------------------------------------------------------
# ADR-008 B2
# ----------------------------------------------------------------------
@pytest.mark.asyncio
async def test_mission_failed_path_returns_to_checkpoint_before_landing():
    """A2 row 2 -- the exact path the 2026-08-16 run took, which landed
    39.8m+ from its checkpoint."""
    flight = _StubFlight()
    ctx = MissionContext(mission_id="b2-1")
    gorev2 = _StubGorev2(flight)
    master = MasterMissionController(gorev2, _StubGorev3(success=True), context=ctx)

    # Görev 2 returns normally but leaves the phase at MISSION_FAILED.
    async def _failed_run():
        ctx.transition_to(MissionPhase.MISSION_FAILED, reason="search_incomplete_mission_finished")
    gorev2.run = _failed_run

    await master.run()

    _assert_returned_to_checkpoint(gorev2)
    assert flight.land_called is True
    assert ctx.current_phase == MissionPhase.MISSION_FAILED


@pytest.mark.asyncio
async def test_lands_in_place_when_checkpoint_was_never_recorded():
    """The one permitted exception: a failure before CHECKPOINT_SAVE (e.g.
    takeoff never reached 15m) has no recorded position to return to."""
    flight = _StubFlight()
    ctx = MissionContext(mission_id="b2-2")
    gorev2 = _StubGorev2(flight, raises=RuntimeError("arm rejected"), checkpoint_saved=False)
    master = MasterMissionController(gorev2, _StubGorev3(), context=ctx)

    await master.run()

    assert gorev2.centering.goto_calls == [], "nowhere to return to"
    assert flight.land_called is True
    assert ctx.current_phase == MissionPhase.MISSION_FAILED


@pytest.mark.asyncio
async def test_return_navigation_failure_still_lands():
    """A failed return must never cost the landing itself."""
    class _ExplodingCentering(_StubCentering):
        async def goto_global_position_and_wait(self, lat, lon, alt_m):
            raise RuntimeError("offboard rejected mid-return")

    flight = _StubFlight()
    ctx = MissionContext(mission_id="b2-3")
    gorev2 = _StubGorev2(flight, centering=_ExplodingCentering())
    master = MasterMissionController(gorev2, _StubGorev3(success=True), context=ctx)

    await master.run()

    assert flight.land_called is True


@pytest.mark.asyncio
async def test_offboard_is_acquired_before_the_return_leg_when_not_already_active():
    """After a failed pursuit the vehicle sits in HOLD, but
    goto_global_position_and_wait() streams Offboard setpoints."""
    flight = _StubFlight(flight_mode="HOLD")
    ctx = MissionContext(mission_id="b2-4")
    gorev2 = _StubGorev2(flight)
    master = MasterMissionController(gorev2, _StubGorev3(success=True), context=ctx)

    await master.run()

    assert gorev2.centering.switch_calls == 1
    _assert_returned_to_checkpoint(gorev2)


@pytest.mark.asyncio
async def test_ctrl_c_cancel_returns_to_checkpoint_and_lands():
    """A2 row 7. Cancelling used to skip _safe_land() entirely --
    CancelledError is a BaseException -- leaving the vehicle airborne."""
    flight = _StubFlight()
    ctx = MissionContext(mission_id="b2-5")
    gorev2 = _StubGorev2(flight, runs_forever=True)
    master = MasterMissionController(gorev2, _StubGorev3(), context=ctx)

    task = asyncio.ensure_future(master.run())
    await asyncio.sleep(0.05)
    task.cancel()
    await task

    _assert_returned_to_checkpoint(gorev2)
    assert flight.land_called is True
    assert ctx.current_phase == MissionPhase.MISSION_FAILED


@pytest.mark.asyncio
async def test_mission_timeout_abort_returns_to_checkpoint_and_lands():
    """A2 row 6. GOREV2_MAX_FLIGHT_DURATION_S is Şartname Bölüm 5.6's
    MANDATORY limit; firing the watchdog used to only relabel the phase."""
    flight = _StubFlight()
    ctx = MissionContext(mission_id="b2-6")
    gorev2 = _StubGorev2(flight, runs_forever=True)
    master = MasterMissionController(gorev2, _StubGorev3(), context=ctx)

    task = asyncio.ensure_future(master.run())
    await asyncio.sleep(0.05)
    master.request_abort("MISSION_TIMEOUT: exceeded 600s budget")  # what the watchdog hook calls
    await task

    _assert_returned_to_checkpoint(gorev2)
    assert flight.land_called is True
    assert ctx.current_phase == MissionPhase.MISSION_FAILED


@pytest.mark.asyncio
async def test_abort_return_is_bounded_and_still_lands_if_it_overruns():
    """The abort return must never outlive the process that runs it -- if
    it does, land immediately rather than unwind still airborne."""
    from core.config import parameters

    class _HangingCentering(_StubCentering):
        async def goto_global_position_and_wait(self, lat, lon, alt_m):
            self.goto_calls.append((lat, lon, alt_m))
            await asyncio.sleep(3600)

    original = parameters.ABORT_RETURN_DEADLINE_S
    import core.mission.master_fsm as master_fsm
    master_fsm.ABORT_RETURN_DEADLINE_S = 0.2
    try:
        flight = _StubFlight()
        ctx = MissionContext(mission_id="b2-7")
        gorev2 = _StubGorev2(flight, runs_forever=True, centering=_HangingCentering())
        master = MasterMissionController(gorev2, _StubGorev3(), context=ctx)

        task = asyncio.ensure_future(master.run())
        await asyncio.sleep(0.05)
        master.request_abort("MISSION_TIMEOUT: exceeded 600s budget")
        await asyncio.wait_for(task, timeout=5)

        assert flight.land_called is True
    finally:
        master_fsm.ABORT_RETURN_DEADLINE_S = original
