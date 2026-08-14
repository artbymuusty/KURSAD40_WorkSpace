"""
Regression guard for a bug found during end-to-end verification: a
failure in Görev 3 (or in the landing call itself) must always still
attempt a landing, and the mission's final phase must accurately reflect
the failure -- not get silently overwritten to MISSION_COMPLETE just
because the landing call itself happened to succeed afterward.
"""
import pytest
from core.mission.context import MissionContext
from core.mission.phase import MissionPhase
from core.telemetry.event_bus import EventBus
from core.mission.master_fsm import MasterMissionController


class _StubFlight:
    def __init__(self):
        self.land_called = False

    async def land(self):
        self.land_called = True


class _StubGorev2:
    def __init__(self, flight, raises=None):
        self.flight = flight
        self._raises = raises

    async def run(self):
        if self._raises:
            raise self._raises


class _StubGorev3:
    def __init__(self, raises=None, success=True):
        self._raises = raises
        self._success = success

    async def run(self):
        if self._raises:
            raise self._raises
        return self._success


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
    assert ctx.current_phase == MissionPhase.MISSION_FAILED


@pytest.mark.asyncio
async def test_landing_failure_itself_reports_mission_failed():
    class _FailingFlight:
        async def land(self):
            raise RuntimeError("actuator fault")

    ctx = MissionContext(mission_id="m5")
    gorev2 = _StubGorev2(_FailingFlight())
    gorev3 = _StubGorev3(success=True)
    master = MasterMissionController(gorev2, gorev3, context=ctx)

    await master.run()

    assert ctx.current_phase == MissionPhase.MISSION_FAILED
