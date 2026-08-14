"""
Regression guard for an operator-reported bug: Gorev2Orchestrator used to
call _generate_square_mission() + flight.upload_mission(), which SILENTLY
OVERWRITES whatever search route the operator already planned and uploaded
via QGroundControl before flight. The operator explicitly does not want
this -- route definition belongs to QGroundControl, not this system.

These tests exercise the real Gorev2Orchestrator.run() against mocks to
prove: (1) upload_mission() is never called, (2) confirm_existing_mission()
is, (3) a route already on the vehicle (operator-defined) lets the mission
proceed, (4) no route present makes the mission fail loudly instead of
starting an empty mission (the exact bug class fixed earlier this
engagement, just from a different cause).
"""
import pytest

from mocks.mock_flight_backend import MockFlightBackend
from mocks.mock_camera_source import MockCameraSource
from mocks.mock_payload_actuator import MockPayloadActuator

from core.detection.target_validator import TargetValidator
from core.detection.target_selector import TargetSelector
from core.mission.debounce import DebounceTracker
from core.position_log.position_store import PositionStore
from core.mission.interlock import PayloadInterlock
from core.navigation.centering_controller import CenteringController
from core.mission.payload_release import PayloadReleaseService
from core.mission.gorev2_fsm import PayloadMissionSequencer
from core.navigation.checkpoint import MissionCheckpoint
from core.mission.gorev2_orchestrator import Gorev2Orchestrator
from core.mission.phase import MissionPhase


class _NullDetector:
    async def detect(self, frame):
        return []


def _build_orchestrator(flight, tmp_path):
    camera = MockCameraSource()
    actuator = MockPayloadActuator()
    detector = _NullDetector()
    validator = TargetValidator()
    selector = TargetSelector()
    debounce = DebounceTracker()
    position_store = PositionStore(str(tmp_path / "positions.json"))
    interlock = PayloadInterlock()
    checkpoint = MissionCheckpoint()
    centering = CenteringController(flight, detector, camera)
    release_service = PayloadReleaseService(actuator, detector, camera, centering, flight)
    sequencer = PayloadMissionSequencer(flight, centering, interlock, position_store, release_service)

    return Gorev2Orchestrator(
        flight=flight, camera=camera, detector=detector, actuator=actuator,
        interlock=interlock, position_store=position_store, debounce=debounce,
        validator=validator, selector=selector, centering=centering, sequencer=sequencer,
        checkpoint=checkpoint, release_service=release_service,
    )


@pytest.mark.asyncio
async def test_never_generates_or_uploads_its_own_route(tmp_path):
    flight = MockFlightBackend()
    flight._is_mission_finished = True  # end the search loop immediately, we only care about startup
    orch = _build_orchestrator(flight, tmp_path)

    await orch.run()

    call_names = [c[0] for c in flight.calls]
    assert "confirm_existing_mission" in call_names
    assert "upload_mission" not in call_names


@pytest.mark.asyncio
async def test_operator_defined_route_lets_mission_proceed(tmp_path):
    flight = MockFlightBackend()
    flight._existing_mission_item_count = 5  # operator already uploaded a route via QGC
    flight._is_mission_finished = True
    # MockFlightBackend._flight_mode defaults to "MISSION", simulating the
    # operator having already pressed Start Mission in QGC -- the wait in
    # _wait_for_operator_mission_start() resolves on its first poll.
    orch = _build_orchestrator(flight, tmp_path)

    await orch.run()  # must not raise

    call_names = [c[0] for c in flight.calls]
    # BUG FIX (operator revision, 2026-08-13, "mission_gz supervisor
    # model"): starting Mission mode is exclusively the operator's action
    # in QGroundControl -- this system must never call start_mission()
    # itself, only observe that Mission mode became active.
    assert "start_mission" not in call_names
    assert "get_flight_mode" in call_names


@pytest.mark.asyncio
async def test_missing_operator_route_fails_loudly_instead_of_starting_empty_mission(tmp_path):
    flight = MockFlightBackend()
    flight._existing_mission_item_count = 0  # operator forgot to define/upload a route
    orch = _build_orchestrator(flight, tmp_path)

    with pytest.raises(RuntimeError, match="MISSION_ROUTE_MISSING"):
        await orch.run()

    call_names = [c[0] for c in flight.calls]
    # Must refuse before ever starting the (nonexistent) mission -- this is
    # the exact bug class already fixed once this engagement (empty
    # MissionPlan -> instant "finished" -> hover forever), now guarded
    # against this different cause too.
    assert "start_mission" not in call_names
    assert orch.context.current_phase == MissionPhase.MISSION_FAILED
