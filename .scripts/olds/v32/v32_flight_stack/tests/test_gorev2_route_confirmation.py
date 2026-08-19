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

from mocks.mock_flight_backend import MockFlightBackend, _RawItem
from mocks.mock_camera_source import MockCameraSource
from mocks.mock_payload_actuator import MockPayloadActuator

from core.detection.target_validator import TargetValidator
from core.detection.target_selector import TargetSelector
from core.mission.debounce import DebounceTracker
from core.position_log.position_store import PositionStore
from core.mission.interlock import PayloadInterlock
from core.detection.detection_feed import DetectionFeed
from core.detection.vision_runtime import VisionRuntime
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
    # ADR-008 B1: centering/verification consume the orchestrator's one
    # detection loop through this feed; only the orchestrator gets the detector.
    detection_feed = DetectionFeed()
    centering = CenteringController(flight, detection_feed, camera)
    release_service = PayloadReleaseService(actuator, detection_feed, camera, centering, flight)
    sequencer = PayloadMissionSequencer(flight, centering, interlock, position_store, release_service)

    return Gorev2Orchestrator(
        flight=flight, camera=camera, detector=detector, actuator=actuator,
        interlock=interlock, position_store=position_store, debounce=debounce,
        validator=validator, selector=selector, centering=centering, sequencer=sequencer,
        checkpoint=checkpoint, release_service=release_service,
        detection_feed=detection_feed,
        # ADR-010 P3: vision now lives outside the orchestrator. These
        # tests exercise Görev 2 alone, so they scope a runtime to this
        # run; production (main_gz) owns one for the whole mission.
        vision_runtime=VisionRuntime(camera, detector, detection_feed),
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
    # MockFlightBackend._flight_mode defaults to "MISSION", so the ADR-007
    # mission-mode confirmation resolves on its first poll.
    orch = _build_orchestrator(flight, tmp_path)

    await orch.run()  # must not raise

    call_names = [c[0] for c in flight.calls]
    # ADR-007 (supersedes the 2026-08-13 "mission_gz supervisor model"):
    # this system now starts the operator-uploaded route itself after its
    # own takeoff, then confirms PX4 actually entered Mission mode. Route
    # DEFINITION is still exclusively the operator's job in QGroundControl,
    # which test_missing_operator_route_... below still enforces.
    assert "start_mission" in call_names
    assert "get_raw_mission_items" in call_names
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


# ---------------------------------------------------------------------
# ADR-007 route validation rule
# ---------------------------------------------------------------------
@pytest.mark.parametrize("items, expect_ok, expect_start_index", [
    ([_RawItem(0, 16), _RawItem(1, 16), _RawItem(2, 16)], True, 0),   # waypoints only
    ([_RawItem(0, 22), _RawItem(1, 16), _RawItem(2, 16)], True, 1),   # takeoff at seq0 -> skip
    ([_RawItem(0, 16)], False, None),                                  # too few
    ([_RawItem(0, 16), _RawItem(1, 21)], False, None),                 # NAV_LAND
    ([_RawItem(0, 16), _RawItem(1, 20)], False, None),                 # NAV_RETURN_TO_LAUNCH
    ([_RawItem(0, 16), _RawItem(1, 22)], False, None),                 # takeoff not at seq0
])
def test_adr007_route_validation(items, expect_ok, expect_start_index, tmp_path):
    orch = _build_orchestrator(MockFlightBackend(), tmp_path)
    if expect_ok:
        assert orch._validate_route_and_start_index(items) == expect_start_index
    else:
        with pytest.raises(RuntimeError, match="MISSION_ROUTE_INVALID"):
            orch._validate_route_and_start_index(items)


@pytest.mark.asyncio
async def test_adr007_refuses_route_containing_land(tmp_path):
    """A NAV_LAND in the route must be refused BEFORE arming."""
    flight = MockFlightBackend()
    flight._raw_mission_items = [_RawItem(0, 16), _RawItem(1, 16), _RawItem(2, 21)]
    orch = _build_orchestrator(flight, tmp_path)

    with pytest.raises(RuntimeError, match="MISSION_ROUTE_INVALID"):
        await orch.run()

    call_names = [c[0] for c in flight.calls]
    assert "start_mission" not in call_names   # never started an unflyable route
    assert orch.context.current_phase == MissionPhase.MISSION_FAILED


@pytest.mark.asyncio
async def test_adr007_start_sequence_holds_before_starting(tmp_path, monkeypatch):
    """The settling hold is real: it runs BEFORE start_mission(), and the
    flight heartbeat is published throughout so HealthMonitor does not age
    MavsdkBackendBase into STALE while we wait (conftest zeroes the hold for
    every other test, so this is where the behaviour is pinned)."""
    from core.config import parameters
    monkeypatch.setattr(parameters, "MISSION_START_HOLD_S", 0.6)

    flight = MockFlightBackend()
    flight._is_mission_finished = True
    orch = _build_orchestrator(flight, tmp_path)

    import time as _t
    t0 = _t.monotonic()
    await orch.run()
    elapsed = _t.monotonic() - t0

    assert elapsed >= 0.6                      # the hold was actually taken
    names = [c[0] for c in flight.calls]
    start_i = names.index("start_mission")
    # heartbeat (get_global_position) published during the hold, i.e. before start
    assert "get_global_position" in names[:start_i]
    # validation happened before the start, not after
    assert names.index("get_raw_mission_items") < start_i
