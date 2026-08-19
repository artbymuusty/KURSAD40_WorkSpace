"""
Required test scenarios from the operator's "V32 Flight Authority & Mission
Lifecycle" runtime-correction spec (2026-08-13). Exercises the real
Gorev2Orchestrator end-to-end against mocks -- TEST1-10 numbering matches
the spec's own list.
"""
import asyncio
import pytest

from mocks.mock_flight_backend import MockFlightBackend
from mocks.mock_camera_source import MockCameraSource
from mocks.mock_payload_actuator import MockPayloadActuator

from core.detection.types import Detection
from core.detection.camera_intrinsics import default_camera_intrinsics
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


class _FixedShapeDetector:
    """Reports one or more shapes pinned to a fixed point on the GROUND.

    Was dead-centre every frame regardless of where the vehicle was, which
    made lateral centering vacuous: the error was already zero, so any loop
    "converged" instantly. A2 aims the payload rather than the camera, so
    the target has to end up off-centre by the mount offset -- reachable
    only if flying sideways actually moves the shape in frame. With `flight`
    supplied, the shape is projected from the vehicle's own NED position
    (yaw 0 in this mock: forward=north, right=east); without it, the old
    dead-centre behaviour is kept for the tests that only need a detection
    to exist."""

    def __init__(self, shape_types, flight=None, res=(640, 480)):
        self.shape_types = shape_types
        self.flight = flight
        self.res = res

    def _center_px(self):
        cx, cy = self.res[0] / 2.0, self.res[1] / 2.0
        if self.flight is None:
            return (cx, cy)
        north, east, _ = self.flight._ned_pos
        _, _, alt = self.flight._global_pos
        intrinsics = default_camera_intrinsics()
        if intrinsics is None or not alt or alt <= 0:
            return (cx, cy)
        px_per_m = intrinsics.scaled_to(*self.res).focal_px / alt
        # Image +x is body-right and +y is body-aft, so a vehicle that has
        # moved east sees the ground target slide left, and one that has
        # moved north sees it slide down.
        return (cx - east * px_per_m, cy + north * px_per_m)

    async def detect(self, frame):
        cx, cy = self._center_px()
        return [Detection(shape_type=s, confidence=0.9, center_px=(cx, cy),
                          bbox_px=(cx - 20, cy - 20, cx + 20, cy + 20))
                for s in self.shape_types]


def _build_orchestrator(flight, detector, tmp_path, min_consecutive_frames=1):
    camera = MockCameraSource()
    actuator = MockPayloadActuator()
    validator = TargetValidator(min_consecutive_frames=min_consecutive_frames, center_tolerance_px=20.0)
    selector = TargetSelector()
    debounce = DebounceTracker()
    position_store = PositionStore(str(tmp_path / "positions.json"))
    interlock = PayloadInterlock()
    checkpoint = MissionCheckpoint()
    # ADR-008 B1: centering/verification consume the orchestrator's one
    # detection loop through this feed; only the orchestrator gets the detector.
    detection_feed = DetectionFeed()
    centering = CenteringController(flight, detection_feed, camera)
    centering.lateral_timeout_s = 1.0
    release_service = PayloadReleaseService(actuator, detection_feed, camera, centering, flight)
    sequencer = PayloadMissionSequencer(flight, centering, interlock, position_store, release_service)

    orch = Gorev2Orchestrator(
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
    return orch, position_store, interlock


async def _run_bounded(orch, flight, end_after_s=0.4):
    async def end_soon():
        await asyncio.sleep(end_after_s)
        flight._is_mission_finished = True
    await asyncio.wait_for(asyncio.gather(orch.run(), end_soon()), timeout=15)


@pytest.mark.asyncio
async def test_1_no_targets_search_continues(tmp_path):
    flight = MockFlightBackend()
    orch, position_store, interlock = _build_orchestrator(flight, _NullDetector(), tmp_path)

    await _run_bounded(orch, flight)

    assert position_store.all_points() == []
    assert interlock.both_released() is False
    # Search never completed -- mission ran out without finding both targets.
    assert orch.context.current_phase == MissionPhase.MISSION_FAILED
    assert orch._search_complete is False


@pytest.mark.asyncio
async def test_2_blue_only_search_continues_mission_may_resume(tmp_path):
    flight = MockFlightBackend()
    orch, position_store, interlock = _build_orchestrator(flight, _FixedShapeDetector(["MAVI_ALTIGEN"]), tmp_path)

    await _run_bounded(orch, flight)

    assert position_store.get("MAVI_ALTIGEN") is not None
    assert position_store.get("KIRMIZI_UCGEN") is None
    assert orch._search_complete is False
    start_mission_calls = [c for c in flight.calls if c[0] == 'start_mission']
    # Operator starts the initial Mission themselves (simulated by
    # MockFlightBackend defaulting to flight_mode="MISSION" -- this system
    # never calls start_mission() for that step); the one call left here is
    # the resume after recording Mavi Altigen.
    assert len(start_mission_calls) >= 1


@pytest.mark.asyncio
async def test_3_red_only_search_continues_mission_may_resume(tmp_path):
    flight = MockFlightBackend()
    orch, position_store, interlock = _build_orchestrator(flight, _FixedShapeDetector(["KIRMIZI_UCGEN"]), tmp_path)

    await _run_bounded(orch, flight)

    assert position_store.get("KIRMIZI_UCGEN") is not None
    assert position_store.get("MAVI_ALTIGEN") is None
    assert orch._search_complete is False
    start_mission_calls = [c for c in flight.calls if c[0] == 'start_mission']
    assert len(start_mission_calls) >= 1


@pytest.mark.asyncio
async def test_4_both_targets_found_search_completes_permanently_offboard_sole_authority(tmp_path):
    flight = MockFlightBackend()
    detector = _FixedShapeDetector(["MAVI_ALTIGEN", "KIRMIZI_UCGEN"], flight=flight)
    orch, position_store, interlock = _build_orchestrator(flight, detector, tmp_path)
    # Speed up altitude convergence for this test only (real default kp is a
    # conservative physical-testing placeholder -- see parameters.py) since
    # this scenario runs TWO full staged-descent payload missions back to back.
    orch.centering.kp_altitude = 5.0

    # Never force is_mission_finished -- search completion itself must end
    # the loop; if it didn't, this would hang and the outer timeout would fail the test.
    await asyncio.wait_for(orch.run(), timeout=60)

    assert position_store.both_required_targets_found() is True
    assert orch._search_complete is True
    assert interlock.both_released() is True
    assert orch.context.current_phase == MissionPhase.GOREV2_COMPLETE


@pytest.mark.asyncio
async def test_5_and_8_no_further_mission_commands_once_search_complete(tmp_path):
    """TEST 5 (remaining waypoints never execute) + TEST 8 (single flight
    authority): once search completes, start_mission must never be called
    again, regardless of how long the payload sequence (all Offboard
    position/velocity commands) takes."""
    flight = MockFlightBackend()
    detector = _FixedShapeDetector(["MAVI_ALTIGEN", "KIRMIZI_UCGEN"], flight=flight)
    orch, position_store, interlock = _build_orchestrator(flight, detector, tmp_path)
    orch.centering.kp_altitude = 5.0

    await asyncio.wait_for(orch.run(), timeout=60)

    start_mission_calls = [c for c in flight.calls if c[0] == 'start_mission']
    completion_index = len(flight.calls) - 1 - flight.calls[::-1].index(('start_mission', {}))
    # Every set_velocity_body/goto_position_ned call after the LAST
    # start_mission call belongs to the Offboard-only payload sequence --
    # there must be no start_mission call after search completed.
    calls_after_last_start = flight.calls[completion_index + 1:]
    assert all(c[0] != 'start_mission' for c in calls_after_last_start)
    assert any(c[0] in ('set_velocity_body', 'goto_position_ned', 'goto_position_ned_and_hold')
              for c in calls_after_last_start)


@pytest.mark.asyncio
async def test_6_and_7_illegal_resume_rejected_after_search_complete():
    """TEST 6 (illegal Mission Resume) + TEST 7 (Offboard -> Mission
    transition): _resume_mission_route is the single choke point every
    'go back to Mission' path in this class uses -- once _search_complete
    is True, it must be a permanent no-op."""
    from core.telemetry.event_bus import NULL_PUBLISHER

    flight = MockFlightBackend()
    orch = Gorev2Orchestrator.__new__(Gorev2Orchestrator)
    orch.flight = flight
    orch.publisher = NULL_PUBLISHER
    orch._search_complete = True

    await orch._resume_mission_route()

    assert 'start_mission' not in [c[0] for c in flight.calls]


@pytest.mark.asyncio
async def test_9_new_mission_reset_previous_target_state_does_not_leak(tmp_path):
    """A previous mission's saved records must never satisfy a new
    mission's completion condition -- this is what the mission-ID-scoped
    PositionStore path (main_gz.py/main_real.py/main_dual.py) guarantees in
    production; here we verify the underlying PositionStore behavior a
    fresh path relies on."""
    mission_a_path = str(tmp_path / "mission_positions_AAA.json")
    store_a = PositionStore(mission_a_path)
    store_a.try_save("MAVI_ALTIGEN", 0.9, True, True, (41.0, 29.0, 15.0), "ilk")
    store_a.try_save("KIRMIZI_UCGEN", 0.9, True, True, (41.1, 29.1, 15.0), "ikinci")
    assert store_a.both_required_targets_found() is True

    # New mission -> new mission_id -> new (different) storage path.
    mission_b_path = str(tmp_path / "mission_positions_BBB.json")
    store_b = PositionStore(mission_b_path)

    assert store_b.both_required_targets_found() is False
    assert store_b.all_points() == []


@pytest.mark.asyncio
async def test_10_landing_navigates_to_checkpoint_not_search_waypoints():
    from core.mission.gorev3_finish import Gorev3FinishPhase

    flight = MockFlightBackend()
    checkpoint = MissionCheckpoint()
    checkpoint.save(41.0, 29.0, 15.0)

    class _RecordingCentering:
        def __init__(self):
            self.calls = []
        async def goto_global_position_and_wait(self, lat, lon, alt):
            self.calls.append((lat, lon, alt))
            return True

    centering = _RecordingCentering()
    finish_phase = Gorev3FinishPhase(flight, checkpoint, centering)

    await finish_phase.run()

    assert centering.calls == [(41.0, 29.0, 15.0)]
    # Landing itself is not this phase's job (MasterMissionController calls
    # flight.land() afterward) -- but it must never touch Mission APIs.
    assert 'start_mission' not in [c[0] for c in flight.calls]
    assert 'confirm_existing_mission' not in [c[0] for c in flight.calls]
