"""
End-to-end regression guard for the operator-reported Mission -> Offboard
handover bug, exercising the real Gorev2Orchestrator against mocks:
  - a rejected/unconfirmed Offboard switch must NOT abort the whole mission
    (it used to propagate uncaught and take down all of Görev 2 over one
    failed engagement attempt) -- it must fall back to SEARCHING instead.
  - a successful switch really does drive the vehicle with real velocity
    commands during centering, not silence.
"""
import pytest

from mocks.mock_flight_backend import MockFlightBackend
from mocks.mock_camera_source import MockCameraSource
from mocks.mock_payload_actuator import MockPayloadActuator

from core.detection.types import Detection
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


class _AlwaysCenteredDetector:
    """Reports MAVI_ALTIGEN sitting exactly at frame center every time --
    satisfies is_track_ready() quickly (consecutive frames + altitude) and
    lets go_to_and_center() converge on its first check."""
    async def detect(self, frame):
        return [Detection(shape_type="MAVI_ALTIGEN", confidence=0.9,
                          center_px=(320.0, 240.0), bbox_px=(300, 220, 340, 260))]


class _RejectingFlight(MockFlightBackend):
    """PX4 never confirms OFFBOARD -- switch_to_offboard() must return False,
    and the orchestrator must not crash the mission over it."""
    async def start_offboard(self) -> None:
        self.calls.append(('start_offboard', {}))
        # deliberately do NOT set self._flight_mode = "OFFBOARD"


def _build_orchestrator(flight, tmp_path, min_consecutive_frames=1):
    camera = MockCameraSource()
    actuator = MockPayloadActuator()
    detector = _AlwaysCenteredDetector()
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
async def test_rejected_offboard_switch_does_not_abort_the_mission(tmp_path):
    import asyncio

    flight = _RejectingFlight()
    orch = _build_orchestrator(flight, tmp_path)

    async def end_after_one_pass():
        # Let the loop attempt the (failing) offboard switch at least once, then end.
        await asyncio.sleep(0.3)
        flight._is_mission_finished = True

    # BUG FIX assertion: this used to raise OffboardError straight out of
    # run() (aborting all of Görev 2 over one failed engagement attempt).
    # It must not raise here -- wait_for would surface that as an exception.
    await asyncio.wait_for(asyncio.gather(orch.run(), end_after_one_pass()), timeout=10)

    call_names = [c[0] for c in flight.calls]
    assert 'start_offboard' in call_names
    # Having abandoned the pursuit and returned to SEARCHING, it must NOT
    # have proceeded into centering with a vehicle that was never
    # confirmed to be in Offboard.
    assert 'set_velocity_body' not in call_names
    # The mission still ends via the ordinary "ran out of mission without
    # completing both drops" path, not a crash -- a real, different,
    # already-covered failure mode (test_master_mission_controller.py),
    # not the bug under test here.
    assert orch.context.current_phase == MissionPhase.MISSION_FAILED


@pytest.mark.asyncio
async def test_successful_handover_actually_streams_velocity_commands(tmp_path):
    """Operator revision (2026-08-13): a full engagement now runs a real
    staged multi-altitude descent (PayloadReleaseService._staged_approach),
    which legitimately takes many real seconds against MockFlightBackend's
    velocity-integrated altitude physics -- realistic flight timing, not a
    bug. This test only cares about the initial handover + first centering
    pass, so it runs the orchestrator as a background task and cancels it
    once those calls have been observed, instead of waiting for the whole
    mission (including the staged descent) to finish."""
    import asyncio

    flight = MockFlightBackend()  # start_offboard() confirms OFFBOARD
    orch = _build_orchestrator(flight, tmp_path)

    run_task = asyncio.ensure_future(orch.run())
    try:
        deadline = asyncio.get_event_loop().time() + 5.0
        while asyncio.get_event_loop().time() < deadline:
            call_names = [c[0] for c in flight.calls]
            if 'hold_position' in call_names:
                break
            await asyncio.sleep(0.05)
    finally:
        run_task.cancel()
        try:
            await run_task
        except asyncio.CancelledError:
            pass

    call_names = [c[0] for c in flight.calls]
    assert 'start_offboard' in call_names
    assert 'set_velocity_body' in call_names  # the actual bug: this used to never appear
    assert 'hold_position' in call_names
