"""
Regression guard for the operator-reported "hedefe kilitleniyor ama
sonrasında Offboard'da görevleri tamamlayamıyor" bug: switch_to_offboard()
always pauses the PX4 Mission (via flight.switch_to_offboard_from_mission()),
but nothing ever called start_mission() again after an abandoned or
completed pursuit -- the Mission stayed paused and PX4 auto-exited Offboard
~500ms later with nothing left flying the vehicle. Every non-Görev2-ending
pursuit outcome must now resume the route via
Gorev2Orchestrator._resume_mission_route().
"""
import asyncio
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


@pytest.mark.asyncio
async def test_resume_mission_route_stops_offboard_then_starts_mission():
    flight = MockFlightBackend()
    orch = Gorev2Orchestrator.__new__(Gorev2Orchestrator)  # bypass full __init__, only need self.flight/self.publisher
    orch.flight = flight
    orch._search_complete = False
    orch._last_resume_at = 0.0   # ADR-009: resume spacing state (bypassed __init__)
    from core.telemetry.event_bus import NULL_PUBLISHER
    orch.publisher = NULL_PUBLISHER

    await orch._resume_mission_route()

    call_names = [c[0] for c in flight.calls]
    assert 'stop_offboard' in call_names
    assert 'start_mission' in call_names
    assert call_names.index('stop_offboard') < call_names.index('start_mission')


@pytest.mark.asyncio
async def test_resume_mission_route_rejected_once_search_complete():
    """Defensive Resume Guard: once _search_complete is True, resume must be
    a permanent no-op regardless of caller -- Mission Resume must be
    IMPOSSIBLE after both targets are confirmed."""
    flight = MockFlightBackend()
    orch = Gorev2Orchestrator.__new__(Gorev2Orchestrator)
    orch.flight = flight
    orch._search_complete = True
    from core.telemetry.event_bus import NULL_PUBLISHER
    orch.publisher = NULL_PUBLISHER

    await orch._resume_mission_route()

    call_names = [c[0] for c in flight.calls]
    assert 'start_mission' not in call_names
    assert 'stop_offboard' not in call_names


@pytest.mark.asyncio
async def test_resume_mission_route_survives_stop_offboard_raising():
    class _RaisingStopOffboard(MockFlightBackend):
        async def stop_offboard(self) -> None:
            raise RuntimeError("OffboardError: offboard was never active")

    flight = _RaisingStopOffboard()
    orch = Gorev2Orchestrator.__new__(Gorev2Orchestrator)
    orch.flight = flight
    orch._search_complete = False
    orch._last_resume_at = 0.0   # ADR-009: resume spacing state (bypassed __init__)
    from core.telemetry.event_bus import NULL_PUBLISHER
    orch.publisher = NULL_PUBLISHER

    await orch._resume_mission_route()  # must not raise

    assert 'start_mission' in [c[0] for c in flight.calls]


class _NullDetector:
    async def detect(self, frame):
        return []


class _AlwaysCenteredDetector:
    async def detect(self, frame):
        return [Detection(shape_type="MAVI_ALTIGEN", confidence=0.9,
                          center_px=(320.0, 240.0), bbox_px=(300, 220, 340, 260))]


class _NeverConvergesDetector:
    async def detect(self, frame):
        return [Detection(shape_type="MAVI_ALTIGEN", confidence=0.9,
                          center_px=(639.0, 0.0), bbox_px=(600, 0, 640, 20))]


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
    centering.lateral_timeout_s = 1.0  # keep the never-converges scenario fast
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
async def test_failed_offboard_switch_resumes_the_paused_route(tmp_path):
    class _RejectingFlight(MockFlightBackend):
        async def start_offboard(self) -> None:
            self.calls.append(('start_offboard', {}))
            # deliberately never confirms OFFBOARD

    flight = _RejectingFlight()
    orch = _build_orchestrator(flight, _AlwaysCenteredDetector(), tmp_path)

    async def end_soon():
        await asyncio.sleep(0.3)
        flight._is_mission_finished = True

    await asyncio.wait_for(asyncio.gather(orch.run(), end_soon()), timeout=10)

    start_mission_calls = [c for c in flight.calls if c[0] == 'start_mission']
    # ADR-007: this system now issues the INITIAL start itself after its own
    # takeoff (it previously waited for the operator), so start_mission() is
    # called once at startup. The resume after the abandoned pursuit is
    # therefore the SECOND call -- asserting >= 1 would be satisfied by the
    # startup call alone and would no longer prove a resume happened.
    assert len(start_mission_calls) >= 2


@pytest.mark.asyncio
async def test_centering_timeout_resumes_the_paused_route(tmp_path):
    flight = MockFlightBackend()  # start_offboard() confirms OFFBOARD
    orch = _build_orchestrator(flight, _NeverConvergesDetector(), tmp_path)

    run_task = asyncio.ensure_future(orch.run())
    try:
        deadline = asyncio.get_event_loop().time() + 8.0
        while asyncio.get_event_loop().time() < deadline:
            start_mission_calls = [c for c in flight.calls if c[0] == 'start_mission']
            # ADR-007: call #1 is this system's own initial start after
            # takeoff; the RESUME we are waiting for is call #2. Breaking on
            # >= 1 would exit during startup, before the centering timeout
            # had even happened.
            if len(start_mission_calls) >= 2:
                break
            await asyncio.sleep(0.1)
    finally:
        run_task.cancel()
        try:
            await run_task
        except asyncio.CancelledError:
            pass

    start_mission_calls = [c for c in flight.calls if c[0] == 'start_mission']
    assert len(start_mission_calls) >= 2, "route was never resumed after the centering timeout"
    assert 'stop_offboard' in [c[0] for c in flight.calls]
