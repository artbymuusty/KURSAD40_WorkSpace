"""
Operator revision (2026-08-13): release_and_verify() now owns the full
staged approach (descend through PAYLOAD_APPROACH_ALTITUDES_M, re-centering
at each step, forward nudge, servo call) and the post-drop climb back to
MISSION_ALTITUDE_M, not just the actuator call + verification it used to.
"""
import pytest

from mocks.mock_payload_actuator import MockPayloadActuator
from mocks.mock_camera_source import MockCameraSource

from core.detection.types import Detection
from core.mission.payload_release import PayloadReleaseService
from core.config.parameters import PAYLOAD_APPROACH_ALTITUDES_M, MISSION_ALTITUDE_M


class _RecordingCentering:
    """Records the exact sequence/arguments of centering calls
    PayloadReleaseService makes, without re-exercising CenteringController's
    own convergence physics (already covered by test_centering_controller.py)."""
    def __init__(self):
        self.calls: list = []

    async def go_to_and_center(self, shape_type: str, altitude_m: float) -> bool:
        self.calls.append(('go_to_and_center', shape_type, altitude_m))
        return True

    async def nudge_forward(self, distance_m: float) -> None:
        self.calls.append(('nudge_forward', distance_m))

    async def climb_to_altitude(self, altitude_m: float) -> bool:
        self.calls.append(('climb_to_altitude', altitude_m))
        return True


class _NoMarkerDetector:
    """Never finds the verification marker -- exercises the best-effort
    (non-blocking) verification-failure path."""
    async def detect(self, frame):
        return []


@pytest.mark.asyncio
async def test_release_and_verify_runs_staged_approach_then_servo_then_climb_back():
    actuator = MockPayloadActuator()
    centering = _RecordingCentering()
    camera = MockCameraSource()
    detector = _NoMarkerDetector()

    service = PayloadReleaseService(actuator, detector, camera, centering, flight=None)

    result = await service.release_and_verify("MAVI_ALTIGEN")

    step_names = [c[0] for c in centering.calls]
    # Staged descent through every configured altitude, in order.
    descent_calls = [c for c in centering.calls if c[0] == 'go_to_and_center']
    assert [c[2] for c in descent_calls] == PAYLOAD_APPROACH_ALTITUDES_M
    assert all(c[1] == "MAVI_ALTIGEN" for c in descent_calls)

    # Order: all descent steps -> nudge -> (servo call happens between nudge
    # and climb, verified below) -> climb back to mission altitude, last.
    assert step_names[-1] == 'climb_to_altitude'
    assert centering.calls[-1] == ('climb_to_altitude', MISSION_ALTITUDE_M)
    nudge_idx = step_names.index('nudge_forward')
    assert nudge_idx == len(descent_calls)  # nudge immediately follows the last descent step

    assert ('release_payload_at_mavi_altigen', {}) in actuator.calls
    assert result is False  # verification marker not found -- best-effort, does not raise


@pytest.mark.asyncio
async def test_release_and_verify_selects_correct_actuator_method_for_kirmizi_ucgen():
    actuator = MockPayloadActuator()
    centering = _RecordingCentering()
    camera = MockCameraSource()
    detector = _NoMarkerDetector()

    service = PayloadReleaseService(actuator, detector, camera, centering, flight=None)

    await service.release_and_verify("KIRMIZI_UCGEN")

    assert ('release_payload_at_kirmizi_ucgen', {}) in actuator.calls
    assert ('release_payload_at_mavi_altigen', {}) not in actuator.calls


@pytest.mark.asyncio
async def test_release_and_verify_returns_true_when_marker_found():
    class _FindsMarkerDetector:
        async def detect(self, frame):
            return [Detection(shape_type="KIRMIZI_DIKDORTGEN", confidence=0.9,
                              center_px=(0, 0), bbox_px=(0, 0, 1, 1))]

    actuator = MockPayloadActuator()
    centering = _RecordingCentering()
    camera = MockCameraSource()
    service = PayloadReleaseService(actuator, _FindsMarkerDetector(), camera, centering, flight=None)

    result = await service.release_and_verify("MAVI_ALTIGEN")

    assert result is True


@pytest.mark.asyncio
async def test_release_and_verify_unknown_shape_skips_approach_entirely():
    actuator = MockPayloadActuator()
    centering = _RecordingCentering()
    camera = MockCameraSource()
    service = PayloadReleaseService(actuator, _NoMarkerDetector(), camera, centering, flight=None)

    result = await service.release_and_verify("BILINMEYEN_SEKIL")

    assert result is False
    assert centering.calls == []
    assert actuator.calls == []
