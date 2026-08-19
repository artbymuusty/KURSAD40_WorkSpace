"""
Operator revision (2026-08-13): release_and_verify() now owns the full
staged approach (descend through PAYLOAD_APPROACH_ALTITUDES_M, re-centering
at each step, forward nudge, servo call) and the post-drop climb back to
MISSION_ALTITUDE_M, not just the actuator call + verification it used to.
"""
import pytest

from mocks.mock_payload_actuator import MockPayloadActuator
from mocks.mock_camera_source import MockCameraSource

from core.detection.detection_feed import DetectionFeed
from core.detection.types import Detection
from core.mission.payload_release import PayloadReleaseService
from core.config.parameters import PAYLOAD_APPROACH_ALTITUDES_M, MISSION_ALTITUDE_M


class _RecordingCentering:
    """Records the exact sequence/arguments of centering calls
    PayloadReleaseService makes, without re-exercising CenteringController's
    own convergence physics (already covered by test_centering_controller.py)."""
    def __init__(self):
        self.calls: list = []

    async def go_to_and_center(self, shape_type: str, altitude_m: float,
                               alt_tolerance_m: float = None, aim_offset_body_m=None) -> bool:
        self.calls.append(('go_to_and_center', shape_type, altitude_m))
        return True

    async def descend_to_release(self, shape_type: str, altitude_m: float, mount_body_m):
        # PHASE 13 D3: the final step finishes with a mount-translated,
        # open-loop descent instead of a vision-biased one.
        self.calls.append(('descend_to_release', shape_type, altitude_m))
        return altitude_m

    async def nudge_forward(self, distance_m: float) -> None:
        self.calls.append(('nudge_forward', distance_m))

    async def climb_to_altitude(self, altitude_m: float) -> bool:
        self.calls.append(('climb_to_altitude', altitude_m))
        return True


def _feed(*detections) -> DetectionFeed:
    """ADR-008 B1: _verify_marker() now reads the orchestrator's detection
    feed instead of calling detect() itself. An empty feed is the
    "verification marker never seen" case (best-effort, non-blocking); one
    carrying the marker is the success case."""
    feed = DetectionFeed()
    feed.publish(list(detections))
    return feed


@pytest.mark.asyncio
async def test_release_and_verify_runs_staged_approach_then_servo_then_climb_back():
    actuator = MockPayloadActuator()
    centering = _RecordingCentering()
    camera = MockCameraSource()
    detection_feed = _feed()

    service = PayloadReleaseService(actuator, detection_feed, camera, centering, flight=None)

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
    # PHASE 13 D3: the final step is now two actions -- the vision-guided
    # centring above, then a mount-translated open-loop descent -- so the
    # nudge follows the descent SEQUENCE, not the go_to_and_center count.
    assert step_names[len(descent_calls)] == 'descend_to_release'
    assert centering.calls[len(descent_calls)][2] == PAYLOAD_APPROACH_ALTITUDES_M[-1]
    nudge_idx = step_names.index('nudge_forward')
    assert nudge_idx == len(descent_calls) + 1  # nudge immediately follows the descent

    assert ('release_payload_at_mavi_altigen', {}) in actuator.calls
    assert result is False  # verification marker not found -- best-effort, does not raise


@pytest.mark.asyncio
async def test_release_and_verify_selects_correct_actuator_method_for_kirmizi_ucgen():
    actuator = MockPayloadActuator()
    centering = _RecordingCentering()
    camera = MockCameraSource()
    detection_feed = _feed()

    service = PayloadReleaseService(actuator, detection_feed, camera, centering, flight=None)

    await service.release_and_verify("KIRMIZI_UCGEN")

    assert ('release_payload_at_kirmizi_ucgen', {}) in actuator.calls
    assert ('release_payload_at_mavi_altigen', {}) not in actuator.calls


@pytest.mark.asyncio
async def test_release_and_verify_returns_true_when_marker_found():
    actuator = MockPayloadActuator()
    centering = _RecordingCentering()
    camera = MockCameraSource()
    detection_feed = _feed(Detection(shape_type="KIRMIZI_DIKDORTGEN", confidence=0.9,
                                     center_px=(0, 0), bbox_px=(0, 0, 1, 1)))
    service = PayloadReleaseService(actuator, detection_feed, camera, centering, flight=None)

    result = await service.release_and_verify("MAVI_ALTIGEN")

    assert result is True


@pytest.mark.asyncio
async def test_release_and_verify_unknown_shape_skips_approach_entirely():
    actuator = MockPayloadActuator()
    centering = _RecordingCentering()
    camera = MockCameraSource()
    service = PayloadReleaseService(actuator, _feed(), camera, centering, flight=None)

    result = await service.release_and_verify("BILINMEYEN_SEKIL")

    assert result is False
    assert centering.calls == []
    assert actuator.calls == []


@pytest.mark.asyncio
async def test_verification_reports_not_found_when_the_detection_feed_is_stale():
    """ADR-008 B1: verification reads the orchestrator's detection loop. If
    that loop has stopped, the marker must read as NOT found rather than
    being re-confirmed from a frozen sample -- the same staleness rule the
    centering loop and the dashboard overlay follow."""
    import asyncio

    actuator = MockPayloadActuator()
    centering = _RecordingCentering()
    camera = MockCameraSource()
    detection_feed = DetectionFeed(stale_after_s=0.05)
    detection_feed.publish([Detection(shape_type="KIRMIZI_DIKDORTGEN", confidence=0.9,
                                      center_px=(0, 0), bbox_px=(0, 0, 1, 1))])
    service = PayloadReleaseService(actuator, detection_feed, camera, centering, flight=None)

    await asyncio.sleep(0.1)  # producer goes quiet
    result = await service.release_and_verify("MAVI_ALTIGEN")

    assert result is False
    # Görev 2 Rapor Bölüm 13: verification never gates mission flow -- the
    # servo still fired and the climb-back still happened.
    assert actuator.calls, "the payload must still have been released"
    assert ('climb_to_altitude', MISSION_ALTITUDE_M) in centering.calls
