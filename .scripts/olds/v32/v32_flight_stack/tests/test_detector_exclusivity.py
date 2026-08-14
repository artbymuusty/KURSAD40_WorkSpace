"""
Regression guard for a race condition found during continuous audit
(2026-08-13): _detection_loop() and precision-control consumers
(CenteringController.go_to_and_center, PayloadReleaseService._verify_marker)
used to independently call self.detector.detect() on the same detector
instance while both were active concurrently. HSVContourDetector's mutable
per-shape streak state (_last_center/_streak) depends on being fed a
coherent sequence of consecutive frames -- two interleaved, independently-
scheduled callers corrupt that sequence, making streak-commit timing
scheduling-order-dependent instead of deterministic.
"""
import asyncio
import pytest

from mocks.mock_camera_source import MockCameraSource
from core.mission.gorev2_orchestrator import Gorev2Orchestrator
from core.telemetry.event_bus import NULL_PUBLISHER


class _CountingDetector:
    def __init__(self):
        self.call_count = 0

    async def detect(self, frame):
        self.call_count += 1
        return []


def _bare_orchestrator(detector) -> Gorev2Orchestrator:
    """Bypasses the full constructor -- _detection_loop only touches
    self.camera/self.detector/self._latest_detections/self._precision_control_active/self.publisher."""
    orch = Gorev2Orchestrator.__new__(Gorev2Orchestrator)
    orch.camera = MockCameraSource()
    orch.detector = detector
    orch._latest_detections = []
    orch._precision_control_active = False
    orch._vision_consecutive_failures = 0
    orch.publisher = NULL_PUBLISHER
    return orch


@pytest.mark.asyncio
async def test_detection_loop_skips_detect_while_precision_control_active():
    detector = _CountingDetector()
    orch = _bare_orchestrator(detector)
    orch._precision_control_active = True

    task = asyncio.ensure_future(orch._detection_loop())
    await asyncio.sleep(0.35)  # a few 0.1s ticks
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass

    assert detector.call_count == 0


@pytest.mark.asyncio
async def test_detection_loop_calls_detect_when_not_reserved():
    detector = _CountingDetector()
    orch = _bare_orchestrator(detector)
    orch._precision_control_active = False

    task = asyncio.ensure_future(orch._detection_loop())
    await asyncio.sleep(0.35)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass

    assert detector.call_count >= 2


@pytest.mark.asyncio
async def test_detection_loop_stops_calling_detect_once_reservation_acquired_mid_flight():
    """The flag can flip while the loop is already running (a pursuit
    starting mid-search) -- the very next iteration must respect it."""
    detector = _CountingDetector()
    orch = _bare_orchestrator(detector)

    task = asyncio.ensure_future(orch._detection_loop())
    await asyncio.sleep(0.25)
    count_before_reservation = detector.call_count
    assert count_before_reservation > 0

    orch._precision_control_active = True
    await asyncio.sleep(0.35)
    count_after_reservation = detector.call_count

    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass

    assert count_after_reservation == count_before_reservation
