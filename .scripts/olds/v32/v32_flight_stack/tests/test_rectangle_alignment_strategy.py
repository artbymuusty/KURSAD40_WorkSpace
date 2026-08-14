import numpy as np
import pytest

from core.detection.types import Detection
from core.mission.rectangle_alignment_strategy import RectangleAlignmentStrategy


class _StubDetector:
    def __init__(self, detections):
        self._detections = detections

    async def detect(self, frame):
        return self._detections


@pytest.mark.asyncio
async def test_locate_target_finds_kirmizi_dikdortgen():
    target = Detection(shape_type="KIRMIZI_DIKDORTGEN", confidence=0.9,
                       center_px=(100, 100), bbox_px=(80, 80, 120, 120), rotation_deg=10.0)
    detector = _StubDetector([target])
    strategy = RectangleAlignmentStrategy()

    found = await strategy.locate_target(detector, np.zeros((10, 10, 3), dtype=np.uint8))

    assert found is target


@pytest.mark.asyncio
async def test_locate_target_raises_when_not_found():
    detector = _StubDetector([])
    strategy = RectangleAlignmentStrategy()

    with pytest.raises(RuntimeError):
        await strategy.locate_target(detector, np.zeros((10, 10, 3), dtype=np.uint8))


@pytest.mark.asyncio
async def test_locate_carried_payload_always_none():
    strategy = RectangleAlignmentStrategy()
    result = await strategy.locate_carried_payload(_StubDetector([]), np.zeros((10, 10, 3), dtype=np.uint8))
    assert result is None


@pytest.mark.asyncio
async def test_compute_alignment_yaw_adds_90_and_normalizes():
    strategy = RectangleAlignmentStrategy()
    target = Detection(shape_type="KIRMIZI_DIKDORTGEN", confidence=0.9,
                       center_px=(0, 0), bbox_px=(0, 0, 1, 1), rotation_deg=10.0)

    delta = await strategy.compute_alignment_yaw(target, None)

    # 10 + 90 = 100, normalized into [-90, 90) -> 100 - 180 = -80
    assert delta == pytest.approx(-80.0)


@pytest.mark.asyncio
async def test_compute_alignment_yaw_raises_without_rotation():
    strategy = RectangleAlignmentStrategy()
    target = Detection(shape_type="KIRMIZI_DIKDORTGEN", confidence=0.9,
                       center_px=(0, 0), bbox_px=(0, 0, 1, 1), rotation_deg=None)

    with pytest.raises(RuntimeError):
        await strategy.compute_alignment_yaw(target, None)
