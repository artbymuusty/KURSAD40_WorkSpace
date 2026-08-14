import numpy as np
import cv2
import pytest

from core.detection.hsv_contour_detector import HSVContourDetector
from core.config.parameters import HSV_STREAK_FRAMES


def _frame_with_shapes() -> np.ndarray:
    frame = np.zeros((400, 400, 3), dtype=np.uint8)

    # Pure red (BGR) equilateral-ish triangle, top-left quadrant.
    tri_pts = np.array([[60, 40], [20, 120], [100, 120]], dtype=np.int32)
    cv2.fillPoly(frame, [tri_pts], (0, 0, 255))

    # Pure blue (BGR) regular hexagon, bottom-right quadrant.
    center = (280, 280)
    radius = 70
    hex_pts = np.array(
        [
            (
                int(center[0] + radius * np.cos(np.pi / 3 * i)),
                int(center[1] + radius * np.sin(np.pi / 3 * i)),
            )
            for i in range(6)
        ],
        dtype=np.int32,
    )
    cv2.fillPoly(frame, [hex_pts], (255, 0, 0))

    return frame


@pytest.mark.asyncio
async def test_detects_both_shapes_after_streak_confirmation():
    detector = HSVContourDetector()
    frame = _frame_with_shapes()

    detections = []
    for _ in range(HSV_STREAK_FRAMES):
        detections = await detector.detect(frame)

    shape_types = {d.shape_type for d in detections}
    assert "KIRMIZI_UCGEN" in shape_types
    assert "MAVI_ALTIGEN" in shape_types
    for d in detections:
        assert d.confidence >= 0.7


@pytest.mark.asyncio
async def test_single_frame_not_committed_before_streak():
    detector = HSVContourDetector()
    frame = _frame_with_shapes()

    detections = await detector.detect(frame)

    assert detections == [] or HSV_STREAK_FRAMES <= 1


@pytest.mark.asyncio
async def test_empty_frame_yields_no_detections():
    detector = HSVContourDetector()
    frame = np.zeros((400, 400, 3), dtype=np.uint8)

    detections = []
    for _ in range(HSV_STREAK_FRAMES):
        detections = await detector.detect(frame)

    assert detections == []


def _rotated_rect_points(center, size, angle_deg):
    rect = (center, size, angle_deg)
    return cv2.boxPoints(rect).astype(np.int32)


@pytest.mark.asyncio
async def test_detects_rotated_rectangle_with_orientation():
    """Görev 3 (operatör revizyonu, 2026-08-13): KIRMIZI_DIKDORTGEN tespiti
    rotation_deg doldurmalı, ve bu değer aynı çokgen üzerinde bağımsızca
    çalıştırılan cv2.minAreaRect ile tutarlı olmalı (OpenCV sürüm-özel açı
    kuralına bağlı kalmadan, kendi içinde tutarlılık doğrulaması)."""
    frame = np.zeros((400, 400, 3), dtype=np.uint8)
    center = (200.0, 200.0)
    size = (140.0, 60.0)  # w > h -- long edge is the "width" side
    angle_deg = 25.0
    pts = _rotated_rect_points(center, size, angle_deg)
    cv2.fillPoly(frame, [pts], (0, 0, 255))  # pure red (BGR)

    detector = HSVContourDetector()
    detections = []
    for _ in range(HSV_STREAK_FRAMES):
        detections = await detector.detect(frame)

    rect_dets = [d for d in detections if d.shape_type == "KIRMIZI_DIKDORTGEN"]
    assert len(rect_dets) == 1
    det = rect_dets[0]
    assert det.rotation_deg is not None

    # Ground truth: re-run minAreaRect independently on the same drawn
    # contour, using the exact same w>=h branch logic as _detect_rectangle.
    contour = pts.reshape(-1, 1, 2)
    (_, _), (rw, rh), gt_angle = cv2.minAreaRect(contour)
    expected = float(gt_angle) if rw >= rh else float(gt_angle) + 90.0

    assert abs(det.rotation_deg - expected) < 1.0


@pytest.mark.asyncio
async def test_triangle_and_hexagon_detections_have_no_rotation():
    """rotation_deg is only meaningful for rectangle detections."""
    detector = HSVContourDetector()
    frame = _frame_with_shapes()

    detections = []
    for _ in range(HSV_STREAK_FRAMES):
        detections = await detector.detect(frame)

    for d in detections:
        assert d.shape_type in ("KIRMIZI_UCGEN", "MAVI_ALTIGEN")
        assert d.rotation_deg is None
