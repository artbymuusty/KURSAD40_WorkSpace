"""D2c: one contour must not commit as two classes.

Measured over 1741 live frames on 2026-08-17: 122 of them emitted
KIRMIZI_DIKDORTGEN *and* KIRMIZI_UCGEN together, from the SAME single red
blob -- identical area distributions (median 512 px2 in both sets), 3
vertices at eps 0.03, and not one touching the frame edge. No frame in the
entire flight contained more than one red or more than one blue blob, so
the payload contributed nothing: the arena triangle was committing as a
rectangle as well as a triangle, and Görev 3's pickup target IS a rectangle.

The cause is two shape gates sweeping different eps ranges over one
contour: 3 vertices over the triangle's 0.03-0.09, 4 over the rectangle's
tighter 0.02-0.06 where a rounded corner splits in two.
"""
import cv2
import numpy as np
import pytest

from core.detection.hsv_contour_detector import HSVContourDetector
from core.detection.types import Detection


def _triangle_frame(side_px=120, centre=(640, 480), colour=(0, 0, 220)):
    """A solid red triangle on grey, big enough to clear every area gate."""
    frame = np.full((960, 1280, 3), 120, dtype=np.uint8)
    cx, cy = centre
    h = int(side_px * 0.866)
    pts = np.array([[cx, cy - 2 * h // 3],
                    [cx - side_px // 2, cy + h // 3],
                    [cx + side_px // 2, cy + h // 3]], dtype=np.int32)
    cv2.fillPoly(frame, [pts], colour)
    return frame


def _det(shape_type, centre, bbox):
    return Detection(shape_type=shape_type, confidence=0.9,
                     center_px=centre, bbox_px=bbox)


def test_rectangle_on_top_of_a_committed_triangle_is_suppressed():
    """THE regression: a rectangle whose centre sits inside an already
    committed triangle is that same triangle, not a second object."""
    tri = _det("KIRMIZI_UCGEN", (640.0, 480.0), (580.0, 420.0, 700.0, 540.0))
    rect = _det("KIRMIZI_DIKDORTGEN", (641.0, 483.0), (581.0, 421.0, 699.0, 539.0))
    assert HSVContourDetector._overlaps_committed(rect, [tri]) is True


def test_a_genuinely_separate_rectangle_still_commits():
    """The suppression must not blind Görev 3: a rectangle somewhere else
    in the frame is a real second object and has to survive."""
    tri = _det("KIRMIZI_UCGEN", (300.0, 300.0), (240.0, 240.0, 360.0, 360.0))
    rect = _det("KIRMIZI_DIKDORTGEN", (900.0, 700.0), (840.0, 640.0, 960.0, 760.0))
    assert HSVContourDetector._overlaps_committed(rect, [tri]) is False


def test_no_committed_shapes_means_nothing_to_suppress():
    rect = _det("KIRMIZI_DIKDORTGEN", (640.0, 480.0), (580.0, 420.0, 700.0, 540.0))
    assert HSVContourDetector._overlaps_committed(rect, []) is False


@pytest.mark.asyncio
async def test_a_lone_triangle_never_also_commits_as_a_rectangle():
    """End to end through the real detector, on a synthetic frame holding
    exactly one red triangle. Whatever else it reports, it must not report
    a rectangle in the same place -- that is what sent the vehicle after the
    wrong class."""
    det = HSVContourDetector()
    frame = _triangle_frame()
    classes = []
    # Streak gates need consecutive frames before anything commits.
    for _ in range(6):
        classes = [d.shape_type for d in await det.detect(frame)]

    assert "KIRMIZI_DIKDORTGEN" not in classes, classes
    assert "MAVI_DIKDORTGEN" not in classes, classes
