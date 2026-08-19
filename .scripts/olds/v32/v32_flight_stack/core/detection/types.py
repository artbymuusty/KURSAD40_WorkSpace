from dataclasses import dataclass
from typing import Literal, Optional, Sequence, Tuple

# ADR-010 P5 / detector adapter contract. A detector produces Detection
# values into DetectionFeed; nothing downstream knows which detector ran.
# The contract is exactly these fields:
#
#   shape_type   -- one of the four class names below
#   confidence   -- 0..1
#   center_px    -- the CENTRE SOURCE the controller steers on. HSV supplies
#                   the contour-moment centre (and, below
#                   LOW_ALT_VISION_LIMIT_M, the controller substitutes the
#                   bbox centre itself -- see ADR-010 P1); a YOLO adapter
#                   may supply the box centre directly.
#   bbox_px      -- always present, always (x1, y1, x2, y2)
#   contour_px   -- OPTIONAL polygon. Present for HSV (its own approxPolyDP
#                   result). A YOLO adapter supplies its segmentation
#                   polygon if it has one and leaves this None if it does
#                   not; the overlay then strokes the bbox instead, in the
#                   same green. Nothing in the control law reads it.
#   rotation_deg -- optional, rectangles only.
ContourPx = Sequence[Tuple[float, float]]

@dataclass
class Detection:
    shape_type: Literal["MAVI_ALTIGEN", "KIRMIZI_UCGEN", "KIRMIZI_DIKDORTGEN", "MAVI_DIKDORTGEN"]
    confidence: float
    center_px: tuple[float, float]
    bbox_px: tuple[float, float, float, float]
    # ADR-010 P5: the detected shape's own outline, so the overlay can draw
    # a triangle as a triangle and a hexagon as a hexagon instead of a
    # bounding rectangle. Optional by contract -- see the note above.
    contour_px: Optional[ContourPx] = None
    # Görev 3 Rapor (operatör revizyonu, 2026-08-13): dikdörtgen hedeflerin
    # (KIRMIZI_DIKDORTGEN/MAVI_DIKDORTGEN) uzun kenarına dik yaklaşım için
    # gereken yönelim açısı (derece, cv2.minAreaRect). Üçgen/altıgen
    # tespitleri ve YOLO tabanlı tespitler için anlamsızdır -- None kalır.
    rotation_deg: Optional[float] = None

@dataclass
class TargetPoint:
    shape_type: str
    gps_lat: float
    gps_lon: float
    gps_alt: float
    detection_order: Literal["ilk", "ikinci"]
    timestamp: float
    payload_released: bool = False
