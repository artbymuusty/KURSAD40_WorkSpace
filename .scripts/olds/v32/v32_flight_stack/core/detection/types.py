from dataclasses import dataclass
from typing import Literal, Optional

@dataclass
class Detection:
    shape_type: Literal["MAVI_ALTIGEN", "KIRMIZI_UCGEN", "KIRMIZI_DIKDORTGEN", "MAVI_DIKDORTGEN"]
    confidence: float
    center_px: tuple[float, float]
    bbox_px: tuple[float, float, float, float]
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
