"""
Görev 2 Rapor Bölüm 5, 6: IDetector'ün ikinci, gerçek (ML-olmayan) bir
implementasyonu. yolo_detector.py takımın gerçek YOLO26 modelini bekliyor;
bu dosya, klasik HSV-eşik + kontur şekil tespiti kullanarak MAVI_ALTIGEN /
KIRMIZI_UCGEN için BUGÜN çalışan bir alternatif sağlar. Doğrulama işaretleri
KIRMIZI_DIKDORTGEN / MAVI_DIKDORTGEN de (Görev 3 Rapor, operatör revizyonu
2026-08-13) aynı renk maskeleri üzerinde 4 köşeli poligon aranarak tespit
edilir -- bkz. _detect_rectangle, rotation_deg de hesaplar (Görev 3'ün
dikdörtgene dik yaklaşım hizalaması için).

.scripts/v29.py ve .scripts/olds/v32/vision_backends.py::HSVContourDetectorBackend
içindeki kanıtlanmış çalışan mantığın birebir portu (renk eşikleri, morfoloji,
poligon yaklaşıklığı, renk-doluluk oranı, N-kare "streak" onayı) -- sadece
çıktı tipi Detection (types.py) ve sınıf isimleri MAVI_ALTIGEN/KIRMIZI_UCGEN
olacak şekilde uyarlandı. GCS tarafında çalışır (Bölüm 16 kısıtı: drone
üzerinde görüntü işleme yok), tıpkı YoloDetector gibi.
"""

import logging
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from core.config.parameters import (
    HSV_BLUE_HI,
    HSV_BLUE_LO,
    HSV_COLOR_FRAC_HEX,
    HSV_COLOR_FRAC_RECT,
    HSV_COLOR_FRAC_TRI_BASE,
    HSV_EPS_HEX,
    HSV_EPS_RECT_MAX,
    HSV_EPS_RECT_MIN,
    HSV_EPS_TRI_MAX,
    HSV_EPS_TRI_MIN,
    HSV_MIN_AREA_HEX_BASE,
    HSV_MIN_AREA_RECT_BASE,
    HSV_MIN_AREA_TRI_BASE,
    HSV_RED_HI_1,
    HSV_RED_HI_2,
    HSV_RED_LO_1,
    HSV_RED_LO_2,
    HSV_STREAK_DIST_PX,
    HSV_STREAK_FRAMES,
)
from core.detection.types import Detection
from core.interfaces.i_detector import IDetector

logger = logging.getLogger(__name__)


class HSVContourDetector(IDetector):
    def __init__(self):
        self._last_center: Dict[str, Optional[Tuple[int, int]]] = {
            "triangle": None, "hexagon": None, "red_rect": None, "blue_rect": None,
        }
        self._streak: Dict[str, int] = {"triangle": 0, "hexagon": 0, "red_rect": 0, "blue_rect": 0}

    @staticmethod
    def _preprocess(frame_bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = cv2.createCLAHE(2.0, (8, 8)).apply(v)
        return cv2.merge([h, s, v])

    @staticmethod
    def _color_masks(hsv: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        red1 = cv2.inRange(hsv, HSV_RED_LO_1, HSV_RED_HI_1)
        red2 = cv2.inRange(hsv, HSV_RED_LO_2, HSV_RED_HI_2)
        red = cv2.bitwise_or(red1, red2)
        blue = cv2.inRange(hsv, HSV_BLUE_LO, HSV_BLUE_HI)

        k = np.ones((5, 5), np.uint8)
        red = cv2.morphologyEx(red, cv2.MORPH_OPEN, k, 1)
        red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, k, 2)
        blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, k, 1)
        blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, k, 2)
        return red, blue

    @staticmethod
    def _center_of(poly) -> Tuple[int, int]:
        m = cv2.moments(poly)
        if m["m00"] != 0:
            return (int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]))
        x, y, w, h = cv2.boundingRect(poly)
        return (x + w // 2, y + h // 2)

    @staticmethod
    def _min_side_len(poly) -> float:
        pts = [poly[i][0] for i in range(len(poly))]
        lens = [np.linalg.norm(pts[i] - pts[(i + 1) % len(pts)]) for i in range(len(pts))]
        return min(lens) if lens else 0.0

    def _detect_rectangle(self, mask: np.ndarray, area_scale: float,
                          shape_type: str, streak_key: str) -> Optional[Detection]:
        """Görev 3 (operatör revizyonu, 2026-08-13): KIRMIZI_DIKDORTGEN/
        MAVI_DIKDORTGEN tespiti -- aynı renk maskeleri üzerinde 4 köşeli
        dışbükey poligon arar (üçgen=3, altıgen=6 mantığının aynısı), ve
        cv2.minAreaRect ile uzun kenarın açısını (rotation_deg) hesaplar --
        RectangleAlignmentStrategy'nin dik yaklaşım hesaplaması için."""
        best = None
        best_score = -1.0
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) < (HSV_MIN_AREA_RECT_BASE * area_scale):
                continue
            peri = cv2.arcLength(cnt, True)
            for eps in np.linspace(HSV_EPS_RECT_MIN, HSV_EPS_RECT_MAX, 5):
                approx = cv2.approxPolyDP(cnt, eps * peri, True)
                if len(approx) != 4 or not cv2.isContourConvex(approx):
                    continue
                if self._min_side_len(approx) < 8:
                    continue
                rect_area = abs(cv2.contourArea(approx))
                if rect_area <= 0:
                    continue

                poly_mask = np.zeros(mask.shape, dtype=np.uint8)
                cv2.fillPoly(poly_mask, [approx], 255)
                colored = cv2.countNonZero(cv2.bitwise_and(mask, mask, mask=poly_mask))
                area_px = cv2.countNonZero(poly_mask)
                frac = 0.0 if area_px == 0 else colored / float(area_px)
                if frac < HSV_COLOR_FRAC_RECT:
                    continue

                score = rect_area * frac
                if score > best_score:
                    best_score = score
                    x, y, bw, bh = cv2.boundingRect(approx)
                    center = self._center_of(approx)
                    # minAreaRect angle convention (OpenCV >=4.5): angle is
                    # the rotation of the box's "width" side from horizontal,
                    # in [0, 90). Long edge is width if w>=h (angle as-is),
                    # otherwise the long edge is the height side (+90).
                    (_, _), (rw, rh), angle = cv2.minAreaRect(approx)
                    long_edge_deg = float(angle) if rw >= rh else float(angle) + 90.0
                    best = Detection(
                        shape_type=shape_type,
                        confidence=0.9,  # see triangle/hexagon comment above -- deterministic classifier
                        center_px=(float(center[0]), float(center[1])),
                        bbox_px=(float(x), float(y), float(x + bw), float(y + bh)),
                        rotation_deg=long_edge_deg,
                    )

        if best and self._update_streak(streak_key, (int(best.center_px[0]), int(best.center_px[1]))):
            return best
        return None

    def _update_streak(self, name: str, center: Tuple[int, int]) -> bool:
        last = self._last_center[name]
        if last is not None and np.hypot(center[0] - last[0], center[1] - last[1]) <= HSV_STREAK_DIST_PX:
            self._streak[name] += 1
        else:
            self._streak[name] = 1
        self._last_center[name] = center
        return self._streak[name] >= HSV_STREAK_FRAMES

    async def detect(self, frame: np.ndarray) -> List[Detection]:
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = self._preprocess(blurred)
        red_mask, blue_mask = self._color_masks(hsv)

        h, w = frame.shape[:2]
        area_scale = (w * h) / float(1280 * 960)

        detections: List[Detection] = []

        best_tri = None
        best_tri_score = -1.0
        contours_r, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_r:
            if cv2.contourArea(cnt) < (HSV_MIN_AREA_TRI_BASE * area_scale):
                continue
            peri = cv2.arcLength(cnt, True)
            for eps in np.linspace(HSV_EPS_TRI_MIN, HSV_EPS_TRI_MAX, 6):
                approx = cv2.approxPolyDP(cnt, eps * peri, True)
                if len(approx) != 3 or not cv2.isContourConvex(approx):
                    continue
                if self._min_side_len(approx) < 8:
                    continue
                tri_area = abs(cv2.contourArea(approx))
                if tri_area <= 0:
                    continue

                poly_mask = np.zeros(red_mask.shape, dtype=np.uint8)
                cv2.fillPoly(poly_mask, [approx], 255)
                colored = cv2.countNonZero(cv2.bitwise_and(red_mask, red_mask, mask=poly_mask))
                area_px = cv2.countNonZero(poly_mask)
                frac = 0.0 if area_px == 0 else colored / float(area_px)
                frac_thr = max(0.20, HSV_COLOR_FRAC_TRI_BASE - 0.10 * np.log10(max(1.0, tri_area / 1500.0)))
                if frac < frac_thr:
                    continue

                score = tri_area * (0.7 * frac + 0.3 * 0.5)
                if score > best_tri_score:
                    best_tri_score = score
                    x, y, bw, bh = cv2.boundingRect(approx)
                    center = self._center_of(approx)
                    # Confidence is a fixed 0.9, unchanged from the ported
                    # original: this is a deterministic geometric/color-fill
                    # classifier, not a probabilistic model, so there is no
                    # real per-detection confidence signal to report -- 0.9
                    # is just comfortably above YOLO_CONFIDENCE_THRESHOLD
                    # (0.70) for any detection that already passed every
                    # shape/color-fill/streak gate above.
                    best_tri = Detection(
                        shape_type="KIRMIZI_UCGEN",
                        confidence=0.9,
                        center_px=(float(center[0]), float(center[1])),
                        bbox_px=(float(x), float(y), float(x + bw), float(y + bh)),
                    )
        if best_tri and self._update_streak("triangle", (int(best_tri.center_px[0]), int(best_tri.center_px[1]))):
            detections.append(best_tri)

        best_hex = None
        best_hex_score = -1.0
        contours_b, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_b:
            if cv2.contourArea(cnt) < (HSV_MIN_AREA_HEX_BASE * area_scale):
                continue
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, HSV_EPS_HEX * peri, True)
            if len(approx) != 6 or not cv2.isContourConvex(approx):
                continue
            if self._min_side_len(approx) < 8:
                continue

            poly_mask = np.zeros(blue_mask.shape, dtype=np.uint8)
            cv2.fillPoly(poly_mask, [approx], 255)
            colored = cv2.countNonZero(cv2.bitwise_and(blue_mask, blue_mask, mask=poly_mask))
            area_px = cv2.countNonZero(poly_mask)
            frac = 0.0 if area_px == 0 else colored / float(area_px)
            if frac < HSV_COLOR_FRAC_HEX:
                continue

            area = cv2.contourArea(approx)
            score = area * frac
            if score > best_hex_score:
                best_hex_score = score
                x, y, bw, bh = cv2.boundingRect(approx)
                center = self._center_of(approx)
                best_hex = Detection(
                    shape_type="MAVI_ALTIGEN",
                    confidence=0.9,
                    center_px=(float(center[0]), float(center[1])),
                    bbox_px=(float(x), float(y), float(x + bw), float(y + bh)),
                )
        if best_hex and self._update_streak("hexagon", (int(best_hex.center_px[0]), int(best_hex.center_px[1]))):
            detections.append(best_hex)

        red_rect = self._detect_rectangle(red_mask, area_scale, "KIRMIZI_DIKDORTGEN", "red_rect")
        if red_rect:
            detections.append(red_rect)

        blue_rect = self._detect_rectangle(blue_mask, area_scale, "MAVI_DIKDORTGEN", "blue_rect")
        if blue_rect:
            detections.append(blue_rect)

        return detections
