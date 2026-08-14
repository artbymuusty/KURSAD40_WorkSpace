import asyncio
import logging
import os
import numpy as np
from typing import List
from core.interfaces.i_detector import IDetector
from core.detection.types import Detection
from core.config.parameters import YOLO_CONFIDENCE_THRESHOLD

logger = logging.getLogger(__name__)

# GAP FIX (Görev 2 Rapor Bölüm 5, 6): model loading was previously commented
# out entirely (self._model always None), so this detector could never see
# anything regardless of what model_path pointed to. This wires the actual
# load, guarded so a missing/invalid file or missing ultralytics dependency
# degrades to the same documented "no detections yet" behavior instead of
# crashing GCS startup -- no trained YOLO26 weights for the competition
# classes exist in this repo yet (only stock COCO-pretrained yolov8n.pt from
# earlier versions), so that safe fallback is still the common case today.
_ALLOWED_CLASSES = {"MAVI_ALTIGEN", "KIRMIZI_UCGEN", "KIRMIZI_DIKDORTGEN", "MAVI_DIKDORTGEN"}


class YoloDetector(IDetector):
    def __init__(self, model_path: str, confidence_threshold: float = YOLO_CONFIDENCE_THRESHOLD):
        self.confidence_threshold = confidence_threshold
        self.model_path = model_path
        self._model = self._load_model(model_path)

    def _load_model(self, model_path: str):
        if not os.path.isfile(model_path):
            logger.warning(
                f"YOLO model dosyasi bulunamadi: {model_path} -- tespit devre disi "
                "(bos liste donecek) taki gercek model saglanana kadar."
            )
            return None
        try:
            from ultralytics import YOLO
        except ImportError:
            logger.warning("ultralytics paketi kurulu degil -- tespit devre disi.")
            return None
        try:
            model = YOLO(model_path)
        except Exception as e:
            logger.error(f"YOLO model yuklenemedi ({model_path}): {e}")
            return None

        # FOOTGUN GUARD: the only weight file present in earlier versions of
        # this repo (yolov8n.pt) is stock COCO-pretrained and shares none of
        # our class names -- loading it "succeeds" but silently never detects
        # a real target (see flat vision_backends.py's UltralyticsYoloBackend,
        # which instead remapped COCO classes as stand-ins -- that produced
        # false positives on real people/cars and is exactly what we do NOT
        # want here). Warn loudly rather than let a placeholder model look
        # indistinguishable from a real one on competition day.
        model_classes = set(model.names.values()) if getattr(model, "names", None) else set()
        if model_classes and not (model_classes & _ALLOWED_CLASSES):
            logger.warning(
                f"YOLO model '{model_path}' sinif isimleri beklenen hedeflerle "
                f"ORTUSMUYOR (model siniflari: {sorted(model_classes)[:10]}...). "
                "Bu muhtemelen egitimsiz/placeholder bir model (orn. stok COCO "
                "yolov8n.pt) -- gercek tespit uretmeyecek. Yarisma oncesi "
                "MAVI_ALTIGEN/KIRMIZI_UCGEN/KIRMIZI_DIKDORTGEN/MAVI_DIKDORTGEN "
                "siniflariyla egitilmis gercek bir YOLO26 modeliyle degistirin."
            )
        return model

    async def detect(self, frame: np.ndarray) -> List[Detection]:
        if self._model is None:
            # Fallback for now since model is not loaded yet
            return []

        # BUG FIX: real inference is a synchronous, CPU/GPU-bound call with
        # no internal awaits -- calling it directly here would block the
        # single asyncio event loop for the full inference duration, which
        # would also stall the camera-feed publish loop and everything else
        # sharing this loop (mission ticks, MAVSDK telemetry) for that same
        # window. Running it in a worker thread keeps the live camera feed
        # (and everything else) responsive regardless of inference latency
        # -- "frames must keep arriving even while detection is running."
        loop = asyncio.get_running_loop()
        results = await loop.run_in_executor(None, lambda: self._model(frame, verbose=False))
        detections = []

        # BUG FIX: Yalnızca MAVI_ALTIGEN/KIRMIZI_UCGEN'e izin vermek, payload_release.py'nin
        # yük doğrulaması (Bölüm 13) için aradığı KIRMIZI_DIKDORTGEN/MAVI_DIKDORTGEN
        # tespitlerini görmezden geliyordu — doğrulama bu yüzden her zaman başarısız oluyordu.
        # Detection.shape_type (types.py) zaten 4 sınıfı da tanımlıyor; allowlist bununla
        # tutarlı olmalı.
        allowed_classes = _ALLOWED_CLASSES

        for result in results:
            boxes = result.boxes
            for box in boxes:
                conf = float(box.conf[0])
                if conf < self.confidence_threshold:
                    continue
                    
                cls_id = int(box.cls[0])
                cls_name = result.names[cls_id]
                
                if cls_name not in allowed_classes:
                    continue
                    
                xyxy = box.xyxy[0].cpu().numpy()
                center_px = (float((xyxy[0] + xyxy[2]) / 2), float((xyxy[1] + xyxy[3]) / 2))
                bbox_px = (float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3]))
                
                detections.append(Detection(
                    shape_type=cls_name,
                    confidence=conf,
                    center_px=center_px,
                    bbox_px=bbox_px
                ))
                
        return detections
