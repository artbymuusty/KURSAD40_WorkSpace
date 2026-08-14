import json
import os
import time
import logging
from typing import Tuple, Optional, List
from core.detection.types import TargetPoint
from core.config.parameters import YOLO_CONFIDENCE_THRESHOLD
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger(__name__)

class PositionStore:
    def __init__(self, storage_path: str = "mission_positions.json", publisher: EventPublisher = NULL_PUBLISHER):
        self.storage_path = storage_path
        self._points: List[TargetPoint] = []
        self.publisher = publisher

        # Load existing if exists
        if os.path.exists(self.storage_path):
            try:
                with open(self.storage_path, 'r') as f:
                    data = json.load(f)
                    for item in data:
                        self._points.append(TargetPoint(**item))
            except Exception as e:
                logger.error(f"Eski konum dosyasını yüklerken hata: {e}")

    def _publish(self, code, message="", severity=Severity.INFO, data=None):
        self.publisher.publish(Event(
            code=code, subsystem="PositionStore", category=Category.VISION,
            severity=severity, message=message, data=data or {},
        ))

    def try_save(
        self,
        shape_type: str,
        confidence: float,
        is_centered: bool,
        hover_completed: bool,
        gps: Tuple[float, float, float],
        detection_order: str,   # "ilk" | "ikinci"
    ) -> Optional[TargetPoint]:
        """Üç ön koşuldan biri bile sağlanmamışsa None döner ve KAYIT YAPMAZ.
        Hepsi sağlanmışsa TargetPoint oluşturur, iç listeye ekler, JSON dosyasına
        yazar (append, mevcut kayıtları KORUYARAK) ve TargetPoint'i döner."""

        if confidence < YOLO_CONFIDENCE_THRESHOLD:
            logger.warning(f"Kayıt reddedildi ({shape_type}): Güven skoru yetersiz ({confidence} < {YOLO_CONFIDENCE_THRESHOLD})")
            self._publish("GPS_SAVE_REJECTED", f"{shape_type}: confidence {confidence} < {YOLO_CONFIDENCE_THRESHOLD}",
                          severity=Severity.WARN, data={"shape_type": shape_type, "reason": "confidence"})
            return None

        if not is_centered:
            logger.warning(f"Kayıt reddedildi ({shape_type}): Hedef merkezde değil")
            self._publish("GPS_SAVE_REJECTED", f"{shape_type}: not centered", severity=Severity.WARN,
                          data={"shape_type": shape_type, "reason": "not_centered"})
            return None

        if not hover_completed:
            logger.warning(f"Kayıt reddedildi ({shape_type}): 2sn Hover tamamlanmadı")
            self._publish("GPS_SAVE_REJECTED", f"{shape_type}: hover incomplete", severity=Severity.WARN,
                          data={"shape_type": shape_type, "reason": "hover_incomplete"})
            return None

        point = TargetPoint(
            shape_type=shape_type,
            gps_lat=gps[0],
            gps_lon=gps[1],
            gps_alt=gps[2],
            detection_order=detection_order,
            timestamp=time.time(),
            payload_released=False
        )

        self._points.append(point)
        self._save_to_file()
        logger.info(f"Konum basariyla kaydedildi: {shape_type} ({detection_order})")
        self._publish("GPS_SAVE_CONFIRMED", f"{shape_type} ({detection_order})",
                      data={"shape_type": shape_type, "detection_order": detection_order})
        return point

    def _save_to_file(self) -> None:
        try:
            with open(self.storage_path, 'w') as f:
                json.dump([p.__dict__ for p in self._points], f, indent=4)
        except Exception as e:
            logger.error(f"Konum dosyaya yazılırken hata: {e}")
            self._publish("POSITION_STORE_WRITE_FAILED", str(e), severity=Severity.CRITICAL,
                          data={"path": self.storage_path})

    def get(self, shape_type: str) -> Optional[TargetPoint]:
        """Belirtilen şekil tipindeki en son kaydedilen noktayı döndürür."""
        for p in reversed(self._points):
            if p.shape_type == shape_type:
                return p
        return None

    def mark_payload_released(self, shape_type: str) -> None:
        """Belirtilen şekil için yük bırakıldı bayrağını True yapar."""
        for p in self._points:
            if p.shape_type == shape_type:
                p.payload_released = True
        self._save_to_file()

    def all_points(self) -> List[TargetPoint]:
        return self._points

    def both_required_targets_found(self) -> bool:
        """Görev 2 Rapor (operatör revizyonu, 2026-08-13): Search-completion
        invariant'ının TEK doğruluk kaynağı -- MAVI_ALTIGEN VE KIRMIZI_UCGEN
        ikisi de kaydedilmiş olmalı (OR değil, AND). Ayrı blue_found/
        red_found bayrakları eklemek yerine (ikinci bir hedef veritabanı
        anlamına gelir) mevcut PositionStore zaten bu durumu tutuyor --
        bu yalnızca onu okunabilir bir invariant olarak ifade eder."""
        return self.get("MAVI_ALTIGEN") is not None and self.get("KIRMIZI_UCGEN") is not None
