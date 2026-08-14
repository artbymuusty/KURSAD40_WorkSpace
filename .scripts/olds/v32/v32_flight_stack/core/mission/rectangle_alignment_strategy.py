"""
Görev 3 Rapor Bölüm 5 (operatör revizyonu, 2026-08-13): IPayloadVisibilityStrategy'nin
ilk gerçek implementasyonu. UnimplementedVisibilityStrategy'nin kendi
docstring'i bunun "kapsam dışı, burada icat edilecek bir şey değil"
olduğunu söylüyordu -- ancak operatör artık Görev 3 algoritmasını bizzat
tanımladığı için bu yetkilendirilmiş bir kapsam genişlemesidir, icat edilen
bir mimari değil.

Kullanım: Gorev3PickupPhase, Kırmızı Dikdörtgen'e (1. yükün bırakıldığı
şekil) dik yaklaşım için bu stratejiyi kullanır -- 'SequentialReferenceStrategy'
tarzı (bkz. IPayloadVisibilityStrategy docstring): önce GPS ile Mavi Altıgen
konumuna gidilir, ardından yalnızca kamera/dikdörtgen'e bakılarak son
hizalama yapılır. compute_alignment_yaw tek referanslıdır (yalnızca
target.rotation_deg kullanılır) -- locate_carried_payload bu tasarımda
gerekli değildir, her zaman None döner.
"""
import logging
from typing import Optional

import numpy as np

from core.detection.types import Detection
from core.interfaces.i_payload_visibility_strategy import IPayloadVisibilityStrategy

logger = logging.getLogger(__name__)


class RectangleAlignmentStrategy(IPayloadVisibilityStrategy):
    async def locate_target(self, detector, camera_frame: np.ndarray) -> Detection:
        """Kırmızı Dikdörtgen'i bu karede arar. Bulunamazsa RAISE eder --
        çağıran taraf (Gorev3PickupPhase) bir sonraki karede tekrar dener,
        tıpkı go_to_and_center()'ın kendi 'hedef kayboldu' döngüsü gibi."""
        detections = await detector.detect(camera_frame)
        for d in detections:
            if d.shape_type == "KIRMIZI_DIKDORTGEN":
                return d
        raise RuntimeError("KIRMIZI_DIKDORTGEN bu karede bulunamadi")

    async def locate_carried_payload(self, detector, camera_frame: np.ndarray) -> Optional[Detection]:
        """Bu tasarımda kullanılmaz -- dik yaklaşım hizalaması yalnızca
        hedefin kendi yönelimine (target.rotation_deg) dayanır, ikinci bir
        referansa ihtiyaç duymaz."""
        return None

    async def compute_alignment_yaw(self, target: Detection, payload: Optional[Detection]) -> float:
        """target.rotation_deg, sabit nadir kamera montajı nedeniyle GÖVDE
        eksenlerine göre ölçülmüş uzun kenar açısıdır (bkz.
        HSVContourDetector._detect_rectangle). Dik yaklaşım için +90 derece
        döndürülür.

        DÖNÜŞ DEĞERİ GÖRECELİDİR (mutlak pusula yönü DEĞİL) -- aracın
        MEVCUT yaw'ına (flight.get_yaw_deg()) eklenmesi çağıranın
        sorumluluğundadır. Dik yaklaşım simetriktir (uzun kenarın hangi
        ucundan yaklaşıldığı önemsizdir), bu yüzden sonuç en kısa dönüşü
        veren [-90, 90) aralığına normalize edilir."""
        if target.rotation_deg is None:
            raise RuntimeError(
                "target.rotation_deg is None -- detector bu tespit için yönelim saglamadi "
                "(yalnızca dikdörtgen tespitlerinde dolu olur, ucgen/altigen icin degil)."
            )
        perpendicular_deg = target.rotation_deg + 90.0
        return ((perpendicular_deg + 90.0) % 180.0) - 90.0
