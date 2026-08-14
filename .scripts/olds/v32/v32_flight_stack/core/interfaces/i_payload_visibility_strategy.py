"""
Görev 3 Rapor Bölüm 11, Madde A (AÇIK NOKTA) için strateji arayüzü.

Olası Stratejiler:
  (a) SingleFrameDualReferenceStrategy — tek karede her iki referansın da görünür olduğu
      varsayımına dayanır (yalnızca payload aracın altında yeterince öne/yana monte edilmişse
      mümkündür).
  (b) SequentialReferenceStrategy — önce hedefe göre konum sabitlenir (GPS ile kilitlenir),
      ardından kamera/irtifa değiştirilip yalnızca payload'a bakılarak son hizalama yapılır.
"""

from abc import ABC, abstractmethod
from typing import Optional
import numpy as np

# 'Detection' import edilecek, şimdilik type hint için bekliyoruz.
# core.detection.types'dan Detection'ı daha sonra import edebiliriz, ancak 
# circular dependency önlemek için any kullanabilir veya types.py'yi ekleyebiliriz.
from core.detection.types import Detection

class IPayloadVisibilityStrategy(ABC):

    @abstractmethod
    async def locate_target(self, detector: 'IDetector', camera_frame: np.ndarray) -> Detection:
        """Kırmızı Üçgen'i bulur."""
        pass
        
    @abstractmethod
    async def locate_carried_payload(self, detector: 'IDetector', camera_frame: np.ndarray) -> Optional[Detection]:
        """Taşınan Kırmızı Dikdörtgen'i bul; strateji uygulanamıyorsa None dönebilir."""
        pass
        
    @abstractmethod
    async def compute_alignment_yaw(self, target: Detection, payload: Optional[Detection]) -> float:
        """Hedef ve taşınan yüke göre hizalama yaw açısını hesaplar."""
        pass
