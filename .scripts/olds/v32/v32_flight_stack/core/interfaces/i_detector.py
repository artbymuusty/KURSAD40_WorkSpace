"""
Gerçek YOLO26 model çağrısı bunun somut bir implementasyonudur; bu backend'den
bağımsızdır çünkü tespit her zaman GCS bilgisayarında çalışır (Görev 2 Rapor Bölüm 16,
Geliştirme Kısıtları: 'Drone üzerinde görüntü işleme çalıştırılmayacaktır').
"""

from abc import ABC, abstractmethod
import numpy as np
from core.detection.types import Detection

class IDetector(ABC):
    
    @abstractmethod
    async def detect(self, frame: np.ndarray) -> list[Detection]:
        """Kare üzerinde nesne tespiti yapar."""
        pass
