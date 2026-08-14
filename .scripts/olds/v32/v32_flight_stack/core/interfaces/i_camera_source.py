"""
real_system implementasyonu gerçek kamera cihazından (OpenCV/GStreamer),
gz_system implementasyonu Gazebo kamera topiğinden (ROS2 + cv_bridge) kare okur.
Şartname gereği yalnızca TEK kamera kullanılabilir (Görev 2 Rapor Bölüm 5.4, ZORUNLU).
"""

from abc import ABC, abstractmethod
import numpy as np

class ICameraSource(ABC):

    @abstractmethod
    async def start(self) -> None:
        """Kamera akışını başlatır."""
        pass
        
    @abstractmethod
    async def stop(self) -> None:
        """Kamera akışını durdurur."""
        pass
        
    @abstractmethod
    async def get_frame(self) -> np.ndarray:
        """BGR formatında tek kare döndürür."""
        pass
        
    @abstractmethod
    def get_resolution(self) -> tuple[int, int]:
        """Kamera çözünürlüğünü (genişlik, yükseklik) döndürür."""
        pass
