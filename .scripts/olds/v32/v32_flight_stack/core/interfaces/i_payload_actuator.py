"""
real_system implementasyonunda HER metod gövdesi yalnızca standart bir '# TODO[DONANIM]:' 
yorum bloğu ve simüle bir log+delay içerir (bkz. Prompt Book Bölüm 1.4); 
gz_system implementasyonunda gerçek bir Gazebo servis/topik çağrısı vardır, TODO YOKTUR.
"""

from abc import ABC, abstractmethod

class IPayloadActuator(ABC):
    
    @abstractmethod
    async def release_payload_at_mavi_altigen(self) -> bool:
        """Görev 2, 1. yük bırakma."""
        pass
        
    @abstractmethod
    async def release_payload_at_kirmizi_ucgen(self) -> bool:
        """Görev 2, 2. yük bırakma."""
        pass
        
    @abstractmethod
    async def activate_pickup_mechanism(self) -> bool:
        """Görev 3 Faz 1: Yük alma mekanizmasını aktifleştirir."""
        pass
        
    @abstractmethod
    async def activate_drop_mechanism(self) -> bool:
        """Görev 3 Faz 3: Taşıma görevindeki yükü bırakır."""
        pass
