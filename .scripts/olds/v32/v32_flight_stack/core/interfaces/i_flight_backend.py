"""
Bu arayüzün hem real_system hem gz_system implementasyonu, farklı bağlantı adresi dışında AYNI MAVSDK kod tabanını paylaşabilir;
ayrı sınıflara bölünmesinin nedeni yalnızca gelecekte farklılaşma ihtimaline karşı esneklik sağlamaktır (bkz. Prompt Book Bölüm 1.2).
"""

from abc import ABC, abstractmethod

class IFlightBackend(ABC):
    
    @abstractmethod
    async def connect(self) -> None:
        """Sisteme (Gerçek veya SITL) bağlanır."""
        pass
        
    @abstractmethod
    async def arm(self) -> None:
        """Aracı arm eder (Görev 2 Rapor Bölüm 16)."""
        pass
        
    @abstractmethod
    async def takeoff(self, target_altitude_m: float) -> None:
        """Belirtilen irtifaya kalkış yapar (Görev 2 Rapor Bölüm 16)."""
        pass
        
    @abstractmethod
    async def land(self) -> None:
        """Aracı bulunduğu konuma indirir."""
        pass
        
    @abstractmethod
    async def start_offboard(self) -> None:
        """Offboard kontrol modunu başlatır (Görev 2 Rapor Bölüm 8)."""
        pass
        
    @abstractmethod
    async def stop_offboard(self) -> None:
        """Offboard kontrol modunu durdurur."""
        pass
        
    @abstractmethod
    async def goto_position_ned(self, north_m: float, east_m: float, down_m: float, yaw_deg: float) -> None:
        """Offboard modunda NED koordinatlarına gider."""
        pass

    @abstractmethod
    async def goto_position_ned_and_hold(self, north_m: float, east_m: float, down_m: float,
                                          yaw_deg: float, duration_s: float) -> None:
        """goto_position_ned ile AYNI setpoint'i duration_s boyunca tekrar
        tekrar gönderir (Görev 3 Rapor, operatör revizyonu 2026-08-13).
        PX4 ~500ms setpoint'siz kalırsa Offboard'dan çıkar (bkz.
        hold_position/go_to_and_center'daki aynı BUG FIX) -- goto_position_ned'i
        tek seferlik çağırıp ardından asyncio.sleep() yapmak (Görev 3
        fazlarının önceki, tamamen simüle edilmiş haliydi) bu yüzden gerçek
        uçuşta Offboard'dan düşerdi. Çok adımlı Görev 3 hareketleri
        (yaklaşma, geri çekilme, transit) artık bunu kullanır."""
        pass
        
    @abstractmethod
    async def set_velocity_body(self, forward_m_s: float, right_m_s: float, down_m_s: float, yaw_rate_deg_s: float) -> None:
        """Offboard modunda gövde eksenli hızları ayarlar."""
        pass
        
    @abstractmethod
    async def hold_position(self, duration_s: float) -> None:
        """Belirtilen süre boyunca konumu korur (Görev 2 Rapor Bölüm 9)."""
        pass
        
    @abstractmethod
    async def get_position_ned(self) -> tuple[float, float, float]:
        """Güncel NED konumunu (Kuzey, Doğu, Aşağı) döndürür."""
        pass

    @abstractmethod
    async def get_velocity_ned(self) -> tuple[float, float, float]:
        """Güncel NED hızını (Kuzey, Doğu, Aşağı, m/s) döndürür.
        goto_global_position_and_wait()'in yakınsama koşulunun konum YANI
        SIRA hız büyüklüğünü de gerektirmesi için eklendi (bkz. o metodun
        kendi BUG FIX yorumu) -- araç hedefin 2m yarıçapından ~11 m/s hızla
        geçerken "yakınsadı" denmesini önler."""
        pass

    @abstractmethod
    async def get_global_position(self) -> tuple[float, float, float]:
        """Güncel GPS konumunu (Enlem, Boylam, İrtifa) döndürür (Görev 2 Rapor Bölüm 4.2 checkpoint)."""
        pass
        
    @abstractmethod
    async def get_yaw_deg(self) -> float:
        """Güncel Yaw açısını döndürür."""
        pass

    @abstractmethod
    async def get_flight_mode(self) -> str:
        """Güncel uçuş modunu döndürür (örn. 'MISSION', 'OFFBOARD', 'HOLD').
        switch_to_offboard()'un PX4 tarafından GERÇEKTEN kabul edilip
        edilmediğini doğrulamak için kullanılır -- start_offboard()'un
        istisna fırlatmaması tek başına yeterli kanıt değildir."""
        pass
        
    @abstractmethod
    async def upload_mission(self, waypoints: list) -> None:
        """Rota noktalarını araca yükler."""
        pass

    @abstractmethod
    async def confirm_existing_mission(self) -> int:
        """Operatörün QGroundControl üzerinden UÇUŞ ÖNCESİ zaten yüklediği
        mission'ı doğrular ve kalem (item) sayısını döner -- bu sistem
        KENDİ arama rotasını üretip yüklemez; rota tanımı operatörün
        sorumluluğundadır (bkz. Görev 2 Rapor: 'QGroundControl: Operatörün
        görev öncesi waypoint/tarama rotası tanımlaması'). 0 dönerse
        çağıran taraf mission'ı BAŞLATMAMALIDIR -- operatör henüz bir
        rota yüklememiş demektir."""
        pass

    @abstractmethod
    async def start_mission(self) -> None:
        """Mission modunu başlatır."""
        pass
        
    @abstractmethod
    async def is_mission_finished(self) -> bool:
        """Mission modunun tamamlanıp tamamlanmadığını kontrol eder."""
        pass
        
    @abstractmethod
    async def switch_to_offboard_from_mission(self) -> None:
        """Görev 2 Bölüm 8: Mission durur, Offboard başlar."""
        pass
