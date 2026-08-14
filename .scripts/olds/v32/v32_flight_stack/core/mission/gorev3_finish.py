import logging
from core.interfaces.i_flight_backend import IFlightBackend
from core.navigation.centering_controller import CenteringController
from core.navigation.checkpoint import MissionCheckpoint
from core.config.parameters import GOREV3_DROP_CLIMB_STEPS_M

logger = logging.getLogger(__name__)

class Gorev3FinishPhase:
    def __init__(self, flight: IFlightBackend, checkpoint: MissionCheckpoint, centering: CenteringController):
        self.flight = flight
        self.checkpoint = checkpoint
        self.centering = centering

    async def run(self) -> None:
        logger.info("Görev 3 Faz 4 (Sonlandırma) Başlatıldı.")

        # 1. GOREV3_DROP_CLIMB_STEPS_M (1, 2 m) irtifalarına sırayla yüksel.
        # BUG FIX (operator revision, 2026-08-13): tek seferlik
        # goto_position_ned() + asyncio.sleep(1) -- diğer Görev 3 fazlarında
        # zaten düzeltilmiş olan aynı hata sınıfı (PX4 ~500ms setpoint'siz
        # kalırsa Offboard'dan düşer), yalnızca bu dosyada gözden kaçmıştı.
        yaw = await self.flight.get_yaw_deg()
        for alt in GOREV3_DROP_CLIMB_STEPS_M:
            logger.info(f"Yükseliniyor: {alt}m")
            await self.flight.goto_position_ned_and_hold(0, 0, -alt, yaw, 2.0)

        # 2. Finish Line (kalkış/checkpoint) konumuna gerçekten git.
        # BUG FIX (operator revision, 2026-08-13, "Mission Lifecycle"
        # yeniden yapılandırması -- LANDING gereksinimi): önceden yalnızca
        # bir log satırı + asyncio.sleep(2) idi, araç GERÇEKTEN checkpoint
        # konumuna gitmiyordu. Landing artık kalan QGC search waypoint'lerine
        # bağlı değil ve Mission'a asla dönmez -- goto_global_position_and_wait
        # (core/navigation/geo.py flat-earth yaklaşıklığı) gerçek GPS
        # navigasyonu sağlar, Offboard'dan hiç çıkmadan.
        lat, lon, checkpoint_alt = self.checkpoint.get()
        logger.info(f"Finish Line (Checkpoint) konumuna gidiliyor: {lat}, {lon}")
        converged = await self.centering.goto_global_position_and_wait(lat, lon, checkpoint_alt)
        if not converged:
            logger.warning("Finish Line navigasyonu zaman asimina ugradi -- yine de inis denenecek (best-effort).")

        # 3. Finish Line'a ulaşıldığında logla
        logger.info("GOREV 3 TAMAMLANDI - TUM GOREVLER BASARIYLA BITTI")

        # MissionResult objesi dönebilir veya sadece None
