import logging
from core.interfaces.i_flight_backend import IFlightBackend
from core.navigation.centering_controller import CenteringController
from core.mission.interlock import PayloadInterlock
from core.position_log.position_store import PositionStore
from core.mission.payload_release import PayloadReleaseService
from core.config.parameters import MISSION_ALTITUDE_M
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger(__name__)


class PayloadMissionSequencer:
    """Görev 2 Rapor (operatör revizyonu, 2026-08-13, "Mission Lifecycle"
    yeniden yapılandırması) -- eski Gorev2DurumMachine'in (DURUM-1..4,
    hangi hedefin ÖNCE tespit edildiğine göre dallanan mantık) yerini alır.

    Yeni kural: Mission (Search Phase) ASLA yük bırakma işlemi yapmaz --
    yalnızca iki hedefi de tespit edip PositionStore'a kaydeder. Hangi
    hedefin önce bulunduğu artık ÖNEMLİ DEĞİL: payload sırası HER ZAMAN
    sabittir -- önce Payload Mission 1 (Mavi Altıgen / RED payload), sonra
    Payload Mission 2 (Kırmızı Üçgen / BLUE payload) -- ve bu yalnızca
    Search Phase TAMAMEN bittikten (PositionStore.both_required_targets_found()
    True olduktan) SONRA, Offboard'ın tek yetkili olduğu evrede çalışır.

    PayloadInterlock (değişmedi) payload_2'nin payload_1'den önce asla
    bırakılamayacağını yazılım seviyesinde hâlâ garanti eder -- bu
    sequencer zaten sabit sırada çağırdığı için interlock'u ihlal etmesi
    mümkün değildir, ama guard yine de kaldırılmadı (savunma amaçlı)."""

    def __init__(self, flight: IFlightBackend, centering: CenteringController,
                 interlock: PayloadInterlock, position_store: PositionStore,
                 release_service: PayloadReleaseService, publisher: EventPublisher = NULL_PUBLISHER):
        self.flight = flight
        self.centering = centering
        self.interlock = interlock
        self.position_store = position_store
        self.release_service = release_service
        self.publisher = publisher

    def _publish(self, code, message="", severity=Severity.INFO, data=None):
        self.publisher.publish(Event(
            code=code, subsystem="PayloadMissionSequencer", category=Category.PAYLOAD,
            severity=severity, message=message, data=data or {},
        ))

    async def _navigate_to_recorded(self, shape_type: str) -> None:
        """Search sırasında kaydedilen GPS konumuna döner -- araç o an
        büyük olasılıkla İKİNCİ hedefin yakınındadır, kaydedilen konumda
        DEĞİL (bkz. CenteringController.goto_global_position_and_wait)."""
        tp = self.position_store.get(shape_type)
        if tp is None:
            raise RuntimeError(
                f"PayloadMissionSequencer: {shape_type} icin kayitli konum yok -- "
                "bu yalnizca both_required_targets_found() True iken cagrilmali."
            )
        converged = await self.centering.goto_global_position_and_wait(
            tp.gps_lat, tp.gps_lon, MISSION_ALTITUDE_M)
        if not converged:
            logger.warning(f"{shape_type} kayitli konumuna navigasyon zaman asimina ugradi -- "
                            "yine de devam ediliyor (best-effort).")

    async def execute_payload_mission_1(self) -> bool:
        """Payload Mission 1: Mavi Altıgen (RED payload) -- her zaman ilk."""
        logger.info("PAYLOAD MISSION 1 basliyor (Mavi Altigen / RED payload)")
        self._publish("PAYLOAD_MISSION_1_STARTED")
        await self._navigate_to_recorded("MAVI_ALTIGEN")
        result = await self.release_service.release_and_verify("MAVI_ALTIGEN")
        self.interlock.mark_payload_1_released()
        self.position_store.mark_payload_released("MAVI_ALTIGEN")
        self._publish("PAYLOAD_MISSION_1_COMPLETE", data={"verified": result})
        return result

    async def execute_payload_mission_2(self) -> bool:
        """Payload Mission 2: Kırmızı Üçgen (BLUE payload) -- her zaman
        ikinci. ÖNKOŞUL: interlock.mark_payload_1_released() zaten
        çağrılmış olmalı (PayloadInterlock bunu yazılım seviyesinde
        zorunlu kılar, aksi halde RuntimeError fırlatır)."""
        logger.info("PAYLOAD MISSION 2 basliyor (Kirmizi Ucgen / BLUE payload)")
        self._publish("PAYLOAD_MISSION_2_STARTED")
        await self._navigate_to_recorded("KIRMIZI_UCGEN")
        result = await self.release_service.release_and_verify("KIRMIZI_UCGEN")
        self.interlock.mark_payload_2_released()
        self.position_store.mark_payload_released("KIRMIZI_UCGEN")
        self._publish("PAYLOAD_MISSION_2_COMPLETE", data={"verified": result})
        return result

    async def execute_all(self) -> None:
        """İki payload görevini de sabit sırada çalıştırır. Yalnızca
        Search Phase TAMAMEN bittiğinde (both_required_targets_found())
        çağrılmalıdır -- çağıran taraf (Gorev2Orchestrator) bu invariant'ı
        garanti eder; burada da savunma amaçlı yeniden doğrulanır."""
        if not self.position_store.both_required_targets_found():
            raise RuntimeError(
                "PayloadMissionSequencer.execute_all() cagrildi ancak iki hedef de "
                "henuz kayitli degil -- Search Phase invariant'i ihlal edildi."
            )
        await self.execute_payload_mission_1()
        await self.execute_payload_mission_2()
