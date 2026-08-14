import logging
from core.interfaces.i_flight_backend import IFlightBackend
from core.interfaces.i_payload_actuator import IPayloadActuator
from core.navigation.centering_controller import CenteringController
from core.position_log.position_store import PositionStore
from core.config.parameters import GOREV3_DESCENT_ALTITUDE_M

logger = logging.getLogger(__name__)

class Gorev3RedropPhase:
    """Görev 3 Rapor Bölüm 7 (operatör revizyonu, 2026-08-13): "İkinci yük
    bırakma konumunda; Araç tekrar 30 cm irtifaya alçalacaktır. Son
    merkezleme tamamlandıktan sonra GRAB SERVO ... yük hedef konuma
    bırakılacaktır." No alignment/retreat sequence needed here (unlike
    pickup) -- reuses CenteringController.go_to_and_center's staged
    descend-and-center primitive, same one Görev 2's payload drops use
    (core/mission/payload_release.py), rather than the previous hand-rolled
    IPayloadVisibilityStrategy-based approach that gorev3_pickup.py now owns
    instead."""

    def __init__(self, flight: IFlightBackend, actuator: IPayloadActuator,
                 position_store: PositionStore, centering: CenteringController):
        self.flight = flight
        self.actuator = actuator
        self.position_store = position_store
        self.centering = centering

    async def run(self) -> bool:
        logger.info("Görev 3 Faz 3 (Yeniden Bırakma) Başlatıldı.")

        kirmizi_ucgen_point = self.position_store.get('KIRMIZI_UCGEN')
        if kirmizi_ucgen_point is None:
            raise RuntimeError("Kırmızı Üçgen konumu bulunamadı! Görev 3 Faz 3 başlatılamaz.")

        converged = await self.centering.go_to_and_center("KIRMIZI_UCGEN", altitude_m=GOREV3_DESCENT_ALTITUDE_M)
        if not converged:
            logger.warning("Son merkezleme yakınsamadı -- yine de bırakma denenecek (best-effort).")

        logger.info("Taşınan yük bırakma mekanizması aktifleşiyor...")
        await self.actuator.activate_drop_mechanism()

        logger.info("Görev 3 Faz 3 Başarılı.")
        return True
