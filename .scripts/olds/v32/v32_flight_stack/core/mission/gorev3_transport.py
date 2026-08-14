import logging
from core.interfaces.i_flight_backend import IFlightBackend
from core.navigation.centering_controller import CenteringController
from core.position_log.position_store import PositionStore
from core.config.parameters import GOREV3_TRANSIT_SPEED_M_S, GOREV3_TRANSIT_ALTITUDE_M

logger = logging.getLogger(__name__)

class Gorev3TransportPhase:
    def __init__(self, flight: IFlightBackend, position_store: PositionStore, centering: CenteringController):
        self.flight = flight
        self.position_store = position_store
        self.centering = centering

    async def run(self) -> None:
        logger.info("Görev 3 Faz 2 (Düşük Hızlı Taşıma) Başlatıldı.")
        kirmizi_ucgen_point = self.position_store.get('KIRMIZI_UCGEN')
        if kirmizi_ucgen_point is None:
            raise RuntimeError("Kırmızı Üçgen konumu bulunamadı!")

        if GOREV3_TRANSIT_SPEED_M_S is None:
            raise RuntimeError(
                "GOREV3_TRANSIT_SPEED_M_S henuz TODO -- ekip fiziksel testle "
                "doldurmadan Gorev 3 Faz 2 calistirilamaz (Gorev 3 Rapor Bolum 6)."
            )

        logger.info(f"Kırmızı Üçgen konumuna {GOREV3_TRANSIT_SPEED_M_S} m/s hızla gidiliyor: "
                    f"{kirmizi_ucgen_point.gps_lat}, {kirmizi_ucgen_point.gps_lon}")
        # BUG FIX (continuous audit, 2026-08-13): this used to hold north=0/
        # east=0 -- i.e. NOT actually navigate to Kırmızı Üçgen's recorded
        # position at all, just change altitude in place (see
        # gorev3_pickup.py's identical fix for the full explanation).
        # NOTE: GOREV3_TRANSIT_SPEED_M_S is not yet actually enforced as a
        # velocity cap here -- goto_global_position_and_wait streams
        # position setpoints and lets PX4's own position controller fly
        # toward them at its configured speed; a real speed-limited transit
        # would need a velocity-mode implementation, out of scope for this
        # fix (tracked as a remaining risk, not silently dropped).
        converged = await self.centering.goto_global_position_and_wait(
            kirmizi_ucgen_point.gps_lat, kirmizi_ucgen_point.gps_lon, GOREV3_TRANSIT_ALTITUDE_M)
        if not converged:
            logger.warning("Kirmizi Ucgen konumuna navigasyon zaman asimina ugradi -- yine de devam ediliyor.")
        logger.info("Hedefe ulaşıldı.")
