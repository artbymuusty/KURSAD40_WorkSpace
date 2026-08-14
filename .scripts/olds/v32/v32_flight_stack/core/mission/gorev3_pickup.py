import asyncio
import logging
from core.interfaces.i_flight_backend import IFlightBackend
from core.interfaces.i_camera_source import ICameraSource
from core.interfaces.i_detector import IDetector
from core.interfaces.i_payload_actuator import IPayloadActuator
from core.interfaces.i_payload_visibility_strategy import IPayloadVisibilityStrategy
from core.navigation.centering_controller import CenteringController
from core.position_log.position_store import PositionStore
from core.config.parameters import (
    GOREV3_TRANSIT_ALTITUDE_M,
    GOREV3_DESCENT_ALTITUDE_M,
    GOREV3_RETREAT_DISTANCE_M,
    GOREV3_ADVANCE_DISTANCE_M,
    GOREV3_PICKUP_VERIFY_CLIMB_STEPS_M,
    GOREV3_PICKUP_ALIGN_MAX_ATTEMPTS,
    GOREV3_PICKUP_VISIBILITY_CONFIRM_FRAMES,
    OFFBOARD_SETPOINT_INTERVAL_S,
)

logger = logging.getLogger(__name__)

class Gorev3PickupPhase:
    """Görev 3 Rapor Bölüm 5 (operatör revizyonu, 2026-08-13): Mavi Altıgen
    konumuna (1. yükün bırakıldığı yer) dönülür, orada artık görünen Kırmızı
    Dikdörtgen'e (fiziksel 1. yük) uzun kenarına dik olacak şekilde
    hizalanılır, 30cm geriden görüntüyle doğrulanır, 60cm ileri gidilerek
    alma pozisyonuna geçilir, THIRD MISSION SERVO ile alınır, ve
    GOREV3_PICKUP_VERIFY_CLIMB_STEPS_M irtifalarına yükselerek Kırmızı
    Dikdörtgen'in artık görünmediği doğrulanır."""

    def __init__(self, flight: IFlightBackend, camera: ICameraSource, detector: IDetector,
                 actuator: IPayloadActuator, position_store: PositionStore,
                 visibility_strategy: IPayloadVisibilityStrategy, centering: CenteringController):
        self.flight = flight
        self.camera = camera
        self.detector = detector
        self.actuator = actuator
        self.position_store = position_store
        self.visibility_strategy = visibility_strategy
        self.centering = centering

    async def _locate_target_with_retries(self):
        """Kırmızı Dikdörtgen bulunana kadar (veya deneme sınırına kadar)
        her karede yeniden dener -- go_to_and_center()'ın 'hedef kayboldu'
        döngüsüyle aynı mantık."""
        for _ in range(GOREV3_PICKUP_ALIGN_MAX_ATTEMPTS):
            frame = await self.camera.get_frame()
            try:
                return await self.visibility_strategy.locate_target(self.detector, frame)
            except RuntimeError:
                await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)
        return None

    async def run(self) -> bool:
        logger.info("Görev 3 Faz 1 (Alma) Başlatıldı.")

        mavi_altigen_point = self.position_store.get('MAVI_ALTIGEN')
        if mavi_altigen_point is None:
            raise RuntimeError("Mavi Altıgen konumu bulunamadı! Görev 3 başlatılamaz.")

        logger.info(f"Mavi Altıgen konumuna {GOREV3_TRANSIT_ALTITUDE_M}m irtifada gidiliyor: "
                    f"{mavi_altigen_point.gps_lat}, {mavi_altigen_point.gps_lon}")
        # BUG FIX (continuous audit, 2026-08-13): this used to hold north=0/
        # east=0 -- i.e. NOT actually navigate anywhere, just change
        # altitude in place. That was an accepted simplification before
        # CenteringController.goto_global_position_and_wait() existed (see
        # its own docstring); left unfixed after that, it meant Görev 3
        # Faz 1 started searching for Kırmızı Dikdörtgen wherever Payload
        # Mission 2 happened to leave the vehicle (Kırmızı Üçgen's
        # position), never at Mavi Altıgen where the payload actually is.
        converged = await self.centering.goto_global_position_and_wait(
            mavi_altigen_point.gps_lat, mavi_altigen_point.gps_lon, GOREV3_TRANSIT_ALTITUDE_M)
        if not converged:
            logger.warning("Mavi Altigen konumuna navigasyon zaman asimina ugradi -- yine de devam ediliyor.")

        target = await self._locate_target_with_retries()
        if target is None:
            logger.error("Kırmızı Dikdörtgen bulunamadı -- Görev 3 Faz 1 başarısız.")
            return False

        alignment_delta_deg = await self.visibility_strategy.compute_alignment_yaw(target, None)
        current_yaw = await self.flight.get_yaw_deg()
        aligned_yaw = current_yaw + alignment_delta_deg
        logger.info(f"Kırmızı Dikdörtgenin uzun kenarına dik hizalanılıyor: "
                    f"{current_yaw:.1f} -> {aligned_yaw:.1f} derece")
        await self.flight.goto_position_ned_and_hold(0, 0, -GOREV3_TRANSIT_ALTITUDE_M, aligned_yaw, 2.0)

        logger.info(f"{GOREV3_RETREAT_DISTANCE_M}m geride, {GOREV3_DESCENT_ALTITUDE_M}m irtifaya alçalınıyor...")
        await self.flight.goto_position_ned_and_hold(
            0, -GOREV3_RETREAT_DISTANCE_M, -GOREV3_DESCENT_ALTITUDE_M, aligned_yaw, 3.0)

        # Bu 30cm'lik pozisyonda hedefin hâlâ görüntüde olduğunu N kare
        # boyunca doğrula (Görev 3 Rapor: "bu 30 cm'de görüntü işleme ile
        # aktif görmek istiyorum") -- best-effort görünürlük onayı, tam
        # yeniden ortalama değil (araç zaten hizalı ve konumlanmış).
        visible_frames = 0
        for _ in range(GOREV3_PICKUP_VISIBILITY_CONFIRM_FRAMES * 3):
            frame = await self.camera.get_frame()
            try:
                await self.visibility_strategy.locate_target(self.detector, frame)
                visible_frames += 1
                if visible_frames >= GOREV3_PICKUP_VISIBILITY_CONFIRM_FRAMES:
                    break
            except RuntimeError:
                visible_frames = 0
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)
        if visible_frames < GOREV3_PICKUP_VISIBILITY_CONFIRM_FRAMES:
            logger.warning("30cm pozisyonunda hedef görünürlüğü doğrulanamadı -- devam ediliyor (best-effort).")

        logger.info(f"{GOREV3_ADVANCE_DISTANCE_M}m ileri, alma pozisyonuna gidiliyor...")
        await self.flight.goto_position_ned_and_hold(
            GOREV3_ADVANCE_DISTANCE_M, -GOREV3_RETREAT_DISTANCE_M, -GOREV3_DESCENT_ALTITUDE_M, aligned_yaw, 3.0)

        logger.info("Yük alma mekanizması aktifleşiyor...")
        # THIRD MISSION SERVO
        await self.actuator.activate_pickup_mechanism()

        for alt in GOREV3_PICKUP_VERIFY_CLIMB_STEPS_M:
            logger.info(f"Yükseliniyor: {alt}m")
            await self.flight.goto_position_ned_and_hold(0, 0, -alt, aligned_yaw, 2.0)
            frame = await self.camera.get_frame()
            detections = await self.detector.detect(frame)
            still_visible = any(d.shape_type == "KIRMIZI_DIKDORTGEN" for d in detections)
            if still_visible:
                logger.warning(f"{alt}m irtifada Kırmızı Dikdörtgen hâlâ görüntüde.")

        logger.info("Doğrulama kontrolü yapılıyor (son irtifa)...")
        frame = await self.camera.get_frame()
        detections = await self.detector.detect(frame)

        for d in detections:
            if d.shape_type == "KIRMIZI_DIKDORTGEN":
                logger.warning("Kırmızı Dikdörtgen hala görüntüde! Alma başarısız.")
                return False

        logger.info("Kırmızı Dikdörtgen görüntüde yok. Yük Alma Başarılı.")
        return True
