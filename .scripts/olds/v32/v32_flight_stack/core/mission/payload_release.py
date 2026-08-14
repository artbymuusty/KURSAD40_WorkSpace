import logging
from core.interfaces.i_payload_actuator import IPayloadActuator
from core.interfaces.i_detector import IDetector
from core.interfaces.i_camera_source import ICameraSource
from core.interfaces.i_flight_backend import IFlightBackend
from core.navigation.centering_controller import CenteringController
from core.config.parameters import (
    VERIFICATION_MARKER, MISSION_ALTITUDE_M, PAYLOAD_APPROACH_ALTITUDES_M,
    PAYLOAD_FINAL_FORWARD_M,
)
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger(__name__)

class PayloadReleaseService:
    def __init__(self, actuator: IPayloadActuator, detector: IDetector, camera: ICameraSource,
                 centering: CenteringController, flight: IFlightBackend,
                 publisher: EventPublisher = NULL_PUBLISHER):
        self.actuator = actuator
        self.detector = detector
        self.camera = camera
        self.centering = centering
        self.flight = flight
        self.publisher = publisher

    def _publish(self, code, message="", severity=Severity.INFO, data=None):
        self.publisher.publish(Event(
            code=code, subsystem="PayloadReleaseService", category=Category.PAYLOAD,
            severity=severity, message=message, data=data or {},
        ))

    async def _staged_approach(self, shape_type: str) -> None:
        """Görev 2 Rapor (operatör revizyonu, 2026-08-13): '1. yük bırakma
        görevi' / '2. yük bırakma görevi' -- 15m'den (Gorev2Orchestrator
        zaten oradan çağırıyor) PAYLOAD_APPROACH_ALTITUDES_M listesindeki
        irtifalara sırayla in, her adımda yeniden ortala, sonra
        PAYLOAD_FINAL_FORWARD_M kadar ileri kay. Ara adımlardan biri
        yakınsayamazsa akışı durdurmak yerine devam edilir (best-effort --
        alçalmanın ortasında durup hiç bırakmamak, hafif kusurlu bir
        pozisyondan bırakmaktan daha kötüdür)."""
        for altitude_m in PAYLOAD_APPROACH_ALTITUDES_M:
            converged = await self.centering.go_to_and_center(shape_type, altitude_m=altitude_m)
            if not converged:
                logger.warning(f"Yaklaşma adımı yakınsamadı ({altitude_m}m) -- devam ediliyor.")
                self._publish("PAYLOAD_APPROACH_STEP_TIMED_OUT", shape_type, severity=Severity.WARN,
                              data={"shape_type": shape_type, "altitude_m": altitude_m})

        await self.centering.nudge_forward(PAYLOAD_FINAL_FORWARD_M)

    async def release_and_verify(self, shape_type: str) -> bool:
        """shape_type 'MAVI_ALTIGEN' ise actuator.release_payload_at_mavi_altigen() çağır;
        'KIRMIZI_UCGEN' ise actuator.release_payload_at_kirmizi_ucgen() çağır.
        Ardından VERIFICATION_MARKER[shape_type] değerine bakarak (parameters.py'den),
        kameradan bir kare al, detector ile tespit et, beklenen doğrulama işaretinin
        (Kırmızı/Mavi Dikdörtgen) görüntüde olup olmadığını kontrol et.
        Görev 2 Rapor Bölüm 13'e göre: 'Bu doğrulama yalnızca kontrol amaçlıdır, görev
        akışını durdurmaz' — yani doğrulama başarısız olsa bile fonksiyon False döner
        ama İSTİSNA FIRLATMAZ (akışı durdurmaz), yalnızca log'lar.

        Gorev2Orchestrator bu noktaya geldiğinde araç zaten MISSION_ALTITUDE_M'de
        hedefin üzerinde ortalanmış ve hover tamamlanmıştır -- bu metod
        oradan devralıp kademeli alçalma + SERVO + doğrulama + geri
        tırmanma dizisinin TAMAMINI yürütür (operatör revizyonu,
        2026-08-13)."""

        logger.info(f"Yük bırakma başlatılıyor: {shape_type}")
        self._publish("PAYLOAD_RELEASE_REQUESTED", shape_type, data={"shape_type": shape_type})

        if shape_type not in ("MAVI_ALTIGEN", "KIRMIZI_UCGEN"):
            logger.warning(f"Bilinmeyen hedef tipi: {shape_type}")
            self._publish("PAYLOAD_RELEASE_UNKNOWN_SHAPE", shape_type, severity=Severity.WARN,
                          data={"shape_type": shape_type})
            return False

        await self._staged_approach(shape_type)

        if shape_type == "MAVI_ALTIGEN":
            await self.actuator.release_payload_at_mavi_altigen()
        else:
            await self.actuator.release_payload_at_kirmizi_ucgen()

        self._publish("PAYLOAD_RELEASE_CONFIRMED", shape_type, data={"shape_type": shape_type})

        verified = await self._verify_marker(shape_type)

        # Operator revision (2026-08-13): the vehicle is now at
        # PAYLOAD_APPROACH_ALTITUDES_M's last (lowest) altitude, nudged
        # forward -- always climb back to mission altitude before returning,
        # regardless of verification outcome, so the caller (Gorev2Orchestrator)
        # can unconditionally resume search or engage the second target from
        # MISSION_ALTITUDE_M. Vision-independent (climb_to_altitude, not
        # go_to_and_center): the just-released shape may no longer be in
        # frame at this altitude/offset.
        await self.centering.climb_to_altitude(MISSION_ALTITUDE_M)

        return verified

    async def _verify_marker(self, shape_type: str) -> bool:
        """Görev 2 Rapor Bölüm 13: bırakılan yükün doğrulama işaretini
        (Kırmızı/Mavi Dikdörtgen) arar. Yalnızca bilgilendirme amaçlıdır --
        sonucu ne olursa olsun görev akışını durdurmaz."""
        expected_marker = VERIFICATION_MARKER.get(shape_type)
        if not expected_marker:
            logger.warning(f"Doğrulama işareti bulunamadı: {shape_type}")
            return False

        logger.info(f"Doğrulama işareti aranıyor: {expected_marker}")

        frame = await self.camera.get_frame()
        detections = await self.detector.detect(frame)

        found = any(d.shape_type == expected_marker for d in detections)
        # ADR-004 §7.2/§9.4: best-effort, informational only -- this event
        # must never gate mission flow, only be visible on the dashboard.
        self._publish("PAYLOAD_VERIFICATION_RESULT",
                      f"expected={expected_marker} found={found}",
                      severity=Severity.INFO if found else Severity.WARN,
                      data={"expected_marker": expected_marker, "found": found})

        if found:
            logger.info(f"Yük bırakma DOĞRULANDI: {expected_marker} tespit edildi.")
            return True

        logger.warning(f"Yük bırakma DOĞRULANAMADI: {expected_marker} görüntüde yok. (Görev akışı durdurulmaz)")
        return False
