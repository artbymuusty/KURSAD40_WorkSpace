import asyncio
import logging
import time
from core.interfaces.i_payload_actuator import IPayloadActuator
from core.interfaces.i_camera_source import ICameraSource
from core.interfaces.i_flight_backend import IFlightBackend
from core.detection.camera_intrinsics import default_camera_intrinsics
from core.detection.detection_feed import DetectionFeed
from core.navigation.centering_controller import CenteringController
from core.config.parameters import (
    VERIFICATION_MARKER, MISSION_ALTITUDE_M, PAYLOAD_APPROACH_ALTITUDES_M,
    PAYLOAD_FINAL_FORWARD_M, PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M,
    LOW_ALT_VISION_LIMIT_M, PAYLOAD_FINAL_POSE_DELAY_S,
    PAYLOAD_DETACH_HOLD_MAX_S, PAYLOAD_ON_TARGET_RADIUS_M,
    PAYLOAD_ON_TARGET_Z_TOLERANCE_M, PAYLOAD_MOUNT_OFFSET_BODY_M,
)
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger(__name__)

class PayloadReleaseService:
    def __init__(self, actuator: IPayloadActuator, detection_feed: DetectionFeed, camera: ICameraSource,
                 centering: CenteringController, flight: IFlightBackend,
                 publisher: EventPublisher = NULL_PUBLISHER):
        self.actuator = actuator
        # ADR-008 B1: was `detector: IDetector`. _verify_marker() was the
        # second of the two competing detect() callers the single-loop
        # invariant removes (see core/detection/detection_feed.py). Only the
        # SOURCE of the detections changed here -- what is checked, when, and
        # the "informational only, never gates mission flow" rule of Görev 2
        # Rapor Bölüm 13 are untouched.
        self.detection_feed = detection_feed
        self.camera = camera
        self.centering = centering
        self.flight = flight
        self.publisher = publisher
        # ADR-010 P2: PAYLOAD panel state. `_payload_index` counts drops in
        # the order they are commanded (1 = first target engaged), which is
        # what the operator sees on screen; `_last_offset_cm` is the most
        # recent COMMITTED offset, kept because at the release altitude the
        # target is usually no longer detectable (see ADR-010 P1) and the
        # last committed value is then the best knowable answer.
        self._payload_index = 0
        self._last_offset_cm = None
        self._last_offset_alt_m = None

    def _publish(self, code, message="", severity=Severity.INFO, data=None):
        self.publisher.publish(Event(
            code=code, subsystem="PayloadReleaseService", category=Category.PAYLOAD,
            severity=severity, message=message, data=data or {},
        ))

    def _publish_state(self, shape_type: str, **fields) -> None:
        """ADR-010 P2: one event carrying everything the PAYLOAD panel
        shows, published at every step of the drop so the operator can see
        WHY a release happened where it did rather than only that it did.
        Read-only telemetry -- nothing here gates the release."""
        target = self.detection_feed.get(shape_type)
        data = {"shape_type": shape_type, "payload_index": self._payload_index,
                "vision_committed": target is not None,
                "last_offset_cm": self._last_offset_cm,
                "low_alt_vision_limit_m": LOW_ALT_VISION_LIMIT_M}
        data.update(fields)
        self._publish("PAYLOAD_STATE", shape_type, data=data)

    async def _staged_approach(self, shape_type: str) -> None:
        """Görev 2 Rapor (operatör revizyonu, 2026-08-13): '1. yük bırakma
        görevi' / '2. yük bırakma görevi' -- 15m'den (Gorev2Orchestrator
        zaten oradan çağırıyor) PAYLOAD_APPROACH_ALTITUDES_M listesindeki
        irtifalara sırayla in, her adımda yeniden ortala, sonra
        PAYLOAD_FINAL_FORWARD_M kadar ileri kay. Ara adımlardan biri
        yakınsayamazsa akışı durdurmak yerine devam edilir (best-effort --
        alçalmanın ortasında durup hiç bırakmamak, hafif kusurlu bir
        pozisyondan bırakmaktan daha kötüdür)."""
        final_step = len(PAYLOAD_APPROACH_ALTITUDES_M)
        for step_index, altitude_m in enumerate(PAYLOAD_APPROACH_ALTITUDES_M, start=1):
            await self._track_offset(shape_type)
            self._publish_state(shape_type, descent_step=f"{step_index}/{final_step}",
                                target_alt_m=altitude_m,
                                current_alt_m=await self._altitude_m(), released=False)
            # A1 + A2: only the FINAL step gets the release-altitude band and
            # the payload-centred aim. The higher steps keep the loose 0.30 m
            # band and camera-centred aim, because their job is only to get
            # close enough to keep the target in frame -- tightening them
            # would spend time converging at 10 m and 5 m for no benefit.
            is_final = step_index == final_step
            converged = await self.centering.go_to_and_center(
                shape_type, altitude_m=altitude_m,
                alt_tolerance_m=PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M if is_final else None)
            if is_final:
                # D3: the vision loop above kept the target CENTRED, so its
                # last committed fix is the cleanest one available. Only now
                # is the mount offset applied -- as a translation of the hold
                # point -- and the descent finished open-loop on it. Doing it
                # the other way round (biasing the vision error) corrupted the
                # measurement and bought nothing.
                mount = PAYLOAD_MOUNT_OFFSET_BODY_M.get(shape_type)
                if mount:
                    reached = await self.centering.descend_to_release(
                        shape_type, altitude_m, mount)
                    converged = abs(reached - altitude_m) <= PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M
            await self._track_offset(shape_type)
            if not converged:
                logger.warning(f"Yaklaşma adımı yakınsamadı ({altitude_m}m) -- devam ediliyor.")
                self._publish("PAYLOAD_APPROACH_STEP_TIMED_OUT", shape_type, severity=Severity.WARN,
                              data={"shape_type": shape_type, "altitude_m": altitude_m})

        await self.centering.nudge_forward(PAYLOAD_FINAL_FORWARD_M)

    async def _altitude_m(self):
        try:
            _, _, alt_m = await self.flight.get_global_position()
            return round(alt_m, 3)
        except Exception:  # noqa: BLE001 -- a panel value must never break a drop
            return None

    def _aim_offset_px(self, shape_type: str, alt_m: float, focal_px: float):
        """A2: where the payload sits in frame, in pixels.

        Every offset this service reports is the PAYLOAD-to-target
        distance, not the camera-to-target distance. Without this the
        numbers would read ~28 cm at a perfectly aimed release and 0 cm at
        one that is about to miss by the full mount offset -- exactly
        backwards."""
        mount = PAYLOAD_MOUNT_OFFSET_BODY_M.get(shape_type)
        if not mount or not alt_m or not focal_px:
            return (0.0, 0.0)
        px_per_m = focal_px / alt_m
        return (mount[1] * px_per_m, -mount[0] * px_per_m)

    async def _track_offset(self, shape_type: str) -> None:
        """Remember the newest COMMITTED offset. ADR-010 P1 established that
        the target is generally not detectable at the 0.45 m release
        altitude (the hexagon stops committing at ~1.6 m), so the release
        offset can only ever be reported as "last known, measured at
        altitude X" -- and that is only honest if the altitude is carried
        alongside the number."""
        target = self.detection_feed.get(shape_type)
        if target is None:
            return
        res_w, res_h = self.camera.get_resolution()
        intrinsics = default_camera_intrinsics()
        if intrinsics is None:
            return
        focal = intrinsics.scaled_to(res_w, res_h).focal_px
        alt_m = await self._altitude_m()
        if not focal or not alt_m:
            return
        aim_dx, aim_dy = self._aim_offset_px(shape_type, alt_m, focal)
        dx = target.center_px[0] - res_w / 2.0 - aim_dx
        dy = target.center_px[1] - res_h / 2.0 - aim_dy
        px_to_cm = alt_m * 100.0 / focal
        self._last_offset_cm = round((dx * dx + dy * dy) ** 0.5 * px_to_cm, 2)
        self._last_offset_alt_m = round(alt_m, 3)

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
        self._payload_index += 1
        self._publish("PAYLOAD_RELEASE_REQUESTED", shape_type, data={"shape_type": shape_type})

        if shape_type not in ("MAVI_ALTIGEN", "KIRMIZI_UCGEN"):
            logger.warning(f"Bilinmeyen hedef tipi: {shape_type}")
            self._publish("PAYLOAD_RELEASE_UNKNOWN_SHAPE", shape_type, severity=Severity.WARN,
                          data={"shape_type": shape_type})
            return False

        await self._staged_approach(shape_type)

        await self._publish_release_offset(shape_type)

        # ADR-010 P1: the release altitude is now a REPORTED, checked
        # quantity, not an assumed one. V1''' released payload 1 at 1.587 m
        # and payload 2 at 0.407 m from the same commanded 0.45 m, and
        # nothing in the log said so at the time -- the altitude only came
        # out of post-run analysis. A drop outside the band is still
        # performed (stopping mid-descent with an undropped payload is
        # worse), but it is now loud.
        release_alt_m = await self._altitude_m()
        target_alt_m = PAYLOAD_APPROACH_ALTITUDES_M[-1]
        within = (release_alt_m is not None
                  and abs(release_alt_m - target_alt_m) <= PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M)
        if not within:
            logger.warning("[RELEASE_ALTITUDE] %s: %s m -- hedef %.2f +/- %.2f m BANDI DISINDA.",
                           shape_type, release_alt_m, target_alt_m, PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M)
        self._publish("PAYLOAD_RELEASE_ALTITUDE", shape_type,
                      severity=Severity.INFO if within else Severity.WARN,
                      data={"shape_type": shape_type, "release_alt_m": release_alt_m,
                            "target_alt_m": target_alt_m,
                            "tolerance_m": PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M,
                            "within_tolerance": within})

        # A2: the mount vector as MEASURED at the servo instant, alongside
        # the one the aim correction was computed from. If they disagree,
        # the release offset is explained rather than mysterious.
        measured_mount = getattr(self.actuator, "measure_mount_vector", lambda _s: None)(shape_type)
        if measured_mount is not None:
            logger.info("[MOUNT_VECTOR_MEASURED] %s: %s (uygulanan: %s)",
                        shape_type, measured_mount,
                        PAYLOAD_MOUNT_OFFSET_BODY_M.get(shape_type))
            self._publish("MOUNT_VECTOR_MEASURED", shape_type,
                          data={"shape_type": shape_type, "measured": measured_mount,
                                "applied_body_m": list(
                                    PAYLOAD_MOUNT_OFFSET_BODY_M.get(shape_type) or ())})

        servo_at = time.monotonic()
        if shape_type == "MAVI_ALTIGEN":
            confirmed = await self.actuator.release_payload_at_mavi_altigen()
        else:
            confirmed = await self.actuator.release_payload_at_kirmizi_ucgen()

        # F2: the actuator now answers "did the body actually leave", not
        # just "was a message sent". An unconfirmed release must not be
        # followed by a climb-out -- that is exactly how payload 2 ended up
        # 4.9 m past the triangle on the first ADR-011 flight.
        if not confirmed:
            confirmed = await self._hold_until_detached(shape_type)

        latency_s = getattr(self.actuator, "detach_latency", lambda _s: None)(shape_type)
        if latency_s is None:
            latency_s = round(time.monotonic() - servo_at, 3)
        self._publish("PAYLOAD_RELEASE_CONFIRMED", shape_type,
                      severity=Severity.INFO if confirmed else Severity.CRITICAL,
                      data={"shape_type": shape_type, "separation_confirmed": confirmed,
                            "detach_latency_s": latency_s})

        # W4: one operator-facing announcement, on every channel at once, so
        # "did it actually drop?" is answerable from the dashboard, from QGC
        # and from the log without cross-referencing three different codes.
        released_at = time.time()
        banner = f"PAYLOAD {self._payload_index} RELEASED @{release_alt_m:.2f}m" \
            if release_alt_m is not None else f"PAYLOAD {self._payload_index} RELEASED"
        logger.info("[%s] %s", "PAYLOAD_RELEASED", banner)
        self._publish("PAYLOAD_RELEASED", banner,
                      data={"shape_type": shape_type, "payload_index": self._payload_index,
                            "released_alt_m": release_alt_m, "target_alt_m": target_alt_m,
                            "within_tolerance": within, "released_at": released_at,
                            "status_text": banner})
        # Best-effort: never let a cosmetic GCS message affect the drop.
        try:
            await self.flight.send_status_text(banner, "INFO")
        except Exception:  # noqa: BLE001
            pass

        self._publish_state(shape_type, released=True, released_alt_m=release_alt_m,
                            target_alt_m=target_alt_m, within_tolerance=within,
                            current_alt_m=release_alt_m, descent_step="RELEASED",
                            released_at=released_at)

        # W4.3: where did it actually end up? Logged 2 s after the servo so
        # the body has had time to fall and settle. Purely observational --
        # the answer is currently expected to be "fell through the world"
        # until W2 lands, and recording it is how that gets verified.
        await self._log_payload_final_pose(shape_type)

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

    async def _hold_until_detached(self, shape_type: str) -> bool:
        """F2: the payload did not separate. Hold at release altitude and
        keep trying, rather than carrying an attached payload away.

        Holding is the safe branch here: the vehicle is already at 0.45 m
        over the target, so an operator can land it at any moment. What is
        NOT safe is climbing to 15 m with a payload that is about to let go
        somewhere over the field. The hold is bounded by
        PAYLOAD_DETACH_HOLD_MAX_S so an unrecoverable joint cannot hover the
        battery flat; when the bound is hit the mission continues, but the
        failure stays on the record at CRITICAL."""
        checker = getattr(self.actuator, "is_release_confirmed", None)
        if checker is None:
            return False
        logger.critical("[PAYLOAD_DETACH_UNCONFIRMED] %s: yuk ayrilmadi -- "
                        "TIRMANIS YOK, pozisyon korunuyor.", shape_type)
        self._publish("PAYLOAD_DETACH_UNCONFIRMED", f"{shape_type}: yuk ayrilmadi",
                      severity=Severity.CRITICAL,
                      data={"shape_type": shape_type,
                            "hold_max_s": PAYLOAD_DETACH_HOLD_MAX_S})
        try:
            await self.flight.send_status_text(
                f"PAYLOAD {self._payload_index} DETACH UNCONFIRMED - HOLDING", "CRITICAL")
        except Exception:  # noqa: BLE001
            pass

        retry = getattr(self.actuator, "retry_release", None)
        started = time.monotonic()
        while time.monotonic() - started < PAYLOAD_DETACH_HOLD_MAX_S:
            # hold_position keeps the offboard setpoint stream alive; a
            # silent sleep here would drop PX4 out of Offboard mid-hold.
            await self.flight.hold_position(1.0)
            if await checker(shape_type):
                held_s = round(time.monotonic() - started, 2)
                logger.info("Yuk %s ayrildi (%.2f s tutma sonrasi).", shape_type, held_s)
                self._publish("PAYLOAD_DETACH_RECOVERED", shape_type,
                              severity=Severity.WARN,
                              data={"shape_type": shape_type, "held_s": held_s})
                return True
            if retry is not None:
                await retry(shape_type)
        logger.critical("[PAYLOAD_DETACH_UNCONFIRMED] %s: %.0f s sonunda hala "
                        "ayrilmadi -- goreve devam ediliyor.",
                        shape_type, PAYLOAD_DETACH_HOLD_MAX_S)
        self._publish("PAYLOAD_DETACH_GAVE_UP", shape_type, severity=Severity.CRITICAL,
                      data={"shape_type": shape_type, "held_s": PAYLOAD_DETACH_HOLD_MAX_S})
        return False

    async def _log_payload_final_pose(self, shape_type: str) -> None:
        """W4.3: ask the actuator where the released body ended up.

        Only the actuator knows whether the release even HAS a body to
        locate -- the Gazebo one spawns a model it can query, the real one
        drops a physical object it cannot. Backends that cannot answer
        return None and this reports 'unavailable' rather than inventing a
        pose."""
        locator = getattr(self.actuator, "get_released_payload_pose", None)
        if locator is None:
            return
        await asyncio.sleep(PAYLOAD_FINAL_POSE_DELAY_S)
        try:
            pose = await locator(shape_type)
        except Exception as e:  # noqa: BLE001 -- observation must never break a mission
            logger.debug("Yuk son konumu alinamadi: %s", e)
            pose = None
        if pose is None:
            logger.info("[PAYLOAD_FINAL_POSE] %s: konum alinamadi.", shape_type)
            self._publish("PAYLOAD_FINAL_POSE", shape_type, severity=Severity.WARN,
                          data={"shape_type": shape_type, "available": False})
            return
        x, y, z = pose
        on_ground = z > -0.05
        data = {"shape_type": shape_type, "available": True,
                "x": round(x, 3), "y": round(y, 3), "z": round(z, 3),
                "settled_above_ground": on_ground,
                "delay_s": PAYLOAD_FINAL_POSE_DELAY_S}

        # F3: settled_above_ground only ever answered "did it fall through
        # the world". It called a payload lying 4.9 m off the triangle a
        # success. The real question has two halves -- did it land on the
        # SHAPE, and is it lying FLAT on it -- and both are now measured.
        reference = getattr(self.actuator, "landing_reference", lambda _s: None)(shape_type)
        tilt_deg = getattr(self.actuator, "get_released_payload_tilt_deg",
                           lambda _s: None)(shape_type)
        if tilt_deg is not None:
            data["tilt_deg"] = tilt_deg
        on_target = on_ground
        if reference is not None:
            cx, cy, expected_z = reference
            offset_m = ((x - cx) ** 2 + (y - cy) ** 2) ** 0.5
            z_error = abs(z - expected_z)
            on_target = (on_ground and offset_m <= PAYLOAD_ON_TARGET_RADIUS_M
                         and z_error <= PAYLOAD_ON_TARGET_Z_TOLERANCE_M)
            data.update(target_x=cx, target_y=cy,
                        offset_from_center_cm=round(offset_m * 100.0, 1),
                        expected_rest_z_m=expected_z, z_error_m=round(z_error, 3),
                        on_target_radius_m=PAYLOAD_ON_TARGET_RADIUS_M,
                        on_target_z_tolerance_m=PAYLOAD_ON_TARGET_Z_TOLERANCE_M)
        data["settled_on_target"] = on_target

        if not on_ground:
            verdict = "DUNYANIN ICINE DUSTU"
        elif on_target:
            verdict = "HEDEFTE"
        else:
            verdict = "HEDEF DISINDA"
        logger.info("[PAYLOAD_FINAL_POSE] %s: x=%.3f y=%.3f z=%.3f%s%s (%s)",
                    shape_type, x, y, z,
                    "" if reference is None else
                    f" merkeze {data['offset_from_center_cm']:.1f} cm",
                    "" if tilt_deg is None else f" egim {tilt_deg:.1f} deg",
                    verdict)
        self._publish("PAYLOAD_FINAL_POSE", shape_type,
                      severity=Severity.INFO if on_target else Severity.WARN,
                      data=data)

    async def _publish_release_offset(self, shape_type: str) -> None:
        """Operator request (2026-08-17): the offset at the INSTANT the
        servo fires -- the number that actually decides where the payload
        lands. Goal is |offset| < 5 cm at the 0.45m release altitude.
        Read-only; it does not gate the release."""
        target = self.detection_feed.get(shape_type)
        try:
            _, _, alt_m = await self.flight.get_global_position()
        except Exception:  # noqa: BLE001 -- never let a measurement block a release
            alt_m = None

        data = {"shape_type": shape_type, "altitude_m": round(alt_m, 3) if alt_m else None,
                "target_visible": target is not None,
                # ADR-010 P1: BEST-KNOWN offset. When the target is not
                # visible at release altitude (the normal case below
                # LOW_ALT_VISION_LIMIT_M) the live measurement is impossible,
                # so the last committed one is reported together with the
                # altitude it was measured at. Reporting it without that
                # altitude would imply a precision the number does not have.
                "best_known_offset_cm": self._last_offset_cm,
                "best_known_offset_alt_m": self._last_offset_alt_m}
        if target is not None and alt_m:
            res_w, res_h = self.camera.get_resolution()
            intrinsics = default_camera_intrinsics()
            if intrinsics is not None:
                focal = intrinsics.scaled_to(res_w, res_h).focal_px
                # A2: measured from the PAYLOAD's position in frame.
                aim_dx, aim_dy = self._aim_offset_px(shape_type, alt_m, focal)
                dx = target.center_px[0] - res_w / 2.0 - aim_dx
                dy = target.center_px[1] - res_h / 2.0 - aim_dy
                px_to_cm = alt_m * 100.0 / focal
                data.update(dx_cm=round(dx * px_to_cm, 2), dy_cm=round(dy * px_to_cm, 2),
                            offset_cm=round((dx * dx + dy * dy) ** 0.5 * px_to_cm, 2))
        logger.info(f"[RELEASE_OFFSET] {shape_type}: {data}")
        self._publish("PAYLOAD_RELEASE_OFFSET", shape_type, data=data)

    async def _verify_marker(self, shape_type: str) -> bool:
        """Görev 2 Rapor Bölüm 13: bırakılan yükün doğrulama işaretini
        (Kırmızı/Mavi Dikdörtgen) arar. Yalnızca bilgilendirme amaçlıdır --
        sonucu ne olursa olsun görev akışını durdurmaz."""
        expected_marker = VERIFICATION_MARKER.get(shape_type)
        if not expected_marker:
            logger.warning(f"Doğrulama işareti bulunamadı: {shape_type}")
            return False

        logger.info(f"Doğrulama işareti aranıyor: {expected_marker}")

        # ADR-008 B1: reads the orchestrator's detection loop instead of
        # running its own detect(). A stale feed yields no detections, so a
        # dead vision pipeline reports "not verified" rather than silently
        # re-checking a frozen result.
        detections = self.detection_feed.detections()

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
