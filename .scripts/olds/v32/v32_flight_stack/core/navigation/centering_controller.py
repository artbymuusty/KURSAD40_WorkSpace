import logging
import asyncio
from core.interfaces.i_flight_backend import IFlightBackend
from core.interfaces.i_detector import IDetector
from core.interfaces.i_camera_source import ICameraSource
from core.detection.types import Detection
from core.config.parameters import (
    MISSION_ALTITUDE_M, HOVER_DURATION_S, MAX_CENTERING_SPEED_M_S,
    OFFBOARD_SETPOINT_INTERVAL_S, OFFBOARD_MODE_CONFIRM_TIMEOUT_S,
    CENTERING_TOLERANCE_X_NORM, CENTERING_TOLERANCE_Y_NORM,
    KP_ALTITUDE, ALTITUDE_CONVERGENCE_TOLERANCE_M, CENTERING_LATERAL_TIMEOUT_S,
    GPS_POSITION_CONVERGENCE_TOLERANCE_M, GPS_POSITION_VELOCITY_TOLERANCE_M_S,
    GLOBAL_POSITION_NAV_TIMEOUT_S,
)
from core.navigation.geo import gps_to_ned_delta, haversine_distance_m
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger(__name__)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class CenteringController:
    def __init__(self, flight: IFlightBackend, detector: IDetector, camera: ICameraSource,
                 publisher: EventPublisher = NULL_PUBLISHER):
        self.flight = flight
        self.detector = detector
        self.camera = camera
        self.publisher = publisher
        # TODO[KONTROL]: Gerçek kapalı döngü kazanç (gain) değerleri fiziksel testlerle
        # ayarlanacaktır (bkz. real_system.yaml / gz_system.yaml config parametreleri
        # Kp_yatay, Kp_dikey). Bu değerler artık GERÇEKTEN kullanılıyor (bkz.
        # go_to_and_center) -- daha önce tanımlı olup hiçbir yerde okunmuyorlardı.
        self.kp_horizontal = 0.5
        self.kp_vertical = 0.3
        # Operator-specified precision (2026-08-13 revision): ±0.01 normalized
        # in both axes, config-injected per real_system.yaml/gz_system.yaml
        # exactly like kp_horizontal/kp_vertical above.
        self.tolerance_x = CENTERING_TOLERANCE_X_NORM
        self.tolerance_y = CENTERING_TOLERANCE_Y_NORM
        self.kp_altitude = KP_ALTITUDE
        # BUG FIX (operator-reported, 2026-08-13): the lateral-only (no
        # altitude change) branch of go_to_and_center's max_attempts used to
        # be a fixed 30 (3s), sized for the old 20px tolerance -- too short
        # for the new, much tighter ±0.01 normalized precision to reliably
        # converge against real detector/control noise. Overridable per-call
        # like tolerance_x/y (tests can shrink this for speed).
        self.lateral_timeout_s = CENTERING_LATERAL_TIMEOUT_S

    def _publish(self, code, message="", severity=Severity.INFO, data=None):
        self.publisher.publish(Event(
            code=code, subsystem="CenteringController", category=Category.NAVIGATION,
            severity=severity, message=message, data=data or {},
        ))

    async def switch_to_offboard(self) -> bool:
        """Mission modu durur, Offboard'a geçilir (Bölüm 8).

        BUG FIX (operator-reported): previously this called
        switch_to_offboard_from_mission() + start_offboard() and returned
        None unconditionally -- nothing ever confirmed PX4 actually
        accepted the mode change, and an OffboardError from PX4 rejecting
        it would propagate uncaught all the way out of
        Gorev2Orchestrator.run(), aborting the entire Görev 2 mission over
        a single failed engagement attempt. Now returns bool: the caller
        must check it and fall back to SEARCHING instead of blindly
        proceeding into go_to_and_center() while still in Mission mode."""
        logger.info("Mission modu durduruluyor, Offboard'a geciliyor...")
        await self.flight.switch_to_offboard_from_mission()

        try:
            await self.flight.start_offboard()
        except Exception as e:
            logger.error(f"Offboard baslatma reddedildi: {e}")
            self._publish("OFFBOARD_SWITCH_FAILED", str(e), severity=Severity.CRITICAL, data={"error": str(e)})
            return False

        # Verify PX4 actually reports OFFBOARD instead of trusting
        # start_offboard()'s mere absence of an exception -- PX4 can accept
        # the command and still not be in OFFBOARD a moment later for
        # reasons the MAVSDK call itself won't surface.
        deadline = asyncio.get_event_loop().time() + OFFBOARD_MODE_CONFIRM_TIMEOUT_S
        while asyncio.get_event_loop().time() < deadline:
            mode = await self.flight.get_flight_mode()
            if mode == "OFFBOARD":
                self._publish("OFFBOARD_SWITCH_CONFIRMED", data={"flight_mode": mode})
                return True
            await asyncio.sleep(0.2)

        logger.error("PX4 OFFBOARD modunu onaylamadi (timeout).")
        self._publish("OFFBOARD_SWITCH_FAILED", "PX4 did not report OFFBOARD before timeout",
                      severity=Severity.CRITICAL, data={"timeout_s": OFFBOARD_MODE_CONFIRM_TIMEOUT_S})
        return False

    async def go_to_and_center(self, target_shape_type: str, altitude_m: float = MISSION_ALTITUDE_M) -> bool:
        """Aracı hedefe yönlendirir, irtifayı korur, şekli kare merkezine getirir.
        Merkezleme tamamlanınca True döner.

        BUG FIX (operator-reported): this used to compute pixel error and
        just check it against a threshold -- it never called
        set_velocity_body() at all, so nothing ever drove the vehicle
        toward the target, and PX4 auto-exits Offboard after ~500ms without
        a new setpoint regardless. Now streams a real proportional-control
        velocity setpoint every iteration (well under PX4's Offboard
        timeout), and always ends on an explicit zero-velocity stop so no
        residual drift carries into hover_and_confirm()."""
        logger.info(f"{target_shape_type} hedefine merkezleniyor (irtifa hedefi: {altitude_m}m)...")
        self._publish("CENTERING_STARTED", target_shape_type, data={"shape_type": target_shape_type, "altitude_m": altitude_m})

        # GAP FIX (operator revision, 2026-08-13): `altitude_m` was a dead
        # parameter -- nothing in this loop ever read it or commanded a
        # vertical setpoint, so every call centered laterally at whatever
        # altitude the vehicle already happened to be at. This is what makes
        # the staged payload approach (15m -> 10m -> 5m -> 0.30m, each step
        # re-centered) possible: this same loop now also closes the
        # altitude loop every iteration.
        #
        # max_attempts: the altitude loop is pure proportional control
        # (down_m_s = kp_altitude * alt_error), which only *asymptotically*
        # approaches the target -- a naive "distance / max_speed" time
        # estimate covers the initial clamped-speed phase but badly
        # underestimates the long decaying tail as alt_error shrinks toward
        # ALTITUDE_CONVERGENCE_TOLERANCE_M (verified: with kp_altitude=0.5,
        # a 5m descent needs ~55-60 iterations, not the ~25 a linear
        # estimate would suggest). Rather than an exact analytical settling-
        # time formula that's fragile to get right for arbitrary future
        # kp_altitude values, use a large fixed budget for any real altitude
        # change (200x0.1s=20s, comfortably covers even the largest jump --
        # the post-drop 0.30m -> MISSION_ALTITUDE_M climb-back) and
        # self.lateral_timeout_s (default 15s -- see its own BUG FIX comment
        # in __init__) for same-altitude lateral-only calls, i.e. the FIRST
        # lock-on pass at mission altitude.
        _, _, start_alt = await self.flight.get_global_position()
        lateral_only_attempts = int(self.lateral_timeout_s / OFFBOARD_SETPOINT_INTERVAL_S)
        max_attempts = lateral_only_attempts if abs(start_alt - altitude_m) < ALTITUDE_CONVERGENCE_TOLERANCE_M else 200

        converged = False
        for _ in range(max_attempts):
            frame = await self.camera.get_frame()
            detections = await self.detector.detect(frame)

            target = next((d for d in detections if d.shape_type == target_shape_type), None)
            if not target:
                logger.warning("Merkezleme sirasinda hedef kayboldu!")
                # Keep streaming a hold (zero-velocity) setpoint rather than
                # going silent -- a silent gap here is exactly what lets
                # PX4 fall back out of Offboard mid-pursuit.
                await self.flight.set_velocity_body(0.0, 0.0, 0.0, 0.0)
                await asyncio.sleep(0.5)
                continue

            res_w, res_h = self.camera.get_resolution()
            center_x, center_y = res_w / 2.0, res_h / 2.0

            error_x = target.center_px[0] - center_x
            error_y = target.center_px[1] - center_y

            # Downward-facing camera (x500_mono_cam_down): image "up" is
            # aligned with body-forward. Target below center (error_y > 0)
            # is physically behind the vehicle -> negative forward_m_s;
            # target right of center (error_x > 0) is physically to the
            # vehicle's right -> positive right_m_s. Sign convention to be
            # confirmed against the real camera mount during physical
            # testing (see kp_horizontal/kp_vertical TODO above) -- the
            # part that was actually missing is that a command is sent at
            # all, every iteration, not just computed and discarded.
            error_x_norm = error_x / center_x if center_x else 0.0
            error_y_norm = error_y / center_y if center_y else 0.0

            _, _, current_alt = await self.flight.get_global_position()
            alt_error = current_alt - altitude_m

            # Operator precision requirement (2026-08-13 revision): ±0.01
            # normalized in both axes, replacing the previous raw-pixel
            # threshold -- and now also gated on altitude, since this loop
            # actually commands descent/climb.
            if (abs(error_x_norm) < self.tolerance_x and abs(error_y_norm) < self.tolerance_y
                    and abs(alt_error) < ALTITUDE_CONVERGENCE_TOLERANCE_M):
                logger.info("Merkezleme tamamlandi.")
                converged = True
                break

            right_m_s = _clamp(error_x_norm * self.kp_horizontal * MAX_CENTERING_SPEED_M_S, MAX_CENTERING_SPEED_M_S)
            forward_m_s = _clamp(-error_y_norm * self.kp_vertical * MAX_CENTERING_SPEED_M_S, MAX_CENTERING_SPEED_M_S)
            # NED down is positive-downward; alt_error > 0 means "too high"
            # (current > target) so a positive down_m_s (descend) is correct.
            down_m_s = _clamp(alt_error * self.kp_altitude, MAX_CENTERING_SPEED_M_S)

            await self.flight.set_velocity_body(forward_m_s, right_m_s, down_m_s, 0.0)
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)

        # Explicit stop -- always sent, converged or not, so no residual
        # velocity command carries into whatever runs next.
        await self.flight.set_velocity_body(0.0, 0.0, 0.0, 0.0)

        if converged:
            self._publish("CENTERING_CONVERGED", target_shape_type, data={"shape_type": target_shape_type, "altitude_m": altitude_m})
            return True

        logger.error("Merkezleme zaman asimina ugradi!")
        self._publish("CENTERING_TIMED_OUT", target_shape_type, severity=Severity.WARN,
                      data={"shape_type": target_shape_type, "altitude_m": altitude_m, "max_attempts": max_attempts})
        return False

    async def nudge_forward(self, distance_m: float, speed_m_s: float = 0.3) -> None:
        """Sabit bir hızda kısa bir süre ileri hareket ederek yaklaşık
        `distance_m` kadar yol alır (Görev 2 Rapor: yük bırakma öncesi
        '10 cm ileri hareket'). Süre-tabanlı bir tahmindir -- gerçek mesafe
        rüzgar/gecikme nedeniyle sapabilir; kritik değildir çünkü bu son
        adım zaten SERVO tetiklemesinden hemen önce gelir ve yük bırakma
        pozisyonu yalnızca kabaca bu kadar ileride olmalıdır.

        Diğer tüm metodlarla aynı sürekli-akış güvenliği: PX4 ~500ms
        setpoint'siz kalırsa Offboard'dan çıkar, bu yüzden tek seferlik bir
        komut değil, süre boyunca tekrarlanan bir akış gönderilir."""
        if distance_m <= 0 or speed_m_s <= 0:
            return
        duration_s = distance_m / speed_m_s
        logger.info(f"{distance_m}m ileri hareket ediliyor ({speed_m_s} m/s, {duration_s:.1f}s)...")
        self._publish("NUDGE_FORWARD_STARTED", data={"distance_m": distance_m, "speed_m_s": speed_m_s})

        deadline = asyncio.get_event_loop().time() + duration_s
        while asyncio.get_event_loop().time() < deadline:
            await self.flight.set_velocity_body(speed_m_s, 0.0, 0.0, 0.0)
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)

        await self.flight.set_velocity_body(0.0, 0.0, 0.0, 0.0)
        self._publish("NUDGE_FORWARD_DONE", data={"distance_m": distance_m})

    async def climb_to_altitude(self, target_altitude_m: float, timeout_s: float = 20.0) -> bool:
        """Görüntüden bağımsız dikey hareket -- go_to_and_center()'ın aksine
        hedefin görüntüde olmasını GEREKTİRMEZ. Yük bırakma sonrası (araç
        yere yakın, az önce ileri kaydı, hedef artık kare içinde
        olmayabilir) MISSION_ALTITUDE_M'e geri tırmanmak için kullanılır --
        go_to_and_center burada kullanılsaydı hedef görünmediği sürece
        sadece beklerdi ve zaman aşımına uğrardı, hiç tırmanmadan."""
        logger.info(f"{target_altitude_m}m irtifasina tirmaniliyor (goruntuden bagimsiz)...")
        self._publish("CLIMB_STARTED", data={"target_altitude_m": target_altitude_m})

        deadline = asyncio.get_event_loop().time() + timeout_s
        converged = False
        while asyncio.get_event_loop().time() < deadline:
            _, _, current_alt = await self.flight.get_global_position()
            alt_error = current_alt - target_altitude_m
            if abs(alt_error) < ALTITUDE_CONVERGENCE_TOLERANCE_M:
                converged = True
                break
            down_m_s = _clamp(alt_error * self.kp_altitude, MAX_CENTERING_SPEED_M_S)
            await self.flight.set_velocity_body(0.0, 0.0, down_m_s, 0.0)
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)

        await self.flight.set_velocity_body(0.0, 0.0, 0.0, 0.0)
        self._publish("CLIMB_DONE" if converged else "CLIMB_TIMED_OUT",
                      data={"target_altitude_m": target_altitude_m},
                      severity=Severity.INFO if converged else Severity.WARN)
        return converged

    async def goto_global_position_and_wait(self, target_lat: float, target_lon: float,
                                              target_alt_m: float,
                                              timeout_s: float = GLOBAL_POSITION_NAV_TIMEOUT_S) -> bool:
        """Görev 2 Rapor (operatör revizyonu, 2026-08-13 "Mission Lifecycle"
        yeniden yapılandırması): Search tamamlandığında Payload Mission 1/2
        için kaydedilen GPS konumuna dönüş -- araç o an ikinci hedefin
        yakınında olabilir, kaydedilen konumda DEĞİL. Görüntüden bağımsız
        (climb_to_altitude gibi): hedef şeklin o an kamerada görünmesi
        gerekmez, yalnızca GPS mesafesi kullanılır.

        BUG FIX (regression investigation, 2026-08-13): this used to
        recompute gps_to_ned_delta(CURRENT, target) fresh every iteration
        and send that -- a value that shrinks toward (0,0) as the vehicle
        approaches -- straight into goto_position_ned(), which sends an
        ABSOLUTE local-NED setpoint (proven earlier this session: repeated
        identical PositionNedYaw setpoints hold the vehicle at one fixed
        point, not a moving one). Feeding a moving relative delta into an
        absolute-position API means the vehicle chases a different,
        physically meaningless point every iteration -- proven via live
        instrumentation to produce a chaotic, non-monotonic trajectory
        (distance swinging between ~1.5m and ~26m repeatedly) rather than a
        smooth approach. Compounding this, the old convergence check only
        looked at position, not velocity -- proven to fire while the
        vehicle was moving at ~11 m/s mid-flight through the target's 2m
        radius, coasting ~10-25m past afterward with nothing correcting it.

        Fixed by restoring the two principles the prior (pre-Mission-
        Lifecycle-revision) codebase's proven-working equivalent always
        used (.scripts/olds/v32/mission.py::_state_return_home, used only
        as a behavioral reference -- not copied, not reintroduced as a
        dependency): (1) the target's absolute local-NED position is
        computed ONCE, by adding a GPS-derived delta to a single
        get_position_ned() snapshot, and sent unchanged on every
        iteration -- not recomputed from a moving current position; (2)
        convergence requires 3D velocity magnitude to also be below
        GPS_POSITION_VELOCITY_TOLERANCE_M_S, not position alone. No new
        navigation framework introduced -- still goto_position_ned(),
        still this same function's existing structure."""
        logger.info(f"Kayitli GPS konumuna gidiliyor: lat={target_lat}, lon={target_lon}, alt={target_alt_m}m")
        self._publish("GLOBAL_POSITION_NAV_STARTED",
                      data={"target_lat": target_lat, "target_lon": target_lon, "target_alt_m": target_alt_m})

        # Fixed absolute local-NED target, computed once -- see BUG FIX note
        # above. start_n/start_e are already in PX4's true local-NED frame
        # (get_position_ned()); adding the GPS-derived delta translates
        # that frame's origin-relative coordinates to the target's position
        # in the SAME frame, without ever needing to know the EKF origin's
        # own GPS coordinate directly.
        start_lat, start_lon, _ = await self.flight.get_global_position()
        start_n, start_e, _ = await self.flight.get_position_ned()
        delta_n, delta_e = gps_to_ned_delta(start_lat, start_lon, target_lat, target_lon)
        target_n, target_e = start_n + delta_n, start_e + delta_e

        deadline = asyncio.get_event_loop().time() + timeout_s
        converged = False
        while asyncio.get_event_loop().time() < deadline:
            current_lat, current_lon, current_alt = await self.flight.get_global_position()
            distance_m = haversine_distance_m(current_lat, current_lon, target_lat, target_lon)
            alt_error = current_alt - target_alt_m

            vel_n, vel_e, vel_d = await self.flight.get_velocity_ned()
            speed = (vel_n ** 2 + vel_e ** 2 + vel_d ** 2) ** 0.5

            if (distance_m < GPS_POSITION_CONVERGENCE_TOLERANCE_M
                    and abs(alt_error) < ALTITUDE_CONVERGENCE_TOLERANCE_M
                    and speed < GPS_POSITION_VELOCITY_TOLERANCE_M_S):
                converged = True
                break

            yaw = await self.flight.get_yaw_deg()
            await self.flight.goto_position_ned(target_n, target_e, -target_alt_m, yaw)
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)

        self._publish("GLOBAL_POSITION_NAV_CONVERGED" if converged else "GLOBAL_POSITION_NAV_TIMED_OUT",
                      data={"target_lat": target_lat, "target_lon": target_lon},
                      severity=Severity.INFO if converged else Severity.WARN)
        return converged

    async def hover_and_confirm(self, duration_s: float = HOVER_DURATION_S) -> None:
        """flight.hold_position(duration_s) çağırır — GPS/görüntü stabilizasyonu ve konum
        doğrulaması için (Bölüm 9). hold_position() artık süre boyunca setpoint
        akışını KENDİSİ sürdürüyor (bkz. MavsdkBackendBase.hold_position) -- bu yüzden
        burada ayrıca sessizce uyumaya gerek yok; o da PX4'ün Offboard'dan
        düşmesine yol açan sessiz bir boşluktu."""
        logger.info(f"{duration_s} saniye hover yapiliyor (Konum dogrulama)...")
        self._publish("HOVER_STARTED", data={"duration_s": duration_s})
        await self.flight.hold_position(duration_s)
        self._publish("HOVER_CONFIRMED", data={"duration_s": duration_s})
