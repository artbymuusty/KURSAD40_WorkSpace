import logging
import time
import asyncio
from core.interfaces.i_flight_backend import IFlightBackend
from core.interfaces.i_camera_source import ICameraSource
from core.interfaces.i_detector import IDetector
from core.interfaces.i_payload_actuator import IPayloadActuator

from core.mission.interlock import PayloadInterlock
from core.position_log.position_store import PositionStore
from core.mission.debounce import DebounceTracker
from core.detection.target_validator import TargetValidator
from core.detection.target_selector import TargetSelector
from core.navigation.centering_controller import CenteringController
from core.mission.gorev2_fsm import PayloadMissionSequencer
from core.navigation.checkpoint import MissionCheckpoint
from core.mission.payload_release import PayloadReleaseService
from core.mission.context import MissionContext
from core.mission.phase import MissionPhase
from core.mission.blocking import BlockingKind
from core.config.parameters import (
    MISSION_ALTITUDE_M, CONNECTION_ESTABLISH_TIMEOUT_S, MISSION_UPLOAD_ACK_TIMEOUT_S,
    CENTERING_CONVERGENCE_TIMEOUT_S, OPERATOR_MISSION_START_TIMEOUT_S,
)
from core.telemetry.event_bus import EventPublisher, NULL_PUBLISHER
from core.telemetry.events import Category, Event, Severity
from core.telemetry.frame_channel import FrameChannel

logger = logging.getLogger(__name__)

VISION_SUBSYSTEM = "Gorev2Orchestrator.vision"

# ADR-004 §5 (#5 CLIMB_TO_ALTITUDE): replaces the previous blind
# `asyncio.sleep(5.0)` "wait" -- that was not a real check at all, just a
# guess; if takeoff was slow the orchestrator proceeded believing it was at
# altitude regardless. This polls real telemetry instead, bounded by a
# timeout so it becomes a reportable BLOCKING_WAIT instead of a silent one.
_ALTITUDE_TOLERANCE_M = 1.0
_ALTITUDE_POLL_INTERVAL_S = 0.5
_ALTITUDE_POLL_TIMEOUT_S = 30.0


class Gorev2Orchestrator:
    def __init__(self, flight: IFlightBackend, camera: ICameraSource, detector: IDetector, actuator: IPayloadActuator,
                 interlock: PayloadInterlock, position_store: PositionStore, debounce: DebounceTracker,
                 validator: TargetValidator, selector: TargetSelector, centering: CenteringController,
                 sequencer: PayloadMissionSequencer, checkpoint: MissionCheckpoint, release_service: PayloadReleaseService,
                 context: MissionContext = None, publisher: EventPublisher = NULL_PUBLISHER,
                 frame_channel: FrameChannel = None):
        self.flight = flight
        self.camera = camera
        self.detector = detector
        self.actuator = actuator
        self.interlock = interlock
        self.position_store = position_store
        self.debounce = debounce
        self.validator = validator
        self.selector = selector
        self.centering = centering
        self.sequencer = sequencer
        self.checkpoint = checkpoint
        self.release_service = release_service
        # Operator revision (2026-08-13): one-way Search->Offboard authority
        # guard. Once True, _resume_mission_route() permanently refuses to
        # resume Mission -- the "Defensive Resume Guard" the spec requires,
        # enforced in code rather than by caller convention.
        self._search_complete: bool = False
        # MissionOpsDashboard itself is deliberately NOT injected here
        # (ADR-004 §3: outbound-only edge) -- this class only ever publishes
        # events/frames; the dashboard consumes them independently. The
        # camera feed is the one deliberate exception to "structured events
        # only": it's a local GCS-side resource (vision runs on this same
        # machine per the Görev 2 mandate), never a call across the
        # vehicle's MAVLink/telemetry link, so a lightweight, non-blocking
        # FrameChannel handoff carries it to the dashboard without going
        # through EventBus/EventStore's structured-telemetry pipeline.
        self.context = context or MissionContext(publisher=publisher)
        self.publisher = publisher
        self.frame_channel = frame_channel
        # Shared between _frame_grab_loop and _detection_loop (see run()) --
        # plain attribute is safe here: both are coroutines on the same
        # single-threaded asyncio loop, so assignment/read never races
        # mid-instruction, only ever interleaves at await points.
        self._latest_detections: list = []
        # BUG FIX (continuous audit, 2026-08-13): _detection_loop() runs for
        # the whole span of run() -- which, since the Mission Lifecycle
        # restructuring, now covers the full payload mission sequence too,
        # not just search. CenteringController.go_to_and_center() and
        # PayloadReleaseService._verify_marker() both independently call
        # self.detector.detect() on their own tight closed-loop cadence
        # during that same span -- a second, concurrently-scheduled
        # consumer of the SAME detector instance, which corrupts
        # HSVContourDetector's mutable per-shape streak state exactly like
        # the _search_and_engage_loop case (see that method's own BUG FIX
        # comment). Redirecting go_to_and_center/_verify_marker to consume
        # _latest_detections instead (like _search_and_engage_loop now does)
        # would couple a real-time control loop to an unrelated background
        # loop's cadence and add latency -- the correct fix is the other
        # direction: _detection_loop yields exclusive detector access
        # whenever a precision-control consumer needs it. Set True right
        # after switch_to_offboard() succeeds; cleared in
        # _resume_mission_route() (called on every "back to search" exit),
        # left True through the Payload Mission 1->2 sequence once Search
        # Phase permanently ends (resume is never called again at that point).
        self._precision_control_active: bool = False
        # RUNTIME INVESTIGATION (2026-08-13): tracks consecutive
        # camera/detector failures in _detection_loop, so a persistently
        # broken vision pipeline can be distinguished (and escalated) from
        # "vision is fine but genuinely sees nothing" -- see that method.
        self._vision_consecutive_failures: int = 0

    def _publish(self, code, message="", severity=Severity.INFO, category=Category.LIFECYCLE, subsystem="Gorev2Orchestrator", data=None):
        self.publisher.publish(Event(
            code=code, subsystem=subsystem, category=category, severity=severity, message=message, data=data or {},
        ))

    async def _wait_for_altitude(self, target_altitude_m: float) -> None:
        self.context.transition_to(MissionPhase.CLIMB_TO_ALTITUDE, reason=f"target={target_altitude_m}m")
        self.context.set_blocking("WAITING_ALTITUDE_REACHED", "Gorev2Orchestrator",
                                   BlockingKind.BLOCKING_WAIT, timeout_s=_ALTITUDE_POLL_TIMEOUT_S)
        deadline = time.time() + _ALTITUDE_POLL_TIMEOUT_S
        while time.time() < deadline:
            _, _, alt = await self.flight.get_global_position()
            if abs(alt - target_altitude_m) <= _ALTITUDE_TOLERANCE_M:
                self.context.clear_blocking()
                self._publish("ALTITUDE_REACHED", f"alt={alt:.1f}m", data={"altitude_m": alt})
                return
            await asyncio.sleep(_ALTITUDE_POLL_INTERVAL_S)
        # Does not abort the mission (takeoff already succeeded and PX4 is
        # still climbing) -- but the timeout firing is itself reportable so
        # an operator sees a slow climb instead of it looking identical to
        # a normal one.
        self._publish("ALTITUDE_REACHED", f"timed out waiting for {target_altitude_m}m, proceeding anyway",
                      severity=Severity.WARN, data={"target_altitude_m": target_altitude_m})

    async def run(self) -> None:
        """
        1. flight.arm(); flight.takeoff(MISSION_ALTITUDE_M)
        2. checkpoint.save(*await flight.get_global_position())
        3. flight.confirm_existing_mission() -- the operator's QGroundControl-
           defined route, NEVER generated/uploaded by this system; refuses
           to proceed if none is found. flight.start_mission()
        4. while not flight.is_mission_finished():
             frame = await camera.get_frame() ...
        """
        logger.info("GOREV 2 BASLIYOR")
        self._publish("MISSION_STARTED", "Gorev 2 starting")

        # BUG FIX: the camera/detector were previously only ever touched
        # inside _search_and_engage_loop(), which doesn't start until AFTER
        # connect/arm/takeoff/climb/checkpoint/upload/start_mission all
        # complete. That left the dashboard showing "WAITING FOR CAMERA
        # FEED..." (and Gorev2Orchestrator.vision reporting UNKNOWN health)
        # for the entire early phase of a real flight, even though the
        # camera and vision pipeline are already live and streaming real
        # footage at that point (confirmed against a running Gazebo sim:
        # drone airborne, camera producing frames, dashboard still stuck on
        # the placeholder). Görev 2 Bölüm 5.1 also specifies YOLO26 runs
        # continuously, not only once SEARCHING begins -- so these tasks are
        # spec-correct, not just a workaround. Neither touches
        # TargetValidator/DebounceTracker/TargetSelector; only
        # _search_and_engage_loop() drives actual tracking/engagement.
        #
        # Split into two independent tasks, not one, so live video display
        # is never gated by detection latency: _frame_grab_loop publishes
        # frames on its own fast, steady cadence; _detection_loop runs
        # detect() (potentially slow, e.g. real model inference) on its own
        # pace and only updates which boxes get overlaid on the next
        # published frame. Frames keep arriving even while a detection pass
        # is still running.
        frame_task = asyncio.ensure_future(self._frame_grab_loop())
        detection_task = asyncio.ensure_future(self._detection_loop())
        try:
            await self._run_inner()
        except Exception as e:
            self.context.transition_to(MissionPhase.MISSION_FAILED, reason=str(e))
            self._publish("MISSION_FAILED", str(e), severity=Severity.CRITICAL)
            raise
        finally:
            for task in (frame_task, detection_task):
                task.cancel()
            for task in (frame_task, detection_task):
                try:
                    await task
                except asyncio.CancelledError:
                    pass

    async def _frame_grab_loop(self) -> None:
        """Runs for the whole mission lifetime, independent of mission
        phase and of detection latency: continuously grabs frames and
        publishes them to the dashboard's FrameChannel, overlaid with
        whatever detections _detection_loop most recently computed (which
        may be very slightly behind the newest frame during heavy
        inference -- normal and expected, and far better than the video
        itself stalling)."""
        while True:
            try:
                frame = await self.camera.get_frame()
                if self.frame_channel is not None:
                    self.frame_channel.publish(frame, self._latest_detections)
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001 -- a preview-loop hiccup must never affect the mission
                logger.warning(f"Frame grab loop hatasi (mission etkilenmez): {e}")

            # ~30Hz, matching CAMERA_PROCESS_FREQ_HZ_MAX -- smooth playback,
            # decoupled from however long detection takes.
            await asyncio.sleep(1.0 / 30.0)

    async def _detection_loop(self) -> None:
        """Runs detect() at its own sustainable pace (never faster than
        inference allows, since it awaits the previous call's completion
        before starting the next) and publishes the vision heartbeat --
        HealthMonitor sees Gorev2Orchestrator.vision as HEALTHY from mission
        start, not only once SEARCHING begins.

        BUG FIX (continuous audit, 2026-08-13): skips its own detect() call
        while self._precision_control_active is True -- a precision-control
        consumer (CenteringController.go_to_and_center,
        PayloadReleaseService._verify_marker) owns the detector instance
        exclusively during that window (see the attribute's own comment in
        __init__ for the full race-condition this prevents). The dashboard
        overlay just shows the last-known detections, frozen, until control
        is released -- purely cosmetic staleness, not a mission-logic
        concern (nothing reads _latest_detections for mission logic while
        precision control is active)."""
        heartbeat_interval_s = 3.0
        last_heartbeat = 0.0
        while True:
            try:
                if not self._precision_control_active:
                    frame = await self.camera.get_frame()
                    detections = await self.detector.detect(frame)
                    self._latest_detections = detections

                    self._publish("VISION_FRAME_PROCESSED", severity=Severity.DEBUG, category=Category.VISION,
                                  subsystem=VISION_SUBSYSTEM,
                                  data={"detector_ready": True, "detection_count": len(self._latest_detections)})

                    # RUNTIME INVESTIGATION (2026-08-13): the only per-cycle
                    # vision signal used to be the DEBUG-severity structured
                    # event above -- invisible on plain console logging even
                    # after the handler-configuration fix
                    # (configure_all_loggers). Throttled INFO heartbeat so a
                    # "no detections at all" run is immediately
                    # distinguishable from "detection_loop isn't running" or
                    # "camera keeps failing" (see the except branch below).
                    now = time.time()
                    if now - last_heartbeat >= heartbeat_interval_s:
                        last_heartbeat = now
                        shapes = [d.shape_type for d in self._latest_detections]
                        logger.info(f"[VISION] detect() calisiyor -- bu karede {len(shapes)} tespit: {shapes}")
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001 -- a preview-loop hiccup must never affect the mission
                self._vision_consecutive_failures += 1
                now = time.time()
                if now - last_heartbeat >= heartbeat_interval_s:
                    last_heartbeat = now
                    logger.warning(f"[VISION] detect() basarisiz (art arda {self._vision_consecutive_failures}. hata, "
                                   f"mission etkilenmez): {e}")
                # RUNTIME INVESTIGATION (2026-08-13): a persistently failing
                # camera/detector (e.g. GzCameraSource never receiving a
                # frame) used to be indistinguishable from "vision is fine
                # but sees nothing" -- both looked like an empty
                # _latest_detections forever. 30 consecutive failures (~3s
                # at this loop's cadence) is a real, mission-critical
                # condition, escalated once, loud, to both the structured
                # event stream (dashboard-visible) and console.
                if self._vision_consecutive_failures == 30:
                    logger.error("[VISION] KRITIK: kamera/detector 3 saniyedir surekli hata veriyor -- "
                                 "vision pipeline calismiyor olabilir.")
                    self._publish("VISION_PIPELINE_DOWN", str(e), severity=Severity.CRITICAL,
                                  category=Category.VISION, subsystem=VISION_SUBSYSTEM,
                                  data={"consecutive_failures": self._vision_consecutive_failures})
                self._latest_detections = []
                continue
            self._vision_consecutive_failures = 0

            await asyncio.sleep(0.1)  # ~10Hz ceiling, matching CAMERA_PROCESS_FREQ_HZ_MIN

    async def _run_inner(self) -> None:
        self.context.transition_to(MissionPhase.CONNECTING)
        self.context.set_blocking("WAITING_MAVSDK_CONNECTION", "MavsdkBackendBase",
                                   BlockingKind.BLOCKING_WAIT, timeout_s=CONNECTION_ESTABLISH_TIMEOUT_S)
        try:
            await asyncio.wait_for(self.flight.connect(), timeout=CONNECTION_ESTABLISH_TIMEOUT_S)
        except asyncio.TimeoutError:
            raise RuntimeError(f"WAITING_MAVSDK_CONNECTION timed out after {CONNECTION_ESTABLISH_TIMEOUT_S:.0f}s")
        self.context.clear_blocking()

        self.context.transition_to(MissionPhase.ARMING)
        await self.flight.arm()

        self.context.transition_to(MissionPhase.TAKEOFF)
        await self.flight.takeoff(MISSION_ALTITUDE_M)

        await self._wait_for_altitude(MISSION_ALTITUDE_M)

        self.context.transition_to(MissionPhase.CHECKPOINT_SAVE)
        global_pos = await self.flight.get_global_position()
        self.checkpoint.save(*global_pos)  # publishes CHECKPOINT_SAVED itself
        logger.info(f"Start Checkpoint kaydedildi: {global_pos}")

        # BUG FIX (operator-reported): this used to call
        # _generate_square_mission() and upload_mission() here, which
        # SILENTLY OVERWROTE whatever search route the operator had already
        # planned and uploaded via QGroundControl before flight. The
        # operator explicitly does not want this -- route definition is
        # QGroundControl's job (Görev 2 Rapor: "QGroundControl: Operatörün
        # görev öncesi waypoint/tarama rotası tanımlaması"), not this
        # system's. This now only confirms a route is already present and
        # refuses to proceed if it isn't, instead of silently generating
        # and uploading one of its own.
        self.context.transition_to(MissionPhase.MISSION_ROUTE_CONFIRM)
        self.context.set_blocking("WAITING_EXISTING_MISSION", "MavsdkBackendBase",
                                   BlockingKind.BLOCKING_WAIT, timeout_s=MISSION_UPLOAD_ACK_TIMEOUT_S)
        item_count = await asyncio.wait_for(self.flight.confirm_existing_mission(), timeout=MISSION_UPLOAD_ACK_TIMEOUT_S)
        self.context.clear_blocking()

        if item_count == 0:
            raise RuntimeError(
                "MISSION_ROUTE_MISSING: no mission found on the vehicle. The operator must define the "
                "search route in QGroundControl and upload it before RUN MISSION -- this system will not "
                "generate or upload a route on its own."
            )

        self.context.transition_to(MissionPhase.MISSION_START)
        self.context.set_blocking("WAITING_OPERATOR_MISSION_START", "QGroundControl",
                                   BlockingKind.BLOCKING_WAIT, timeout_s=OPERATOR_MISSION_START_TIMEOUT_S)
        await self._wait_for_operator_mission_start()
        self.context.clear_blocking()

        self.context.transition_to(MissionPhase.SEARCHING)
        await self._search_and_engage_loop()

    async def _wait_for_operator_mission_start(self) -> None:
        """BUG FIX (operator revision, 2026-08-13, "mission_gz supervisor
        model"): this system must NEVER issue the MAVLink command that
        starts Mission mode -- starting the mission, exactly like uploading
        its route, is exclusively the operator's action in QGroundControl.
        _run_inner() previously called flight.start_mission() itself right
        after confirming a route was present, silently doing what only the
        operator's own Start Mission button is supposed to do. This instead
        blocks, polling get_flight_mode(), until Mission mode is externally
        observed active."""
        deadline = time.monotonic() + OPERATOR_MISSION_START_TIMEOUT_S
        last_log = 0.0
        while True:
            mode = await self.flight.get_flight_mode()
            if mode == "MISSION":
                logger.info("Operator started Mission mode in QGroundControl -- search phase beginning.")
                return
            now = time.monotonic()
            if now - last_log >= 5.0:
                last_log = now
                logger.info(f"[MISSION_START] Operatorun QGroundControl'de Start Mission basmasi "
                            f"bekleniyor (su anki mod: {mode})...")
            if now > deadline:
                raise RuntimeError(
                    f"OPERATOR_MISSION_START_TIMEOUT: operator did not start Mission mode in "
                    f"QGroundControl within {OPERATOR_MISSION_START_TIMEOUT_S:.0f}s of the route "
                    f"being confirmed present."
                )
            await asyncio.sleep(0.5)

    async def _resume_mission_route(self) -> None:
        """BUG FIX (operator-reported, 2026-08-13): "hedefe kilitleniyor ama
        sonrasında Offboard'da görevleri tamamlayamıyor". CenteringController.
        switch_to_offboard() unconditionally calls
        flight.switch_to_offboard_from_mission(), which pauses the PX4
        Mission (drone.mission.pause_mission()) -- but nothing anywhere in
        this file ever called start_mission() again. Every single pursuit,
        whether it succeeded, failed to switch to Offboard, or timed out
        centering, just relabeled the phase and looped back to SEARCHING
        with the Mission left permanently paused. PX4 auto-exits Offboard
        ~500ms after the last streamed setpoint (same hard behavior called
        out elsewhere in this codebase), so after the FIRST engagement
        attempt of any outcome, the vehicle was left with nothing actually
        flying it -- Mission paused, Offboard about to lapse. This is called
        after every abandoned or completed pursuit that does NOT end Görev 2
        (interlock.both_released() is the one case that intentionally stays
        in Offboard, per Görev 2 Rapor Bölüm 15 -- straight into Görev 3).

        DEFENSIVE RESUME GUARD (operator revision, 2026-08-13): once
        self._search_complete is True (both targets recorded), this is a
        hard, permanent no-op -- Mission Resume must be IMPOSSIBLE after
        Search Phase ends, enforced here regardless of which caller thinks
        it needs to resume. This is the single choke point every resume
        attempt in this class goes through, so guarding here closes the
        door for all of them at once."""
        if self._search_complete:
            logger.warning("MISSION_RESUME_REJECTED: Search Phase tamamlandi, Mission Resume kalici olarak yasak.")
            self._publish("MISSION_RESUME_REJECTED", severity=Severity.WARN,
                          data={"reason": "search_already_complete"})
            return
        # Releases exclusive detector access back to _detection_loop -- we're
        # returning to plain searching, no precision-control consumer needs
        # it anymore (see self._precision_control_active's own comment).
        self._precision_control_active = False
        try:
            await self.flight.stop_offboard()
        except Exception as e:  # noqa: BLE001 -- PX4 may reject this if Offboard was never actually confirmed active; not fatal
            logger.warning(f"stop_offboard sirasinda hata (yoksayiliyor): {e}")
        await self.flight.start_mission()
        self._publish("MISSION_ROUTE_RESUMED")

    async def _search_and_engage_loop(self) -> None:
        """Görev 2 Rapor (operatör revizyonu, 2026-08-13, "Mission Lifecycle"
        yeniden yapılandırması): Mission Mode ARTIK yalnızca SEARCH -- tespit
        ve kayıt. Yük bırakma işlemleri burada YAPILMAZ; bir hedef (MAVI_ALTIGEN
        veya KIRMIZI_UCGEN) doğrulanıp kaydedildiğinde, PositionStore.
        both_required_targets_found() henüz False ise Mission Route'a geri
        dönülür (`_resume_mission_route`); True olduğunda ise bu, KALICI,
        TEK YÖNLÜ bir geçiştir -- Mission bir daha ASLA resume edilmez
        (`self._search_complete` bayrağı, `_resume_mission_route`'un kendi
        savunma guard'ı üzerinden bunu yazılım seviyesinde zorunlu kılar).
        Search tamamlandığında döngüden çıkılır ve PayloadMissionSequencer
        Offboard'ın TEK yetkili olduğu evrede Payload Mission 1 -> 2'yi
        sabit sırada çalıştırır."""
        while (not self.position_store.both_required_targets_found()
               and not await self.flight.is_mission_finished()):
            now = time.time()
            # BUG FIX (continuous audit, 2026-08-13): this used to call
            # self.camera.get_frame() + self.detector.detect(frame) itself,
            # a SECOND call site independently competing with
            # _detection_loop() (started in run(), runs for the whole
            # mission) over the exact same self.detector instance.
            # HSVContourDetector carries real mutable per-shape streak state
            # (_last_center/_streak) that its N-consecutive-frame commit
            # logic depends on being fed a coherent sequence -- two
            # independently-scheduled asyncio tasks interleaving detect()
            # calls into it corrupts that sequence (streak advances against
            # an interleaved, incoherent frame order), making commit timing
            # scheduling-order-dependent instead of deterministic. This is
            # exactly what the class's own __init__ comment on
            # self._latest_detections already documented as the intended
            # fix ("avoids two independent detector.detect() calls
            # competing over the same frame stream") but never actually
            # implemented here. _detection_loop already publishes at the
            # same ~10Hz ceiling this loop was polling at, so reading its
            # result introduces no meaningful staleness.
            detections = self._latest_detections

            res_w, res_h = self.camera.get_resolution()
            frame_center = (res_w / 2.0, res_h / 2.0)

            # Yalnızca iki stratejik search hedefi önemlidir -- doğrulama
            # işaretleri (KIRMIZI_DIKDORTGEN vb.) Search Phase'in ilgi alanı
            # değildir, onlar yalnızca PayloadReleaseService'in yük bırakma
            # SONRASI doğrulamasında kullanılır. Zaten kaydedilmiş bir hedef
            # de bir daha asla yeniden işlenmez (INVALID STATE 7'nin
            # search-içi analoğu: aynı hedefin tekrar "keşfedilmesi" search
            # tamamlanma koşulunu bozmamalı).
            candidates = [
                d for d in detections
                if d.shape_type in ("MAVI_ALTIGEN", "KIRMIZI_UCGEN")
                and self.position_store.get(d.shape_type) is None
            ]

            for d in candidates:
                if self.debounce.is_in_cooldown(d.shape_type, now):
                    continue  # DebounceTracker already published DEBOUNCE_STATE_SYNC

                current_alt = (await self.flight.get_global_position())[2]
                self.validator.update(d, current_alt, frame_center)
                self._publish("TRACK_STATE_UPDATED", severity=Severity.DEBUG, category=Category.VISION,
                              data={"shape_type": d.shape_type, **self.validator.get_track_state(d.shape_type)})

                if not self.validator.is_track_ready(d.shape_type):
                    continue

                self.context.transition_to(MissionPhase.TARGET_TRACKING, reason=d.shape_type)
                selected, other = self.selector.select(candidates, frame_center)
                self._publish("TARGET_SELECTED", selected.shape_type, category=Category.VISION,
                              data={"shape_type": selected.shape_type, "confidence": selected.confidence})

                self.validator.set_navigating_to(selected.shape_type, True)

                self.context.transition_to(MissionPhase.SWITCH_TO_OFFBOARD)
                self._publish("MISSION_AUTHORITY_RELEASED", data={"reason": "target_pause"})
                offboard_ok = await self.centering.switch_to_offboard()

                if not offboard_ok:
                    # BUG FIX (operator-reported): switch_to_offboard() used
                    # to return None unconditionally -- a rejected or
                    # unconfirmed PX4 mode change was never checked here.
                    # Abandon this pursuit, resume Mission (search is not
                    # complete -- only one target could even be pending).
                    self.validator.set_navigating_to(selected.shape_type, False)
                    self.context.transition_to(MissionPhase.SEARCHING, reason="offboard_switch_failed")
                    await self._resume_mission_route()
                    continue

                self._publish("OFFBOARD_AUTHORITY_ACQUIRED", data={"reason": "target_pause"})
                # From here through the end of this pursuit (and, if this is
                # the second target, through the whole Payload Mission 1->2
                # sequence), go_to_and_center/_verify_marker need exclusive,
                # low-latency access to the detector -- _detection_loop
                # yields (see its own comment) until _resume_mission_route()
                # releases this back, or it stays True permanently once
                # Search Phase ends (resume is never called again then).
                self._precision_control_active = True
                self.context.transition_to(MissionPhase.GOTO_TARGET_CENTERING, reason=selected.shape_type)
                self.context.set_blocking("WAITING_CENTERING_CONVERGENCE", "CenteringController",
                                           BlockingKind.BLOCKING_WAIT, timeout_s=CENTERING_CONVERGENCE_TIMEOUT_S)
                converged = await self.centering.go_to_and_center(selected.shape_type)
                self.context.clear_blocking()

                if not converged:
                    # CenteringController already published CENTERING_TIMED_OUT.
                    self.context.transition_to(MissionPhase.SEARCHING, reason="centering_timed_out")
                    self.validator.set_navigating_to(selected.shape_type, False)
                    await self._resume_mission_route()
                    continue

                self.context.transition_to(MissionPhase.HOVER_CONFIRM)
                await self.centering.hover_and_confirm()

                self.context.transition_to(MissionPhase.GPS_SAVE)
                global_pos_after = await self.flight.get_global_position()

                order = "ilk" if len(self.position_store.all_points()) < 1 else "ikinci"
                tp = self.position_store.try_save(
                    shape_type=selected.shape_type,
                    confidence=selected.confidence,
                    is_centered=True,
                    hover_completed=True,
                    gps=global_pos_after,
                    detection_order=order
                )

                self.validator.set_navigating_to(selected.shape_type, False)

                if tp is None:
                    # PositionStore already published GPS_SAVE_REJECTED with the
                    # specific precondition that failed.
                    self.context.transition_to(MissionPhase.SEARCHING, reason="gps_save_rejected")
                    await self._resume_mission_route()
                    continue

                # PositionStore already published GPS_SAVE_CONFIRMED.
                self.debounce.mark_processed(selected.shape_type, now)
                self._publish("TARGET_CONFIRMED", selected.shape_type,
                              data={"shape_type": selected.shape_type, "detection_order": order})

                # THE core one-way transition (Görev 2 Rapor "Mission
                # Lifecycle" revizyonu): Mission NEVER performs payload
                # operations, only detect+record. Once BOTH are recorded,
                # Search Phase ends permanently -- no resume, ever.
                if self.position_store.both_required_targets_found():
                    logger.info("SEARCH COMPLETE: MAVI_ALTIGEN ve KIRMIZI_UCGEN ikisi de kaydedildi. "
                                "Mission kalici olarak sona erdi, Offboard tek yetkili.")
                    self._search_complete = True
                    self.context.transition_to(MissionPhase.SEARCH_COMPLETE)
                    self._publish("SEARCH_COMPLETE")
                    self._publish("SEARCH_WAYPOINTS_CANCELLED",
                                  data={"reason": "both_targets_found_remaining_waypoints_ignored"})
                    break  # exits the for-loop; while-loop condition is now also False

                # Search incomplete (only one of two found so far) -- return
                # to the operator's QGroundControl route to keep searching.
                self.context.transition_to(MissionPhase.SEARCHING, reason="single_target_recorded_resuming_route")
                await self._resume_mission_route()

            await asyncio.sleep(0.1)  # CPU yormamak için kısa bekleme

        if not self.position_store.both_required_targets_found():
            logger.warning("Gorev 2 Search Phase tamamlanamadi (mission bitti, iki hedef de bulunamadi).")
            self._publish("MISSION_FINISHED_UNEXPECTED",
                          "flight.is_mission_finished() returned True before both targets were found",
                          severity=Severity.CRITICAL, category=Category.WATCHDOG)
            self.context.transition_to(MissionPhase.MISSION_FAILED, reason="search_incomplete_mission_finished")
            return

        # OFFBOARD ONLY from here on -- Payload Mission 1 -> Payload Mission 2,
        # sabit sırada (bkz. PayloadMissionSequencer, gorev2_fsm.py).
        await self.sequencer.execute_all()
        self._publish_payload_sync()

        if self.interlock.both_released():
            logger.info("Gorev 2 tamamlandi (Payload Mission 1 + 2 basariyla calisti).")
            self.context.transition_to(MissionPhase.GOREV2_COMPLETE)
            self._publish("GOREV2_COMPLETE")
        else:
            # Should be unreachable -- PayloadMissionSequencer.execute_all()
            # runs both missions unconditionally and PayloadInterlock raises
            # if payload_2 is ever marked without payload_1 -- but treated
            # as a real failure, not silently ignored, if it somehow happens.
            self._publish("MISSION_FINISHED_UNEXPECTED", "payload sequence completed without both released",
                          severity=Severity.CRITICAL, category=Category.WATCHDOG)
            self.context.transition_to(MissionPhase.MISSION_FAILED, reason="payload_sequence_incomplete")

    def _publish_payload_sync(self) -> None:
        self._publish("PAYLOAD_STATE_SYNC", category=Category.PAYLOAD, data={
            "payload_1_released": self.interlock.payload_1_released,
            "payload_2_released": self.interlock.payload_2_released,
        })
