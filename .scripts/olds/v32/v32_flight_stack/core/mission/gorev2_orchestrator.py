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
from core.detection.camera_intrinsics import default_camera_intrinsics
from core.detection.detection_feed import DetectionFeed
from core.config.parameters import (
    MISSION_ALTITUDE_M, CONNECTION_ESTABLISH_TIMEOUT_S, MISSION_UPLOAD_ACK_TIMEOUT_S,
    MISSION_MODE_CONFIRM_TIMEOUT_S,
    CENTERING_MAX_ATTEMPTS_PER_TARGET, CENTERING_RETRY_COOLDOWN_S,
    MISSION_RESUME_MIN_INTERVAL_S, MISSION_MODE_CONFIRM_TIMEOUT_S,
    OFFBOARD_AFTER_RESUME_SETTLE_S, POST_LOCK_DRIFT_WINDOW_S,
    OFFBOARD_SETPOINT_INTERVAL_S,
)
# ADR-007: referenced through the module (not imported by value) so the
# pre-start settling hold can be zeroed in tests -- it is a real-flight
# stabilisation delay, and a 3s wall-clock sleep inside every orchestrator
# run would otherwise dominate the suite and break time-bounded tests.
from core.config import parameters as _params
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
                 frame_channel: FrameChannel = None, detection_feed: DetectionFeed = None,
                 vision_runtime=None):
        self.flight = flight
        self.camera = camera
        # ADR-010 P3: kept on the constructor for callers/tests, but this
        # class no longer calls detect() -- VisionRuntime is the single
        # owner. Deliberately NOT removed from the signature: the parameter
        # is how a caller declares which detector the mission runs, and
        # main_gz hands the same instance to VisionRuntime.
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
        # ADR-008 B1: the ONE place a detection result is published, and the
        # only thing any other component reads one from. _detection_loop()
        # is its sole producer; CenteringController and
        # PayloadReleaseService are consumers.
        #
        # This replaces the _precision_control_active exclusivity flag that
        # used to live here. That flag existed for a real reason -- two
        # independently-scheduled callers of the same HSVContourDetector
        # corrupt its per-shape streak state -- but it solved it by muting
        # the detection loop (and, fatally, its VISION_FRAME_PROCESSED
        # heartbeat) for the whole duration of every pursuit. Proven on the
        # 2026-08-16 21:04 run: zero detection cycles across 77.3s and
        # 85.2s of centering, vision health DOWN within 0.3s of each
        # centering start (VISION_HEARTBEAT_INTERVAL_S=0.1 x grace 3.0),
        # and -- because it stayed True permanently once _search_complete
        # blocked _resume_mission_route() -- it would have stayed muted for
        # the entire payload phase too. Keeping exactly one detector
        # consumer removes the race without any flag to get stuck.
        # ADR-010 P3: this class is now a pure CONSUMER of the feed. The
        # producer (and the _latest_detections / _vision_consecutive_failures
        # bookkeeping that went with it) moved to
        # core/detection/vision_runtime.py so the pipeline outlives Görev 2 --
        # see that module for the measurement that forced the move.
        self.detection_feed = detection_feed or DetectionFeed()
        # ADR-010 P3: optional, and never constructed here -- see run().
        self.vision_runtime = vision_runtime
        # ADR-009 D3: per-shape centering-attempt budget. A failed pursuit
        # used to re-engage the same target the moment it was track-ready
        # again -- which, with a fast failure, is the very next loop
        # iteration. The 2026-08-16 23:06 run turned that into 585 pursuits
        # and 601 Mission resumes in 3.5 minutes (~3 PX4 pause/resume cycles
        # per second) until PX4 stopped answering and pause_mission() timed
        # out. The route also never advanced: the Mission spent more time
        # paused than flying. Both are prevented here rather than in the
        # controller, because "how many times do we chase this shape" is a
        # search-policy question, not a control-loop one.
        self._centering_attempts: dict = {}
        self._centering_cooldown_until: dict = {}
        self._centering_abandoned: set = set()
        # ADR-009 PX4-STABILITY: wall-clock of the last Mission resume,
        # so consecutive resumes can be spaced out (see _resume_mission_route).
        self._last_resume_at: float = 0.0

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

        # ADR-010 P3: the frame-grab and detection loops USED to be started
        # here and cancelled in this method's `finally`. That gave them
        # Görev 2's lifetime instead of the mission's, so they died at
        # GOREV2_COMPLETE -- measured on V1''': the last
        # VISION_FRAME_PROCESSED lands at t=236.3s, exactly the
        # GOREV2_COMPLETE -> GOREV3_START transition, and the remaining
        # 64.7s of flight had none at all. The dashboard froze there and
        # Görev 3's centering read a feed with no producer (0 committed
        # detections in 200 attempts).
        #
        # They now live in core/detection/vision_runtime.py, started before
        # the master FSM and stopped after it, so every phase consumes the
        # same single-owner feed. This orchestrator is now a pure CONSUMER
        # of detection_feed, exactly like CenteringController and
        # PayloadReleaseService.
        # `vision_runtime` is optional and is never CREATED here -- this
        # class does not own the pipeline. A caller that wants the pipeline
        # scoped to Görev 2 (a test exercising only this orchestrator) hands
        # one in and it is started/stopped around this run; the production
        # composition root deliberately does NOT, because it owns a runtime
        # spanning the whole mission. Either way there is exactly one
        # detect() caller, which is the invariant that matters.
        if self.vision_runtime is not None:
            self.vision_runtime.start()
        try:
            await self._run_inner()
        except Exception as e:
            self.context.transition_to(MissionPhase.MISSION_FAILED, reason=str(e))
            self._publish("MISSION_FAILED", str(e), severity=Severity.CRITICAL)
            raise
        finally:
            if self.vision_runtime is not None:
                await self.vision_runtime.stop()

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
        self.context.set_blocking("STARTING_UPLOADED_MISSION", "MavsdkBackendBase",
                                   BlockingKind.BLOCKING_WAIT,
                                   timeout_s=_params.MISSION_START_HOLD_S + MISSION_MODE_CONFIRM_TIMEOUT_S)
        await self._start_uploaded_mission()
        self.context.clear_blocking()

        self.context.transition_to(MissionPhase.SEARCHING)
        await self._search_and_engage_loop()

    # MAVLink command ids used for route validation (MAV_CMD).
    _CMD_NAV_WAYPOINT = 16
    _CMD_NAV_RETURN_TO_LAUNCH = 20
    _CMD_NAV_LAND = 21
    _CMD_NAV_TAKEOFF = 22
    _CMD_NAMES = {16: "NAV_WAYPOINT", 20: "NAV_RETURN_TO_LAUNCH",
                  21: "NAV_LAND", 22: "NAV_TAKEOFF"}

    def _validate_route_and_start_index(self, items: list) -> int:
        """ADR-007 route validation. Returns the index to start from.

        Accepts only: count >= 2; every item NAV_WAYPOINT, except that a
        NAV_TAKEOFF is permitted at seq 0 only; no NAV_LAND and no
        NAV_RETURN_TO_LAUNCH anywhere. A NAV_LAND/RTL in the route would
        make PX4 fly the route to a landing while the search phase still
        expects to be airborne for the Offboard payload phase, and the
        search loop treats an early is_mission_finished() as
        MISSION_FAILED -- so such a route is refused before arming rather
        than discovered mid-flight. The route itself is never modified.
        """
        if len(items) < 2:
            raise RuntimeError(
                f"MISSION_ROUTE_INVALID: route has {len(items)} item(s), at least 2 waypoints "
                f"are required. Plan the search route in QGroundControl and upload it."
            )

        for it in items:
            name = self._CMD_NAMES.get(it.command, f"cmd_{it.command}")
            if it.command in (self._CMD_NAV_LAND, self._CMD_NAV_RETURN_TO_LAUNCH):
                raise RuntimeError(
                    f"MISSION_ROUTE_INVALID: seq={it.seq} is {name}. The search route must not "
                    f"contain a landing or return-to-launch item -- PX4 would fly the vehicle "
                    f"into it while the payload phase still needs to be airborne. Remove it in "
                    f"QGroundControl (waypoints only) and re-upload."
                )
            if it.command == self._CMD_NAV_TAKEOFF and it.seq != 0:
                raise RuntimeError(
                    f"MISSION_ROUTE_INVALID: seq={it.seq} is NAV_TAKEOFF; a takeoff item is only "
                    f"allowed at seq 0."
                )
            if it.command not in (self._CMD_NAV_WAYPOINT, self._CMD_NAV_TAKEOFF):
                raise RuntimeError(
                    f"MISSION_ROUTE_INVALID: seq={it.seq} is {name}; only NAV_WAYPOINT items "
                    f"(plus an optional NAV_TAKEOFF at seq 0) are supported."
                )

        # This system performs its own takeoff before starting the route, so
        # a NAV_TAKEOFF at seq 0 must be skipped or PX4 re-runs it against an
        # already-airborne vehicle.
        return 1 if items[0].command == self._CMD_NAV_TAKEOFF else 0

    async def _start_uploaded_mission(self) -> None:
        """ADR-007 (supersedes the 2026-08-13 "mission_gz supervisor model"):
        this system now starts the uploaded route itself, after its own
        arm/takeoff, instead of blocking until an operator pressed Start
        Mission in QGroundControl. The superseded rule was:

            "this system must NEVER issue the MAVLink command that starts
             Mission mode -- starting the mission, exactly like uploading
             its route, is exclusively the operator's action in
             QGroundControl."

        Route DEFINITION remains the operator's job in QGroundControl; only
        the start command moved. Sequence: validate route -> settle
        MISSION_START_HOLD_S -> point PX4 at the first waypoint -> start ->
        confirm Mission mode within MISSION_MODE_CONFIRM_TIMEOUT_S.
        """
        items = await self.flight.get_raw_mission_items()
        start_index = self._validate_route_and_start_index(items)
        summary = ", ".join(
            f"{it.seq}:{self._CMD_NAMES.get(it.command, it.command)}" for it in items)
        logger.info(f"[MISSION_START] Rota dogrulandi ({len(items)} item): {summary}")
        logger.info(f"[MISSION_START] Baslangic indeksi: {start_index}")

        # Settle in HOLD before Mission mode engages. The flight heartbeat is
        # published throughout, otherwise HealthMonitor ages MavsdkBackendBase
        # into STALE during the hold (get_global_position() is the heartbeat).
        logger.info(f"[MISSION_START] {_params.MISSION_START_HOLD_S:.1f}s stabilizasyon bekleniyor...")
        hold_deadline = time.monotonic() + _params.MISSION_START_HOLD_S
        while time.monotonic() < hold_deadline:
            await self.flight.get_global_position()  # publishes VEHICLE_TELEMETRY heartbeat
            await asyncio.sleep(0.5)

        if start_index != 0:
            await self.flight.set_current_mission_item(start_index)
            logger.info(f"[MISSION_START] PX4 mevcut gorev ogesi {start_index} olarak ayarlandi "
                        f"(seq0 NAV_TAKEOFF atlandi -- kalkis zaten yapildi).")

        logger.info("[MISSION_START] Yuklu gorev baslatiliyor...")
        await self.flight.start_mission()

        deadline = time.monotonic() + MISSION_MODE_CONFIRM_TIMEOUT_S
        mode = None
        while time.monotonic() < deadline:
            await self.flight.get_global_position()  # keep the heartbeat alive
            mode = await self.flight.get_flight_mode()
            if mode == "MISSION":
                logger.info("[MISSION_START] PX4 MISSION moduna gecti -- arama fazi basliyor.")
                return
            await asyncio.sleep(0.5)

        raise RuntimeError(
            f"MISSION_MODE_NOT_ENTERED: PX4 did not enter MISSION mode within "
            f"{MISSION_MODE_CONFIRM_TIMEOUT_S:.0f}s of start_mission(); last observed mode "
            f"was {mode!r}. Check QGroundControl for a rejection message."
        )

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
        # ADR-008 B1: nothing to release here anymore. _detection_loop never
        # stops, so returning to plain searching needs no detector handover
        # -- which also removes the trap where this method being permanently
        # guarded off (_search_complete) left the detector reserved, and
        # vision DOWN, for the entire payload phase.
        try:
            await self.flight.stop_offboard()
        except Exception as e:  # noqa: BLE001 -- PX4 may reject this if Offboard was never actually confirmed active; not fatal
            logger.warning(f"stop_offboard sirasinda hata (yoksayiliyor): {e}")
        # ADR-009 PX4-STABILITY (measured, 2026-08-17). Two things were
        # wrong here, and together they are the best explanation for PX4
        # dying three times in one day:
        #
        #  1. NO SPACING. Resumes went out as fast as pursuits failed. The
        #     navigator's own "Executing Mission" lines show 14.39s minimum
        #     spacing in the session that survived, 5.04s in the one where
        #     the 4th resume was silently ignored, and 0.11s in the one that
        #     wedged PX4 outright.
        #  2. NO CONFIRMATION. _start_uploaded_mission() already polls until
        #     PX4 reports MISSION before proceeding -- this path fired
        #     start_mission() and published MISSION_ROUTE_RESUMED
        #     immediately, trusting it. On 2026-08-17 the 4th resume
        #     produced no "Executing Mission" at all: PX4 accepted the
        #     command without error and stayed in HOLD, the route froze at
        #     3/4, is_mission_finished() could never become True, and the
        #     search loop span until the watchdog. Nothing reported it,
        #     because nothing looked.
        await self._space_out_resume()
        await self._issue_resume()

        if await self._confirm_mission_mode():
            self._last_resume_at = time.time()
            self._publish("MISSION_ROUTE_RESUMED")
            return

        # One retry, then report loudly. Deliberately NOT an abort: a stuck
        # Mission is already covered by MISSION_TIMEOUT, and escalating here
        # would change failure behaviour beyond this helper.
        logger.warning("Mission resume onaylanmadi -- tekrarlaniyor.")
        await self._issue_resume()
        self._last_resume_at = time.time()
        if await self._confirm_mission_mode():
            self._publish("MISSION_ROUTE_RESUMED", data={"retried": True})
            return

        logger.error("MISSION_RESUME_NOT_CONFIRMED: PX4 iki denemede de MISSION moduna donmedi -- "
                     "arac muhtemelen HOLD'da kalacak, rota ilerlemeyecek.")
        self._publish("MISSION_RESUME_NOT_CONFIRMED",
                      "PX4 did not report MISSION after start_mission() (2 attempts)",
                      severity=Severity.CRITICAL, category=Category.WATCHDOG)

    async def _issue_resume(self) -> None:
        """ADR-010 R2: point PX4 explicitly at the route index it is
        actually on, then start.

        A bare start_mission() relies on PX4's implicit resume state, and on
        2026-08-17 PX4 accepted it without error and silently stayed in
        HOLD -- the navigator never printed "Executing Mission", the route
        froze at 3/4 and is_mission_finished() could never become True.
        set_current_mission_item() makes the intent unambiguous. Failure to
        set the index is not fatal on its own: start_mission() is still
        issued and the MISSION-mode confirm below is what actually decides
        whether the resume worked."""
        try:
            index = await self.flight.get_current_mission_index()
            await self.flight.set_current_mission_item(index)
            logger.info(f"[MISSION_RESUME] rota {index}. ogeden devam ettiriliyor.")
        except Exception as e:  # noqa: BLE001 -- confirm step is the real gate
            logger.warning(f"[MISSION_RESUME] set_current_mission_item basarisiz ({e}) -- "
                           "yalnizca start_mission() ile denenecek.")
        await self.flight.start_mission()

    async def _space_out_resume(self) -> None:
        """Hold off until MISSION_RESUME_MIN_INTERVAL_S has passed since the
        last resume. Costs search time; cheaper than a wedged autopilot."""
        if self._last_resume_at <= 0.0:
            return
        wait_s = MISSION_RESUME_MIN_INTERVAL_S - (time.time() - self._last_resume_at)
        if wait_s <= 0:
            return
        logger.info(f"[MISSION_RESUME] PX4'e nefes payi: {wait_s:.1f}s bekleniyor "
                    f"(min aralik {MISSION_RESUME_MIN_INTERVAL_S:.0f}s).")
        self._publish("MISSION_RESUME_THROTTLED", f"waiting {wait_s:.1f}s",
                      severity=Severity.DEBUG, category=Category.NAVIGATION,
                      data={"wait_s": round(wait_s, 2),
                            "min_interval_s": MISSION_RESUME_MIN_INTERVAL_S})
        await asyncio.sleep(wait_s)

    async def _confirm_mission_mode(self) -> bool:
        """Poll until PX4 actually reports MISSION -- the same check
        _start_uploaded_mission() has always done for the FIRST start, now
        applied to every subsequent one."""
        deadline = time.monotonic() + MISSION_MODE_CONFIRM_TIMEOUT_S
        while time.monotonic() < deadline:
            try:
                if await self.flight.get_flight_mode() == "MISSION":
                    return True
            except Exception as e:  # noqa: BLE001 -- incl. TelemetryStale; a dead link is not a mode answer
                logger.warning(f"[MISSION_RESUME] ucus modu okunamadi: {e}")
                return False
            await asyncio.sleep(0.5)
        return False

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
            # scheduling-order-dependent instead of deterministic. The
            # detection loop already publishes at the same ~10Hz ceiling
            # this loop was polling at, so reading its result introduces no
            # meaningful staleness.
            #
            # ADR-010 P3: reads the FEED rather than a plain
            # self._latest_detections attribute. Same data, but freshness is
            # now part of it -- if the producer (VisionRuntime, which owns
            # the mission-lifetime loop) goes quiet, this correctly sees no
            # detections instead of the last good list forever.
            detections = self.detection_feed.detections()

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
                # ADR-009 D3: a shape that has exhausted its centering
                # attempts, or is still cooling down from the last one, is
                # not a candidate. Detection continues normally -- it is
                # only the PURSUIT that backs off.
                and d.shape_type not in self._centering_abandoned
                and now >= self._centering_cooldown_until.get(d.shape_type, 0.0)
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
                await self._settle_after_resume()
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
                self.context.transition_to(MissionPhase.GOTO_TARGET_CENTERING, reason=selected.shape_type)
                # ADR-008 B1: the REAL budget, asked of the controller that
                # will spend it, instead of the fictional
                # CENTERING_CONVERGENCE_TIMEOUT_S (5.0s) this used to report
                # while the loop actually ran 77s and 82s on 2026-08-16.
                centering_budget_s = self.centering.budget_s(current_alt, MISSION_ALTITUDE_M)
                self.context.set_blocking("WAITING_CENTERING_CONVERGENCE", "CenteringController",
                                           BlockingKind.BLOCKING_WAIT, timeout_s=centering_budget_s)
                converged = await self._center_with_retries(selected.shape_type, current_alt)
                self.context.clear_blocking()

                if not converged:
                    # Cap reached: _center_with_retries already published
                    # TARGET_SEEN_BUT_NOT_CENTERED and abandoned the shape.
                    self.context.transition_to(MissionPhase.SEARCHING, reason="centering_attempts_exhausted")
                    self.validator.set_navigating_to(selected.shape_type, False)
                    await self._resume_mission_route()
                    continue

                await self._measure_post_lock_drift(selected.shape_type, current_alt)

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

    async def _measure_post_lock_drift(self, shape_type: str, alt_m: float) -> None:
        """Operator request (2026-08-17): how far the target slides in the
        frame over POST_LOCK_DRIFT_WINDOW_S after convergence.

        Convergence is a single-frame test -- it proves the vehicle was
        inside tolerance at one instant, not that it stays there. This
        samples the same detection feed the controller used, while
        streaming zero-velocity setpoints (so Offboard survives the
        window), and reports peak and final drift. Read-only: it commands
        nothing but a hold, and does not gate the GPS save."""
        intrinsics = default_camera_intrinsics()
        res_w, res_h = self.camera.get_resolution()
        cx, cy = res_w / 2.0, res_h / 2.0
        px_to_cm = None
        if intrinsics is not None and alt_m and alt_m > 0:
            px_to_cm = alt_m * 100.0 / intrinsics.scaled_to(res_w, res_h).focal_px

        samples, lost = [], 0
        deadline = time.monotonic() + POST_LOCK_DRIFT_WINDOW_S
        while time.monotonic() < deadline:
            target = self.detection_feed.get(shape_type)
            if target is None:
                lost += 1
            else:
                samples.append((target.center_px[0] - cx, target.center_px[1] - cy))
            await self.flight.set_velocity_body(0.0, 0.0, 0.0, 0.0)
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)

        if not samples:
            logger.warning(f"[POST_LOCK_DRIFT] {shape_type}: pencerede hic tespit yok "
                           f"({lost} kayip ornek).")
            self._publish("POST_LOCK_DRIFT", shape_type, severity=Severity.WARN,
                          category=Category.VISION,
                          data={"shape_type": shape_type, "samples": 0, "lost": lost,
                                "window_s": POST_LOCK_DRIFT_WINDOW_S})
            return

        first = samples[0]
        drifts = [((dx - first[0]) ** 2 + (dy - first[1]) ** 2) ** 0.5 for dx, dy in samples]
        peak_px, final_px = max(drifts), drifts[-1]
        to_cm = (lambda v: round(v * px_to_cm, 2)) if px_to_cm else (lambda v: None)
        logger.info(f"[POST_LOCK_DRIFT] {shape_type} {POST_LOCK_DRIFT_WINDOW_S:.0f}s: "
                    f"peak={peak_px:.1f}px final={final_px:.1f}px "
                    f"({to_cm(peak_px)}cm / {to_cm(final_px)}cm), {len(samples)} ornek, {lost} kayip")
        self._publish("POST_LOCK_DRIFT", shape_type, category=Category.VISION,
                      data={"shape_type": shape_type, "window_s": POST_LOCK_DRIFT_WINDOW_S,
                            "samples": len(samples), "lost": lost,
                            "peak_drift_px": round(peak_px, 2), "final_drift_px": round(final_px, 2),
                            "peak_drift_cm": to_cm(peak_px), "final_drift_cm": to_cm(final_px),
                            "start_offset_px": [round(first[0], 1), round(first[1], 1)]})

    async def _center_with_retries(self, shape_type: str, current_alt: float) -> bool:
        """ADR-010 R1: retry the SAME target WITHOUT giving up Offboard and
        WITHOUT flying the route in between.

        Measured on V1'' (2026-08-17): attempt 1 on MAVI_ALTIGEN saw the
        target in 150/150 frames and closed to |ey|=0.0271 at 0.5m ground
        distance -- 2.7x tolerance, needing only ~5.3s more at its own
        measured tau of 5.3s. But the old design then resumed the route,
        and 68.3 SECONDS passed before the next attempt; attempts 2 and 3
        never saw the target in a single frame out of 150. The vehicle had
        simply flown away from it. The target was never recorded, so the
        payload phase never ran.

        Holding position between attempts keeps the vehicle where the
        target is and keeps Offboard alive (hold_position streams
        zero-velocity setpoints, so PX4 never times out). The detection
        loop runs throughout -- it always has since ADR-008 B1 -- so the
        next attempt starts from a live, current detection."""
        for attempt in range(1, CENTERING_MAX_ATTEMPTS_PER_TARGET + 1):
            self._centering_attempts[shape_type] = attempt
            if await self.centering.go_to_and_center(shape_type):
                return True

            if attempt >= CENTERING_MAX_ATTEMPTS_PER_TARGET:
                self._abandon_target(shape_type, attempt)
                return False

            await self._hold_and_retry(shape_type, attempt, current_alt)
        return False

    async def _hold_and_retry(self, shape_type: str, attempt: int, current_alt: float) -> None:
        """Hold over the target between attempts and report where we are."""
        target = self.detection_feed.get(shape_type)
        distance_m = None
        if target is not None:
            res_w, res_h = self.camera.get_resolution()
            distance_m = self.centering._ground_distance_m(
                target.center_px[0] - res_w / 2.0, target.center_px[1] - res_h / 2.0,
                current_alt, res_w, res_h)

        seen = "hedef goruniyor" if target is not None else "hedef su an karede yok"
        dist_note = f", d={distance_m:.2f}m" if distance_m is not None else ""
        logger.warning(f"[RETRY_IN_PLACE] {shape_type} {attempt}/{CENTERING_MAX_ATTEMPTS_PER_TARGET} "
                       f"merkezlenemedi -- {CENTERING_RETRY_COOLDOWN_S:.0f}s yerinde beklenip "
                       f"tekrar denenecek ({seen}{dist_note}).")
        self._publish("RETRY_IN_PLACE", f"{shape_type} {attempt}/{CENTERING_MAX_ATTEMPTS_PER_TARGET}",
                      severity=Severity.WARN, category=Category.NAVIGATION,
                      data={"shape_type": shape_type, "attempt": attempt,
                            "max_attempts": CENTERING_MAX_ATTEMPTS_PER_TARGET,
                            "target_visible": target is not None,
                            "ground_distance_m": round(distance_m, 3) if distance_m is not None else None,
                            "hold_s": CENTERING_RETRY_COOLDOWN_S})
        # hold_position() streams zero-velocity setpoints for the whole
        # duration, so Offboard stays alive across the pause -- a bare
        # sleep here would drop us out of Offboard in ~500ms.
        await self.flight.hold_position(CENTERING_RETRY_COOLDOWN_S)

    def _abandon_target(self, shape_type: str, attempts: int) -> None:
        self._centering_abandoned.add(shape_type)
        logger.warning(
            f"TARGET SEEN BUT NOT CENTERED: {shape_type} {attempts} denemede merkezlenemedi -- "
            f"bu hedef icin pursuit birakildi, arama devam ediyor.")
        self._publish("TARGET_SEEN_BUT_NOT_CENTERED", shape_type, severity=Severity.CRITICAL,
                      category=Category.VISION,
                      data={"shape_type": shape_type, "attempts": attempts,
                            "max_attempts": CENTERING_MAX_ATTEMPTS_PER_TARGET})

    async def _settle_after_resume(self) -> None:
        """ADR-010 R2: PX4 refused OFFBOARD 4 times in V1'' when a pursuit
        asked to pause the Mission ~1s after a resume had started it. Give
        the autopilot OFFBOARD_AFTER_RESUME_SETTLE_S in MISSION first."""
        if self._last_resume_at <= 0.0:
            return
        wait_s = OFFBOARD_AFTER_RESUME_SETTLE_S - (time.time() - self._last_resume_at)
        if wait_s <= 0:
            return
        logger.info(f"[OFFBOARD] resume sonrasi {wait_s:.1f}s oturma payi bekleniyor.")
        await asyncio.sleep(wait_s)

    def _note_centering_failure(self, shape_type: str) -> None:
        """ADR-009 D3: record a failed pursuit, hold the target off for
        CENTERING_RETRY_COOLDOWN_S, and give up on it entirely after
        CENTERING_MAX_ATTEMPTS_PER_TARGET.

        Giving up is deliberately loud but NOT fatal: "I can see it and I
        cannot centre on it" is a genuinely different outcome from "I never
        saw it", and the operator needs to be able to tell them apart in the
        log afterwards. The search carries on for the other shape."""
        # BUG FIX (V1', 2026-08-17): this used to take the caller's `now`,
        # which _search_and_engage_loop captures at the TOP of its while
        # iteration -- i.e. before the up-to-15s centering attempt that just
        # failed. The cooldown was therefore computed from a timestamp ~15s
        # in the past and had already expired by the time it was set, so the
        # retry fired 0.7s after the failure instead of 10s. Observed live:
        # "KIRMIZI_UCGEN merkezlenemedi (1/3)" at 13:19:57.145 followed by a
        # fresh pursuit at 13:19:57.814. The attempt CAP still bounded it,
        # so it never became a storm -- but the spacing that keeps the route
        # advancing between pursuits was simply not there.
        now = time.time()
        attempts = self._centering_attempts.get(shape_type, 0) + 1
        self._centering_attempts[shape_type] = attempts
        self._centering_cooldown_until[shape_type] = now + CENTERING_RETRY_COOLDOWN_S

        if attempts >= CENTERING_MAX_ATTEMPTS_PER_TARGET:
            self._centering_abandoned.add(shape_type)
            logger.warning(
                f"TARGET SEEN BUT NOT CENTERED: {shape_type} {attempts} denemede merkezlenemedi -- "
                f"bu hedef icin pursuit birakildi, arama devam ediyor.")
            self._publish("TARGET_SEEN_BUT_NOT_CENTERED", shape_type, severity=Severity.CRITICAL,
                          category=Category.VISION,
                          data={"shape_type": shape_type, "attempts": attempts,
                                "max_attempts": CENTERING_MAX_ATTEMPTS_PER_TARGET})
        else:
            logger.warning(
                f"{shape_type} merkezlenemedi ({attempts}/{CENTERING_MAX_ATTEMPTS_PER_TARGET}) -- "
                f"{CENTERING_RETRY_COOLDOWN_S:.0f}s bekleyip rotaya devam ediliyor.")
            self._publish("CENTERING_RETRY_SCHEDULED", shape_type, severity=Severity.WARN,
                          category=Category.NAVIGATION,
                          data={"shape_type": shape_type, "attempts": attempts,
                                "max_attempts": CENTERING_MAX_ATTEMPTS_PER_TARGET,
                                "cooldown_s": CENTERING_RETRY_COOLDOWN_S})

    def _publish_payload_sync(self) -> None:
        self._publish("PAYLOAD_STATE_SYNC", category=Category.PAYLOAD, data={
            "payload_1_released": self.interlock.payload_1_released,
            "payload_2_released": self.interlock.payload_2_released,
        })
