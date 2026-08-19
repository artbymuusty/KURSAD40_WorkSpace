import asyncio
import logging
from core.mission.gorev2_orchestrator import Gorev2Orchestrator
from core.mission.gorev3_orchestrator import Gorev3Orchestrator
from core.mission.blocking import BlockingKind
from core.mission.context import MissionContext
from core.mission.phase import MissionPhase
from core.config.parameters import (
    ABORT_RETURN_DEADLINE_S,
    TELEMETRY_RECOVERY_WAIT_S,
    GLOBAL_POSITION_NAV_TIMEOUT_S,
    MISSION_ALTITUDE_M,
    RETURN_TO_START_FINISH_HOLD_S,
)
from core.interfaces.i_flight_backend import TelemetryStale
from core.navigation.geo import haversine_distance_m
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger(__name__)

class MasterMissionController:
    """
    ADR-005 §7: previously never instantiated by any of the three live
    entrypoints -- each called `asyncio.run(Gorev2Orchestrator.run())`
    directly, so Görev 3 and any explicit MISSION_COMPLETE transition were
    unreachable regardless of dashboard wiring. main_gz.py/main_real.py/
    main_dual.py now construct and run this instead (ADR-005 §12 Phase 4).

    ADR-008 B2: this class also owns "wherever the mission ends, the vehicle
    comes home first". Before it, exactly one of the eight terminal paths
    flew back to the recorded start/finish checkpoint (Görev 3 completing
    successfully). Every failure path landed in place -- proven on the
    2026-08-16 run, which landed 39.8m+ north of its checkpoint after an
    incomplete search -- and two paths (Ctrl-C/window-close, and the
    mandatory 10-minute MISSION_TIMEOUT watchdog) did not land at all.
    """
    def __init__(self, gorev2: Gorev2Orchestrator, gorev3: Gorev3Orchestrator,
                 context: MissionContext = None, publisher: EventPublisher = NULL_PUBLISHER):
        self.gorev2 = gorev2
        self.gorev3 = gorev3
        self.context = context or MissionContext(publisher=publisher)
        self.publisher = publisher
        self._mission_task: "asyncio.Task | None" = None
        self._abort_reason: str = ""

    def _publish(self, code, message="", severity=Severity.INFO, data=None):
        self.publisher.publish(Event(
            code=code, subsystem="MasterMissionController", category=Category.LIFECYCLE,
            severity=severity, message=message, data=data or {},
        ))

    # ------------------------------------------------------------------
    # Abort entry points (ADR-008 B2, A2 table rows 6 and 7)
    # ------------------------------------------------------------------
    def request_abort(self, reason: str) -> None:
        """Cancel the running mission so run()'s CancelledError handler can
        bring the vehicle home and land it.

        Safe to call from the ops-center supervisor loop (MISSION_TIMEOUT
        watchdog) -- it runs on this same asyncio loop. It is NOT the
        Ctrl-C path: that one cancels run() itself from the GUI thread via
        loop.call_soon_threadsafe, and lands in the same handler."""
        if self._abort_reason:
            return  # already aborting; do not stack cancellations
        self._abort_reason = reason
        logger.warning(f"MISSION ABORT istendi: {reason}")
        self._publish("MISSION_ABORT_REQUESTED", reason, severity=Severity.CRITICAL,
                      data={"reason": reason})
        if self._mission_task is not None and not self._mission_task.done():
            self._mission_task.cancel()

    async def run(self) -> None:
        """
        Görev 2 tamamlandıktan sonra Görev 3'e Offboard modundan çıkmadan doğrudan geçiş.
        """
        logger.info("MASTER MISSION CONTROLLER BASLATILDI")
        self._publish("MISSION_STARTED", f"mission_id={self.context.mission_id}")

        # ADR-008 B2: the mission body runs as its own task so BOTH abort
        # sources land in one place -- request_abort() cancels this task
        # directly, and a Ctrl-C/window-close cancel of run() propagates
        # into it through this await. asyncio.CancelledError derives from
        # BaseException, so the `except Exception` handlers below never saw
        # it: on 2026-08-16's Ctrl-C path _safe_land() was skipped entirely
        # and the vehicle was simply left airborne while the process exited.
        self._mission_task = asyncio.ensure_future(self._run_missions())
        try:
            await self._mission_task
        except asyncio.CancelledError:
            await self._abort_and_land()

    async def _run_missions(self) -> None:
        try:
            await self.gorev2.run()
        except asyncio.CancelledError:
            raise
        except Exception as e:
            # Gorev2Orchestrator already transitioned to MISSION_FAILED and
            # published MISSION_FAILED before re-raising -- this catch only
            # stops the exception from skipping the controlled landing below.
            logger.error(f"Gorev 2 basarisiz oldu, guvenli inis deneniyor: {e}")
            await self._safe_land(mission_already_failed=True, reason=f"gorev2_exception: {e}")
            return

        gorev2_failed = self.context.current_phase == MissionPhase.MISSION_FAILED
        if gorev2_failed:
            logger.warning("Gorev 2 tamamlanamadi (interlock kosullari saglanmadi) -- Gorev 3 atlanacak.")
            await self._safe_land(mission_already_failed=True, reason="gorev2_failed")
            return

        # KRİTİK: Burada flight.land() veya flight.stop_offboard() ÇAĞRILMAMALI --
        # Görev 2 Rapor Bölüm 15: 'Araç Offboard modundan çıkmadan doğrudan Görev 3'e geçer.'

        logger.info("Görev 2 akışı bitti, doğrudan Görev 3'e geçiliyor...")

        try:
            gorev3_success = await self.gorev3.run()
        except asyncio.CancelledError:
            raise
        except Exception as e:
            # BUG FIX (found during end-to-end verification): only Görev 2's
            # call was guarded before -- an exception out of Görev 3 (e.g.
            # GOREV3_TRANSIT_SPEED_M_S still being an unfilled TODO) used to
            # propagate straight out of run(), skipping _safe_land()
            # entirely and leaving the vehicle without a controlled landing
            # attempt. Every path through this method must reach _safe_land().
            logger.error(f"Gorev 3 basarisiz oldu, guvenli inis deneniyor: {e}")
            self.context.transition_to(MissionPhase.MISSION_FAILED, reason=f"gorev3_exception: {e}")
            self._publish("MISSION_FAILED", f"gorev3_exception: {e}", severity=Severity.CRITICAL)
            await self._safe_land(mission_already_failed=True, reason=f"gorev3_exception: {e}")
            return

        if gorev3_success:
            logger.info('TUM GOREVLER (2 + 3) BASARIYLA TAMAMLANDI')
        else:
            logger.warning('GOREV 2 TAMAMLANDI ANCAK GOREV 3 BASARISIZ - '
                           'yalnizca Gorev 2 puani gecerli olabilir')

        await self._safe_land(mission_already_failed=not gorev3_success,
                              reason="gorev3_complete" if gorev3_success else "gorev3_failed")

    async def _abort_and_land(self) -> None:
        """The MISSION_TIMEOUT-watchdog and Ctrl-C/window-close path.

        Bounded by ABORT_RETURN_DEADLINE_S, which must stay under
        main_gz.py's mission-thread join timeout -- an abort that outlived
        the process it is running in would be no better than the old
        behaviour of not landing at all."""
        reason = self._abort_reason or "cancelled"
        logger.warning(f"Gorev iptal edildi ({reason}) -- baslangic/bitis noktasina donulup inilecek.")
        self.context.transition_to(
            MissionPhase.MISSION_TIMEOUT if reason.startswith("MISSION_TIMEOUT") else MissionPhase.MISSION_ABORTED,
            reason=reason)
        try:
            await asyncio.wait_for(
                self._return_to_start_finish_and_land(reason=reason, mission_already_failed=True),
                timeout=ABORT_RETURN_DEADLINE_S)
        except asyncio.TimeoutError:
            logger.error(f"Abort inisi {ABORT_RETURN_DEADLINE_S:.0f}s icinde tamamlanamadi -- "
                         "dogrudan iniliyor.")
            self._publish("ABORT_RETURN_DEADLINE_EXCEEDED", severity=Severity.CRITICAL,
                          data={"deadline_s": ABORT_RETURN_DEADLINE_S, "reason": reason})
            await self._land(mission_already_failed=True, reason="abort_deadline_exceeded")
        except asyncio.CancelledError:
            # A second cancel arrived mid-return. Still make one direct
            # landing attempt -- never unwind leaving the vehicle airborne.
            logger.error("Abort sirasinda ikinci iptal geldi -- dogrudan iniliyor.")
            await self._land(mission_already_failed=True, reason="abort_recancelled")

    # ------------------------------------------------------------------
    # ADR-008 B2: the single return-to-start/finish-then-land path
    # ------------------------------------------------------------------
    async def _safe_land(self, mission_already_failed: bool = False, reason: str = "") -> None:
        """`mission_already_failed` is threaded through explicitly rather than
        inferred from `context.current_phase` afterward -- this method itself
        transitions through MissionPhase.LANDING, which would otherwise
        overwrite/erase whatever MISSION_FAILED state the caller had already
        recorded (a real bug caught during end-to-end verification: a Görev 3
        failure was being silently reported as MISSION_COMPLETE once landing
        itself happened to succeed).

        ADR-008 B2: this used to be `flight.land()` and nothing else, which
        descends wherever the vehicle happens to be. It now returns to the
        recorded start/finish checkpoint first, on every path that reaches
        it. The Görev-3-success path (A2 row 5) is unaffected in practice:
        Gorev3FinishPhase has already flown there, so the return converges
        immediately."""
        await self._return_to_start_finish_and_land(
            reason=reason or "mission_end", mission_already_failed=mission_already_failed)

    async def _return_to_start_finish_and_land(self, reason: str, mission_already_failed: bool) -> None:
        checkpoint = self.gorev2.checkpoint

        # Fallback: a failure before CHECKPOINT_SAVE (connect/arm/takeoff/
        # climb) means there is no recorded start/finish position at all.
        # Görev 2 Rapor Bölüm 4.2 makes checkpoint.get() raise in that case
        # rather than invent one, so land in place and say why.
        if not checkpoint.is_saved():
            logger.warning("Start/Finish checkpoint kaydedilmemis (15m'ye ulasilamadan hata) -- "
                           "bulunulan konumda inilecek.")
            self._publish("RETURN_TO_START_FINISH_SKIPPED",
                          "checkpoint never recorded -- landing in place",
                          severity=Severity.WARN,
                          data={"reason": reason, "cause": "checkpoint_not_saved"})
            await self._land(mission_already_failed=mission_already_failed, reason=reason)
            return

        lat, lon, _ = checkpoint.get()
        # ADR-009 D1: the return leg needs LIVE position/velocity -- it is a
        # closed-loop navigation, not a fire-and-forget command. If the link
        # is currently stale, give it a bounded chance to come back rather
        # than either flying on frozen numbers (the 2026-08-16 failure) or
        # abandoning the return over a transient hiccup.
        if not await self._await_fresh_telemetry():
            logger.error("Telemetri bayat kaldi -- baslangic/bitis noktasina donulemiyor, "
                         "bulunulan konumda inis denenecek.")
            self._publish("RETURN_TO_START_FINISH_SKIPPED",
                          "telemetry stale -- cannot navigate, landing in place",
                          severity=Severity.CRITICAL,
                          data={"reason": reason, "cause": "telemetry_stale"})
            await self._land(mission_already_failed=mission_already_failed, reason=reason)
            return

        try:
            await self._fly_to_start_finish(lat, lon, reason)
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001 -- a failed return must still land, never abort the landing
            logger.error(f"Baslangic/bitis noktasina donus basarisiz ({e}) -- bulunulan konumda iniliyor.")
            self._publish("RETURN_TO_START_FINISH_FAILED", str(e), severity=Severity.CRITICAL,
                          data={"reason": reason, "error": str(e)})

        await self._land(mission_already_failed=mission_already_failed, reason=reason)

    async def _await_fresh_telemetry(self) -> bool:
        """Poll until a telemetry read succeeds, bounded by
        TELEMETRY_RECOVERY_WAIT_S. Returns False if the link never
        recovers, which is the caller's signal to skip the return."""
        deadline = asyncio.get_event_loop().time() + TELEMETRY_RECOVERY_WAIT_S
        first = True
        while asyncio.get_event_loop().time() < deadline:
            try:
                await self.gorev2.flight.get_global_position()
                return True
            except TelemetryStale as e:
                if first:
                    first = False
                    logger.warning(f"Telemetri bayat ({e}) -- {TELEMETRY_RECOVERY_WAIT_S:.0f}s "
                                   "boyunca toparlanmasi bekleniyor.")
                    self._publish("TELEMETRY_STALE", str(e), severity=Severity.CRITICAL,
                                  data={"waiting_s": TELEMETRY_RECOVERY_WAIT_S})
            except Exception:  # noqa: BLE001 -- any other read failure is not a staleness question
                return True
            await asyncio.sleep(0.2)
        return False

    async def _fly_to_start_finish(self, lat: float, lon: float, reason: str) -> None:
        flight, centering = self.gorev2.flight, self.gorev2.centering

        cur_lat, cur_lon, _ = await flight.get_global_position()
        distance_m = haversine_distance_m(cur_lat, cur_lon, lat, lon)

        self.context.transition_to(MissionPhase.RETURN_TO_CHECKPOINT, reason=reason)
        self.context.set_blocking("RETURNING_TO_START_FINISH", "MasterMissionController",
                                   BlockingKind.BLOCKING_WAIT,
                                   timeout_s=GLOBAL_POSITION_NAV_TIMEOUT_S, cause=reason)
        logger.info(f"RETURNING_TO_START_FINISH (dist={distance_m:.1f} m) -- "
                    f"hedef ({lat:.7f}, {lon:.7f}) @ {MISSION_ALTITUDE_M}m, sebep: {reason}")
        self._publish("RETURNING_TO_START_FINISH", f"dist={distance_m:.1f} m",
                      data={"distance_m": distance_m, "target_lat": lat, "target_lon": lon,
                            "altitude_m": MISSION_ALTITUDE_M, "reason": reason})

        # goto_global_position_and_wait() streams Offboard setpoints, so
        # Offboard must actually be active. Depending on which terminal path
        # got here the vehicle may be in HOLD (a failed pursuit stopped
        # Offboard), MISSION (search still running when the abort landed) or
        # already OFFBOARD (Görev 3 success / mid-pursuit abort).
        if not await self._ensure_offboard():
            raise RuntimeError("OFFBOARD_UNAVAILABLE_FOR_RETURN: PX4 did not enter OFFBOARD")

        converged = await centering.goto_global_position_and_wait(lat, lon, MISSION_ALTITUDE_M)

        # Settle before descending, same reason hover_and_confirm() exists:
        # arriving and immediately landing bleeds residual lateral velocity
        # into the descent.
        await flight.hold_position(RETURN_TO_START_FINISH_HOLD_S)

        try:
            cur_lat, cur_lon, _ = await flight.get_global_position()
            final_distance_m = haversine_distance_m(cur_lat, cur_lon, lat, lon)
        except TelemetryStale:
            # Report the leg honestly rather than inventing a distance.
            final_distance_m = float("nan")
        self.context.clear_blocking()
        logger.info(f"{'Baslangic/bitis noktasina ulasildi' if converged else 'Donus zaman asimina ugradi'} "
                    f"-- checkpoint'e uzaklik {final_distance_m:.1f} m. Inis baslatiliyor.")
        self._publish("RETURN_TO_START_FINISH_ARRIVED" if converged else "RETURN_TO_START_FINISH_TIMED_OUT",
                      f"dist={final_distance_m:.1f} m",
                      severity=Severity.INFO if converged else Severity.WARN,
                      data={"distance_m": final_distance_m, "converged": converged,
                            "target_lat": lat, "target_lon": lon, "reason": reason})

    async def _ensure_offboard(self) -> bool:
        flight = self.gorev2.flight
        try:
            if await flight.get_flight_mode() == "OFFBOARD":
                return True
        except Exception as e:  # noqa: BLE001 -- fall through to an explicit switch attempt
            logger.warning(f"Ucus modu okunamadi ({e}) -- Offboard'a gecis denenecek.")
        return await self.gorev2.centering.switch_to_offboard()

    async def _land(self, mission_already_failed: bool, reason: str = "") -> None:
        logger.info("Son iniş gerçekleştiriliyor...")
        self.context.transition_to(MissionPhase.LANDING, reason=reason)
        try:
            await self.gorev2.flight.land()
        except Exception as e:  # noqa: BLE001 -- landing must be attempted regardless
            logger.error(f"Inis sirasinda hata: {e}")
            self.context.transition_to(MissionPhase.MISSION_FAILED, reason=f"land_failed: {e}")
            self._publish("MISSION_FAILED", f"land_failed: {e}", severity=Severity.CRITICAL)
            return

        if mission_already_failed:
            self.context.transition_to(MissionPhase.MISSION_FAILED, reason="landed_after_prior_failure")
        else:
            self.context.transition_to(MissionPhase.MISSION_COMPLETE)
            self._publish("MISSION_COMPLETE")
