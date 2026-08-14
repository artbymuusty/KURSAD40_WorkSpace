import logging
from core.mission.gorev2_orchestrator import Gorev2Orchestrator
from core.mission.gorev3_orchestrator import Gorev3Orchestrator
from core.mission.context import MissionContext
from core.mission.phase import MissionPhase
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
    """
    def __init__(self, gorev2: Gorev2Orchestrator, gorev3: Gorev3Orchestrator,
                 context: MissionContext = None, publisher: EventPublisher = NULL_PUBLISHER):
        self.gorev2 = gorev2
        self.gorev3 = gorev3
        self.context = context or MissionContext(publisher=publisher)
        self.publisher = publisher

    def _publish(self, code, message="", severity=Severity.INFO, data=None):
        self.publisher.publish(Event(
            code=code, subsystem="MasterMissionController", category=Category.LIFECYCLE,
            severity=severity, message=message, data=data or {},
        ))

    async def run(self) -> None:
        """
        Görev 2 tamamlandıktan sonra Görev 3'e Offboard modundan çıkmadan doğrudan geçiş.
        """
        logger.info("MASTER MISSION CONTROLLER BASLATILDI")
        self._publish("MISSION_STARTED", f"mission_id={self.context.mission_id}")

        try:
            await self.gorev2.run()
        except Exception as e:
            # Gorev2Orchestrator already transitioned to MISSION_FAILED and
            # published MISSION_FAILED before re-raising -- this catch only
            # stops the exception from skipping the controlled landing below.
            logger.error(f"Gorev 2 basarisiz oldu, guvenli inis deneniyor: {e}")
            await self._safe_land(mission_already_failed=True)
            return

        gorev2_failed = self.context.current_phase == MissionPhase.MISSION_FAILED
        if gorev2_failed:
            logger.warning("Gorev 2 tamamlanamadi (interlock kosullari saglanmadi) -- Gorev 3 atlanacak.")
            await self._safe_land(mission_already_failed=True)
            return

        # KRİTİK: Burada flight.land() veya flight.stop_offboard() ÇAĞRILMAMALI --
        # Görev 2 Rapor Bölüm 15: 'Araç Offboard modundan çıkmadan doğrudan Görev 3'e geçer.'

        logger.info("Görev 2 akışı bitti, doğrudan Görev 3'e geçiliyor...")

        try:
            gorev3_success = await self.gorev3.run()
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
            await self._safe_land(mission_already_failed=True)
            return

        if gorev3_success:
            logger.info('TUM GOREVLER (2 + 3) BASARIYLA TAMAMLANDI')
        else:
            logger.warning('GOREV 2 TAMAMLANDI ANCAK GOREV 3 BASARISIZ - '
                           'yalnizca Gorev 2 puani gecerli olabilir')

        await self._safe_land(mission_already_failed=not gorev3_success)

    async def _safe_land(self, mission_already_failed: bool = False) -> None:
        """`mission_already_failed` is threaded through explicitly rather than
        inferred from `context.current_phase` afterward -- this method itself
        transitions through MissionPhase.LANDING, which would otherwise
        overwrite/erase whatever MISSION_FAILED state the caller had already
        recorded (a real bug caught during end-to-end verification: a Görev 3
        failure was being silently reported as MISSION_COMPLETE once landing
        itself happened to succeed)."""
        logger.info("Son iniş gerçekleştiriliyor...")
        self.context.transition_to(MissionPhase.LANDING)
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
