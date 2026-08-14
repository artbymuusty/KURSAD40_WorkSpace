import logging
from core.mission.interlock import PayloadInterlock
from core.mission.gorev3_precondition import check_gorev3_precondition
from core.mission.gorev3_pickup import Gorev3PickupPhase
from core.mission.gorev3_transport import Gorev3TransportPhase
from core.mission.gorev3_redrop import Gorev3RedropPhase
from core.mission.gorev3_finish import Gorev3FinishPhase
from core.mission.context import MissionContext
from core.mission.phase import MissionPhase
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger(__name__)

class Gorev3Orchestrator:
    """
    ADR-004 §3: the dashboard is deliberately NOT injected here anymore (it
    was previously `dashboard: Dashboard = None` plus a `camera`-pulling
    `_update_hud` helper that called `dashboard.update(...)` directly --
    the exact synchronous, unisolated coupling ADR-005 §6 flagged as unsafe).
    This orchestrator now only ever publishes phase transitions/events;
    MissionOpsDashboard observes independently via RuntimeStateAggregator.
    """
    def __init__(self, interlock: PayloadInterlock, pickup_phase: Gorev3PickupPhase,
                 transport_phase: Gorev3TransportPhase, redrop_phase: Gorev3RedropPhase,
                 finish_phase: Gorev3FinishPhase, context: MissionContext = None,
                 publisher: EventPublisher = NULL_PUBLISHER):
        self.interlock = interlock
        self.pickup_phase = pickup_phase
        self.transport_phase = transport_phase
        self.redrop_phase = redrop_phase
        self.finish_phase = finish_phase
        self.context = context or MissionContext(publisher=publisher)
        self.publisher = publisher

    def _publish(self, code, message="", severity=Severity.INFO, data=None):
        self.publisher.publish(Event(
            code=code, subsystem="Gorev3Orchestrator", category=Category.LIFECYCLE,
            severity=severity, message=message, data=data or {},
        ))

    async def run(self) -> bool:
        if not check_gorev3_precondition(self.interlock):
            logger.error("GOREV 3 BASLATILAMADI: on kosul saglanmadi (payload_1 ve "
                         "payload_2 ikisi de birakilmis olmali)")
            self._publish("GOREV3_PRECONDITION_FAILED", severity=Severity.CRITICAL,
                          data={"payload_1_released": self.interlock.payload_1_released,
                                "payload_2_released": self.interlock.payload_2_released})
            return False

        self.context.transition_to(MissionPhase.GOREV3_START)
        self._publish("GOREV3_HOOK_INVOKED")

        self.context.transition_to(MissionPhase.GOREV3_RUNNING, reason="pickup")
        self._publish("GOREV3_PHASE_STARTED", "pickup", data={"phase": "pickup"})
        pickup_ok = await self.pickup_phase.run()
        if not pickup_ok:
            logger.error("GOREV 3 FAZ 1 (ALMA) BASARISIZ")
            self.context.transition_to(MissionPhase.MISSION_FAILED, reason="gorev3_pickup_failed")
            self._publish("GOREV3_PHASE_FAILED", "pickup", severity=Severity.CRITICAL, data={"phase": "pickup"})
            return False

        self.context.transition_to(MissionPhase.GOREV3_RUNNING, reason="transport")
        self._publish("GOREV3_PHASE_STARTED", "transport", data={"phase": "transport"})
        await self.transport_phase.run()

        self.context.transition_to(MissionPhase.GOREV3_RUNNING, reason="redrop")
        self._publish("GOREV3_PHASE_STARTED", "redrop", data={"phase": "redrop"})
        redrop_ok = await self.redrop_phase.run()
        if not redrop_ok:
            logger.error("GOREV 3 FAZ 3 (BIRAKMA) BASARISIZ")
            self.context.transition_to(MissionPhase.MISSION_FAILED, reason="gorev3_redrop_failed")
            self._publish("GOREV3_PHASE_FAILED", "redrop", severity=Severity.CRITICAL, data={"phase": "redrop"})
            return False

        self.context.transition_to(MissionPhase.RETURN_TO_CHECKPOINT, reason="gorev3_finish")
        self._publish("GOREV3_PHASE_STARTED", "finish", data={"phase": "finish"})
        await self.finish_phase.run()

        self._publish("GOREV3_COMPLETE")
        return True
