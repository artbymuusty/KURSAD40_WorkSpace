"""
ADR-004 §13 (Mission Operations Center Architecture): composition root.
Wires EventBus + MissionContext + RuntimeStateAggregator + HealthMonitor +
WatchdogEngine + EventStore + MissionOpsDashboard into one handle that
main_gz.py / main_real.py / main_dual.py construct once and inject into
the orchestrators.

Nothing in this module touches flight/vision/payload objects -- it is pure
infrastructure, matching ADR-004 §3's "outbound-only edge": the mission
runtime publishes to `ops_center.bus`; nothing here calls back into it.
"""
import asyncio
import logging
import time
from dataclasses import dataclass
from typing import Callable, Optional

from core.config.parameters import (
    DASHBOARD_REFRESH_HZ,
    FLIGHT_TELEMETRY_HEARTBEAT_INTERVAL_S,
    GOREV2_MAX_FLIGHT_DURATION_S,
    HEALTH_GRACE_MULTIPLIER,
    QGC_CHECK_INTERVAL_S,
    QGC_UDP_PORT,
    VISION_HEARTBEAT_INTERVAL_S,
    WATCHDOG_CHECK_INTERVAL_S,
)
from core.mission.context import MissionContext
from core.mission.phase import MissionPhase
from core.telemetry.aggregator import RuntimeStateAggregator
from core.telemetry.dashboard import MissionOpsDashboard
from core.telemetry.event_bus import EventBus
from core.telemetry.event_store import EventStore
from core.telemetry.frame_channel import FrameChannel
from core.telemetry.health import HealthMonitor
from core.telemetry.qgc_monitor import QgcMonitor
from core.telemetry.watchdog import WatchdogEngine

logger = logging.getLogger("telemetry.ops_center")

# Subsystem names as published by their respective modules -- kept in one
# place so HealthMonitor registration and the actual `subsystem=` strings
# used in publish() calls can't silently drift apart.
FLIGHT_BACKEND = "MavsdkBackendBase"
VISION_PIPELINE = "Gorev2Orchestrator.vision"


@dataclass
class OpsCenter:
    mission_id: str
    bus: EventBus
    context: MissionContext
    aggregator: RuntimeStateAggregator
    health: HealthMonitor
    watchdog: WatchdogEngine
    event_store: EventStore
    frame_channel: FrameChannel
    qgc_monitor: QgcMonitor
    dashboard: MissionOpsDashboard
    _supervisor_task: "asyncio.Task | None" = None
    # ADR-008 B2 (A2 table row 6): set by the entrypoint once the
    # MasterMissionController exists (this object is built and started
    # BEFORE the mission runtime, by design -- ADR-004 §13). Called when
    # MISSION_TIMEOUT fires, so the mandatory 10-minute budget actually
    # acts. Optional: with no hook the watchdog degrades to its previous
    # report-only behaviour rather than failing.
    mission_timeout_hook: Optional[Callable[[str], None]] = None

    def start(self) -> None:
        """Auto-launch: the dashboard opens and background monitors start
        the instant this is called -- no operator action (ADR-004 §13)."""
        self.event_store.start()
        self.dashboard.start()
        self.watchdog.arm(
            "MISSION_TIMEOUT", "MasterMissionController", GOREV2_MAX_FLIGHT_DURATION_S,
            on_fire=self._on_mission_timeout,
        )
        self._supervisor_task = asyncio.ensure_future(self._supervisor_loop())
        logger.info("Mission Operations Center started (mission_id=%s).", self.mission_id)

    def _on_mission_timeout(self, _name: str) -> None:
        """ADR-008 B2: GOREV2_MAX_FLIGHT_DURATION_S is Şartname Bölüm 5.6's
        MANDATORY 10-minute limit, but firing this watchdog used to do
        nothing except relabel the phase -- the mission kept flying past its
        own budget with no abort, no landing, and no way for an operator to
        tell the difference. It now aborts the mission through the same
        return-to-start/finish-then-land path every other terminal route
        uses.

        Runs on the mission's own asyncio loop (the ops-center supervisor
        tick), so the hook can safely touch mission tasks."""
        reason = f"MISSION_TIMEOUT: exceeded {GOREV2_MAX_FLIGHT_DURATION_S:.0f}s budget"
        self.context.transition_to(MissionPhase.MISSION_TIMEOUT, reason=reason,
                                   subsystem="WatchdogEngine")
        if self.mission_timeout_hook is None:
            logger.error("MISSION_TIMEOUT fired but no abort hook is wired -- "
                         "vehicle will NOT be brought home automatically.")
            return
        self.mission_timeout_hook(reason)

    async def _supervisor_loop(self) -> None:
        """Owns the periodic health/watchdog/QGC tick. Runs on the mission's
        own asyncio loop (cheap, in-process checks only -- no I/O),
        independent of the dashboard's separate render thread."""
        last_qgc_check = 0.0
        try:
            while True:
                self.health.check()
                self.watchdog.check()
                now = time.time()
                if now - last_qgc_check >= QGC_CHECK_INTERVAL_S:
                    self.qgc_monitor.check()
                    last_qgc_check = now
                await asyncio.sleep(WATCHDOG_CHECK_INTERVAL_S)
        except asyncio.CancelledError:
            pass

    async def stop(self) -> None:
        if self._supervisor_task is not None:
            self._supervisor_task.cancel()
            try:
                await self._supervisor_task
            except asyncio.CancelledError:
                pass
        self.watchdog.disarm("MISSION_TIMEOUT")
        self.dashboard.stop()
        self.event_store.stop()
        logger.info("Mission Operations Center stopped (mission_id=%s).", self.mission_id)


def build_ops_center(mission_id: str, log_dir: str = "logs") -> OpsCenter:
    bus = EventBus(mission_id=mission_id)
    context = MissionContext(publisher=bus, mission_id=mission_id, timeout_budget_s=GOREV2_MAX_FLIGHT_DURATION_S)
    aggregator = RuntimeStateAggregator(mission_id=mission_id, timeout_budget_s=GOREV2_MAX_FLIGHT_DURATION_S)
    health = HealthMonitor(publisher=bus)
    watchdog = WatchdogEngine(publisher=bus)
    event_store = EventStore(mission_id=mission_id, log_dir=log_dir)
    frame_channel = FrameChannel()
    qgc_monitor = QgcMonitor(publisher=bus, udp_port=QGC_UDP_PORT)
    dashboard = MissionOpsDashboard(aggregator, frame_channel=frame_channel,
                                     mission_id=mission_id, refresh_hz=DASHBOARD_REFRESH_HZ)

    bus.subscribe(aggregator.on_event)
    bus.subscribe(health.on_event)
    bus.subscribe(event_store.on_event)

    health.register(FLIGHT_BACKEND, FLIGHT_TELEMETRY_HEARTBEAT_INTERVAL_S, HEALTH_GRACE_MULTIPLIER)
    health.register(VISION_PIPELINE, VISION_HEARTBEAT_INTERVAL_S, HEALTH_GRACE_MULTIPLIER)
    # Vision depends on the flight backend only insofar as the mission can't
    # proceed without it -- not modeled as a hard dependency here since the
    # vision pipeline is a genuinely independent process per the Görev 2
    # architecture mandate (GCS-side, MAVLink-only coupling to the drone).

    return OpsCenter(
        mission_id=mission_id, bus=bus, context=context, aggregator=aggregator,
        health=health, watchdog=watchdog, event_store=event_store,
        frame_channel=frame_channel, qgc_monitor=qgc_monitor, dashboard=dashboard,
    )
