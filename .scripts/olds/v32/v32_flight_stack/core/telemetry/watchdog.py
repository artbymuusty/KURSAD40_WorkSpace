"""
ADR-004 §18 (Watchdog Architecture) / §8: "the current codebase has zero
instances of BLOCKING_WAIT actually enforcing a timeout." This is what
closes that gap -- including, concretely, finally making
GOREV2_MAX_FLIGHT_DURATION_S do something (core/config/parameters.py
defines it; before this module nothing ever read it).

Every armed watchdog is visible in MissionSnapshot.watchdogs (via
WATCHDOG_ARMED/UPDATED events feeding RuntimeStateAggregator) with its
remaining time, while armed -- not just after it fires.
"""
import logging
import time
from dataclasses import dataclass
from threading import RLock
from typing import Callable, Dict, Optional

from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger("telemetry.watchdog")

OnFire = Callable[[str], None]


@dataclass
class _Timer:
    name: str
    subsystem: str
    threshold_s: float
    deadline: float
    on_fire: Optional[OnFire] = None
    fired: bool = False


class WatchdogEngine:
    def __init__(self, publisher: EventPublisher = NULL_PUBLISHER):
        self.publisher = publisher
        self._lock = RLock()
        self._timers: Dict[str, _Timer] = {}

    def arm(self, name: str, subsystem: str, threshold_s: float, on_fire: Optional[OnFire] = None) -> None:
        now = time.time()
        with self._lock:
            self._timers[name] = _Timer(name, subsystem, threshold_s, now + threshold_s, on_fire)
        self.publisher.publish(Event(
            code="WATCHDOG_ARMED",
            subsystem=subsystem,
            category=Category.WATCHDOG,
            severity=Severity.DEBUG,
            message=f"{name} armed, threshold={threshold_s:.0f}s",
            data={"name": name, "threshold_s": threshold_s, "remaining_s": threshold_s},
        ))

    def feed(self, name: str, now: Optional[float] = None) -> None:
        """Reset a watchdog's deadline -- for watchdogs that must be kicked
        periodically (e.g. TELEMETRY_STALENESS) rather than fired once."""
        with self._lock:
            timer = self._timers.get(name)
            if timer is not None:
                timer.deadline = (now or time.time()) + timer.threshold_s
                timer.fired = False

    def disarm(self, name: str) -> None:
        with self._lock:
            timer = self._timers.pop(name, None)
        if timer is not None:
            self.publisher.publish(Event(
                code="WATCHDOG_DISARMED",
                subsystem=timer.subsystem,
                category=Category.WATCHDOG,
                severity=Severity.DEBUG,
                message=f"{name} disarmed",
                data={"name": name},
            ))

    def check(self, now: Optional[float] = None) -> None:
        """Call periodically (ops_center's supervisor loop, ~1Hz). Fires any
        overdue watchdog exactly once and publishes a live countdown for
        every still-armed one so the dashboard can show it ticking down."""
        now = now or time.time()
        to_fire = []
        with self._lock:
            for timer in self._timers.values():
                if timer.fired:
                    continue
                remaining = timer.deadline - now
                if remaining <= 0:
                    timer.fired = True
                    to_fire.append(timer)
                else:
                    self.publisher.publish(Event(
                        code="WATCHDOG_UPDATED",
                        subsystem=timer.subsystem,
                        category=Category.WATCHDOG,
                        severity=Severity.DEBUG,
                        message=f"{timer.name} remaining={remaining:.0f}s",
                        data={"name": timer.name, "remaining_s": remaining, "threshold_s": timer.threshold_s},
                    ))

        for timer in to_fire:
            self.publisher.publish(Event(
                code="WATCHDOG_FIRED",
                subsystem=timer.subsystem,
                category=Category.WATCHDOG,
                severity=Severity.CRITICAL,
                message=f"{timer.name} exceeded {timer.threshold_s:.0f}s",
                data={"name": timer.name, "threshold_s": timer.threshold_s},
            ))
            if timer.on_fire is not None:
                try:
                    timer.on_fire(timer.name)
                except Exception as e:  # noqa: BLE001 -- watchdog firing must never crash the checker
                    logger.error("Watchdog on_fire callback failed for %s: %s", timer.name, e)

    def is_armed(self, name: str) -> bool:
        with self._lock:
            return name in self._timers
