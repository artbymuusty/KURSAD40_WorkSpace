"""
ADR-004 §10 (Health Model). Health is computed purely from event timestamps
and declared cadences -- subsystems don't self-report "I am healthy," they
just keep publishing on schedule. Going silent IS the signal, so a frozen
detector (e.g. YoloDetector never loading its model) becomes visible the
same way a crashed one would, instead of looking identical to "no targets
in view" (the exact blind spot ADR-004 §8/§9.2 identified).
"""
import logging
import time
from dataclasses import dataclass
from threading import RLock
from typing import Callable, Dict, Optional

from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger("telemetry.health")

HEALTHY, DEGRADED, STALE, DOWN, UNKNOWN = "HEALTHY", "DEGRADED", "STALE", "DOWN", "UNKNOWN"


@dataclass
class SubsystemCadence:
    subsystem: str
    expected_interval_s: float
    grace_multiplier: float = 3.0  # STALE at 1x-3x interval, DOWN beyond

    @property
    def down_after_s(self) -> float:
        return self.expected_interval_s * self.grace_multiplier


class HealthMonitor:
    """Subscribe `on_event` to EventBus (any event from a registered
    subsystem counts as a heartbeat for it) and call `check()` periodically
    (e.g. from the same loop that services the WatchdogEngine) to recompute
    states and publish HEALTH_STATE_CHANGED transitions."""

    def __init__(self, publisher: EventPublisher = NULL_PUBLISHER):
        self.publisher = publisher
        self._lock = RLock()
        self._cadences: Dict[str, SubsystemCadence] = {}
        self._last_seen: Dict[str, float] = {}
        self._last_state: Dict[str, str] = {}
        # Dependency propagation (ADR-004 §10): if `parent` is DOWN, every
        # subsystem in its dependents set is forced DOWN regardless of its
        # own last-heartbeat.
        self._dependents: Dict[str, set] = {}

    def register(self, subsystem: str, expected_interval_s: float, grace_multiplier: float = 3.0) -> None:
        with self._lock:
            self._cadences[subsystem] = SubsystemCadence(subsystem, expected_interval_s, grace_multiplier)
            self._last_state.setdefault(subsystem, UNKNOWN)

    def set_dependency(self, subsystem: str, depends_on: str) -> None:
        """`subsystem`'s health is forced DOWN whenever `depends_on` is DOWN."""
        with self._lock:
            self._dependents.setdefault(depends_on, set()).add(subsystem)

    def on_event(self, event: Event) -> None:
        if event.subsystem in self._cadences:
            with self._lock:
                self._last_seen[event.subsystem] = event.ts

    def _compute(self, subsystem: str, now: float) -> str:
        cadence = self._cadences[subsystem]
        last = self._last_seen.get(subsystem)
        if last is None:
            return UNKNOWN
        age = now - last
        if age <= cadence.expected_interval_s:
            return HEALTHY
        if age <= cadence.down_after_s:
            return STALE if age > cadence.expected_interval_s * 1.5 else DEGRADED
        return DOWN

    def check(self, now: Optional[float] = None) -> Dict[str, str]:
        now = now or time.time()
        results: Dict[str, str] = {}
        with self._lock:
            for subsystem in self._cadences:
                results[subsystem] = self._compute(subsystem, now)
            # Dependency propagation pass.
            for parent, children in self._dependents.items():
                if results.get(parent) == DOWN:
                    for child in children:
                        if child in results:
                            results[child] = DOWN
            changed = {s: st for s, st in results.items() if self._last_state.get(s) != st}
            self._last_state.update(results)

        for subsystem, state in changed.items():
            severity = Severity.CRITICAL if state == DOWN else (Severity.WARN if state in (DEGRADED, STALE) else Severity.INFO)
            self.publisher.publish(Event(
                code="HEALTH_STATE_CHANGED",
                subsystem=subsystem,
                category=Category.HEALTH,
                severity=severity,
                message=f"{subsystem} -> {state}",
                data={"state": state, "last_seen": self._last_seen.get(subsystem)},
            ))
        return results
