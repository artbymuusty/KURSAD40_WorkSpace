"""
ADR-004 §12 (Observability Architecture) / ADR-005 §8.1.

This restores V30/V31's mission_types.EventBus pattern almost verbatim
(subscribe/publish, per-subscriber try/except isolation) rather than
inventing a new mechanism -- ADR-005 identified that pattern as already
proven and simply not carried into v32_flight_stack.

publish() is synchronous and non-blocking-by-construction: every subscriber
callback must be fast (dict/attribute mutation) or must internally hand off
to its own worker thread (see EventStore). A subscriber that raises is
logged and skipped -- it can never affect the publisher (the mission
runtime) or other subscribers. This is what makes it safe to call directly
from Gorev2Orchestrator's/Gorev3Orchestrator's async loop without an
await and without risk of the mission coroutine being taken down by an
observability bug.
"""
import logging
from typing import Callable, List, Optional, Protocol

from core.telemetry.events import Event

logger = logging.getLogger("telemetry.event_bus")

Subscriber = Callable[[Event], None]


class EventPublisher(Protocol):
    """Narrow interface subsystems depend on -- they publish, they never
    subscribe or read back. Keeps subsystems decoupled from EventBus's own
    subscriber-management concerns (ADR-004 §3: outbound-only edge)."""

    def publish(self, event: Event) -> None: ...


class EventBus:
    def __init__(self, mission_id: str = ""):
        self.mission_id = mission_id
        self._subscribers: List[Subscriber] = []

    def subscribe(self, callback: Subscriber) -> None:
        self._subscribers.append(callback)

    def unsubscribe(self, callback: Subscriber) -> None:
        if callback in self._subscribers:
            self._subscribers.remove(callback)

    def publish(self, event: Event) -> None:
        if not event.mission_id:
            event.mission_id = self.mission_id
        for sub in list(self._subscribers):
            try:
                sub(event)
            except Exception as e:  # noqa: BLE001 -- isolation boundary, must never propagate
                logger.error("EventBus subscriber failed for %s: %s", event.code, e)


class NullEventPublisher:
    """Default no-op publisher. Every subsystem constructor accepts an
    optional publisher defaulting to this, so existing callers/tests that
    construct subsystems without one keep working unchanged (ADR-004 §14:
    minimal-diff integration)."""

    def publish(self, event: Event) -> None:
        pass


NULL_PUBLISHER = NullEventPublisher()
