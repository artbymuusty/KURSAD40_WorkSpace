from typing import Dict
from core.config.parameters import DEBOUNCE_DURATION_S
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

class DebounceTracker:
    def __init__(self, duration_s: float = DEBOUNCE_DURATION_S, publisher: EventPublisher = NULL_PUBLISHER):
        self.duration_s = duration_s
        self._last_processed: Dict[str, float] = {}
        self.publisher = publisher

    def mark_processed(self, shape_type: str, now: float) -> None:
        self._last_processed[shape_type] = now

    def is_in_cooldown(self, shape_type: str, now: float) -> bool:
        """shape_type, mark_processed çağrısından bu yana duration_s saniyeden az
        zaman geçtiyse True döner — bu süre boyunca aynı hedef yeniden algılansa bile
        hiçbir işlem yapılmamalıdır (Offboard döngüsüne tekrar girilmesini, aynı şeklin
        tekrar işlenmesini ve sonsuz döngüyü önlemek için)."""
        last = self._last_processed.get(shape_type, 0.0)
        remaining = self.duration_s - (now - last)
        in_cooldown = remaining > 0
        if in_cooldown:
            # ADR-004 §8: EXPECTED_WAIT, never escalated -- just visible so
            # an operator doesn't mistake "correctly ignoring a debounced
            # detection" for a stuck pipeline.
            self.publisher.publish(Event(
                code="DEBOUNCE_STATE_SYNC", subsystem="DebounceTracker", category=Category.VISION,
                severity=Severity.DEBUG, data={"shape_type": shape_type, "remaining_s": remaining},
            ))
        return in_cooldown
