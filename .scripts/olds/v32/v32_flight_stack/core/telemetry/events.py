"""
ADR-004 §7 (Event Model). Structured event envelope used by every subsystem
to report state to the Mission Operations Center. This is the single event
type published on EventBus, consumed by RuntimeStateAggregator, HealthMonitor,
WatchdogEngine, and EventStore alike -- one source of truth, no parallel
telemetry format.
"""
import itertools
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional


class Category(str, Enum):
    LIFECYCLE = "LIFECYCLE"
    TELEMETRY = "TELEMETRY"
    VISION = "VISION"
    NAVIGATION = "NAVIGATION"
    PAYLOAD = "PAYLOAD"
    HEALTH = "HEALTH"
    WATCHDOG = "WATCHDOG"
    LOG = "LOG"
    OPERATOR = "OPERATOR"


class Severity(str, Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARN = "WARN"
    CRITICAL = "CRITICAL"
    FATAL = "FATAL"


# Monotonically increasing counter so event_id stays sortable even when two
# events share the same wall-clock millisecond (ULID-like without pulling in
# a dependency).
_seq = itertools.count()


def new_event_id() -> str:
    return f"{int(time.time() * 1000):013d}-{next(_seq):06d}-{uuid.uuid4().hex[:8]}"


@dataclass
class Event:
    """ADR-004 §7.1 envelope. `data` is event-specific, structured, JSON-safe."""
    code: str
    subsystem: str
    message: str = ""
    category: Category = Category.LIFECYCLE
    severity: Severity = Severity.INFO
    mission_id: str = ""
    correlation_id: Optional[str] = None
    data: dict = field(default_factory=dict)
    event_id: str = field(default_factory=new_event_id)
    ts: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "event_id": self.event_id,
            "ts": self.ts,
            "mission_id": self.mission_id,
            "category": self.category.value if isinstance(self.category, Category) else self.category,
            "subsystem": self.subsystem,
            "severity": self.severity.value if isinstance(self.severity, Severity) else self.severity,
            "code": self.code,
            "message": self.message,
            "correlation_id": self.correlation_id,
            "data": self.data,
        }
