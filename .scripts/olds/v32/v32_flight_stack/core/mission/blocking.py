"""
ADR-004 §8 (Blocking / Waiting Taxonomy). Every point where the mission
runtime is blocked is a named, reportable value -- never a silent wait.

EXPECTED_WAIT: correctly blocked by design (payload interlock before target
1 drops, debounce cooldown, non-blocking verification) -- never escalates.

BLOCKING_WAIT: acceptable only up to a timeout, after which WatchdogEngine
must treat it as a fault. Before ADR-004, the codebase had zero instances of
this second category actually enforcing a timeout -- that gap is what
WatchdogEngine (core/telemetry/watchdog.py) closes.
"""
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class BlockingKind(str, Enum):
    EXPECTED_WAIT = "EXPECTED_WAIT"
    BLOCKING_WAIT = "BLOCKING_WAIT"


@dataclass
class BlockingState:
    waiting_on: str
    owning_subsystem: str
    kind: BlockingKind = BlockingKind.BLOCKING_WAIT
    since: float = field(default_factory=time.time)
    timeout_at: Optional[float] = None
    last_known_cause: Optional[str] = None

    def elapsed_s(self, now: Optional[float] = None) -> float:
        return (now or time.time()) - self.since

    def remaining_s(self, now: Optional[float] = None) -> Optional[float]:
        if self.timeout_at is None:
            return None
        return max(0.0, self.timeout_at - (now or time.time()))

    def is_overdue(self, now: Optional[float] = None) -> bool:
        if self.kind != BlockingKind.BLOCKING_WAIT or self.timeout_at is None:
            return False
        return (now or time.time()) >= self.timeout_at
