"""
ADR-004 §4 (The Core Correction: Explicit Mission State).

Before this, "what phase is the mission in" was answerable only by knowing
which line of Gorev2Orchestrator.run() the interpreter was currently
executing. MissionContext is the single source of truth instead: mutated
only through named transition/blocking calls, published to EventBus on
every change, and synchronously queryable at any instant via snapshot_data()
so a late-attaching Ops Center can render current state without replaying
history.
"""
import time
import uuid
from dataclasses import dataclass, field
from threading import RLock
from typing import Optional

from core.mission.blocking import BlockingKind, BlockingState
from core.mission.phase import TERMINAL_PHASES, MissionPhase
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity


@dataclass
class MissionContextSnapshot:
    mission_id: str
    started_at: float
    elapsed_s: float
    timeout_budget_s: Optional[float]
    timeout_remaining_s: Optional[float]
    phase: MissionPhase
    phase_entered_at: float
    phase_elapsed_s: float
    blocking: Optional[BlockingState]


class MissionContext:
    def __init__(
        self,
        publisher: EventPublisher = NULL_PUBLISHER,
        mission_id: Optional[str] = None,
        timeout_budget_s: Optional[float] = None,
    ):
        self.publisher = publisher
        self.mission_id = mission_id or uuid.uuid4().hex[:12]
        self.timeout_budget_s = timeout_budget_s
        self._lock = RLock()
        self._started_at = time.time()
        self._phase = MissionPhase.MISSION_INIT
        self._phase_entered_at = self._started_at
        self._blocking: Optional[BlockingState] = None

    # ------------------------------------------------------------------
    # Phase transitions
    # ------------------------------------------------------------------
    @property
    def current_phase(self) -> MissionPhase:
        with self._lock:
            return self._phase

    def transition_to(self, phase: MissionPhase, reason: str = "", subsystem: str = "MissionContext") -> None:
        with self._lock:
            previous = self._phase
            now = time.time()
            phase_duration = now - self._phase_entered_at
            self._phase = phase
            self._phase_entered_at = now
            # A phase transition supersedes whatever it was blocked on.
            self._blocking = None

        severity = Severity.CRITICAL if phase in (
            MissionPhase.MISSION_FAILED, MissionPhase.MISSION_ABORTED, MissionPhase.MISSION_TIMEOUT,
        ) else Severity.INFO

        self.publisher.publish(Event(
            code="MISSION_PHASE_CHANGED",
            subsystem=subsystem,
            category=Category.LIFECYCLE,
            severity=severity,
            message=f"{previous.value} -> {phase.value}" + (f" ({reason})" if reason else ""),
            mission_id=self.mission_id,
            data={
                "from_phase": previous.value,
                "to_phase": phase.value,
                "previous_phase_duration_s": phase_duration,
                "reason": reason,
            },
        ))

    def is_terminal(self) -> bool:
        return self.current_phase in TERMINAL_PHASES

    # ------------------------------------------------------------------
    # Blocking-reason reporting (ADR-004 §8)
    # ------------------------------------------------------------------
    def set_blocking(
        self,
        waiting_on: str,
        owning_subsystem: str,
        kind: BlockingKind = BlockingKind.BLOCKING_WAIT,
        timeout_s: Optional[float] = None,
        cause: Optional[str] = None,
    ) -> None:
        now = time.time()
        state = BlockingState(
            waiting_on=waiting_on,
            owning_subsystem=owning_subsystem,
            kind=kind,
            since=now,
            timeout_at=(now + timeout_s) if timeout_s is not None else None,
            last_known_cause=cause,
        )
        with self._lock:
            self._blocking = state
        self.publisher.publish(Event(
            code="BLOCKING_STATE_ENTERED",
            subsystem=owning_subsystem,
            category=Category.WATCHDOG if kind == BlockingKind.BLOCKING_WAIT else Category.LIFECYCLE,
            severity=Severity.WARN if kind == BlockingKind.BLOCKING_WAIT else Severity.DEBUG,
            message=f"waiting_on={waiting_on} kind={kind.value}",
            mission_id=self.mission_id,
            data={"waiting_on": waiting_on, "kind": kind.value, "timeout_s": timeout_s, "cause": cause},
        ))

    def clear_blocking(self) -> None:
        with self._lock:
            had = self._blocking
            self._blocking = None
        if had is not None:
            self.publisher.publish(Event(
                code="BLOCKING_STATE_CLEARED",
                subsystem=had.owning_subsystem,
                category=Category.LIFECYCLE,
                severity=Severity.DEBUG,
                message=f"waiting_on={had.waiting_on} resolved after {had.elapsed_s():.1f}s",
                mission_id=self.mission_id,
                data={"waiting_on": had.waiting_on},
            ))

    @property
    def blocking(self) -> Optional[BlockingState]:
        with self._lock:
            return self._blocking

    # ------------------------------------------------------------------
    # Snapshot (§6) -- synchronous, lock-protected, cheap
    # ------------------------------------------------------------------
    def snapshot_data(self) -> MissionContextSnapshot:
        now = time.time()
        with self._lock:
            phase, phase_entered_at, blocking = self._phase, self._phase_entered_at, self._blocking
        elapsed = now - self._started_at
        remaining = (self.timeout_budget_s - elapsed) if self.timeout_budget_s is not None else None
        return MissionContextSnapshot(
            mission_id=self.mission_id,
            started_at=self._started_at,
            elapsed_s=elapsed,
            timeout_budget_s=self.timeout_budget_s,
            timeout_remaining_s=remaining,
            phase=phase,
            phase_entered_at=phase_entered_at,
            phase_elapsed_s=now - phase_entered_at,
            blocking=blocking,
        )
