"""
ADR-004 §15 (Logging Architecture) / §19 (Mission Replay). Append-only,
ordered JSONL record per mission_id -- the durability boundary independent
of whether any live subscriber (dashboard, health monitor) is attached, and
the source replay folds through the same RuntimeStateAggregator logic the
live view uses (ADR-004 §19: replay and live view are the same code path).

File I/O must never happen on the caller's thread (that caller is often the
mission's own asyncio loop) -- this reuses the exact thread+bounded-queue
isolation pattern ADR-005 recovered from V30/V31's UIWorker, applied here to
a different purpose.
"""
import json
import logging
import os
import queue
import threading
import time
from typing import Optional

from core.telemetry.events import Event

logger = logging.getLogger("telemetry.event_store")


class EventStore:
    def __init__(self, mission_id: str, log_dir: str = "logs", queue_size: int = 2000):
        self.mission_id = mission_id
        os.makedirs(log_dir, exist_ok=True)
        self.path = os.path.join(log_dir, f"mission_{mission_id}.jsonl")
        self._queue: "queue.Queue[Optional[Event]]" = queue.Queue(maxsize=queue_size)
        self._dropped = 0
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._run, name="EventStoreWriter", daemon=True)
        self._thread.start()

    def on_event(self, event: Event) -> None:
        """Subscribe this to EventBus. Never blocks the publisher: on a full
        queue the event is dropped and counted, not waited on."""
        try:
            self._queue.put_nowait(event)
        except queue.Full:
            self._dropped += 1

    def _run(self) -> None:
        with open(self.path, "a", encoding="utf-8") as f:
            while self._running or not self._queue.empty():
                try:
                    event = self._queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                if event is None:
                    continue
                try:
                    f.write(json.dumps(event.to_dict(), default=str) + "\n")
                    f.flush()
                except Exception as e:  # noqa: BLE001 -- writer thread must never die on one bad record
                    logger.error("EventStore write failed: %s", e)

    def stop(self, timeout_s: float = 2.0) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)

    @property
    def dropped_count(self) -> int:
        return self._dropped


def replay(path: str):
    """Yields Event-shaped dicts in recorded order, for ADR-004 §19 replay
    (feed through RuntimeStateAggregator.on_event to reconstruct state)."""
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                yield json.loads(line)
