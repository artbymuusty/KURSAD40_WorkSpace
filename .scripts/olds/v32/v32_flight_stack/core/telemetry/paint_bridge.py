"""Single-slot, drop-oldest handoff of COMPOSED dashboard frames to the main thread.

ADR-006 (macOS): Cocoa requires every cv2 GUI call on the process main thread,
while ADR-005 §3 requires the dashboard's state/composition/lifecycle to stay
on its own dedicated thread and forbids cv2 calls on the mission thread. This
bridge is what lets both hold at once: MissionOpsDashboard still composes the
full frame (camera panel + telemetry column) on its own thread and publishes
the finished image here; the main thread -- which, under ADR-006's implemented
design, no longer runs the mission -- drains it and performs the paint.

Deliberately single-slot and drop-oldest: the dashboard must never block on
the painter, and a painter that falls behind should show the newest frame, not
a backlog. This mirrors the bounded `queue.Queue(maxsize=1)` + drop-oldest
behaviour ADR-005 §4 records as UIWorker's proven pattern.

Linux/Windows do not use this at all -- the dashboard paints on its own thread
exactly as before.
"""
import threading
from typing import Optional, Tuple

import numpy as np


class PaintBridge:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._slot: Optional[Tuple[str, "np.ndarray"]] = None
        self._enabled = False
        self._close_requested = False

    # -- wiring -------------------------------------------------------
    def enable(self) -> None:
        """Called by the dashboard when it decides to delegate painting."""
        self._enabled = True

    @property
    def enabled(self) -> bool:
        return self._enabled

    # -- producer (dashboard thread) ----------------------------------
    def publish(self, window_name: str, image: "np.ndarray") -> None:
        with self._lock:
            self._slot = (window_name, image)  # drop-oldest: overwrite

    # -- consumer (main thread) ---------------------------------------
    def take(self) -> Optional[Tuple[str, "np.ndarray"]]:
        """Return the newest composed frame, or None if nothing new."""
        with self._lock:
            item = self._slot
            self._slot = None
            return item

    # -- shutdown -----------------------------------------------------
    def request_close(self) -> None:
        """Main thread asks the mission to shut down (window closed / 'q')."""
        self._close_requested = True

    @property
    def close_requested(self) -> bool:
        return self._close_requested


# Process-wide instance. A module-level singleton is used deliberately: the
# dashboard is constructed deep inside build_ops_center() during the mission
# coroutine, so the entrypoint cannot hand it a reference before it exists.
MAIN_THREAD_PAINT = PaintBridge()
