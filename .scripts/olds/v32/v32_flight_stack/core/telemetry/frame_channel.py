"""
Separate from EventBus on purpose. The vision pipeline's camera feed does
NOT come from the vehicle over MAVLink/telemetry -- per the Görev 2
architecture mandate, vision processing runs on the GCS machine itself, the
same place the Mission Operations Center runs. So the camera frame is a
local, already-available resource, not something reached across the
vehicle telemetry link -- exactly why V31's Mission Dashboard could show it
directly, and why this one should too.

Frames are large, binary, and not meaningful as replay history the way
structured events are -- they do not belong in EventBus/EventStore's JSONL
timeline. FrameChannel is a single-slot, latest-frame-wins, non-blocking
handoff (same drop-oldest discipline as EventBus and V31's UIWorker
snapshot_queue), kept entirely separate from the structured telemetry path.
"""
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from core.detection.types import Detection


@dataclass
class FrameSample:
    frame_bgr: np.ndarray
    detections: List[Detection] = field(default_factory=list)
    ts: float = field(default_factory=time.time)


class FrameChannel:
    def __init__(self):
        self._lock = threading.Lock()
        self._last: Optional[FrameSample] = None

    def publish(self, frame_bgr: np.ndarray, detections: Optional[List[Detection]] = None) -> None:
        """Called from the mission loop -- must never block it. Drops the
        previous frame if the dashboard hasn't consumed it yet."""
        sample = FrameSample(frame_bgr=frame_bgr, detections=detections or [])
        with self._lock:
            self._last = sample

    def latest(self) -> Optional[FrameSample]:
        """Called from the dashboard's render thread. Always returns the
        most recent frame published so far (or None before the first one)."""
        with self._lock:
            return self._last
