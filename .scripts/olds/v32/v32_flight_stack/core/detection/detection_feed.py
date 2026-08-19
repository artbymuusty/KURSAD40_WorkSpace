"""
ADR-008 B1: the single place a detection result crosses from the one
component allowed to call `IDetector.detect()` to everyone that needs to
read one.

Why this exists at all -- the 2026-08-16 21:04 run:

    Gorev2Orchestrator._detection_loop() and
    CenteringController.go_to_and_center() were BOTH calling detect() on
    the same HSVContourDetector instance. That detector carries real
    mutable per-shape streak state (_last_center/_streak) whose
    N-consecutive-frame commit logic only works when it is fed one
    coherent frame sequence, so the previous fix made _detection_loop skip
    its own detect() (and, fatally, its VISION_FRAME_PROCESSED heartbeat)
    for the entire duration of a pursuit. Vision therefore reported DOWN
    within 0.3s of every centering start and stayed there: 77.3s and 85.2s
    with zero detection cycles, while the dashboard kept redrawing frozen
    boxes on live frames so it all looked fine.

The exclusivity flag solved the right problem in the wrong direction.
Here the loop is the ONLY consumer of the detector -- so the streak stays
coherent by construction, with no flag to get stuck True -- and everyone
else reads the loop's latest result through this feed.

Freshness is part of the contract, not an afterthought: a consumer must be
able to distinguish "the loop just told me there is no target" from "the
loop has gone quiet", which is precisely the distinction the old frozen
`_latest_detections` list destroyed. Anything older than `stale_after_s`
is reported as no sample at all.

Concurrency: one producer coroutine and all consumers live on the same
single-threaded asyncio loop, and every published sample is an immutable
snapshot replaced by whole-object assignment -- so a reader either sees
the previous sample or the next one, never a half-updated one, and no lock
is needed. (FrameChannel differs: it is read from the dashboard's own
thread, so it does take one.)
"""
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from core.config.parameters import DETECTION_STALE_AFTER_S
from core.detection.types import Detection


@dataclass(frozen=True)
class DetectionSample:
    """One complete detect() result: every shape found in a single frame."""
    detections: Tuple[Detection, ...]
    seq: int
    ts: float

    def age_s(self, now: Optional[float] = None) -> float:
        return (now or time.time()) - self.ts

    def get(self, shape_type: str) -> Optional[Detection]:
        return next((d for d in self.detections if d.shape_type == shape_type), None)


class DetectionFeed:
    def __init__(self, stale_after_s: float = DETECTION_STALE_AFTER_S):
        self.stale_after_s = stale_after_s
        self._sample: Optional[DetectionSample] = None
        self._seq: int = 0

    # -- producer side (Gorev2Orchestrator._detection_loop only) -----------
    def publish(self, detections: Sequence[Detection]) -> DetectionSample:
        self._seq += 1
        sample = DetectionSample(detections=tuple(detections), seq=self._seq, ts=time.time())
        self._sample = sample
        return sample

    # -- consumer side ----------------------------------------------------
    def latest(self) -> Optional[DetectionSample]:
        """The most recent sample regardless of age. Use `fresh()` for
        anything that drives the vehicle."""
        return self._sample

    def fresh(self, now: Optional[float] = None) -> Optional[DetectionSample]:
        """The most recent sample, or None if the producer has gone quiet
        for longer than `stale_after_s`."""
        sample = self._sample
        if sample is None or sample.age_s(now) > self.stale_after_s:
            return None
        return sample

    def get(self, shape_type: str, now: Optional[float] = None) -> Optional[Detection]:
        """The active target's latest detection, or None if it is not in the
        newest frame OR the feed has gone stale. Both cases mean the same
        thing to a control loop -- "I do not currently know where it is" --
        and conflating them is safe here only because they are conflated
        deliberately; `fresh()` is available when a caller needs to tell
        them apart for reporting."""
        sample = self.fresh(now)
        return sample.get(shape_type) if sample is not None else None

    def detections(self, now: Optional[float] = None) -> List[Detection]:
        sample = self.fresh(now)
        return list(sample.detections) if sample is not None else []

    def age_s(self, now: Optional[float] = None) -> Optional[float]:
        sample = self._sample
        return sample.age_s(now) if sample is not None else None

    def is_stale(self, now: Optional[float] = None) -> bool:
        return self.fresh(now) is None

    @property
    def seq(self) -> int:
        return self._seq
