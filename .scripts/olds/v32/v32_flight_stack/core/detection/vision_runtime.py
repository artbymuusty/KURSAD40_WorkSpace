"""ADR-010 P3: the vision pipeline, owned for the whole MISSION rather than
for the duration of Görev 2.

Why this file exists -- measured on the V1''' run (mission_81cfefe66ad7):

    VISION_FRAME_PROCESSED: 2185 events, 9.35 Hz, worst gap 0.13 s,
    ZERO gaps > 1.5 s ... and then the last one at t=236.3 s, which is
    exactly the instant of GOREV2_COMPLETE -> GOREV3_START. No frames at
    all for the remaining 64.7 s of a 301 s flight.

Both loops used to live in Gorev2Orchestrator.run(), started at its top and
cancelled in its `finally`. So while Görev 2 ran the feed was healthy --
that part of ADR-008 B1 worked exactly as designed -- but the moment Görev 2
returned, the frame grabber and the detector died with it. Three consequences,
all of which showed up in that run:

  1. The dashboard froze on its last frame for the rest of the flight: the
     camera panel keeps rendering whatever FrameChannel last received, and
     nothing was publishing any more.
  2. Görev 3's own descent centering got 0 committed detections out of 200
     attempts (first miss already at 3.03 m) and could only time out --
     it was reading a feed whose producer no longer existed.
  3. Gorev3PickupPhase held a direct reference to the detector and called
     detect() itself, which is the SECOND-caller problem ADR-008 B1 exists
     to prevent: HSVContourDetector's per-shape streak state is only
     coherent when exactly one caller feeds it one frame sequence.

So the invariant ADR-008 B1 established -- "exactly one detect() caller,
running unconditionally" -- was right, but scoped to the wrong lifetime.
Here it is scoped to the mission: started before the master FSM runs and
stopped after it finishes, so every phase (Görev 2, Görev 3, return, land)
consumes the same single-owner feed and the display never stops.

Nothing about the loops themselves changed -- cadence, throttled logging,
failure escalation, the busy-spin fix and the deliberate "do not publish on
failure, let the feed age out" rule are all carried over verbatim.
"""
import asyncio
import logging
import time

from core.detection.detection_feed import DetectionFeed
from core.interfaces.i_camera_source import ICameraSource
from core.interfaces.i_detector import IDetector
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger(__name__)

# Unchanged from Gorev2Orchestrator so HealthMonitor's watchdog registration,
# and every historical log/event that refers to it, keep working. The name
# now describes where the vision heartbeat comes from rather than which
# mission phase happens to be running.
VISION_SUBSYSTEM = "Gorev2Orchestrator.vision"


class FeedDetector(IDetector):
    """An IDetector that answers from the feed instead of running inference.

    Görev 3's phases take an IDetector because IPayloadVisibilityStrategy is
    written against that interface. Handing them the real detector makes
    them a second detect() caller, which is exactly what corrupts
    HSVContourDetector's per-shape streak state -- so they get this instead:
    same interface, but every call returns whatever the one real caller
    (VisionRuntime._detection_loop) most recently published.

    `frame` is accepted and ignored, deliberately. The frame a caller
    happens to hold is not the frame the streak logic was advanced on, and
    re-running detect() on it is the bug. Freshness still applies: if the
    producer has gone quiet, this returns [] rather than a stale answer."""

    def __init__(self, detection_feed: DetectionFeed):
        self.detection_feed = detection_feed

    async def detect(self, frame=None) -> list:
        return self.detection_feed.detections()


class VisionRuntime:
    """Owns the camera->detector->DetectionFeed pipeline for the mission."""

    def __init__(self, camera: ICameraSource, detector: IDetector,
                 detection_feed: DetectionFeed, frame_channel=None,
                 publisher: EventPublisher = NULL_PUBLISHER):
        self.camera = camera
        self.detector = detector
        self.detection_feed = detection_feed
        self.frame_channel = frame_channel
        self.publisher = publisher
        self._latest_detections: list = []
        self._vision_consecutive_failures: int = 0
        self._tasks: list = []

    def _publish(self, code, message="", severity=Severity.INFO,
                 category=Category.VISION, subsystem=VISION_SUBSYSTEM, data=None):
        self.publisher.publish(Event(
            code=code, subsystem=subsystem, category=category, severity=severity,
            message=message, data=data or {},
        ))

    # -- lifecycle --------------------------------------------------------
    def start(self) -> None:
        if self._tasks:
            return
        self._tasks = [
            asyncio.ensure_future(self._frame_grab_loop()),
            asyncio.ensure_future(self._detection_loop()),
        ]
        logger.info("[VISION] pipeline basladi (mission omru boyunca calisir).")

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        for task in self._tasks:
            try:
                await task
            except asyncio.CancelledError:
                pass
        self._tasks = []
        logger.info("[VISION] pipeline durduruldu.")

    async def __aenter__(self):
        self.start()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self.stop()
        return False

    # -- loops ------------------------------------------------------------
    async def _frame_grab_loop(self) -> None:
        """Continuously grabs frames and publishes them to the dashboard's
        FrameChannel, overlaid with whatever detections _detection_loop most
        recently computed (which may be very slightly behind the newest
        frame during heavy inference -- normal and expected, and far better
        than the video itself stalling).

        Split from _detection_loop, not merged with it, so live video is
        never gated by detection latency."""
        # Throttled the same way _detection_loop's failure log is, and for
        # the same reason (operator-approved, 2026-08-16): this branch runs
        # at ~30Hz, so a camera that is merely slow to come up buries the
        # whole run.
        failure_log_interval_s = 3.0
        last_failure_log = 0.0
        consecutive_failures = 0
        while True:
            try:
                frame = await self.camera.get_frame()
                if self.frame_channel is not None:
                    # ADR-008 B1: publish only detections the detection loop
                    # actually produced RECENTLY. A stale feed draws no boxes
                    # at all and says so (see FrameSample.detections_stale),
                    # rather than redrawing frozen boxes over live frames.
                    sample = self.detection_feed.fresh()
                    self.frame_channel.publish(
                        frame,
                        list(sample.detections) if sample is not None else [],
                        detections_stale=sample is None,
                        detections_age_s=self.detection_feed.age_s(),
                    )
                consecutive_failures = 0
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001 -- a preview-loop hiccup must never affect the mission
                consecutive_failures += 1
                now = time.time()
                if now - last_failure_log >= failure_log_interval_s:
                    last_failure_log = now
                    logger.warning(f"Frame grab loop hatasi (art arda {consecutive_failures}. hata, "
                                   f"mission etkilenmez): {e}")

            # ~30Hz, matching CAMERA_PROCESS_FREQ_HZ_MAX.
            await asyncio.sleep(1.0 / 30.0)

    async def _detection_loop(self) -> None:
        """THE detection loop -- the only place detect() is called.

        ADR-008 B1 established the invariant; ADR-010 P3 gave it the right
        lifetime. HSVContourDetector's per-shape streak state
        (_last_center/_streak) is fed exactly one coherent frame sequence,
        and VISION_FRAME_PROCESSED is published every cycle for the whole
        mission -- so vision health reflects the vision pipeline rather than
        which mission phase happens to be running."""
        heartbeat_interval_s = 3.0
        last_heartbeat = 0.0
        while True:
            try:
                frame = await self.camera.get_frame()
                detections = await self.detector.detect(frame)
                self._latest_detections = detections
                sample = self.detection_feed.publish(detections)

                self._publish("VISION_FRAME_PROCESSED", severity=Severity.DEBUG,
                              data={"detector_ready": True, "detection_count": len(detections),
                                    "seq": sample.seq,
                                    "shapes": [d.shape_type for d in detections]})

                # Throttled INFO heartbeat so a "no detections at all" run is
                # immediately distinguishable from "detection_loop isn't
                # running" or "camera keeps failing" (see the except branch).
                now = time.time()
                if now - last_heartbeat >= heartbeat_interval_s:
                    last_heartbeat = now
                    shapes = [d.shape_type for d in detections]
                    logger.info(f"[VISION] detect() calisiyor -- bu karede {len(shapes)} tespit: {shapes}")
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001 -- a vision hiccup must never affect the mission
                self._vision_consecutive_failures += 1
                now = time.time()
                if now - last_heartbeat >= heartbeat_interval_s:
                    last_heartbeat = now
                    logger.warning(f"[VISION] detect() basarisiz (art arda {self._vision_consecutive_failures}. hata, "
                                   f"mission etkilenmez): {e}")
                # 30 consecutive failures (~3s at this cadence) is a real,
                # mission-critical condition, escalated once and loudly.
                if self._vision_consecutive_failures == 30:
                    logger.error("[VISION] KRITIK: kamera/detector 3 saniyedir surekli hata veriyor -- "
                                 "vision pipeline calismiyor olabilir.")
                    self._publish("VISION_PIPELINE_DOWN", str(e), severity=Severity.CRITICAL,
                                  data={"consecutive_failures": self._vision_consecutive_failures})
                self._latest_detections = []
                # NOTE: nothing is published to detection_feed here on
                # purpose -- a failing detector must let the feed AGE OUT
                # (consumers then correctly read "target unknown") rather
                # than keep the last good sample alive forever.
            else:
                self._vision_consecutive_failures = 0

            # ADR-008 B1: outside the try/except so BOTH paths pay it. When
            # this was inside, a camera whose get_frame() raises without
            # awaiting gave the loop no yield point at all and spun the
            # event loop hot, starving every other coroutine on it.
            await asyncio.sleep(0.1)  # ~10Hz ceiling, matching CAMERA_PROCESS_FREQ_HZ_MIN
