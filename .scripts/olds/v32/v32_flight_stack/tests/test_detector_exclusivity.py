"""
Guards the single-detector-consumer invariant (ADR-008 B1).

HISTORY -- what this file used to assert, and why that was wrong:

A race found during continuous audit (2026-08-13) had _detection_loop() and
precision-control consumers (CenteringController.go_to_and_center,
PayloadReleaseService._verify_marker) independently calling detect() on the
same detector instance. HSVContourDetector's mutable per-shape streak state
(_last_center/_streak) depends on being fed one coherent frame sequence, so
two interleaved callers made streak-commit timing scheduling-order-
dependent. The fix at the time was a _precision_control_active flag that
made _detection_loop SKIP its own detect() while a consumer was running, and
this file asserted exactly that skipping.

That flag also gated the VISION_FRAME_PROCESSED heartbeat, so vision health
went DOWN 0.3s into every pursuit and stayed there (2026-08-16 21:04 run:
two windows of 77.3s and 85.2s with zero detection cycles, and it would have
stayed muted for the whole payload phase since _search_complete permanently
blocks the only code path that cleared the flag).

ADR-008 B1 inverts the fix: the detection loop is the ONE caller of detect()
and never stops; everyone else consumes DetectionFeed. The race is closed by
construction rather than by a flag that can get stuck. So the invariant this
file guards is now:

  1. _detection_loop calls detect() unconditionally, for the whole mission.
  2. Every cycle publishes a heartbeat AND a feed sample.
  3. CenteringController never calls detect() -- it reads the feed.
  4. A silent producer must read as "no detection", not as a frozen one.

ADR-010 P3 moved the loop, not the invariant. It used to live on
Gorev2Orchestrator and was cancelled in that class's `finally`, which gave
it GOREV 2's lifetime rather than the mission's -- measured on the
2026-08-17 14:49 run: the last VISION_FRAME_PROCESSED lands at t=236.3s,
exactly the GOREV2_COMPLETE -> GOREV3_START transition, leaving the final
64.7s of flight with no frames at all, a frozen dashboard, and a Gorev 3
centering that saw 0 committed detections in 200 attempts. The loop now
lives in core/detection/vision_runtime.py with MISSION lifetime, so these
tests target VisionRuntime -- and point 1 above now means what it always
said it meant.
"""
import asyncio
import pytest

from mocks.mock_camera_source import MockCameraSource
from mocks.mock_flight_backend import MockFlightBackend
from core.detection.detection_feed import DetectionFeed
from core.detection.types import Detection
from core.detection.vision_runtime import VisionRuntime
from core.navigation.centering_controller import CenteringController
from core.telemetry.event_bus import NULL_PUBLISHER


class _CountingDetector:
    """Counts detect() calls and reports one fixed shape, so a test can tell
    both 'was detect() called' and 'did the result reach the feed'."""

    def __init__(self, shape_type="MAVI_ALTIGEN", center_px=(500.0, 240.0)):
        self.call_count = 0
        self.shape_type = shape_type
        self.center_px = center_px

    async def detect(self, frame):
        self.call_count += 1
        return [Detection(shape_type=self.shape_type, confidence=0.9,
                          center_px=self.center_px, bbox_px=(0, 0, 10, 10))]


def _bare_orchestrator(detector, feed=None) -> VisionRuntime:
    """The vision pipeline under test. ADR-010 P3: a real VisionRuntime
    rather than a hand-assembled orchestrator -- the runtime's entire
    constructor is camera/detector/feed/publisher, so there is nothing left
    to bypass, and the test now exercises the shipped object."""
    return VisionRuntime(MockCameraSource(), detector, feed or DetectionFeed(),
                         publisher=NULL_PUBLISHER)


async def _run_loop_briefly(orch, duration_s=0.35):
    task = asyncio.ensure_future(orch._detection_loop())
    await asyncio.sleep(duration_s)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


@pytest.mark.asyncio
async def test_detection_loop_is_the_only_detect_caller_and_never_pauses():
    """THE invariant. There is no longer any state -- flag, phase or
    otherwise -- that can stop this loop short of cancelling it. The old
    test asserted call_count == 0 while a pursuit was active; that is now
    precisely the regression to catch."""
    detector = _CountingDetector()
    orch = _bare_orchestrator(detector)

    await _run_loop_briefly(orch)

    assert detector.call_count >= 2
    assert not hasattr(orch, "_precision_control_active"), (
        "the exclusivity flag is gone by design -- reintroducing it would "
        "re-mute the vision heartbeat during every pursuit (ADR-008 B1)")


@pytest.mark.asyncio
async def test_detection_loop_publishes_every_result_to_the_shared_feed():
    detector = _CountingDetector()
    feed = DetectionFeed()
    orch = _bare_orchestrator(detector, feed)

    await _run_loop_briefly(orch)

    assert feed.seq == detector.call_count, "every detect() result must reach the feed"
    assert feed.get("MAVI_ALTIGEN") is not None


@pytest.mark.asyncio
async def test_centering_consumes_the_feed_and_never_calls_detect():
    """CenteringController has no detector to call anymore -- it is
    constructed with the feed. This asserts it drives the vehicle purely
    from feed contents: a detector that is never invoked, yet centering
    still commands velocity because the loop filled the feed."""
    detector = _CountingDetector(center_px=(500.0, 240.0))  # off-centre, never converges
    feed = DetectionFeed()
    orch = _bare_orchestrator(detector, feed)
    flight = MockFlightBackend()
    controller = CenteringController(flight, feed, MockCameraSource())
    controller.lateral_timeout_s = 0.5  # keep this "never converges" test fast

    assert not hasattr(controller, "detector")

    loop_task = asyncio.ensure_future(orch._detection_loop())
    await asyncio.sleep(0.15)  # let the feed fill
    detect_calls_before = detector.call_count

    converged = await controller.go_to_and_center("MAVI_ALTIGEN")

    loop_task.cancel()
    try:
        await loop_task
    except asyncio.CancelledError:
        pass

    assert converged is False
    velocity_calls = [c for c in flight.calls if c[0] == "set_velocity_body"]
    assert any(c[1]["right_m_s"] != 0.0 for c in velocity_calls), (
        "centering must have acted on feed contents")
    # detect() kept being called -- by the LOOP, throughout centering. That
    # is the behaviour whose absence caused vision to report DOWN for 82s.
    assert detector.call_count > detect_calls_before


@pytest.mark.asyncio
async def test_feed_goes_stale_rather_than_serving_a_frozen_detection():
    """A stopped producer must read as 'I do not know where the target is',
    not as the last thing it ever saw. This is the property that makes a
    dead detection loop impossible to mistake for a live one -- both in the
    control loop and on the dashboard overlay."""
    feed = DetectionFeed(stale_after_s=0.05)
    feed.publish([Detection(shape_type="KIRMIZI_UCGEN", confidence=0.9,
                            center_px=(10.0, 10.0), bbox_px=(0, 0, 20, 20))])

    assert feed.get("KIRMIZI_UCGEN") is not None
    assert feed.is_stale() is False

    await asyncio.sleep(0.1)  # producer went quiet

    assert feed.get("KIRMIZI_UCGEN") is None
    assert feed.is_stale() is True
    assert feed.detections() == []
    # latest() still exposes the frozen sample for diagnostics -- only the
    # control-path accessors refuse it.
    assert feed.latest() is not None


@pytest.mark.asyncio
async def test_detector_failure_lets_the_feed_expire_instead_of_republishing():
    """A camera/detector that starts failing must NOT keep the last good
    sample alive -- the loop publishes nothing, so freshness decays and
    consumers correctly report the target as lost."""
    class _FailingAfterFirst:
        def __init__(self):
            self.call_count = 0

        async def detect(self, frame):
            self.call_count += 1
            if self.call_count > 1:
                raise RuntimeError("camera gone")
            return [Detection(shape_type="MAVI_ALTIGEN", confidence=0.9,
                              center_px=(320.0, 240.0), bbox_px=(0, 0, 10, 10))]

    feed = DetectionFeed(stale_after_s=0.05)
    orch = _bare_orchestrator(_FailingAfterFirst(), feed)

    await _run_loop_briefly(orch, duration_s=0.3)

    assert feed.seq == 1, "only the one successful cycle may publish"
    assert feed.is_stale() is True
    assert feed.get("MAVI_ALTIGEN") is None
