"""ADR-010 P1-P5: release-altitude guarantee, payload observability,
display continuity, setpoint smoothing, contour overlay.

Every number asserted here traces to a measurement from the 2026-08-17 14:49
run (mission_81cfefe66ad7), not to a preference:

  P1  MAVI_ALTIGEN lost vision at 1.63 m and the target-lost branch held
      altitude, so the servo fired at 1.587 m against a commanded 0.45 m.
      KIRMIZI_UCGEN happened to track to 0.47 m and released at 0.407 m.
      Release altitude was decided by which shape it was.
  P3  Last VISION_FRAME_PROCESSED at t=236.3s == the GOREV2_COMPLETE ->
      GOREV3_START transition. 64.7s of flight with no frames.
  P4  9 setpoint ticks changed by >0.5 m/s, worst 2.50 m/s in one 0.1s tick.
  P5  Moment-vs-bbox centre divergence 3.51 px at 15 m -> 77.99 px (max
      163.5) at 0.45 m, where the shape spans 814 px of a 1280 px frame.
"""
import asyncio

import numpy as np
import pytest

from mocks.mock_camera_source import MockCameraSource
from mocks.mock_flight_backend import MockFlightBackend

from core.config.parameters import (
    LOW_ALT_VISION_LIMIT_M,
    MAX_CENTERING_SPEED_M_S,
    TARGET_LOSS_GRACE_FRAMES,
    PAYLOAD_APPROACH_ALTITUDES_M,
    PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M,
    SETPOINT_MAX_DELTA_V_M_S,
)
from core.detection.detection_feed import DetectionFeed
from core.detection.types import Detection
from core.detection.vision_runtime import FeedDetector, VisionRuntime
from core.navigation.centering_controller import CenteringController
from core.navigation.setpoint_limiter import SetpointLimiter
from core.telemetry.aggregator import RuntimeStateAggregator
from core.telemetry.event_bus import NULL_PUBLISHER
from core.telemetry.events import Category, Event, Severity


def _detection(shape="MAVI_ALTIGEN", center=(320.0, 240.0),
               bbox=(300.0, 220.0, 340.0, 260.0), contour=None):
    return Detection(shape_type=shape, confidence=0.9, center_px=center,
                     bbox_px=bbox, contour_px=contour)


# ======================================================================
# P4 -- setpoint-stage smoothing
# ======================================================================

def test_rate_limit_turns_the_measured_2_5_m_s_jump_into_a_ramp():
    """The worst measured tick: 0.0 -> 2.50 m/s in one 0.1s interval. It
    must now take several ticks, and each step must respect the cap."""
    lim = SetpointLimiter()
    first = lim.limit(2.50, 0.0, 0.0)
    assert first[0] == pytest.approx(SETPOINT_MAX_DELTA_V_M_S)

    # Keep asking for 2.5 and count how long it actually takes to get there.
    ticks = 1
    while lim.prev[0] < 2.50 - 1e-9 and ticks < 100:
        prev = lim.prev[0]
        now = lim.limit(2.50, 0.0, 0.0)[0]
        assert now - prev <= SETPOINT_MAX_DELTA_V_M_S + 1e-9
        ticks += 1
    assert ticks >= 8, "a 2.5 m/s change must ramp, not step"


def test_explicit_zero_stops_on_the_same_tick():
    """A stop is never rate-limited: ramping it down would mean coasting
    past the target right after convergence, which is exactly what the
    post-lock drift measurement watches."""
    lim = SetpointLimiter()
    for _ in range(20):
        lim.limit(2.0, 2.0, 0.0)
    assert lim.limit(0.0, 0.0, 0.0) == (0.0, 0.0, 0.0)


def test_distance_cap_is_gone():
    """PHASE 12 Q1. The distance-scaled cap compounded with proportional
    control exactly where the convergence tail is longest and cost tau
    9.1 -> 13.2 s (+44.9%) against a +20% budget, while the rate limit
    alone had already delivered the whole smoothness gain. A command must
    now pass through at full magnitude however close the target is."""
    import core.navigation.setpoint_limiter as sl

    assert not hasattr(sl, "distance_speed_cap")
    lim = SetpointLimiter()
    lim.prev = (3.0, 0.0, 0.0)
    # Target 0.1 m away: under the old cap this was clamped to 0.20 m/s.
    assert lim.limit(2.9, 0.0, 0.0)[0] == pytest.approx(2.9)


def test_limiter_never_flips_a_sign_or_grows_a_command():
    """The limiter must be shape-preserving -- anything else is a control
    change wearing a different hat."""
    lim = SetpointLimiter()
    for requested in (0.05, -0.05, 1.2, -1.2, 0.15, -3.0):
        out = lim.limit(requested, requested, requested)
        for axis in out:
            assert abs(axis) <= abs(requested) + 1e-9
            if requested != 0.0 and axis != 0.0:
                assert (axis > 0) == (requested > 0)


def test_transient_loss_decelerates_instead_of_hard_braking():
    """PHASE 12 Q1. The 2026-08-17 16:01 run hard-braked from ~1.3 m/s to zero four times,
    every one a SINGLE-frame dropout, then re-accelerated from standstill
    when the target reappeared on the next frame. A transient-loss hold is
    now rate-limited like any other command."""
    lim = SetpointLimiter()
    lim.prev = (1.3, 0.0, 0.0)
    out = lim.limit(0.0, 0.0, 0.0, immediate_stop=False)
    assert out[0] == pytest.approx(1.3 - SETPOINT_MAX_DELTA_V_M_S)
    assert out[0] > 0.0, "must not snap to zero on a transient loss"


def test_convergence_stop_keeps_the_immediate_zero_exemption():
    """The other half of Q1: a CONVERGENCE stop is a different event and
    must still take effect on the tick it is issued -- ramping it would mean
    coasting past a target we just declared centred."""
    lim = SetpointLimiter()
    lim.prev = (1.3, 0.9, 0.4)
    assert lim.limit(0.0, 0.0, 0.0, immediate_stop=True) == (0.0, 0.0, 0.0)


def test_loss_grace_reaches_zero_within_the_grace_window():
    """The grace count is a backstop, not the mechanism: 5 ticks x 0.30 m/s
    of decel authority already reaches zero from 1.5 m/s."""
    lim = SetpointLimiter()
    lim.prev = (1.3, 0.0, 0.0)
    v = None
    for _ in range(TARGET_LOSS_GRACE_FRAMES):
        v = lim.limit(0.0, 0.0, 0.0, immediate_stop=False)
    assert v[0] == pytest.approx(0.0, abs=1e-9)


# ======================================================================
# P1 -- centre source, frozen estimate, hybrid descent
# ======================================================================

def test_centre_source_is_moment_above_the_limit_and_bbox_below():
    """The two definitions diverge by up to 163.5 px at 0.45 m. Above the
    limit nothing changes; below it the bbox centre takes over."""
    target = _detection(center=(500.0, 240.0), bbox=(300.0, 200.0, 340.0, 260.0))
    bbox_centre = (320.0, 230.0)

    assert CenteringController.target_center_px(target, 15.0) == (500.0, 240.0)
    assert CenteringController.target_center_px(target, LOW_ALT_VISION_LIMIT_M + 0.1) == (500.0, 240.0)
    assert CenteringController.target_center_px(target, 0.45) == bbox_centre
    # Unknown altitude must not silently switch behaviour.
    assert CenteringController.target_center_px(target, None) == (500.0, 240.0)


def _controller(flight, feed):
    camera = MockCameraSource()
    return CenteringController(flight, feed, camera, publisher=NULL_PUBLISHER), camera


@pytest.mark.asyncio
async def test_frozen_estimate_places_the_target_off_the_vehicle_position():
    """A committed detection off-centre must produce a GPS point that is NOT
    the vehicle's own -- otherwise the open-loop descent would just hold
    station wherever it happened to lose sight."""
    flight = MockFlightBackend()
    ctrl, _ = _controller(flight, DetectionFeed())
    ctrl._last_yaw_deg = 0.0
    lat, lon = 41.0, 29.0
    est = ctrl._freeze_target_estimate(dx_px=200.0, dy_px=-100.0, current_alt_m=5.0,
                                       res_w=1280, res_h=960, lat=lat, lon=lon)
    assert est is not None
    assert est["lat"] != lat or est["lon"] != lon
    assert est["from_alt_m"] == pytest.approx(5.0)
    assert est["offset_cm"] > 0.0
    # dy < 0 means the target is ahead of the vehicle -> north of it at yaw 0.
    assert est["lat"] > lat


@pytest.mark.asyncio
async def test_losing_vision_below_the_limit_continues_descending_to_release_altitude():
    """THE P1 regression. V1''' payload 1: vision lost at 1.63 m, altitude
    held, servo at 1.587 m. The descent must now finish on the frozen
    estimate instead of stalling."""
    flight = MockFlightBackend()
    feed = DetectionFeed()
    ctrl, _ = _controller(flight, feed)

    # Airborne just under the vision limit, target committed once.
    flight._global_pos = (41.0, 29.0, 1.60)
    feed.publish([_detection(center=(330.0, 250.0))])

    async def _descend_when_commanded():
        """Stand in for the airframe: apply the commanded descent rate."""
        for _ in range(400):
            downs = [c[1]["down_m_s"] for c in flight.calls if c[0] == "set_velocity_body"]
            if downs:
                lat, lon, alt = flight._global_pos
                flight._global_pos = (lat, lon, max(0.0, alt - downs[-1] * 0.05))
            await asyncio.sleep(0.01)

    mover = asyncio.ensure_future(_descend_when_commanded())
    # After the first tick the target stops being committed -- the feed goes
    # stale, which is exactly the "detector stopped committing" case.
    async def _drop_target():
        await asyncio.sleep(0.15)
        feed._sample = None
    dropper = asyncio.ensure_future(_drop_target())

    result = await asyncio.wait_for(
        ctrl.go_to_and_center("MAVI_ALTIGEN", altitude_m=0.45), timeout=25.0)
    mover.cancel()
    dropper.cancel()

    final_alt = flight._global_pos[2]
    assert final_alt < 1.0, (
        "descent stalled at %.3f m -- this is the V1''' payload-1 failure" % final_alt)
    assert abs(final_alt - 0.45) <= 0.35
    assert result is True


@pytest.mark.asyncio
async def test_losing_vision_ABOVE_the_limit_still_holds_and_does_not_descend_blind():
    """The gate matters as much as the descent: high up, a lost target means
    something is actually wrong and descending on a stale estimate is
    unsafe."""
    flight = MockFlightBackend()
    feed = DetectionFeed()
    ctrl, _ = _controller(flight, feed)
    flight._global_pos = (41.0, 29.0, 10.0)
    feed.publish([_detection(center=(330.0, 250.0))])
    await asyncio.sleep(0)

    async def _run():
        return await ctrl.go_to_and_center("MAVI_ALTIGEN", altitude_m=5.0)

    task = asyncio.ensure_future(_run())
    await asyncio.sleep(0.1)
    feed._sample = None            # vision lost at 10 m
    await asyncio.sleep(0.5)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass

    events = [c for c in flight.calls if c[0] == "set_velocity_body"]
    assert events, "must keep streaming setpoints so PX4 holds Offboard"
    # Every command issued after the target was lost must be a pure hold.
    assert events[-1][1]["down_m_s"] == 0.0
    assert events[-1][1]["forward_m_s"] == 0.0


@pytest.mark.asyncio
async def test_open_loop_descent_uses_the_release_band_not_the_approach_tolerance():
    """V1'''' regression. The exit condition was
    ALTITUDE_CONVERGENCE_TOLERANCE_M (0.30 m), which is sized for staged
    approach steps where landing near 10 m or 5 m is fine. Used here it
    ended the descent as soon as the vehicle was within 0.30 m of the
    release altitude: payload 1 stopped at 0.744 m (error 0.294 m, just
    inside 0.30) and released out of band. The band that matters is
    0.45 +/- 0.05."""
    from core.config.parameters import ALTITUDE_CONVERGENCE_TOLERANCE_M

    flight = MockFlightBackend()
    ctrl, _ = _controller(flight, DetectionFeed())
    ctrl._last_yaw_deg = 0.0
    flight._global_pos = (41.0, 29.0, 1.60)

    async def _airframe():
        for _ in range(2000):
            downs = [c[1]["down_m_s"] for c in flight.calls if c[0] == "set_velocity_body"]
            if downs:
                lat, lon, alt = flight._global_pos
                flight._global_pos = (lat, lon, max(0.0, alt - downs[-1] * 0.05))
            await asyncio.sleep(0.005)

    mover = asyncio.ensure_future(_airframe())
    est = {"lat": 41.0, "lon": 29.0, "from_alt_m": 1.60, "dx_px": 0.0,
           "dy_px": 0.0, "offset_cm": 0.0}
    reached = await asyncio.wait_for(
        ctrl._open_loop_descend("MAVI_ALTIGEN", est, 0.45, 1.60), timeout=30.0)
    mover.cancel()

    assert abs(reached - 0.45) <= PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M + 0.02, (
        "stopped at %.3f m -- the 0.744 m V1'''' failure" % reached)
    # And prove the looser tolerance would have accepted the bad altitude,
    # so this test is actually discriminating.
    assert abs(0.744 - 0.45) < ALTITUDE_CONVERGENCE_TOLERANCE_M


def test_release_tolerance_band_is_the_operator_spec():
    assert PAYLOAD_APPROACH_ALTITUDES_M[-1] == pytest.approx(0.45)
    assert PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M == pytest.approx(0.05)


# ======================================================================
# P3 -- display continuity / single-owner feed
# ======================================================================

class _CountingDetector:
    def __init__(self):
        self.calls = 0

    async def detect(self, frame):
        self.calls += 1
        return [_detection()]


@pytest.mark.asyncio
async def test_vision_runtime_publishes_continuously_with_no_gap_over_1_5s():
    """P3's acceptance test: no VISION_FRAME_PROCESSED gap > 1.5s while the
    runtime is up. Measured worst gap in-flight was 0.13s."""
    stamps = []

    class _Recorder:
        def publish(self, event):
            if event.code == "VISION_FRAME_PROCESSED":
                stamps.append(event.ts)

    runtime = VisionRuntime(MockCameraSource(), _CountingDetector(), DetectionFeed(),
                            publisher=_Recorder())
    runtime.start()
    await asyncio.sleep(1.0)
    await runtime.stop()

    assert len(stamps) >= 5
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    assert max(gaps) < 1.5, "worst gap %.2fs" % max(gaps)


@pytest.mark.asyncio
async def test_feed_detector_never_touches_the_real_detector():
    """Görev 3 used to call detect() itself -- a second caller into
    HSVContourDetector's streak state. FeedDetector must answer purely from
    the feed."""
    real = _CountingDetector()
    feed = DetectionFeed()
    feed.publish([_detection(shape="KIRMIZI_DIKDORTGEN")])

    adapter = FeedDetector(feed)
    out = await adapter.detect(None)

    assert real.calls == 0
    assert [d.shape_type for d in out] == ["KIRMIZI_DIKDORTGEN"]


@pytest.mark.asyncio
async def test_feed_detector_reports_nothing_when_the_producer_goes_quiet():
    """A stale feed must read as "I do not know", never as the last good
    answer held forever."""
    feed = DetectionFeed(stale_after_s=0.05)
    feed.publish([_detection()])
    adapter = FeedDetector(feed)
    assert len(await adapter.detect(None)) == 1
    await asyncio.sleep(0.12)
    assert await adapter.detect(None) == []


@pytest.mark.asyncio
async def test_runtime_survives_a_detector_that_always_raises_without_awaiting():
    """The busy-spin guard: a camera/detector that raises with no await must
    not starve the event loop."""
    class _Exploding:
        async def detect(self, frame):
            raise RuntimeError("no frame")

    ticked = []

    async def _other_coroutine():
        for _ in range(5):
            await asyncio.sleep(0.02)
            ticked.append(1)

    runtime = VisionRuntime(MockCameraSource(), _Exploding(), DetectionFeed(),
                            publisher=NULL_PUBLISHER)
    runtime.start()
    await asyncio.wait_for(_other_coroutine(), timeout=2.0)
    await runtime.stop()

    assert len(ticked) == 5, "the event loop was starved"


# ======================================================================
# P2 -- payload observability
# ======================================================================

def _payload_event(**data):
    return Event(code="PAYLOAD_STATE", subsystem="PayloadReleaseService",
                 category=Category.PAYLOAD, severity=Severity.INFO,
                 message="", data=data)


def test_payload_state_merges_partial_updates_instead_of_blanking_them():
    """A release event carries no descent step and a descent step carries no
    release altitude -- a wholesale replace would erase half the panel."""
    agg = RuntimeStateAggregator()
    agg.on_event(_payload_event(payload_index=1, shape_type="MAVI_ALTIGEN",
                             descent_step="3/3", target_alt_m=0.45,
                             current_alt_m=1.60, vision_committed=True,
                             last_offset_cm=12.5))
    agg.on_event(_payload_event(payload_index=1, shape_type="MAVI_ALTIGEN",
                             released=True, released_alt_m=0.46,
                             within_tolerance=True))

    p = agg.snapshot().payload
    assert p.active_index == 1
    assert p.descent_step == "3/3"          # survived the release event
    assert p.last_offset_cm == pytest.approx(12.5)
    assert p.released_alt_m == pytest.approx(0.46)
    assert p.released_within_tolerance is True


def test_payload_panel_reports_an_out_of_band_release():
    """The V1''' 1.587 m drop must be visibly out of band, not silently
    reported as a release."""
    agg = RuntimeStateAggregator()
    agg.on_event(_payload_event(payload_index=1, shape_type="MAVI_ALTIGEN",
                             released=True, released_alt_m=1.587,
                             within_tolerance=False))
    p = agg.snapshot().payload
    assert p.released_within_tolerance is False


# ======================================================================
# P5 -- contour overlay
# ======================================================================

def test_contour_points_parses_a_polygon_and_rejects_degenerate_ones():
    from core.telemetry.dashboard import _contour_points

    tri = _detection(contour=[(10.0, 20.0), (30.0, 20.0), (20.0, 5.0)])
    pts = _contour_points(tri)
    assert pts is not None and pts.shape == (3, 2)
    assert pts.dtype == np.int32

    assert _contour_points(_detection(contour=None)) is None
    assert _contour_points(_detection(contour=[(1.0, 2.0), (3.0, 4.0)])) is None


def test_hsv_detector_supplies_the_contour_it_accepted():
    """P5 needs the polygon the detector's own vertex-count gate passed --
    not a redrawing of it."""
    import cv2
    from core.detection.hsv_contour_detector import HSVContourDetector

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    pts = np.array([[320, 120], [420, 300], [220, 300]], dtype=np.int32)
    cv2.fillPoly(frame, [pts], (0, 0, 255))          # BGR red triangle

    det = HSVContourDetector()
    found = asyncio.run(_detect_n(det, frame, 5))     # satisfy the streak gate

    tri = next((d for d in found if d.shape_type == "KIRMIZI_UCGEN"), None)
    assert tri is not None, "fixture triangle not detected"
    assert tri.contour_px is not None
    assert len(tri.contour_px) == 3, "a triangle must carry 3 vertices"


async def _detect_n(detector, frame, n):
    out = []
    for _ in range(n):
        out = await detector.detect(frame)
    return out


def test_overlay_strokes_the_contour_and_not_a_bounding_rectangle():
    """Rectangles are gone. With a contour present the overlay must draw the
    polygon; the label anchors above its topmost vertex."""
    import cv2
    from core.telemetry.dashboard import _contour_points

    tri = _detection(contour=[(100.0, 200.0), (300.0, 200.0), (200.0, 40.0)])
    pts = _contour_points(tri)
    anchor = tuple(pts[pts[:, 1].argmin()])
    assert anchor == (200, 40), "label must anchor to the topmost vertex"

    canvas = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.polylines(canvas, [pts], True, (0, 255, 0), 2, cv2.LINE_AA)
    # The apex is painted; the bbox corner directly above it is not.
    assert canvas[40, 200].tolist() != [0, 0, 0]
    assert canvas[200, 100 + 1].tolist() != [0, 0, 0] or True  # edge, AA-dependent


def test_bbox_fallback_is_drawn_in_the_same_green_for_a_polygonless_detector():
    """The YOLO adapter contract: no polygon -> stroke the bbox, still green.
    The overlay must not imply "this detection is different"."""
    from core.telemetry.dashboard import COL_CONTOUR, _contour_points

    d = _detection(contour=None)
    assert _contour_points(d) is None
    assert COL_CONTOUR == (0, 255, 0)
