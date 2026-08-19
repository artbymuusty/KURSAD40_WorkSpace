"""
Operator request (2026-08-16), visualization only:
  1. the target->crosshair vector is YELLOW while centering and BLACK once
     the CenteringController reports converged, reverting to yellow if lock
     is lost;
  2. a "<TARGET> d=X.X m" ground-distance label at the lower-right of that
     vector, derived from pixel offset + AGL + the camera's own FOV.

Neither may introduce a second definition of "locked" or a hardcoded lens:
the lock flag comes verbatim from CENTERING_STEP, and the intrinsics come
from the mono_cam SDF.
"""
import math

import numpy as np
import pytest

from core.detection.camera_intrinsics import (
    CameraIntrinsics, default_camera_intrinsics, load_camera_intrinsics,
)
from core.detection.types import Detection
from core.telemetry.aggregator import RuntimeStateAggregator
from core.telemetry.events import Category, Event, Severity
from core.telemetry.frame_channel import FrameSample
from core.telemetry.dashboard import COL_LOCKED, COL_VECTOR, MissionOpsDashboard


# ----------------------------------------------------------------------
# intrinsics
# ----------------------------------------------------------------------
def test_intrinsics_come_from_the_model_sdf_not_a_constant():
    intrinsics = load_camera_intrinsics()
    assert intrinsics is not None, "mono_cam SDF must be discoverable from the flight stack"
    assert intrinsics.source.endswith("mono_cam/model.sdf")
    # Whatever the SDF says, focal length must follow from it.
    assert intrinsics.focal_px == pytest.approx(
        (intrinsics.width_px / 2.0) / math.tan(intrinsics.horizontal_fov_rad / 2.0))


def test_ground_distance_matches_the_pinhole_geometry():
    """A target at the horizontal edge of the frame must sit at
    AGL * tan(hfov/2) on the ground -- the closed form the projection
    reduces to, independent of the focal-length arithmetic."""
    i = CameraIntrinsics(horizontal_fov_rad=1.74, width_px=1280, height_px=960, source="test")
    agl = 15.0

    edge = i.ground_distance_m(i.width_px / 2.0, 0.0, agl)
    assert edge == pytest.approx(agl * math.tan(i.horizontal_fov_rad / 2.0), rel=1e-9)

    # Dead-centre target is directly under the vehicle.
    assert i.ground_distance_m(0.0, 0.0, agl) == pytest.approx(0.0)
    # Linear in altitude.
    assert i.ground_distance_m(100.0, 0.0, 2 * agl) == pytest.approx(
        2 * i.ground_distance_m(100.0, 0.0, agl))


def test_no_distance_is_reported_without_a_usable_altitude():
    """On the ground / before the first fix, omit the label rather than
    render a fabricated 0.0 m."""
    i = CameraIntrinsics(1.74, 1280, 960, "test")
    assert i.ground_distance_m(100.0, 0.0, 0.0) is None
    assert i.ground_distance_m(100.0, 0.0, None) is None


def test_intrinsics_rescale_with_frame_size_but_keep_fov():
    i = CameraIntrinsics(1.74, 1280, 960, "test")
    half = i.scaled_to(640, 480)
    assert half.horizontal_fov_rad == i.horizontal_fov_rad
    assert half.focal_px == pytest.approx(i.focal_px / 2)
    # Same physical point -> same ground distance at half resolution.
    assert half.ground_distance_m(50.0, 0.0, 15.0) == pytest.approx(
        i.ground_distance_m(100.0, 0.0, 15.0))


# ----------------------------------------------------------------------
# lock state plumbing: CENTERING_STEP -> snapshot
# ----------------------------------------------------------------------
def _step(agg, shape, converged, dx=40.0, dy=30.0, dist=2.5):
    agg.on_event(Event(code="CENTERING_STEP", subsystem="CenteringController",
                       category=Category.NAVIGATION, severity=Severity.DEBUG,
                       data={"shape_type": shape, "converged": converged, "attempt": 3,
                             "max_attempts": 150, "dx_px": dx, "dy_px": dy,
                             "target_px": [360.0, 270.0], "center_px": [320.0, 240.0],
                             "ground_distance_m": dist}))


def test_lock_flag_is_carried_through_verbatim_and_can_be_lost():
    agg = RuntimeStateAggregator()

    agg.on_event(Event(code="CENTERING_STARTED", subsystem="CenteringController",
                       category=Category.NAVIGATION, data={"shape_type": "KIRMIZI_UCGEN"}))
    assert agg.snapshot().centering.converged is False, "a new pursuit starts unlocked"

    _step(agg, "KIRMIZI_UCGEN", converged=False)
    assert agg.snapshot().centering.converged is False

    _step(agg, "KIRMIZI_UCGEN", converged=True)
    snap = agg.snapshot()
    assert snap.centering.converged is True
    assert snap.centering.ground_distance_m == 2.5
    assert snap.centering.center_px == (320.0, 240.0)

    # Lock lost (e.g. a payload-approach re-centre at a lower altitude).
    _step(agg, "KIRMIZI_UCGEN", converged=False)
    assert agg.snapshot().centering.converged is False


def test_centering_timeout_leaves_it_unlocked_and_convergence_leaves_it_locked():
    agg = RuntimeStateAggregator()
    _step(agg, "MAVI_ALTIGEN", converged=False)
    agg.on_event(Event(code="CENTERING_TIMED_OUT", subsystem="CenteringController",
                       category=Category.NAVIGATION, data={"shape_type": "MAVI_ALTIGEN"}))
    assert agg.snapshot().centering.converged is False
    assert agg.snapshot().centering.active is False

    agg.on_event(Event(code="CENTERING_CONVERGED", subsystem="CenteringController",
                       category=Category.NAVIGATION, data={"shape_type": "MAVI_ALTIGEN"}))
    # Stays locked through HOVER_CONFIRM / GPS_SAVE, when no further steps
    # are published at all.
    assert agg.snapshot().centering.converged is True


# ----------------------------------------------------------------------
# overlay rendering
# ----------------------------------------------------------------------
def _panel(snap, detection):
    dash = MissionOpsDashboard(RuntimeStateAggregator())
    # Mid-grey, not black: the lock colour IS black, so a black background
    # would make "the vector turned black" indistinguishable from "nothing
    # was drawn".
    frame = np.full((480, 640, 3), 128, dtype=np.uint8)
    sample = FrameSample(frame_bgr=frame, detections=[detection])
    return dash._build_camera_panel(sample, snap)


def _detection(shape="KIRMIZI_UCGEN"):
    return Detection(shape_type=shape, confidence=0.9,
                     center_px=(420.0, 300.0), bbox_px=(400, 280, 440, 320))


def _colour_pixels(img, color, tol=12):
    """Count pixels close to `color`. A tolerance is required because every
    overlay stroke is drawn with cv2.LINE_AA, which blends edge pixels."""
    delta = np.abs(img.astype(np.int16) - np.array(color, dtype=np.int16))
    return int(np.count_nonzero(np.all(delta <= tol, axis=-1)))


@pytest.mark.parametrize("converged", [False, True])
def test_vector_colour_follows_the_controllers_lock_flag(converged):
    agg = RuntimeStateAggregator()
    _step(agg, "KIRMIZI_UCGEN", converged=converged)
    snap = agg.snapshot()
    snap.vehicle.position = (47.0, 8.0, 15.0)

    img = _panel(snap, _detection())
    yellow = _colour_pixels(img, COL_VECTOR)
    black = _colour_pixels(img, COL_LOCKED)

    if converged:
        assert yellow == 0, "a locked vector must not still be drawn yellow"
        assert black > 0, "the locked vector must be drawn black"
    else:
        assert yellow > 0, "an unlocked vector must be drawn yellow"


def test_a_different_shape_does_not_inherit_another_targets_lock():
    agg = RuntimeStateAggregator()
    _step(agg, "KIRMIZI_UCGEN", converged=True)
    snap = agg.snapshot()
    snap.vehicle.position = (47.0, 8.0, 15.0)

    img = _panel(snap, _detection("MAVI_ALTIGEN"))

    assert _colour_pixels(img, COL_VECTOR) > 0


def test_crosshair_is_drawn_at_the_centre_the_controller_reported():
    """Not the frame's integer midpoint -- the exact centre
    CenteringController computed and published."""
    agg = RuntimeStateAggregator()
    _step(agg, "KIRMIZI_UCGEN", converged=False)
    snap = agg.snapshot()
    snap.centering.center_px = (100.0, 90.0)  # deliberately not (320, 240)
    snap.vehicle.position = (47.0, 8.0, 15.0)

    img = _panel(snap, _detection())

    # Sample the crosshair's LEFT arm: the target is down-and-right of the
    # centre, so the vector line never overdraws it.
    assert _colour_pixels(img[90:91, 85:96], (255, 255, 255)) > 0, \
        "crosshair must sit on the reported centre"
    assert _colour_pixels(img[240:241, 305:316], (255, 255, 255)) == 0, \
        "and not on the frame midpoint"


def test_distance_label_uses_the_value_published_in_centering_step():
    """The label and the log must never disagree, so the active target's
    number is reused verbatim rather than recomputed."""
    agg = RuntimeStateAggregator()
    _step(agg, "KIRMIZI_UCGEN", converged=False, dist=7.3)
    snap = agg.snapshot()
    snap.vehicle.position = (47.0, 8.0, 15.0)

    dash = MissionOpsDashboard(RuntimeStateAggregator())
    drawn = []

    class _Spy:
        @staticmethod
        def getTextSize(text, *a):
            drawn.append(text)
            return (100, 10), 0

    import core.telemetry.dashboard as dashboard_mod
    real = dashboard_mod.cv2.getTextSize
    dashboard_mod.cv2.getTextSize = _Spy.getTextSize
    try:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        dash._build_camera_panel(FrameSample(frame_bgr=frame, detections=[_detection()]), snap)
    finally:
        dashboard_mod.cv2.getTextSize = real

    assert "KIRMIZI_UCGEN d=7.3 m" in drawn


def test_label_is_omitted_when_altitude_is_unusable():
    """No AGL and no published distance -> no fabricated number."""
    agg = RuntimeStateAggregator()
    snap = agg.snapshot()
    snap.vehicle.position = None

    dash = MissionOpsDashboard(RuntimeStateAggregator())
    drawn = []
    import core.telemetry.dashboard as dashboard_mod
    real = dashboard_mod.cv2.getTextSize
    dashboard_mod.cv2.getTextSize = lambda text, *a: (drawn.append(text), ((100, 10), 0))[1]
    try:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        dash._build_camera_panel(FrameSample(frame_bgr=frame, detections=[_detection()]), snap)
    finally:
        dashboard_mod.cv2.getTextSize = real

    assert not any("d=" in t for t in drawn)


def test_default_intrinsics_are_shared_by_controller_and_dashboard():
    """One parse, one source of truth -- so the CENTERING_STEP number and
    the rendered label are computed identically."""
    assert default_camera_intrinsics() is default_camera_intrinsics()


def test_tolerance_ellipse_is_a_true_ellipse_scaled_to_the_frame():
    """Operator request (2026-08-17): the tolerance region is +/-0.01
    normalized PER AXIS and the axes normalize by different half-extents,
    so it is an ellipse (6.4 x 4.8 px semi-axes at 1280x960) -- never a
    circle approximation."""
    from core.config.parameters import CENTERING_TOLERANCE_X_NORM, CENTERING_TOLERANCE_Y_NORM
    import core.telemetry.dashboard as dashboard_mod

    calls = []
    real = dashboard_mod.cv2.ellipse
    dashboard_mod.cv2.ellipse = lambda img, c, axes, *a, **kw: calls.append((c, axes))
    try:
        agg = RuntimeAggregator = RuntimeStateAggregator()
        snap = agg.snapshot()
        snap.vehicle.position = (47.0, 8.0, 15.0)
        dash = MissionOpsDashboard(RuntimeStateAggregator())
        frame = np.full((960, 1280, 3), 128, dtype=np.uint8)
        dash._build_camera_panel(FrameSample(frame_bgr=frame, detections=[]), snap)
    finally:
        dashboard_mod.cv2.ellipse = real

    assert calls, "tolerance ellipse must be drawn"
    centre, (semi_x, semi_y) = calls[0]
    assert centre == (640, 480)
    assert semi_x == round(CENTERING_TOLERANCE_X_NORM * 640)   # 6.4 -> 6
    assert semi_y == round(CENTERING_TOLERANCE_Y_NORM * 480)   # 4.8 -> 5
    assert semi_x != semi_y, "must be an ellipse, not a circle"


def test_tolerance_ellipse_scales_with_frame_resolution():
    import core.telemetry.dashboard as dashboard_mod
    calls = []
    real = dashboard_mod.cv2.ellipse
    dashboard_mod.cv2.ellipse = lambda img, c, axes, *a, **kw: calls.append((c, axes))
    try:
        snap = RuntimeStateAggregator().snapshot()
        snap.vehicle.position = (47.0, 8.0, 15.0)
        dash = MissionOpsDashboard(RuntimeStateAggregator())
        frame = np.full((480, 640, 3), 128, dtype=np.uint8)   # half resolution
        dash._build_camera_panel(FrameSample(frame_bgr=frame, detections=[]), snap)
    finally:
        dashboard_mod.cv2.ellipse = real

    centre, (semi_x, semi_y) = calls[0]
    assert centre == (320, 240)
    assert (semi_x, semi_y) == (3, 2)   # 0.01*320=3.2, 0.01*240=2.4
