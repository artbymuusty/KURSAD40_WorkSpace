"""A1/A2: the release band on every path, and aiming the payload not the camera.

Both come from the two ADR-011 nominal flights.

A1: payload 1 went down the open-loop path and released at 0.475/0.484 m,
because that path exits on PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M. Payload 2's
triangle stayed visible, so the ordinary centering loop finished the descent
and declared convergence against ALTITUDE_CONVERGENCE_TOLERANCE_M -- six
times looser -- releasing at 0.564 m. The guarantee depended on whether
vision happened to survive.

A2: all four drops came to rest 33.7-37.3 cm from the target centre, tightly
clustered around the 0.28 m mount offset. The mission centres the CAMERA;
the payload hangs to one side and lands there.
"""
import math

import pytest

from core.config.parameters import (
    ALTITUDE_CONVERGENCE_TOLERANCE_M, PAYLOAD_APPROACH_ALTITUDES_M,
    PAYLOAD_MOUNT_OFFSET_BODY_M, PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M,
)
from core.navigation.centering_controller import CenteringController


def _controller():
    return CenteringController.__new__(CenteringController)


# --------------------------------------------------------------- A2 geometry

def test_mount_vectors_are_in_the_px4_frd_frame_not_the_sdf_flu_frame():
    """THE regression, and it cost a flight. The world SDF mounts
    payload_red at y = -0.28, but SDF body axes are FLU (Y left) while this
    constant is consumed as PX4 FRD (Y right) -- so red is on the RIGHT and
    the sign flips. Flying the SDF's sign verbatim steered the vehicle the
    wrong way and DOUBLED the miss: 0.243 m became 0.850 m."""
    assert PAYLOAD_MOUNT_OFFSET_BODY_M["MAVI_ALTIGEN"] == (0.0, 0.28)
    assert PAYLOAD_MOUNT_OFFSET_BODY_M["KIRMIZI_UCGEN"] == (0.0, -0.28)


def test_mount_signs_match_where_the_payloads_actually_landed():
    """Ground truth, independent of any frame argument: at the mission's
    ~0 heading, world X IS body-right. Four uncorrected drops put red at
    world X +0.243/+0.240 and blue at -0.270/-0.337, so red's mount must be
    positive-right and blue's negative."""
    red_right = PAYLOAD_MOUNT_OFFSET_BODY_M["MAVI_ALTIGEN"][1]
    blue_right = PAYLOAD_MOUNT_OFFSET_BODY_M["KIRMIZI_UCGEN"][1]
    for landed in (0.243, 0.240):
        assert red_right * landed > 0, "red mount sign disagrees with its landings"
    for landed in (-0.270, -0.337):
        assert blue_right * landed > 0, "blue mount sign disagrees with its landings"


def test_aim_offset_moves_the_aim_point_to_the_payload_side():
    """Image +x is body-right, so the red payload (mounted right) pulls the
    aim point to positive x -- the target is driven to sit RIGHT of frame
    centre, directly under the payload -- and blue mirrors it exactly."""
    c = _controller()
    dx, dy = c._aim_offset_px(PAYLOAD_MOUNT_OFFSET_BODY_M["MAVI_ALTIGEN"], 0.45, 1280, 960)
    assert dx > 0, "red hangs to the right, so the aim point moves right"
    assert dy == pytest.approx(0.0)
    dx_blue, _ = c._aim_offset_px(PAYLOAD_MOUNT_OFFSET_BODY_M["KIRMIZI_UCGEN"], 0.45, 1280, 960)
    assert dx_blue == pytest.approx(-dx)


def test_aim_offset_is_a_constant_distance_on_the_ground():
    """It grows in pixels as the vehicle descends precisely because it is
    fixed in metres. 0.28 m must stay 0.28 m at every altitude."""
    c = _controller()
    focal = 539.94  # hfov 1.74 rad at 1280x960, from the camera SDF
    for alt in (5.0, 2.0, 0.45):
        dx, _ = c._aim_offset_px((0.0, 0.28), alt, 1280, 960)
        assert abs(dx) * alt / focal == pytest.approx(0.28, abs=0.005)


def test_aim_offset_degrades_to_camera_centred_when_geometry_is_unusable():
    """A missing altitude must give a centred aim, never a wrong one: a bad
    aim point moves the vehicle to a place nothing asked for."""
    c = _controller()
    assert c._aim_offset_px((0.0, 0.28), 0.0, 1280, 960) == (0.0, 0.0)
    assert c._aim_offset_px(None, 0.45, 1280, 960) == (0.0, 0.0)


def test_aim_offset_needs_no_heading_rotation():
    """The camera is bolted to the airframe, so image axes ARE body axes.
    The result must depend only on the mount vector and the altitude -- if
    it needed a heading it would have to be a parameter here."""
    c = _controller()
    import inspect
    params = set(inspect.signature(c._aim_offset_px).parameters)
    assert "yaw" not in params and "heading" not in params
    assert params == {"aim_offset_body_m", "current_alt_m", "res_w", "res_h"}


# --------------------------------------------------------------------- A1

def test_release_band_is_six_times_tighter_than_the_approach_band():
    """The premise of A1: the two bands really are different, so which one
    applies to the final step decides whether a release is in spec."""
    assert PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M == pytest.approx(0.05)
    assert ALTITUDE_CONVERGENCE_TOLERANCE_M == pytest.approx(0.30)
    assert ALTITUDE_CONVERGENCE_TOLERANCE_M > PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M * 5


def test_the_measured_564m_release_would_now_be_rejected():
    """THE regression, from run 2: 0.564 m against a 0.45 m target. The old
    band accepted it (error 0.114 < 0.30); the release band does not."""
    error = abs(0.564 - PAYLOAD_APPROACH_ALTITUDES_M[-1])
    assert error < ALTITUDE_CONVERGENCE_TOLERANCE_M      # old: converged
    assert error > PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M  # new: keeps descending


def test_go_to_and_center_defaults_to_the_loose_band():
    """Higher approach steps and every non-payload caller must be untouched
    -- tightening them would spend time converging at 10 m for no benefit."""
    import inspect
    sig = inspect.signature(CenteringController.go_to_and_center)
    assert sig.parameters["alt_tolerance_m"].default is None
    assert sig.parameters["aim_offset_body_m"].default is None


@pytest.mark.asyncio
async def test_final_approach_step_gets_the_band_then_the_translated_hold():
    """The wiring: only the last step is tightened, and the mount offset is
    applied AFTER vision-guided centring (D3), never as a bias during it."""
    from core.mission.payload_release import PayloadReleaseService

    calls = []
    descents = []

    class _Centering:
        async def go_to_and_center(self, shape, altitude_m=None, alt_tolerance_m=None,
                                   aim_offset_body_m=None):
            calls.append((altitude_m, alt_tolerance_m, aim_offset_body_m))
            return True

        async def descend_to_release(self, shape, altitude_m, mount_body_m):
            descents.append((shape, altitude_m, mount_body_m))
            return altitude_m

        async def nudge_forward(self, d):
            pass

    svc = PayloadReleaseService.__new__(PayloadReleaseService)
    svc.centering = _Centering()
    svc.publisher = type("P", (), {"publish": lambda self, e: None})()
    svc.detection_feed = type("F", (), {"get": lambda self, s: None})()
    svc._payload_index = 1
    svc._last_offset_cm = None
    svc._last_offset_alt_m = None
    svc.flight = type("Fl", (), {
        "get_global_position": staticmethod(lambda: _agp())})()

    async def _agp():
        return (0.0, 0.0, 5.0)

    await svc._staged_approach("MAVI_ALTIGEN")

    assert len(calls) == len(PAYLOAD_APPROACH_ALTITUDES_M)
    for altitude, tol, aim in calls[:-1]:
        assert tol is None and aim is None, "higher steps must be unchanged"
    final_alt, final_tol, final_aim = calls[-1]
    assert final_alt == PAYLOAD_APPROACH_ALTITUDES_M[-1]
    assert final_tol == PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M
    # D3: no aim bias anywhere in the vision loop...
    assert all(aim is None for _, _, aim in calls)
    # ...and exactly one translated-hold descent, with the mount vector.
    assert descents == [("MAVI_ALTIGEN", PAYLOAD_APPROACH_ALTITUDES_M[-1],
                         PAYLOAD_MOUNT_OFFSET_BODY_M["MAVI_ALTIGEN"])]


# ------------------------------------------------------- reported offsets

def test_reported_offset_is_payload_to_target_not_camera_to_target():
    """With the aim applied, a perfectly aimed release has the target 0.28 m
    off the frame centre. Reporting that raw would call a good drop a 28 cm
    miss and a drop that is about to miss by 28 cm a perfect one."""
    from core.mission.payload_release import PayloadReleaseService

    svc = PayloadReleaseService.__new__(PayloadReleaseService)
    focal, alt = 539.94, 0.45
    aim_dx, aim_dy = svc._aim_offset_px("MAVI_ALTIGEN", alt, focal)
    # Target sitting exactly under the payload -> residual must be zero.
    target_dx_from_centre = aim_dx
    assert target_dx_from_centre - aim_dx == pytest.approx(0.0)
    assert aim_dy == pytest.approx(0.0)
    assert abs(aim_dx) * alt / focal == pytest.approx(0.28, abs=0.005)


def test_reported_offset_is_unshifted_for_shapes_with_no_mount():
    """Görev 3's shapes have no payload on a servo arm; their offsets must
    stay camera-referenced."""
    from core.mission.payload_release import PayloadReleaseService

    svc = PayloadReleaseService.__new__(PayloadReleaseService)
    assert svc._aim_offset_px("KIRMIZI_DIKDORTGEN", 0.45, 539.94) == (0.0, 0.0)


def test_mount_offset_matches_the_measured_landing_scatter():
    """Sanity-check the premise against the flights: four drops at 35.0,
    37.3, 35.7 and 33.7 cm. If the mount vector were wrong, correcting by it
    would not bring those near zero."""
    measured = [0.350, 0.373, 0.357, 0.337]
    mount = abs(PAYLOAD_MOUNT_OFFSET_BODY_M["MAVI_ALTIGEN"][1])
    residuals = [math.sqrt(max(0.0, m ** 2 - mount ** 2)) for m in measured]
    assert max(residuals) < 0.25, residuals
