"""T1/T2: the translation as its own phase, and the camera lever arm.

T1: folding the mount translation into the descent let the asymptotic
lateral move eat the descent budget. Measured on the Phase 13 flight:
releases at 0.385 m and 0.159 m against a 0.45 +- 0.05 band, and at 0.159 m
the payload's underside was already on the ground, so detaching moved it by
nothing and the release was reported UNCONFIRMED for a full 60 s.

T2: x500_mono_cam_down mounts the camera 0.35 m FORWARD of base_link, but
the pixel back-projection was added to the VEHICLE's GPS. Every frozen
estimate therefore landed 0.35 m aft of the real target. Predicted landing
error 0.35 - 0.10 (the final nudge) = 0.25 m south; measured across four
flights: 0.252, 0.264, 0.290, 0.305 m south.
"""
import math

import pytest

from core.config.parameters import (
    CAMERA_LEVER_ARM_BODY_M, LOW_ALT_OPEN_LOOP_TIMEOUT_S,
    MOUNT_TRANSLATE_BUDGET_S, MOUNT_TRANSLATE_TOLERANCE_M,
    PAYLOAD_EXPECTED_REST_Z_M, PAYLOAD_ON_TARGET_Z_TOLERANCE_M,
    PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M,
)
from core.navigation.centering_controller import CenteringController


# ------------------------------------------------------------------ T2

def test_camera_lever_arm_matches_the_vehicle_sdf():
    """0.35 m forward, 0 lateral -- straight off the mono_cam include pose.
    If this drifts from the SDF the correction silently becomes a new bias."""
    assert CAMERA_LEVER_ARM_BODY_M == (0.35, 0.0)


def test_lever_arm_predicts_the_measured_north_residual():
    """The diagnosis has to be quantitative, not plausible: the correction's
    size must match the error it claims to explain."""
    nudge_forward_m = 0.10
    predicted = CAMERA_LEVER_ARM_BODY_M[0] - nudge_forward_m
    measured = [0.252, 0.264, 0.290, 0.305]
    assert predicted == pytest.approx(0.25)
    assert abs(sum(measured) / len(measured) - predicted) < 0.05


def test_frozen_estimate_places_the_target_ahead_of_the_vehicle():
    """A perfectly centred target sits under the CAMERA, which is ahead of
    the body origin -- so with zero pixel error the estimate must still be
    0.35 m north of the vehicle, not on top of it."""
    c = CenteringController.__new__(CenteringController)
    c._last_yaw_deg = 0.0                      # heading north
    est = c._freeze_target_estimate(0.0, 0.0, 15.0, 1280, 960, 47.0, 8.5)
    assert est is not None
    north_m = (est["lat"] - 47.0) * 111320.0
    east_m = (est["lon"] - 8.5) * 111320.0 * math.cos(math.radians(47.0))
    assert north_m == pytest.approx(CAMERA_LEVER_ARM_BODY_M[0], abs=0.01)
    assert east_m == pytest.approx(0.0, abs=0.01)


def test_lever_arm_rotates_with_heading():
    """It is a BODY offset. Facing east, the same lever arm must move the
    estimate east, not north -- getting this wrong would trade a fixed bias
    for a heading-dependent one, which is worse."""
    c = CenteringController.__new__(CenteringController)
    c._last_yaw_deg = 90.0
    est = c._freeze_target_estimate(0.0, 0.0, 15.0, 1280, 960, 47.0, 8.5)
    north_m = (est["lat"] - 47.0) * 111320.0
    east_m = (est["lon"] - 8.5) * 111320.0 * math.cos(math.radians(47.0))
    assert north_m == pytest.approx(0.0, abs=0.02)
    assert east_m == pytest.approx(CAMERA_LEVER_ARM_BODY_M[0], abs=0.02)


# ------------------------------------------------------------------ T1

def test_translation_budget_cannot_consume_the_descent():
    """The whole point: two separate budgets. If the translation could still
    run inside the descent's 20 s, the 0.385/0.159 m releases come back."""
    assert MOUNT_TRANSLATE_BUDGET_S > 0
    assert MOUNT_TRANSLATE_BUDGET_S < LOW_ALT_OPEN_LOOP_TIMEOUT_S


def test_translation_tolerance_is_tighter_than_the_offset_goal():
    """Goal is a sub-10 cm final offset, so the hold itself must be tighter."""
    assert MOUNT_TRANSLATE_TOLERANCE_M <= 0.05


@pytest.mark.asyncio
async def test_translation_holds_altitude_and_stops_inside_tolerance():
    """It must command zero vertical rate throughout -- descending during
    the translation is exactly what broke the release band -- and it must
    stop once it is inside tolerance rather than burning its budget."""
    c = CenteringController.__new__(CenteringController)
    c._last_yaw_deg = 0.0
    c.kp_horizontal = 0.5
    sent = []
    published = []

    class _Flight:
        def __init__(self):
            self.north = -0.28      # start 0.28 m short of the hold point

        async def get_global_position(self):
            return (47.0 + self.north / 111320.0, 8.5, 0.45)

    flight = _Flight()
    c.flight = flight
    c.publisher = type("P", (), {"publish": lambda self, e: published.append(e)})()

    async def _send(fwd, right, down, immediate_stop=True):
        sent.append((fwd, right, down))
        flight.north += fwd * 0.1          # integrate the commanded move
        return (fwd, right, down)

    c._send_setpoint = _send
    residual = await c._mount_translate("MAVI_ALTIGEN", {"lat": 47.0, "lon": 8.5})

    assert all(down == 0.0 for _, _, down in sent), "translation must hold altitude"
    assert residual <= MOUNT_TRANSLATE_TOLERANCE_M
    done = [e for e in published if e.code == "MOUNT_TRANSLATE_DONE"]
    assert len(done) == 1
    assert done[0].data["converged"] is True
    assert done[0].data["residual_cm"] <= MOUNT_TRANSLATE_TOLERANCE_M * 100


@pytest.mark.asyncio
async def test_translation_gives_up_on_budget_rather_than_hanging():
    """A translation that cannot converge must hand over with its residual
    logged, never stall the release."""
    c = CenteringController.__new__(CenteringController)
    c._last_yaw_deg = 0.0
    c.kp_horizontal = 0.5
    published = []

    class _StuckFlight:
        async def get_global_position(self):
            return (47.0 + 5.0 / 111320.0, 8.5, 0.45)   # never moves

    c.flight = _StuckFlight()
    c.publisher = type("P", (), {"publish": lambda self, e: published.append(e)})()

    async def _send(fwd, right, down, immediate_stop=True):
        return (fwd, right, down)

    c._send_setpoint = _send
    residual = await c._mount_translate("MAVI_ALTIGEN", {"lat": 47.0, "lon": 8.5})

    assert residual > MOUNT_TRANSLATE_TOLERANCE_M
    done = [e for e in published if e.code == "MOUNT_TRANSLATE_DONE"]
    assert done and done[0].data["converged"] is False


# ------------------------------------------- low release must still confirm

def test_a_payload_already_at_rest_height_counts_as_released():
    """Phase 13 payload 2: released at 0.159 m with its underside on the
    ground, so detaching moved it by nothing and separation-by-falling could
    never fire. A body sitting at its rest height is released."""
    from gz_system.gz_payload_actuator import GzPayloadActuator

    class _Monitor:
        def __init__(self, z):
            self.z = z

        def get(self, name):
            return (0.0, 0.0, self.z)

    act = GzPayloadActuator("svc", pose_monitor=_Monitor(PAYLOAD_EXPECTED_REST_Z_M))
    assert act._at_rest_height("red") is True

    # And an attached payload at a normal release altitude must NOT qualify.
    attached_z = 0.45 + 0.024
    act_high = GzPayloadActuator("svc", pose_monitor=_Monitor(attached_z))
    assert act_high._at_rest_height("red") is False
    assert attached_z - PAYLOAD_EXPECTED_REST_Z_M > PAYLOAD_ON_TARGET_Z_TOLERANCE_M * 5


def test_release_band_is_unchanged_by_all_of_this():
    assert PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M == pytest.approx(0.05)
