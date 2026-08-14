"""
Regression guard for the operator-reported Mission -> Offboard handover
bug. go_to_and_center() previously computed pixel error and never called
set_velocity_body() at all -- nothing ever drove the vehicle toward a
target regardless of Offboard being active. switch_to_offboard() returned
None unconditionally, with no confirmation PX4 actually entered OFFBOARD.

Also covers the operator's 2026-08-13 precision revision: convergence on
normalized +/-0.01 tolerance (not the old raw-pixel 20px threshold), real
altitude control (altitude_m was previously a dead parameter), and the new
nudge_forward()/climb_to_altitude() primitives used by the staged payload
approach (see PayloadReleaseService).
"""
import pytest

from mocks.mock_flight_backend import MockFlightBackend
from mocks.mock_camera_source import MockCameraSource

from core.detection.types import Detection
from core.navigation.centering_controller import CenteringController


class _FixedDetector:
    """Always reports the same detection, at a fixed pixel offset from
    center -- lets a test assert set_velocity_body is called repeatedly
    (continuous streaming) without needing real flight physics."""
    def __init__(self, center_px):
        self.center_px = center_px
    async def detect(self, frame):
        return [Detection(shape_type="MAVI_ALTIGEN", confidence=0.9,
                          center_px=self.center_px, bbox_px=(0, 0, 10, 10))]


@pytest.mark.asyncio
async def test_go_to_and_center_actually_commands_velocity_when_off_center():
    flight = MockFlightBackend()
    camera = MockCameraSource()  # 640x480 -> center (320, 240)
    detector = _FixedDetector(center_px=(500.0, 240.0))  # far right of center, never "arrives"
    controller = CenteringController(flight, detector, camera)
    controller.lateral_timeout_s = 2.0  # keep this "never converges" test fast

    converged = await controller.go_to_and_center("MAVI_ALTIGEN")

    velocity_calls = [c for c in flight.calls if c[0] == 'set_velocity_body']
    assert converged is False  # never reaches the +/-0.01 normalized tolerance in this test
    # BUG FIX assertion: this used to be zero calls, always, regardless of
    # error magnitude -- the whole point of the fix.
    assert len(velocity_calls) > 5
    # A real command (not just a zero/no-op) must have been sent while off-center.
    assert any(c[1]['right_m_s'] != 0.0 for c in velocity_calls[:-1])
    # Final call must be an explicit stop, converged or not.
    assert velocity_calls[-1][1] == {'forward_m_s': 0.0, 'right_m_s': 0.0, 'down_m_s': 0.0, 'yaw_rate_deg_s': 0.0}


@pytest.mark.asyncio
async def test_go_to_and_center_converges_and_stops_when_already_centered():
    flight = MockFlightBackend()  # _global_pos altitude fixed at 15.0 == MISSION_ALTITUDE_M default
    camera = MockCameraSource()  # 640x480 -> center (320, 240)
    # dx=2/320=0.00625, dy=1/240=0.00417 -- within the +/-0.01 normalized tolerance.
    detector = _FixedDetector(center_px=(322.0, 241.0))
    controller = CenteringController(flight, detector, camera)

    converged = await controller.go_to_and_center("MAVI_ALTIGEN")

    assert converged is True
    velocity_calls = [c for c in flight.calls if c[0] == 'set_velocity_body']
    # Still sends the explicit final stop command even on immediate convergence.
    assert velocity_calls[-1][1] == {'forward_m_s': 0.0, 'right_m_s': 0.0, 'down_m_s': 0.0, 'yaw_rate_deg_s': 0.0}


@pytest.mark.asyncio
async def test_go_to_and_center_precision_tightened_old_20px_point_no_longer_converges():
    """Operator revision (2026-08-13): a point that was 'centered' under the
    old raw-pixel 20px threshold (dx=15, dy=10) must NOT converge under the
    new +/-0.01 normalized tolerance (dx_norm=15/320=0.047, dy_norm=10/240=0.042)
    -- this is a deliberate precision tightening, not a regression."""
    flight = MockFlightBackend()
    camera = MockCameraSource()
    detector = _FixedDetector(center_px=(335.0, 250.0))
    controller = CenteringController(flight, detector, camera)
    controller.lateral_timeout_s = 2.0  # keep this "never converges" test fast

    converged = await controller.go_to_and_center("MAVI_ALTIGEN")

    assert converged is False


@pytest.mark.asyncio
async def test_go_to_and_center_descends_to_target_altitude():
    """altitude_m was previously a dead parameter -- this is the staged
    payload approach's core primitive (15m -> 10m -> 5m -> 0.30m, each step
    re-centered): the loop must actually command descent, not just hold
    lateral position at whatever altitude it started."""
    flight = MockFlightBackend()  # starts at 15.0m (MockFlightBackend default)
    camera = MockCameraSource()
    detector = _FixedDetector(center_px=(320.0, 240.0))  # dead-center, lateral already converged

    controller = CenteringController(flight, detector, camera)

    converged = await controller.go_to_and_center("MAVI_ALTIGEN", altitude_m=10.0)

    assert converged is True
    _, _, final_alt = flight._global_pos
    assert abs(final_alt - 10.0) < 0.3  # ALTITUDE_CONVERGENCE_TOLERANCE_M
    velocity_calls = [c for c in flight.calls if c[0] == 'set_velocity_body']
    # A real descent command (positive down_m_s) must have been sent while above target.
    assert any(c[1]['down_m_s'] > 0.0 for c in velocity_calls[:-1])


@pytest.mark.asyncio
async def test_nudge_forward_streams_then_stops():
    flight = MockFlightBackend()
    controller = CenteringController(flight, detector=None, camera=None)

    await controller.nudge_forward(distance_m=0.10, speed_m_s=0.5)

    velocity_calls = [c for c in flight.calls if c[0] == 'set_velocity_body']
    assert len(velocity_calls) >= 1
    assert any(c[1]['forward_m_s'] == 0.5 for c in velocity_calls[:-1])
    assert velocity_calls[-1][1] == {'forward_m_s': 0.0, 'right_m_s': 0.0, 'down_m_s': 0.0, 'yaw_rate_deg_s': 0.0}


@pytest.mark.asyncio
async def test_climb_to_altitude_is_vision_independent_and_reaches_target():
    """Used after a payload drop, when the vehicle is close to the ground
    and the just-released shape may no longer be in frame -- must not
    depend on detector/camera at all (unlike go_to_and_center)."""
    flight = MockFlightBackend()
    lat, lon, _ = flight._global_pos
    flight._global_pos = (lat, lon, 0.30)  # simulate post-drop altitude
    controller = CenteringController(flight, detector=None, camera=None)

    converged = await controller.climb_to_altitude(15.0)

    assert converged is True
    _, _, final_alt = flight._global_pos
    assert abs(final_alt - 15.0) < 0.3


@pytest.mark.asyncio
async def test_switch_to_offboard_returns_true_when_px4_confirms_offboard():
    flight = MockFlightBackend()  # start_offboard() sets _flight_mode = "OFFBOARD"
    controller = CenteringController(flight, detector=None, camera=None)

    result = await controller.switch_to_offboard()

    assert result is True
    assert 'switch_to_offboard_from_mission' in [c[0] for c in flight.calls]
    assert 'start_offboard' in [c[0] for c in flight.calls]


@pytest.mark.asyncio
async def test_switch_to_offboard_returns_false_when_px4_never_confirms():
    class _StuckInMissionFlight(MockFlightBackend):
        async def start_offboard(self) -> None:
            self.calls.append(('start_offboard', {}))
            # Simulates PX4 accepting the command with no exception, but
            # never actually reporting OFFBOARD -- exactly the failure mode
            # switch_to_offboard()'s old implementation could never detect.

    flight = _StuckInMissionFlight()
    controller = CenteringController(flight, detector=None, camera=None)

    result = await controller.switch_to_offboard()

    assert result is False


@pytest.mark.asyncio
async def test_switch_to_offboard_returns_false_when_px4_rejects_start():
    class _RejectingFlight(MockFlightBackend):
        async def start_offboard(self) -> None:
            raise RuntimeError("OffboardError: NoSetpointSet")

    flight = _RejectingFlight()
    controller = CenteringController(flight, detector=None, camera=None)

    result = await controller.switch_to_offboard()  # must not raise

    assert result is False
