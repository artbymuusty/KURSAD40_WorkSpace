"""
Regression guard for the operator-reported Mission -> Offboard handover
bug: hold_position() used to send exactly ONE zero-velocity setpoint and
return immediately, leaving the caller (CenteringController.hover_and_confirm)
to just asyncio.sleep() for the hover duration with nothing streamed to PX4.
PX4 auto-exits Offboard after ~500ms without a new setpoint -- so the
vehicle was falling out of Offboard mid-hover on every single engagement.
"""
import time
import pytest

from core.telemetry.event_bus import EventBus
from mavsdk_common.mavsdk_backend_base import MavsdkBackendBase


class _FakeOffboard:
    def __init__(self):
        self.velocity_calls = 0
    async def set_velocity_body(self, setpoint):
        self.velocity_calls += 1


class _FakeDrone:
    def __init__(self):
        self.offboard = _FakeOffboard()


@pytest.mark.asyncio
async def test_hold_position_streams_setpoints_for_the_full_duration():
    backend = MavsdkBackendBase("udp://:14540", publisher=EventBus())
    backend.drone = _FakeDrone()

    start = time.time()
    await backend.hold_position(duration_s=0.5)
    elapsed = time.time() - start

    # BUG FIX assertion: this used to be exactly 1, always, regardless of
    # duration_s -- the whole point of the fix. At OFFBOARD_SETPOINT_INTERVAL_S
    # (0.1s) over 0.5s, expect on the order of 4-6 calls, comfortably more than 1.
    assert backend.drone.offboard.velocity_calls >= 4
    assert elapsed >= 0.4  # actually took roughly the requested duration, not an instant return
