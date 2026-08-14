"""
Regression guard for the bug found and fixed this engagement (ADR-005 §5):
MavsdkBackendBase.upload_mission() used to discard its `waypoints` argument
and always upload an empty MissionPlan([]), which made PX4 report the
mission "finished" (0==0) before ever flying anywhere -- the root cause of
"takes off, then just hovers, dashboard never opens."

These tests exercise the real upload_mission()/_to_mission_items() logic
against a fake `drone.mission` (no real MAVSDK connection needed) so this
exact regression can never silently return.
"""
import pytest
from core.telemetry.event_bus import EventBus
from mavsdk_common.mavsdk_backend_base import MavsdkBackendBase


class _FakeMissionPlan:
    def __init__(self, mission_items):
        self.mission_items = mission_items


class _FakeMission:
    """Simulates PX4's mission plugin: what you upload is what you get back
    on download -- the exact round-trip the fix's assertion depends on."""
    def __init__(self):
        self.uploaded_plan = None

    async def upload_mission(self, plan):
        self.uploaded_plan = plan

    async def download_mission(self):
        return self.uploaded_plan


class _FakeDrone:
    def __init__(self):
        self.mission = _FakeMission()


WAYPOINTS = [
    (41.0, 29.0, 15.0),
    (41.001, 29.001, 15.0),
    (41.001, 29.0, 15.0),
]


@pytest.mark.asyncio
async def test_to_mission_items_builds_one_item_per_waypoint():
    items = MavsdkBackendBase._to_mission_items(WAYPOINTS)
    assert len(items) == len(WAYPOINTS)
    assert items[0].latitude_deg == 41.0
    assert items[0].longitude_deg == 29.0
    assert items[0].relative_altitude_m == 15.0


@pytest.mark.asyncio
async def test_upload_mission_actually_uploads_all_waypoints_not_empty():
    bus = EventBus()
    received = []
    bus.subscribe(received.append)

    backend = MavsdkBackendBase("udp://:14540", publisher=bus)
    backend.drone = _FakeDrone()  # bypass real MAVSDK connection

    await backend.upload_mission(WAYPOINTS)

    # This is the exact assertion that would have caught the original bug:
    # the fake PX4's uploaded plan must contain the real waypoint count, not 0.
    assert len(backend.drone.mission.uploaded_plan.mission_items) == len(WAYPOINTS)

    codes = [e.code for e in received]
    assert "MISSION_UPLOAD_REQUESTED" in codes
    assert "MISSION_UPLOAD_CONFIRMED" in codes
    assert "MISSION_UPLOAD_MISMATCH" not in codes


@pytest.mark.asyncio
async def test_upload_mission_raises_and_reports_on_mismatch():
    bus = EventBus()
    received = []
    bus.subscribe(received.append)

    backend = MavsdkBackendBase("udp://:14540", publisher=bus)
    backend.drone = _FakeDrone()

    # Simulate the exact historical bug: PX4/the plugin silently drops items.
    real_upload = backend.drone.mission.upload_mission
    async def truncating_upload(plan):
        await real_upload(_FakeMissionPlan([]))
    backend.drone.mission.upload_mission = truncating_upload

    with pytest.raises(RuntimeError, match="MISSION_UPLOAD_MISMATCH"):
        await backend.upload_mission(WAYPOINTS)

    codes = [e.code for e in received]
    assert "MISSION_UPLOAD_MISMATCH" in codes
    assert "MISSION_UPLOAD_CONFIRMED" not in codes
