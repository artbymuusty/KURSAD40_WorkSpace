import time
from core.mission.blocking import BlockingKind
from core.mission.context import MissionContext
from core.mission.phase import MissionPhase
from core.telemetry.event_bus import EventBus


def test_transition_updates_current_phase_and_publishes_event():
    bus = EventBus()
    received = []
    bus.subscribe(received.append)
    ctx = MissionContext(publisher=bus, mission_id="m1")

    ctx.transition_to(MissionPhase.CONNECTING, reason="test")

    assert ctx.current_phase == MissionPhase.CONNECTING
    assert received[-1].code == "MISSION_PHASE_CHANGED"
    assert received[-1].data["to_phase"] == "CONNECTING"


def test_transition_clears_any_active_blocking_state():
    ctx = MissionContext()
    ctx.set_blocking("WAITING_X", "SubsystemX", BlockingKind.BLOCKING_WAIT, timeout_s=5)
    assert ctx.blocking is not None

    ctx.transition_to(MissionPhase.SEARCHING)

    assert ctx.blocking is None


def test_set_blocking_then_clear_round_trip():
    bus = EventBus()
    received = []
    bus.subscribe(received.append)
    ctx = MissionContext(publisher=bus)

    ctx.set_blocking("WAITING_Y", "SubsystemY", BlockingKind.EXPECTED_WAIT)
    assert ctx.blocking.waiting_on == "WAITING_Y"
    assert ctx.blocking.kind == BlockingKind.EXPECTED_WAIT

    ctx.clear_blocking()
    assert ctx.blocking is None
    codes = [e.code for e in received]
    assert "BLOCKING_STATE_ENTERED" in codes
    assert "BLOCKING_STATE_CLEARED" in codes


def test_blocking_state_reports_remaining_and_overdue():
    ctx = MissionContext()
    ctx.set_blocking("WAITING_Z", "SubsystemZ", BlockingKind.BLOCKING_WAIT, timeout_s=0.05)
    assert ctx.blocking.is_overdue() is False
    time.sleep(0.08)
    assert ctx.blocking.is_overdue() is True
    assert ctx.blocking.remaining_s() == 0.0


def test_snapshot_data_reflects_timeout_budget():
    ctx = MissionContext(mission_id="m2", timeout_budget_s=10.0)
    snap = ctx.snapshot_data()
    assert snap.mission_id == "m2"
    assert snap.timeout_budget_s == 10.0
    assert snap.timeout_remaining_s is not None
    assert snap.timeout_remaining_s <= 10.0
