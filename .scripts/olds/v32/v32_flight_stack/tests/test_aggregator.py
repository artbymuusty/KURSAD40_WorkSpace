from core.mission.phase import MissionPhase
from core.telemetry.aggregator import RuntimeStateAggregator
from core.telemetry.events import Category, Event


def test_mission_phase_changed_updates_snapshot_phase():
    agg = RuntimeStateAggregator(mission_id="m1")
    agg.on_event(Event(code="MISSION_PHASE_CHANGED", subsystem="MissionContext",
                        data={"from_phase": "MISSION_INIT", "to_phase": "CONNECTING"}))

    snap = agg.snapshot()
    assert snap.phase == MissionPhase.CONNECTING


def test_blocking_entered_then_cleared():
    agg = RuntimeStateAggregator()
    agg.on_event(Event(code="BLOCKING_STATE_ENTERED", subsystem="Sub",
                        data={"waiting_on": "WAITING_X", "kind": "BLOCKING_WAIT"}))
    assert agg.snapshot().blocking is not None
    assert agg.snapshot().blocking.waiting_on == "WAITING_X"

    agg.on_event(Event(code="BLOCKING_STATE_CLEARED", subsystem="Sub"))
    assert agg.snapshot().blocking is None


def test_mission_upload_confirmed_updates_mission_state():
    agg = RuntimeStateAggregator()
    agg.on_event(Event(code="MISSION_UPLOAD_CONFIRMED", subsystem="MavsdkBackendBase",
                        data={"requested_item_count": 5, "uploaded_item_count": 5}))
    snap = agg.snapshot()
    assert snap.mission.uploaded is True
    assert snap.mission.uploaded_item_count == 5


def test_mission_upload_mismatch_reports_unuploaded():
    agg = RuntimeStateAggregator()
    agg.on_event(Event(code="MISSION_UPLOAD_MISMATCH", subsystem="MavsdkBackendBase",
                        data={"requested_item_count": 5, "uploaded_item_count": 0}))
    snap = agg.snapshot()
    assert snap.mission.uploaded is False
    assert snap.mission.uploaded_item_count == 0


def test_payload_release_events_update_payload_state():
    agg = RuntimeStateAggregator()
    agg.on_event(Event(code="PAYLOAD_1_RELEASED", subsystem="PayloadInterlock",
                        category=Category.PAYLOAD,
                        data={"payload_1_released": True, "payload_2_released": False}))
    snap = agg.snapshot()
    assert snap.payload.payload_1_released is True
    assert snap.payload.payload_2_released is False


def test_recent_events_ring_buffer_is_capped():
    agg = RuntimeStateAggregator()
    for i in range(200):
        agg.on_event(Event(code=f"EVT_{i}", subsystem="Sub"))
    snap = agg.snapshot()
    assert len(snap.recent_events) <= 80
    assert snap.recent_events[-1].code == "EVT_199"


def test_snapshot_is_a_safe_copy_not_a_live_reference():
    agg = RuntimeStateAggregator()
    agg.on_event(Event(code="MISSION_PHASE_CHANGED", subsystem="MissionContext",
                        data={"to_phase": "SEARCHING"}))
    snap1 = agg.snapshot()
    agg.on_event(Event(code="MISSION_PHASE_CHANGED", subsystem="MissionContext",
                        data={"to_phase": "MISSION_COMPLETE"}))
    # snap1 must not have mutated when the aggregator's internal state changed later.
    assert snap1.phase == MissionPhase.SEARCHING
