"""
ADR-004 §13 (Mission Operations Center Architecture) -- "Runtime State
Aggregator: folds the event stream into the live MissionSnapshot. This is
the only component allowed to mutate the snapshot; everything else reads it."

Subscribed to EventBus (subscribe(aggregator.on_event)). Every event first
gets appended to the recent_events ring buffer regardless of code -- that
alone gives replay/diagnostics a complete timeline. A handful of event
codes additionally update typed sub-state (vehicle/mission/vision/payload)
so the dashboard doesn't have to re-derive it from raw event history on
every render tick.
"""
import copy
import time
from threading import RLock
from typing import Optional

from core.mission.blocking import BlockingKind, BlockingState
from core.mission.phase import MissionPhase
from core.telemetry.events import Event
from core.telemetry.snapshot import MissionSnapshot, TrackState

RECENT_EVENTS_MAX = 80

# Event codes whose `data` dict is merged wholesale into the matching
# sub-dataclass -- keeps the aggregator generic instead of one branch per
# domain event. Producers are responsible for only including keys that are
# real fields of the target dataclass.
_VEHICLE_SYNC_CODES = {"VEHICLE_TELEMETRY"}
_MISSION_SYNC_CODES = {"MISSION_STATE_SYNC"}
_VISION_SYNC_CODES = {"VISION_FRAME_PROCESSED"}
_PAYLOAD_SYNC_CODES = {"PAYLOAD_STATE_SYNC"}


class RuntimeStateAggregator:
    def __init__(self, mission_id: str = "", timeout_budget_s: Optional[float] = None):
        self._lock = RLock()
        self._snapshot = MissionSnapshot(mission_id=mission_id, timeout_budget_s=timeout_budget_s)

    # ------------------------------------------------------------------
    def on_event(self, event: Event) -> None:
        with self._lock:
            snap = self._snapshot
            snap.recent_events.append(event)
            if len(snap.recent_events) > RECENT_EVENTS_MAX:
                snap.recent_events.pop(0)

            if event.code == "MISSION_PHASE_CHANGED":
                try:
                    snap.phase = MissionPhase(event.data["to_phase"])
                except (KeyError, ValueError):
                    pass
                snap.phase_entered_at = event.ts

            elif event.code == "BLOCKING_STATE_ENTERED":
                snap.blocking = BlockingState(
                    waiting_on=event.data.get("waiting_on", "UNKNOWN"),
                    owning_subsystem=event.subsystem,
                    kind=BlockingKind(event.data.get("kind", BlockingKind.BLOCKING_WAIT.value)),
                    since=event.ts,
                    timeout_at=(event.ts + event.data["timeout_s"]) if event.data.get("timeout_s") else None,
                    last_known_cause=event.data.get("cause"),
                )

            elif event.code == "BLOCKING_STATE_CLEARED":
                snap.blocking = None

            elif event.code in _VEHICLE_SYNC_CODES:
                for k, v in event.data.items():
                    if hasattr(snap.vehicle, k):
                        setattr(snap.vehicle, k, v)
                if "position" in event.data:
                    snap.vehicle.position_updated_at = event.ts

            elif event.code in _MISSION_SYNC_CODES:
                for k, v in event.data.items():
                    if hasattr(snap.mission, k):
                        setattr(snap.mission, k, v)

            elif event.code == "MISSION_UPLOAD_CONFIRMED":
                snap.mission.uploaded = True
                snap.mission.requested_item_count = event.data.get("requested_item_count")
                snap.mission.uploaded_item_count = event.data.get("uploaded_item_count")

            elif event.code == "MISSION_UPLOAD_MISMATCH":
                snap.mission.uploaded = False
                snap.mission.requested_item_count = event.data.get("requested_item_count")
                snap.mission.uploaded_item_count = event.data.get("uploaded_item_count")

            elif event.code == "QGC_STATUS":
                snap.qgc_connected = event.data.get("connected")

            elif event.code == "CHECKPOINT_SAVED":
                ckpt = event.data.get("checkpoint")
                if ckpt:
                    snap.mission.checkpoint = tuple(ckpt)

            elif event.code == "MISSION_ROUTE_CONFIRMED":
                # Operator-defined route (QGroundControl), confirmed already
                # present on the vehicle -- this system never generates or
                # uploads one itself (replaces the old ROUTE_GENERATED code,
                # which implied the opposite).
                snap.mission.uploaded = True
                snap.mission.uploaded_item_count = event.data.get("item_count")

            elif event.code == "MISSION_ROUTE_MISSING":
                snap.mission.uploaded = False
                snap.mission.uploaded_item_count = 0

            elif event.code in _VISION_SYNC_CODES:
                snap.vision.detector_ready = event.data.get("detector_ready", snap.vision.detector_ready)
                snap.vision.last_frame_at = event.ts
                snap.vision.frame_count += 1
                snap.vision.last_detection_count = event.data.get("detection_count", 0)

            elif event.code == "TRACK_STATE_UPDATED":
                shape = event.data.get("shape_type")
                if shape:
                    snap.vision.active_tracks[shape] = TrackState(
                        shape_type=shape,
                        consecutive_frames=event.data.get("consecutive_frames", 0),
                        is_centered=event.data.get("is_centered", False),
                        is_navigating_to=event.data.get("is_navigating_to", False),
                        altitude_ok=event.data.get("altitude_ok", False),
                    )

            elif event.code in _PAYLOAD_SYNC_CODES or event.code in (
                "PAYLOAD_1_RELEASED", "PAYLOAD_2_RELEASED", "PAYLOAD_STATE_SYNC",
            ):
                for k, v in event.data.items():
                    if hasattr(snap.payload, k):
                        setattr(snap.payload, k, v)

            elif event.code == "PAYLOAD_VERIFICATION_RESULT":
                snap.payload.last_verification_marker = event.data.get("expected_marker")
                snap.payload.last_verification_found = event.data.get("found")

            elif event.code == "TARGET_DEBOUNCED" or event.code == "DEBOUNCE_STATE_SYNC":
                shape = event.data.get("shape_type")
                remaining = event.data.get("remaining_s")
                if shape is not None and remaining is not None:
                    snap.debounce[shape] = remaining

            elif event.code == "HEALTH_STATE_CHANGED":
                from core.telemetry.snapshot import HealthEntry
                snap.health[event.subsystem] = HealthEntry(
                    state=event.data.get("state", "UNKNOWN"),
                    last_seen=event.data.get("last_seen"),
                    detail=event.message,
                )

            elif event.code in ("WATCHDOG_ARMED", "WATCHDOG_UPDATED", "WATCHDOG_DISARMED", "WATCHDOG_FIRED"):
                from core.telemetry.snapshot import WatchdogEntry
                name = event.data.get("name", event.subsystem)
                if event.code == "WATCHDOG_DISARMED":
                    snap.watchdogs.pop(name, None)
                else:
                    snap.watchdogs[name] = WatchdogEntry(
                        name=name,
                        armed=event.code != "WATCHDOG_FIRED",
                        remaining_s=event.data.get("remaining_s"),
                        threshold_s=event.data.get("threshold_s"),
                    )

    # ------------------------------------------------------------------
    def snapshot(self) -> MissionSnapshot:
        """Cheap, lock-protected copy -- safe to call from any thread
        (the dashboard's render thread polls this) without risking a
        torn read while the mission coroutine is publishing."""
        with self._lock:
            snap = copy.copy(self._snapshot)
            snap.vehicle = copy.copy(self._snapshot.vehicle)
            snap.mission = copy.copy(self._snapshot.mission)
            snap.vision = copy.copy(self._snapshot.vision)
            snap.vision.active_tracks = dict(self._snapshot.vision.active_tracks)
            snap.payload = copy.copy(self._snapshot.payload)
            snap.debounce = dict(self._snapshot.debounce)
            snap.health = dict(self._snapshot.health)
            snap.watchdogs = dict(self._snapshot.watchdogs)
            snap.recent_events = list(self._snapshot.recent_events)

        now = time.time()
        snap.elapsed_s = now - snap.started_at
        if snap.timeout_budget_s is not None:
            snap.timeout_remaining_s = max(0.0, snap.timeout_budget_s - snap.elapsed_s)
        snap.phase_elapsed_s = now - snap.phase_entered_at
        return snap
