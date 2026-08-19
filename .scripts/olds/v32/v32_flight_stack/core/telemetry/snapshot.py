"""
ADR-004 §6 (Runtime Data Model). MissionSnapshot is the single object the
Mission Operations Center reads -- everything else (the dashboard render,
the diagnostic panel) is a projection of it. RuntimeStateAggregator
(aggregator.py) is the only component allowed to mutate it; it folds the
EventBus stream into these fields.
"""
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from core.mission.blocking import BlockingState
from core.mission.phase import MissionPhase
from core.telemetry.events import Event


@dataclass
class VehicleState:
    connected: bool = False
    armed: bool = False
    flight_mode: str = "UNKNOWN"
    position: Optional[Tuple[float, float, float]] = None  # lat, lon, alt_rel_m
    position_updated_at: Optional[float] = None
    yaw_deg: Optional[float] = None


@dataclass
class MissionUploadState:
    checkpoint: Optional[Tuple[float, float, float]] = None
    route: Optional[List[tuple]] = None
    requested_item_count: Optional[int] = None
    uploaded_item_count: Optional[int] = None
    uploaded: bool = False
    progress_current: int = 0
    progress_total: int = 0


@dataclass
class TrackState:
    shape_type: str
    consecutive_frames: int = 0
    is_centered: bool = False
    is_navigating_to: bool = False
    altitude_ok: bool = False


@dataclass
class VisionState:
    detector_ready: bool = False
    last_frame_at: Optional[float] = None
    frame_count: int = 0
    last_detection_count: int = 0
    effective_hz: float = 0.0
    active_tracks: Dict[str, TrackState] = field(default_factory=dict)


@dataclass
class CenteringState:
    """Latest CENTERING_STEP, folded for the camera overlay (operator
    request, 2026-08-16: lock indicator + ground-distance label).

    `converged` is the CONTROLLER's own flag, carried through unchanged --
    the overlay must not re-derive "locked" from pixel offsets, or the
    indicator and the control loop could disagree about the same instant.
    """
    shape_type: str = ""
    converged: bool = False
    active: bool = False
    attempt: int = 0
    max_attempts: int = 0
    dx_px: Optional[float] = None
    dy_px: Optional[float] = None
    target_px: Optional[Tuple[float, float]] = None
    center_px: Optional[Tuple[float, float]] = None
    ground_distance_m: Optional[float] = None
    updated_at: Optional[float] = None


@dataclass
class PayloadState:
    payload_1_released: bool = False
    payload_2_released: bool = False
    last_verification_marker: Optional[str] = None
    last_verification_found: Optional[bool] = None

    # ADR-010 P2: live PAYLOAD panel, fed by PAYLOAD_STATE. V1''' released
    # payload 1 at 1.587 m and payload 2 at 0.407 m from the same commanded
    # 0.45 m and the operator had no way to see either at the time -- the
    # altitudes only came out of post-run log analysis. These make the drop
    # observable while it happens.
    active_index: int = 0          # 1 or 2; 0 = no drop in progress
    active_shape: str = ""
    current_alt_m: Optional[float] = None
    target_alt_m: Optional[float] = None
    descent_step: str = ""
    vision_committed: Optional[bool] = None
    last_offset_cm: Optional[float] = None
    released_alt_m: Optional[float] = None
    released_within_tolerance: Optional[bool] = None
    # W4: wall-clock of the servo firing. Drives both the panel's timestamp
    # and the camera overlay's 3-second RELEASED tag. Kept for the rest of
    # the flight -- the panel must still read RELEASED at disarm, which is
    # when an operator actually goes looking for it.
    released_at: Optional[float] = None
    released_index: int = 0
    released_shape: str = ""
    updated_at: Optional[float] = None


@dataclass
class HealthEntry:
    state: str = "UNKNOWN"  # HEALTHY | DEGRADED | STALE | DOWN | UNKNOWN
    last_seen: Optional[float] = None
    detail: str = ""


@dataclass
class WatchdogEntry:
    name: str
    armed: bool
    remaining_s: Optional[float]
    threshold_s: Optional[float]


@dataclass
class MissionSnapshot:
    mission_id: str = ""
    started_at: float = field(default_factory=time.time)
    elapsed_s: float = 0.0
    timeout_budget_s: Optional[float] = None
    timeout_remaining_s: Optional[float] = None

    phase: MissionPhase = MissionPhase.MISSION_INIT
    phase_entered_at: float = field(default_factory=time.time)
    phase_elapsed_s: float = 0.0
    blocking: Optional[BlockingState] = None

    vehicle: VehicleState = field(default_factory=VehicleState)
    mission: MissionUploadState = field(default_factory=MissionUploadState)
    vision: VisionState = field(default_factory=VisionState)
    centering: CenteringState = field(default_factory=CenteringState)
    payload: PayloadState = field(default_factory=PayloadState)

    # None = not yet checked / undeterminable (psutil unavailable), distinct
    # from a confirmed True/False -- see core/telemetry/qgc_monitor.py.
    qgc_connected: Optional[bool] = None

    debounce: Dict[str, float] = field(default_factory=dict)
    health: Dict[str, HealthEntry] = field(default_factory=dict)
    watchdogs: Dict[str, WatchdogEntry] = field(default_factory=dict)

    recent_events: List[Event] = field(default_factory=list)
