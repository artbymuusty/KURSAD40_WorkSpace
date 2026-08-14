"""
ADR-004 §5 (Mission Lifecycle State Machine). The 19-step Görev 2 sequence
extended with the Görev 3 phases already implemented in gorev3_*.py, plus
the FAILURE/ABORT/TIMEOUT terminal states the pre-ADR-004 code had no
concept of at all -- this is what makes "the drone just sits there forever"
a bounded, reportable outcome instead of an open-ended one.
"""
from enum import Enum


class MissionPhase(str, Enum):
    MISSION_INIT = "MISSION_INIT"
    CONNECTING = "CONNECTING"
    ARMING = "ARMING"
    TAKEOFF = "TAKEOFF"
    CLIMB_TO_ALTITUDE = "CLIMB_TO_ALTITUDE"
    CHECKPOINT_SAVE = "CHECKPOINT_SAVE"
    # Renamed from MISSION_ROUTE_GENERATE: this system never generates its
    # own search route -- the operator defines it in QGroundControl before
    # flight (Görev 2 Rapor: "QGroundControl: Operatörün görev öncesi
    # waypoint/tarama rotası tanımlaması"). This phase only confirms one is
    # already present on the vehicle; renamed so the event timeline can't
    # be misread as "the system generated a route" (it previously did,
    # silently overwriting whatever the operator had planned in QGC --
    # fixed together with this rename).
    MISSION_ROUTE_CONFIRM = "MISSION_ROUTE_CONFIRM"
    MISSION_UPLOAD = "MISSION_UPLOAD"
    MISSION_START = "MISSION_START"
    SEARCHING = "SEARCHING"
    TARGET_TRACKING = "TARGET_TRACKING"
    SWITCH_TO_OFFBOARD = "SWITCH_TO_OFFBOARD"
    GOTO_TARGET_CENTERING = "GOTO_TARGET_CENTERING"
    HOVER_CONFIRM = "HOVER_CONFIRM"
    GPS_SAVE = "GPS_SAVE"
    PAYLOAD_DECISION = "PAYLOAD_DECISION"
    PAYLOAD_RELEASE = "PAYLOAD_RELEASE"
    PAYLOAD_VERIFY = "PAYLOAD_VERIFY"
    RETURN_TO_SECOND_TARGET = "RETURN_TO_SECOND_TARGET"
    # Operator revision (2026-08-13, "Mission Lifecycle" restructuring): the
    # one-way Search->Offboard transition needs its own observable phase --
    # BlueFound && RedFound becoming true, permanently ending Mission
    # authority. See Gorev2Orchestrator._search_and_engage_loop.
    SEARCH_COMPLETE = "SEARCH_COMPLETE"
    GOREV2_COMPLETE = "GOREV2_COMPLETE"
    GOREV3_START = "GOREV3_START"
    GOREV3_RUNNING = "GOREV3_RUNNING"
    RETURN_TO_CHECKPOINT = "RETURN_TO_CHECKPOINT"
    LANDING = "LANDING"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    MISSION_FAILED = "MISSION_FAILED"
    MISSION_ABORTED = "MISSION_ABORTED"
    MISSION_TIMEOUT = "MISSION_TIMEOUT"


TERMINAL_PHASES = frozenset({
    MissionPhase.MISSION_COMPLETE,
    MissionPhase.MISSION_FAILED,
    MissionPhase.MISSION_ABORTED,
    MissionPhase.MISSION_TIMEOUT,
})
