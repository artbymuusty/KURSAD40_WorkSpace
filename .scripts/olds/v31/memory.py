# memory.py
import time
from typing import Optional, Dict, Any

class MissionMemory:
    """
    Stores mission state and locations for top locks and payload drops.
    """
    def __init__(self):
        # Mission start position: the SINGLE canonical reference point for
        # RETURN_HOME/LANDING. Captured exactly once by mission.py
        # (MissionManager.update_telemetry), the very first time valid
        # LOCAL_POSITION_NED telemetry is observed -- on the ground, before
        # ARM_OFFBOARD has even run. This is the drone's real physical
        # pre-arm resting point, NOT a post-takeoff hover sample at any
        # altitude. save_start_position is a no-op once set, so nothing
        # downstream can ever overwrite it.
        self.start_position: Optional[Dict[str, Any]] = None

        # Drops
        self.first_drop_position: Optional[Dict[str, Any]] = None
        self.first_dropped_payload_color: Optional[str] = None
        self.first_target: Optional[str] = None
        
        self.second_drop_position: Optional[Dict[str, Any]] = None
        self.second_dropped_payload_color: Optional[str] = None
        self.second_target: Optional[str] = None
        
        # Mission flow
        self.completed_targets = []

        # RUNTIME DEBUG FIX (KURSAD40 second-payload investigation), STEP 8:
        # completed_targets only ever meant "this target was attempted and
        # disabled" (set unconditionally in _state_payload_drop even on a
        # drop timeout/failure, so the mission doesn't get stuck re-locking a
        # target it can't drop forever). It was ALSO the sole gate deciding
        # whether to enter BONUS_PICKUP, which let pickup start even if a
        # drop was never actually confirmed. confirmed_drops tracks only
        # payload colors PayloadManager's DROP_STATE confirmation actually
        # verified as spawned -- see _state_payload_drop/_state_ascend_after_drop.
        self.confirmed_drops = []
        
        # Target Metadata 
        # stores a list of dictionaries with target detection metadata
        self.target_metadata = []
        
        # Recovery Status (e.g., PENDING, SUCCESS, FAILED, DELIVERED)
        self.recovery_status = "PENDING"

    def _format_position(self, gps, ned, alt) -> Dict[str, Any]:
        try:
            ned_safe = tuple(ned) if ned is not None else None
        except Exception:
            print("[MEMORY] Warning: NED unavailable, saving None")
            ned_safe = None
        try:
            alt_safe = float(alt) if alt is not None else None
        except Exception:
            print("[MEMORY] Warning: altitude unavailable, saving None")
            alt_safe = None
        return {
            "gps": gps,          # None if GPS unavailable — callers must handle
            "ned": ned_safe,
            "alt": alt_safe,
            "timestamp": time.time()
        }

    def save_start_position(self, gps, ned, alt, heading):
        """
        Records the mission's single canonical start/return point. Idempotent
        by design (first call wins, later calls are no-ops) so that "must
        NEVER change during the mission" is enforced here, not just by
        callers remembering to only call it once.
        """
        if self.start_position is not None:
            return
        data = self._format_position(gps, ned, alt)
        data["north"] = ned[0] if ned is not None else None
        data["east"] = ned[1] if ned is not None else None
        # Explicit NED "down" component, kept alongside north/east so
        # mission.py's return-to-start logic has a literal
        # (north, east, down) triple to goto_position() -- no separate
        # AGL-to-down conversion needed for the return target itself (only
        # for the intermediate descent waypoints, see mission._agl_to_down).
        data["down"] = ned[2] if ned is not None else None
        data["heading"] = float(heading) if heading is not None else 0.0
        self.start_position = data
        print("[MISSION] GROUND START SAVED")
        print(f"[MISSION] North: {data['north']:.3f}")
        print(f"[MISSION] East:  {data['east']:.3f}")
        print(f"[MISSION] Down:  {data['down']:.3f}")

    def save_drop(self, payload_color: str, target_name: str, gps, ned, alt):
        data = self._format_position(gps, ned, alt)
        data["target_name"] = target_name
        data["selected_payload_color"] = payload_color
        
        if self.first_drop_position is None:
            self.first_drop_position = data
            self.first_dropped_payload_color = payload_color
            self.first_target = target_name
            print(f"[MEMORY] First drop saved: payload={payload_color}, target={target_name}, ned={ned}")
        elif self.second_drop_position is None:
            self.second_drop_position = data
            self.second_dropped_payload_color = payload_color
            self.second_target = target_name
            print(f"[MEMORY] Second drop saved: payload={payload_color}, target={target_name}, ned={ned}")

    def mark_target_completed(self, target_type: str):
        if target_type not in self.completed_targets:
            self.completed_targets.append(target_type)
            print(f"[MEMORY] Target '{target_type}' marked as completed.")

    def mark_drop_confirmed(self, payload_color: str):
        """See confirmed_drops in __init__: only ever called when
        PayloadManager's DROP_STATE confirmation actually verified a spawn."""
        if payload_color not in self.confirmed_drops:
            self.confirmed_drops.append(payload_color)
            print(f"[MEMORY] Drop CONFIRMED for payload '{payload_color}'. "
                  f"confirmed_drops={self.confirmed_drops}")

    def save_target_metadata(self, metadata: dict):
        self.target_metadata.append(metadata)

    def get_target_for_pickup(self) -> Optional[str]:
        """
        Dynamically returns the vision class required for the pickup phase based on the first drop.
        Returns the exact class associated with the payload color.
        """
        if self.first_dropped_payload_color is None:
            return None
        return f"{self.first_dropped_payload_color}_payload_target"

    def compute_pickup_trajectory(self) -> Optional[Dict[str, tuple]]:
        """
        KURSAD40 bonus mission pickup geometry: heading is locked toward
        Drop 2 for the ENTIRE pickup + transport (never rotates -- "the
        transport flight will continue in the same direction"). start_ned
        is 0.30m behind Drop 1 (opposite the Drop1->Drop2 direction);
        end_ned is 0.60m forward of start_ned, i.e. 0.30m past Drop 1 --
        the hook passes directly over Drop 1 during that one continuous
        0.60m forward pass. drop1_ned is Drop 1's own position, used for
        the initial horizontal approach before descending.
        """
        if not self.first_drop_position or not self.second_drop_position:
            return None

        n1, e1, _ = self.first_drop_position["ned"]
        n2, e2, _ = self.second_drop_position["ned"]

        # Vector Drop1 -> Drop2 (the locked heading direction for pickup+transport).
        dn = n2 - n1
        de = e2 - e1
        mag = (dn**2 + de**2)**0.5

        if mag < 0.01: # Drops are identical, fallback to a north-facing approach
            dn, de = 1.0, 0.0
            mag = 1.0

        un = dn / mag
        ue = de / mag

        start_n = n1 - un * 0.30
        start_e = e1 - ue * 0.30

        end_n = start_n + un * 0.60
        end_e = start_e + ue * 0.60

        import math
        heading = math.degrees(math.atan2(de, dn))  # toward Drop2, locked

        return {
            "drop1_ned": (n1, e1, -0.30),
            "start_ned": (start_n, start_e, -0.30),
            "end_ned": (end_n, end_e, -0.30),
            "heading": heading,
            "vector_uv": (un, ue)
        }

    def compute_delivery_trajectory(self) -> Optional[Dict[str, tuple]]:
        """
        KURSAD40 bonus mission delivery geometry: drop2_ned is the transport
        waypoint near Drop 2 (final precise centering is done with live
        vision + visual servo, not dead reckoning -- see
        MissionManager._state_payload_transfer). heading/vector_uv are the
        SAME locked Drop1->Drop2 direction used throughout pickup and
        transport, so the backward hook-extraction move (0.30m opposite
        this vector, computed live off the actual ground-contact position
        in _state_payload_release) retraces the identical line instead of
        an independently-computed one.
        """
        if not self.first_drop_position or not self.second_drop_position:
            return None

        n1, e1, _ = self.first_drop_position["ned"]
        n2, e2, _ = self.second_drop_position["ned"]

        dn = n2 - n1
        de = e2 - e1
        mag = (dn**2 + de**2)**0.5

        if mag < 0.01:
            dn, de = 1.0, 0.0
            mag = 1.0

        un = dn / mag
        ue = de / mag

        import math
        heading = math.degrees(math.atan2(de, dn))  # same locked heading as pickup

        return {
            "drop2_ned": (n2, e2, -0.30),
            "heading": heading,
            "vector_uv": (un, ue)
        }

