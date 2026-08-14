import math
from typing import Optional, Tuple
from mission_types import FlightIntent


class SearchPlanner:
    """
    Deterministic single straight-line forward search (TEKNOFEST Donel Kanat
    Gorev 2 flow): locks in a heading once and continuously commands forward
    flight along that fixed heading, at the fixed search altitude, until the
    mission state machine stops calling get_search_intent() (a target was
    spotted). This replaced the previous lawnmower/area-sweep pattern, which
    does not match the competition's deterministic single-pass search.

    There is no waypoint list or internal position memory. "Resuming from
    where it left off" after a payload drop + re-ascent needs no special
    logic here: start_straight_search() is only called once (on the first
    ascent-to-search-altitude), so the locked heading survives untouched
    across every later SEARCH_RESUME -> SEARCH cycle, and each tick's target
    point is always computed fresh from whatever current NED position is
    passed in -- so the line simply continues onward from wherever the
    vehicle currently is.

    Reuses MavController's existing NAVIGATE_NED handling (proportional
    velocity toward target_ned + proportional yaw correction toward
    target_heading) rather than adding new flight-control logic -- this is
    the same mechanism _state_return_home/_state_payload_transfer already
    rely on elsewhere in mission.py.
    """
    LOOKAHEAD_M = 8.0

    def __init__(self):
        self.heading_deg: Optional[float] = None
        self.altitude_down: Optional[float] = None  # NED down (negative altitude)

    def start_straight_search(self, heading_deg: float, altitude_down: float):
        """Locks the forward heading and cruise altitude for the rest of the mission's search legs."""
        self.heading_deg = heading_deg
        self.altitude_down = altitude_down

    def is_active(self) -> bool:
        return self.heading_deg is not None

    def get_search_intent(self, current_ned: Tuple[float, float, float]) -> FlightIntent:
        if self.heading_deg is None:
            return FlightIntent(mode="HOVER")

        heading_rad = math.radians(self.heading_deg)
        n, e, _ = current_ned
        target_ned = (
            n + self.LOOKAHEAD_M * math.cos(heading_rad),
            e + self.LOOKAHEAD_M * math.sin(heading_rad),
            self.altitude_down,
        )
        return FlightIntent(mode="NAVIGATE_NED", target_ned=target_ned, target_heading=self.heading_deg)

    def get_progress(self) -> str:
        return f"heading={self.heading_deg:.0f}deg" if self.heading_deg is not None else "N/A"
