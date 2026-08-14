from typing import Tuple, List
from core.interfaces.i_flight_backend import IFlightBackend
from core.config.parameters import OFFBOARD_SETPOINT_INTERVAL_S

class MockFlightBackend(IFlightBackend):
    def __init__(self):
        self.calls: List[Tuple[str, dict]] = []
        self._yaw_deg = 0.0
        self._global_pos = (41.0, 29.0, 15.0)
        self._ned_pos = (0.0, 0.0, -15.0)
        # Defaults to stationary -- goto_global_position_and_wait()'s
        # convergence check gates on this alongside position (see its own
        # BUG FIX comment). Tests exercising the "still moving" case set
        # this to something above GPS_POSITION_VELOCITY_TOLERANCE_M_S.
        self._velocity_ned = (0.0, 0.0, 0.0)
        self._is_mission_finished = False
        # Simulates an operator having already defined a route in
        # QGroundControl pre-flight. Tests exercising the "operator forgot
        # to define a route" failure path set this to 0.
        self._existing_mission_item_count = 5
        # Simulates PX4's actual flight mode -- tests exercising a rejected/
        # unconfirmed Offboard handover set this to something other than
        # "OFFBOARD" (or override start_offboard() to not update it).
        self._flight_mode = "MISSION"

    async def connect(self) -> None:
        self.calls.append(('connect', {}))
        
    async def arm(self) -> None:
        self.calls.append(('arm', {}))
        
    async def takeoff(self, target_altitude_m: float) -> None:
        self.calls.append(('takeoff', {'target_altitude_m': target_altitude_m}))
        
    async def land(self) -> None:
        self.calls.append(('land', {}))
        
    async def start_offboard(self) -> None:
        self.calls.append(('start_offboard', {}))
        self._flight_mode = "OFFBOARD"

    async def stop_offboard(self) -> None:
        self.calls.append(('stop_offboard', {}))
        self._flight_mode = "HOLD"

    async def get_flight_mode(self) -> str:
        self.calls.append(('get_flight_mode', {}))
        return self._flight_mode
        
    async def goto_position_ned(self, north_m: float, east_m: float, down_m: float, yaw_deg: float) -> None:
        self.calls.append(('goto_position_ned', {'north_m': north_m, 'east_m': east_m, 'down_m': down_m, 'yaw_deg': yaw_deg}))
        self._ned_pos = (north_m, east_m, down_m)
        self._yaw_deg = yaw_deg

    async def goto_position_ned_and_hold(self, north_m: float, east_m: float, down_m: float,
                                          yaw_deg: float, duration_s: float) -> None:
        self.calls.append(('goto_position_ned_and_hold', {
            'north_m': north_m, 'east_m': east_m, 'down_m': down_m,
            'yaw_deg': yaw_deg, 'duration_s': duration_s,
        }))
        self._ned_pos = (north_m, east_m, down_m)
        self._yaw_deg = yaw_deg
        
    async def set_velocity_body(self, forward_m_s: float, right_m_s: float, down_m_s: float, yaw_rate_deg_s: float) -> None:
        self.calls.append(('set_velocity_body', {'forward_m_s': forward_m_s, 'right_m_s': right_m_s, 'down_m_s': down_m_s, 'yaw_rate_deg_s': yaw_rate_deg_s}))
        # GAP FIX (operator revision, 2026-08-13): a velocity command that
        # never actually moves _global_pos was harmless while nothing read
        # altitude back mid-loop, but CenteringController.go_to_and_center()
        # now closes a real altitude loop (altitude_m was previously a dead
        # parameter) -- without this, any test driving a real altitude
        # change through this mock would spin for its full max_attempts
        # budget every time, since alt_error could never shrink. Integrates
        # over the same OFFBOARD_SETPOINT_INTERVAL_S every real caller
        # actually sleeps between setpoints, same idea as real MAVSDK/PX4
        # actually flying in response to a velocity command.
        lat, lon, alt = self._global_pos
        self._global_pos = (lat, lon, alt - down_m_s * OFFBOARD_SETPOINT_INTERVAL_S)
        
    async def hold_position(self, duration_s: float) -> None:
        self.calls.append(('hold_position', {'duration_s': duration_s}))
        
    async def get_position_ned(self) -> Tuple[float, float, float]:
        self.calls.append(('get_position_ned', {}))
        return self._ned_pos

    async def get_velocity_ned(self) -> Tuple[float, float, float]:
        self.calls.append(('get_velocity_ned', {}))
        return self._velocity_ned

    async def get_global_position(self) -> Tuple[float, float, float]:
        self.calls.append(('get_global_position', {}))
        return self._global_pos
        
    async def get_yaw_deg(self) -> float:
        self.calls.append(('get_yaw_deg', {}))
        return self._yaw_deg
        
    async def upload_mission(self, waypoints: list) -> None:
        self.calls.append(('upload_mission', {'waypoints': waypoints}))

    async def confirm_existing_mission(self) -> int:
        self.calls.append(('confirm_existing_mission', {}))
        return self._existing_mission_item_count

    async def start_mission(self) -> None:
        self.calls.append(('start_mission', {}))
        
    async def is_mission_finished(self) -> bool:
        self.calls.append(('is_mission_finished', {}))
        return self._is_mission_finished
        
    async def switch_to_offboard_from_mission(self) -> None:
        self.calls.append(('switch_to_offboard_from_mission', {}))
