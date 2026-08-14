import asyncio
import logging
from typing import Tuple
import numpy as np

from core.interfaces.i_flight_backend import IFlightBackend
from core.interfaces.i_camera_source import ICameraSource
from core.interfaces.i_payload_actuator import IPayloadActuator

logger = logging.getLogger(__name__)

class DualFlightBackend(IFlightBackend):
    def __init__(self, real: IFlightBackend, sim: IFlightBackend):
        self._real = real
        self._sim = sim

    async def connect(self) -> None:
        await asyncio.gather(self._real.connect(), self._sim.connect())
        
    async def arm(self) -> None:
        await asyncio.gather(self._real.arm(), self._sim.arm())
        
    async def takeoff(self, target_altitude_m: float) -> None:
        await asyncio.gather(self._real.takeoff(target_altitude_m), self._sim.takeoff(target_altitude_m))
        
    async def land(self) -> None:
        await asyncio.gather(self._real.land(), self._sim.land())
        
    async def start_offboard(self) -> None:
        await asyncio.gather(self._real.start_offboard(), self._sim.start_offboard())
        
    async def stop_offboard(self) -> None:
        await asyncio.gather(self._real.stop_offboard(), self._sim.stop_offboard())
        
    async def goto_position_ned(self, north_m: float, east_m: float, down_m: float, yaw_deg: float) -> None:
        await asyncio.gather(
            self._real.goto_position_ned(north_m, east_m, down_m, yaw_deg),
            self._sim.goto_position_ned(north_m, east_m, down_m, yaw_deg)
        )
        
    async def goto_position_ned_and_hold(self, north_m: float, east_m: float, down_m: float,
                                          yaw_deg: float, duration_s: float) -> None:
        await asyncio.gather(
            self._real.goto_position_ned_and_hold(north_m, east_m, down_m, yaw_deg, duration_s),
            self._sim.goto_position_ned_and_hold(north_m, east_m, down_m, yaw_deg, duration_s)
        )

    async def set_velocity_body(self, forward_m_s: float, right_m_s: float, down_m_s: float, yaw_rate_deg_s: float) -> None:
        await asyncio.gather(
            self._real.set_velocity_body(forward_m_s, right_m_s, down_m_s, yaw_rate_deg_s),
            self._sim.set_velocity_body(forward_m_s, right_m_s, down_m_s, yaw_rate_deg_s)
        )
        
    async def hold_position(self, duration_s: float) -> None:
        await asyncio.gather(self._real.hold_position(duration_s), self._sim.hold_position(duration_s))
        
    async def get_position_ned(self) -> Tuple[float, float, float]:
        real_pos, sim_pos = await asyncio.gather(self._real.get_position_ned(), self._sim.get_position_ned())
        # Check diff optionally, return real
        return real_pos

    async def get_velocity_ned(self) -> Tuple[float, float, float]:
        real_vel, sim_vel = await asyncio.gather(self._real.get_velocity_ned(), self._sim.get_velocity_ned())
        return real_vel

    async def get_global_position(self) -> Tuple[float, float, float]:
        real_pos, sim_pos = await asyncio.gather(self._real.get_global_position(), self._sim.get_global_position())
        return real_pos
        
    async def get_yaw_deg(self) -> float:
        real_yaw, sim_yaw = await asyncio.gather(self._real.get_yaw_deg(), self._sim.get_yaw_deg())
        return real_yaw

    async def get_flight_mode(self) -> str:
        real_mode, sim_mode = await asyncio.gather(self._real.get_flight_mode(), self._sim.get_flight_mode())
        if real_mode != sim_mode:
            logger.warning(f"DUAL UYUMSUZLUK: real flight_mode={real_mode} sim flight_mode={sim_mode}")
        return real_mode
        
    async def upload_mission(self, waypoints: list) -> None:
        await asyncio.gather(self._real.upload_mission(waypoints), self._sim.upload_mission(waypoints))

    async def confirm_existing_mission(self) -> int:
        real_count, sim_count = await asyncio.gather(
            self._real.confirm_existing_mission(), self._sim.confirm_existing_mission()
        )
        if real_count != sim_count:
            logger.warning(
                f"DUAL UYUMSUZLUK: real mission item count={real_count} sim item count={sim_count} -- "
                f"operatör her iki taraf için de aynı planı yüklemiş mi kontrol edilmeli"
            )
        return real_count

    async def start_mission(self) -> None:
        await asyncio.gather(self._real.start_mission(), self._sim.start_mission())
        
    async def is_mission_finished(self) -> bool:
        real_finished, sim_finished = await asyncio.gather(self._real.is_mission_finished(), self._sim.is_mission_finished())
        return real_finished
        
    async def switch_to_offboard_from_mission(self) -> None:
        await asyncio.gather(self._real.switch_to_offboard_from_mission(), self._sim.switch_to_offboard_from_mission())


class DualCameraSource(ICameraSource):
    def __init__(self, real: ICameraSource, sim: ICameraSource):
        self._real = real
        self._sim = sim

    async def start(self) -> None:
        await asyncio.gather(self._real.start(), self._sim.start())
        
    async def stop(self) -> None:
        await asyncio.gather(self._real.stop(), self._sim.stop())
        
    async def get_frame(self) -> np.ndarray:
        real_frame, sim_frame = await asyncio.gather(self._real.get_frame(), self._sim.get_frame())
        return real_frame
        
    def get_resolution(self) -> Tuple[int, int]:
        return self._real.get_resolution()


class DualPayloadActuator(IPayloadActuator):
    def __init__(self, real: IPayloadActuator, sim: IPayloadActuator):
        self._real = real
        self._sim = sim

    async def release_payload_at_mavi_altigen(self) -> bool:
        real_result, sim_result = await asyncio.gather(
            self._real.release_payload_at_mavi_altigen(),
            self._sim.release_payload_at_mavi_altigen()
        )
        if real_result != sim_result:
            logger.warning(
                f"DUAL UYUMSUZLUK: real={real_result} sim={sim_result} - "
                f"gercek ve simule sonuc farkli, saha ekibi incelemeli"
            )
        return real_result

    async def release_payload_at_kirmizi_ucgen(self) -> bool:
        real_result, sim_result = await asyncio.gather(
            self._real.release_payload_at_kirmizi_ucgen(),
            self._sim.release_payload_at_kirmizi_ucgen()
        )
        if real_result != sim_result:
            logger.warning(f"DUAL UYUMSUZLUK: real={real_result} sim={sim_result}")
        return real_result
        
    async def activate_pickup_mechanism(self) -> bool:
        real_result, sim_result = await asyncio.gather(
            self._real.activate_pickup_mechanism(),
            self._sim.activate_pickup_mechanism()
        )
        if real_result != sim_result:
            logger.warning(f"DUAL UYUMSUZLUK: real={real_result} sim={sim_result}")
        return real_result
        
    async def activate_drop_mechanism(self) -> bool:
        real_result, sim_result = await asyncio.gather(
            self._real.activate_drop_mechanism(),
            self._sim.activate_drop_mechanism()
        )
        if real_result != sim_result:
            logger.warning(f"DUAL UYUMSUZLUK: real={real_result} sim={sim_result}")
        return real_result
