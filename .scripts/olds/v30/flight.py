# flight_controller.py
import asyncio
import threading
import time
import math
from typing import Optional, Tuple
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, PositionNedYaw
from mavsdk.telemetry import FlightMode

from mission_types import FlightIntent
from mavsdk.telemetry import FlightMode

class MavController:
    """
    Manages single MAVSDK System connection and handles all flight control.
    """
    def __init__(self, url: str):
        self.url = url
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        self.drone: Optional[System] = None
        self.connected = threading.Event()

        self.alt_rel = 0.0
        self.ned = (0.0, 0.0, 0.0)
        self.yaw_deg = 0.0
        # KURSAD40 post-attach stability fix: roll/pitch were already
        # arriving on the attitude_euler stream but discarded; angular rates
        # need a new (but already-existing-in-MAVSDK) subscription. Both are
        # needed to detect "has the vehicle actually settled" from real
        # telemetry instead of an arbitrary timer (see mission.py's
        # _attitude_settled).
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate_actual = 0.0
        self.flight_mode = None
        self.is_armed = False
        self.last_intent_time = 0.0
        self.last_telemetry_time = 0.0

        self.submit(self._connect())

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    async def _connect(self):
        self.drone = System()
        await self.drone.connect(system_address=self.url)
        async for st in self.drone.core.connection_state():
            if st.is_connected:
                print("[MAVSDK] Connected.")
                self.connected.set()
                break

        self.loop.create_task(self._tel_position())
        self.loop.create_task(self._tel_ned())
        self.loop.create_task(self._tel_att())
        self.loop.create_task(self._tel_angular_velocity())
        self.loop.create_task(self._tel_fm())
        self.loop.create_task(self._tel_armed())
        self.loop.create_task(self._offboard_watchdog())

    async def _offboard_watchdog(self):
        """Monitors intent starvation and commands HOVER to prevent OFFBOARD exit."""
        while True:
            await asyncio.sleep(0.05)
            if self.flight_mode == FlightMode.OFFBOARD:
                if time.time() - self.last_intent_time > 0.15:
                    # Stale intent, force hover safely
                    try:
                        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                    except Exception:
                        pass

    async def _resilient_stream(self, name: str, stream_factory, on_value):
        """
        Runs an MAVSDK telemetry async-generator forever, reconnecting on failure.
        MAVSDK telemetry streams can die (e.g. gRPC 'Stream removed' on a PX4/link
        hiccup); without this wrapper the asyncio task simply dies and the field
        it updates (alt/ned/yaw/flight_mode/armed) freezes forever with no warning.
        """
        while True:
            try:
                async for item in stream_factory():
                    on_value(item)
                    self.last_telemetry_time = time.time()
            except Exception as e:
                print(f"[MAVSDK] Telemetry stream '{name}' failed: {e}. Reconnecting in 1s...")
                await asyncio.sleep(1.0)

    async def _tel_position(self):
        def _set(pos):
            self.alt_rel = float(pos.relative_altitude_m)
            self.gps = (float(pos.latitude_deg), float(pos.longitude_deg), float(pos.absolute_altitude_m))
        await self._resilient_stream("position", self.drone.telemetry.position, _set)

    async def _tel_ned(self):
        def _set(pv):
            self.ned = (float(pv.position.north_m), float(pv.position.east_m), float(pv.position.down_m))
        await self._resilient_stream("position_velocity_ned", self.drone.telemetry.position_velocity_ned, _set)

    async def _tel_att(self):
        def _set(e):
            self.yaw_deg = float(e.yaw_deg)
            self.roll_deg = float(e.roll_deg)
            self.pitch_deg = float(e.pitch_deg)
        await self._resilient_stream("attitude_euler", self.drone.telemetry.attitude_euler, _set)

    async def _tel_angular_velocity(self):
        def _set(av):
            self.roll_rate = float(av.roll_rad_s)
            self.pitch_rate = float(av.pitch_rad_s)
            self.yaw_rate_actual = float(av.yaw_rad_s)
        await self._resilient_stream("attitude_angular_velocity_body",
                                      self.drone.telemetry.attitude_angular_velocity_body, _set)

    async def _tel_fm(self):
        def _set(fm):
            self.flight_mode = fm
        await self._resilient_stream("flight_mode", self.drone.telemetry.flight_mode, _set)

    async def _tel_armed(self):
        def _set(armed):
            self.is_armed = bool(armed)
        await self._resilient_stream("armed", self.drone.telemetry.armed, _set)

    def telemetry_is_fresh(self, max_age: float = 2.0) -> bool:
        """True once telemetry has been received and is not stale."""
        return self.last_telemetry_time > 0.0 and (time.time() - self.last_telemetry_time) <= max_age

    def alt(self) -> float:
        return self.alt_rel

    def get_ned(self) -> Tuple[float, float, float]:
        return self.ned

    def get_gps(self) -> Optional[Tuple[float, float, float]]:
        return getattr(self, "gps", None)

    def is_offboard(self) -> bool:
        return self.flight_mode == FlightMode.OFFBOARD

    async def arm(self) -> bool:
        try:
            await self.drone.action.arm()
            return True
        except Exception as e:
            print(f"[MAVSDK] Arm failed: {e}")
            return False

    async def disarm(self) -> bool:
        try:
            await self.drone.action.disarm()
            return True
        except Exception as e:
            print(f"[MAVSDK] Disarm failed: {e}")
            return False

    async def start_offboard(self) -> bool:
        """Starts offboard mode safely by setting an initial zero velocity setpoint."""
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            return True
        except OffboardError as e:
            print(f"[MAVSDK] Offboard start failed: {e}")
            return False

    async def stop_offboard(self):
        try:
            await self.drone.offboard.stop()
        except Exception:
            pass

    async def arm_and_start_offboard(self) -> bool:
        """
        Idempotent step-function toward ARMED + OFFBOARD. Safe to call every tick:
        it only issues the arm/offboard commands that are still needed and no-ops
        once both conditions are already satisfied.
        """
        if not self.is_armed:
            armed_ok = await self.arm()
            if not armed_ok:
                return False

        if self.flight_mode == FlightMode.OFFBOARD:
            return True

        return await self.start_offboard()

    async def set_velocity_body(self, v_fwd: float, v_right: float, v_down: float, yaw_rate: float = 0.0) -> bool:
        """Sends velocity body commands in offboard mode."""
        try:
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(v_fwd, v_right, v_down, yaw_rate)
            )
            return True
        except OffboardError:
            print("[MAVSDK] set_velocity_body failed, trying to restart offboard")
            return await self.start_offboard()

    async def execute_intent(self, intent: FlightIntent):
        """Translates a high-level FlightIntent into MAVSDK commands."""
        if not getattr(self, "drone", None):
            return

        self.last_intent_time = time.time()

        if intent.mode == "HOVER":
            await self.set_velocity_body(0.0, 0.0, 0.0, 0.0)

        elif intent.mode == "ARM_OFFBOARD":
            await self.arm_and_start_offboard()

        elif intent.mode == "DISARM":
            await self.disarm()

        elif intent.mode == "VELOCITY_BODY":
            fwd, right, down = intent.velocity
            yaw_r = intent.yaw_rate if hasattr(intent, 'yaw_rate') else 0.0
            await self.set_velocity_body(fwd, right, down, yaw_r)
            
        elif intent.mode == "NAVIGATE_NED":
            if intent.target_ned:
                n, e, d = intent.target_ned
                # Proportional velocity towards NED target
                dx = n - self.ned[0]
                dy = e - self.ned[1]
                dz = d - self.ned[2]
                
                v_n = max(-4.8, min(4.8, dx))
                v_e = max(-4.8, min(4.8, dy))
                v_d = max(-1.0, min(1.0, dz))
                
                # Convert NED velocity to Body frame based on current yaw
                yaw_rad = math.radians(self.yaw_deg)
                v_fwd  = math.cos(yaw_rad) * v_n + math.sin(yaw_rad) * v_e
                v_right = -math.sin(yaw_rad) * v_n + math.cos(yaw_rad) * v_e
                
                yaw_r = 0.0
                if hasattr(intent, 'target_heading') and intent.target_heading is not None:
                    heading_err = (intent.target_heading - self.yaw_deg + 180) % 360 - 180
                    yaw_r = max(-30.0, min(30.0, heading_err))
                    
                await self.set_velocity_body(v_fwd, v_right, v_d, yaw_r)
                
        elif intent.mode == "LAND":
            try:
                await self.drone.action.land()
            except Exception:
                pass

    async def goto_ned(self, target_ne: Tuple[float, float], target_down: float, arrive_m: float = 0.8) -> bool:
        """Goes to local NED coordinates."""
        try:
            await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, self.yaw_deg))
            await self.drone.offboard.start()
        except OffboardError as e:
            print(f"[MAVSDK] Goto NED offboard start err: {e}")
            return False

        try:
            while True:
                n, e, d = self.get_ned()
                dist = math.hypot(target_ne[0] - n, target_ne[1] - e)
                await self.drone.offboard.set_position_ned(PositionNedYaw(target_ne[0], target_ne[1], target_down, self.yaw_deg))
                if dist <= arrive_m:
                    return True
                await asyncio.sleep(0.05)
        except Exception as e:
            print(f"[MAVSDK] Goto NED error: {e}")
            return False

    async def pause_mission(self):
        try:
            await self.drone.mission.pause_mission()
        except Exception:
            pass

    async def resume_mission(self) -> bool:
        """Strongly attempts to resume the mission mode."""
        try:
            await self.drone.mission.start_mission()
        except Exception as e:
            print(f"[MAVSDK] start_mission err: {e}")

        t0 = time.monotonic()
        while (time.monotonic() - t0) < 2.5:
            if self.flight_mode == FlightMode.MISSION:
                return True
            await asyncio.sleep(0.05)

        # fallback raw
        try:
            await self.drone.mission_raw.start_mission()
        except Exception:
            pass
            
        t0 = time.monotonic()
        while (time.monotonic() - t0) < 2.0:
            if self.flight_mode == FlightMode.MISSION:
                return True
            await asyncio.sleep(0.05)
            
        return False
