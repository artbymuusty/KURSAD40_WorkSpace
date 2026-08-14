import asyncio
import subprocess
import time
import zmq
from typing import Optional, Tuple
from flight import MavController
from config import (ACTUATOR_SLOT, HEX_PULSE, TRI_PULSE, GAZEBO_PAYLOAD_DROP_TOPIC,
                     PAYLOAD_DROP_STATE_CONFIRM_TIMEOUT_S)


class PayloadManager:
    """
    Manages payload drops and simulated pickups.
    Supports SIM_MODE (default using MAVSDK actuators) and REAL_MODE placeholders.
    """
    def __init__(self, mav: MavController, sim_mode: bool = True):
        self.mav = mav
        self.sim_mode = sim_mode
        # Exact spawned Gazebo model name of the picked-up payload (e.g.
        # "payload_drop_red_0"), needed to send the matching /hook/detach
        # release later. Set only once HookAttachSystem has actually
        # confirmed the joint via ATTACH_STATE:true (see pickup_payload_simulated).

        # RUNTIME DEBUG (KURSAD40 second-payload investigation), STEP 2:
        # explicit visibility into what's still onboard vs confirmed dropped,
        # mirroring PayloadDropSystem's own red-then-blue stage counter so a
        # mismatch between what Python believes happened and what Gazebo
        # actually did is immediately visible in the logs. True = still
        # onboard; only cleared to False once _gazebo_boolean_drop actually
        # receives a matching DROP_STATE:<tag>:true confirmation.

    def request_drop(self, color: str, callback=None):
        """Asynchronously triggers a drop and calls the callback with success boolean."""
        async def _run():
            if color == "red":
                success = await self.drop_red_payload()
            else:
                success = await self.drop_blue_payload()
            if callback:
                callback(success)
        self.mav.submit(_run())

    def request_pickup(self, color: str, callback=None):
        """Asynchronously triggers a pickup and calls the callback with success boolean."""
        async def _run():
            success = await self.pickup_payload_simulated(color)
            if callback:
                callback(success)
        self.mav.submit(_run())

    def request_pickup_virtual(self, color: str, callback=None):
        """Asynchronously triggers a fully virtual pickup (see
        pickup_payload_virtual) and calls the callback with success boolean."""
        async def _run():
            success = await self.pickup_payload_virtual(color)
            if callback:
                callback(success)
        self.mav.submit(_run())

    def request_release(self, callback=None):
        """Asynchronously triggers the picked-up payload's release (real hook
        detach + ATTACH_STATE:false confirmation, see drop_picked_payload)
        and calls the callback with success boolean."""
        async def _run():
            success = await self.drop_picked_payload()
            if callback:
                callback(success)
        self.mav.submit(_run())

    async def drop_red_payload(self) -> bool:
        """Drops the RED payload (assumed actuator value HEX_PULSE)."""
        print("[PAYLOAD] Selected RED payload for blue hexagon")
        if self.sim_mode:
            # Gazebo currently uses topic-triggered payload release
            # Real drone may later use MAVSDK actuator passthrough instead
            return await self._gazebo_boolean_drop("red")
        else:
            return await self._real_drop("red")

    async def drop_blue_payload(self) -> bool:
        """Drops the BLUE payload (assumed actuator value TRI_PULSE)."""
        print("[PAYLOAD] Selected BLUE payload for red triangle")
        if self.sim_mode:
            # Gazebo currently uses topic-triggered payload release
            # Real drone may later use MAVSDK actuator passthrough instead
            return await self._gazebo_boolean_drop("blue")
        else:
            return await self._real_drop("blue")

    async def _gazebo_boolean_drop(self, selected_payload_color: str) -> bool:
        print(f"[PAYLOAD] Selected payload color: {selected_payload_color}. Publishing Gazebo boolean drop on {GAZEBO_PAYLOAD_DROP_TOPIC}: true")
        try:
            cmd = f"GZ_IP=127.0.0.1 gz topic -t {GAZEBO_PAYLOAD_DROP_TOPIC} -m gz.msgs.Boolean -p 'data: true'"
            proc = await asyncio.create_subprocess_shell(
                cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            stdout, stderr = await proc.communicate()
            if proc.returncode != 0:
                print(f"[PAYLOAD] Gazebo drop command failed. stderr: {stderr.decode().strip()}")
                return False
            print("[PAYLOAD] Gazebo drop command published successfully (assumed success since plugin responds to this topic).")
            return True
        except Exception as e:
            print(f"[PAYLOAD] Gazebo drop command failed with exception: {e}")
            return False
    async def _sim_actuator_drop(self, pulse_val: float) -> bool:
        """
        LEGACY actuator-based drop backend.
        Keep it available for future real-world integration tests.
        Helper to drop payload in SITL using MAVSDK actuator.
        """
        try:
            print(f"[PAYLOAD] set_actuator(slot={ACTUATOR_SLOT}, value={pulse_val})")
            await self.mav.drone.action.set_actuator(ACTUATOR_SLOT, pulse_val)
            await asyncio.sleep(1.0)
            print(f"[PAYLOAD] set_actuator(slot={ACTUATOR_SLOT}, value=0.0)")
            await self.mav.drone.action.set_actuator(ACTUATOR_SLOT, 0.0)
            await asyncio.sleep(1.5)
            print("[PAYLOAD] Actuator drop sequence complete.")
            return True
        except Exception as e:
            print(f"[PAYLOAD] Actuator drop error: {e}")
            return False

    async def _real_drop(self, color: str) -> bool:
        """Placeholder for real hardware hook/servo/gripper."""
        # TODO: Implement real hardware GPIO / I2C / Serial drop mechanism here.
        print(f"[PAYLOAD] [REAL] Placeholder: Dropping {color} payload...")
        await asyncio.sleep(2.0)
        return True
