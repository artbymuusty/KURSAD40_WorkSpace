"""Standalone MAVSDK arming-health probe, meant to run as an isolated subprocess.

Must never be imported into the same interpreter as gz.transport13 (see
process_manager.py's check_health()): gz-transport13's protobuf runtime and
mavsdk's generated protobuf stubs are mutually incompatible once both get
imported into one process (whichever 'google.protobuf' resolves first wins,
breaking the other's import). camera_service.py/hook_sensor_service.py are
already isolated from process_manager.py for this exact reason; this script
gives verify_vehicle_armable() the same isolation instead of importing
mavsdk directly into process_manager.py.
"""
import asyncio
import sys

from mavsdk import System
from config import DEFAULT_MAVSDK_URL


async def main(timeout_s: float):
    async def _check():
        drone = System()
        await drone.connect(system_address=DEFAULT_MAVSDK_URL)
        async for state in drone.core.connection_state():
            if state.is_connected:
                break
        async for health in drone.telemetry.health():
            return health.is_armable

    result = await asyncio.wait_for(_check(), timeout=timeout_s)
    print(f"ARMABLE={result}")


if __name__ == "__main__":
    timeout_s = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
    try:
        asyncio.run(main(timeout_s))
    except Exception as e:
        print(f"ARMABLE=UNKNOWN error={e}")
