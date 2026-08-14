import asyncio, subprocess
from mavsdk import System

DROP_CMD = ["gz", "topic", "-t", "/payload_drop", "-m", "gz.msgs.Boolean", "-p", "data: true"]

async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    async for s in drone.core.connection_state():
        if s.is_connected:
            break

    input("ENTER -> DROP\n")
    subprocess.run(DROP_CMD, check=False)

asyncio.run(main())
