"""ADR-010 R3: clear a stuck PX4 LAND mode so the next mission can arm.

After a mission lands, PX4 stays in flight_mode=LAND even once it is
disarmed and ON_GROUND, and refuses to arm from there: telemetry.health()
reports is_armable=False while every individual pre-arm check
(gyro/accel/mag/local-pos/global-pos/home) reports True. Three runs on
2026-08-17 died on exactly this, each after ~3 minutes of futile waiting.
Commanding HOLD takes it out of LAND and it becomes armable in ~2s.

Deliberately a launcher/hygiene tool and NOT part of the mission code: the
mission must never silently re-arm a vehicle that an operator has left in
LAND. Resetting the rig between runs is the operator's action, so it lives
with the other rig-reset steps in safe_sitl_launcher.sh.

Exit codes: 0 = armable (or already was), 1 = still not armable.
"""
import asyncio
import sys
import time

from mavsdk import System

CONNECTION = "udp://:14540"
CONNECT_TIMEOUT_S = 15.0
ARMABLE_TIMEOUT_S = 60.0


async def _state(drone):
    async for health in drone.telemetry.health():
        armable = health.is_armable
        break
    async for armed in drone.telemetry.armed():
        is_armed = armed
        break
    async for landed in drone.telemetry.landed_state():
        on_ground = "ON_GROUND" in str(landed)
        break
    async for mode in drone.telemetry.flight_mode():
        return armable, str(mode), is_armed, on_ground
    return armable, "UNKNOWN", is_armed, on_ground


async def main() -> int:
    drone = System()
    await drone.connect(system_address=CONNECTION)

    deadline = time.time() + CONNECT_TIMEOUT_S
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
        if time.time() > deadline:
            print("[LAND-RESET] no vehicle on %s -- nothing to do." % CONNECTION)
            return 0

    armable, mode, is_armed, on_ground = await _state(drone)
    print("[LAND-RESET] armable=%s mode=%s armed=%s on_ground=%s" % (armable, mode, is_armed, on_ground))
    if armable:
        return 0

    # The mode label is not a reliable gate on its own: right after a
    # mission it can still read OFFBOARD for a moment while PX4 transitions
    # into LAND, and a run started in that window is refused arming anyway.
    # What actually makes a reset safe is that the vehicle is DISARMED and
    # ON THE GROUND -- then HOLD is the correct way to clear whatever
    # terminal mode it is parked in.
    if is_armed or not on_ground:
        print("[LAND-RESET] vehicle is armed or airborne -- refusing to touch it.")
        return 1

    print("[LAND-RESET] disarmed + on ground but not armable (mode=%s) -- commanding HOLD..." % mode)
    try:
        await drone.action.hold()
    except Exception as e:  # noqa: BLE001 -- report and let the caller decide
        print("[LAND-RESET] hold() rejected: %s" % e)
        return 1

    started = time.time()
    while time.time() - started < ARMABLE_TIMEOUT_S:
        armable, mode, _is_armed, _on_ground = await _state(drone)
        if armable:
            print("[LAND-RESET] armable after %.0fs (mode=%s)" % (time.time() - started, mode))
            return 0
        await asyncio.sleep(2)

    print("[LAND-RESET] STILL not armable after %.0fs (mode=%s)" % (ARMABLE_TIMEOUT_S, mode))
    return 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
