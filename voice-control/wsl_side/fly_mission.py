"""
Basic drone mission: arm -> takeoff -> fly to location -> land
Connects to ArduPilot SITL running in another terminal.
"""

import asyncio
from mavsdk import System


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14551")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("Waiting for global position estimate (GPS lock)...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- GPS lock good, home position set")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.set_takeoff_altitude(30)  # meters
    await drone.action.takeoff()
    await asyncio.sleep(15)  # give it time to climb

    # --- CHANGE THESE COORDINATES to somewhere near your SITL start location ---
    target_lat = -85.363000
    target_lon = 49.166000
    target_alt = 30  # meters above takeoff point

    print(f"-- Flying to {target_lat}, {target_lon} at {target_alt}m")
    await drone.action.goto_location(target_lat, target_lon, target_alt, 0)

    # Wait until it's close to the target before landing
    async for position in drone.telemetry.position():
        distance = abs(position.latitude_deg - target_lat) + abs(position.longitude_deg - target_lon)
        if distance < 0.0002:  # roughly ~20m, close enough for demo purposes
            print("-- Reached target location")
            break
        await asyncio.sleep(1)

    await asyncio.sleep(3)

    print("-- Landing")
    await drone.action.land()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("-- Landed!")
            break
        await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(run())	
