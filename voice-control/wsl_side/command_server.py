"""
Stage 3 (expanded): Command server (run this in WSL)
Receives voice commands over HTTP and executes them on the SITL drone via MAVSDK.

Supported commands (simple keyword matching):
  - "take off" / "takeoff"        -> arm and take off to 10m
  - "land"                        -> land
  - "hover" / "hold" / "stop"     -> hold current position
  - "return home" / "come back" / "rtl" -> return to launch point and land
  - "go to <name>" / "fly to <name>"    -> fly to a predefined named location
  - "go forward" / "go back" / "go left" / "go right" -> move ~20m relative to current position

Run:
  python3 command_server.py
"""

import asyncio
import math
import re
import threading
from flask import Flask, request, jsonify
from mavsdk import System

app = Flask(__name__)

# --- Shared state between Flask thread and asyncio drone thread ---
drone = System()
drone_loop = None
drone_ready = threading.Event()
# --- Home position override ---
# Leave these as None to auto-capture wherever SITL spawns/arms.
# Or set real numbers here to force a specific home location regardless of SITL's spawn point.
OVERRIDE_HOME_LAT = None   # e.g. -35.360000
OVERRIDE_HOME_LON = None   # e.g. 149.170000
OVERRIDE_HOME_ALT = None   # e.g. 0 (meters, absolute altitude)

home_position = {"lat": None, "lon": None, "alt": None}

# --- Named locations, defined as (north_offset_m, east_offset_m) from home ---
# Add more here as you need -- values are meters relative to takeoff/home point.
LOCATIONS = {
    "point a": (30, 0),
    "point b": (30, 30),
    "base": (0, 0),
}

MOVE_STEP_M = 20  # how far each relative move command travels


def start_drone_loop():
    """Runs in a background thread: owns the asyncio event loop and the drone connection."""
    global drone_loop
    drone_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(drone_loop)
    drone_loop.run_until_complete(connect_drone())
    drone_loop.run_forever()


async def connect_drone():
    print("Connecting to drone...")
    await drone.connect(system_address="udp://:14551")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Drone connected")
            break

    print("Waiting for GPS lock...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- GPS lock good")
            break

    # Capture home position once, used as the reference point for named locations.
    # Using telemetry.position() instead of telemetry.home() -- more reliable in SITL,
    # and since the drone hasn't moved yet, current position = home position.
    print("Capturing home position...")
    async for position in drone.telemetry.position():
        home_position["lat"] = OVERRIDE_HOME_LAT if OVERRIDE_HOME_LAT is not None else position.latitude_deg
        home_position["lon"] = OVERRIDE_HOME_LON if OVERRIDE_HOME_LON is not None else position.longitude_deg
        home_position["alt"] = OVERRIDE_HOME_ALT if OVERRIDE_HOME_ALT is not None else position.absolute_altitude_m
        print(f"-- Home position set to: {home_position}")
        break

    print("-- Ready for commands")
    drone_ready.set()


def run_coroutine(coro, timeout=25):
    """Safely schedule an async drone action from Flask's sync thread and wait for the result."""
    future = asyncio.run_coroutine_threadsafe(coro, drone_loop)
    return future.result(timeout=timeout)


def offset_latlon(base_lat, base_lon, north_m, east_m):
    """Convert a north/east meter offset into a new lat/lon (approximation, fine for short distances)."""
    delta_lat = north_m / 111320
    delta_lon = east_m / (111320 * math.cos(math.radians(base_lat)))
    return base_lat + delta_lat, base_lon + delta_lon


def extract_altitude(text, default):
    """Look for a number followed by 'm' or 'meter(s)' in the text, e.g. 'take off to 25 meters'."""
    match = re.search(r'(\d+)\s*(meters|meter|m)\b', text)
    if match:
        return float(match.group(1))
    return default


# --- Drone actions ---
async def do_takeoff(altitude=10):
    print(f"-- Arming")
    await drone.action.arm()
    print(f"-- Taking off to {altitude}m")
    await drone.action.set_takeoff_altitude(altitude)
    await drone.action.takeoff()
    return f"Taking off to {altitude}m"


async def do_land():
    print("-- Landing")
    await drone.action.land()
    return "Landing"


async def do_hold():
    print("-- Holding position")
    await drone.action.hold()
    return "Holding position"


async def do_return_home():
    print("-- Returning to launch")
    await drone.action.return_to_launch()
    return "Returning home"


async def do_goto_named(name, altitude=None):
    north_m, east_m = LOCATIONS[name]
    target_alt = home_position["alt"] + (altitude if altitude else 15)
    lat, lon = offset_latlon(home_position["lat"], home_position["lon"], north_m, east_m)
    print(f"-- Flying to '{name}' ({lat:.6f}, {lon:.6f}) at {target_alt}m")
    await drone.action.goto_location(lat, lon, target_alt, 0)
    return f"Flying to {name}"


async def do_change_altitude(delta_m):
    current = None
    async for position in drone.telemetry.position():
        current = position
        break
    new_alt = current.absolute_altitude_m + delta_m
    print(f"-- Changing altitude by {delta_m}m -> {new_alt:.1f}m")
    await drone.action.goto_location(current.latitude_deg, current.longitude_deg, new_alt, 0)
    return f"Altitude changed by {delta_m}m"


async def do_move_relative(north_m, east_m):
    current = None
    async for position in drone.telemetry.position():
        current = position
        break
    lat, lon = offset_latlon(current.latitude_deg, current.longitude_deg, north_m, east_m)
    print(f"-- Moving by north={north_m}m east={east_m}m -> ({lat:.6f}, {lon:.6f})")
    await drone.action.goto_location(lat, lon, current.absolute_altitude_m, 0)
    return f"Moving {north_m}m north, {east_m}m east"


# --- Command interpretation ---
def interpret_and_run(text):
    text = text.lower()

    if "take off" in text or "takeoff" in text:
        altitude = extract_altitude(text, default=10)
        return run_coroutine(do_takeoff(altitude))

    elif "land" in text:
        return run_coroutine(do_land())

    elif "hover" in text or "hold" in text or "stop" in text:
        return run_coroutine(do_hold())

    elif "return home" in text or "come back" in text or "rtl" in text or "return to launch" in text:
        return run_coroutine(do_return_home())

    elif "go up" in text or "climb" in text or "go higher" in text or "increase altitude" in text:
        step = extract_altitude(text, default=5)
        return run_coroutine(do_change_altitude(step))

    elif "go down" in text or "descend" in text or "go lower" in text or "decrease altitude" in text:
        step = extract_altitude(text, default=5)
        return run_coroutine(do_change_altitude(-step))

    elif "go to" in text or "fly to" in text:
        altitude = extract_altitude(text, default=None)
        for name in LOCATIONS:
            if name in text:
                return run_coroutine(do_goto_named(name, altitude))
        return f"Named location not recognized in: '{text}'. Known: {list(LOCATIONS.keys())}"

    elif "forward" in text:
        return run_coroutine(do_move_relative(MOVE_STEP_M, 0))
    elif "back" in text:
        return run_coroutine(do_move_relative(-MOVE_STEP_M, 0))
    elif "left" in text:
        return run_coroutine(do_move_relative(0, -MOVE_STEP_M))
    elif "right" in text:
        return run_coroutine(do_move_relative(0, MOVE_STEP_M))

    else:
        return f"Command not recognized: '{text}'"


@app.route("/command", methods=["POST"])
def receive_command():
    data = request.get_json()
    command_text = data.get("command", "").strip()
    print(f"[RECEIVED] '{command_text}'")

    if not drone_ready.is_set():
        return jsonify({"status": "not_ready", "message": "Drone not connected yet"}), 503

    try:
        result = interpret_and_run(command_text)
        return jsonify({"status": "ok", "result": result})
    except Exception as e:
        print(f"Error executing command: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500


if __name__ == "__main__":
    threading.Thread(target=start_drone_loop, daemon=True).start()

    print("Waiting for drone connection before accepting commands...")
    drone_ready.wait()

    print("Command server listening on http://0.0.0.0:5005")
    app.run(host="0.0.0.0", port=5005)