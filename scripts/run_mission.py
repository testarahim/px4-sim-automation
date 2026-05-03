import asyncio
import argparse
from pathlib import Path

import yaml
from mavsdk import System
from mavsdk.action import ActionError


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "sim_config.yaml"
DEFAULT_SYSTEM_ADDRESS = "udpin://0.0.0.0:14540"


def parse_args():
    parser = argparse.ArgumentParser(description="Run a PX4 SITL mission.")
    parser.add_argument(
        "--config",
        type=Path,
        default=CONFIG_PATH,
        help=f"Mission config/scenario YAML path. Default: {CONFIG_PATH}",
    )
    return parser.parse_args()


def report_event(event, detail=None):
    message = f"[mission] {event}"
    if detail:
        message = f"{message}: {detail}"
    print(message, flush=True)


def load_config(config_path=CONFIG_PATH):
    with config_path.open("r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def get_positive_number(config, section, key):
    try:
        value = config[section][key]
    except KeyError as exc:
        raise ValueError(f"Missing config value: {section}.{key}") from exc

    if not isinstance(value, (int, float)) or value <= 0:
        raise ValueError(f"Config value must be a positive number: {section}.{key}")

    return value


async def wait_until_connected(drone, timeout):
    async def wait_for_connection_state():
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    try:
        await asyncio.wait_for(wait_for_connection_state(), timeout=timeout)
    except asyncio.TimeoutError as exc:
        raise TimeoutError(f"Drone connection timed out after {timeout} seconds") from exc


async def wait_for_telemetry_value(stream, expected_value, timeout, description):
    async def wait_for_value():
        async for value in stream:
            if value == expected_value:
                return

    try:
        await asyncio.wait_for(wait_for_value(), timeout=timeout)
    except asyncio.TimeoutError as exc:
        raise TimeoutError(f"Timed out waiting for {description}") from exc


async def wait_until_altitude_reached(drone, target_altitude, tolerance, timeout):
    minimum_altitude = target_altitude - tolerance

    async def wait_for_altitude():
        async for position in drone.telemetry.position():
            if position.relative_altitude_m >= minimum_altitude:
                return position.relative_altitude_m

    try:
        return await asyncio.wait_for(wait_for_altitude(), timeout=timeout)
    except asyncio.TimeoutError as exc:
        raise TimeoutError(
            f"Timed out waiting to reach {minimum_altitude:.1f} m relative altitude"
        ) from exc


async def wait_until_preflight_ready(drone, timeout):
    async def wait_for_health():
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                return

    try:
        await asyncio.wait_for(wait_for_health(), timeout=timeout)
    except asyncio.TimeoutError as exc:
        raise TimeoutError(f"Preflight readiness timed out after {timeout} seconds") from exc


async def arm_with_retry(drone, timeout):
    deadline = asyncio.get_running_loop().time() + timeout
    last_error = None

    while asyncio.get_running_loop().time() < deadline:
        try:
            await drone.action.arm()
            return
        except ActionError as exc:
            last_error = exc
            print(f"Arm reddedildi, tekrar denenecek: {exc}", flush=True)
            await asyncio.sleep(1)

    raise TimeoutError(f"Arming timed out after {timeout} seconds") from last_error


async def run(config_path=CONFIG_PATH):
    config = load_config(config_path)
    takeoff_altitude = get_positive_number(config, "mission", "takeoff_altitude")
    hover_time = get_positive_number(config, "mission", "hover_time")
    takeoff_timeout = get_positive_number(config, "mission", "takeoff_timeout")
    climb_timeout = get_positive_number(config, "mission", "climb_timeout")
    altitude_tolerance = get_positive_number(config, "mission", "altitude_tolerance")
    landing_timeout = get_positive_number(config, "mission", "landing_timeout")
    connection_timeout = get_positive_number(config, "connection", "timeout")
    preflight_timeout = get_positive_number(config, "connection", "preflight_timeout")
    arm_timeout = get_positive_number(config, "connection", "arm_timeout")

    drone = System()
    await drone.connect(system_address=DEFAULT_SYSTEM_ADDRESS)

    print("Bağlanıyor...", flush=True)
    await wait_until_connected(drone, connection_timeout)
    report_event("connected")

    print("Preflight readiness bekleniyor...", flush=True)
    await wait_until_preflight_ready(drone, preflight_timeout)
    report_event("preflight ready")

    print("Arm ediliyor...", flush=True)
    await arm_with_retry(drone, arm_timeout)
    report_event("armed")

    print(f"Takeoff altitude set ediliyor: {takeoff_altitude} m", flush=True)
    await drone.action.set_takeoff_altitude(takeoff_altitude)

    print("Takeoff...", flush=True)
    await drone.action.takeoff()
    await wait_for_telemetry_value(
        drone.telemetry.in_air(),
        True,
        takeoff_timeout,
        "takeoff detection",
    )
    report_event("takeoff detected")

    print(f"Hedef irtifa bekleniyor: {takeoff_altitude} m", flush=True)
    reached_altitude = await wait_until_altitude_reached(
        drone,
        takeoff_altitude,
        altitude_tolerance,
        climb_timeout,
    )
    report_event("target altitude reached", f"{reached_altitude:.1f} m")

    print(f"Hover bekleniyor: {hover_time} s", flush=True)
    await asyncio.sleep(hover_time)

    print("Landing...", flush=True)
    await drone.action.land()
    await wait_for_telemetry_value(
        drone.telemetry.in_air(),
        False,
        landing_timeout,
        "landing detection",
    )
    report_event("landing detected")
    await wait_for_telemetry_value(
        drone.telemetry.armed(),
        False,
        landing_timeout,
        "disarm after landing",
    )
    report_event("disarmed")
    print("Landing tamamlandı.", flush=True)

if __name__ == "__main__":
    args = parse_args()
    asyncio.run(run(args.config))
