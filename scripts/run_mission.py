import asyncio
import argparse
from pathlib import Path

import yaml
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, VelocityNedYaw
from mavsdk.param import ParamError


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "sim_config.yaml"
DEFAULT_SYSTEM_ADDRESS = "udpin://0.0.0.0:14540"
DEFAULT_SETPOINT_INTERVAL_S = 0.2


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


def get_optional_number(
    config,
    key,
    default,
    positive=False,
    non_negative=False,
    path="mission.landing_profile",
):
    value = config.get(key, default)
    if not isinstance(value, (int, float)):
        raise ValueError(f"Config value must be a number: {path}.{key}")
    if positive and value <= 0:
        raise ValueError(f"Config value must be a positive number: {path}.{key}")
    if non_negative and value < 0:
        raise ValueError(f"Config value must be non-negative: {path}.{key}")
    return float(value)


def get_nullable_number(
    config,
    key,
    default=None,
    positive=False,
    non_negative=False,
    path="mission.landing_profile",
):
    value = config.get(key, default)
    if value is None:
        return None
    return get_optional_number(
        config,
        key,
        value,
        positive=positive,
        non_negative=non_negative,
        path=path,
    )


def load_px4_parameters(config):
    parameters = config.get("mission", {}).get("px4_parameters", {})
    if parameters is None:
        return {}
    if not isinstance(parameters, dict):
        raise ValueError("Config value must be a mapping: mission.px4_parameters")

    resolved_parameters = {}
    for name, value in parameters.items():
        if not isinstance(name, str) or not name:
            raise ValueError("PX4 parameter names must be non-empty strings")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(
                f"Config value must be a number: mission.px4_parameters.{name}"
            )
        resolved_parameters[name] = value
    return resolved_parameters


def load_landing_profile(config):
    profile = config.get("mission", {}).get("landing_profile", {})
    if profile is None:
        return {"mode": "standard"}
    if not isinstance(profile, dict):
        raise ValueError("Config value must be a mapping: mission.landing_profile")

    mode = profile.get("mode", "standard")
    if mode not in ("standard", "offboard_body", "offboard_ned"):
        raise ValueError(
            "mission.landing_profile.mode must be standard, offboard_body, or offboard_ned"
        )
    if mode == "standard":
        return {"mode": "standard"}

    resolved_profile = {
        "mode": mode,
        "forward_velocity_m_s": get_optional_number(
            profile,
            "forward_velocity_m_s",
            0.0,
        ),
        "right_velocity_m_s": get_optional_number(
            profile,
            "right_velocity_m_s",
            0.0,
        ),
        "north_velocity_m_s": get_optional_number(
            profile,
            "north_velocity_m_s",
            0.0,
        ),
        "east_velocity_m_s": get_optional_number(
            profile,
            "east_velocity_m_s",
            0.0,
        ),
        "descent_rate_m_s": get_optional_number(
            profile,
            "descent_rate_m_s",
            0.0,
            non_negative=True,
        ),
        "yaw_rate_deg_s": get_optional_number(
            profile,
            "yaw_rate_deg_s",
            0.0,
        ),
        "end_altitude_m": get_nullable_number(
            profile,
            "end_altitude_m",
            None,
            non_negative=True,
        ),
        "duration_s": get_nullable_number(
            profile,
            "duration_s",
            None,
            positive=True,
        ),
        "timeout": get_optional_number(
            profile,
            "timeout",
            120.0,
            positive=True,
        ),
        "setpoint_interval_s": get_optional_number(
            profile,
            "setpoint_interval_s",
            0.2,
            positive=True,
        ),
    }
    if (
        resolved_profile["end_altitude_m"] is None
        and resolved_profile["duration_s"] is None
    ):
        raise ValueError(
            "mission.landing_profile requires duration_s or end_altitude_m"
        )
    return resolved_profile


def load_motion_profile(config):
    profile = config.get("mission", {}).get("motion_profile", {})
    if profile is None:
        return {"mode": "none"}
    if not isinstance(profile, dict):
        raise ValueError("Config value must be a mapping: mission.motion_profile")

    mode = profile.get("mode", "none")
    if mode not in ("none", "offboard_ned"):
        raise ValueError("mission.motion_profile.mode must be none or offboard_ned")
    if mode == "none":
        return {"mode": "none"}

    if "legs" in profile:
        legs = profile["legs"]
        if not isinstance(legs, list) or not legs:
            raise ValueError("mission.motion_profile.legs must be a non-empty list")
        resolved_legs = [
            load_motion_profile_leg(
                leg,
                f"mission.motion_profile.legs[{index}]",
            )
            for index, leg in enumerate(legs)
        ]
        return {
            "mode": mode,
            "legs": resolved_legs,
            "timeout": sum(leg["timeout"] for leg in resolved_legs),
            "setpoint_interval_s": min(
                leg["setpoint_interval_s"] for leg in resolved_legs
            ),
        }

    return load_motion_profile_leg(profile, "mission.motion_profile", mode=mode)


def load_motion_profile_leg(profile, path, mode="offboard_ned"):
    north_m = get_optional_number(
        profile,
        "north_m",
        0.0,
        path=path,
    )
    east_m = get_optional_number(
        profile,
        "east_m",
        0.0,
        path=path,
    )
    distance_m = (north_m**2 + east_m**2) ** 0.5
    duration_s = get_nullable_number(
        profile,
        "duration_s",
        None,
        positive=True,
        path=path,
    )
    horizontal_speed_m_s = get_nullable_number(
        profile,
        "horizontal_speed_m_s",
        None,
        positive=True,
        path=path,
    )
    if duration_s is None and horizontal_speed_m_s is None:
        raise ValueError(
            "mission.motion_profile requires duration_s or horizontal_speed_m_s"
        )
    if duration_s is None:
        if distance_m <= 0:
            raise ValueError(
                "mission.motion_profile horizontal_speed_m_s requires non-zero displacement"
            )
        duration_s = distance_m / horizontal_speed_m_s
    if horizontal_speed_m_s is None:
        horizontal_speed_m_s = distance_m / duration_s if distance_m > 0 else 0.0

    yaw_setpoints = load_yaw_setpoints(
        profile.get("yaw_setpoints"),
        path=f"{path}.yaw_setpoints",
    )

    return {
        "mode": mode,
        "north_m": north_m,
        "east_m": east_m,
        "target_altitude_m": get_nullable_number(
            profile,
            "target_altitude_m",
            None,
            positive=True,
            path=path,
        ),
        "horizontal_speed_m_s": float(horizontal_speed_m_s),
        "duration_s": float(duration_s),
        "yaw_start_deg": get_nullable_number(
            profile,
            "yaw_start_deg",
            None,
            path=path,
        ),
        "yaw_end_deg": get_nullable_number(
            profile,
            "yaw_end_deg",
            None,
            path=path,
        ),
        "yaw_deg": get_nullable_number(
            profile,
            "yaw_deg",
            None,
            path=path,
        ),
        "yaw_rate_deg_s": get_optional_number(
            profile,
            "yaw_rate_deg_s",
            0.0,
            path=path,
        ),
        "yaw_setpoints": yaw_setpoints,
        "timeout": get_optional_number(
            profile,
            "timeout",
            float(duration_s + 15.0),
            positive=True,
            path=path,
        ),
        "setpoint_interval_s": get_optional_number(
            profile,
            "setpoint_interval_s",
            DEFAULT_SETPOINT_INTERVAL_S,
            positive=True,
            path=path,
        ),
    }


def load_yaw_setpoints(setpoints, path):
    if setpoints is None:
        return []
    if not isinstance(setpoints, list):
        raise ValueError(f"Config value must be a list: {path}")

    resolved = []
    previous_time_s = None
    for index, setpoint in enumerate(setpoints):
        setpoint_path = f"{path}[{index}]"
        if not isinstance(setpoint, dict):
            raise ValueError(f"Config value must be a mapping: {setpoint_path}")
        time_s = get_nullable_number(
            setpoint,
            "time_s",
            None,
            non_negative=True,
            path=setpoint_path,
        )
        yaw_deg = get_nullable_number(
            setpoint,
            "yaw_deg",
            None,
            path=setpoint_path,
        )
        if time_s is None or yaw_deg is None:
            raise ValueError(f"{setpoint_path} requires time_s and yaw_deg")
        if previous_time_s is not None and time_s <= previous_time_s:
            raise ValueError(f"{path} time_s values must be strictly increasing")
        previous_time_s = time_s
        resolved.append({"time_s": time_s, "yaw_deg": yaw_deg})
    return resolved


def interpolated_yaw_from_setpoints(yaw_setpoints, elapsed_s):
    if not yaw_setpoints:
        return None
    if elapsed_s <= yaw_setpoints[0]["time_s"]:
        return yaw_setpoints[0]["yaw_deg"]
    if elapsed_s >= yaw_setpoints[-1]["time_s"]:
        return yaw_setpoints[-1]["yaw_deg"]

    for previous_setpoint, next_setpoint in zip(yaw_setpoints, yaw_setpoints[1:]):
        if elapsed_s > next_setpoint["time_s"]:
            continue
        duration_s = next_setpoint["time_s"] - previous_setpoint["time_s"]
        if duration_s <= 0:
            return next_setpoint["yaw_deg"]
        progress = (elapsed_s - previous_setpoint["time_s"]) / duration_s
        return previous_setpoint["yaw_deg"] + (
            next_setpoint["yaw_deg"] - previous_setpoint["yaw_deg"]
        ) * progress

    return yaw_setpoints[-1]["yaw_deg"]


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


async def get_current_position(drone, timeout, description):
    async def read_position():
        async for position in drone.telemetry.position():
            return position

    try:
        return await asyncio.wait_for(read_position(), timeout=timeout)
    except asyncio.TimeoutError as exc:
        raise TimeoutError(f"Timed out waiting for {description}") from exc


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


async def apply_px4_parameters(drone, parameters):
    original_parameters = {}
    for name, value in parameters.items():
        try:
            if isinstance(value, int):
                original_parameters[name] = (
                    "int",
                    await drone.param.get_param_int(name),
                )
                await drone.param.set_param_int(name, value)
            else:
                original_parameters[name] = (
                    "float",
                    await drone.param.get_param_float(name),
                )
                await drone.param.set_param_float(name, float(value))
        except ParamError as exc:
            raise RuntimeError(f"Failed to set PX4 parameter {name}={value}") from exc
        report_event("px4 parameter set", f"{name}={value}")
    return original_parameters


async def restore_px4_parameters(drone, original_parameters):
    for name, (value_type, value) in original_parameters.items():
        try:
            if value_type == "int":
                await drone.param.set_param_int(name, value)
            else:
                await drone.param.set_param_float(name, value)
        except ParamError as exc:
            raise RuntimeError(f"Failed to restore PX4 parameter {name}") from exc
        report_event("px4 parameter restored", f"{name}={value}")


def offboard_profile_is_complete(position, profile, start_time):
    end_altitude = profile["end_altitude_m"]
    duration_s = profile["duration_s"]
    elapsed = asyncio.get_running_loop().time() - start_time
    if end_altitude is not None and position.relative_altitude_m <= end_altitude:
        return True, f"{position.relative_altitude_m:.1f} m"
    if duration_s is not None and elapsed >= duration_s:
        return True, f"{elapsed:.1f} s"
    return False, None


async def run_offboard_body_landing_profile(drone, profile):
    setpoint = VelocityBodyYawspeed(
        profile["forward_velocity_m_s"],
        profile["right_velocity_m_s"],
        profile["descent_rate_m_s"],
        profile["yaw_rate_deg_s"],
    )
    timeout = profile["timeout"]
    interval = profile["setpoint_interval_s"]
    start_time = asyncio.get_running_loop().time()
    deadline = start_time + timeout

    await drone.offboard.set_velocity_body(setpoint)
    try:
        await drone.offboard.start()
    except OffboardError as exc:
        raise RuntimeError(f"Offboard landing profile failed to start: {exc}") from exc

    report_event(
        "offboard landing profile started",
        (
            f"forward={profile['forward_velocity_m_s']} m/s, "
            f"right={profile['right_velocity_m_s']} m/s, "
            f"down={profile['descent_rate_m_s']} m/s, "
            f"yaw_rate={profile['yaw_rate_deg_s']} deg/s"
        ),
    )

    try:
        async for position in drone.telemetry.position():
            await drone.offboard.set_velocity_body(setpoint)
            complete, detail = offboard_profile_is_complete(
                position,
                profile,
                start_time,
            )
            if complete:
                report_event(
                    "offboard landing profile complete",
                    detail,
                )
                return
            if asyncio.get_running_loop().time() >= deadline:
                raise TimeoutError("Timed out waiting for offboard landing profile")
            await asyncio.sleep(interval)
    finally:
        try:
            await drone.offboard.stop()
        except OffboardError as exc:
            report_event("offboard stop warning", str(exc))


async def run_offboard_ned_landing_profile(drone, profile):
    timeout = profile["timeout"]
    interval = profile["setpoint_interval_s"]
    start_time = asyncio.get_running_loop().time()
    deadline = start_time + timeout

    def setpoint():
        elapsed = asyncio.get_running_loop().time() - start_time
        yaw_deg = profile["yaw_rate_deg_s"] * elapsed
        return VelocityNedYaw(
            profile["north_velocity_m_s"],
            profile["east_velocity_m_s"],
            profile["descent_rate_m_s"],
            yaw_deg,
        )

    await drone.offboard.set_velocity_ned(setpoint())
    try:
        await drone.offboard.start()
    except OffboardError as exc:
        raise RuntimeError(f"Offboard landing profile failed to start: {exc}") from exc

    report_event(
        "offboard landing profile started",
        (
            f"north={profile['north_velocity_m_s']} m/s, "
            f"east={profile['east_velocity_m_s']} m/s, "
            f"down={profile['descent_rate_m_s']} m/s, "
            f"yaw_rate={profile['yaw_rate_deg_s']} deg/s"
        ),
    )

    try:
        async for position in drone.telemetry.position():
            await drone.offboard.set_velocity_ned(setpoint())
            complete, detail = offboard_profile_is_complete(
                position,
                profile,
                start_time,
            )
            if complete:
                report_event(
                    "offboard landing profile complete",
                    detail,
                )
                return
            if asyncio.get_running_loop().time() >= deadline:
                raise TimeoutError("Timed out waiting for offboard landing profile")
            await asyncio.sleep(interval)
    finally:
        try:
            await drone.offboard.stop()
        except OffboardError as exc:
            report_event("offboard stop warning", str(exc))


async def run_offboard_ned_motion_profile(drone, profile):
    duration_s = profile["duration_s"]
    interval = profile["setpoint_interval_s"]
    timeout = profile["timeout"]
    position = await get_current_position(
        drone,
        timeout=min(timeout, 10.0),
        description="motion profile start position",
    )
    current_altitude_m = position.relative_altitude_m
    target_altitude_m = profile["target_altitude_m"]
    down_velocity_m_s = 0.0
    if target_altitude_m is not None:
        down_velocity_m_s = (current_altitude_m - target_altitude_m) / duration_s

    north_velocity_m_s = profile["north_m"] / duration_s
    east_velocity_m_s = profile["east_m"] / duration_s
    start_time = asyncio.get_running_loop().time()
    deadline = start_time + timeout

    def setpoint():
        elapsed = asyncio.get_running_loop().time() - start_time
        yaw_start_deg = profile.get("yaw_start_deg")
        yaw_end_deg = profile.get("yaw_end_deg")
        yaw_deg = interpolated_yaw_from_setpoints(
            profile.get("yaw_setpoints", []),
            elapsed,
        )
        if yaw_deg is None and yaw_start_deg is not None and yaw_end_deg is not None:
            progress = min(1.0, max(0.0, elapsed / duration_s))
            yaw_deg = yaw_start_deg + (yaw_end_deg - yaw_start_deg) * progress
        elif yaw_deg is None:
            yaw_deg = profile["yaw_deg"]
        if yaw_deg is None:
            yaw_deg = profile["yaw_rate_deg_s"] * elapsed
        return VelocityNedYaw(
            north_velocity_m_s,
            east_velocity_m_s,
            down_velocity_m_s,
            yaw_deg,
        )

    await drone.offboard.set_velocity_ned(setpoint())
    try:
        await drone.offboard.start()
    except OffboardError as exc:
        raise RuntimeError(f"Offboard motion profile failed to start: {exc}") from exc

    report_event(
        "offboard motion profile started",
        (
            f"north={profile['north_m']} m, east={profile['east_m']} m, "
            f"duration={duration_s:.1f} s, "
            f"target_altitude={target_altitude_m if target_altitude_m is not None else 'current'} m"
        ),
    )

    try:
        while True:
            await drone.offboard.set_velocity_ned(setpoint())
            elapsed = asyncio.get_running_loop().time() - start_time
            if elapsed >= duration_s:
                report_event("offboard motion profile complete", f"{elapsed:.1f} s")
                return
            if asyncio.get_running_loop().time() >= deadline:
                raise TimeoutError("Timed out waiting for offboard motion profile")
            await asyncio.sleep(interval)
    finally:
        try:
            await drone.offboard.stop()
        except OffboardError as exc:
            report_event("offboard stop warning", str(exc))


async def execute_motion(drone, motion_profile):
    if motion_profile["mode"] == "offboard_ned":
        if "legs" in motion_profile:
            for index, leg in enumerate(motion_profile["legs"], start=1):
                report_event("offboard motion leg", f"{index}/{len(motion_profile['legs'])}")
                await run_offboard_ned_motion_profile(drone, leg)
        else:
            await run_offboard_ned_motion_profile(drone, motion_profile)


async def execute_landing(drone, landing_profile, landing_timeout):
    if landing_profile["mode"] == "offboard_body":
        await run_offboard_body_landing_profile(drone, landing_profile)
    elif landing_profile["mode"] == "offboard_ned":
        await run_offboard_ned_landing_profile(drone, landing_profile)

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


async def run(config_path=CONFIG_PATH):
    config = load_config(config_path)
    takeoff_altitude = get_positive_number(config, "mission", "takeoff_altitude")
    hover_time = get_positive_number(config, "mission", "hover_time")
    takeoff_timeout = get_positive_number(config, "mission", "takeoff_timeout")
    climb_timeout = get_positive_number(config, "mission", "climb_timeout")
    altitude_tolerance = get_positive_number(config, "mission", "altitude_tolerance")
    landing_timeout = get_positive_number(config, "mission", "landing_timeout")
    landing_profile = load_landing_profile(config)
    motion_profile = load_motion_profile(config)
    px4_parameters = load_px4_parameters(config)
    connection_timeout = get_positive_number(config, "connection", "timeout")
    preflight_timeout = get_positive_number(config, "connection", "preflight_timeout")
    arm_timeout = get_positive_number(config, "connection", "arm_timeout")

    try:
        drone = System()
        await drone.connect(system_address=DEFAULT_SYSTEM_ADDRESS)

        print("Bağlanıyor...", flush=True)
        await wait_until_connected(drone, connection_timeout)
        report_event("connected")

        print("Preflight readiness bekleniyor...", flush=True)
        await wait_until_preflight_ready(drone, preflight_timeout)
        report_event("preflight ready")

        original_parameters = await apply_px4_parameters(drone, px4_parameters)

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

        await execute_motion(drone, motion_profile)

        await execute_landing(drone, landing_profile, landing_timeout)
    finally:
        if "drone" in locals():
            try:
                if "original_parameters" in locals():
                    await restore_px4_parameters(drone, original_parameters)
            finally:
                drone._stop_mavsdk_server()

if __name__ == "__main__":
    args = parse_args()
    asyncio.run(run(args.config))
