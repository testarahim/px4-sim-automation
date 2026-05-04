import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml
from pyulog import ULog


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_JSON = PROJECT_DIR / "results" / "real_log_profile.json"
DEFAULT_OUTPUT_SCENARIO = PROJECT_DIR / "scenarios" / "generated_from_real.yaml"
DEFAULT_GROUND_ALTITUDE_M = 0.3
DEFAULT_TAKEOFF_THRESHOLD_M = 1.0
DEFAULT_ALTITUDE_TOLERANCE_M = 1.2
DEFAULT_SETPOINT_INTERVAL_S = 0.2


def parse_args():
    parser = argparse.ArgumentParser(
        description="Extract a mission profile from a real PX4 .ulg log."
    )
    parser.add_argument("log", type=Path, help="Real PX4 .ulg log to profile.")
    parser.add_argument(
        "--output-json",
        type=Path,
        default=DEFAULT_OUTPUT_JSON,
        help=f"Profile JSON output path. Default: {DEFAULT_OUTPUT_JSON}",
    )
    parser.add_argument(
        "--output-scenario",
        type=Path,
        default=None,
        help="Optional scenario YAML output path generated from the profile.",
    )
    parser.add_argument(
        "--ground-altitude-m",
        type=float,
        default=DEFAULT_GROUND_ALTITUDE_M,
        help="Altitude considered landed/near ground.",
    )
    parser.add_argument(
        "--takeoff-threshold-m",
        type=float,
        default=DEFAULT_TAKEOFF_THRESHOLD_M,
        help="Altitude threshold used to detect takeoff.",
    )
    parser.add_argument(
        "--altitude-tolerance-m",
        type=float,
        default=DEFAULT_ALTITUDE_TOLERANCE_M,
        help="Scenario altitude tolerance for generated YAML.",
    )
    return parser.parse_args()


def resolve_path(path):
    return path if path.is_absolute() else PROJECT_DIR / path


def relative_time(timestamp_us):
    time_s = timestamp_us * 1e-6
    return time_s - time_s[0]


def load_local_position(log_path):
    ulog = ULog(str(log_path))
    data = ulog.get_dataset("vehicle_local_position")
    time_s = relative_time(data.data["timestamp"])
    return {
        "time_s": time_s,
        "x_m": data.data["x"],
        "y_m": data.data["y"],
        "altitude_m": -data.data["z"],
        "vx_m_s": data.data["vx"],
        "vy_m_s": data.data["vy"],
        "vz_m_s": data.data["vz"],
    }


def first_index_at_or_above(values, threshold, start_index=0):
    matching_indexes = np.flatnonzero(values[start_index:] >= threshold)
    if matching_indexes.size == 0:
        return None
    return int(start_index + matching_indexes[0])


def first_index_at_or_below(values, threshold, start_index=0):
    matching_indexes = np.flatnonzero(values[start_index:] <= threshold)
    if matching_indexes.size == 0:
        return None
    return int(start_index + matching_indexes[0])


def finite_or_none(value):
    if value is None or not np.isfinite(value):
        return None
    return float(value)


def mean_or_none(values):
    if values.size == 0:
        return None
    return finite_or_none(np.mean(values))


def percentile_or_none(values, percentile):
    if values.size == 0:
        return None
    return finite_or_none(np.percentile(values, percentile))


def segment_slice(start_index, end_index):
    if start_index is None or end_index is None or end_index <= start_index:
        return slice(0, 0)
    return slice(start_index, end_index + 1)


def compute_displacement(x_m, y_m, start_index, end_index):
    if start_index is None or end_index is None or end_index <= start_index:
        return {
            "north_m": 0.0,
            "east_m": 0.0,
            "distance_m": 0.0,
            "heading_deg": None,
        }

    north_m = float(x_m[end_index] - x_m[start_index])
    east_m = float(y_m[end_index] - y_m[start_index])
    distance_m = float(math.hypot(north_m, east_m))
    heading_deg = None
    if distance_m > 0:
        heading_deg = float((math.degrees(math.atan2(east_m, north_m)) + 360) % 360)
    return {
        "north_m": north_m,
        "east_m": east_m,
        "distance_m": distance_m,
        "heading_deg": heading_deg,
    }


def compute_real_profile(
    time_s,
    x_m,
    y_m,
    altitude_m,
    vx_m_s,
    vy_m_s,
    vz_m_s,
    ground_altitude_m=DEFAULT_GROUND_ALTITUDE_M,
    takeoff_threshold_m=DEFAULT_TAKEOFF_THRESHOLD_M,
):
    sizes = {
        len(time_s),
        len(x_m),
        len(y_m),
        len(altitude_m),
        len(vx_m_s),
        len(vy_m_s),
        len(vz_m_s),
    }
    if sizes == {0}:
        raise ValueError("Local position series is empty")
    if len(sizes) != 1:
        raise ValueError("Local position series lengths do not match")

    duration_s = float(time_s[-1] - time_s[0])
    max_altitude_m = float(np.max(altitude_m))
    target_altitude_m = float(np.percentile(altitude_m, 95))
    target_floor_m = max(takeoff_threshold_m, target_altitude_m * 0.9)

    takeoff_index = first_index_at_or_above(altitude_m, takeoff_threshold_m)
    target_reached_index = first_index_at_or_above(
        altitude_m,
        target_floor_m,
        takeoff_index or 0,
    )
    max_index = int(np.argmax(altitude_m))
    landing_index = first_index_at_or_below(
        altitude_m,
        ground_altitude_m,
        max_index,
    )
    landing_detected = landing_index is not None

    if landing_index is None:
        landing_index = len(time_s) - 1

    cruise_slice = segment_slice(target_reached_index, landing_index)
    airborne_slice = segment_slice(takeoff_index, landing_index)
    horizontal_speed = np.sqrt(vx_m_s**2 + vy_m_s**2)
    vertical_speed_down = vz_m_s

    cruise_displacement = compute_displacement(
        x_m,
        y_m,
        target_reached_index,
        landing_index,
    )
    total_displacement = compute_displacement(
        x_m,
        y_m,
        takeoff_index,
        landing_index,
    )

    takeoff_time_s = time_s[takeoff_index] if takeoff_index is not None else None
    target_reached_time_s = (
        time_s[target_reached_index] if target_reached_index is not None else None
    )
    landing_time_s = time_s[landing_index] if landing_index is not None else None

    cruise_duration_s = None
    if target_reached_time_s is not None and landing_time_s is not None:
        cruise_duration_s = max(0.0, float(landing_time_s - target_reached_time_s))

    profile = {
        "duration_s": duration_s,
        "takeoff_detected": takeoff_index is not None,
        "landing_detected": landing_detected,
        "takeoff_time_s": finite_or_none(takeoff_time_s),
        "target_reached_time_s": finite_or_none(target_reached_time_s),
        "landing_time_s": finite_or_none(landing_time_s),
        "initial_altitude_m": float(altitude_m[0]),
        "target_altitude_m": target_altitude_m,
        "max_altitude_m": max_altitude_m,
        "final_altitude_m": float(altitude_m[-1]),
        "ground_altitude_m": float(ground_altitude_m),
        "takeoff_threshold_m": float(takeoff_threshold_m),
        "airborne_duration_s": (
            finite_or_none(time_s[landing_index] - time_s[takeoff_index])
            if takeoff_index is not None and landing_index is not None
            else None
        ),
        "cruise_duration_s": finite_or_none(cruise_duration_s),
        "horizontal_speed_mean_m_s": mean_or_none(horizontal_speed[airborne_slice]),
        "horizontal_speed_p95_m_s": percentile_or_none(
            horizontal_speed[airborne_slice],
            95,
        ),
        "cruise_horizontal_speed_mean_m_s": mean_or_none(
            horizontal_speed[cruise_slice]
        ),
        "descent_rate_mean_m_s": mean_or_none(
            vertical_speed_down[cruise_slice][vertical_speed_down[cruise_slice] > 0]
        ),
        "cruise_displacement": cruise_displacement,
        "total_displacement": total_displacement,
    }
    return profile


def positive_or_default(value, default):
    if value is None or not np.isfinite(value) or value <= 0:
        return default
    return float(value)


def build_scenario_from_profile(
    profile,
    altitude_tolerance_m=DEFAULT_ALTITUDE_TOLERANCE_M,
):
    target_altitude = max(1.0, round(profile["target_altitude_m"], 1))
    cruise_duration = positive_or_default(profile.get("cruise_duration_s"), 10.0)
    displacement = profile.get("cruise_displacement", {})
    distance = positive_or_default(displacement.get("distance_m"), 0.0)
    cruise_speed = positive_or_default(
        profile.get("cruise_horizontal_speed_mean_m_s"),
        0.0,
    )

    landing_profile = None
    hover_time = max(3.0, min(cruise_duration, 10.0))
    if distance > 1.0 and cruise_speed > 0.1:
        move_duration = max(1.0, distance / cruise_speed)
        north_velocity = displacement.get("north_m", 0.0) / move_duration
        east_velocity = displacement.get("east_m", 0.0) / move_duration
        hover_time = max(3.0, cruise_duration - move_duration)
        landing_profile = {
            "mode": "offboard_ned",
            "north_velocity_m_s": round(float(north_velocity), 3),
            "east_velocity_m_s": round(float(east_velocity), 3),
            "descent_rate_m_s": 0.0,
            "yaw_rate_deg_s": 0.0,
            "duration_s": round(float(move_duration), 1),
            "timeout": round(float(move_duration + 15.0), 1),
            "setpoint_interval_s": DEFAULT_SETPOINT_INTERVAL_S,
        }

    mission = {
        "takeoff_altitude": target_altitude,
        "hover_time": round(float(hover_time), 1),
        "takeoff_timeout": 20,
        "climb_timeout": max(45, int(target_altitude * 3)),
        "altitude_tolerance": float(altitude_tolerance_m),
        "landing_timeout": max(60, int(target_altitude * 4)),
    }
    if landing_profile is not None:
        mission["landing_profile"] = landing_profile

    return {
        "sitl": {
            "startup_timeout": 90,
            "startup_ready_patterns": ["Startup script returned successfully"],
        },
        "connection": {
            "timeout": 30,
            "preflight_timeout": 45,
            "arm_timeout": 20,
        },
        "mission": mission,
        "paths": {
            "px4_log_dir": "~/PX4-Autopilot/build/px4_sitl_default/rootfs/log",
        },
        "thresholds": {
            "max_altitude_error_m": 2.0,
            "hover_altitude_rmse_m": 2.0,
            "max_landing_final_altitude_m": 0.3,
            "velocity_rmse": 1.0,
            "attitude_rmse": 5.0,
        },
    }


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output_file:
        json.dump(data, output_file, indent=4)


def write_yaml(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output_file:
        yaml.safe_dump(data, output_file, sort_keys=False)


def main():
    args = parse_args()
    if args.ground_altitude_m < 0:
        raise ValueError("--ground-altitude-m must be non-negative")
    if args.takeoff_threshold_m < 0:
        raise ValueError("--takeoff-threshold-m must be non-negative")
    if args.altitude_tolerance_m <= 0:
        raise ValueError("--altitude-tolerance-m must be positive")

    log_path = resolve_path(args.log)
    output_json = resolve_path(args.output_json)
    local_position = load_local_position(log_path)
    profile = compute_real_profile(
        ground_altitude_m=args.ground_altitude_m,
        takeoff_threshold_m=args.takeoff_threshold_m,
        **local_position,
    )
    profile["log"] = str(log_path)
    profile["log_id"] = log_path.stem
    write_json(output_json, profile)
    print(f"Saved profile JSON: {output_json}")

    if args.output_scenario:
        output_scenario = resolve_path(args.output_scenario)
        scenario = build_scenario_from_profile(
            profile,
            altitude_tolerance_m=args.altitude_tolerance_m,
        )
        write_yaml(output_scenario, scenario)
        print(f"Saved generated scenario: {output_scenario}")


if __name__ == "__main__":
    main()
