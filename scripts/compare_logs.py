import argparse
import json
import os
import tempfile
from pathlib import Path

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(tempfile.gettempdir()) / "px4_sim_matplotlib"),
)

import matplotlib.pyplot as plt
import numpy as np
import yaml
from pyulog import ULog
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SIM_LOG = PROJECT_DIR / "data" / "sim" / "sim_log.ulg"
DEFAULT_REAL_LOG = PROJECT_DIR / "data" / "real" / "real_log.ulg"
DEFAULT_METRICS = PROJECT_DIR / "results" / "compare_metrics.json"
DEFAULT_PLOT_DIR = PROJECT_DIR / "results" / "plots" / "compare"
DEFAULT_THRESHOLDS = {
    "altitude_rmse": 1.5,
    "velocity_rmse": 1.0,
    "attitude_rmse": 5.0,
}
DEFAULT_TAKEOFF_THRESHOLD_M = 0.3
MIN_SEGMENT_SAMPLES = 2


def parse_args():
    parser = argparse.ArgumentParser(description="Compare simulation and real PX4 logs.")
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Optional scenario/config YAML path used for comparison thresholds.",
    )
    parser.add_argument(
        "--sim",
        type=Path,
        default=DEFAULT_SIM_LOG,
        help=f"Simulation .ulg path. Default: {DEFAULT_SIM_LOG}",
    )
    parser.add_argument(
        "--real",
        type=Path,
        default=DEFAULT_REAL_LOG,
        help=f"Real flight .ulg path. Default: {DEFAULT_REAL_LOG}",
    )
    parser.add_argument(
        "--metrics",
        type=Path,
        default=DEFAULT_METRICS,
        help=f"Metrics JSON output path. Default: {DEFAULT_METRICS}",
    )
    parser.add_argument(
        "--plot-dir",
        type=Path,
        default=DEFAULT_PLOT_DIR,
        help=f"Comparison plot directory. Default: {DEFAULT_PLOT_DIR}",
    )
    parser.add_argument(
        "--altitude-threshold",
        type=float,
        default=None,
        help="Altitude RMSE pass threshold in meters. Overrides --config.",
    )
    parser.add_argument(
        "--velocity-threshold",
        type=float,
        default=None,
        help="Velocity RMSE pass threshold in m/s. Overrides --config.",
    )
    parser.add_argument(
        "--attitude-threshold",
        type=float,
        default=None,
        help="Roll/pitch/yaw RMSE pass threshold in degrees. Overrides --config.",
    )
    parser.add_argument(
        "--alignment",
        choices=("time-zero", "takeoff"),
        default="time-zero",
        help="Time alignment method used before comparing series.",
    )
    parser.add_argument(
        "--takeoff-threshold-m",
        type=float,
        default=None,
        help=(
            "Altitude threshold used by --alignment takeoff. Defaults to "
            "scenario-derived threshold or 0.3 m."
        ),
    )
    return parser.parse_args()


def load_config(config_path):
    if config_path is None:
        return {}
    if not config_path.exists():
        raise FileNotFoundError(f"Config not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def threshold_value(thresholds, keys, default):
    for key in keys:
        value = thresholds.get(key)
        if value is None:
            continue
        if not isinstance(value, (int, float)) or value < 0:
            raise ValueError(f"Invalid threshold value: thresholds.{key}")
        return float(value)
    return float(default)


def resolve_thresholds(args, config):
    config_thresholds = config.get("thresholds", {})
    thresholds = {
        "altitude_rmse": threshold_value(
            config_thresholds,
            ("altitude_rmse", "max_altitude_error_m"),
            DEFAULT_THRESHOLDS["altitude_rmse"],
        ),
        "velocity_rmse": threshold_value(
            config_thresholds,
            ("velocity_rmse",),
            DEFAULT_THRESHOLDS["velocity_rmse"],
        ),
        "attitude_rmse": threshold_value(
            config_thresholds,
            ("attitude_rmse",),
            DEFAULT_THRESHOLDS["attitude_rmse"],
        ),
    }

    if args.altitude_threshold is not None:
        thresholds["altitude_rmse"] = args.altitude_threshold
    if args.velocity_threshold is not None:
        thresholds["velocity_rmse"] = args.velocity_threshold
    if args.attitude_threshold is not None:
        thresholds["attitude_rmse"] = args.attitude_threshold

    for key, value in thresholds.items():
        if value < 0:
            raise ValueError(f"Invalid threshold value: {key}")

    return thresholds


def load_log(log_path):
    if not log_path.exists():
        raise FileNotFoundError(f"Log not found: {log_path}")
    return ULog(str(log_path))


def relative_time(timestamp_us):
    time_s = timestamp_us * 1e-6
    return time_s - time_s[0]


def get_altitude(ulog):
    data = ulog.get_dataset("vehicle_local_position")
    time_s = relative_time(data.data["timestamp"])
    altitude_m = -data.data["z"]
    return time_s, altitude_m


def get_velocity(ulog):
    data = ulog.get_dataset("vehicle_local_position")
    time_s = relative_time(data.data["timestamp"])
    vx = data.data["vx"]
    vy = data.data["vy"]
    vz = data.data["vz"]
    speed = np.sqrt(vx**2 + vy**2 + vz**2)
    return time_s, speed


def get_attitude(ulog):
    data = ulog.get_dataset("vehicle_attitude")
    time_s = relative_time(data.data["timestamp"])

    # pyulog stores quaternions as [w, x, y, z]; scipy expects [x, y, z, w].
    quaternion_xyzw = np.vstack(
        (
            data.data["q[1]"],
            data.data["q[2]"],
            data.data["q[3]"],
            data.data["q[0]"],
        )
    ).T

    euler = R.from_quat(quaternion_xyzw).as_euler("xyz", degrees=True)
    roll = euler[:, 0]
    pitch = euler[:, 1]
    yaw = euler[:, 2]
    return time_s, roll, pitch, yaw


def compute_rmse(left, right):
    if left.size == 0:
        raise ValueError("Cannot compute RMSE for empty aligned arrays")
    return float(np.sqrt(np.mean((left - right) ** 2)))


def angular_difference_deg(left, right):
    return (left - right + 180.0) % 360.0 - 180.0


def compute_circular_rmse_deg(left, right):
    if left.size == 0:
        raise ValueError("Cannot compute RMSE for empty aligned arrays")
    difference = angular_difference_deg(left, right)
    return float(np.sqrt(np.mean(difference**2)))


def circular_mean_deg(values):
    if values.size == 0:
        raise ValueError("Cannot compute circular mean for empty array")
    radians = np.deg2rad(values)
    return float(
        np.rad2deg(
            np.arctan2(
                np.mean(np.sin(radians)),
                np.mean(np.cos(radians)),
            )
        )
    )


def compute_heading_normalized_yaw_metrics(sim_yaw, real_yaw):
    difference = angular_difference_deg(sim_yaw, real_yaw)
    heading_offset = circular_mean_deg(difference)
    residual = angular_difference_deg(difference, heading_offset)
    return {
        "yaw_heading_offset_deg": heading_offset,
        "yaw_heading_normalized_rmse": float(np.sqrt(np.mean(residual**2))),
    }


def first_time_at_or_above(time_s, values, threshold):
    matching_indexes = np.flatnonzero(values >= threshold)
    if matching_indexes.size == 0:
        return None
    return float(time_s[int(matching_indexes[0])])


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


def default_takeoff_threshold(config):
    target_altitude = config.get("mission", {}).get("takeoff_altitude")
    if not isinstance(target_altitude, (int, float)) or target_altitude <= 0:
        return DEFAULT_TAKEOFF_THRESHOLD_M
    return float(
        min(
            max(DEFAULT_TAKEOFF_THRESHOLD_M, target_altitude * 0.05),
            target_altitude,
        )
    )


def resolve_alignment(args, config, sim_t, sim_altitude, real_t, real_altitude):
    if args.takeoff_threshold_m is not None and args.takeoff_threshold_m < 0:
        raise ValueError("--takeoff-threshold-m must be non-negative")

    threshold = (
        float(args.takeoff_threshold_m)
        if args.takeoff_threshold_m is not None
        else default_takeoff_threshold(config)
    )

    alignment = {
        "method": args.alignment,
        "takeoff_threshold_m": threshold if args.alignment == "takeoff" else None,
        "sim_time_offset_s": 0.0,
        "real_time_offset_s": 0.0,
    }

    if args.alignment == "time-zero":
        return alignment

    sim_offset = first_time_at_or_above(sim_t, sim_altitude, threshold)
    real_offset = first_time_at_or_above(real_t, real_altitude, threshold)
    if sim_offset is None:
        raise ValueError(
            f"Simulation log never reached takeoff alignment threshold {threshold} m"
        )
    if real_offset is None:
        raise ValueError(
            f"Real log never reached takeoff alignment threshold {threshold} m"
        )

    alignment["sim_time_offset_s"] = sim_offset
    alignment["real_time_offset_s"] = real_offset
    return alignment


def shifted_time(time_s, alignment, side):
    return time_s - alignment[f"{side}_time_offset_s"]


def align_and_compare(sim_t, sim_values, real_t, real_values, rmse_func=compute_rmse):
    common_t_start = max(sim_t[0], real_t[0])
    common_t_end = min(sim_t[-1], real_t[-1])
    if common_t_end <= common_t_start:
        raise ValueError("Simulation and real logs do not have overlapping time ranges")

    mask = (sim_t >= common_t_start) & (sim_t <= common_t_end)
    sim_t_aligned = sim_t[mask]
    sim_values_aligned = sim_values[mask]

    real_interp = interp1d(real_t, real_values, bounds_error=True)
    real_values_aligned = real_interp(sim_t_aligned)

    rmse = rmse_func(sim_values_aligned, real_values_aligned)
    details = {
        "common_time_start_s": float(common_t_start),
        "common_time_end_s": float(common_t_end),
        "sample_count": int(sim_t_aligned.size),
    }
    return sim_t_aligned, sim_values_aligned, real_values_aligned, rmse, details


def finite_float(value):
    if value is None:
        return None
    if not np.isfinite(value):
        return None
    return float(value)


def segment_metadata(start_time, end_time):
    if start_time is None or end_time is None or end_time <= start_time:
        return None
    return {
        "start_s": float(start_time),
        "end_s": float(end_time),
        "duration_s": float(end_time - start_time),
    }


def detect_segments(time_s, altitude_m, config, alignment, side):
    mission = config.get("mission", {})
    target_altitude = mission.get("takeoff_altitude")
    altitude_tolerance = mission.get("altitude_tolerance", 1.0)
    hover_time = mission.get("hover_time", 0.0)
    landing_altitude_threshold = config.get("thresholds", {}).get(
        "max_landing_final_altitude_m",
        DEFAULT_TAKEOFF_THRESHOLD_M,
    )

    if not isinstance(target_altitude, (int, float)) or target_altitude <= 0:
        return {}
    if not isinstance(altitude_tolerance, (int, float)) or altitude_tolerance < 0:
        altitude_tolerance = 1.0
    if not isinstance(hover_time, (int, float)) or hover_time < 0:
        hover_time = 0.0
    if (
        not isinstance(landing_altitude_threshold, (int, float))
        or landing_altitude_threshold < 0
    ):
        landing_altitude_threshold = DEFAULT_TAKEOFF_THRESHOLD_M

    shifted_t = shifted_time(time_s, alignment, side)
    takeoff_threshold = (
        alignment["takeoff_threshold_m"]
        if alignment["method"] == "takeoff"
        else default_takeoff_threshold(config)
    )
    target_floor = target_altitude - altitude_tolerance

    takeoff_start_index = first_index_at_or_above(altitude_m, takeoff_threshold)
    target_reached_index = first_index_at_or_above(
        altitude_m,
        target_floor,
        takeoff_start_index or 0,
    )

    takeoff_start_time = (
        finite_float(shifted_t[takeoff_start_index])
        if takeoff_start_index is not None
        else None
    )
    target_reached_time = (
        finite_float(shifted_t[target_reached_index])
        if target_reached_index is not None
        else None
    )

    landing_start_time = None
    landing_start_index = None
    landing_end_time = None
    if target_reached_index is not None and target_reached_time is not None:
        earliest_landing_time = target_reached_time + float(hover_time)
        earliest_landing_index = int(
            np.searchsorted(shifted_t, earliest_landing_time, side="left")
        )
        if earliest_landing_index < len(shifted_t):
            landing_start_index = first_index_at_or_below(
                altitude_m,
                target_floor,
                earliest_landing_index,
            )

    if landing_start_index is not None:
        landing_start_time = finite_float(shifted_t[landing_start_index])
        landing_end_index = first_index_at_or_below(
            altitude_m,
            landing_altitude_threshold,
            landing_start_index,
        )
        if landing_end_index is not None:
            landing_end_time = finite_float(shifted_t[landing_end_index])

    cruise_end_time = landing_start_time
    if cruise_end_time is None and landing_end_time is not None:
        cruise_end_time = landing_end_time
    if cruise_end_time is None and target_reached_time is not None:
        cruise_end_time = finite_float(shifted_t[-1])

    return {
        "takeoff_climb": segment_metadata(takeoff_start_time, target_reached_time),
        "hover_cruise": segment_metadata(target_reached_time, cruise_end_time),
        "landing": segment_metadata(landing_start_time, landing_end_time),
    }


def segment_values(time_s, values, segment):
    if segment is None:
        return None, None

    mask = (time_s >= segment["start_s"]) & (time_s <= segment["end_s"])
    segment_t = time_s[mask]
    segment_data = values[mask]
    if segment_t.size < MIN_SEGMENT_SAMPLES:
        return None, None

    duration = segment["duration_s"]
    if duration <= 0:
        return None, None

    normalized_t = (segment_t - segment["start_s"]) / duration
    return normalized_t, segment_data


def compare_segment_series(
    sim_t,
    sim_values,
    sim_segment,
    real_t,
    real_values,
    real_segment,
    rmse_func=compute_rmse,
):
    sim_segment_t, sim_segment_values = segment_values(sim_t, sim_values, sim_segment)
    real_segment_t, real_segment_values = segment_values(
        real_t,
        real_values,
        real_segment,
    )
    if sim_segment_t is None or real_segment_t is None:
        return None, {
            "available": False,
            "sample_count": 0,
        }

    common_t_start = max(sim_segment_t[0], real_segment_t[0])
    common_t_end = min(sim_segment_t[-1], real_segment_t[-1])
    if common_t_end <= common_t_start:
        return None, {
            "available": False,
            "sample_count": 0,
        }

    mask = (sim_segment_t >= common_t_start) & (sim_segment_t <= common_t_end)
    sim_aligned_t = sim_segment_t[mask]
    sim_aligned_values = sim_segment_values[mask]
    if sim_aligned_t.size < MIN_SEGMENT_SAMPLES:
        return None, {
            "available": False,
            "sample_count": int(sim_aligned_t.size),
        }

    real_interp = interp1d(real_segment_t, real_segment_values, bounds_error=True)
    real_aligned_values = real_interp(sim_aligned_t)

    return rmse_func(sim_aligned_values, real_aligned_values), {
        "available": True,
        "normalized_time_start": float(common_t_start),
        "normalized_time_end": float(common_t_end),
        "sample_count": int(sim_aligned_t.size),
    }


def compare_segment_heading_normalized_yaw(
    sim_t,
    sim_yaw,
    sim_segment,
    real_t,
    real_yaw,
    real_segment,
):
    sim_segment_t, sim_segment_yaw = segment_values(sim_t, sim_yaw, sim_segment)
    real_segment_t, real_segment_yaw = segment_values(real_t, real_yaw, real_segment)
    if sim_segment_t is None or real_segment_t is None:
        return {
            "yaw_heading_offset_deg": None,
            "yaw_heading_normalized_rmse": None,
        }

    common_t_start = max(sim_segment_t[0], real_segment_t[0])
    common_t_end = min(sim_segment_t[-1], real_segment_t[-1])
    if common_t_end <= common_t_start:
        return {
            "yaw_heading_offset_deg": None,
            "yaw_heading_normalized_rmse": None,
        }

    mask = (sim_segment_t >= common_t_start) & (sim_segment_t <= common_t_end)
    sim_aligned_t = sim_segment_t[mask]
    sim_aligned_yaw = sim_segment_yaw[mask]
    if sim_aligned_t.size < MIN_SEGMENT_SAMPLES:
        return {
            "yaw_heading_offset_deg": None,
            "yaw_heading_normalized_rmse": None,
        }

    real_interp = interp1d(real_segment_t, real_segment_yaw, bounds_error=True)
    real_aligned_yaw = real_interp(sim_aligned_t)
    return compute_heading_normalized_yaw_metrics(sim_aligned_yaw, real_aligned_yaw)


def compute_segment_rmse(series, segments):
    segment_metrics = {}
    for segment_name in ("takeoff_climb", "hover_cruise", "landing"):
        sim_segment = segments["sim"].get(segment_name)
        real_segment = segments["real"].get(segment_name)
        segment_metrics[segment_name] = {
            "sim": sim_segment,
            "real": real_segment,
            "comparison_axis": "normalized_segment_time",
        }

        for series_name, (sim_t, sim_values, real_t, real_values) in series.items():
            rmse_func = (
                compute_circular_rmse_deg
                if series_name == "yaw"
                else compute_rmse
            )
            rmse, details = compare_segment_series(
                sim_t,
                sim_values,
                sim_segment,
                real_t,
                real_values,
                real_segment,
                rmse_func,
            )
            segment_metrics[segment_name][f"{series_name}_rmse"] = rmse
            segment_metrics[segment_name][series_name] = details

        if "yaw" in series:
            sim_t, sim_yaw, real_t, real_yaw = series["yaw"]
            segment_metrics[segment_name].update(
                compare_segment_heading_normalized_yaw(
                    sim_t,
                    sim_yaw,
                    sim_segment,
                    real_t,
                    real_yaw,
                    real_segment,
                )
            )

    return segment_metrics


def evaluate(metrics, thresholds):
    altitude_pass = metrics["altitude_rmse"] <= thresholds["altitude_rmse"]
    velocity_pass = metrics["velocity_rmse"] <= thresholds["velocity_rmse"]
    attitude_pass = all(
        metrics[key] <= thresholds["attitude_rmse"]
        for key in ("roll_rmse", "pitch_rmse", "yaw_rmse")
    )

    return {
        "thresholds": thresholds,
        "altitude_pass": altitude_pass,
        "velocity_pass": velocity_pass,
        "attitude_pass": attitude_pass,
        "overall_pass": all([altitude_pass, velocity_pass, attitude_pass]),
    }


def plot_comparison(time_s, sim_data, real_data, name, plot_dir):
    plot_dir.mkdir(parents=True, exist_ok=True)
    plot_path = plot_dir / f"{name}.png"

    plt.figure()
    plt.plot(time_s, sim_data, label="Simulation")
    plt.plot(time_s, real_data, label="Real")
    plt.legend()
    plt.title(f"{name.capitalize()} Comparison")
    plt.xlabel("Time (s)")
    plt.ylabel(name)
    plt.savefig(plot_path)
    plt.close()
    return plot_path


def compare_series(sim_ulog, real_ulog, plot_dir, alignment, config):
    sim_t_altitude_raw, sim_altitude = get_altitude(sim_ulog)
    real_t_altitude_raw, real_altitude = get_altitude(real_ulog)
    sim_t_altitude_shifted = shifted_time(sim_t_altitude_raw, alignment, "sim")
    real_t_altitude_shifted = shifted_time(real_t_altitude_raw, alignment, "real")
    sim_t_alt, sim_alt, real_alt, altitude_rmse, altitude_details = align_and_compare(
        sim_t_altitude_shifted,
        sim_altitude,
        real_t_altitude_shifted,
        real_altitude,
    )
    altitude_plot = plot_comparison(sim_t_alt, sim_alt, real_alt, "altitude", plot_dir)

    sim_t, sim_speed = get_velocity(sim_ulog)
    real_t, real_speed = get_velocity(real_ulog)
    sim_t_velocity_shifted = shifted_time(sim_t, alignment, "sim")
    real_t_velocity_shifted = shifted_time(real_t, alignment, "real")
    (
        sim_t_speed,
        sim_speed_al,
        real_speed_al,
        velocity_rmse,
        velocity_details,
    ) = align_and_compare(
        sim_t_velocity_shifted,
        sim_speed,
        real_t_velocity_shifted,
        real_speed,
    )
    velocity_plot = plot_comparison(
        sim_t_speed,
        sim_speed_al,
        real_speed_al,
        "velocity",
        plot_dir,
    )

    sim_t, sim_roll, sim_pitch, sim_yaw = get_attitude(sim_ulog)
    real_t, real_roll, real_pitch, real_yaw = get_attitude(real_ulog)
    sim_t_attitude_shifted = shifted_time(sim_t, alignment, "sim")
    real_t_attitude_shifted = shifted_time(real_t, alignment, "real")

    (
        sim_t_roll,
        sim_roll_al,
        real_roll_al,
        roll_rmse,
        roll_details,
    ) = align_and_compare(
        sim_t_attitude_shifted,
        sim_roll,
        real_t_attitude_shifted,
        real_roll,
    )
    roll_plot = plot_comparison(
        sim_t_roll,
        sim_roll_al,
        real_roll_al,
        "roll",
        plot_dir,
    )

    (
        sim_t_pitch,
        sim_pitch_al,
        real_pitch_al,
        pitch_rmse,
        pitch_details,
    ) = align_and_compare(
        sim_t_attitude_shifted,
        sim_pitch,
        real_t_attitude_shifted,
        real_pitch,
    )
    pitch_plot = plot_comparison(
        sim_t_pitch,
        sim_pitch_al,
        real_pitch_al,
        "pitch",
        plot_dir,
    )

    (
        sim_t_yaw,
        sim_yaw_al,
        real_yaw_al,
        yaw_rmse,
        yaw_details,
    ) = align_and_compare(
        sim_t_attitude_shifted,
        sim_yaw,
        real_t_attitude_shifted,
        real_yaw,
        compute_circular_rmse_deg,
    )
    yaw_plot = plot_comparison(sim_t_yaw, sim_yaw_al, real_yaw_al, "yaw", plot_dir)
    heading_normalized_yaw = compute_heading_normalized_yaw_metrics(
        sim_yaw_al,
        real_yaw_al,
    )

    segments = {
        "sim": detect_segments(
            sim_t_altitude_raw,
            sim_altitude,
            config,
            alignment,
            "sim",
        ),
        "real": detect_segments(
            real_t_altitude_raw,
            real_altitude,
            config,
            alignment,
            "real",
        ),
    }
    segment_metrics = compute_segment_rmse(
        {
            "altitude": (
                sim_t_altitude_shifted,
                sim_altitude,
                real_t_altitude_shifted,
                real_altitude,
            ),
            "velocity": (
                sim_t_velocity_shifted,
                sim_speed,
                real_t_velocity_shifted,
                real_speed,
            ),
            "roll": (
                sim_t_attitude_shifted,
                sim_roll,
                real_t_attitude_shifted,
                real_roll,
            ),
            "pitch": (
                sim_t_attitude_shifted,
                sim_pitch,
                real_t_attitude_shifted,
                real_pitch,
            ),
            "yaw": (
                sim_t_attitude_shifted,
                sim_yaw,
                real_t_attitude_shifted,
                real_yaw,
            ),
        },
        segments,
    )

    return {
        "altitude_rmse": altitude_rmse,
        "velocity_rmse": velocity_rmse,
        "roll_rmse": roll_rmse,
        "pitch_rmse": pitch_rmse,
        "yaw_rmse": yaw_rmse,
        **heading_normalized_yaw,
        "segments": segment_metrics,
    }, {
        "altitude": str(altitude_plot),
        "velocity": str(velocity_plot),
        "roll": str(roll_plot),
        "pitch": str(pitch_plot),
        "yaw": str(yaw_plot),
    }, {
        "altitude": altitude_details,
        "velocity": velocity_details,
        "roll": roll_details,
        "pitch": pitch_details,
        "yaw": yaw_details,
    }


def main():
    args = parse_args()
    config = load_config(args.config)
    sim_ulog = load_log(args.sim)
    real_ulog = load_log(args.real)

    sim_t, sim_altitude = get_altitude(sim_ulog)
    real_t, real_altitude = get_altitude(real_ulog)
    alignment = resolve_alignment(
        args,
        config,
        sim_t,
        sim_altitude,
        real_t,
        real_altitude,
    )

    metrics, plots, alignment_series = compare_series(
        sim_ulog,
        real_ulog,
        args.plot_dir,
        alignment,
        config,
    )
    thresholds = resolve_thresholds(args, config)
    metrics["evaluation"] = evaluate(metrics, thresholds)
    metrics["alignment"] = alignment
    metrics["alignment"]["series"] = alignment_series
    metrics["inputs"] = {
        "sim": str(args.sim),
        "real": str(args.real),
        "config": str(args.config) if args.config else None,
    }
    metrics["plots"] = plots

    args.metrics.parent.mkdir(parents=True, exist_ok=True)
    with args.metrics.open("w", encoding="utf-8") as metrics_file:
        json.dump(metrics, metrics_file, indent=4)

    print(f"Altitude RMSE: {metrics['altitude_rmse']}")
    print(f"Velocity RMSE: {metrics['velocity_rmse']}")
    print(f"Roll RMSE: {metrics['roll_rmse']}")
    print(f"Pitch RMSE: {metrics['pitch_rmse']}")
    print(f"Yaw RMSE: {metrics['yaw_rmse']}")
    print(f"Yaw heading offset: {metrics['yaw_heading_offset_deg']}")
    print(f"Yaw heading-normalized RMSE: {metrics['yaw_heading_normalized_rmse']}")
    for segment_name, segment in metrics["segments"].items():
        print(f"{segment_name} altitude RMSE: {segment['altitude_rmse']}")
    print(f"Overall pass: {metrics['evaluation']['overall_pass']}")
    print(f"Saved metrics: {args.metrics}")
    print(f"Saved plots: {args.plot_dir}")


if __name__ == "__main__":
    main()
