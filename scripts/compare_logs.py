import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SIM_LOG = PROJECT_DIR / "data" / "sim" / "sim_log.ulg"
DEFAULT_REAL_LOG = PROJECT_DIR / "data" / "real" / "real_log.ulg"
DEFAULT_METRICS = PROJECT_DIR / "results" / "compare_metrics.json"
DEFAULT_PLOT_DIR = PROJECT_DIR / "results" / "plots" / "compare"


def parse_args():
    parser = argparse.ArgumentParser(description="Compare simulation and real PX4 logs.")
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
        default=1.5,
        help="Altitude RMSE pass threshold in meters.",
    )
    parser.add_argument(
        "--velocity-threshold",
        type=float,
        default=1.0,
        help="Velocity RMSE pass threshold in m/s.",
    )
    parser.add_argument(
        "--attitude-threshold",
        type=float,
        default=5.0,
        help="Roll/pitch/yaw RMSE pass threshold in degrees.",
    )
    return parser.parse_args()


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


def align_and_compare(sim_t, sim_values, real_t, real_values):
    common_t_start = max(sim_t[0], real_t[0])
    common_t_end = min(sim_t[-1], real_t[-1])
    if common_t_end <= common_t_start:
        raise ValueError("Simulation and real logs do not have overlapping time ranges")

    mask = (sim_t >= common_t_start) & (sim_t <= common_t_end)
    sim_t_aligned = sim_t[mask]
    sim_values_aligned = sim_values[mask]

    real_interp = interp1d(real_t, real_values, bounds_error=True)
    real_values_aligned = real_interp(sim_t_aligned)

    rmse = compute_rmse(sim_values_aligned, real_values_aligned)
    return sim_t_aligned, sim_values_aligned, real_values_aligned, rmse


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


def compare_series(sim_ulog, real_ulog, plot_dir):
    sim_t, sim_altitude = get_altitude(sim_ulog)
    real_t, real_altitude = get_altitude(real_ulog)
    sim_t_alt, sim_alt, real_alt, altitude_rmse = align_and_compare(
        sim_t,
        sim_altitude,
        real_t,
        real_altitude,
    )
    altitude_plot = plot_comparison(sim_t_alt, sim_alt, real_alt, "altitude", plot_dir)

    sim_t, sim_speed = get_velocity(sim_ulog)
    real_t, real_speed = get_velocity(real_ulog)
    sim_t_speed, sim_speed_al, real_speed_al, velocity_rmse = align_and_compare(
        sim_t,
        sim_speed,
        real_t,
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

    sim_t_roll, sim_roll_al, real_roll_al, roll_rmse = align_and_compare(
        sim_t,
        sim_roll,
        real_t,
        real_roll,
    )
    roll_plot = plot_comparison(
        sim_t_roll,
        sim_roll_al,
        real_roll_al,
        "roll",
        plot_dir,
    )

    sim_t_pitch, sim_pitch_al, real_pitch_al, pitch_rmse = align_and_compare(
        sim_t,
        sim_pitch,
        real_t,
        real_pitch,
    )
    pitch_plot = plot_comparison(
        sim_t_pitch,
        sim_pitch_al,
        real_pitch_al,
        "pitch",
        plot_dir,
    )

    sim_t_yaw, sim_yaw_al, real_yaw_al, yaw_rmse = align_and_compare(
        sim_t,
        sim_yaw,
        real_t,
        real_yaw,
    )
    yaw_plot = plot_comparison(sim_t_yaw, sim_yaw_al, real_yaw_al, "yaw", plot_dir)

    return {
        "altitude_rmse": altitude_rmse,
        "velocity_rmse": velocity_rmse,
        "roll_rmse": roll_rmse,
        "pitch_rmse": pitch_rmse,
        "yaw_rmse": yaw_rmse,
    }, {
        "altitude": str(altitude_plot),
        "velocity": str(velocity_plot),
        "roll": str(roll_plot),
        "pitch": str(pitch_plot),
        "yaw": str(yaw_plot),
    }


def main():
    args = parse_args()
    sim_ulog = load_log(args.sim)
    real_ulog = load_log(args.real)

    metrics, plots = compare_series(sim_ulog, real_ulog, args.plot_dir)
    thresholds = {
        "altitude_rmse": args.altitude_threshold,
        "velocity_rmse": args.velocity_threshold,
        "attitude_rmse": args.attitude_threshold,
    }
    metrics["evaluation"] = evaluate(metrics, thresholds)
    metrics["inputs"] = {
        "sim": str(args.sim),
        "real": str(args.real),
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
    print(f"Overall pass: {metrics['evaluation']['overall_pass']}")
    print(f"Saved metrics: {args.metrics}")
    print(f"Saved plots: {args.plot_dir}")


if __name__ == "__main__":
    main()
