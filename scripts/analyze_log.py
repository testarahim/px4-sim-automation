import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml
from pyulog import ULog


PROJECT_DIR = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_DIR / "config" / "sim_config.yaml"
LOG_PATH = PROJECT_DIR / "logs" / "latest_log.ulg"
RESULTS_DIR = PROJECT_DIR / "results"
PLOTS_DIR = RESULTS_DIR / "plots"
METRICS_PATH = RESULTS_DIR / "metrics.json"


def load_config():
    with CONFIG_PATH.open("r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def load_altitude(ulog):
    data = ulog.get_dataset("vehicle_local_position")
    time_s = data.data["timestamp"] * 1e-6
    time_s = time_s - time_s[0]
    altitude_m = -data.data["z"]
    return time_s, altitude_m


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
    if value is None:
        return None
    if not np.isfinite(value):
        return None
    return float(value)


def compute_rmse(values, target):
    if values.size == 0:
        return None
    return finite_or_none(np.sqrt(np.mean((values - target) ** 2)))


def compute_altitude_metrics(
    time_s,
    altitude_m,
    target_altitude,
    hover_time,
    altitude_tolerance,
    landing_altitude_threshold,
):
    max_altitude = float(np.max(altitude_m))
    final_altitude = float(altitude_m[-1])
    altitude_error = float(abs(max_altitude - target_altitude))
    duration = float(time_s[-1] - time_s[0])
    max_overshoot = float(max(0.0, max_altitude - target_altitude))

    takeoff_start_altitude = min(max(0.3, target_altitude * 0.05), target_altitude)
    takeoff_start_index = first_index_at_or_above(altitude_m, takeoff_start_altitude)
    target_reached_index = first_index_at_or_above(
        altitude_m,
        target_altitude - altitude_tolerance,
        takeoff_start_index or 0,
    )

    takeoff_duration = None
    hover_altitude_mean = None
    hover_altitude_rmse = None
    hover_start_time = None
    hover_end_time = None

    if takeoff_start_index is not None and target_reached_index is not None:
        takeoff_duration = time_s[target_reached_index] - time_s[takeoff_start_index]
        hover_start_time = time_s[target_reached_index]
        hover_end_time = hover_start_time + hover_time
        hover_mask = (time_s >= hover_start_time) & (time_s <= hover_end_time)
        hover_altitudes = altitude_m[hover_mask]
        if hover_altitudes.size > 0:
            hover_altitude_mean = float(np.mean(hover_altitudes))
            hover_altitude_rmse = compute_rmse(hover_altitudes, target_altitude)

    landing_duration = None
    if hover_end_time is not None:
        landing_start_index = int(np.searchsorted(time_s, hover_end_time, side="left"))
        landing_end_index = first_index_at_or_below(
            altitude_m,
            landing_altitude_threshold,
            landing_start_index,
        )
        if landing_end_index is not None and landing_start_index < len(time_s):
            landing_duration = time_s[landing_end_index] - time_s[landing_start_index]

    return {
        "target_altitude_m": float(target_altitude),
        "max_altitude_m": max_altitude,
        "final_altitude_m": final_altitude,
        "landing_final_altitude_abs_m": float(abs(final_altitude)),
        "max_altitude_error_m": altitude_error,
        "max_overshoot_m": max_overshoot,
        "takeoff_duration_s": finite_or_none(takeoff_duration),
        "hover_altitude_mean_m": finite_or_none(hover_altitude_mean),
        "hover_altitude_rmse_m": finite_or_none(hover_altitude_rmse),
        "landing_duration_s": finite_or_none(landing_duration),
        "duration_s": duration,
    }


def plot_altitude(time_s, altitude_m, target_altitude):
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

    plt.figure()
    plt.plot(time_s, altitude_m, label="Altitude")
    plt.axhline(target_altitude, color="tab:orange", linestyle="--", label="Target")
    plt.title("Altitude over time")
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.legend()
    plt.savefig(PLOTS_DIR / "altitude.png")
    plt.close()


def threshold_value(thresholds, key, default):
    value = thresholds.get(key, default)
    if not isinstance(value, (int, float)) or value < 0:
        raise ValueError(f"Invalid threshold value: thresholds.{key}")
    return value


def metric_pass(value, threshold):
    return value is not None and value <= threshold


def evaluate(metrics, config):
    thresholds = config.get("thresholds", {})
    legacy_altitude_threshold = thresholds.get("altitude_rmse", 1.5)
    max_altitude_error_threshold = threshold_value(
        thresholds,
        "max_altitude_error_m",
        legacy_altitude_threshold,
    )
    hover_rmse_threshold = threshold_value(
        thresholds,
        "hover_altitude_rmse_m",
        legacy_altitude_threshold,
    )
    landing_altitude_threshold = threshold_value(
        thresholds,
        "max_landing_final_altitude_m",
        0.3,
    )

    max_altitude_error_pass = metric_pass(
        metrics["max_altitude_error_m"],
        max_altitude_error_threshold,
    )
    hover_altitude_rmse_pass = metric_pass(
        metrics["hover_altitude_rmse_m"],
        hover_rmse_threshold,
    )
    landing_final_altitude_pass = metric_pass(
        metrics["landing_final_altitude_abs_m"],
        landing_altitude_threshold,
    )

    return {
        "thresholds": {
            "max_altitude_error_m": max_altitude_error_threshold,
            "hover_altitude_rmse_m": hover_rmse_threshold,
            "max_landing_final_altitude_m": landing_altitude_threshold,
        },
        "max_altitude_error_pass": max_altitude_error_pass,
        "hover_altitude_rmse_pass": hover_altitude_rmse_pass,
        "landing_final_altitude_pass": landing_final_altitude_pass,
        "altitude_pass": max_altitude_error_pass,
        "overall_pass": all(
            [
                max_altitude_error_pass,
                hover_altitude_rmse_pass,
                landing_final_altitude_pass,
            ]
        ),
    }


def main():
    config = load_config()
    target_altitude = config["mission"]["takeoff_altitude"]
    hover_time = config["mission"]["hover_time"]
    altitude_tolerance = config["mission"]["altitude_tolerance"]
    landing_altitude_threshold = config.get("thresholds", {}).get(
        "max_landing_final_altitude_m",
        0.3,
    )

    ulog = ULog(str(LOG_PATH))
    time_s, altitude_m = load_altitude(ulog)

    metrics = compute_altitude_metrics(
        time_s,
        altitude_m,
        target_altitude,
        hover_time,
        altitude_tolerance,
        landing_altitude_threshold,
    )
    metrics["evaluation"] = evaluate(metrics, config)

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    plot_altitude(time_s, altitude_m, target_altitude)

    with METRICS_PATH.open("w", encoding="utf-8") as metrics_file:
        json.dump(metrics, metrics_file, indent=4)

    print(f"Analyzed log: {LOG_PATH}")
    print(f"Saved plot: {PLOTS_DIR / 'altitude.png'}")
    print(f"Saved metrics: {METRICS_PATH}")
    print(f"Overall pass: {metrics['evaluation']['overall_pass']}")


if __name__ == "__main__":
    main()
