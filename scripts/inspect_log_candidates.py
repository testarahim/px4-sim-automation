import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_LOG_DIR = PROJECT_DIR / "data" / "real" / "public_logs"
DEFAULT_OUTPUT_JSON = PROJECT_DIR / "results" / "real_log_candidates.json"
DEFAULT_OUTPUT_CSV = PROJECT_DIR / "results" / "real_log_candidates.csv"
DEFAULT_GROUND_ALTITUDE_M = 0.3
DEFAULT_TAKEOFF_ALTITUDE_M = 1.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Inspect PX4 .ulg logs for real-log candidate quality."
    )
    parser.add_argument(
        "logs",
        nargs="*",
        type=Path,
        help="Specific .ulg files to inspect. Defaults to --log-dir/*.ulg.",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=DEFAULT_LOG_DIR,
        help=f"Directory scanned when no log files are provided. Default: {DEFAULT_LOG_DIR}",
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        default=DEFAULT_OUTPUT_JSON,
        help=f"JSON output path. Default: {DEFAULT_OUTPUT_JSON}",
    )
    parser.add_argument(
        "--output-csv",
        type=Path,
        default=DEFAULT_OUTPUT_CSV,
        help=f"CSV output path. Default: {DEFAULT_OUTPUT_CSV}",
    )
    parser.add_argument(
        "--ground-altitude-m",
        type=float,
        default=DEFAULT_GROUND_ALTITUDE_M,
        help="Altitude considered near ground for start/end quality checks.",
    )
    parser.add_argument(
        "--takeoff-altitude-m",
        type=float,
        default=DEFAULT_TAKEOFF_ALTITUDE_M,
        help="Altitude threshold used to detect a takeoff signature.",
    )
    return parser.parse_args()


def resolve_log_paths(args):
    if args.logs:
        paths = [
            path if path.is_absolute() else PROJECT_DIR / path
            for path in args.logs
        ]
    else:
        log_dir = (
            args.log_dir
            if args.log_dir.is_absolute()
            else PROJECT_DIR / args.log_dir
        )
        paths = sorted(log_dir.glob("*.ulg"))

    if not paths:
        raise FileNotFoundError("No .ulg logs found to inspect")
    return [path.resolve() for path in paths]


def load_altitude(log_path):
    ulog = ULog(str(log_path))
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
    if value is None or not np.isfinite(value):
        return None
    return float(value)


def compute_candidate_metrics(
    time_s,
    altitude_m,
    ground_altitude_m=DEFAULT_GROUND_ALTITUDE_M,
    takeoff_altitude_m=DEFAULT_TAKEOFF_ALTITUDE_M,
):
    if time_s.size == 0 or altitude_m.size == 0:
        raise ValueError("Altitude series is empty")

    initial_altitude = float(altitude_m[0])
    final_altitude = float(altitude_m[-1])
    max_altitude = float(np.max(altitude_m))
    min_altitude = float(np.min(altitude_m))
    duration_s = float(time_s[-1] - time_s[0])

    takeoff_index = first_index_at_or_above(altitude_m, takeoff_altitude_m)
    max_index = int(np.argmax(altitude_m))
    landing_index = first_index_at_or_below(
        altitude_m,
        ground_altitude_m,
        max_index,
    )

    starts_near_ground = abs(initial_altitude) <= ground_altitude_m
    ends_near_ground = abs(final_altitude) <= ground_altitude_m
    takeoff_detected = starts_near_ground and takeoff_index is not None
    landing_detected = (
        takeoff_index is not None
        and landing_index is not None
        and landing_index > takeoff_index
    )

    takeoff_time = time_s[takeoff_index] if takeoff_detected else None
    landing_time = time_s[landing_index] if landing_detected else None

    return {
        "duration_s": duration_s,
        "initial_altitude_m": initial_altitude,
        "max_altitude_m": max_altitude,
        "final_altitude_m": final_altitude,
        "min_altitude_m": min_altitude,
        "altitude_range_m": float(max_altitude - min_altitude),
        "starts_near_ground": bool(starts_near_ground),
        "ends_near_ground": bool(ends_near_ground),
        "airborne_start": bool(not starts_near_ground),
        "takeoff_detected": bool(takeoff_detected),
        "landing_detected": bool(landing_detected),
        "takeoff_time_s": finite_or_none(takeoff_time),
        "landing_time_s": finite_or_none(landing_time),
        "ground_altitude_m": float(ground_altitude_m),
        "takeoff_altitude_m": float(takeoff_altitude_m),
    }


def inspect_log(log_path, ground_altitude_m, takeoff_altitude_m):
    result = {
        "log": str(log_path),
        "log_id": log_path.stem,
        "ok": False,
        "error": None,
    }
    try:
        time_s, altitude_m = load_altitude(log_path)
        result.update(
            compute_candidate_metrics(
                time_s,
                altitude_m,
                ground_altitude_m=ground_altitude_m,
                takeoff_altitude_m=takeoff_altitude_m,
            )
        )
        result["ok"] = True
    except Exception as exc:
        result["error"] = str(exc)
    return result


def write_json(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output_file:
        json.dump(rows, output_file, indent=4)


def write_csv(path, rows):
    fieldnames = [
        "log_id",
        "ok",
        "airborne_start",
        "takeoff_detected",
        "landing_detected",
        "duration_s",
        "initial_altitude_m",
        "max_altitude_m",
        "final_altitude_m",
        "min_altitude_m",
        "altitude_range_m",
        "takeoff_time_s",
        "landing_time_s",
        "log",
        "error",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fieldnames})


def print_summary(rows, output_json, output_csv):
    ok_rows = [row for row in rows if row["ok"]]
    print(f"Inspected logs: {len(rows)}")
    print(f"Readable logs: {len(ok_rows)}")
    print(f"Airborne starts: {sum(1 for row in ok_rows if row['airborne_start'])}")
    print(f"Takeoff detected: {sum(1 for row in ok_rows if row['takeoff_detected'])}")
    print(f"Landing detected: {sum(1 for row in ok_rows if row['landing_detected'])}")
    print(f"Saved JSON: {output_json}")
    print(f"Saved CSV: {output_csv}")


def main():
    args = parse_args()
    if args.ground_altitude_m < 0:
        raise ValueError("--ground-altitude-m must be non-negative")
    if args.takeoff_altitude_m < 0:
        raise ValueError("--takeoff-altitude-m must be non-negative")

    log_paths = resolve_log_paths(args)
    rows = [
        inspect_log(
            log_path,
            ground_altitude_m=args.ground_altitude_m,
            takeoff_altitude_m=args.takeoff_altitude_m,
        )
        for log_path in log_paths
    ]

    write_json(args.output_json, rows)
    write_csv(args.output_csv, rows)
    print_summary(rows, args.output_json, args.output_csv)

    return 1 if any(not row["ok"] for row in rows) else 0


if __name__ == "__main__":
    sys.exit(main())
