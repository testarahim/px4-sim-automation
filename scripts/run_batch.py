import argparse
import csv
import json
import os
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SCENARIO_DIR = PROJECT_DIR / "scenarios"
DEFAULT_RESULTS_DIR = PROJECT_DIR / "results"
RUN_SCRIPT = PROJECT_DIR / "run.sh"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run multiple PX4 SITL scenarios sequentially."
    )
    parser.add_argument(
        "scenarios",
        nargs="*",
        type=Path,
        help="Scenario YAML files. Defaults to all *.yaml files in scenarios/.",
    )
    parser.add_argument(
        "--scenario-dir",
        type=Path,
        default=DEFAULT_SCENARIO_DIR,
        help=f"Scenario directory used when no files are provided. Default: {DEFAULT_SCENARIO_DIR}",
    )
    parser.add_argument(
        "--batch-id",
        default=None,
        help="Optional batch id. Default: current timestamp.",
    )
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=DEFAULT_RESULTS_DIR,
        help=f"Results directory. Default: {DEFAULT_RESULTS_DIR}",
    )
    parser.add_argument(
        "--stop-on-fail",
        action="store_true",
        help="Stop after the first failed scenario.",
    )
    return parser.parse_args()


def slugify(value):
    slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    slug = slug.strip("._-")
    return slug or "scenario"


def batch_id_now():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def resolve_scenarios(args):
    scenario_dir = (
        args.scenario_dir
        if args.scenario_dir.is_absolute()
        else PROJECT_DIR / args.scenario_dir
    )

    if args.scenarios:
        scenarios = args.scenarios
    else:
        scenarios = sorted(scenario_dir.glob("*.yaml"))

    resolved = []
    for scenario in scenarios:
        path = scenario if scenario.is_absolute() else PROJECT_DIR / scenario
        if not path.exists():
            raise FileNotFoundError(f"Scenario not found: {scenario}")
        resolved.append(path.resolve())

    if not resolved:
        raise ValueError(f"No scenario YAML files found in {scenario_dir}")

    return resolved


def load_metrics(metrics_path):
    if not metrics_path.exists():
        return None

    with metrics_path.open("r", encoding="utf-8") as metrics_file:
        return json.load(metrics_file)


def metric_value(metrics, key):
    if not metrics:
        return None
    return metrics.get(key)


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output_file:
        json.dump(data, output_file, indent=4)


def write_csv(path, rows):
    fieldnames = [
        "scenario",
        "run_id",
        "status",
        "returncode",
        "overall_pass",
        "target_altitude_m",
        "max_altitude_error_m",
        "hover_altitude_rmse_m",
        "landing_final_altitude_abs_m",
        "run_dir",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fieldnames})


def format_cell(value):
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def print_summary(rows, summary_json, summary_csv):
    columns = [
        ("scenario", "Scenario"),
        ("status", "Status"),
        ("overall_pass", "Pass"),
        ("max_altitude_error_m", "Alt Err"),
        ("hover_altitude_rmse_m", "Hover RMSE"),
        ("landing_final_altitude_abs_m", "Land Abs"),
    ]
    widths = {
        key: max(len(title), *(len(format_cell(row.get(key))) for row in rows))
        for key, title in columns
    }

    header = "  ".join(title.ljust(widths[key]) for key, title in columns)
    separator = "  ".join("-" * widths[key] for key, _ in columns)
    print(header)
    print(separator)
    for row in rows:
        print(
            "  ".join(
                format_cell(row.get(key)).ljust(widths[key])
                for key, _ in columns
            )
        )

    print(f"Saved batch summary: {summary_json}")
    print(f"Saved batch CSV: {summary_csv}")


def run_scenario(index, scenario, batch_id, batch_dir, results_dir):
    scenario_name = scenario.stem
    run_id = f"{batch_id}_{index:02d}_{slugify(scenario_name)}"
    run_dir = results_dir / "runs" / run_id
    runner_log = batch_dir / f"{run_id}.runner.log"

    env = os.environ.copy()
    env["RUN_ID"] = run_id

    command = ["bash", str(RUN_SCRIPT), str(scenario)]
    print(f"[batch] Running {scenario.name} -> {run_id}", flush=True)

    with runner_log.open("w", encoding="utf-8") as log_file:
        process = subprocess.run(
            command,
            cwd=PROJECT_DIR,
            env=env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )

    metrics_path = run_dir / "metrics.json"
    metrics = load_metrics(metrics_path)
    overall_pass = None
    if metrics:
        overall_pass = metrics.get("evaluation", {}).get("overall_pass")

    status = "passed" if process.returncode == 0 and overall_pass is True else "failed"

    return {
        "scenario": scenario.name,
        "scenario_path": str(scenario),
        "run_id": run_id,
        "run_dir": str(run_dir),
        "runner_log": str(runner_log),
        "metrics_path": str(metrics_path) if metrics_path.exists() else None,
        "status": status,
        "returncode": process.returncode,
        "overall_pass": overall_pass,
        "target_altitude_m": metric_value(metrics, "target_altitude_m"),
        "max_altitude_error_m": metric_value(metrics, "max_altitude_error_m"),
        "hover_altitude_rmse_m": metric_value(metrics, "hover_altitude_rmse_m"),
        "landing_final_altitude_abs_m": metric_value(
            metrics,
            "landing_final_altitude_abs_m",
        ),
    }


def main():
    args = parse_args()
    scenarios = resolve_scenarios(args)
    batch_id = args.batch_id or batch_id_now()
    results_dir = args.results_dir
    if not results_dir.is_absolute():
        results_dir = PROJECT_DIR / results_dir

    batch_dir = results_dir / "batches" / batch_id
    batch_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for index, scenario in enumerate(scenarios, start=1):
        row = run_scenario(index, scenario, batch_id, batch_dir, results_dir)
        rows.append(row)
        print(
            f"[batch] {row['scenario']}: {row['status']} "
            f"(returncode={row['returncode']}, overall_pass={row['overall_pass']})",
            flush=True,
        )
        if args.stop_on_fail and row["status"] != "passed":
            break

    summary = {
        "batch_id": batch_id,
        "scenario_count": len(rows),
        "passed": sum(1 for row in rows if row["status"] == "passed"),
        "failed": sum(1 for row in rows if row["status"] != "passed"),
        "results": rows,
    }
    summary["overall_pass"] = summary["failed"] == 0 and summary["scenario_count"] > 0

    summary_json = batch_dir / "summary.json"
    summary_csv = batch_dir / "summary.csv"
    write_json(summary_json, summary)
    write_csv(summary_csv, rows)
    print_summary(rows, summary_json, summary_csv)

    return 0 if summary["overall_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
