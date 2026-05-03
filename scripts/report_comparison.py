import argparse
import csv
import io
import json
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_METRICS = PROJECT_DIR / "results" / "compare_metrics.json"
SERIES_NAMES = ("altitude", "velocity", "roll", "pitch", "yaw")
SEGMENT_NAMES = ("takeoff_climb", "hover_cruise", "landing")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate Markdown and CSV reports from comparison metrics."
    )
    parser.add_argument(
        "--metrics",
        type=Path,
        default=DEFAULT_METRICS,
        help=f"Comparison metrics JSON path. Default: {DEFAULT_METRICS}",
    )
    parser.add_argument(
        "--markdown",
        type=Path,
        default=None,
        help="Markdown report output path. Default: <metrics_stem>_report.md",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="CSV segment summary output path. Default: <metrics_stem>_segments.csv",
    )
    return parser.parse_args()


def default_markdown_path(metrics_path):
    return metrics_path.with_name(f"{metrics_path.stem}_report.md")


def default_csv_path(metrics_path):
    return metrics_path.with_name(f"{metrics_path.stem}_segments.csv")


def load_metrics(metrics_path):
    if not metrics_path.exists():
        raise FileNotFoundError(f"Metrics file not found: {metrics_path}")
    with metrics_path.open("r", encoding="utf-8") as metrics_file:
        return json.load(metrics_file)


def format_value(value, digits=3):
    if value is None:
        return "n/a"
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, (int, float)):
        return f"{float(value):.{digits}f}"
    return str(value)


def duration_ratio(real_duration_s, sim_duration_s):
    if real_duration_s is None or sim_duration_s in (None, 0):
        return None
    return real_duration_s / sim_duration_s


def segment_duration(segment, side):
    side_data = segment.get(side)
    if not side_data:
        return None
    return side_data.get("duration_s")


def build_segment_rows(metrics):
    rows = []
    segments = metrics.get("segments", {})

    for segment_name in SEGMENT_NAMES:
        segment = segments.get(segment_name, {})
        sim_duration_s = segment_duration(segment, "sim")
        real_duration_s = segment_duration(segment, "real")
        row = {
            "segment": segment_name,
            "sim_duration_s": sim_duration_s,
            "real_duration_s": real_duration_s,
            "real_to_sim_duration_ratio": duration_ratio(
                real_duration_s,
                sim_duration_s,
            ),
        }

        for series_name in SERIES_NAMES:
            row[f"{series_name}_rmse"] = segment.get(f"{series_name}_rmse")

        rows.append(row)

    return rows


def build_global_rows(metrics):
    return [
        ("altitude_rmse", metrics.get("altitude_rmse")),
        ("velocity_rmse", metrics.get("velocity_rmse")),
        ("roll_rmse", metrics.get("roll_rmse")),
        ("pitch_rmse", metrics.get("pitch_rmse")),
        ("yaw_rmse", metrics.get("yaw_rmse")),
    ]


def max_numeric_row(rows, key):
    numeric_rows = [
        row for row in rows if isinstance(row.get(key), (int, float))
    ]
    if not numeric_rows:
        return None
    return max(numeric_rows, key=lambda row: row[key])


def render_markdown(metrics):
    segment_rows = build_segment_rows(metrics)
    evaluation = metrics.get("evaluation", {})
    alignment = metrics.get("alignment", {})
    inputs = metrics.get("inputs", {})
    worst_altitude = max_numeric_row(segment_rows, "altitude_rmse")
    longest_ratio = max_numeric_row(segment_rows, "real_to_sim_duration_ratio")

    lines = [
        "# Comparison Report",
        "",
        "## Inputs",
        "",
        "| Input | Path |",
        "| --- | --- |",
        f"| Simulation | `{inputs.get('sim', 'n/a')}` |",
        f"| Real | `{inputs.get('real', 'n/a')}` |",
        f"| Config | `{inputs.get('config', 'n/a')}` |",
        "",
        "## Evaluation",
        "",
        "| Field | Value |",
        "| --- | --- |",
        f"| overall_pass | {format_value(evaluation.get('overall_pass'))} |",
        f"| altitude_pass | {format_value(evaluation.get('altitude_pass'))} |",
        f"| velocity_pass | {format_value(evaluation.get('velocity_pass'))} |",
        f"| attitude_pass | {format_value(evaluation.get('attitude_pass'))} |",
        "",
        "## Alignment",
        "",
        "| Field | Value |",
        "| --- | --- |",
        f"| method | {format_value(alignment.get('method'))} |",
        f"| takeoff_threshold_m | {format_value(alignment.get('takeoff_threshold_m'))} |",
        f"| sim_time_offset_s | {format_value(alignment.get('sim_time_offset_s'))} |",
        f"| real_time_offset_s | {format_value(alignment.get('real_time_offset_s'))} |",
        "",
        "## Global Metrics",
        "",
        "| Metric | RMSE |",
        "| --- | ---: |",
    ]

    for metric_name, value in build_global_rows(metrics):
        lines.append(f"| {metric_name} | {format_value(value)} |")

    lines.extend(
        [
            "",
            "## Segment Metrics",
            "",
            "| Segment | Sim duration s | Real duration s | Real/Sim duration | Alt RMSE | Vel RMSE | Roll RMSE | Pitch RMSE | Yaw RMSE |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )

    for row in segment_rows:
        lines.append(
            "| {segment} | {sim_duration} | {real_duration} | {duration_ratio} | "
            "{altitude} | {velocity} | {roll} | {pitch} | {yaw} |".format(
                segment=row["segment"],
                sim_duration=format_value(row["sim_duration_s"]),
                real_duration=format_value(row["real_duration_s"]),
                duration_ratio=format_value(row["real_to_sim_duration_ratio"]),
                altitude=format_value(row["altitude_rmse"]),
                velocity=format_value(row["velocity_rmse"]),
                roll=format_value(row["roll_rmse"]),
                pitch=format_value(row["pitch_rmse"]),
                yaw=format_value(row["yaw_rmse"]),
            )
        )

    lines.extend(["", "## Summary", ""])
    if worst_altitude:
        lines.append(
            "- Largest segment altitude RMSE: "
            f"{worst_altitude['segment']} ({format_value(worst_altitude['altitude_rmse'])})."
        )
    if longest_ratio:
        lines.append(
            "- Largest real/sim duration ratio: "
            f"{longest_ratio['segment']} "
            f"({format_value(longest_ratio['real_to_sim_duration_ratio'])}x)."
        )
    if not worst_altitude and not longest_ratio:
        lines.append("- Segment summary is unavailable in this metrics file.")

    return "\n".join(lines) + "\n"


def render_csv(metrics):
    rows = build_segment_rows(metrics)
    output = io.StringIO()
    fieldnames = [
        "segment",
        "sim_duration_s",
        "real_duration_s",
        "real_to_sim_duration_ratio",
        "altitude_rmse",
        "velocity_rmse",
        "roll_rmse",
        "pitch_rmse",
        "yaw_rmse",
    ]
    writer = csv.DictWriter(output, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return output.getvalue()


def write_text(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as output_file:
        output_file.write(text)


def main():
    args = parse_args()
    metrics = load_metrics(args.metrics)
    markdown_path = args.markdown or default_markdown_path(args.metrics)
    csv_path = args.csv or default_csv_path(args.metrics)

    write_text(markdown_path, render_markdown(metrics))
    write_text(csv_path, render_csv(metrics))

    print(f"Loaded metrics: {args.metrics}")
    print(f"Saved Markdown report: {markdown_path}")
    print(f"Saved CSV report: {csv_path}")


if __name__ == "__main__":
    main()
