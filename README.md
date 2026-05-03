# PX4 Simulation Automation Pipeline

This project provides a repeatable PX4 SITL automation pipeline. It starts PX4,
runs a MAVSDK mission, extracts the PX4 `.ulg` log, computes metrics, and
returns a pass/fail result.

## AI Assistance

This project was developed with support from artificial intelligence tools.
AI assistance was used for code generation, documentation, analysis, and
iterative engineering decisions, with the project owner reviewing and directing
the work.

## Features

- One-command PX4 SITL run
- Scenario-based mission configuration
- MAVSDK-based takeoff, hover, and landing workflow
- Automatic PX4 log extraction
- Per-run artifacts under `results/runs/<run_id>/`
- Altitude metrics and pass/fail evaluation
- Sequential scenario batch runner
- Optional simulation-vs-real log comparison

## Requirements

- Ubuntu 20.04+
- PX4 Autopilot, tested with PX4 v1.16.2
- Gazebo/Gz simulator support for `gz_x500`
- Python 3.8+

## Setup

```bash
bash setup.sh
```

If PX4 is not installed at `~/PX4-Autopilot`, set `PX4_DIR` when running the
pipeline:

```bash
PX4_DIR=/path/to/PX4-Autopilot bash run.sh
```

## Single Scenario Run

Run the default 20 m takeoff/land scenario:

```bash
bash run.sh
```

Run a specific scenario:

```bash
bash run.sh scenarios/takeoff_land_10m.yaml
```

Available starter scenarios:

- `scenarios/takeoff_land_10m.yaml`
- `scenarios/takeoff_land_20m.yaml`
- `scenarios/takeoff_land_31m.yaml`
- `scenarios/profiled_landing_31m.yaml`
- `scenarios/short_hover.yaml`

## Batch Scenario Run

Run every `*.yaml` file in `scenarios/` sequentially:

```bash
python3 scripts/run_batch.py
```

Run selected scenarios:

```bash
python3 scripts/run_batch.py scenarios/short_hover.yaml scenarios/takeoff_land_20m.yaml
```

Useful batch options:

```bash
python3 scripts/run_batch.py --stop-on-fail
python3 scripts/run_batch.py --batch-id manual_regression_001
```

Batch summaries are written to:

- `results/batches/<batch_id>/summary.json`
- `results/batches/<batch_id>/summary.csv`

Batch summaries include start time, finish time, duration, pass/fail status,
and per-scenario metrics.

## Outputs

The latest single-run aliases are:

- `results/metrics.json`
- `results/plots/altitude.png`
- `logs/latest_log.ulg`
- `logs/sitl.log`

Each full run also gets an artifact directory:

- `results/runs/<run_id>/px4.ulg`
- `results/runs/<run_id>/metrics.json`
- `results/runs/<run_id>/altitude.png`
- `results/runs/<run_id>/sitl.log`

Watch SITL output while a run is active:

```bash
tail -f logs/sitl.log
```

## Simulation vs Real Comparison

Compare one simulation log with one real flight log:

```bash
python3 scripts/compare_logs.py --sim path/to/sim.ulg --real path/to/real.ulg
```

Use a scenario/config file for comparison thresholds:

```bash
python3 scripts/compare_logs.py \
  --config scenarios/takeoff_land_20m.yaml \
  --sim path/to/sim.ulg \
  --real path/to/real.ulg
```

Align logs by first takeoff-threshold crossing before comparing:

```bash
python3 scripts/compare_logs.py \
  --config scenarios/takeoff_land_20m.yaml \
  --alignment takeoff \
  --sim path/to/sim.ulg \
  --real path/to/real.ulg
```

The comparison computes:

- `altitude_rmse`
- `velocity_rmse`
- `roll_rmse`
- `pitch_rmse`
- `yaw_rmse` using circular angle difference
- `yaw_heading_normalized_rmse` after removing mean heading offset

It also writes phase-based metrics under `segments` for `takeoff_climb`,
`hover_cruise`, and `landing`. Segment metrics are reported separately from
`overall_pass` and compare each phase on normalized segment time so logs with
different phase durations can still be inspected.
Each segment also includes profile summaries for simulation and real logs:
mean descent rate, mean horizontal speed, and mean absolute yaw-rate.

Generate a readable Markdown and CSV report from comparison metrics:

```bash
python3 scripts/report_comparison.py \
  --metrics results/comparisons/public_log_31m_takeoff_aligned_metrics.json
```

Default comparison outputs:

- `results/compare_metrics.json`
- `results/plots/compare/`

Compare every `.ulg` file in `data/sim/` against `data/real/real_log.ulg`:

```bash
python3 scripts/batch_compare.py
```

Pass a scenario/config file to batch comparison:

```bash
python3 scripts/batch_compare.py --config scenarios/takeoff_land_20m.yaml
```

Inspect public PX4 Flight Review logs before downloading:

```bash
python3 scripts/download_public_logs.py \
  --print \
  --mav-type Quadrotor \
  --flight-modes Mission \
  --max-num 10
```

Download a small real-log dataset:

```bash
python3 scripts/download_public_logs.py \
  --mav-type Quadrotor \
  --flight-modes Mission \
  --rating Good \
  --min-duration-s 30 \
  --max-duration-s 180 \
  --min-flight-mode-duration-s Mission 30 \
  --exclude-sys-hw PX4_SITL \
  --max-logged-warnings 5 \
  --max-logged-errors 0 \
  --max-num 10
```

Inspect downloaded real-log candidates before comparison:

```bash
python3 scripts/inspect_log_candidates.py \
  --log-dir data/real/public_logs \
  --ground-altitude-m 0.3 \
  --takeoff-altitude-m 1.0
```

## Configuration

Scenario YAML files define:

- SITL startup readiness timeout and log patterns
- MAVSDK connection/preflight/arm timeouts
- Mission takeoff altitude, hover time, and landing timeout
- Optional `mission.landing_profile` settings for Offboard NED/body-frame
  horizontal/descent/yaw-rate landing profiles
- Pass/fail thresholds

The legacy default config remains available at:

- `config/sim_config.yaml`
