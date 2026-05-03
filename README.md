# PX4 Simulation Automation Pipeline

This project provides a repeatable PX4 SITL automation pipeline. It starts PX4,
runs a MAVSDK mission, extracts the PX4 `.ulg` log, computes metrics, and
returns a pass/fail result.

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

The comparison computes:

- `altitude_rmse`
- `velocity_rmse`
- `roll_rmse`
- `pitch_rmse`
- `yaw_rmse`

Default comparison outputs:

- `results/compare_metrics.json`
- `results/plots/compare/`

Compare every `.ulg` file in `data/sim/` against `data/real/real_log.ulg`:

```bash
python3 scripts/batch_compare.py
```

## Configuration

Scenario YAML files define:

- SITL startup readiness timeout and log patterns
- MAVSDK connection/preflight/arm timeouts
- Mission takeoff altitude, hover time, and landing timeout
- Pass/fail thresholds

The legacy default config remains available at:

- `config/sim_config.yaml`
