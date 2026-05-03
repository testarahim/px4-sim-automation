# Architecture

This project is a PX4 SITL regression pipeline. The core loop is:

1. Start PX4 SITL.
2. Run a MAVSDK mission.
3. Save PX4 logs and run artifacts.
4. Extract metrics from the `.ulg` log.
5. Produce a pass/fail result.
6. Optionally compare simulation logs with real flight logs.

## Single Scenario Flow

```text
scenario YAML
    |
    v
run.sh
    |
    +--> scripts/start_sitl.py
    |       |
    |       v
    |   PX4 SITL gz_x500
    |
    +--> scripts/wait_for_sitl_ready.py
    |
    +--> scripts/run_mission.py
    |       |
    |       v
    |   MAVSDK over udpin://0.0.0.0:14540
    |
    +--> scripts/extract_log.py
    |
    +--> scripts/analyze_log.py
    |
    +--> scripts/compare_logs.py (optional)
    |
    v
results/runs/<run_id>/
```

`run.sh` accepts an optional scenario/config file:

```bash
bash run.sh scenarios/takeoff_land_20m.yaml
```

If no scenario is provided, it uses `scenarios/takeoff_land_20m.yaml`.

## Scenario Files

Scenario YAML files live in `scenarios/`. They define:

- SITL readiness timeout and ready log patterns
- MAVSDK connection, preflight, and arm timeouts
- Mission parameters such as takeoff altitude and hover time
- Metric thresholds used for pass/fail evaluation

Current starter scenarios:

- `scenarios/short_hover.yaml`
- `scenarios/takeoff_land_10m.yaml`
- `scenarios/takeoff_land_20m.yaml`
- `scenarios/takeoff_land_31m.yaml`

The legacy default config remains at `config/sim_config.yaml`.

## Component Roles

- `run.sh`: Orchestrates one full scenario run and owns cleanup.
- `scripts/start_sitl.py`: Starts PX4 SITL with `gz_x500`, filters PX4 console output, and shuts PX4 down cleanly.
- `scripts/wait_for_sitl_ready.py`: Watches `logs/sitl.log` until PX4 reports the startup-ready pattern.
- `scripts/run_mission.py`: Uses MAVSDK to connect, wait for preflight readiness, arm, take off, hover, land, and wait for disarm.
- `scripts/extract_log.py`: Copies the newest PX4 `.ulg` log to both a latest alias and the current run artifact directory.
- `scripts/analyze_log.py`: Computes altitude metrics, plots altitude, and writes `metrics.json`.
- `scripts/run_batch.py`: Runs multiple scenarios sequentially and writes batch summaries.
- `scripts/compare_logs.py`: Compares one simulation log with one real log.
- `scripts/batch_compare.py`: Runs `compare_logs.py` over multiple simulation logs.
- `scripts/download_public_logs.py`: Downloads filtered public PX4 Flight Review logs into the local real-log pool.
- `scripts/inspect_log_candidates.py`: Pre-analyzes downloaded real logs for
  altitude-based candidate quality before comparison.

## Artifacts

Each full run creates:

```text
results/runs/<run_id>/
    px4.ulg
    metrics.json
    altitude.png
    sitl.log
    compare_metrics.json      (optional)
    compare_plots/            (optional)
```

Latest aliases are also maintained:

```text
logs/latest_log.ulg
logs/sitl.log
results/metrics.json
results/plots/altitude.png
```

Generated artifacts are ignored by git. Durable examples should be stored as
small explicit sample files, not as live run outputs.

## Batch Flow

```text
scripts/run_batch.py
    |
    +--> scenario A -> bash run.sh scenario A -> results/runs/<run_id_A>/
    +--> scenario B -> bash run.sh scenario B -> results/runs/<run_id_B>/
    +--> scenario C -> bash run.sh scenario C -> results/runs/<run_id_C>/
    |
    v
results/batches/<batch_id>/
    summary.json
    summary.csv
    *.runner.log
```

`summary.json` contains:

- batch id
- start time, finish time, and duration
- passed/failed scenario counts
- per-scenario metrics and artifact paths

`summary.csv` contains the same per-scenario result rows in spreadsheet-friendly
form.

## Metrics And Evaluation

`scripts/analyze_log.py` currently computes:

- `target_altitude_m`
- `max_altitude_m`
- `final_altitude_m`
- `landing_final_altitude_abs_m`
- `max_altitude_error_m`
- `max_overshoot_m`
- `takeoff_duration_s`
- `hover_altitude_mean_m`
- `hover_altitude_rmse_m`
- `landing_duration_s`
- `duration_s`

Pass/fail evaluation currently uses:

- `max_altitude_error_m`
- `hover_altitude_rmse_m`
- `max_landing_final_altitude_m`

The thresholds come from the active scenario/config YAML.

## Simulation Vs Real Comparison

`scripts/compare_logs.py` compares one simulation `.ulg` log against one real
flight `.ulg` log:

```bash
python3 scripts/compare_logs.py \
  --config scenarios/takeoff_land_20m.yaml \
  --sim path/to/sim.ulg \
  --real path/to/real.ulg
```

By default, comparison uses time-zero alignment. For logs whose missions start
at different offsets, `--alignment takeoff` shifts both logs to the first
takeoff-threshold altitude crossing before computing RMSE:

```bash
python3 scripts/compare_logs.py \
  --config scenarios/takeoff_land_20m.yaml \
  --alignment takeoff \
  --sim path/to/sim.ulg \
  --real path/to/real.ulg
```

It computes:

- `altitude_rmse`
- `velocity_rmse`
- `roll_rmse`
- `pitch_rmse`
- `yaw_rmse`

The metrics JSON also contains a `segments` object with `takeoff_climb`,
`hover_cruise`, and `landing` entries. Segment boundaries are detected from
each log's altitude profile using the scenario target altitude, tolerance,
hover time, and landing altitude threshold. Segment RMSE values are computed on
normalized segment time and are intentionally not folded into the global
`overall_pass`.

Thresholds can come from a scenario/config file. CLI threshold arguments still
override config values.

`run.sh` runs this comparison automatically only when both default files exist:

```text
data/sim/sim_log.ulg
data/real/real_log.ulg
```

`scripts/batch_compare.py` compares every `.ulg` file in `data/sim/` against a
single real log.

## Real Log Data Plan

The planned Flight Review integration is for building a real-flight log pool.
PX4 Flight Review provides a public log database and a downloader script:

```text
https://github.com/PX4/flight_review/blob/main/app/download_logs.py
```

That script downloads public Flight Review logs from the server APIs; it does
not download logs from a connected vehicle or SITL instance.

The intended project role is:

```text
PX4 Flight Review public logs
    |
    v
scripts/download_public_logs.py
    |
    v
data/real/public_logs/
    |
    v
scripts/compare_logs.py / scripts/batch_compare.py
```

The downloader uses conservative defaults, supports print-only inspection, and
keeps downloaded logs out of git. Keep focused filters and download delays to
respect public Flight Review hosting.

Downloaded real-log candidates can be inspected before comparison:

```bash
python3 scripts/inspect_log_candidates.py \
  --log-dir data/real/public_logs \
  --ground-altitude-m 0.3 \
  --takeoff-altitude-m 1.0
```

The inspector reports altitude-derived quality fields such as
`initial_altitude_m`, `max_altitude_m`, `final_altitude_m`, `airborne_start`,
`takeoff_detected`, and `landing_detected`.

## Operational Notes

- PX4 SITL target: `gz_x500`
- Default PX4 directory: `~/PX4-Autopilot`
- Override PX4 directory with `PX4_DIR=/path/to/PX4-Autopilot`.
- MAVSDK connection string: `udpin://0.0.0.0:14540`
- PX4 log source directory:
  `~/PX4-Autopilot/build/px4_sitl_default/rootfs/log`
- SITL readiness pattern: `Startup script returned successfully`
- `scripts/start_sitl.py` removes expected stale PX4 rootfs symlinks before
  start to avoid repeated-run startup failures.
