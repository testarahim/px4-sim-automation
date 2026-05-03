# Public Flight Log Download Plan

This document describes how this project should use public PX4 Flight Review
logs as a real-flight data source for simulation-vs-real comparisons.

## Purpose

The goal is to build a small, controlled pool of real PX4 `.ulg` logs that can
be compared against SITL outputs.

This is not the same as downloading logs from a connected flight controller or
from SITL. The planned source is the public PX4 Flight Review database.

## Upstream Reference

Reference script:

```text
https://github.com/PX4/flight_review/blob/main/app/download_logs.py
```

The upstream script downloads public logs from Flight Review. It first fetches
database metadata, filters entries locally, and then downloads matching `.ulg`
files by log id.

Related public browsing page:

```text
https://review.px4.io/browse
```

Related PX4 documentation:

```text
https://docs.px4.io/main/en/dev_log/flight_log_analysis_statistical
```

## Intended Project Flow

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

Implementation status: `scripts/download_public_logs.py` provides the first
project-native wrapper for this flow.

The downloaded logs should be treated as input data, not as source files. They
must stay out of git unless a tiny explicit fixture is intentionally added later
for tests.

## Default Storage

Recommended local directory:

```text
data/real/public_logs/
```

Recommended `.gitignore` rule:

```text
data/real/public_logs/
```

## Safe Defaults

The project wrapper should use conservative defaults:

- `--max-num 10`
- `--download-folder data/real/public_logs/`
- `--delay 6`
- skip already downloaded logs by default
- require confirmation for large downloads
- support a print-only mode before download

The downloader must be resumable in practice: if a `.ulg` file already exists,
the script should skip it unless `--overwrite` is explicitly set.

## Filters To Support

The first useful filter set should cover:

- MAV type, for example `Quadrotor`
- flight modes, for example `Mission`
- rating, for example `Good`
- airframe type
- airframe name
- PX4 git hash
- upload source
- specific log id
- latest log per vehicle

These filters should make it possible to build a smaller real-log dataset that
is close to the SITL scenario being tested.

## API Shape

The upstream script uses two server endpoints:

```text
https://review.px4.io/dbinfo
https://review.px4.io/download?log=<log_id>
```

The implementation should keep these URLs configurable:

```bash
python3 scripts/download_public_logs.py \
  --db-info-api https://review.px4.io/dbinfo \
  --download-api https://review.px4.io/download
```

## Recommended Commands

Inspect matching public logs without downloading:

```bash
python3 scripts/download_public_logs.py \
  --print \
  --mav-type Quadrotor \
  --flight-modes Mission \
  --max-num 10
```

Download a small starter dataset:

```bash
python3 scripts/download_public_logs.py \
  --mav-type Quadrotor \
  --flight-modes Mission \
  --rating Good \
  --max-num 10 \
  --download-folder data/real/public_logs/
```

Compare a downloaded real log with a simulation artifact:

```bash
python3 scripts/compare_logs.py \
  --config scenarios/takeoff_land_20m.yaml \
  --sim results/runs/<run_id>/px4.ulg \
  --real data/real/public_logs/<log_id>.ulg
```

## Implementation Choice

The project uses a project-native wrapper.

- Keep only the features needed by this project.
- Use the same public API model as Flight Review.
- Avoid copying upstream code directly.
- Easier to test and maintain inside this repository.

## License And Attribution

This project does not vendor the upstream downloader script. It uses the public
API shape and references the upstream repository in the docs.

## Rate Limits And Etiquette

Public Flight Review hosting has network and storage costs. The downloader must
avoid aggressive bulk downloads:

- keep a delay between file downloads
- retry gently on temporary errors
- stop clearly on access block or repeated failures
- warn before large downloads
- prefer focused filters over broad mirroring

## Integration With Existing Compare Tools

After logs are downloaded, the next useful project change is to make
`scripts/batch_compare.py` accept a directory of real logs as well as a directory
of simulation logs. For example:

```bash
python3 scripts/batch_compare.py \
  --config scenarios/takeoff_land_20m.yaml \
  --sim-dir results/runs/ \
  --real-dir data/real/public_logs/
```

That should be a later step. The first implementation should only download and
organize public logs safely.
