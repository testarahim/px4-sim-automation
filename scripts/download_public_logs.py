import argparse
import datetime
import json
import sys
import time
from pathlib import Path

import requests


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_DOWNLOAD_FOLDER = PROJECT_DIR / "data" / "real" / "public_logs"
DEFAULT_DB_INFO_API = "https://review.px4.io/dbinfo"
DEFAULT_DOWNLOAD_API = "https://review.px4.io/download"
DEFAULT_DELAY_SECONDS = 6.0
DEFAULT_MAX_NUM = 10
WARN_THRESHOLD = 100
REQUEST_TIMEOUT_S = 10 * 60
CHUNK_SIZE = 1024 * 1024
METADATA_DURATION_KEYS = (
    "duration_s",
    "duration",
    "duration_sec",
    "duration_seconds",
)
METADATA_WARNING_KEYS = (
    "num_logged_warnings",
    "logged_warnings",
    "warning_count",
    "warnings",
)
METADATA_ERROR_KEYS = (
    "num_logged_errors",
    "logged_errors",
    "error_count",
    "errors",
)

FLIGHT_MODE_IDS = {
    "manual": 0,
    "altitude": 1,
    "position": 2,
    "mission": 3,
    "loiter": 4,
    "return to land": 5,
    "rtl": 5,
    "position slow": 6,
    "acro": 10,
    "descend": 12,
    "terminate": 13,
    "offboard": 14,
    "stabilized": 15,
    "takeoff": 17,
    "land": 18,
    "follow target": 19,
    "precision land": 20,
    "orbit": 21,
    "vtol takeoff": 22,
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Download public PX4 Flight Review logs.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--max-num",
        "-n",
        type=int,
        default=DEFAULT_MAX_NUM,
        help="Maximum matching logs to download or print. Use -1 for all.",
    )
    parser.add_argument(
        "--download-folder",
        type=Path,
        default=DEFAULT_DOWNLOAD_FOLDER,
        help="Directory where downloaded .ulg files are stored.",
    )
    parser.add_argument(
        "--print",
        dest="print_entries",
        action="store_true",
        help="Print matching database entries instead of downloading logs.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite already downloaded .ulg files.",
    )
    parser.add_argument(
        "--db-info-api",
        default=DEFAULT_DB_INFO_API,
        help="Flight Review dbinfo API URL.",
    )
    parser.add_argument(
        "--download-api",
        default=DEFAULT_DOWNLOAD_API,
        help="Flight Review download API URL.",
    )
    parser.add_argument(
        "--mav-type",
        nargs="+",
        default=None,
        help="Filter by MAV type, for example Quadrotor.",
    )
    parser.add_argument(
        "--flight-modes",
        nargs="+",
        default=None,
        help="Filter by flight modes. All listed modes must be present.",
    )
    parser.add_argument(
        "--rating",
        nargs="+",
        default=None,
        help="Filter by rating, for example Good.",
    )
    parser.add_argument(
        "--airframe-type",
        default=None,
        help="Filter by exact airframe type.",
    )
    parser.add_argument(
        "--airframe-name",
        default=None,
        help="Filter by exact airframe name.",
    )
    parser.add_argument(
        "--git-hash",
        default=None,
        help="Filter by PX4 git hash.",
    )
    parser.add_argument(
        "--source",
        default=None,
        help="Filter by upload source, for example CI.",
    )
    parser.add_argument(
        "--sys-hw",
        nargs="+",
        default=None,
        help="Filter by system hardware label, for example PX4_FMU_V5.",
    )
    parser.add_argument(
        "--exclude-sys-hw",
        nargs="+",
        default=None,
        help="Exclude system hardware labels, for example PX4_SITL.",
    )
    parser.add_argument(
        "--log-id",
        nargs="+",
        default=None,
        help="Filter by specific log id. Dashes are ignored for matching.",
    )
    parser.add_argument(
        "--latest-per-vehicle",
        action="store_true",
        help="Keep only the latest log per vehicle UUID.",
    )
    parser.add_argument(
        "--min-duration-s",
        type=float,
        default=None,
        help="Keep logs with metadata duration greater than or equal to this many seconds.",
    )
    parser.add_argument(
        "--max-duration-s",
        type=float,
        default=None,
        help="Keep logs with metadata duration less than or equal to this many seconds.",
    )
    parser.add_argument(
        "--max-logged-warnings",
        type=int,
        default=None,
        help="Keep logs with at most this many logged warnings in metadata.",
    )
    parser.add_argument(
        "--max-logged-errors",
        type=int,
        default=None,
        help="Keep logs with at most this many logged errors in metadata.",
    )
    parser.add_argument(
        "--min-flight-mode-duration-s",
        nargs=2,
        action="append",
        metavar=("MODE", "SECONDS"),
        default=None,
        help="Keep logs where MODE has at least SECONDS total duration.",
    )
    parser.add_argument(
        "--max-flight-mode-duration-s",
        nargs=2,
        action="append",
        metavar=("MODE", "SECONDS"),
        default=None,
        help="Keep logs where MODE has at most SECONDS total duration.",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=DEFAULT_DELAY_SECONDS,
        help="Delay in seconds between downloads.",
    )
    parser.add_argument(
        "--yes",
        "-y",
        action="store_true",
        help="Skip confirmation prompt for large downloads.",
    )
    return parser.parse_args()


def normalize(value):
    return str(value).strip().lower()


def normalize_log_id(value):
    return normalize(value).replace("-", "")


def validate_non_negative(name, value):
    if value is not None and value < 0:
        raise ValueError(f"{name} must be non-negative")


def validate_args(args):
    validate_non_negative("--min-duration-s", args.min_duration_s)
    validate_non_negative("--max-duration-s", args.max_duration_s)
    validate_non_negative("--max-logged-warnings", args.max_logged_warnings)
    validate_non_negative("--max-logged-errors", args.max_logged_errors)
    args.min_flight_mode_duration_s = parse_mode_duration_filters(
        args.min_flight_mode_duration_s,
        "--min-flight-mode-duration-s",
    )
    args.max_flight_mode_duration_s = parse_mode_duration_filters(
        args.max_flight_mode_duration_s,
        "--max-flight-mode-duration-s",
    )
    if (
        args.min_duration_s is not None
        and args.max_duration_s is not None
        and args.min_duration_s > args.max_duration_s
    ):
        raise ValueError("--min-duration-s cannot be greater than --max-duration-s")


def parse_log_date(entry):
    value = entry.get("log_date") or ""
    try:
        return datetime.datetime.strptime(value, "%Y-%m-%d")
    except ValueError:
        return datetime.datetime.min


def selected_entries(entries, max_num):
    if max_num < 0:
        return entries
    return entries[:max_num]


def fetch_db_entries(db_info_api):
    print(f"Fetching database info: {db_info_api}", flush=True)
    response = requests.get(db_info_api, timeout=5 * 60)
    response.raise_for_status()
    entries = response.json()
    if not isinstance(entries, list):
        raise ValueError("dbinfo API did not return a list")
    print(f"Found {len(entries)} public logs.", flush=True)
    return entries


def value_matches_any(entry, key, requested_values):
    if requested_values is None:
        return True
    entry_value = entry.get(key)
    if entry_value is None:
        return False
    requested = {normalize(value) for value in requested_values}
    return normalize(entry_value) in requested


def value_matches_exact(entry, key, requested_value):
    if requested_value is None:
        return True
    entry_value = entry.get(key)
    return entry_value is not None and normalize(entry_value) == normalize(requested_value)


def value_excludes_any(entry, key, excluded_values):
    if excluded_values is None:
        return True
    entry_value = entry.get(key)
    if entry_value is None:
        return True
    excluded = {normalize(value) for value in excluded_values}
    return normalize(entry_value) not in excluded


def flight_mode_token(value):
    normalized = normalize(value).replace("(", "").replace(")", "")
    if normalized.isdigit():
        return int(normalized)
    if normalized not in FLIGHT_MODE_IDS:
        supported = ", ".join(sorted(FLIGHT_MODE_IDS))
        raise ValueError(f"Unknown flight mode: {value}. Supported labels: {supported}")
    return FLIGHT_MODE_IDS[normalized]


def parse_mode_duration_filters(filters, argument_name):
    parsed = []
    if filters is None:
        return parsed

    for mode, duration_s in filters:
        try:
            duration = float(duration_s)
        except ValueError as exc:
            raise ValueError(
                f"{argument_name} duration must be numeric: {duration_s}"
            ) from exc
        if duration < 0:
            raise ValueError(f"{argument_name} duration must be non-negative")
        parsed.append((flight_mode_token(mode), duration))

    return parsed


def entry_has_flight_modes(entry, requested_modes):
    if requested_modes is None:
        return True

    entry_modes = entry.get("flight_modes")
    if entry_modes is None:
        return False

    available = set()
    for mode in entry_modes:
        try:
            available.add(flight_mode_token(mode))
        except ValueError:
            available.add(normalize(mode))

    requested = {flight_mode_token(mode) for mode in requested_modes}
    return requested.issubset(available)


def flight_mode_durations(entry):
    durations = {}
    for item in entry.get("flight_mode_durations") or []:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            continue
        mode, duration_s = item
        try:
            mode_id = flight_mode_token(mode)
            duration = float(duration_s)
        except (TypeError, ValueError):
            continue
        durations[mode_id] = durations.get(mode_id, 0.0) + duration
    return durations


def entry_matches_mode_duration_filters(entry, minimum_filters, maximum_filters):
    if not minimum_filters and not maximum_filters:
        return True

    durations = flight_mode_durations(entry)
    if not durations:
        return False

    for mode, minimum in minimum_filters or []:
        if durations.get(mode, 0.0) < minimum:
            return False

    for mode, maximum in maximum_filters or []:
        if mode not in durations or durations[mode] > maximum:
            return False

    return True


def entry_matches_log_ids(entry, requested_log_ids):
    if requested_log_ids is None:
        return True
    entry_id = entry.get("log_id")
    if entry_id is None:
        return False
    requested = {normalize_log_id(log_id) for log_id in requested_log_ids}
    return normalize_log_id(entry_id) in requested


def numeric_metadata_value(entry, keys):
    for key in keys:
        if key not in entry or entry[key] is None:
            continue

        value = entry[key]
        if isinstance(value, bool):
            continue
        if isinstance(value, (int, float)):
            return float(value)
        if isinstance(value, str):
            try:
                return float(value.strip())
            except ValueError:
                continue
        if isinstance(value, (list, tuple, set, dict)):
            return float(len(value))

    return None


def metadata_number_in_range(entry, keys, minimum=None, maximum=None):
    if minimum is None and maximum is None:
        return True

    value = numeric_metadata_value(entry, keys)
    if value is None:
        return False
    if minimum is not None and value < minimum:
        return False
    if maximum is not None and value > maximum:
        return False
    return True


def filter_latest_per_vehicle(entries):
    latest_by_uuid = {}
    for entry in entries:
        uuid = entry.get("vehicle_uuid")
        if not uuid:
            continue
        date = parse_log_date(entry)
        if uuid not in latest_by_uuid or date > latest_by_uuid[uuid][0]:
            latest_by_uuid[uuid] = (date, entry)
    return [entry for _, entry in latest_by_uuid.values()]


def filter_entries(entries, args):
    filtered = [
        entry
        for entry in entries
        if value_matches_any(entry, "mav_type", args.mav_type)
        and value_matches_any(entry, "rating", args.rating)
        and value_matches_exact(entry, "airframe_type", args.airframe_type)
        and value_matches_exact(entry, "airframe_name", args.airframe_name)
        and value_matches_exact(entry, "ver_sw", args.git_hash)
        and value_matches_exact(entry, "source", args.source)
        and value_matches_any(entry, "sys_hw", args.sys_hw)
        and value_excludes_any(entry, "sys_hw", args.exclude_sys_hw)
        and entry_matches_log_ids(entry, args.log_id)
        and entry_has_flight_modes(entry, args.flight_modes)
        and entry_matches_mode_duration_filters(
            entry,
            args.min_flight_mode_duration_s,
            args.max_flight_mode_duration_s,
        )
        and metadata_number_in_range(
            entry,
            METADATA_DURATION_KEYS,
            minimum=args.min_duration_s,
            maximum=args.max_duration_s,
        )
        and metadata_number_in_range(
            entry,
            METADATA_WARNING_KEYS,
            maximum=args.max_logged_warnings,
        )
        and metadata_number_in_range(
            entry,
            METADATA_ERROR_KEYS,
            maximum=args.max_logged_errors,
        )
    ]

    if args.latest_per_vehicle:
        filtered = filter_latest_per_vehicle(filtered)

    return sorted(filtered, key=parse_log_date, reverse=True)


def existing_log_ids(download_folder):
    if not download_folder.exists():
        return set()
    return {
        path.stem
        for path in download_folder.glob("*.ulg")
        if path.is_file()
    }


def confirm_large_download(n_files):
    print()
    print("=" * 60)
    print(f"WARNING: You are about to download {n_files} files.")
    print("=" * 60)
    print("Use focused filters and keep delays to respect Flight Review hosting.")
    response = input("Continue with download? [y/N]: ")
    return response.strip().lower() in {"y", "yes"}


def download_request(download_api, log_id, max_retries=5):
    url = f"{download_api}?log={log_id}"

    for attempt in range(max_retries):
        try:
            response = requests.get(url, stream=True, timeout=REQUEST_TIMEOUT_S)
        except requests.exceptions.RequestException as exc:
            wait_s = min(10 * (attempt + 1), 60)
            print(f"Request failed for {log_id}: {exc}. Retrying in {wait_s}s.")
            time.sleep(wait_s)
            continue

        if response.status_code == 200:
            return response
        if response.status_code == 404:
            print(f"Log not found: {log_id}")
            return None
        if response.status_code == 503:
            retry_after = response.headers.get("Retry-After")
            wait_s = int(retry_after) if retry_after else min(30 * (2**attempt), 300)
            print(f"Rate limited while downloading {log_id}. Waiting {wait_s}s.")
            time.sleep(wait_s)
            continue
        if response.status_code in {403, 444}:
            raise RuntimeError(
                f"Download blocked by server while requesting {log_id} "
                f"(HTTP {response.status_code})"
            )

        wait_s = min(10 * (attempt + 1), 60)
        print(
            f"Unexpected HTTP {response.status_code} for {log_id}. "
            f"Retrying in {wait_s}s."
        )
        time.sleep(wait_s)

    print(f"Failed after {max_retries} attempts: {log_id}")
    return None


def download_log(entry, args, download_folder):
    log_id = entry["log_id"]
    target_path = download_folder / f"{log_id}.ulg"
    response = download_request(args.download_api, log_id)
    if response is None:
        return False

    with target_path.open("wb") as log_file:
        for chunk in response.iter_content(chunk_size=CHUNK_SIZE):
            if chunk:
                log_file.write(chunk)
    return True


def print_entries(entries):
    print(json.dumps(entries, indent=4, sort_keys=True))


def main():
    args = parse_args()
    validate_args(args)
    download_folder = (
        args.download_folder
        if args.download_folder.is_absolute()
        else PROJECT_DIR / args.download_folder
    )

    entries = fetch_db_entries(args.db_info_api)
    entries = filter_entries(entries, args)
    print(f"{len(entries)} logs match the filter criteria.", flush=True)
    entries = selected_entries(entries, args.max_num)

    if args.print_entries:
        print_entries(entries)
        return 0

    if len(entries) > WARN_THRESHOLD and not args.yes:
        if not confirm_large_download(len(entries)):
            print("Download cancelled.")
            return 0

    download_folder.mkdir(parents=True, exist_ok=True)
    downloaded_ids = existing_log_ids(download_folder)

    n_downloaded = 0
    n_skipped = 0
    n_failed = 0
    for index, entry in enumerate(entries, start=1):
        log_id = entry.get("log_id")
        if not log_id:
            n_failed += 1
            print("Skipping entry without log_id.")
            continue
        if not args.overwrite and log_id in downloaded_ids:
            n_skipped += 1
            print(f"Skipping existing log {index}/{len(entries)}: {log_id}")
            continue

        print(f"Downloading {index}/{len(entries)}: {log_id}", flush=True)
        if download_log(entry, args, download_folder):
            n_downloaded += 1
        else:
            n_failed += 1

        if index < len(entries):
            time.sleep(args.delay)

    print()
    print(f"Download folder: {download_folder}")
    print(f"Downloaded: {n_downloaded}")
    print(f"Skipped: {n_skipped}")
    print(f"Failed: {n_failed}")

    return 1 if n_failed else 0


if __name__ == "__main__":
    sys.exit(main())
