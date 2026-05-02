import argparse
import os
import time
from collections import deque
from pathlib import Path

import yaml


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = PROJECT_DIR / "config" / "sim_config.yaml"
DEFAULT_LOG_PATH = PROJECT_DIR / "logs" / "sitl.log"
DEFAULT_READY_PATTERNS = ["Startup script returned successfully"]
DEFAULT_TIMEOUT_S = 60
POLL_INTERVAL_S = 0.25
TAIL_LINES = 40


def load_config(config_path):
    if not config_path.exists():
        return {}

    with config_path.open("r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def config_ready_patterns(sitl_config):
    patterns = sitl_config.get("startup_ready_patterns", DEFAULT_READY_PATTERNS)
    if isinstance(patterns, str):
        patterns = [patterns]

    if not isinstance(patterns, list) or not all(
        isinstance(pattern, str) and pattern for pattern in patterns
    ):
        raise ValueError("sitl.startup_ready_patterns must be a non-empty string list")

    return patterns


def config_timeout(sitl_config):
    timeout = sitl_config.get(
        "startup_timeout",
        sitl_config.get("startup_delay", DEFAULT_TIMEOUT_S),
    )

    if not isinstance(timeout, (int, float)) or timeout <= 0:
        raise ValueError("sitl.startup_timeout must be a positive number")

    return float(timeout)


def process_is_running(pid):
    if pid is None:
        return True

    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True

    stat_path = Path("/proc") / str(pid) / "stat"
    if stat_path.exists():
        try:
            stat = stat_path.read_text(encoding="utf-8")
        except OSError:
            return True

        try:
            state = stat.split(") ", 1)[1][0]
        except IndexError:
            return True

        return state != "Z"

    return True


def tail_log(log_path):
    if not log_path.exists():
        return ""

    with log_path.open("r", encoding="utf-8", errors="replace") as log_file:
        return "".join(deque(log_file, maxlen=TAIL_LINES))


def wait_for_ready(log_path, ready_patterns, timeout, pid=None):
    deadline = time.monotonic() + timeout
    position = 0

    while time.monotonic() < deadline:
        if log_path.exists():
            with log_path.open("r", encoding="utf-8", errors="replace") as log_file:
                log_file.seek(position)
                chunk = log_file.read()
                position = log_file.tell()

            for pattern in ready_patterns:
                if pattern in chunk:
                    elapsed = timeout - (deadline - time.monotonic())
                    print(
                        f"SITL ready after {elapsed:.1f}s "
                        f"(matched: {pattern})",
                        flush=True,
                    )
                    return

        if not process_is_running(pid):
            raise RuntimeError(
                "SITL process exited before readiness signal.\n"
                f"Last log lines:\n{tail_log(log_path)}"
            )

        time.sleep(POLL_INTERVAL_S)

    raise TimeoutError(
        f"SITL readiness timed out after {timeout:.1f}s. "
        f"Expected one of: {', '.join(ready_patterns)}\n"
        f"Last log lines:\n{tail_log(log_path)}"
    )


def parse_args():
    parser = argparse.ArgumentParser(description="Wait for PX4 SITL startup readiness")
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="Path to sim_config.yaml",
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=DEFAULT_LOG_PATH,
        help="Path to the SITL stdout/stderr log",
    )
    parser.add_argument(
        "--pid",
        type=int,
        default=None,
        help="Optional SITL wrapper process id to monitor",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    config = load_config(args.config)
    sitl_config = config.get("sitl", {})
    ready_patterns = config_ready_patterns(sitl_config)
    timeout = config_timeout(sitl_config)

    print(
        f"Waiting for SITL readiness in {args.log} "
        f"(timeout: {timeout:.1f}s)",
        flush=True,
    )
    wait_for_ready(args.log, ready_patterns, timeout, pid=args.pid)


if __name__ == "__main__":
    main()
