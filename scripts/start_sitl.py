import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path


PX4_DIR = Path(os.environ.get("PX4_DIR", "~/PX4-Autopilot")).expanduser()
SITL_TARGET = os.environ.get("PX4_SITL_TARGET", "gz_x500")
SHUTDOWN_COMMAND_TIMEOUT_S = 8
SIGINT_TIMEOUT_S = 8
SIGTERM_TIMEOUT_S = 5
OUTPUT_READ_SIZE = 4096
ROOTFS_SYMLINKS = {
    "etc": PX4_DIR / "build" / "px4_sitl_default" / "etc",
    "test_data": PX4_DIR / "test_data",
}

ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
PX4_PROMPT = "pxh>"
PX4_SHUTDOWN_COMMAND = "shutdown"
EXPECTED_SHUTDOWN_NOISE_PREFIXES = (
    "ninja: build stopped: interrupted by user",
    "make: *** [Makefile:232: px4_sitl] Error 130",
    "make: *** wait: No child processes.",
    "make: *** Waiting for unfinished jobs",
)
STOP_REQUESTED = False


def clean_log_fragment(fragment):
    clean_fragment = ANSI_ESCAPE_RE.sub("", fragment).strip()
    while clean_fragment.startswith(PX4_PROMPT):
        clean_fragment = clean_fragment[len(PX4_PROMPT) :].lstrip()
    return clean_fragment


def should_log_fragment(fragment):
    if not fragment:
        return False
    if PX4_SHUTDOWN_COMMAND.startswith(fragment):
        return False
    return not any(
        fragment.startswith(prefix) for prefix in EXPECTED_SHUTDOWN_NOISE_PREFIXES
    )


def write_clean_output(text):
    clean_text = clean_log_fragment(text)
    if should_log_fragment(clean_text):
        print(clean_text, flush=True)


def stream_clean_output(stream):
    if stream is None:
        return

    buffer = ""
    file_descriptor = stream.fileno()

    while True:
        try:
            chunk = os.read(file_descriptor, OUTPUT_READ_SIZE)
        except OSError:
            break

        if not chunk:
            break

        buffer += chunk.decode("utf-8", errors="replace")
        parts = re.split(r"[\r\n]", buffer)
        buffer = parts.pop()

        for part in parts:
            write_clean_output(part)

    write_clean_output(buffer)


def stop_sitl(process):
    if process.poll() is not None:
        return

    if process.stdin and not process.stdin.closed:
        try:
            print("PX4 shutdown komutu gonderiliyor...", flush=True)
            process.stdin.write(b"shutdown\n")
            process.stdin.flush()
            process.wait(timeout=SHUTDOWN_COMMAND_TIMEOUT_S)
            return
        except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
            pass

    for signal_number, timeout in (
        (signal.SIGINT, SIGINT_TIMEOUT_S),
        (signal.SIGTERM, SIGTERM_TIMEOUT_S),
    ):
        if process.poll() is not None:
            return

        try:
            os.killpg(process.pid, signal_number)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass

    if process.poll() is None:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def start_sitl():
    if not PX4_DIR.exists():
        raise FileNotFoundError(f"PX4 dizini bulunamadı: {PX4_DIR}")

    cleanup_rootfs_symlinks()

    env = os.environ.copy()
    env.setdefault("HEADLESS", "1")

    cmd = ["make", "px4_sitl", SITL_TARGET]
    print(f"PX4 SITL başlatılıyor: {' '.join(cmd)}", flush=True)
    print(f"PX4 dizini: {PX4_DIR}", flush=True)

    return subprocess.Popen(
        cmd,
        cwd=PX4_DIR,
        env=env,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=False,
        preexec_fn=os.setsid,
    )


def cleanup_rootfs_symlinks():
    rootfs_dir = PX4_DIR / "build" / "px4_sitl_default" / "rootfs"
    for link_name, expected_target in ROOTFS_SYMLINKS.items():
        link_path = rootfs_dir / link_name
        if not link_path.is_symlink():
            continue

        try:
            current_target = link_path.resolve(strict=False)
        except OSError:
            current_target = None

        if current_target == expected_target:
            link_path.unlink()
            print(f"Stale PX4 rootfs symlink temizlendi: {link_path}", flush=True)


def main():
    global STOP_REQUESTED

    process = start_sitl()
    output_thread = threading.Thread(
        target=stream_clean_output,
        args=(process.stdout,),
        daemon=True,
    )
    output_thread.start()

    def handle_stop(signum, frame):
        global STOP_REQUESTED
        STOP_REQUESTED = True

    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)

    try:
        while process.poll() is None:
            if STOP_REQUESTED:
                stop_sitl(process)
                break
            time.sleep(0.2)
    finally:
        stop_sitl(process)
        output_thread.join(timeout=2)

    if STOP_REQUESTED:
        return 0
    return process.returncode or 0


if __name__ == "__main__":
    sys.exit(main())
