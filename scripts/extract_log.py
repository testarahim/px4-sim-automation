import shutil
import os
import argparse
from pathlib import Path

PX4_DIR = Path(os.environ.get("PX4_DIR", "~/PX4-Autopilot")).expanduser()
PX4_LOG_DIR = PX4_DIR / "build" / "px4_sitl_default" / "rootfs" / "log"
DEST_DIR = Path("logs")


def parse_args():
    parser = argparse.ArgumentParser(description="Copy the latest PX4 .ulg log.")
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        help="Optional run artifact directory that receives px4.ulg.",
    )
    return parser.parse_args()


def copy_latest_log(artifact_dir=None):
    if not PX4_LOG_DIR.exists():
        raise FileNotFoundError(f"PX4 log dizini bulunamadı: {PX4_LOG_DIR}")

    logs = sorted(PX4_LOG_DIR.glob("*/*.ulg"), key=lambda path: path.stat().st_mtime)
    if not logs:
        raise FileNotFoundError(f"PX4 log dizininde .ulg bulunamadı: {PX4_LOG_DIR}")

    latest = logs[-1]
    run_dir = DEST_DIR / latest.parent.name
    run_dir.mkdir(parents=True, exist_ok=True)

    archived_log = run_dir / latest.name
    latest_log = DEST_DIR / "latest_log.ulg"

    shutil.copy2(latest, archived_log)
    shutil.copy2(latest, latest_log)

    if artifact_dir is not None:
        artifact_dir.mkdir(parents=True, exist_ok=True)
        artifact_log = artifact_dir / "px4.ulg"
        shutil.copy2(latest, artifact_log)
        print(f"Run artifact log kopyalandı: {artifact_log}")

    print(f"Log kopyalandı: {archived_log}")
    print(f"Latest alias güncellendi: {latest_log}")


if __name__ == "__main__":
    args = parse_args()
    copy_latest_log(args.artifact_dir)
