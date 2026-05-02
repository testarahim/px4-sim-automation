import shutil
import os
from pathlib import Path

PX4_DIR = Path(os.environ.get("PX4_DIR", "~/PX4-Autopilot")).expanduser()
PX4_LOG_DIR = PX4_DIR / "build" / "px4_sitl_default" / "rootfs" / "log"
DEST_DIR = Path("logs")

def copy_latest_log():
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

    print(f"Log kopyalandı: {archived_log}")
    print(f"Latest alias güncellendi: {latest_log}")

if __name__ == "__main__":
    copy_latest_log()
