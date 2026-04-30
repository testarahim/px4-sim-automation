import shutil
import os

PX4_LOG_DIR = os.path.expanduser("~/PX4-Autopilot/build/px4_sitl_default/logs/")
DEST_DIR = "./logs/"

def copy_latest_log():
    logs = sorted(os.listdir(PX4_LOG_DIR))
    latest = logs[-1]

    src = os.path.join(PX4_LOG_DIR, latest)
    dst = os.path.join(DEST_DIR, latest)

    shutil.copytree(src, dst)
    print(f"Log kopyalandı: {latest}")

if __name__ == "__main__":
    copy_latest_log()
