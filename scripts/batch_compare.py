import argparse
import subprocess
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
SIM_DIR = PROJECT_DIR / "data" / "sim"
REAL_LOG = PROJECT_DIR / "data" / "real" / "real_log.ulg"
COMPARISON_DIR = PROJECT_DIR / "results" / "comparisons"
PLOT_DIR = PROJECT_DIR / "results" / "plots" / "comparisons"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare every simulation log against one real PX4 log."
    )
    parser.add_argument(
        "--sim-dir",
        type=Path,
        default=SIM_DIR,
        help=f"Directory containing simulation .ulg files. Default: {SIM_DIR}",
    )
    parser.add_argument(
        "--real",
        type=Path,
        default=REAL_LOG,
        help=f"Real flight .ulg path. Default: {REAL_LOG}",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Optional scenario/config YAML path passed to compare_logs.py.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    sim_dir = args.sim_dir if args.sim_dir.is_absolute() else PROJECT_DIR / args.sim_dir
    real_log = args.real if args.real.is_absolute() else PROJECT_DIR / args.real

    if not sim_dir.exists():
        raise FileNotFoundError(f"Simulation log directory not found: {sim_dir}")
    if not real_log.exists():
        raise FileNotFoundError(f"Real log not found: {real_log}")

    COMPARISON_DIR.mkdir(parents=True, exist_ok=True)

    for sim_log in sorted(sim_dir.glob("*.ulg")):
        print(f"Comparing {sim_log.name}...")

        cmd = [
            "python3",
            str(PROJECT_DIR / "scripts" / "compare_logs.py"),
            "--sim",
            str(sim_log),
            "--real",
            str(real_log),
            "--metrics",
            str(COMPARISON_DIR / f"{sim_log.stem}_metrics.json"),
            "--plot-dir",
            str(PLOT_DIR / sim_log.stem),
        ]
        if args.config:
            config = args.config if args.config.is_absolute() else PROJECT_DIR / args.config
            cmd.extend(["--config", str(config)])

        subprocess.run(cmd, cwd=PROJECT_DIR, check=False)


if __name__ == "__main__":
    main()
