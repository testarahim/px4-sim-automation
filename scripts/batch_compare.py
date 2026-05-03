import subprocess
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
SIM_DIR = PROJECT_DIR / "data" / "sim"
REAL_LOG = PROJECT_DIR / "data" / "real" / "real_log.ulg"
COMPARISON_DIR = PROJECT_DIR / "results" / "comparisons"
PLOT_DIR = PROJECT_DIR / "results" / "plots" / "comparisons"


def main():
    if not SIM_DIR.exists():
        raise FileNotFoundError(f"Simulation log directory not found: {SIM_DIR}")
    if not REAL_LOG.exists():
        raise FileNotFoundError(f"Real log not found: {REAL_LOG}")

    COMPARISON_DIR.mkdir(parents=True, exist_ok=True)

    for sim_log in sorted(SIM_DIR.glob("*.ulg")):
        print(f"Comparing {sim_log.name}...")

        cmd = [
            "python3",
            str(PROJECT_DIR / "scripts" / "compare_logs.py"),
            "--sim",
            str(sim_log),
            "--real",
            str(REAL_LOG),
            "--metrics",
            str(COMPARISON_DIR / f"{sim_log.stem}_metrics.json"),
            "--plot-dir",
            str(PLOT_DIR / sim_log.stem),
        ]

        subprocess.run(cmd, cwd=PROJECT_DIR, check=False)


if __name__ == "__main__":
    main()
