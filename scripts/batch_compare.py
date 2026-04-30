import os
import subprocess

SIM_DIR = "data/sim/"
REAL_LOG = "data/real/real_log.ulg"

for sim_log in os.listdir(SIM_DIR):
    if sim_log.endswith(".ulg"):
        print(f"Comparing {sim_log}...")
        
        cmd = [
            "python3",
            "scripts/compare_logs.py",
            "--sim", os.path.join(SIM_DIR, sim_log),
            "--real", REAL_LOG
        ]
        
        subprocess.run(cmd)
