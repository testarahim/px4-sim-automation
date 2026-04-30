#!/bin/bash

echo "Simulation başlatılıyor..."

python3 scripts/start_sitl.py &
SITL_PID=$!

sleep 10

python3 scripts/run_mission.py

sleep 10

python3 scripts/extract_log.py
python3 scripts/analyze_log.py

echo "Tamamlandı."

kill $SITL_PID
