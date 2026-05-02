#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib}"

SITL_PID=""
SITL_LOG="$PROJECT_DIR/logs/sitl.log"
SITL_SHUTDOWN_TIMEOUT=15

sitl_process_is_running() {
    if [[ -z "$SITL_PID" ]] || ! kill -0 "$SITL_PID" 2>/dev/null; then
        return 1
    fi

    local state
    state="$(ps -o stat= -p "$SITL_PID" 2>/dev/null || true)"
    [[ "$state" != Z* ]]
}

cleanup() {
    if sitl_process_is_running; then
        echo "Simulation kapatılıyor..."
        kill "$SITL_PID" 2>/dev/null || true

        for _ in $(seq 1 "$SITL_SHUTDOWN_TIMEOUT"); do
            if ! sitl_process_is_running; then
                wait "$SITL_PID" 2>/dev/null || true
                return
            fi
            sleep 1
        done

        echo "SITL kapanmadı, zorla kapatılıyor..."
        kill -KILL "$SITL_PID" 2>/dev/null || true
        wait "$SITL_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

cd "$PROJECT_DIR"
mkdir -p "$PROJECT_DIR/logs"

echo "Simulation başlatılıyor..."
echo "SITL log: $SITL_LOG"
python3 scripts/start_sitl.py >"$SITL_LOG" 2>&1 &
SITL_PID=$!

echo "SITL hazır olma sinyali bekleniyor..."
python3 scripts/wait_for_sitl_ready.py --log "$SITL_LOG" --pid "$SITL_PID"

python3 scripts/run_mission.py
python3 scripts/extract_log.py
python3 scripts/analyze_log.py

if [[ -f data/real/real_log.ulg && -f data/sim/sim_log.ulg ]]; then
    echo "Sim vs Real karşılaştırılıyor..."
    python3 scripts/compare_logs.py
else
    echo "Sim vs Real karşılaştırması atlandı: data/real/real_log.ulg ve data/sim/sim_log.ulg gerekiyor."
fi

python3 - <<'PY'
import json
import sys

with open("results/metrics.json", "r", encoding="utf-8") as metrics_file:
    metrics = json.load(metrics_file)

overall_pass = metrics.get("evaluation", {}).get("overall_pass", False)
print(f"Overall pass: {overall_pass}")
sys.exit(0 if overall_pass else 1)
PY

echo "Tamamlandı."
