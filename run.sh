#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib}"

DEFAULT_SCENARIO="$PROJECT_DIR/scenarios/takeoff_land_20m.yaml"
SCENARIO_CONFIG="${1:-$DEFAULT_SCENARIO}"
SITL_PID=""
SITL_LOG="$PROJECT_DIR/logs/sitl.log"
SITL_SHUTDOWN_TIMEOUT=15
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
RUN_DIR="$PROJECT_DIR/results/runs/$RUN_ID"
ARTIFACT_SITL_LOG="$RUN_DIR/sitl.log"

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

archive_sitl_log() {
    if [[ -f "$SITL_LOG" ]]; then
        mkdir -p "$RUN_DIR"
        cp "$SITL_LOG" "$ARTIFACT_SITL_LOG" || true
    fi
}

on_exit() {
    cleanup
    archive_sitl_log
}
trap on_exit EXIT

cd "$PROJECT_DIR"
if [[ ! -f "$SCENARIO_CONFIG" ]]; then
    echo "Scenario/config dosyası bulunamadı: $SCENARIO_CONFIG" >&2
    echo "Kullanım: bash run.sh [scenarios/takeoff_land_20m.yaml]" >&2
    exit 2
fi

mkdir -p "$PROJECT_DIR/logs" "$PROJECT_DIR/results/plots" "$RUN_DIR"

echo "Simulation başlatılıyor..."
echo "Scenario/config: $SCENARIO_CONFIG"
echo "SITL log: $SITL_LOG"
echo "Run artifacts: $RUN_DIR"
python3 scripts/start_sitl.py >"$SITL_LOG" 2>&1 &
SITL_PID=$!

echo "SITL hazır olma sinyali bekleniyor..."
python3 scripts/wait_for_sitl_ready.py --config "$SCENARIO_CONFIG" --log "$SITL_LOG" --pid "$SITL_PID"

python3 scripts/run_mission.py --config "$SCENARIO_CONFIG"
python3 scripts/extract_log.py --artifact-dir "$RUN_DIR"
python3 scripts/analyze_log.py \
    --config "$SCENARIO_CONFIG" \
    --log "$RUN_DIR/px4.ulg" \
    --metrics "$RUN_DIR/metrics.json" \
    --plot "$RUN_DIR/altitude.png"

if [[ -f data/real/real_log.ulg && -f data/sim/sim_log.ulg ]]; then
    echo "Sim vs Real karşılaştırılıyor..."
    python3 scripts/compare_logs.py
else
    echo "Sim vs Real karşılaştırması atlandı: data/real/real_log.ulg ve data/sim/sim_log.ulg gerekiyor."
fi

cp "$RUN_DIR/metrics.json" "$PROJECT_DIR/results/metrics.json"
cp "$RUN_DIR/altitude.png" "$PROJECT_DIR/results/plots/altitude.png"

python3 - "$RUN_DIR/metrics.json" <<'PY'
import json
import sys

with open(sys.argv[1], "r", encoding="utf-8") as metrics_file:
    metrics = json.load(metrics_file)

overall_pass = metrics.get("evaluation", {}).get("overall_pass", False)
print(f"Overall pass: {overall_pass}")
sys.exit(0 if overall_pass else 1)
PY

echo "Tamamlandı."
