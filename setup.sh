#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

echo "PX4 simulation dependencies kontrol ediliyor..."
if [[ ! -d "$PX4_DIR" ]]; then
    echo "PX4-Autopilot dizini bulunamadı: $PX4_DIR"
    echo "Farklı bir konum kullanıyorsan şöyle çalıştır:"
    echo "PX4_DIR=/path/to/PX4-Autopilot bash setup.sh"
    exit 1
fi

if [[ ! -f "$PX4_DIR/Tools/setup/ubuntu.sh" ]]; then
    echo "PX4 setup script bulunamadı: $PX4_DIR/Tools/setup/ubuntu.sh"
    exit 1
fi

echo "PX4 Ubuntu setup çalıştırılıyor (--no-nuttx)..."
bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx

echo "Python dependencies kuruluyor..."
python3 -m pip install -r "$PROJECT_DIR/requirements.txt"

echo "Klasörler oluşturuluyor..."
mkdir -p "$PROJECT_DIR/logs" "$PROJECT_DIR/results" "$PROJECT_DIR/results/plots" "$PROJECT_DIR/data/sim" "$PROJECT_DIR/data/real"

echo "Setup tamamlandı."
