#!/usr/bin/env bash
# Reproducible offline gate; NEVER flashes a board or claims live accuracy.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
echo "== Firmware, IMU, ESKF, serial, Python offline gate =="
tools/imu/run_stm32_python_gate.sh
echo "== P2 ROS2 + LIO/serial integration regression =="
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ERROR: ROS Humble is required for the P2 Python tests" >&2
    exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH="$ROOT/ros2/sideboard_p2_fusion:${PYTHONPATH:-}"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q     ros2/sideboard_p2_fusion/test tests/test_gui_smoke.py
echo "== Source whitespace integrity =="
git diff --check
echo "PASS: ALL OFFLINE ROBUSTNESS GATES (HARDWARE NOT VERIFIED)"
