#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
PIO="${PIO:-/home/sirobo/.platformio/penv/bin/pio}"

echo "== STM32 firmware builds =="
"$PIO" run -e APP_STLINK
"$PIO" run -e APP_USART
"$PIO" run -e BOOTLOADER_STLINK
"$PIO" run -e APP_STLINK_1KHZ
"$PIO" run -e APP_USART_1KHZ

python3 - <<'PY_FLASH'
from pathlib import Path
root=Path(".pio/build")
limit=56320  # 55KiB app partition (excludes bootloader/settings)
for target in ("APP_STLINK","APP_USART","APP_STLINK_1KHZ","APP_USART_1KHZ"):
    path=root/target/"firmware.bin"
    size=path.stat().st_size
    if size>limit:
        raise SystemExit(f"FAIL: {target} exceeds app partition: {size}>{limit}")
    print(f"PASS flash {target}: {size}/{limit} bytes, remaining={limit-size}")
PY_FLASH

echo "== C estimator/IMU regressions =="
tools/imu/run_host_math_tests.sh
tools/imu/run_agent1_tests.sh
tools/imu/run_agent2_tests.sh

echo "== Python syntax/regressions =="
python3 -m py_compile \
  tools/imu/read_imu.py \
  tools/imu/serial_common.py \
  tools/imu/imu_log.py \
  tools/imu/imu_gui.py \
  tools/imu/validate_nav_stream.py \
  tools/imu/analyze_nav_log.py

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tests/test_agent3_integration.py \
  tests/test_autosave_csv.py \
  tests/test_tools_serial_robustness.py \
  tests/test_p0_failclosed.py \
  tests/test_p0_source_contract.py \
  tests/test_p1_sideboard_identity_layout.py \
  tests/test_p2_stream_continuity.py \
  tests/test_bootloader_hotplug.py \
  tests/test_bootloader_virtual_uart.py \
  tests/test_hotplug_parser_recovery.py

echo "== Serial identity =="
python3 tools/imu/serial_common.py

if [[ "${1:-}" == "--live" ]]; then
  duration="${2:-60}"
  echo "== LIVE 500Hz stationary hardware gate: ${duration}s =="
  python3 tools/imu/validate_nav_stream.py --duration "$duration"
elif [[ "${1:-}" == "--live-1khz" ]]; then
  duration="${2:-60}"
  echo "== LIVE 1kHz stationary hardware gate: ${duration}s =="
  python3 tools/imu/validate_nav_stream.py --duration "$duration"     --min-stream-hz 95 --min-sensor-hz 950 --max-sensor-hz 1050     --max-host-interval-p95-ms 15 --max-lag-p95-us 5000
else
  echo "LIVE hardware gate skipped; sideboard not connected. Choose:"
  echo "  $0 --live 60         # board flashed with 500Hz baseline"
  echo "  $0 --live-1khz 60    # board flashed with 1kHz bench firmware"
fi

echo "== STM32 + IMU + Python software gate PASS =="
