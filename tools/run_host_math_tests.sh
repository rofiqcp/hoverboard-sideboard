#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
out="${TMPDIR:-/tmp}/hoverboard_sideboard_stage2_test"
gcc -std=c11 -O2 -IInc tests/test_stage2_math.c Src/eskf_nav.c Src/imu_calibration.c \
  -lm -DM_PI=3.14159265358979323846 -o "$out"
"$out"
