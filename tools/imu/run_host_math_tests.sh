#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."

base="${TMPDIR:-/tmp}/hoverboard_sideboard_stage2_test"
v2="${TMPDIR:-/tmp}/hoverboard_sideboard_eskf_v2_test"

gcc -std=c11 -O2 -Wall -Wextra -IInc tests/test_stage2_math.c \
  Src/eskf_nav.c Src/imu_calibration.c -lm -DM_PI=3.14159265358979323846 -o "$base"
"$base"

gcc -std=c11 -O2 -Wall -Wextra -IInc tests/test_eskf_v2_consistency.c \
  Src/eskf_nav.c -lm -o "$v2"
"$v2"
