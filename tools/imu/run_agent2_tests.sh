#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
out="/tmp/hoverboard_sideboard_agent2_test"
gcc -std=c11 -O2 -Wall -Wextra -Werror -IInc \
  tests/test_agent2_eskf_slip.c Src/eskf_nav.c Src/slip_observer.c \
  -lm -DM_PI=3.14159265358979323846 -o "$out"
"$out"
