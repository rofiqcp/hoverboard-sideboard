#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2/install/setup.bash"

exec ros2 launch sideboard_p2_fusion p2_fusion.launch.py "$@"
