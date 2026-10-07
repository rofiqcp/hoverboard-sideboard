#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LIVOX_ROOT="${LIVOX_ROOT:-/media/sirobo/HDD/livox-mid360s/livox_ros_driver2-official}"
FAST_LIO_ROOT="${FAST_LIO_ROOT:-/media/sirobo/HDD/slam3d/FAST_LIO_SAM_ROS2}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

source "$ROS_SETUP"
source "$LIVOX_ROOT/install/setup.bash"
source "$FAST_LIO_ROOT/install/setup.bash"
source "$ROOT/ros2/install/setup.bash"

exec ros2 launch sideboard_p2_fusion mid360_p2_stack.launch.py "$@"
