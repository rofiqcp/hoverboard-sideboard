#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LIVOX_ROOT="${LIVOX_ROOT:-/media/sirobo/HDD/livox-mid360s/livox_ros_driver2-official}"
FAST_LIO_ROOT="${FAST_LIO_ROOT:-/media/sirobo/HDD/slam3d/FAST_LIO_SAM_ROS2}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
SYSTEM_PCL_DIR="${SYSTEM_PCL_DIR:-/usr/lib/x86_64-linux-gnu/cmake/pcl}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS2 Humble setup not found: $ROS_SETUP" >&2
  exit 2
fi
source "$ROS_SETUP"

if [[ ! -f "$LIVOX_ROOT/install/setup.bash" ]]; then
  echo "ERROR: livox_ros_driver2 is not built under $LIVOX_ROOT/install" >&2
  echo "Build the existing driver first, then rerun this script." >&2
  exit 3
fi
source "$LIVOX_ROOT/install/setup.bash"

# Guard against the package-name mismatch found in this ROS2 fork.
if grep -q "package='fast_lio'" "$FAST_LIO_ROOT/launch/mapping.launch.py" ||
   grep -q "get_package_share_directory('fast_lio')" "$FAST_LIO_ROOT/launch/mapping.launch.py"; then
  echo "ERROR: FAST_LIO_SAM_ROS2 launch still references package fast_lio." >&2
  echo "Expected package name: fast_lio_sam." >&2
  exit 4
fi

echo "[1/2] Building fast_lio_sam with system ROS Humble PCL..."
(
  cd "$FAST_LIO_ROOT"
  colcon build --symlink-install --packages-select fast_lio_sam     --cmake-args "-DPCL_DIR=$SYSTEM_PCL_DIR"
)
source "$FAST_LIO_ROOT/install/setup.bash"

echo "[2/2] Building sideboard_p2_fusion..."
(
  cd "$ROOT/ros2"
  colcon build --symlink-install --packages-select sideboard_p2_fusion
)

echo
echo "P2 stack build complete."
echo "Run:"
echo "  $ROOT/tools/p2/run_mid360_p2.sh"
