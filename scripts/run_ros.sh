#!/usr/bin/env bash
# Ubuntu 24.04 (native or WSL2) with ROS 2 Jazzy: build ros2_ws and launch Nav2 + SLAM +
# perception + RViz. Run scripts/setup_ubuntu.sh once first.
#   bash scripts/run_ros.sh                    # build, then launch
#   bash scripts/run_ros.sh rviz:=false        # extra arguments go to ros2 launch
#   bash scripts/run_ros.sh --build-only
# Build output goes to $UC2_WS (default ~/uc2_ws), on the Linux filesystem even when the
# repository itself is on the Windows drive (/mnt/c/...) for Unity.
set -euo pipefail

repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ws="${UC2_WS:-$HOME/uc2_ws}"

set +u  # ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash
set -u
mkdir -p "$ws"
cd "$ws"
colcon build --base-paths "$repo/ros2_ws/src"
[ "${1:-}" = --build-only ] && exit 0

set +u
source "$ws/install/setup.bash"
set -u
echo "Starting ROS. In Unity: ROS2 protocol, 127.0.0.1:10000; open SimpleWarehouseScene, press Play."
echo "If you stop Play in Unity, stop this (Ctrl+C) and run it again before pressing Play."
exec ros2 launch unity_slam_example unity_slam_example.py "$@"
