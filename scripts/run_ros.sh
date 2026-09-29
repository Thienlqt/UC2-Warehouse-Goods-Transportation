#!/usr/bin/env bash
# Ubuntu 20.04 (native or WSL2) with ROS 1 Noetic: build catkin_ws and launch move_base + SLAM +
# perception + RViz. Run scripts/setup_ubuntu.sh once first.
#   bash scripts/run_ros.sh                    # build, then launch
#   bash scripts/run_ros.sh rviz:=false        # extra arguments go to roslaunch
#   bash scripts/run_ros.sh --build-only
# Build output goes to $UC2_WS (default ~/uc2_ws), on the Linux filesystem even when the
# repository itself is on the Windows drive (/mnt/c/...) for Unity. Its src links back to
# catkin_ws/src here, so edits in the repository are picked up without copying.
set -euo pipefail

repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ws="${UC2_WS:-$HOME/uc2_ws}"

set +u  # ROS setup scripts read unset variables
source /opt/ros/noetic/setup.bash
set -u
mkdir -p "$ws"
cd "$ws"
if [ -e src ] && [ ! -L src ]; then
    echo "$ws/src exists and is not a link to this repository; move it away or set UC2_WS." >&2
    exit 1
fi
ln -sfn "$repo/catkin_ws/src" src
catkin_make
[ "${1:-}" = --build-only ] && exit 0

set +u
source "$ws/devel/setup.bash"
set -u
echo "Starting ROS. In Unity: ROS1 protocol, 127.0.0.1:10000; open SimpleWarehouseScene, press Play."
echo "If you stop Play in Unity, stop this (Ctrl+C) and run it again before pressing Play."
exec roslaunch unity_slam_example unity_slam_example.launch "$@"
