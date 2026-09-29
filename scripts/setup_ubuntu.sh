#!/usr/bin/env bash
# Ubuntu 20.04 (native or WSL2) with ROS 1 Noetic: install this project's dependencies once.
# Same packages as docker/Dockerfile, so results match the Mac container.
#   bash scripts/setup_ubuntu.sh
set -euo pipefail

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "ROS 1 Noetic not found in /opt/ros/noetic. Install ros-noetic-desktop first:" >&2
    echo "  https://wiki.ros.org/noetic/Installation/Ubuntu" >&2
    echo "  (the ROS apt signing key changed in 2025; set up the apt source with the" >&2
    echo "   ros-apt-source package as described at https://docs.ros.org/)" >&2
    exit 1
fi
. /etc/os-release
if [ "${VERSION_CODENAME:-}" != focal ]; then
    echo "Warning: expected Ubuntu 20.04 (focal), found ${PRETTY_NAME:-unknown}." >&2
fi

# Keep this package list in sync with docker/Dockerfile.
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    ros-noetic-move-base \
    ros-noetic-navfn \
    ros-noetic-dwa-local-planner \
    ros-noetic-slam-toolbox \
    ros-noetic-cv-bridge \
    ros-noetic-tf2-ros \
    ros-noetic-rviz \
    python3-opencv \
    python3-scipy \
    python3-pytest \
    python3-pip

# Python 3.8: onnxruntime <1.20 still ships Python 3.8 wheels (1.16.3 is the newest on arm64).
# It needs a newer NumPy than apt's 1.17, so upgrade explicitly; stay below 1.24, which removed
# aliases apt's SciPy 1.3 still uses. --user keeps this out of /usr.
pip3 install --user --upgrade "onnxruntime<1.20" "numpy>=1.21.6,<1.24"

set +u  # ROS setup scripts read unset variables
source /opt/ros/noetic/setup.bash
set -u
python3 -c 'import numpy, scipy.optimize, cv_bridge, onnxruntime
assert numpy.__version__.startswith("1."), "NumPy %s found; need 1.x" % numpy.__version__
print("Dependencies OK: numpy", numpy.__version__, "| onnxruntime", onnxruntime.__version__)'
echo "Next: bash scripts/run_ros.sh"
