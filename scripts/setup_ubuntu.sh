#!/usr/bin/env bash
# Ubuntu 24.04 (native or WSL2) with ROS 2 Jazzy: install this project's dependencies once.
# Same packages as docker/Dockerfile, so results match the Mac container.
#   bash scripts/setup_ubuntu.sh
set -euo pipefail

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "ROS 2 Jazzy not found in /opt/ros/jazzy. Install ros-jazzy-desktop first:" >&2
    echo "  https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html" >&2
    exit 1
fi
. /etc/os-release
if [ "${VERSION_CODENAME:-}" != noble ]; then
    echo "Warning: expected Ubuntu 24.04 (noble), found ${PRETTY_NAME:-unknown}." >&2
fi

# Keep this package list in sync with docker/Dockerfile.
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    python3-opencv \
    python3-scipy \
    python3-pytest \
    python3-pip \
    python3-colcon-common-extensions

# Ubuntu 24.04's system Python is externally managed (PEP 668), so pip needs
# --break-system-packages. Pin NumPy 1.x: apt's SciPy and cv_bridge are built against it,
# and onnxruntime would otherwise pull in NumPy 2. --user keeps this out of /usr.
pip3 install --user --break-system-packages onnxruntime "numpy<2"

set +u  # ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash
set -u
python3 -c 'import numpy, scipy.optimize, cv_bridge, onnxruntime
assert numpy.__version__.startswith("1."), "NumPy %s found; need 1.x" % numpy.__version__
print("Dependencies OK: numpy", numpy.__version__, "| onnxruntime", onnxruntime.__version__)'
echo "Next: bash scripts/run_ros.sh"
