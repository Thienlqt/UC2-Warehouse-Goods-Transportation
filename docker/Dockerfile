# Ubuntu 20.04 + ROS 1 Noetic desktop with noVNC (display :1, noVNC on port 80).
# Multi-arch base (amd64/arm64): builds natively on Apple Silicon, no emulation.
# Dependencies only: start-navigation.sh copies catkin_ws/src in and builds it at start-up,
# so code changes never need an image rebuild. Build once with:
#   docker build -t unity-robotics:noetic docker
FROM tiryoh/ros-desktop-vnc:noetic

ENV DEBIAN_FRONTEND=noninteractive

# Keep this package list in sync with scripts/setup_ubuntu.sh (native Ubuntu / WSL).
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
        ros-noetic-move-base \
        ros-noetic-navfn \
        ros-noetic-dwa-local-planner \
        ros-noetic-slam-toolbox \
        ros-noetic-tf2-ros \
        ros-noetic-rviz \
        python3-opencv \
        python3-scipy \
        python3-pytest \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*
# Python 3.8: onnxruntime <1.20 still ships Python 3.8 wheels (1.16.3 is the newest on arm64).
# It needs a newer NumPy than apt's 1.17; stay below 1.24, which removed aliases apt's SciPy 1.3
# still uses (np.bool in linear_sum_assignment).
RUN pip3 install --no-cache-dir --upgrade "onnxruntime<1.20" "numpy>=1.21.6,<1.24" && \
    bash -c "source /opt/ros/noetic/setup.bash && python3 -c \
        'import numpy, scipy.optimize, cv2, onnxruntime; assert numpy.__version__.startswith(\"1.\")'"

# The base image's entrypoint creates the `ubuntu` user at container start (from /etc/skel)
# and sets up the desktop, so ROS shells get their setup from the skeleton .bashrc.
RUN printf '%s\n' 'source /opt/ros/noetic/setup.bash' \
        '[ -f ~/catkin_ws/devel/setup.bash ] && source ~/catkin_ws/devel/setup.bash' \
        >> /etc/skel/.bashrc

ENV TURTLEBOT3_MODEL=waffle
ENV USER=ubuntu
