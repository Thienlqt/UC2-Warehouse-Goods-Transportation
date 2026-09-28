# Ubuntu 24.04 + ROS 2 Jazzy desktop with noVNC (MATE on display :1, noVNC on port 80).
# Multi-arch base (amd64/arm64): builds natively on Apple Silicon, no emulation.
# Dependencies only: start-navigation.sh copies ros2_ws/src in and builds it at start-up,
# so code changes never need an image rebuild. Build once with:
#   docker build -t unity-robotics:jazzy docker
FROM tiryoh/ros2-desktop-vnc:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# Keep this package list in sync with scripts/setup_ubuntu.sh (native Ubuntu / WSL).
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-vision-msgs \
        ros-jazzy-cv-bridge \
        python3-opencv \
        python3-scipy \
        python3-pytest \
        python3-pip \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
# Ubuntu 24.04 marks the system Python as externally managed (PEP 668). The base image ships
# NumPy 2 in /usr/local, but SciPy and cv_bridge from apt are built against NumPy 1.x.
RUN pip3 install --break-system-packages --no-cache-dir onnxruntime "numpy<2" && \
    bash -c "source /opt/ros/jazzy/setup.bash && python3 -c \
        'import numpy, scipy.optimize, cv_bridge, onnxruntime; assert numpy.__version__.startswith(\"1.\")'"

# The base image already has the `ubuntu` user (uid 1000); its entrypoint sets up the desktop.
RUN echo '[ -f ~/colcon_ws/install/local_setup.bash ] && source ~/colcon_ws/install/local_setup.bash' \
        >> /home/ubuntu/.bashrc

ENV TURTLEBOT3_MODEL=waffle
ENV USER=ubuntu
