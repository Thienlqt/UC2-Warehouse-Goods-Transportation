#!/usr/bin/env bash
# macOS (or any Docker host): start the ROS 1 Noetic stack in a container, with RViz on noVNC.
# Build the image once:  docker build -t unity-robotics:noetic docker
# Then:                  bash docker/start-navigation.sh [container-name]
set -euo pipefail

repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
container_name="${1:-unity-nav2}"
image_name="${IMAGE:-unity-robotics:noetic}"
ws=/home/ubuntu/catkin_ws

if docker container inspect "$container_name" >/dev/null 2>&1; then
    echo "Container $container_name already exists. Use its running stack, or stop it first:" >&2
    echo "  docker stop $container_name" >&2
    exit 1
fi
docker image inspect "$image_name" >/dev/null 2>&1 || {
    echo "Image $image_name not found. Build it once: docker build -t $image_name docker" >&2
    exit 1
}
# No --platform: the image is built for this machine (arm64 on Apple Silicon, amd64 on PCs).
docker run -d --rm --name "$container_name" \
    -p 6080:80 -p 10000:10000 --shm-size=1024m \
    "$image_name"
# Copy the workspace sources (all packages, including the detector model) and build them.
docker exec -u ubuntu "$container_name" mkdir -p "$ws/src"
docker cp "$repo/catkin_ws/src/." "$container_name:$ws/src/"
docker exec -u root "$container_name" chown -R ubuntu:ubuntu "$ws"
docker exec -u ubuntu -e HOME=/home/ubuntu "$container_name" bash -c "
    set -e
    source /opt/ros/noetic/setup.bash
    cd $ws
    catkin_make
"
# RViz exits if it starts before the desktop's X server on :1 is up.
if ! docker exec "$container_name" bash -c '
    for i in $(seq 1 60); do [ -S /tmp/.X11-unix/X1 ] && exit 0; sleep 1; done; exit 1'; then
    echo "Virtual desktop (display :1) did not start within 60 s." >&2
    exit 1
fi
docker exec -d -u ubuntu -e DISPLAY=:1 -e HOME=/home/ubuntu "$container_name" bash -c "
    source /opt/ros/noetic/setup.bash
    source $ws/devel/setup.bash
    export PYTHONUNBUFFERED=1  # node logs reach the file as they happen
    exec roslaunch unity_slam_example unity_slam_example.launch > /tmp/unity-navigation.log 2>&1
"
echo "Launch requested. Desktop: http://localhost:6080/vnc.html?password=ubuntu&autoconnect=true"
echo "Open SimpleWarehouseScene in Unity, wait for script compilation, then press Play."
echo "Check startup: docker exec $container_name tail -n 40 /tmp/unity-navigation.log"
echo "Stop:          docker stop $container_name   (restart it whenever you stop Play in Unity)"
