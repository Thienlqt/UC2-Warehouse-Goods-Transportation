# Setup: macOS (Apple Silicon)

Unity runs on the **Mac**. ROS 2 Jazzy (Ubuntu 24.04) runs in a **Docker container**, native
arm64 with no emulation. RViz is shown in the browser through noVNC.

```text
macOS: Unity --TCP 127.0.0.1:10000--> container unity-nav2: ros_tcp_endpoint -> Nav2 / SLAM / perception / RViz
                                      browser http://localhost:6080 (noVNC desktop with RViz)
```

## 1. Install (once)

1. **Unity Hub**, then the editor version **2021.3.45f2** exactly, Apple silicon build
   (Hub → Installs → Install Editor → Archive, or <https://unity.com/releases/editor/archive>).
2. **Docker**: either Docker Desktop, or Colima (free):
   ```bash
   brew install colima docker
   colima start --vm-type vz --cpu 6 --memory 10 --disk 100
   ```
   Give the VM at least 6 CPUs and 8–10 GiB of memory: Nav2, SLAM, the detector and RViz run
   together. Colima keeps an instance copy of its config at `~/.colima/_lima/colima/colima.yaml`,
   which wins over `~/.colima/default/colima.yaml`; edit that copy if a resize seems ignored.
3. **Clone and build the image** (about 10 minutes, once; code changes never need a rebuild):
   ```bash
   git clone https://github.com/<org>/UC2-Warehouse-Goods-Transportation.git
   cd UC2-Warehouse-Goods-Transportation
   docker build -t unity-robotics:jazzy docker
   ```

## 2. Open the Unity project (once)

Unity Hub → **Add** → **Add project from disk** → the repository's `UnityProject` folder.
The first open imports everything and downloads the git packages: allow **10–20 minutes**.
Then open `Assets/Scenes/SimpleWarehouseScene`. The Console should show no red errors.

Check **Robotics → ROS Settings**: Protocol **ROS2**, ROS IP Address **127.0.0.1**, port **10000**.

## 3. Run (every time)

1. From the repository root:
   ```bash
   bash docker/start-navigation.sh
   ```
   It starts the `unity-nav2` container, copies and builds `ros2_ws/src`, and launches Nav2,
   SLAM, the perception nodes and RViz.
2. Open <http://localhost:6080/vnc.html?password=ubuntu&autoconnect=true> (password `ubuntu`).
3. In Unity, press **Play**. RViz shows the map growing, the laser scan and the camera detections.
4. In RViz, use **2D Goal Pose** to send the robot somewhere in known free space.
5. **After stopping Play, restart the ROS side before the next Play:**
   `docker stop unity-nav2`, then `bash docker/start-navigation.sh` again.

Startup log: `docker exec unity-nav2 tail -n 40 /tmp/unity-navigation.log`. It should end with
`lifecycle_manager_unity: Managed nodes are active` after you press Play.

ROS shell in the container (always as user `ubuntu`, or topics look missing):

```bash
docker exec -it -u ubuntu -e HOME=/home/ubuntu unity-nav2 bash
source /opt/ros/jazzy/setup.bash && source ~/colcon_ws/install/setup.bash
```

Tests: see "Automated ROS checks" in [obstacle_avoidance.md](obstacle_avoidance.md). More help in
[troubleshooting.md](troubleshooting.md).
