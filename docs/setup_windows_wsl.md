# Setup: Windows + WSL2 (Ubuntu 20.04, ROS 1 Noetic)

Unity runs on **Windows**. ROS 1 Noetic (move_base, SLAM, perception, RViz) runs natively in
**WSL2 Ubuntu 20.04**. They talk over TCP port 10000. No Docker is needed. The same scripts run
on a native Ubuntu 20.04 machine (not a tested setup): skip the Windows-only parts.

```text
Windows: Unity (SimpleWarehouseScene) --TCP 127.0.0.1:10000--> WSL2: ros_tcp_endpoint -> move_base / SLAM / perception / RViz
```

## 1. Windows side (once)

1. **Git for Windows** (<https://git-scm.com/download/win>), with "Git from the command line"
   enabled. Unity's Package Manager needs `git` on the `PATH` to fetch ROS-TCP-Connector,
   URDF-Importer and the Warehouse package. Then, in PowerShell:
   ```powershell
   git config --global core.longpaths true
   ```
2. **Unity Hub**, then the editor version **2021.3.45f2** exactly (Hub → Installs → Install Editor →
   Archive, or <https://unity.com/releases/editor/archive>). No extra modules are needed.
3. **Clone on the Windows drive**, in a short path without spaces, so Unity works at full speed:
   ```powershell
   mkdir C:\dev; cd C:\dev
   git clone https://github.com/Thienlqt/UC2-Warehouse-Goods-Transportation.git
   ```
   Do your `git` work from Windows (terminal, VS Code or GitHub Desktop). If you also use `git` in
   WSL on this clone, run `git config core.fileMode false` there once, or every file shows as changed.

## 2. WSL side (once)

1. WSL2 with Ubuntu 20.04: `wsl --install -d Ubuntu-20.04` in PowerShell. Keep WSL current
   for GUI apps (WSLg, Windows 11): `wsl --update`.
2. ROS 1 Noetic (`ros-noetic-desktop`), following the
   [Noetic install guide](https://wiki.ros.org/noetic/Installation/Ubuntu). The ROS apt signing
   key changed in 2025: if `apt-get update` reports an expired or missing key, set up the ROS apt
   source with the `ros-apt-source` package as described on <https://docs.ros.org/>.
3. Install this project's dependencies (move_base, DWA, SLAM Toolbox, OpenCV, SciPy,
   ONNX Runtime, NumPy 1.2x):
   ```bash
   cd /mnt/c/dev/UC2-Warehouse-Goods-Transportation
   bash scripts/setup_ubuntu.sh
   ```
   It ends with `Dependencies OK`.

## 3. Open the Unity project (once)

Unity Hub → **Add** → **Add project from disk** → `C:\dev\UC2-Warehouse-Goods-Transportation\UnityProject`.
The first open imports everything and downloads the git packages: allow **10–20 minutes**.
Then open `Assets/Scenes/SimpleWarehouseScene`. The Console should show no red errors.

Check **Robotics → ROS Settings**: Protocol **ROS1**, ROS IP Address **127.0.0.1**, port **10000**.

## 4. Run (every time)

1. In WSL:
   ```bash
   cd /mnt/c/dev/UC2-Warehouse-Goods-Transportation
   bash scripts/run_ros.sh
   ```
   It builds `catkin_ws` into `~/uc2_ws` (on the Linux filesystem, so builds stay fast; its `src`
   links back to the repository), then starts move_base, SLAM, the perception nodes and RViz.
   RViz opens as a normal window.
2. In Unity, press **Play**. The ROS connection arrows in the Game view turn blue, and RViz shows
   the map growing, the laser scan and the camera detections.
3. In RViz, use **2D Nav Goal** to send the robot somewhere in known free space.
4. **After stopping Play, stop `run_ros.sh` (Ctrl+C) and start it again before the next Play.**
   Unity's clock restarts at 0, and a running ROS stack would reject the new timestamps.

Launch options go straight to `roslaunch`, for example `bash scripts/run_ros.sh rviz:=false`
or `perception:=false`.

## If something is wrong

- **Unity cannot connect (red arrows).** Check that `run_ros.sh` is running and printed
  `Starting server on 0.0.0.0:10000`. WSL2 forwards `127.0.0.1` from Windows by default. If it still
  fails, use the WSL IP (`hostname -I` in WSL, first address) as the ROS IP in Unity, or enable
  mirrored networking (Windows 11): create `%UserProfile%\.wslconfig` with
  ```ini
  [wsl2]
  networkingMode=mirrored
  ```
  then run `wsl --shutdown` and reopen WSL. Allow Unity through Windows Firewall if asked.
- **RViz window is black or crashes.** Run `export LIBGL_ALWAYS_SOFTWARE=1` before `run_ros.sh`.
- More in [troubleshooting.md](troubleshooting.md).

## Tests

```bash
source ~/uc2_ws/devel/setup.bash
cd catkin_ws/src/unity_slam_example
python3 -m pytest -q test/
python3 test/check_obstacle_costmap.py   # starts move_base on its own ROS master (port 11312)
```
