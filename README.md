# UC2: Warehouse Goods Transportation (ROS 1 Noetic branch)

A TurtleBot3 Waffle drives through a simulated warehouse in **Unity** while **ROS 1 Noetic**
(Ubuntu 20.04) maps it with SLAM, plans and follows routes with move_base while avoiding
obstacles, and detects objects with the robot's camera and 2D lidar (boxes, labels, confidence).

> **This is the `ros1-noetic` branch.** The `main` branch runs the same project on ROS 2 Jazzy
> (Ubuntu 24.04) with Nav2. ROS 1 Noetic reached end of life in May 2025 and Ubuntu 20.04's
> standard support ended in April 2025: neither gets fixes or security updates any more. Use this
> branch where ROS 1 is required (existing ROS 1 robots or code, coursework); prefer `main`
> otherwise.

```text
 Unity 2021.3 (Windows / macOS)                        ROS 1 Noetic, Ubuntu 20.04 (WSL2 / Docker)
 ┌───────────────────────────────┐   TCP :10000   ┌──────────────────────────────────────────┐
 │ SimpleWarehouseScene          │ ─────────────► │ ros_tcp_endpoint                         │
 │  TurtleBot3: lidar /scan,     │  /scan /tf     │  slam_toolbox ──► /map                   │
 │  camera /camera/.../compressed│  /clock camera │  move_base (Navfn, DWA) + cmd_vel_guard  │
 │  AGVController ◄── /cmd_vel   │ ◄───────────── │  perception: YOLOv8n + lidar boxes       │
 │  Detection overlay ◄── boxes  │  /cmd_vel      │   + fusion ──► /obstacles                │
 └───────────────────────────────┘  /detections_2d│  RViz (WSLg window / noVNC in browser)   │
                                                  └──────────────────────────────────────────┘
```

## Quick start

| Your machine | Guide | Every run |
|---|---|---|
| Windows + WSL2 Ubuntu 20.04 (ROS 1 Noetic) | [docs/setup_windows_wsl.md](docs/setup_windows_wsl.md) | `bash scripts/run_ros.sh` in WSL, then Play in Unity |
| macOS (Apple Silicon) | [docs/setup_macos.md](docs/setup_macos.md) | `bash docker/start-navigation.sh`, then Play in Unity |

Native Ubuntu 20.04 uses the same scripts as WSL ([docs/setup_windows_wsl.md](docs/setup_windows_wsl.md),
skip the Windows parts) but is not a tested setup.

Everyone needs **Unity 2021.3.45f2** exactly. After stopping Play, restart the ROS side before
pressing Play again ([why](docs/troubleshooting.md#the-robot-ignores-goals)).

## Repository layout

| Path | What |
|---|---|
| `UnityProject/` | Unity project: warehouse scene, robot, lidar and camera sensors, detection overlay, first-person driving (**P**), dataset capture tool |
| `catkin_ws/src/unity_slam_example/` | ROS 1 package: launch file, move_base / SLAM / perception config, RViz config, `cmd_vel_guard`, perception nodes, tests, detector model (`models/`) |
| `catkin_ws/src/uc2_vision_msgs/` | Detection messages in the ROS 2 `vision_msgs` 4.x layout, built for Noetic (Apache-2.0) |
| `catkin_ws/src/ROS-TCP-Endpoint/` | Unity's ROS-TCP-Endpoint v0.6.0, ROS 1 release (vendored, Apache-2.0; `python3` shebang) |
| `scripts/` | Ubuntu / WSL: `setup_ubuntu.sh` (dependencies, once), `run_ros.sh` (build + launch) |
| `docker/` | macOS: `Dockerfile` (dependencies only) and `start-navigation.sh` |
| `detection_training/` | Retrain the camera detector on frames captured in Unity |
| `docs/` | Setup guides, [obstacle avoidance](docs/obstacle_avoidance.md), [obstacle detection](docs/obstacle_detection.md), [troubleshooting](docs/troubleshooting.md) |

## Working together

- Branch from `main`, open a pull request, and get a review before merging.
- Unity: never commit `Library/`, `Temp/`, `Logs/` or `UserSettings/` (git-ignored). Always commit a
  new asset together with its `.meta` file. Coordinate edits to `SimpleWarehouseScene`: scene
  merges are painful.
- ROS: change parameters in `catkin_ws/src/unity_slam_example/config/`, then restart the ROS side;
  both run scripts rebuild the workspace.
- Fixes that apply to both stacks (Unity scene, perception algorithms, training) go to `main`
  first, then get cherry-picked here, or the other way round.
- The detector model is small enough (~12 MB) to live in git. After retraining, commit
  `models/detector.onnx` and `models/classes.txt` together.

## Credits and license

Derived from Unity Technologies'
[Robotics-Nav2-SLAM-Example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example)
(Apache-2.0) and includes [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint).
This repository modifies it: ROS 2 Jazzy / Ubuntu 24.04 port (`main`), this ROS 1 Noetic / Ubuntu
20.04 port, Unity 2021.3 upgrade, obstacle avoidance tuning, camera + lidar perception, detector
training, and cross-platform setup. See
[LICENSE.md](LICENSE.md) and [Third Party Notices.md](Third%20Party%20Notices.md). The original
tutorials (Unity visualizations, custom visualizers) remain in the upstream repository.
