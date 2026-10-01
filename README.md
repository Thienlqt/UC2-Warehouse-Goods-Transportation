# UC2: Warehouse Goods Transportation

A TurtleBot3 Waffle drives through a simulated warehouse in **Unity** while **ROS 1 Noetic**
(Ubuntu 20.04) maps it with SLAM, plans and follows routes with move_base while avoiding
obstacles, and detects objects with the robot's camera and 2D lidar (boxes, labels, confidence).
Tracked obstacles are predicted 2 s ahead, so the robot yields to or steers around moving ones.
Driving by hand (P, hold W), ROS keeps the heading and steers around obstacles within 10 m.

> ROS 1 Noetic reached end of life in May 2025 and Ubuntu 20.04's standard support ended in
> April 2025: neither gets fixes or security updates any more. Keep the ROS machine (WSL or the
> Docker container) off untrusted networks.

```text
 Unity 2021.3 (Windows / macOS)                        ROS 1 Noetic, Ubuntu 20.04 (WSL2 / Docker)
 ┌───────────────────────────────┐   TCP :10000   ┌──────────────────────────────────────────┐
 │ SimpleWarehouseScene          │ ─────────────► │ ros_tcp_endpoint                         │
 │  TurtleBot3: lidar /scan,     │  /scan /tf     │  slam_toolbox ──► /map                   │
 │  camera /camera/.../compressed│  /clock camera │  move_base (Navfn, DWA) + cmd_vel_guard  │
 │  AGVController ◄── /cmd_vel   │ ◄───────────── │  perception: YOLOv8n + lidar boxes       │
 │  Detection overlay ◄── boxes  │  /cmd_vel      │   + fusion ──► /obstacles ──► avoidance  │
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
| `catkin_ws/src/unity_slam_example/` | ROS 1 package: launch file, move_base / SLAM / perception config, RViz config, `cmd_vel_guard`, perception nodes, tests, detector model (`models/`, AGPL-3.0) |
| `catkin_ws/src/uc2_vision_msgs/` | Detection messages: the `vision_msgs` 4.x layout (string class ids), which Noetic's `vision_msgs` lacks (Apache-2.0) |
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
- The detector model is small enough (~12 MB) to live in git. After retraining, commit
  `models/detector.onnx` and `models/classes.txt` together.

## Credits and license

Derived from Unity Technologies'
[Robotics-Nav2-SLAM-Example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example)
(Apache-2.0, Copyright 2021 Unity Technologies) and includes
[ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) (Apache-2.0).
Licensed under Apache-2.0 ([LICENSE.md](LICENSE.md)); components under other terms, including
the detector model, are listed in [Third Party Notices.md](Third%20Party%20Notices.md).

Changes from the upstream project: ROS 1 Noetic / Ubuntu 20.04 port (catkin workspace, move_base,
`cmd_vel_guard`), Unity 2021.3 upgrade, obstacle avoidance tuning, camera + lidar perception,
detector training, and cross-platform setup. The upstream Unity scripts were modified in place;
the original tutorials (Unity visualizations, custom visualizers) remain in the upstream
repository.

To cite the upstream project in a report:

> Unity Technologies. *Robotics-Nav2-SLAM-Example* [Computer software], 2021. Apache-2.0.
> https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example
