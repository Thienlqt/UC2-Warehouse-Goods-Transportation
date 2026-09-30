# Troubleshooting

"ROS shell" below means: macOS `docker exec -it -u ubuntu -e HOME=/home/ubuntu unity-nav2 bash`
(its `.bashrc` sources the workspace); Ubuntu / WSL a new terminal with
`source /opt/ros/noetic/setup.bash && source ~/uc2_ws/devel/setup.bash`.

## The robot ignores goals

- **Play was restarted while ROS kept running.** Unity's `/clock` restarts at 0, but SLAM keeps
  publishing `map -> odom` from the old session. The planner logs "extrapolation into the past".
  Restart the ROS side (`docker stop unity-nav2` + start script, or Ctrl+C + `run_ros.sh`), then Play.
- **move_base is still waiting for TF.** Until slam_toolbox publishes `map -> odom` (shortly after
  Play), move_base logs `Timed out waiting for transform from base_link to map` and accepts no
  goals. It keeps waiting, however long Unity takes.
- **cmd_vel_guard is holding the robot.** `No fresh /scan in 1.0 s: holding the robot still` means
  the lidar stopped; the guard sends zero velocity until `/scan` is back. Compare
  `rostopic echo /cmd_vel_nav` (move_base) with `/cmd_vel` (after the guard): near obstacles the
  guard slows the command on purpose.
- **Robot controller mode.** On `TurtleBot3ManualConfig`, the AGV Controller must be in ROS mode,
  and first-person driving (**P**) must be off: while driving, `/cmd_vel` is ignored.
- Goals must be in known free space on the map. Start with a route of a few metres.

## Unity does not connect (red arrows in the Game view)

- The ROS side must be running first and log `Starting server on 0.0.0.0:10000`.
- **Robotics → ROS Settings**: ROS1, `127.0.0.1`, port 10000. If it shows ROS2, the project
  settings were not reloaded after a pull: reopen the project.
- macOS: port 10000 must be free (`lsof -i :10000`) and the container running (`docker ps`).
- WSL: see "If something is wrong" in [setup_windows_wsl.md](setup_windows_wsl.md) (WSL IP or
  mirrored networking, firewall).

## Topics look missing ("Unable to communicate with master", 0 messages)

- `Unable to communicate with master`: the ROS side is not running, or the shell did not source
  the setup files (see "ROS shell" above).
- `/scan` exists as soon as SLAM subscribes, even before Unity publishes. Use
  `rostopic info /scan` to check for a publisher, then `rostopic hz /scan`.

## Camera detection shows no boxes

- The camera detector log line `Loaded .../models/detector.onnx (416 px, 3 classes)` confirms the
  model. `No model at ...` means `catkin_ws/src/unity_slam_example/models/` is missing files.
- The model knows `box`, `shelf` and `station` only. It has no `person` class yet.
- In Unity, **C** toggles the detection overlay. `waiting for /detections_2d` means no messages
  have arrived: check the ROS side.

## Python / dependency errors on Ubuntu or WSL

- `module compiled against API version 0xe but this version of numpy is 0xd` when importing
  onnxruntime: apt's NumPy 1.17 is still the one in use. Rerun `bash scripts/setup_ubuntu.sh`,
  which upgrades NumPy for your user.
- `module 'numpy' has no attribute 'bool'` from SciPy (tracking, fusion): NumPy 1.24 or newer got
  installed. apt's SciPy 1.3 needs NumPy below 1.24; the setup script pins `>=1.21.6,<1.24`.
- `package 'ros_tcp_endpoint' not found` or `uc2_vision_msgs` import errors: build with
  `bash scripts/run_ros.sh`, which builds all packages in `catkin_ws/src`.
- `/usr/bin/env: 'python': No such file or directory` from a vendored script: Ubuntu 20.04 has
  only `python3`. This repository's copy of the endpoint already uses `python3`.
- Line-ending errors such as `$'\r': command not found`: the clone was converted to CRLF. Run
  `git config --global core.autocrlf false` in Windows, delete the clone and clone again.

## Unity

- **First open takes long / package errors.** The first import takes 10–20 minutes. Package
  errors about git URLs mean `git` is not on the `PATH` (Windows: install Git for Windows, restart
  Unity Hub).
- **Wrong editor version.** Use exactly 2021.3.45f2. Opening in another version can modify many
  files; don't commit those.
- **Never commit** `UnityProject/Library`, `Temp`, `Logs` or `UserSettings` (already git-ignored).
- **Scene merge conflicts.** Scenes and prefabs are text (YAML), but conflicts are hard to merge.
  Agree who edits `SimpleWarehouseScene` at a time, or work in your own copy of the scene.

## macOS / Docker

- `Container unity-nav2 already exists`: `docker stop unity-nav2`, then start again.
- Colima slow or crashing: use `--vm-type vz`, at least 6 CPUs and 8–10 GiB memory
  (see [setup_macos.md](setup_macos.md)).
