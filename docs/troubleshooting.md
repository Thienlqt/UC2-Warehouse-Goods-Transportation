# Troubleshooting

"ROS shell" below means: macOS `docker exec -it -u ubuntu -e HOME=/home/ubuntu unity-nav2 bash`
then `source ~/colcon_ws/install/setup.bash`; Ubuntu / WSL a new terminal with
`source ~/uc2_ws/install/setup.bash`. Source `/opt/ros/jazzy/setup.bash` first in both.

## The robot ignores goals

- **Play was restarted while ROS kept running.** Unity's `/clock` restarts at 0, but SLAM keeps
  publishing `map -> odom` from the old session. The planner logs "extrapolation into the past".
  Restart the ROS side (`docker stop unity-nav2` + start script, or Ctrl+C + `run_ros.sh`), then Play.
- **Nav2 is not active yet.** The log should contain `lifecycle_manager_unity: Managed nodes are
  active` a few seconds after Play. It waits for Unity's TF, up to an hour after launch.
- **Robot controller mode.** On `TurtleBot3ManualConfig`, the AGV Controller must be in ROS mode,
  and first-person driving (**P**) must be off: while driving, Nav2's `cmd_vel` is ignored.
- Goals must be in known free space on the map. Start with a route of a few metres.

## Unity does not connect (red arrows in the Game view)

- The ROS side must be running first and log `Starting server on 0.0.0.0:10000`.
- **Robotics → ROS Settings**: ROS2, `127.0.0.1`, port 10000.
- macOS: port 10000 must be free (`lsof -i :10000`) and the container running (`docker ps`).
- WSL: see "If something is wrong" in [setup_windows_wsl.md](setup_windows_wsl.md) (WSL IP or
  mirrored networking, firewall).

## Topics look missing ("Unknown topic", 0 messages)

- On macOS, open the ROS shell as `-u ubuntu`. As root, Fast DDS shared memory does not reach the
  stack's nodes, so topics look empty.
- `/scan` exists as soon as SLAM subscribes, even before Unity publishes. Use
  `ros2 topic info /scan --verbose` to check for a publisher, then `ros2 topic hz /scan`.

## Camera detection shows no boxes

- The camera detector log line `Loaded .../models/detector.onnx (416 px, 3 classes)` confirms the
  model. `No model at ...` means `ros2_ws/src/unity_slam_example/models/` is missing files.
- The model knows `box`, `shelf` and `station` only. It has no `person` class yet.
- In Unity, **C** toggles the detection overlay. `waiting for /detections_2d` means no messages
  have arrived: check the ROS side.

## Python / dependency errors on Ubuntu or WSL

- `numpy.core.multiarray failed to import` or a cv_bridge / SciPy crash: NumPy 2 got installed.
  Rerun `bash scripts/setup_ubuntu.sh`, which pins `numpy<2` (apt's SciPy and cv_bridge need 1.x).
- `package 'ros_tcp_endpoint' not found`: build with `bash scripts/run_ros.sh`, which builds both
  packages in `ros2_ws/src`.
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
