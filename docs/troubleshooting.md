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
  `rostopic echo /cmd_vel_nav` (move_base, after `reverse_limit`) with `/cmd_vel` (after the guard): near obstacles the
  guard slows the command on purpose.
- **Robot controller mode.** On `TurtleBot3ManualConfig`, the AGV Controller must be in ROS mode,
  and first-person driving (**P**) must be off or in assisted mode: in direct mode (**O**),
  `/cmd_vel` is ignored. In assisted mode, holding a drive key cancels the goal.
- Goals must be in known free space on the map, at least 0.25 m (the robot's radius) from
  obstacles: move_base logs "Failed to get a plan" and does not move for a goal it cannot reach,
  rather than heading somewhere nearby. Start with a route of a few metres.

### Inspect commands and wheel motion in Unity

While in Play mode, select `turtlebot3_manual_config` in the Hierarchy. In the Inspector,
find **AGV Controller → Navigation diagnostics**. These values are read-only and refresh
while the simulation runs. Send a nearby **2D Nav Goal**, then click **Copy diagnostics**
to capture the current readings for troubleshooting.

- **Received ROS commands / Last command age** show whether movement commands reach the
  Unity controller. A connection or ROS subscriber alone does not prove command delivery.
- **Last forward / turn command** preserve the most recent received command, including
  after it times out. Forward speed is in metres/s; turning speed is in radians/s.
- **Requested wheel speeds** are the controller's calculated wheel speeds after speed limits
  and timeout handling. **Motor target speeds** read back the targets from Unity's wheel drives.
- **Measured wheel speeds** read the actual wheel joint velocities, converted to degrees/s
  to match the targets. X is Wheel 1 and Y is Wheel 2. `NaN` means the joint is unavailable
  or is not a single-axis revolute joint. **Sleeping** shows each wheel's physics sleep state.

| Observation during an active goal | Next area to inspect |
|---|---|
| No received commands, although ROS `/cmd_vel` contains commands | Unity receiver and TCP topic delivery |
| Last command timed out | Command publication rate and connection continuity |
| Fresh nonzero commands but zero requested wheel speeds | Controller mode and speed limits |
| Requested speeds differ from motor targets | Another component modifying the wheel drives |
| Nonzero motor targets but near-zero measured wheel speeds | Wheel joint constraints, sleeping, motor settings, or collisions |
| Wheels turn but the robot stays in place | Ground contact, wheel traction, or body collisions |

With separate Windows and WSL clones, the diagnostics script must be present in the Windows
Unity clone. Restart the ROS side before the next Play session after Unity recompiles.

## The robot turns on the spot near obstacles

- **Old move_base settings.** Before the October 2026 tuning, the robot often stopped 1–3 m from a
  shelf and turned, sometimes all the way round, before driving round it
  ([why](obstacle_avoidance.md#why-the-robot-used-to-turn-on-the-spot-near-shelves)). Pull, then
  restart the ROS side: both run scripts rebuild the workspace.
- **A turn at the goal is normal.** Once in position, the robot turns to the goal arrow's
  heading (`latch_xy_goal_tolerance`).
- **The passage is too narrow.** With less than about 0.15 m either side of the 0.50 m robot,
  DWA hesitates in front of the gap, and the global path may switch to another route. RViz's
  **Global Path** shows where it is trying to go.
- **It hit something first.** Contact with an obstacle lower than the lidar plane (0.13 m), such
  as a pallet edge, is invisible to move_base and the guard; the robot keeps pushing, then
  move_base's recovery clears and replans. Keep such objects out of the robot's routes.
- **A moving box is in the way.** **Predicted Obstacles** shows where tracked boxes will be in
  the next 2 s; the robot waits for that band to clear.

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

- **Hundreds of `Trying to read value of type Dimension while reading a value of type
  CommaSeparator` errors.** Perception 0.8.0-preview.3's `FloatRangeElement.uxml` has a
  comma-separated `margin` value; Unity 2021.3 expects spaces. Run
  `python3 scripts/fix_unity_perception_ui.py` from the repository root, then select another
  object and reselect the robot, or reopen Unity to rebuild the Inspector. With separate
  clones, point the script at the Windows project, for example:
  `python3 scripts/fix_unity_perception_ui.py /mnt/d/Project/UC2-Warehouse-Goods-Transportation/UnityProject`.
  The script changes only that margin in the generated package cache. Reapply it if you
  delete `Library/` or reinstall the package; do not commit `Library/`.
- **Errors after changing scripts during Play** (`No registered publisher`, null references
  in the sensor publishers, or `the GUIStateObj is deleted`). Stop Play before editing
  scripts or pulling changes. If a reload already broke the running session, stop Play,
  stop ROS, and reopen Unity. Allow compilation to finish, then start ROS and press Play.
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
