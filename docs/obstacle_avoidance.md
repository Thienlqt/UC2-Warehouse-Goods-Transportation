# Obstacle detection and avoidance in Unity + Nav2

This project uses 2D laser scans for geometric obstacle detection and Nav2's
DWB controller for local avoidance. It does not classify people/boxes or predict
moving objects' future trajectories. Moving obstacles are handled reactively
as new scans update the costmap.

## Your subsystem

```text
Unity colliders -> LaserScanSensor -> /scan -> obstacle layer -> inflation layer
                                                               |
Goal + SLAM map -> global path -> DWB local controller <----------+
                                     |
                                  /cmd_vel -> Unity AGVController -> wheels

Unity /tf -> simulated /odom (pose + measured motion) -> DWB
Unity /clock -> simulation time for all ROS nodes
```

Your main deliverables are reliable scan input, obstacle marking/clearing,
appropriate robot clearance, controller tuning, and recorded test results.
The navigation goal and global map can come from other team members.

## Run

Start the ROS side (Ubuntu 24.04 + ROS 2 Jazzy) as described in the setup guide for your OS:

```bash
bash docker/start-navigation.sh   # macOS: Docker container, RViz at http://localhost:6080
bash scripts/run_ros.sh           # Ubuntu / WSL: native, RViz opens as a window
```

Nav2 and SLAM are brought up by `lifecycle_manager_unity` (see the launch file): SLAM first,
then the Nav2 servers. Bond heartbeats are off, and the costmaps wait up to an hour for TF,
so ROS can start before you press Play. Jazzy's `route_server` and `docking_server` are not
used and stay unconfigured. Commands flow controller → `cmd_vel_nav` → velocity smoother →
`cmd_vel_smoothed` → collision monitor → `/cmd_vel` (Unity). The collision monitor is a
last-resort stop if the predicted footprint would hit a scan point within 1.2 s.

1. Open `UnityProject` in Unity and open `SimpleWarehouseScene`.
2. Let Unity recompile the changed scripts, then press Play once. **If you stop Play, restart
   the ROS side before pressing Play again** (macOS: `docker stop unity-nav2`, then rerun the
   script; Ubuntu / WSL: Ctrl+C `run_ros.sh`, then rerun it).
   Unity's `/clock` restarts at 0, but SLAM keeps publishing `map -> odom` stamped in the old
   session. The planner then logs "extrapolation into the past" and the robot never moves.
3. Keep ROS settings at ROS2, `127.0.0.1:10000`, and the robot controller in ROS mode.
4. In RViz (macOS: <http://localhost:6080>), wait for the map and scan, then set a **2D Goal Pose**
   in known free space. Start with a short route, a few metres long.

On macOS, check launch failures with `docker exec unity-nav2 tail -n 60 /tmp/unity-navigation.log`
(on Ubuntu / WSL the log is in the `run_ros.sh` terminal). `docker stop unity-nav2` stops and
removes the disposable container; export any recordings first. Either script rebuilds the
workspace on every start, so rerun it to pick up ROS configuration/code changes.

## Settings you own

Edit `ros2_ws/src/unity_slam_example/config/nav2_obstacle_avoidance.yaml`.
This configuration targets the installed **ROS 2 Jazzy** packages (Nav2 1.3).

| Setting | Initial value | Purpose |
|---|---:|---|
| Scan | 360 rays, 10 Hz | One-degree sampling over 0–359 degrees |
| Local costmap | 6 × 6 m, 5 cm cells, 10 Hz | Nearby obstacles around the robot |
| Obstacle marking range | 3 m | Insert observed obstacles |
| Ray clearing range | 4 m | Clear previously observed obstacles when scans show free space |
| Inflation radius | 0.55 m | Penalize paths near obstacles; not a guaranteed clearance |
| Robot radius | 0.22 m | Inherited circular approximation; verify it covers the robot |
| Maximum forward speed | 0.2 m/s | Initial simulation test speed |
| Maximum turn rate | 0.5 rad/s | Matches the Unity controller limit |
| DWB prediction horizon | 2 s | Evaluate candidate motion against current obstacles |
| Maximum scan interval | 0.5 s | Mark the observation source stale if scans stop |

Keep `marking`, `clearing`, and `inf_is_valid` enabled. Unity now publishes positive
infinity for rays with no hit, allowing the obstacle layer to clear them. A hidden
obstacle cannot be cleared until a later ray observes free space behind its old
position. `observation_persistence: 0` is not an automatic cell-expiration timer.

The scanner reports ranges from its origin, ignores trigger volumes, and samples
the angle endpoints consistently. The wheel bridge uses the differential-drive
half-track formula and clamps positive and negative commands symmetrically.

`tf_odometry` derives planar velocity from Unity's `odom -> base_link` transforms.
It is simulation ground truth, not a real odometry estimator, and does not publish
another TF transform. If a teammate supplies `/odom`, launch with
`publish_sim_odom:=false`. Only SLAM should publish `map -> odom`; the warehouse
scene already overrides the Unity TF publisher to start at `odom`.

## Inspect detection separately from avoidance

Open a diagnostic terminal: on macOS `docker exec -it -u ubuntu -e HOME=/home/ubuntu unity-nav2 bash`;
on Ubuntu / WSL a new terminal, then `source ~/uc2_ws/install/setup.bash` instead of the
`colcon_ws` line below.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 topic info /scan --verbose --no-daemon
ros2 topic echo /odom --no-daemon
# Ctrl+C stops an echo command.
ros2 lifecycle get /controller_server
ros2 topic echo /cmd_vel --no-daemon
```

RViz has **Local Obstacles**, **Global Costmap**, **Global Path**, and **Local
Trajectory** displays. Red laser points only verify sensor input. Detection is
working when the local costmap marks the object and adds inflated costs around
it. Avoidance is working when the controller changes motion or stops in response.
The local trajectory display updates while a navigation goal is active.

## Repeatable Unity tests

Use a cube on the `Default` layer with a non-trigger `BoxCollider`. For example,
a 0.5 × 1 × 0.5 m cube with its centre 0.5 m above the floor intersects the laser
plane. A rendered object without a collider will not be detected by this sensor.
Make changes during Play mode for temporary tests, or save a separate test scene.

| Test | Procedure | Observe |
|---|---|---|
| Clear path | Send a goal a few metres away in visible free space | Robot reaches goal; `/odom` follows motion |
| Static obstacle | Put a cube on that route with room on either side | Costmap marks it; route/motion goes around it without contact |
| New obstacle | After a goal is active, place a cube ahead with ample stopping distance | Local costmap updates; robot changes trajectory or stops |
| Moving obstacle | Slowly move the cube across the route, then away | New position is marked; visible old position clears; robot reacts |
| Blocked passage | Block the full corridor | Robot stops, waits, replans, or reports failure rather than driving through |
| Removed obstacle | Move the object out of the laser's view after blocking the path | Local obstacle cells clear when free-space rays cross them |

Start moving-obstacle trials slowly. There is no object tracking or prediction,
and 2D lidar only detects geometry intersecting the scan plane. A blocked-route
goal may abort after recovery; resend the goal once the passage is free.
SLAM may retain a transient object in `/map` even after the local obstacle layer
clears it. For controlled moving-obstacle evaluation, map the empty environment
first and coordinate a saved-map/localization workflow with the mapping teammate.

Record goal success/failure, contacts, closest robot-to-obstacle clearance,
detection delay, and time to goal for each scenario. Measure clearance from the
robot body boundary, not just laser range. Repeat with several obstacle positions
and speeds before claiming robust avoidance.

For ROS evidence, in a ROS shell:

```bash
ros2 bag record -o /tmp/avoidance_trial /scan /odom /tf /tf_static /clock \
  /cmd_vel /plan /local_plan /local_costmap/costmap /map
```

Stop recording with Ctrl+C. On macOS, export it before stopping Docker:

```bash
docker cp unity-nav2:/tmp/avoidance_trial ./avoidance_trial
```

ROS bags do not measure Unity contacts automatically; log those separately.
Record short trials to limit disk use.

## Automated ROS checks

Unit tests cover body-frame velocities, yaw wrap, and stale/reset timestamps.
`test/check_obstacle_costmap.py` runs the real Jazzy controller with synthetic
TF and scans to check obstacle marking, clearing, forward commands, and stopping
short of a wall. Its simulated robot moves according to `/cmd_vel`, so the wall check fails if the robot body
(`robot_radius`) ever reaches the wall. Run that script only in an isolated test container/domain, never alongside
Unity's live `/scan` publisher. These checks do not replace Unity physics trials.

On Ubuntu / WSL, from the repository root, with Unity **not** running:

```bash
source ~/uc2_ws/install/setup.bash
cd ros2_ws/src/unity_slam_example
python3 -m pytest -q test/test_tf_odometry.py && ROS_DOMAIN_ID=77 python3 test/check_obstacle_costmap.py
```

On macOS, from the repository root (no network, separate domain, repo mounted read-only):

```bash
docker run --rm --network none --shm-size=1g \
  -e ROS_DOMAIN_ID=77 -e HOME=/home/ubuntu -u ubuntu --entrypoint bash \
  -v "$PWD/ros2_ws/src/unity_slam_example:/src/unity_slam_example:ro" \
  unity-robotics:jazzy -c '
source /opt/ros/jazzy/setup.bash
mkdir -p /tmp/ws/src && cp -a /src/unity_slam_example /tmp/ws/src/ && cd /tmp/ws
colcon build --packages-select unity_slam_example && source install/setup.bash
cd src/unity_slam_example
python3 -m pytest -q test/test_tf_odometry.py && python3 test/check_obstacle_costmap.py'
```

Expect four `PASS:` lines. The last one reports how far the robot stopped from the wall.

References: [Nav2 obstacle layer implementation for Jazzy](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/plugins/obstacle_layer.cpp)
and [Nav2 sensor/costmap overview](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization/).
