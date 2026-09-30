# Obstacle detection and avoidance in Unity + move_base

This project uses 2D laser scans for geometric obstacle detection and move_base's
DWA local planner for local avoidance, with a small safety guard in front of the
wheels. It does not predict moving objects' future trajectories. Moving obstacles
are handled reactively as new scans update the costmap.

## Your subsystem

```text
Unity colliders -> LaserScanSensor -> /scan -> obstacle layer -> inflation layer
                                        |                      |
Goal + SLAM map -> NavfnROS global path -> DWA local planner <-+
                                        |      |
                                        |   /cmd_vel_nav
                                        +-> cmd_vel_guard -> /cmd_vel -> Unity AGVController -> wheels

Unity /tf -> simulated /odom (pose + measured motion) -> DWA
Unity /clock -> simulation time for all ROS nodes
```

Your main deliverables are reliable scan input, obstacle marking/clearing,
appropriate robot clearance, controller tuning, and recorded test results.
The navigation goal and global map can come from other team members.

## Run

Start the ROS side (Ubuntu 20.04 + ROS 1 Noetic) as described in the setup guide for your OS:

```bash
bash docker/start-navigation.sh   # macOS: Docker container, RViz at http://localhost:6080
bash scripts/run_ros.sh           # Ubuntu / WSL: native, RViz opens as a window
```

`launch/unity_slam_example.launch` starts slam_toolbox, move_base (NavfnROS global planner,
DWAPlannerROS local planner) and `cmd_vel_guard`. The costmaps wait for TF indefinitely, so ROS
can start before you press Play. Commands flow move_base → `/cmd_vel_nav` → `cmd_vel_guard` →
`/cmd_vel` (Unity). The guard is a last-resort stop: it
rolls the command forward and slows it if the robot's circle (0.22 m) would touch at least 6
scan points within 1.2 s, down to zero at contact within 0.1 s. It also sends zero velocity
while `/scan` is older than 1 s. Its parameters are in the launch file.

1. Open `UnityProject` in Unity and open `SimpleWarehouseScene`.
2. Let Unity recompile the changed scripts, then press Play once. **If you stop Play, restart
   the ROS side before pressing Play again** (macOS: `docker stop unity-nav2`, then rerun the
   script; Ubuntu / WSL: Ctrl+C `run_ros.sh`, then rerun it).
   Unity's `/clock` restarts at 0, but SLAM keeps publishing `map -> odom` stamped in the old
   session, so the planner can't transform the robot pose and the robot never moves.
3. Keep ROS settings at ROS1, `127.0.0.1:10000`, and the robot controller in ROS mode.
4. In RViz (macOS: <http://localhost:6080>), wait for the map and scan, then set a **2D Nav Goal**
   in known free space. Start with a short route, a few metres long.

On macOS, check launch failures with `docker exec unity-nav2 tail -n 60 /tmp/unity-navigation.log`
(on Ubuntu / WSL the log is in the `run_ros.sh` terminal). `docker stop unity-nav2` stops and
removes the disposable container; export any recordings first. Either script rebuilds the
workspace on every start, so rerun it to pick up ROS configuration/code changes.

## Settings you own

Edit `catkin_ws/src/unity_slam_example/config/move_base.yaml`. This configuration targets the
installed **ROS 1 Noetic** navigation stack. The values below are starting points, not tuned
results: re-tune them with the tests below and record the values you settle on.

| Setting | Initial value | Purpose |
|---|---:|---|
| Scan | 360 rays, 10 Hz | One-degree sampling over 0–359 degrees |
| Local costmap | 6 × 6 m, 5 cm cells, 10 Hz | Nearby obstacles around the robot |
| Obstacle marking range (`obstacle_range`) | 3 m | Insert observed obstacles |
| Ray clearing range (`raytrace_range`) | 4 m | Clear previously observed obstacles when scans show free space |
| Inflation radius | 0.55 m | Penalize paths near obstacles; not a guaranteed clearance |
| Robot radius | 0.22 m | Inherited circular approximation; verify it covers the robot (also in `cmd_vel_guard`) |
| Maximum forward speed | 0.2 m/s | Initial simulation test speed |
| Maximum turn rate | 0.5 rad/s | Matches the Unity controller limit |
| Minimum turn rate (`min_vel_theta`) | 0.2 rad/s | DWA stalls when asked to turn slower than the robot can |
| DWA prediction horizon (`sim_time`) | 2 s | Evaluate candidate motion against current obstacles |
| Scoring (`path_distance_bias`, `goal_distance_bias`, `occdist_scale`) | 32, 24, 0.02 | Follow the path vs. head for the goal vs. keep away from obstacles |
| Maximum scan interval (`expected_update_rate`) | 0.5 s | Mark the observation source stale if scans stop |

Keep `marking`, `clearing`, and `inf_is_valid` enabled. Unity publishes positive
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

Open a ROS shell: on macOS `docker exec -it -u ubuntu -e HOME=/home/ubuntu unity-nav2 bash`;
on Ubuntu / WSL a new terminal with `source /opt/ros/noetic/setup.bash && source ~/uc2_ws/devel/setup.bash`.

```bash
rostopic info /scan
rostopic echo /odom
# Ctrl+C stops an echo command.
rostopic echo /cmd_vel_nav   # what move_base asks for
rostopic echo /cmd_vel       # what reaches Unity after cmd_vel_guard
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
| Blocked passage | Block the full corridor | Robot stops, runs its recovery behaviours, then reports failure rather than driving through |
| Removed obstacle | Move the object out of the laser's view after blocking the path | Local obstacle cells clear when free-space rays cross them |

Start moving-obstacle trials slowly. There is no object tracking or prediction in
the planner, and 2D lidar only detects geometry intersecting the scan plane. A
blocked-route goal aborts after move_base's recovery behaviours (clear costmaps,
rotate); resend the goal once the passage is free. SLAM may retain a transient
object in `/map` even after the local obstacle layer clears it. For controlled
moving-obstacle evaluation, map the empty environment first and coordinate a
saved-map/localization workflow with the mapping teammate.

Record goal success/failure, contacts, closest robot-to-obstacle clearance,
detection delay, and time to goal for each scenario. Measure clearance from the
robot body boundary, not just laser range. Repeat with several obstacle positions
and speeds before claiming robust avoidance.

For ROS evidence, in a ROS shell:

```bash
rosbag record -O /tmp/avoidance_trial.bag /scan /odom /tf /tf_static /clock \
  /cmd_vel_nav /cmd_vel /move_base/NavfnROS/plan /move_base/DWAPlannerROS/local_plan \
  /move_base/local_costmap/costmap /map
```

Stop recording with Ctrl+C. On macOS, export it before stopping Docker:

```bash
docker cp unity-nav2:/tmp/avoidance_trial.bag ./avoidance_trial.bag
```

ROS bags do not measure Unity contacts automatically; log those separately.
Record short trials to limit disk use.

## Automated ROS checks

Unit tests cover body-frame velocities, yaw wrap, stale/reset timestamps and the guard's
slow-down and stop logic (`test/test_guard_math.py`). `test/check_obstacle_costmap.py` runs the
real Noetic move_base, `cmd_vel_guard` and `tf_odometry` with synthetic TF, map and scans to check
obstacle marking, clearing, forward commands, stopping short of a wall, and the guard's stop when
scans cease. Its simulated robot moves according to `/cmd_vel`, so the wall check fails if the
robot body (`robot_radius`) ever reaches the wall. It starts its own ROS master on port 11312,
so its synthetic scans never reach a live stack. These checks do not replace Unity physics trials.

On Ubuntu / WSL, from the repository root, after `bash scripts/run_ros.sh --build-only`:

```bash
source ~/uc2_ws/devel/setup.bash
cd catkin_ws/src/unity_slam_example
python3 -m pytest -q test/ && python3 test/check_obstacle_costmap.py
```

On macOS, from the repository root (no network, repo mounted read-only):

```bash
docker run --rm --network none -e HOME=/home/ubuntu -u ubuntu --entrypoint bash \
  -v "$PWD/catkin_ws/src:/src:ro" unity-robotics:noetic -c '
source /opt/ros/noetic/setup.bash
mkdir -p /tmp/ws && cp -a /src /tmp/ws/src && cd /tmp/ws
catkin_make > /tmp/build.log && source devel/setup.bash
cd src/unity_slam_example
python3 -m pytest -q test/ && python3 test/check_obstacle_costmap.py'
```

Expect five `PASS:` lines. The fourth reports how far the robot stopped from the wall.

References: [costmap_2d obstacle layer (ROS wiki)](https://wiki.ros.org/costmap_2d/hydro/obstacles),
[dwa_local_planner](https://wiki.ros.org/dwa_local_planner) and
[move_base](https://wiki.ros.org/move_base).
