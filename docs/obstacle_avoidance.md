# Obstacle detection and avoidance in Unity + move_base

This project uses 2D laser scans for geometric obstacle detection and move_base's
DWA local planner for local avoidance, with a small safety guard in front of the
wheels. With perception on (the default), obstacles tracked by the perception stack
([obstacle_detection.md](obstacle_detection.md)) are also predicted 2 s ahead at
constant velocity. Their predicted paths go into the local costmap, and the guard
rolls them forward alongside the robot. Without perception, moving obstacles are
handled reactively as new scans update the costmap.

## Your subsystem

```text
Unity colliders -> LaserScanSensor -> /scan -> obstacle layer -> inflation layer
                                        |                      |
Goal + SLAM map -> NavfnROS global path -> DWA local planner <-+
                                        |      |
                                        |   /cmd_vel_nav
                                        +-> cmd_vel_guard -> /cmd_vel -> Unity AGVController -> wheels

With perception (default):
/scan + camera -> perception -> /obstacles -> predicted_obstacles -> /obstacles_predicted
                                    |           -> predicted layer of the local costmap (before inflation)
                                    +-> cmd_vel_guard (moving tracks rolled forward with the robot)

Driving by hand in Unity (P, assisted):
keys -> /cmd_vel_teleop -> teleop_assist (/scan + /obstacles, 10 m lookahead) -> /cmd_vel_nav -> cmd_vel_guard

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
DWAPlannerROS local planner), `cmd_vel_guard` and, with perception, `predicted_obstacles`. The
costmaps wait for TF indefinitely, so ROS can start before you press Play. Commands flow
move_base → `/cmd_vel_nav` → `cmd_vel_guard` → `/cmd_vel` (Unity). The guard is a last-resort
stop: it rolls the command forward and slows it if the robot's circle (0.22 m) would touch at
least 6 scan points, or one moving tracked obstacle, within 1.2 s, down to zero at contact
within 0.1 s. It also sends zero velocity while `/scan` is older than 1 s. Its parameters are in
the launch file.

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

## Moving and labelled obstacles

With perception on, `predicted_obstacles` reads the fused tracks on `/obstacles` (box, class,
score, velocity in `odom`). Ten times a second, it publishes `/obstacles_predicted`: a 6 × 6 m,
5 cm grid around the robot.
- Each confident moving track (score ≥ 0.3, speed ≥ 0.15 m/s) sweeps its box outline along its
  velocity for 2 s. The swept cells are lethal.
- A class listed under `padding` gets that much extra clearance. It is marked even when it stands
  still, so a person (0.35 m) is passed wider than a box.
- Cells within 0.45 m of the robot are left free, so a prediction crossing the robot cannot
  invalidate every DWA trajectory. The guard covers that range.
- The grid is empty while `/obstacles` is older than 0.5 s.

`config/move_base_predicted.yaml`, loaded by the launch file only with perception, adds the grid
to the local costmap as a `StaticLayer` (`predicted_layer`, maximum with the scan obstacles,
before inflation). The layer replaces its whole grid on every message, so a prediction disappears
as soon as the track moves on. A scan obstacle layer would keep cells its rays happen to miss.
The global costmap and Navfn are unchanged: DWA steers around or waits for crossing traffic
along the existing path.

`cmd_vel_guard` also subscribes to `/obstacles`. It moves each confident moving track forward in
time alongside the simulated robot motion, and slows or stops when the box would enter the
robot's circle. A track that would reach the robot even if it stopped is ignored, because braking
cannot avoid it and would only freeze the robot in its way.

Tune `predicted_obstacles` in `config/perception.yaml`:

| Setting | Initial value | Purpose |
|---|---:|---|
| `horizon` / `step` | 2 s / 0.2 s | How far ahead to predict (DWA `sim_time`) and how finely |
| `min_speed` | 0.15 m/s | Slower tracks count as static (Kalman velocity noise on parked boxes) |
| `min_confidence` | 0.3 | Fused scores can be lower than lidar-only ones |
| `padding` | `person: 0.35` m | Extra clearance per class; unlisted classes get none |
| `keep_out` | 0.45 m | No predicted cells this close to the robot centre |
| `grid_size` / `resolution` | 6 m / 0.05 m | Match the local costmap |

`min_speed`, `min_confidence` and `obstacle_timeout` are repeated as `cmd_vel_guard` parameters
in the launch file; keep them equal.

Limits:
- Constant velocity only. A track that turns or stops is re-predicted at the next scan (10 Hz),
  not anticipated.
- A track needs two scans to be confirmed, and its velocity settles over a few more. A fast
  object appearing at close range is handled by the scan and the guard first.
- Only what the 2D lidar sees gets a track. The camera adds labels, not new obstacles.
- If `predicted_obstacles` dies, the local costmap keeps its last grid. The launch file respawns
  the node.

## Assisted driving: hold W and let it steer

In Unity, press **P** to drive in first person (see
[obstacle_detection.md](obstacle_detection.md#4-in-unity-boxes-and-first-person-driving)).
Driving starts **assisted**: the keys go to ROS and the robot follows ROS `/cmd_vel`, so the ROS
side must be running. **O** switches to direct driving, where the keys drive the wheels and ROS
is ignored, and back again. The bottom-left help line shows the mode, and warns if no command
comes back from ROS.

Unity publishes the keys on `/cmd_vel_teleop` (0.5 m/s, 0.5 rad/s) and `teleop_assist` turns
them into `/cmd_vel_nav`, which `cmd_vel_guard` still checks:
- **Hold W:** the robot keeps the heading it had when W was pressed. In every direction within
  ±90° of the robot's nose, it measures how far a corridor as wide as the robot plus 0.2 m on
  each side is free, up to **10 m**. The corridor is checked against the scan and the predicted
  paths of moving tracks.
  - If the way ahead is clear for 10 m, it drives straight on.
  - If not, it steers to the clear direction closest to the held heading, keeping to the side it
    already chose.
  - Once past the obstacle, it swings back to the held heading.
  - It slows as it turns away from its nose or as the free corridor gets short, and turns in place
    when boxed in.
- **W with A/D:** you steer and the held heading follows you. It still slows before an obstacle
  straight ahead.
- **S, or A/D alone:** passed through. The guard still stops short of obstacles.
- The first key press cancels any navigation goal. When you leave P mode, ROS stops sending
  commands and navigation goals work again.

The settings are on the `teleop_assist` node in the launch file: `lookahead` (10 m), `margin`
(0.2 m), `max_turn` (0.5 rad/s), plus `span_deg`, `turn_gain`, `stop_distance`,
`slow_distance`, `deviation_weight` and `change_weight` (defaults in
`unity_slam_example/teleop_assist.py`). RViz's **Assist Heading** arrow shows the direction it
chose (`/teleop_assist/heading`).

Try it:
1. Put a cube (with a collider) a few metres ahead in an open area.
2. Press **P**, point the robot at the cube and hold **W**.
3. The robot should veer off before reaching it, pass it with clearance, and turn back to its
   original heading.
4. Press **O** and repeat: in direct mode it drives straight into the cube.

In a warehouse aisle, walls and shelves within 10 m also count. Driving at the end wall of an
aisle, the robot turns away from it early. Driving along an aisle wider than about 0.85 m, it
keeps going straight.

## Inspect detection separately from avoidance

Open a ROS shell: on macOS `docker exec -it -u ubuntu -e HOME=/home/ubuntu unity-nav2 bash`;
on Ubuntu / WSL a new terminal with `source /opt/ros/noetic/setup.bash && source ~/uc2_ws/devel/setup.bash`.

```bash
rostopic info /scan
rostopic echo /odom
# Ctrl+C stops an echo command.
rostopic echo /cmd_vel_nav   # what move_base asks for
rostopic echo /cmd_vel       # what reaches Unity after cmd_vel_guard
rostopic hz /obstacles_predicted   # 10 Hz with perception on
rostopic echo /cmd_vel_teleop      # the keys while driving assisted (P)
```

RViz has **Local Obstacles**, **Global Costmap**, **Global Path**, **Local
Trajectory** and **Predicted Obstacles** (the predicted-path grid) displays. Red laser points only verify sensor input. Detection is
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
| Moving obstacle | Slowly move the cube across the route, then away | New position is marked; **Predicted Obstacles** shows a band ahead of the cube; robot yields or steers before the cube reaches its route; old marks clear |
| Blocked passage | Block the full corridor | Robot stops, runs its recovery behaviours, then reports failure rather than driving through |
| Removed obstacle | Move the object out of the laser's view after blocking the path | Local obstacle cells clear when free-space rays cross them |

Start moving-obstacle trials slowly. Prediction assumes constant velocity (see
[Moving and labelled obstacles](#moving-and-labelled-obstacles)), and 2D lidar only
detects geometry intersecting the scan plane. Repeat each trial with `perception:=false`
(Ubuntu / WSL: `bash scripts/run_ros.sh perception:=false`) to compare against scan-only avoidance. A
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

Unit tests cover body-frame velocities, yaw wrap, stale/reset timestamps, the guard's
slow-down and stop logic including moving obstacles (`test/test_guard_math.py`), and the
prediction geometry and grid (`test/test_avoidance_math.py`). `test/check_obstacle_costmap.py`
runs the real Noetic move_base, `cmd_vel_guard`, `tf_odometry` and `predicted_obstacles` with
synthetic TF, map, scans and tracks. It checks obstacle marking and clearing, the predicted path
of a moving track being marked and then cleared, forward commands, stopping short of a wall, and
the guard's stop when scans cease. Finally it holds "W" towards a box 3 m ahead and checks that
`teleop_assist` steers around it without the body touching it, then turns back to its heading.
`test/test_teleop_math.py` covers the steering choice. Its simulated robot moves according to `/cmd_vel`, so the wall check fails if the
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
docker run --rm --network none -e HOME=/root -e ROS_HOSTNAME=localhost --entrypoint bash \
  -v "$PWD/catkin_ws/src:/src:ro" unity-robotics:noetic -c '
source /opt/ros/noetic/setup.bash
mkdir -p /tmp/ws && cp -a /src /tmp/ws/src && cd /tmp/ws
catkin_make > /tmp/build.log && source devel/setup.bash
cd src/unity_slam_example
python3 -m pytest -q test/ && python3 test/check_obstacle_costmap.py'
```

Expect eight `PASS:` lines. The sixth reports how far the robot stopped from the wall, the last
how close the body came to the box.

References: [costmap_2d obstacle layer (ROS wiki)](https://wiki.ros.org/costmap_2d/hydro/obstacles),
[dwa_local_planner](https://wiki.ros.org/dwa_local_planner) and
[move_base](https://wiki.ros.org/move_base).
