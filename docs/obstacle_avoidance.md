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
stop: it rolls the command forward and slows it if the robot's circle (0.25 m) would touch at
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
| Obstacle marking range (`obstacle_range`) | 3 m local, 6 m global | Insert observed obstacles; the global costmap sees a whole shelf row before Navfn picks a side |
| Ray clearing range (`raytrace_range`) | 4 m local, 7 m global | Clear previously observed obstacles when scans show free space |
| Inflation radius | 0.60 m global, 0.30 m local | Global: Navfn keeps its path about 0.35 m from obstacles. Local: DWA's obstacle cost (measured from the robot's edge) only starts closer than that. Not a guaranteed clearance |
| Robot radius | 0.25 m | Covers the Unity robot's colliders, including the rear corners 0.238 m from the turning centre (also in `cmd_vel_guard` and `teleop_assist`) |
| Goal tolerance (`NavfnROS/default_tolerance`) | 0 m | Plan to the clicked goal or fail; never substitute a nearby point |
| Maximum forward speed | 0.2 m/s | Initial simulation test speed |
| Maximum turn rate | 0.5 rad/s | Matches the Unity controller limit |
| Minimum translating speed (`min_vel_trans`) | 0.03 m/s | Reject barely moving candidates unless they have a useful turn rate |
| Minimum turn rate (`min_vel_theta`) | 0.2 rad/s | With a positive translating minimum, reject weak in-place turns |
| Acceleration (`acc_lim_x`, `acc_lim_trans`) | 2.5 m/s² | Every 10 Hz cycle can sample any speed up to the maximum; Unity's wheels ramp physically |
| Turn acceleration (`acc_lim_theta`) | 2.5 rad/s² | Make the minimum turn rate reachable from rest in the 10 Hz dynamic window |
| DWA prediction horizon (`sim_time`) | 2 s | Evaluate candidate motion against current obstacles |
| Scoring (`path_distance_bias`, `goal_distance_bias`, `occdist_scale`) | 32, 24, 0.1 | Follow the path vs. head for the goal vs. keep away from obstacles |
| Recovery (`recovery_behaviors`) | Clear obstacle layer outside 3 m, then outside 0.92 m | Replan on a cleaned costmap; no in-place spins |
| Maximum scan interval (`expected_update_rate`) | 0.5 s | Mark the observation source stale if scans stop |

DWA requires a candidate to meet at least one of its translating/turning minimums.
Setting `min_vel_trans` to zero permits tiny in-place turns even when `min_vel_theta`
is positive. The startup settings above prevent that stall while `min_vel_x: 0`
still permits rotation without driving forward. They are planner sampling limits;
the collision checks and `cmd_vel_guard` can still slow or stop the robot. Restart ROS after
editing these settings.

`occdist_scale` was 0.02, which let DWA follow the path within about 0.25 m of thin objects
such as posts. The scan marks those only from close up, and once the robot's footprint touches
a lethal cell every trajectory is invalid, so the goal fails. At 0.1 the robot keeps about
0.45 m from obstacles in the Unity warehouse.

### Why the robot hit shelves and changed its destination

- **The robot was bigger than ROS thought.** Every node modelled it as a 0.22 m circle around
  `base_link`, but the Unity Waffle's body box sits 0.064 m behind it: its rear corners are
  0.238 m out. Turning on the spot next to a shelf, a corner could swing into it, and the guard
  never limits a turn on the spot (a circle does not sweep when it turns). `robot_radius` is now
  0.25 m everywhere, and the global inflation 0.60 m so that Navfn's path keeps its clearance.
- **Navfn replaced unreachable goals.** With `default_tolerance: 0.5`, a goal within reach of an
  obstacle (clicked next to a shelf, or covered later by a newly seen obstacle) was quietly
  replaced by the nearest free point up to 0.5 m away. Navfn picked that point again at every
  1 Hz replan, so the robot drove to a spot you did not choose and turned there until move_base
  gave up (`test/scenarios.py goal_at_box`: 0.47 m from the goal, over 1500° of turning, then
  "the robot appears to be oscillating"). Now Navfn plans to the goal itself or fails: put
  goals at least 0.25 m from obstacles, or move_base stops and logs "Failed to get a plan".

### Why the robot used to turn on the spot near shelves

With a goal behind a shelf, the robot drove to within 1–3 m of it, then stopped and turned on
the spot, sometimes several full turns, before it drove round. `test/scenarios.py rack_end`
reproduces this with the real move_base. Three settings combined:

- **The local inflation counted twice.** DWA scores a trajectory by the highest cost under the
  footprint's *outline*, but the inflation layer already measures from the robot's centre. With
  0.55 m local inflation, any body clearance under 0.55 m was costly, including the 0.33 m that
  Navfn's path kept. A short, slow trajectory reaches less of that cost, so DWA slowed down
  next to shelves and corners. Local inflation is now 0.30 m: following the path costs nothing,
  and DWA still keeps away from anything closer.
- **The dynamic window was too narrow to recover.** DWA only samples velocities reachable within
  one 0.1 s cycle. At 0.5 m/s², a robot that had slowed to a crawl could sample at most
  0.05 m/s, so turning on the spot always looked better. `acc_lim_x` is now 2.5 m/s², so every
  cycle can pick any speed. Unity's wheel drives still ramp physically.
- **The far end of a shelf was not in the global costmap.** Scans marked obstacles only within
  3 m, so Navfn first routed round the end it could see, then switched sides as more of the
  shelf appeared. A robot already turning on the spot cannot reverse the turn within one cycle
  (the minimum in-place rate is 0.2 rad/s), so it turned the long way round, up to several full
  turns. The global costmap now marks up to 6 m, so the path no longer switches. A larger
  `acc_lim_theta` would allow the reversal but makes DWA dither in front of narrow gaps.

Recovery clears the scan obstacles (`obstacle_layer`) and replans. move_base's default list also
spins the robot twice on the spot (`rotate_recovery`). With a 360 degree lidar a spin shows
nothing new, so it is left out. The clears name `obstacle_layer` explicitly: their default layer
name, `obstacles`, matches nothing in these costmaps, so the default clears did nothing.

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
score, velocity in `odom`). A track's velocity is zero until the scan confirms that it moves
([obstacle_detection.md](obstacle_detection.md)): driving past a static box no longer makes it
look like it is moving, so its predicted path no longer blocks the robot's own route. A box
coming straight at the robot (or from behind) is confirmed too, within about 0.5 s. Ten times a
second, it publishes `/obstacles_predicted`: a 6 × 6 m, 5 cm grid around the robot.
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
robot's circle. A track that would reach the robot even if it stopped cannot be avoided by
braking, so it only counts while the command brings the robot closer to it than standing still:
turning and backing away stay allowed, driving on into its way does not. Before, such a track
was ignored, and DWA (which does not see predictions within `keep_out`) could drive straight
into a box crossing just ahead.

With perception, slam_toolbox maps `/scan_static` instead of `/scan`: `lidar_obstacles` blanks
the returns within 0.1 m (`static_scan_margin`) of each moving track. Otherwise a box driving
past the robot leaves a trail of occupied cells in `/map`, the global costmap's static layer
keeps them, and Navfn routes round a wall that is not there (`test/scenarios.py headon` with
slam_toolbox: 12 such cells in the middle of the aisle with `/scan`, at most 1 with
`/scan_static`). A box is still mapped while it stands still and in the 0.5 s before its track is
confirmed as moving. Scans that later see through that space can clear those cells, but
slam_toolbox needs many of them to outweigh a few hits. The costmaps always use the raw `/scan`.

Tune `predicted_obstacles` in `config/perception.yaml`:

| Setting | Initial value | Purpose |
|---|---:|---|
| `horizon` / `step` | 2 s / 0.2 s | How far ahead to predict (DWA `sim_time`) and how finely |
| `min_speed` | 0.15 m/s | Slower tracks count as static |
| `min_confidence` | 0.3 | Fused scores can be lower than lidar-only ones |
| `padding` | `person: 0.35` m | Extra clearance per class; unlisted classes get none |
| `keep_out` | 0.45 m | No predicted cells this close to the robot centre |
| `grid_size` / `resolution` | 6 m / 0.05 m | Match the local costmap |

`min_speed`, `min_confidence` and `obstacle_timeout` are repeated as `cmd_vel_guard` parameters
in the launch file; keep them equal.

Limits:
- Constant velocity only. A track that turns or stops is re-predicted at the next scan (10 Hz),
  not anticipated. A box that turns back towards a robot already crossing its path can still
  reach it: DWA does not reverse (`min_vel_x: 0`) and the guard only stops it driving further in.
  `test/scenarios.py sweeper` shows this with a box sweeping back and forth across an aisle.
- The robot does not get out of the way of something faster coming from behind (`overtaken`).
  It is ignored by the guard (braking cannot help) and DWA only plans forward.
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
- The first key press cancels any navigation goal. When you release the keys (or leave P mode),
  `teleop_assist` goes quiet and navigation goals work again, also in the first-person view.
  In direct mode (**O**) navigation goals are ignored.

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

### Moving boxes in Unity

Any scene with the robot gets a `MovingObstacleSpawner` in Play mode (bottom-right help line).
With the Game view focused:

| Key | Spawns |
|---|---|
| **M** | A box crossing the robot's path 3 m ahead at 0.4 m/s, entering from the side with more room |
| **N** | A box coming straight at the robot from up to 6 m ahead at 0.3 m/s, turning back 0.6 m short of where the robot was |
| **K** | Removes the spawned boxes |

Each box is 0.5 × 0.6 × 0.5 m (it crosses the lidar plane), looks like the warehouse cartons
(so the camera labels it `box`), and goes back and forth over the free floor it finds, pausing
1 s at each end. It is kinematic: it pushes the robot rather than the other way round, so any
contact in Unity is a real avoidance failure. Speeds, distances and keys are on the spawner
(select `MovingObstacleSpawner` in the Hierarchy during Play). For a fixed set-up, add a
`MovingObstacle` component (with a `Rigidbody` and collider) to any object in a test scene: it
travels `Travel` metres along `Direction` and back at `Speed`.

Try it: send a 2D Nav Goal 6–8 m ahead in an open area, then press **M** or **N** while the
robot drives. RViz should show the box's track (**Obstacles (fused)**) and a band ahead of it
(**Predicted Obstacles**); the robot should wait for a crossing box or steer round an oncoming
one. Driving by hand (**P**, hold **W**) works the same way.

Start moving-obstacle trials slowly. Prediction assumes constant velocity (see
[Moving and labelled obstacles](#moving-and-labelled-obstacles)), and 2D lidar only
detects geometry intersecting the scan plane. Repeat each trial with `perception:=false`
(Ubuntu / WSL: `bash scripts/run_ros.sh perception:=false`) to compare against scan-only avoidance. A
blocked-route goal aborts after move_base's recovery behaviours (two costmap clears);
resend the goal once the passage is free. SLAM may retain a transient
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
the guard's stop when scans cease. Startup trials use headings of 0, 90 and 180 degrees and
model a motor deadband for very weak commands: each trial must produce useful motion commands
within 2 s and complete the goal. Finally it holds "W" towards a box 3 m ahead and checks that
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

Expect fourteen `PASS:` lines. The twelfth reports how far the robot stopped from the wall, the
last how close the body came to the box.

### Scenario trials

`test/scenarios.py` runs whole navigation goals against the same real nodes plus
`lidar_obstacles` and `obstacle_fusion`, with a simulated robot (0.3 s motor lag, 1 cm range
noise) and boxes, some of them moving. Each trial publishes its own `/clock`, so a busy machine
slows it down without changing the result. Run it after the checks above, from
`catkin_ws/src/unity_slam_example` (all scenes once take about 20 minutes on 6 cores):

```bash
python3 test/scenarios.py                       # every scene once
python3 test/scenarios.py rack_end cross --runs 3
```

Results, 3 trials per scene, October 2026. Time is to the goal (or to giving up); *stopped*
includes the final turn to the goal heading (2–3 s); clearance is the closest the robot's
colliders came to a box (0: contact); *off goal* is where it ended (goal tolerance 0.25 m).

| Scene | What | Reached | Time | Stopped | Clearance | Off goal |
|---|---|---|---|---|---|---|
| `rack_end` | Goal behind a rack with uprights in front | 3/3 | 42 s | 4 s | 0.46 m | 0.21 m |
| `shelf_end` | Goal behind a deep shelf | 3/3 | 41 s | 4 s | 0.46 m | 0.21 m |
| `open_box` | Box on the route | 3/3 | 39 s | 2 s | 0.49 m | 0.20 m |
| `goal_at_box` | Goal 0.15 m in front of a box | 0/3, as intended | 15 s | 15 s | — | does not move |
| `post` | 5 cm post beside the route | 3/3 | 38 s | 2 s | 0.51 m | 0.20 m |
| `aisle_narrow` | Box in a 2.4 m aisle | 3/3 | 39 s | 2 s | 0.42 m | 0.20 m |
| `aisle_tight` | 0.70 m gaps either side of a box | 2/3 | up to 100 s | up to 62 s | 0.14 m | — |
| `cross` | Box crossing at 0.4 m/s | 3/3 | 42 s | 0 s | 1.24 m | 0.21 m |
| `cross_fast` | Box crossing at 0.8 m/s | 3/3 | 44 s | 2 s | 0.41 m | 0.20 m |
| `headon` | Box coming head-on in a 3 m aisle | 3/3 | 45 s | 2 s | 0.12 m | 0.20 m |
| `overtaken` | Faster box from behind (known limit) | 3/3 | 55 s | 10 s | 0 (contact) | 0.20 m |
| `sweeper` | Box sweeping across the aisle (known limit) | 3/3 | 73 s | 17 s | 0 (contact) | 0.21 m |

Compared with the previous commit: `rack_end` failed 1 trial in 3 and stalled for up to 74 s
with up to 1537° of turning on the spot; `cross_fast` ended in contact in 2 of 3 trials;
`goal_at_box` drove to a point 0.47 m from the goal and turned there for a minute until
move_base aborted. `aisle_tight` is a stress test: the 0.25 m robot circle leaves 0.10 m either
side, and the robot can still hesitate there, turning on the spot while Navfn chooses between
the two gaps. `overtaken` and `sweeper` are the limits listed under
[Moving and labelled obstacles](#moving-and-labelled-obstacles).

References: [costmap_2d obstacle layer (ROS wiki)](https://wiki.ros.org/costmap_2d/hydro/obstacles),
[dwa_local_planner](https://wiki.ros.org/dwa_local_planner) and
[move_base](https://wiki.ros.org/move_base).
