# Busy-Warehouse Planner Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the robot stalling for more than 3 s among warehouse traffic. Build two candidates
(improved DWA and TEB), benchmark both the same way, and keep the winner.

**Architecture:**
- **The bench.** `test/scenarios.py` gains a stall metric, a seeded busy-traffic scene
  (`test/busy_traffic.py`), log counters, a `--slam` variant and a `--planner` switch.
- **Shared fixes, both candidates.** Shorter move_base waits, with a recovery list long enough
  that goals don't abort sooner than today, plus slow reversing.
- **Candidate A.** DWA, with predicted cart paths graded instead of lethal beyond 0.8 s.
- **Candidate B.** `teb_local_planner`, fed the moving tracks with their velocities.
- **Unity.** A small monitor script measures the same stall metric during Unity sessions.

**Tech Stack:** ROS 1 Noetic (move_base, NavfnROS, dwa_local_planner, teb_local_planner,
costmap_converter), Python 3.8 (rospy, numpy), pytest, Docker (`unity-robotics:noetic`).

**Spec:** `docs/superpowers/specs/2026-10-24-busy-warehouse-planner-design.md`

## Global Constraints

- Branch: `thien/busy-warehouse-planner`. Commit messages carry **no** Claude credit, no
  `Co-Authored-By` trailer (CLAUDE.md). Style: `[ADD] ...` / `[UPDATE] ...` / `[FIX] ...`.
- After every task, check `.gitignore` per CLAUDE.md and run `git status --short --ignored`.
  Every new source file must be tracked, and nothing generated may appear untracked.
- Stall: forward speed `|v| < 0.02` m/s, turning or not. Not counted:
  - the first turn, until the robot first drives;
  - the final turn, from the first time within 0.25 m of the goal;
  - traffic waits: a mover within 1.5 m of the robot and within ±45° of its heading.
- Target: longest stall ≤ 3 s in every `busy` run.
- Safety pass lines:
  - no contacts;
  - each existing scene's clearance at most 0.05 m below the October 2026 table in
    `docs/obstacle_avoidance.md`;
  - in `busy`, the robot's colliders never within 0.10 m of a mover.
- Limits, unchanged: `max_vel_x` 0.2 m/s, `max_vel_theta` 0.5 rad/s, `robot_radius` 0.25 m,
  `acc_lim_x` 2.5, `acc_lim_theta` 2.5.
- New: both candidates reverse at up to 0.1 m/s.
- `cmd_vel_guard` stays in the chain, unchanged.
- Goals must not abort sooner than today: about 45 s of continuous controller failure.
- An apt dependency goes into **both** `docker/Dockerfile` and `scripts/setup_ubuntu.sh`, and
  stays only if TEB wins.
- `OBSTACLE_DETECTION_AND_AVOIDANCE.md` is git-ignored local notes. The spec's "update it" is
  void, so don't edit it.
- **Bench command (macOS),** run from the repo root. `<ARGS>` are `scenarios.py` arguments.

```bash
docker run --rm --network none -e HOME=/root -e ROS_HOSTNAME=localhost --entrypoint bash \
  -v "$PWD/catkin_ws/src:/src:ro" unity-robotics:noetic -c '
source /opt/ros/noetic/setup.bash
mkdir -p /tmp/ws && cp -a /src /tmp/ws/src && cd /tmp/ws
catkin_make > /tmp/build.log && source devel/setup.bash
cd src/unity_slam_example && <COMMAND>'
```

- **Unit tests:** `<COMMAND>` = `python3 -m pytest -q test/`.
- **Integration checks:** `<COMMAND>` = `python3 test/check_obstacle_costmap.py`. Expect
  fourteen `PASS:` lines.
- Bench output goes to the terminal or the scratchpad, never into the repo.

## Review Focus

1. A delivery goal blocked for a long time must not abort sooner than today. Task 4 adds
   `test_abort_budget_matches_today`.
2. Reversing towards an obstacle behind the robot must be slowed by the guard. Task 4 adds
   `test_reversing_towards_wall_behind_slows`.
3. TEB with perception off has no `/obstacles`, and must still drive. Task 7 verifies
   `open_box --planner teb --no-perception` reaches the goal.
4. The Unity monitor must measure stalls the same way as the bench. Task 8 runs it beside a
   bench trial and compares.
5. Where one track's graded cells overlap another track's lethal cells, the lethal value must
   win. Task 6 adds `test_occupancy_grid_keeps_maximum_value`.

---

### Task 1: Stall metric and traffic-wait rule

**Files:**
- Modify: `catkin_ws/src/unity_slam_example/test/scenarios.py` (constants near `RADIUS`; new
  helpers after `body_clearance`)
- Test: `catkin_ws/src/unity_slam_example/test/test_bench_metrics.py`

**Interfaces:**
- Produces, in `scenarios.py`:
  - constants `GOAL_TOLERANCE = 0.25` and `STALL_SPEED = 0.02`;
  - `box_points(box, spacing=0.1) -> list[(x, y)]`: perimeter points of an axis-aligned
    `(cx, cy, hx, hy)` box;
  - `traffic_blocking(x, y, yaw, outlines, distance=1.5, cone_deg=45.0, radius=RADIUS) -> bool`.
    `outlines` is an iterable of point lists. It returns True if some point has
    `hypot(p - c) - radius <= distance` and its bearing from the robot is within `cone_deg` of
    `yaw`.
  - `stall_report(trace, goal, tolerance=GOAL_TOLERANCE, speed=STALL_SPEED) -> dict`.
    - `trace` is a list of `(t, x, y, v, waiting)`.
    - It returns `longest_stall`, `stalls_over_3s`, `wait_longest`, `wait_total` and `waits`
      (count).
    - A run of samples lasts from its first sample to the first sample after it (or to the
      last sample's `t`).
    - Samples before the first `|v| >= speed` are dropped, and so are samples from the first
      one within `tolerance` of `goal` onwards.
    - A sample with `waiting` True ends a stall run and belongs to a wait run.

- [ ] **Step 1: Write the failing tests** in `test/test_bench_metrics.py`. Use
  `from scenarios import stall_report, traffic_blocking, box_points`. pytest puts `test/` on
  `sys.path`, and `scenarios.py` imports only the standard library at module level. Use a helper
  `trace(segments)` that turns `[(duration, v, waiting)]` into 0.1 s samples along +x, far from
  the goal `(100, 0)`.

```python
def test_first_turn_not_counted():          # v=0 for 5 s, then 0.2 for 5 s
    assert stall_report(trace([(5, 0, False), (5, 0.2, False)]), (100, 0))['longest_stall'] == 0

def test_mid_route_stall_counted():         # 0.2 for 2 s, 0 for 4.5 s, 0.2 for 2 s
    r = stall_report(trace([(2, .2, False), (4.5, 0, False), (2, .2, False)]), (100, 0))
    assert r['longest_stall'] == pytest.approx(4.5, abs=0.11) and r['stalls_over_3s'] == 1

def test_final_turn_not_counted():          # drive to within 0.2 m of the goal, then turn 5 s
    ...                                     # build samples ending at (goal_x - 0.2, 0) with v=0
    assert r['longest_stall'] == 0

def test_traffic_wait_split_out():          # stall 1 s, wait 2 s, stall 1 s
    r = stall_report(trace([(2, .2, False), (1, 0, False), (2, 0, True), (1, 0, False),
                            (1, .2, False)]), (100, 0))
    assert r['longest_stall'] == pytest.approx(1.0, abs=0.11)
    assert r['wait_longest'] == pytest.approx(2.0, abs=0.11) and r['waits'] == 1

def test_reversing_is_not_a_stall():
    assert stall_report(trace([(1, .2, False), (4, -0.1, False)]), (100, 0))['longest_stall'] == 0

def test_traffic_blocking_cone_and_distance():
    cart = box_points((2.0, 0.0, 0.3, 0.3))          # near face 1.7 m ahead: 1.45 m from body
    assert traffic_blocking(0, 0, 0.0, [cart])
    assert not traffic_blocking(0, 0, math.pi, [cart])                 # behind
    assert not traffic_blocking(0, 0, 0.0, [box_points((2.2, 0, 0.3, 0.3))])   # 1.65 m
    assert not traffic_blocking(0, 0, 0.0, [box_points((1.0, 1.4, 0.1, 0.1))]) # every point ≥ 49.8° off
```

- [ ] **Step 2: Run them to make sure they fail.** Use the bench command with
  `python3 -m pytest -q test/test_bench_metrics.py`. Expected: ImportError, `stall_report`
  not defined.
- [ ] **Step 3: Implement** `box_points`, `traffic_blocking` and `stall_report` in
  `scenarios.py`, with the signatures above.
- [ ] **Step 4: Run all unit tests and make sure they pass.** Use the bench command with
  `python3 -m pytest -q test/`. Expected: all pass (62+ passed, 1 skipped).
- [ ] **Step 5: Commit** `[ADD] scenario bench: stall metric and traffic-wait rule`

### Task 2: Seeded busy-warehouse traffic

**Files:**
- Create: `catkin_ws/src/unity_slam_example/test/busy_traffic.py`
- Test: `catkin_ws/src/unity_slam_example/test/test_busy_traffic.py`

**Interfaces:**
- Produces:
  - `busy_traffic.BOXES`: the static layout, as `(cx, cy, hx, hy)` boxes.
  - `busy_traffic.GOAL = (16.2, 0.0)`.
  - `class Traffic(seed, carts=10, speed=0.8, half_length=0.8, half_width=0.3, lane=0.5,
    gap=0.8, stops_per_minute=1.0, stop_seconds=3.0, patience=5.0, start_clearance=3.0)`, with:
    - `.step(dt, robot_xy, robot_radius=0.25) -> None`;
    - `.boxes() -> list[(cx, cy, hx, hy)]`;
    - `.carts`, a list of mutable cart states (fields `start`, `end` (node tuples),
      `progress` in [0, 1], `paused`, `blocked`), so tests can place a cart.
- **Layout (fixed by this plan):**
  - Aisles 3 m wide, on centre lines `y ∈ {-4, 0, 4}` (along x) and `x ∈ {2, 8, 14}` (across).
  - Shelves 1 m deep: `y ∈ ±[1.5, 2.5]`, spanning `x ∈ [3.5, 6.5]` and `[9.5, 12.5]`.
  - Outer walls (0.05 m half-thickness) at `y = ±5.5`, `x = -1.0` and `x = 17.0`.
  - The robot starts at the origin facing +x. The goal `(16.2, 0)` lies past the last cross
    aisle.
- **Motion (fixed by this plan):**
  - Carts drive between adjacent nodes of the 3 × 3 grid `{2, 8, 14} × {-4, 0, 4}`, offset
    `lane` m to the right of their travel direction (keep right).
  - The box is long along travel, and swaps half-extents on a turn.
    `# shortcut: instant 90° turns at nodes; model the swing if a planner reacts to it.`
  - At a node, a cart picks a random neighbour other than the one it came from, unless it's at
    a dead end.
  - Random pauses: probability `stops_per_minute / 60 * dt` per step, lasting `stop_seconds`.
  - A cart holds still while the robot circle, or another cart's box, intersects the
    `gap`-long strip ahead of it.
  - Held for `patience` s, a cart turns back: swap `start` and `end`, `progress = 1 - progress`.
  - Start positions are drawn from `random.Random(seed)`, at least `start_clearance` from the
    origin and not overlapping another cart.

- [ ] **Step 1: Write the failing tests**

```python
def run(seed, seconds, robot=(100.0, 100.0)):
    t = Traffic(seed)
    frames = []
    for _ in range(int(seconds / 0.1)):
        t.step(0.1, robot)
        frames.append(t.boxes())
    return frames

def test_same_seed_same_traffic():
    assert run(3, 60) == run(3, 60)

def test_carts_stay_in_aisles_and_apart():          # 120 s, robot out of the way
    for boxes in run(1, 120):
        assert not any(overlap(c, s) for c in boxes for s in BOXES)
        assert not any(overlap(a, b) for a, b in itertools.combinations(boxes, 2))

def test_start_clear_of_robot():
    assert all(math.hypot(cx, cy) >= 3.0 for cx, cy, _, _ in Traffic(5).boxes())

def test_cart_yields_then_turns_back():
    # one cart heading +x along y=-0.5 lane at (5.0, -0.5); robot circle 1.3 m ahead of its centre
    # step 3 s: the cart's box never overlaps the robot circle and its position is unchanged
    # step 6 s more (past patience 5 s): its distance to the robot has grown
```
`overlap(a, b)` is a small axis-aligned test written in the test file.

- [ ] **Step 2: Run them to make sure they fail** (`ModuleNotFoundError: busy_traffic`).
- [ ] **Step 3: Implement** `busy_traffic.py` to the layout and motion above.
  `random.Random(seed)` is the only source of randomness.
- [ ] **Step 4: Run all unit tests.** Expected: all pass.
- [ ] **Step 5: Commit** `[ADD] scenario bench: seeded busy-warehouse traffic`

### Task 3: Busy scene, seeds, SLAM variant and log counters in the bench

**Files:**
- Modify: `catkin_ws/src/unity_slam_example/test/scenarios.py` (`SCENES`, `boxes_at`,
  `run_trial`, `main`, LAUNCH template)

**Interfaces:**
- Consumes: Task 1 helpers, Task 2 `Traffic`, `BOXES`, `GOAL`.
- Produces:
  - `movers_at(scene, elapsed) -> list[box]`, the moving boxes only.
    `boxes_at = scene boxes + movers_at`.
  - `SCENES['busy'] = dict(goal=GOAL, boxes=BOXES, traffic=dict(carts=10), limit=240.0)`.
  - `run_trial(name, port, rate, perception, limit, seed=0, slam=False)`.
  - Extra result keys: `longest_stall`, `stalls_over_3s`, `wait_longest`, `wait_total`,
    `waits`, `mover_clearance`, `empty_plan`, `no_path`, `recoveries`.
  - CLI: `--seed N`. Run k of a scene uses seed `N + k`, so run k is identical traffic for
    every planner. `--slam` is rejected with `--no-perception`.

- [ ] **Step 1: Wire the traffic in.**
  - For a scene with `traffic`, `run_trial` builds `Traffic(seed, **scene['traffic'])`.
  - Once the goal is sent, `world_loop` calls `traffic.step(dt, (x, y))`.
  - Scans and clearances use `scene boxes + traffic.boxes()`.
  - A scene's `limit` overrides `--limit`.
- [ ] **Step 2: Record and score.**
  - The 0.1 s trace records `(elapsed, x, y, v, waiting)`.
  - `waiting = traffic_blocking(x, y, yaw, [box_points(b) for b in movers])`.
  - `mover_clearance = min body_clearance` to the movers only.
  - Pass the result through `stall_report(trace, scene['goal'])`.
- [ ] **Step 3: Count log lines.** After the trial, count these strings in `nodes.log`:
  - `empty_plan`: `Received an empty transformed plan`
  - `no_path`: `DWA planner failed to produce path`
  - `recoveries`: `Clearing both costmaps`
- [ ] **Step 4: Add `--slam`.**
  - Add `slam_toolbox` (`async_slam_toolbox_node`, `config/slam_toolbox.yaml`,
    `scan_topic: /scan_static`) to the launch.
  - Don't publish the static `map → odom` transform or `/map`.
- [ ] **Step 5: Extend the output.** Add the longest stall, the longest wait, mover clearance
  and the three counters to the per-trial line. Add the longest stall (max), the longest wait
  (max) and mover clearance (min) to the summary table.
- [ ] **Step 6: Check nothing regressed.**
  - Unit tests pass.
  - `python3 test/scenarios.py open_box cross` prints both rows as `ok`, with the new columns.
- [ ] **Step 7: Baseline, today's config.** Run
  `python3 test/scenarios.py busy --runs 5 --seed 0` and keep the summary in the scratchpad.
  It's the "before" row for Task 9. Expected: trials complete; failures or stalls over 3 s are
  data, not errors.
- [ ] **Step 8: Commit** `[ADD] scenario bench: busy scene, seeds, slam variant, log counters`

### Task 4: Shared fixes: shorter waits, same abort budget, slow reversing

**Files:**
- Modify: `catkin_ws/src/unity_slam_example/config/move_base.yaml` (lines 14–37 and the
  `DWAPlannerROS` block)
- Test: `catkin_ws/src/unity_slam_example/test/test_move_base_config.py` (create)
- Test: `catkin_ws/src/unity_slam_example/test/test_guard_math.py` (add tests)

**Interfaces:**
- Produces config used by Tasks 6, 7 and 9:
  - `controller_patience: 2.0`, `planner_patience: 2.0`, `oscillation_timeout: 10.0`.
  - `recovery_behaviors`: 22 `ClearCostmapRecovery` entries. The first clears outside 3 m, the
    rest outside 1 m, all with `layer_names: [obstacle_layer]` (YAML anchors keep it short).
  - DWA `min_vel_x: -0.1`.

- [ ] **Step 1: Write the failing tests**

```python
# test_move_base_config.py: yaml.safe_load(config/move_base.yaml)
def test_abort_budget_matches_today():
    c = load()
    assert c['controller_patience'] <= 2.0
    assert (len(c['recovery_behaviors']) + 1) * c['controller_patience'] >= 45.0

def test_both_candidates_may_reverse_slowly():
    assert load()['DWAPlannerROS']['min_vel_x'] == -0.1

# test_guard_math.py
def test_reversing_towards_wall_behind_slows():
    v, w = guard_twist(-0.1, 0.0, wall(-0.25))
    assert -0.1 < v < 0.0

def test_wall_behind_does_not_slow_forward():
    assert guard_twist(0.2, 0.0, wall(-0.25)) == (0.2, 0.0)
```
- [ ] **Step 2: Run them to make sure they fail.** The config tests fail and the guard tests
  pass. The guard already handles negative speeds; these tests pin that.
- [ ] **Step 3: Change `move_base.yaml`** to the values above. Update the comments to say why:
  act within 2 s, and the 22 entries keep about 46 s before an abort, as today, for
  `delivery_controller`.
- [ ] **Step 4: Run the unit tests and the integration checks.** Expected: all unit tests pass,
  and fourteen `PASS:` lines.
- [ ] **Step 5: Benchmark the shared fixes alone.** Run
  `python3 test/scenarios.py busy --runs 5 --seed 0` and keep the summary in the scratchpad.
- [ ] **Step 6: Commit** `[UPDATE] move_base: act within 2 s, keep the abort budget, reverse at 0.1 m/s`

### Task 5: Reproduce and diagnose the lost-route stall

**Files:**
- Modify: whatever the diagnosis shows. `slam_toolbox.yaml` and `tf_odometry` are allowed, and
  any change there gets flagged for Đức Trí.
- Modify: `docs/obstacle_avoidance.md` (a new subsection, "Lost route: empty transformed plan")

- [ ] **Step 1: Try to reproduce.** Run `python3 test/scenarios.py busy --slam --runs 3 --seed 0`.
  Look at the `empty_plan` column.
- [ ] **Step 2: If `empty_plan > 0`, find the cause** using superpowers:systematic-debugging,
  then fix it. Leads:
  - `base_local_planner::prunePlan` erases the whole plan when no point is within 1 m of the
    robot.
  - `map → odom` jumps from `slam_toolbox`.
- [ ] **Step 3: Write it up** in the doc subsection: the cause and the fix. If it couldn't be
  reproduced, say so, and say that Task 4's 2 s patience bounds it.
- [ ] **Step 4: Rerun** Step 1, and also the integration checks if the fix changed code or
  config. Expected: `empty_plan` stays 0, or every empty-plan stretch is under 3 s.
- [ ] **Step 5: Commit** `[FIX] ...` or `[UPDATE] docs: lost-route stall findings`

### Task 6: Candidate A: graded predicted-path cost for DWA

**Files:**
- Modify: `catkin_ws/src/unity_slam_example/unity_slam_example/avoidance_math.py`
- Modify: `catkin_ws/src/unity_slam_example/unity_slam_example/predicted_obstacles.py`
- Modify: `catkin_ws/src/unity_slam_example/config/perception.yaml` (the `predicted_obstacles`
  block)
- Modify: `catkin_ws/src/unity_slam_example/config/move_base_predicted.yaml`
  (`trinary_costmap: false`)
- Test: `catkin_ws/src/unity_slam_example/test/test_avoidance_math.py`

**Interfaces:**
- Produces:
  - `predicted_costs(tracks, horizon=2.0, step=0.2, min_speed=0.15, min_confidence=0.3,
    padding_by_class=None, spacing=0.025, robot_xy=(0.0, 0.0), keep_out=0.45,
    lethal_horizon=0.8, graded_max=80, graded_min=30) -> (points (N, 2), values (N,) int)`.
    - Value 100 at `t <= lethal_horizon`, and for padded static tracks.
    - Beyond that,
      `round(graded_max - (graded_max - graded_min) * (t - lethal_horizon) / (horizon - lethal_horizon))`.
  - `predicted_points(...)` becomes `predicted_costs(..., lethal_horizon=horizon)[0]`, so it
    behaves as before.
  - `occupancy_grid(points, robot_xy, size=6.0, resolution=0.05, values=None)`. With
    `values=None` every point is 100, as today. Otherwise each cell takes the maximum value
    (`np.maximum.at`).
  - New `perception.yaml` keys: `lethal_horizon: 0.8`, `graded_max: 80`, `graded_min: 30`.
    The StaticLayer maps 80 to cost 203, below inscribed (253), so DWA can still use those
    cells.

- [ ] **Step 1: Write the failing tests**

```python
def test_predicted_costs_lethal_near_then_graded():
    mover = Track(2.0, 0.0, 0.0, 0.2, 0.2, 0.5, 0.0, 'obstacle', 1.0)
    pts, vals = predicted_costs([mover], robot_xy=(-5.0, 0.0))
    assert vals.max() == 100 and vals.min() == 30
    assert set(vals[vals < 100]) <= set(range(30, 81))
    front = pts[:, 0] > 2.0 + 0.5 * 0.8 + 0.1 + 1e-6       # past the box's front at t = 0.8 s
    assert (vals[front] < 100).all()

def test_occupancy_grid_keeps_maximum_value():
    _, _, _, data = occupancy_grid([(0.01, 0.01), (0.02, 0.02), (1.01, 0.01)], (0.0, 0.0),
                                   values=[40, 100, 40])
    assert data.max() == 100 and 40 in data
```
- [ ] **Step 2: Run them to make sure they fail.**
- [ ] **Step 3: Implement** the two functions. `predicted_obstacles.publish` then uses
  `predicted_costs` and passes `values` to `occupancy_grid`. Read the three new parameters.
  Comment `trinary_costmap: false` with why.
- [ ] **Step 4: Run the unit tests and the integration checks.** If the "predicted path ...
  marked lethal ahead of it" check probes further than 0.8 s ahead, move its probe inside
  0.8 s, and add a probe that expects a graded cost (1–252) further out.
- [ ] **Step 5: Commit** `[UPDATE] predicted obstacles: lethal for 0.8 s, graded after (candidate A)`

### Task 7: Candidate B: TEB with moving tracks as dynamic obstacles

**Files:**
- Modify: `docker/Dockerfile`, `scripts/setup_ubuntu.sh` (add `ros-noetic-teb-local-planner`
  in both lists)
- Modify: `catkin_ws/src/unity_slam_example/package.xml` (`exec_depend` `teb_local_planner`,
  `costmap_converter`)
- Create: `catkin_ws/src/unity_slam_example/config/teb_local_planner.yaml`
- Modify: `catkin_ws/src/unity_slam_example/unity_slam_example/avoidance_math.py`
  (`moving_polygons`)
- Create: `catkin_ws/src/unity_slam_example/unity_slam_example/teb_obstacles.py`, and
  `nodes/teb_obstacles` (the same wrapper as `nodes/predicted_obstacles`)
- Modify: `catkin_ws/src/unity_slam_example/CMakeLists.txt` (install `nodes/teb_obstacles`)
- Modify: `catkin_ws/src/unity_slam_example/launch/unity_slam_example.launch`
- Modify: `catkin_ws/src/unity_slam_example/test/scenarios.py` (`--planner`)
- Test: `catkin_ws/src/unity_slam_example/test/test_avoidance_math.py`

**Interfaces:**
- Produces:
  - `moving_polygons(tracks, min_speed=0.15, min_confidence=0.3) -> list[(corners (4, 2), vx,
    vy, yaw)]`, with the corners from `box_outline(track, 0.0, spacing=1e9)`.
  - The node `teb_obstacles` reads `/obstacles` and publishes
    `costmap_converter/ObstacleArrayMsg` on `/move_base/TebLocalPlannerROS/obstacles`:
    - header copied from the input (odom);
    - one `ObstacleMsg` per polygon: `id` = index, `polygon` = the corners,
      `orientation` = `yaw_to_quaternion(yaw)`, `velocities.twist.linear.x/y` = `vx`/`vy`.
    - Parameters `~min_speed` and `~min_confidence` (0.15, 0.3, the guard's values).
  - Launch arg `planner` (default `dwa`):
    - `teb` loads `teb_local_planner.yaml` after `params_file` and starts `teb_obstacles` in
      the perception group;
    - `move_base_predicted.yaml` is loaded only when perception is on **and** the planner is
      `dwa` (TEB predicts in time itself; loading both would count carts twice).
  - `scenarios.py --planner {dwa,teb}` (default `dwa`), passed to each trial and applied to its
    LAUNCH the same way.
- **`teb_local_planner.yaml`** (fixed by this plan; tune the rest):
  - Planner: `base_local_planner: teb_local_planner/TebLocalPlannerROS`.
  - Under `TebLocalPlannerROS:`
    - Speed: `max_vel_x: 0.2`, `max_vel_x_backwards: 0.1`, `max_vel_theta: 0.5`,
      `acc_lim_x: 2.5`, `acc_lim_theta: 2.5`.
    - Footprint: `footprint_model: {type: circular, radius: 0.25}`.
    - Obstacles: `min_obstacle_dist: 0.15`, `inflation_dist: 0.35`,
      `include_dynamic_obstacles: true`, `dynamic_obstacle_inflation_dist: 0.4`,
      `include_costmap_obstacles: true`, `costmap_converter_plugin: ""`.
    - Goal: `xy_goal_tolerance: 0.25`, `yaw_goal_tolerance: 0.25`.
    - Forward preference: `weight_kinematics_forward_drive: 100`.
    - Alternative routes: `enable_homotopy_class_planning: true`, `max_number_classes: 3`.
    - Frames: `odom_topic: odom`, `map_frame: odom`.

- [ ] **Step 1: Write the failing test**

```python
def test_moving_polygons_filters_and_keeps_velocity():
    cart = Track(1.0, 2.0, 0.0, 1.6, 0.6, 0.8, 0.0, 'obstacle', 0.9)
    slow = cart._replace(vx=0.1)
    unsure = cart._replace(score=0.2)
    (corners, vx, vy, yaw), = moving_polygons([cart, slow, unsure])
    assert sorted(map(tuple, np.round(corners, 6))) == [(0.2, 1.7), (0.2, 2.3), (1.8, 1.7), (1.8, 2.3)]
    assert (vx, vy, yaw) == (0.8, 0.0, 0.0)
```
- [ ] **Step 2: Run it to make sure it fails.**
- [ ] **Step 3: Implement** `moving_polygons`, the `teb_obstacles` node, the config, the launch
  arg and the `--planner` switch. Add TEB's "cannot produce a command" log line to the bench's
  `no_path` counter: find its exact text in a trial's `nodes.log`. Check that
  `scripts/run_ros.sh` and `docker/start-navigation.sh` forward `planner:=teb` the way they
  forward `perception:=false`, and fix them if they don't.
- [ ] **Step 4: Rebuild the image** with `docker build -t unity-robotics:noetic docker`, then
  run the unit tests and the integration checks. Expected: all unit tests pass, and fourteen
  `PASS:`, since the checks still use DWA.
- [ ] **Step 5: Smoke test.**
  - `python3 test/scenarios.py open_box cross --planner teb`: both rows `ok`.
  - `python3 test/scenarios.py open_box --planner teb --no-perception`: `ok` (Review Focus 3).
  - On `cross`, the trial's `nodes.log` shows no warnings about the `obstacles` topic.
- [ ] **Step 6: Commit** `[ADD] TEB local planner with tracked obstacles (candidate B)`

### Task 8: Unity stall monitor

**Files:**
- Create: `catkin_ws/src/unity_slam_example/test/unity_stall_monitor.py`

**Interfaces:**
- Consumes: `stall_report`, `traffic_blocking` and `RADIUS` from `scenarios.py`;
  `box_outline` and `tracks_from_detections` from the package.
- Produces: a script for a ROS shell during Play.
  - It samples at 10 Hz: `v` from `/odom`, and the pose from TF `odom → base_link`.
  - A goal from `/move_base/goal` is converted to `odom` once, when it arrives.
  - `/move_base/result` ends a goal.
  - **Waiting:** `traffic_blocking` over the outlines of `/obstacles` tracks that have **ever**
    moved (a track id once seen at speed ≥ 0.15 m/s). Shelves never count, and a stopped cart
    still counts.
  - **Clearance:** min over `/scan` points of the distance to the robot centre, minus `RADIUS`.
  - It prints one line per goal: status, time, the `stall_report` fields, and min clearance.
    On Ctrl+C it prints a session summary.
  - If the clock jumps back more than 1 s (Play restarted), it drops the open goal.

- [ ] **Step 1: Implement** the script. It reuses the existing functions and adds no new
  geometry.
- [ ] **Step 2: Compare with the bench** (Review Focus 4).
  - Inside one bench container, start `python3 test/scenarios.py busy --runs 1 --parallel 1`
    in the background.
  - When its master is up, run the monitor with `ROS_MASTER_URI=http://localhost:11320` and
    `use_sim_time`.
  - Expected: the monitor's `longest_stall` is within 0.3 s of the bench result.
- [ ] **Step 3: Commit** `[ADD] Unity stall monitor`

### Task 9: Benchmark, Unity sessions, decision, docs, cleanup

**Files:**
- Modify: `docs/obstacle_avoidance.md` (settings tables, results, how to run `busy` and the
  monitor)
- Delete or revert: the losing candidate's files and changes (Task 6 or Task 7)

- [ ] **Step 1: Run the bench for both candidates.** The image now has TEB installed. Run the
  same commands for each planner:
  - `python3 test/scenarios.py --runs 3 --planner X` (every existing scene);
  - `python3 test/scenarios.py busy --runs 5 --seed 0 --planner X`.

  Save both summaries in the scratchpad.
- [ ] **Step 2: Unity sessions, with the owner.** Ask the owner to pick 4 goals across
  `SimpleWarehouseScene` and write them down. For each candidate, the owner runs 3 sessions of
  about 10 minutes:
  - start the stack (`bash docker/start-navigation.sh planner:=X`);
  - spawn about 10 `WorkerWithCart` (B to choose, then T);
  - send the 4 goals while the monitor runs;
  - note container CPU from `docker stats`.

  Collect the monitor summaries and the ROS log warning counts.
- [ ] **Step 3: Apply the spec's decision rules,** in order:
  1. every `busy` run has a longest stall ≤ 3 s and meets the safety pass lines;
  2. existing scenes: no fewer goals reached, and the clearance pass line holds;
  3. no contacts in Unity;
  4. if both pass, shorter `busy` time to goal wins, and DWA wins if they're within 10 %;
  5. if neither passes, stop and report to the owner.

  If a Unity stall exceeds 3 s, or neither candidate qualifies, **stop and show the owner**
  before going on.
- [ ] **Step 4: Remove the loser.**
  - **DWA wins:** delete
    - `teb_local_planner.yaml` and `teb_obstacles` (module, node, CMake line);
    - `moving_polygons` and its test;
    - the launch `planner` arg and `--planner teb`;
    - the TEB apt and package.xml dependencies.
  - **TEB wins:** make TEB the default in `move_base.yaml`, then revert Task 6:
    - `predicted_costs`, its parameters and `trinary_costmap`;
    - keep `predicted_obstacles`, which RViz still displays.

  Rerun the unit tests and the integration checks.
- [ ] **Step 5: Docs.**
  - A results table for both candidates: today's table columns plus longest stall, longest
    wait, mover clearance and the counters, with the baseline and shared-only `busy` rows from
    Tasks 3 and 4.
  - The Unity results, and the reason for the choice.
  - Updated settings tables: patience, the recovery list, `min_vel_x`, and the winner's
    settings.
  - How to run `busy` and the monitor.
- [ ] **Step 6: Final check.**
  - Unit tests and the integration checks pass.
  - `git status --short --ignored` shows nothing generated left untracked, and no new source
    file untracked.
  - Commit `[UPDATE] busy-warehouse benchmark results; choose <winner>`.
