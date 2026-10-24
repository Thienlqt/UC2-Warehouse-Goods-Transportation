# Busy-warehouse navigation: stop the stalls (DWA fixes vs TEB, benchmarked)

Date: 2026-10-24. Owner: Thiện (obstacle avoidance). The owner decides the planner choice alone.

## Why

With about 10 `WorkerWithCart` movers roaming `SimpleWarehouseScene` (key **T**), the robot still
reaches its goals and avoids everything, but it stops or turns on the spot for 10 s or more at a
time, even with the goal straight ahead. The problem is wasted time, not safety.

The ROS log of such a run (13 min, 2 goals, Docker on macOS, 6 cores) showed:

| Symptom in the log | Where | What it means |
|---|---|---|
| `Received an empty transformed plan` ×160 | t = 60–77 s, ended by a costmap clear | DWA found no part of the global path near the robot and sent nothing for 17 s, until `controller_patience` (15 s) triggered recovery. Root cause **not confirmed**; suspect a momentary jump of the SLAM pose (`map → odom`) |
| `DWA planner failed to produce path` ×153 | bursts of 4 s, 8 s, 5 s and ~45 s on and off (t = 463–507 s) | Every sampled trajectory hit a lethal cell, almost certainly the 2 s predicted bands of moving carts (`predicted_layer`) |
| `Control loop missed its desired rate` ×10, `Map update loop missed` ×10 | scattered | Minor |
| Container CPU 97 % of 6 cores | `camera_detector` 2.6 cores (14 FPS vs 30), RViz 1.2, `lidar_obstacles` 0.75, move_base 0.06 | Planning computation is **not** the bottleneck |

So the stalls come from the planners giving up (a lost route, or no safe move) and then waiting
for move_base's timers, not from slow path computation.

## Goal

- **In the busy scene, no stall longer than 3 s** (the owner's number).
  - A *stall* is a continuous stretch where the robot's forward speed is below 0.02 m/s, whether
    or not it is turning on the spot (the owner's definition).
  - **Not counted (owner's decision):**
    - the first turn to face the route: from the goal being sent until the robot first drives
      forward;
    - the final turn to the goal heading: once the robot is within the goal tolerance (0.25 m)
      of the goal position;
    - **traffic waits**: while a mover's footprint is within 1.5 m of the robot's body and within
      ±45° of its heading *(the 1.5 m / ±45° rule is the builder's call)*. Traffic waits are
      reported separately (count, longest, total) so they stay visible.
  - Any other turn on the spot counts, including turning round mid-route.
- Keep today's safety (pass lines are the builder's call, stated to the owner):
  - no contacts in any run;
  - in each existing scene, the closest clearance is at most 0.05 m below today's value in the
    results table (`docs/obstacle_avoidance.md`);
  - in `busy`, the robot's colliders never come within 0.10 m of a mover.
- Implement **two candidates**, benchmark both the same way, and pick one from the numbers.

## Non-goals

- The physical robot (week 6 is dropped from the plan).
- Raising the speed limits (0.2 m/s, 0.5 rad/s) or shrinking `robot_radius` (0.25 m).
- Detection changes: no `person` class, no retraining, no tracker rework.
- Lowering the camera detector's or RViz's CPU use. The busy-scene Unity run should report the
  container CPU, but fixing it is separate work.
- Changes to the delivery flow (`origin/Ngan`).

## Anti-goals (a failure even if it gets faster)

- Getting faster by cutting clearance, so the robot brushes past carts.
- Breaking anything that sends goals through the `move_base` action: RViz 2D Nav Goal, assisted
  driving (`teleop_assist` cancels goals), Xuân Ngân's `delivery_controller`.
- Only working on the scenario bench and not in Unity.
- macOS (Docker) and Ubuntu/WSL setups drifting apart.

## Constraints

- ROS 1 Noetic, `move_base`, NavfnROS global planner (its parameters may be tuned).
- `cmd_vel_guard` stays in the chain for both candidates, unchanged, as the last-resort stop.
- The same speed, acceleration and footprint limits for both candidates, so the comparison is fair.
- **Reversing (owner's decision):** both candidates may drive backwards at up to 0.1 m/s (DWA:
  `min_vel_x: -0.1`; TEB: `max_vel_x_backwards: 0.1`). The lidar covers 360° and `cmd_vel_guard`
  still checks backward commands; the builder confirms the guard handles negative speeds and adds
  a test if it doesn't already have one.
- **Goals must not abort sooner.** `delivery_controller` (`origin/Ngan`) treats any result other
  than SUCCEEDED as a failed delivery, with no retry. The shorter waits must lead to a replan or a
  recovery, not an earlier abort, for goals the robot reaches today.
- A new apt package goes into **both** `docker/Dockerfile` and `scripts/setup_ubuntu.sh`, and is
  kept only if that candidate wins.
- Follow the repo's CLAUDE.md: update `.gitignore` for anything new the bench writes into the
  repo, and check `git status --short --ignored`.

## The two candidates

Both get the same planner-independent move_base fixes, so the comparison isolates the local
planner. *(Builder's call.)*

**Shared: shorter waits.** A lost route or no safe move must lead to action within the 3 s
budget, not after the 15–20 s `controller_patience` / `oscillation_timeout`. The builder picks
the mechanism (for example shorter patience and timeouts, or a faster replan when the local plan
comes back empty) and checks it doesn't make the robot give up on goals it reaches today.

**Shared: the lost-route stall.** Try to reproduce `Received an empty transformed plan` (the
bench normally serves a fixed empty map; a variant with `slam_toolbox` on `/scan_static` in the
loop, as the docs describe for `headon`, may be needed). Fix the cause if found. If it can't be
reproduced, the shared shorter waits must still bound it to 3 s, and the report says so. If the
cause is in `slam_toolbox.yaml` or `tf_odometry` (Đức Trí's area), the fix may change them, and
the report flags it for Đức Trí. *(Builder's call.)*

**A: DWA, improved.** Keep `dwa_local_planner`. Make moving carts block less: today every cell a
cart sweeps in the next 2 s is lethal for the whole 2 s. The builder chooses the change (for
example graded instead of lethal cost along the predicted path, a shorter horizon, or tighter
marking), within the safety goal above.

**B: TEB.** Replace DWA with `teb_local_planner` (Noetic package). Feed it the tracks from
`/obstacles` with their velocities (its `~obstacles` input, `costmap_converter/ObstacleArrayMsg`,
with `include_dynamic_obstacles: true`) so it plans around carts in time. Avoid counting carts
twice: decide whether TEB's local costmap keeps `predicted_layer`. Note that TEB's dynamic-obstacle
support is marked experimental upstream, and at least one user reports collisions with it.
Configure it as a differential drive, with the reversing limit under Constraints.

## Benchmark

Extend `catkin_ws/src/unity_slam_example/test/scenarios.py`:

- **Planner switch:** run any scene with either candidate (for example `--planner dwa|teb`).
- **New metric: longest stall**, as defined in Goal, next to the existing time, stopped, rotation,
  clearance and off-goal columns.
- **New scene `busy`:** about 10 cart-sized movers (0.6 × 1.6 m, up to 0.8 m/s) in warehouse-like
  aisles, which stop at random, turn and change direction, from a fixed seed so every run sees the
  same traffic. Today's movers only shuttle back and forth at constant speed, so this needs a new
  mover model. The builder designs the layout and motion; it should resemble the Unity roaming
  traffic (keep-right passing, pauses), not just random boxes.
- **Runs:** `busy` at least 5 runs per candidate. Every existing scene 3 runs per candidate as a
  regression check.

Then **a Unity confirmation per candidate**: 3 sessions of about 10 minutes each in
`SimpleWarehouseScene`, about 10 `WorkerWithCart` roaming, the same list of 2D Nav Goals across
the warehouse each time. Unity spawns are not seeded, and adding seeding is out of scope, so the
Unity runs confirm the bench result rather than decide it. Record the longest stall and traffic
waits (from `/odom`, the move_base result and the movers' positions; a small script is fine),
contacts, goals reached, warning counts from the ROS log, and container CPU. *(Builder's call.)*

## How the winner is chosen

1. It must meet the 3 s longest stall in every `busy` run, with the safety pass lines under Goal.
2. On the existing scenes, it must reach at least as many goals as today and meet the clearance
   pass line.
3. In Unity, no contacts. A stall over 3 s in Unity doesn't disqualify on its own, but its cause
   goes in the report, and the owner sees it before the choice is final.
4. If both qualify: shorter time to goal in `busy` wins. If that's close (within about 10 %),
   DWA wins, since it adds no dependency.
5. If neither qualifies: report the numbers and the reasons to the owner before going further.

## Deliverables

- The winning configuration and code on a branch from `main`. The losing candidate's code and
  dependency are not merged; its numbers stay in the report.
- `scenarios.py` with the planner switch, the stall metric and the `busy` scene, plus unit tests
  for any new non-trivial logic (stall measurement, mover motion).
- A results table for both candidates in `docs/obstacle_avoidance.md` (the same format as the
  current table, plus the longest stall), the Unity run results, and the reason for the choice.
- Updated docs for whatever settings changed (the settings tables in `docs/obstacle_avoidance.md`
  and `OBSTACLE_DETECTION_AND_AVOIDANCE.md`).

## Left to the builder

- The mechanism for the shorter waits, and any Navfn tuning.
- How candidate A lightens the predicted-path marking.
- TEB's parameters, and how tracks are converted to its obstacle message.
- The `busy` layout, mover model and seed.
- The Unity stall-measuring script.
