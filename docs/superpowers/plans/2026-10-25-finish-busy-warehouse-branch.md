# Finish the Busy-Warehouse Branch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close out `thien/busy-warehouse-planner` with the tuned DWA as the winner. Lock in its
settings, remove TEB, confirm the result in Unity, and document everything. Leave the working
tree ready for the owner to review and commit.

**Architecture:** Configuration and deletion only, with no new logic.
- The DWA tuning goes into `move_base.yaml`, pinned by the existing config test.
- Candidate B (TEB, commit `b0bc914`) is removed. Its generic parts stay.
- `docs/obstacle_avoidance.md` gets the settings, results and the decision.
- Task 5 (SLAM lost-route stall) is documented as an open issue, not fixed.

**Tech Stack:** ROS 1 Noetic (move_base, dwa_local_planner), Python 3.8, pytest, Docker
`unity-robotics:noetic`.

**Spec:** `docs/superpowers/specs/2026-10-24-busy-warehouse-planner-design.md`
(previous plan: `docs/superpowers/plans/2026-10-24-busy-warehouse-planner.md`, Task 9. Status
and all bench numbers: `temp_docs.md`, plus outputs in
`.superpowers/sdd/2026-10-24-busy-warehouse-planner/` (`$SDD` below). Both are git-ignored.)

## Owner's decisions (2026-10-25)

- **Winner: DWA tuned** (`min_vel_x: -0.05`, `oscillation_reset_dist: 0.01`), although it doesn't
  pass every spec rule (rule 5 applies).
- **Task 5:** defer it and document it as an open issue.
- **Unity:** 3 sessions with the winner only.
- **No commits by the executor.** Never run `git commit`, `merge` or `push`, and add no Claude
  credit anywhere (no `Co-Authored-By`, no "Generated with Claude Code"). Each task ends by
  leaving its files in the working tree and listing them, with a suggested commit message the
  owner may use.

## Global Constraints

- Branch: `thien/busy-warehouse-planner`. Suggested message style: `[ADD]`, `[UPDATE]`, `[FIX]`,
  `[REMOVE]`.
- After every task, follow CLAUDE.md: check `.gitignore`, run `git status --short --ignored`, and
  confirm no new source file is ignored and nothing generated is untracked. The existing
  uncommitted `.gitignore` lines (`graphify-out/`, `.archify/`) stay as they are.
- These stay unchanged:
  - `max_vel_x` 0.2 m/s, `max_vel_theta` 0.5 rad/s, `robot_radius` 0.25 m;
  - the 2 s patience and the 22 recoveries (abort budget about 46 s);
  - `cmd_vel_guard`;
  - candidate A's graded cost (`lethal_horizon` 0.8 s, `graded_max` 80, `graded_min` 30).
- Pass lines, from the spec:
  - every `busy` run's longest stall ≤ 3 s;
  - no contacts;
  - each static scene's clearance at most 0.05 m below the October 2026 table;
  - in `busy`, never within 0.10 m of a mover.
- Bench command, from the repo root, with `SDD=.superpowers/sdd/2026-10-24-busy-warehouse-planner`:
  `bash $SDD/bench.sh '<command>'`. It runs in `unity_slam_example` inside a fresh container. `SRC`
  (absolute) picks another source tree. The default is `catkin_ws/src`.
  - Unit tests: `python3 -m pytest -q test/`.
  - Integration checks: `python3 test/check_obstacle_costmap.py`. Expect 14 `PASS:` lines.
- Bench output goes to `$SDD/` (git-ignored) or the scratchpad, never elsewhere in the repo.
- `OBSTACLE_DETECTION_AND_AVOIDANCE.md` is git-ignored local notes. Don't edit it.

## Review Focus

1. **Perception off after the launch `if` revert.** `move_base_predicted.yaml` must load only with
   `perception:=true`, and the robot must still drive without it. Task 2 runs
   `open_box --no-perception`.
2. **A fresh image without TEB.** Nothing in `catkin_ws/src` may still import or launch
   `teb_*` / `costmap_converter`. Task 2 adds a grep check that must print nothing.
3. **Stale `planner:=` in instructions.** A user following old docs passes `planner:=teb`.
   - No doc or script in the repo may still mention it. Task 2 greps for it.
   - `start-navigation.sh` keeps forwarding `name:=value` arguments generically.
4. **Delivery abort budget.** The tuning must not shorten how long a goal survives.
   `test_abort_budget_matches_today` runs in Task 1's suite unchanged.
5. **Slower reverse towards something behind the robot.** The guard must still slow it.
   `test_reversing_towards_wall_behind_slows` and integration check 12 run in Task 1.

---

### Task 0: Gate: is `aisle_tight` 1/3 noise?

The tuned DWA reached `aisle_tight` 1/3 against today's 2/3, which fails spec rule 2. Before
anything is deleted, run 10 trials of each so the docs can state the real rates. No repo changes.

**Files:** none in the repo. Outputs go to `$SDD/bench-aisle_tight-baseline-x10.txt` and
`$SDD/bench-aisle_tight-osc-x10.txt`.

- [ ] **Step 1: Run today's settings.** `SRC` must be absolute.
  Run: `SRC="$PWD/$SDD/src-baseline/catkin_ws/src" bash $SDD/bench.sh 'python3 test/scenarios.py aisle_tight --runs 10' | tee $SDD/bench-aisle_tight-baseline-x10.txt`
- [ ] **Step 2: Run the tuned DWA.**
  Run the same command with `SRC="$PWD/$SDD/src-osc/catkin_ws/src"`, and tee it to
  `$SDD/bench-aisle_tight-osc-x10.txt`.
- [ ] **Step 3: Apply the gate.** Compare the `reached` and `clearance` columns of the two summary
  rows.
  - **Pass:** tuned reached ≥ today's reached, and tuned clearance ≥ 0.09 m. Go on to Task 1.
  - **Fail:** stop. Show the owner both summary lines and wait for their call before Task 2,
    which deletes TEB. Task 1 may still go ahead.
  - Either way, record both lines for Task 4's docs.

### Task 1: Lock in the DWA tuning

**Files:**
- Modify: `catkin_ws/src/unity_slam_example/config/move_base.yaml:99` (`min_vel_x`) and `:140`
  (`oscillation_reset_dist`)
- Test: `catkin_ws/src/unity_slam_example/test/test_move_base_config.py:23-24`

**Interfaces:** Produces the final DWA values that Task 4 documents.

- [ ] **Step 1: Write the failing test.** Replace `test_both_candidates_may_reverse_slowly` with:

```python
def test_dwa_reverses_slowly_and_unlocks_quickly():
    # -0.1 let a reverse arc win beside a box and the 0.05 m oscillation lock kept it reversing
    # (aisle_narrow, October 2026); -0.05 with a 0.01 m lock passed 15/15.
    dwa = load()['DWAPlannerROS']
    assert dwa['min_vel_x'] == -0.05
    assert dwa['oscillation_reset_dist'] == 0.01
```

- [ ] **Step 2: Run it and check that it fails.**
  Run: `bash $SDD/bench.sh 'python3 -m pytest -q test/test_move_base_config.py'`
  Expected: 1 failed (`-0.1 == -0.05`).
- [ ] **Step 3: Change the config.** In `move_base.yaml`, set `min_vel_x: -0.05` (keep its
  comment, but drop "(both candidates)") and `oscillation_reset_dist: 0.01`. Add a one-line
  comment above the latter: `# release DWA's direction lock quickly so one reverse arc does not
  lock out forward motion`. The result must match `$SDD/src-osc` exactly, apart from the comment
  lines: check with
  `diff catkin_ws/src/unity_slam_example/config/move_base.yaml $SDD/src-osc/catkin_ws/src/unity_slam_example/config/move_base.yaml`.
- [ ] **Step 4: Run the full suite and the checks.**
  Run: `bash $SDD/bench.sh 'python3 -m pytest -q test/ && python3 test/check_obstacle_costmap.py'`
  Expected: 84 passed, and 14 `PASS:` lines.
- [ ] **Step 5: Hand-off, with no commit.** Leave both files modified. Suggested message:
  `[UPDATE] DWA: reverse at 0.05 m/s, 0.01 m oscillation reset (aisle_narrow 15/15)`.

### Task 2: Remove candidate B (TEB)

Delete everything TEB-only that commit `b0bc914` added. Keep the generic parts:
- the `LOG_COUNTS` tuple form;
- the `name:=value` forwarding in `start-navigation.sh`;
- the `temp_docs.md` ignore line.

**Files:**
- Delete: `catkin_ws/src/unity_slam_example/config/teb_local_planner.yaml`,
  `catkin_ws/src/unity_slam_example/nodes/teb_obstacles`,
  `catkin_ws/src/unity_slam_example/unity_slam_example/teb_obstacles.py`
- Modify:
  - `catkin_ws/src/unity_slam_example/CMakeLists.txt:16`: drop `nodes/teb_obstacles`.
  - `catkin_ws/src/unity_slam_example/package.xml:17-18`: drop `teb_local_planner` and
    `costmap_converter`.
  - `docker/Dockerfile:16`, `scripts/setup_ubuntu.sh:25`: drop `ros-noetic-teb-local-planner`.
  - `catkin_ws/src/unity_slam_example/launch/unity_slam_example.launch`:
    - drop the `planner` arg, the TEB `rosparam` and the `teb_obstacles` node;
    - restore `if="$(arg perception)"` on `move_base_predicted.yaml`;
    - restore its comment to "Predicted paths of tracked obstacles (predicted_obstacles, below)
      in the local costmap."
  - `catkin_ws/src/unity_slam_example/unity_slam_example/avoidance_math.py`: drop
    `moving_polygons`.
  - `catkin_ws/src/unity_slam_example/test/test_avoidance_math.py`: drop
    `test_moving_polygons_filters_and_keeps_velocity` and its import.
  - `catkin_ws/src/unity_slam_example/test/scenarios.py`:
    - drop `LOCAL_PLANNER`, `TEB_OBSTACLES`, `--planner`, the `planner` parameter of
      `run_trial` and the TEB strings in `LOG_COUNTS['no_path']`;
    - put `<rosparam command="load" file="{config}/move_base_predicted.yaml"/>` back in `LAUNCH`.
  - `docker/start-navigation.sh:5`: change the example to `perception:=false`.

**Interfaces:** `run_trial(name, port, rate, perception, limit, seed=0, slam=False)`, as before
`b0bc914`.

- [ ] **Step 1: Make the deletions and edits above.**
  `git show b0bc914 -- <file>` shows the exact lines to remove from each file.
- [ ] **Step 2: Check nothing TEB is left.**
  Run: `grep -rniE 'teb|costmap_converter|planner:=' catkin_ws/src/unity_slam_example docker scripts README.md docs/*.md`
  Expected: no output.
- [ ] **Step 3: Run the unit tests and the checks.**
  Run: `bash $SDD/bench.sh 'python3 -m pytest -q test/ && python3 test/check_obstacle_costmap.py'`
  Expected: 83 passed (84 minus the `moving_polygons` test), and 14 `PASS:` lines.
- [ ] **Step 4: Smoke-test both launch paths.**
  Run: `bash $SDD/bench.sh 'python3 test/scenarios.py open_box && python3 test/scenarios.py open_box --no-perception'`
  Expected: both lines `ok ... Goal reached.`
- [ ] **Step 5: Hand-off, with no commit.** Run `git status --short --ignored`. The three
  deleted files show as `D`, and nothing new is untracked. Suggested message:
  `[REMOVE] TEB candidate: DWA tuned wins the busy-warehouse benchmark`.
  - Tell the owner the existing `unity-robotics:noetic` image still contains TEB, which is
    harmless.
  - Rebuilding it is optional (see the Docker setup guide).

### Task 3: Unity confirmation, run by the owner

The executor can't run Unity. Prepare the session sheet, wait for the owner's numbers, and apply
the gate.

**Files:** `$SDD/unity-sessions.md` (git-ignored): goals, the monitor summaries, warning counts
and CPU.

- [ ] **Step 1: Write the session sheet** in `$SDD/unity-sessions.md`, and ask the owner to:
  - pick 4 goals across `SimpleWarehouseScene`, and write them on the sheet;
  - for each of 3 sessions of about 10 minutes:
    1. start the stack: `bash docker/start-navigation.sh`;
    2. press Play, then spawn about 10 `WorkerWithCart` (B to choose, then T);
    3. in a ROS shell, start
       `python3 ~/catkin_ws/src/unity_slam_example/test/unity_stall_monitor.py`;
    4. send the 4 goals;
    5. note `docker stats` CPU;
    6. Ctrl+C the monitor and paste its summary into the sheet;
    7. count the warnings:
       `docker exec unity-nav2 grep -c -e 'empty transformed plan' -e 'failed to produce path' -e 'missed its desired rate' /tmp/unity-navigation.log`.
- [ ] **Step 2: Apply the gate** to the 3 summaries.
  - Any contact (body clearance ≤ 0): stop and show the owner.
  - A stall over 3 s: write down its cause from the log and the monitor line, then show the
    owner before Task 4 calls the result final.
  - Otherwise, go on to Task 4.

### Task 4: Docs: settings, results, decision, open issue

**Files:**
- Modify: `docs/obstacle_avoidance.md`:
  - `## Settings you own`, lines 70-106;
  - `## Moving and labelled obstacles`, lines 166-230;
  - `## Automated ROS checks`, lines 458-495;
  - `### Scenario trials`, lines 497-540.

**Interfaces:** consumes the Task 0 lines, Task 1's values and the Task 3 sheet.
- Numbers come only from `temp_docs.md` "Results" and "Recheck", and from `$SDD/bench-*.txt`.
  Copy them; don't round them differently.

- [ ] **Step 1: Settings table.** Add rows:
  - `planner_patience` / `controller_patience` 2 s;
  - `oscillation_timeout` 10 s;
  - recoveries: 22 clears, about 46 s before an abort;
  - `min_vel_x` −0.05 m/s;
  - `oscillation_reset_dist` 0.01 m.

  Update the existing recovery row. Rewrite the paragraph at lines 96-101, which says
  `min_vel_x: 0`, to say DWA may back away at up to 0.05 m/s.
- [ ] **Step 2: Moving obstacles.**
  - Swept cells are lethal for the first 0.8 s, then graded from 80 down to 30, a cost DWA may
    cross.
  - Add `lethal_horizon`, `graded_max` and `graded_min` to the `predicted_obstacles` table.
  - Fix the limits at lines 222-229: DWA now backs away slowly. Keep `sweeper` and `overtaken`
    as known limits: both still reach 0.00 m.
- [ ] **Step 3: Automated checks.** Check 12 now "holds short of the wall (it may back off and
  follow it)", not "stops". Update both the prose and the "twelfth reports" sentence.
- [ ] **Step 4: Scenario trials.**
  - Add `busy` to the scene table, with the new options `--seed`, `--slam` and `--runs`, and the
    columns stall, wait, movers, empty plan, no path and recoveries.
  - Add a dated results table for the tuned DWA, from the `bench-osc-scenes.txt` summary. Add
    the Task 0 `aisle_tight` ×10 lines under it.
- [ ] **Step 5: New subsection "Busy warehouse: choosing the local planner".**
  - The `busy` table: today's settings, shared fixes only, A, and TEB (from `temp_docs.md`).
  - Then the tuned A, all 20 runs: seeds 0-4 twice and seeds 5-14.
  - State that 1 of the 20 runs had a 3.8 s stall, and that the shared fixes are the 2 s
    patience and the 22 recoveries.
  - Why DWA:
    - TEB cut static clearance by 0.10-0.15 m and reached `aisle_tight` 0/3;
    - the tuned DWA is about 10-30 % slower in `busy` but adds no dependency;
    - the reverse-lock trace (the 4 points under "Why A reverses at the box") explains
      −0.05 / 0.01.
  - The Unity session results from Task 3, including container CPU.
- [ ] **Step 6: Open issue, lost route with SLAM.** Two or three sentences:
  - `busy --slam` reached 1/3. Run 2 logged 173 "empty transformed plan" warnings, an 8.3 s
    stall, then an abort. Run 1 failed at once with "Failed to find a valid plan".
  - This was before the Task 1 tuning, and the cause is unconfirmed. The suspect is a jump in
    `map → odom`.
  - Repro: `python3 test/scenarios.py busy --slam --runs 3`.
  - Tracked separately. If the cause is in `slam_toolbox.yaml` or `tf_odometry`, it goes to
    Đức Trí.
- [ ] **Step 7: Check the doc.** Run `grep -nE 'min_vel_x: 0|TEB|teb' docs/obstacle_avoidance.md`.
  - Expected: matches only inside the new subsection's TEB comparison.
  - Every number in the new tables appears in a `$SDD/bench-*.txt` file or in `temp_docs.md`.
- [ ] **Step 8: Hand-off, with no commit.** Suggested message:
  `[UPDATE] obstacle avoidance docs: busy-warehouse results, DWA chosen, SLAM lost-route open`.

### Task 5: Final check and whole-branch review

- [ ] **Step 1: Run everything once more.**
  Run: `bash $SDD/bench.sh 'python3 -m pytest -q test/ && python3 test/check_obstacle_costmap.py'`
  Expected: 83 passed, and 14 `PASS:` lines.
- [ ] **Step 2: Check the tree.** Run `git status --short --ignored`.
  - Only this plan's files are changed, plus the earlier `.gitignore` lines.
  - This plan file is untracked, for the owner to commit or drop.
  - Nothing generated is untracked, and no source file is ignored.
- [ ] **Step 3: Whole-branch review.** Dispatch one fresh reviewer on `main...` plus the working
  tree, with the spec and this plan. Fix what it confirms, and rerun Step 1.
- [ ] **Step 4: Report to the owner.**
  - The files changed per task, with the suggested commit messages.
  - Test output.
  - What's still open: Task 5 (SLAM), the one 3.8 s `busy` run, and anything from the review.
  - Merge or PR is the owner's call. The executor doesn't run either.
