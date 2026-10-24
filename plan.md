# UC2 Warehouse Goods Transportation: 8-week plan

## Team

| Member | Area |
|---|---|
| 1 · Đức Trí | Mapping & Localization |
| 2 · Tài | Path Planning & Control |
| 3 · Xuân Ngân | Delivery Control |
| 4 · Thiện | Obstacle Avoidance |

## Overview

| Week | Theme | Goal |
|---|---|---|
| 1 | Setup and Planning | Prepare the software and understand the robot |
| 2 | Factory Environment (Unity pivot) | Build the environment where the robot will operate |
| 3 | Basic Robot Navigation | Make the robot travel automatically between locations |
| 4 | Obstacle Avoidance | Make the robot safely avoid obstacles |
| 5 | Complete Delivery System | Complete the whole material-delivery process |
| 6 | ~~Physical Robot Testing~~ | Dropped: the project stays in simulation |
| 7 | Testing and Evaluation | Measure how well the system works, together as a team |
| 8 | Final Improvements and Demonstration | Prepare the final project demonstration |

Each task below lists what to do, the **Check** that shows it is done, and what it
**Depends on**.

---

## Week 1: Setup and Planning (completed)

**Goal:** Prepare the software and understand the robot.

- **Đức Trí (Mapping & Localization):** Set up basic ROS 1 Noetic. Learned ROS 1's packages,
  nodes, topics, Publisher, and Subscriber models.
- **Tài (Path Planning & Control):** Set up the basic Gazebo simulation environment for early
  testing. Researched a way to implement the ROS 1 Navigation Stack using move_base in the
  project.
  - Note: as instructed, the simulation environment pivots to Unity from here on.
- **Xuân Ngân (Delivery Control):** Designed the delivery workflow and defined the robot task
  states. Defined the ROS topic/service communication interface between Delivery Control and
  Navigation.
- **Thiện (Obstacle Avoidance):** Researched obstacle detection methods using LiDAR data.
  Researched obstacle avoidance methods compatible with the ROS 1 Navigation Stack.

## Week 2: Create the Factory Environment (Unity pivot)

**Goal:** Build the environment where the robot will operate.

### Đức Trí (Mapping & Localization)
- Learn ROS 1 mapping and localization basics using tools such as gmapping, map_server, and AMCL.
- Check robot sensors such as LiDAR and odometry.
- Build out the full factory environment in Unity (walls, stations, obstacle placeholders).
- Create a map; define Storage, Station A, Station B, and Station C.

### Tài (Path Planning & Control)
- Learn ROS 1 move_base path planning basics and review robot motion control concepts.
- Set up initial move_base, global costmap, and local costmap parameters for the new map.
- **Depends on:** Đức Trí's map.

### Xuân Ngân (Delivery Control)
- Create a simple ROS node for the delivery request queue.
- **Check:** submit several requests through the node and confirm they queue and dequeue in the
  correct order.

### Thiện (Obstacle Avoidance)
- Define safety distance thresholds.
- Implement basic obstacle detection using `/scan` LiDAR data.
- **Check:** the detector correctly flags a single simulated obstacle.

## Week 3: Basic Robot Navigation

**Goal:** Make the robot travel automatically between locations.

### Đức Trí (Mapping & Localization)
- Finalize map integration using map_server and AMCL.
- **Check:** while the robot travels, compare the AMCL-estimated position with the robot's actual
  position in Unity.

### Tài (Path Planning & Control)
- Configure the ROS 1 Navigation Stack and move_base.
- **Check:** send navigation goals from Storage to each station and confirm the robot reaches them
  automatically.
- **Depends on:** Đức Trí's finalized map and localization.

### Xuân Ngân (Delivery Control)
- Connect delivery requests to move_base navigation goals.
- **Check:** trigger a request and confirm the robot automatically navigates to the correct
  station.
- **Depends on:** Tài's working move_base navigation.

### Thiện (Obstacle Avoidance)
- Integrate LiDAR obstacle detection with the costmap_2d obstacle and inflation layers.
- **Check:** a detected obstacle appears correctly inflated on the local costmap in RViz.

## Week 4: Obstacle Avoidance

**Goal:** Make the robot safely avoid obstacles.

### Đức Trí (Mapping & Localization)
- Support obstacle mapping by ensuring sensor data correctly updates the global and local
  costmaps.
- **Check:** place a temporary obstacle and confirm it appears on the costmap in RViz.

### Tài (Path Planning & Control)
- Configure the global planner and local planner used by move_base for obstacle avoidance.
- **Check:** run the "box in the path" example and confirm a valid alternate path is generated.
- **Depends on:** Đức Trí's updated costmap.

### Xuân Ngân (Delivery Control)
- Handle move_base navigation success and failure states.
- Log robot status such as Moving, Arrived, Obstacle detected, or Navigation failed through ROS
  console/log output.
- **Depends on:** Tài's working move_base navigation.

### Thiện (Obstacle Avoidance)
- Configure and improve local obstacle-avoidance behavior using the ROS 1 local planner and
  costmap parameters.
- **Check:** run the "box in the path" example and confirm the robot detects the obstacle, avoids
  it, and continues to its goal.
- **Depends on:** Tài's global/local planner configuration.

## Week 5: Complete Delivery System

**Goal:** Complete the whole material-delivery process.

### Đức Trí (Mapping & Localization)
- Improve AMCL localization accuracy by tuning localization and sensor parameters.
- **Check:** re-run the Week 3 localization test and confirm the position error has decreased.

### Tài (Path Planning & Control)
- Improve global and local path planning and replanning accuracy.
- **Check:** re-run the Week 4 obstacle test on a tighter obstacle course and confirm move_base
  finds a valid path.

### Xuân Ngân (Delivery Control)
- Complete the delivery state machine.
- **Check:** run one full delivery cycle end-to-end (request, load, travel, unload, return) in
  simulation.
- **Depends on:** Đức Trí's improved localization and Tài's improved path planning.

### Thiện (Obstacle Avoidance)
- Improve local planner and costmap parameters for tighter scenarios such as narrow corridors or
  multiple obstacles.
- **Check:** re-run the avoidance test on a narrower course with two obstacles and confirm both
  are avoided.
- **Depends on:** Tài's working path planning and Xuân Ngân's completed delivery workflow.

## Week 7: Testing and Evaluation

**Goal:** Measure how well the system works, together as a team.

| Member | Logs across all test runs |
|---|---|
| Đức Trí | AMCL localization accuracy |
| Tài | move_base navigation success rate and travel time |
| Xuân Ngân | Delivery success rate and correct-station rate |
| Thiện | Obstacle-avoidance success rate and any near-misses or failures |

**Together:** run full end-to-end scenarios (normal delivery, obstacle in front, blocked route)
and combine the individual logs into one shared evaluation report.

## Week 8: Final Improvements and Demonstration

**Goal:** Prepare the final project demonstration.

| Member | Finalize | Check |
|---|---|---|
| Đức Trí | Mapping and AMCL localization parameters | Regression test: localization stays stable across all Week 7 scenarios |
| Tài | move_base, global/local planner, and costmap configuration | Regression test: path planning and replanning are stable |
| Xuân Ngân | ROS 1 delivery-control software; consolidate test results | Regression test: the state machine handles all Week 7 cases without errors |
| Thiện | Obstacle-avoidance logic, local planner settings, and safety parameters | Final dry-run of the obstacle-avoidance sequence |

**Together:** run the complete final demonstration: delivery request, station navigation, obstacle
avoidance, delivery completion, and return to Storage.
