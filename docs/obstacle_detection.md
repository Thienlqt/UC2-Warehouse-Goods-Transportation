# Obstacle detection: camera + lidar boxes, labels and confidence

move_base avoids obstacles from the raw `/scan` ([obstacle_avoidance.md](obstacle_avoidance.md)).
This layer adds **perception**: oriented boxes around obstacles with a **class name** and a
**confidence score**, from the robot's camera and 2D lidar. Navigation also uses the tracks on
`/obstacles`: their predicted paths go into the local costmap, and `cmd_vel_guard` brakes for
moving ones ([Moving and labelled obstacles](obstacle_avoidance.md#moving-and-labelled-obstacles)).

```text
Unity camera --JPEG 30 Hz--> camera_detector (YOLOv8n, ONNX Runtime) --> /detections_2d
                                         \--> /detections_image (boxes + "person 0.91")
Unity lidar  --/scan 10 Hz--> lidar_obstacles (segments -> L-shape boxes -> Kalman tracks)
                                         \--> /obstacles_lidar (class "obstacle")
/detections_2d + /obstacles_lidar + TF + camera_info --> obstacle_fusion
                                         \--> /obstacles (named, scored 3D boxes)
                                         \--> /obstacles_markers (RViz boxes + labels)
/obstacles --> predicted_obstacles --> /obstacles_predicted (predicted paths, local costmap)
           \--> cmd_vel_guard (brakes for moving tracks)
```

The detection topics use `uc2_vision_msgs` (`catkin_ws/src/uc2_vision_msgs`): the
`vision_msgs` 4.x layout with string class ids and track ids, which Noetic's own `vision_msgs`
lacks.

| Stage | Algorithm | Why |
|---|---|---|
| Camera | YOLOv8n, 416 px input, ONNX Runtime CPU (CUDA if available) | One-stage detector that outputs boxes, classes and scores directly; nano size runs in real time without a GPU |
| LiDAR segmentation | Adaptive breakpoint detection (Borges & Aldon) | Splits objects with a range-dependent gap threshold, robust as points spread out with distance |
| LiDAR boxes | L-shape fitting (Zhang et al. 2017), with a least-squares refinement | Oriented rectangle even when only one or two faces are visible |
| Tracking | Constant-velocity Kalman filter + Hungarian matching, in `odom` | Stable IDs and velocities; robot motion is not mistaken for object motion |
| Fusion | Late fusion: project lidar boxes into the image, match by horizontal (bearing) IoU | A 2D scan has no height, so only the horizontal extent is compared |

Confidence scores:
- **Camera:** the YOLO class score.
- **LiDAR only** (class `obstacle`): 0.4 × point count + 0.2 × rectangle fit + 0.4 × persistence, each in [0, 1].
- **Fused:** camera score × (0.5 + 0.5 × overlap), where overlap is the bearing IoU. Several lidar
  clusters inside one camera box (a person's legs) all receive its label.
- Tracks outside the camera's view stay `obstacle` with the lidar confidence. A label persists for
  1 s without re-detection.

## Setup

### 1. Model

The trained model is committed in `catkin_ws/src/unity_slam_example/models/` (`detector.onnx`
and `classes.txt`, about 12 MB) and installed with the package, so a fresh clone detects out of
the box on every OS. Retrain only when the scene or the classes change, as below. The model is
YOLOv8n fine-tuned with Ultralytics, so it is licensed under AGPL-3.0
([models/README.md](../catkin_ws/src/unity_slam_example/models/README.md)); the rest of the
repository is Apache-2.0.

The committed model is the best epoch (15) of a 60-epoch run that stopped at epoch 16. On the
900 validation frames it scores precision 0.91, recall 0.77, mAP50 0.84, mAP50-95 0.64.
Running the full training (below) should improve it.

**Warehouse model (classes `box`, `shelf`, `station`).** This is YOLOv8n fine-tuned from COCO
weights on synthetic frames of this warehouse, rendered in Unity through a camera that matches the
robot's. The COCO model is not an option here: it has no box, shelf or station class.
It scores the scene's pallet boxes at 0.03 and misreads a close pallet as `bench`.

1. Unity, with `SimpleWarehouseScene` open and not playing: **Robotics → Detection Dataset →
   Capture Full Dataset**. Unity enters Play, captures 9,000 frames (about 3 min), exits Play, and
   writes `detection_training/dataset/`.
   - **Capture Preview (100 frames)** writes `dataset_preview/` so you can check quickly with
     `detection_training/.venv/bin/python detection_training/preview_labels.py detection_training/dataset_preview`.
   - Each frame renders RGB plus an object-ID pass, taken from the warehouse's Perception
     `Labeling` components. A box is the visible extent of its object, so hidden parts are
     excluded, and objects less than 12% visible are dropped.
   - Every 10 frames the layout changes: 4 to 19 extra floor boxes, a quarter of the scene's boxes
     hidden, and new light intensity and tint. The camera is 0.1 to 0.2 m up, level ±4°, and 0.3
     to 14 m from a target object (80% of frames) or looking anywhere.
   - The train/val split is by layout, 90/10.
   - Code: `UnityProject/Assets/DetectionDataset/DatasetCapture.cs`.
2. `bash detection_training/train.sh [epochs, default 60]` creates `detection_training/.venv`
   (Python 3.13 if installed, else `python3`; Ultralytics). It trains at 416 px on the Apple GPU
   (MPS), CUDA (for example an NVIDIA GPU under WSL) or the CPU, and exports ONNX. It then
   replaces `catkin_ws/src/unity_slam_example/models/detector.onnx` and `classes.txt`.
   Commit both files, then restart the ROS stack to load the new model.

`person` returns when pedestrians are added to the warehouse. Append it to `Classes` in
`DatasetCapture.cs`, recapture and retrain.

Without a model, `camera_detector` logs an error and the lidar boxes still run.

### 2. Camera on the robot

The robot camera is already in `SimpleWarehouseScene`: `camera_link` under
`turtlebot3_manual_config/base_footprint/base_link`, at the TurtleBot3 Waffle's RealSense mount
(0.065, 0.094, 0.064), with **Ros Camera Sensor** at 640×480, 42° vertical FOV, 30 Hz, publishing
`/camera/image_raw/compressed`. The launch file publishes the matching TF. The sensor renders into
its own texture, so the Game view is unaffected.

### 3. Run

```bash
bash docker/start-navigation.sh   # macOS (Docker); RViz at http://localhost:6080
bash scripts/run_ros.sh           # Ubuntu / WSL; RViz opens as a window (WSLg)
```

Press Play in Unity. RViz shows:
- **Detections (camera)**: the camera image with YOLO boxes, labels and scores.
- **Obstacles (fused)**: 3D boxes around lidar obstacles, coloured by class, with text such as
  `person 0.87 · 2.4 m` (grey boxes are unlabelled `obstacle`).

### 4. In Unity: boxes and first-person driving

The robot prefab carries two more components (on `TurtleBot3ManualConfig`, next to `AGVController`):

- **Detection Overlay** subscribes to `/detections_2d`. It draws a picture-in-picture of the robot
  camera, top right, with the YOLO boxes, labels and scores (**C** toggles it). The panel says
  `waiting for /detections_2d` until the first message arrives. It shows
  `no detections received recently` if messages stop, for example after the ROS stack is stopped.
- **First Person Drive**: **P** switches to driving. The Game view then renders from the robot
  camera's pose and vertical FOV, with the robot hidden from that view only. **W/S** (or Up/Down)
  drive and **A/D** (or Left/Right) turn. Driving is **assisted** by default: the keys go to ROS,
  which holds your heading and steers around obstacles within 10 m
  ([assisted driving](obstacle_avoidance.md#assisted-driving-hold-w-and-let-it-steer)). **O**
  switches to direct driving, which ignores ROS. The free camera is paused while driving. The boxes are drawn full screen, and a faint frame marks
  the 4:3 region the detector sees, because a wider Game view shows more at the sides.

Click the Game view first so it has keyboard focus. Boxes arrive one inference later (~0.1 s on
CPU), so they trail fast turns slightly.

Turn perception off with `bash scripts/run_ros.sh perception:=false` (Ubuntu / WSL). Avoidance
then falls back to the scan alone.
Tuning lives in `catkin_ws/src/unity_slam_example/config/perception.yaml`.

## Checks

Open a ROS shell: on macOS `docker exec -it -u ubuntu -e HOME=/home/ubuntu unity-nav2 bash`;
on Ubuntu / WSL a new terminal with `source /opt/ros/noetic/setup.bash && source ~/uc2_ws/devel/setup.bash`.

```bash
rostopic hz /camera/image_raw/compressed   # target 30
rostopic hz /detections_2d                 # target 30
```

`camera_detector` logs its FPS and per-stage timings every 5 s: in the `run_ros.sh` terminal, or
on macOS with `docker exec unity-nav2 grep FPS /tmp/unity-navigation.log | tail -3`.

Unit tests (segmentation, box fitting, tracking, YOLO decoding, fusion geometry):

```bash
cd catkin_ws/src/unity_slam_example && python3 -m pytest -q test/
```

## Limits

- The 2D lidar sees only its scan plane (~0.13 m above the base). Anything entirely above or below
  it is invisible to the lidar and gets no 3D box; the camera may still detect it in 2D.
- Box height is not measured; 3D boxes use a fixed 0.5 m height for display.
- The warehouse model has no `person` class yet; pedestrians and a retrain are the next milestone.
- It is trained only on this warehouse's rendered assets. Validation frames come from new layouts of
  the same assets, so the scores measure this simulation, not real cameras or other warehouses.

## References

- Borges, G. A., & Aldon, M.-J. (2004). Line extraction in 2D range images for mobile robotics.
  *Journal of Intelligent and Robotic Systems*, 40(3), 267–297. (Adaptive breakpoint detection.)
- Zhang, X., Xu, W., Dong, C., & Dolan, J. M. (2017). Efficient L-shape fitting for vehicle
  detection using laser scanners. *IEEE Intelligent Vehicles Symposium (IV)*, 54–59.
- Jocher, G., Chaurasia, A., & Qiu, J. (2023). *Ultralytics YOLOv8* (Version 8.0.0) [Computer
  software]. AGPL-3.0. https://github.com/ultralytics/ultralytics
- Lin, T.-Y., et al. (2014). Microsoft COCO: Common objects in context. *ECCV 2014*. (Pretraining
  data of the YOLOv8n weights the detector is fine-tuned from.)
