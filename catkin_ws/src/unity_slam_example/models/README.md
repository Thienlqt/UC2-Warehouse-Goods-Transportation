# Detector model

`detector.onnx` and `classes.txt` (`box`, `shelf`, `station`) are licensed under the
**GNU Affero General Public License v3.0** ([LICENSE](LICENSE)), not the Apache-2.0 licence of the
rest of this repository.

The model is Ultralytics YOLOv8n, pretrained on COCO, fine-tuned with the Ultralytics package on
frames rendered from this project's Unity warehouse. Ultralytics distributes YOLOv8 under AGPL-3.0,
so the fine-tuned model is too.

Source for rebuilding it, all in this repository:

- Dataset capture: `UnityProject/Assets/DetectionDataset/DatasetCapture.cs`
- Training and ONNX export: `detection_training/train.sh`

Steps and validation scores: [docs/obstacle_detection.md](../../../../docs/obstacle_detection.md).

The code that runs the model (`unity_slam_example/perception/yolo.py`, ONNX Runtime) does not use
Ultralytics and stays under Apache-2.0.
