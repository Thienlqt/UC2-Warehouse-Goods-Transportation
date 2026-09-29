#!/usr/bin/env bash
# Fine-tune YOLOv8n (COCO weights) on the Unity warehouse dataset and install it as the ROS detector.
#   1. Unity: Robotics > Detection Dataset > Capture Full Dataset  (writes detection_training/dataset)
#   2. bash detection_training/train.sh [epochs]
# Uses the Apple GPU (MPS) or CUDA when available. Installs the model into the ROS package
# (catkin_ws/src/unity_slam_example/models); commit it, then restart the ROS stack to load it.
set -euo pipefail

here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(dirname "$here")"
epochs="${1:-60}"
venv="$here/.venv"
data="$here/dataset/data.yaml"
models="$repo/catkin_ws/src/unity_slam_example/models"

[ -f "$data" ] || { echo "No $data: capture the dataset in Unity first." >&2; exit 1; }
if [ ! -x "$venv/bin/yolo" ]; then
    python="${PYTHON:-$(command -v python3.13 || command -v python3)}"
    "$python" -m venv "$venv"
    "$venv/bin/pip" install -r "$here/requirements.txt"
fi
device="$("$venv/bin/python" -c "import torch
print('mps' if torch.backends.mps.is_available() else 0 if torch.cuda.is_available() else 'cpu')")"

cd "$here"  # yolov8n.pt is downloaded here
"$venv/bin/yolo" detect train model=yolov8n.pt data="$data" imgsz=416 epochs="$epochs" batch=32 \
    device="$device" patience=15 project="$here/runs" name=warehouse exist_ok=True
best="$here/runs/warehouse/weights/best.pt"
"$venv/bin/yolo" export model="$best" format=onnx imgsz=416 opset=12 simplify=True

mkdir -p "$models"
cp "${best%.pt}.onnx" "$models/detector.onnx"
cp "$here/dataset/classes.txt" "$models/classes.txt"
echo "Installed $models/detector.onnx ($(tr '\n' ' ' < "$models/classes.txt"))"
