"""YOLOv8 ONNX inference: letterbox, decode, class-wise NMS. No ROS.

Ultralytics exports YOLOv8 detection as one output of shape (1, 4 + num_classes, anchors):
rows 0-3 are box centre x, y, width, height in input pixels; the rest are class scores in [0, 1].
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    class_id: int


def letterbox(image, size):
    """Resize keeping aspect ratio and pad to size x size. Returns (padded, scale, (pad_x, pad_y))."""
    h, w = image.shape[:2]
    scale = min(size / w, size / h)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    pad_x, pad_y = (size - new_w) // 2, (size - new_h) // 2
    padded = np.full((size, size, 3), 114, dtype=np.uint8)
    padded[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    return padded, scale, (pad_x, pad_y)


def preprocess(image_bgr, size):
    padded, scale, pad = letterbox(image_bgr, size)
    tensor = padded[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return np.ascontiguousarray(tensor), scale, pad


def decode(output, scale, pad, image_shape, conf_threshold=0.35, iou_threshold=0.5,
           max_detections=100):
    """Raw (1, 4+C, N) output -> detections in original image pixels, after class-wise NMS."""
    pred = output[0].T                                  # N x (4 + C)
    scores = pred[:, 4:]
    class_ids = scores.argmax(1)
    confidences = scores[np.arange(len(scores)), class_ids]
    keep = confidences >= conf_threshold
    if not keep.any():
        return []
    boxes, confidences, class_ids = pred[keep, :4], confidences[keep], class_ids[keep]

    h, w = image_shape[:2]
    cx = (boxes[:, 0] - pad[0]) / scale
    cy = (boxes[:, 1] - pad[1]) / scale
    bw, bh = boxes[:, 2] / scale, boxes[:, 3] / scale
    x1 = np.clip(cx - bw / 2, 0, w)
    y1 = np.clip(cy - bh / 2, 0, h)
    x2 = np.clip(cx + bw / 2, 0, w)
    y2 = np.clip(cy + bh / 2, 0, h)

    # Class-wise NMS in one call: offset each class far apart so boxes of different classes
    # never overlap (cv2.dnn.NMSBoxesBatched needs OpenCV >= 4.7; Ubuntu 24.04 ships 4.6).
    offset = class_ids * (max(w, h) + 1.0)
    rects = np.column_stack((x1 + offset, y1 + offset, x2 - x1, y2 - y1)).tolist()
    indices = cv2.dnn.NMSBoxes(rects, confidences.tolist(), conf_threshold, iou_threshold)
    indices = np.array(indices).reshape(-1)[:max_detections]
    return [Detection(float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i]),
                      float(confidences[i]), int(class_ids[i])) for i in indices]


class YoloDetector:
    """ONNX Runtime session; uses CUDA when onnxruntime-gpu and a GPU are available."""

    def __init__(self, model_path, threads=4):
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        available = ort.get_available_providers()
        providers = [p for p in ('CUDAExecutionProvider', 'CPUExecutionProvider') if p in available]
        self.session = ort.InferenceSession(model_path, options, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.size = int(self.session.get_inputs()[0].shape[2])
        self.provider = self.session.get_providers()[0]

    def __call__(self, image_bgr, conf_threshold=0.35, iou_threshold=0.5):
        tensor, scale, pad = preprocess(image_bgr, self.size)
        output = self.session.run(None, {self.input_name: tensor})[0]
        return decode(output, scale, pad, image_bgr.shape, conf_threshold, iou_threshold)


def load_class_names(path):
    with open(path) as f:
        return [line.strip() for line in f if line.strip()]
