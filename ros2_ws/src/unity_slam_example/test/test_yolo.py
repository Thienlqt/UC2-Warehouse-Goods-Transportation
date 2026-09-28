import numpy as np

from unity_slam_example.perception.yolo import decode, letterbox


def raw_output(entries, num_classes=3, anchors=50):
    """Build a (1, 4+C, N) YOLOv8 tensor with given (cx, cy, w, h, class, score) entries."""
    out = np.zeros((1, 4 + num_classes, anchors), dtype=np.float32)
    for i, (cx, cy, w, h, cls, score) in enumerate(entries):
        out[0, :4, i] = (cx, cy, w, h)
        out[0, 4 + cls, i] = score
    return out


def test_letterbox_pads_640x480_to_square():
    padded, scale, pad = letterbox(np.zeros((480, 640, 3), np.uint8), 416)
    assert padded.shape == (416, 416, 3)
    assert abs(scale - 0.65) < 1e-6 and pad == (0, 52)


def test_decode_maps_back_to_image_pixels_and_applies_nms():
    scale, pad = 0.65, (0, 52)
    # Box (100..300, 100..200) in the 640x480 image, in letterboxed input coordinates.
    cx, cy, w, h = 200 * scale, 150 * scale + 52, 200 * scale, 100 * scale
    out = raw_output([
        (cx, cy, w, h, 1, 0.9),
        (cx + 2, cy, w, h, 1, 0.8),          # duplicate: suppressed by NMS
        (cx, cy, w, h, 2, 0.7),              # same place, other class: kept (class-wise NMS)
        (50, 50, 20, 20, 0, 0.1),            # below confidence threshold
    ])
    dets = decode(out, scale, pad, (480, 640, 3), conf_threshold=0.35)
    assert sorted((d.class_id, round(d.score, 1)) for d in dets) == [(1, 0.9), (2, 0.7)]
    best = max(dets, key=lambda d: d.score)
    assert np.allclose([best.x1, best.y1, best.x2, best.y2], [100, 100, 300, 200], atol=0.5)


def test_decode_returns_empty_when_nothing_is_confident():
    assert decode(raw_output([(10, 10, 5, 5, 0, 0.2)]), 1.0, (0, 0), (416, 416, 3)) == []
