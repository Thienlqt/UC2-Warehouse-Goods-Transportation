"""Late camera-lidar fusion geometry. Pure numpy/scipy, no ROS.

A 2D scan gives no object height, so association uses only the horizontal image extent:
each lidar box is projected into the image and matched to camera boxes by 1-D (bearing) IoU.
"""

import math

import numpy as np
from scipy.optimize import linear_sum_assignment


def box_corners(cx, cy, yaw, length, width, z_min, z_max):
    """8x3 corners of an oriented box in its (fixed) frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for a in (-length / 2, length / 2):
        for b in (-width / 2, width / 2):
            for z in (z_min, z_max):
                pts.append((cx + c * a - s * b, cy + s * a + c * b, z))
    return np.array(pts)


def project_interval(corners_cam, fx, cx, image_width, min_depth=0.1):
    """Horizontal pixel interval (u_min, u_max, depth) of camera-frame corners, or None.

    Camera optical frame: z forward, x right. Corners behind the camera are dropped; an interval
    fully outside the image is None.
    """
    front = corners_cam[corners_cam[:, 2] > min_depth]
    if len(front) == 0:
        return None
    u = fx * front[:, 0] / front[:, 2] + cx
    u_min, u_max = max(0.0, float(u.min())), min(float(image_width), float(u.max()))
    if u_max <= u_min:
        return None
    return u_min, u_max, float(front[:, 2].min())


def iou_1d(a, b):
    inter = max(0.0, min(a[1], b[1]) - max(a[0], b[0]))
    union = max(a[1], b[1]) - min(a[0], b[0])
    return inter / union if union > 0 else 0.0


def associate(track_intervals, camera_boxes, iou_gate=0.3, merge_coverage=0.6):
    """Match lidar tracks to camera boxes.

    track_intervals: {track_id: (u_min, u_max)}; camera_boxes: [(u_min, u_max, label, score)].
    Returns {track_id: (label, score, overlap)}. After one-to-one Hungarian matching on 1 - IoU,
    unmatched tracks lying mostly inside a matched camera box inherit its label (e.g. the two
    leg clusters of one person).
    """
    ids = list(track_intervals)
    if not ids or not camera_boxes:
        return {}
    iou = np.array([[iou_1d(track_intervals[t], box[:2]) for box in camera_boxes] for t in ids])
    result, used = {}, set()
    for ti, bi in zip(*linear_sum_assignment(1.0 - iou)):
        if iou[ti, bi] >= iou_gate:
            label, score = camera_boxes[bi][2:]
            result[ids[ti]] = (label, score, float(iou[ti, bi]))
            used.add(bi)
    for t in ids:
        if t in result:
            continue
        lo, hi = track_intervals[t]
        for bi in used:
            b_lo, b_hi, label, score = camera_boxes[bi]
            coverage = max(0.0, min(hi, b_hi) - max(lo, b_lo)) / max(hi - lo, 1e-6)
            if coverage >= merge_coverage:
                result[t] = (label, score, coverage)
                break
    return result


def fused_score(camera_score, overlap):
    """Camera confidence discounted by how well the lidar and camera extents agree."""
    return round(camera_score * (0.5 + 0.5 * overlap), 3)
