import math

import numpy as np

from unity_slam_example.perception.lidar_segmentation import detect_boxes, fit_box


def rectangle(cx, cy, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    half = [(length / 2, width / 2), (-length / 2, width / 2),
            (-length / 2, -width / 2), (length / 2, -width / 2)]
    return [(cx + c * a - s * b, cy + s * a + c * b) for a, b in half]


def raycast(polygons, beams=360, max_range=10.0):
    """Ideal 360 degree scan from the origin; inf where nothing is hit."""
    ranges = []
    for i in range(beams):
        ang = 2 * math.pi * i / beams
        d = np.array([math.cos(ang), math.sin(ang)])
        best = math.inf
        for poly in polygons:
            for p, q in zip(poly, poly[1:] + poly[:1]):
                p, q = np.array(p), np.array(q)
                edge = q - p
                den = d[0] * -edge[1] + d[1] * edge[0]
                if abs(den) < 1e-12:
                    continue
                t = (p[0] * -edge[1] + p[1] * edge[0]) / den
                u = (d[0] * p[1] - d[1] * p[0]) / den
                if t > 0 and 0 <= u <= 1:
                    best = min(best, t)
        ranges.append(best if best <= max_range else math.inf)
    return ranges


def yaw_error(a, b):
    """Rectangle heading error modulo 90 degrees."""
    d = (a - b) % (math.pi / 2)
    return min(d, math.pi / 2 - d)


def test_fit_recovers_rotated_rectangle_from_two_visible_faces():
    # Bearing to the box is ~14 degrees; a 60 degree heading points a corner at the sensor.
    ranges = raycast([rectangle(2.0, 0.5, math.radians(60), 0.8, 0.5)])
    boxes = detect_boxes(ranges, 0.0, math.radians(1), 0.12, 10.0)
    assert len(boxes) == 1
    box = boxes[0]
    assert yaw_error(box.yaw, math.radians(60)) < math.radians(2)
    assert math.hypot(box.x - 2.0, box.y - 0.5) < 0.02
    assert abs(max(box.length, box.width) - 0.8) < 0.02
    assert abs(min(box.length, box.width) - 0.5) < 0.02


def test_single_visible_face_gives_accurate_heading():
    ranges = raycast([rectangle(2.0, 0.5, math.radians(25), 0.8, 0.5)])
    box = detect_boxes(ranges, 0.0, math.radians(1), 0.12, 10.0)[0]
    assert yaw_error(box.yaw, math.radians(25)) < math.radians(1)


def test_fit_box_exact_on_full_outline():
    pts = []
    for (px, py), (qx, qy) in zip(*(lambda r: (r, r[1:] + r[:1]))(rectangle(1, -1, 0.3, 1.0, 0.4))):
        for t in np.linspace(0, 1, 20, endpoint=False):
            pts.append((px + t * (qx - px), py + t * (qy - py)))
    box = fit_box(np.array(pts))
    assert yaw_error(box.yaw, 0.3) < math.radians(2)
    assert abs(box.x - 1) < 0.02 and abs(box.y + 1) < 0.02
    assert abs(max(box.length, box.width) - 1.0) < 0.02
    assert abs(min(box.length, box.width) - 0.4) < 0.02


def test_two_separated_objects_are_two_segments():
    ranges = raycast([rectangle(2.0, 1.0, 0.0, 0.4, 0.4), rectangle(2.0, -1.0, 0.0, 0.4, 0.4)])
    boxes = detect_boxes(ranges, 0.0, math.radians(1), 0.12, 10.0)
    assert len(boxes) == 2
    assert sorted(round(b.y) for b in boxes) == [-1, 1]


def test_wall_is_dropped_and_object_wrapping_zero_degrees_stays_whole():
    wall = rectangle(0.0, 3.0, 0.0, 12.0, 0.1)          # long wall on the left
    ahead = rectangle(1.5, 0.0, 0.0, 0.3, 0.6)          # straddles the 0/360 degree seam
    boxes = detect_boxes(raycast([wall, ahead]), 0.0, math.radians(1), 0.12, 10.0)
    assert len(boxes) == 1
    assert abs(boxes[0].x - 1.5) < 0.2 and abs(boxes[0].y) < 0.1
