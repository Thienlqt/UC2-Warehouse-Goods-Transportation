import math

import numpy as np

from test_lidar_segmentation import raycast, rectangle
from unity_slam_example.perception.lidar_segmentation import Box, detect_boxes
from unity_slam_example.perception.tracking import (boxes_to_frame, mask_returns,
                                                    scan_motion_evidence, Tracker)


def box(x, y):
    return Box(x, y, 0.0, 0.4, 0.4, 12, 0.005)


def test_moving_object_keeps_id_and_estimates_velocity():
    tracker = Tracker()
    for k in range(20):                      # 10 Hz, moving +x at 0.8 m/s
        tracks = tracker.step([box(1.0 + 0.08 * k, 0.5)], stamp=0.1 * k)
    assert len(tracks) == 1
    assert tracks[0].id == 1
    assert abs(tracks[0].x[2] - 0.8) < 0.1 and abs(tracks[0].x[3]) < 0.1
    assert tracks[0].confidence() > 0.8


def test_new_track_needs_two_hits_and_is_dropped_after_misses():
    tracker = Tracker(max_misses=3)
    assert tracker.step([box(1, 1)], 0.0) == []          # tentative after one scan
    assert len(tracker.step([box(1, 1)], 0.1)) == 1      # confirmed
    for k in range(4):
        tracks = tracker.step([], 0.2 + 0.1 * k)
    assert tracks == [] and tracker.tracks == []


def test_two_objects_are_not_swapped():
    tracker = Tracker()
    for k in range(10):
        tracks = tracker.step([box(2, -0.5 + 0.02 * k), box(2, 0.5 - 0.02 * k)], 0.1 * k)
    by_id = {t.id: t for t in tracks}
    assert by_id[1].x[1] < by_id[2].x[1]                  # first stays below the second


def test_boxes_to_frame_rotates_and_translates():
    (b,) = boxes_to_frame([box(1.0, 0.0)], 2.0, 3.0, math.pi / 2)
    assert abs(b.x - 2.0) < 1e-9 and abs(b.y - 4.0) < 1e-9


def drive(polygons_at, sensor_at, steps, dt=0.1):
    """Run the lidar pipeline with the motion check; polygons and sensor (yaw 0) move per step."""
    tracker, history = Tracker(), []
    for k in range(steps):
        sx, sy = sensor_at(k * dt)
        shifted = [[(x - sx, y - sy) for x, y in poly] for poly in polygons_at(k * dt)]
        ranges = raycast(shifted)
        boxes = boxes_to_frame(detect_boxes(ranges, 0.0, math.radians(1), 0.12, 10.0), sx, sy, 0)

        def evidence(points, sx=sx, sy=sy, ranges=ranges):    # the tracker keeps it for later
            return scan_motion_evidence(points, (sx, sy, 0.0), ranges, 0.0, math.radians(1),
                                        0.12, 10.0)
        history.append(tracker.step(boxes, k * dt, evidence))
    return history


def test_static_box_passed_closely_is_not_moving():
    # Driving past a box reveals its faces one after another, so its centre (and the Kalman
    # velocity) drifts. The scan never sees free space where the box was, so it stays static.
    box = rectangle(2.0, 0.7, 0.0, 1.0, 0.6)
    history = drive(lambda t: [box], lambda t: (0.4 * t, 0.0), 40)
    tracks = [t for step in history for t in step]
    assert tracks and max(t.speed for t in tracks) > 0.15     # the Kalman filter is fooled
    assert all(t.velocity == (0.0, 0.0) for t in tracks)


def test_box_crossing_in_front_is_moving_at_its_velocity():
    history = drive(lambda t: [rectangle(2.5, -1.5 + 0.5 * t, 0.0, 0.4, 0.4)],
                    lambda t: (0.0, 0.0), 30)
    (track,) = history[-1]
    assert track.moving
    vx, vy = track.velocity
    assert abs(vx) < 0.1 and abs(vy - 0.5) < 0.1


def test_static_box_driven_at_is_not_moving():
    history = drive(lambda t: [rectangle(3.0, 0.0, 0.0, 0.5, 0.5)], lambda t: (0.4 * t, 0.0), 40)
    tracks = [t for step in history for t in step]
    assert tracks and all(t.velocity == (0.0, 0.0) for t in tracks)


def test_box_coming_straight_at_the_sensor_is_moving():
    # Its earlier points are hidden behind it, so only the earlier scans, which saw through the
    # space it now occupies, can tell that it moves.
    history = drive(lambda t: [rectangle(4.0 - 0.5 * t, 0.0, 0.0, 0.4, 0.4)],
                    lambda t: (0.0, 0.0), 20)
    (track,) = history[-1]
    assert track.moving
    vx, vy = track.velocity
    assert abs(vx + 0.5) < 0.1 and abs(vy) < 0.1


def test_motion_evidence_counts_points_left_free_and_points_still_hit():
    ranges = raycast([rectangle(2.05, 0.0, 0.0, 0.1, 4.0)])  # wall face at x = 2
    args = ((0.0, 0.0, 0.0), ranges, 0.0, math.radians(1), 0.12, 10.0)
    on_wall = np.column_stack((np.full(5, 2.0), np.linspace(-0.5, 0.5, 5)))
    assert scan_motion_evidence(on_wall, *args) == (0, 5)
    # Where the scan now passes through: the object there has gone.
    in_front = on_wall - (0.5, 0.0)
    assert scan_motion_evidence(in_front, *args) == (5, 0)
    # Hidden behind the wall: no evidence either way.
    assert scan_motion_evidence(on_wall + (0.5, 0.0), *args) == (0, 0)
    # The same from a sensor that has moved and turned.
    world = raycast([[(x - 0.5, y) for x, y in rectangle(2.05, 0.0, 0.0, 0.1, 4.0)]])
    turned = world[30:] + world[:30]        # beam i of the turned sensor looks along i + 30 deg
    assert scan_motion_evidence(on_wall, (0.5, 0.0, math.radians(30)), turned, 0.0,
                                math.radians(1), 0.12, 10.0) == (0, 5)


def test_mask_returns_blanks_only_returns_on_the_boxes():
    # A wall at x = 3 and a box at (1.5, 0) in front of it, seen from (0.5, 0) facing +x.
    world = [rectangle(3.05, 0.0, 0.0, 0.1, 6.0), rectangle(1.5, 0.0, 0.0, 0.4, 0.4)]
    ranges = raycast([[(x - 0.5, y) for x, y in poly] for poly in world])
    masked = mask_returns(ranges, 0.0, math.radians(1), (0.5, 0.0, 0.0),
                          [(1.5, 0.0, 0.0, 0.4, 0.4)])
    on_box = [i for i, r in enumerate(ranges) if math.isfinite(r) and r < 1.5]
    assert on_box and all(math.isnan(masked[i]) for i in on_box)
    others = [i for i in range(len(ranges)) if i not in on_box]
    assert all(masked[i] == ranges[i] or (math.isinf(masked[i]) and math.isinf(ranges[i]))
               for i in others)
    assert np.array_equal(mask_returns(ranges, 0.0, math.radians(1), (0.5, 0.0, 0.0), []),
                          np.array(ranges))
