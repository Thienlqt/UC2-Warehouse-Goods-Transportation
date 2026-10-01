import math

import numpy as np
import pytest

from unity_slam_example.avoidance_math import (advance, box_outline, inverse_pose,
                                               occupancy_grid, predicted_points, rect_distance,
                                               Track, track_to_frame)


def box(x=0.0, y=0.0, vx=0.0, vy=0.0, yaw=0.0, length=0.4, width=0.4, label='obstacle',
        score=0.9):
    return Track(x, y, yaw, length, width, vx, vy, label, score)


def test_outline_lies_on_the_padded_perimeter():
    pts = box_outline(box(1.0, 2.0, length=0.4, width=0.2), padding=0.1, spacing=0.05)
    assert pts[:, 0].min() == pytest.approx(0.7) and pts[:, 0].max() == pytest.approx(1.3)
    assert pts[:, 1].min() == pytest.approx(1.8) and pts[:, 1].max() == pytest.approx(2.2)
    on_x_edge = np.isclose(np.abs(pts[:, 0] - 1.0), 0.3)
    on_y_edge = np.isclose(np.abs(pts[:, 1] - 2.0), 0.2)
    assert np.all(on_x_edge | on_y_edge)
    # Walking the perimeter, no gap is wider than the spacing.
    loop = np.vstack((pts, pts[:1]))
    assert np.hypot(*np.diff(loop, axis=0).T).max() <= 0.05 + 1e-9


def test_outline_follows_heading():
    pts = box_outline(box(length=0.6, width=0.2, yaw=math.pi / 2))
    assert np.ptp(pts[:, 0]) == pytest.approx(0.2) and np.ptp(pts[:, 1]) == pytest.approx(0.6)


def test_moving_track_sweeps_along_its_velocity_up_to_the_horizon():
    pts = predicted_points([box(2.0, 0.0, vy=0.5)], horizon=2.0, step=0.2)
    assert pts[:, 1].min() == pytest.approx(-0.2)            # now
    assert pts[:, 1].max() == pytest.approx(0.5 * 2.0 + 0.2)  # in 2 s, not beyond
    assert pts[:, 0].min() == pytest.approx(1.8) and pts[:, 0].max() == pytest.approx(2.2)


def test_static_tracks_are_left_to_the_scan_unless_padded():
    slow = box(2.0, 0.0, vx=0.1)
    assert len(predicted_points([slow], min_speed=0.15)) == 0
    person = slow._replace(label='person')
    pts = predicted_points([person], min_speed=0.15, padding_by_class={'person': 0.3})
    assert pts[:, 0].min() == pytest.approx(1.5) and pts[:, 0].max() == pytest.approx(2.5)


def test_low_confidence_tracks_are_ignored():
    assert len(predicted_points([box(2.0, 0.0, vx=1.0, score=0.2)], min_confidence=0.3)) == 0


def test_points_near_the_robot_are_dropped():
    pts = predicted_points([box(1.0, 1.0, vx=-0.5)], robot_xy=(0.5, 1.0), keep_out=0.45)
    assert len(pts) > 0
    assert np.hypot(pts[:, 0] - 0.5, pts[:, 1] - 1.0).min() > 0.45


def test_rect_distance():
    b = box(1.0, 0.0, length=0.4, width=0.2)
    assert rect_distance(1.1, 0.05, b) == 0.0                    # inside
    assert rect_distance(0.5, 0.0, b) == pytest.approx(0.3)      # in front of an edge
    assert rect_distance(1.5, 0.4, b) == pytest.approx(math.hypot(0.3, 0.3))  # off a corner
    turned = b._replace(yaw=math.pi / 2)                         # now 0.2 long in x
    assert rect_distance(0.5, 0.0, turned) == pytest.approx(0.4)


def test_advance_moves_position_only():
    b = advance(box(1.0, 2.0, vx=0.5, vy=-1.0), 2.0)
    assert (b.x, b.y, b.vx, b.vy) == pytest.approx((2.0, 0.0, 0.5, -1.0))


def test_track_to_base_frame():
    # Robot at (1, 1) facing +y; a box 1 m ahead of it moving along odom +x.
    to_base = inverse_pose(1.0, 1.0, math.pi / 2)
    b = track_to_frame(box(1.0, 2.0, vx=0.5), *to_base)
    assert (b.x, b.y) == pytest.approx((1.0, 0.0))
    assert (b.vx, b.vy) == pytest.approx((0.0, -0.5))           # odom +x is the robot's right
    assert b.yaw == pytest.approx(-math.pi / 2)


def test_occupancy_grid_is_aligned_around_the_robot():
    points = np.array([[1.33, -0.47], [9.0, 0.0]])          # the second is outside the grid
    ox, oy, n, data = occupancy_grid(points, (0.12, 0.0), size=6.0, resolution=0.05)
    assert n == 120 and data.shape == (120, 120)
    # Origins are multiples of the resolution, as the rolling local costmap's are.
    assert ox == pytest.approx(-2.9) and oy == pytest.approx(-3.0)
    assert np.count_nonzero(data) == 1
    assert data[int((-0.47 - oy) / 0.05), int((1.33 - ox) / 0.05)] == 100


def test_occupancy_grid_without_points_is_free():
    _, _, _, data = occupancy_grid(np.zeros((0, 2)), (0.0, 0.0))
    assert not data.any()
