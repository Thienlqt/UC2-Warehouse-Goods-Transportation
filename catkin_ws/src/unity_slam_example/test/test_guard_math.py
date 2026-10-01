import math

import numpy as np
import pytest

from unity_slam_example.avoidance_math import Track
from unity_slam_example.guard_math import guard_twist, position_after, safe_time, scan_points


def wall(x, half_width=0.5, spacing=0.02):
    """Points of a wall across the robot's path at distance x (base frame)."""
    ys = np.arange(-half_width, half_width + spacing / 2, spacing)
    return np.column_stack((np.full_like(ys, x), ys))


def test_scan_points_applies_mount_and_drops_invalid():
    pts = scan_points([1.0, float('inf'), 0.05, 2.0], -math.pi / 2, math.pi / 2, 0.12, 3.5,
                      x=0.1, yaw=math.pi / 2)
    # Valid beams: 1.0 m at -90° and 2.0 m at +180°, both rotated by the +90° mount yaw.
    assert pts.shape == (2, 2)
    np.testing.assert_allclose(pts[0], [1.1, 0.0], atol=1e-9)
    np.testing.assert_allclose(pts[1], [0.1, -2.0], atol=1e-9)


def test_position_after_straight_and_arc():
    assert position_after(0.2, 0.0, 1.0) == pytest.approx((0.2, 0.0))
    x, y = position_after(0.5, 0.5, math.pi)       # quarter circle of radius 1
    assert x == pytest.approx(1.0) and y == pytest.approx(1.0)
    x, y = position_after(0.5, 0.5, 2 * math.pi)   # half circle
    assert x == pytest.approx(0.0, abs=1e-9) and y == pytest.approx(2.0)


def test_clear_path_passes_through():
    assert guard_twist(0.2, 0.3, wall(3.0)) == (0.2, 0.3)


def test_wall_ahead_slows_then_stops():
    # Wall 0.41 m ahead at 0.2 m/s: at 0.9 s the base is 0.23 m away (clear); at 1.0 s it is
    # 0.21 m away with 7 wall points inside the 0.22 m circle. Last safe step 0.9 s -> x0.75.
    v, w = guard_twist(0.2, 0.0, wall(0.41))
    assert v == pytest.approx(0.2 * 0.9 / 1.2) and w == 0.0
    # Contact already on the first 0.1 s step: stop.
    assert guard_twist(0.2, 0.1, wall(0.225)) == (0.0, 0.0)


def test_turning_in_place_is_never_limited():
    # A circular footprint cannot hit anything by rotating.
    assert guard_twist(0.0, 0.5, wall(0.25)) == (0.0, 0.5)


def test_driving_away_is_not_limited():
    assert guard_twist(-0.2, 0.0, wall(0.30)) == (-0.2, 0.0)
    assert guard_twist(0.2, 0.0, wall(-0.30)) == (0.2, 0.0)


def test_reversing_toward_a_wall_is_limited():
    v, _ = guard_twist(-0.2, 0.0, wall(-0.41))
    assert v == pytest.approx(-0.2 * 0.9 / 1.2)


def test_points_inside_footprint_are_ignored():
    # Something already touching the body must not freeze the robot (it could never escape).
    inside = np.array([[0.1, 0.0]] * 10)
    assert guard_twist(0.2, 0.0, inside) == (0.2, 0.0)


def test_min_points_rejects_isolated_returns():
    post = wall(0.41, half_width=0.04)      # 5 points straight ahead
    assert safe_time(post, 0.2, 0.0, 0.22, 1.2, 0.1, 6) is None
    assert safe_time(post, 0.2, 0.0, 0.22, 1.2, 0.1, 5) == pytest.approx(0.9)


NO_POINTS = np.zeros((0, 2))


def mover(x, y, vx=0.0, vy=0.0, size=0.4):
    return Track(x, y, 0.0, size, size, vx, vy, 'obstacle', 0.9)


def test_box_crossing_the_path_slows_the_robot():
    # A box crossing from the right at 0.8 m/s meets the robot (0.2 m/s) at 1.0 s; 0.9 s is the
    # last clear step. The scan alone (box still to the side) would not limit the robot.
    crossing = mover(0.6, -1.0, vy=0.8)
    v, w = guard_twist(0.2, 0.0, NO_POINTS, movers=[crossing])
    assert v == pytest.approx(0.2 * 0.9 / 1.2) and w == 0.0


def test_oncoming_box_slows_the_robot_earlier_than_a_static_one():
    # Closing at 0.2 + 0.5 m/s from 0.8 m (box edge to centre): contact at ~0.83 s.
    v, _ = guard_twist(0.2, 0.0, NO_POINTS, movers=[mover(1.0, 0.0, vx=-0.5)])
    assert v == pytest.approx(0.2 * 0.8 / 1.2)


def test_boxes_moving_away_or_alongside_do_not_limit():
    assert guard_twist(0.2, 0.0, NO_POINTS, movers=[mover(0.5, 0.0, vx=0.5)]) == (0.2, 0.0)
    assert guard_twist(0.2, 0.0, NO_POINTS, movers=[mover(0.0, 0.6, vx=0.2)]) == (0.2, 0.0)


def test_box_already_touching_is_ignored():
    assert guard_twist(0.2, 0.0, NO_POINTS, movers=[mover(0.3, 0.0, vx=-0.5)]) == (0.2, 0.0)


def test_box_that_would_hit_a_stopped_robot_does_not_freeze_it():
    # Stopping cannot avoid it, so turning (or driving clear) stays allowed.
    incoming = mover(0.0, -0.8, vy=0.8)
    assert guard_twist(0.0, 0.5, NO_POINTS, movers=[incoming]) == (0.0, 0.5)
