import math

import numpy as np
import pytest

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
