import math

import numpy as np
import pytest

from unity_slam_example.teleop_math import (assisted_twist, choose_heading, corridor_free,
                                            straight_free)

HALF_WIDTH = 0.42


def block(x, y0=-0.5, y1=0.5, spacing=0.02):
    """Points of an obstacle face across the path at distance x (base frame)."""
    ys = np.arange(y0, y1 + spacing / 2, spacing)
    return np.column_stack((np.full_like(ys, x), ys))


def test_empty_world_is_free_to_the_lookahead():
    assert np.all(corridor_free(np.zeros((0, 2)), [0.0, 1.0], HALF_WIDTH, 10.0) == 10.0)


def test_corridor_ends_where_the_body_would_touch():
    free = corridor_free([[3.0, 0.0]], [0.0, math.pi / 2], HALF_WIDTH, 10.0)
    assert free[0] == pytest.approx(3.0 - HALF_WIDTH)
    assert free[1] == 10.0                              # the point is not to the left
    # Off to the side by more than the half width, or behind: no effect.
    assert straight_free([[3.0, 0.5], [-1.0, 0.0]], HALF_WIDTH) == 10.0


def test_clear_road_drives_straight_on():
    v, w, heading, free = assisted_twist(block(12.0), 0.5, 0.0)
    assert (v, w, heading, free) == pytest.approx((0.5, 0.0, 0.0, 10.0))


def test_obstacle_within_lookahead_is_steered_around():
    v, w, heading, free = assisted_twist(block(3.0), 0.5, 0.0)
    # The chosen corridor clears the block: its edge (0.5 m) plus the half width, 3 m ahead.
    assert abs(heading) >= math.atan2(0.5 + HALF_WIDTH, 3.0) - math.radians(2)
    assert free == 10.0
    assert w * heading > 0 and v > 0.0
    # Smallest turn that clears it, not a wild swerve.
    assert abs(heading) < math.radians(30)


def test_obstacle_beyond_lookahead_is_ignored():
    assert assisted_twist(block(11.0), 0.5, 0.0)[2] == pytest.approx(0.0, abs=1e-9)


def test_returns_to_the_held_heading_once_past():
    # Driving 20 degrees off to the left after an avoidance; the block is now behind.
    _, w, heading, _ = assisted_twist(block(-0.5), 0.5, math.radians(-20))
    assert heading == pytest.approx(math.radians(-20)) and w < 0.0


def test_keeps_to_the_side_it_chose():
    left = assisted_twist(block(3.0), 0.5, 0.0, previous=math.radians(20))[2]
    right = assisted_twist(block(3.0), 0.5, 0.0, previous=math.radians(-20))[2]
    assert left > 0.0 > right


def test_stops_and_turns_when_boxed_in():
    angles = np.linspace(-math.pi, math.pi, 72, endpoint=False)
    ring = np.column_stack((0.6 * np.cos(angles), 0.6 * np.sin(angles)))
    v, _, _, _ = assisted_twist(ring, 0.5, 0.0)
    assert v == 0.0


def test_heading_behind_turns_in_place():
    v, w, heading, _ = assisted_twist(np.zeros((0, 2)), 0.5, math.radians(170))
    assert heading == pytest.approx(math.pi / 2) and w == 0.5
    assert v == pytest.approx(0.0, abs=1e-9)


def test_choose_heading_prefers_clear_then_close():
    angles = np.radians([-20.0, 0.0, 10.0, 30.0])
    free = np.array([10.0, 2.0, 10.0, 10.0])
    assert choose_heading(angles, free, 0.0, None, 10.0) == 2
