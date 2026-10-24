"""Stall metric and traffic-wait rule of the scenario bench (test/scenarios.py)."""

import math

import pytest

from scenarios import box_points, stall_report, traffic_blocking

GOAL = (100.0, 0.0)


def trace(segments, x=0.0, step=0.1):
    """0.1 s samples (t, x, y, v, waiting) along +x from `segments` of (seconds, v, waiting)."""
    out, t = [], 0.0
    for seconds, v, waiting in segments:
        for _ in range(int(round(seconds / step))):
            out.append((t, x, 0.0, v, waiting))
            t += step
            x += v * step
    return out


def test_first_turn_not_counted():
    assert stall_report(trace([(5, 0, False), (5, 0.2, False)]), GOAL)['longest_stall'] == 0


def test_mid_route_stall_counted():
    r = stall_report(trace([(2, 0.2, False), (4.5, 0, False), (2, 0.2, False)]), GOAL)
    assert r['longest_stall'] == pytest.approx(4.5, abs=0.11)
    assert r['stalls_over_3s'] == 1


def test_final_turn_not_counted():
    # 0.2 m/s for 9 s from x = 0 ends 0.2 m short of a goal at x = 2.0, then turns on the spot.
    r = stall_report(trace([(9, 0.2, False), (5, 0, False)]), (2.0, 0.0))
    assert r['longest_stall'] == 0


def test_traffic_wait_split_out():
    r = stall_report(trace([(2, 0.2, False), (1, 0, False), (2, 0, True), (1, 0, False),
                            (1, 0.2, False)]), GOAL)
    assert r['longest_stall'] == pytest.approx(1.0, abs=0.11)
    assert r['wait_longest'] == pytest.approx(2.0, abs=0.11)
    assert r['wait_total'] == pytest.approx(2.0, abs=0.11)
    assert r['waits'] == 1


def test_reversing_is_not_a_stall():
    assert stall_report(trace([(1, 0.2, False), (4, -0.1, False)]), GOAL)['longest_stall'] == 0


def test_traffic_blocking_cone_and_distance():
    cart = box_points((2.0, 0.0, 0.3, 0.3))          # near face 1.7 m ahead: 1.45 m from the body
    assert traffic_blocking(0, 0, 0.0, [cart])
    assert not traffic_blocking(0, 0, math.pi, [cart])                         # behind
    assert not traffic_blocking(0, 0, 0.0, [box_points((2.2, 0.0, 0.3, 0.3))])   # 1.65 m
    assert not traffic_blocking(0, 0, 0.0, [box_points((1.0, 1.4, 0.1, 0.1))])   # >= 49.8° off
