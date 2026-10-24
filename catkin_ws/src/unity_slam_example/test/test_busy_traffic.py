"""Seeded busy-warehouse traffic for the scenario bench (test/busy_traffic.py)."""

import itertools
import math

from busy_traffic import BOXES, Traffic

FAR = (100.0, 100.0)    # robot out of everyone's way


def overlap(a, b):
    """Axis-aligned boxes (cx, cy, hx, hy) overlap."""
    return abs(a[0] - b[0]) < a[2] + b[2] and abs(a[1] - b[1]) < a[3] + b[3]


def run(seed, seconds, robot=FAR):
    traffic, frames = Traffic(seed), []
    for _ in range(int(round(seconds / 0.1))):
        traffic.step(0.1, robot)
        frames.append(traffic.boxes())
    return frames


def test_same_seed_same_traffic():
    assert run(3, 60) == run(3, 60)
    assert run(3, 10) != run(4, 10)


def test_carts_stay_in_aisles_and_apart():
    for boxes in run(1, 120):
        assert not any(overlap(c, s) for c in boxes for s in BOXES)
        assert not any(overlap(a, b) for a, b in itertools.combinations(boxes, 2))


def test_carts_keep_moving():
    first, last = run(2, 60)[0], run(2, 60)[-1]
    assert sum(math.hypot(a[0] - b[0], a[1] - b[1]) > 2.0 for a, b in zip(first, last)) >= 5


def test_start_clear_of_robot():
    assert all(math.hypot(cx, cy) >= 3.0 for cx, cy, _, _ in Traffic(5).boxes())


def test_cart_yields_then_turns_back():
    traffic = Traffic(0, carts=1)
    cart = traffic.carts[0]
    cart.start, cart.end, cart.progress = (2.0, 0.0), (8.0, 0.0), 0.5    # at x = 5, heading +x
    cart.paused = cart.blocked = 0.0
    (cx, cy, _, _), = traffic.boxes()
    assert (cx, cy) == (5.0, -0.5)                                      # keeps right
    robot = (cx + 1.3, cy)
    for _ in range(30):                                                 # 3 s: it waits
        traffic.step(0.1, robot)
        box, = traffic.boxes()
        assert not overlap(box, (robot[0], robot[1], 0.25, 0.25))
    assert traffic.boxes()[0][:2] == (cx, cy)
    for _ in range(60):                                                 # past patience (5 s)
        traffic.step(0.1, robot)
    bx, by, _, _ = traffic.boxes()[0]
    assert math.hypot(bx - robot[0], by - robot[1]) > 1.3 + 0.5
