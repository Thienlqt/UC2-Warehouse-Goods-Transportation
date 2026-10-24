"""A busy warehouse for test/scenarios.py: shelves, aisles and seeded roaming carts. No ROS.

The aisles are 3 m wide, along x at y = -4, 0, 4 and across at x = 2, 8, 14, between 1 m deep
shelves. The robot starts at the origin facing +x; GOAL is past the last cross aisle. Carts
(WorkerWithCart: 0.6 x 1.6 m, 0.8 m/s) roam between aisle crossings like the Unity traffic:
they keep right, pause at random, hold back from whatever is in the strip ahead of them (the
robot included, with a `berth` around it, as the Unity movers pass wide) and turn back after
`patience` seconds blocked.
"""

from dataclasses import dataclass
import math
import random

AISLE_X = (2.0, 8.0, 14.0)
AISLE_Y = (-4.0, 0.0, 4.0)
WALL = 0.05                                              # half-thickness (m)
BOXES = [(cx, cy, 1.5, 0.5) for cx in (5.0, 11.0) for cy in (-2.0, 2.0)] + [
    (8.0, -5.5, 9.0, WALL), (8.0, 5.5, 9.0, WALL),       # long walls
    (-1.0, 0.0, WALL, 5.5), (17.0, 0.0, WALL, 5.5)]      # end walls
GOAL = (16.2, 0.0)
NODES = [(x, y) for x in AISLE_X for y in AISLE_Y]


def neighbours(node):
    """Crossings one aisle segment away."""
    i, j = AISLE_X.index(node[0]), AISLE_Y.index(node[1])
    return [(AISLE_X[a], AISLE_Y[b]) for a, b in ((i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1))
            if 0 <= a < len(AISLE_X) and 0 <= b < len(AISLE_Y)]


def overlap(a, b):
    """Axis-aligned boxes (cx, cy, hx, hy) overlap."""
    return abs(a[0] - b[0]) < a[2] + b[2] and abs(a[1] - b[1]) < a[3] + b[3]


def touches_circle(box, xy, radius):
    cx, cy, hx, hy = box
    return math.hypot(max(abs(xy[0] - cx) - hx, 0.0), max(abs(xy[1] - cy) - hy, 0.0)) < radius


@dataclass
class Cart:
    start: tuple                 # crossing it comes from
    end: tuple                   # crossing it drives to
    progress: float = 0.0        # 0 at start, 1 at end
    paused: float = 0.0          # seconds of random stop left
    blocked: float = 0.0         # seconds held back so far


class Traffic:
    def __init__(self, seed, carts=10, speed=0.8, half_length=0.8, half_width=0.3, lane=0.5,
                 gap=0.8, stops_per_minute=1.0, stop_seconds=3.0, patience=5.0,
                 start_clearance=3.0, berth=0.3):
        self.rng = random.Random(seed)
        self.speed, self.lane, self.gap, self.berth = speed, lane, gap, berth
        self.half_length, self.half_width = half_length, half_width
        self.stop_rate, self.stop_seconds, self.patience = stops_per_minute / 60.0, stop_seconds, \
            patience
        self.carts = []
        for _ in range(1000):
            if len(self.carts) == carts:
                break
            start = self.rng.choice(NODES)
            cart = Cart(start, self.rng.choice(neighbours(start)), self.rng.random())
            box = self.box(cart)
            if math.hypot(box[0], box[1]) >= start_clearance and not any(
                    overlap(self.grown(box, 0.2), self.box(c)) for c in self.carts):
                self.carts.append(cart)

    def box(self, cart):
        """(cx, cy, hx, hy) of a cart: long along its travel, `lane` m right of the centre line."""
        (px, py), (qx, qy) = cart.start, cart.end
        length = math.hypot(qx - px, qy - py)
        dx, dy = (qx - px) / length, (qy - py) / length
        x = px + (qx - px) * cart.progress + dy * self.lane
        y = py + (qy - py) * cart.progress - dx * self.lane
        along_x = abs(dx) > 0.5
        return (x, y, self.half_length if along_x else self.half_width,
                self.half_width if along_x else self.half_length)

    @staticmethod
    def grown(box, margin):
        return (box[0], box[1], box[2] + margin, box[3] + margin)

    def strip_ahead(self, cart, box):
        """The `gap`-long strip in front of a cart, as wide as the cart."""
        (px, py), (qx, qy) = cart.start, cart.end
        dx, dy = math.copysign(1.0, qx - px) if qx != px else 0.0, \
            math.copysign(1.0, qy - py) if qy != py else 0.0
        if dx:
            return (box[0] + dx * (box[2] + self.gap / 2), box[1], self.gap / 2, box[3])
        return (box[0], box[1] + dy * (box[3] + self.gap / 2), box[2], self.gap / 2)

    def free(self, cart, box, robot_xy, robot_radius):
        return not touches_circle(box, robot_xy, robot_radius + self.berth) and not any(
            overlap(box, self.box(c)) for c in self.carts if c is not cart)

    def step(self, dt, robot_xy, robot_radius=0.25):
        for cart in self.carts:
            roll = self.rng.random()         # one draw per cart and step keeps runs repeatable
            if cart.paused > 0.0:
                cart.paused = max(0.0, cart.paused - dt)
                continue
            if roll < self.stop_rate * dt:
                cart.paused = self.stop_seconds
                continue
            if cart.blocked >= self.patience:
                back = Cart(cart.end, cart.start, 1.0 - cart.progress)
                if self.free(cart, self.box(back), robot_xy, robot_radius):
                    cart.start, cart.end, cart.progress, cart.blocked = \
                        back.start, back.end, back.progress, 0.0
                    continue
            if cart.progress >= 1.0:
                # shortcut: instant 90° turns at crossings; model the swing if a planner reacts to it.
                options = [n for n in neighbours(cart.end) if n != cart.start] or [cart.start]
                turn = Cart(cart.end, self.rng.choice(options))
                if self.free(cart, self.box(turn), robot_xy, robot_radius):
                    cart.start, cart.end, cart.progress = turn.start, turn.end, 0.0
                else:
                    cart.blocked += dt
                continue
            box = self.box(cart)
            if not self.free(cart, self.strip_ahead(cart, box), robot_xy, robot_radius):
                cart.blocked += dt
                continue
            cart.blocked = 0.0
            (px, py), (qx, qy) = cart.start, cart.end
            cart.progress = min(1.0, cart.progress + self.speed * dt / math.hypot(qx - px, qy - py))

    def boxes(self):
        return [self.box(c) for c in self.carts]
