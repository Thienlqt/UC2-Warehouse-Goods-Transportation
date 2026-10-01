"""Pure maths for assisted keyboard driving (no ROS imports, unit tested).

The driver asks for a heading (straight on when W was pressed). Each candidate direction around
the robot gets the free length of a straight corridor as wide as the robot plus a margin, up to a
lookahead. The robot steers to the best direction: clear far ahead, close to the driver's heading,
and close to the previous choice (so it does not flip from side to side).
"""

import math

import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def corridor_free(points, angles, half_width, lookahead):
    """Free length (m) of a straight corridor from the robot in each direction, up to lookahead.

    points: (N, 2) obstacle points in the base frame. A point blocks a direction when it lies
    ahead within half_width of the corridor's centre line; the corridor ends where the robot's
    circle of radius half_width would first touch it.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    angles = np.asarray(angles, dtype=float)
    if not len(pts):
        return np.full(len(angles), float(lookahead))
    c, s = np.cos(angles)[:, None], np.sin(angles)[:, None]
    along = c * pts[:, 0] + s * pts[:, 1]
    across = -s * pts[:, 0] + c * pts[:, 1]
    inside = (along > 0.0) & (np.abs(across) < half_width)
    reach = along - np.sqrt(np.maximum(half_width ** 2 - across ** 2, 0.0))
    reach = np.where(inside, reach, lookahead)
    return np.clip(reach.min(axis=1), 0.0, lookahead)


def choose_heading(angles, free, desired, previous, lookahead, deviation_weight=0.5,
                   change_weight=0.2):
    """Index of the best direction: score = free fraction - weighted turn from desired/previous."""
    angles = np.asarray(angles, dtype=float)
    away = np.abs(np.arctan2(np.sin(angles - desired), np.cos(angles - desired)))
    score = np.minimum(free, lookahead) / lookahead - deviation_weight * away / math.pi
    if previous is not None:
        turn = np.abs(np.arctan2(np.sin(angles - previous), np.cos(angles - previous)))
        score = score - change_weight * turn / math.pi
    return int(np.argmax(score))


def assisted_twist(points, speed, desired, previous=None, half_width=0.42, lookahead=10.0,
                   span=math.pi / 2, resolution=math.radians(2.0), turn_gain=1.5,
                   max_turn=0.5, stop_distance=0.3, slow_distance=1.0,
                   deviation_weight=0.5, change_weight=0.2):
    """(v, w, heading, free) for driving forward at `speed` towards `desired` (rad, base frame).

    Candidates span +-span around straight ahead, plus the desired heading itself when it is in
    range. The robot turns towards the chosen heading and slows as it turns away from its nose
    or as the chosen corridor gets short; it stops stop_distance before the corridor ends.
    """
    angles = np.arange(-span, span + 1e-9, resolution)
    if abs(desired) <= span:
        angles = np.append(angles, desired)
    free = corridor_free(points, angles, half_width, lookahead)
    i = choose_heading(angles, free, desired, previous, lookahead, deviation_weight,
                       change_weight)
    heading, length = float(angles[i]), float(free[i])
    w = max(-max_turn, min(max_turn, turn_gain * heading))
    clear = max(0.0, min(1.0, (length - stop_distance) / slow_distance))
    v = speed * max(0.0, math.cos(heading)) * clear
    return v, w, heading, length


def straight_free(points, half_width=0.42, lookahead=10.0):
    """Free corridor length straight ahead (for slowing while the driver steers by hand)."""
    return float(corridor_free(points, [0.0], half_width, lookahead)[0])
