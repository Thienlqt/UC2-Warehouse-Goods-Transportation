"""Pure maths for the cmd_vel safety guard (no ROS imports, unit tested).

Stands in for Nav2's collision_monitor "approach" action on a circular footprint: roll the
commanded twist forward and slow it so that contact stays at least `horizon` seconds away.
"""

import math

import numpy as np


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, x=0.0, y=0.0, yaw=0.0):
    """Valid scan returns as an (N, 2) array in the base frame; the laser sits at (x, y, yaw)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, a = r[ok], a[ok] + yaw
    return np.column_stack((x + r * np.cos(a), y + r * np.sin(a)))


def position_after(v, w, t):
    """Base position after driving (v, w) for t seconds from the origin (unicycle model)."""
    if abs(w) < 1e-9:
        return v * t, 0.0
    return v / w * math.sin(w * t), v / w * (1.0 - math.cos(w * t))


def safe_time(points, v, w, radius, horizon, step, min_points):
    """Seconds the twist can run before `min_points` points enter the footprint, or None.

    The result is the last collision-free simulated time, so contact on the very first step
    gives 0 (stop). Points already inside the footprint are ignored: they cannot be avoided
    by slowing down, and counting them would also block turning or backing away.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    pts = pts[np.hypot(pts[:, 0], pts[:, 1]) > radius]
    if len(pts) < min_points:
        return None
    for k in range(1, int(round(horizon / step)) + 1):
        x, y = position_after(v, w, k * step)
        if np.count_nonzero(np.hypot(pts[:, 0] - x, pts[:, 1] - y) <= radius) >= min_points:
            return (k - 1) * step
    return None


def guard_twist(v, w, points, radius=0.22, horizon=1.2, step=0.1, min_points=6):
    """Return (v, w) scaled by safe time / horizon, or unchanged if no contact is predicted."""
    t = safe_time(points, v, w, radius, horizon, step, min_points)
    if t is None:
        return v, w
    scale = t / horizon
    return v * scale, w * scale
