"""Pure maths for the cmd_vel safety guard (no ROS imports, unit tested).

Circular footprint, collision-monitor "approach" style: roll the
commanded twist forward and slow it so that contact stays at least `horizon` seconds away.
Tracked moving obstacles (avoidance_math.Track, base frame) are rolled forward alongside it.
"""

import math

import numpy as np

from .avoidance_math import advance, rect_distance


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


def safe_time(points, v, w, radius, horizon, step, min_points, movers=()):
    """Seconds the twist can run before `min_points` points enter the footprint, or None.

    The result is the last collision-free simulated time, so contact on the very first step
    gives 0 (stop). Points already inside the footprint are ignored: they cannot be avoided
    by slowing down, and counting them would also block turning or backing away.

    `movers` are tracked boxes moving at constant velocity. One box entering the footprint is a
    contact. A box that would reach a stationary robot too cannot be avoided by stopping, so it
    only counts while the commanded motion brings the robot closer to it than standing still
    would: turning and backing away stay allowed, driving into its way does not. The same
    applies to boxes already touching the footprint.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    pts = pts[np.hypot(pts[:, 0], pts[:, 1]) > radius]
    if len(pts) < min_points:
        pts = pts[:0]
    if not len(pts) and not movers:
        return None
    for k in range(1, int(round(horizon / step)) + 1):
        t = k * step
        x, y = position_after(v, w, t)
        if len(pts) and np.count_nonzero(
                np.hypot(pts[:, 0] - x, pts[:, 1] - y) <= radius) >= min_points:
            return (k - 1) * step
        for m in movers:
            ahead = advance(m, t)
            moving = rect_distance(x, y, ahead)
            if moving > radius:
                continue
            standing = rect_distance(0.0, 0.0, ahead)
            if standing > radius or moving < standing - 0.01:
                return (k - 1) * step
    return None


def guard_twist(v, w, points, radius=0.22, horizon=1.2, step=0.1, min_points=6, movers=()):
    """Return (v, w) scaled by safe time / horizon, or unchanged if no contact is predicted."""
    t = safe_time(points, v, w, radius, horizon, step, min_points, movers)
    if t is None:
        return v, w
    scale = t / horizon
    return v * scale, w * scale
