"""Pure maths for avoiding tracked obstacles (no ROS imports, unit tested).

Tracks are oriented boxes with a constant velocity, as published on /obstacles. Their predicted
positions become costmap points (predicted_obstacles) and moving footprints for cmd_vel_guard.
"""

from collections import namedtuple
import math

import numpy as np

# Box centre, heading, size along / across the heading (m), velocity (m/s), class and score.
Track = namedtuple('Track', 'x y yaw length width vx vy label score')


def speed(track):
    return math.hypot(track.vx, track.vy)


def advance(track, t):
    """The track moved on by t seconds at constant velocity."""
    return track._replace(x=track.x + track.vx * t, y=track.y + track.vy * t)


def inverse_pose(x, y, yaw):
    """Inverse of a planar pose: if (x, y, yaw) is B in A, return A in B."""
    c, s = math.cos(yaw), math.sin(yaw)
    return -c * x - s * y, s * x - c * y, -yaw


def track_to_frame(track, tx, ty, yaw):
    """Transform a track by a planar pose; the velocity is rotated, not translated."""
    c, s = math.cos(yaw), math.sin(yaw)
    return track._replace(x=tx + c * track.x - s * track.y, y=ty + s * track.x + c * track.y,
                          yaw=track.yaw + yaw, vx=c * track.vx - s * track.vy,
                          vy=s * track.vx + c * track.vy)


def box_outline(track, padding=0.0, spacing=0.025):
    """(N, 2) points along the perimeter of the box grown by `padding` on every side."""
    hl, hw = track.length / 2 + padding, track.width / 2 + padding
    local = []
    for (x0, y0), (x1, y1) in [((-hl, -hw), (hl, -hw)), ((hl, -hw), (hl, hw)),
                               ((hl, hw), (-hl, hw)), ((-hl, hw), (-hl, -hw))]:
        n = max(1, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / spacing)))
        s = np.arange(n) / n                # the next edge supplies the end corner
        local.append(np.column_stack((x0 + (x1 - x0) * s, y0 + (y1 - y0) * s)))
    local = np.vstack(local)
    c, s = math.cos(track.yaw), math.sin(track.yaw)
    return np.column_stack((track.x + c * local[:, 0] - s * local[:, 1],
                            track.y + s * local[:, 0] + c * local[:, 1]))


def predicted_points(tracks, horizon=2.0, step=0.2, min_speed=0.15, min_confidence=0.3,
                     padding_by_class=None, spacing=0.025, robot_xy=(0.0, 0.0), keep_out=0.45):
    """Costmap points for where confident tracks are now and will be within `horizon` seconds.

    Moving tracks sweep their (padded) outline along their velocity. Static tracks are already in
    the scan, so they only add points when their class has padding. Points within `keep_out` of
    the robot are dropped: lethal cells on the robot itself would invalidate every trajectory.
    """
    return predicted_costs(tracks, horizon, step, min_speed, min_confidence, padding_by_class,
                           spacing, robot_xy, keep_out, lethal_horizon=horizon)[0]


def predicted_costs(tracks, horizon=2.0, step=0.2, min_speed=0.15, min_confidence=0.3,
                    padding_by_class=None, spacing=0.025, robot_xy=(0.0, 0.0), keep_out=0.45,
                    lethal_horizon=0.8, graded_max=80, graded_min=30):
    """predicted_points with a value per point: 100 (lethal) up to `lethal_horizon` seconds ahead,
    then falling from `graded_max` to `graded_min` at `horizon`.

    A cart 2 s away along its path blocks the robot only if the robot is there then, which a
    costmap cannot say; lethal cells there made DWA wait for carts still metres away.
    """
    padding_by_class = padding_by_class or {}
    chunks, values = [], []
    for track in tracks:
        if track.score < min_confidence:
            continue
        padding = padding_by_class.get(track.label, 0.0)
        if speed(track) >= min_speed:
            times = np.arange(int(math.floor(horizon / step + 1e-9)) + 1) * step
        elif padding > 0.0:
            times = [0.0]
        else:
            continue
        for t in times:
            chunks.append(box_outline(advance(track, t), padding, spacing))
            graded = graded_max - (graded_max - graded_min) * (t - lethal_horizon) / max(
                horizon - lethal_horizon, 1e-9)
            values.append(np.full(len(chunks[-1]), 100 if t <= lethal_horizon + 1e-9
                                  else int(round(graded))))
    if not chunks:
        return np.zeros((0, 2)), np.zeros(0, dtype=int)
    points, values = np.vstack(chunks), np.concatenate(values)
    keep = np.hypot(points[:, 0] - robot_xy[0], points[:, 1] - robot_xy[1]) > keep_out
    return points[keep], values[keep]


def occupancy_grid(points, robot_xy, size=6.0, resolution=0.05, values=None):
    """Square grid `size` metres wide around the robot: 100 in cells holding a point, else 0.

    With `values` (one per point), each cell holds the highest value of its points instead.

    The origin is a multiple of `resolution`, as the rolling local costmap's is, so the cells line
    up with the costmap's. Returns (origin_x, origin_y, cells per side, data[row y, column x]).
    """
    n = int(round(size / resolution))
    ox = math.floor((robot_xy[0] - size / 2) / resolution) * resolution
    oy = math.floor((robot_xy[1] - size / 2) / resolution) * resolution
    data = np.zeros((n, n), dtype=np.int8)
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    ix = np.floor((points[:, 0] - ox) / resolution).astype(int)
    iy = np.floor((points[:, 1] - oy) / resolution).astype(int)
    inside = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
    if values is None:
        data[iy[inside], ix[inside]] = 100
    else:
        np.maximum.at(data, (iy[inside], ix[inside]),
                      np.asarray(values, dtype=np.int8)[inside])
    return ox, oy, n, data


def rect_distance(px, py, track):
    """Distance from a point to the track's box (0 inside it)."""
    dx, dy = px - track.x, py - track.y
    c, s = math.cos(track.yaw), math.sin(track.yaw)
    along, across = c * dx + s * dy, -s * dx + c * dy
    return math.hypot(max(abs(along) - track.length / 2, 0.0),
                      max(abs(across) - track.width / 2, 0.0))
