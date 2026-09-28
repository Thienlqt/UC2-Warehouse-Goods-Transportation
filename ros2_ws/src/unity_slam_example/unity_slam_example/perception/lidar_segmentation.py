"""2D laser scan -> object segments -> oriented bounding boxes. Pure numpy, no ROS.

Segmentation: adaptive breakpoint detection (Borges & Aldon 2004). Consecutive points belong to
different objects when their gap exceeds what the beam geometry allows at that range.
Box fitting: search-based L-shape fitting (Zhang et al. 2017, closeness criterion), which gives
an oriented rectangle even when only one or two faces of an object are visible.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class Box:
    x: float          # centre, in the frame of the input points
    y: float
    yaw: float        # heading of the `length` edge, radians in [0, pi/2)
    length: float
    width: float
    num_points: int
    fit_error: float  # mean distance of points to the nearest rectangle edge (m)


def scan_points(ranges, angle_min, angle_increment, range_min, range_max):
    """Return (points Nx2, beam indices N) for finite in-range returns."""
    r = np.asarray(ranges, dtype=np.float64)
    idx = np.nonzero(np.isfinite(r) & (r >= range_min) & (r <= range_max))[0]
    angles = angle_min + idx * angle_increment
    pts = np.column_stack((r[idx] * np.cos(angles), r[idx] * np.sin(angles)))
    return pts, idx


def segment(points, beam_idx, angle_increment, num_beams, breakpoint_angle_deg=10.0,
            sigma=0.03, min_points=3):
    """Split ordered scan points into segments (lists of row indices into `points`).

    A beam gap (missing returns) also splits. For a full 360 degree scan the first and last
    segments are joined when they meet across the wrap-around.
    """
    n = len(points)
    if n == 0:
        return []
    lam = math.radians(breakpoint_angle_deg)
    ranges = np.hypot(points[:, 0], points[:, 1])

    def is_break(a, b):
        dphi = abs(angle_increment) * ((beam_idx[b] - beam_idx[a]) % num_beams)
        if dphi <= 0 or dphi >= lam:
            return True
        d_max = ranges[a] * math.sin(dphi) / math.sin(lam - dphi) + 3 * sigma
        return float(np.linalg.norm(points[b] - points[a])) > d_max

    segments, current = [], [0]
    for i in range(1, n):
        if is_break(i - 1, i):
            segments.append(current)
            current = []
        current.append(i)
    segments.append(current)

    full_circle = abs(angle_increment) * num_beams >= 2 * math.pi - 2 * abs(angle_increment)
    if full_circle and len(segments) > 1 and not is_break(n - 1, 0):
        segments[0] = segments.pop() + segments[0]
    return [s for s in segments if len(s) >= min_points]


def _edge_distances(points, thetas):
    """NxT distance of each point to the nearest edge of the bounding rectangle at each heading."""
    e1 = np.stack((np.cos(thetas), np.sin(thetas)))      # 2xT
    e2 = np.stack((-np.sin(thetas), np.cos(thetas)))
    c1 = points @ e1                                      # NxT projections
    c2 = points @ e2
    d1 = np.minimum(c1.max(0) - c1, c1 - c1.min(0))
    d2 = np.minimum(c2.max(0) - c2, c2 - c2.min(0))
    return np.minimum(d1, d2)


def _closeness(points, thetas, min_distance):
    """Zhang et al. closeness score (higher = points hug the rectangle edges)."""
    return (1.0 / np.maximum(_edge_distances(points, thetas), min_distance)).sum(0)


def fit_box(points, angle_step_deg=1.0, min_size=0.05, min_distance=0.01):
    """Fit an oriented rectangle to Nx2 points (L-shape fitting, closeness criterion)."""
    step = math.radians(angle_step_deg)
    coarse = np.arange(0.0, math.pi / 2, step)
    theta = coarse[int(np.argmax(_closeness(points, coarse, min_distance)))]
    # Refine around the coarse optimum by least squares to the edges. The clamped closeness score
    # is flat within a few degrees of a single visible face, so it cannot refine on its own.
    fine = theta + np.linspace(-2 * step, 2 * step, 41)
    residual = (_edge_distances(points, fine) ** 2).sum(0)
    theta = float(fine[int(np.argmin(residual))]) % (math.pi / 2)

    u1 = np.array([math.cos(theta), math.sin(theta)])
    u2 = np.array([-math.sin(theta), math.cos(theta)])
    a1, a2 = points @ u1, points @ u2
    lo1, hi1, lo2, hi2 = a1.min(), a1.max(), a2.min(), a2.max()
    length, width = max(hi1 - lo1, min_size), max(hi2 - lo2, min_size)
    m1, m2 = (lo1 + hi1) / 2, (lo2 + hi2) / 2
    centre = m1 * u1 + m2 * u2
    error = float(np.minimum(np.minimum(hi1 - a1, a1 - lo1),
                             np.minimum(hi2 - a2, a2 - lo2)).mean())
    return Box(float(centre[0]), float(centre[1]), theta,
               float(length), float(width), len(points), error)


def detect_boxes(ranges, angle_min, angle_increment, range_min, range_max, max_extent=3.0,
                 **segment_args):
    """Full pipeline for one scan. Boxes longer than `max_extent` (walls) are dropped."""
    pts, idx = scan_points(ranges, angle_min, angle_increment, range_min, range_max)
    boxes = []
    for seg in segment(pts, idx, angle_increment, len(ranges), **segment_args):
        box = fit_box(pts[seg])
        if max(box.length, box.width) <= max_extent:
            boxes.append(box)
    return boxes
