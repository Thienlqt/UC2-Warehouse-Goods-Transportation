"""Multi-object tracking of lidar boxes: constant-velocity Kalman filter + Hungarian matching.

Pure numpy/scipy, no ROS. Boxes must already be in a fixed frame (odom), so a moving robot does
not look like moving obstacles.

The Kalman velocity alone is not trusted: a lidar box's centre is the centre of the part that is
visible, so it shifts whenever a face comes into view or a segment splits or merges, even on a
static box. A track counts as moving only when the scan shows it moved through space
(scan_motion_evidence): a later scan sees free space where its earlier points were, or its points
now lie where an earlier scan saw free space. The second catches objects coming straight at the
sensor, whose earlier points are hidden behind them. Otherwise its velocity is reported as zero.
"""

from dataclasses import dataclass, field
import itertools
import math

import numpy as np
from scipy.optimize import linear_sum_assignment

from .lidar_segmentation import Box


@dataclass
class Track:
    id: int
    x: np.ndarray                 # state [px, py, vx, vy]
    P: np.ndarray                 # 4x4 covariance
    yaw: float
    length: float
    width: float
    num_points: int
    fit_error: float
    hits: int = 1
    misses: int = 0
    age: int = 1
    stamp: float = 0.0            # time of the last update (s)
    moving: bool = False          # the scan confirmed that it moves (see Tracker.step)
    history: list = field(default_factory=list, repr=False)   # [(stamp, points in fixed frame)]

    @property
    def confirmed(self):
        return self.hits >= 2

    @property
    def speed(self):
        return float(math.hypot(self.x[2], self.x[3]))

    @property
    def velocity(self):
        """(vx, vy) to publish: the Kalman velocity if the track moves, otherwise zero."""
        return (float(self.x[2]), float(self.x[3])) if self.moving else (0.0, 0.0)

    def confidence(self):
        """Geometric confidence in [0, 1]: enough points, a good rectangle fit, seen repeatedly."""
        points = min(1.0, self.num_points / 10.0)
        fit = max(0.0, 1.0 - self.fit_error / 0.05)
        persistence = min(1.0, self.hits / 5.0)
        return round(0.4 * points + 0.2 * fit + 0.4 * persistence, 3)


@dataclass
class Tracker:
    gate: float = 0.6              # max centre distance (m) for a box to update a track
    max_misses: int = 3            # scans without a match before a track is dropped
    process_noise: float = 1.0     # acceleration noise (m/s^2)
    measurement_noise: float = 0.05
    # Motion check: compare a track's points from motion_min_age..motion_max_age seconds ago with
    # the new scan. Moving needs min_evidence points seen through, and at least moved_fraction of
    # the points that gave evidence; static needs min_evidence points still in place.
    motion_min_age: float = 0.4
    motion_max_age: float = 1.0
    min_evidence: int = 3
    moved_fraction: float = 0.25
    tracks: list = field(default_factory=list)
    _ids: itertools.count = field(default_factory=lambda: itertools.count(1))
    _last_stamp: float = None
    _scans: list = field(default_factory=list)  # [(stamp, evidence)] of recent scans

    def _predict(self, dt):
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = self.process_noise ** 2
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        Q = G @ G.T * q
        for t in self.tracks:
            t.x = F @ t.x
            t.P = F @ t.P @ F.T + Q

    def _update(self, track, box, stamp):
        H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        R = np.eye(2) * self.measurement_noise ** 2
        z = np.array([box.x, box.y])
        S = H @ track.P @ H.T + R
        K = track.P @ H.T @ np.linalg.inv(S)
        track.x = track.x + K @ (z - H @ track.x)
        track.P = (np.eye(4) - K @ H) @ track.P
        # Shape: smooth size, take the latest heading (rectangle yaw is ambiguous modulo 90 deg).
        a = 0.5
        track.length = a * box.length + (1 - a) * track.length
        track.width = a * box.width + (1 - a) * track.width
        track.yaw, track.num_points, track.fit_error = box.yaw, box.num_points, box.fit_error
        track.hits += 1
        track.misses = 0
        track.stamp = stamp

    def step(self, boxes, stamp, evidence=None):
        """Advance to `stamp` (s) with the boxes observed then; returns the live tracks.

        `evidence(points)` -> (moved, stayed) checks earlier points of a track (fixed frame)
        against the scan these boxes came from (scan_motion_evidence). The tracker keeps it for
        motion_max_age, to check later points against this scan too. Without it, motion cannot
        be checked and every confirmed track is taken as moving at its Kalman velocity.
        """
        dt = 0.0 if self._last_stamp is None else max(0.0, stamp - self._last_stamp)
        if self._last_stamp is not None and stamp < self._last_stamp - 1.0:
            self.tracks.clear()    # simulation clock reset (Play restarted)
            self._scans.clear()
            dt = 0.0
        self._last_stamp = stamp
        self._predict(dt)
        # The oldest scan within the motion window: the largest displacement since.
        past = next((e for t, e in self._scans
                     if self.motion_min_age <= stamp - t <= self.motion_max_age), None)

        matched_tracks, matched_boxes = set(), set()
        if self.tracks and boxes:
            cost = np.array([[math.hypot(t.x[0] - b.x, t.x[1] - b.y) for b in boxes]
                             for t in self.tracks])
            for ti, bi in zip(*linear_sum_assignment(cost)):
                if cost[ti, bi] <= self.gate:
                    self._update(self.tracks[ti], boxes[bi], stamp)
                    self._check_motion(self.tracks[ti], boxes[bi], stamp, evidence, past)
                    matched_tracks.add(ti)
                    matched_boxes.add(bi)

        for ti, t in enumerate(self.tracks):
            t.age += 1
            if ti not in matched_tracks:
                t.misses += 1
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]

        for bi, b in enumerate(boxes):
            if bi not in matched_boxes:
                P = np.diag([self.measurement_noise ** 2] * 2 + [1.0, 1.0])
                self.tracks.append(Track(next(self._ids), np.array([b.x, b.y, 0.0, 0.0]), P,
                                         b.yaw, b.length, b.width, b.num_points, b.fit_error,
                                         stamp=stamp))
                self._check_motion(self.tracks[-1], b, stamp, evidence, past)
        if evidence is not None:
            self._scans = [(t, e) for t, e in self._scans if stamp - t <= self.motion_max_age]
            self._scans.append((stamp, evidence))
        return [t for t in self.tracks if t.confirmed]

    def _check_motion(self, track, box, stamp, evidence, past=None):
        """Update track.moving from the scan, then remember the box's points for later checks.

        Evidence both ways: the track's earlier points against this scan (has it left where it
        was?) and its points now against an earlier scan `past` (was this space free before?).
        """
        if evidence is None:
            track.moving = True
            return
        moved = stayed = 0
        old = [p for t, p in track.history
               if self.motion_min_age <= stamp - t <= self.motion_max_age]
        if old:
            moved, stayed = evidence(old[0])        # the oldest: the largest displacement
        if past is not None and box.points is not None:
            entered, still = past(box.points)
            moved, stayed = moved + entered, stayed + still
        if moved or stayed:
            if moved >= self.min_evidence and moved >= self.moved_fraction * (moved + stayed):
                track.moving = True
            elif stayed >= self.min_evidence and moved < self.moved_fraction * (moved + stayed):
                track.moving = False
            # Too little evidence (e.g. the old points are now hidden): keep the last verdict.
        if box.points is not None:
            track.history = [(t, p) for t, p in track.history if stamp - t <= self.motion_max_age]
            track.history.append((stamp, box.points))


def scan_motion_evidence(points, sensor_pose, ranges, angle_min, angle_increment, range_min,
                         range_max, margin=0.15):
    """Check earlier object points (Nx2, fixed frame) against a new scan: (moved, stayed) counts.

    moved: both beams either side of the point's bearing now pass more than `margin` beyond it
    (or return nothing), so the space it occupied is free: the object has left it. A static
    surface cannot do this, however its visible extent changes.
    stayed: a beam either side still returns within `margin` of the point's range.
    A point now hidden behind something nearer, or outside the scan, gives no evidence: an object
    coming straight at the sensor hides its own earlier points. Tracker therefore also checks
    an object's new points against an earlier scan (moved: that scan saw through them).
    `sensor_pose` is the scan's (x, y, yaw) in the fixed frame.
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    r = np.asarray(ranges, dtype=np.float64)
    if not len(pts) or not len(r) or angle_increment == 0:
        return 0, 0
    sx, sy, syaw = sensor_pose
    c, s = math.cos(syaw), math.sin(syaw)
    dx, dy = pts[:, 0] - sx, pts[:, 1] - sy
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    old_r = np.hypot(lx, ly)
    # Fractional beam index of each bearing.
    step = abs(angle_increment)
    rel = (np.sign(angle_increment) * (np.arctan2(ly, lx) - angle_min)) % (2 * math.pi)
    i0 = np.floor(rel / step).astype(int)
    i1 = i0 + 1
    n = len(r)
    if n * step >= 2 * math.pi - 2 * step:      # full circle: the last beam neighbours the first
        i0, i1, inside = i0 % n, i1 % n, np.ones(len(pts), bool)
    else:
        inside = i1 < n
        i0, i1 = np.minimum(i0, n - 1), np.minimum(i1, n - 1)
    # No return within range: free out to range_max. NaN or below range_min: unknown.
    free = np.where(np.isfinite(r) & (r >= range_min) & (r <= range_max), r,
                    np.where(np.isnan(r) | (r < range_min), np.nan, np.inf))
    ra, rb = free[i0], free[i1]
    with np.errstate(invalid='ignore'):
        moved = inside & (ra > old_r + margin) & (rb > old_r + margin) & \
            (old_r + margin < range_max)
        stayed = inside & ~moved & ((np.abs(ra - old_r) <= margin) | (np.abs(rb - old_r) <= margin))
    return int(moved.sum()), int(stayed.sum())


def mask_returns(ranges, angle_min, angle_increment, sensor_pose, boxes, margin=0.1):
    """Copy of `ranges` with NaN (no reading) for returns inside any of `boxes`.

    boxes: (x, y, yaw, length, width) in the fixed frame, each grown by `margin` on every side.
    `sensor_pose` is the scan's (x, y, yaw) in the same frame. SLAM given this scan neither marks
    nor clears anything where moving objects are, so they leave no trail in the map.
    """
    r = np.array(ranges, dtype=np.float64)
    if not len(boxes) or not len(r):
        return r
    sx, sy, syaw = sensor_pose
    a = syaw + angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r)
    d = np.where(ok, r, 0.0)
    px, py = sx + d * np.cos(a), sy + d * np.sin(a)
    inside = np.zeros(len(r), bool)
    for x, y, yaw, length, width in boxes:
        c, s = math.cos(yaw), math.sin(yaw)
        along, across = c * (px - x) + s * (py - y), -s * (px - x) + c * (py - y)
        inside |= (np.abs(along) <= length / 2 + margin) & (np.abs(across) <= width / 2 + margin)
    r[ok & inside] = np.nan
    return r


def boxes_to_frame(boxes, tx, ty, yaw):
    """Transform boxes by a planar pose (sensor frame -> fixed frame)."""
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return [Box(tx + c * b.x - s * b.y, ty + s * b.x + c * b.y, (b.yaw + yaw) % (math.pi / 2),
                b.length, b.width, b.num_points, b.fit_error,
                None if b.points is None else np.asarray(b.points) @ rot.T + (tx, ty))
            for b in boxes]
