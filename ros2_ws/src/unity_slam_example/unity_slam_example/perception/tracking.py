"""Multi-object tracking of lidar boxes: constant-velocity Kalman filter + Hungarian matching.

Pure numpy/scipy, no ROS. Boxes must already be in a fixed frame (odom), so a moving robot does
not look like moving obstacles.
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

    @property
    def confirmed(self):
        return self.hits >= 2

    @property
    def speed(self):
        return float(math.hypot(self.x[2], self.x[3]))

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
    tracks: list = field(default_factory=list)
    _ids: itertools.count = field(default_factory=lambda: itertools.count(1))
    _last_stamp: float = None

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

    def step(self, boxes, stamp):
        """Advance to `stamp` (s) with the boxes observed then; returns the live tracks."""
        dt = 0.0 if self._last_stamp is None else max(0.0, stamp - self._last_stamp)
        if self._last_stamp is not None and stamp < self._last_stamp - 1.0:
            self.tracks.clear()    # simulation clock reset (Play restarted)
            dt = 0.0
        self._last_stamp = stamp
        self._predict(dt)

        matched_tracks, matched_boxes = set(), set()
        if self.tracks and boxes:
            cost = np.array([[math.hypot(t.x[0] - b.x, t.x[1] - b.y) for b in boxes]
                             for t in self.tracks])
            for ti, bi in zip(*linear_sum_assignment(cost)):
                if cost[ti, bi] <= self.gate:
                    self._update(self.tracks[ti], boxes[bi], stamp)
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
        return [t for t in self.tracks if t.confirmed]


def boxes_to_frame(boxes, tx, ty, yaw):
    """Transform boxes by a planar pose (sensor frame -> fixed frame)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [Box(tx + c * b.x - s * b.y, ty + s * b.x + c * b.y, (b.yaw + yaw) % (math.pi / 2),
                b.length, b.width, b.num_points, b.fit_error) for b in boxes]
