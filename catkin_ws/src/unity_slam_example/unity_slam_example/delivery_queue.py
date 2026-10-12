"""Validated delivery requests and a FIFO, three-stop task queue.

This module has no ROS dependency so queue behavior can be checked without a running robot.
"""

import json
import math
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class MapPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class DeliveryRequest:
    request_id: str
    pickup: MapPose
    dropoff: MapPose
    home: MapPose


def _parse_pose(value, field):
    if not isinstance(value, dict):
        raise ValueError("%s must be an object with x, y and yaw" % field)
    try:
        coordinates = [float(value[key]) for key in ("x", "y", "yaw")]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("%s requires numeric x, y and yaw" % field) from exc
    if not all(math.isfinite(number) for number in coordinates):
        raise ValueError("%s coordinates must be finite" % field)
    return MapPose(*coordinates)


def parse_request(raw):
    try:
        payload = json.loads(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError("request must be valid JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError("request must be a JSON object")
    request_id = payload.get("id")
    if not isinstance(request_id, str) or not request_id.strip():
        raise ValueError("id must be a non-empty string")
    request_id = request_id.strip()
    if len(request_id) > 128:
        raise ValueError("id must be at most 128 characters")
    return DeliveryRequest(
        request_id,
        _parse_pose(payload.get("pickup"), "pickup"),
        _parse_pose(payload.get("dropoff"), "dropoff"),
        _parse_pose(payload.get("home"), "home"),
    )


class DeliveryQueue:
    """One active delivery; pickup, dropoff and home run in that order."""

    def __init__(self, max_pending=50):
        if max_pending < 1:
            raise ValueError("max_pending must be positive")
        self.max_pending = max_pending
        self.pending = deque()
        self.active = None
        self.stage = None

    def enqueue(self, request):
        if self.active and self.active.request_id == request.request_id:
            raise ValueError("request id is already active")
        if any(item.request_id == request.request_id for item in self.pending):
            raise ValueError("request id is already queued")
        if len(self.pending) >= self.max_pending:
            raise ValueError("delivery queue is full")
        self.pending.append(request)

    def begin_next(self):
        if self.active is not None or not self.pending:
            return None
        self.active = self.pending.popleft()
        self.stage = "pickup"
        return self.active, self.stage, self.active.pickup

    def finish_leg(self, succeeded):
        """Return the next leg, or None after a terminal result."""
        if self.active is None:
            raise RuntimeError("no active delivery")
        if succeeded and self.stage == "pickup":
            self.stage = "dropoff"
            return self.active, self.stage, self.active.dropoff
        if succeeded and self.stage == "dropoff":
            self.stage = "home"
            return self.active, self.stage, self.active.home
        self.active = None
        self.stage = None
        return None

    def cancel_pending(self, request_id):
        canceled = [item for item in self.pending
                    if request_id == "*" or item.request_id == request_id]
        self.pending = deque(item for item in self.pending if item not in canceled)
        return canceled

    def snapshot(self):
        return {
            "active": None if self.active is None else {
                "id": self.active.request_id, "stage": self.stage},
            "pending": [item.request_id for item in self.pending],
        }
