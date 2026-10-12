"""Check that the ROS adapter dispatches one move_base goal at a time."""

import json
import threading
from collections import deque

import rospy
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

from unity_slam_example.delivery_controller import DeliveryController
from unity_slam_example.delivery_queue import DeliveryQueue, parse_request


class ImmediateMoveBase:
    def __init__(self, outcomes):
        self.outcomes = deque(outcomes)
        self.goals = []
        self.cancel_calls = 0
        self.on_send = None
        self.done = None

    def wait_for_server(self, _timeout):
        return True

    def send_goal(self, goal, done_cb, feedback_cb):
        self.goals.append(goal)
        self.done = done_cb
        if self.on_send:
            self.on_send()
        elif self.outcomes:
            done_cb(self.outcomes.popleft(), None)

    def cancel_goal(self):
        self.cancel_calls += 1
        self.done(GoalStatus.PREEMPTED, None)


def request(identifier, x):
    return parse_request(json.dumps({
        "id": identifier,
        "pickup": {"x": x, "y": 0, "yaw": 0},
        "dropoff": {"x": x + 1, "y": 0, "yaw": 1.57},
        "home": {"x": 0, "y": 0, "yaw": 0},
    }))


def controller(monkeypatch, outcomes):
    node = DeliveryController.__new__(DeliveryController)
    node.queue = DeliveryQueue()
    node.goal_timeout = 1.0
    node.lock = threading.RLock()
    node.wake = threading.Event()
    node.cancel_active = threading.Event()
    node.client = ImmediateMoveBase(outcomes)
    node.statuses = []
    node.snapshots = []
    node.publish_status = lambda identifier, state, stage=None, detail="": (
        node.statuses.append((identifier, state, stage, detail)))
    node.publish_queue = lambda: node.snapshots.append(node.queue.snapshot())
    monkeypatch.setattr(rospy.Time, "now", classmethod(lambda cls: cls(123)))
    monkeypatch.setattr(rospy, "is_shutdown", lambda: not node.queue.active and not node.queue.pending)
    return node


def test_successful_requests_dispatch_all_goals_in_fifo_order(monkeypatch):
    node = controller(monkeypatch, [GoalStatus.SUCCEEDED] * 6)
    node.queue.enqueue(request("first", 1))
    node.queue.enqueue(request("second", 5))

    node.run()

    assert [goal.target_pose.pose.position.x for goal in node.client.goals] == [1, 2, 0, 5, 6, 0]
    assert all(goal.target_pose.header.frame_id == "map" for goal in node.client.goals)
    assert [(identifier, state) for identifier, state, _, _ in node.statuses
            if state == "completed"] == [("first", "completed"), ("second", "completed")]
    assert node.snapshots[-1] == {"active": None, "pending": []}


def test_failed_dropoff_skips_home_and_starts_next_request(monkeypatch):
    node = controller(monkeypatch, [GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                                    GoalStatus.SUCCEEDED, GoalStatus.SUCCEEDED,
                                    GoalStatus.SUCCEEDED])
    node.queue.enqueue(request("first", 1))
    node.queue.enqueue(request("second", 5))

    node.run()

    assert [goal.target_pose.pose.position.x for goal in node.client.goals] == [1, 2, 5, 6, 0]
    assert any(item[:3] == ("first", "failed", "dropoff") for item in node.statuses)
    assert any(item[:3] == ("second", "completed", "home") for item in node.statuses)


def test_cancel_active_goal_stops_its_delivery_and_preserves_next(monkeypatch):
    node = controller(monkeypatch, [GoalStatus.SUCCEEDED] * 3)
    node.queue.enqueue(request("first", 1))
    node.queue.enqueue(request("second", 5))

    def on_send():
        node.client.on_send = None
        node.on_cancel(String(data="first"))

    node.client.on_send = on_send
    node.run()

    assert node.client.cancel_calls == 1
    assert [goal.target_pose.pose.position.x for goal in node.client.goals] == [1, 5, 6, 0]
    assert any(item[:3] == ("first", "canceled", "pickup") for item in node.statuses)
    assert any(item[:3] == ("second", "completed", "home") for item in node.statuses)


def test_unresponsive_goal_times_out_and_is_canceled(monkeypatch):
    node = controller(monkeypatch, [])
    node.queue.enqueue(request("stuck", 1))
    node.queue.begin_next()
    node.goal_timeout = 0.01
    node.client.on_send = lambda: None

    outcome, detail = node.run_leg("stuck", "pickup", node.queue.active.pickup)

    assert outcome == "failed"
    assert "timed out" in detail
    assert node.client.cancel_calls == 1
