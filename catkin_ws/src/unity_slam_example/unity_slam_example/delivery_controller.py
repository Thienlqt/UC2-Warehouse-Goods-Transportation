"""ROS adapter for sequential pickup, dropoff and return-home navigation."""

import json
import math
import threading
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

from unity_slam_example.delivery_queue import DeliveryQueue, parse_request


class DeliveryController:
    def __init__(self):
        self.queue = DeliveryQueue(rospy.get_param("~max_pending", 50))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 300.0))
        if not math.isfinite(self.goal_timeout) or self.goal_timeout <= 0:
            raise ValueError("goal_timeout must be a positive finite number")
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.cancel_active = threading.Event()
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.status_pub = rospy.Publisher("/delivery/status", String, queue_size=20)
        self.queue_pub = rospy.Publisher("/delivery/queue", String, queue_size=1, latch=True)
        rospy.Subscriber("/delivery/request", String, self.on_request, queue_size=20)
        rospy.Subscriber("/delivery/cancel", String, self.on_cancel, queue_size=20)
        self.publish_queue()

    def publish_queue(self):
        with self.lock:
            snapshot = self.queue.snapshot()
        self.queue_pub.publish(String(data=json.dumps(snapshot, sort_keys=True)))

    def publish_status(self, request_id, state, stage=None, detail=""):
        payload = {"id": request_id, "state": state, "stage": stage, "detail": detail}
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        rospy.loginfo("Delivery %s: %s %s %s", request_id, state, stage or "", detail)

    def on_request(self, message):
        request = None
        try:
            request = parse_request(message.data)
            with self.lock:
                self.queue.enqueue(request)
                position = len(self.queue.pending)
        except ValueError as exc:
            self.publish_status(request.request_id if request else "", "rejected",
                                detail=str(exc))
            return
        self.publish_status(request.request_id, "queued", detail="position %d" % position)
        self.publish_queue()
        self.wake.set()

    def on_cancel(self, message):
        request_id = message.data.strip()
        if not request_id:
            self.publish_status("", "rejected", detail="cancel id is empty")
            return
        with self.lock:
            canceled = self.queue.cancel_pending(request_id)
            active = self.queue.active
            if active is not None and (request_id == "*" or active.request_id == request_id):
                self.cancel_active.set()
                active_id = active.request_id
                active_stage = self.queue.stage
            else:
                active_id = None
                active_stage = None
        for item in canceled:
            self.publish_status(item.request_id, "canceled", detail="removed from queue")
        if active_id is not None:
            self.publish_status(active_id, "canceling", active_stage)
        if not canceled and active_id is None:
            self.publish_status(request_id, "not_found")
        self.publish_queue()
        self.wake.set()

    @staticmethod
    def make_goal(pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose.x
        goal.target_pose.pose.position.y = pose.y
        goal.target_pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        return goal

    def run_leg(self, request_id, stage, pose):
        finished = threading.Event()
        result = [None]
        last_feedback = [0.0]

        def on_done(state, _result):
            result[0] = state
            finished.set()

        def on_feedback(feedback):
            now = time.monotonic()
            if finished.is_set() or self.cancel_active.is_set() or now - last_feedback[0] < 2.0:
                return
            last_feedback[0] = now
            position = feedback.base_position.pose.position
            self.publish_status(request_id, "navigating", stage,
                                "robot at (%.2f, %.2f)" % (position.x, position.y))

        if self.cancel_active.is_set():
            return "canceled", "canceled before goal dispatch"
        try:
            self.client.send_goal(self.make_goal(pose), done_cb=on_done,
                                  feedback_cb=on_feedback)
        except Exception as exc:
            return "failed", "could not send move_base goal: %s" % exc
        self.publish_status(request_id, "navigating", stage)
        deadline = time.monotonic() + self.goal_timeout
        while not rospy.is_shutdown():
            if self.cancel_active.is_set():
                self.client.cancel_goal()
                finished.wait(1.0)
                return "canceled", "canceled by request"
            if finished.wait(0.1):
                if result[0] == GoalStatus.SUCCEEDED:
                    return "succeeded", "move_base reached %s" % stage
                return "failed", "move_base status %s" % result[0]
            if time.monotonic() >= deadline:
                self.client.cancel_goal()
                finished.wait(1.0)
                return "failed", "navigation timed out after %.1f s" % self.goal_timeout
        self.client.cancel_goal()
        return "canceled", "ROS is shutting down"

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                # A cancellation may arrive between two legs while no goal is in flight.
                canceled_between_legs = self.cancel_active.is_set() and self.queue.active
                if canceled_between_legs:
                    canceled_id = self.queue.active.request_id
                    canceled_stage = self.queue.stage
                    self.queue.finish_leg(False)
                    self.cancel_active.clear()
                has_work = self.queue.active is not None or bool(self.queue.pending)
            if canceled_between_legs:
                self.publish_status(canceled_id, "canceled", canceled_stage,
                                    "canceled before the next goal")
                self.publish_queue()
                continue
            if not has_work:
                self.wake.wait(0.2)
                self.wake.clear()
                continue
            # /use_sim_time is enabled by the launch file. Wait until Unity publishes /clock.
            if rospy.Time.now().to_sec() == 0 or not self.client.wait_for_server(rospy.Duration(1.0)):
                self.wake.wait(0.2)
                self.wake.clear()
                continue
            with self.lock:
                leg = self.queue.begin_next()
                if leg is None and self.queue.active is not None:
                    active = self.queue.active
                    pose = getattr(active, self.queue.stage)
                    leg = active, self.queue.stage, pose
            if leg is None:
                continue
            request, stage, pose = leg
            self.publish_queue()
            outcome, detail = self.run_leg(request.request_id, stage, pose)
            with self.lock:
                if self.cancel_active.is_set():
                    outcome, detail = "canceled", "canceled by request"
                next_leg = self.queue.finish_leg(outcome == "succeeded")
                self.cancel_active.clear()
            state = ({"pickup": "pickup_complete", "dropoff": "dropoff_complete"}[stage]
                     if next_leg else ("completed" if outcome == "succeeded" else outcome))
            self.publish_status(request.request_id, state, stage, detail)
            self.publish_queue()


def main():
    rospy.init_node("delivery_controller")
    DeliveryController().run()
