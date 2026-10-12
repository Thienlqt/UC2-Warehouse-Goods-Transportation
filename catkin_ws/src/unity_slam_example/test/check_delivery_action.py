#!/usr/bin/env python3
"""ROS topic/action smoke test using a simulated move_base action server.

Run after building the catkin workspace and sourcing its devel/setup.bash.
"""

import json
import os
import subprocess
import threading
import time

import actionlib
import rosgraph
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from std_msgs.msg import String


PORT = 11342
MASTER_URI = "http://127.0.0.1:%d" % PORT


def until(predicate, seconds=15):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


def stop(process):
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def master_ready():
    try:
        return bool(rosgraph.Master("delivery_check").getPid())
    except Exception:
        return False


def main():
    os.environ["ROS_MASTER_URI"] = MASTER_URI
    master = subprocess.Popen(["roscore", "-p", str(PORT)],
                              stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    controller = None
    try:
        assert until(lambda: master.poll() is None and master_ready()), \
            "roscore did not start"
        rospy.init_node("delivery_action_check", disable_signals=True)
        goals = []
        statuses = []
        lock = threading.Lock()

        def on_goal(goal):
            with lock:
                goals.append(goal.target_pose.pose.position.x)
            time.sleep(0.05)
            server.set_succeeded(MoveBaseResult())

        def on_status(message):
            with lock:
                statuses.append(json.loads(message.data))

        server = actionlib.SimpleActionServer("move_base", MoveBaseAction,
                                              execute_cb=on_goal, auto_start=False)
        server.start()
        rospy.Subscriber("/delivery/status", String, on_status, queue_size=20)
        request_pub = rospy.Publisher("/delivery/request", String, queue_size=10)
        controller = subprocess.Popen(["rosrun", "unity_slam_example", "delivery_controller"],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        assert until(lambda: request_pub.get_num_connections() > 0), \
            "delivery controller did not subscribe"

        for identifier, x in (("first", 1), ("second", 5)):
            request_pub.publish(String(data=json.dumps({
                "id": identifier,
                "pickup": {"x": x, "y": 0, "yaw": 0},
                "dropoff": {"x": x + 1, "y": 0, "yaw": 0},
                "home": {"x": 0, "y": 0, "yaw": 0},
            })))

        def both_completed():
            with lock:
                return {item["id"] for item in statuses if item["state"] == "completed"} == \
                    {"first", "second"}

        assert until(both_completed), "both deliveries did not complete"
        with lock:
            assert goals == [1, 2, 0, 5, 6, 0], goals
        print("ROS delivery action check passed: pickup, dropoff, home in FIFO order")
    finally:
        stop(controller)
        rospy.signal_shutdown("done")
        stop(master)


if __name__ == "__main__":
    main()
