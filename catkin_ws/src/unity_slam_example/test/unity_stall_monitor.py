#!/usr/bin/env python3
"""Measure stalls while driving in Unity, the way test/scenarios.py scores its trials.

Run in a ROS shell during Play, then send 2D Nav Goals (or deliveries):
  python3 test/unity_stall_monitor.py
Prints one line per goal: result, time, the longest stall and traffic waits (stall_report), and
the closest scan point to the robot's body (0 or less: contact). Ctrl+C prints a summary.

Traffic waits: Unity's movers are not published to ROS, so a mover is an /obstacles track that
has been seen moving at some point (a cart that stops keeps its track); shelves never move.
"""

import math

import rospy
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from uc2_vision_msgs.msg import Detection3DArray

from scenarios import RADIUS, stall_report, traffic_blocking
from unity_slam_example.avoidance_math import box_outline, speed
from unity_slam_example.guard_math import scan_points
from unity_slam_example.perception.ros_utils import quaternion_to_yaw
from unity_slam_example.predicted_obstacles import tracks_from_detections

FIXED, BASE = 'odom', 'base_link'


class Monitor:
    def __init__(self, min_speed=0.15):
        self.min_speed = min_speed
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)
        self.v, self.tracks, self.ever_moving = 0.0, [], set()
        self.mount, self.clearance = None, float('inf')
        self.goal = None            # {'xy': odom goal, 'start': s, 'samples': [], 'clear': m}
        self.last_t, self.done = None, []
        rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber('/obstacles', Detection3DArray, self.on_obstacles, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.on_goal, queue_size=5)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.on_result, queue_size=5)
        rospy.Timer(rospy.Duration(0.1), self.sample)

    def on_odom(self, msg):
        self.v = msg.twist.twist.linear.x

    def on_obstacles(self, msg):
        ids = [d.id for d in msg.detections if d.results]
        self.tracks = list(zip(ids, tracks_from_detections(msg)))
        self.ever_moving |= {i for i, t in self.tracks if speed(t) >= self.min_speed}

    def on_scan(self, scan):
        if self.mount is None:
            try:
                t = self.buffer.lookup_transform(BASE, scan.header.frame_id, rospy.Time(0))
            except TransformException:
                return
            r = t.transform
            self.mount = (r.translation.x, r.translation.y, quaternion_to_yaw(r.rotation))
        points = scan_points(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
                             scan.range_max, *self.mount)
        if len(points):
            self.clearance = float(min(math.hypot(x, y) for x, y in points)) - RADIUS

    def pose(self, frame, child, stamp=rospy.Time(0)):
        t = self.buffer.lookup_transform(frame, child, stamp).transform
        return t.translation.x, t.translation.y, quaternion_to_yaw(t.rotation)

    def on_goal(self, msg):
        target = msg.goal.target_pose
        try:
            gx, gy, gyaw = self.pose(FIXED, target.header.frame_id or 'map')
        except TransformException as e:
            rospy.logwarn('Goal ignored, no %s transform: %s', FIXED, e)
            return
        px, py = target.pose.position.x, target.pose.position.y
        c, s = math.cos(gyaw), math.sin(gyaw)
        self.goal = {'xy': (gx + c * px - s * py, gy + s * px + c * py),
                     'start': rospy.Time.now().to_sec(), 'samples': [], 'clear': float('inf')}

    def sample(self, _):
        now = rospy.Time.now().to_sec()
        if self.last_t is not None and now < self.last_t - 1.0:
            rospy.logwarn('Clock jumped back (Play restarted): dropping the open goal')
            self.goal = None
        self.last_t = now
        goal = self.goal
        if goal is None:
            return
        try:
            x, y, yaw = self.pose(FIXED, BASE)
        except TransformException:
            return
        movers = [box_outline(t) for i, t in self.tracks if i in self.ever_moving]
        goal['samples'].append((now - goal['start'], x, y, self.v,
                                traffic_blocking(x, y, yaw, movers)))
        goal['clear'] = min(goal['clear'], self.clearance)

    def on_result(self, msg):
        goal, self.goal = self.goal, None
        if goal is None or not goal['samples']:
            return
        report = stall_report(goal['samples'], goal['xy'])
        result = dict(report, ok=msg.status.status == 3, time=goal['samples'][-1][0],
                      clearance=goal['clear'], status=msg.status.text)
        self.done.append(result)
        print('%s  %5.1f s  longest stall %4.1f s (%d over 3 s)  waits %d, longest %4.1f s, '
              'total %4.1f s  clearance %5.2f m  %s'
              % ('ok  ' if result['ok'] else 'FAIL', result['time'], result['longest_stall'],
                 result['stalls_over_3s'], result['waits'], result['wait_longest'],
                 result['wait_total'], result['clearance'], result['status']), flush=True)

    def summary(self):
        if not self.done:
            print('No goals finished.')
            return
        print('\n%d goals, %d reached; longest stall %.1f s; %d stalls over 3 s; longest wait '
              '%.1f s; closest %.2f m' % (
                  len(self.done), sum(r['ok'] for r in self.done),
                  max(r['longest_stall'] for r in self.done),
                  sum(r['stalls_over_3s'] for r in self.done),
                  max(r['wait_longest'] for r in self.done),
                  min(r['clearance'] for r in self.done)), flush=True)


def main():
    rospy.init_node('unity_stall_monitor', anonymous=True)
    monitor = Monitor(rospy.get_param('~min_speed', 0.15))
    rospy.on_shutdown(monitor.summary)
    rospy.spin()


if __name__ == '__main__':
    main()
