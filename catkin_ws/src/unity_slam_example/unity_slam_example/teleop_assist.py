"""ROS node: assisted keyboard driving, /cmd_vel_teleop -> /cmd_vel_nav (-> cmd_vel_guard).

Unity's first-person drive (P) publishes the driver's keys on /cmd_vel_teleop.
- Forward only (W): hold the heading the robot had when forward was pressed. Steer around
  anything in /scan or the predicted paths of moving tracks within `lookahead` (10 m), then
  swing back to that heading once it is clear (see teleop_math).
- Forward and turning (W + A/D): the turn passes through and the held heading follows it; the
  speed still drops before an obstacle straight ahead.
- Reverse or turning in place: passes through; cmd_vel_guard still guards it.
The first command cancels any move_base goal. When commands stop for teleop_timeout, the node
sends one zero twist and goes quiet, so move_base can drive again.
"""

import math

import numpy as np
import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from uc2_vision_msgs.msg import Detection3DArray

from .avoidance_math import predicted_points
from .guard_math import scan_points
from .perception.ros_utils import quaternion_to_yaw, yaw_to_quaternion
from .predicted_obstacles import MovingTracks
from .teleop_math import assisted_twist, straight_free, wrap


class TeleopAssist:
    def __init__(self):
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.radius = rospy.get_param('~robot_radius', 0.22)
        self.args = {
            'half_width': self.radius + rospy.get_param('~margin', 0.2),
            'lookahead': rospy.get_param('~lookahead', 10.0),
            'span': math.radians(rospy.get_param('~span_deg', 90.0)),
            'resolution': math.radians(rospy.get_param('~resolution_deg', 2.0)),
            'turn_gain': rospy.get_param('~turn_gain', 1.5),
            'max_turn': rospy.get_param('~max_turn', 0.5),
            'stop_distance': rospy.get_param('~stop_distance', 0.3),
            'slow_distance': rospy.get_param('~slow_distance', 1.0),
            'deviation_weight': rospy.get_param('~deviation_weight', 0.5),
            'change_weight': rospy.get_param('~change_weight', 0.2),
        }
        self.prediction = rospy.get_param('~prediction_horizon', 2.0)
        self.teleop_timeout = rospy.get_param('~teleop_timeout', 0.5)
        self.scan_timeout = rospy.get_param('~scan_timeout', 1.0)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)
        self.movers = MovingTracks(self.buffer, self.base_frame,
                                   rospy.get_param('~min_speed', 0.15),
                                   rospy.get_param('~min_confidence', 0.3),
                                   rospy.get_param('~obstacle_timeout', 0.5))
        self.mount = None       # laser pose (x, y, yaw) in base_frame
        self.scan = None        # (stamp, points in base_frame)
        self.command = None     # (receipt time, Twist)
        self.active = False
        self.desired = None     # held heading in odom (rad)
        self.previous = None    # last chosen heading in odom (rad)
        self.publisher = rospy.Publisher('cmd_vel_nav', Twist, queue_size=10)
        self.cancel = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
        self.heading_pub = rospy.Publisher('teleop_assist/heading', PoseStamped, queue_size=1)
        rospy.Subscriber('cmd_vel_teleop', Twist, self.on_teleop, queue_size=1)
        rospy.Subscriber('scan', LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber('obstacles', Detection3DArray, self.movers.on_obstacles, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / rospy.get_param('~rate', 10.0)), self.control)

    def on_teleop(self, msg):
        self.command = (rospy.Time.now(), msg)

    def on_scan(self, scan):
        if self.mount is None:
            try:
                t = self.buffer.lookup_transform(self.base_frame, scan.header.frame_id,
                                                 rospy.Time(0)).transform
            except TransformException:
                return
            self.mount = (t.translation.x, t.translation.y, quaternion_to_yaw(t.rotation))
        self.scan = (scan.header.stamp, scan_points(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
            *self.mount))

    def obstacle_points(self):
        """Scan points plus predicted paths of moving tracks, base frame; None without a scan."""
        scan = self.scan
        if scan is None or abs((rospy.Time.now() - scan[0]).to_sec()) > self.scan_timeout:
            return None
        predicted = predicted_points(self.movers.in_base_frame(), horizon=self.prediction,
                                     min_speed=0.0, min_confidence=0.0, keep_out=self.radius)
        return np.vstack((scan[1], predicted))

    def control(self, _):
        command = self.command
        if command is None:
            return
        if (rospy.Time.now() - command[0]).to_sec() > self.teleop_timeout:
            if self.active:     # the driver let go (or left first-person drive)
                self.publisher.publish(Twist())
                self.active, self.desired, self.previous = False, None, None
            return
        if not self.active:
            self.active = True
            self.cancel.publish(GoalID())   # the driver takes over from navigation
        try:
            pose = self.buffer.lookup_transform(self.odom_frame, self.base_frame, rospy.Time(0))
        except TransformException:
            self.publisher.publish(Twist())
            return
        yaw = quaternion_to_yaw(pose.transform.rotation)
        points = self.obstacle_points()
        cmd, out = command[1], Twist()
        if points is None:
            pass                            # no fresh scan: hold still (the guard would too)
        elif cmd.linear.x > 0.0 and abs(cmd.angular.z) < 1e-3:
            if self.desired is None:
                self.desired = yaw
            previous = None if self.previous is None else wrap(self.previous - yaw)
            v, w, heading, _ = assisted_twist(points, cmd.linear.x, wrap(self.desired - yaw),
                                              previous, **self.args)
            out.linear.x, out.angular.z = v, w
            self.previous = yaw + heading
            self.publish_heading(pose.header.stamp, heading)
        elif cmd.linear.x > 0.0:
            self.desired, self.previous = yaw, None
            free = straight_free(points, self.args['half_width'], self.args['lookahead'])
            slow = (free - self.args['stop_distance']) / self.args['slow_distance']
            out.linear.x = cmd.linear.x * max(0.0, min(1.0, slow))
            out.angular.z = cmd.angular.z
        else:
            self.desired, self.previous = None, None
            out.linear.x, out.angular.z = cmd.linear.x, cmd.angular.z
        self.publisher.publish(out)

    def publish_heading(self, stamp, heading):
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = stamp, self.base_frame
        msg.pose.orientation = yaw_to_quaternion(heading)
        self.heading_pub.publish(msg)


def main():
    rospy.init_node('teleop_assist')
    TeleopAssist()
    rospy.spin()
