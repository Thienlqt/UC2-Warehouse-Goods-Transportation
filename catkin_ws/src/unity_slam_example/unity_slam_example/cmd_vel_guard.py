"""Last-resort stop between move_base and the robot: /cmd_vel_nav + /scan -> /cmd_vel.

Slows or stops commands that would hit the scan (see guard_math). Publishes zero velocity while
the scan is older than source_timeout, so a stalled lidar never leaves the robot driving blind.
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .guard_math import guard_twist, scan_points
from .perception.ros_utils import quaternion_to_yaw


class CmdVelGuard:
    def __init__(self):
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.radius = rospy.get_param('~robot_radius', 0.22)
        self.horizon = rospy.get_param('~time_before_collision', 1.2)
        self.step = rospy.get_param('~simulation_time_step', 0.1)
        self.min_points = rospy.get_param('~min_points', 6)
        self.source_timeout = rospy.get_param('~source_timeout', 1.0)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)
        self.mount = None       # laser pose (x, y, yaw) in base_frame; the lidar is fixed
        self.scan = None        # (stamp, points in base_frame)
        self.warned = False
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('scan', LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber('cmd_vel_nav', Twist, self.on_cmd, queue_size=10)

    def on_scan(self, scan):
        if self.mount is None:
            try:
                t = self.buffer.lookup_transform(self.base_frame, scan.header.frame_id,
                                                 rospy.Time(0)).transform
            except TransformException:
                return
            self.mount = (t.translation.x, t.translation.y, quaternion_to_yaw(t.rotation))
        points = scan_points(scan.ranges, scan.angle_min, scan.angle_increment,
                             scan.range_min, scan.range_max, *self.mount)
        self.scan = (scan.header.stamp, points)

    def on_cmd(self, cmd):
        scan = self.scan
        out = Twist()
        if scan is None or (rospy.Time.now() - scan[0]).to_sec() > self.source_timeout:
            if not self.warned:
                rospy.logwarn('No fresh /scan in %.1f s: holding the robot still' %
                              self.source_timeout)
                self.warned = True
        else:
            self.warned = False
            out.linear.x, out.angular.z = guard_twist(
                cmd.linear.x, cmd.angular.z, scan[1], self.radius, self.horizon, self.step,
                self.min_points)
        self.publisher.publish(out)


def main():
    rospy.init_node('cmd_vel_guard')
    CmdVelGuard()
    rospy.spin()
