"""Small ROS message helpers shared by the perception nodes."""

import math

from geometry_msgs.msg import Quaternion


def yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_to_seconds(stamp):
    return stamp.to_sec()
