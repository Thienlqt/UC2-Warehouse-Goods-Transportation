"""Publish simulation ground-truth odometry from Unity's existing TF tree.

This node does not broadcast TF or estimate wheel odometry. A real robot should
provide its own measured/fused odometry and disable this simulation helper.
"""

import math

import rospy
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException, TransformListener


def planar_velocity(previous, current):
    """Return body-frame vx, vy, yaw rate for (seconds, x, y, yaw) samples."""
    dt = current[0] - previous[0]
    if dt <= 0.0 or dt > 0.5:
        return None
    vx = (current[1] - previous[1]) / dt
    vy = (current[2] - previous[2]) / dt
    yaw = current[3]
    delta = math.atan2(math.sin(yaw - previous[3]), math.cos(yaw - previous[3]))
    return (math.cos(yaw) * vx + math.sin(yaw) * vy,
            -math.sin(yaw) * vx + math.cos(yaw) * vy, delta / dt)


class TfOdometry:
    def __init__(self):
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)
        self.publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.previous = None
        self.timer = rospy.Timer(rospy.Duration(0.05), lambda _: self.publish_odometry())

    def publish_odometry(self):
        try:
            tf = self.buffer.lookup_transform(self.odom_frame, self.base_frame, rospy.Time(0))
        except TransformException:
            self.previous = None
            return
        stamp = tf.header.stamp
        age = (rospy.Time.now() - stamp).to_sec()
        if age < 0.0 or age > 0.5:
            self.previous = None
            return
        p, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        current = (stamp.to_sec(), p.x, p.y, yaw)
        if self.previous is not None and current[0] == self.previous[0]:
            return  # Never relabel an old transform as a fresh measurement.
        velocity = None if self.previous is None else planar_velocity(self.previous, current)
        self.previous = current
        if velocity is None:
            return  # Require two fresh samples after startup, a gap, or a clock reset.
        msg = Odometry()
        msg.header = tf.header
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = p.x
        msg.pose.pose.position.y = p.y
        msg.pose.pose.position.z = p.z
        msg.pose.pose.orientation = q
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = velocity
        self.publisher.publish(msg)


def main():
    rospy.init_node('unity_tf_odometry')
    TfOdometry()
    rospy.spin()
