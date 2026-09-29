"""ROS node: /scan -> oriented, tracked obstacle boxes in the odom frame (/obstacles_lidar).

Every box is class "obstacle" with a geometric confidence; the fusion node adds camera labels.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from .lidar_segmentation import detect_boxes
from .ros_utils import quaternion_to_yaw, stamp_to_seconds, yaw_to_quaternion
from .tracking import boxes_to_frame, Tracker


class LidarObstacles(Node):
    def __init__(self):
        super().__init__('lidar_obstacles')
        self.fixed_frame = self.declare_parameter('fixed_frame', 'odom').value
        self.max_extent = self.declare_parameter('max_extent', 3.0).value
        self.box_height = self.declare_parameter('box_height', 0.5).value
        self.segment_args = {
            'breakpoint_angle_deg': self.declare_parameter('breakpoint_angle_deg', 10.0).value,
            'sigma': self.declare_parameter('sigma', 0.03).value,
            'min_points': self.declare_parameter('min_points', 3).value,
        }
        self.tracker = Tracker(gate=self.declare_parameter('track_gate', 0.6).value,
                               max_misses=self.declare_parameter('max_misses', 3).value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Detection3DArray, '/obstacles_lidar', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def lookup(self, scan):
        """Sensor pose in the fixed frame at the scan time (latest if not yet available)."""
        for when in (Time.from_msg(scan.header.stamp), Time()):
            try:
                return self.buffer.lookup_transform(self.fixed_frame, scan.header.frame_id, when,
                                                    Duration(seconds=0.05))
            except TransformException:
                continue
        return None

    def on_scan(self, scan):
        tf = self.lookup(scan)
        if tf is None:
            return
        boxes = detect_boxes(scan.ranges, scan.angle_min, scan.angle_increment,
                             scan.range_min, scan.range_max, self.max_extent, **self.segment_args)
        t = tf.transform
        boxes = boxes_to_frame(boxes, t.translation.x, t.translation.y,
                               quaternion_to_yaw(t.rotation))
        tracks = self.tracker.step(boxes, stamp_to_seconds(scan.header.stamp))

        out = Detection3DArray()
        out.header.stamp = scan.header.stamp
        out.header.frame_id = self.fixed_frame
        for track in tracks:
            det = Detection3D()
            det.header = out.header
            det.id = str(track.id)
            det.bbox.center.position.x = float(track.x[0])
            det.bbox.center.position.y = float(track.x[1])
            det.bbox.center.position.z = self.box_height / 2
            det.bbox.center.orientation = yaw_to_quaternion(track.yaw)
            det.bbox.size.x = float(track.length)
            det.bbox.size.y = float(track.width)
            det.bbox.size.z = float(self.box_height)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'obstacle'
            hypothesis.hypothesis.score = track.confidence()
            # Velocity rides in the hypothesis pose (vision_msgs has no velocity field).
            hypothesis.pose.pose.position.x = float(track.x[2])
            hypothesis.pose.pose.position.y = float(track.x[3])
            det.results.append(hypothesis)
            out.detections.append(det)
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
