"""ROS node: /scan -> oriented, tracked obstacle boxes in the odom frame (/obstacles_lidar).

Every box is class "obstacle" with a geometric confidence; the fusion node adds camera labels.
A track's velocity is published only once the scan confirms that it moves (see tracking).
Also publishes /scan_static: the scan without the returns from moving tracks, for SLAM, so
moving obstacles leave no trail in the map. The costmaps keep using /scan.
"""

import copy

import rospy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from uc2_vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from .lidar_segmentation import detect_boxes
from .ros_utils import quaternion_to_yaw, stamp_to_seconds, yaw_to_quaternion
from .tracking import boxes_to_frame, mask_returns, scan_motion_evidence, Tracker


class LidarObstacles:
    def __init__(self):
        self.fixed_frame = rospy.get_param('~fixed_frame', 'odom')
        self.max_extent = rospy.get_param('~max_extent', 3.0)
        self.box_height = rospy.get_param('~box_height', 0.5)
        self.segment_args = {
            'breakpoint_angle_deg': rospy.get_param('~breakpoint_angle_deg', 10.0),
            'sigma': rospy.get_param('~sigma', 0.03),
            'min_points': rospy.get_param('~min_points', 3),
        }
        self.tracker = Tracker(gate=rospy.get_param('~track_gate', 0.6),
                               max_misses=rospy.get_param('~max_misses', 3),
                               motion_min_age=rospy.get_param('~motion_min_age', 0.4),
                               motion_max_age=rospy.get_param('~motion_max_age', 1.0),
                               min_evidence=rospy.get_param('~motion_min_evidence', 3),
                               moved_fraction=rospy.get_param('~motion_moved_fraction', 0.25))
        self.motion_margin = rospy.get_param('~motion_margin', 0.15)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)
        self.mask_margin = rospy.get_param('~static_scan_margin', 0.1)
        self.publisher = rospy.Publisher('/obstacles_lidar', Detection3DArray, queue_size=10)
        self.static_publisher = rospy.Publisher('/scan_static', LaserScan, queue_size=5)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=5)

    def lookup(self, scan):
        """Sensor pose in the fixed frame at the scan time (latest if not yet available)."""
        for when in (scan.header.stamp, rospy.Time(0)):
            try:
                return self.buffer.lookup_transform(self.fixed_frame, scan.header.frame_id, when,
                                                    rospy.Duration(0.05))
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
        pose = (t.translation.x, t.translation.y, quaternion_to_yaw(t.rotation))
        boxes = boxes_to_frame(boxes, *pose)

        def evidence(points):
            return scan_motion_evidence(points, pose, scan.ranges, scan.angle_min,
                                        scan.angle_increment, scan.range_min, scan.range_max,
                                        self.motion_margin)
        tracks = self.tracker.step(boxes, stamp_to_seconds(scan.header.stamp), evidence)

        static = copy.copy(scan)
        static.ranges = mask_returns(
            scan.ranges, scan.angle_min, scan.angle_increment, pose,
            [(t.x[0], t.x[1], t.yaw, t.length, t.width) for t in tracks if t.moving],
            self.mask_margin).tolist()
        self.static_publisher.publish(static)

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
            hypothesis.pose.pose.position.x, hypothesis.pose.pose.position.y = track.velocity
            det.results.append(hypothesis)
            out.detections.append(det)
        self.publisher.publish(out)


def main():
    rospy.init_node('lidar_obstacles')
    LidarObstacles()
    rospy.spin()
