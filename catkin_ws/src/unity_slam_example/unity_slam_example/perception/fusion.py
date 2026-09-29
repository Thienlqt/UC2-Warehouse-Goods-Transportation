"""ROS node: fuse camera detections with lidar tracks into named, scored 3D obstacles.

Inputs: /obstacles_lidar (Detection3DArray, odom), /detections_2d, /camera/camera_info.
Outputs: /obstacles (Detection3DArray) and /obstacles_markers (MarkerArray for RViz).
Lidar tracks outside the camera view, or not matched, stay "obstacle" with lidar confidence.
"""

import math
import threading

import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener
from uc2_vision_msgs.msg import Detection2DArray, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from .fusion_math import associate, box_corners, fused_score, project_interval
from .ros_utils import quaternion_to_yaw, stamp_to_seconds

COLOURS = {'obstacle': (0.6, 0.6, 0.6), 'person': (1.0, 0.2, 0.2), 'box': (1.0, 0.6, 0.1),
           'shelf': (0.2, 0.5, 1.0), 'station': (0.2, 0.8, 0.3)}


def colour_for(label):
    if label in COLOURS:
        return COLOURS[label]
    h = hash(label) % 360 / 360.0
    return (0.5 + 0.5 * math.cos(2 * math.pi * h), 0.5 + 0.5 * math.cos(2 * math.pi * (h + 1 / 3)),
            0.5 + 0.5 * math.cos(2 * math.pi * (h + 2 / 3)))


def transform_matrix(tf):
    q, t = tf.transform.rotation, tf.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R, np.array([t.x, t.y, t.z])


class Fusion:
    def __init__(self):
        self.iou_gate = rospy.get_param('~iou_gate', 0.3)
        self.label_ttl = rospy.get_param('~label_ttl', 1.0)
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)
        self.info = None
        self.tracks = None
        self.labels = {}        # track id -> (label, fused score, stamp seconds)
        # rospy runs each subscription in its own thread; ROS 2's single-threaded executor
        # serialised these callbacks, so keep that guarantee.
        self.lock = threading.Lock()
        self.obstacles_pub = rospy.Publisher('/obstacles', Detection3DArray, queue_size=10)
        self.marker_pub = rospy.Publisher('/obstacles_markers', MarkerArray, queue_size=10)
        rospy.Subscriber('/camera/camera_info', CameraInfo, self.locked(self.on_info), queue_size=1)
        rospy.Subscriber('/obstacles_lidar', Detection3DArray, self.locked(self.on_tracks),
                         queue_size=10)
        rospy.Subscriber('/detections_2d', Detection2DArray, self.locked(self.on_detections),
                         queue_size=10)

    def locked(self, callback):
        def run(msg):
            with self.lock:
                callback(msg)
        return run

    def on_info(self, msg):
        self.info = msg

    def on_tracks(self, msg):
        self.tracks = msg
        self.publish(stamp_to_seconds(msg.header.stamp))

    def lookup(self, target, source, stamp):
        for when in (stamp, rospy.Time(0)):
            try:
                return self.buffer.lookup_transform(target, source, when, rospy.Duration(0.02))
            except TransformException:
                continue
        return None

    def on_detections(self, msg):
        if self.tracks is None or self.info is None or not self.tracks.detections:
            return
        tf = self.lookup(msg.header.frame_id or self.info.header.frame_id,
                         self.tracks.header.frame_id, msg.header.stamp)
        if tf is None:
            return
        R, t = transform_matrix(tf)
        fx, cx = self.info.K[0], self.info.K[2]
        intervals = {}
        for det in self.tracks.detections:
            b = det.bbox
            corners = box_corners(b.center.position.x, b.center.position.y,
                                  quaternion_to_yaw(b.center.orientation), b.size.x, b.size.y,
                                  b.center.position.z - b.size.z / 2,
                                  b.center.position.z + b.size.z / 2)
            span = project_interval(corners @ R.T + t, fx, cx, self.info.width)
            if span is not None:
                intervals[det.id] = span[:2]
        camera_boxes = []
        for det in msg.detections:
            if det.results:
                u, half = det.bbox.center.position.x, det.bbox.size_x / 2
                hyp = det.results[0].hypothesis
                camera_boxes.append((u - half, u + half, hyp.class_id, hyp.score))
        now = stamp_to_seconds(msg.header.stamp)
        for track_id, (label, score, overlap) in associate(intervals, camera_boxes,
                                                           self.iou_gate).items():
            self.labels[track_id] = (label, fused_score(score, overlap), now)
        self.publish(now)

    def publish(self, now):
        if self.tracks is None:
            return
        live = {det.id for det in self.tracks.detections}
        self.labels = {k: v for k, v in self.labels.items()
                       if k in live and abs(now - v[2]) <= self.label_ttl}
        robot = self.lookup(self.tracks.header.frame_id, self.robot_frame, self.tracks.header.stamp)

        out = Detection3DArray(header=self.tracks.header)
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for det in self.tracks.detections:
            label, score = 'obstacle', det.results[0].hypothesis.score
            if det.id in self.labels:
                label, score = self.labels[det.id][:2]
            det.results[0].hypothesis.class_id = label
            det.results[0].hypothesis.score = score
            out.detections.append(det)

            p = det.bbox.center.position
            text = '%s %.2f' % (label, score)
            if robot is not None:
                r = robot.transform.translation
                text += ' · %.1f m' % math.hypot(p.x - r.x, p.y - r.y)
            r_, g_, b_ = colour_for(label)
            box = Marker(header=self.tracks.header, ns='boxes', id=int(det.id), type=Marker.CUBE,
                         action=Marker.ADD)
            box.pose = det.bbox.center
            box.scale.x, box.scale.y, box.scale.z = det.bbox.size.x, det.bbox.size.y, det.bbox.size.z
            box.color.r, box.color.g, box.color.b, box.color.a = r_, g_, b_, 0.45
            caption = Marker(header=self.tracks.header, ns='labels', id=int(det.id),
                             type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, text=text)
            caption.pose.position.x, caption.pose.position.y = p.x, p.y
            caption.pose.position.z = p.z + det.bbox.size.z / 2 + 0.25
            caption.pose.orientation.w = 1.0
            caption.scale.z = 0.2
            caption.color.r = caption.color.g = caption.color.b = caption.color.a = 1.0
            markers.markers += [box, caption]
        self.obstacles_pub.publish(out)
        self.marker_pub.publish(markers)


def main():
    rospy.init_node('obstacle_fusion')
    Fusion()
    rospy.spin()
