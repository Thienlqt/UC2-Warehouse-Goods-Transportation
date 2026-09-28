"""ROS node: camera JPEG -> YOLOv8 detections (/detections_2d) + annotated image (/detections_image)."""

import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .yolo import load_class_names, YoloDetector

# One colour per class id (BGR), cycled.
PALETTE = [(56, 56, 255), (151, 157, 255), (31, 112, 255), (29, 178, 255), (49, 210, 207),
           (10, 249, 72), (23, 204, 146), (134, 219, 61), (211, 188, 0), (209, 99, 20)]


def draw(image, detections, names):
    for d in detections:
        colour = PALETTE[d.class_id % len(PALETTE)]
        p1, p2 = (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2))
        cv2.rectangle(image, p1, p2, colour, 2)
        label = '%s %.2f' % (names[d.class_id] if d.class_id < len(names) else d.class_id, d.score)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(p1[1], th + 4)
        cv2.rectangle(image, (p1[0], top - th - 4), (p1[0] + tw + 4, top), colour, -1)
        cv2.putText(image, label, (p1[0] + 2, top - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return image


class CameraDetector(Node):
    def __init__(self):
        super().__init__('camera_detector')
        # Default: the model installed with this package (ros2_ws/src/unity_slam_example/models).
        models = os.path.join(get_package_share_directory('unity_slam_example'), 'models')
        model = self.declare_parameter('model_path', os.path.join(models, 'detector.onnx')).value
        classes = self.declare_parameter('classes_path', os.path.join(models, 'classes.txt')).value
        self.conf = self.declare_parameter('conf_threshold', 0.35).value
        self.iou = self.declare_parameter('iou_threshold', 0.5).value
        threads = self.declare_parameter('threads', 4).value
        self.publish_image = self.declare_parameter('publish_annotated_image', True).value

        self.detector = None
        if os.path.exists(model) and os.path.exists(classes):
            self.detector = YoloDetector(model, threads)
            self.names = load_class_names(classes)
            self.get_logger().info('Loaded %s (%d px, %d classes) on %s' % (
                model, self.detector.size, len(self.names), self.detector.provider))
        else:
            self.get_logger().error('No model at %s / %s: camera detection disabled. See '
                                    'docs/obstacle_detection.md.' % (model, classes))

        self.det_pub = self.create_publisher(Detection2DArray, '/detections_2d', 10)
        self.img_pub = self.create_publisher(Image, '/detections_image', 1)
        # Newest frame only: never queue stale images behind a slow inference.
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.on_image, qos)
        self.timings = []
        self.create_timer(5.0, self.report)

    def on_image(self, msg):
        if self.detector is None:
            return
        t0 = time.perf_counter()
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        t1 = time.perf_counter()
        detections = self.detector(image, self.conf, self.iou)
        t2 = time.perf_counter()

        out = Detection2DArray()
        out.header = msg.header
        for d in detections:
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = (d.x1 + d.x2) / 2
            det.bbox.center.position.y = (d.y1 + d.y2) / 2
            det.bbox.size_x = d.x2 - d.x1
            det.bbox.size_y = d.y2 - d.y1
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = self.names[d.class_id]
            hypothesis.hypothesis.score = d.score
            det.results.append(hypothesis)
            out.detections.append(det)
        self.det_pub.publish(out)

        if self.publish_image and self.img_pub.get_subscription_count() > 0:
            draw(image, detections, self.names)
            img = Image(header=msg.header, height=image.shape[0], width=image.shape[1],
                        encoding='bgr8', is_bigendian=0, step=image.shape[1] * 3,
                        data=image.tobytes())
            self.img_pub.publish(img)
        self.timings.append((t1 - t0, t2 - t1, time.perf_counter() - t0))

    def report(self):
        if not self.timings:
            return
        t = np.array(self.timings) * 1000
        self.get_logger().info(
            '%.1f FPS | decode %.1f ms, inference %.1f ms, total %.1f ms (p95 %.1f)' % (
                len(t) / 5.0, t[:, 0].mean(), t[:, 1].mean(), t[:, 2].mean(),
                np.percentile(t[:, 2], 95)))
        self.timings.clear()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
