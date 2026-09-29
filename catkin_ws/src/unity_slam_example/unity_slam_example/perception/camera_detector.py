"""ROS node: camera JPEG -> YOLOv8 detections (/detections_2d) + annotated image (/detections_image)."""

import os
import time

import cv2
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage, Image
from uc2_vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

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


class CameraDetector:
    def __init__(self):
        # Default: the model shipped with this package (catkin_ws/src/unity_slam_example/models).
        models = os.path.join(rospkg.RosPack().get_path('unity_slam_example'), 'models')
        model = rospy.get_param('~model_path', os.path.join(models, 'detector.onnx'))
        classes = rospy.get_param('~classes_path', os.path.join(models, 'classes.txt'))
        self.conf = rospy.get_param('~conf_threshold', 0.35)
        self.iou = rospy.get_param('~iou_threshold', 0.5)
        threads = rospy.get_param('~threads', 4)
        self.publish_image = rospy.get_param('~publish_annotated_image', True)

        self.detector = None
        if os.path.exists(model) and os.path.exists(classes):
            self.detector = YoloDetector(model, threads)
            self.names = load_class_names(classes)
            rospy.loginfo('Loaded %s (%d px, %d classes) on %s' % (
                model, self.detector.size, len(self.names), self.detector.provider))
        else:
            rospy.logerr('No model at %s / %s: camera detection disabled. See '
                         'docs/obstacle_detection.md.' % (model, classes))

        self.det_pub = rospy.Publisher('/detections_2d', Detection2DArray, queue_size=10)
        self.img_pub = rospy.Publisher('/detections_image', Image, queue_size=1)
        self.timings = []
        # Newest frame only: never queue stale images behind a slow inference. The large
        # buff_size keeps rospy from reading old frames out of its socket buffer first.
        rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, self.on_image,
                         queue_size=1, buff_size=2 ** 24)
        rospy.Timer(rospy.Duration(5.0), lambda _: self.report())

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

        if self.publish_image and self.img_pub.get_num_connections() > 0:
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
        rospy.loginfo(
            '%.1f FPS | decode %.1f ms, inference %.1f ms, total %.1f ms (p95 %.1f)' % (
                len(t) / 5.0, t[:, 0].mean(), t[:, 1].mean(), t[:, 2].mean(),
                np.percentile(t[:, 2], 95)))
        self.timings.clear()


def main():
    rospy.init_node('camera_detector')
    CameraDetector()
    rospy.spin()
