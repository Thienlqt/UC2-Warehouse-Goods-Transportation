"""ROS node: /obstacles tracks -> /obstacles_predicted (OccupancyGrid) for the local costmap.

Marks where moving obstacles will be within `horizon` seconds (constant velocity), and pads
classes that need extra clearance (see avoidance_math.predicted_points). The grid is a square
around the robot, published on a timer and stamped with the latest robot TF; a costmap
StaticLayer replaces its copy on every message, so old predictions disappear exactly. While
/obstacles is stale the grid is empty.
"""

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from uc2_vision_msgs.msg import Detection3DArray

from .avoidance_math import (advance, inverse_pose, occupancy_grid, predicted_points, speed,
                             Track, track_to_frame)
from .perception.ros_utils import quaternion_to_yaw


def tracks_from_detections(msg):
    """Tracks from a Detection3DArray; velocity rides in results[0].pose (see lidar_obstacles)."""
    tracks = []
    for det in msg.detections:
        if not det.results:
            continue
        b, result = det.bbox, det.results[0]
        tracks.append(Track(b.center.position.x, b.center.position.y,
                            quaternion_to_yaw(b.center.orientation), b.size.x, b.size.y,
                            result.pose.pose.position.x, result.pose.pose.position.y,
                            result.hypothesis.class_id, result.hypothesis.score))
    return tracks


class MovingTracks:
    """The latest confident moving tracks from /obstacles, served in the robot's base frame.

    Feed on_obstacles from a /obstacles subscriber. in_base_frame() advances the tracks to the
    latest robot TF and returns them in base_frame, or [] when they are older than timeout.
    """

    def __init__(self, buffer, base_frame, min_speed, min_confidence, timeout):
        self.buffer, self.base_frame, self.timeout = buffer, base_frame, timeout
        self.min_speed, self.min_confidence = min_speed, min_confidence
        self.latest = None      # (stamp, frame, tracks)

    def on_obstacles(self, msg):
        tracks = [t for t in tracks_from_detections(msg)
                  if t.score >= self.min_confidence and speed(t) >= self.min_speed]
        self.latest = (msg.header.stamp, msg.header.frame_id, tracks)

    def in_base_frame(self):
        latest = self.latest
        if latest is None or not latest[2]:
            return []
        try:
            t = self.buffer.lookup_transform(latest[1], self.base_frame, rospy.Time(0))
        except TransformException:
            return []
        age = (t.header.stamp - latest[0]).to_sec()
        if abs(age) > self.timeout:
            return []
        to_base = inverse_pose(t.transform.translation.x, t.transform.translation.y,
                               quaternion_to_yaw(t.transform.rotation))
        return [track_to_frame(advance(track, age), *to_base) for track in latest[2]]


class PredictedObstacles:
    def __init__(self):
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.fixed_frame = rospy.get_param('~fixed_frame', 'odom')
        self.args = {
            'horizon': rospy.get_param('~horizon', 2.0),
            'step': rospy.get_param('~step', 0.2),
            'min_speed': rospy.get_param('~min_speed', 0.15),
            'min_confidence': rospy.get_param('~min_confidence', 0.3),
            'padding_by_class': rospy.get_param('~padding', {}),
            'spacing': rospy.get_param('~spacing', 0.025),
            'keep_out': rospy.get_param('~keep_out', 0.45),
        }
        self.size = rospy.get_param('~grid_size', 6.0)
        self.resolution = rospy.get_param('~resolution', 0.05)
        self.timeout = rospy.get_param('~obstacle_timeout', 0.5)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)
        self.latest = None      # (stamp, frame, tracks)
        # Latched: the costmap's StaticLayer waits for a first grid when move_base starts.
        self.publisher = rospy.Publisher('/obstacles_predicted', OccupancyGrid, queue_size=1,
                                         latch=True)
        rospy.Subscriber('/obstacles', Detection3DArray, self.on_obstacles, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / rospy.get_param('~rate', 10.0)), self.publish)

    def on_obstacles(self, msg):
        self.latest = (msg.header.stamp, msg.header.frame_id, tracks_from_detections(msg))

    def publish(self, _):
        latest = self.latest
        frame = latest[1] if latest else self.fixed_frame
        try:
            robot = self.buffer.lookup_transform(frame, self.base_frame, rospy.Time(0))
        except TransformException:
            return
        stamp = robot.header.stamp
        tracks = []
        if latest is not None:
            age = (stamp - latest[0]).to_sec()
            if abs(age) <= self.timeout:    # far in the future: the sim clock restarted
                tracks = [advance(t, age) for t in latest[2]]
        r = robot.transform.translation
        points = predicted_points(tracks, robot_xy=(r.x, r.y), **self.args)
        ox, oy, n, data = occupancy_grid(points, (r.x, r.y), self.size, self.resolution)
        grid = OccupancyGrid(header=Header(stamp=stamp, frame_id=frame))
        grid.info.map_load_time = stamp
        grid.info.resolution = self.resolution
        grid.info.width = grid.info.height = n
        grid.info.origin.position.x, grid.info.origin.position.y = ox, oy
        grid.info.origin.orientation.w = 1.0
        grid.data = data.ravel().tolist()
        self.publisher.publish(grid)


def main():
    rospy.init_node('predicted_obstacles')
    PredictedObstacles()
    rospy.spin()
