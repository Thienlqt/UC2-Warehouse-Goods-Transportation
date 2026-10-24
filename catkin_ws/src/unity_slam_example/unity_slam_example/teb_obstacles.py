"""ROS node: /obstacles tracks -> TEB's dynamic obstacles (costmap_converter/ObstacleArrayMsg).

Confident moving tracks become polygons with their velocity, so TEB plans around where they will
be, not only where they are (see avoidance_math.moving_polygons). Static obstacles stay in the
costmap. Used with planner:=teb only.
"""

from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32
import rospy
from uc2_vision_msgs.msg import Detection3DArray

from .avoidance_math import moving_polygons
from .perception.ros_utils import yaw_to_quaternion
from .predicted_obstacles import tracks_from_detections


class TebObstacles:
    def __init__(self):
        self.min_speed = rospy.get_param('~min_speed', 0.15)
        self.min_confidence = rospy.get_param('~min_confidence', 0.3)
        self.publisher = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles',
                                         ObstacleArrayMsg, queue_size=1)
        rospy.Subscriber('/obstacles', Detection3DArray, self.on_obstacles, queue_size=1)

    def on_obstacles(self, msg):
        out = ObstacleArrayMsg(header=msg.header)
        for i, (corners, vx, vy, yaw) in enumerate(moving_polygons(
                tracks_from_detections(msg), self.min_speed, self.min_confidence)):
            obstacle = ObstacleMsg(header=msg.header, id=i, orientation=yaw_to_quaternion(yaw))
            obstacle.polygon.points = [Point32(x=float(x), y=float(y)) for x, y in corners]
            obstacle.velocities.twist.linear.x, obstacle.velocities.twist.linear.y = vx, vy
            out.obstacles.append(obstacle)
        self.publisher.publish(out)


def main():
    rospy.init_node('teb_obstacles')
    TebObstacles()
    rospy.spin()
