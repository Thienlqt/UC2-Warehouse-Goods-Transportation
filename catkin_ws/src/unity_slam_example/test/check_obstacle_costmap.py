"""Integration check against the installed Noetic move_base (no Unity required).

Starts its own ROS master on port 11312, so synthetic scans never reach a live robot graph.
Run explicitly after building the workspace and sourcing devel/setup.bash:
  python3 test/check_obstacle_costmap.py
"""

import math
import os
import signal
import subprocess
import tempfile
import time

MASTER_PORT = 11312
os.environ['ROS_MASTER_URI'] = 'http://localhost:%d' % MASTER_PORT

import rospkg  # noqa: E402
import rospy  # noqa: E402
from actionlib_msgs.msg import GoalID  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster  # noqa: E402
from uc2_vision_msgs.msg import (Detection3D, Detection3DArray,  # noqa: E402
                                 ObjectHypothesisWithPose)

# Obstacle wall x in metres: the centre of a 5 cm costmap cell, not a cell
# boundary, so float rounding cannot mark the neighbouring cell instead.
OBSTACLE_X = 1.025
ROBOT_RADIUS = 0.22  # Must match robot_radius in config/move_base.yaml.
# A tracked 0.4 m box at (MOVER_X, -1.0) moving +y at 0.5 m/s: its near edge sweeps along
# x = MOVER_X - 0.2 (a cell centre) towards the robot's path, where the scan sees nothing.
MOVER_X = 1.525
PREDICTED_CELL = (MOVER_X - 0.2, -0.475)
# Assisted driving: a 0.5 x 1.0 m box (centre x, y, half sizes) 3 m ahead of the robot's start.
BOX = (3.0, 0.0, 0.25, 0.5)

LAUNCH = """<launch>
  <param name="/use_sim_time" value="false"/>
  <node pkg="move_base" type="move_base" name="move_base">
    <rosparam command="load" file="{config}/move_base.yaml"/>
    <rosparam command="load" file="{config}/move_base_predicted.yaml"/>
    <remap from="cmd_vel" to="cmd_vel_nav"/>
  </node>
  <node pkg="unity_slam_example" type="cmd_vel_guard" name="cmd_vel_guard">
    <param name="robot_radius" value="{radius}"/>
  </node>
  <node pkg="unity_slam_example" type="tf_odometry" name="unity_tf_odometry"/>
  <rosparam command="load" file="{config}/perception.yaml"/>
  <node pkg="unity_slam_example" type="predicted_obstacles" name="predicted_obstacles"/>
  <node pkg="unity_slam_example" type="teleop_assist" name="teleop_assist">
    <param name="robot_radius" value="{radius}"/>
  </node>
</launch>
"""


class Fixture:
    def __init__(self):
        self.tf = TransformBroadcaster()
        self.static_tf = StaticTransformBroadcaster()
        self.scan = rospy.Publisher('/scan', LaserScan, queue_size=10)
        self.map = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        self.goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.nav_cmd = rospy.Publisher('/cmd_vel_nav', Twist, queue_size=10)
        self.obstacles = rospy.Publisher('/obstacles', Detection3DArray, queue_size=1)
        self.tracks = None              # [(x, y, vx, vy)] in odom, published as /obstacles
        self.teleop = rospy.Publisher('/cmd_vel_teleop', Twist, queue_size=1)
        self.keys = None                # Twist the "driver" holds, published as /cmd_vel_teleop
        self.costmap = None
        self.odom = None
        self.commands = []
        self.mode = 'obstacle'
        # Simulated planar pose driven by /cmd_vel, so the robot really closes on walls.
        self.x = self.y = self.yaw = 0.0
        self.velocity = (0.0, 0.0)
        self.wall_x = None
        self.last_step = time.monotonic()
        self.publish_static()
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.on_costmap)
        rospy.Subscriber('/odom', Odometry, self.on_odom)
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd)
        rospy.Timer(rospy.Duration(0.05), lambda _: self.publish())

    def publish_static(self):
        # The global costmap needs a map and map -> odom (slam_toolbox's job in the real stack).
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = rospy.Time.now(), 'map', 'odom'
        tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform(tf)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution, grid.info.width, grid.info.height = 0.05, 400, 400
        grid.info.origin.position.x = grid.info.origin.position.y = -10.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (400 * 400)
        self.map.publish(grid)

    def on_costmap(self, msg):
        self.costmap = msg

    def on_odom(self, msg):
        self.odom = msg

    def on_cmd(self, msg):
        self.commands.append((time.monotonic(), msg.linear.x, msg.angular.z))
        self.velocity = (msg.linear.x, msg.angular.z)

    def publish(self):
        now = time.monotonic()
        dt, self.last_step = now - self.last_step, now
        vx, wz = self.velocity
        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt
        self.yaw += wz * dt
        stamp = rospy.Time.now()
        transforms = []
        for parent, child, x, y, z, yaw in [
                ('odom', 'base_link', self.x, self.y, 0.0, self.yaw),
                ('base_link', 'base_scan', 0.0, 0.0, 0.2, 0.0)]:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, parent, child
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            tf.transform.rotation.z = math.sin(yaw / 2.0)
            tf.transform.rotation.w = math.cos(yaw / 2.0)
            transforms.append(tf)
        self.tf.sendTransform(transforms)
        if self.tracks is not None:
            self.obstacles.publish(self.detections(stamp))
        if self.keys is not None:
            self.teleop.publish(self.keys)
        if self.mode == 'silent':
            return
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'base_scan'
        scan.angle_min, scan.angle_max = 0.0, math.radians(359)
        scan.angle_increment = math.radians(1)
        scan.range_min, scan.range_max, scan.scan_time = 0.12, 100.0, 0.05
        scan.ranges = [float('inf')] * 360
        if self.mode == 'obstacle':
            for i in list(range(6)) + list(range(355, 360)):
                scan.ranges[i] = OBSTACLE_X / math.cos(math.radians(i))
        elif self.mode == 'wall':
            # Infinite wall across the path at odom x = wall_x.
            for i in range(360):
                heading = math.cos(self.yaw + math.radians(i))
                if heading > 1e-3:
                    distance = (self.wall_x - self.x) / heading
                    if 0.0 < distance < 10.0:
                        scan.ranges[i] = distance
        elif self.mode == 'box':
            for i in range(360):
                distance = box_range(self.x, self.y, self.yaw + math.radians(i), BOX)
                if distance is not None:
                    scan.ranges[i] = distance
        self.scan.publish(scan)

    def detections(self, stamp):
        out = Detection3DArray()
        out.header.stamp, out.header.frame_id = stamp, 'odom'
        for i, (x, y, vx, vy) in enumerate(self.tracks):
            det = Detection3D(header=out.header, id=str(i + 1))
            det.bbox.center.position.x, det.bbox.center.position.y = x, y
            det.bbox.center.orientation.w = 1.0
            det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = 0.4, 0.4, 0.5
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id, hypothesis.hypothesis.score = 'obstacle', 0.9
            hypothesis.pose.pose.position.x, hypothesis.pose.pose.position.y = vx, vy
            det.results.append(hypothesis)
            out.detections.append(det)
        return out

    def wait(self, predicate, seconds=15):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            time.sleep(0.05)
        raise AssertionError('Timed out waiting for ' + repr(predicate))

    def cost(self, x, y):
        if self.costmap is None:
            return None
        info = self.costmap.info
        ix = int((x - info.origin.position.x) / info.resolution)
        iy = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= ix < info.width and 0 <= iy < info.height):
            return None
        return self.costmap.data[iy * info.width + ix]


def box_range(x, y, angle, box):
    """Distance along a ray from (x, y) to an axis-aligned box (cx, cy, hx, hy), or None."""
    cx, cy, hx, hy = box
    near, far = 0.0, float('inf')
    for origin, direction, lo, hi in ((x, math.cos(angle), cx - hx, cx + hx),
                                      (y, math.sin(angle), cy - hy, cy + hy)):
        if abs(direction) < 1e-12:
            if not lo <= origin <= hi:
                return None
            continue
        t1, t2 = (lo - origin) / direction, (hi - origin) / direction
        near, far = max(near, min(t1, t2)), min(far, max(t1, t2))
    return near if 0.0 < near <= far else None


def box_clearance(x, y, box):
    """Distance from a point to the box's surface (0 inside)."""
    cx, cy, hx, hy = box
    return math.hypot(max(abs(x - cx) - hx, 0.0), max(abs(y - cy) - hy, 0.0))


def main():
    work = tempfile.mkdtemp(prefix='move-base-obstacle-check-')
    log = open(os.path.join(work, 'nodes.log'), 'w')
    config = os.path.join(rospkg.RosPack().get_path('unity_slam_example'), 'config')
    launch = os.path.join(work, 'check.launch')
    with open(launch, 'w') as f:
        f.write(LAUNCH.format(config=config, radius=ROBOT_RADIUS))
    # roslaunch starts the private master on MASTER_PORT (from ROS_MASTER_URI).
    roslaunch = subprocess.Popen(['roslaunch', '-p', str(MASTER_PORT), launch],
                                 stdout=log, stderr=log, start_new_session=True)
    try:
        deadline = time.monotonic() + 30
        while True:
            try:
                rospy.get_master().getSystemState()
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise AssertionError('No ROS master on port %d' % MASTER_PORT)
                time.sleep(0.5)
        rospy.init_node('obstacle_test_fixture', disable_signals=True)
        fixture = Fixture()
        fixture.wait(lambda: fixture.odom is not None)
        fixture.wait(lambda: fixture.cost(OBSTACLE_X, 0.0) == 100, seconds=45)
        print('PASS: laser obstacle marked lethal in local costmap', flush=True)
        fixture.mode = 'clear'
        fixture.wait(lambda: fixture.cost(OBSTACLE_X, 0.0) == 0)
        print('PASS: no-return rays clear the removed obstacle', flush=True)
        fixture.tracks = [(MOVER_X, -1.0, 0.0, 0.5)]
        fixture.wait(lambda: fixture.cost(*PREDICTED_CELL) == 100)
        print('PASS: predicted path of a moving obstacle marked lethal ahead of it', flush=True)
        fixture.tracks = []
        fixture.wait(lambda: fixture.cost(*PREDICTED_CELL) == 0)
        print('PASS: predicted cells clear once the obstacle is gone', flush=True)

        goal = PoseStamped()
        goal.header.frame_id, goal.header.stamp = 'map', rospy.Time.now()
        goal.pose.position.x = 2.0
        goal.pose.orientation.w = 1.0
        fixture.goal.publish(goal)
        fixture.wait(lambda: any(x > 0.02 for _, x, _ in fixture.commands))
        print('PASS: DWA commands forward motion on a clear path', flush=True)
        fixture.wall_x = fixture.x + 0.6
        fixture.mode = 'wall'
        blocked_at = time.monotonic()
        closest = [float('inf')]

        def stopped():
            closest[0] = min(closest[0], fixture.wall_x - fixture.x)
            assert closest[0] > ROBOT_RADIUS, 'Robot body reached the wall'
            return any(t > blocked_at + 0.5 and abs(x) < 0.001
                       for t, x, _ in fixture.commands)
        fixture.wait(stopped, seconds=45)
        hold_until = time.monotonic() + 3.0  # Keep checking: no creeping into the wall.
        while time.monotonic() < hold_until:
            time.sleep(0.05)
            stopped()
        print('PASS: robot stops %.2f m short of a wall blocking the path (body clearance %.2f m)'
              % (closest[0], closest[0] - ROBOT_RADIUS), flush=True)

        fixture.cancel.publish(GoalID())
        fixture.mode = 'silent'
        time.sleep(1.5)                     # longer than the guard's 1.0 s source_timeout
        sent = time.monotonic()
        forward = Twist()
        forward.linear.x = 0.2
        for _ in range(5):
            fixture.nav_cmd.publish(forward)
            time.sleep(0.1)
        fixture.wait(lambda: any(t > sent for t, _, _ in fixture.commands))
        assert all(abs(x) < 1e-9 for t, x, _ in fixture.commands if t > sent), \
            'Guard passed a command through without a fresh scan'
        print('PASS: cmd_vel_guard holds the robot still when /scan stops', flush=True)

        # Assisted driving: hold "W" towards a box 3 m ahead, from a fresh start pose.
        fixture.x = fixture.y = fixture.yaw = 0.0
        fixture.mode = 'box'
        time.sleep(1.5)                     # fresh scans for the guard and teleop_assist
        fixture.keys = Twist()
        fixture.keys.linear.x = 0.5
        track = {'clearance': float('inf'), 'offset': 0.0}

        def passed():
            track['clearance'] = min(track['clearance'],
                                     box_clearance(fixture.x, fixture.y, BOX) - ROBOT_RADIUS)
            track['offset'] = max(track['offset'], abs(fixture.y))
            assert track['clearance'] > 0.0, 'Robot body touched the box'
            return fixture.x > 6.0 and abs(fixture.yaw) < 0.2
        fixture.wait(passed, seconds=45)
        fixture.keys = None
        assert track['offset'] > BOX[3], 'Robot did not steer around the box'
        print('PASS: assisted driving steers around a box 3 m ahead (closest body clearance '
              '%.2f m, %.2f m to the side) and returns to its heading'
              % (track['clearance'], track['offset']), flush=True)
    finally:
        rospy.signal_shutdown('done')
        if roslaunch.poll() is None:
            os.killpg(roslaunch.pid, signal.SIGINT)
        try:
            roslaunch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(roslaunch.pid, signal.SIGKILL)
            roslaunch.wait()
        log.close()
        print('Node logs: ' + work)


if __name__ == '__main__':
    main()
