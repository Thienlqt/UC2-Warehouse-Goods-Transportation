"""Integration check against the installed Galactic controller (no Unity required).

Run explicitly in an isolated container/domain after building the package:
  python3 test/check_obstacle_costmap.py
Synthetic scans must never be published into the live robot's ROS graph.
"""

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
import yaml


# Obstacle wall x in metres: the centre of a 5 cm costmap cell, not a cell
# boundary, so float rounding cannot mark the neighbouring cell instead.
OBSTACLE_X = 1.025
ROBOT_RADIUS = 0.22  # Must match robot_radius in nav2_obstacle_avoidance.yaml.


class Fixture(Node):
    def __init__(self):
        super().__init__('obstacle_test_fixture')
        self.tf = TransformBroadcaster(self)
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.costmap = None
        self.odom = None
        self.commands = []
        self.mode = 'obstacle'
        # Simulated planar pose driven by /cmd_vel, so the robot really closes on walls.
        self.x = self.y = self.yaw = 0.0
        self.velocity = (0.0, 0.0)
        self.wall_x = None
        self.last_step = time.monotonic()
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 self.on_costmap, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_timer(0.05, self.publish)

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
        stamp = self.get_clock().now().to_msg()
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
        self.scan.publish(scan)

    def wait(self, predicate, seconds=15):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
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

    def transition(self, transition_id):
        client = self.create_client(ChangeState, '/controller_server/change_state')
        self.wait(client.service_is_ready)
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        self.wait(future.done)
        assert future.result().success, 'Lifecycle transition failed'
        self.destroy_client(client)


def main():
    rclpy.init()
    fixture = Fixture()
    processes = []
    work = tempfile.mkdtemp(prefix='nav2-obstacle-check-')
    log = open(os.path.join(work, 'nodes.log'), 'w')
    try:
        package = get_package_share_directory('unity_slam_example')
        with open(os.path.join(package, 'config/nav2_obstacle_avoidance.yaml')) as f:
            params = yaml.safe_load(f)

        def wall_clock(value):
            if isinstance(value, dict):
                for key, child in value.items():
                    if key == 'use_sim_time':
                        value[key] = False
                    else:
                        wall_clock(child)
        wall_clock(params)
        config = os.path.join(work, 'params.yaml')
        with open(config, 'w') as f:
            yaml.safe_dump(params, f)
        for package_name, executable, args in [
                ('unity_slam_example', 'tf_odometry', []),
                ('nav2_controller', 'controller_server', ['--ros-args', '--params-file', config])]:
            binary = Path(get_package_prefix(package_name)) / 'lib' / package_name / executable
            processes.append(subprocess.Popen([str(binary)] + args, stdout=log, stderr=log))
        fixture.wait(lambda: fixture.odom is not None)
        fixture.transition(1)  # configure
        fixture.transition(3)  # activate
        fixture.wait(lambda: fixture.cost(OBSTACLE_X, 0.0) == 100)
        print('PASS: laser obstacle marked lethal in local costmap', flush=True)
        fixture.mode = 'clear'
        fixture.wait(lambda: fixture.cost(OBSTACLE_X, 0.0) == 0)
        print('PASS: no-return rays clear the removed obstacle', flush=True)

        action = ActionClient(fixture, FollowPath, '/follow_path')
        fixture.wait(action.server_is_ready)
        goal = FollowPath.Goal()
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'
        goal.path = PathMsg()
        goal.path.header.frame_id = 'odom'
        goal.path.header.stamp = fixture.get_clock().now().to_msg()
        for i in range(41):
            pose = PoseStamped()
            pose.header = goal.path.header
            pose.pose.position.x = i * 0.05
            pose.pose.orientation.w = 1.0
            goal.path.poses.append(pose)
        future = action.send_goal_async(goal)
        fixture.wait(future.done)
        handle = future.result()
        assert handle.accepted
        fixture.wait(lambda: any(x > 0.02 for _, x, _ in fixture.commands))
        print('PASS: DWB commands forward motion on a clear path', flush=True)
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
            rclpy.spin_once(fixture, timeout_sec=0.05)
            stopped()
        print('PASS: DWB stops %.2f m short of a wall blocking the path (body clearance %.2f m)'
              % (closest[0], closest[0] - ROBOT_RADIUS), flush=True)
        cancel = handle.cancel_goal_async()
        fixture.wait(cancel.done)
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        fixture.destroy_node()
        rclpy.shutdown()
        log.close()
        print('Node logs: ' + work)


if __name__ == '__main__':
    main()
