"""Repeatable avoidance trials against the installed Noetic move_base (no Unity required).

Each trial starts its own ROS master and runs move_base, cmd_vel_guard, tf_odometry,
predicted_obstacles, teleop_assist and (by default) lidar_obstacles + obstacle_fusion with this
package's config, against a simulated robot (unicycle with a 0.3 s motor lag, driven by /cmd_vel)
and a world of boxes, some of them moving. The trial publishes /clock itself, so a busy machine or
a wall-clock jump (Docker Desktop resyncs its VM clock) slows a trial down without changing it.

Run explicitly after building the workspace and sourcing devel/setup.bash:
  python3 test/scenarios.py                  # every scene once
  python3 test/scenarios.py rack_end cross --runs 3
Prints one line per scene: goals reached, time, time stopped, total rotation, closest clearance
of the robot's colliders (0: contact) and how far from the goal it ended. These checks do not
replace Unity trials.
"""

import argparse
import json
import math
import os
import random
import signal
import subprocess
import sys
import tempfile
import threading
import time

RADIUS = 0.25                    # robot_radius in config/move_base.yaml
# The Unity TurtleBot3 Waffle's colliders (UnityProject/Assets/turtlebot3/model.urdf), base_link
# frame: the body box (x -0.197..0.069, y +-0.133) and the wheels (x +-0.033, y +-0.153).
# Clearance is measured from these, so a corner swinging into a shelf counts as contact.
BODY = [(-0.197, -0.133, 0.069, 0.133), (-0.033, -0.153, 0.033, 0.153)]
LAG = 0.3                        # first-order motor lag (s)
NOISE = 0.01                     # range noise (m)
WALLS = [(5.0, 1.5, 9.0, 0.05), (5.0, -1.5, 9.0, 0.05)]     # a 3 m wide aisle along x

# boxes: (centre x, centre y, half x, half y). movers: boxes moving back and forth,
# (centre x, centre y, half x, half y, vx, vy, start s, travel m). The robot starts at the origin
# facing +x; goal is (x, y) in metres.
SCENES = {
    # Static: a goal behind a shelf or rack, so the route turns round its end.
    'shelf_end': dict(goal=(4.2, 0.5), boxes=[(2.5, 1.0, 0.5, 2.5)]),
    'rack_end': dict(goal=(4.2, 0.5), boxes=[(2.6, 1.0, 0.4, 2.5)] + [
        (2.1, y, 0.05, 0.05) for y in (-1.5, -0.5, 0.5, 1.5, 2.5, 3.5)]),   # uprights in front
    'open_box': dict(goal=(7.0, 0.0), boxes=[(3.5, 0.0, 0.25, 0.25)]),
    # A goal clicked 0.15 m in front of a box: too close for the robot, which must not quietly
    # settle for another point.
    'goal_at_box': dict(goal=(3.6, 0.0), boxes=[(4.0, 0.0, 0.25, 0.25)]),
    'post': dict(goal=(7.0, 0.0), boxes=[(3.5, 0.3, 0.025, 0.025)]),
    'aisle_narrow': dict(goal=(7.0, 0.0), boxes=[(3.5, 0.3, 0.25, 0.25), (3.5, 1.2, 6.0, 0.05),
                                                  (3.5, -1.2, 6.0, 0.05)]),
    # 0.70 m gaps either side of a box for the 0.50 m robot circle: a stress test.
    'aisle_tight': dict(goal=(7.0, 0.0), boxes=[(3.5, 0.0, 0.25, 0.25), (3.5, 1.0, 6.0, 0.05),
                                                 (3.5, -1.0, 6.0, 0.05)]),
    # Moving: a box crossing the route once (0.4 and 0.8 m/s), one coming head-on down an aisle.
    'cross': dict(goal=(8.0, 0.0), movers=[(4.0, -3.0, 0.25, 0.25, 0.0, 0.4, 4.0, 10.0)]),
    'cross_fast': dict(goal=(8.0, 0.0), movers=[(4.0, -3.0, 0.25, 0.25, 0.0, 0.8, 6.0, 10.0)]),
    'headon': dict(goal=(8.0, 0.0), boxes=WALLS,
                   movers=[(9.0, 0.1, 0.25, 0.25, -0.3, 0.0, 0.0, 9.0)]),
    # Known limits (see docs/obstacle_avoidance.md): a faster box from behind that never yields,
    # and a small box sweeping back and forth across the aisle without pausing.
    'overtaken': dict(goal=(8.0, 0.0), boxes=WALLS,
                      movers=[(-2.0, 0.0, 0.25, 0.25, 0.5, 0.0, 2.0, 12.0)]),
    'sweeper': dict(goal=(8.0, 0.0), boxes=WALLS,
                    movers=[(4.0, -1.2, 0.15, 0.15, 0.0, 0.5, 5.0, 2.4)]),
}

LAUNCH = """<launch>
  <param name="/use_sim_time" value="true"/>
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
  {perception}
</launch>
"""
PERCEPTION = """<node pkg="unity_slam_example" type="lidar_obstacles" name="lidar_obstacles"/>
  <node pkg="unity_slam_example" type="obstacle_fusion" name="obstacle_fusion"/>"""


def box_range(x, y, angle, box):
    """Distance along a ray from (x, y) to an axis-aligned box, or None."""
    cx, cy, hx, hy = box[:4]
    near, far = 0.0, float('inf')
    for origin, d, lo, hi in ((x, math.cos(angle), cx - hx, cx + hx),
                              (y, math.sin(angle), cy - hy, cy + hy)):
        if abs(d) < 1e-12:
            if not lo <= origin <= hi:
                return None
            continue
        t1, t2 = (lo - origin) / d, (hi - origin) / d
        near, far = max(near, min(t1, t2)), min(far, max(t1, t2))
    return near if 0.0 < near <= far else None


def box_clearance(x, y, box):
    """Distance from a point to the box's surface (0 inside)."""
    cx, cy, hx, hy = box[:4]
    return math.hypot(max(abs(x - cx) - hx, 0.0), max(abs(y - cy) - hy, 0.0))


def body_outline(x, y, yaw, spacing=0.02):
    """Points along the robot's collider outlines at pose (x, y, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    points = []
    for x0, y0, x1, y1 in BODY:
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        for (ax, ay), (bx, by) in zip(corners, corners[1:]):
            n = max(1, int(math.hypot(bx - ax, by - ay) / spacing))
            for k in range(n):
                px, py = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
                points.append((x + c * px - s * py, y + s * px + c * py))
    return points


def body_clearance(x, y, yaw, boxes):
    """Closest distance between the robot's colliders and any box (0: touching or inside)."""
    if not boxes:
        return 99.0
    return min(box_clearance(px, py, b) for px, py in body_outline(x, y, yaw) for b in boxes)


def boxes_at(scene, elapsed):
    """Static boxes plus movers at `elapsed` seconds after the goal was sent."""
    out = list(scene.get('boxes', []))
    for cx, cy, hx, hy, vx, vy, start, travel in scene.get('movers', []):
        speed = math.hypot(vx, vy)
        d = max(0.0, elapsed - start) * speed % (2 * travel)
        d = d if d <= travel else 2 * travel - d
        out.append((cx + vx / speed * d, cy + vy / speed * d, hx, hy))
    return out


def run_trial(name, port, rate, perception, limit):
    """One trial on its own ROS master; returns a result dict."""
    os.environ['ROS_MASTER_URI'] = 'http://localhost:%d' % port
    import rospkg
    import rospy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from move_base_msgs.msg import MoveBaseActionResult
    from nav_msgs.msg import OccupancyGrid
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    from geometry_msgs.msg import Twist

    scene = SCENES[name]
    work = tempfile.mkdtemp(prefix='scenario-%s-' % name)
    log = open(os.path.join(work, 'nodes.log'), 'w')
    config = os.path.join(rospkg.RosPack().get_path('unity_slam_example'), 'config')
    launch = os.path.join(work, 'trial.launch')
    with open(launch, 'w') as f:
        f.write(LAUNCH.format(config=config, radius=RADIUS,
                              perception=PERCEPTION if perception else ''))
    roslaunch = subprocess.Popen(['roslaunch', '-p', str(port), launch], stdout=log, stderr=log,
                                 start_new_session=True)
    stop = threading.Event()
    try:
        deadline = time.monotonic() + 30
        while True:
            try:
                rospy.get_master().getSystemState()
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise RuntimeError('No ROS master on port %d' % port)
                time.sleep(0.3)
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('scenario', disable_signals=True)
        m0, base = time.monotonic(), 1000.0

        def sim_now():
            return base + rate * (time.monotonic() - m0)

        clock = rospy.Publisher('/clock', Clock, queue_size=10)

        def clock_loop():
            while not stop.is_set():
                clock.publish(Clock(clock=rospy.Time.from_sec(sim_now())))
                time.sleep(0.002)
        threading.Thread(target=clock_loop, daemon=True).start()

        state = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'v': 0.0, 'w': 0.0, 'cmd': (0.0, 0.0),
                 't0': None}
        results = []
        rospy.Subscriber('/cmd_vel', Twist,
                         lambda m: state.update(cmd=(m.linear.x, m.angular.z)))
        rospy.Subscriber('/move_base/result', MoveBaseActionResult,
                         lambda m: results.append((sim_now(), m.status)))
        tf, static = TransformBroadcaster(), StaticTransformBroadcaster()
        scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=5)
        map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1,
                                   latch=True)

        statics = []
        for parent, child, z in (('map', 'odom', 0.0), ('base_footprint', 'base_link', 0.0),
                                 ('base_link', 'base_scan', 0.2)):
            t = TransformStamped()
            t.header.stamp = rospy.Time.from_sec(base)
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            statics.append(t)
        static.sendTransform(statics)
        grid = OccupancyGrid()                  # a known, empty floor (slam_toolbox's job)
        grid.header.frame_id = 'map'
        grid.info.resolution, grid.info.width, grid.info.height = 0.05, 600, 400
        grid.info.origin.position.x = grid.info.origin.position.y = -10.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (600 * 400)
        map_pub.publish(grid)

        def world_loop():
            """Robot motion and TF at 50 Hz, a 360-ray scan at 10 Hz, in simulation time."""
            last = next_scan = sim_now()
            while not stop.is_set():
                time.sleep(0.01 / max(rate, 0.1))
                now = sim_now()
                dt = now - last
                if dt < 0.02:
                    continue
                last = now
                a = min(1.0, dt / LAG)
                state['v'] += a * (state['cmd'][0] - state['v'])
                state['w'] += a * (state['cmd'][1] - state['w'])
                state['x'] += state['v'] * math.cos(state['yaw']) * dt
                state['y'] += state['v'] * math.sin(state['yaw']) * dt
                state['yaw'] += state['w'] * dt
                stamp = rospy.Time.from_sec(now)
                t = TransformStamped()
                t.header.stamp, t.header.frame_id, t.child_frame_id = \
                    stamp, 'odom', 'base_footprint'
                t.transform.translation.x, t.transform.translation.y = state['x'], state['y']
                t.transform.rotation.z = math.sin(state['yaw'] / 2)
                t.transform.rotation.w = math.cos(state['yaw'] / 2)
                tf.sendTransform(t)
                if now < next_scan:
                    continue
                next_scan = now + 0.1
                boxes = boxes_at(scene, 0.0 if state['t0'] is None else now - state['t0'])
                ranges = []
                for i in range(360):
                    angle = state['yaw'] + math.radians(i)
                    hits = [d for d in (box_range(state['x'], state['y'], angle, b)
                                        for b in boxes) if d is not None]
                    ranges.append(min(hits) + random.gauss(0.0, NOISE) if hits else float('inf'))
                scan = LaserScan(ranges=ranges)
                scan.header.stamp, scan.header.frame_id = stamp, 'base_scan'
                scan.angle_min, scan.angle_max = 0.0, math.radians(359)
                scan.angle_increment = math.radians(1)
                scan.range_min, scan.range_max, scan.scan_time = 0.12, 100.0, 0.1
                scan_pub.publish(scan)
        threading.Thread(target=world_loop, daemon=True).start()

        time.sleep(6.0 / rate)          # costmaps, perception and the predicted grid settle
        goal = PoseStamped()
        goal.header.frame_id, goal.header.stamp = 'map', rospy.Time.from_sec(sim_now())
        goal.pose.position.x, goal.pose.position.y = scene['goal']
        goal.pose.orientation.w = 1.0
        state['t0'] = sim_now()
        goal_pub.publish(goal)

        trace, status = [], None
        while sim_now() - state['t0'] < limit:
            time.sleep(0.1 / rate)
            elapsed = sim_now() - state['t0']
            clear = body_clearance(state['x'], state['y'], state['yaw'],
                                   boxes_at(scene, elapsed))
            trace.append((elapsed, state['yaw'], clear, state['v']))
            done = [s for t, s in results if t >= state['t0']]
            if done:
                status = done[-1]
                break
        return {
            'scene': name,
            'ok': status is not None and status.status == 3,
            'time': trace[-1][0],
            'stopped': sum(0.1 for t in trace if abs(t[3]) < 0.02),
            'rotation': math.degrees(sum(abs(b[1] - a[1]) for a, b in zip(trace, trace[1:]))),
            'clearance': min(t[2] for t in trace),
            'goal_error': math.hypot(state['x'] - scene['goal'][0],
                                     state['y'] - scene['goal'][1]),
            'status': status.text if status is not None else 'no result in %.0f s' % limit,
            'logs': work,
        }
    finally:
        stop.set()
        rospy.signal_shutdown('done')
        if roslaunch.poll() is None:
            os.killpg(roslaunch.pid, signal.SIGINT)
        try:
            roslaunch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(roslaunch.pid, signal.SIGKILL)
            roslaunch.wait()
        log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('scenes', nargs='*', default=list(SCENES), help='default: all')
    parser.add_argument('--runs', type=int, default=1, help='trials per scene')
    parser.add_argument('--parallel', type=int, default=max(1, (os.cpu_count() or 2) - 1))
    parser.add_argument('--rate', type=float, default=0.3,
                        help='simulation seconds per wall second (lower for busy machines)')
    parser.add_argument('--limit', type=float, default=100.0, help='simulation seconds per trial')
    parser.add_argument('--no-perception', action='store_true',
                        help='scan-only avoidance (the launch file\'s perception:=false)')
    parser.add_argument('--trial', help=argparse.SUPPRESS)
    parser.add_argument('--port', type=int, default=11320, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.trial:
        result = run_trial(args.trial, args.port, args.rate, not args.no_perception, args.limit)
        print('RESULT ' + json.dumps(result), flush=True)
        return
    unknown = [s for s in args.scenes if s not in SCENES]
    if unknown:
        parser.error('unknown scene(s) %s; choose from %s' % (unknown, ', '.join(SCENES)))

    queue = [s for _ in range(args.runs) for s in args.scenes]
    running, results, port = [], {}, args.port
    while queue or running:
        while queue and len(running) < args.parallel:
            scene = queue.pop(0)
            command = [sys.executable, os.path.abspath(__file__), '--trial', scene,
                       '--port', str(port), '--rate', str(args.rate), '--limit', str(args.limit)]
            if args.no_perception:
                command.append('--no-perception')
            running.append((scene, subprocess.Popen(command, stdout=subprocess.PIPE,
                                                    stderr=subprocess.DEVNULL, text=True)))
            port += 1
        time.sleep(0.5)
        for scene, process in list(running):
            if process.poll() is None:
                continue
            running.remove((scene, process))
            lines = [l for l in process.stdout.read().splitlines() if l.startswith('RESULT ')]
            result = json.loads(lines[-1][7:]) if lines else {
                'scene': scene, 'ok': False, 'time': 0, 'stopped': 0, 'rotation': 0,
                'clearance': float('nan'), 'goal_error': float('nan'), 'status': 'trial crashed',
                'logs': ''}
            results.setdefault(scene, []).append(result)
            print('%-13s %s  %5.1f s  stopped %4.1f s  rotation %4.0f deg  clearance %5.2f m  '
                  'off goal %4.2f m  %s'
                  % (scene, 'ok  ' if result['ok'] else 'FAIL', result['time'], result['stopped'],
                     result['rotation'], result['clearance'], result['goal_error'],
                     result['status']), flush=True)

    print('\n%-13s %7s %8s %10s %10s %10s %9s' % ('scene', 'reached', 'time s', 'stopped s',
                                                    'rotation', 'clearance', 'off goal'))
    for scene in args.scenes:
        rs = results.get(scene, [])
        if not rs:
            continue
        print('%-13s %4d/%-2d %8.0f %10.0f %9.0f° %9.2f %9.2f' % (
            scene, sum(r['ok'] for r in rs), len(rs),
            max(r['time'] for r in rs), max(r['stopped'] for r in rs),
            max(r['rotation'] for r in rs), min(r['clearance'] for r in rs),
            max(r['goal_error'] for r in rs)))


if __name__ == '__main__':
    main()
