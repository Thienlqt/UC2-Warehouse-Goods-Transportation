"""Repeatable avoidance trials against the installed Noetic move_base (no Unity required).

Each trial starts its own ROS master and runs move_base, cmd_vel_guard, tf_odometry,
predicted_obstacles, teleop_assist and (by default) lidar_obstacles + obstacle_fusion with this
package's config, against a simulated robot (unicycle with a 0.3 s motor lag, driven by /cmd_vel)
and a world of boxes, some of them moving. The trial publishes /clock itself, so a busy machine or
a wall-clock jump (Docker Desktop resyncs its VM clock) slows a trial down without changing it.

Run explicitly after building the workspace and sourcing devel/setup.bash:
  python3 test/scenarios.py                  # every scene once
  python3 test/scenarios.py rack_end cross --runs 3
  python3 test/scenarios.py busy --runs 5 --seed 0   # run k: traffic seed k, for every planner
Prints one line per scene: goals reached, time, time stopped, the longest stall and traffic wait
(stall_report), total rotation, closest clearance of the robot's colliders to anything and to
movers (0: contact), how far from the goal it ended, and planner warnings counted in the node
log. These checks do not replace Unity trials.
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

from busy_traffic import BOXES as BUSY_BOXES, GOAL as BUSY_GOAL, Traffic

RADIUS = 0.25                    # robot_radius in config/move_base.yaml
# The Unity TurtleBot3 Waffle's colliders (UnityProject/Assets/turtlebot3/model.urdf), base_link
# frame: the body box (x -0.197..0.069, y +-0.133) and the wheels (x +-0.033, y +-0.153).
# Clearance is measured from these, so a corner swinging into a shelf counts as contact.
BODY = [(-0.197, -0.133, 0.069, 0.133), (-0.033, -0.153, 0.033, 0.153)]
GOAL_TOLERANCE = 0.25            # xy_goal_tolerance: the final turn to the goal heading is free
STALL_SPEED = 0.02               # slower than this (m/s, either way) counts as stalled
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
    # Ten carts roaming warehouse aisles from a seed (busy_traffic): the robot crosses the whole
    # floor among them. Traffic waits do not count as stalls (stall_report).
    'busy': dict(goal=BUSY_GOAL, boxes=BUSY_BOXES, traffic=dict(carts=10), limit=240.0),
}

LAUNCH = """<launch>
  <param name="/use_sim_time" value="true"/>
  <node pkg="move_base" type="move_base" name="move_base">
    <rosparam command="load" file="{config}/move_base.yaml"/>
    {local_planner}
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
  {slam}
</launch>
"""
PERCEPTION = """<node pkg="unity_slam_example" type="lidar_obstacles" name="lidar_obstacles"/>
  <node pkg="unity_slam_example" type="obstacle_fusion" name="obstacle_fusion"/>"""
# --slam: slam_toolbox maps /scan_static and publishes map -> odom, as in the launch file.
SLAM = """<node pkg="slam_toolbox" type="async_slam_toolbox_node" name="slam_toolbox">
    <rosparam command="load" file="{config}/slam_toolbox.yaml"/>
    <param name="scan_topic" value="/scan_static"/>
  </node>"""
# --planner: the launch file's planner:=dwa|teb (local planner and its obstacle inputs).
LOCAL_PLANNER = {
    'dwa': '<rosparam command="load" file="{config}/move_base_predicted.yaml"/>',
    'teb': '<rosparam command="load" file="{config}/teb_local_planner.yaml"/>',
}
TEB_OBSTACLES = '<node pkg="unity_slam_example" type="teb_obstacles" name="teb_obstacles"/>'
# Counted in each trial's node log (see the trial's 'logs' directory).
LOG_COUNTS = {'empty_plan': ('Received an empty transformed plan',),
              'no_path': ('DWA planner failed to produce path',
                          'TebLocalPlannerROS: trajectory is not feasible',
                          'teb_local_planner was not able to obtain a local plan for the'),
              'recoveries': ('Clearing both costmaps',)}


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


def box_points(box, spacing=0.1):
    """Points along the perimeter of an axis-aligned box (cx, cy, hx, hy)."""
    cx, cy, hx, hy = box[:4]
    corners = [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]
    points = []
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
        n = max(1, int(math.ceil(math.hypot(bx - ax, by - ay) / spacing)))
        points += [(ax + (bx - ax) * k / n, ay + (by - ay) * k / n) for k in range(n)]
    return points


def traffic_blocking(x, y, yaw, outlines, distance=1.5, cone_deg=45.0, radius=RADIUS):
    """True when a mover's outline is within `distance` of the robot and `cone_deg` of its nose.

    A stop then counts as a traffic wait, not a stall. The robot's body is its `radius` circle.
    """
    cone = math.radians(cone_deg)
    for outline in outlines:
        for px, py in outline:
            dx, dy = px - x, py - y
            off = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
            if math.hypot(dx, dy) - radius <= distance and abs(off) <= cone:
                return True
    return False


def stall_report(trace, goal, tolerance=GOAL_TOLERANCE, speed=STALL_SPEED):
    """Stalls and traffic waits in a trace of (t, x, y, v, waiting) samples.

    Not counted: the first turn (before the robot first drives) and the final turn (from the
    first sample within `tolerance` of the goal). A waiting sample belongs to a traffic wait and
    ends a stall. A run lasts from its first sample to the next sample after it.
    """
    start = next((i for i, s in enumerate(trace) if abs(s[3]) >= speed), len(trace))
    end = next((i for i, s in enumerate(trace)
                if math.hypot(s[1] - goal[0], s[2] - goal[1]) <= tolerance), len(trace))
    stalls, waits = [], []
    run, kind = None, None
    for i in range(start, max(start, end)):
        t, _, _, v, waiting = trace[i]
        now = 'wait' if waiting and abs(v) < speed else 'stall' if abs(v) < speed else None
        if now != kind and run is not None:
            (stalls if kind == 'stall' else waits).append(t - run)
            run = None
        if now is not None and run is None:
            run = t
        kind = now
    if run is not None:
        last = trace[end][0] if end < len(trace) else trace[-1][0]
        (stalls if kind == 'stall' else waits).append(last - run)
    return {
        'longest_stall': round(max(stalls, default=0.0), 2),
        'stalls_over_3s': sum(s > 3.0 for s in stalls),
        'wait_longest': round(max(waits, default=0.0), 2),
        'wait_total': round(sum(waits), 2),
        'waits': len(waits),
    }


def boxes_at(scene, elapsed):
    """Static boxes plus movers at `elapsed` seconds after the goal was sent."""
    return list(scene.get('boxes', [])) + movers_at(scene, elapsed)


def movers_at(scene, elapsed):
    """The scene's back-and-forth movers at `elapsed` seconds after the goal was sent."""
    out = []
    for cx, cy, hx, hy, vx, vy, start, travel in scene.get('movers', []):
        speed = math.hypot(vx, vy)
        d = max(0.0, elapsed - start) * speed % (2 * travel)
        d = d if d <= travel else 2 * travel - d
        out.append((cx + vx / speed * d, cy + vy / speed * d, hx, hy))
    return out


def log_counts(path):
    """How often each LOG_COUNTS message appears in a trial's node log."""
    with open(path, errors='replace') as f:
        text = f.read()
    return {key: sum(text.count(m) for m in messages) for key, messages in LOG_COUNTS.items()}


def run_trial(name, port, rate, perception, limit, seed=0, slam=False, planner='dwa'):
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
    limit = scene.get('limit', limit)
    traffic = Traffic(seed, **scene['traffic']) if 'traffic' in scene else None
    work = tempfile.mkdtemp(prefix='scenario-%s-' % name)
    log = open(os.path.join(work, 'nodes.log'), 'w')
    config = os.path.join(rospkg.RosPack().get_path('unity_slam_example'), 'config')
    launch = os.path.join(work, 'trial.launch')
    with open(launch, 'w') as f:
        f.write(LAUNCH.format(config=config, radius=RADIUS,
                              local_planner=LOCAL_PLANNER[planner].format(config=config),
                              perception=(PERCEPTION + ('\n  ' + TEB_OBSTACLES if planner == 'teb'
                                                        else '')) if perception else '',
                              slam=SLAM.format(config=config) if slam else ''))
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
                 't0': None, 'movers': traffic.boxes() if traffic else []}

        def movers_now(elapsed):
            return state['movers'] if traffic else movers_at(scene, elapsed)
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
        frames = [('map', 'odom', 0.0)] if not slam else []     # with --slam, slam_toolbox's
        for parent, child, z in frames + [('base_footprint', 'base_link', 0.0),
                                          ('base_link', 'base_scan', 0.2)]:
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
        if not slam:
            map_pub.publish(grid)

        def world_loop():
            """Robot motion and TF at 50 Hz, a 360-ray scan at 10 Hz, in simulation time."""
            last = next_scan = sim_now()
            last_traffic = [last]           # traffic steps at the scan rate (10 Hz)
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
                if traffic and state['t0'] is not None:
                    traffic.step(now - last_traffic[0], (state['x'], state['y']), RADIUS)
                    state['movers'] = traffic.boxes()
                last_traffic[0] = now
                elapsed = 0.0 if state['t0'] is None else now - state['t0']
                boxes = list(scene.get('boxes', [])) + movers_now(elapsed)
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
            x, y, yaw, movers = state['x'], state['y'], state['yaw'], movers_now(elapsed)
            statics = list(scene.get('boxes', []))
            mover_clear = body_clearance(x, y, yaw, movers)
            clear = min(body_clearance(x, y, yaw, statics), mover_clear)
            waiting = traffic_blocking(x, y, yaw, [box_points(b) for b in movers])
            trace.append((elapsed, x, y, state['v'], waiting, yaw, clear, mover_clear))
            done = [s for t, s in results if t >= state['t0']]
            if done:
                status = done[-1]
                break
        return {
            'scene': name,
            'ok': status is not None and status.status == 3,
            'time': trace[-1][0],
            'stopped': sum(0.1 for t in trace if abs(t[3]) < 0.02),
            'rotation': math.degrees(sum(abs(b[5] - a[5]) for a, b in zip(trace, trace[1:]))),
            'clearance': min(t[6] for t in trace),
            'mover_clearance': min(t[7] for t in trace),
            **stall_report([t[:5] for t in trace], scene['goal']),
            **log_counts(os.path.join(work, 'nodes.log')),
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
    parser.add_argument('--seed', type=int, default=0,
                        help='traffic seed of the first run; run k uses seed + k')
    parser.add_argument('--slam', action='store_true',
                        help='map with slam_toolbox (on /scan_static) instead of a known map')
    parser.add_argument('--planner', choices=sorted(LOCAL_PLANNER), default='dwa',
                        help='local planner, as the launch file\'s planner:=')
    parser.add_argument('--trial', help=argparse.SUPPRESS)
    parser.add_argument('--port', type=int, default=11320, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.trial:
        result = run_trial(args.trial, args.port, args.rate, not args.no_perception, args.limit,
                           args.seed, args.slam, args.planner)
        print('RESULT ' + json.dumps(result), flush=True)
        return
    unknown = [s for s in args.scenes if s not in SCENES]
    if unknown:
        parser.error('unknown scene(s) %s; choose from %s' % (unknown, ', '.join(SCENES)))
    if args.slam and args.no_perception:
        parser.error('--slam maps /scan_static, which needs perception')

    queue = [(s, k) for k in range(args.runs) for s in args.scenes]
    running, results, port = [], {}, args.port
    while queue or running:
        while queue and len(running) < args.parallel:
            scene, run = queue.pop(0)
            command = [sys.executable, os.path.abspath(__file__), '--trial', scene,
                       '--port', str(port), '--rate', str(args.rate), '--limit', str(args.limit),
                       '--seed', str(args.seed + run), '--planner', args.planner]
            if args.no_perception:
                command.append('--no-perception')
            if args.slam:
                command.append('--slam')
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
                'logs': '', 'mover_clearance': float('nan'), 'longest_stall': float('nan'),
                'stalls_over_3s': 0, 'wait_longest': float('nan'), 'wait_total': float('nan'),
                'waits': 0, **{key: 0 for key in LOG_COUNTS}}
            results.setdefault(scene, []).append(result)
            print('%-13s %s  %5.1f s  stopped %4.1f s  longest stall %4.1f s  wait %4.1f s  '
                  'rotation %4.0f deg  clearance %5.2f m  movers %5.2f m  off goal %4.2f m  '
                  'empty plan %d  no path %d  recoveries %d  %s'
                  % (scene, 'ok  ' if result['ok'] else 'FAIL', result['time'], result['stopped'],
                     result['longest_stall'], result['wait_longest'], result['rotation'],
                     result['clearance'], result['mover_clearance'], result['goal_error'],
                     result['empty_plan'], result['no_path'], result['recoveries'],
                     result['status']), flush=True)

    print('\n%-13s %7s %8s %10s %8s %8s %10s %10s %8s %9s' % (
        'scene', 'reached', 'time s', 'stopped s', 'stall s', 'wait s', 'rotation', 'clearance',
        'movers', 'off goal'))
    for scene in args.scenes:
        rs = results.get(scene, [])
        if not rs:
            continue
        print('%-13s %4d/%-2d %8.0f %10.0f %8.1f %8.1f %9.0f° %9.2f %8.2f %9.2f' % (
            scene, sum(r['ok'] for r in rs), len(rs),
            max(r['time'] for r in rs), max(r['stopped'] for r in rs),
            max(r['longest_stall'] for r in rs), max(r['wait_longest'] for r in rs),
            max(r['rotation'] for r in rs), min(r['clearance'] for r in rs),
            min(r['mover_clearance'] for r in rs), max(r['goal_error'] for r in rs)))


if __name__ == '__main__':
    main()
