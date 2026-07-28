#!/usr/bin/env python3
"""Drive the boat to a sequence of goals through the buoy course.

The end-to-end check: planner produces a path, MPPI tracks it, the boat
arrives. Also records how hard MPPI is working, because a controller that
reaches the goal while missing its control period is not actually passing.

    ros2 launch roboboat_bringup boat_nav.launch.py headless:=true rviz:=false &
    python3 tools/nav_test.py

    python3 tools/nav_test.py --goals "20,6,0 34,20,1.57"

Exits non-zero if any goal is not reached.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from xml.etree import ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
# Aliased: nav_msgs' Path would shadow pathlib.Path.
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
from course_tasks import FOOTPRINT_HALF_Y  # noqa: E402

DEFAULT_GOALS = [(20.0, 6.0, 0.0), (34.0, 20.0, 1.57)]

# The same rectangular footprint nav2_mppi.yaml gives both costmaps, as
# half-extents. Clearance is measured against this polygon, not its
# circumscribed circle: at 0.93 m the circumscribed radius exceeds the real
# half-beam by 0.38 m, which reports a comfortable beam-on pass as a
# collision.
#
# Half-beam itself is imported, not restated: course_tasks needs it to
# normalise gate clearance and cannot import this module (ROS message types),
# so a second copy here is exactly the cross-file drift validate.py exists to
# catch.
FOOTPRINT_HALF_X = 0.75


def footprint_clearance(x: float, y: float, yaw: float,
                        ox: float, oy: float, radius: float) -> float:
    """Gap between the hull rectangle and a circular obstacle, in metres.

    Negative means overlap. Exact for a rectangle: rotate the obstacle into
    the body frame, clamp onto the rectangle to find the nearest point.
    """
    dx, dy = ox - x, oy - y
    cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
    bx = dx * cos_y - dy * sin_y
    by = dx * sin_y + dy * cos_y
    nearest_x = min(max(bx, -FOOTPRINT_HALF_X), FOOTPRINT_HALF_X)
    nearest_y = min(max(by, -FOOTPRINT_HALF_Y), FOOTPRINT_HALF_Y)
    return math.hypot(bx - nearest_x, by - nearest_y) - radius


def load_obstacles(world_path: Path) -> list[dict]:
    """Every obstacle in the generated world, keeping its real shape.

    Read from the world SDF rather than the course YAML because the SDF is
    what Gazebo actually loaded -- if the two ever disagree, this test should
    believe the simulator.

    Colour is carried through so a replay can draw the course the way a chart
    would: red and green lateral marks are not decoration, they are how you
    read which side of a gate you are on.
    """
    obstacles: list[dict] = []
    world = ET.parse(world_path).getroot().find('world')
    for model in world.findall('model'):
        if model.get('name') == 'water':
            continue
        pose = [float(v) for v in model.find('pose').text.split()]
        visual = model.find('link').find('visual')
        geom = visual.find('geometry')

        material = visual.find('material')
        colour = '#888888'
        if material is not None and material.find('diffuse') is not None:
            rgb = [float(v) for v in material.find('diffuse').text.split()[:3]]
            colour = '#%02x%02x%02x' % tuple(
                max(0, min(255, round(c * 255))) for c in rgb)

        if geom.find('sphere') is not None:
            radius = float(geom.find('sphere').find('radius').text)
        elif geom.find('cylinder') is not None:
            radius = float(geom.find('cylinder').find('radius').text)
        elif geom.find('box') is not None:
            size = [float(v) for v in geom.find('box').find('size').text.split()]
            # A 9 m dock wall is not a circle. Collapsing it to a circumscribed
            # disc would put a 4.5 m radius obstacle in open water and report
            # collisions that never happened.
            obstacles.append({
                'shape': 'box', 'x': pose[0], 'y': pose[1], 'yaw': pose[5],
                'hx': size[0] / 2.0, 'hy': size[1] / 2.0, 'colour': colour,
            })
            continue
        else:
            continue

        obstacles.append({'shape': 'circle', 'x': pose[0], 'y': pose[1],
                          'r': radius, 'colour': colour})
    return obstacles


def collision_circles(obstacles: list[dict]) -> list[tuple[float, float, float]]:
    """Flatten obstacles into (x, y, r) circles for the clearance metric.

    Boxes are sampled along their long axis as a chain of circles whose radius
    is the half-thickness -- exact for a capsule, and within a millimetre of a
    thin rectangular wall, which is all the dock is.
    """
    circles: list[tuple[float, float, float]] = []
    for obstacle in obstacles:
        if obstacle['shape'] == 'circle':
            circles.append((obstacle['x'], obstacle['y'], obstacle['r']))
            continue
        hx, hy = obstacle['hx'], obstacle['hy']
        half_long, radius = max(hx, hy), min(hx, hy)
        along = obstacle['yaw'] + (0.0 if hx >= hy else math.pi / 2.0)
        steps = max(2, int(math.ceil(half_long / max(radius, 0.05))))
        for i in range(-steps, steps + 1):
            offset = half_long * i / steps
            circles.append((obstacle['x'] + math.cos(along) * offset,
                            obstacle['y'] + math.sin(along) * offset,
                            radius))
    return circles


class NavTest(Node):

    def __init__(self, goals, timeout: float, obstacles,
                 record: Path | None = None,
                 scorecard: Path | None = None) -> None:
        super().__init__('nav_test', parameter_overrides=[])
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.goals = goals
        self.timeout = timeout
        self.obstacles = obstacles
        self.circles = collision_circles(obstacles)
        self.min_clearance = float('inf')
        self.closest_at = None
        self.cmd_count = 0
        self.first_cmd = None
        self.last_cmd = None
        self.odom: Odometry | None = None

        # Tuning metrics. Command jerk is the one that catches MPPI
        # chattering: a config can reach every goal and still be unusable if
        # it saws the thrusters back and forth to get there.
        self.path_length = 0.0
        self.clearances: list[float] = []
        self.speeds: list[float] = []
        self.jerk_sum = 0.0
        self.jerk_n = 0
        self._prev_cmd = None
        self._prev_xy = None
        self._path_t = 0.0
        self._prev_cmd_t = None
        self.per_goal: list[dict] = []
        self.cmd_periods: list[float] = []
        self.odom_samples = 0
        self.odom_first_t = None
        self.odom_last_t = None
        # Closest approach to each obstacle over the whole run. A run-level
        # minimum is one sample of an extremum; per-obstacle approaches give
        # 10-30 near-deterministic geometric samples for the same wall clock,
        # which is what makes an A/B comparison possible at all.
        self.encounters = [float('inf')] * len(self.circles)

        # Replay recording. Everything is downsampled on the way in: the point
        # is a viewable trace, not a bag file.
        self.record_path = record
        self.scorecard_path = scorecard
        self.frames: list[list] = []
        self.scans: list[dict] = []
        self.plans: list[dict] = []
        self.thrust = [0.0, 0.0, 0.0, 0.0]
        self.goal_log: list[dict] = []
        self.t0: float | None = None
        self._last_frame = 0.0
        self._last_scan = 0.0
        self._last_plan = 0.0

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        if record is not None:
            self.create_subscription(LaserScan, '/scan', self._on_scan, sensor_qos)
            self.create_subscription(NavPath, '/plan', self._on_plan, 10)
            self.create_subscription(
                Float64MultiArray, '/thrusters/thrust', self._on_thrust, 10)

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        # Watching the command stream is how we tell "MPPI is keeping up" from
        # "MPPI is running at half its configured rate and still arriving".
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        # Track how close the hull ever came to an obstacle. Arriving at the
        # goal by driving through a buoy is not a pass.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = self.get_clock().now().nanoseconds * 1e-9

        # Accumulate path length from decimated samples, not every 50 Hz odom
        # frame. Summing |hypot| at full rate adds a bias proportional to the
        # SAMPLE COUNT, i.e. to duration -- so a slower run mechanically scores
        # a longer, less direct path even with identical geometry. Measured in
        # the baseline itself: on leg 3 the slowest run scored +0.98 m over the
        # fastest for +8.7 s, about 2.4 mm per extra sample. That made
        # path_length and efficiency non-independent of reached_seconds, and
        # compare_runs.py was reading them as separate confirmations.
        if self._prev_xy is not None:
            step = math.hypot(x - self._prev_xy[0], y - self._prev_xy[1])
            if stamp - self._path_t >= 0.1:
                self.path_length += step
                self._prev_xy = (x, y)
                self._path_t = stamp
        else:
            self._prev_xy = (x, y)
            self._path_t = stamp
        self.speeds.append(math.hypot(msg.twist.twist.linear.x,
                                      msg.twist.twist.linear.y))

        self.odom_samples += 1
        if self.odom_first_t is None:
            self.odom_first_t = stamp
        self.odom_last_t = stamp

        frame_min = float('inf')
        for i, (ox, oy, radius) in enumerate(self.circles):
            clearance = footprint_clearance(x, y, yaw, ox, oy, radius)
            frame_min = min(frame_min, clearance)
            if clearance < self.encounters[i]:
                self.encounters[i] = clearance
            if clearance < self.min_clearance:
                self.min_clearance = clearance
                self.closest_at = (x, y)
        if frame_min < 1e6:
            self.clearances.append(frame_min)

        if self.record_path is not None:
            now = time.monotonic()
            if self.t0 is None:
                self.t0 = now
            if now - self._last_frame >= 0.1:          # 10 Hz is plenty to watch
                self._last_frame = now
                t = msg.twist.twist
                self.frames.append([
                    round(now - self.t0, 2), round(x, 3), round(y, 3),
                    round(yaw, 4), round(t.linear.x, 3), round(t.linear.y, 3),
                    round(t.angular.z, 3),
                    *[round(f, 1) for f in self.thrust],
                    round(frame_min if frame_min < 1e6 else 99.0, 3),
                ])

    def _on_thrust(self, msg: Float64MultiArray) -> None:
        if len(msg.data) == 4:
            self.thrust = list(msg.data)

    def _on_scan(self, msg: LaserScan) -> None:
        now = time.monotonic()
        if self.t0 is None or now - self._last_scan < 1.0 or self.odom is None:
            return
        self._last_scan = now
        # Store hits already projected into the map frame: the replay just
        # draws points, and this keeps the payload small.
        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        points = []
        for i in range(0, len(msg.ranges), 4):
            r = msg.ranges[i]
            if not math.isfinite(r) or not (msg.range_min < r < msg.range_max):
                continue
            a = yaw + msg.angle_min + i * msg.angle_increment
            points.append([round(px + r * math.cos(a), 2),
                           round(py + r * math.sin(a), 2)])
        self.scans.append({'t': round(now - self.t0, 2), 'p': points})

    def _on_plan(self, msg: NavPath) -> None:
        now = time.monotonic()
        if self.t0 is None or now - self._last_plan < 2.0:
            return
        self._last_plan = now
        step = max(1, len(msg.poses) // 120)
        self.plans.append({
            't': round(now - self.t0, 2),
            'p': [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)]
                  for p in msg.poses[::step]],
        })

    def _on_cmd(self, msg: Twist) -> None:
        # Sim time, not wall clock. The controller publishes on the sim clock,
        # so normalising by wall time makes a config that costs more CPU look
        # *smoother*: real-time factor drops, dt grows, and the rate shrinks.
        # That bias is correlated with exactly the parameters worth tuning
        # (footprint checking, batch size), so it would reward the wrong ones.
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.first_cmd is None:
            self.first_cmd = now
        self.last_cmd = now
        self.cmd_count += 1

        cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        if self._prev_cmd is not None and self._prev_cmd_t is not None:
            dt = now - self._prev_cmd_t
            if dt > 0.0:
                self.cmd_periods.append(dt)
            if 0.0 < dt < 1.0:
                # First difference of command over time: mean absolute command
                # acceleration, not jerk. Named for what it is.
                delta = sum(abs(a - b) for a, b in zip(cmd, self._prev_cmd))
                self.jerk_sum += delta / dt
                self.jerk_n += 1
        self._prev_cmd, self._prev_cmd_t = cmd, now

    def wait_ready(self, seconds: float = 120.0) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.odom is not None and self.client.server_is_ready():
                return True
            self.client.wait_for_server(timeout_sec=0.1)
        return False

    def go(self, x: float, y: float, yaw: float) -> tuple[bool, str]:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, 'goal rejected'

        # Sim time for the metric, wall clock for the timeout guard: the
        # metric must not shift with real-time factor (the same bias already
        # fixed for cmd_jerk and the command period), but a stalled sim clock
        # must still be able to time the test out.
        started_sim = self.get_clock().now().nanoseconds * 1e-9
        started = time.monotonic()
        path_at_start = self.path_length
        start_xy = (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
        leg_encounters = list(self.encounters)
        result_future = handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.monotonic() - started > self.timeout:
                handle.cancel_goal_async()
                rclpy.spin_once(self, timeout_sec=1.0)
                return False, f'timed out after {self.timeout:.0f}s'

        status = result_future.result().status
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - started_sim
        error = math.hypot(self.odom.pose.pose.position.x - x,
                           self.odom.pose.pose.position.y - y)
        ok = status == GoalStatus.STATUS_SUCCEEDED
        travelled = self.path_length - path_at_start
        end_xy = (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
        # Straight line to where the boat ACTUALLY ended, not to the goal
        # centre. Measuring against the centre while the goal checker accepts
        # anything within 0.50 m lets a boat that stops short score
        # efficiency > 1.0 -- the metric would reward undershooting.
        straight = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
        leg_min = min((n for n, o in zip(self.encounters, leg_encounters)
                       if n < o), default=None)
        self.per_goal.append({
            'x': x, 'y': y, 'ok': ok, 'status': int(status),
            'seconds': round(elapsed, 1), 'error_m': round(error, 3),
            'travelled_m': round(travelled, 2),
            'straight_m': round(straight, 2),
            'efficiency': round(straight / travelled, 3) if travelled > 0.1 else None,
            'leg_min_clearance_m': round(leg_min, 3) if leg_min is not None else None,
        })
        return ok, (f'{"reached" if ok else "status " + str(status)} in '
                    f'{elapsed:.0f}s, {error:.2f} m from goal')

    def run(self) -> int:
        if not self.wait_ready():
            print('FATAL: /odom or the navigate_to_pose server never came up')
            return 2

        failures = 0
        goals_reached = 0
        for x, y, yaw in self.goals:
            ok, detail = self.go(x, y, yaw)
            print(f"[{'ok  ' if ok else 'FAIL'}] goal ({x:.1f}, {y:.1f}, "
                  f'{math.degrees(yaw):.0f} deg)  {detail}')
            failures += 0 if ok else 1
            goals_reached += 1 if ok else 0
            if self.record_path is not None:
                self.goal_log.append({
                    'x': x, 'y': y, 'yaw': round(yaw, 4),
                    'ok': ok, 'detail': detail,
                    't': round(time.monotonic() - (self.t0 or time.monotonic()), 2),
                })

        # Clearance over the whole run.
        if self.obstacles:
            clear = self.min_clearance
            ok = clear > 0.0
            where = (f' near ({self.closest_at[0]:.1f}, {self.closest_at[1]:.1f})'
                     if self.closest_at else '')
            print(f"[{'ok  ' if ok else 'FAIL'}] no contact with any obstacle  "
                  f'min clearance {clear:.2f} m{where} '
                  f'({len(self.obstacles)} obstacles, '
                  f'{len(self.circles)} collision primitives)')
            failures += 0 if ok else 1

        if self.cmd_count > 1 and self.first_cmd and self.last_cmd:
            span = self.last_cmd - self.first_cmd
            rate = self.cmd_count / span if span > 0 else 0.0
            # controller_frequency is 20 Hz; well under that means MPPI is not
            # finishing its optimisation inside the control period.
            healthy = rate > 15.0
            print(f"[{'ok  ' if healthy else 'warn'}] controller output rate    "
                  f'{rate:.1f} Hz over {span:.0f}s (configured 20 Hz)')

        if self.record_path is not None:
            self.write_recording()
        if self.scorecard_path is not None:
            self.write_scorecard(goals_reached)

        print(f'\n{goals_reached} of {len(self.goals)} goals reached, '
              f'{failures} check(s) failed')
        return 1 if failures else 0

    def write_scorecard(self, goals_reached: int) -> None:
        """Machine-readable summary for A/B comparison between configs."""
        clear = sorted(self.clearances)
        span = ((self.last_cmd - self.first_cmd)
                if self.first_cmd and self.last_cmd else 0.0)
        reached = [g for g in self.per_goal if g['ok']]

        # Closest approach per obstacle, for everything the boat came near.
        # These are the comparison samples: geometric, near-deterministic, and
        # there are 10-30 of them per run instead of one run-level extremum.
        near = sorted(round(c, 3) for c in self.encounters if c < 3.0)

        periods = sorted(self.cmd_periods)
        odom_span = ((self.odom_last_t - self.odom_first_t)
                     if self.odom_first_t and self.odom_last_t else 0.0)
        card = {
            'goals_total': len(self.goals),
            'goals_reached': goals_reached,
            'min_clearance_m': round(self.min_clearance, 3),
            # 5th percentile is the honest safety number: a single-frame dip
            # is noise, sustained proximity is not.
            'p05_clearance_m': round(clear[len(clear) // 20], 3) if clear else None,
            'median_clearance_m': round(clear[len(clear) // 2], 3) if clear else None,
            'total_seconds': round(sum(g['seconds'] for g in self.per_goal), 1),
            'reached_seconds': round(sum(g['seconds'] for g in reached), 1),
            'path_length_m': round(self.path_length, 2),
            'mean_efficiency': (
                round(sum(g['efficiency'] for g in reached if g['efficiency'])
                      / max(1, len([g for g in reached if g['efficiency']])), 3)
                if reached else None),
            'mean_speed_ms': (round(sum(self.speeds) / len(self.speeds), 3)
                              if self.speeds else None),
            'cmd_jerk': (round(self.jerk_sum / self.jerk_n, 3)
                         if self.jerk_n else None),
            'controller_hz': round(self.cmd_count / span, 2) if span > 0 else None,
            # p95 command period is the honest "did MPPI miss its deadline"
            # number: immune to the idle gaps between goals that drag the
            # naive rate down, and measured on the sim clock.
            'cmd_period_p95_s': (round(periods[int(len(periods) * 0.95)], 4)
                                 if periods else None),
            'cmd_period_median_s': (round(periods[len(periods) // 2], 4)
                                    if periods else None),
            'mean_error_m': (round(sum(g['error_m'] for g in self.per_goal)
                                   / len(self.per_goal), 3)
                             if self.per_goal else None),
            'encounters': near,
            'encounter_count': len(near),
            'encounter_min_mean': (round(sum(near) / len(near), 3)
                                   if near else None),
            # Diagnostic: clearance is only evaluated on odom frames actually
            # delivered. A single-threaded spin competing with cmd_vel and
            # action feedback can silently drop them, putting blind spots in
            # the very metric being compared.
            'odom_samples': self.odom_samples,
            'odom_rate_hz': (round(self.odom_samples / odom_span, 1)
                             if odom_span > 0 else None),
            'per_goal': self.per_goal,
        }
        self.scorecard_path.parent.mkdir(parents=True, exist_ok=True)
        self.scorecard_path.write_text(json.dumps(card, indent=2))
        print(f'scorecard -> {self.scorecard_path}')

    def write_recording(self) -> None:
        payload = {
            'obstacles': self.obstacles,
            'footprint': [FOOTPRINT_HALF_X, FOOTPRINT_HALF_Y],
            'goals': self.goal_log,
            'frames': self.frames,
            'scans': self.scans,
            'plans': self.plans,
            'min_clearance': round(self.min_clearance, 3),
            'frame_fields': ['t', 'x', 'y', 'yaw', 'vx', 'vy', 'wz',
                             'f_fl', 'f_fr', 'f_rl', 'f_rr', 'clearance'],
        }
        self.record_path.parent.mkdir(parents=True, exist_ok=True)
        self.record_path.write_text(json.dumps(payload, separators=(',', ':')))
        size = self.record_path.stat().st_size / 1024.0
        print(f'\nrecorded {len(self.frames)} frames, {len(self.scans)} scans, '
              f'{len(self.plans)} plans -> {self.record_path} ({size:.0f} KB)')


def parse_goals(text: str | None):
    if not text:
        return DEFAULT_GOALS
    out = []
    for chunk in text.split():
        parts = [float(v) for v in chunk.split(',')]
        out.append((parts[0], parts[1], parts[2] if len(parts) > 2 else 0.0))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goals', default=None,
                        help='space-separated x,y[,yaw] triples')
    parser.add_argument('--timeout', type=float, default=180.0,
                        help='seconds allowed per goal')
    parser.add_argument('--world', type=Path, default=None,
                        help='world SDF to read obstacle geometry from')
    parser.add_argument('--record', type=Path, default=None,
                        help='write a JSON replay trace to this path')
    parser.add_argument('--scorecard', type=Path, default=None,
                        help='write machine-readable tuning metrics here')
    args = parser.parse_args()

    world = args.world
    if world is None:
        from ament_index_python.packages import get_package_share_directory
        world = Path(get_package_share_directory('roboboat_description')) \
            / 'worlds' / 'roboboat_course.sdf'
    obstacles = load_obstacles(world) if world.exists() else []

    rclpy.init()
    node = NavTest(parse_goals(args.goals), args.timeout, obstacles,
                   args.record, args.scorecard)
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
