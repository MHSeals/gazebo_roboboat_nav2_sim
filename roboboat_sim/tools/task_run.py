#!/usr/bin/env python3
"""Run one RoboBoat task and score it the way that task is actually scored.

This generalises tools/course_run.py, which could only do one thing: send
every gate centre in the course to NavigateThroughPoses and check the hull
crossed each gate line in order. That is the right scoring rule for the
navigation channel and the wrong rule for every other task on the course --
"crossed a line" does not score a task whose objective is to circle a mark.

A task is declared in the course file:

    tasks:
      - {name: channel, type: gate_transit, gates: [start, chan_a, ...]}
      - {name: speed,   type: circle_mark, gate: sprint_entry,
         mark: sprint_mark, direction: ccw, laps: 1, radius: 3.0, points: 8}

and run by name:

    python3 tools/task_run.py --task channel --scorecard card.json

If the course file declares no tasks at all, a single `channel` task covering
every gate in file order is synthesised. That is exactly what course_run.py
did, and it is what makes this refactor checkable: run it against the
unmodified course and the scorecard must match the previous trial.

Per-task behaviour trees are selected through the `behavior_tree` field of the
NavigateThroughPoses goal, so a task that needs different BT parameters costs
one XML file and no launch changes.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathThroughPoses, NavigateThroughPoses
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nav_test import (  # noqa: E402
    FOOTPRINT_HALF_X,
    FOOTPRINT_HALF_Y,
    collision_circles,
    footprint_clearance,
    load_obstacles,
)

from course_tasks import (  # noqa: E402
    TASK_TYPES,
    CircleMark,
    GateTransit,
    Task,
    build_task,
    crossing_point,
    crossings,
    gate_buoys,
    ring_waypoints,
    segments_cross,
    signed_turn,
)


def resolve_bt(name: str) -> Path:
    """Turn a behaviour tree name into the installed absolute path.

    bt_navigator treats the goal's `behavior_tree` field as a filesystem path
    and quietly falls back to its default tree when the file is missing, so a
    bare filename produces a run that looks fine and used the wrong tree.
    Fail here instead.
    """
    path = Path(name)
    if not path.is_absolute():
        share = Path(get_package_share_directory('roboboat_bringup'))
        path = share / 'behavior_trees' / path.name
    if not path.exists():
        raise SystemExit(f'behaviour tree {name!r} is not installed at {path}; '
                         'colcon build the bringup package')
    return path


# ------------------------------------------------------------------ runner


class TaskRun(Node):

    def __init__(self, task: Task, obstacles: list[dict],
                 record: Path | None, scorecard: Path | None) -> None:
        super().__init__('task_run', parameter_overrides=[])
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.task = task
        self.obstacles = obstacles
        self.circles = collision_circles(obstacles)
        self.record_path = record
        self.scorecard_path = scorecard

        self.track: list[tuple] = []
        self.frames: list[list] = []
        self.scans: list[dict] = []
        self.plans: list[dict] = []
        self.thrust = [0.0] * 4
        self.min_clearance = float('inf')
        self.closest_at = None
        self.odom: Odometry | None = None
        self.t0 = None
        self._last_frame = self._last_scan = self._last_plan = 0.0
        self._prev_xy = None
        self._path_t = 0.0
        self.path_length = 0.0
        self.sim_start: float | None = None
        self.sim_end: float | None = None

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(LaserScan, '/scan', self._on_scan, sensor_qos)
        self.create_subscription(NavPath, '/plan', self._on_plan, 10)
        self.create_subscription(
            Float64MultiArray, '/thrusters/thrust', self._on_thrust, 10)
        self.client = ActionClient(self, NavigateThroughPoses,
                                   'navigate_through_poses')
        # bt_navigator reaching 'active' does NOT mean the servers it calls
        # are answering yet. Measured: a goal sent 13.6 s after the lifecycle
        # manager reported all nodes active aborted 30 ms later with 'timed
        # out while waiting for action server to acknowledge goal request for
        # compute_path_through_poses'. That is the fourth distinct way this
        # stack has looked ready without being ready, so gate on the server
        # the tree actually calls, not on the one we call.
        self.planner = ActionClient(self, ComputePathThroughPoses,
                                    'compute_path_through_poses')

    # ---------------------------------------------------------------- inputs
    def _on_thrust(self, msg: Float64MultiArray) -> None:
        if len(msg.data) == 4:
            self.thrust = list(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = stamp

        # Full-rate track: gate crossings and swept angle are geometric events
        # and must not be missed by decimation.
        self.track.append((x, y))

        if self._prev_xy is None:
            self._prev_xy, self._path_t = (x, y), stamp
        elif stamp - self._path_t >= 0.1:
            self.path_length += math.hypot(x - self._prev_xy[0],
                                           y - self._prev_xy[1])
            self._prev_xy, self._path_t = (x, y), stamp

        frame_min = float('inf')
        for ox, oy, radius in self.circles:
            clearance = footprint_clearance(x, y, yaw, ox, oy, radius)
            frame_min = min(frame_min, clearance)
            if clearance < self.min_clearance:
                self.min_clearance, self.closest_at = clearance, (x, y)

        if self.record_path is not None and stamp - self._last_frame >= 0.1:
            self._last_frame = stamp
            t = msg.twist.twist
            self.frames.append([
                round(stamp - self.t0, 2), round(x, 3), round(y, 3), round(yaw, 4),
                round(t.linear.x, 3), round(t.linear.y, 3), round(t.angular.z, 3),
                *[round(f, 1) for f in self.thrust],
                round(frame_min if frame_min < 1e6 else 99.0, 3),
            ])

    def _on_scan(self, msg: LaserScan) -> None:
        stamp = self.get_clock().now().nanoseconds * 1e-9
        if (self.record_path is None or self.t0 is None
                or stamp - self._last_scan < 1.0 or self.odom is None):
            return
        self._last_scan = stamp
        px, py = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y
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
        self.scans.append({'t': round(stamp - self.t0, 2), 'p': points})

    def _on_plan(self, msg: NavPath) -> None:
        stamp = self.get_clock().now().nanoseconds * 1e-9
        if (self.record_path is None or self.t0 is None
                or stamp - self._last_plan < 2.0):
            return
        self._last_plan = stamp
        step = max(1, len(msg.poses) // 150)
        self.plans.append({
            't': round(stamp - self.t0, 2),
            'p': [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)]
                  for p in msg.poses[::step]],
        })

    # ------------------------------------------------------------------ run
    def wait_ready(self, seconds: float = 120.0) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if (self.odom is not None and self.client.server_is_ready()
                    and self.planner.server_is_ready()):
                return True
            self.client.wait_for_server(timeout_sec=0.1)
            self.planner.wait_for_server(timeout_sec=0.1)
        return False

    def _sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def go(self) -> tuple[bool, str]:
        """Drive every leg of the task, in order, stopping at the first failure.

        Elapsed time spans all the legs: the task is timed as a whole, and the
        gap between one goal succeeding and the next being accepted is part of
        what the boat costs to run.
        """
        legs = self.task.legs()
        self.sim_start = self._sim_now()
        for i, leg in enumerate(legs):
            ok, detail = self._drive(leg)
            self.sim_end = self._sim_now()
            if not ok:
                where = f'leg {i + 1}/{len(legs)}: ' if len(legs) > 1 else ''
                return False, where + detail
        return True, f'{len(legs)} leg(s)'

    def _drive(self, waypoints: list[tuple]) -> tuple[bool, str]:
        goal = NavigateThroughPoses.Goal()
        if self.task.bt:
            # bt_navigator resolves this as a filesystem path, so a bare
            # filename silently falls back to the default tree rather than
            # erroring. Always send the installed absolute path.
            goal.behavior_tree = str(resolve_bt(self.task.bt))
        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal.poses.append(pose)

        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=20.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, 'goal rejected'

        started = time.monotonic()
        result = handle.get_result_async()
        while not result.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.monotonic() - started > self.task.timeout:
                handle.cancel_goal_async()
                rclpy.spin_once(self, timeout_sec=1.0)
                return False, f'timed out after {self.task.timeout:.0f}s'
        status = result.result().status
        return status == GoalStatus.STATUS_SUCCEEDED, f'status {status}'

    # ------------------------------------------------------------- scoring
    def report(self) -> int:
        card = self.task.score(self.track, self.min_clearance)
        card['task'] = self.task.name
        card['type'] = self.task.type_name
        card['min_clearance_m'] = round(self.min_clearance, 3)
        card['path_length_m'] = round(self.path_length, 2)
        elapsed = ((self.sim_end - self.sim_start)
                   if None not in (self.sim_start, self.sim_end) else None)
        card['elapsed_s'] = round(elapsed, 2) if elapsed else None
        card['mean_speed_mps'] = (round(self.path_length / elapsed, 3)
                                  if elapsed and elapsed > 0 else None)

        print()
        for line in self.task.describe(card):
            print(line)
        print(f'  contact         : {"YES" if card["contact"] else "none"} '
              f'(min clearance {self.min_clearance:.3f} m'
              + (f' near ({self.closest_at[0]:.1f}, {self.closest_at[1]:.1f})'
                 if self.closest_at else '') + ')')
        print(f'  course distance : {self.path_length:.1f} m')
        if card['elapsed_s']:
            print(f'  elapsed         : {card["elapsed_s"]:.1f} s (sim clock), '
                  f'mean {card["mean_speed_mps"]:.2f} m/s')
        print(f'  result          : {"PASS" if card["passed"] else "FAIL"}')

        if self.scorecard_path is not None:
            self.scorecard_path.parent.mkdir(parents=True, exist_ok=True)
            self.scorecard_path.write_text(json.dumps(card, indent=2))

        if self.record_path is not None:
            self.record_path.parent.mkdir(parents=True, exist_ok=True)
            goals = []
            for i, (x, y, yaw) in enumerate(self.task.waypoints()):
                goals.append({'x': x, 'y': y, 'yaw': round(yaw, 4),
                              'ok': card['passed'],
                              'detail': f'{self.task.name} wp {i}',
                              't': self.frames[-1][0] if self.frames else 0.0})
            self.record_path.write_text(json.dumps({
                'obstacles': self.obstacles,
                'footprint': [FOOTPRINT_HALF_X, FOOTPRINT_HALF_Y],
                'goals': goals,
                'frames': self.frames, 'scans': self.scans, 'plans': self.plans,
                'min_clearance': round(self.min_clearance, 3),
                'frame_fields': ['t', 'x', 'y', 'yaw', 'vx', 'vy', 'wz',
                                 'f_fl', 'f_fr', 'f_rl', 'f_rr', 'clearance'],
            }, separators=(',', ':')))
            print(f'  trace           : {self.record_path}')

        return 0 if card['passed'] else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=('Requires an active Nav2 stack. List tasks first with --list. '
                'For a self-contained headless run, use tools/task_trial.sh.'))
    parser.add_argument('--task', default='channel',
                        help='declared task name (default: channel)')
    parser.add_argument('--course', type=Path, default=None,
                        help='course YAML; defaults to the installed course')
    parser.add_argument('--world', type=Path, default=None,
                        help='world SDF used for obstacle scoring')
    parser.add_argument('--record', type=Path, default=None,
                        help='write recorded telemetry JSON to this path')
    parser.add_argument('--scorecard', type=Path, default=None,
                        help='write pass/fail scorecard JSON to this path')
    parser.add_argument('--list', action='store_true',
                        help='print the declared tasks and exit')
    args = parser.parse_args()

    share = Path(get_package_share_directory('roboboat_description'))
    course = yaml.safe_load(
        (args.course or share / 'config' / 'course_default.yaml').read_text())

    if args.list:
        for spec in course.get('tasks', [{'name': 'channel',
                                          'type': 'gate_transit'}]):
            print(f"{spec['name']:>12}  {spec.get('type', 'gate_transit')}")
        return 0

    task = build_task(args.task, course)
    obstacles = load_obstacles(
        args.world or share / 'worlds' / 'roboboat_course.sdf')

    rclpy.init()
    node = TaskRun(task, obstacles, args.record, args.scorecard)
    try:
        if not node.wait_ready():
            print('FATAL: navigate_through_poses server never came up')
            return 2
        legs = task.legs()
        total = sum(len(leg) for leg in legs)
        print(f'task {task.name} ({task.type_name}): {total} waypoints in '
              f'{len(legs)} leg(s)')
        ok, detail = node.go()
        print(f'  navigation result: {"SUCCEEDED" if ok else detail}')
        return node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
