#!/usr/bin/env python3
"""Run the actual RoboBoat course: through every gate, in order, hitting nothing.

This is a trial, not a tuning benchmark. The difference matters.

`benchmark.sh` deliberately places its goals in open water so that minimum
clearance measures the controller rather than where the boat parked inside a
goal tolerance. That is the right choice for A/B comparison and the wrong
choice for a demonstration: it never requires the boat to thread a gate.

Here the waypoints ARE the gate centres, taken from the course file, and the
run is scored the way the task is scored:

  * did the hull pass BETWEEN the red and green of every gate, in order,
  * without contacting anything.

Passing near a gate, or around the outside of it, is a miss even if the boat
ends up in the right place afterwards.

It uses NavigateThroughPoses rather than a sequence of NavigateToPose goals,
because the task is one continuous transit. Sending ten separate goals would
make the boat stop and satisfy a yaw tolerance at each gate, which is neither
what the task asks for nor how it would ever be driven.

    python3 tools/course_run.py --record trace.json --scorecard card.json
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
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses
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


def gate_buoys(gate: dict, radius: float) -> tuple[tuple, tuple]:
    """The two buoy centres of a gate, port and starboard."""
    heading = math.radians(float(gate['heading_deg']))
    half = float(gate['width']) / 2.0
    px, py = -math.sin(heading) * half, math.cos(heading) * half
    cx, cy = float(gate['x']), float(gate['y'])
    return (cx + px, cy + py), (cx - px, cy - py)


def segments_cross(p1, p2, q1, q2) -> bool:
    """True if segment p1-p2 properly crosses segment q1-q2."""
    def side(a, b, c):
        return ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    d1, d2 = side(q1, q2, p1), side(q1, q2, p2)
    d3, d4 = side(p1, p2, q1), side(p1, p2, q2)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


class CourseRun(Node):

    def __init__(self, course: dict, obstacles: list[dict],
                 record: Path | None, scorecard: Path | None) -> None:
        super().__init__('course_run', parameter_overrides=[])
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.course = course
        self.obstacles = obstacles
        self.circles = collision_circles(obstacles)
        self.record_path = record
        self.scorecard_path = scorecard

        types = course['buoy_types']
        self.gates = []
        for gate in course['gates']:
            radius = float(types[gate.get('left', 'green')]['radius'])
            left, right = gate_buoys(gate, radius)
            self.gates.append({
                'name': gate['name'],
                'x': float(gate['x']), 'y': float(gate['y']),
                'yaw': math.radians(float(gate['heading_deg'])),
                'left': left, 'right': right,
                'width': float(gate['width']),
                'free_water': float(gate['width']) - 2 * radius,
            })

        self.track: list[tuple] = []
        self.frames: list[list] = []
        self.scans: list[dict] = []
        self.plans: list[dict] = []
        self.thrust = [0.0] * 4
        self.min_clearance = float('inf')
        self.closest_at = None
        self.encounters = [float('inf')] * len(self.circles)
        self.odom: Odometry | None = None
        self.t0 = None
        self._last_frame = self._last_scan = self._last_plan = 0.0
        self._prev_xy = None
        self._path_t = 0.0
        self.path_length = 0.0

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(LaserScan, '/scan', self._on_scan, sensor_qos)
        self.create_subscription(NavPath, '/plan', self._on_plan, 10)
        self.create_subscription(
            Float64MultiArray, '/thrusters/thrust', self._on_thrust, 10)
        self.client = ActionClient(self, NavigateThroughPoses,
                                   'navigate_through_poses')

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

        # Full-rate track: gate crossings are geometric events and must not be
        # missed by decimation.
        self.track.append((x, y))

        if self._prev_xy is None:
            self._prev_xy, self._path_t = (x, y), stamp
        elif stamp - self._path_t >= 0.1:
            self.path_length += math.hypot(x - self._prev_xy[0], y - self._prev_xy[1])
            self._prev_xy, self._path_t = (x, y), stamp

        frame_min = float('inf')
        for i, (ox, oy, radius) in enumerate(self.circles):
            clearance = footprint_clearance(x, y, yaw, ox, oy, radius)
            frame_min = min(frame_min, clearance)
            if clearance < self.encounters[i]:
                self.encounters[i] = clearance
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
            if self.odom is not None and self.client.server_is_ready():
                return True
            self.client.wait_for_server(timeout_sec=0.1)
        return False

    def go(self, timeout: float) -> tuple[bool, str]:
        goal = NavigateThroughPoses.Goal()
        for gate in self.gates:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = gate['x']
            pose.pose.position.y = gate['y']
            pose.pose.orientation.z = math.sin(gate['yaw'] / 2.0)
            pose.pose.orientation.w = math.cos(gate['yaw'] / 2.0)
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
            if time.monotonic() - started > timeout:
                handle.cancel_goal_async()
                rclpy.spin_once(self, timeout_sec=1.0)
                return False, f'timed out after {timeout:.0f}s'
        status = result.result().status
        return status == GoalStatus.STATUS_SUCCEEDED, f'status {status}'

    # ------------------------------------------------------------- scoring
    def score_gates(self) -> list[dict]:
        """Did the hull cross each gate line, between the buoys, in order?"""
        results = []
        search_from = 0
        for gate in self.gates:
            crossed_at = None
            for i in range(search_from, len(self.track) - 1):
                if segments_cross(self.track[i], self.track[i + 1],
                                  gate['left'], gate['right']):
                    crossed_at = i
                    break
            if crossed_at is not None:
                # Offset from the gate centreline at the moment of crossing.
                px, py = self.track[crossed_at]
                lx, ly = gate['left']
                rx, ry = gate['right']
                mid = ((lx + rx) / 2.0, (ly + ry) / 2.0)
                offset = math.hypot(px - mid[0], py - mid[1])
                search_from = crossed_at + 1
            else:
                offset = None
            results.append({
                'gate': gate['name'], 'transited': crossed_at is not None,
                'offset_from_centre_m': round(offset, 2) if offset else None,
                'free_water_m': round(gate['free_water'], 2),
            })
        return results

    def report(self) -> int:
        gates = self.score_gates()
        transited = sum(1 for g in gates if g['transited'])
        contact = self.min_clearance <= 0.0

        print()
        print(f'{"gate":>10} {"transited":>10} {"offset from centre":>20} '
              f'{"free water":>12}')
        for g in gates:
            mark = 'YES' if g['transited'] else 'MISSED'
            off = f"{g['offset_from_centre_m']:.2f} m" if g['offset_from_centre_m'] is not None else '-'
            print(f"{g['gate']:>10} {mark:>10} {off:>20} {g['free_water_m']:>10.2f} m")

        print()
        print(f'  gates transited : {transited}/{len(gates)}')
        print(f'  contact         : {"YES" if contact else "none"} '
              f'(min clearance {self.min_clearance:.3f} m'
              + (f' near ({self.closest_at[0]:.1f}, {self.closest_at[1]:.1f})'
                 if self.closest_at else '') + ')')
        print(f'  course distance : {self.path_length:.1f} m')

        if self.scorecard_path is not None:
            self.scorecard_path.parent.mkdir(parents=True, exist_ok=True)
            self.scorecard_path.write_text(json.dumps({
                'gates_total': len(gates),
                'gates_transited': transited,
                'contact': contact,
                'min_clearance_m': round(self.min_clearance, 3),
                'path_length_m': round(self.path_length, 2),
                'gates': gates,
            }, indent=2))

        if self.record_path is not None:
            self.record_path.parent.mkdir(parents=True, exist_ok=True)
            self.record_path.write_text(json.dumps({
                'obstacles': self.obstacles,
                'footprint': [FOOTPRINT_HALF_X, FOOTPRINT_HALF_Y],
                'goals': [{'x': g['x'], 'y': g['y'], 'yaw': round(g['yaw'], 4),
                           'ok': gates[i]['transited'],
                           'detail': f"gate {g['name']}",
                           't': self.frames[-1][0] if self.frames else 0.0}
                          for i, g in enumerate(self.gates)],
                'frames': self.frames, 'scans': self.scans, 'plans': self.plans,
                'min_clearance': round(self.min_clearance, 3),
                'frame_fields': ['t', 'x', 'y', 'yaw', 'vx', 'vy', 'wz',
                                 'f_fl', 'f_fr', 'f_rl', 'f_rr', 'clearance'],
            }, separators=(',', ':')))
            print(f'  trace           : {self.record_path}')

        return 0 if (transited == len(gates) and not contact) else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--course', type=Path, default=None)
    parser.add_argument('--world', type=Path, default=None)
    parser.add_argument('--record', type=Path, default=None)
    parser.add_argument('--scorecard', type=Path, default=None)
    parser.add_argument('--timeout', type=float, default=420.0)
    args = parser.parse_args()

    share = Path(get_package_share_directory('roboboat_description'))
    course = yaml.safe_load(
        (args.course or share / 'config' / 'course_default.yaml').read_text())
    obstacles = load_obstacles(
        args.world or share / 'worlds' / 'roboboat_course.sdf')

    rclpy.init()
    node = CourseRun(course, obstacles, args.record, args.scorecard)
    try:
        if not node.wait_ready():
            print('FATAL: navigate_through_poses server never came up')
            return 2
        print(f'running the course: {len(node.gates)} gates')
        ok, detail = node.go(args.timeout)
        print(f'  navigation result: {"SUCCEEDED" if ok else detail}')
        return node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
