#!/usr/bin/env python3
"""Functional smoke test against a running sim.

Checks the things static validation cannot: that the bridge is wired up, that
sensor frames survived the URDF -> SDF conversion, that the sign conventions
are right, and that thrust saturation actually binds.

    ros2 launch roboboat_bringup sim.launch.py headless:=true &
    python3 tools/smoke_test.py

Exits non-zero if any check fails. Sign errors in sway or yaw are the single
most likely thing to go wrong when this is ported to another simulator, and
they are almost invisible by eye -- hence checks 5 and 6.
"""

from __future__ import annotations

import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

TIMEOUT = 60.0          # seconds of wall clock to wait for first messages


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SmokeTest(Node):

    def __init__(self) -> None:
        super().__init__('smoke_test', parameter_overrides=[])
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.odom: Odometry | None = None
        self.scan: LaserScan | None = None
        self.results: list[tuple[bool, str, str]] = []

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, sensor_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    # ---------------------------------------------------------------- helpers
    def check(self, ok: bool, name: str, detail: str = '') -> None:
        self.results.append((bool(ok), name, detail))

    def spin_for(self, seconds: float, twist: Twist | None = None) -> None:
        """Spin wall-clock seconds, optionally holding a command."""
        end = self.get_clock_wall() + seconds
        while self.get_clock_wall() < end:
            if twist is not None:
                self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

    @staticmethod
    def get_clock_wall() -> float:
        import time
        return time.monotonic()

    def wait_for_data(self) -> bool:
        end = self.get_clock_wall() + TIMEOUT
        while self.get_clock_wall() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.odom is not None and self.scan is not None:
                return True
        return False

    def drive(self, vx=0.0, vy=0.0, wz=0.0, seconds=8.0) -> dict:
        """Hold a command, then report what the boat did."""
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
        start = self.odom
        x0, y0, yaw0 = (start.pose.pose.position.x,
                        start.pose.pose.position.y, yaw_of(start))
        self.spin_for(seconds, cmd)
        end = self.odom
        return {
            'dx': end.pose.pose.position.x - x0,
            'dy': end.pose.pose.position.y - y0,
            'dyaw': math.atan2(math.sin(yaw_of(end) - yaw0),
                               math.cos(yaw_of(end) - yaw0)),
            'vx': end.twist.twist.linear.x,
            'vy': end.twist.twist.linear.y,
            'wz': end.twist.twist.angular.z,
        }

    def stop(self) -> None:
        self.spin_for(6.0, Twist())

    # ------------------------------------------------------------------ tests
    def run(self) -> int:
        if not self.wait_for_data():
            print(f'FATAL: no /odom and /scan within {TIMEOUT:.0f}s -- '
                  'is the sim running?')
            return 2

        # 1. Sensor frame survived the URDF -> SDF conversion. sdformat warns
        #    that <gz_frame_id> is not a known <sensor> child, but preserves
        #    it; this confirms gz-sensors actually consumed it.
        self.check(self.scan.header.frame_id == 'lidar_link',
                   'scan frame_id is lidar_link',
                   f'got {self.scan.header.frame_id!r}')

        # 2. The scan plane actually intersects the buoys.
        hits = [r for r in self.scan.ranges
                if math.isfinite(r) and self.scan.range_min < r < self.scan.range_max]
        self.check(len(hits) > 0, 'lidar returns hits from the course',
                   f'{len(hits)}/{len(self.scan.ranges)} rays, '
                   f'nearest {min(hits):.2f} m' if hits else 'no returns')

        # 3. Odometry frames.
        self.check(self.odom.header.frame_id == 'odom'
                   and self.odom.child_frame_id == 'base_link',
                   'odom frames are odom -> base_link',
                   f'{self.odom.header.frame_id} -> {self.odom.child_frame_id}')

        # 4. Surge tracks a reachable command.
        self.stop()
        r = self.drive(vx=1.0, seconds=10.0)
        self.check(abs(r['vx'] - 1.0) < 0.12 and r['dx'] > 5.0,
                   'surge tracks vx=1.0',
                   f"vx={r['vx']:.2f} dx={r['dx']:.2f} m")

        # 5. Sway sign. REP-103: +y is PORT. Getting this backwards looks
        #    exactly like a badly tuned controller for a long time.
        self.stop()
        r = self.drive(vy=0.3, seconds=10.0)
        self.check(r['dy'] > 1.0 and abs(r['dx']) < 0.5,
                   '+vy moves the boat to port (+y)',
                   f"dy={r['dy']:.2f} m dx={r['dx']:.2f} m")

        # 6. Yaw sign: positive wz is counter-clockwise.
        self.stop()
        r = self.drive(wz=0.3, seconds=8.0)
        self.check(r['dyaw'] > 0.5,
                   '+wz yaws counter-clockwise',
                   f"dyaw={math.degrees(r['dyaw']):.1f} deg wz={r['wz']:.2f}")

        # 7. Thrust saturation binds. Asking for 5 m/s must not produce 5 m/s;
        #    it must converge on the envelope the thrusters can sustain.
        self.stop()
        r = self.drive(vx=5.0, seconds=20.0)
        self.check(1.3 < r['vx'] < 1.9,
                   'unreachable command saturates near 1.63 m/s',
                   f"vx={r['vx']:.2f}")

        # 8. Command timeout stops the boat. A stalled controller must not
        #    leave it driving.
        self.spin_for(4.0)          # publish nothing at all
        self.check(abs(self.odom.twist.twist.linear.x) < 0.1,
                   'boat stops when /cmd_vel goes quiet',
                   f'vx={self.odom.twist.twist.linear.x:.3f}')

        return self.report()

    def report(self) -> int:
        width = max(len(name) for _, name, _ in self.results)
        for ok, name, detail in self.results:
            print(f"[{'ok  ' if ok else 'FAIL'}] {name.ljust(width)}  {detail}")
        failed = sum(1 for ok, _, _ in self.results if not ok)
        print(f'\n{len(self.results) - failed} passed, {failed} failed')
        return 1 if failed else 0


def main() -> int:
    rclpy.init()
    node = SmokeTest()
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
