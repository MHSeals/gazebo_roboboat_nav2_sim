#!/usr/bin/env python3
"""Block until the simulator is genuinely ready for Nav2 to start.

Nav2's lifecycle transitions are timed against the clock its nodes are using.
Bring the stack up at the same instant as Gazebo and those timeouts race the
sim clock's first tick: the lifecycle manager can declare a node failed
roughly 100 ms after asking it to configure, while the node in question is
still loading plugins perfectly happily, and the whole bringup aborts. The
symptom is every goal returning ABORTED in zero seconds.

A fixed sleep does not fix this reliably, because how long Gazebo needs
depends on rendering backend and machine load -- 60 s or more under software
rendering, a few seconds with a GPU. So gate on the real preconditions
instead:

  * /clock is advancing (the sim is stepping, not merely loaded),
  * odom -> base_link exists (the robot is spawned and publishing),
  * /scan is arriving (the sensor pipeline is alive).

Exit 0 when all three hold, non-zero on timeout.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class SimReady(Node):

    def __init__(self) -> None:
        super().__init__('wait_for_sim', parameter_overrides=[])
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.odom_seen = False
        self.scan_seen = False
        self.first_clock = None
        self.clock_advanced = False

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, sensor_qos)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def _on_odom(self, _msg: Odometry) -> None:
        self.odom_seen = True

    def _on_scan(self, _msg: LaserScan) -> None:
        self.scan_seen = True

    def check_clock(self) -> None:
        now = self.get_clock().now().nanoseconds
        if now <= 0:
            return
        if self.first_clock is None:
            self.first_clock = now
        elif now > self.first_clock:
            self.clock_advanced = True

    def tf_ready(self) -> bool:
        return self.buffer.can_transform('odom', 'base_link',
                                         rclpy.time.Time())

    def status(self) -> str:
        return (f'clock={"ok" if self.clock_advanced else "..."} '
                f'odom={"ok" if self.odom_seen else "..."} '
                f'scan={"ok" if self.scan_seen else "..."} '
                f'tf={"ok" if self.tf_ready() else "..."}')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=240.0)
    parser.add_argument('--quiet', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = SimReady()
    deadline = time.monotonic() + args.timeout
    last_report = 0.0
    ready = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            node.check_clock()
            if (node.clock_advanced and node.odom_seen
                    and node.scan_seen and node.tf_ready()):
                ready = True
                break
            now = time.monotonic()
            if not args.quiet and now - last_report > 15.0:
                last_report = now
                print(f'  waiting for sim: {node.status()}', flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if ready:
        if not args.quiet:
            print('  sim ready')
        return 0
    print(f'TIMEOUT after {args.timeout:.0f}s waiting for the sim', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
