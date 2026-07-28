#!/usr/bin/env python3
"""Offline tests for nav_test.py's metric callbacks. No ROS required.

These exist because `validate.py` only does `ast.parse`, which cannot catch a
use-before-assignment inside a callback. One such bug reached a benchmark run
and cost six minutes of wall clock to discover, at which point the run had
already thrown away its data. Exercising the callback against fake messages
catches that class of mistake in milliseconds.

    python3 tools/test_metrics.py
"""

from __future__ import annotations

import math
import sys
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent


def load_callback():
    """Pull _on_odom out of nav_test.py without importing rclpy."""
    src = (HERE / 'nav_test.py').read_text()
    # Half-beam now lives in course_tasks, which has no ROS dependency, so it
    # can simply be imported rather than sliced out of the source.
    sys.path.insert(0, str(HERE))
    from course_tasks import FOOTPRINT_HALF_Y
    ns = {'math': math, 'FOOTPRINT_HALF_Y': FOOTPRINT_HALF_Y}
    exec(compile(src[src.index('FOOTPRINT_HALF_X'):src.index('def load_obstacles')],
                 'constants', 'exec'), ns)
    body = src[src.index('    def _on_odom'):src.index('    def _on_thrust')]
    body = body.replace(': Odometry', '')      # drop the ROS type annotation
    exec(compile('class Probe:\n' + body, 'callback', 'exec'), ns)
    return ns['Probe']


CLOCK = [0.0]


def make_node(probe):
    node = probe.__new__(probe)
    node.odom = None
    node.circles = [(5.0, 0.0, 0.2)]
    node.encounters = [float('inf')]
    node.min_clearance = float('inf')
    node.closest_at = None
    node.clearances = []
    node.speeds = []
    node.path_length = 0.0
    node._prev_xy = None
    node._path_t = 0.0
    node.odom_samples = 0
    node.odom_first_t = None
    node.odom_last_t = None
    node.record_path = None
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(nanoseconds=CLOCK[0] * 1e9))
    return node


def odom(x, y, vx=0.5):
    ns = types.SimpleNamespace
    return ns(pose=ns(pose=ns(position=ns(x=x, y=y),
                              orientation=ns(x=0.0, y=0.0, z=0.0, w=1.0))),
              twist=ns(twist=ns(linear=ns(x=vx, y=0.0), angular=ns(z=0.0))))


def drive(node, metres, seconds, rate_hz=50.0):
    steps = int(seconds * rate_hz)
    for i in range(steps + 1):
        CLOCK[0] = i / rate_hz
        node._on_odom(odom(metres * min(i / steps, 1.0), 0.0))


def main() -> int:
    probe = load_callback()
    failures = []

    # 1. The callback runs at all. Guards against use-before-assignment.
    fast = make_node(probe)
    drive(fast, 1.0, 2.0)
    if abs(fast.path_length - 1.0) > 0.06:
        failures.append(f'path length {fast.path_length:.3f} m, expected ~1.0')

    # 2. Path length must not grow with duration. Summing every 50 Hz frame
    #    adds error proportional to sample count, which made a slower run
    #    score a longer path with identical geometry -- and made path length
    #    and efficiency non-independent of the time metric.
    CLOCK[0] = 0.0
    slow = make_node(probe)
    drive(slow, 1.0, 4.0)
    if abs(slow.path_length - fast.path_length) > 0.03:
        failures.append(
            f'path length is duration-dependent: {fast.path_length:.3f} m in '
            f'{fast.odom_samples} samples vs {slow.path_length:.3f} m in '
            f'{slow.odom_samples}')

    # 3. Clearance is measured against the obstacle at (5, 0), r=0.2, with the
    #    hull's 0.75 m half-length: closest approach from x=1.0 is
    #    5.0 - 1.0 - 0.75 - 0.2 = 3.05 m.
    if abs(fast.min_clearance - 3.05) > 0.02:
        failures.append(f'min clearance {fast.min_clearance:.3f}, expected 3.05')

    # 4. Per-obstacle encounters must be populated, since they are the primary
    #    comparison samples.
    if not fast.encounters or fast.encounters[0] == float('inf'):
        failures.append('encounters never recorded')

    for line in failures:
        print(f'  FAIL  {line}')
    if failures:
        return 1
    print(f'  metric callbacks ok: path {fast.path_length:.3f} m '
          f'(duration-independent), clearance {fast.min_clearance:.2f} m')
    return 0


if __name__ == '__main__':
    sys.exit(main())
