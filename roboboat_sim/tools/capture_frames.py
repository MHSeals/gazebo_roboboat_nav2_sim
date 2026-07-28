#!/usr/bin/env python3
"""Save frames from the world's camera sensors while the sim runs.

These are Gazebo's own rendered pixels -- the ogre2 output of the camera
sensors defined in the course file -- not a redrawing of telemetry. Use it
when you want to see the simulation as Gazebo draws it rather than as the
replay page interprets it.

    ros2 launch roboboat_bringup boat_nav.launch.py headless:=true \\
        world:=<...>/roboboat_course_cameras.sdf &
    python3 tools/capture_frames.py --out frames/ --cameras overhead hero

Encode afterwards with ffmpeg, e.g.

    ffmpeg -framerate 12 -i frames/hero_%04d.png -pix_fmt yuv420p hero.mp4
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

try:
    from PIL import Image as PILImage
except ImportError:                                       # pragma: no cover
    sys.exit('pillow is required: pip install pillow')


class FrameCapture(Node):

    def __init__(self, cameras: list[str], out: Path, rate: float,
                 limit: int) -> None:
        super().__init__('frame_capture', parameter_overrides=[])
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.out = out
        self.min_period = 1.0 / rate if rate > 0 else 0.0
        self.limit = limit
        self.counts = {name: 0 for name in cameras}
        self.last = {name: 0.0 for name in cameras}
        out.mkdir(parents=True, exist_ok=True)

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        for name in cameras:
            self.create_subscription(
                Image, f'/camera/{name}',
                lambda msg, n=name: self._on_image(msg, n), qos)
        self.get_logger().info(f'capturing {cameras} into {out}')

    def _on_image(self, msg: Image, name: str) -> None:
        now = time.monotonic()
        if now - self.last[name] < self.min_period:
            return
        if self.counts[name] >= self.limit:
            return
        self.last[name] = now

        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'unhandled encoding {msg.encoding}',
                                   throttle_duration_sec=10.0)
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        frame = frame.reshape(msg.height, msg.step // 3, 3)[:, :msg.width, :]
        if msg.encoding == 'bgr8':
            frame = frame[:, :, ::-1]

        index = self.counts[name]
        PILImage.fromarray(frame).save(self.out / f'{name}_{index:04d}.png')
        self.counts[name] = index + 1
        if index and index % 25 == 0:
            self.get_logger().info(f'{name}: {index} frames')

    @property
    def done(self) -> bool:
        return all(count >= self.limit for count in self.counts.values())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path('frames'))
    parser.add_argument('--cameras', nargs='+', default=['overhead', 'hero'])
    parser.add_argument('--rate', type=float, default=6.0,
                        help='frames per second to keep, per camera')
    parser.add_argument('--limit', type=int, default=400,
                        help='max frames per camera')
    parser.add_argument('--seconds', type=float, default=240.0,
                        help='wall-clock capture window')
    args = parser.parse_args()

    rclpy.init()
    node = FrameCapture(args.cameras, args.out, args.rate, args.limit)
    end = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for name, count in node.counts.items():
            print(f'{name}: {count} frames -> {args.out}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
