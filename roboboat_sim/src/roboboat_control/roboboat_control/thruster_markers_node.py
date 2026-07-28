#!/usr/bin/env python3
"""Publish per-thruster force arrows for RViz.

Purely a tuning aid. When MPPI produces jittery motion it is usually obvious
here first: arrows flipping sign every control tick means the velocity
command is chattering, arrows pinned at full length means you asked for a
wrench the layout cannot deliver and the allocator is scaling everything back.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from roboboat_control.allocation import build_layout


class ThrusterMarkersNode(Node):

    def __init__(self) -> None:
        super().__init__('thruster_markers')

        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('thrusters.layout', 'pinwheel')
        self.declare_parameter('thrusters.sense', 1.0)
        self.declare_parameter('thrusters.x', 0.55)
        self.declare_parameter('thrusters.y', 0.40)
        self.declare_parameter('thrusters.angle_deg', 45.0)
        self.declare_parameter('thrusters.max_forward', 35.0)
        self.declare_parameter('thrusters.max_reverse', 25.0)
        self.declare_parameter('thrusters.z', -0.10)
        self.declare_parameter('arrow_scale', 0.02)   # metres of arrow per newton

        self.frame_id = self.get_parameter('frame_id').value
        self.z = float(self.get_parameter('thrusters.z').value)
        self.arrow_scale = float(self.get_parameter('arrow_scale').value)

        self.thrusters = build_layout(
            str(self.get_parameter('thrusters.layout').value),
            x=float(self.get_parameter('thrusters.x').value),
            y=float(self.get_parameter('thrusters.y').value),
            max_forward=float(self.get_parameter('thrusters.max_forward').value),
            max_reverse=float(self.get_parameter('thrusters.max_reverse').value),
            angle=math.radians(float(self.get_parameter('thrusters.angle_deg').value)),
            sense=float(self.get_parameter('thrusters.sense').value),
        )

        self.pub = self.create_publisher(MarkerArray, 'thrusters/markers', 1)
        self.create_subscription(
            Float64MultiArray, 'thrusters/thrust', self._on_thrust, 10)

    def _on_thrust(self, msg: Float64MultiArray) -> None:
        forces = np.asarray(msg.data, dtype=float)
        if forces.size != len(self.thrusters):
            self.get_logger().warn(
                f'expected {len(self.thrusters)} thrust values, got {forces.size}',
                throttle_duration_sec=5.0)
            return

        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for idx, (thruster, force) in enumerate(zip(self.thrusters, forces)):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = 'thrust'
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.04    # shaft diameter
            marker.scale.y = 0.09    # head diameter
            marker.scale.z = 0.10    # head length

            length = force * self.arrow_scale
            ca, sa = math.cos(thruster.angle), math.sin(thruster.angle)
            start = _point(thruster.x, thruster.y, self.z)
            end = _point(thruster.x + ca * length,
                         thruster.y + sa * length,
                         self.z)
            marker.points = [start, end]

            # Green ahead, red astern; saturation is visible as full length.
            if force >= 0.0:
                marker.color.r, marker.color.g, marker.color.b = 0.15, 0.85, 0.25
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.9, 0.2, 0.15
            marker.color.a = 0.9
            array.markers.append(marker)

        self.pub.publish(array)


def _point(x: float, y: float, z: float):
    from geometry_msgs.msg import Point
    point = Point()
    point.x, point.y, point.z = float(x), float(y), float(z)
    return point


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThrusterMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
