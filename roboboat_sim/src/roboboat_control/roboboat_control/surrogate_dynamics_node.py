#!/usr/bin/env python3
"""Surrogate boat dynamics: the only thing standing between Nav2 and Gazebo.

    /cmd_vel  (Nav2)                    -- desired body twist
        -> velocity controller          -- desired wrench
        -> thrust allocation            -- 4 thruster forces, saturated
        -> first-order thruster lag     -- motors cannot step
        -> achievable wrench            -- what the hull really feels
        -> 3-DOF integration            -- surge/sway/yaw with drag + Coriolis
    /boat/cmd_vel_applied  (Gazebo)     -- resulting body twist

Gazebo's VelocityControl system consumes the final twist and rigidly moves the
model, so the simulator never solves buoyancy or contact. Everything Nav2 can
observe about the vehicle's limits is produced here, in ~200 lines of Python
that port unchanged to the Unity sim.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from roboboat_control.allocation import ThrustAllocator, build_layout
from roboboat_control.dynamics import (
    PlanarVessel,
    ThrusterLag,
    VelocityController,
    VesselParams,
)


class SurrogateDynamicsNode(Node):

    def __init__(self) -> None:
        super().__init__('surrogate_dynamics')

        self._declare_parameters()

        self.mode = self.get_parameter('mode').value
        if self.mode not in ('dynamic', 'kinematic'):
            self.get_logger().warn(
                f"unknown mode '{self.mode}', falling back to 'dynamic'")
            self.mode = 'dynamic'

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        rate = float(self.get_parameter('update_rate').value)

        self.params = VesselParams(
            mass_surge=float(self.get_parameter('vessel.mass_surge').value),
            mass_sway=float(self.get_parameter('vessel.mass_sway').value),
            inertia_yaw=float(self.get_parameter('vessel.inertia_yaw').value),
            lin_surge=float(self.get_parameter('vessel.lin_surge').value),
            lin_sway=float(self.get_parameter('vessel.lin_sway').value),
            lin_yaw=float(self.get_parameter('vessel.lin_yaw').value),
            quad_surge=float(self.get_parameter('vessel.quad_surge').value),
            quad_sway=float(self.get_parameter('vessel.quad_sway').value),
            quad_yaw=float(self.get_parameter('vessel.quad_yaw').value),
        )

        self.allocator = ThrustAllocator(build_layout(
            str(self.get_parameter('thrusters.layout').value),
            x=float(self.get_parameter('thrusters.x').value),
            y=float(self.get_parameter('thrusters.y').value),
            max_forward=float(self.get_parameter('thrusters.max_forward').value),
            max_reverse=float(self.get_parameter('thrusters.max_reverse').value),
            angle=math.radians(float(self.get_parameter('thrusters.angle_deg').value)),
            sense=float(self.get_parameter('thrusters.sense').value),
        ))
        self.lag = ThrusterLag(
            count=len(self.allocator.names),
            time_constant=float(self.get_parameter('thrusters.time_constant').value),
            max_rate=float(self.get_parameter('thrusters.max_rate').value),
        )
        self.controller = VelocityController(self.params, gains=(
            float(self.get_parameter('controller.gain_surge').value),
            float(self.get_parameter('controller.gain_sway').value),
            float(self.get_parameter('controller.gain_yaw').value),
        ))
        self.vessel = PlanarVessel(self.params)

        self.kinematic_accel = np.array([
            float(self.get_parameter('kinematic.accel_surge').value),
            float(self.get_parameter('kinematic.accel_sway').value),
            float(self.get_parameter('kinematic.accel_yaw').value),
        ])

        self.cmd = np.zeros(3)
        self.last_cmd_time = None
        self.last_step_time = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.applied_pub = self.create_publisher(Twist, 'boat/cmd_vel_applied', qos)
        self.thrust_pub = self.create_publisher(Float64MultiArray, 'thrusters/thrust', qos)
        self.command_pub = self.create_publisher(Float64MultiArray, 'thrusters/command', qos)
        self.velocity_pub = self.create_publisher(TwistStamped, 'boat/velocity', qos)

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        if bool(self.get_parameter('use_stamped_cmd_vel').value):
            self.create_subscription(TwistStamped, cmd_topic, self._on_cmd_stamped, qos)
        else:
            self.create_subscription(Twist, cmd_topic, self._on_cmd, qos)

        self.timer = self.create_timer(1.0 / rate, self._step)
        self._log_envelope()

    # ------------------------------------------------------------------ setup
    def _declare_parameters(self) -> None:
        defaults = {
            'mode': 'dynamic',
            'update_rate': 100.0,
            'cmd_vel_topic': 'cmd_vel',
            'use_stamped_cmd_vel': False,
            'cmd_timeout': 0.5,

            'thrusters.layout': 'pinwheel',
            'thrusters.sense': 1.0,
            'thrusters.x': 0.55,
            'thrusters.y': 0.40,
            'thrusters.angle_deg': 45.0,
            'thrusters.max_forward': 35.0,
            'thrusters.max_reverse': 25.0,
            'thrusters.time_constant': 0.15,
            'thrusters.max_rate': 400.0,

            'vessel.mass_surge': 38.5,
            'vessel.mass_sway': 63.0,
            'vessel.inertia_yaw': 12.0,
            'vessel.lin_surge': 20.0,
            'vessel.lin_sway': 60.0,
            'vessel.lin_yaw': 8.0,
            'vessel.quad_surge': 25.0,
            'vessel.quad_sway': 90.0,
            'vessel.quad_yaw': 10.0,

            'controller.gain_surge': 2.5,
            'controller.gain_sway': 2.5,
            'controller.gain_yaw': 3.0,

            'kinematic.accel_surge': 1.2,
            'kinematic.accel_sway': 0.8,
            'kinematic.accel_yaw': 1.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _log_envelope(self) -> None:
        """Report the achievable envelope so Nav2 limits can be set honestly."""
        wrench = self.allocator.max_wrench()
        peak = np.array([wrench['fx'], wrench['fy'], wrench['mz']])
        terminal = self.params.terminal_velocity(peak)
        self.get_logger().info(
            f'mode={self.mode} | peak wrench Fx={peak[0]:.1f} N '
            f'Fy={peak[1]:.1f} N Mz={peak[2]:.2f} N.m')
        self.get_logger().info(
            f'steady-state envelope: vx={terminal[0]:.2f} m/s '
            f'vy={terminal[1]:.2f} m/s wz={terminal[2]:.2f} rad/s '
            '(set Nav2 vx_max/vy_max/wz_max at or below these)')

    # --------------------------------------------------------------- callbacks
    def _on_cmd(self, msg: Twist) -> None:
        self._store_cmd(msg)

    def _on_cmd_stamped(self, msg: TwistStamped) -> None:
        self._store_cmd(msg.twist)

    def _store_cmd(self, twist: Twist) -> None:
        self.cmd = np.array([twist.linear.x, twist.linear.y, twist.angular.z])
        self.last_cmd_time = self.get_clock().now()

    # -------------------------------------------------------------------- loop
    def _step(self) -> None:
        now = self.get_clock().now()
        if self.last_step_time is None:
            self.last_step_time = now
            return
        dt = (now - self.last_step_time).nanoseconds * 1e-9
        self.last_step_time = now
        if dt <= 0.0:
            # Sim clock paused or stepped backwards; hold state.
            return

        cmd = self._active_command(now)
        if self.mode == 'kinematic':
            nu = self._step_kinematic(cmd, dt)
            forces = np.zeros(len(self.allocator.names))
        else:
            nu, forces = self._step_dynamic(cmd, dt)

        self._publish(nu, forces, now)

    def _active_command(self, now) -> np.ndarray:
        """Zero the command if Nav2 has gone quiet -- a stalled controller
        must not leave the boat driving."""
        if self.last_cmd_time is None:
            return np.zeros(3)
        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            return np.zeros(3)
        return self.cmd

    def _step_kinematic(self, cmd: np.ndarray, dt: float) -> np.ndarray:
        """Rate-limited passthrough. Fastest possible; no thruster model."""
        delta = np.clip(cmd - self.vessel.nu,
                        -self.kinematic_accel * dt,
                        self.kinematic_accel * dt)
        self.vessel.nu = self.vessel.nu + delta
        return self.vessel.nu.copy()

    def _step_dynamic(self, cmd: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        tau_desired = self.controller.wrench(cmd, self.vessel.nu)
        target_forces, _ = self.allocator.allocate(tau_desired)
        forces = self.lag.step(target_forces, dt)
        # Re-clip after the lag filter: the filtered state is a blend of past
        # targets and can only shrink toward the limits, but be explicit.
        forces = np.clip(forces, self.allocator.lower, self.allocator.upper)
        tau = self.allocator.matrix @ forces
        nu = self.vessel.step(tau, dt)
        return nu, forces

    # ---------------------------------------------------------------- publish
    def _publish(self, nu: np.ndarray, forces: np.ndarray, stamp) -> None:
        twist = Twist()
        twist.linear.x = float(nu[0])
        twist.linear.y = float(nu[1])
        twist.angular.z = float(nu[2])
        self.applied_pub.publish(twist)

        stamped = TwistStamped()
        stamped.header.stamp = stamp.to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = twist
        self.velocity_pub.publish(stamped)

        self.thrust_pub.publish(self._array(forces))
        self.command_pub.publish(self._array(self.allocator.normalize(forces)))

    def _array(self, values: np.ndarray) -> Float64MultiArray:
        msg = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label = ','.join(self.allocator.names)
        dim.size = len(values)
        dim.stride = len(values)
        msg.layout.dim.append(dim)
        msg.data = [float(v) for v in values]
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SurrogateDynamicsNode()
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
