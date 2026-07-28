"""3-DOF surrogate dynamics for a planar surface vessel.

This is the *entire* physics of the boat. Gazebo integrates nothing: it is
handed a body-frame twist each tick and moves the model rigidly. That is the
central speed trade in this workspace -- no buoyancy solve, no wave field, no
contact, no added-mass tensor plugin, no 200 Hz physics step.

What survives is the part a trajectory controller actually feels:

* different inertia and drag in surge, sway and yaw (a catamaran slides
  sideways far worse than it goes forward);
* rigid-body Coriolis coupling, so turning while moving behaves like a boat
  rather than like a floating turntable;
* first-order thruster lag and hard thrust saturation, so commanded
  accelerations that the motors cannot deliver do not appear for free.

Model (Fossen, planar reduction), body velocities ``nu = [u, v, r]``::

    m_u * u_dot = tau_x + m_v * v * r      - (Xu*u   + Xuu*|u|*u)
    m_v * v_dot = tau_y - m_u * u * r      - (Yv*v   + Yvv*|v|*v)
    Izz * r_dot = tau_n - (m_v - m_u)*u*v  - (Nr*r   + Nrr*|r|*r)

``m_u``, ``m_v`` and ``Izz`` already include hydrodynamic added mass, which is
why they are larger than the hull's dry mass.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class VesselParams:
    """Rigid-body + damping coefficients. All SI."""

    mass_surge: float = 38.5      # kg, dry mass + added mass in x
    mass_sway: float = 63.0       # kg, dry mass + added mass in y
    inertia_yaw: float = 12.0     # kg*m^2, Izz + added inertia

    lin_surge: float = 20.0       # N per (m/s)
    lin_sway: float = 60.0
    lin_yaw: float = 8.0          # N*m per (rad/s)

    quad_surge: float = 25.0      # N per (m/s)^2
    quad_sway: float = 90.0
    quad_yaw: float = 10.0        # N*m per (rad/s)^2

    def mass_vector(self) -> np.ndarray:
        return np.array([self.mass_surge, self.mass_sway, self.inertia_yaw], dtype=float)

    def damping(self, nu: np.ndarray) -> np.ndarray:
        """Damping force/moment opposing ``nu`` (returned as a positive drag)."""
        nu = np.asarray(nu, dtype=float)
        lin = np.array([self.lin_surge, self.lin_sway, self.lin_yaw])
        quad = np.array([self.quad_surge, self.quad_sway, self.quad_yaw])
        return lin * nu + quad * np.abs(nu) * nu

    def coriolis(self, nu: np.ndarray) -> np.ndarray:
        """Rigid-body Coriolis/centripetal terms, as a wrench to be added."""
        u, v, r = (float(x) for x in nu)
        return np.array([
            self.mass_sway * v * r,
            -self.mass_surge * u * r,
            -(self.mass_sway - self.mass_surge) * u * v,
        ])

    def terminal_velocity(self, tau: np.ndarray) -> np.ndarray:
        """Steady-state ``nu`` for a constant wrench, ignoring coupling.

        Solves ``lin*x + quad*|x|*x = tau`` per axis. Handy for choosing
        Nav2's velocity limits from the thruster layout instead of guessing.
        """
        tau = np.asarray(tau, dtype=float)
        lin = np.array([self.lin_surge, self.lin_sway, self.lin_yaw])
        quad = np.array([self.quad_surge, self.quad_sway, self.quad_yaw])
        mag = np.abs(tau)
        # Positive root of quad*x^2 + lin*x - |tau| = 0.
        disc = lin ** 2 + 4.0 * quad * mag
        root = (-lin + np.sqrt(disc)) / (2.0 * quad)
        return np.sign(tau) * root


class ThrusterLag:
    """First-order lag plus slew limiting on commanded thrust.

    A T200-class thruster does not step from 0 to full thrust; pretending it
    does is what produces MPPI tunings that oscillate on the real boat.
    """

    def __init__(self, count: int, time_constant: float = 0.15,
                 max_rate: float = 400.0) -> None:
        self.time_constant = max(float(time_constant), 1e-6)
        self.max_rate = float(max_rate)          # N/s
        self.state = np.zeros(int(count), dtype=float)

    def reset(self) -> None:
        self.state[:] = 0.0

    def step(self, target: np.ndarray, dt: float) -> np.ndarray:
        target = np.asarray(target, dtype=float)
        alpha = 1.0 - np.exp(-dt / self.time_constant)
        delta = (target - self.state) * alpha
        cap = self.max_rate * dt
        self.state = self.state + np.clip(delta, -cap, cap)
        return self.state.copy()


class PlanarVessel:
    """Integrates ``nu`` under an applied wrench."""

    def __init__(self, params: VesselParams | None = None) -> None:
        self.params = params or VesselParams()
        self.nu = np.zeros(3, dtype=float)

    def reset(self) -> None:
        self.nu[:] = 0.0

    def step(self, tau: np.ndarray, dt: float) -> np.ndarray:
        """Advance one tick with semi-implicit (symplectic) Euler.

        Damping is evaluated at the *new* velocity estimate, which keeps the
        integrator stable at the large-ish steps this sim runs at.
        """
        tau = np.asarray(tau, dtype=float).reshape(3)
        p = self.params
        m = p.mass_vector()

        accel = (tau + p.coriolis(self.nu) - p.damping(self.nu)) / m
        predicted = self.nu + accel * dt
        # Corrector pass: re-evaluate the strongly nonlinear damping term at
        # the predicted state.
        accel = (tau + p.coriolis(predicted) - p.damping(predicted)) / m
        self.nu = self.nu + accel * dt
        return self.nu.copy()


class VelocityController:
    """Turns a commanded twist into a desired wrench.

    Feedforward (drag needed to *hold* the commanded velocity) plus a
    proportional term scaled by the mass matrix, so the gains read as
    closed-loop bandwidth in rad/s rather than as arbitrary numbers.
    """

    def __init__(self, params: VesselParams, gains: tuple[float, float, float] = (2.5, 2.5, 3.0)) -> None:
        self.params = params
        self.gains = np.asarray(gains, dtype=float)

    def wrench(self, nu_cmd: np.ndarray, nu: np.ndarray) -> np.ndarray:
        nu_cmd = np.asarray(nu_cmd, dtype=float).reshape(3)
        nu = np.asarray(nu, dtype=float).reshape(3)
        feedforward = self.params.damping(nu_cmd) - self.params.coriolis(nu_cmd)
        feedback = self.params.mass_vector() * self.gains * (nu_cmd - nu)
        return feedforward + feedback
