"""Thrust allocation for a fixed-thruster omnidirectional surface vessel.

Pure math -- no ROS imports -- so it can be unit tested and reused verbatim
by whatever drives the 1:1 Unity sim later.

Body frame convention (REP-103): +x forward (surge), +y port (sway),
+z up, yaw positive counter-clockwise.

Each thruster ``i`` sits at ``(x_i, y_i)`` and pushes along the unit vector
``(cos a_i, sin a_i)``. A scalar thrust ``f_i`` therefore contributes::

    Fx  = f_i * cos(a_i)
    Fy  = f_i * sin(a_i)
    Mz  = f_i * (x_i * sin(a_i) - y_i * cos(a_i))

Stacking those columns gives the 3xN allocation matrix ``A`` with
``tau = A @ f``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class Thruster:
    """One fixed-orientation thruster."""

    name: str
    x: float
    y: float
    angle: float          # rad, thrust-axis heading in the body frame
    max_forward: float    # N, positive command limit
    max_reverse: float    # N, magnitude of the negative command limit

    def column(self) -> np.ndarray:
        ca, sa = np.cos(self.angle), np.sin(self.angle)
        return np.array([ca, sa, self.x * sa - self.y * ca], dtype=float)


@dataclass
class ThrustAllocator:
    """Least-norm allocation with saturation-aware uniform back-off.

    ``allocate`` solves ``A f = tau`` in the least-squares/least-norm sense,
    then, if any thruster exceeds its limit, scales the whole solution down
    until it fits. Uniform scaling is used on purpose: it preserves the
    *direction* of the commanded wrench, which matters far more to a
    trajectory tracker than squeezing out the last newton of authority.
    """

    thrusters: list[Thruster]
    _matrix: np.ndarray = field(init=False, repr=False)
    _pinv: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if len(self.thrusters) < 3:
            raise ValueError('at least 3 thrusters are needed for 3-DOF control')
        self._matrix = np.column_stack([t.column() for t in self.thrusters])
        rank = np.linalg.matrix_rank(self._matrix)
        if rank < 3:
            raise ValueError(
                f'thruster layout is not 3-DOF controllable (rank {rank} < 3); '
                'check positions and angles')
        self._pinv = np.linalg.pinv(self._matrix)

    # -- introspection -----------------------------------------------------
    @property
    def matrix(self) -> np.ndarray:
        """The 3xN allocation matrix ``A`` where ``tau = A @ f``."""
        return self._matrix.copy()

    @property
    def names(self) -> list[str]:
        return [t.name for t in self.thrusters]

    @property
    def upper(self) -> np.ndarray:
        return np.array([t.max_forward for t in self.thrusters], dtype=float)

    @property
    def lower(self) -> np.ndarray:
        return np.array([-t.max_reverse for t in self.thrusters], dtype=float)

    # -- core --------------------------------------------------------------
    def allocate(self, tau: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Map a desired wrench to per-thruster forces.

        Args:
            tau: desired ``[Fx, Fy, Mz]`` in the body frame (N, N, N*m).

        Returns:
            ``(f, tau_achieved)`` -- the saturated per-thruster forces and the
            wrench they actually produce. ``tau_achieved`` is what the caller
            must integrate; using ``tau`` would silently pretend the boat has
            infinite thrust, which is exactly the modelling error that makes a
            simulator-tuned controller fail on the water.
        """
        tau = np.asarray(tau, dtype=float).reshape(3)
        f = self._pinv @ tau

        scale = self._saturation_scale(f)
        if scale < 1.0:
            f = f * scale
        # Clip guards against floating-point overshoot after scaling.
        f = np.clip(f, self.lower, self.upper)
        return f, self._matrix @ f

    def _saturation_scale(self, f: np.ndarray) -> float:
        """Largest ``s`` in (0, 1] with ``lower <= s*f <= upper`` elementwise."""
        scale = 1.0
        for value, lo, hi in zip(f, self.lower, self.upper):
            if value > hi > 0.0:
                scale = min(scale, hi / value)
            elif value < lo < 0.0:
                scale = min(scale, lo / value)
        return max(scale, 0.0)

    def normalize(self, f: np.ndarray) -> np.ndarray:
        """Express forces as per-thruster commands in ``[-1, 1]``.

        This is the signal a real ESC/PWM layer consumes, so publishing it
        keeps the sim's interface identical to the boat's.
        """
        f = np.asarray(f, dtype=float)
        limit = np.where(f >= 0.0, self.upper, np.abs(self.lower))
        with np.errstate(divide='ignore', invalid='ignore'):
            out = np.where(limit > 0.0, f / limit, 0.0)
        return np.clip(out, -1.0, 1.0)

    def max_wrench(self) -> dict[str, float]:
        """Peak steady-state authority along each axis, for sanity checks.

        Useful when picking Nav2's ``vx_max``/``vy_max``/``wz_max``: the
        velocity limits you hand MPPI should be reachable by *this* layout,
        not aspirational.
        """
        axes = {}
        for idx, key in enumerate(('fx', 'fy', 'mz')):
            unit = np.zeros(3)
            unit[idx] = 1.0
            # Push hard along the axis and read back what survives saturation.
            _, achieved = self.allocate(unit * 1.0e6)
            axes[key] = float(achieved[idx])
        return axes


def x_configuration(
    x: float,
    y: float,
    max_forward: float,
    max_reverse: float,
    angle: float = np.pi / 4.0,
) -> list[Thruster]:
    """The standard 4-thruster X layout for a catamaran.

    Front-left and rear-right point at ``+angle``; front-right and rear-left
    point at ``-angle``. That yields cleanly decoupled modes:

    * surge -- all four positive,
    * sway  -- FL/RR positive, FR/RL negative,
    * yaw   -- FL/RL positive, FR/RR negative,

    with a one-dimensional null space (1, 1, -1, -1) that the pseudo-inverse
    zeroes out.
    """
    return [
        Thruster('fl', x, y, angle, max_forward, max_reverse),
        Thruster('fr', x, -y, -angle, max_forward, max_reverse),
        Thruster('rl', -x, y, -angle, max_forward, max_reverse),
        Thruster('rr', -x, -y, angle, max_forward, max_reverse),
    ]


def pinwheel_configuration(
    x: float,
    y: float,
    max_forward: float,
    max_reverse: float,
    angle: float = np.pi / 4.0,
    sense: float = 1.0,
) -> list[Thruster]:
    """Rotationally symmetric 45-degree layout -- the owner's sketch.

    Each thruster points along the tangent to the circle through all four
    mounts, so every yaw arm carries the **same sign** and the four moments
    add instead of cancelling::

        fl (+x, +y) @ +135    fr (+x, -y) @  +45
        rl (-x, +y) @ -135    rr (-x, -y) @  -45

    Contrast ``x_configuration``, whose arms come out ``+, -, +, -``: all four
    thrusters at full forward there produce *exactly zero* yaw, and its quoted
    yaw authority is only reachable by running half the thrusters in reverse.
    Here all four forward is the maximum-yaw command, which is what lets the
    stronger forward limit do the work.

    ``sense`` is the chirality: ``+1`` makes all-forward spin the boat
    counter-clockwise (positive yaw under REP-103), ``-1`` reverses it. The
    owner's sketches draw a two-headed arrow through every motor, so they fix
    the thrust *axis* and not the sign -- a T200 is reversible, so chirality is
    a convention here rather than a mounting fact. What matters, and what the X
    layout gets wrong, is that all four agree.
    """
    # Reversing chirality reverses each thrust *vector* -- a half-turn of every
    # axis. Negating the angles instead would mirror the layout about the
    # centreline, which is a different (and wrong) thing: it collapses the arms
    # back to the X layout's 0.106 m.
    flip = 0.0 if sense >= 0.0 else np.pi
    return [
        Thruster('fl', x, y, np.pi - angle + flip, max_forward, max_reverse),
        Thruster('fr', x, -y, angle + flip, max_forward, max_reverse),
        Thruster('rl', -x, y, angle - np.pi + flip, max_forward, max_reverse),
        Thruster('rr', -x, -y, -angle + flip, max_forward, max_reverse),
    ]


def tangential_configuration(
    x: float,
    y: float,
    max_forward: float,
    max_reverse: float,
    angle: float = np.pi / 4.0,
) -> list[Thruster]:
    """45-degree quad with the motors canted TANGENTIALLY::

        fl (+x, +y) @ -45     fr (+x, -y) @ +45
        rl (-x, +y) @ +45     rr (-x, -y) @ -45

    Every motor's forward thrust drives the boat **forward**, so all four
    forward is pure surge and gets the full 35 N limit from each -- exactly as
    in ``x_configuration``. Yaw still needs a mix of forward and reverse,
    because the arms alternate in sign. What changes is how big those arms are.

    The X layout points each motor nearly *radially*: its thrust line makes
    only 9 degrees with the vector from the CoG, so ``x sin a`` and ``y cos a``
    very nearly cancel and the arm collapses to 0.106 m out of a 0.680 m
    radius. Cant the same motors the other way and the thrust line is 81
    degrees off the radius -- essentially tangential -- so the arm is 0.672 m,
    **99% of the radius, and 6.3x the yaw moment for no loss of surge or sway
    whatsoever**.

    Contrast ``pinwheel_configuration``, which is tangential too but orients
    every motor the same way round, making all four forward a pure *yaw*
    command. That buys more yaw still, but it costs 28.6% of the surge because
    surge then has to reverse two motors. This layout is the one that gives up
    nothing.
    """
    return [
        Thruster('fl', x, y, -angle, max_forward, max_reverse),
        Thruster('fr', x, -y, angle, max_forward, max_reverse),
        Thruster('rl', -x, y, angle, max_forward, max_reverse),
        Thruster('rr', -x, -y, -angle, max_forward, max_reverse),
    ]


#: Layouts selectable by name from ``boat_params.yaml``.
LAYOUTS = {
    'tangential': tangential_configuration,
    'x': x_configuration,
    'pinwheel': pinwheel_configuration,
}


def build_layout(name: str, **kwargs) -> list[Thruster]:
    """Construct a named layout, so the config file picks the geometry.

    Keeping both layouts constructible is what makes an A/B possible; the
    alternative -- editing angles in place -- loses the ability to re-measure
    the old one.
    """
    try:
        factory = LAYOUTS[name]
    except KeyError:
        raise ValueError(
            f'unknown thruster layout {name!r}; '
            f'expected one of {sorted(LAYOUTS)}') from None
    if factory in (x_configuration, tangential_configuration):
        # chirality is only meaningful for the pinwheel
        kwargs.pop('sense', None)
    return factory(**kwargs)
