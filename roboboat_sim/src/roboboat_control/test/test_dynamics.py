"""Dynamics tests -- runnable without ROS installed."""

import numpy as np
import pytest

from roboboat_control.allocation import ThrustAllocator, x_configuration
from roboboat_control.dynamics import (
    PlanarVessel,
    ThrusterLag,
    VelocityController,
    VesselParams,
)


def test_drag_free_boat_holds_velocity():
    params = VesselParams(lin_surge=0.0, lin_sway=0.0, lin_yaw=0.0,
                          quad_surge=0.0, quad_sway=0.0, quad_yaw=0.0)
    vessel = PlanarVessel(params)
    vessel.nu = np.array([1.0, 0.0, 0.0])
    for _ in range(200):
        vessel.step(np.zeros(3), 0.01)
    assert vessel.nu[0] == pytest.approx(1.0, abs=1e-9)


def test_drag_brings_boat_to_rest():
    vessel = PlanarVessel()
    vessel.nu = np.array([1.5, 0.5, 0.4])
    for _ in range(2000):
        vessel.step(np.zeros(3), 0.01)
    assert np.allclose(vessel.nu, 0.0, atol=1e-3)


def test_terminal_velocity_matches_integration():
    params = VesselParams()
    vessel = PlanarVessel(params)
    tau = np.array([80.0, 0.0, 0.0])
    for _ in range(6000):
        vessel.step(tau, 0.005)
    predicted = params.terminal_velocity(tau)
    assert vessel.nu[0] == pytest.approx(predicted[0], rel=1e-3)


def test_sway_is_harder_than_surge():
    """A catamaran must resist sideways motion more than forward motion."""
    params = VesselParams()
    terminal = params.terminal_velocity(np.array([60.0, 60.0, 0.0]))
    assert terminal[1] < terminal[0]


def test_coriolis_is_power_neutral():
    """Coriolis terms redistribute momentum; they must not inject energy."""
    params = VesselParams()
    nu = np.array([1.2, -0.4, 0.6])
    power = float(nu @ params.coriolis(nu))
    assert power == pytest.approx(0.0, abs=1e-9)


def test_integrator_stable_at_coarse_step():
    """The sim runs the dynamics at 100 Hz; 20 Hz must not blow up either."""
    vessel = PlanarVessel()
    for _ in range(500):
        vessel.step(np.array([99.0, 0.0, 0.0]), 0.05)
    assert np.all(np.isfinite(vessel.nu))
    assert vessel.nu[0] < 3.0


def test_thruster_lag_converges_and_respects_slew():
    lag = ThrusterLag(4, time_constant=0.15, max_rate=100.0)
    first = lag.step(np.full(4, 35.0), 0.01)
    assert np.all(first <= 100.0 * 0.01 + 1e-9)     # slew limited on tick one
    for _ in range(500):
        state = lag.step(np.full(4, 35.0), 0.01)
    assert state == pytest.approx(np.full(4, 35.0), abs=1e-3)


def test_closed_loop_tracks_commanded_velocity():
    """The full chain must actually reach a reachable setpoint."""
    params = VesselParams()
    vessel = PlanarVessel(params)
    controller = VelocityController(params)
    allocator = ThrustAllocator(x_configuration(0.55, 0.40, 35.0, 25.0))
    lag = ThrusterLag(4)

    target = np.array([1.0, 0.0, 0.0])
    dt = 0.01
    for _ in range(1500):
        tau_desired = controller.wrench(target, vessel.nu)
        forces, _ = allocator.allocate(tau_desired)
        forces = lag.step(forces, dt)
        vessel.step(allocator.matrix @ forces, dt)

    assert vessel.nu[0] == pytest.approx(1.0, abs=0.05)
    assert abs(vessel.nu[1]) < 0.02
    assert abs(vessel.nu[2]) < 0.02


def test_unreachable_command_saturates_instead_of_diverging():
    params = VesselParams()
    vessel = PlanarVessel(params)
    controller = VelocityController(params)
    allocator = ThrustAllocator(x_configuration(0.55, 0.40, 35.0, 25.0))
    lag = ThrusterLag(4)

    target = np.array([10.0, 0.0, 0.0])          # nowhere near achievable
    for _ in range(3000):
        tau = controller.wrench(target, vessel.nu)
        forces, _ = allocator.allocate(tau)
        forces = lag.step(forces, 0.01)
        vessel.step(allocator.matrix @ forces, 0.01)

    peak = allocator.max_wrench()
    ceiling = params.terminal_velocity(np.array([peak['fx'], 0.0, 0.0]))[0]
    assert vessel.nu[0] == pytest.approx(ceiling, rel=0.02)
