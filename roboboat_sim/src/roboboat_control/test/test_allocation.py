"""Allocation tests -- runnable without ROS installed.

    python3 -m pytest src/roboboat_control/test -q
"""

import math

import numpy as np
import pytest

from roboboat_control.allocation import (
    LAYOUTS,
    ThrustAllocator,
    Thruster,
    build_layout,
    pinwheel_configuration,
    tangential_configuration,
    x_configuration,
)


def make() -> ThrustAllocator:
    return ThrustAllocator(x_configuration(x=0.55, y=0.40,
                                           max_forward=35.0, max_reverse=25.0))


def make_pinwheel(sense: float = 1.0) -> ThrustAllocator:
    return ThrustAllocator(pinwheel_configuration(
        x=0.55, y=0.40, max_forward=35.0, max_reverse=25.0, sense=sense))


def make_tangential() -> ThrustAllocator:
    return ThrustAllocator(tangential_configuration(
        x=0.55, y=0.40, max_forward=35.0, max_reverse=25.0))


def yaw_arms(thrusters) -> list[float]:
    return [t.x * math.sin(t.angle) - t.y * math.cos(t.angle) for t in thrusters]


def test_layout_is_three_dof_controllable():
    assert np.linalg.matrix_rank(make().matrix) == 3


def test_rejects_degenerate_layout():
    # Four parallel thrusters on the centreline cannot produce yaw or sway.
    parallel = [Thruster(f't{i}', 0.5 - 0.3 * i, 0.0, 0.0, 30.0, 20.0)
                for i in range(4)]
    with pytest.raises(ValueError, match='3-DOF controllable'):
        ThrustAllocator(parallel)


def test_pure_surge_uses_all_thrusters_equally():
    alloc = make()
    forces, achieved = alloc.allocate([40.0, 0.0, 0.0])
    assert np.allclose(forces, forces[0])
    assert forces[0] > 0.0
    assert achieved == pytest.approx([40.0, 0.0, 0.0], abs=1e-9)


def test_pure_sway_opposes_diagonal_pairs():
    alloc = make()
    forces, achieved = alloc.allocate([0.0, 30.0, 0.0])
    # FL and RR push port; FR and RL push starboard.
    assert forces[0] > 0 and forces[3] > 0
    assert forces[1] < 0 and forces[2] < 0
    assert achieved == pytest.approx([0.0, 30.0, 0.0], abs=1e-9)


def test_pure_yaw_produces_no_net_force():
    alloc = make()
    _, achieved = alloc.allocate([0.0, 0.0, 5.0])
    assert achieved[0] == pytest.approx(0.0, abs=1e-9)
    assert achieved[1] == pytest.approx(0.0, abs=1e-9)
    assert achieved[2] == pytest.approx(5.0, abs=1e-9)


def test_least_norm_solution_avoids_the_null_space():
    """The X layout's null space is (1, 1, -1, -1); the pinv must not use it."""
    alloc = make()
    forces, _ = alloc.allocate([50.0, 10.0, 2.0])
    null = np.array([1.0, 1.0, -1.0, -1.0]) / 2.0
    assert float(forces @ null) == pytest.approx(0.0, abs=1e-9)


def test_saturation_preserves_wrench_direction():
    alloc = make()
    tau = np.array([500.0, 120.0, 30.0])       # far beyond the layout's authority
    forces, achieved = alloc.allocate(tau)
    assert np.all(forces <= alloc.upper + 1e-9)
    assert np.all(forces >= alloc.lower - 1e-9)
    # Scaled back, not distorted.
    cosine = float(achieved @ tau / (np.linalg.norm(achieved) * np.linalg.norm(tau)))
    assert cosine == pytest.approx(1.0, abs=1e-6)
    assert np.linalg.norm(achieved) < np.linalg.norm(tau)


def test_asymmetric_limits_bind_on_reverse():
    alloc = make()
    forces, _ = alloc.allocate([-1.0e4, 0.0, 0.0])
    assert np.allclose(forces, -25.0)          # max_reverse, not max_forward


def test_normalize_maps_limits_to_unit():
    alloc = make()
    assert alloc.normalize(np.array([35.0, -25.0, 0.0, 17.5])) == pytest.approx(
        [1.0, -1.0, 0.0, 0.5])


def test_max_wrench_matches_hand_computation():
    """Uniform back-off means the *tightest* limit sets the whole solution.

    Surge asks all four thrusters to push forward, so the 35 N forward limit
    binds. Sway and yaw are antisymmetric -- two thrusters reverse -- so the
    25 N reverse limit binds and scales the forward pair down with it. That
    costs ~17% of the theoretical sway authority and is the price of never
    distorting the commanded wrench direction under saturation.
    """
    alloc = make()
    peak = alloc.max_wrench()
    c = math.cos(math.pi / 4.0)
    assert peak['fx'] == pytest.approx(4 * 35.0 * c, rel=1e-6)
    assert peak['fy'] == pytest.approx(4 * 25.0 * c, rel=1e-6)
    arm = 0.55 * c - 0.40 * c
    assert peak['mz'] == pytest.approx(arm * 4 * 25.0, rel=1e-6)


def test_zero_wrench_is_zero_thrust():
    forces, achieved = make().allocate([0.0, 0.0, 0.0])
    assert np.allclose(forces, 0.0)
    assert np.allclose(achieved, 0.0)


# --------------------------------------------------------------- pinwheel
#
# The property that would have caught the X layout's defect. Everything else
# here is downstream of it.

def test_pinwheel_yaw_arms_all_share_one_sign():
    arms = yaw_arms(pinwheel_configuration(x=0.55, y=0.40,
                                           max_forward=35.0, max_reverse=25.0))
    assert all(a > 0 for a in arms), arms
    # ...and they are equal, because the layout is rotationally symmetric.
    assert arms == pytest.approx([arms[0]] * 4, rel=1e-12)
    # The full physical radius is sqrt(0.55^2 + 0.40^2) = 0.680 m; a tangential
    # thruster keeps essentially all of it.
    assert arms[0] == pytest.approx(0.6718, abs=5e-4)


def test_x_configuration_yaw_arms_alternate_and_cancel():
    """Documents the defect being fixed, so it cannot come back unnoticed."""
    thr = x_configuration(x=0.55, y=0.40, max_forward=35.0, max_reverse=25.0)
    arms = yaw_arms(thr)
    assert [a > 0 for a in arms] == [True, False, True, False], arms
    assert sum(arms) == pytest.approx(0.0, abs=1e-12)
    # 0.106 m against a 0.680 m physical radius: 84% of the arm thrown away.
    assert abs(arms[0]) == pytest.approx(0.1061, abs=5e-4)


# ------------------------------------------------- mounting sets the yaw arm
#
# The reason the old layout was weak, and the reason the fix is a mounting
# change rather than an angle change.

def test_yaw_arm_is_the_difference_of_the_offsets():
    c = math.cos(math.pi / 4.0)
    for x, y in ((0.55, 0.40), (0.60, 0.25), (0.70, 0.10)):
        arms = yaw_arms(x_configuration(x=x, y=y,
                                        max_forward=35.0, max_reverse=25.0))
        assert abs(arms[0]) == pytest.approx(c * (x - y), rel=1e-12)


def test_surge_and_sway_do_not_depend_on_mounting_position():
    """Why (x - y) is free yaw: only the yaw row of A sees position at all."""
    near = ThrustAllocator(x_configuration(x=0.55, y=0.40,
                                           max_forward=35.0, max_reverse=25.0))
    wide = ThrustAllocator(x_configuration(x=0.60, y=0.25,
                                           max_forward=35.0, max_reverse=25.0))
    assert wide.max_wrench()['fx'] == pytest.approx(near.max_wrench()['fx'], rel=1e-12)
    assert wide.max_wrench()['fy'] == pytest.approx(near.max_wrench()['fy'], rel=1e-12)
    # ...while yaw more than doubles, bought with nothing.
    assert wide.max_wrench()['mz'] / near.max_wrench()['mz'] == pytest.approx(2.33, abs=0.02)


def test_shipped_mounting_keeps_most_of_its_moment_arm():
    """Guards the actual defect: an arm that is small next to the radius.

    boat_params.yaml ships (0.60, 0.25). The check is deliberately expressed
    as a fraction of the physical radius, because that is the quantity the old
    mounting quietly threw away while every other number looked healthy.
    """
    x, y = 0.60, 0.25
    arms = yaw_arms(x_configuration(x=x, y=y, max_forward=35.0, max_reverse=25.0))
    assert abs(arms[0]) / math.hypot(x, y) > 0.25
    assert abs(arms[0]) == pytest.approx(0.2475, abs=5e-4)


def test_shipped_mounting_costs_less_surge_to_turn():
    """The coupling that actually showed up on the course."""
    def surge_while_turning(alloc, mz):
        lo, hi = 0.0, 200.0
        for _ in range(60):
            mid = (lo + hi) / 2.0
            _, achieved = alloc.allocate([mid, 0.0, mz])
            if abs(achieved[0] - mid) < 1e-6 and abs(achieved[2] - mz) < 1e-6:
                lo = mid
            else:
                hi = mid
        return lo
    near = ThrustAllocator(x_configuration(x=0.55, y=0.40,
                                           max_forward=35.0, max_reverse=25.0))
    wide = ThrustAllocator(x_configuration(x=0.60, y=0.25,
                                           max_forward=35.0, max_reverse=25.0))
    assert surge_while_turning(near, 5.0) == pytest.approx(65.7, abs=0.5)
    assert surge_while_turning(wide, 5.0) == pytest.approx(84.7, abs=0.5)


def test_pinwheel_all_forward_is_pure_yaw():
    """The point of the layout: full forward on all four is maximum spin."""
    alloc = make_pinwheel()
    tau = alloc.matrix @ np.array([35.0, 35.0, 35.0, 35.0])
    assert tau[0] == pytest.approx(0.0, abs=1e-9)
    assert tau[1] == pytest.approx(0.0, abs=1e-9)
    assert tau[2] == pytest.approx(94.05, abs=0.01)


def test_x_configuration_all_forward_is_pure_surge_and_no_yaw():
    tau = make().matrix @ np.array([35.0, 35.0, 35.0, 35.0])
    assert tau[2] == pytest.approx(0.0, abs=1e-9)
    assert tau[0] == pytest.approx(98.99, abs=0.01)


def test_pinwheel_trades_surge_for_yaw():
    """8.9x the yaw for 29% of the decoupled surge. Both halves matter."""
    x_peak, pin_peak = make().max_wrench(), make_pinwheel().max_wrench()
    assert pin_peak['mz'] / x_peak['mz'] == pytest.approx(8.86, abs=0.05)
    assert pin_peak['fx'] / x_peak['fx'] == pytest.approx(0.714, abs=0.005)
    assert pin_peak['fy'] == pytest.approx(x_peak['fy'], rel=1e-9)


def test_pinwheel_max_wrench_matches_hand_computation():
    c = math.cos(math.pi / 4.0)
    peak = make_pinwheel().max_wrench()
    # Surge and sway both need two thrusters reversed, so the 25 N limit binds.
    assert peak['fx'] == pytest.approx(4 * 25.0 * c, rel=1e-6)
    assert peak['fy'] == pytest.approx(4 * 25.0 * c, rel=1e-6)
    # Yaw needs none reversed, so the 35 N forward limit binds -- this is the
    # asymmetry the layout exists to exploit.
    assert peak['mz'] == pytest.approx((0.55 + 0.40) * c * 4 * 35.0, rel=1e-6)


def test_pinwheel_chirality_mirrors_the_layout_not_the_arms():
    """Reversing chirality must reverse each thrust vector, not mirror it.

    Negating the angles is the tempting implementation and it is wrong: it
    reflects the layout about the centreline and collapses the yaw arm back to
    the X layout's 0.106 m. The arm magnitude is what catches it.
    """
    ccw, cw = make_pinwheel(sense=1.0), make_pinwheel(sense=-1.0)
    assert all(a > 0 for a in yaw_arms(ccw.thrusters))
    assert all(a < 0 for a in yaw_arms(cw.thrusters))
    assert [abs(a) for a in yaw_arms(cw.thrusters)] == pytest.approx(
        yaw_arms(ccw.thrusters), rel=1e-12)
    assert cw.max_wrench()['fx'] == pytest.approx(ccw.max_wrench()['fx'], rel=1e-9)


def test_pinwheel_yaw_authority_is_asymmetric_between_directions():
    """A consequence of asymmetric T200s that the X layout did not have.

    Spinning the way all four thrusters point forward uses the 35 N limit;
    spinning the other way uses the 25 N reverse limit. Both directions are
    still far better than the X layout's 10.61 N*m, but they are not equal --
    and the sprint is run in whichever direction the beacon calls at run time.
    """
    alloc = make_pinwheel(sense=1.0)
    _, strong = alloc.allocate([0.0, 0.0, +1.0e6])
    _, weak = alloc.allocate([0.0, 0.0, -1.0e6])
    assert strong[2] == pytest.approx(94.045, abs=0.01)
    assert weak[2] == pytest.approx(-67.175, abs=0.01)
    assert abs(weak[2]) / make().max_wrench()['mz'] > 6.0


def test_pinwheel_is_three_dof_controllable():
    assert np.linalg.matrix_rank(make_pinwheel().matrix) == 3


def test_pinwheel_pure_yaw_produces_no_net_force():
    _, achieved = make_pinwheel().allocate([0.0, 0.0, 20.0])
    assert achieved == pytest.approx([0.0, 0.0, 20.0], abs=1e-9)


def test_build_layout_selects_by_name():
    for name in ('x', 'pinwheel'):
        assert name in LAYOUTS
        thr = build_layout(name, x=0.55, y=0.40,
                           max_forward=35.0, max_reverse=25.0)
        assert [t.name for t in thr] == ['fl', 'fr', 'rl', 'rr']
    with pytest.raises(ValueError, match='unknown thruster layout'):
        build_layout('quadcopter', x=0.55, y=0.40,
                     max_forward=35.0, max_reverse=25.0)


# ------------------------------------------------- tangential (the shipped one)
#
# Same "all four forward is surge" property as the X, but the cant is flipped
# so the motors point tangentially instead of nearly radially. That is worth
# 6.3x the yaw for nothing.

def test_tangential_all_forward_is_pure_surge():
    """The defining property: forward thrust on every motor drives it FORWARD."""
    alloc = make_tangential()
    tau = alloc.matrix @ np.array([35.0, 35.0, 35.0, 35.0])
    assert tau[0] == pytest.approx(98.99, abs=0.01)
    assert tau[1] == pytest.approx(0.0, abs=1e-9)
    assert tau[2] == pytest.approx(0.0, abs=1e-9)
    # ...and every motor individually pushes the boat forwards.
    assert all(math.cos(t.angle) > 0 for t in alloc.thrusters)


def test_tangential_yaw_still_needs_forward_and_reverse():
    """Yaw is a mix of + and -, exactly as in the X. That is expected."""
    forces, achieved = make_tangential().allocate([0.0, 0.0, 1.0e6])
    assert sorted(round(f) for f in forces) == [-25, -25, 25, 25]
    assert achieved[2] == pytest.approx(67.175, abs=0.01)


def test_tangential_keeps_the_moment_arm_the_x_throws_away():
    """Same mounts, same 45 degrees, opposite cant -- 6.3x the yaw."""
    x, y = 0.55, 0.40
    radius = math.hypot(x, y)
    tang = yaw_arms(tangential_configuration(x=x, y=y, max_forward=35.0, max_reverse=25.0))
    ex = yaw_arms(x_configuration(x=x, y=y, max_forward=35.0, max_reverse=25.0))
    assert abs(tang[0]) == pytest.approx(0.7071 * (x + y), rel=1e-3)
    assert abs(ex[0]) == pytest.approx(0.7071 * (x - y), rel=1e-3)
    assert abs(tang[0]) / radius > 0.98          # tangential keeps ~all of it
    assert abs(ex[0]) / radius < 0.20            # radial throws ~all of it away
    # Arms still alternate in sign -- this is NOT a pinwheel.
    assert [a > 0 for a in tang] == [False, True, False, True]


def test_tangential_costs_nothing_in_surge_or_sway():
    """The whole point: 6.3x the yaw and the other two axes are untouched."""
    ex, tang = make(), make_tangential()
    a, b = ex.max_wrench(), tang.max_wrench()
    assert b['fx'] == pytest.approx(a['fx'], rel=1e-12)
    assert b['fy'] == pytest.approx(a['fy'], rel=1e-12)
    assert b['mz'] / a['mz'] == pytest.approx(6.33, abs=0.02)


def test_tangential_beats_the_pinwheel_on_surge():
    """The pinwheel buys more yaw but pays 28.6% of surge for it."""
    tang, pin = make_tangential().max_wrench(), make_pinwheel().max_wrench()
    assert tang['fx'] > pin['fx']
    assert tang['fx'] / pin['fx'] == pytest.approx(1.400, abs=0.005)
    assert pin['mz'] > tang['mz']


def test_tangential_is_three_dof_controllable_with_a_clean_null_space():
    alloc = make_tangential()
    assert np.linalg.matrix_rank(alloc.matrix) == 3
    forces, _ = alloc.allocate([40.0, 12.0, 6.0])
    null = np.array([1.0, 1.0, -1.0, -1.0]) / 2.0
    assert float(forces @ null) == pytest.approx(0.0, abs=1e-9)
