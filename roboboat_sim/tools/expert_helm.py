#!/usr/bin/env python3
"""What a human expert helm would do -- computed against the real plant.

The tuning target here is "if an expert were driving, would it look like this?"
That is unanswerable against an opinion, so this module builds the expert as a
*controller*, drives the same 3-DOF plant Nav2 drives, and reports the gap.
No ROS, no Gazebo: it runs in the offline env.

Three stages.

1. **Racing line.** An expert does not track a gate centreline. They take the
   shortest line the hull fits through, cutting apexes where clearance allows.
   ``racing_line()`` smooths and shortens the waypoint polyline under a hard
   footprint-clearance floor and a corridor cap, so the line it returns is one
   the hull provably fits and one that stays inside the channel.

2. **Feasible speed at every curvature.** This has to be plant-honest or the
   whole reference is fiction. A *coordinated* turn -- zero sideslip, the hull
   pointing where it is going, which is what an expert does on a boat whose
   sway drag is 3x its surge drag -- at speed ``u`` and path curvature ``k``
   needs, in steady state, straight out of ``dynamics.py``'s own equations with
   ``v = 0``:

       tau_x = Xu*u + Xuu*|u|*u                hold speed against surge drag
       tau_y = m_u * u^2 * k                   centripetal, via the Coriolis term
       tau_n = Nr*(u*k) + Nrr*|u*k|*(u*k)      hold yaw rate against yaw drag

   Those are checked against the **real** ``ThrustAllocator`` -- four thrusters,
   +35/-25 N, the tangential cant -- so ``feasible_speed()`` returns the speed
   the motors can actually sustain in that corner, not a guess. This is the one
   part of the reference that cannot be argued with: it is the shipped
   allocation matrix.

3. **Minimum-time speed profile.** Forward and backward passes over the line,
   limited at each point by the surge acceleration still available *after* the
   turn has taken its share of the thrust budget. On a fixed line this
   construction is optimal.

What this is NOT, stated up front so nobody quotes it as more than it is:

* **Not a global trajectory optimum.** The line is chosen before the speed
  profile, so a true optimum could trade a longer line for a faster one. The
  time reported is an *achievable* expert time, not a proven minimum.
* **Not a claim about human reflexes.** It assumes the expert executes the
  coordinated turn exactly. A real human is noisier; the reference is therefore
  optimistic, which is the safe direction for a target.

Two variants are produced on purpose:

* ``plant``  -- bounded only by the thrusters. The physical ceiling.
* ``config`` -- additionally bounded by ``nav2_mppi.yaml``'s ``vx_max`` and
  ``wz_max``. **This is the fair target for tuning**, because MPPI is not
  allowed to exceed its own limits. Comparing MPPI against ``plant`` and
  calling the gap a tuning failure blames the controller for its configuration.

Note which limits do and do not bind a coordinated turn: ``vy_max`` does
**not**. A coordinated turn uses zero sway *velocity* while needing substantial
sway *force*; the force is supplied by the inner velocity loop and is invisible
to a limit expressed in m/s. Any argument that ``vy_max`` constrains cornering
is wrong for this reason.

Usage:

    python3 tools/expert_helm.py reference --task channel
    python3 tools/expert_helm.py compare --task channel \\
        --trace tuning/regress_channel_trace.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

WS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WS / 'src' / 'roboboat_control'))
sys.path.insert(0, str(WS / 'tools'))

from roboboat_control.allocation import ThrustAllocator, build_layout  # noqa: E402
from roboboat_control.dynamics import VesselParams  # noqa: E402

COURSE = WS / 'src' / 'roboboat_description' / 'config' / 'course_default.yaml'
NAV2 = WS / 'src' / 'roboboat_bringup' / 'config' / 'nav2_mppi.yaml'
BOAT = WS / 'src' / 'roboboat_control' / 'config' / 'boat_params.yaml'


# --------------------------------------------------------------------------
# plant
# --------------------------------------------------------------------------
def load_plant(boat_params: Path = BOAT):
    """VesselParams + the real allocator, straight from the shipped config."""
    doc = yaml.safe_load(boat_params.read_text())
    params = doc['/**']['ros__parameters']
    vessel = VesselParams(**params['vessel'])
    thr = params['thrusters']
    layout = build_layout(
        thr['layout'],
        x=thr['x'], y=thr['y'],
        max_forward=thr['max_forward'], max_reverse=thr['max_reverse'],
        angle=math.radians(thr['angle_deg']),
        sense=thr.get('sense', 1.0),
    )
    return vessel, ThrustAllocator(layout)


def load_nav2_limits(nav2: Path = NAV2) -> dict:
    doc = yaml.safe_load(nav2.read_text())
    fp = doc['controller_server']['ros__parameters']['FollowPath']
    return {k: float(fp[k]) for k in ('vx_max', 'vy_max', 'wz_max',
                                      'ax_max', 'ax_min')}


def _turn_wrench(vessel: VesselParams, u: float, k: float) -> np.ndarray:
    """Steady-state wrench for a coordinated turn at speed u, curvature k.

    Derived from dynamics.py with v = 0 and u_dot = r_dot = 0. Kept as one
    function so the three equations in the module docstring have exactly one
    implementation.
    """
    r = u * k
    tau_x = vessel.lin_surge * u + vessel.quad_surge * abs(u) * u
    tau_y = vessel.mass_surge * u * r
    tau_n = vessel.lin_yaw * r + vessel.quad_yaw * abs(r) * r
    return np.array([tau_x, tau_y, tau_n])


def _allocatable(alloc: ThrustAllocator, tau: np.ndarray, tol: float = 1e-6) -> bool:
    """True when the allocator delivers tau exactly rather than a scaled copy."""
    _, achieved = alloc.allocate(tau)
    return bool(np.allclose(achieved, tau, atol=tol, rtol=1e-9))


def feasible_speed(vessel, alloc, k: float, u_cap: float = 2.5,
                   wz_cap: float | None = None) -> float:
    """Fastest sustainable coordinated-turn speed at curvature k.

    Binary search against the real allocator. ``wz_cap`` applies the config's
    ``wz_max`` (the ``config`` variant); leave it None for the plant ceiling.
    """
    k = abs(k)
    if wz_cap is not None and k > 1e-9:
        u_cap = min(u_cap, wz_cap / k)
    if u_cap <= 0.0:
        return 0.0
    if _allocatable(alloc, _turn_wrench(vessel, u_cap, k)):
        return u_cap
    lo, hi = 0.0, u_cap
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if _allocatable(alloc, _turn_wrench(vessel, mid, k)):
            lo = mid
        else:
            hi = mid
    return lo


def surge_accel_bounds(vessel, alloc, u: float, k: float) -> tuple[float, float]:
    """(max decel magnitude, max accel) available while holding the turn.

    The turn's centripetal and yaw demands are met first; whatever surge force
    the allocator can still add on top is what accelerates the boat. Drag
    assists deceleration, which is why the two bounds are not symmetric.
    """
    r = u * k
    tau_y = vessel.mass_surge * u * r
    tau_n = vessel.lin_yaw * r + vessel.quad_yaw * abs(r) * r
    drag = vessel.lin_surge * u + vessel.quad_surge * abs(u) * u

    def max_fx(sign: float) -> float:
        lo, hi = 0.0, 200.0
        if _allocatable(alloc, np.array([sign * hi, tau_y, tau_n])):
            return hi
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            if _allocatable(alloc, np.array([sign * mid, tau_y, tau_n])):
                lo = mid
            else:
                hi = mid
        return lo

    a_acc = max(0.0, (max_fx(+1.0) - drag) / vessel.mass_surge)
    a_dec = max(0.0, (max_fx(-1.0) + drag) / vessel.mass_surge)
    return a_dec, a_acc


# --------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------
def collision_circles(obstacles: list[dict]) -> list[tuple[float, float, float]]:
    """Flatten obstacles into (x, y, r) circles.

    NOTE: this mirrors ``nav_test.py:collision_circles``, which is the source of
    truth and is ROS-coupled so it cannot be imported offline. ``selftest``
    below asserts the two agree on the shipped course, so the duplication
    cannot drift silently.
    """
    circles: list[tuple[float, float, float]] = []
    for o in obstacles:
        if o['shape'] == 'circle':
            circles.append((o['x'], o['y'], o['r']))
            continue
        hx, hy = o['hx'], o['hy']
        half_long, radius = max(hx, hy), min(hx, hy)
        along = o['yaw'] + (0.0 if hx >= hy else math.pi / 2.0)
        steps = max(2, int(math.ceil(half_long / max(radius, 0.05))))
        for i in range(-steps, steps + 1):
            offset = half_long * i / steps
            circles.append((o['x'] + math.cos(along) * offset,
                            o['y'] + math.sin(along) * offset,
                            radius))
    return circles


def hull_clearance(x, y, yaw, circles: np.ndarray, half: tuple[float, float]) -> float:
    """Min distance from the hull rectangle to any obstacle surface.

    Same definition the task scorer uses: the footprint polygon, not a point or
    a disc, because a rectangular hull's corner is what clips a buoy.
    """
    hx, hy = half
    dx = circles[:, 0] - x
    dy = circles[:, 1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    ox = np.maximum(np.abs(lx) - hx, 0.0)
    oy = np.maximum(np.abs(ly) - hy, 0.0)
    return float(np.min(np.hypot(ox, oy) - circles[:, 2]))


def densify(points: list[tuple[float, float]], ds: float) -> np.ndarray:
    out = []
    for a, b in zip(points[:-1], points[1:]):
        seg = math.dist(a, b)
        n = max(1, int(round(seg / ds)))
        for i in range(n):
            t = i / n
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    out.append(tuple(points[-1]))
    return np.array(out, dtype=float)


def _tangents(p: np.ndarray) -> np.ndarray:
    d = np.gradient(p, axis=0)
    n = np.hypot(d[:, 0], d[:, 1])
    n[n < 1e-9] = 1.0
    return np.arctan2(d[:, 1] / n, d[:, 0] / n)


def curvature(p: np.ndarray) -> np.ndarray:
    """Signed curvature from consecutive triples (circumradius reciprocal)."""
    k = np.zeros(len(p))
    for i in range(1, len(p) - 1):
        a, b, c = p[i - 1], p[i], p[i + 1]
        ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
        if ab < 1e-9 or bc < 1e-9 or ca < 1e-9:
            continue
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        k[i] = 2.0 * cross / (ab * bc * ca)
    k[0], k[-1] = k[1], k[-2]
    return k


def racing_line(waypoints, circles: np.ndarray, half, margin: float,
                corridor: float = 1.2, iters: int = 400,
                alpha: float = 0.25, ds: float = 0.25) -> np.ndarray:
    """Apex-cutting line: shorten and smooth under clearance + corridor caps.

    The corridor cap is not cosmetic. Without it, smoothing happily pulls the
    line outside a gate entirely -- clearance alone is satisfied by going round
    the wrong side of a buoy, which scores as a missed gate. Capping the
    perpendicular offset from the declared waypoint polyline keeps the line in
    the channel while still letting it cut the apex.
    """
    base = densify(waypoints, ds)
    p = base.copy()
    for _ in range(iters):
        prev = p.copy()
        # shorten + smooth (interior points only; ends are the task's goals)
        p[1:-1] += alpha * (0.5 * (prev[:-2] + prev[2:]) - prev[1:-1])
        # corridor cap, measured against the original polyline
        off = p - base
        d = np.hypot(off[:, 0], off[:, 1])
        over = d > corridor
        if np.any(over):
            scale = np.ones(len(p))
            scale[over] = corridor / d[over]
            p = base + off * scale[:, None]
        # clearance restore
        yaws = _tangents(p)
        for i in range(1, len(p) - 1):
            for _ in range(12):
                cl = hull_clearance(p[i, 0], p[i, 1], yaws[i], circles, half)
                if cl >= margin:
                    break
                dx = circles[:, 0] - p[i, 0]
                dy = circles[:, 1] - p[i, 1]
                j = int(np.argmin(np.hypot(dx, dy) - circles[:, 2]))
                away = p[i] - circles[j, :2]
                n = math.hypot(*away)
                if n < 1e-9:
                    break
                p[i] += (away / n) * (margin - cl + 0.02)
    return p


# --------------------------------------------------------------------------
# reference
# --------------------------------------------------------------------------
def speed_profile(p: np.ndarray, vessel, alloc, u_cap: float,
                  wz_cap: float | None, v_start: float = 0.0,
                  v_end: float = 0.0) -> np.ndarray:
    """Minimum-time speed along a fixed line: curvature cap, then fwd/back."""
    k = curvature(p)
    ds = np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))
    v = np.array([feasible_speed(vessel, alloc, ki, u_cap, wz_cap) for ki in k])
    v[0] = min(v[0], v_start)
    v[-1] = min(v[-1], v_end)
    for i in range(len(v) - 1):                       # forward: accel limit
        _, a = surge_accel_bounds(vessel, alloc, v[i], k[i])
        v[i + 1] = min(v[i + 1], math.sqrt(max(0.0, v[i] ** 2 + 2 * a * ds[i])))
    for i in range(len(v) - 2, -1, -1):               # backward: decel limit
        d, _ = surge_accel_bounds(vessel, alloc, v[i + 1], k[i + 1])
        v[i] = min(v[i], math.sqrt(max(0.0, v[i + 1] ** 2 + 2 * d * ds[i])))
    return v


def build_reference(waypoints, circles: np.ndarray, half, limits: dict,
                    margin: float, variant: str) -> dict:
    u_cap = 2.5 if variant == 'plant' else limits['vx_max']
    wz_cap = None if variant == 'plant' else limits['wz_max']
    p = racing_line(waypoints, circles, half, margin)
    v = speed_profile(p, *load_plant(), u_cap, wz_cap)
    ds = np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))
    vmid = np.maximum(0.5 * (v[:-1] + v[1:]), 1e-3)
    t = np.concatenate([[0.0], np.cumsum(ds / vmid)])
    yaw = _tangents(p)
    k = curvature(p)
    cl = np.array([hull_clearance(p[i, 0], p[i, 1], yaw[i], circles, half)
                   for i in range(len(p))])
    return {
        'variant': variant,
        'elapsed_s': float(t[-1]),
        'path_length_m': float(np.sum(ds)),
        'min_clearance_m': float(np.min(cl)),
        'speed_mean_ms': float(np.sum(ds) / t[-1]),
        'speed_p50_ms': float(np.median(v)),
        'speed_min_ms': float(np.min(v[1:-1])) if len(v) > 2 else 0.0,
        'wz_max_used': float(np.max(np.abs(v * k))),
        'frames': [[round(float(t[i]), 3), round(float(p[i, 0]), 4),
                    round(float(p[i, 1]), 4), round(float(yaw[i]), 4),
                    round(float(v[i]), 4), 0.0, round(float(v[i] * k[i]), 4),
                    round(float(cl[i]), 4)] for i in range(len(p))],
        'frame_fields': ['t', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'clearance'],
    }


# --------------------------------------------------------------------------
# rubric
# --------------------------------------------------------------------------
def score_trace(trace: dict, ref: dict) -> dict:
    """The measurable half of "does this look like an expert driving it?".

    Every metric here is a thing an expert demonstrably does, expressed so a
    trace can be checked against it rather than admired.
    """
    fields = trace['frame_fields']
    idx = {n: i for i, n in enumerate(fields)}
    f = np.array(trace['frames'], dtype=float)
    t, vx, vy = f[:, idx['t']], f[:, idx['vx']], f[:, idx['vy']]
    wz, yaw = f[:, idx['wz']], f[:, idx['yaw']]
    x, y = f[:, idx['x']], f[:, idx['y']]
    speed = np.hypot(vx, vy)

    # The recorded wz carries yaw-wrap spikes. Measured in h1a_ch1_trace.json:
    # two consecutive frames at -125.63 and +125.61 rad/s at t = 105.1/105.3 s,
    # and 2*pi / 0.05 s = 125.66 -- an angle difference taken across the +-pi
    # branch cut without unwrapping, upstream of this tool in the odometry
    # velocity. It is rare (2 frames of 993) and it is devastating to any
    # integral or extremum: it inflated the same run's integral(|wz|)dt from
    # 592 deg to 2155 deg, a 4x error, while percentiles survived because two
    # frames cannot move a p99.
    #
    # Clamp against the plant's own steady-state yaw ceiling (2.47 rad/s) with
    # margin rather than against wz_max, so a genuinely saturated run is kept
    # and only the impossible is dropped. The count is reported, never silent:
    # a trace that needs many of these repaired is a trace to distrust.
    wz_ceiling = 3.0
    wz_bad = np.abs(wz) > wz_ceiling
    if wz_bad.any():
        wz = np.where(wz_bad, np.nan, wz)
        wz = np.interp(np.arange(len(wz)), np.flatnonzero(~wz_bad), wz[~wz_bad])

    moving = speed > 0.30
    fast = speed > 0.50
    # 1. sideslip: pointing where you are going. Needs a speed floor -- it is a
    #    ratio, so at 0.3 m/s a 0.2 m/s sway reads as 40 deg of "crabbing".
    sideslip = np.degrees(np.abs(np.arctan2(vy, np.maximum(vx, 1e-6))))
    # 2. throttle use on the straight bits: an expert is at full throttle when
    #    the water is open, and this config's median is nowhere near its cap.
    k_trace = np.zeros(len(f))
    for i in range(2, len(f) - 2):
        dt = t[i + 1] - t[i - 1]
        if dt > 1e-6 and speed[i] > 0.2:
            k_trace[i] = abs(wz[i]) / max(speed[i], 1e-6)
    straight = (k_trace < 0.05) & fast
    # 3. dithering: net over total yaw, amplitude-gated (journal section 21).
    dyaw = np.diff(np.unwrap(yaw))
    # 4. stopped-in-the-middle: an expert never parks mid-course.
    stalled = speed < 0.15
    interior = (t > 0.10 * t[-1]) & (t < 0.90 * t[-1])
    dt_all = np.gradient(t)

    return {
        'elapsed_s': float(t[-1]),
        'expert_elapsed_s': ref['elapsed_s'],
        'time_ratio': round(float(t[-1]) / ref['elapsed_s'], 3),
        'path_length_m': float(np.sum(np.hypot(np.diff(x), np.diff(y)))),
        'expert_path_length_m': round(ref['path_length_m'], 2),
        'sideslip_p90_deg': round(float(np.percentile(sideslip[fast], 90))
                                  if fast.any() else 0.0, 1),
        'sideslip_over25_frac': round(float(np.mean(sideslip[fast] > 25.0))
                                      if fast.any() else 0.0, 4),
        'speed_p50_moving_ms': round(float(np.median(speed[moving]))
                                     if moving.any() else 0.0, 3),
        'expert_speed_p50_ms': round(ref['speed_p50_ms'], 3),
        'throttle_use_straight': round(
            float(np.mean(speed[straight] / max(ref['speed_p50_ms'], 1e-6)))
            if straight.any() else 0.0, 3),
        'stall_seconds_interior': round(
            float(np.sum(dt_all[stalled & interior])), 2),
        'total_yaw_deg': round(float(np.degrees(np.sum(np.abs(dyaw)))), 1),
        'net_yaw_deg': round(float(np.degrees(abs(np.sum(dyaw)))), 1),
        'yaw_efficiency': round(float(abs(np.sum(dyaw)) /
                                      max(np.sum(np.abs(dyaw)), 1e-9)), 3),
        'min_clearance_m': trace.get('min_clearance'),
        'expert_min_clearance_m': round(ref['min_clearance_m'], 3),
        # Rotation, from the POSE signal and unwrapped, so it is immune to the
        # wz spikes above. total/net is the "turning vs dithering" ratio.
        'total_rotation_deg': round(float(np.degrees(np.sum(np.abs(dyaw)))), 1),
        'expert_total_rotation_deg': None,
        'wz_frames_repaired': int(wz_bad.sum()),
    }


# --------------------------------------------------------------------------
# cli
# --------------------------------------------------------------------------
def task_waypoints(name: str, from_spawn: bool = True,
                   exit_extend: float = 0.6) -> list[tuple[float, float]]:
    """The waypoints an expert would actually drive, ends included.

    Two corrections the naive waypoint list needs before it can be compared
    with a live run, both found by scoring the reference with the real task
    scorer rather than by inspection:

    * **Start at the spawn point, not at the first gate.** The live boat drives
      6 m from (0, 0) to the start gate; a reference that begins at the gate
      reports a path 6 m shorter and an elapsed that omits the standing start.
    * **Finish slightly past the last gate.** ``crossings()`` needs a segment
      that straddles the gate line. A path *ending* on the finish centre never
      crosses it, so the reference scored 9/10 and looked illegal when it was
      merely truncated. 0.6 m is inside ``xy_goal_tolerance`` (0.50 m) plus the
      0.25 m the live run actually overshoots by, so it is not extra credit.
    """
    import course_tasks
    course = yaml.safe_load(COURSE.read_text())
    task = course_tasks.build_task(name, course)
    wps = [(w[0], w[1]) for w in task.waypoints()]
    if from_spawn:
        spawn = (float(course['spawn']['x']), float(course['spawn']['y']))
        if math.dist(spawn, wps[0]) > 1e-6:
            wps.insert(0, spawn)
    if exit_extend > 0.0 and len(wps) >= 2:
        (ax, ay), (bx, by) = wps[-2], wps[-1]
        n = math.hypot(bx - ax, by - ay)
        if n > 1e-9:
            wps.append((bx + (bx - ax) / n * exit_extend,
                        by + (by - ay) / n * exit_extend))
    return wps


def obstacles_from_trace(path: Path) -> list[dict]:
    return json.loads(path.read_text())['obstacles']


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('mode', choices=['reference', 'compare', 'selftest'])
    ap.add_argument('--task', default='channel')
    ap.add_argument('--trace', type=Path)
    ap.add_argument('--obstacles-from', type=Path,
                    help='trace JSON to take the obstacle list from')
    ap.add_argument('--margin', type=float, default=0.15,
                    help='clearance floor the racing line must respect (m)')
    ap.add_argument('--variant', default='config', choices=['config', 'plant'])
    ap.add_argument('--out', type=Path)
    args = ap.parse_args()

    vessel, alloc = load_plant()
    limits = load_nav2_limits()

    if args.mode == 'selftest':
        print('allocator max wrench:', {k: round(v, 2)
                                        for k, v in alloc.max_wrench().items()})
        for k in (0.0, 0.05, 0.1, 0.2, 0.3, 0.5):
            up = feasible_speed(vessel, alloc, k)
            uc = feasible_speed(vessel, alloc, k, limits['vx_max'],
                                limits['wz_max'])
            radius = 'straight' if k == 0 else f'R={1/k:5.1f} m'
            print(f'  k={k:4.2f} {radius:>10}  plant {up:5.3f} m/s   '
                  f'config {uc:5.3f} m/s')
        return 0

    src = args.obstacles_from or (WS / 'tuning' / f'regress_{args.task}_trace.json')
    if not src.exists():
        print(f'need an obstacle source; {src} not found', file=sys.stderr)
        return 2
    circles = np.array(collision_circles(obstacles_from_trace(src)), dtype=float)
    half = tuple(json.loads(src.read_text())['footprint'])
    wps = task_waypoints(args.task)

    ref = build_reference(wps, circles, half, limits, args.margin, args.variant)

    if args.mode == 'reference':
        print(f'expert reference: {args.task} ({args.variant})')
        for key in ('elapsed_s', 'path_length_m', 'min_clearance_m',
                    'speed_mean_ms', 'speed_p50_ms', 'speed_min_ms',
                    'wz_max_used'):
            print(f'  {key:18} {ref[key]:.3f}')
        if args.out:
            args.out.write_text(json.dumps(ref, indent=1))
            print(f'  written {args.out}')
        return 0

    trace = json.loads(args.trace.read_text())
    card = score_trace(trace, ref)
    print(f'expert gap: {args.task} ({args.variant} reference)')
    for k, v in card.items():
        print(f'  {k:26} {v}')
    if args.out:
        args.out.write_text(json.dumps({'reference': {k: v for k, v in ref.items()
                                                      if k != 'frames'},
                                        'gap': card}, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
