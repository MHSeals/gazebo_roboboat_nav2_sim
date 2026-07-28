#!/usr/bin/env python3
"""Offline checks for the task scoring rules. No ROS, no simulator.

A scoring rule that is wrong is worse than no scoring rule, because it reports
a number. These tests drive synthetic tracks -- a correct lap, a lap the wrong
way, a boat that drives to the mark and back without circling, a 270 degree
orbit, a run that winds back and forth to a net full turn -- through the real
scorers and assert the verdicts.

It imports course_tasks rather than task_run: the geometry and the scoring are
deliberately in a module with no ROS dependency, so these run anywhere.

    python3 tools/test_task_scoring.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import course_tasks as task_run  # noqa: E402

FAILURES: list[str] = []


def check(name: str, condition: bool, detail: str = '') -> None:
    if condition:
        print(f'  ok   {name}')
    else:
        print(f'  FAIL {name}  {detail}')
        FAILURES.append(name)


def arc(centre, radius, a0, a1, n=400):
    return [(centre[0] + radius * math.cos(a0 + (a1 - a0) * i / n),
             centre[1] + radius * math.sin(a0 + (a1 - a0) * i / n))
            for i in range(n + 1)]


COURSE = {
    'buoy_types': {
        'red': {'radius': 0.20}, 'green': {'radius': 0.20},
        'yellow': {'radius': 0.30},
    },
    'gates': [
        {'name': 'g1', 'x': 0.0, 'y': 0.0, 'heading_deg': 0.0, 'width': 3.0,
         'right': 'red', 'left': 'green'},
        {'name': 'g2', 'x': 10.0, 'y': 0.0, 'heading_deg': 0.0, 'width': 3.0,
         'right': 'red', 'left': 'green'},
    ],
    'buoys': [{'name': 'mark', 'type': 'yellow', 'x': 20.0, 'y': 0.0}],
    'tasks': [
        {'name': 'chan', 'type': 'gate_transit', 'gates': ['g1', 'g2']},
        {'name': 'sprint', 'type': 'circle_mark', 'gate': 'g1',
         'mark': 'mark', 'direction': 'ccw', 'laps': 1,
         'radius': 3.0, 'points': 8},
    ],
}


def test_signed_turn() -> None:
    print('signed_turn')
    c = (0.0, 0.0)
    ccw, _, _ = task_run.signed_turn(arc(c, 3.0, 0.0, 2 * math.pi), c)
    check('a full CCW lap sweeps +2pi', abs(ccw - 2 * math.pi) < 1e-3,
          f'got {ccw:.4f}')

    cw, _, _ = task_run.signed_turn(arc(c, 3.0, 0.0, -2 * math.pi), c)
    check('a full CW lap sweeps -2pi', abs(cw + 2 * math.pi) < 1e-3,
          f'got {cw:.4f}')

    # Out and back along a radius: ends where it started, sweeps nothing.
    out_back = [(x, 0.0) for x in list(range(1, 20)) + list(range(19, 0, -1))]
    check('out and back sweeps ~0',
          abs(task_run.signed_turn(out_back, c)[0]) < 1e-6)

    # Half a lap out, half a lap back the same way: net zero, and must not be
    # mistaken for a lap by a start-to-end bearing comparison.
    there = arc(c, 3.0, 0.0, math.pi)
    back = list(reversed(there))
    check('half out, half back sweeps ~0',
          abs(task_run.signed_turn(there + back, c)[0]) < 1e-6)

    # Two laps.
    two, _, _ = task_run.signed_turn(arc(c, 3.0, 0.0, 4 * math.pi, n=800), c)
    check('two CCW laps sweep +4pi', abs(two - 4 * math.pi) < 1e-3,
          f'got {two:.4f}')

    # A sample sitting exactly on the centre must not produce a NaN or a jump.
    check('a sample at the centre is skipped',
          math.isfinite(task_run.signed_turn([(0, 0)] + arc(c, 3, 0, math.pi), c)[0]))


def test_ring() -> None:
    print('ring_waypoints')
    ring = task_run.ring_waypoints((0.0, 0.0), 3.0, 8, 0.0, ccw=True)
    # Eight steps around, plus the part-step overshoot.
    check('ring has the requested count', len(ring) == 9)
    check('every ring point is on the circle',
          all(abs(math.hypot(x, y) - 3.0) < 1e-9 for x, y, _ in ring))
    # Chord spacing, which is what RemovePassedGoals sees.
    spacing = math.hypot(ring[1][0] - ring[0][0], ring[1][1] - ring[0][1])
    check('R=3.0, N=8 gives 2.30 m spacing', abs(spacing - 2.2961) < 1e-3,
          f'got {spacing:.4f}')
    check('ring spacing clears RemovePassedGoals radius 1.6', spacing > 1.6)

    ccw_turn = task_run.signed_turn([(x, y) for x, y, _ in ring], (0.0, 0.0),
                                    max_step=math.pi)[0]
    cw = task_run.ring_waypoints((0.0, 0.0), 3.0, 8, 0.0, ccw=False)
    cw_turn = task_run.signed_turn([(x, y) for x, y, _ in cw], (0.0, 0.0),
                                   max_step=math.pi)[0]
    check('ccw ring runs positive', ccw_turn > 0, f'got {ccw_turn:.3f}')
    check('cw ring runs negative', cw_turn < 0, f'got {cw_turn:.3f}')

    # The ring deliberately does NOT close onto the start bearing: see
    # test_ring_overshoot for why a closed ring loses a run.
    closed = task_run.ring_waypoints((0.0, 0.0), 3.0, 8, 0.0, True, overshoot=0.0)
    check('with no overshoot the ring closes exactly',
          math.hypot(closed[-1][0] - 3.0, closed[-1][1]) < 1e-9)


def test_gate_transit() -> None:
    print('gate_transit scoring')
    task = task_run.build_task('chan', COURSE)
    straight = [(x * 0.1 - 2.0, 0.0) for x in range(200)]
    card = task.score(straight, 0.5)
    check('a straight run transits both gates', card['gates_transited'] == 2)
    check('and passes', card['passed'])

    # Around the outside of gate 1: y = 3 m is outside the 1.5 m half-width.
    around = ([(x * 0.1 - 2.0, 3.0) for x in range(60)]
              + [(4.0, 3.0 - y * 0.1) for y in range(30)]
              + [(4.0 + x * 0.1, 0.0) for x in range(100)])
    card = task.score(around, 0.5)
    check('going around gate 1 is a miss', not card['gates'][0]['transited'])
    check('and the task fails', not card['passed'])

    # Correct geometry but touching something.
    card = task.score(straight, -0.01)
    check('contact fails the task', not card['passed'] and card['contact'])

    # Out-of-order: crossing gate 2 before gate 1 must not score gate 2.
    backwards = ([(10.0, -2.0), (10.0, 2.0)]      # cross g2 first
                 + [(0.0, 2.0), (0.0, -2.0)])     # then g1
    card = task.score(backwards, 0.5)
    check('gates must be taken in order', card['gates_transited'] < 2,
          f"got {card['gates_transited']}")


def test_circle_mark() -> None:
    print('circle_mark scoring')
    task = task_run.build_task('sprint', COURSE)
    mark = (20.0, 0.0)

    def run(track):
        return task.score(track, 0.5)

    # A correct run: out through the gate, one CCW lap, back out through it.
    approach = [(-2.0 + x * 0.1, 0.0) for x in range(190)]     # to (17, 0)
    lap = arc(mark, 3.0, math.pi, math.pi + 2 * math.pi)       # CCW from gate side
    ret = [(17.0 - x * 0.1, 0.0) for x in range(190)]          # back to (-2, 0)
    card = run(approach + lap + ret)
    check('a correct CCW run passes', card['passed'],
          f"swept {card['swept_rad']:.2f}, entered {card['entered']}, "
          f"exited {card['exited']}")
    check('and reports one lap', abs(card['laps_completed'] - 1.0) < 0.02)

    # Same run, driven clockwise: must fail, and be called out as such.
    lap_cw = arc(mark, 3.0, math.pi, math.pi - 2 * math.pi)
    card = run(approach + lap_cw + ret)
    check('a CW lap fails a CCW task', not card['passed'])
    check('and is flagged wrong-direction', card['wrong_direction'],
          f"swept {card['swept_rad']:.2f}")

    # Drive to the mark and straight back: crosses every line a correct run
    # crosses, sweeps nothing.
    card = run(approach + ret)
    check('out-and-back without circling fails', not card['passed'])
    check('and reports ~0 laps', card['laps_completed'] < 0.1,
          f"got {card['laps_completed']}")

    # Never came back through the gate.
    card = run(approach + lap)
    check('not returning through the gate fails', not card['passed'])
    check('but the lap is still credited', card['circled'])

    # Circling the wrong thing: a lap about a point well away from the mark.
    decoy = arc((30.0, 10.0), 3.0, 0.0, 2 * math.pi)
    card = run(approach + decoy + ret)
    check('circling somewhere else fails', not card['passed'],
          f"swept {card['swept_rad']:.2f}")

    # Contact anywhere fails even a geometrically perfect run.
    card = task.score(approach + lap + ret, -0.01)
    check('contact fails the sprint', not card['passed'])

    # Half a lap is not a lap.
    half = arc(mark, 3.0, math.pi, 2 * math.pi)
    card = run(approach + half + [(23.0, 0.0), (17.0, 0.0)] + ret)
    check('half a lap fails', not card['passed'],
          f"laps {card['laps_completed']:.2f}")

    # A 270 degree orbit. The approach and departure legs each sweep bearing
    # about the mark, so an unwindowed accumulator credits them and this
    # scores as a full lap. It must not.
    three_quarter = arc(mark, 3.0, math.pi, math.pi + 1.5 * math.pi)
    tail = [(20.0, -3.0 - y * 0.1) for y in range(30)] \
        + [(20.0 - x * 0.1, -6.0) for x in range(220)]
    card = run(approach + three_quarter + tail)
    check('a 270 degree orbit fails', not card['passed'],
          f"laps {card['laps_completed']:.2f}")

    # One and a half laps the right way, half a lap back: nets 2pi, and the
    # net alone would pass it.
    over = arc(mark, 3.0, math.pi, math.pi + 3 * math.pi, n=600)
    back_half = arc(mark, 3.0, math.pi + 3 * math.pi, math.pi + 2 * math.pi,
                    n=200)
    card = run(approach + over + back_half + ret)
    check('winding back and forth to 2pi fails', not card['passed'],
          f"net {card['swept_rad']:.2f}, counter {card['counter_wound_rad']:.2f}")
    check('and is flagged as backtracking', card['backtracked'])

    # Entering the gate from the wrong side is not an entry. Drive in from
    # beyond the mark, lap, and leave the way a correct run would arrive.
    reversed_run = [(x, y) for x, y in reversed(ret)] \
        + list(reversed(lap)) + [(x, y) for x, y in reversed(approach)]
    card = run(reversed_run)
    check('approaching the gate backwards fails', not card['passed'],
          f"entered {card['entered']}, exited {card['exited']}")

    # The threshold is relaxed by exactly the angle the gate subtends at the
    # mark, because entering and leaving through the same gate at different
    # lateral offsets makes a whole turn measure slightly under 2*pi. Check
    # the size of that relaxation directly: a track short by a real amount
    # cannot be built here, since entering and exiting through one gate
    # quantises the winding number -- which is why the slack is safe, and is
    # itself the thing worth asserting.
    card = run(approach + lap + ret)
    slack = card['gate_slack_rad']
    check('the slack is the gate subtense, not a fudge', 0.0 < slack < 0.25,
          f'slack {slack:.3f} rad')
    check('the threshold is still essentially a full turn',
          card['laps_required_rad'] > 2 * math.pi - 0.25,
          f"needed {card['laps_required_rad']:.3f}")
    # The nearest physically reachable wrong answer is three quarters of a
    # turn, 1.57 rad short -- six times the slack. Already covered by the
    # 270 degree orbit above; assert the margin explicitly.
    check('a three-quarter turn is far outside the slack',
          2 * math.pi - 1.5 * math.pi > slack * 5)

    # Sampling so coarse that +170 deg is indistinguishable from -190 deg.
    coarse = arc(mark, 3.0, math.pi, math.pi + 2 * math.pi, n=5)
    card = run(approach + coarse + ret)
    check('an under-sampled track is refused, not scored',
          not card['well_sampled'] and not card['passed'])


def test_leg_structure() -> None:
    """No leg may end where it has already been.

    Two runs were lost to this. MPPI takes its target from the global path and
    the goal checker fires on proximity to that path's last pose, so a leg
    whose final goal sits on an earlier part of its own route completes the
    moment the boat passes that spot -- skipping everything in between. Both
    failures looked like controller problems and were geometry problems.
    """
    print('leg structure')
    task = task_run.build_task('sprint', COURSE)
    legs = task.legs()
    check('the sprint is more than one leg', len(legs) > 1)

    tol = 1.0                    # comfortably over any goal xy tolerance
    ok = True
    for n, leg in enumerate(legs, 1):
        goal = leg[-1]
        for i, wp in enumerate(leg[:-1]):
            d = math.hypot(wp[0] - goal[0], wp[1] - goal[1])
            if d < tol:
                print(f'    leg {n}: waypoint {i} is {d:.2f} m from the '
                      f'final goal')
                ok = False
    check('no leg ends within 1 m of its own earlier waypoints', ok)

    # And the same against the straight line between consecutive waypoints,
    # which is what the boat actually drives.
    def seg_dist(p, a, b):
        vx, vy = b[0] - a[0], b[1] - a[1]
        L2 = vx * vx + vy * vy
        if L2 < 1e-12:
            return math.hypot(p[0] - a[0], p[1] - a[1])
        t = max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2))
        return math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy)

    ok = True
    for n, leg in enumerate(legs, 1):
        goal = leg[-1]
        for i in range(len(leg) - 2):
            d = seg_dist(goal, leg[i], leg[i + 1])
            if d < tol:
                print(f'    leg {n}: the run from waypoint {i} to {i + 1} '
                      f'passes {d:.2f} m from the final goal')
                ok = False
    check('no leg passes within 1 m of its own final goal en route', ok)

    # The whole route must still be one continuous journey.
    flat = task.waypoints()
    check('waypoints() is the concatenation of the legs',
          flat == [wp for leg in legs for wp in leg])


def test_ring_overshoot() -> None:
    print('ring overshoot')
    entry = (4.0, 0.0)           # where the boat joins the ring
    ring = task_run.ring_waypoints((0.0, 0.0), 4.0, 8, 0.0, ccw=True)
    # A whole extra step would put the last point on top of the first.
    d = math.hypot(ring[0][0] - ring[-1][0], ring[0][1] - ring[-1][1])
    check('the last ring point is clear of the first', d > 1.0, f'{d:.2f} m')

    def sweep(pts, laps=1):
        track = [entry] + [(x, y) for x, y, _ in pts]
        return task_run.signed_turn(track, (0.0, 0.0), max_step=math.pi)[0]

    swept = sweep(ring)
    check('a driven ring sweeps more than a full turn', swept > 2 * math.pi,
          f'{swept:.3f} rad')
    check('but not much more', swept < 2 * math.pi * 1.1, f'{swept:.3f} rad')

    two = task_run.ring_waypoints((0.0, 0.0), 4.0, 8, 0.0, ccw=True, laps=2)
    check('two laps sweep past 4pi', sweep(two) > 4 * math.pi,
          f'{sweep(two):.3f} rad')

    # Without the overshoot a perfectly driven ring lands exactly on 2pi, so
    # any tracking shortfall reads as an incomplete lap.
    closed = task_run.ring_waypoints((0.0, 0.0), 4.0, 8, 0.0, True, overshoot=0.0)
    check('a closed ring has no margin at all',
          abs(sweep(closed) - 2 * math.pi) < 1e-9, f'{sweep(closed):.6f} rad')

    try:
        task_run.ring_waypoints((0.0, 0.0), 4.0, 8, 0.0, True, overshoot=1.0)
        check('a whole-step overshoot is rejected', False)
    except AssertionError:
        check('a whole-step overshoot is rejected', True)


def test_missing_elements() -> None:
    print('course wiring')
    bad = dict(COURSE)
    bad['tasks'] = [{'name': 'x', 'type': 'circle_mark', 'gate': 'nope',
                     'mark': 'mark'}]
    try:
        task_run.build_task('x', bad)
        check('a task naming a missing gate raises', False)
    except KeyError:
        check('a task naming a missing gate raises', True)

    # No tasks declared at all: synthesise the whole-course channel, which is
    # what the previous runner did.
    legacy = {k: v for k, v in COURSE.items() if k != 'tasks'}
    task = task_run.build_task('channel', legacy)
    check('a course with no tasks yields the whole-course channel',
          isinstance(task, task_run.GateTransit) and len(task.gates) == 2)


def main() -> int:
    for fn in (test_signed_turn, test_ring, test_ring_overshoot,
               test_gate_transit, test_circle_mark, test_leg_structure,
               test_missing_elements):
        fn()
    print()
    if FAILURES:
        print(f'{len(FAILURES)} failure(s): {", ".join(FAILURES)}')
        return 1
    print('all task scoring checks pass')
    return 0


if __name__ == '__main__':
    sys.exit(main())
