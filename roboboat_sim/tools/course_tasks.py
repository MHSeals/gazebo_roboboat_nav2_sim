#!/usr/bin/env python3
"""Task definitions, waypoint generation and scoring. No ROS.

Kept separate from tools/task_run.py so that everything except the action
client can be exercised without a sourced workspace: the offline tests import
this directly, and tools/validate.py uses it to check that a task's goals fit
inside the global costmap. Importing rclpy to answer a question about geometry
is how offline checks quietly stop being offline.
"""

from __future__ import annotations

import math
from pathlib import Path

# Hull half-beam. It lives here rather than in nav_test because nav_test pulls
# in ROS message types, and this module exists precisely so the geometry can be
# checked without them. nav_test imports it back.
FOOTPRINT_HALF_Y = 0.55


# --------------------------------------------------------------- geometry


def gate_buoys(gate: dict) -> tuple[tuple, tuple]:
    """The two buoy centres of a gate, port and starboard."""
    heading = math.radians(float(gate['heading_deg']))
    half = float(gate['width']) / 2.0
    px, py = -math.sin(heading) * half, math.cos(heading) * half
    cx, cy = float(gate['x']), float(gate['y'])
    return (cx + px, cy + py), (cx - px, cy - py)


def segments_cross(p1, p2, q1, q2) -> bool:
    """True if segment p1-p2 properly crosses segment q1-q2."""
    def side(a, b, c):
        return ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    d1, d2 = side(q1, q2, p1), side(q1, q2, p2)
    d3, d4 = side(p1, p2, q1), side(p1, p2, q2)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def crossings(track: list[tuple], a, b, start: int = 0) -> list[int]:
    """Indices in `track` where the hull crossed the segment a-b."""
    return [i for i in range(start, len(track) - 1)
            if segments_cross(track[i], track[i + 1], a, b)]


def signed_turn(track: list[tuple], centre: tuple,
                min_radius: float = 0.5,
                max_step: float = math.pi / 4) -> tuple[float, float, bool]:
    """Signed angle swept about `centre`: (net, counter-wound, sane).

    Positive is counter-clockwise. Summing per-sample deltas rather than
    comparing start and end bearings is what distinguishes a full lap from a
    boat that merely ended up where it started -- and what makes a lap driven
    the wrong way come out negative instead of also reading as a lap.

    Three things this has to survive:

    * A sample near the centre. Bearing is undefined through the mark and
      ill-conditioned around it, so a close pass can fabricate or destroy a
      whole lap in one wrap. Samples inside `min_radius` are dropped.
    * A step larger than `max_step`, which means the sampling was too coarse
      to tell +170 deg from -190 deg. That is not a lap to be scored down, it
      is a measurement that cannot be trusted, so it is reported separately.
    * Net cancellation. A run that goes 1.5 laps the right way and 0.5 back
      nets a full lap. `counter` accumulates only the wrong-signed motion, so
      the caller can refuse to launder it.
    """
    net = 0.0
    ccw_sum = 0.0
    cw_sum = 0.0
    sane = True
    prev = None
    for x, y in track:
        dx, dy = x - centre[0], y - centre[1]
        if math.hypot(dx, dy) < min_radius:
            prev = None          # break the chain rather than bridge the gap
            continue
        bearing = math.atan2(dy, dx)
        if prev is not None:
            step = (bearing - prev + math.pi) % (2 * math.pi) - math.pi
            if abs(step) > max_step:
                sane = False
            net += step
            if step > 0:
                ccw_sum += step
            else:
                cw_sum -= step
        prev = bearing
    counter = cw_sum if net >= 0 else ccw_sum
    return net, counter, sane


def crossing_side(p, a, b) -> float:
    """Which side of the line a-b the point p lies on."""
    return ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]))


def crossing_point(p1, p2, q1, q2) -> tuple[float, float]:
    """Where segment p1-p2 meets segment q1-q2.

    The track sample before a crossing is up to one odom period short of the
    gate line. At 0.5 m/s and 50 Hz that is a centimetre, which is harmless
    until the offset it biases goes into a contract -- so interpolate.
    """
    d1 = crossing_side(p1, q1, q2)
    d2 = crossing_side(p2, q1, q2)
    if d1 == d2:
        return p1
    t = d1 / (d1 - d2)
    return (p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]))


def ring_waypoints(centre: tuple, radius: float, points: int,
                   start_bearing: float, ccw: bool,
                   laps: float = 1.0, overshoot: float = 0.5) -> list[tuple]:
    """Poses around `centre`, ordered in the commanded direction.

    `points` divides a full turn into that many steps; `laps` says how many
    turns to emit; `overshoot` adds a final part-step beyond them, measured in
    steps.

    The overshoot is not decoration. Without it the last waypoint sits exactly
    on the entry bearing, so the boat only sweeps a full 2*pi if it tracks the
    ring perfectly, and any shortfall reads as an incomplete lap. It must be a
    FRACTION of a step, never a whole one: a whole extra step lands the final
    waypoint on top of the first, and then arriving at the first ring waypoint
    satisfies the leg's final goal. Measured -- the boat drove to the first
    ring waypoint, the goal checker declared the leg complete, and it turned
    round and came home having swept 0.01 rad.

    Each pose faces along the direction of travel, which is the tangent, so
    the goal orientation never fights the turn.
    """
    assert 0.0 <= overshoot < 1.0, 'overshoot is a fraction of a step'
    sign = 1.0 if ccw else -1.0
    step = 2 * math.pi / points
    angles = [i * step for i in range(1, int(round(points * laps)) + 1)]
    if overshoot:
        angles.append(angles[-1] + overshoot * step)

    out = []
    for a in angles:
        theta = start_bearing + sign * a
        out.append((centre[0] + radius * math.cos(theta),
                    centre[1] + radius * math.sin(theta),
                    theta + sign * math.pi / 2.0))
    return out


# ------------------------------------------------------------------ tasks


class Task:
    """A task: the waypoints to drive, and the rule that scores the result."""

    type_name = 'abstract'

    def __init__(self, spec: dict, course: dict) -> None:
        self.spec = spec
        self.course = course
        self.name = spec['name']
        self.bt = spec.get('behavior_tree')
        self.timeout = float(spec.get('timeout_s', 420.0))
        # Where the boat is expected to be when this task begins. On the water
        # a task starts wherever the previous one ended, not at the spawn, and
        # the costmap-extent check needs to know which.
        self.start = spec.get('start')
        self.gates_by_name = {g['name']: g for g in course.get('gates', [])}
        self.marks_by_name = {b['name']: b for b in course.get('buoys', [])
                              if 'name' in b}

    def gate(self, name: str) -> dict:
        if name not in self.gates_by_name:
            raise KeyError(f'task {self.name}: no gate named {name!r}')
        g = self.gates_by_name[name]
        radius = float(self.course['buoy_types'][g.get('left', 'green')]['radius'])
        left, right = gate_buoys(g)
        return {
            'name': g['name'], 'x': float(g['x']), 'y': float(g['y']),
            'yaw': math.radians(float(g['heading_deg'])),
            'left': left, 'right': right, 'width': float(g['width']),
            'free_water': float(g['width']) - 2 * radius,
        }

    def mark(self, name: str) -> dict:
        if name not in self.marks_by_name:
            raise KeyError(f'task {self.name}: no named buoy {name!r}')
        b = self.marks_by_name[name]
        radius = float(self.course['buoy_types'][b['type']]['radius'])
        return {'name': name, 'x': float(b['x']), 'y': float(b['y']),
                'radius': radius}

    def waypoints(self) -> list[tuple]:
        """(x, y, yaw) poses to send, in order."""
        raise NotImplementedError

    def legs(self) -> list[list[tuple]]:
        """Waypoints grouped into separate NavigateThroughPoses goals.

        Most tasks are one leg. A task whose route returns to where it started
        must not be: MPPI takes its target from the global path, and when that
        path both begins and ends at the boat's current position there is no
        target to move toward. Measured -- the sprint sent as a single goal
        drove to its first waypoint, found the last pose of the path underneath
        the hull, and held station there for 265 s while the controller
        reported 'failed to make progress' every 20 s.
        """
        return [self.waypoints()]

    def score(self, track: list[tuple], min_clearance: float) -> dict:
        raise NotImplementedError


class GateTransit(Task):
    """Drive through a sequence of gates, between the buoys, in order.

    The scoring rule is carried over unchanged from course_run.py so that this
    refactor cannot move the channel result.
    """

    type_name = 'gate_transit'

    def __init__(self, spec: dict, course: dict) -> None:
        super().__init__(spec, course)
        names = spec.get('gates') or [g['name'] for g in course['gates']]
        self.gates = [self.gate(n) for n in names]

    def waypoints(self) -> list[tuple]:
        return [(g['x'], g['y'], g['yaw']) for g in self.gates]

    def score(self, track, min_clearance) -> dict:
        results = []
        search_from = 0
        for gate in self.gates:
            hits = crossings(track, gate['left'], gate['right'], search_from)
            offset = None
            if hits:
                i = hits[0]
                px, py = crossing_point(track[i], track[i + 1],
                                        gate['left'], gate['right'])
                mid = ((gate['left'][0] + gate['right'][0]) / 2.0,
                       (gate['left'][1] + gate['right'][1]) / 2.0)
                offset = math.hypot(px - mid[0], py - mid[1])
                search_from = i + 1
            results.append({
                'gate': gate['name'], 'transited': bool(hits),
                # `if offset` would report None for a perfectly centred
                # crossing, which is indistinguishable from a miss.
                'offset_from_centre_m': (round(offset, 3)
                                         if offset is not None else None),
                'free_water_m': round(gate['free_water'], 2),
                # The most the hull can clear this gate by, if it threads the
                # exact centreline. Normalising the measured clearance by this
                # is what lets a safety criterion survive a change of gate
                # width -- see tuning/regression_channel.json.
                'clearance_ceiling_m': round(
                    (gate['free_water'] - 2 * FOOTPRINT_HALF_Y) / 2.0, 3),
            })
        transited = sum(1 for r in results if r['transited'])
        ceiling = min((r['clearance_ceiling_m'] for r in results), default=None)
        return {
            'gates_total': len(results), 'gates_transited': transited,
            'contact': min_clearance <= 0.0,
            'clearance_ceiling_m': ceiling,
            'clearance_fraction': (round(min_clearance / ceiling, 3)
                                   if ceiling and ceiling > 0 else None),
            'gates': results,
            'passed': transited == len(results) and min_clearance > 0.0,
        }

    def describe(self, card: dict) -> list[str]:
        lines = [f'{"gate":>12} {"transited":>10} {"offset":>10} {"free water":>12}']
        for g in card['gates']:
            off = (f"{g['offset_from_centre_m']:.2f} m"
                   if g['offset_from_centre_m'] is not None else '-')
            lines.append(f"{g['gate']:>12} "
                         f"{'YES' if g['transited'] else 'MISSED':>10} "
                         f"{off:>10} {g['free_water_m']:>10.2f} m")
        lines.append('')
        lines.append(f"  gates transited : {card['gates_transited']}/{card['gates_total']}")
        return lines


class CircleMark(Task):
    """Enter through a gate, circle a mark buoy, come back out through it.

    This is the speed challenge. Scoring cannot be "crossed a line": a boat
    that drives to the mark and back crosses every line the correct run does.
    What separates them is the angle swept about the mark, which is why the
    rule is an accumulated signed turn rather than a set of waypoint hits.
    """

    type_name = 'circle_mark'

    def __init__(self, spec: dict, course: dict) -> None:
        super().__init__(spec, course)
        self.entry = self.gate(spec['gate'])
        self.mk = self.mark(spec['mark'])
        self.direction = spec.get('direction', 'ccw').lower()
        if self.direction not in ('cw', 'ccw'):
            raise ValueError(f'task {self.name}: direction must be cw or ccw')
        self.ccw = self.direction == 'ccw'
        self.laps = float(spec.get('laps', 1))
        self.radius = float(spec.get('radius', 3.0))
        self.points = int(spec.get('points', 8))
        self.standoff = float(spec.get('standoff', 4.0))

        # Ring spacing must exceed the BT's RemovePassedGoals radius, or a
        # single pose update retires the whole ring at once and the boat cuts
        # straight past the mark -- scoring the gate and never circling.
        self.ring_spacing = 2 * self.radius * math.sin(math.pi / self.points)

    def waypoints(self) -> list[tuple]:
        """Approach, cross the gate, ring the mark, cross back, stand off.

        No waypoint sits at the gate centre. That is deliberate: a gate centre
        is inside inflated space, which is what forced the channel tree's
        1.6 m goal-retirement radius, and a 1.6 m radius on a ring this size
        would retire waypoints two at a time. Standing the approach and exit
        poses off in open water on the gate axis means the boat still transits
        the gate -- it is between the two poses -- while every waypoint of
        this task sits in genuinely free water.
        """
        return [wp for leg in self.legs() for wp in leg]

    def legs(self) -> list[list[tuple]]:
        gx, gy, gyaw = self.entry['x'], self.entry['y'], self.entry['yaw']
        mx, my = self.mk['x'], self.mk['y']

        # Enter the ring on the side facing the gate, so the first ring
        # waypoint is the one the boat naturally reaches first.
        start_bearing = math.atan2(gy - my, gx - mx)
        ring = ring_waypoints((mx, my), self.radius, self.points,
                              start_bearing, self.ccw, laps=self.laps)

        approach = (gx - self.standoff * math.cos(gyaw),
                    gy - self.standoff * math.sin(gyaw), gyaw)
        # First point of the ring, entered head-on from the gate.
        ring_entry = (mx + self.radius * math.cos(start_bearing),
                      my + self.radius * math.sin(start_bearing),
                      start_bearing + math.pi)
        # Lined up on the gate from the mark side. Without this the return leg
        # is a single goal beyond the gate, and the planner is free to go round
        # the outside of a 2.4 m opening rather than through it -- cheaper by
        # cost, and a failed task.
        gate_run_in = (gx + self.standoff * math.cos(gyaw),
                       gy + self.standoff * math.sin(gyaw), gyaw + math.pi)
        # Back out the way it came in, and further: an exit pose on top of the
        # approach pose would make the outbound and return legs one path that
        # begins and ends in the same place.
        stand_off = (gx - 2 * self.standoff * math.cos(gyaw),
                     gy - 2 * self.standoff * math.sin(gyaw), gyaw + math.pi)

        # Split at the far side of the ring, not at the gate. A leg's final
        # goal must be somewhere the boat does not pass through earlier in
        # that same leg, or the goal checker fires on the way past and the
        # rest of the leg is skipped -- and the ring is precisely a shape that
        # comes back to where it has already been. Cutting at the far side
        # gives leg 1 a final goal 8 m off its own outbound corridor, and
        # leg 2 a final goal outside the gate it has not yet reached.
        half = max(1, len(ring) // 2)
        return [[approach, ring_entry] + ring[:half],
                ring[half:] + [gate_run_in, stand_off]]

    def score(self, track, min_clearance) -> dict:
        left, right = self.entry['left'], self.entry['right']
        centre = (self.mk['x'], self.mk['y'])

        # Outbound and inbound are not interchangeable, and a bare crossing
        # test cannot tell them apart. Which way the boat went through the
        # gate is the sign of its travel along the gate heading -- not which
        # side of the gate line it started on, whose sign depends on the
        # port/starboard ordering and so flips with the gate's heading.
        nx, ny = math.cos(self.entry['yaw']), math.sin(self.entry['yaw'])

        def outward(i: int) -> bool:
            dx = track[i + 1][0] - track[i][0]
            dy = track[i + 1][1] - track[i][1]
            return dx * nx + dy * ny > 0

        hits = crossings(track, left, right)
        outbound = [i for i in hits if outward(i)]
        entered_at = outbound[0] if outbound else None
        exited_at = None
        if entered_at is not None:
            back = [i for i in hits if i > entered_at and not outward(i)]
            exited_at = back[-1] if back else None

        # The window matters. Accumulating over the whole track would credit
        # the bearing swept on the approach and the departure -- a straight
        # pass by a point sweeps up to pi about it -- so a 270 deg orbit would
        # score as a lap. Only what happens between the gate crossings counts.
        end = (exited_at + 1) if exited_at is not None else len(track)
        window = track[entered_at:end] if entered_at is not None else []
        swept, counter, sane = signed_turn(window, centre,
                                           min_radius=self.mk['radius'] + 0.2)

        # How much of a turn the gate itself subtends at the mark. The window
        # runs from one gate crossing to another, and the boat does not cross
        # at the same lateral offset both times, so the measured sweep differs
        # from a whole turn by at most this -- it is the resolution of the
        # instrument, not a shortfall in the lap. Measured: a run that circled
        # cleanly came out at 6.270 rad against 2*pi = 6.283, and demanding
        # 2*pi exactly failed it.
        #
        # This cannot rescue a lap that was not driven: the gate subtends
        # 0.18 rad here, and the nearest wrong answer -- three quarters of a
        # turn -- is 1.57 rad short.
        span = math.hypot(self.entry['x'] - centre[0],
                          self.entry['y'] - centre[1])
        slack = 2 * math.atan2(self.entry['width'] / 2.0, max(span, 1e-6))
        required = 2 * math.pi * self.laps - slack
        wrong_way = (swept < 0) if self.ccw else (swept > 0)
        circled = (swept >= required) if self.ccw else (swept <= -required)
        # A run that goes one and a half laps the right way and half a lap
        # back nets a full lap. Refuse to launder it.
        laundered = counter > 0.25 * 2 * math.pi

        passed = bool(entered_at is not None and exited_at is not None
                      and circled and not laundered and sane
                      and min_clearance > 0.0)
        # Same normalisation the channel uses: the most the hull could clear
        # the entrance gate by on the exact centreline. The sprint gate is
        # 2.4 m against the channel's 2.8 m, so the ceiling is 0.47 m rather
        # than 0.65 m, and a raw clearance is not comparable between them.
        ceiling = (self.entry['free_water'] - 2 * FOOTPRINT_HALF_Y) / 2.0
        return {
            'gate': self.entry['name'],
            'mark': self.mk['name'],
            'direction': self.direction,
            'laps_required': self.laps,
            'entered': entered_at is not None,
            'exited': exited_at is not None,
            'swept_rad': round(swept, 3),
            'laps_completed': round(abs(swept) / (2 * math.pi), 3),
            'laps_required_rad': round(required, 3),
            'gate_slack_rad': round(slack, 3),
            'counter_wound_rad': round(counter, 3),
            'backtracked': laundered,
            'well_sampled': sane,
            'wrong_direction': wrong_way,
            'circled': circled,
            'contact': min_clearance <= 0.0,
            'clearance_ceiling_m': round(ceiling, 3),
            'clearance_fraction': (round(min_clearance / ceiling, 3)
                                   if ceiling > 0 else None),
            'ring_radius_m': self.radius,
            'ring_spacing_m': round(self.ring_spacing, 2),
            'passed': passed,
        }

    def describe(self, card: dict) -> list[str]:
        return [
            f"  entrance gate   : {card['gate']}  "
            f"{'transited' if card['entered'] else 'NOT TRANSITED'}",
            f"  mark            : {card['mark']}, circle {card['direction']} "
            f"x{card['laps_required']:.0f}",
            f"  swept           : {card['swept_rad']:.2f} rad "
            f"({card['laps_completed']:.2f} laps)"
            + ('  WRONG DIRECTION' if card['wrong_direction'] else '')
            + ('  BACKTRACKED' if card['backtracked'] else '')
            + ('' if card['well_sampled'] else '  UNDER-SAMPLED'),
            f"  returned        : {'yes' if card['exited'] else 'NO'}",
            f"  ring            : R {card['ring_radius_m']:.1f} m, "
            f"waypoint spacing {card['ring_spacing_m']:.2f} m",
        ]


TASK_TYPES = {cls.type_name: cls for cls in (GateTransit, CircleMark)}


def resolve_bt(name: str) -> Path:
    """Turn a behaviour tree name into the installed absolute path.

    bt_navigator treats the goal's `behavior_tree` field as a filesystem path
    and quietly falls back to its default tree when the file is missing, so a
    bare filename produces a run that looks fine and used the wrong tree.
    Fail here instead.
    """
    path = Path(name)
    if not path.is_absolute():
        share = Path(get_package_share_directory('roboboat_bringup'))
        path = share / 'behavior_trees' / path.name
    if not path.exists():
        raise SystemExit(f'behaviour tree {name!r} is not installed at {path}; '
                         'colcon build the bringup package')
    return path


def build_task(name: str, course: dict) -> Task:
    specs = course.get('tasks')
    if not specs:
        # No task declarations: behave exactly like course_run.py did.
        specs = [{'name': 'channel', 'type': 'gate_transit'}]
    by_name = {s['name']: s for s in specs}
    if name not in by_name:
        raise SystemExit(f'no task named {name!r}; have '
                         f'{", ".join(sorted(by_name))}')
    spec = by_name[name]
    kind = spec.get('type', 'gate_transit')
    if kind not in TASK_TYPES:
        raise SystemExit(f'task {name!r} has unknown type {kind!r}; have '
                         f'{", ".join(sorted(TASK_TYPES))}')
    return TASK_TYPES[kind](spec, course)
