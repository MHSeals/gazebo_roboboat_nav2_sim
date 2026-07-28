#!/usr/bin/env python3
"""Offline consistency checks for the RoboBoat workspace.

Runs without ROS, Gazebo, or a GPU -- the point is to catch the mistakes that
would otherwise cost a full build-and-launch cycle to discover. It checks the
things that span files and therefore drift silently:

  * every YAML parses and every XML/SDF/xacro is well formed;
  * the xacro actually expands;
  * thruster geometry in the URDF matches boat_params.yaml;
  * Nav2's velocity limits are inside the envelope the thrusters can deliver;
  * MPPI's prediction horizon fits inside the local costmap;
  * the generated world's obstacles intersect the lidar's scan plane;
  * every Python source compiles.

Usage:
    python3 tools/validate.py            # from the roboboat_sim directory
"""

from __future__ import annotations

import ast
import math
import re
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

try:
    import yaml
except ImportError:                                     # pragma: no cover
    sys.exit('pyyaml is required: pip install pyyaml')

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / 'src'
DESCRIPTION = SRC / 'roboboat_description'
CONTROL = SRC / 'roboboat_control'
BRINGUP = SRC / 'roboboat_bringup'

PASS, FAIL, SKIP = 'ok  ', 'FAIL', 'skip'
_results: list[tuple[str, str, str]] = []


def record(status: str, name: str, detail: str = '') -> None:
    _results.append((status, name, detail))


def check(name: str):
    """Decorator: run a check, turn a raised AssertionError into a failure."""
    def wrap(fn):
        try:
            detail = fn() or ''
            record(PASS, name, detail)
        except _Skip as exc:
            record(SKIP, name, str(exc))
        except Exception as exc:                        # noqa: BLE001
            record(FAIL, name, f'{type(exc).__name__}: {exc}')
        return fn
    return wrap


class _Skip(Exception):
    pass


# --------------------------------------------------------------------- helpers
def load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text())


def xacro_properties(path: Path) -> dict[str, float]:
    """Pull <xacro:property name=".." value=".."/> numeric constants."""
    text = path.read_text()
    found = {}
    for name, value in re.findall(
            r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"\s*/>', text):
        try:
            found[name] = float(value)
        except ValueError:
            pass
    return found


def strip_ros_params(doc: dict) -> dict:
    """Unwrap the '/**: ros__parameters:' envelope used by ROS 2 param files."""
    for key in ('/**', '/*'):
        if key in doc:
            return doc[key].get('ros__parameters', doc[key])
    return doc


# ---------------------------------------------------------------------- checks
@check('all YAML files parse')
def _yaml_parses() -> str:
    files = sorted(SRC.rglob('*.yaml')) + sorted(ROOT.glob('tools/*.yaml'))
    for path in files:
        try:
            load_yaml(path)
        except yaml.YAMLError as exc:
            raise AssertionError(f'{path.relative_to(ROOT)}: {exc}') from exc
    return f'{len(files)} files'


@check('all XML/SDF/xacro well formed')
def _xml_parses() -> str:
    patterns = ('*.xml', '*.xacro', '*.sdf')
    files = [p for pattern in patterns for p in SRC.rglob(pattern)]
    for path in files:
        try:
            ET.parse(path)
        except ET.ParseError as exc:
            raise AssertionError(f'{path.relative_to(ROOT)}: {exc}') from exc
    return f'{len(files)} files'


@check('xacro expands to a URDF')
def _xacro_expands() -> str:
    try:
        import xacro
    except ImportError as exc:
        raise _Skip('xacro not installed (pip install xacro)') from exc
    doc = xacro.process_file(str(DESCRIPTION / 'urdf' / 'roboboat.urdf.xacro'))
    urdf = ET.fromstring(doc.toxml())
    links = {el.get('name') for el in urdf.iter('link')}
    joints = urdf.findall('joint')
    assert 'base_link' in links, 'no base_link'
    # Every non-root link must be reachable through a joint.
    children = {j.find('child').get('link') for j in joints}
    orphans = links - children - {'base_link'}
    assert not orphans, f'links with no parent joint: {sorted(orphans)}'
    return f'{len(links)} links, {len(joints)} joints'


def _urdf_thruster_angles() -> dict[str, float]:
    """Thrust-axis heading in degrees per thruster, from the xacro source.

    The thrusters are visuals rather than links, so there is no joint to read.
    Evaluating each ``ang`` expression against the file's own properties checks
    the expression and not merely a constant -- a layout half-converted by
    flipping ``deg45`` alone would still be caught.
    """
    path = DESCRIPTION / 'urdf' / 'roboboat.urdf.xacro'
    props = xacro_properties(path)

    def evaluate(expr: str) -> float:
        node = ast.parse(expr.strip(), mode='eval').body

        def walk(n):
            if isinstance(n, ast.Constant):
                return float(n.value)
            if isinstance(n, ast.Name):
                return props[n.id]
            if isinstance(n, ast.UnaryOp) and isinstance(n.op, (ast.UAdd, ast.USub)):
                v = walk(n.operand)
                return v if isinstance(n.op, ast.UAdd) else -v
            if isinstance(n, ast.BinOp) and isinstance(
                    n.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)):
                a, b = walk(n.left), walk(n.right)
                return {ast.Add: a + b, ast.Sub: a - b,
                        ast.Mult: a * b, ast.Div: a / b}[type(n.op)]
            raise AssertionError(f'unsupported expression in URDF: {expr!r}')

        return walk(node)

    angles = {}
    for name, ang in re.findall(
            r'<xacro:thruster_visual\s+name="([^"]+)"[^>]*?ang="\$\{([^}]+)\}"',
            path.read_text()):
        angles[name] = math.degrees(evaluate(ang))
    assert angles, 'no thruster_visual invocations found in the URDF'
    return angles


@check('thruster geometry matches between URDF and boat_params')
def _thrusters_agree() -> str:
    props = xacro_properties(DESCRIPTION / 'urdf' / 'roboboat.urdf.xacro')
    params = strip_ros_params(load_yaml(CONTROL / 'config' / 'boat_params.yaml'))
    thr = params['thrusters']

    assert math.isclose(props['thr_x'], float(thr['x']), abs_tol=1e-9), \
        f"x: urdf {props['thr_x']} vs params {thr['x']}"
    assert math.isclose(props['thr_y'], float(thr['y']), abs_tol=1e-9), \
        f"y: urdf {props['thr_y']} vs params {thr['y']}"
    urdf_angle = math.degrees(props['deg45'])
    assert math.isclose(urdf_angle, float(thr['angle_deg']), abs_tol=1e-6), \
        f'angle: urdf {urdf_angle:.3f} deg vs params {thr["angle_deg"]} deg'

    # A pinwheel differs from an X only in the SIGN of each thruster's angle,
    # so a single scalar cannot express it and comparing one scalar cannot
    # catch a layout that is half-converted. Compare all four, per thruster,
    # against the layout the params actually select.
    sys.path.insert(0, str(CONTROL))
    from roboboat_control.allocation import build_layout

    expected = build_layout(
        str(thr['layout']), x=float(thr['x']), y=float(thr['y']),
        max_forward=float(thr['max_forward']),
        max_reverse=float(thr['max_reverse']),
        angle=math.radians(float(thr['angle_deg'])),
        sense=float(thr.get('sense', 1.0)))
    urdf_angles = _urdf_thruster_angles()
    assert set(urdf_angles) == {t.name for t in expected}, (
        f'thruster names: urdf {sorted(urdf_angles)} vs '
        f'layout {sorted(t.name for t in expected)}')
    for t in expected:
        want = math.degrees(t.angle)
        got = urdf_angles[t.name]
        assert math.isclose((got - want + 180.0) % 360.0 - 180.0, 0.0, abs_tol=1e-6), \
            (f'{t.name}: urdf {got:.3f} deg vs {thr["layout"]} layout '
             f'{want:.3f} deg')

    # The defect this check exists for is a DEGENERATE yaw arm, not a
    # particular sign pattern. A 45 degree vectored quad has a yaw arm of
    # 0.7071*(x - y) while Fx and Fy do not depend on position at all, so a
    # mounting with x close to y throws its moment arm away silently and
    # everything else still looks healthy. The old (0.55, 0.40) kept 15.6% of
    # its physical radius. Require a quarter.
    arms = [t.x * math.sin(t.angle) - t.y * math.cos(t.angle) for t in expected]
    radius = math.hypot(float(thr['x']), float(thr['y']))
    keep = abs(arms[0]) / radius
    assert keep >= 0.25, (
        f'yaw arm {abs(arms[0]):.3f} m is only {100 * keep:.0f}% of the '
        f'{radius:.3f} m physical radius -- this mounting throws its moment '
        f'arm away. For a 45 deg quad the arm is 0.7071*(x - y).')
    assert len({round(abs(a), 9) for a in arms}) == 1, \
        f'yaw arms are not equal in magnitude: {arms}'
    same_sign = all(a > 0 for a in arms) or all(a < 0 for a in arms)
    mode = 'yaw' if same_sign else 'surge'
    return (f"layout={thr['layout']} x={thr['x']} y={thr['y']} "
            f"cant={thr['angle_deg']} deg, arm {abs(arms[0]):.3f} m "
            f"({100 * keep:.0f}% of radius), all-forward mode = {mode}")


@check('lidar height matches the course generator')
def _lidar_height_agrees() -> str:
    props = xacro_properties(DESCRIPTION / 'urdf' / 'roboboat.urdf.xacro')
    course = load_yaml(DESCRIPTION / 'config' / 'course_default.yaml')
    plane = float(course['world']['lidar_plane_z'])
    assert math.isclose(props['lidar_z'], plane, abs_tol=1e-9), \
        f"urdf lidar_z {props['lidar_z']} vs course lidar_plane_z {plane}"
    return f'z={plane}'


@check('generated world obstacles intersect the scan plane')
def _world_visible() -> str:
    course = load_yaml(DESCRIPTION / 'config' / 'course_default.yaml')
    plane = float(course['world']['lidar_plane_z'])
    margin = float(course['world'].get('lidar_plane_margin', 0.10))
    tree = ET.parse(DESCRIPTION / 'worlds' / 'roboboat_course.sdf')

    checked = 0
    for model in tree.getroot().find('world').findall('model'):
        name = model.get('name')
        if name == 'water':
            continue
        pose = model.find('pose')
        z_center = float(pose.text.split()[2])
        geom = model.find('link').find('visual').find('geometry')
        if geom.find('sphere') is not None:
            half = float(geom.find('sphere').find('radius').text)
        elif geom.find('cylinder') is not None:
            half = float(geom.find('cylinder').find('length').text) / 2.0
        elif geom.find('box') is not None:
            half = float(geom.find('box').find('size').text.split()[2]) / 2.0
        else:
            continue
        bottom, top = z_center - half, z_center + half
        assert bottom + margin <= plane <= top - margin, (
            f'{name} spans [{bottom:.2f}, {top:.2f}], scan plane at {plane:.2f}')
        checked += 1
    assert checked > 0, 'no obstacles found in the world'
    return f'{checked} obstacles'


@check('Nav2 velocity limits are inside the thrust envelope')
def _limits_reachable() -> str:
    sys.path.insert(0, str(CONTROL))
    try:
        from roboboat_control.allocation import ThrustAllocator, build_layout
        from roboboat_control.dynamics import VesselParams
    except ImportError as exc:
        raise _Skip(f'numpy/roboboat_control unavailable ({exc})') from exc

    params = strip_ros_params(load_yaml(CONTROL / 'config' / 'boat_params.yaml'))
    thr, ves = params['thrusters'], params['vessel']

    allocator = ThrustAllocator(build_layout(
        str(thr['layout']),
        x=float(thr['x']), y=float(thr['y']),
        max_forward=float(thr['max_forward']),
        max_reverse=float(thr['max_reverse']),
        angle=math.radians(float(thr['angle_deg'])),
        sense=float(thr.get('sense', 1.0))))
    vessel = VesselParams(**{k: float(v) for k, v in ves.items()})

    peak = allocator.max_wrench()
    envelope = vessel.terminal_velocity(
        [peak['fx'], peak['fy'], peak['mz']])

    nav2 = load_yaml(BRINGUP / 'config' / 'nav2_mppi.yaml')
    mppi = nav2['controller_server']['ros__parameters']['FollowPath']

    for key, index, label in (('vx_max', 0, 'surge'),
                              ('vy_max', 1, 'sway'),
                              ('wz_max', 2, 'yaw')):
        limit, reachable = float(mppi[key]), float(envelope[index])
        assert limit <= reachable, (
            f'{key}={limit} exceeds the achievable {label} speed '
            f'{reachable:.2f}; MPPI would sample trajectories the boat '
            'cannot execute')
    assert abs(float(mppi['vx_min'])) <= float(envelope[0]), 'vx_min unreachable'
    return (f"vx {mppi['vx_max']}/{envelope[0]:.2f} "
            f"vy {mppi['vy_max']}/{envelope[1]:.2f} "
            f"wz {mppi['wz_max']}/{envelope[2]:.2f}")


@check('MPPI horizon fits inside the local costmap')
def _horizon_fits() -> str:
    nav2 = load_yaml(BRINGUP / 'config' / 'nav2_mppi.yaml')
    controller = nav2['controller_server']['ros__parameters']
    mppi = controller['FollowPath']
    local = nav2['local_costmap']['local_costmap']['ros__parameters']

    horizon_s = int(mppi['time_steps']) * float(mppi['model_dt'])
    reach = horizon_s * float(mppi['vx_max'])
    radius = min(float(local['width']), float(local['height'])) / 2.0
    assert reach <= radius, (
        f'{horizon_s:.2f}s at {mppi["vx_max"]} m/s reaches {reach:.2f} m but the '
        f'local costmap radius is only {radius:.2f} m; the horizon would be '
        'silently truncated')

    model_dt, freq = float(mppi['model_dt']), float(controller['controller_frequency'])
    assert model_dt <= 1.0 / freq + 1e-9, (
        f'model_dt {model_dt} exceeds the control period {1.0 / freq:.3f}')
    return f'{reach:.2f} m reach, {radius:.2f} m radius'


@check('MPPI critic list matches the configured critic blocks')
def _critics_configured() -> str:
    nav2 = load_yaml(BRINGUP / 'config' / 'nav2_mppi.yaml')
    mppi = nav2['controller_server']['ros__parameters']['FollowPath']
    listed = set(mppi['critics'])
    blocks = {k for k, v in mppi.items()
              if k.endswith('Critic') and isinstance(v, dict)}
    missing = listed - blocks
    assert not missing, f'listed but not configured: {sorted(missing)}'
    unused = blocks - listed
    assert not unused, f'configured but not listed in critics: {sorted(unused)}'
    return f'{len(listed)} critics'


@check('costmap footprints agree with the hull dimensions')
def _footprint_sane() -> str:
    props = xacro_properties(DESCRIPTION / 'urdf' / 'roboboat.urdf.xacro')
    nav2 = load_yaml(BRINGUP / 'config' / 'nav2_mppi.yaml')
    hull_half_len = props['hull_length'] / 2.0
    hull_half_beam = props['hull_separation'] / 2.0 + props['hull_radius']

    for costmap in ('local_costmap', 'global_costmap'):
        params = nav2[costmap][costmap]['ros__parameters']
        points = yaml.safe_load(params['footprint'])
        xs = [abs(p[0]) for p in points]
        ys = [abs(p[1]) for p in points]
        assert max(xs) >= hull_half_len, (
            f'{costmap} footprint half-length {max(xs)} < hull {hull_half_len}')
        assert max(ys) >= hull_half_beam, (
            f'{costmap} footprint half-beam {max(ys)} < hull {hull_half_beam}')
        inscribed = min(max(xs), max(ys))
        inflation = float(params['inflation_layer']['inflation_radius'])
        assert inflation > inscribed, (
            f'{costmap} inflation_radius {inflation} <= inscribed radius '
            f'{inscribed}; there would be no cost gradient to follow')
    return f'hull {2 * hull_half_len:.2f} x {2 * hull_half_beam:.2f} m'


@check('behavior tree references only the configured behaviors')
def _bt_behaviors() -> str:
    nav2 = load_yaml(BRINGUP / 'config' / 'nav2_mppi.yaml')
    plugins = set(nav2['behavior_server']['ros__parameters']['behavior_plugins'])
    required = {'Spin': 'spin', 'BackUp': 'backup', 'Wait': 'wait',
                'DriveOnHeading': 'drive_on_heading'}

    # Every tree, not just the to-pose one. Per-task trees are the intended way
    # to vary behaviour here, so a check that only lints one of them stops
    # covering the thing it was written to cover the moment a task is added.
    trees = sorted((BRINGUP / 'behavior_trees').glob('*.xml'))
    assert trees, 'no behaviour trees found'
    for path in trees:
        used = {el.tag for el in ET.parse(path).getroot().iter()}
        for node, plugin in required.items():
            if node in used:
                assert plugin in plugins, (
                    f'{path.name} uses <{node}> but behavior_server does not '
                    f"load '{plugin}'")
    return f'{len(trees)} trees'


@check("every task's goals fit inside the global costmap")
def _tasks_fit_costmap() -> str:
    """A goal outside the rolling global costmap is an unplannable goal.

    ComputePathThroughPoses plans against one robot-centred costmap, so every
    goal still in the list has to be inside the window at plan time -- not
    just the next one. Smac throws GoalOutsideMapBounds, the recovery clears
    the costmap, the retry fails the same way, and the goal aborts. That
    presents as a mysterious abort a long way from anything, so catch it here.

    This is also the reason tasks are run one NavigateThroughPoses call at a
    time: the window has to hold one task's extent, not the whole course.
    """
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import course_tasks

    course = load_yaml(DESCRIPTION / 'config' / 'course_default.yaml')
    nav2 = load_yaml(BRINGUP / 'config' / 'nav2_mppi.yaml')
    glob = nav2['global_costmap']['global_costmap']['ros__parameters']
    half = min(float(glob['width']), float(glob['height'])) / 2.0

    spawn = course.get('spawn', {})
    start = (float(spawn.get('x', 0.0)), float(spawn.get('y', 0.0)))

    worst_name, worst = None, 0.0
    specs = course.get('tasks') or [{'name': 'channel', 'type': 'gate_transit'}]
    for spec in specs:
        task = course_tasks.build_task(spec['name'], course)
        here = ((float(task.start['x']), float(task.start['y']))
                if task.start else start)
        route = [here] + [(x, y) for x, y, _ in task.waypoints()]
        # The boat can be anywhere on the route while a later goal is still in
        # the list, so the binding number is the largest separation between
        # any route point and any goal that comes after it.
        for i, here in enumerate(route):
            for x, y in route[i:]:
                d = max(abs(x - here[0]), abs(y - here[1]))
                if d > worst:
                    worst_name, worst = spec['name'], d
    assert worst < half - 2.0, (
        f'task {worst_name!r} spans {worst:.1f} m, which does not fit inside '
        f'the {half:.0f} m half-window of the global costmap with 2 m to '
        'spare; goals will abort as GoalOutsideMapBounds')
    return f'worst span {worst:.1f} m of {half:.0f} m ({worst_name})'


@check('metric callbacks run and are duration-independent')
def _metrics_ok() -> str:
    import subprocess
    result = subprocess.run(
        [sys.executable, str(ROOT / 'tools' / 'test_metrics.py')],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout.strip() or result.stderr.strip()
    return result.stdout.strip().replace('  metric callbacks ok: ', '')


@check('all Python sources compile')
def _python_compiles() -> str:
    files = sorted(SRC.rglob('*.py')) + sorted((ROOT / 'tools').rglob('*.py'))
    for path in files:
        try:
            ast.parse(path.read_text(), filename=str(path))
        except SyntaxError as exc:
            raise AssertionError(f'{path.relative_to(ROOT)}: {exc}') from exc
    return f'{len(files)} files'


# ------------------------------------------------------------------------ main
def main() -> int:
    width = max(len(name) for _, name, _ in _results)
    for status, name, detail in _results:
        line = f'[{status}] {name.ljust(width)}'
        if detail:
            line += f'  {detail}'
        print(line)

    failures = sum(1 for status, _, _ in _results if status == FAIL)
    skipped = sum(1 for status, _, _ in _results if status == SKIP)
    print()
    print(f'{len(_results) - failures - skipped} passed, '
          f'{failures} failed, {skipped} skipped')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
