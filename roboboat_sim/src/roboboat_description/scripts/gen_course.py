#!/usr/bin/env python3
"""Generate a Gazebo Harmonic world from a RoboBoat course description.

The generated world is deliberately cheap to simulate:

* the water is a single visual plane with no collision, no waves, no shader
  work beyond a flat colour;
* buoys and walls are static, visual-only models (the GPU lidar rasterises
  visuals, so collision geometry buys nothing and costs broadphase time);
* only the systems we actually consume are loaded -- physics, scene
  broadcaster, user commands, sensors, imu, navsat.

Usage:
    gen_course.py --course course_default.yaml --output worlds/roboboat_course.sdf
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET

import yaml

INDENT = '  '


def _el(parent: ET.Element, tag: str, text: str | None = None, **attrs) -> ET.Element:
    el = ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        el.text = text
    return el


def _pose(parent: ET.Element, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0) -> None:
    _el(parent, 'pose', f'{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}')


def _material(parent: ET.Element, rgb, alpha: float = 1.0, emissive: float = 0.0) -> None:
    mat = _el(parent, 'material')
    colour = f'{rgb[0]:.3f} {rgb[1]:.3f} {rgb[2]:.3f} {alpha:.3f}'
    _el(mat, 'ambient', colour)
    _el(mat, 'diffuse', colour)
    _el(mat, 'specular', '0.1 0.1 0.1 1')
    if emissive:
        e = emissive
        _el(mat, 'emissive', f'{rgb[0] * e:.3f} {rgb[1] * e:.3f} {rgb[2] * e:.3f} 1')


def _geometry(parent: ET.Element, spec: dict) -> None:
    geom = _el(parent, 'geometry')
    shape = spec.get('shape', 'cylinder')
    if shape == 'cylinder':
        cyl = _el(geom, 'cylinder')
        _el(cyl, 'radius', f"{spec['radius']:.4f}")
        _el(cyl, 'length', f"{spec['height']:.4f}")
    elif shape == 'sphere':
        _el(_el(geom, 'sphere'), 'radius', f"{spec['radius']:.4f}")
    elif shape == 'box':
        _el(_el(geom, 'box'), 'size',
            f"{spec['size'][0]:.4f} {spec['size'][1]:.4f} {spec['size'][2]:.4f}")
    else:
        raise ValueError(f'unsupported shape: {shape}')


def _vertical_extent(spec: dict, z_center: float) -> tuple[float, float]:
    """(bottom, top) of a shape whose centre sits at ``z_center``."""
    if spec['shape'] == 'sphere':
        half = float(spec['radius'])
    elif spec['shape'] == 'cylinder':
        half = float(spec['height']) / 2.0
    elif spec['shape'] == 'box':
        half = float(spec['size'][2]) / 2.0
    else:
        raise ValueError(f"unsupported shape: {spec['shape']}")
    return z_center - half, z_center + half


def _check_lidar_visibility(name: str, spec: dict, z_center: float,
                            plane_z: float, margin: float) -> None:
    """Fail loudly if the boat's scan plane would miss or graze an obstacle.

    A single-plane lidar sees nothing that does not straddle its own height.
    Getting this wrong produces a world that looks correct in Gazebo and is
    completely empty to Nav2 -- an expensive hour of debugging that this
    check costs nothing to prevent.
    """
    bottom, top = _vertical_extent(spec, z_center)
    if not (bottom + margin <= plane_z <= top - margin):
        raise ValueError(
            f"{name!r} spans z=[{bottom:.2f}, {top:.2f}] but the lidar plane is "
            f'at z={plane_z:.2f} (margin {margin:.2f}). The boat would not see '
            'it. Raise the obstacle, or lower lidar_plane_z and lidar_z in '
            'roboboat.urdf.xacro together.')


def _static_visual_model(world: ET.Element, name: str, pose: tuple, geom_spec: dict,
                         rgb, with_collision: bool) -> None:
    """A static model carrying one visual (and optionally a matching collision)."""
    model = _el(world, 'model', name=name)
    _el(model, 'static', 'true')
    _pose(model, *pose)
    link = _el(model, 'link', name='link')
    visual = _el(link, 'visual', name='visual')
    _geometry(visual, geom_spec)
    _material(visual, rgb, emissive=0.25)
    if with_collision:
        collision = _el(link, 'collision', name='collision')
        _geometry(collision, geom_spec)


def _add_physics(world: ET.Element, cfg: dict) -> None:
    physics = _el(world, 'physics', name='fast', type='ignored')
    _el(physics, 'max_step_size', str(cfg['max_step_size']))
    _el(physics, 'real_time_factor', str(cfg['real_time_factor']))
    # 0 real_time_update_rate == run as fast as the solver allows, throttled
    # only by real_time_factor.
    _el(physics, 'real_time_update_rate',
        str(0.0 if float(cfg['real_time_factor']) == 0.0
            else round(1.0 / float(cfg['max_step_size']))))


def _add_plugins(world: ET.Element) -> None:
    specs = [
        ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
        ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
        ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
        ('gz-sim-imu-system', 'gz::sim::systems::Imu'),
        ('gz-sim-navsat-system', 'gz::sim::systems::NavSat'),
    ]
    for filename, name in specs:
        _el(world, 'plugin', filename=filename, name=name)
    sensors = _el(world, 'plugin',
                  filename='gz-sim-sensors-system',
                  name='gz::sim::systems::Sensors')
    _el(sensors, 'render_engine', 'ogre2')


def _add_scene(world: ET.Element) -> None:
    scene = _el(world, 'scene')
    _el(scene, 'ambient', '0.85 0.88 0.92 1')
    _el(scene, 'background', '0.55 0.70 0.86 1')
    _el(scene, 'grid', 'false')
    # Shadows are the single most expensive rendering feature and buy us
    # nothing for lidar-driven navigation.
    _el(scene, 'shadows', 'false')

    light = _el(world, 'light', type='directional', name='sun')
    _el(light, 'cast_shadows', 'false')
    _pose(light, 0, 0, 40)
    _el(light, 'diffuse', '0.9 0.9 0.9 1')
    _el(light, 'specular', '0.2 0.2 0.2 1')
    _el(light, 'direction', '-0.4 0.3 -0.9')


def _add_camera(world: ET.Element, spec: dict) -> None:
    """A static model carrying a camera sensor.

    Cameras exist so a headless run can be reviewed as Gazebo actually renders
    it, rather than only as recorded telemetry. They are expensive under
    software rendering, which is why the default world has none -- generate a
    second world with --cameras when you want footage.
    """
    model = _el(world, 'model', name=f"cam_{spec['name']}")
    _el(model, 'static', 'true')
    _pose(model, spec['x'], spec['y'], spec['z'],
          0.0, math.radians(spec.get('pitch_deg', 0.0)),
          math.radians(spec.get('yaw_deg', 0.0)))
    link = _el(model, 'link', name='link')
    sensor = _el(link, 'sensor', name=spec['name'], type='camera')
    _el(sensor, 'topic', f"camera/{spec['name']}")
    _el(sensor, 'update_rate', str(spec.get('rate', 10)))
    _el(sensor, 'always_on', '1')
    camera = _el(sensor, 'camera')
    _el(camera, 'horizontal_fov', str(spec.get('hfov', 1.2)))
    image = _el(camera, 'image')
    _el(image, 'width', str(spec.get('width', 960)))
    _el(image, 'height', str(spec.get('height', 640)))
    _el(image, 'format', 'R8G8B8')
    clip = _el(camera, 'clip')
    _el(clip, 'near', '0.2')
    _el(clip, 'far', '400')


def _add_water(world: ET.Element, size: float) -> None:
    model = _el(world, 'model', name='water')
    _el(model, 'static', 'true')
    _pose(model, 0, 0, -0.02)
    link = _el(model, 'link', name='link')
    visual = _el(link, 'visual', name='surface')
    _geometry(visual, {'shape': 'box', 'size': [size, size, 0.04]})
    _material(visual, (0.09, 0.28, 0.42), alpha=1.0)


def _gate_buoys(gate: dict) -> list[tuple[str, str, float, float]]:
    """Expand a gate into (name, buoy_type, x, y) tuples."""
    heading = math.radians(float(gate['heading_deg']))
    half = float(gate['width']) / 2.0
    # Port (+y in the gate's own frame) is to the left of the heading.
    px, py = -math.sin(heading) * half, math.cos(heading) * half
    cx, cy = float(gate['x']), float(gate['y'])
    return [
        (f"gate_{gate['name']}_left", gate.get('left', 'green'), cx + px, cy + py),
        (f"gate_{gate['name']}_right", gate.get('right', 'red'), cx - px, cy - py),
    ]


def build_world(course: dict, cameras: bool = False) -> ET.Element:
    wcfg = course['world']
    types = course['buoy_types']
    with_collision = bool(wcfg.get('buoy_collision', False))

    sdf = ET.Element('sdf', version='1.10')
    world = _el(sdf, 'world', name=wcfg['name'])

    _add_physics(world, wcfg)
    _add_plugins(world)
    _add_scene(world)

    # Gravity is irrelevant (the boat's link disables it and everything else
    # is static) but keep it physical so anything you add later behaves.
    _el(world, 'gravity', '0 0 -9.8')

    sph = _el(world, 'spherical_coordinates')
    _el(sph, 'surface_model', 'EARTH_WGS84')
    _el(sph, 'world_frame_orientation', 'ENU')
    _el(sph, 'latitude_deg', str(wcfg['latitude_deg']))
    _el(sph, 'longitude_deg', str(wcfg['longitude_deg']))
    _el(sph, 'elevation', str(wcfg.get('elevation_m', 0.0)))
    _el(sph, 'heading_deg', '0')

    _add_water(world, float(wcfg['water_size']))

    if cameras:
        for spec in course.get('cameras') or []:
            _add_camera(world, spec)

    placed: list[tuple[str, str, float, float]] = []
    for gate in course.get('gates') or []:
        placed.extend(_gate_buoys(gate))
    seen: set[str] = set()
    for i, buoy in enumerate(course.get('buoys') or []):
        # A buoy a task refers to by name -- the sprint mark, a dock marker --
        # gets that name in the world, so the SDF reads like the course file
        # and a replay can label it. Unnamed scenery keeps the index form.
        label = buoy.get('name') or f"{i:02d}_{buoy['type']}"
        if label in seen:
            raise KeyError(f'two buoys are both named {label!r}')
        seen.add(label)
        placed.append((f'buoy_{label}', buoy['type'],
                       float(buoy['x']), float(buoy['y'])))

    plane_z = float(wcfg.get('lidar_plane_z', 0.35))
    margin = float(wcfg.get('lidar_plane_margin', 0.10))

    for name, btype, x, y in placed:
        if btype not in types:
            raise KeyError(f'buoy type {btype!r} used by {name!r} is not defined')
        spec = types[btype]
        z = float(spec['z_center'])
        _check_lidar_visibility(name, spec, z, plane_z, margin)
        _static_visual_model(world, name, (x, y, z, 0, 0, 0), spec,
                             spec['color'], with_collision)

    for wall in course.get('walls') or []:
        yaw = math.radians(float(wall['yaw_deg']))
        height = float(wall['height'])
        spec = {
            'shape': 'box',
            'size': [float(wall['length']), float(wall['thickness']), height],
        }
        name = f"wall_{wall['name']}"
        _check_lidar_visibility(name, spec, height / 2.0, plane_z, margin)
        _static_visual_model(
            world, name,
            (float(wall['x']), float(wall['y']), height / 2.0, 0, 0, yaw),
            spec, (0.55, 0.50, 0.45), with_collision)

    return sdf


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    here = Path(__file__).resolve().parent
    parser.add_argument('--course', type=Path,
                        default=here.parent / 'config' / 'course_default.yaml')
    parser.add_argument('--output', type=Path,
                        default=here.parent / 'worlds' / 'roboboat_course.sdf')
    parser.add_argument('--cameras', action='store_true',
                        help='include the camera sensors from the course file')
    args = parser.parse_args(argv)

    course = yaml.safe_load(args.course.read_text())
    sdf = build_world(course, cameras=args.cameras)

    raw = ET.tostring(sdf, encoding='unicode')
    pretty = minidom.parseString(raw).toprettyxml(indent=INDENT)
    # minidom emits a bare <?xml?> header plus blank lines; tidy both.
    lines = [ln for ln in pretty.splitlines() if ln.strip()]
    banner = (f'<!-- GENERATED by gen_course.py from {args.course.name}. '
              'Edit the YAML, not this file. -->')
    lines.insert(1, banner)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text('\n'.join(lines) + '\n')
    print(f'wrote {args.output} ({len(lines)} lines)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
