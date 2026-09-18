# roboboat_sim

A deliberately fake-physics Gazebo simulation of a RoboBoat omni-catamaran
(4 thrusters, X configuration), built as a testbed for a **Nav2 MPPI**
configuration that will later be ported to a 1:1 Unity simulator.

The navigation stack is the product. The boat is a fixture.

**Start here:** this is a colcon workspace, not a standalone Python project.
On a native ROS machine, follow [Quick start](#quick-start); for the supported
Podman workflow, begin at the [repository README](../README.md). The
[documentation map](docs/README.md) links architecture, tuning, task, Unity,
and behavior-tree material.

---

## The trade this workspace makes

Water physics is the most expensive thing in a marine simulator and the least
relevant to whether MPPI can follow a path through a buoy field. So there is
none:

| Normally | Here | Why |
| --- | --- | --- |
| Buoyancy + hydrodynamics plugins | `<gravity>false</gravity>` on the hull | The boat cannot sink, heel, or bounce, so no solver work and no settling transients |
| Physics-driven thrusters | Gazebo `VelocityControl`, fed a body twist | Gazebo integrates nothing; it moves the model rigidly |
| Wave field / ocean shader | One flat coloured plane | Rendering cost is dominated by the lidar, not the water |
| Collision meshes on every buoy | Visual-only static models | The GPU lidar rasterises *visuals*; collision geometry costs broadphase time and buys nothing |
| Physics at 1 kHz | 5 ms step, shadows off | Nothing in the world has contact or constraint dynamics |

What is **not** faked is the part a trajectory controller actually feels.
`roboboat_control` runs a real 3-DOF vessel model in ~200 lines of Python:
separate surge/sway/yaw inertia, linear + quadratic drag, rigid-body Coriolis
coupling, thrust allocation across 4 thrusters with hard saturation, and
first-order thruster lag. That is deliberate — a controller tuned against a
boat with infinite thrust and no sideways drag will not transfer to anything,
Unity included.

```
/cmd_vel ──► velocity controller ──► thrust allocation ──► thruster lag
   (Nav2)         (wrench)             (4 forces, saturated)      │
                                                                  ▼
Gazebo ◄── /boat/cmd_vel_applied ◄── 3-DOF integration ◄── achievable wrench
  │
  └─► ground-truth odom + TF, GPU lidar ──► Nav2 costmaps
```

## Layout

```
src/roboboat_description/   URDF/xacro, world generator, bridge config, RViz
src/roboboat_control/       Thrust allocation + surrogate dynamics (the physics)
src/roboboat_bringup/       Launch files, Nav2 MPPI params, behavior tree
tools/validate.py           Offline cross-file consistency checks
```

Target platform: **ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8) on Ubuntu 24.04**.
Every Nav2 and Gazebo parameter name in here was checked against the `jazzy` /
`gz-sim8` sources rather than recalled.

---

## Quick start

### Offline checks (no ROS, Gazebo, or GPU)

Everything below runs on plain Python — no ROS, no Gazebo, no GPU. It is
enough to keep developing the parts that matter.

```bash
pip install numpy pytest pyyaml xacro

# 1. Cross-file consistency: does the whole workspace still hang together?
python3 tools/validate.py

# 2. The physics and allocation model, properly unit tested
cd src/roboboat_control && python3 -m pytest test -q
```

`tools/validate.py` is the useful one. It catches the class of mistake that
otherwise costs a full build-and-launch cycle to find:

- Nav2 asking MPPI to sample velocities the thrusters cannot produce
- the MPPI prediction horizon overrunning the local costmap
- a critic listed in `critics:` with no parameter block (silent default)
- thruster geometry drifting apart between the URDF and `boat_params.yaml`
- buoys that the single-plane lidar would pass straight over — a world that
  looks perfect in Gazebo and is completely empty to Nav2
- costmap inflation radius smaller than the hull's inscribed radius, leaving
  no cost gradient to follow

Editing the course is also laptop-free. `config/course_default.yaml` describes
gates, buoys and dock walls; regenerate the world with:

```bash
cd src/roboboat_description && python3 scripts/gen_course.py
```

The generator refuses to emit a world containing an obstacle the scan plane
would miss.

---

### Native ROS + Gazebo setup

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-navigation2 \
                 ros-jazzy-nav2-bringup ros-jazzy-ros-gz \
                 ros-jazzy-nav2-mppi-controller ros-jazzy-nav2-smac-planner

cd roboboat_sim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Bring it up in the order that isolates failures:

```bash
# 1. Boat only. Does it move sensibly?
ros2 launch roboboat_bringup sim.launch.py
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 0.2}}'

# 2. Everything, with RViz. Send goals with the "2D Goal Pose" tool.
ros2 launch roboboat_bringup boat_nav.launch.py

# 3. No GUI, for iterating on tuning quickly
ros2 launch roboboat_bringup boat_nav.launch.py headless:=true rviz:=false
```

Useful arguments: `mode:=kinematic` (bypass the thruster model entirely, to
tell "MPPI is misconfigured" apart from "the boat cannot do that"),
`use_velocity_smoother:=true`, `nav2_params:=<your file>`.

Discover every launch argument and its description with:

```bash
ros2 launch roboboat_bringup sim.launch.py --show-args
ros2 launch roboboat_bringup boat_nav.launch.py --show-args
ros2 launch roboboat_bringup nav2.launch.py --show-args
python3 tools/task_run.py --help
```

On startup the dynamics node logs the boat's achievable envelope:

```
steady-state envelope: vx=1.63 m/s vy=0.61 m/s wz=0.71 rad/s
```

Those are the ceilings for `vx_max` / `vy_max` / `wz_max` in
`config/nav2_mppi.yaml`. Change the thrusters, re-read that line, update the
config — `tools/validate.py` enforces the relationship.

## Testing

Three layers, cheapest first.

```bash
python3 tools/validate.py                 # offline, no ROS: cross-file invariants
cd src/roboboat_control && python3 -m pytest test -q    # allocation + dynamics
bash tools/run_test.sh smoke              # live sim, boat only
bash tools/run_test.sh nav                # live sim, full Nav2 stack
bash tools/run_test.sh nav --goals "14,1,0.2 30,8,0.7 36,22,1.6"
```

`run_test.sh` brings the stack up, runs the test, and tears everything down.
The teardown is not incidental — see "Gotchas" below.

`make_replay.py` turns a recorded run into a single self-contained HTML page
— fonts subset and inlined, trace embedded, no external requests — so a run can
be reviewed from a phone or attached to a PR without a server:

```bash
python3 tools/nav_test.py --record run.json    # while the sim is up
python3 tools/make_replay.py --record run.json --out replay.html
```

It gives you a scrubbable top-down chart with the boat's footprint, the active
global plan, lidar returns, thruster forces, and a clearance ribbon under the
scrubber so the tight passes are visible before you go looking for them.

### Seeing it as Gazebo draws it

The replay page above is **not** Gazebo's renderer — it is a chart drawn from
recorded telemetry. When you want Gazebo's own pixels, the course file defines
camera sensors and the boat has an optional follow camera:

```bash
# one-off: a world that includes the overhead and hero cameras
python3 src/roboboat_description/scripts/gen_course.py --cameras \
  --output src/roboboat_description/worlds/roboboat_course_cameras.sdf

ros2 launch roboboat_bringup boat_nav.launch.py headless:=true rviz:=false \
    chase_camera:=true
python3 tools/capture_frames.py --out frames/ --cameras chase
ffmpeg -framerate 24 -i frames/chase_%04d.png -pix_fmt yuv420p chase.mp4
```

Cameras are off in the default world and off on the boat because rendering one
costs far more than the lidar and nothing in the navigation stack consumes it.
Under software rendering the first render takes ~30 s (real-time factor dips
to ~0.1 while it initialises, then recovers to 1.0); with a GPU this is not
noticeable.

`smoke_test.py` checks what static analysis cannot: that the bridge is wired,
that sensor frames survived URDF → SDF, that the sign conventions are right,
and that thrust saturation actually binds. `nav_test.py` drives real goals and
measures clearance to every obstacle using the same footprint polygon Nav2
uses, so "reached the goal by driving through a buoy" cannot pass.

### Measured results

Run on 4 cores with **software rendering** (`LIBGL_ALWAYS_SOFTWARE=1`, no GPU):

```
[ok] scan frame_id is lidar_link                  got 'lidar_link'
[ok] lidar returns hits from the course           53/640 rays, nearest 5.99 m
[ok] surge tracks vx=1.0                          vx=1.00 dx=9.54 m
[ok] +vy moves the boat to port (+y)              dy=2.91 m dx=-0.00 m
[ok] +wz yaws counter-clockwise                   dyaw=130.1 deg
[ok] unreachable command saturates near 1.63 m/s  vx=1.63
[ok] boat stops when /cmd_vel goes quiet          vx=0.000

[ok] goal (20.0, 6.0)    reached in 55s, 0.14 m from goal
[ok] goal (34.0, 20.0)   reached in 53s, 0.22 m from goal
[ok] no contact with any obstacle   min clearance 0.33 m (33 obstacles)
[ok] controller output rate         20.0 Hz over 108s (configured 20 Hz)
```

Real-time factor holds at **1.0** and MPPI sustains its full **20 Hz** with
2000 rollouts × 56 steps — on four cores, with no GPU, with Gazebo running.
That is the physics-free trade paying off. Zero `[ERROR]` lines in the stack.

### Gotchas this environment surfaced

Four things cost real time, and all four will happen again on your laptop:

1. **`gz sim` is a ruby wrapper.** `pkill -f "gz sim"` kills the wrapper and
   leaves `gz-sim-server` running. The next launch then talks to *both*
   servers: two publishers on `/model/roboboat/odometry`, two authorities for
   `odom → base_link`, and tf2 rejecting the conflict with `Failure to set
   received transform`. It looks exactly like a broken TF config and is not.
   `run_test.sh` uses `setsid` and kills the whole process group.
2. **Python version skew.** ROS Jazzy is built for python3.12. If your
   `python3` is anything else you get `No module named
   'rclpy._rclpy_pybind11'`. `tools/env.sh` shims it, including for `ros2` and
   `colcon`, which carry a hardcoded `#!/usr/bin/python3`.
3. **`set -u` and ROS.** `setup.bash` dereferences unset variables and dies
   instantly under `set -u`. Any wrapper script must not use it.
4. **Software rendering is slow to start, not hung.** The GPU lidar needs a GL
   context even headless; first scan can take ~60 s to appear. With a real GPU
   this is seconds.

## Documentation

- [`docs/README.md`](docs/README.md) — documentation map and reading paths
- [`docs/architecture.md`](docs/architecture.md) — topic graph, frames, what
  each process owns, and what was verified against upstream source
- [`docs/mppi_tuning.md`](docs/mppi_tuning.md) — why each non-default MPPI
  parameter is what it is, and the order to tune them in
- [`docs/behavior_trees.md`](docs/behavior_trees.md) — Nav2 task/recovery
  orchestration, shipped trees, and how to add one
- [`docs/tasks.md`](docs/tasks.md) — competition task mapping, scoring, and
  when a task needs a tree versus a plugin/configuration change
- [`docs/unity_port.md`](docs/unity_port.md) — the interface contract to hold
  fixed so this config drops onto the 1:1 sim

## Status

Built and run end to end on ROS 2 Jazzy + Gazebo Harmonic: `colcon build`
clean, 12 offline checks, 20 unit tests, 8 live smoke checks, and Nav2 driving
the boat to goals through the buoy course with no collisions and no errors.

One open item. With an aggressive goal placed 2.26 m from a gate buoy — inside
the 1.30 m inflation ring — the boat threads the gap with only ~0.15 m
clearance, and that straddles contact: two identical runs measured −0.17 m and
+0.14 m. Goals in open water are comfortable (0.33 m). Before trusting this
near obstacles, either keep goals out of inflated space or raise
`footprint_padding` / `CostCritic.cost_weight`. See `docs/mppi_tuning.md`.
