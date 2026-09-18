# RoboBoat Nav2 / Gazebo demo

This repository contains a ROS 2 workspace (`roboboat_sim/`) and a Podman
container wrapper (`container/`) for testing Nav2 MPPI on an omnidirectional
RoboBoat-style catamaran. Gazebo supplies the world, lidar, odometry and clock;
the workspace supplies the boat's thrust-limited 3-DOF response and Nav2.

Start with the workspace guide: [`roboboat_sim/README.md`](roboboat_sim/README.md).
It covers native ROS installation, validation, launch modes, testing, and the
simulation's intentional shortcuts. The documentation map is
[`roboboat_sim/docs/README.md`](roboboat_sim/docs/README.md).

## Container quick start

The supported container runtime is rootless Podman. Run these from this
repository root:

```bash
container/rb.sh help                 # commands, prerequisites, and examples
container/rb.sh build                # build the ROS 2 Jazzy image
container/rb.sh gui                  # Gazebo + RViz + Nav2
container/rb.sh tasks                # in another terminal: list declared tasks
container/rb.sh task channel         # drive a task on that visible stack
```

For a repeatable, headless measurement rather than an observed GUI run:

```bash
container/rb.sh run 'cd /ws && bash tools/task_trial.sh channel demo'
```

`gui` and `task` intentionally share one running container. Do not start a
second stack on the host network: competing simulation clocks and TF publishers
make Nav2 appear broken. `container/rb.sh help` explains the safe alternatives.

## Repository layout

| Path | Purpose |
| --- | --- |
| `roboboat_sim/src/roboboat_description` | Boat URDF, course declarations/worlds, Gazebo bridge, RViz |
| `roboboat_sim/src/roboboat_control` | Thruster allocation and surrogate vessel dynamics |
| `roboboat_sim/src/roboboat_bringup` | ROS launch files, Nav2/MPPI configuration, behavior trees |
| `roboboat_sim/tools` | Validation, task execution, regression, recording and replay |
| `container` | Containerfile and `rb.sh` operator wrapper |

Target platform: ROS 2 Jazzy, Gazebo Harmonic (`gz-sim8`), Ubuntu 24.04.
