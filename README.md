# RoboBoat Nav2 / Gazebo demo

This repository contains a ROS 2 workspace (`roboboat_sim/`) and a Podman
container wrapper (`container/`) for testing Nav2 MPPI on an omnidirectional
RoboBoat-style catamaran. Gazebo supplies the world, lidar, odometry and clock;
the workspace supplies the boat's thrust-limited 3-DOF response and Nav2.

Start with the workspace guide: [`roboboat_sim/README.md`](roboboat_sim/README.md).
It covers native ROS installation, validation, launch modes, testing, and the
simulation's intentional shortcuts. The documentation map is
[`roboboat_sim/docs/README.md`](roboboat_sim/docs/README.md).

## Container setup

The supported runtime is **rootless Podman on Linux**. Install it using the
[official Podman installation guide](https://podman.io/docs/installation), then
confirm `podman info` succeeds as your normal user. On Ubuntu 24.04, the
distribution package is usually sufficient:

```bash
sudo apt update && sudo apt install podman
podman info
```

Docker is not the supported path: `container/rb.sh` invokes Podman directly and
relies on its rootless behavior. Docker users can adapt the `Containerfile`,
but should expect to translate runtime flags and validate ROS 2 discovery,
shared memory, and GUI access themselves.

For GUI runs, also use a local graphical Linux session with a usable `DISPLAY`.
The default software renderer is intentional and works without GPU passthrough;
see `container/rb.sh help` before setting `RB_GPU=1`.

From this repository root:

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

## A good first direction for new contributors

1. Establish the baseline: build the image, run `container/rb.sh help`, then
   run the headless channel trial. Do not tune from a GUI-observed run.
2. Learn the boundaries in [Architecture](roboboat_sim/docs/architecture.md):
   Gazebo owns sensors/world/ground-truth odometry; `roboboat_control` owns
   achievable boat response; Nav2 owns planning and control.
3. Choose one layer before editing: course/URDF and bridge, boat parameters,
   Nav2 MPPI configuration, or task/behavior-tree policy. Keep vehicle limits
   and task semantics out of each other's files.
4. Run the cheapest relevant check first (`tools/validate.py`, unit tests, then
   smoke/navigation/task trials). Use [the docs map](roboboat_sim/docs/README.md)
   to find the owner and test for each subsystem.

The `tuning/` archive records prior experiments; consult its journal when
comparing results, not as a prerequisite to getting the baseline running.

## Repository layout

| Path | Purpose |
| --- | --- |
| `roboboat_sim/src/roboboat_description` | Boat URDF, course declarations/worlds, Gazebo bridge, RViz |
| `roboboat_sim/src/roboboat_control` | Thruster allocation and surrogate vessel dynamics |
| `roboboat_sim/src/roboboat_bringup` | ROS launch files, Nav2/MPPI configuration, behavior trees |
| `roboboat_sim/tools` | Validation, task execution, regression, recording and replay |
| `container` | Containerfile and `rb.sh` operator wrapper |

Target platform: ROS 2 Jazzy, Gazebo Harmonic (`gz-sim8`), Ubuntu 24.04.
