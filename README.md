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
confirm `podman info` succeeds as your normal user. The distribution package is
usually sufficient:

```bash
# Debian / Ubuntu
sudo apt update && sudo apt install podman

# Fedora / RHEL / Rocky / AlmaLinux (DNF-based releases)
sudo dnf install podman

# Arch Linux
sudo pacman -S podman

podman info
```

Use the package manager that matches the host; do not run all three blocks.
On RHEL systems without `dnf`, use the equivalent `yum install podman`. A
current distribution release is recommended because the project needs rootless
containers, host networking, and GUI socket mounts. The official guide covers
additional distributions and repository setup when the system package is old.

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

## Getting started and working in the repository

Follow the [getting-started guide](roboboat_sim/docs/getting_started.md) after
installing Podman. It covers the missing steps between image build and launch:
building the colcon workspace, establishing an offline and headless baseline,
interactive operation, source rebuilds, subsystem ownership, and the matching
verification path. It also distinguishes task results suitable for comparison
from GUI runs intended for observation.

## Repository layout

| Path | Purpose |
| --- | --- |
| `roboboat_sim/src/roboboat_description` | Boat URDF, course declarations/worlds, Gazebo bridge, RViz |
| `roboboat_sim/src/roboboat_control` | Thruster allocation and surrogate vessel dynamics |
| `roboboat_sim/src/roboboat_bringup` | ROS launch files, Nav2/MPPI configuration, behavior trees |
| `roboboat_sim/tools` | Validation, task execution, regression, recording and replay |
| `container` | Containerfile and `rb.sh` operator wrapper |

Target platform: ROS 2 Jazzy, Gazebo Harmonic (`gz-sim8`), Ubuntu 24.04.
