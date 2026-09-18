# Getting started

This guide gets a fresh checkout to a reproducible baseline and points each
kind of change at the component that owns it. It assumes the supported path:
rootless Podman on Linux. Install instructions and GUI prerequisites are in the
[repository README](../../README.md).

## 1. Build the container and workspace

From the repository root, build the image, then build the mounted colcon
workspace inside it. The image contains ROS dependencies; the second command
creates the workspace's `build/`, `install/`, and `log/` outputs.

```bash
container/rb.sh build
container/rb.sh run 'cd /ws && bash tools/setup_ws.sh --build'
```

For a host-native setup instead, use the package and colcon instructions in
the [workspace README](../README.md). In either case, source the environment
before issuing `ros2` commands manually:

```bash
. tools/env.sh
```

It sources ROS Jazzy and the local overlay when it exists. On a non-Ubuntu host
it also handles the common case where the default Python does not match the
Python ABI used by ROS.

## 2. Establish a baseline

First run the offline checks. They catch configuration disagreements without
requiring ROS, Gazebo, or a GPU:

```bash
container/rb.sh run 'cd /ws && bash tools/setup_ws.sh --check'
```

Then run the scored, headless channel task:

```bash
container/rb.sh run 'cd /ws && bash tools/task_trial.sh channel baseline'
```

This starts and tears down its own stack and writes
`roboboat_sim/tuning/baseline_{trace,card}.json`. Treat it as the baseline for
changes affecting behavior or performance. GUI runs are useful for inspection,
but renderer load can alter timing and trajectories, so they are not a
benchmark.

For lower-level diagnosis, use the private-stack tests:

```bash
container/rb.sh run 'cd /ws && bash tools/run_test.sh smoke'
container/rb.sh run 'cd /ws && bash tools/run_test.sh nav'
```

Run `container/rb.sh help`, `bash tools/setup_ws.sh --help`, and the relevant
tool's `--help` before inventing flags.

## 3. Run an interactive stack

Use two terminals only for the watchable workflow:

```bash
# terminal 1
container/rb.sh gui

# terminal 2
container/rb.sh tasks
container/rb.sh task channel
```

`task` connects to the container launched by `gui`; it does not launch another
stack. Do not start a second Gazebo/Nav2 container while one is running: host
networking would mix `/clock`, TF, and discovery traffic. Use `podman exec` as
shown by `container/rb.sh help` for a second shell in the existing container.

## 4. Choose the correct ownership boundary

| If you are changing… | Start here | Verify with… |
| --- | --- | --- |
| Course geometry, robot description, Gazebo bridge, frames | [Architecture](architecture.md), `roboboat_description/` | `validate.py`, smoke test |
| Thrust layout, drag, lag, or achievable velocity | `roboboat_control/`, [MPPI tuning](mppi_tuning.md) | unit tests, `validate.py`, smoke/task test |
| Paths, costmaps, MPPI critics, limits, goal checking | `roboboat_bringup/config/nav2_mppi.yaml`, [MPPI tuning](mppi_tuning.md) | validation, navigation test, headless task trial |
| Task waypoints, scoring, or recovery/task policy | [Task set](tasks.md), [Behavior trees](behavior_trees.md) | task scoring tests and the affected trial |
| ROS interface parity with Unity | [Unity port](unity_port.md) | frame/topic checks before Nav2 |

Keep these boundaries explicit. Gazebo provides world, lidar, clock, and
ground-truth odometry; `roboboat_control` provides the limited boat response;
Nav2 provides planning and control; behavior trees orchestrate actions and
recoveries. [Architecture](architecture.md) describes the full topic and frame
contract.

## 5. Work and verify incrementally

- Source edits are mounted into the container immediately, but changes to ROS
  packages require a rebuild: `container/rb.sh run 'cd /ws && colcon build
  --symlink-install'`.
- Regenerate the world after changing the course declaration:
  `cd roboboat_sim/src/roboboat_description && python3 scripts/gen_course.py`.
- Re-read the dynamics node's reported velocity envelope after changing boat
  parameters, then keep MPPI limits at or below it.
- Save task trials under a meaningful `tuning/` label and consult
  `tuning/JOURNAL.md` when comparing with historical experiments.

See the [documentation map](README.md) for deeper references. The major
operational hazards—leftover Gazebo servers, ROS Python version skew, and slow
first GPU-lidar scan under software rendering—are documented in the workspace
README's **Gotchas** section.
