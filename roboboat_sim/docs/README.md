# Documentation map

Use this page as the entry point after the [workspace README](../README.md).
The simulation is deliberately split into a visual/sensor world, a lightweight
boat-response model, and Nav2. That boundary is the key to understanding both
the demo and its Unity port.

| Need | Read |
| --- | --- |
| Run, build, validate, test, or diagnose startup | [Workspace README](../README.md) |
| Understand processes, topics, frames, and ownership | [Architecture](architecture.md) |
| Tune the Nav2 MPPI controller without exceeding boat authority | [MPPI tuning](mppi_tuning.md) |
| Understand or change Nav2 recovery/task logic | [Behavior trees](behavior_trees.md) |
| Run or extend competition-style tasks and scoring | [Task set](tasks.md) |
| Historical multi-task design rationale | [Multi-task plan](task_plan.md) |
| Preserve the ROS contract when moving to Unity | [Unity port](unity_port.md) |
| Revisit the thruster-layout experiment | [Thruster handoff](handoff_thrusters.md) |

## Recommended reading paths

- **New operator:** workspace README → Architecture → container `rb.sh help`.
- **Controller/tuning work:** Architecture → MPPI tuning → Behavior trees →
  `python3 tools/validate.py` and the live tests in the workspace README.
- **New task:** Task set → Behavior trees → `tools/task_run.py --help` →
  task-specific regression.
- **Unity integration:** Unity port → Architecture → MPPI tuning.

`tuning/` is an experiment ledger and result archive, not required reading for
first use. Read `tuning/JOURNAL.md` when reproducing or comparing a historical
result.
