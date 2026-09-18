# Nav2 behavior trees in this project

Nav2 behavior trees (BTs) are XML programs executed by `bt_navigator` for each
navigation action. They decide **when** to plan, control, retry, recover, or
switch a Nav2 plugin; planners and MPPI decide **how** to generate a path and
velocity command. A BT does not replace the controller or add vehicle physics.

This project uses rolling costmaps and ground-truth odometry; see
[Architecture](architecture.md). MPPI limits and costmap behavior live in
[`config/nav2_mppi.yaml`](../src/roboboat_bringup/config/nav2_mppi.yaml) and
are explained in [MPPI tuning](mppi_tuning.md).

## What is installed

| Action | Default tree | File | Purpose |
| --- | --- | --- | --- |
| `NavigateToPose` | `navigate_to_pose_boat.xml` | `src/roboboat_bringup/behavior_trees/` | One pose: plan at 1 Hz, follow it, then recover |
| `NavigateThroughPoses` | `navigate_through_poses_boat.xml` | same | Ordered task/channel waypoints |
| Sprint task | selected per action | `navigate_sprint_boat.xml` | Through-poses variant with a smaller passed-goal radius |

`nav2.launch.py` passes the first two paths to `bt_navigator`. A task client may
override them per action through the action goal's `behavior_tree` field;
`tools/task_run.py` does this for the sprint. That means one running Nav2 stack
can serve task-specific logic without a second parameter file or relaunch.

## Tree shape

All shipped trees follow this pattern:

```text
RecoveryNode (up to 6 retries)
└─ PipelineSequence
   ├─ plan at 1 Hz (and, for pose lists, remove completed goals)
   └─ FollowPath with the selected controller
on a planning/control failure → clear relevant costmap
after a failed attempt        → clear both maps, BackUp/Wait/Spin in rotation
```

`PipelineSequence` lets planning refresh while `FollowPath` is executing.
`RecoveryNode` retries its child after its recovery branch succeeds.
`RoundRobin` rotates recoveries rather than repeatedly applying the first one.
These are built-in Jazzy Nav2 BT nodes, so the current trees need no custom
plugin library.

## Boat-specific choices

- Replanning is 1 Hz. The course changes slowly, but gate navigation should
  not follow a several-second-old path.
- Channel gate centres lie inside the inflated obstacle field. The through-poses
  tree therefore uses `RemovePassedGoals radius="1.6"`: it retires a goal once
  the hull reaches its safe stand-off distance, rather than demanding passage
  through a buoy gate centre.
- Sprint ring waypoints are close together, so its tree uses `0.8 m` to avoid
  removing two waypoints at once and cutting the circle short.
- The one-pose recovery ordering is intentionally boat-safe: clear costmaps,
  back up, wait for the vessel to settle, then perform a small spin. The
  through-poses trees retain Nav2's broader recovery sequence.

## Adding a task tree

1. Copy the closest XML tree in
   `src/roboboat_bringup/behavior_trees/`; keep the BTCPP v4 root and existing
   action ports unless the Nav2 node documentation says otherwise.
2. Change only task policy: a goal-removal radius, recovery order, controller
   selection, or a task-specific condition/action. Keep shared vehicle facts in
   `nav2_mppi.yaml`, not duplicated XML.
3. For a built-in node, no code change is needed. For a custom condition or
   action, write and install a BT plugin, add its library to bt_navigator's
   plugin configuration, and make its ROS interfaces explicit.
4. Select the tree with the `behavior_tree` action-goal field (preferred for a
   task), or set `bt_xml:=...` / `through_bt_xml:=...` at launch when changing a
   stack-wide default.
5. Test a nominal run and each recovery path. Use a headless task trial for
   metrics; GUI load can affect timing. See [Task set](tasks.md) and the
   workspace README's testing section.

The practical rule is: use BTs for orchestration and task semantics; use Nav2
plugins/parameters for planning, control, and cost evaluation. The task guide
shows where controller selection belongs when a task needs a different driving
style.
