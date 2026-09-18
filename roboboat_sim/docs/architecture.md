# Architecture

For setup and smoke tests, start with the [workspace README](../README.md).
This page explains the boundaries that must remain stable when tuning Nav2,
adding tasks, or moving to Unity. Behavior-tree orchestration is documented
separately in [Behavior trees](behavior_trees.md).

## Process and topic graph

```
                    ┌──────────────────────────────────────────┐
                    │ gz sim  (roboboat_course.sdf)            │
                    │   VelocityControl   ← /model/roboboat/cmd_vel
                    │   OdometryPublisher → /model/roboboat/odometry
                    │                     → /model/roboboat/pose
                    │   GPU lidar         → /lidar/scan        │
                    │   IMU, NavSat       → /imu/data, /navsat/fix
                    └───────────────┬──────────────────────────┘
                                    │ gz transport
                    ┌───────────────┴──────────────────────────┐
                    │ ros_gz_bridge  (config/gz_bridge.yaml)   │
                    └───────────────┬──────────────────────────┘
      /boat/cmd_vel_applied ────────┤ /clock /odom /tf /scan /imu/data /navsat/fix
                    ▲               ▼
      ┌─────────────┴──────┐   ┌────────────────────────────────┐
      │ surrogate_dynamics │   │ Nav2                           │
      │  allocation        │   │  controller_server (MPPI)      │
      │  thruster lag      │◄──┤  planner_server (Smac 2D)      │
      │  3-DOF integration │   │  bt_navigator, behavior_server │
      └────────────────────┘   │  local + global costmaps       │
             /cmd_vel          └────────────────────────────────┘
                                            ▲
                                robot_state_publisher → /tf (sensor frames)
                                static_transform_publisher → map → odom
```

## Who owns what

| Concern | Owner | Notes |
| --- | --- | --- |
| Rigid-body motion | `surrogate_dynamics` | Gazebo integrates nothing |
| Thruster limits | `surrogate_dynamics` | Saturation is applied *before* the wrench is integrated |
| Pose / odometry | Gazebo `OdometryPublisher` | Ground truth; `dimensions: 2` |
| `odom → base_link` | Gazebo, via the bridge | Bridged from `gz.msgs.Pose_V` to `/tf` |
| `base_link → sensors` | `robot_state_publisher` | From the URDF's fixed joints |
| `map → odom` | `static_transform_publisher` | Identity; see "Frames" below |
| Obstacles | GPU lidar → costmap `ObstacleLayer` | Single plane at z = 0.35 m |

## Frames

`map → odom → base_link → {lidar_link, imu_link, navsat_link, thruster_*_link}`

`map → odom` is a **static identity transform**. Open water has no features to
localise against and no prior map, so the map frame is simply wherever the boat
booted. Both costmaps roll with the vehicle, so nothing downstream needs a
global map to exist.

When you add a GPS/IMU state estimator (`robot_localization`'s
`navsat_transform_node` is the usual choice), it takes over that link and you
launch with `publish_map_odom_tf:=false`. Nothing else in the config changes —
that is the point of keeping the boundary here.

## Why one physical link

The URDF puts every visual and collision on `base_link`. The extra links exist
only so `robot_state_publisher` emits sensor TF; each Gazebo sensor overrides
its message `frame_id` with `<gz_frame_id>`.

This sidesteps sdformat's fixed-joint lumping entirely. When URDF is converted
to SDF, fixed joints are collapsed and links merged — a long-standing source of
sensors ending up at the wrong pose or attached to a link that no longer
exists. With nothing to lump, there is nothing to get wrong.

## Speed levers, in the order worth pulling

1. `headless:=true` — drops the Gazebo GUI's render pass. The lidar still
   renders offscreen, which is why the launch passes `--headless-rendering`
   rather than plain `-s`.
2. `real_time_factor` in `course_default.yaml` — set `0.0` to run unthrottled.
   Nav2 uses sim time, so it follows along, but its wall-clock compute becomes
   the bottleneck and the controller may not keep up. Useful for long soak
   runs, misleading for tuning.
3. `batch_size` / `time_steps` in the MPPI block — the dominant CPU cost in the
   whole system once rendering is off.
4. Lidar `<samples>` (640) and `<update_rate>` (15 Hz) — cut these before you
   cut MPPI's batch size; 640 rays is generous for buoy-sized obstacles.
5. `mode:=kinematic` — skips allocation and integration. Marginal on its own;
   its real value is diagnostic.

## Verified against upstream

Parameter and plugin names here were read out of the actual sources rather
than recalled, because a wrong name usually fails silently as a default:

- `gz::sim::systems::VelocityControl` — subscribes `/model/<name>/cmd_vel`
  when `<topic>` is unset; sets `components::LinearVelocityCmd`, which
  `Physics.cc` rotates by the model pose, so **the twist is body-frame**. The
  component is zeroed every step, which is why the plugin re-publishes it each
  `PreUpdate`.
- `gz::sim::systems::OdometryPublisher` — `odom_frame`, `robot_base_frame`,
  `odom_publish_frequency`, `dimensions`, `odom_topic`, `tf_topic`; defaults
  are `/model/<name>/odometry` and `/model/<name>/pose`.
- `<gz_frame_id>` — read in `gz-sensors8/src/Sensor.cc`; overrides the message
  `frame_id`.
- `nav2_util::TwistPublisher` — on Jazzy, `enable_stamped_cmd_vel` defaults to
  **false**, so `/cmd_vel` carries `geometry_msgs/Twist`. Kilted and newer are
  `TwistStamped`-only.
- `nav2_mppi_controller` — `ax_max`, `ax_min`, `ay_max`, `ay_min`, `az_max`
  all exist on Jazzy (they are absent from the README table but present in
  `optimizer.cpp`). `PathAngleCritic` takes `mode`, not the older
  `forward_preference`.
- `nav2_smac_planner::SmacPlanner2D` — `downsample_costmap`,
  `downsampling_factor`, `cost_travel_multiplier`, `terminal_checking_interval`,
  `use_final_approach_orientation`.

## Known gaps

- **Marginal clearance for goals inside inflated space.** Reproducible; see
  `docs/mppi_tuning.md`. Open-water navigation is clean.
- **Perception is abstracted away.** Buoys are modelled taller than reality so
  a single scan plane sees them. On the water this is a 3D lidar plus a camera
  classifier; its *output* is a set of obstacle points either way, which is all
  Nav2 consumes. Buoy colour is not exposed to the stack — red/green channel
  logic is a task-planner concern, not a Nav2 one.
- **Ground-truth odometry.** No drift, no GPS noise, no EKF. That is on
  purpose: it isolates controller behaviour from estimator behaviour. Add the
  estimator second, once MPPI is trusted.
- **No currents or wind.** A constant disturbance wrench is the natural next
  addition to `surrogate_dynamics` and would be a genuinely useful MPPI test.
