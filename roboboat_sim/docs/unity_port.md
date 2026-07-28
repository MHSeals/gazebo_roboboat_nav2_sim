# Porting this Nav2 config to the 1:1 Unity sim

The whole point of this workspace is that the Nav2 side should move across
unchanged. That holds only if Unity presents the same interface Gazebo does
here.

## The contract

Hold these fixed and `nav2_mppi.yaml` transfers as-is.

| Topic | Type | Direction | Notes |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | sim → ROS | Everything runs `use_sim_time:=true` |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 → boat | **Body frame.** `TwistStamped` on Kilted+ |
| `/odom` | `nav_msgs/Odometry` | sim → ROS | `odom` → `base_link`, twist in body frame |
| `/tf` | `tf2_msgs/TFMessage` | sim → ROS | Must carry `odom` → `base_link` |
| `/scan` | `sensor_msgs/LaserScan` | sim → ROS | Or `PointCloud2`; see below |
| `/imu/data` | `sensor_msgs/Imu` | sim → ROS | Unused by Nav2 here; needed once you add an EKF |
| `/navsat/fix` | `sensor_msgs/NavSatFix` | sim → ROS | Same |

Frames: `map → odom → base_link → sensors`, REP-103 (x forward, y **port**,
z up, yaw CCW). Unity is left-handed and Y-up; the conversion has to happen at
the bridge, once, and be tested — a sign error in the sway axis will look like
a badly tuned controller for a long time before you find it.

## What moves, and what does not

**Moves unchanged:** `nav2_mppi.yaml`, `navigate_to_pose_boat.xml`,
`roboboat_control/` in its entirety (it is pure Python and imports nothing from
Gazebo), `tools/validate.py`.

**Does not move:** the world SDF, the URDF's `<gazebo>` blocks, and
`gz_bridge.yaml`. Unity supplies its own scene and bridge.

**Needs re-derivation:** the velocity and acceleration limits. They are
properties of the *vehicle*, not the simulator. Two ways to get them for the
real boat:

1. Measure. Command full thrust in surge, sway and yaw on the water, record
   the terminal velocities, put them in `boat_params.yaml`, and read the
   envelope the node prints.
2. Compute. Update the thruster and damping coefficients in `boat_params.yaml`
   from the real hull, then read the same log line.

Either way `tools/validate.py` will fail until `nav2_mppi.yaml` agrees.

## Where the dynamics should live

Two options, and the choice matters more than it looks:

**Unity owns the physics.** Unity applies thruster forces to a rigidbody and
publishes the resulting odometry, and `surrogate_dynamics` is dropped. More
faithful, and the right end state. But you now have two implementations of the
boat's behaviour, and if Unity's differs from the model MPPI was tuned against,
you will be re-tuning without knowing why.

**Keep `surrogate_dynamics` as the reference.** Unity renders and provides
perception; the Python model still owns the vehicle response. Less faithful,
but it means "the boat behaved differently" can only mean something you
changed on purpose.

Recommended: start with the second, so the Unity bring-up is only testing the
bridge and perception. Move to the first once navigation is trusted, and keep
`surrogate_dynamics` around as an oracle to diff against.

## Lidar

If the Unity sim gives you a 3D `PointCloud2` rather than a `LaserScan`,
nothing in the MPPI block changes — only the costmap observation source. In
both costmaps replace the `scan` source with:

```yaml
observation_sources: cloud
cloud:
  topic: /points
  data_type: "PointCloud2"
  clearing: True
  marking: True
  min_obstacle_height: 0.10
  max_obstacle_height: 1.50
  raytrace_max_range: 20.0
  obstacle_max_range: 18.0
```

Set `min_obstacle_height` above the water surface or every wave return becomes
an obstacle. This is the single most likely thing to go wrong in the port, and
it presents as the boat refusing to move with a costmap that looks solid red.

Consider `nav2_costmap_2d`'s `VoxelLayer` instead of `ObstacleLayer` if you
need proper 3D clearing — that is the case the voxel grid is actually for, and
the reason it is *not* used in this workspace.

## Suggested order

1. Bridge `/clock`, `/tf`, `/odom` only. Verify TF is sane in RViz with the
   boat driven by hand. Do not proceed until this is boring.
2. Add `/cmd_vel` and drive it manually. Verify sway sign and yaw sign
   explicitly — command pure +y and confirm the boat moves to **port**.
3. Add perception, and check the costmap against a known buoy layout before
   trusting it.
4. Only then start Nav2, with this config, and expect it to mostly work.
