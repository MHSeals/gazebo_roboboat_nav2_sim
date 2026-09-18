# Tuning MPPI for the boat

Everything below refers to `src/roboboat_bringup/config/nav2_mppi.yaml`.
Read [Architecture](architecture.md) first for frames and data ownership, and
[Behavior trees](behavior_trees.md) for the separate policy that invokes MPPI
and handles recoveries. Validate each parameter edit with the workspace
README's offline and live test sequence.

## Non-default parameters, and why

| Parameter | Stock | Here | Reason |
| --- | --- | --- | --- |
| `motion_model` | `DiffDrive` | `Omni` | 4 thrusters in X give independent surge/sway/yaw |
| `min_y_velocity_threshold` | `0.5` | `0.001` | **The one that silently breaks holonomic robots.** Nav2 zeroes odometry velocities below this. At 0.5 m/s it erases almost the boat's entire sway range before MPPI ever sees it |
| `vx_max` / `vy_max` / `wz_max` | 0.5 / 0.5 / 1.9 | 1.30 / 0.45 / 0.55 | Derived from the thrust envelope, not chosen. See below |
| `ax_max` … `az_max` | 3.0 / 3.5 | 1.5 / 0.8 / 0.8 | Peak thrust ÷ surge mass is ~2.6 m/s², but thruster lag and the inner velocity loop mean it is not sustained |
| `PathAlignCritic.offset_from_furthest` | 20 | 6 | The stock value assumes ~5 cm path resolution. Ours is 20 cm (the global costmap's), so 20 would essentially never fire |
| `GoalCritic.threshold_to_consider` | 1.4 | 2.5 | Matched to `PathFollowCritic` for a clean hand-off at this speed |
| `CostCritic.consider_footprint` | false | true | The hull is 1.5 × 1.1 m. Point-cost checking would let a corner clip a buoy mid-crab |
| `TwirlingCritic` | off | on, weight 5 | Omni vehicles will spin while translating if nothing objects |
| `inflation_radius` | 0.7 | 1.30 | Inscribed radius is 0.55 m; ~2.4× gives a gradient to ride without walling off a 3 m gate |
| `progress_checker.movement_time_allowance` | 10 s | 20 s | A boat under thrust saturation accelerates slowly; 10 s trips spurious recoveries |
| `goal_checker` tolerances | 0.25 / 0.25 | 0.50 / 0.30 | Station-keeping tolerance, not AMR docking tolerance |

## Setting the velocity limits honestly

Do not guess these. `surrogate_dynamics` prints the achievable envelope on
startup:

```
peak wrench Fx=99.0 N Fy=70.7 N Mz=10.61 N.m
steady-state envelope: vx=1.63 m/s vy=0.61 m/s wz=0.71 rad/s
```

`vx_max`/`vy_max`/`wz_max` must sit **at or below** those. MPPI sampling
trajectories the boat cannot execute is the single most common cause of "it
works in sim, it wanders on the water": the optimiser keeps selecting rollouts
built on velocities that never materialise, so tracking error accumulates
faster than replanning corrects it.

`tools/validate.py` enforces the relationship. Change the thrusters and it
fails until you update the Nav2 config.

Note that sway authority is ~40% of surge. That asymmetry is real — a
catamaran resists sideways motion hard — and MPPI needs to know about it or it
will plan crab manoeuvres the hull cannot execute.

## Horizon and costmap sizing

```
horizon        = time_steps × model_dt = 56 × 0.05 = 2.8 s
reach at vx_max = 2.8 × 1.30            = 3.64 m
local costmap  = 12 m across            = 6.00 m radius   ✓
```

If reach ever exceeds the costmap radius, MPPI is silently speed-limited by
the map rather than by the vehicle. `validate.py` checks this too.

`model_dt` must equal `1 / controller_frequency` (0.05 s at 20 Hz). It may be
smaller, never larger.

## Tuning order

Work outside in. Each step assumes the previous one is settled.

1. **Boat first, controller never.** `ros2 launch roboboat_bringup
   sim.launch.py`, then step `/cmd_vel` and watch `/boat/velocity`. Raise the
   `controller.gain_*` values in `boat_params.yaml` until thrust saturates on a
   step command, then back off ~30%. If the boat itself oscillates, no MPPI
   tuning will save you.
2. **Confirm the plumbing.** `mode:=kinematic` bypasses allocation and
   integration. If navigation works in kinematic mode and not in dynamic mode,
   the problem is the vehicle envelope, not the controller.
3. **Obstacle behaviour.** With `CostCritic` and the inflation layer only,
   drive past a buoy field. You want smooth motion roughly mid-channel and no
   close passes. Adjust `cost_weight` and `inflation_radius` together — they
   are coupled through the cost distribution, not independent knobs.
4. **Path following.** Bring `PathAlignCritic` up until the boat tracks the
   plan closely in open water but can still deviate around an unplanned
   obstacle. If it refuses to leave zero-cost space to make progress, raise
   `PathFollowCritic.cost_weight` rather than lowering the obstacle term.
5. **Approach and goal.** Tune `GoalCritic` / `GoalAngleCritic`
   `threshold_to_consider` for how early you want it to stop tracking the path
   and start converging on the pose.

## Open issue: marginal clearance for goals inside inflated space

Measured, reproducible, and worth understanding before you trust this config
near obstacles.

**Correction.** An earlier version of this section claimed goal `(22, 32)`
sits inside inflated space and blamed the excursion on that. Measured against
the world file, it does not: the goal is 2.06 m from the nearest buoy
*surface*, and inflation extends 1.30 m from the surface. Clearance with the
hull parked exactly on that goal is +1.28 m. The stated mechanism was wrong.

What is true is that the excursion happened **en route**, at `(20.7, 32.7)` —
1.48 m from the goal, and therefore also outside `CostCritic`'s
`near_goal_distance` of 1.0 m, which rules out the near-goal exemption as the
cause too. The boat threads a gap at roughly 0.15 m clearance there, and that
is close enough to zero that it varies run to run:

| Run | Min clearance at (20.7, 32.7) |
| --- | --- |
| 1 | **−0.17 m** (footprint overlapped the buoy) |
| 2 | **+0.14 m** |
| open-water goals | +0.33 m |

Clearance here is the exact rectangle-to-circle gap using the same footprint
polygon Nav2 uses, not a circumscribed-circle approximation — the overlap in
run 1 was real.

Both runs also logged one `Failed to make progress` from the progress checker,
which in this behavior tree triggers `ClearEntireCostmap` on the local
costmap. That momentarily empties the obstacle layer until the next scan
arrives (lidar 15 Hz, costmap 10 Hz), and MPPI has nothing to avoid in that
window. That is a plausible contributor but is **not** confirmed as the cause.

Things to try, roughly in order of how much they cost you:

1. Keep goals out of inflated space — still worth doing, but note it does
   **not** explain this incident, since this goal already was.
2. Raise `footprint_padding` (currently 0.05) toward 0.15. Directly buys
   standoff everywhere, at the price of tighter gates.
3. Raise `CostCritic.cost_weight` above 3.81, or `critical_cost` above 300.
4. Lower `CostCritic.near_goal_distance` from 1.0. It exists so the robot can
   converge on goals near obstacles, and it does that by dropping the
   preferential obstacle term — exactly the situation here.
5. Reduce `CostCritic.trajectory_point_step` from 2 to 1 so every predicted
   pose is footprint-checked rather than every other one.

Worth knowing about the implementation: `CostCritic` skips any trajectory
point whose *centre-point* cost is zero before it ever runs the footprint
check, and only runs the full footprint check when the centre cost exceeds the
cost at the circumscribed radius. So footprint checking is gated behind the
inflation layer being configured sensibly — which is another reason
`inflation_radius` must stay above the circumscribed radius (0.93 m here).

## Debugging

- `visualize: true` publishes the candidate trajectory bundle to
  `/trajectories`. It is genuinely expensive — 2000 rollouts × 56 points at
  20 Hz — so use it while looking at it and turn it off afterwards. The RViz
  display is present but disabled by default.
- `/thrusters/markers` shows per-thruster force as arrows. Arrows flipping sign
  every control tick means the velocity command is chattering; arrows pinned at
  full length mean the allocator is scaling back a wrench the layout cannot
  deliver.
- `/transformed_global_plan` is the slice of the plan MPPI is actually
  considering. If it looks truncated, check `prune_distance` against your speed
  and horizon.
- A boat that creeps toward the goal and stops short is usually the goal
  critics taking over too early — raise the `threshold_to_consider` values in
  step 5 rather than raising the speed limits.
