# Alternate planner's round-1 plan (streamlined context, no journal access)

Transcribed by the lead from the agent's delivered plan. Numbers are its own.

## Its diagnosis

It probed the real plant (dynamics + allocation + VelocityController, 100 Hz):

| commanded | achieved mean surge | elapsed over 95.4 m |
|---|---|---|
| vx 1.30, wz 0 | 1.300 | 73.4 s |
| vx 1.30, wz ±0.15 sin | 1.300 | 73.4 s |
| vx 1.30, wz ±0.30 sin | 1.300 | 73.4 s |
| vx 1.30, vy 0.20 | 1.300 | 73.4 s |
| vx 1.30, vy 0.45 | 1.107 | 86.2 s |

Conclusion: **yaw hunting costs this plant zero forward speed** (67.2 N·m on a
0.672 m arm; a 20 N·m demand takes nothing off the 68.25 N needed to hold
1.30 m/s). So excess rotation is an expert-likeness defect, not the time
mechanism. **The whole 34 s is "MPPI commands ~0.99 m/s instead of 1.30".**

Why: cost-unit arithmetic at `temperature: 0.3`.

* `PathFollowCritic` is a single-point cost on ‖traj terminal − path[furthest+5]‖.
  Its aim index is clamped by the pruned path (`PathHandler` prunes at
  `prune_distance` = 4.0 m). The term is ≈ weight × (4.0 − 2.8v), so across the
  whole 0.99 → 1.30 m/s range it changes by 5.0 × 2.8 × 0.31 = **4.3 units**.
* `CostCritic` (logs itself as `InflationCostCritic`) is weight × mean footprint
  cost over `traj_len/2` = 28 points. `InflationLayer` gives
  252·exp(−k(d − 0.60)); at k = 2 that is 113 at d = 1.00 m and 62 at the 1.30 m
  rim. Ten of 28 points in the field costs 3.81 × 10 × 113 / 28 = **154 units**.

And `gamma: 0.015` is control regularisation, **not** a time discount, and there
is no terminal value function — so slowing down to push obstacle cost past the
2.8 s horizon is nearly free. Buoys line the whole course.

Resolution it works to: 2σ ≈ 2.7 s on elapsed; no clearance change below 0.06 m
resolvable. Safety gate on every arm: `contact: false`, **every individual run**
min clearance > 0.15 m, both contracts pass.

## Its ranked arms

1. **`local_costmap.inflation_layer.cost_scaling_factor` 2.00 → 6.00** (global
   untouched, so the plan is held fixed). Cuts the varying part of the dominant
   term ~5x while the near wall gets 3x steeper. Argues it **provably cannot
   degrade tight-gate clearance**: padded inscribed radius 0.60 m, and the 2.8 m
   gates offer a 0.65 m ceiling, so the footprint outline sits inside 0.60 m of a
   buoy cell throughout any tight transit and `footprintCostAtPose` returns 253
   regardless of k. Corollary: `CostCritic` gives **no centring gradient** inside
   the 2.8 m gates, so this file's claim that it centres the boat between paired
   gate buoys holds only in the 3.0–4.0 m gates. CPU exactly zero
   (`possible_collision_cost_` falls but the physical trigger distance is
   unchanged). **Cliff: do not approach k = 13.8**, where
   `possible_collision_cost_` = 252·exp(−0.40k) < 1 makes `inCollision()` run the
   footprint check unconditionally and silently blow the 4-core budget.
   Predict mean elapsed ≤ 96.5 s; falsify at ≥ 101.8 s. Secondary watch: if
   elapsed falls but rotation exceeds 620°, de-escalate to k = 3.5.
2. **`PathFollowCritic.cost_weight` 5.0 → 40.0.** d(cost)/dv = weight × 2.8, so
   14 units per m/s at weight 5 and 112 at weight 40. Matching the obstacle
   swing needs weight ≈ 20–55. Predict ≤ 95.0 s; falsify ≥ 101.8 s or any
   clearance breach. Ranked second because it buys speed by driving harder at an
   aim point sitting in inflated water.
3. **`prune_distance` 4.0 → 5.5.** This one parameter *is* the lookahead for both
   terminal-point critics. At 0.99 m/s the fastest sample reaches ~3.6 m ≈ index
   16 and a 4.0 m pruned path holds only ~18–20 poses, so `furthest+5` and
   `furthest+4` both clamp to the last pruned pose: PathFollow sits saturated,
   and PathAngle's aim is only ~1.2 m out, where 0.10 m of grid ripple subtends
   4.8° of commanded heading, chased at weight 6. 5.5 not 6.0 because the plan is
   truncated at the 6 m local costmap edge. Predict ≤ 99.5 s **and** rotation
   ≤ 500°. CPU +3–5% (`findPathFurthestReachedPoint` is batch × path double
   loop, +37% on that function).
4. **`PathAlignCritic.cost_weight` 14.0 → 4.0.** Largest weight in the file and
   the only term forbidding a different line from the plan. But it measured the
   geometry this gambles with: three free-standing buoys sit essentially on the
   gate-centre lines — yellow r=0.30 at (18.0, 2.6) is 0.38 m off chan_a→chan_b,
   (26.5, 6.2) is 0.39 m off chan_b→chan_c, and (6.0, 31.5) is **0.25 m** off
   return_b→finish. Those three are where the align term is doing essential
   work. Predict course distance ≤ 93.0 m (on a ≪0.1 m instrument) and elapsed
   ≤ 99.5 s; falsify at distance ≥ 95.0 m, which would mean the 1.07x path ratio
   is the planner's, not the controller's.
5. **`PathAngleCritic.max_angle_to_furthest` 0.35 → 0.0.** The gate is an early
   return on the current pose, so heading error must accumulate past 20° before
   any correction fires at weight 6, then switches off — "a relay with a dead
   band is an oscillator, and its amplitude is set by the dead band." At 0.0 the
   term is continuously on. Predict rotation ≤ 500° with elapsed unchanged
   (±2.7 s); falsify at rotation ≥ 536° or elapsed ≥ 107.8 s.

## Its zero-parameter control experiment (ranked as highest information)

Build a spread-out channel — same ten gate headings and turn angles, all gates
widened to ≥ 4.0 m, no free-standing buoy within 1.5 m of any gate-to-gate line
— and run the **shipped** config on it unchanged. If it already runs near
1.30 m/s, the diagnosis is confirmed and the tight channel is a course problem,
not a tuning problem. **If it still cruises at ~0.99 m/s, arms 1–4 are all built
on a wrong premise and should not be run.** Produces the training asset the owner
asked for either way.

## Its dead ends

* `wz_max`, `az_max`, `ax_max`, `vx_max`, `vy_max` raises — nothing binds;
  `ax_max` 1.5 already ramps 0 → 1.30 in 0.87 s of a 2.8 s horizon.
* `TwirlingCritic` either direction — it trades alignment authority against a
  physically free quantity.
* `vy_max` reductions — and note `ConstraintCritic` builds
  `max_vel_ = hypot(vx_max, vy_max)` = 1.376, so cutting `vy_max` lowers the
  speed wall toward 1.30, the wrong direction.
* `velocity_smoother` — off, limits already equal MPPI's.
* `smoother_server` / `simple_smoother` — **dead config**, no `SmoothPath` node
  in any tree. Real smoothing is `GridBased.smoother.*`, unset, at nav2 defaults
  (w_data 0.2, w_smooth 0.3). `w_smooth` 0.3 → 1.5 is the named candidate if
  arm 4's falsifier fires.
* `ConstraintCritic.cost_weight` — real asymmetry recorded but **not** an arm:
  samples are not clipped to `vx_max` (only the nominal sequence is, in
  `applyControlSequenceConstraints`), so samples above 1.376 pay a horizon
  **sum** of 4.0 × Σ₅₆(|v| − 1.376)⁺ = 224 units per m/s — 16x PathFollow's whole
  authority. But it only bites within 0.076 m/s of 1.376 while the boat cruises
  at 0.987, so predicted effect ≤ 2 s, **under the 2.7 s resolution and
  therefore unfalsifiable as an arm.**
* Raising `CostCritic.cost_weight` — backwards under this diagnosis.
* `iteration_count` 1 → 2 — doubles cost for refinement the shifted control
  sequence already partly supplies.

## Its two flagged checks

1. `CostCritic.collision_cost: 1000000.0` **may be a dead key** — absent from the
   library's parameter-name strings, though the test is inconclusive because GCC
   suffix-merges literals and "collision_cost" is a suffix of
   "near_collision_cost". Settle free on the next launch with
   `ros2 param list /controller_server | grep CostCritic`. Note that
   **`PathAlignCritic` logs itself as `ReferenceTrajectoryCritic` and
   `CostCritic` as `InflationCostCritic`** in 1.3.12.
2. **If arms 1–3 all miss, the suspect is the sampler, not the critics.**
   `MotionModel::predict` clamps each sampled control to within `model_dt·a_max`
   of the *previous sampled step*: ±0.075 m/s for vx, ±0.04 rad/s for wz. Against
   `vx_std` 0.35 and `wz_std` 0.25 that clamp is active on **83% of vx steps and
   87% of wz steps**, so the sample cloud is a saturated bang-bang random walk
   from the current measured velocity, not a Gaussian about the nominal. The
   batch may never contain a clean "cruise at 1.30" candidate. The arm would then
   be `vx_std` 0.35 → 0.10.
