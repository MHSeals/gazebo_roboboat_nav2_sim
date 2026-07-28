# Arm J — pre-registered

## Change (two lines, `nav2_mppi.yaml`, no rebuild — install config is a symlink)

```
PathAngleCritic.cost_weight          2.0 -> 6.0
PathAngleCritic.max_angle_to_furthest 1.0 -> 0.35
TwirlingCritic.cost_weight           10.0 (UNCHANGED)
```

## Why this and not the obvious thing

Verified in `nav2_mppi_controller` 1.3.12 source (matches the installed
`ros-jazzy-nav2-mppi-controller 1.3.12-1noble`):

* `PathAngleCritic` scores the **terminal** trajectory yaw
  (`xt::view(data.trajectories.yaws, xt::all(), -1)`), so yawing at rate `w`
  over the T = 2.8 s horizon saves `W_pa · w · T = 2.8 · W_pa · w`.
* `TwirlingCritic` scores the **horizon mean** of `|wz|`, costing `W_tw · w`.
* Break-even is therefore `W_tw <= 2.8 · W_pa`. At the shipped 10.0 vs 2.0 the
  threshold is 5.6 against a cost of 10.0 — **yawing to fix heading loses by
  1.79x at every yaw rate.** At `W_pa = 6.0` the threshold is 16.8 against
  10.0, a **1.68x margin in favour of yawing**.
* `PathAngleCritic` early-returns when `posePointAngle(current_pose, ...) <
  max_angle_to_furthest_`. At 1.0 rad that is a **57.3° dead band in which yaw
  is purely penalised and sway is free**. The measured channel crab episodes
  sit at 31.3° and 41.1° median sideslip — inside it. 0.35 rad = 20°.

`TwirlingCritic` is deliberately left at 10.0: it is the only regulariser on a
DOF nothing else observes, and lowering it has a recorded 20.7 s regression
(n=2, agreeing to 1 s). If the ratio can be flipped without touching it, the
risky change is never spent.

## Predictions, with falsifiers

| # | endpoint | baseline | predicted | FALSIFIED IF |
|---|---|---|---|---|
| J1 | **sprint: seconds from first arrival within 0.8 m of the leg-1 goal to the leg actually changing** | **20.0 s** (I), 19.7 s (A) | < 14 s | **>= 17 s** |
| J2 | **sprint: max excursion back out after first arrival** | **1.44 m** (I), 0.91 m (A) | < 0.9 m | **>= 1.2 m** |
| J3 | **sprint: heading error at the min-clearance gate pass** | **37.5°** (I), 38.5° (A) | < 25° | **> 30°** |
| J4 | channel crab frames (\|vy\|>0.15 & \|wz\|<0.10) | 27.2% | < 22% | > 23.6% (repeat spread is 1.1-3.6 pp) |
| J5 | channel \|wz\| p90 | 0.181 | > 0.22 | <= 0.20 (within-config sd 0.004) |

J1/J2 are primary: the waypoint overshoot-and-crab-back is 20 s of a 149 s
sprint and reproduces across both thruster layouts, so it is the largest single
recoverable cost in the record. J3 is the strongest endpoint available — it
reproduced to 1° across two different layouts (37.5° vs 38.5°).

## Guards — any one of these kills the arm regardless of J1-J5

* `contact: false` on every run. **A single contact kills it outright.**
* Both regression contracts pass under `tools/regress.sh`. Neither JSON is
  edited.
* Sprint min clearance must not fall below **0.123 m**. Its margin over the
  contract floor is 0.030 m and it is the binding constraint of the whole
  exercise.

## Registered null prediction

`wz_max` is **not** binding: measured |wz| p99 is 0.232 against a cap of 0.55,
and **0.0%** of either run is spent above half the cap. If arm J returns |wz|
p99 below 0.45, `wz_max` must not be raised — write the prediction down rather
than buying a run to confirm it.

## Things ruled out offline, so no run is spent on them

* **Deceleration authority.** `VelocityController` gives
  `M·gain·(ν_cmd − ν) = 38.5 × 2.5 × (−0.75) = −72 N` for a full stop from
  0.75 m/s, against 99 N available. Working back from the 2–4 N commanded
  during the overshoot, MPPI's commanded `vx` is within ~0.04 m/s of the
  actual coast — **MPPI is not asking the boat to stop.** Cost function, not
  authority.
* **`PathAlignCritic`/`PathFollowCritic` offsets.** `PathFollowCritic`'s
  `offset_from_furthest` is not a gate at all (it clamps an index);
  `PathFollowCritic` is off in both stalls via `threshold_to_consider: 2.5`,
  which no offset change can re-enable. `PathAlignCritic` fires through 100% of
  the channel stall. The "dead band" mechanism does not exist.
* **`lin_yaw` / `quad_yaw`.** Required Mz at the operating point is 2.6 N·m of
  80.61 available (3.3%). No value changes any decision.
* **`use_path_orientations: true`.** `SmacPlanner2D` sets every pose
  orientation to identity and overrides only the last; the flag would score
  heading against world +x.

## Corrected numbers carried in

Decoupled tangential envelope is **80.61 N·m -> 2.467 rad/s** (2x35 N forward +
2x25 N reverse on a 0.6718 m arm), not 67.18/2.223 (which uses the reverse
limit on all four) and not the header's 2.69 (which is the abandoned pinwheel,
4x35).
