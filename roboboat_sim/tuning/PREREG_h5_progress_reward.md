# H5 — nothing in this cost function rewards speed. Pre-registered.

> **RETRACTION, added while the arm was running.** The framing below repeatedly
> ties excess rotation to lost time — "every degree of unnecessary yaw on a hull
> whose sway drag is 3x its surge drag is thrust spent going nowhere". **That is
> false for this plant**, and the alternate planner caught it.
>
> Probed on the real plant (100 Hz closed loop, real allocator, thruster lag),
> commanding vx 1.30 with a sinusoidal wz and measuring progress along the
> intended line, not just surge:
>
> | case | surge | along-path | 95.4 m in |
> |---|---|---|---|
> | no yaw | 1.300 | 1.286 | 74.2 s |
> | wz ±0.15 sin (≈ the measured mean \|wz\| of 0.096) | 1.300 | 1.282 | 74.4 s |
> | wz ±0.30 sin | 1.300 | 1.275 | 74.8 s |
> | wz ±0.55 sin (at the cap) | 1.295 | 1.251 | 76.2 s |
> | vy 0.45 (max sway) | 1.107 | **0.862** | **110.7 s** |
>
> Yaw oscillation at the *full* cap amplitude costs 2.0 s over the course; at
> the amplitude actually measured it costs **~0.4 s of the 34 s gap**. The
> tangential layout has 67.2 N·m on a 0.672 m arm, and a 20 N·m yaw demand takes
> essentially nothing off the 68.25 N needed to hold 1.30 m/s — rotation is very
> nearly free. Sway is the expensive DOF, and arm J already fixed sway.
>
> So the 1.95x rotation figure is a **style** finding — it bears on "does this
> look like a human helm", which is the stated end test — and **not** a time
> finding. Conflating the two is my error and it inflated the apparent stakes of
> the rotation metric throughout the section below.
>
> The arm itself survives, because its *other* justification — the cost ledger —
> does not depend on rotation at all. But see the dose note at the end: at
> weight 14.0 it is probably still under-powered, and the reason is arithmetic I
> did not do before launching it.

Written **before** H1a runs 2–3 and both sprint runs land, and before the
alternate-planner and novelty agents report, so it cannot be a rationalisation
of either.

## Why H1a going the wrong way points here

H1a run 1 came back at 110.2 s against a 104.5 s baseline — a longer horizon
made the boat *slower*. My registered mechanism was "the boat cannot see the
exit of a corner, so it has no rollout proving the corner is exitable at speed."
If that were the whole story, more look-ahead could not hurt.

The reading that explains a slower boat: **more horizon is more opportunity for
cost to accumulate, and there is almost nothing on the other side of the ledger
pushing for speed.** Extra look-ahead buys extra caution, because caution is
what the cost function is mostly made of.

## The ledger, from the shipped weights

Terms that oppose motion or deviation:

| critic | weight | what it penalises |
|---|---|---|
| PathAlignCritic | **14.0** | deviation from the path |
| TwirlingCritic | **10.0** | horizon mean of \|wz\| |
| PathAngleCritic | 6.0 | terminal yaw vs bearing to a path point |
| CostCritic | 3.81 | proximity to inflated cells |
| ConstraintCritic | 4.0 | exceeding the velocity envelope |
| PreferForwardCritic | 5.0 | reverse surge |

Terms that reward getting further along:

| critic | weight | active when |
|---|---|---|
| PathFollowCritic | **5.0** | only outside `threshold_to_consider: 2.5` m of the goal |
| GoalCritic | 5.0 | only *inside* `threshold_to_consider: 2.5` m |

So on the open part of a leg, `PathFollowCritic` at 5.0 is the **only** term
rewarding progress, against 14.0 + 10.0 = 24.0 of path-and-yaw restraint before
obstacles are counted at all. Progress is outvoted roughly 5:24.

That is a sufficient explanation for the measured flat **17% speed deficit on
straights** (1.08 m/s used of 1.300 m/s feasible, at curvature < 0.05 where no
cornering argument applies). It also explains why the deficit is *flat* rather
than concentrated: a weight ratio does not care where on the course you are.

## Source verification, and a wrong turn I took on the way

Between writing the section above and registering the arm I talked myself into
believing the opposite — that `PathFollowCritic` is a speed *regulator* rather
than a progress reward. The reasoning was: it scores the distance from each
trajectory's **last** point to the path point `offset_from_furthest: 5` poses
(~1.0 m at the 0.20 m planner resolution) ahead, so a trajectory reaching 2.8 m
in a 2.8 s horizon overshoots that target and is penalised, making the
least-cost trajectory the slowest one. That would have made raising the weight
*harmful*, and it would have explained H1a's slowdown neatly.

**It is wrong, and the installed source says so.** From
`/opt/ros/jazzy/include/nav2_mppi_controller/tools/utils.hpp`:

```cpp
inline size_t findPathFurthestReachedPoint(const CriticData & data)
{
  const auto traj_x = xt::view(data.trajectories.x, xt::all(), -1, xt::newaxis());
  const auto traj_y = xt::view(data.trajectories.y, xt::all(), -1, xt::newaxis());
  ...
  // "Idx of furthest path point reached by a set of trajectories"
```

The index tracks **where the rollouts reach, not where the robot is**. The
target therefore advances *with* the trajectories: faster rollouts push
`furthest_reached_path_point` further along, which pushes the follow point
further ahead. It is a pursuit point that runs away from you, not a fixed mark
you can overshoot. So there is no speed pinning.

And upstream states the intended effect explicitly, in
`critics/path_follow_critic.hpp`:

> "A higher weight here with an offset > 1 will **accelerate the samples to full
> speed faster** and push the follow point further ahead, creating some
> shortcutting."

That is the library's own documentation of exactly the lever this arm pulls,
including the side effect — "some shortcutting" — which is a clearance risk and
is why the clearance guard below is not a formality.

This is recorded rather than quietly corrected because the wrong version was
*more* satisfying: it explained H1a in one step. A mechanism that explains the
last result is the one to distrust most.

**Note what this does NOT explain.** With the pursuit point advancing with the
rollouts, a longer horizon should not slow the boat down, so H1a's 110.2 / 107.4
s remains unexplained by this mechanism. The leading candidate is that the
restraint terms accumulate over trajectory points while the progress term reads
only the endpoint — `PathAlignCritic` (14.0) and `CostCritic` (3.81, with
`trajectory_point_step: 2`) both integrate along the trajectory, and
`TwirlingCritic` (10.0) averages `|wz|` over it. Lengthening the horizon would
then add restraint without adding reward. **I could not verify this**: the
critic `.cpp` bodies are compiled into the shared object and this box has no
network access, so the summation form of those three critics is an inference,
not a reading. Flagged as unverified rather than asserted.

## The arm

    PathFollowCritic.cost_weight   5.0 -> 14.0

One parameter. 14.0 is chosen to match `PathAlignCritic`, so progress and
path-adherence carry equal weight rather than progress being outvoted ~3:1 by
that one term alone. Not larger, because `PathFollowCritic` scores the distance
from the trajectory's end to a point `offset_from_furthest: 5` poses ahead, and
making it dominant risks corner-cutting toward the lookahead point — which is
a clearance risk and the thing the hard contract exists to catch.

**CPU cost: none.** A weight is a multiply.

## Predictions, with falsifiers

| # | endpoint | baseline | predicted | FALSIFIED IF |
|---|---|---|---|---|
| H5a | channel elapsed | 104.5 s (sd 1.66, n=3) | **≤ 97 s** | ≥ 101.5 s |
| H5b | speed utilisation at curvature < 0.05 | 0.83 | **≥ 0.90** | ≤ 0.855 |
| H5c | channel min clearance, mean of n=3 | 0.221 m | ≥ 0.190 m | < 0.170 m |

H5b is the primary endpoint: it is the one measured directly against a
plant-derived ceiling rather than against a historical number, and it is where
the mechanism makes its most specific claim. H5a is the thing anyone cares
about but it pools ten gates and four curvature regimes.

## Guards — any one kills the arm

* `contact: false` on every run. A single contact kills it outright.
* Both frozen contracts pass under `tools/regress.sh`; neither JSON edited.
* Sprint min clearance not below **0.130 m** (baseline 0.145/0.148, and the
  sprint is the binding contract with the least margin).

## Registered null

If H5 moves elapsed by less than 3 s **and** utilisation by less than 0.03, the
"progress is outvoted" reading is wrong and the flat speed deficit is not a
weight-ratio effect. In that case the next suspect is the softmax itself —
`temperature: 0.3` averaging a truncated sample distribution whose optimum sits
on the `vx_max` boundary, which would put the deficit in the optimiser rather
than in the cost function. That is a different arm and must not be smuggled in
alongside this one.
