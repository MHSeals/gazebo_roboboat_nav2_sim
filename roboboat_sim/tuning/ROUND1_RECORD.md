# Round 1 record — for the critic

Everything measured, with the retractions in place. Three arms registered, two
resolved, one running.

## Instruments

* **Baseline, this box (12 cores, headless), shipped config.**
  channel n=3: 105.0 / 102.7 / 105.9 s (mean 104.5, sd 1.66);
  clearance 0.254 / 0.181 / 0.228 m (mean 0.221, sd 0.037).
  sprint n=2: 96.2 / 97.1 s; clearance 0.145 / 0.148 m.
  Both contracts pass on every run. Course distance 95.4 m on all three.
* **Resolution.** SE of a 3-run mean is 0.96 s, so SE of an arm-vs-baseline
  difference is 1.36 s and 2σ ≈ 2.7 s. Clearance: SE of the difference 0.030 m,
  so nothing below ~0.06 m of clearance change is resolvable.
* **Cross-box warning.** The journal's arm J was measured on 4 cores: same
  elapsed (103.1 s) but clearance 0.326 m against this box's 0.221 m. There is
  0.105 m less margin here than the journal implies.
* **Expert reference** (`tools/expert_helm.py`, minimum-time on an apex-cutting
  racing line, real plant, real allocator, verified legal at 10/10 gates):
  70.3 s, 89.2 m, at matched clearance and inside the same velocity limits.
  Insensitive to the clearance margin: 70.0 s at 0.15 m, 70.8 s at 0.40 m.

## Arms

| arm | change | registered falsifier | outcome |
|---|---|---|---|
| H1a | `time_steps` 56→72, `prune_distance` 4.0→5.0 | elapsed ≥101 s, rotation ≥520° | **FALSIFIED, both** — 107.7 s, 586° |
| H5 | `PathFollowCritic.cost_weight` 5.0→14.0 | sprint clearance < 0.130 m | **REJECTED on guard** — 0.055 m, contract FAIL, leg timed out |
| I1 | `local` `cost_scaling_factor` 2.00→6.00 | elapsed ≥101.8 s | running; sprint 98.4 s / 0.139 m PASS |

## Retractions, all mine

1. **"Excess rotation costs time."** Wrong for this plant, and the alternate
   planner caught it. Probed on the real plant with the real allocator, thruster
   lag, and along-path projection: yaw oscillation at the **full** `wz_max`
   amplitude costs 2.0 s over the 95.4 m course; at the amplitude actually
   measured (mean |wz| 0.096 rad/s) it costs **~0.4 s of the 34 s gap**. Max
   sway, by contrast, costs 36 s. Rotation is nearly free on a 67.2 N·m /
   0.672 m layout. The 1.95x rotation figure is an **expert-likeness** finding,
   not a time finding — which still matters, because expert-likeness is the
   stated end test, but it is not where the time is.
2. **"PathFollowCritic is a speed regulator."** I talked myself into this while
   H1a was running because it explained the slowdown in one step. The installed
   `utils.hpp` refutes it: `findPathFurthestReachedPoint` indexes off
   `xt::view(trajectories.x, all(), -1)` — the target tracks where the
   **rollouts** reach, not the robot, so it runs away from you and cannot pin
   speed. The version that explained the last result was the wrong one.
3. **Sprint clearance under H1a understated in the first commit message.** I
   wrote "0.147 → 0.131" and omitted the 0.096 m run, which made a near-miss
   (0.206 fraction against a 0.200 floor — passed by 0.006) look like ordinary
   noise. Amended.
4. **Reference legality.** My first expert reference scored 9/10 gates: the path
   began at the first gate and ended on the finish line, so it crossed neither,
   and it also omitted the 6 m from spawn. Caught by scoring it with the real
   task scorer instead of trusting it. Both ends fixed.
5. **Gap decomposition corrected.** I first read the first-segment loss as 6 s of
   startup latency. Measured: startup dead time is 0.86–2.36 s. Of the 34.2 s,
   ~1.4 s is handshake and ~3 s is terminal yaw convergence the reference does
   not model, leaving ~30 s genuinely addressable — of which the standing start
   is the largest single item (MPPI 8.5 s over the first 6 m at 0.71 m/s, while
   the plant reaches 1.30 m/s in 0.87 s even under its own `ax_max` of 1.50).

## The diagnosis now on the table (alternate planner's, which I have checked)

`CostCritic` dominates by an order of magnitude, and MPPI truncates at the
horizon with no terminal value function, so wherever inflated water lies inside
the horizon, **slowing down is nearly free**:

* `PathFollowCritic`'s entire authority across the 0.99 → 1.30 m/s range is
  `5.0 × 2.8 × 0.31` = **4.3 cost units**.
* `CostCritic` with 10 of its 28 evaluated points at d = 1.00 m contributes
  `3.81 × 10 × 113 / 28` = **140 cost units**.

I verified the inflation field arithmetic independently: at k = 2.00 the cost is
113.2 at 1.00 m and 62.1 at the 1.30 m rim; at k = 6.00 it is 22.9 and 3.8,
while everything at d ≤ 0.60 m stays at 253 — identical. The tight gates cannot
be affected, because the padded inscribed radius is 0.60 m and the 2.8 m channel
gates leave exactly 0.60 m (the 2.4 m sprint gate only 0.415 m), so
`footprintCostAtPose` saturates at 253 there regardless of k.

**This diagnosis retro-explains H1a**, which is why I take it seriously: a longer
horizon pushes more trajectory points into inflated water, raising the cost of
speed — so the boat got slower *and* its channel clearance improved
(0.221 → 0.278 m), which is exactly the signature. My own horizon hypothesis
predicted the opposite and was wrong.

**It also predicts H5's failure direction**, though not its severity: the
progress term cannot be raised to out-shout the obstacle term because the follow
point sits in inflated water inside the gate, so the boat drives at it.

## Verified incidentals

* `smoother_server` / `simple_smoother` is **dead config** — no `SmoothPath`
  node exists in any of the three behavior trees. The plan *is* smoothed, by
  `SmacPlanner2D`'s internal smoother, whose parameters (`GridBased.smoother.*`)
  are **not set at all**, so it runs at nav2 defaults nobody in this project
  chose. Untouched axis.
* `temperature`, `gamma`, `iteration_count`, `time_steps`, `batch_size`,
  `vx_std`, `vy_std`, `wz_std` appear **nowhere** in 2294 lines of journal. The
  optimiser was never tuned; only critic weights and velocity caps.
* Instrument bug, fixed: recorded `wz` carries ±125.63 rad/s spikes
  (= 2π/0.05 s) when heading crosses ±π. Two frames of 993 inflated ∫|wz|dt
  from 592° to 2155°. Percentiles survive; integrals and extrema do not.
* Harness gap: `check_critics.py` reports `1 log no weight: PathFollowCritic`,
  so that critic's weight is **not** machine-verified against the startup log.
  This is the same class of hole as the `twirling_cost_weight` dead key that
  cost this workspace a whole experiment.

## What I want the critic to attack

1. Is the obstacle-dominance diagnosis actually load-bearing, or is it a
   just-so story that fits two failures after the fact? It was formed *after*
   H1a and *predicts* H5 only in direction.
2. Am I over-fitting to the tight channel? The owner says the real course is
   more spread and forgiving. A `cost_scaling_factor` tuned to make a
   buoy-lined 2.8 m channel fast may be actively wrong for open water.
3. Is the expert reference a fair target at all, or is 70.3 s an artefact of
   assuming perfect coordinated turns and a racing line no perception stack
   could follow?
4. Which of the remaining axes are rabbit holes given the journal's closed list?
