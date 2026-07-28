# Arm R2 — the relay dead band. Pre-registered before any R2 data exists.

    PathAngleCritic.max_angle_to_furthest   0.35 -> 0.0

One parameter. Zero CPU (the critic stops early-returning, so it does very
slightly *more* work: one atan2 and one angular difference over 2000 rollouts,
well under 1% of the loop).

## Mechanism

`max_angle_to_furthest` is an early-return gate on the **current pose**, not an
angle offset:

    if (posePointAngle(pose, goal_x, goal_y, true) < max_angle_to_furthest_) return;

So below 0.35 rad (20°) of bearing error `PathAngleCritic` contributes **nothing
at all**, and above it the term arrives at weight 6.0 — the second largest in the
file. A proportional corrector with a dead band, first-order thruster lag
(0.15 s), an inner velocity loop (~0.4 s) and a 20 Hz update is a textbook
limit-cycle recipe, and the amplitude of the cycle is set by the dead band. At
0.0 the gate never fires (`0 < 0` is false) and the term is continuously on.

This is monotone with a change already validated: arm J moved this same parameter
1.0 -> 0.35 and it worked (crab frames 27% -> 5%). R2 continues the same
direction rather than opening a new axis.

## Evidence it is the right target

Measured on the **obstacle-blind** portion of the three baseline traces — frames
where no buoy lies within the 1.30 m inflation radius of a 3.64 m look-ahead, so
`CostCritic` is identically zero for every rollout and cannot be the cause:

* `corr(speed, |wz|)` = **−0.33 / −0.38 / −0.44**. Speed dips are coupled to yaw
  activity in *empty water*.
* Speed there does not sit at a wall: p50 ≈ 1.00 but p99 is **1.274** and max
  **1.280**. The boat can hold 1.28 and does not.
* The obstacle-free control course (shipped config, zero parameters changed) runs
  at 1.056 m/s, not 1.300 — so ~60% of the speed deficit survives removing every
  obstacle from the horizon and must be intrinsic to the controller.

## Instruments, both measured rather than assumed

| endpoint | baseline | n | sd | 2σ on a difference | MIN_EFFECT |
|---|---|---|---|---|---|
| channel elapsed | 104.5 s | 3 | 1.66 | 2.7 s | **5.0 s** |
| total rotation | 581.9° | 3 | 15.8 | **26°** | none set |
| channel min clearance | 0.221 m | 3 | 0.037 | 0.06 m | 0.05 m |

The rotation sd is newly measured. H1a's rotation falsifier was registered
without it and was worthless: its 4.1° move was 0.32σ.

## Predictions

| # | endpoint | predicted | FALSIFIED IF |
|---|---|---|---|
| R2a | **total rotation** (primary) | **≤ 530°** (−52°, 2× resolvable) | **≥ 556°** (under 2σ) |
| R2b | channel elapsed | **≤ 99.5 s** (−5.0 s, clears MIN_EFFECT) | **≥ 101.8 s** |

**Both must hold.** Either failure rejects the arm. Rotation is primary because
it measures the oscillation directly; elapsed is what anyone cares about but it
pools ten gates and four curvature regimes.

## Guards — any one kills the arm outright

* `contact: false` on every run.
* **Every individual run** min clearance ≥ 0.15 m (not the mean — H1a passed on
  the mean while one sprint run sat 0.006 above the contract floor).
* Both frozen contracts pass under their own criteria; neither JSON edited.
* Sprint min clearance ≥ 0.130 m.

## Registered risk

With the term always on, the boat chases reference ripple it previously ignored
inside the dead band. The plan is rough — I measured the logged global plans
demanding **907°** of turning over 95.4 m (median 9.50 deg/m) against the racing
line's 282° — so there is plenty of ripple to chase. If rotation goes **up**, the
mechanism is inverted: the dead band was suppressing ripple-chasing rather than
causing a limit cycle, and the axis closes.

## Registered null

If rotation moves less than 26° and elapsed less than 2.7 s, the relay reading is
wrong. The next suspect is then the plan itself, not the controller: on an
analytically smooth path the offline surrogate drove 197.8° for 192° required — a
ratio of 1.03 — so given a smooth path this controller does not hunt at all. That
would make `SmoothPath` in the behaviour tree (currently absent, so
`smoother_server` is dead config) the arm, and it is a plan change that must not
share a round with a controller change.
