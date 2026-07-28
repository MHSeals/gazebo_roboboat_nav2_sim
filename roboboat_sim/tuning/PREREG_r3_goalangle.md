# Arm R3 — GoalAngleCritic 3.0 -> 12.0. Pre-registered before any R3 data.

    GoalAngleCritic.cost_weight   3.0 -> 12.0

One parameter. Zero CPU.

## Why re-opening a rejected axis is legitimate here

Journal §5 ran this exact change and measured **−8.1 s** against a registered
≥5 s prediction — the only correct prediction of that session, and the only one
derived from measurement rather than reasoning. It was rejected solely because
`encounter_min_mean` fell 0.191 m against a registered 0.10 m limit, while
worst-case clearance was unchanged (0.593 vs 0.605, noise).

The journal did not close the axis. It set a precondition, verbatim:

> "settle whether `encounter_min_mean` is the right safety metric *before*
> re-testing, on its own merits and with no result in view. If the conclusion is
> that worst-case clearance is what matters and mean standoff is not, then re-run
> this experiment against the corrected criterion."

**That precondition is satisfied, and by other means rather than by my choosing.**
The contract governing this phase of the project is, in both
`regression_channel.json` and `regression_sprint.json`,
`min_clearance_fraction_at_least: 0.20` plus `contact: false` — per-run
worst-case, not mean standoff. `encounter_min_mean` appears in neither contract.
It was an artefact of the retired 4-goal open-water benchmark suite, which this
phase replaced with gate-transit tasks.

Recorded **before** the run, as the journal demanded. I am not arguing that mean
standoff does not matter; I am observing that the criterion in force is the
worst-case one, and that is the one this arm will be judged against.

## Mechanism

    TwirlingCritic   weight 10.0, switches OFF inside xy_goal_tolerance    = 0.50 m
    GoalAngleCritic  weight  3.0, switches ON  inside threshold_to_consider = 0.80 m

Between 0.50 m and 0.80 m both are live and the term **opposing** yaw outweighs
the term **demanding** it 10:3. Raising GoalAngleCritic makes it dominant in
exactly that band without touching `TwirlingCritic`, which is bracketed at 10.0
on both sides (1.5 cost 20.7 s; 20.0 gained nothing).

Structurally this arm is much safer than R2: `GoalAngleCritic` gates on
`threshold_to_consider: 0.80` m from the goal, so it cannot affect how the boat
threads a gate 20 m earlier. R2 acted everywhere on the course, which is why it
tightened tracking through the sprint gate and lost 0.040 m of clearance.

## Target, measured

Terminal settle — time from first entry within 1.5 m of the final waypoint to run
end:

| | runs | mean | sd |
|---|---|---|---|
| baseline | 12.2 / 12.1 / 14.7 s | **13.0** | 1.4 |
| R2 (rejected) | 9.6 / 9.2 / 11.9 s | 10.2 | 1.5 |

2σ on an arm-vs-baseline difference = **2.3 s**. The expert reference covers that
last 1.5 m in about 1.5 s including its braking, so ~11.5 s of excess sits in
this band — the largest single item in the whole gap.

## Predictions

| # | endpoint | baseline | predicted | FALSIFIED IF |
|---|---|---|---|---|
| R3a | **terminal settle** (primary) | 13.0 s | **≤ 9.0 s** | **≥ 10.7 s** |
| R3b | channel elapsed | 104.5 s | **≤ 101.0 s** | ≥ 103.0 s |

**Stated up front so it cannot be spun afterwards:** R3b at 3.5 s is resolvable
(2.6σ) but **below the project's `MIN_EFFECT['reached_seconds']` of 5.0 s**, so
even a successful R3 is *immaterial on elapsed by itself*. Journal §5's −8.1 s
came from a 4-goal suite with four full stops; the channel has one, so ~2 s per
stop predicts 3–5 s here, not 8. R3 earns its place by being the arm that fixes
the largest *item*, not by clearing the elapsed floor alone. If it works it
belongs in a package, and the package is what gets judged against 5.0 s.

## Guards — any one kills the arm

* `contact: false` on every run.
* **Every individual run** channel min clearance ≥ 0.15 m.
* Sprint min clearance ≥ 0.130 m on every run.
* Both frozen contracts pass; neither JSON edited.

## Registered risk

A stronger yaw term near the goal may oscillate on approach (rotation up) or pull
the boat off the path on the way in (clearance down). Journal §5 registered the
same two risks and neither materialised on the old suite; the new one ends inside
a gate, so the clearance risk is higher here than it was there.
