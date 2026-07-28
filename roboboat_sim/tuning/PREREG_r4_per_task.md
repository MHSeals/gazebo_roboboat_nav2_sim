# Arm R4 — per-task controllers. Pre-registered before any R4 data.

    controller_plugins: ["FollowPath", "FollowPathSprint"]
    FollowPath.PathAngleCritic.max_angle_to_furthest       0.35 -> 0.0
    FollowPathSprint.PathAngleCritic.max_angle_to_furthest 0.35   (unchanged)
    navigate_sprint_boat.xml  default_controller  FollowPath -> FollowPathSprint

Verified before running: the two blocks hold 74 keys each and differ in **exactly
one** — `PathAngleCritic.max_angle_to_furthest`. Anything else would make the arm
unattributable.

## Why this is not a compromise dose

The R2 bracket (0.35 / 0.15 / 0.00) is a clean monotone trade: closing the relay
dead band buys channel time, rotation and terminal settle, and costs sprint gate
clearance. Both intermediate doses fail — 0.00 clears `MIN_EFFECT` but breaches the
sprint guard, 0.15 breaches the guard *and* misses `MIN_EFFECT`.

The trade only exists because one parameter set serves two tasks whose geometry
differs materially: **2.8 m channel gates leave 0.60 m of padded clearance a side,
the 2.4 m sprint gate leaves 0.415 m.** Nav2 already supports per-task
controllers, `ControllerSelector` is already present in all three behaviour trees,
and the `FollowPath` action node already passes
`controller_id="{selected_controller}"`. No new infrastructure.

It is also the more human-like answer, which matters because that is the stated
target: a helm does not use one set of habits for a 2.8 m channel and a 2.4 m
gate. Per-task tuning is how the competition is actually run — the tasks are
separate runs.

## Predictions

The sprint arm is a **positive control**, and that is the point of running it
first. `FollowPathSprint` is byte-identical to the shipped `FollowPath`, so:

| # | endpoint | predicted | FALSIFIED IF |
|---|---|---|---|
| R4a | sprint min clearance | **reproduces baseline, ≥ 0.135 m** | < 0.130 m (standing guard) |
| R4b | sprint elapsed | 96.7 ± 3 s | — |
| R4c | channel elapsed | **reproduces R2, ≤ 100.0 s** | ≥ 101.8 s |
| R4d | channel total rotation | reproduces R2, ≤ 535° | ≥ 556° |

**The diagnostic value is in R4a.** Three outcomes and they mean different things:

* sprint ≈ 0.146 m (baseline) → the selection works, and the trade is dissolved.
* sprint ≈ 0.107 m (R2) → **the selection is NOT working**; the sprint tree is
  still running `FollowPath` and the whole arm is a no-op on that task. Most
  likely cause would be `ControllerSelector` publishing an empty string on
  `controller_selector` and overriding `default_controller`, or the plugin name
  not resolving.
* anything else → a third mechanism, investigate before believing either.

## Guards — unchanged from R2/R2b

* `contact: false` on every run.
* Every individual run: channel min clearance ≥ 0.15 m, sprint ≥ 0.130 m.
* Both frozen contracts pass; neither JSON edited.

## Known cost, recorded rather than hidden

Two MPPI instances are constructed at configure time, so the noise tensors are
allocated twice (2000 x 56 x 3 floats each, ~1.3 MB) and both critic sets are
loaded. Only the selected controller scores, so steady-state CPU is unchanged.

**Maintenance hazard, and it is real:** the two blocks must stay in sync except
for the one parameter, and nothing enforces that. `validate.py` should grow a
check; it currently does not know `FollowPathSprint` exists. This project has
already lost an experiment to a silently-ignored parameter name, and a silently
diverging duplicate block is the same class of failure.
