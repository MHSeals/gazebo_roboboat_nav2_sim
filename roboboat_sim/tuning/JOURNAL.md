# MPPI tuning journal

One change per iteration, measured against a fixed benchmark. Every entry
records the change, the evidence, and the decision — including reverts, which
are the entries most worth keeping.

## Method

```bash
tools/benchmark.sh <label> <repeats>      # 4-goal suite, ~5.5 min per repeat
tools/compare_runs.py baseline <label>
```

Ground rules, set before any results arrived so they cannot be rationalised
after the fact:

1. **One parameter per iteration.** Two changes at once means neither is
   attributable.
2. **A difference inside the baseline's own observed range is noise.** Two
   identical configs on this course have measured 0.31 m apart in minimum
   clearance. `compare_runs.py` enforces this and prints `noise` rather than
   letting a favourable number pass as a result.
3. **Revert by default.** A change stays only if it beats the noise floor on
   a metric that matters, and costs nothing material on the others.
4. **Safety before speed.** Clearance regressions are not tradeable against
   time-to-goal at this stage; the boat has to not hit buoys first.
5. **Stop when the remaining candidates are all noise-sized.** The benchmark
   cannot resolve them, and continuing would be fitting to this exact course.

## Benchmark suite

Fixed 4-goal route: `14,1,0.2  30,8,0.7  36,22,1.6  20,30,2.9`.
Goals sit in open water, not inside the inflation ring — a goal 0.5 m from a
buoy makes every config look equally bad and measures nothing about the
config. (Learned the hard way: an earlier ad-hoc goal at `26,6` sits 0.5 m
from the buoy at `26.5,6.2` and is unreachable without contact.)

## Entries

### 0a. Baseline — first attempt, discarded

Every goal returned ABORTED in 0 s across all runs. **No parameter was
changed; the harness was broken.** Two separate defects, and they compounded:

1. **`setsid cmd & ; PGID=$!` does not give you the process group.** setsid
   forks, so `$!` is setsid's own pid and `kill -- -$!` signals nothing. Every
   run leaked its entire stack. By the third run there were multiple
   `gz-sim-server` processes alive at once, publishing duplicate `/odom` and
   competing `odom -> base_link` transforms.
2. **The leaked stacks starved the machine.** Load average reached 8.1 on 4
   cores. Nav2's lifecycle manager then timed out state transitions: it
   declared `controller_server` failed ~95 ms after asking it to configure,
   while the log shows that node still cheerfully loading MPPI critics
   afterwards. Bringup aborted, so every goal was rejected instantly.

The tell that it was contamination rather than a config problem: two "different"
runs produced byte-identical distances (15.34 / 18.02 / 22.69 / 14.91 m) with
the boat sitting at (14.0, 16.3) — a position it never drove to, and not the
spawn point. That is a stale simulator's odometry, not a measurement.

Fixes, in `tools/benchmark.sh`:
- `launch_group()` has the session leader write its own pid, so the group id
  is unambiguous and cleanup actually reaches the whole tree;
- `assert_clean()` refuses to measure while any previous stack is alive;
- startup is two-phase, gated by `tools/wait_for_sim.py` on clock advancing,
  odom, scan and TF, rather than a fixed sleep.

Worth stating plainly: had the harness been trusted, the first parameter
change would have been evaluated against a baseline of "nothing works", and
any change would have looked neutral. Every number would have been fiction.

### 0b. Baseline — second attempt, also discarded

Nav2 reached `Managed nodes are active`, then 7 s later:
`CRITICAL FAILURE: SERVER behavior_server IS DOWN after not receiving a
heartbeat for 4000 ms. Shutting down related nodes.`

The lifecycle bond heartbeat is timed on the node clock, which here is sim
time. Under software rendering the sim clock stutters, a server misses the
4 s default while working perfectly well, and the manager tears the stack
down mid-goal. Set `bond_timeout: 0.0` in `nav2.launch.py`, which
`nav2_lifecycle_manager` treats as "disabled" (`createBondTimer` returns
early when the value is <= 0). Restore 4.0 on the real vehicle, where a dead
server is a safety concern and the clock is real.

### 0c. Reviewer findings that changed the plan

An independent reviewer was asked to critique the tuning plan *before* the
first parameter was chosen. It rejected the plan. The load-bearing claims
were checked against the world file and upstream source rather than taken on
trust, and they hold:

**The benchmark goals were not in open water.** Measuring the worst clearance
anywhere the goal checker will accept a pose (`xy_goal_tolerance` 0.50 m,
`yaw_goal_tolerance` 0.30 rad):

| old goal | worst clearance in the acceptance region |
| --- | --- |
| `14,1` | +0.18 m |
| `30,8` | **−0.20 m** |
| `36,22` | +0.18 m |
| `20,30` | +0.87 m |

Goal `(30,8)` sits 0.923 m from a buoy surface — inside the boat's own
circumscribed radius of 0.930 m — so the boat could legally declare success
while overlapping a buoy. `min_clearance` was largely measuring where inside
a tolerance ball the boat happened to stop, which no controller parameter
influences. The comment in `benchmark.sh` asserting these goals were in open
water was simply false.

New suite: `10,6,0.6  34.5,9.5,0.9  33,25,2.0  12,26,3.0`. Worst clearance
across each acceptance region is +1.85 m or better, while the straight line
between consecutive goals still passes through buoys (−0.20 m if flown
directly), so avoidance is exercised en route — which is the part worth
measuring.

**`docs/mppi_tuning.md` stated a mechanism that is false.** It claimed the
`(22,32)` incident goal was inside inflated space. It is 2.06 m from the
nearest buoy surface; inflation reaches 1.30 m. Clearance parked on the goal
is +1.28 m. The excursion happened en route at `(20.7, 32.7)`, 1.48 m from
the goal — also outside `CostCritic.near_goal_distance` of 1.0 m, which
rules out that exemption too. Corrected in place rather than quietly edited.

**`footprint_padding` was the wrong first experiment**, and is dropped. Its
maximum possible effect is +0.10 m of standoff against a noise floor of
0.31 m, so it was arithmetically guaranteed to return "noise". It is also not
one parameter: it shifts inscribed and circumscribed radii, which moves the
inflation cost field, the cost-aware planner's path, and `CostCritic`'s
footprint-check gate.

**Metric defects, all confirmed by reading the code:**
- `cmd_jerk` normalised by wall clock while the controller publishes on sim
  time — a config costing more CPU would score *smoother*, a bias correlated
  with exactly the parameters worth tuning. Now uses the sim clock.
- `mean_efficiency` measured straight-line distance to the goal *centre*
  while the checker accepts 0.50 m away, so stopping short scored above 1.0.
  It now measures against the pose actually reached, and `mean_error_m` is a
  first-class metric so undershoot is visible.
- `controller_hz` averaged across the idle gaps between goals. Replaced by
  p95 command period on the sim clock.
- A failed run still writes a scorecard reporting huge clearance because the
  boat never moved. `compare_runs.py` now refuses such cards; aggregating one
  would have inflated the range until every future verdict read "noise".

**Verified upstream, both real:**
- `TwirlingCritic` (Jazzy) declares only `cost_power` and `cost_weight` — no
  `threshold_to_consider`. It therefore opposes yaw right through goal
  convergence, at weight 5.0, while `GoalAngleCritic` (weight 3.0) only
  activates inside 0.8 m. Prime suspect for the progress-checker failures.
- `gz-sim8`'s `OdometryPublisher` filters velocity through
  `math::RollingMean` with `SetWindowSize(10)`. At 50 Hz that is ~100 ms of
  group delay on the velocity Nav2 feeds MPPI as its initial state, while the
  surrogate's true velocity on `boat/velocity` is never used.

**Statistical objection, accepted:** with 2 repeats, using the observed
min–max range as the noise floor labels pure noise as a directional result
about 39% of the time. Mitigation taken is not more runs but more samples per
run: closest approach is now recorded *per obstacle*, giving 10–30
near-deterministic geometric samples per run instead of one run-level
extremum.

**Applied without an A/B, on first principles:** `prune_distance` 3.0 → 4.0.
The horizon reaches 3.64 m at `vx_max`, so rollouts were extending past the
end of the pruned path and the path-align and path-follow critics had nothing
to score their tails against.

### 0d. Baseline — valid

4/4 goals both runs, no contact, controller at 20.1/20.2 Hz, 0 and 1 stack
errors (the one being the known-spurious Smac inflation warning).

| metric | r1 | r2 | spread |
| --- | --- | --- | --- |
| min clearance | 0.616 m | 0.614 m | **0.002 m** |
| mean approach (13–14 encounters) | 1.780 m | 1.869 m | 0.089 m |
| goal error | 0.279 m | 0.316 m | 0.037 m |
| time to goals | 183.5 s | 183.6 s | 0.1 s |
| path length | 89.86 m | 89.41 m | 0.45 m |
| mean speed | 0.491 m/s | 0.493 m/s | 0.002 m/s |
| p95 cmd period | 0.055 s | 0.055 s | 0 |
| odom rate | 49.2 Hz | 49.6 Hz | 0.4 Hz |

~~The instrument now resolves ~0.002 m on minimum clearance.~~ **Wrong, and
corrected in entry 1.** A 0.002 m range across *two* samples is a lucky draw,
not a specification: the expected range of two normal samples is 1.13σ, so
n=2 systematically understates spread. Two further runs (see entry 1) put the
true 4-run range at 0.035 m on minimum clearance, 0.112 m on goal error and
4.7 s on time.

What *is* true: the old 0.31 m spread was goal-region dwell, exactly as the
review predicted, and removing it made the instrument far better. Just not
150x better.

**Reviewer hypothesis falsified:** odom callbacks are *not* being dropped.
49.2–49.6 Hz against a configured 50 Hz leaves no meaningful blind spot in
the clearance metric, so the single-threaded executor is not a problem here
and `MultiThreadedExecutor` is not worth the change. Recorded because a
hypothesis that survives unchecked becomes folklore.

**Also worth noting:** zero `Failed to make progress` events on this suite.
The reviewer's proposed chain (twirling → yaw stall → progress failure →
costmap clear → blind MPPI) therefore does not manifest here, which weakens
that specific mechanism even though the structural oddity behind it is real.

### 1. TwirlingCritic weight 5.0 → 1.5

**Pre-registered before running.**

Rationale, verified in the Jazzy source rather than assumed: `TwirlingCritic`
declares only `cost_power` and `cost_weight` — it has no
`threshold_to_consider`, so it opposes yaw rate for the entire run including
final convergence. At weight 5.0 it outweighs `GoalAngleCritic` (3.0), which
is the term actually trying to satisfy `yaw_goal_tolerance`, and which only
activates inside 0.8 m. Three separate terms shape rotation
(`TwirlingCritic`, `GoalAngleCritic`, `PathAngleCritic mode: 0`) and they do
not agree near the goal.

Predictions, stated in advance so they cannot be revised afterwards:
- `mean_error_m` **decreases** (currently 0.28–0.32 m, with goal 1 pinned at
  0.43–0.45 m near the 0.50 m tolerance edge)
- `reached_seconds` **decreases** — the boat currently averages 0.49 m/s
  against a 1.30 m/s limit; less rotational fighting should cost less time
- `min_clearance` / `encounter_min_mean` **unchanged** — this critic has no
  obstacle term
- Risk: an omni boat with little twirling penalty may crab and yaw
  simultaneously, which could *raise* `cmd_jerk` and look worse in the video

Reject if clearance regresses at all, or if error and time both fail to
improve beyond the baseline spread.

**Result: INVALID — the experiment never ran.** Then reverted anyway.

`TwirlingCritic` was configured with `twirling_cost_power` /
`twirling_cost_weight`. Jazzy's `TwirlingCritic::initialize()` reads
`cost_power` / `cost_weight`. Undeclared keys in a ROS parameter file are
silently ignored, so **the critic ran at its 10.0 default in all four runs** —
baseline and candidate alike. Both arms were identical.

Proof, from every `controller_server` startup line in all four runs:

```
TwirlingCritic instantiated with 1 power and 10.000000 weight.
```

The wrong keys came from Nav2's own MPPI README, whose example config uses
`twirling_cost_power` / `twirling_cost_weight`. The README is wrong. Anyone
copying it gets a silently ignored critic.

Worse: the evidence was already on screen. Earlier in the session the Jazzy
source was fetched and printed `getParam(weight_, "cost_weight", 10.0f)` —
and the mismatch with the config was not noticed. The reviewer caught it.

Three consequences worth keeping:

1. **`twirl1p5_r1/r2` are not an experiment, they are baseline repeats 3 and
   4.** Renamed accordingly. They are the most useful data in the session: an
   accidental positive control, and the only reason the true noise floor is
   known.
2. **The pre-registered prediction "`mean_error_m` decreases" came true —
   from a change that did nothing** (0.297 → 0.235, non-overlapping ranges).
   Pre-registration does not protect against a broken manipulation. Only a
   control does.
3. `tools/check_critics.py` now compares every configured critic weight
   against what the controller logged at startup, and `benchmark.sh` runs it
   on every benchmark run. A dead key can no longer pass silently.

The original (meaningless) numbers, kept for the record:

| metric | baseline | twirl 1.5 | verdict |
| --- | --- | --- | --- |
| min clearance | 0.615 [0.61–0.62] | 0.595 [0.58–0.61] | noise |
| mean approach | 1.825 | 1.788 | noise |
| goal error | 0.297 [0.28–0.32] | 0.235 [0.20–0.27] | noise |
| time to goals | 183.6 [183.5–183.6] | 182.9 [180.6–185.3] | noise |
| command accel | 0.18 | 0.16 | noise |

Goal error is the one metric whose ranges do not overlap, so there is
probably a real effect of about −0.06 m. It is worthless: both values sit
comfortably inside a 0.50 m goal tolerance, so improving it buys nothing.
Time moved 0.7 s out of 183 s, and the candidate's own two runs differ by
4.7 s. Predicted improvements did not materialise at any size worth having,
so by ground rule 3 the change does not stay.

The registered risk (an omni hull yawing and crabbing at once, raising
command acceleration) did **not** appear — command accel went slightly down,
not up. Recorded because the prediction was wrong in the safe direction and
that is still a wrong prediction.

**Methodological note, stated against interest.** The first scoring of this
experiment, before the comparison rule was hardened, read `better` on goal
error, `better` on time and `WORSE` on min clearance — and on that basis the
change might have been kept, or the clearance regression treated as real.
Two flaws caused it: the floor used only the *baseline's* spread, so a noisy
candidate looked decisive; and there was no minimum practically-meaningful
effect, so 0.7 s out of 183 s and 0.6 Hz of odom jitter registered as
results. `MIN_EFFECT` now sets a floor per metric from what matters to the
vehicle, and the floor is the widest of baseline spread, candidate spread and
that minimum.

Those thresholds were written **after** seeing this run's numbers, which is a
real hazard — it is exactly how a rule gets tuned to produce a desired
verdict. Two things limit the damage here: the question "does the min-max
rule become too sensitive once the spread is tiny, and what minimum effect
sizes should I use" was put to the reviewer *before* these results existed,
and both the old and new rules lead to the same decision to revert. The
thresholds should be treated as binding from the next experiment onward, not
as a lens to re-read this one.

### 1b. TwirlingCritic 10.0 → 1.5, with the correct key

Same intent as entry 1, corrected. The real delta is 6.7x, not the 3.3x
intended. The mechanism in entry 1 was also wrong and is restated here:
`TwirlingCritic::score()` early-returns inside `xy_goal_tolerance`, so it does
**not** fight yaw during final convergence. It penalises `mean(|wz|)` over the
whole horizon *everywhere else*, which is where it bites — on curved legs.

The speed data supports that reading. Per-leg, speed tracks path directness
almost exactly, and the shortest leg with both a standing start and a full
stop is the *fastest*, so stopping overhead is not the limiter:

| leg | directness | m/s |
| --- | --- | --- |
| 1 | 0.984 | 0.673 |
| 2 | 0.868 | 0.578 |
| 3 | 0.724 | 0.391 |
| 4 | 0.774 | 0.457 |

At 0.49 m/s the hull uses ~17 N of the 99 N available, so it is nowhere near
thrust-limited. Curvature is the cap.

Prediction: `reached_seconds` drops by 10–30 s from 183.5 s. **If it moves
less than 5 s, the mechanism is dead and speed gets dropped as a target.**

**Result: REVERTED — the prediction was backwards.** Manipulation verified
this time (`TwirlingCritic instantiated with 1 power and 1.500000 weight`
against the baseline's `10.000000`).

| metric | baseline (4 runs) | twirling 1.5 | verdict |
| --- | --- | --- | --- |
| time to goals | 183.2 [180.6–185.3] | **203.9** [203.4–204.4] | **WORSE** |
| path length | 89.5 [89.0–89.9] | **94.3** [94.2–94.4] | **WORSE** |
| path directness | 0.837 [0.83–0.84] | **0.798** | **WORSE** |
| min clearance | 0.605 | 0.568 | noise |
| goal error | 0.267 | 0.191 | noise |

Predicted 10–30 s faster; measured **20.7 s slower**, with the two candidate
runs agreeing to 1 s. Not a marginal miss — the sign is wrong.

Per-leg, the damage is concentrated on leg 2, the long channel run:

| leg | baseline m/s | twirling 1.5 m/s |
| --- | --- | --- |
| 1 | 0.666–0.686 | 0.661–0.679 |
| 2 | **0.556–0.578** | **0.478–0.489** |
| 3 | 0.373–0.418 | 0.400–0.401 |
| 4 | 0.444–0.480 | 0.441 |

**Why the mechanism was backwards.** The reasoning treated yaw penalty as a
brake on curved legs. For *this* hull it is the opposite: sway costs roughly
three times the thrust of surge (`vy_max` 0.45 against `vx_max` 1.30), so the
efficient way to move is to point along the direction of travel. A strong
twirling penalty forces exactly that. Weaken it and MPPI is free to pick
trajectories that crab and rotate at once — which is why path length grew
4.8 m and directness fell while speed dropped. `TwirlingCritic` at 10.0 is
not a speed limiter; it is what keeps an omni hull from swimming sideways.

**Restored to 10.0, not 5.0.** The file said 5.0 before the key was fixed,
but that value was never in effect. Every measurement ever taken on this
workspace ran at 10.0, so 10.0 is the only setting with evidence behind it.
Reverting to the number written in the file would have shipped an untested
config.

### 2. TwirlingCritic 10.0 → 20.0

**Pre-registered.** The effect is monotonic across the two points measured
(1.5 → slow, 10.0 → faster), and the mechanism now has a physical reading:
more yaw penalty means the hull points along travel and stops paying sway
drag. This tests whether 10.0 is already past the useful part of that curve.

Prediction: `reached_seconds` improves by **≥5 s** from 183.2 s, driven by
leg 2. Risk: too much yaw penalty should eventually prevent the boat turning
into gates at all, which would show as *worse* directness on legs 3 and 4, or
a goal it cannot satisfy the yaw tolerance on.

**Stopping rule: if this gains less than 5 s, the twirling axis is done and
so is speed as a target.** Two points either side of 10.0 would then bracket
it, and further runs on this axis would be fitting noise.

**Result: REVERTED. The stopping rule fires — the twirling axis is closed.**

Manipulation verified (`instantiated with 1 power and 20.000000 weight`).
One of the two runs was lost to a harness timeout (see below), so n=1.

| metric | baseline (4 runs) | twirling 20 (1 run) |
| --- | --- | --- |
| time to goals | 183.2 [180.6–185.3] | **187.6** |
| min clearance | 0.605 [0.58–0.62] | **0.533** |
| mean approach | 1.806 | **1.446** |
| path directness | 0.837 | 0.851 |

No speed gain — 4.4 s *slower*, not the ≥5 s faster required — and clearance
regressed on both measures.

n=1 is normally too thin to act on, but here it is sufficient, and the
arithmetic says why. Keeping the change needs a mean ≤ 178.3 s. With one run
at 187.6 s, a second would have to come in at **≤ 168.9 s**. The fastest run
ever observed on this workspace, in any configuration, is 180.6 s. No second
run can rescue it, so buying one is 5.5 minutes spent on a foregone
conclusion.

Combined with entry 1b, 10.0 is now bracketed: 1.5 is much worse (+20.7 s),
20.0 is no better (+4.4 s). **10.0 is at or near the optimum on this axis.**
That it also happens to be the Nav2 default is worth noticing — the original
config's "5.0" was never in effect, so this workspace never actually departed
from the default, and testing either side of it found nothing better.

`twirling_cost_weight` is restored to 10.0 permanently.

**Harness fix prompted by the lost run.** Run 1 reported "navigate_to_pose
server never came up" with zero stack errors — Nav2 started cleanly, just
slower than the fixed 20 s sleep after launch. `benchmark.sh` now polls for
the action server for up to 2 minutes instead of sleeping. A fixed sleep was
the same class of mistake as the fixed sim-startup sleep replaced earlier by
`wait_for_sim.py`, and it cost half an experiment.

## Status: speed and clearance are both closed

| target | verdict |
| --- | --- |
| clearance | **closed** — 0.616 m measured against a 0.65 m geometric maximum; 95% of achievable |
| speed via twirling | **closed** — 10.0 bracketed on both sides, nothing better |
| p95 command period | **closed** — a 5 ms sim-tick quantisation artefact, not an overrun |
| odom loss | **closed** — 49.2–50.1 Hz against 50 configured |

### Corrections to entry 1b, from review

**The mechanism I gave was wrong.** I wrote that MPPI avoids sway because it
costs ~3x the thrust of surge. MPPI cannot know that. Verified in the Jazzy
source:

- `ConstraintCritic` (Omni) sets `max_vel_ = sqrt(vx_max² + vy_max²)` = 1.376
  m/s and penalises only `maximum(vx - max_vel_, 0)`. At 0.49 m/s the cost is
  exactly zero.
- `MotionModel::predict()` is purely kinematic — it clamps accelerations and
  nothing else.
- `PreferForwardCritic` scores `maximum(-vx, 0)`: reverse surge only. **Sway
  is free.**

There is no thrust, energy or sway term anywhere in the cost function. The
*plant* pays for crabbing; the optimiser is blind to it.

The correct reading is less flattering and more useful: outside 0.8 m of a
goal (`GoalAngleCritic`) and away from obstacles, **`TwirlingCritic` is the
only term in the entire cost function that constrains yaw at all** —
`PathAlignCritic` has `use_path_orientations: false`, and `PathAngleCritic`
gates on the *current* pose with `max_angle_to_furthest: 1.0` (57.3°), so it
contributes nothing in normal driving. Yaw is otherwise an unobserved DOF. At
weight 1.5 the regulariser stops working, heading random-walks under
`wz_std` noise, and the only way to convert a heading error into progress is
`vy` — hard-clamped at 0.45. **The limiter is the config value `vy_max`, not
hull thrust.** That names a knob that can actually be turned.

**"Damage concentrated on leg 2" was also wrong.** Measured in time rather
than m/s:

| leg | Δ time | Δ path |
| --- | --- | --- |
| 1 | +0.45 s | +0.19 m |
| 2 | **+11.27 s** | +1.20 m |
| 3 | +1.25 s | +1.08 m |
| 4 | **+7.67 s** | **+2.35 m** |

Leg 4 is 37% of the loss and is a *different* failure: its path grew 2.35 m
at essentially flat speed, while leg 2 lost 11 s for 1.2 m. The m/s-only
table collapsed two mechanisms into one.

**`path_length_m` was duration-biased, and it inflated that result.**
Summing `hypot` over every 50 Hz odom frame adds error proportional to sample
*count*, so a slower run scores a longer path with identical geometry.
Demonstrated within the baseline itself, where the config never changed: on
leg 3 the slowest run scored +0.98 m over the fastest for +8.7 s — about
2.4 mm per extra sample. So the twirl-1.5 verdict of "time WORSE, path
length WORSE, directness WORSE" was **not three independent confirmations**;
roughly 0.9 m of leg 4's 2.35 m was measurement bias. The regression is real
but was overstated. Path length now accumulates from 10 Hz-decimated samples.

**`PathAlignCritic.offset_from_furthest` is documented wrong in this repo.**
It is not an alignment offset: in `path_align_critic.cpp` it is a
minimum-index gate (`if (path_segments_count < offset_from_furthest_)
return;`). The value 6 was derived from a rule of thumb for a parameter that
does not have that meaning — the same failure class as the dead twirling key,
on the highest-weighted critic in the file. Comment corrected; the value is
untested and now flagged as a candidate.

### 3. Diagnostic run (no config change)

Legs 3 and 4 are the slowest with the worst directness in **every** run
including all four baselines — structural, not noise. The scale is worth more
than anything the twirling axis could deliver:

- **Leg 4's straight line is completely clear.** Chord 21.02 m; the nearest
  obstacle is 2.59 m off it, and `leg_min_clearance_m` is 3.07–3.25 m in
  every run. The boat drove 26.4–27.2 m. That is ~6 m of excess in open
  water, and it bows *away* from the only nearby buoy, so it is not
  avoidance.
- **Leg 3's straight line genuinely is blocked** (a buoy 0.07 m off it), but a
  minimum feasible route is ~15.9 m against 20.8–22.2 m measured.

Together ~11 m of the 90 m path and ~25 s of the 183 s, versus the ~20 s the
whole twirling axis could move.

Rather than guess between candidates, this run records a full trace with the
global plan, so the question "is the excess in the plan, or in MPPI's
tracking of it?" is answered by data. No parameter changes.

**Result: the excess is entirely in tracking. The planner is near-optimal.**

⚠️ **First analysis of this run was wrong and is corrected below.** It
reported leg 3 as a "planner problem" with a route 58% longer than the chord,
and that was called the largest single inefficiency found. It does not exist.
Goal 2 aborted in this run, ending 18.9 m from its target, so the boat began
leg 3 from (16.4, 7.0). The analysis measured each leg's chord from the
*nominal* previous goal (34.5, 9.5) rather than from where the boat actually
was. Measuring from actual positions:

| leg | reached | chord | plan | drove | plan/chord | drove/plan |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | yes | 11.66 | 12.06 | 11.45 | 1.034 | 0.949 |
| 2 | **no** | 24.98 | 24.51 | 6.21 | 0.981 | 0.253 |
| 3 | yes | 24.91 | 24.62 | 31.06 | **0.989** | **1.262** |
| 4 | yes | 20.87 | 20.88 | 26.28 | **1.000** | **1.259** |

**Smac is producing essentially optimal routes on every leg** (plan/chord
0.98–1.03). There is no planner detour to chase.

**Both slow legs are the same failure: MPPI drives 26% further than a
near-optimal plan.** Leg 1, which is short and nearly straight, tracks at
0.95. So the excess correlates with leg length and initial heading error, not
with the route.

The lesson is the same one the harness bugs taught: a derived quantity is only
as good as the frame it is measured in. A failed goal silently invalidated
every downstream chord in that table, and the resulting number was confident,
specific, and completely wrong.

**The boat crabs, continuously.** Measured over the moving portion of each
leg:

| leg | mean \|vy\| | sideways fraction of velocity | mean crab angle |
| --- | --- | --- | --- |
| 3 | 0.086 m/s | 38% | 54.8° |
| 4 | 0.201 m/s | **58%** | 59.9° |

This is the direct confirmation that the earlier "MPPI avoids sway" claim was
backwards. It does not avoid sway; it crabs with the majority of its velocity
sideways for most of a leg, because nothing in the cost function charges for
it.

**Leg 4's excess is not weaving.** Zero cross-track direction reversals, a
single smooth 1.96 m bow. The along-track profile shows what it actually is:
the boat reaches 22.6 m along a 20.9 m chord at t=39 s — **1.7 m past the
goal** — then spends the remaining ~22 s settling back while heading swings
122° → 102° → 79° → 148° at near-zero speed.

The critic band governing that end-game is worth writing down:

```
TwirlingCritic   weight 10.0, switches OFF inside xy_goal_tolerance = 0.50 m
GoalAngleCritic  weight  3.0, switches ON  inside threshold_to_consider = 0.80 m
```

Between 0.50 m and 0.80 m from the goal both are live, and the term opposing
yaw outweighs the term demanding it **10:3**. That is the structural concern
raised in review at iteration 2, which was set aside because there were no
progress-checker failures. It does not need a progress failure to cost time.

### 4. vy_max 0.45 → 0.20

**Pre-registered.** `vy_max` is the only place in the entire stack where sway
is constrained — `ConstraintCritic` does not bind below 1.376 m/s,
`PreferForwardCritic` scores reverse surge only, and the motion model is
kinematic. The trace shows the boat using 58% of its velocity sideways, so
this is the knob that decides whether it may.

This is also the change most likely to *generalise*: it encodes a true
property of the hull (sway costs roughly three times the thrust of surge)
rather than a preference tuned to this course's turn geometry.

Predictions:
- mean crab angle **falls well below 55°**
- `path_length_m` **falls 3–6 m**, concentrated on legs 3 and 4
- `reached_seconds` **falls 8–20 s**
- Risk: less lateral authority in the 2.40 m gates. **Reject if
  `encounter_min_mean` drops by more than 0.10 m, regardless of time.**

**Result: UNDER-POWERED — the constraint barely bound. Not a fair test.**

Every metric came back noise: time 183.2 → 181.2 s (−2.0 s, floor 5.0), path
89.5 → 88.5 m (−1.0 m, floor 2.0), clearance unchanged. 4/4 goals both runs.

Then the obvious check, on data that already existed before the run was
launched. |vy| in the baseline trace:

| percentile | \|vy\| |
| --- | --- |
| p50 | 0.115 m/s |
| p75 | 0.240 |
| p90 | 0.313 |
| p100 | 0.397 |
| mean | 0.147 |

**A cap of 0.20 binds only the top 34% of samples.** To constrain the median
it needed roughly 0.12. The "mean |vy| = 0.201 on leg 4" figure quoted when
choosing the value was a per-leg mean over moving samples — not the
distribution the cap acts on — and it made 0.20 look binding when it is not.

This is an analysis error, not a measurement error: the trace needed to size
the experiment was already recorded and sitting in `tuning/`. Sizing an
intervention against the distribution it acts on should have preceded the
run, and would have cost nothing.

### 4b. vy_max 0.45 → 0.10, properly sized

Below the p50 of 0.115, so it binds the majority of samples rather than the
tail, and it materially shrinks the reachable set MPPI samples from rather
than merely clipping outputs.

Same predictions and the same rejection criterion as 4. Additionally:
**if this is also noise, the crabbing hypothesis is dead** — a cap below the
median that changes nothing means sway was never what cost the time, and
`vy_max` gets restored to 0.45 permanently.

**Result: REJECTED on the pre-registered criterion, and worse than noise.**

| metric | baseline | vy_max 0.10 | verdict |
| --- | --- | --- | --- |
| goals reached | 4.0 | **3.5** | one goal not reached |
| mean approach | 1.806 | **1.646** | **WORSE — 0.160 m, over the 0.10 limit** |
| goal error | 0.267 | **0.419** | WORSE |
| mean speed | 0.491 | 0.455 | WORSE |
| min clearance | 0.605 | 0.700 | better |
| path directness | 0.837 | 0.869 | better |

The registered rejection criterion fires: `encounter_min_mean` fell 0.160 m
against a 0.10 m limit. Harder still, **run 2 failed goal 4 outright** —
status 6 after 69 s at 0.46 m from target, i.e. *inside* the 0.50 m position
tolerance but unable to satisfy yaw within the budget.

The crabbing hypothesis is not merely dead, it is inverted. **Sway authority
is what the boat uses to converge on a pose.** Constrain it below the median
and the hull can still travel, but it can no longer finish: it arrives, sits
inside the position tolerance, and cannot rotate itself into the yaw
tolerance without the lateral authority to hold station while turning.

There is a real trade-off here worth recording rather than a simple loss.
Capping sway **did** make paths straighter (directness 0.837 → 0.869) and
passes wider (min clearance 0.605 → 0.700), exactly as predicted. It just
costs more than it buys. `vy_max` is a lever between path directness and
goal-convergence capability, and 0.45 is on the right side of it.

`vy_max` restored to 0.45 permanently.

### Retracted: there is no planner detour

Recorded here because it was reported as a finding before being checked.
Smac's routes are 0.98–1.03x the chord on every leg. The planner is not worth
tuning; all remaining excess is in how MPPI tracks it, and within that, the
trace points at the terminal phase — leg 4 overshoots 1.7 m past the goal and
then spends ~22 s settling while its heading swings through 70°.

### Closed: clearance is finished, do not tune it further

The narrowest gates (`chan_b`, `chan_c`) are 2.8 m centre-to-centre with
0.20 m buoy radii, leaving **2.40 m of free water**. The hull is 1.10 m in
beam. Perfectly centred and aligned, the maximum achievable per-side
clearance is `(2.40 − 1.10)/2 = 0.65 m`.

Measured minimum clearance is **0.616 m — 95% of the geometric maximum**, and
the remaining 0.034 m is below the 0.05 m `MIN_EFFECT` floor. For scale, 10°
of yaw misalignment alone costs 0.12 m of swept half-width. Median clearance
is 3.1 m and p05 is 0.94 m, so the boat is far from everything 95% of the
time.

There is no competition value in more clearance, and the documented "open
issue" was never a controller problem. Any further work here is a rabbit
hole.

### Ruled out, so they are not re-chased

- **p95 command period 0.055 s vs a 0.050 s target is not an overrun.** The
  sim step is 5 ms, so 0.055 is the smallest representable deviation — one
  tick on ~5% of cycles. `controller_hz` is 20.13–20.17 against 20.0. Cutting
  `batch_size` or disabling `consider_footprint` to chase it would trade real
  footprint-accurate collision checking for a quantisation artefact.
- **Odom callback loss:** 49.2–49.6 Hz against 50 configured. Not happening.
- **`consider_footprint: true` CPU cost:** affordable, per the period data.
  Keep it.

### Latent, documented rather than fixed

`planner_server.GridBased.tolerance: 1.0` lets Smac return a path ending up
to 1 m short of the requested goal, and MPPI takes its target from the *last
pose of the global plan*, not the action goal — while `SimpleGoalChecker`
measures against the true goal. On the current suite every goal sits in free
water so the plan reaches the exact cell and nothing truncates. It will bite
the moment a goal is set near an obstacle, and would present as unexplained
goal error.

### 2. Next experiment — selection pending

Everything measured on this suite is now noise, and the config already
achieves 4/4 goals, no contact, 0.61 m minimum clearance and a controller
holding its period. The remaining candidate with genuine headroom is speed:
the boat averages **0.49 m/s against a 1.30 m/s limit**, taking 183 s for a
90 m path. Whether that is a tuning deficiency or simply the cost of four
full stops with a yaw tolerance to satisfy is the open question, and is with
the reviewer.

Ground rule 5 applies: if the remaining candidates are all noise-sized on
this benchmark, the honest move is to stop rather than keep spending runs.

### 5. GoalAngleCritic 3.0 → 12.0 — final experiment

**Pre-registered.** The last hypothesis the trace supports, and the only one
this session derived from measurement rather than intuition.

```
TwirlingCritic   weight 10.0, OFF inside xy_goal_tolerance    = 0.50 m
GoalAngleCritic  weight  3.0, ON  inside threshold_to_consider = 0.80 m
```

Between 0.50 m and 0.80 m both are live and the term opposing yaw outweighs
the term demanding it 10:3. The trace shows the cost: leg 4 overshoots 1.7 m
past the goal at t=39 s, then spends ~22 s of a 62 s leg settling while its
heading swings 122° → 102° → 79° → 148° at near-zero speed. Legs 3 and 4 have
the largest final yaw changes (115° and 172°) and are the slowest in every run.

Raising `GoalAngleCritic` makes it dominant in exactly that band without
touching `TwirlingCritic`, already bracketed as optimal at 10.0 en route.

Predictions: `reached_seconds` falls **≥5 s** concentrated on legs 3 and 4;
`mean_error_m` falls from 0.267; clearance and goals reached unchanged.

Registered risks: a stronger yaw term near the goal may oscillate (higher
`cmd_jerk`) or pull the boat off the path on approach (lower directness).

**Reject if** `encounter_min_mean` drops >0.10 m, any goal is not reached, or
the gain is under 5 s.

**This is the last experiment.** Continuing past the last supported hypothesis
is how tuning becomes fitting noise.

**Result: REJECTED — and this is the hard one.**

| metric | baseline | GoalAngle 12.0 | verdict |
| --- | --- | --- | --- |
| time to goals | 183.2 [180.6–185.3] | **175.1** [172.8–177.3] | **better, −8.1 s** |
| goals reached | 4.0 | 4.0 | unchanged |
| min clearance | 0.605 | 0.593 | noise |
| mean approach | 1.806 | **1.615** | **WORSE, −0.191 m** |
| goal error | 0.267 | 0.344 | noise |

**The prediction was met** — ≥5 s was registered, 8.1 s was delivered. That is
the first correct prediction of the session, and the mechanism it was derived
from (the 10:3 imbalance in the 0.50–0.80 m band) is the only one that came
from measuring rather than reasoning.

**The registered rejection criterion also fired**, at nearly twice its limit:
`encounter_min_mean` fell 0.191 m against a 0.10 m ceiling. Ground rule 4,
written before any result existed, is explicit: *safety before speed;
clearance regressions are not tradeable against time-to-goal at this stage.*

So it is rejected. Worth being clear about why that is not a formality: this
is the most promising candidate found all session — a real 4.4% time gain
with the worst-case clearance unchanged (0.593 vs 0.605, noise). Every
incentive points at rewriting the criterion, arguing that mean approach is
not really a safety metric and that `min_clearance` is the one that matters.

That argument might even be correct. It is still not available now. The
criterion was chosen in advance, knowing what it measured, precisely so that
a favourable time number could not be used to renegotiate it afterwards. A
rule that only binds when it costs nothing is not a rule.

**What to do with it instead:** settle whether `encounter_min_mean` is the
right safety metric *before* re-testing, on its own merits and with no result
in view. If the conclusion is that worst-case clearance is what matters and
mean standoff is not, then re-run this experiment against the corrected
criterion. That is a legitimate path; reinterpreting today's numbers is not.

`GoalAngleCritic` restored to 3.0.


## Loop closed

Five experiments, sixteen benchmark runs, **zero parameter changes kept.**

That is the honest outcome, and it is not a null result. The config was
already at or near a local optimum on every axis that could be tested, and
most of the session's value was in finding that out and in fixing the
instrument that was hiding it.

### Parameters tested and settled

| parameter | tested | verdict |
| --- | --- | --- |
| `TwirlingCritic.cost_weight` | 1.5 / 10 / 20 | 10.0 bracketed on both sides |
| `vy_max` | 0.45 / 0.20 / 0.10 | 0.45; below the median the boat cannot finish |
| `GoalAngleCritic.cost_weight` | 3.0 / 12.0 | 12.0 gains 8.1 s but fails the safety criterion |
| `prune_distance` | 3.0 → 4.0 | changed on first principles; the stated benefit was later shown not to follow |
| clearance generally | — | closed on geometry: 0.616 m of a 0.65 m maximum |

### Bugs found, all of which invalidated measurements

1. `setsid cmd & ; PGID=$!` never captured the process group; every run leaked
   its stack until load starved Nav2's lifecycle manager.
2. Lifecycle bond heartbeats timed against a stuttering sim clock tore the
   stack down mid-goal.
3. `twirling_cost_weight` is a dead key — Nav2's own README documents it, the
   code reads `cost_weight`. An entire experiment measured nothing.
4. `reached_seconds`, `cmd_jerk` and the command period were on wall clock
   while the controller runs on sim time, biasing exactly the parameters
   under test.
5. `path_length` accumulated 50 Hz odom noise, making it a proxy for duration
   rather than distance.
6. `mean_efficiency` measured to the goal centre inside a 0.50 m tolerance, so
   it rewarded stopping short.
7. Three successive wrong readiness gates: a fixed sleep, then the action
   *existing* rather than being *active*.
8. Benchmark goals sat inside the inflation ring — one of them inside the
   hull's own circumscribed radius — so clearance measured parking position,
   not control.

### Claims of mine that turned out to be false

- "The instrument resolves 0.002 m" — a two-sample range read as a
  specification; the true 4-run range is 0.035 m.
- "MPPI avoids sway because sway costs thrust" — nothing in the cost function
  charges for sway; the trace shows the boat crabbing at ~60°.
- "Leg 3 is a planner problem, 58% longer than the chord" — an artefact of
  measuring from a goal the boat never reached. Smac is near-optimal
  everywhere.
- "Damage concentrated on leg 2" — leg 4 is 37% of it, and a different
  failure.
- Three predictions registered in advance were wrong, one of them
  (`mean_error_m` decreasing) *confirmed* by a manipulation that never
  applied. Pre-registration does not protect against a broken manipulation;
  only a control does.

### If this is picked up again

The single highest-value next step is **not** another parameter. It is the
holdout the review specified and this session never ran: the same route
reversed, at least one leg demanding >120° of heading change, a non-zero
spawn yaw, and ±0.3 m jitter on spawn and goals. Four baseline runs spreading
only 4.7 s is a measure of determinism, not robustness — every conclusion
here is conditional on a course whose hardest turn is 63° from a standstill,
with no disturbance at all.

---

# Phase 2 — the full course and the speed challenge

Picked up after the channel task was passing 10/10. The request: rebuild the
sim to the official course, build the speed challenge, provision for the rest,
and do not undo the channel while doing it.

## The independent review, before any course edit

Same pattern as the tuning loop: a reviewer read the plan and the tree before
anything was changed. It rejected five things, and it was right about four of
them. Recording all five, including the one it got wrong, because the
distinction is the useful part.

**Right, and it changed the design.**

1. *The regression contract was frozen from n=1.* The first version of
   `regression_channel.json` set a 0.25 m clearance floor from a single 0.380 m
   observation. Three more runs of the *unchanged* configuration came back at
   0.320, 0.245 and 0.376 m. The 0.245 run would have failed a floor set from
   the 0.380 run — a clean false positive on the very next commit, and exactly
   the failure the journal's own n=2 argument predicts. The spread is
   sd 0.063 m on a mean of 0.330: **four times** the 0.035 m range measured on
   the open-water benchmark suite. Threading a 2.4 m gate is a much noisier
   thing to measure than parking in open water, and the old suite could not
   have shown that.

2. *The global costmap is an 80 m ceiling with 3.5 m of margin.*
   `ComputePathThroughPoses` plans against one robot-centred rolling costmap,
   so every goal still in the list has to be inside the window — not just the
   next one. Measured span of the channel task from spawn: **36.5 m against a
   40 m half-window**. Nothing had ever said so. `validate.py` now checks it,
   and it is the strongest argument for running one task per action call:
   the window has to hold a task, not a course.

3. *`if offset` reports None for a perfectly centred crossing*, which is
   indistinguishable from a miss — in a field that was about to go into a
   contract.

4. *No elapsed time in the scorecard.* You cannot write a contract for a
   **speed** challenge without it. Added, on the sim clock.

**Wrong, but the fix it proposed was right anyway.**

5. It argued the ring waypoints would be retired two at a time by
   `RemovePassedGoals`, and that no choice of ring radius could prevent it. The
   arithmetic offered for that ("every chord midpoint is within 1.6 m of both
   endpoints") describes a moment the boat has already passed — the front
   waypoint pops on approach, well before the midpoint, and the next one is
   3.6 m away at that instant. But it does not matter: the design it proposed
   instead — a sprint-specific tree with a **0.8 m** retirement radius, and the
   approach and stand-off poses moved out of the gate into open water — is
   correct under either reading and removes the question entirely. The 1.6 m
   radius exists only because the channel's waypoints are gate centres sitting
   inside inflated space. No waypoint of the sprint task is.

## What "keep the channel intact" was taken to mean

Not "the same numbers" — the reviewer's better idea was to not move the channel
at all. The ten channel gates keep their exact positions and widths, and the
sprint geometry goes into the empty water south of them. So the channel's
absolute metrics stay comparable, and the contract is anchored on a task whose
geometry did not change.

The official dimensions that *are* now in the file: buoy radii (Polyform A-0
0.102 m, A-2 0.184 m, Sur-Mark 0.229 m), the sprint gate width 2.4 m inside the
documented 6–10 ft, and the 13.0 m gate-to-mark distance inside the documented
40–100 ft. Radii matter because they set a gate's free water and therefore the
clearance the controller has to work with. Heights are deliberately *not*
official: a real A-0 stands 0.15 m above the water and a scan plane at 0.35 m
would miss it entirely.

## The contract that replaced the absolute floor

`min_clearance / ((free_water - beam) / 2)` — clearance as a fraction of the
most the hull could possibly clear the tightest gate by. Widening or narrowing
a gate moves numerator and denominator together, so the criterion means the
same thing before and after a geometry change, which is the property the first
version of this contract claimed was impossible to have. Floor 0.20, which is
mean − 3sd (0.217) rounded down. Measured fractions across the four runs:
0.585, 0.492, 0.377, 0.578.

Negative-controlled both ways: a run through *wider* gates with the same
fraction passes, and a run through *narrower* gates whose clearance looks
healthy in metres but is a sixth of what was available fails.

## Pre-registered predictions for the first sprint run

Written before the run, so they can be wrong.

1. **It completes.** 1.00 ± 0.03 laps, no contact, entrance gate transited
   outbound and inbound.
2. **It is slow.** Predicted 130–190 s of sim time for roughly 60 m of path,
   i.e. a mean of 0.3–0.45 m/s against a 1.30 m/s limit. `TwirlingCritic` at
   weight 10.0 penalises `mean(|wz|)` over the whole horizon, and circling at
   4 m demands a sustained 0.12 rad/s at the channel's measured 0.49 m/s. Two
   of the three cheapest ways for MPPI to reduce that penalty are to slow down
   and to crab, and sway is free in this cost function.
3. **The driven radius exceeds 4.0 m.** `CostCritic` with `consider_footprint`
   pushes away from the mark's inflation, and nothing pulls the boat inward
   except `PathAlignCritic`. Predicted closest approach to the mark 4.1–5.0 m.
4. **The obstacle inside the loop is not the binding constraint.** It sits
   1.5 m off the mark, so the ring passes 2.5 m from its centre and 1.77 m from
   its surface. Predicted min clearance is against the *gate*, not the
   obstacle.

If (1) fails the task framework is wrong. If (2) is badly wrong in the fast
direction, the reviewer's reading of `TwirlingCritic` is wrong and the
open-water benchmark's speed numbers were being set by something else.

## The sprint: three runs, two geometry bugs, and how the predictions did

### Prediction 1 — "it completes" — WRONG, twice, for two different reasons

Both failures looked like controller problems. Both were geometry.

**Run 1: the route returned to its own start.** The boat drove to the first
waypoint at (10, −3), stopped dead, and held station there to within 2 cm for
265 s while the controller logged "failed to make progress" every 20 s. The
plan was correct — 242 poses, the whole route — and the boat ignored it.

The cause: the approach pose and the exit stand-off pose were the same point,
so the concatenated path both began and ended under the hull. MPPI takes its
target from the global path; with the path's last pose already reached there is
nothing to move toward. Nothing in the logs says this. The evidence was the
trace: perfect station-keeping is not a controller struggling, it is a
controller that believes it has arrived.

Fix: a task may return more than one *leg*, each sent as its own
`NavigateThroughPoses` goal. Out through the gate and round the mark is one
journey; coming back out is another.

**Run 2: the last ring waypoint landed on top of the first.** The boat went out
through the gate, reached the first ring waypoint, turned round and came home.
Swept 0.01 rad. Gate transited both ways, no contact, and completely wrong.

The cause was the fix for a *different* problem. To stop the lap ending exactly
on the entry bearing I had added one extra ring segment — and with 8 points a
whole extra step is 360° + 45°, which is the first ring waypoint again. So
arriving at ring waypoint 1 satisfied the leg's final goal.

Fix: the overshoot is now a *fraction* of a step (half), and the ring is split
across the two legs so that no leg's final goal lies on its own earlier route.
`test_task_scoring.py` now asserts that directly, for every leg of every task —
including against the straight line between consecutive waypoints, because the
goal checker fires on proximity to the path, not to a waypoint.

**Run 3 circled correctly and still scored FAIL**, at 6.270 rad against a
required 2π = 6.2832. That one was the *scorer* being wrong. Entering and
leaving through the same gate means the sweep is measured between two crossings
at different lateral offsets, so a whole turn does not measure as exactly 2π.
The threshold is now relaxed by the angle the gate subtends at the mark —
0.184 rad here — which is the actual resolution of the instrument. It cannot
rescue a lap that was not driven: entering and exiting through one gate
quantises the winding number, and the nearest reachable wrong answer is three
quarters of a turn, 1.57 rad short, roughly eight times the slack.

### Prediction 2 — "it is slow" — RIGHT, mechanism plausible, band slightly off

Predicted 130–190 s and 0.30–0.45 m/s. Measured **138.6 s** and **0.49 m/s**
overall, **0.42 m/s while actually circling**, against **0.71 m/s** on the
channel measured the same day. So the sprint is 40% slower than the channel,
which is the direction and roughly the size the `TwirlingCritic` argument
predicts. The absolute band was set from a stale 0.49 m/s figure that came from
the open-water benchmark, not from the channel — worth noting, because it means
the prediction was right partly by luck.

### Prediction 3 — "the driven radius exceeds 4.0 m" — WRONG, and backwards

Predicted a closest approach of 4.1–5.0 m, reasoning that `CostCritic` would
push the boat away from the mark's inflation. Measured: mean radius **3.90 m**,
minimum **2.46 m**. The boat cuts *inside* the commanded ring, not outside.

The reasoning was wrong at the first step. The mark's inflation reaches
0.184 + 1.30 = 1.48 m; the ring is at 4.0 m. There is no outward push to
oppose, because the boat is never in the inflated region at all. What actually
happens is corner-cutting between waypoints 3.06 m apart — and it cuts harder
than the inscribed polygon (3.70 m at the chord midpoint), so this is not
geometry alone.

`|wz|` while circling averages **0.247 rad/s** against the 0.106 rad/s a clean
4 m circle at 0.42 m/s would need — 2.3 times more yaw than the manoeuvre
requires — with 0.141 m/s of sway alongside it. So the boat is not gliding
round the ring; it is repeatedly turning toward the next waypoint and crabbing.
That is a real finding about how this configuration takes a curve, and the
open-water benchmark could not have produced it: its hardest turn was 63° from
a standstill.

### Prediction 4 — "the binding clearance is the gate, not the obstacle" — RIGHT

Min clearance 0.118 m at (9.9, −6.1), which is inside the sprint gate. The
black obstacle inside the loop was never the constraint.

That 0.118 m deserves attention on its own. The sprint gate is 2.4 m wide
against the channel's 2.8 m, so the most the hull can clear it by is 0.47 m
rather than 0.65 m — and the boat used three quarters of that. As a fraction it
is 0.25, against 0.49 on the channel. **The 6 ft end of the documented 6–10 ft
gate range is not comfortably within this configuration's reach**, and that is
the single most useful thing this task has surfaced for the real vehicle.

## The fourth way this stack looks ready without being ready

The first full regression run after the marina geometry landed reported the
sprint as REGRESSED. It had not. The goal aborted 30 ms after being sent, with

    Timed out while waiting for action server to acknowledge goal request
    for compute_path_through_poses

13.6 s *after* the lifecycle manager reported every node active. The boat never
moved: 0.0 m driven, 0.1 s elapsed, minimum clearance 5.154 m — which is the
distance from the spawn point to the nearest buoy.

Three things made this diagnosable rather than a day lost. The scorecard
carried elapsed time, so "0.1 s" was on the screen. The clearance was reported
as a fraction, and 1106% of the achievable maximum is obviously not a
measurement of anything. And the identical geometry had passed twice in the
previous suite run, with the new marina 45 m from the sprint course.

This is the *fourth* distinct readiness failure in this project — after a fixed
sleep, after the action merely existing, and after lifecycle `active`. Each
previous fix was correct and each was insufficient, because "the node is
active" and "the servers that node calls are answering" are different claims.
The gate now waits for `compute_path_through_poses`, which is the server the
behaviour tree actually calls.

Worth stating plainly: a flaky harness that reports a false regression is worse
than no harness, because the natural response to a red build you believe is
spurious is to stop believing red builds.

## The camera changes the run

Filming the sprint failed it, three times, in three different places.

| attempt | camera | ack window | outcome |
|---|---|---|---|
| 1 | 900x560 @ 10 Hz | 1000 ms | leg 2 aborted at 0.91 laps: `follow_path` ack timed out |
| 2 | 900x560 @ 10 Hz | 5000 ms | leg 1 aborted: `compute_path_through_poses` ack timed out |
| 3 | 640x400 @ 6 Hz | 5000 ms | leg 1 aborted, same server, same reason |

Every one of these was reported as a navigation failure. None of them was one.
Attempt 1 had already driven 0.91 of the lap cleanly, with no contact and
nothing wrong with the trajectory; the run died because `controller_server`
took longer than a second to *acknowledge* a goal — not to act on it.

The mechanism is dull and worth writing down anyway. This box has four cores.
Gazebo, software rendering, MPPI at 20 Hz over 2000 rollouts, Smac replanning
at 1 Hz and a PNG encoder all want them. Adding a camera adds a renderer, and
the thing that gives first is a server's ability to answer a handshake
promptly. `bt_navigator`'s `wait_for_service_timeout` defaults to 1000 ms and
is applied to goal *acknowledgement*, so a scheduling delay presents as an
aborted navigation.

Two things follow, and only one of them is a fix.

The fix: the ack window is now 20 s. It cannot mask a genuinely dead server —
the lifecycle manager and `action_server_result_timeout` still catch that — it
only stops a scheduling delay being reported as a navigation failure. On the
real vehicle, with a known CPU budget and a real clock, 1000 ms is the more
informative setting and should be restored.

The observation, which matters more: **the sprint is more fragile than the
channel for a structural reason, not a tuning one.** The channel is one leg and
therefore one handshake. The sprint is two legs, so it hands off mid-run, and
every leg boundary is a fresh goal request that can time out. Splitting a task
into legs was the right fix for the path-doubling-back problem and it bought a
new failure mode with it. Any task added later that needs three or four legs
inherits this, and the docking task will need at least two.

And a note on method: the filmed runs are not the scored runs, and the report
says so. An instrument that perturbs the system it measures is fine as long as
nobody quietly reads the numbers off it.

### It films now, on the fifth attempt

| # | camera | ack window | outcome |
|---|---|---|---|
| 4 | 640x400 @ 6 Hz | 20 s | leg 2 aborted; the boat went AROUND the gate at x=12.4 |
| 5 | 480x300 @ 4 Hz | 20 s | PASS, 1.00 laps, 0.141 m clearance, 156.3 s |

Attempt 4 is the one worth keeping. It did not time out on a handshake -- it
navigated worse. The boat passed outside the sprint gate entirely, at x = 12.4
against a gate spanning 8.8 to 11.2. Under that load the control loop no longer
held its rate, and MPPI tracking degraded until it took a route it does not
take unloaded. So the camera was not only delaying acknowledgements by then; it
was changing the trajectory.

That distinction matters for what the videos are worth. At 480x300 and 4 Hz
both tasks now film *and* pass, so the footage and the numbers come from the
same run, which is the only arrangement in which showing them side by side is
honest. The regression suite was re-run afterwards on the raised
acknowledgement window and both contracts hold: channel 10/10 at 54.4% of
available clearance, sprint 1.00 laps at 29.4%.

The generalisable point for the Unity port: a headless run and an observed run
are different experiments on a machine this size, and the difference is large
enough to change a pass into a fail. Budget for the observer.

---

# Thruster relayout: X configuration to pinwheel

A separate stretch of work, on its own branch. The vehicle owner said the four
motors should not be in an X. They were right about the geometry. The
consequences were not what the handoff predicted, and not what I predicted
either.

## 0. The box had no simulator on it

Worth recording before any result. The session started in a container with the
repository but no ROS, no Gazebo, no colcon and no `install/` --
`tools/env.sh` fails on `/opt/ros/jazzy/setup.bash`. Ubuntu 24.04 and 4 cores
matched, so Jazzy + Nav2 + ros-gz went in from apt and the workspace built
clean.

The check that mattered: **run the frozen configuration first and confirm it
reproduces the contract**, before changing anything. It did -- channel 10/10
gates, 0.349 m clearance, 132.0 s against a 0.330 +/- 0.063 reference. Without
that run every number below would have been uninterpretable, because a
freshly-provisioned box is itself an untested variable.

## 1. What the sketches say, and what they cannot say

Measured from the two committed sketches rather than eyeballed: threshold the
ink, connected-component label the strokes, take each motor's principal axis
and each arrowhead's direction from its barb bisector.

Both sketches agree on the axes. **TL and BR sit on one 45 degree axis, TR and
BL on the other**, which in the boat frame is `fl,rr` on 135 and `fr,rl` on 45
-- exactly swapped from `x_configuration`. The hull sketch's motor-centre
spacing is 1.35 longitudinal:lateral against the config's 1.375, so
`(0.55, 0.40)` m is consistent with what was drawn.

**The sketches do not fix the chirality, and they were never going to.** Every
motor is drawn with *two antiparallel arrows*, confirmed motor by motor:

| motor | head 1 | head 2 |
|---|---|---|
| TL | NE, +55 deg | SW, 228 deg |
| TR | NW, 130 deg | SE, -55 deg |
| BR | NE, +45 deg | SW, 239 deg |

That is the correct way to draw it. A T200 is reversible, so which way the
pinwheel spins is a wiring convention, not a mounting fact. The handoff's
warning that "a pinwheel has two chiralities and picking the wrong one inverts
yaw" is real but misaddressed: what the X layout gets wrong is not the
chirality, it is that its four yaw arms have **mixed signs**. Adopted
CCW-positive (all four forward gives +Mz, REP-103).

Bow-at-top in the hull sketch turns out not to matter: the layout has 180
degree rotational symmetry, so bow-at-bottom remaps the thruster names and the
frame conversion together and lands on the same answer.

## 2. Three corrections to the handoff's arithmetic

Measured against the real `allocation.py`, not restated from the handoff.

| | Fx (N) | Fy (N) | Mz (N.m) | yaw arm |
|---|---|---|---|---|
| X, handoff | 98.99 | 84.85 | 12.73 | 0.106 |
| X, **measured** | 98.99 | **70.71** | **10.61** | 0.1061 |
| pinwheel, handoff | 84.85 | 84.85 | 94.05 | 0.672 |
| pinwheel, **measured** | **70.71** | 70.71 | 94.05 | 0.6718 |

**(a) The handoff's numbers are the `max_reverse = 30 N` row.** Not a different
definition of "peak" -- one different parameter. Sweeping it reproduces the
handoff exactly at 30 N and gives a *zero* surge loss at 35 N:

```
max_reverse   pinwheel fx    X fy    X mz    surge loss
   25.0          70.71      70.71   10.61      28.6%     <- config today
   30.0          84.85      84.85   12.73      14.3%     <- the handoff's table
   35.0          98.99      98.99   14.85       0.0%
```

Two consequences, and they pull against each other. **You cannot have 8.9x and
zero surge loss at once**: the yaw gain is itself a function of `max_reverse`
(8.87x at 25 N, 7.39x at 30 N, **6.33x at 35 N**), because a symmetric reverse
limit helps the X layout's yaw as much as the pinwheel's surge. Both headline
numbers are also allocator-policy figures rather than pure geometry -- a
saturation-aware allocator reaches 84.85 N of pinwheel surge under 35/25, so
**half the 28.6% is recoverable in software**.

With that said: **the surge loss is an artefact of the forward/reverse
asymmetry**, because a pinwheel needs `(-,+,-,+)` to surge and
is therefore always reverse-limited, while the X layout surges on `(+,+,+,+)`.
Open question 2 -- is 35 N / 25 N right for the real vehicle -- is not a side
issue, it *is* the magnitude of the only real cost.

**(b) The surge loss is 28.6% of force / 18.5% of speed, not 14%.** The
codebase never agreed with the handoff either:
`test_max_wrench_matches_hand_computation` has asserted `4*25*cos45` and
`arm*4*25` all along.

**(c) The X layout does not merely "nearly cancel".** Its arms are
`+,-,+,-`, so all four at full forward produce **exactly zero** yaw. Its
10.61 N.m is only reachable by reversing half the motors.

Yaw gain is therefore **8.9x**, not 7.4x.

## 3. Section 4.1: the damping is not the thing that is wrong

The handoff says 2.69 rad/s is not credible, and that either the damping is too
low or the thrust is optimistic. I checked both, and the answer is the second,
not the first -- but the conclusion is still "do not raise `wz_max`".

**Damping.** Strip theory over the real URDF hull (length 1.40 m, hulls at
y = +-0.375 m), distributing the configured sway and surge damping along it:

```
lin_yaw  = lin_sway *L^2/12 + lin_surge *y^2   = 9.80 + 2.81 = 12.61
quad_yaw = quad_sway*L^3/32 + quad_surge*|y|^3 =  7.72 + 1.32 =  9.04
```
against the configured 8.0 / 10.0. So `lin_yaw` *is* low, by 58% -- section 4.1
is partly right. But correcting it moves the steady state only 2.69 -> 2.60
rad/s, because the quadratic term dominates at that rate. **Fixing the damping
does not rescue the number**, so it cannot be the explanation.

Note also this is a consistency check, not a validation: the yaw damping is
derived *from* `lin_sway`/`quad_sway`, which are themselves hand-set. If the
sway damping is low, the derived yaw damping is low by the same factor.

**Thrust.** This is the live branch. 35 N is a T200's **bollard** figure. At
2.6 rad/s the thrusters, at 0.672 m radius, see ~1.7 m/s of inflow and the
model has no advance-speed term at all, so putting bollard thrust on the
right-hand side of a steady-state solve is exactly the error section 4.1
suspected. The X layout never exposed this -- its peak 0.705 rad/s meant only
0.48 m/s of inflow. The pinwheel is the first thing that reaches the regime
where the thruster model stops being valid.

The sanity check that settles it: at 2.60 rad/s the hull ends (half-length
0.70 m) sweep at **1.82 m/s**, 37% faster than the boat's own full-throttle
straight-line speed of **1.33 m/s**. A boat that cannot be driven through water
at 1.4 m/s but swings its own ends through it at 1.8 m/s is not credible, and
strip theory being internally self-consistent does not rescue that.

**Decision: damping left UNCHANGED, `wz_max` left UNCHANGED at 0.55.** The
decision rests on the two arguments that do hold, not on the causal claim
above: changing the damping would be a second silent plant change confounding
the whole A/B, and the cap was never binding in the first place (peak demand
0.24 against 0.55). The honest correction is not knowable without either a
vehicle spin test (open question 3) or a thruster advance-speed curve. 2.69 rad/s is
recorded in the config as an **overestimate of a limit that was never binding
anyway** -- measured peak yaw on the channel is 0.24 rad/s, against a 0.55
cap. Raising `wz_max` to chase the envelope would have been sizing a limit
against a number the model cannot support.

## 4. The experiment

`nav2_mppi.yaml`'s limits are all still legal under the pinwheel envelope
(vx 1.30 < 1.329, vy 0.45 < 0.614, wz 0.55 < 2.693), so **"pinwheel, nothing
else changed" is a valid arm**. That is what makes the layout attributable.

| arm | layout | vx_max | prune | channel | sprint |
|---|---|---|---|---|---|
| A | X | 1.30 | 4.0 | 0.349 (53.8%) PASS, 132.0 s | 0.119 (25.5%) PASS, 141.7 s |
| B | pinwheel | 1.30 | 4.0 | **0.105 (16.1%) FAIL**, 110.6 s | 0.142 (30.4%) PASS, 154.3 s |
| C | pinwheel | 1.05 | 3.0 | 0.327 (50.3%) PASS, 142.2 s | **FAIL** -- leg 2 timed out, 0.50 laps |
| D | pinwheel | 1.05 | 4.0 | 0.128 / 0.376 / 0.321 / 0.142 | 0.115 (24.7%) PASS, 118.4 s |

### 4a. Four of five registered predictions for arm B were wrong

Registered before the run, and scored honestly:

| # | prediction | outcome |
|---|---|---|
| B1 | both `require` hold | **FALSIFIED** -- channel failed |
| B2 | channel elapsed rises 2-12 s | **FALSIFIED, backwards** -- fell 21.4 s |
| B3 | channel saturation rises above 29.1% | **FALSIFIED** -- fell to 15.7% |
| B4 | sprint unchanged beyond noise | **FALSIFIED** -- clearance +0.049 fraction |

The mechanism I argued from was wrong, and it is worth writing down why,
because the handoff's mechanism is wrong in the same place.

I reasoned from **thruster saturation**: the sprint never saturates (peak
32.8 N of 35, 0.3% of frames) while the channel saturates 29.1% of frames, so
I predicted more yaw authority could not help the sprint and the surge loss
would slow the channel. Both halves failed.

**The X layout reaches saturation far sooner for any mixed surge-plus-yaw
command.** Producing yaw there requires reversing two thrusters, and that
subtracts from surge directly. (The earlier phrasing here, "compete well below
saturation", was wrong: with a pseudo-inverse allocator anything inside the
limits is achieved exactly. The competition *is* the saturation boundary, and
the table below is that boundary plotted.)
The pinwheel removes the coupling. Measured off the allocator -- the most surge
still available while simultaneously holding a given yaw moment:

| Mz (N.m) | X layout Fx | pinwheel Fx |
|---|---|---|
| 0 | **98.99** | 70.71 |
| 2 | 85.66 | 72.82 |
| 4 | 72.33 | 74.92 |
| 6 | 58.99 | 77.03 |
| 8 | 45.66 | 79.13 |
| 10 | 32.33 | **81.24** |

In the X layout every N.m of yaw costs **6.7 N of surge**. In the pinwheel
surge *rises* with yaw, because a yaw command puts all four thrusters forward
and so relieves the reverse limit that binds the surge solution. **The two
curves cross at Mz ~ 3.6 N.m.**

That is the whole result in one table. The pinwheel is worse at surge only when
going perfectly straight; when turning past the 3.6 N.m crossover it has more
surge available, not less. That regime covers 43% of arm A's frames and 46% of
arm B's -- a substantial minority, not "most of the time". Hence a faster boat,
and less saturation rather than more, because it is no longer fighting itself.
Saturation was not the wrong instrument -- it moved 29.1% -> 15.7% exactly as
the coupling predicts. The wrong thing was the prediction I drew from it.

The handoff's "yaw starvation" is the same error in reverse. It diagnosed a
shortage of yaw *authority* from the sprint's behaviour. The trace says
authority was never the constraint on that task -- the coupling was.

### 4b. What actually costs the clearance: speed, not yaw

Arm A vs arm B on the channel, from the traces:

| | saturated frames | vx p50 | vx p90 | median abs(wz) | max abs(wz) | min clearance |
|---|---|---|---|---|---|---|
| A (X) | 29.1% | 0.79 | 1.15 | 0.057 | 0.239 | 0.351 |
| B (pinwheel) | 15.7% | 0.90 | 1.17 | 0.065 | 0.242 | **0.106** |

("saturated" = any thruster within 3% of a limit. At a 1% definition the same
frames read 24.5% and 11.0%; the ratio is what matters, but state the
threshold, because the number is meaningless without it.)

**Both of the vx figures in that table are contaminated, and the conclusion
drawn from them is retracted below in §5a.**

**The yaw distribution barely moves.** The boat is not over-rotating. It
sustains 14% more speed at the median, and that is what costs the clearance.
This is why `az_max`, `wz_max`, `wz_std` and `TwirlingCritic` were all left
alone: nothing in the data justifies touching them, and `az_max` never gets
within a quarter of its current limit.

So the retune is one parameter: **`vx_max` 1.30 -> 1.05**, which is 80% of the
new 1.329 envelope -- the same ratio the file has always used, and independently
motivated because 1.30 would have been 98% of achievable.

### 4c. A mirror edit that was not a mirror

Arm C also moved `prune_distance` 4.0 -> 3.0 to track `vx_max`, since the file
documents it as the horizon's reach. **That cost the sprint its second leg**:
the boat crawled at 0.10 m/s and timed out at 0.50 laps.

`prune_distance` is not only a look-ahead. `PathAlignCritic.offset_from_furthest`
is a minimum-index *gate* counted in path poses, and at a 0.20 m planner
resolution 4.0 m is the ~21 poses it was calibrated against -- at the largest
critic weight in the file (14.0). Shortening the path moved the gate.
Reverted to 4.0, and the sprint passed again at 118.4 s. Recorded as trap 9.

## 5. Result: the sprint improves, the channel does not hold

**Sprint: much faster, clearance unchanged, and passing.** Being precise about
which half is real, because the first draft of this entry overclaimed it:

| | clearance fraction | elapsed (s) |
|---|---|---|
| X layout (3 historical + arm A) | 0.253 0.270 0.279 0.255 -> mean **0.264**, sd 0.012 | 138.6 136.5 131.4 141.7 -> mean **137.1** |
| pinwheel, vx_max 1.30 (arm B) | 0.304 | 154.3 |
| pinwheel, vx_max 1.05 (**shipped**, arm D) | **0.247** | **118.4** |

**The time gain is NOT solid either, and the second draft of this entry
overclaimed it too.** 118.4 s is n=1, and it appears only in the arm that also
cut `vx_max` -- a parameter that is essentially inactive on the sprint (sprint
vx never exceeds 1.006 m/s). The one *clean* layout-only arm, B, ran the sprint
in **154.3 s: the slowest sprint in the whole record**, 12.6 s worse than arm A.
A 36 s swing from an inactive parameter means sprint elapsed is dominated by
path selection (67.8 / 69.5 / 65.4 m), not by the layout. Applying the same
n>=6 standard demanded of the channel: sprint elapsed is unresolved. **The clearance improvement is not real.** Arm B's 0.304 looked like a gain,
but the shipped configuration returns 0.247 -- *below* the X mean of 0.264,
about 1.4 sd on an sd of 0.012, at n=1. The honest reading is clearance
unchanged-to-slightly-worse and comfortably above the 0.20 floor either way.

So the handoff was right that the sprint benefits, and wrong about the currency:
the gain is speed, not clearance, and not by the yaw-starvation mechanism it
argued.

**Channel: this is the part that does not work.** Four runs of the shipped
config (D) against five of the X layout:

| | n | samples (m) | mean | sd | contract |
|---|---|---|---|---|---|
| X layout | 5 | 0.349 0.380 0.320 0.245 0.376 | 0.334 | 0.055 | 5 pass |
| pinwheel + vx_max 1.05 | 4 | **0.128** 0.376 0.321 **0.142** | 0.242 | 0.125 | **1 fail, 1 marginal** |

Mean clearance falls 0.092 m and the spread more than doubles (0.055 -> 0.125),
but **neither difference is resolvable at this n**: an exact permutation test on
the means gives p = 0.21, Mann-Whitney U = 5.5 (not significant at n = 4,5),
and the variance ratio F = 5.15 gives p ~ 0.15. The claim below rests on the
`require` failure, which is an observation, not on these.

Disclosure: four of the five X samples are the historical contract samples from
commit `67b39e6`, measured on a different box; only 0.349 was measured this
session. One run of four lands below the 0.130 m
floor outright, and a second passes at 0.142 m (21.8% against a 20.0% floor) --
a margin of 0.018 in fraction, 0.012 m in metres. Elapsed time goes the other way from what
the handoff feared: 132.5 -> 136.9 s, +3.3%.

**The channel contract does not reliably hold, and I am not proposing to loosen
it.** The floor of 0.20 was set at mean - 3sd from four runs and it is doing
exactly the job it was written for: binding on a real degradation rather than
on noise. One outright failure in four runs is a direct observation, not an inference. Moving the floor to
accommodate it would delete the only instrument that noticed.

**Where it fails is one specific place, and the first version of this entry got
that backwards.** I wrote that the minima occur at different places and called
it a less repeatable boat. The opposite is true. Closest approach to the
free-standing buoy at (6.0, 31.5) on the return leg:

| run | approach to that buoy | global min clearance |
|---|---|---|
| A (X) | 1.64 m | 0.351 |
| B (pinwheel) | **1.33 m** | **0.106** at (7.3, 31.1) |
| D (pinwheel) | **1.36 m** | **0.129** at (7.3, 31.0) |
| D4 (pinwheel) | -- | **0.142** at (7.3, 31.0) |
| D2 (pinwheel) | 1.67 m | 0.376 elsewhere |

Every bad run clips **the same buoy on the same leg**, at ~0.6 m/s with
abs(wz) below 0.07 -- so neither speed nor rotation. The good runs clear it by
1.63-1.67 m, exactly like the X layout. The global minima only "moved" because
on the good runs the hotspot did not bite. It is a bimodal line choice at one
obstacle.

That also kills the next-step recommendation I made: `TwirlingCritic` is a
global yaw-restraint term aimed at over-rotation everywhere, and the data show
no over-rotation anywhere.

**And on the thing the channel task is actually about, the pinwheel is fine.**
Per-gate offsets from centre, paired by gate, arm A against the mean of the
four pinwheel runs: pinwheel better at 6 gates, worse at 3, tied at 1; mean
abs(offset) 0.181 -> 0.173 m. The red contract is driven entirely by one
free-standing obstacle pass, not by degraded gate threading. Worth noting too
that `clearance_ceiling_m` (0.65) is calibrated on the tightest *gate*, so
normalising a lone-buoy event by it is applying the fraction outside the regime
it was defined in. The contract is still legitimately red; the caveat belongs
with it.

## 6. Where this leaves the change

Shipped: the layout, the schema, the URDF, the validation and the tests. They
are correct and they are covered. `x_configuration` is retained and still
tested, so the A/B remains available.

Not shipped as settled: the MPPI retune. `vx_max 1.05` is right on the envelope
argument and it recovers most of what arm B lost, but it does not restore the
channel to a reliably-passing state.

The next candidate is the one the handoff already named and I deliberately did
not spend runs on: **`TwirlingCritic.cost_weight`, bracketed at 10.0 when yaw
was expensive.** With yaw ~9x cheaper the same weight buys ~9x less restraint,
and it is the only term opposing yaw. That is a coherent mechanism for "less
repeatable near obstacles" in a way a speed cap is not. It needs n>=6 per arm
to resolve a half-sd shift against an sd of 0.125, which is ~50 minutes of runs per arm and is
the honest cost of settling it.

Ground rule 5 applies to the *stopping* decision, not to shipping a red
contract: the remaining candidate has a real mechanism, so the work is not
finished -- it is handed over mid-experiment with the instrument in place.

## 5a. Retracted: sustained speed is not what cost the clearance

The claim in §4b -- "it sustains 14% more speed at the median, and that is what
costs the clearance" -- is **withdrawn**. It was the basis for the whole
`vx_max` retune, and it is an artefact of how the metric was taken.

Every channel run ends by creeping backwards ~1.4 m onto the final waypoint at
(1.0, 29.0) while it satisfies goal yaw. Splitting each trace at first entry
within 1.5 m of that goal separates transit from settling:

| arm | total (s) | transit (s) | settle (s) | transit vx p50 | transit vx p90 |
|---|---|---|---|---|---|
| A (X, vx 1.30) | 131.9 | 101.2 | 30.7 | 0.884 | 1.177 |
| B (pinwheel, vx 1.30) | 110.5 | **99.8** | **10.7** | 0.935 | 1.176 |
| D (pinwheel, vx 1.05) | 138.3 | 107.2 | 31.2 | 0.924 | 1.050 |
| D2 | 135.3 | 103.3 | 32.0 | 0.932 | 1.049 |
| D3 | 136.8 | 103.6 | 33.1 | 0.945 | 1.049 |

**20.0 s of the 21.4 s "speedup" is the terminal settle**, not course speed;
transit differs by 1.4 s, i.e. 1.4%. And the "vx p50 0.79 -> 0.90" that the
diagnosis rested on is the same 20 s of near-stationary frames pulling the
median down in arm A. Transit-only it is 0.884 -> 0.935, +5.8%.

The refutation is in the data I already had: **D2 and D3 carry a *higher*
transit median than D (0.932, 0.945 vs 0.924) and passed at 0.376 and 0.321,
while D failed at 0.128.** Within the pinwheel runs, more median speed goes
with *better* clearance. And `vx_max` 1.30 -> 1.05 cut transit p90 (1.177 ->
1.05) while leaving the median untouched -- which is exactly why it did not fix
the clearance.

So `vx_max: 1.05` survived only on the envelope argument (1.30 would be 98% of
1.329 achievable). The causal story attached to it was wrong. This is the same
error as JOURNAL entry 4: sizing an intervention against a distribution without
first checking that the distribution says what it appears to.

---

# The layout the sim actually ships: corrected X mounting

The pinwheel above was built to the owner's sketch. Freed from that, the
question is what geometry is actually best for this vehicle, and the answer
falls out of one line of algebra that neither the handoff nor I had written
down.

## 6. Position is free yaw

For a 45 degree vectored quad at `(+-x, +-y)`:

```
Fx = cos(45) * sum(f)          <- no x, no y
Fy = sin(45) * (+f_fl -f_fr -f_rl +f_rr)   <- no x, no y
Mz arm = 0.7071 * (x - y)      <- position enters HERE and nowhere else
```

**Surge and sway do not depend on the mounting position at all.** Only the yaw
arm does, and only through the *difference* `x - y`. The old mounting was
(0.55, 0.40): `x - y` = 0.15, giving an arm of 0.106 m against a 0.680 m
physical radius. That is the entire defect. It is not the 45 degree cant, and
it is not the X sign pattern -- it is that somebody chose x and y close
together, and every other number on the vehicle still looked healthy.

So `(x - y)` is free yaw, and the fix is a mounting change:

| | arm | Fx | Fy | Mz | vx | vy | wz | Fx at Mz=5 | Fx at Mz=10 |
|---|---|---|---|---|---|---|---|---|---|
| old X (0.55, 0.40) | 0.106 | 98.99 | 70.71 | 10.61 | 1.630 | 0.614 | 0.705 | 65.66 | 32.33 |
| sketch pinwheel | 0.672 | 70.71 | 70.71 | 94.05 | 1.329 | 0.614 | 2.693 | 75.97 | 81.24 |
| **shipped (0.60, 0.25)** | **0.247** | **98.99** | **70.71** | **24.75** | **1.630** | **0.614** | **1.223** | **84.71** | **70.42** |

(0.60, 0.25) puts the motors at the inboard face of each pontoon (centres
+-0.375, radius 0.125) and 0.10 m short of the hull ends.

Against the old mounting: **2.3x the yaw moment, and surge and sway are
bit-identical.** Against the pinwheel: 40% more surge, and *more* surge
available while turning at a realistic 5 N.m (84.71 vs 75.97) -- the pinwheel
only wins the coupling comparison once past ~8 N.m, which the course does not
ask for. Peak yaw demand measured on the course is 0.24 rad/s against a 0.55
cap that never binds, so the pinwheel's 2.69 rad/s buys nothing and costs 29%
of the surge to have.

The best part is what it does *not* require. **Every limit in `nav2_mppi.yaml`
is still legal** (vx 1.30 < 1.630, vy 0.45 < 0.614, wz 0.55 < 1.223), so the
file went back to its frozen values -- `vx_max: 1.30`, `prune_distance: 4.0`.
The controller that was tuned over the previous stretch is untouched; only the
plant improved.

## 7. Result: both contracts hold

| | channel | sprint |
|---|---|---|
| A -- old X | 0.349 (53.8%) PASS, 132.0 s | 0.119 (25.5%) PASS, 141.7 s |
| B -- pinwheel | **0.105 FAIL**, 110.6 s | 0.142 PASS, 154.3 s |
| D -- pinwheel + vx 1.05 | 0.128 **FAIL** / 0.376 / 0.321 / 0.142 | 0.115 PASS, 118.4 s |
| **E -- shipped X (0.60, 0.25)** | **0.311 / 0.343 / 0.344, all PASS**, 127.1 / 125.0 / 129.4 s | **0.115 (24.7%) PASS, 150.0 s** |

Channel clearance across the three arms, same instrument:

| | n | samples (m) | mean | sd | contract |
|---|---|---|---|---|---|
| old X (0.55, 0.40) | 5 | 0.349 0.380 0.320 0.245 0.376 | 0.334 | 0.055 | 5 pass |
| pinwheel + vx_max 1.05 | 4 | **0.128** 0.376 0.321 0.142 | 0.242 | 0.125 | 1 fail, 1 marginal |
| **shipped X (0.60, 0.25)** | 3 | 0.311 0.343 0.344 | **0.333** | **0.019** | **3 pass** |

The mean is the old layout's to three decimal places, and the spread is a third
of it -- though at n=3 against n=5 the variance difference is not resolvable and
is not claimed. Elapsed 127.2 s against 132.5 s, ~4% faster, with `vx_max` back
at its frozen 1.30.

Registered predictions for arm E, scored:

| # | prediction | outcome |
|---|---|---|
| E1 | both `require` hold | **correct** |
| E2 | channel clearance inside the X band 0.245-0.380 | **correct** -- 0.311 |
| E3 | channel elapsed within 127-137 s | **correct** -- 127.1, at the edge |
| E4 | sprint faster than arm A's 141.7 s | **FALSIFIED** -- 150.0 s, slower |
| E5 | channel saturation below arm A's | **correct** -- 24.5% -> 7.7% at a 1% definition |

E4 is the interesting failure. Sprint elapsed swung 118.4 / 150.0 / 154.3 s
across arms whose sprint-relevant parameters barely differ, which is the same
signal as §5's Finding: on this task elapsed is set by which path the planner
picks, not by thruster authority. Sprint time is not a usable instrument at
n=1, in either direction, and no claim here rests on it.

On the failure mode that actually killed the pinwheel -- the free-standing buoy
at (6.0, 31.5) -- arm E clears it by **1.65 m**, against 1.64 m for the old X
layout and 1.33-1.36 m on the three pinwheel runs that failed. It behaves like
the baseline exactly where the pinwheel did not.

### Seeing it

`media/run_report_tangential.html` is a self-contained interactive replay of
both scored runs on the shipped layout, with the old X runs available as a
ghost overlay -- plan view, the boat's own footprint, the
planned route, and the four motors' live per-thruster force, scrubbable. It has
no external requests, so it opens straight from disk. The trace it is built
from is `tuning/report_rundata_armI.json`.

## 8. What is still open

* **n is small.** One full regress plus repeats. The channel's own spread means
  n>=6 is what would settle a half-sd shift; arm E is inside the X band on the
  runs taken, which is the honest claim and not more.
* **Sprint elapsed is unresolved** and should not be quoted as a gain or a loss
  until someone spends the runs.
* **The mounting is a real-vehicle change**, not just a sim edit. (0.60, 0.25)
  assumes the motors can hang off the inboard face of each pontoon near the
  ends. If the real boat cannot take that, the same algebra applies to whatever
  `(x - y)` it *can* take: at the old cant every centimetre of `x - y` is
  0.7071 cm of arm, free.
* **Open question 2 is now the important one**, and for a new reason: a
  symmetric reverse limit would lift the old layout's yaw as well as the
  pinwheel's surge. 35/25 shapes every trade in this entry.
* `TwirlingCritic` is **not** the next thing to try -- see §5. The pinwheel's
  failure was one obstacle on one leg, and nothing in the traces shows
  over-rotation anywhere.

---

# Correction: ship the pinwheel, and fix what actually broke it

The previous section shipped a corrected X mounting on the grounds that it
measured better. That was the wrong call on the brief: the owner asked for a
layout where **every thruster contributes yaw in the same direction**, and an X
with better mounting is still an X. Reverted. The pinwheel ships.

## 9. The requirement forces the surge cost — that part was not the sketch's fault

Worth settling before re-shipping, because it decides whether the 28.6% surge
loss was intrinsic or an artefact of the sketch's particular 45 degrees.

The sign pattern of the yaw arms decides which mode is *free*:

* **mixed signs** (`+,-,+,-`, the X) -> all four forward is a pure SURGE
  command, so surge gets the 35 N forward limit from every motor;
* **matching signs** (the pinwheel) -> all four forward is a pure YAW command,
  so yaw gets it instead, and surge must reverse two motors and is bound by 25 N.

You cannot have both. Producing *pure* surge under matching-sign arms means
cancelling a yaw moment you are generating by construction. I checked whether a
less symmetric same-sign layout escapes it -- splaying the front and rear pairs
to different cants (60/20, 70/30, 55/15, 75/40 with the arms still all one
sign). Every one is far worse than the tangential 45:

| layout | arms all one sign | decoupled Fx | Fy | Mz |
|---|---|---|---|---|
| X, (0.60, 0.25) | no | 98.99 | 70.71 | 24.75 |
| **pinwheel 45** | **yes** | **70.71** | **70.71** | **94.05** |
| same-sign 60/20 | yes | 2.77 | 50.05 | 1.19 |
| same-sign 70/30 | yes | 9.38 | 60.29 | 9.42 |
| same-sign 75/40 | yes | 11.88 | 69.68 | 14.33 |

So the tangential 45 degree pinwheel is the **best member of the family the
owner asked for**, and its surge cost is the price of the requirement, not of
the sketch. The only thing that would remove it is a symmetric reverse limit
(open question 2): at 35 N reverse the pinwheel's surge loss is zero.

Mounting stays at the vehicle's existing **(0.55, 0.40)**. For a pinwheel the
arm is `0.7071*(x + y)` = 0.672 m, which is 99% of the physical radius, and the
change the owner asked for is where the motors *point*. No motor moves.

## 10. What actually broke the pinwheel: a plan nobody had checked

Arm D failed the channel 1 run in 4, always clipping the same free-standing
buoy at (6.0, 31.5) on the return leg. I had recorded that as unexplained. It
is explainable, and the explanation indicts the *planner*, not the layout:

| run | planned distance to that buoy | actual | speed at closest |
|---|---|---|---|
| A (X) | 1.24 | 1.64 | 0.76 |
| B (pinwheel) | 1.28 | **1.33** | 0.47 |
| D (pinwheel) | 1.24 | **1.36** | 0.52 |
| D2 (pinwheel) | 1.26 | 1.67 | 0.94 |
| D3 (pinwheel) | 1.27 | 1.63 | 1.00 |
| E (X) | 1.24 | 1.65 | 0.92 |

**The global plan routes ~1.25 m from that buoy in every run of every layout.**
Runs that clear it do so by *overshooting the plan outward* while moving fast.
Runs that fail track the plan closely at low speed and inherit its margin.

So the margin there was never designed, it was tracking error. The X layout was
not safer; it was luckier. The pinwheel has more authority, tracks the plan
better, and so loses the luck. **Better tracking exposed a bad plan.**

The fix belongs in the term that holds the boat off inflated cells regardless of
what the plan asks: `CostCritic.cost_weight` 3.81 -> 8.00.

## 11. Result: both contracts hold, and the failure mode is gone

Registered before the run, scored after:

| # | prediction | outcome |
|---|---|---|
| F1 | both `require` hold on every run | **correct** — 3 channel + 1 sprint, all pass |
| F2 | approach to (6.0, 31.5) above 1.55 m on all runs | **correct** — 1.66 / 1.67 / 1.75, was 1.33 |
| F3 | channel min clearance inside the X band 0.245–0.380 | **FALSIFIED** — 0.178 / 0.216 / 0.279 |
| F4 | per-gate mean offset improves on 0.173 m | **FALSIFIED** — 0.185 m, slightly worse |
| F5 | elapsed no more than 8 s over arm D's 136.9 | **correct** — 135.1 s |

| | n | channel min clearance | mean | sd | contract |
|---|---|---|---|---|---|
| X layout | 5 | 0.349 0.380 0.320 0.245 0.376 | 0.334 | 0.055 | 5 pass |
| pinwheel, CostCritic 3.81 | 4 | **0.128** 0.376 0.321 0.142 | 0.242 | 0.125 | 1 fail |
| **pinwheel, CostCritic 8.00** | 3 | 0.216 0.279 0.178 | 0.224 | **0.051** | **3 pass** |

Sprint: **0.180 m = 38.7% of achievable against a 26.8% reference** -- the best
sprint clearance in the record -- in 131.9 s.

**The variance is the real result.** sd falls 0.125 -> 0.051, back in line with
the X layout's 0.055, and the bimodal collapse to 0.128 is gone: the buoy that
caused it is now cleared by 1.66 m at worst, against the X layout's 1.64.

**What is still worse, and is not hidden:** typical channel clearance is 0.224
against the X layout's 0.334. The pinwheel passes closer. It passes closer
*repeatably* rather than occasionally-catastrophically, and every run clears the
0.130 m floor by at least 0.048 m, but it is a real reduction in margin and the
`drift_warn` fires on two of the three runs.

The binding constraint has also moved, and is now the same event every time:
all three minima occur at **(21.4, 3.4) at t ~ 30 s, at 0.20-0.36 m/s**, near
the green buoy at (21.5, 4.7) -- a slow, yawed manoeuvre at the speed gate
rather than a fast pass. That is the next thing to look at, and unlike the
(6.0, 31.5) event it is repeatable enough to measure properly.

Neither regression contract was edited.

---

# The layout that was actually being asked for: tangential

Two wrong turns before this one, both mine, both from mis-reading the same
sentence. The owner said the motors are set up so that **forward thrust on
every motor drives the boat forward**, and that yawing is therefore still a
mix of positive and negative -- just far easier than in an X. I read that first
as "rotationally consistent yaw" (the pinwheel, which makes all-forward a *yaw*
command and costs 28.6% of surge), and then over-corrected into keeping the X
sign pattern with better mounts. Neither is what was described.

## 12. It is the cant SIGN, and it costs nothing

Same mounts, same 45 degrees, same T200s, same "all four forward is surge".
Flip which way each motor is canted:

```
x_configuration   fl +45   fr -45   rl -45   rr +45     arms +.106 -.106 +.106 -.106
tangential        fl -45   fr +45   rl +45   rr -45     arms -.672 +.672 -.672 +.672
```

Both have `cos(a) > 0` on all four, so both surge on all four forward. Both
need a mix of forward and reverse to yaw. The only difference is **how much
moment that mix produces**, and it is the difference between pointing the
thrust line 9 degrees off the radius from the CoG and pointing it 81 degrees
off. A force aimed nearly *at* the pivot turns nothing; a force aimed
tangentially is all leverage.

| | thrust line vs radius | yaw arm | Fx | Fy | Mz |
|---|---|---|---|---|---|
| X | 9 deg | 0.106 m | 98.99 | 70.71 | 10.61 |
| **tangential** | **81 deg** | **0.672 m** | **98.99** | **70.71** | **67.18** |
| pinwheel | 81 deg | 0.672 m | 70.71 | 70.71 | 94.05 |

**6.3x the yaw moment. Surge and sway identical to the newton.** Not a trade --
nothing is given up. The pinwheel's extra 40% of moment on top of that is the
only part that costs surge, and it is the part nobody needed: measured peak
yaw demand on this course is 0.24 rad/s against a 0.55 cap that never binds.

`vx_max` goes back to its frozen 1.30, because the surge envelope never moved.

## 13. Retracted: the CostCritic raise

Entry 10 raised `CostCritic.cost_weight` 3.81 -> 8.00 to fix the pinwheel's
habit of clipping the buoy at (6.0, 31.5). **Reverted.** It was treating a
symptom the tangential layout does not have, and it was costing clearance
everywhere else:

| arm | layout | CostCritic | channel clearance | mean | sd |
|---|---|---|---|---|---|
| A | X | 3.81 | 0.349 0.380 0.320 0.245 0.376 | 0.334 | 0.055 |
| F | pinwheel | 8.00 | 0.216 0.279 0.178 | 0.224 | 0.051 |
| G | tangential | 8.00 | 0.280 0.194 0.236 | 0.237 | 0.043 |
| **H** | **tangential** | **3.81** | **0.325 0.317 0.292** | **0.311** | **0.017** |

Both CostCritic-8.00 arms sit ~0.09 m below the X baseline; putting it back at
3.81 recovers that in full. And the thing it was supposed to protect is fine
without it: closest approach to (6.0, 31.5) is **1.67 m** at CostCritic 3.81
with the tangential layout, against the 1.33 m that failed with the pinwheel.

That reframes entry 10's finding. The plan really does route ~1.25 m from that
buoy, and the margin really is tracking error -- but it is only *dangerous*
tracking error for the pinwheel, which had 8.9x yaw and tracked the plan hard
enough to sit on it. The tangential layout tracks it from further out. Fixing
the planner is still worth doing on its own merits; it was not needed here, and
I should have tested the layout at frozen parameters before adding a change.

## 14. Result

Registered before the runs, scored after:

| # | prediction | outcome |
|---|---|---|
| G1/H1 | both `require` hold every run | **correct** — 6 channel + 2 sprint, all pass |
| G2 | channel mean inside the X band 0.245–0.380 | at CostCritic 8.00 **falsified** (0.237); at 3.81 **correct** (0.311) |
| G3 | elapsed within 6 s of 132.5 | **correct** — 128.5 s, 4 s faster |
| G4/H3 | buoy approach above 1.50 m | **correct** — 1.65–1.67 m |
| G5 | saturation falls below the X's 24.5% | **correct, and by a lot** — 2.1% |

Shipped configuration, `layout: tangential`, every MPPI parameter frozen:

| | X layout | tangential | |
|---|---|---|---|
| channel min clearance | 0.334 (sd 0.055, n=5) | **0.311 (sd 0.017, n=3)** | same, 3x more repeatable |
| channel elapsed | 132.5 s | **128.5 s** | 4 s faster |
| sprint clearance | 0.119 (25.5%) | 0.126 (26.9%) | unchanged |
| sprint elapsed | 141.7 s | 139.3 s | unchanged |
| thruster saturation | 24.5% of frames | **2.1%** | 12x less |
| peak yaw moment | 10.61 N·m | **67.18 N·m** | 6.3x |

**The saturation number is the one that shows the layout doing work.** The X
spent a quarter of every run with at least one thruster pinned at a limit,
because turning cost it so much thrust. The tangential layout is at a limit 2%
of the time. That headroom is what buys the tighter spread and the 4 seconds.

Nothing measurably got worse. Channel clearance is 0.023 m lower on a 0.055
baseline sd, which is well inside noise and n=3 against n=5 resolves nothing;
the spread being 3x tighter is the more interesting number and is also not
resolvable at this n. Neither regression contract was edited.

## 15. A fresh trial, and a replay that would not draw

The report was reported blank -- interface present, replay empty, no map. It
was not the data: the canvas backing store was being set to **width 0** and
never recovered.

`fit()` read `wrap.clientWidth` once at script time. In a viewer that lays the
page out inside a hidden or not-yet-sized container -- which is the normal case
for an embed -- that reads 0, `canvas.width = 0` is committed, and nothing
repaints afterwards because only a window `resize` was listened for and the
window never resized. My own screenshot test passed because Playwright's
`setContent` gives layout a full second to settle before the script runs, so it
only ever exercised the healthy path.

Reproduced it directly (hidden-then-shown, and zero-width-then-widened: both
gave `canvas.width = 0, painted = 0`), then fixed it three ways: `fit()` now
refuses to commit a size below 40 px and reports failure, `renderAll()` gates
on a successful fit, and a `ResizeObserver` on the chart container repaints on
the first real layout. `visibilitychange`, `fonts.ready` and `load` also
retrigger. All three cases now paint identically to the healthy one.

The diagram was never affected -- it is SVG with a viewBox, so it scales without
measuring anything.

Fresh trial on the shipped configuration, both contracts holding:

| | result |
|---|---|
| channel | 10/10 gates, no contact, **0.352 m (54.2%)**, 126.4 s |
| sprint | 1.00 laps ccw, no contact, 0.123 m (26.3%), 149.5 s |

That is the best channel clearance of any tangential run and the fastest, and
it takes the layout to n=4: 0.325 / 0.317 / 0.292 / 0.352, mean **0.322**,
sd **0.025**, against the X layout's 0.334 mean and 0.055 sd. The gap is now
0.012 m, comfortably inside noise. Channel saturation 2.3% against 24.5%.

## 17. The report was blank because the viewer runs it without JavaScript

Section 16's canvas-sizing bug was real and worth fixing, but it was not why
the user saw nothing. Their screenshot showed the *static HTML placeholders* --
`0.00`, `0.000`, `0deg` -- and none of the JS-populated chips or chart. That is
not a sizing failure; a mis-sized canvas leaves the surrounding numbers filled
in. It is the signature of scripts never running at all.

Reproduced exactly with Playwright at `javaScriptEnabled: false`: CSS perfect,
type perfect, chart empty, chips missing, placeholder digits intact. Pixel for
pixel the screenshot. The attachment viewer does not execute scripts.

The user's own hypothesis -- that too much time passes between building the
file and opening it -- is not the cause. The file is a single self-contained
document with no network requests and no expiry; it renders the same the
minute it is written and a week later. It renders blank at both.

So the fix is not another JS repair. `tools/make_static_report.py` rebuilds the
whole report with **no script tag at all**:

* the plan view is inline SVG emitted at build time -- 5 m grid, dashed planner
  route, orange track, hull strobed every 8 s, filled hull and amber ring at the
  tightest pass, green start ring, scale bar;
* the thruster layout is inline SVG with the yaw-arm construction drawn, plus an
  arrow on each motor's forward axis so "all four forward is surge" is visible
  rather than asserted;
* every metric is literal text baked in from the run cards.

Two things caught in the no-JS screenshot pass and fixed before shipping: SVG
annotation type has to be sized in *user units* (the plan is 760 units wide and
renders at ~370 CSS px, so an 11 px label came out at 5 px), and the tightest-pass
label has to flip to the left of its ring when the ring sits in the right of the
frame or it clips off the edge.

Verified at 430 px and 900 px viewports, JavaScript disabled, zero non-`file:`
requests. The interactive replays are kept, but they are not the deliverable --
they only work in a browser.

## 18. Five formats of the same replay, because viewers differ

The static report proved the numbers can be delivered, but a static report is
not the trial. `tools/make_replay_formats.py` renders one recorded trace into
five independent formats so that at least one survives whatever the viewer is:

| file | needs | interactive |
|---|---|---|
| `_css.html` | CSS only | play/pause and 1x/2x/4x, as checkbox and radio inputs |
| `_smil.svg` | SVG SMIL | self-animating, vector |
| `.gif` | image decoder | plays, loops |
| `.mp4` | H.264 | native player scrubbing |
| `.png` | image decoder | none -- the whole run in one frame |

The CSS replay uses `offset-path` with `offset-rotate: auto` for the hull and a
`stroke-dashoffset` sweep for the track. Controls are `:checked` sibling
selectors, so they work with scripting off. Verified under Playwright at
`javaScriptEnabled: false`: clicking pause froze it (two screenshots 3 s apart
pixel-identical) and 4x resumed it at the new rate.

The SMIL build needed the hull rotation derived rather than guessed. The hull is
drawn bow-along-local-+X and SVG `rotate()` turns clockwise, so yaw = 0 has to
map to `rotate(-90)`, not `+90`. The first version sailed backwards.

The raster frames carry a HUD the vector ones cannot: clock, speed, yaw rate,
clearance, and a signed bar per thruster. In mid-channel surge all four bars sit
positive together, which is the layout claim visible directly in the replay.

The planner route is taken from the *longest* published plan, not the last one.
The last plan is a stub a metre long at the goal, so the earlier builds drew
almost no route.

## 19. Arm J — the boat was being charged 1.79x to turn

Three observations from the user: it crab-walks instead of yawing, it stops
mid-sprint, and it struggles with heading afterwards. All three are one
mechanism, and it was in the cost function, not the plant.

Measured first, on the shipped config: |wz| p99 is 0.232 against `wz_max` 0.55,
and **0.0%** of either run is spent above half the yaw cap, while |vy| sits
pinned at 0.450 of a 0.45 cap for a quarter of the run. So the obvious fix --
raise `wz_max` to match the new 6.3x moment arm -- was registered up front as a
predicted **no-op**, because the cap was provably never binding.

### The actual mechanism, from the 1.3.12 source

`PathAngleCritic` scores the **terminal** trajectory yaw
(`xt::view(data.trajectories.yaws, xt::all(), -1)`); `TwirlingCritic` scores the
**horizon mean** of `|wz|`. So over the T = 2.8 s horizon, yawing at rate `w` to
fix heading saves `2.8 * W_pa * w` and costs `W_tw * w`. Break-even is
`W_tw <= 2.8 * W_pa`. At the shipped 2.0 against 10.0 that is 5.6 against 10.0:
**correcting heading by rotating lost by 1.79x at every yaw rate, everywhere on
the course.** Sway, meanwhile, is charged by nothing anywhere in the stack --
`ConstraintCritic`'s Omni branch only fires above `hypot(vx,vy) = 1.376`, and
`PreferForwardCritic` scores `max(-vx, 0)`, reverse surge only.

And `max_angle_to_furthest` is an early-return gate on the *current* pose, so at
1.0 rad there was a **57.3 deg band in which yaw was purely penalised and sway
was free**. The two measured channel crab episodes sit at 31.3 and 41.1 deg of
sideslip -- inside it.

Change: `PathAngleCritic.cost_weight` 2.0 -> 6.0, `max_angle_to_furthest`
1.0 -> 0.35. **`TwirlingCritic` left at 10.0** -- the ratio flips without it, so
the parameter with the recorded 20.7 s regression was never spent.

### Result, both contracts holding, neither JSON edited

| | channel base -> J | sprint base -> J |
|---|---|---|
| crab frames | 27.2% -> **5.1%** | 29.0% -> **6.9%** |
| \|wz\| p90 | 0.181 -> **0.219** | 0.146 -> **0.215** |
| \|vy\| p90 | 0.338 -> **0.194** | 0.286 -> **0.184** |
| stall seconds | 20.1 -> **0.0** | (see J1) |
| elapsed | 126.4 -> **101.9 s** | 149.5 -> **106.0 s** |
| course distance | 96.8 -> 95.1 m | 69.1 -> 62.4 m |
| min clearance | 0.352 -> 0.307 m | 0.123 -> **0.164 m** |
| thruster saturation | 1.6% -> **0.0%** | 0.0% -> 0.0% |

Sprint primaries, each with a baseline reproduced across *both* thruster
layouts: waypoint arrival -> leg change **20.0 s -> 2.5 s** (X layout 19.7);
max excursion past the waypoint **1.44 m -> 0.67 m** (X 0.91); heading error at
the tight gate **37.5 deg -> 6.6** (X 38.6). Channel course distance is
unchanged at 95.1 m against a 96.2 reference, so the 24.5 s came from not
stalling, not from cutting the course.

### Two things to be honest about

`|wz|` p90 on the channel landed at **0.219 against a predicted > 0.22** -- a
miss by 0.001, inside the 0.004 within-config sd. The falsifier (<= 0.20) did
not fire, but the prediction did not strictly hit either.

Channel min clearance fell **0.352 -> 0.307 m**. That is inside the contract
reference (0.330 +/- 0.063) and inside the 0.245-0.380 historical spread, and it
is n=1 -- but it is the one number that moved the unfavourable way, and it is
recorded as such rather than absorbed into the summary. The sprint's clearance,
which is the *binding* contract with only 0.030 m of margin, improved to 0.164.

### The waypoint overshoot, which nobody had located

Both stalls are the same event: the boat enters the goal circle at 0.75 m/s,
**coasts 1.4 m past it** with commanded thrust decaying smoothly to zero, ends
up with the goal 140 deg astern, and then crabs back sideways for ~12 s. It
reproduced on the X layout too (0.91 m, 134 deg, 19.7 s), so it was never a
consequence of the layout change. Deceleration authority was ruled out with
algebra rather than a run: `M*gain*(nu_cmd - nu)` gives -72 N of the 99 N
available for a full stop from 0.75 m/s, and back-solving the 2-4 N actually
commanded shows MPPI's `vx` command tracking the coast to within 0.04 m/s.
It was not being asked to stop.

### Retractions from my own prior analysis in this session

* I claimed a "critic dead band" during stalls -- that at 5-7 plan poses
  "the two largest path terms stop firing entirely". **False.**
  `PathFollowCritic.offset_from_furthest` is not a gate at all (it clamps an
  index), PathFollow is disabled in both stalls by `threshold_to_consider: 2.5`
  which no offset change can re-enable, and `PathAlignCritic` fires through
  **100%** of the channel stall. The proposed arm would have been a no-op.
* I claimed plan age staying at 0.1-1.9 s ruled out a blocked action server.
  Invalid -- that shows the *planner* is alive, not that FollowPath is being
  serviced. The right evidence is that thrust varies smoothly and continuously
  through the window, which a blocked server cannot produce.

### Harness note

The first channel attempt aborted at 0.0 s having travelled 0.0 m:
`Timed out while waiting for action server to acknowledge goal request for
compute_path_through_poses`. That fires before the controller is ever invoked,
so it cannot be caused by a critic weight. Scored as **void**, re-run, not
counted as a regression. Worth watching -- it is the trap `wait_for_service_timeout`
was already raised to 20000 ms for, and it still fires occasionally.

## 20. Arm J at n=2 on both tasks, and the clearance scare was a bad readout

| sprint | I (base) | A (old X) | J | J2 |
|---|---|---|---|---|
| elapsed | 149.4 | 141.6 | 105.9 | **95.3** |
| min clearance | 0.123 | 0.119 | 0.164 | 0.127 |
| crab frames | 29.0% | 26.0% | 6.9% | **0.8%** |
| mid-task pause s | 40.9 | 33.7 | 8.5 | 9.1 |
| arrival -> leg change | 20.0 | 19.7 | 2.5 | **1.6** |
| max excursion past wp | 1.44 | 0.91 | 0.67 | 0.73 |
| gate heading error | 37.5 deg | 38.6 | 6.6 | 9.9 |

All three pre-registered falsifiers reproduce. Both channel runs and both sprint
runs pass, `contact: false` throughout, neither contract JSON edited.

### The channel clearance drop was an artefact of the statistic, not a regression

I recorded 0.352 -> 0.307 as "inside the historical spread". Right conclusion,
**invalid test, and I compared two different obstacles**: I_channel's minimum is
at gate@13 (t=15.3, 1.21 m/s) and J_channel's is at gate@21 (t=23.8, 0.89 m/s).
`min_clearance` is a single-frame extreme-value statistic over ~900 frames and
four obstacles, and the "0.245-0.380 historical spread" pools configurations
whose speed through gate@21 differs by 3x -- a mixture of two populations, not
a noise band.

Duration-free readout instead -- the mean of the four per-gate minima taken in
fixed spatial windows:

| | gate@13 | gate@21 | buoy@7,31 | goal@1,29 | **mean4** |
|---|---|---|---|---|---|
| I_channel | 0.355 | 0.362 | 0.580 | 0.381 | **0.419** |
| H / H2 / H3 (one config, 3 repeats) | | | | | **0.413 / 0.384 / 0.373** |
| J_channel | 0.313 | 0.308 | 0.613 | 0.399 | **0.408** |
| J2_channel | 0.355 | 0.322 | 0.598 | 0.325 | **0.400** |

J sits inside the H triplet's own spread and is indistinguishable from I. Two
gates down ~1σ, two up. **The clearance question is closed and no runs were
spent on it.** Settling the raw `min_clearance` delta would have needed n≈6-8
per arm -- 1.5-3.5 hours to chase a 1σ move in a metric both contracts pass.

Also recorded: `p05_clearance_m` is **duration-biased** here and must not be
promoted as the more sensitive endpoint. Excluding stalled frames alone moves
I_channel's p05 from 0.511 to 0.480 -- 58% of its apparent delta is just the
removal of 20 s of slow frames. Same trap the twirl-1.5 result fell into.

### Monotonicity separates "turning" from "dithering"

Every mid-task pause, as net yaw / total |dyaw| in degrees:

```
I_sprint : 38/38, 19/19, 18/84, 6/6      <- one episode reverses 4.7x over
A_sprint : 37/37, 19/19, 5/5, 46/56, 7/7
J_sprint : 36/36, 15/15, 24/24           <- all monotone, zero reversal
J2_sprint: 25/25, 24/24, 38/38           <- all monotone, zero reversal
```

The baseline's stalls contained genuine direction reversal. Arm J's remaining
slow episodes are pure monotone corner turns at 39% of yaw cap and ~25% of
thrust. **A raw seconds-below-a-speed-threshold metric cannot tell those apart,
and should be replaced by the reversal test.**

### Sideslip needs a speed floor, because it is scale-free

Fraction of moving frames above 25 deg of sideslip:

| trace | floor 0.3 m/s | floor 0.5 m/s |
|---|---|---|
| I_sprint | 24.6% | **21.0%** |
| J_sprint | 9.4% | **0.0%** |
| J2_sprint | 4.8% | **0.0%** |
| J_channel | 0.0% | 0.0% |
| J2_channel | 2.9% | 0.0% |

At 0.34 m/s, 40 deg of sideslip needs only 0.216 m/s of sway. The apparent
sprint "residual crabbing" at a 0.3 m/s floor is a low-speed artefact: at a
matched 0.5 m/s floor the sprint is **cleaner than the channel** (p90 9.5/10.8
against 11.4/14.2). The baseline still reads 21.0% at that floor, so the metric
discriminates; it is not being flattened.

### `vy_max` 0.45 -> 0.25 is refuted, not deferred

`|vy|` max across the whole of J_sprint is **0.257** and J_channel **0.275**; a
0.25 cap would clip 1.2-1.6% of frames. At baseline the cap bound hard (20.7% of
I_channel above 0.25) -- the cost function has already done the job. Worse, the
J_channel frames it would clip carry median speed 0.79 m/s: fast lateral gate
centring, which is exactly the "lateral correction" the user asked to keep. And
`vy_std` is 0.20, so sampling against a 0.25 cap would saturate a large fraction
of draws and degrade exploration, making it a two-parameter arm. Both planners'
version of this arm is withdrawn.

### `wz_max` null, registered for the third time

|wz| p99 is 0.250-0.256 against a 0.55 cap, with **0.0%** of frames in either
task above half the cap. Under the control law arm J installed (`w = e/2.8`,
tracked to within 6% for e < 0.7 rad), `wz_max` 0.55 is only demanded at
heading error above 1.54 rad. **The cap does not bind by construction of the
control law, not merely by observed statistics.** The same argument retires
`wz_std` and `az_max` (measured peak |dwz/dt| 0.192 against 0.80).

### One phrasing correction for whoever reads this next

"The boat is 24.5 s faster" is misleading. Moving-frame median speed is
1.004 (I), 0.981 (H), **0.991** (J) -- flat. The time came from deleting 20.1 s
of stall plus ~4 s of corner efficiency, not from driving faster. Reported as
speed it will send someone hunting a regression that does not exist.

## 21. Loop closed — all three reviewers concur, arm J ships

Final configuration: `PathAngleCritic.cost_weight` 6.0, `max_angle_to_furthest`
0.35. `TwirlingCritic` 10.0, `wz_max` 0.55, `vy_max` 0.45 -- all untouched.
Neither regression contract edited at any point in this exercise.

**5 runs of the shipped configuration, 3 channel + 2 sprint, all PASS,
`contact: false` on every one.**

| channel | I (base) | J | J2 | J3 |
|---|---|---|---|---|
| elapsed | 126.4 | 101.9 | 104.4 | 103.1 |
| min clearance | 0.352 | 0.307 | 0.321 | **0.349** |
| mean-of-4-gate | 0.419 | 0.408 | 0.400 | 0.435 |
| crab frames | 27.2% | 5.1% | 6.5% | 6.9% |
| sideslip >25 deg @ 0.5 m/s | 11.5% | 0.0% | 0.0% | 1.0% |
| sideslip p90 @ 0.5 m/s | 27.5 deg | 11.1 | 12.7 | 13.0 |
| mid-task pause | 0.0%* | 2.5% | 2.0% | 2.3% |

\* the baseline's 20 s of stall sits inside the terminal window, so the
"mid-task" filter scores it 0.0% -- which is exactly why the pause metric alone
was never sufficient and had to be paired with monotonicity.

Channel min clearance at n=3 is **0.307 / 0.321 / 0.349, mean 0.326**, against a
0.330 +/- 0.063 reference. The scare in section 20 is fully retired.

| sprint | I (base) | A (old X) | J | J2 |
|---|---|---|---|---|
| elapsed | 149.4 | 141.6 | 105.9 | **95.3** |
| min clearance | 0.123 | 0.119 | 0.164 | 0.127 |
| crab frames | 29.0% | 26.0% | 6.9% | **0.8%** |
| sideslip >25 deg @ 0.5 m/s | 21.0% | -- | 0.0% | 0.0% |
| arrival -> leg change | 20.0 s | 19.7 | 2.5 | **1.6** |

### The monotonicity criterion needs an amplitude gate — recorded honestly

Every pause episode, net/total |dyaw|:

```
J_channel   94.1-96.6  net  1.0 tot  6.2  ratio 0.17   at 95% of run
J2_channel  94.6-96.7  net 10.6 tot 10.6  ratio 1.00   at 93% of run
J3_channel  93.8-96.1  net  4.0 tot  5.0  ratio 0.81   at 93% of run
J_sprint    17.2-21.7  net 36.3 tot 36.3  ratio 1.00
            39.7-41.3  net 15.0 tot 15.0  ratio 1.00
            86.8-89.2  net 23.9 tot 23.9  ratio 1.00
J2_sprint   16.4-19.7  net 25.4 tot 25.4  ratio 1.00
            36.4-39.3  net 24.4 tot 24.4  ratio 1.00
            76.0-78.8  net 38.4 tot 38.4  ratio 1.00
```

Two ratios fall below the 0.90 threshold. **Both are the same 2-3 s
goal-approach settle at 93-95% of the channel run, and both have a total
rotation of 5-6 degrees** -- the ratio is dividing noise by noise. The identical
episode in J2 scores 1.00 purely because its amplitude happened to be 10.6 deg.
**With an amplitude gate of total >= 10 deg, every qualifying episode in all
five runs is exactly 1.00.** The criterion is sound; as originally written it
was not, and the fix is the gate, not a re-reading of the result.

Every mid-task episode of real amplitude (15-38 deg) is perfectly monotone.
The baseline contained an episode that rotated 18 deg net through 84 deg of
travel. That distinction -- turning versus dithering -- is the whole result.

### The yaw ceiling hypothesis is refuted, free

The second planner flagged a "suspiciously tight ceiling" at max |wz| ~ 0.26
across a 6.3x layout change and a 3x weight change, and proposed checking
whether any recorded trace exceeds 0.27. Checked across **44 traces**: five do,
and the global maximum is **0.506 rad/s -- 92% of `wz_max`** (in
`tuning/sprint_r1_trace.json`); J2_sprint reaches 0.315 and J2_channel 0.278.
There is no ceiling. The boat uses the cap when it needs it and rarely needs it,
which is a different and more comfortable fact than a ceiling would have been.
The `wz_max` null stands on the p99 (0.25), not on the max.

### Both reviewers' concessions, for the record

The second planner withdrew its own `vy_max` arm, its own stopping criterion,
and its own final arm (`offset_from_furthest` 4 -> 10), noting that it had
killed the `vy_max` arm with the argument "sideslip is a ratio; the numerator is
already small" and then failed to apply it one paragraph later to its own
residual. Its `offset_from_furthest` mechanism was nonetheless **verified in
source** and is correct as an explanation of the low-speed regime -- it was
simply not a defect worth trading gate clearance for.

My own retractions across this exercise: the "critic dead band" mechanism (does
not exist), plan-age as evidence against a blocked action server (invalid), and
the channel clearance regression (a single-frame extreme-value statistic
comparing two different obstacles).

### Known and deliberately unpursued

The boat decelerates from ~0.98 to ~0.08 m/s to take the sprint's 92 degree
corner rather than carrying speed through it, at 39% of yaw cap and 25% of
available thrust. Worth roughly 4 s on a 95 s run. Every candidate lever for it
(more `PathAngle` weight -- provably inert, since the equilibrium `w = e/2.8` is
independent of the weight once `W_tw < 2.8*W_pa`; a shorter `PathAlign`; a
longer aim point) trades against gate clearance, which is the binding contract.
Not a defect against the stated target. Recorded, not chased.
