# Round 1 plan — full-context planner (Claude, has read all 2294 lines of JOURNAL.md)

## Gap decomposition

Expert 70.3 s vs MPPI 104.5 s at matched clearance, same velocity limits.
Where the 34.2 s sits:

* **path**: 95.4 vs 89.2 m = 6.2 m longer. At 1.0 m/s that is ~6 s.
* **speed**: p50 0.987 vs 1.30 m/s. If the boat held 1.30 over 95.4 m it would
  take 73 s. So ~28 s is speed, and it is the dominant term.
* **rotation**: 575.8° vs 295.5°. 280° of excess.
* **stalls**: 0.0 s. Already fixed (arm J). Not available.
* **sideslip**: p90 13.1°. Already fixed (arm J). Not available.

So: **speed is the prize, and excess rotation is the most likely cause of not
having it.** Every degree of unnecessary yaw on a hull whose sway drag is 3x its
surge drag is thrust spent going nowhere, and it forces the velocity controller
to supply centripetal force that competes with surge in the allocator.

## H1 — the prediction horizon is too short to plan a corner. **Test first.**

`time_steps: 56` × `model_dt: 0.05` = 2.8 s = 3.64 m of look-ahead at `vx_max`.
A 2 m-radius corner is 3.14 m of arc for 90°. So at speed the boat **cannot see
the exit of a corner while it is entering it**. It therefore has no rollout that
demonstrates the corner is exitable at speed, and the cheapest trajectory its
horizon can score is a slow one.

This is the mechanism I believe explains both the speed deficit and the heading
hunting: a controller that re-decides its turn every cycle without ever seeing
the whole turn will oscillate, and 80.7° of rotation for 4.2° of net change at
0.92 m/s is what that looks like.

**Never tested in this workspace.** `time_steps` does not appear anywhere in
JOURNAL.md's 2294 lines. Twenty-one entries of tuning changed critic weights and
velocity caps and never touched the horizon.

The change is coupled values, not independent parameters — the horizon is one
quantity and `prune_distance` exists so rollouts do not overrun the pruned path.

**Registered as H1a, the cheap version, deliberately:**

    time_steps      56 -> 72        (2.8 s -> 3.6 s; 4.68 m at vx_max)
    prune_distance  4.0 -> 5.0      (must exceed the 4.68 m reach)

Two values, and the local costmap does **not** move: 4.68 m reach against a
6.0 m costmap radius still fits, so `validate.py`'s horizon check passes
unchanged. Rollout cost is `batch_size × time_steps`, so **+29%**, not the +61%
a 90-step horizon would cost, and there is no costmap resize (12 → 16 m would
have been +78% cells for the inflation layer and `CostCritic`'s footprint
checks — a second CPU cost I nearly hid inside the first).

Sizing: a 90° turn at 2 m radius is 3.14 m of arc, which at the 1.1 m/s that
corner permits is **2.86 s** — just outside the current 2.8 s horizon and
comfortably inside 3.6 s. So 72 steps is the smallest horizon that can contain
the manoeuvre the mechanism is about. If H1a shows a partial effect, 90 steps is
the monotonicity follow-up, exactly as `TwirlingCritic` was bracketed.

**Registered prediction:** channel elapsed falls to **≤ 95 s** (from 104.5;
5 s is the smallest resolvable move given sd 1.66) and total rotation falls
**below 450°** (from 575.8). **FALSIFIED IF** elapsed ≥ 101 s or total rotation
≥ 520°.

**Guard:** min clearance fraction ≥ 0.20 on every run, `contact: false`, both
contracts pass. Reject if mean clearance falls below 0.170 m (baseline mean
0.221 less one contract sd 0.063 ≈ 0.158; I am setting it slightly tighter).

**CPU cost, stated because it is the real objection:** rollout work is
`batch_size × time_steps` = 2000 × 56 → 2000 × 90, **+61%**. On this 12-core box
that is affordable. On the target 4-core vehicle it may not be, and the journal
records three separate failures caused by CPU starvation presenting as
navigation failures. So if H1 wins, the follow-up is `batch_size` 2000 → 1400 to
buy the horizon back at constant cost, which is itself a testable arm.

## H2 — `wz_max` is not the null the journal thinks it is

Recorded three times as "does not bind": measured |wz| p99 is 0.25 against a cap
of 0.55, with 0.0% of frames above half the cap.

**That argument is circular.** It measures yaw demand *while the boat crawls
through corners*. The feasible-speed table shows a 2 m corner at expert speed
needs `wz = v·k = 1.1 × 0.5 = 0.55` — **exactly the cap**. So "wz never
approaches the cap" is a consequence of the slow cornering, not evidence that
fast cornering would not need it.

I am **not** proposing to raise `wz_max` as its own arm — on the present
behaviour it would be a no-op, and the journal is right about that. I am
recording that **if H1 succeeds, `wz_max` becomes the next binding constraint**,
and the |wz| p99 must be re-measured on the H1 trace before the null is
re-asserted. This is a prediction about what the next measurement will show, not
a change.

## H3 — arm J may have bought the oscillation

Arm J raised `PathAngleCritic.cost_weight` 2.0 → 6.0 to make yawing-to-fix-
heading cheaper than crabbing. It worked: crab frames 27% → 5%, stalls 20 s → 0.

But `PathAngleCritic` scores only the **terminal** yaw of each rollout
(`yaws[:, -1]`), so it is indifferent to what the heading does *inside* the
horizon. `TwirlingCritic` penalises the horizon **mean of |wz|** — which
penalises sustained turning but **not reversal**: 40° left then 40° right costs
the same as 80° in one direction. So nothing in the cost function charges for
changing your mind about heading.

Arm J flipped the ratio 5.6:10.0 → 16.8:10.0 in favour of yawing. 280° of excess
rotation is a plausible price for that, and it was never measured because the
journal's rotation metrics only looked at *pause episodes*, never at the whole
run.

**I am deliberately not testing this first.** It is a trade against a change
that fixed two confirmed problems, and H1 may remove the oscillation without
touching it. If H1 fails, this is next: `TwirlingCritic` 10.0 → 14.0, which
raises the cost of *any* yaw without re-opening the 1.5 regression (recorded
20.7 s worse) or disturbing the arm-J ratio's sign (16.8 > 14.0 still favours
yawing).

## H4 — throttle on the straights

`throttle_use_straight` is 0.807 — on low-curvature fast sections the boat runs
at 81% of the expert's speed. Candidate cause: `GoalCritic` and
`PathFollowCritic` both have `threshold_to_consider: 2.5`, so within 2.5 m of
the goal the boat is being pulled by a positional term rather than a path term,
which on a 10-waypoint task is 10 separate decelerations. **Not yet a mechanism
I trust** — the channel is one `NavigateThroughPoses` leg, so intermediate
waypoints may not trigger goal-critic behaviour at all. Needs a trace check
before it becomes an arm, and that check is free.

## Rabbit holes I am pre-committing to avoid

The journal closed these with evidence; re-opening any of them needs a *new*
mechanism, not a new hope:

* `TwirlingCritic` downward (1.5 measured 20.7 s worse, n=2 agreeing to 1 s).
* `vy_max` in any direction (0.10/0.20/0.25 all refuted; 0.45 is on the right
  side of a real trade).
* `CostCritic.cost_weight` upward (8.00 cost ~0.09 m of clearance everywhere).
* `prune_distance` **downward** (3.0 cost the sprint its second leg).
* Clearance as a target (0.616 m of a 0.65 m geometric maximum on the old
  suite; the binding constraint is geometry, not tuning).
* `offset_from_furthest` on `PathFollowCritic` (it clamps an index, it is not a
  gate — verified in source).

## First action

Run H1. One arm, three coupled values, registered above. n=3 on the channel and
n=2 on the sprint, which is what the measured sd demands to resolve a 5 s move.
