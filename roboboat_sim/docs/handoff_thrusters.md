# Handoff: re-do the thruster layout, then re-tune and re-test Nav2

This is an experiment handoff and result record. For normal operation, begin
with the [workspace README](../README.md); after any propulsion change, follow
[MPPI tuning](mppi_tuning.md), [Behavior trees](behavior_trees.md), and the
documented regression sequence before trusting task results.

You are picking up a working RoboBoat simulator. Read this before touching
anything; most of it is things that have already cost a run to learn.

Repo: `evan-destafeno/arch-dotfiles`, branch
`claude/gazebo-boat-mppi-nav2-8w2nq3`, workspace `roboboat_sim/`.
The full experimental record is `tuning/JOURNAL.md` — read at least its
retractions section.

---

## 1. The job

The vehicle owner has looked at the thruster layout and says it is wrong: the
four motors should not be in an **X configuration**. They should be 45°
motors arranged so the boat gets **spin, thrust, surge and strafe** — i.e. a
rotationally symmetric layout where every thruster contributes to yaw in the
same direction.

**They are right, and the arithmetic is worse than they think.** See §3.

> **Status, after the relayout was done** (branch
> `claude/roboboat-thruster-config-kq7vr7`; full record in `tuning/JOURNAL.md`):
>
> The pinwheel was built to the sketches and **measured worse**: it failed the
> channel contract (1 outright failure and 1 marginal pass in 4 runs), because a
> pinwheel makes YAW the all-four-forward mode and spends 28.6% of the surge to
> do it — while measured peak yaw demand on this course is 0.24 rad/s against a
> 0.55 cap that never binds. It is kept selectable for A/B.
>
> **The tangential layout ships.** The ask was never the pinwheel: forward
> thrust on every motor drives the boat *forward*, so all four forward is surge
> exactly as in the X, and yaw is still a mix of forward and reverse — just far
> easier. That is one sign flip on each cant. The X aimed each thrust line 9°
> off the radius from the CoG, nearly at the pivot, so its moments cancelled to
> 0.106 m of a 0.680 m radius; tangential aims it 81° off and gets 0.672 m.
>
> **6.3× the turning moment, with Fx and Fy identical to the newton.** Nothing
> is given up, no motor moves, and every MPPI parameter stays frozen.
>
> Measured against the X baseline: channel clearance 0.334 → 0.311 (sd 0.055 →
> 0.017), elapsed 132.5 → 128.5 s, sprint 0.119 → 0.126, and thruster
> saturation **24.5% → 2.1% of frames**. Both contracts hold on 6 channel and
> 2 sprint runs; neither was loosened.
>
> The pinwheel (all-forward = yaw) was built and measured first: it costs 28.6%
> of surge and failed the channel on 1 run in 4. Kept selectable, rejected.
>
> Three numbers in §3.3 are wrong and are struck there. Sprint elapsed swung
> 118–154 s across arms whose sprint-relevant parameters barely differ, so it
> is not a usable instrument at n=1 in either direction.

Your task, in order:

1. Confirm the intended geometry against the owner's sketches (§3.4) and
   implement it.
2. Re-derive the velocity envelope and re-tune the parts of `nav2_mppi.yaml`
   that depend on it.
3. Re-test. Both existing task contracts must still hold.
4. Report what actually changed, including anything that got worse.

---

## 2. What already works, and must keep working

Two tasks are implemented, scored the way the competition scores them, and
frozen behind contracts. `tools/regress.sh` runs both unattended.

| task | type | current result |
|---|---|---|
| `channel` | `gate_transit` | 10/10 gates in order, no contact, 54% of available clearance, 132 s |
| `sprint` | `circle_mark` | 1.00 laps ccw, gate transited both ways, 29% of available clearance, 153 s |

Contracts live in `tuning/regression_channel.json` and
`tuning/regression_sprint.json`. They are two-tier on purpose:

* `require` — pass/fail. Breaking one fails the build.
* `drift_warn` — reported, never fatal, because a legitimate change moves
  metrics by construction.

The clearance floor is expressed as a **fraction of the achievable maximum**,
not an absolute metre value, so it keeps meaning if gate widths or the hull
change. Keep it that way.

**A changed thruster layout changes the achievable envelope, so it will move
these numbers.** That is expected. What is not acceptable is a task that stops
passing. If you have to loosen a contract, say so explicitly and justify it
with measurements, do not quietly edit the JSON.

---

## 3. The finding you are acting on

### 3.1 What is there now

`roboboat_control/allocation.py::x_configuration` places four thrusters at
`(±0.55, ±0.40)` m from the CoG, each canted 45°:

```
fl (+x, +y) @ +45°     fr (+x, −y) @ −45°
rl (−x, +y) @ −45°     rr (−x, −y) @ +45°
```

### 3.2 Why that is bad

A thruster's yaw contribution is `Mz = x·sin(a) − y·cos(a)`. For this layout
that is `(x − y)·cos45` — **the longitudinal and lateral offsets subtract.**
With x = 0.55 and y = 0.40 they nearly cancel:

```
yaw arm per thruster   0.106 m
```

against a physical radius from the CoG of **0.680 m**. The layout throws away
84% of the moment arm it has.

### 3.3 What a rotationally symmetric layout gives

Point each thruster along the tangent instead, so all four moments add:

```
fl (+x, +y) @ +135°    fr (+x, −y) @ +45°
rl (−x, +y) @ −135°    rr (−x, −y) @ −45°
```

Measured with the existing allocator (`max_forward` 35 N, `max_reverse` 25 N):

| | Fx | Fy | Mz | yaw arm |
|---|---|---|---|---|
| current X | 98.99 N | ~~84.85 N~~ **70.71 N** | ~~12.73~~ **10.61 N·m** | 0.106 m |
| pinwheel | ~~84.85 N~~ **70.71 N** | 70.71 N | **94.05 N·m** | 0.672 m |

~~**7.4× the yaw authority for a 14% loss of surge, and sway unchanged.**~~

**CORRECTED: 8.9× the yaw authority for a 28.6% loss of surge force (18.5% of
speed), and sway unchanged — but that 28.6% is the loss when travelling
perfectly straight, and it reverses as soon as the boat turns.** The most surge
still available while also holding a yaw moment:

| Mz (N·m) | X layout Fx | pinwheel Fx |
|---|---|---|
| 0 | 98.99 N | 70.71 N |
| 4 | 72.33 N | 74.92 N |
| 10 | 32.33 N | 81.24 N |

In the X layout each N·m of yaw costs 6.7 N of surge; in the pinwheel surge
*rises* with yaw. The curves cross at Mz ≈ 3.6 N·m. The struck figures are this table computed at
`max_reverse = 30 N`; the config says 25 N. Sweeping that one parameter
reproduces the original table exactly at 30 N and gives a *zero* surge loss at
35 N — because a pinwheel needs `(−,+,−,+)` to surge and is therefore always
reverse-limited, while the X layout surges on `(+,+,+,+)`. So the whole surge
cost is the forward/reverse asymmetry, and §8 question 2 sets its size.

Also: the X layout's arms are `+,−,+,−`, so all four at full forward give
**exactly zero** yaw. Its 10.61 N·m needs half the motors reversed.

This matters for the sprint specifically. While circling, the boat currently
uses `|wz|` averaging 0.247 rad/s against the 0.106 rad/s a clean 4 m circle at
its speed would need — 2.3× more yaw than the manoeuvre requires — plus
0.14 m/s of sway. It is not gliding round the ring; it is repeatedly hauling
its head round toward the next waypoint and crabbing. That is the signature of
yaw starvation.

### 3.4 What you must check before implementing

The two sketches the owner supplied show a top view: two pontoons with a motor
at each corner, canted. **Confirm the sign convention against the sketches
before you commit to the angles above** — a pinwheel has two chiralities and
picking the wrong one inverts yaw, which will look like a controller bug.
The sketches are committed alongside this document:

* `docs/img/thruster_layout_current.html` — **the layout as it actually
  ships**: a scale plan view of the (0.60, 0.25) mounting, with the motor
  pattern for surge, sway and yaw. Self-contained, opens from disk.
* `docs/img/thruster_layout_hull.jpeg` — the two pontoons with a canted motor
  at each corner.
* `docs/img/thruster_layout_pinwheel.jpeg` — the four motors alone, showing the
  rotational arrangement.

Look at them. If they contradict anything in this document, the sketches win
and you should say so rather than reconciling silently.

Also verify: are the motors canted **outward** (thrust lines tangent to a
circle, as modelled above) or **inward**? Both are 45°; they differ in sign
and one of them is what was drawn.

---

## 4. Consequences you must not skip

### 4.1 The damping parameters are now wrong

`config/boat_params.yaml` has `lin_yaw: 8.0`, `quad_yaw: 10.0`. Those were
plausible when peak Mz was 12.7 N·m. With 94 N·m the steady-state yaw rate
solves to **2.69 rad/s (154°/s)**, which is not credible for a 35 kg boat.

Either the damping is too low, or the thrust figures are optimistic, or both.
Do not simply raise `wz_max` to match the new envelope — that is how you get a
config that works in sim and wanders on the water. Get a real number from the
vehicle if you can (a full-throttle spin test gives you steady-state yaw rate
directly, and that pins `lin_yaw` and `quad_yaw` together with the new Mz).
If you cannot, pick a defensible figure, **write down that it is assumed**,
and make `wz_max` conservative against it.

### 4.2 Things that read the layout

Change these together or `tools/validate.py` will catch you (it cross-checks
the URDF against the params):

* `src/roboboat_control/config/boat_params.yaml` — `thrusters.angle_deg` is
  currently a single scalar. A pinwheel needs **per-thruster angles**; the
  schema has to change, not just the value.
* `src/roboboat_description/urdf/roboboat.urdf.xacro` — thruster frames and
  the `deg45` property.
* `src/roboboat_control/roboboat_control/allocation.py` — add the new layout
  function. **Keep `x_configuration` and its tests**; you want to be able to
  A/B the two.
* `src/roboboat_control/test/test_allocation.py` — 10 tests, currently all
  pass. Add coverage for the new layout: the property that every yaw arm has
  the same sign is the one that would have caught the existing defect.
* `src/roboboat_bringup/config/nav2_mppi.yaml` — `vx_max`, `vy_max`, `wz_max`
  and the acceleration limits all derive from the envelope. The header comment
  in that file explains the relationship; keep it accurate.

### 4.3 MPPI parameters that may want revisiting

Only after the envelope is settled and the contracts still pass:

* `wz_max` (currently 0.55) and `wz_std` (0.25) — the obvious ones.
* `TwirlingCritic` `cost_weight: 10.0`. This penalises yaw. It was bracketed
  at 10.0 when yaw was expensive; with cheap yaw the right value may differ.
  It is also the critic whose parameter names are a documented trap — see §5.
* `vy_max: 0.45`. Do not inflate it. Sway on a catamaran is genuinely
  expensive and a previous experiment confirmed that reducing it to 0.10 broke
  goal convergence, but that does not license raising it.

---

## 5. Traps already paid for — do not re-discover these

Each of these cost at least one run. They are in `tuning/JOURNAL.md` in full.

1. **`twirling_cost_power` / `twirling_cost_weight` are dead keys.** Nav2's own
   README documents them; the Jazzy source reads `cost_power` / `cost_weight`.
   An entire experiment measured nothing. `tools/check_critics.py` now compares
   configured weights against what `controller_server` logged — run it.
2. **`--` inside an XML comment is illegal** and silently breaks a xacro or a
   behaviour tree. Hit twice. Use em dashes.
3. **The lidar is a single plane at z = 0.35 m.** `gen_course.py` refuses to
   emit an obstacle the plane would miss. If you move the lidar, that guard is
   what stops you shipping an invisible buoy.
4. **A leg whose final goal lies on its own earlier route completes early.**
   MPPI takes its target from the global path and the goal checker fires on
   proximity to it. `test_task_scoring.py` asserts this property for every leg
   of every task, including against the straight lines between waypoints.
5. **Wall-clock metrics on a sim-time system are biased.** Everything reported
   is on the sim clock. Keep it that way.
6. **Readiness has bitten five times**: a fixed sleep, the action merely
   existing, lifecycle `active`, then goal-acknowledgement timeouts at startup
   and again mid-run. `wait_for_service_timeout` is currently **20000 ms**,
   raised because the follow camera starves the control loop. On real hardware
   restore it to 1000; there a slow ack is information.
7. **Filming changes the run.** At 640×400/6 Hz the boat went *around* the
   sprint gate instead of through it. The chase camera is now 480×300 at 4 Hz.
   If you add rendering, re-check that scored runs still pass.
8. **Python version skew**: system `python3` is 3.11, ROS Jazzy needs 3.12.
   Always `. tools/env.sh` first. Do not run `set -u` before sourcing ROS.
9. **`prune_distance` is not only a look-ahead.** It also sets how many path
   poses exist, and `PathAlignCritic.offset_from_furthest` is a minimum-index
   gate counted in poses at the largest critic weight in the file. Retuning
   `prune_distance` 4.0 -> 3.0 to track a reduced `vx_max` cost the sprint its
   second leg: the boat crawled at 0.10 m/s and timed out at 0.50 laps. Change
   `vx_max` without it.
10. **`/odom` carries yaw-rate spikes of +-125.66 rad/s** (= 2*pi/0.05). They
   come from Gazebo's odometry publisher differencing yaw without unwrapping,
   they appear in about half of all sprint traces, and MPPI consumes `/odom` as
   its initial state. Use the **median** of `|wz|`, never the mean -- the mean
   ranges 0.070 to 0.790 across runs of an unchanged config while the median
   ranges 0.062 to 0.093. Pre-existing and not fixed here (fixing it would have
   confounded the layout A/B); it is worth its own change.

---

## 6. How to run things

```bash
cd roboboat_sim
. tools/env.sh                      # MUST be first; shims python3/ros2/colcon

python3 tools/validate.py           # 14 offline checks, seconds, no sim
python3 tools/test_task_scoring.py  # scoring rules, no ROS
colcon test --packages-select roboboat_control   # allocation + dynamics

bash tools/regress.sh               # both task contracts, ~12 min, headless
bash tools/task_trial.sh sprint     # one task
CHASE=1 bash tools/task_trial.sh channel   # ...and film it

python3 tools/task_run.py --list    # declared tasks
```

After changing the course file, regenerate both worlds:

```bash
python3 src/roboboat_description/scripts/gen_course.py \
  --course src/roboboat_description/config/course_default.yaml \
  --output src/roboboat_description/worlds/roboboat_course.sdf
# ...and again with --cameras into roboboat_course_cameras.sdf
```

Reporting:

```bash
python3 tools/make_report.py --run channel:CARD:TRACE --run sprint:CARD:TRACE \
    --out media/report.html --title "Course run"
python3 tools/make_replay.py --record TRACE --out media/replay.html --title "..."
bash tools/make_video.sh /tmp/frames_LABEL media/run.mp4 6
```

This box has **4 cores** and renders in software. A full regression run is
about 12 minutes. Budget accordingly and run things in the background.

---

## 7. Method

The previous stretch of work used pre-registered predictions with explicit
rejection criteria, and independent critic agents reviewing plans *before* runs
were spent on them. Both earned their keep: a reviewer rejected a contract
built on a single sample that would have failed the next good run, and of four
predictions registered for the sprint, two were wrong — one of them backwards
from the argued mechanism.

Do the same. Specifically:

* Write down what you expect **before** you run it, with the number that would
  falsify it.
* A/B against the existing layout rather than against memory. Both layouts can
  coexist in `allocation.py`.
* Four runs of the *unchanged* config spread 0.245–0.380 m on min clearance.
  Any effect smaller than that spread is not an effect.
* Report what got worse. Surge drops 14% in the pinwheel; if that costs the
  channel time, say so.

---

## 8. Open questions the owner may need to answer

1. Which chirality, and are the motors canted outward or inward? (§3.4)
2. Is 35 N forward / 25 N reverse per thruster still right for the real
   vehicle, and is `(0.55, 0.40)` m the real mounting position?
3. Is there a measured yaw rate from the real boat — even a rough
   full-throttle spin — to pin the yaw damping? (§4.1) Still open: strip
   theory over the hull does not settle it, and 35 N is a *bollard* figure
   that the model applies at 1.7 m/s of thruster inflow.
3b. **Can the motors move to `(0.60, 0.25)` on the real hull** — inboard face
   of each pontoon, 0.10 m short of the ends? That is the whole change. If not,
   the same algebra applies to whatever `x − y` the boat can take: at a 45°
   cant every centimetre of `x − y` is 0.71 cm of yaw arm, free.
4. Is the surge loss acceptable? The speed challenge is scored on time, and
   the pinwheel trades 14% of peak forward thrust for the yaw authority.
