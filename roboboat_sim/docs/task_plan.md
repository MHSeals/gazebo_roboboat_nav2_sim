# Multi-task course: plan

Status: **proposal, under review.** Nothing in `course_default.yaml` has been
modified yet.

## What we have

One course file, one flat list of ten gates, one runner (`tools/course_run.py`)
that sends all ten gate centres to `NavigateThroughPoses` and scores "did the
hull cross each gate line, in order, between the buoys". It passes 10/10 with
no contact, min clearance 0.380 m. That is the thing we must not break.

## The tension in the request, stated up front

"Rebuild the sim to include the full course" and "keep the navigate-the-channel
task intact" cannot both mean *bit-identical geometry* — rebuilding the course
moves the gates, so the 96.04 m path length and the 0.380 m min clearance are
invalidated by construction. "Intact" here is taken to mean:

> the channel task still runs, still transits every one of its gates in order,
> and still makes no contact, on the new course.

The locked baseline is therefore a **pass/fail contract**, not a metric
snapshot. Metrics are recorded for information and are allowed to move.

## Architecture: one world, many tasks

The real course has every task in the water simultaneously; a run visits them
in sequence without anything being reloaded. The sim should match that.

- **One world.** All task geometry coexists in `roboboat_course.sdf`.
- **One Nav2 config.** `nav2_mppi.yaml` is shared. Tasks that need different
  controller behaviour get an extra *named plugin instance* in the same file
  (Nav2's `controller_plugins` is a vector), selected at runtime by a BT
  `ControllerSelector` node — not a second config set.
- **Per-task behaviour trees.** This is where tasks actually differ.
- **Per-task scoring.** Each task type has its own rule; "crossed the gate
  line" does not score a task whose objective is to circle a mark.

### Course file schema change

`gates` and `buoys` gain a `task:` tag. A new top-level `tasks:` list declares,
for each task, its type, its elements and its scoring parameters:

```yaml
tasks:
  - {name: channel, type: gate_transit, gates: [start, chan_a, ...]}
  - {name: speed,   type: circle_mark, gate: speed_entry, mark: speed_mark,
     direction: ccw, laps: 1}
```

`tools/course_run.py` becomes `tools/task_run.py --task <name>`, dispatching on
`type`. `gate_transit` is the existing scorer, moved verbatim so the channel
result cannot change as a side effect of the refactor.

## Task 3 (speed challenge / Emergency Response Sprint)

Objective: enter through a green/red gate, circle a mark buoy, exit back
through the same gate. Circling direction is signalled at run time.

**Waypoints.** A circle cannot be expressed as a single goal pose. The task
generates a ring of N waypoints at radius R about the mark, ordered in the
commanded direction, bracketed by an approach pose outside the gate and a
return pose outside the gate.

**Scoring.** Not "crossed a line". Accumulate the signed bearing of the boat
about the mark over the whole track; require

- entrance gate transited outbound,
- accumulated signed angle about the mark ≥ 2π·laps with the commanded sign
  (so a wrong-direction loop scores zero, not one),
- entrance gate transited inbound afterwards,
- no contact.

**Pre-registered failure mode.** `RemovePassedGoals radius="1.6"` is what made
the channel task work. The ring waypoints will be spaced closer together than
1.6 m for any sensible R and N, so a single pose update could retire the whole
ring at once and the boat would cut straight past the mark — scoring a "gate
transit" while never circling. Mitigations, in order of preference:

1. Set R and N so that ring spacing > 1.6 m (R = 3.0 m, N = 8 gives 2.30 m).
2. Failing that, a dedicated speed-challenge BT with its own
   `RemovePassedGoals` radius, which is exactly the "new BT, same config"
   pattern.

Option 1 is tried first because it costs nothing and keeps one BT. If the ring
spacing constraint forces R so large that the boat leaves the task area, fall
back to option 2.

## Order of work

1. Freeze the channel contract (`tuning/regression_channel.json`) and add
   `tools/regress.sh`. **Before** any course edit.
2. Refactor `course_run.py` → `task_run.py` with `gate_transit` unchanged.
   Re-run the channel task: must still be 10/10. This isolates refactor risk
   from geometry risk.
3. Rebuild the course geometry. Re-run the channel task: must still be N/N.
4. Add the speed challenge. Re-run channel *and* speed.
5. Scaffold the remaining tasks with geometry + scoring stubs, no BT work.

Each step ends with the channel task green before the next begins, so a
regression is always attributable to one change.

## Where the numbers come from

Official RoboBoat documentation for 2026 is behind an authenticated GitBook;
the public overview page states it "does not provide exact gate widths,
precise course dimensions, or complete spatial layout coordinates". Every
dimension in the course file therefore carries a provenance comment: either a
citation, or the word ASSUMED with the reasoning. Assumed numbers are chosen to
be *harder* than the likely real ones where that is cheap, so the config does
not silently depend on generous clearances.
