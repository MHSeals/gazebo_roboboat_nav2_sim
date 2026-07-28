# The RoboBoat task set, and how each one lands in this sim

The 2026 competition runs six tasks on one course. A scored run visits them in
sequence; nothing is reloaded between them. This document is the map from each
task to the machinery it needs here, so that adding one is a known amount of
work rather than an open question.

## The three axes a task can vary on

Every task differs from the navigation channel along at most three axes, and
recognising which ones is the whole planning problem:

| axis | mechanism | cost |
|---|---|---|
| **what to drive** | waypoints generated from the task declaration | a `Task` subclass in `tools/task_run.py` |
| **how to drive it** | a behaviour tree, passed per goal in the `behavior_tree` field of `NavigateThroughPoses` | one XML file |
| **how it is scored** | the `score()` method of that subclass | tests in `tools/test_task_scoring.py` |

What a task almost never needs is a *second Nav2 config*. `controller_plugins`
and `goal_checker_plugins` are vectors: a task that wants slower, tighter
control gets an extra named instance in the same `nav2_mppi.yaml`, selected at
run time by the `ControllerSelector` node already present in the tree. One
parameter file keeps the costmaps, the footprint and the critic set common
across tasks, which is what you want — those are properties of the boat, not
of the task.

The exception is anything that needs a *capability* Nav2 does not have.
Station-keeping is the real one: holding a pose against drift is not a
navigation goal, and neither the controller nor the BT expresses it. That is
new code, not new configuration.

## Task by task

### Task 1 — Evacuation Route and Return

Two red/green gate pairs, entered from at least 6 ft before the first, and
exited back through the same gates at the end of the run.

- **drive**: `gate_transit`, forward through both gates then the reverse list.
- **BT**: the existing `navigate_through_poses_boat.xml`.
- **score**: existing gate-crossing rule, with the outbound and return
  crossings counted separately so a boat that never comes back fails.
- **status**: covered by the existing machinery. The current `channel` task is
  a superset of it.

### Task 2 — Debris Clearance

Gates defining a lane, black obstacle buoys inside it, and coloured indicators
that classify what is found: red = hazard, report and avoid; green = survivor,
circle and report.

- **drive**: `gate_transit` for the lane, plus a conditional `circle_mark` on
  whichever mark is green.
- **BT**: existing, for the transit.
- **score**: gate transits, plus a *report* — the task is scored on what the
  boat says it saw, not only where it went. This needs a reporting channel
  that does not exist yet: a topic the run publishes classifications on, and a
  scorer that checks them against ground truth from the course file.
- **new work**: the classification report. Perception itself is out of scope
  here — the sim has one scan plane, not a camera classifier — so the
  honest version is a ground-truth publisher standing in for perception, with
  the *navigation* consequences (avoid the red, circle the green) genuinely
  driven and genuinely scored.
- **status**: geometry and transit scoring are cheap; the reporting half is a
  new subsystem. Defer the reporting half.

### Task 3 — Emergency Response Sprint (the speed challenge)

A green/red entrance gate, a yellow buoy some distance beyond it, circled in a
direction signalled at run time (red indicator = counter-clockwise, green =
clockwise), then back out through the entrance gate. Scored on time.

- **drive**: `circle_mark` — an approach pose, a ring of waypoints about the
  mark ordered in the commanded direction, and a return pose outside the gate.
- **BT**: existing, *if* the ring waypoint spacing exceeds the tree's
  `RemovePassedGoals` radius of 1.6 m. At radius 3.0 m with 8 points the
  spacing is 2.30 m, so it does. If a smaller ring is ever needed, this task
  gets its own tree with a smaller retirement radius — which is exactly the
  "new BT, same config" pattern.
- **score**: accumulated *signed* angle about the mark. Not waypoint hits: a
  boat that drives to the mark and back crosses every line a correct run
  crosses and hits every waypoint tolerance, and has circled nothing. The
  signed sum also makes a lap driven the wrong way come out negative rather
  than reading as a lap.
- **status**: **implemented and passing**, both directions, under a frozen
  contract in `tuning/regression_sprint.json`. 136 s, one lap, no contact.
  Three things it cost that are worth knowing before building the next task:
  a leg whose final goal lies on its own earlier route completes early; a ring
  overshoot must be a fraction of a waypoint step, never a whole one; and a lap
  measured between two crossings of the same gate does not come to exactly 2*pi.

### Task 4 — Supply Drop

Yellow vessels marked with a black triangle take water; black vessels marked
with a black plus take racquetballs. Up to three balls.

- **drive**: approach and *hold station* at a stand-off from the target while
  the payload is delivered.
- **BT**: needs a hold — Nav2 has no station-keeping behaviour. The cheapest
  honest version is a `Wait` in the tree with the controller commanded to zero
  and the surrogate dynamics left to drift, which measures nothing, or a small
  station-keep node that closes a pose loop directly on `/cmd_vel`. The latter
  is the useful one and is the single largest piece of new code in the whole
  task set.
- **score**: time on station, and pose error while holding.
- **status**: not started. Needs the station-keeper first. This is also the
  piece most worth having for the Unity port, because station-keeping against
  real disturbance is where a physics-free sim stops being a good proxy.

### Task 5 — Navigating the Marina (docking)

Slips 1–3 between a north and a south dock; dock at the lowest available
number.

- **drive**: an approach pose on the slip centreline, then a straight run in,
  then reverse out. The boat is omnidirectional, so the entry does not need a
  turn — but the slip is narrower than the inflation radius, which is the
  interesting part.
- **BT**: needs a tighter goal checker and almost certainly a second named
  controller instance with lower speeds and a shorter horizon. This is the
  motivating case for the plugin-vector design above.
- **score**: hull fully inside the slip, no contact with either divider, held
  for a dwell time.
- **new work**: the arithmetic, now that the real dimensions are in the course
  file. The handbook gives a **1.52 m** clear opening between tines. The hull
  is **1.10 m** across, so there is 0.21 m a side before the paint touches --
  and `inflation_radius` is **1.30 m**, so the whole slip plus 1.3 m of the
  water outside it is inflated from both tines at once. Nav2 as configured
  cannot plan into a slip; it is not a tuning question, the goal is not
  reachable. Options, in the order they should be tried:
    1. A second costmap inflation setting selected per task. Inflation
       parameters are dynamically reconfigurable, so this needs no second
       config file -- but it changes a costmap the channel also uses, which is
       exactly why the regression contract exists.
    2. A docking-specific controller instance with a footprint-aware cost
       critic and no inflation penalty near the goal.
    3. Approaching on a pose sequence that ends outside the slip and driving
       the last 2 m open-loop, which is what a lot of teams actually do and
       which this sim is honest enough to admit is not navigation.
  `opennav_docking` is **not installed** in this workspace, so this is ours.
- **status**: geometry is in the course at documented dimensions -- two docks,
  three slips each, 1.52 m openings, 1.83 m tines. Behaviour is not started,
  deliberately: this is the task that will actually stress the configuration
  and it should not be started while anything else is in flight.

### Task 6 — Harbor Alert

An acoustic signal — 600, 800 or 1000 Hz, ±5% — selects what the boat does
next: one blast sends it to the Task 3 zone, two blasts send it back to the
marina.

- **drive**: nothing new. It selects between existing tasks.
- **BT**: nothing new — the decision happens above the navigation layer.
- **score**: did the boat go to the place the signal named.
- **new work**: a signal source. There is no hydrophone in this sim and adding
  acoustic propagation to a deliberately physics-free world would be a poor
  trade. The right stand-in is a topic carrying the commanded frequency, which
  is exactly the interface a real hydrophone driver would present, so the
  decision logic above it is real even though the sensing below it is not.
- **status**: cheap, and worth doing *last*, because it is a dispatcher over
  tasks that must exist first.

## Order of work, and why

1. **Task 3 (sprint)** — implemented. It exercises a genuinely different
   scoring rule and a genuinely different path shape, which is what proves the
   task framework is a framework and not one function with a flag.
2. **Task 5 (docking)** — next. It is the one that will actually stress the
   Nav2 configuration, because it puts the goal inside inflated space on
   purpose. Everything learned there transfers to the Unity port.
3. **Task 4 (supply drop)** — needs the station-keeper, which is a real piece
   of engineering and should not be started while other things are in flight.
4. **Task 2 (debris)** — needs a reporting channel; the navigation half is
   nearly free once the sprint's `circle_mark` exists.
5. **Task 6 (harbor alert)** — a dispatcher, cheap, last.

Task 1 is already covered.

## What is deliberately not being built

Perception. The sim has a single lidar plane standing in for a whole camera
and 3D-lidar stack, and it is the *output* of that stack — a ring of obstacle
points — that Nav2 consumes either way. Colour classification, placard
recognition and ball launching are all above that line. Where a task is scored
on classification, this sim publishes ground truth on the topic a real
classifier would publish on, so that everything downstream of perception is
genuinely exercised and nothing pretends to be perception.
