#!/usr/bin/env bash
# Run the fixed tuning benchmark and write a scorecard.
#
#   tools/benchmark.sh <label> [repeats]
#
# Writes tuning/<label>_r<N>.json. Compare with tools/compare_runs.py.
#
# The suite is deliberately fixed: A/B comparison only means anything if the
# task does not move.
#
# Goals are chosen so that the ENTIRE region the goal checker will accept
# (xy_goal_tolerance 0.50 m, yaw 0.30 rad) sits in clean water -- worst-case
# clearance anywhere in that ball is +1.8 m or better. The previous suite did
# not: goal (30,8) sat 0.92 m from a buoy surface, inside the boat's own
# circumscribed radius, and the boat could legally declare success anywhere
# between -0.20 m and +0.64 m clearance. Minimum clearance then measured
# where inside the tolerance ball it happened to stop, which no controller
# parameter influences.
#
# The straight line BETWEEN consecutive goals still passes through buoys
# (-0.20 m if flown directly), so avoidance is genuinely exercised en route.
# That is the part worth measuring.
#
# Startup is two-phase on purpose. Launching Nav2 at the same instant as
# Gazebo races the sim clock's first tick: the lifecycle manager times a
# transition against a clock that has just jumped, declares a node failed
# ~100 ms after asking it to configure while that node is still loading
# plugins, and aborts the whole bringup. Every goal then returns ABORTED in
# zero seconds. Gating on real preconditions removes the race entirely.
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LABEL="${1:?usage: benchmark.sh <label> [repeats]}"
REPEATS="${2:-1}"

export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
# shellcheck source=/dev/null
. "$WS/tools/env.sh" > /dev/null 2>&1

# Long leg through the channel, a turn into the speed gate, then a return.
GOALS="10,6,0.6 34.5,9.5,0.9 33,25,2.0 12,26,3.0"
OUT="$WS/tuning"
mkdir -p "$OUT"

# Launch in a new session and record the real process group id.
#
# `setsid cmd &` then `$!` does NOT give you the new group: setsid forks, so
# $! is setsid's own pid and `kill -- -$!` signals nothing. Stale Gazebo
# servers then survive every run, pile up, and the next run inherits both
# their CPU load and their duplicate /odom and TF publishers. That presents as
# Nav2 lifecycle timeouts and a boat frozen at coordinates it never drove to.
# Having the session leader report its own pid is unambiguous.
launch_group() {
  local pgid_file="$1"; shift
  local log="$1"; shift
  rm -f "$pgid_file"
  setsid bash -c 'echo $$ > "$0"; exec "$@"' "$pgid_file" "$@" > "$log" 2>&1 &
  for _ in $(seq 1 50); do
    [[ -s "$pgid_file" ]] && break
    sleep 0.1
  done
  cat "$pgid_file" 2>/dev/null
}

cleanup_stack() {
  for pgid in "$@"; do
    [[ -n "$pgid" ]] || continue
    kill -TERM "-$pgid" 2>/dev/null
  done
  sleep 3
  for pgid in "$@"; do
    [[ -n "$pgid" ]] || continue
    kill -KILL "-$pgid" 2>/dev/null
  done
  pkill -KILL -f gz-sim-server 2>/dev/null
  pkill -KILL -f 'gz_tools_vendor/bin/gz sim' 2>/dev/null
  sleep 2
  return 0
}

# Refuse to measure anything while a previous stack is still alive: leftover
# publishers corrupt odom and TF, and leftover CPU load changes the result.
assert_clean() {
  local leftovers
  leftovers=$(pgrep -f 'gz-sim-server|nav2_controller/controller_server|ros_gz_bridge/parameter_bridge' 2>/dev/null | wc -l)
  if [[ "$leftovers" -ne 0 ]]; then
    echo "    $leftovers stale process(es) found; killing before measuring"
    pkill -KILL -f gz-sim-server 2>/dev/null
    pkill -KILL -f 'gz_tools_vendor/bin/gz sim' 2>/dev/null
    pkill -KILL -f 'nav2_controller/controller_server' 2>/dev/null
    pkill -KILL -f 'ros_gz_bridge/parameter_bridge' 2>/dev/null
    pkill -KILL -f 'robot_state_publisher' 2>/dev/null
    pkill -KILL -f 'roboboat_control' 2>/dev/null
    sleep 3
  fi
}

STATUS=0
for run in $(seq 1 "$REPEATS"); do
  CARD="$OUT/${LABEL}_r${run}.json"
  SIM_LOG="/tmp/bench_${LABEL}_r${run}_sim.log"
  NAV_LOG="/tmp/bench_${LABEL}_r${run}_nav.log"
  echo "==> $LABEL run $run/$REPEATS -> $(basename "$CARD")"

  assert_clean
  # CHASE=1 mounts the follow camera and captures frames alongside the run,
  # so the footage and the recorded trace describe the same journey.
  CHASE_ARG="chase_camera:=false"
  [[ -n "${CHASE:-}" ]] && CHASE_ARG="chase_camera:=true"
  SIM_PGID=$(launch_group "/tmp/bench_sim.pgid" "$SIM_LOG" \
      ros2 launch roboboat_bringup sim.launch.py headless:=true "$CHASE_ARG")

  if ! python3 "$WS/tools/wait_for_sim.py" --timeout 240; then
    echo "    sim never became ready; see $SIM_LOG"
    cleanup_stack "$SIM_PGID"
    STATUS=1
    continue
  fi

  NAV_PGID=$(launch_group "/tmp/bench_nav.pgid" "$NAV_LOG" \
      ros2 launch roboboat_bringup nav2.launch.py)

  # Gate on the lifecycle nodes reaching ACTIVE, not on the action existing.
  #
  # Two wrong versions of this preceded the current one, and both cost runs:
  # a fixed 20 s sleep (lost a twirl20 run when activation ran long), then a
  # poll for `ros2 action list | grep navigate_to_pose` -- which passes as
  # soon as the server is CREATED, during configure, well before activate.
  # The benchmark then sent goals to an inactive bt_navigator and every one
  # came back "goal rejected" with zero stack errors.
  READY=0
  for _ in $(seq 1 50); do
    ACTIVE=1
    for node in bt_navigator controller_server planner_server behavior_server; do
      if ! ros2 lifecycle get "/$node" 2>/dev/null | grep -q '^active'; then
        ACTIVE=0; break
      fi
    done
    if [[ "$ACTIVE" -eq 1 ]]; then READY=1; break; fi
    sleep 3
  done
  if [[ "$READY" -ne 1 ]]; then
    echo "    nav2 never reached active; see $NAV_LOG"
    cleanup_stack "$NAV_PGID" "$SIM_PGID"
    STATUS=1
    continue
  fi

  RECORD_ARG=()
  [[ -n "${RECORD:-}" ]] && RECORD_ARG=(--record "$OUT/${LABEL}_r${run}_trace.json")

  CAP_PID=""
  if [[ -n "${CHASE:-}" ]]; then
    rm -rf "/tmp/frames_${LABEL}"
    python3 "$WS/tools/capture_frames.py" --out "/tmp/frames_${LABEL}" \
        --cameras chase --rate 8 --limit 900 --seconds 260 \
        > "/tmp/capture_${LABEL}.log" 2>&1 &
    CAP_PID=$!
  fi

  python3 "$WS/tools/nav_test.py" --goals "$GOALS" --timeout 190 \
      --scorecard "$CARD" "${RECORD_ARG[@]}" 2>&1 | sed 's/^/    /'
  [[ ${PIPESTATUS[0]} -eq 0 ]] || STATUS=1

  if [[ -n "$CAP_PID" ]]; then
    kill -INT "$CAP_PID" 2>/dev/null
    wait "$CAP_PID" 2>/dev/null
    echo "    frames: $(ls /tmp/frames_${LABEL} 2>/dev/null | wc -l)"
  fi

  cleanup_stack "$NAV_PGID" "$SIM_PGID"

  # A config key the code does not read is silently ignored, so an
  # experiment can "change" a parameter and measure nothing. Verify against
  # what the controller actually logged.
  python3 "$WS/tools/check_critics.py" --log "$NAV_LOG" || STATUS=1

  ERRORS=$(grep -ac '\[ERROR\]' "$NAV_LOG" 2>/dev/null || echo 0)
  echo "    stack errors: $ERRORS"
done

exit "$STATUS"
