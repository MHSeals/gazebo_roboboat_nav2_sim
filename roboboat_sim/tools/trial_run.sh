#!/usr/bin/env bash
# Run the actual course as a trial: through every gate, in order, no contact.
#
#   tools/trial_run.sh [label]
#   CHASE=1 tools/trial_run.sh    # also film it from the follow camera
#
# This is not the tuning benchmark. benchmark.sh puts its goals in open water
# so that clearance measures the controller rather than where the boat parked;
# that is correct for A/B comparison and useless as a demonstration, because
# it never requires threading a gate. Here the waypoints are the gate centres
# and the run is scored the way the task is: every gate transited between its
# red and green, nothing touched.
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LABEL="${1:-trial}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
# shellcheck source=/dev/null
. "$WS/tools/env.sh" > /dev/null 2>&1

OUT="$WS/tuning"; mkdir -p "$OUT"
SIM_LOG="/tmp/trial_${LABEL}_sim.log"
NAV_LOG="/tmp/trial_${LABEL}_nav.log"

launch_group() {
  local pgid_file="$1"; shift
  local log="$1"; shift
  rm -f "$pgid_file"
  setsid bash -c 'echo $$ > "$0"; exec "$@"' "$pgid_file" "$@" > "$log" 2>&1 &
  for _ in $(seq 1 50); do [[ -s "$pgid_file" ]] && break; sleep 0.1; done
  cat "$pgid_file" 2>/dev/null
}
cleanup() {
  for pgid in "$@"; do [[ -n "$pgid" ]] && kill -TERM "-$pgid" 2>/dev/null; done
  sleep 3
  for pgid in "$@"; do [[ -n "$pgid" ]] && kill -KILL "-$pgid" 2>/dev/null; done
  pkill -KILL -f gz-sim-server 2>/dev/null
  pkill -KILL -f 'gz_tools_vendor/bin/gz sim' 2>/dev/null
  sleep 2; return 0
}

pkill -KILL -f gz-sim-server 2>/dev/null; sleep 1

CHASE_ARG="chase_camera:=false"
[[ -n "${CHASE:-}" ]] && CHASE_ARG="chase_camera:=true"
SIM_PGID=$(launch_group /tmp/trial_sim.pgid "$SIM_LOG" \
    ros2 launch roboboat_bringup sim.launch.py headless:=true "$CHASE_ARG")

if ! python3 "$WS/tools/wait_for_sim.py" --timeout 240; then
  echo "sim never became ready; see $SIM_LOG"; cleanup "$SIM_PGID"; exit 1
fi

NAV_PGID=$(launch_group /tmp/trial_nav.pgid "$NAV_LOG" \
    ros2 launch roboboat_bringup nav2.launch.py)

READY=0
for _ in $(seq 1 50); do
  ACTIVE=1
  for node in bt_navigator controller_server planner_server behavior_server; do
    ros2 lifecycle get "/$node" 2>/dev/null | grep -q '^active' || { ACTIVE=0; break; }
  done
  [[ "$ACTIVE" -eq 1 ]] && { READY=1; break; }
  sleep 3
done
if [[ "$READY" -ne 1 ]]; then
  echo "nav2 never reached active; see $NAV_LOG"; cleanup "$NAV_PGID" "$SIM_PGID"; exit 1
fi

CAP_PID=""
if [[ -n "${CHASE:-}" ]]; then
  rm -rf "/tmp/frames_${LABEL}"
  python3 "$WS/tools/capture_frames.py" --out "/tmp/frames_${LABEL}" \
      --cameras chase --rate 8 --limit 1600 --seconds 460 \
      > "/tmp/capture_${LABEL}.log" 2>&1 &
  CAP_PID=$!
fi

python3 "$WS/tools/course_run.py" \
    --record "$OUT/${LABEL}_trace.json" \
    --scorecard "$OUT/${LABEL}_card.json"
STATUS=$?

if [[ -n "$CAP_PID" ]]; then
  kill -INT "$CAP_PID" 2>/dev/null; wait "$CAP_PID" 2>/dev/null
  echo "  frames          : $(ls "/tmp/frames_${LABEL}" 2>/dev/null | wc -l)"
fi

cleanup "$NAV_PGID" "$SIM_PGID"
python3 "$WS/tools/check_critics.py" --log "$NAV_LOG"
exit "$STATUS"
