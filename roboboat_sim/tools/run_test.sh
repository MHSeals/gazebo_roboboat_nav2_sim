#!/usr/bin/env bash
# Bring the sim up, run a test against it, tear everything down.
#
#   tools/run_test.sh smoke        # sim only + tools/smoke_test.py
#   tools/run_test.sh nav          # full stack + tools/nav_test.py
#   tools/run_test.sh nav --goals "20,6,0"
#
# Teardown matters more than it looks. `gz sim` is a ruby wrapper that execs
# gz-sim-server, so killing the launch process leaves the server alive. A
# second run then talks to BOTH servers over gz transport: two publishers on
# /model/roboboat/odometry, two authorities for odom->base_link, and tf2
# rejecting the conflicts with "Failure to set received transform". It looks
# exactly like a broken TF configuration. Hence the process group.
# NB: no `set -u`. ROS's setup.bash dereferences unset variables
# (AMENT_TRACE_SETUP_FILES and friends) and dies instantly under it.
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-smoke}"
shift || true

# Software rendering: the GPU lidar needs a GL context even headless, and
# there is no GPU in CI-style environments.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

# shellcheck source=/dev/null
. "$WS/tools/env.sh" > /dev/null 2>&1

LAUNCH_PGID=""

cleanup() {
  if [[ -n "$LAUNCH_PGID" ]]; then
    kill -TERM "-$LAUNCH_PGID" 2>/dev/null
    sleep 3
    kill -KILL "-$LAUNCH_PGID" 2>/dev/null
  fi
  # Belt and braces: gz-sim-server can outlive its process group if the ruby
  # wrapper already exited.
  pkill -KILL -f gz-sim-server 2>/dev/null
  pkill -KILL -f 'gz_tools_vendor/bin/gz sim' 2>/dev/null
  sleep 1
  return 0
}
trap cleanup EXIT INT TERM

case "$MODE" in
  smoke)
    LAUNCH="roboboat_bringup sim.launch.py headless:=true"
    TEST="$WS/tools/smoke_test.py"
    SETTLE=70
    ;;
  nav)
    LAUNCH="roboboat_bringup boat_nav.launch.py headless:=true rviz:=false"
    TEST="$WS/tools/nav_test.py"
    SETTLE=90
    ;;
  *)
    echo "usage: run_test.sh {smoke|nav} [test args...]" >&2
    exit 1
    ;;
esac

LOG="${LOG:-/tmp/roboboat_${MODE}.log}"
echo "==> launching ($MODE), logging to $LOG"
# setsid puts the whole launch tree in its own process group so cleanup can
# signal every descendant at once.
setsid ros2 launch $LAUNCH > "$LOG" 2>&1 &
LAUNCH_PGID=$!

echo "==> waiting ${SETTLE}s for Gazebo and the stack to settle"
echo "    (software rendering makes sensor startup slow; this is not a hang)"
sleep "$SETTLE"

echo "==> running $(basename "$TEST")"
python3 "$TEST" "$@"
STATUS=$?

echo
echo "==> log warnings and errors"
grep -aiE '\[ERROR\]|\[FATAL\]|exception' "$LOG" \
  | grep -av 'Ogre material scripts' \
  | grep -av 'internal gazebo.material' \
  | head -10

exit "$STATUS"
