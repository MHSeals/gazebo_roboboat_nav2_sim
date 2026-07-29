#!/usr/bin/env bash
# Drive the roboboat_sim container.
#
#   container/rb.sh build              build the image
#   container/rb.sh gui                full stack with Gazebo + RViz on screen
#   container/rb.sh run <cmd...>       run a command in a fresh container
#   container/rb.sh shell              interactive shell in a fresh container
#
#   container/rb.sh tasks              list the tasks the course declares
#   container/rb.sh task <name> [lbl]  drive one task against the RUNNING stack
#
# The last two are the watchable path: they attach to a stack that is already up
# (`gui`) instead of starting their own, so a task can be watched in Gazebo and
# RViz while it runs. `tools/task_trial.sh` is the scored path -- it brings up its
# own headless stack, runs the task and tears everything down. Both exist because
# an observed run and a headless run are measurably different experiments on a
# machine this size; see tuning/JOURNAL.md.
#
# Rootless podman, host networking. Host networking is not a shortcut: ROS 2
# DDS discovery uses multicast, and sharing the host netns avoids needing a
# bridge — which this host's kernel cannot provide anyway.
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HERE/../roboboat_sim"
IMAGE=roboboat:jazzy

# Rendering. Do NOT pass /dev/dri: Ogre2 asks EGL for a hardware device, Mesa
# then refuses LIBGL_ALWAYS_SOFTWARE with "Not allowed to force software
# rendering when API explicitly selects a hardware device", and segfaults in
# driCreateNewScreen3 because the container's Mesa cannot drive the host's
# Intel UHD. With no DRI node visible, EGL finds only llvmpipe and works.
# Set RB_GPU=1 to pass the device through anyway (expect the above crash).
# EGL_PLATFORM=surfaceless is for the offscreen GPU-lidar context only; the
# on-screen GUIs go through Qt/GLX and are unaffected by it.
RENDER_ENV=(
  -e LIBGL_ALWAYS_SOFTWARE=1
  -e GALLIUM_DRIVER=llvmpipe
  -e MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
  -e EGL_PLATFORM=surfaceless
  # Qt otherwise warns and falls back to this path anyway; setting it keeps the
  # Gazebo/RViz consoles readable.
  -e XDG_RUNTIME_DIR=/tmp/runtime-root
)
DEVICES=()
if [[ "${RB_GPU:-0}" == "1" ]]; then
  DEVICES=(--device /dev/dri)
fi

# Only ask podman for a TTY when we actually have one. `rb.sh gui &` and
# `rb.sh gui > log 2>&1` are both normal ways to run the stack, and -it without
# a terminal either errors or swallows the output.
TTY_FLAGS=(-i)
if [[ -t 0 && -t 1 ]]; then
  TTY_FLAGS=(-it)
elif [[ ! -t 0 ]]; then
  TTY_FLAGS=()
fi

# Refuse to start a second container while one is up. --network=host puts every
# container in one DDS domain AND one gz-transport domain, so a second stack is
# not merely redundant: two gz sim instances publish /clock at different sim
# times, TF stamps jump backwards, and both costmaps drop every lidar scan with
# "earlier than all the data in the transform cache". The symptom is an empty
# costmap and goals that never move the boat, which looks like a Nav2 tuning
# problem and is not. Use podman exec for a second shell -- each container gets
# a private /dev/shm, so DDS shared-memory discovery does not work between two
# of them anyway.
guard_single_stack() {
  local running
  running="$(podman ps -q --filter "ancestor=$IMAGE")"
  [[ -z "$running" ]] && return 0
  if [[ "${RB_FORCE:-0}" == "1" ]]; then
    echo "rb.sh: WARNING: $IMAGE already running; RB_FORCE=1, starting anyway." >&2
    return 0
  fi
  {
    echo "rb.sh: refusing to start -- $IMAGE is already running:"
    podman ps --filter "ancestor=$IMAGE" --format '  {{.ID}}  up {{.RunningFor}}'
    echo
    echo "  attach a shell:  podman exec -it $(echo "$running" | head -1) bash"
    echo "  stop it:         podman stop \$(podman ps -q --filter ancestor=$IMAGE)"
    echo "  override:        RB_FORCE=1 $0 $*"
  } >&2
  exit 1
}

# Find the running stack, or explain how to start one. Everything that drives a
# task against an already-running stack goes through here.
running_container() {
  podman ps -q --filter "ancestor=$IMAGE" | head -1
}
require_stack() {
  local running
  running="$(running_container)"
  if [[ -z "$running" ]]; then
    {
      echo "rb.sh: no $IMAGE container is running, so there is no stack to drive."
      echo
      echo "  start one with the GUI:   $0 gui"
      echo "  headless scored trial:    $0 run 'cd /ws && bash tools/task_trial.sh channel'"
    } >&2
    exit 1
  fi
  printf '%s' "$running"
}

# Run something in the RUNNING container. Must be podman exec, not a second
# `rb.sh run`: podman gives each container a private /dev/shm, so DDS
# shared-memory discovery does not work between two of them and `ros2 topic
# list` from a second container returns partial or empty results. That failure
# is silent and looks like a Nav2 problem.
exec_in_stack() {
  local c; c="$(require_stack)"
  exec podman exec -e PYTHONUNBUFFERED=1 "$c" \
    bash -lc "cd /ws && . tools/env.sh > /dev/null 2>&1 && $*"
}

case "${1:-shell}" in
  build)
    exec podman build -t "$IMAGE" -f "$HERE/Containerfile" "$HERE"
    ;;
  gui)
    # The full stack with Gazebo and RViz on screen. No xhost or XAUTHORITY is
    # needed on this host: XWayland's :0 socket is world-writable, /tmp/.X11-unix
    # is already mounted, and the GUIs go through Qt/GLX (EGL_PLATFORM=surfaceless
    # only affects the offscreen GPU-lidar context).
    guard_single_stack "$@"
    shift
    exec podman run --rm "${TTY_FLAGS[@]}" \
      --network=host \
      "${DEVICES[@]}" \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v "$(cd "$WS" && pwd)":/ws \
      -e "DISPLAY=${DISPLAY:-:0}" \
      "${RENDER_ENV[@]}" \
      --shm-size=2g \
      "$IMAGE" \
      bash -lc "cd /ws && . tools/env.sh > /dev/null 2>&1 && \
                exec ros2 launch roboboat_bringup boat_nav.launch.py $*"
    ;;
  tasks)
    exec_in_stack "python3 tools/task_run.py --list"
    ;;
  task)
    # Drive one declared task against the stack that is already up, so it can be
    # watched in Gazebo and RViz. task_run.py is a pure action client -- it never
    # launches or tears down the sim -- and it sets goal.behavior_tree per goal,
    # so a task carries its own tree regardless of how the stack was started.
    #
    #   rb.sh task channel              watch it, record nothing
    #   rb.sh task sprint demo1         also write tuning/demo1_{trace,card}.json
    #
    # Contrast tools/task_trial.sh, which brings up its own headless stack and
    # tears it down afterwards. That is the scored path; this one is the
    # watchable path. They are not interchangeable -- see the warning below.
    shift
    TASK="${1:-channel}"; LABEL="${2:-}"
    ARGS="--task $TASK"
    if [[ -n "$LABEL" ]]; then
      ARGS="$ARGS --record /ws/tuning/${LABEL}_trace.json"
      ARGS="$ARGS --scorecard /ws/tuning/${LABEL}_card.json"
    fi
    # Alternate course/world, same contract as tools/task_trial.sh: both or
    # neither, or the boat is scored against geometry it is not driving through.
    if [[ -n "${COURSE:-}" || -n "${WORLD:-}" ]]; then
      if [[ -z "${COURSE:-}" || -z "${WORLD:-}" ]]; then
        echo "rb.sh: COURSE and WORLD must be set together" >&2; exit 2
      fi
      ARGS="$ARGS --course $COURSE --world $WORLD"
    fi
    {
      echo "rb.sh: running task '$TASK' against the live stack."
      echo "  NOT a scored run. Rendering Gazebo and RViz costs real CPU, and on"
      echo "  this project that has changed trajectories rather than merely"
      echo "  slowing them -- tuning/JOURNAL.md records a filmed sprint where the"
      echo "  boat went around the gate at x=12.4 against a gate spanning 8.8-11.2"
      echo "  under observation load. Use tools/task_trial.sh for numbers."
    } >&2
    exec_in_stack "python3 tools/task_run.py $ARGS"
    ;;
  run)
    guard_single_stack "$@"
    shift
    exec podman run --rm \
      --network=host \
      "${DEVICES[@]}" \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v "$(cd "$WS" && pwd)":/ws \
      -e "DISPLAY=${DISPLAY:-:0}" \
      "${RENDER_ENV[@]}" \
      --shm-size=2g \
      "$IMAGE" \
      bash -lc "$*"
    ;;
  shell)
    guard_single_stack "$@"
    exec podman run --rm -it \
      --network=host \
      "${DEVICES[@]}" \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v "$(cd "$WS" && pwd)":/ws \
      -e "DISPLAY=${DISPLAY:-:0}" \
      "${RENDER_ENV[@]}" \
      --shm-size=2g \
      "$IMAGE" \
      bash
    ;;
  *)
    cat >&2 <<'USAGE'
usage: rb.sh <command>

  build                   build the image
  gui [launch args...]    full stack with Gazebo + RViz on screen
  shell                   interactive shell in a fresh container
  run <cmd...>            run a command in a fresh container

  tasks                   list the tasks declared in the course file
  task <name> [label]     drive one task against the ALREADY-RUNNING stack,
                          so it can be watched. With a label, also writes
                          tuning/<label>_{trace,card}.json.

Watch a task run:
  rb.sh gui &             (or in another terminal)
  rb.sh tasks
  rb.sh task channel

Scored, headless, self-contained -- use this for numbers, not the above:
  rb.sh run 'cd /ws && bash tools/task_trial.sh channel my_label'

Alternate course (task/tasks honour these, both or neither):
  COURSE=/ws/src/roboboat_description/config/course_wide.yaml \
  WORLD=/ws/src/roboboat_description/worlds/roboboat_course_wide.sdf \
  rb.sh task channel_wide
USAGE
    exit 1
    ;;
esac
