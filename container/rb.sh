#!/usr/bin/env bash
# Drive the roboboat_sim container.
#
#   container/rb.sh build          build the image
#   container/rb.sh run <cmd...>   run a command in the workspace
#   container/rb.sh shell          interactive shell
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

case "${1:-shell}" in
  build)
    exec podman build -t "$IMAGE" -f "$HERE/Containerfile" "$HERE"
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
    echo "usage: rb.sh [build|run <cmd>|shell]" >&2
    exit 1
    ;;
esac
