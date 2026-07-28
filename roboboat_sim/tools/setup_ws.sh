#!/usr/bin/env bash
# Build the workspace, or (with --check) run the offline validation that needs
# nothing but Python. Safe to run from anywhere.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_DEFAULT=jazzy

usage() {
  cat <<'EOF'
usage: setup_ws.sh [--check | --build | --deps]

  --check   Offline checks only: validate.py + the unit tests. No ROS needed.
  --deps    apt-install the ROS 2 / Gazebo packages this workspace needs.
  --build   colcon build (default).

Live tests (after --build) go through tools/run_test.sh:
  bash tools/run_test.sh smoke     boat only
  bash tools/run_test.sh nav       full Nav2 stack
EOF
}

offline_checks() {
  echo "==> offline validation"
  python3 "$WS/tools/validate.py"
  echo
  echo "==> unit tests"
  ( cd "$WS/src/roboboat_control" && python3 -m pytest test -q )
}

install_deps() {
  local distro="${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}"
  echo "==> installing dependencies for ROS 2 $distro"
  sudo apt update
  sudo apt install -y \
    "ros-$distro-desktop" \
    "ros-$distro-navigation2" \
    "ros-$distro-nav2-bringup" \
    "ros-$distro-nav2-mppi-controller" \
    "ros-$distro-nav2-smac-planner" \
    "ros-$distro-ros-gz" \
    "ros-$distro-xacro" \
    "ros-$distro-robot-state-publisher" \
    python3-colcon-common-extensions
}

build() {
  if [[ -z "${ROS_DISTRO:-}" ]]; then
    # shellcheck source=/dev/null
    source "/opt/ros/$ROS_DISTRO_DEFAULT/setup.bash"
  fi
  echo "==> colcon build (ROS 2 $ROS_DISTRO)"
  cd "$WS"
  colcon build --symlink-install
  cat <<EOF

Built. Next:

  . $WS/tools/env.sh
  bash $WS/tools/run_test.sh nav
  ros2 launch roboboat_bringup boat_nav.launch.py
EOF
}

case "${1:---build}" in
  --check) offline_checks ;;
  --deps)  install_deps ;;
  --build) build ;;
  -h|--help) usage ;;
  *) usage; exit 1 ;;
esac
