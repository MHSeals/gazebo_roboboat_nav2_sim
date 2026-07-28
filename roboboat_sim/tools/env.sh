# Source this before building or launching:  . tools/env.sh
#
# Only needed on machines whose default `python3` is not the one ROS was built
# against. Ubuntu 24.04 + ROS 2 Jazzy is python3.12; if `python3 -V` already
# reports 3.12 the shim below is a harmless no-op.
#
# The symptom this fixes is:
#   ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
#   The C extension '..._rclpy_pybind11.cpython-311-...so' isn't present
# which means the interpreter running your node is not the interpreter the
# rclpy C extension was compiled for.

_ROS_DISTRO="${ROS_DISTRO:-jazzy}"
_ROS_PY="$(ls -d "/opt/ros/${_ROS_DISTRO}/lib/python3."* 2>/dev/null | head -1)"
_ROS_PY_VER="${_ROS_PY##*/python}"

# shellcheck source=/dev/null
. "/opt/ros/${_ROS_DISTRO}/setup.bash"

# The shim must be prepended *after* the ROS setup, which puts its own bin
# directory at the front of PATH.
if [ -n "$_ROS_PY_VER" ] && [ "$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')" != "$_ROS_PY_VER" ]; then
  _SHIM=/opt/rospy/bin
  mkdir -p "$_SHIM"
  ln -sf "/usr/bin/python${_ROS_PY_VER}" "$_SHIM/python3"
  ln -sf "/usr/bin/python${_ROS_PY_VER}" "$_SHIM/python"
  # `ros2` and `colcon` are installed with a hardcoded `#!/usr/bin/python3`
  # shebang, so a PATH entry alone does not redirect them.
  for _tool in "/opt/ros/${_ROS_DISTRO}/bin/ros2" /usr/bin/colcon; do
    [ -f "$_tool" ] || continue
    printf '#!/bin/sh\nexec /usr/bin/python%s %s "$@"\n' "$_ROS_PY_VER" "$_tool" \
      > "$_SHIM/$(basename "$_tool")"
    chmod +x "$_SHIM/$(basename "$_tool")"
  done
  export PATH="$_SHIM:$PATH"
  echo "python3 shimmed to python${_ROS_PY_VER} for ROS ${_ROS_DISTRO}"
fi

_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$_WS/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  . "$_WS/install/setup.bash"
fi

unset _ROS_PY _ROS_PY_VER _SHIM _WS _ROS_DISTRO
