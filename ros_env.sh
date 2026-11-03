#!/usr/bin/env bash

# Resolve this version without consulting HOME or the caller's shell startup files.
VERSION_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION_WS="$VERSION_ROOT/catkin_ws"
VERSION_SRC="$VERSION_WS/src"
LIVOX_WS="${LIVOX_WS:-$HOME/ws_livox}"

# A caller may already have sourced ~/catkin_ws/devel/setup.bash from ~/.bashrc.
# Remove the inherited ROS/catkin path state before building this version's overlay.
unset CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES
unset CATKIN_SHELL
unset PYTHONPATH

source /opt/ros/noetic/setup.bash
[ -f "$LIVOX_WS/devel/setup.bash" ] || {
  echo "ERROR: missing Livox workspace setup: $LIVOX_WS/devel/setup.bash" >&2
  return 1 2>/dev/null || exit 1
}
[ -f "$VERSION_WS/devel/setup.bash" ] || {
  echo "ERROR: repository workspace is not built; run $VERSION_ROOT/build.sh" >&2
  return 1 2>/dev/null || exit 1
}
source "$LIVOX_WS/devel/setup.bash"
source "$VERSION_WS/devel/setup.bash"
export PYTHONPATH="$LIVOX_WS/devel/lib/python3/dist-packages:${PYTHONPATH:-}"

expected_fast_livo="$VERSION_SRC/FAST-LIVO2"
actual_fast_livo="$(rospack find fast_livo 2>/dev/null || true)"
if [ "$actual_fast_livo" != "$expected_fast_livo" ]; then
  echo "ERROR: fast_livo resolved to '$actual_fast_livo', expected '$expected_fast_livo'" >&2
  return 1 2>/dev/null || exit 1
fi

export FAST_LIVO_VERSION_ROOT="$VERSION_ROOT"
export FAST_LIVO_WORKSPACE="$VERSION_WS"
export LIVOX_WS
