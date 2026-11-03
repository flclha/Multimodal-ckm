#!/usr/bin/env bash
set -euo pipefail

VERSION_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION_WS="$VERSION_ROOT/catkin_ws"
LIVOX_WS="${LIVOX_WS:-$HOME/ws_livox}"

unset CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES
unset CATKIN_SHELL
unset PYTHONPATH

set +u
source /opt/ros/noetic/setup.bash
[ -f "$LIVOX_WS/devel/setup.bash" ] || {
  echo "ERROR: missing Livox workspace setup: $LIVOX_WS/devel/setup.bash" >&2
  exit 1
}
source "$LIVOX_WS/devel/setup.bash"
set -u

cd "$VERSION_WS"
catkin_make -DCMAKE_BUILD_TYPE=Release "${@}"

source "$VERSION_ROOT/ros_env.sh"
echo "Built workspace: $VERSION_WS"
echo "fast_livo package: $(rospack find fast_livo)"
echo "mapping binary: $VERSION_WS/devel/lib/fast_livo/fastlivo_mapping"
