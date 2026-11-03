#!/usr/bin/env bash
set -euo pipefail

RUNTIME_USER="${FAST_LIVO_RUNTIME_USER:-nuc}"
RUNTIME_HOME="${FAST_LIVO_RUNTIME_HOME:-/home/$RUNTIME_USER}"
LIVOX_WS="${LIVOX_WS:-$RUNTIME_HOME/ws_livox}"
RUNTIME_TIMESHARE="$RUNTIME_HOME/timeshare"
INTERNAL_TIMESHARE="$RUNTIME_HOME/timeshare.livox_driver_internal"
READY_FILE=/tmp/fast_livo_livox_timeshare_namespace.ready

[ -f "$LIVOX_WS/devel/setup.bash" ] || {
  echo "ERROR: missing Livox workspace setup: $LIVOX_WS/devel/setup.bash" >&2
  exit 1
}

# This script runs as root inside a private mount namespace. The official
# driver can keep its historical writer without sharing an inode with the
# timestamp consumed by the camera.
mount --bind "$INTERNAL_TIMESHARE" "$RUNTIME_TIMESHARE"
printf '%s\n' "$(stat -c %i "$RUNTIME_TIMESHARE")" > "$READY_FILE"

exec setpriv --reuid="$RUNTIME_USER" --regid="$RUNTIME_USER" --init-groups env \
  HOME="$RUNTIME_HOME" USER="$RUNTIME_USER" LOGNAME="$RUNTIME_USER" LIVOX_WS="$LIVOX_WS" \
  bash --noprofile --norc -c 'unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES CATKIN_SHELL PYTHONPATH; source /opt/ros/noetic/setup.bash; source "$LIVOX_WS/devel/setup.bash"; exec roslaunch livox_ros_driver2 msg_MID360.launch'
