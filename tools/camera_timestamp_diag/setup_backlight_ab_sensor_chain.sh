#!/usr/bin/env bash
set -euo pipefail

VERSION=/home/nuc/versions/version5
LIVOX_IP=192.168.1.50
RUNTIME="$VERSION/tools/camera_timestamp_diag/backlight_ab_runtime"
mkdir -p "$RUNTIME"

/usr/local/bin/wait_fast_livo_camera 45

for name in ab_camera ab_metadata mid360_repub mid360_imu_repub livox_driver2 roscore; do
  screen -S "$name" -X quit >/dev/null 2>&1 || true
done
sudo -n pkill -TERM -f '[l]aunch_livox_driver_private_timeshare.sh' 2>/dev/null || true
sudo -n pkill -TERM -f '[l]ivox_ros_driver2_node.*__name:=livox_lidar_publisher2' 2>/dev/null || true
sleep 2

sudo -n ip addr add "$LIVOX_IP/24" dev enp86s0 2>/dev/null || true
sudo -n ip route replace 192.168.1.0/24 dev enp86s0 src "$LIVOX_IP"

screen -L -Logfile "$RUNTIME/roscore.log" -dmS roscore \
  bash --noprofile --norc -c "source '$VERSION/ros_env.sh'; roscore"
for _ in $(seq 1 20); do
  if bash --noprofile --norc -c "source '$VERSION/ros_env.sh'; rostopic list >/dev/null 2>&1"; then
    break
  fi
  sleep 0.5
done

rm -f /home/nuc/timeshare /home/nuc/timeshare.livox_driver_internal \
  /home/nuc/mid360_clock_offset
truncate -s 16 /home/nuc/timeshare
truncate -s 16 /home/nuc/timeshare.livox_driver_internal
sudo -n rm -f /tmp/fast_livo_livox_timeshare_namespace.ready

screen -L -Logfile "$RUNTIME/livox_driver.log" -dmS livox_driver2 \
  bash --noprofile --norc -c \
  "sudo -n unshare --mount --propagation private '$VERSION/launch_livox_driver_private_timeshare.sh'"
for _ in $(seq 1 60); do
  [ -s /tmp/fast_livo_livox_timeshare_namespace.ready ] && break
  sleep 0.2
done
test -s /tmp/fast_livo_livox_timeshare_namespace.ready

screen -L -Logfile "$RUNTIME/mid360_repub.log" -dmS mid360_repub \
  bash --noprofile --norc -c \
  "source '$VERSION/ros_env.sh'; python3 '$VERSION/catkin_ws/src/FAST-Calib/scripts/mid360_udp_livox_republisher.py' --bind-ip '$LIVOX_IP' --topic /livox/lidar --publish-hz 10 --timeshare-path ~/timeshare --clock-offset-path ~/mid360_clock_offset"
screen -L -Logfile "$RUNTIME/mid360_imu_repub.log" -dmS mid360_imu_repub \
  bash --noprofile --norc -c \
  "source '$VERSION/ros_env.sh'; python3 '$VERSION/catkin_ws/src/FAST-LIVO2/scripts/mid360_udp_imu_republisher.py' --bind-ip '$LIVOX_IP' --topic /livox/imu"

sleep 8
bash --noprofile --norc -c \
  "source '$VERSION/ros_env.sh'; timeout 8 rostopic echo -n 1 /livox/lidar/header/stamp >/dev/null"
test -s /home/nuc/mid360_clock_offset
echo SENSOR_CHAIN_READY
