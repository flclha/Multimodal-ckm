#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 3 ]; then
  echo "usage: $0 GROUP YAML OUTPUT_ROOT" >&2
  exit 2
fi

GROUP=$1
YAML=$2
OUTPUT_ROOT=$3
VERSION=/home/nuc/versions/version5
TOOLS="$VERSION/tools/camera_timestamp_diag"
OUT="$OUTPUT_ROOT/$GROUP"
VIDEO_DEVICE=/dev/fast_livo_camera
META_DEVICE=/dev/video1
CAPTURE_SECONDS=130
META_SECONDS=133

mkdir -p "$OUT"
cp "$YAML" "$OUT/config.yaml"
screen -S ab_camera -X quit >/dev/null 2>&1 || true
screen -S ab_metadata -X quit >/dev/null 2>&1 || true
sleep 2

expected=$(awk '/^Backlight:/ {print $2; exit}' "$YAML")
test -n "$expected"

screen -L -Logfile "$OUT/metadata.log" -dmS ab_metadata \
  "$TOOLS/uvc_metadata_capture" "$META_DEVICE" "$OUT/metadata_frames.csv" "$META_SECONDS"

screen -L -Logfile "$OUT/camera_driver.log" -dmS ab_camera \
  bash --noprofile --norc -c \
  "source '$VERSION/ros_env.sh'; export FAST_LIVO_TIME_DIAG_CSV='$OUT/video_frames.csv'; exec rosrun mvs_ros_driver uvc_driver '$YAML'"

verified=0
for _ in $(seq 1 50); do
  if grep -q "Set and verified backlight=$expected" "$OUT/camera_driver.log" 2>/dev/null; then
    verified=1
    break
  fi
  sleep 0.2
done
if [ "$verified" -ne 1 ]; then
  tail -n 80 "$OUT/camera_driver.log" >&2 || true
  exit 3
fi

v4l2-ctl -d "$VIDEO_DEVICE" --get-ctrl=backlight_compensation \
  | tee "$OUT/backlight_readback.txt"
grep -Eq "backlight_compensation:[[:space:]]*$expected$" "$OUT/backlight_readback.txt"
printf 'expected=%s\nverified=%s\n' "$expected" "$expected" > "$OUT/control_check.txt"

echo "$GROUP formal acquisition running for $CAPTURE_SECONDS seconds"
sleep "$CAPTURE_SECONDS"

screen -S ab_camera -X stuff $'\003' >/dev/null 2>&1 || true
sleep 2
screen -S ab_camera -X quit >/dev/null 2>&1 || true

for _ in $(seq 1 30); do
  if ! screen -ls 2>/dev/null | grep -q '[.]ab_metadata'; then
    break
  fi
  sleep 0.2
done
screen -S ab_metadata -X quit >/dev/null 2>&1 || true

test -s "$OUT/video_frames.csv"
test -s "$OUT/metadata_frames.csv"
wc -l "$OUT/video_frames.csv" "$OUT/metadata_frames.csv" \
  | tee "$OUT/row_counts.txt"
echo "$GROUP COMPLETE"

