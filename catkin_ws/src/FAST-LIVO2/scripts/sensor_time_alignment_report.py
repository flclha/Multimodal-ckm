#!/usr/bin/env python3
"""Print a pre-mapping LiDAR/IMU/camera header timestamp alignment report."""

import argparse
import bisect
import math
import re
import sys
import threading
import time

import rospy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Image, Imu


TOPICS = {
    "lidar": ("/livox/lidar", CustomMsg),
    "imu": ("/livox/imu", Imu),
    "camera": ("/uvc_camera/image_sync", Image),
}


def percentile(values, percent):
    if not values:
        return math.nan
    ordered = sorted(values)
    rank = (len(ordered) - 1) * percent / 100.0
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return ordered[low]
    weight = rank - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def median(values):
    return percentile(values, 50.0)


def load_simple_opencv_yaml(path):
    values = {}
    if not path:
        return values
    try:
        with open(path, "r", encoding="utf-8") as stream:
            for raw_line in stream:
                line = raw_line.split("#", 1)[0].strip()
                match = re.match(r"^([A-Za-z0-9_]+):\s*(.*?)\s*$", line)
                if not match:
                    continue
                value = match.group(2).strip().strip('"').strip("'")
                values[match.group(1)] = value
    except OSError as exc:
        print("[time-align] WARN: cannot read camera config {}: {}".format(path, exc))
    return values


def nearest_signed_ms(reference, candidates):
    ordered = sorted(candidates)
    deltas = []
    for stamp in reference:
        # Do not report an artificial large maximum from the beginning/end of
        # the finite sampling window when the other stream has no bracketing
        # sample yet.
        if stamp < ordered[0] or stamp > ordered[-1]:
            continue
        index = bisect.bisect_left(ordered, stamp)
        choices = []
        if index < len(ordered):
            choices.append(ordered[index])
        if index > 0:
            choices.append(ordered[index - 1])
        if choices:
            nearest = min(choices, key=lambda value: abs(stamp - value))
            deltas.append((stamp - nearest) * 1000.0)
    return deltas


def sequence_stats(stamps):
    duplicate = 0
    rollback = 0
    for previous, current in zip(stamps, stamps[1:]):
        delta = current - previous
        if abs(delta) < 1e-12:
            duplicate += 1
        elif delta < 0.0:
            rollback += 1
    span = stamps[-1] - stamps[0] if len(stamps) >= 2 else 0.0
    hz = (len(stamps) - 1) / span if span > 0.0 else 0.0
    return hz, duplicate, rollback


def ros_param(name, default="N/A"):
    try:
        return rospy.get_param(name, default)
    except Exception:
        return default


def fmt(value, digits=3):
    try:
        return ("{:.%df}" % digits).format(float(value))
    except (TypeError, ValueError):
        return str(value)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--max-current-spread", type=float, default=0.5)
    parser.add_argument("--camera-config", default="")
    args = parser.parse_args()

    config = load_simple_opencv_yaml(args.camera_config)
    stamps = {name: [] for name in TOPICS}
    lock = threading.Lock()

    def callback(message, name):
        stamp = message.header.stamp.to_sec()
        with lock:
            stamps[name].append(stamp)

    rospy.init_node("sensor_time_alignment_report", anonymous=True,
                    disable_signals=True)
    subscribers = [
        rospy.Subscriber(topic, message_type, callback,
                         callback_args=name, queue_size=1000)
        for name, (topic, message_type) in TOPICS.items()
    ]

    deadline = time.monotonic() + max(0.5, args.duration)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(0.02)

    with lock:
        samples = {name: list(values) for name, values in stamps.items()}

    print("")
    print("================ SENSOR TIME ALIGNMENT (before FAST-LIVO) ================")
    print("[time-align] clock domain: MID360 packet timestamp")
    print("[time-align] camera map : t_cam = a * (PTS / PtsClockHz) + b_session")
    print("[time-align] source={}  PtsClockHz={}  fixed_a={}".format(
        config.get("TimestampSource", "N/A"),
        config.get("PtsClockHz", "N/A"),
        config.get("CameraClockScale", "N/A")))
    print("[time-align] anchor: locked={}  b_session_ns={}  elapsed_ms={}  valid_pairs={}".format(
        ros_param("/uvc_camera/session_anchor_locked"),
        ros_param("/uvc_camera/session_anchor_b_ns"),
        fmt(ros_param("/uvc_camera/session_anchor_elapsed_ms")),
        ros_param("/uvc_camera/session_anchor_valid_pairs")))
    print("[time-align] sampled {:.1f} s".format(args.duration))
    print("[time-align] sensor   samples   header_hz   duplicate   rollback   newest_header")

    missing = []
    rollback_total = 0
    for name in ("lidar", "camera", "imu"):
        values = samples[name]
        if not values:
            missing.append(name)
            print("[time-align] {:<7} {:>7}      NO_DATA".format(name, 0))
            continue
        hz, duplicate, rollback = sequence_stats(values)
        rollback_total += rollback
        print("[time-align] {:<7} {:>7d} {:>11.3f} {:>11d} {:>10d}   {:.9f}".format(
            name, len(values), hz, duplicate, rollback, values[-1]))

    if missing:
        print("[time-align] RESULT=INCOMPLETE missing={}".format(",".join(missing)))
        print("=========================================================================")
        return 2

    pairs = (
        ("camera-imu", "camera", "imu"),
        ("lidar-imu", "lidar", "imu"),
    )
    print("[time-align] nearest header delta (first sensor minus second sensor)")
    print("[time-align] pair            median_ms   abs_p95_ms   abs_max_ms   pairs")
    for label, reference, candidate in pairs:
        deltas = nearest_signed_ms(samples[reference], samples[candidate])
        absolute = [abs(value) for value in deltas]
        print("[time-align] {:<15} {:>10.3f} {:>12.3f} {:>12.3f} {:>7d}".format(
            label, median(deltas), percentile(absolute, 95.0),
            max(absolute), len(deltas)))

    newest = {name: values[-1] for name, values in samples.items()}
    current_spread = max(newest.values()) - min(newest.values())
    result = "PASS" if (rollback_total == 0 and
                        current_spread <= args.max_current_spread) else "WARN"
    print("[time-align] newest-triplet spread={:.3f} ms (clock-domain limit={:.1f} ms)".format(
        current_spread * 1000.0, args.max_current_spread * 1000.0))
    print("[time-align] RESULT={} same clock domain; detailed anchor residual is deferred to stop_slam.".format(result))
    print("=========================================================================")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (rospy.ROSException, rospy.ROSInterruptException) as exc:
        print("[time-align] RESULT=ERROR {}".format(exc))
        sys.exit(2)
