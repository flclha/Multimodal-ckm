#!/usr/bin/env python3
"""Capture a FAST-Calib calibration sample.

Records a LiDAR rosbag and saves one camera frame as a JPG into FAST-Calib's
calib_data directory. Run this after roscore, LiDAR driver, and camera driver
are publishing topics.
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


COMMON_IMAGE_TOPICS = (
    "/camera/image_raw",
    "/camera/color/image_raw",
    "/usb_cam/image_raw",
    "/image_raw",
    "/mvs_camera/image_raw",
)


def expand(path):
    return os.path.abspath(os.path.expandvars(os.path.expanduser(path)))


def topic_exists(topic):
    try:
        published = dict(rospy.get_published_topics())
    except Exception:
        return False
    return topic in published


def choose_image_topic(requested):
    if requested:
        return requested
    for topic in COMMON_IMAGE_TOPICS:
        if topic_exists(topic):
            return topic
    return COMMON_IMAGE_TOPICS[0]


def save_image(topic, output_path, timeout, encoding):
    rospy.loginfo("Waiting for one image on %s", topic)
    msg = rospy.wait_for_message(topic, Image, timeout=timeout)
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
    except CvBridgeError as exc:
        raise RuntimeError("Failed to convert image from %s: %s" % (topic, exc))
    if not cv2.imwrite(output_path, image):
        raise RuntimeError("cv2.imwrite failed for %s" % output_path)
    rospy.loginfo("Saved image: %s", output_path)


def start_rosbag(topic, output_path):
    cmd = ["rosbag", "record", "-O", output_path, topic]
    rospy.loginfo("Starting rosbag: %s", " ".join(cmd))
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )


def stop_rosbag(proc, timeout=10.0):
    if proc.poll() is not None:
        return proc.returncode
    rospy.loginfo("Stopping rosbag...")
    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    try:
        return proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        rospy.logwarn("rosbag did not stop after SIGINT; terminating it")
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        return proc.wait(timeout=5.0)


def main():
    parser = argparse.ArgumentParser(description="Capture LiDAR bag + camera JPG for FAST-Calib")
    parser.add_argument("--lidar-topic", default="/livox/lidar", help="LiDAR topic to record into the bag")
    parser.add_argument("--image-topic", default=None, help="Camera image topic. If omitted, common topics are tried.")
    parser.add_argument("--duration", type=float, default=5.0, help="Seconds to record the LiDAR bag")
    parser.add_argument("--image-timeout", type=float, default=10.0, help="Seconds to wait for one camera image")
    parser.add_argument("--name", default=None, help="Output basename without extension. Default: timestamp")
    parser.add_argument("--output-dir", default="~/catkin_ws/src/FAST-Calib/calib_data", help="Directory for .bag and .jpg")
    parser.add_argument("--encoding", default="bgr8", help="cv_bridge desired encoding for JPG, usually bgr8")
    parser.add_argument("--no-bag", action="store_true", help="Only save the JPG")
    parser.add_argument("--no-image", action="store_true", help="Only record the bag")
    args = parser.parse_args()

    if args.no_bag and args.no_image:
        parser.error("--no-bag and --no-image cannot be used together")
    if args.duration <= 0 and not args.no_bag:
        parser.error("--duration must be positive")

    rospy.init_node("fast_calib_capture_sample", anonymous=True)

    out_dir = expand(args.output_dir)
    os.makedirs(out_dir, exist_ok=True)
    stem = args.name or datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_path = os.path.join(out_dir, stem + ".bag")
    jpg_path = os.path.join(out_dir, stem + ".jpg")
    image_topic = choose_image_topic(args.image_topic)

    rospy.loginfo("Output bag: %s", bag_path)
    rospy.loginfo("Output jpg: %s", jpg_path)
    rospy.loginfo("LiDAR topic: %s", args.lidar_topic)
    if not args.no_image:
        rospy.loginfo("Image topic: %s", image_topic)

    bag_proc = None
    bag_output = ""
    try:
        if not args.no_bag:
            bag_proc = start_rosbag(args.lidar_topic, bag_path)
            time.sleep(0.8)
            if bag_proc.poll() is not None:
                bag_output = bag_proc.stdout.read() if bag_proc.stdout else ""
                raise RuntimeError("rosbag exited early:\n%s" % bag_output)

        start = time.time()
        if not args.no_image:
            save_image(image_topic, jpg_path, args.image_timeout, args.encoding)

        if bag_proc is not None:
            remaining = args.duration - (time.time() - start)
            if remaining > 0:
                rospy.loginfo("Recording LiDAR for %.1f more seconds", remaining)
                time.sleep(remaining)
            code = stop_rosbag(bag_proc)
            bag_output = bag_proc.stdout.read() if bag_proc.stdout else ""
            if code not in (0, -signal.SIGINT):
                raise RuntimeError("rosbag returned code %s:\n%s" % (code, bag_output))
            rospy.loginfo("Saved bag: %s", bag_path)

    except Exception as exc:
        if bag_proc is not None and bag_proc.poll() is None:
            stop_rosbag(bag_proc)
        rospy.logerr(str(exc))
        return 1

    print("\nCaptured FAST-Calib sample:")
    if not args.no_bag:
        print("  bag: %s" % bag_path)
    if not args.no_image:
        print("  jpg: %s" % jpg_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
