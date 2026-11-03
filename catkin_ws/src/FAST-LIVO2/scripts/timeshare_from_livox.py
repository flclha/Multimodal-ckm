#!/usr/bin/env python3
import argparse
import mmap
import os
import struct
import time

import rospy
import rostopic


class TimeShareWriter:
    def __init__(self, path, topic):
        self.path = os.path.expanduser(path)
        self.topic = topic
        self.seq = 0
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o666)
        os.ftruncate(self.fd, 16)
        self.mm = mmap.mmap(self.fd, 16, mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ)
        msg_class, real_topic, _ = rostopic.get_topic_class(self.topic, blocking=True)
        if msg_class is None:
            raise RuntimeError("Cannot determine message type for {}".format(self.topic))
        rospy.Subscriber(real_topic, msg_class, self.callback, queue_size=20)
        rospy.loginfo(
            "Writing /home/<user>/timeshare from %s header.stamp (%s) to %s",
            real_topic,
            msg_class.__name__,
            self.path,
        )

    def callback(self, msg):
        stamp = msg.header.stamp
        if stamp.is_zero():
            rospy.logwarn_throttle(1.0, "%s has zero header.stamp; not writing timeshare", self.topic)
            return

        self.seq += 1
        timestamp_ns = stamp.secs * 1000000000 + stamp.nsecs
        self.mm.seek(0)
        self.mm.write(struct.pack("qq", self.seq, timestamp_ns))
        self.mm.flush()
        if self.seq <= 10 or self.seq % 100 == 0:
            rospy.loginfo(
                "timeshare update seq=%d low=%d stamp=%.9f",
                self.seq,
                timestamp_ns,
                stamp.to_sec(),
            )

    def close(self):
        self.mm.close()
        os.close(self.fd)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Write the FAST-LIVO2 timeshare file from a synchronized LiDAR ROS header stamp. "
            "This provides the existing shared timebase; true camera shutter timestamps still "
            "require per-trigger timestamps from the STM32 or camera hardware."
        )
    )
    parser.add_argument("--path", default="/home/nuc/timeshare")
    parser.add_argument("--topic", default="/livox/lidar")
    args, _ = parser.parse_known_args()

    rospy.init_node("timeshare_from_livox")
    writer = TimeShareWriter(args.path, args.topic)
    try:
        rospy.spin()
    finally:
        writer.close()


if __name__ == "__main__":
    main()
