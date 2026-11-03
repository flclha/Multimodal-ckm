#!/usr/bin/env python3
"""Republish MID360 UDP point packets as livox_ros_driver2/CustomMsg.

This helper is for calibration capture when the MID360 is visibly sending UDP
point packets, but livox_ros_driver2 does not expose /livox/lidar in ROS1.
It parses MID360 Cartesian high-resolution packets sent to host port 56301.
"""

import argparse
import mmap
import os
import socket
import struct
import sys
import time

import rospy
from livox_ros_driver2.msg import CustomMsg, CustomPoint

HEADER_LEN = 36
POINT_SIZE = 14
MID360_LINES = 4
DATA_TYPE_CARTESIAN_HIGH = 1
TIMESTAMP_TYPE_NO_SYNC = 0
TIMESTAMP_TYPE_GPTP_OR_PTP = 1
TIMESTAMP_TYPE_GPS = 2
CLOCK_OFFSET_WINDOW_PACKETS = 100


class TimeShareWriter:
    """Publish the same LiDAR frame timestamp consumed by the camera driver."""

    def __init__(self, path):
        self.path = os.path.expanduser(path)
        self.seq = 0
        self.closed = False
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.fd = os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o666)
        os.ftruncate(self.fd, 16)
        self.mm = mmap.mmap(
            self.fd, 16, mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ)

    def write(self, timestamp_ns):
        # Seqlock layout: odd sequence while updating, even when committed.
        # The camera reader rejects odd or changing sequences, so it cannot
        # combine a new sequence with an old timestamp (or vice versa).
        self.seq += 2
        struct.pack_into('q', self.mm, 0, self.seq - 1)
        struct.pack_into('q', self.mm, 8, int(timestamp_ns))
        struct.pack_into('q', self.mm, 0, self.seq)
        self.mm.flush()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.mm.close()
        os.close(self.fd)


class ClockOffsetWriter(TimeShareWriter):
    """Publish host realtime minus MID360 hardware time."""

    def __init__(self, path):
        super().__init__(path)
        self.window_min_ns = None
        self.window_count = 0

    def observe(self, host_receive_ns, lidar_timestamp_ns):
        # The minimum over a short window rejects most positive UDP latency.
        candidate_ns = int(host_receive_ns) - int(lidar_timestamp_ns)
        if self.window_min_ns is None or candidate_ns < self.window_min_ns:
            self.window_min_ns = candidate_ns
        self.window_count += 1
        if self.seq == 0 or self.window_count >= CLOCK_OFFSET_WINDOW_PACKETS:
            self.write(self.window_min_ns)
            self.window_min_ns = None
            self.window_count = 0


def parse_packet(data):
    if len(data) < HEADER_LEN:
        return None

    packet_len = struct.unpack_from('<H', data, 1)[0]
    time_interval = struct.unpack_from('<H', data, 3)[0]
    dot_num = struct.unpack_from('<H', data, 5)[0]
    data_type = data[10]
    time_type = data[11]
    lidar_timestamp_ns = struct.unpack_from('<Q', data, 28)[0]

    if data_type != DATA_TYPE_CARTESIAN_HIGH or dot_num == 0:
        return None

    need = HEADER_LEN + dot_num * POINT_SIZE
    if len(data) < need or packet_len != len(data):
        return None

    # Livox SDK2 encodes high-resolution Cartesian points as millimetres.
    point_interval_ns = int((time_interval * 100) / dot_num) if dot_num else 0
    points = []
    pos = HEADER_LEN
    for i in range(dot_num):
        x_mm, y_mm, z_mm, reflectivity, tag = struct.unpack_from('<iiiBB', data, pos)
        pos += POINT_SIZE

        pt = CustomPoint()
        pt.offset_time = i * point_interval_ns
        pt.x = x_mm / 1000.0
        pt.y = y_mm / 1000.0
        pt.z = z_mm / 1000.0
        pt.reflectivity = reflectivity
        pt.tag = tag
        pt.line = i % MID360_LINES
        points.append(pt)

    return points, time_type, lidar_timestamp_ns


def packet_timestamp_is_synced(time_type, timestamp_ns):
    return (
        timestamp_ns > 0
        and time_type in (TIMESTAMP_TYPE_GPTP_OR_PTP, TIMESTAMP_TYPE_GPS)
    )


def main():
    parser = argparse.ArgumentParser(description='Publish /livox/lidar from MID360 UDP packets')
    parser.add_argument('--bind-ip', default='192.168.1.5')
    parser.add_argument('--port', type=int, default=56301)
    parser.add_argument('--topic', default='/livox/lidar')
    parser.add_argument('--frame-id', default='livox_frame')
    parser.add_argument('--publish-hz', type=float, default=10.0)
    parser.add_argument('--lidar-id', type=int, default=0)
    parser.add_argument(
        '--timeshare-path',
        default='',
        help='Legacy shared LiDAR-frame timestamp (diagnostics only)')
    parser.add_argument(
        '--clock-offset-path',
        default='/home/nuc/mid360_clock_offset',
        help='Shared host-realtime minus MID360-time estimate for camera stamps')
    args = parser.parse_args()

    rospy.init_node('mid360_udp_livox_republisher', anonymous=False)
    pub = rospy.Publisher(args.topic, CustomMsg, queue_size=16)
    timeshare = TimeShareWriter(args.timeshare_path) if args.timeshare_path else None
    clock_offset = ClockOffsetWriter(args.clock_offset_path) if args.clock_offset_path else None
    if timeshare:
        rospy.on_shutdown(timeshare.close)
    if clock_offset:
        rospy.on_shutdown(clock_offset.close)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_ip, args.port))
    sock.settimeout(0.2)

    rospy.loginfo(
        'Listening for MID360 UDP on %s:%d, publishing %s, timeshare=%s clock_offset=%s',
        args.bind_ip, args.port, args.topic,
        timeshare.path if timeshare else 'disabled',
        clock_offset.path if clock_offset else 'disabled')
    period = 1.0 / args.publish_hz if args.publish_hz > 0 else 0.1
    frame_period_ns = max(1, int(round(period * 1e9)))
    msg_points = []
    packet_count = 0
    sync_timestamp_count = 0
    fallback_timestamp_count = 0
    batch_raw_base_ns = 0
    batch_time_type = TIMESTAMP_TYPE_NO_SYNC
    batch_started_ns = 0
    batch_stamp_source = 'ROS_TIME_FALLBACK'
    batch_bucket = None
    discard_first_bucket = True
    previous_bucket = None

    while not rospy.is_shutdown():
        try:
            data, _addr = sock.recvfrom(2048)
        except socket.timeout:
            continue

        parsed = parse_packet(data)
        if not parsed:
            continue
        points, time_type, lidar_timestamp_ns = parsed
        host_receive_ns = rospy.Time.now().to_nsec()
        if clock_offset and packet_timestamp_is_synced(time_type, lidar_timestamp_ns):
            clock_offset.observe(host_receive_ns, lidar_timestamp_ns)

        packet_count += 1
        if packet_timestamp_is_synced(time_type, lidar_timestamp_ns):
            packet_bucket = int(lidar_timestamp_ns) // frame_period_ns
        else:
            packet_bucket = int(time.monotonic() / period)

        if batch_bucket is None or packet_bucket != batch_bucket:
            if batch_bucket is not None and msg_points:
                if discard_first_bucket:
                    rospy.loginfo(
                        'Discarded initial partial LiDAR bucket=%d points=%d',
                        batch_bucket, len(msg_points))
                    discard_first_bucket = False
                else:
                    # Preserve the native MID360 sampling clock. Camera and IMU
                    # are independently expressed in this same clock domain.
                    stamp_ns = int(batch_raw_base_ns or batch_started_ns)
                    stamp = rospy.Time(
                        stamp_ns // 1000000000, stamp_ns % 1000000000)
                    msg = CustomMsg()
                    msg.header.stamp = stamp
                    msg.header.frame_id = args.frame_id
                    msg.timebase = stamp_ns
                    msg.point_num = len(msg_points)
                    msg.lidar_id = args.lidar_id & 0xFF
                    msg.rsvd = [0, 0, 0]
                    msg.points = msg_points
                    if timeshare:
                        timeshare.write(stamp_ns)
                    pub.publish(msg)

                    if packet_timestamp_is_synced(batch_time_type, batch_raw_base_ns):
                        sync_timestamp_count += 1
                    else:
                        fallback_timestamp_count += 1
                    total_frames = sync_timestamp_count + fallback_timestamp_count
                    if total_frames <= 20 or total_frames % 100 == 0:
                        rospy.loginfo(
                            'LiDAR timestamp source=%s frames=%d sync=%d fallback=%d '
                            'time_type=%d stamp_ns=%d points=%d packets=%d bucket=%d',
                            batch_stamp_source, total_frames, sync_timestamp_count,
                            fallback_timestamp_count, batch_time_type, stamp_ns,
                            msg.point_num, packet_count, batch_bucket)

            if previous_bucket is not None and packet_bucket != previous_bucket + 1:
                rospy.logwarn(
                    'LiDAR hardware-time bucket jump: previous=%d current=%d delta=%d',
                    previous_bucket, packet_bucket, packet_bucket - previous_bucket)
            previous_bucket = packet_bucket
            batch_bucket = packet_bucket
            msg_points = []
            batch_started_mono = time.monotonic()
            batch_started_ns = host_receive_ns
            batch_time_type = time_type
            if packet_timestamp_is_synced(time_type, lidar_timestamp_ns):
                batch_raw_base_ns = int(lidar_timestamp_ns)
                batch_stamp_source = 'LIDAR_PACKET_TIMESTAMP'
            else:
                batch_raw_base_ns = 0
                batch_stamp_source = 'ROS_TIME_FALLBACK'

        if (
            packet_timestamp_is_synced(time_type, lidar_timestamp_ns)
            and packet_timestamp_is_synced(batch_time_type, batch_raw_base_ns)
        ):
            elapsed_ns = int(lidar_timestamp_ns) - int(batch_raw_base_ns)
            if elapsed_ns < 0:
                elapsed_ns = 0
        else:
            elapsed_ns = int((time.monotonic() - batch_started_mono) * 1e9)
        for p in points:
            p.offset_time = (elapsed_ns + int(p.offset_time)) & 0xFFFFFFFF
        msg_points.extend(points)

    return 0


if __name__ == '__main__':
    sys.exit(main())
