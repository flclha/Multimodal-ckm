#!/usr/bin/env python3
import argparse
import socket
import struct
import sys

import rospy
from sensor_msgs.msg import Imu

HEADER_LEN = 36
G = 9.80665
TIMESTAMP_TYPE_GPTP_OR_PTP = 1
TIMESTAMP_TYPE_GPS = 2


def parse_imu_packet(data):
    if len(data) < HEADER_LEN + 24:
        return None
    packet_len = struct.unpack_from('<H', data, 1)[0]
    if packet_len and packet_len != len(data):
        return None
    time_type = data[11]
    timestamp_ns = struct.unpack_from('<Q', data, 28)[0]
    vals = struct.unpack_from('<ffffff', data, HEADER_LEN)
    gx, gy, gz, ax_g, ay_g, az_g = vals
    return gx, gy, gz, ax_g * G, ay_g * G, az_g * G, time_type, timestamp_ns


def packet_timestamp_is_synced(time_type, timestamp_ns):
    return timestamp_ns > 0 and time_type in (
        TIMESTAMP_TYPE_GPTP_OR_PTP, TIMESTAMP_TYPE_GPS)


def main():
    parser = argparse.ArgumentParser(description='Publish MID360 UDP IMU as sensor_msgs/Imu')
    parser.add_argument('--bind-ip', default='192.168.1.5')
    parser.add_argument('--port', type=int, default=56401)
    parser.add_argument('--topic', default='/livox/imu')
    parser.add_argument('--frame-id', default='livox_frame')
    args = parser.parse_args()

    rospy.init_node('mid360_udp_imu_republisher', anonymous=False)
    pub = rospy.Publisher(args.topic, Imu, queue_size=200)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_ip, args.port))
    sock.settimeout(0.2)
    rospy.loginfo('Listening for MID360 IMU UDP on %s:%d, publishing %s', args.bind_ip, args.port, args.topic)

    count = 0
    while not rospy.is_shutdown():
        try:
            data, _addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        parsed = parse_imu_packet(data)
        if parsed is None:
            continue
        gx, gy, gz, ax, ay, az, time_type, timestamp_ns = parsed
        msg = Imu()
        if packet_timestamp_is_synced(time_type, timestamp_ns):
            msg.header.stamp = rospy.Time(
                timestamp_ns // 1000000000, timestamp_ns % 1000000000)
            stamp_source = 'LIDAR_PACKET_TIMESTAMP'
        else:
            msg.header.stamp = rospy.Time.now()
            stamp_source = 'ROS_TIME_FALLBACK'
        msg.header.frame_id = args.frame_id
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1
        pub.publish(msg)
        count += 1
        if count <= 20 or count % 500 == 0:
            rospy.loginfo(
                'IMU timestamp source=%s packets=%d time_type=%d stamp=%.9f',
                stamp_source, count, time_type, msg.header.stamp.to_sec())
    return 0


if __name__ == '__main__':
    sys.exit(main())
