#!/usr/bin/env python3
import argparse
import socket
import struct
import sys

import rospy
from sensor_msgs.msg import Imu

HEADER_LEN = 36
G = 9.80665


def parse_imu_packet(data):
    if len(data) < HEADER_LEN + 24:
        return None
    packet_len = struct.unpack_from('<H', data, 1)[0]
    if packet_len and packet_len != len(data):
        return None
    vals = struct.unpack_from('<ffffff', data, HEADER_LEN)
    gx, gy, gz, ax_g, ay_g, az_g = vals
    return gx, gy, gz, ax_g * G, ay_g * G, az_g * G


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
        gx, gy, gz, ax, ay, az = parsed
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
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
        if count % 500 == 0:
            rospy.loginfo('Published %d IMU packets', count)
    return 0


if __name__ == '__main__':
    sys.exit(main())
