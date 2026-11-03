#!/usr/bin/env python3
import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def normalize_quaternion(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 0.0:
        return [0.0, 0.0, 0.0, 1.0]
    return [v / norm for v in q]


def msg_to_quaternion(msg):
    return normalize_quaternion([msg.x, msg.y, msg.z, msg.w])


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def quaternion_inverse(q):
    x, y, z, w = normalize_quaternion(q)
    return [-x, -y, -z, w]


def rotate_vector(q, v):
    qv = [v[0], v[1], v[2], 0.0]
    return quaternion_multiply(quaternion_multiply(q, qv), quaternion_inverse(q))[:3]


def yaw_quaternion(yaw):
    half_yaw = 0.5 * yaw
    return [0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)]


def level_heading_quaternion(q, heading_axis, yaw_offset):
    heading = rotate_vector(q, heading_axis)
    heading[2] = 0.0
    norm = math.hypot(heading[0], heading[1])
    if norm <= 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    yaw = math.atan2(heading[1], heading[0]) + yaw_offset
    return yaw_quaternion(yaw)


class Mid360ToCarTf:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/aft_mapped_to_init")
        self.mid360_frame = rospy.get_param("~mid360_frame", "aft_mapped")
        self.car_frame = rospy.get_param("~car_frame", "tracer_base")
        self.heading_axis = rospy.get_param("~heading_axis", [1.0, 0.0, 0.0])
        if len(self.heading_axis) != 3:
            raise ValueError("~heading_axis must contain exactly 3 values")
        axis_norm = math.sqrt(sum(v * v for v in self.heading_axis))
        if axis_norm <= 0.0:
            raise ValueError("~heading_axis must be non-zero")
        self.heading_axis = [v / axis_norm for v in self.heading_axis]
        self.yaw_offset = math.radians(rospy.get_param("~yaw_offset_deg", 0.0))
        self.use_initial_mount_calibration = rospy.get_param(
            "~use_initial_mount_calibration", True
        )

        self.mount_inv = [0.0, 0.0, 0.0, 1.0]
        self.calibrated = not self.use_initial_mount_calibration
        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=20)

        rospy.loginfo(
            "Publishing MID360-to-car TF from %s to %s using %s; heading_axis=%s, yaw_offset=%.3f deg",
            self.mid360_frame,
            self.car_frame,
            self.odom_topic,
            self.heading_axis,
            math.degrees(self.yaw_offset),
        )

    def odom_cb(self, msg):
        q_mid360 = msg_to_quaternion(msg.pose.pose.orientation)

        if not self.calibrated:
            # At startup the car is assumed level. The first MID360 roll/pitch is
            # therefore treated as the fixed sensor mounting tilt.
            q_car_initial = level_heading_quaternion(
                q_mid360, self.heading_axis, self.yaw_offset
            )
            q_mount = normalize_quaternion(
                quaternion_multiply(quaternion_inverse(q_car_initial), q_mid360)
            )
            self.mount_inv = normalize_quaternion(quaternion_inverse(q_mount))
            self.calibrated = True
            rospy.loginfo("Estimated fixed MID360 mounting tilt from first odometry pose")

        q_car_est = normalize_quaternion(quaternion_multiply(q_mid360, self.mount_inv))
        q_car_world = level_heading_quaternion(
            q_car_est, self.heading_axis, self.yaw_offset
        )
        q_mid360_to_car = normalize_quaternion(
            quaternion_multiply(quaternion_inverse(q_mid360), q_car_world)
        )

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        tf_msg.header.frame_id = self.mid360_frame or msg.child_frame_id or "aft_mapped"
        tf_msg.child_frame_id = self.car_frame
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = q_mid360_to_car[0]
        tf_msg.transform.rotation.y = q_mid360_to_car[1]
        tf_msg.transform.rotation.z = q_mid360_to_car[2]
        tf_msg.transform.rotation.w = q_mid360_to_car[3]
        self.br.sendTransform(tf_msg)


if __name__ == "__main__":
    rospy.init_node("mid360_to_car_tf")
    Mid360ToCarTf()
    rospy.spin()
