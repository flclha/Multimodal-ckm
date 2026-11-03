#!/usr/bin/env python3
"""Publish the calibrated Gemini root pose in the FAST-LIVO moving frame."""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def _finite_vector(name, value, length):
    """校验 YAML 中的平移/四元数，避免错误外参静默进入 TF 树。"""
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("{} must contain {} values".format(name, length))
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError("{} contains a non-finite value".format(name))
    return result


def main():
    rospy.init_node("gemini335l_extrinsic_tf")
    parent = rospy.get_param("~parent_frame", "aft_mapped")
    child = rospy.get_param("~child_frame", "gemini335l_link")
    translation = _finite_vector(
        "translation_m", rospy.get_param("~translation_m"), 3
    )
    quaternion = _finite_vector(
        "quaternion_xyzw", rospy.get_param("~quaternion_xyzw"), 4
    )

    norm = math.sqrt(sum(item * item for item in quaternion))
    if norm < 1.0e-12:
        raise ValueError("quaternion norm is zero")
    quaternion = [item / norm for item in quaternion]

    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transform)
    rospy.loginfo(
        "Gemini calibrated TF active: %s -> %s, t=[%.6f %.6f %.6f] m, "
        "q_xyzw=[%.8f %.8f %.8f %.8f]",
        parent,
        child,
        *translation,
        *quaternion
    )
    rospy.spin()


if __name__ == "__main__":
    main()

