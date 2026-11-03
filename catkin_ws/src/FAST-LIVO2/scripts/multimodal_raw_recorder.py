#!/usr/bin/env python3
"""Record frame LiDAR, continuous IMU and FAST-LIVO pose without blocking SLAM."""

import csv
import json
import math
import os
import queue
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField


def _stamp_ns(header):
    value = int(header.stamp.to_nsec())
    return value if value else time.time_ns()


def _rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(siny, cosy)


class RawRecorder:
    def __init__(self):
        self.root = os.path.abspath(rospy.get_param("~output_root"))
        self.lidar_dir = os.path.join(self.root, "lidar")
        self.imu_dir = os.path.join(self.root, "imu")
        self.pose_dir = os.path.join(self.root, "pos")
        for path in (self.lidar_dir, self.imu_dir, self.pose_dir):
            os.makedirs(path, exist_ok=True)
        self.queue = queue.Queue(maxsize=max(8, int(rospy.get_param("~lidar_queue_size", 64))))
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.sync_lock = threading.Lock()
        self.last_csv_sync = 0.0
        self.count = {"lidar_received": 0, "lidar_saved": 0, "lidar_dropped": 0,
                      "lidar_failed": 0, "imu": 0, "pose": 0}
        self.lidar_csv_file, self.lidar_csv = self._csv(
            os.path.join(self.lidar_dir, "frames.csv"),
            ["filename", "header_stamp_ns", "receive_time_ns", "seq", "frame_id",
             "topic", "processing", "width", "height", "point_count", "fields", "point_step",
             "row_step", "is_bigendian", "is_dense"],
        )
        self.imu_file, self.imu_csv = self._csv(
            os.path.join(self.imu_dir, "imu.csv"),
            ["time_ns", "imu_stamp_ns", "seq", "frame_id", "ax_m_s2", "ay_m_s2", "az_m_s2",
             "gx_rad_s", "gy_rad_s", "gz_rad_s", "qx", "qy", "qz", "qw"],
        )
        self.pose_file, self.pose_csv = self._csv(
            os.path.join(self.pose_dir, "poses.csv"),
            ["time_ns", "pose_stamp_ns", "seq", "frame_id", "child_frame_id", "x_m", "y_m", "z_m",
             "qx", "qy", "qz", "qw", "roll_rad", "pitch_rad", "yaw_rad",
             "vx_m_s", "vy_m_s", "vz_m_s", "wx_rad_s", "wy_rad_s", "wz_rad_s"],
        )
        self.worker = threading.Thread(target=self._writer_loop, name="pcd_writer", daemon=True)
        self.worker.start()
        self.subscribers = [
            rospy.Subscriber("/cloud_registered_raw", PointCloud2, self._lidar_cb,
                             queue_size=2, buff_size=32 * 1024 * 1024),
            rospy.Subscriber("/livox/imu", Imu, self._imu_cb, queue_size=1000),
            rospy.Subscriber("/aft_mapped_to_init", Odometry, self._pose_cb, queue_size=100),
        ]
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "Dataset recorder started: per-frame LiDAR source=/cloud_registered_raw "
            "(deskewed, globally registered, uncolored), output=%s", self.root)

    def _sync_csv_if_due(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_csv_sync < 1.0:
            return
        with self.sync_lock:
            now = time.monotonic()
            if not force and now - self.last_csv_sync < 1.0:
                return
            for handle in (self.lidar_csv_file, self.imu_file, self.pose_file):
                handle.flush()
                os.fsync(handle.fileno())
            self.last_csv_sync = now

    @staticmethod
    def _csv(path, header):
        handle = open(path, "w", newline="", buffering=1, encoding="utf-8")
        writer = csv.writer(handle)
        writer.writerow(header)
        return handle, writer

    def _lidar_cb(self, msg):
        receive_ns = time.time_ns()
        with self.lock:
            self.count["lidar_received"] += 1
        try:
            self.queue.put_nowait((msg, receive_ns))
        except queue.Full:
            with self.lock:
                self.count["lidar_dropped"] += 1
            rospy.logwarn_throttle(5.0, "LiDAR PCD queue full; dropping newest frame")

    def _imu_cb(self, msg):
        receive_ns = time.time_ns()
        self.imu_csv.writerow([
            receive_ns, _stamp_ns(msg.header), int(msg.header.seq), msg.header.frame_id,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
        ])
        with self.lock:
            self.count["imu"] += 1
        self._sync_csv_if_due()

    def _pose_cb(self, msg):
        receive_ns = time.time_ns()
        pose, twist = msg.pose.pose, msg.twist.twist
        roll, pitch, yaw = _rpy(pose.orientation)
        self.pose_csv.writerow([
            receive_ns, _stamp_ns(msg.header), int(msg.header.seq), msg.header.frame_id, msg.child_frame_id,
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
            roll, pitch, yaw, twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        ])
        with self.lock:
            self.count["pose"] += 1
        self._sync_csv_if_due()

    @staticmethod
    def _pcd_payload(msg):
        # /cloud_registered_raw is generated from pcl_w_wait_pub in LIVMapper:
        # every point has already been motion-deskewed and transformed into the
        # camera_init global frame, while no RGB/FOV/visibility filter is used.
        fields = {field.name: field for field in msg.fields}
        required = ("x", "y", "z", "intensity")
        missing = [name for name in required if name not in fields]
        if missing:
            raise ValueError("PointCloud2 missing fields: {}".format(",".join(missing)))
        if any(fields[name].datatype != PointField.FLOAT32 for name in required):
            raise ValueError("x/y/z/intensity must be FLOAT32")

        byte_order = ">" if msg.is_bigendian else "<"
        view_dtype = np.dtype({
            "names": list(required),
            "formats": [byte_order + "f4"] * len(required),
            "offsets": [int(fields[name].offset) for name in required],
            "itemsize": int(msg.point_step),
        })
        rows = [
            np.frombuffer(msg.data, dtype=view_dtype, count=int(msg.width),
                          offset=row * int(msg.row_step))
            for row in range(int(msg.height))
        ]
        source = np.concatenate(rows) if len(rows) > 1 else rows[0]
        packed = np.empty(source.size, dtype=np.dtype([
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")
        ]))
        for name in required:
            packed[name] = source[name]
        finite = np.isfinite(packed["x"]) & np.isfinite(packed["y"]) & np.isfinite(packed["z"])
        if not np.all(finite):
            packed = packed[finite]
        point_count = int(packed.size)
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
            "FIELDS {}\nSIZE {}\nTYPE {}\nCOUNT {}\nWIDTH {}\nHEIGHT {}\n"
            "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {}\nDATA binary\n"
        ).format(
            "x y z intensity",
            "4 4 4 4", "F F F F", "1 1 1 1",
            point_count, 1, point_count,
        )
        return header.encode("ascii"), packed.tobytes(order="C"), point_count

    def _writer_loop(self):
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                msg, receive_ns = self.queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                header, payload, point_count = self._pcd_payload(msg)
                with self.lock:
                    frame_number = self.count["lidar_saved"] + 1
                filename = "{:06d}.pcd".format(frame_number)
                destination = os.path.join(self.lidar_dir, filename)
                temporary = destination + ".tmp"
                with open(temporary, "wb") as stream:
                    stream.write(header); stream.write(payload)
                    stream.flush(); os.fsync(stream.fileno())
                os.replace(temporary, destination)
                directory_fd = os.open(self.lidar_dir, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
                self.lidar_csv.writerow([
                    filename, _stamp_ns(msg.header), receive_ns, int(msg.header.seq), msg.header.frame_id,
                    "/cloud_registered_raw", "deskewed+registered+uncolored",
                    point_count, 1, point_count,
                    "x y z intensity", 16, point_count * 16, 0, int(msg.is_dense),
                ])
                with self.lock:
                    self.count["lidar_saved"] += 1
                self._sync_csv_if_due()
            except Exception as exc:
                with self.lock:
                    self.count["lidar_failed"] += 1
                rospy.logerr_throttle(5.0, "Failed to save LiDAR PCD: %s", exc)
            finally:
                self.queue.task_done()

    def shutdown(self):
        if self.stop_event.is_set():
            return
        self.stop_event.set()
        self.worker.join(timeout=120.0)
        self._sync_csv_if_due(force=True)
        for handle in (self.lidar_csv_file, self.imu_file, self.pose_file):
            handle.flush(); handle.close()
        with open(os.path.join(self.root, "raw_record_summary.json"), "w", encoding="utf-8") as stream:
            json.dump(self.count, stream, indent=2, sort_keys=True)
        rospy.loginfo("Raw dataset recorder stopped: %s", self.count)


if __name__ == "__main__":
    rospy.init_node("multimodal_raw_recorder")
    recorder = RawRecorder()
    rospy.spin()
