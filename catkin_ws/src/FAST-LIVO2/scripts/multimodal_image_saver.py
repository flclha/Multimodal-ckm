#!/usr/bin/env python3
"""Asynchronously save main RGB plus Gemini IR/depth images with timestamps."""

import csv
import json
import os
import queue
import threading
import time

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import CameraInfo, Image


class MultimodalImageSaver:
    def __init__(self):
        self.output_root = os.path.abspath(rospy.get_param("~output_root"))
        self.save_hz = max(0.0, float(rospy.get_param("~save_hz", 10.0)))
        self.rgb_save_hz = max(0.0, float(rospy.get_param("~rgb_save_hz", self.save_hz)))
        self.png_compression = int(rospy.get_param("~png_compression", 1))
        self.queue_size = max(8, int(rospy.get_param("~queue_size", 256)))
        camera_name = rospy.get_param("~camera_name", "gemini335l").strip("/")
        self.streams = {
            "rgb": "/uvc_camera/image_sync",
            "ir_left": "/{}/left_ir/image_raw".format(camera_name),
            "ir_right": "/{}/right_ir/image_raw".format(camera_name),
            "depth": "/{}/depth/image_raw".format(camera_name),
        }
        self.min_interval_ns = {}
        for name in self.streams:
            rate = self.rgb_save_hz if name == "rgb" else self.save_hz
            # Nominal 10 Hz hardware is measured at about 10.05 Hz (99.44 ms).
            # A strict 100 ms gate would therefore save every second frame.
            # The 10% tolerance preserves one frame per nominal period while
            # still reducing Gemini's 30 Hz stream to approximately 10 Hz.
            self.min_interval_ns[name] = int(0.90e9 / rate) if rate > 0 else 0

        self.work_queue = queue.Queue(maxsize=self.queue_size)
        self.stop_event = threading.Event()
        self.state_lock = threading.Lock()
        self.last_selected_ns = {name: 0 for name in self.streams}
        self.received = {name: 0 for name in self.streams}
        self.selected = {name: 0 for name in self.streams}
        self.saved = {name: 0 for name in self.streams}
        self.dropped = {name: 0 for name in self.streams}
        self.failed = {name: 0 for name in self.streams}
        self.first_stamp_ns = {name: None for name in self.streams}
        self.last_stamp_ns = {name: None for name in self.streams}
        self.csv_files = {}
        self.csv_writers = {}
        self.camera_info = {}
        self.last_disk_sync = 0.0

        os.makedirs(self.output_root, exist_ok=True)
        for name in self.streams:
            stream_dir = os.path.join(self.output_root, name)
            os.makedirs(stream_dir, exist_ok=True)
            csv_file = open(
                os.path.join(stream_dir, "frames.csv"), "w", newline="", buffering=1, encoding="utf-8"
            )
            writer = csv.writer(csv_file)
            writer.writerow(
                ["filename", "header_stamp_ns", "receive_time_ns", "seq", "frame_id",
                 "topic", "encoding", "width", "height", "step"]
            )
            self.csv_files[name] = csv_file
            self.csv_writers[name] = writer

        self.worker = threading.Thread(target=self._writer_loop, name="image_writer", daemon=True)
        self.worker.start()
        self.subscribers = [
            rospy.Subscriber(topic, Image, self._callback, callback_args=name,
                             queue_size=2, buff_size=32 * 1024 * 1024)
            for name, topic in self.streams.items()
        ]
        info_topics = {
            "gemini_rgb": "/{}/color/camera_info".format(camera_name),
            "gemini_depth": "/{}/depth/camera_info".format(camera_name),
            "gemini_ir_left": "/{}/left_ir/camera_info".format(camera_name),
            "gemini_ir_right": "/{}/right_ir/camera_info".format(camera_name),
        }
        self.info_subscribers = [
            rospy.Subscriber(topic, CameraInfo, self._camera_info_callback,
                             callback_args=name, queue_size=1)
            for name, topic in info_topics.items()
        ]
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "Multimodal image saver: output=%s rgb_hz=%.3f gemini_hz=%.3f queue=%d topics=%s",
            self.output_root, self.rgb_save_hz, self.save_hz, self.queue_size, self.streams,
        )

    def _sync_if_due(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_disk_sync < 1.0:
            return
        for handle in self.csv_files.values():
            handle.flush()
            os.fsync(handle.fileno())
        for name in self.streams:
            stream_dir = os.path.join(self.output_root, name)
            directory_fd = os.open(stream_dir, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        self.last_disk_sync = now

    def _callback(self, message, stream_name):
        receive_ns = time.time_ns()
        stamp_ns = int(message.header.stamp.to_nsec()) or receive_ns
        with self.state_lock:
            self.received[stream_name] += 1
            previous = self.last_selected_ns[stream_name]
            if previous and stamp_ns >= previous and stamp_ns - previous < self.min_interval_ns[stream_name]:
                return
            if stamp_ns < previous:
                self.last_selected_ns[stream_name] = 0
            self.last_selected_ns[stream_name] = stamp_ns
            self.selected[stream_name] += 1
        try:
            self.work_queue.put_nowait((stream_name, message, stamp_ns, receive_ns))
        except queue.Full:
            with self.state_lock:
                self.dropped[stream_name] += 1
            rospy.logwarn_throttle(5.0, "Image saver queue full; dropping newest frame")

    def _camera_info_callback(self, message, name):
        with self.state_lock:
            if name in self.camera_info:
                return
            self.camera_info[name] = {
                "frame_id": message.header.frame_id,
                "width": int(message.width), "height": int(message.height),
                "distortion_model": message.distortion_model,
                "D": list(message.D), "K": list(message.K), "R": list(message.R), "P": list(message.P),
                "binning_x": int(message.binning_x), "binning_y": int(message.binning_y),
                "roi": {"x_offset": int(message.roi.x_offset), "y_offset": int(message.roi.y_offset),
                        "height": int(message.roi.height), "width": int(message.roi.width),
                        "do_rectify": bool(message.roi.do_rectify)},
            }

    @staticmethod
    def _to_numpy(message):
        encoding = message.encoding.lower()
        layout = {
            "mono8": (np.uint8, 1), "8uc1": (np.uint8, 1),
            "mono16": (np.uint16, 1), "16uc1": (np.uint16, 1), "16sc1": (np.int16, 1),
            "bgr8": (np.uint8, 3), "rgb8": (np.uint8, 3),
            "bgra8": (np.uint8, 4), "rgba8": (np.uint8, 4),
        }
        if encoding not in layout:
            raise ValueError("unsupported image encoding: {}".format(message.encoding))
        dtype, channels = layout[encoding]
        wire_dtype = np.dtype(dtype).newbyteorder(">" if message.is_bigendian else "<")
        row_elements = message.step // wire_dtype.itemsize
        array = np.frombuffer(message.data, dtype=wire_dtype).reshape(message.height, row_elements)
        array = array[:, : message.width * channels]
        if channels > 1:
            array = array.reshape(message.height, message.width, channels)
        array = np.ascontiguousarray(array.astype(dtype, copy=False))
        if encoding == "rgb8":
            array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        elif encoding == "rgba8":
            array = cv2.cvtColor(array, cv2.COLOR_RGBA2BGRA)
        return array

    def _writer_loop(self):
        while not self.stop_event.is_set() or not self.work_queue.empty():
            try:
                stream_name, message, stamp_ns, receive_ns = self.work_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                image = self._to_numpy(message)
                with self.state_lock:
                    frame_number = self.saved[stream_name] + 1
                filename = "{:06d}.png".format(frame_number)
                destination = os.path.join(self.output_root, stream_name, filename)
                temporary = destination + ".tmp.png"
                if not cv2.imwrite(temporary, image, [cv2.IMWRITE_PNG_COMPRESSION, self.png_compression]):
                    raise IOError("cv2.imwrite returned false")
                with open(temporary, "rb") as durable_file:
                    os.fsync(durable_file.fileno())
                os.replace(temporary, destination)
                self.csv_writers[stream_name].writerow(
                    [filename, stamp_ns, receive_ns, int(message.header.seq), message.header.frame_id,
                     self.streams[stream_name], message.encoding, int(message.width),
                     int(message.height), int(message.step)]
                )
                with self.state_lock:
                    self.saved[stream_name] += 1
                    if self.first_stamp_ns[stream_name] is None:
                        self.first_stamp_ns[stream_name] = stamp_ns
                    self.last_stamp_ns[stream_name] = stamp_ns
                self._sync_if_due()
            except Exception as exc:
                with self.state_lock:
                    self.failed[stream_name] += 1
                rospy.logerr_throttle(5.0, "Failed to save %s image: %s", stream_name, exc)
            finally:
                self.work_queue.task_done()

    def shutdown(self):
        if self.stop_event.is_set():
            return
        self.stop_event.set()
        self.worker.join(timeout=60.0)
        self._sync_if_due(force=True)
        summary = {
            "rgb_save_hz": self.rgb_save_hz, "gemini_save_hz": self.save_hz,
            "png_compression": self.png_compression, "queue_size": self.queue_size,
            "topics": self.streams, "received": self.received, "selected": self.selected,
            "saved": self.saved, "queue_dropped": self.dropped, "failed": self.failed,
            "first_stamp_ns": self.first_stamp_ns, "last_stamp_ns": self.last_stamp_ns,
        }
        for csv_file in self.csv_files.values():
            csv_file.flush()
            csv_file.close()
        with open(os.path.join(self.output_root, "image_save_summary.json"), "w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
        with open(os.path.join(self.output_root, "camera_info.yaml"), "w", encoding="utf-8") as stream:
            yaml.safe_dump(self.camera_info, stream, allow_unicode=True, sort_keys=False)
        rospy.loginfo("Multimodal image saver stopped: %s", summary)


if __name__ == "__main__":
    rospy.init_node("multimodal_image_saver")
    saver = MultimodalImageSaver()
    rospy.spin()
