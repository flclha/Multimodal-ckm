#!/usr/bin/env python3
"""在小车完全静止时，测量 FAST-LIVO 坐标随时间间隔产生的重复定位漂移。"""

import argparse
import csv
import json
import math
import os
import statistics
import threading
import time
from collections import deque
from datetime import datetime

import rospy
from nav_msgs.msg import Odometry


class PoseSampler:
    """缓存里程计位姿，并在指定静止窗口内计算稳健的坐标中位数。"""

    def __init__(self, topic):
        self.topic = topic
        self.lock = threading.Lock()
        self.samples = deque(maxlen=20000)

    def callback(self, msg):
        """记录本机单调时钟、ROS 时间戳和 XYZ 坐标。"""
        p = msg.pose.pose.position
        row = (
            time.monotonic(),
            msg.header.stamp.to_sec(),
            float(p.x),
            float(p.y),
            float(p.z),
        )
        with self.lock:
            self.samples.append(row)

    def wait_for_pose(self, timeout=30.0):
        """确认 FAST-LIVO 位姿 topic 正在发布。"""
        rospy.wait_for_message(self.topic, Odometry, timeout=timeout)
        time.sleep(0.2)

    def sample(self, window):
        """采样 window 秒，用 XYZ 中位数降低单帧定位噪声影响。"""
        start = time.monotonic()
        time.sleep(window)
        end = time.monotonic()
        with self.lock:
            rows = [row for row in self.samples if start <= row[0] <= end]
        if len(rows) < 5:
            raise RuntimeError("静止窗口只收到 {} 个位姿样本".format(len(rows)))

        x = statistics.median(row[2] for row in rows)
        y = statistics.median(row[3] for row in rows)
        z = statistics.median(row[4] for row in rows)
        return {
            "mono": statistics.median(row[0] for row in rows),
            "stamp": statistics.median(row[1] for row in rows),
            "x": x,
            "y": y,
            "z": z,
            "count": len(rows),
            # 静止抖动取窗口内相对中位数的最大 XY 距离。
            "spread": max(math.hypot(row[2] - x, row[3] - y) for row in rows),
        }


def linear_relation(xs, ys):
    """计算漂移均值与时间间隔之间的一阶斜率和 Pearson 相关系数。"""
    mean_x = statistics.mean(xs)
    mean_y = statistics.mean(ys)
    denom_x = sum((x - mean_x) ** 2 for x in xs)
    slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom_x
    intercept = mean_y - slope * mean_x
    denom_y = sum((y - mean_y) ** 2 for y in ys)
    correlation = 0.0
    if denom_x > 0.0 and denom_y > 0.0:
        correlation = sum(
            (x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)
        ) / math.sqrt(denom_x * denom_y)
    return slope, intercept, correlation


def summarize(rows, intervals):
    """按5/10/15秒分组，统计漂移均值、标准差和时间相关性。"""
    summary = {}
    for interval in sorted(intervals):
        group = [row for row in rows if row["requested_interval_s"] == interval]
        summary[str(interval)] = {
            "groups": len(group),
            "actual_interval_mean_s": statistics.mean(row["actual_interval_s"] for row in group),
            "mean_dx_mm": statistics.mean(row["dx"] for row in group) * 1000.0,
            "mean_dy_mm": statistics.mean(row["dy"] for row in group) * 1000.0,
            "mean_dz_mm": statistics.mean(row["dz"] for row in group) * 1000.0,
            "mean_xy_drift_mm": statistics.mean(row["drift_xy"] for row in group) * 1000.0,
            "std_xy_drift_mm": statistics.stdev(row["drift_xy"] for row in group) * 1000.0,
            "mean_3d_drift_mm": statistics.mean(row["drift_3d"] for row in group) * 1000.0,
            "std_3d_drift_mm": statistics.stdev(row["drift_3d"] for row in group) * 1000.0,
            "mean_start_spread_mm": statistics.mean(row["start_spread"] for row in group) * 1000.0,
            "mean_end_spread_mm": statistics.mean(row["end_spread"] for row in group) * 1000.0,
        }

    xs = sorted(intervals)
    xy_means = [summary[str(x)]["mean_xy_drift_mm"] for x in xs]
    drift_3d_means = [summary[str(x)]["mean_3d_drift_mm"] for x in xs]
    xy_slope, xy_intercept, xy_r = linear_relation(xs, xy_means)
    d3_slope, d3_intercept, d3_r = linear_relation(xs, drift_3d_means)
    return {
        "by_interval": summary,
        "relationship": {
            "xy_slope_mm_per_s": xy_slope,
            "xy_intercept_mm": xy_intercept,
            "xy_pearson_r": xy_r,
            "drift_3d_slope_mm_per_s": d3_slope,
            "drift_3d_intercept_mm": d3_intercept,
            "drift_3d_pearson_r": d3_r,
        },
    }


def main():
    """依次完成15组静止漂移测试并保存原始数据与汇总结果。"""
    parser = argparse.ArgumentParser(description="FAST-LIVO 静止坐标时间漂移测试")
    parser.add_argument("--topic", default="/aft_mapped_to_init")
    parser.add_argument("--groups", type=int, default=5)
    parser.add_argument("--intervals", type=float, nargs="+", default=[10.0, 5.0, 15.0])
    parser.add_argument("--window", type=float, default=1.0)
    args = parser.parse_args()

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.expanduser("~/fast_livo_baseline/static_drift_{}".format(run_id))
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "static_drift_groups.csv")
    summary_path = os.path.join(out_dir, "static_drift_summary.json")

    rospy.init_node("fast_livo_static_drift_test", anonymous=True, disable_signals=True)
    sampler = PoseSampler(args.topic)
    subscriber = rospy.Subscriber(args.topic, Odometry, sampler.callback, queue_size=2000)
    rows = []

    fieldnames = [
        "requested_interval_s", "group", "actual_interval_s",
        "start_stamp", "end_stamp", "start_x", "start_y", "start_z",
        "end_x", "end_y", "end_z", "dx", "dy", "dz",
        "drift_xy", "drift_3d", "start_spread", "end_spread",
        "start_samples", "end_samples",
    ]

    try:
        sampler.wait_for_pose()
        print("静止漂移测试开始：topic={}，输出={}".format(args.topic, out_dir), flush=True)
        with open(csv_path, "w", newline="", buffering=1) as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            for interval in args.intervals:
                for group_index in range(1, args.groups + 1):
                    print("[{:.0f}s] 第 {}/{} 组：采集起始坐标".format(
                        interval, group_index, args.groups), flush=True)
                    start_pose = sampler.sample(args.window)

                    # 让两个采样窗口的中心时刻尽可能相隔指定秒数。
                    target_second_window_start = start_pose["mono"] + interval - args.window / 2.0
                    remaining = target_second_window_start - time.monotonic()
                    if remaining > 0.0:
                        time.sleep(remaining)
                    end_pose = sampler.sample(args.window)

                    dx = end_pose["x"] - start_pose["x"]
                    dy = end_pose["y"] - start_pose["y"]
                    dz = end_pose["z"] - start_pose["z"]
                    row = {
                        "requested_interval_s": interval,
                        "group": group_index,
                        "actual_interval_s": end_pose["mono"] - start_pose["mono"],
                        "start_stamp": start_pose["stamp"],
                        "end_stamp": end_pose["stamp"],
                        "start_x": start_pose["x"], "start_y": start_pose["y"], "start_z": start_pose["z"],
                        "end_x": end_pose["x"], "end_y": end_pose["y"], "end_z": end_pose["z"],
                        "dx": dx, "dy": dy, "dz": dz,
                        "drift_xy": math.hypot(dx, dy),
                        "drift_3d": math.sqrt(dx * dx + dy * dy + dz * dz),
                        "start_spread": start_pose["spread"],
                        "end_spread": end_pose["spread"],
                        "start_samples": start_pose["count"],
                        "end_samples": end_pose["count"],
                    }
                    rows.append(row)
                    writer.writerow(row)
                    csv_file.flush()
                    print(
                        "[{:.0f}s] 第 {}/{} 组完成：实际间隔 {:.3f}s，XY={:.2f}mm，3D={:.2f}mm".format(
                            interval, group_index, args.groups, row["actual_interval_s"],
                            row["drift_xy"] * 1000.0, row["drift_3d"] * 1000.0
                        ),
                        flush=True,
                    )

        result = summarize(rows, args.intervals)
        result.update({
            "topic": args.topic,
            "groups_per_interval": args.groups,
            "sampling_window_s": args.window,
            "csv_path": csv_path,
        })
        with open(summary_path, "w") as summary_file:
            json.dump(result, summary_file, indent=2, ensure_ascii=False)
        print("FINAL_SUMMARY_JSON={}".format(json.dumps(result, ensure_ascii=False)), flush=True)
        print("SUMMARY_PATH={}".format(summary_path), flush=True)
    finally:
        subscriber.unregister()


if __name__ == "__main__":
    main()
