#!/usr/bin/env python3
"""FAST-LIVO 0.6 m 网格定位精度交互式测量工具。"""

import argparse
import csv
import math
import os
import socket
import statistics
import sys
import threading
import time
import xmlrpc.client
from collections import deque
from datetime import datetime
import rospy
from nav_msgs.msg import Odometry


def check_ros_master(timeout=3.0):
    """在创建 ROS 节点前检查 master，避免只看到难以理解的注册失败提示。"""
    master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    old_timeout = socket.getdefaulttimeout()
    socket.setdefaulttimeout(timeout)
    try:
        master = xmlrpc.client.ServerProxy(master_uri)
        code, message, _ = master.getPid("/fast_livo_grid_baseline_preflight")
        if code != 1:
            raise RuntimeError("ROS master 返回异常：{}".format(message))
    except (OSError, xmlrpc.client.Error) as exc:
        raise RuntimeError(
            "无法连接 ROS master ({})。请先运行 Version5 的 start_slam1，"
            "等待定位和 RViz 启动完成后，再运行本工具。原始错误：{}".format(
                master_uri, exc
            )
        )
    finally:
        socket.setdefaulttimeout(old_timeout)
    return master_uri


class Baseline:
    """接收 FAST-LIVO 位姿，并完成单段与累计距离误差统计。"""

    def __init__(self, topic, grid, window, out_dir):
        """保存测量参数，初始化位姿缓存和结果文件状态。"""
        self.topic, self.grid, self.window = topic, grid, window
        self.out_dir = out_dir
        self.lock = threading.Lock()
        # 最多缓存 10000 个位姿，供静止窗口取样和中位数估计。
        self.samples = deque(maxlen=10000)
        # 保存上一帧位置，用于估计连续轨迹长度。
        self.last = None
        # 累积每相邻两帧 XY 位移，作为辅助轨迹长度，不替代 A 点测距。
        self.raw_path = 0.0
        self.points = []
        self.file = self.writer = None
        # C1 保存回环基准点；后续每次 C2 都与该基准点比较全局漂移。
        self.return_reference = None
        self.return_check_index = 0
        self.drift_file = self.drift_writer = None
        self.drift_path = None
        # D 命令采用两次输入配对：第一次保存起点，第二次计算终点距离。
        self.distance_start = None
        self.distance_index = 0
        self.distance_file = self.distance_writer = None
        self.distance_path = None

    def callback(self, msg):
        """位姿回调：缓存坐标、时间戳，并累加相邻帧的 XY 路程。"""
        p = msg.pose.pose.position
        # 提取当前位姿，计算它与上一帧之间的水平距离。
        xyz = (float(p.x), float(p.y), float(p.z))
        with self.lock:
            if self.last:
                self.raw_path += math.hypot(xyz[0]-self.last[0], xyz[1]-self.last[1])
            self.last = xyz
            self.samples.append((time.monotonic(), msg.header.stamp.to_sec(), *xyz))

    def wait(self):
        """等待第一条有效里程计消息，确认 FAST-LIVO 位姿 topic 已经发布。"""
        print("等待位姿 {} ...".format(self.topic), flush=True)
        try:
            rospy.wait_for_message(self.topic, Odometry, timeout=30)
        except rospy.ROSException:
            raise RuntimeError(
                "ROS master 已连接，但 30 秒内没有收到 {}。"
                "请确认 Version5 SLAM 已完成启动，且 topic 名称正确。".format(self.topic)
            )
        time.sleep(0.2)

    def open_csv(self):
        """创建网格误差和回点漂移 CSV，所有测量结果都会即时落盘。"""
        os.makedirs(self.out_dir, exist_ok=True)
        path = os.path.join(self.out_dir, "grid_baseline.csv")
        self.file = open(path, "w", newline="", buffering=1)
        names = ["index", "time", "pose_stamp", "x", "y", "z", "samples",
                 "stationary_spread", "segment_xy", "segment_3d", "segment_error",
                 "segment_error_pct", "cumulative_xy", "expected", "cumulative_error",
                 "cumulative_error_pct", "raw_path_since_a"]
        self.writer = csv.DictWriter(self.file, fieldnames=names)
        self.writer.writeheader()

        # C1/C2 使用独立文件，避免与原来的 0.6 m 网格 A/B 数据混在一起。
        self.drift_path = os.path.join(self.out_dir, "return_drift.csv")
        self.drift_file = open(self.drift_path, "w", newline="", buffering=1)
        drift_names = [
            "command", "check_index", "time", "pose_stamp",
            "reference_x", "reference_y", "reference_z",
            "current_x", "current_y", "current_z",
            "dx", "dy", "dz", "drift_xy", "drift_3d",
            "reference_stationary_spread", "current_stationary_spread",
            "raw_path_since_c1",
        ]
        self.drift_writer = csv.DictWriter(self.drift_file, fieldnames=drift_names)
        self.drift_writer.writeheader()

        # D/D 两点测距单独保存，便于重复测量和后续统计。
        self.distance_path = os.path.join(self.out_dir, "distance_pairs.csv")
        self.distance_file = open(self.distance_path, "w", newline="", buffering=1)
        distance_names = [
            "pair_index", "start_time", "end_time", "start_pose_stamp", "end_pose_stamp",
            "start_x", "start_y", "start_z", "end_x", "end_y", "end_z",
            "dx", "dy", "dz", "distance_xy", "distance_3d", "elapsed_s",
            "start_stationary_spread", "end_stationary_spread", "raw_path_between_d",
        ]
        self.distance_writer = csv.DictWriter(self.distance_file, fieldnames=distance_names)
        self.distance_writer.writeheader()
        return path

    def stationary(self):
        """静止采样 window 秒，以坐标中位数作为当前稳定位置。"""
        print("保持完全静止，采样 {:.1f} 秒...".format(self.window), flush=True)
        start = time.monotonic()
        time.sleep(self.window)
        end = time.monotonic()
        with self.lock:
            rows = [r for r in self.samples if start <= r[0] <= end]
            raw = self.raw_path
        if len(rows) < 5:
            raise RuntimeError("静止窗口仅收到 {} 个位姿".format(len(rows)))
        x = statistics.median(r[2] for r in rows)
        y = statistics.median(r[3] for r in rows)
        z = statistics.median(r[4] for r in rows)
        return {"time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                "stamp": statistics.median(r[1] for r in rows), "x": x, "y": y, "z": z,
                "samples": len(rows),
                "spread": max(math.hypot(r[2]-x, r[3]-y) for r in rows), "raw": raw}

    def mark(self):
        """用户输入 A 时记录稳定点，并计算上一网格段和累计距离误差。"""
        p = self.stationary(); p["index"] = len(self.points)
        if not self.points:
            p.update(seg=0.0, seg3=0.0, err=0.0, err_pct=0.0, cum=0.0,
                     expected=0.0, cum_err=0.0, cum_pct=0.0, raw_seg=0.0)
            self.points.append(p); self.save(p)
            print("\n[A] 起点#0 x={:.4f} y={:.4f} z={:.4f} m，静止抖动 {:.2f} mm".format(
                  p["x"], p["y"], p["z"], p["spread"]*1000))
            print("移动一个0.6m网格、停稳，再输入A。"); return
        old = self.points[-1]
        dx, dy, dz = p["x"]-old["x"], p["y"]-old["y"], p["z"]-old["z"]
        seg = math.hypot(dx, dy); seg3 = math.sqrt(dx*dx+dy*dy+dz*dz)
        expected = p["index"] * self.grid; cum = old["cum"] + seg
        p.update(seg=seg, seg3=seg3, err=seg-self.grid,
                 err_pct=(seg-self.grid)/self.grid*100, cum=cum, expected=expected,
                 cum_err=cum-expected, cum_pct=(cum-expected)/expected*100,
                 raw_seg=p["raw"]-old["raw"])
        self.points.append(p); self.save(p)
        print("\n[A] 第{}段：真值 {:.4f} m，定位 {:.4f} m".format(p["index"], self.grid, seg))
        print("  本段误差 {:+.4f} m ({:+.2f}%)，dz={:+.4f} m".format(p["err"], p["err_pct"], dz))
        print("  累计真值 {:.4f} m，定位 {:.4f} m，误差 {:+.4f} m ({:+.2f}%)".format(
              expected, cum, p["cum_err"], p["cum_pct"]))
        print("  静止抖动 {:.2f} mm".format(p["spread"]*1000))

    def save(self, p):
        """把一个 A 点及其误差指标写入 CSV，并立即刷新到磁盘。"""
        self.writer.writerow({"index": p["index"], "time": p["time"], "pose_stamp": p["stamp"],
            "x": p["x"], "y": p["y"], "z": p["z"], "samples": p["samples"],
            "stationary_spread": p["spread"], "segment_xy": p["seg"], "segment_3d": p["seg3"],
            "segment_error": p["err"], "segment_error_pct": p["err_pct"],
            "cumulative_xy": p["cum"], "expected": p["expected"],
            "cumulative_error": p["cum_err"], "cumulative_error_pct": p["cum_pct"],
            "raw_path_since_a": p["raw_seg"]}); self.file.flush()

    def save_return_drift(self, command, current, dx, dy, dz, drift_xy, drift_3d):
        """保存一次 C1/C2 回点检查，便于后续比较多次闭环漂移。"""
        ref = self.return_reference
        self.drift_writer.writerow({
            "command": command,
            "check_index": self.return_check_index,
            "time": current["time"],
            "pose_stamp": current["stamp"],
            "reference_x": ref["x"],
            "reference_y": ref["y"],
            "reference_z": ref["z"],
            "current_x": current["x"],
            "current_y": current["y"],
            "current_z": current["z"],
            "dx": dx,
            "dy": dy,
            "dz": dz,
            "drift_xy": drift_xy,
            "drift_3d": drift_3d,
            "reference_stationary_spread": ref["spread"],
            "current_stationary_spread": current["spread"],
            # 这是位姿逐帧累计的辅助路程，会受定位抖动影响，不作为真值距离。
            "raw_path_since_c1": current["raw"] - ref["raw"],
        })
        self.drift_file.flush()

    def set_return_reference(self):
        """C1：在已知物理点静止采样，保存第一次到达时的全局坐标。"""
        reference = self.stationary()
        self.return_reference = reference
        self.return_check_index = 0
        self.save_return_drift("C1", reference, 0.0, 0.0, 0.0, 0.0, 0.0)
        print("\n[C1] 已设置回点基准：")
        print("  x={:.4f}  y={:.4f}  z={:.4f} m".format(
            reference["x"], reference["y"], reference["z"]))
        print("  静止抖动 {:.2f} mm".format(reference["spread"] * 1000))
        print("  现在可让小车行驶一段路线；回到同一物理点并停稳后输入 C2。")

    def check_return_drift(self):
        """C2：回到 C1 的物理位置后，计算 XYZ、水平和三维全局漂移。"""
        if self.return_reference is None:
            raise RuntimeError("尚未设置基准点，请先在起点停稳后输入 C1")

        current = self.stationary()
        ref = self.return_reference
        dx = current["x"] - ref["x"]
        dy = current["y"] - ref["y"]
        dz = current["z"] - ref["z"]
        drift_xy = math.hypot(dx, dy)
        drift_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
        self.return_check_index += 1
        self.save_return_drift("C2", current, dx, dy, dz, drift_xy, drift_3d)

        print("\n[C2] 第 {} 次回点漂移检查：".format(self.return_check_index))
        print("  C1=({:.4f}, {:.4f}, {:.4f}) m".format(
            ref["x"], ref["y"], ref["z"]))
        print("  C2=({:.4f}, {:.4f}, {:.4f}) m".format(
            current["x"], current["y"], current["z"]))
        print("  Δx={:+.4f}  Δy={:+.4f}  Δz={:+.4f} m".format(dx, dy, dz))
        print("  XY 全局漂移={:.4f} m ({:.1f} mm)".format(
            drift_xy, drift_xy * 1000))
        print("  三维全局漂移={:.4f} m ({:.1f} mm)".format(
            drift_3d, drift_3d * 1000))
        print("  C2 静止抖动 {:.2f} mm；辅助累计轨迹 {:.3f} m".format(
            current["spread"] * 1000, current["raw"] - ref["raw"]))

    def toggle_distance_measurement(self):
        """D：第一次保存起点，第二次计算两点的水平距离和三维距离。"""
        current = self.stationary()

        if self.distance_start is None:
            self.distance_start = current
            print("\n[D1] 已记录测距起点：")
            print("  x={:.4f}  y={:.4f}  z={:.4f} m".format(
                current["x"], current["y"], current["z"]))
            print("  静止抖动 {:.2f} mm".format(current["spread"] * 1000))
            print("  移动到终点并完全停稳后，再输入一次 D。")
            return

        start = self.distance_start
        dx = current["x"] - start["x"]
        dy = current["y"] - start["y"]
        dz = current["z"] - start["z"]
        distance_xy = math.hypot(dx, dy)
        distance_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
        elapsed_s = current["stamp"] - start["stamp"]
        self.distance_index += 1

        self.distance_writer.writerow({
            "pair_index": self.distance_index,
            "start_time": start["time"],
            "end_time": current["time"],
            "start_pose_stamp": start["stamp"],
            "end_pose_stamp": current["stamp"],
            "start_x": start["x"], "start_y": start["y"], "start_z": start["z"],
            "end_x": current["x"], "end_y": current["y"], "end_z": current["z"],
            "dx": dx, "dy": dy, "dz": dz,
            "distance_xy": distance_xy,
            "distance_3d": distance_3d,
            "elapsed_s": elapsed_s,
            "start_stationary_spread": start["spread"],
            "end_stationary_spread": current["spread"],
            # 逐帧累计路程容易受回退和静止抖动影响，仅作为辅助信息。
            "raw_path_between_d": current["raw"] - start["raw"],
        })
        self.distance_file.flush()

        print("\n[D2] 第 {} 组两点测距：".format(self.distance_index))
        print("  起点=({:.4f}, {:.4f}, {:.4f}) m".format(
            start["x"], start["y"], start["z"]))
        print("  终点=({:.4f}, {:.4f}, {:.4f}) m".format(
            current["x"], current["y"], current["z"]))
        print("  Δx={:+.4f}  Δy={:+.4f}  Δz={:+.4f} m".format(dx, dy, dz))
        print("  XY 水平距离={:.4f} m；三维距离={:.4f} m".format(
            distance_xy, distance_3d))
        print("  起点/终点静止抖动={:.2f}/{:.2f} mm；间隔={:.1f} s".format(
            start["spread"] * 1000, current["spread"] * 1000, elapsed_s))

        # 一组测量完成后清空起点，下一次 D 自动开始新的一组。
        self.distance_start = None
        

    def report(self):
        """用户输入 B 时汇总所有已记录网格段的长期累计误差。"""
        n = len(self.points)-1
        if n < 1: print("[B] 至少需要起点和一个终点（两次A）"); return
        expected, measured = n*self.grid, self.points[-1]["cum"]
        errors = [p["err"] for p in self.points[1:]]
        mae = sum(abs(e) for e in errors)/n
        rmse = math.sqrt(sum(e*e for e in errors)/n)
        print("\n[B] 长期累计定位精度：{}个0.6m路段".format(n))
        print("  理论/定位累计：{:.4f} / {:.4f} m".format(expected, measured))
        print("  累计误差：{:+.4f} m ({:+.2f}%)".format(measured-expected, (measured-expected)/expected*100))
        print("  单段 MAE/RMSE：{:.4f} / {:.4f} m；最大误差 {:+.4f} m".format(
              mae, rmse, max(errors, key=abs)))

    def status(self):
        """用户输入 S 时显示最新位置、接收延迟和已记录 A 点数。"""
        with self.lock: row = self.samples[-1] if self.samples else None
        if not row: print("[S] 无位姿"); return
        print("[S] x={:.4f} y={:.4f} z={:.4f} m，延迟{:.3f}s，A点{}".format(
              row[2], row[3], row[4], time.monotonic()-row[0], len(self.points)))


def main():
    """解析参数、完成 ROS 预检，然后进入 A/B/C/D/S/Q 交互循环。"""
    ap = argparse.ArgumentParser(description="FAST-LIVO 0.6m网格定位精度baseline")
    ap.add_argument("--topic", default="/aft_mapped_to_init")
    ap.add_argument("--grid", type=float, default=0.6)
    ap.add_argument("--sample-seconds", type=float, default=1.0)
    args = ap.parse_args()
    out = os.path.expanduser("~/fast_livo_baseline/" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    try:
        master_uri = check_ros_master()
        print("ROS master 已连接：{}".format(master_uri), flush=True)
        rospy.init_node("fast_livo_grid_baseline", anonymous=True, disable_signals=True)
    except (RuntimeError, rospy.ROSException) as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        return 2

    tool = Baseline(args.topic, args.grid, args.sample_seconds, out)
    sub = rospy.Subscriber(args.topic, Odometry, tool.callback, queue_size=2000)
    try:
        tool.wait(); path = tool.open_csv()
        print("就绪：起点输入A；每移动0.6m并停稳后输入A；输入B查看累计误差。")
        print("回点漂移：起点输入C1；绕行后回到同一物理点并停稳，输入C2。")
        print("两点测距：第一次输入D记录起点；移动并停稳后第二次输入D计算距离。")
        print("A=网格点 B=累计 C1=回点基准 C2=回点检查 D=两点测距 S=状态 Q=退出")
        print("网格CSV={}；回点CSV={}；测距CSV={}".format(
            path, tool.drift_path, tool.distance_path))
        while not rospy.is_shutdown():
            cmd = input("baseline> ").strip().upper()
            if cmd == "A":
                try: tool.mark()
                except RuntimeError as e: print("[A]失败：{}".format(e))
            elif cmd == "B": tool.report()
            elif cmd == "C1":
                try: tool.set_return_reference()
                except RuntimeError as e: print("[C1]失败：{}".format(e))
            elif cmd == "C2":
                try: tool.check_return_drift()
                except RuntimeError as e: print("[C2]失败：{}".format(e))
            elif cmd == "D":
                try: tool.toggle_distance_measurement()
                except RuntimeError as e: print("[D]失败：{}".format(e))
            elif cmd == "S": tool.status()
            elif cmd in ("Q", "QUIT", "EXIT"): break
            elif cmd: print("请输入A、B、C1、C2、D、S或Q")
    except (KeyboardInterrupt, EOFError): print("\n退出")
    except RuntimeError as e: print("ERROR: {}".format(e), file=sys.stderr); return 2
    finally:
        sub.unregister()
        if tool.file: tool.file.close()
        if tool.drift_file: tool.drift_file.close()
        if tool.distance_file: tool.distance_file.close()
    return 0


if __name__ == "__main__": sys.exit(main())
