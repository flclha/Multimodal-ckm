#!/usr/bin/env python3
"""Create/finalize the Version5 Multimodal_Dataset folder and unified index."""

import argparse
import bisect
import csv
import datetime as dt
import json
import os
import re
import shutil

import yaml


def load_csv(path):
    if not os.path.isfile(path):
        return []
    with open(path, "r", newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def pcd_points(path):
    if not os.path.isfile(path):
        return 0
    with open(path, "rb") as stream:
        for _ in range(100):
            line = stream.readline().decode("ascii", errors="ignore").strip()
            match = re.match(r"POINTS\s+(\d+)$", line)
            if match:
                return int(match.group(1))
            if line.startswith("DATA "):
                break
    return 0


class NearestIndex:
    def __init__(self, rows, key):
        self.rows = rows
        self.key = key
        self.stamps = [int(row[key]) for row in rows]

    def find(self, target):
        if not self.rows:
            return None, None, None
        index = bisect.bisect_left(self.stamps, target)
        choices = [i for i in (index - 1, index) if 0 <= i < len(self.rows)]
        best = min(choices, key=lambda i: abs(self.stamps[i] - target))
        return self.rows[best], self.stamps[best] - target, best


def relative(root, path):
    return os.path.relpath(path, root).replace(os.sep, "/") if path else ""


def sorted_rows(rows, key):
    return sorted((row for row in rows if row.get(key)), key=lambda row: int(row[key]))


def parse_simple_yaml(path):
    result = {}
    if not path or not os.path.isfile(path):
        return result
    for raw in open(path, "r", encoding="utf-8", errors="replace"):
        line = raw.split("#", 1)[0].strip()
        if not line or line.startswith("%") or ":" not in line:
            continue
        key, value = [part.strip() for part in line.split(":", 1)]
        try:
            result[key] = yaml.safe_load(value)
        except Exception:
            result[key] = value.strip('"')
    return result


def image_summary(path):
    rows = load_csv(os.path.join(path, "frames.csv"))
    result = {"saved_frames": len(rows)}
    if len(rows) >= 2:
        first, last = int(rows[0]["header_stamp_ns"]), int(rows[-1]["header_stamp_ns"])
        duration = max(0.0, (last - first) / 1e9)
        result.update({"first_stamp_ns": first, "last_stamp_ns": last,
                       "duration_s": duration,
                       "measured_hz": (len(rows) - 1) / duration if duration else 0.0,
                       "encoding": rows[0].get("encoding", ""),
                       "width": int(rows[0].get("width", 0)),
                       "height": int(rows[0].get("height", 0))})
    return result


def write_configs(args, config, root):
    config_dir = os.path.join(root, "global", "config")
    os.makedirs(config_dir, exist_ok=True)
    gemini_extrinsic = {}
    if args.gemini_extrinsics and os.path.isfile(args.gemini_extrinsics):
        gemini_extrinsic = yaml.safe_load(open(args.gemini_extrinsics, "r", encoding="utf-8"))
    extrinsics = {
        "convention": "p_target = R_target_source * p_source + t_target_source",
        "fast_livo": config.get("extrin_calib", {}),
        "gemini335l": gemini_extrinsic,
    }
    with open(os.path.join(config_dir, "extrinsics.yaml"), "w", encoding="utf-8") as stream:
        yaml.safe_dump(extrinsics, stream, allow_unicode=True, sort_keys=False)

    intrinsics = {"main_rgb": parse_simple_yaml(args.camera_intrinsics)}
    cache = os.path.join(root, "raw", "camera_info.yaml")
    if os.path.isfile(cache):
        intrinsics["gemini335l"] = yaml.safe_load(open(cache, "r", encoding="utf-8")) or {}
    with open(os.path.join(config_dir, "intrinsics.yaml"), "w", encoding="utf-8") as stream:
        yaml.safe_dump(intrinsics, stream, allow_unicode=True, sort_keys=False)

    camera_time = parse_simple_yaml(args.camera_time_config)
    time_sync = {
        "clock_domains": {
            "lidar_imu_main_rgb_pose": "MID360-aligned sensor clock",
            "gemini335l": "Orbbec global/host clock; not hardware-triggered by STM32",
        },
        "main_rgb_model": "t_rgb = fixed_a * (PTS / 48MHz) + b_session",
        "main_rgb": {key: camera_time.get(key) for key in (
            "TimestampSource", "PtsClockHz", "CameraClockScale", "CameraClockOffsetNs",
            "ExposureCenterOffsetNs", "CameraCalibrationOffsetNs", "Stm32TriggerToExposureCenterUs",
            "V4l2SoeToExposureCenterUs") if key in camera_time},
        "manifest_matching": {
            "rgb_and_pose": "nearest header_stamp_ns to LiDAR header_stamp_ns",
            "imu": "nearest imu_stamp_ns to LiDAR header_stamp_ns",
            "gemini_depth_ir": "nearest receive_time_ns to LiDAR receive_time_ns (different clock domains)",
        },
        "important_limit": "Gemini receive-time association is software synchronization, not hardware synchronization.",
    }
    anchor_path = os.path.join(root, "global", "config", "time_alignment_summary.txt")
    if os.path.isfile(anchor_path):
        time_sync["session_anchor_report"] = "time_alignment_summary.txt"
        anchor_values = {}
        for raw_line in open(anchor_path, "r", encoding="utf-8", errors="replace"):
            match = re.match(r"\[anchor\]\s+([^:]+):\s*(.*)$", raw_line.strip())
            if match:
                anchor_values[match.group(1).strip()] = match.group(2).strip()
            result_match = re.match(r"\[anchor\]\s+RESULT=(.*)$", raw_line.strip())
            if result_match:
                anchor_values["RESULT"] = result_match.group(1).strip()
        time_sync["session_anchor"] = anchor_values
    time_sync["timestamp_evidence"] = {
        "main_rgb_pts": "main_rgb_pts_mid360.csv",
        "gemini_device_and_sdk": "../../raw/gemini_hardware_timestamps.csv",
    }
    with open(os.path.join(config_dir, "time_sync.yaml"), "w", encoding="utf-8") as stream:
        yaml.safe_dump(time_sync, stream, allow_unicode=True, sort_keys=False)

    map_cfg = {
        "coordinate_frame": "camera_init",
        "unit": "metre",
        "global_map": "../global_map.pcd",
        "global_map_fields": "x y z rgb",
        "per_frame_lidar": "../../raw/lidar/*.pcd",
        "per_frame_lidar_content": "complete uncolored FAST-LIVO frame after motion deskew and global registration",
        "per_frame_lidar_frame": "camera_init",
        "per_frame_lidar_topic": "/cloud_registered_raw",
        "per_frame_lidar_fields": "x y z intensity",
        "parameters": {
            "point_filter_num": config["preprocess"]["point_filter_num"],
            "filter_size_surf_m": config["preprocess"]["filter_size_surf"],
            "blind_m": config["preprocess"]["blind"],
            "lio_voxel_size_m": config["lio"]["voxel_size"],
            "filter_size_pcd_m": config["pcd_save"]["filter_size_pcd"],
            "pcd_chunk_frames": config["pcd_save"]["interval"],
            "dense_map_en": config["publish"]["dense_map_en"],
        },
        "note": "filter_size_pcd is inactive for chunked interval>0 output; per-frame PCD uses the complete deskewed/registered published cloud and is not filtered by RGB visibility.",
    }
    with open(os.path.join(config_dir, "map_config.yaml"), "w", encoding="utf-8") as stream:
        yaml.safe_dump(map_cfg, stream, allow_unicode=True, sort_keys=False)


def create_manifest(root):
    raw = os.path.join(root, "raw")
    lidar = sorted_rows(load_csv(os.path.join(raw, "lidar", "frames.csv")), "header_stamp_ns")
    rgb = sorted_rows(load_csv(os.path.join(raw, "rgb", "frames.csv")), "header_stamp_ns")
    pose = sorted_rows(load_csv(os.path.join(raw, "pos", "poses.csv")), "pose_stamp_ns")
    imu = sorted_rows(load_csv(os.path.join(raw, "imu", "imu.csv")), "imu_stamp_ns")
    depth = sorted_rows(load_csv(os.path.join(raw, "depth", "frames.csv")), "receive_time_ns")
    left = sorted_rows(load_csv(os.path.join(raw, "ir_left", "frames.csv")), "receive_time_ns")
    right = sorted_rows(load_csv(os.path.join(raw, "ir_right", "frames.csv")), "receive_time_ns")
    rgb_index = NearestIndex(rgb, "header_stamp_ns")
    pose_index = NearestIndex(pose, "pose_stamp_ns")
    imu_index = NearestIndex(imu, "imu_stamp_ns")
    depth_index = NearestIndex(depth, "receive_time_ns")
    left_index = NearestIndex(left, "receive_time_ns")
    right_index = NearestIndex(right, "receive_time_ns")
    columns = [
        "frame_id", "global_time_ns",
        "lidar_time_delta_us", "lidar_path",
        "rgb_time_ns", "rgb_time_delta_us", "rgb_path",
        "pose_time_ns", "pose_time_delta_us", "pose_row",
        "imu_time_ns", "imu_time_delta_us", "imu_row",
        "depth_align_time_ns", "depth_time_delta_us", "depth_path",
        "ir_left_align_time_ns", "ir_left_time_delta_us", "ir_left_path",
        "ir_right_align_time_ns", "ir_right_time_delta_us", "ir_right_path",
        "channel_align_time_ns", "channel_time_delta_us", "channel_path",
        "valid", "notes",
    ]
    out_path = os.path.join(root, "global", "manifest.csv")
    with open(out_path, "w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for index, lid in enumerate(lidar, 1):
            ls, lr = int(lid["header_stamp_ns"]), int(lid["receive_time_ns"])
            rr, rd, _ = rgb_index.find(ls)
            pr, pd, pose_row_index = pose_index.find(ls)
            ir, imd, imu_row_index = imu_index.find(ls)
            dr, dd, _ = depth_index.find(lr)
            il, ild, _ = left_index.find(lr)
            irr, ird, _ = right_index.find(lr)
            notes = []
            valid = True
            if rr is None or abs(rd) > 100e6:
                valid = False; notes.append("rgb_missing_or_gt100ms")
            if pr is None or abs(pd) > 100e6:
                valid = False; notes.append("pose_missing_or_gt100ms")
            row = {
                "frame_id": "{:06d}".format(index),
                # The registered LiDAR stamp defines the dataset-global time
                # axis. Every *_time_delta_us is signed sensor-minus-global.
                "global_time_ns": ls,
                "lidar_time_delta_us": "0.000",
                "lidar_path": relative(root, os.path.join(raw, "lidar", lid["filename"])),
                "rgb_time_ns": rr.get("header_stamp_ns", "") if rr else "",
                "rgb_time_delta_us": "{:.3f}".format(rd / 1e3) if rd is not None else "",
                "rgb_path": relative(root, os.path.join(raw, "rgb", rr["filename"])) if rr else "",
                "pose_time_ns": pr.get("pose_stamp_ns", "") if pr else "",
                "pose_time_delta_us": "{:.3f}".format(pd / 1e3) if pd is not None else "",
                "pose_row": pose_row_index + 2 if pose_row_index is not None else "",
                "imu_time_ns": ir.get("imu_stamp_ns", "") if ir else "",
                "imu_time_delta_us": "{:.3f}".format(imd / 1e3) if imd is not None else "",
                "imu_row": imu_row_index + 2 if imu_row_index is not None else "",
                # Gemini is not hardware-synchronized to MID360. Its matched
                # receive-time delta is projected onto the global axis here;
                # raw receive timestamps remain only in modality frames.csv.
                "depth_align_time_ns": ls + dd if dd is not None else "",
                "depth_time_delta_us": "{:.3f}".format(dd / 1e3) if dd is not None else "",
                "depth_path": relative(root, os.path.join(raw, "depth", dr["filename"])) if dr else "",
                "ir_left_align_time_ns": ls + ild if ild is not None else "",
                "ir_left_time_delta_us": "{:.3f}".format(ild / 1e3) if ild is not None else "",
                "ir_left_path": relative(root, os.path.join(raw, "ir_left", il["filename"])) if il else "",
                "ir_right_align_time_ns": ls + ird if ird is not None else "",
                "ir_right_time_delta_us": "{:.3f}".format(ird / 1e3) if ird is not None else "",
                "ir_right_path": relative(root, os.path.join(raw, "ir_right", irr["filename"])) if irr else "",
                "channel_align_time_ns": "", "channel_time_delta_us": "", "channel_path": "",
                "valid": str(valid).lower(), "notes": ";".join(notes),
            }
            writer.writerow(row)
    return len(lidar)


def write_readme(root, run_id):
    text = """# Version5 Multimodal Dataset `{}`

## Directory definition

- `raw/lidar`: one binary PCD per FAST-LIVO LiDAR update from `/cloud_registered_raw`. Every frame is motion-deskewed, registered in `camera_init`, uncolored, and not filtered by camera FOV/visibility. Fields are XYZ and intensity.
- `raw/imu/imu.csv`: continuous MID360 IMU samples (m/s² and rad/s).
- `raw/rgb`: original global-shutter RGB PNG and frame timestamp table.
- `raw/pos/poses.csv`: FAST-LIVO global pose for every LiDAR update.
- `raw/depth`: Gemini raw depth, losslessly stored as 16-bit PNG.
- `raw/ir_left`, `raw/ir_right`: Gemini infrared PNG frames.
- `raw/channel`: reserved for wireless-channel MAT files; Version5 does not fabricate channel data.
- `global/manifest.csv`: one row per LiDAR frame with nearest RGB/pose/IMU/depth/IR paths and signed time differences.
- `global/global_map.pcd`: FAST-LIVO RGB global map in `camera_init` (XYZRGB).
- `global/lidar_map.pcd`: complete uncolored LiDAR map retained for geometry/classification.
- `global/config`: calibration, intrinsics, synchronization evidence and map parameters.

## Time semantics

MID360 LiDAR, IMU, main RGB and FAST-LIVO pose use the MID360-aligned sensor time axis. Main RGB hardware PTS is 48 MHz and is mapped by `t = a*(PTS/48MHz)+b_session`; raw PTS evidence remains under `global/config` but is intentionally omitted from the compact manifest. Gemini 335L is not driven by this trigger board, so its receive-time association is projected onto `global_time_ns` as `*_align_time_ns`. All `*_time_delta_us` values are signed as aligned sensor time minus the LiDAR reference time.

## Important notes

`global/manifest.csv` is the authoritative file association table. `global_time_ns` is the registered LiDAR/MID360 reference time. Every `*_time_delta_us` is signed as aligned sensor time minus `global_time_ns`; a positive value means the selected sensor sample is later. Gemini aligned times are receive-time associations projected onto this global axis because Gemini is not hardware-synchronized to MID360. Never infer correspondence only from six-digit filenames. Check `valid` and signed time deltas before training or evaluation. See `global/config/map_config.yaml` for filtering and resolution. Additional `global/pcd_chunks` and `global/colmap` directories are retained as reproducibility/debug artifacts.
""".format(run_id)
    with open(os.path.join(root, "README.md"), "w", encoding="utf-8") as stream:
        stream.write(text)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("start", "finalize"))
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--dataset-dir", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--camera-intrinsics", default="")
    parser.add_argument("--camera-time-config", default="")
    parser.add_argument("--gemini-extrinsics", default="")
    args = parser.parse_args()
    root = os.path.abspath(args.dataset_dir)
    os.makedirs(os.path.join(root, "global", "config"), exist_ok=True)
    config = yaml.safe_load(open(args.config, "r", encoding="utf-8"))
    write_configs(args, config, root)
    write_readme(root, args.run_id)
    metadata_path = os.path.join(root, "global", "dataset_metadata.yaml")
    metadata = {
        "run_id": args.run_id,
        "dataset_root": root,
        "started_or_updated_at": dt.datetime.now().astimezone().isoformat(),
        "coordinate_frame": "camera_init",
        "topics": {"lidar": "/cloud_registered_raw", "imu": "/livox/imu",
                   "rgb": "/uvc_camera/image_sync", "pose": "/aft_mapped_to_init",
                   "depth": "/gemini335l/depth/image_raw",
                   "ir_left": "/gemini335l/left_ir/image_raw",
                   "ir_right": "/gemini335l/right_ir/image_raw"},
    }
    if args.mode == "finalize":
        frames = create_manifest(root)
        raw = os.path.join(root, "raw")
        raw_summary_path = os.path.join(raw, "raw_record_summary.json")
        image_summary_path = os.path.join(raw, "image_save_summary.json")
        metadata["finalized_at"] = dt.datetime.now().astimezone().isoformat()
        metadata["results"] = {
            "manifest_lidar_frames": frames,
            "imu_samples": len(load_csv(os.path.join(raw, "imu", "imu.csv"))),
            "pose_samples": len(load_csv(os.path.join(raw, "pos", "poses.csv"))),
            "rgb": image_summary(os.path.join(raw, "rgb")),
            "depth": image_summary(os.path.join(raw, "depth")),
            "ir_left": image_summary(os.path.join(raw, "ir_left")),
            "ir_right": image_summary(os.path.join(raw, "ir_right")),
            "global_map_points": pcd_points(os.path.join(root, "global", "global_map.pcd")),
            "lidar_map_points": pcd_points(os.path.join(root, "global", "lidar_map.pcd")),
        }
        for name, path in (("raw_writer", raw_summary_path), ("image_writer", image_summary_path)):
            if os.path.isfile(path):
                metadata["results"][name] = json.load(open(path, "r", encoding="utf-8"))
    with open(metadata_path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, allow_unicode=True, sort_keys=False)
    print(metadata_path)


if __name__ == "__main__":
    main()
