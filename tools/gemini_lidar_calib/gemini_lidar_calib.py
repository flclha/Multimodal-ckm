#!/usr/bin/env python3
"""Gemini 335L RGB <-> Livox LiDAR multi-view calibration workflow.

The per-view target detector is FAST-Calib (four ArUco markers plus four
LiDAR-visible circular holes).  This wrapper owns capture, immutable session
metadata, multi-view SVD, two independent quality gates and Depth-frame export.
It never edits FAST-LIVO's active YAML automatically.
"""

import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml


ROOT = Path("/home/nuc/gemini335l_lidar_calibration")
CURRENT = ROOT / "current_session.txt"
FAST_CALIB = Path("/home/nuc/versions/version5/catkin_ws/src/FAST-Calib")
CAPTURE = FAST_CALIB / "scripts/capture_calib_sample.py"
LIDAR_TOPIC = "/livox/lidar"
RGB_TOPIC = "/gemini335l/color/image_raw"
RGB_INFO_TOPIC = "/gemini335l/color/camera_info"
DEPTH_INFO_TOPIC = "/gemini335l/depth/camera_info"
DEPTH_TO_COLOR_TOPIC = "/gemini335l/depth_to_color"
MIN_SCENES = 3
REQUIRED_VIEW_GROUPS = ("front", "left", "right")
MAX_RMSE_3D_M = 0.0065
MAX_RMSE_PIXEL = 1.5
MAX_RAW_LIDAR_MODEL_RMSE_M = 0.012
MAX_RAW_LIDAR_MODEL_ERROR_M = 0.020

# 在不能改变俯仰角的机械结构上，用偏航角和位置/距离变化补足几何多样性。
# 第一轮三组即可求解；若门限未通过，再按顺序补第二轮，而不是一次采很多重复数据。
GUIDED_VIEWS = (
    ("front_01", "正视：板面朝向传感器，中心放在共同视场中央，距离约 2.0～2.3 m。"),
    ("left_01", "左视：保持板竖直，绕竖直轴向一侧转 15°～20°；中心和距离尽量不变。"),
    ("right_01", "右视：保持板竖直，向与上一组相反方向转 15°～20°；中心和距离尽量不变。"),
    ("front_02", "补采正视：仍然正视，但将板前后移动约 0.3 m，并横向错开约 0.15 m。"),
    ("left_02", "补采左视：偏航增至约 25°，距离与 left_01 相差约 0.3 m，四圆孔必须完整可见。"),
    ("right_02", "补采右视：与 left_02 反向约 25°，距离做对应变化，四圆孔必须完整可见。"),
)


class UserError(RuntimeError):
    pass


def run(cmd, check=True, timeout=None, capture=True):
    try:
        result = subprocess.run(
            [str(x) for x in cmd],
            text=True,
            stdout=subprocess.PIPE if capture else None,
            stderr=subprocess.STDOUT if capture else None,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as exc:
        raise UserError("Command timed out after %s s: %s" %
                        (timeout, " ".join(map(str, cmd)))) from exc
    if check and result.returncode != 0:
        output = result.stdout or ""
        raise UserError("Command failed (%d): %s\n%s" %
                        (result.returncode, " ".join(map(str, cmd)), output[-5000:]))
    return result


def topic_yaml(topic, timeout=12):
    try:
        result = run(["timeout", str(timeout), "rostopic", "echo", "-n", "1", topic],
                     timeout=timeout + 3)
    except (subprocess.TimeoutExpired, UserError) as exc:
        raise UserError("无法读取话题 %s；请先运行 Version5/start_slam1，并确认 Gemini 与雷达在线。\n%s" %
                        (topic, exc))
    try:
        docs = list(yaml.safe_load_all(result.stdout))
        return next(x for x in docs if isinstance(x, dict))
    except Exception as exc:
        raise UserError("话题 %s 的 YAML 无法解析: %s" % (topic, exc))


def require_topic(topic):
    result = run(["rostopic", "type", topic], check=False, timeout=5)
    if result.returncode != 0 or not (result.stdout or "").strip():
        raise UserError("找不到话题 %s；请确认对应传感器已经启动。" % topic)
    return result.stdout.strip()


def first_key(data, names):
    """Recursively find one named field in a rostopic YAML document."""
    if isinstance(data, dict):
        for name in names:
            if name in data:
                value = data[name]
                if isinstance(value, dict) and "data" in value:
                    value = value["data"]
                return value
        for value in data.values():
            found = first_key(value, names)
            if found is not None:
                return found
    elif isinstance(data, list):
        for value in data:
            found = first_key(value, names)
            if found is not None:
                return found
    return None


def normalize_camera_info(raw, name):
    K = first_key(raw, ("K", "k"))
    D = first_key(raw, ("D", "d"))
    width = first_key(raw, ("width",))
    height = first_key(raw, ("height",))
    if K is None or D is None or len(K) != 9:
        raise UserError("%s CameraInfo 缺少有效 K/D。" % name)
    d = [float(x) for x in D]
    d += [0.0] * (8 - len(d))
    return {
        "name": name,
        "width": int(width),
        "height": int(height),
        "distortion_model": str(first_key(raw, ("distortion_model",)) or "unknown"),
        "K": [float(x) for x in K],
        "D": d[:8],
    }


def normalize_depth_to_color(raw):
    rotation = first_key(raw, ("rotation", "R"))
    translation = first_key(raw, ("translation", "T", "t"))
    if rotation is None or translation is None:
        raise UserError("Depth→RGB 外参消息中没有 rotation/translation 字段。")
    rotation = [float(x) for x in rotation]
    translation = [float(x) for x in translation]
    if len(rotation) != 9 or len(translation) != 3:
        raise UserError("Depth→RGB 外参维度错误：R=%d, t=%d。" %
                        (len(rotation), len(translation)))
    return {
        "convention": "p_rgb = R_rgb_depth * p_depth + t_rgb_depth",
        "R_rgb_depth": rotation,
        "t_rgb_depth_m": translation,
    }


def now_text():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def save_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(path)


def load_session():
    if not CURRENT.exists():
        raise UserError("还没有当前标定会话，请先运行：gemini_lidar_calib.sh new")
    session_dir = Path(CURRENT.read_text(encoding="utf-8").strip())
    state_path = session_dir / "session.json"
    if not state_path.exists():
        raise UserError("当前会话状态文件不存在：%s" % state_path)
    state = json.loads(state_path.read_text(encoding="utf-8"))
    # 从早期“至少六个任意姿态”规则迁移为“至少三个且覆盖正/左/右”。
    if state.get("schema", 1) < 2:
        state["schema"] = 2
        state.setdefault("quality_gate", {})["minimum_enabled_scenes"] = MIN_SCENES
        state["quality_gate"]["required_view_groups"] = list(REQUIRED_VIEW_GROUPS)
        save_json(state_path, state)
    return session_dir, state


def update_session(session_dir, state):
    state["updated_at"] = now_text()
    save_json(session_dir / "session.json", state)


def command_new(args):
    # Reading these five messages also proves that both sensors and the factory
    # RGB/depth relationship are available before a session can begin.
    run(["rosnode", "list"], timeout=5)
    rgb = normalize_camera_info(topic_yaml(RGB_INFO_TOPIC), "Gemini335L RGB")
    depth = normalize_camera_info(topic_yaml(DEPTH_INFO_TOPIC), "Gemini335L Depth")
    d2c = normalize_depth_to_color(topic_yaml(DEPTH_TO_COLOR_TOPIC))
    # Do not echo the image/point-cloud body (it is very large); type lookup is
    # sufficient here because CameraInfo and factory extrinsics were read above.
    require_topic(LIDAR_TOPIC)
    require_topic(RGB_TOPIC)

    session_dir = ROOT / datetime.now().strftime("%Y%m%d_%H%M%S")
    for child in ("data", "scenes", "result"):
        (session_dir / child).mkdir(parents=True, exist_ok=True)

    target = {
        "type": "FAST-Calib 4x ArUco DICT_6X6_250 + 4x circular holes",
        "marker_ids": [1, 2, 4, 3],
        "marker_size_m": args.marker_size,
        "half_marker_center_width_m": args.qr_half_width,
        "half_marker_center_height_m": args.qr_half_height,
        "circle_center_width_m": args.circle_width,
        "circle_center_height_m": args.circle_height,
        "circle_radius_m": args.circle_radius,
    }
    state = {
        "schema": 2,
        "created_at": now_text(),
        "updated_at": now_text(),
        "topics": {
            "lidar": LIDAR_TOPIC,
            "rgb_image": RGB_TOPIC,
            "rgb_info": RGB_INFO_TOPIC,
            "depth_info": DEPTH_INFO_TOPIC,
            "depth_to_color": DEPTH_TO_COLOR_TOPIC,
        },
        "rgb_camera_info": rgb,
        "depth_camera_info": depth,
        "depth_to_rgb_factory": d2c,
        "target": target,
        "quality_gate": {
            "minimum_enabled_scenes": MIN_SCENES,
            "required_view_groups": list(REQUIRED_VIEW_GROUPS),
            "three_d_rmse_less_than_m": MAX_RMSE_3D_M,
            "pixel_rmse_less_than_px": MAX_RMSE_PIXEL,
        },
        "scenes": [],
    }
    update_session(session_dir, state)
    ROOT.mkdir(parents=True, exist_ok=True)
    CURRENT.write_text(str(session_dir) + "\n", encoding="utf-8")
    (session_dir / "相机与深度出厂参数.yaml").write_text(
        yaml.safe_dump({"rgb": rgb, "depth": depth, "depth_to_rgb": d2c},
                       sort_keys=False, allow_unicode=True), encoding="utf-8")
    print("[新会话] %s" % session_dir)
    print("[RGB] %dx%d, fx=%.6f fy=%.6f cx=%.6f cy=%.6f" %
          (rgb["width"], rgb["height"], rgb["K"][0], rgb["K"][4],
           rgb["K"][2], rgb["K"][5]))
    print("[标定板] ArUco中心半宽/半高=%.3f/%.3f m；圆心宽/高=%.3f/%.3f m；圆半径=%.3f m" %
          (args.qr_half_width, args.qr_half_height, args.circle_width,
           args.circle_height, args.circle_radius))
    print("请确认上述尺寸与实体板完全一致，然后从 front 开始采集。")


def make_scene_config(state, bag, image, output, roi):
    K = state["rgb_camera_info"]["K"]
    D = state["rgb_camera_info"]["D"]
    t = state["target"]
    return {
        "fx": K[0], "fy": K[4], "cx": K[2], "cy": K[5],
        "k1": D[0], "k2": D[1], "p1": D[2], "p2": D[3],
        "k3": D[4], "k4": D[5], "k5": D[6], "k6": D[7],
        "marker_size": t["marker_size_m"],
        "delta_width_qr_center": t["half_marker_center_width_m"],
        "delta_height_qr_center": t["half_marker_center_height_m"],
        "delta_width_circles": t["circle_center_width_m"],
        "delta_height_circles": t["circle_center_height_m"],
        "circle_radius": t["circle_radius_m"],
        "min_detected_markers": 4,
        "x_min": roi[0], "x_max": roi[1],
        "y_min": roi[2], "y_max": roi[3],
        "z_min": roi[4], "z_max": roi[5],
        "lidar_topic": state["topics"]["lidar"],
        "bag_path": str(bag), "image_path": str(image),
        "output_path": str(output), "run_once": True,
    }


POINT_RE = re.compile(r"\{\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\}")
RAW_LIDAR_FIT_RE = re.compile(
    r"Raw LiDAR target-model fit:\s*RMSE=([-+0-9.eE]+)\s*m\s*max=([-+0-9.eE]+)\s*m"
)


def parse_record(path):
    if not path.exists():
        raise UserError("FAST-Calib 未生成四圆心记录：%s" % path)
    lidar = camera = None
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        points = [[float(a), float(b), float(c)] for a, b, c in POINT_RE.findall(line)]
        if line.startswith("lidar_centers:"):
            lidar = points
        elif line.startswith("qr_centers:"):
            camera = points
    if lidar is None or camera is None or len(lidar) != 4 or len(camera) != 4:
        raise UserError("圆心记录不完整（LiDAR 和 RGB 都必须正好为 4 点）：%s" % path)
    return np.asarray(lidar, dtype=float), np.asarray(camera, dtype=float)


def parse_raw_lidar_fit(log_text):
    matches = RAW_LIDAR_FIT_RE.findall(log_text or "")
    if not matches:
        raise UserError("FAST-Calib 日志缺少 LiDAR 原始圆心刚性模型残差，拒绝登记场景。")
    rmse_m, max_m = map(float, matches[-1])
    if (rmse_m > MAX_RAW_LIDAR_MODEL_RMSE_M or
            max_m > MAX_RAW_LIDAR_MODEL_ERROR_M):
        raise UserError(
            "LiDAR 原始圆心模型残差超限：RMSE %.2f mm / max %.2f mm，门限为 %.1f / %.1f mm。" %
            (rmse_m * 1000.0, max_m * 1000.0,
             MAX_RAW_LIDAR_MODEL_RMSE_M * 1000.0,
             MAX_RAW_LIDAR_MODEL_ERROR_M * 1000.0))
    return rmse_m, max_m


def register_scene(session_dir, state, label, stem, bag, image, scene_dir, roi,
                   calib_output):
    lidar, camera = parse_record(scene_dir / "circle_center_record.txt")
    raw_rmse_m, raw_max_m = parse_raw_lidar_fit(calib_output)
    R, t = rigid_fit(lidar, camera)
    residual = np.linalg.norm((R @ lidar.T).T + t - camera, axis=1)
    scene = {
        "label": label,
        "stem": stem,
        "enabled": True,
        "captured_at": now_text(),
        "bag": str(bag), "image": str(image), "directory": str(scene_dir),
        "roi": roi,
        "raw_lidar_model_rmse_mm": raw_rmse_m * 1000.0,
        "raw_lidar_model_max_mm": raw_max_m * 1000.0,
        "single_scene_rmse_mm": float(np.sqrt(np.mean(residual ** 2)) * 1000.0),
    }
    state["scenes"].append(scene)
    update_session(session_dir, state)
    print("[成功] %s：LiDAR 原始模型 RMSE/max = %.3f/%.3f mm；单场景原始圆心 3D RMSE = %.3f mm" %
          (label, scene["raw_lidar_model_rmse_mm"],
           scene["raw_lidar_model_max_mm"], scene["single_scene_rmse_mm"]))
    print("检测图：%s" % (scene_dir / "qr_detect.png"))
    print("当前有效场景：%d；至少需要 %d 个不同角度。" %
          (sum(x["enabled"] for x in state["scenes"]), MIN_SCENES))
    print_next_guidance(state)
    return scene


def archive_failed_attempt(session_dir, stem, bag, image, scene_dir):
    """Move an unregistered partial attempt aside so the same label is retryable."""
    artifacts = [path for path in (bag, image, scene_dir) if path.exists()]
    if not artifacts:
        return None
    failed_dir = (session_dir / "failed_attempts" /
                  (stem + "_" + datetime.now().strftime("%Y%m%d_%H%M%S")))
    failed_dir.mkdir(parents=True, exist_ok=False)
    for path in artifacts:
        destination = failed_dir / ("scene" if path == scene_dir else path.name)
        shutil.move(str(path), str(destination))
    print("[重试] 已将上次未成功登记的数据移到：%s" % failed_dir)
    return failed_dir


def view_group(label):
    name = label.lower()
    for group in REQUIRED_VIEW_GROUPS:
        if name.startswith(group):
            return group
    return None


def next_guided_view(state):
    existing = {x["label"] for x in state.get("scenes", []) if x.get("enabled", True)}
    for label, instruction in GUIDED_VIEWS:
        if label not in existing:
            return label, instruction
    round_index = 1 + max(2, len(existing) // 3)
    counts = {g: sum(view_group(x["label"]) == g for x in state.get("scenes", [])
                     if x.get("enabled", True)) for g in REQUIRED_VIEW_GROUPS}
    group = min(REQUIRED_VIEW_GROUPS, key=lambda g: counts[g])
    return "%s_%02d" % (group, round_index), (
        "继续补采%s视：换一个尚未使用的距离/横向位置，偏航控制在 15°～25°，不要重复原姿态。" %
        {"front": "正", "left": "左", "right": "右"}[group])


def print_next_guidance(state):
    label, instruction = next_guided_view(state)
    print("\n[下一步摆板] %s" % instruction)
    print("摆稳后执行：bash /home/nuc/Desktop/gemini_lidar_calib.sh capture %s" % label)


def command_capture(args):
    session_dir, state = load_session()
    label = re.sub(r"[^A-Za-z0-9_-]+", "_", args.label).strip("_")
    if not label:
        raise UserError("场景标签不能为空。")
    if any(x["label"] == label for x in state["scenes"]):
        raise UserError("标签 %s 已存在；请换一个名称，或先 disable 后使用新标签重采。" % label)

    index = len(state["scenes"]) + 1
    stem = "%02d_%s" % (index, label)
    bag = session_dir / "data" / (stem + ".bag")
    image = session_dir / "data" / (stem + ".jpg")
    scene_dir = session_dir / "scenes" / stem
    # A detector failure occurs after bag/JPG creation.  Preserve that failed
    # attempt for diagnosis, but let the operator reuse the same logical label.
    archive_failed_attempt(session_dir, stem, bag, image, scene_dir)
    scene_dir.mkdir(parents=True)

    print("[采集] %s：标定板和设备必须保持完全静止 %.1f 秒。" % (label, args.duration))
    result = run([
        sys.executable, CAPTURE,
        "--lidar-topic", state["topics"]["lidar"],
        "--image-topic", state["topics"]["rgb_image"],
        "--duration", str(args.duration),
        "--name", stem,
        "--output-dir", session_dir / "data",
    ], check=False, timeout=max(30, int(args.duration + 25)))
    (scene_dir / "capture.log").write_text(result.stdout or "", encoding="utf-8")
    if result.returncode != 0 or not bag.exists() or not image.exists():
        raise UserError("采集失败：\n%s" % (result.stdout or "")[-4000:])

    roi = [args.x_min, args.x_max, args.y_min, args.y_max, args.z_min, args.z_max]
    config = make_scene_config(state, bag, image, scene_dir, roi)
    config_path = scene_dir / "scene.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    namespace = "/gemini_lidar_calib_%s_%02d" % (os.getpid(), index)
    run(["rosparam", "load", config_path, namespace])
    calib = run(["rosrun", "fast_calib", "fast_calib", "__ns:=" + namespace],
                check=False, timeout=180)
    (scene_dir / "fast_calib.log").write_text(calib.stdout or "", encoding="utf-8")
    run(["rosparam", "delete", namespace], check=False)
    if calib.returncode != 0:
        raise UserError("场景 %s 检测失败，未加入多角度求解。请检查标定板是否完整可见，并收紧 ROI。\n%s" %
                        (label, (calib.stdout or "")[-5000:]))

    register_scene(session_dir, state, label, stem, bag, image, scene_dir, roi,
                   calib.stdout or "")


def command_reprocess(args):
    """Re-run detection for an existing, unregistered bag/JPG without recapture."""
    session_dir, state = load_session()
    label = re.sub(r"[^A-Za-z0-9_-]+", "_", args.label).strip("_")
    registered = [x for x in state["scenes"] if x["label"] == label]
    if registered and not args.replace:
        raise UserError("标签 %s 已登记；若需用当前检测器重算，请添加 --replace。" % label)
    if registered:
        pairs = [(Path(registered[0]["bag"]), Path(registered[0]["image"]))]
        state["scenes"] = [x for x in state["scenes"] if x["label"] != label]
    else:
        candidates = sorted((session_dir / "data").glob("*_%s.bag" % label))
        pairs = [(bag, bag.with_suffix(".jpg")) for bag in candidates
                 if bag.with_suffix(".jpg").exists()]
    if len(pairs) != 1:
        raise UserError("需要恰好一组未登记的 %s bag/JPG，当前找到 %d 组。" %
                        (label, len(pairs)))
    bag, image = pairs[0]
    stem = bag.stem
    scene_dir = session_dir / "scenes" / stem
    scene_dir.mkdir(parents=True, exist_ok=True)
    roi = [args.x_min, args.x_max, args.y_min, args.y_max, args.z_min, args.z_max]
    config = make_scene_config(state, bag, image, scene_dir, roi)
    config_path = scene_dir / "scene.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    namespace = "/gemini_lidar_reprocess_%s" % os.getpid()
    run(["rosparam", "load", config_path, namespace])
    calib = run(["rosrun", "fast_calib", "fast_calib", "__ns:=" + namespace],
                check=False, timeout=180)
    (scene_dir / "fast_calib.log").write_text(calib.stdout or "", encoding="utf-8")
    run(["rosparam", "delete", namespace], check=False)
    if calib.returncode != 0:
        raise UserError("已有数据重新检测失败，未登记：\n%s" %
                        (calib.stdout or "")[-5000:])
    register_scene(session_dir, state, label, stem, bag, image, scene_dir, roi,
                   calib.stdout or "")


def rigid_fit(source, target):
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise UserError("刚体拟合输入维度错误。")
    cs, ct = source.mean(axis=0), target.mean(axis=0)
    U, _, Vt = np.linalg.svd((source - cs).T @ (target - ct))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = ct - R @ cs
    return R, t


def project(points, camera_info):
    K = np.asarray(camera_info["K"], dtype=float).reshape(3, 3)
    D = np.asarray(camera_info["D"], dtype=float).reshape(-1, 1)
    uv, _ = cv2.projectPoints(np.asarray(points, dtype=float).reshape(-1, 1, 3),
                              np.zeros(3), np.zeros(3), K, D)
    return uv.reshape(-1, 2)


def error_stats(values):
    values = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values ** 2))),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def rotation_difference_deg(Ra, Rb):
    value = np.clip((np.trace(Ra @ Rb.T) - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(value))


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, list):
        return [plain(v) for v in value]
    return value


def write_transform_yaml(path, frame_to, R, t, extra):
    doc = {
        "accepted": True,
        "convention": "p_%s = R_%s_lidar * p_lidar + t_%s_lidar" %
                      (frame_to, frame_to, frame_to),
        "R_%s_lidar" % frame_to: np.asarray(R).reshape(3, 3).tolist(),
        "t_%s_lidar_m" % frame_to: np.asarray(t).reshape(3).tolist(),
        # FAST-LIVO naming used by this repository.
        "Rcl": np.asarray(R).reshape(-1).tolist(),
        "Pcl": np.asarray(t).reshape(3).tolist(),
    }
    doc.update(extra)
    path.write_text(yaml.safe_dump(plain(doc), sort_keys=False, allow_unicode=True),
                    encoding="utf-8")


def command_solve(_args):
    session_dir, state = load_session()
    scenes = [x for x in state["scenes"] if x.get("enabled", True)]
    if len(scenes) < MIN_SCENES:
        raise UserError("有效场景只有 %d 个，至少需要 %d 个（正视、左视、右视各一组）。" %
                        (len(scenes), MIN_SCENES))
    groups = {view_group(x["label"]) for x in scenes}
    missing_groups = [x for x in REQUIRED_VIEW_GROUPS if x not in groups]
    if missing_groups:
        raise UserError("场景数量够，但缺少视角组：%s。标签必须以 front/left/right 开头。" %
                        ", ".join(missing_groups))

    lidar_all, camera_all, labels = [], [], []
    scene_points = []
    for scene in scenes:
        lidar, camera = parse_record(Path(scene["directory"]) / "circle_center_record.txt")
        lidar_all.append(lidar)
        camera_all.append(camera)
        labels.extend([scene["label"]] * 4)
        scene_points.append((scene, lidar, camera))
    lidar_all = np.vstack(lidar_all)
    camera_all = np.vstack(camera_all)
    R, t = rigid_fit(lidar_all, camera_all)
    predicted = (R @ lidar_all.T).T + t
    errors_3d_m = np.linalg.norm(predicted - camera_all, axis=1)
    uv_lidar = project(predicted, state["rgb_camera_info"])
    uv_camera = project(camera_all, state["rgb_camera_info"])
    errors_px = np.linalg.norm(uv_lidar - uv_camera, axis=1)

    per_scene = []
    for scene, lidar, camera in scene_points:
        pred = (R @ lidar.T).T + t
        e3 = np.linalg.norm(pred - camera, axis=1)
        ep = np.linalg.norm(project(pred, state["rgb_camera_info"]) -
                            project(camera, state["rgb_camera_info"]), axis=1)
        per_scene.append({
            "label": scene["label"],
            "raw_lidar_model_rmse_mm": scene.get("raw_lidar_model_rmse_mm"),
            "raw_lidar_model_max_mm": scene.get("raw_lidar_model_max_mm"),
            "rmse_3d_mm": float(np.sqrt(np.mean(e3 ** 2)) * 1000.0),
            "rmse_pixel": float(np.sqrt(np.mean(ep ** 2))),
            "max_3d_mm": float(np.max(e3) * 1000.0),
            "max_pixel": float(np.max(ep)),
        })
    per_scene.sort(key=lambda x: x["rmse_3d_mm"], reverse=True)

    # Leave-one-scene-out stability exposes weak pose diversity without
    # automatically deleting a bad scene and making the final number look good.
    loo = []
    for i, (scene, _, _) in enumerate(scene_points):
        keep = np.asarray([j // 4 != i for j in range(len(lidar_all))])
        Ri, ti = rigid_fit(lidar_all[keep], camera_all[keep])
        loo.append({
            "excluded": scene["label"],
            "rotation_change_deg": rotation_difference_deg(Ri, R),
            "translation_change_mm": float(np.linalg.norm(ti - t) * 1000.0),
        })

    stats_3d = {k + "_mm": v * 1000.0 for k, v in error_stats(errors_3d_m).items()}
    stats_px = {k + "_px": v for k, v in error_stats(errors_px).items()}
    passed_3d = stats_3d["rmse_mm"] < MAX_RMSE_3D_M * 1000.0
    passed_px = stats_px["rmse_px"] < MAX_RMSE_PIXEL
    passed_raw = all(
        x.get("raw_lidar_model_rmse_mm") is not None and
        x["raw_lidar_model_rmse_mm"] <= MAX_RAW_LIDAR_MODEL_RMSE_M * 1000.0 and
        x.get("raw_lidar_model_max_mm") is not None and
        x["raw_lidar_model_max_mm"] <= MAX_RAW_LIDAR_MODEL_ERROR_M * 1000.0
        for x in scenes)
    accepted = bool(passed_raw and passed_3d and passed_px)

    report = {
        "session": str(session_dir),
        "solved_at": now_text(),
        "enabled_scene_count": len(scenes),
        "point_pair_count": len(lidar_all),
        "view_groups": sorted(groups),
        "accepted": accepted,
        "quality_gate": {
            "raw_lidar_target_model": {
                "rmse_limit_mm": MAX_RAW_LIDAR_MODEL_RMSE_M * 1000.0,
                "max_limit_mm": MAX_RAW_LIDAR_MODEL_ERROR_M * 1000.0,
                "passed": passed_raw,
            },
            "three_d_rmse": {"value_mm": stats_3d["rmse_mm"], "limit_mm": 6.5,
                             "strict_less_than": True, "passed": passed_3d},
            "pixel_rmse": {"value_px": stats_px["rmse_px"], "limit_px": 1.5,
                           "strict_less_than": True, "passed": passed_px},
        },
        "three_d_error": stats_3d,
        "pixel_reprojection_error": stats_px,
        "R_rgb_lidar": R,
        "t_rgb_lidar_m": t,
        "per_scene_sorted_worst_first": per_scene,
        "leave_one_scene_out": loo,
    }
    result_dir = session_dir / "result"
    result_dir.mkdir(exist_ok=True)
    for stale in ("ACCEPTED", "accepted_lidar_to_rgb.yaml", "accepted_lidar_to_depth.yaml"):
        path = result_dir / stale
        if path.exists():
            path.unlink()
    save_json(result_dir / "quality_report.json", plain(report))

    lines = [
        "Gemini 335L RGB - LiDAR 多角度标定质量报告",
        "会话: %s" % session_dir,
        "场景/点对: %d / %d" % (len(scenes), len(lidar_all)),
        "LiDAR原始圆心模型门限 RMSE/max <= %.1f/%.1f mm：%s" %
        (MAX_RAW_LIDAR_MODEL_RMSE_M * 1000.0,
         MAX_RAW_LIDAR_MODEL_ERROR_M * 1000.0, "通过" if passed_raw else "未通过"),
        "3D RMSE: %.3f mm，门限 < 6.500 mm：%s" %
        (stats_3d["rmse_mm"], "通过" if passed_3d else "未通过"),
        "2D重投影 RMSE: %.3f px，门限 < 1.500 px：%s" %
        (stats_px["rmse_px"], "通过" if passed_px else "未通过"),
        "最终结论: %s" % ("可用于标定" if accepted else "不可用于标定，不导出可用外参"),
        "",
        "各场景（最差在前）:",
    ]
    lines += ["  %-20s raw %.3f/%.3f mm | 3D %.3f mm | pixel %.3f px | max %.3f mm / %.3f px" %
              (x["label"], x["raw_lidar_model_rmse_mm"],
               x["raw_lidar_model_max_mm"], x["rmse_3d_mm"], x["rmse_pixel"],
               x["max_3d_mm"], x["max_pixel"]) for x in per_scene]
    lines += ["", "R_rgb_lidar:", np.array2string(R, precision=9, suppress_small=False),
              "t_rgb_lidar_m: " + np.array2string(t, precision=9), ""]
    (result_dir / "quality_report.txt").write_text("\n".join(lines), encoding="utf-8")

    # A candidate is always written for diagnostics, but only a passing result
    # receives the accepted filenames and marker.
    candidate = {
        "accepted": accepted,
        "R_rgb_lidar": R.tolist(), "t_rgb_lidar_m": t.tolist(),
        "quality_gate": report["quality_gate"],
    }
    (result_dir / "candidate_lidar_to_rgb.yaml").write_text(
        yaml.safe_dump(candidate, sort_keys=False), encoding="utf-8")

    if accepted:
        extra = {"quality": report["quality_gate"],
                 "rgb_camera_info": state["rgb_camera_info"]}
        write_transform_yaml(result_dir / "accepted_lidar_to_rgb.yaml", "rgb", R, t, extra)
        d2c = state["depth_to_rgb_factory"]
        R_cd = np.asarray(d2c["R_rgb_depth"], dtype=float).reshape(3, 3)
        t_cd = np.asarray(d2c["t_rgb_depth_m"], dtype=float)
        R_dl = R_cd.T @ R
        t_dl = R_cd.T @ (t - t_cd)
        write_transform_yaml(
            result_dir / "accepted_lidar_to_depth.yaml", "depth", R_dl, t_dl,
            {"derived_from": "accepted LiDAR->RGB and Gemini factory Depth->RGB",
             "depth_to_rgb_factory": d2c, "quality_in_rgb_frame": report["quality_gate"]})
        (result_dir / "ACCEPTED").write_text(
            "3D RMSE %.3f mm < 6.5 mm; pixel RMSE %.3f px < 1.5 px\n" %
            (stats_3d["rmse_mm"], stats_px["rmse_px"]), encoding="utf-8")

    print("\n".join(lines[:8]))
    print("报告：%s" % (result_dir / "quality_report.txt"))
    if accepted:
        print("RGB外参：%s" % (result_dir / "accepted_lidar_to_rgb.yaml"))
        print("Depth外参：%s" % (result_dir / "accepted_lidar_to_depth.yaml"))
        return 0
    print("最差场景：%s" % ", ".join(x["label"] for x in per_scene[:3]))
    print_next_guidance(state)
    return 2


def command_status(_args):
    session_dir, state = load_session()
    print("会话：%s" % session_dir)
    print("创建：%s；更新：%s" % (state["created_at"], state["updated_at"]))
    print("场景：")
    if not state["scenes"]:
        print("  （尚未采集）")
    for scene in state["scenes"]:
        print("  [%s] %-20s LiDAR模型RMSE/max %.3f/%.3f mm | 单场景3D RMSE %.3f mm  %s" %
              ("启用" if scene.get("enabled", True) else "禁用",
               scene["label"], scene.get("raw_lidar_model_rmse_mm", float("nan")),
               scene.get("raw_lidar_model_max_mm", float("nan")),
               scene["single_scene_rmse_mm"], scene["directory"]))
    print("有效 %d / 最少 %d；必须包含 front、left、right。先采三组求解，不通过再补第二轮。" %
          (sum(x.get("enabled", True) for x in state["scenes"]), MIN_SCENES))
    print_next_guidance(state)


def command_next(_args):
    _session_dir, state = load_session()
    print_next_guidance(state)


def command_toggle(args, enabled):
    session_dir, state = load_session()
    matches = [x for x in state["scenes"] if x["label"] == args.label]
    if not matches:
        raise UserError("找不到场景：%s" % args.label)
    matches[0]["enabled"] = enabled
    update_session(session_dir, state)
    print("场景 %s 已%s；原始数据未删除。" % (args.label, "启用" if enabled else "禁用"))


def command_self_test(_args):
    rng = np.random.default_rng(335)
    source = rng.normal(size=(40, 3))
    rvec = np.array([0.03, -0.12, 0.06])
    R_true, _ = cv2.Rodrigues(rvec)
    t_true = np.array([0.04, -0.02, 0.08])
    target = (R_true @ source.T).T + t_true
    R, t = rigid_fit(source, target)
    assert rotation_difference_deg(R, R_true) < 1e-5
    assert np.linalg.norm(t - t_true) < 1e-10
    print("SELF-TEST PASS")


def build_parser():
    p = argparse.ArgumentParser(description="Gemini 335L RGB/LiDAR 多角度严格质量门限标定")
    sub = p.add_subparsers(dest="command", required=True)
    new = sub.add_parser("new", help="读取在线相机参数并新建会话")
    new.add_argument("--marker-size", type=float, default=0.20)
    new.add_argument("--qr-half-width", type=float, default=0.55)
    new.add_argument("--qr-half-height", type=float, default=0.35)
    new.add_argument("--circle-width", type=float, default=0.50)
    new.add_argument("--circle-height", type=float, default=0.40)
    new.add_argument("--circle-radius", type=float, default=0.12)
    new.set_defaults(func=command_new)

    cap = sub.add_parser("capture", help="采集并检测一个静止角度")
    cap.add_argument("label", help="唯一场景名，例如 front/left15/right15/up15")
    cap.add_argument("--duration", type=float, default=5.0)
    cap.add_argument("--x-min", type=float, default=1.2)
    cap.add_argument("--x-max", type=float, default=3.5)
    cap.add_argument("--y-min", type=float, default=-1.8)
    cap.add_argument("--y-max", type=float, default=1.8)
    cap.add_argument("--z-min", type=float, default=0.0)
    # The current room ceiling appears at z≈1.8–2.0 m in MID360 coordinates
    # and used to win plane RANSAC.  1.2 m retains the calibration board while
    # excluding that ceiling; users can still override this at the command line.
    cap.add_argument("--z-max", type=float, default=1.2)
    cap.set_defaults(func=command_capture)

    retry = sub.add_parser("reprocess", help="重新检测现有未登记 bag/JPG，不重复采集")
    retry.add_argument("label", help="现有数据的场景名，例如 front_01")
    retry.add_argument("--replace", action="store_true",
                       help="替换同名已登记场景；仅在新检测成功后写回会话")
    retry.add_argument("--x-min", type=float, default=1.2)
    retry.add_argument("--x-max", type=float, default=3.5)
    retry.add_argument("--y-min", type=float, default=-1.8)
    retry.add_argument("--y-max", type=float, default=1.8)
    retry.add_argument("--z-min", type=float, default=0.0)
    retry.add_argument("--z-max", type=float, default=1.2)
    retry.set_defaults(func=command_reprocess)

    solve = sub.add_parser("solve", help="多角度联合求解并执行质量门限")
    solve.set_defaults(func=command_solve)
    status = sub.add_parser("status", help="查看当前会话和场景")
    status.set_defaults(func=command_status)
    next_view = sub.add_parser("next", help="提示下一组标定板摆放方式和采集指令")
    next_view.set_defaults(func=command_next)
    disable = sub.add_parser("disable", help="禁用坏场景但保留原始文件")
    disable.add_argument("label")
    disable.set_defaults(func=lambda a: command_toggle(a, False))
    enable = sub.add_parser("enable", help="重新启用场景")
    enable.add_argument("label")
    enable.set_defaults(func=lambda a: command_toggle(a, True))
    test = sub.add_parser("self-test", help=argparse.SUPPRESS)
    test.set_defaults(func=command_self_test)
    return p


def main():
    try:
        args = build_parser().parse_args()
        result = args.func(args)
        return int(result or 0)
    except UserError as exc:
        print("[错误] %s" % exc, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n[中止] 用户停止操作；已完成的原始文件会保留。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
