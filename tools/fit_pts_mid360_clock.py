#!/usr/bin/env python3
"""Offline PTS -> MID360 affine calibration and frozen-map validation.

The input is FAST_LIVO_TIME_DIAG_CSV written by mvs_ros_driver/uvc_driver.
No parameter is estimated or changed in the camera process.  This program is
the only place that fits CameraClockScale and CameraClockOffsetNs.
"""

import argparse
import csv
import json
import math
import re
from pathlib import Path


PTS_CLOCK_HZ = 48_000_000.0


def nanoseconds_as_decimal_seconds(value):
    sign = "-" if value < 0 else ""
    whole, fraction = divmod(abs(value), 1_000_000_000)
    return f"{sign}{whole}.{fraction:09d}"


def percentile(values, probability):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def centered_line_fit(xs, ys):
    """Fit y = slope*x + intercept with centered arithmetic."""
    if len(xs) < 2 or len(xs) != len(ys):
        raise ValueError("at least two paired samples are required")
    x_mean = math.fsum(xs) / len(xs)
    y_mean = math.fsum(ys) / len(ys)
    denominator = math.fsum((x - x_mean) ** 2 for x in xs)
    if denominator <= 0.0:
        raise ValueError("paired PTS times have zero span")
    slope = math.fsum(
        (x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)
    ) / denominator
    intercept = y_mean - slope * x_mean
    return slope, intercept


def integer(row, key, default=0):
    value = row.get(key, "")
    if value in (None, ""):
        return default
    return int(value)


def floating(row, key, default=0.0):
    value = row.get(key, "")
    if value in (None, ""):
        return default
    return float(value)


def load_pairs(path):
    rows = []
    rejected = {
        "pts_invalid": 0,
        "mid360_observation_invalid": 0,
        "non_increasing_pts": 0,
        "non_increasing_sequence": 0,
    }
    last_ticks = None
    last_sequence = None
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {
            "frame_seq",
            "pts_valid",
            "pts_raw",
            "pts_unwrapped_ticks",
            "pts_mid360_observation_ns",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError("input CSV is missing columns: " + ", ".join(sorted(missing)))
        for row in reader:
            if integer(row, "pts_valid") != 1:
                rejected["pts_invalid"] += 1
                continue
            observation_ns = integer(row, "pts_mid360_observation_ns")
            if observation_ns <= 0:
                rejected["mid360_observation_invalid"] += 1
                continue
            ticks = integer(row, "pts_unwrapped_ticks")
            sequence = integer(row, "frame_seq")
            if last_ticks is not None and ticks <= last_ticks:
                rejected["non_increasing_pts"] += 1
                continue
            if last_sequence is not None and sequence <= last_sequence:
                rejected["non_increasing_sequence"] += 1
                continue
            rows.append(
                {
                    "frame_seq": sequence,
                    "pts_raw": integer(row, "pts_raw"),
                    "ticks": ticks,
                    "observation_ns": observation_ns,
                    "scr_stc": integer(row, "pts_scr_stc"),
                    "scr_sof": integer(row, "pts_scr_sof"),
                    "host_monotonic_ns": integer(row, "dequeue_monotonic_ns"),
                    "payload_monotonic_ns": integer(row, "pts_payload_monotonic_ns"),
                    "mid360_latest_ns": integer(row, "current_mid360_time_ns"),
                    "match_delta_ns": integer(row, "pts_video_match_delta_ns"),
                    "metadata_duplicate_count": integer(
                        row, "pts_metadata_duplicate_count"
                    ),
                    "metadata_rollback_count": integer(
                        row, "pts_metadata_rollback_count"
                    ),
                    "match_drop_count": integer(row, "pts_match_drop_count"),
                    "recorded_scale": floating(row, "camera_clock_scale", 0.0),
                    "recorded_offset_ns": integer(row, "camera_clock_offset_ns"),
                }
            )
            last_ticks = ticks
            last_sequence = sequence
    return rows, rejected


def discard_warmup(rows, warmup_seconds):
    """Drop only the leading acquisition transient, measured in PTS time."""
    if not rows or warmup_seconds <= 0.0:
        return rows, 0
    first_ticks = rows[0]["ticks"]
    minimum_delta_ticks = warmup_seconds * PTS_CLOCK_HZ
    first_kept = 0
    while (first_kept < len(rows)
           and rows[first_kept]["ticks"] - first_ticks < minimum_delta_ticks):
        first_kept += 1
    return rows[first_kept:], first_kept


def anomaly_stats(rows, mapped_ns):
    pts_duplicates = 0
    pts_rollbacks = 0
    mapped_duplicates = 0
    mapped_rollbacks = 0
    sequence_jumps = 0
    periods_ms = []
    for index in range(1, len(rows)):
        tick_delta = rows[index]["ticks"] - rows[index - 1]["ticks"]
        sequence_delta = rows[index]["frame_seq"] - rows[index - 1]["frame_seq"]
        mapped_delta = mapped_ns[index] - mapped_ns[index - 1]
        if tick_delta == 0:
            pts_duplicates += 1
        elif tick_delta < 0:
            pts_rollbacks += 1
        if mapped_delta == 0:
            mapped_duplicates += 1
        elif mapped_delta < 0:
            mapped_rollbacks += 1
        if sequence_delta != 1:
            sequence_jumps += 1
        if sequence_delta > 0 and mapped_delta > 0:
            periods_ms.append(mapped_delta / sequence_delta / 1e6)
    return {
        # These are anomalies in the accepted, mapped camera PTS stream.
        "pts_duplicate_count": pts_duplicates,
        "pts_rollback_count": pts_rollbacks,
        # The metadata reader rejects auxiliary/stale payloads before matching.
        # Report those counters separately so they cannot be mistaken for
        # duplicate or rollback events in accepted camera timestamps.
        "metadata_filtered_duplicate_count": max(
            (row.get("metadata_duplicate_count", 0) for row in rows), default=0
        ),
        "metadata_filtered_rollback_count": max(
            (row.get("metadata_rollback_count", 0) for row in rows), default=0
        ),
        "pts_match_drop_count": max(
            (row.get("match_drop_count", 0) for row in rows), default=0
        ),
        "mapped_duplicate_count": mapped_duplicates,
        "mapped_rollback_count": mapped_rollbacks,
        "sequence_jump_count": sequence_jumps,
        "mapped_camera_period_ms": summarize(periods_ms, absolute=False),
    }


def summarize(values, absolute=False):
    if not values:
        return {"count": 0, "mean": None, "std": None, "p95": None, "max": None}
    selected = [abs(value) for value in values] if absolute else list(values)
    mean = math.fsum(selected) / len(selected)
    variance = math.fsum((value - mean) ** 2 for value in selected) / len(selected)
    return {
        "count": len(selected),
        "mean": mean,
        "std": math.sqrt(variance),
        "p95": percentile(selected, 0.95),
        "max": max(selected),
    }


def parse_fast_livo_backlog(path):
    if not path:
        return None
    pattern = re.compile(r"buffered_lidar_frames_left=(\d+)")
    values = []
    with Path(path).open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = pattern.search(line)
            if match:
                values.append(float(match.group(1)))
    result = summarize(values, absolute=False)
    result["source"] = str(path)
    return result


def evaluate(rows, scale, offset_ns, exposure_center_offset_ns=0,
             calibration_offset_ns=0):
    mapped_ns = [
        int(round(scale * row["ticks"] * (1e9 / PTS_CLOCK_HZ)))
        + offset_ns + exposure_center_offset_ns + calibration_offset_ns
        for row in rows
    ]
    residual_ms = [
        (mapped - row["observation_ns"]) / 1e6
        for mapped, row in zip(mapped_ns, rows)
    ]
    elapsed_s = [
        (row["ticks"] - rows[0]["ticks"]) / PTS_CLOCK_HZ for row in rows
    ]
    if len(rows) >= 2 and elapsed_s[-1] > 0.0:
        residual_s = [value / 1000.0 for value in residual_ms]
        drift_s_per_s, _ = centered_line_fit(elapsed_s, residual_s)
        drift_ms_per_s = drift_s_per_s * 1000.0
    else:
        drift_ms_per_s = None
    signed = summarize(residual_ms, absolute=False)
    absolute = summarize(residual_ms, absolute=True)
    return mapped_ns, residual_ms, {
        "signed_mean_ms": signed["mean"],
        "signed_std_ms": signed["std"],
        "absolute_p95_ms": absolute["p95"],
        "absolute_max_ms": absolute["max"],
        "drift_ms_per_s": drift_ms_per_s,
    }


def fit(rows):
    # Explicitly use the first pair as the requested numerical origin.
    pts_ticks_0 = rows[0]["ticks"]
    mid360_ns_0 = rows[0]["observation_ns"]
    pts_time_0 = pts_ticks_0 / PTS_CLOCK_HZ
    mid360_time_0 = mid360_ns_0 / 1e9
    xs = [(row["ticks"] - pts_ticks_0) / PTS_CLOCK_HZ for row in rows]
    ys = [(row["observation_ns"] - mid360_ns_0) / 1e9 for row in rows]
    scale, c_seconds = centered_line_fit(xs, ys)
    # Restore b around the integer nanosecond origin.  Never convert the full
    # MID360 epoch to float and multiply it back by 1e9.
    offset_ns = mid360_ns_0 + int(round(
        c_seconds * 1e9 - scale * pts_ticks_0 * (1e9 / PTS_CLOCK_HZ)
    ))
    return scale, offset_ns, {
        "pts_time_0_s": pts_time_0,
        "mid360_time_0_s": mid360_time_0,
        "centered_intercept_c_s": c_seconds,
        "span_s": xs[-1] - xs[0],
    }


def write_pairs(path, rows, mapped_ns, residual_ms):
    fields = [
        "frame_seq", "pts_raw", "pts_unwrapped_ticks", "pts_time_s",
        "scr_stc", "scr_sof", "host_monotonic_ns", "payload_monotonic_ns",
        "mid360_latest_ns", "pts_mid360_observation_ns", "mapped_camera_ns",
        "camera_minus_mid360_residual_ms", "pts_video_match_delta_ns",
    ]
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row, mapped, residual in zip(rows, mapped_ns, residual_ms):
            writer.writerow({
                "frame_seq": row["frame_seq"],
                "pts_raw": row["pts_raw"],
                "pts_unwrapped_ticks": row["ticks"],
                "pts_time_s": f"{row['ticks'] / PTS_CLOCK_HZ:.12f}",
                "scr_stc": row["scr_stc"],
                "scr_sof": row["scr_sof"],
                "host_monotonic_ns": row["host_monotonic_ns"],
                "payload_monotonic_ns": row["payload_monotonic_ns"],
                "mid360_latest_ns": row["mid360_latest_ns"],
                "pts_mid360_observation_ns": row["observation_ns"],
                "mapped_camera_ns": mapped,
                "camera_minus_mid360_residual_ms": f"{residual:.9f}",
                "pts_video_match_delta_ns": row["match_delta_ns"],
            })


def replace_or_add(text, key, value):
    replacement = f'{key}: {value}'
    pattern = re.compile(rf"(?m)^{re.escape(key)}\s*:.*$")
    if pattern.search(text):
        return pattern.sub(replacement, text, count=1)
    if not text.endswith("\n"):
        text += "\n"
    return text + replacement + "\n"


def write_config(path, scale, offset_ns, activate):
    config = Path(path)
    text = config.read_text(encoding="utf-8")
    text = replace_or_add(text, "CameraClockScale", f"{scale:.17g}")
    # Never persist an absolute b from this STREAMON as a startup fallback.
    # The fitted offset remains in the JSON report; runtime re-anchors b_session.
    text = replace_or_add(text, "CameraClockOffsetNs", '"0"')
    text = replace_or_add(text, "CameraClockCalibrationValid", "1")
    text = replace_or_add(text, "ExposureCenterOffsetNs", "0")
    text = replace_or_add(text, "CameraCalibrationOffsetNs", "0")
    if activate:
        text = replace_or_add(
            text, "TimestampSource", '"pts_mid360_session_anchor"'
        )
    config.write_text(text, encoding="utf-8")


def read_config(path):
    text = Path(path).read_text(encoding="utf-8")

    def value(key):
        match = re.search(rf"(?m)^{re.escape(key)}\s*:\s*([^#\r\n]+)", text)
        if not match:
            raise ValueError(f"{key} is missing from {path}")
        return match.group(1).strip().strip('"').strip("'")

    if value("CameraClockCalibrationValid") != "1":
        raise ValueError(f"CameraClockCalibrationValid is not 1 in {path}")
    return float(value("CameraClockScale")), int(value("CameraClockOffsetNs"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv")
    parser.add_argument("--mode", choices=("fit", "validate"), default="fit")
    parser.add_argument("--scale", type=float)
    parser.add_argument("--offset-ns", type=int)
    parser.add_argument("--config", help="read frozen a,b for validation")
    parser.add_argument(
        "--use-recorded-session-map", action="store_true",
        help="validate with the single a,b_session recorded in the CSV",
    )
    parser.add_argument("--label", default="calibration")
    parser.add_argument("--output-json")
    parser.add_argument("--output-pairs")
    parser.add_argument("--write-config")
    parser.add_argument("--activate", action="store_true")
    parser.add_argument("--fast-livo-log")
    parser.add_argument("--min-fit-span-s", type=float, default=300.0)
    parser.add_argument(
        "--warmup-s", type=float, default=0.0,
        help="discard this leading PTS duration before fit/validation",
    )
    args = parser.parse_args()

    rows, rejected = load_pairs(args.input_csv)
    rows, warmup_rejected = discard_warmup(rows, args.warmup_s)
    rejected["warmup"] = warmup_rejected
    if len(rows) < 30:
        raise SystemExit(f"only {len(rows)} valid pairs; at least 30 are required")

    fit_details = None
    if args.mode == "fit":
        scale, offset_ns, fit_details = fit(rows)
        if fit_details["span_s"] < args.min_fit_span_s:
            raise SystemExit(
                f"fit span is {fit_details['span_s']:.3f}s; "
                f"at least {args.min_fit_span_s:.3f}s is required"
            )
    else:
        if args.use_recorded_session_map:
            maps = {
                (row["recorded_scale"], row["recorded_offset_ns"])
                for row in rows
            }
            maps.discard((0.0, 0))
            if len(maps) != 1:
                raise SystemExit(
                    "validation CSV must contain exactly one locked session map; "
                    f"found {len(maps)}"
                )
            scale, offset_ns = maps.pop()
        elif args.config:
            scale, offset_ns = read_config(args.config)
        elif args.scale is not None and args.offset_ns is not None:
            scale, offset_ns = args.scale, args.offset_ns
        else:
            parser.error(
                "--mode validate requires --use-recorded-session-map, "
                "--config, or both --scale and --offset-ns"
            )

    mapped_ns, residual_ms, residual = evaluate(rows, scale, offset_ns)
    result = {
        "label": args.label,
        "mode": args.mode,
        "input_csv": str(Path(args.input_csv)),
        "pair_count": len(rows),
        "rejected": rejected,
        "camera_clock_scale_a": scale,
        "camera_clock_offset_b_seconds": offset_ns / 1e9,
        "camera_clock_offset_b_seconds_decimal": nanoseconds_as_decimal_seconds(
            offset_ns
        ),
        "camera_clock_offset_ns": offset_ns,
        "clock_skew_ppm": (scale - 1.0) * 1e6,
        "exposure_center_offset_ns": 0,
        "camera_calibration_offset_ns": 0,
        "residual_definition": "mapped_camera_time_minus_mid360_observation",
        "residual": residual,
        "anomalies": anomaly_stats(rows, mapped_ns),
        "fast_livo_lidar_backlog": parse_fast_livo_backlog(args.fast_livo_log),
        "fit_centering": fit_details,
    }

    rendered = json.dumps(result, indent=2, ensure_ascii=False)
    print(rendered)
    if args.output_json:
        Path(args.output_json).write_text(rendered + "\n", encoding="utf-8")
    if args.output_pairs:
        write_pairs(args.output_pairs, rows, mapped_ns, residual_ms)
    if args.write_config:
        if args.mode != "fit":
            parser.error("--write-config is only allowed in fit mode")
        write_config(args.write_config, scale, offset_ns, args.activate)


if __name__ == "__main__":
    main()
