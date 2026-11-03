#!/usr/bin/env python3
import argparse
import bisect
import csv
import json
import math
import statistics
from collections import Counter
from pathlib import Path


U32 = 1 << 32
CAMERA_CLOCK_HZ = 48_000_000.0


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def as_int(row, key, default=0):
    try:
        return int(row.get(key, default))
    except (TypeError, ValueError):
        return default


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def summarize(values, duplicates=0, rollbacks=0):
    if not values:
        return {
            "samples": 0, "mean_ms": None, "std_ms": None,
            "min_ms": None, "max_ms": None, "p50_ms": None,
            "p95_ms": None, "frequency_hz": None, "below_97ms": 0,
            "above_103ms": 0, "duplicates": duplicates,
            "rollbacks": rollbacks,
        }
    mean = statistics.fmean(values)
    return {
        "samples": len(values),
        "mean_ms": mean,
        "std_ms": statistics.pstdev(values),
        "min_ms": min(values),
        "max_ms": max(values),
        "p50_ms": percentile(values, 0.50),
        "p95_ms": percentile(values, 0.95),
        "frequency_hz": 1000.0 / mean if mean > 0 else None,
        "below_97ms": sum(value < 97.0 for value in values),
        "above_103ms": sum(value > 103.0 for value in values),
        "duplicates": duplicates,
        "rollbacks": rollbacks,
    }


def period_series(rows, value_key, sequence_key="buf_sequence", scale=1_000_000.0):
    values = []
    by_sequence = {}
    duplicates = 0
    rollbacks = 0
    sequence_jumps = 0
    previous = None
    for row in rows:
        sequence = as_int(row, sequence_key)
        value = as_int(row, value_key)
        if previous is not None:
            sequence_delta = sequence - previous[0]
            value_delta = value - previous[1]
            if sequence_delta != 1:
                sequence_jumps += 1
            if value_delta == 0:
                duplicates += 1
            elif value_delta < 0:
                rollbacks += 1
            elif sequence_delta == 1:
                period_ms = value_delta / scale
                values.append(period_ms)
                by_sequence[sequence] = period_ms
        previous = (sequence, value)
    return values, by_sequence, duplicates, rollbacks, sequence_jumps


def u32_period_series(rows, value_key, sequence_key="meta_buf_sequence"):
    values = []
    by_sequence = {}
    duplicates = 0
    rollbacks = 0
    sequence_jumps = 0
    previous = None
    for row in rows:
        sequence = as_int(row, sequence_key)
        value = as_int(row, value_key)
        if value == 0:
            continue
        if previous is not None:
            sequence_delta = sequence - previous[0]
            raw_delta = value - previous[1]
            if sequence_delta != 1:
                sequence_jumps += 1
            if raw_delta == 0:
                duplicates += 1
            else:
                delta = raw_delta % U32
                if delta > U32 // 2:
                    rollbacks += 1
                elif sequence_delta == 1:
                    period_ms = delta * 1000.0 / CAMERA_CLOCK_HZ
                    values.append(period_ms)
                    by_sequence[sequence] = period_ms
        previous = (sequence, value)
    return values, by_sequence, duplicates, rollbacks, sequence_jumps


def modulo_period_series(rows, value_key, modulus, unit_ms,
                         sequence_key="metadata_frame_index"):
    values = []
    by_sequence = {}
    duplicates = 0
    rollbacks = 0
    sequence_jumps = 0
    previous = None
    for row in rows:
        sequence = as_int(row, sequence_key)
        value = as_int(row, value_key, -1)
        if value < 0:
            continue
        if previous is not None:
            sequence_delta = sequence - previous[0]
            raw_delta = value - previous[1]
            if sequence_delta != 1:
                sequence_jumps += 1
            if raw_delta == 0:
                duplicates += 1
            elif sequence_delta == 1:
                delta = raw_delta % modulus
                if delta > modulus // 2:
                    rollbacks += 1
                else:
                    period_ms = delta * unit_ms
                    values.append(period_ms)
                    by_sequence[sequence] = period_ms
        previous = (sequence, value)
    return values, by_sequence, duplicates, rollbacks, sequence_jumps


def pair_metadata_to_video(video_rows, metadata_rows):
    timed_video = sorted(
        (as_int(row, "raw_v4l2_soe_ns"), as_int(row, "buf_sequence"), row)
        for row in video_rows if as_int(row, "raw_v4l2_soe_ns") > 0
    )
    video_times = [item[0] for item in timed_video]
    matches = {}
    residuals = []
    used_sequences = set()
    last_video_sequence = None
    for metadata in metadata_rows:
        metadata_ns = as_int(metadata, "first_payload_host_ns")
        if metadata_ns <= 0:
            continue
        position = bisect.bisect_left(video_times, metadata_ns)
        candidates = []
        for index in (position - 1, position, position + 1):
            if 0 <= index < len(timed_video):
                candidates.append(timed_video[index])
        if not candidates:
            continue
        video_ns, video_sequence, _ = min(
            candidates, key=lambda item: abs(item[0] - metadata_ns)
        )
        residual_ms = (video_ns - metadata_ns) / 1_000_000.0
        if abs(residual_ms) > 20.0 or video_sequence in used_sequences:
            continue
        if last_video_sequence is not None and video_sequence <= last_video_sequence:
            continue
        metadata_index = as_int(metadata, "metadata_frame_index")
        matches[video_sequence] = metadata
        used_sequences.add(video_sequence)
        residuals.append(residual_ms)
        last_video_sequence = video_sequence
    if not residuals:
        return None, {}
    median = statistics.median(residuals)
    mad = statistics.median(abs(value - median) for value in residuals)
    coverage = len(matches) / max(1, min(len(timed_video), len(metadata_rows)))
    offsets = [
        video_sequence - as_int(metadata, "metadata_frame_index")
        for video_sequence, metadata in matches.items()
    ]
    offset_mode, offset_mode_count = Counter(offsets).most_common(1)[0]
    reliable = (
        coverage >= 0.90 and mad <= 1.0 and
        offset_mode_count / len(offsets) >= 0.95
    )
    return {
        "method": "nearest same-clock payload timestamp with monotonic one-to-one sequence validation",
        "sequence_offset_video_minus_metadata_mode": offset_mode,
        "sequence_offset_consistency": offset_mode_count / len(offsets),
        "matched_pairs": len(matches),
        "coverage": coverage,
        "median_video_minus_first_payload_ms": median,
        "mad_ms": mad,
        "max_abs_residual_ms": max(abs(value) for value in residuals),
        "reliable": reliable,
    }, matches


def fmt(value):
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("video_csv")
    parser.add_argument("metadata_csv")
    parser.add_argument("output_dir")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    video_rows = read_csv(args.video_csv)
    metadata_rows = read_csv(args.metadata_csv)

    pairing, metadata_by_video_seq = pair_metadata_to_video(video_rows, metadata_rows)
    if metadata_by_video_seq:
        first_sequence = min(metadata_by_video_seq)
        last_sequence = max(metadata_by_video_seq)
        video_rows = [
            row for row in video_rows
            if first_sequence <= as_int(row, "buf_sequence") <= last_sequence
        ]

    raw_values, raw_by_seq, raw_dup, raw_rb, video_seq_jumps = period_series(
        video_rows, "raw_v4l2_soe_ns")
    header_values, header_by_seq, header_dup, header_rb, _ = period_series(
        video_rows, "camera_time_ns")
    dequeue_values, dequeue_by_seq, dequeue_dup, dequeue_rb, _ = period_series(
        video_rows, "dequeue_monotonic_ns")
    mid360_values, mid360_by_seq, _, _, _ = period_series(
        video_rows, "current_mid360_time_ns")

    # Use only reliably paired metadata frames. The collector can start or stop in
    # the middle of a UVC frame, so the first/last reconstructed metadata group is
    # not automatically a complete frame.
    metadata_ordered = []
    for video_sequence in sorted(metadata_by_video_seq):
        metadata = dict(metadata_by_video_seq[video_sequence])
        metadata["paired_video_sequence"] = str(video_sequence)
        metadata_ordered.append(metadata)
    pts_values, pts_by_meta_seq, pts_dup, pts_rb, metadata_seq_jumps = u32_period_series(
        metadata_ordered, "dominant_pts", "paired_video_sequence")
    scr_values, scr_by_meta_seq, scr_dup, scr_rb, _ = u32_period_series(
        metadata_ordered, "first_scr_stc", "paired_video_sequence")
    payload_values, payload_by_seq, payload_dup, payload_rb, _ = period_series(
        metadata_ordered, "first_payload_host_ns", "paired_video_sequence")
    sof_values, sof_by_seq, sof_dup, sof_rb, _ = modulo_period_series(
        metadata_ordered, "first_host_sof", 2048, 1.0,
        "paired_video_sequence")
    pts_by_seq = pts_by_meta_seq
    scr_by_seq = scr_by_meta_seq

    stats = {
        "v4l2_buf_timestamp": summarize(raw_values, raw_dup, raw_rb),
        "pts_48mhz": summarize(pts_values, pts_dup, pts_rb),
        "scr_stc_48mhz": summarize(scr_values, scr_dup, scr_rb),
        "metadata_first_payload_host": summarize(
            payload_values, payload_dup, payload_rb),
        "usb_host_sof": summarize(sof_values, sof_dup, sof_rb),
        "host_dequeue_monotonic": summarize(dequeue_values, dequeue_dup, dequeue_rb),
        "final_image_header": summarize(header_values, header_dup, header_rb),
    }
    diagnostics = {
        "video_frames": len(video_rows),
        "metadata_buffers": len(metadata_rows),
        "video_sequence_jumps": video_seq_jumps,
        "metadata_sequence_jumps": metadata_seq_jumps,
        "metadata_missing_pts": sum(as_int(row, "dominant_pts") == 0 for row in metadata_rows),
        "metadata_missing_scr": sum(as_int(row, "first_scr_stc") == 0 for row in metadata_rows),
        "pairing": pairing,
    }

    anomaly_sequences = [
        sequence for sequence, period in raw_by_seq.items()
        if 95.0 <= period < 97.0
    ]
    video_by_seq = {as_int(row, "buf_sequence"): row for row in video_rows}
    anomaly_rows = []
    for center in anomaly_sequences:
        for relative in range(-2, 3):
            sequence = center + relative
            video = video_by_seq.get(sequence)
            if video is None:
                continue
            metadata = metadata_by_video_seq.get(sequence, {})
            anomaly_rows.append({
                "anomaly_center_sequence": center,
                "relative_frame": relative,
                "sequence": sequence,
                "v4l2_delta_ms": raw_by_seq.get(sequence),
                "pts_delta_ms": pts_by_seq.get(sequence),
                "scr_delta_ms": scr_by_seq.get(sequence),
                "metadata_first_payload_delta_ms": payload_by_seq.get(sequence),
                "usb_host_sof_delta_ms": sof_by_seq.get(sequence),
                "host_dequeue_delta_ms": dequeue_by_seq.get(sequence),
                "mid360_latest_delta_ms": mid360_by_seq.get(sequence),
                "metadata_payload_flags_or": as_int(metadata, "payload_flags_or"),
                "metadata_pts": as_int(metadata, "dominant_pts"),
                "metadata_first_scr_stc": as_int(metadata, "first_scr_stc"),
                "metadata_first_host_sof": as_int(metadata, "first_host_sof"),
                "metadata_match": bool(metadata),
            })

    with open(output_dir / "summary.json", "w", encoding="utf-8") as handle:
        json.dump({"statistics": stats, "diagnostics": diagnostics,
                   "anomaly_count_95_to_97ms": len(anomaly_sequences)},
                  handle, ensure_ascii=False, indent=2)

    anomaly_fields = list(anomaly_rows[0].keys()) if anomaly_rows else [
        "anomaly_center_sequence", "relative_frame", "sequence"
    ]
    with open(output_dir / "anomaly_context.csv", "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=anomaly_fields)
        writer.writeheader()
        writer.writerows(anomaly_rows)

    metric_order = [
        "v4l2_buf_timestamp", "pts_48mhz", "scr_stc_48mhz",
        "metadata_first_payload_host", "usb_host_sof",
        "host_dequeue_monotonic", "final_image_header"
    ]
    with open(output_dir / "statistics.csv", "w", newline="", encoding="utf-8") as handle:
        fields = ["metric"] + list(next(iter(stats.values())).keys())
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for metric in metric_order:
            writer.writerow({"metric": metric, **stats[metric]})

    lines = [
        "# Version5 Camera timestamp diagnostics",
        "",
        "All periods are computed without exposure-center or calibration offsets.",
        "",
        "| Metric | Samples | Mean ms | Std ms | Min ms | Max ms | P50 ms | P95 ms | Hz | <97 ms | >103 ms | Duplicate | Rollback |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for metric in metric_order:
        item = stats[metric]
        lines.append(
            f"| {metric} | {item['samples']} | {fmt(item['mean_ms'])} | "
            f"{fmt(item['std_ms'])} | {fmt(item['min_ms'])} | "
            f"{fmt(item['max_ms'])} | {fmt(item['p50_ms'])} | "
            f"{fmt(item['p95_ms'])} | {fmt(item['frequency_hz'])} | "
            f"{item['below_97ms']} | {item['above_103ms']} | "
            f"{item['duplicates']} | {item['rollbacks']} |"
        )
    lines.extend([
        "",
        "## Pairing",
        "",
        f"- Video frames: {len(video_rows)}",
        f"- Reconstructed metadata frames: {len(metadata_rows)}",
        f"- Pairing result: `{json.dumps(pairing, ensure_ascii=False)}`",
        f"- Video sequence jumps: {video_seq_jumps}",
        f"- Metadata sequence jumps: {metadata_seq_jumps}",
        f"- Metadata buffers without PTS: {diagnostics['metadata_missing_pts']}",
        f"- Metadata buffers without SCR: {diagnostics['metadata_missing_scr']}",
        f"- 95-97 ms anomaly frames: {len(anomaly_sequences)}",
    ])
    (output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
