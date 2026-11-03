#!/usr/bin/env python3
"""Compare fixed-map static/motion validation reports."""

import argparse
import json
from pathlib import Path


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("static_json")
    parser.add_argument("motion_json")
    parser.add_argument("--max-p95-ms", type=float, default=5.0)
    parser.add_argument("--max-drift-ms-per-s", type=float, default=0.01)
    parser.add_argument("--max-lidar-backlog", type=float, default=2.0)
    parser.add_argument(
        "--cross-restart-status",
        choices=("not-tested", "passed", "failed"),
        default="not-tested",
        help="whether the same absolute b survived a new STREAMON/power cycle",
    )
    parser.add_argument("--output-json")
    args = parser.parse_args()

    reports = {"static": load(args.static_json), "motion": load(args.motion_json)}
    failures = []
    comparison = {}
    for label, report in reports.items():
        residual = report["residual"]
        anomalies = report["anomalies"]
        backlog = report.get("fast_livo_lidar_backlog")
        comparison[label] = {
            "pair_count": report["pair_count"],
            "residual_mean_ms": residual["signed_mean_ms"],
            "residual_std_ms": residual["signed_std_ms"],
            "residual_p95_ms": residual["absolute_p95_ms"],
            "residual_max_ms": residual["absolute_max_ms"],
            "residual_drift_ms_per_s": residual["drift_ms_per_s"],
            "mapped_camera_period_ms": anomalies["mapped_camera_period_ms"],
            "pts_duplicates": anomalies["pts_duplicate_count"],
            "pts_rollbacks": anomalies["pts_rollback_count"],
            "metadata_filtered_duplicates": anomalies.get(
                "metadata_filtered_duplicate_count", 0
            ),
            "metadata_filtered_rollbacks": anomalies.get(
                "metadata_filtered_rollback_count", 0
            ),
            "pts_match_drops": anomalies["pts_match_drop_count"],
            "mapped_duplicates": anomalies["mapped_duplicate_count"],
            "mapped_rollbacks": anomalies["mapped_rollback_count"],
            "sequence_jumps": anomalies["sequence_jump_count"],
            "fast_livo_lidar_backlog": backlog,
        }
        if residual["absolute_p95_ms"] > args.max_p95_ms:
            failures.append(f"{label}: residual P95 exceeds threshold")
        if abs(residual["drift_ms_per_s"]) > args.max_drift_ms_per_s:
            failures.append(f"{label}: residual drift exceeds threshold")
        if (anomalies["pts_duplicate_count"] or anomalies["pts_rollback_count"]
                or anomalies["mapped_duplicate_count"]
                or anomalies["mapped_rollback_count"]):
            failures.append(f"{label}: PTS/mapped timestamp duplicate or rollback")
        if anomalies["pts_match_drop_count"]:
            failures.append(f"{label}: video frames were dropped for missing PTS matches")
        if anomalies["sequence_jump_count"]:
            failures.append(f"{label}: camera sequence jumps were observed")
        if backlog and backlog["count"] and backlog["max"] > args.max_lidar_backlog:
            failures.append(f"{label}: FAST-LIVO LiDAR backlog exceeds threshold")

    first = reports["static"]
    same_parameters = all(
        report["camera_clock_scale_a"] == first["camera_clock_scale_a"]
        and report["camera_clock_offset_ns"] == first["camera_clock_offset_ns"]
        for report in reports.values()
    )
    if not same_parameters:
        failures.append("static and motion reports did not use the same frozen a,b")

    continuous_session_passed = not failures
    if args.cross_restart_status == "failed":
        failures.append("fixed absolute b failed the cross-STREAMON/restart test")
    elif args.cross_restart_status == "not-tested":
        failures.append("cross-STREAMON/restart persistence was not verified")

    result = {
        "camera_clock_scale_a": first["camera_clock_scale_a"],
        "camera_clock_offset_ns": first["camera_clock_offset_ns"],
        "clock_skew_ppm": first["clock_skew_ppm"],
        "thresholds": {
            "max_absolute_residual_p95_ms": args.max_p95_ms,
            "max_absolute_residual_drift_ms_per_s": args.max_drift_ms_per_s,
            "max_fast_livo_lidar_backlog": args.max_lidar_backlog,
        },
        "static_vs_motion": comparison,
        "continuous_session_validation_passed": continuous_session_passed,
        "cross_restart_status": args.cross_restart_status,
        "recommend_as_version5_default": not failures,
        "failures": failures,
        "caveat": "A fixed b is valid only while the camera PTS and MID360 clock origins remain unchanged across restart/power-cycle tests.",
    }
    rendered = json.dumps(result, indent=2, ensure_ascii=False)
    print(rendered)
    if args.output_json:
        Path(args.output_json).write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
