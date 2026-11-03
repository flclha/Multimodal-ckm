# Version5 Camera timestamp diagnostics

All periods are computed without exposure-center or calibration offsets.

| Metric | Samples | Mean ms | Std ms | Min ms | Max ms | P50 ms | P95 ms | Hz | <97 ms | >103 ms | Duplicate | Rollback |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| v4l2_buf_timestamp | 1315 | 99.771888 | 12.224349 | 95.767000 | 540.014000 | 99.990000 | 100.142000 | 10.022863 | 185 | 1 | 0 | 0 |
| pts_48mhz | 1314 | 99.511841 | 2.742444 | 98.304000 | 198.872333 | 99.436167 | 99.436188 | 10.049055 | 0 | 1 | 0 | 0 |
| scr_stc_48mhz | 1315 | 99.778137 | 12.214806 | 99.374958 | 542.499958 | 99.375042 | 99.500042 | 10.022236 | 0 | 2 | 0 | 0 |
| metadata_first_payload_host | 1315 | 99.778047 | 12.225649 | 95.890230 | 540.013604 | 99.988876 | 100.088371 | 10.022245 | 185 | 2 | 0 | 0 |
| usb_host_sof | 1315 | 99.777947 | 12.225982 | 96.000000 | 540.000000 | 100.000000 | 100.000000 | 10.022255 | 185 | 2 | 0 | 0 |
| host_dequeue_monotonic | 1315 | 99.437356 | 1.392562 | 95.744367 | 100.355565 | 99.973761 | 100.175613 | 10.056583 | 185 | 0 | 0 | 0 |
| final_image_header | 1315 | 99.771888 | 12.224349 | 95.767000 | 540.014000 | 99.990000 | 100.142000 | 10.022863 | 185 | 1 | 0 | 0 |

## Pairing

- Video frames: 1316
- Reconstructed metadata frames: 1318
- Pairing result: `{"method": "nearest same-clock payload timestamp with monotonic one-to-one sequence validation", "sequence_offset_video_minus_metadata_mode": -1, "sequence_offset_consistency": 0.6968085106382979, "matched_pairs": 1316, "coverage": 1.0, "median_video_minus_first_payload_ms": -7.9306155, "mad_ms": 0.10086900000000032, "max_abs_residual_ms": 8.154937, "reliable": false}`
- Video sequence jumps: 0
- Metadata sequence jumps: 0
- Metadata buffers without PTS: 1
- Metadata buffers without SCR: 0
- 95-97 ms anomaly frames: 185
