# Version5 Camera timestamp diagnostics

All periods are computed without exposure-center or calibration offsets.

| Metric | Samples | Mean ms | Std ms | Min ms | Max ms | P50 ms | P95 ms | Hz | <97 ms | >103 ms | Duplicate | Rollback |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| v4l2_buf_timestamp | 1315 | 99.771949 | 12.223672 | 95.729000 | 539.994000 | 99.989000 | 100.163000 | 10.022857 | 185 | 1 | 0 | 0 |
| pts_48mhz | 1314 | 99.511841 | 2.742088 | 99.436167 | 198.872333 | 99.436167 | 99.436167 | 10.049055 | 0 | 1 | 0 | 0 |
| scr_stc_48mhz | 1315 | 99.778042 | 12.211362 | 99.374938 | 542.375000 | 99.375042 | 99.500042 | 10.022245 | 0 | 2 | 0 | 0 |
| metadata_first_payload_host | 1315 | 99.778029 | 12.225078 | 95.911883 | 539.993878 | 99.990579 | 100.082217 | 10.022246 | 185 | 2 | 0 | 0 |
| usb_host_sof | 1315 | 99.777947 | 12.225360 | 95.000000 | 540.000000 | 100.000000 | 101.000000 | 10.022255 | 129 | 2 | 0 | 0 |
| host_dequeue_monotonic | 1315 | 99.437482 | 1.387060 | 95.671774 | 100.359273 | 99.976810 | 100.188662 | 10.056570 | 185 | 0 | 0 | 0 |
| final_image_header | 1315 | 99.771949 | 12.223672 | 95.729000 | 539.994000 | 99.989000 | 100.163000 | 10.022857 | 185 | 1 | 0 | 0 |

## Pairing

- Video frames: 1316
- Reconstructed metadata frames: 1317
- Pairing result: `{"method": "nearest same-clock payload timestamp with monotonic one-to-one sequence validation", "sequence_offset_video_minus_metadata_mode": -1, "sequence_offset_consistency": 0.9984802431610942, "matched_pairs": 1316, "coverage": 1.0, "median_video_minus_first_payload_ms": -7.9207615, "mad_ms": 0.12528050000000013, "max_abs_residual_ms": 8.167766, "reliable": true}`
- Video sequence jumps: 0
- Metadata sequence jumps: 0
- Metadata buffers without PTS: 1
- Metadata buffers without SCR: 0
- 95-97 ms anomaly frames: 185
