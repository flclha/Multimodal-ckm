# Version5 Camera timestamp diagnostics

All periods are computed without exposure-center or calibration offsets.

| Metric | Samples | Mean ms | Std ms | Min ms | Max ms | P50 ms | P95 ms | Hz | <97 ms | >103 ms | Duplicate | Rollback |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| v4l2_buf_timestamp | 1256 | 99.436260 | 1.386618 | 95.749000 | 100.297000 | 99.990000 | 100.156000 | 10.056694 | 177 | 0 | 0 | 0 |
| pts_48mhz | 1257 | 99.436167 | 0.000017 | 99.436146 | 99.436188 | 99.436167 | 99.436188 | 10.056703 | 0 | 0 | 0 | 0 |
| scr_stc_48mhz | 1257 | 99.417462 | 0.667219 | 75.875000 | 99.500062 | 99.375042 | 99.500042 | 10.058595 | 1 | 0 | 0 | 0 |
| host_dequeue_monotonic | 1256 | 99.436442 | 1.383570 | 95.682153 | 100.369200 | 99.982831 | 100.174963 | 10.056675 | 177 | 0 | 0 | 0 |
| final_image_header | 1256 | 99.436260 | 1.386618 | 95.749000 | 100.297000 | 99.990000 | 100.156000 | 10.056694 | 177 | 0 | 0 | 0 |

## Pairing

- Video frames: 1257
- Reconstructed metadata frames: 1258
- Pairing result: `{"method": "nearest same-clock payload timestamp with monotonic one-to-one sequence validation", "sequence_offset_video_minus_metadata_mode": 30, "sequence_offset_consistency": 1.0, "matched_pairs": 1257, "coverage": 0.9992050874403816, "median_video_minus_first_payload_ms": -7.907824, "mad_ms": 0.12017399999999956, "max_abs_residual_ms": 8.171915, "reliable": true}`
- Video sequence jumps: 0
- Metadata sequence jumps: 0
- Metadata buffers without PTS: 0
- Metadata buffers without SCR: 0
- 95-97 ms anomaly frames: 177
