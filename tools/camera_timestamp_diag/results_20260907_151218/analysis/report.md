# Version5 Camera timestamp diagnostics

All periods are computed without exposure-center or calibration offsets.

| Metric | Samples | Mean ms | Std ms | Min ms | Max ms | P50 ms | P95 ms | Hz | <97 ms | >103 ms | Duplicate | Rollback |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| v4l2_buf_timestamp | 1437 | 99.743931 | 11.706252 | 95.715000 | 540.209000 | 99.989000 | 100.180000 | 10.025673 | 202 | 1 | 0 | 0 |
| pts_48mhz | 1257 | 99.436167 | 0.000000 | 99.436167 | 99.436167 | 99.436167 | 99.436167 | 10.056703 | 0 | 0 | 0 | 0 |
| scr_stc_48mhz | 1257 | 99.415672 | 0.730606 | 73.624979 | 99.625000 | 99.375042 | 99.500021 | 10.058776 | 1 | 0 | 0 | 0 |
| host_dequeue_monotonic | 1437 | 99.437640 | 1.390938 | 95.666854 | 100.400935 | 99.983143 | 100.198799 | 10.056554 | 202 | 0 | 0 | 0 |
| final_image_header | 1437 | 99.743931 | 11.706252 | 95.715000 | 540.209000 | 99.989000 | 100.180000 | 10.025673 | 202 | 1 | 0 | 0 |

## Pairing

- Video frames: 1438
- Metadata buffers: 1258
- Pairing result: `{"sequence_offset_video_minus_metadata": 0, "matched_pairs": 1258, "coverage": 1.0, "median_video_minus_first_payload_ms": -0.000645, "mad_ms": 0.000259, "reliable": true}`
- Video sequence jumps: 0
- Metadata sequence jumps: 0
- Metadata buffers without PTS: 0
- Metadata buffers without SCR: 0
- 95-97 ms anomaly frames: 202
