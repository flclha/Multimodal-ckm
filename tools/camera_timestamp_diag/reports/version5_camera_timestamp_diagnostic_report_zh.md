# Version5 Camera 时间戳诊断报告

## 1. 诊断范围

- 目标：仅诊断 Camera 时间来源、UVC metadata PTS/SCR、95~96 ms 短周期及硬触发配置。
- 有效重叠时长：约 124.9 s。
- Video 有效帧：1257；重建 metadata 帧：1258；成功匹配：1257。
- 诊断期间使用独立 `uvc_camera_diag_zero.yaml`，所有固定补偿均为 0。
- Version5 活动 YAML 和最终 Camera timestamp 公式未修改。
- LiDAR、IMU、per-point time、deskew、FAST-LIVO 后端和外参均未修改。

## 2. 检查的文件与函数

### Version5 Camera driver

文件：`/home/nuc/versions/version5/catkin_ws/src/mvs_ros_driver/src/uvc_driver.cpp`

- `V4L2BufferTimestampNs()`：读取 `v4l2_buffer.timestamp`。
- `V4L2CaptureHostRealtimeNs()`：MONOTONIC 到 REALTIME 的诊断映射。
- `setParams()`：读取 YAML、写入曝光和 Backlight 控件。
- Camera capture loop：`VIDIOC_DQBUF` 后记录 V4L2、dequeue、MID360 和最终 header。
- 本轮对该文件只增加诊断字段，没有改变时间戳计算分支。

### 独立 metadata 采集器

文件：`/home/nuc/versions/version5/tools/camera_timestamp_diag/uvc_metadata_capture.cpp`

- `ParseMetadata()`：解析 `/dev/video1` metadata buffer 内的 PTS、SCR STC、device SOF、host SOF 和 payload flags。
- `AccumulateMetadataFrames()`：按 PTS 变化重建 Camera frame，避免把一个 metadata buffer 盲目当成一帧。

### 分析脚本

文件：`/home/nuc/versions/version5/tools/camera_timestamp_diag/analyze_uvc_timestamps.py`

- `pair_metadata_to_video()`：用同一 CLOCK_MONOTONIC 域的时间做候选匹配，再用序列偏移和单调一一对应验证。
- `period_series()`、`u32_period_series()`、`modulo_period_series()`：分别统计 host ns、32-bit PTS/SCR 和 11-bit SOF 周期。

### Linux uvcvideo

- `drivers/media/usb/uvc/uvc_video.c`
- `drivers/media/usb/uvc/uvc_queue.c`
- `include/uapi/linux/uvcvideo.h`

当前模块参数：`hwtimestamps=0`、`clock=CLOCK_MONOTONIC`、`nodrop=1`。

## 3. 诊断日志格式

`video_frames.csv`：

```text
frame_index, frame_seq, raw_v4l2_soe_ns, v4l2_flags,
dequeue_monotonic_ns, dequeue_realtime_ns,
current_mid360_time_ns, current_mid360_time_seq,
v4l2_mapped_soe_ns, camera_time_ns, sequence_discontinuity
```

`metadata_frames.csv`：

```text
metadata_frame_index, dominant_pts, first_scr_stc, last_scr_stc,
first_scr_sof, last_scr_sof, first_host_sof, last_host_sof,
first_payload_host_ns, last_payload_host_ns,
payload_flags_or, fid, record_count, scr_record_count, eof_count
```

`anomaly_context.csv`：每个 95~97 ms 异常帧及前后各 2 帧，包含 V4L2、PTS、SCR、payload host、USB host SOF、dequeue、MID360 和 flags 的周期。

## 4. 120 秒统计

| 时间来源 | 样本 | 均值 ms | 标准差 ms | 最小 ms | 最大 ms | P50 ms | P95 ms | 频率 Hz | <97 ms | >103 ms | 重复 | 回退 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| V4L2 `buf.timestamp` | 1256 | 99.436260 | 1.386618 | 95.749000 | 100.297000 | 99.990000 | 100.156000 | 10.056694 | 177 | 0 | 0 | 0 |
| PTS，按 48 MHz | 1256 | 99.436167 | 0.000017 | 99.436146 | 99.436188 | 99.436167 | 99.436188 | 10.056703 | 0 | 0 | 0 | 0 |
| SCR STC，按 48 MHz | 1256 | 99.436206 | 0.062487 | 99.374938 | 99.500062 | 99.375042 | 99.500042 | 10.056699 | 0 | 0 | 0 | 0 |
| metadata 首 payload host time | 1256 | 99.436346 | 1.380338 | 95.887056 | 100.184565 | 99.990426 | 100.082573 | 10.056685 | 177 | 0 | 0 | 0 |
| USB host SOF | 1256 | 99.436306 | 1.391771 | 96.000000 | 100.000000 | 100.000000 | 100.000000 | 10.056689 | 177 | 0 | 0 | 0 |
| host dequeue monotonic | 1256 | 99.436442 | 1.383570 | 95.682153 | 100.369200 | 99.982831 | 100.174963 | 10.056675 | 177 | 0 | 0 | 0 |
| 最终 header，诊断补偿为 0 | 1256 | 99.436260 | 1.386618 | 95.749000 | 100.297000 | 99.990000 | 100.156000 | 10.056694 | 177 | 0 | 0 | 0 |

PTS/SCR 的 48 MHz 时钟与 host elapsed time 交叉估计分别为 47,999,913 Hz 和 47,999,933 Hz，接近描述符的 48 MHz。因此 10.0567 Hz 不是简单的 PTS 标称频率换算错误；它对应当前 USB 实际输出节奏。

## 5. Video 与 metadata 配对

| 项目 | 结果 |
|---|---:|
| 匹配覆盖率 | 99.9205% |
| 固定序列偏移 `video_seq - metadata_frame_index` | 30 |
| 序列偏移一致率 | 100% |
| `video timestamp - metadata first payload` 中位数 | -7.907824 ms |
| 配对残差 MAD | 0.120174 ms |
| 最大绝对配对残差 | 8.171915 ms |
| Video sequence jump | 0 |
| Metadata sequence jump | 0 |
| 缺 PTS / SCR | 0 / 0 |

一个 `/dev/video1` metadata buffer 常跨越两个 PTS，因此不能按 metadata buffer 到达顺序盲配。当前工具按 PTS 重建帧后，再使用同一主机时钟域和固定序列偏移验证，配对可靠。

## 6. 95~96 ms 异常帧示例

| seq | V4L2 ms | PTS ms | SCR ms | metadata payload host ms | USB host SOF ms | dequeue ms | MID360 latest ms |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 34 | 95.930 | 99.436167 | 99.375042 | 100.052428 | 100 | 99.995002 | 99.840 |
| 41 | 96.027 | 99.436146 | 99.500042 | 99.988272 | 100 | 100.065729 | 99.840 |
| 48 | 95.979 | 99.436167 | 99.374958 | 99.950728 | 100 | 99.976839 | 100.320 |
| 55 | 96.091 | 99.436146 | 99.500042 | 99.964740 | 100 | 99.805788 | 99.840 |
| 62 | 96.009 | 99.436167 | 99.375000 | 100.008267 | 100 | 100.068506 | 99.840 |

在同一逻辑帧上，V4L2 出现 95~96 ms 时，PTS/SCR 仍稳定约 99.436 ms，当前帧的 payload/SOF/dequeue 也约 100 ms。全局上 payload/SOF/dequeue 同样各有 177 个 96 ms 周期，但相对 V4L2 异常滞后或超前 1~2 帧，这是 UVC 传输和缓冲流水线的相位差，不是 MID360 映射造成的阶跃。

## 7. 结论

### `buf.timestamp` 是否是真实曝光 SOE

不能确认，而且当前实现不是设备硬件曝光时间。

`uvcvideo` 在 FID 翻转时把 `ktime_get()` 写入 buffer timestamp。当前 `hwtimestamps=0`，`uvc_video_clock_update()` 直接返回，不用 PTS/SCR 覆盖该值。`SOE` flag 则在队列初始化时统一设置，所以 flag 本身不能证明这个时间来自传感器曝光寄存器或 STRB。

当前值可视为“host 在新 UVC frame/payload 边界观察到的时间”，可能接近 SOE，但没有 STRB 就无法测出其相对物理曝光起点的固定 offset。

### PTS/SCR 是否更可靠

- 作为 Camera 设备侧的相对时间，PTS 明显更稳定：没有 95~96 ms 尖峰，标准差约 0.017 us。
- SCR 也稳定，但其标准语义更接近首批视频数据进入 USB 总线时的设备时钟采样，不等于物理曝光起点。
- PTS 是 presentation timestamp，也不能仅凭名称认定为曝光 SOE。
- 因此 PTS/SCR 值得用于下一轮时钟恢复 A/B 测试，但目前不能直接宣布它们是准确的绝对曝光时间。

### 95~96 ms 的来源

- 不是 MID360 fixed-anchor 映射：raw V4L2 与最终 header 周期逐帧相同。
- 不是旧的每 100 包 offset 覆盖：该 offset 现在只做诊断。
- 尖峰来自 host/USB 时间观测路径。USB host SOF 显示典型 100/96 ms 量化节奏，PTS/SCR 设备计数则连续稳定。
- 更深一层仍有待硬件确认：PTS、SCR 和 host elapsed 都给出 10.0567 Hz，而非严格 10 Hz。这可能是相机没有真正服从外部 Trigger，或 STM32 实际输出并非理论 10 Hz；不能只靠软件配置二选一。

## 8. `hwtimestamps=1` 评估

值得进行临时 A/B，但本轮未启用。

启用后，Linux 会使用 PTS，经 SCR/device SOF 和 USB host SOF 的滑动窗口映射到 host clock，并覆盖 `buf.timestamp`。预期可消除当前 95~96 ms 的 host 首包时间尖峰。

风险和验收：

1. 需要停止 Camera、卸载并重新加载 `uvcvideo`，设备号可能变化。
2. 驱动至少需要约 100 ms 的时钟样本范围，启动前几帧可能仍是旧 host timestamp。
3. 某些设备的 device SOF 不合规，可能需要 `UVC_QUIRK_INVALID_DEVICE_SOF`；不能预先假设本相机需要该 quirk。
4. 必须复跑同一套 120 s 诊断，检查 PTS/SCR 配对、单调性、短周期、长期频率和 header 相对 MID360 的漂移。
5. 即使统计改善，也仍需用 STRB 或硬件 Trigger 停帧实验验证曝光语义。

## 9. 硬触发配置检查

- YAML：`Backlight=2`、`TriggerEnable=1`。
- 驱动实际行为：读取了 YAML `Backlight`，但写控件时硬编码为 `2`；`v4l2-ctl` 读回 `backlight_compensation=2`。
- `TriggerEnable` 仅被读取，没有参与任何 V4L2 控件写入或采集逻辑。
- V4L2 negotiated rate 显示 10 fps，这只是格式协商值，不是硬触发生效证明。
- STM32 源码按 72 MHz timer clock、PSC=7199、ARR=999 配置名义 10 Hz；但软件公式不能证明输出引脚的实际频率和 Camera 是否响应。

独立硬件验证：保持 USB 采集和帧计数运行，临时断开 Camera SYNC/Trigger 线，或停止 STM32 TIM2 输出。如果外部硬触发真实生效，清空 1~2 帧已有 buffer 后新帧应停止；若仍以约 10.0567 Hz 连续输出，则 Camera 未受外部 Trigger 控制。恢复 Trigger 后帧流应恢复。本轮没有自动修改 STM32。

## 10. 下一步

1. 先执行“断 Trigger 是否停帧”测试，确认物理硬触发链。
2. 若确实停帧，测量 STM32 PA1 实际周期；没有示波器时，可用 MCU input-capture、逻辑分析仪或把 Trigger 引到 NUC 可时间戳 GPIO。
3. 再临时启用 `uvcvideo hwtimestamps=1` 做同条件 120 s A/B，判断 PTS/SCR 恢复后的 `buf.timestamp` 是否消除 96 ms 尖峰。
4. 在以上证据完成前，不改 Version5 最终 Camera timestamp 公式，也不直接把 PTS 当成绝对曝光 SOE。
