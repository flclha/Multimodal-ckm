# Version5 Backlight 2/0 Camera PTS/SCR A/B 测试报告

## 测试边界

- A 组：`Backlight=2`，写入并读回确认值为 2。
- B 组：`Backlight=0`，写入并读回确认值为 0。
- 两组均保持 1920x1080 MJPG、FrameRate=10、手动曝光 6 ms、Gain 等其他参数一致。
- 使用独立诊断 YAML，将 3 ms、1.76 ms、4.76 ms 和 CameraCalibrationOffset 全部置 0。
- 未修改最终 Camera timestamp 公式、STM32、MID360、LiDAR/IMU timestamp、FAST-LIVO 或外参。
- 每组原始采集 130 s；正式统计从启动后 5 s 开始，使用严格 120 s 窗口。
- 正式窗口每组均有 1207 帧、1206 个相邻周期，Camera video sequence 均连续。
- 测试结束后 UVC Backlight 已恢复并读回为 2，临时 Camera 驱动改动已撤回并重新编译。

## PTS/SCR 对比

| 指标 | A：Backlight=2 | B：Backlight=0 | 差异 |
|---|---:|---:|---:|
| PTS period mean | 99.436166684 ms | 99.436166667 ms | 0.000000017 ms |
| PTS period std | 0.000017867 ms | 0.000000000 ms | 0.000017867 ms |
| PTS period min | 99.436145833 ms | 99.436166667 ms | -0.000020834 ms |
| PTS period max | 99.436187500 ms | 99.436166667 ms | +0.000020833 ms |
| PTS frequency | 10.056703042 Hz | 10.056703044 Hz | -0.000000002 Hz |
| SCR period mean | 99.436152605 ms | 99.436152536 ms | 0.000000069 ms |
| SCR period std | 0.062484892 ms | 0.062486168 ms | -0.000001276 ms |
| SCR period min | 99.374958333 ms | 99.374937500 ms | +0.000020833 ms |
| SCR period max | 99.500083333 ms | 99.500041667 ms | +0.000041666 ms |
| SCR frequency | 10.056704466 Hz | 10.056704473 Hz | -0.000000007 Hz |

PTS 和 SCR 的均值、频率与分布在两组之间没有可测量的实质差异。A 组 PTS 的极小抖动只有约 18 ns 标准差，量级远小于本实验要辨别的 0.0567 Hz 差异。

## V4L2 与 Host 对比

| 指标 | A：Backlight=2 | B：Backlight=0 |
|---|---:|---:|
| V4L2 period mean | 99.436179104 ms | 99.436229685 ms |
| V4L2 period std | 1.393586654 ms | 1.393186266 ms |
| V4L2 period min | 95.767 ms | 95.729 ms |
| V4L2 period max | 100.261 ms | 100.282 ms |
| V4L2 frequency | 10.056701786 Hz | 10.056696671 Hz |
| Host dequeue frequency | 10.056713355 Hz | 10.056696543 Hz |
| `<97 ms` V4L2 周期 | 170 | 170 |

Backlight 切换也没有改变 V4L2/USB host 层约每 7 帧一次的 95~96 ms 时间戳量化现象。

## 连续性

| 项目 | A：Backlight=2 | B：Backlight=0 |
|---|---:|---:|
| 正式窗口帧数 | 1207 | 1207 |
| Video sequence jump | 0 | 0 |
| PTS pair sequence jump | 0 | 0 |
| SCR pair sequence jump | 0 | 0 |
| PTS/SCR duplicate | 0 | 0 |
| PTS/SCR rollback | 0 | 0 |
| Video-metadata 配对覆盖率 | 100% | 100% |

A 组 metadata 中出现过一个仅含 1 条 record 的重组碎片，这是 metadata buffer 边界造成的采集器片段，不是 Camera video sequence 丢帧。正式统计按同一规则排除低于中位 record 数 10% 的不完整 metadata 片段；两组保留阈值均为 46 records。

## MID360 PPS 窗口与相位

| 指标 | A：Backlight=2 | B：Backlight=0 |
|---|---:|---:|
| 完整 MID360 整秒窗口 | 119 | 119 |
| 10 帧/秒窗口 | 113 | 112 |
| 11 帧/秒窗口 | 6 | 7 |
| 其他帧数窗口 | 0 | 0 |
| 120 s 网格残差终值 | -679.968 ms | -679.907 ms |
| 相对 100 ms 网格的相位漂移 | -5.669292 ms/s | -5.670426 ms/s |

两组相位漂移差约 0.001134 ms/s，远小于共同的约 -5.67 ms/s 漂移，且没有形成模式相关差异。

## 结论

1. A 组在 Backlight=2 下，PTS/SCR 仍稳定为约 10.0567 Hz。
2. B 组在 Backlight=0 下，同样稳定为约 10.0567 Hz。
3. 两组 PTS/SCR 周期、频率、PPS 每秒帧数和 phase drift 没有明显差异。
4. 因此，PTS/SCR 很可能主要反映 Camera/UVC 设备内部输出时钟，而不是 STM32 Trigger 或真实曝光时刻；不能用 PTS/SCR 的 10.0567 Hz 证明 STM32 Trigger 是 10.0567 Hz。
5. 该结果还有一种兼容解释：虽然 Backlight 控件成功写入并读回，但厂商触发模式没有真正切换物理曝光路径。A/B 软件测试无法排除这一点。
6. 下一项具有决定性的实验仍是停止 TIM2/PA1 Trigger、保持 Camera USB 采集，观察 video sequence、PTS 和 SCR 是否停止增长。

## 数据文件

- `formal_120s_comparison.csv`：A/B 汇总。
- `formal_120s_comparison.json`：完整统计。
- `A_backlight2/formal_120s_metrics.json`、`B_backlight0/formal_120s_metrics.json`：单组结果。
- `A_backlight2/formal_120s_pps_phase.csv`、`B_backlight0/formal_120s_pps_phase.csv`：每个 PPS 整秒窗口明细。
- 两组目录中的 `video_frames.csv` 和 `metadata_frames.csv`：未经统计裁剪的原始记录。
