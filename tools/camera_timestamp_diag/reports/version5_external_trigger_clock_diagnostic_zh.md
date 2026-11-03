# Version5 Camera 外触发、时钟基准与频率诊断报告

## 1. 本轮边界

- 未修改 Version5 最终 Camera timestamp 公式。
- 未修改 LiDAR、IMU、per-point timestamp、deskew、FAST-LIVO 后端或外参。
- 未启用 `uvcvideo hwtimestamps=1`；实测仍为 `hwtimestamps=0`、`clock=CLOCK_MONOTONIC`。
- 诊断段临时把固定曝光/标定补偿设为 0，只用于隔离频率来源；常量 offset 不改变周期和频率。

## 2. STM32 Trigger 完整代码链

工程：`/home/nuc/LIV_handhold/stm32_timersync-open/USER/PWM.uvprojx`

- MCU：`STM32F103C8`。
- `USER/main.c:22`：`TIM2_PWM_Init(999, 7199)`，Camera Trigger 输出。
- `HARDWARE/TIMER/timer.c:157-205`：启用 TIM2，PA1 配置为复用推挽输出，实际使用 TIM2 CH2、PWM2；`CCR2 = ARR / 2 = 499`。
- `USER/main.c:24`：`TIM3_PWM_Init(9999, 7199)`，1 Hz PPS 输出。
- `HARDWARE/TIMER/timer.c:216-253`：TIM3 CH2 输出至 PB5，并启用 TIM3 update 中断。
- `HARDWARE/TIMER/timer.c:103`：每次 TIM3 update 中断执行 `TIM2->CNT = TIM2->ARR / 2`。
- 全工程未发现其他运行期 TIM2 `PSC`、`ARR` 或 `CNT` 写入；唯一运行期相位干预就是上述每秒一次的 `CNT=499`。

### RCC 时钟推导

`USER/system_stm32f10x.c` 选择 `SYSCLK_FREQ_72MHz`。对于本项目的 STM32F103C8：

1. `HSE_VALUE = 8,000,000 Hz`。
2. 非 Connectivity Line 分支使用 `HSE * 9`，PLL/SYSCLK = 72 MHz。
3. HCLK = SYSCLK = 72 MHz。
4. PCLK1 = HCLK / 2 = 36 MHz。
5. APB1 prescaler 不为 1，STM32F1 的 TIM2/TIM3 kernel clock = 2 * PCLK1 = 72 MHz。

所以源代码对应的理论值为：

```text
TIM2 counter = 72,000,000 / (7199 + 1) = 10,000 Hz
TIM2 PWM     = 10,000 / (999 + 1) = 10.000000 Hz
TIM3 PWM     = 10,000 / (9999 + 1) = 1.000000 Hz
```

这只证明“仓库源码按 8 MHz 晶振计算时”的理论频率。NUC 没有 SWD/固件回读，也没有示波器或频率计，因此不能证明板上当前烧录固件与仓库一致，也不能证明实际 HSE 与 PA1 输出正好为 10.000000 Hz。

### TIM3 对 TIM2 的影响

`TIM2->CNT=499` 会改变包含该写入的那一个 Trigger 周期。若写入前 TIM2 本来就在 499 附近，影响很小；若不在，则该周期会被缩短或拉长。代码没有记录写入前的 CNT，故无法从源码计算实际修正量。

观测到的约 5.67 ms/s 相位移动，在量级上可以由“每秒固定推进约 56~57 个 100 us timer tick”解释，但这只是待验证假设，不能当成结论。必须测 PA1，或在固件中记录每次 TIM3 ISR 进入前的 `TIM2->CNT`。

## 3. Camera 硬触发配置链

配置：`/home/nuc/versions/version5/catkin_ws/src/mvs_ros_driver/config/uvc_camera.yaml`

- `TriggerEnable: 1`
- `FrameRate: 10`
- `Backlight: 2`
- `ExposureAutoMode: 0`
- `Exposure: 156`，对应当前相机手动曝光模式下 6 ms。

驱动：`/home/nuc/versions/version5/catkin_ws/src/mvs_ros_driver/src/uvc_driver.cpp`

- `main():1408` 在开启采集线程前调用 `setParams()`。
- `setParams():809-813` 把 `V4L2_CID_BACKLIGHT_COMPENSATION` **硬编码写为 2**，并未使用读取到的 YAML `Backlight` 变量。
- `TriggerEnable` 在 `main():1324` 被读取，但后续没有用于写 UVC 控件或控制采集，因此它目前只是无效配置变量。
- `WorkThread():838-842` 设置格式并协商 10 fps。
- `WorkThread():893-898` 最后才 `VIDIOC_STREAMON`，不存在先出 free-run 帧再设置 Backlight 的正常启动路径。
- `start_slam1:398-428` 的 Camera 重试会杀掉旧进程并重新启动；每次都重新经过 `setParams()`，未发现重试时跳过 Backlight 设置的路径。

运行态连续两次独立启动验证：

- 写入日志均显示 Backlight=2 成功；
- 流期间 `v4l2-ctl` 读回 `backlight_compensation=2`；
- 停流后仍读回 2；
- 再启动仍读回 2；
- 协商帧率为 10/1。

这些证据只证明厂商复用控件值被写入并保持，**不能单独证明 PA1 Trigger 边沿实际控制曝光**。物理依赖仍需断 Trigger 实验。

## 4. 各频率统计层的含义

| 统计量 | 时钟域与单位 | 计算方式 | 是否受 USB/队列影响 |
|---|---|---|---|
| V4L2 `buf.timestamp` | host `CLOCK_MONOTONIC`，ns | `tv_sec*1e9 + tv_usec*1e3` | 是。当前 `hwtimestamps=0`，UVC 在首个 payload/FID 处取 host time |
| PTS | Camera/UVC 设备 32-bit clock，nominal 48 MHz | `delta_ticks * 1000 / 48,000,000` ms | 不直接受 host dequeue 影响；但 PTS 语义不是已证明的曝光 SOE |
| SCR/STC | Camera/UVC 设备 32-bit source clock，nominal 48 MHz | 同上，unsigned 32-bit wrap | 不直接受 host dequeue；规范语义接近首个视频数据放上 USB 时刻 |
| SCR device SOF | USB device 11-bit frame number | `(cur-prev)&2047`，1 ms/帧 | 1 ms 量化，2048 ms wrap |
| host USB SOF | USB host controller frame number | 同上 | 是 USB 调度参考，不是曝光时刻 |
| host dequeue | host `CLOCK_MONOTONIC`，ns | `clock_gettime()` 紧跟 `VIDIOC_DQBUF` | 明显受 URB、排队和调度影响 |
| ROS `header.stamp` | 当前映射后的 MID360 时间域 | 本轮零补偿诊断中保留 V4L2 周期 | 常量 offset 不影响频率 |
| `rostopic hz` | ROS callback 到达时间 | `rospy.get_rostime()` 的 callback 间隔 | 是；默认不读消息 header |
| `start_slam1 header_hz` | 消息 `header.stamp` | `(N-1)/(last_header-first_header)` | 取决于 header 来源 |
| `start_slam1 receive_hz` | callback `rospy.Time.now()` | `(N-1)/(last_recv-first_recv)` | 是 |

因此“打印为 10.0567 Hz”必须注明统计层，不能统一称为传感器真实采样频率。

## 5. PTS/SCR 字段与换算

metadata 中：

- PTS：32 bit，按设备声明的 `dwClockFrequency=48 MHz` 换算；wrap 使用 unsigned modulo `2^32`。
- SCR：6 byte，由 32-bit STC 和 11-bit SOF 构成；STC 同样按 48 MHz，SOF modulo 2048。
- PTS 相邻原始增量主要为 `4,772,936` 或 `4,772,937` tick：

```text
4,772,936 / 48,000,000 * 1000 = 99.436166667 ms
4,772,937 / 48,000,000 * 1000 = 99.436187500 ms
```

用 124.9 s host elapsed 反算得到：

- PTS tick rate 约 47,999,913 Hz；
- SCR/STC tick rate 约 47,999,933 Hz。

与 nominal 48 MHz 的差仅约 1~2 ppm，因此 `10.0567 Hz` 不是“误把其他 clock 当 48 MHz 除”造成的 0.567% 比例错误。

## 6. 120 秒统计

稳定重叠区间：1257 帧、1256 个周期、124.891942 s。无 sequence jump、duplicate 或 rollback。

| 时间源 | mean ms | std ms | min ms | max ms | 频率 Hz | <97 ms | >103 ms |
|---|---:|---:|---:|---:|---:|---:|---:|
| V4L2 `buf.timestamp` | 99.436260 | 1.386618 | 95.749000 | 100.297000 | 10.056694 | 177 | 0 |
| PTS / nominal 48 MHz | 99.436167 | 0.000017 | 99.436146 | 99.436188 | 10.056703 | 0 | 0 |
| SCR/STC / nominal 48 MHz | 99.436206 | 0.062487 | 99.374938 | 99.500062 | 10.056699 | 0 | 0 |
| metadata 首 payload host time | 99.436346 | 1.380338 | 95.887000 | 100.184565 | 10.056685 | 177 | 0 |
| USB host SOF | 99.436306 | 1.391771 | 96.000000 | 100.000000 | 10.056689 | 177 | 0 |
| host dequeue monotonic | 99.436442 | 1.383570 | 95.682000 | 100.369000 | 10.056675 | 177 | 0 |
| 最终 header（诊断零补偿） | 99.436260 | 1.386618 | 95.749000 | 100.297000 | 10.056694 | 177 | 0 |

Video/metadata 配对：1257/1258，覆盖率 99.9205%；固定 `video_seq - metadata_index = 30`，重叠区间一致率 100%；无 PTS/SCR 缺失。metadata buffer 会跨两个 PTS，本轮按 PTS 重组后再用单调和 sequence 一致性匹配，没有简单按 buffer 到达顺序盲配。

### 95~96 ms 异常定位

例如 Camera sequence 34：

| 层 | 周期 ms |
|---|---:|
| V4L2 `buf.timestamp` | 95.930 |
| PTS | 99.436167 |
| SCR/STC | 99.375042 |
| metadata 首 payload host | 100.052 |
| USB host SOF | 100.000 |
| host dequeue | 99.995 |

sequence 41、48、55、62 等也呈相同规律。结论是：95~96 ms 尖峰来自 host 首 payload/V4L2/USB 的毫秒量化和批处理表现，不存在于 PTS/SCR 设备相对时钟中；fixed-anchor MID360 映射也没有制造这些尖峰。

## 7. PPS-Camera 相位

逐秒明细：`results_20260907_151218_second/analysis_refined/pps_phase_per_second.csv`

- 排除首尾不完整秒后，共 124 个 MID360 integer-second 窗口。
- 117 秒包含 10 帧，7 秒包含 11 帧，无其他计数。
- 11 帧秒出现在 1590212195、2213、2230、2248、2265、2283、2301，间隔约 17~18 秒。
- 相对于严格 `sequence*100 ms` 网格，残差从 0 变化到约 -708.058 ms；线性斜率约 -0.563788 ms/frame，即 -5.66985 ms/s。
- 未发现周期性重新归位或 PPS 重锚；相位持续滑动并跨 1 s wrap。

但要注意：这里的 mapped Camera time 来自 host monotonic 固定锚点，不是直接由 Camera PTS 映射到 MID360。用同步采样的最新 `/home/nuc/timeshare` 交叉检查，host 与 MID360 只显示约 177 ppm 量级斜率差，远小于 5670 ppm；该共享值又是约 100 ms 离散更新，含 0 和 200 ms 跳步，因此只能排除“大比例换算错误”，不能替代电气测量。

## 8. 当前能下的结论

1. 仓库固件理论上输出 TIM2 10.000000 Hz，但没有证据证明实际 PA1 波形或当前已烧录固件就是 10.000000 Hz。
2. Camera 软件配置链会在 STREAMON 前写 Backlight=2，运行中可读回 2，重启后也保持；`TriggerEnable` 本身未生效。
3. `10.0567 Hz` 不是 ROS callback 或 MID360 clock-offset 更新独自产生的，因为 PTS/SCR 设备侧相对计数也稳定给出同一频率。
4. 原始 V4L2 的 95~96 ms 短周期是 host/USB timestamp 抖动；PTS/SCR 更适合描述 Camera/UVC 设备内部相对周期。
5. PTS/SCR 仍不能直接证明真实曝光 SOE。它们可以来自 UVC 控制器/编码管线，必须与断 Trigger 实验或 STRB/PA1 测量结合。
6. 当前存在“Camera/UVC 输出流相对 MID360/PPS 不按严格 10 Hz 锁定”的证据，但尚不能唯一判断是：外触发未物理生效、当前 STM32 固件/实际时钟不同、TIM3 每秒改 TIM2 CNT 造成额外边沿，或 Camera 在触发后自行重定时/生成 PTS。

## 9. 是否启用 `hwtimestamps=1`

现在不建议先启用。Linux `uvcvideo` 开启后会用 PTS，经 SCR/device SOF 和 host SOF 的滑动窗口恢复成 host clock timestamp；它很可能消除 95~96 ms host 首包尖峰，但：

- 不会把 99.436 ms 自动变成 100 ms；
- 不会证明 PTS 就是曝光 SOE；
- 不会证明外部 Trigger 生效；
- 启动阶段需要积累时钟样本，且设备 metadata/PTS 语义错误时可能引入新偏差。

所以顺序应是先做断 Trigger 物理依赖验证，再决定是否进行同条件 `hwtimestamps=0/1` A/B。

## 10. 最小断 Trigger 实验

当前 STM32 无可用 NUC USB 控制链，建议只制作临时测试固件，不改 Camera 模式：

1. Camera USB 保持连接，NUC 持续采集并同时观察 frame sequence、PTS 和 `rostopic hz`。
2. 临时固件中保留 TIM3 PPS/GPRMC，但不调用 `TIM2_PWM_Init(999,7199)`；或在初始化后执行 `TIM_Cmd(TIM2, DISABLE)`，并把 PA1 配成普通推挽低电平，确保没有残留边沿。
3. 烧录该临时固件。允许最多清空 4 个 V4L2 buffer 和少量 USB 在途数据。
4. 若新 frame 随后完全停止，且恢复原固件后恢复，则证明 Camera 物理依赖外部 Trigger。
5. 若 frame 仍以约 10.0567 Hz 连续输出，则 Backlight=2 没有真正启用外触发，或 Trigger 输入/接线并未控制曝光。
6. 若能测 PA1，优先用逻辑分析仪、频率计或示波器连续计数至少 120 s，同时记录 PPS；这是区分 TIM2 输出和 Camera/UVC PTS 的最直接证据。

不要通过把 Camera 改成 free-run 来替代此实验，那只能证明 free-run 可用，不能证明当前硬触发依赖。

## 11. 参考源码

- Linux UVC clock/timestamp 恢复实现：https://github.com/torvalds/linux/blob/master/drivers/media/usb/uvc/uvc_video.c
- Linux UVC queue timestamp flags：https://github.com/torvalds/linux/blob/master/drivers/media/usb/uvc/uvc_queue.c
- V4L2 timestamp flags 文档：https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/buffer.html
- STM32F1 RM0008：https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101-103-105-107-stm32f100-series-armbased-32bit-mcus-stmicroelectronics.pdf
