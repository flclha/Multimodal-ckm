# Version5 / FAST-LIVO2 并发架构阅读指南

## 1. 核心建图主线程

1. `src/main.cpp`：ROS节点入口，创建 `LIVMapper` 并调用 `run()`。
2. `src/LIVMapper.cpp` 的 `run()`：使用 `ros::spinOnce()` 处理回调，然后执行时间同步、IMU传播、LIO/VIO状态更新。这是主状态机。
3. `include/LIVMapper.h`：`LIVMapper` 的状态、传感器缓冲区、ROS订阅/发布器和子模块。
4. `src/LIVMapper.cpp` 的 `handleLIO()` / `handleVIO()`：LiDAR-IMU更新和视觉更新的主入口。

当前使用 `ros::spinOnce()`，所以LiDAR、IMU、图像和ROS Timer回调由同一个主线程串行执行。`ros::Timer` 不等于独立线程。

## 2. 现有C++后台线程

- `include/vio.h`：`colmap_write_thread_`、有界队列、mutex、condition_variable和计数器。
- `src/vio.cpp`：`startColmapWriter()`、`stopColmapWriter()`、`colmapWriterLoop()`。

这是工程内最适合仿照的完整生产者-消费者示例：主线程快速投递图像快照，写盘线程异步处理，析构前停止并 `join()`。

## 3. OpenMP计算并行

- `src/voxel_map.cpp`：点云/体素相关循环。
- `src/vio.cpp`：视觉残差计算循环。

OpenMP适合同一帧内大量独立计算，不适合长时间写盘或带状态的常驻任务。

## 4. Python数据保存线程

- `scripts/multimodal_image_saver.py`：ROS回调投递图像，`image_writer` 线程写PNG和CSV。
- `scripts/multimodal_raw_recorder.py`：ROS回调投递配准LiDAR帧，`pcd_writer` 线程写PCD。

这两个脚本是独立ROS进程，不与FAST-LIVO核心共享C++内存。

## 5. 新线程模板

模板已以注释形式放在：

- `include/LIVMapper.h` 类末尾：任务、队列、锁、条件变量和线程成员。
- `src/LIVMapper.cpp` 的构造/析构函数及 `run()` 后：启动、停止、投递和worker loop。

启用前先明确任务是否只需读快照。如果必须修改SLAM状态，不要直接照搬此模板，应先设计状态所有权和同步边界。

## 6. 安全规则

1. 建图主线程不得等待磁盘、网络或长时间算法。
2. 队列必须有上限；辅助任务积压时丢帧，不拖慢SLAM。
3. 跨线程只传递深拷贝或共享的 `const` 快照。
4. 停止顺序是“设置stop标志→notify_all→join→销毁其他成员”。
5. 不要detach C++线程；否则对象销毁后线程可能继续访问 `this`。
6. 每个新线程都应增加队列长度、丢弃数、处理数和异常数诊断。
