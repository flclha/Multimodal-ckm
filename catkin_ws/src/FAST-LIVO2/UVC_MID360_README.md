# FAST-LIVO2 UVC + MID360 配置说明

## 📁 创建的文件

### 1. 主配置文件
- **uvc_mid360.yaml**: FAST-LIVO2主配置文件，配置UVC摄像头和MID360雷达

### 2. 摄像头配置文件
- **camera_uvc.yaml**: UVC摄像头内参配置文件

### 3. Launch文件
- **mapping_uvc_mid360.launch**: 启动UVC摄像头、MID360雷达和FAST-LIVO2的launch文件

## 🔧 配置参数说明

### UVC摄像头配置 (uvc_mid360.yaml)
```yaml
common:
  img_topic: "/uvc_camera/image"    # UVC摄像头图像话题
  lid_topic: "/livox/lidar"         # MID360雷达点云话题
  imu_topic: "/livox/imu"           # MID360 IMU话题

extrin_calib:
  # 需要根据实际标定结果调整
  extrinsic_T: [0.0, 0.0, 0.0]      # 外参平移
  extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # 外参旋转
  Rcl: [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0]  # 相机到激光雷达的旋转矩阵
  Pcl: [0.0, 0.0, 0.0]              # 相机到激光雷达的平移向量

preprocess:
  lidar_type: 1                     # Livox MID360 (使用AVIA类型)
  scan_line: 6                      # 扫描线数
  blind: 1.0                        # 盲区距离
```

### 摄像头内参配置 (camera_uvc.yaml)
```yaml
cam_model: Pinhole
cam_width: 1920                     # 图像宽度
cam_height: 1080                    # 图像高度
scale: 0.5                          # 图像缩放比例
cam_fx: 1000.0                      # 焦距x (需要标定)
cam_fy: 1000.0                      # 焦距y (需要标定)
cam_cx: 960.0                       # 光心x (需要标定)
cam_cy: 540.0                       # 光心y (需要标定)
cam_d0: 0.0                         # 畸变参数 (需要标定)
cam_d1: 0.0                         # 畸变参数 (需要标定)
cam_d2: 0.0                         # 畸变参数 (需要标定)
cam_d3: 0.0                         # 畸变参数 (需要标定)
```

## 🚀 使用方法

### 1. 启动系统
```bash
roslaunch fast_livo mapping_uvc_mid360.launch
```

### 2. 单独启动各组件
```bash
# 启动UVC摄像头
roslaunch mvs_ros_driver uvc_camera.launch

# 启动MID360雷达 (根据你使用的驱动选择)
roslaunch livox_ros_driver2 livox_lidar.launch  # 或其他相应的launch文件

# 启动FAST-LIVO2
roslaunch fast_livo mapping_uvc_mid360.launch
```

## ⚙️ 重要配置步骤

### 1. 摄像头标定
在使用前，请使用相机标定工具标定UVC摄像头，获取准确的内参，并更新 `camera_uvc.yaml` 文件。

推荐使用：
- ROS camera_calibration包
- MATLAB相机标定工具箱
- OpenCV标定工具

### 2. 外参标定
需要标定UVC摄像头和MID360雷达之间的相对位姿，更新 `uvc_mid360.yaml` 中的 `extrin_calib` 参数。

推荐使用：
- FAST-Calib工具包 (https://github.com/hku-mars/FAST-Calib)
- 手动测量并计算变换矩阵

### 3. 话题名称确认
确保以下话题名称与你的传感器驱动发布的话题匹配：
- 图像话题: `/uvc_camera/image`
- 点云话题: `/livox/lidar`
- IMU话题: `/livox/imu`

## 📝 注意事项

1. **分辨率**: UVC摄像头配置为1920x1080，请根据实际摄像头调整
2. **雷达类型**: MID360使用lidar_type: 1 (AVIA)，如果不兼容可能需要调整
3. **时间同步**: 确保摄像头和雷达的时间戳同步
4. **网络配置**: 如果使用网络摄像头，确保网络延迟最小
5. **性能**: 根据你的硬件性能调整算法参数

## 🔧 故障排除

### 常见问题
1. **图像话题不存在**: 检查UVC摄像头驱动是否正常启动
2. **点云话题不存在**: 检查MID360雷达驱动配置
3. **标定参数错误**: 使用标定工具重新标定传感器
4. **时间戳不同步**: 检查传感器的时间同步设置

### 调试命令
```bash
# 检查话题列表
rostopic list

# 检查图像话题
rostopic echo /uvc_camera/image

# 检查点云话题
rostopic echo /livox/lidar

# 查看FAST-LIVO2日志
rosnode info /laserMapping
```
