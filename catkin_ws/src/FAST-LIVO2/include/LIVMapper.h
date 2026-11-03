/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <vikit/camera_loader.h>
#include <cstdint>

// ==================== 后台线程模板：预留头文件 ====================
// 当你准备启用文件末尾的 BackgroundWorker 模板时，再取消下面注释。
// 当前全部保持注释，不参与编译，不改变 FAST-LIVO 行为。
// #include <condition_variable>
// #include <deque>
// #include <memory>
// #include <mutex>
// #include <thread>

struct LioPredictionDiagnostic
{
  int imu_sample_count = 0;
  double imu_start_time = 0.0;
  double imu_end_time = 0.0;
  double prediction_dt = 0.0;
  V3D raw_acc_mean = V3D::Zero();
  V3D raw_acc_std = V3D::Zero();
  V3D raw_gyro_mean = V3D::Zero();
  V3D raw_gyro_std = V3D::Zero();
  double raw_acc_peak_deviation = 0.0;
  double raw_gyro_peak_norm = 0.0;
  double world_acc_z_mean = 0.0;
  double world_acc_z_rms = 0.0;
  double world_acc_z_peak_abs = 0.0;
};

struct RGBColorizationDiagnosticCounters
{
  uint64_t total_lidar_points = 0;
  uint64_t colorized_points = 0;
  uint64_t out_of_camera_fov = 0;
  uint64_t projection_outside_image = 0;
  uint64_t behind_camera = 0;
  uint64_t depth_invalid = 0;
  uint64_t occluded_zbuffer_rejected = 0;
  uint64_t raycast_rejected = 0;
  uint64_t time_not_matched = 0;
  uint64_t image_not_available = 0;
  uint64_t invalid_rgb = 0;
  uint64_t other = 0;
  // This is not a failure: the current implementation accepts black pixels.
  uint64_t accepted_black_rgb = 0;

  void add(const RGBColorizationDiagnosticCounters &rhs)
  {
    total_lidar_points += rhs.total_lidar_points;
    colorized_points += rhs.colorized_points;
    out_of_camera_fov += rhs.out_of_camera_fov;
    projection_outside_image += rhs.projection_outside_image;
    behind_camera += rhs.behind_camera;
    depth_invalid += rhs.depth_invalid;
    occluded_zbuffer_rejected += rhs.occluded_zbuffer_rejected;
    raycast_rejected += rhs.raycast_rejected;
    time_not_matched += rhs.time_not_matched;
    image_not_available += rhs.image_not_available;
    invalid_rgb += rhs.invalid_rgb;
    other += rhs.other;
    accepted_black_rgb += rhs.accepted_black_rgb;
  }
};

class LIVMapper
{
public:
  LIVMapper(ros::NodeHandle &nh);
  ~LIVMapper();
  void initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it);
  void initializeComponents();
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void savePCD();
  void flushPendingPCD();
  void processImu();
  
  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback(const ros::TimerEvent &e);
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
 
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
  void writeRGBColorizationDiagnostic(const char *scope, double image_time,
                                      const RGBColorizationDiagnosticCounters &frame);
  void writeRGBColorizationFinal();
  void publish_visual_sub_map(const ros::Publisher &pubSubVisualMap);
  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);
  void publish_mavros(const ros::Publisher &mavros_pose_publisher);
  void publish_path(const ros::Publisher pubPath);
  void publish_lio_update_diagnostic();
  void readParameters(ros::NodeHandle &nh);
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  string root_dir;
  string lid_topic, imu_topic, seq_name, img_topic;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double latest_lidar_begin_time = -1.0, latest_lidar_end_time = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int pcd_save_interval = -1, pcd_index = 0, pcd_scan_wait_num = 0;
  std::string pcd_output_dir;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;
  StatesGroup lio_prediction_start_state;
  StatesGroup previous_lio_update_state;
  LioPredictionDiagnostic lio_prediction_diag;
  bool previous_lio_update_valid = false;
  double previous_lio_diagnostic_time = -1.0;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::Imu> prop_imu_buffer;
  sensor_msgs::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::Odometry imu_prop_odom;
  ros::Publisher pubImuPropOdom;
  ros::Publisher pubLioPredictionOdom;
  ros::Publisher pubLioUpdateOdom;
  ros::Publisher pubLioInnovation;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  double IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;
  // 完整注册LiDAR点云单独分块保存，不与RGB着色点云混用。
  PointCloudXYZI::Ptr pcl_wait_save_lidar;

  ofstream fout_pre, fout_out, fout_pcd_pos, fout_points;
  ofstream fout_rgb_color_diag;
  ofstream fout_lio_innovation;
  std::string lio_innovation_csv_path;
  std::string rgb_color_diag_path;
  RGBColorizationDiagnosticCounters rgb_color_diag_total;
  uint64_t rgb_color_diag_frame_index = 0;
  uint64_t rgb_diag_time_mismatch_image_drop_events = 0;
  uint64_t rgb_diag_image_jump_drop_events = 0;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::Path path;
  nav_msgs::Odometry odomAftMapped;
  geometry_msgs::Quaternion geoQuat;
  geometry_msgs::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;

  ros::Publisher plane_pub;
  ros::Publisher voxel_pub;
  ros::Subscriber sub_pcl;
  ros::Subscriber sub_imu;
  ros::Subscriber sub_img;
  ros::Publisher pubLaserCloudFullRes;
  ros::Publisher pubLaserCloudRaw;
  ros::Publisher pubNormal;
  ros::Publisher pubSubVisualMap;
  ros::Publisher pubLaserCloudEffect;
  ros::Publisher pubLaserCloudMap;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubPath;
  ros::Publisher pubLaserCloudDyn;
  ros::Publisher pubLaserCloudDynRmed;
  ros::Publisher pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  ros::Publisher mavros_pose_publisher;
  ros::Timer imu_prop_timer;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
  int colmap_async_queue_size = 16;
  int colmap_png_compression = 1;
  int colmap_save_stride = 1;

  // ========================================================================
  // 新后台线程模板（当前完全注释，不会创建任何线程）
  //
  // 适用场景：写盘、统计、轻量级可视化、不参与当前帧状态估计的后处理。
  // 不适用：在后台线程中直接修改 _state、voxel_map、VIO/LIO 中间量。
  // 原因：它们由建图主线程持续读写，未加锁会引入竞争和崩溃。
  //
  // 使用步骤：
  // 1. 取消文件顶部线程头文件的注释。
  // 2. 取消下面声明的注释，按任务需求修改 BackgroundJob。
  // 3. 在 LIVMapper.cpp 中取消对应实现和 start/stop 调用的注释。
  // 4. 主线程只投递“不可变快照”，队列满时丢弃新任务，不阻塞SLAM。
  // ========================================================================
  // struct BackgroundJob
  // {
  //   double sensor_time = 0.0;
  //   std::shared_ptr<const PointCloudXYZI> cloud_snapshot;
  //   // TODO: 在这里增加你的任务数据，不要存放指向临时局部变量的指针。
  // };
  //
  // void startBackgroundWorker();
  // void stopBackgroundWorker();
  // void backgroundWorkerLoop();
  // bool tryEnqueueBackgroundJob(BackgroundJob job);
  //
  // std::deque<BackgroundJob> background_queue_;
  // std::mutex background_mutex_;
  // std::condition_variable background_cv_;
  // std::thread background_thread_;
  // bool background_started_ = false;       // 只由主线程读写
  // bool background_stop_ = false;          // 在 background_mutex_ 保护下读写
  // size_t background_queue_limit_ = 8;     // 有界队列，防止延迟/内存无限累积
  // uint64_t background_enqueued_ = 0;
  // uint64_t background_processed_ = 0;
  // uint64_t background_dropped_ = 0;
};
#endif
