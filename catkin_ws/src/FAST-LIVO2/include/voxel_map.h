/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include "common_lib.h"
#include <Eigen/Dense>
#include <fstream>
#include <math.h>
#include <mutex>
#include <omp.h>
#include <pcl/common/io.h>
#include <ros/ros.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

static int voxel_plane_id = 0;

typedef struct VoxelMapConfig
{
  double max_voxel_size_;
  int max_layer_;
  int max_iterations_;
  std::vector<int> layer_init_num_;
  int max_points_num_;
  double planner_threshold_;
  double beam_err_;
  double dept_err_;
  double sigma_num_;
  bool is_pub_plane_map_;
  // LiDAR measurement update diagnostics and robust residual weighting.
  bool robust_weight_en_;
  double robust_huber_delta_;
  double robust_min_weight_;
  bool innovation_diag_en_;
  double innovation_warn_threshold_;
  // Optional ground-vehicle pseudo measurement n^T v = 0. The update is
  // applied only when a reliable ground normal can be estimated from the
  // current LiDAR point-to-plane correspondences.
  bool ground_velocity_constraint_en_;
  bool ground_velocity_constraint_post_vio_en_;
  // Diagnose and optionally limit only the ground-normal velocity increment
  // introduced by VIO. Unlike the older pseudo measurement, this guard does
  // not force n^T v to zero and does not contract covariance.
  bool vio_velocity_delta_diag_en_;
  bool vio_velocity_delta_guard_en_;
  double vio_velocity_delta_guard_trigger_;
  double vio_velocity_delta_guard_limit_;
  int vio_velocity_delta_guard_min_stable_frames_;
  // Guard only the state handed to the next IMU prediction.  The reference is
  // the previous LiDAR-updated velocity, not zero velocity.
  bool prediction_start_velocity_delta_diag_en_;
  bool prediction_start_velocity_delta_guard_en_;
  double prediction_start_velocity_delta_guard_trigger_;
  double prediction_start_velocity_delta_guard_limit_;
  int prediction_start_velocity_delta_guard_min_stable_frames_;
  double ground_velocity_sigma_;
  double ground_velocity_post_vio_min_velocity_change_;
  int ground_velocity_min_features_;
  double ground_velocity_max_normal_tilt_deg_;
  double ground_velocity_min_below_sensor_;
  double ground_velocity_max_below_sensor_;
  double ground_velocity_max_residual_;
  double ground_velocity_min_normal_confidence_;
  double ground_velocity_max_normal_change_deg_;
  double ground_velocity_max_abs_normal_velocity_;
  double ground_velocity_max_correction_;
  // Prediction-stage local-ground displacement pseudo measurement.
  // It constrains only displacement along a recently validated local ground
  // normal, so motion tangent to a ramp remains observable.
  bool prediction_ground_displacement_constraint_en_;
  double prediction_ground_displacement_sigma_;
  double prediction_ground_displacement_trigger_;
  double prediction_ground_displacement_max_abs_;
  double prediction_ground_displacement_max_correction_;
  double prediction_ground_displacement_correction_gain_;
  int prediction_ground_min_stable_frames_;
  int prediction_ground_max_reference_age_frames_;

  // config of local map sliding
  double sliding_thresh;
  bool map_sliding_en;
  int half_map_size;
} VoxelMapConfig;

typedef struct PointToPlane
{
  Eigen::Vector3d point_b_;
  Eigen::Vector3d point_w_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d center_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  M3D body_cov_;
  int layer_;
  double d_;
  double eigen_value_;
  bool is_valid_;
  float dis_to_plane_;
} PointToPlane;

typedef struct VoxelPlane
{
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d y_normal_;
  Eigen::Vector3d x_normal_;
  Eigen::Matrix3d covariance_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float mid_eigen_value_ = 1;
  float max_eigen_value_ = 1;
  float d_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
  bool is_init_ = false;
  int id_ = 0;
  bool is_update_ = false;
  VoxelPlane()
  {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

class VOXEL_LOCATION
{
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

// Hash value
namespace std
{
template <> struct hash<VOXEL_LOCATION>
{
  int64_t operator()(const VOXEL_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};
} // namespace std

struct DS_POINT
{
  float xyz[3];
  float intensity;
  int count = 0;
};

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

class VoxelOctoTree
{

public:
  VoxelOctoTree() = default;
  std::vector<pointWithVar> temp_points_;
  VoxelPlane *plane_ptr_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  VoxelOctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  std::vector<int> layer_init_num_;
  float quater_length_;
  float planer_threshold_;
  int points_size_threshold_;
  int update_size_threshold_;
  int max_points_num_;
  int max_layer_;
  int new_points_;
  bool init_octo_;
  bool update_enable_;

  VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold)
      : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
        planer_threshold_(planer_threshold)
  {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    for (int i = 0; i < 8; i++)
    {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new VoxelPlane;
  }

  ~VoxelOctoTree()
  {
    for (int i = 0; i < 8; i++)
    {
      delete leaves_[i];
    }
    delete plane_ptr_;
  }
  void init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane);
  void init_octo_tree();
  void cut_octo_tree();
  void UpdateOctoTree(const pointWithVar &pv);

  VoxelOctoTree *find_correspond(Eigen::Vector3d pw);
  VoxelOctoTree *Insert(const pointWithVar &pv);
};

void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config);

class VoxelMapManager
{
public:
  VoxelMapManager() = default;
  VoxelMapConfig config_setting_;
  int current_frame_id_ = 0;
  ros::Publisher voxel_map_pub_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_;

  PointCloudXYZI::Ptr feats_undistort_;
  PointCloudXYZI::Ptr feats_down_body_;
  PointCloudXYZI::Ptr feats_down_world_;

  M3D extR_;
  V3D extT_;
  float build_residual_time, ekf_time;
  float ave_build_residual_time = 0.0;
  float ave_ekf_time = 0.0;
  int scan_count = 0;
  StatesGroup state_;
  V3D position_last_;

  // Per-scan prediction/update diagnostics. These values are populated by
  // StateEstimation() and published by LIVMapper after the LiDAR update.
  V3D last_prediction_pos_ = V3D::Zero();
  V3D last_update_pos_ = V3D::Zero();
  V3D last_position_innovation_ = V3D::Zero();
  V3D last_rotation_innovation_ = V3D::Zero();
  double last_innovation_norm_ = 0.0;
  double last_average_residual_ = 0.0;
  double last_weighted_average_residual_ = 0.0;
  double last_max_abs_residual_ = 0.0;
  double last_mean_robust_weight_ = 1.0;
  double last_downweighted_ratio_ = 0.0;
  double last_z_information_ = 0.0;
  double last_z_information_ratio_ = 0.0;
  int last_downweighted_count_ = 0;
  int last_iteration_count_ = 0;

  // Ground-normal velocity soft-constraint diagnostics. skip_code mapping:
  // 0=applied, 1=disabled, 2=gravity_invalid, 3=not_enough_ground,
  // 4=normal_unreliable, 5=normal_change, 6=velocity_gate,
  // 7=covariance_invalid.
  bool last_ground_constraint_applied_ = false;
  int last_ground_constraint_skip_code_ = 1;
  int last_ground_feature_count_ = 0;
  V3D last_ground_normal_ = V3D::Zero();
  double last_ground_normal_confidence_ = 0.0;
  double last_normal_velocity_before_ = 0.0;
  double last_normal_velocity_after_ = 0.0;
  double last_normal_velocity_correction_ = 0.0;
  double last_ground_constraint_variance_ = 0.0;
  double last_ground_constraint_gain_norm_ = 0.0;
  VD(DIM_STATE) last_ground_constraint_state_delta_ = VD(DIM_STATE)::Zero();
  V3D last_lidar_update_pos_ = V3D::Zero();
  V3D last_lidar_update_vel_ = V3D::Zero();

  // The post-VIO update reuses the latest LiDAR-validated ground normal and
  // only corrects the normal-velocity change introduced by VIO. Keeping a
  // separate diagnostic block avoids confusing it with the LIO update above.
  bool last_post_vio_ground_constraint_applied_ = false;
  int last_post_vio_ground_constraint_skip_code_ = 1;
  double last_post_vio_ground_constraint_time_ = 0.0;
  double last_post_vio_reference_normal_velocity_ = 0.0;
  double last_post_vio_normal_velocity_before_ = 0.0;
  double last_post_vio_normal_velocity_after_ = 0.0;
  double last_post_vio_velocity_change_ = 0.0;
  double last_post_vio_normal_velocity_correction_ = 0.0;
  VD(DIM_STATE) last_post_vio_ground_constraint_state_delta_ = VD(DIM_STATE)::Zero();

  // VIO velocity-delta guard diagnostics. These values are written on the
  // following LIO diagnostic row because VIO runs after the LIO update.
  // skip_code: 0=applied, 1=disabled, 2=reference_invalid,
  // 3=reference_stale, 4=reference_not_stable, 5=below_trigger,
  // 6=velocity_invalid.
  bool last_vio_velocity_delta_guard_applied_ = false;
  int last_vio_velocity_delta_guard_skip_code_ = 1;
  double last_vio_velocity_delta_guard_time_ = 0.0;
  V3D last_vio_velocity_delta_world_ = V3D::Zero();
  double last_vio_velocity_delta_normal_before_ = 0.0;
  double last_vio_velocity_delta_normal_after_ = 0.0;
  double last_vio_velocity_delta_normal_correction_ = 0.0;
  V3D last_vio_velocity_delta_guard_state_dv_ = V3D::Zero();

  // Prediction-start velocity guard diagnostics. skip_code follows the VIO
  // delta guard mapping. These values belong to the current LIO row.
  bool last_prediction_start_velocity_delta_guard_applied_ = false;
  int last_prediction_start_velocity_delta_guard_skip_code_ = 1;
  V3D last_prediction_start_velocity_delta_world_ = V3D::Zero();
  double last_prediction_start_velocity_delta_normal_before_ = 0.0;
  double last_prediction_start_velocity_delta_normal_after_ = 0.0;
  double last_prediction_start_velocity_delta_normal_correction_ = 0.0;
  V3D last_prediction_start_velocity_delta_guard_state_dv_ = V3D::Zero();

  // Prediction-stage ground-displacement diagnostics. skip_code mapping:
  // 0=applied, 1=disabled, 2=reference_invalid, 3=reference_stale,
  // 4=reference_not_stable, 5=below_trigger, 6=displacement_gate,
  // 7=velocity_gate, 8=covariance_invalid.
  bool ground_reference_valid_ = false;
  int ground_reference_age_frames_ = 1000000;
  int ground_reference_stable_frames_ = 0;
  bool last_prediction_ground_constraint_applied_ = false;
  int last_prediction_ground_constraint_skip_code_ = 1;
  double last_prediction_ground_displacement_before_ = 0.0;
  double last_prediction_ground_displacement_after_ = 0.0;
  double last_prediction_ground_residual_ = 0.0;
  double last_prediction_ground_variance_ = 0.0;
  double last_prediction_ground_gain_norm_ = 0.0;
  VD(DIM_STATE) last_prediction_ground_constraint_state_delta_ = VD(DIM_STATE)::Zero();

  V3D last_slide_position = {0,0,0};

  geometry_msgs::Quaternion geoQuat_;

  int feats_down_size_;
  int effct_feat_num_;
  std::vector<M3D> cross_mat_list_;
  std::vector<M3D> body_cov_list_;
  std::vector<pointWithVar> pv_list_;
  std::vector<PointToPlane> ptpl_list_;

  VoxelMapManager(VoxelMapConfig &config_setting, std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map)
      : config_setting_(config_setting), voxel_map_(voxel_map)
  {
    current_frame_id_ = 0;
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
  };

  void StateEstimation(StatesGroup &state_propagat);
  bool ApplyPredictionGroundDisplacementConstraint(StatesGroup &predicted_state,
                                                   const StatesGroup &prediction_start_state);
  bool ApplyGroundVelocityConstraintAfterVIO(StatesGroup &target_state, double measurement_time);
  bool ApplyVioVelocityDeltaGuard(StatesGroup &target_state,
                                  const StatesGroup &state_before_vio,
                                  double measurement_time);
  bool ApplyPredictionStartVelocityDeltaGuard(StatesGroup &target_state);
  void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);

  void BuildVoxelMap();
  V3F RGBFromVoxel(const V3D &input_point);

  void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);

  void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);

  void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, double &prob,
                             PointToPlane &single_ptpl);

  void pubVoxelMap();

  void mapSliding();
  void clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min );

private:
  void ApplyGroundVelocityConstraint();
  void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list);

  void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane, const float alpha,
                      const Eigen::Vector3d rgb);
  void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q);

  void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
};
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

#endif // VOXEL_MAP_H_
