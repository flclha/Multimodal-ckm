/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIDAR_DETECT_HPP
#define LIDAR_DETECT_HPP
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include "common_lib.h"

class LidarDetect
{
private:
    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    double circle_radius_;
    double circle_center_width_, circle_center_height_;

    // 存储中间结果的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr center_z0_cloud_;

public:
    ros::Publisher filtered_pub_;
    ros::Publisher plane_pub_;
    ros::Publisher aligned_pub_;
    ros::Publisher edge_pub_;
    ros::Publisher center_z0_pub_;
    ros::Publisher center_pub_;

    LidarDetect(ros::NodeHandle &nh, Params &params)
        : filtered_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          plane_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          aligned_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          edge_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          center_z0_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        x_min_ = params.x_min;
        x_max_ = params.x_max;
        y_min_ = params.y_min;
        y_max_ = params.y_max;
        z_min_ = params.z_min;
        z_max_ = params.z_max;
        circle_radius_ = params.circle_radius;
        circle_center_width_ = params.delta_width_circles;
        circle_center_height_ = params.delta_height_circles;

        filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
        plane_pub_ = nh.advertise<sensor_msgs::PointCloud2>("plane_cloud", 1);
        aligned_pub_ = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 1);
        edge_pub_ = nh.advertise<sensor_msgs::PointCloud2>("edge_cloud", 1);
        center_z0_pub_ = nh.advertise<sensor_msgs::PointCloud2>("center_z0_cloud", 10);
        center_pub_ = nh.advertise<sensor_msgs::PointCloud2>("center_cloud", 10);
    }

    void detect_lidar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud)
    {
        // 1. X、Y、Z方向滤波
        filtered_cloud_->reserve(cloud->size());

        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_min_, x_max_);  // 设置X轴范围
        pass_x.filter(*filtered_cloud_);
    
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(filtered_cloud_);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(y_min_, y_max_);  // 设置Y轴范围
        pass_y.filter(*filtered_cloud_);
    
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(filtered_cloud_);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(z_min_, z_max_);  // 设置Z轴范围
        pass_z.filter(*filtered_cloud_);
    
        ROS_INFO("Filtered cloud size: %ld", filtered_cloud_->size());
        
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(filtered_cloud_);
        voxel_filter.setLeafSize(0.005f, 0.005f, 0.005f);
        voxel_filter.filter(*filtered_cloud_);
        ROS_INFO("Filtered cloud size: %ld", filtered_cloud_->size());

        // 2. 平面分割
        plane_cloud_->reserve(filtered_cloud_->size());

        pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation;
        plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
        plane_segmentation.setMethodType(pcl::SAC_RANSAC);
        plane_segmentation.setDistanceThreshold(0.01);  // 平面分割阈值
        plane_segmentation.setInputCloud(filtered_cloud_);
        plane_segmentation.segment(*plane_inliers, *plane_coefficients);
    
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(filtered_cloud_);
        extract.setIndices(plane_inliers);
        extract.filter(*plane_cloud_);
        ROS_INFO("Plane cloud size: %ld", plane_cloud_->size());
    
        // 3. 平面点云对齐   
        aligned_cloud_->reserve(plane_cloud_->size());

        Eigen::Vector3d normal(plane_coefficients->values[0],
            plane_coefficients->values[1],
            plane_coefficients->values[2]);
        normal.normalize();
        Eigen::Vector3d z_axis(0, 0, 1);

        // Eigen::AngleAxis requires a unit-length axis.  The previous
        // normal.cross(z_axis) axis was not normalized, which made R contain
        // scale/shear and distorted the recovered hole spacing.  FromTwoVectors
        // produces a proper orthonormal rotation and also handles the nearly
        // parallel/antiparallel cases safely.
        Eigen::Quaterniond alignment =
            Eigen::Quaterniond::FromTwoVectors(normal, z_axis);
        Eigen::Matrix3d R = alignment.toRotationMatrix();

        // 应用旋转矩阵，将平面对齐到 Z=0 平面
        float average_z = 0.0;
        int cnt = 0;
        for (const auto& pt : *plane_cloud_) {
            Eigen::Vector3d point(pt.x, pt.y, pt.z);
            Eigen::Vector3d aligned_point = R * point;
            aligned_cloud_->push_back(pcl::PointXYZ(aligned_point.x(), aligned_point.y(), 0.0));
            average_z += aligned_point.z();
            cnt++;
        }
        average_z /= cnt;

        // 4. 提取边缘点
        edge_cloud_->reserve(aligned_cloud_->size());

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        normal_estimator.setInputCloud(aligned_cloud_);
        normal_estimator.setRadiusSearch(0.03); // 设置法线估计的搜索半径
        normal_estimator.compute(*normals);
    
        pcl::PointCloud<pcl::Boundary> boundaries;
        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimator;
        boundary_estimator.setInputCloud(aligned_cloud_);
        boundary_estimator.setInputNormals(normals);
        boundary_estimator.setRadiusSearch(0.03); // 设置边界检测的搜索半径
        boundary_estimator.setAngleThreshold(M_PI / 4); // 设置角度阈值
        boundary_estimator.compute(boundaries);
    
        for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
            if (boundaries.points[i].boundary_point > 0) {
                edge_cloud_->push_back(aligned_cloud_->points[i]);
            }
        }
        ROS_INFO("Extracted %ld edge points.", edge_cloud_->size());

        // 5. 对边缘点进行聚类
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(edge_cloud_);
    
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02); // 设置聚类距离阈值
        ec.setMinClusterSize(50);     // 最小点数
        // 标定板边缘噪点可能把圆孔连成一个大聚类；后续已经有严格的
        // 半径约束和矩形几何验证，因此这里不能用 1000 点提前丢弃它。
        ec.setMaxClusterSize(10000);  // 最大点数
        ec.setSearchMethod(tree);
        ec.setInputCloud(edge_cloud_);
        ec.extract(cluster_indices);
    
        ROS_INFO("Number of edge clusters: %ld", cluster_indices.size());
    
        // 6. 对每个聚类进行带半径约束的迭代圆拟合。
        // 背景边缘有时会把两个圆孔连成一个大聚类。旧实现每个聚类只拟合
        // 一次，会优先得到一个巨大伪圆并丢掉两个真实圆。这里限定物理半径，
        // 每找到一个圆就移除它的环带，再在同一聚类中继续寻找。
        center_z0_cloud_->reserve(4);
        Eigen::Matrix3d R_inv = R.inverse();
        std::vector<pcl::PointXYZ> circle_candidates;

        for (size_t i = 0; i < cluster_indices.size(); ++i) 
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : cluster_indices[i].indices) {
                remaining->push_back(edge_cloud_->points[idx]);
            }

            // Large connected components can contain board edges plus both
            // right-hand holes. Keep several hypotheses; the rectangle check
            // below decides which four are physically consistent.
            for (int pass = 0; pass < 8 && remaining->size() >= 50; ++pass)
            {
                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_CIRCLE2D);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(0.005);
                seg.setMaxIterations(2000);
                // Keep the physical-radius prior reasonably tight.  A broad
                // +/-30% interval lets the board border and room edges form a
                // larger pseudo-circle whose center is shifted by centimetres.
                const double radius_tolerance = std::max(0.012, circle_radius_ * 0.15);
                seg.setRadiusLimits(std::max(0.02, circle_radius_ - radius_tolerance),
                                    circle_radius_ + radius_tolerance);
                seg.setInputCloud(remaining);
                seg.segment(*inliers, *coefficients);

                if (inliers->indices.empty() || coefficients->values.size() < 3)
                {
                    ROS_WARN("Circle cluster %zu pass %d: points=%zu, constrained RANSAC returned no circle.",
                             i, pass, remaining->size());
                    break;
                }

                const double cx = coefficients->values[0];
                const double cy = coefficients->values[1];
                const double fitted_radius = coefficients->values[2];
                double fit_error = 0.0;
                for (const auto& idx : inliers->indices) 
                {
                    const double dx = remaining->points[idx].x - cx;
                    const double dy = remaining->points[idx].y - cy;
                    fit_error += std::abs(std::sqrt(dx * dx + dy * dy) - fitted_radius);
                }
                fit_error /= inliers->indices.size();
                const bool enough_inliers = inliers->indices.size() >= 30;
                const bool good_fit = fit_error < 0.012;
                bool duplicate = false;
                for (const auto& prior : circle_candidates)
                {
                    duplicate = duplicate || std::hypot(prior.x - cx, prior.y - cy) < 0.05;
                }
                const bool accepted = enough_inliers && good_fit && !duplicate;
                ROS_INFO("Circle cluster %zu pass %d: remaining=%zu inliers=%zu center=(%.5f,%.5f) radius=%.5f m fit_error=%.5f m accepted=%s",
                         i, pass, remaining->size(), inliers->indices.size(),
                         cx, cy, fitted_radius, fit_error, accepted ? "yes" : "no");
                if (accepted)
                {
                    circle_candidates.emplace_back(cx, cy, 0.0f);
                }

                // Remove the complete annulus, not just RANSAC inliers. This
                // prevents the same physical hole from being detected twice.
                pcl::PointCloud<pcl::PointXYZ>::Ptr next(new pcl::PointCloud<pcl::PointXYZ>);
                next->reserve(remaining->size());
                for (const auto& point : *remaining)
                {
                    const double radial = std::hypot(point.x - cx, point.y - cy);
                    if (std::abs(radial - fitted_radius) > 0.025)
                        next->push_back(point);
                }
                remaining.swap(next);
            }
        }

        ROS_INFO("Radius-constrained circle candidates: %zu", circle_candidates.size());

        // Select exactly one geometrically valid rectangle from all candidates.
        // Do not stop at the first combination: a background arc can occasionally
        // pass the 6% gate before the four real holes are visited.  Rank every
        // valid rectangle by its normalized side-length residual and keep the
        // best one.
        std::vector<pcl::PointXYZ> selected;
        double best_geometry_score = 1e9;
        int valid_rectangles = 0;
        std::array<double, 4> best_sides{{0.0, 0.0, 0.0, 0.0}};
        for (size_t a = 0; a < circle_candidates.size(); ++a)
          for (size_t b = a + 1; b < circle_candidates.size(); ++b)
            for (size_t c = b + 1; c < circle_candidates.size(); ++c)
              for (size_t d = c + 1; d < circle_candidates.size(); ++d)
              {
                  std::vector<pcl::PointXYZ> combination{
                      circle_candidates[a], circle_candidates[b],
                      circle_candidates[c], circle_candidates[d]};
                  Square target(combination, circle_center_width_, circle_center_height_);
                  if (!target.is_valid()) continue;

                  ++valid_rectangles;
                  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
                  pcl::PointCloud<pcl::PointXYZ>::Ptr ordered(new pcl::PointCloud<pcl::PointXYZ>);
                  for (const auto& point : combination) raw->push_back(point);
                  sortPatternCenters(raw, ordered, "camera");

                  std::array<double, 4> sides;
                  for (int side = 0; side < 4; ++side)
                  {
                      const auto& p = ordered->points[side];
                      const auto& q = ordered->points[(side + 1) % 4];
                      sides[side] = std::hypot(p.x - q.x, p.y - q.y);
                  }

                  auto pattern_score = [&](double first, double second)
                  {
                      double score = 0.0;
                      const double expected[4] = {first, second, first, second};
                      for (int side = 0; side < 4; ++side)
                      {
                          const double normalized = (sides[side] - expected[side]) /
                                                    expected[side];
                          score += normalized * normalized;
                      }
                      return score;
                  };
                  const double score = std::min(
                      pattern_score(circle_center_width_, circle_center_height_),
                      pattern_score(circle_center_height_, circle_center_width_));
                  if (score < best_geometry_score)
                  {
                      best_geometry_score = score;
                      best_sides = sides;
                      selected = combination;
                  }
              }

        if (selected.empty())
        {
            ROS_ERROR("No set of four circles matches target geometry %.3f x %.3f m.",
                      circle_center_width_, circle_center_height_);
            return;
        }

        ROS_INFO("Selected best of %d valid rectangles: geometry_score=%.8f sides=%.5f,%.5f,%.5f,%.5f m",
                 valid_rectangles, best_geometry_score, best_sides[0], best_sides[1],
                 best_sides[2], best_sides[3]);
        for (size_t i = 0; i < selected.size(); ++i)
        {
            ROS_INFO("Selected circle %zu: center=(%.5f,%.5f)",
                     i, selected[i].x, selected[i].y);
        }

        // The four holes belong to a manufactured rigid rectangle.  A circle
        // fitted independently to a finite/thick LiDAR boundary can have a
        // centimetre-level radial sampling bias, especially when room edges
        // connect to the hole contour.  Estimate the board pose from all four
        // raw centers and compare them with the exact physical template.  This
        // is a diagnostic/rejection test only; the raw centers remain the
        // inputs to FAST-Calib's cross-sensor SVD.
        pcl::PointCloud<pcl::PointXYZ>::Ptr measured_ordered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr measured_raw(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : selected) measured_raw->push_back(point);
        sortPatternCenters(measured_raw, measured_ordered, "camera");

        std::array<double, 4> measured_sides;
        for (int side = 0; side < 4; ++side)
        {
            const auto& p = measured_ordered->points[side];
            const auto& q = measured_ordered->points[(side + 1) % 4];
            measured_sides[side] = std::hypot(p.x - q.x, p.y - q.y);
        }
        auto side_pattern_score = [&](double first, double second)
        {
            const double expected[4] = {first, second, first, second};
            double score = 0.0;
            for (int side = 0; side < 4; ++side)
            {
                const double normalized = (measured_sides[side] - expected[side]) /
                                          expected[side];
                score += normalized * normalized;
            }
            return score;
        };
        double first_side = circle_center_width_;
        double second_side = circle_center_height_;
        if (side_pattern_score(circle_center_height_, circle_center_width_) <
            side_pattern_score(circle_center_width_, circle_center_height_))
        {
            first_side = circle_center_height_;
            second_side = circle_center_width_;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr ideal_pattern(new pcl::PointCloud<pcl::PointXYZ>);
        // Clockwise order, matching sortPatternCenters().
        ideal_pattern->push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
        ideal_pattern->push_back(pcl::PointXYZ(0.0f, first_side, 0.0f));
        ideal_pattern->push_back(pcl::PointXYZ(second_side, first_side, 0.0f));
        ideal_pattern->push_back(pcl::PointXYZ(second_side, 0.0f, 0.0f));

        Eigen::Matrix4f pattern_pose;
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> pattern_svd;
        pattern_svd.estimateRigidTransformation(*ideal_pattern, *measured_ordered, pattern_pose);
        pcl::PointCloud<pcl::PointXYZ>::Ptr constrained_pattern(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*ideal_pattern, *constrained_pattern, pattern_pose);

        double raw_model_squared_error = 0.0;
        double raw_model_max_error = 0.0;
        for (int i = 0; i < 4; ++i)
        {
            const double dx = measured_ordered->points[i].x - constrained_pattern->points[i].x;
            const double dy = measured_ordered->points[i].y - constrained_pattern->points[i].y;
            const double error = std::hypot(dx, dy);
            raw_model_squared_error += error * error;
            raw_model_max_error = std::max(raw_model_max_error, error);
        }
        const double raw_model_rmse = std::sqrt(raw_model_squared_error / 4.0);
        ROS_INFO("Raw LiDAR target-model fit: RMSE=%.5f m max=%.5f m (limits 0.012/0.020 m)",
                 raw_model_rmse, raw_model_max_error);
        if (raw_model_rmse > 0.012 || raw_model_max_error > 0.020)
        {
            ROS_ERROR("LiDAR circles do not fit the rigid target accurately enough; refusing constrained refinement.");
            return;
        }

        // Keep FAST-Calib's original calibration principle: the independently
        // detected LiDAR centers, rather than geometry-projected centers, are
        // passed to the cross-sensor SVD.  The rigid template above is only a
        // diagnostic/rejection test; otherwise one view would be forced to an
        // unrealistically near-zero calibration residual.
        ROS_INFO("Rigid %.3f x %.3f m target check passed; keeping raw LiDAR centers for SVD.",
                 circle_center_width_, circle_center_height_);

        for (const auto& center_point : selected)
        {
            center_z0_cloud_->push_back(center_point);
            Eigen::Vector3d aligned_point(center_point.x, center_point.y,
                                          center_point.z + average_z);
            Eigen::Vector3d original_point = R_inv * aligned_point;
            center_cloud->push_back(pcl::PointXYZ(original_point.x(),
                                                  original_point.y(),
                                                  original_point.z()));
        }
    }

    // 获取中间结果的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud() const { return filtered_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPlaneCloud() const { return plane_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getAlignedCloud() const { return aligned_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getEdgeCloud() const { return edge_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCenterZ0Cloud() const { return center_z0_cloud_; }
};

typedef std::shared_ptr<LidarDetect> LidarDetectPtr;

#endif
