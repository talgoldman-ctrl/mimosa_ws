// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// OpenMP
#include <omp.h>

// GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// mimosa
#include "mimosa/lidar/geometric_config.hpp"
#include "mimosa/lidar/geometric_factor.hpp"
#include "mimosa/lidar/incremental_voxel_map.hpp"
#include "mimosa/lidar/utils.hpp"
#include "mimosa/state.hpp"
#include "mimosa/stopwatch.hpp"
#include "mimosa/utils.hpp"
#include "mimosa_msgs/msg/lidar_geometric_debug.hpp"

// ROS
#include <geometry_msgs/msg/pose_array.hpp>

namespace mimosa
{
namespace lidar
{
class Geometric
{
public:
  const GeometricConfig config;

private:
  std::unique_ptr<spdlog::logger> logger_;

  // Member variables
  pcl::PointCloud<Point>::Ptr Be_cloud_;
  pcl::PointCloud<Point> sm_Be_cloud_ds_;
  ICPFactor::Ptr factor_;

  // Variables for map
  IncrementalVoxelMapPCL::Ptr ivox_map_;
  std::vector<gtsam::Pose3> map_poses_;
  geometry_msgs::msg::PoseArray keyframe_poses_;

  // Variables for downsampling
  std::vector<FlatContainerMinimal> flat_voxels_;
  std::unordered_map<Eigen::Vector3i, size_t, XORVector3iHash> voxels_;
  std::vector<size_t> indices_;

  double ts_;
  mimosa_msgs::msg::LidarGeometricDebug debug_msg_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sm_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sm_cloud_ds_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_sm_correspondances_ma_;
  rclcpp::Publisher<mimosa_msgs::msg::LidarGeometricDebug>::SharedPtr pub_debug_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_localizability_marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_degen_marker_array_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_keyframe_poses_;

  void fillMarkerArray(
    const ICPFactor & factor, visualization_msgs::msg::MarkerArray & ma, const std::string & frame_id,
    const double ts);
  void downsample(
    const pcl::PointCloud<Point> & input_cloud, pcl::PointCloud<Point> & output_cloud,
    const double leaf_size, const size_t max_points_per_voxel = 20,
    const double min_dist_in_voxel = 0.1);

public:
  Geometric(rclcpp::Node & pnh);
  void preprocess(
    const pcl::PointCloud<Point> & points_deskewed, const std::vector<size_t> & idxs,
    const double ts);
  void getFactors(
    const gtsam::Key & key, const gtsam::Values & values, gtsam::NonlinearFactorGraph & graph,
    M66 & eigenvectors_block_matrix, V6D & degen_directions);
  void updateMap(const gtsam::Key key, const gtsam::Values & values);
  void publishDebug();
};
}  // namespace lidar
}  // namespace mimosa
