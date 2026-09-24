// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// mimosa
#include "mimosa_msgs/msg/radar_manager_debug.hpp"

#include "mimosa/radar/factor.hpp"
#include "mimosa/radar/utils.hpp"
#include "mimosa/sensor_manager_base.hpp"

namespace mimosa
{
namespace radar
{
struct ManagerConfig
{
  SensorManagerBaseConfig base;

  // Radar-specific fields
  bool is_exposure_compensated = true;
  float range_min = 0.1;
  float range_max = 20.0;
  float threshold_azimuth_deg = 60.0;
  float threshold_elevation_deg = 60.0;
  float filter_min_db = 5;
  float frame_ms = 18.5;
  float noise_sigma = 0.1;
};

void declare_config(ManagerConfig & config);

class Manager : public SensorManagerBase<ManagerConfig, sensor_msgs::msg::PointCloud2>
{
private:
  // Member variables
  TargetVector valid_targets_;
  mimosa_msgs::msg::RadarManagerDebug debug_msg_;

  // Outputs
  rclcpp::Publisher<mimosa_msgs::msg::RadarManagerDebug>::SharedPtr pub_debug_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_points_;

public:
  Manager(
    rclcpp::Node & pnh, mimosa::imu::Manager::Ptr imu_manager,
    mimosa::graph::Manager::Ptr graph_manager);
  void callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) override;

private:
  template <typename PointT>
  void preprocess(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void getFactors(gtsam::NonlinearFactorGraph & new_factors);
};
}  // namespace radar
}  // namespace mimosa
