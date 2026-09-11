// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following disclaimer
//    in the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "graph_slam_config.hpp"

namespace
{

class GraphSlamConfigTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(GraphSlamConfigTest, DefaultsAreValid)
{
  const graphslam::GraphSlamConfig config;
  EXPECT_TRUE(graphslam::validateGraphSlamConfig(config).empty());
}

TEST_F(GraphSlamConfigTest, LoaderAppliesRosOverridesAndDeclaresCompleteSurface)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("registration_method", "GICP"),
      rclcpp::Parameter("voxel_leaf_size", 0.4),
      rclcpp::Parameter("use_gnss", true),
      rclcpp::Parameter("gnss_topic", "/fix/filtered"),
      rclcpp::Parameter("odom_cloud_sync_use_exact_time", true),
      rclcpp::Parameter("cloud_subscriber_qos_reliable", false),
  });
  auto node = std::make_shared<rclcpp::Node>("graph_slam_config_test", options);
  const auto parameters_before_load = node->list_parameters({}, 0).names.size();

  const auto config = graphslam::loadGraphSlamConfig(*node);

  EXPECT_EQ(config.registration_method, "GICP");
  EXPECT_DOUBLE_EQ(config.voxel_leaf_size, 0.4);
  EXPECT_TRUE(config.use_gnss_);
  EXPECT_EQ(config.gnss_topic_, "/fix/filtered");
  EXPECT_TRUE(config.odom_cloud_sync_use_exact_time_);
  EXPECT_FALSE(config.cloud_subscriber_qos_reliable_);
  EXPECT_TRUE(node->has_parameter("triangle_descriptor_keypoint_mode"));
  EXPECT_TRUE(node->has_parameter("degeneracy_diagnostics_csv_path"));
  EXPECT_EQ(node->list_parameters({}, 0).names.size() - parameters_before_load, 145U);
}

TEST_F(GraphSlamConfigTest, ValidationReportsIndependentConfigurationErrors)
{
  graphslam::GraphSlamConfig config;
  config.voxel_leaf_size = 0.0;
  config.map_grid_size_x_ = -1.0;
  config.odom_cloud_sync_queue_size_ = 0;
  config.use_pcd_cache_ = true;
  config.pcd_cache_dir_.clear();

  const auto errors = graphslam::validateGraphSlamConfig(config);

  EXPECT_EQ(errors.size(), 4U);
  EXPECT_NE(
    std::find(errors.begin(), errors.end(), "voxel_leaf_size must be finite and > 0"),
    errors.end());
  EXPECT_NE(
    std::find(errors.begin(), errors.end(),
      "pcd_cache_dir must not be empty when use_pcd_cache is true"),
    errors.end());
}

TEST_F(GraphSlamConfigTest, NormalizationRepairsDependentAndBoundedValues)
{
  graphslam::GraphSlamConfig config;
  config.adjacent_edge_info_weight_ = -2.0;
  config.adjacent_edge_info_weight_trans_ = -1.0;
  config.adjacent_edge_info_weight_rot_ = -1.0;
  config.gnss_covariance_min_variance_m2_ = 2.0;
  config.gnss_covariance_max_variance_m2_ = 1.0;
  config.loop_min_overlap_ratio_ = 2.0;
  config.triangle_descriptor_keypoint_mode_ = "unknown";
  config.planar_map_filter_min_retained_ratio_ = 1.5;

  const auto result = graphslam::normalizeGraphSlamConfig(config);

  EXPECT_FALSE(result.warnings.empty());
  EXPECT_DOUBLE_EQ(config.adjacent_edge_info_weight_, 1000.0);
  EXPECT_DOUBLE_EQ(config.adjacent_edge_info_weight_trans_, 1000.0);
  EXPECT_DOUBLE_EQ(config.adjacent_edge_info_weight_rot_, 1000.0);
  EXPECT_DOUBLE_EQ(config.gnss_covariance_max_variance_m2_, 2.0);
  EXPECT_DOUBLE_EQ(config.loop_min_overlap_ratio_, 0.0);
  EXPECT_EQ(config.triangle_descriptor_keypoint_mode_, "bev_max_height");
  EXPECT_DOUBLE_EQ(config.planar_map_filter_min_retained_ratio_, 1.0);
}

TEST_F(GraphSlamConfigTest, NormalizationIsIdempotent)
{
  graphslam::GraphSlamConfig config;
  config.search_submap_num_ = 0;
  config.bev_descriptor_sequence_threshold_ = -1.0;

  graphslam::normalizeGraphSlamConfig(config);
  const graphslam::GraphSlamConfig once = config;
  const auto second = graphslam::normalizeGraphSlamConfig(config);

  EXPECT_TRUE(second.warnings.empty());
  EXPECT_EQ(config.search_submap_num_, once.search_submap_num_);
  EXPECT_DOUBLE_EQ(
    config.bev_descriptor_sequence_threshold_, once.bev_descriptor_sequence_threshold_);
}

}  // namespace
