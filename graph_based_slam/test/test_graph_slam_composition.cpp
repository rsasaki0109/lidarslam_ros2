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

#include "graph_slam_composition.hpp"

TEST(GraphSlamComposition, BuildsDescriptorAndLoopSearchBoundaries)
{
  graphslam::GraphSlamConfig config;
  config.use_triangle_descriptor_ = true;
  config.triangle_descriptor_max_keypoints_ = 73;
  config.search_submap_num_ = 9;
  config.max_loop_candidate_count_ = 7;
  config.scan_context_query_stride_ = 0;
  config.loop_max_translation_delta_ = 12.5;
  config.registration_method = "GICP";
  config.voxel_leaf_size = 0.35;
  config.loop_search_query_stride_ = 3;

  const auto descriptor = graphslam::makeDescriptorConfig(config);
  const auto loop_search = graphslam::makeLoopSearchConfig(config);
  const auto application = graphslam::makeGraphSlamApplicationConfig(config);

  EXPECT_TRUE(descriptor.use_triangle_descriptor);
  EXPECT_EQ(descriptor.triangle_descriptor_max_keypoints, 73);
  EXPECT_EQ(loop_search.search_submap_num, 9);
  EXPECT_EQ(loop_search.aggregator.max_loop_candidate_count, 7);
  EXPECT_EQ(loop_search.aggregator.scan_context_query_stride, 1);
  EXPECT_DOUBLE_EQ(loop_search.gates.max_translation_m, 12.5);
  EXPECT_EQ(application.registration_method, "GICP");
  EXPECT_DOUBLE_EQ(application.voxel_leaf_size, 0.35);
  EXPECT_EQ(application.loop_search_query_stride, 3);
  EXPECT_EQ(application.loop_search.search_submap_num, 9);
}

TEST(GraphSlamComposition, KeepsAdaptiveStateOutsideStartupConfig)
{
  graphslam::GraphSlamConfig config;
  config.num_adjacent_pose_cnstraints_ = 8;
  config.adjacent_edge_info_auto_scale_ = true;
  config.adjacent_edge_info_auto_scale_split_trans_rot_ = true;
  config.adjacent_edge_info_auto_scale_ema_alpha_ = 0.42;
  config.loop_edge_robust_kernel_type_ = "cauchy";

  const auto result = graphslam::makePoseGraphConfig(config, 11.0, 22.0, 33.0);

  EXPECT_EQ(result.adjacent.num_adjacent_pose_constraints, 8);
  EXPECT_DOUBLE_EQ(result.adjacent.info_weight, 11.0);
  EXPECT_DOUBLE_EQ(result.adjacent.info_weight_trans, 22.0);
  EXPECT_DOUBLE_EQ(result.adjacent.info_weight_rot, 33.0);
  EXPECT_EQ(result.loop.robust_kernel_type, "cauchy");
  EXPECT_EQ(result.chi2_collection, graphslam::pose_graph::Chi2Collection::SPLIT);
  EXPECT_DOUBLE_EQ(result.auto_scale.ema_alpha, 0.42);
}

TEST(GraphSlamComposition, BuildsIoAndSensorPolicies)
{
  graphslam::GraphSlamConfig config;
  config.dynamic_object_filter_min_observations_ = 6;
  config.planar_map_filter_min_retained_ratio_ = 0.81;
  config.map_grid_size_x_ = 31.0;
  config.gnss_rtk_fix_weight_scale_ = 4.5;

  EXPECT_EQ(graphslam::makeDynamicObjectFilterConfig(config).min_observations, 6);
  EXPECT_DOUBLE_EQ(graphslam::makePlanarMapFilterConfig(config).min_retained_ratio, 0.81);
  EXPECT_DOUBLE_EQ(graphslam::makeGridConfig(config).grid_size_x, 31.0);
  EXPECT_DOUBLE_EQ(graphslam::makeGnssWeightingConfig(config).rtk_fix_weight_scale, 4.5);
}
