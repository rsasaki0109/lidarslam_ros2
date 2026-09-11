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

#include "graph_slam_composition.hpp"

#include <algorithm>

namespace graphslam
{

backend_core::DescriptorConfig makeDescriptorConfig(const GraphSlamConfig & config)
{
  backend_core::DescriptorConfig result;
  result.use_scan_context = config.use_scan_context_;
  result.use_bev_descriptor = config.use_bev_descriptor_;
  result.use_solid_descriptor = config.use_solid_descriptor_;
  result.use_triangle_descriptor = config.use_triangle_descriptor_;
  result.bev_descriptor_grid_size_m = config.bev_descriptor_grid_size_m_;
  result.bev_descriptor_grid_cells = config.bev_descriptor_grid_cells_;
  result.bev_descriptor_yaw_bins = config.bev_descriptor_yaw_bins_;
  result.triangle_descriptor_keypoint_mode = config.triangle_descriptor_keypoint_mode_;
  result.triangle_descriptor_grid_size_m = config.triangle_descriptor_grid_size_m_;
  result.triangle_descriptor_grid_cells = config.triangle_descriptor_grid_cells_;
  result.triangle_descriptor_min_salience_m = config.triangle_descriptor_min_salience_m_;
  result.triangle_descriptor_max_keypoints = config.triangle_descriptor_max_keypoints_;
  result.triangle_descriptor_edge_voxel_size_m = config.triangle_descriptor_edge_voxel_size_m_;
  result.triangle_descriptor_edge_neighbor_radius_m =
    config.triangle_descriptor_edge_neighbor_radius_m_;
  result.triangle_descriptor_edge_min_neighbors = config.triangle_descriptor_edge_min_neighbors_;
  result.triangle_descriptor_edge_min_edgeness = config.triangle_descriptor_edge_min_edgeness_;
  result.triangle_descriptor_edge_nms_radius_m = config.triangle_descriptor_edge_nms_radius_m_;
  result.triangle_descriptor_min_edge_m = config.triangle_descriptor_min_edge_m_;
  result.triangle_descriptor_max_edge_m = config.triangle_descriptor_max_edge_m_;
  result.triangle_descriptor_max_triangles = config.triangle_descriptor_max_triangles_;
  result.triangle_descriptor_edge_bin_m = config.triangle_descriptor_edge_bin_m_;
  result.triangle_descriptor_quad_feature_bin_m = config.triangle_descriptor_quad_feature_bin_m_;
  return result;
}

backend_core::LoopSearchConfig makeLoopSearchConfig(const GraphSlamConfig & config)
{
  backend_core::LoopSearchConfig result;
  result.search_submap_num = config.search_submap_num_;
  result.prefer_scan_context_candidates = config.prefer_scan_context_candidates_;
  result.use_3d_bbs_for_scan_context = config.use_3d_bbs_for_scan_context_;
  result.three_d_bbs_source_submap_num = config.three_d_bbs_source_submap_num_;
  result.three_d_bbs_target_submap_radius = config.three_d_bbs_target_submap_radius_;
  result.three_d_bbs_voxel_leaf_size = config.three_d_bbs_voxel_leaf_size_;
  result.three_d_bbs_min_level_res = config.three_d_bbs_min_level_res_;
  result.three_d_bbs_max_level = config.three_d_bbs_max_level_;
  result.three_d_bbs_score_threshold_percentage =
    config.three_d_bbs_score_threshold_percentage_;
  result.three_d_bbs_timeout_msec = config.three_d_bbs_timeout_msec_;
  result.three_d_bbs_num_threads = config.three_d_bbs_num_threads_;
  result.three_d_bbs_translation_search_margin_m =
    config.three_d_bbs_translation_search_margin_m_;
  result.three_d_bbs_roll_pitch_search_deg = config.three_d_bbs_roll_pitch_search_deg_;
  result.three_d_bbs_yaw_search_deg = config.three_d_bbs_yaw_search_deg_;

  candidate_aggregator::Config & aggregator = result.aggregator;
  aggregator.debug = config.debug_flag_;
  aggregator.max_loop_candidate_count = config.max_loop_candidate_count_;
  aggregator.distance_loop_closure = config.distance_loop_closure_;
  aggregator.range_of_searching_loop_closure = config.range_of_searching_loop_closure_;
  aggregator.scan_context_threshold = config.scan_context_threshold_;
  aggregator.scan_context_query_stride = std::max(1, config.scan_context_query_stride_);
  aggregator.scan_context_exclude_recent = std::max(1, config.scan_context_exclude_recent_);
  aggregator.bev_use_mutual_visibility = config.bev_use_mutual_visibility_;
  aggregator.bev_mutual_visibility_min_overlap_ratio =
    config.bev_mutual_visibility_min_overlap_ratio_;
  aggregator.bev_mutual_visibility_occupancy_eps = config.bev_mutual_visibility_occupancy_eps_;
  aggregator.bev_descriptor_yaw_bins = config.bev_descriptor_yaw_bins_;
  aggregator.bev_descriptor_max_euclidean_distance_m =
    config.bev_descriptor_max_euclidean_distance_m_;
  aggregator.bev_descriptor_threshold = config.bev_descriptor_threshold_;
  aggregator.bev_descriptor_sequence_window = config.bev_descriptor_sequence_window_;
  aggregator.bev_descriptor_sequence_threshold = config.bev_descriptor_sequence_threshold_;
  aggregator.bev_descriptor_pose_consistency_threshold_m =
    config.bev_descriptor_pose_consistency_threshold_m_;
  aggregator.bev_descriptor_rerank_weight_m = config.bev_descriptor_rerank_weight_m_;
  aggregator.solid_descriptor_max_euclidean_distance_m =
    config.solid_descriptor_max_euclidean_distance_m_;
  aggregator.solid_descriptor_min_similarity = config.solid_descriptor_min_similarity_;
  aggregator.solid_descriptor_sequence_window = config.solid_descriptor_sequence_window_;
  aggregator.solid_descriptor_sequence_min_similarity =
    config.solid_descriptor_sequence_min_similarity_;
  aggregator.solid_descriptor_pose_consistency_threshold_m =
    config.solid_descriptor_pose_consistency_threshold_m_;
  aggregator.triangle_descriptor_exclude_recent = config.triangle_descriptor_exclude_recent_;
  aggregator.triangle_descriptor_edge_bin_m = config.triangle_descriptor_edge_bin_m_;
  aggregator.triangle_descriptor_quad_feature_bin_m =
    config.triangle_descriptor_quad_feature_bin_m_;
  aggregator.triangle_descriptor_inlier_translation_m =
    config.triangle_descriptor_inlier_translation_m_;
  aggregator.triangle_descriptor_inlier_rotation_deg =
    config.triangle_descriptor_inlier_rotation_deg_;
  aggregator.triangle_descriptor_min_inliers = config.triangle_descriptor_min_inliers_;
  aggregator.triangle_descriptor_min_inlier_ratio = config.triangle_descriptor_min_inlier_ratio_;
  aggregator.triangle_descriptor_max_pairs = config.triangle_descriptor_max_pairs_;
  aggregator.triangle_descriptor_min_4th_point_agreements =
    config.triangle_descriptor_min_4th_point_agreements_;
  aggregator.triangle_descriptor_fourth_point_max_distance_m =
    config.triangle_descriptor_fourth_point_max_distance_m_;
  aggregator.triangle_descriptor_refine_se3_with_all_inliers =
    config.triangle_descriptor_refine_se3_with_all_inliers_;
  aggregator.triangle_descriptor_min_votes = config.triangle_descriptor_min_votes_;
  aggregator.triangle_descriptor_skip_ransac = config.triangle_descriptor_skip_ransac_;
  aggregator.triangle_verify_with_bev = config.triangle_verify_with_bev_;
  aggregator.triangle_verify_bev_max_distance = config.triangle_verify_bev_max_distance_;

  loop_verifier::GateConfig & gates = result.gates;
  gates.generic_score_threshold = config.threshold_loop_closure_score_;
  gates.scan_context_score_threshold = config.scan_context_loop_closure_score_threshold_;
  gates.max_translation_m = config.loop_max_translation_delta_;
  gates.max_rotation_deg = config.loop_max_rotation_delta_deg_;
  gates.min_overlap_ratio = config.loop_min_overlap_ratio_;
  gates.min_overlap_ratio_large_correction = config.loop_min_overlap_ratio_large_correction_;
  gates.overlap_large_correction_translation_m =
    config.loop_overlap_large_correction_translation_m_;
  gates.overlap_max_distance_m = config.loop_overlap_max_distance_m_;
  gates.max_translation_descriptor_m = config.loop_max_translation_delta_descriptor_;
  gates.max_rotation_descriptor_deg = config.loop_max_rotation_delta_deg_descriptor_;
  return result;
}

GraphSlamApplicationConfig makeGraphSlamApplicationConfig(const GraphSlamConfig & config)
{
  GraphSlamApplicationConfig result;
  result.descriptors = makeDescriptorConfig(config);
  result.loop_search = makeLoopSearchConfig(config);
  result.registration_method = config.registration_method;
  result.ndt_resolution = config.ndt_resolution;
  result.ndt_num_threads = config.ndt_num_threads;
  result.voxel_leaf_size = config.voxel_leaf_size;
  result.loop_search_query_stride = config.loop_search_query_stride_;
  result.loop_edge_dedup_index_window = config.loop_edge_dedup_index_window_;
  return result;
}

PoseGraphConfigBundle makePoseGraphConfig(
  const GraphSlamConfig & config,
  double adjacent_weight,
  double adjacent_weight_trans,
  double adjacent_weight_rot)
{
  PoseGraphConfigBundle result;
  result.adjacent.num_adjacent_pose_constraints = config.num_adjacent_pose_cnstraints_;
  result.adjacent.split_trans_rot = config.adjacent_edge_info_auto_scale_split_trans_rot_;
  result.adjacent.info_weight = adjacent_weight;
  result.adjacent.info_weight_trans = adjacent_weight_trans;
  result.adjacent.info_weight_rot = adjacent_weight_rot;
  result.loop.info_weight = config.loop_edge_info_weight_;
  result.loop.robust_kernel_type = config.loop_edge_robust_kernel_type_;
  result.loop.robust_kernel_delta = config.loop_edge_robust_kernel_delta_;
  result.imu.info_roll_pitch = config.imu_rotation_info_roll_pitch_;
  result.imu.info_yaw = config.imu_rotation_info_yaw_;
  if (config.adjacent_edge_info_auto_scale_) {
    result.chi2_collection = config.adjacent_edge_info_auto_scale_split_trans_rot_ ?
      pose_graph::Chi2Collection::SPLIT : pose_graph::Chi2Collection::UNIFIED;
  }
  result.auto_scale.ema_alpha = config.adjacent_edge_info_auto_scale_ema_alpha_;
  result.auto_scale.min_scale = config.adjacent_edge_info_auto_scale_min_;
  result.auto_scale.max_scale = config.adjacent_edge_info_auto_scale_max_;
  return result;
}

DynamicObjectFilterConfig makeDynamicObjectFilterConfig(const GraphSlamConfig & config)
{
  DynamicObjectFilterConfig result;
  result.voxel_size = config.dynamic_object_filter_voxel_size_;
  result.min_observations = config.dynamic_object_filter_min_observations_;
  result.temporal_window = config.dynamic_object_filter_temporal_window_;
  result.max_range_from_sensor_m = config.dynamic_object_filter_max_range_from_sensor_m_;
  return result;
}

PlanarMapFilterConfig makePlanarMapFilterConfig(const GraphSlamConfig & config)
{
  PlanarMapFilterConfig result;
  result.voxel_size = config.planar_map_filter_voxel_size_;
  result.min_neighbors = config.planar_map_filter_min_neighbors_;
  result.max_small_eigenvalue_ratio = config.planar_map_filter_max_small_eigenvalue_ratio_;
  result.min_middle_eigenvalue_ratio = config.planar_map_filter_min_middle_eigenvalue_ratio_;
  result.min_retained_ratio = config.planar_map_filter_min_retained_ratio_;
  return result;
}

map_saver::GridConfig makeGridConfig(const GraphSlamConfig & config)
{
  map_saver::GridConfig result;
  result.grid_size_x = config.map_grid_size_x_;
  result.grid_size_y = config.map_grid_size_y_;
  return result;
}

detail::GnssWeightingConfig makeGnssWeightingConfig(const GraphSlamConfig & config)
{
  detail::GnssWeightingConfig result;
  result.base_info_weight = config.gnss_info_weight_;
  result.vertical_weight_scale = 0.1;
  result.use_covariance_weighting = config.gnss_use_covariance_weighting_;
  result.covariance_min_variance_m2 = config.gnss_covariance_min_variance_m2_;
  result.covariance_max_variance_m2 = config.gnss_covariance_max_variance_m2_;
  result.rtk_fix_max_horizontal_stddev_m = config.gnss_rtk_fix_max_horizontal_stddev_m_;
  result.rtk_fix_weight_scale = config.gnss_rtk_fix_weight_scale_;
  result.non_rtk_weight_scale = config.gnss_non_rtk_weight_scale_;
  return result;
}

}  // namespace graphslam
