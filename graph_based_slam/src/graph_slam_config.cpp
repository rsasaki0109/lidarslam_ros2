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

#include "graph_slam_config.hpp"

#include <cmath>
#include <algorithm>
#include <string>
#include <vector>

namespace graphslam
{
namespace
{

template<typename T>
void loadParameter(rclcpp::Node & node, const char * name, T & value)
{
  value = node.declare_parameter<T>(name, value);
}

}  // namespace

GraphSlamConfig loadGraphSlamConfig(rclcpp::Node & node)
{
  GraphSlamConfig config;
#define LOAD(member, name) loadParameter(node, name, config.member)
  LOAD(registration_method, "registration_method");
  LOAD(voxel_leaf_size, "voxel_leaf_size");
  LOAD(ndt_resolution, "ndt_resolution");
  LOAD(ndt_num_threads, "ndt_num_threads");
  LOAD(threshold_loop_closure_score_, "threshold_loop_closure_score");
  LOAD(scan_context_loop_closure_score_threshold_, "scan_context_loop_closure_score_threshold");
  LOAD(distance_loop_closure_, "distance_loop_closure");
  LOAD(range_of_searching_loop_closure_, "range_of_searching_loop_closure");
  LOAD(search_submap_num_, "search_submap_num");
  LOAD(loop_search_query_stride_, "loop_search_query_stride");
  LOAD(max_loop_candidate_count_, "max_loop_candidate_count");
  LOAD(loop_edge_dedup_index_window_, "loop_edge_dedup_index_window");
  LOAD(loop_max_translation_delta_, "loop_max_translation_delta");
  LOAD(loop_max_rotation_delta_deg_, "loop_max_rotation_delta_deg");
  LOAD(loop_min_overlap_ratio_, "loop_min_overlap_ratio");
  LOAD(loop_min_overlap_ratio_large_correction_, "loop_min_overlap_ratio_large_correction");
  LOAD(
    loop_overlap_large_correction_translation_m_,
    "loop_overlap_large_correction_translation_m");
  LOAD(loop_overlap_max_distance_m_, "loop_overlap_max_distance_m");
  LOAD(loop_max_translation_delta_descriptor_, "loop_max_translation_delta_descriptor");
  LOAD(loop_max_rotation_delta_deg_descriptor_, "loop_max_rotation_delta_deg_descriptor");
  LOAD(num_adjacent_pose_cnstraints_, "num_adjacent_pose_cnstraints");
  LOAD(use_save_map_in_loop_, "use_save_map_in_loop");
  LOAD(debug_flag_, "debug_flag");
  LOAD(adjacent_edge_info_weight_, "adjacent_edge_info_weight");
  LOAD(adjacent_edge_info_auto_scale_, "adjacent_edge_info_auto_scale");
  LOAD(adjacent_edge_info_auto_scale_target_nis_, "adjacent_edge_info_auto_scale_target_nis");
  LOAD(adjacent_edge_info_auto_scale_ema_alpha_, "adjacent_edge_info_auto_scale_ema_alpha");
  LOAD(adjacent_edge_info_auto_scale_min_, "adjacent_edge_info_auto_scale_min");
  LOAD(adjacent_edge_info_auto_scale_max_, "adjacent_edge_info_auto_scale_max");
  LOAD(
    adjacent_edge_info_auto_scale_split_trans_rot_,
    "adjacent_edge_info_auto_scale_split_trans_rot");
  LOAD(adjacent_edge_info_weight_trans_, "adjacent_edge_info_weight_trans");
  LOAD(adjacent_edge_info_weight_rot_, "adjacent_edge_info_weight_rot");
  LOAD(
    adjacent_edge_info_auto_scale_target_nis_trans_,
    "adjacent_edge_info_auto_scale_target_nis_trans");
  LOAD(
    adjacent_edge_info_auto_scale_target_nis_rot_,
    "adjacent_edge_info_auto_scale_target_nis_rot");
  LOAD(loop_edge_info_weight_, "loop_edge_info_weight");
  LOAD(loop_edge_robust_kernel_delta_, "loop_edge_robust_kernel_delta");
  LOAD(loop_edge_robust_kernel_type_, "loop_edge_robust_kernel_type");
  LOAD(use_scan_context_, "use_scan_context");
  LOAD(use_bev_descriptor_, "use_bev_descriptor");
  LOAD(use_solid_descriptor_, "use_solid_descriptor");
  LOAD(use_triangle_descriptor_, "use_triangle_descriptor");
  LOAD(triangle_descriptor_grid_size_m_, "triangle_descriptor_grid_size_m");
  LOAD(triangle_descriptor_grid_cells_, "triangle_descriptor_grid_cells");
  LOAD(triangle_descriptor_max_keypoints_, "triangle_descriptor_max_keypoints");
  LOAD(triangle_descriptor_min_salience_m_, "triangle_descriptor_min_salience_m");
  LOAD(triangle_descriptor_min_edge_m_, "triangle_descriptor_min_edge_m");
  LOAD(triangle_descriptor_max_edge_m_, "triangle_descriptor_max_edge_m");
  LOAD(triangle_descriptor_max_triangles_, "triangle_descriptor_max_triangles");
  LOAD(triangle_descriptor_edge_bin_m_, "triangle_descriptor_edge_bin_m");
  LOAD(triangle_descriptor_quad_feature_bin_m_, "triangle_descriptor_quad_feature_bin_m");
  LOAD(triangle_descriptor_keypoint_mode_, "triangle_descriptor_keypoint_mode");
  LOAD(triangle_descriptor_edge_voxel_size_m_, "triangle_descriptor_edge_voxel_size_m");
  LOAD(
    triangle_descriptor_edge_neighbor_radius_m_,
    "triangle_descriptor_edge_neighbor_radius_m");
  LOAD(triangle_descriptor_edge_min_neighbors_, "triangle_descriptor_edge_min_neighbors");
  LOAD(triangle_descriptor_edge_min_edgeness_, "triangle_descriptor_edge_min_edgeness");
  LOAD(triangle_descriptor_edge_nms_radius_m_, "triangle_descriptor_edge_nms_radius_m");
  LOAD(triangle_descriptor_min_votes_, "triangle_descriptor_min_votes");
  LOAD(triangle_descriptor_min_inliers_, "triangle_descriptor_min_inliers");
  LOAD(triangle_descriptor_min_inlier_ratio_, "triangle_descriptor_min_inlier_ratio");
  LOAD(triangle_descriptor_max_pairs_, "triangle_descriptor_max_pairs");
  LOAD(
    triangle_descriptor_min_4th_point_agreements_,
    "triangle_descriptor_min_4th_point_agreements");
  LOAD(
    triangle_descriptor_fourth_point_max_distance_m_,
    "triangle_descriptor_fourth_point_max_distance_m");
  LOAD(
    triangle_descriptor_refine_se3_with_all_inliers_,
    "triangle_descriptor_refine_se3_with_all_inliers");
  LOAD(triangle_descriptor_skip_ransac_, "triangle_descriptor_skip_ransac");
  LOAD(
    triangle_descriptor_inlier_translation_m_,
    "triangle_descriptor_inlier_translation_m");
  LOAD(triangle_descriptor_inlier_rotation_deg_, "triangle_descriptor_inlier_rotation_deg");
  LOAD(triangle_descriptor_exclude_recent_, "triangle_descriptor_exclude_recent");
  LOAD(triangle_verify_with_bev_, "triangle_verify_with_bev");
  LOAD(triangle_verify_bev_max_distance_, "triangle_verify_bev_max_distance");
  LOAD(use_pcd_cache_, "use_pcd_cache");
  LOAD(pcd_cache_dir_, "pcd_cache_dir");
  LOAD(scan_context_threshold_, "scan_context_threshold");
  LOAD(scan_context_query_stride_, "scan_context_query_stride");
  LOAD(scan_context_exclude_recent_, "scan_context_exclude_recent");
  LOAD(bev_descriptor_threshold_, "bev_descriptor_threshold");
  LOAD(bev_descriptor_grid_size_m_, "bev_descriptor_grid_size_m");
  LOAD(bev_descriptor_grid_cells_, "bev_descriptor_grid_cells");
  LOAD(bev_descriptor_yaw_bins_, "bev_descriptor_yaw_bins");
  LOAD(bev_descriptor_sequence_window_, "bev_descriptor_sequence_window");
  LOAD(bev_descriptor_sequence_threshold_, "bev_descriptor_sequence_threshold");
  LOAD(
    bev_descriptor_pose_consistency_threshold_m_,
    "bev_descriptor_pose_consistency_threshold_m");
  LOAD(bev_descriptor_max_euclidean_distance_m_, "bev_descriptor_max_euclidean_distance_m");
  LOAD(bev_descriptor_rerank_weight_m_, "bev_descriptor_rerank_weight_m");
  LOAD(bev_use_mutual_visibility_, "bev_use_mutual_visibility");
  LOAD(
    bev_mutual_visibility_min_overlap_ratio_,
    "bev_mutual_visibility_min_overlap_ratio");
  LOAD(bev_mutual_visibility_occupancy_eps_, "bev_mutual_visibility_occupancy_eps");
  LOAD(solid_descriptor_min_similarity_, "solid_descriptor_min_similarity");
  LOAD(solid_descriptor_sequence_window_, "solid_descriptor_sequence_window");
  LOAD(solid_descriptor_sequence_min_similarity_, "solid_descriptor_sequence_min_similarity");
  LOAD(
    solid_descriptor_pose_consistency_threshold_m_,
    "solid_descriptor_pose_consistency_threshold_m");
  LOAD(solid_descriptor_max_euclidean_distance_m_, "solid_descriptor_max_euclidean_distance_m");
  LOAD(prefer_scan_context_candidates_, "prefer_scan_context_candidates");
  LOAD(use_3d_bbs_for_scan_context_, "use_3d_bbs_for_scan_context");
  LOAD(three_d_bbs_min_level_res_, "three_d_bbs_min_level_res");
  LOAD(three_d_bbs_max_level_, "three_d_bbs_max_level");
  LOAD(three_d_bbs_score_threshold_percentage_, "three_d_bbs_score_threshold_percentage");
  LOAD(three_d_bbs_timeout_msec_, "three_d_bbs_timeout_msec");
  LOAD(three_d_bbs_num_threads_, "three_d_bbs_num_threads");
  LOAD(three_d_bbs_voxel_leaf_size_, "three_d_bbs_voxel_leaf_size");
  LOAD(three_d_bbs_source_submap_num_, "three_d_bbs_source_submap_num");
  LOAD(three_d_bbs_target_submap_radius_, "three_d_bbs_target_submap_radius");
  LOAD(
    three_d_bbs_translation_search_margin_m_,
    "three_d_bbs_translation_search_margin_m");
  LOAD(three_d_bbs_roll_pitch_search_deg_, "three_d_bbs_roll_pitch_search_deg");
  LOAD(three_d_bbs_yaw_search_deg_, "three_d_bbs_yaw_search_deg");
  LOAD(use_dynamic_object_filter_, "use_dynamic_object_filter");
  LOAD(dynamic_object_filter_voxel_size_, "dynamic_object_filter_voxel_size");
  LOAD(dynamic_object_filter_min_observations_, "dynamic_object_filter_min_observations");
  LOAD(dynamic_object_filter_temporal_window_, "dynamic_object_filter_temporal_window");
  LOAD(
    dynamic_object_filter_max_range_from_sensor_m_,
    "dynamic_object_filter_max_range_from_sensor_m");
  LOAD(use_planar_map_filter_, "use_planar_map_filter");
  LOAD(planar_map_filter_voxel_size_, "planar_map_filter_voxel_size");
  LOAD(planar_map_filter_min_neighbors_, "planar_map_filter_min_neighbors");
  LOAD(
    planar_map_filter_max_small_eigenvalue_ratio_,
    "planar_map_filter_max_small_eigenvalue_ratio");
  LOAD(
    planar_map_filter_min_middle_eigenvalue_ratio_,
    "planar_map_filter_min_middle_eigenvalue_ratio");
  LOAD(planar_map_filter_min_retained_ratio_, "planar_map_filter_min_retained_ratio");
  LOAD(map_save_dir_, "map_save_dir");
  LOAD(save_pose_graph_path_, "save_pose_graph_path");
  LOAD(map_grid_size_x_, "map_grid_size_x");
  LOAD(map_grid_size_y_, "map_grid_size_y");
  LOAD(map_leaf_size_, "map_leaf_size");
  LOAD(use_gnss_, "use_gnss");
  LOAD(gnss_topic_, "gnss_topic");
  LOAD(gnss_info_weight_, "gnss_info_weight");
  LOAD(gnss_use_covariance_weighting_, "gnss_use_covariance_weighting");
  LOAD(gnss_covariance_min_variance_m2_, "gnss_covariance_min_variance_m2");
  LOAD(gnss_covariance_max_variance_m2_, "gnss_covariance_max_variance_m2");
  LOAD(
    gnss_rtk_fix_max_horizontal_stddev_m_,
    "gnss_rtk_fix_max_horizontal_stddev_m");
  LOAD(gnss_rtk_fix_weight_scale_, "gnss_rtk_fix_weight_scale");
  LOAD(gnss_non_rtk_weight_scale_, "gnss_non_rtk_weight_scale");
  LOAD(gnss_header_stamp_max_skew_sec_, "gnss_header_stamp_max_skew_sec");
  LOAD(gnss_align_yaw_, "gnss_align_yaw");
  LOAD(gnss_yaw_alignment_min_anchors_, "gnss_yaw_alignment_min_anchors");
  LOAD(gnss_yaw_alignment_min_baseline_m_, "gnss_yaw_alignment_min_baseline_m");
  LOAD(gnss_origin_min_samples_, "gnss_origin_min_samples");
  LOAD(gnss_origin_consistency_threshold_m_, "gnss_origin_consistency_threshold_m");
  LOAD(use_imu_preintegration_, "use_imu_preintegration");
  LOAD(imu_rotation_info_roll_pitch_, "imu_rotation_info_roll_pitch");
  LOAD(imu_rotation_info_yaw_, "imu_rotation_info_yaw");
  LOAD(use_odom_input_, "use_odom_input");
  LOAD(submap_distance_threshold_, "submap_distance_threshold");
  LOAD(odom_cloud_sync_queue_size_, "odom_cloud_sync_queue_size");
  LOAD(odom_cloud_sync_use_exact_time_, "odom_cloud_sync_use_exact_time");
  LOAD(cloud_subscriber_qos_reliable_, "cloud_subscriber_qos_reliable");
  LOAD(degeneracy_diagnostics_csv_path_, "degeneracy_diagnostics_csv_path");
  LOAD(save_degeneracy_report_, "save_degeneracy_report");
#undef LOAD
  return config;
}

ConfigNormalization normalizeGraphSlamConfig(GraphSlamConfig & config)
{
  ConfigNormalization result;
  const auto warn = [&result](const char * name, const char * action) {
      result.warnings.emplace_back(std::string(name) + " " + action);
    };
  const auto atLeastOne = [&warn](int & value, const char * name) {
      if (value < 1) {
        warn(name, "must be >= 1; clamped to 1");
        value = 1;
      }
    };
  const auto positiveOr = [&warn](double & value, double fallback, const char * name) {
      if (value <= 0.0) {
        warn(name, "must be positive; reset to its safe default");
        value = fallback;
      }
    };
  const auto unitInterval = [&warn](double & value, double fallback, const char * name) {
      if (value < 0.0 || value > 1.0) {
        warn(name, "must be in [0, 1]; reset to its safe default");
        value = fallback;
      }
    };

  positiveOr(config.adjacent_edge_info_weight_, 1000.0, "adjacent_edge_info_weight");
  if (config.adjacent_edge_info_weight_trans_ <= 0.0) {
    config.adjacent_edge_info_weight_trans_ = config.adjacent_edge_info_weight_;
  }
  if (config.adjacent_edge_info_weight_rot_ <= 0.0) {
    config.adjacent_edge_info_weight_rot_ = config.adjacent_edge_info_weight_;
  }
  positiveOr(
    config.adjacent_edge_info_auto_scale_target_nis_trans_, 3.0,
    "adjacent_edge_info_auto_scale_target_nis_trans");
  positiveOr(
    config.adjacent_edge_info_auto_scale_target_nis_rot_, 3.0,
    "adjacent_edge_info_auto_scale_target_nis_rot");

  atLeastOne(config.gnss_origin_min_samples_, "gnss_origin_min_samples");
  positiveOr(
    config.gnss_origin_consistency_threshold_m_, 20.0,
    "gnss_origin_consistency_threshold_m");
  positiveOr(
    config.gnss_covariance_min_variance_m2_, 0.01,
    "gnss_covariance_min_variance_m2");
  if (config.gnss_covariance_max_variance_m2_ < config.gnss_covariance_min_variance_m2_) {
    warn("gnss_covariance_max_variance_m2", "must be >= minimum; raised to minimum");
    config.gnss_covariance_max_variance_m2_ = config.gnss_covariance_min_variance_m2_;
  }
  positiveOr(
    config.gnss_rtk_fix_max_horizontal_stddev_m_, 0.3,
    "gnss_rtk_fix_max_horizontal_stddev_m");
  positiveOr(config.gnss_rtk_fix_weight_scale_, 3.0, "gnss_rtk_fix_weight_scale");
  positiveOr(config.gnss_non_rtk_weight_scale_, 1.0, "gnss_non_rtk_weight_scale");
  positiveOr(config.gnss_header_stamp_max_skew_sec_, 30.0, "gnss_header_stamp_max_skew_sec");

  atLeastOne(config.search_submap_num_, "search_submap_num");
  atLeastOne(config.loop_search_query_stride_, "loop_search_query_stride");
  atLeastOne(config.max_loop_candidate_count_, "max_loop_candidate_count");
  if (config.loop_edge_dedup_index_window_ < 0) {
    warn("loop_edge_dedup_index_window", "must be >= 0; clamped to 0");
    config.loop_edge_dedup_index_window_ = 0;
  }
  positiveOr(config.loop_max_translation_delta_, 15.0, "loop_max_translation_delta");
  positiveOr(config.loop_max_rotation_delta_deg_, 45.0, "loop_max_rotation_delta_deg");
  unitInterval(config.loop_min_overlap_ratio_, 0.0, "loop_min_overlap_ratio");
  unitInterval(
    config.loop_min_overlap_ratio_large_correction_, 0.0,
    "loop_min_overlap_ratio_large_correction");
  if (config.loop_overlap_large_correction_translation_m_ < 0.0) {
    warn(
      "loop_overlap_large_correction_translation_m",
      "must be non-negative; reset to disabled");
    config.loop_overlap_large_correction_translation_m_ = 0.0;
  }
  positiveOr(config.loop_overlap_max_distance_m_, 0.5, "loop_overlap_max_distance_m");
  if (config.loop_max_translation_delta_descriptor_ <= 0.0 &&
    config.loop_max_translation_delta_descriptor_ != -1.0)
  {
    warn("loop_max_translation_delta_descriptor", "must be positive or -1; reset to -1");
    config.loop_max_translation_delta_descriptor_ = -1.0;
  }
  if (config.loop_max_rotation_delta_deg_descriptor_ <= 0.0 &&
    config.loop_max_rotation_delta_deg_descriptor_ != -1.0)
  {
    warn("loop_max_rotation_delta_deg_descriptor", "must be positive or -1; reset to -1");
    config.loop_max_rotation_delta_deg_descriptor_ = -1.0;
  }
  atLeastOne(config.num_adjacent_pose_cnstraints_, "num_adjacent_pose_cnstraints");
  positiveOr(config.loop_edge_info_weight_, 100.0, "loop_edge_info_weight");
  positiveOr(config.loop_edge_robust_kernel_delta_, 1.0, "loop_edge_robust_kernel_delta");

  positiveOr(config.bev_descriptor_threshold_, 0.20, "bev_descriptor_threshold");
  positiveOr(config.bev_descriptor_grid_size_m_, 80.0, "bev_descriptor_grid_size_m");
  if (config.bev_descriptor_grid_cells_ < 8) {
    warn("bev_descriptor_grid_cells", "must be >= 8; clamped to 8");
    config.bev_descriptor_grid_cells_ = 8;
  }
  atLeastOne(config.bev_descriptor_yaw_bins_, "bev_descriptor_yaw_bins");
  if (config.bev_descriptor_sequence_window_ < 0) {
    warn("bev_descriptor_sequence_window", "must be >= 0; clamped to 0");
    config.bev_descriptor_sequence_window_ = 0;
  }
  if (config.bev_descriptor_sequence_threshold_ <= 0.0) {
    config.bev_descriptor_sequence_threshold_ = config.bev_descriptor_threshold_;
  }
  if (config.bev_descriptor_rerank_weight_m_ < 0.0) {
    warn("bev_descriptor_rerank_weight_m", "must be >= 0; clamped to 0");
    config.bev_descriptor_rerank_weight_m_ = 0.0;
  }
  if (config.bev_descriptor_pose_consistency_threshold_m_ == 0.0) {
    warn("bev_descriptor_pose_consistency_threshold_m", "cannot be zero; reset to disabled");
    config.bev_descriptor_pose_consistency_threshold_m_ = -1.0;
  }

  if (config.triangle_descriptor_keypoint_mode_ != "bev_max_height" &&
    config.triangle_descriptor_keypoint_mode_ != "edge_3d")
  {
    warn("triangle_descriptor_keypoint_mode", "is unknown; reset to bev_max_height");
    config.triangle_descriptor_keypoint_mode_ = "bev_max_height";
  }
  config.triangle_descriptor_edge_voxel_size_m_ =
    std::max(0.0, config.triangle_descriptor_edge_voxel_size_m_);
  config.triangle_descriptor_edge_neighbor_radius_m_ =
    std::max(0.05, config.triangle_descriptor_edge_neighbor_radius_m_);
  config.triangle_descriptor_edge_min_neighbors_ =
    std::max(4, config.triangle_descriptor_edge_min_neighbors_);
  config.triangle_descriptor_edge_min_edgeness_ =
    std::max(0.0, std::min(1.0, config.triangle_descriptor_edge_min_edgeness_));
  config.triangle_descriptor_edge_nms_radius_m_ =
    std::max(0.0, config.triangle_descriptor_edge_nms_radius_m_);
  positiveOr(config.triangle_descriptor_grid_size_m_, 60.0, "triangle_descriptor_grid_size_m");
  if (config.triangle_descriptor_grid_cells_ < 8) {
    warn("triangle_descriptor_grid_cells", "must be >= 8; clamped to 8");
    config.triangle_descriptor_grid_cells_ = 8;
  }
  if (config.triangle_descriptor_max_keypoints_ < 4) {
    warn("triangle_descriptor_max_keypoints", "must be >= 4; clamped to 4");
    config.triangle_descriptor_max_keypoints_ = 4;
  }
  positiveOr(config.triangle_descriptor_min_edge_m_, 2.0, "triangle_descriptor_min_edge_m");
  if (config.triangle_descriptor_max_edge_m_ <= config.triangle_descriptor_min_edge_m_) {
    warn("triangle_descriptor_max_edge_m", "must exceed minimum; reset to minimum x 5");
    config.triangle_descriptor_max_edge_m_ = config.triangle_descriptor_min_edge_m_ * 5.0;
  }
  config.triangle_descriptor_max_triangles_ =
    std::max(100, config.triangle_descriptor_max_triangles_);
  positiveOr(config.triangle_descriptor_edge_bin_m_, 1.0, "triangle_descriptor_edge_bin_m");
  if (config.triangle_descriptor_quad_feature_bin_m_ < 0.0) {
    warn("triangle_descriptor_quad_feature_bin_m", "must be >= 0; reset to disabled");
    config.triangle_descriptor_quad_feature_bin_m_ = 0.0;
  }
  config.triangle_descriptor_min_votes_ = std::max(1, config.triangle_descriptor_min_votes_);
  config.triangle_descriptor_min_inliers_ = std::max(1, config.triangle_descriptor_min_inliers_);
  config.triangle_descriptor_min_inlier_ratio_ =
    std::max(0.0, std::min(1.0, config.triangle_descriptor_min_inlier_ratio_));
  if (config.triangle_descriptor_max_pairs_ < 3) {
    warn("triangle_descriptor_max_pairs", "must be >= 3; clamped to 3");
    config.triangle_descriptor_max_pairs_ = 3;
  }
  config.triangle_descriptor_min_4th_point_agreements_ =
    std::max(0, config.triangle_descriptor_min_4th_point_agreements_);
  positiveOr(
    config.triangle_descriptor_fourth_point_max_distance_m_, 2.0,
    "triangle_descriptor_fourth_point_max_distance_m");
  positiveOr(
    config.triangle_descriptor_inlier_translation_m_, 2.0,
    "triangle_descriptor_inlier_translation_m");
  positiveOr(
    config.triangle_descriptor_inlier_rotation_deg_, 5.0,
    "triangle_descriptor_inlier_rotation_deg");
  config.triangle_descriptor_exclude_recent_ =
    std::max(0, config.triangle_descriptor_exclude_recent_);
  positiveOr(config.triangle_verify_bev_max_distance_, 0.30, "triangle_verify_bev_max_distance");

  if (config.solid_descriptor_min_similarity_ <= -1.0 ||
    config.solid_descriptor_min_similarity_ > 1.0)
  {
    warn("solid_descriptor_min_similarity", "must be in (-1, 1]; reset to 0.70");
    config.solid_descriptor_min_similarity_ = 0.70;
  }
  if (config.solid_descriptor_sequence_window_ < 0) {
    warn("solid_descriptor_sequence_window", "must be >= 0; clamped to 0");
    config.solid_descriptor_sequence_window_ = 0;
  }
  if (config.solid_descriptor_sequence_min_similarity_ <= -1.0 ||
    config.solid_descriptor_sequence_min_similarity_ > 1.0)
  {
    config.solid_descriptor_sequence_min_similarity_ = config.solid_descriptor_min_similarity_;
  }
  if (config.solid_descriptor_pose_consistency_threshold_m_ == 0.0) {
    warn("solid_descriptor_pose_consistency_threshold_m", "cannot be zero; reset to disabled");
    config.solid_descriptor_pose_consistency_threshold_m_ = -1.0;
  }

  positiveOr(config.three_d_bbs_min_level_res_, 1.0, "three_d_bbs_min_level_res");
  atLeastOne(config.three_d_bbs_max_level_, "three_d_bbs_max_level");
  positiveOr(
    config.three_d_bbs_score_threshold_percentage_, 0.25,
    "three_d_bbs_score_threshold_percentage");
  positiveOr(config.three_d_bbs_voxel_leaf_size_, 1.0, "three_d_bbs_voxel_leaf_size");
  atLeastOne(config.three_d_bbs_source_submap_num_, "three_d_bbs_source_submap_num");
  if (config.three_d_bbs_target_submap_radius_ < 0) {
    warn("three_d_bbs_target_submap_radius", "must be >= 0; clamped to 0");
    config.three_d_bbs_target_submap_radius_ = 0;
  }
  positiveOr(
    config.three_d_bbs_translation_search_margin_m_, 15.0,
    "three_d_bbs_translation_search_margin_m");
  positiveOr(
    config.three_d_bbs_roll_pitch_search_deg_, 10.0,
    "three_d_bbs_roll_pitch_search_deg");
  positiveOr(config.three_d_bbs_yaw_search_deg_, 180.0, "three_d_bbs_yaw_search_deg");

  positiveOr(
    config.dynamic_object_filter_voxel_size_, 0.3,
    "dynamic_object_filter_voxel_size");
  atLeastOne(
    config.dynamic_object_filter_min_observations_,
    "dynamic_object_filter_min_observations");
  if (config.dynamic_object_filter_temporal_window_ < 0) {
    warn("dynamic_object_filter_temporal_window", "must be >= 0; clamped to 0");
    config.dynamic_object_filter_temporal_window_ = 0;
  }
  positiveOr(
    config.dynamic_object_filter_max_range_from_sensor_m_, 30.0,
    "dynamic_object_filter_max_range_from_sensor_m");
  positiveOr(config.planar_map_filter_voxel_size_, 0.1, "planar_map_filter_voxel_size");
  if (config.planar_map_filter_min_neighbors_ < 3) {
    warn("planar_map_filter_min_neighbors", "must be >= 3; clamped to 3");
    config.planar_map_filter_min_neighbors_ = 3;
  }
  config.planar_map_filter_max_small_eigenvalue_ratio_ =
    std::max(0.0, std::min(1.0, config.planar_map_filter_max_small_eigenvalue_ratio_));
  config.planar_map_filter_min_middle_eigenvalue_ratio_ =
    std::max(0.0, std::min(1.0, config.planar_map_filter_min_middle_eigenvalue_ratio_));
  config.planar_map_filter_min_retained_ratio_ =
    std::max(0.0, std::min(1.0, config.planar_map_filter_min_retained_ratio_));
  return result;
}

std::vector<std::string> validateGraphSlamConfig(const GraphSlamConfig & config)
{
  std::vector<std::string> errors;
  const auto positive = [&errors](double value, const char * name) {
      if (!std::isfinite(value) || value <= 0.0) {
        errors.emplace_back(std::string(name) + " must be finite and > 0");
      }
    };

  positive(config.voxel_leaf_size, "voxel_leaf_size");
  positive(config.ndt_resolution, "ndt_resolution");
  positive(config.map_grid_size_x_, "map_grid_size_x");
  positive(config.map_grid_size_y_, "map_grid_size_y");
  positive(config.map_leaf_size_, "map_leaf_size");
  if (config.odom_cloud_sync_queue_size_ < 1) {
    errors.emplace_back("odom_cloud_sync_queue_size must be >= 1");
  }
  if (config.use_pcd_cache_ && config.pcd_cache_dir_.empty()) {
    errors.emplace_back("pcd_cache_dir must not be empty when use_pcd_cache is true");
  }
  return errors;
}

void logGraphSlamConfig(const GraphSlamConfig & config, const rclcpp::Logger & logger)
{
  RCLCPP_INFO(
    logger,
    "graph SLAM config: registration=%s voxel=%.3f ndt_resolution=%.3f ndt_threads=%d",
    config.registration_method.c_str(), config.voxel_leaf_size, config.ndt_resolution,
    config.ndt_num_threads);
  RCLCPP_INFO(
    logger,
    "graph SLAM config: loop(search_submaps=%d stride=%d candidates=%d dedup_window=%d) "
    "descriptors(scan_context=%s bev=%s solid=%s triangle=%s)",
    config.search_submap_num_, config.loop_search_query_stride_, config.max_loop_candidate_count_,
    config.loop_edge_dedup_index_window_, config.use_scan_context_ ? "on" : "off",
    config.use_bev_descriptor_ ? "on" : "off", config.use_solid_descriptor_ ? "on" : "off",
    config.use_triangle_descriptor_ ? "on" : "off");
  RCLCPP_INFO(
    logger,
    "graph SLAM config: inputs(odom=%s gnss=%s imu=%s) storage(pcd_cache=%s map_dir=%s)",
    config.use_odom_input_ ? "on" : "off", config.use_gnss_ ? "on" : "off",
    config.use_imu_preintegration_ ? "on" : "off", config.use_pcd_cache_ ? "on" : "off",
    config.map_save_dir_.c_str());
  RCLCPP_INFO(
    logger,
    "graph SLAM config: odom_cloud_sync(mode=%s queue=%d cloud_qos=%s)",
    config.odom_cloud_sync_use_exact_time_ ? "exact" : "approximate",
    config.odom_cloud_sync_queue_size_,
    config.cloud_subscriber_qos_reliable_ ? "reliable" : "best_effort");
}

}  // namespace graphslam
