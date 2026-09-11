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

#ifndef GRAPH_SLAM_CONFIG_HPP_
#define GRAPH_SLAM_CONFIG_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace graphslam
{

// Complete, typed snapshot of the ROS parameter surface. This source-private
// type keeps parameter declaration/loading out of the component lifecycle and
// separates the normalized startup snapshot from live graph and sensor state.
struct GraphSlamConfig
{
  // Registration and loop search.
  std::string registration_method {"NDT"};
  double voxel_leaf_size {0.2};
  double ndt_resolution {5.0};
  int ndt_num_threads {0};
  double threshold_loop_closure_score_ {1.0};
  double scan_context_loop_closure_score_threshold_ {-1.0};
  double distance_loop_closure_ {20.0};
  double range_of_searching_loop_closure_ {20.0};
  int search_submap_num_ {3};
  int loop_search_query_stride_ {1};
  int max_loop_candidate_count_ {3};
  int loop_edge_dedup_index_window_ {8};
  double loop_max_translation_delta_ {15.0};
  double loop_max_rotation_delta_deg_ {45.0};
  double loop_min_overlap_ratio_ {0.0};
  double loop_min_overlap_ratio_large_correction_ {0.0};
  double loop_overlap_large_correction_translation_m_ {0.0};
  double loop_overlap_max_distance_m_ {0.5};
  double loop_max_translation_delta_descriptor_ {-1.0};
  double loop_max_rotation_delta_deg_descriptor_ {-1.0};

  // Pose graph optimization.
  int num_adjacent_pose_cnstraints_ {5};
  bool use_save_map_in_loop_ {true};
  bool debug_flag_ {false};
  double adjacent_edge_info_weight_ {1000.0};
  bool adjacent_edge_info_auto_scale_ {false};
  double adjacent_edge_info_auto_scale_target_nis_ {6.0};
  double adjacent_edge_info_auto_scale_ema_alpha_ {0.3};
  double adjacent_edge_info_auto_scale_min_ {1.0};
  double adjacent_edge_info_auto_scale_max_ {1.0e6};
  bool adjacent_edge_info_auto_scale_split_trans_rot_ {false};
  double adjacent_edge_info_weight_trans_ {-1.0};
  double adjacent_edge_info_weight_rot_ {-1.0};
  double adjacent_edge_info_auto_scale_target_nis_trans_ {3.0};
  double adjacent_edge_info_auto_scale_target_nis_rot_ {3.0};
  double loop_edge_info_weight_ {100.0};
  double loop_edge_robust_kernel_delta_ {1.0};
  std::string loop_edge_robust_kernel_type_ {"huber"};

  // Place recognition and verification.
  bool use_scan_context_ {false};
  bool use_bev_descriptor_ {false};
  bool use_solid_descriptor_ {false};
  bool use_triangle_descriptor_ {false};
  double triangle_descriptor_grid_size_m_ {60.0};
  int triangle_descriptor_grid_cells_ {100};
  int triangle_descriptor_max_keypoints_ {40};
  double triangle_descriptor_min_salience_m_ {0.8};
  double triangle_descriptor_min_edge_m_ {2.0};
  double triangle_descriptor_max_edge_m_ {50.0};
  int triangle_descriptor_max_triangles_ {3000};
  double triangle_descriptor_edge_bin_m_ {0.5};
  double triangle_descriptor_quad_feature_bin_m_ {0.0};
  std::string triangle_descriptor_keypoint_mode_ {"bev_max_height"};
  double triangle_descriptor_edge_voxel_size_m_ {0.4};
  double triangle_descriptor_edge_neighbor_radius_m_ {1.0};
  int triangle_descriptor_edge_min_neighbors_ {6};
  double triangle_descriptor_edge_min_edgeness_ {0.5};
  double triangle_descriptor_edge_nms_radius_m_ {2.0};
  int triangle_descriptor_min_votes_ {6};
  int triangle_descriptor_min_inliers_ {4};
  double triangle_descriptor_min_inlier_ratio_ {0.0};
  int triangle_descriptor_max_pairs_ {64};
  int triangle_descriptor_min_4th_point_agreements_ {0};
  double triangle_descriptor_fourth_point_max_distance_m_ {2.0};
  bool triangle_descriptor_refine_se3_with_all_inliers_ {false};
  bool triangle_descriptor_skip_ransac_ {false};
  double triangle_descriptor_inlier_translation_m_ {2.0};
  double triangle_descriptor_inlier_rotation_deg_ {5.0};
  int triangle_descriptor_exclude_recent_ {4};
  bool triangle_verify_with_bev_ {false};
  double triangle_verify_bev_max_distance_ {0.30};
  double scan_context_threshold_ {0.3};
  int scan_context_query_stride_ {1};
  int scan_context_exclude_recent_ {50};
  double bev_descriptor_threshold_ {0.20};
  double bev_descriptor_grid_size_m_ {80.0};
  int bev_descriptor_grid_cells_ {40};
  int bev_descriptor_yaw_bins_ {24};
  int bev_descriptor_sequence_window_ {0};
  double bev_descriptor_sequence_threshold_ {-1.0};
  double bev_descriptor_pose_consistency_threshold_m_ {-1.0};
  double bev_descriptor_max_euclidean_distance_m_ {-1.0};
  double bev_descriptor_rerank_weight_m_ {100.0};
  bool bev_use_mutual_visibility_ {false};
  double bev_mutual_visibility_min_overlap_ratio_ {0.05};
  double bev_mutual_visibility_occupancy_eps_ {0.5};
  double solid_descriptor_min_similarity_ {0.70};
  int solid_descriptor_sequence_window_ {0};
  double solid_descriptor_sequence_min_similarity_ {-1.0};
  double solid_descriptor_pose_consistency_threshold_m_ {-1.0};
  double solid_descriptor_max_euclidean_distance_m_ {-1.0};
  bool prefer_scan_context_candidates_ {false};
  bool use_3d_bbs_for_scan_context_ {false};
  double three_d_bbs_min_level_res_ {1.0};
  int three_d_bbs_max_level_ {3};
  double three_d_bbs_score_threshold_percentage_ {0.25};
  int three_d_bbs_timeout_msec_ {50};
  int three_d_bbs_num_threads_ {0};
  double three_d_bbs_voxel_leaf_size_ {1.0};
  int three_d_bbs_source_submap_num_ {2};
  int three_d_bbs_target_submap_radius_ {1};
  double three_d_bbs_translation_search_margin_m_ {15.0};
  double three_d_bbs_roll_pitch_search_deg_ {10.0};
  double three_d_bbs_yaw_search_deg_ {180.0};

  // Map filtering, storage and export.
  bool use_dynamic_object_filter_ {false};
  double dynamic_object_filter_voxel_size_ {0.3};
  int dynamic_object_filter_min_observations_ {2};
  int dynamic_object_filter_temporal_window_ {5};
  double dynamic_object_filter_max_range_from_sensor_m_ {30.0};
  bool use_planar_map_filter_ {false};
  double planar_map_filter_voxel_size_ {0.1};
  int planar_map_filter_min_neighbors_ {3};
  double planar_map_filter_max_small_eigenvalue_ratio_ {0.24};
  double planar_map_filter_min_middle_eigenvalue_ratio_ {0.0};
  double planar_map_filter_min_retained_ratio_ {0.90};
  bool use_pcd_cache_ {false};
  std::string pcd_cache_dir_ {"/tmp/graph_slam_pcd_cache"};
  std::string map_save_dir_ {"."};
  std::string save_pose_graph_path_ {"pose_graph.g2o"};
  double map_grid_size_x_ {20.0};
  double map_grid_size_y_ {20.0};
  double map_leaf_size_ {0.2};

  // Optional absolute and inertial constraints.
  bool use_gnss_ {false};
  std::string gnss_topic_ {"/gnss/fix"};
  double gnss_info_weight_ {1.0};
  bool gnss_use_covariance_weighting_ {true};
  double gnss_covariance_min_variance_m2_ {0.01};
  double gnss_covariance_max_variance_m2_ {25.0};
  double gnss_rtk_fix_max_horizontal_stddev_m_ {0.3};
  double gnss_rtk_fix_weight_scale_ {3.0};
  double gnss_non_rtk_weight_scale_ {1.0};
  double gnss_header_stamp_max_skew_sec_ {30.0};
  bool gnss_align_yaw_ {true};
  int gnss_yaw_alignment_min_anchors_ {10};
  double gnss_yaw_alignment_min_baseline_m_ {5.0};
  int gnss_origin_min_samples_ {3};
  double gnss_origin_consistency_threshold_m_ {20.0};
  bool use_imu_preintegration_ {false};
  double imu_rotation_info_roll_pitch_ {100.0};
  double imu_rotation_info_yaw_ {10.0};

  // Direct odometry input and diagnostics.
  bool use_odom_input_ {false};
  double submap_distance_threshold_ {1.5};
  int odom_cloud_sync_queue_size_ {100};
  bool odom_cloud_sync_use_exact_time_ {false};
  bool cloud_subscriber_qos_reliable_ {true};
  std::string degeneracy_diagnostics_csv_path_;
  bool save_degeneracy_report_ {false};
};

struct ConfigNormalization
{
  std::vector<std::string> warnings;
};

GraphSlamConfig loadGraphSlamConfig(rclcpp::Node & node);
ConfigNormalization normalizeGraphSlamConfig(GraphSlamConfig & config);
std::vector<std::string> validateGraphSlamConfig(const GraphSlamConfig & config);
void logGraphSlamConfig(const GraphSlamConfig & config, const rclcpp::Logger & logger);

}  // namespace graphslam

#endif  // GRAPH_SLAM_CONFIG_HPP_
