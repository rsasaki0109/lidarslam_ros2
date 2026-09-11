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
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
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

#ifndef GRAPH_BASED_SLAM__BACKEND_CORE_HPP_
#define GRAPH_BASED_SLAM__BACKEND_CORE_HPP_

// The ROS-free, clock-free backend state (docs/roadmap/v0.6.md, Phase 2).
// This engine owns the four loop-closure descriptor databases and their
// submap-ingestion/search algorithms. GraphSlamApplication is the aggregate
// lifetime boundary: its private engine owns this state together with the
// registration, filters, verifier, scheduling and accepted graph. The contract:
// the same ordered submap sequence plus the same config produce the same
// state, independent of wall-clock timing. The shell supplies clouds via
// a provider callback, so message-vs-PCD-cache stays its concern.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <functional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <pcl/common/transforms.h>  // NOLINT(build/include_order)
#include <pcl/filters/voxel_grid.h>  // NOLINT(build/include_order)
#include <pcl/kdtree/kdtree_flann.h>  // NOLINT(build/include_order)
#include <pcl/point_cloud.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)

#include <lidarslam_plugin_interfaces/registration.hpp>

#include "graph_based_slam/candidate_aggregator.hpp"
#include "graph_based_slam/loop_edge_set.hpp"
#include "graph_based_slam/loop_verifier.hpp"
#include "graph_based_slam/scan_context.hpp"
#include "graph_based_slam/solid_descriptor.hpp"
#include "graph_based_slam/submap_bev_descriptor.hpp"
#include "graph_based_slam/target_cloud_cache.hpp"
#include "graph_based_slam/three_d_bbs_loop_verifier.hpp"
#include "graph_based_slam/triangle_descriptor.hpp"
#include "graph_based_slam/triangle_descriptor_database.hpp"

namespace graphslam
{
namespace backend_core
{

struct DescriptorConfig
{
  bool use_scan_context{false};
  bool use_bev_descriptor{false};
  bool use_solid_descriptor{false};
  bool use_triangle_descriptor{false};
  double bev_descriptor_grid_size_m{80.0};
  int bev_descriptor_grid_cells{40};
  int bev_descriptor_yaw_bins{24};
  std::string triangle_descriptor_keypoint_mode{"bev_max_height"};
  double triangle_descriptor_grid_size_m{60.0};
  int triangle_descriptor_grid_cells{100};
  double triangle_descriptor_min_salience_m{0.8};
  int triangle_descriptor_max_keypoints{40};
  double triangle_descriptor_edge_voxel_size_m{0.4};
  double triangle_descriptor_edge_neighbor_radius_m{1.0};
  int triangle_descriptor_edge_min_neighbors{6};
  double triangle_descriptor_edge_min_edgeness{0.5};
  double triangle_descriptor_edge_nms_radius_m{2.0};
  double triangle_descriptor_min_edge_m{2.0};
  double triangle_descriptor_max_edge_m{50.0};
  int triangle_descriptor_max_triangles{3000};
  double triangle_descriptor_edge_bin_m{0.5};
  double triangle_descriptor_quad_feature_bin_m{0.0};
};

// Pose and travel distance of one submap, pre-converted by the shell
// (tf2::fromMsg once per submap; translation() is bit-identical to the
// message position doubles).
struct SubmapMeta
{
  Eigen::Affine3d pose{Eigen::Affine3d::Identity()};
  double travel_distance{0.0};
  // A non-zero revision is required before a target aggregate can enter the
  // cache.  Shells must change it whenever the submap cloud content changes;
  // zero intentionally selects the historical uncached path.
  std::uint64_t content_revision{0};
};

struct LoopEdgeProposal
{
  bool found{false};
  std::pair<int, int> pair_id{-1, -1};
  Eigen::Isometry3d relative_pose{Eigen::Isometry3d::Identity()};
  double fitness_score{0.0};
};

struct LoopSearchOutput
{
  LoopEdgeProposal proposal;
  std::vector<candidate_aggregator::LogLine> logs;
};

struct LoopSearchConfig
{
  int search_submap_num{3};
  // This is the leaf size of the regular target/source voxel grid.  It is
  // part of the target-cache key; callers set it from the existing
  // voxel_leaf_size parameter without changing its default behavior.
  double target_voxel_leaf_size{0.2};
  bool prefer_scan_context_candidates{false};
  bool use_3d_bbs_for_scan_context{false};
  int three_d_bbs_source_submap_num{2};
  int three_d_bbs_target_submap_radius{1};
  double three_d_bbs_voxel_leaf_size{1.0};
  double three_d_bbs_min_level_res{1.0};
  int three_d_bbs_max_level{3};
  double three_d_bbs_score_threshold_percentage{0.25};
  int three_d_bbs_timeout_msec{50};
  int three_d_bbs_num_threads{0};
  double three_d_bbs_translation_search_margin_m{15.0};
  double three_d_bbs_roll_pitch_search_deg{10.0};
  double three_d_bbs_yaw_search_deg{180.0};
  candidate_aggregator::Config aggregator;
  loop_verifier::GateConfig gates;
};

// Registration output is already expressed in the target/world frame.
// Measure how much of that aligned source has actual target support instead
// of relying only on the registration optimizer's aggregate fitness score.
struct RegistrationOverlapMetrics
{
  double source_to_target {0.0};
  double target_to_source {0.0};
  double harmonic_mean {0.0};
  double source_support_rmse_m {0.0};
  double source_support_p90_m {0.0};
};

struct DirectedOverlapMetrics
{
  double ratio {0.0};
  double support_rmse_m {0.0};
  double support_p90_m {0.0};
};

inline pcl::PointCloud<pcl::PointXYZI>::Ptr finiteCloud(
  const pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr finite(new pcl::PointCloud<pcl::PointXYZI>);
  finite->reserve(cloud.size());
  for (const auto & point : cloud) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      finite->push_back(point);
    }
  }
  return finite;
}

inline DirectedOverlapMetrics directedOverlapMetrics(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & query,
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & reference,
  double max_distance_m)
{
  DirectedOverlapMetrics metrics;
  if (!query || !reference || query->empty() || reference->empty() || max_distance_m <= 0.0) {
    return metrics;
  }
  pcl::KdTreeFLANN<pcl::PointXYZI> reference_tree;
  reference_tree.setInputCloud(reference);
  const float max_distance_squared = static_cast<float>(max_distance_m * max_distance_m);
  std::vector<int> neighbor_index(1);
  std::vector<float> neighbor_distance_squared(1);
  std::vector<double> matched_distance_squared;
  matched_distance_squared.reserve(query->size());
  for (const auto & point : *query) {
    if (
      reference_tree.nearestKSearch(point, 1, neighbor_index, neighbor_distance_squared) > 0 &&
      neighbor_distance_squared[0] <= max_distance_squared)
    {
      matched_distance_squared.push_back(neighbor_distance_squared[0]);
    }
  }
  metrics.ratio =
    static_cast<double>(matched_distance_squared.size()) / static_cast<double>(query->size());
  if (matched_distance_squared.empty()) {
    return metrics;
  }
  double sum_squared = 0.0;
  for (const double distance_squared : matched_distance_squared) {
    sum_squared += distance_squared;
  }
  metrics.support_rmse_m =
    std::sqrt(sum_squared / static_cast<double>(matched_distance_squared.size()));
  const std::size_t p90_index =
    (9 * matched_distance_squared.size() + 9) / 10 - 1;
  std::nth_element(
    matched_distance_squared.begin(),
    matched_distance_squared.begin() + static_cast<std::ptrdiff_t>(p90_index),
    matched_distance_squared.end());
  metrics.support_p90_m = std::sqrt(matched_distance_squared[p90_index]);
  return metrics;
}

inline double directedOverlapRatio(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & query,
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & reference,
  double max_distance_m)
{
  return directedOverlapMetrics(query, reference, max_distance_m).ratio;
}

inline RegistrationOverlapMetrics registrationOverlapMetrics(
  const pcl::PointCloud<pcl::PointXYZI> & aligned_source,
  const pcl::PointCloud<pcl::PointXYZI> & target,
  double max_distance_m,
  bool compute_reverse = true)
{
  RegistrationOverlapMetrics metrics;
  if (aligned_source.empty() || target.empty() || max_distance_m <= 0.0) {
    return metrics;
  }
  const auto finite_source = finiteCloud(aligned_source);
  const auto finite_target = finiteCloud(target);
  const DirectedOverlapMetrics forward =
    directedOverlapMetrics(finite_source, finite_target, max_distance_m);
  metrics.source_to_target = forward.ratio;
  metrics.source_support_rmse_m = forward.support_rmse_m;
  metrics.source_support_p90_m = forward.support_p90_m;
  if (!compute_reverse) {
    return metrics;
  }
  metrics.target_to_source = directedOverlapRatio(finite_target, finite_source, max_distance_m);
  const double sum = metrics.source_to_target + metrics.target_to_source;
  if (sum > 0.0) {
    metrics.harmonic_mean =
      2.0 * metrics.source_to_target * metrics.target_to_source / sum;
  }
  return metrics;
}

inline double registrationOverlapRatio(
  const pcl::PointCloud<pcl::PointXYZI> & aligned_source,
  const pcl::PointCloud<pcl::PointXYZI> & target,
  double max_distance_m)
{
  return registrationOverlapMetrics(aligned_source, target, max_distance_m).source_to_target;
}

// Backend-owned loop-closure state. Single-threaded by contract: the owning
// GraphSlamApplication serializes access, while the ROS shell only coalesces
// notifications from a potentially multi-threaded executor.
class BackendCore
{
public:
  using CloudPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
  // Returns the voxel-filtered aggregate of the submap at idx and its
  // recent neighbors, expressed in that submap's local frame.
  using LocalSubmapProvider = std::function<CloudPtr(int idx)>;

  void configure(const DescriptorConfig & config)
  {
    config_ = config;
    target_cloud_cache_.clear();
    bev_descriptor_db_.configure(
      config.bev_descriptor_grid_size_m,
      config.bev_descriptor_grid_cells,
      config.bev_descriptor_yaw_bins);
  }

  // Keep every enabled descriptor database aligned 1:1 with submap
  // indices, computing descriptors for indices that arrived since the
  // last call. Each database family queries the provider independently
  // (historical behavior; the provider result is deterministic, so the
  // recompute is byte-identical).
  void ingestDescriptors(int num_submaps, const LocalSubmapProvider & provider)
  {
    if (config_.use_scan_context && scan_context_db_.nextSubmapIndex() < num_submaps) {
      for (int idx = scan_context_db_.nextSubmapIndex(); idx < num_submaps; ++idx) {
        const auto filtered_aggregated_cloud = provider(idx);
        if (filtered_aggregated_cloud->empty()) {
          scan_context_db_.add(
            idx, ScanContext::Descriptor::Zero(
              ScanContext::NUM_RINGS,
              ScanContext::NUM_SECTORS));
          continue;
        }
        scan_context_db_.add(idx, ScanContext::computeDescriptor(filtered_aggregated_cloud));
      }
    }

    if (config_.use_bev_descriptor && bev_descriptor_db_.nextSubmapIndex() < num_submaps) {
      for (int idx = bev_descriptor_db_.nextSubmapIndex(); idx < num_submaps; ++idx) {
        const auto filtered_aggregated_cloud = provider(idx);
        bev_descriptor_db_.add(
          idx,
          SubmapBEVDescriptor::computeDescriptor(
            filtered_aggregated_cloud,
            config_.bev_descriptor_grid_size_m,
            config_.bev_descriptor_grid_cells));
      }
    }
    if (config_.use_solid_descriptor && solid_descriptor_db_.nextSubmapIndex() < num_submaps) {
      for (int idx = solid_descriptor_db_.nextSubmapIndex(); idx < num_submaps; ++idx) {
        const auto filtered_aggregated_cloud = provider(idx);
        solid_descriptor_db_.add(
          idx,
          SolidDescriptor::computeDescriptor(filtered_aggregated_cloud));
      }
    }
    if (config_.use_triangle_descriptor && triangle_next_submap_idx_ < num_submaps) {
      triangle::KeypointExtractionConfig kp_cfg;
      if (config_.triangle_descriptor_keypoint_mode == "edge_3d") {
        kp_cfg.mode = triangle::KeypointMode::EDGE_3D;
      } else {
        kp_cfg.mode = triangle::KeypointMode::BEV_MAX_HEIGHT;
      }
      kp_cfg.grid_size_m = config_.triangle_descriptor_grid_size_m;
      kp_cfg.grid_cells = config_.triangle_descriptor_grid_cells;
      kp_cfg.min_salience_m = static_cast<float>(config_.triangle_descriptor_min_salience_m);
      kp_cfg.max_keypoints = config_.triangle_descriptor_max_keypoints;
      kp_cfg.edge_voxel_size_m =
        static_cast<float>(config_.triangle_descriptor_edge_voxel_size_m);
      kp_cfg.edge_neighbor_radius_m =
        static_cast<float>(config_.triangle_descriptor_edge_neighbor_radius_m);
      kp_cfg.edge_min_neighbors = config_.triangle_descriptor_edge_min_neighbors;
      kp_cfg.edge_min_edgeness =
        static_cast<float>(config_.triangle_descriptor_edge_min_edgeness);
      kp_cfg.edge_nms_radius_m =
        static_cast<float>(config_.triangle_descriptor_edge_nms_radius_m);
      triangle::TriangleBuildConfig build_cfg;
      build_cfg.min_edge_m = static_cast<float>(config_.triangle_descriptor_min_edge_m);
      build_cfg.max_edge_m = static_cast<float>(config_.triangle_descriptor_max_edge_m);
      build_cfg.max_triangles = config_.triangle_descriptor_max_triangles;
      triangle::HashConfig hash_cfg;
      hash_cfg.edge_bin_m = static_cast<float>(config_.triangle_descriptor_edge_bin_m);
      hash_cfg.quad_feature_bin_m =
        static_cast<float>(config_.triangle_descriptor_quad_feature_bin_m);
      for (int idx = triangle_next_submap_idx_; idx < num_submaps; ++idx) {
        const auto filtered_aggregated_cloud = provider(idx);
        std::vector<triangle::Keypoint> kps;
        std::vector<triangle::TriangleDescriptor> tris;
        if (filtered_aggregated_cloud && !filtered_aggregated_cloud->empty()) {
          kps = triangle::extractKeypoints(*filtered_aggregated_cloud, kp_cfg);
          tris = triangle::buildTriangles(kps, build_cfg);
        }
        candidate_aggregator::TriangleSubmapFeatures entry;
        entry.keypoints = kps;
        entry.triangles = tris;
        triangle_per_submap_.push_back(entry);
        triangle_db_.addSubmap(idx, kps, tris, hash_cfg);
      }
      triangle_next_submap_idx_ = num_submaps;
    }
  }

  // The complete loop search for one query submap: candidate generation
  // (pure aggregator calls over the descriptor databases), registration
  // verification and best-candidate selection. Raw submap clouds come
  // from the provider (message vs PCD cache stays the shell's concern);
  // registration / voxel filter / 3D-BBS compute objects are borrowed from
  // the owning GraphSlamApplication engine for this synchronous operation.
  // Every operator-visible line is returned as a LogLine so the shell
  // emits byte-identical output.
  LoopSearchOutput searchLoopForSubmap(
    const std::vector<SubmapMeta> & submaps,
    int latest_idx,
    const LoopSearchConfig & search_config,
    const LocalSubmapProvider & raw_cloud_provider,
    lidarslam::plugins::registration::RegistrationPlugin & registration,
    pcl::VoxelGrid<pcl::PointXYZI> & voxelgrid,
    ThreeDBBSLoopVerifier & three_d_bbs_verifier)
  {
    using LoopCandidate = loop_verifier::LoopCandidate;
    using LoopCandidateResult = loop_verifier::LoopCandidateResult;

    LoopSearchOutput output;
    const int num_submaps = static_cast<int>(submaps.size());
    const Eigen::Affine3d & latest_affine = submaps[latest_idx].pose;

    // Aggregate latest N submaps as source (improves matching quality)
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_latest_submap_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_latest_submap_cloud_sc_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_local_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_local_bbs_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
    for (int k = 0; k < search_config.search_submap_num && (latest_idx - k) >= 0; k++) {
      int src_idx = latest_idx - k;
      pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud = raw_cloud_provider(src_idx);
      if (src_cloud->empty()) {
        continue;
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_src(new pcl::PointCloud<pcl::PointXYZI>);
      const Eigen::Affine3d & src_affine = submaps[src_idx].pose;
      pcl::transformPointCloud(*src_cloud, *transformed_src, src_affine.matrix().cast<float>());
      *transformed_latest_submap_cloud_ptr += *transformed_src;
      if (k < search_config.three_d_bbs_source_submap_num) {
        *transformed_latest_submap_cloud_sc_ptr += *transformed_src;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_src_local(
        new pcl::PointCloud<pcl::PointXYZI>);
      const Eigen::Matrix4f latest_frame_transform =
        (latest_affine.inverse() * src_affine).matrix().cast<float>();
      pcl::transformPointCloud(*src_cloud, *transformed_src_local, latest_frame_transform);
      *latest_submap_cloud_local_ptr += *transformed_src_local;
      if (k < search_config.three_d_bbs_source_submap_num) {
        *latest_submap_cloud_local_bbs_ptr += *transformed_src_local;
      }
    }
    if (
      transformed_latest_submap_cloud_ptr->empty() ||
      transformed_latest_submap_cloud_sc_ptr->empty() ||
      latest_submap_cloud_local_ptr->empty() ||
      latest_submap_cloud_local_bbs_ptr->empty())
    {
      return output;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source(new pcl::PointCloud<pcl::PointXYZI>);
    voxelgrid.setInputCloud(transformed_latest_submap_cloud_ptr);
    voxelgrid.filter(*filtered_source);
    if (filtered_source->empty()) {
      return output;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_sc(
      new pcl::PointCloud<pcl::PointXYZI>);
    voxelgrid.setInputCloud(transformed_latest_submap_cloud_sc_ptr);
    voxelgrid.filter(*filtered_source_sc);
    if (filtered_source_sc->empty()) {
      return output;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_local(
      new pcl::PointCloud<pcl::PointXYZI>);
    voxelgrid.setInputCloud(latest_submap_cloud_local_ptr);
    voxelgrid.filter(*filtered_source_local);
    if (filtered_source_local->empty()) {
      return output;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_local_bbs(
      new pcl::PointCloud<pcl::PointXYZI>);
    voxelgrid.setInputCloud(latest_submap_cloud_local_bbs_ptr);
    voxelgrid.filter(*filtered_source_local_bbs);
    if (filtered_source_local_bbs->empty()) {
      return output;
    }
    const double latest_moving_distance = submaps[latest_idx].travel_distance;
    const Eigen::Vector3d latest_submap_pos = latest_affine.translation();

    std::vector<LoopCandidate> candidates;

    std::vector<Eigen::Vector3d> submap_positions;
    std::vector<double> submap_travel_distances;
    std::vector<Eigen::Affine3d> submap_affines;
    submap_positions.reserve(latest_idx + 1);
    submap_travel_distances.reserve(latest_idx + 1);
    submap_affines.reserve(latest_idx + 1);
    for (int i = 0; i <= latest_idx; i++) {
      submap_positions.emplace_back(submaps[i].pose.translation());
      submap_travel_distances.push_back(submaps[i].travel_distance);
      submap_affines.push_back(submaps[i].pose);
    }

    std::vector<std::pair<double, int>> distance_candidates =
      candidate_aggregator::collectDistanceCandidates(
      submap_positions, submap_travel_distances, latest_idx, search_config.aggregator);

    if (config_.use_scan_context) {
      candidate_aggregator::collectScanContextCandidate(
        scan_context_db_,
        submap_travel_distances,
        latest_idx,
        search_config.aggregator,
        candidates,
        output.logs);
    }

    if (config_.use_bev_descriptor &&
      bev_descriptor_db_.size() > SubmapBEVDescriptor::DEFAULT_EXCLUDE_RECENT)
    {
      candidate_aggregator::rerankDistanceCandidatesWithBev(
        bev_descriptor_db_,
        submap_affines,
        latest_idx,
        search_config.aggregator,
        distance_candidates,
        candidates,
        output.logs);
    } else {
      candidate_aggregator::appendTopDistanceCandidates(
        distance_candidates, search_config.aggregator, candidates);
    }
    if (config_.use_solid_descriptor) {
      candidate_aggregator::collectSolidCandidates(
        solid_descriptor_db_,
        submap_affines,
        distance_candidates,
        latest_idx,
        search_config.aggregator,
        candidates,
        output.logs);
    }

    if (config_.use_triangle_descriptor) {
      candidate_aggregator::collectTriangleCandidate(
        triangle_db_,
        triangle_per_submap_,
        bev_descriptor_db_,
        config_.use_bev_descriptor,
        submap_travel_distances,
        latest_idx,
        search_config.aggregator,
        candidates,
        output.logs);
    }
    if (candidates.empty()) {
      return output;
    }

    const bool debug = search_config.aggregator.debug;
    char log_buffer[512];

    loop_verifier::SelectionState selection;
    bool attempted_registration = false;

    for (const auto & candidate : candidates) {
      if (candidate.index < 0 || candidate.index >= latest_idx) {
        continue;
      }

      const Eigen::Affine3d & candidate_affine = submaps[candidate.index].pose;
      const bool scan_context_candidate =
        candidate.source == LoopCandidate::Source::SCAN_CONTEXT;
      const TargetCloudCacheVariant target_cache_variant =
        !scan_context_candidate ? TargetCloudCacheVariant::kRegular :
        (search_config.use_3d_bbs_for_scan_context ?
        TargetCloudCacheVariant::kScanContextWithThreeDBbs :
        TargetCloudCacheVariant::kScanContext);
      const int target_neighbor_radius =
        target_cache_variant == TargetCloudCacheVariant::kRegular ?
        search_config.search_submap_num : search_config.three_d_bbs_target_submap_radius;
      TargetCloudCacheKey target_cache_key;
      target_cache_key.candidate_index = candidate.index;
      target_cache_key.variant = target_cache_variant;
      target_cache_key.neighbor_radius = target_neighbor_radius;
      target_cache_key.bbs_neighbor_radius =
        search_config.three_d_bbs_target_submap_radius;
      target_cache_key.voxel_leaf_size = search_config.target_voxel_leaf_size;
      target_cache_key.bbs_voxel_leaf_size = search_config.three_d_bbs_voxel_leaf_size;
      for (int offset = -target_neighbor_radius;
        offset <= target_neighbor_radius; ++offset)
      {
        const int near_idx = candidate.index + offset;
        if (near_idx < 0 || near_idx >= num_submaps) {
          continue;
        }
        TargetCloudCacheRevision revision;
        revision.submap_index = near_idx;
        revision.content_revision = submaps[near_idx].content_revision;
        revision.pose = submaps[near_idx].pose.matrix();
        target_cache_key.revisions.push_back(revision);
      }

      TargetCloudCacheValue target_cache_value;
      const bool target_cache_hit = target_cloud_cache_.lookup(
        target_cache_key, &target_cache_value);
      if (!target_cache_hit) {
        // Build only the aggregate required by this candidate variant.  The
        // local aggregate is intentionally released after voxelization for
        // regular and non-BBS ScanContext candidates; retaining it would
        // recreate the RSS regression that motivated this cache.
        TargetCloudPtr aggregate(new TargetCloud);
        for (int offset = -target_neighbor_radius;
          offset <= target_neighbor_radius; ++offset)
        {
          const int near_idx = candidate.index + offset;
          if (near_idx < 0 || near_idx >= num_submaps) {
            continue;
          }
          pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr = raw_cloud_provider(near_idx);
          if (submap_cloud_ptr->empty()) {
            continue;
          }
          pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZI>);
          const Eigen::Affine3d & affine = submaps[near_idx].pose;
          pcl::transformPointCloud(
            *submap_cloud_ptr, *transformed_submap_cloud_ptr,
            affine.matrix().cast<float>());
          *aggregate += *transformed_submap_cloud_ptr;
        }
        if (aggregate->empty()) {
          continue;
        }

        TargetCloudPtr filtered(new TargetCloud);
        voxelgrid.setInputCloud(aggregate);
        voxelgrid.filter(*filtered);
        if (filtered->empty()) {
          continue;
        }
        if (target_cache_variant == TargetCloudCacheVariant::kRegular) {
          target_cache_value.filtered = filtered;
        } else {
          target_cache_value.filtered_bbs = filtered;
          if (target_cache_variant == TargetCloudCacheVariant::kScanContextWithThreeDBbs) {
            target_cache_value.bbs_aggregate = aggregate;
          }
        }
        // A zero content revision makes the key non-cacheable, so this
        // insertion is a no-op for legacy/unknown providers.  The generic
        // RegistrationPlugin contract is unaffected: setInputTarget below
        // still runs for every candidate, including cache hits.
        target_cloud_cache_.insert(target_cache_key, target_cache_value);
      }

      const TargetCloudPtr & submap_clouds_bbs_ptr = target_cache_value.bbs_aggregate;
      const TargetCloudPtr & filtered_clouds_ptr = target_cache_value.filtered;
      const TargetCloudPtr & filtered_clouds_sc_ptr = target_cache_value.filtered_bbs;

      pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
      bool used_3d_bbs = false;
      double three_d_bbs_score_percentage = 0.0;
      double three_d_bbs_elapsed_msec = 0.0;
      Eigen::Matrix4f initial_guess =
        loop_verifier::computeInitialGuess(candidate_affine, latest_affine, candidate);
      const pcl::PointCloud<pcl::PointXYZI>::Ptr registration_source =
        candidate.source == LoopCandidate::Source::SCAN_CONTEXT ?
        filtered_source_sc : filtered_source;
      const pcl::PointCloud<pcl::PointXYZI>::Ptr registration_target =
        candidate.source == LoopCandidate::Source::SCAN_CONTEXT ?
        filtered_clouds_sc_ptr : filtered_clouds_ptr;
      if (candidate.source == LoopCandidate::Source::SCAN_CONTEXT &&
        search_config.use_3d_bbs_for_scan_context)
      {
        pcl::VoxelGrid<pcl::PointXYZI> three_d_bbs_voxelgrid;
        three_d_bbs_voxelgrid.setLeafSize(
          search_config.three_d_bbs_voxel_leaf_size,
          search_config.three_d_bbs_voxel_leaf_size,
          search_config.three_d_bbs_voxel_leaf_size);
        pcl::PointCloud<pcl::PointXYZI>::Ptr three_d_bbs_source(
          new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr three_d_bbs_target(
          new pcl::PointCloud<pcl::PointXYZI>);
        three_d_bbs_voxelgrid.setInputCloud(filtered_source_local_bbs);
        three_d_bbs_voxelgrid.filter(*three_d_bbs_source);
        three_d_bbs_voxelgrid.setInputCloud(submap_clouds_bbs_ptr);
        three_d_bbs_voxelgrid.filter(*three_d_bbs_target);

        ThreeDBBSLoopVerifierConfig bbs_config;
        bbs_config.min_level_res = search_config.three_d_bbs_min_level_res;
        bbs_config.max_level = search_config.three_d_bbs_max_level;
        bbs_config.score_threshold_percentage =
          search_config.three_d_bbs_score_threshold_percentage;
        bbs_config.timeout_msec = search_config.three_d_bbs_timeout_msec;
        bbs_config.num_threads = search_config.three_d_bbs_num_threads;
        bbs_config.translation_search_margin_m =
          search_config.three_d_bbs_translation_search_margin_m;
        bbs_config.roll_pitch_search_deg = search_config.three_d_bbs_roll_pitch_search_deg;
        bbs_config.yaw_search_deg = search_config.three_d_bbs_yaw_search_deg;
        const auto bbs_result = three_d_bbs_verifier.localize(
          three_d_bbs_source,
          three_d_bbs_target,
          Eigen::Isometry3d(latest_affine.matrix()),
          Eigen::Isometry3d(candidate_affine.matrix()),
          bbs_config);
        if (bbs_result.available) {
          three_d_bbs_score_percentage = bbs_result.score_percentage;
          three_d_bbs_elapsed_msec = bbs_result.elapsed_msec;
          used_3d_bbs = bbs_result.localized;
          if (bbs_result.localized) {
            initial_guess = bbs_result.correction_guess;
          }
          if (debug) {
            std::snprintf(
              log_buffer, sizeof(log_buffer),
              "3D-BBS %s for loop candidate %d -> %d "
              "(score=%.3f elapsed=%.2f ms timed_out=%s "
              "src=%zu tar=%zu)",
              bbs_result.localized ? "localized" : "missed",
              candidate.index,
              latest_idx,
              bbs_result.score_percentage,
              bbs_result.elapsed_msec,
              bbs_result.timed_out ? "true" : "false",
              three_d_bbs_source->size(),
              three_d_bbs_target->size());
            output.logs.push_back({true, std::string(log_buffer)});
          }
        }
      }
      attempted_registration = true;
      std::string target_error;
      // The cloud cache is deliberately below the plugin boundary.  Every
      // candidate still calls setInputTarget, because only a concrete plugin
      // may know whether its target cells/state can be safely reused; generic
      // RegistrationPlugin implementations must not inherit an NDT-specific
      // target_cells_ assumption.
      if (!registration.setInputTarget(registration_target, &target_error)) {
        if (debug) {
          std::snprintf(
            log_buffer, sizeof(log_buffer),
            "Rejected loop candidate %d -> %d because registration target setup failed: %s",
            candidate.index,
            latest_idx,
            target_error.c_str());
          output.logs.push_back({true, std::string(log_buffer)});
        }
        continue;
      }
      lidarslam::plugins::registration::AlignmentRequest alignment_request;
      alignment_request.source = registration_source;
      alignment_request.initial_guess_enabled =
        loop_verifier::shouldUseInitialGuess(candidate.source, used_3d_bbs);
      if (alignment_request.initial_guess_enabled) {
        alignment_request.initial_guess = initial_guess;
      }
      const auto alignment_result = registration.align(alignment_request);
      if (alignment_result.failure !=
        lidarslam::plugins::registration::FailureCode::kNone ||
        !alignment_result.converged || !alignment_result.aligned_source)
      {
        if (debug) {
          std::snprintf(
            log_buffer, sizeof(log_buffer),
            "Rejected loop candidate %d -> %d because registration did not converge",
            candidate.index,
            latest_idx);
          output.logs.push_back({true, std::string(log_buffer)});
        }
        continue;
      }

      output_cloud_ptr = alignment_result.aligned_source;
      const double fitness_score = alignment_result.fitness_score;
      const Eigen::Matrix4f final_transformation = alignment_result.final_transformation;
      const loop_verifier::RegistrationDelta registration_delta =
        loop_verifier::computeRegistrationDelta(final_transformation);
      // Avoid building a target KD-tree for candidates that an earlier,
      // cheaper gate already rejects. Passing overlap=1 cannot trigger the
      // overlap gate and preserves the historical gate order.
      const loop_verifier::GateResult pre_overlap_gate = loop_verifier::evaluateGates(
        candidate.source, fitness_score, registration_delta, search_config.gates, 1.0);
      RegistrationOverlapMetrics overlap_metrics;
      if (pre_overlap_gate.rejection == loop_verifier::GateRejection::NONE) {
        const pcl::PointCloud<pcl::PointXYZI>::Ptr overlap_target =
          candidate.source == LoopCandidate::Source::SCAN_CONTEXT ?
          filtered_clouds_sc_ptr : filtered_clouds_ptr;
        overlap_metrics = registrationOverlapMetrics(
          *output_cloud_ptr, *overlap_target, search_config.gates.overlap_max_distance_m,
          debug);
      }

      LoopCandidateResult candidate_result;
      candidate_result.index = candidate.index;
      candidate_result.selection_metric = candidate.selection_metric;
      candidate_result.fitness_score = fitness_score;
      candidate_result.travel_distance =
        latest_moving_distance - submaps[candidate.index].travel_distance;
      const Eigen::Vector3d candidate_submap_pos = candidate_affine.translation();
      candidate_result.euclidean_distance = (latest_submap_pos - candidate_submap_pos).norm();
      candidate_result.translation_delta_m = registration_delta.translation_m;
      candidate_result.rotation_delta_deg = registration_delta.rotation_deg;
      candidate_result.overlap_ratio = overlap_metrics.source_to_target;
      candidate_result.reverse_overlap_ratio = overlap_metrics.target_to_source;
      candidate_result.mutual_overlap_ratio = overlap_metrics.harmonic_mean;
      candidate_result.support_rmse_m = overlap_metrics.source_support_rmse_m;
      candidate_result.support_p90_m = overlap_metrics.source_support_p90_m;
      candidate_result.source = candidate.source;
      candidate_result.used_3d_bbs = used_3d_bbs;
      candidate_result.three_d_bbs_score_percentage = three_d_bbs_score_percentage;
      candidate_result.three_d_bbs_elapsed_msec = three_d_bbs_elapsed_msec;
      candidate_result.final_transformation = final_transformation;

      selection.considerConverged(candidate_result);

      const loop_verifier::GateResult gate = loop_verifier::evaluateGates(
        candidate.source, fitness_score, registration_delta, search_config.gates,
        overlap_metrics.source_to_target);
      if (gate.rejection != loop_verifier::GateRejection::NONE) {
        if (debug) {
          switch (gate.rejection) {
            case loop_verifier::GateRejection::FITNESS:
              std::snprintf(
                log_buffer, sizeof(log_buffer),
                "Rejected loop candidate %d -> %d because fitness %.6f exceeds threshold %.6f",
                candidate.index,
                latest_idx,
                fitness_score,
                gate.score_threshold);
              output.logs.push_back({true, std::string(log_buffer)});
              break;
            case loop_verifier::GateRejection::TRANSLATION:
              std::snprintf(
                log_buffer, sizeof(log_buffer),
                "Rejected loop candidate %d -> %d because translation correction %.3f m "
                "exceeds %.3f m",
                candidate.index,
                latest_idx,
                registration_delta.translation_m,
                gate.translation_cap_m);
              output.logs.push_back({true, std::string(log_buffer)});
              break;
            case loop_verifier::GateRejection::ROTATION:
              std::snprintf(
                log_buffer, sizeof(log_buffer),
                "Rejected loop candidate %d -> %d because rotation correction %.3f deg "
                "exceeds %.3f deg",
                candidate.index,
                latest_idx,
                registration_delta.rotation_deg,
                gate.rotation_cap_deg);
              output.logs.push_back({true, std::string(log_buffer)});
              break;
            case loop_verifier::GateRejection::OVERLAP:
              std::snprintf(
                log_buffer, sizeof(log_buffer),
                "Rejected loop candidate %d -> %d because overlap %.6f is below %.6f "
                "(max neighbor distance %.3f m)",
                candidate.index,
                latest_idx,
                overlap_metrics.source_to_target,
                gate.min_overlap_ratio,
                search_config.gates.overlap_max_distance_m);
              output.logs.push_back({true, std::string(log_buffer)});
              break;
            default:
              break;
          }
        }
        continue;
      }

      candidate_result.valid = true;
      selection.considerValid(candidate_result);
    }

    if (search_config.prefer_scan_context_candidates && selection.best_scan_context.valid) {
      if (
        !selection.best_valid.valid ||
        selection.best_valid.index != selection.best_scan_context.index ||
        selection.best_valid.source != LoopCandidate::Source::SCAN_CONTEXT)
      {
        std::ostringstream prefer_line;
        prefer_line << "Preferring valid ScanContext candidate id:" <<
          selection.best_scan_context.index << " over best candidate id:" <<
          (selection.best_valid.valid ?
        std::to_string(selection.best_valid.index) : std::string("none"));
        output.logs.push_back({false, prefer_line.str()});
      }
    }
    const LoopCandidateResult best_candidate =
      selection.select(search_config.prefer_scan_context_candidates);

    if (!best_candidate.valid) {
      const LoopCandidateResult & best_attempt = selection.best_attempt;
      if (best_attempt.index >= 0) {
        std::ostringstream attempt_line;
        attempt_line << "best_loop_candidate id:" << best_attempt.index
                     << " source:" << loop_verifier::sourceName(best_attempt.source)
                     << " latest_id:" << latest_idx
                     << " travel_distance:" << best_attempt.travel_distance
                     << " euclidean_distance:" << best_attempt.euclidean_distance
                     << " fitness:" << best_attempt.fitness_score
                     << " correction_translation:" << best_attempt.translation_delta_m
                     << " correction_rotation_deg:" << best_attempt.rotation_delta_deg
                     << " overlap_ratio:" << best_attempt.overlap_ratio
                     << " support_rmse_m:" << best_attempt.support_rmse_m
                     << " support_p90_m:" << best_attempt.support_p90_m;
        if (debug) {
          attempt_line << " reverse_overlap_ratio:" << best_attempt.reverse_overlap_ratio
                       << " mutual_overlap_ratio:" << best_attempt.mutual_overlap_ratio;
        }
        attempt_line
                     << " used_3d_bbs:" << best_attempt.used_3d_bbs
                     << " 3d_bbs_score:" << best_attempt.three_d_bbs_score_percentage;
        output.logs.push_back({false, attempt_line.str()});
      } else if (attempted_registration && debug) {
        std::snprintf(
          log_buffer, sizeof(log_buffer),
          "No converged loop candidate remained for latest submap %d",
          latest_idx);
        output.logs.push_back({true, std::string(log_buffer)});
      }
      return output;
    }

    output.proposal.found = true;
    output.proposal.pair_id = std::pair<int, int>(best_candidate.index, latest_idx);
    output.proposal.relative_pose = loop_verifier::composeLoopRelativePose(
      submaps[best_candidate.index].pose, latest_affine, best_candidate.final_transformation);
    output.proposal.fitness_score = best_candidate.fitness_score;

    output.logs.push_back({false, "---"});
    std::ostringstream adjustment_line;
    adjustment_line << "PoseAdjustment distance:" << best_candidate.travel_distance
                    << ", score:" << best_candidate.fitness_score;
    output.logs.push_back({false, adjustment_line.str()});
    std::ostringstream id_line;
    id_line << "id_loop_point 1:" << best_candidate.index
            << " id_loop_point 2:" << latest_idx;
    output.logs.push_back({false, id_line.str()});
    std::ostringstream source_line;
    source_line << "loop_candidate_source:" << loop_verifier::sourceName(best_candidate.source);
    output.logs.push_back({false, source_line.str()});
    if (best_candidate.used_3d_bbs) {
      std::ostringstream bbs_line;
      bbs_line << "3d_bbs_score_percentage:" << best_candidate.three_d_bbs_score_percentage
               << " elapsed_msec:" << best_candidate.three_d_bbs_elapsed_msec;
      output.logs.push_back({false, bbs_line.str()});
    }
    std::ostringstream correction_line;
    correction_line << "correction translation[m]:" << best_candidate.translation_delta_m
                    << " rotation[deg]:" << best_candidate.rotation_delta_deg
                    << " overlap_ratio:" << best_candidate.overlap_ratio
                    << " support_rmse_m:" << best_candidate.support_rmse_m
                    << " support_p90_m:" << best_candidate.support_p90_m;
    if (debug) {
      correction_line << " reverse_overlap_ratio:" << best_candidate.reverse_overlap_ratio
                      << " mutual_overlap_ratio:" << best_candidate.mutual_overlap_ratio;
    }
    output.logs.push_back({false, correction_line.str()});
    output.logs.push_back({false, "final transformation:"});
    std::ostringstream transformation_line;
    transformation_line << best_candidate.final_transformation;
    output.logs.push_back({false, transformation_line.str()});

    return output;
  }

  const DescriptorConfig & config() const {return config_;}

  ScanContext::Database & scanContextDb() {return scan_context_db_;}
  SubmapBEVDescriptor::Database & bevDescriptorDb() {return bev_descriptor_db_;}
  SolidDescriptor::Database & solidDescriptorDb() {return solid_descriptor_db_;}
  triangle::TriangleDatabase & triangleDb() {return triangle_db_;}
  const std::vector<candidate_aggregator::TriangleSubmapFeatures> & trianglePerSubmap() const
  {
    return triangle_per_submap_;
  }

  const TargetCloudCache & targetCloudCache() const {return target_cloud_cache_;}

private:
  DescriptorConfig config_;
  ScanContext::Database scan_context_db_;
  SubmapBEVDescriptor::Database bev_descriptor_db_;
  SolidDescriptor::Database solid_descriptor_db_;
  triangle::TriangleDatabase triangle_db_;
  std::vector<candidate_aggregator::TriangleSubmapFeatures> triangle_per_submap_;
  int triangle_next_submap_idx_{0};
  TargetCloudCache target_cloud_cache_;
};

}  // namespace backend_core
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__BACKEND_CORE_HPP_
