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

#ifndef GRAPH_BASED_SLAM__GRAPH_SLAM_APPLICATION_HPP_
#define GRAPH_BASED_SLAM__GRAPH_SLAM_APPLICATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "graph_based_slam/backend_core.hpp"
#include "graph_based_slam/loop_edge_set.hpp"
#include "graph_based_slam/pose_graph_optimization.hpp"

namespace graphslam
{

// The ordered, ROS-free workflow boundary shared by live and replay adapters.
// It is also the lifetime boundary for every stateful mapping-engine resource:
// descriptor databases, registration, filtering, 3D-BBS, scheduling and the
// canonical graph. Adapters provide ordered data, never engine objects.
struct GraphSlamApplicationConfig
{
  backend_core::DescriptorConfig descriptors;
  backend_core::LoopSearchConfig loop_search;
  std::string registration_method {"NDT"};
  double ndt_resolution {3.0};
  int ndt_num_threads {0};
  double voxel_leaf_size {0.2};
  int loop_search_query_stride {1};
  int loop_edge_dedup_index_window {8};
};

struct LoopSearchEvent
{
  int query_index {-1};
  bool registration_searched {false};
  backend_core::LoopSearchOutput search_output;
  bool graph_changed {false};
  std::vector<backend_core::LoopEdgeSet::Edge> loop_edges;
};

struct PoseGraphRequest
{
  std::vector<pose_graph::SubmapNode> submaps;
  std::vector<pose_graph::ImuRotationConstraint> imu_constraints;
  std::vector<pose_graph::GnssConstraint> gnss_constraints;
  pose_graph::AdjacentEdgeConfig adjacent_config;
  pose_graph::LoopEdgeConfig loop_config;
  pose_graph::ImuEdgeConfig imu_config;
  pose_graph::Chi2Collection chi2_collection {pose_graph::Chi2Collection::NONE};
  bool fix_first_vertex {true};
  int iterations {10};
  std::vector<pose_graph::PlaneRevisitConstraint> plane_constraints;
};

struct GraphSlamStateSnapshot
{
  int next_query_index {1};
  std::vector<backend_core::LoopEdgeSet::Edge> loop_edges;
};

struct ArtifactRequest
{
  std::vector<double> timestamps;
  std::vector<Eigen::Isometry3d> optimized_poses;
};

struct DeterministicArtifacts
{
  std::string loop_edges_csv;
  std::string trajectory_optimized_tum;
  std::size_t loop_edge_count {0};
};

struct OptimizationArtifacts
{
  pose_graph::OptimizationResult optimization;
  DeterministicArtifacts artifacts;
};

class GraphSlamApplication
{
public:
  using LocalSubmapProvider = backend_core::BackendCore::LocalSubmapProvider;
  using LoopEdge = backend_core::LoopEdgeSet::Edge;

  explicit GraphSlamApplication(GraphSlamApplicationConfig config);
  ~GraphSlamApplication();

  GraphSlamApplication(const GraphSlamApplication &) = delete;
  GraphSlamApplication & operator=(const GraphSlamApplication &) = delete;

  // Process every not-yet-observed query from the ordered prefix. The same
  // input prefix produces the same event order regardless of callback/bag
  // batching. Providers are invoked synchronously and are not retained.
  std::vector<LoopSearchEvent> processSubmaps(
    const std::vector<backend_core::SubmapMeta> & ordered_submaps,
    const LocalSubmapProvider & raw_cloud_provider);

  bool upsertLoopEdge(const LoopEdge & edge);
  GraphSlamStateSnapshot stateSnapshot() const;

  pose_graph::OptimizationResult optimize(const PoseGraphRequest & request) const;

  // One canonical graph snapshot drives optimization and artifact bytes.
  // Both live and replay adapters use this transaction boundary.
  OptimizationArtifacts optimizeAndSerialize(
    const PoseGraphRequest & request,
    const std::vector<double> & timestamps) const;

  // Canonical artifact bytes shared by live ROS and offline replay adapters.
  // Edges outside the supplied optimized prefix are excluded exactly as they
  // are by optimize(), preserving query-prefix semantics after a batched drain.
  DeterministicArtifacts deterministicArtifacts(const ArtifactRequest & request) const;

private:
  class Engine;
  std::unique_ptr<Engine> engine_;
};

}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__GRAPH_SLAM_APPLICATION_HPP_
