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

#include "graph_based_slam/graph_slam_application.hpp"

#include <algorithm>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "graph_based_slam/loop_search_schedule.hpp"
#include "graph_based_slam/map_saver.hpp"
#include "graph_based_slam/registration_factory.hpp"

namespace graphslam
{

class GraphSlamApplication::Engine
{
public:
  using CloudPtr = backend_core::BackendCore::CloudPtr;
  using LocalSubmapProvider = backend_core::BackendCore::LocalSubmapProvider;

  explicit Engine(GraphSlamApplicationConfig application_config)
  : config(std::move(application_config))
  {
    if (config.loop_search_query_stride < 1) {
      throw std::invalid_argument("loop_search_query_stride must be at least 1");
    }
    if (config.loop_edge_dedup_index_window < 0) {
      throw std::invalid_argument("loop_edge_dedup_index_window must not be negative");
    }
    if (!(config.voxel_leaf_size > 0.0)) {
      throw std::invalid_argument("voxel_leaf_size must be positive");
    }
    registration = backend_core::makeLoopRegistration(
      config.registration_method, config.ndt_resolution, config.ndt_num_threads);
    if (!registration) {
      throw std::invalid_argument("unknown registration_method: " + config.registration_method);
    }
    const float voxel_leaf_size = static_cast<float>(config.voxel_leaf_size);
    voxelgrid.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    backend.configure(config.descriptors);
    loop_edges.configure(config.loop_edge_dedup_index_window);
  }

  CloudPtr filteredLocalSubmap(
    const std::vector<backend_core::SubmapMeta> & ordered_submaps,
    const LocalSubmapProvider & raw_cloud_provider,
    int reference_index)
  {
    CloudPtr aggregate(new pcl::PointCloud<pcl::PointXYZI>);
    const Eigen::Affine3d & reference_pose = ordered_submaps[reference_index].pose;
    for (int offset = 0;
      offset < config.loop_search.search_submap_num && reference_index - offset >= 0;
      ++offset)
    {
      const int source_index = reference_index - offset;
      const CloudPtr source = raw_cloud_provider(source_index);
      if (!source || source->empty()) {
        continue;
      }
      CloudPtr transformed(new pcl::PointCloud<pcl::PointXYZI>);
      const Eigen::Matrix4f local_transform =
        (reference_pose.inverse() * ordered_submaps[source_index].pose).matrix().cast<float>();
      pcl::transformPointCloud(*source, *transformed, local_transform);
      *aggregate += *transformed;
    }

    CloudPtr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    if (!aggregate->empty()) {
      voxelgrid.setInputCloud(aggregate);
      voxelgrid.filter(*filtered);
    }
    return filtered;
  }

  GraphSlamApplicationConfig config;
  backend_core::BackendCore backend;
  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  ThreeDBBSLoopVerifier three_d_bbs_verifier;
  mutable std::mutex mutex;
  int next_query_index {1};
  backend_core::LoopEdgeSet loop_edges;
};

GraphSlamApplication::GraphSlamApplication(GraphSlamApplicationConfig config)
: engine_(new Engine(std::move(config)))
{
}

GraphSlamApplication::~GraphSlamApplication() = default;

std::vector<LoopSearchEvent> GraphSlamApplication::processSubmaps(
  const std::vector<backend_core::SubmapMeta> & ordered_submaps,
  const LocalSubmapProvider & raw_cloud_provider)
{
  std::lock_guard<std::mutex> lock(engine_->mutex);
  std::vector<LoopSearchEvent> events;
  const int submap_count = static_cast<int>(ordered_submaps.size());
  if (engine_->next_query_index >= submap_count) {
    return events;
  }

  const LocalSubmapProvider filtered_local_provider =
    [this, &ordered_submaps, &raw_cloud_provider](int index) {
      return engine_->filteredLocalSubmap(ordered_submaps, raw_cloud_provider, index);
    };
  events.reserve(static_cast<std::size_t>(submap_count - engine_->next_query_index));
  for (; engine_->next_query_index < submap_count; ++engine_->next_query_index) {
    LoopSearchEvent event;
    event.query_index = engine_->next_query_index;
    engine_->backend.ingestDescriptors(engine_->next_query_index + 1, filtered_local_provider);
    event.registration_searched = loop_search_schedule::shouldSearch(
      engine_->next_query_index, engine_->config.loop_search_query_stride);
    if (event.registration_searched) {
      // Expose only the ordered prefix visible to this query. This makes
      // batching observationally equivalent to one-submap-at-a-time input.
      std::vector<backend_core::SubmapMeta> visible(
        ordered_submaps.begin(), ordered_submaps.begin() + engine_->next_query_index + 1);
      event.search_output = engine_->backend.searchLoopForSubmap(
        visible, engine_->next_query_index, engine_->config.loop_search, raw_cloud_provider,
        *engine_->registration, engine_->voxelgrid, engine_->three_d_bbs_verifier);
      if (event.search_output.proposal.found) {
        LoopEdge edge;
        edge.pair_id = event.search_output.proposal.pair_id;
        edge.relative_pose = event.search_output.proposal.relative_pose;
        edge.fitness_score = event.search_output.proposal.fitness_score;
        event.graph_changed = engine_->loop_edges.upsert(edge);
      }
    }
    event.loop_edges = engine_->loop_edges.edges();
    events.push_back(std::move(event));
  }
  return events;
}

bool GraphSlamApplication::upsertLoopEdge(const LoopEdge & edge)
{
  std::lock_guard<std::mutex> lock(engine_->mutex);
  return engine_->loop_edges.upsert(edge);
}

GraphSlamStateSnapshot GraphSlamApplication::stateSnapshot() const
{
  std::lock_guard<std::mutex> lock(engine_->mutex);
  GraphSlamStateSnapshot result;
  result.next_query_index = engine_->next_query_index;
  result.loop_edges = engine_->loop_edges.edges();
  return result;
}

namespace
{
std::vector<pose_graph::LoopConstraint> loopConstraintsForPrefix(
  const std::vector<GraphSlamApplication::LoopEdge> & graph_edges,
  std::size_t submap_count)
{
  std::vector<pose_graph::LoopConstraint> constraints;
  constraints.reserve(graph_edges.size());
  for (const auto & edge : graph_edges) {
    if (
      edge.pair_id.first < 0 || edge.pair_id.second < 0 ||
      edge.pair_id.first >= static_cast<int>(submap_count) ||
      edge.pair_id.second >= static_cast<int>(submap_count))
    {
      continue;
    }
    pose_graph::LoopConstraint constraint;
    constraint.from = edge.pair_id.first;
    constraint.to = edge.pair_id.second;
    constraint.relative_pose = edge.relative_pose;
    constraint.fitness_score = edge.fitness_score;
    constraints.push_back(constraint);
  }
  return constraints;
}

pose_graph::OptimizationResult optimizeWithEdges(
  const PoseGraphRequest & request,
  const std::vector<GraphSlamApplication::LoopEdge> & graph_edges)
{
  const auto loop_constraints = loopConstraintsForPrefix(
    graph_edges, request.submaps.size());
  return pose_graph::optimizePoseGraph(
    request.submaps, loop_constraints, request.imu_constraints,
    request.gnss_constraints, request.adjacent_config, request.loop_config,
    request.imu_config, request.chi2_collection, request.fix_first_vertex,
    request.iterations, request.plane_constraints);
}

DeterministicArtifacts serializeWithEdges(
  const ArtifactRequest & request,
  const std::vector<GraphSlamApplication::LoopEdge> & graph_edges)
{
  if (request.timestamps.size() != request.optimized_poses.size()) {
    throw std::invalid_argument("artifact timestamps and optimized poses must have equal size");
  }

  DeterministicArtifacts artifacts;
  std::ostringstream loop_csv;
  loop_csv << map_saver::loopEdgesCsvHeader() << "\n";
  const int pose_count = static_cast<int>(request.optimized_poses.size());
  for (const auto & edge : graph_edges) {
    if (
      edge.pair_id.first < 0 || edge.pair_id.second < 0 ||
      edge.pair_id.first >= pose_count || edge.pair_id.second >= pose_count)
    {
      continue;
    }
    const Eigen::Vector3d translation = edge.relative_pose.translation();
    const Eigen::Quaterniond orientation(edge.relative_pose.rotation());
    map_saver::LoopEdgeRecord record;
    record.from = edge.pair_id.first;
    record.to = edge.pair_id.second;
    record.fitness = edge.fitness_score;
    record.tx = translation.x();
    record.ty = translation.y();
    record.tz = translation.z();
    record.qx = orientation.x();
    record.qy = orientation.y();
    record.qz = orientation.z();
    record.qw = orientation.w();
    loop_csv << map_saver::loopEdgeCsvLine(record) << "\n";
    ++artifacts.loop_edge_count;
  }
  artifacts.loop_edges_csv = loop_csv.str();

  std::ostringstream trajectory;
  for (std::size_t i = 0; i < request.optimized_poses.size(); ++i) {
    const Eigen::Vector3d translation = request.optimized_poses[i].translation();
    const Eigen::Quaterniond orientation(request.optimized_poses[i].rotation());
    map_saver::TrajectoryPose record;
    record.timestamp = request.timestamps[i];
    record.tx = translation.x();
    record.ty = translation.y();
    record.tz = translation.z();
    record.qx = orientation.x();
    record.qy = orientation.y();
    record.qz = orientation.z();
    record.qw = orientation.w();
    trajectory << map_saver::trajectoryTumLine(record) << "\n";
  }
  artifacts.trajectory_optimized_tum = trajectory.str();
  return artifacts;
}
}  // namespace

pose_graph::OptimizationResult GraphSlamApplication::optimize(
  const PoseGraphRequest & request) const
{
  std::vector<LoopEdge> graph_edges;
  {
    std::lock_guard<std::mutex> lock(engine_->mutex);
    graph_edges = engine_->loop_edges.edges();
  }
  return optimizeWithEdges(request, graph_edges);
}

OptimizationArtifacts GraphSlamApplication::optimizeAndSerialize(
  const PoseGraphRequest & request,
  const std::vector<double> & timestamps) const
{
  std::vector<LoopEdge> graph_edges;
  {
    std::lock_guard<std::mutex> lock(engine_->mutex);
    graph_edges = engine_->loop_edges.edges();
  }
  OptimizationArtifacts result;
  result.optimization = optimizeWithEdges(request, graph_edges);
  ArtifactRequest artifact_request;
  artifact_request.timestamps = timestamps;
  artifact_request.optimized_poses = result.optimization.poses;
  result.artifacts = serializeWithEdges(artifact_request, graph_edges);
  return result;
}

DeterministicArtifacts GraphSlamApplication::deterministicArtifacts(
  const ArtifactRequest & request) const
{
  std::vector<LoopEdge> graph_edges;
  {
    std::lock_guard<std::mutex> lock(engine_->mutex);
    graph_edges = engine_->loop_edges.edges();
  }

  return serializeWithEdges(request, graph_edges);
}

}  // namespace graphslam
