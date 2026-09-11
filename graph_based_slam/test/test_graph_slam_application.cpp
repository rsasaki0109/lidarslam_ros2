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

#include <memory>
#include <vector>

#include "graph_based_slam/graph_slam_application.hpp"

namespace graphslam
{
namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

struct Fixture
{
  std::unique_ptr<GraphSlamApplication> makeApplication(int stride = 1)
  {
    GraphSlamApplicationConfig config;
    config.registration_method = "GICP";
    config.loop_search_query_stride = stride;
    return std::unique_ptr<GraphSlamApplication>(new GraphSlamApplication(config));
  }
};

backend_core::BackendCore::CloudPtr emptyCloud(int index)
{
  static_cast<void>(index);
  backend_core::BackendCore::CloudPtr cloud(new Cloud);
  return cloud;
}

std::vector<backend_core::SubmapMeta> submaps(int count)
{
  std::vector<backend_core::SubmapMeta> result(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    result[static_cast<std::size_t>(i)].pose.translation().x() = i * 10.0;
    result[static_cast<std::size_t>(i)].travel_distance = i * 10.0;
  }
  return result;
}

TEST(GraphSlamApplication, BatchAndIncrementalInputProduceTheSameQueryOrder)
{
  Fixture batch_fixture;
  auto batch = batch_fixture.makeApplication();
  const auto batch_events = batch->processSubmaps(submaps(5), emptyCloud);

  Fixture incremental_fixture;
  auto incremental = incremental_fixture.makeApplication();
  std::vector<LoopSearchEvent> incremental_events;
  for (int count = 1; count <= 5; ++count) {
    const auto events = incremental->processSubmaps(submaps(count), emptyCloud);
    incremental_events.insert(incremental_events.end(), events.begin(), events.end());
  }

  ASSERT_EQ(batch_events.size(), 4U);
  ASSERT_EQ(incremental_events.size(), batch_events.size());
  for (std::size_t i = 0; i < batch_events.size(); ++i) {
    EXPECT_EQ(batch_events[i].query_index, incremental_events[i].query_index);
    EXPECT_EQ(batch_events[i].registration_searched, incremental_events[i].registration_searched);
    EXPECT_EQ(batch_events[i].graph_changed, incremental_events[i].graph_changed);
    EXPECT_EQ(batch_events[i].loop_edges.size(), incremental_events[i].loop_edges.size());
  }
  EXPECT_EQ(batch->stateSnapshot().next_query_index, 5);
  EXPECT_EQ(incremental->stateSnapshot().next_query_index, 5);
}

TEST(GraphSlamApplication, StrideSkipsRegistrationButStillAdvancesEveryQuery)
{
  Fixture fixture;
  auto application = fixture.makeApplication(2);
  const auto events = application->processSubmaps(submaps(6), emptyCloud);

  ASSERT_EQ(events.size(), 5U);
  EXPECT_TRUE(events[0].registration_searched);
  EXPECT_FALSE(events[1].registration_searched);
  EXPECT_TRUE(events[2].registration_searched);
  EXPECT_FALSE(events[3].registration_searched);
  EXPECT_TRUE(events[4].registration_searched);
  EXPECT_EQ(application->stateSnapshot().next_query_index, 6);
}

TEST(GraphSlamApplication, OwnsDeterministicDescriptorAggregationAndFiltering)
{
  GraphSlamApplicationConfig config;
  config.registration_method = "GICP";
  config.descriptors.use_scan_context = true;
  config.loop_search.search_submap_num = 3;
  config.loop_search_query_stride = 100;
  GraphSlamApplication application(config);
  std::vector<int> requested_indices;
  const auto provider = [&requested_indices](int index) {
      requested_indices.push_back(index);
      backend_core::BackendCore::CloudPtr cloud(new Cloud);
      pcl::PointXYZI point;
      point.x = static_cast<float>(index);
      cloud->push_back(point);
      return cloud;
    };

  const auto events = application.processSubmaps(submaps(3), provider);

  ASSERT_EQ(events.size(), 2U);
  const std::vector<int> expected {0, 1, 0, 2, 1, 0};
  EXPECT_EQ(requested_indices, expected);
}

TEST(GraphSlamApplication, OwnsTheCanonicalDeduplicatedLoopEdgeSet)
{
  Fixture fixture;
  auto application = fixture.makeApplication();
  GraphSlamApplication::LoopEdge edge;
  edge.pair_id = {9, 2};
  edge.relative_pose.translation().x() = 7.0;
  edge.fitness_score = 0.3;

  ASSERT_TRUE(application->upsertLoopEdge(edge));
  const auto edges = application->stateSnapshot().loop_edges;
  ASSERT_EQ(edges.size(), 1U);
  EXPECT_EQ(edges[0].pair_id, std::make_pair(2, 9));
  EXPECT_DOUBLE_EQ(edges[0].relative_pose.translation().x(), -7.0);
  EXPECT_DOUBLE_EQ(edges[0].fitness_score, 0.3);
}

TEST(GraphSlamApplication, OptimizesPlainPoseGraphRequestsThroughTheSharedEntryPoint)
{
  Fixture fixture;
  auto application = fixture.makeApplication();
  GraphSlamApplication::LoopEdge future_edge;
  future_edge.pair_id = {0, 5};
  future_edge.fitness_score = 0.1;
  ASSERT_TRUE(application->upsertLoopEdge(future_edge));
  PoseGraphRequest request;
  request.submaps.resize(1);
  request.submaps[0].pose.translation().x() = 3.5;

  const auto result = application->optimize(request);

  ASSERT_EQ(result.poses.size(), 1U);
  EXPECT_DOUBLE_EQ(result.poses[0].translation().x(), 3.5);
}

TEST(GraphSlamApplication, OnlineAndOfflineBatchingProduceByteIdenticalArtifacts)
{
  Fixture fixture;
  auto online = fixture.makeApplication();
  auto offline = fixture.makeApplication();
  const auto ordered_submaps = submaps(4);

  // A live callback may receive the complete prefix, while bag replay can
  // submit one ordered submap at a time. Both paths enter the same engine.
  online->processSubmaps(ordered_submaps, emptyCloud);
  for (int count = 1; count <= 4; ++count) {
    offline->processSubmaps(submaps(count), emptyCloud);
  }

  GraphSlamApplication::LoopEdge edge;
  edge.pair_id = {0, 3};
  edge.relative_pose.translation().x() = 30.0;
  edge.fitness_score = 0.125;
  ASSERT_TRUE(online->upsertLoopEdge(edge));
  ASSERT_TRUE(offline->upsertLoopEdge(edge));

  PoseGraphRequest pose_graph_request;
  pose_graph_request.submaps.resize(ordered_submaps.size());
  for (std::size_t i = 0; i < ordered_submaps.size(); ++i) {
    pose_graph_request.submaps[i].pose =
      Eigen::Isometry3d(ordered_submaps[i].pose.matrix());
  }
  const std::vector<double> timestamps {1000.0, 1001.0, 1002.0, 1003.0};
  const auto online_result = online->optimizeAndSerialize(
    pose_graph_request, timestamps);
  const auto offline_result = offline->optimizeAndSerialize(
    pose_graph_request, timestamps);

  EXPECT_EQ(
    online_result.artifacts.loop_edges_csv,
    offline_result.artifacts.loop_edges_csv);
  EXPECT_EQ(
    online_result.artifacts.trajectory_optimized_tum,
    offline_result.artifacts.trajectory_optimized_tum);
  EXPECT_EQ(
    online_result.optimization.pose_graph_g2o,
    offline_result.optimization.pose_graph_g2o);
  EXPECT_EQ(online_result.artifacts.loop_edge_count, 1U);
  EXPECT_FALSE(online_result.artifacts.trajectory_optimized_tum.empty());
}

TEST(GraphSlamApplication, ArtifactRequestRejectsMismatchedTimestamps)
{
  Fixture fixture;
  auto application = fixture.makeApplication();
  ArtifactRequest request;
  request.timestamps.push_back(1.0);
  EXPECT_THROW(
    static_cast<void>(application->deterministicArtifacts(request)),
    std::invalid_argument);
}

TEST(GraphSlamApplication, RejectsInvalidWorkflowConfiguration)
{
  GraphSlamApplicationConfig config;
  config.registration_method = "GICP";
  config.loop_search_query_stride = 0;
  EXPECT_THROW(static_cast<void>(GraphSlamApplication(config)), std::invalid_argument);
}

TEST(GraphSlamApplication, RejectsInvalidOwnedEngineConfiguration)
{
  GraphSlamApplicationConfig config;
  config.registration_method = "not-a-registration";
  EXPECT_THROW(static_cast<void>(GraphSlamApplication(config)), std::invalid_argument);

  config.registration_method = "GICP";
  config.voxel_leaf_size = 0.0;
  EXPECT_THROW(static_cast<void>(GraphSlamApplication(config)), std::invalid_argument);
}

}  // namespace
}  // namespace graphslam
