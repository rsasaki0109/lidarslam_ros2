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

// Characterization + determinism-contract tests for the pose-graph
// optimization extracted from doPoseAdjustment() (v0.6 Phase 1).

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include "graph_based_slam/pose_graph_optimization.hpp"
#include "graph_based_slam/plane_revisit_constraints.hpp"

namespace graphslam
{
namespace
{

using pose_graph::AdjacentEdgeConfig;
using pose_graph::Chi2Collection;
using pose_graph::GnssConstraint;
using pose_graph::ImuEdgeConfig;
using pose_graph::ImuRotationConstraint;
using pose_graph::LoopConstraint;
using pose_graph::LoopEdgeConfig;
using pose_graph::PlaneRevisitConstraint;
using pose_graph::SubmapNode;
using pose_graph::optimizePoseGraph;

// A straight 11-node chain along +x with 1 m spacing whose later nodes carry
// an injected +y drift, plus the ground-truth loop measurement that says node
// 10 should coincide with node 0.
std::vector<SubmapNode> makeDriftedChain()
{
  std::vector<SubmapNode> submaps;
  for (int i = 0; i <= 10; ++i) {
    SubmapNode node;
    const double x = (i <= 5) ? static_cast<double>(i) : static_cast<double>(10 - i);
    const double drift = 0.06 * std::max(0, i - 5);
    node.pose = Eigen::Isometry3d::Identity();
    node.pose.translation() = Eigen::Vector3d(x, drift, 0.0);
    submaps.push_back(node);
  }
  return submaps;
}

LoopConstraint makeClosingLoop()
{
  // Ground truth: node 10 sits exactly on node 0. A good registration
  // fitness (0.1) gives the loop edge information weight 100 / 0.1 = 1000,
  // on par with the adjacent odometry edges.
  LoopConstraint loop;
  loop.from = 0;
  loop.to = 10;
  loop.relative_pose = Eigen::Isometry3d::Identity();
  loop.fitness_score = 0.1;
  return loop;
}

std::array<double, 36> makeWeakYCovariance()
{
  std::array<double, 36> covariance {};
  for (int i = 0; i < 6; ++i) {
    covariance[static_cast<size_t>(i * 6 + i)] = 1.0e-3;
  }
  covariance[7] = 1.0e3;
  return covariance;
}

TEST(PoseGraphOptimization, LoopClosurePullsDriftedEndpointHome)
{
  const auto submaps = makeDriftedChain();
  const double drift_before =
    (submaps.back().pose.translation() - submaps.front().pose.translation()).norm();
  ASSERT_GT(drift_before, 0.25);

  const auto result = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::NONE);

  ASSERT_EQ(result.poses.size(), submaps.size());
  const double drift_after =
    (result.poses.back().translation() - result.poses.front().translation()).norm();
  EXPECT_LT(drift_after, drift_before * 0.5)
    << "loop closure should pull the drifted endpoint toward node 0";
  // Vertex 0 is fixed.
  EXPECT_LT(
    (result.poses.front().translation() -
    submaps.front().pose.translation()).norm(), 1e-12);
}

TEST(PoseGraphOptimization, NoConstraintsBeyondOdometryKeepsChainShape)
{
  const auto submaps = makeDriftedChain();
  const auto result = optimizePoseGraph(
    submaps, {}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::NONE);

  ASSERT_EQ(result.poses.size(), submaps.size());
  // Adjacent constraints reproduce the input relative poses exactly, so the
  // optimum is the input chain itself.
  for (size_t i = 0; i < submaps.size(); ++i) {
    EXPECT_LT((result.poses[i].translation() - submaps[i].pose.translation()).norm(), 1e-6)
      << "node " << i << " moved without any loop/IMU/GNSS constraint";
  }
}

TEST(PoseGraphOptimization, DegeneracyWeightingLetsLoopCorrectWeakAxis)
{
  auto submaps = makeDriftedChain();
  for (auto & submap : submaps) {
    submap.odometry_covariance = makeWeakYCovariance();
  }

  AdjacentEdgeConfig legacy_config;
  const auto legacy = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {}, legacy_config, LoopEdgeConfig{},
    ImuEdgeConfig{}, Chi2Collection::NONE);

  AdjacentEdgeConfig weighted_config;
  weighted_config.covariance_weighting.enabled = true;
  weighted_config.covariance_weighting.degenerate_information_scale = 0.1;
  const auto weighted = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {}, weighted_config, LoopEdgeConfig{},
    ImuEdgeConfig{}, Chi2Collection::NONE);

  const double legacy_endpoint_y = std::abs(legacy.poses.back().translation().y());
  const double weighted_endpoint_y = std::abs(weighted.poses.back().translation().y());
  EXPECT_LT(weighted_endpoint_y, legacy_endpoint_y)
    << "a competing loop must be able to correct more of a frontend-degenerate direction";
}

TEST(PoseGraphOptimization, SameInputsGiveBitwiseIdenticalPoses)
{
  // The Phase 2 determinism contract at the optimizer layer: identical
  // inputs must give identical outputs, bit for bit.
  const auto submaps = makeDriftedChain();
  const auto a = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::UNIFIED);
  const auto b = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::UNIFIED);

  ASSERT_EQ(a.poses.size(), b.poses.size());
  for (size_t i = 0; i < a.poses.size(); ++i) {
    EXPECT_EQ(
      0,
      std::memcmp(a.poses[i].matrix().data(), b.poses[i].matrix().data(), sizeof(double) * 16))
      << "pose " << i << " differs between identical runs";
  }
  ASSERT_EQ(a.adjacent_chi2.size(), b.adjacent_chi2.size());
  for (size_t i = 0; i < a.adjacent_chi2.size(); ++i) {
    EXPECT_EQ(a.adjacent_chi2[i], b.adjacent_chi2[i]);
  }
  EXPECT_FALSE(a.pose_graph_g2o.empty());
  EXPECT_EQ(a.pose_graph_g2o, b.pose_graph_g2o);
}

TEST(PoseGraphOptimization, GnssAnchorPullsVertexTowardAnchor)
{
  // The intended GNSS semantics, enabled by the block-order fix: position
  // weights sit on the translation block of g2o's (x, y, z, qx, qy, qz)
  // error, so a strong anchor reduces the anchored vertex position error.
  const auto submaps = makeDriftedChain();
  GnssConstraint gnss;
  gnss.submap_index = 10;
  gnss.position = submaps.front().pose.translation();  // anchor at ground truth
  gnss.info_diag = Eigen::Vector3d(1000.0, 1000.0, 1000.0);

  const auto without = optimizePoseGraph(
    submaps, {}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::NONE);
  const auto with = optimizePoseGraph(
    submaps, {}, {}, {gnss},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::NONE);

  const double err_without =
    (without.poses[10].translation() - gnss.position).norm();
  const double err_with = (with.poses[10].translation() - gnss.position).norm();
  EXPECT_LT(err_with, err_without * 0.5)
    << "a strong GNSS anchor must substantially reduce the anchored vertex error";
}

TEST(PoseGraphOptimization, FreeGaugeLetsAnchorsGovernTheGlobalPose)
{
  // A straight 11-node chain along +x, anchored by GNSS constraints that
  // live in a frame translated by (100, 50): with the vertex-0 gauge
  // released the whole graph must settle onto the anchors; with the gauge
  // fixed it cannot.
  std::vector<SubmapNode> submaps;
  for (int i = 0; i <= 10; ++i) {
    SubmapNode node;
    node.pose = Eigen::Isometry3d::Identity();
    node.pose.translation() = Eigen::Vector3d(static_cast<double>(i), 0.0, 0.0);
    submaps.push_back(node);
  }
  std::vector<GnssConstraint> anchors;
  for (int i = 0; i <= 10; i += 2) {
    GnssConstraint g;
    g.submap_index = i;
    g.position = Eigen::Vector3d(static_cast<double>(i) + 100.0, 50.0, 0.0);
    g.info_diag = Eigen::Vector3d(1000.0, 1000.0, 1000.0);
    anchors.push_back(g);
  }

  const auto fixed_gauge = optimizePoseGraph(
    submaps, {}, {}, anchors,
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::NONE,
    /*fix_first_vertex=*/ true);
  const auto free_gauge = optimizePoseGraph(
    submaps, {}, {}, anchors,
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::NONE,
    /*fix_first_vertex=*/ false, /*iterations=*/ 50);

  const Eigen::Vector3d anchor0(100.0, 50.0, 0.0);
  const double err_fixed = (fixed_gauge.poses[0].translation() - anchor0).norm();
  const double err_free = (free_gauge.poses[0].translation() - anchor0).norm();
  EXPECT_GT(err_fixed, 50.0) << "the pinned vertex cannot reach the anchor frame";
  EXPECT_LT(err_free, 1.0) << "with the gauge released the graph must settle on the anchors";
  // Chain shape is preserved while moving.
  const double length_free =
    (free_gauge.poses[10].translation() - free_gauge.poses[0].translation()).norm();
  EXPECT_NEAR(length_free, 10.0, 0.5);
}

TEST(PoseGraphOptimization, Chi2CollectionModesPopulateExpectedVectors)
{
  const auto submaps = makeDriftedChain();
  const auto none = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::NONE);
  EXPECT_TRUE(none.adjacent_chi2.empty());
  EXPECT_TRUE(none.adjacent_trans_chi2.empty());

  const auto unified = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {},
    AdjacentEdgeConfig{}, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::UNIFIED);
  EXPECT_FALSE(unified.adjacent_chi2.empty());
  EXPECT_TRUE(unified.adjacent_trans_chi2.empty());

  AdjacentEdgeConfig split_cfg;
  split_cfg.split_trans_rot = true;
  const auto split = optimizePoseGraph(
    submaps, {makeClosingLoop()}, {}, {},
    split_cfg, LoopEdgeConfig{}, ImuEdgeConfig{}, Chi2Collection::SPLIT);
  EXPECT_TRUE(split.adjacent_chi2.empty());
  EXPECT_FALSE(split.adjacent_trans_chi2.empty());
  EXPECT_EQ(split.adjacent_trans_chi2.size(), split.adjacent_rot_chi2.size());
}

TEST(PoseGraphOptimization, ImuRotationConstraintInfluencesOrientation)
{
  // The intended IMU semantics, enabled by the block-order fix: rotation
  // weights sit on the rotation block of g2o's (x, y, z, qx, qy, qz) error,
  // so a dominant IMU yaw measurement rotates the relative orientation.
  std::vector<SubmapNode> submaps(2);
  submaps[0].pose = Eigen::Isometry3d::Identity();
  submaps[1].pose = Eigen::Isometry3d::Identity();
  submaps[1].pose.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  ImuRotationConstraint imu;
  imu.from = 0;
  imu.to = 1;
  imu.measurement = Eigen::Isometry3d::Identity();
  imu.measurement.linear() =
    Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  imu.measurement.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  ImuEdgeConfig imu_cfg;
  imu_cfg.info_roll_pitch = 1e6;
  imu_cfg.info_yaw = 1e6;

  AdjacentEdgeConfig weak_odom;
  weak_odom.info_weight = 1.0;

  const auto result = optimizePoseGraph(
    submaps, {}, {imu}, {}, weak_odom, LoopEdgeConfig{}, imu_cfg, Chi2Collection::NONE);

  const Eigen::AngleAxisd delta(
    result.poses[0].linear().transpose() * result.poses[1].linear());
  EXPECT_GT(delta.angle() * 180.0 / M_PI, 5.0)
    << "a dominant IMU yaw constraint must rotate the relative orientation";
}

TEST(PoseGraphOptimization, PlaneRevisitCorrectsNormalDirectionDriftOnly)
{
  std::vector<SubmapNode> submaps(2);
  submaps[0].pose = Eigen::Isometry3d::Identity();
  submaps[1].pose = Eigen::Isometry3d::Identity();
  submaps[1].pose.translation() = Eigen::Vector3d(1.30, 0.70, -0.20);

  // Both keyframes observe world plane x=0. In keyframe 0 it is x=0;
  // keyframe 1's true origin is x=1, hence local equation x+1=0.
  PlaneRevisitConstraint plane;
  plane.from = 0;
  plane.to = 1;
  plane.measurement.from.normal = Eigen::Vector3d::UnitX();
  plane.measurement.from.offset = 0.0;
  plane.measurement.from.support_points = 200;
  plane.measurement.to.normal = Eigen::Vector3d::UnitX();
  plane.measurement.to.offset = 1.0;
  plane.measurement.to.support_points = 180;
  plane.normal_info_weight = 1000.0;
  plane.offset_info_weight = 1000.0;

  AdjacentEdgeConfig no_odometry;
  no_odometry.num_adjacent_pose_constraints = 0;
  const auto result = optimizePoseGraph(
    submaps, {}, {}, {}, no_odometry, LoopEdgeConfig{}, ImuEdgeConfig{},
    Chi2Collection::NONE, true, 20, {plane});

  ASSERT_EQ(result.plane_revisit_edges, 1);
  EXPECT_GT(result.plane_revisit_chi2_before, 1.0);
  EXPECT_LT(result.plane_revisit_chi2_after, result.plane_revisit_chi2_before * 1e-4);
  EXPECT_NEAR(result.poses[1].translation().x(), 1.0, 1e-4);
  // A single x-normal plane must not invent constraints tangent to itself.
  EXPECT_NEAR(result.poses[1].translation().y(), 0.70, 1e-6);
  EXPECT_NEAR(result.poses[1].translation().z(), -0.20, 1e-6);
}

TEST(PoseGraphOptimization, PlaneRevisitAlignsNormalsWithoutConstrainingNormalYaw)
{
  std::vector<SubmapNode> submaps(2);
  submaps[0].pose = Eigen::Isometry3d::Identity();
  submaps[1].pose = Eigen::Isometry3d::Identity();
  submaps[1].pose.linear() =
    Eigen::AngleAxisd(8.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  PlaneRevisitConstraint plane;
  plane.from = 0;
  plane.to = 1;
  plane.measurement.from.normal = Eigen::Vector3d::UnitX();
  plane.measurement.to.normal = Eigen::Vector3d::UnitX();
  plane.normal_info_weight = 1000.0;
  plane.offset_info_weight = 1000.0;

  AdjacentEdgeConfig no_odometry;
  no_odometry.num_adjacent_pose_constraints = 0;
  const auto result = optimizePoseGraph(
    submaps, {}, {}, {}, no_odometry, LoopEdgeConfig{}, ImuEdgeConfig{},
    Chi2Collection::NONE, true, 20, {plane});
  const Eigen::Vector3d corrected_normal =
    result.poses[1].rotation() * Eigen::Vector3d::UnitX();
  EXPECT_GT(corrected_normal.dot(Eigen::Vector3d::UnitX()), 0.999999);
  EXPECT_LT(result.plane_revisit_chi2_after, result.plane_revisit_chi2_before * 1e-4);
}

TEST(PoseGraphOptimization, BuildsSparseFactorsFromAssociatedLocalPlaneClusters)
{
  map_refinement::PlaneFeature feature;
  for (const int pose_index : {0, 3, 8, 12}) {
    map_refinement::PlaneFeatureObservation observation;
    observation.pose_index = pose_index;
    for (int y = 0; y < 8; ++y) {
      for (int z = 0; z < 6; ++z) {
        observation.local_cluster.add(
          Eigen::Vector3d(-static_cast<double>(pose_index), 0.2 * y, 0.2 * z));
      }
    }
    feature.observations.push_back(observation);
  }
  pose_graph::PlaneRevisitBuilderConfig config;
  config.min_pose_separation = 5;
  config.max_constraints_per_feature = 2;
  const auto result = pose_graph::buildPlaneRevisitConstraints({feature}, config);

  ASSERT_EQ(result.constraints.size(), 2U);
  EXPECT_EQ(result.constraints[0].from, 0);
  EXPECT_EQ(result.constraints[0].to, 8);
  EXPECT_EQ(result.constraints[1].to, 12);
  EXPECT_EQ(result.constraints[0].measurement.from.support_points, 48);
  EXPECT_NEAR(result.constraints[0].measurement.to.offset, 8.0, 1e-12);
  EXPECT_EQ(result.observations_rejected, 0);
  EXPECT_EQ(result.features_with_constraints, 1);
  EXPECT_EQ(result.max_pose_separation, 12);
}

TEST(PoseGraphOptimization, RejectsNoisyOrWeakPlaneObservations)
{
  map_refinement::PlaneFeature feature;
  map_refinement::PlaneFeatureObservation weak;
  weak.pose_index = 0;
  weak.local_cluster.add(Eigen::Vector3d::Zero());
  feature.observations.push_back(weak);
  map_refinement::PlaneFeatureObservation noisy;
  noisy.pose_index = 10;
  for (int i = 0; i < 100; ++i) {
    noisy.local_cluster.add(Eigen::Vector3d(
      static_cast<double>(i % 5), static_cast<double>((i / 5) % 5),
      static_cast<double>(i / 25)));
  }
  feature.observations.push_back(noisy);
  const auto result = pose_graph::buildPlaneRevisitConstraints(
    {feature}, pose_graph::PlaneRevisitBuilderConfig{});
  EXPECT_TRUE(result.constraints.empty());
  EXPECT_EQ(result.observations_rejected, 2);
  EXPECT_EQ(result.features_with_constraints, 0);
  EXPECT_EQ(result.max_pose_separation, 0);
}

TEST(PoseGraphOptimization, InitialResidualGateRejectsDifferentParallelWall)
{
  std::vector<Eigen::Isometry3d> poses(2, Eigen::Isometry3d::Identity());
  poses[1].translation() = Eigen::Vector3d::UnitX();

  PlaneRevisitConstraint consistent;
  consistent.from = 0;
  consistent.to = 1;
  consistent.measurement.from.normal = Eigen::Vector3d::UnitX();
  consistent.measurement.to.normal = Eigen::Vector3d::UnitX();
  consistent.measurement.from.offset = 0.0;
  consistent.measurement.to.offset = 1.0;

  PlaneRevisitConstraint different_wall = consistent;
  different_wall.measurement.to.offset = 0.0;
  const auto result = pose_graph::gatePlaneRevisitConstraintsByInitialResidual(
    {consistent, different_wall}, poses, 5.0, 0.20);

  ASSERT_EQ(result.constraints.size(), 1U);
  EXPECT_EQ(result.rejected, 1);
  EXPECT_NEAR(result.accepted_max_normal_error_deg, 0.0, 1e-12);
  EXPECT_NEAR(result.accepted_max_offset_error_m, 0.0, 1e-12);
}

TEST(PoseGraphOptimization, OrthogonalPlaneRevisitsReduceTrajectoryRmse)
{
  std::vector<SubmapNode> estimated(6);
  std::vector<Eigen::Vector3d> truth;
  std::vector<PlaneRevisitConstraint> constraints;
  double squared_error_before = 0.0;
  for (int i = 0; i < 6; ++i) {
    const Eigen::Vector3d true_position(i, 0.5 * i, 0.2 * i);
    truth.push_back(true_position);
    estimated[i].pose = Eigen::Isometry3d::Identity();
    estimated[i].pose.translation() =
      true_position + Eigen::Vector3d(0.05 * i, -0.03 * i, 0.02 * i);
    estimated[i].pose.linear() = Eigen::AngleAxisd(
      i * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    squared_error_before +=
      (estimated[i].pose.translation() - true_position).squaredNorm();
    if (i == 0) {
      continue;
    }
    for (int axis = 0; axis < 3; ++axis) {
      PlaneRevisitConstraint constraint;
      constraint.from = 0;
      constraint.to = i;
      constraint.measurement.from.normal = Eigen::Vector3d::Unit(axis);
      constraint.measurement.to.normal = Eigen::Vector3d::Unit(axis);
      constraint.measurement.to.offset = true_position(axis);
      constraint.normal_info_weight = 1000.0;
      constraint.offset_info_weight = 1000.0;
      constraints.push_back(constraint);
    }
  }
  const double rmse_before = std::sqrt(squared_error_before / estimated.size());
  AdjacentEdgeConfig weak_odometry;
  weak_odometry.num_adjacent_pose_constraints = 1;
  weak_odometry.info_weight = 1.0;
  const auto result = optimizePoseGraph(
    estimated, {}, {}, {}, weak_odometry, LoopEdgeConfig{}, ImuEdgeConfig{},
    Chi2Collection::NONE, true, 50, constraints);

  double squared_error_after = 0.0;
  for (std::size_t i = 0; i < result.poses.size(); ++i) {
    squared_error_after +=
      (result.poses[i].translation() - truth[i]).squaredNorm();
  }
  const double rmse_after = std::sqrt(squared_error_after / result.poses.size());
  EXPECT_GT(rmse_before, 0.15);
  EXPECT_LT(rmse_after, 0.002);
  EXPECT_LT(rmse_after, rmse_before * 0.02);
  EXPECT_EQ(result.plane_revisit_edges, 15);
  EXPECT_LT(result.plane_revisit_chi2_after, result.plane_revisit_chi2_before * 1e-3);
}

}  // namespace
}  // namespace graphslam
