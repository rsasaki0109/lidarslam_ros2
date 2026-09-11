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

#ifndef GRAPH_BASED_SLAM__POSE_GRAPH_OPTIMIZATION_HPP_
#define GRAPH_BASED_SLAM__POSE_GRAPH_OPTIMIZATION_HPP_

// The g2o pose-graph assembly + optimization extracted from
// doPoseAdjustment() as a pure function of plain data (v0.6 Phase 1;
// this becomes the Phase 2 BackendCore optimize step). The construction
// order mirrors the historical code exactly — vertex i, then its adjacent
// edges, then IMU edges, loop edges, GNSS anchors — so optimization
// results match the pre-extraction behaviour bit for bit.

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"
#ifndef GRAPH_BASED_SLAM_WITH_G2O
#define GRAPH_BASED_SLAM_WITH_G2O 1
#endif
#include "graph_based_slam/adjacent_edge_covariance_weighting.hpp"
#include "graph_based_slam/loop_edge_robustifier.hpp"
#include "graph_based_slam/plane_revisit_edge.hpp"

namespace graphslam
{
namespace pose_graph
{

struct SubmapNode
{
  Eigen::Isometry3d pose {Eigen::Isometry3d::Identity()};
  std::array<double, 36> odometry_covariance {};
};

struct LoopConstraint
{
  int from {0};
  int to {0};
  Eigen::Isometry3d relative_pose {Eigen::Isometry3d::Identity()};
  double fitness_score {0.0};
};

// Pre-built relative measurement between submap `from` and `to` whose
// rotation comes from IMU integration (the caller owns the IMU buffer).
struct ImuRotationConstraint
{
  int from {0};
  int to {0};
  Eigen::Isometry3d measurement {Eigen::Isometry3d::Identity()};
};

// Pre-matched GNSS anchor for one submap (the caller owns the GNSS buffer
// and the nearest-measurement matching).
struct GnssConstraint
{
  int submap_index {0};
  Eigen::Vector3d position {Eigen::Vector3d::Zero()};
  Eigen::Vector3d info_diag {Eigen::Vector3d::Zero()};
};

struct PlaneRevisitConstraint
{
  int from {0};
  int to {0};
  PlaneRevisitMeasurement measurement;
  double normal_info_weight {10.0};
  double offset_info_weight {10.0};
  double robust_kernel_delta {1.0};
};

struct AdjacentEdgeConfig
{
  int num_adjacent_pose_constraints {5};
  bool split_trans_rot {false};
  double info_weight {1000.0};
  double info_weight_trans {1000.0};
  double info_weight_rot {1000.0};
  degeneracy::AdjacentEdgeCovarianceWeightingConfig covariance_weighting {};
};

struct LoopEdgeConfig
{
  double info_weight {100.0};
  std::string robust_kernel_type {"NONE"};
  double robust_kernel_delta {1.0};
};

struct ImuEdgeConfig
{
  double info_roll_pitch {100.0};
  double info_yaw {10.0};
};

struct OptimizationResult
{
  std::vector<Eigen::Isometry3d> poses;
  // Canonical g2o artifact bytes. Persistence belongs to an outer output
  // port; the optimizer never opens a path or touches the filesystem.
  std::string pose_graph_g2o;
  // Post-optimization adjacent-edge chi2 contributions, for the caller's
  // NIS auto-scale update (finite values only, in edge construction order).
  std::vector<double> adjacent_chi2;
  std::vector<double> adjacent_trans_chi2;
  std::vector<double> adjacent_rot_chi2;
  int plane_revisit_edges {0};
  double plane_revisit_chi2_before {0.0};
  double plane_revisit_chi2_after {0.0};
};

// What to collect for the caller's NIS auto-scale update; NONE skips the
// post-optimization error recomputation entirely (matching the historical
// cost when auto-scale is off).
enum class Chi2Collection
{
  NONE,
  UNIFIED,
  SPLIT,
};

inline OptimizationResult optimizePoseGraph(
  const std::vector<SubmapNode> & submaps,
  const std::vector<LoopConstraint> & loop_constraints,
  const std::vector<ImuRotationConstraint> & imu_constraints,
  const std::vector<GnssConstraint> & gnss_constraints,
  const AdjacentEdgeConfig & adjacent_cfg,
  const LoopEdgeConfig & loop_cfg,
  const ImuEdgeConfig & imu_cfg,
  Chi2Collection chi2_collection,
  // When GNSS anchors provide the gauge, release vertex 0 so the graph can
  // settle into the anchor (ENU) frame; the historical default pins it.
  bool fix_first_vertex = true,
  int iterations = 10,
  const std::vector<PlaneRevisitConstraint> & plane_constraints = {})
{
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver =
    std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(
    std::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));
  optimizer.setAlgorithm(solver);

  const int submaps_size = static_cast<int>(submaps.size());
  std::vector<g2o::EdgeSE3 *> adjacent_edges;
  for (int i = 0; i < submaps_size; i++) {
    const Eigen::Isometry3d & pose = submaps[i].pose;

    g2o::VertexSE3 * vertex_se3 = new g2o::VertexSE3();
    vertex_se3->setId(i);
    vertex_se3->setEstimate(pose);
    if (i == 0 && fix_first_vertex) {vertex_se3->setFixed(true);}
    optimizer.addVertex(vertex_se3);

    if (i > 0) {
      const int start_idx = std::max(0, i - adjacent_cfg.num_adjacent_pose_constraints);
      for (int pre_idx = start_idx; pre_idx < i; pre_idx++) {
        const Eigen::Isometry3d & pre_pose = submaps[pre_idx].pose;
        Eigen::Isometry3d relative_pose = pre_pose.inverse() * pose;

        const int separation = i - pre_idx;
        const double sep_d = static_cast<double>(separation);
        Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Zero();
        if (adjacent_cfg.split_trans_rot) {
          const double w_trans = adjacent_cfg.info_weight_trans / sep_d;
          const double w_rot = adjacent_cfg.info_weight_rot / sep_d;
          info_mat.topLeftCorner<3, 3>().diagonal().setConstant(w_trans);
          info_mat.bottomRightCorner<3, 3>().diagonal().setConstant(w_rot);
        } else {
          const double edge_weight = adjacent_cfg.info_weight / sep_d;
          info_mat = Eigen::Matrix<double, 6, 6>::Identity() * edge_weight;
        }
        const degeneracy::AdjacentEdgeCovarianceWeightingResult covariance_weighting =
          degeneracy::weightAdjacentEdgeFromCovariance(
          info_mat, submaps[i].odometry_covariance, adjacent_cfg.covariance_weighting);
        info_mat = covariance_weighting.information;
        g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
        edge_se3->setMeasurement(relative_pose);
        edge_se3->setInformation(info_mat);
        edge_se3->vertices()[0] = optimizer.vertex(pre_idx);
        edge_se3->vertices()[1] = optimizer.vertex(i);
        optimizer.addEdge(edge_se3);
        adjacent_edges.push_back(edge_se3);
      }
    }
  }

  /* IMU rotation constraint edges */
  for (const auto & imu_constraint : imu_constraints) {
    g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
    edge_se3->setMeasurement(imu_constraint.measurement);

    Eigen::Matrix<double, 6, 6> imu_info = Eigen::Matrix<double, 6, 6>::Zero();
    // g2o EdgeSE3 error order is (x, y, z, qx, qy, qz) — translation first
    // (toVectorMQT, isometry3d_mappings.h). Rotation weights go on the
    // rotation block (3..5); translation stays zero on purpose (no trust in
    // IMU double integration).
    imu_info(3, 3) = imu_cfg.info_roll_pitch;  // qx ~ roll
    imu_info(4, 4) = imu_cfg.info_roll_pitch;  // qy ~ pitch
    imu_info(5, 5) = imu_cfg.info_yaw;         // qz ~ yaw
    edge_se3->setInformation(imu_info);

    edge_se3->vertices()[0] = optimizer.vertex(imu_constraint.from);
    edge_se3->vertices()[1] = optimizer.vertex(imu_constraint.to);
    optimizer.addEdge(edge_se3);
  }

  /* loop edges */
  const auto loop_kernel_type =
    graphslam::robust::parseLoopEdgeKernelType(loop_cfg.robust_kernel_type);
  for (const auto & loop_constraint : loop_constraints) {
    g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
    edge_se3->setMeasurement(loop_constraint.relative_pose);
    const double fitness = std::max(loop_constraint.fitness_score, 1e-3);
    Eigen::Matrix<double, 6, 6> loop_info_mat =
      Eigen::Matrix<double, 6, 6>::Identity() * (loop_cfg.info_weight / fitness);
    edge_se3->setInformation(loop_info_mat);
    edge_se3->setRobustKernel(
      graphslam::robust::makeLoopEdgeKernel(
        loop_kernel_type, loop_cfg.robust_kernel_delta));
    edge_se3->vertices()[0] = optimizer.vertex(loop_constraint.from);
    edge_se3->vertices()[1] = optimizer.vertex(loop_constraint.to);
    optimizer.addEdge(edge_se3);
  }

  /* Structural plane re-observation edges. */
  std::vector<PlaneRevisitEdge *> plane_edges;
  for (const auto & constraint : plane_constraints) {
    if (constraint.from < 0 || constraint.to < 0 ||
      constraint.from >= submaps_size || constraint.to >= submaps_size ||
      constraint.from == constraint.to)
    {
      continue;
    }
    PlaneRevisitEdge * edge = new PlaneRevisitEdge();
    edge->setMeasurement(constraint.measurement);
    Eigen::Matrix4d information = Eigen::Matrix4d::Zero();
    information.topLeftCorner<3, 3>().diagonal().setConstant(
      std::max(0.0, constraint.normal_info_weight));
    information(3, 3) = std::max(0.0, constraint.offset_info_weight);
    edge->setInformation(information);
    if (constraint.robust_kernel_delta > 0.0) {
      auto * kernel = new g2o::RobustKernelHuber();
      kernel->setDelta(constraint.robust_kernel_delta);
      edge->setRobustKernel(kernel);
    }
    edge->vertices()[0] = optimizer.vertex(constraint.from);
    edge->vertices()[1] = optimizer.vertex(constraint.to);
    optimizer.addEdge(edge);
    edge->computeError();
    plane_edges.push_back(edge);
  }
  double plane_chi2_before = 0.0;
  for (auto * edge : plane_edges) {
    plane_chi2_before += edge->chi2();
  }

  /* GNSS position anchors */
  int gnss_edges_added = 0;
  for (const auto & gnss_constraint : gnss_constraints) {
    const int gnss_vertex_id = submaps_size + gnss_edges_added;
    g2o::VertexSE3 * gnss_vertex = new g2o::VertexSE3();
    gnss_vertex->setId(gnss_vertex_id);
    Eigen::Isometry3d gnss_pose = Eigen::Isometry3d::Identity();
    gnss_pose.translation() = gnss_constraint.position;
    gnss_vertex->setEstimate(gnss_pose);
    gnss_vertex->setFixed(true);
    optimizer.addVertex(gnss_vertex);

    g2o::EdgeSE3 * edge = new g2o::EdgeSE3();
    edge->setMeasurement(Eigen::Isometry3d::Identity());
    Eigen::Matrix<double, 6, 6> gnss_info = Eigen::Matrix<double, 6, 6>::Zero();
    // g2o EdgeSE3 error order is (x, y, z, qx, qy, qz) — translation first —
    // so the position weights go on the translation block (0..2); rotation
    // stays unconstrained (GNSS carries no orientation).
    gnss_info(0, 0) = gnss_constraint.info_diag.x();
    gnss_info(1, 1) = gnss_constraint.info_diag.y();
    gnss_info(2, 2) = gnss_constraint.info_diag.z();
    edge->setInformation(gnss_info);
    edge->vertices()[0] = gnss_vertex;
    edge->vertices()[1] = optimizer.vertex(gnss_constraint.submap_index);
    optimizer.addEdge(edge);
    gnss_edges_added++;
  }

  optimizer.initializeOptimization();
  optimizer.optimize(iterations);

  OptimizationResult result;
  std::ostringstream graph_stream;
  optimizer.save(graph_stream);
  result.pose_graph_g2o = graph_stream.str();
  result.plane_revisit_edges = static_cast<int>(plane_edges.size());
  result.plane_revisit_chi2_before = plane_chi2_before;
  result.poses.reserve(submaps_size);
  for (int i = 0; i < submaps_size; i++) {
    g2o::VertexSE3 * vertex_se3 = static_cast<g2o::VertexSE3 *>(optimizer.vertex(i));
    result.poses.push_back(vertex_se3->estimate());
  }
  for (auto * edge : plane_edges) {
    edge->computeError();
    result.plane_revisit_chi2_after += edge->chi2();
  }

  if (chi2_collection == Chi2Collection::SPLIT) {
    result.adjacent_trans_chi2.reserve(adjacent_edges.size());
    result.adjacent_rot_chi2.reserve(adjacent_edges.size());
    for (auto * e : adjacent_edges) {
      e->computeError();
      const auto err = e->error();
      const Eigen::Matrix<double, 6, 6> info = e->information();
      const double w_t = info(0, 0);
      const double w_r = info(3, 3);
      const double trans = w_t * err.template head<3>().squaredNorm();
      const double rot = w_r * err.template tail<3>().squaredNorm();
      if (std::isfinite(trans)) {
        result.adjacent_trans_chi2.push_back(trans);
      }
      if (std::isfinite(rot)) {
        result.adjacent_rot_chi2.push_back(rot);
      }
    }
  } else if (chi2_collection == Chi2Collection::UNIFIED) {
    result.adjacent_chi2.reserve(adjacent_edges.size());
    for (auto * e : adjacent_edges) {
      e->computeError();
      const double v = e->chi2();
      if (std::isfinite(v)) {
        result.adjacent_chi2.push_back(v);
      }
    }
  }

  return result;
}

}  // namespace pose_graph
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__POSE_GRAPH_OPTIMIZATION_HPP_
