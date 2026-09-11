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

#ifndef GRAPH_SLAM_COMPOSITION_HPP_
#define GRAPH_SLAM_COMPOSITION_HPP_

#include "graph_slam_config.hpp"

#include "graph_based_slam/backend_core.hpp"
#include "graph_based_slam/adjacent_edge_auto_scale.hpp"
#include "graph_based_slam/dynamic_object_filter.hpp"
#include "graph_based_slam/gnss_weighting.hpp"
#include "graph_based_slam/graph_slam_application.hpp"
#include "graph_based_slam/map_saver.hpp"
#include "graph_based_slam/planar_map_filter.hpp"
#include "graph_based_slam/pose_graph_optimization.hpp"

namespace graphslam
{

backend_core::DescriptorConfig makeDescriptorConfig(const GraphSlamConfig & config);
backend_core::LoopSearchConfig makeLoopSearchConfig(const GraphSlamConfig & config);
GraphSlamApplicationConfig makeGraphSlamApplicationConfig(const GraphSlamConfig & config);

struct PoseGraphConfigBundle
{
  pose_graph::AdjacentEdgeConfig adjacent;
  pose_graph::LoopEdgeConfig loop;
  pose_graph::ImuEdgeConfig imu;
  pose_graph::Chi2Collection chi2_collection {pose_graph::Chi2Collection::NONE};
  detail::AutoScaleConfig auto_scale;
};

PoseGraphConfigBundle makePoseGraphConfig(
  const GraphSlamConfig & config,
  double adjacent_weight,
  double adjacent_weight_trans,
  double adjacent_weight_rot);
DynamicObjectFilterConfig makeDynamicObjectFilterConfig(const GraphSlamConfig & config);
PlanarMapFilterConfig makePlanarMapFilterConfig(const GraphSlamConfig & config);
map_saver::GridConfig makeGridConfig(const GraphSlamConfig & config);
detail::GnssWeightingConfig makeGnssWeightingConfig(const GraphSlamConfig & config);

}  // namespace graphslam

#endif  // GRAPH_SLAM_COMPOSITION_HPP_
