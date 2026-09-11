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

#ifndef GRAPH_BASED_SLAM__LOOP_EDGE_SET_HPP_
#define GRAPH_BASED_SLAM__LOOP_EDGE_SET_HPP_

#include <cstdlib>

#include <algorithm>
#include <utility>
#include <vector>

#include <Eigen/Geometry>  // NOLINT(build/include_order)

namespace graphslam
{
namespace backend_core
{

// Accepted loop edges with the nearby-pair dedup/upsert policy. This
// container is deliberately synchronization-free; GraphSlamApplication owns
// the single canonical instance and protects it with its engine mutex.
class LoopEdgeSet
{
public:
  struct Edge
  {
    std::pair<int, int> pair_id{-1, -1};
    Eigen::Isometry3d relative_pose{Eigen::Isometry3d::Identity()};
    double fitness_score{0.0};
  };

  void configure(int dedup_index_window) {dedup_index_window_ = dedup_index_window;}

  // Reject negative/self pairs, normalize index order (swap + inverse
  // pose), and keep the better fitness for nearby duplicate pairs.
  bool upsert(const Edge & edge)
  {
    if (edge.pair_id.first < 0 || edge.pair_id.second < 0) {
      return false;
    }

    Edge normalized = edge;
    if (normalized.pair_id.first > normalized.pair_id.second) {
      std::swap(normalized.pair_id.first, normalized.pair_id.second);
      normalized.relative_pose = normalized.relative_pose.inverse();
    }
    if (normalized.pair_id.first == normalized.pair_id.second) {
      return false;
    }

    auto is_nearby_pair = [this](const Edge & lhs, const Edge & rhs) {
        return std::abs(lhs.pair_id.first - rhs.pair_id.first) <= dedup_index_window_ &&
               std::abs(lhs.pair_id.second - rhs.pair_id.second) <= dedup_index_window_;
      };
    for (auto & existing : edges_) {
      if (!is_nearby_pair(existing, normalized)) {
        continue;
      }
      if (existing.fitness_score > 0.0 &&
        normalized.fitness_score >= existing.fitness_score)
      {
        return false;
      }
      existing = normalized;
      return true;
    }

    edges_.push_back(normalized);
    return true;
  }

  const std::vector<Edge> & edges() const {return edges_;}

private:
  int dedup_index_window_{8};
  std::vector<Edge> edges_;
};

}  // namespace backend_core
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__LOOP_EDGE_SET_HPP_
