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

#ifndef GRAPH_BASED_SLAM__PLANAR_MAP_FILTER_HPP_
#define GRAPH_BASED_SLAM__PLANAR_MAP_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>

namespace graphslam
{

struct PlanarMapFilterConfig
{
  double voxel_size {0.1};
  int min_neighbors {3};
  double max_small_eigenvalue_ratio {0.24};
  double min_middle_eigenvalue_ratio {0.0};
  // If fewer than this fraction of input points have planar support, return
  // the original cloud. This prevents a plane-oriented quality refinement
  // from silently deleting most of a geometrically diverse map.
  double min_retained_ratio {0.90};
};

struct PlanarMapFilterStats
{
  std::size_t input_points {0};
  std::size_t finite_points {0};
  std::size_t voxel_count {0};
  std::size_t planar_voxels {0};
  std::size_t supported_points {0};
  std::size_t output_points {0};
  bool fallback_to_input {false};
};

struct PlanarMapFilterResult
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZI>()};
  PlanarMapFilterStats stats;
};

namespace planar_map_filter_detail
{

struct VoxelKey
{
  int x {0};
  int y {0};
  int z {0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const std::uint64_t hx = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.x));
    const std::uint64_t hy = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.y));
    const std::uint64_t hz = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.z));
    std::uint64_t seed = hx * 0x9E3779B185EBCA87ULL;
    seed ^= hy + 0x9E3779B97F4A7C15ULL + (seed << 6U) + (seed >> 2U);
    seed ^= hz + 0xC2B2AE3D27D4EB4FULL + (seed << 6U) + (seed >> 2U);
    return static_cast<std::size_t>(seed);
  }
};

inline VoxelKey voxelKey(const pcl::PointXYZI & point, double voxel_size)
{
  return VoxelKey{
    static_cast<int>(std::floor(point.x / voxel_size)),
    static_cast<int>(std::floor(point.y / voxel_size)),
    static_cast<int>(std::floor(point.z / voxel_size))};
}

inline bool isFinite(const pcl::PointXYZI & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

struct Moments
{
  std::size_t count {0};
  Eigen::Vector3d sum {Eigen::Vector3d::Zero()};
  Eigen::Matrix3d outer_sum {Eigen::Matrix3d::Zero()};

  void add(const pcl::PointXYZI & point)
  {
    const Eigen::Vector3d value(point.x, point.y, point.z);
    ++count;
    sum += value;
    outer_sum.noalias() += value * value.transpose();
  }

  void add(const Moments & other)
  {
    count += other.count;
    sum += other.sum;
    outer_sum += other.outer_sum;
  }
};

}  // namespace planar_map_filter_detail

inline PlanarMapFilterResult buildPlanarMapFilteredMap(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & input,
  const PlanarMapFilterConfig & config)
{
  using planar_map_filter_detail::Moments;
  using planar_map_filter_detail::VoxelKey;
  using planar_map_filter_detail::VoxelKeyHash;

  PlanarMapFilterResult result;
  if (!input) {
    return result;
  }
  result.stats.input_points = input->size();
  if (input->empty() || config.voxel_size <= 0.0 || config.min_neighbors < 3) {
    *result.cloud = *input;
    result.stats.output_points = input->size();
    result.stats.fallback_to_input = true;
    return result;
  }

  std::unordered_map<VoxelKey, Moments, VoxelKeyHash> voxels;
  voxels.reserve(input->size() / 4U + 1U);
  for (const auto & point : input->points) {
    if (!planar_map_filter_detail::isFinite(point)) {
      continue;
    }
    ++result.stats.finite_points;
    voxels[planar_map_filter_detail::voxelKey(point, config.voxel_size)].add(point);
  }
  result.stats.voxel_count = voxels.size();

  std::unordered_map<VoxelKey, bool, VoxelKeyHash> planar_support;
  planar_support.reserve(voxels.size());
  for (const auto & entry : voxels) {
    Moments neighborhood;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const VoxelKey neighbor{entry.first.x + dx, entry.first.y + dy, entry.first.z + dz};
          const auto found = voxels.find(neighbor);
          if (found != voxels.end()) {
            neighborhood.add(found->second);
          }
        }
      }
    }

    bool supported = false;
    if (neighborhood.count >= static_cast<std::size_t>(config.min_neighbors)) {
      const double count = static_cast<double>(neighborhood.count);
      const Eigen::Vector3d mean = neighborhood.sum / count;
      Eigen::Matrix3d covariance = neighborhood.outer_sum / count - mean * mean.transpose();
      covariance = 0.5 * (covariance + covariance.transpose());
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
      if (solver.info() == Eigen::Success) {
        const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
        const double sum = eigenvalues.sum();
        if (sum > std::numeric_limits<double>::epsilon()) {
          const double small_ratio = eigenvalues.x() / sum;
          const double middle_ratio = eigenvalues.y() / sum;
          supported =
            small_ratio <= config.max_small_eigenvalue_ratio &&
            middle_ratio >= config.min_middle_eigenvalue_ratio;
        }
      }
    }
    planar_support.emplace(entry.first, supported);
    if (supported) {
      ++result.stats.planar_voxels;
      result.stats.supported_points += entry.second.count;
    }
  }

  const double retained_ratio = input->empty() ? 0.0 :
    static_cast<double>(result.stats.supported_points) / static_cast<double>(input->size());
  const double safe_min_retained_ratio =
    std::max(0.0, std::min(1.0, config.min_retained_ratio));
  if (retained_ratio < safe_min_retained_ratio) {
    *result.cloud = *input;
    result.stats.output_points = input->size();
    result.stats.fallback_to_input = true;
    return result;
  }

  result.cloud->reserve(result.stats.supported_points);
  for (const auto & point : input->points) {
    if (!planar_map_filter_detail::isFinite(point)) {
      continue;
    }
    const auto found = planar_support.find(
      planar_map_filter_detail::voxelKey(point, config.voxel_size));
    if (found != planar_support.end() && found->second) {
      result.cloud->push_back(point);
    }
  }
  result.stats.output_points = result.cloud->size();
  return result;
}

}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__PLANAR_MAP_FILTER_HPP_
