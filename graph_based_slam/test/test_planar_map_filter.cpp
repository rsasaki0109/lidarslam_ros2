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

#include <gtest/gtest.h>

#include <memory>

#include "graph_based_slam/planar_map_filter.hpp"

namespace
{

void addPoint(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
  double x, double y, double z)
{
  pcl::PointXYZI point;
  point.x = static_cast<float>(x);
  point.y = static_cast<float>(y);
  point.z = static_cast<float>(z);
  point.intensity = 1.0F;
  cloud->push_back(point);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr makePlaneAndVolumeCloud()
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int ix = 0; ix < 4; ++ix) {
    for (int iy = 0; iy < 4; ++iy) {
      addPoint(cloud, 0.1 + 0.2 * ix, 0.1 + 0.2 * iy, 0.0);
    }
  }
  for (int ix = 0; ix < 2; ++ix) {
    for (int iy = 0; iy < 2; ++iy) {
      for (int iz = 0; iz < 2; ++iz) {
        addPoint(cloud, 5.1 + 0.6 * ix, 0.1 + 0.6 * iy, 0.1 + 0.6 * iz);
      }
    }
  }
  return cloud;
}

}  // namespace

TEST(PlanarMapFilter, RetainsPlanarSupportAndRemovesVolumetricCluster)
{
  const auto cloud = makePlaneAndVolumeCloud();
  graphslam::PlanarMapFilterConfig config;
  config.voxel_size = 1.0;
  config.min_neighbors = 8;
  config.max_small_eigenvalue_ratio = 0.03;
  config.min_middle_eigenvalue_ratio = 0.05;
  config.min_retained_ratio = 0.5;

  const auto result = graphslam::buildPlanarMapFilteredMap(cloud, config);

  EXPECT_FALSE(result.stats.fallback_to_input);
  EXPECT_EQ(result.stats.input_points, 24u);
  EXPECT_EQ(result.stats.supported_points, 16u);
  EXPECT_EQ(result.stats.output_points, 16u);
  for (const auto & point : result.cloud->points) {
    EXPECT_LT(point.x, 2.0F);
  }
}

TEST(PlanarMapFilter, FallsBackWhenPlanarSupportWouldDeleteTooMuch)
{
  const auto cloud = makePlaneAndVolumeCloud();
  graphslam::PlanarMapFilterConfig config;
  config.voxel_size = 1.0;
  config.min_neighbors = 8;
  config.min_retained_ratio = 0.75;

  const auto result = graphslam::buildPlanarMapFilteredMap(cloud, config);

  EXPECT_TRUE(result.stats.fallback_to_input);
  EXPECT_EQ(result.stats.supported_points, 16u);
  EXPECT_EQ(result.stats.output_points, cloud->size());
}

TEST(PlanarMapFilter, InvalidConfigurationPreservesInput)
{
  const auto cloud = makePlaneAndVolumeCloud();
  graphslam::PlanarMapFilterConfig config;
  config.voxel_size = 0.0;

  const auto result = graphslam::buildPlanarMapFilteredMap(cloud, config);

  EXPECT_TRUE(result.stats.fallback_to_input);
  EXPECT_EQ(result.stats.output_points, cloud->size());
}
