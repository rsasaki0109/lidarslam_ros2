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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "graph_based_slam/pointcloud_conversion.hpp"

TEST(PointcloudConversion, PreservesIntensityWhenAvailable)
{
  pcl::PointCloud<pcl::PointXYZI> source;
  source.width = 2;
  source.height = 1;
  source.is_dense = true;
  source.points.resize(2);
  source.points[0].x = 1.0F;
  source.points[0].y = 2.0F;
  source.points[0].z = 3.0F;
  source.points[0].intensity = 17.0F;
  source.points[1].x = -1.0F;
  source.points[1].y = -2.0F;
  source.points[1].z = -3.0F;
  source.points[1].intensity = 42.0F;

  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(source, message);
  pcl::PointCloud<pcl::PointXYZI> converted;

  EXPECT_TRUE(
    graphslam::pointcloud_conversion::fromRosMsgWithOptionalIntensity(
      message, converted));
  ASSERT_EQ(converted.points.size(), source.points.size());
  EXPECT_FLOAT_EQ(converted.points[0].intensity, 17.0F);
  EXPECT_FLOAT_EQ(converted.points[1].intensity, 42.0F);
  EXPECT_FLOAT_EQ(converted.points[1].z, -3.0F);
}

TEST(PointcloudConversion, QuietlyDefaultsMissingIntensityToZero)
{
  pcl::PointCloud<pcl::PointXYZ> source;
  source.width = 2;
  source.height = 1;
  source.is_dense = true;
  source.points.resize(2);
  source.points[0].x = 1.0F;
  source.points[0].y = 2.0F;
  source.points[0].z = 3.0F;
  source.points[1].x = -1.0F;
  source.points[1].y = -2.0F;
  source.points[1].z = -3.0F;

  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(source, message);
  ASSERT_FALSE(graphslam::pointcloud_conversion::hasField(message, "intensity"));
  pcl::PointCloud<pcl::PointXYZI> converted;

  EXPECT_FALSE(
    graphslam::pointcloud_conversion::fromRosMsgWithOptionalIntensity(
      message, converted));
  ASSERT_EQ(converted.points.size(), source.points.size());
  EXPECT_EQ(converted.width, source.width);
  EXPECT_EQ(converted.height, source.height);
  EXPECT_EQ(converted.is_dense, source.is_dense);
  EXPECT_FLOAT_EQ(converted.points[0].intensity, 0.0F);
  EXPECT_FLOAT_EQ(converted.points[1].intensity, 0.0F);
  EXPECT_FLOAT_EQ(converted.points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(converted.points[1].z, -3.0F);
}
