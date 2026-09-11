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

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace graphslam
{
namespace pointcloud_conversion
{

inline bool hasField(
  const sensor_msgs::msg::PointCloud2 & message,
  const std::string & name)
{
  return std::any_of(
    message.fields.cbegin(), message.fields.cend(),
    [&name](const sensor_msgs::msg::PointField & field) {
      return field.name == name;
    });
}

// PCL prints "Failed to find match for field 'intensity'" every time an XYZ-only
// PointCloud2 is converted directly to PointXYZI. RKO-LIO's deskewed geometry
// stream intentionally contains XYZ only, so use a quiet, explicit zero default.
// The return value tells callers whether real intensity values were preserved.
inline bool fromRosMsgWithOptionalIntensity(
  const sensor_msgs::msg::PointCloud2 & message,
  pcl::PointCloud<pcl::PointXYZI> & output)
{
  if (hasField(message, "intensity")) {
    pcl::fromROSMsg(message, output);
    return true;
  }

  pcl::PointCloud<pcl::PointXYZ> xyz;
  pcl::fromROSMsg(message, xyz);

  output.header = xyz.header;
  output.width = xyz.width;
  output.height = xyz.height;
  output.is_dense = xyz.is_dense;
  output.sensor_origin_ = xyz.sensor_origin_;
  output.sensor_orientation_ = xyz.sensor_orientation_;
  output.points.resize(xyz.points.size());
  for (std::size_t index = 0; index < xyz.points.size(); ++index) {
    output.points[index].x = xyz.points[index].x;
    output.points[index].y = xyz.points[index].y;
    output.points[index].z = xyz.points[index].z;
    output.points[index].intensity = 0.0F;
  }
  return false;
}

}  // namespace pointcloud_conversion
}  // namespace graphslam
