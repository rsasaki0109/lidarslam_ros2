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

#include <filesystem>
#include <fstream>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "filesystem_io_ports.hpp"
#include "graph_based_slam/external_io_ports.hpp"

namespace graphslam
{
namespace ports
{
namespace
{

TEST(ExternalIoPorts, RequiresEveryBoundary)
{
  ExternalIoPorts ports;
  EXPECT_THROW(ports.validate(), std::invalid_argument);
  ports.clock = std::make_shared<ManualClock>();
  ports.submap_storage = std::make_shared<InMemorySubmapStorage>();
  ports.diagnostics = std::make_shared<InMemoryDiagnostics>();
  ports.map_output = std::make_shared<InMemoryMapOutput>();
  EXPECT_NO_THROW(ports.validate());
}

TEST(ExternalIoPorts, ManualClockIsControlledByInput)
{
  ManualClock clock(12.5);
  EXPECT_DOUBLE_EQ(clock.nowSeconds(), 12.5);
  clock.set(42.25);
  EXPECT_DOUBLE_EQ(clock.nowSeconds(), 42.25);
}

TEST(ExternalIoPorts, InMemoryStorageReturnsValueSnapshots)
{
  InMemorySubmapStorage storage;
  PointCloud cloud;
  pcl::PointXYZI point;
  point.x = 1.0F;
  point.intensity = 7.0F;
  cloud.push_back(point);
  ASSERT_TRUE(storage.store(3, cloud));

  PointCloudPtr first = storage.load(3);
  ASSERT_EQ(first->size(), 1U);
  first->front().x = 99.0F;
  PointCloudPtr second = storage.load(3);
  ASSERT_EQ(second->size(), 1U);
  EXPECT_FLOAT_EQ(second->front().x, 1.0F);
  EXPECT_TRUE(storage.load(4)->empty());

  PointCloudPtr second_cloud(new PointCloud(cloud));
  second_cloud->front().x = 2.0F;
  const auto stored = storage.storeBatch({{5, second_cloud}, {-1, second_cloud}});
  ASSERT_EQ(stored.size(), 2U);
  EXPECT_TRUE(stored[0]);
  EXPECT_FALSE(stored[1]);
  EXPECT_FLOAT_EQ(storage.load(5)->front().x, 2.0F);
}

TEST(ExternalIoPorts, InMemoryDiagnosticsPreservesEmissionOrder)
{
  InMemoryDiagnostics diagnostics;
  diagnostics.emit({DiagnosticLevel::INFO, "first", "one"});
  diagnostics.emit({DiagnosticLevel::WARNING, "second", "two"});
  const auto events = diagnostics.events();
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].code, "first");
  EXPECT_EQ(events[1].code, "second");
}

TEST(ExternalIoPorts, InMemoryOutputPreservesExactArtifactBytes)
{
  InMemoryMapOutput output;
  const std::string bytes("g2o\0artifact\n", 13);
  ASSERT_TRUE(output.writeBytes("pose_graph.g2o", bytes));
  const std::string loaded = output.bytes("pose_graph.g2o");
  ASSERT_EQ(loaded.size(), bytes.size());
  EXPECT_EQ(std::memcmp(loaded.data(), bytes.data(), bytes.size()), 0);
  ASSERT_TRUE(output.appendBytes("pose_graph.g2o", "tail"));
  EXPECT_EQ(output.bytes("pose_graph.g2o"), bytes + "tail");
  ASSERT_TRUE(output.writeBytes("map/stale.yaml", "stale"));
  ASSERT_TRUE(output.writeBytes("map/keep.txt", "keep"));
  ASSERT_TRUE(output.removeByExtension("map", {".yaml"}));
  EXPECT_TRUE(output.bytes("map/stale.yaml").empty());
  EXPECT_EQ(output.bytes("map/keep.txt"), "keep");

  PointCloud cloud;
  pcl::PointXYZI point;
  point.z = 4.0F;
  cloud.push_back(point);
  ASSERT_TRUE(output.writePointCloud("map.pcd", cloud));
  ASSERT_EQ(output.pointCloud("map.pcd")->size(), 1U);
  EXPECT_FLOAT_EQ(output.pointCloud("map.pcd")->front().z, 4.0F);
}

TEST(ExternalIoPorts, FilesystemAdaptersRoundTripWithoutLeakingIntoEngine)
{
  const std::filesystem::path directory =
    std::filesystem::temp_directory_path() / "graph_slam_external_io_ports_test";
  std::filesystem::remove_all(directory);

  adapters::FilesystemMapOutput output;
  ASSERT_TRUE(output.prepareDirectory(directory.string()));
  const std::string text_path = (directory / "artifact.txt").string();
  ASSERT_TRUE(output.writeBytes(text_path, "first"));
  ASSERT_TRUE(output.appendBytes(text_path, "-second"));
  std::ifstream text_input(text_path, std::ios::binary);
  const std::string loaded_text(
    (std::istreambuf_iterator<char>(text_input)), std::istreambuf_iterator<char>());
  EXPECT_EQ(loaded_text, "first-second");

  adapters::PcdSubmapStorage storage((directory / "cache").string());
  PointCloud cloud;
  pcl::PointXYZI point;
  point.x = 3.0F;
  point.intensity = 9.0F;
  cloud.push_back(point);
  ASSERT_TRUE(storage.store(7, cloud));
  PointCloudPtr loaded_cloud = storage.load(7);
  ASSERT_EQ(loaded_cloud->size(), 1U);
  EXPECT_FLOAT_EQ(loaded_cloud->front().x, 3.0F);
  EXPECT_TRUE(storage.load(8)->empty());
  const auto batch_result = storage.storeBatch({{8, loaded_cloud}});
  ASSERT_EQ(batch_result.size(), 1U);
  EXPECT_TRUE(batch_result.front());
  EXPECT_EQ(storage.load(8)->size(), 1U);

  ASSERT_TRUE(output.writeBytes((directory / "stale.yaml").string(), "stale"));
  ASSERT_TRUE(output.writeBytes((directory / "keep.txt").string(), "keep"));
  ASSERT_TRUE(output.removeByExtension(directory.string(), {".yaml"}));
  EXPECT_FALSE(std::filesystem::exists(directory / "stale.yaml"));
  EXPECT_TRUE(std::filesystem::exists(directory / "keep.txt"));
  std::filesystem::remove_all(directory);
}

}  // namespace
}  // namespace ports
}  // namespace graphslam
