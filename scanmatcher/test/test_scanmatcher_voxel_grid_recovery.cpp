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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lidarslam_msgs/msg/map_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "scanmatcher/scanmatcher_component.h"

namespace
{

using namespace std::chrono_literals;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

constexpr char kCloudTopic[] = "/scanmatcher_voxel_grid_recovery/input_cloud";
constexpr char kMapTopic[] = "/scanmatcher_voxel_grid_recovery/map";
constexpr char kMapArrayTopic[] = "/scanmatcher_voxel_grid_recovery/map_array";
constexpr char kPoseTopic[] = "/scanmatcher_voxel_grid_recovery/current_pose";
constexpr std::int32_t kMalformedStampSec = 5;
constexpr std::int32_t kUnsafeStampSec = 10;
constexpr std::int32_t kSafeStampSec = 20;
constexpr std::int32_t kInitialAsyncStampSec = 30;
constexpr std::int32_t kUnsafeAsyncStampSec = 40;
constexpr std::int32_t kRecoveredAsyncStampSec = 50;

Point makePoint(float x, float y, float z, float intensity = 0.0F)
{
  Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

sensor_msgs::msg::PointCloud2 makeCloudMessage(
  const Cloud & cloud,
  std::int32_t stamp_sec)
{
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = "base_link";
  message.header.stamp.sec = stamp_sec;
  message.header.stamp.nanosec = 0;
  return message;
}

sensor_msgs::msg::PointCloud2 makeXyzOnlyCloudMessage(
  const Cloud & cloud,
  std::int32_t stamp_sec)
{
  pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
  xyz_cloud.is_dense = cloud.is_dense;
  xyz_cloud.reserve(cloud.size());
  for (const auto & source : cloud) {
    pcl::PointXYZ point;
    point.x = source.x;
    point.y = source.y;
    point.z = source.z;
    xyz_cloud.push_back(point);
  }
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(xyz_cloud, message);
  message.header.frame_id = "base_link";
  message.header.stamp.sec = stamp_sec;
  message.header.stamp.nanosec = 0;
  return message;
}

sensor_msgs::msg::PointCloud2 makePaddedOrganizedXyzOnlyCloudMessage(
  const Cloud & cloud,
  std::int32_t stamp_sec)
{
  sensor_msgs::msg::PointCloud2 message =
    makeXyzOnlyCloudMessage(cloud, stamp_sec);
  constexpr uint32_t kRows = 5U;
  constexpr uint32_t kPaddingBytes = 8U;
  EXPECT_EQ(0U, message.width % kRows);
  const uint32_t points_per_row = message.width / kRows;
  const uint32_t packed_row_step = points_per_row * message.point_step;
  const std::vector<uint8_t> packed_data = std::move(message.data);
  message.width = points_per_row;
  message.height = kRows;
  message.row_step = packed_row_step + kPaddingBytes;
  message.data.assign(message.row_step * message.height, 0xA5U);
  for (uint32_t row = 0U; row < message.height; ++row) {
    const auto source = packed_data.cbegin() + row * packed_row_step;
    const auto destination = message.data.begin() + row * message.row_step;
    std::copy_n(source, packed_row_step, destination);
  }
  return message;
}

Cloud makeIssue69Cloud()
{
  Cloud cloud;
  cloud.is_dense = true;
  cloud.push_back(makePoint(-200.0F, -200.0F, -10.0F));
  cloud.push_back(makePoint(200.0F, 200.0F, 10.0F));
  return cloud;
}

Cloud makeSafeRegistrationCloud()
{
  Cloud cloud;
  cloud.is_dense = true;
  for (int x = -3; x <= 3; ++x) {
    for (int y = -3; y <= 3; ++y) {
      for (int z = -2; z <= 2; ++z) {
        cloud.push_back(
          makePoint(
            static_cast<float>(x) * 0.2F,
            static_cast<float>(y) * 0.2F,
            static_cast<float>(z) * 0.2F,
            static_cast<float>((x + 3) * 100 + (y + 3) * 10 + z + 2)));
      }
    }
  }
  return cloud;
}

Cloud makeIssue69MapUpdateCloud()
{
  Cloud cloud = makeSafeRegistrationCloud();
  for (auto & point : cloud) {
    point.x += 1.0F;
    point.y += 0.4F;
  }
  cloud.push_back(makePoint(-200.0F, -200.0F, -10.0F));
  cloud.push_back(makePoint(200.0F, 200.0F, 10.0F));
  return cloud;
}

Cloud makeRecoveredMapUpdateCloud()
{
  Cloud cloud = makeSafeRegistrationCloud();
  for (auto & point : cloud) {
    point.x += 1.0F;
    point.y += 0.4F;
  }
  return cloud;
}

template<typename Predicate>
bool spinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  Predicate predicate,
  std::chrono::steady_clock::duration timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  do {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  } while (std::chrono::steady_clock::now() < deadline);
  executor.spin_some();
  return predicate();
}

void spinFor(
  rclcpp::executors::SingleThreadedExecutor & executor,
  std::chrono::steady_clock::duration duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  executor.spin_some();
}

class ScanMatcherVoxelGridRecoveryTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST(MapUpdateCommitState, FailedUpdatePreservesDistanceThresholdForRetry)
{
  graphslam::detail::MapUpdateCommitState state;
  const Eigen::Vector3d origin(1.0, 2.0, 3.0);
  const Eigen::Vector3d candidate(4.0, 6.0, 3.0);
  state.reset(origin);

  EXPECT_DOUBLE_EQ(5.0, state.distanceTo(candidate));
  state.begin(candidate);
  EXPECT_TRUE(state.pending());
  EXPECT_TRUE(state.complete(false));

  EXPECT_FALSE(state.pending());
  EXPECT_TRUE(state.committedPosition().isApprox(origin));
  EXPECT_DOUBLE_EQ(5.0, state.distanceTo(candidate));

  state.begin(candidate);
  EXPECT_TRUE(state.complete(true));
  EXPECT_TRUE(state.committedPosition().isApprox(candidate));
  EXPECT_DOUBLE_EQ(0.0, state.distanceTo(candidate));
  EXPECT_FALSE(state.complete(true));
}

TEST_F(
  ScanMatcherVoxelGridRecoveryTest,
  RejectsMalformedAndUnsafeCloudsThenProcessesSafeCloud)
{
  rclcpp::NodeOptions probe_options;
  probe_options.use_intra_process_comms(true);
  auto probe = std::make_shared<rclcpp::Node>(
    "scanmatcher_voxel_grid_recovery_probe", probe_options);

  std::size_t malformed_map_messages = 0;
  std::size_t unsafe_map_messages = 0;
  std::size_t safe_map_messages = 0;
  std::size_t safe_pose_messages = 0;
  auto map_subscription = probe->create_subscription<sensor_msgs::msg::PointCloud2>(
    kMapTopic,
    rclcpp::QoS(10),
    [&malformed_map_messages, &unsafe_map_messages, &safe_map_messages](
      sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
    {
      if (message->header.stamp.sec == kMalformedStampSec) {
        ++malformed_map_messages;
      } else if (message->header.stamp.sec == kUnsafeStampSec) {
        ++unsafe_map_messages;
      } else if (message->header.stamp.sec == kSafeStampSec) {
        ++safe_map_messages;
      }
    });
  auto pose_subscription = probe->create_subscription<geometry_msgs::msg::PoseStamped>(
    kPoseTopic,
    rclcpp::QoS(10),
    [&safe_pose_messages](geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
    {
      if (message->header.stamp.sec == kSafeStampSec) {
        ++safe_pose_messages;
      }
    });
  auto cloud_publisher = probe->create_publisher<sensor_msgs::msg::PointCloud2>(
    kCloudTopic,
    rclcpp::SensorDataQoS().keep_last(10));

  rclcpp::NodeOptions component_options;
  component_options.use_intra_process_comms(true);
  component_options.arguments({
      "--ros-args",
      "-r", std::string("input_cloud:=") + kCloudTopic,
      "-r", std::string("map:=") + kMapTopic,
      "-r", std::string("current_pose:=") + kPoseTopic,
    });
  component_options.parameter_overrides({
      rclcpp::Parameter("set_initial_pose", true),
      rclcpp::Parameter("publish_tf", false),
      rclcpp::Parameter("registration_method", "NDT"),
      rclcpp::Parameter("ndt_resolution", 0.5),
      rclcpp::Parameter("ndt_num_threads", 1),
      rclcpp::Parameter("ndt_max_iterations", 10),
      rclcpp::Parameter("vg_size_for_input", 0.1),
      rclcpp::Parameter("vg_size_for_map", 0.1),
      rclcpp::Parameter("min_points_for_scan", 20),
      rclcpp::Parameter("async_map_update", false),
      rclcpp::Parameter("trans_for_mapupdate", 1000.0),
      rclcpp::Parameter("motion_gate_enable", false),
      rclcpp::Parameter("reject_nonconverged_pose_update", false),
    });
  auto component = std::make_shared<graphslam::ScanMatcherComponent>(component_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(probe);
  executor.add_node(component);

  ASSERT_TRUE(
    spinUntil(
      executor,
      [&cloud_publisher, &probe]() {
        return cloud_publisher->get_subscription_count() == 1U &&
               probe->count_publishers(kMapTopic) == 1U &&
               probe->count_publishers(kPoseTopic) == 1U;
      },
      5s));

  auto malformed_message = makeCloudMessage(
    makeSafeRegistrationCloud(), kMalformedStampSec);
  malformed_message.data.clear();
  testing::internal::CaptureStderr();
  cloud_publisher->publish(malformed_message);
  spinFor(executor, 750ms);
  const std::string malformed_log = testing::internal::GetCapturedStderr();

  EXPECT_NE(
    std::string::npos,
    malformed_log.find("[POINTCLOUD_LAYOUT_REJECTED]"));
  EXPECT_NE(
    std::string::npos,
    malformed_log.find("data size must equal row_step * height"));
  EXPECT_EQ(0U, malformed_map_messages);
  ASSERT_TRUE(rclcpp::ok());

  testing::internal::CaptureStderr();
  cloud_publisher->publish(makeCloudMessage(makeIssue69Cloud(), kUnsafeStampSec));
  spinFor(executor, 750ms);
  const std::string unsafe_log = testing::internal::GetCapturedStderr();

  EXPECT_NE(std::string::npos, unsafe_log.find("[VOXEL_GRID_LAYOUT_OVERFLOW]"));
  EXPECT_NE(std::string::npos, unsafe_log.find("initial_map rejected before PCL"));
  EXPECT_EQ(0U, malformed_map_messages);
  EXPECT_EQ(0U, unsafe_map_messages);
  ASSERT_TRUE(rclcpp::ok());

  cloud_publisher->publish(
    makePaddedOrganizedXyzOnlyCloudMessage(
      makeSafeRegistrationCloud(), kSafeStampSec));
  ASSERT_TRUE(
    spinUntil(
      executor,
      [&safe_map_messages, &safe_pose_messages]() {
        return safe_map_messages >= 1U && safe_pose_messages >= 1U;
      },
      10s));

  EXPECT_EQ(0U, malformed_map_messages);
  EXPECT_EQ(0U, unsafe_map_messages);
  EXPECT_GE(safe_map_messages, 1U);
  EXPECT_GE(safe_pose_messages, 1U);
  EXPECT_TRUE(rclcpp::ok());

  executor.remove_node(component);
  executor.remove_node(probe);
}

TEST_F(
  ScanMatcherVoxelGridRecoveryTest,
  RejectsUnsafeAsyncMapUpdateThenProcessesSafeCloudAndShutsDown)
{
  rclcpp::NodeOptions probe_options;
  probe_options.use_intra_process_comms(true);
  auto probe = std::make_shared<rclcpp::Node>(
    "scanmatcher_voxel_grid_async_recovery_probe", probe_options);

  std::size_t initial_map_arrays = 0;
  std::size_t unsafe_map_arrays = 0;
  std::size_t recovered_map_arrays = 0;
  auto map_array_subscription =
    probe->create_subscription<lidarslam_msgs::msg::MapArray>(
    kMapArrayTopic,
    rclcpp::QoS(10),
    [&initial_map_arrays, &unsafe_map_arrays, &recovered_map_arrays](
      lidarslam_msgs::msg::MapArray::ConstSharedPtr message)
    {
      if (message->header.stamp.sec == kInitialAsyncStampSec) {
        ++initial_map_arrays;
      } else if (message->header.stamp.sec == kUnsafeAsyncStampSec) {
        ++unsafe_map_arrays;
      } else if (message->header.stamp.sec == kRecoveredAsyncStampSec) {
        ++recovered_map_arrays;
      }
    });
  auto cloud_publisher = probe->create_publisher<sensor_msgs::msg::PointCloud2>(
    kCloudTopic,
    rclcpp::SensorDataQoS().keep_last(10));

  rclcpp::NodeOptions component_options;
  component_options.use_intra_process_comms(true);
  component_options.arguments({
      "--ros-args",
      "-r", std::string("input_cloud:=") + kCloudTopic,
      "-r", std::string("map_array:=") + kMapArrayTopic,
    });
  component_options.parameter_overrides({
      rclcpp::Parameter("set_initial_pose", true),
      rclcpp::Parameter("publish_tf", false),
      rclcpp::Parameter("registration_method", "NDT"),
      rclcpp::Parameter("ndt_resolution", 0.5),
      rclcpp::Parameter("ndt_num_threads", 1),
      rclcpp::Parameter("ndt_max_iterations", 10),
      rclcpp::Parameter("vg_size_for_input", 0.5),
      rclcpp::Parameter("vg_size_for_map", 0.1),
      rclcpp::Parameter("min_points_for_scan", 20),
      rclcpp::Parameter("async_map_update", true),
      rclcpp::Parameter("async_map_update_warmup_submaps", 1),
      // The unsafe scan crosses this threshold. Its rejected map update must
      // not consume the movement, so the nearly identical safe retry crosses
      // the same threshold and publishes without requiring more travel.
      rclcpp::Parameter("trans_for_mapupdate", 0.02),
      rclcpp::Parameter("debug_flag", true),
      rclcpp::Parameter("motion_gate_enable", false),
      rclcpp::Parameter("reject_nonconverged_pose_update", false),
    });
  auto component = std::make_shared<graphslam::ScanMatcherComponent>(component_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(probe);
  executor.add_node(component);

  ASSERT_TRUE(
    spinUntil(
      executor,
      [&cloud_publisher, &probe]() {
        return cloud_publisher->get_subscription_count() == 1U &&
               probe->count_publishers(kMapArrayTopic) == 1U;
      },
      5s));

  cloud_publisher->publish(
    makeCloudMessage(makeSafeRegistrationCloud(), kInitialAsyncStampSec));
  ASSERT_TRUE(
    spinUntil(
      executor,
      [&initial_map_arrays]() {return initial_map_arrays >= 1U;},
      5s));

  testing::internal::CaptureStderr();
  cloud_publisher->publish(
    makeCloudMessage(makeIssue69MapUpdateCloud(), kUnsafeAsyncStampSec));
  spinFor(executor, 1s);
  const std::string unsafe_log = testing::internal::GetCapturedStderr();

  EXPECT_NE(std::string::npos, unsafe_log.find("[VOXEL_GRID_LAYOUT_OVERFLOW]"));
  EXPECT_NE(std::string::npos, unsafe_log.find("map_update rejected before PCL"));
  EXPECT_EQ(0U, unsafe_map_arrays);
  ASSERT_TRUE(rclcpp::ok());

  cloud_publisher->publish(
    makeCloudMessage(makeRecoveredMapUpdateCloud(), kRecoveredAsyncStampSec));
  ASSERT_TRUE(
    spinUntil(
      executor,
      [&recovered_map_arrays]() {return recovered_map_arrays >= 1U;},
      10s));

  EXPECT_EQ(0U, unsafe_map_arrays);
  EXPECT_GE(recovered_map_arrays, 1U);
  EXPECT_TRUE(rclcpp::ok());

  executor.remove_node(component);
  component.reset();
  executor.remove_node(probe);
}

}  // namespace
