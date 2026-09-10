#include "glim_clean_room/phase3d_node.hpp"

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using glim_clean_room::phase3d::GlimPhase3dNode;

void write_file(const std::filesystem::path& path, const std::string& value) {
  std::ofstream output(path);
  if (!output.good()) throw std::runtime_error("cannot create ROS fixture file");
  output << value;
  output.close();
  if (!output.good()) throw std::runtime_error("cannot write ROS fixture file");
}

std::filesystem::path make_config() {
  const auto root = std::filesystem::temp_directory_path() /
                    "glim-clean-room-phase3d-ros-fixture";
  std::error_code error;
  std::filesystem::remove_all(root, error);
  if (error) throw std::runtime_error("cannot clear ROS fixture root");
  std::filesystem::create_directories(root);
  write_file(root / "config.json",
             R"({"global":{"config_path":"","config_logging":"config_logging.json","config_sensors":"config_sensors.json","config_preprocess":"config_preprocess.json","config_odometry":"config_odometry.json","config_sub_mapping":"config_sub_mapping.json","config_global_mapping":"config_global_mapping.json"}})");
  write_file(root / "config_logging.json",
             R"({"logging":{"save_logs":false,"rotate_logs":false}})");
  write_file(root / "config_sensors.json",
             R"({"sensors":{"imu_acc_noise":0.05,"imu_gyro_noise":0.02,"imu_int_noise":0.001,"imu_bias_noise":0.00001,"global_shutter_lidar":true,"T_lidar_imu":[0,0,0,0,0,0,1],"intensity_field":"intensity","ring_field":""}})");
  write_file(root / "config_preprocess.json",
             R"({"preprocess":{"distance_near_thresh":0.5,"distance_far_thresh":1000.0,"use_random_grid_downsampling":false,"downsample_resolution":0.01,"enable_outlier_removal":false,"enable_cropbox_filter":false,"k_correspondences":8,"num_threads":1}})");
  write_file(root / "config_odometry.json",
             R"({"odometry_estimation":{"initialization_mode":"NAIVE","initialization_window_size":0.0,"init_pose_damping_scale":10000000000.0,"smoother_lag":5.0,"registration_type":"GICP","max_iterations":1,"lru_thresh":100,"ivox_resolution":0.5,"ivox_min_dist":0.1,"target_downsampling_rate":0.1,"validate_imu":false,"save_imu_rate_trajectory":false,"num_threads":1,"fix_imu_bias":false,"compute_covs":false}})");
  write_file(root / "config_sub_mapping.json",
             R"({"sub_mapping":{"keyframe_update_interval_rot":0.01,"keyframe_update_interval_trans":0.1,"max_num_keyframes":50,"max_num_voxels":50000,"adaptive_max_num_voxels":0.5,"submap_target_num_points":40000,"submap_voxel_resolution":0.5,"min_dist_in_voxel":0.1,"max_num_points_in_voxel":100}})");
  write_file(root / "config_global_mapping.json",
             R"({"global_mapping":{"enable_imu":false,"enable_optimization":true,"create_between_factors":false,"between_registration_type":"GICP","registration_error_factor_type":"VGICP","submap_voxel_resolution":1.0,"submap_voxel_resolution_max":1.0,"submap_voxel_resolution_dmin":5.0,"submap_voxel_resolution_dmax":20.0,"submap_voxelmap_levels":2,"submap_voxelmap_scaling_factor":2.0,"randomsampling_rate":1.0,"max_implicit_loop_distance":100.0,"min_implicit_loop_overlap":0.1,"use_isam2_dogleg":false,"isam2_relinearize_skip":1,"isam2_relinearize_thresh":0.1,"init_pose_damping_scale":10000000000.0}})");
  return root;
}

sensor_msgs::msg::PointCloud2 make_cloud(std::int32_t sec, double shift) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  cloud.header.stamp.sec = sec;
  cloud.header.stamp.nanosec = 0;
  cloud.height = 1;
  cloud.width = 128;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  const auto add_field = [&cloud](const std::string& name, std::uint32_t offset,
                                  std::uint8_t datatype) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    cloud.fields.push_back(field);
  };
  add_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
  add_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
  add_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
  add_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
  add_field("ring", 16, sensor_msgs::msg::PointField::UINT32);
  add_field("time", 20, sensor_msgs::msg::PointField::FLOAT32);
  cloud.point_step = 24;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.row_step);
  for (std::size_t index = 0; index < cloud.width; ++index) {
    const auto row = static_cast<std::uint32_t>(index / 16);
    const auto column = static_cast<std::uint32_t>(index % 16);
    const float values[] = {
        static_cast<float>(1.0 + 0.11 * column + shift),
        static_cast<float>(2.0 + 0.13 * row),
        static_cast<float>(3.0 + 0.02 * ((row + column) % 4)),
        static_cast<float>(1.0 + column), 0.0F};
    auto* destination = cloud.data.data() + index * cloud.point_step;
    std::memcpy(destination, &values[0], sizeof(float));
    std::memcpy(destination + 4, &values[1], sizeof(float));
    std::memcpy(destination + 8, &values[2], sizeof(float));
    std::memcpy(destination + 12, &values[3], sizeof(float));
    const auto ring = row;
    std::memcpy(destination + 16, &ring, sizeof(ring));
    std::memcpy(destination + 20, &values[4], sizeof(float));
  }
  return cloud;
}

sensor_msgs::msg::Imu make_imu(std::int32_t sec) {
  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "imu";
  imu.header.stamp.sec = sec;
  imu.header.stamp.nanosec = 0;
  imu.linear_acceleration.z = 9.81;
  return imu;
}

template <typename Predicate>
bool spin_until(rclcpp::executors::SingleThreadedExecutor& executor,
                Predicate predicate, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(10));
    if (predicate()) return true;
  }
  return predicate();
}

TEST(Phase3dRosIntegration, SyntheticTopicsFinalizeExactlyOnceWithBoundedOutputs) {
  int argc = 0;
  char** argv = nullptr;
  rclcpp::init(argc, argv);
  const auto config = make_config();

  rclcpp::NodeOptions options;
  options.append_parameter_override("config_directory", config.string());
  options.append_parameter_override("require_map_output", true);
  // The two DDS topics have independent delivery order.  Hold the complete
  // synthetic three-second horizon and flush it deterministically at EOF;
  // a zero window would correctly reject a cross-topic late arrival.
  options.append_parameter_override("reorder_window_ns",
                                   static_cast<std::int64_t>(4000000000LL));
  options.append_parameter_override("max_cloud_bytes", 1024 * 1024);
  options.append_parameter_override("max_pending_payload_bytes", 4 * 1024 * 1024);
  auto node = std::make_shared<GlimPhase3dNode>(options);
  auto io = std::make_shared<rclcpp::Node>("phase3d_synthetic_io");

  auto lidar_pub = io->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/points", rclcpp::QoS(10).reliable());
  auto imu_pub = io->create_publisher<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::QoS(10).reliable());
  std::size_t path_count = 0;
  std::size_t map_count = 0;
  std::size_t input_event_count = 0;
  std::size_t post_eof_count = 0;
  nav_msgs::msg::Path received_path;
  sensor_msgs::msg::PointCloud2 received_map;
  auto path_sub = io->create_subscription<nav_msgs::msg::Path>(
      "/glim_clean_room/trajectory", rclcpp::QoS(1).reliable().transient_local(),
      [&path_count, &received_path](nav_msgs::msg::Path::ConstSharedPtr value) {
        ++path_count;
        received_path = *value;
      });
  auto map_sub = io->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/glim_clean_room/map", rclcpp::QoS(1).reliable().transient_local(),
      [&map_count, &received_map](sensor_msgs::msg::PointCloud2::ConstSharedPtr value) {
        ++map_count;
        received_map = *value;
      });
  auto diagnostic_sub = io->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/glim_clean_room/diagnostics", rclcpp::QoS(10).reliable(),
      [&input_event_count, &post_eof_count](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr value) {
        for (const auto& status : value->status) {
          if (status.message == "input event accepted") ++input_event_count;
          if (status.message.find("post_eof") != std::string::npos) {
            ++post_eof_count;
          }
        }
      });
  // Keep the subscription alive and make the connection itself deterministic
  // before any synthetic message is sent.
  (void)diagnostic_sub;
  auto finalize_client = io->create_client<std_srvs::srv::Trigger>(
      "/glim_clean_room/finalize");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(io);
  ASSERT_TRUE(spin_until(executor, [&]() { return lidar_pub->get_subscription_count() == 1 &&
                                                imu_pub->get_subscription_count() == 1; },
                         std::chrono::seconds(2)));
  ASSERT_TRUE(spin_until(executor, [&]() { return diagnostic_sub->get_publisher_count() >= 1; },
                         std::chrono::seconds(2)));
  const auto publish_and_wait = [&](auto publish, std::size_t expected) {
    publish();
    ASSERT_TRUE(spin_until(executor, [&]() { return input_event_count >= expected; },
                           std::chrono::seconds(2))) <<
        "diagnostic input event count=" << input_event_count;
  };
  publish_and_wait([&]() { imu_pub->publish(make_imu(1)); }, 1);
  publish_and_wait([&]() { imu_pub->publish(make_imu(2)); }, 2);
  publish_and_wait([&]() { lidar_pub->publish(make_cloud(2, 0.0)); }, 3);
  publish_and_wait([&]() { imu_pub->publish(make_imu(3)); }, 4);
  publish_and_wait([&]() { lidar_pub->publish(make_cloud(3, 0.02)); }, 5);
  ASSERT_TRUE(spin_until(executor, [&]() { return finalize_client->service_is_ready(); },
                         std::chrono::seconds(2)));
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = finalize_client->async_send_request(request);
  ASSERT_TRUE(spin_until(executor, [&]() {
    return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
  }, std::chrono::seconds(20)));
  const auto response = future.get();
  ASSERT_TRUE(response->success) << response->message;
  ASSERT_TRUE(spin_until(executor, [&]() { return path_count == 1 && map_count == 1; },
                         std::chrono::seconds(2)));
  EXPECT_EQ(path_count, 1U);
  EXPECT_EQ(map_count, 1U);
  ASSERT_EQ(received_path.header.frame_id, "world");
  ASSERT_EQ(received_path.poses.size(), 2U);
  EXPECT_EQ(received_path.poses[0].header.frame_id, "world");
  EXPECT_EQ(received_path.poses[0].header.stamp.sec, 2);
  EXPECT_EQ(received_path.poses[1].header.stamp.sec, 3);
  EXPECT_EQ(received_map.header.frame_id, "world");
  EXPECT_EQ(received_map.header.stamp.sec, 3);
  EXPECT_GT(received_map.width, 0U);
  EXPECT_EQ(received_map.row_step,
            received_map.width * received_map.point_step);

  // EOF is a one-way transport boundary.  A message sent after the sealed
  // output must be rejected and cannot create a second publication.
  lidar_pub->publish(make_cloud(4, 0.04));
  ASSERT_TRUE(spin_until(executor, [&]() { return post_eof_count >= 1; },
                         std::chrono::seconds(2)));

  auto second_future = finalize_client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(spin_until(executor, [&]() {
    return second_future.wait_for(std::chrono::milliseconds(0)) ==
           std::future_status::ready;
  }, std::chrono::seconds(2)));
  const auto second = second_future.get();
  EXPECT_FALSE(second->success);
  EXPECT_EQ(path_count, 1U);
  EXPECT_EQ(map_count, 1U);

  executor.remove_node(io);
  executor.remove_node(node);
  io.reset();
  node.reset();
  rclcpp::shutdown();
}

}  // namespace
