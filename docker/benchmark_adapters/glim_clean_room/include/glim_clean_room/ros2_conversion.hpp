#pragma once

// This is an independently written message-to-view boundary.  It does not
// include or link any upstream ROS bridge.  The header is intentionally
// opt-in so the Phase 0 contract remains buildable on a host without ROS 2.
#if defined(GLIM_CLEAN_ROOM_WITH_ROS2)

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "glim_clean_room/pointcloud2_parser.hpp"

namespace glim_clean_room {

inline Result<PointCloud2View> pointcloud2_view_from_ros2(
    const sensor_msgs::msg::PointCloud2& message, std::uint64_t order) {
  const auto stamp = event_stamp_from_ros(
      order, static_cast<std::int64_t>(message.header.stamp.sec),
      static_cast<std::int64_t>(message.header.stamp.nanosec));
  if (!stamp) {
    return Result<PointCloud2View>::failure(stamp.error.code,
                                            stamp.error.detail);
  }
  PointCloud2View view;
  view.event = stamp.value;
  view.frame_id = message.header.frame_id;
  view.is_bigendian = message.is_bigendian;
  view.height = message.height;
  view.width = message.width;
  view.point_step = message.point_step;
  view.row_step = message.row_step;
  view.data = message.data;
  view.fields.reserve(message.fields.size());
  for (const auto& field : message.fields) {
    view.fields.push_back(PointCloud2FieldView{
        field.name, field.offset,
        static_cast<PointFieldType>(field.datatype), field.count});
  }
  return Result<PointCloud2View>::success(std::move(view));
}

inline Result<ImuSample> imu_sample_from_ros2(
    const sensor_msgs::msg::Imu& message, std::uint64_t order) {
  const auto stamp = event_stamp_from_ros(
      order, static_cast<std::int64_t>(message.header.stamp.sec),
      static_cast<std::int64_t>(message.header.stamp.nanosec));
  if (!stamp) {
    return Result<ImuSample>::failure(stamp.error.code, stamp.error.detail);
  }
  ImuSample sample;
  sample.event = stamp.value;
  sample.frame_id = message.header.frame_id;
  sample.linear_acceleration = {
      message.linear_acceleration.x, message.linear_acceleration.y,
      message.linear_acceleration.z};
  sample.angular_velocity = {
      message.angular_velocity.x, message.angular_velocity.y,
      message.angular_velocity.z};
  sample.acceleration_unit = AccelerationUnit::kMetersPerSecondSquared;
  sample.angular_velocity_unit = AngularVelocityUnit::kRadiansPerSecond;
  return Result<ImuSample>::success(std::move(sample));
}

}  // namespace glim_clean_room

#endif  // GLIM_CLEAN_ROOM_WITH_ROS2
