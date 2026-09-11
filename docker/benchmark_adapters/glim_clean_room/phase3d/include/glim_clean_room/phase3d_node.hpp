#pragma once

#include "glim_clean_room/phase3b_session.hpp"
#include "glim_clean_room/phase3d_ingress.hpp"
#include "glim_clean_room/ros2_conversion.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace glim_clean_room::phase3d {

// A ROS 2 transport boundary around the host-owned conversion/parser and the
// exact-core Phase 3b/3c session.  No upstream GLIM ROS bridge is used.
class GlimPhase3dNode final : public rclcpp::Node {
 public:
  explicit GlimPhase3dNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GlimPhase3dNode() noexcept override;

  GlimPhase3dNode(const GlimPhase3dNode&) = delete;
  GlimPhase3dNode& operator=(const GlimPhase3dNode&) = delete;

 private:
  struct PendingPayload {
    IngressKind kind{IngressKind::kLidar};
    std::uint64_t bytes{0};
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    sensor_msgs::msg::Imu::ConstSharedPtr imu;
  };

  Status initialize();
  Status build_session();
  Status enqueue_lidar(const sensor_msgs::msg::PointCloud2& message);
  Status enqueue_imu(const sensor_msgs::msg::Imu& message);
  Status enqueue_payload(IngressItem item, PendingPayload payload);
  Status flush_ready_locked(bool force_at_eof);
  Status process_item_locked(const IngressItem& item);
  Status finalize_locked(std_srvs::srv::Trigger::Response& response);
  Status publish_outputs_locked(
      const std::vector<TrajectorySample>& trajectory,
      const std::vector<MapChunk>& maps);
  Result<nav_msgs::msg::Path> make_path(
      const std::vector<TrajectorySample>& trajectory) const;
  Result<sensor_msgs::msg::PointCloud2> make_map_cloud(
      const std::vector<MapChunk>& maps) const;
  Status latch_locked(ErrorCode code, const std::string& detail);
  void publish_diagnostic_locked(const std::string& message, std::uint8_t level);
  void on_lidar_message(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void on_imu_message(sensor_msgs::msg::Imu::ConstSharedPtr message);
  void on_finalize(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  SequenceContract contract_{};
  phase3b::RuntimeOptions runtime_options_{};
  std::unique_ptr<phase3b::GlimCoreSession> session_;
  std::unique_ptr<BoundedIngressQueue> ingress_;
  std::map<std::uint64_t, PendingPayload> payloads_;
  std::uint64_t next_arrival_{0};
  std::uint64_t next_submission_order_{0};
  std::uint64_t max_pending_events_{0};
  std::uint64_t reorder_window_ns_{0};
  bool terminal_{false};
  bool finalize_started_{false};
  bool close_called_{false};
  bool output_published_{false};
  bool reliable_input_qos_{true};
  std::uint64_t max_cloud_bytes_{0};
  std::uint64_t max_pending_payload_bytes_{0};
  std::uint64_t pending_payload_bytes_{0};
  std::mutex mutex_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finalize_service_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;

  std::string lidar_topic_;
  std::string imu_topic_;
  std::string trajectory_topic_;
  std::string map_topic_;
  std::string diagnostics_topic_;
  std::string finalize_service_name_;
  std::string lidar_frame_;
  std::string imu_frame_;
  std::string trajectory_frame_;
  std::string map_frame_;
};

}  // namespace glim_clean_room::phase3d
