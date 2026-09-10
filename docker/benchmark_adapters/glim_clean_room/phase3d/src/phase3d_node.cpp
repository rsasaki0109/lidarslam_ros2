#include "glim_clean_room/phase3d_node.hpp"
#include "glim_clean_room/phase3d_output_contract.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <utility>

namespace glim_clean_room::phase3d {
namespace {

constexpr std::uint64_t kMaxReorderWindowNs = 60ULL * 1000000000ULL;
constexpr std::uint64_t kMaxPendingEvents = 1000000ULL;

std::array<double, 16> identity_transform() {
  return {1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0};
}

Result<PointTimeUnit> parse_time_unit(const std::string& value) {
  if (value == "seconds") {
    return Result<PointTimeUnit>::success(PointTimeUnit::kSeconds);
  }
  if (value == "milliseconds") {
    return Result<PointTimeUnit>::success(PointTimeUnit::kMilliseconds);
  }
  if (value == "microseconds") {
    return Result<PointTimeUnit>::success(PointTimeUnit::kMicroseconds);
  }
  if (value == "nanoseconds") {
    return Result<PointTimeUnit>::success(PointTimeUnit::kNanoseconds);
  }
  return Result<PointTimeUnit>::failure(
      ErrorCode::kWrongTimeUnit,
      "point_time_unit must be seconds, milliseconds, microseconds, or nanoseconds");
}

Status positive_parameter(std::int64_t value, const char* name,
                          std::uint64_t* destination) {
  if (destination == nullptr || value <= 0) {
    return Status::failure(ErrorCode::kInvalidLayout,
                           std::string(name == nullptr ? "parameter" : name) +
                               " must be positive");
  }
  if (static_cast<std::uint64_t>(value) >
      static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    return Status::failure(ErrorCode::kLedgerOverflow,
                           std::string(name == nullptr ? "parameter" : name) +
                               " exceeds host size_t");
  }
  *destination = static_cast<std::uint64_t>(value);
  return Status::success();
}

void add_key_value(diagnostic_msgs::msg::DiagnosticStatus& status,
                   const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(std::move(item));
}

std::string error_text(const Status& status) {
  std::ostringstream stream;
  stream << error_code_name(status.error.code);
  if (!status.error.detail.empty()) stream << ": " << status.error.detail;
  return stream.str();
}

bool float32_representable(double value) {
  const float converted = static_cast<float>(value);
  return std::isfinite(value) && std::isfinite(converted);
}

Result<std::uint64_t> checked_add(std::uint64_t lhs, std::uint64_t rhs,
                                  const char* detail) {
  if (rhs > std::numeric_limits<std::uint64_t>::max() - lhs) {
    return Result<std::uint64_t>::failure(ErrorCode::kLedgerOverflow,
                                          detail == nullptr ? "size overflow"
                                                            : detail);
  }
  return Result<std::uint64_t>::success(lhs + rhs);
}

Result<std::uint64_t> pointcloud_payload_bytes(
    const sensor_msgs::msg::PointCloud2& message) {
  std::uint64_t total = static_cast<std::uint64_t>(message.data.size());
  const auto fields = checked_add(
      total, static_cast<std::uint64_t>(message.fields.size()),
      "PointCloud2 field count overflows payload accounting");
  if (!fields) return fields;
  total = fields.value;
  for (const auto& field : message.fields) {
    const auto name = checked_add(
        total, static_cast<std::uint64_t>(field.name.size()),
        "PointCloud2 field name size overflows payload accounting");
    if (!name) return name;
    total = name.value;
  }
  return Result<std::uint64_t>::success(total);
}

constexpr std::uint64_t kImuPayloadBytes = 256;

Result<builtin_interfaces::msg::Time> ros_time_from_stamp(
    StampNanoseconds nanoseconds) {
  constexpr StampNanoseconds kNanosecondsPerSecond = 1000000000;
  StampNanoseconds seconds = nanoseconds / kNanosecondsPerSecond;
  StampNanoseconds remainder = nanoseconds % kNanosecondsPerSecond;
  if (remainder < 0) {
    --seconds;
    remainder += kNanosecondsPerSecond;
  }
  if (seconds < std::numeric_limits<std::int32_t>::min() ||
      seconds > std::numeric_limits<std::int32_t>::max()) {
    return Result<builtin_interfaces::msg::Time>::failure(
        ErrorCode::kInvalidStamp,
        "event stamp seconds exceed the ROS builtin Time int32 range");
  }
  builtin_interfaces::msg::Time output;
  output.sec = static_cast<std::int32_t>(seconds);
  output.nanosec = static_cast<std::uint32_t>(remainder);
  return Result<builtin_interfaces::msg::Time>::success(output);
}

}  // namespace

GlimPhase3dNode::GlimPhase3dNode(const rclcpp::NodeOptions& options)
    : Node("glim_clean_room_phase3d", options) {
  callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/glim_clean_room/diagnostics", rclcpp::QoS(10).reliable());
  Status status;
  try {
    status = initialize();
  } catch (const std::exception& exception) {
    status = Status::failure(
        ErrorCode::kCoreFailure,
        std::string("Phase 3d node initialization threw: ") + exception.what());
  } catch (...) {
    status = Status::failure(ErrorCode::kCoreFailure,
                             "Phase 3d node initialization threw an unknown exception");
  }
  if (!status) {
    std::lock_guard<std::mutex> lock(mutex_);
    (void)latch_locked(status.error.code, status.error.detail);
  }
}

GlimPhase3dNode::~GlimPhase3dNode() noexcept {
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session_ && !close_called_) {
      close_called_ = true;
      (void)session_->close();
    }
  } catch (...) {
    // A ROS node destructor cannot publish a second failure.  The session's
    // terminal latch remains the authoritative cleanup record.
  }
}

Status GlimPhase3dNode::initialize() {
  lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/points");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
  trajectory_topic_ = declare_parameter<std::string>(
      "trajectory_topic", "/glim_clean_room/trajectory");
  map_topic_ = declare_parameter<std::string>("map_topic", "/glim_clean_room/map");
  diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/glim_clean_room/diagnostics");
  finalize_service_name_ = declare_parameter<std::string>(
      "finalize_service", "/glim_clean_room/finalize");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "imu");
  trajectory_frame_ = declare_parameter<std::string>("trajectory_frame", "world");
  map_frame_ = declare_parameter<std::string>("map_frame", "world");
  const auto config_directory = declare_parameter<std::string>(
      "config_directory", "");
  const auto sequence_id = declare_parameter<std::string>(
      "sequence_id", "phase3d_ros2_live");
  const auto time_unit_name = declare_parameter<std::string>(
      "point_time_unit", "milliseconds");
  const auto require_map = declare_parameter<bool>("require_map_output", true);
  const auto qos_depth = declare_parameter<std::int64_t>("qos_depth", 10);
  const auto max_pending = declare_parameter<std::int64_t>(
      "max_pending_events", 4096);
  const auto reorder_window = declare_parameter<std::int64_t>(
      "reorder_window_ns", 0);
  reliable_input_qos_ = declare_parameter<bool>("reliable_input_qos", true);
  const auto max_cloud_bytes = declare_parameter<std::int64_t>(
      "max_cloud_bytes", 64 * 1024 * 1024);
  const auto max_pending_payload_bytes = declare_parameter<std::int64_t>(
      "max_pending_payload_bytes", 256 * 1024 * 1024);
  const auto ledger_capacity = declare_parameter<std::int64_t>(
      "ledger_capacity", 4096);
  const auto max_pending_trajectory = declare_parameter<std::int64_t>(
      "max_pending_trajectory", 4096);
  const auto max_map_frames = declare_parameter<std::int64_t>(
      "max_map_frames", 4096);
  const auto max_map_submaps = declare_parameter<std::int64_t>(
      "max_map_submaps", 1024);
  const auto max_map_points = declare_parameter<std::int64_t>(
      "max_map_points", 1048576);
  const auto max_map_chunks = declare_parameter<std::int64_t>(
      "max_map_chunks", 1);
  const auto default_transform = identity_transform();
  const auto transform = declare_parameter<std::vector<double>>(
      "calibration_transform",
      std::vector<double>(default_transform.begin(), default_transform.end()));
  const auto inverse_transform = declare_parameter<std::vector<double>>(
      "calibration_inverse_transform",
      std::vector<double>(default_transform.begin(), default_transform.end()));

  if (qos_depth <= 0 || qos_depth > 1000000) {
    return Status::failure(ErrorCode::kInvalidLayout,
                           "qos_depth must be in [1, 1000000]");
  }
  if (reorder_window < 0 ||
      static_cast<std::uint64_t>(reorder_window) > kMaxReorderWindowNs) {
    return Status::failure(
        ErrorCode::kInvalidLayout,
        "reorder_window_ns must be nonnegative and at most 60 seconds");
  }
  if (max_pending <= 0 ||
      static_cast<std::uint64_t>(max_pending) > kMaxPendingEvents) {
    return Status::failure(
        ErrorCode::kLedgerOverflow,
        "max_pending_events must be in [1, 1000000]; overflow is terminal");
  }
  if (max_cloud_bytes <= 0 || max_pending_payload_bytes <= 0 ||
      max_pending_payload_bytes < max_cloud_bytes) {
    return Status::failure(
        ErrorCode::kInvalidLayout,
        "max_cloud_bytes and max_pending_payload_bytes must be positive, with pending >= cloud");
  }
  max_cloud_bytes_ = static_cast<std::uint64_t>(max_cloud_bytes);
  max_pending_payload_bytes_ =
      static_cast<std::uint64_t>(max_pending_payload_bytes);
  if (transform.size() != 16 || inverse_transform.size() != 16) {
    return Status::failure(ErrorCode::kInvalidCalibration,
                           "calibration transforms must contain exactly 16 values");
  }
  const auto time_unit = parse_time_unit(time_unit_name);
  if (!time_unit) return Status::failure(time_unit.error.code, time_unit.error.detail);

  std::uint64_t ledger_capacity_value = 0;
  std::uint64_t max_pending_trajectory_value = 0;
  std::uint64_t max_map_frames_value = 0;
  std::uint64_t max_map_submaps_value = 0;
  std::uint64_t max_map_points_value = 0;
  std::uint64_t max_map_chunks_value = 0;
  const auto ledger_valid = positive_parameter(
      ledger_capacity, "ledger_capacity", &ledger_capacity_value);
  if (!ledger_valid) return ledger_valid;
  const auto trajectory_valid = positive_parameter(
      max_pending_trajectory, "max_pending_trajectory",
      &max_pending_trajectory_value);
  if (!trajectory_valid) return trajectory_valid;
  const auto map_frames_valid = positive_parameter(
      max_map_frames, "max_map_frames", &max_map_frames_value);
  if (!map_frames_valid) return map_frames_valid;
  const auto map_submaps_valid = positive_parameter(
      max_map_submaps, "max_map_submaps", &max_map_submaps_value);
  if (!map_submaps_valid) return map_submaps_valid;
  const auto map_points_valid = positive_parameter(
      max_map_points, "max_map_points", &max_map_points_value);
  if (!map_points_valid) return map_points_valid;
  const auto map_chunks_valid = positive_parameter(
      max_map_chunks, "max_map_chunks", &max_map_chunks_value);
  if (!map_chunks_valid) return map_chunks_valid;

  diagnostics_topic_ = diagnostics_topic_.empty()
                            ? "/glim_clean_room/diagnostics"
                            : diagnostics_topic_;
  diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(static_cast<std::size_t>(qos_depth)).reliable());
  trajectory_publisher_ = create_publisher<nav_msgs::msg::Path>(
      trajectory_topic_, rclcpp::QoS(1).reliable().transient_local());
  map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::QoS(1).reliable().transient_local());

  runtime_options_.config_directory = config_directory;
  runtime_options_.ledger_capacity = static_cast<std::size_t>(ledger_capacity_value);
  runtime_options_.max_pending_trajectory =
      static_cast<std::size_t>(max_pending_trajectory_value);
  runtime_options_.max_map_frames = static_cast<std::size_t>(max_map_frames_value);
  runtime_options_.max_map_submaps = static_cast<std::size_t>(max_map_submaps_value);
  runtime_options_.max_map_points = static_cast<std::size_t>(max_map_points_value);
  runtime_options_.max_map_chunks = static_cast<std::size_t>(max_map_chunks_value);
  max_pending_events_ = static_cast<std::uint64_t>(max_pending);
  reorder_window_ns_ = static_cast<std::uint64_t>(reorder_window);

  contract_.sequence_id = sequence_id;
  contract_.point_time_unit = time_unit.value;
  contract_.calibration.convention = CalibrationConvention::kTImuLidar;
  contract_.calibration.inverse_convention = CalibrationConvention::kTLidarImu;
  contract_.calibration.imu_frame = imu_frame_;
  contract_.calibration.lidar_frame = lidar_frame_;
  std::copy(transform.begin(), transform.end(), contract_.calibration.transform.begin());
  std::copy(inverse_transform.begin(), inverse_transform.end(),
            contract_.calibration.inverse_transform.begin());
  contract_.point_fields.x = {"x", PointFieldType::kFloat32, 1};
  contract_.point_fields.y = {"y", PointFieldType::kFloat32, 1};
  contract_.point_fields.z = {"z", PointFieldType::kFloat32, 1};
  contract_.point_fields.intensity = {"intensity", PointFieldType::kFloat32, 1};
  contract_.point_fields.ring = {"ring", PointFieldType::kUInt32, 1};
  contract_.point_fields.relative_time = {"time", PointFieldType::kFloat32, 1};
  contract_.point_fields.time_unit = time_unit.value;
  contract_.trajectory_frame = trajectory_frame_;
  contract_.map_frame = map_frame_;
  contract_.require_trajectory_output = true;
  contract_.require_map_output = require_map;
  contract_.artifacts = {
      {ArtifactRole::kInput, "ros2/topics"},
      {ArtifactRole::kCalibration, "ros2/calibration"},
      {ArtifactRole::kConfiguration, "config/runtime"},
      {ArtifactRole::kCoreSource, "glim/pinned-core"}};

  const auto created = build_session();
  if (!created) return created;
  ingress_ = std::make_unique<BoundedIngressQueue>(
      static_cast<std::size_t>(max_pending_events_), reorder_window_ns_);

  auto input_qos = rclcpp::QoS(static_cast<std::size_t>(qos_depth))
                       .durability_volatile();
  if (reliable_input_qos_) {
    input_qos.reliable();
  } else {
    input_qos.best_effort();
  }
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  lidar_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, input_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        on_lidar_message(std::move(message));
      }, subscription_options);
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, input_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        on_imu_message(std::move(message));
      }, subscription_options);
  finalize_service_ = create_service<std_srvs::srv::Trigger>(
      finalize_service_name_,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        on_finalize(request, std::move(response));
      }, rclcpp::ServicesQoS(), callback_group_);
  publish_diagnostic_locked("Phase 3d node ready; awaiting explicit finalize", 0);
  return Status::success();
}

Status GlimPhase3dNode::build_session() {
  auto created = phase3b::GlimCoreSession::create(
      contract_, runtime_options_);
  if (!created) {
    return Status::failure(created.error.code,
                           "Phase 3d exact-core session creation failed: " +
                               created.error.detail);
  }
  session_ = std::move(created.value);
  return Status::success();
}

Status GlimPhase3dNode::latch_locked(ErrorCode code, const std::string& detail) {
  if (!terminal_) {
    terminal_ = true;
    payloads_.clear();
    pending_payload_bytes_ = 0;
    publish_diagnostic_locked(error_code_name(code) + std::string(": ") + detail, 2);
  }
  return Status::failure(code, detail);
}

void GlimPhase3dNode::publish_diagnostic_locked(const std::string& message,
                                             std::uint8_t level) {
  if (!diagnostic_publisher_) return;
  try {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "glim_clean_room/phase3d";
    status.level = level;
    status.message = message;
    add_key_value(status, "terminal", terminal_ ? "true" : "false");
    add_key_value(status, "finalize_started", finalize_started_ ? "true" : "false");
    add_key_value(status, "output_published", output_published_ ? "true" : "false");
    add_key_value(status, "pending_events",
                  ingress_ ? std::to_string(ingress_->pending_size()) : "0");
    if (session_) {
      const auto& counters = session_->boundary().counters();
      add_key_value(status, "lidar_submitted", std::to_string(counters.lidar_submitted));
      add_key_value(status, "imu_submitted", std::to_string(counters.imu_submitted));
      add_key_value(status, "trajectory_outputs",
                    std::to_string(counters.trajectory_outputs));
      add_key_value(status, "map_outputs", std::to_string(counters.map_outputs));
      add_key_value(status, "failures", std::to_string(counters.failures));
    }
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(std::move(array));
  } catch (...) {
    // Diagnostics cannot turn a terminal core/input error into a second
    // publication or an exception escaping a ROS callback.
  }
}

Status GlimPhase3dNode::enqueue_lidar(
    const sensor_msgs::msg::PointCloud2& message) {
  const auto stamp = event_stamp_from_ros(
      0, static_cast<std::int64_t>(message.header.stamp.sec),
      static_cast<std::int64_t>(message.header.stamp.nanosec));
  std::lock_guard<std::mutex> lock(mutex_);
  if (terminal_) return Status::failure(ErrorCode::kInvalidState,
                                        "Phase 3d node is terminally failed");
  if (!stamp) return latch_locked(stamp.error.code, stamp.error.detail);
  const auto payload_bytes = pointcloud_payload_bytes(message);
  if (!payload_bytes) {
    return latch_locked(payload_bytes.error.code, payload_bytes.error.detail);
  }
  if (payload_bytes.value > max_cloud_bytes_) {
    return latch_locked(ErrorCode::kLedgerOverflow,
                        "PointCloud2 payload exceeds max_cloud_bytes");
  }
  if (next_arrival_ == std::numeric_limits<std::uint64_t>::max()) {
    return latch_locked(ErrorCode::kCounterOverflow,
                        "ROS host arrival ordinal exhausted");
  }
  const auto arrival = next_arrival_++;
  auto payload = PendingPayload{IngressKind::kLidar, payload_bytes.value,
                                std::make_shared<sensor_msgs::msg::PointCloud2>(message),
                                nullptr};
  return enqueue_payload(IngressItem{stamp.value.nanoseconds, arrival,
                                     IngressKind::kLidar},
                         std::move(payload));
}

Status GlimPhase3dNode::enqueue_imu(const sensor_msgs::msg::Imu& message) {
  const auto stamp = event_stamp_from_ros(
      0, static_cast<std::int64_t>(message.header.stamp.sec),
      static_cast<std::int64_t>(message.header.stamp.nanosec));
  std::lock_guard<std::mutex> lock(mutex_);
  if (terminal_) return Status::failure(ErrorCode::kInvalidState,
                                        "Phase 3d node is terminally failed");
  if (!stamp) return latch_locked(stamp.error.code, stamp.error.detail);
  if (kImuPayloadBytes > max_pending_payload_bytes_) {
    return latch_locked(ErrorCode::kLedgerOverflow,
                        "IMU payload exceeds max_pending_payload_bytes");
  }
  if (next_arrival_ == std::numeric_limits<std::uint64_t>::max()) {
    return latch_locked(ErrorCode::kCounterOverflow,
                        "ROS host arrival ordinal exhausted");
  }
  const auto arrival = next_arrival_++;
  auto payload = PendingPayload{IngressKind::kImu, kImuPayloadBytes,
                                nullptr,
                                std::make_shared<sensor_msgs::msg::Imu>(message)};
  return enqueue_payload(IngressItem{stamp.value.nanoseconds, arrival,
                                     IngressKind::kImu},
                         std::move(payload));
}

Status GlimPhase3dNode::enqueue_payload(IngressItem item, PendingPayload payload) {
  if (payload.bytes > max_pending_payload_bytes_ ||
      pending_payload_bytes_ > max_pending_payload_bytes_ - payload.bytes) {
    return latch_locked(ErrorCode::kLedgerOverflow,
                        "pending ROS payload bytes exhausted; no message was dropped silently");
  }
  try {
    const auto inserted = payloads_.emplace(item.arrival, std::move(payload));
    if (!inserted.second) {
      return latch_locked(ErrorCode::kOrderViolation,
                          "ROS payload arrival ordinal was duplicated");
    }
    const auto queued = ingress_->enqueue(item);
    if (!queued) {
      payloads_.erase(item.arrival);
      return latch_locked(queued.error.code, queued.error.detail);
    }
    pending_payload_bytes_ += inserted.first->second.bytes;
    publish_diagnostic_locked("input event accepted", 0);
  } catch (const std::exception& exception) {
    payloads_.erase(item.arrival);
    return latch_locked(ErrorCode::kCoreFailure,
                        std::string("ROS payload allocation failed: ") +
                            exception.what());
  }
  return flush_ready_locked(false);
}

Status GlimPhase3dNode::flush_ready_locked(bool force_at_eof) {
  const auto ready = ingress_->flush_ready(force_at_eof);
  if (!ready) return latch_locked(ready.error.code, ready.error.detail);
  for (const auto& item : ready.value) {
    const auto found = payloads_.find(item.arrival);
    if (found == payloads_.end()) {
      return latch_locked(ErrorCode::kCoreFailure,
                          "ROS ingress queue referenced a missing payload");
    }
    const auto payload_bytes = found->second.bytes;
    const auto status = process_item_locked(item);
    if (!status) {
      pending_payload_bytes_ = 0;
      return status;
    }
    if (payload_bytes > pending_payload_bytes_) {
      return latch_locked(ErrorCode::kCoreFailure,
                          "ROS pending payload byte accounting underflow");
    }
    pending_payload_bytes_ -= payload_bytes;
    payloads_.erase(found);
  }
  return Status::success();
}

Status GlimPhase3dNode::process_item_locked(const IngressItem& item) {
  if (session_ == nullptr || terminal_) {
    return latch_locked(ErrorCode::kInvalidState,
                        "ROS event arrived without an active exact-core session");
  }
  if (next_submission_order_ == std::numeric_limits<std::uint64_t>::max()) {
    return latch_locked(ErrorCode::kCounterOverflow,
                        "ROS submission order exhausted");
  }
  const auto found = payloads_.find(item.arrival);
  if (found == payloads_.end()) {
    return latch_locked(ErrorCode::kCoreFailure,
                        "ROS event payload disappeared before serialization");
  }
  Status status;
  if (item.kind == IngressKind::kLidar) {
    if (!found->second.cloud) {
      return latch_locked(ErrorCode::kCoreFailure,
                          "ROS LiDAR payload is null at serialization");
    }
    const auto view = pointcloud2_view_from_ros2(*found->second.cloud,
                                                 next_submission_order_);
    if (!view) return latch_locked(view.error.code, view.error.detail);
    const auto frame = parse_pointcloud2(view.value, contract_.point_fields);
    if (!frame) return latch_locked(frame.error.code, frame.error.detail);
    status = session_->submit_lidar(frame.value);
  } else {
    if (!found->second.imu) {
      return latch_locked(ErrorCode::kCoreFailure,
                          "ROS IMU payload is null at serialization");
    }
    const auto sample = imu_sample_from_ros2(*found->second.imu,
                                             next_submission_order_);
    if (!sample) return latch_locked(sample.error.code, sample.error.detail);
    status = session_->submit_imu(sample.value);
  }
  if (!status) return latch_locked(status.error.code, status.error.detail);
  const auto submitted = ingress_->note_submitted(item.stamp);
  if (!submitted) return latch_locked(submitted.error.code, submitted.error.detail);
  ++next_submission_order_;
  return Status::success();
}

Result<nav_msgs::msg::Path> GlimPhase3dNode::make_path(
    const std::vector<TrajectorySample>& trajectory) const {
  if (trajectory.empty()) {
    return Result<nav_msgs::msg::Path>::failure(
        ErrorCode::kUnsupportedCoreOutput, "Phase 3d received an empty trajectory");
  }
  const auto path_stamp = ros_time_from_stamp(trajectory.back().event.nanoseconds);
  if (!path_stamp) {
    return Result<nav_msgs::msg::Path>::failure(path_stamp.error.code,
                                                path_stamp.error.detail);
  }
  nav_msgs::msg::Path path;
  path.header.frame_id = trajectory_frame_;
  path.header.stamp = path_stamp.value;
  path.poses.reserve(trajectory.size());
  bool have_stamp = false;
  StampNanoseconds previous_stamp = 0;
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    const auto& sample = trajectory[index];
    const auto valid = validate_trajectory_sample(sample);
    if (!valid) return Result<nav_msgs::msg::Path>::failure(valid.error.code,
                                                            valid.error.detail);
    if (sample.frame_id != trajectory_frame_) {
      return Result<nav_msgs::msg::Path>::failure(
          ErrorCode::kInvalidLayout, "trajectory frame differs from ROS output frame");
    }
    if (sample.event.order != index ||
        (have_stamp && sample.event.nanoseconds < previous_stamp)) {
      return Result<nav_msgs::msg::Path>::failure(
          ErrorCode::kOutputOrderViolation,
          "trajectory output order or stamp is not contiguous and monotonic");
    }
    previous_stamp = sample.event.nanoseconds;
    have_stamp = true;
    const auto pose_stamp = ros_time_from_stamp(sample.event.nanoseconds);
    if (!pose_stamp) {
      return Result<nav_msgs::msg::Path>::failure(pose_stamp.error.code,
                                                  pose_stamp.error.detail);
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = trajectory_frame_;
    pose.header.stamp = pose_stamp.value;
    pose.pose.position.x = sample.pose[0];
    pose.pose.position.y = sample.pose[1];
    pose.pose.position.z = sample.pose[2];
    pose.pose.orientation.x = sample.pose[3];
    pose.pose.orientation.y = sample.pose[4];
    pose.pose.orientation.z = sample.pose[5];
    pose.pose.orientation.w = sample.pose[6];
    path.poses.push_back(std::move(pose));
  }
  return Result<nav_msgs::msg::Path>::success(std::move(path));
}

Result<sensor_msgs::msg::PointCloud2> GlimPhase3dNode::make_map_cloud(
    const std::vector<MapChunk>& maps) const {
  if (maps.size() != 1) {
    return Result<sensor_msgs::msg::PointCloud2>::failure(
        ErrorCode::kUnsupportedCoreOutput,
        "Phase 3d requires exactly one completed map chunk");
  }
  const auto& chunk = maps.front();
  const auto valid = validate_map_chunk(chunk);
  if (!valid) return Result<sensor_msgs::msg::PointCloud2>::failure(
      valid.error.code, valid.error.detail);
  if (chunk.frame_id != map_frame_ || chunk.points.size() >
                                         std::numeric_limits<std::uint32_t>::max()) {
    return Result<sensor_msgs::msg::PointCloud2>::failure(
        ErrorCode::kInvalidLayout, "map chunk cannot be represented by PointCloud2");
  }
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = map_frame_;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(chunk.points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  const auto add_field = [&cloud](const std::string& name, std::uint32_t offset,
                                  std::uint8_t datatype) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    cloud.fields.push_back(std::move(field));
  };
  add_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
  add_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
  add_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
  add_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
  add_field("ring", 16, sensor_msgs::msg::PointField::UINT32);
  add_field("time", 20, sensor_msgs::msg::PointField::FLOAT32);
  cloud.point_step = 24;
  const auto row_step = checked_row_step(cloud.width, cloud.point_step);
  if (!row_step) {
    return Result<sensor_msgs::msg::PointCloud2>::failure(
        row_step.error.code, row_step.error.detail);
  }
  cloud.row_step = row_step.value;
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
  for (std::size_t index = 0; index < chunk.points.size(); ++index) {
    const auto& point = chunk.points[index];
    if (!float32_representable(point.x) || !float32_representable(point.y) ||
        !float32_representable(point.z) || !float32_representable(point.intensity) ||
        !float32_representable(point.relative_time)) {
      return Result<sensor_msgs::msg::PointCloud2>::failure(
          ErrorCode::kNonFinite, "map PointCloud2 float32 conversion is non-finite");
    }
    auto* destination = cloud.data.data() + index * cloud.point_step;
    const float values[] = {static_cast<float>(point.x),
                            static_cast<float>(point.y),
                            static_cast<float>(point.z),
                            static_cast<float>(point.intensity),
                            static_cast<float>(point.relative_time)};
    std::memcpy(destination, &values[0], sizeof(float));
    std::memcpy(destination + 4, &values[1], sizeof(float));
    std::memcpy(destination + 8, &values[2], sizeof(float));
    std::memcpy(destination + 12, &values[3], sizeof(float));
    std::memcpy(destination + 16, &point.ring, sizeof(point.ring));
    std::memcpy(destination + 20, &values[4], sizeof(float));
  }
  return Result<sensor_msgs::msg::PointCloud2>::success(std::move(cloud));
}

Status GlimPhase3dNode::publish_outputs_locked(
    const std::vector<TrajectorySample>& trajectory,
    const std::vector<MapChunk>& maps) {
  const auto path = make_path(trajectory);
  if (!path) return latch_locked(path.error.code, path.error.detail);
  Result<sensor_msgs::msg::PointCloud2> map;
  if (contract_.require_map_output) {
    map = make_map_cloud(maps);
    if (!map) return latch_locked(map.error.code, map.error.detail);
    map.value.header.stamp = path.value.header.stamp;
  }
  try {
    trajectory_publisher_->publish(path.value);
    if (contract_.require_map_output) map_publisher_->publish(map.value);
  } catch (const std::exception& exception) {
    return latch_locked(ErrorCode::kCoreFailure,
                        std::string("ROS output publication threw: ") +
                            exception.what());
  } catch (...) {
    return latch_locked(ErrorCode::kCoreFailure,
                        "ROS output publication threw an unknown exception");
  }
  output_published_ = true;
  publish_diagnostic_locked("Phase 3d finalized and published complete outputs", 0);
  return Status::success();
}

Status GlimPhase3dNode::finalize_locked(
    std_srvs::srv::Trigger::Response& response) {
  if (terminal_) {
    response.success = false;
    response.message = "Phase 3d is terminally failed";
    return Status::failure(ErrorCode::kInvalidState, response.message);
  }
  if (finalize_started_) {
    response.success = false;
    response.message = "finalize is exactly-once and was already requested";
    return Status::failure(ErrorCode::kInvalidState, response.message);
  }
  finalize_started_ = true;
  const auto eof = ingress_->request_eof();
  if (!eof) {
    response.success = false;
    response.message = error_text(eof);
    return latch_locked(eof.error.code, eof.error.detail);
  }
  const auto flushed = flush_ready_locked(true);
  if (!flushed) {
    response.success = false;
    response.message = error_text(flushed);
    return flushed;
  }
  if (!payloads_.empty()) {
    response.success = false;
    response.message = "internal ROS payload table was not drained";
    return latch_locked(ErrorCode::kCoreFailure, response.message);
  }
  if (!session_) {
    response.success = false;
    response.message = "exact-core session is unavailable";
    return latch_locked(ErrorCode::kInvalidState, response.message);
  }
  const auto closed = session_->close();
  close_called_ = true;
  if (!closed) {
    response.success = false;
    response.message = error_text(closed);
    return latch_locked(closed.error.code, closed.error.detail);
  }
  const auto trajectory = session_->take_trajectory();
  if (!trajectory) {
    response.success = false;
    response.message = error_text(Status::failure(trajectory.error.code,
                                                   trajectory.error.detail));
    return latch_locked(trajectory.error.code, trajectory.error.detail);
  }
  const auto maps = session_->take_map_chunks();
  if (!maps) {
    response.success = false;
    response.message = error_text(Status::failure(maps.error.code,
                                                   maps.error.detail));
    return latch_locked(maps.error.code, maps.error.detail);
  }
  const auto published = publish_outputs_locked(trajectory.value, maps.value);
  if (!published) {
    response.success = false;
    response.message = error_text(published);
    return published;
  }
  response.success = true;
  response.message = "complete trajectory and map published exactly once";
  return Status::success();
}

void GlimPhase3dNode::on_lidar_message(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
  if (!message) {
    std::lock_guard<std::mutex> lock(mutex_);
    (void)latch_locked(ErrorCode::kInvalidLayout,
                       "ROS LiDAR callback received a null message");
    return;
  }
  try {
    const auto status = enqueue_lidar(*message);
    if (!status) RCLCPP_ERROR(get_logger(), "%s", error_text(status).c_str());
  } catch (const std::exception& exception) {
    std::lock_guard<std::mutex> lock(mutex_);
    (void)latch_locked(ErrorCode::kCoreFailure,
                       std::string("ROS LiDAR callback threw: ") + exception.what());
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    (void)latch_locked(ErrorCode::kCoreFailure,
                       "ROS LiDAR callback threw an unknown exception");
  }
}

void GlimPhase3dNode::on_imu_message(sensor_msgs::msg::Imu::ConstSharedPtr message) {
  if (!message) {
    std::lock_guard<std::mutex> lock(mutex_);
    (void)latch_locked(ErrorCode::kInvalidLayout,
                       "ROS IMU callback received a null message");
    return;
  }
  try {
    const auto status = enqueue_imu(*message);
    if (!status) RCLCPP_ERROR(get_logger(), "%s", error_text(status).c_str());
  } catch (const std::exception& exception) {
    std::lock_guard<std::mutex> lock(mutex_);
    (void)latch_locked(ErrorCode::kCoreFailure,
                       std::string("ROS IMU callback threw: ") + exception.what());
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    (void)latch_locked(ErrorCode::kCoreFailure,
                       "ROS IMU callback threw an unknown exception");
  }
}

void GlimPhase3dNode::on_finalize(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  static_cast<void>(request);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto status = finalize_locked(*response);
  if (!status && response->message.empty()) response->message = error_text(status);
}

}  // namespace glim_clean_room::phase3d
