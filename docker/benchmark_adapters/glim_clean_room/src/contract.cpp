#include "glim_clean_room/contract.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <exception>
#include <sstream>

namespace glim_clean_room {
namespace {

constexpr double kCalibrationTolerance = 1e-9;
constexpr double kQuaternionTolerance = 1e-6;
constexpr StampNanoseconds kNanosecondsPerSecond = 1000000000;

bool finite(double value) { return std::isfinite(value); }

bool floating_field(PointFieldType type) {
  return type == PointFieldType::kFloat32 ||
         type == PointFieldType::kFloat64;
}

std::size_t field_width(PointFieldType type) {
  switch (type) {
    case PointFieldType::kUInt16:
      return 2;
    case PointFieldType::kUInt32:
    case PointFieldType::kFloat32:
      return 4;
    case PointFieldType::kFloat64:
      return 8;
  }
  return 0;
}

bool forbidden_segment(std::string_view segment) {
  std::string lowered(segment);
  std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                 [](unsigned char value) {
                   return static_cast<char>(std::tolower(value));
                 });
  return lowered == "gt" || lowered == "ground_truth" ||
         lowered == "ground-truth" || lowered == "scorer" ||
         lowered == "score" || lowered == "metrics" ||
         lowered == "metric";
}

bool absolute_path(std::string_view path) {
  return !path.empty() && (path.front() == '/' || path.front() == '\\' ||
                           (path.size() > 1 && path[1] == ':'));
}

bool path_has_forbidden_segment(std::string_view path) {
  std::string normalized(path);
  std::replace(normalized.begin(), normalized.end(), '\\', '/');
  std::size_t begin = 0;
  while (begin <= normalized.size()) {
    const auto end = normalized.find('/', begin);
    const auto segment = std::string_view(normalized).substr(
        begin, end == std::string::npos ? normalized.size() - begin
                                       : end - begin);
    if (forbidden_segment(segment)) {
      return true;
    }
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1;
  }
  return false;
}

bool near(double lhs, double rhs, double tolerance = kCalibrationTolerance) {
  return std::abs(lhs - rhs) <= tolerance;
}

Status validate_homogeneous_transform(const std::array<double, 16>& matrix,
                                      const char* name) {
  for (double value : matrix) {
    if (!finite(value)) {
      return Status::failure(ErrorCode::kInvalidCalibration,
                             std::string(name) + " contains a non-finite value");
    }
  }
  if (!near(matrix[12], 0.0) || !near(matrix[13], 0.0) ||
      !near(matrix[14], 0.0) || !near(matrix[15], 1.0)) {
    return Status::failure(ErrorCode::kInvalidCalibration,
                           std::string(name) + " has an invalid homogeneous last row");
  }

  for (std::size_t row = 0; row < 3; ++row) {
    double norm = 0.0;
    for (std::size_t column = 0; column < 3; ++column) {
      norm += matrix[row * 4 + column] * matrix[row * 4 + column];
    }
    if (!near(norm, 1.0)) {
      return Status::failure(ErrorCode::kInvalidCalibration,
                             std::string(name) + " rotation is not unit length");
    }
    for (std::size_t other = row + 1; other < 3; ++other) {
      double dot = 0.0;
      for (std::size_t column = 0; column < 3; ++column) {
        dot += matrix[row * 4 + column] * matrix[other * 4 + column];
      }
      if (!near(dot, 0.0)) {
        return Status::failure(
            ErrorCode::kInvalidCalibration,
            std::string(name) + " rotation is not orthonormal");
      }
    }
  }
  const double determinant =
      matrix[0] * (matrix[5] * matrix[10] - matrix[6] * matrix[9]) -
      matrix[1] * (matrix[4] * matrix[10] - matrix[6] * matrix[8]) +
      matrix[2] * (matrix[4] * matrix[9] - matrix[5] * matrix[8]);
  if (!near(determinant, 1.0)) {
    return Status::failure(ErrorCode::kInvalidCalibration,
                           std::string(name) + " rotation determinant is not +1");
  }
  return Status::success();
}

bool transforms_are_inverses(const std::array<double, 16>& lhs,
                             const std::array<double, 16>& rhs) {
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t column = 0; column < 4; ++column) {
      double value = 0.0;
      for (std::size_t k = 0; k < 4; ++k) {
        value += lhs[row * 4 + k] * rhs[k * 4 + column];
      }
      if (!near(value, row == column ? 1.0 : 0.0)) {
        return false;
      }
    }
  }
  return true;
}

bool inverse_convention(CalibrationConvention lhs,
                        CalibrationConvention rhs) {
  return (lhs == CalibrationConvention::kTImuLidar &&
          rhs == CalibrationConvention::kTLidarImu) ||
         (lhs == CalibrationConvention::kTLidarImu &&
          rhs == CalibrationConvention::kTImuLidar);
}

bool quaternion_is_unit(const std::array<double, 7>& pose) {
  const double norm = std::sqrt(pose[3] * pose[3] + pose[4] * pose[4] +
                                pose[5] * pose[5] + pose[6] * pose[6]);
  return finite(norm) && std::abs(norm - 1.0) <= kQuaternionTolerance;
}

bool point_is_finite_and_nonnegative(const PointRecord& point) {
  return finite(point.x) && finite(point.y) && finite(point.z) &&
         finite(point.intensity) && finite(point.relative_time_raw) &&
         finite(point.relative_time) && point.relative_time_raw >= 0.0 &&
         point.relative_time >= 0.0;
}

}  // namespace

const char* error_code_name(ErrorCode code) noexcept {
  switch (code) {
    case ErrorCode::kNone: return "none";
    case ErrorCode::kMissingField: return "missing_field";
    case ErrorCode::kDuplicateField: return "duplicate_field";
    case ErrorCode::kInvalidLayout: return "invalid_layout";
    case ErrorCode::kUnsupportedFieldType: return "unsupported_field_type";
    case ErrorCode::kWrongTimeUnit: return "wrong_time_unit";
    case ErrorCode::kInvalidStamp: return "invalid_stamp";
    case ErrorCode::kEmptyPayload: return "empty_payload";
    case ErrorCode::kNonFinite: return "non_finite";
    case ErrorCode::kNonMonotonicStamp: return "non_monotonic_stamp";
    case ErrorCode::kOrderViolation: return "order_violation";
    case ErrorCode::kMissingCalibration: return "missing_calibration";
    case ErrorCode::kInvalidCalibration: return "invalid_calibration";
    case ErrorCode::kForbiddenArtifactRole: return "forbidden_artifact_role";
    case ErrorCode::kForbiddenArtifactPath: return "forbidden_artifact_path";
    case ErrorCode::kInvalidArtifactPath: return "invalid_artifact_path";
    case ErrorCode::kInvalidState: return "invalid_state";
    case ErrorCode::kPostEof: return "post_eof";
    case ErrorCode::kOutputOrderViolation: return "output_order_violation";
    case ErrorCode::kCoreFailure: return "core_failure";
    case ErrorCode::kCounterOverflow: return "counter_overflow";
    case ErrorCode::kStampLedgerAmbiguous: return "stamp_ledger_ambiguous";
    case ErrorCode::kStampLedgerUnmatched: return "stamp_ledger_unmatched";
    case ErrorCode::kStampLedgerReused: return "stamp_ledger_reused";
    case ErrorCode::kLedgerOverflow: return "stamp_ledger_overflow";
    case ErrorCode::kUnsupportedCoreOutput: return "unsupported_core_output";
    case ErrorCode::kNotImplemented: return "not_implemented";
  }
  return "unknown";
}

double point_time_to_seconds(double value, PointTimeUnit unit) noexcept {
  switch (unit) {
    case PointTimeUnit::kSeconds: return value;
    case PointTimeUnit::kMilliseconds: return value * 1e-3;
    case PointTimeUnit::kMicroseconds: return value * 1e-6;
    case PointTimeUnit::kNanoseconds: return value * 1e-9;
    case PointTimeUnit::kUnspecified:
      return std::numeric_limits<double>::quiet_NaN();
  }
  return std::numeric_limits<double>::quiet_NaN();
}

Status validate_point_time_unit(PointTimeUnit unit) {
  if (unit == PointTimeUnit::kUnspecified) {
    return Status::failure(ErrorCode::kWrongTimeUnit,
                           "point relative-time unit must be explicit");
  }
  return Status::success();
}

bool operator==(const PointFieldSpec& lhs, const PointFieldSpec& rhs) noexcept {
  return lhs.name == rhs.name && lhs.datatype == rhs.datatype &&
         lhs.count == rhs.count;
}

bool operator==(const PointFieldMapping& lhs,
                const PointFieldMapping& rhs) noexcept {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z &&
         lhs.intensity == rhs.intensity && lhs.ring == rhs.ring &&
         lhs.relative_time == rhs.relative_time &&
         lhs.time_unit == rhs.time_unit;
}

Status validate_point_field_mapping(const PointFieldMapping& mapping) {
  const auto unit = validate_point_time_unit(mapping.time_unit);
  if (!unit) return unit;
  const PointFieldSpec specs[] = {mapping.x,       mapping.y,
                                  mapping.z,       mapping.intensity,
                                  mapping.ring,     mapping.relative_time};
  const char* labels[] = {"x", "y", "z", "intensity", "ring", "relative_time"};
  for (std::size_t i = 0; i < 6; ++i) {
    if (specs[i].name.empty() || specs[i].count != 1 ||
        field_width(specs[i].datatype) == 0) {
      return Status::failure(
          ErrorCode::kInvalidLayout,
          std::string("sequence field mapping is incomplete for ") + labels[i]);
    }
    for (std::size_t prior = 0; prior < i; ++prior) {
      if (specs[i].name == specs[prior].name) {
        return Status::failure(ErrorCode::kDuplicateField,
                               "sequence field mapping contains duplicate names");
      }
    }
  }
  if (!floating_field(mapping.x.datatype) ||
      !floating_field(mapping.y.datatype) ||
      !floating_field(mapping.z.datatype) ||
      !floating_field(mapping.intensity.datatype) ||
      !floating_field(mapping.relative_time.datatype) ||
      (mapping.ring.datatype != PointFieldType::kUInt16 &&
       mapping.ring.datatype != PointFieldType::kUInt32)) {
    return Status::failure(
        ErrorCode::kUnsupportedFieldType,
        "sequence field mapping has an unsupported semantic datatype");
  }
  return Status::success();
}

Result<EventStamp> event_stamp_from_ros(std::uint64_t order,
                                        std::int64_t sec,
                                        std::int64_t nanosec) {
  if (nanosec < 0 || nanosec >= kNanosecondsPerSecond) {
    return Result<EventStamp>::failure(
        ErrorCode::kInvalidStamp,
        "ROS nanosecond component must be in [0, 1e9)");
  }
  const auto max_value = std::numeric_limits<StampNanoseconds>::max();
  const auto min_value = std::numeric_limits<StampNanoseconds>::min();
  const auto max_seconds = max_value / kNanosecondsPerSecond;
  const auto min_seconds = min_value / kNanosecondsPerSecond;
  if (sec > max_seconds || sec < min_seconds ||
      (sec == max_seconds && nanosec > max_value % kNanosecondsPerSecond)) {
    return Result<EventStamp>::failure(
        ErrorCode::kInvalidStamp,
        "ROS sec/nanosec would overflow signed int64 nanoseconds");
  }
  // For the negative boundary, truncation toward zero leaves enough margin
  // for the non-negative nanosecond component after the lower-bound check.
  const StampNanoseconds total =
      sec * kNanosecondsPerSecond + nanosec;
  return Result<EventStamp>::success(EventStamp{order, total});
}

double event_stamp_to_seconds(const EventStamp& stamp) noexcept {
  return static_cast<double>(stamp.nanoseconds) * 1e-9;
}

Status validate_artifact_references(
    const std::vector<ArtifactReference>& artifacts) {
  for (const auto& artifact : artifacts) {
    if (artifact.logical_path.empty() || absolute_path(artifact.logical_path) ||
        artifact.logical_path.find("..") != std::string::npos) {
      return Status::failure(ErrorCode::kInvalidArtifactPath,
                             "artifact path must be relative and traversal-free");
    }
    if (path_has_forbidden_segment(artifact.logical_path)) {
      return Status::failure(
          ErrorCode::kForbiddenArtifactPath,
          "ground-truth/scorer paths are forbidden at the adapter boundary");
    }
    if (artifact.role == ArtifactRole::kGroundTruth ||
        artifact.role == ArtifactRole::kScorer ||
        artifact.role == ArtifactRole::kUnknown) {
      return Status::failure(
          ErrorCode::kForbiddenArtifactRole,
          "adapter artifacts may not include GT or scorer roles");
    }
  }
  return Status::success();
}

Status validate_calibration(const Calibration& calibration) {
  if (calibration.convention == CalibrationConvention::kUnspecified ||
      calibration.inverse_convention == CalibrationConvention::kUnspecified ||
      !inverse_convention(calibration.convention,
                          calibration.inverse_convention) ||
      calibration.imu_frame.empty() || calibration.lidar_frame.empty() ||
      calibration.imu_frame == calibration.lidar_frame) {
    return Status::failure(
        ErrorCode::kMissingCalibration,
        "calibration frames and explicit forward/inverse conventions are required");
  }
  const auto forward = validate_homogeneous_transform(calibration.transform,
                                                      "calibration transform");
  if (!forward) return forward;
  const auto inverse = validate_homogeneous_transform(
      calibration.inverse_transform, "calibration inverse transform");
  if (!inverse) return inverse;
  if (!transforms_are_inverses(calibration.transform,
                               calibration.inverse_transform) ||
      !transforms_are_inverses(calibration.inverse_transform,
                               calibration.transform)) {
    return Status::failure(
        ErrorCode::kInvalidCalibration,
        "calibration transform and inverse do not multiply to identity");
  }
  return Status::success();
}

Status validate_lidar_frame(const LidarFrame& frame,
                            PointTimeUnit expected_time_unit,
                            const PointFieldMapping& expected_mapping,
                            std::string_view expected_frame) {
  const auto unit = validate_point_time_unit(expected_time_unit);
  if (!unit) return unit;
  const auto mapping = validate_point_field_mapping(expected_mapping);
  if (!mapping) return mapping;
  if (expected_frame.empty() || frame.frame_id != expected_frame) {
    return Status::failure(ErrorCode::kInvalidLayout,
                           "LiDAR frame ID differs from calibration contract");
  }
  if (frame.point_time_unit != expected_time_unit ||
      !(frame.field_mapping == expected_mapping)) {
    return Status::failure(
        ErrorCode::kWrongTimeUnit,
        "LiDAR field mapping or time unit differs from the sequence contract");
  }
  if (frame.points.empty()) {
    return Status::failure(ErrorCode::kEmptyPayload,
                           "LiDAR frame requires at least one point");
  }
  for (std::size_t i = 0; i < frame.points.size(); ++i) {
    const auto& point = frame.points[i];
    if (!point_is_finite_and_nonnegative(point)) {
      return Status::failure(ErrorCode::kNonFinite,
                             "LiDAR frame contains an invalid point at index " +
                                 std::to_string(i));
    }
  }
  return Status::success();
}

Status validate_imu_sample(const ImuSample& sample,
                           std::string_view expected_frame) {
  if (expected_frame.empty() || sample.frame_id != expected_frame ||
      sample.acceleration_unit != AccelerationUnit::kMetersPerSecondSquared ||
      sample.angular_velocity_unit != AngularVelocityUnit::kRadiansPerSecond) {
    return Status::failure(
        ErrorCode::kInvalidLayout,
        "IMU frame ID and SI units must match the calibration contract");
  }
  for (double value : sample.linear_acceleration) {
    if (!finite(value)) {
      return Status::failure(ErrorCode::kNonFinite,
                             "IMU acceleration contains a non-finite value");
    }
  }
  for (double value : sample.angular_velocity) {
    if (!finite(value)) {
      return Status::failure(ErrorCode::kNonFinite,
                             "IMU angular velocity contains a non-finite value");
    }
  }
  return Status::success();
}

Status validate_sequence_contract(const SequenceContract& contract) {
  if (contract.sequence_id.empty()) {
    return Status::failure(ErrorCode::kInvalidLayout,
                           "sequence ID must be explicit");
  }
  const auto unit = validate_point_time_unit(contract.point_time_unit);
  if (!unit) return unit;
  const auto calibration = validate_calibration(contract.calibration);
  if (!calibration) return calibration;
  const auto mapping = validate_point_field_mapping(contract.point_fields);
  if (!mapping) return mapping;
  if (contract.point_fields.time_unit != contract.point_time_unit) {
    return Status::failure(
        ErrorCode::kWrongTimeUnit,
        "sequence point-field mapping unit differs from sequence unit");
  }
  if (contract.trajectory_frame.empty() || contract.map_frame.empty()) {
    return Status::failure(
        ErrorCode::kInvalidLayout,
        "trajectory and map output frame conventions must be explicit");
  }
  return validate_artifact_references(contract.artifacts);
}

Status validate_trajectory_sample(const TrajectorySample& sample) {
  if (sample.frame_id.empty()) {
    return Status::failure(ErrorCode::kInvalidLayout,
                           "trajectory requires a frame ID");
  }
  for (double value : sample.pose) {
    if (!finite(value)) {
      return Status::failure(ErrorCode::kNonFinite,
                             "trajectory contains a non-finite pose value");
    }
  }
  if (!quaternion_is_unit(sample.pose)) {
    return Status::failure(ErrorCode::kInvalidLayout,
                           "trajectory quaternion is not finite and unit norm");
  }
  return Status::success();
}

Status validate_map_chunk(const MapChunk& chunk) {
  if (chunk.frame_id.empty() || chunk.points.empty()) {
    return Status::failure(ErrorCode::kEmptyPayload,
                           "map chunk requires a frame ID and points");
  }
  for (const auto& point : chunk.points) {
    if (!point_is_finite_and_nonnegative(point)) {
      return Status::failure(ErrorCode::kNonFinite,
                             "map chunk contains an invalid point");
    }
  }
  return Status::success();
}

AdapterBoundary::AdapterBoundary(SequenceContract contract, CoreSink& sink)
    : contract_(std::move(contract)), sink_(sink) {
  const auto status = validate_sequence_contract(contract_);
  if (!status) {
    state_ = BoundaryState::kFailed;
    terminal_error_ = status.error;
    counters_.failures = 1;
  }
}

Status AdapterBoundary::reject(ErrorCode code, const std::string& detail,
                               std::uint64_t* rejected_counter) {
  const bool failure_has_capacity =
      counters_.failures != std::numeric_limits<std::uint64_t>::max();
  const bool rejected_has_capacity =
      rejected_counter == nullptr ||
      *rejected_counter != std::numeric_limits<std::uint64_t>::max();
  if (!failure_has_capacity || !rejected_has_capacity) {
    // Counter overflow has precedence over the semantic error.  Neither
    // counter is partially updated; the latch is still terminal even when the
    // failures counter itself is already saturated.
    const std::string overflow_detail =
        "counter overflow while rejecting: " + detail;
    state_ = BoundaryState::kFailed;
    if (terminal_error_.code == ErrorCode::kNone) {
      terminal_error_ = Error{ErrorCode::kCounterOverflow, overflow_detail};
    }
    return Status::failure(ErrorCode::kCounterOverflow, overflow_detail);
  }
  const auto counted = rejected_counter == nullptr
                           ? increment_counters({CounterSlot{
                                 &counters_.failures, "failures"}})
                           : increment_counters(
                                 {CounterSlot{&counters_.failures, "failures"},
                                  CounterSlot{rejected_counter, "rejected"}});
  if (!counted) {
    const std::string overflow_detail =
        "counter overflow while rejecting: " + detail;
    state_ = BoundaryState::kFailed;
    if (terminal_error_.code == ErrorCode::kNone) {
      terminal_error_ = Error{ErrorCode::kCounterOverflow, overflow_detail};
    }
    return Status::failure(ErrorCode::kCounterOverflow, overflow_detail);
  }
  if (state_ != BoundaryState::kFailed) {
    state_ = BoundaryState::kFailed;
    terminal_error_ = Error{code, detail};
  }
  return Status::failure(code, detail);
}

Status AdapterBoundary::preflight_counter_capacity(
    std::initializer_list<CounterSlot> counters) const {
  for (const auto& counter : counters) {
    if (counter.value == nullptr ||
        *counter.value == std::numeric_limits<std::uint64_t>::max()) {
      const std::string name = counter.name == nullptr ? "unknown" : counter.name;
      return Status::failure(ErrorCode::kCounterOverflow,
                             "counter has no increment capacity: " + name);
    }
  }
  return Status::success();
}

Status AdapterBoundary::increment_counters(
    std::initializer_list<CounterSlot> counters) {
  const auto capacity = preflight_counter_capacity(counters);
  if (!capacity) return capacity;
  for (const auto& counter : counters) {
    ++*counter.value;
  }
  return Status::success();
}

Status AdapterBoundary::increment_counter(std::uint64_t& counter,
                                           const char* name) {
  const auto status = increment_counters({CounterSlot{&counter, name}});
  if (!status) {
    return reject(ErrorCode::kCounterOverflow, status.error.detail);
  }
  return status;
}

Status AdapterBoundary::core_call(const char* operation,
                                   const std::function<Status()>& call,
                                   std::uint64_t* rejected_counter) {
  try {
    const auto status = call();
    if (!status) {
      // A nested transactional operation may already have latched the
      // boundary and counted its failure.  Preserve that first terminal
      // reason instead of incrementing failures a second time while wrapping
      // it as a generic core error.
      if (state_ == BoundaryState::kFailed &&
          terminal_error_.code != ErrorCode::kNone) {
        return Status::failure(terminal_error_.code, terminal_error_.detail);
      }
      std::ostringstream detail;
      detail << operation << " returned " << error_code_name(status.error.code);
      if (!status.error.detail.empty()) {
        detail << ": " << status.error.detail;
      }
      return reject(ErrorCode::kCoreFailure, detail.str(), rejected_counter);
    }
    return Status::success();
  } catch (const std::exception& exception) {
    return reject(ErrorCode::kCoreFailure,
                  std::string(operation) + " threw: " + exception.what(),
                  rejected_counter);
  } catch (...) {
    return reject(ErrorCode::kCoreFailure,
                  std::string(operation) + " threw an unknown exception",
                  rejected_counter);
  }
}

Status AdapterBoundary::accept_event(const EventStamp& event) {
  if (state_ != BoundaryState::kOpen) {
    return Status::failure(ErrorCode::kPostEof,
                           "input was submitted after EOF or terminal failure");
  }
  if (event.order != next_event_order_) {
    return Status::failure(
        ErrorCode::kOrderViolation,
        "input order is not the preregistered contiguous order");
  }
  if (have_event_stamp_ && event.nanoseconds < last_event_nanoseconds_) {
    return Status::failure(
        ErrorCode::kNonMonotonicStamp,
        "input timestamp regressed; adapter will not reorder it");
  }
  const auto capacity = preflight_counter_capacity(
      {CounterSlot{&next_event_order_, "input_event_order"}});
  if (!capacity) {
    return capacity;
  }
  const auto incremented = increment_counters(
      {CounterSlot{&next_event_order_, "input_event_order"}});
  if (!incremented) return incremented;
  last_event_nanoseconds_ = event.nanoseconds;
  have_event_stamp_ = true;
  return Status::success();
}

Status AdapterBoundary::submit_lidar(const LidarFrame& frame) {
  const auto submitted_capacity = preflight_counter_capacity(
      {CounterSlot{&counters_.lidar_submitted, "lidar_submitted"},
       CounterSlot{&counters_.lidar_rejected, "lidar_rejected"},
       CounterSlot{&counters_.failures, "failures"}});
  if (!submitted_capacity) {
    return reject(ErrorCode::kCounterOverflow, submitted_capacity.error.detail,
                  &counters_.lidar_rejected);
  }
  const auto submitted = increment_counters(
      {CounterSlot{&counters_.lidar_submitted, "lidar_submitted"}});
  if (!submitted) {
    return reject(ErrorCode::kCounterOverflow, submitted.error.detail,
                  &counters_.lidar_rejected);
  }
  const auto valid = validate_lidar_frame(
      frame, contract_.point_time_unit, contract_.point_fields,
      contract_.calibration.lidar_frame);
  if (!valid) {
    return reject(valid.error.code, valid.error.detail,
                  &counters_.lidar_rejected);
  }
  const auto event_capacity = preflight_counter_capacity(
      {CounterSlot{&next_event_order_, "input_event_order"},
       CounterSlot{&counters_.lidar_accepted, "lidar_accepted"},
       CounterSlot{&counters_.lidar_received, "lidar_received"},
       CounterSlot{&counters_.lidar_processed, "lidar_processed"},
       CounterSlot{&counters_.lidar_rejected, "lidar_rejected"},
       CounterSlot{&counters_.failures, "failures"}});
  if (!event_capacity) {
    return reject(ErrorCode::kCounterOverflow, event_capacity.error.detail,
                  &counters_.lidar_rejected);
  }
  const auto event = accept_event(frame.event);
  if (!event) {
    return reject(event.error.code, event.error.detail,
                  &counters_.lidar_rejected);
  }
  const auto accepted = increment_counters(
      {CounterSlot{&counters_.lidar_accepted, "lidar_accepted"},
       CounterSlot{&counters_.lidar_received, "lidar_received"}});
  if (!accepted) {
    return reject(ErrorCode::kCounterOverflow, accepted.error.detail,
                  &counters_.lidar_rejected);
  }
  const auto sink_status = core_call(
      "on_lidar", [this, &frame]() { return sink_.on_lidar(frame); },
      &counters_.lidar_rejected);
  if (!sink_status) {
    return sink_status;
  }
  const auto processed = increment_counters(
      {CounterSlot{&counters_.lidar_processed, "lidar_processed"}});
  if (!processed) {
    return reject(ErrorCode::kCounterOverflow, processed.error.detail,
                  &counters_.lidar_rejected);
  }
  return Status::success();
}

Status AdapterBoundary::submit_imu(const ImuSample& sample) {
  const auto submitted_capacity = preflight_counter_capacity(
      {CounterSlot{&counters_.imu_submitted, "imu_submitted"},
       CounterSlot{&counters_.imu_rejected, "imu_rejected"},
       CounterSlot{&counters_.failures, "failures"}});
  if (!submitted_capacity) {
    return reject(ErrorCode::kCounterOverflow, submitted_capacity.error.detail,
                  &counters_.imu_rejected);
  }
  const auto submitted = increment_counters(
      {CounterSlot{&counters_.imu_submitted, "imu_submitted"}});
  if (!submitted) {
    return reject(ErrorCode::kCounterOverflow, submitted.error.detail,
                  &counters_.imu_rejected);
  }
  const auto valid = validate_imu_sample(sample,
                                         contract_.calibration.imu_frame);
  if (!valid) {
    return reject(valid.error.code, valid.error.detail,
                  &counters_.imu_rejected);
  }
  const auto event_capacity = preflight_counter_capacity(
      {CounterSlot{&next_event_order_, "input_event_order"},
       CounterSlot{&counters_.imu_accepted, "imu_accepted"},
       CounterSlot{&counters_.imu_received, "imu_received"},
       CounterSlot{&counters_.imu_processed, "imu_processed"},
       CounterSlot{&counters_.imu_rejected, "imu_rejected"},
       CounterSlot{&counters_.failures, "failures"}});
  if (!event_capacity) {
    return reject(ErrorCode::kCounterOverflow, event_capacity.error.detail,
                  &counters_.imu_rejected);
  }
  const auto event = accept_event(sample.event);
  if (!event) {
    return reject(event.error.code, event.error.detail,
                  &counters_.imu_rejected);
  }
  const auto accepted = increment_counters(
      {CounterSlot{&counters_.imu_accepted, "imu_accepted"},
       CounterSlot{&counters_.imu_received, "imu_received"}});
  if (!accepted) {
    return reject(ErrorCode::kCounterOverflow, accepted.error.detail,
                  &counters_.imu_rejected);
  }
  const auto sink_status = core_call(
      "on_imu", [this, &sample]() { return sink_.on_imu(sample); },
      &counters_.imu_rejected);
  if (!sink_status) {
    return sink_status;
  }
  const auto processed = increment_counters(
      {CounterSlot{&counters_.imu_processed, "imu_processed"}});
  if (!processed) {
    return reject(ErrorCode::kCounterOverflow, processed.error.detail,
                  &counters_.imu_rejected);
  }
  return Status::success();
}

Status AdapterBoundary::request_eof() {
  if (state_ != BoundaryState::kOpen) {
    return reject(ErrorCode::kInvalidState,
                  "EOF must be requested exactly once while open");
  }
  const auto capacity = preflight_counter_capacity(
      {CounterSlot{&counters_.eof_requests, "eof_requests"},
       CounterSlot{&counters_.failures, "failures"}});
  if (!capacity) {
    return reject(ErrorCode::kCounterOverflow, capacity.error.detail);
  }
  const auto status = core_call("on_eof", [this]() { return sink_.on_eof(); });
  if (!status) return status;
  const auto counted = increment_counters(
      {CounterSlot{&counters_.eof_requests, "eof_requests"}});
  if (!counted) return reject(ErrorCode::kCounterOverflow, counted.error.detail);
  state_ = BoundaryState::kEofRequested;
  return Status::success();
}

Status AdapterBoundary::begin_drain() {
  if (state_ != BoundaryState::kEofRequested) {
    return reject(ErrorCode::kInvalidState,
                  "drain must begin after exactly one EOF request");
  }
  const auto capacity = preflight_counter_capacity(
      {CounterSlot{&counters_.failures, "failures"}});
  if (!capacity) {
    return reject(ErrorCode::kCounterOverflow, capacity.error.detail);
  }
  // Publish the Draining state before invoking the sink.  A sink may need to
  // validate/seal staged output during its begin-drain callback; doing this
  // after the callback made an otherwise valid transactional batch appear to
  // be an out-of-phase publication.  core_call still latches a callback
  // failure terminally, so no failed callback can leave the boundary draining.
  state_ = BoundaryState::kDraining;
  return core_call("on_begin_drain",
                   [this]() { return sink_.on_begin_drain(); });
}

Status AdapterBoundary::record_trajectory(const TrajectorySample& sample) {
  const auto submitted_capacity = preflight_counter_capacity(
      {CounterSlot{&counters_.trajectory_submitted, "trajectory_submitted"},
       CounterSlot{&counters_.trajectory_rejected, "trajectory_rejected"},
       CounterSlot{&counters_.failures, "failures"}});
  if (!submitted_capacity) {
    return reject(ErrorCode::kCounterOverflow, submitted_capacity.error.detail,
                  &counters_.trajectory_rejected);
  }
  const auto submitted = increment_counters(
      {CounterSlot{&counters_.trajectory_submitted, "trajectory_submitted"}});
  if (!submitted) {
    return reject(ErrorCode::kCounterOverflow, submitted.error.detail,
                  &counters_.trajectory_rejected);
  }
  if (state_ != BoundaryState::kDraining) {
    return reject(ErrorCode::kInvalidState,
                  "trajectory output is allowed only during the drain phase",
                  &counters_.trajectory_rejected);
  }
  const auto valid = validate_trajectory_sample(sample);
  if (!valid) {
    return reject(valid.error.code, valid.error.detail,
                  &counters_.trajectory_rejected);
  }
  if (sample.frame_id != contract_.trajectory_frame) {
    return reject(ErrorCode::kInvalidLayout,
                  "trajectory frame differs from the sequence output convention",
                  &counters_.trajectory_rejected);
  }
  if (sample.event.order != next_trajectory_order_) {
    return reject(ErrorCode::kOutputOrderViolation,
                  "trajectory output order is not contiguous from zero",
                  &counters_.trajectory_rejected);
  }
  if (have_trajectory_stamp_ &&
      sample.event.nanoseconds < last_trajectory_nanoseconds_) {
    return reject(ErrorCode::kNonMonotonicStamp,
                  "trajectory output timestamp regressed",
                  &counters_.trajectory_rejected);
  }
  const auto capacity = preflight_counter_capacity(
      {CounterSlot{&next_trajectory_order_, "trajectory_order"},
       CounterSlot{&counters_.trajectory_accepted, "trajectory_accepted"},
       CounterSlot{&counters_.trajectory_processed, "trajectory_processed"},
       CounterSlot{&counters_.trajectory_outputs, "trajectory_outputs"},
       CounterSlot{&counters_.trajectory_rejected, "trajectory_rejected"},
       CounterSlot{&counters_.failures, "failures"}});
  if (!capacity) {
    return reject(ErrorCode::kCounterOverflow,
                  capacity.error.detail, &counters_.trajectory_rejected);
  }
  const auto counted = increment_counters(
      {CounterSlot{&next_trajectory_order_, "trajectory_order"},
       CounterSlot{&counters_.trajectory_accepted, "trajectory_accepted"},
       CounterSlot{&counters_.trajectory_processed, "trajectory_processed"},
       CounterSlot{&counters_.trajectory_outputs, "trajectory_outputs"}});
  if (!counted) {
    return reject(ErrorCode::kCounterOverflow, counted.error.detail,
                  &counters_.trajectory_rejected);
  }
  last_trajectory_nanoseconds_ = sample.event.nanoseconds;
  have_trajectory_stamp_ = true;
  return Status::success();
}

Status AdapterBoundary::record_map_chunk(const MapChunk& chunk) {
  const auto submitted_capacity = preflight_counter_capacity(
      {CounterSlot{&counters_.map_submitted, "map_submitted"},
       CounterSlot{&counters_.map_rejected, "map_rejected"},
       CounterSlot{&counters_.failures, "failures"}});
  if (!submitted_capacity) {
    return reject(ErrorCode::kCounterOverflow, submitted_capacity.error.detail,
                  &counters_.map_rejected);
  }
  const auto submitted = increment_counters(
      {CounterSlot{&counters_.map_submitted, "map_submitted"}});
  if (!submitted) {
    return reject(ErrorCode::kCounterOverflow, submitted.error.detail,
                  &counters_.map_rejected);
  }
  if (state_ != BoundaryState::kDraining) {
    return reject(ErrorCode::kInvalidState,
                  "map output is allowed only during the drain phase",
                  &counters_.map_rejected);
  }
  const auto valid = validate_map_chunk(chunk);
  if (!valid) {
    return reject(valid.error.code, valid.error.detail,
                  &counters_.map_rejected);
  }
  if (chunk.frame_id != contract_.map_frame) {
    return reject(ErrorCode::kInvalidLayout,
                  "map frame differs from the sequence output convention",
                  &counters_.map_rejected);
  }
  if (chunk.order != next_map_order_) {
    return reject(ErrorCode::kOutputOrderViolation,
                  "map output order is not contiguous from zero",
                  &counters_.map_rejected);
  }
  const auto capacity = preflight_counter_capacity(
      {CounterSlot{&next_map_order_, "map_order"},
       CounterSlot{&counters_.map_accepted, "map_accepted"},
       CounterSlot{&counters_.map_processed, "map_processed"},
       CounterSlot{&counters_.map_outputs, "map_outputs"},
       CounterSlot{&counters_.map_rejected, "map_rejected"},
       CounterSlot{&counters_.failures, "failures"}});
  if (!capacity) {
    return reject(ErrorCode::kCounterOverflow,
                  capacity.error.detail, &counters_.map_rejected);
  }
  const auto counted = increment_counters(
      {CounterSlot{&next_map_order_, "map_order"},
       CounterSlot{&counters_.map_accepted, "map_accepted"},
       CounterSlot{&counters_.map_processed, "map_processed"},
       CounterSlot{&counters_.map_outputs, "map_outputs"}});
  if (!counted) {
    return reject(ErrorCode::kCounterOverflow, counted.error.detail,
                  &counters_.map_rejected);
  }
  return Status::success();
}

Status AdapterBoundary::record_trajectory_batch(
    const std::vector<TrajectorySample>& samples) {
  return record_output_batch(samples, {});
}

Status AdapterBoundary::record_output_batch(
    const std::vector<TrajectorySample>& trajectories,
    const std::vector<MapChunk>& maps) {
  if (state_ != BoundaryState::kDraining) {
    return reject(ErrorCode::kInvalidState,
                  "output batch is allowed only during the drain phase");
  }
  if (trajectories.empty() && maps.empty()) {
    return reject(ErrorCode::kInvalidState,
                  "output batch must contain at least one output");
  }
  if (!maps.empty() && !contract_.require_map_output) {
    return reject(ErrorCode::kInvalidState,
                  "map output is not enabled by the sequence contract");
  }

  const auto max_value = std::numeric_limits<std::uint64_t>::max();
  const auto as_counter_count = [](std::size_t count,
                                   const char* name)
      -> Result<std::uint64_t> {
    if (count > std::numeric_limits<std::uint64_t>::max()) {
      return Result<std::uint64_t>::failure(
          ErrorCode::kCounterOverflow,
          std::string(name) + " batch size exceeds uint64 counter range");
    }
    return Result<std::uint64_t>::success(static_cast<std::uint64_t>(count));
  };
  const auto trajectory_count = as_counter_count(trajectories.size(),
                                                 "trajectory");
  if (!trajectory_count) {
    return reject(trajectory_count.error.code, trajectory_count.error.detail);
  }
  const auto map_count = as_counter_count(maps.size(), "map");
  if (!map_count) {
    return reject(map_count.error.code, map_count.error.detail);
  }

  const auto has_capacity = [max_value](std::uint64_t value,
                                         std::uint64_t increment) {
    return increment <= max_value - value;
  };
  const auto trajectory_increment = trajectory_count.value;
  const auto map_increment = map_count.value;
  if (!has_capacity(counters_.trajectory_submitted, trajectory_increment) ||
      !has_capacity(counters_.trajectory_accepted, trajectory_increment) ||
      !has_capacity(counters_.trajectory_processed, trajectory_increment) ||
      !has_capacity(counters_.trajectory_outputs, trajectory_increment) ||
      !has_capacity(next_trajectory_order_, trajectory_increment) ||
      !has_capacity(counters_.map_submitted, map_increment) ||
      !has_capacity(counters_.map_accepted, map_increment) ||
      !has_capacity(counters_.map_processed, map_increment) ||
      !has_capacity(counters_.map_outputs, map_increment) ||
      !has_capacity(next_map_order_, map_increment)) {
    return reject(ErrorCode::kCounterOverflow,
                  "output batch counter capacity exhausted");
  }

  std::uint64_t expected_order = next_trajectory_order_;
  StampNanoseconds previous_stamp = last_trajectory_nanoseconds_;
  bool have_previous_stamp = have_trajectory_stamp_;
  for (const auto& sample : trajectories) {
    const auto valid = validate_trajectory_sample(sample);
    if (!valid) return reject(valid.error.code, valid.error.detail);
    if (sample.frame_id != contract_.trajectory_frame) {
      return reject(ErrorCode::kInvalidLayout,
                    "trajectory frame differs from the sequence output convention");
    }
    if (sample.event.order != expected_order) {
      return reject(ErrorCode::kOutputOrderViolation,
                    "trajectory batch order is not contiguous from zero");
    }
    if (have_previous_stamp && sample.event.nanoseconds < previous_stamp) {
      return reject(ErrorCode::kNonMonotonicStamp,
                    "trajectory batch timestamp regressed");
    }
    ++expected_order;
    previous_stamp = sample.event.nanoseconds;
    have_previous_stamp = true;
  }

  std::uint64_t expected_map_order = next_map_order_;
  for (const auto& map : maps) {
    const auto valid = validate_map_chunk(map);
    if (!valid) return reject(valid.error.code, valid.error.detail);
    if (map.frame_id != contract_.map_frame) {
      return reject(ErrorCode::kInvalidLayout,
                    "map frame differs from the sequence output convention");
    }
    if (map.order != expected_map_order) {
      return reject(ErrorCode::kOutputOrderViolation,
                    "map batch order is not contiguous from zero");
    }
    ++expected_map_order;
  }

  // All semantic and capacity checks are complete.  This is the only
  // mutation point, so a bad map cannot expose a trajectory (or vice versa).
  counters_.trajectory_submitted += trajectory_increment;
  counters_.trajectory_accepted += trajectory_increment;
  counters_.trajectory_processed += trajectory_increment;
  counters_.trajectory_outputs += trajectory_increment;
  counters_.map_submitted += map_increment;
  counters_.map_accepted += map_increment;
  counters_.map_processed += map_increment;
  counters_.map_outputs += map_increment;
  next_trajectory_order_ = expected_order;
  next_map_order_ = expected_map_order;
  if (!trajectories.empty()) {
    last_trajectory_nanoseconds_ = previous_stamp;
    have_trajectory_stamp_ = true;
  }
  return Status::success();
}

Status AdapterBoundary::complete_drain() {
  if (state_ != BoundaryState::kDraining) {
    return reject(ErrorCode::kInvalidState,
                  "drain completion requires the draining phase");
  }
  if ((contract_.require_trajectory_output &&
       counters_.trajectory_processed == 0) ||
      (contract_.require_map_output && counters_.map_processed == 0)) {
    return reject(ErrorCode::kInvalidState,
                  "required trajectory/map output is missing before drain completion");
  }
  const auto capacity = preflight_counter_capacity(
      {CounterSlot{&counters_.drain_completions, "drain_completions"},
       CounterSlot{&counters_.failures, "failures"}});
  if (!capacity) {
    return reject(ErrorCode::kCounterOverflow, capacity.error.detail);
  }
  const auto status = core_call("on_drain_complete",
                                [this]() { return sink_.on_drain_complete(); });
  if (!status) return status;
  state_ = BoundaryState::kDrained;
  const auto counted = increment_counters(
      {CounterSlot{&counters_.drain_completions, "drain_completions"}});
  if (!counted) return reject(ErrorCode::kCounterOverflow, counted.error.detail);
  return Status::success();
}

}  // namespace glim_clean_room
