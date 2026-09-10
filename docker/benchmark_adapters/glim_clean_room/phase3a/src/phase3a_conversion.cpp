#include "glim_clean_room/phase3a_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <sstream>
#include <utility>

namespace glim_clean_room::phase3a {
namespace {

constexpr StampNanoseconds kNanosecondsPerSecond = 1000000000;
constexpr double kHomogeneousTolerance = 1e-9;
constexpr double kRotationTolerance = 1e-9;
constexpr double kQuaternionTolerance = 1e-6;
constexpr std::uint64_t kMaxOutputUlps = 1;

bool finite(double value) noexcept { return std::isfinite(value); }

std::uint64_t raw_bits(double value) noexcept {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

std::uint64_t ulp_distance(double lhs, double rhs) noexcept {
  if (lhs == rhs) return 0;
  if (!finite(lhs) || !finite(rhs) || std::signbit(lhs) != std::signbit(rhs)) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  const auto lhs_bits = raw_bits(lhs);
  const auto rhs_bits = raw_bits(rhs);
  const auto lhs_ordered = std::signbit(lhs) ? ~lhs_bits : lhs_bits;
  const auto rhs_ordered = std::signbit(rhs) ? ~rhs_bits : rhs_bits;
  return lhs_ordered >= rhs_ordered ? lhs_ordered - rhs_ordered
                                    : rhs_ordered - lhs_ordered;
}

bool near(double lhs, double rhs, double tolerance) noexcept {
  return std::abs(lhs - rhs) <= tolerance;
}

Status validate_rigid_transform(const Eigen::Isometry3d& transform) {
  const Eigen::Matrix4d matrix = transform.matrix();
  if (!matrix.allFinite()) {
    return Status::failure(ErrorCode::kInvalidCalibration,
                           "core trajectory transform is non-finite");
  }
  if (!near(matrix(3, 0), 0.0, kHomogeneousTolerance) ||
      !near(matrix(3, 1), 0.0, kHomogeneousTolerance) ||
      !near(matrix(3, 2), 0.0, kHomogeneousTolerance) ||
      !near(matrix(3, 3), 1.0, kHomogeneousTolerance)) {
    return Status::failure(ErrorCode::kInvalidCalibration,
                           "core trajectory transform has an invalid homogeneous row");
  }

  const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
  if (!rotation.allFinite()) {
    return Status::failure(ErrorCode::kInvalidCalibration,
                           "core trajectory rotation is non-finite");
  }
  const Eigen::Matrix3d orthonormal = rotation.transpose() * rotation;
  if (!orthonormal.isApprox(Eigen::Matrix3d::Identity(),
                            kRotationTolerance)) {
    return Status::failure(ErrorCode::kInvalidCalibration,
                           "core trajectory rotation is not orthonormal");
  }
  if (!near(rotation.determinant(), 1.0, kRotationTolerance)) {
    return Status::failure(ErrorCode::kInvalidCalibration,
                           "core trajectory rotation determinant is not +1");
  }
  return Status::success();
}

Result<double> fail_seconds(ErrorCode code, const std::string& detail) {
  return Result<double>::failure(code, detail);
}

}  // namespace

Result<double> checked_event_seconds(const EventStamp& event) {
  // Split the signed integer before conversion.  This avoids an intermediate
  // integer overflow and makes the lossy conversion explicit; identity is
  // recovered only from StampLedger's retained integer entry.
  const StampNanoseconds seconds = event.nanoseconds / kNanosecondsPerSecond;
  const StampNanoseconds remainder = event.nanoseconds % kNanosecondsPerSecond;
  const double converted = static_cast<double>(seconds) +
                           static_cast<double>(remainder) * 1e-9;
  if (!finite(converted)) {
    return fail_seconds(ErrorCode::kInvalidStamp,
                        "event stamp cannot be represented as finite core seconds");
  }
  return Result<double>::success(converted);
}

Status validate_core_raw_points(const glim::RawPoints& raw) {
  const auto point_count = raw.points.size();
  if (point_count == 0 ||
      point_count > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return Status::failure(ErrorCode::kUnsupportedCoreOutput,
                           "core RawPoints has an invalid point count");
  }
  if (raw.times.size() != point_count ||
      (!raw.intensities.empty() && raw.intensities.size() != point_count) ||
      (!raw.colors.empty() && raw.colors.size() != point_count) ||
      (!raw.rings.empty() && raw.rings.size() != point_count)) {
    return Status::failure(ErrorCode::kUnsupportedCoreOutput,
                           "core RawPoints vectors have mismatched sizes");
  }
  if (!finite(raw.stamp)) {
    return Status::failure(ErrorCode::kUnsupportedCoreOutput,
                           "core RawPoints stamp is non-finite");
  }
  for (std::size_t i = 0; i < point_count; ++i) {
    if (!finite(raw.times[i]) || raw.times[i] < 0.0 ||
        !raw.points[i].allFinite() ||
        std::abs(raw.points[i].w() - 1.0) > kHomogeneousTolerance) {
      return Status::failure(
          ErrorCode::kUnsupportedCoreOutput,
          "core RawPoints contains a non-finite, negative-time, or non-homogeneous point");
    }
    if (!raw.intensities.empty() && !finite(raw.intensities[i])) {
      return Status::failure(ErrorCode::kUnsupportedCoreOutput,
                             "core RawPoints intensity is non-finite");
    }
  }
  return Status::success();
}

StampLedger::StampLedger(std::size_t capacity) : capacity_(capacity) {
  entries_.reserve(capacity_);
  if (capacity_ == 0) {
    terminal_error_ = Error{ErrorCode::kLedgerOverflow,
                            "stamp ledger capacity must be non-zero"};
  }
}

Result<double> StampLedger::fail_double(ErrorCode code,
                                        const std::string& detail) {
  if (!terminal()) terminal_error_ = Error{code, detail};
  return Result<double>::failure(terminal_error_.code, terminal_error_.detail);
}

Result<StampLedger::Lookup> StampLedger::fail_lookup(
    ErrorCode code, const std::string& detail) {
  if (!terminal()) terminal_error_ = Error{code, detail};
  return Result<Lookup>::failure(terminal_error_.code, terminal_error_.detail);
}

Status StampLedger::fail_status(ErrorCode code, const std::string& detail) {
  if (!terminal()) terminal_error_ = Error{code, detail};
  return Status::failure(terminal_error_.code, terminal_error_.detail);
}

bool StampLedger::core_stamp_matches(double lhs, double rhs) const noexcept {
  return finite(lhs) && finite(rhs) && ulp_distance(lhs, rhs) <= kMaxOutputUlps;
}

Result<double> StampLedger::bind_input(const EventStamp& event) {
  if (terminal()) return fail_double(terminal_error_.code, terminal_error_.detail);
  if (entries_.size() >= capacity_) {
    return fail_double(ErrorCode::kLedgerOverflow,
                       "stamp ledger capacity exhausted before core submission");
  }
  if (have_last_input_order_ && event.order <= last_input_order_) {
    return fail_double(ErrorCode::kOrderViolation,
                       "stamp ledger input order must increase globally");
  }
  if (have_last_nanoseconds_ && event.nanoseconds < last_nanoseconds_) {
    return fail_double(ErrorCode::kNonMonotonicStamp,
                       "stamp ledger input timestamp regressed");
  }
  const auto converted = checked_event_seconds(event);
  if (!converted) {
    return fail_double(converted.error.code, converted.error.detail);
  }
  for (const auto& entry : entries_) {
    if (core_stamp_matches(entry.core_seconds, converted.value)) {
      return fail_double(
          ErrorCode::kStampLedgerAmbiguous,
          "integer timestamps are equal or ULP-near in the core double domain");
    }
  }
  try {
    entries_.push_back(Entry{event, converted.value, false});
  } catch (const std::exception& exception) {
    return fail_double(ErrorCode::kCoreFailure,
                       std::string("stamp ledger allocation failed: ") +
                           exception.what());
  } catch (...) {
    return fail_double(ErrorCode::kCoreFailure,
                       "stamp ledger allocation failed with an unknown exception");
  }
  last_nanoseconds_ = event.nanoseconds;
  have_last_nanoseconds_ = true;
  last_input_order_ = event.order;
  have_last_input_order_ = true;
  return Result<double>::success(converted.value);
}

Result<StampLedger::Lookup> StampLedger::lookup_output(double core_seconds) {
  if (terminal()) return fail_lookup(terminal_error_.code, terminal_error_.detail);
  if (!finite(core_seconds)) {
    return fail_lookup(ErrorCode::kUnsupportedCoreOutput,
                       "core output stamp must be finite");
  }

  std::size_t available_matches = 0;
  std::size_t consumed_matches = 0;
  std::size_t selected_index = 0;
  for (std::size_t index = 0; index < entries_.size(); ++index) {
    const auto& entry = entries_[index];
    if (!core_stamp_matches(entry.core_seconds, core_seconds)) continue;
    if (entry.emitted) {
      ++consumed_matches;
    } else {
      ++available_matches;
      selected_index = index;
    }
  }
  if (available_matches > 1) {
    return fail_lookup(ErrorCode::kStampLedgerAmbiguous,
                       "core output stamp matches multiple integer events");
  }
  if (available_matches == 0 && consumed_matches != 0) {
    return fail_lookup(ErrorCode::kStampLedgerReused,
                       "core output stamp was already emitted");
  }
  if (available_matches == 0) {
    return fail_lookup(ErrorCode::kStampLedgerUnmatched,
                       "core output stamp has no submitted integer event");
  }
  return Result<Lookup>::success(
      Lookup{selected_index, entries_[selected_index].event});
}

Status StampLedger::commit_output(const Lookup& lookup) {
  if (terminal()) return fail_status(terminal_error_.code, terminal_error_.detail);
  if (lookup.index >= entries_.size()) {
    return fail_status(ErrorCode::kStampLedgerUnmatched,
                       "stamp ledger lookup index is outside the ledger");
  }
  auto& entry = entries_[lookup.index];
  if (entry.emitted) {
    return fail_status(ErrorCode::kStampLedgerReused,
                       "stamp ledger output was committed more than once");
  }
  if (entry.event.order != lookup.event.order ||
      entry.event.nanoseconds != lookup.event.nanoseconds) {
    return fail_status(ErrorCode::kStampLedgerAmbiguous,
                       "stamp ledger lookup identity changed before commit");
  }
  entry.emitted = true;
  ++emitted_count_;
  return Status::success();
}

Result<glim::RawPoints::Ptr> to_core_raw_points(const LidarFrame& frame,
                                                const SequenceContract& contract,
                                                StampLedger& ledger) {
  const auto contract_status = validate_sequence_contract(contract);
  if (!contract_status) {
    return Result<glim::RawPoints::Ptr>::failure(contract_status.error.code,
                                                 contract_status.error.detail);
  }
  const auto frame_status = validate_lidar_frame(
      frame, contract.point_time_unit, contract.point_fields,
      contract.calibration.lidar_frame);
  if (!frame_status) {
    return Result<glim::RawPoints::Ptr>::failure(frame_status.error.code,
                                                 frame_status.error.detail);
  }
  if (frame.points.size() >
      static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return Result<glim::RawPoints::Ptr>::failure(
        ErrorCode::kInvalidLayout, "LiDAR point count exceeds core int size");
  }

  const auto stamp = checked_event_seconds(frame.event);
  if (!stamp) {
    return Result<glim::RawPoints::Ptr>::failure(stamp.error.code,
                                                 stamp.error.detail);
  }
  try {
    auto raw = std::make_shared<glim::RawPoints>();
    raw->stamp = stamp.value;
    raw->times.reserve(frame.points.size());
    raw->intensities.reserve(frame.points.size());
    raw->rings.reserve(frame.points.size());
    raw->points.reserve(frame.points.size());
    for (const auto& point : frame.points) {
      const double point_seconds =
          point_time_to_seconds(point.relative_time_raw, frame.point_time_unit);
      if (!finite(point_seconds) || point_seconds < 0.0) {
        return Result<glim::RawPoints::Ptr>::failure(
            ErrorCode::kInvalidStamp,
            "LiDAR relative point time cannot be represented as finite seconds");
      }
      raw->times.push_back(point_seconds);
      raw->intensities.push_back(point.intensity);
      raw->rings.push_back(point.ring);
      // The core source checks/uses homogeneous points with w == 1 for input
      // point clouds.  LidarFrame is 3D Cartesian data, so this is explicit,
      // never inferred from an incoming fourth coordinate.
      raw->points.emplace_back(point.x, point.y, point.z, 1.0);
    }

    const auto raw_status = validate_core_raw_points(*raw);
    if (!raw_status) {
      return Result<glim::RawPoints::Ptr>::failure(raw_status.error.code,
                                                   raw_status.error.detail);
    }

    const auto bound = ledger.bind_input(frame.event);
    if (!bound) {
      return Result<glim::RawPoints::Ptr>::failure(bound.error.code,
                                                   bound.error.detail);
    }
    raw->stamp = bound.value;
    return Result<glim::RawPoints::Ptr>::success(std::move(raw));
  } catch (const std::exception& exception) {
    return Result<glim::RawPoints::Ptr>::failure(
        ErrorCode::kCoreFailure,
        std::string("LiDAR core conversion failed: ") + exception.what());
  } catch (...) {
    return Result<glim::RawPoints::Ptr>::failure(
        ErrorCode::kCoreFailure,
        "LiDAR core conversion failed with an unknown exception");
  }
}

Result<CoreImuInput> to_core_imu(const ImuSample& sample,
                                 std::string_view expected_frame) {
  const auto status = validate_imu_sample(sample, expected_frame);
  if (!status) {
    return Result<CoreImuInput>::failure(status.error.code, status.error.detail);
  }
  const auto stamp = checked_event_seconds(sample.event);
  if (!stamp) {
    return Result<CoreImuInput>::failure(stamp.error.code, stamp.error.detail);
  }
  CoreImuInput output;
  output.stamp_seconds = stamp.value;
  for (std::size_t i = 0; i < 3; ++i) {
    if (!finite(sample.linear_acceleration[i]) ||
        !finite(sample.angular_velocity[i])) {
      return Result<CoreImuInput>::failure(
          ErrorCode::kNonFinite, "IMU core conversion received a non-finite value");
    }
    output.linear_acceleration[static_cast<Eigen::Index>(i)] =
        sample.linear_acceleration[i];
    output.angular_velocity[static_cast<Eigen::Index>(i)] =
        sample.angular_velocity[i];
  }
  return Result<CoreImuInput>::success(std::move(output));
}

TrajectoryConverter::TrajectoryConverter(const SequenceContract& contract)
    : trajectory_frame_(contract.trajectory_frame) {
  const auto valid = validate_sequence_contract(contract);
  if (!valid) terminal_error_ = valid.error;
}

Result<TrajectorySample> TrajectoryConverter::fail(
    ErrorCode code, const std::string& detail) {
  if (!terminal()) terminal_error_ = Error{code, detail};
  return Result<TrajectorySample>::failure(terminal_error_.code,
                                           terminal_error_.detail);
}

Result<TrajectorySample> TrajectoryConverter::convert(
    const glim::EstimationFrame::ConstPtr& frame, StampLedger& ledger) {
  if (terminal()) return fail(terminal_error_.code, terminal_error_.detail);
  if (!frame) return fail(ErrorCode::kUnsupportedCoreOutput,
                          "core returned a null estimation frame");
  if (!finite(frame->stamp)) {
    return fail(ErrorCode::kUnsupportedCoreOutput,
                "core estimation frame stamp is non-finite");
  }
  if (next_order_ == std::numeric_limits<std::uint64_t>::max()) {
    return fail(ErrorCode::kLedgerOverflow,
                "trajectory output order cannot advance beyond uint64 maximum");
  }
  const auto transform_status = validate_rigid_transform(frame->T_world_lidar);
  if (!transform_status) {
    return fail(transform_status.error.code, transform_status.error.detail);
  }

  const auto lookup = ledger.lookup_output(frame->stamp);
  if (!lookup) return fail(lookup.error.code, lookup.error.detail);

  Eigen::Quaterniond quaternion(frame->T_world_lidar.linear());
  const double norm = quaternion.norm();
  if (!finite(norm) || std::abs(norm - 1.0) > kQuaternionTolerance) {
    return fail(ErrorCode::kInvalidCalibration,
                "core trajectory quaternion is outside normalized tolerance");
  }
  quaternion.normalize();
  // Canonical sign: positive w; for the exact w==0 half-turn, lexicographically
  // positive (x, y, z).  This removes the q/-q representation ambiguity.
  const bool negate = quaternion.w() < 0.0 ||
                      (quaternion.w() == 0.0 &&
                       (quaternion.x() < 0.0 ||
                        (quaternion.x() == 0.0 &&
                         (quaternion.y() < 0.0 ||
                          (quaternion.y() == 0.0 && quaternion.z() < 0.0)))));
  if (negate) quaternion.coeffs() *= -1.0;

  TrajectorySample output;
  output.event = EventStamp{next_order_, lookup.value.event.nanoseconds};
  output.frame_id = trajectory_frame_;
  const Eigen::Vector3d translation = frame->T_world_lidar.translation();
  output.pose = {translation.x(), translation.y(), translation.z(),
                 quaternion.x(), quaternion.y(), quaternion.z(),
                 quaternion.w()};
  const auto valid = validate_trajectory_sample(output);
  if (!valid) return fail(valid.error.code, valid.error.detail);
  const auto committed = ledger.commit_output(lookup.value);
  if (!committed) return fail(committed.error.code, committed.error.detail);
  ++next_order_;
  return Result<TrajectorySample>::success(std::move(output));
}

Status map_conversion_not_implemented() noexcept {
  return Status::failure(ErrorCode::kNotImplemented,
                         "Phase 3a does not convert GLIM map objects");
}

}  // namespace glim_clean_room::phase3a
