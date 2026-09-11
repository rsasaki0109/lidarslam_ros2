#pragma once

// This header belongs only to the opt-in exact-core Phase 3a target.  The
// Phase 0 host ABI in include/glim_clean_room contains no GLIM types.
#include "glim_clean_room/contract.hpp"

#include <Eigen/Core>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/util/raw_points.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace glim_clean_room::phase3a {

struct CoreImuInput {
  double stamp_seconds{0.0};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

// Binds the authoritative integer event identity to the lossy double stamp
// required by the exact public core API.  Entries are consumed at most once;
// a terminal error is permanent and all subsequent operations fail closed.
class StampLedger final {
 public:
  struct Lookup {
    std::size_t index{0};
    EventStamp event{};
  };

  explicit StampLedger(std::size_t capacity);

  Result<double> bind_input(const EventStamp& event);
  Result<Lookup> lookup_output(double core_seconds);
  Status commit_output(const Lookup& lookup);

  std::size_t capacity() const noexcept { return capacity_; }
  std::size_t size() const noexcept { return entries_.size(); }
  std::size_t emitted_count() const noexcept { return emitted_count_; }
  bool all_emitted() const noexcept {
    return emitted_count_ == entries_.size();
  }
  bool terminal() const noexcept { return terminal_error_.code != ErrorCode::kNone; }
  const Error& terminal_error() const noexcept { return terminal_error_; }

 private:
  struct Entry {
    EventStamp event{};
    double core_seconds{0.0};
    bool emitted{false};
  };

  Result<double> fail_double(ErrorCode code, const std::string& detail);
  Result<Lookup> fail_lookup(ErrorCode code, const std::string& detail);
  Status fail_status(ErrorCode code, const std::string& detail);
  bool core_stamp_matches(double lhs, double rhs) const noexcept;

  std::size_t capacity_{0};
  std::vector<Entry> entries_;
  std::size_t emitted_count_{0};
  std::uint64_t last_input_order_{0};
  bool have_last_input_order_{false};
  StampNanoseconds last_nanoseconds_{0};
  bool have_last_nanoseconds_{false};
  Error terminal_error_{};
};

Result<double> checked_event_seconds(const EventStamp& event);

// Validates the exact core value shape before it can cross into processing.
// Optional GLIM vectors are either empty or exactly point-count sized; the
// host conversion uses w == 1 for every Cartesian point.
Status validate_core_raw_points(const glim::RawPoints& raw);

// Conversion is deliberately explicit about the sequence contract.  GLIM's
// RawPoints has no frame-id field; this function validates the host frame ID
// but never infers or silently rewrites it.
Result<glim::RawPoints::Ptr> to_core_raw_points(const LidarFrame& frame,
                                                const SequenceContract& contract,
                                                StampLedger& ledger);

Result<CoreImuInput> to_core_imu(const ImuSample& sample,
                                 std::string_view expected_frame);

class TrajectoryConverter final {
 public:
  explicit TrajectoryConverter(const SequenceContract& contract);

  Result<TrajectorySample> convert(const glim::EstimationFrame::ConstPtr& frame,
                                   StampLedger& ledger);

  std::uint64_t next_order() const noexcept { return next_order_; }
  bool terminal() const noexcept { return terminal_error_.code != ErrorCode::kNone; }
  const Error& terminal_error() const noexcept { return terminal_error_; }

 private:
  Result<TrajectorySample> fail(ErrorCode code, const std::string& detail);

  std::string trajectory_frame_;
  std::uint64_t next_order_{0};
  Error terminal_error_{};
};

// GLIM's public map objects are not converted in Phase 3a.  This explicit
// status prevents a caller from treating an absent map conversion as success.
Status map_conversion_not_implemented() noexcept;

}  // namespace glim_clean_room::phase3a
