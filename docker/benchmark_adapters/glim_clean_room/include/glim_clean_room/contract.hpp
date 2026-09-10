#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace glim_clean_room {

enum class ErrorCode {
  kNone,
  kMissingField,
  kDuplicateField,
  kInvalidLayout,
  kUnsupportedFieldType,
  kWrongTimeUnit,
  kInvalidStamp,
  kEmptyPayload,
  kNonFinite,
  kNonMonotonicStamp,
  kOrderViolation,
  kMissingCalibration,
  kInvalidCalibration,
  kForbiddenArtifactRole,
  kForbiddenArtifactPath,
  kInvalidArtifactPath,
  kInvalidState,
  kPostEof,
  kOutputOrderViolation,
  kCoreFailure,
  kCounterOverflow,
  kStampLedgerAmbiguous,
  kStampLedgerUnmatched,
  kStampLedgerReused,
  kLedgerOverflow,
  kUnsupportedCoreOutput,
  kNotImplemented,
};

const char* error_code_name(ErrorCode code) noexcept;

struct Error {
  ErrorCode code{ErrorCode::kNone};
  std::string detail;
};

struct Status {
  bool ok{true};
  Error error{};

  static Status success() noexcept { return Status{}; }
  static Status failure(ErrorCode code, std::string detail) {
    return Status{false, Error{code, std::move(detail)}};
  }
  explicit operator bool() const noexcept { return ok; }
};

template <typename T>
struct Result {
  bool ok{false};
  T value{};
  Error error{};

  static Result success(T result) {
    Result output;
    output.ok = true;
    output.value = std::move(result);
    return output;
  }
  static Result failure(ErrorCode code, std::string detail) {
    Result output;
    output.error = Error{code, std::move(detail)};
    return output;
  }
  explicit operator bool() const noexcept { return ok; }
};

enum class PointTimeUnit {
  kSeconds,
  kMilliseconds,
  kMicroseconds,
  kNanoseconds,
  kUnspecified,
};

double point_time_to_seconds(double value, PointTimeUnit unit) noexcept;
Status validate_point_time_unit(PointTimeUnit unit);

// These values mirror sensor_msgs/PointField datatype IDs without requiring a
// ROS dependency in the host-owned contract.  A sequence must bind every
// semantic field name, datatype, and count explicitly.
enum class PointFieldType : std::uint8_t {
  kUInt16 = 4,
  kUInt32 = 6,
  kFloat32 = 7,
  kFloat64 = 8,
};

struct PointFieldSpec {
  std::string name;
  PointFieldType datatype{PointFieldType::kFloat32};
  std::uint32_t count{0};
};

struct PointFieldMapping {
  PointFieldSpec x{};
  PointFieldSpec y{};
  PointFieldSpec z{};
  PointFieldSpec intensity{};
  PointFieldSpec ring{};
  PointFieldSpec relative_time{};
  PointTimeUnit time_unit{PointTimeUnit::kUnspecified};
};

Status validate_point_field_mapping(const PointFieldMapping& mapping);
bool operator==(const PointFieldSpec& lhs, const PointFieldSpec& rhs) noexcept;
bool operator==(const PointFieldMapping& lhs,
                const PointFieldMapping& rhs) noexcept;

using StampNanoseconds = std::int64_t;

// ROS 2 header stamps are converted only through this checked constructor.
// The rest of the adapter retains integer nanoseconds; floating-point seconds
// are a core-facing conversion at the final API boundary only.
struct EventStamp {
  std::uint64_t order{0};
  StampNanoseconds nanoseconds{0};
};

Result<EventStamp> event_stamp_from_ros(std::uint64_t order,
                                        std::int64_t sec,
                                        std::int64_t nanosec);
double event_stamp_to_seconds(const EventStamp& stamp) noexcept;

enum class ArtifactRole {
  kInput,
  kCalibration,
  kConfiguration,
  kCoreSource,
  kGroundTruth,
  kScorer,
  kUnknown,
};

struct ArtifactReference {
  ArtifactRole role{ArtifactRole::kUnknown};
  std::string logical_path;
};

Status validate_artifact_references(const std::vector<ArtifactReference>& artifacts);

enum class CalibrationConvention {
  kTImuLidar,
  kTLidarImu,
  kUnspecified,
};

struct Calibration {
  CalibrationConvention convention{CalibrationConvention::kUnspecified};
  CalibrationConvention inverse_convention{CalibrationConvention::kUnspecified};
  std::string imu_frame;
  std::string lidar_frame;
  // Row-major homogeneous transforms.  `transform` and `inverse_transform`
  // are both bound so a consumer cannot silently invert an ambiguous input.
  std::array<double, 16> transform{};
  std::array<double, 16> inverse_transform{};
};

Status validate_calibration(const Calibration& calibration);

struct PointRecord {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double intensity{0.0};
  double relative_time{0.0};
  std::uint32_t ring{0};
  // Raw relative time is retained for auditability; `relative_time` is the
  // checked seconds conversion used only at the core-facing boundary.
  double relative_time_raw{0.0};
};

struct LidarFrame {
  EventStamp event{};
  std::string frame_id;
  PointTimeUnit point_time_unit{PointTimeUnit::kUnspecified};
  PointFieldMapping field_mapping{};
  std::vector<PointRecord> points;
};

Status validate_lidar_frame(const LidarFrame& frame,
                            PointTimeUnit expected_time_unit,
                            const PointFieldMapping& expected_mapping,
                            std::string_view expected_frame);

enum class AccelerationUnit {
  kMetersPerSecondSquared,
  kUnspecified,
};

enum class AngularVelocityUnit {
  kRadiansPerSecond,
  kUnspecified,
};

struct ImuSample {
  EventStamp event{};
  std::string frame_id;
  std::array<double, 3> linear_acceleration{};
  std::array<double, 3> angular_velocity{};
  AccelerationUnit acceleration_unit{AccelerationUnit::kUnspecified};
  AngularVelocityUnit angular_velocity_unit{AngularVelocityUnit::kUnspecified};
};

Status validate_imu_sample(const ImuSample& sample,
                           std::string_view expected_frame);

struct SequenceContract {
  std::string sequence_id;
  PointTimeUnit point_time_unit{PointTimeUnit::kUnspecified};
  Calibration calibration{};
  PointFieldMapping point_fields{};
  std::string trajectory_frame;
  std::string map_frame;
  bool require_trajectory_output{true};
  bool require_map_output{true};
  std::vector<ArtifactReference> artifacts;
};

Status validate_sequence_contract(const SequenceContract& contract);

struct TrajectorySample {
  EventStamp event{};
  std::string frame_id;
  // Translation followed by quaternion x,y,z,w.  The frame convention is
  // bound by the sequence contract; this type carries no implicit conversion.
  std::array<double, 7> pose{};
};

struct MapChunk {
  std::uint64_t order{0};
  std::string frame_id;
  std::vector<PointRecord> points;
};

Status validate_trajectory_sample(const TrajectorySample& sample);
Status validate_map_chunk(const MapChunk& chunk);

struct ConsumerCounters {
  std::uint64_t lidar_submitted{0};
  std::uint64_t lidar_accepted{0};
  std::uint64_t lidar_rejected{0};
  std::uint64_t lidar_received{0};
  std::uint64_t imu_received{0};
  std::uint64_t imu_submitted{0};
  std::uint64_t imu_accepted{0};
  std::uint64_t imu_rejected{0};
  std::uint64_t lidar_processed{0};
  std::uint64_t imu_processed{0};
  std::uint64_t trajectory_submitted{0};
  std::uint64_t trajectory_accepted{0};
  std::uint64_t trajectory_rejected{0};
  std::uint64_t trajectory_processed{0};
  std::uint64_t map_submitted{0};
  std::uint64_t map_accepted{0};
  std::uint64_t map_rejected{0};
  std::uint64_t map_processed{0};
  std::uint64_t trajectory_outputs{0};
  std::uint64_t map_outputs{0};
  std::uint64_t eof_requests{0};
  std::uint64_t drain_completions{0};
  std::uint64_t failures{0};
};

class CoreSink {
 public:
  virtual ~CoreSink() = default;
  virtual Status on_lidar(const LidarFrame& frame) = 0;
  virtual Status on_imu(const ImuSample& sample) = 0;
  virtual Status on_eof() = 0;
  virtual Status on_begin_drain() = 0;
  virtual Status on_drain_complete() = 0;
};

enum class BoundaryState {
  kOpen,
  kEofRequested,
  kDraining,
  kDrained,
  kFailed,
};

class AdapterBoundary {
 public:
  AdapterBoundary(SequenceContract contract, CoreSink& sink);

  Status submit_lidar(const LidarFrame& frame);
  Status submit_imu(const ImuSample& sample);
  Status request_eof();
  Status begin_drain();
  Status record_trajectory(const TrajectorySample& sample);
  // Validate and seal a staged output vector atomically.  All counter and
  // output-order capacity is checked before the first counter changes.
  Status record_trajectory_batch(const std::vector<TrajectorySample>& samples);
  Status record_map_chunk(const MapChunk& chunk);
  // Validate and seal trajectory and map batches as one publication unit.
  // Neither output family changes counters unless every sample/chunk and all
  // counter capacities pass preflight.  The host session owns the actual
  // staged bytes; this boundary owns lifecycle, order, and accounting.
  Status record_output_batch(const std::vector<TrajectorySample>& trajectories,
                             const std::vector<MapChunk>& maps);
  Status complete_drain();

  const SequenceContract& contract() const noexcept { return contract_; }
  const ConsumerCounters& counters() const noexcept { return counters_; }
  BoundaryState state() const noexcept { return state_; }
  const Error& terminal_error() const noexcept { return terminal_error_; }

#ifdef GLIM_CLEAN_ROOM_TESTING
  // Test-only fault-injection hook.  It is not present in the installed or
  // production ABI and exists solely to exercise uint64 counter boundaries.
  void set_test_counters(const ConsumerCounters& counters) noexcept {
    counters_ = counters;
  }
  void set_test_orders(std::uint64_t input_order,
                       std::uint64_t trajectory_order,
                       std::uint64_t map_order) noexcept {
    next_event_order_ = input_order;
    next_trajectory_order_ = trajectory_order;
    next_map_order_ = map_order;
  }
#endif

 private:
  struct CounterSlot {
    std::uint64_t* value;
    const char* name;
  };

  Status reject(ErrorCode code, const std::string& detail,
                std::uint64_t* rejected_counter = nullptr);
  Status preflight_counter_capacity(
      std::initializer_list<CounterSlot> counters) const;
  Status increment_counters(std::initializer_list<CounterSlot> counters);
  Status increment_counter(std::uint64_t& counter, const char* name);
  Status accept_event(const EventStamp& event);
  Status core_call(const char* operation, const std::function<Status()>& call,
                   std::uint64_t* rejected_counter = nullptr);

  SequenceContract contract_;
  CoreSink& sink_;
  ConsumerCounters counters_{};
  BoundaryState state_{BoundaryState::kOpen};
  Error terminal_error_{};
  std::uint64_t next_event_order_{0};
  StampNanoseconds last_event_nanoseconds_{0};
  bool have_event_stamp_{false};
  std::uint64_t next_trajectory_order_{0};
  StampNanoseconds last_trajectory_nanoseconds_{0};
  bool have_trajectory_stamp_{false};
  std::uint64_t next_map_order_{0};
};

}  // namespace glim_clean_room
