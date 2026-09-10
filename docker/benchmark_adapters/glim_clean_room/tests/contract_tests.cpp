#include "glim_clean_room/contract.hpp"
#include "glim_clean_room/pointcloud2_parser.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using namespace glim_clean_room;

struct RecordingSink final : CoreSink {
  int lidar{0};
  int imu{0};
  int eof{0};
  int begin_drain{0};
  int complete{0};
  bool throw_lidar{false};
  bool fail_lidar{false};

  Status on_lidar(const LidarFrame&) override {
    ++lidar;
    if (throw_lidar) throw std::runtime_error("synthetic lidar failure");
    if (fail_lidar) {
      return Status::failure(ErrorCode::kCoreFailure, "synthetic lidar failure");
    }
    return Status::success();
  }
  Status on_imu(const ImuSample&) override {
    ++imu;
    return Status::success();
  }
  Status on_eof() override {
    ++eof;
    return Status::success();
  }
  Status on_begin_drain() override {
    ++begin_drain;
    return Status::success();
  }
  Status on_drain_complete() override {
    ++complete;
    return Status::success();
  }
};

PointFieldMapping mapping() {
  PointFieldMapping value;
  value.x = {"x", PointFieldType::kFloat32, 1};
  value.y = {"y", PointFieldType::kFloat32, 1};
  value.z = {"z", PointFieldType::kFloat32, 1};
  value.intensity = {"intensity", PointFieldType::kFloat32, 1};
  value.ring = {"ring", PointFieldType::kUInt32, 1};
  value.relative_time = {"time", PointFieldType::kFloat32, 1};
  value.time_unit = PointTimeUnit::kMilliseconds;
  return value;
}

std::array<double, 16> identity_transform() {
  return {1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0};
}

SequenceContract contract() {
  SequenceContract value;
  value.sequence_id = "synthetic-sequence";
  value.point_time_unit = PointTimeUnit::kMilliseconds;
  value.calibration.convention = CalibrationConvention::kTImuLidar;
  value.calibration.inverse_convention = CalibrationConvention::kTLidarImu;
  value.calibration.imu_frame = "imu";
  value.calibration.lidar_frame = "lidar";
  value.calibration.transform = identity_transform();
  value.calibration.inverse_transform = identity_transform();
  value.point_fields = mapping();
  value.trajectory_frame = "world";
  value.map_frame = "world";
  value.artifacts = {
      {ArtifactRole::kInput, "input/sequence"},
      {ArtifactRole::kCalibration, "calibration/imu_lidar.yaml"},
      {ArtifactRole::kConfiguration, "config/adapter.yaml"},
      {ArtifactRole::kCoreSource, "source/glim-core.tar.gz"},
  };
  return value;
}

EventStamp stamp(std::uint64_t order, StampNanoseconds nanoseconds) {
  return EventStamp{order, nanoseconds};
}

LidarFrame lidar(std::uint64_t order, StampNanoseconds nanoseconds) {
  LidarFrame value;
  value.event = stamp(order, nanoseconds);
  value.frame_id = "lidar";
  value.point_time_unit = PointTimeUnit::kMilliseconds;
  value.field_mapping = mapping();
  value.points.push_back(PointRecord{1.0, 2.0, 3.0, 4.0, 0.0, 7, 0.0});
  return value;
}

ImuSample imu(std::uint64_t order, StampNanoseconds nanoseconds) {
  ImuSample value;
  value.event = stamp(order, nanoseconds);
  value.frame_id = "imu";
  value.linear_acceleration = {0.0, 0.0, 9.81};
  value.angular_velocity = {0.0, 0.0, 0.0};
  value.acceleration_unit = AccelerationUnit::kMetersPerSecondSquared;
  value.angular_velocity_unit = AngularVelocityUnit::kRadiansPerSecond;
  return value;
}

void expect_error(const Status& status, ErrorCode code) {
  assert(!status);
  assert(status.error.code == code);
}

template <typename T>
void expect_error(const Result<T>& result, ErrorCode code) {
  assert(!result);
  assert(result.error.code == code);
}

void write_f32_le(std::vector<std::uint8_t>& data, std::size_t offset,
                  float value) {
  std::memcpy(data.data() + offset, &value, sizeof(value));
}

void write_f32_be(std::vector<std::uint8_t>& data, std::size_t offset,
                  float value) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  data[offset] = static_cast<std::uint8_t>((bits >> 24U) & 0xffU);
  data[offset + 1] = static_cast<std::uint8_t>((bits >> 16U) & 0xffU);
  data[offset + 2] = static_cast<std::uint8_t>((bits >> 8U) & 0xffU);
  data[offset + 3] = static_cast<std::uint8_t>(bits & 0xffU);
}

void write_u32_be(std::vector<std::uint8_t>& data, std::size_t offset,
                  std::uint32_t value) {
  data[offset] = static_cast<std::uint8_t>((value >> 24U) & 0xffU);
  data[offset + 1] = static_cast<std::uint8_t>((value >> 16U) & 0xffU);
  data[offset + 2] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
  data[offset + 3] = static_cast<std::uint8_t>(value & 0xffU);
}

PointCloud2View valid_cloud(bool bigendian = false) {
  PointCloud2View view;
  view.event = stamp(0, 10000000000LL);
  view.frame_id = "lidar";
  view.is_bigendian = bigendian;
  view.width = 1;
  view.height = 1;
  view.point_step = 24;
  view.row_step = 32;  // Explicit row padding is permitted and audited.
  view.fields = {
      {"x", 0, PointFieldType::kFloat32, 1},
      {"y", 4, PointFieldType::kFloat32, 1},
      {"z", 8, PointFieldType::kFloat32, 1},
      {"intensity", 12, PointFieldType::kFloat32, 1},
      {"time", 16, PointFieldType::kFloat32, 1},
      {"ring", 20, PointFieldType::kUInt32, 1},
  };
  view.data.assign(view.row_step, 0);
  if (!bigendian) {
    write_f32_le(view.data, 0, 1.0F);
    write_f32_le(view.data, 4, 2.0F);
    write_f32_le(view.data, 8, 3.0F);
    write_f32_le(view.data, 12, 4.0F);
    write_f32_le(view.data, 16, 10.0F);
    std::uint32_t ring = 7;
    std::memcpy(view.data.data() + 20, &ring, sizeof(ring));
  } else {
    write_f32_be(view.data, 0, 1.0F);
    write_f32_be(view.data, 4, 2.0F);
    write_f32_be(view.data, 8, 3.0F);
    write_f32_be(view.data, 12, 4.0F);
    write_f32_be(view.data, 16, 10.0F);
    write_u32_be(view.data, 20, 7);
  }
  return view;
}

void test_parser_preserves_typed_fields_unit_and_padding() {
  const auto result = parse_pointcloud2(valid_cloud(), mapping());
  assert(result);
  assert(result.value.points.size() == 1);
  assert(result.value.points[0].x == 1.0);
  assert(result.value.points[0].ring == 7);
  assert(result.value.points[0].relative_time_raw == 10.0);
  assert(std::abs(result.value.points[0].relative_time - 0.01) < 1e-12);
}

void test_parser_decodes_big_endian() {
  const auto result = parse_pointcloud2(valid_cloud(true), mapping());
  assert(result);
  assert(result.value.points[0].x == 1.0);
  assert(result.value.points[0].ring == 7);
}

void test_parser_rejects_missing_duplicate_overlap_and_wrong_mapping() {
  auto missing = valid_cloud();
  missing.fields.erase(missing.fields.begin() + 4);
  expect_error(parse_pointcloud2(missing, mapping()), ErrorCode::kMissingField);

  auto duplicate = valid_cloud();
  duplicate.fields.push_back({"x", 24, PointFieldType::kFloat32, 1});
  expect_error(parse_pointcloud2(duplicate, mapping()), ErrorCode::kDuplicateField);

  auto overlap = valid_cloud();
  overlap.fields.push_back({"other", 2, PointFieldType::kFloat32, 1});
  expect_error(parse_pointcloud2(overlap, mapping()), ErrorCode::kInvalidLayout);

  auto wrong_type = mapping();
  wrong_type.x.datatype = PointFieldType::kFloat64;
  expect_error(parse_pointcloud2(valid_cloud(), wrong_type),
               ErrorCode::kUnsupportedFieldType);
}

void test_parser_rejects_units_raw_range_and_layout_overflow() {
  auto unspecified = mapping();
  unspecified.time_unit = PointTimeUnit::kUnspecified;
  expect_error(parse_pointcloud2(valid_cloud(), unspecified),
               ErrorCode::kWrongTimeUnit);

  auto negative = valid_cloud();
  write_f32_le(negative.data, 16, -1.0F);
  expect_error(parse_pointcloud2(negative, mapping()), ErrorCode::kNonFinite);

  auto nonfinite = valid_cloud();
  const std::uint32_t quiet_nan = 0x7fc00000U;
  std::memcpy(nonfinite.data.data() + 16, &quiet_nan, sizeof(quiet_nan));
  expect_error(parse_pointcloud2(nonfinite, mapping()), ErrorCode::kNonFinite);

  auto overflow = valid_cloud();
  overflow.width = std::numeric_limits<std::uint32_t>::max();
  expect_error(parse_pointcloud2(overflow, mapping()), ErrorCode::kInvalidLayout);
}

void test_ros_stamp_constructor_is_integer_checked() {
  const auto valid = event_stamp_from_ros(2, 3, 4);
  assert(valid);
  assert(valid.value.nanoseconds == 3000000004LL);
  assert(std::abs(event_stamp_to_seconds(valid.value) - 3.000000004) < 1e-12);
  const auto negative = event_stamp_from_ros(3, -1, 500000000);
  assert(negative && negative.value.nanoseconds == -500000000LL);
  expect_error(event_stamp_from_ros(0, 1, 1000000000),
               ErrorCode::kInvalidStamp);
  expect_error(event_stamp_from_ros(0, std::numeric_limits<std::int64_t>::max(),
                                    0),
               ErrorCode::kInvalidStamp);
  expect_error(event_stamp_from_ros(0, std::numeric_limits<std::int64_t>::min(),
                                    0),
               ErrorCode::kInvalidStamp);
}

void test_sequence_rejects_gt_and_scorer_paths() {
  auto gt_role = contract();
  gt_role.artifacts.push_back({ArtifactRole::kGroundTruth, "input/gt.txt"});
  RecordingSink sink;
  AdapterBoundary boundary(gt_role, sink);
  assert(boundary.state() == BoundaryState::kFailed);
  assert(boundary.terminal_error().code == ErrorCode::kForbiddenArtifactRole);

  auto path = contract();
  path.artifacts.push_back(
      {ArtifactRole::kInput, std::string("input/") + "gt" + "/values"});
  AdapterBoundary path_boundary(path, sink);
  assert(path_boundary.state() == BoundaryState::kFailed);
  assert(path_boundary.terminal_error().code == ErrorCode::kForbiddenArtifactPath);
}

void test_boundary_enforces_integer_order_frames_and_fault_latch() {
  RecordingSink sink;
  AdapterBoundary boundary(contract(), sink);
  assert(boundary.submit_lidar(lidar(0, 1000000000LL)));
  auto wrong_frame = imu(1, 1000000000LL);
  wrong_frame.frame_id = "wrong_imu";
  expect_error(boundary.submit_imu(wrong_frame), ErrorCode::kInvalidLayout);
  assert(boundary.state() == BoundaryState::kFailed);
  assert(sink.imu == 0);

  auto after_fault = lidar(1, 2000000000LL);
  expect_error(boundary.submit_lidar(after_fault), ErrorCode::kPostEof);
  assert(sink.lidar == 1);
  assert(boundary.counters().lidar_submitted == 2);
  assert(boundary.counters().lidar_accepted == 1);
  assert(boundary.counters().lidar_processed == 1);
  assert(boundary.counters().lidar_rejected == 1);
}

void test_boundary_rejects_timestamp_regression_and_core_throw() {
  RecordingSink sink;
  AdapterBoundary boundary(contract(), sink);
  assert(boundary.submit_lidar(lidar(0, 1000000000LL)));
  auto regressed = lidar(1, 500000000LL);
  expect_error(boundary.submit_lidar(regressed), ErrorCode::kNonMonotonicStamp);
  assert(boundary.state() == BoundaryState::kFailed);

  RecordingSink throwing;
  throwing.throw_lidar = true;
  AdapterBoundary core_fault(contract(), throwing);
  expect_error(core_fault.submit_lidar(lidar(0, 1000000000LL)),
               ErrorCode::kCoreFailure);
  assert(core_fault.state() == BoundaryState::kFailed);
  expect_error(core_fault.submit_lidar(lidar(1, 1000000001LL)),
               ErrorCode::kPostEof);
  assert(throwing.lidar == 1);
}

void test_boundary_requires_si_imu_units() {
  RecordingSink sink;
  AdapterBoundary boundary(contract(), sink);
  auto sample = imu(0, 1000000000LL);
  sample.angular_velocity_unit = AngularVelocityUnit::kUnspecified;
  expect_error(boundary.submit_imu(sample), ErrorCode::kInvalidLayout);
  assert(sink.imu == 0);
}

void test_calibration_is_rigid_and_inverse_bound() {
  auto bad_row = contract();
  bad_row.calibration.transform[12] = 1.0;
  RecordingSink sink;
  AdapterBoundary row_boundary(bad_row, sink);
  assert(row_boundary.state() == BoundaryState::kFailed);
  assert(row_boundary.terminal_error().code == ErrorCode::kInvalidCalibration);

  auto bad_rotation = contract();
  bad_rotation.calibration.transform[0] = 2.0;
  AdapterBoundary rotation_boundary(bad_rotation, sink);
  assert(rotation_boundary.state() == BoundaryState::kFailed);
  assert(rotation_boundary.terminal_error().code == ErrorCode::kInvalidCalibration);

  auto bad_inverse = contract();
  bad_inverse.calibration.inverse_convention = CalibrationConvention::kTImuLidar;
  AdapterBoundary inverse_boundary(bad_inverse, sink);
  assert(inverse_boundary.state() == BoundaryState::kFailed);
  assert(inverse_boundary.terminal_error().code == ErrorCode::kMissingCalibration);
}

void test_boundary_eof_drain_output_contract_and_counters() {
  RecordingSink sink;
  AdapterBoundary boundary(contract(), sink);
  assert(boundary.submit_imu(imu(0, 1000000000LL)));
  assert(boundary.submit_lidar(lidar(1, 1000000000LL)));
  assert(boundary.request_eof());
  assert(boundary.begin_drain());

  TrajectorySample trajectory;
  trajectory.event = stamp(0, 2000000000LL);
  trajectory.frame_id = "world";
  trajectory.pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  assert(boundary.record_trajectory(trajectory));

  MapChunk map;
  map.order = 0;
  map.frame_id = "world";
  map.points = {PointRecord{0.0, 0.0, 0.0, 1.0, 0.0, 0, 0.0}};
  assert(boundary.record_map_chunk(map));
  assert(boundary.complete_drain());
  assert(boundary.state() == BoundaryState::kDrained);
  assert(boundary.counters().lidar_submitted == 1);
  assert(boundary.counters().lidar_accepted == 1);
  assert(boundary.counters().lidar_processed == 1);
  assert(boundary.counters().imu_submitted == 1);
  assert(boundary.counters().imu_accepted == 1);
  assert(boundary.counters().imu_processed == 1);
  assert(boundary.counters().trajectory_processed == 1);
  assert(boundary.counters().map_processed == 1);
  // Successful output is not a rejection or failure.  Rejection/failure
  // capacity is preflighted only for error paths and must not be counted on a
  // normal trajectory/map seal.
  assert(boundary.counters().trajectory_rejected == 0);
  assert(boundary.counters().map_rejected == 0);
  assert(boundary.counters().failures == 0);
  assert(boundary.counters().eof_requests == 1);
  assert(boundary.counters().drain_completions == 1);
  assert(sink.eof == 1 && sink.begin_drain == 1 && sink.complete == 1);
}

void test_trajectory_map_batch_is_atomic() {
  RecordingSink sink;
  AdapterBoundary boundary(contract(), sink);
  assert(boundary.request_eof());
  assert(boundary.begin_drain());

  TrajectorySample trajectory;
  trajectory.event = stamp(0, 1000000000LL);
  trajectory.frame_id = "world";
  trajectory.pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  // The empty map is deliberately invalid.  The boundary must validate both
  // batches before changing any trajectory/map counter or output order.
  const std::vector<MapChunk> invalid_maps(1);
  expect_error(boundary.record_output_batch({trajectory}, invalid_maps),
               ErrorCode::kEmptyPayload);
  assert(boundary.state() == BoundaryState::kFailed);
  assert(boundary.counters().trajectory_submitted == 0);
  assert(boundary.counters().trajectory_processed == 0);
  assert(boundary.counters().trajectory_outputs == 0);
  assert(boundary.counters().map_submitted == 0);
  assert(boundary.counters().map_processed == 0);
  assert(boundary.counters().map_outputs == 0);
  assert(boundary.counters().failures == 1);
}

void test_state_and_output_faults_have_no_sink_side_effects() {
  RecordingSink sink;
  AdapterBoundary duplicate_eof(contract(), sink);
  assert(duplicate_eof.request_eof());
  expect_error(duplicate_eof.request_eof(), ErrorCode::kInvalidState);
  assert(duplicate_eof.state() == BoundaryState::kFailed);
  expect_error(duplicate_eof.begin_drain(), ErrorCode::kInvalidState);
  assert(sink.begin_drain == 0);

  RecordingSink missing;
  AdapterBoundary missing_output(contract(), missing);
  assert(missing_output.request_eof());
  assert(missing_output.begin_drain());
  expect_error(missing_output.complete_drain(), ErrorCode::kInvalidState);
  assert(missing.complete == 0);

  RecordingSink output;
  AdapterBoundary wrong_output(contract(), output);
  assert(wrong_output.request_eof());
  assert(wrong_output.begin_drain());
  TrajectorySample bad;
  bad.event = stamp(0, 1);
  bad.frame_id = "world";
  bad.pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0};
  expect_error(wrong_output.record_trajectory(bad), ErrorCode::kInvalidLayout);
  assert(wrong_output.state() == BoundaryState::kFailed);
}

void test_counter_overflow_is_atomic_and_fault_latched() {
  const auto maximum = std::numeric_limits<std::uint64_t>::max();

  RecordingSink input_sink;
  AdapterBoundary input(contract(), input_sink);
  ConsumerCounters input_counters;
  input_counters.lidar_accepted = maximum;
  input.set_test_counters(input_counters);
  expect_error(input.submit_lidar(lidar(0, 1000000000LL)),
               ErrorCode::kCounterOverflow);
  assert(input_sink.lidar == 0);
  assert(input.counters().lidar_accepted == maximum);
  assert(input.counters().lidar_processed == 0);
  assert(input.state() == BoundaryState::kFailed);

  RecordingSink order_sink;
  AdapterBoundary order(contract(), order_sink);
  order.set_test_orders(maximum, 0, 0);
  expect_error(order.submit_lidar(lidar(maximum, 1000000000LL)),
               ErrorCode::kCounterOverflow);
  assert(order_sink.lidar == 0);
  assert(order.state() == BoundaryState::kFailed);

  RecordingSink failure_sink;
  AdapterBoundary saturated_failure(contract(), failure_sink);
  ConsumerCounters saturated;
  saturated.failures = maximum;
  saturated_failure.set_test_counters(saturated);
  auto malformed = lidar(0, 1000000000LL);
  malformed.frame_id.clear();
  expect_error(saturated_failure.submit_lidar(malformed),
               ErrorCode::kCounterOverflow);
  assert(saturated_failure.counters().failures == maximum);
  assert(saturated_failure.counters().lidar_rejected == 0);
  assert(saturated_failure.state() == BoundaryState::kFailed);

  RecordingSink rejected_sink;
  AdapterBoundary saturated_rejected(contract(), rejected_sink);
  ConsumerCounters rejected;
  rejected.lidar_rejected = maximum;
  saturated_rejected.set_test_counters(rejected);
  expect_error(saturated_rejected.submit_lidar(malformed),
               ErrorCode::kCounterOverflow);
  assert(saturated_rejected.counters().lidar_rejected == maximum);
  assert(saturated_rejected.counters().failures == 0);
  assert(saturated_rejected.state() == BoundaryState::kFailed);
}

void test_counter_overflow_precedes_eof_drain_and_output_side_effects() {
  const auto maximum = std::numeric_limits<std::uint64_t>::max();

  RecordingSink eof_sink;
  AdapterBoundary eof(contract(), eof_sink);
  ConsumerCounters eof_counters;
  eof_counters.eof_requests = maximum;
  eof.set_test_counters(eof_counters);
  expect_error(eof.request_eof(), ErrorCode::kCounterOverflow);
  assert(eof_sink.eof == 0);
  assert(eof.state() == BoundaryState::kFailed);

  RecordingSink drain_sink;
  AdapterBoundary drain(contract(), drain_sink);
  assert(drain.request_eof());
  assert(drain.begin_drain());
  ConsumerCounters drain_counters;
  drain_counters.trajectory_processed = 1;
  drain_counters.map_processed = 1;
  drain_counters.drain_completions = maximum;
  drain.set_test_counters(drain_counters);
  expect_error(drain.complete_drain(), ErrorCode::kCounterOverflow);
  assert(drain_sink.complete == 0);
  assert(drain.counters().drain_completions == maximum);
  assert(drain.state() == BoundaryState::kFailed);

  RecordingSink trajectory_sink;
  AdapterBoundary trajectory(contract(), trajectory_sink);
  assert(trajectory.request_eof());
  assert(trajectory.begin_drain());
  ConsumerCounters trajectory_counters;
  trajectory_counters.trajectory_accepted = maximum;
  trajectory.set_test_counters(trajectory_counters);
  TrajectorySample sample;
  sample.event = stamp(0, 1000000000LL);
  sample.frame_id = "world";
  sample.pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  expect_error(trajectory.record_trajectory(sample),
               ErrorCode::kCounterOverflow);
  assert(trajectory.counters().trajectory_accepted == maximum);
  assert(trajectory.counters().trajectory_processed == 0);
  assert(trajectory.state() == BoundaryState::kFailed);

  RecordingSink map_sink;
  AdapterBoundary map(contract(), map_sink);
  assert(map.request_eof());
  assert(map.begin_drain());
  ConsumerCounters map_counters;
  map_counters.map_accepted = maximum;
  map.set_test_counters(map_counters);
  MapChunk chunk;
  chunk.order = 0;
  chunk.frame_id = "world";
  chunk.points = {PointRecord{0.0, 0.0, 0.0, 1.0, 0.0, 0, 0.0}};
  expect_error(map.record_map_chunk(chunk), ErrorCode::kCounterOverflow);
  assert(map.counters().map_accepted == maximum);
  assert(map.counters().map_processed == 0);
  assert(map.state() == BoundaryState::kFailed);
}

}  // namespace

int main() {
  test_parser_preserves_typed_fields_unit_and_padding();
  test_parser_decodes_big_endian();
  test_parser_rejects_missing_duplicate_overlap_and_wrong_mapping();
  test_parser_rejects_units_raw_range_and_layout_overflow();
  test_ros_stamp_constructor_is_integer_checked();
  test_sequence_rejects_gt_and_scorer_paths();
  test_boundary_enforces_integer_order_frames_and_fault_latch();
  test_boundary_rejects_timestamp_regression_and_core_throw();
  test_boundary_requires_si_imu_units();
  test_calibration_is_rigid_and_inverse_bound();
  test_boundary_eof_drain_output_contract_and_counters();
  test_trajectory_map_batch_is_atomic();
  test_state_and_output_faults_have_no_sink_side_effects();
  test_counter_overflow_is_atomic_and_fault_latched();
  test_counter_overflow_precedes_eof_drain_and_output_side_effects();
  std::cout << "glim_clean_room Phase 0.3 contract tests: 15 groups passed\n";
  return 0;
}
