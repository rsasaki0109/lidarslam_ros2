#include "glim_clean_room/phase3a_conversion.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

namespace {

using namespace glim_clean_room;
using namespace glim_clean_room::phase3a;

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
  value.sequence_id = "phase3a-synthetic";
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
  value.artifacts = {{ArtifactRole::kInput, "input/sequence"},
                     {ArtifactRole::kCalibration, "calibration/imu_lidar"},
                     {ArtifactRole::kConfiguration, "config/adapter"},
                     {ArtifactRole::kCoreSource, "source/core"}};
  return value;
}

EventStamp event(std::uint64_t order, StampNanoseconds nanoseconds) {
  return EventStamp{order, nanoseconds};
}

LidarFrame lidar_frame(std::uint64_t order, StampNanoseconds nanoseconds) {
  LidarFrame frame;
  frame.event = event(order, nanoseconds);
  frame.frame_id = "lidar";
  frame.point_time_unit = PointTimeUnit::kMilliseconds;
  frame.field_mapping = mapping();
  frame.points = {{1.0, 2.0, 3.0, 4.0, 0.0, 7, 0.0},
                  {-1.0, 0.5, 6.0, 8.0, 1.5, 9, 1500.0}};
  return frame;
}

ImuSample imu_sample() {
  ImuSample sample;
  sample.event = event(0, -1000000001LL);
  sample.frame_id = "imu";
  sample.linear_acceleration = {0.0, 0.0, 9.81};
  sample.angular_velocity = {0.1, -0.2, 0.3};
  sample.acceleration_unit = AccelerationUnit::kMetersPerSecondSquared;
  sample.angular_velocity_unit = AngularVelocityUnit::kRadiansPerSecond;
  return sample;
}

template <typename T>
void expect_error(const Result<T>& result, ErrorCode code) {
  assert(!result);
  assert(result.error.code == code);
}

void expect_error(const Status& status, ErrorCode code) {
  assert(!status);
  assert(status.error.code == code);
}

void test_checked_seconds_and_raw_conversion() {
  const auto negative = checked_event_seconds(event(0, -1));
  assert(negative);
  assert(negative.value < 0.0);
  const auto minimum = checked_event_seconds(
      event(0, std::numeric_limits<StampNanoseconds>::min()));
  assert(minimum && std::isfinite(minimum.value));

  const auto sequence = contract();
  StampLedger ledger(4);
  const auto converted = to_core_raw_points(
      lidar_frame(0, 1700000000000000000LL), sequence, ledger);
  assert(converted);
  assert(converted.value->size() == 2);
  assert(converted.value->intensities.size() == 2);
  assert(converted.value->rings.size() == 2);
  assert(converted.value->intensities[1] == 8.0);
  assert(converted.value->rings[1] == 9U);
  assert(converted.value->points[0].w() == 1.0);
  assert(converted.value->points[1].w() == 1.0);
  assert(converted.value->times[0] == 0.0);
  assert(std::abs(converted.value->times[1] - 1.5) < 1e-12);
  assert(validate_core_raw_points(*converted.value));
  assert(ledger.size() == 1);
  assert(ledger.emitted_count() == 0);
  assert(!ledger.all_emitted());
}

void test_core_raw_shape_rejects_vector_mismatch_and_bad_w() {
  auto sequence = contract();
  StampLedger ledger(3);
  auto converted = to_core_raw_points(lidar_frame(0, 1000000000LL),
                                      sequence, ledger);
  assert(converted);
  converted.value->times.pop_back();
  expect_error(validate_core_raw_points(*converted.value),
               ErrorCode::kUnsupportedCoreOutput);

  StampLedger second_ledger(3);
  converted = to_core_raw_points(lidar_frame(0, 1000000000LL),
                                 sequence, second_ledger);
  assert(converted);
  converted.value->points[0].w() = 0.0;
  expect_error(validate_core_raw_points(*converted.value),
               ErrorCode::kUnsupportedCoreOutput);
}

void test_raw_conversion_rejects_size_and_time_failures() {
  auto sequence = contract();
  StampLedger ledger(4);
  auto frame = lidar_frame(0, 1000000000LL);
  frame.points[0].x = std::numeric_limits<double>::infinity();
  expect_error(to_core_raw_points(frame, sequence, ledger), ErrorCode::kNonFinite);
  assert(ledger.size() == 0);

  auto bad_time = lidar_frame(0, 1000000000LL);
  bad_time.points[0].relative_time_raw =
      std::numeric_limits<double>::quiet_NaN();
  expect_error(to_core_raw_points(bad_time, sequence, ledger),
               ErrorCode::kNonFinite);
  assert(ledger.size() == 0);

  auto bad_unit = lidar_frame(0, 1000000000LL);
  bad_unit.point_time_unit = PointTimeUnit::kSeconds;
  expect_error(to_core_raw_points(bad_unit, sequence, ledger),
               ErrorCode::kWrongTimeUnit);
}

void test_imu_conversion_is_typed_and_checked() {
  const auto converted = to_core_imu(imu_sample(), "imu");
  assert(converted);
  assert(converted.value.stamp_seconds < -1.0);
  assert(converted.value.linear_acceleration.z() == 9.81);
  assert(converted.value.angular_velocity.y() == -0.2);

  auto bad = imu_sample();
  bad.linear_acceleration[0] = std::numeric_limits<double>::quiet_NaN();
  expect_error(to_core_imu(bad, "imu"), ErrorCode::kNonFinite);
  bad = imu_sample();
  bad.angular_velocity[2] = std::numeric_limits<double>::infinity();
  expect_error(to_core_imu(bad, "imu"), ErrorCode::kNonFinite);
  bad = imu_sample();
  bad.frame_id = "other";
  expect_error(to_core_imu(bad, "imu"), ErrorCode::kInvalidLayout);
}

void test_ledger_order_capacity_and_collision_latch() {
  StampLedger capacity(1);
  assert(capacity.bind_input(event(0, 1000000000LL)));
  expect_error(capacity.bind_input(event(1, 2000000000LL)),
               ErrorCode::kLedgerOverflow);
  assert(capacity.terminal());
  expect_error(capacity.lookup_output(1.0), ErrorCode::kLedgerOverflow);

  StampLedger order(3);
  assert(order.bind_input(event(0, 0)));
  assert(order.bind_input(event(2, 1000000000LL)));
  expect_error(order.bind_input(event(1, 2000000000LL)),
               ErrorCode::kOrderViolation);
  assert(order.terminal());

  StampLedger regression(3);
  assert(regression.bind_input(event(0, 2000000000LL)));
  expect_error(regression.bind_input(event(1, 1000000000LL)),
               ErrorCode::kNonMonotonicStamp);

  // At this epoch a one-nanosecond change is below one core-double ULP and
  // therefore must never be silently assigned a distinct output identity.
  const StampNanoseconds large_epoch = 1000000000000000000LL;
  StampLedger collision(3);
  assert(collision.bind_input(event(0, large_epoch)));
  expect_error(collision.bind_input(event(1, large_epoch + 1)),
               ErrorCode::kStampLedgerAmbiguous);

  const auto large_seconds =
      checked_event_seconds(event(0, large_epoch));
  assert(large_seconds);
  const double next_double = std::nextafter(
      large_seconds.value, std::numeric_limits<double>::infinity());
  StampNanoseconds near_nanoseconds = 0;
  for (StampNanoseconds offset = 1; offset < 4096; ++offset) {
    const auto candidate = checked_event_seconds(
        event(1, large_epoch + offset));
    if (candidate && candidate.value == next_double) {
      near_nanoseconds = large_epoch + offset;
      break;
    }
  }
  assert(near_nanoseconds != 0);
  StampLedger near_collision(3);
  assert(near_collision.bind_input(event(0, large_epoch)));
  expect_error(near_collision.bind_input(event(1, near_nanoseconds)),
               ErrorCode::kStampLedgerAmbiguous);
}

void test_ledger_ulp_matching_unmatched_and_reuse() {
  StampLedger ledger(3);
  const auto bound = ledger.bind_input(event(0, 1000000000LL));
  assert(bound);
  const double adjacent = std::nextafter(bound.value,
                                          std::numeric_limits<double>::infinity());
  const auto lookup = ledger.lookup_output(adjacent);
  assert(lookup && lookup.value.event.nanoseconds == 1000000000LL);
  assert(ledger.commit_output(lookup.value));
  expect_error(ledger.lookup_output(bound.value),
               ErrorCode::kStampLedgerReused);

  StampLedger unmatched(2);
  assert(unmatched.bind_input(event(0, 1000000000LL)));
  expect_error(unmatched.lookup_output(123.5),
               ErrorCode::kStampLedgerUnmatched);

  StampLedger changed_lookup(2);
  assert(changed_lookup.bind_input(event(0, 1000000000LL)));
  const auto pending = changed_lookup.lookup_output(1.0);
  assert(pending);
  const StampLedger::Lookup forged{pending.value.index,
                                   event(7, pending.value.event.nanoseconds)};
  expect_error(changed_lookup.commit_output(forged),
               ErrorCode::kStampLedgerAmbiguous);
}

std::shared_ptr<glim::EstimationFrame> estimation_frame(double stamp);

void test_mixed_global_order_and_independent_trajectory_order() {
  const auto sequence = contract();
  StampLedger ledger(4);
  const auto lidar_zero = to_core_raw_points(
      lidar_frame(0, 1000000000LL), sequence, ledger);
  assert(lidar_zero);

  auto imu = imu_sample();
  imu.event = event(1, 1500000000LL);
  assert(to_core_imu(imu, "imu"));
  assert(ledger.size() == 1);

  const auto lidar_two = to_core_raw_points(
      lidar_frame(2, 2000000000LL), sequence, ledger);
  assert(lidar_two);
  assert(ledger.size() == 2);
  const auto preserved_global_order = ledger.lookup_output(lidar_two.value->stamp);
  assert(preserved_global_order && preserved_global_order.value.event.order == 2);

  TrajectoryConverter converter(sequence);
  const auto first = converter.convert(estimation_frame(lidar_zero.value->stamp),
                                       ledger);
  assert(first);
  const auto second = converter.convert(estimation_frame(lidar_two.value->stamp),
                                        ledger);
  assert(second);
  assert(first.value.event.nanoseconds == 1000000000LL);
  assert(second.value.event.nanoseconds == 2000000000LL);
  assert(first.value.event.order == 0);
  assert(second.value.event.order == 1);
  assert(ledger.emitted_count() == 2);
  assert(ledger.all_emitted());
}

std::shared_ptr<glim::EstimationFrame> estimation_frame(double stamp) {
  auto frame = std::make_shared<glim::EstimationFrame>();
  frame->stamp = stamp;
  frame->T_world_lidar.setIdentity();
  frame->frame_id = glim::FrameID::IMU;
  return frame;
}

void test_trajectory_identity_and_canonical_quaternion() {
  StampLedger ledger(2);
  const auto bound = ledger.bind_input(event(0, 5000000001LL));
  assert(bound);
  auto frame = estimation_frame(bound.value);
  frame->T_world_lidar.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  TrajectoryConverter converter(contract());
  const auto output = converter.convert(frame, ledger);
  assert(output);
  assert(output.value.event.nanoseconds == 5000000001LL);
  assert(output.value.event.order == 0);
  assert(output.value.frame_id == "world");
  assert(output.value.pose[0] == 1.0 && output.value.pose[2] == 3.0);
  assert(output.value.pose[6] >= 0.0);
  assert(converter.next_order() == 1);

  StampLedger half_turn_ledger(2);
  const auto half_bound = half_turn_ledger.bind_input(event(0, 6000000001LL));
  assert(half_bound);
  auto half_turn = estimation_frame(half_bound.value);
  const double pi = std::acos(-1.0);
  half_turn->T_world_lidar.linear() =
      Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  TrajectoryConverter half_converter(contract());
  const auto half_output = half_converter.convert(half_turn, half_turn_ledger);
  assert(half_output);
  assert(half_output.value.pose[3] >= 0.0);
}

void test_trajectory_rejects_bad_transform_and_reuse() {
  StampLedger bad_row_ledger(2);
  const auto bound = bad_row_ledger.bind_input(event(0, 1000000000LL));
  assert(bound);
  auto bad_row = estimation_frame(bound.value);
  bad_row->T_world_lidar.matrix()(3, 3) = 2.0;
  TrajectoryConverter row_converter(contract());
  expect_error(row_converter.convert(bad_row, bad_row_ledger),
               ErrorCode::kInvalidCalibration);
  assert(row_converter.terminal());

  StampLedger nan_ledger(2);
  const auto nan_bound = nan_ledger.bind_input(event(0, 2000000000LL));
  assert(nan_bound);
  auto nan_frame = estimation_frame(nan_bound.value);
  nan_frame->T_world_lidar.linear()(0, 0) =
      std::numeric_limits<double>::quiet_NaN();
  TrajectoryConverter nan_converter(contract());
  expect_error(nan_converter.convert(nan_frame, nan_ledger),
               ErrorCode::kInvalidCalibration);

  StampLedger reuse_ledger(2);
  const auto reuse_bound = reuse_ledger.bind_input(event(0, 3000000000LL));
  assert(reuse_bound);
  auto reuse_frame = estimation_frame(reuse_bound.value);
  TrajectoryConverter reuse_converter(contract());
  assert(reuse_converter.convert(reuse_frame, reuse_ledger));
  expect_error(reuse_converter.convert(reuse_frame, reuse_ledger),
               ErrorCode::kStampLedgerReused);

  expect_error(map_conversion_not_implemented(), ErrorCode::kNotImplemented);
}

}  // namespace

int main() {
  test_checked_seconds_and_raw_conversion();
  test_core_raw_shape_rejects_vector_mismatch_and_bad_w();
  test_raw_conversion_rejects_size_and_time_failures();
  test_imu_conversion_is_typed_and_checked();
  test_ledger_order_capacity_and_collision_latch();
  test_ledger_ulp_matching_unmatched_and_reuse();
  test_mixed_global_order_and_independent_trajectory_order();
  test_trajectory_identity_and_canonical_quaternion();
  test_trajectory_rejects_bad_transform_and_reuse();
  std::cout << "glim_clean_room Phase 3a conversion tests: 8 groups passed\n";
  return 0;
}
