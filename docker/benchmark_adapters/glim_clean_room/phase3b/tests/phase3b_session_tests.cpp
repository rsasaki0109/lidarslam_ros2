#include "glim_clean_room/phase3b_session.hpp"

#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

using namespace glim_clean_room;
using namespace glim_clean_room::phase3b;

std::array<double, 16> identity_transform() {
  return {1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0};
}

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

SequenceContract contract(bool require_map = false) {
  SequenceContract value;
  value.sequence_id = "phase3b-synthetic";
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
  value.require_trajectory_output = true;
  value.require_map_output = require_map;
  value.artifacts = {{ArtifactRole::kInput, "input/synthetic"},
                     {ArtifactRole::kCalibration, "calibration/synthetic"},
                     {ArtifactRole::kConfiguration, "config/synthetic"},
                     {ArtifactRole::kCoreSource, "source/pinned-core"}};
  return value;
}

EventStamp event(std::uint64_t order, StampNanoseconds nanoseconds) {
  return EventStamp{order, nanoseconds};
}

ImuSample imu(std::uint64_t order, StampNanoseconds nanoseconds) {
  ImuSample value;
  value.event = event(order, nanoseconds);
  value.frame_id = "imu";
  value.linear_acceleration = {0.0, 0.0, 9.81};
  value.angular_velocity = {0.0, 0.0, 0.0};
  value.acceleration_unit = AccelerationUnit::kMetersPerSecondSquared;
  value.angular_velocity_unit = AngularVelocityUnit::kRadiansPerSecond;
  return value;
}

LidarFrame lidar(std::uint64_t order, StampNanoseconds nanoseconds,
                 double shift) {
  LidarFrame value;
  value.event = event(order, nanoseconds);
  value.frame_id = "lidar";
  value.point_time_unit = PointTimeUnit::kMilliseconds;
  value.field_mapping = mapping();
  value.points.reserve(128);
  for (std::uint32_t row = 0; row < 8; ++row) {
    for (std::uint32_t column = 0; column < 16; ++column) {
      const double x = 1.0 + 0.11 * static_cast<double>(column) + shift;
      const double y = 2.0 + 0.13 * static_cast<double>(row);
      const double z = 3.0 + 0.02 * static_cast<double>((row + column) % 4);
      value.points.push_back(
          PointRecord{x, y, z, 1.0 + static_cast<double>(column), 0.0,
                      row, 0.0});
    }
  }
  return value;
}

void write_file(const std::filesystem::path& path, const std::string& content) {
  std::ofstream output(path);
  assert(output.good());
  output << content;
  output.close();
  assert(output.good());
}

std::filesystem::path make_config() {
  const std::filesystem::path root(GLIM_CLEAN_ROOM_PHASE3B_TEST_ROOT);
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

std::filesystem::path make_preflight_root(const std::string& name) {
  const auto root = std::filesystem::path(GLIM_CLEAN_ROOM_PHASE3B_TEST_ROOT) /
                    ("config-preflight-" + name);
  std::filesystem::create_directories(root);
  return root;
}

void write_preflight_components(const std::filesystem::path& root) {
  write_file(root / "config_logging.json", R"({"logging":{}})");
  write_file(root / "config_sensors.json", R"({"sensors":{}})");
  write_file(root / "config_preprocess.json", R"({"preprocess":{}})");
  write_file(root / "config_odometry.json", R"({"odometry_estimation":{}})");
  write_file(root / "config_sub_mapping.json", R"({"sub_mapping":{}})");
  write_file(root / "config_global_mapping.json", R"({"global_mapping":{}})");
}

void write_preflight_manifest(const std::filesystem::path& root,
                              const std::string& logging_file) {
  write_file(
      root / "config.json",
      "{\"global\":{\"config_path\":\"\",\"config_logging\":\"" +
          logging_file +
          "\",\"config_sensors\":\"config_sensors.json\",\"config_preprocess\":\"config_preprocess.json\",\"config_odometry\":\"config_odometry.json\",\"config_sub_mapping\":\"config_sub_mapping.json\",\"config_global_mapping\":\"config_global_mapping.json\"}}");
}

void assert_config_rejected(const std::filesystem::path& root,
                            const std::string& detail_token,
                            bool require_map = false) {
  RuntimeOptions options;
  options.config_directory = root.string();
  options.ledger_capacity = 2;
  options.max_pending_trajectory = 2;
  auto no_output_contract = contract(require_map);
  no_output_contract.require_trajectory_output = false;
  const auto created = GlimCoreSession::create(no_output_contract, options);
  assert(!created);
  assert(created.error.code == ErrorCode::kCoreFailure);
  assert(created.error.detail.find(detail_token) != std::string::npos);
}

void test_config_preflight_is_strict_and_core_free() {
  const auto legacy = make_preflight_root("legacy-four-file");
  write_preflight_components(legacy);
  write_file(legacy / "config.json",
             R"({"global":{"config_logging":"config_logging.json","config_sensors":"config_sensors.json","config_preprocess":"config_preprocess.json","config_odometry":"config_odometry.json"}})");
  RuntimeOptions legacy_options;
  legacy_options.config_directory = legacy.string();
  legacy_options.ledger_capacity = 2;
  legacy_options.max_pending_trajectory = 2;
  auto legacy_contract = contract(false);
  legacy_contract.require_trajectory_output = false;
  auto legacy_session = GlimCoreSession::create(legacy_contract, legacy_options);
  assert(legacy_session);
  legacy_session.value.reset();

  const auto missing = make_preflight_root("missing");
  write_preflight_components(missing);
  write_file(missing / "config.json",
             R"({"global":{"config_logging":"config_logging.json"}})");
  assert_config_rejected(missing, "config_sensors");

  const auto malformed = make_preflight_root("malformed");
  write_preflight_components(malformed);
  write_preflight_manifest(malformed, "config_logging.json");
  write_file(malformed / "config_logging.json", "{malformed");
  assert_config_rejected(malformed, "malformed JSON");

  const auto traversal = make_preflight_root("traversal");
  write_preflight_components(traversal);
  write_preflight_manifest(traversal, "../outside.json");
  assert_config_rejected(traversal, "traversal");

  const auto absolute = make_preflight_root("absolute");
  write_preflight_components(absolute);
  const auto absolute_file = absolute.parent_path() / "absolute-outside.json";
  write_file(absolute_file, R"({})");
  write_preflight_manifest(absolute, absolute_file.string());
  assert_config_rejected(absolute, "absolute");

  const auto symlink = make_preflight_root("symlink");
  write_preflight_components(symlink);
  write_preflight_manifest(symlink, "config_logging.json");
  const auto outside = symlink.parent_path() / "symlink-outside.json";
  write_file(outside, R"({})");
  std::error_code error;
  std::filesystem::remove(symlink / "config_logging.json", error);
  assert(!error);
  std::filesystem::create_symlink(outside, symlink / "config_logging.json",
                                  error);
  assert(!error);
  assert_config_rejected(symlink, "symlink");

  const auto root_target = make_preflight_root("root-target");
  write_preflight_components(root_target);
  write_preflight_manifest(root_target, "config_logging.json");
  const auto root_link = root_target.parent_path() / "root-link";
  std::filesystem::remove(root_link, error);
  assert(!error);
  std::filesystem::create_directory_symlink(root_target, root_link, error);
  assert(!error);
  assert_config_rejected(root_link, "config_directory");

  const auto mapping_missing = make_preflight_root("mapping-missing");
  write_preflight_components(mapping_missing);
  write_file(mapping_missing / "config_global_mapping.json",
             R"({"global_mapping":{}})");
  write_file(mapping_missing / "config.json",
             R"({"global":{"config_logging":"config_logging.json","config_sensors":"config_sensors.json","config_preprocess":"config_preprocess.json","config_odometry":"config_odometry.json","config_global_mapping":"config_global_mapping.json"}})");
  assert_config_rejected(mapping_missing, "config_sub_mapping", true);
}

void print_failure(const Status& status, const char* operation) {
  if (!status) {
    std::cerr << operation << " failed: " << error_code_name(status.error.code)
              << " " << status.error.detail << "\n";
  }
}

void test_exact_core_vertical_slice() {
  const auto config_directory = make_config();
  RuntimeOptions options;
  options.config_directory = config_directory.string();
  options.ledger_capacity = 8;
  options.max_pending_trajectory = 8;

  auto created = GlimCoreSession::create(contract(false), options);
  assert(created);
  auto session = std::move(created.value);

  // The first two events make the NAIVE initializer ready before the first
  // LiDAR frame.  The current return value of insert_frame() is not
  // republished because the same frame may later be returned by
  // get_remaining_frames().  The two drain remainders are staged and then
  // sealed as one transaction; every bound stamp is proved emitted before
  // publication.
  //
  // The first initialized frame is followed by a second frame to exercise
  // both initialized LiDAR frames through the drain-remainder path without
  // ever publishing
  // insert_frame()'s current return directly.
  auto status = session->submit_imu(imu(0, 1000000000LL));
  print_failure(status, "submit imu 0");
  assert(status);
  status = session->submit_imu(imu(1, 2000000000LL));
  print_failure(status, "submit imu 1");
  assert(status);
  status = session->submit_lidar(lidar(2, 2000000000LL, 0.0));
  print_failure(status, "submit lidar 0");
  assert(status);
  status = session->submit_imu(imu(3, 3000000000LL));
  print_failure(status, "submit imu 2");
  assert(status);
  status = session->submit_lidar(lidar(4, 3000000000LL, 0.02));
  print_failure(status, "submit lidar 1");
  assert(status);

  status = session->close();
  print_failure(status, "close");
  assert(status);
  assert(session->close());
  assert(session->boundary().state() == BoundaryState::kDrained);
  assert(session->boundary().counters().trajectory_rejected == 0);
  assert(session->boundary().counters().failures == 0);
  assert(session->boundary().counters().trajectory_processed == 2);
  assert(session->boundary().counters().trajectory_outputs == 2);

  const auto trajectory = session->take_trajectory();
  assert(trajectory);
  assert(trajectory.value.size() == 2);
  assert(trajectory.value[0].event.order == 0);
  assert(trajectory.value[1].event.order == 1);
  assert(trajectory.value[0].event.nanoseconds == 2000000000LL);
  assert(trajectory.value[1].event.nanoseconds == 3000000000LL);
  assert(!session->take_trajectory());
  const auto maps = session->take_map_chunks();
  assert(maps && maps.value.empty());
}

void test_complete_map_vertical_slice() {
  RuntimeOptions options;
  options.config_directory = make_config().string();
  options.ledger_capacity = 8;
  options.max_pending_trajectory = 8;
  options.max_map_frames = 8;
  options.max_map_submaps = 4;
  options.max_map_points = 100000;
  options.max_map_chunks = 1;

  auto created = GlimCoreSession::create(contract(true), options);
  assert(created);
  auto session = std::move(created.value);
  assert(session->submit_imu(imu(0, 1000000000LL)));
  assert(session->submit_imu(imu(1, 2000000000LL)));
  assert(session->submit_lidar(lidar(2, 2000000000LL, 0.0)));
  assert(session->submit_imu(imu(3, 3000000000LL)));
  assert(session->submit_lidar(lidar(4, 3000000000LL, 0.02)));
  const auto closed = session->close();
  print_failure(closed, "complete map close");
  assert(closed);
  assert(session->boundary().counters().trajectory_rejected == 0);
  assert(session->boundary().counters().map_rejected == 0);
  assert(session->boundary().counters().failures == 0);
  assert(session->boundary().counters().trajectory_processed == 2);
  assert(session->boundary().counters().map_processed == 1);
  assert(session->boundary().counters().map_outputs == 1);

  const auto trajectory = session->take_trajectory();
  assert(trajectory && trajectory.value.size() == 2);
  const auto maps = session->take_map_chunks();
  assert(maps);
  assert(maps.value.size() == 1);
  assert(maps.value[0].order == 0);
  assert(maps.value[0].frame_id == "world");
  assert(!maps.value[0].points.empty());
  assert(!session->take_map_chunks());
}

void test_map_point_bound_fails_before_publication() {
  RuntimeOptions options;
  options.config_directory = make_config().string();
  options.ledger_capacity = 8;
  options.max_pending_trajectory = 8;
  options.max_map_frames = 8;
  options.max_map_submaps = 4;
  options.max_map_points = 1;
  options.max_map_chunks = 1;

  auto created = GlimCoreSession::create(contract(true), options);
  assert(created);
  auto session = std::move(created.value);
  assert(session->submit_imu(imu(0, 1000000000LL)));
  assert(session->submit_imu(imu(1, 2000000000LL)));
  assert(session->submit_lidar(lidar(2, 2000000000LL, 0.0)));
  assert(session->submit_imu(imu(3, 3000000000LL)));
  assert(session->submit_lidar(lidar(4, 3000000000LL, 0.02)));
  const auto closed = session->close();
  print_failure(closed, "bounded map close");
  assert(!closed);
  assert(closed.error.code == ErrorCode::kCoreFailure);
  assert(closed.error.detail.find("unsupported_core_output") !=
         std::string::npos);
  assert(session->boundary().state() == BoundaryState::kFailed);
  assert(session->boundary().counters().trajectory_outputs == 0);
  assert(session->boundary().counters().map_outputs == 0);
  assert(session->boundary().counters().failures == 1);
  const auto maps = session->take_map_chunks();
  assert(!maps && maps.error.code == ErrorCode::kInvalidState);
}

void test_map_callback_reentry_fails_before_atomic_seal() {
  RuntimeOptions options;
  options.config_directory = make_config().string();
  options.ledger_capacity = 8;
  options.max_pending_trajectory = 8;
  options.max_map_frames = 8;
  options.max_map_submaps = 4;
  options.max_map_points = 100000;
  options.max_map_chunks = 1;

  auto created = GlimCoreSession::create(contract(true), options);
  assert(created);
  auto session = std::move(created.value);
  GlimCoreSession* session_ptr = session.get();
  const int callback_id = glim::SubMappingCallbacks::on_insert_frame.add(
      [session_ptr](const glim::EstimationFrame::ConstPtr&) {
        // Ignore the nested rejection deliberately.  finalize_map must check
        // the session latch after every callback-capable GLIM call and must
        // not reach the atomic trajectory+map seal.
        (void)session_ptr->submit_imu(imu(5, 4000000000LL));
      });
  assert(session->submit_imu(imu(0, 1000000000LL)));
  assert(session->submit_imu(imu(1, 2000000000LL)));
  assert(session->submit_lidar(lidar(2, 2000000000LL, 0.0)));
  assert(session->submit_imu(imu(3, 3000000000LL)));
  assert(session->submit_lidar(lidar(4, 3000000000LL, 0.02)));
  const auto closed = session->close();
  glim::SubMappingCallbacks::on_insert_frame.remove(callback_id);
  assert(!closed);
  assert(session->boundary().state() == BoundaryState::kFailed);
  assert(session->boundary().counters().trajectory_outputs == 0);
  assert(session->boundary().counters().map_outputs == 0);
  assert(session->boundary().counters().trajectory_processed == 0);
  assert(session->boundary().counters().map_processed == 0);
}

void test_map_callback_exception_fails_before_atomic_seal() {
  RuntimeOptions options;
  options.config_directory = make_config().string();
  options.ledger_capacity = 8;
  options.max_pending_trajectory = 8;
  options.max_map_frames = 8;
  options.max_map_submaps = 4;
  options.max_map_points = 100000;
  options.max_map_chunks = 1;

  auto created = GlimCoreSession::create(contract(true), options);
  assert(created);
  auto session = std::move(created.value);
  const int callback_id = glim::SubMappingCallbacks::on_insert_frame.add(
      [](const glim::EstimationFrame::ConstPtr&) {
        throw std::runtime_error("synthetic mapping callback failure");
      });
  assert(session->submit_imu(imu(0, 1000000000LL)));
  assert(session->submit_imu(imu(1, 2000000000LL)));
  assert(session->submit_lidar(lidar(2, 2000000000LL, 0.0)));
  assert(session->submit_imu(imu(3, 3000000000LL)));
  assert(session->submit_lidar(lidar(4, 3000000000LL, 0.02)));
  const auto closed = session->close();
  glim::SubMappingCallbacks::on_insert_frame.remove(callback_id);
  assert(!closed);
  assert(session->boundary().state() == BoundaryState::kFailed);
  assert(session->boundary().counters().trajectory_outputs == 0);
  assert(session->boundary().counters().map_outputs == 0);
  assert(session->boundary().counters().failures == 1);
}

void test_core_callback_exception_faults_without_publication() {
  RuntimeOptions options;
  options.config_directory = make_config().string();
  options.ledger_capacity = 4;
  options.max_pending_trajectory = 4;
  auto created = GlimCoreSession::create(contract(false), options);
  assert(created);
  auto session = std::move(created.value);

  const int callback_id = glim::OdometryEstimationCallbacks::on_insert_imu.add(
      [](double, const Eigen::Vector3d&, const Eigen::Vector3d&) {
        throw std::runtime_error("synthetic callback failure");
      });
  const auto status = session->submit_imu(imu(0, 0));
  glim::OdometryEstimationCallbacks::on_insert_imu.remove(callback_id);
  assert(!status);
  assert(status.error.code == ErrorCode::kCoreFailure);
  assert(session->boundary().state() == BoundaryState::kFailed);
  assert(session->boundary().counters().trajectory_outputs == 0);
  assert(session->boundary().counters().failures == 1);
}

void test_reentrant_callback_return_is_not_accepted_as_success() {
  RuntimeOptions options;
  options.config_directory = make_config().string();
  options.ledger_capacity = 4;
  options.max_pending_trajectory = 4;
  auto created = GlimCoreSession::create(contract(false), options);
  assert(created);
  auto session = std::move(created.value);
  GlimCoreSession* session_ptr = session.get();
  const int callback_id = glim::OdometryEstimationCallbacks::on_insert_imu.add(
      [session_ptr](double, const Eigen::Vector3d&, const Eigen::Vector3d&) {
        // Deliberately ignore the nested failure.  The outer core call must
        // re-check the session latch instead of reporting processed success.
        (void)session_ptr->submit_imu(imu(1, 2000000000LL));
      });
  const auto status = session->submit_imu(imu(0, 1000000000LL));
  glim::OdometryEstimationCallbacks::on_insert_imu.remove(callback_id);
  assert(!status);
  assert(status.error.code == ErrorCode::kCoreFailure);
  assert(session->boundary().state() == BoundaryState::kFailed);
  assert(session->boundary().counters().imu_processed == 0);
  assert(session->boundary().counters().failures == 1);
}

void test_global_config_lease_is_bounded_and_fail_closed() {
  RuntimeOptions options;
  options.config_directory = make_config().string();
  options.ledger_capacity = 2;
  options.max_pending_trajectory = 2;
  auto no_output_contract = contract(false);
  no_output_contract.require_trajectory_output = false;
  auto first = GlimCoreSession::create(no_output_contract, options);
  assert(first);
  auto second = GlimCoreSession::create(no_output_contract, options);
  assert(!second);
  assert(second.error.code == ErrorCode::kCoreFailure);
  assert(second.error.detail.find("config lease") != std::string::npos);
  assert(first.value->close());
}

void test_counter_overflow_and_map_contract_fail_closed() {
  RuntimeOptions options;
  options.config_directory = make_config().string();
  options.ledger_capacity = 2;
  options.max_pending_trajectory = 2;
  auto created = GlimCoreSession::create(contract(false), options);
  assert(created);
  auto session = std::move(created.value);
  ConsumerCounters counters;
  counters.lidar_accepted = std::numeric_limits<std::uint64_t>::max();
  session->set_test_counters(counters);
  const auto overflow = session->submit_lidar(lidar(0, 1000000000LL, 0.0));
  assert(!overflow && overflow.error.code == ErrorCode::kCounterOverflow);
  assert(session->boundary().state() == BoundaryState::kFailed);
  assert(session->boundary().counters().lidar_accepted ==
         std::numeric_limits<std::uint64_t>::max());
  assert(session->boundary().counters().lidar_processed == 0);

  // The failed session must release the process-global GLIM configuration
  // lease before the independent batch-overflow fixture is constructed.
  session.reset();

  auto batch_created = GlimCoreSession::create(contract(false), options);
  assert(batch_created);
  auto batch_session = std::move(batch_created.value);
  assert(batch_session->submit_imu(imu(0, 1000000000LL)));
  assert(batch_session->submit_imu(imu(1, 2000000000LL)));
  assert(batch_session->submit_lidar(lidar(2, 2000000000LL, 0.0)));
  ConsumerCounters batch_counters;
  batch_counters.trajectory_outputs =
      std::numeric_limits<std::uint64_t>::max();
  batch_session->set_test_counters(batch_counters);
  const auto batch_close = batch_session->close();
  assert(!batch_close && batch_close.error.code == ErrorCode::kCounterOverflow);
  assert(batch_session->boundary().state() == BoundaryState::kFailed);
  assert(batch_session->boundary().counters().trajectory_outputs ==
         std::numeric_limits<std::uint64_t>::max());
  assert(batch_session->boundary().counters().trajectory_processed == 0);
  assert(batch_session->boundary().counters().failures == 1);

  batch_session.reset();
  auto map_created = GlimCoreSession::create(contract(true), options);
  assert(map_created);
  map_created.value.reset();
}

void test_map_batch_counter_overflow_has_no_publication() {
  RuntimeOptions options;
  options.config_directory = make_config().string();
  options.ledger_capacity = 8;
  options.max_pending_trajectory = 8;
  options.max_map_frames = 8;
  options.max_map_submaps = 4;
  options.max_map_points = 100000;
  options.max_map_chunks = 1;

  auto created = GlimCoreSession::create(contract(true), options);
  assert(created);
  auto session = std::move(created.value);
  assert(session->submit_imu(imu(0, 1000000000LL)));
  assert(session->submit_imu(imu(1, 2000000000LL)));
  assert(session->submit_lidar(lidar(2, 2000000000LL, 0.0)));
  assert(session->submit_imu(imu(3, 3000000000LL)));
  assert(session->submit_lidar(lidar(4, 3000000000LL, 0.02)));

  ConsumerCounters counters;
  counters.map_outputs = std::numeric_limits<std::uint64_t>::max();
  session->set_test_counters(counters);
  const auto closed = session->close();
  assert(!closed);
  assert(closed.error.code == ErrorCode::kCounterOverflow);
  assert(session->boundary().state() == BoundaryState::kFailed);
  assert(session->boundary().counters().map_outputs ==
         std::numeric_limits<std::uint64_t>::max());
  assert(session->boundary().counters().trajectory_outputs == 0);
  assert(session->boundary().counters().map_processed == 0);
  assert(session->boundary().counters().trajectory_processed == 0);
  assert(session->boundary().counters().failures == 1);
  const auto maps = session->take_map_chunks();
  assert(!maps && maps.error.code == ErrorCode::kInvalidState);
}

}  // namespace

int main() {
  test_config_preflight_is_strict_and_core_free();
  test_exact_core_vertical_slice();
  test_complete_map_vertical_slice();
  test_map_point_bound_fails_before_publication();
  test_map_callback_reentry_fails_before_atomic_seal();
  test_map_callback_exception_fails_before_atomic_seal();
  test_core_callback_exception_faults_without_publication();
  test_reentrant_callback_return_is_not_accepted_as_success();
  test_global_config_lease_is_bounded_and_fail_closed();
  test_counter_overflow_and_map_contract_fail_closed();
  test_map_batch_counter_overflow_has_no_publication();
  std::cout << "glim_clean_room Phase 3b/3c runtime tests: 11 groups passed\n";
  return 0;
}
