#include "glim_clean_room/phase3b_session.hpp"

#include "glim_clean_room/phase3a_conversion.hpp"

#include <glim/odometry/callbacks.hpp>
#include <glim/odometry/odometry_estimation_cpu.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/sub_mapping_passthrough.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#include <glim/util/config.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace glim_clean_room::phase3b {
namespace {

std::mutex& global_config_mutex() {
  static std::mutex mutex;
  return mutex;
}

[[noreturn]] void config_preflight_failure(const std::string& detail) {
  throw std::invalid_argument("Phase 3b/3c config preflight: " + detail);
}

bool path_within(const std::filesystem::path& root,
                 const std::filesystem::path& candidate) {
  auto root_it = root.begin();
  auto candidate_it = candidate.begin();
  for (; root_it != root.end(); ++root_it, ++candidate_it) {
    if (candidate_it == candidate.end() || *root_it != *candidate_it) {
      return false;
    }
  }
  return true;
}

nlohmann::json parse_config_object(const std::filesystem::path& path,
                                   const char* role) {
  std::error_code error;
  const auto status = std::filesystem::symlink_status(path, error);
  if (error) {
    config_preflight_failure(std::string(role) +
                             " cannot be inspected: " + error.message());
  }
  if (status.type() == std::filesystem::file_type::symlink) {
    config_preflight_failure(std::string(role) + " is a symlink or cannot be inspected");
  }
  if (status.type() != std::filesystem::file_type::regular) {
    config_preflight_failure(std::string(role) + " is not a regular file: " +
                             path.string());
  }

  std::ifstream input(path);
  if (!input.good()) {
    config_preflight_failure(std::string(role) + " cannot be opened: " +
                             path.string());
  }
  try {
    const auto value = nlohmann::json::parse(input, nullptr, true, true);
    if (!value.is_object()) {
      config_preflight_failure(std::string(role) + " must contain a JSON object");
    }
    return value;
  } catch (const nlohmann::json::parse_error& exception) {
    config_preflight_failure(std::string(role) + " has malformed JSON: " +
                             exception.what());
  }
}

std::filesystem::path validate_config_directory(const std::string& value,
                                                bool map_enabled) {
  if (value.empty()) {
    config_preflight_failure("config_directory is empty");
  }
  std::error_code error;
  auto root = std::filesystem::absolute(value, error);
  if (error) {
    config_preflight_failure("config_directory cannot be made absolute: " +
                             error.message());
  }
  root = root.lexically_normal();
  auto root_component = root.root_path();
  for (const auto& component : root.relative_path()) {
    root_component /= component;
    const auto component_status =
        std::filesystem::symlink_status(root_component, error);
    if (error) {
      config_preflight_failure("config_directory cannot be inspected: " +
                               error.message());
    }
    if (component_status.type() == std::filesystem::file_type::symlink) {
      config_preflight_failure("config_directory contains a symlink: " +
                               root_component.string());
    }
    if (component_status.type() == std::filesystem::file_type::not_found) {
      break;
    }
  }
  const auto root_status = std::filesystem::symlink_status(root, error);
  if (error) {
    config_preflight_failure("config_directory cannot be inspected: " +
                             error.message());
  }
  if (root_status.type() == std::filesystem::file_type::symlink) {
    config_preflight_failure("config_directory is a symlink or cannot be inspected");
  }
  if (root_status.type() != std::filesystem::file_type::directory) {
    config_preflight_failure("config_directory is not a directory: " +
                             root.string());
  }

  const auto canonical_root = std::filesystem::canonical(root, error);
  if (error) {
    config_preflight_failure("config_directory cannot be canonicalized: " +
                             error.message());
  }
  const auto global = parse_config_object(root / "config.json", "config.json");
  const auto global_it = global.find("global");
  if (global_it == global.end() || !global_it->is_object()) {
    config_preflight_failure("config.json/global must be a JSON object");
  }

  constexpr std::array<const char*, 4> base_components = {
      "config_logging", "config_sensors", "config_preprocess", "config_odometry",
  };
  for (const char* key : base_components) {
    const auto component_it = global_it->find(key);
    if (component_it == global_it->end() || !component_it->is_string() ||
        component_it->get<std::string>().empty()) {
      config_preflight_failure(std::string("global/") + key +
                               " must be a nonempty relative filename");
    }
    const std::filesystem::path relative(component_it->get<std::string>());
    if (relative.is_absolute() || relative.has_root_name() ||
        relative.has_root_directory()) {
      config_preflight_failure(std::string("global/") + key +
                               " must not be absolute: " + relative.string());
    }
    for (const auto& component : relative) {
      if (component == "." || component == "..") {
        config_preflight_failure(std::string("global/") + key +
                                 " contains traversal: " + relative.string());
      }
    }
    const auto candidate = (root / relative).lexically_normal();
    if (!path_within(root, candidate) || candidate == root) {
      config_preflight_failure(std::string("global/") + key +
                               " escapes config_directory: " + relative.string());
    }
    auto current = root;
    for (const auto& component : relative) {
      current /= component;
      const auto current_status =
          std::filesystem::symlink_status(current, error);
      if (error) {
        config_preflight_failure(std::string("global/") + key +
                                 " cannot be inspected: " + error.message());
      }
      if (current_status.type() == std::filesystem::file_type::symlink) {
        config_preflight_failure(std::string("global/") + key +
                                 " contains a symlink: " + current.string());
      }
    }
    const auto canonical_candidate = std::filesystem::canonical(candidate, error);
    if (error || !path_within(canonical_root, canonical_candidate)) {
      config_preflight_failure(std::string("global/") + key +
                               " resolves outside config_directory: " +
                               candidate.string());
    }
    (void)parse_config_object(candidate, key);
  }
  if (map_enabled) {
    constexpr std::array<const char*, 2> mapping_components = {
        "config_sub_mapping", "config_global_mapping"};
    for (const char* key : mapping_components) {
      const auto component_it = global_it->find(key);
      if (component_it == global_it->end() || !component_it->is_string() ||
          component_it->get<std::string>().empty()) {
        config_preflight_failure(std::string("global/") + key +
                                 " must be a nonempty relative filename");
      }
      const std::filesystem::path relative(component_it->get<std::string>());
      if (relative.is_absolute() || relative.has_root_name() ||
          relative.has_root_directory()) {
        config_preflight_failure(std::string("global/") + key +
                                 " must not be absolute: " + relative.string());
      }
      for (const auto& component : relative) {
        if (component == "." || component == "..") {
          config_preflight_failure(std::string("global/") + key +
                                   " contains traversal: " + relative.string());
        }
      }
      const auto candidate = (root / relative).lexically_normal();
      if (!path_within(root, candidate) || candidate == root) {
        config_preflight_failure(std::string("global/") + key +
                                 " escapes config_directory: " + relative.string());
      }
      auto current = root;
      for (const auto& component : relative) {
        current /= component;
        const auto current_status =
            std::filesystem::symlink_status(current, error);
        if (error) {
          config_preflight_failure(std::string("global/") + key +
                                   " cannot be inspected: " + error.message());
        }
        if (current_status.type() == std::filesystem::file_type::symlink) {
          config_preflight_failure(std::string("global/") + key +
                                   " contains a symlink: " + current.string());
        }
      }
      const auto canonical_candidate = std::filesystem::canonical(candidate, error);
      if (error || !path_within(canonical_root, canonical_candidate)) {
        config_preflight_failure(std::string("global/") + key +
                                 " resolves outside config_directory: " +
                                 candidate.string());
      }
      (void)parse_config_object(candidate, key);
    }
  }
  return root;
}

struct CallbackFence final {
  // GLIM callback slots are process-global and have no clear operation.  The
  // weak state makes a retained/late callback harmless after teardown; the
  // exact core path is still serialized by GlimCoreSession.
  std::shared_ptr<std::atomic_bool> alive{
      std::make_shared<std::atomic_bool>(true)};
  int update_new_frame_id{-1};

  CallbackFence() {
    const std::weak_ptr<std::atomic_bool> weak_alive = alive;
    update_new_frame_id = glim::OdometryEstimationCallbacks::on_update_new_frame.add(
        [weak_alive](const glim::EstimationFrame::ConstPtr&) {
          const auto state = weak_alive.lock();
          if (!state || !state->load(std::memory_order_acquire)) return;
          // Output is intentionally collected from insert_frame and
          // get_remaining_frames, not from this process-global callback.  The
          // callback is only a lifetime fence and has no publication side
          // effect.
        });
  }

  ~CallbackFence() {
    alive->store(false, std::memory_order_release);
    if (update_new_frame_id >= 0) {
      glim::OdometryEstimationCallbacks::on_update_new_frame.remove(
          update_new_frame_id);
    }
  }
};

}  // namespace

struct GlimCoreSession::Impl final {
  struct MapFrameSignature {
    long id{0};
    double stamp{0.0};
    int frame_id{0};
  };

  Impl(const SequenceContract& contract, const RuntimeOptions& options)
      : config_lease(global_config_mutex(), std::defer_lock),
        ledger(options.ledger_capacity),
        trajectory_converter(contract),
        max_pending_trajectory(options.max_pending_trajectory),
        map_enabled(contract.require_map_output),
        map_frame(contract.map_frame),
        max_map_frames(options.max_map_frames),
        max_map_submaps(options.max_map_submaps),
        max_map_points(options.max_map_points),
        max_map_chunks(options.max_map_chunks) {
    if (options.config_directory.empty()) {
      throw std::invalid_argument("Phase 3b requires an explicit config directory");
    }
    const auto config_directory =
        validate_config_directory(options.config_directory, map_enabled);
    if (options.ledger_capacity == 0 || options.max_pending_trajectory == 0 ||
        (map_enabled && (options.max_map_frames == 0 ||
                         options.max_map_submaps == 0 ||
                         options.max_map_points == 0 ||
                         options.max_map_chunks == 0))) {
      throw std::invalid_argument(
          "Phase 3b capacities must be positive and bounded");
    }

    // GLIM GlobalConfig is a process-global mutable singleton.  Hold a
    // process lease for the complete core lifetime so two sessions cannot
    // race a configuration replacement or use one another's config paths.
    if (!config_lease.try_lock()) {
      throw std::runtime_error(
          "Phase 3b exact-core config lease is already held by another session; "
          "use an isolated process for concurrent GLIM configurations");
    }
    glim::GlobalConfig::instance(config_directory.string(), true);

    glim::CloudPreprocessorParams preprocess_params;
    preprocess_params.global_shutter = true;
    preprocess_params.use_random_grid_downsampling = false;
    preprocess_params.downsample_resolution = 0.01;
    preprocess_params.distance_near_thresh = 0.0;
    preprocess_params.distance_far_thresh = 1000.0;
    preprocess_params.enable_outlier_removal = false;
    preprocess_params.enable_cropbox_filter = false;
    preprocess_params.k_correspondences = 8;
    preprocess_params.num_threads = 1;
    preprocessor = std::make_unique<glim::CloudPreprocessor>(preprocess_params);

    glim::OdometryEstimationCPUParams odometry_params;
    odometry_params.registration_type = "GICP";
    odometry_params.max_iterations = 1;
    odometry_params.num_threads = 1;
    odometry_params.num_smoother_update_threads = 1;
    odometry_params.initialization_mode = "NAIVE";
    odometry_params.validate_imu = false;
    odometry_params.save_imu_rate_trajectory = false;
    odometry_params.smoother_lag = 5.0;
    odometry = std::make_unique<glim::OdometryEstimationCPU>(odometry_params);

    if (map_enabled) {
      // These are concrete public GLIM implementations, rather than the
      // abstract loaders whose save/export completion semantics are weaker.
      // Their constructors read the six-file, preflighted exact-core config.
      sub_mapping = std::make_unique<glim::SubMappingPassthrough>();
      glim::GlobalMappingParams mapping_params;
      mapping_params.enable_imu = false;
      mapping_params.enable_optimization = true;
      global_mapping = std::make_unique<glim::GlobalMapping>(mapping_params);
      pending_map_frames.reserve(options.max_map_frames);
      pending_map_chunks.reserve(options.max_map_chunks);
    }

    pending_trajectory.reserve(options.max_pending_trajectory);
  }

  Status latch(ErrorCode code, const std::string& detail) {
    if (terminal_error.code == ErrorCode::kNone) {
      terminal_error = Error{code, detail};
    }
    return Status::failure(terminal_error.code, terminal_error.detail);
  }

  Status append_frame(const glim::EstimationFrame::ConstPtr& frame) {
    if (terminal_error.code != ErrorCode::kNone) {
      return Status::failure(terminal_error.code, terminal_error.detail);
    }
    if (!frame) {
      return latch(ErrorCode::kUnsupportedCoreOutput,
                   "GLIM returned a null estimation frame");
    }
    if (pending_trajectory.size() >= max_pending_trajectory) {
      return latch(ErrorCode::kLedgerOverflow,
                   "Phase 3b pending trajectory capacity exhausted");
    }
    if (map_enabled) {
      if (pending_map_frames.size() >= max_map_frames) {
        return latch(ErrorCode::kLedgerOverflow,
                     "Phase 3c pending map-frame capacity exhausted");
      }
      if (map_frame_identity.find(frame.get()) != map_frame_identity.end()) {
        return latch(ErrorCode::kUnsupportedCoreOutput,
                     "GLIM emitted the same estimation frame more than once");
      }
      if (map_frame_ids.find(frame->id) != map_frame_ids.end()) {
        return latch(ErrorCode::kUnsupportedCoreOutput,
                     "GLIM emitted a duplicate estimation-frame ID");
      }
      const Eigen::Matrix4d transform = frame->T_world_lidar.matrix();
      const Eigen::Matrix3d rotation = transform.topLeftCorner<3, 3>();
      if (!std::isfinite(frame->stamp) || !frame->frame ||
          frame->frame->size() == 0 || frame->frame->points == nullptr) {
        return latch(ErrorCode::kUnsupportedCoreOutput,
                     "GLIM emitted a map frame without finite stamp/points");
      }
      if (!transform.allFinite() ||
          std::abs(transform(3, 0)) > 1e-9 ||
          std::abs(transform(3, 1)) > 1e-9 ||
          std::abs(transform(3, 2)) > 1e-9 ||
          std::abs(transform(3, 3) - 1.0) > 1e-9 ||
          !(rotation.transpose() * rotation).isApprox(
              Eigen::Matrix3d::Identity(), 1e-6) ||
          std::abs(rotation.determinant() - 1.0) > 1e-6) {
        return latch(ErrorCode::kUnsupportedCoreOutput,
                     "GLIM emitted a non-rigid map frame transform");
      }
      map_frame_identity.insert(frame.get());
      map_frame_ids.insert(frame->id);
      pending_map_frame_manifest.emplace(
          frame->id, MapFrameSignature{frame->id, frame->stamp,
                                       static_cast<int>(frame->frame_id)});
    }
    const auto converted = trajectory_converter.convert(frame, ledger);
    if (!converted) {
      return latch(converted.error.code, converted.error.detail);
    }
    pending_trajectory.push_back(std::move(converted.value));
    if (map_enabled) pending_map_frames.push_back(frame);
    return Status::success();
  }

  Status finalize_map() {
    if (!map_enabled) return Status::success();
    if (map_finalized) {
      return latch(ErrorCode::kInvalidState,
                   "Phase 3c map finalization was requested more than once");
    }
    if (pending_map_frames.empty()) {
      return latch(ErrorCode::kUnsupportedCoreOutput,
                   "GLIM drain produced no map frames");
    }
    if (pending_map_frames.size() != pending_trajectory.size()) {
      return latch(ErrorCode::kUnsupportedCoreOutput,
                   "map and trajectory frame counts diverged before mapping");
    }

    auto rigid_transform = [](const Eigen::Isometry3d& transform) {
      const Eigen::Matrix4d matrix = transform.matrix();
      if (!matrix.allFinite() || std::abs(matrix(3, 0)) > 1e-9 ||
          std::abs(matrix(3, 1)) > 1e-9 || std::abs(matrix(3, 2)) > 1e-9 ||
          std::abs(matrix(3, 3) - 1.0) > 1e-9) {
        return false;
      }
      const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
      return (rotation.transpose() * rotation).isApprox(
                 Eigen::Matrix3d::Identity(), 1e-6) &&
             std::abs(rotation.determinant() - 1.0) <= 1e-6;
    };

    std::size_t accepted_submap_points = 0;
    std::vector<glim::SubMap::Ptr> accepted_submaps;
    accepted_submaps.reserve(max_map_submaps);
    std::unordered_set<long> covered_map_frame_ids;
    auto accept_submaps = [&](const std::vector<glim::SubMap::Ptr>& submaps) {
      if (consumed_submap_ids.size() > max_map_submaps ||
          submaps.size() > max_map_submaps - consumed_submap_ids.size()) {
        return latch(ErrorCode::kLedgerOverflow,
                     "Phase 3c maximum submap count exceeded");
      }
      for (const auto& submap : submaps) {
        if (!submap || !submap->frame || submap->frame->size() == 0 ||
            submap->frame->points == nullptr || submap->id < 0 ||
            consumed_submap_identity.find(submap.get()) !=
                consumed_submap_identity.end() ||
            consumed_submap_ids.find(submap->id) != consumed_submap_ids.end() ||
            !rigid_transform(submap->T_world_origin) ||
            submap->frames.empty() || submap->odom_frames.empty() ||
            submap->frames.size() != submap->odom_frames.size() ||
            accepted_submap_points > max_map_points ||
            submap->frame->size() > max_map_points - accepted_submap_points) {
          return latch(ErrorCode::kUnsupportedCoreOutput,
                       "GLIM emitted a null, duplicate, empty, or non-rigid submap");
        }
        for (std::size_t index = 0; index < submap->odom_frames.size(); ++index) {
          const auto& odom_frame = submap->odom_frames[index];
          const auto& optimized_frame = submap->frames[index];
          if (!odom_frame || !optimized_frame ||
              odom_frame->id != optimized_frame->id ||
              odom_frame->stamp != optimized_frame->stamp ||
              odom_frame->frame_id != optimized_frame->frame_id) {
            return latch(ErrorCode::kUnsupportedCoreOutput,
                         "GLIM submap frame metadata is not paired exactly");
          }
          const auto expected = pending_map_frame_manifest.find(odom_frame->id);
          if (expected == pending_map_frame_manifest.end() ||
              expected->second.stamp != odom_frame->stamp ||
              expected->second.frame_id !=
                  static_cast<int>(odom_frame->frame_id) ||
              !covered_map_frame_ids.insert(odom_frame->id).second) {
            return latch(ErrorCode::kUnsupportedCoreOutput,
                         "GLIM submap frame coverage is missing, foreign, or duplicated");
          }
        }
        consumed_submap_identity.insert(submap.get());
        consumed_submap_ids.insert(submap->id);
        accepted_submap_points += submap->frame->size();
        accepted_submaps.push_back(submap);
      }
      return Status::success();
    };

    try {
      for (const auto& frame : pending_map_frames) {
        sub_mapping->insert_frame(frame);
        if (terminal_error.code != ErrorCode::kNone) {
          return Status::failure(terminal_error.code, terminal_error.detail);
        }
        // The queue read is destructive.  Read it after each finalized frame
        // so a long sequence cannot retain an unbounded submap queue; the
        // final read below still proves no residual queue remains at EOF.
        const auto periodic = sub_mapping->get_submaps();
        if (terminal_error.code != ErrorCode::kNone) {
          return Status::failure(terminal_error.code, terminal_error.detail);
        }
        const auto accepted_periodic = accept_submaps(periodic);
        if (!accepted_periodic) return accepted_periodic;
      }
      const auto completed = accept_submaps(sub_mapping->get_submaps());
      if (!completed) return completed;

      if (map_end_submitted) {
        return latch(ErrorCode::kInvalidState,
                     "GLIM submapping end-of-sequence was already submitted");
      }
      map_end_submitted = true;
      const auto tail = sub_mapping->submit_end_of_sequence();
      if (terminal_error.code != ErrorCode::kNone) {
        return Status::failure(terminal_error.code, terminal_error.detail);
      }
      const auto accepted_tail = accept_submaps(tail);
      if (!accepted_tail) return accepted_tail;
      // A concrete Passthrough implementation returns the forced tail from
      // submit_end_of_sequence and get_submaps is destructive.  Calling it a
      // second time proves that no queued submap was left unpublished.
      const auto residual = sub_mapping->get_submaps();
      if (terminal_error.code != ErrorCode::kNone) {
        return Status::failure(terminal_error.code, terminal_error.detail);
      }
      const auto accepted_residual = accept_submaps(residual);
      if (!accepted_residual) return accepted_residual;
      if (consumed_submap_ids.empty()) {
        return latch(ErrorCode::kUnsupportedCoreOutput,
                     "GLIM submapping produced no complete submap");
      }

      if (covered_map_frame_ids.size() != pending_map_frames.size()) {
        return latch(ErrorCode::kUnsupportedCoreOutput,
                     "GLIM submaps did not cover every finalized frame exactly once");
      }
      for (const auto& expected : pending_map_frame_manifest) {
        if (covered_map_frame_ids.find(expected.first) ==
            covered_map_frame_ids.end()) {
          return latch(ErrorCode::kUnsupportedCoreOutput,
                       "GLIM submap coverage omitted a finalized frame");
        }
      }
      for (std::size_t index = 0; index < accepted_submaps.size(); ++index) {
        if (accepted_submaps[index]->id != static_cast<int>(index)) {
          return latch(ErrorCode::kUnsupportedCoreOutput,
                       "GLIM submap IDs are not contiguous from zero");
        }
      }
      if (accepted_submap_points == 0 || accepted_submap_points > max_map_points) {
        return latch(ErrorCode::kUnsupportedCoreOutput,
                     "GLIM submap point count is outside the configured bound");
      }

      for (const auto& submap : accepted_submaps) {
        try {
          // GlobalMapping consumes the submap's frame metadata and drops only
          // the redundant per-frame point storage.  The merged submap frame
          // remains owned by the submap until export_points() completes.
          global_mapping->insert_submap(submap);
          if (terminal_error.code != ErrorCode::kNone) {
            return Status::failure(terminal_error.code, terminal_error.detail);
          }
        } catch (const std::exception& exception) {
          return latch(ErrorCode::kCoreFailure,
                       std::string("GLIM global mapping insertion threw: ") +
                           exception.what());
        } catch (...) {
          return latch(ErrorCode::kCoreFailure,
                       "GLIM global mapping insertion threw an unknown exception");
        }
      }

      global_mapping->optimize();
      if (terminal_error.code != ErrorCode::kNone) {
        return Status::failure(terminal_error.code, terminal_error.detail);
      }
      const auto exported = global_mapping->export_points();
      if (terminal_error.code != ErrorCode::kNone) {
        return Status::failure(terminal_error.code, terminal_error.detail);
      }
      if (!exported || exported->size() == 0 || exported->points == nullptr ||
          exported->size() != accepted_submap_points ||
          exported->size() > max_map_points ||
          exported->size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        return latch(ErrorCode::kUnsupportedCoreOutput,
                     "GLIM global mapping did not export a bounded point cloud");
      }
      if (max_map_chunks == 0) {
        return latch(ErrorCode::kLedgerOverflow,
                     "Phase 3c maximum map-chunk count is zero");
      }
      MapChunk chunk;
      chunk.order = 0;
      chunk.frame_id = map_frame;
      chunk.points.reserve(exported->size());
      const bool has_intensities = exported->has_intensities();
      if (has_intensities && exported->intensities == nullptr) {
        return latch(ErrorCode::kUnsupportedCoreOutput,
                     "GLIM exported an intensity flag without storage");
      }
      for (std::size_t index = 0; index < exported->size(); ++index) {
        const Eigen::Vector4d point = exported->points[index];
        if (!point.allFinite() || std::abs(point.w() - 1.0) > 1e-6) {
          return latch(ErrorCode::kUnsupportedCoreOutput,
                       "GLIM exported a non-finite or non-homogeneous point");
        }
        const double intensity =
            has_intensities ? exported->intensities[index] : 0.0;
        if (!std::isfinite(intensity)) {
          return latch(ErrorCode::kUnsupportedCoreOutput,
                       "GLIM exported a non-finite intensity");
        }
        chunk.points.push_back(PointRecord{point.x(), point.y(), point.z(),
                                           intensity, 0.0, 0U, 0.0});
      }
      const auto valid = validate_map_chunk(chunk);
      if (!valid) return latch(valid.error.code, valid.error.detail);
      pending_map_chunks.clear();
      pending_map_chunks.push_back(std::move(chunk));
      map_finalized = true;
      return Status::success();
    } catch (const std::exception& exception) {
      return latch(ErrorCode::kCoreFailure,
                   std::string("GLIM map finalization threw: ") +
                       exception.what());
    } catch (...) {
      return latch(ErrorCode::kCoreFailure,
                   "GLIM map finalization threw an unknown exception");
    }
  }

  std::unique_lock<std::mutex> config_lease;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;
  std::unique_ptr<glim::OdometryEstimationCPU> odometry;
  std::unique_ptr<glim::SubMappingPassthrough> sub_mapping;
  std::unique_ptr<glim::GlobalMapping> global_mapping;
  phase3a::StampLedger ledger;
  phase3a::TrajectoryConverter trajectory_converter;
  std::vector<TrajectorySample> pending_trajectory;
  const std::size_t max_pending_trajectory;
  const bool map_enabled;
  const std::string map_frame;
  const std::size_t max_map_frames;
  const std::size_t max_map_submaps;
  const std::size_t max_map_points;
  const std::size_t max_map_chunks;
  std::vector<glim::EstimationFrame::ConstPtr> pending_map_frames;
  std::unordered_set<const void*> map_frame_identity;
  std::unordered_set<long> map_frame_ids;
  std::unordered_map<long, MapFrameSignature> pending_map_frame_manifest;
  std::vector<MapChunk> pending_map_chunks;
  std::unordered_set<const void*> consumed_submap_identity;
  std::unordered_set<int> consumed_submap_ids;
  bool map_end_submitted{false};
  bool map_finalized{false};
  CallbackFence callback_fence;
  bool eof_seen{false};
  bool drain_seen{false};
  Error terminal_error{};
};

Result<std::unique_ptr<GlimCoreSession>> GlimCoreSession::create(
    SequenceContract contract, RuntimeOptions options) {
  const auto contract_status = validate_sequence_contract(contract);
  if (!contract_status) {
    return Result<std::unique_ptr<GlimCoreSession>>::failure(
        contract_status.error.code, contract_status.error.detail);
  }
  try {
    auto impl = std::make_unique<Impl>(contract, options);
    auto session = std::unique_ptr<GlimCoreSession>(
        new GlimCoreSession(std::move(contract), std::move(options),
                            std::move(impl)));
    return Result<std::unique_ptr<GlimCoreSession>>::success(std::move(session));
  } catch (const std::exception& exception) {
    return Result<std::unique_ptr<GlimCoreSession>>::failure(
        ErrorCode::kCoreFailure,
        std::string("Phase 3b core construction failed: ") + exception.what());
  } catch (...) {
    return Result<std::unique_ptr<GlimCoreSession>>::failure(
        ErrorCode::kCoreFailure,
        "Phase 3b core construction failed with an unknown exception");
  }
}

GlimCoreSession::GlimCoreSession(SequenceContract contract,
                                 RuntimeOptions options,
                                 std::unique_ptr<Impl> impl)
    : contract_(std::move(contract)),
      options_(std::move(options)),
      impl_(std::move(impl)),
      boundary_(std::make_unique<AdapterBoundary>(contract_, *this)) {}

GlimCoreSession::~GlimCoreSession() noexcept {
  try {
    (void)close();
  } catch (...) {
    // Destruction cannot report a second failure.  Impl's terminal latch and
    // the retained evidence remain authoritative for the failed close.
  }
}

Status GlimCoreSession::failure(ErrorCode code, const std::string& detail) {
  if (impl_->terminal_error.code == ErrorCode::kNone) {
    impl_->terminal_error = Error{code, detail};
  }
  return Status::failure(impl_->terminal_error.code,
                         impl_->terminal_error.detail);
}

Status GlimCoreSession::reject_reentrant() {
  return failure(ErrorCode::kInvalidState,
                 "Phase 3b rejected reentrant input during a core call");
}

Status GlimCoreSession::submit_lidar(const LidarFrame& frame) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (core_call_active_) return reject_reentrant();
  return boundary_->submit_lidar(frame);
}

Status GlimCoreSession::submit_imu(const ImuSample& sample) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (core_call_active_) return reject_reentrant();
  return boundary_->submit_imu(sample);
}

Status GlimCoreSession::request_eof() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (core_call_active_) return reject_reentrant();
  return boundary_->request_eof();
}

Status GlimCoreSession::begin_drain() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (core_call_active_) return reject_reentrant();
  return boundary_->begin_drain();
}

Status GlimCoreSession::complete_drain() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (core_call_active_) return reject_reentrant();
  return boundary_->complete_drain();
}

Status GlimCoreSession::close() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (boundary_->state() == BoundaryState::kDrained) {
    return Status::success();
  }
  if (boundary_->state() == BoundaryState::kFailed) {
    return boundary_->terminal_error().code == ErrorCode::kNone
               ? Status::failure(ErrorCode::kCoreFailure,
                                 "Phase 3b boundary failed without a reason")
               : Status::failure(boundary_->terminal_error().code,
                                 boundary_->terminal_error().detail);
  }
  if (boundary_->state() == BoundaryState::kOpen) {
    const auto eof = boundary_->request_eof();
    if (!eof) return eof;
  }
  if (boundary_->state() == BoundaryState::kEofRequested) {
    const auto drain = boundary_->begin_drain();
    if (!drain) return drain;
  }
  if (boundary_->state() == BoundaryState::kDraining) {
    return boundary_->complete_drain();
  }
  return failure(ErrorCode::kInvalidState,
                 "Phase 3b close reached an unknown lifecycle state");
}

Result<std::vector<TrajectorySample>> GlimCoreSession::take_trajectory() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (boundary_->state() != BoundaryState::kDrained) {
    return Result<std::vector<TrajectorySample>>::failure(
        ErrorCode::kInvalidState,
        "trajectory cannot be taken before successful drain completion");
  }
  if (trajectory_taken_) {
    return Result<std::vector<TrajectorySample>>::failure(
        ErrorCode::kInvalidState,
        "trajectory output has already been taken");
  }
  trajectory_taken_ = true;
  return Result<std::vector<TrajectorySample>>::success(
      std::move(impl_->pending_trajectory));
}

Result<std::vector<MapChunk>> GlimCoreSession::take_map_chunks() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (boundary_->state() != BoundaryState::kDrained) {
    return Result<std::vector<MapChunk>>::failure(
        ErrorCode::kInvalidState,
        "map output cannot be taken before successful drain completion");
  }
  if (map_taken_) {
    return Result<std::vector<MapChunk>>::failure(
        ErrorCode::kInvalidState, "map output has already been taken");
  }
  map_taken_ = true;
  if (!contract_.require_map_output) {
    return Result<std::vector<MapChunk>>::success({});
  }
  return Result<std::vector<MapChunk>>::success(
      std::move(impl_->pending_map_chunks));
}

#ifdef GLIM_CLEAN_ROOM_TESTING
void GlimCoreSession::set_test_counters(
    const ConsumerCounters& counters) noexcept {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  boundary_->set_test_counters(counters);
}
#endif

Status GlimCoreSession::on_lidar(const LidarFrame& frame) {
  if (impl_->terminal_error.code != ErrorCode::kNone) {
    return Status::failure(impl_->terminal_error.code,
                           impl_->terminal_error.detail);
  }
  core_call_active_ = true;
  struct ActiveReset final {
    bool& active;
    ~ActiveReset() { active = false; }
  } reset{core_call_active_};

  const auto raw = phase3a::to_core_raw_points(
      frame, contract_, impl_->ledger);
  if (!raw) return failure(raw.error.code, raw.error.detail);
  try {
    const auto preprocessed = impl_->preprocessor->preprocess(raw.value);
    if (!preprocessed) {
      return failure(ErrorCode::kUnsupportedCoreOutput,
                     "GLIM preprocessor returned a null frame");
    }
    std::vector<glim::EstimationFrame::ConstPtr> marginalized;
    // The current return value is intentionally not published: the same
    // frame may later be returned by get_remaining_frames().
    (void)impl_->odometry->insert_frame(preprocessed, marginalized);
    for (const auto& output : marginalized) {
      const auto status = impl_->append_frame(output);
      if (!status) return status;
    }
    if (impl_->terminal_error.code != ErrorCode::kNone) {
      return Status::failure(impl_->terminal_error.code,
                             impl_->terminal_error.detail);
    }
    return Status::success();
  } catch (const std::exception& exception) {
    return failure(ErrorCode::kCoreFailure,
                   std::string("GLIM LiDAR processing threw: ") +
                       exception.what());
  } catch (...) {
    return failure(ErrorCode::kCoreFailure,
                   "GLIM LiDAR processing threw an unknown exception");
  }
}

Status GlimCoreSession::on_imu(const ImuSample& sample) {
  if (impl_->terminal_error.code != ErrorCode::kNone) {
    return Status::failure(impl_->terminal_error.code,
                           impl_->terminal_error.detail);
  }
  core_call_active_ = true;
  struct ActiveReset final {
    bool& active;
    ~ActiveReset() { active = false; }
  } reset{core_call_active_};

  const auto imu = phase3a::to_core_imu(sample, contract_.calibration.imu_frame);
  if (!imu) return failure(imu.error.code, imu.error.detail);
  try {
    impl_->odometry->insert_imu(imu.value.stamp_seconds,
                                imu.value.linear_acceleration,
                                imu.value.angular_velocity);
    if (impl_->terminal_error.code != ErrorCode::kNone) {
      return Status::failure(impl_->terminal_error.code,
                             impl_->terminal_error.detail);
    }
    return Status::success();
  } catch (const std::exception& exception) {
    return failure(ErrorCode::kCoreFailure,
                   std::string("GLIM IMU processing threw: ") + exception.what());
  } catch (...) {
    return failure(ErrorCode::kCoreFailure,
                   "GLIM IMU processing threw an unknown exception");
  }
}

Status GlimCoreSession::on_eof() {
  if (impl_->terminal_error.code != ErrorCode::kNone) {
    return Status::failure(impl_->terminal_error.code,
                           impl_->terminal_error.detail);
  }
  if (impl_->eof_seen) {
    return failure(ErrorCode::kInvalidState, "duplicate Phase 3b EOF");
  }
  impl_->eof_seen = true;
  return Status::success();
}

Status GlimCoreSession::on_begin_drain() {
  if (impl_->terminal_error.code != ErrorCode::kNone) {
    return Status::failure(impl_->terminal_error.code,
                           impl_->terminal_error.detail);
  }
  if (!impl_->eof_seen || impl_->drain_seen) {
    return failure(ErrorCode::kInvalidState,
                   "Phase 3b drain requires exactly one EOF");
  }
  core_call_active_ = true;
  struct ActiveReset final {
    bool& active;
    ~ActiveReset() { active = false; }
  } reset{core_call_active_};
  try {
    const auto remaining = impl_->odometry->get_remaining_frames();
    for (const auto& output : remaining) {
      const auto status = impl_->append_frame(output);
      if (!status) return status;
    }
    impl_->drain_seen = true;
    if (!impl_->ledger.all_emitted()) {
      return failure(ErrorCode::kStampLedgerUnmatched,
                     "GLIM drain ended with an un-emitted LiDAR stamp (bound=" +
                         std::to_string(impl_->ledger.size()) + ", emitted=" +
                         std::to_string(impl_->ledger.emitted_count()) +
                         ", staged=" +
                         std::to_string(impl_->pending_trajectory.size()) +
                         ", remaining=" + std::to_string(remaining.size()) + ")");
    }
    if (impl_->map_enabled) {
      const auto map_status = impl_->finalize_map();
      if (!map_status) return map_status;
    }
    if (impl_->terminal_error.code != ErrorCode::kNone) {
      return Status::failure(impl_->terminal_error.code,
                             impl_->terminal_error.detail);
    }
    // The boundary is already Draining (see AdapterBoundary::begin_drain).
    // Validate the complete staged vector and reserve every output counter
    // before publishing any one sample.  This is the only publication point
    // for the vertical slice and prevents a late overflow from exposing a
    // partial trajectory.
    if (impl_->pending_trajectory.empty() && impl_->pending_map_chunks.empty()) {
      if (contract_.require_trajectory_output || contract_.require_map_output) {
        return failure(ErrorCode::kUnsupportedCoreOutput,
                       "GLIM drain produced no required output");
      }
      return Status::success();
    }
    return boundary_->record_output_batch(impl_->pending_trajectory,
                                           impl_->pending_map_chunks);
  } catch (const std::exception& exception) {
    return failure(ErrorCode::kCoreFailure,
                   std::string("GLIM drain threw: ") + exception.what());
  } catch (...) {
    return failure(ErrorCode::kCoreFailure,
                   "GLIM drain threw an unknown exception");
  }
}

Status GlimCoreSession::on_drain_complete() {
  if (impl_->terminal_error.code != ErrorCode::kNone) {
    return Status::failure(impl_->terminal_error.code,
                           impl_->terminal_error.detail);
  }
  if (!impl_->drain_seen) {
    return failure(ErrorCode::kInvalidState,
                   "Phase 3b finalize requires a completed core drain");
  }
  if (impl_->map_enabled &&
      (!impl_->map_finalized || impl_->pending_map_chunks.empty())) {
    return failure(ErrorCode::kUnsupportedCoreOutput,
                   "Phase 3c map batch was not finalized before completion");
  }
  return Status::success();
}

}  // namespace glim_clean_room::phase3b
