#pragma once

// Phase 3b is an opt-in exact-core target.  The Phase 0 installed host ABI
// remains GLIM-free; this public session surface exposes only host-owned
// contract types and keeps exact GLIM objects behind Impl.
#include "glim_clean_room/contract.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace glim_clean_room::phase3b {

struct RuntimeOptions {
  // The directory must contain the exact-core config.json and the component
  // config files named by its global section.  It is an explicit runtime
  // input; no process working-directory or source-tree fallback is allowed.
  std::string config_directory;
  std::size_t ledger_capacity{4096};
  std::size_t max_pending_trajectory{4096};
  // Mapping is opt-in through SequenceContract::require_map_output.  These
  // explicit bounds are part of the host contract; GLIM's internal vectors
  // are not used as capacity policy.
  std::size_t max_map_frames{4096};
  std::size_t max_map_submaps{1024};
  std::size_t max_map_points{1U << 20};
  std::size_t max_map_chunks{1};
};

class GlimCoreSession final : public CoreSink {
 public:
  static Result<std::unique_ptr<GlimCoreSession>> create(
      SequenceContract contract, RuntimeOptions options);

  ~GlimCoreSession() noexcept override;

  GlimCoreSession(const GlimCoreSession&) = delete;
  GlimCoreSession& operator=(const GlimCoreSession&) = delete;
  GlimCoreSession(GlimCoreSession&&) = delete;
  GlimCoreSession& operator=(GlimCoreSession&&) = delete;

  Status submit_lidar(const LidarFrame& frame);
  Status submit_imu(const ImuSample& sample);
  Status request_eof();
  Status begin_drain();
  Status complete_drain();

  // Close is transactional at the host boundary and idempotent after a
  // successful drain.  It never claims a hard interrupt of a synchronous core
  // call; a core exception or late callback faults the session instead.
  Status close();

  Result<std::vector<TrajectorySample>> take_trajectory();
  Result<std::vector<MapChunk>> take_map_chunks();

  const AdapterBoundary& boundary() const noexcept { return *boundary_; }

#ifdef GLIM_CLEAN_ROOM_TESTING
  void set_test_counters(const ConsumerCounters& counters) noexcept;
#endif

  // CoreSink implementation.  These methods are invoked only by the
  // serialized AdapterBoundary above; callers must not invoke them directly.
  Status on_lidar(const LidarFrame& frame) override;
  Status on_imu(const ImuSample& sample) override;
  Status on_eof() override;
  Status on_begin_drain() override;
  Status on_drain_complete() override;

 private:
  struct Impl;

  GlimCoreSession(SequenceContract contract, RuntimeOptions options,
                  std::unique_ptr<Impl> impl);

  Status failure(ErrorCode code, const std::string& detail);
  Status reject_reentrant();

  SequenceContract contract_;
  RuntimeOptions options_;
  std::unique_ptr<Impl> impl_;
  std::unique_ptr<AdapterBoundary> boundary_;
  std::recursive_mutex mutex_;
  bool core_call_active_{false};
  bool trajectory_taken_{false};
  bool map_taken_{false};
};

}  // namespace glim_clean_room::phase3b
