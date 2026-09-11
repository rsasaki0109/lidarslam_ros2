#pragma once

#include <memory>
#include <vector>

#include "glim_clean_room/contract.hpp"

namespace glim_clean_room {

// The production implementation is a host-owned CoreSink.  Its only allowed
// upstream surface is the public GLIM core API at the pinned commit.  Keeping
// this seam ROS-free also permits the same contract to serve replay and ROS 2.
class GlimCoreEngine final : public CoreSink {
 public:
  GlimCoreEngine();
  ~GlimCoreEngine() override;

  GlimCoreEngine(GlimCoreEngine&&) noexcept;
  GlimCoreEngine& operator=(GlimCoreEngine&&) noexcept;
  GlimCoreEngine(const GlimCoreEngine&) = delete;
  GlimCoreEngine& operator=(const GlimCoreEngine&) = delete;

  Status on_lidar(const LidarFrame& frame) override;
  Status on_imu(const ImuSample& sample) override;
  Status on_eof() override;
  Status on_begin_drain() override;
  Status on_drain_complete() override;

  // Phase 0 exposes typed output ownership.  The exact core-backed
  // serializer is enabled only by the opt-in pinned-core build and must be
  // supplied before a result can be considered complete.
  Result<std::vector<TrajectorySample>> take_trajectory();
  Result<std::vector<MapChunk>> take_map_chunks();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace glim_clean_room
