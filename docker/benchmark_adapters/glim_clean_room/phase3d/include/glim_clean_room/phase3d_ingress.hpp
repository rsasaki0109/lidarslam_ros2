#pragma once

#include "glim_clean_room/contract.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace glim_clean_room::phase3d {

enum class IngressKind : std::uint8_t {
  kLidar,
  kImu,
};

// This type contains no ROS or GLIM object.  ROS callbacks copy their owned
// message into the node's payload table and put only this deterministic key in
// the bounded ordering queue.
struct IngressItem {
  StampNanoseconds stamp{0};
  std::uint64_t arrival{0};
  IngressKind kind{IngressKind::kLidar};
};

struct IngressCounters {
  std::uint64_t enqueued{0};
  std::uint64_t flushed{0};
  std::uint64_t rejected{0};
};

// A bounded, deterministic timestamp reorder policy.  Items are ordered by
// (sensor stamp, host arrival ordinal).  The queue never silently drops an
// item: capacity exhaustion, late arrival, duplicate/non-monotonic arrival,
// counter exhaustion, and post-EOF input latch a terminal error.
class BoundedIngressQueue final {
 public:
  BoundedIngressQueue(std::size_t max_pending,
                      std::uint64_t reorder_window_ns);

  Status enqueue(IngressItem item);
  Result<std::vector<IngressItem>> flush_ready(bool force_at_eof);
  Status note_submitted(StampNanoseconds stamp);
  Status request_eof();

  bool eof_requested() const noexcept { return eof_requested_; }
  bool terminal() const noexcept { return terminal_error_.code != ErrorCode::kNone; }
  const Error& terminal_error() const noexcept { return terminal_error_; }
  std::size_t pending_size() const noexcept { return pending_.size(); }
  const IngressCounters& counters() const noexcept { return counters_; }

 private:
  Status latch(ErrorCode code, const char* detail);
  Status reject(ErrorCode code, const char* detail);
  Status increment(std::uint64_t& value, const char* name);

  std::size_t max_pending_{0};
  std::uint64_t reorder_window_ns_{0};
  std::vector<IngressItem> pending_;
  bool eof_requested_{false};
  bool have_arrival_{false};
  std::uint64_t last_arrival_{0};
  bool have_max_seen_{false};
  StampNanoseconds max_seen_stamp_{0};
  bool have_last_submitted_{false};
  StampNanoseconds last_submitted_stamp_{0};
  IngressCounters counters_{};
  Error terminal_error_{};
};

}  // namespace glim_clean_room::phase3d
