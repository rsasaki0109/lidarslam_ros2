#include "glim_clean_room/phase3d_ingress.hpp"

#include <algorithm>
#include <exception>
#include <limits>
#include <string>
#include <utility>

namespace glim_clean_room::phase3d {
namespace {

bool less_item(const IngressItem& lhs, const IngressItem& rhs) {
  if (lhs.stamp != rhs.stamp) return lhs.stamp < rhs.stamp;
  return lhs.arrival < rhs.arrival;
}

}  // namespace

BoundedIngressQueue::BoundedIngressQueue(
    std::size_t max_pending, std::uint64_t reorder_window_ns)
    : max_pending_(max_pending), reorder_window_ns_(reorder_window_ns) {
  try {
    pending_.reserve(max_pending_);
  } catch (const std::exception& exception) {
    terminal_error_ = Error{ErrorCode::kLedgerOverflow,
                            std::string("ingress queue allocation failed: ") +
                                exception.what()};
  } catch (...) {
    terminal_error_ = Error{ErrorCode::kLedgerOverflow,
                            "ingress queue allocation failed"};
  }
}

Status BoundedIngressQueue::latch(ErrorCode code, const char* detail) {
  if (terminal_error_.code == ErrorCode::kNone) {
    terminal_error_ = Error{code, detail == nullptr ? "" : detail};
  }
  return Status::failure(terminal_error_.code, terminal_error_.detail);
}

Status BoundedIngressQueue::reject(ErrorCode code, const char* detail) {
  if (terminal()) {
    return Status::failure(terminal_error_.code, terminal_error_.detail);
  }
  if (counters_.rejected == std::numeric_limits<std::uint64_t>::max()) {
    return latch(ErrorCode::kCounterOverflow,
                 "ingress rejected counter overflow");
  }
  ++counters_.rejected;
  return latch(code, detail);
}

Status BoundedIngressQueue::increment(std::uint64_t& value, const char* name) {
  if (value == std::numeric_limits<std::uint64_t>::max()) {
    return latch(ErrorCode::kCounterOverflow,
                 name == nullptr ? "ingress counter overflow"
                                  : "ingress counter overflow");
  }
  ++value;
  return Status::success();
}

Status BoundedIngressQueue::enqueue(IngressItem item) {
  if (terminal()) {
    return Status::failure(terminal_error_.code, terminal_error_.detail);
  }
  if (eof_requested_) {
    return reject(ErrorCode::kPostEof,
                  "ROS input arrived after the explicit finalize request");
  }
  if (max_pending_ == 0 || pending_.size() >= max_pending_) {
    return reject(ErrorCode::kLedgerOverflow,
                  "ROS ingress queue exhausted; no message was dropped silently");
  }
  if (have_arrival_ && item.arrival <= last_arrival_) {
    return reject(ErrorCode::kOrderViolation,
                  "ROS host arrival ordinal is not strictly increasing");
  }
  if (have_last_submitted_ && item.stamp < last_submitted_stamp_) {
    return reject(ErrorCode::kNonMonotonicStamp,
                  "late ROS message arrived behind an already submitted stamp");
  }
  const auto accepted = increment(counters_.enqueued, "enqueued");
  if (!accepted) return accepted;
  pending_.push_back(item);
  std::sort(pending_.begin(), pending_.end(), less_item);
  have_arrival_ = true;
  last_arrival_ = item.arrival;
  if (!have_max_seen_ || item.stamp > max_seen_stamp_) {
    max_seen_stamp_ = item.stamp;
    have_max_seen_ = true;
  }
  return Status::success();
}

Result<std::vector<IngressItem>> BoundedIngressQueue::flush_ready(
    bool force_at_eof) {
  if (terminal()) {
    return Result<std::vector<IngressItem>>::failure(
        terminal_error_.code, terminal_error_.detail);
  }
  if (force_at_eof && !eof_requested_) {
    const auto status = reject(ErrorCode::kInvalidState,
                               "forced ingress flush requires EOF");
    return Result<std::vector<IngressItem>>::failure(status.error.code,
                                                     status.error.detail);
  }
  std::vector<IngressItem> output;
  if (pending_.empty()) return Result<std::vector<IngressItem>>::success(output);

  StampNanoseconds cutoff = max_seen_stamp_;
  if (!force_at_eof && reorder_window_ns_ != 0) {
    const auto min_value = std::numeric_limits<StampNanoseconds>::min();
    const auto distance = static_cast<std::uint64_t>(max_seen_stamp_) -
                          static_cast<std::uint64_t>(min_value);
    if (reorder_window_ns_ > distance) {
      cutoff = min_value;
    } else {
      cutoff = max_seen_stamp_ -
               static_cast<StampNanoseconds>(reorder_window_ns_);
    }
  }

  std::size_t count = 0;
  while (count < pending_.size() &&
         (force_at_eof || pending_[count].stamp <= cutoff)) {
    ++count;
  }
  if (static_cast<std::uint64_t>(count) >
      std::numeric_limits<std::uint64_t>::max() - counters_.flushed) {
    const auto status = latch(ErrorCode::kCounterOverflow,
                              "ingress flushed counter overflow");
    return Result<std::vector<IngressItem>>::failure(status.error.code,
                                                     status.error.detail);
  }
  try {
    output.reserve(count);
    output.insert(output.end(), pending_.begin(), pending_.begin() + count);
  } catch (const std::exception& exception) {
    const auto status = latch(
        ErrorCode::kCoreFailure,
        (std::string("ingress flush allocation failed: ") + exception.what()).c_str());
    return Result<std::vector<IngressItem>>::failure(status.error.code,
                                                     status.error.detail);
  } catch (...) {
    const auto status = latch(ErrorCode::kCoreFailure,
                              "ingress flush allocation failed");
    return Result<std::vector<IngressItem>>::failure(status.error.code,
                                                     status.error.detail);
  }
  pending_.erase(pending_.begin(), pending_.begin() + count);
  counters_.flushed += static_cast<std::uint64_t>(count);
  return Result<std::vector<IngressItem>>::success(std::move(output));
}

Status BoundedIngressQueue::note_submitted(StampNanoseconds stamp) {
  if (terminal()) {
    return Status::failure(terminal_error_.code, terminal_error_.detail);
  }
  if (have_last_submitted_ && stamp < last_submitted_stamp_) {
    return reject(ErrorCode::kNonMonotonicStamp,
                  "serialized ingress submitted a regressing sensor stamp");
  }
  have_last_submitted_ = true;
  last_submitted_stamp_ = stamp;
  return Status::success();
}

Status BoundedIngressQueue::request_eof() {
  if (terminal()) {
    return Status::failure(terminal_error_.code, terminal_error_.detail);
  }
  if (eof_requested_) {
    return reject(ErrorCode::kInvalidState,
                  "ROS ingress EOF/finalize was requested more than once");
  }
  eof_requested_ = true;
  return Status::success();
}

}  // namespace glim_clean_room::phase3d
