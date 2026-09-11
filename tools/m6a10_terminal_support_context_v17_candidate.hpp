/*
 * Candidate production ledger for the M6A10 v5 terminal contract.
 *
 * The v10--v16 helper made completion_disposition_valid_ sticky when a
 * synchronization attempt was abandoned.  FAST-LIVO2 uses that path for a
 * normal "not enough sensor data yet" retry, before any record has been
 * incorporated.  This small production-facing ledger makes that distinction
 * explicit: an empty retry is not a completion failure, while a partial
 * retry, discard, queue error, or estimator failure remains fail-closed.
 *
 * This is an additive source candidate only.  It is not wired into the v12--
 * v16 image or launchers.  A future source integration must call
 * abort(AbortKind::RetryableNotReady) only on a no-record retry branch and
 * abort(AbortKind::Fatal) for all discard/error/exception branches.
 * No input, transport, terminal JSON, or authority fields are handled here.
 */
#ifndef M6A10_TERMINAL_SUPPORT_CONTEXT_V17_CANDIDATE_HPP
#define M6A10_TERMINAL_SUPPORT_CONTEXT_V17_CANDIDATE_HPP

#include <cstddef>

class M6A10V17SynchronizationLedger
{
public:
  enum class AbortKind
  {
    RetryableNotReady,
    Fatal
  };

  struct Snapshot
  {
    bool unit_open = false;
    bool terminal_valid = false;
    bool fatal = false;
    std::size_t candidate_count = 0;
    std::size_t completed_boundaries = 0;
    std::size_t retryable_aborts = 0;
    std::size_t fatal_aborts = 0;
    std::size_t processing_failures = 0;
    std::size_t discarded_records = 0;
  };

  bool begin_unit() noexcept
  {
    if (unit_open_)
      return fail_closed();
    unit_open_ = true;
    candidate_count_ = 0;
    return true;
  }

  bool observe_candidate() noexcept
  {
    if (!unit_open_ || fatal_)
      return fail_closed();
    ++candidate_count_;
    return true;
  }

  // A queue pop that was not incorporated is an observed discard, never a
  // retry.  This keeps data-cut and drop paths fail-closed.
  bool observe_discard() noexcept
  {
    if (!unit_open_ || fatal_)
      return fail_closed();
    ++discarded_records_;
    ++processing_failures_;
    fatal_ = true;
    unit_open_ = false;
    candidate_count_ = 0;
    ++fatal_aborts_;
    return false;
  }

  bool complete_boundary() noexcept
  {
    if (!unit_open_ || fatal_ || candidate_count_ == 0)
      return fail_closed();
    unit_open_ = false;
    candidate_count_ = 0;
    ++completed_boundaries_;
    return true;
  }

  bool abort(AbortKind kind) noexcept
  {
    return kind == AbortKind::RetryableNotReady ?
      abort_retryable() : abort_fatal();
  }

  bool abort_retryable() noexcept
  {
    // A partial unit cannot be silently retried: it has already changed the
    // disposition ledger and must remain a failure.
    if (!unit_open_ || fatal_ || candidate_count_ != 0)
      return fail_closed();
    unit_open_ = false;
    ++retryable_aborts_;
    return true;
  }

  bool abort_fatal() noexcept
  {
    if (!unit_open_ || fatal_)
      return fail_closed();
    fatal_ = true;
    unit_open_ = false;
    candidate_count_ = 0;
    ++processing_failures_;
    ++fatal_aborts_;
    return false;
  }

  bool record_processing_failure() noexcept
  {
    ++processing_failures_;
    fatal_ = true;
    unit_open_ = false;
    candidate_count_ = 0;
    return false;
  }

  bool terminal_valid() const noexcept
  {
    return completed_boundaries_ > 0 && !unit_open_ && !fatal_ &&
      processing_failures_ == 0 && discarded_records_ == 0;
  }

  Snapshot snapshot() const noexcept
  {
    Snapshot result;
    result.unit_open = unit_open_;
    result.terminal_valid = terminal_valid();
    result.fatal = fatal_;
    result.candidate_count = candidate_count_;
    result.completed_boundaries = completed_boundaries_;
    result.retryable_aborts = retryable_aborts_;
    result.fatal_aborts = fatal_aborts_;
    result.processing_failures = processing_failures_;
    result.discarded_records = discarded_records_;
    return result;
  }

private:
  bool fail_closed() noexcept
  {
    fatal_ = true;
    unit_open_ = false;
    candidate_count_ = 0;
    ++processing_failures_;
    return false;
  }

  bool unit_open_ = false;
  bool fatal_ = false;
  std::size_t candidate_count_ = 0;
  std::size_t completed_boundaries_ = 0;
  std::size_t retryable_aborts_ = 0;
  std::size_t fatal_aborts_ = 0;
  std::size_t processing_failures_ = 0;
  std::size_t discarded_records_ = 0;
};

#endif  // M6A10_TERMINAL_SUPPORT_CONTEXT_V17_CANDIDATE_HPP
