/*
 * Host-only selftest for the v17 synchronization-disposition candidate.
 *
 * It exercises the production-facing ledger with synthetic event sequences.
 * There is no ROS master, sensor input, image/bag access, container, map, or
 * scoring path.  In particular, the no-input case must remain invalid until
 * a real completed boundary exists.
 */
#include "m6a10_terminal_support_context_v17_candidate.hpp"

#include <iostream>
#include <string>

namespace {

using Ledger = M6A10V17SynchronizationLedger;

bool expect(const std::string &name, bool actual, bool expected,
            bool terminal_valid, const Ledger::Snapshot &snapshot)
{
  const bool pass = actual == expected && terminal_valid == expected;
  std::cout << "M6A10_V17_CASE " << name
            << " actual=" << (actual ? "true" : "false")
            << " expected=" << (expected ? "true" : "false")
            << " terminal_valid=" << (terminal_valid ? "true" : "false")
            << " retries=" << snapshot.retryable_aborts
            << " fatal=" << snapshot.fatal_aborts
            << " failures=" << snapshot.processing_failures
            << " gate=" << (pass ? "PASS" : "FAIL") << '\n';
  return pass;
}

bool no_input_waits_are_not_a_completion()
{
  Ledger ledger;
  bool actual = true;
  for (int index = 0; index < 3; ++index)
  {
    actual = ledger.begin_unit() && actual;
    actual = ledger.abort(Ledger::AbortKind::RetryableNotReady) && actual;
  }
  const Ledger::Snapshot snapshot = ledger.snapshot();
  // Retryable waits are harmless, but no-input cannot claim a terminal pass.
  const bool pass = actual && !ledger.terminal_valid();
  std::cout << "M6A10_V17_CASE no_input_retryable_waits"
            << " actual=" << (actual ? "true" : "false")
            << " expected=true terminal_valid="
            << (ledger.terminal_valid() ? "true" : "false")
            << " retries=" << snapshot.retryable_aborts
            << " fatal=" << snapshot.fatal_aborts
            << " failures=" << snapshot.processing_failures
            << " gate=" << (pass ? "PASS" : "FAIL") << '\n';
  return pass;
}

bool retry_then_boundary_passes()
{
  Ledger ledger;
  bool actual = ledger.begin_unit();
  actual = ledger.abort_retryable() && actual;
  actual = ledger.begin_unit() && actual;
  actual = ledger.observe_candidate() && actual;
  actual = ledger.complete_boundary() && actual;
  return expect("retry_then_completed_boundary", actual, true,
                ledger.terminal_valid(), ledger.snapshot());
}

bool repeated_retry_after_boundary_passes()
{
  Ledger ledger;
  ledger.begin_unit();
  ledger.observe_candidate();
  ledger.complete_boundary();
  ledger.begin_unit();
  const bool actual = ledger.abort_retryable();
  return expect("later_empty_retry_preserves_boundary", actual, true,
                ledger.terminal_valid(), ledger.snapshot());
}

bool partial_retry_fails()
{
  Ledger ledger;
  ledger.begin_unit();
  ledger.observe_candidate();
  const bool actual = ledger.abort_retryable();
  return expect("partial_retry_fail_closed", actual, false,
                ledger.terminal_valid(), ledger.snapshot());
}

bool explicit_discard_fails()
{
  Ledger ledger;
  ledger.begin_unit();
  const bool actual = ledger.observe_discard();
  return expect("explicit_discard_fail_closed", actual, false,
                ledger.terminal_valid(), ledger.snapshot());
}

bool missing_candidate_fails()
{
  Ledger ledger;
  ledger.begin_unit();
  const bool actual = ledger.complete_boundary();
  return expect("missing_candidate_fail_closed", actual, false,
                ledger.terminal_valid(), ledger.snapshot());
}

bool processing_failure_fails()
{
  Ledger ledger;
  ledger.begin_unit();
  ledger.observe_candidate();
  const bool actual = ledger.record_processing_failure();
  return expect("processing_failure_fail_closed", actual, false,
                ledger.terminal_valid(), ledger.snapshot());
}

}  // namespace

int main()
{
  bool all_pass = true;
  all_pass = no_input_waits_are_not_a_completion() && all_pass;
  all_pass = retry_then_boundary_passes() && all_pass;
  all_pass = repeated_retry_after_boundary_passes() && all_pass;
  all_pass = partial_retry_fails() && all_pass;
  all_pass = explicit_discard_fails() && all_pass;
  all_pass = missing_candidate_fails() && all_pass;
  all_pass = processing_failure_fails() && all_pass;
  std::cout << "M6A10_V17_SELFTEST " << (all_pass ? "PASS" : "FAIL") << '\n';
  return all_pass ? 0 : 1;
}
