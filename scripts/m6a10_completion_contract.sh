#!/usr/bin/env bash
# Copyright 2026 Sasaki
# All rights reserved.
#
# BSD 2-Clause Simplified License. See the repository license headers.

# Return success only when the historical trajectory completion shortcut is
# allowed.  Keep the function call outside [[ ... ]]: Bash treats a bare
# function name inside [[ ]] as a non-empty string rather than invoking it.
# The caller supplies offline_completion_recorded from the benchmark runner.
m6a10_completion_barrier_allows() {
  if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" == "m6a10-online-compute-v2" ]] &&
     ! offline_completion_recorded; then
    return 1
  fi
  return 0
}
