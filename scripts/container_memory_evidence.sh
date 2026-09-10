#!/usr/bin/env bash
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# The process-tree sampler is the primary RSS measurement. Cgroup memory is
# retained as a diagnostic footprint and pressure/OOM record.

M6A7_SAMPLER_PID="${M6A7_SAMPLER_PID:-}"
M6A7_MEMORY_SCRIPT="${M6A7_MEMORY_SCRIPT:-/runner/scripts/container_memory_evidence.py}"
M6A7_EVIDENCE_FINALIZED="${M6A7_EVIDENCE_FINALIZED:-0}"
M6A7_EVIDENCE_STATUS="${M6A7_EVIDENCE_STATUS:-1}"
M6A7_EVIDENCE_FINALIZING="${M6A7_EVIDENCE_FINALIZING:-0}"
M6A7_SAMPLER_STOP_TIMEOUT_SECS="${M6A7_SAMPLER_STOP_TIMEOUT_SECS:-5}"

m6a7_capture_cgroup_baseline() {
  [[ -n "${OUT_DIR:-}" ]] || return 1
  python3 "${M6A7_MEMORY_SCRIPT}" baseline \
    --output "${OUT_DIR}/.m6a7_cgroup_baseline.json" \
    --cgroup-root /sys/fs/cgroup
}

m6a7_start_process_rss_sampler() {
  [[ -n "${OUT_DIR:-}" && -z "${M6A7_SAMPLER_PID}" ]] || return 1
  local sampler_script="${M6A7_SAMPLER_SCRIPT:-/runner/scripts/sample_container_process_rss.py}"
  local output="${OUT_DIR}/container_process_rss.json"
  [[ -r "${sampler_script}" && ! -e "${output}" && ! -e "${output}.part" ]] || return 1
  m6a7_capture_cgroup_baseline || return 1
  python3 "${sampler_script}" \
    --output "${output}" \
    --interval-ms "${M6A7_RSS_INTERVAL_MS:-250}" \
    --min-samples "${M6A7_RSS_MIN_SAMPLES:-2}" \
    --max-errors "${M6A7_RSS_MAX_ERRORS:-100}" \
    --max-race-skips "${M6A7_RSS_MAX_RACE_SKIPS:-100}" \
    --max-jitter-percent "${M6A7_RSS_MAX_JITTER_PERCENT:-100}" \
    --scheduler-nice "${M6A7_SAMPLER_NICE:-10}" \
    >"${OUT_DIR}/container_process_rss_sampler.log" 2>&1 &
  M6A7_SAMPLER_PID=$!
}

m6a7_stop_process_rss_sampler() {
  local sampler_pid="${M6A7_SAMPLER_PID:-}"
  local output="${OUT_DIR:-}/container_process_rss.json"
  if [[ -z "${sampler_pid}" ]]; then
    [[ -s "${output}" ]]
    return $?
  fi
  kill -TERM "${sampler_pid}" >/dev/null 2>&1 || true
  local started_at="${SECONDS}"
  while kill -0 "${sampler_pid}" >/dev/null 2>&1 &&
        [[ ! -s "${output}" ]]; do
    if (( SECONDS - started_at >= M6A7_SAMPLER_STOP_TIMEOUT_SECS )); then
      kill -KILL "${sampler_pid}" >/dev/null 2>&1 || true
      break
    fi
    sleep 0.05
  done
  wait "${sampler_pid}" >/dev/null 2>&1 || true
  M6A7_SAMPLER_PID=""
  [[ -s "${output}" ]]
}

m6a5_write_container_memory_evidence() {
  local process_exit_status="${1:-$?}"
  local out_dir="${OUT_DIR:-}"
  local output="${out_dir}/container_memory.json"
  local part="${output}.part"
  if [[ "${M6A7_EVIDENCE_FINALIZED}" == 1 ]]; then
    return "${M6A7_EVIDENCE_STATUS}"
  fi
  [[ "${M6A7_EVIDENCE_FINALIZING}" == 0 ]] || return 1
  [[ -n "${out_dir}" && ! -e "${output}" && ! -e "${part}" ]] || return 1
  M6A7_EVIDENCE_FINALIZING=1
  set +e
  m6a7_stop_process_rss_sampler >/dev/null 2>&1
  local sampler_stop_status=$?

  local readability_status=pass
  local readability_reason=output_tree_chmod_a+rX
  if [[ "${out_dir}" == / || "${out_dir}" == /runner ||
        "${out_dir}" == /runner/* || "${out_dir}" == /input ||
        "${out_dir}" == /input/* ]]; then
    M6A7_EVIDENCE_FINALIZING=0
    return 1
  elif ! chmod -R a+rX -- "${out_dir}" >/dev/null 2>&1; then
    readability_status=invalid
    readability_reason=chmod_output_tree_failed
  fi

  local cgroup_path=''
  local proc_self_cgroup
  proc_self_cgroup="$(cat /proc/self/cgroup 2>/dev/null || true)"
  while IFS= read -r line; do
    if [[ "${line}" == 0::* ]]; then
      cgroup_path="${line#0::}"
      break
    fi
  done <<<"${proc_self_cgroup}"

  python3 "${M6A7_MEMORY_SCRIPT}" final \
    --part "${part}" --output "${output}" \
    --baseline "${out_dir}/.m6a7_cgroup_baseline.json" \
    --sampler "${out_dir}/container_process_rss.json" \
    --cgroup-root /sys/fs/cgroup \
    --process-status "${process_exit_status}" \
    --sampler-stop-status "${sampler_stop_status}" \
    --cgroup-path "${cgroup_path}" \
    --proc-self-cgroup "${proc_self_cgroup}" \
    --readability-status "${readability_status}" \
    --readability-reason "${readability_reason}"
  local write_status=$?
  chmod a+rX -- "${output}" >/dev/null 2>&1 || true
  M6A7_EVIDENCE_STATUS="${write_status}"
  M6A7_EVIDENCE_FINALIZED=1
  M6A7_EVIDENCE_FINALIZING=0
  return "${write_status}"
}

m6a5_container_signal_trap() {
  local signal_status="${1:?signal status is required}"
  # Let EXIT perform exactly one finalization, then preserve shell signal
  # semantics (SIGINT=130, SIGTERM=143) for the caller/driver.
  trap - INT TERM
  exit "${signal_status}"
}

m6a5_install_container_signal_traps() {
  trap 'm6a5_container_signal_trap 130' INT
  trap 'm6a5_container_signal_trap 143' TERM
}

m6a5_container_exit_trap() {
  local exit_status="${1:-$?}"
  set +e
  local phase_status=0
  if declare -F m6a10_phase_finalize >/dev/null 2>&1; then
    m6a10_phase_finalize "${exit_status}" "${M6A10_RESOURCE_REPORT:-}" || \
      phase_status=$?
  fi
  m6a5_write_container_memory_evidence "${exit_status}"
  local evidence_status=$?
  if [[ "${exit_status}" -eq 0 && "${phase_status}" -ne 0 && \
        "${M6A3_SYNTHETIC_SMOKE:-0}" != 1 ]]; then
    return "${phase_status}"
  fi
  if [[ "${exit_status}" -eq 0 && "${evidence_status}" -ne 0 ]]; then
    return "${evidence_status}"
  fi
  return "${exit_status}"
}
