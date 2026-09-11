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

set -euo pipefail

REPORT="${1:?report path is required}"
shift
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Keep the resource recorder outside the launch process group.  The benchmark
# shell intentionally terminates that group after map-save; if ``time`` were
# its leader, the recorder could be killed before writing its report.  A
# separate session lets this small supervisor stop the workload, wait for the
# recorder to flush, and preserve the caller's signal status.
# The Python implementation intentionally provides the same GNU-time field
# names on every image.  Keeping one implementation here also means signal
# shutdown can flush the report before the benchmark's process-group cleanup
# runs; GNU time itself may be killed before it writes its output.
COMMAND=(python3 "${SCRIPT_DIR}/portable_resource_time.py" --output "${REPORT}" -- "$@")

setsid "${COMMAND[@]}" &
CHILD_PID="$!"
CHILD_PGID=""
for _ in {1..20}; do
  CHILD_PGID="$(ps -o pgid= -p "${CHILD_PID}" 2>/dev/null | tr -d ' ' || true)"
  [[ -n "${CHILD_PGID}" ]] && break
  sleep 0.01
done

stop_child() {
  local signal="${1:-TERM}"
  if [[ -n "${CHILD_PGID}" && "${CHILD_PGID}" != "$$" ]]; then
    kill -"${signal}" -- "-${CHILD_PGID}" >/dev/null 2>&1 || true
  else
    kill -"${signal}" "${CHILD_PID}" >/dev/null 2>&1 || true
  fi
}

on_signal() {
  local status="${1:?signal status is required}"
  trap - INT TERM
  stop_child TERM
  # The launch has already reached its completion boundary when this helper
  # is interrupted.  Bound cleanup so a stuck child cannot prevent the
  # recorder from being finalized or change the semantic signal status.
  for _ in {1..20}; do
    if ! kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
      break
    fi
    sleep 0.01
  done
  if kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
    stop_child KILL
  fi
  wait "${CHILD_PID}" >/dev/null 2>&1 || true
  exit "${status}"
}

trap 'on_signal 130' INT
trap 'on_signal 143' TERM
set +e
wait "${CHILD_PID}"
STATUS="$?"
set -e
trap - INT TERM
exit "${STATUS}"
