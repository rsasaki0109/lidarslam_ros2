# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[2]
WRAPPER = ROOT / 'scripts/fast_livo2_m6a10_v22_formal_container_run.sh'
V17 = ROOT / 'scripts/fast_livo2_m6a10_v17_formal_container_run.sh'


def test_v22_wrapper_is_additive_and_shell_valid():
    text = WRAPPER.read_text(encoding='utf-8')
    assert WRAPPER != V17
    assert subprocess.run(['bash', '-n', str(WRAPPER)],
                          check=False).returncode == 0
    assert 'v22' in text
    assert 'rosbag play' not in text
    assert 'docker run' not in text


def test_feeder_exit_is_persisted_before_branch_for_both_statuses():
    text = WRAPPER.read_text(encoding='utf-8')
    wait = text.index('wait "${FEEDER_PID}"')
    persist = text.index('atomic_text "${FEEDER_EXIT_STATUS}"', wait)
    branch = text.index('(( feeder_status == 0 ))', persist)
    assert wait < persist < branch
    assert 'M6A10_FEEDER_EXIT_STATUS_EVIDENCE' in text
    assert 'os.O_EXCL' in text and 'os.fsync' in text and 'os.link' in text
    assert 'FEEDER_STATUS' in text


def test_timing_follows_terminal_finalize_and_is_atomic():
    text = WRAPPER.read_text(encoding='utf-8')
    finalize = text.index('call_trigger /m6a10/terminal_finalize')
    timing_comment = text.index('Timing is generated only after')
    assert finalize < timing_comment
    assert 'DRAIN_END_MONOTONIC_NS' in text
    assert 'online_compute_rtf' in text
    assert '"boundary": "input_start_to_drain_end"' in text
    assert '"map_saved": False' in text


def test_trap_persists_failure_reason_and_manifest_on_early_exit():
    text = WRAPPER.read_text(encoding='utf-8')
    assert 'trap cleanup EXIT' in text
    assert 'write_failure_reason' in text
    assert 'write_artifact_manifest' in text
    assert 'artifact_manifest.json' in text
    assert 'failure_reason.json' in text
    assert 'MANIFEST_WRITTEN' in text
    assert re.search(r'if \(\( exit_status != 0 \)\); then', text)


def test_service_order_and_safety_contract_are_preserved():
    text = WRAPPER.read_text(encoding='utf-8')
    sequence = [
        '/m6a10/consumer_eof', '/m6a10/consumer_status',
        '/m6a10/consumer_finalize', '/m6a10/terminal_eof',
        '/m6a10/terminal_status', '/m6a10/terminal_status',
        '/m6a10/terminal_finalize',
    ]
    positions = []
    start = 0
    for item in sequence:
        pos = text.index(item, start)
        positions.append(pos)
        start = pos + 1
    assert positions == sorted(positions)
    assert 'ground_truth_content_opened' in text
    assert 'scorer_invoked' in text
    assert 'map_saved' in text
