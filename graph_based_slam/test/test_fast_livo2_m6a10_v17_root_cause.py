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


"""
Read-only v16 diagnosis and additive v17 disposition candidate gates.

The test intentionally does not start a container or touch sensor input.  It
checks the sealed v16 observation when available, mechanically applies every
serialized v5 terminal invariant, and compiles the new production-facing
retry ledger with warnings as errors.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
V10_PATCH = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch'
V11_PATCH = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch'
V12_PATCH = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch'
CANDIDATE = ROOT / 'tools/m6a10_terminal_support_context_v17_candidate.hpp'
SELFTEST = ROOT / 'tools/m6a10_terminal_support_context_v17_selftest.cpp'
ARTIFACT_ROOT = Path(
    '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/'
    'fast_livo2_v2c_v16_formal_replay_20260823T213533Z_agentv16formal')
TERMINAL = ARTIFACT_ROOT / 'out/consumer_evidence.json'
CALLBACK = ARTIFACT_ROOT / 'out/callback_consumer_evidence.json'
TERMINAL_SHA = '56523b593676961b83ca9f9340706e964086b6aa3916e47902219b81dc41d21a'
CLOSURE = ARTIFACT_ROOT / 'closure_receipt.json'
CLOSURE_SHA = 'a7a5bfaaaf99a0851cb1b5b19759cdc568c3d4f681f8f2ac81c438944d1c1ccc'


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _serialized_v5_checks(document: dict) -> dict[str, bool]:
    required = 1623491515.148352
    maximum_gap = 0.25
    backend = document['backend']
    boundary = backend['completed_boundary']
    buffers = document['buffers']
    received = document['received_topic_counts']
    completed = document['completed_counts']
    trajectory = document['trajectory']
    records = [record for value in buffers.values()
               for record in value['records']]

    def support_record(record: dict) -> bool:
        if record['topic'] == 'lidar':
            return False
        if not record['support_context_proven']:
            return False
        if not record['sync_predicate_evaluated']:
            return False
        if record['can_form_synchronization_unit']:
            return False
        strict_after = (
            record['post_boundary']
            and not record['equal_to_completed_boundary']
            and record['timestamp_seconds'] > boundary['timestamp_seconds']
            and record['reason_code'] == 'strictly_after_completed_boundary'
        )
        equal_boundary = (
            record['equal_to_completed_boundary']
            and not record['post_boundary']
            and record['timestamp_seconds'] == boundary['timestamp_seconds']
            and record['reason_code']
            == 'equal_to_completed_boundary_nonlidar_support_context'
        )
        return strict_after or equal_boundary

    checks = {
        'backend_boundary': (
            backend['quiescent']
            and backend['quiescence_observed']
            and backend['in_flight']['active'] is False
            and boundary['observed']
            and boundary['source']
            and boundary['sequence'] >= 0
        ),
        'count_conservation': all(
            completed[topic] + buffers[topic]['count'] == received[topic]
            for topic in ('lidar', 'imu', 'image')
        ),
        'failure_counters_zero': (
            document['processing_failures'] == 0
            and all(value == 0 for value in document['dropped_counts'].values())
            and all(value == 0 for value in document['overflow_counts'].values())
        ),
        'residual_lidar_zero': buffers['lidar']['count'] == 0,
        'stable_observation': (
            document['terminal_observation']['eof_observed']
            and document['terminal_observation']['stable']
            and document['terminal_observation']['poll_count'] >= 2
            and document['terminal_observation']['stable_poll_count'] >= 2
            and document['terminal_observation']['identical_snapshot']
        ),
        'bounded_gap': (
            0.0 <= trajectory['end_gap_seconds'] <= maximum_gap
            and abs(
                required
                - trajectory['last_timestamp_seconds']
                - trajectory['end_gap_seconds']
            )
            <= 1e-9
        ),
        'support_records': all(support_record(record) for record in records),
        'support_counts': (
            document['terminal_support_context']['total_count'] == 81
            and document['terminal_support_context']['by_topic']
            == {'lidar': 0, 'imu': 78, 'image': 3}
        ),
    }
    return checks


def test_v16_sealed_terminal_is_invalid_but_all_serialized_v5_invariants_hold():
    if not TERMINAL.is_file():
        pytest.skip('sealed v16 artifact is not mounted')
    assert _sha(TERMINAL) == TERMINAL_SHA
    value = json.loads(TERMINAL.read_text(encoding='utf-8'))
    assert value['status'] == 'invalid'
    assert all(_serialized_v5_checks(value).values())
    assert value['backend']['completed_boundary']['timestamp_seconds'] == 1623491515.03615
    assert value['trajectory']['end_gap_seconds'] == 0.11220192909240723
    assert value['received_topic_counts'] == {
        'lidar': 5793, 'imu': 225102, 'image': 5792}
    assert value['completed_counts'] == {
        'lidar': 5793, 'imu': 225024, 'image': 5789}
    assert {
        topic: value['buffers'][topic]['count'] for topic in (
            'lidar',
            'imu',
            'image')} == {
        'lidar': 0,
        'imu': 78,
        'image': 3}


def test_callback_and_closure_are_the_separate_sealed_observations():
    if not CALLBACK.is_file() or not CLOSURE.is_file():
        pytest.skip('sealed v16 artifacts are not mounted')
    callback = json.loads(CALLBACK.read_text(encoding='utf-8'))
    closure = json.loads(CLOSURE.read_text(encoding='utf-8'))
    assert callback['status'] == 'pass'
    assert callback['consumer']['transport_outstanding_at_drain'] == 0
    assert callback['consumer']['maximum_transport_outstanding_messages'] == 1
    assert closure['status'] == 'FAIL_CLOSED'
    assert _sha(CLOSURE) == CLOSURE_SHA


def test_exact_producer_root_cause_and_not_timestamp_or_validator_relaxation():
    v10 = V10_PATCH.read_text(encoding='utf-8')
    v11 = V11_PATCH.read_text(encoding='utf-8')
    v12 = V12_PATCH.read_text(encoding='utf-8')
    assert 'completion_disposition_valid_ = false' in v10
    assert 'void abort_synchronization_unit()' in v10
    assert 'discard_unit_locked(\"synchronization_unit_aborted\")' in v10
    assert '!completion_disposition_valid_' in v10
    assert 'timestamp_seconds < required_end_timestamp_seconds_' in v10
    assert '-        boundary_.timestamp_seconds < required_end_timestamp_seconds_ ||' in v11
    assert 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary' in v12
    assert 'support_record' in v12
    # v10 integration calls the sticky abort helper on normal no-data guards;
    # the v17 candidate requires those call sites to classify empty retries.
    assert v10.count(
        'm6a10_terminal_support_context.abort_synchronization_unit()') >= 6


def test_v17_candidate_compiles_and_selftest_is_input_free(tmp_path):
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('g++ unavailable')
    binary = tmp_path / 'm6a10_v17_selftest'
    subprocess.run(
        [
            compiler,
            '-std=c++14',
            '-Wall',
            '-Wextra',
            '-Werror',
            '-O2',
            '-I',
            str(ROOT / 'tools'),
            str(SELFTEST),
            '-o',
            str(binary),
        ],
        cwd=ROOT,
        check=True,
    )
    result = subprocess.run([str(binary)], cwd=ROOT,
                            check=True, capture_output=True, text=True)
    assert 'M6A10_V17_CASE no_input_retryable_waits' in result.stdout
    assert 'M6A10_V17_CASE retry_then_completed_boundary' in result.stdout
    assert 'M6A10_V17_CASE partial_retry_fail_closed' in result.stdout
    assert 'M6A10_V17_SELFTEST PASS' in result.stdout


def test_v17_candidate_has_no_runtime_or_authority_surface():
    text = CANDIDATE.read_text(encoding='utf-8')
    assert 'rosbag' not in text.lower()
    assert 'docker' not in text.lower()
    assert 'profile_sha256' not in text
    assert 'ground_truth' not in text
    assert 'scorer' not in text
