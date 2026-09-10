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
Static gates for the v12 formal container wrapper candidate.

Only shell syntax and source contracts are inspected.  These tests never
start ROS, open an input, invoke Docker, or execute a feeder/replay.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
WRAPPER = ROOT / 'scripts/fast_livo2_m6a10_v12_formal_container_run.sh'
V11 = ROOT / 'scripts/fast_livo2_m6a10_v11_container_run.sh'
CONTRACT = 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary'
PROFILE_SHA = '675996a7a5fd81d59d752de4bbea1d4373487a17d03ee085aa08ae9493a0b28d'
FEEDER_SHA = '1ea1c9bb9c625795c51c4b8c4b6aa81364370da17f8cac562f46d00af9691831'


def _text() -> str:
    assert WRAPPER.is_file()
    return WRAPPER.read_text(encoding='utf-8')


def test_v12_wrapper_is_new_executable_and_bash_valid():
    assert WRAPPER.stat().st_mode & 0o111
    result = subprocess.run(
        ['bash', '-n', str(WRAPPER)],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_v11_wrapper_is_unchanged_and_v12_identity_is_pinned():
    assert V11.is_file()
    # This is the immutable v11 source pin from its own gate; v12 must not
    # silently modify the predecessor while adding its candidate wrapper.
    assert hashlib.sha256(V11.read_bytes()).hexdigest() == (
        '309ebb310e78f0d28dfa969e505bb0629df277947368433fe60fff009b6f9783'
    )
    text = _text()
    assert "EXPECTED_CONTRACT='" + CONTRACT + "'" in text
    assert f"EXPECTED_PROFILE_SHA256='{PROFILE_SHA}'" in text
    assert f"EXPECTED_FEEDER_SHA256='{FEEDER_SHA}'" in text
    assert (
        "EXPECTED_PROFILE_PATH='configs/slam_benchmark_profiles/"
        "fast_livo2_m6a10_v12_formal_ready.yaml'"
        in text
    )


def test_v12_requires_all_seven_services_and_separate_outputs():
    text = _text()
    for service in (
        '/m6a10/consumer_status',
        '/m6a10/consumer_ack',
        '/m6a10/consumer_eof',
        '/m6a10/consumer_finalize',
        '/m6a10/terminal_status',
        '/m6a10/terminal_eof',
        '/m6a10/terminal_finalize',
    ):
        assert service in text
    assert 'callback_consumer_evidence.json' in text
    assert 'consumer_evidence.json' in text
    assert 'CALLBACK_EVIDENCE}' != 'TERMINAL_EVIDENCE}'  # distinct variables
    assert 'CALLBACK_EVIDENCE}' in text and 'TERMINAL_EVIDENCE}' in text
    assert 'callback/terminal paths alias' in text


def test_callback_then_terminal_then_timing_order_is_fail_closed():
    text = _text()
    ordered = (
        'call_trigger /m6a10/consumer_eof',
        'call_trigger /m6a10/consumer_status',
        'call_trigger /m6a10/consumer_finalize',
        'CALLBACK_EVIDENCE_SHA256=',
        'call_trigger /m6a10/terminal_eof',
        'if call_trigger /m6a10/terminal_status',
        'call_trigger /m6a10/terminal_finalize',
        'RAW_EVIDENCE_SHA256=',
        'DRAIN_END_MONOTONIC_NS=',
        'M6A10_TIMING_CONTRACT_VERSION',
    )
    positions = [
        text.rindex(item) if item == 'M6A10_TIMING_CONTRACT_VERSION' else text.index(item)
        for item in ordered
    ]
    assert positions == sorted(positions)
    assert '(( terminal_status_successes >= 2 ))' in text
    assert 'sleep "${M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS}"' in text


def test_callback_schema3_v5_transport_and_authority_safety_are_checked():
    text = _text()
    assert "value.get('schema_version') != 3" in text
    assert "value.get('contract_version') != '" + CONTRACT + "'" in text
    assert (
        (
            "value.get('transport_contract_version') != "
            "'m6a10-v12-callback-ack-transport-outstanding-v1'"
        )
        in text
    )
    for field in (
        'received_topic_counts',
        'acked_messages',
        'ack_exact',
        'transport_outstanding_at_drain',
        'maximum_transport_outstanding_messages',
        'maximum_allowed_transport_outstanding_messages',
        'mapper_internal_deque_current_messages',
        'mapper_internal_deque_peak_messages',
        'ground_truth_content_opened',
        'scorer_invoked',
    ):
        assert field in text
    for field in ('input', 'profile_sha256', 'published', 'ack', 'binding', 'validation'):
        assert field in text


def test_terminal_schema1_raw_v5_fields_and_timing_order_are_checked():
    text = _text()
    assert "value.get('schema_version') != 1" in text
    assert "value.get('contract_id') != 'm6a10-fast-livo2-consumer-terminal-v1'" in text
    assert 'nonlidar_at_or_after_boundary_support_context' in text
    assert 'equal_to_completed_boundary_nonlidar_support_context' in text
    assert 'strictly_after_completed_boundary' in text
    for field in (
        'completed_boundary',
        'completed_counts',
        'dropped_counts',
        'overflow_counts',
        'processing_failures',
        'terminal_observation',
        'trajectory',
        'end_gap_seconds',
        'ground_truth_content_opened',
        'scorer_invoked',
    ):
        assert field in text
    assert 'timing is written only after callback and terminal raw validation/hash' in text


def test_wrapper_does_not_use_csv_or_forbidden_evaluation_surfaces():
    text = _text().lower()
    assert 'rosbag play' not in text
    assert 'wait_for_trajectory' not in text
    assert 'trajectory_required_end' not in text
    assert 'ground_truth' in text and 'scorer_invoked' in text
    assert 'map_save' not in text
    assert 'rviz:=false' in text
