#!/usr/bin/env python3

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
Static YAML gates for the v11 bounded-end-gap profile candidate.

These tests parse only repository/profile/receipt metadata.  They do not open
the bag, invoke Docker, start ROS, run formal replay, or evaluate GT/scorer
outputs.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
PROFILE_PATH = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v11_formal.yaml'
V10_PROFILE_PATH = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v10_formal.yaml'


def _profile() -> dict:
    return yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))


def _candidate() -> dict:
    return _profile()['competitive_slam_profile']['m6a10_fast_livo2_v2c_v11']


def test_v11_profile_is_versioned_from_immutable_v10_and_contract_is_v4():
    profile = _profile()
    candidate = _candidate()
    assert profile['name'] == 'fast_livo2_m6a10_v2c_v11_terminal_support_context'
    preregistration = profile['preregistration']
    assert preregistration['predecessor_profile'] == (
        'configs/slam_benchmark_profiles/fast_livo2_m6a10_v10_formal.yaml')
    assert preregistration['predecessor_profile_sha256'] == (
        'f706f41b0a985347ff98e4953532590c7a9c8c2d75db355e53cfb558f95bc459')
    assert preregistration['predecessor_closure_path'].endswith(
        'fast_livo2_v2c_v10_formal_replay3_20260823T112240Z_agentv10formal/'
        'closure_receipt.json')
    assert preregistration['predecessor_closure_sha256'] == (
        '16d1ee5bd55588391e3019bcdd7828c85f74c484f314f64a4dcc8a95a300fb05')
    assert hashlib.sha256(V10_PROFILE_PATH.read_bytes()).hexdigest() == (
        'f706f41b0a985347ff98e4953532590c7a9c8c2d75db355e53cfb558f95bc459')
    assert candidate['profile_key'] == 'm6a10_fast_livo2_v2c_v11'
    assert candidate['phase']['contract_version'] == (
        'm6a10-online-compute-v4-terminal-bounded-end-gap')
    assert candidate['contract_tests']['path'] == (
        'graph_based_slam/test/test_fast_livo2_m6a10_v11_profile.py')
    assert candidate['contract_tests']['sha256'] == hashlib.sha256(
        Path(__file__).read_bytes()).hexdigest()
    assert 'boundary_covers_required_end' not in PROFILE_PATH.read_text(
        encoding='utf-8')


def test_v11_profile_binds_exact_counts_and_bounded_gap_derivation():
    candidate = _candidate()
    input_binding = candidate['input']
    expected = {'lidar': 5793, 'imu': 225102, 'image': 5792}
    assert input_binding['expected_messages'] == 236687
    assert input_binding['expected_topic_counts'] == expected
    assert input_binding['observed_full_input_exact'] is True
    assert input_binding['observed_received_messages'] == 236687
    assert input_binding['observed_received_topic_counts'] == expected

    phase = candidate['phase']
    terminal = phase['observed_terminal']
    completed = {'lidar': 5793, 'imu': 225024, 'image': 5789}
    support = {'lidar': 0, 'imu': 78, 'image': 3}
    assert terminal['completed_boundary_timestamp_seconds'] == 1623491515.03615
    assert terminal['completed_topic_counts'] == completed
    assert terminal['support_context_counts'] == support
    assert terminal['support_context_total'] == 81
    assert terminal['exact_received_counts'] == expected
    assert {
        topic: completed[topic] + support[topic] for topic in expected
    } == expected

    gap = phase['nonnegative_bounded_trajectory_end_gap']
    required = phase['required_evaluation_end_timestamp_seconds']
    boundary = gap['completed_boundary_timestamp_seconds']
    assert required == 1623491515.148352
    assert boundary == 1623491515.03615
    derived = float(required) - float(boundary)
    assert gap['observed_gap_seconds'] == pytest.approx(derived, abs=1e-9)
    assert gap['observed_gap_seconds'] >= 0.0
    assert gap['observed_gap_seconds'] <= gap['maximum_end_gap_seconds'] == 0.25
    assert gap['finite'] is True
    assert gap['nonnegative'] is True
    assert gap['bounded'] is True
    assert phase['pass_requires']['nonnegative_bounded_trajectory_end_gap'] is True
    assert phase['boundary_at_or_above_required_end_forbidden'] is True


def test_v11_profile_records_root_cause_and_immutable_receipt_bindings():
    candidate = _candidate()
    root_cause = candidate['phase']['root_cause']
    assert root_cause['kind'] == 'v10_contradictory_boundary_predicate'
    assert root_cause['predicate'] == (
        'boundary_.timestamp_seconds < required_end_timestamp_seconds_ ||')
    assert root_cause['v11_correction'] == 'remove_only_the_contradictory_predicate'

    source = candidate['source']
    assert source['v11_delta_patch_sha256'] == (
        '2cbca3a7bb465981cb5e3072efd1713302e002127d4ae5872413ed5749d80ba2')
    execution = candidate['execution']
    assert execution['patch_sha256'] == source['v11_delta_patch_sha256']
    assert execution['image_tag'] == (
        'm6a10-v2c-v11-bounded-end-gap-20260823t121749z-fast-livo2-benchmark:'
        'ros1-pinned')
    assert execution['image_id'] == (
        'sha256:729a7bba2127fc6517c106d59a12668294aeee1a0b31d04d31f9c25c762f6c3a')
    assert execution['build_receipt_sha256'] == (
        '98ceaba27849e23491e19f8f6c7c3392306cd9ff22cfa87048a1810a7b83c8b6')
    assert execution['synthetic_receipt_sha256'] == (
        'eaac94b7572330e09fe889d4868691eb075fbb33c1aabae60d45686055626790')
    assert execution['build_receipt_path'].endswith(
        'fast_livo2_v2c_v11_build_20260823T121749Z_agentv11/'
        'build_identity.receipt.json')
    assert execution['synthetic_receipt_path'].endswith(
        'fast_livo2_v2c_v11_synthetic_20260823T121749Z_agentv11/'
        'synthetic_cpp_gates.receipt.json')


def test_v11_candidate_is_fail_closed_until_dual_service_and_formal_are_allowed():
    profile = _profile()
    candidate = _candidate()
    preregistration = profile['preregistration']
    source = candidate['source']
    execution = candidate['execution']
    safety = candidate['safety']
    assert preregistration['replay_authorized'] is False
    assert preregistration['formal_replay_forbidden'] is True
    assert preregistration['replay_count'] == 0
    assert source['dual_service_no_input_status'] == 'PENDING'
    assert source['dual_service_pending'] is True
    assert source['runtime_ready'] is False
    assert source['runtime_ready_for_preregistered_replay'] is False
    assert execution['no_input_service_gate'] == 'pending_dual_service'
    assert execution['runtime_ready'] is False
    assert execution['formal_replay_forbidden'] is True
    assert execution['replay_count'] == 0
    for field in (
            'input_opened', 'input_mount_performed',
            'ground_truth_content_opened', 'scorer_invoked', 'map_saved'):
        assert execution[field] is False, field
    for field in (
            'ground_truth_content_opened', 'scorer_invoked', 'map_saved',
            'formal_replay_started'):
        assert safety[field] is False, field
    assert safety['formal_replay_forbidden'] is True
    assert safety['replay_count'] == 0
