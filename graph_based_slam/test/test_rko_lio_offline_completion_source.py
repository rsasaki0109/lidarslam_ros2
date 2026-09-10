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

"""Source-level regression checks for RKO-LIO offline completion."""

from __future__ import annotations

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
RKO_ROS_DIR = REPO_ROOT / 'Thirdparty' / 'rko_lio' / 'rko_lio' / 'ros'


def test_offline_completion_does_not_wait_for_trailing_imu_buffer():
    offline_node = (RKO_ROS_DIR / 'offline_node.cpp').read_text(encoding='utf-8')
    node_header = (RKO_ROS_DIR / 'node.hpp').read_text(encoding='utf-8')
    node_cpp = (RKO_ROS_DIR / 'node.cpp').read_text(encoding='utf-8')

    assert 'atomic_registration_active' in node_header
    assert 'atomic_registration_active = true;' in node_cpp
    assert 'atomic_registration_active = false;' in node_cpp
    assert 'Trailing IMU messages are expected' in offline_node
    assert 'lidar_buffer.empty() && !atomic_registration_active' in offline_node
    assert 'imu_buffer.empty() && lidar_buffer.empty()' not in offline_node


def test_v2_drain_fixture_pass_and_timeout_are_distinct_fail_closed_states():
    offline_node = (RKO_ROS_DIR / 'offline_node.cpp').read_text(encoding='utf-8')

    # Keep the fixture deliberately small and representation-only: a pass is
    # valid only for an empty lidar queue, while a timeout remains invalid even
    # when all input counters happen to match.  This mirrors the production
    # diagnostic's fail-closed meaning without opening a bag or GT.
    passing_snapshot = {
        'status': 'eof_observed',
        'lidar_buffer_size': 0,
        'registration_active': False,
        'expected_messages': 3,
        'received_messages': 3,
        'processed_messages': 3,
        'dropped_messages': 0,
        'queue_overflow': 0,
        'processing_failures': 0,
    }
    timeout_snapshot = dict(passing_snapshot, status='timeout', lidar_buffer_size=1)
    assert (
        passing_snapshot['status'] == 'eof_observed'
        and passing_snapshot['lidar_buffer_size'] == 0
        and not passing_snapshot['registration_active']
    )
    assert not (
        timeout_snapshot['status'] == 'eof_observed'
        and timeout_snapshot['lidar_buffer_size'] == 0
        and not timeout_snapshot['registration_active']
    )
    assert 'write_benchmark_drain_diagnostic("eof"' in offline_node
    assert 'write_benchmark_drain_diagnostic(' in offline_node
    assert '"timeout", "benchmark drain timeout before lidar buffer became empty"' in offline_node
    assert 'exit_status = diagnostic_written ? 124 : 1' in offline_node


def test_rko_lio_has_kidnap_relocalization_recovery_path():
    rko_core_dir = REPO_ROOT / 'Thirdparty' / 'rko_lio' / 'rko_lio' / 'core'
    config_path = (
        REPO_ROOT / 'configs' / 'mid360_robot' / 'rko_lio_mid360_kidnap_tolerant.yaml'
    )
    core_header = (rko_core_dir / 'lio.hpp').read_text(encoding='utf-8')
    core_cpp = (REPO_ROOT / 'Thirdparty' / 'rko_lio' / 'rko_lio' / 'core' / 'lio.cpp').read_text(
        encoding='utf-8'
    )
    node_cpp = (RKO_ROS_DIR / 'node.cpp').read_text(encoding='utf-8')
    config = config_path.read_text(encoding='utf-8')

    assert 'enable_kidnap_relocalization' in core_header
    assert 'reset_on_registration_failure' in core_header
    assert 'relocalize_after_scan_gap' in core_header
    assert 'try_global_relocalization' in core_cpp
    assert 'Kidnap relocalization matched' in core_cpp
    assert 'Kidnap recovery accepted scan' in core_cpp
    assert 'relocalization_map.AddPoints' in core_cpp
    assert 'declare_parameter<bool>("enable_kidnap_relocalization"' in node_cpp
    assert 'enable_kidnap_relocalization: true' in config
    assert 'reset_on_registration_failure: true' in config
    assert 'relocalize_after_scan_gap: false' in config
