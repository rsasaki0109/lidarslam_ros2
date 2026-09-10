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

"""Static/unit gates for the v10 inside-container runtime entrypoint."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/fast_livo2_m6a10_v10_container_run.sh'


def _text() -> str:
    return SCRIPT.read_text(encoding='utf-8')


def test_v10_entrypoint_is_bash_syntax_valid_and_executable():
    assert SCRIPT.stat().st_mode & 0o111
    completed = subprocess.run(
        ['bash', '-n', str(SCRIPT)], check=False, capture_output=True, text=True
    )
    assert completed.returncode == 0, completed.stderr


def test_exact_v3_identity_gate_and_pinned_runtime_inputs():
    text = _text()
    required = (
        "EXPECTED_CONTRACT='m6a10-online-compute-v3-terminal-support-context'",
        "EXPECTED_PHASE_MODE='unpaced_ack'",
        "EXPECTED_PROFILE_SHA256='f706f41b0a985347ff98e4953532590c7a9c8c2d75db"
        "355e53cfb558f95bc459'",
        "EXPECTED_BAG_BYTES='11290464091'",
        "EXPECTED_BAG_SHA256='5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310'",
        "EXPECTED_MESSAGES='236687'",
        "EXPECTED_LIDAR_MESSAGES='5793'",
        "EXPECTED_IMU_MESSAGES='225102'",
        "EXPECTED_IMAGE_MESSAGES='5792'",
        "EXPECTED_END_TIMESTAMP='1623491515.148352'",
        "EXPECTED_CALLBACK_LATENCY='0.25'",
        "EXPECTED_BACKLOG='1'",
        "EXPECTED_SENSOR_DURATION='579.278127298'",
        "EXPECTED_TIMING_CONTRACT='m6a10-online-compute-v3-timing-v1'",
        'M6A10_PROFILE_SHA256',
        'M6A10_PROFILE_PATH',
        'M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS',
        'M6A10_CONSUMER_EVIDENCE',
        'M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE',
        'M6A10_ONLINE_TIMING_EVIDENCE',
        'M6A10_SENSOR_DURATION_SECONDS',
        'M6A10_TIMING_CONTRACT_VERSION',
    )
    for marker in required:
        assert marker in text
    assert 'M6A10_FAST_EXPECTED_MESSAGES' in text
    assert 'M6A10_FAST_EXPECTED_LIDAR_MESSAGES' in text
    assert 'M6A10_FAST_EXPECTED_IMU_MESSAGES' in text
    assert 'M6A10_FAST_EXPECTED_IMAGE_MESSAGES' in text
    assert 'M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS' in text
    assert 'M6A10_FAST_MAX_BACKLOG_MESSAGES' in text
    assert 'online_compute_timing.json' in text


def test_callback_and_terminal_evidence_paths_are_fixed_and_distinct():
    text = _text()
    assert 'CALLBACK_EVIDENCE="${M6A10_CONSUMER_EVIDENCE}"' in text
    assert '"${OUT_DIR}/callback_consumer_evidence.json"' in text
    assert '"${OUT_DIR}/consumer_evidence.json"' in text
    assert '[[ "${CALLBACK_EVIDENCE}" != "${RAW_EVIDENCE}" ]]' in text
    assert 'evidence output already exists or is staging' in text


def test_helper_environment_is_exported_before_mapper_start():
    text = _text()
    mapper = text.index('setsid roslaunch fast_livo mapping_ouster_ntu.launch')
    for marker in (
        'export M6A10_CONSUMER_EVIDENCE=',
        'export M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE=',
        'export M6A10_FAST_EXPECTED_MESSAGES',
        'export M6A10_FAST_EXPECTED_LIDAR_MESSAGES',
        'export M6A10_FAST_EXPECTED_IMU_MESSAGES',
        'export M6A10_FAST_EXPECTED_IMAGE_MESSAGES',
        'export M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS',
        'export M6A10_FAST_MAX_BACKLOG_MESSAGES',
        'export M6A10_ONLINE_TIMING_EVIDENCE=',
        'export M6A10_SENSOR_DURATION_SECONDS',
        'export M6A10_TIMING_CONTRACT_VERSION',
    ):
        assert text.index(marker) < mapper, marker


def test_runtime_sequence_uses_exact_services_and_single_feeder():
    text = _text()
    for service in (
        '/m6a10/consumer_status',
        '/m6a10/consumer_ack',
        '/m6a10/terminal_status',
        '/m6a10/terminal_eof',
        '/m6a10/terminal_finalize',
    ):
        assert service in text
    assert 'setsid roscore' in text
    assert 'rosparam set use_sim_time true' in text
    assert 'setsid roslaunch fast_livo mapping_ouster_ntu.launch rviz:=false' in text
    assert 'rostopic echo -p /aft_mapped_to_init' in text
    feeder = 'setsid python3 "${FEEDER_PATH}"'
    assert text.count(feeder) == 1
    assert 'wait_for_trajectory' not in text
    assert 'trajectory_required_end.txt' not in text
    assert text.count('odometry.csv') == 1
    feeder_validation = text.index('python3 - "${OUT_DIR}/feeder_receipt.json"')
    feeder_validation_end = text.index('PY\n\ncall_trigger() {')
    eof = text.index('call_trigger /m6a10/terminal_eof')
    status = text.index('call_trigger /m6a10/terminal_status')
    finalize = text.index('call_trigger /m6a10/terminal_finalize')
    assert feeder_validation < feeder_validation_end < eof < status < finalize
    assert 'odometry.csv' not in text[feeder_validation_end:eof]
    assert 'terminal_status_successes >= 2' in text
    assert 'M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS' in text
    assert 'sleep "${M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS}"' in text


def test_raw_evidence_is_application_owned_and_safety_checked():
    text = _text()
    assert "value.get('schema_version') != 1" in text
    assert 'm6a10-fast-livo2-consumer-terminal-v1' in text
    assert "value.get('ground_truth_content_opened') is not False" in text
    assert "value.get('scorer_invoked') is not False" in text
    assert "value.get('input')" not in text
    assert 'json.dump' not in text
    assert 'json.dumps' not in text
    assert 'M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE' in text
    assert 'consumer_evidence.sha256' in text
    assert 'INPUT_START_MONOTONIC_NS' in text
    assert 'DRAIN_END_MONOTONIC_NS' in text
    assert 'os.O_EXCL' in text
    assert 'os.link(part_path, final_path)' in text
    assert 'online_compute_rtf' in text


def test_no_legacy_replay_drain_or_external_evaluation_surface():
    text = _text().lower()
    assert 'rosbag play' not in text
    assert '/m6a10/consumer_finalize' not in text
    assert '--drain' not in text
    assert 'map_save' not in text
    assert 'docker ' not in text
    assert '--mount' not in text
    assert 'ground_truth_mount' not in text
    assert 'scorer_mount' not in text


def test_process_groups_are_cleaned_on_all_exit_paths():
    text = _text()
    assert 'trap cleanup EXIT' in text
    for pid in ('ROSCORE_PID', 'MAPPER_PID', 'ODOM_PID', 'FEEDER_PID'):
        assert f'stop_group "${{{pid}}}"' in text
        assert f'"${{{pid}}}"' in text
    assert 'kill -TERM -- "-${pid}"' in text
    assert 'wait "${child_pid}"' in text


def test_missing_environment_fails_before_any_runtime_command(tmp_path):
    environment = dict(os.environ)
    for name in (
        'BAG_PATH',
        'M6A10_PROFILE_SHA256',
        'M6A10_PROFILE_PATH',
        'M6A10_BAG_BYTES',
        'M6A10_BAG_SHA256',
        'M6A10_FAST_EXPECTED_MESSAGES',
        'M6A10_FAST_EXPECTED_LIDAR_MESSAGES',
        'M6A10_FAST_EXPECTED_IMU_MESSAGES',
        'M6A10_FAST_EXPECTED_IMAGE_MESSAGES',
        'M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS',
        'M6A10_REQUIRED_END_TIMESTAMP_SECONDS',
        'M6A10_CONSUMER_EVIDENCE',
        'M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE',
        'M6A10_FAST_FEEDER_SHA256',
        'M6A10_PHASE_CONTRACT_VERSION',
        'M6A10_PHASE_MODE',
    ):
        environment.pop(name, None)
    completed = subprocess.run(
        ['bash', str(SCRIPT)],
        cwd=tmp_path,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode != 0
    assert 'BAG_PATH is required' in completed.stderr
    assert 'roscore' not in completed.stderr
