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

"""GT-free source, synthetic-handshake, and profile contracts for FAST v9."""

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
PATCH = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v9-wallrate.patch'
V8_PATCH = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v8-service-queue.patch'
DOCKERFILE = ROOT / 'docker/fast_livo2_m6a10_v9.Dockerfile'
BUILD = ROOT / 'scripts/build_fast_livo2_m6a10_v9_image.sh'
PREFLIGHT = ROOT / 'scripts/preflight_fast_livo2_m6a10_v2c_v9_handshake.py'
PROFILE = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v9_formal.yaml'
RUNNER = ROOT / 'scripts/run_fast_livo2_benchmark.py'
ROOT_CAUSE = ROOT / 'docs/architecture/fast-livo2-m6a10-v8-v9-wallrate-addendum.md'


def _module():
    spec = importlib.util.spec_from_file_location('fast_livo2_v9_handshake', PREFLIGHT)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def _phase():
    return yaml.safe_load(PROFILE.read_text(encoding='utf-8'))['competitive_slam_profile'][
        'm6a10_fast_livo2_v2c_v9'
    ]


def test_v9_patch_is_separate_and_applies_after_v8(tmp_path):
    assert hashlib.sha256(PATCH.read_bytes()).hexdigest() == (
        '1cf3067a634a34d5c3a2126f9f495fcb675068cb94edf562eb7e84b1e470146b'
    )
    text = PATCH.read_text(encoding='utf-8')
    assert '-  ros::Rate rate(5000);' in text
    assert '+  ros::WallRate rate(5000);' in text
    assert 'ros::spinOnce()' in text
    assert '5000 Hz loop cadence' in text
    assert 'stateEstimationAndMapping' not in text
    assert PATCH.read_bytes() != V8_PATCH.read_bytes()
    phase = _phase()
    assert phase['source']['patch_apply_order'] == [
        'base_patch',
        'v8_delta_patch',
        'v9_delta_patch',
    ]
    assert phase['source']['v9_delta_patch_sha256'] == (
        hashlib.sha256(PATCH.read_bytes()).hexdigest()
    )


def test_v9_patch_applies_to_run_loop_fixture_and_changes_only_clock_type(tmp_path):
    repo = tmp_path / 'source'
    (repo / 'src').mkdir(parents=True)
    (repo / 'src' / 'LIVMapper.cpp').write_text(
        'void LIVMapper::run() \n'
        '{\n'
        '  ros::Rate rate(5000);\n'
        '  while (ros::ok()) \n'
        '  {\n'
        '    ros::spinOnce();\n'
        '  }\n',
        encoding='utf-8',
    )
    subprocess.run(['git', 'init', '-q', str(repo)], check=True)
    subprocess.run(['git', '-C', str(repo), 'add', 'src/LIVMapper.cpp'], check=True)
    subprocess.run(
        ['git', '-C', str(repo), 'apply', '--check', '--recount', str(PATCH)], check=True
    )
    subprocess.run(['git', '-C', str(repo), 'apply', '--recount', str(PATCH)], check=True)
    value = (repo / 'src' / 'LIVMapper.cpp').read_text(encoding='utf-8')
    assert 'ros::WallRate rate(5000);' in value
    assert 'ros::Rate rate(5000);' not in value
    assert value.count('ros::spinOnce();') == 1


def test_v9_recipe_and_build_gate_verify_all_source_layers_without_replay():
    docker = DOCKERFILE.read_text(encoding='utf-8')
    build = BUILD.read_text(encoding='utf-8')
    assert docker.index('fast_livo2.m6a10-v2c.patch') < docker.index(
        'fast_livo2.m6a10-v2c-v8-service-queue.patch'
    )
    assert docker.index('fast_livo2.m6a10-v2c-v8-service-queue.patch') < docker.index(
        'fast_livo2.m6a10-v2c-v9-wallrate.patch'
    )
    assert 'git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --check --recount' in docker
    assert "grep -q 'ros::WallRate rate(5000)'" in docker
    assert 'FAST_LIVO2_M6A10_VARIANT=v9' in build
    assert 'FAST_LIVO2_M6A10_V9_DELTA_PATCH_SHA256' in build
    assert 'bag replay' in build.lower()
    assert 'benchmark.fast_livo2.m6a10_v9_delta_patch_sha256' in docker
    assert 'benchmark.fast_livo2.m6a10_service_handshake_contract' in docker


def test_root_cause_addendum_binds_ordering_and_immutable_v8_accounting():
    text = ROOT_CAUSE.read_text(encoding='utf-8')
    assert 'ros::init' in text
    assert 'initializeSubscribersAndPublishers' in text
    assert 'mapper.run()' in text
    assert 'ros::spinOnce()' in text
    assert 'ros::Rate rate(5000)' in text
    assert 'use_sim_time=true' in text
    assert 'no `/clock` message' in text
    assert 'b94b96f153423964a01515ab0e0208a15eaf3f68fe263216726d89601165d7fd' in text
    assert 'publisher_returned=true' in text
    assert 'does not claim' in text


def test_synthetic_handshake_is_one_lidar_no_mount_and_no_estimator_claim():
    module = _module()
    script = module.handshake_script()
    assert 'PointCloud2' in script
    assert "PointField('x', 0, PointField.FLOAT32, 1)" in script
    assert "PointField('range', 32, PointField.UINT32, 1)" in script
    assert 'publisher.publish(message)' in script
    assert "'publisher_returned': True" in script
    assert "'/os1_cloud_node1/points'" in script
    assert 'M6A10_FAST_EXPECTED_MESSAGES=1' in script
    assert 'M6A10_FAST_EXPECTED_IMU_MESSAGES=0' in script
    assert 'M6A10_FAST_EXPECTED_IMAGE_MESSAGES=0' in script
    assert 'finalize_not_attempted_due_to_uninitialized_estimator_buffers' in script
    command = module.handshake_command(module.IMAGE_TAG)
    assert command[:6] == ['docker', 'run', '--pull=never', '--rm', '--init', '--network']
    assert '--read-only' in command
    assert not any(value in ('-v', '--volume', '--mount') for value in command)


def test_synthetic_handshake_parser_requires_exact_one_and_safety():
    module = _module()
    record = {
        'status': 'PASS',
        'contract_id': module.HANDSHAKE_CONTRACT_ID,
        'services': {'status': True, 'ack': True, 'duplicate_ack_rejected': True, 'eof': True},
        'publisher_returned': True,
        'published_topic': module.LIDAR_TOPIC,
        'published_count': 1,
        'received_messages': 1,
        'received_topic_counts': {'lidar': 1, 'imu': 0, 'image': 0},
        'acknowledged_messages': 1,
        'dropped_messages': 0,
        'queue_overflow': 0,
        'processing_failures': 0,
        'maximum_callback_latency_seconds': 0.02,
        'estimator_claim': False,
        'estimator_prerequisites_supplied': False,
        'input_mount_performed': False,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    assert module.parse_handshake('M6A10_HANDSHAKE ' + json.dumps(record) + '\n') == record
    bad = dict(record, acknowledged_messages=2)
    with pytest.raises(module.PreflightError):
        module.parse_handshake('M6A10_HANDSHAKE ' + json.dumps(bad) + '\n')


def test_v9_profile_binds_explicit_key_and_runner_hashes():
    phase = _phase()
    assert phase['profile_key'] == 'm6a10_fast_livo2_v2c_v9'
    assert phase['status'] == 'preregistered_not_built'
    assert phase['result'] is None
    assert phase['execution']['image_digest'] is None
    assert phase['safety']['formal_replay_started'] is False
    assert phase['handshake_preflight']['input_mount_performed'] is False
    assert phase['handshake_preflight']['estimator_claim'] is False
    assert hashlib.sha256(PREFLIGHT.read_bytes()).hexdigest() == (
        phase['handshake_preflight']['script_sha256']
    )
    assert hashlib.sha256(RUNNER.read_bytes()).hexdigest() == phase['runner']['sha256']


def test_runner_selects_v8_and_v9_by_persisted_key_without_phase_monkeypatch():
    spec = importlib.util.spec_from_file_location('fast_runner', RUNNER)
    runner = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(runner)
    v8 = runner.load_m6a10_fast_contract(
        ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v8_formal.yaml',
        'm6a10_fast_livo2_v2c_v8',
    )
    v9 = runner.load_m6a10_fast_contract(PROFILE, 'm6a10_fast_livo2_v2c_v9')
    assert v8['contract_id'] == 'm6a10-v2c-fast-livo2-service-queue-v8'
    assert v9['contract_id'] == 'm6a10-v2c-fast-livo2-wallrate-v9'
    assert v9['profile_key'] == 'm6a10_fast_livo2_v2c_v9'
    with pytest.raises(ValueError, match='selected M6a10 key'):
        runner.load_m6a10_fast_contract(PROFILE, 'm6a10_fast_livo2_v2c_v8')


def test_v9_identity_probe_is_bounded_and_not_run_by_static_tests():
    source = RUNNER.read_text(encoding='utf-8')
    assert '--m6a10-profile-key' in source
    assert 'M6A10_PROFILE_SCHEMA' in source
    assert 'selected_profile_key' in source
    assert 'M6A10_PROFILE_KEY=' in source
    assert 'in-memory phase' not in source
