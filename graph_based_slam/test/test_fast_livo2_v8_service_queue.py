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

"""GT-free contracts for the FAST-LIVO2 M6a10 v8 service-queue gate."""

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
BASE_PATCH = ROOT / 'docker/patches/fast_livo2.m6a10-v2c.patch'
DELTA_PATCH = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v8-service-queue.patch'
V7_DOCKERFILE = ROOT / 'docker/fast_livo2_benchmark.Dockerfile'
V8_DOCKERFILE = ROOT / 'docker/fast_livo2_m6a10_v8.Dockerfile'
V7_BUILD_SCRIPT = ROOT / 'scripts/build_competitive_benchmark_images.sh'
V8_BUILD_SCRIPT = ROOT / 'scripts/build_fast_livo2_m6a10_v8_image.sh'
PROFILE = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v8_formal.yaml'
HANDSHAKE = ROOT / (
    'scripts/preflight_fast_livo2_m6a10_v2c_v8_handshake.py')


def _load_handshake():
    spec = importlib.util.spec_from_file_location('fast_livo2_v8_handshake', HANDSHAKE)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _phase():
    return yaml.safe_load(PROFILE.read_text(encoding='utf-8'))[
        'competitive_slam_profile']['m6a10_fast_livo2_v2c_v8']


def test_v7_base_patch_is_immutable_and_v8_delta_is_separate():
    assert _sha(BASE_PATCH) == (
        '33f30a40ad54db5eea331a87b8b86f32410aac2980e37e4cb6d5027100095297')
    assert _sha(DELTA_PATCH) == (
        'e4a3b2b9d981ec0f740365695f53a662797aaf9400e1d7186551687143c4fd87')
    phase = _phase()
    assert phase['source']['base_patch_sha256'] == _sha(BASE_PATCH)
    assert phase['source']['v8_delta_patch_sha256'] == _sha(DELTA_PATCH)
    assert phase['source']['patch_apply_order'] == ['base_patch', 'v8_delta_patch']
    assert BASE_PATCH != DELTA_PATCH
    assert _sha(V7_DOCKERFILE) == (
        '1b916086ce40f386ea1a4bb2dc591f784c249c81723f6d7e486bca217a9b3da5')
    assert _sha(V7_BUILD_SCRIPT) == (
        '136f52c41da1b090f47c310bb3ce85fd8c317e4824410cecba960815c7ad42d1')


def test_v8_delta_has_valid_added_lines_and_lifecycle_contract():
    text = DELTA_PATCH.read_text(encoding='utf-8')
    assert '+    m6a10_eof_server = m6a10_service_node->advertiseService(' in text
    assert 'ros::CallbackQueue m6a10_service_callback_queue' in text
    assert 'ros::AsyncSpinner' in text
    assert 'queue_snapshot' in text
    assert 'm6a10_eof_server.shutdown();' in text
    assert 'm6a10_service_spinner->stop();' in text
    assert 'm6a10_service_callback_queue' in text
    assert 'stateEstimationAndMapping' not in text


def test_docker_and_build_contract_apply_base_then_delta_without_replay():
    docker = V8_DOCKERFILE.read_text(encoding='utf-8')
    build = V8_BUILD_SCRIPT.read_text(encoding='utf-8')
    assert 'FAST_LIVO2_M6A10_VARIANT=v8' in docker
    assert 'FAST_LIVO2_M6A10_V8_DELTA_PATCH_SHA256=' in docker
    assert 'COPY docker/patches/fast_livo2.m6a10-v2c.patch' in docker
    assert 'COPY docker/patches/fast_livo2.m6a10-v2c-v8-service-queue.patch' in docker
    assert docker.index('fast_livo2.m6a10-v2c.patch') < docker.index(
        'fast_livo2.m6a10-v2c-v8-service-queue.patch')
    assert 'apply --check --recount' in docker
    assert 'benchmark.fast_livo2.m6a10_base_patch_sha256' in docker
    assert 'benchmark.fast_livo2.m6a10_v8_delta_patch_sha256' in docker
    assert 'benchmark.fast_livo2.m6a10_service_handshake_contract' in docker
    assert 'fast_livo2_m6a10_v8.Dockerfile' in build
    assert 'FAST_LIVO2_M6A10_VARIANT=v8' in build
    assert 'FAST_LIVO2_M6A10_V8_DELTA_PATCH_SHA256' in build
    assert 'FAST_LIVO2_M6A10_CONSUMER_CONTRACT=' in build
    assert 'FAST_LIVO2_M6A10_SERVICE_HANDSHAKE_CONTRACT=' in build


def test_v8_profile_is_preregistered_and_input_only():
    phase = _phase()
    assert phase['status'] == 'preregistered_not_built'
    assert phase['result'] is None
    assert phase['execution']['image_id'] is None
    assert phase['execution']['image_reuse_forbidden'] is True
    assert phase['safety']['ground_truth_content_opened'] is False
    assert phase['safety']['scorer_invoked'] is False
    handshake = phase['handshake_preflight']
    assert handshake['input_mount_performed'] is False
    assert handshake['host_mounts'] == []
    assert handshake['simulated_time_idle_probe'] is True
    assert handshake['ack_without_callback_must_reject'] is True


def test_handshake_command_is_live_but_has_no_input_or_gt_mount():
    module = _load_handshake()
    command = module.handshake_command(module.IMAGE_TAG)
    assert command[:6] == [
        'docker', 'run', '--pull=never', '--rm', '--init', '--network']
    assert '--read-only' in command
    assert not any(value in ('-v', '--volume', '--mount') for value in command)
    script = command[-1]
    assert 'rosservice call /m6a10/consumer_status' in script
    assert 'rosservice call /m6a10/consumer_ack' in script
    assert 'rosservice call /m6a10/consumer_eof' in script
    assert 'rosservice call /m6a10/consumer_finalize' in script
    assert 'BAG_PATH' not in script
    assert '/input/' not in script
    assert '--mount' not in script


def test_handshake_parser_requires_all_live_service_checks():
    module = _load_handshake()
    good = {
        'status': 'PASS',
        'contract_id': module.HANDSHAKE_CONTRACT_ID,
        'services': {
            'status': True,
            'ack_rejected_without_callback': True,
            'eof': True,
            'finalize': True,
        },
        'received_messages': 0,
        'acknowledged_messages': 0,
        'input_mount_performed': False,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    stdout = 'M6A10_HANDSHAKE ' + json.dumps(good, separators=(',', ':')) + '\n'
    assert module.parse_handshake(stdout) == good
    bad = dict(good)
    bad['services'] = dict(good['services'], finalize=False)
    with pytest.raises(module.PreflightError):
        module.parse_handshake(
            'M6A10_HANDSHAKE ' + json.dumps(bad, separators=(',', ':')) + '\n')


def test_live_handshake_runner_is_bounded_and_returns_machine_record():
    module = _load_handshake()
    good = {
        'status': 'PASS',
        'contract_id': module.HANDSHAKE_CONTRACT_ID,
        'services': {
            'status': True,
            'ack_rejected_without_callback': True,
            'eof': True,
            'finalize': True,
        },
        'received_messages': 0,
        'acknowledged_messages': 0,
        'input_mount_performed': False,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    calls = []

    def fake_runner(command, *, timeout):
        calls.append((command, timeout))
        return subprocess.CompletedProcess(
            command, 0, stdout='M6A10_HANDSHAKE ' + json.dumps(good) + '\n',
            stderr='')

    result = module.run_handshake('sha256:' + 'a' * 64, timeout=7,
                                  runner=fake_runner)
    assert result['exit_status'] == 0
    assert result['result']['services']['finalize'] is True
    assert calls[0][1] == 7
    assert not any(value in calls[0][0] for value in ('-v', '--volume', '--mount'))


def test_receipt_is_atomic_and_refuses_overwrite(tmp_path):
    module = _load_handshake()
    output = tmp_path / 'handshake.json'
    digest = module.atomic_write(output, {'status': 'PASS'})
    assert _sha(output) == digest
    with pytest.raises(module.PreflightError):
        module.atomic_write(output, {'status': 'PASS'})
    assert not list(tmp_path.glob('*.part'))
