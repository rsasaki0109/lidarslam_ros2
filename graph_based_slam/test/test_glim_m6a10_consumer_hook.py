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

"""Static and synthetic checks for the opt-in GLIM M6a10 hook."""

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
PROFILE_PATH = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
SELECTION_PATH = ROOT / (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_2026-08.yaml')
CORE_PATCH = ROOT / 'docker/patches/glim.m6a10-v2b.patch'
ROS2_PATCH = ROOT / 'docker/patches/glim_ros2.m6a10-v2b.patch'
WRAPPER = ROOT / 'scripts/glim_container_run.sh'
PHASE_BRIDGE = ROOT / 'scripts/container_phase_evidence.sh'
CONTRACT = ROOT / 'scripts/benchmark_phase_contract.py'


def _load(path):
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _load_contract():
    spec = importlib.util.spec_from_file_location('phase_contract', CONTRACT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _sidecar():
    return {
        'schema_version': 1,
        'kind': 'm6a10_consumer_eof_boundary',
        'contract_version': 'm6a10-online-compute-v2',
        'phase_mode': 'unpaced_ack',
        'status': 'eof',
        'eof_observed': True,
        'eof_source': 'fixture.reader_eof',
        'expected_messages': 3,
        'received_messages': 3,
        'processed_messages': 3,
        'dropped_messages': 0,
        'queue_overflow': 0,
        'processing_failures': 0,
        'first_processed_timestamp_seconds': 1.0,
        'last_processed_timestamp_seconds': 2.0,
    }


def test_glim_v2b_profile_and_selection_close_v3_fail_closed_with_v2_lineage():
    profile = _load(PROFILE_PATH)['competitive_slam_profile']
    contract = profile['m6a10_glim_v2b']
    selection = _load(SELECTION_PATH)['m6a10_phase_contract']
    selected = selection['v2_additive_contract']['glim_m6a10_v2b']
    for value in (contract, selected):
        assert value['status'] == 'INVALID_SAFETY'
        assert value['result']['status'] == 'INVALID_SAFETY'
        assert value['result']['closure_receipt_sha256'] == (
            '492d39b1f7c4a1a2e86893dd979b0ed57b3f37267cb7f21849b990d95bcc32ff')
        assert value['result']['failure_reason'] == (
            'phase_evidence_consumer_backlog_bound_exceeded')
        assert value['lineage']['prior_fixed10_v2']['status'] == 'INVALID_SAFETY'
        assert value['lineage']['prior_fixed10_v2']['closure_receipt_sha256'] == (
            '72d8ef9cd2c8e26d4c2120463fdbff3057a75e3f81203863049c11028baa41d5')
        assert value['input']['expected_messages'] == 236687
        assert value['input']['lidar_messages'] == 5793
        assert value['input']['imu_messages'] == 225102
        assert value['input']['image_messages'] == 5792
        assert value['config']['topics'] == [
            '/os1_cloud_node1/points', '/imu/imu', '/left/image_raw']
        assert value['build']['build_with_cv_bridge'] == 'ON'
        assert value['build']['status'] == 'PASS'
        assert value['build']['image_tag'] == (
            'm6a10-v2b-20260823-glim-cpu-benchmark:competitive-v1')
        assert value['build']['image_digest'] == (
            'sha256:010c0019a077116edf4d1e7462dfa28561c4fb17db3b5db52e3652c8a875eb41')
        assert value['build']['build_receipt_sha256'] == (
            'fe4d7e3b3b3fec28185f5faa016b19dfb30bd9a52928b099a68d6679e0a09df2')
        assert value['build']['image_inspect_sha256'] == (
            '0c3f096fbb5193d6a3f93aabf5236bc3bf4eaf7eec3e069e6f0576dc33ae9d2a')
        assert value['build']['installed_verification_status'] == 'PASS'
        assert value['build']['installed_verification_sha256'] == (
            '17af1a7b3fbf55c5def2b02327acc3d6260d9fec887ab39092c0bca8678b15e2')
        assert value['build']['marker_verification_sha256'] == (
            'bd209654a9be2fa9880fb7f8c28732013a25dc29bce72d908d121b71d1a3597c')
        assert value['build']['execution_identity_preflight_path'] == (
            '/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/'
            'm6a10-v2b-execution-preflight-20260823-v3/'
            'execution_identity_preflight.json')
        assert value['build']['execution_identity_preflight_sha256'] == (
            '9b8cc1d089a73113930557c6558a32a5ded3725d386b20a78c61fc14e740e190')
        assert value['build']['execution_identity_preflight_status'] == 'PASS'
        assert value['config']['tree_sha256'] == (
            '842be775f7ee4b555f60957cf9f4cc8c35eb790e3ffc3bca767170c6415bb942')
        assert value['phase']['ack_source_kind'] == 'consumer_callback'
        assert value['phase']['maximum_backlog_messages'] > 0
        assert value['phase']['single_message_buffer_verified'] is False
        assert value['phase']['validator_sha256'] == hashlib.sha256(
            (ROOT / value['phase']['validator_path']).read_bytes()).hexdigest()
        assert value['runner']['path'] == 'scripts/run_glim_benchmark.py'
        assert value['runner']['sha256'] == hashlib.sha256(
            (ROOT / value['runner']['path']).read_bytes()).hexdigest()
        assert value['tests']['sha256'] == hashlib.sha256(
            (ROOT / value['tests']['path']).read_bytes()).hexdigest()
        for section, path_key, sha_key in (
                ('source', 'core_patch_path', 'core_patch_sha256'),
                ('source', 'ros2_patch_path', 'ros2_patch_sha256'),
                ('build', 'dockerfile_path', 'dockerfile_sha256'),
                ('build', 'build_script_path', 'build_script_sha256'),
                ('wrapper', 'path', 'sha256')):
            assert value[section][sha_key] == hashlib.sha256(
                (ROOT / value[section][path_key]).read_bytes()).hexdigest()
        assert value['safety']['benchmark_run_executed'] is True
        assert value['safety']['ground_truth_mount_exposed'] is False
        assert value['safety']['docker_build_executed'] is True
        assert value['safety']['new_build_executed'] is False
        assert value['safety']['strict_gt_blind_closure'] is False
        assert value['safety']['retry'] == 0


def test_glim_system_identity_tracks_v2b_preflight_without_legacy_rewrite():
    selection = _load(SELECTION_PATH)
    system = selection['systems']['glim']
    assert system['runner']['sha256'] == hashlib.sha256(
        (ROOT / 'scripts/run_glim_benchmark.py').read_bytes()).hexdigest()
    assert system['runner']['container_entrypoint_sha256'] == hashlib.sha256(
        (ROOT / 'scripts/glim_container_run.sh').read_bytes()).hexdigest()
    assert system['container']['image_tag'] == (
        'm6a10-v2b-20260823-glim-cpu-benchmark:competitive-v1')
    assert system['container']['image_id'] == (
        'sha256:010c0019a077116edf4d1e7462dfa28561c4fb17db3b5db52e3652c8a875eb41')
    assert system['container']['image_digest'] == system['container']['image_id']
    assert system['container']['recipe']['sha256'] == hashlib.sha256(
        (ROOT / 'docker/glim_cpu_benchmark.Dockerfile').read_bytes()).hexdigest()
    assert system['container']['build_entrypoint_sha256'] == hashlib.sha256(
        (ROOT / 'scripts/build_competitive_benchmark_images.sh').read_bytes()).hexdigest()
    assert system['container']['build_receipt_sha256'] == (
        'fe4d7e3b3b3fec28185f5faa016b19dfb30bd9a52928b099a68d6679e0a09df2')
    assert system['container']['execution_identity_preflight_sha256'] == (
        '9b8cc1d089a73113930557c6558a32a5ded3725d386b20a78c61fc14e740e190')
    assert system['toolchain']['image_digest'] == system['container']['image_id']
    assert system['toolchain']['execution_identity_preflight_sha256'] == (
        '9b8cc1d089a73113930557c6558a32a5ded3725d386b20a78c61fc14e740e190')


def test_glim_fixed10_v4_is_preregistered_with_bound_and_new_root():
    profile = _load(PROFILE_PATH)['competitive_slam_profile']
    selection = _load(SELECTION_PATH)['m6a10_phase_contract']
    values = (
        profile['m6a10_glim_v2b_fixed10_v4'],
        selection['v2_additive_contract']['glim_m6a10_v2b_fixed10_v4'],
    )
    for value in values:
        assert value['contract_id'] == (
            'm6a10-v2b-glim-consumer-followability-fixed10-v4')
        assert value['status'] == 'PASS'
        assert value['result']['status'] == 'PASS'
        assert value['result']['closure_receipt_sha256'] == (
            'f33953c4126939dfce841b030cdb2e5560615d42963ae4aaaf4ab0aad7a7f6c1')
        assert value['predecessor']['closure_receipt_sha256'] == (
            '492d39b1f7c4a1a2e86893dd979b0ed57b3f37267cb7f21849b990d95bcc32ff')
        assert value['phase']['maximum_backlog_messages'] == 100000
        assert value['phase']['validator_sha256'] == hashlib.sha256(
            (ROOT / value['phase']['validator_path']).read_bytes()).hexdigest()
        assert value['runner']['sha256'] == hashlib.sha256(
            (ROOT / value['runner']['path']).read_bytes()).hexdigest()
        assert value['output']['evidence_root'].endswith(
            'glim_m6a10_v2b_unpaced_ack_fixed10_v4')
        assert value['build']['execution_identity_preflight_sha256'] == (
            'fda87bce9af6c60a60fcc3523e3f64f4b0fba288f2eea53988432085ab95af8a')
        assert value['build']['execution_identity_preflight_status'] == 'PASS'
        assert value['quiescence_preflight']['receipt_sha256'] == (
            'de015c5b7530c66ba222d1c9fcfdc20d486fbde8ee8cfec36a01c0a677814894')
        assert value['quiescence_preflight']['status'] == 'PASS'
        assert value['safety']['benchmark_run_executed'] is True
        assert value['safety']['ground_truth_content_opened'] is False
        assert value['safety']['scorer_invoked'] is False


def test_glim_v2b_patch_and_wrapper_have_atomic_consumer_boundaries():
    for patch in (CORE_PATCH, ROS2_PATCH):
        lines = patch.read_text(encoding='utf-8').splitlines()
        assert '++' not in lines
    ros2_text = ROS2_PATCH.read_text(encoding='utf-8')
    for marker in (
            'benchmark_queue_state', 'write_eof_boundary',
            'consumer_callback', 'single_message_buffer_verified',
            'glim->wait(auto_quit)'):
        assert marker in ros2_text
    assert ros2_text.index('consumer.callback_returned()') < \
        ros2_text.index('consumer.processed(')
    assert ros2_text.index('consumer.write_eof_boundary()') < \
        ros2_text.index('glim->wait(auto_quit)')
    assert ros2_text.index('glim->wait(auto_quit)') < \
        ros2_text.index('consumer.write()') < ros2_text.index('glim->save')
    wrapper = WRAPPER.read_text(encoding='utf-8')
    for marker in (
            'M6A10_CONSUMER_EVIDENCE', 'M6A10_CONSUMER_EOF_EVIDENCE',
            'consumer-eof-state', 'consumer-state',
            'M6A10_GLIM_MAX_BACKLOG_MESSAGES'):
        assert marker in wrapper
    bridge = PHASE_BRIDGE.read_text(encoding='utf-8')
    assert '--maximum-backlog-messages "${phase_max_backlog}"' in bridge
    assert 'grep -Eiq' not in wrapper


def test_glim_v2b_recipe_and_host_entrypoint_are_content_pinned():
    profile = _load(PROFILE_PATH)['competitive_slam_profile']
    phase = profile['m6a10_glim_v2b']
    recipe = (ROOT / phase['build']['dockerfile_path']).read_text(
        encoding='utf-8')
    build = (ROOT / phase['build']['build_script_path']).read_text(
        encoding='utf-8')
    for arg, value in (
            ('GLIM_M6A10_CORE_PATCH_SHA256',
             phase['source']['core_patch_sha256']),
            ('GLIM_M6A10_ROS2_PATCH_SHA256',
             phase['source']['ros2_patch_sha256'])):
        assert f'ARG {arg}={value}' in recipe
        assert value in build
    assert 'BUILD_WITH_CV_BRIDGE=${GLIM_BUILD_WITH_CV_BRIDGE}' in recipe
    assert 'benchmark.glim.build_with_cv_bridge=' in recipe
    assert '--build-arg "GLIM_BUILD_WITH_CV_BRIDGE=$GLIM_BUILD_WITH_CV_BRIDGE"' in build


def test_glim_v2b_source_patches_apply_cleanly_when_mirrors_exist():
    mirrors = (
        ('/media/sasaki/aiueo1/benchmarks/glim/glim', CORE_PATCH),
        ('/media/sasaki/aiueo1/benchmarks/glim/glim_ros2', ROS2_PATCH),
    )
    for source, patch in mirrors:
        if not Path(source).exists():
            pytest.skip(f'GLIM source mirror unavailable: {source}')
        result = subprocess.run(
            ['git', '-C', source, 'apply', '--check', '--recount', str(patch)],
            check=False, capture_output=True, text=True)
        assert result.returncode == 0, result.stderr


def test_consumer_eof_sidecar_validator_and_tamper():
    phase = _load_contract()
    sidecar = _sidecar()
    phase._validate_consumer_eof_boundary(sidecar)
    sidecar['processed_messages'] = 2
    with pytest.raises(phase.PhaseContractError):
        phase._validate_consumer_eof_boundary(sidecar)


def test_consumer_eof_state_cli_rejects_part_and_tamper(tmp_path):
    sidecar_path = tmp_path / 'consumer_evidence.json.eof.json'
    sidecar_path.write_text(json.dumps(_sidecar()), encoding='utf-8')
    command = [
        'python3', str(CONTRACT), 'consumer-eof-state', '--input',
        str(sidecar_path)]
    result = subprocess.run(command, check=False, capture_output=True)
    assert result.returncode == 0, result.stderr.decode()
    sidecar_path.with_name(sidecar_path.name + '.part').write_text(
        '{}', encoding='utf-8')
    result = subprocess.run(command, check=False, capture_output=True)
    assert result.returncode == 1
    sidecar_path.with_name(sidecar_path.name + '.part').unlink()
    sidecar_path.write_text(
        json.dumps({**_sidecar(), 'dropped_messages': 1}), encoding='utf-8')
    result = subprocess.run(command, check=False, capture_output=True)
    assert result.returncode == 1
