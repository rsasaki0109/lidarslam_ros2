#!/usr/bin/env python3
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

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_m6a10_fixed10_v10.py'
SPEC = importlib.util.spec_from_file_location('m6a10_fixed10_v10', SCRIPT)
V10 = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
sys.modules[SPEC.name] = V10
SPEC.loader.exec_module(V10)


def test_v10_rejects_changed_v9_base_sha(monkeypatch):
    monkeypatch.setattr(V10, 'V9_SHA256', '0' * 64)
    with pytest.raises(RuntimeError, match='pinned v9 launcher'):
        V10.verify_pinned_v9()


def test_v10_defaults_and_runner_argv_use_complete_image_id(tmp_path):
    repo = tmp_path / 'repo'
    input_root = tmp_path / 'input'
    repo.mkdir()
    input_root.mkdir()
    config = V10.LaunchConfig(
        root=tmp_path / 'attempt', repo_root=repo, input_root=input_root)
    assert type(config).__name__ == 'LaunchConfig'
    assert config.image_digest == V10.IMAGE_DIGEST
    assert len(config.image_digest.removeprefix('sha256:')) == 64
    assert V10._IMPL.IMAGE_DIGEST == V10.IMAGE_DIGEST
    argv = V10.build_runner_argv(config, tmp_path / 'out')
    assert V10.IMAGE_DIGEST in argv
    assert (
        'sha256:385b6eeda3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69'
        not in argv)


def test_v10_marker_and_failure_closure_use_v10_contract(tmp_path):
    repo = tmp_path / 'repo'
    input_root = tmp_path / 'input'
    repo.mkdir()
    input_root.mkdir()
    (input_root / 'metadata.yaml').write_text('fixture\n', encoding='utf-8')
    config = V10.LaunchConfig(
        root=tmp_path / 'attempt', repo_root=repo, input_root=input_root,
        expected_input_tree_sha256=V10.tree_sha256(input_root))

    def identity(_config):
        return {'fixture': True}

    def failed_quiescence(_config, path):
        path.write_text(json.dumps({'status': 'FAIL_CLOSED'}), encoding='utf-8')
        return 1

    def should_not_start(*_args, **_kwargs):
        pytest.fail('v10 runner must not start after failed preflight')

    assert V10.run_v10(
        config, identity_validator=identity,
        quiescence_runner=failed_quiescence,
        process_factory=should_not_start) == 10
    marker = json.loads((config.root / 'attempt_root_marker.json').read_text())
    closure = json.loads((config.root / 'closure_receipt.json').read_text())
    assert marker['contract_id'] == V10.CONTRACT_ID
    assert marker['launcher_path'].endswith('run_m6a10_fixed10_v10.py')
    assert closure['contract_id'] == V10.CONTRACT_ID
    assert closure['failure_kind'] == 'PREFLIGHT_FAIL_CLOSED'
    assert not (config.root / 'launch_attempt.json').exists()


def test_v10_identity_receipt_binds_current_launcher(tmp_path, monkeypatch):
    repo = tmp_path / 'repo'
    input_root = tmp_path / 'input'
    repo.mkdir()
    input_root.mkdir()
    (input_root / 'metadata.yaml').write_text('fixture\n', encoding='utf-8')
    config = V10.LaunchConfig(
        root=tmp_path / 'attempt', repo_root=repo, input_root=input_root,
        expected_input_tree_sha256=V10.tree_sha256(input_root))
    monkeypatch.setattr(
        V10, 'validate_preflight_identity',
        lambda _config, **_kwargs: {
            'input_tree_sha256': V10.tree_sha256(input_root),
            'sources': {}, 'image': {'id': V10.IMAGE_DIGEST, 'labels': {}},
        })
    output = tmp_path / 'identity' / 'receipt.json'
    captured = V10.capture_identity_receipt(config, output)
    value = json.loads(output.read_text(encoding='utf-8'))
    assert captured['sha256'] == hashlib.sha256(output.read_bytes()).hexdigest()
    assert value['status'] == 'PASS'
    assert value['launcher']['sha256'] == hashlib.sha256(SCRIPT.read_bytes()).hexdigest()
    assert value['launcher']['sha256'] == captured['receipt']['launcher']['sha256']
    assert value['contract_id'] == V10.CONTRACT_ID
    assert value['execution']['runner_start_attempted'] is False


def test_v10_profile_selection_and_receipt_are_bound():
    profile = yaml.safe_load(
        (ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml')
        .read_text(encoding='utf-8'))['competitive_slam_profile']
    selection = yaml.safe_load(
        (ROOT / 'configs/slam_benchmark_profiles/'
         'competitive_execution_selection_2026-08.yaml')
        .read_text(encoding='utf-8'))
    profile_contract = profile['runtime_policy']['phase_contract_v2'][
        'ours_m6a10_v2a_unpaced_ack_fixed10_v10']
    selection_contract = selection['m6a10_phase_contract']['v2_additive_contract'][
        'ours_m6a10_v2a_unpaced_ack_fixed10_v10']
    assert profile_contract == selection_contract
    assert profile_contract['status'] == 'COMPLETED_GT_BLIND_FUNCTIONAL'
    result = profile_contract['result']
    assert result['status'] == 'PASS'
    assert result['runner_returncode'] == 0
    assert result['consumer_status'] == 'pass'
    assert result['phase_status'] == 'pass'
    assert result['expected_messages'] == 230895
    assert result['received_messages'] == 230895
    assert result['processed_messages'] == 230895
    assert result['dropped_messages'] == 0
    assert result['queue_overflow'] == 0
    assert result['processing_failures'] == 0
    assert result['eof_observed'] is True
    assert result['drain_complete'] is True
    assert result['backlog_at_drain'] == 0
    assert result['online_compute_rtf'] <= 1.0
    assert result['maximum_callback_latency_seconds'] <= 0.25
    assert result['ground_truth_content_opened'] is False
    assert result['scorer_invoked'] is False
    launcher = profile_contract['launcher']
    assert hashlib.sha256((ROOT / launcher['path']).read_bytes()).hexdigest() == (
        launcher['sha256'])
    assert hashlib.sha256((ROOT / launcher['test_path']).read_bytes()).hexdigest() == (
        launcher['test_sha256'])
    receipt_path = Path(launcher['identity_preflight_receipt']['path'])
    assert receipt_path.is_file()
    assert hashlib.sha256(receipt_path.read_bytes()).hexdigest() == (
        launcher['identity_preflight_receipt']['sha256'])
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    assert receipt['launcher']['sha256'] == launcher['sha256']
    assert receipt['identity']['image']['id'] == V10.IMAGE_DIGEST
    assert receipt['execution']['runner_start_attempted'] is False

    output_root = Path(profile_contract['output']['root'])
    closure = json.loads((output_root / 'closure_receipt.json').read_text())
    assert closure['status'] == 'COMPLETED'
    assert closure['runner_returncode'] == 0
    assert closure['completion']['status'] == 'PASS'
    assert closure['completion']['ground_truth_content_opened'] is False
    assert closure['completion']['scorer_invoked'] is False
    for required in profile_contract['output']['required_artifacts']:
        assert (output_root / 'out' / required).is_file()
    assert not list((output_root / 'out').rglob('*.part'))
    assert not list((output_root / 'out').rglob('map.pcd'))


@pytest.mark.skipif(
    __import__('os').environ.get('M6A10_V10_REAL_IDENTITY') != '1',
    reason='set M6A10_V10_REAL_IDENTITY=1 for read-only host identity check')
def test_v10_real_host_identity_opt_in_is_read_only():
    config = V10.LaunchConfig(
        root=Path('/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/'
                  'ours_m6a10_v2a_unpaced_ack_fixed10_v10'),
        repo_root=ROOT)
    identity = V10.validate_preflight_identity(config)
    assert identity['input_tree_sha256'] == V10.EXPECTED_INPUT_TREE_SHA256
    assert identity['image']['id'] == V10.IMAGE_DIGEST
