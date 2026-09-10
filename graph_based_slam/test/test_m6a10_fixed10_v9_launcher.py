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
SCRIPT = ROOT / 'scripts' / 'run_m6a10_fixed10_v9.py'
SPEC = importlib.util.spec_from_file_location('m6a10_fixed10_v9', SCRIPT)
V9 = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
sys.modules[SPEC.name] = V9
SPEC.loader.exec_module(V9)


def test_v9_defaults_use_complete_image_id_and_canonical_tree(tmp_path):
    config = V9.LaunchConfig(root=tmp_path / 'attempt', repo_root=tmp_path)
    assert config.image_digest == V9.IMAGE_DIGEST
    assert config.image_digest == (
        'sha256:385b6eeedae3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69')
    assert len(config.image_digest.removeprefix('sha256:')) == 64
    assert V9._IMPL.IMAGE_DIGEST == V9.IMAGE_DIGEST
    assert V9.TREE_HASH_KIND == 'relative_path_size_content_sha256_v1'
    tree = tmp_path / 'tree'
    (tree / 'sub').mkdir(parents=True)
    (tree / 'a.txt').write_bytes(b'abc')
    (tree / 'sub' / 'b.txt').write_bytes(b'z')
    assert V9.tree_sha256(tree) == (
        '62a6652bfe7117f13f6738aeecf1464d4b570023638f7e8368be926a6b533b16')


def test_v9_rejects_malformed_v7_digest_before_identity_probe(tmp_path):
    config = V9.LaunchConfig(
        root=tmp_path / 'attempt', repo_root=tmp_path,
        image_digest='sha256:385b6eeda3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69')
    with pytest.raises(V9.LaunchError, match='complete pinned local image ID'):
        V9.validate_preflight_identity(
            config, image_probe=lambda _config: pytest.fail('probe must not run'))


def test_v9_runner_argv_uses_complete_image_id(tmp_path):
    repo = tmp_path / 'repo'
    input_root = tmp_path / 'input'
    repo.mkdir()
    input_root.mkdir()
    config = V9.LaunchConfig(
        root=tmp_path / 'attempt', repo_root=repo, input_root=input_root)
    argv = V9.build_runner_argv(config, tmp_path / 'out')
    assert V9.IMAGE_DIGEST in argv
    assert (
        'sha256:385b6eeda3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69'
        not in argv)


def test_v9_identity_probe_accepts_only_complete_image_id(tmp_path, monkeypatch):
    config = V9.LaunchConfig(root=tmp_path / 'attempt', repo_root=tmp_path)
    monkeypatch.setattr(
        V9._IMPL, 'validate_preflight_identity',
        lambda _config, **_kwargs: {'image': {'id': 'sha256:bad'}})
    with pytest.raises(V9.LaunchError, match='observed image ID'):
        V9.validate_preflight_identity(
            config, image_probe=lambda _config: {'id': 'sha256:bad', 'labels': {}})


def test_v9_identity_receipt_is_atomic_and_gt_blind(tmp_path):
    repo = tmp_path / 'repo'
    input_root = tmp_path / 'input'
    repo.mkdir()
    input_root.mkdir()
    (input_root / 'metadata.yaml').write_text('fixture\n', encoding='utf-8')
    config = V9.LaunchConfig(
        root=tmp_path / 'attempt', repo_root=repo, input_root=input_root,
        expected_input_tree_sha256=V9.tree_sha256(input_root))

    def fake_identity(_config, **_kwargs):
        return {
            'input_tree_sha256': V9.tree_sha256(input_root),
            'sources': {},
            'image': {'id': V9.IMAGE_DIGEST, 'labels': {}},
        }

    original = V9.validate_preflight_identity
    V9.validate_preflight_identity = fake_identity
    try:
        output = tmp_path / 'identity' / 'receipt.json'
        captured = V9.capture_identity_receipt(config, output)
    finally:
        V9.validate_preflight_identity = original
    assert output.is_file()
    assert not output.with_name('receipt.json.part').exists()
    assert captured['sha256'] == hashlib.sha256(output.read_bytes()).hexdigest()
    value = json.loads(output.read_text(encoding='utf-8'))
    assert value['status'] == 'PASS'
    assert value['execution']['runner_start_attempted'] is False
    assert value['execution']['ground_truth_content_opened'] is False


def test_v9_profile_selection_and_real_identity_receipt_are_bound():
    profile = yaml.safe_load(
        (ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml')
        .read_text(encoding='utf-8'))['competitive_slam_profile']
    selection = yaml.safe_load(
        (ROOT / 'configs/slam_benchmark_profiles/'
         'competitive_execution_selection_2026-08.yaml')
        .read_text(encoding='utf-8'))
    profile_contract = profile['runtime_policy']['phase_contract_v2'][
        'ours_m6a10_v2a_unpaced_ack_fixed10_v9']
    selection_contract = selection['m6a10_phase_contract']['v2_additive_contract'][
        'ours_m6a10_v2a_unpaced_ack_fixed10_v9']
    assert profile_contract == selection_contract
    assert profile_contract['status'] == 'FAIL_CLOSED'
    assert profile_contract['result'] is None
    assert profile_contract['execution']['image_digest'] == V9.IMAGE_DIGEST
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
    assert receipt['status'] == 'PASS'
    assert receipt['identity']['image']['id'] == V9.IMAGE_DIGEST
    assert receipt['execution']['runner_start_attempted'] is False
    closure = profile_contract['failure_closure']
    assert closure['failure_kind'] == 'PREFLIGHT_FAIL_CLOSED'
    assert closure['quiescence_returncode'] == 1
    assert closure['runner_start_attempted'] is False
    assert closure['load1_per_cpu'] > closure['max_load1_per_cpu']
    attempt = profile_contract['quiescence_attempt']
    assert attempt['status'] == 'FAIL_CLOSED'
    assert attempt['runner_start_allowed'] is False
    assert attempt['retry_count'] == 0
    assert attempt['receipt_sha256'] == (
        '585be4f8da6fb1de8481139ee5a5497aaba8755e3225e56eb298faed3bb5d443')
    superseded = launcher['identity_preflight_receipt']['superseded_identity_receipt']
    old_receipt = Path(superseded['path'])
    assert superseded['status'] == 'superseded_not_promoted'
    assert hashlib.sha256(old_receipt.read_bytes()).hexdigest() == superseded['sha256']
    assert superseded['launcher_sha256'] != launcher['sha256']


@pytest.mark.skipif(
    __import__('os').environ.get('M6A10_V9_REAL_IDENTITY') != '1',
    reason='set M6A10_V9_REAL_IDENTITY=1 for read-only host identity check')
def test_v9_real_host_identity_opt_in_is_read_only():
    config = V9.LaunchConfig(
        root=Path('/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/'
                  'ours_m6a10_v2a_unpaced_ack_fixed10_v9'),
        repo_root=ROOT)
    identity = V9.validate_preflight_identity(config)
    assert identity['input_tree_sha256'] == V9.EXPECTED_INPUT_TREE_SHA256
    assert identity['image']['id'] == V9.IMAGE_DIGEST
