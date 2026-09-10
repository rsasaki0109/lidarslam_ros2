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

"""Adversarial synthetic-fixture tests for the final evidence bundle gate."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path

from lidarslam_benchmark_tools.competitive_execution_attempt_receipt import (
    ExecutionReceiptError, receipt_sha256, seal_receipt, validate_receipt)

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'verify_competitive_evidence_bundle.py'
SPEC = importlib.util.spec_from_file_location('competitive_bundle_verifier', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


PROFILE = {
    'competitive_slam_profile': {
        'name': 'synthetic-competitive-profile',
        'evidence_gate_v2': {},
    }
}
EXPECTED_REVISIONS = {'ours': 'a' * 40, 'glim': 'b' * 40}
SCORER_FINGERPRINT = 'f' * 64
ROLE_FILES = {
    'input': ('inputs/input.json', b'{"input":true}\n'),
    'result': ('results/result.json', b'{"result":true}\n'),
    'config': (
        'config/profile.yaml',
        b'competitive_slam_profile:\n'
        b'  name: synthetic-competitive-profile\n'
        b'  evidence_gate_v2: {}\n'),
    'calibration': ('calibration/calibration.bin', b'calibration\n'),
    'revision': (
        'provenance/revision.json',
        b'{"systems":{"ours":"' + b'a' * 40 + b'","glim":"' +
        b'b' * 40 + b'"}}\n'),
    'scorer': ('scorer/scorer.py', b'# opaque scorer bytes\n'),
    'trajectory': ('trajectory/trajectory.tum', b'0 0 0 0 0 0 0 1\n'),
    'resource': ('resources/resource.json', b'{"cpu":"synthetic"}\n'),
    'map_metric': ('metrics/map.json', b'{"status":"opaque"}\n'),
    'failure': ('failures/failure.json', b'{"status":"none"}\n'),
}


def _execution_receipt_bytes() -> bytes:
    value = seal_receipt({
        'campaign_id': 'c' * 64,
        'schedule': {'system': 'ours', 'sequence': 'seq_a', 'repetition': 1},
        'identity': {
            'profile_canonical_sha256': 'a' * 64,
            'execution_receipt_file_sha256': 'b' * 64,
            'selection_receipt_file_sha256': 'd' * 64,
            'image_digest': 'sha256:' + 'e' * 64,
        },
        'argv': ['synthetic-attempt'],
        'mounts': [],
        'execution': {'exit_status': 0, 'timed_out': False, 'signal': None},
        'completion': {'complete': True},
        'artifact_hashes': {'traj_raw.tum': 'a' * 64},
        'output_tree_sha256': 'f' * 64,
        'gt_blind_proof': {
            'ground_truth_reachable': False,
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
            'mount_sources': [],
        },
    })
    return (json.dumps(value, sort_keys=True, separators=(',', ':')) + '\n').encode()


RUN_ARTIFACT_FILES = {
    'trajectory': ('runs/ours/seq_a/1/trajectory.tum', b'0 0 0 0 0 0 0 1\n'),
    'map': ('runs/ours/seq_a/1/map.json', b'{"map":"synthetic"}\n'),
    'resource': ('runs/ours/seq_a/1/resource.json', b'{"rss":123}\n'),
    'execution': ('runs/ours/seq_a/1/execution.json', _execution_receipt_bytes()),
    'score': (
        'runs/ours/seq_a/1/score.json',
        b'{"schema":"competitive_run_score_v1","synthetic":true}\n'),
}


def _sha(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _write_manifest(root: Path, manifest: dict) -> None:
    manifest['manifest_sha256'] = MODULE.canonical_manifest_sha256(manifest)
    sidecar = (
        f"{manifest['manifest_sha256']}  {manifest['manifest_path']}\n"
    ).encode()
    manifest['manifest_sidecar_sha256'] = _sha(sidecar)
    manifest_path = root / manifest['manifest_path']
    manifest_path.write_bytes(MODULE.canonical_manifest_bytes(manifest))
    (root / manifest['manifest_sidecar_path']).write_bytes(sidecar)


def _bundle(tmp_path: Path) -> tuple[Path, dict]:
    root = tmp_path / 'bundle'
    root.mkdir(parents=True)
    artifacts = []
    for role, (relative, payload) in ROLE_FILES.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
        digest = _sha(payload)
        sidecar_path = relative + '.sha256'
        sidecar_bytes = f'{digest}  {relative}\n'.encode()
        (root / sidecar_path).write_bytes(sidecar_bytes)
        artifacts.append({
            'path': relative,
            'role': role,
            'size_bytes': len(payload),
            'sha256': digest,
            'sidecar_path': sidecar_path,
            'sidecar_sha256': _sha(sidecar_bytes),
        })
    profile_relative = ROLE_FILES['config'][0]
    profile_bytes = (root / profile_relative).read_bytes()
    scorer_relative = ROLE_FILES['scorer'][0]
    scorer_bytes = (root / scorer_relative).read_bytes()
    revision_relative = ROLE_FILES['revision'][0]
    revision_bytes = (root / revision_relative).read_bytes()
    run_refs = {}
    for role, (relative, payload) in RUN_ARTIFACT_FILES.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
        digest = _sha(payload)
        sidecar_path = relative + '.sha256'
        sidecar_bytes = f'{digest}  {relative}\n'.encode()
        (root / sidecar_path).write_bytes(sidecar_bytes)
        run_refs[role] = {
            'path': relative,
            'size_bytes': len(payload),
            'sha256': digest,
            'sidecar_path': sidecar_path,
            'sidecar_sha256': _sha(sidecar_bytes),
        }
    execution_document = json.loads(
        RUN_ARTIFACT_FILES['execution'][1].decode())
    run_refs['execution']['receipt_sha256'] = (
        execution_document['execution_receipt_sha256'])
    manifest = {
        'schema_version': 1,
        'manifest_kind': MODULE.MANIFEST_KIND,
        'manifest_path': MODULE.MANIFEST_FILENAME,
        'canonical_root': '.',
        'claim_status': 'claim_eligible',
        'manifest_sidecar_path': MODULE.MANIFEST_SIDECAR_FILENAME,
        'profile': {
            'path': profile_relative,
            'sha256': _sha(profile_bytes),
            'sha256_kind': MODULE.FILE_HASH_KIND,
            'canonical_sha256': MODULE.canonical_profile_sha256(PROFILE),
            'canonical_sha256_kind': MODULE.PROFILE_CANONICAL_HASH_KIND,
            'name': PROFILE['competitive_slam_profile']['name'],
        },
        'scorer': {
            'path': scorer_relative,
            'sha256': _sha(scorer_bytes),
            'fingerprint': SCORER_FINGERPRINT,
        },
        'revision': {
            'path': revision_relative,
            'sha256': _sha(revision_bytes),
            'systems': copy.deepcopy(EXPECTED_REVISIONS),
        },
        'artifacts': artifacts,
        'run_artifact_bindings': [{
            'system': 'ours',
            'dataset': 'seq_a',
            'run_index': 1,
            **run_refs,
        }],
    }
    _write_manifest(root, manifest)
    return root, manifest


def _verify(root: Path):
    expected = [{
        'system': 'ours',
        'dataset': 'seq_a',
        'run_index': 1,
        'trajectory_sha256': _sha(RUN_ARTIFACT_FILES['trajectory'][1]),
        'map_sha256': _sha(RUN_ARTIFACT_FILES['map'][1]),
        'resource_sha256': _sha(RUN_ARTIFACT_FILES['resource'][1]),
        'score_sha256': _sha(RUN_ARTIFACT_FILES['score'][1]),
        'execution_receipt_sha256': json.loads(
            RUN_ARTIFACT_FILES['execution'][1].decode())[
                'execution_receipt_sha256'],
        'execution_receipt_file_sha256': _sha(RUN_ARTIFACT_FILES['execution'][1]),
        'campaign_id': 'c' * 64,
    }]
    return MODULE.verify_evidence_bundle(
        root,
        profile=PROFILE,
        expected_scorer_fingerprint=SCORER_FINGERPRINT,
        expected_revision_by_system=EXPECTED_REVISIONS,
        expected_run_artifacts=expected,
    )


def test_complete_bundle_reopens_all_required_roles(tmp_path):
    root, _ = _bundle(tmp_path)
    result = _verify(root)
    assert result['status'] == 'PASS'
    assert result['pass'] is True
    assert result['claim_eligible'] is True
    assert result['checks']['roles']['pass'] is True
    assert result['checks']['artifacts']['required_reopened'] == 10
    assert result['checks']['run_artifact_bindings']['reopened'] == 5
    assert {item['role'] for item in result['artifacts']} == set(MODULE.REQUIRED_ROLES)


def test_claim_bundle_requires_per_run_artifact_index(tmp_path):
    root, manifest = _bundle(tmp_path)
    manifest.pop('run_artifact_bindings')
    _write_manifest(root, manifest)
    result = _verify(root)
    assert result['pass'] is False
    assert result['checks']['run_artifact_bindings']['pass'] is False
    assert any('run_artifact_bindings must be a non-empty list' in item
               for item in result['errors'])


def test_per_run_artifact_bytes_and_evaluator_hashes_are_bound(tmp_path):
    root, manifest = _bundle(tmp_path)
    manifest['run_artifact_bindings'][0]['trajectory']['sha256'] = '0' * 64
    _write_manifest(root, manifest)
    result = _verify(root)
    assert result['pass'] is False
    assert any(
        'run artifact ours:seq_a:1/trajectory SHA does not match evaluator '
        'run identity' in item for item in result['errors'])

    root2, _ = _bundle(tmp_path / 'second')
    expected = [{
        'system': 'ours', 'dataset': 'seq_a', 'run_index': 1,
        'trajectory_sha256': _sha(RUN_ARTIFACT_FILES['trajectory'][1]),
        'map_sha256': '1' * 64,
        'resource_sha256': _sha(RUN_ARTIFACT_FILES['resource'][1]),
        'score_sha256': _sha(RUN_ARTIFACT_FILES['score'][1]),
        'execution_receipt_sha256': json.loads(
            RUN_ARTIFACT_FILES['execution'][1].decode())[
                'execution_receipt_sha256'],
        'execution_receipt_file_sha256': _sha(RUN_ARTIFACT_FILES['execution'][1]),
        'campaign_id': 'c' * 64,
    }]
    result2 = MODULE.verify_evidence_bundle(
        root2, profile=PROFILE,
        expected_scorer_fingerprint=SCORER_FINGERPRINT,
        expected_revision_by_system=EXPECTED_REVISIONS,
        expected_run_artifacts=expected)
    assert result2['pass'] is False
    assert any('map SHA does not match evaluator run identity' in item
               for item in result2['errors'])

    root3, _ = _bundle(tmp_path / 'third')
    (root3 / RUN_ARTIFACT_FILES['map'][0]).write_bytes(b'tampered-map\n')
    result3 = _verify(root3)
    assert result3['pass'] is False
    assert any(
        'run artifact ours:seq_a:1/map size mismatch' in item or
        'run artifact ours:seq_a:1/map SHA-256 mismatch' in item
        for item in result3['errors'])


def test_execution_canonical_and_file_hashes_are_distinct_bindings(tmp_path):
    root, manifest = _bundle(tmp_path)
    manifest['run_artifact_bindings'][0]['execution']['receipt_sha256'] = '0' * 64
    _write_manifest(root, manifest)
    result = _verify(root)
    assert result['pass'] is False
    assert any('execution receipt canonical SHA' in item
               for item in result['errors'])


def test_per_run_coverage_is_unique_and_exact(tmp_path):
    root, manifest = _bundle(tmp_path)
    manifest['run_artifact_bindings'].append(
        copy.deepcopy(manifest['run_artifact_bindings'][0]))
    _write_manifest(root, manifest)
    result = _verify(root)
    assert result['pass'] is False
    assert any('duplicate run identity' in item for item in result['errors'])

    root2, _ = _bundle(tmp_path / 'second')
    expected = [{
        'system': 'ours', 'dataset': 'seq_a', 'run_index': 1,
        'trajectory_sha256': _sha(RUN_ARTIFACT_FILES['trajectory'][1]),
        'map_sha256': _sha(RUN_ARTIFACT_FILES['map'][1]),
        'resource_sha256': _sha(RUN_ARTIFACT_FILES['resource'][1]),
        'score_sha256': _sha(RUN_ARTIFACT_FILES['score'][1]),
        'execution_receipt_sha256': json.loads(
            RUN_ARTIFACT_FILES['execution'][1].decode())[
                'execution_receipt_sha256'],
        'execution_receipt_file_sha256': _sha(RUN_ARTIFACT_FILES['execution'][1]),
        'campaign_id': 'c' * 64,
    }, {
        'system': 'ours', 'dataset': 'seq_a', 'run_index': 2,
        'trajectory_sha256': _sha(RUN_ARTIFACT_FILES['trajectory'][1]),
        'map_sha256': _sha(RUN_ARTIFACT_FILES['map'][1]),
        'resource_sha256': _sha(RUN_ARTIFACT_FILES['resource'][1]),
        'score_sha256': _sha(RUN_ARTIFACT_FILES['score'][1]),
        'execution_receipt_sha256': json.loads(
            RUN_ARTIFACT_FILES['execution'][1].decode())[
                'execution_receipt_sha256'],
        'execution_receipt_file_sha256': _sha(RUN_ARTIFACT_FILES['execution'][1]),
        'campaign_id': 'c' * 64,
    }]
    result2 = MODULE.verify_evidence_bundle(
        root2, profile=PROFILE,
        expected_scorer_fingerprint=SCORER_FINGERPRINT,
        expected_revision_by_system=EXPECTED_REVISIONS,
        expected_run_artifacts=expected)
    assert result2['pass'] is False
    assert any('coverage does not exactly match scored runs' in item
               for item in result2['errors'])


def test_missing_required_role_fails_closed(tmp_path):
    root, manifest = _bundle(tmp_path)
    manifest['artifacts'] = [item for item in manifest['artifacts']
                             if item['role'] != 'failure']
    _write_manifest(root, manifest)
    result = _verify(root)
    assert result['pass'] is False
    assert result['checks']['roles']['pass'] is False
    assert any('required artifact roles are missing' in item for item in result['errors'])


def test_extra_undeclared_file_fails_closed(tmp_path):
    root, _ = _bundle(tmp_path)
    (root / 'results' / 'extra-alias.bin').write_bytes(b'extra')
    result = _verify(root)
    assert result['pass'] is False
    assert result['checks']['root_inventory']['pass'] is False
    assert any('undeclared regular files' in item for item in result['errors'])


def test_duplicate_and_casefold_collision_aliases_fail_closed(tmp_path):
    root, manifest = _bundle(tmp_path)
    duplicate = copy.deepcopy(manifest['artifacts'][0])
    duplicate['role'] = 'failure'
    manifest['artifacts'].append(duplicate)
    _write_manifest(root, manifest)
    result = _verify(root)
    assert result['pass'] is False
    assert any('collides with another normalized path' in item
               for item in result['errors'])

    root2, manifest2 = _bundle(tmp_path / 'casefold')
    alias = copy.deepcopy(manifest2['artifacts'][0])
    alias['role'] = 'failure'
    alias['path'] = 'inputs/INPUT.JSON'
    alias['sidecar_path'] = 'inputs/INPUT.JSON.sha256'
    manifest2['artifacts'].append(alias)
    _write_manifest(root2, manifest2)
    result2 = _verify(root2)
    assert result2['pass'] is False
    assert any('collides with another normalized path' in item
               for item in result2['errors'])


@pytest.mark.parametrize('bad_path', ['../outside.json', '/tmp/outside.json',
                                      './inputs/input.json', 'inputs//input.json'])
def test_traversal_absolute_and_alias_paths_fail_before_reopen(tmp_path, bad_path):
    root, manifest = _bundle(tmp_path)
    manifest['artifacts'][0]['path'] = bad_path
    _write_manifest(root, manifest)
    result = _verify(root)
    assert result['pass'] is False
    assert any('artifacts[0].path' in item for item in result['errors'])


def test_symlinked_artifact_and_symlinked_component_fail_closed(tmp_path):
    root, manifest = _bundle(tmp_path)
    target = tmp_path / 'outside.bin'
    target.write_bytes(b'outside')
    input_path = root / ROLE_FILES['input'][0]
    input_path.unlink()
    input_path.symlink_to(target)
    result = _verify(root)
    assert result['pass'] is False
    assert any('symlink' in item for item in result['errors'])

    root2, manifest2 = _bundle(tmp_path / 'second')
    outside_dir = tmp_path / 'outside-dir'
    outside_dir.mkdir()
    link_dir = root2 / 'linked'
    link_dir.symlink_to(outside_dir, target_is_directory=True)
    manifest2['artifacts'][0]['path'] = 'linked/input.json'
    _write_manifest(root2, manifest2)
    result2 = _verify(root2)
    assert result2['pass'] is False
    assert any('symlink' in item for item in result2['errors'])


def test_hardlink_artifact_fails_closed(tmp_path):
    root, _ = _bundle(tmp_path)
    source = tmp_path / 'hardlink-source'
    source.write_bytes(b'hardlinked')
    target = root / ROLE_FILES['resource'][0]
    target.unlink()
    os.link(source, target)
    result = _verify(root)
    assert result['pass'] is False
    assert any('hard-link' in item or 'nlink' in item for item in result['errors'])


def test_size_sha_and_sidecar_mismatch_fail_closed(tmp_path):
    root, manifest = _bundle(tmp_path)
    (root / ROLE_FILES['result'][0]).write_bytes(b'tampered\n')
    result = _verify(root)
    assert result['pass'] is False
    assert any('size mismatch' in item or 'SHA-256 mismatch' in item
               for item in result['errors'])

    root2, _ = _bundle(tmp_path / 'second')
    sidecar = root2 / (ROLE_FILES['result'][0] + '.sha256')
    sidecar.write_text('0' * 64 + '  wrong\n')
    result2 = _verify(root2)
    assert result2['pass'] is False
    assert any('sidecar mismatch' in item for item in result2['errors'])


def test_manifest_must_be_deterministic_and_self_bound(tmp_path):
    root, manifest = _bundle(tmp_path)
    manifest_path = root / MODULE.MANIFEST_FILENAME
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    result = _verify(root)
    assert result['pass'] is False
    assert any('deterministic canonical JSON' in item for item in result['errors'])


def test_profile_scorer_and_revision_drift_fail_closed(tmp_path):
    root, manifest = _bundle(tmp_path)
    manifest['profile']['canonical_sha256'] = '0' * 64
    _write_manifest(root, manifest)
    result = _verify(root)
    assert result['pass'] is False
    assert result['checks']['profile_identity']['pass'] is False

    root2, manifest2 = _bundle(tmp_path / 'second')
    manifest2['scorer']['fingerprint'] = '0' * 64
    manifest2['revision']['systems']['ours'] = 'c' * 40
    _write_manifest(root2, manifest2)
    result2 = _verify(root2)
    assert result2['pass'] is False
    assert result2['checks']['scorer_identity']['pass'] is False
    assert result2['checks']['revision_identity']['pass'] is False


def test_ground_truth_role_is_rejected_before_artifact_open(tmp_path):
    root, manifest = _bundle(tmp_path)
    manifest['artifacts'][0]['role'] = 'ground_truth'
    _write_manifest(root, manifest)
    result = _verify(root)
    assert result['pass'] is False
    assert any('ground-truth artifact/role is forbidden' in item
               for item in result['errors'])


def test_manifest_claim_status_and_root_are_required(tmp_path):
    root, manifest = _bundle(tmp_path)
    manifest['claim_status'] = 'NOT_READY'
    _write_manifest(root, manifest)
    result = _verify(root)
    assert result['pass'] is False
    assert any('claim_status' in item for item in result['errors'])

    root2, _ = _bundle(tmp_path / 'second')
    result2 = _verify(root2 / 'missing-root')
    assert result2['pass'] is False
    assert result2['checks']['manifest']['pass'] is False


def test_required_fresh_holdout_authorization_is_not_ready_without_attestation(tmp_path):
    root, _ = _bundle(tmp_path)
    contract = copy.deepcopy(PROFILE)
    contract['competitive_slam_profile']['evidence_gate_v2'] = {
        'fresh_holdout_authorization': {
            'required': True,
            'status': 'NOT_READY',
            'status_reason': 'independent_external_custodian_attestation_absent',
            'external_attestation': {'required': True},
        }
    }
    result = MODULE.verify_evidence_bundle(
        root, profile=contract,
        expected_scorer_fingerprint=SCORER_FINGERPRINT,
        expected_revision_by_system=EXPECTED_REVISIONS,
        contract=contract['competitive_slam_profile'])
    assert result['pass'] is False
    assert result['status'] == 'NOT_READY'
    assert result['checks']['authorization']['pass'] is False
    assert any('authorization mapping is missing' in item
               for item in result['errors'])


def test_execution_receipt_failed_completion_is_not_claim_bound():
    document = json.loads(RUN_ARTIFACT_FILES['execution'][1].decode())
    document['completion']['complete'] = False
    document['execution_receipt_sha256'] = receipt_sha256(document)
    with pytest.raises(ExecutionReceiptError, match='incomplete'):
        validate_receipt(document, expected_system='ours',
                         expected_dataset='seq_a', expected_run_index=1,
                         require_success=True)
