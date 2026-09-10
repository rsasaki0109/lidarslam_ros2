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

"""Synthetic tests for the post-score-to-bundle handoff boundary."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

from lidarslam_benchmark_tools.competitive_execution_attempt_receipt import (
    seal_receipt)

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'prepare_competitive_evidence_bundle_handoff.py'
SPEC = importlib.util.spec_from_file_location('competitive_bundle_handoff', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


PROFILE = {
    'competitive_slam_profile': {
        'name': 'synthetic-competitive-profile',
        'evidence_gate_v2': {},
    }
}
REVISION = {'ours': 'a' * 40}
SCORER = 'f' * 64
GLOBAL = {
    'input': ('inputs/input.json', b'{"input":true}\n'),
    'result': ('results/result.json', b'{"result":true}\n'),
    'config': ('config/profile.yaml', yaml.safe_dump(
        PROFILE, sort_keys=False).encode()),
    'calibration': ('calibration/calibration.bin', b'calibration\n'),
    'revision': ('provenance/revision.json', (
        json.dumps({'systems': REVISION}, sort_keys=True) + '\n').encode()),
    'scorer': ('scorer/scorer.py', b'# scorer receipt is opaque\n'),
    'trajectory': ('campaign/trajectory.tum', b'0 0 0 0 0 0 0 1\n'),
    'resource': ('campaign/resource.json', b'{"resource":"global"}\n'),
    'map_metric': ('metrics/map.json', b'{"map":"global"}\n'),
    'failure': ('failures/failure.json', b'{"status":"none"}\n'),
}
RUN_FILES = {
    'trajectory': ('runs/ours/seq_a/1/trajectory.tum', b'0 0 0 0 0 0 0 1\n'),
    'map': ('runs/ours/seq_a/1/map.ply', b'ply\nsynthetic\n'),
    'resource': ('runs/ours/seq_a/1/resource.json', b'{"rss":123}\n'),
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


RUN_FILES['execution'] = (
    'runs/ours/seq_a/1/execution.json', _execution_receipt_bytes())


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _source(root: Path, relative: str, data: bytes) -> dict:
    return {
        'source_root': str(root),
        'source_path': relative,
        'size_bytes': len(data),
        'sha256': _sha(data),
    }


def _fixture(tmp_path: Path) -> tuple[Path, Path, dict, dict]:
    source_root = tmp_path / 'sources'
    source_root.mkdir(parents=True)
    global_sources = []
    for role, (relative, data) in GLOBAL.items():
        path = source_root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        global_sources.append({
            'role': role, **_source(source_root, relative, data)})
    run_sources = {}
    run_hashes = {}
    for role, (relative, data) in RUN_FILES.items():
        path = source_root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        run_sources[role] = _source(source_root, relative, data)
        run_hashes[role] = _sha(data)
    execution_document = json.loads(RUN_FILES['execution'][1].decode())
    run_sources['execution']['receipt_sha256'] = (
        execution_document['execution_receipt_sha256'])
    evidence = {
        'systems': {'ours': {'runs': [{
            'campaign_id': 'c' * 64,
            'dataset': 'seq_a',
            'run_index': 1,
            'complete': True,
            'process_exit_status': 0,
            'trajectory_complete': True,
            'sequence_failure': False,
            'catastrophic_failure': False,
            'verified_false_loops': 0,
            'trajectory': {'ape_rmse_m': 0.12},
            'runtime': {
                'processing_rtf': 0.4,
                'peak_rss_mb': 123.0,
                'phase_contract_version': 'synthetic-v1',
            },
            'map': {
                'plane_thickness_mean_m': 0.02,
                'plane_thickness_p95_m': 0.04,
                'planar_coverage': 0.8,
            },
            'artifacts': {
                'trajectory_sha256': run_hashes['trajectory'],
                'map_sha256': run_hashes['map'],
            },
            'resource_evidence': {
                'receipt_sha256': run_hashes['resource'],
                'timing_authority': 'AUTHORITATIVE',
                'functional_only': False,
                'performance_gate_eligible': True,
            },
            'execution_receipt_sha256': execution_document[
                'execution_receipt_sha256'],
            'execution_receipt_file_sha256': run_hashes['execution'],
        }]}}
    }
    evidence_path = tmp_path / 'scored-evidence.json'
    evidence_path.write_text(json.dumps(evidence) + '\n', encoding='utf-8')
    evidence_sha = _sha(evidence_path.read_bytes())
    authorization_path = tmp_path / 'post-score-authorization.json'
    authorization_path.write_text(json.dumps({
        'status': 'PASS', 'scorer_fingerprint': SCORER,
        'input_evidence_sha256': evidence_sha,
    }) + '\n', encoding='utf-8')
    request = {
        'schema_version': 1,
        'evidence_path': str(evidence_path),
        'profile': {
            'name': PROFILE['competitive_slam_profile']['name'],
            'canonical_sha256': MODULE.composer.canonical_profile_sha256(PROFILE),
        },
        'scorer': {'fingerprint': SCORER},
        'revision': {'systems': dict(REVISION)},
        'expected_runs': [
            {'system': 'ours', 'dataset': 'seq_a', 'run_index': 1}],
        'artifacts': global_sources,
        'run_artifact_sources': [{
            'system': 'ours', 'dataset': 'seq_a', 'run_index': 1,
            **run_sources,
        }],
        'scoring_authorization': {
            'receipt_path': str(authorization_path),
            'status': 'PASS',
            'scorer_fingerprint': SCORER,
            'input_evidence_sha256': evidence_sha,
        },
    }
    request_path = tmp_path / 'handoff-request.json'
    request_path.write_text(json.dumps(request, indent=2) + '\n', encoding='utf-8')
    return request_path, evidence_path, request, evidence


def test_handoff_computes_score_digest_and_feeds_bundle_composer(tmp_path):
    request_path, _, _, _ = _fixture(tmp_path)
    handoff_root = tmp_path / 'handoff'
    receipt = MODULE.prepare_handoff(request_path, handoff_root)
    assert receipt['status'] == 'PASS'
    assert receipt['claim_eligible'] is False
    assert receipt['gt_content_copied'] is False
    assert receipt['scorer_invoked'] is False
    evidence = json.loads((handoff_root / 'scored-evidence.json').read_text())
    run = evidence['systems']['ours']['runs'][0]
    score_bytes = MODULE.composer._score_bytes('ours', run)
    assert run['score_artifact_sha256'] == _sha(score_bytes)
    spec = json.loads((handoff_root / 'composition-spec.json').read_text())
    assert spec['post_score_authorization']['status'] == 'PASS'
    assert spec['post_score_authorization']['input_evidence_sha256'] == (
        receipt['input_evidence_sha256'])
    assert spec['post_score_authorization']['scored_evidence_sha256'] == _sha(
        (handoff_root / 'scored-evidence.json').read_bytes())
    assert spec['run_artifact_bindings'][0]['trajectory']['size_bytes'] > 0
    assert 'ground_truth' not in (handoff_root / 'scored-evidence.json').read_text()

    bundle = tmp_path / 'bundle'
    composed = MODULE.composer.compose_bundle(
        handoff_root / 'composition-spec.json', bundle)
    assert composed['status'] == 'NOT_READY'
    manifest = json.loads((bundle / 'bundle_manifest.json').read_text())
    assert manifest['score_handoff']['receipt_sha256'] == (
        spec['post_score_authorization']['receipt_sha256'])
    assert manifest['run_artifact_bindings'][0]['score']['sha256'] == (
        run['score_artifact_sha256'])

    tampered_spec = json.loads(
        (handoff_root / 'composition-spec.json').read_text())
    tampered_spec['post_score_authorization']['scored_evidence_sha256'] = '0' * 64
    tampered_spec_path = tmp_path / 'tampered-spec.json'
    tampered_spec_path.write_text(json.dumps(tampered_spec) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.composer.CompositionError, match='not bound'):
        MODULE.composer.compose_bundle(
            tampered_spec_path, tmp_path / 'tampered-bundle')


def test_handoff_rejects_legacy_result_without_run_evidence(tmp_path):
    request_path, evidence_path, request, _ = _fixture(tmp_path)
    evidence_path.write_text(json.dumps({
        'schema_version': 1,
        'system': 'ours',
        'sequence': 'seq_a',
        'trajectory': {'ape_rmse_median_m': 0.1},
    }) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.HandoffError, match='systems|score-complete'):
        MODULE.prepare_handoff(request_path, tmp_path / 'legacy')


def test_handoff_rejects_missing_run_binding_and_gt_receipt_metadata(tmp_path):
    request_path, _, request, _ = _fixture(tmp_path)
    request['run_artifact_sources'] = []
    request_path.write_text(json.dumps(request) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.HandoffError, match='too short|coverage|invalid'):
        MODULE.prepare_handoff(request_path, tmp_path / 'missing-run')

    request_path, _, request, _ = _fixture(tmp_path / 'coverage')
    request['expected_runs'].append({
        'system': 'fast_livo2', 'dataset': 'seq_a', 'run_index': 1})
    request_path.write_text(json.dumps(request) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.HandoffError, match='expected run coverage'):
        MODULE.prepare_handoff(request_path, tmp_path / 'coverage-output')

    request_path, _, request, _ = _fixture(tmp_path / 'gt-receipt')
    auth_path = Path(request['scoring_authorization']['receipt_path'])
    auth_path.write_text(json.dumps({
        'status': 'PASS', 'scorer_fingerprint': SCORER,
        'input_evidence_sha256': _sha(
            Path(request['evidence_path']).read_bytes()),
        'ground_truth_path': '/opaque/ground_truth/reference.tum',
    }) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.HandoffError, match='forbidden GT metadata'):
        MODULE.prepare_handoff(request_path, tmp_path / 'gt-receipt-output')


def test_handoff_rejects_source_mutation_and_evidence_gt_path(tmp_path):
    request_path, evidence_path, request, evidence = _fixture(tmp_path)
    source_path = Path(request['artifacts'][0]['source_root']) / \
        request['artifacts'][0]['source_path']
    source_path.write_bytes(b'mutated\n')
    with pytest.raises(MODULE.HandoffError, match='receipt|size|sha|drift'):
        MODULE.prepare_handoff(request_path, tmp_path / 'source-drift')

    request_path, evidence_path, _, evidence = _fixture(tmp_path / 'gt-evidence')
    evidence['systems']['ours']['provenance'] = {
        'reference_path': '/opaque/ground_truth/reference.tum'}
    evidence_path.write_text(json.dumps(evidence) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.HandoffError, match='forbidden GT'):
        MODULE.prepare_handoff(request_path, tmp_path / 'gt-evidence-output')


def test_handoff_rejects_post_score_metric_mutation(tmp_path):
    request_path, evidence_path, _, evidence = _fixture(tmp_path)
    evidence['systems']['ours']['runs'][0]['trajectory']['ape_rmse_m'] = 0.99
    evidence_path.write_text(json.dumps(evidence) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.HandoffError, match='evidence SHA drift'):
        MODULE.prepare_handoff(request_path, tmp_path / 'metric-drift')


def test_handoff_cli_clean_source_checkout_without_pythonpath(tmp_path):
    request_path, _, _, _ = _fixture(tmp_path)
    output = tmp_path / 'cli-handoff'
    environment = os.environ.copy()
    environment.pop('PYTHONPATH', None)
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--request', str(request_path),
         '--output', str(output)],
        env=environment, capture_output=True, text=True, check=False)
    assert result.returncode == 0
    assert json.loads(result.stdout)['status'] == 'PASS'
    assert output.is_dir()
