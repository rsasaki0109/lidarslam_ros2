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

"""Synthetic adversarial tests for the fail-closed evidence composer."""

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
SCRIPT = ROOT / 'scripts' / 'compose_competitive_evidence_bundle.py'
SPEC = importlib.util.spec_from_file_location('competitive_bundle_composer', SCRIPT)
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
SCORER_FINGERPRINT = 'f' * 64
GLOBAL_PAYLOADS = {
    'input': ('inputs/input.json', b'{"input":true}\n'),
    'result': ('results/result.json', b'{"result":true}\n'),
    'config': (
        'config/profile.yaml',
        yaml.safe_dump(PROFILE, sort_keys=False).encode('utf-8')),
    'calibration': ('calibration/calibration.bin', b'calibration\n'),
    'revision': (
        'provenance/revision.json',
        (json.dumps({'systems': REVISION}, sort_keys=True) + '\n').encode()),
    'scorer': ('scorer/scorer.py', b'# opaque scorer bytes\n'),
    'trajectory': ('campaign/trajectory.tum', b'0 0 0 0 0 0 0 1\n'),
    'resource': ('campaign/resource.json', b'{"rss":123}\n'),
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


RUN_PAYLOADS = {
    'trajectory': ('run/ours/seq_a/1/trajectory.tum', b'0 0 0 0 0 0 0 1\n'),
    'map': ('run/ours/seq_a/1/map.ply', b'ply\nsynthetic\n'),
    'resource': ('run/ours/seq_a/1/resource.json', b'{"peak_rss_mb":123}\n'),
    'execution': ('run/ours/seq_a/1/execution.json', _execution_receipt_bytes()),
}


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _descriptor(root: Path, relative: str, payload: bytes) -> dict:
    return {
        'source_root': str(root),
        'source_path': relative,
        'size_bytes': len(payload),
        'sha256': _sha(payload),
    }


def _make_fixture(tmp_path: Path) -> tuple[Path, Path, dict, dict]:
    source = tmp_path / 'source'
    source.mkdir(parents=True)
    global_descriptors = []
    for role, (relative, payload) in GLOBAL_PAYLOADS.items():
        path = source / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
        descriptor = _descriptor(source, relative, payload)
        descriptor['role'] = role
        global_descriptors.append(descriptor)
    run_descriptors = {}
    run_hashes = {}
    for role, (relative, payload) in RUN_PAYLOADS.items():
        path = source / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
        run_descriptors[role] = _descriptor(source, relative, payload)
        run_hashes[role] = _sha(payload)
    execution_document = json.loads(RUN_PAYLOADS['execution'][1].decode())
    execution_receipt_sha = execution_document['execution_receipt_sha256']
    run_descriptors['execution']['receipt_sha256'] = execution_receipt_sha

    run = {
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
        'campaign_id': 'c' * 64,
        'execution_receipt_sha256': execution_receipt_sha,
        'execution_receipt_file_sha256': run_hashes['execution'],
    }
    score_bytes = MODULE._score_bytes('ours', run)
    run['score_artifact_sha256'] = _sha(score_bytes)
    evidence = {'systems': {'ours': {'runs': [run]}}}
    evidence_path = tmp_path / 'scored-evidence.json'
    evidence_path.write_text(
        json.dumps(evidence, sort_keys=True, indent=2) + '\n', encoding='utf-8')
    spec = {
        'schema_version': 1,
        'evidence_path': str(evidence_path),
        'profile': {
            'name': PROFILE['competitive_slam_profile']['name'],
            'canonical_sha256': MODULE.canonical_profile_sha256(PROFILE),
        },
        'scorer': {'fingerprint': SCORER_FINGERPRINT},
        'revision': {'systems': dict(REVISION)},
        'artifacts': global_descriptors,
        'run_artifact_bindings': [{
            'system': 'ours',
            'dataset': 'seq_a',
            'run_index': 1,
            **run_descriptors,
        }],
    }
    spec_path = tmp_path / 'composition-spec.json'
    spec_path.write_text(
        json.dumps(spec, sort_keys=True, indent=2) + '\n', encoding='utf-8')
    return spec_path, evidence_path, spec, evidence


def _files(root: Path) -> dict[str, bytes]:
    return {
        path.relative_to(root).as_posix(): path.read_bytes()
        for path in root.rglob('*') if path.is_file()
    }


def test_composer_seals_deterministic_not_ready_candidate(tmp_path):
    spec_path, _, _, _ = _make_fixture(tmp_path)
    output = tmp_path / 'bundle'
    receipt = MODULE.compose_bundle(spec_path, output)
    assert receipt['status'] == 'NOT_READY'
    assert receipt['pass'] is False
    assert receipt['claim_eligible'] is False
    assert output.is_dir()
    manifest = json.loads((output / MODULE.MANIFEST_FILENAME).read_text())
    assert manifest['claim_status'] == 'NOT_READY'
    assert receipt['schema_validation']['status'] == 'NOT_READY'
    schema_candidate = dict(manifest)
    schema_candidate['claim_status'] = 'claim_eligible'
    assert MODULE._schema_status(schema_candidate)['status'] == 'PASS'
    binding = manifest['run_artifact_bindings'][0]
    assert binding['system'] == 'ours'
    assert binding['score']['path'].endswith('/score.json')
    score_sha = _sha((output / binding['score']['path']).read_bytes())
    assert binding['score']['sha256'] == score_sha
    for path in output.rglob('*'):
        if path.is_file():
            assert not path.is_symlink()
            assert path.stat().st_nlink == 1
            assert path.stat().st_mode & 0o777 == 0o444

    second = tmp_path / 'bundle-second'
    second_receipt = MODULE.compose_bundle(spec_path, second)
    assert second_receipt['manifest_sha256'] == receipt['manifest_sha256']
    assert _files(output) == _files(second)


def test_composer_direct_cli_never_requires_pythonpath(tmp_path):
    spec_path, _, _, _ = _make_fixture(tmp_path)
    output = tmp_path / 'cli-bundle'
    receipt_path = tmp_path / 'cli-receipt.json'
    environment = os.environ.copy()
    environment.pop('PYTHONPATH', None)
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--spec', str(spec_path),
         '--output', str(output), '--receipt', str(receipt_path)],
        env=environment, capture_output=True, text=True, check=False)
    assert result.returncode == 1
    assert json.loads(result.stdout)['status'] == 'NOT_READY'
    assert output.is_dir()
    assert json.loads(receipt_path.read_text())['status'] == 'NOT_READY'
    assert receipt_path.stat().st_mode & 0o777 == 0o444


def test_composer_refuses_existing_output_and_invoking_scorer(tmp_path):
    spec_path, _, spec, _ = _make_fixture(tmp_path)
    output = tmp_path / 'existing'
    output.mkdir()
    (output / 'sentinel').write_bytes(b'preserve')
    with pytest.raises(MODULE.CompositionError, match='already exists'):
        MODULE.compose_bundle(spec_path, output)
    assert (output / 'sentinel').read_bytes() == b'preserve'

    spec['invoke_scorer'] = True
    spec_path.write_text(json.dumps(spec) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.CompositionError, match='never invokes'):
        MODULE.compose_bundle(spec_path, tmp_path / 'scorer-attempt')


def test_claim_failure_source_is_canonical_authorization_log(tmp_path):
    source = tmp_path / 'failure-source'
    source.mkdir()
    failure_log = {
        'complete': True,
        'events': [{'event_id': 'failure-1', 'reason': 'synthetic'}],
    }
    authorization = {'authorization': {'failure_log': failure_log}}
    path = source / 'failure.json'
    valid = MODULE._canonical_failure_artifact_bytes(failure_log)
    path.write_bytes(valid)
    descriptor = _descriptor(source, 'failure.json', valid)
    MODULE._validate_failure_source(descriptor, authorization)
    invalid = [
        b'not-json\n',
        valid.replace(b'"failure_log":', b'"extra":true,"failure_log":'),
        MODULE._canonical_failure_artifact_bytes({
            'complete': True, 'events': [],
        }),
    ]
    for candidate in invalid:
        path.write_bytes(candidate)
        descriptor = _descriptor(source, 'failure.json', candidate)
        with pytest.raises(MODULE.CompositionError):
            MODULE._validate_failure_source(descriptor, authorization)


def test_composer_claim_request_without_authorization_stays_not_ready(tmp_path):
    spec_path, _, _, _ = _make_fixture(tmp_path)
    receipt = MODULE.compose_bundle(
        spec_path, tmp_path / 'claim-request', allow_claim=True)
    assert receipt['status'] == 'NOT_READY'
    assert receipt['claim_eligible'] is False
    assert receipt['authorization']['pass'] is False
    assert receipt['authorization']['status'] == 'NOT_READY'

    spec_path, _, spec, _ = _make_fixture(tmp_path / 'report-only')
    spec['authorization'] = {}
    spec['authorization_policy'] = {'required': False}
    spec_path.write_text(json.dumps(spec) + '\n', encoding='utf-8')
    report_only = MODULE.compose_bundle(
        spec_path, tmp_path / 'report-only-bundle', allow_claim=True)
    assert report_only['status'] == 'NOT_READY'
    assert report_only['claim_eligible'] is False
    assert report_only['authorization']['status'] == 'NOT_REQUIRED'


def test_composer_rejects_revision_metadata_drift(tmp_path):
    spec_path, _, spec, _ = _make_fixture(tmp_path)
    spec['revision']['systems']['ours'] = 'b' * 40
    spec_path.write_text(json.dumps(spec) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.CompositionError, match='revision metadata'):
        MODULE.compose_bundle(spec_path, tmp_path / 'revision-drift')


def test_composer_rejects_one_physical_file_aliasing_multiple_roles(tmp_path):
    spec_path, _, spec, _ = _make_fixture(tmp_path)
    source = spec['artifacts'][0]
    result = next(item for item in spec['artifacts'] if item['role'] == 'result')
    result.update({
        'source_root': source['source_root'],
        'source_path': source['source_path'],
        'size_bytes': source['size_bytes'],
        'sha256': source['sha256'],
    })
    spec_path.write_text(json.dumps(spec) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.CompositionError, match='source file alias'):
        MODULE.compose_bundle(spec_path, tmp_path / 'aliased')


def test_composer_rejects_score_drift_and_incomplete_identity(tmp_path):
    spec_path, evidence_path, _, evidence = _make_fixture(tmp_path)
    evidence['systems']['ours']['runs'][0]['trajectory']['ape_rmse_m'] = 9.0
    evidence_path.write_text(json.dumps(evidence) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.CompositionError, match='scored evidence digest drift'):
        MODULE.compose_bundle(spec_path, tmp_path / 'metric-drift')

    spec_path, _, spec, _ = _make_fixture(tmp_path / 'incomplete')
    spec['run_artifact_bindings'] = []
    spec_path.write_text(json.dumps(spec) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.CompositionError, match='coverage is incomplete|schema invalid'):
        MODULE.compose_bundle(spec_path, tmp_path / 'incomplete-output')


def test_composer_rejects_traversal_symlink_and_hardlink_sources(tmp_path):
    spec_path, _, spec, _ = _make_fixture(tmp_path)
    spec['artifacts'][0]['source_path'] = '../outside'
    spec_path.write_text(json.dumps(spec) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.CompositionError, match='traversal|schema invalid'):
        MODULE.compose_bundle(spec_path, tmp_path / 'traversal')

    spec_path, _, spec, _ = _make_fixture(tmp_path / 'symlink')
    source = Path(spec['artifacts'][0]['source_root'])
    original = source / spec['artifacts'][0]['source_path']
    alias = source / 'inputs' / 'link.json'
    alias.symlink_to(original)
    spec['artifacts'][0]['source_path'] = 'inputs/link.json'
    spec_path.write_text(json.dumps(spec) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.CompositionError, match='symlink'):
        MODULE.compose_bundle(spec_path, tmp_path / 'symlink-output')

    spec_path, _, spec, _ = _make_fixture(tmp_path / 'hardlink')
    source = Path(spec['artifacts'][0]['source_root'])
    original = source / spec['artifacts'][0]['source_path']
    alias = source / 'inputs' / 'hardlink.json'
    os.link(original, alias)
    spec['artifacts'][0]['source_path'] = 'inputs/hardlink.json'
    spec_path.write_text(json.dumps(spec) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.CompositionError, match='single-link'):
        MODULE.compose_bundle(spec_path, tmp_path / 'hardlink-output')


def test_composer_rejects_source_hash_mutation_and_functional_resource(tmp_path):
    spec_path, _, spec, _ = _make_fixture(tmp_path)
    source = Path(spec['artifacts'][0]['source_root'])
    path = source / spec['artifacts'][0]['source_path']
    path.write_bytes(b'mutated\n')
    with pytest.raises(MODULE.CompositionError, match='receipt drift'):
        MODULE.compose_bundle(spec_path, tmp_path / 'hash-drift')
    assert not (tmp_path / 'hash-drift').exists()
    failed_roots = list(tmp_path.glob('hash-drift.staging.*'))
    assert len(failed_roots) == 1
    failure_receipt = failed_roots[0] / 'composition.failure.json'
    assert json.loads(failure_receipt.read_text())['status'] == 'FAIL_CLOSED'
    assert failure_receipt.stat().st_mode & 0o777 == 0o444

    spec_path, evidence_path, _, evidence = _make_fixture(tmp_path / 'functional')
    evidence['systems']['ours']['runs'][0]['resource_evidence']['functional_only'] = True
    evidence_path.write_text(json.dumps(evidence) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.CompositionError, match='functional-only'):
        MODULE.compose_bundle(spec_path, tmp_path / 'functional-output')
