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
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Synthetic tests for the fail-closed competitive claim publication boundary."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'publish_competitive_claim.py'
SPEC = importlib.util.spec_from_file_location('competitive_claim_publication', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _write(path: Path, value: object) -> str:
    data = (value if isinstance(value, bytes) else
            value.encode() if isinstance(value, str) else
            (json.dumps(value, sort_keys=True, indent=2) + '\n').encode())
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return _sha(data)


def _scope() -> dict:
    value = {
        'systems': ['glim', 'ours'],
        'datasets': ['holdout_a'],
        'revisions': {'glim': 'b' * 40, 'ours': 'a' * 40},
        'run_count': 3,
        'metrics': ['APE', 'RSS', 'RTF', 'completion', 'map'],
        'thresholds': {
            'minimum_aggregate_ape_improvement_percent': 10.0,
            'maximum_primary_regression_percent': 2.0,
            'maximum_peak_rss_ratio_to_rival': 1.2,
            'maximum_mapping_regression_percent': 2.0,
        },
        'confidence_interval': {
            'level': 0.95,
            'method': 'fixed_seed_two_stage_hierarchical_dataset_run_bootstrap',
            'seed': 20260825,
            'samples': 1000,
        },
    }
    value['scope_sha256'] = MODULE.canonical_json_sha256(value)
    return value


def _manifest(tmp_path: Path, scope: dict) -> tuple[Path, str, str]:
    root = tmp_path / 'bundle'
    root.mkdir()
    failure_log = {'complete': True, 'events': []}
    failure_bytes = MODULE._canonical_failure_artifact_bytes(failure_log)
    (root / 'failure.json').write_bytes(failure_bytes)
    bindings = []
    for system in scope['systems']:
        for index in range(1, scope['run_count'] + 1):
            bindings.append({
                'system': system,
                'dataset': 'holdout_a',
                'run_index': index,
                'trajectory': {'path': f'{system}/{index}.trajectory'},
                'map': {'path': f'{system}/{index}.map'},
                'resource': {'path': f'{system}/{index}.resource'},
                'score': {'path': f'{system}/{index}.score'},
            })
    manifest = {
        'schema_version': 1,
        'manifest_kind': 'competitive_slam_evidence_bundle',
        'manifest_path': 'bundle_manifest.json',
        'canonical_root': '.',
        'claim_status': 'claim_eligible',
        'artifacts': [{
            'role': 'failure', 'path': 'failure.json',
            'size_bytes': len(failure_bytes), 'sha256': _sha(failure_bytes),
        }],
        'revision': {'systems': scope['revisions']},
        'run_artifact_bindings': bindings,
    }
    manifest['manifest_sha256'] = MODULE.canonical_manifest_sha256(manifest)
    manifest['manifest_sidecar_sha256'] = '0' * 64
    manifest_bytes = (json.dumps(manifest, sort_keys=True, separators=(',', ':'))
                      + '\n').encode()
    manifest_path = root / 'bundle_manifest.json'
    manifest_path.write_bytes(manifest_bytes)
    return root, manifest['manifest_sha256'], _sha(manifest_bytes)


def _recomputed(scope: dict) -> dict:
    checks = {name: {'pass': True, 'evidence': {}}
              for name in MODULE.REQUIRED_CHECKS}
    checks['three_complete_runs_and_completion']['evidence'] = {
        'expected_per_system': 3, 'complete_per_system': {'glim': 3, 'ours': 3},
        'failed_or_incomplete': [],
    }
    checks['mapping_non_regression']['evidence'] = {'comparisons': {}}
    return {
        'schema_version': 2,
        'evidence_kind': 'competitive_slam_victory_evidence_receipt',
        'status': 'PASS',
        'pass': True,
        'claim_eligible': True,
        'policy': {
            'minimum_aggregate_ape_improvement_percent': 10.0,
            'maximum_primary_regression_percent': 2.0,
        },
        'aggregate_ape_improvement_percent': 25.0,
        'aggregate_ape_m': {'ours': 0.75, 'glim': 1.0},
        'best_rival': 'glim',
        'bootstrap_ci': {
            'method': 'fixed_seed_two_stage_hierarchical_dataset_run_bootstrap',
            'seed': 20260825,
            'samples': 1000,
            'ci95_lower_m': 0.1,
            'ci95_upper_m': 0.4,
            'superiority': True,
        },
        'bootstrap_ci_by_rival': {
            'glim': {'ci95_lower_m': 0.1, 'superiority': True},
        },
        'memory_gate': {
            'checks': {'aggregate_best_rival': {
                'maximum_ratio': 1.2,
                'comparisons': {'glim': {'ratio': 0.9, 'pass': True}},
            }},
        },
        'checks': checks,
    }


def _suite(recomputed: dict, profile_sha: str, evidence_sha: str,
           manifest_sha: str) -> dict:
    suite = copy.deepcopy(recomputed)
    suite['receipt_identity'] = {
        'profile_sha256': profile_sha,
        'profile_sha256_kind': MODULE.PROFILE_CANONICAL_HASH_KIND,
        'profile_file_sha256': '0' * 64,
        'evidence_sha256': evidence_sha,
    }
    suite['claim_scope_sha256'] = '0' * 64
    suite['evidence_bundle_manifest_sha256'] = manifest_sha
    return suite


def _scored_evidence(scope: dict) -> dict:
    systems = {}
    for system in scope['systems']:
        systems[system] = {
            'provenance': {'revision': scope['revisions'][system]},
            'runs': [{
                'dataset': 'holdout_a', 'run_index': index,
                'complete': True, 'process_exit_status': 0,
                'trajectory_complete': True, 'sequence_failure': False,
                'catastrophic_failure': False, 'verified_false_loops': 0,
                'trajectory': {'ape_rmse_m': 0.75 if system == 'ours' else 1.0},
                'runtime': {'processing_rtf': 0.8, 'peak_rss_mb': 100.0},
                'map': {'plane_thickness_mean_m': 0.1,
                        'plane_thickness_p95_m': 0.2, 'planar_coverage': 0.8},
                'artifacts': {'trajectory_sha256': '1' * 64,
                              'map_sha256': '2' * 64},
            } for index in range(1, scope['run_count'] + 1)],
        }
    return {'schema_version': 2, 'evidence_kind': 'synthetic', 'systems': systems}


def _fixture(tmp_path: Path) -> tuple[Path, dict, Path]:
    repository = tmp_path / 'repo'
    repository.mkdir()
    profile = {
        'competitive_slam_profile': {
            'name': 'synthetic-profile',
            'evidence_gate_v2': {
                'required_systems': ['glim', 'ours'],
                'minimum_aggregate_ape_improvement_percent': 10.0,
                'maximum_peak_rss_ratio_to_rival': 1.2,
                'maximum_mapping_regression_percent': 2.0,
                'fresh_holdout_authorization': {
                    'required': True, 'status': 'READY',
                },
                'rival_source_closure': {'required': True},
                'dataset_source_closure': {'required': True},
            },
        }
    }
    profile_path = repository / 'profile.yaml'
    profile_sha_file = _write(profile_path, yaml.safe_dump(profile, sort_keys=True))
    profile_sha = MODULE.canonical_profile_sha256(profile)
    scope = _scope()
    evidence = _scored_evidence(scope)
    evidence_path = repository / 'scored-evidence.json'
    evidence_sha = _write(evidence_path, evidence)
    execution = {
        'status': 'PASS', 'pass': True, 'claim_scope_sha256': scope['scope_sha256'],
        'checks': {},
    }
    execution_path = repository / 'execution.json'
    execution_sha = _write(execution_path, execution)
    authorization = {
        'authorization': {
            'schema_version': 1, 'status': 'READY',
            'failure_log': {'complete': True, 'events': []},
        },
    }
    authorization_path = repository / 'authorization.json'
    authorization_sha = _write(authorization_path, authorization)
    bundle_root, manifest_sha, manifest_file_sha = _manifest(tmp_path, scope)
    manifest_path = bundle_root / 'bundle_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['authorization'] = authorization['authorization']
    manifest['manifest_sha256'] = MODULE.canonical_manifest_sha256(manifest)
    manifest_path.write_bytes(
        (json.dumps(manifest, sort_keys=True, separators=(',', ':')) + '\n').encode())
    manifest_sha = manifest['manifest_sha256']
    manifest_file_sha = _sha(manifest_path.read_bytes())
    recomputed = _recomputed(scope)
    suite = _suite(recomputed, profile_sha, evidence_sha, manifest_sha)
    suite['receipt_identity']['profile_file_sha256'] = profile_sha_file
    suite['claim_scope_sha256'] = scope['scope_sha256']
    suite_path = repository / 'suite.json'
    suite_sha = _write(suite_path, suite)
    spec = {
        'schema_version': 1,
        'publication_kind': MODULE.PUBLICATION_KIND,
        'repository_root': str(repository),
        'profile': {'path': str(profile_path), 'sha256': profile_sha_file},
        'suite_receipt': {'path': str(suite_path), 'sha256': suite_sha},
        'scored_evidence': {'path': str(evidence_path), 'sha256': evidence_sha},
        'execution_receipt': {'path': str(execution_path), 'sha256': execution_sha},
        'fresh_holdout_authorization': {
            'path': str(authorization_path), 'sha256': authorization_sha,
        },
        'evidence_bundle': {
            'root': str(bundle_root),
            'manifest_path': 'bundle_manifest.json',
            'manifest_sha256': manifest_sha,
            'manifest_file_sha256': manifest_file_sha,
        },
        'scope': scope,
        'caveats': ['Synthetic fixture only; not a benchmark result.'],
        'failures': [],
        'reproduction_command': ['python3', 'run_frozen_suite.py', '--sealed'],
    }
    spec_path = repository / 'publication.json'
    _write(spec_path, spec)
    return spec_path, spec, repository, recomputed


def _patch_verifiers(monkeypatch, recomputed, *, auth_pass=True):
    calls = []
    monkeypatch.setattr(
        MODULE, 'evaluate_evidence_v2',
        lambda evidence, contract, require_bundle=False: (
            calls.append((evidence, contract, require_bundle)) or
            copy.deepcopy(recomputed)))
    monkeypatch.setattr(
        MODULE, 'evaluate_execution_selection',
        lambda receipt, profile, root: {
            'status': 'PASS', 'pass': True,
            'claim_scope_sha256': receipt['claim_scope_sha256']})
    monkeypatch.setattr(MODULE, 'verify_fresh_holdout_authorization',
                        lambda document, **kwargs: {'status': 'PASS' if auth_pass else 'NOT_READY',
                                                    'pass': auth_pass,
                                                    'claim_eligible': auth_pass})
    monkeypatch.setattr(MODULE, 'verify_evidence_bundle',
                        lambda root, manifest, **kwargs: {
                            'status': 'PASS', 'pass': True, 'claim_eligible': True,
                            'manifest_sha256': kwargs['expected_manifest_sha256'],
                            'artifacts': [{
                                'role': 'failure', 'path': 'failure.json',
                                'size_bytes': len(MODULE._canonical_failure_artifact_bytes(
                                    {'complete': True, 'events': []})),
                                'sha256': _sha(MODULE._canonical_failure_artifact_bytes(
                                    {'complete': True, 'events': []})),
                            }],
                        })
    monkeypatch.setattr(MODULE, 'verify_rival_source_closure',
                        lambda profile, **kwargs: {'status': 'PASS', 'pass': True})
    monkeypatch.setattr(MODULE, 'verify_dataset_source_closure',
                        lambda profile, **kwargs: {'status': 'PASS', 'pass': True})
    return calls


def test_positive_publication_is_deterministic_and_atomic(tmp_path, monkeypatch):
    spec_path, _, _, recomputed = _fixture(tmp_path)
    calls = _patch_verifiers(monkeypatch, recomputed)
    first = MODULE.publish(spec_path, tmp_path / 'published-a')
    second = MODULE.publish(spec_path, tmp_path / 'published-b')
    assert first['status'] == 'PASS'
    assert first['claim_eligible'] is True
    assert first['receipt_sha256'] == second['receipt_sha256']
    assert len(calls) == 2
    assert all(call[2] is True for call in calls)
    markdown = (tmp_path / 'published-a' / 'claim.md').read_text()
    assert 'Aggregate APE:' in markdown
    assert 'improvement `25%' in markdown
    assert '95% CI' in markdown
    assert 'Ours maximum processing RTF:' in markdown
    assert 'RSS ratio:' in markdown
    assert 'Map gate: `PASS`' in markdown
    assert _sha(MODULE._canonical_failure_artifact_bytes(
        {'complete': True, 'events': []})) in markdown
    assert MODULE.STANDARD_CAVEAT in markdown
    assert ((tmp_path / 'published-a' / 'claim.md').read_bytes()) == (
        tmp_path / 'published-b' / 'claim.md').read_bytes()
    assert (tmp_path / 'published-a' / 'publication_receipt.json').stat().st_mode & 0o777 == 0o444
    with pytest.raises(MODULE.PublicationError, match='refusing to overwrite'):
        MODULE.publish(spec_path, tmp_path / 'published-a')


def test_report_only_suite_is_rejected_before_publication(tmp_path, monkeypatch):
    spec_path, spec, _, recomputed = _fixture(tmp_path)
    _patch_verifiers(monkeypatch, recomputed)
    suite_path = Path(spec['suite_receipt']['path'])
    suite = json.loads(suite_path.read_text())
    suite['status'] = 'INCOMPLETE'
    suite['pass'] = False
    suite_path.write_text(json.dumps(suite) + '\n')
    with pytest.raises(MODULE.PublicationError, match='sha256 does not match'):
        MODULE.publish(spec_path, tmp_path / 'not-published')
    assert not (tmp_path / 'not-published').exists()


def test_expired_or_missing_authorization_is_fail_closed(tmp_path, monkeypatch):
    spec_path, _, _, recomputed = _fixture(tmp_path)
    _patch_verifiers(monkeypatch, recomputed, auth_pass=False)
    with pytest.raises(MODULE.PublicationError, match='fresh holdout authorization'):
        MODULE.publish(spec_path, tmp_path / 'not-published')
    assert not (tmp_path / 'not-published').exists()


def test_partial_bundle_coverage_is_rejected(tmp_path, monkeypatch):
    spec_path, spec, _, recomputed = _fixture(tmp_path)
    _patch_verifiers(monkeypatch, recomputed)
    manifest_path = Path(spec['evidence_bundle']['root']) / 'bundle_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['run_artifact_bindings'].pop()
    manifest['manifest_sha256'] = MODULE.canonical_manifest_sha256(manifest)
    manifest_path.write_text(json.dumps(manifest, sort_keys=True, separators=(',', ':')) + '\n')
    spec['evidence_bundle']['manifest_sha256'] = manifest['manifest_sha256']
    spec['evidence_bundle']['manifest_file_sha256'] = _sha(manifest_path.read_bytes())
    suite_path = Path(spec['suite_receipt']['path'])
    suite = json.loads(suite_path.read_text())
    suite['checks']['evidence_bundle_integrity']['evidence']['manifest_sha256'] = (
        manifest['manifest_sha256'])
    suite_path.write_text(json.dumps(suite, sort_keys=True) + '\n')
    spec['suite_receipt']['sha256'] = _sha(suite_path.read_bytes())
    Path(spec_path).write_text(json.dumps(spec, sort_keys=True) + '\n')
    with pytest.raises(MODULE.PublicationError, match='differs from recomputed'):
        MODULE.publish(spec_path, tmp_path / 'not-published')


def test_scope_hash_tamper_is_rejected(tmp_path, monkeypatch):
    spec_path, spec, _, recomputed = _fixture(tmp_path)
    _patch_verifiers(monkeypatch, recomputed)
    spec['scope']['datasets'] = ['different']
    Path(spec_path).write_text(json.dumps(spec, sort_keys=True) + '\n')
    with pytest.raises(MODULE.PublicationError, match='scope.scope_sha256'):
        MODULE.publish(spec_path, tmp_path / 'not-published')


def test_supplied_metric_tamper_cannot_override_recomputed_evaluator(tmp_path, monkeypatch):
    spec_path, spec, _, recomputed = _fixture(tmp_path)
    calls = _patch_verifiers(monkeypatch, recomputed)
    suite_path = Path(spec['suite_receipt']['path'])
    suite = json.loads(suite_path.read_text())
    suite['aggregate_ape_improvement_percent'] = 999.0
    suite_path.write_text(json.dumps(suite, sort_keys=True) + '\n')
    spec['suite_receipt']['sha256'] = _sha(suite_path.read_bytes())
    Path(spec_path).write_text(json.dumps(spec, sort_keys=True) + '\n')
    with pytest.raises(MODULE.PublicationError, match='differs from recomputed'):
        MODULE.publish(spec_path, tmp_path / 'not-published')
    assert len(calls) == 1
    assert calls[0][2] is True


def test_authorization_failure_log_is_part_of_sealed_failure_projection():
    auth = {'authorization': {
        'failure_log': {'complete': True, 'events': [
            {'event_id': 'failure-1', 'reason': 'synthetic'}]},
    }}
    failures = MODULE._failure_ledger({'errors': []}, auth)
    assert failures == [{
        'kind': 'authorization_failure', 'index': 0,
        'event': {'event_id': 'failure-1', 'reason': 'synthetic'},
    }]
    with pytest.raises(MODULE.PublicationError, match='failure_log'):
        MODULE._failure_ledger({'errors': []}, {'authorization': {
            'failure_log': {'complete': False, 'events': []},
        }})


def test_global_failure_artifact_identity_is_reopened_and_bound():
    manifest = {'artifacts': [{
        'role': 'failure', 'path': 'failure.json', 'size_bytes': 2,
        'sha256': 'a' * 64,
    }]}
    auth = {'authorization': {'failure_log': {
        'complete': True, 'events': [{'artifact_sha256': 'a' * 64}],
    }}}
    assert MODULE._failure_artifact(manifest, auth)['sha256'] == 'a' * 64
    auth['authorization']['failure_log']['events'][0]['artifact_sha256'] = 'b' * 64
    with pytest.raises(MODULE.PublicationError, match='failure artifact SHA'):
        MODULE._failure_artifact(manifest, auth)


def test_failure_artifact_content_must_equal_authorization_log():
    failure_log = {
        'complete': True,
        'events': [{'event_id': 'failure-1', 'reason': 'synthetic'}],
    }
    auth = {'authorization': {'failure_log': failure_log}}
    valid = MODULE._canonical_failure_artifact_bytes(failure_log)
    MODULE._validate_failure_artifact_bytes(valid, auth)
    invalid = [
        b'not-json\n',
        valid.replace(b'"failure_log":', b'"extra":true,"failure_log":'),
        MODULE._canonical_failure_artifact_bytes({
            'complete': True, 'events': [],
        }),
        MODULE._canonical_failure_artifact_bytes({
            'complete': True,
            'events': [{'artifact_sha256': 'a' * 64}],
        }),
    ]
    for candidate in invalid:
        with pytest.raises(MODULE.PublicationError):
            MODULE._validate_failure_artifact_bytes(candidate, auth)


def test_direct_cli_help_has_no_checkout_pythonpath_dependency():
    environment = os.environ.copy()
    environment.pop('PYTHONPATH', None)
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--help'],
        cwd=ROOT, env=environment, capture_output=True, text=True,
        check=False)
    assert result.returncode == 0
    assert '--output-root' in result.stdout
