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

"""Prepare a GT-blind post-score handoff for evidence bundle composition.

This tool is a handoff boundary, not a scorer and not a benchmark runner.  It
accepts already-scored v2 run records, an explicit all-run source index, and a
GT-safe scorer authorization receipt.  It computes source byte identities and
the repository's canonical ``competitive_run_score_v1`` digest, then emits a
sanitized evidence document and a composition spec.  It never copies source
artifacts, opens a dataset, or emits GT paths/content.  The later bundle
composer remains responsible for copying bytes and the fresh-holdout claim
authorization gate remains separate.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import sys
import tempfile
from typing import Any, Mapping

import yaml

_SCRIPT_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if (
        (_SCRIPT_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file()
        and (_SCRIPT_SOURCE_ROOT / 'scripts' / 'benchmark_phase_contract.py').is_file()
        and str(_SCRIPT_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SCRIPT_SOURCE_ROOT))

try:
    from lidarslam_benchmark_tools import package_root
    from lidarslam_benchmark_tools.compose_competitive_evidence_bundle import (
        CompositionError,)
    from lidarslam_benchmark_tools.competitive_execution_attempt_receipt import (
        ExecutionReceiptError, validate_receipt)
except ModuleNotFoundError:  # pragma: no cover - installed/source bootstrap
    def package_root() -> Path:
        return Path(__file__).resolve().parents[1]

    from lidarslam_benchmark_tools.compose_competitive_evidence_bundle import (
        CompositionError,)
    from lidarslam_benchmark_tools.competitive_execution_attempt_receipt import (
        ExecutionReceiptError, validate_receipt)

composer = importlib.import_module(
    'lidarslam_benchmark_tools.compose_competitive_evidence_bundle')


ROOT = package_root()
REQUIRED_ROLES = composer.REQUIRED_ROLES
HANDOFF_SCHEMA_PATH = ROOT / (
    'configs/slam_benchmark_profiles/'
    'competitive_evidence_bundle_handoff_v1.schema.json')
HANDOFF_SCHEMA_VERSION = 1
HANDOFF_RECEIPT_KIND = 'competitive_slam_evidence_bundle_handoff'
# The per-attempt execution receipt is mandatory for claim-bound handoff;
# deterministic metric bytes alone do not prove independent repetitions.
RUN_SOURCE_ROLES = ('trajectory', 'map', 'resource', 'execution')
_RESOURCE_KEYS = (
    'receipt_sha256', 'timing_authority', 'functional_only',
    'performance_gate_eligible')
_RUNTIME_KEYS = (
    'processing_rtf', 'online_compute_rtf', 'peak_rss_mb',
    'phase_contract_version', 'phase_mode',
    'paced_followability_passed', 'unpaced_throughput_gate_passed')
_GT_KEY_PARTS = (
    'ground_truth', 'ground-truth', 'gt_path', 'gt_root', 'gt_file',
    'gt_mount', 'gt_content')


class HandoffError(ValueError):
    """A malformed or unsafe post-score handoff."""


def _canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _sha_field(value: Any, label: str) -> str:
    if (not isinstance(value, str) or len(value) != 64 or
            any(char not in '0123456789abcdef' for char in value)):
        raise HandoffError(f'{label} must be lowercase 64-hex')
    return value


def _write_new(path: Path, data: bytes, label: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() or path.is_symlink():
        raise HandoffError(f'{label} destination already exists')
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, 'wb') as stream:
            fd = -1
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        if fd >= 0:
            os.close(fd)


def _seal(root: Path) -> None:
    for directory, dirnames, filenames in os.walk(root, topdown=False, followlinks=False):
        for name in filenames:
            path = Path(directory) / name
            if path.is_symlink() or not path.is_file() or path.stat().st_nlink != 1:
                raise HandoffError('handoff output contains an unsafe file')
            path.chmod(0o444)
        for name in dirnames:
            path = Path(directory) / name
            if path.is_symlink() or not path.is_dir():
                raise HandoffError('handoff output contains an unsafe directory')
            path.chmod(0o555)
    root.chmod(0o555)


def _validate_schema(request: Mapping[str, Any]) -> None:
    try:
        import jsonschema
        schema = json.loads(HANDOFF_SCHEMA_PATH.read_text(encoding='utf-8'))
        errors = sorted(
            jsonschema.Draft202012Validator(schema).iter_errors(request),
            key=lambda item: list(item.path))
    except (OSError, ImportError, ValueError, TypeError) as exc:
        raise HandoffError(f'handoff schema is unavailable: {exc}') from exc
    if errors:
        message = '; '.join(error.message for error in errors[:8])
        raise HandoffError(f'handoff request schema is invalid: {message}')


def _audit_no_gt_metadata(value: Any, label: str) -> None:
    """Reject GT paths/content while retaining harmless GT-blind booleans."""
    if isinstance(value, Mapping):
        for key, child in value.items():
            if isinstance(key, str):
                lowered = key.lower()
                if any(part in lowered for part in _GT_KEY_PARTS):
                    raise HandoffError(f'{label} contains forbidden GT metadata key')
            _audit_no_gt_metadata(child, f'{label}.{key}')
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _audit_no_gt_metadata(child, f'{label}[{index}]')
    elif isinstance(value, str):
        if composer._contains_gt(value):
            raise HandoffError(f'{label} contains a forbidden GT path/component')


def _descriptor(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise HandoffError(f'{label} must be a mapping')
    try:
        root = composer._source_root(value.get('source_root'), f'{label}.source_root')
        relative = composer._safe_relative(
            value.get('source_path'), f'{label}.source_path')
        if composer._contains_gt(relative):
            raise HandoffError(f'{label}.source_path contains a forbidden GT component')
        declared_size = composer._size(
            value.get('size_bytes'), f'{label}.size_bytes')
        declared_sha = composer._sha(
            value.get('sha256'), f'{label}.sha256')
        path = composer._safe_source_path(root, relative, label)
        data = composer._read_nofollow(path, label)
    except CompositionError as exc:
        raise HandoffError(str(exc)) from exc
    observed_sha = _sha(data)
    if len(data) != declared_size or observed_sha != declared_sha:
        raise HandoffError(
            f'{label} source receipt drift: expected {declared_size}/{declared_sha}, '
            f'observed {len(data)}/{observed_sha}')
    receipt_sha = value.get('receipt_sha256')
    if receipt_sha is not None:
        receipt_sha = _sha_field(receipt_sha, f'{label}.receipt_sha256')
    return {
        'source_root': root,
        'source_path': relative,
        'size_bytes': declared_size,
        'sha256': declared_sha,
        **({'receipt_sha256': receipt_sha} if receipt_sha is not None else {}),
    }


def _json_descriptor(value: Mapping[str, Any]) -> dict[str, Any]:
    """Convert an internal descriptor to the path-bearing JSON form."""
    return {
        **value,
        'source_root': str(value['source_root']),
    }


def _global_sources(request: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    raw = request.get('artifacts')
    if not isinstance(raw, list):
        raise HandoffError('artifacts must be a list')
    sources: dict[str, dict[str, Any]] = {}
    for index, value in enumerate(raw):
        if not isinstance(value, Mapping):
            raise HandoffError(f'artifacts[{index}] must be a mapping')
        role = value.get('role')
        if not isinstance(role, str) or role not in REQUIRED_ROLES:
            raise HandoffError(f'artifacts[{index}].role is invalid')
        if role in sources:
            raise HandoffError(f'duplicate global role: {role}')
        sources[role] = _descriptor(value, f'artifacts[{index}]')
    if set(sources) != set(REQUIRED_ROLES):
        missing = sorted(set(REQUIRED_ROLES) - set(sources))
        raise HandoffError('global artifact coverage is incomplete: ' + ','.join(missing))
    return sources


def _run_sources(
        request: Mapping[str, Any],
        expected: Mapping[tuple[str, str, int], Mapping[str, Any]],
) -> dict[tuple[str, str, int], dict[str, dict[str, Any]]]:
    raw = request.get('run_artifact_sources')
    if not isinstance(raw, list):
        raise HandoffError('run_artifact_sources must be a list')
    sources: dict[tuple[str, str, int], dict[str, dict[str, Any]]] = {}
    for index, value in enumerate(raw):
        if not isinstance(value, Mapping):
            raise HandoffError(f'run_artifact_sources[{index}] must be a mapping')
        try:
            system = composer._component(
                value.get('system'), f'run_artifact_sources[{index}].system')
            dataset = composer._component(
                value.get('dataset'), f'run_artifact_sources[{index}].dataset')
        except CompositionError as exc:
            raise HandoffError(str(exc)) from exc
        run_index = value.get('run_index')
        if isinstance(run_index, bool) or not isinstance(run_index, int) or run_index < 1:
            raise HandoffError(f'run_artifact_sources[{index}].run_index is invalid')
        key = (system, dataset, run_index)
        if key in sources:
            raise HandoffError(f'duplicate run source identity: {key}')
        if key not in expected:
            raise HandoffError(f'run source identity is not scored: {key}')
        rows: dict[str, dict[str, Any]] = {}
        for role in RUN_SOURCE_ROLES:
            descriptor = _descriptor(
                value.get(role), f'run_artifact_sources[{index}].{role}')
            expected_sha = (
                expected[key]['execution_receipt_file_sha256']
                if role == 'execution' else expected[key][f'{role}_sha256'])
            if descriptor['sha256'] != expected_sha:
                raise HandoffError(f'{key}/{role} does not match scored evidence')
            if role == 'execution':
                expected_receipt_sha = expected[key]['execution_receipt_sha256']
                if descriptor.get('receipt_sha256') != expected_receipt_sha:
                    raise HandoffError(
                        f'{key}/execution canonical receipt identity does not '
                        'match scored evidence')
                try:
                    payload = composer._read_nofollow(
                        descriptor['source_root'] / descriptor['source_path'],
                        f'run_artifact_sources[{index}].execution')
                    document = json.loads(payload.decode('utf-8'))
                    validate_receipt(
                        document, expected_system=system,
                        expected_dataset=dataset, expected_run_index=run_index,
                        require_success=True,
                        expected_sha256=expected_receipt_sha)
                except (UnicodeDecodeError, json.JSONDecodeError,
                        ExecutionReceiptError, OSError, TypeError) as exc:
                    raise HandoffError(
                        f'{key}/execution receipt is invalid: {exc}') from exc
            rows[role] = descriptor
        sources[key] = rows
    if set(sources) != set(expected):
        missing = sorted(set(expected) - set(sources))
        extra = sorted(set(sources) - set(expected))
        raise HandoffError(f'run source coverage mismatch missing={missing} extra={extra}')
    return sources


def _sanitize_run(system: str, run: Mapping[str, Any], score_sha: str) -> dict[str, Any]:
    """Keep only the scalar non-GT fields consumed by the canonical score gate."""
    trajectory = run.get('trajectory')
    runtime = run.get('runtime')
    mapping = run.get('map', run.get('mapping'))
    resource = None
    for key in ('resource_evidence', 'resource_receipt', 'resource'):
        if isinstance(run.get(key), Mapping):
            resource = run[key]
            break
    if not isinstance(trajectory, Mapping) or not isinstance(runtime, Mapping) or \
            not isinstance(mapping, Mapping) or not isinstance(resource, Mapping):
        raise HandoffError(f'{system}:{run.get("dataset")} run is not score-complete')
    resource_sha = resource.get('receipt_sha256', resource.get('evidence_sha256'))
    sanitized_resource = {
        key: resource.get(key) for key in _RESOURCE_KEYS
        if resource.get(key) is not None
    }
    sanitized = {
        key: run.get(key) for key in (
            'dataset', 'run_index', 'complete', 'process_exit_status',
            'trajectory_complete', 'sequence_failure', 'catastrophic_failure',
            'verified_false_loops')
    }
    campaign_id = run.get('campaign_id')
    if (not isinstance(campaign_id, str) or len(campaign_id) != 64 or
            any(char not in '0123456789abcdef' for char in campaign_id)):
        raise HandoffError(
            f'{system}:{run.get("dataset")} campaign identity is missing')
    sanitized['campaign_id'] = campaign_id
    sanitized['trajectory'] = {'ape_rmse_m': trajectory.get('ape_rmse_m')}
    sanitized['runtime'] = {
        key: runtime.get(key) for key in _RUNTIME_KEYS
        if runtime.get(key) is not None
    }
    sanitized['map'] = {
        key: mapping.get(key) for key in (
            'plane_thickness_mean_m', 'plane_thickness_p95_m',
            'planar_coverage')
    }
    sanitized['artifacts'] = {
        'trajectory_sha256': run.get('artifacts', {}).get('trajectory_sha256'),
        'map_sha256': run.get('artifacts', {}).get('map_sha256'),
    }
    execution_sha = run.get('execution_receipt_sha256')
    if (not isinstance(execution_sha, str) or len(execution_sha) != 64 or
            any(char not in '0123456789abcdef' for char in execution_sha)):
        raise HandoffError(
            f'{system}:{run.get("dataset")} execution receipt is missing or invalid')
    sanitized['execution_receipt_sha256'] = execution_sha
    execution_file_sha = run.get('execution_receipt_file_sha256')
    if (not isinstance(execution_file_sha, str) or len(execution_file_sha) != 64 or
            any(char not in '0123456789abcdef' for char in execution_file_sha)):
        raise HandoffError(
            f'{system}:{run.get("dataset")} execution receipt file SHA is missing')
    sanitized['execution_receipt_file_sha256'] = execution_file_sha
    sanitized['resource_evidence'] = sanitized_resource
    sanitized['score_artifact_sha256'] = score_sha
    if resource_sha != sanitized_resource.get('receipt_sha256'):
        raise HandoffError(f'{system}:{run.get("dataset")} resource receipt is invalid')
    return sanitized


def _score_evidence(evidence: Mapping[str, Any]) -> tuple[dict, dict[tuple[str, str, int], dict]]:
    _audit_no_gt_metadata(evidence, 'scored evidence')
    try:
        runs = composer._collect_runs(evidence, require_score_digest=False)
    except CompositionError as exc:
        raise HandoffError(str(exc)) from exc
    systems = evidence.get('systems')
    if not isinstance(systems, Mapping):
        raise HandoffError('scored evidence systems are missing')
    output_systems: dict[str, dict[str, list[dict[str, Any]]]] = {}
    for raw_system, record in systems.items():
        try:
            system = composer._component(raw_system, 'system')
        except CompositionError as exc:
            raise HandoffError(str(exc)) from exc
        if not isinstance(record, Mapping) or not isinstance(record.get('runs'), list):
            raise HandoffError(f'{system}.runs is missing')
        output_runs = []
        for run in record['runs']:
            key = (system, run.get('dataset'), run.get('run_index'))
            normalized = runs.get(key)
            if normalized is None:
                raise HandoffError(f'{system} run identity is not normalized')
            score_sha = _sha(normalized['score_bytes'])
            output_runs.append(_sanitize_run(system, run, score_sha))
        output_systems[system] = {'runs': output_runs}
    sanitized = {'systems': output_systems}
    try:
        checked = composer._collect_runs(sanitized)
    except CompositionError as exc:
        raise HandoffError(f'sanitized evidence failed composer validation: {exc}') from exc
    return sanitized, checked


def _validate_expected_runs(
        request: Mapping[str, Any],
        observed: Mapping[tuple[str, str, int], Any],
) -> set[tuple[str, str, int]]:
    raw = request.get('expected_runs')
    if not isinstance(raw, list):
        raise HandoffError('expected_runs must be a list')
    expected: set[tuple[str, str, int]] = set()
    for index, value in enumerate(raw):
        if not isinstance(value, Mapping):
            raise HandoffError(f'expected_runs[{index}] must be a mapping')
        try:
            system = composer._component(
                value.get('system'), f'expected_runs[{index}].system')
            dataset = composer._component(
                value.get('dataset'), f'expected_runs[{index}].dataset')
        except CompositionError as exc:
            raise HandoffError(str(exc)) from exc
        run_index = value.get('run_index')
        if isinstance(run_index, bool) or not isinstance(run_index, int) or run_index < 1:
            raise HandoffError(f'expected_runs[{index}].run_index is invalid')
        key = (system, dataset, run_index)
        if key in expected:
            raise HandoffError(f'duplicate expected run identity: {key}')
        expected.add(key)
    if expected != set(observed):
        missing = sorted(expected - set(observed))
        extra = sorted(set(observed) - expected)
        raise HandoffError(
            f'expected run coverage mismatch missing={missing} extra={extra}')
    return expected


def _authorization(request: Mapping[str, Any], scorer_fingerprint: str,
                   evidence_sha: str, *, base: Path) -> dict[str, Any]:
    value = request.get('scoring_authorization')
    if not isinstance(value, Mapping) or value.get('status') != 'PASS':
        raise HandoffError('scoring_authorization must be PASS')
    declared = _sha_field(
        value.get('scorer_fingerprint'),
        'scoring_authorization.scorer_fingerprint')
    if declared != scorer_fingerprint:
        raise HandoffError('scoring authorization scorer fingerprint drift')
    declared_evidence_sha = _sha_field(
        value.get('input_evidence_sha256'),
        'scoring_authorization.input_evidence_sha256')
    if declared_evidence_sha != evidence_sha:
        raise HandoffError('scoring authorization evidence SHA drift')
    receipt_value = value.get('receipt_path')
    try:
        receipt, receipt_sha = composer._safe_document(
            receipt_value, 'scoring authorization receipt', base=base)
    except CompositionError as exc:
        raise HandoffError(str(exc)) from exc
    _audit_no_gt_metadata(receipt, 'scoring authorization receipt')
    if receipt.get('status') != 'PASS':
        raise HandoffError('scoring authorization receipt is not PASS')
    receipt_fingerprint = _sha_field(
        receipt.get('scorer_fingerprint'),
        'scoring authorization receipt scorer_fingerprint')
    if receipt_fingerprint != scorer_fingerprint:
        raise HandoffError('scoring authorization receipt fingerprint drift')
    receipt_evidence_sha = _sha_field(
        receipt.get('input_evidence_sha256'),
        'scoring authorization receipt input_evidence_sha256')
    if receipt_evidence_sha != evidence_sha:
        raise HandoffError('scoring authorization receipt evidence SHA drift')
    return {
        'status': 'PASS',
        'receipt_sha256': receipt_sha,
        'scorer_fingerprint': scorer_fingerprint,
        'input_evidence_sha256': evidence_sha,
    }


def _spec_metadata(request: Mapping[str, Any], global_sources: Mapping[str, Any],
                   evidence: Mapping[str, Any]) -> dict[str, Any]:
    profile = request.get('profile')
    scorer = request.get('scorer')
    revision = request.get('revision')
    if not isinstance(profile, Mapping) or not isinstance(scorer, Mapping) or \
            not isinstance(revision, Mapping):
        raise HandoffError('profile/scorer/revision metadata is incomplete')
    try:
        metadata = composer._metadata(
            {'profile': profile, 'scorer': scorer, 'revision': revision},
            global_sources)
    except CompositionError as exc:
        raise HandoffError(str(exc)) from exc
    expected_systems = set(revision.get('systems', {}))
    observed_systems = set(evidence.get('systems', {}))
    if expected_systems != observed_systems:
        raise HandoffError(
            f'system coverage mismatch missing={sorted(expected_systems - observed_systems)} '
            f'extra={sorted(observed_systems - expected_systems)}')
    return metadata


def prepare_handoff(request_path: Path, output_root: Path) -> dict[str, Any]:
    request, request_sha = composer._safe_document(
        request_path, 'handoff request', base=Path.cwd())
    _validate_schema(request)
    evidence, evidence_sha = composer._safe_document(
        request.get('evidence_path'), 'scored evidence',
        base=request_path.resolve().parent)
    sanitized_evidence, runs = _score_evidence(evidence)
    expected_runs = _validate_expected_runs(request, runs)
    global_sources = _global_sources(request)
    run_sources = _run_sources(request, runs)
    try:
        composer._reject_source_aliases(global_sources, run_sources)
    except CompositionError as exc:
        raise HandoffError(str(exc)) from exc
    metadata = _spec_metadata(request, global_sources, sanitized_evidence)
    scorer_fingerprint = metadata['scorer_fingerprint']
    authorization = _authorization(
        request, scorer_fingerprint, evidence_sha,
        base=request_path.resolve().parent)
    output = Path(output_root)
    if composer._contains_gt(str(output)):
        raise HandoffError('handoff output root contains a forbidden GT component')
    if output.exists() or output.is_symlink():
        raise HandoffError('handoff output root already exists; overwrite is forbidden')
    if not output.parent.is_dir() or output.parent.is_symlink():
        raise HandoffError('handoff output parent must be an existing directory')
    output = output.absolute()
    staging = Path(tempfile.mkdtemp(prefix=output.name + '.staging.', dir=output.parent))
    try:
        _, current_evidence_sha = composer._safe_document(
            request.get('evidence_path'), 'scored evidence',
            base=request_path.resolve().parent)
        if current_evidence_sha != evidence_sha:
            raise HandoffError('scored evidence changed during handoff')
        evidence_final = output / 'scored-evidence.json'
        spec = {
            'schema_version': 1,
            'evidence_path': str(evidence_final),
            'profile': {
                'name': metadata['profile_name'],
                'canonical_sha256': metadata['profile_canonical_sha256'],
            },
            'scorer': {'fingerprint': scorer_fingerprint},
            'revision': {'systems': metadata['revision_systems']},
            'post_score_authorization': authorization,
            'artifacts': [
                {**_json_descriptor(global_sources[role]), 'role': role}
                for role in REQUIRED_ROLES],
            'run_artifact_bindings': [],
        }
        for key in sorted(run_sources):
            system, dataset, run_index = key
            binding = {
                'system': system, 'dataset': dataset, 'run_index': run_index,
                **{
                    role: _json_descriptor(run_sources[key][role])
                    for role in RUN_SOURCE_ROLES
                },
            }
            spec['run_artifact_bindings'].append(binding)
        evidence_bytes = _canonical_bytes(sanitized_evidence)
        spec['post_score_authorization'] = {
            **authorization,
            'scored_evidence_sha256': _sha(evidence_bytes),
        }
        spec_bytes = _canonical_bytes(spec)
        _write_new(staging / 'scored-evidence.json', evidence_bytes, 'handoff evidence')
        _write_new(staging / 'composition-spec.json', spec_bytes, 'handoff spec')
        receipt = {
            'schema_version': HANDOFF_SCHEMA_VERSION,
            'receipt_kind': HANDOFF_RECEIPT_KIND,
            'status': 'PASS',
            'claim_eligible': False,
            'request_sha256': request_sha,
            'input_evidence_sha256': evidence_sha,
            'sanitized_evidence_sha256': _sha(evidence_bytes),
            'scored_evidence_sha256': _sha(evidence_bytes),
            'composition_spec_sha256': _sha(spec_bytes),
            'scoring_authorization': authorization,
            'system_count': len(metadata['revision_systems']),
            'run_count': len(runs),
            'expected_run_count': len(expected_runs),
            'global_artifact_count': len(global_sources),
            'source_descriptor_count': (
                len(global_sources) + len(run_sources) * len(RUN_SOURCE_ROLES)),
            'gt_content_copied': False,
            'scorer_invoked': False,
        }
        _write_new(
            staging / 'handoff.receipt.json', _canonical_bytes(receipt),
            'handoff receipt')
        _seal(staging)
        os.rename(staging, output)
        receipt['output_root'] = str(output)
        return receipt
    except Exception as exc:
        failure = staging / 'handoff.failure.json'
        try:
            _write_new(failure, _canonical_bytes({
                'schema_version': HANDOFF_SCHEMA_VERSION,
                'receipt_kind': HANDOFF_RECEIPT_KIND,
                'status': 'FAIL_CLOSED',
                'error': str(exc),
                'staging_root': str(staging),
            }), 'handoff failure receipt')
            _seal(staging)
        except (OSError, HandoffError):
            pass
        if isinstance(exc, HandoffError):
            raise
        raise HandoffError(str(exc)) from exc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = prepare_handoff(args.request, args.output)
    except (HandoffError, OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        result = {
            'schema_version': HANDOFF_SCHEMA_VERSION,
            'receipt_kind': HANDOFF_RECEIPT_KIND,
            'status': 'FAIL_CLOSED',
            'claim_eligible': False,
            'errors': [str(exc)],
        }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result.get('status') == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
