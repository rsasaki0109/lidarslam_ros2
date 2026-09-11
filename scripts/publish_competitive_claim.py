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

"""Publish a deterministic competitive claim only after every gate reopens.

This is a publication boundary, not a benchmark runner.  It reads only
already-sealed metadata and the canonical evidence bundle.  It never opens
ground-truth content, invokes a scorer, starts a subprocess, or writes the
repository README.  A profile or receipt with a promising status string is not
trusted: the bundle, execution-selection, holdout, rival, and dataset
verifiers are called again before an output is atomically sealed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import shlex
import shutil
import stat
import sys
import tempfile
from typing import Any, Mapping

import yaml

_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if ((_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file()
        and (_SOURCE_ROOT / 'scripts' / 'benchmark_phase_contract.py').is_file()
        and str(_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SOURCE_ROOT))

try:
    from lidarslam_benchmark_tools import package_root
    from lidarslam_benchmark_tools.check_competitive_dataset_source_closure import (
        verify_dataset_source_closure)
    from lidarslam_benchmark_tools.check_competitive_execution_selection import (
        evaluate as evaluate_execution_selection)
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (
        verify_rival_source_closure)
    from lidarslam_benchmark_tools.competitive_holdout_authorization import (
        verify_fresh_holdout_authorization)
    from lidarslam_benchmark_tools.competitive_identity_hash import (
        canonical_json_sha256, canonical_profile_sha256,
        PROFILE_CANONICAL_HASH_KIND)
    from lidarslam_benchmark_tools.evaluate_competitive_suite_gate import (
        evaluate_evidence_v2)
    from lidarslam_benchmark_tools.verify_competitive_evidence_bundle import (
        canonical_manifest_sha256, verify_evidence_bundle)
except ModuleNotFoundError:  # pragma: no cover - package absence fails closed later
    from lidarslam_benchmark_tools import package_root  # type: ignore[no-redef]
    from lidarslam_benchmark_tools.check_competitive_dataset_source_closure import (
        verify_dataset_source_closure)  # type: ignore[no-redef]
    from lidarslam_benchmark_tools.check_competitive_execution_selection import (
        evaluate as evaluate_execution_selection)  # type: ignore[no-redef]
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (
        verify_rival_source_closure)  # type: ignore[no-redef]
    from lidarslam_benchmark_tools.competitive_holdout_authorization import (
        verify_fresh_holdout_authorization)  # type: ignore[no-redef]
    from lidarslam_benchmark_tools.competitive_identity_hash import (
        canonical_json_sha256, canonical_profile_sha256,
        PROFILE_CANONICAL_HASH_KIND)  # type: ignore[no-redef]
    from lidarslam_benchmark_tools.evaluate_competitive_suite_gate import (
        evaluate_evidence_v2)  # type: ignore[no-redef]
    from lidarslam_benchmark_tools.verify_competitive_evidence_bundle import (
        canonical_manifest_sha256, verify_evidence_bundle)  # type: ignore[no-redef]


ROOT = package_root()
SCHEMA_PATH = ROOT / (
    'configs/slam_benchmark_profiles/competitive_claim_publication_v1.schema.json')
SCHEMA_VERSION = 1
PUBLICATION_KIND = 'competitive_sota_claim_publication'
RECEIPT_KIND = 'competitive_claim_publication_receipt'
SHA256_HEX = set('0123456789abcdef')
COMMIT_HEX = set('0123456789abcdef')
REQUIRED_CHECKS = (
    'three_complete_runs_and_completion',
    'processing_rtf_leq_one',
    'peak_rss_non_regression',
    'mapping_non_regression',
    'aggregate_ape_improvement',
    'dataset_cluster_bootstrap_95_superiority',
    'evidence_bundle_integrity',
)
METRIC_NAMES = {'ape', 'completion', 'rtf', 'rss', 'map'}
STANDARD_CAVEAT = (
    'This is a fixed-scope claim, not a universal or all-datasets claim.')
FAILURE_ARTIFACT_SCHEMA_VERSION = 1


class PublicationError(ValueError):
    """A malformed, stale, incomplete, or unauthorised claim input."""


def _canonical(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True).encode('utf-8')


def _strict_json_document(data: bytes, label: str) -> Any:
    """Parse one JSON document while rejecting duplicate object keys."""
    def object_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise PublicationError(f'{label} contains duplicate JSON keys')
            result[key] = value
        return result

    try:
        return json.loads(data.decode('utf-8'), object_pairs_hook=object_pairs)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PublicationError(f'{label} must be one UTF-8 JSON document') from exc


def _canonical_failure_artifact_bytes(failure_log: Mapping[str, Any]) -> bytes:
    """Encode the only failure artifact accepted by claim publication."""
    payload = {
        'schema_version': FAILURE_ARTIFACT_SCHEMA_VERSION,
        'failure_log': failure_log,
    }
    try:
        encoded = json.dumps(
            payload, sort_keys=True, separators=(',', ':'), ensure_ascii=True,
            allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise PublicationError('authorization failure_log is not canonical JSON') from exc
    return (encoded + '\n').encode('utf-8')


def _sha_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _sha(value: Any, label: str) -> str:
    if (not isinstance(value, str) or len(value) != 64 or
            value != value.lower() or any(char not in SHA256_HEX for char in value)):
        raise PublicationError(f'{label} must be lowercase 64-hex SHA-256')
    return value


def _commit(value: Any, label: str) -> str:
    if (not isinstance(value, str) or len(value) != 40 or value != value.lower()
            or any(char not in COMMIT_HEX for char in value)):
        raise PublicationError(f'{label} must be lowercase 40-hex revision')
    return value


def _text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip() or any(
            ord(char) < 0x20 and char not in '\t' for char in value):
        raise PublicationError(f'{label} must be non-empty printable text')
    return value


def _contains_gt(value: str) -> bool:
    lowered = value.lower().replace('\\', '/')
    components = set(lowered.split('/'))
    return bool(components & {'gt', 'ground_truth', 'ground-truth', 'groundtruth'}
                or 'ground_truth' in lowered or 'ground-truth' in lowered)


def _safe_path(path: Path, label: str, *, must_exist: bool = True) -> Path:
    if _contains_gt(str(path)):
        raise PublicationError(f'{label} points at a forbidden GT path')
    if path.is_symlink():
        raise PublicationError(f'{label} must not be a symlink')
    if must_exist and not path.exists():
        raise PublicationError(f'{label} is missing: {path}')
    return path


def _read_bytes(path: Path, label: str) -> bytes:
    """Read a single-link regular file and detect replacement during read."""
    _safe_path(path, label)
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0)
    try:
        fd = os.open(path, flags)
    except OSError as exc:
        raise PublicationError(f'{label} cannot be opened safely') from exc
    try:
        before = os.fstat(fd)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
            raise PublicationError(f'{label} must be a single-link regular file')
        with os.fdopen(fd, 'rb', closefd=True) as stream:
            fd = -1
            data = stream.read()
            after = os.fstat(stream.fileno())
        if (after.st_dev != before.st_dev or after.st_ino != before.st_ino or
                after.st_nlink != 1 or after.st_size != before.st_size):
            raise PublicationError(f'{label} changed during read')
        return data
    finally:
        if fd >= 0:
            os.close(fd)


def _doc(path: Path, label: str) -> tuple[Any, str]:
    data = _read_bytes(path, label)
    try:
        value = yaml.safe_load(data.decode('utf-8'))
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise PublicationError(f'{label} is not UTF-8 YAML/JSON') from exc
    return value, _sha_bytes(data)


def _ref(value: Any, label: str, *, base: Path) -> tuple[Any, str, Path]:
    if not isinstance(value, Mapping):
        raise PublicationError(f'{label} must be a mapping')
    raw_path = value.get('path')
    _text(raw_path, f'{label}.path')
    path = Path(raw_path)
    if not path.is_absolute():
        path = base / path
    path = _safe_path(path, f'{label}.path')
    document, actual = _doc(path, label)
    if actual != _sha(value.get('sha256'), f'{label}.sha256'):
        raise PublicationError(f'{label}.sha256 does not match bytes')
    return document, actual, path


def _validate_schema(spec: Mapping[str, Any]) -> None:
    try:
        import jsonschema
        schema = json.loads(_read_bytes(SCHEMA_PATH, 'publication schema').decode())
        errors = sorted(jsonschema.Draft7Validator(schema).iter_errors(spec),
                        key=lambda error: list(error.path))
    except ImportError as exc:
        raise PublicationError('jsonschema is required for publication validation') from exc
    if errors:
        message = '; '.join(error.message for error in errors)
        raise PublicationError('spec schema error: ' + message)


def _scope_payload(scope: Mapping[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in scope.items() if key != 'scope_sha256'}


def _check_scope(scope: Any) -> tuple[dict[str, Any], str]:
    if not isinstance(scope, Mapping):
        raise PublicationError('scope must be a mapping')
    systems = scope.get('systems')
    datasets = scope.get('datasets')
    if (not isinstance(systems, list) or not systems or
            systems != sorted(set(systems))):
        raise PublicationError('scope.systems must be sorted and unique')
    if (not isinstance(datasets, list) or not datasets or
            datasets != sorted(set(datasets))):
        raise PublicationError('scope.datasets must be sorted and unique')
    for index, item in enumerate([*systems, *datasets]):
        _text(item, f'scope identity {index}')
        if '/' in item or '\\' in item:
            raise PublicationError('scope identities must not contain path separators')
    revisions = scope.get('revisions')
    if not isinstance(revisions, Mapping) or sorted(revisions) != systems:
        raise PublicationError('scope.revisions must cover exactly scope.systems')
    for system in systems:
        _commit(revisions[system], f'scope.revisions[{system}]')
    run_count = scope.get('run_count')
    if (not isinstance(run_count, int) or isinstance(run_count, bool)
            or run_count < 3):
        raise PublicationError('scope.run_count must be an integer >= 3')
    metrics = scope.get('metrics')
    if (not isinstance(metrics, list) or metrics != sorted(set(metrics))
            or not METRIC_NAMES.issubset({str(item).lower() for item in metrics})):
        raise PublicationError('scope.metrics must include APE/completion/RTF/RSS/map')
    thresholds = scope.get('thresholds')
    if not isinstance(thresholds, Mapping):
        raise PublicationError('scope.thresholds must be a mapping')
    improvement = thresholds.get('minimum_aggregate_ape_improvement_percent')
    if (not isinstance(improvement, (int, float)) or isinstance(improvement, bool)
            or not math.isfinite(float(improvement)) or float(improvement) < 10.0):
        raise PublicationError('scope threshold must require aggregate improvement >= 10%')
    ci = scope.get('confidence_interval')
    if not isinstance(ci, Mapping) or ci.get('level') != 0.95:
        raise PublicationError('scope.confidence_interval.level must be 0.95')
    scope_sha = _sha(scope.get('scope_sha256'), 'scope.scope_sha256')
    actual = canonical_json_sha256(_scope_payload(scope))
    if scope_sha != actual:
        raise PublicationError('scope.scope_sha256 does not match canonical scope')
    return dict(scope), scope_sha


def _all_checks_pass(receipt: Mapping[str, Any], label: str) -> None:
    checks = receipt.get('checks')
    if not isinstance(checks, Mapping):
        raise PublicationError(f'{label}.checks is missing')
    for name, row in checks.items():
        if not isinstance(row, Mapping) or row.get('pass') is not True:
            raise PublicationError(f'{label} check is not PASS: {name}')


def _finite(value: Any, label: str, *, positive: bool = False) -> float:
    if (not isinstance(value, (int, float)) or isinstance(value, bool) or
            not math.isfinite(float(value)) or (positive and float(value) <= 0.0)):
        raise PublicationError(f'{label} must be finite' + (' and > 0' if positive else ''))
    return float(value)


def _derive_scope(profile: Mapping[str, Any], evidence: Mapping[str, Any],
                  result: Mapping[str, Any]) -> tuple[dict[str, Any], str]:
    """Derive the publication scope from evaluated evidence and profile policy."""
    systems_value = evidence.get('systems')
    if not isinstance(systems_value, Mapping) or not systems_value:
        raise PublicationError('scored evidence systems are missing')
    systems = sorted(str(item) for item in systems_value)
    if any(not item or '/' in item or '\\' in item for item in systems):
        raise PublicationError('scored evidence system identity is unsafe')
    if 'ours' not in systems:
        raise PublicationError("scored evidence must include the 'ours' system")
    contract = profile.get('competitive_slam_profile', profile)
    policy = contract.get('evidence_gate_v2', {}) if isinstance(contract, Mapping) else {}
    configured_systems = policy.get('required_systems') if isinstance(policy, Mapping) else None
    if configured_systems is not None and sorted(configured_systems) != systems:
        raise PublicationError('scored evidence systems do not match profile policy')
    revisions: dict[str, str] = {}
    dataset_counts: dict[str, dict[str, int]] = {}
    for system in systems:
        record = systems_value.get(system)
        if not isinstance(record, Mapping):
            raise PublicationError(f'scored evidence system is malformed: {system}')
        provenance = record.get('provenance')
        if not isinstance(provenance, Mapping):
            raise PublicationError(f'scored evidence provenance is missing: {system}')
        revisions[system] = _commit(provenance.get('revision'),
                                    f'evidence.{system}.provenance.revision')
        runs = record.get('runs')
        if not isinstance(runs, list) or not runs:
            raise PublicationError(f'scored evidence runs are missing: {system}')
        counts: dict[str, int] = {}
        for run in runs:
            if not isinstance(run, Mapping):
                raise PublicationError(f'scored evidence run is malformed: {system}')
            dataset = _text(run.get('dataset'), f'{system}.run.dataset')
            if '/' in dataset or '\\' in dataset:
                raise PublicationError('scored evidence dataset identity is unsafe')
            counts[dataset] = counts.get(dataset, 0) + 1
        dataset_counts[system] = counts
    datasets = sorted(set().union(*(counts for counts in dataset_counts.values())))
    if not datasets or any(counts != dataset_counts[systems[0]]
                           for counts in dataset_counts.values()):
        raise PublicationError('scored evidence system/dataset coverage differs')
    run_counts = set(dataset_counts[systems[0]].values())
    if len(run_counts) != 1 or next(iter(run_counts)) < 3:
        raise PublicationError('scored evidence requires at least three runs per dataset')
    run_count = next(iter(run_counts))
    result_policy = result.get('policy')
    if not isinstance(result_policy, Mapping):
        raise PublicationError('recomputed suite result policy is missing')
    minimum_improvement = _finite(
        result_policy.get('minimum_aggregate_ape_improvement_percent'),
        'suite policy minimum aggregate improvement')
    if minimum_improvement < 10.0:
        raise PublicationError('profile/evaluator improvement threshold is below 10%')
    profile_gate = policy if isinstance(policy, Mapping) else {}
    max_ratio = profile_gate.get('maximum_peak_rss_ratio_to_rival')
    if max_ratio is None:
        memory = result.get('memory_gate')
        checks = memory.get('checks', {}) if isinstance(memory, Mapping) else {}
        aggregate = checks.get('aggregate_best_rival', {})
        max_ratio = aggregate.get('maximum_ratio') if isinstance(aggregate, Mapping) else None
    max_ratio = _finite(max_ratio, 'maximum RSS ratio', positive=True)
    max_primary = _finite(
        result_policy.get('maximum_primary_regression_percent'),
        'maximum primary regression')
    max_mapping = _finite(
        profile_gate.get('maximum_mapping_regression_percent'),
        'maximum mapping regression')
    ci = result.get('bootstrap_ci')
    if not isinstance(ci, Mapping):
        raise PublicationError('recomputed 95% bootstrap result is missing')
    ci_scope = {
        'level': 0.95,
        'method': _text(ci.get('method'), 'bootstrap method'),
        'seed': ci.get('seed'),
        'samples': ci.get('samples'),
    }
    if (not isinstance(ci_scope['seed'], int) or isinstance(ci_scope['seed'], bool) or
            not isinstance(ci_scope['samples'], int) or isinstance(ci_scope['samples'], bool)):
        raise PublicationError('bootstrap seed/sample identity is missing')
    scope = {
        'systems': systems,
        'datasets': datasets,
        'revisions': revisions,
        'run_count': run_count,
        'metrics': ['APE', 'RSS', 'RTF', 'completion', 'map'],
        'thresholds': {
            'minimum_aggregate_ape_improvement_percent': minimum_improvement,
            'maximum_primary_regression_percent': max_primary,
            'maximum_peak_rss_ratio_to_rival': max_ratio,
            'maximum_mapping_regression_percent': max_mapping,
        },
        'confidence_interval': ci_scope,
    }
    scope['scope_sha256'] = canonical_json_sha256(scope)
    return scope, scope['scope_sha256']


def _suite_projection(value: Mapping[str, Any]) -> dict[str, Any]:
    """Keep only the exact evaluator result; metadata is checked separately."""
    return {key: value[key] for key in value
            if key not in {'receipt_identity', 'claim_scope_sha256',
                           'evidence_bundle_manifest_sha256'}}


def _require_suite_receipt(receipt: Any, recomputed: Mapping[str, Any],
                           scope_sha: str, bundle_sha: str, profile_sha: str,
                           profile_file_sha: str, evidence_sha: str) -> None:
    if not isinstance(receipt, Mapping):
        raise PublicationError('suite receipt must be a mapping')
    expected = _suite_projection(recomputed)
    supplied = _suite_projection(receipt)
    if supplied != expected:
        raise PublicationError(
            'supplied suite receipt differs from recomputed evaluator result')
    identity = receipt.get('receipt_identity')
    expected_identity = {
        'profile_sha256': profile_sha,
        'profile_sha256_kind': PROFILE_CANONICAL_HASH_KIND,
        'profile_file_sha256': profile_file_sha,
        'evidence_sha256': evidence_sha,
    }
    if identity != expected_identity:
        raise PublicationError('suite receipt identity is not bound to recomputed inputs')
    if receipt.get('claim_scope_sha256') not in {None, scope_sha}:
        raise PublicationError('suite receipt scope identity is mismatched')
    if receipt.get('evidence_bundle_manifest_sha256') not in {None, bundle_sha}:
        raise PublicationError('suite receipt bundle identity is mismatched')
    if recomputed.get('schema_version') != 2 or recomputed.get('pass') is not True or \
            recomputed.get('claim_eligible') is not True or recomputed.get('status') != 'PASS':
        raise PublicationError('recomputed suite result is not PASS/claim_eligible')
    checks = recomputed.get('checks')
    if not isinstance(checks, Mapping):
        raise PublicationError('recomputed suite checks are missing')
    for name in REQUIRED_CHECKS:
        row = checks.get(name)
        if not isinstance(row, Mapping) or row.get('pass') is not True:
            raise PublicationError(f'recomputed suite required check is not PASS: {name}')
    _all_checks_pass(recomputed, 'recomputed suite result')
    threshold = recomputed['policy']['minimum_aggregate_ape_improvement_percent']
    if _finite(recomputed.get('aggregate_ape_improvement_percent'),
               'aggregate APE improvement') < _finite(threshold, 'improvement threshold'):
        raise PublicationError('recomputed aggregate improvement is below threshold')
    candidates = []
    by_rival = recomputed.get('bootstrap_ci_by_rival')
    if isinstance(by_rival, Mapping):
        candidates.extend(by_rival.values())
    if isinstance(recomputed.get('bootstrap_ci'), Mapping):
        candidates.append(recomputed['bootstrap_ci'])
    if (not candidates or any(
            not isinstance(item, Mapping) or item.get('superiority') is not True or
            _finite(item.get('ci95_lower_m'), 'bootstrap lower CI') <= 0
            for item in candidates)):
        raise PublicationError('recomputed result lacks positive 95% CI superiority')


def _manifest_coverage(manifest: Mapping[str, Any], scope: Mapping[str, Any]) -> None:
    if manifest.get('claim_status') != 'claim_eligible':
        raise PublicationError('bundle manifest is not claim_eligible')
    systems = scope['systems']
    datasets = scope['datasets']
    count = scope['run_count']
    expected = {(system, dataset, index) for system in systems for dataset in datasets
                for index in range(1, count + 1)}
    bindings = manifest.get('run_artifact_bindings')
    if not isinstance(bindings, list):
        raise PublicationError('bundle run_artifact_bindings is missing')
    observed: set[tuple[str, str, int]] = set()
    for item in bindings:
        if not isinstance(item, Mapping):
            raise PublicationError('bundle run binding is malformed')
        key = (item.get('system'), item.get('dataset'), item.get('run_index'))
        if (not isinstance(key[0], str) or not isinstance(key[1], str) or
                not isinstance(key[2], int) or isinstance(key[2], bool)):
            raise PublicationError(f'bundle run binding identity is malformed: {key}')
        if key in observed:
            raise PublicationError(f'duplicate bundle run binding: {key}')
        observed.add(key)
        for role in ('trajectory', 'map', 'resource', 'score'):
            if not isinstance(item.get(role), Mapping):
                raise PublicationError(f'bundle run binding lacks {role}: {key}')
    if observed != expected:
        missing = sorted(expected - observed)
        extra = sorted(observed - expected)
        raise PublicationError(f'bundle run coverage mismatch missing={missing} extra={extra}')
    revision = manifest.get('revision', {}).get('systems')
    if revision != scope['revisions']:
        raise PublicationError('bundle revision systems do not match publication scope')


def _required_result(result: Any, label: str) -> None:
    if not isinstance(result, Mapping) or result.get('pass') is not True:
        raise PublicationError(f'{label} verifier did not PASS: {result}')
    if result.get('status') not in {'PASS', 'READY', 'frozen', 'ready'}:
        raise PublicationError(f'{label} verifier status is not PASS: {result.get("status")}')


def _metric_projection(result: Mapping[str, Any], evidence: Mapping[str, Any],
                       scope: Mapping[str, Any]) -> dict[str, Any]:
    """Extract only finite, evaluator-derived values used in the Markdown."""
    aggregate = result.get('aggregate_ape_m')
    best_rival = result.get('best_rival')
    if not isinstance(aggregate, Mapping) or not isinstance(best_rival, str):
        raise PublicationError('recomputed aggregate APE projection is missing')
    ours_ape = _finite(aggregate.get('ours'), 'aggregate ours APE', positive=True)
    rival_ape = _finite(
        aggregate.get(best_rival), f'aggregate {best_rival} APE', positive=True)
    improvement = _finite(
        result.get('aggregate_ape_improvement_percent'),
        'aggregate APE improvement')
    ci = result.get('bootstrap_ci')
    if not isinstance(ci, Mapping):
        raise PublicationError('recomputed bootstrap projection is missing')
    ci_lower = _finite(ci.get('ci95_lower_m'), '95% CI lower bound')
    ci_upper = _finite(ci.get('ci95_upper_m'), '95% CI upper bound')
    if ci_lower > ci_upper:
        raise PublicationError('95% CI bounds are inverted')
    systems = evidence.get('systems')
    if not isinstance(systems, Mapping):
        raise PublicationError('evidence systems are missing for RTF projection')
    rtf_values_by_system: dict[str, list[float]] = {}
    for system, record in systems.items():
        runs = record.get('runs') if isinstance(record, Mapping) else None
        if not isinstance(runs, list):
            raise PublicationError(f'RTF runs are missing: {system}')
        for run in runs:
            runtime = run.get('runtime') if isinstance(run, Mapping) else None
            if not isinstance(runtime, Mapping):
                raise PublicationError('runtime projection is missing')
            rtf_values_by_system.setdefault(str(system), []).append(
                _finite(runtime.get('processing_rtf'),
                        'processing RTF', positive=True))
    if not rtf_values_by_system.get('ours'):
        raise PublicationError('ours processing RTF values are missing')
    memory = result.get('memory_gate')
    memory_checks = memory.get('checks', {}) if isinstance(memory, Mapping) else {}
    aggregate_memory = memory_checks.get('aggregate_best_rival', {})
    if not isinstance(aggregate_memory, Mapping):
        raise PublicationError('RSS aggregate check is missing')
    comparisons = aggregate_memory.get('comparisons')
    if not isinstance(comparisons, Mapping) or best_rival not in comparisons:
        raise PublicationError('RSS best-rival comparison is missing')
    rss_row = comparisons[best_rival]
    if not isinstance(rss_row, Mapping):
        raise PublicationError('RSS best-rival comparison is malformed')
    rss_ratio = _finite(rss_row.get('ratio'), 'RSS ratio', positive=True)
    rss_limit = _finite(aggregate_memory.get('maximum_ratio'),
                        'RSS ratio limit', positive=True)
    checks = result.get('checks')
    if not isinstance(checks, Mapping):
        raise PublicationError('recomputed metric checks are missing')
    map_check = checks.get('mapping_non_regression')
    completion_check = checks.get('three_complete_runs_and_completion')
    if not isinstance(map_check, Mapping) or not isinstance(completion_check, Mapping):
        raise PublicationError('completion/map result checks are missing')
    if map_check.get('pass') is not True or completion_check.get('pass') is not True:
        raise PublicationError('completion or map gate is not PASS')
    return {
        'aggregate_ape_m': {
            'ours': ours_ape, 'best_rival': best_rival,
            'best_rival_ape_m': rival_ape,
        },
        'aggregate_ape_improvement_percent': improvement,
        'bootstrap_ci95_m': {'lower': ci_lower, 'upper': ci_upper},
        'completion': completion_check.get('evidence'),
        'maximum_processing_rtf': max(rtf_values_by_system['ours']),
        'maximum_processing_rtf_by_system': {
            system: max(values) for system, values in sorted(rtf_values_by_system.items())},
        'rss': {
            'best_rival': best_rival, 'ratio': rss_ratio,
            'maximum_ratio': rss_limit,
            'pass': rss_row.get('pass') is True,
        },
        'map': {'pass': map_check.get('pass') is True},
        'thresholds': scope['thresholds'],
    }


def _authorization_failure_log(authorization: Mapping[str, Any]) -> Mapping[str, Any]:
    auth = authorization.get('authorization', authorization)
    failure_log = auth.get('failure_log') if isinstance(auth, Mapping) else None
    if (not isinstance(failure_log, Mapping) or
            failure_log.get('complete') is not True or
            not isinstance(failure_log.get('events'), list)):
        raise PublicationError(
            'authorization failure_log must be complete and list-valued')
    for index, event in enumerate(failure_log['events']):
        if not isinstance(event, Mapping):
            raise PublicationError(f'authorization failure event {index} is malformed')
    return failure_log


def _validate_failure_artifact_bytes(
        data: bytes, authorization: Mapping[str, Any]) -> None:
    """Require canonical bytes containing exactly the sealed failure log."""
    failure_log = _authorization_failure_log(authorization)
    expected = {
        'schema_version': FAILURE_ARTIFACT_SCHEMA_VERSION,
        'failure_log': failure_log,
    }
    if data != _canonical_failure_artifact_bytes(failure_log):
        parsed = _strict_json_document(data, 'global failure artifact')
        if parsed != expected:
            raise PublicationError(
                'global failure artifact does not exactly match authorization failure_log')
        raise PublicationError('global failure artifact is not canonical JSON')
    parsed = _strict_json_document(data, 'global failure artifact')
    if parsed != expected:
        raise PublicationError(
            'global failure artifact does not exactly match authorization failure_log')


def _failure_ledger(result: Mapping[str, Any], authorization: Mapping[str, Any]) -> list[Any]:
    """Return evaluator and complete authorization failure projections."""
    failures: list[Any] = []
    declared = result.get('failure_ledger')
    if declared is not None:
        if not isinstance(declared, list):
            raise PublicationError('recomputed failure ledger is malformed')
        failures.extend(declared)
    for error in result.get('errors', []):
        failures.append({'kind': 'error', 'detail': error})
    checks = result.get('checks')
    if isinstance(checks, Mapping):
        for name in sorted(checks):
            row = checks[name]
            if isinstance(row, Mapping) and row.get('pass') is not True:
                failures.append({'kind': 'check', 'name': name,
                                 'evidence': row.get('evidence')})
    failure_log = _authorization_failure_log(authorization)
    for index, event in enumerate(failure_log['events']):
        failures.append({'kind': 'authorization_failure', 'index': index,
                         'event': event})
    return failures


def _failure_artifact(manifest: Mapping[str, Any],
                      authorization: Mapping[str, Any]) -> dict[str, Any]:
    """Bind the required global failure artifact to authorization identities."""
    entries = manifest.get('artifacts')
    if not isinstance(entries, list):
        raise PublicationError('bundle global artifacts are missing')
    failures = [entry for entry in entries
                if isinstance(entry, Mapping) and entry.get('role') == 'failure']
    if len(failures) != 1:
        raise PublicationError('bundle must contain exactly one global failure artifact')
    entry = failures[0]
    path = _text(entry.get('path'), 'global failure artifact path')
    if _contains_gt(path):
        raise PublicationError('global failure artifact path is forbidden')
    digest = _sha(entry.get('sha256'), 'global failure artifact SHA')
    size = entry.get('size_bytes')
    if not isinstance(size, int) or isinstance(size, bool) or size < 0:
        raise PublicationError('global failure artifact size is malformed')
    failure_log = _authorization_failure_log(authorization)
    identity_hashes: set[str] = set()
    candidates = [failure_log, *failure_log.get('events', [])]
    for candidate in candidates:
        for key in ('failure_artifact_sha256', 'artifact_sha256', 'sha256'):
            value = candidate.get(key)
            if value is not None:
                identity_hashes.add(_sha(value, f'authorization failure {key}'))
    if identity_hashes and digest not in identity_hashes:
        raise PublicationError(
            'global failure artifact SHA does not match authorization failure identity')
    return {'path': path, 'size_bytes': size, 'sha256': digest}


def _reopen_failure_artifact(bundle_root: Path, descriptor: Mapping[str, Any],
                             authorization: Mapping[str, Any]) -> None:
    path_value = descriptor.get('path')
    if (not isinstance(path_value, str) or '\\' in path_value or
            any(part in {'', '.', '..'} for part in PurePosixPath(path_value).parts) or
            PurePosixPath(path_value).is_absolute()):
        raise PublicationError('global failure artifact path is not normalized relative text')
    path = bundle_root.joinpath(*PurePosixPath(path_value).parts)
    data = _read_bytes(path, 'global failure artifact')
    if (len(data) != descriptor['size_bytes'] or
            _sha_bytes(data) != descriptor['sha256']):
        raise PublicationError('global failure artifact bytes do not match manifest identity')
    _validate_failure_artifact_bytes(data, authorization)


def _check_reopened_failure_artifact(result: Mapping[str, Any],
                                     expected: Mapping[str, Any]) -> None:
    reopened = result.get('artifacts')
    if not isinstance(reopened, list):
        raise PublicationError('bundle verifier did not return reopened artifacts')
    matches = [item for item in reopened
               if isinstance(item, Mapping) and item.get('role') == 'failure']
    if len(matches) != 1:
        raise PublicationError('bundle verifier did not reopen exactly one failure artifact')
    observed = matches[0]
    for key in ('path', 'size_bytes', 'sha256'):
        if observed.get(key) != expected.get(key):
            raise PublicationError(
                f'bundle verifier failure artifact identity mismatch: {key}')


def _render(scope: Mapping[str, Any], receipt_sha: str, hashes: Mapping[str, str],
            failures: list[Any], caveats: list[str], command: list[str],
            result: Mapping[str, Any]) -> str:
    metrics = _metric_projection(result['suite'], result['scored_evidence'], scope)
    thresholds_json = json.dumps(scope['thresholds'], sort_keys=True,
                                 separators=(',', ':'))
    confidence_json = json.dumps(scope['confidence_interval'], sort_keys=True,
                                 separators=(',', ':'))
    completion_json = json.dumps(metrics['completion'], sort_keys=True,
                                 separators=(',', ':'))
    rtf_json = json.dumps(metrics['maximum_processing_rtf_by_system'],
                          sort_keys=True, separators=(',', ':'))
    rss_status = 'PASS' if metrics['rss']['pass'] else 'FAIL'
    map_status = 'PASS' if metrics['map']['pass'] else 'FAIL'
    lines = [
        '# Competitive claim publication',
        '',
        'Claim eligibility was established from the sealed scope below; this '
        'fragment contains no unstated datasets or systems.',
        '',
        f'- Receipt SHA-256: `{receipt_sha}`',
        f'- Systems: `{", ".join(scope["systems"])}`',
        f'- Datasets: `{", ".join(scope["datasets"])}`',
        f'- Runs per system/dataset: `{scope["run_count"]}`',
        f'- Revisions: `{json.dumps(scope["revisions"], sort_keys=True, separators=(",", ":"))}`',
        f'- Metrics: `{json.dumps(scope["metrics"], sort_keys=True, separators=(",", ":"))}`',
        f'- Thresholds: `{thresholds_json}`',
        f'- Confidence interval: `{confidence_json}`',
        f'- Aggregate APE: ours `{metrics["aggregate_ape_m"]["ours"]:.12g}` m; '
        f'{metrics["aggregate_ape_m"]["best_rival"]} '
        f'`{metrics["aggregate_ape_m"]["best_rival_ape_m"]:.12g}` m; '
        f'improvement `{metrics["aggregate_ape_improvement_percent"]:.12g}%`',
        f'Within this fixed scope, ours is `{metrics["aggregate_ape_improvement_percent"]:.12g}%` '
        f'lower aggregate APE than pinned best rival '
        f'`{metrics["aggregate_ape_m"]["best_rival"]}`; '
        f'95% CI lower bound is `{metrics["bootstrap_ci95_m"]["lower"]:.12g}` m '
        'above zero. This is not a universal/all-datasets claim.',
        f'- 95% CI (m): lower `{metrics["bootstrap_ci95_m"]["lower"]:.12g}`, '
        f'upper `{metrics["bootstrap_ci95_m"]["upper"]:.12g}`',
        f'- Completion: `{completion_json}`',
        f'- Ours maximum processing RTF: `{metrics["maximum_processing_rtf"]:.12g}`',
        f'- Maximum processing RTF by system: `{rtf_json}`',
        f'- RSS ratio: `{metrics["rss"]["ratio"]:.12g}` / limit '
        f'`{metrics["rss"]["maximum_ratio"]:.12g}` '
        f'({rss_status})',
        f'- Map gate: `{map_status}`',
        '',
        'Evidence hashes:',
    ]
    for key in sorted(hashes):
        lines.append(f'- `{key}`: `{hashes[key]}`')
    lines.extend(['', 'Failure results (complete evaluator projection):'])
    if failures:
        lines.extend(f'- `{json.dumps(item, sort_keys=True, separators=(",", ":"))}`'
                     for item in failures)
    else:
        lines.append('- none recorded by the sealed suite receipt')
    lines.extend(['', 'Caveats:'])
    lines.extend(f'- {item}' for item in caveats)
    lines.extend(['', 'Reproduction command:', '', '```bash',
                  ' '.join(shlex.quote(item) for item in command), '```', ''])
    return '\n'.join(lines)


def _receipt_identity_payload(receipt: Mapping[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in receipt.items()
            if key not in {'receipt_sha256', 'rendered_markdown_sha256'}}


def _seal_output(output_root: Path, markdown: str, receipt: Mapping[str, Any]) -> None:
    output_root = output_root.absolute()
    if output_root.exists() or output_root.is_symlink():
        raise PublicationError(f'refusing to overwrite publication root: {output_root}')
    output_root.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f'.{output_root.name}.staging-',
                                    dir=str(output_root.parent)))
    try:
        (staging / 'claim.md').write_text(markdown, encoding='utf-8')
        (staging / 'publication_receipt.json').write_text(
            json.dumps(receipt, sort_keys=True, indent=2) + '\n', encoding='utf-8')
        for path in staging.iterdir():
            path.chmod(0o444)
        staging.chmod(0o555)
        os.replace(staging, output_root)
        output_root.chmod(0o555)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def publish(spec_path: Path, output_root: Path) -> dict[str, Any]:
    spec_value, spec_sha = _doc(spec_path, 'publication spec')
    if not isinstance(spec_value, Mapping):
        raise PublicationError('publication spec must be a mapping')
    _validate_schema(spec_value)
    if spec_value.get('schema_version') != SCHEMA_VERSION or \
            spec_value.get('publication_kind') != PUBLICATION_KIND:
        raise PublicationError('publication spec identity is invalid')
    repository_root = Path(spec_value['repository_root'])
    if not repository_root.is_absolute() or not repository_root.is_dir():
        raise PublicationError('repository_root must be an existing absolute directory')
    repository_root = repository_root.resolve(strict=True)
    profile, profile_file_sha, profile_path = _ref(
        spec_value['profile'], 'profile', base=repository_root)
    if not isinstance(profile, Mapping):
        raise PublicationError('profile document must be a mapping')
    profile_sha = canonical_profile_sha256(profile)
    scored_evidence, evidence_sha, _ = _ref(
        spec_value['scored_evidence'], 'scored_evidence', base=repository_root)
    if not isinstance(scored_evidence, Mapping):
        raise PublicationError('scored evidence document must be a mapping')
    contract = profile.get('competitive_slam_profile', profile)
    recomputed = evaluate_evidence_v2(
        dict(scored_evidence), dict(contract), require_bundle=True)
    if not isinstance(recomputed, Mapping):
        raise PublicationError('suite evaluator returned a non-mapping')
    derived_scope, derived_scope_sha = _derive_scope(
        profile, scored_evidence, recomputed)
    supplied_scope, supplied_scope_sha = _check_scope(spec_value['scope'])
    if (_scope_payload(supplied_scope) != _scope_payload(derived_scope) or
            supplied_scope_sha != derived_scope_sha):
        raise PublicationError(
            'publication scope does not match profile/evaluated evidence')
    scope, scope_sha = derived_scope, derived_scope_sha
    suite, suite_sha, _ = _ref(spec_value['suite_receipt'], 'suite_receipt', base=repository_root)
    execution, execution_sha, _ = _ref(
        spec_value['execution_receipt'], 'execution_receipt', base=repository_root)
    auth_value = spec_value.get('fresh_holdout_authorization')
    if auth_value is None:
        raise PublicationError('fresh_holdout_authorization reference is required')
    authorization, authorization_sha, _ = _ref(
        auth_value, 'fresh_holdout_authorization', base=repository_root)
    bundle = spec_value['evidence_bundle']
    bundle_root = _safe_path(Path(bundle['root']), 'evidence_bundle.root')
    if not bundle_root.is_dir() or bundle_root.resolve() != bundle_root:
        raise PublicationError('evidence_bundle.root must be a canonical directory')
    manifest_path = Path(bundle['manifest_path'])
    if manifest_path.is_absolute() or any(part in {'', '.', '..'} for part in manifest_path.parts):
        raise PublicationError('evidence_bundle.manifest_path must be normalized relative text')
    manifest_path = bundle_root / PurePosixPath(manifest_path.as_posix())
    manifest_value, manifest_file_sha = _doc(manifest_path, 'bundle manifest')
    declared_bundle_sha = _sha(bundle['manifest_sha256'], 'evidence_bundle.manifest_sha256')
    declared_bundle_file_sha = _sha(
        bundle['manifest_file_sha256'], 'evidence_bundle.manifest_file_sha256')
    if manifest_file_sha != declared_bundle_file_sha:
        raise PublicationError('bundle manifest file SHA does not match spec')
    if not isinstance(manifest_value, Mapping):
        raise PublicationError('bundle manifest must be a mapping')
    if canonical_manifest_sha256(manifest_value) != manifest_value.get('manifest_sha256'):
        raise PublicationError('bundle manifest canonical self hash is invalid')
    if manifest_value.get('manifest_sha256') != declared_bundle_sha:
        raise PublicationError('bundle manifest canonical hash differs from spec')
    _require_suite_receipt(
        suite, recomputed, scope_sha, declared_bundle_sha, profile_sha,
        profile_file_sha, evidence_sha)
    if not isinstance(execution, Mapping):
        raise PublicationError('execution receipt must be a mapping')
    execution_result = evaluate_execution_selection(
        dict(execution), dict(profile), root=repository_root)
    _required_result(execution_result, 'execution selection')
    if execution_result.get('claim_scope_sha256') not in {None, scope_sha}:
        raise PublicationError('execution verifier returned a mismatched scope identity')
    policy = contract.get('evidence_gate_v2', {}).get(
        'fresh_holdout_authorization', {}) if isinstance(contract, Mapping) else {}
    authorization_result = verify_fresh_holdout_authorization(
        authorization, policy=policy, profile=profile,
        expected_profile_sha256=profile_sha)
    _required_result(authorization_result, 'fresh holdout authorization')
    if authorization_result.get('claim_eligible') is not True:
        raise PublicationError('fresh holdout authorization is not claim eligible')
    manifest_authorization = manifest_value.get('authorization')
    document_authorization = authorization.get('authorization') \
        if isinstance(authorization, Mapping) else None
    if (not isinstance(manifest_authorization, Mapping) or
            manifest_authorization != document_authorization):
        raise PublicationError(
            'bundle authorization is not byte-identical to fresh authorization')
    failure_artifact = _failure_artifact(manifest_value, authorization)
    bundle_result = verify_evidence_bundle(
        bundle_root, manifest_path, contract=profile.get('competitive_slam_profile', profile),
        profile=profile, expected_profile_sha256=profile_sha,
        expected_manifest_sha256=declared_bundle_file_sha,
        expected_revision_by_system=scope['revisions'])
    _required_result(bundle_result, 'evidence bundle')
    if bundle_result.get('manifest_sha256') != declared_bundle_file_sha:
        raise PublicationError('bundle verifier returned a mismatched manifest file hash')
    _check_reopened_failure_artifact(bundle_result, failure_artifact)
    _reopen_failure_artifact(bundle_root, failure_artifact, authorization)
    rival_result = verify_rival_source_closure(profile, root=repository_root,
                                               receipt=execution)
    _required_result(rival_result, 'rival source closure')
    dataset_result = verify_dataset_source_closure(profile, root=repository_root)
    _required_result(dataset_result, 'dataset source closure')
    _manifest_coverage(manifest_value, scope)
    hashes = {
        'bundle_manifest': declared_bundle_sha,
        'failure_artifact': failure_artifact['sha256'],
        'execution_receipt': execution_sha,
        'fresh_holdout_authorization': authorization_sha,
        'scored_evidence': evidence_sha,
        'profile_file': profile_file_sha,
        'profile_canonical': profile_sha,
        'suite_receipt': suite_sha,
        'spec': spec_sha,
    }
    sealed_failures = _failure_ledger(recomputed, authorization)
    failures = spec_value['failures']
    if failures != sealed_failures:
        raise PublicationError(
            'publication failures do not exactly match recomputed failure ledger')
    recomputed_caveats = recomputed.get('caveats', [])
    if recomputed_caveats is None:
        recomputed_caveats = []
    if not isinstance(recomputed_caveats, list):
        raise PublicationError('recomputed caveat ledger is malformed')
    caveats = sorted(set(
        [_text(item, 'caveat') for item in spec_value['caveats']] +
        [_text(item, 'recomputed caveat') for item in recomputed_caveats] +
        [STANDARD_CAVEAT]))
    if any(item not in caveats for item in recomputed_caveats):
        raise PublicationError('recomputed caveat ledger is not projected')
    command = [_text(item, 'reproduction_command item')
               for item in spec_value['reproduction_command']]
    result_summary = _metric_projection(recomputed, scored_evidence, scope)
    receipt_base = {
        'schema_version': SCHEMA_VERSION,
        'receipt_kind': RECEIPT_KIND,
        'status': 'PASS',
        'claim_eligible': True,
        'spec_sha256': spec_sha,
        'scope': scope,
        'scope_sha256': scope_sha,
        'evidence_hashes': hashes,
        'result_summary': result_summary,
        'failure_artifact': failure_artifact,
        'failures': failures,
        'caveats': caveats,
        'reproduction_command': command,
    }
    receipt_sha = canonical_json_sha256(_receipt_identity_payload(receipt_base))
    render_context = {
        'suite': recomputed,
        'scored_evidence': scored_evidence,
    }
    markdown = _render(scope, receipt_sha, hashes, failures, caveats, command,
                       render_context)
    receipt = dict(receipt_base)
    receipt['rendered_markdown_sha256'] = _sha_bytes(markdown.encode('utf-8'))
    receipt['receipt_sha256'] = receipt_sha
    _seal_output(output_root, markdown, receipt)
    return receipt


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = publish(args.spec, args.output_root)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(json.dumps({'status': 'FAIL_CLOSED', 'claim_eligible': False,
                          'error': str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
