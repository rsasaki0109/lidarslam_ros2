#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Fail-closed comparative process-tree RSS gate for competitive runs.

This module validates *authoritative* resource receipts before comparing RSS.
The receipt identity is bound to the run's machine, hardware, algorithm
configuration, thread policy, and resource-tool source.  Functional-only
registration-plugin resource records are explicitly ineligible and cannot be
used as benchmark RSS evidence.

The aggregate comparison is deterministic: the maximum valid process-tree RSS
over all repetitions is computed per sequence and the maximum of those values
is the system aggregate.  The existing profile ratio is used for the
aggregate best-rival criterion and, when configured, every-rival comparison.
The profile currently has no approved per-sequence serious-regression ceiling;
that gate therefore returns ``NOT_READY`` instead of inventing a threshold.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from typing import Any, Iterable, Mapping


SCHEMA_VERSION = 1
RECEIPT_KIND = 'competitive_process_rss_resource_receipt'
MEASUREMENT_VERSION = 'm6a7-container-memory-v2'
MEASUREMENT_SCOPE = 'container_cgroup_v2_with_pid_rss'
PRIMARY_METRIC = 'aggregate_process_tree_peak_rss_bytes'
PRIMARY_METRIC_DEFINITION = (
    'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted')
SOURCE_KIND = 'benchmark_process_rss_authoritative_v1'
MINIMUM_MATCHED_COMPLETE_RUNS = 3
EXPECTED_RELEASE = 'Release'
_SHA256_RE = re.compile(r'^[0-9a-fA-F]{64}$')
_REVISION_RE = re.compile(r'^[0-9a-fA-F]{40}$')
_FORBIDDEN_MARKERS = (
    'registration-plugin',
    'functional_non_authoritative',
    'non_authoritative_contaminated',
    'timing_authority_non_authoritative',
)


def canonical_json_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(',', ':'), ensure_ascii=True
    ).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _finite(value: Any, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{label} must be numeric')
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{label} must be numeric') from exc
    if not math.isfinite(number) or (positive and number <= 0.0):
        raise ValueError(f'{label} must be finite and {"positive" if positive else "non-negative"}')
    return number


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or _SHA256_RE.fullmatch(value) is None:
        raise ValueError(f'{label} must be a 64-hex SHA-256')
    return value.lower()


def _revision(value: Any, label: str) -> str:
    if not isinstance(value, str) or _REVISION_RE.fullmatch(value) is None:
        raise ValueError(f'{label} must be a pinned 40-hex revision')
    return value.lower()


def _thread_hash(value: Any) -> str:
    return canonical_json_sha256(value)


def _resource_value(resource: Mapping[str, Any], receipt: Mapping[str, Any]) -> int:
    values: list[Any] = []
    for source in (resource, receipt):
        for key in (PRIMARY_METRIC, 'process_tree_peak_rss_bytes', 'peak_rss_bytes'):
            if key in source:
                values.append(source[key])
    if not values:
        raise ValueError('resource receipt peak RSS value is missing')
    normalized: list[int] = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError('resource receipt peak RSS must be a positive integer byte count')
        normalized.append(value)
    if len(set(normalized)) != 1:
        raise ValueError('resource receipt contains duplicate RSS values with different bytes')
    return normalized[0]


def _resource_document(run: Mapping[str, Any]) -> Mapping[str, Any] | None:
    for key in ('resource_evidence', 'resource_receipt', 'resource'):
        value = run.get(key)
        if isinstance(value, Mapping):
            return value
    return None


def _validate_complete_run(run: Mapping[str, Any]) -> None:
    """Require a complete, successful run before reopening its RSS receipt."""
    required = (
        'complete', 'process_exit_status', 'trajectory_complete',
        'sequence_failure', 'catastrophic_failure', 'verified_false_loops')
    missing = [field for field in required if field not in run]
    if missing:
        raise ValueError(
            'complete run fields are missing: ' + ','.join(missing))
    if (run.get('complete') is not True or
            run.get('process_exit_status') != 0 or
            run.get('trajectory_complete') is not True or
            run.get('sequence_failure') is not False or
            run.get('catastrophic_failure') is not False or
            run.get('verified_false_loops') != 0):
        raise ValueError('resource RSS run is not a complete successful run')


def _ratio_policy(policy: Mapping[str, Any]) -> tuple[float, str | None]:
    """Return the sole preregistered RSS ratio and any policy error.

    ``max_peak_rss_ratio_vs_best_rival`` is the only accepted source.  A
    nested aggregate number is deliberately not treated as an independent
    threshold because that would permit aggregate/per-sequence drift.
    """
    aggregate = policy.get('aggregate')
    aggregate = aggregate if isinstance(aggregate, Mapping) else {}
    canonical = policy.get('max_peak_rss_ratio_vs_best_rival')
    legacy = aggregate.get('maximum_ratio')
    if canonical is None:
        return 1.0, (
            'max_peak_rss_ratio_vs_best_rival is the required canonical '
            'RSS threshold source')
    if legacy is not None:
        try:
            if not math.isclose(float(canonical), float(legacy),
                                rel_tol=0.0, abs_tol=0.0):
                return 1.0, (
                    'aggregate.maximum_ratio must equal the canonical '
                    'max_peak_rss_ratio_vs_best_rival')
        except (TypeError, ValueError):
            return 1.0, (
                'aggregate.maximum_ratio must equal the canonical '
                'max_peak_rss_ratio_vs_best_rival')
    try:
        return _finite(canonical, 'memory max_peak_rss_ratio_vs_best_rival',
                       positive=True), None
    except ValueError as exc:
        return 1.0, str(exc)


def _policy_resource_identity(policy: Mapping[str, Any]) -> Mapping[str, Any]:
    value = policy.get('resource_identity')
    return value if isinstance(value, Mapping) else {}


def _forbidden_functional_resource(resource: Mapping[str, Any],
                                   receipt: Mapping[str, Any]) -> str | None:
    source_kind = resource.get('source_kind')
    if source_kind != SOURCE_KIND:
        return 'resource source_kind is not the authoritative benchmark kind'
    if resource.get('functional_only') is True or \
            resource.get('performance_gate_eligible') is False:
        return 'functional-only resource evidence is not benchmark-eligible'
    timing_authority = resource.get('timing_authority')
    if isinstance(timing_authority, str) and (
            'NON_AUTHORITATIVE' in timing_authority.upper() or
            'CONTAMINATED' in timing_authority.upper()):
        return 'non-authoritative contaminated timing cannot enter RSS gate'
    marker_text = json.dumps(
        {'resource': resource, 'receipt': receipt},
        sort_keys=True, ensure_ascii=True).lower()
    if any(marker in marker_text for marker in _FORBIDDEN_MARKERS):
        return 'functional/plugin resource lineage is forbidden'
    return None


def validate_resource_receipt(
        run: Mapping[str, Any], *, system: str, sequence: str, run_index: int,
        policy: Mapping[str, Any],
        provenance: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Validate one run's authoritative resource receipt without scoring."""
    result: dict[str, Any] = {
        'system': system, 'sequence': sequence, 'run_index': run_index,
        'pass': False,
    }
    resource = _resource_document(run)
    if resource is None:
        result['reason'] = 'resource_receipt_missing'
        return result
    receipt = resource.get('receipt', resource)
    if not isinstance(receipt, Mapping):
        result['reason'] = 'resource_receipt_payload_missing'
        return result
    try:
        _validate_complete_run(run)
        receipt_sha = _sha(
            resource.get('receipt_sha256', resource.get('evidence_sha256')),
            'resource.receipt_sha256')
        result['receipt_sha256'] = receipt_sha
        forbidden_reason = _forbidden_functional_resource(resource, receipt)
        if forbidden_reason:
            raise ValueError(forbidden_reason)
        if resource.get('status') not in {'PASS', 'pass'}:
            raise ValueError('resource receipt status is not PASS')
        expected_version = policy.get('measurement_version', MEASUREMENT_VERSION)
        expected_scope = policy.get('measurement_scope', MEASUREMENT_SCOPE)
        expected_metric = policy.get('primary_metric', PRIMARY_METRIC)
        expected_definition = policy.get(
            'primary_metric_definition', PRIMARY_METRIC_DEFINITION)
        for label, observed, expected in (
                ('measurement_version', receipt.get('measurement_version'), expected_version),
                ('measurement_scope', receipt.get('measurement_scope'), expected_scope),
                ('primary_metric', receipt.get('primary_metric'), expected_metric),
                ('primary_metric_definition', receipt.get('primary_metric_definition'),
                 expected_definition)):
            if observed != expected:
                raise ValueError(f'resource {label} does not match profile')
        if receipt.get('status') != 'pass':
            raise ValueError('embedded memory receipt status is not pass')
        peak = _resource_value(resource, receipt)
        result['peak_rss_bytes'] = peak
        runtime = run.get('runtime')
        if not isinstance(runtime, Mapping):
            raise ValueError('run runtime is missing for RSS binding')
        runtime_mb = _finite(runtime.get('peak_rss_mb'),
                              'runtime.peak_rss_mb', positive=True)
        expected_mb = peak / (1024.0 * 1024.0)
        if not math.isclose(runtime_mb, expected_mb, rel_tol=1e-9, abs_tol=1e-6):
            raise ValueError('runtime.peak_rss_mb does not match resource receipt bytes')
        identity = resource.get('identity')
        if not isinstance(identity, Mapping):
            raise ValueError('resource identity is missing')
        identity_fields = (
            'tool_revision', 'sampler_script_sha256',
            'memory_helper_script_sha256', 'config_sha256',
            'thread_policy_sha256', 'hardware_fingerprint', 'machine_id',
            'release')
        for field in identity_fields:
            if field not in identity or not isinstance(identity[field], str) or not identity[field]:
                raise ValueError(f'resource identity.{field} is missing')
        _sha(identity['sampler_script_sha256'], 'resource identity sampler_script_sha256')
        _sha(identity['memory_helper_script_sha256'],
             'resource identity memory_helper_script_sha256')
        _sha(identity['config_sha256'], 'resource identity config_sha256')
        _sha(identity['thread_policy_sha256'],
             'resource identity thread_policy_sha256')
        _sha(identity['hardware_fingerprint'],
             'resource identity hardware_fingerprint')
        if identity['release'] != EXPECTED_RELEASE:
            raise ValueError(
                f'resource identity.release must be exactly {EXPECTED_RELEASE}')
        expected_tool_revision = _policy_resource_identity(policy).get('tool_revision')
        if expected_tool_revision is not None and identity['tool_revision'] != expected_tool_revision:
            raise ValueError('resource tool revision drift')
        for policy_field, identity_field in (
                ('sampler_script_sha256', 'sampler_script_sha256'),
                ('memory_helper_script_sha256', 'memory_helper_script_sha256')):
            expected_hash = _policy_resource_identity(policy).get(policy_field)
            if expected_hash is not None and identity[identity_field].lower() != str(expected_hash).lower():
                raise ValueError(f'resource {identity_field} drift')
        if provenance is not None:
            if identity['machine_id'] != provenance.get('machine_id'):
                raise ValueError('resource machine_id does not match run provenance')
            if identity['hardware_fingerprint'].lower() != str(
                    provenance.get('hardware_fingerprint', '')).lower():
                raise ValueError('resource hardware fingerprint does not match provenance')
            if identity['config_sha256'].lower() != str(
                    provenance.get('config_sha256', '')).lower():
                raise ValueError('resource config identity does not match provenance')
            thread_policy = provenance.get('thread_policy')
            if not isinstance(thread_policy, Mapping) or \
                    identity['thread_policy_sha256'].lower() != _thread_hash(thread_policy):
                raise ValueError('resource thread-policy identity does not match provenance')
            if provenance.get('release') != identity['release']:
                raise ValueError('resource release identity does not match provenance')
        if receipt.get('ground_truth_content_opened') is not False or \
                receipt.get('scorer_invoked') is not False:
            raise ValueError('resource receipt is not explicitly GT-blind')
        cgroup_events = receipt.get('cgroup_events')
        if not isinstance(cgroup_events, Mapping):
            raise ValueError('resource cgroup event identity is missing')
        if cgroup_events.get('oom_free') is not True:
            raise ValueError('resource receipt is not OOM-free')
        oom_delta = cgroup_events.get('oom_delta')
        if not isinstance(oom_delta, Mapping) or not oom_delta or \
                any(value != 0 for value in oom_delta.values()):
            raise ValueError('resource receipt contains missing/non-zero OOM delta')
        result['identity'] = {
            field: identity[field] for field in identity_fields}
        result['pass'] = True
        return result
    except (TypeError, ValueError) as exc:
        result['reason'] = str(exc)
        return result


def _expected_keys(sequences: Iterable[str], repetitions: int) -> set[tuple[str, int]]:
    return {(sequence, index) for sequence in sequences
            for index in range(1, repetitions + 1)}


def evaluate_memory_gate(
        runs_by_system: Mapping[str, Iterable[Mapping[str, Any]]],
        *, required_systems: Iterable[str], expected_sequences: Iterable[str],
        repetitions: int, policy: Mapping[str, Any],
        best_rival: str | None,
        provenance_by_system: Mapping[str, Mapping[str, Any]] | None = None,
        claim_requested: bool = True) -> dict[str, Any]:
    """Validate and compare all matched run/sequence RSS receipts."""
    required = list(required_systems)
    sequences = list(expected_sequences)
    errors: list[str] = []
    checks: dict[str, dict[str, Any]] = {}
    if not claim_requested:
        return {
            'schema_version': SCHEMA_VERSION, 'status': 'NOT_REQUESTED',
            'pass': True, 'claim_eligible': False,
            'checks': {'resource_receipt_identity': {'pass': True}},
            'errors': [], 'reason': 'memory gate is report-only until resource evidence is declared',
        }
    if not isinstance(policy, Mapping):
        return {
            'schema_version': SCHEMA_VERSION, 'status': 'NOT_READY',
            'pass': False, 'claim_eligible': False,
            'checks': {'memory_gate_policy': {'pass': False}},
            'errors': ['memory gate policy is missing'],
        }
    try:
        repetition_count = int(repetitions)
    except (TypeError, ValueError):
        repetition_count = 0
    expected = _expected_keys(sequences, repetition_count)
    if len(required) != len(set(required)):
        errors.append('required systems contain duplicates')
    if len(sequences) != len(set(sequences)):
        errors.append('expected sequences contain duplicates')
    if 'ours' not in required:
        errors.append('required systems must include ours')
    minimum_runs = policy.get(
        'minimum_matched_complete_runs', MINIMUM_MATCHED_COMPLETE_RUNS)
    try:
        minimum_runs = int(minimum_runs)
    except (TypeError, ValueError):
        minimum_runs = 0
    if repetition_count < max(MINIMUM_MATCHED_COMPLETE_RUNS, minimum_runs):
        errors.append(
            'RSS gate requires at least three matched complete runs per '
            'system and sequence')
    validated: dict[str, dict[tuple[str, int], dict[str, Any]]] = {}
    seen_receipts: dict[str, str] = {}
    common_tool_identity: tuple[str, str, str] | None = None
    common_environment_identity: tuple[str, str, str, str] | None = None
    config_identity_by_system: dict[str, str] = {}
    for system in required:
        rows = list(runs_by_system.get(system, []))
        by_key: dict[tuple[str, int], Mapping[str, Any]] = {}
        for row in rows:
            sequence = row.get('dataset', row.get('sequence'))
            run_index = row.get('run_index')
            key = (sequence, run_index) if isinstance(sequence, str) and \
                isinstance(run_index, int) and not isinstance(run_index, bool) else None
            if key is None:
                errors.append(f'{system}: resource run key is missing or invalid')
                continue
            if key in by_key:
                errors.append(f'{system}: duplicate resource run key {sequence}/{run_index}')
                continue
            by_key[key] = row
        missing = sorted(expected - set(by_key))
        unexpected = sorted(set(by_key) - expected)
        if missing:
            errors.append(f'{system}: missing resource runs {missing}')
        if unexpected:
            errors.append(f'{system}: unexpected resource runs {unexpected}')
        validated[system] = {}
        for key in sorted(expected):
            row = by_key.get(key)
            if row is None:
                continue
            receipt = validate_resource_receipt(
                row, system=system, sequence=key[0], run_index=key[1],
                policy=policy,
                provenance=(provenance_by_system or {}).get(system))
            validated[system][key] = receipt
            if not receipt.get('pass'):
                errors.append(
                    f'{system}: {key[0]}/{key[1]} resource: {receipt.get("reason")}')
                continue
            receipt_sha = receipt['receipt_sha256']
            prior = seen_receipts.get(receipt_sha)
            if prior is not None:
                errors.append(
                    f'duplicate resource receipt identity: {receipt_sha} reused by {prior} '
                    f'and {system}:{key[0]}/{key[1]}')
            seen_receipts[receipt_sha] = f'{system}:{key[0]}/{key[1]}'
            tool_identity = (
                receipt['identity']['tool_revision'],
                receipt['identity']['sampler_script_sha256'],
                receipt['identity']['memory_helper_script_sha256'])
            if common_tool_identity is None:
                common_tool_identity = tool_identity
            elif tool_identity != common_tool_identity:
                errors.append('resource tool identity differs across systems/runs')
            environment_identity = (
                receipt['identity']['hardware_fingerprint'],
                receipt['identity']['machine_id'],
                receipt['identity']['thread_policy_sha256'],
                receipt['identity']['release'])
            if common_environment_identity is None:
                common_environment_identity = environment_identity
            elif environment_identity != common_environment_identity:
                errors.append(
                    'resource hardware/machine/thread identity differs across '
                    'systems/runs')
            config_identity = receipt['identity']['config_sha256']
            prior_config = config_identity_by_system.get(system)
            if prior_config is None:
                config_identity_by_system[system] = config_identity
            elif prior_config != config_identity:
                errors.append(
                    f'{system}: resource config identity differs across runs')
    matched = bool(validated) and all(
        set(validated.get(system, {})) == expected for system in required)
    checks['matched_run_conditions'] = {
        'pass': matched and bool(expected),
        'expected_keys': sorted([f'{sequence}/{index}' for sequence, index in expected]),
        'provided_keys': {
            system: sorted([f'{sequence}/{index}' for sequence, index in rows])
            for system, rows in validated.items()},
    }
    peaks: dict[str, dict[str, dict[int, int]]] = {
        system: {sequence: {} for sequence in sequences} for system in required}
    for system in required:
        for (sequence, index), receipt in validated.get(system, {}).items():
            if receipt.get('pass'):
                peaks[system][sequence][index] = int(receipt['peak_rss_bytes'])
    per_sequence: dict[str, dict[str, int]] = {
        system: {sequence: max(values.values())
                 for sequence, values in by_sequence.items() if values}
        for system, by_sequence in peaks.items()}
    aggregate = {
        system: max(values.values()) if values else None
        for system, values in per_sequence.items()}
    checks['resource_receipt_identity'] = {
        'pass': not errors and matched,
        'receipt_count': len(seen_receipts),
        'tool_identity': common_tool_identity,
        'environment_identity': common_environment_identity,
        'config_identity_by_system': config_identity_by_system,
    }

    rivals = [system for system in required if system != 'ours']
    aggregate_policy = policy.get('aggregate')
    aggregate_policy = aggregate_policy if isinstance(aggregate_policy, Mapping) else {}
    ratio_limit, ratio_error = _ratio_policy(policy)
    aggregate_ok = ratio_error is None
    aggregate_comparisons: dict[str, Any] = {}
    if ratio_error is not None:
        errors.append(ratio_error)
    if aggregate_policy.get('rival_selection') != 'best_rival':
        errors.append('memory aggregate rival_selection must be best_rival')
        aggregate_ok = False
    if aggregate_policy.get('threshold_source') != (
            'max_peak_rss_ratio_vs_best_rival'):
        errors.append(
            'memory aggregate threshold_source must be '
            'max_peak_rss_ratio_vs_best_rival')
        aggregate_ok = False
    if best_rival not in rivals:
        errors.append('memory aggregate best rival is missing or not pinned')
        aggregate_ok = False
    elif aggregate.get('ours') is None or aggregate.get(best_rival) is None:
        errors.append('memory aggregate RSS is missing for ours or best rival')
        aggregate_ok = False
    else:
        best_ratio = aggregate['ours'] / aggregate[best_rival]
        aggregate_comparisons[best_rival] = {
            'ours_bytes': aggregate['ours'],
            'rival_bytes': aggregate[best_rival],
            'ratio': best_ratio,
            'pass': math.isfinite(best_ratio) and best_ratio <= ratio_limit,
        }
        aggregate_ok = aggregate_ok and aggregate_comparisons[best_rival]['pass']
    every_rival_ok = True
    for rival in rivals:
        if aggregate.get('ours') is None or aggregate.get(rival) is None:
            every_rival_ok = False
            aggregate_comparisons.setdefault(rival, {'pass': False, 'reason': 'RSS missing'})
            continue
        ratio = aggregate['ours'] / aggregate[rival]
        row = aggregate_comparisons.setdefault(rival, {})
        row.update({'ours_bytes': aggregate['ours'], 'rival_bytes': aggregate[rival],
                    'ratio': ratio, 'pass': math.isfinite(ratio) and ratio <= ratio_limit})
        every_rival_ok = every_rival_ok and row['pass']
    if aggregate_policy.get('compare_every_pinned_rival', True) is True:
        aggregate_ok = aggregate_ok and every_rival_ok
    checks['aggregate_best_rival'] = {
        'pass': aggregate_ok and matched,
        'criterion': aggregate_policy.get('rival_selection'),
        'maximum_ratio': ratio_limit,
        'best_rival': best_rival,
        'comparisons': aggregate_comparisons,
        'aggregation': 'max_peak_over_repetitions_and_sequences_v1',
    }

    sequence_policy = policy.get('per_sequence')
    sequence_policy = sequence_policy if isinstance(sequence_policy, Mapping) else {}
    sequence_status = sequence_policy.get('status', 'NOT_READY')
    ceiling = sequence_policy.get('serious_regression_ceiling_ratio')
    threshold_source = sequence_policy.get('threshold_source')
    sequence_rows: dict[str, Any] = {}
    sequence_ok = True
    if (sequence_status != 'READY' or
            threshold_source != 'max_peak_rss_ratio_vs_best_rival'):
        sequence_ok = False
        errors.append(
            'per-sequence serious-regression ceiling must be READY and bound '
            'to max_peak_rss_ratio_vs_best_rival')
        for sequence in sequences:
            sequence_rows[sequence] = {'pass': False, 'status': sequence_status}
    else:
        try:
            ceiling_value = _finite(
                ceiling, 'per-sequence serious_regression_ceiling_ratio',
                positive=True)
            if not math.isclose(ceiling_value, ratio_limit,
                                rel_tol=0.0, abs_tol=0.0):
                raise ValueError(
                    'per-sequence serious-regression ceiling must equal '
                    'max_peak_rss_ratio_vs_best_rival')
        except ValueError as exc:
            errors.append(str(exc))
            sequence_ok = False
            ceiling_value = ratio_limit
        for sequence in sequences:
            comparisons: dict[str, Any] = {}
            ours_value = per_sequence.get('ours', {}).get(sequence)
            row_ok = ours_value is not None
            for rival in rivals:
                rival_value = per_sequence.get(rival, {}).get(sequence)
                if ours_value is None or rival_value is None or rival_value <= 0:
                    row_ok = False
                    comparisons[rival] = {'pass': False, 'reason': 'RSS missing or zero'}
                    continue
                regression = ours_value / rival_value
                comparison_ok = (math.isfinite(regression) and
                                 regression <= ceiling_value)
                comparisons[rival] = {'pass': comparison_ok,
                                      'ratio': regression,
                                      'maximum_ratio': ceiling_value}
                row_ok = row_ok and comparison_ok
            sequence_rows[sequence] = {'pass': row_ok, 'comparisons': comparisons}
            sequence_ok = sequence_ok and row_ok
    checks['per_sequence_serious_regression'] = {
        'pass': sequence_ok and matched,
        'status': sequence_status,
        'threshold_source': threshold_source,
        'serious_regression_ceiling_ratio': ceiling,
        'sequences': sequence_rows,
    }

    uncertainty = policy.get('uncertainty')
    uncertainty = uncertainty if isinstance(uncertainty, Mapping) else {}
    uncertainty_status = uncertainty.get('status', 'NOT_CONFIGURED')
    uncertainty_required = uncertainty.get('required', False) is True
    uncertainty_scope = uncertainty.get('scope')
    rss_ci_claim = uncertainty.get('rss_ci_claim')
    uncertainty_ok = (
        uncertainty_required is False and
        uncertainty_status == 'NOT_APPLICABLE' and
        uncertainty_scope == 'APE_ONLY' and
        rss_ci_claim is False)
    if not uncertainty_ok:
        errors.append(
            'RSS uncertainty/CI policy must explicitly be APE_ONLY with no '
            'RSS CI claim')
    checks['uncertainty'] = {
        'pass': uncertainty_ok,
        'status': uncertainty_status,
        'required': uncertainty_required,
        'scope': uncertainty_scope,
        'rss_ci_claim': rss_ci_claim,
        'method': uncertainty.get('method'),
    }
    passed = not errors and all(row.get('pass') is True for row in checks.values())
    status = 'PASS' if passed else (
        'NOT_READY' if not isinstance(policy.get('per_sequence'), Mapping) or
        sequence_status != 'READY' else 'FAIL_CLOSED')
    return {
        'schema_version': SCHEMA_VERSION,
        'receipt_kind': RECEIPT_KIND,
        'status': status,
        'pass': passed,
        'claim_eligible': passed,
        'errors': errors,
        'checks': checks,
        'per_sequence_peak_rss_bytes': per_sequence,
        'aggregate_peak_rss_bytes': aggregate,
        'resource_receipts_reopened': len(seen_receipts),
    }
