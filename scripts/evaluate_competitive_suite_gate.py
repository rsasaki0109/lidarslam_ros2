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

"""Require every frozen holdout to pass both competitive SLAM tracks."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import re
import sys
from typing import Any, Mapping

import yaml

# When this canonical source script is invoked directly (``python
# scripts/evaluate_competitive_suite_gate.py``), Python places ``scripts/``
# rather than the checkout root on ``sys.path``.  Bootstrap only the exact
# source checkout that owns this script; an installed package or an ambiguous
# parent is never accepted as a fallback.  Installed entry points import the
# package normally and do not take this branch.
_SCRIPT_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if (
        (_SCRIPT_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file()
        and (_SCRIPT_SOURCE_ROOT / 'scripts' / 'benchmark_phase_contract.py').is_file()
        and str(_SCRIPT_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SCRIPT_SOURCE_ROOT))

try:
    from lidarslam_benchmark_tools.competitive_identity_hash import (
        PROFILE_CANONICAL_HASH_KIND, canonical_profile_sha256)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.competitive_identity_hash import (  # type: ignore[no-redef]
        PROFILE_CANONICAL_HASH_KIND, canonical_profile_sha256)

try:
    from lidarslam_benchmark_tools.verify_competitive_evidence_bundle import (
        verify_evidence_bundle)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.verify_competitive_evidence_bundle import (  # type: ignore[no-redef]
        verify_evidence_bundle)

try:
    from lidarslam_benchmark_tools.competitive_holdout_authorization import (
        verify_fresh_holdout_authorization)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.competitive_holdout_authorization import (  # type: ignore[no-redef]
        verify_fresh_holdout_authorization)

try:
    from lidarslam_benchmark_tools.competitive_memory_gate import evaluate_memory_gate
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.competitive_memory_gate import (  # type: ignore[no-redef]
        evaluate_memory_gate)

try:
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (
        current_rival_source_closure_identity,
        validate_rival_source_closure_receipt_identity,
        verify_rival_source_closure)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (  # type: ignore[no-redef]
        current_rival_source_closure_identity,
        validate_rival_source_closure_receipt_identity,
        verify_rival_source_closure)

try:
    from lidarslam_benchmark_tools.check_competitive_dataset_source_closure import (
        verify_dataset_source_closure)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.check_competitive_dataset_source_closure import (  # type: ignore[no-redef]
        verify_dataset_source_closure)

try:
    from lidarslam_benchmark_tools.check_competitive_execution_selection import (
        evaluate as evaluate_execution_selection)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.check_competitive_execution_selection import (  # type: ignore[no-redef]
        evaluate as evaluate_execution_selection)


try:
    from lidarslam_benchmark_tools import package_root
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    def package_root() -> Path:
        return Path(__file__).resolve().parents[1]


ROOT = package_root()
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
REQUIRED_TRACKS = {
    'glim_cpu_lidar_imu', 'fast_livo2_lidar_imu_visual'}
_SHA256_RE = re.compile(r'^[0-9a-fA-F]{64}$')
_V2_BOOTSTRAP_SEED = 20260821
_V2_BOOTSTRAP_SAMPLES = 10000
_MISSING = object()


def evaluate(gates: list[dict[str, Any]], contract: dict[str, Any]) -> dict[str, Any]:
    slots = contract['datasets']['holdout_slots']
    assigned = {
        slot['sequence'] for slot in slots.values()
        if slot.get('status') in {'assigned_inputs_pending_hash', 'frozen'}}
    by_pair = {(gate.get('sequence'), gate.get('track')): gate for gate in gates}
    expected = {(sequence, track) for sequence in assigned
                for track in REQUIRED_TRACKS}
    missing = sorted(expected.difference(by_pair))
    unexpected = sorted(set(by_pair).difference(expected))
    failed = sorted(pair for pair in expected
                    if pair in by_pair and not by_pair[pair].get('pass'))
    minimum = int(contract['win_policy']['minimum_holdout_wins'])
    passed_sequences = sorted(
        sequence for sequence in assigned
        if all(by_pair.get((sequence, track), {}).get('pass')
               for track in REQUIRED_TRACKS))
    checks = {
        'all_holdout_inputs_frozen': all(
            slot.get('status') == 'frozen' for slot in slots.values()),
        'all_expected_track_gates_present': not missing and not unexpected,
        'all_track_gates_pass': not missing and not failed,
        'minimum_complete_holdout_wins': len(passed_sequences) >= minimum,
    }
    return {
        'schema_version': 1, 'pass': all(checks.values()), 'checks': checks,
        'assigned_sequences': sorted(assigned),
        'required_tracks': sorted(REQUIRED_TRACKS),
        'expected_gate_count': len(expected), 'provided_gate_count': len(gates),
        'missing_gates': missing, 'unexpected_gates': unexpected,
        'failed_gates': failed, 'complete_holdout_wins': passed_sequences,
        'minimum_complete_holdout_wins': minimum,
    }


def _v2_get(document: Any, path: str, default: Any = _MISSING) -> Any:
    value = document
    for key in path.split('.'):
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def _v2_jsonable(value: Any) -> Any:
    """Convert validator diagnostics to JSON/YAML-safe values.

    ``_MISSING`` is an internal control-flow sentinel and must never escape
    through a machine receipt.  The validator does not mutate evidence; this
    copy is only applied to diagnostic data returned to callers.
    """
    if value is _MISSING:
        return None
    if isinstance(value, dict):
        return {str(key): _v2_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_v2_jsonable(item) for item in value]
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


def _v2_sha256(value: Any, field: str) -> str:
    if not isinstance(value, str) or not _SHA256_RE.fullmatch(value):
        raise ValueError(f'{field} must be a 64-character SHA-256 hex digest')
    return value.lower()


def _v2_finite(value: Any, field: str, *, positive: bool = False,
               maximum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{field} must be numeric')
    number = float(value)
    if not math.isfinite(number) or (number <= 0.0 if positive else number < 0.0):
        operator = '> 0' if positive else '>= 0'
        raise ValueError(f'{field} must be finite and {operator}')
    if maximum is not None and number > maximum:
        raise ValueError(f'{field} must be <= {maximum}')
    return number


def _v2_hierarchical_bootstrap(
        run_values: dict[str, dict[str, list[float]]], seed: int,
        samples: int) -> dict[str, Any]:
    """Compute a fixed-seed two-stage hierarchical 95% bootstrap interval.

    The outer resampling unit is a dataset cluster.  For every selected
    dataset, ours and the rival are independently resampled within that
    dataset and reduced to a run mean before taking their difference.  This
    preserves repeated-run uncertainty without pretending that the runs are
    independent datasets.
    """
    if len(run_values) < 2:
        raise ValueError('at least two holdout datasets are required for CI')
    if samples < 100:
        raise ValueError('bootstrap_samples must be >= 100')
    names = sorted(run_values)
    if any(not run_values[name].get('ours') or not run_values[name].get('rival')
           for name in names):
        raise ValueError('each dataset must have ours and rival run arrays')
    if any(len(run_values[name]['ours']) != len(run_values[name]['rival'])
           or len(run_values[name]['ours']) < 3 for name in names):
        raise ValueError('each dataset must have at least three paired run values')
    values = [sum(run_values[name]['rival']) / len(run_values[name]['rival']) -
              sum(run_values[name]['ours']) / len(run_values[name]['ours'])
              for name in names]
    generator = random.Random(seed)
    draws: list[float] = []
    for _ in range(samples):
        selected = [names[generator.randrange(len(names))] for _ in names]
        differences: list[float] = []
        for name in selected:
            ours = run_values[name]['ours']
            rival = run_values[name]['rival']
            ours_mean = sum(ours[generator.randrange(len(ours))]
                            for _ in ours) / len(ours)
            rival_mean = sum(rival[generator.randrange(len(rival))]
                             for _ in rival) / len(rival)
            differences.append(rival_mean - ours_mean)
        draws.append(sum(differences) / len(differences))
    draws.sort()

    def percentile(fraction: float) -> float:
        position = fraction * (len(draws) - 1)
        lower = math.floor(position)
        upper = math.ceil(position)
        if lower == upper:
            return draws[lower]
        weight = position - lower
        return draws[lower] * (1.0 - weight) + draws[upper] * weight

    lower = percentile(0.025)
    upper = percentile(0.975)
    return {
        'method': 'fixed_seed_two_stage_hierarchical_dataset_run_bootstrap',
        'seed': seed,
        'samples': samples,
        'independent_unit': 'dataset',
        'outer_cluster_sampling': True,
        'within_dataset': 'independent_run_resampling_per_system_then_mean',
        'runs_treated_as_pseudo_independent': False,
        'dataset_mean_differences_m': {
            name: values[index] for index, name in enumerate(names)},
        'mean_difference_m': sum(values) / len(values),
        'ci95_lower_m': lower,
        'ci95_upper_m': upper,
        'superiority': lower > 0.0,
    }


def _v2_fresh_slots(contract: dict[str, Any]) -> dict[str, dict[str, Any]]:
    slots = contract.get('datasets', {}).get('fresh_holdout_slots', {})
    return slots if isinstance(slots, dict) else {}


def _v2_historical_slots(contract: dict[str, Any]) -> dict[str, dict[str, Any]]:
    slots = contract.get('datasets', {}).get('holdout_slots', {})
    return slots if isinstance(slots, dict) else {}


def _v2_slot_sequences(slots: dict[str, dict[str, Any]]) -> list[str]:
    return sorted(slot['sequence'] for slot in slots.values()
                  if isinstance(slot, dict) and isinstance(slot.get('sequence'), str)
                  and slot.get('sequence'))


def _v2_expected_sequences(contract: dict[str, Any]) -> list[str]:
    """Return only sequences assigned to the fresh-slot contract.

    The historical ``holdout_slots`` are intentionally not consulted: they
    are exposed/frozen evaluation material and cannot be relabelled as fresh.
    """
    return sorted(slot['sequence'] for slot in _v2_fresh_slots(contract).values()
                  if isinstance(slot, dict) and isinstance(slot.get('sequence'), str)
                  and slot.get('sequence'))


def _v2_historical_sequences(contract: dict[str, Any]) -> list[str]:
    return _v2_slot_sequences(_v2_historical_slots(contract))


def _v2_partition_identity_values(slot: Mapping[str, Any]) -> dict[str, str]:
    """Return immutable partition identities; shared calibration is excluded."""
    values: dict[str, str] = {}
    for field in (
            'dataset', 'sequence', 'input_manifest_sha256',
            'ground_truth_sha256', 'raw_rosbag1_sha256', 'bag_sha256',
            'canonical_rosbag2_tree_sha256'):
        value = slot.get(field)
        if isinstance(value, str) and value:
            values[field] = value.lower()
        frozen = slot.get('frozen_identity')
        if isinstance(frozen, Mapping):
            if field == 'canonical_rosbag2_tree_sha256':
                nested = frozen.get(field)
                if isinstance(nested, str) and nested:
                    values[field] = nested.lower()
            if field in {'raw_rosbag1_sha256', 'bag_sha256'}:
                raw_bag = frozen.get('raw_bag')
                if isinstance(raw_bag, Mapping) and isinstance(raw_bag.get('sha256'), str):
                    values[field] = raw_bag['sha256'].lower()
    return values


def _v2_fresh_partition_disjointness(contract: dict[str, Any]) -> dict[str, Any]:
    """Reject fresh IDs/hashes reused by historical or development data."""
    fresh = _v2_fresh_slots(contract)
    historical = _v2_historical_slots(contract)
    identities: dict[tuple[str, str], list[str]] = {}
    for slot_id, slot in [*historical.items(), *fresh.items()]:
        if not isinstance(slot, Mapping):
            continue
        partition = 'fresh' if slot_id in fresh else 'historical'
        for field, value in _v2_partition_identity_values(slot).items():
            identities.setdefault((field, value), []).append(
                f'{partition}:{slot_id}')
    datasets = contract.get('datasets')
    if isinstance(datasets, Mapping):
        for partition_name in ('bringup', 'development', 'regression_only'):
            group = datasets.get(partition_name)
            if isinstance(group, Mapping):
                for dataset_id, value in group.items():
                    sequence = (value.get('sequence') if isinstance(value, Mapping)
                                else None)
                    for field, item in (
                            ('dataset', dataset_id), ('sequence', sequence)):
                        if isinstance(item, str) and item:
                            identities.setdefault((field, item.lower()), []).append(
                                f'{partition_name}:{dataset_id}')
    overlaps: dict[str, list[str]] = {}
    for (field, value), owners in identities.items():
        fresh_owners = [owner for owner in owners if owner.startswith('fresh:')]
        nonfresh_owners = [owner for owner in owners if not owner.startswith('fresh:')]
        if fresh_owners and nonfresh_owners:
            overlaps[f'{field}:{value}'] = sorted(set(owners))
        elif len(fresh_owners) > 1:
            overlaps[f'{field}:{value}'] = sorted(set(owners))
    return {
        'pass': not overlaps,
        'compared_identity_fields': [
            'dataset', 'sequence', 'input_manifest_sha256',
            'ground_truth_sha256', 'raw_rosbag1_sha256', 'bag_sha256',
            'canonical_rosbag2_tree_sha256'],
        'overlaps': overlaps,
        'calibration_hashes_excluded': True,
    }


def _v2_required_systems(contract: dict[str, Any], policy: dict[str, Any]) -> list[str]:
    configured = policy.get('required_systems')
    if isinstance(configured, list) and configured:
        return sorted(str(item) for item in configured)
    rivals = sorted({track.get('rival') for track in contract.get('tracks', {}).values()
                     if isinstance(track, dict) and track.get('rival')
                     and not track.get('enabled_when')})
    return ['ours', *rivals]


def _v2_provenance(record: dict[str, Any], system: str,
                   common_fields: list[str], pinned_fields: list[str],
                   errors: list[str], incomplete: list[str]) -> bool:
    provenance = record.get('provenance')
    if not isinstance(provenance, dict):
        incomplete.append(f'{system}.provenance is missing')
        return False
    valid = True
    for field in [*common_fields, *pinned_fields]:
        value = provenance.get(field, _MISSING)
        if value is _MISSING or value in ('', None, {}, []):
            incomplete.append(f'{system}.provenance.{field} is missing')
            valid = False
            continue
        if field.endswith('_sha256'):
            try:
                _v2_sha256(value, f'{system}.provenance.{field}')
            except ValueError as exc:
                errors.append(str(exc))
                valid = False
        if field.endswith('_fingerprint'):
            try:
                _v2_sha256(value, f'{system}.provenance.{field}')
            except ValueError as exc:
                errors.append(str(exc))
                valid = False
        if field == 'release' and value != 'Release':
            errors.append(f'{system}.provenance.release must be exactly Release')
            valid = False
        if field == 'revision' and (
                not isinstance(value, str) or
                not re.fullmatch(r'[0-9a-fA-F]{40}', value)):
            errors.append(f'{system}.provenance.revision must be a pinned 40-hex commit')
            valid = False
        if field == 'container_digest' and (
                not isinstance(value, str) or
                not re.fullmatch(r'sha256:[0-9a-fA-F]{64}', value)):
            errors.append(
                f'{system}.provenance.container_digest must be a pinned sha256 digest')
            valid = False
    if not isinstance(provenance.get('thread_policy'), dict):
        errors.append(f'{system}.provenance.thread_policy must be a mapping')
        valid = False
    return valid


def _v2_profile_rival_revisions(
        provenance_values: dict[str, dict[str, Any]],
        required_systems: list[str], contract: dict[str, Any],
        errors: list[str], incomplete: list[str]) -> tuple[bool, dict[str, Any]]:
    """Bind rival evidence to the revisions declared by the profile.

    The generic provenance validator intentionally checks representation (a
    pinned 40-hex revision), but representation alone does not prove that a
    run used the preregistered rival.  This gate is the narrow bridge between
    the profile's named-rival pins and the submitted evidence.  Missing pins
    remain incomplete; malformed or mismatched pins are invalid.
    """
    rivals = contract.get('rivals')
    if not isinstance(rivals, dict):
        incomplete.append('profile.rivals is missing or not a mapping')
        return False, {'status': 'INCOMPLETE', 'expected': {}, 'observed': {}}

    expected: dict[str, str] = {}
    observed: dict[str, Any] = {}
    valid = True
    for system in sorted(name for name in required_systems if name != 'ours'):
        rival = rivals.get(system)
        if not isinstance(rival, dict):
            incomplete.append(f'profile.rivals.{system} is missing')
            valid = False
            continue
        revision = rival.get('revision')
        if (not isinstance(revision, str) or
                not re.fullmatch(r'[0-9a-fA-F]{40}', revision)):
            errors.append(f'profile.rivals.{system}.revision must be a pinned 40-hex commit')
            valid = False
            continue
        expected[system] = revision.lower()
        provenance = provenance_values.get(system, {})
        observed_revision = provenance.get('revision', _MISSING)
        observed[system] = None if observed_revision is _MISSING else observed_revision
        if observed_revision is _MISSING:
            incomplete.append(f'{system}.provenance.revision is missing')
            valid = False
        elif not isinstance(observed_revision, str) or not re.fullmatch(
                r'[0-9a-fA-F]{40}', observed_revision):
            errors.append(f'{system}.provenance.revision is not a pinned 40-hex commit')
            valid = False
        elif observed_revision.lower() != revision.lower():
            errors.append(
                f'{system}.provenance.revision does not match profile.rivals.{system}.revision')
            valid = False
    return valid, {'status': 'PASS' if valid else 'FAIL_CLOSED',
                   'expected_revision_by_system': expected,
                   'observed_revision_by_system': observed}


def _v2_thread_policy(policy_value: Any, required_keys: list[str],
                      system: str, errors: list[str],
                      incomplete: list[str]) -> bool:
    if not isinstance(policy_value, dict):
        errors.append(f'{system}.provenance.thread_policy must be a mapping')
        return False
    valid = True
    for key in required_keys:
        if key not in policy_value:
            incomplete.append(f'{system}.thread_policy.{key} is missing')
            valid = False
            continue
        value = policy_value[key]
        if key == 'cpu_affinity':
            if (not isinstance(value, list) or not value or
                    any(isinstance(item, bool) or not isinstance(item, int)
                        or item < 0 for item in value)):
                errors.append(
                    f'{system}.thread_policy.cpu_affinity must be non-empty '
                    'non-negative integer list')
                valid = False
        elif key == 'accelerator_policy':
            if not isinstance(value, str) or not value:
                errors.append(
                    f'{system}.thread_policy.accelerator_policy must be non-empty string')
                valid = False
        elif (not isinstance(value, int) or isinstance(value, bool) or
              value <= 0):
            errors.append(f'{system}.thread_policy.{key} must be positive integer')
            valid = False
    return valid


def _v2_canonical_hash(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(',', ':'),
                         ensure_ascii=True).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _v2_score_artifact_payload(system: str, run: Mapping[str, Any]) -> dict[str, Any]:
    """Return the non-GT score record whose bytes are bound per run.

    This is intentionally a small, deterministic projection of values used by
    the v2 metric gates.  It contains no reference/GT identity or raw result
    content; the bundle verifier only compares its opaque SHA and never parses
    the file.
    """
    trajectory = run.get('trajectory')
    runtime = run.get('runtime')
    mapping = run.get('map', run.get('mapping'))
    if not isinstance(trajectory, Mapping) or not isinstance(runtime, Mapping):
        raise ValueError('score artifact projection requires trajectory/runtime mappings')
    if not isinstance(mapping, Mapping):
        raise ValueError('score artifact projection requires map metrics')
    runtime_keys = (
        'processing_rtf', 'online_compute_rtf', 'peak_rss_mb',
        'phase_contract_version', 'phase_mode',
        'paced_followability_passed', 'unpaced_throughput_gate_passed',
    )
    runtime_projection = {
        key: runtime[key] for key in runtime_keys if key in runtime}
    return {
        'schema': 'competitive_run_score_v1',
        'system': system,
        'dataset': run.get('dataset'),
        'run_index': run.get('run_index'),
        'completion': {
            key: run.get(key) for key in (
                'complete', 'process_exit_status', 'trajectory_complete',
                'sequence_failure', 'catastrophic_failure',
                'verified_false_loops')
        },
        'trajectory': {'ape_rmse_m': trajectory.get('ape_rmse_m')},
        'runtime': runtime_projection,
        'map': {
            key: mapping.get(key) for key in (
                'plane_thickness_mean_m', 'plane_thickness_p95_m',
                'planar_coverage')
        },
    }


def _v2_score_artifact_sha256(system: str, run: Mapping[str, Any]) -> str:
    payload = _v2_score_artifact_payload(system, run)
    encoded = (json.dumps(payload, sort_keys=True, separators=(',', ':'),
                           ensure_ascii=True) + '\n').encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _v2_normalize_runs(
    record: dict[str, Any],
    expected: list[str],
    repetitions: int,
) -> tuple[
    dict[tuple[str, int], dict[str, Any]],
    list[str],
    list[str],
]:
    """Normalize the v2 flat run-list shape and report missing/extra keys."""
    missing: list[str] = []
    errors: list[str] = []
    raw_runs = record.get('runs')
    if not isinstance(raw_runs, list):
        return {}, ['runs is missing or not a list'], []
    rows: dict[tuple[str, int], dict[str, Any]] = {}
    for position, run in enumerate(raw_runs):
        if not isinstance(run, dict):
            errors.append(f'runs[{position}] must be a mapping')
            continue
        dataset = run.get('dataset')
        index = run.get('run_index')
        if not isinstance(dataset, str) or not isinstance(index, int) or isinstance(index, bool):
            errors.append(f'runs[{position}] requires dataset and integer run_index')
            continue
        key = (dataset, index)
        if key in rows:
            errors.append(f'duplicate run record: {dataset}/{index}')
        rows[key] = run
    expected_keys = {(dataset, index) for dataset in expected
                     for index in range(1, repetitions + 1)}
    for key in sorted(expected_keys - set(rows)):
        missing.append(f'missing run record: {key[0]}/{key[1]}')
    for key in sorted(set(rows) - expected_keys):
        errors.append(f'unexpected run record: {key[0]}/{key[1]}')
    return rows, missing, errors


def evaluate_evidence_v2(evidence: dict[str, Any],
                         contract: dict[str, Any],
                         *, require_bundle: bool = False) -> dict[str, Any]:
    """Fail-closed validator for competitive victory evidence schema v2.

    This is intentionally additive to :func:`evaluate`, which is the legacy
    report-only bool-gate adapter.  A v1 gate or a report-only manifest cannot
    satisfy this validator because the provenance and per-run evidence are
    validated before any metric is considered.
    """
    policy = contract.get('evidence_gate_v2', {})
    errors: list[str] = []
    incomplete: list[str] = []
    checks: dict[str, dict[str, Any]] = {}
    phase_policy = policy.get('online_phase_contract_v2', {})
    if not isinstance(phase_policy, dict):
        phase_policy = {}
    configured_unpaced_rtf = phase_policy.get(
        'maximum_unpaced_throughput_rtf', 1.0)
    try:
        configured_unpaced_rtf = _v2_finite(
            configured_unpaced_rtf,
            'evidence_gate_v2.online_phase_contract_v2.'
            'maximum_unpaced_throughput_rtf')
    except ValueError as exc:
        errors.append(str(exc))
        configured_unpaced_rtf = 1.0

    def check(name: str, passed: bool, details: Any) -> None:
        checks[name] = {
            'pass': bool(passed),
            'evidence': _v2_jsonable(details),
        }

    schema_ok = (isinstance(evidence, dict) and
                 evidence.get('schema_version') == 2 and
                 evidence.get('evidence_kind') == 'competitive_slam_victory_evidence')
    check('schema_v2', schema_ok, {
        'schema_version': evidence.get('schema_version') if isinstance(evidence, dict) else None,
        'evidence_kind': evidence.get('evidence_kind') if isinstance(evidence, dict) else None,
    })
    if not schema_ok:
        errors.append(
            'schema_version=2 and evidence_kind=competitive_slam_victory_evidence '
            'are required')

    # A victory receipt must identify the execution-selection preregistration
    # used to freeze containers, toolchains, configs, and thread policy.  The
    # Both this gate and the standalone preflight checker enforce the
    # pending/ready decision.  The gate additionally verifies that the
    # profile's registered path and digest are real and unchanged, so a
    # metric-complete evidence file cannot bypass execution identity.
    execution_receipt_path = policy.get('execution_selection_receipt_path', _MISSING)
    execution_receipt_sha = policy.get('execution_selection_receipt_sha256', _MISSING)
    execution_receipt_actual = None
    execution_receipt_ok = True
    execution_receipt_status = None
    execution_receipt = None
    execution_closure_identity = None
    execution_closure_identity_ok = True
    if execution_receipt_path is _MISSING or execution_receipt_sha is _MISSING:
        incomplete.append('profile execution-selection receipt path/SHA is missing')
        execution_receipt_ok = False
    else:
        try:
            _v2_sha256(execution_receipt_sha,
                       'profile.execution_selection_receipt_sha256')
        except ValueError as exc:
            errors.append(str(exc))
            execution_receipt_ok = False
        receipt_path = (ROOT / execution_receipt_path
                        if isinstance(execution_receipt_path, str) and
                        not Path(execution_receipt_path).is_absolute()
                        else Path(execution_receipt_path)
                        if isinstance(execution_receipt_path, str) else None)
        if receipt_path is None or not receipt_path.is_file():
            incomplete.append('profile execution-selection receipt file is missing')
            execution_receipt_ok = False
        else:
            execution_receipt_actual = hashlib.sha256(
                receipt_path.read_bytes()).hexdigest()
            if (isinstance(execution_receipt_sha, str) and
                    execution_receipt_actual != execution_receipt_sha.lower()):
                errors.append('profile execution-selection receipt SHA does not match file')
                execution_receipt_ok = False
            try:
                execution_receipt = yaml.safe_load(
                    receipt_path.read_text(encoding='utf-8'))
            except (OSError, yaml.YAMLError) as exc:
                errors.append(f'execution-selection receipt cannot be loaded: {exc}')
                execution_receipt = None
            if not isinstance(execution_receipt, dict):
                errors.append('execution-selection receipt must be a YAML mapping')
            else:
                if execution_receipt.get('schema_version') != 1:
                    errors.append(
                        'execution-selection receipt schema_version must be 1')
                    execution_receipt_ok = False
                if execution_receipt.get('receipt_kind') != (
                        'competitive_execution_selection'):
                    errors.append(
                        'execution-selection receipt receipt_kind must be '
                        'competitive_execution_selection')
                    execution_receipt_ok = False
                execution_receipt_status = execution_receipt.get('status')
                if execution_receipt_status not in {'ready', 'frozen'}:
                    if (execution_receipt_status is None or
                            'pending' in str(execution_receipt_status)):
                        incomplete.append(
                            'execution-selection receipt status is not ready/frozen: '
                            f'{execution_receipt_status!r}')
                    else:
                        errors.append(
                            'execution-selection receipt status must be ready/frozen: '
                            f'{execution_receipt_status!r}')
                    execution_receipt_ok = False
                receipt_common = execution_receipt.get('common_identity')
                if not isinstance(receipt_common, dict):
                    errors.append(
                        'execution-selection receipt common_identity must be a mapping')
                    execution_receipt_ok = False
                    receipt_common = {}
                receipt_profile_sha = receipt_common.get('profile_sha256')
                receipt_profile_kind = receipt_common.get('profile_sha256_kind')
                try:
                    _v2_sha256(receipt_profile_sha,
                               'execution-selection receipt common_identity.profile_sha256')
                except ValueError as exc:
                    if receipt_profile_sha is None:
                        incomplete.append(str(exc))
                    else:
                        errors.append(str(exc))
                    execution_receipt_ok = False
                if receipt_profile_kind is None:
                    incomplete.append(
                        'execution-selection receipt common_identity.profile_sha256_kind '
                        'is missing')
                    execution_receipt_ok = False
                elif receipt_profile_kind != PROFILE_CANONICAL_HASH_KIND:
                    errors.append(
                        'execution-selection receipt common_identity.profile_sha256_kind '
                        f'must be {PROFILE_CANONICAL_HASH_KIND}')
                    execution_receipt_ok = False
                try:
                    computed_profile_sha = canonical_profile_sha256(
                        {'competitive_slam_profile': contract})
                except ValueError as exc:
                    errors.append(str(exc))
                    computed_profile_sha = None
                if (computed_profile_sha is not None and
                        isinstance(receipt_profile_sha, str) and
                        _SHA256_RE.fullmatch(receipt_profile_sha) and
                        receipt_profile_sha.lower() != computed_profile_sha):
                    errors.append(
                        'execution-selection receipt common_identity.profile_sha256 '
                        'does not match canonical profile')
                    execution_receipt_ok = False
                check('execution_receipt_profile_canonical_hash',
                      execution_receipt_ok and
                      isinstance(receipt_profile_sha, str) and
                      _SHA256_RE.fullmatch(receipt_profile_sha) is not None and
                      receipt_profile_kind == PROFILE_CANONICAL_HASH_KIND,
                      {
                          'declared_sha256': receipt_profile_sha,
                          'computed_sha256': computed_profile_sha,
                          'hash_kind': receipt_profile_kind,
                          'expected_hash_kind': PROFILE_CANONICAL_HASH_KIND,
                      })
                closure_policy = policy.get('rival_source_closure', {})
                if (isinstance(closure_policy, dict) and
                        closure_policy.get('required') is True):
                    try:
                        execution_closure_identity = (
                            current_rival_source_closure_identity(
                                {'competitive_slam_profile': contract},
                                root=ROOT))
                        closure_errors = (
                            validate_rival_source_closure_receipt_identity(
                                execution_receipt,
                                execution_closure_identity))
                    except (OSError, ValueError, TypeError, UnicodeError,
                            yaml.YAMLError) as exc:
                        closure_errors = [
                            f'current rival source closure identity is invalid: {exc}']
                    if closure_errors:
                        errors.extend(closure_errors)
                        errors.append(
                            'execution-selection receipt is not eligible: '
                            'missing or mismatched rival source closure identity')
                        execution_closure_identity_ok = False
                        execution_receipt_ok = False
                check('execution_receipt_rival_source_closure_identity',
                      execution_closure_identity_ok, {
                          'expected': execution_closure_identity,
                          'observed': receipt_common.get(
                              'rival_source_closure'),
                      })
    check('execution_selection_receipt_registered', execution_receipt_ok, {
        'path': None if execution_receipt_path is _MISSING else execution_receipt_path,
        'expected_sha256': None if execution_receipt_sha is _MISSING else execution_receipt_sha,
        'actual_sha256': execution_receipt_actual,
        'status': execution_receipt_status,
        'ready_statuses': ['frozen', 'ready'],
    })

    profile_fresh_slots = _v2_fresh_slots(contract)
    profile_historical_slots = _v2_historical_slots(contract)
    fresh_sequences = _v2_expected_sequences(contract)
    historical_sequences = _v2_historical_sequences(contract)
    expected_sequences = sorted(set(historical_sequences + fresh_sequences))
    repetitions = int(policy.get('repetitions', contract.get('repetitions', 3)))
    required_systems = _v2_required_systems(contract, policy)
    source = evidence if isinstance(evidence, dict) else {}
    declared = source.get('contract', {})
    if not isinstance(declared, dict):
        declared = {}
    source_closure_requested = bool(
        require_bundle or source.get('claim_eligible') is True or
        source.get('evidence_bundle') is not None)
    # The receipt registration checks above bind only the selected receipt
    # path/SHA, profile canonical hash, and rival-closure identity.  A claim
    # must also reopen the registered execution-selection receipt through the
    # authoritative preflight checker: that is where input/calibration/config,
    # container/toolchain, scorer, machine/thread, resource, and GT-blind
    # identities are checked against their bytes and the current profile.
    # Report-only evidence deliberately remains compatible and does not claim
    # this stronger execution contract.
    if source_closure_requested:
        try:
            execution_preflight = evaluate_execution_selection(
                execution_receipt if isinstance(execution_receipt, dict) else {},
                {'competitive_slam_profile': contract},
                root=ROOT)
            if not isinstance(execution_preflight, dict):
                raise TypeError('execution preflight checker returned a non-mapping')
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            # A claim gate must turn a malformed/unreopenable preflight into a
            # deterministic failure receipt, never fall through to metric
            # evaluation or expose an exception as an apparent pass.
            execution_preflight = {
                'schema_version': 1,
                'status': 'FAIL_CLOSED',
                'pass': False,
                'claim_eligible': False,
                'errors': [f'checker exception: {exc}'],
            }
        execution_preflight_ok = execution_preflight.get('pass') is True
        check('execution_selection_preflight', execution_preflight_ok,
              execution_preflight)
        if not execution_preflight_ok:
            execution_preflight_errors = execution_preflight.get('errors', [])
            if execution_preflight.get('status') == 'INCOMPLETE':
                incomplete.extend(
                    'execution selection preflight: ' + str(item)
                    for item in execution_preflight_errors)
            else:
                errors.extend(
                    'execution selection preflight: ' + str(item)
                    for item in execution_preflight_errors)
    else:
        execution_preflight = {
            'schema_version': 1,
            'status': 'NOT_REQUESTED',
            'pass': True,
            'claim_eligible': False,
            'errors': [],
            'reason': 'report-only evidence did not request execution preflight',
        }
        check('execution_selection_preflight', True, execution_preflight)
    if source_closure_requested:
        rival_source_closure_result = verify_rival_source_closure(
            {'competitive_slam_profile': contract},
            root=ROOT,
            receipt=execution_receipt)
        source_closure_ok = rival_source_closure_result.get('pass') is True
        check('rival_source_closure', source_closure_ok,
              rival_source_closure_result)
        if not source_closure_ok:
            if rival_source_closure_result.get('status') == 'NOT_READY':
                incomplete.extend(
                    'rival source closure: ' + str(item)
                    for item in rival_source_closure_result.get('not_ready', [])
                )
                incomplete.extend(
                    'rival source closure incomplete: ' + str(item)
                    for item in rival_source_closure_result.get('incomplete', [])
                )
            else:
                errors.extend(
                    'rival source closure: ' + str(item)
                    for item in rival_source_closure_result.get('errors', [])
                )
                incomplete.extend(
                    'rival source closure incomplete: ' + str(item)
                    for item in rival_source_closure_result.get('incomplete', [])
                )
        dataset_source_closure_result = verify_dataset_source_closure(
            {'competitive_slam_profile': contract}, root=ROOT)
        dataset_source_closure_ok = dataset_source_closure_result.get('pass') is True
        check('dataset_source_closure', dataset_source_closure_ok,
              dataset_source_closure_result)
        if not dataset_source_closure_ok:
            if dataset_source_closure_result.get('status') == 'NOT_READY':
                incomplete.extend(
                    'dataset source closure: ' + str(item)
                    for item in dataset_source_closure_result.get('not_ready', [])
                )
            else:
                errors.extend(
                    'dataset source closure: ' + str(item)
                    for item in dataset_source_closure_result.get('errors', [])
                )
    else:
        rival_source_closure_result = {
            'schema_version': 1,
            'status': 'NOT_REQUESTED',
            'pass': True,
            'claim_eligible': False,
            'reason': 'report-only evidence did not request a claim',
        }
        check('rival_source_closure', True, rival_source_closure_result)
        dataset_source_closure_result = {
            'schema_version': 1,
            'status': 'NOT_REQUESTED',
            'pass': True,
            'claim_eligible': False,
            'reason': 'report-only evidence did not request a claim',
        }
        check('dataset_source_closure', True, dataset_source_closure_result)
    declared_execution_sha = declared.get(
        'execution_selection_receipt_sha256', _MISSING)
    declared_execution_ok = True
    if declared_execution_sha is _MISSING:
        incomplete.append(
            'contract.execution_selection_receipt_sha256 is missing')
        declared_execution_ok = False
    else:
        try:
            _v2_sha256(declared_execution_sha,
                       'contract.execution_selection_receipt_sha256')
        except ValueError as exc:
            errors.append(str(exc))
            declared_execution_ok = False
        if (isinstance(execution_receipt_sha, str) and
                isinstance(declared_execution_sha, str) and
                declared_execution_sha.lower() != execution_receipt_sha.lower()):
            errors.append(
                'contract.execution_selection_receipt_sha256 does not match profile')
            declared_execution_ok = False
    check('execution_selection_receipt_in_evidence_contract',
          declared_execution_ok and execution_receipt_ok, {
              'declared_sha256': None if declared_execution_sha is _MISSING
              else declared_execution_sha,
              'profile_sha256': None if execution_receipt_sha is _MISSING
              else execution_receipt_sha,
              'status': execution_receipt_status,
          })
    try:
        computed_profile_sha = canonical_profile_sha256(
            {'competitive_slam_profile': contract})
    except ValueError as exc:
        errors.append(str(exc))
        computed_profile_sha = None
    declared_profile_sha = declared.get('profile_sha256', _MISSING)
    declared_profile_kind = declared.get('profile_sha256_kind', _MISSING)
    declared_profile_ok = True
    if declared_profile_sha is _MISSING:
        incomplete.append('contract.profile_sha256 is missing')
        declared_profile_ok = False
    else:
        try:
            _v2_sha256(declared_profile_sha, 'contract.profile_sha256')
        except ValueError as exc:
            errors.append(str(exc))
            declared_profile_ok = False
    if declared_profile_kind is _MISSING:
        incomplete.append('contract.profile_sha256_kind is missing')
        declared_profile_ok = False
    elif declared_profile_kind != PROFILE_CANONICAL_HASH_KIND:
        errors.append(
            'contract.profile_sha256_kind must be '
            f'{PROFILE_CANONICAL_HASH_KIND}')
        declared_profile_ok = False
    if (declared_profile_ok and computed_profile_sha is not None and
            declared_profile_sha.lower() != computed_profile_sha):
        errors.append('contract.profile_sha256 does not match canonical profile')
        declared_profile_ok = False
    check('profile_canonical_hash_in_evidence_contract',
          declared_profile_ok and computed_profile_sha is not None, {
              'declared_sha256': None if declared_profile_sha is _MISSING
              else declared_profile_sha,
              'computed_sha256': computed_profile_sha,
              'declared_kind': None if declared_profile_kind is _MISSING
              else declared_profile_kind,
              'expected_kind': PROFILE_CANONICAL_HASH_KIND,
          })
    declared_partitions = declared.get('partitions', {})
    if not isinstance(declared_partitions, dict):
        declared_partitions = {}
    declared_historical_partition = declared_partitions.get('historical', {})
    declared_fresh_partition = declared_partitions.get('fresh', {})
    if not isinstance(declared_historical_partition, dict):
        declared_historical_partition = {}
    if not isinstance(declared_fresh_partition, dict):
        declared_fresh_partition = {}
    declared_sequences = declared.get('datasets', _MISSING)
    if isinstance(declared_sequences, list):
        declared_sequences = sorted(
            item.get('sequence') if isinstance(item, dict) else item
            for item in declared_sequences)
    declared_slots = declared_fresh_partition.get(
        'slots', declared.get('fresh_holdout_slots', _MISSING))
    if isinstance(declared_slots, list):
        declared_slots = {
            item.get('slot_id'): item for item in declared_slots
            if isinstance(item, dict) and item.get('slot_id')}
    declared_fresh_sequences = declared_fresh_partition.get(
        'datasets', _MISSING)
    if isinstance(declared_fresh_sequences, list):
        declared_fresh_sequences = sorted(
            item.get('sequence') if isinstance(item, dict) else item
            for item in declared_fresh_sequences)
    historical_slot_ids = sorted(profile_historical_slots)
    declared_historical_slots = declared_historical_partition.get('slots', _MISSING)
    if isinstance(declared_historical_slots, list):
        declared_historical_slots = {
            item.get('slot_id'): item for item in declared_historical_slots
            if isinstance(item, dict) and item.get('slot_id')}
    declared_historical_sequences = declared_historical_partition.get(
        'datasets', _MISSING)
    if isinstance(declared_historical_sequences, list):
        declared_historical_sequences = sorted(
            item.get('sequence') if isinstance(item, dict) else item
            for item in declared_historical_sequences)
    profile_slot_ids = sorted(profile_fresh_slots)
    declared_slot_ids = (sorted(declared_slots) if isinstance(declared_slots, dict)
                         else [])
    slot_statuses = {slot_id: slot.get('status')
                     for slot_id, slot in profile_fresh_slots.items()
                     if isinstance(slot, dict)}
    ready_statuses = {'frozen_unopened', 'frozen'}
    minimum_fresh_slots = int(policy.get('minimum_fresh_holdout_slots', 3))
    fresh_slots_ready = (
        len(profile_fresh_slots) >= minimum_fresh_slots and
        all(slot_statuses.get(slot_id) in ready_statuses
            for slot_id in profile_slot_ids) and
        len(fresh_sequences) == len(profile_fresh_slots) and
        len(set(fresh_sequences)) == len(fresh_sequences))
    if not profile_fresh_slots:
        incomplete.append('profile.datasets.fresh_holdout_slots is missing')
    elif not fresh_slots_ready:
        incomplete.append(
            'profile fresh holdout slots are pending assignment or incomplete')
    if declared_slots is _MISSING:
        incomplete.append('contract.fresh_holdout_slots is missing')
    elif declared_slot_ids != profile_slot_ids and fresh_slots_ready:
        errors.append('contract fresh slot IDs must exactly match the profile')
    elif declared_slot_ids != profile_slot_ids:
        incomplete.append('fresh slot IDs cannot be checked while profile slots are pending')
    slot_identity_fields = (
        'selection_receipt_sha256', 'input_manifest_sha256',
        'ground_truth_sha256', 'calibration_archive_sha256')
    slot_identity_ok = True
    for slot_id in profile_slot_ids:
        profile_slot = profile_fresh_slots.get(slot_id, {})
        observed_slot = (declared_slots.get(slot_id, {})
                         if isinstance(declared_slots, dict) else {})
        if not isinstance(profile_slot, dict) or not isinstance(observed_slot, dict):
            slot_identity_ok = False
            continue
        if profile_slot.get('status') not in ready_statuses:
            # Pending profile slots intentionally cannot be compared with a
            # producer's self-declared selection; the final result is
            # INCOMPLETE rather than allowing stale evidence to become INVALID.
            continue
        if profile_slot.get('sequence') != observed_slot.get('sequence'):
            errors.append(f'fresh slot sequence mismatch: {slot_id}')
            slot_identity_ok = False
        for field in slot_identity_fields:
            expected_hash = profile_slot.get(field, _MISSING)
            observed_hash = observed_slot.get(field, _MISSING)
            if expected_hash is _MISSING or expected_hash is None:
                if profile_slot.get('status') in ready_statuses:
                    incomplete.append(f'profile fresh slot {slot_id} lacks {field}')
                slot_identity_ok = False
                continue
            if observed_hash is _MISSING:
                incomplete.append(f'evidence fresh slot {slot_id} lacks {field}')
                slot_identity_ok = False
                continue
            try:
                _v2_sha256(expected_hash, f'profile fresh slot {slot_id}.{field}')
                _v2_sha256(observed_hash, f'evidence fresh slot {slot_id}.{field}')
            except ValueError as exc:
                errors.append(str(exc))
                slot_identity_ok = False
                continue
            if expected_hash.lower() != observed_hash.lower():
                errors.append(f'fresh slot hash mismatch: {slot_id}.{field}')
                slot_identity_ok = False
    declared_for_receipt = None if declared_sequences is _MISSING else declared_sequences
    historical_statuses = {
        slot_id: slot.get('status')
        for slot_id, slot in profile_historical_slots.items()
        if isinstance(slot, dict)
    }
    historical_slots_frozen = (
        bool(historical_statuses) and
        all(status == 'frozen' for status in historical_statuses.values()))
    historical_partition_ok = (
        isinstance(declared_historical_slots, dict) and
        sorted(declared_historical_slots) == historical_slot_ids and
        declared_historical_sequences == historical_sequences and
        historical_slots_frozen)
    if declared_historical_slots is _MISSING:
        incomplete.append('contract.partitions.historical.slots is missing')
    elif sorted(declared_historical_slots) != historical_slot_ids:
        incomplete.append('historical partition slot IDs are incomplete')
    if declared_historical_sequences is _MISSING:
        incomplete.append('contract.partitions.historical.datasets is missing')
    elif declared_historical_sequences != historical_sequences:
        errors.append('historical partition datasets do not match profile')
    if not historical_slots_frozen:
        incomplete.append('profile historical holdout slots must all be frozen')
    historical_identity_fields = (
        'input_manifest_sha256', 'ground_truth_sha256',
        'calibration_archive_sha256')
    for slot_id in historical_slot_ids:
        profile_slot = profile_historical_slots[slot_id]
        observed_slot = (declared_historical_slots.get(slot_id, {})
                         if isinstance(declared_historical_slots, dict) else {})
        if not isinstance(observed_slot, dict):
            historical_partition_ok = False
            continue
        for field in historical_identity_fields:
            expected_hash = profile_slot.get(field, _MISSING)
            observed_hash = observed_slot.get(field, _MISSING)
            if expected_hash is _MISSING or observed_hash is _MISSING:
                incomplete.append(f'historical slot {slot_id} lacks {field}')
                historical_partition_ok = False
                continue
            try:
                _v2_sha256(expected_hash, f'profile historical slot {slot_id}.{field}')
                _v2_sha256(observed_hash, f'evidence historical slot {slot_id}.{field}')
            except ValueError as exc:
                errors.append(str(exc))
                historical_partition_ok = False
                continue
            if expected_hash.lower() != observed_hash.lower():
                errors.append(f'historical slot hash mismatch: {slot_id}.{field}')
                historical_partition_ok = False
    holdout_ok = (fresh_slots_ready and slot_identity_ok and
                  declared_slot_ids == profile_slot_ids and
                  declared_fresh_sequences == fresh_sequences and
                  historical_partition_ok and
                  declared_sequences == expected_sequences)
    check('fresh_slot_contract_not_self_declared', holdout_ok, {
        'profile_slot_statuses': slot_statuses,
        'minimum_fresh_slots': minimum_fresh_slots,
        'declared_slot_ids': declared_slot_ids,
        'profile_slot_ids': profile_slot_ids,
        'declared_datasets': declared_for_receipt,
        'expected_datasets': fresh_sequences,
        'declared_fresh_partition_datasets': declared_fresh_sequences,
        'historical_partition_datasets': historical_sequences,
        'historical_slot_statuses': historical_statuses,
        'self_declared_fresh_holdout_ignored': source.get('fresh_holdout'),
    })
    if declared_sequences is _MISSING:
        incomplete.append('contract.datasets is missing')
    elif declared_sequences != expected_sequences:
        if fresh_slots_ready:
            errors.append('contract.datasets must match both partitions')
        else:
            incomplete.append('dataset identities await fresh-slot assignment')
    if declared_fresh_sequences is _MISSING:
        incomplete.append('contract.partitions.fresh.datasets is missing')
    elif fresh_slots_ready and declared_fresh_sequences != fresh_sequences:
        errors.append('fresh partition datasets do not match profile')
    elif not fresh_slots_ready and declared_fresh_sequences != fresh_sequences:
        incomplete.append('fresh partition datasets await slot assignment')
    check('partition_contract', holdout_ok, {
        'historical': {
            'datasets': historical_sequences,
            'slot_ids': historical_slot_ids,
            'identity_fields': historical_identity_fields,
        },
        'fresh': {
            'datasets': fresh_sequences,
            'slot_ids': profile_slot_ids,
            'identity_fields': slot_identity_fields,
        },
    })
    partition_disjointness = _v2_fresh_partition_disjointness(contract)
    check('fresh_partition_disjoint_immutable_identities',
          holdout_ok and partition_disjointness['pass'],
          partition_disjointness)
    if not partition_disjointness['pass']:
        errors.append('fresh holdout partition reuses an immutable dataset ID or hash')

    authorization_policy = policy.get('fresh_holdout_authorization', {})
    if not isinstance(authorization_policy, Mapping):
        authorization_policy = {}
    authorization_requested = bool(
        require_bundle or source.get('claim_eligible') is True or
        source.get('evidence_bundle') is not None)
    if authorization_requested:
        authorization_result = verify_fresh_holdout_authorization(
            source,
            policy=authorization_policy,
            profile={'competitive_slam_profile': contract},
            expected_profile_sha256=computed_profile_sha,
        )
        check('fresh_holdout_authorization',
              authorization_result.get('pass') is True,
              authorization_result)
        if authorization_result.get('status') == 'NOT_READY':
            incomplete.extend(
                'fresh holdout authorization: ' + str(item)
                for item in authorization_result.get('not_ready', []))
        else:
            errors.extend(
                'fresh holdout authorization: ' + str(item)
                for item in authorization_result.get('errors', []))
    else:
        check('fresh_holdout_authorization', True, {
            'status': 'NOT_REQUESTED',
            'claim_eligible': False,
            'reason': 'report-only evidence did not request a claim',
        })

    fresh_slots_by_sequence = {
        slot.get('sequence'): slot for slot in profile_fresh_slots.values()
        if isinstance(slot, dict) and isinstance(slot.get('sequence'), str)
    }
    historical_slots_by_sequence = {
        slot.get('sequence'): slot for slot in profile_historical_slots.values()
        if isinstance(slot, dict) and isinstance(slot.get('sequence'), str)
    }
    dataset_to_partition = {
        **{dataset: 'historical' for dataset in historical_sequences},
        **{dataset: 'fresh' for dataset in fresh_sequences},
    }
    systems = source.get('systems')
    if not isinstance(systems, dict):
        systems = {}
        incomplete.append('systems mapping is missing')
    missing_systems = sorted(set(required_systems) - set(systems))
    unexpected_systems = sorted(set(systems) - set(required_systems))
    systems_ok = not missing_systems and not unexpected_systems
    check('all_expected_rivals', systems_ok, {
        'required': required_systems,
        'provided': sorted(systems),
        'missing': missing_systems,
        'unexpected': unexpected_systems,
    })
    incomplete.extend(f'missing system: {name}' for name in missing_systems)
    if unexpected_systems:
        errors.append(f'unexpected systems: {unexpected_systems}')

    common_fields = list(policy.get('common_identity_fields', [
        'input_sha256', 'reference_sha256', 'calibration_sha256',
        'machine_id', 'hardware_fingerprint', 'thread_policy', 'release',
        'scorer_fingerprint',
    ]))
    pinned_fields = list(policy.get('pinned_identity_fields', [
        'revision', 'container_digest', 'toolchain_fingerprint',
        'config_sha256',
    ]))
    provenance_values: dict[str, dict[str, Any]] = {}
    provenance_ok = True
    for system in required_systems:
        record = systems.get(system)
        if not isinstance(record, dict):
            incomplete.append(f'{system} record is missing')
            provenance_ok = False
            continue
        if not _v2_provenance(record, system, common_fields, pinned_fields,
                              errors, incomplete):
            provenance_ok = False
        if isinstance(record.get('provenance'), dict):
            provenance_values[system] = record['provenance']
    closure_policy = policy.get('rival_source_closure', {})
    closure_required = (isinstance(closure_policy, dict) and
                        closure_policy.get('required') is True)
    suite_closure_identity = None
    suite_closure_ok = True
    if closure_required:
        try:
            suite_closure_identity = current_rival_source_closure_identity(
                {'competitive_slam_profile': contract}, root=ROOT)
        except (OSError, ValueError, TypeError, UnicodeError, yaml.YAMLError) as exc:
            errors.append(f'current rival source closure identity is invalid: {exc}')
            suite_closure_ok = False
        for system in required_systems:
            observed = provenance_values.get(system, {}).get(
                'rival_source_closure', _MISSING)
            if observed is _MISSING:
                incomplete.append(
                    f'{system}.provenance.rival_source_closure is missing')
                suite_closure_ok = False
            elif suite_closure_identity is None or observed != suite_closure_identity:
                errors.append(
                    f'{system}.provenance.rival_source_closure does not match '
                    'the current closure')
                suite_closure_ok = False
    check('rival_source_closure_identity', suite_closure_ok, {
        'required': closure_required,
        'expected': suite_closure_identity,
        'observed_by_system': {
            system: provenance_values.get(system, {}).get(
                'rival_source_closure', _MISSING)
            for system in required_systems
        },
    })
    for field in common_fields:
        values = [row.get(field, _MISSING) for row in provenance_values.values()]
        missing = not values or any(value is _MISSING for value in values)
        equal = (not missing and
                 all(value == values[0] for value in values))
        if missing:
            incomplete.append(f'common identity field is missing: {field}')
        elif not equal:
            errors.append(f'common identity field differs or is missing: {field}')
        provenance_ok = provenance_ok and equal
    check('pinned_common_identity', provenance_ok, {
        'common_fields': common_fields,
        'pinned_fields': pinned_fields,
        'provenance': provenance_values,
    })
    rival_revision_ok, rival_revision_evidence = _v2_profile_rival_revisions(
        provenance_values, required_systems, contract, errors, incomplete)
    check('profile_pinned_rival_revisions', rival_revision_ok,
          rival_revision_evidence)
    thread_required_keys = list(policy.get('thread_policy_required_keys', [
        'cpu_affinity', 'max_threads', 'omp_num_threads',
        'openblas_num_threads', 'mkl_num_threads', 'tbb_num_threads',
        'accelerator_policy']))
    thread_policy_ok = True
    thread_hashes: dict[str, str] = {}
    for system, provenance in provenance_values.items():
        value = provenance.get('thread_policy', _MISSING)
        if not _v2_thread_policy(value, thread_required_keys, system,
                                 errors, incomplete):
            thread_policy_ok = False
            continue
        thread_hashes[system] = _v2_canonical_hash(value)
    if thread_hashes and len(set(thread_hashes.values())) != 1:
        errors.append('thread_policy differs between systems')
        thread_policy_ok = False
    scorer_values = {
        system: (None if provenance.get('scorer_fingerprint', _MISSING) is _MISSING
                 else provenance.get('scorer_fingerprint'))
        for system, provenance in provenance_values.items()}
    scorer_missing = (not scorer_values or
                      any(value is None for value in scorer_values.values()))
    scorer_ok = (not scorer_missing and
                 len(set(scorer_values.values())) == 1)
    if scorer_missing:
        incomplete.append('scorer_fingerprint is missing from one or more systems')
    elif not scorer_ok:
        errors.append('scorer_fingerprint must be identical across all systems')
    check('thread_policy_contract', thread_policy_ok, {
        'required_keys': thread_required_keys,
        'canonical_sha256_by_system': thread_hashes,
        'canonical_sha256': next(iter(set(thread_hashes.values())), None),
    })
    check('scorer_fingerprint_common', scorer_ok, {
        'fingerprint_by_system': scorer_values,
    })

    run_records: dict[str, dict[tuple[str, int], dict[str, Any]]] = {}
    run_missing: list[str] = []
    run_errors: list[str] = []
    complete = True
    for system in required_systems:
        record = systems.get(system)
        if not isinstance(record, dict):
            complete = False
            continue
        if not fresh_slots_ready:
            # Do not reinterpret legacy/exposed holdout runs against an
            # unassigned fresh profile.  The receipt remains INCOMPLETE until
            # slot assignment is complete.
            run_records[system] = {}
            complete = False
            continue
        rows, missing, row_errors = _v2_normalize_runs(
            record, expected_sequences, repetitions)
        run_records[system] = rows
        run_missing.extend(f'{system}: {item}' for item in missing)
        run_errors.extend(f'{system}: {item}' for item in row_errors)
        complete = complete and not missing and not row_errors
    incomplete.extend(run_missing)
    errors.extend(run_errors)

    complete_rows: dict[str, list[dict[str, Any]]] = {name: [] for name in required_systems}
    # A claim cannot manufacture repetitions by relabelling one execution.
    # Metric/result bytes may be equal for deterministic runs, but each
    # complete run must carry a distinct sealed attempt receipt.  The receipt
    # is reopened/bound by the bundle verifier below; this map enforces
    # cross-slot/system uniqueness before any claim can be promoted.
    execution_receipts_seen: dict[str, str] = {}
    execution_campaigns_seen: dict[str, str] = {}
    ape_by_dataset: dict[str, dict[str, list[float]]] = {
        name: {dataset: [] for dataset in expected_sequences}
        for name in required_systems}
    runtime_failures: list[str] = []
    phase_v2_failures: list[str] = []
    phase_versions_seen: set[str] = set()
    safety_failures: list[str] = []
    map_rows: dict[str, dict[str, list[dict[str, float]]]] = {
        name: {dataset: [] for dataset in expected_sequences}
        for name in required_systems}
    for system, rows in run_records.items():
        for key, run in rows.items():
            dataset, index = key
            partition = dataset_to_partition.get(dataset)
            slot = (fresh_slots_by_sequence.get(dataset)
                    if partition == 'fresh' else
                    historical_slots_by_sequence.get(dataset))
            run_identity = run.get('dataset_identity')
            identity_fields = {
                'input_manifest_sha256': 'input_manifest_sha256',
                'reference_sha256': 'ground_truth_sha256',
                'calibration_sha256': 'calibration_archive_sha256',
            }
            if partition == 'fresh':
                identity_fields = {
                    'selection_receipt_sha256': 'selection_receipt_sha256',
                    **identity_fields,
                }
            if not isinstance(slot, dict) or not isinstance(run_identity, dict):
                incomplete.append(
                    f'{system}: {partition or "unknown"} {dataset}/{index} '
                    'dataset_identity is missing')
                complete = False
                continue
            identity_failed = False
            for observed_field, profile_field in identity_fields.items():
                observed = run_identity.get(observed_field, _MISSING)
                expected = slot.get(profile_field, _MISSING)
                if observed is _MISSING or expected is _MISSING or expected is None:
                    incomplete.append(
                        f'{system}: {dataset}/{index} missing dataset identity {observed_field}')
                    identity_failed = True
                    continue
                try:
                    _v2_sha256(observed,
                               f'{system}: {dataset}/{index} {observed_field}')
                    _v2_sha256(
                        expected,
                        f'profile {partition} slot {dataset} {profile_field}')
                except ValueError as exc:
                    errors.append(str(exc))
                    identity_failed = True
                    continue
                if observed.lower() != expected.lower():
                    errors.append(
                        f'{system}: {dataset}/{index} dataset identity mismatch: '
                        f'{observed_field}')
                    identity_failed = True
            if identity_failed:
                complete = False
                continue
            required_status = ('complete', 'process_exit_status',
                               'trajectory_complete', 'sequence_failure',
                               'catastrophic_failure', 'verified_false_loops')
            if any(field not in run for field in required_status):
                incomplete.append(f'{system}: {dataset}/{index} missing completion fields')
                complete = False
                continue
            is_complete = run.get('complete') is True
            if (not isinstance(run.get('process_exit_status'), int) or
                    isinstance(run.get('process_exit_status'), bool)):
                errors.append(f'{system}: {dataset}/{index} invalid process_exit_status')
                continue
            if not isinstance(run.get('trajectory_complete'), bool):
                errors.append(f'{system}: {dataset}/{index} invalid trajectory_complete')
                continue
            if not isinstance(run.get('sequence_failure'), bool) or not isinstance(
                    run.get('catastrophic_failure'), bool):
                errors.append(f'{system}: {dataset}/{index} invalid failure flags')
                continue
            false_loops = run.get('verified_false_loops')
            if (not isinstance(false_loops, int) or isinstance(false_loops, bool)
                    or false_loops < 0):
                errors.append(f'{system}: {dataset}/{index} invalid false-loop count')
                continue
            if (not is_complete or run.get('process_exit_status') != 0 or
                    run.get('trajectory_complete') is not True or
                    run.get('sequence_failure') is not False or
                    run.get('catastrophic_failure') is not False or
                    false_loops != 0):
                safety_failures.append(f'{system}: {dataset}/{index}')
                complete = False
                continue
            if source_closure_requested:
                try:
                    execution_sha = _v2_sha256(
                        run.get('execution_receipt_sha256'),
                        f'{system}: {dataset}/{index} execution receipt')
                    _v2_sha256(
                        run.get('execution_receipt_file_sha256'),
                        f'{system}: {dataset}/{index} execution receipt file')
                    campaign_id = _v2_sha256(
                        run.get('campaign_id'),
                        f'{system}: {dataset}/{index} campaign identity')
                except ValueError as exc:
                    errors.append(str(exc))
                    complete = False
                    continue
                prior = execution_receipts_seen.get(execution_sha)
                if prior is not None:
                    errors.append(
                        'duplicate execution receipt identity: '
                        f'{execution_sha} reused by {prior} and '
                        f'{system}: {dataset}/{index}')
                    complete = False
                    continue
                execution_receipts_seen[execution_sha] = (
                    f'{system}: {dataset}/{index}')
                prior_campaign = next(iter(execution_campaigns_seen.values()), None)
                if (prior_campaign is not None and campaign_id != prior_campaign):
                    errors.append(
                        'mixed execution campaign identities: '
                        f'{prior_campaign} and {campaign_id}')
                    complete = False
                    continue
                execution_campaigns_seen[f'{system}:{dataset}:{index}'] = campaign_id
            try:
                ape_value = _v2_get(run, 'trajectory.ape_rmse_m')
                rtf_value = _v2_get(run, 'runtime.processing_rtf')
                online_rtf_value = _v2_get(run, 'runtime.online_compute_rtf')
                rss_value = _v2_get(run, 'runtime.peak_rss_mb')
                phase_version = (run.get('runtime') or {}).get(
                    'phase_contract_version')
                if ape_value is _MISSING:
                    raise KeyError('trajectory.ape_rmse_m')
                if rtf_value is _MISSING:
                    raise KeyError('runtime.processing_rtf')
                if phase_version == 'm6a10-online-compute-v2' and \
                        online_rtf_value is _MISSING:
                    raise KeyError('runtime.online_compute_rtf')
                if rss_value is _MISSING:
                    raise KeyError('runtime.peak_rss_mb')
                ape = _v2_finite(ape_value,
                                 f'{system}: {dataset}/{index} APE', positive=True)
                rtf = _v2_finite(rtf_value,
                                 f'{system}: {dataset}/{index} RTF')
                online_rtf = None
                if online_rtf_value is not _MISSING:
                    online_rtf = _v2_finite(
                        online_rtf_value,
                        f'{system}: {dataset}/{index} online compute RTF')
                rss = _v2_finite(rss_value,
                                 f'{system}: {dataset}/{index} RSS', positive=True)
                phase_versions_seen.add(
                    phase_version if isinstance(phase_version, str) else 'missing')
                if phase_version == 'm6a10-online-compute-v2':
                    phase_mode = (run.get('runtime') or {}).get('phase_mode')
                    phase_runtime = run.get('runtime') or {}
                    if phase_mode == 'paced_1x':
                        if phase_runtime.get('paced_followability_passed') is not True:
                            phase_v2_failures.append(
                                f'{system}: {dataset}/{index} paced followability')
                    elif phase_mode == 'unpaced_ack':
                        if phase_runtime.get(
                                'unpaced_throughput_gate_passed') is not True:
                            phase_v2_failures.append(
                                f'{system}: {dataset}/{index} unpaced throughput')
                        if online_rtf is None or online_rtf > configured_unpaced_rtf:
                            phase_v2_failures.append(
                                f'{system}: {dataset}/{index} unpaced online_compute_rtf='
                                f'{online_rtf}')
                    else:
                        phase_v2_failures.append(
                            f'{system}: {dataset}/{index} invalid phase mode')
                else:
                    if rtf > float(policy.get('maximum_processing_rtf', 1.0)):
                        runtime_failures.append(f'{system}: {dataset}/{index} RTF={rtf}')
                mapping = run.get('map', run.get('mapping'))
                if not isinstance(mapping, dict):
                    raise ValueError('map metrics are missing')
                map_row = {
                    'plane_thickness_mean_m': _v2_finite(
                        mapping['plane_thickness_mean_m'], 'map mean', positive=True),
                    'plane_thickness_p95_m': _v2_finite(
                        mapping['plane_thickness_p95_m'], 'map p95', positive=True),
                    'planar_coverage': _v2_finite(
                        mapping['planar_coverage'], 'map coverage', maximum=1.0),
                }
                artifacts = run.get('artifacts')
                if not isinstance(artifacts, dict):
                    raise ValueError('artifacts are missing')
                _v2_sha256(artifacts['trajectory_sha256'], 'trajectory artifact')
                _v2_sha256(artifacts['map_sha256'], 'map artifact')
            except KeyError as exc:
                incomplete.append(f'{system}: {dataset}/{index}: missing {exc}')
                complete = False
                continue
            except (TypeError, ValueError) as exc:
                errors.append(f'{system}: {dataset}/{index}: {exc}')
                continue
            complete_rows[system].append(run)
            ape_by_dataset[system][dataset].append(ape)
            map_rows[system][dataset].append(map_row)
            # RSS is kept in the run for the receipt without inventing a
            # second aggregation; use it for the RSS gate below.
            run['_v2_peak_rss_mb'] = rss

    # Completion is deliberately independent from the performance gates: a
    # complete run with RTF=1.01 is still a recorded complete run and must not
    # disappear from the receipt.  The dedicated RTF check below rejects it.
    all_complete = (complete and not safety_failures and
                    all(len(rows) == len(expected_sequences) * repetitions
                        for rows in complete_rows.values()))
    check('three_complete_runs_and_completion', all_complete, {
        'expected_per_system': len(expected_sequences) * repetitions,
        'complete_per_system': {
            system: len(rows) for system, rows in complete_rows.items()},
        'failed_or_incomplete': safety_failures,
    })
    check('distinct_execution_receipt_per_run',
          (not source_closure_requested) or (
              all_complete and len(execution_receipts_seen) == sum(
                  len(rows) for rows in complete_rows.values())),
          {
              'status': 'REQUIRED_FOR_CLAIM' if source_closure_requested
              else 'NOT_REQUESTED',
              'reopened_unique_receipt_identities': len(execution_receipts_seen),
              'execution_campaign_ids': sorted(set(execution_campaigns_seen.values())),
              'complete_run_count': sum(
                  len(rows) for rows in complete_rows.values()),
              'identity_kind': 'sealed_per_attempt_execution_receipt_sha256_v1',
          })
    only_v2 = phase_versions_seen == {'m6a10-online-compute-v2'}
    mixed_phase_versions = len(phase_versions_seen) > 1
    if only_v2:
        check('processing_rtf_leq_one', all_complete and not phase_v2_failures, {
            'role': 'diagnostic_only_for_v2',
            'failures': phase_v2_failures,
        })
        check('online_compute_rtf_leq_one', all_complete and not phase_v2_failures, {
            'role': 'paced_followability_or_unpaced_ack_throughput',
            'maximum_unpaced_throughput_rtf': configured_unpaced_rtf,
            'failures': phase_v2_failures,
        })
    else:
        check('processing_rtf_leq_one', all_complete and not runtime_failures and
              not mixed_phase_versions, {
                  'maximum': policy.get('maximum_processing_rtf', 1.0),
                  'failures': runtime_failures,
              })
        check('online_compute_rtf_leq_one', all_complete and not runtime_failures and
              not mixed_phase_versions, {
                  'role': 'legacy_wall_diagnostic',
                  'failures': [],
              })
    if mixed_phase_versions:
        errors.append('phase contract versions are mixed within the suite')
    check('zero_catastrophic_or_false_loops', all_complete and not safety_failures,
          {'failures': safety_failures})

    # APE is aggregated within each dataset before the cluster bootstrap.
    dataset_means: dict[str, dict[str, float]] = {name: {} for name in required_systems}
    ape_shape_ok = all_complete
    for system in required_systems:
        for dataset in expected_sequences:
            values = ape_by_dataset[system][dataset]
            if len(values) != repetitions:
                ape_shape_ok = False
                continue
            dataset_means[system][dataset] = sum(values) / repetitions
    fresh_ape_shape_ok = (ape_shape_ok and
                          all(dataset in dataset_means[system]
                              for system in required_systems
                              for dataset in fresh_sequences))
    rival_names = [name for name in required_systems if name != 'ours']
    aggregate = ({system: sum(dataset_means[system][dataset]
                              for dataset in fresh_sequences) / len(fresh_sequences)
                  for system in required_systems}
                 if fresh_ape_shape_ok and fresh_sequences else {})
    best_rival = (min(rival_names, key=lambda name: aggregate[name])
                  if aggregate and rival_names else None)
    improvement = None
    bootstrap = None
    bootstrap_by_rival: dict[str, dict[str, Any]] = {}
    if best_rival is not None:
        rival_mean = aggregate[best_rival]
        if rival_mean <= 0.0:
            errors.append('best rival aggregate APE must be > 0')
        else:
            improvement = 100.0 * (rival_mean - aggregate['ours']) / rival_mean
    if fresh_ape_shape_ok:
        for rival in rival_names:
            rival_value = aggregate.get(rival, 0.0)
            if rival_value <= 0.0:
                errors.append(f'{rival} aggregate APE must be > 0')
                continue
            try:
                bootstrap_by_rival[rival] = _v2_hierarchical_bootstrap(
                    {dataset: {
                        'ours': ape_by_dataset['ours'][dataset],
                        'rival': ape_by_dataset[rival][dataset],
                    } for dataset in fresh_sequences},
                    int(policy.get('bootstrap_seed', _V2_BOOTSTRAP_SEED)),
                    int(policy.get('bootstrap_samples', _V2_BOOTSTRAP_SAMPLES)))
            except (TypeError, ValueError) as exc:
                errors.append(f'{rival} bootstrap: {exc}')
    bootstrap = bootstrap_by_rival.get(best_rival) if best_rival else None
    required_improvement = float(policy.get(
        'minimum_aggregate_ape_improvement_percent', 10.0))
    # Percentages at the policy boundary are expected to be reproducible from
    # decimal receipts; tolerate only the final floating-point ulps, not a
    # meaningful sub-threshold result.
    improvement_gate = (improvement is not None and
                        improvement + 1.0e-9 >= required_improvement)
    check('aggregate_ape_improvement', fresh_ape_shape_ok and improvement_gate, {
        'aggregate_ape_m': aggregate,
        'best_rival': best_rival,
        'improvement_percent': improvement,
        'required_percent': required_improvement,
    })
    check('dataset_cluster_bootstrap_95_superiority',
          fresh_ape_shape_ok and bool(bootstrap_by_rival) and all(
              result['superiority'] for result in bootstrap_by_rival.values()),
          {'best_rival': best_rival, 'all_rivals': bootstrap_by_rival,
           'reason': 'every rival must have a positive 95% lower bound'})

    # A strong aggregate must not hide a catastrophic sequence.  For every
    # dataset, compare ours with that sequence's best (lowest-APE) rival and
    # apply the primary-regression budget independently.
    sequence_rows: dict[str, Any] = {}
    sequence_gate = ape_shape_ok
    primary_regression_limit = float(policy.get(
        'maximum_primary_regression_percent', 2.0))
    for dataset in expected_sequences:
        if dataset not in dataset_means.get('ours', {}) or any(
                dataset not in dataset_means.get(rival, {}) for rival in rival_names):
            sequence_gate = False
            sequence_rows[dataset] = {'pass': False, 'reason': 'APE missing'}
            continue
        sequence_best = min(rival_names,
                            key=lambda rival: dataset_means[rival][dataset])
        rival_value = dataset_means[sequence_best][dataset]
        ours_value = dataset_means['ours'][dataset]
        if rival_value <= 0.0:
            errors.append(f'{dataset} best rival APE must be > 0')
            sequence_gate = False
            sequence_rows[dataset] = {'pass': False, 'reason': 'zero rival APE'}
            continue
        regression = 100.0 * (ours_value - rival_value) / rival_value
        passed = regression <= primary_regression_limit + 1.0e-9
        sequence_rows[dataset] = {
            'pass': passed,
            'best_rival': sequence_best,
            'ours_ape_m': ours_value,
            'best_rival_ape_m': rival_value,
            'regression_percent': regression,
            'maximum_percent': primary_regression_limit,
        }
        sequence_gate = sequence_gate and passed
    check('per_sequence_primary_non_regression', sequence_gate, {
        'maximum_regression_percent': primary_regression_limit,
        'sequences': sequence_rows,
    })

    # Memory and map quality remain required evidence for every system.  Map
    # comparisons are against every rival, not only the best-APE rival.
    rss_max: dict[str, float] = {}
    for system, rows in complete_rows.items():
        if rows:
            rss_max[system] = max(row['_v2_peak_rss_mb'] for row in rows)
    resource_declared = any(
        isinstance(run, dict) and any(
            key in run for key in ('resource_evidence', 'resource_receipt',
                                   'resource'))
        for rows in run_records.values() for run in rows.values())
    memory_claim_requested = bool(
        require_bundle or source.get('claim_eligible') is True or
        source.get('evidence_bundle') is not None or resource_declared)
    memory_policy = policy.get('memory_gate', {})
    if not isinstance(memory_policy, dict):
        memory_policy = {}
    memory_gate_result = evaluate_memory_gate(
        {system: list(rows.values()) for system, rows in run_records.items()},
        required_systems=required_systems,
        expected_sequences=expected_sequences,
        repetitions=repetitions,
        policy=memory_policy,
        best_rival=best_rival,
        provenance_by_system=provenance_values,
        claim_requested=memory_claim_requested)
    if memory_claim_requested:
        check('peak_rss_non_regression', memory_gate_result.get('pass') is True,
              memory_gate_result)
        if not memory_gate_result.get('pass'):
            memory_messages = [
                'memory gate: ' + str(item)
                for item in memory_gate_result.get('errors', [])]
            if memory_gate_result.get('status') == 'NOT_READY':
                incomplete.extend(memory_messages)
            else:
                errors.extend(memory_messages)
    else:
        # Preserve the legacy aggregate as a report-only diagnostic.  It is
        # never sufficient for a claim: the strict per-run receipt gate above
        # activates as soon as a claim or resource declaration is present.
        rss_gate = bool(
            best_rival and 'ours' in rss_max and best_rival in rss_max and
            rss_max['ours'] <= rss_max[best_rival] * float(
                policy.get('maximum_peak_rss_ratio_to_rival', 1.20)))
        check('peak_rss_non_regression', all_complete and rss_gate, {
            'maximum_ratio': policy.get('maximum_peak_rss_ratio_to_rival', 1.20),
            'max_mb': rss_max,
            'best_rival': best_rival,
            'status': 'LEGACY_REPORT_ONLY',
        })
    check('memory_resource_receipt_gate', memory_gate_result.get('pass') is True,
          memory_gate_result)
    tolerance = float(policy.get('maximum_mapping_regression_percent', 2.0)) / 100.0
    map_checks: dict[str, Any] = {}
    map_gate = all_complete
    for rival in rival_names:
        per_dataset: dict[str, Any] = {}
        rival_gate = True
        for dataset in expected_sequences:
            rows_for_ours = map_rows['ours'][dataset]
            rows_for_rival = map_rows[rival][dataset]
            if len(rows_for_ours) != len(rows_for_rival) or not rows_for_ours:
                rival_gate = False
                per_dataset[dataset] = {
                    'pass': False, 'reason': 'map metrics missing'}
                continue
            ours = {
                'mean': max(row['plane_thickness_mean_m'] for row in rows_for_ours),
                'p95': max(row['plane_thickness_p95_m'] for row in rows_for_ours),
                'coverage': min(row['planar_coverage'] for row in rows_for_ours),
            }
            other = {
                'mean': max(row['plane_thickness_mean_m'] for row in rows_for_rival),
                'p95': max(row['plane_thickness_p95_m'] for row in rows_for_rival),
                'coverage': min(row['planar_coverage'] for row in rows_for_rival),
            }
            rows = {
                'mean': ours['mean'] <= other['mean'] * (1.0 + tolerance),
                'p95': ours['p95'] <= other['p95'] * (1.0 + tolerance),
                'coverage': ours['coverage'] >= other['coverage'] * (1.0 - tolerance),
            }
            per_dataset[dataset] = {
                'pass': all(rows.values()), 'checks': rows,
                'ours_worst': ours, 'rival_worst': other}
            rival_gate = rival_gate and all(rows.values())
        map_checks[rival] = {'pass': rival_gate, 'datasets': per_dataset}
        map_gate = map_gate and rival_gate
    check('mapping_non_regression', map_gate, {
        'tolerance_percent': 100.0 * tolerance, 'comparisons': map_checks})

    # A claim-eligible receipt must bind one canonical, deterministic evidence
    # bundle before it can be promoted.  Existing report-only receipts and
    # metric fixtures intentionally remain compatible: absence of a bundle is
    # recorded as NOT_REQUESTED and does not claim eligibility.  As soon as a
    # producer declares ``claim_eligible`` or ``evidence_bundle``, however,
    # the filesystem verifier is mandatory and fail-closed.
    bundle_declaration = source.get('evidence_bundle')
    bundle_requested = (
        require_bundle or source.get('claim_eligible') is True or
        bundle_declaration is not None)
    bundle_result: dict[str, Any] | None = None
    if bundle_requested:
        bundle_ok = isinstance(bundle_declaration, dict)
        if not bundle_ok:
            errors.append(
                'claim-eligible evidence must declare evidence_bundle mapping')
            bundle_result = {
                'status': 'FAIL_CLOSED', 'pass': False,
                'claim_eligible': False,
                'errors': ['evidence_bundle declaration is missing or malformed'],
            }
        else:
            bundle_root = bundle_declaration.get('root')
            bundle_manifest = bundle_declaration.get('manifest_path')
            if not isinstance(bundle_root, str) or not bundle_root:
                errors.append('evidence_bundle.root is missing')
                bundle_ok = False
            if bundle_manifest is not None and not isinstance(bundle_manifest, str):
                errors.append('evidence_bundle.manifest_path must be relative text')
                bundle_ok = False
            if bundle_ok:
                provenance_revisions = {
                    system: value.get('revision')
                    for system, value in provenance_values.items()
                    if isinstance(value, dict) and isinstance(value.get('revision'), str)
                }
                scorer_fingerprints = {
                    value.get('scorer_fingerprint')
                    for value in provenance_values.values()
                    if isinstance(value, dict) and
                    isinstance(value.get('scorer_fingerprint'), str)
                }
                expected_scorer_fingerprint = (
                    next(iter(scorer_fingerprints))
                    if len(scorer_fingerprints) == 1 else None)
                expected_run_artifacts: list[dict[str, Any]] = []
                for system, rows in complete_rows.items():
                    for run in rows:
                        run_artifacts = run.get('artifacts')
                        resource_document = None
                        for resource_key in (
                                'resource_evidence', 'resource_receipt', 'resource'):
                            candidate = run.get(resource_key)
                            if isinstance(candidate, Mapping):
                                resource_document = candidate
                                break
                        resource_sha = (
                            resource_document.get(
                                'receipt_sha256',
                                resource_document.get('evidence_sha256'))
                            if isinstance(resource_document, Mapping) else None)
                        expected_run_artifacts.append({
                            'system': system,
                            'dataset': run.get('dataset'),
                            'run_index': run.get('run_index'),
                            'trajectory_sha256': (
                                run_artifacts.get('trajectory_sha256')
                                if isinstance(run_artifacts, Mapping) else None),
                            'map_sha256': (
                                run_artifacts.get('map_sha256')
                                if isinstance(run_artifacts, Mapping) else None),
                            'resource_sha256': resource_sha,
                            'execution_receipt_sha256': run.get(
                                'execution_receipt_sha256'),
                            'execution_receipt_file_sha256': run.get(
                                'execution_receipt_file_sha256'),
                            'campaign_id': run.get('campaign_id'),
                            'score_sha256': _v2_score_artifact_sha256(system, run),
                        })
                bundle_result = verify_evidence_bundle(
                    Path(bundle_root),
                    Path(bundle_manifest) if bundle_manifest is not None else None,
                    contract=contract,
                    profile={'competitive_slam_profile': contract},
                    expected_profile_sha256=computed_profile_sha,
                    expected_profile_name=contract.get('name'),
                    expected_scorer_fingerprint=expected_scorer_fingerprint,
                    expected_revision_by_system=provenance_revisions or None,
                    expected_manifest_sha256=bundle_declaration.get(
                        'manifest_sha256'),
                    expected_run_artifacts=expected_run_artifacts)
            else:
                bundle_result = {
                    'status': 'FAIL_CLOSED', 'pass': False,
                    'claim_eligible': False,
                    'errors': ['bundle declaration is incomplete'],
                }
        check('evidence_bundle_integrity', bool(bundle_result and
              bundle_result.get('pass')), bundle_result or {})
        if bundle_result and not bundle_result.get('pass'):
            errors.extend('evidence bundle: ' + str(item)
                          for item in bundle_result.get('errors', []))
    else:
        check('evidence_bundle_integrity', True, {
            'status': 'NOT_REQUESTED',
            'claim_eligible': False,
            'reason': 'report-only evidence did not declare a bundle',
        })

    # A schema-valid receipt with an entirely absent contract/system payload
    # is an incomplete submission, not malformed evidence.  Keep the detailed
    # missing-field diagnostics below, but do not turn this common pre-run
    # smoke case into INVALID merely because the now-frozen profile allows the
    # deeper per-slot checks to run.
    minimal_submission = (
        schema_ok and not source.get('contract') and not source.get('systems'))
    status = ('INCOMPLETE' if minimal_submission else
              'INVALID' if errors else ('INCOMPLETE' if incomplete else (
                  'PASS' if all(check_row['pass'] for check_row in checks.values())
                  else 'FAIL')))
    claim_eligible = bool(
        status == 'PASS' and bundle_result is not None and
        bundle_result.get('pass'))
    return {
        'schema_version': 2,
        'evidence_kind': 'competitive_slam_victory_evidence_receipt',
        'status': status,
        'pass': status == 'PASS',
        'claim_eligible': claim_eligible,
        'errors': errors + incomplete,
        'checks': checks,
        'policy': {
            'repetitions': repetitions,
            'minimum_aggregate_ape_improvement_percent': policy.get(
                'minimum_aggregate_ape_improvement_percent', 10.0),
            'maximum_primary_regression_percent': policy.get(
                'maximum_primary_regression_percent', 2.0),
            'minimum_fresh_holdout_slots': policy.get(
                'minimum_fresh_holdout_slots', 3),
            'bootstrap_seed': policy.get('bootstrap_seed', _V2_BOOTSTRAP_SEED),
            'bootstrap_samples': policy.get('bootstrap_samples', _V2_BOOTSTRAP_SAMPLES),
        },
        'aggregate_ape_m': aggregate,
        'best_rival': best_rival,
        'memory_gate': memory_gate_result,
        'rival_source_closure': rival_source_closure_result,
        'dataset_source_closure': dataset_source_closure_result,
        'aggregate_ape_improvement_percent': improvement,
        'bootstrap_ci': bootstrap,
        'bootstrap_ci_by_rival': bootstrap_by_rival,
        'partitions': {
            'historical': {
                'role': 'regression',
                'datasets': historical_sequences,
                'required_runs_per_system': repetitions,
                'included_in_aggregate_ape': False,
            },
            'fresh': {
                'role': 'primary_fresh',
                'datasets': fresh_sequences,
                'required_runs_per_system': repetitions,
                'included_in_aggregate_ape': True,
                'profile_ready': fresh_slots_ready,
            },
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--gate', type=Path, action='append')
    mode.add_argument('--evidence', type=Path,
                      help='schema-v2 victory evidence YAML or JSON')
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--yaml-output', type=Path,
                        help='optional schema-v2 YAML receipt path')
    parser.add_argument(
        '--claim-eligible', action='store_true',
        help='require and verify the declared canonical evidence bundle')
    args = parser.parse_args()
    profile_bytes = args.profile.read_bytes()
    profile_file_sha256 = hashlib.sha256(profile_bytes).hexdigest()
    profile_document = yaml.safe_load(profile_bytes)
    if not isinstance(profile_document, dict):
        raise ValueError('competitive profile must be a YAML mapping')
    profile_sha256 = canonical_profile_sha256(profile_document)
    contract = profile_document['competitive_slam_profile']
    if args.evidence is not None:
        evidence_bytes = args.evidence.read_bytes()
        evidence_sha256 = hashlib.sha256(evidence_bytes).hexdigest()
        evidence = yaml.safe_load(evidence_bytes)
        result = evaluate_evidence_v2(
            evidence, contract, require_bundle=args.claim_eligible)
        result['receipt_identity'] = {
            'profile_sha256': profile_sha256,
            'profile_sha256_kind': PROFILE_CANONICAL_HASH_KIND,
            'profile_file_sha256': profile_file_sha256,
            'evidence_sha256': evidence_sha256,
        }
        declared_profile_sha = _v2_get(
            evidence if isinstance(evidence, dict) else {},
            'contract.profile_sha256')
        if declared_profile_sha is not _MISSING:
            try:
                _v2_sha256(declared_profile_sha, 'contract.profile_sha256')
                if declared_profile_sha.lower() != profile_sha256:
                    raise ValueError('contract.profile_sha256 does not match profile')
            except ValueError as exc:
                result['errors'].append(str(exc))
                result['status'] = 'INVALID'
                result['pass'] = False
        if args.claim_eligible and not result.get('claim_eligible', False):
            result['errors'].append(
                'claim-eligible CLI mode requires a passing evidence bundle')
            result['status'] = 'FAIL_CLOSED'
            result['pass'] = False
    else:
        if not args.gate:
            parser.error('--gate must be supplied in legacy mode')
        gates = [json.loads(path.read_text()) for path in args.gate]
        result = evaluate(gates, contract)
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    if args.evidence is not None:
        yaml_output = args.yaml_output or args.output.with_suffix('.yaml')
        if yaml_output.exists():
            raise ValueError(f'refusing to overwrite: {yaml_output}')
        yaml_output.parent.mkdir(parents=True, exist_ok=True)
        yaml_output.write_text(yaml.safe_dump(result, sort_keys=True))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError,
            yaml.YAMLError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
