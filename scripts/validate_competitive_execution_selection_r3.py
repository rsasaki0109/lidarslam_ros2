#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""Validate the additive, non-promoting competitive execution-selection r3 candidate.

This validator is deliberately separate from ``check_competitive_execution_selection``.
The latter remains the fail-closed checker for the historical receipt named by the
active profile.  This module validates an additive candidate which records current
source identities and unresolved external evidence without making the candidate
claim-eligible.  It never opens a dataset, ground-truth file, scorer, container, or
machine evidence volume.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Any, Mapping

_SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(_SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(_SOURCE_ROOT))

from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (  # noqa: E402
    current_rival_source_closure_identity,
)
from scripts.competitive_identity_hash import (  # noqa: E402
    canonical_profile_sha256, PROFILE_CANONICAL_HASH_KIND,
)
import yaml  # noqa: E402


ROOT = _SOURCE_ROOT
CANDIDATE_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_2026-08-r3-candidate.json')
CANDIDATE_SIDECAR_REL = CANDIDATE_REL + '.sha256'
PROFILE_REL = 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
R2_REL = 'configs/slam_benchmark_profiles/competitive_execution_selection_2026-08-r2.yaml'
HISTORICAL_REL = 'configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml'
SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_r3_candidate_v1.schema.json')
SHA256_RE = re.compile(r'^[0-9a-f]{64}$')
IMAGE_RE = re.compile(r'^sha256:[0-9a-f]{64}$')
SYSTEMS = ('ours', 'glim', 'fast_livo2')
THREAD_KEYS = (
    'cpu_affinity', 'max_threads', 'omp_num_threads',
    'openblas_num_threads', 'mkl_num_threads', 'tbb_num_threads',
    'accelerator_policy')
THREAD_VALUES = {
    'cpu_affinity': [0, 1, 2, 3, 4, 5, 6, 7],
    'max_threads': 8,
    'omp_num_threads': 8,
    'openblas_num_threads': 8,
    'mkl_num_threads': 8,
    'tbb_num_threads': 8,
    'accelerator_policy': 'cpu_only',
}
THREAD_HASH = hashlib.sha256(
    json.dumps(THREAD_VALUES, sort_keys=True, separators=(',', ':')).encode()
).hexdigest()

# These are producer identities, not evidence of execution.  Their bytes are
# re-opened and hashed on every validation; a stale candidate is invalid.
SOURCE_BINDING_PATHS: dict[str, dict[str, str]] = {
    'runner': {
        'ours': 'scripts/run_ours_competitive_benchmark.py',
        'ours_entrypoint': 'scripts/ours_container_gt_blind_run.sh',
        'glim': 'scripts/run_glim_benchmark.py',
        'glim_entrypoint': 'scripts/glim_container_run.sh',
        'fast_livo2': 'scripts/run_fast_livo2_benchmark.py',
        'fast_livo2_entrypoint': 'scripts/fast_livo2_container_run.sh',
        'gt_blind_driver': 'scripts/run_competitive_gt_blind_benchmark.py',
    },
    'scorer': {
        'trajectory': 'scripts/ape_from_tum.py',
        'map_quality': 'scripts/run_map_quality_check.sh',
        'paired_map_regression': 'scripts/check_map_quality_regression.py',
        'sequence_gate': 'scripts/evaluate_competitive_sequence_gate.py',
        'suite_gate': 'scripts/evaluate_competitive_suite_gate.py',
        'result_composer': 'scripts/compose_competitive_result.py',
    },
    'memory': {
        'sampler': 'scripts/sample_container_process_rss.py',
        'helper': 'scripts/container_memory_evidence.py',
        'gate': 'scripts/competitive_memory_gate.py',
    },
}

# Profile pointers are explicit so a stale or cross-campaign profile projection
# cannot be made to look like an image identity merely by copying its digest.
PROFILE_SYSTEM_PROJECTIONS = {
    'ours': {
        'image_pointer': (
            'competitive_slam_profile.runtime_policy.phase_contract_v2.'
            'ours_m6a10_v2a_unpaced_ack_fixed10_v10.execution.image_digest'),
    },
    'glim': {
        'image_pointer': (
            'competitive_slam_profile.m6a10_glim_v2b_fixed10_v4.build.'
            'image_digest'),
    },
    'fast_livo2': {
        'image_pointer': (
            'competitive_slam_profile.m6a10_fast_livo2_v2c.retry_v5.image.'
            'image_digest'),
    },
}


class CandidateError(ValueError):
    """A malformed, stale, or cross-selection candidate."""


def _canonical(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def canonical_candidate_sha256(value: Mapping[str, Any]) -> str:
    """Hash candidate content excluding the non-cyclic candidate identity."""
    payload = {key: item for key, item in value.items()
               if key != 'candidate_identity_sha256'}
    return hashlib.sha256(_canonical(payload)).hexdigest()


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _require_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CandidateError(f'{label} must be an object')
    return value


def _require_sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA256_RE.fullmatch(value) is None:
        raise CandidateError(f'{label} must be lowercase 64-hex SHA-256')
    return value


def _safe_relative(root: Path, relative: Any, label: str) -> Path:
    if not isinstance(relative, str) or not relative or relative.startswith('/'):
        raise CandidateError(f'{label} must be a repository-relative path')
    candidate = Path(relative)
    if candidate.is_absolute() or '..' in candidate.parts:
        raise CandidateError(f'{label} escapes the repository')
    current = root
    for component in candidate.parts:
        current = current / component
        if current.is_symlink():
            raise CandidateError(f'{label} contains a symlink: {relative}')
    if not current.is_file() or current.is_symlink():
        raise CandidateError(f'{label} is not a regular file: {relative}')
    if current.stat().st_nlink != 1:
        raise CandidateError(f'{label} is not single-link: {relative}')
    return current


def _file_descriptor(root: Path, value: Any, label: str) -> None:
    item = _require_mapping(value, label)
    expected = _require_sha(item.get('sha256'), f'{label}.sha256')
    path = _safe_relative(root, item.get('path'), f'{label}.path')
    actual = sha256_file(path)
    if actual != expected:
        raise CandidateError(f'{label}.sha256 is stale for {item.get("path")}')


def _json_canonical_sha(value: Any) -> str:
    return _sha256_bytes(_canonical(value))


def _get_path(document: Any, dotted: str) -> Any:
    value = document
    for part in dotted.split('.'):
        if not isinstance(value, Mapping) or part not in value:
            raise CandidateError(f'profile pointer is missing: {dotted}')
        value = value[part]
    return value


def _load_yaml(path: Path, label: str) -> Mapping[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise CandidateError(f'{label} cannot be parsed: {exc}') from exc
    return _require_mapping(value, label)


def _read_sidecar(root: Path, relative: str, candidate_file_sha: str) -> None:
    sidecar = _safe_relative(root, relative, 'candidate sidecar')
    text = sidecar.read_text(encoding='ascii')
    expected = f'{candidate_file_sha}  {Path(CANDIDATE_REL).name}\n'
    if text != expected:
        raise CandidateError('candidate sidecar does not bind exact candidate bytes')


def _validate_source_bindings(candidate: Mapping[str, Any], root: Path) -> None:
    bindings = _require_mapping(candidate.get('source_bindings'), 'source_bindings')
    if set(bindings) != set(SOURCE_BINDING_PATHS):
        raise CandidateError('source_bindings role set is not exact')
    for role, expected_paths in SOURCE_BINDING_PATHS.items():
        values = _require_mapping(bindings.get(role), f'source_bindings.{role}')
        if set(values) != set(expected_paths):
            raise CandidateError(f'source_bindings.{role} producer set is not exact')
        for producer, expected_path in expected_paths.items():
            descriptor = _require_mapping(values.get(producer),
                                          f'source_bindings.{role}.{producer}')
            if set(descriptor) != {'path', 'sha256'}:
                raise CandidateError(
                    f'source_bindings.{role}.{producer} has extra or missing fields')
            if descriptor['path'] != expected_path:
                raise CandidateError(
                    f'source_bindings.{role}.{producer} path is not current contract path')
            _file_descriptor(root, descriptor,
                             f'source_bindings.{role}.{producer}')


def _validate_profile_binding(candidate: Mapping[str, Any], root: Path,
                              profile: Mapping[str, Any]) -> None:
    binding = _require_mapping(candidate.get('profile_binding'), 'profile_binding')
    if set(binding) != {'path', 'file_sha256', 'canonical_sha256',
                        'canonical_hash_kind'}:
        raise CandidateError('profile_binding shape is not exact')
    if binding['path'] != PROFILE_REL:
        raise CandidateError('candidate does not bind the active profile path')
    _file_descriptor(root, {'path': binding['path'], 'sha256': binding['file_sha256']},
                     'profile_binding.file')
    expected_canonical = canonical_profile_sha256(profile)
    if binding['canonical_hash_kind'] != PROFILE_CANONICAL_HASH_KIND or \
            binding['canonical_sha256'] != expected_canonical:
        raise CandidateError('profile canonical identity is stale or wrong kind')


def _validate_selection_binding(candidate: Mapping[str, Any], root: Path,
                                profile: Mapping[str, Any]) -> None:
    binding = _require_mapping(candidate.get('r2_selection_binding'),
                               'r2_selection_binding')
    required = {'path', 'file_sha256', 'canonical_sha256', 'selection_id',
                'status', 'closure_id', 'closure_revision',
                'closure_identity_hash_kind', 'closure_identity_sha256'}
    if set(binding) != required:
        raise CandidateError('r2_selection_binding shape is not exact')
    if binding['path'] != R2_REL or binding['status'] != 'CURRENT':
        raise CandidateError('candidate is not bound to the current r2 selection')
    path = _safe_relative(root, R2_REL, 'r2 selection')
    if sha256_file(path) != binding['file_sha256']:
        raise CandidateError('r2 selection file SHA is stale')
    selection = _load_yaml(path, 'r2 selection')
    if _json_canonical_sha(selection) != binding['canonical_sha256']:
        raise CandidateError('r2 selection canonical identity is stale')
    for key in ('selection_id', 'closure_id', 'closure_revision'):
        if selection.get(key) != binding[key]:
            raise CandidateError(f'r2 selection {key} does not match candidate')
    if selection.get('closure_identity_sha256') != binding['closure_identity_sha256']:
        raise CandidateError('r2 selection closure identity differs')
    contract = _require_mapping(profile.get('competitive_slam_profile'),
                                'profile competitive_slam_profile')
    evidence = _require_mapping(contract.get('evidence_gate_v2'),
                                'profile evidence_gate_v2')
    closure = _require_mapping(evidence.get('rival_source_closure'),
                               'profile rival source closure')
    active = _require_mapping(closure.get('active_selection'),
                              'profile active selection')
    for key in ('selection_id', 'path', 'sha256', 'status'):
        if active.get(key) != binding.get({
                'selection_id': 'selection_id', 'path': 'path',
                'sha256': 'file_sha256', 'status': 'status'}[key]):
            raise CandidateError(f'profile active selection {key} differs from r2')
    observed = current_rival_source_closure_identity(profile, root=root)
    for key in ('closure_id', 'closure_revision', 'closure_identity_hash_kind',
                'closure_identity_sha256', 'selection_id', 'selection_path',
                'selection_sha256'):
        expected_key = {
            'closure_identity_hash_kind': 'closure_identity_hash_kind',
            'closure_identity_sha256': 'closure_identity_sha256',
            'selection_path': 'path', 'selection_sha256': 'file_sha256',
        }.get(key, key)
        if observed.get(key) != binding.get(expected_key):
            raise CandidateError(f'current rival closure {key} differs from r2')


def _validate_lineage(candidate: Mapping[str, Any], root: Path) -> None:
    lineage = _require_mapping(candidate.get('prior_campaign_lineage'),
                               'prior_campaign_lineage')
    required = {'path', 'file_sha256', 'receipt_kind', 'recorded_status',
                'promotion_status', 'superseded_by'}
    if set(lineage) != required:
        raise CandidateError('prior_campaign_lineage shape is not exact')
    if lineage['path'] != HISTORICAL_REL:
        raise CandidateError('historical lineage path is not immutable receipt path')
    _file_descriptor(root, {'path': lineage['path'], 'sha256': lineage['file_sha256']},
                     'prior_campaign_lineage.file')
    old = _load_yaml(root / HISTORICAL_REL, 'historical execution selection')
    if old.get('receipt_kind') != lineage['receipt_kind'] or \
            lineage['receipt_kind'] != 'competitive_execution_selection':
        raise CandidateError('historical lineage receipt kind is wrong')
    if lineage['recorded_status'] != old.get('status') or \
            lineage['promotion_status'] != 'REJECTED_HISTORICAL_INVALID':
        raise CandidateError('historical receipt is not explicitly rejected')
    if lineage['superseded_by'] != 'competitive-execution-selection-2026-08-r2':
        raise CandidateError('historical lineage does not name r2 supersession')


def _validate_thread_policy(candidate: Mapping[str, Any], profile: Mapping[str, Any]) -> None:
    binding = _require_mapping(candidate.get('thread_policy_binding'),
                               'thread_policy_binding')
    if set(binding) != {'status', 'required_keys', 'values', 'canonical_sha256'}:
        raise CandidateError('thread_policy_binding shape is not exact')
    if binding['status'] != 'RECORDED_NOT_READY_EXTERNAL_MACHINE':
        raise CandidateError('thread policy status must remain external NOT_READY')
    if binding['required_keys'] != list(THREAD_KEYS):
        raise CandidateError('thread policy must contain exactly seven required keys')
    if binding['values'] != THREAD_VALUES or binding['canonical_sha256'] != THREAD_HASH:
        raise CandidateError('thread policy values/canonical identity drifted')
    evidence = profile['competitive_slam_profile']['evidence_gate_v2']
    if evidence.get('thread_policy_required_keys') != list(THREAD_KEYS):
        raise CandidateError('profile thread policy required-key contract drifted')


def _validate_external_closures(candidate: Mapping[str, Any], root: Path,
                                profile: Mapping[str, Any]) -> None:
    closures = _require_mapping(candidate.get('external_closures'),
                                'external_closures')
    required = {'rival_source', 'dataset_source', 'fresh_holdout_authorization',
                'machine_identity', 'image_identity', 'toolchain_identity'}
    if set(closures) != required:
        raise CandidateError('external closure role set is not exact')
    contract = profile['competitive_slam_profile']['evidence_gate_v2']
    rival = _require_mapping(closures['rival_source'], 'external_closures.rival_source')
    if set(rival) != {'status', 'identity', 'selection_path',
                      'selection_file_sha256', 'legal_status'}:
        raise CandidateError('rival external closure shape is not exact')
    if rival['status'] != 'NOT_READY' or rival['legal_status'] != 'BLOCKED':
        raise CandidateError('rival legal closure was promoted unexpectedly')
    identity = current_rival_source_closure_identity(profile, root=root)
    if rival['identity'] != identity or rival['selection_path'] != R2_REL or \
            rival['selection_file_sha256'] != identity['selection_sha256']:
        raise CandidateError('rival closure identity is stale')
    dataset = _require_mapping(closures['dataset_source'],
                               'external_closures.dataset_source')
    if set(dataset) != {'status', 'section_sha256', 'selection_path',
                        'selection_file_sha256', 'selection_schema_path',
                        'selection_schema_sha256', 'reviewed_pin_status'}:
        raise CandidateError('dataset external closure shape is not exact')
    if dataset['status'] != contract['dataset_source_closure']['status'] or \
            dataset['reviewed_pin_status'] != 'NOT_CONFIGURED':
        raise CandidateError('dataset closure status changed without review')
    if dataset['section_sha256'] != _json_canonical_sha(
            contract['dataset_source_closure']):
        raise CandidateError('dataset closure section identity is stale')
    for key in ('selection_path', 'selection_schema_path'):
        if dataset[key] != contract['dataset_source_closure'][key]:
            raise CandidateError(f'dataset closure {key} differs from profile')
    for key in ('selection_sha256', 'selection_schema_sha256'):
        if dataset[{
                'selection_sha256': 'selection_file_sha256',
                'selection_schema_sha256': 'selection_schema_sha256'}[key]] != \
                contract['dataset_source_closure'][key]:
            raise CandidateError(f'dataset closure {key} differs from profile')
    _file_descriptor(root, {'path': dataset['selection_path'],
                            'sha256': dataset['selection_file_sha256']},
                     'dataset selection')
    _file_descriptor(root, {'path': dataset['selection_schema_path'],
                            'sha256': dataset['selection_schema_sha256']},
                     'dataset selection schema')
    holdout = _require_mapping(closures['fresh_holdout_authorization'],
                               'external_closures.fresh_holdout_authorization')
    if set(holdout) != {'status', 'section_sha256', 'external_attestation_status',
                        'required_systems', 'run_count'}:
        raise CandidateError('fresh holdout closure shape is not exact')
    auth = contract['fresh_holdout_authorization']
    if holdout['status'] != auth['status'] or \
            holdout['external_attestation_status'] != auth['external_attestation']['status'] or \
            holdout['required_systems'] != list(SYSTEMS) or \
            holdout['run_count'] != auth['run_count']:
        raise CandidateError('fresh holdout authorization status/contract drifted')
    if holdout['section_sha256'] != _json_canonical_sha(auth):
        raise CandidateError('fresh holdout section identity is stale')
    machine = _require_mapping(closures['machine_identity'],
                               'external_closures.machine_identity')
    if set(machine) != {'status', 'path', 'sha256', 'reason'} or \
            machine['status'] != 'NOT_READY' or machine['path'] is not None or \
            machine['sha256'] is not None:
        raise CandidateError('machine identity must remain an unresolved placeholder')
    image = _require_mapping(closures['image_identity'],
                             'external_closures.image_identity')
    if set(image) != {'status', 'systems', 'required_evidence'} or \
            image['status'] != 'NOT_READY_EXTERNAL_REVALIDATION' or \
            image['systems'] != list(SYSTEMS):
        raise CandidateError('image identity is not a strict external placeholder')
    toolchain = _require_mapping(closures['toolchain_identity'],
                                 'external_closures.toolchain_identity')
    if set(toolchain) != {'status', 'required_evidence', 'fingerprints'} or \
            toolchain['status'] != 'NOT_READY_EXTERNAL_REVALIDATION' or \
            toolchain['fingerprints'] != {system: None for system in SYSTEMS}:
        raise CandidateError('toolchain identity placeholder was promoted')


def _validate_system_bindings(candidate: Mapping[str, Any], profile: Mapping[str, Any]) -> None:
    systems = _require_mapping(candidate.get('system_bindings'), 'system_bindings')
    if set(systems) != set(SYSTEMS):
        raise CandidateError('system binding set is not exact')
    for system in SYSTEMS:
        item = _require_mapping(systems[system], f'system_bindings.{system}')
        if set(item) != {
                'profile_image_pointer', 'image_digest', 'image_status',
                'toolchain_fingerprint', 'toolchain_status', 'release'}:
            raise CandidateError(f'system binding {system} shape is not exact')
        expected_pointer = PROFILE_SYSTEM_PROJECTIONS[system]['image_pointer']
        if item['profile_image_pointer'] != expected_pointer:
            raise CandidateError(f'{system} image profile pointer drifted')
        declared = _get_path(profile, expected_pointer)
        if item['image_digest'] != declared or IMAGE_RE.fullmatch(str(declared)) is None:
            raise CandidateError(f'{system} image digest is not current profile value')
        if item['image_status'] != 'NOT_READY_EXTERNAL_IMAGE_REVALIDATION':
            raise CandidateError(f'{system} image was promoted without inspect evidence')
        if item['toolchain_fingerprint'] is not None or \
                item['toolchain_status'] != 'NOT_READY_EXTERNAL_IMAGE_REVALIDATION':
            raise CandidateError(f'{system} toolchain was promoted without image evidence')
        if item['release'] != 'Release':
            raise CandidateError(f'{system} release is not fixed to Release')


def _validate_blockers(candidate: Mapping[str, Any]) -> None:
    blockers = candidate.get('blockers')
    if not isinstance(blockers, list) or not blockers or \
            any(not isinstance(item, str) or not item for item in blockers):
        raise CandidateError('candidate blockers must be a non-empty string list')
    required = {
        'active_profile_points_to_historical_execution_receipt',
        'historical_selection_receipt_rejected_after_r2_closure_revision',
        'rival_source_closure_not_ready_legal_provenance',
        'dataset_source_closure_not_ready_revalidation_or_pin',
        'fresh_holdout_authorization_not_ready_external_custodian',
        'machine_identity_not_ready_external_execution_artifact',
        'image_identity_not_ready_external_inspect',
        'toolchain_identity_not_ready_external_capture',
        'source_worktree_requires_external_review',
    }
    if set(blockers) != required:
        raise CandidateError('candidate blocker set is incomplete or contains invented status')


def validate_candidate(candidate_path: Path | str = ROOT / CANDIDATE_REL,
                       *, root: Path = ROOT) -> dict[str, Any]:
    """Return a structural validation report; NOT_READY is a valid result."""
    candidate_path = Path(candidate_path)
    if not candidate_path.is_absolute():
        candidate_path = root / candidate_path
    try:
        if root.is_symlink() or not root.is_dir():
            raise CandidateError('candidate repository root must be a real directory')
        requested_relative = candidate_path.relative_to(root)
        if requested_relative.as_posix() != CANDIDATE_REL:
            raise CandidateError('candidate path is not the checked-in r3 candidate')
        candidate_path = _safe_relative(root, CANDIDATE_REL, 'candidate')
        if candidate_path.is_symlink() or candidate_path.stat().st_nlink != 1:
            raise CandidateError('candidate must be a single-link regular file')
        candidate_path = candidate_path.resolve(strict=True)
        relative = candidate_path.relative_to(root.resolve()).as_posix()
        if relative != CANDIDATE_REL:
            raise CandidateError('candidate path is not the checked-in r3 candidate')
        candidate = json.loads(candidate_path.read_text(encoding='utf-8'))
        candidate = _require_mapping(candidate, 'candidate')
        _require_sha(candidate.get('candidate_identity_sha256'),
                     'candidate_identity_sha256')
        if candidate['candidate_identity_sha256'] != canonical_candidate_sha256(candidate):
            raise CandidateError('candidate canonical identity self-hash mismatch')
        file_sha = sha256_file(candidate_path)
        _read_sidecar(root, CANDIDATE_SIDECAR_REL, file_sha)
        if candidate.get('schema_version') != 1 or \
                candidate.get('candidate_kind') != (
                    'competitive_execution_selection_r3_candidate_v1'):
            raise CandidateError('candidate schema identity is invalid')
        if candidate.get('status') != 'NOT_READY' or \
                candidate.get('benchmark_eligible') is not False or \
                candidate.get('claim_eligible') is not False:
            raise CandidateError('candidate must remain non-promoting NOT_READY')
        profile_path = _safe_relative(root, PROFILE_REL, 'profile')
        profile = _load_yaml(profile_path, 'active profile')
        _validate_profile_binding(candidate, root, profile)
        _validate_selection_binding(candidate, root, profile)
        _validate_lineage(candidate, root)
        _validate_source_bindings(candidate, root)
        _validate_thread_policy(candidate, profile)
        _validate_external_closures(candidate, root, profile)
        _validate_system_bindings(candidate, profile)
        _validate_blockers(candidate)
        promotion = _require_mapping(candidate.get('promotion_policy'),
                                     'promotion_policy')
        if promotion != {
                'active_profile_switch': False,
                'historical_receipts_immutable': True,
                'requires_separately_reviewed_ready_receipt': True,
                'requires_profile_canonical_reseal': True,
                'handoff_status': 'UNSIGNED_REVIEW_REQUIRED'}:
            raise CandidateError('promotion policy is not the fixed atomic handoff contract')
        return {
            'status': 'NOT_READY', 'structural_valid': True,
            'benchmark_eligible': False, 'claim_eligible': False,
            'candidate_id': candidate['candidate_id'],
            'candidate_identity_sha256': candidate['candidate_identity_sha256'],
            'candidate_file_sha256': file_sha,
            'blockers': list(candidate['blockers']),
            'active_profile_switched': False,
        }
    except (OSError, UnicodeError, json.JSONDecodeError, yaml.YAMLError,
            KeyError, TypeError, ValueError) as exc:
        return {
            'status': 'INVALID', 'structural_valid': False,
            'benchmark_eligible': False, 'claim_eligible': False,
            'error': str(exc),
        }


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, default=ROOT / CANDIDATE_REL)
    args = parser.parse_args()
    report = validate_candidate(args.candidate)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report['structural_valid'] else 1


if __name__ == '__main__':
    raise SystemExit(_main())
