#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""Build and validate the additive, non-promoting rival-source r3 candidate.

The checked-in r2 profile and selection are immutable inputs.  This candidate
copies the complete r2 closure byte-for-byte as a nested projection and adds a
separate projection of current local recipe bytes.  In particular, the GLIM
and FAST-LIVO2 runner hashes are rebound to the bytes that are actually in the
checkout; the r2 declarations are retained for audit and are never edited.

No remote source, image, dataset, scorer, or legal review packet is opened by
this module.  A current hash is a producer identity, not an execution result.
The candidate therefore remains ``NOT_READY`` and cannot be promoted without
an independently reviewed external packet.
"""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import re
import stat
import sys
from typing import Any, Mapping

_SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(_SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(_SOURCE_ROOT))

from scripts.check_competitive_rival_source_closure import (  # noqa: E402
    canonical_rival_source_closure_identity,
    CLOSURE_IDENTITY_HASH_KIND,
    current_rival_source_closure_identity,
    sha256_file as closure_sha256_file,
    sha256_tree,
)
from scripts.competitive_identity_hash import (  # noqa: E402
    canonical_profile_sha256,
    PROFILE_CANONICAL_HASH_KIND,
)

import yaml  # noqa: E402


ROOT = _SOURCE_ROOT
PROFILE_REL = 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
R2_SELECTION_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_2026-08-r2.yaml')
HISTORICAL_SELECTION_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_2026-08.yaml')
R2_CLOSURE_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_rival_source_closure_v1.schema.json')
LEGAL_CAPTURE_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_rival_legal_provenance_capture_v1.schema.json')
LEGAL_CAPTURE_PRODUCER_REL = 'scripts/capture_competitive_rival_legal_provenance.py'
CANDIDATE_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_rival_source_closure_2026-08-r3-candidate.json')
CANDIDATE_SIDECAR_REL = CANDIDATE_REL + '.sha256'
SELECTION_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_rival_source_closure_2026-08-r3-selection-candidate.json')
SELECTION_SIDECAR_REL = SELECTION_REL + '.sha256'
SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_rival_source_closure_r3_candidate_v1.schema.json')
HANDOFF_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_rival_source_closure_r3_handoff_v1.schema.json')
CANDIDATE_KIND = 'competitive_rival_source_closure_r3_candidate_v1'
SELECTION_KIND = 'competitive_rival_source_closure_r3_selection_candidate_v1'
CANDIDATE_HASH_KIND = 'canonical_competitive_rival_source_closure_r3_candidate_sha256_v1'
SELECTION_HASH_KIND = 'canonical_competitive_rival_source_closure_r3_selection_candidate_sha256_v1'
SHA_RE = re.compile('^[0-9a-f]{64}$')
RELATIVE_RE = re.compile('^(?!/)(?!.*(?:^|/)\\.\\.(?:/|$)).+$')
SYSTEMS = ('glim', 'fast_livo2')
LEGAL_BLOCKERS = (
    'glim_ros2_license_text_missing',
    'fast_livo2_license_declaration_conflict',
    'rpg_vikit_license_artifacts_incomplete',
    'sophus_license_artifact_missing',
    'external_custodian_legal_review_required',
)


class CandidateError(ValueError):
    """A stale, malformed, cross-campaign, or promoting candidate."""


def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def candidate_identity_sha256(value: Mapping[str, Any]) -> str:
    return canonical_sha256({k: v for k, v in value.items()
                             if k != 'candidate_identity_sha256'})


def selection_identity_sha256(value: Mapping[str, Any]) -> str:
    return canonical_sha256({k: v for k, v in value.items()
                             if k != 'selection_identity_sha256'})


def sha256_file(path: Path) -> str:
    return closure_sha256_file(path)


def _safe_relative(root: Path, value: Any, label: str,
                   *, allow_directory: bool = False) -> Path:
    try:
        root_info = root.lstat()
    except OSError as exc:
        raise CandidateError('repository root is unavailable') from exc
    if not stat.S_ISDIR(root_info.st_mode) or stat.S_ISLNK(root_info.st_mode):
        raise CandidateError('repository root must be a real directory')
    if not isinstance(value, str) or not RELATIVE_RE.fullmatch(value):
        raise CandidateError(f'{label} must be a safe repository-relative path')
    relative = Path(value)
    if relative.is_absolute() or '..' in relative.parts:
        raise CandidateError(f'{label} escapes the repository')
    current = root
    for component in relative.parts:
        current /= component
        if current.is_symlink():
            raise CandidateError(f'{label} contains a symlink')
    try:
        metadata = current.lstat()
    except OSError as exc:
        raise CandidateError(f'{label} is unavailable: {value}') from exc
    if stat.S_ISDIR(metadata.st_mode) and allow_directory:
        return current
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
        raise CandidateError(f'{label} must be a regular single-link file')
    return current


def _load_yaml(root: Path, relative: str, label: str) -> Mapping[str, Any]:
    path = _safe_relative(root, relative, label)
    try:
        value = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise CandidateError(f'{label} is unreadable') from exc
    if not isinstance(value, Mapping):
        raise CandidateError(f'{label} must be a mapping')
    return value


def _profile_closure(profile: Mapping[str, Any]) -> Mapping[str, Any]:
    contract = profile.get('competitive_slam_profile', profile)
    if not isinstance(contract, Mapping):
        raise CandidateError('competitive profile is not a mapping')
    evidence = contract.get('evidence_gate_v2')
    if not isinstance(evidence, Mapping) or not isinstance(
            evidence.get('rival_source_closure'), Mapping):
        raise CandidateError('rival source closure is missing from profile')
    return evidence['rival_source_closure']


def _assert_legal_blockers_remain(closure: Mapping[str, Any]) -> None:
    if closure.get('status') != 'NOT_READY':
        raise CandidateError('r2 legal closure cannot be promoted by r3 candidate')
    rivals = closure.get('rivals')
    if not isinstance(rivals, Mapping):
        raise CandidateError('r2 legal rivals are missing')
    for system in SYSTEMS:
        rival = rivals.get(system)
        if not isinstance(rival, Mapping) or rival.get('status') != 'NOT_READY':
            raise CandidateError(f'r2 {system} legal status is not NOT_READY')
    names = {
        'glim_ros2': 'glim_ros2_license_text_missing',
        'fast_livo2': 'fast_livo2_license_declaration_conflict',
        'rpg_vikit': 'rpg_vikit_license_artifacts_incomplete',
        'sophus': 'sophus_license_artifact_missing',
    }
    observed: dict[str, Mapping[str, Any]] = {}
    for rival in rivals.values():
        if isinstance(rival, Mapping) and isinstance(rival.get('sources'), list):
            for source in rival['sources']:
                if isinstance(source, Mapping) and isinstance(source.get('name'), str):
                    observed[source['name']] = source
    for name, blocker in names.items():
        source = observed.get(name)
        license_data = source.get('license') if isinstance(source, Mapping) else None
        if not isinstance(source, Mapping) or source.get('status') != 'NOT_READY' or \
                not isinstance(license_data, Mapping) or license_data.get(
                    'status') != 'NOT_READY_LEGAL_PROVENANCE':
            raise CandidateError(f'legal blocker {blocker} was removed')


def _artifact_hash(root: Path, descriptor: Mapping[str, Any], label: str) -> str:
    path_kind = descriptor.get('path_kind', 'local_checkout_relative_path')
    if path_kind == 'pinned_upstream_archive_relative_path':
        raise CandidateError(f'{label} is an unobserved upstream archive artifact')
    kind = descriptor.get('hash_kind', 'file_sha256')
    if kind == 'file_sha256':
        path = _safe_relative(root, descriptor.get('path'),
                              f'{label}.path')
        return sha256_file(path)
    if kind == 'canonical_tree_sha256':
        path = _safe_relative(root, descriptor.get('path'),
                              f'{label}.path', allow_directory=True)
        return sha256_tree(path)
    raise CandidateError(f'{label}.hash_kind is unsupported')


def _recipe_artifact_bindings(root: Path, system: str,
                              recipe: Mapping[str, Any],
                              local_patches: Any) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for role in ('dockerfile', 'build_script', 'runner', 'wrapper'):
        descriptor = recipe.get(role)
        if not isinstance(descriptor, Mapping):
            raise CandidateError(f'{system}.recipe.{role} is missing')
        current = _artifact_hash(root, descriptor, f'{system}.recipe.{role}')
        entries.append({
            'role': role,
            'path': descriptor['path'],
            'hash_kind': descriptor.get('hash_kind'),
            'r2_sha256': descriptor.get('sha256'),
            'current_sha256': current,
            'status': 'CURRENT_BYTES_REBOUND' if current != descriptor.get(
                'sha256') else 'CURRENT_BYTES_MATCH',
        })
    configs = recipe.get('configs')
    if not isinstance(configs, list):
        raise CandidateError(f'{system}.recipe.configs is missing')
    for index, descriptor in enumerate(configs):
        if not isinstance(descriptor, Mapping):
            raise CandidateError(f'{system}.recipe.configs[{index}] is malformed')
        path_kind = descriptor.get('path_kind', 'local_checkout_relative_path')
        if path_kind == 'pinned_upstream_archive_relative_path':
            # This is deliberately a binding-only record.  It does not claim
            # that the archive is present in this checkout.
            entries.append({
                'role': f'config[{index}]',
                'path': descriptor.get('path'),
                'path_kind': path_kind,
                'hash_kind': descriptor.get('hash_kind'),
                'r2_sha256': descriptor.get('sha256'),
                'current_sha256': None,
                'source': descriptor.get('source'),
                'source_role': descriptor.get('role'),
                'status': 'PINNED_UPSTREAM_ARCHIVE_UNOBSERVED',
            })
        else:
            current = _artifact_hash(root, descriptor,
                                     f'{system}.recipe.configs[{index}]')
            entries.append({
                'role': f'config[{index}]',
                'path': descriptor.get('path'),
                'hash_kind': descriptor.get('hash_kind'),
                'r2_sha256': descriptor.get('sha256'),
                'current_sha256': current,
                'status': 'CURRENT_BYTES_REBOUND' if current != descriptor.get(
                    'sha256') else 'CURRENT_BYTES_MATCH',
            })
    patches = local_patches
    if not isinstance(patches, list):
        raise CandidateError(f'{system}.local_patches is missing')
    for index, descriptor in enumerate(patches):
        if not isinstance(descriptor, Mapping):
            raise CandidateError(f'{system}.recipe.patches[{index}] is malformed')
        current = _artifact_hash(root, descriptor, f'{system}.recipe.patches[{index}]')
        entries.append({
            'role': f'patch[{index}]',
            'path': descriptor.get('path'),
            'hash_kind': descriptor.get('hash_kind', 'file_sha256'),
            'r2_sha256': descriptor.get('sha256'),
            'current_sha256': current,
            'status': 'CURRENT_BYTES_REBOUND' if current != descriptor.get(
                'sha256') else 'CURRENT_BYTES_MATCH',
        })
    return entries


def _r2_selection_binding(root: Path, selection: Mapping[str, Any],
                          profile: Mapping[str, Any],
                          closure: Mapping[str, Any]) -> dict[str, Any]:
    path = _safe_relative(root, R2_SELECTION_REL, 'r2 selection')
    identity = current_rival_source_closure_identity(profile, root=root)
    expected_id = selection.get('selection_id')
    if expected_id != identity.get('selection_id'):
        raise CandidateError('r2 selection ID is not current')
    return {
        'path': R2_SELECTION_REL,
        'file_sha256': sha256_file(path),
        'canonical_sha256': canonical_sha256(selection),
        'selection_id': expected_id,
        'status': selection.get('status'),
        'closure_id': closure.get('closure_id'),
        'closure_revision': closure.get('closure_revision'),
        'closure_identity_hash_kind': closure.get('closure_identity_hash_kind'),
        'closure_identity_sha256': identity['closure_identity_sha256'],
    }


def build_candidate_document(root: Path = ROOT) -> dict[str, Any]:
    root = Path(root)
    try:
        root_info = root.lstat()
    except OSError as exc:
        raise CandidateError('repository root is unavailable') from exc
    if not stat.S_ISDIR(root_info.st_mode) or stat.S_ISLNK(root_info.st_mode):
        raise CandidateError('repository root must be a real directory')
    profile = _load_yaml(root, PROFILE_REL, 'active profile')
    closure = _profile_closure(profile)
    _assert_legal_blockers_remain(closure)
    selection = _load_yaml(root, R2_SELECTION_REL, 'r2 selection')
    historical = _safe_relative(root, HISTORICAL_SELECTION_REL, 'historical selection')
    closure_identity = current_rival_source_closure_identity(profile, root=root)
    inherited = copy.deepcopy(dict(closure))
    if canonical_rival_source_closure_identity(inherited) != closure_identity[
            'closure_identity_sha256']:
        raise CandidateError('r2 closure identity is inconsistent')
    profile_path = _safe_relative(root, PROFILE_REL, 'active profile')
    profile_binding = {
        'path': PROFILE_REL,
        'file_sha256': sha256_file(profile_path),
        'canonical_sha256': canonical_profile_sha256(profile),
        'canonical_hash_kind': PROFILE_CANONICAL_HASH_KIND,
    }
    r2_binding = _r2_selection_binding(root, selection, profile, closure)
    recipe_bindings: dict[str, Any] = {}
    runner_bindings: dict[str, Any] = {}
    rivals = closure.get('rivals')
    if not isinstance(rivals, Mapping):
        raise CandidateError('r2 rivals are missing')
    for system in SYSTEMS:
        rival = rivals.get(system)
        recipe = rival.get('recipe') if isinstance(rival, Mapping) else None
        if not isinstance(recipe, Mapping):
            raise CandidateError(f'r2 {system} recipe is missing')
        local_patches = (rival.get('local_patches')
                         if isinstance(rival, Mapping) else None)
        recipe_bindings[system] = {
            'r2_recipe_sha256': canonical_sha256(recipe),
            'artifacts': _recipe_artifact_bindings(root, system, recipe,
                                                   local_patches),
        }
        runner = recipe.get('runner')
        if not isinstance(runner, Mapping):
            raise CandidateError(f'r2 {system} runner is missing')
        current_runner = _artifact_hash(root, runner, f'{system}.runner')
        runner_bindings[system] = {
            'path': runner.get('path'),
            'hash_kind': runner.get('hash_kind'),
            'r2_sha256': runner.get('sha256'),
            'current_sha256': current_runner,
            'status': 'CURRENT_BYTES_REBOUND',
        }
    capture_schema = _safe_relative(root, LEGAL_CAPTURE_SCHEMA_REL,
                                    'legal capture schema')
    capture_producer = _safe_relative(root, LEGAL_CAPTURE_PRODUCER_REL,
                                      'legal capture producer')
    document: dict[str, Any] = {
        'schema_version': 1,
        'candidate_kind': CANDIDATE_KIND,
        'candidate_id': 'competitive-rival-source-closure-2026-08-r3-candidate',
        'status': 'NOT_READY',
        'benchmark_eligible': False,
        'claim_eligible': False,
        'candidate_hash_kind': CANDIDATE_HASH_KIND,
        'candidate_identity_sha256': '',
        'r2_lineage': {
            'closure_id': closure_identity['closure_id'],
            'closure_revision': closure_identity['closure_revision'],
            'closure_identity_hash_kind': CLOSURE_IDENTITY_HASH_KIND,
            'closure_identity_sha256': closure_identity['closure_identity_sha256'],
            'r2_selection_path': R2_SELECTION_REL,
            'r2_selection_id': closure_identity['selection_id'],
            'r2_selection_sha256': closure_identity['selection_sha256'],
            'historical_selection_path': HISTORICAL_SELECTION_REL,
            'historical_selection_sha256': sha256_file(historical),
            'supersedes_status': 'R2_IMMUTABLE_NOT_REWRITTEN',
        },
        'profile_binding': profile_binding,
        'r2_selection_binding': r2_binding,
        'r2_inherited_closure': inherited,
        'current_runner_bindings': runner_bindings,
        'current_recipe_bindings': recipe_bindings,
        'legal_capture_contract': {
            'schema': {
                'path': LEGAL_CAPTURE_SCHEMA_REL,
                'file_sha256': sha256_file(capture_schema),
                'status': 'CONTRACT_BOUND_NOT_REVIEWED',
            },
            'producer': {
                'path': LEGAL_CAPTURE_PRODUCER_REL,
                'file_sha256': sha256_file(capture_producer),
                'status': 'CONTRACT_BOUND_NOT_REVIEWED',
            },
            'status': 'NOT_READY_EXTERNAL_REVIEW_REQUIRED',
            'auto_review': False,
        },
        'legal_status': {
            'status': 'NOT_READY',
            'blockers': list(LEGAL_BLOCKERS),
            'external_review_required': True,
            'package_metadata_is_not_license_text': True,
        },
        'source_provenance': {
            'status': 'CURRENT_BYTES_REBOUND_NOT_LEGAL_REVIEWED',
            'remote_observation': 'NOT_RUN',
            'execution': 'NOT_RUN',
        },
        'promotion_policy': {
            'active_profile_switch': False,
            'r2_immutable': True,
            'requires_external_reviewed_packet': True,
            'requires_profile_canonical_reseal': True,
            'handoff_status': 'UNSIGNED_REVIEW_REQUIRED',
            'benchmark_eligible_until_review': False,
        },
    }
    document['candidate_identity_sha256'] = candidate_identity_sha256(document)
    return document


def build_selection_document(root: Path = ROOT, *, candidate: Mapping[str, Any] | None = None,
                             candidate_file_sha256: str | None = None) -> dict[str, Any]:
    if candidate is None:
        candidate = build_candidate_document(root)
    if candidate_file_sha256 is None:
        candidate_file_sha256 = sha256_file(
            _safe_relative(root, CANDIDATE_REL, 'r3 candidate'))
    lineage = candidate.get('r2_lineage')
    if not isinstance(lineage, Mapping):
        raise CandidateError('candidate lineage is missing')
    document: dict[str, Any] = {
        'schema_version': 1,
        'selection_kind': SELECTION_KIND,
        'selection_id': 'competitive-rival-source-closure-selection-2026-08-r3-candidate',
        'status': 'NOT_READY',
        'benchmark_eligible': False,
        'claim_eligible': False,
        'selection_hash_kind': SELECTION_HASH_KIND,
        'selection_identity_sha256': '',
        'candidate_binding': {
            'path': CANDIDATE_REL,
            'file_sha256': candidate_file_sha256,
            'candidate_identity_sha256': candidate.get('candidate_identity_sha256'),
            'candidate_id': candidate.get('candidate_id'),
        },
        'r2_lineage': copy.deepcopy(dict(lineage)),
        'current_runner_bindings': copy.deepcopy(candidate.get(
            'current_runner_bindings')),
        'legal_status': {
            'status': 'NOT_READY',
            'external_review_required': True,
            'auto_review': False,
        },
        'promotion_policy': {
            'active_profile_switch': False,
            'requires_external_reviewed_packet': True,
            'requires_candidate_and_profile_reseal': True,
            'historical_r2_immutable': True,
            'handoff_status': 'UNSIGNED_REVIEW_REQUIRED',
        },
    }
    document['selection_identity_sha256'] = selection_identity_sha256(document)
    return document


def _read_json(path: Path, label: str) -> Mapping[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CandidateError(f'{label} is unreadable') from exc
    if not isinstance(value, Mapping):
        raise CandidateError(f'{label} must be an object')
    return value


def _check_sidecar(root: Path, relative: str, payload: bytes,
                   label: str) -> None:
    sidecar = _safe_relative(root, relative, f'{label} sidecar')
    text = sidecar.read_text(encoding='utf-8')
    target_name = (Path(relative).name[:-len('.sha256')]
                   if relative.endswith('.sha256') else Path(relative).name)
    expected = f'{hashlib.sha256(payload).hexdigest()}  {target_name}\n'
    if text != expected:
        raise CandidateError(f'{label} sidecar does not bind exact bytes')


def validate_candidate_document(candidate: Mapping[str, Any], root: Path = ROOT) -> dict[str, Any]:
    expected = build_candidate_document(root)
    if dict(candidate) != expected:
        raise CandidateError('candidate differs from the current canonical projection')
    if candidate_identity_sha256(candidate) != candidate.get('candidate_identity_sha256'):
        raise CandidateError('candidate identity is stale')
    if candidate.get('status') != 'NOT_READY' or candidate.get(
            'benchmark_eligible') is not False or candidate.get('claim_eligible') is not False:
        raise CandidateError('candidate cannot be promoted')
    return {
        'status': 'NOT_READY',
        'structural_valid': True,
        'benchmark_eligible': False,
        'claim_eligible': False,
        'candidate_id': candidate['candidate_id'],
        'candidate_identity_sha256': candidate['candidate_identity_sha256'],
        'blockers': list(candidate['legal_status']['blockers']),
    }


def validate_selection_document(selection: Mapping[str, Any], root: Path = ROOT,
                                *, candidate: Mapping[str, Any] | None = None,
                                candidate_file_sha256: str | None = None) -> dict[str, Any]:
    candidate = candidate or _read_json(_safe_relative(root, CANDIDATE_REL, 'r3 candidate'),
                                        'r3 candidate')
    validate_candidate_document(candidate, root)
    expected = build_selection_document(root, candidate=candidate,
                                        candidate_file_sha256=candidate_file_sha256)
    if dict(selection) != expected:
        raise CandidateError('selection differs from the current canonical projection')
    if selection_identity_sha256(selection) != selection.get('selection_identity_sha256'):
        raise CandidateError('selection identity is stale')
    return {
        'status': 'NOT_READY',
        'structural_valid': True,
        'benchmark_eligible': False,
        'claim_eligible': False,
        'selection_id': selection['selection_id'],
        'selection_identity_sha256': selection['selection_identity_sha256'],
    }


def validate_checked_in(root: Path = ROOT) -> dict[str, Any]:
    candidate_path = _safe_relative(root, CANDIDATE_REL, 'r3 candidate')
    selection_path = _safe_relative(root, SELECTION_REL, 'r3 selection')
    candidate_payload = candidate_path.read_bytes()
    selection_payload = selection_path.read_bytes()
    _check_sidecar(root, CANDIDATE_SIDECAR_REL, candidate_payload, 'r3 candidate')
    _check_sidecar(root, SELECTION_SIDECAR_REL, selection_payload, 'r3 selection')
    candidate = _read_json(candidate_path, 'r3 candidate')
    selection = _read_json(selection_path, 'r3 selection')
    candidate_report = validate_candidate_document(candidate, root)
    selection_report = validate_selection_document(
        selection, root, candidate=candidate,
        candidate_file_sha256=sha256_file(candidate_path))
    return {'candidate': candidate_report, 'selection': selection_report,
            'status': 'NOT_READY', 'benchmark_eligible': False,
            'claim_eligible': False}


def _write_exclusive(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() or path.is_symlink():
        raise CandidateError(f'refusing to overwrite existing candidate: {path}')
    descriptor = None
    try:
        descriptor = path.open('xb')
        descriptor.write(payload)
        descriptor.flush()
        import os
        os.fsync(descriptor.fileno())
        descriptor.close()
        descriptor = None
        path.chmod(0o444)
    except Exception:
        if descriptor is not None:
            descriptor.close()
        try:
            path.unlink()
        except FileNotFoundError:
            pass
        raise


def write_candidates(root: Path = ROOT) -> dict[str, Any]:
    candidate = build_candidate_document(root)
    candidate_path = root / CANDIDATE_REL
    candidate_payload = (
        json.dumps(candidate, sort_keys=True, indent=2, ensure_ascii=True) +
        '\n').encode()
    _write_exclusive(candidate_path, candidate_payload)
    _write_exclusive(root / CANDIDATE_SIDECAR_REL,
                     (f'{hashlib.sha256(candidate_payload).hexdigest()}  '
                      f'{candidate_path.name}\n').encode())
    selection = build_selection_document(
        root, candidate=candidate,
        candidate_file_sha256=hashlib.sha256(candidate_payload).hexdigest())
    selection_path = root / SELECTION_REL
    selection_payload = (
        json.dumps(selection, sort_keys=True, indent=2, ensure_ascii=True) +
        '\n').encode()
    _write_exclusive(selection_path, selection_payload)
    _write_exclusive(root / SELECTION_SIDECAR_REL,
                     (f'{hashlib.sha256(selection_payload).hexdigest()}  '
                      f'{selection_path.name}\n').encode())
    return validate_checked_in(root)


def main() -> int:
    parser = __import__('argparse').ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--write-candidates', action='store_true')
    args = parser.parse_args()
    try:
        report = (write_candidates(args.root) if args.write_candidates
                  else validate_checked_in(args.root))
    except (CandidateError, OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        print(json.dumps({'status': 'INVALID', 'error': str(exc)}, indent=2))
        return 1
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
