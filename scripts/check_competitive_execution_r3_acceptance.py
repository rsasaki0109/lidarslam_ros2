#!/usr/bin/env python3
"""Check the checked-in, non-promoting r3 acceptance preflight manifest.

This is deliberately only a preflight checker.  It verifies the current
candidate/profile bindings and the exact set of required closure slots, then
returns ``NOT_READY`` (exit 3) while every slot is pending.  It never accepts
an in-place READY edit: a changed status, path, hash, candidate, profile, or
closure set is an invalid manifest (exit 2).  No data, GT, subprocess,
network, Docker, or external artifact is opened by this checker.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from scripts.validate_competitive_execution_selection_r3 import (  # noqa: E402
    CANDIDATE_REL,
    CandidateError,
    ROOT,
    validate_candidate,
)


MANIFEST_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_r3_acceptance_manifest.json')
SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_r3_acceptance_manifest_v1.schema.json')
SCHEMA = 'competitive_execution_selection_r3_acceptance_manifest_v1'
CAMPAIGN_ID = 'competitive-execution-selection-2026-08-r3-candidate'
REQUIRED_CLOSURES = (
    'selection_handoff', 'rival_source_closure', 'dataset_source_closure',
    'fresh_holdout_authorization', 'machine_identity',
    'image_toolchain_review', 'source_worktree_review')
PENDING = 'PENDING'
NOT_READY = 'NOT_READY'
MAX_BYTES = 2 * 1024 * 1024
SHA256_LENGTH = 64


class AcceptanceManifestError(ValueError):
    """The manifest is malformed, stale, promoted, or otherwise unsafe."""


def _canonical(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def _sha(value: Any) -> str:
    payload = value if isinstance(value, bytes) else _canonical(value)
    return hashlib.sha256(payload).hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    if not path.is_absolute() or '..' in path.parts or path.is_symlink():
        raise AcceptanceManifestError('manifest path is not normalized')
    try:
        info = path.stat()
        if not path.is_file() or info.st_nlink != 1 or info.st_size <= 0 or \
                info.st_size > MAX_BYTES:
            raise AcceptanceManifestError('manifest is not a bounded regular file')
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise AcceptanceManifestError('manifest is unreadable') from error
    if not isinstance(value, dict):
        raise AcceptanceManifestError('manifest must be an object')
    return value


def _current_binding() -> tuple[dict[str, Any], dict[str, Any]]:
    report = validate_candidate(ROOT / CANDIDATE_REL)
    if not report.get('structural_valid') or report.get('status') != NOT_READY:
        raise AcceptanceManifestError('current r3 candidate is not valid NOT_READY')
    candidate_path = ROOT / CANDIDATE_REL
    try:
        candidate = json.loads(candidate_path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise AcceptanceManifestError('current r3 candidate cannot be reopened') from error
    if not isinstance(candidate, dict):
        raise AcceptanceManifestError('current r3 candidate is not an object')
    binding = {
        'path': CANDIDATE_REL,
        'file_sha256': report['candidate_file_sha256'],
        'candidate_identity_sha256': report['candidate_identity_sha256'],
        'candidate_id': report['candidate_id'],
        'status': NOT_READY,
    }
    profile = candidate.get('profile_binding')
    if not isinstance(profile, dict) or set(profile) != {
            'path', 'file_sha256', 'canonical_sha256', 'canonical_hash_kind'}:
        raise AcceptanceManifestError('current profile binding is malformed')
    return binding, dict(profile)


def _expected_shape() -> set[str]:
    return {
        'schema', 'schema_version', 'status', 'benchmark_eligible',
        'claim_eligible', 'active_profile_switch', 'campaign_id',
        'candidate_binding', 'profile_binding', 'required_closures',
        'closures', 'manifest_identity_sha256'}


def _validate_closures(value: Any) -> None:
    if not isinstance(value, dict) or set(value) != set(REQUIRED_CLOSURES):
        raise AcceptanceManifestError('closure names are not exact')
    for name in REQUIRED_CLOSURES:
        entry = value[name]
        if not isinstance(entry, dict) or set(entry) != {
                'status', 'path', 'sha256'}:
            raise AcceptanceManifestError(f'{name} closure shape is not exact')
        if entry['status'] != PENDING or entry['path'] is not None or \
                entry['sha256'] is not None:
            raise AcceptanceManifestError(
                f'{name} closure is promoted or has an invented artifact')


def validate_manifest(path: Path = ROOT / MANIFEST_REL) -> dict[str, Any]:
    """Validate one manifest and return a non-promoting NOT_READY report."""
    path = Path(path)
    if not path.is_absolute():
        path = ROOT / path
    value = _read_json(path)
    if set(value) != _expected_shape():
        raise AcceptanceManifestError('manifest fields are not exact')
    if value['schema'] != SCHEMA or value['schema_version'] != 1:
        raise AcceptanceManifestError('manifest schema identity is invalid')
    if value['status'] != NOT_READY or value['benchmark_eligible'] is not False or \
            value['claim_eligible'] is not False or \
            value['active_profile_switch'] is not False:
        raise AcceptanceManifestError('in-place acceptance promotion is forbidden')
    if value['campaign_id'] != CAMPAIGN_ID:
        raise AcceptanceManifestError('campaign identity is stale')
    if value['required_closures'] != list(REQUIRED_CLOSURES):
        raise AcceptanceManifestError('required closure order is stale')
    candidate, profile = _current_binding()
    if value['candidate_binding'] != candidate:
        raise AcceptanceManifestError('candidate binding drift')
    if value['profile_binding'] != profile:
        raise AcceptanceManifestError('profile binding drift')
    _validate_closures(value['closures'])
    if value['manifest_identity_sha256'] != _sha({
            key: item for key, item in value.items()
            if key != 'manifest_identity_sha256'}):
        raise AcceptanceManifestError('manifest self-hash is stale')
    return {
        'status': NOT_READY,
        'benchmark_eligible': False,
        'claim_eligible': False,
        'active_profile_switch': False,
        'campaign_id': CAMPAIGN_ID,
        'candidate_identity_sha256': candidate['candidate_identity_sha256'],
        'profile_canonical_sha256': profile['canonical_sha256'],
        'required_closures': list(REQUIRED_CLOSURES),
        'missing_closures': list(REQUIRED_CLOSURES),
        'manifest_identity_sha256': value['manifest_identity_sha256'],
    }


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path,
                        default=ROOT / MANIFEST_REL)
    args = parser.parse_args()
    try:
        result = validate_manifest(args.manifest)
    except (AcceptanceManifestError, CandidateError, OSError, ValueError,
            TypeError) as error:
        print(json.dumps({'status': 'INVALID', 'benchmark_eligible': False,
                          'claim_eligible': False, 'error': str(error)},
                         sort_keys=True))
        return 2
    print(json.dumps(result, sort_keys=True))
    return 3


if __name__ == '__main__':
    raise SystemExit(_main())
