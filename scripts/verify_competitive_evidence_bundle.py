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

"""Verify the immutable filesystem boundary of a competitive result bundle.

The metric evaluator validates the *meaning* of a result.  This verifier
validates the bytes that are allowed to reach that evaluator.  A claim-worthy
bundle has one canonical root, one deterministic JSON manifest, one entry for
each required role, and a SHA-256 sidecar for every entry.  Every listed byte
and sidecar is reopened from the verified root and checked for regular-file,
single-link, size, and digest identity.

This module is deliberately GT-blind.  It never parses a result, trajectory,
map, or failure artifact and rejects ground-truth roles/path components before
opening any listed artifact.  Profile/scorer/revision metadata is checked only
as identity (hashes and opaque bytes); scoring remains a separate authorized
stage.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import stat
import sys
from typing import Any, Iterable, Mapping
import unicodedata

import yaml

# Direct source-checkout invocation places ``scripts/`` on sys.path.  Accept
# only the canonical checkout that owns this file; installed entry points use
# the package normally and never fall back to a repository sibling.
_SCRIPT_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if (
        (_SCRIPT_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file()
        and (_SCRIPT_SOURCE_ROOT / 'scripts' / 'benchmark_phase_contract.py').is_file()
        and str(_SCRIPT_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SCRIPT_SOURCE_ROOT))

try:
    from lidarslam_benchmark_tools.competitive_identity_hash import (
        PROFILE_CANONICAL_HASH_KIND,
        canonical_profile_sha256,
    )
    from lidarslam_benchmark_tools.competitive_holdout_authorization import (
        verify_fresh_holdout_authorization,
    )
    from lidarslam_benchmark_tools.competitive_execution_attempt_receipt import (
        ExecutionReceiptError, validate_receipt,
    )
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.competitive_identity_hash import (  # type: ignore[no-redef]
        PROFILE_CANONICAL_HASH_KIND,
        canonical_profile_sha256,
    )
    from lidarslam_benchmark_tools.competitive_holdout_authorization import (
        verify_fresh_holdout_authorization,
    )
    from lidarslam_benchmark_tools.competitive_execution_attempt_receipt import (
        ExecutionReceiptError, validate_receipt,
    )


try:
    from lidarslam_benchmark_tools import package_root
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    def package_root() -> Path:
        return Path(__file__).resolve().parents[1]


ROOT = package_root()
MANIFEST_KIND = 'competitive_slam_evidence_bundle'
SCHEMA_VERSION = 1
MANIFEST_FILENAME = 'bundle_manifest.json'
MANIFEST_SIDECAR_FILENAME = 'bundle_manifest.json.sha256'
MANIFEST_HASH_KIND = 'canonical_manifest_sha256_excluding_self_v1'
SIDECAR_FORMAT = 'sha256sum_relative_path_v1'
FILE_HASH_KIND = 'sha256_file_v1'

# These names are intentionally explicit.  A role is a contract identity,
# not a free-form label that a producer can invent to evade a required role.
REQUIRED_ROLES = (
    'input',
    'result',
    'config',
    'calibration',
    'revision',
    'scorer',
    'trajectory',
    'resource',
    'map_metric',
    'failure',
)
DEFAULT_OPTIONAL_ROLES: tuple[str, ...] = ()
# ``execution`` is the sealed per-attempt receipt.  Metric/result bytes may be
# identical for deterministic runs, but an execution receipt must be unique
# for every scored run so repetition count cannot be manufactured by cloning a
# result/resource projection.
RUN_ARTIFACT_ROLES = ('trajectory', 'map', 'resource', 'score', 'execution')
FORBIDDEN_ROLE_NAMES = {
    'gt',
    'ground_truth',
    'ground-truth',
    'groundtruth',
    'scorer_input_gt',
}
_SHA256_RE = re.compile(r'^[0-9a-f]{64}$')
_REVISION_RE = re.compile(r'^[0-9a-f]{40}$')
_DRIVE_RE = re.compile(r'^[A-Za-z]:')
_MANIFEST_HASH_FIELDS = {'manifest_sha256', 'manifest_sidecar_sha256'}


class BundleVerificationError(ValueError):
    """Raised only by low-level helpers for malformed caller input."""


def _canonical_json(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(',', ':'), ensure_ascii=True
    ).encode('utf-8')


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise BundleVerificationError(
                f'duplicate JSON key in execution receipt: {key}')
        result[key] = value
    return result


def canonical_manifest_payload(manifest: Mapping[str, Any]) -> dict[str, Any]:
    """Return the payload covered by the manifest's self hash.

    Both the manifest hash and its sidecar hash are excluded.  Excluding the
    sidecar hash avoids a recursion while retaining it in the deterministic
    bytes written to disk.
    """
    if not isinstance(manifest, Mapping):
        raise BundleVerificationError('manifest must be a mapping')
    return {
        str(key): value for key, value in manifest.items()
        if key not in _MANIFEST_HASH_FIELDS
    }


def canonical_manifest_sha256(manifest: Mapping[str, Any]) -> str:
    """Hash the canonical manifest payload without a trailing newline."""
    return hashlib.sha256(_canonical_json(canonical_manifest_payload(manifest))).hexdigest()


def canonical_manifest_bytes(manifest: Mapping[str, Any]) -> bytes:
    """Return the only accepted on-disk manifest encoding."""
    return _canonical_json(dict(manifest)) + b'\n'


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    """Hash a known regular file without following a final symlink."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _jsonable(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def _hash_field(value: Any, label: str) -> str:
    if not isinstance(value, str) or _SHA256_RE.fullmatch(value) is None:
        raise BundleVerificationError(f'{label} must be lowercase 64-hex')
    return value


def _size_field(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise BundleVerificationError(f'{label} must be a non-negative integer')
    return value


def _normalise_relative(value: Any, label: str) -> str:
    """Validate a canonical POSIX relative path without resolving it."""
    if not isinstance(value, str) or not value:
        raise BundleVerificationError(f'{label} must be a non-empty string')
    if '\x00' in value:
        raise BundleVerificationError(f'{label} contains NUL')
    # Backslashes and non-NFC spellings make cross-platform/canonical aliases.
    if '\\' in value:
        raise BundleVerificationError(f'{label} must use POSIX separators')
    if unicodedata.normalize('NFC', value) != value:
        raise BundleVerificationError(f'{label} is not NFC-normalized')
    if (PurePosixPath(value).is_absolute() or PureWindowsPath(value).is_absolute()
            or _DRIVE_RE.match(value)):
        raise BundleVerificationError(f'{label} must be relative')
    parts = value.split('/')
    if any(part in {'', '.', '..'} for part in parts):
        raise BundleVerificationError(f'{label} is not normalized')
    normalized = PurePosixPath(value).as_posix()
    if normalized != value:
        raise BundleVerificationError(f'{label} is not normalized')
    return value


def _path_key(value: str) -> str:
    # Case folding catches aliases on case-insensitive filesystems even when
    # the verifier itself is running on a case-sensitive filesystem.
    return unicodedata.normalize('NFC', value).casefold()


def _forbidden_gt_path(value: str) -> bool:
    components = set(value.lower().split('/'))
    lowered = value.lower()
    return bool(components & {'gt', 'ground_truth', 'ground-truth', 'groundtruth'}
                or 'ground_truth' in lowered or 'ground-truth' in lowered)


def _root_path(root: Path) -> Path:
    if not isinstance(root, Path):
        root = Path(root)
    if root.is_symlink() or not root.is_dir():
        raise BundleVerificationError('bundle root is missing, symlinked, or not a directory')
    return root.resolve(strict=True)


def _relative_to_root(root: Path, path: Path, label: str) -> str:
    try:
        relative = path.resolve(strict=False).relative_to(root)
    except ValueError as exc:
        raise BundleVerificationError(f'{label} escapes canonical bundle root') from exc
    return _normalise_relative(relative.as_posix(), label)


def _check_components(root: Path, relative: str, label: str) -> Path:
    """Reject symlinked ancestors before a file is opened."""
    current = root
    components = relative.split('/')
    for index, component in enumerate(components):
        current = current / component
        try:
            info = current.lstat()
        except FileNotFoundError as exc:
            raise BundleVerificationError(f'{label} is missing') from exc
        if stat.S_ISLNK(info.st_mode):
            raise BundleVerificationError(f'{label} contains a symlink')
        if index < len(components) - 1 and not stat.S_ISDIR(info.st_mode):
            raise BundleVerificationError(f'{label} has a non-directory ancestor')
    return current


def _read_regular(root: Path, relative: str, label: str,
                  *, include_bytes: bool = False) -> tuple[int, str, bytes]:
    """Read a regular, single-link file with a no-follow final open."""
    path = _check_components(root, relative, label)
    try:
        before = path.lstat()
    except FileNotFoundError as exc:
        raise BundleVerificationError(f'{label} is missing') from exc
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode):
        raise BundleVerificationError(f'{label} is not a regular file')
    if before.st_nlink != 1:
        raise BundleVerificationError(f'{label} is hard-linked (nlink={before.st_nlink})')
    flags = os.O_RDONLY
    nofollow = getattr(os, 'O_NOFOLLOW', 0)
    try:
        fd = os.open(path, flags | nofollow)
    except OSError as exc:
        raise BundleVerificationError(f'{label} could not be opened safely') from exc
    try:
        with os.fdopen(fd, 'rb') as stream:
            after = os.fstat(stream.fileno())
            if (not stat.S_ISREG(after.st_mode) or after.st_nlink != 1 or
                    after.st_dev != before.st_dev or after.st_ino != before.st_ino):
                raise BundleVerificationError(f'{label} changed during verification')
            digest = hashlib.sha256()
            chunks: list[bytes] = []
            size = 0
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                if include_bytes:
                    chunks.append(block)
                size += len(block)
                digest.update(block)
    except BundleVerificationError:
        raise
    except OSError as exc:
        raise BundleVerificationError(f'{label} could not be read safely') from exc
    return size, digest.hexdigest(), b''.join(chunks) if include_bytes else b''


def _entry_sidecar(entry: Mapping[str, Any], index: int) -> tuple[str, str]:
    nested = entry.get('sidecar')
    if isinstance(nested, Mapping):
        path = nested.get('path')
        digest = nested.get('sha256')
    else:
        path = entry.get('sidecar_path')
        digest = entry.get('sidecar_sha256')
    return (
        _normalise_relative(path, f'artifacts[{index}].sidecar.path'),
        _hash_field(digest, f'artifacts[{index}].sidecar.sha256'),
    )


def _run_artifact_ref(value: Any, label: str) -> dict[str, Any]:
    """Validate one opaque per-run artifact reference.

    The verifier deliberately does not parse the referenced trajectory, map,
    score, or resource bytes.  It only reopens the exact regular file and its
    sidecar later.  Keeping the three references separate from the historical
    one-per-role entries prevents a manifest with one trajectory from being
    mistaken for coverage of every scored run.
    """
    if not isinstance(value, Mapping):
        raise BundleVerificationError(f'{label} must be a mapping')
    path = _normalise_relative(value.get('path'), f'{label}.path')
    if _forbidden_gt_path(path):
        raise BundleVerificationError(
            'ground-truth run artifact/path is forbidden before scoring')
    size_bytes = _size_field(value.get('size_bytes'), f'{label}.size_bytes')
    digest = _hash_field(value.get('sha256'), f'{label}.sha256')
    nested = value.get('sidecar')
    if isinstance(nested, Mapping):
        sidecar_path_value = nested.get('path')
        sidecar_digest_value = nested.get('sha256')
    else:
        sidecar_path_value = value.get('sidecar_path')
        sidecar_digest_value = value.get('sidecar_sha256')
    sidecar_path = _normalise_relative(
        sidecar_path_value, f'{label}.sidecar.path')
    sidecar_digest = _hash_field(
        sidecar_digest_value, f'{label}.sidecar.sha256')
    if _forbidden_gt_path(sidecar_path):
        raise BundleVerificationError(
            'ground-truth run sidecar path is forbidden before scoring')
    receipt_sha = value.get('receipt_sha256')
    if receipt_sha is not None:
        receipt_sha = _hash_field(receipt_sha, f'{label}.receipt_sha256')
    return {
        'path': path,
        'size_bytes': size_bytes,
        'sha256': digest,
        'receipt_sha256': receipt_sha,
        'sidecar_path': sidecar_path,
        'sidecar_sha256': sidecar_digest,
    }


def _run_identity(value: Any, index: int) -> tuple[str, str, int]:
    """Return a canonical system/dataset/run identity without opening bytes."""
    if not isinstance(value, Mapping):
        raise BundleVerificationError(
            f'run_artifact_bindings[{index}] must be a mapping')
    system = value.get('system')
    dataset = value.get('dataset')
    run_index = value.get('run_index')
    for name, item in (('system', system), ('dataset', dataset)):
        if (not isinstance(item, str) or not item or '/' in item or '\\' in item
                or item in {'.', '..'}):
            raise BundleVerificationError(
                f'run_artifact_bindings[{index}].{name} is invalid')
        if _forbidden_gt_path(item):
            raise BundleVerificationError(
                'ground-truth run identity is forbidden before scoring')
    if (isinstance(run_index, bool) or not isinstance(run_index, int)
            or run_index < 1):
        raise BundleVerificationError(
            f'run_artifact_bindings[{index}].run_index is invalid')
    return system, dataset, run_index


def _expected_run_artifact_map(
        expected: Iterable[Mapping[str, Any]] | None,
        errors: list[str]) -> dict[tuple[str, str, int], dict[str, Any]] | None:
    """Normalize evaluator-side opaque hashes used for exact cross-binding."""
    if expected is None:
        return None
    normalized: dict[tuple[str, str, int], dict[str, str]] = {}
    try:
        values = list(expected)
    except TypeError:
        errors.append('expected_run_artifacts must be an iterable of mappings')
        return {}
    for index, item in enumerate(values):
        try:
            key = _run_identity(item, index)
            fields = {
                'trajectory': item.get('trajectory_sha256'),
                'map': item.get('map_sha256'),
                'resource': item.get('resource_sha256'),
                'score': item.get('score_sha256'),
                # The file hash authenticates the copied pretty JSON artifact;
                # the canonical receipt hash authenticates its parsed content.
                'execution': item.get('execution_receipt_file_sha256'),
                'execution_receipt': item.get('execution_receipt_sha256'),
                'campaign': item.get('campaign_id'),
            }
            normalized_fields = {
                role: _hash_field(
                    value,
                    f'expected_run_artifacts[{index}].{role}_sha256')
                for role, value in fields.items()
            }
            if key in normalized:
                raise BundleVerificationError(
                    f'expected_run_artifacts duplicate run identity: '
                    f'{key[0]}:{key[1]}:{key[2]}')
            normalized[key] = normalized_fields
        except (BundleVerificationError, TypeError, ValueError) as exc:
            errors.append(str(exc))
    return normalized


def _policy(contract: Mapping[str, Any] | None) -> dict[str, Any]:
    if not isinstance(contract, Mapping):
        return {}
    direct = contract.get('evidence_bundle_integrity')
    if isinstance(direct, Mapping):
        return dict(direct)
    gate = contract.get('evidence_gate_v2')
    if isinstance(gate, Mapping) and isinstance(gate.get('evidence_bundle_integrity'), Mapping):
        return dict(gate['evidence_bundle_integrity'])
    return {}


def _authorization_policy(contract: Mapping[str, Any] | None) -> dict[str, Any]:
    """Return the preregistered fresh-holdout authorization policy."""
    if not isinstance(contract, Mapping):
        return {}
    direct = contract.get('fresh_holdout_authorization')
    if isinstance(direct, Mapping):
        return dict(direct)
    gate = contract.get('evidence_gate_v2')
    if (isinstance(gate, Mapping) and
            isinstance(gate.get('fresh_holdout_authorization'), Mapping)):
        return dict(gate['fresh_holdout_authorization'])
    return {}


def _expected_profile(profile: Mapping[str, Any] | None) -> tuple[str | None, str | None]:
    if not isinstance(profile, Mapping):
        return None, None
    try:
        canonical = canonical_profile_sha256(profile)
    except (TypeError, ValueError):
        return None, None
    document = profile.get('competitive_slam_profile', profile)
    name = document.get('name') if isinstance(document, Mapping) else None
    return canonical, name if isinstance(name, str) else None


def _metadata_identity(
        root: Path, manifest: Mapping[str, Any], entries: list[dict[str, Any]],
        *, profile: Mapping[str, Any] | None,
        expected_profile_sha256: str | None,
        expected_profile_name: str | None,
        expected_scorer_sha256: str | None,
        expected_scorer_fingerprint: str | None,
        expected_revision_by_system: Mapping[str, str] | None,
        errors: list[str], checks: dict[str, dict[str, Any]]) -> None:
    by_role = {entry['role']: entry for entry in entries}
    declared_profile = manifest.get('profile')
    profile_ok = isinstance(declared_profile, Mapping)
    profile_entry = by_role.get('config')
    if not profile_ok:
        errors.append('manifest.profile is missing or malformed')
    else:
        profile_path = declared_profile.get('path')
        try:
            profile_path = _normalise_relative(profile_path, 'manifest.profile.path')
        except BundleVerificationError as exc:
            errors.append(str(exc))
            profile_path = None
        try:
            declared_file_sha = _hash_field(
                declared_profile.get('sha256'), 'manifest.profile.sha256')
            declared_canonical_sha = _hash_field(
                declared_profile.get('canonical_sha256'),
                'manifest.profile.canonical_sha256')
        except BundleVerificationError as exc:
            errors.append(str(exc))
            declared_file_sha = declared_canonical_sha = None
        if profile_entry is None or profile_path != (profile_entry or {}).get('path'):
            errors.append('manifest.profile.path does not match the config artifact')
            profile_ok = False
        if profile_entry is not None and declared_file_sha != profile_entry.get('sha256'):
            errors.append('manifest.profile.sha256 does not match the config artifact')
            profile_ok = False
        canonical_expected, profile_name = _expected_profile(profile)
        if expected_profile_sha256 is None:
            expected_profile_sha256 = canonical_expected
        if expected_profile_name is None:
            expected_profile_name = profile_name
        if expected_profile_sha256 is None:
            errors.append('expected canonical profile identity is missing')
            profile_ok = False
        elif declared_canonical_sha != expected_profile_sha256:
            errors.append('manifest profile canonical SHA-256 drift')
            profile_ok = False
        if (expected_profile_name is not None and
                declared_profile.get('name') != expected_profile_name):
            errors.append('manifest profile name drift')
            profile_ok = False
        if declared_profile.get('sha256_kind') != FILE_HASH_KIND:
            errors.append('manifest.profile.sha256_kind is invalid')
            profile_ok = False
        if declared_profile.get('canonical_sha256_kind') != PROFILE_CANONICAL_HASH_KIND:
            errors.append('manifest.profile.canonical_sha256_kind is invalid')
            profile_ok = False
        if profile_path is not None:
            try:
                _, _, profile_bytes = _read_regular(
                    root, profile_path, 'profile config artifact', include_bytes=True)
                parsed_profile = yaml.safe_load(profile_bytes.decode('utf-8'))
                actual_canonical_sha = canonical_profile_sha256(parsed_profile)
                if actual_canonical_sha != declared_canonical_sha:
                    errors.append('manifest profile canonical SHA-256 does not match config bytes')
                    profile_ok = False
            except (BundleVerificationError, UnicodeDecodeError,
                    yaml.YAMLError, ValueError) as exc:
                errors.append(f'profile config identity cannot be parsed: {exc}')
                profile_ok = False
    checks['profile_identity'] = {'pass': profile_ok}

    declared_scorer = manifest.get('scorer')
    scorer_ok = isinstance(declared_scorer, Mapping)
    scorer_entry = by_role.get('scorer')
    if not scorer_ok:
        errors.append('manifest.scorer is missing or malformed')
    else:
        try:
            scorer_path = _normalise_relative(
                declared_scorer.get('path'), 'manifest.scorer.path')
            scorer_sha = _hash_field(
                declared_scorer.get('sha256'), 'manifest.scorer.sha256')
            fingerprint = _hash_field(
                declared_scorer.get('fingerprint'), 'manifest.scorer.fingerprint')
        except BundleVerificationError as exc:
            errors.append(str(exc))
            scorer_path = scorer_sha = fingerprint = None
        if scorer_entry is None or scorer_path != (scorer_entry or {}).get('path'):
            errors.append('manifest.scorer.path does not match the scorer artifact')
            scorer_ok = False
        if scorer_entry is not None and scorer_sha != scorer_entry.get('sha256'):
            errors.append('manifest.scorer.sha256 does not match the scorer artifact')
            scorer_ok = False
        if expected_scorer_sha256 is not None and scorer_sha != expected_scorer_sha256:
            errors.append('manifest scorer SHA-256 drift')
            scorer_ok = False
        if expected_scorer_fingerprint is not None and fingerprint != expected_scorer_fingerprint:
            errors.append('manifest scorer fingerprint drift')
            scorer_ok = False
    checks['scorer_identity'] = {'pass': scorer_ok}

    declared_revision = manifest.get('revision')
    revision_ok = isinstance(declared_revision, Mapping)
    revision_entry = by_role.get('revision')
    if not revision_ok:
        errors.append('manifest.revision is missing or malformed')
    else:
        try:
            revision_path = _normalise_relative(
                declared_revision.get('path'), 'manifest.revision.path')
            revision_sha = _hash_field(
                declared_revision.get('sha256'), 'manifest.revision.sha256')
        except BundleVerificationError as exc:
            errors.append(str(exc))
            revision_path = revision_sha = None
        if revision_entry is None or revision_path != (revision_entry or {}).get('path'):
            errors.append('manifest.revision.path does not match the revision artifact')
            revision_ok = False
        if revision_entry is not None and revision_sha != revision_entry.get('sha256'):
            errors.append('manifest.revision.sha256 does not match the revision artifact')
            revision_ok = False
        observed = declared_revision.get('systems')
        if not isinstance(observed, Mapping) or not observed:
            errors.append('manifest.revision.systems is missing')
            revision_ok = False
        else:
            for system, value in observed.items():
                if (not isinstance(system, str) or not isinstance(value, str) or
                        _REVISION_RE.fullmatch(value) is None):
                    errors.append('manifest.revision.systems contains an invalid revision')
                    revision_ok = False
            if expected_revision_by_system is not None:
                expected = {
                    str(system): str(value).lower()
                    for system, value in expected_revision_by_system.items()
                }
                observed_normalized = {
                    str(system): str(value).lower()
                    for system, value in observed.items()
                }
                if observed_normalized != expected:
                    errors.append('manifest revision identity drift')
                    revision_ok = False
            if revision_path is not None:
                try:
                    _, _, revision_bytes = _read_regular(
                        root, revision_path, 'revision artifact', include_bytes=True)
                    revision_document = json.loads(revision_bytes.decode('utf-8'))
                    revision_bytes_identity = (
                        revision_document.get('systems')
                        if isinstance(revision_document, Mapping) and
                        isinstance(revision_document.get('systems'), Mapping)
                        else revision_document)
                    if not isinstance(revision_bytes_identity, Mapping):
                        raise BundleVerificationError(
                            'revision artifact must contain a systems mapping')
                    normalized_bytes_identity = {
                        str(system): str(value).lower()
                        for system, value in revision_bytes_identity.items()
                    }
                    normalized_declared_identity = {
                        str(system): str(value).lower()
                        for system, value in observed.items()
                    }
                    if normalized_bytes_identity != normalized_declared_identity:
                        errors.append('manifest revision metadata does not match revision bytes')
                        revision_ok = False
                except (BundleVerificationError, UnicodeDecodeError,
                        json.JSONDecodeError, ValueError) as exc:
                    errors.append(f'revision identity cannot be parsed: {exc}')
                    revision_ok = False
    checks['revision_identity'] = {'pass': revision_ok}

    score_handoff = manifest.get('score_handoff')
    score_handoff_ok = True
    if score_handoff is not None:
        if not isinstance(score_handoff, Mapping):
            errors.append('manifest.score_handoff is malformed')
            score_handoff_ok = False
        else:
            if score_handoff.get('status') != 'PASS':
                errors.append('manifest.score_handoff.status must be PASS')
                score_handoff_ok = False
            try:
                _hash_field(
                    score_handoff.get('receipt_sha256'),
                    'manifest.score_handoff.receipt_sha256')
                handoff_fingerprint = _hash_field(
                    score_handoff.get('scorer_fingerprint'),
                    'manifest.score_handoff.scorer_fingerprint')
                _hash_field(
                    score_handoff.get('input_evidence_sha256'),
                    'manifest.score_handoff.input_evidence_sha256')
                _hash_field(
                    score_handoff.get('scored_evidence_sha256'),
                    'manifest.score_handoff.scored_evidence_sha256')
            except BundleVerificationError as exc:
                errors.append(str(exc))
                handoff_fingerprint = None
                score_handoff_ok = False
            declared_scorer = manifest.get('scorer')
            declared_fingerprint = (
                declared_scorer.get('fingerprint')
                if isinstance(declared_scorer, Mapping) else None)
            if (handoff_fingerprint is not None and
                    handoff_fingerprint != declared_fingerprint):
                errors.append('manifest.score_handoff scorer fingerprint drift')
                score_handoff_ok = False
    checks['score_handoff_identity'] = {
        'pass': score_handoff_ok,
        'present': score_handoff is not None,
    }


def _walk_inventory(root: Path) -> tuple[set[str], list[str]]:
    files: set[str] = set()
    errors: list[str] = []
    for directory, dirnames, filenames in os.walk(root, topdown=True, followlinks=False):
        directory_path = Path(directory)
        kept_dirs: list[str] = []
        for name in sorted(dirnames):
            path = directory_path / name
            try:
                info = path.lstat()
            except FileNotFoundError:
                errors.append('bundle tree changed during inventory')
                continue
            if stat.S_ISLNK(info.st_mode):
                errors.append('bundle tree contains a symlink')
                continue
            if not stat.S_ISDIR(info.st_mode):
                errors.append('bundle tree contains a non-directory ancestor')
                continue
            kept_dirs.append(name)
        dirnames[:] = kept_dirs
        for name in sorted(filenames):
            path = directory_path / name
            try:
                info = path.lstat()
            except FileNotFoundError:
                errors.append('bundle tree changed during inventory')
                continue
            if stat.S_ISLNK(info.st_mode):
                errors.append('bundle tree contains a symlink')
            elif not stat.S_ISREG(info.st_mode):
                errors.append('bundle tree contains a non-regular file')
            elif info.st_nlink != 1:
                errors.append('bundle tree contains a hard-linked file')
            else:
                relative = _relative_to_root(root, path, 'bundle inventory path')
                files.add(relative)
    return files, errors


def verify_evidence_bundle(
        root: Path,
        manifest_path: Path | None = None,
        *,
        contract: Mapping[str, Any] | None = None,
        profile: Mapping[str, Any] | None = None,
        expected_profile_sha256: str | None = None,
        expected_profile_name: str | None = None,
        expected_scorer_sha256: str | None = None,
        expected_scorer_fingerprint: str | None = None,
        expected_revision_by_system: Mapping[str, str] | None = None,
        expected_manifest_sha256: str | None = None,
        expected_run_artifacts: Iterable[Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Verify a claim-eligible evidence bundle and return a receipt.

    The returned receipt contains hashes, sizes, paths, and failure reasons,
    never artifact contents.  All failures are represented as
    ``status=FAIL_CLOSED`` with ``pass=false``; callers must not promote an
    existing ``NOT_READY`` result merely because this verifier was run.
    """
    errors: list[str] = []
    checks: dict[str, dict[str, Any]] = {}
    root_display = str(root)
    resolved_root: Path | None = None
    manifest_relative: str | None = None
    manifest_actual_sha: str | None = None
    artifact_receipts: list[dict[str, Any]] = []
    run_artifact_receipts: list[dict[str, Any]] = []
    expected_run_map = _expected_run_artifact_map(expected_run_artifacts, errors)
    try:
        resolved_root = _root_path(Path(root))
    except (OSError, BundleVerificationError, TypeError, ValueError) as exc:
        errors.append(str(exc))

    if resolved_root is not None:
        if manifest_path is None:
            candidate = resolved_root / MANIFEST_FILENAME
        else:
            candidate = Path(manifest_path)
            if candidate.is_absolute():
                try:
                    manifest_relative = _relative_to_root(
                        resolved_root, candidate, 'manifest path')
                except BundleVerificationError as exc:
                    errors.append(str(exc))
            else:
                manifest_relative = _normalise_relative(
                    candidate.as_posix(), 'manifest path')
            candidate = resolved_root / (manifest_relative or MANIFEST_FILENAME)
        if manifest_relative is None:
            manifest_relative = _relative_to_root(
                resolved_root, candidate, 'manifest path')
        try:
            manifest_size, manifest_actual_sha, manifest_bytes = _read_regular(
                resolved_root, manifest_relative, 'bundle manifest',
                include_bytes=True)
            del manifest_size
            try:
                manifest = json.loads(manifest_bytes.decode('utf-8'))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise BundleVerificationError('bundle manifest is not UTF-8 JSON') from exc
            if not isinstance(manifest, dict):
                raise BundleVerificationError('bundle manifest must be a JSON object')
            if manifest.get('schema_version') != SCHEMA_VERSION:
                raise BundleVerificationError('bundle manifest schema_version is unsupported')
            if manifest.get('manifest_kind') != MANIFEST_KIND:
                raise BundleVerificationError('bundle manifest kind is invalid')
            if manifest.get('canonical_root') != '.':
                raise BundleVerificationError('manifest must declare canonical_root "."')
            if manifest.get('manifest_path') != manifest_relative:
                raise BundleVerificationError(
                    'manifest_path does not identify the opened manifest')
            if manifest.get('claim_status') != 'claim_eligible':
                raise BundleVerificationError('bundle claim_status is not claim_eligible')
            declared_manifest_sha = _hash_field(
                manifest.get('manifest_sha256'), 'manifest.manifest_sha256')
            if declared_manifest_sha != canonical_manifest_sha256(manifest):
                raise BundleVerificationError('manifest self SHA-256 mismatch')
            if manifest_bytes != canonical_manifest_bytes(manifest):
                raise BundleVerificationError(
                    'manifest bytes are not deterministic canonical JSON')
            if expected_manifest_sha256 is not None:
                expected_manifest_sha256 = _hash_field(
                    expected_manifest_sha256, 'expected_manifest_sha256')
                if manifest_actual_sha != expected_manifest_sha256:
                    raise BundleVerificationError('manifest file SHA-256 drift')
            declared_sidecar_path = _normalise_relative(
                manifest.get('manifest_sidecar_path'),
                'manifest.manifest_sidecar_path')
            declared_sidecar_sha = _hash_field(
                manifest.get('manifest_sidecar_sha256'),
                'manifest.manifest_sidecar_sha256')
            if not declared_sidecar_path.endswith('.sha256'):
                raise BundleVerificationError('manifest sidecar must use .sha256 suffix')
            sidecar_size, sidecar_actual_sha, sidecar_bytes = _read_regular(
                resolved_root, declared_sidecar_path, 'manifest sidecar',
                include_bytes=True)
            expected_sidecar = (
                f'{declared_manifest_sha}  {manifest_relative}\n'.encode('utf-8'))
            if (sidecar_bytes != expected_sidecar or
                    sidecar_actual_sha != declared_sidecar_sha):
                raise BundleVerificationError('manifest sidecar mismatch')
            checks['manifest'] = {
                'pass': True,
                'path': manifest_relative,
                'sha256': manifest_actual_sha,
                'sidecar_path': declared_sidecar_path,
                'sidecar_sha256': sidecar_actual_sha,
                'bytes': len(manifest_bytes),
                'sidecar_bytes': sidecar_size,
            }
        except (OSError, BundleVerificationError, TypeError, ValueError) as exc:
            errors.append(str(exc))
            checks['manifest'] = {'pass': False}
            manifest = None
    else:
        manifest = None
        checks['manifest'] = {'pass': False}

    if isinstance(manifest, dict) and resolved_root is not None:
        policy = _policy(contract)
        authorization_policy = _authorization_policy(contract)
        expected_auth_profile_sha, _ = _expected_profile(profile)
        authorization_result = verify_fresh_holdout_authorization(
            manifest,
            policy=authorization_policy,
            profile=profile,
            expected_profile_sha256=expected_auth_profile_sha,
        )
        checks['authorization'] = {
            key: value for key, value in authorization_result.items()
            if key not in {'errors', 'not_ready'}
        }
        for item in authorization_result.get('errors', []):
            errors.append('authorization: ' + str(item))
        for item in authorization_result.get('not_ready', []):
            errors.append('authorization NOT_READY: ' + str(item))
        configured_roles = policy.get('required_roles', list(REQUIRED_ROLES))
        optional_roles = policy.get('optional_roles', list(DEFAULT_OPTIONAL_ROLES))
        if not isinstance(configured_roles, list) or not all(
                isinstance(item, str) for item in configured_roles):
            errors.append('required_roles policy is malformed')
            configured_roles = list(REQUIRED_ROLES)
        if not isinstance(optional_roles, list) or not all(
                isinstance(item, str) for item in optional_roles):
            errors.append('optional_roles policy is malformed')
            optional_roles = list(DEFAULT_OPTIONAL_ROLES)
        required_roles = list(dict.fromkeys(configured_roles))
        allowed_roles = set(required_roles) | set(optional_roles)
        if set(required_roles) != set(configured_roles):
            errors.append('required_roles contains duplicate roles')
        entries_value = manifest.get('artifacts')
        entries: list[dict[str, Any]] = []
        path_keys: dict[str, str] = {}
        sidecar_keys: dict[str, str] = {}
        role_names: dict[str, int] = {}
        if not isinstance(entries_value, list) or not entries_value:
            errors.append('manifest.artifacts must be a non-empty list')
            entries_value = []
        for index, raw_entry in enumerate(entries_value):
            if not isinstance(raw_entry, Mapping):
                errors.append(f'artifacts[{index}] must be a mapping')
                continue
            try:
                path = _normalise_relative(raw_entry.get('path'), f'artifacts[{index}].path')
                role = raw_entry.get('role')
                if not isinstance(role, str) or not role:
                    raise BundleVerificationError(f'artifacts[{index}].role is invalid')
                if role in FORBIDDEN_ROLE_NAMES or _forbidden_gt_path(path):
                    # Do not open or hash a possible GT artifact.  Keep this
                    # message path-free so a malformed bundle cannot expose GT
                    # provenance through a diagnostic receipt.
                    raise BundleVerificationError(
                        'ground-truth artifact/role is forbidden before scoring')
                if role not in allowed_roles:
                    raise BundleVerificationError(f'artifacts[{index}] uses undeclared role')
                key = _path_key(path)
                if key in path_keys:
                    raise BundleVerificationError(
                        f'artifacts[{index}] collides with another normalized path')
                path_keys[key] = path
                role_names[role] = role_names.get(role, 0) + 1
                if role_names[role] > 1:
                    raise BundleVerificationError(f'role {role} is declared more than once')
                size_bytes = _size_field(
                    raw_entry.get('size_bytes'), f'artifacts[{index}].size_bytes')
                digest = _hash_field(
                    raw_entry.get('sha256'), f'artifacts[{index}].sha256')
                sidecar_path, sidecar_digest = _entry_sidecar(raw_entry, index)
                if _forbidden_gt_path(sidecar_path):
                    raise BundleVerificationError(
                        'ground-truth sidecar path is forbidden before scoring')
                side_key = _path_key(sidecar_path)
                if side_key in sidecar_keys:
                    raise BundleVerificationError(
                        f'artifacts[{index}] reuses a sidecar path')
                sidecar_keys[side_key] = sidecar_path
                entries.append({
                    'path': path, 'role': role, 'size_bytes': size_bytes,
                    'sha256': digest, 'sidecar_path': sidecar_path,
                    'sidecar_sha256': sidecar_digest, '_index': index,
                })
            except BundleVerificationError as exc:
                errors.append(str(exc))
        missing_roles = sorted(set(required_roles) - set(role_names))
        if missing_roles:
            errors.append('required artifact roles are missing: ' + ', '.join(missing_roles))
        run_bindings: list[dict[str, Any]] = []
        run_binding_keys: set[tuple[str, str, int]] = set()
        raw_run_bindings = manifest.get('run_artifact_bindings')
        if not isinstance(raw_run_bindings, list) or not raw_run_bindings:
            errors.append(
                'manifest.run_artifact_bindings must be a non-empty list')
            raw_run_bindings = []
        for index, raw_binding in enumerate(raw_run_bindings):
            try:
                identity = _run_identity(raw_binding, index)
                if identity in run_binding_keys:
                    raise BundleVerificationError(
                        'run_artifact_bindings contains duplicate run identity')
                run_binding_keys.add(identity)
                refs: dict[str, dict[str, Any]] = {}
                for role in RUN_ARTIFACT_ROLES:
                    refs[role] = _run_artifact_ref(
                        raw_binding.get(role),
                        f'run_artifact_bindings[{index}].{role}')
                run_bindings.append({
                    'system': identity[0], 'dataset': identity[1],
                    'run_index': identity[2], 'artifacts': refs,
                })
            except (BundleVerificationError, TypeError, ValueError) as exc:
                errors.append(str(exc))
        reserved_paths: dict[str, str] = {}
        for reserved_label, reserved_value in (
                ('manifest', manifest_relative),
                ('manifest sidecar', manifest.get('manifest_sidecar_path'))):
            if isinstance(reserved_value, str):
                try:
                    reserved_key = _path_key(_normalise_relative(
                        reserved_value, f'{reserved_label} path'))
                    reserved_paths[reserved_key] = reserved_label
                except BundleVerificationError:
                    pass
        for entry in entries:
            for entry_label in ('path', 'sidecar_path'):
                key = _path_key(entry[entry_label])
                prior = reserved_paths.get(key)
                if prior is not None:
                    errors.append(
                        f'artifact {entry["role"]} collides with {prior} path')
                elif key in reserved_paths:
                    errors.append('bundle path collision alias')
                reserved_paths[key] = f'artifact {entry["role"]} {entry_label}'
        for binding in run_bindings:
            identity_label = (
                f'{binding["system"]}:{binding["dataset"]}:'
                f'{binding["run_index"]}')
            for role, ref in binding['artifacts'].items():
                for ref_label in ('path', 'sidecar_path'):
                    key = _path_key(ref[ref_label])
                    prior = reserved_paths.get(key)
                    if prior is not None:
                        errors.append(
                            f'run artifact {identity_label}/{role} '
                            f'{ref_label} collides with {prior} path')
                    reserved_paths[key] = (
                        f'run artifact {identity_label}/{role} {ref_label}')
        checks['roles'] = {
            'pass': not missing_roles and not any(
                entry.get('role') not in allowed_roles for entry in entries),
            'required': sorted(required_roles),
            'provided': sorted(role_names),
        }

        # Scan the complete root before reopening any listed artifact.  This
        # catches undeclared files and symlinks, including aliases not named in
        # the manifest.
        inventory, inventory_errors = _walk_inventory(resolved_root)
        errors.extend(inventory_errors)
        protected = {manifest_relative}
        manifest_sidecar = manifest.get('manifest_sidecar_path')
        if isinstance(manifest_sidecar, str):
            try:
                protected.add(_normalise_relative(manifest_sidecar, 'manifest sidecar path'))
            except BundleVerificationError:
                pass
        protected.update(entry['path'] for entry in entries)
        protected.update(entry['sidecar_path'] for entry in entries)
        for binding in run_bindings:
            for ref in binding['artifacts'].values():
                protected.add(ref['path'])
                protected.add(ref['sidecar_path'])
        undeclared = sorted(inventory - protected)
        if undeclared:
            errors.append('bundle contains undeclared regular files')
        missing_from_inventory = sorted(protected - inventory)
        if missing_from_inventory:
            errors.append('manifest references missing regular files')
        inventory_keys: dict[str, str] = {}
        for path in inventory:
            key = _path_key(path)
            prior = inventory_keys.get(key)
            if prior is not None and prior != path:
                errors.append('bundle inventory contains a collision alias')
            inventory_keys[key] = path
        checks['root_inventory'] = {
            'pass': not inventory_errors and not undeclared and not missing_from_inventory,
            'file_count': len(inventory),
            'undeclared_count': len(undeclared),
        }

        # GT role/path rejection happens during the preflight above, before
        # this loop opens a single listed artifact.
        artifact_preflight_ok = len(entries) == len(entries_value)
        for entry in sorted(entries, key=lambda item: item['path']):
            try:
                size, actual_sha, _ = _read_regular(
                    resolved_root, entry['path'], f"artifact role {entry['role']}")
                if size != entry['size_bytes']:
                    raise BundleVerificationError(
                        f"artifact {entry['role']} size mismatch")
                if actual_sha != entry['sha256']:
                    raise BundleVerificationError(
                        f"artifact {entry['role']} SHA-256 mismatch")
                side_size, side_sha, side_bytes = _read_regular(
                    resolved_root, entry['sidecar_path'],
                    f"artifact {entry['role']} sidecar", include_bytes=True)
                expected_sidecar = (
                    f"{entry['sha256']}  {entry['path']}\n".encode('utf-8'))
                if side_bytes != expected_sidecar or side_sha != entry['sidecar_sha256']:
                    raise BundleVerificationError(
                        f"artifact {entry['role']} sidecar mismatch")
                artifact_receipts.append({
                    'path': entry['path'], 'role': entry['role'],
                    'size_bytes': size, 'sha256': actual_sha,
                    'sidecar_path': entry['sidecar_path'],
                    'sidecar_sha256': side_sha, 'sidecar_bytes': side_size,
                })
            except (OSError, BundleVerificationError, TypeError, ValueError) as exc:
                errors.append(str(exc))
        checks['artifacts'] = {
            'pass': artifact_preflight_ok and len(artifact_receipts) == len(entries),
            'required_reopened': len(artifact_receipts),
            'declared': len(entries),
        }
        # Every scored run must name its own trajectory, map, and resource
        # bytes.  The evaluator supplies the expected opaque hashes below;
        # this verifier reopens the manifest references and compares both
        # declarations and bytes without parsing any GT-sensitive content.
        run_preflight_ok = len(run_bindings) == len(raw_run_bindings)
        execution_campaigns: dict[str, str] = {}
        actual_run_keys = {
            (binding['system'], binding['dataset'], binding['run_index'])
            for binding in run_bindings
        }
        if expected_run_map is not None:
            expected_run_keys = set(expected_run_map)
            if actual_run_keys != expected_run_keys:
                missing_run_keys = sorted(expected_run_keys - actual_run_keys)
                extra_run_keys = sorted(actual_run_keys - expected_run_keys)
                errors.append(
                    'run artifact coverage does not exactly match scored runs: '
                    f'missing={missing_run_keys}, extra={extra_run_keys}')
                run_preflight_ok = False
        for binding in sorted(
                run_bindings,
                key=lambda item: (item['system'], item['dataset'], item['run_index'])):
            identity = (
                binding['system'], binding['dataset'], binding['run_index'])
            identity_label = f'{identity[0]}:{identity[1]}:{identity[2]}'
            expected_fields = (expected_run_map or {}).get(identity)
            for role in RUN_ARTIFACT_ROLES:
                ref = binding['artifacts'][role]
                execution_projection: dict[str, Any] | None = None
                try:
                    if expected_fields is not None and (
                            ref['sha256'] != expected_fields[role]):
                        raise BundleVerificationError(
                            f'run artifact {identity_label}/{role} SHA does not '
                            'match evaluator run identity')
                    size, actual_sha, raw_bytes = _read_regular(
                        resolved_root, ref['path'],
                        f'run artifact {identity_label}/{role}',
                        include_bytes=(role == 'execution'))
                    if size != ref['size_bytes']:
                        raise BundleVerificationError(
                            f'run artifact {identity_label}/{role} size mismatch')
                    if actual_sha != ref['sha256']:
                        raise BundleVerificationError(
                            f'run artifact {identity_label}/{role} SHA-256 mismatch')
                    if role == 'execution':
                        try:
                            if raw_bytes is None:
                                raise ExecutionReceiptError(
                                    'execution receipt bytes were not reopened')
                            expected_receipt_sha = ref.get('receipt_sha256')
                            if expected_receipt_sha is None:
                                raise ExecutionReceiptError(
                                    'execution receipt canonical SHA is missing')
                            if (expected_fields is not None and
                                    expected_receipt_sha !=
                                    expected_fields['execution_receipt']):
                                raise ExecutionReceiptError(
                                    'execution receipt canonical SHA does not '
                                    'match evaluator run identity')
                            execution_document = json.loads(
                                raw_bytes.decode('utf-8'),
                                object_pairs_hook=_reject_duplicate_keys)
                            execution_projection = validate_receipt(
                                execution_document,
                                expected_system=identity[0],
                                expected_dataset=identity[1],
                                expected_run_index=identity[2],
                                expected_campaign_id=(
                                    expected_fields['campaign']
                                    if expected_fields is not None else None),
                                require_success=True,
                                expected_sha256=expected_receipt_sha)
                        except (UnicodeDecodeError, json.JSONDecodeError,
                                ExecutionReceiptError, TypeError) as exc:
                            raise BundleVerificationError(
                                f'run execution receipt is invalid: {exc}') from exc
                    side_size, side_sha, side_bytes = _read_regular(
                        resolved_root, ref['sidecar_path'],
                        f'run artifact {identity_label}/{role} sidecar',
                        include_bytes=True)
                    expected_sidecar = (
                        f"{ref['sha256']}  {ref['path']}\n".encode('utf-8'))
                    if (side_bytes != expected_sidecar or
                            side_sha != ref['sidecar_sha256']):
                        raise BundleVerificationError(
                            f'run artifact {identity_label}/{role} sidecar mismatch')
                    campaign_id = (execution_projection or {}).get('campaign_id')
                    if role == 'execution' and isinstance(campaign_id, str):
                        prior_campaign = next(iter(execution_campaigns), None)
                        if (prior_campaign is not None and
                                campaign_id != prior_campaign):
                            raise BundleVerificationError(
                                'execution receipts belong to mixed campaign identities')
                        execution_campaigns[identity_label] = campaign_id
                    run_artifact_receipts.append({
                        'system': identity[0], 'dataset': identity[1],
                        'run_index': identity[2], 'role': role,
                        'path': ref['path'], 'size_bytes': size,
                        'sha256': actual_sha,
                        **({'campaign_id': campaign_id}
                           if role == 'execution' else {}),
                        **({'receipt_sha256': ref['receipt_sha256']}
                           if role == 'execution' else {}),
                        'sidecar_path': ref['sidecar_path'],
                        'sidecar_sha256': side_sha, 'sidecar_bytes': side_size,
                    })
                except (OSError, BundleVerificationError, TypeError, ValueError) as exc:
                    errors.append(str(exc))
                    run_preflight_ok = False
        checks['run_artifact_bindings'] = {
            'pass': (run_preflight_ok and
                     len(run_artifact_receipts) ==
                     len(run_bindings) * len(RUN_ARTIFACT_ROLES)),
            'declared_runs': len(run_bindings),
            'required_run_artifacts_per_run': list(RUN_ARTIFACT_ROLES),
            'reopened': len(run_artifact_receipts),
            'expected_run_coverage_supplied': expected_run_map is not None,
            'expected_runs': (len(expected_run_map)
                              if expected_run_map is not None else None),
            'execution_campaign_ids': sorted(set(execution_campaigns.values())),
        }
        _metadata_identity(
            resolved_root, manifest, entries, profile=profile,
            expected_profile_sha256=expected_profile_sha256,
            expected_profile_name=expected_profile_name,
            expected_scorer_sha256=expected_scorer_sha256,
            expected_scorer_fingerprint=expected_scorer_fingerprint,
            expected_revision_by_system=expected_revision_by_system,
            errors=errors, checks=checks)
    else:
        checks.setdefault('authorization', {'pass': False, 'status': 'FAIL_CLOSED'})
        checks.setdefault('roles', {'pass': False})
        checks.setdefault('root_inventory', {'pass': False})
        checks.setdefault('artifacts', {'pass': False})
        checks.setdefault('run_artifact_bindings', {'pass': False})
        checks.setdefault('profile_identity', {'pass': False})
        checks.setdefault('scorer_identity', {'pass': False})
        checks.setdefault('revision_identity', {'pass': False})

    passed = not errors and all(
        bool(value.get('pass')) for value in checks.values())
    authorization_not_ready = (
        checks.get('authorization', {}).get('status') == 'NOT_READY')
    status = ('PASS' if passed else
              'NOT_READY' if authorization_not_ready else 'FAIL_CLOSED')
    return {
        'schema_version': SCHEMA_VERSION,
        'receipt_kind': 'competitive_evidence_bundle_verification',
        'status': status,
        'pass': passed,
        'claim_eligible': passed,
        'canonical_root': root_display,
        'manifest_path': manifest_relative,
        'manifest_sha256': manifest_actual_sha,
        'errors': errors,
        'checks': _jsonable(checks),
        'artifacts': artifact_receipts if passed else [],
        'run_artifacts': run_artifact_receipts if passed else [],
    }


# Short aliases used by callers that treat this as a final-gate library.
verify_bundle = verify_evidence_bundle
verify_competitive_evidence_bundle = verify_evidence_bundle


def _load_profile(path: Path) -> tuple[dict[str, Any], str]:
    if path.is_symlink() or not path.is_file():
        raise BundleVerificationError('profile is missing or unsafe')
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise BundleVerificationError('profile must be a YAML mapping')
    return document, canonical_profile_sha256(document)


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--manifest', type=Path,
                        help='manifest path, relative to --root (default: bundle_manifest.json)')
    parser.add_argument('--profile', type=Path,
                        help='profile YAML used to verify canonical profile identity')
    parser.add_argument('--expected-scorer-sha256')
    parser.add_argument('--expected-scorer-fingerprint')
    parser.add_argument('--expected-revision', action='append', default=[],
                        metavar='SYSTEM=40HEX')
    parser.add_argument('--expected-manifest-sha256')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(list(argv) if argv is not None else None)
    profile = None
    profile_sha = None
    if args.profile is not None:
        profile, profile_sha = _load_profile(args.profile)
    revisions: dict[str, str] = {}
    for value in args.expected_revision:
        if '=' not in value:
            raise BundleVerificationError('--expected-revision must be SYSTEM=40HEX')
        system, revision = value.split('=', 1)
        if not system or _REVISION_RE.fullmatch(revision) is None:
            raise BundleVerificationError('--expected-revision contains invalid identity')
        revisions[system] = revision
    result = verify_evidence_bundle(
        args.root, args.manifest, profile=profile,
        expected_profile_sha256=profile_sha,
        expected_scorer_sha256=args.expected_scorer_sha256,
        expected_scorer_fingerprint=args.expected_scorer_fingerprint,
        expected_revision_by_system=revisions or None,
        expected_manifest_sha256=args.expected_manifest_sha256)
    encoded = json.dumps(result, indent=2, sort_keys=True) + '\n'
    if args.output is not None:
        if args.output.exists() or args.output.is_symlink():
            raise BundleVerificationError(f'refusing to overwrite: {args.output}')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding='utf-8')
    print(encoded, end='')
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, TypeError, json.JSONDecodeError,
            yaml.YAMLError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
