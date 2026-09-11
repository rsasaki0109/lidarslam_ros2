#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""Create and validate an unsigned r3 execution-selection review request.

The request is only a handoff envelope.  It does not contain a signer, private
key, authorization, or promotion result.  A NOT_READY candidate may produce a
request that explicitly lists missing external artifacts; it can never become a
READY receipt through this tool.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import sys
from typing import Any, Mapping

_SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(_SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(_SOURCE_ROOT))

from scripts.validate_competitive_execution_selection_r3 import (  # noqa: E402
    _canonical,
    _require_mapping,
    _require_sha,
    CANDIDATE_REL,
    CandidateError,
    ROOT,
    validate_candidate,
)


HANDOFF_KIND = 'competitive_execution_selection_r3_handoff_v1'
HANDOFF_SCHEMA_VERSION = 1
HANDOFF_SIDECAR_SUFFIX = '.sha256'
HANDOFF_READONLY_MODE = 0o444
MAX_HANDOFF_BYTES = 2 * 1024 * 1024
MAX_HANDOFF_SIDECAR_BYTES = 256
MAX_EXTERNAL_MANIFEST_BYTES = 2 * 1024 * 1024
MAX_EXTERNAL_ARTIFACT_BYTES = 16 * 1024 * 1024
REQUIRED_EXTERNAL_ARTIFACTS = (
    'rival_source_closure', 'dataset_source_closure',
    'fresh_holdout_authorization', 'machine_identity', 'image_inspect',
    'toolchain_capture', 'execution_receipt')
HANDOFF_FIELDS = {
    'schema_version', 'handoff_kind', 'status', 'benchmark_eligible',
    'claim_eligible', 'candidate_binding', 'selection_binding',
    'external_artifact_manifest', 'reviewed_external_artifacts',
    'missing_external_artifacts', 'promotion_policy', 'blockers',
    'handoff_identity_sha256'}


def canonical_handoff_sha256(value: Mapping[str, Any]) -> str:
    payload = {key: item for key, item in value.items()
               if key != 'handoff_identity_sha256'}
    return hashlib.sha256(_canonical(payload)).hexdigest()


def _require_absolute(path: Path, label: str) -> Path:
    if not path.is_absolute() or '..' in path.parts:
        raise CandidateError(f'{label} must be an absolute normalized path')
    return path


def _require_output_path(path: Path, label: str) -> Path:
    path = _require_absolute(path, label)
    if path.name in ('', '.', '..') or any(
            character.isspace() or ord(character) < 32 for character in path.name):
        raise CandidateError(f'{label} basename is not safe for a two-token sidecar')
    return path


def _assert_no_symlink_ancestors(path: Path, label: str) -> None:
    path = _require_absolute(path, label)
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        if current.is_symlink():
            raise CandidateError(f'{label} contains a symlink')


def _directory_identity(path: Path, label: str) -> dict[str, int]:
    path = _require_absolute(path, label)
    _assert_no_symlink_ancestors(path, label)
    try:
        info = os.lstat(path)
    except OSError as exc:
        raise CandidateError(f'{label} cannot be inspected: {exc}') from exc
    if not stat.S_ISDIR(info.st_mode):
        raise CandidateError(f'{label} is not a regular directory')
    return {
        'device': info.st_dev,
        'inode': info.st_ino,
        'mode': stat.S_IMODE(info.st_mode),
    }


def _file_identity(path: Path, label: str, *, max_bytes: int,
                   read_only: bool = True) -> dict[str, int]:
    path = _require_absolute(path, label)
    _assert_no_symlink_ancestors(path, label)
    try:
        info = os.lstat(path)
    except OSError as exc:
        raise CandidateError(f'{label} cannot be inspected: {exc}') from exc
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise CandidateError(f'{label} is not a regular single-link file')
    if info.st_size <= 0 or info.st_size > max_bytes:
        raise CandidateError(f'{label} has an invalid bounded size')
    if read_only and stat.S_IMODE(info.st_mode) != HANDOFF_READONLY_MODE:
        raise CandidateError(f'{label} must be immutable mode 0444')
    return {
        'device': info.st_dev,
        'inode': info.st_ino,
        'mode': stat.S_IMODE(info.st_mode),
        'nlink': info.st_nlink,
        'size': info.st_size,
    }


def _same_identity(first: Mapping[str, int], second: Mapping[str, int]) -> bool:
    return dict(first) == dict(second)


def _read_immutable(path: Path, label: str, *, max_bytes: int,
                    read_only: bool = True) -> tuple[bytes, dict[str, int]]:
    before = _file_identity(path, label, max_bytes=max_bytes,
                            read_only=read_only)
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise CandidateError(f'{label} cannot be opened without following links: {exc}') from exc
    try:
        opened = os.fstat(descriptor)
        opened_identity = {
            'device': opened.st_dev,
            'inode': opened.st_ino,
            'mode': stat.S_IMODE(opened.st_mode),
            'nlink': opened.st_nlink,
            'size': opened.st_size,
        }
        if not _same_identity(before, opened_identity):
            raise CandidateError(f'{label} changed before read')
        chunks: list[bytes] = []
        total = 0
        while True:
            chunk = os.read(descriptor, min(1024 * 1024, max_bytes + 1 - total))
            if not chunk:
                break
            chunks.append(chunk)
            total += len(chunk)
            if total > max_bytes:
                raise CandidateError(f'{label} exceeds the bounded read size')
        closed_identity = os.fstat(descriptor)
        closed = {
            'device': closed_identity.st_dev,
            'inode': closed_identity.st_ino,
            'mode': stat.S_IMODE(closed_identity.st_mode),
            'nlink': closed_identity.st_nlink,
            'size': closed_identity.st_size,
        }
        if not _same_identity(before, closed) or total != before['size']:
            raise CandidateError(f'{label} changed during read')
        payload = b''.join(chunks)
    except OSError as exc:
        raise CandidateError(f'{label} read failed: {exc}') from exc
    finally:
        os.close(descriptor)
    after = _file_identity(path, label, max_bytes=max_bytes,
                           read_only=read_only)
    if not _same_identity(before, after):
        raise CandidateError(f'{label} changed after read')
    return payload, after


def _ensure_parent(path: Path, label: str) -> dict[str, int]:
    parent = path.parent
    _require_absolute(parent, f'{label} parent')
    _assert_no_symlink_ancestors(parent, f'{label} parent')
    try:
        parent.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise CandidateError(f'{label} parent cannot be created: {exc}') from exc
    return _directory_identity(parent, f'{label} parent')


def _fsync_directory(path: Path, label: str) -> None:
    flags = os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise CandidateError(f'{label} parent cannot be opened for fsync: {exc}') from exc
    try:
        os.fsync(descriptor)
    except OSError as exc:
        raise CandidateError(f'{label} parent fsync failed: {exc}') from exc
    finally:
        os.close(descriptor)


def _remove_owned(path: Path, identity: Mapping[str, int], label: str) -> None:
    try:
        current = os.lstat(path)
    except FileNotFoundError:
        return
    except OSError as exc:
        raise CandidateError(f'{label} cleanup inspection failed: {exc}') from exc
    if current.st_dev != identity['device'] or current.st_ino != identity['inode']:
        raise CandidateError(f'{label} cleanup ownership changed')
    if current.st_nlink != 1:
        raise CandidateError(f'{label} cleanup target is no longer single-link')
    try:
        os.unlink(path)
        _fsync_directory(path.parent, f'{label} cleanup')
    except OSError as exc:
        raise CandidateError(f'{label} cleanup failed: {exc}') from exc


def _external_regular(root: Path, relative: Any, label: str) -> Path:
    if not isinstance(relative, str) or not relative or relative.startswith('/'):
        raise CandidateError(f'{label} must be relative to the reviewed artifact root')
    root = _require_absolute(root, 'reviewed artifact root')
    _directory_identity(root, 'reviewed artifact root')
    path = Path(relative)
    if path.is_absolute() or '..' in path.parts:
        raise CandidateError(f'{label} escapes the reviewed artifact root')
    current = root
    for component in path.parts:
        current = current / component
        if current.is_symlink():
            raise CandidateError(f'{label} contains a symlink')
    _file_identity(current, label, max_bytes=MAX_EXTERNAL_ARTIFACT_BYTES)
    return current


def _load_external_manifest(
        path: Path, artifact_root: Path) -> tuple[dict[str, Any], dict[str, int]]:
    path = _require_absolute(path, 'external artifact manifest')
    artifact_root = _require_absolute(artifact_root, 'reviewed artifact root')
    root_identity = _directory_identity(artifact_root, 'reviewed artifact root')
    if path.parent != artifact_root or path.name in ('', '.', '..'):
        raise CandidateError('external artifact manifest must be directly under its root')
    payload, file_identity = _read_immutable(
        path, 'external artifact manifest', max_bytes=MAX_EXTERNAL_MANIFEST_BYTES)
    try:
        document = json.loads(payload.decode('utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CandidateError(f'external artifact manifest is unreadable: {exc}') from exc
    document = dict(_require_mapping(document, 'external artifact manifest'))
    if document.get('schema_version') != 1 or \
            document.get('manifest_kind') != (
                'competitive_execution_selection_r3_external_artifacts_v1'):
        raise CandidateError('external artifact manifest schema identity is invalid')
    artifacts = document.get('artifacts')
    if not isinstance(artifacts, dict) or set(artifacts) != set(REQUIRED_EXTERNAL_ARTIFACTS):
        raise CandidateError('external artifact manifest coverage is not exact')
    for kind in REQUIRED_EXTERNAL_ARTIFACTS:
        item = _require_mapping(artifacts[kind], f'external artifact {kind}')
        if set(item) != {'status', 'path', 'sha256'} or item['status'] != 'REVIEWED_EXTERNAL':
            raise CandidateError(f'external artifact {kind} is not reviewed')
        _require_sha(item['sha256'], f'external artifact {kind}.sha256')
        artifact_path = _external_regular(artifact_root, item['path'],
                                          f'external artifact {kind}.path')
        artifact_bytes, _ = _read_immutable(
            artifact_path, f'external artifact {kind}',
            max_bytes=MAX_EXTERNAL_ARTIFACT_BYTES)
        if hashlib.sha256(artifact_bytes).hexdigest() != item['sha256']:
            raise CandidateError(f'external artifact {kind} SHA is stale')
    if _directory_identity(artifact_root, 'reviewed artifact root') != root_identity:
        raise CandidateError('reviewed artifact root changed during read')
    return document, {
        **root_identity,
        'file_device': file_identity['device'],
        'file_inode': file_identity['inode'],
        'file_mode': file_identity['mode'],
        'file_size': file_identity['size'],
        'file_sha256': hashlib.sha256(payload).hexdigest(),
    }


def _exclusive_write(path: Path, payload: bytes) -> dict[str, int]:
    path = _require_output_path(path, 'handoff output')
    if not payload or len(payload) > MAX_HANDOFF_BYTES:
        raise CandidateError('handoff output exceeds the bounded size')
    parent_identity = _ensure_parent(path, 'handoff output')
    del parent_identity
    if path.exists() or path.is_symlink():
        raise CandidateError(f'output already exists: {path}')
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0)
    descriptor = None
    created_identity: dict[str, int] | None = None
    try:
        descriptor = os.open(path, flags, HANDOFF_READONLY_MODE)
        created = os.fstat(descriptor)
        created_identity = {
            'device': created.st_dev,
            'inode': created.st_ino,
            'mode': stat.S_IMODE(created.st_mode),
            'nlink': created.st_nlink,
            'size': created.st_size,
        }
        view = memoryview(payload)
        while view:
            view = view[os.write(descriptor, view):]
        os.fsync(descriptor)
        os.fchmod(descriptor, HANDOFF_READONLY_MODE)
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        identity = _file_identity(path, 'sealed handoff file',
                                  max_bytes=MAX_HANDOFF_BYTES)
        if identity['size'] != len(payload):
            raise CandidateError('sealed handoff file size changed')
        _fsync_directory(path.parent, 'sealed handoff file')
        return identity
    except Exception:
        if descriptor is not None:
            os.close(descriptor)
        if created_identity is not None:
            _remove_owned(path, created_identity, 'sealed handoff file')
        raise


def _write_sealed_pair(output: Path, document: Mapping[str, Any]) -> tuple[Path, Path]:
    payload = (json.dumps(document, sort_keys=True, indent=2,
                          ensure_ascii=True) + '\n').encode('utf-8')
    output = _require_output_path(output, 'handoff output')
    sidecar = output.with_name(output.name + HANDOFF_SIDECAR_SUFFIX)
    _ensure_parent(output, 'handoff output')
    output_identity = _exclusive_write(output, payload)
    try:
        _exclusive_write(
            sidecar, (hashlib.sha256(payload).hexdigest() + '  ' + output.name + '\n').encode())
    except Exception:
        _remove_owned(output, output_identity, 'handoff pair rollback')
        raise
    return output, sidecar


def _candidate_binding(report: Mapping[str, Any]) -> dict[str, Any]:
    return {
        'path': CANDIDATE_REL,
        'file_sha256': report['candidate_file_sha256'],
        'candidate_identity_sha256': report['candidate_identity_sha256'],
        'candidate_id': report['candidate_id'],
        'status': report['status'],
    }


def prepare_handoff(
        *, candidate_path: Path = ROOT / CANDIDATE_REL,
        output: Path,
        external_manifest: Path | None = None,
        external_root: Path | None = None) -> dict[str, Any]:
    output = _require_output_path(output, 'handoff output')
    _ensure_parent(output, 'handoff output')
    if output.exists() or output.is_symlink():
        raise CandidateError(f'output already exists: {output}')
    sidecar = output.with_name(output.name + HANDOFF_SIDECAR_SUFFIX)
    if sidecar.exists() or sidecar.is_symlink():
        raise CandidateError(f'output sidecar already exists: {sidecar}')
    report = validate_candidate(candidate_path)
    if not report.get('structural_valid'):
        raise CandidateError(f'candidate is not structurally valid: {report.get("error")}')
    candidate = _load_candidate(candidate_path)
    manifest = None
    manifest_binding = None
    reviewed = {}
    missing = list(REQUIRED_EXTERNAL_ARTIFACTS)
    if external_manifest is not None:
        external_manifest = _require_absolute(external_manifest,
                                              'external artifact manifest')
        external_root = external_root or external_manifest.parent
        external_root = _require_absolute(external_root, 'reviewed artifact root')
        if external_manifest.parent != external_root:
            raise CandidateError('external manifest must be directly under its root')
        manifest, manifest_identity = _load_external_manifest(
            external_manifest, external_root)
        root_identity = _directory_identity(external_root, 'reviewed artifact root')
        manifest_binding = {
            'path': external_manifest.name,
            'root': str(external_root),
            'file_sha256': manifest_identity['file_sha256'],
            'root_device': root_identity['device'],
            'root_inode': root_identity['inode'],
            'root_mode': root_identity['mode'],
            'file_device': manifest_identity['file_device'],
            'file_inode': manifest_identity['file_inode'],
            'file_mode': manifest_identity['file_mode'],
            'file_size': manifest_identity['file_size'],
        }
        reviewed = manifest['artifacts']
        missing = []
    document: dict[str, Any] = {
        'schema_version': HANDOFF_SCHEMA_VERSION,
        'handoff_kind': HANDOFF_KIND,
        'status': 'UNSIGNED_REVIEW_REQUIRED',
        'benchmark_eligible': False,
        'claim_eligible': False,
        'candidate_binding': _candidate_binding(report),
        'selection_binding': {
            'profile_path': 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml',
            'profile_canonical_sha256': candidate['profile_binding'][
                'canonical_sha256'],
            'r2_selection_path': candidate['r2_selection_binding']['path'],
            'r2_selection_file_sha256': candidate['r2_selection_binding'][
                'file_sha256'],
            'r2_closure_id': candidate['r2_selection_binding']['closure_id'],
            'r2_closure_identity_sha256': candidate['r2_selection_binding'][
                'closure_identity_sha256'],
        },
        'external_artifact_manifest': manifest_binding,
        'reviewed_external_artifacts': reviewed,
        'missing_external_artifacts': missing,
        'promotion_policy': {
            'ready_receipt_required': True,
            'profile_canonical_reseal_required': True,
            'historical_receipts_immutable': True,
            'active_profile_switch_performed': False,
            'signer_or_private_key_included': False,
        },
        'blockers': report['blockers'],
    }
    document['handoff_identity_sha256'] = canonical_handoff_sha256(document)
    _write_sealed_pair(output, document)
    return document


def _load_candidate(path: Path) -> dict[str, Any]:
    try:
        return dict(json.loads(path.read_text(encoding='utf-8')))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CandidateError(f'candidate cannot be reopened: {exc}') from exc


def validate_handoff(path: Path, *, candidate_path: Path = ROOT / CANDIDATE_REL) -> dict[str, Any]:
    path = _require_output_path(path, 'handoff')
    payload, handoff_identity = _read_immutable(
        path, 'handoff', max_bytes=MAX_HANDOFF_BYTES)
    sidecar = path.with_name(path.name + HANDOFF_SIDECAR_SUFFIX)
    sidecar_payload, sidecar_identity = _read_immutable(
        sidecar, 'handoff sidecar', max_bytes=MAX_HANDOFF_SIDECAR_BYTES)
    expected_sidecar = f'{hashlib.sha256(payload).hexdigest()}  {path.name}\n'.encode()
    if sidecar_payload != expected_sidecar:
        raise CandidateError('handoff sidecar does not bind exact handoff bytes')
    try:
        document = json.loads(payload.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise CandidateError(f'handoff is not valid UTF-8 JSON: {exc}') from exc
    document = _require_mapping(document, 'handoff')
    if set(document) != HANDOFF_FIELDS:
        raise CandidateError('handoff field set is not exact')
    if document.get('schema_version') != HANDOFF_SCHEMA_VERSION or \
            document.get('handoff_kind') != HANDOFF_KIND:
        raise CandidateError('handoff schema identity is invalid')
    if document.get('status') != 'UNSIGNED_REVIEW_REQUIRED' or \
            document.get('benchmark_eligible') is not False or \
            document.get('claim_eligible') is not False:
        raise CandidateError('handoff cannot claim promotion')
    identity = _require_sha(document.get('handoff_identity_sha256'),
                            'handoff_identity_sha256')
    if canonical_handoff_sha256(document) != identity:
        raise CandidateError('handoff self-hash mismatch')
    report = validate_candidate(candidate_path)
    if not report.get('structural_valid'):
        raise CandidateError('bound candidate no longer validates')
    binding = _require_mapping(document.get('candidate_binding'), 'candidate_binding')
    if binding != _candidate_binding(report):
        raise CandidateError('handoff candidate binding is stale or swapped')
    candidate = _load_candidate(candidate_path)
    selection = _require_mapping(document.get('selection_binding'), 'selection_binding')
    expected_selection = {
        'profile_path': candidate['profile_binding']['path'],
        'profile_canonical_sha256': candidate['profile_binding']['canonical_sha256'],
        'r2_selection_path': candidate['r2_selection_binding']['path'],
        'r2_selection_file_sha256': candidate['r2_selection_binding']['file_sha256'],
        'r2_closure_id': candidate['r2_selection_binding']['closure_id'],
        'r2_closure_identity_sha256': candidate['r2_selection_binding']['closure_identity_sha256'],
    }
    if selection != expected_selection:
        raise CandidateError('handoff selection/profile binding is stale or cross-campaign')
    artifacts = document.get('reviewed_external_artifacts')
    missing = document.get('missing_external_artifacts')
    if not isinstance(artifacts, Mapping) or not isinstance(missing, list) or \
            set(artifacts) & set(missing):
        raise CandidateError('handoff external coverage is ambiguous')
    if len(missing) + len(artifacts) != len(REQUIRED_EXTERNAL_ARTIFACTS) or \
            set(artifacts) | set(missing) != set(REQUIRED_EXTERNAL_ARTIFACTS):
        raise CandidateError('handoff external coverage is incomplete or extra')
    manifest_binding = document.get('external_artifact_manifest')
    if manifest_binding is None:
        if artifacts or set(missing) != set(REQUIRED_EXTERNAL_ARTIFACTS):
            raise CandidateError('handoff claims external artifacts without a manifest')
    else:
        manifest_binding = _require_mapping(manifest_binding,
                                            'external_artifact_manifest')
        if set(manifest_binding) != {
                'path', 'root', 'file_sha256', 'root_device', 'root_inode',
                'root_mode', 'file_device', 'file_inode', 'file_mode',
                'file_size'}:
            raise CandidateError('external manifest binding shape is not exact')
        manifest_root = Path(manifest_binding['root'])
        manifest_root = _require_absolute(manifest_root, 'reviewed artifact root')
        root_identity = _directory_identity(manifest_root, 'reviewed artifact root')
        if root_identity != {
                'device': manifest_binding['root_device'],
                'inode': manifest_binding['root_inode'],
                'mode': manifest_binding['root_mode']}:
            raise CandidateError('external artifact root identity is stale')
        manifest_name = manifest_binding['path']
        if not isinstance(manifest_name, str) or not manifest_name or \
                Path(manifest_name).name != manifest_name or \
                manifest_name in ('.', '..'):
            raise CandidateError('external manifest path is not a safe root child')
        manifest_path = manifest_root / manifest_name
        reopened, manifest_identity = _load_external_manifest(
            manifest_path, manifest_root)
        if {
                'device': manifest_identity['device'],
                'inode': manifest_identity['inode'],
                'mode': manifest_identity['mode']} != {
                'device': manifest_binding['root_device'],
                'inode': manifest_binding['root_inode'],
                'mode': manifest_binding['root_mode']}:
            raise CandidateError('external artifact root changed during validation')
        expected_manifest_identity = {
            'file_device': manifest_identity['file_device'],
            'file_inode': manifest_identity['file_inode'],
            'file_mode': manifest_identity['file_mode'],
            'file_size': manifest_identity['file_size'],
        }
        if expected_manifest_identity != {
                'file_device': manifest_binding['file_device'],
                'file_inode': manifest_binding['file_inode'],
                'file_mode': manifest_binding['file_mode'],
                'file_size': manifest_binding['file_size']}:
            raise CandidateError('external manifest file identity is stale')
        if manifest_identity['file_sha256'] != manifest_binding['file_sha256']:
            raise CandidateError('external manifest binding SHA is stale')
        if reopened['artifacts'] != artifacts or missing:
            raise CandidateError('external manifest coverage differs from handoff')
    policy = _require_mapping(document.get('promotion_policy'), 'promotion_policy')
    if policy != {
            'ready_receipt_required': True,
            'profile_canonical_reseal_required': True,
            'historical_receipts_immutable': True,
            'active_profile_switch_performed': False,
            'signer_or_private_key_included': False}:
        raise CandidateError('handoff promotion policy was changed')
    final_payload, final_handoff_identity = _read_immutable(
        path, 'handoff', max_bytes=MAX_HANDOFF_BYTES)
    final_sidecar, final_sidecar_identity = _read_immutable(
        sidecar, 'handoff sidecar', max_bytes=MAX_HANDOFF_SIDECAR_BYTES)
    if not _same_identity(handoff_identity, final_handoff_identity) or \
            not _same_identity(sidecar_identity, final_sidecar_identity) or \
            final_payload != payload or final_sidecar != sidecar_payload:
        raise CandidateError('handoff or sidecar changed during validation')
    return {
        'status': document['status'], 'structural_valid': True,
        'benchmark_eligible': False, 'claim_eligible': False,
        'handoff_identity_sha256': identity,
        'missing_external_artifacts': list(missing),
    }


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    prepare = sub.add_parser('prepare')
    prepare.add_argument('--candidate', type=Path, default=ROOT / CANDIDATE_REL)
    prepare.add_argument('--output', type=Path, required=True)
    prepare.add_argument('--external-manifest', type=Path)
    prepare.add_argument('--external-root', type=Path)
    validate = sub.add_parser('validate')
    validate.add_argument('--handoff', type=Path, required=True)
    validate.add_argument('--candidate', type=Path, default=ROOT / CANDIDATE_REL)
    args = parser.parse_args()
    if args.command == 'prepare':
        result = prepare_handoff(candidate_path=args.candidate, output=args.output,
                                 external_manifest=args.external_manifest,
                                 external_root=args.external_root)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    result = validate_handoff(args.handoff, candidate_path=args.candidate)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(_main())
    except (CandidateError, OSError, UnicodeError, json.JSONDecodeError,
            TypeError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(2)
