#!/usr/bin/env python3
# flake8: noqa
"""Capture and validate the non-promoting r3 source-worktree identity.

The artifact is an observation, not an external review.  It records only
hashes and metadata for untracked files (never their contents), while the
tracked diff hash uses the same contract as
``capture_competitive_execution_identity.capture_git_provenance``.  The git
commands are fixed in this module; callers cannot provide a shell command.
No dataset, ground truth, scorer, container, or network is opened.
"""

from __future__ import annotations

import argparse
import datetime as datetime_module
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys
from typing import Any, Callable, Mapping

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from scripts import capture_competitive_execution_identity as identity  # noqa: E402
from scripts.validate_competitive_execution_selection_r3 import (  # noqa: E402
    CANDIDATE_REL,
    CandidateError,
    validate_candidate,
)


ROOT = _ROOT
SCHEMA = 'competitive_execution_source_worktree_snapshot_v1'
SIDECAR_SCHEMA = 'competitive_execution_source_worktree_snapshot_sidecar_v1'
SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_r3_source_worktree_v1.schema.json')
STRICT_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_source_worktree_snapshot_v1.schema.json')
SIDECAR_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_r3_source_worktree_sidecar_v1.schema.json')
SCRIPT_REL = 'scripts/capture_competitive_execution_r3_source_worktree.py'
SCRIPT = ROOT / SCRIPT_REL
IDENTITY_CONTRACT_REL = 'scripts/capture_competitive_execution_identity.py'
IMPLEMENTATION_REL = 'scripts/capture_competitive_execution_source_worktree.py'
IMPLEMENTATION = ROOT / IMPLEMENTATION_REL
CAMPAIGN_ID = 'competitive-execution-selection-2026-08-r3-candidate'
READ_ONLY_MODE = 0o444
MAX_SNAPSHOT_BYTES = 64 * 1024 * 1024
MAX_SIDECAR_BYTES = 4096
MAX_UNTRACKED_FILES = 8192
MAX_UNTRACKED_FILE_BYTES = 16 * 1024 * 1024
MAX_UNTRACKED_TOTAL_BYTES = 128 * 1024 * 1024
MAX_GIT_OUTPUT_BYTES = 64 * 1024 * 1024
SHA_RE = re.compile(r'^[0-9a-f]{64}$')
REVISION_RE = re.compile(r'^[0-9a-f]{40}$')

FIXED_GIT_PROBES: tuple[tuple[str, ...], ...] = (
    ('rev-parse', 'HEAD'),
    ('status', '--porcelain=v1', '--untracked-files=all'),
    ('diff', '--binary', '--no-ext-diff'),
    ('diff', '--cached', '--binary', '--no-ext-diff'),
    ('ls-files', '--others', '--exclude-standard'),
    ('submodule', 'status', '--recursive'),
)


class SourceWorktreeError(ValueError):
    """The source snapshot is malformed, stale, or unsafe to capture."""


RunProbe = Callable[[list[str], Path], tuple[int, str, str]]


def _canonical(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def _hash(value: Any) -> str:
    payload = value if isinstance(value, bytes) else _canonical(value)
    return hashlib.sha256(payload).hexdigest()


def _sha_file(path: Path, *, limit: int | None = None) -> tuple[str, int]:
    digest = hashlib.sha256()
    total = 0
    try:
        with path.open('rb') as stream:
            while True:
                block = stream.read(1024 * 1024)
                if not block:
                    break
                total += len(block)
                if limit is not None and total > limit:
                    raise SourceWorktreeError(f'file exceeds bound: {path}')
                digest.update(block)
    except OSError as error:
        raise SourceWorktreeError(f'cannot hash {path}: {error}') from error
    return digest.hexdigest(), total


def _absolute(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.startswith('/') or '\x00' in value:
        raise SourceWorktreeError(f'{label} must be absolute')
    path = Path(value)
    if path == Path('/') or path.as_posix() != value or '..' in path.parts:
        raise SourceWorktreeError(f'{label} is not normalized')
    return path


def _directory_identity(path: Path, label: str) -> dict[str, int]:
    path = _absolute(str(path), label)
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        try:
            item = os.lstat(current)
        except OSError as error:
            raise SourceWorktreeError(f'{label} is unavailable') from error
        if stat.S_ISLNK(item.st_mode):
            raise SourceWorktreeError(f'{label} contains a symlink')
    try:
        item = os.lstat(path)
    except OSError as error:
        raise SourceWorktreeError(f'{label} cannot be inspected') from error
    if not stat.S_ISDIR(item.st_mode) or stat.S_ISLNK(item.st_mode):
        raise SourceWorktreeError(f'{label} is not a directory')
    return {'device': item.st_dev, 'inode': item.st_ino,
            'mode': stat.S_IMODE(item.st_mode)}


def _safe_relative_file(root: Path, relative: str, label: str) -> Path:
    if not isinstance(relative, str) or not relative or '\x00' in relative:
        raise SourceWorktreeError(f'{label} path is invalid')
    candidate = Path(relative)
    if candidate.is_absolute() or '..' in candidate.parts or relative.startswith('/'):
        raise SourceWorktreeError(f'{label} path escapes the worktree')
    current = root
    for component in candidate.parts:
        current /= component
        try:
            item = os.lstat(current)
        except OSError as error:
            raise SourceWorktreeError(f'{label} path is unavailable') from error
        if stat.S_ISLNK(item.st_mode):
            raise SourceWorktreeError(f'{label} path contains a symlink')
    try:
        item = os.lstat(current)
    except OSError as error:
        raise SourceWorktreeError(f'{label} path is unavailable') from error
    if not stat.S_ISREG(item.st_mode) or item.st_nlink != 1:
        raise SourceWorktreeError(f'{label} is not a regular single-link file')
    if item.st_size > MAX_UNTRACKED_FILE_BYTES:
        raise SourceWorktreeError(f'{label} exceeds the per-file bound')
    return current


def _read_regular(path: Path, label: str) -> tuple[bytes, dict[str, int]]:
    """Read one untracked file through an O_NOFOLLOW descriptor."""
    before = os.lstat(path)
    if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1 or \
            before.st_size > MAX_UNTRACKED_FILE_BYTES:
        raise SourceWorktreeError(f'{label} is not a bounded regular file')
    expected = (before.st_dev, before.st_ino, before.st_nlink,
                before.st_size, stat.S_IMODE(before.st_mode))
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise SourceWorktreeError(f'{label} cannot be opened safely') from error
    chunks: list[bytes] = []
    total = 0
    try:
        opened = os.fstat(descriptor)
        opened_identity = (opened.st_dev, opened.st_ino, opened.st_nlink,
                           opened.st_size, stat.S_IMODE(opened.st_mode))
        if opened_identity != expected:
            raise SourceWorktreeError(f'{label} changed before read')
        while True:
            block = os.read(descriptor, min(1024 * 1024,
                                            MAX_UNTRACKED_FILE_BYTES + 1 - total))
            if not block:
                break
            chunks.append(block)
            total += len(block)
            if total > MAX_UNTRACKED_FILE_BYTES:
                raise SourceWorktreeError(f'{label} exceeds the read bound')
        closed = os.fstat(descriptor)
        closed_identity = (closed.st_dev, closed.st_ino, closed.st_nlink,
                           closed.st_size, stat.S_IMODE(closed.st_mode))
        if closed_identity != expected or total != before.st_size:
            raise SourceWorktreeError(f'{label} changed during read')
    except OSError as error:
        raise SourceWorktreeError(f'{label} read failed') from error
    finally:
        os.close(descriptor)
    after = os.lstat(path)
    after_identity = (after.st_dev, after.st_ino, after.st_nlink,
                      after.st_size, stat.S_IMODE(after.st_mode))
    if after_identity != expected:
        raise SourceWorktreeError(f'{label} changed after read')
    return b''.join(chunks), {
        'device': before.st_dev, 'inode': before.st_ino,
        'nlink': before.st_nlink, 'size': before.st_size,
        'mode': stat.S_IMODE(before.st_mode),
    }


def _default_runner(command: list[str], cwd: Path) -> tuple[int, str, str]:
    return identity.run_read_only(command, cwd=cwd)


def _probe(root: Path, args: tuple[str, ...], runner: RunProbe) -> str:
    command = ['git', '-C', str(root), *args]
    result = runner(command, root)
    if not isinstance(result, tuple) or len(result) != 3:
        raise SourceWorktreeError('git probe returned an invalid result')
    returncode, stdout, stderr = result
    if type(returncode) is not int or returncode != 0:
        raise SourceWorktreeError(
            f'fixed git probe failed: {args[0]}: {str(stderr)[:256]}')
    if not isinstance(stdout, str) or '\x00' in stdout:
        raise SourceWorktreeError(f'fixed git probe output is invalid: {args[0]}')
    if len(stdout.encode('utf-8')) > MAX_GIT_OUTPUT_BYTES or \
            len(str(stderr).encode('utf-8')) > 4096:
        raise SourceWorktreeError(f'fixed git probe output is oversized: {args[0]}')
    # A clean submodule is represented by a leading space.  Do not strip it
    # before parsing; the other fixed probes have no semantic leading space.
    return stdout.rstrip('\r\n') if args[0] == 'submodule' else stdout.strip()


def _submodules(raw: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    seen: set[str] = set()
    for line in raw.splitlines():
        if not line:
            continue
        # ``identity.run_read_only`` historically strips the complete stdout
        # string.  That removes the leading clean-submodule marker from the
        # first line only; restore it from the unambiguous 40-hex layout.
        if len(line) >= 41 and REVISION_RE.fullmatch(line[:40]) and line[40] == ' ':
            line = ' ' + line
        if len(line) < 42 or line[0] not in ' +-U' or line[1:41].lower() != line[1:41] or \
                not REVISION_RE.fullmatch(line[1:41]):
            raise SourceWorktreeError('submodule status output is malformed')
        remainder = line[42:].strip()
        path = remainder.split(' ', 1)[0]
        if not path or path in seen or Path(path).is_absolute() or '..' in Path(path).parts:
            raise SourceWorktreeError('submodule path is invalid or duplicated')
        seen.add(path)
        marker = line[0]
        rows.append({'path': path, 'revision': line[1:41],
                     'status': {' ': 'clean', '+': 'modified',
                                '-': 'uninitialized', 'U': 'conflicted'}[marker]})
    return sorted(rows, key=lambda item: item['path'])


def _worktree(root: Path, runner: RunProbe = _default_runner) -> dict[str, Any]:
    root = _absolute(str(root), 'worktree root')
    root_identity = _directory_identity(root, 'worktree root')
    revision = _probe(root, ('rev-parse', 'HEAD'), runner)
    if not REVISION_RE.fullmatch(revision):
        raise SourceWorktreeError('HEAD is not a full lowercase revision')
    status = _probe(root, ('status', '--porcelain=v1', '--untracked-files=all'), runner)
    diff = _probe(root, ('diff', '--binary', '--no-ext-diff'), runner)
    cached = _probe(root, ('diff', '--cached', '--binary', '--no-ext-diff'), runner)
    listed = _probe(root, ('ls-files', '--others', '--exclude-standard'), runner)
    submodule_rows = _submodules(
        _probe(root, ('submodule', 'status', '--recursive'), runner))
    relative_paths = sorted(item for item in listed.splitlines() if item)
    if len(relative_paths) > MAX_UNTRACKED_FILES or len(set(relative_paths)) != len(relative_paths):
        raise SourceWorktreeError('untracked file list is duplicated or oversized')
    rows: list[dict[str, Any]] = []
    hash_rows: list[dict[str, str]] = []
    total = 0
    for relative in relative_paths:
        path = _safe_relative_file(root, relative, 'untracked file')
        data, _file_identity = _read_regular(path, f'untracked file {relative}')
        total += len(data)
        if total > MAX_UNTRACKED_TOTAL_BYTES:
            raise SourceWorktreeError('untracked content exceeds total bound')
        file_sha = _hash(data)
        rows.append({'path': relative, 'sha256': file_sha, 'bytes': len(data)})
        hash_rows.append({'path': relative, 'sha256': file_sha})
    tracked_diff_sha = _hash((diff + '\n' + cached).encode())
    untracked_sha = identity.canonical_hash(hash_rows)
    dirty = bool(status)
    clean_provenance = identity.canonical_hash({
        'revision': revision,
        'tracked_diff_sha256': tracked_diff_sha,
        'untracked_content_sha256': untracked_sha,
        'worktree_dirty': dirty,
    })
    result = {
        'root': str(root),
        'root_device': root_identity['device'],
        'root_inode': root_identity['inode'],
        'root_mode': root_identity['mode'],
        'revision': revision,
        'worktree_dirty': dirty,
        'tracked_diff_sha256': tracked_diff_sha,
        'tracked_diff_bytes': len((diff + '\n' + cached).encode()),
        'untracked_content_sha256': untracked_sha,
        'untracked_files': rows,
        'untracked_file_count': len(rows),
        'submodules': submodule_rows,
        'submodule_identity_sha256': identity.canonical_hash(submodule_rows),
        'clean_provenance_sha256': clean_provenance,
        'status': 'observed',
    }
    after = _directory_identity(root, 'worktree root')
    if after != root_identity:
        raise SourceWorktreeError('worktree root changed during capture')
    return result


def _candidate_contract() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    candidate_path = ROOT / CANDIDATE_REL
    report = validate_candidate(candidate_path)
    if report.get('status') != 'NOT_READY' or report.get('benchmark_eligible') is not False or \
            report.get('claim_eligible') is not False or not report.get('structural_valid'):
        raise SourceWorktreeError('r3 candidate is not structurally valid NOT_READY')
    try:
        candidate = json.loads(candidate_path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise SourceWorktreeError('r3 candidate cannot be reopened') from error
    if not isinstance(candidate, dict):
        raise SourceWorktreeError('r3 candidate is not an object')
    binding = {
        'path': CANDIDATE_REL,
        'file_sha256': report['candidate_file_sha256'],
        'candidate_identity_sha256': report['candidate_identity_sha256'],
        'candidate_id': report['candidate_id'],
        'status': report['status'],
    }
    profile = candidate.get('profile_binding')
    if not isinstance(profile, dict) or set(profile) != {
            'path', 'file_sha256', 'canonical_sha256', 'canonical_hash_kind'}:
        raise SourceWorktreeError('candidate profile binding is malformed')
    source_bindings = candidate.get('source_bindings')
    if not isinstance(source_bindings, dict):
        raise SourceWorktreeError('candidate source bindings are missing')
    producer = {
        'snapshot_producer': {
            'path': SCRIPT_REL,
            'file_sha256': _hash(SCRIPT.read_bytes()),
            'schema_path': SCHEMA_REL,
        },
        'snapshot_schema': {
            'path': SCHEMA_REL,
            'file_sha256': _hash((ROOT / SCHEMA_REL).read_bytes()),
        },
        'snapshot_strict_schema': {
            'path': STRICT_SCHEMA_REL,
            'file_sha256': _hash((ROOT / STRICT_SCHEMA_REL).read_bytes()),
        },
        'snapshot_sidecar_schema': {
            'path': SIDECAR_SCHEMA_REL,
            'file_sha256': _hash((ROOT / SIDECAR_SCHEMA_REL).read_bytes()),
        },
        'snapshot_implementation': {
            'path': IMPLEMENTATION_REL,
            'file_sha256': _hash(IMPLEMENTATION.read_bytes()),
        },
        'identity_contract': {
            'path': IDENTITY_CONTRACT_REL,
            'file_sha256': _hash((ROOT / IDENTITY_CONTRACT_REL).read_bytes()),
        },
    }
    source = {
        'schema': 'competitive_execution_selection_r3_source_manifest_v1',
        'schema_version': 1,
        'candidate_path': CANDIDATE_REL,
        'candidate_file_sha256': report['candidate_file_sha256'],
        'candidate_identity_sha256': report['candidate_identity_sha256'],
        'source_bindings_sha256': identity.canonical_hash(source_bindings),
        'source_bindings': source_bindings,
        'source_provenance_status': candidate.get('source_provenance', {}).get('status'),
        'producer_bindings': producer,
    }
    return binding, dict(profile), source


def _snapshot_document(root: Path, *, runner: RunProbe = _default_runner,
                       captured_at: str | None = None) -> dict[str, Any]:
    candidate, profile, source = _candidate_contract()
    worktree = _worktree(root, runner)
    document: dict[str, Any] = {
        'schema': SCHEMA,
        'schema_version': 1,
        'status': 'NOT_REVIEWED_EXTERNAL',
        'benchmark_eligible': False,
        'claim_eligible': False,
        'active_profile_switch': False,
        'campaign_id': CAMPAIGN_ID,
        'captured_at': captured_at or datetime_module.datetime.now(
            datetime_module.timezone.utc).isoformat().replace('+00:00', 'Z'),
        'candidate_binding': candidate,
        'profile_binding': profile,
        'source_manifest': source,
        'worktree': worktree,
        'capture_policy': {
            'fixed_git_probes': [list(item) for item in FIXED_GIT_PROBES],
            'network': 'none', 'docker': 'not_used', 'dataset_gt_scorer': 'not_opened',
            'raw_untracked_content_in_artifact': False,
            'symlink_policy': 'reject', 'hardlink_policy': 'reject',
            'max_git_output_bytes': MAX_GIT_OUTPUT_BYTES,
            'max_untracked_file_bytes': MAX_UNTRACKED_FILE_BYTES,
            'max_untracked_total_bytes': MAX_UNTRACKED_TOTAL_BYTES,
        },
        'snapshot_identity_sha256': '',
    }
    document['snapshot_identity_sha256'] = _hash({
        key: value for key, value in document.items()
        if key != 'snapshot_identity_sha256'})
    return document


def _file_identity(path: Path, label: str, *, limit: int,
                   immutable: bool = True) -> dict[str, int]:
    path = _absolute(str(path), label)
    current = Path(path.anchor)
    for component in path.parts[1:-1]:
        current /= component
        try:
            item = os.lstat(current)
        except OSError as error:
            raise SourceWorktreeError(f'{label} parent is unavailable') from error
        if stat.S_ISLNK(item.st_mode):
            raise SourceWorktreeError(f'{label} parent contains a symlink')
    try:
        item = os.lstat(path)
    except OSError as error:
        raise SourceWorktreeError(f'{label} is unavailable') from error
    if not stat.S_ISREG(item.st_mode) or item.st_nlink != 1 or item.st_size <= 0 or item.st_size > limit:
        raise SourceWorktreeError(f'{label} is not bounded regular single-link')
    if immutable and stat.S_IMODE(item.st_mode) != READ_ONLY_MODE:
        raise SourceWorktreeError(f'{label} is not immutable mode 0444')
    return {'device': item.st_dev, 'inode': item.st_ino,
            'nlink': item.st_nlink, 'size': item.st_size,
            'mode': stat.S_IMODE(item.st_mode)}


def _read_immutable(path: Path, label: str, *, limit: int,
                    immutable: bool = True) -> tuple[bytes, dict[str, int]]:
    before = _file_identity(path, label, limit=limit, immutable=immutable)
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0)
    try:
        fd = os.open(path, flags)
    except OSError as error:
        raise SourceWorktreeError(f'{label} cannot be opened safely') from error
    chunks: list[bytes] = []
    total = 0
    try:
        opened = os.fstat(fd)
        opened_identity = {'device': opened.st_dev, 'inode': opened.st_ino,
                           'nlink': opened.st_nlink, 'size': opened.st_size,
                           'mode': stat.S_IMODE(opened.st_mode)}
        if opened_identity != before:
            raise SourceWorktreeError(f'{label} changed before read')
        while True:
            block = os.read(fd, min(1024 * 1024, limit + 1 - total))
            if not block:
                break
            chunks.append(block)
            total += len(block)
            if total > limit:
                raise SourceWorktreeError(f'{label} exceeds read bound')
        closed = os.fstat(fd)
        closed_identity = {'device': closed.st_dev, 'inode': closed.st_ino,
                           'nlink': closed.st_nlink, 'size': closed.st_size,
                           'mode': stat.S_IMODE(closed.st_mode)}
        if closed_identity != before or total != before['size']:
            raise SourceWorktreeError(f'{label} changed during read')
    except OSError as error:
        raise SourceWorktreeError(f'{label} read failed') from error
    finally:
        os.close(fd)
    after = _file_identity(path, label, limit=limit, immutable=immutable)
    if after != before:
        raise SourceWorktreeError(f'{label} changed after read')
    return b''.join(chunks), after


def _directory_fsync(path: Path, label: str) -> None:
    fd = os.open(path, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0) |
                 getattr(os, 'O_NOFOLLOW', 0))
    try:
        os.fsync(fd)
    except OSError as error:
        raise SourceWorktreeError(f'{label} directory fsync failed') from error
    finally:
        os.close(fd)


def _parent_identity(path: Path, label: str) -> dict[str, int]:
    return _directory_identity(path.parent, f'{label} parent')


def _exclusive_write(path: Path, payload: bytes, label: str,
                     parent: Mapping[str, int]) -> dict[str, int]:
    descriptor = None
    owned: tuple[int, int] | None = None
    try:
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                             getattr(os, 'O_NOFOLLOW', 0), READ_ONLY_MODE)
        opened = os.fstat(descriptor)
        owned = (opened.st_dev, opened.st_ino)
        offset = 0
        while offset < len(payload):
            count = os.write(descriptor, payload[offset:])
            if count <= 0:
                raise SourceWorktreeError(f'{label} write made no progress')
            offset += count
        os.fchmod(descriptor, READ_ONLY_MODE)
        os.fsync(descriptor)
        final = os.fstat(descriptor)
        if (final.st_dev, final.st_ino) != owned or final.st_nlink != 1 or \
                final.st_size != len(payload) or stat.S_IMODE(final.st_mode) != READ_ONLY_MODE:
            raise SourceWorktreeError(f'{label} identity changed during seal')
        os.close(descriptor)
        descriptor = None
        identity = _file_identity(path, label, limit=MAX_SNAPSHOT_BYTES)
        if identity['size'] != len(payload) or dict(_parent_identity(path, label)) != dict(parent):
            raise SourceWorktreeError(f'{label} failed post-seal identity check')
        return identity
    except Exception:
        if descriptor is not None:
            os.close(descriptor)
        if owned is not None:
            try:
                current = os.lstat(path)
                if (current.st_dev, current.st_ino) == owned and current.st_nlink == 1:
                    os.unlink(path)
                    _directory_fsync(path.parent, f'{label} cleanup')
            except FileNotFoundError:
                pass
            except OSError as error:
                raise SourceWorktreeError(f'{label} cleanup failed') from error
        raise


def _seal_pair(output: Path, document: Mapping[str, Any]) -> tuple[Path, Path]:
    output = _absolute(str(output), 'snapshot output')
    if output.name in {'', '.', '..'} or output.name.endswith('.sha256.json'):
        raise SourceWorktreeError('snapshot output name is unsafe')
    parent = output.parent
    parent_id = _directory_identity(parent, 'snapshot output parent')
    if output.exists() or output.is_symlink():
        raise SourceWorktreeError('snapshot output already exists')
    sidecar = output.with_name(output.name + '.sha256.json')
    if sidecar.exists() or sidecar.is_symlink():
        raise SourceWorktreeError('snapshot sidecar already exists')
    payload = (json.dumps(document, sort_keys=True, indent=2,
                          ensure_ascii=True) + '\n').encode('utf-8')
    if len(payload) > MAX_SNAPSHOT_BYTES:
        raise SourceWorktreeError('snapshot is oversized')
    artifact_id: dict[str, int] | None = None
    sidecar_id: dict[str, int] | None = None
    try:
        artifact_id = _exclusive_write(output, payload, 'snapshot', parent_id)
        sidecar_doc = {
            'schema': SIDECAR_SCHEMA, 'schema_version': 1,
            'artifact_path': output.name, 'artifact_bytes': len(payload),
            'artifact_sha256': _hash(payload),
            'snapshot_identity_sha256': document['snapshot_identity_sha256'],
        }
        side_payload = _canonical(sidecar_doc)
        if len(side_payload) > MAX_SIDECAR_BYTES:
            raise SourceWorktreeError('snapshot sidecar is oversized')
        sidecar_id = _exclusive_write(sidecar, side_payload,
                                      'snapshot sidecar', parent_id)
        _directory_fsync(parent, 'snapshot pair')
        _read_immutable(output, 'snapshot recheck', limit=MAX_SNAPSHOT_BYTES)
        _read_immutable(sidecar, 'snapshot sidecar recheck', limit=MAX_SIDECAR_BYTES)
        if dict(_parent_identity(output, 'snapshot pair')) != dict(parent_id):
            raise SourceWorktreeError('snapshot output parent changed')
        return output, sidecar
    except Exception:
        for path, owned in ((sidecar, sidecar_id), (output, artifact_id)):
            try:
                current = os.lstat(path)
            except FileNotFoundError:
                continue
            except OSError as error:
                raise SourceWorktreeError('snapshot cleanup inspection failed') from error
            if owned is not None and (current.st_dev, current.st_ino) == (
                    owned['device'], owned['inode']):
                if current.st_nlink == 1 and stat.S_ISREG(current.st_mode):
                    try:
                        os.unlink(path)
                    except OSError as error:
                        raise SourceWorktreeError('snapshot cleanup failed') from error
        raise


def _read_pair(path: Path) -> tuple[dict[str, Any], bytes, dict[str, int], dict[str, Any]]:
    payload, file_id = _read_immutable(path, 'source snapshot', limit=MAX_SNAPSHOT_BYTES)
    try:
        value = json.loads(payload.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise SourceWorktreeError('source snapshot is not valid JSON') from error
    if not isinstance(value, dict):
        raise SourceWorktreeError('source snapshot is not an object')
    sidecar = path.with_name(path.name + '.sha256.json')
    side_payload, side_id = _read_immutable(sidecar, 'source snapshot sidecar', limit=MAX_SIDECAR_BYTES)
    try:
        side = json.loads(side_payload.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise SourceWorktreeError('source snapshot sidecar is not valid JSON') from error
    if not isinstance(side, dict):
        raise SourceWorktreeError('source snapshot sidecar is not an object')
    expected = {
        'schema': SIDECAR_SCHEMA, 'schema_version': 1,
        'artifact_path': path.name, 'artifact_bytes': len(payload),
        'artifact_sha256': _hash(payload),
        'snapshot_identity_sha256': value.get('snapshot_identity_sha256'),
    }
    if side != expected:
        raise SourceWorktreeError('source snapshot sidecar binding drift')
    payload2, file_id2 = _read_immutable(path, 'source snapshot recheck', limit=MAX_SNAPSHOT_BYTES)
    side2, side_id2 = _read_immutable(sidecar, 'source snapshot sidecar recheck', limit=MAX_SIDECAR_BYTES)
    if payload2 != payload or side2 != side_payload or file_id2 != file_id or side_id2 != side_id:
        raise SourceWorktreeError('source snapshot changed during validation')
    return value, payload, file_id, side


def validate_snapshot(path: Path, *, root: Path = ROOT,
                      runner: RunProbe = _default_runner) -> dict[str, Any]:
    path = _absolute(str(path), 'snapshot')
    value, payload, _file_id, _side = _read_pair(path)
    required = {
        'schema', 'schema_version', 'status', 'benchmark_eligible',
        'claim_eligible', 'active_profile_switch', 'campaign_id', 'captured_at',
        'candidate_binding', 'profile_binding', 'source_manifest', 'worktree',
        'capture_policy', 'snapshot_identity_sha256'}
    if set(value) != required or value['schema'] != SCHEMA or value['schema_version'] != 1:
        raise SourceWorktreeError('source snapshot shape is not exact')
    if value['status'] != 'NOT_REVIEWED_EXTERNAL' or value['benchmark_eligible'] is not False or \
            value['claim_eligible'] is not False or value['active_profile_switch'] is not False or \
            value['campaign_id'] != CAMPAIGN_ID:
        raise SourceWorktreeError('source snapshot is promoted or cross-campaign')
    if not isinstance(value['captured_at'], str) or not value['captured_at'].endswith('Z'):
        raise SourceWorktreeError('capture timestamp is malformed')
    if value['snapshot_identity_sha256'] != _hash({
            key: item for key, item in value.items()
            if key != 'snapshot_identity_sha256'}):
        raise SourceWorktreeError('source snapshot identity is stale')
    expected_candidate, expected_profile, expected_source = _candidate_contract()
    if value['candidate_binding'] != expected_candidate or value['profile_binding'] != expected_profile or \
            value['source_manifest'] != expected_source:
        raise SourceWorktreeError('candidate/profile/source manifest drift')
    expected_policy = {
        'fixed_git_probes': [list(item) for item in FIXED_GIT_PROBES],
        'network': 'none', 'docker': 'not_used', 'dataset_gt_scorer': 'not_opened',
        'raw_untracked_content_in_artifact': False,
        'symlink_policy': 'reject', 'hardlink_policy': 'reject',
        'max_git_output_bytes': MAX_GIT_OUTPUT_BYTES,
        'max_untracked_file_bytes': MAX_UNTRACKED_FILE_BYTES,
        'max_untracked_total_bytes': MAX_UNTRACKED_TOTAL_BYTES,
    }
    if value['capture_policy'] != expected_policy:
        raise SourceWorktreeError('capture policy was altered')
    root = _absolute(str(root), 'validation root')
    if value['worktree'].get('root') != str(root):
        raise SourceWorktreeError('snapshot root binding drift')
    current = _worktree(root, runner)
    recorded = dict(value['worktree'])
    # The timestamp is the only intentionally non-reproducible field and is
    # outside the worktree identity.  Everything else must be byte-for-byte
    # equal, including every untracked path and content hash.
    if recorded != current:
        raise SourceWorktreeError('source worktree changed after capture')
    return {
        'status': 'NOT_REVIEWED_EXTERNAL', 'structural_valid': True,
        'benchmark_eligible': False, 'claim_eligible': False,
        'active_profile_switch': False, 'campaign_id': CAMPAIGN_ID,
        'snapshot_identity_sha256': value['snapshot_identity_sha256'],
        'revision': current['revision'],
        'tracked_diff_sha256': current['tracked_diff_sha256'],
        'untracked_content_sha256': current['untracked_content_sha256'],
        'submodule_identity_sha256': current['submodule_identity_sha256'],
    }


def capture_snapshot(*, root: Path = ROOT, output: Path,
                     runner: RunProbe = _default_runner,
                     captured_at: str | None = None) -> dict[str, Any]:
    root = _absolute(str(root), 'worktree root')
    output = _absolute(str(output), 'snapshot output')
    # A snapshot must not become one of the untracked files it records.
    try:
        output.relative_to(root)
    except ValueError:
        pass
    else:
        raise SourceWorktreeError('snapshot output must be outside worktree root')
    document = _snapshot_document(root, runner=runner, captured_at=captured_at)
    _seal_pair(output, document)
    return document


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    capture_parser = sub.add_parser('capture')
    capture_parser.add_argument('--root', type=Path, default=ROOT)
    capture_parser.add_argument('--output', type=Path, required=True)
    validate_parser = sub.add_parser('validate')
    validate_parser.add_argument('--root', type=Path, default=ROOT)
    validate_parser.add_argument('--snapshot', type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.command == 'capture':
            result = capture_snapshot(root=args.root, output=args.output)
            print(json.dumps({
                'status': result['status'],
                'snapshot_identity_sha256': result['snapshot_identity_sha256'],
                'benchmark_eligible': False, 'claim_eligible': False,
                'active_profile_switch': False,
            }, sort_keys=True))
            return 0
        result = validate_snapshot(args.snapshot, root=args.root)
        print(json.dumps(result, sort_keys=True))
        return 0
    except (SourceWorktreeError, CandidateError, OSError, ValueError,
            TypeError, json.JSONDecodeError) as error:
        print(json.dumps({'status': 'INVALID', 'error': str(error)}, sort_keys=True))
        return 2


if __name__ == '__main__':
    raise SystemExit(_main())
