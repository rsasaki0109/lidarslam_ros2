#!/usr/bin/env python3
"""Capture a non-promoting competitive execution machine identity.

This is an additive, candidate-only path.  It creates a challenge that an
operator can send to the holder of an externally provisioned Ed25519 host
key, and it verifies the returned detached proof.  The module never creates,
reads, or stores private key material.  The checked-in trust policy contains
no authorized keys, so the production CLI remains ``NOT_READY`` until an
independent custodian supplies and reviews a policy outside this repository.

The challenge and final artifact bind the current r3 candidate, active profile,
the exact seven-field thread policy, campaign/domain, machine-id digest,
public host facts, key descriptor, output basename, validity window, and a
one-shot replay ledger path.  Raw machine identifiers are used only inside
the capture provider and are never serialized or printed.
"""

from __future__ import annotations

import argparse
import base64
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import secrets
import stat
import sys
import time
from typing import Any, Callable, Mapping

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from scripts.competitive_identity_hash import (  # noqa: E402
    canonical_profile_sha256,
    PROFILE_CANONICAL_HASH_KIND,
)
from scripts.validate_competitive_execution_selection_r3 import (  # noqa: E402
    CANDIDATE_REL,
    CandidateError,
    THREAD_KEYS,
    THREAD_VALUES,
    validate_candidate,
)
import yaml  # noqa: E402


ROOT = _ROOT
MODULE_PATH = Path(__file__)
MODULE_RELATIVE = MODULE_PATH.relative_to(ROOT).as_posix()
CANDIDATE_PATH = ROOT / CANDIDATE_REL
PROFILE_REL = 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
PROFILE_PATH = ROOT / PROFILE_REL
TRUST_POLICY_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_machine_identity_pop_trust_policy.json')
TRUST_POLICY_PATH = ROOT / TRUST_POLICY_REL

DOMAIN = 'lidarslam/competitive-execution-r3/machine-identity-pop/ed25519/v1'
CAMPAIGN_ID = 'competitive-execution-selection-2026-08-r3-candidate'
METHOD = 'PROVISIONED_ED25519'
ALGORITHM = 'Ed25519'
BACKEND_NAME = 'python-cryptography'
BACKEND_IMPLEMENTATION = (
    'cryptography.hazmat.primitives.asymmetric.ed25519.Ed25519PublicKey.verify')
PRODUCER_VERSION = '1.0.0'

KEY_SCHEMA = 'competitive_execution_machine_host_public_key_v1'
INPUT_SIDECAR_SCHEMA = 'competitive_execution_machine_identity_pop_input_sidecar_v1'
KEY_SIDECAR_SCHEMA = INPUT_SIDECAR_SCHEMA
CHALLENGE_SCHEMA = 'competitive_execution_machine_identity_pop_challenge_v1'
PROOF_SCHEMA = 'competitive_execution_machine_identity_pop_proof_v1'
ARTIFACT_SCHEMA = 'competitive_execution_machine_identity_pop_v1'
ARTIFACT_SIDECAR_SCHEMA = 'competitive_execution_machine_identity_pop_sidecar_v1'
POLICY_SCHEMA = 'competitive_execution_machine_identity_pop_trust_policy_v1'
DOMAIN_PROOF = DOMAIN + '/challenge'

READONLY_MODE = 0o444
MAX_FILE_BYTES = 2 * 1024 * 1024
MAX_SIDECAR_BYTES = 4096
MAX_KEY_BYTES = 64 * 1024
MAX_LEDGER_BYTES = 16 * 1024
MAX_VALIDITY_SECONDS = 7 * 24 * 60 * 60
MIN_VALIDITY_SECONDS = 60
SHA_RE = re.compile(r'^[0-9a-f]{64}$')
KEY_ID_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$')
CAMPAIGN_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$')
MACHINE_ID_RE = re.compile(r'^[0-9a-f]{32}$')
ISO_UTC_RE = re.compile(r'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$')
PUBLIC_FIELDS = (
    'architecture', 'cpu_model', 'logical_cpu_count', 'memory_total_kb',
    'kernel_release', 'observed_affinity')
RAW_PATHS = {
    'machine_id': Path('/etc/machine-id'),
    'dmi_uuid': Path('/sys/class/dmi/id/product_uuid'),
    'board_serial': Path('/sys/class/dmi/id/board_serial'),
}


class MachinePoPError(ValueError):
    """A malformed, stale, unsafe, or non-promoting machine identity."""


class MachinePoPNotReady(MachinePoPError):
    """The fixed checked-in trust policy is still intentionally NOT_READY."""


def _canonical(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True).encode('utf-8')


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _canonical_hash(value: Any, excluded: str | None = None) -> str:
    if excluded is not None and isinstance(value, Mapping):
        value = {key: item for key, item in value.items() if key != excluded}
    return _sha(_canonical(value))


def _safe_abs(value: Any, label: str) -> Path:
    if not isinstance(value, (str, Path)):
        raise MachinePoPError(f'{label} is not a path')
    path = Path(value)
    if not path.is_absolute() or path.as_posix() != str(path) or '..' in path.parts:
        raise MachinePoPError(f'{label} must be an absolute normalized path')
    if any(part in {'', '.', '..'} for part in path.parts[1:]):
        raise MachinePoPError(f'{label} contains an unsafe component')
    return path


def _parents_no_symlink(path: Path, label: str) -> None:
    current = Path(path.anchor)
    for component in path.parts[1:-1]:
        current /= component
        try:
            info = current.lstat()
        except OSError as error:
            raise MachinePoPError(f'{label} parent is unavailable') from error
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
            raise MachinePoPError(f'{label} parent is not a real directory')


def _file_identity(path: Path, label: str, *, limit: int,
                   readonly: bool) -> dict[str, int]:
    path = _safe_abs(path, label)
    _parents_no_symlink(path, label)
    try:
        info = path.lstat()
    except OSError as error:
        raise MachinePoPError(f'{label} cannot be inspected') from error
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size <= 0 or \
            info.st_size > limit:
        raise MachinePoPError(f'{label} is not a bounded single-link regular file')
    if readonly and stat.S_IMODE(info.st_mode) != READONLY_MODE:
        raise MachinePoPError(f'{label} is not immutable mode 0444')
    return {
        'device': info.st_dev, 'inode': info.st_ino,
        'mode': stat.S_IMODE(info.st_mode), 'nlink': info.st_nlink,
        'size': info.st_size,
    }


def _same_identity(first: Mapping[str, int], second: Mapping[str, int]) -> bool:
    return dict(first) == dict(second)


def _read_bytes(path: Path, label: str, *, limit: int,
                readonly: bool = True) -> tuple[bytes, dict[str, int]]:
    before = _file_identity(path, label, limit=limit, readonly=readonly)
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0)
    try:
        fd = os.open(path, flags)
    except OSError as error:
        raise MachinePoPError(f'{label} cannot be opened without following links') from error
    try:
        opened = os.fstat(fd)
        opened_identity = {
            'device': opened.st_dev, 'inode': opened.st_ino,
            'mode': stat.S_IMODE(opened.st_mode), 'nlink': opened.st_nlink,
            'size': opened.st_size,
        }
        if not _same_identity(before, opened_identity):
            raise MachinePoPError(f'{label} changed before read')
        chunks: list[bytes] = []
        total = 0
        while True:
            chunk = os.read(fd, min(1024 * 1024, limit + 1 - total))
            if not chunk:
                break
            chunks.append(chunk)
            total += len(chunk)
            if total > limit:
                raise MachinePoPError(f'{label} exceeds bounded size')
        closed = os.fstat(fd)
        closed_identity = {
            'device': closed.st_dev, 'inode': closed.st_ino,
            'mode': stat.S_IMODE(closed.st_mode), 'nlink': closed.st_nlink,
            'size': closed.st_size,
        }
        if not _same_identity(before, closed_identity) or total != before['size']:
            raise MachinePoPError(f'{label} changed during read')
        payload = b''.join(chunks)
    except OSError as error:
        raise MachinePoPError(f'{label} read failed') from error
    finally:
        os.close(fd)
    after = _file_identity(path, label, limit=limit, readonly=readonly)
    if not _same_identity(before, after):
        raise MachinePoPError(f'{label} changed after read')
    return payload, after


def _directory_identity(path: Path, label: str) -> dict[str, int]:
    path = _safe_abs(path, label)
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        try:
            info = current.lstat()
        except OSError as error:
            raise MachinePoPError(f'{label} cannot be inspected') from error
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
            raise MachinePoPError(f'{label} is not a real directory')
    info = path.lstat()
    return {'device': info.st_dev, 'inode': info.st_ino,
            'mode': stat.S_IMODE(info.st_mode)}


def _fresh(path: Path, label: str) -> None:
    path = _safe_abs(path, label)
    _parents_no_symlink(path, label)
    if not path.parent.is_dir():
        raise MachinePoPError(f'{label} parent is not a directory')
    try:
        path.lstat()
    except FileNotFoundError:
        return
    except OSError as error:
        raise MachinePoPError(f'{label} cannot be inspected') from error
    raise MachinePoPError(f'{label} must be fresh')


def _assert_directory_identity(path: Path, expected: Mapping[str, int],
                               label: str) -> None:
    current = _directory_identity(path, label)
    if (current.get('device'), current.get('inode')) != (
            expected.get('device'), expected.get('inode')):
        raise MachinePoPError(f'{label} changed during sealing')


def _fsync_dir(path: Path, label: str,
               expected: Mapping[str, int] | None = None) -> None:
    path = _safe_abs(path, label)
    if expected is not None:
        _assert_directory_identity(path, expected, label)
    flags = os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0) | \
        getattr(os, 'O_NOFOLLOW', 0)
    try:
        fd = os.open(path, flags)
        try:
            opened = os.fstat(fd)
            opened_identity = {
                'device': opened.st_dev, 'inode': opened.st_ino,
                'mode': stat.S_IMODE(opened.st_mode),
            }
            if expected is not None and not _same_identity(
                    expected, opened_identity):
                raise MachinePoPError(f'{label} changed before fsync')
            os.fsync(fd)
            closed = os.fstat(fd)
            closed_identity = {
                'device': closed.st_dev, 'inode': closed.st_ino,
                'mode': stat.S_IMODE(closed.st_mode),
            }
            if not _same_identity(opened_identity, closed_identity):
                raise MachinePoPError(f'{label} changed during fsync')
        finally:
            os.close(fd)
        if expected is not None:
            _assert_directory_identity(path, expected, label)
    except OSError as error:
        raise MachinePoPError(f'{label} directory fsync failed') from error


def _write_new(path: Path, payload: bytes, label: str) -> dict[str, int]:
    _fresh(path, label)
    parent = path.parent
    parent_identity = _directory_identity(parent, f'{label} parent')
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0)
    fd = -1
    created: tuple[int, int] | None = None
    try:
        fd = os.open(path, flags, 0o600)
        opened = os.fstat(fd)
        created = (opened.st_dev, opened.st_ino)
        offset = 0
        while offset < len(payload):
            written = os.write(fd, payload[offset:])
            if written <= 0:
                raise MachinePoPError(f'{label} write made no progress')
            offset += written
            _assert_directory_identity(parent, parent_identity,
                                       f'{label} parent after write')
        os.fchmod(fd, READONLY_MODE)
        _assert_directory_identity(parent, parent_identity,
                                   f'{label} parent after mode')
        os.fsync(fd)
        _assert_directory_identity(parent, parent_identity,
                                   f'{label} parent after fsync')
        os.close(fd)
        fd = -1
        _fsync_dir(parent, label, expected=parent_identity)
        identity = _file_identity(
            path, label, limit=len(payload) + 1, readonly=True)
        checked, _ = _read_bytes(
            path, label, limit=len(payload) + 1, readonly=True)
        if checked != payload:
            raise MachinePoPError(f'{label} changed after seal')
        _assert_directory_identity(parent, parent_identity,
                                   f'{label} parent after reopen')
        return identity
    except (OSError, MachinePoPError) as error:
        if fd >= 0:
            try:
                os.close(fd)
            except OSError:
                pass
        if created is not None:
            try:
                _assert_directory_identity(parent, parent_identity,
                                           f'{label} cleanup parent')
                current = path.lstat()
                if (current.st_dev, current.st_ino) == created and current.st_nlink == 1:
                    path.unlink()
                    _fsync_dir(parent, f'{label} cleanup',
                               expected=parent_identity)
            except (FileNotFoundError, OSError, MachinePoPError):
                pass
        if isinstance(error, MachinePoPError):
            raise
        raise MachinePoPError(f'{label} cannot be sealed') from error


def _seal_pair(first: Path, first_data: bytes, second: Path,
               second_data: bytes, label: str) -> None:
    if first.parent != second.parent:
        raise MachinePoPError(f'{label} pair must share one directory')
    _fresh(first, f'{label} artifact')
    _fresh(second, f'{label} sidecar')
    parent = first.parent
    parent_identity = _directory_identity(parent, f'{label} parent')
    created: list[tuple[Path, tuple[int, int]]] = []
    try:
        _write_new(first, first_data, f'{label} artifact')
        _assert_directory_identity(parent, parent_identity,
                                   f'{label} parent after artifact')
        info = first.lstat()
        created.append((first, (info.st_dev, info.st_ino)))
        _write_new(second, second_data, f'{label} sidecar')
        _assert_directory_identity(parent, parent_identity,
                                   f'{label} parent after sidecar')
        info = second.lstat()
        created.append((second, (info.st_dev, info.st_ino)))
        _fsync_dir(parent, label, expected=parent_identity)
    except Exception:
        for path, identity in reversed(created):
            try:
                _assert_directory_identity(parent, parent_identity,
                                           f'{label} cleanup parent')
                info = path.lstat()
                if (info.st_dev, info.st_ino) == identity and info.st_nlink == 1:
                    path.unlink()
            except (FileNotFoundError, OSError, MachinePoPError):
                pass
        try:
            _fsync_dir(parent, f'{label} cleanup', expected=parent_identity)
        except MachinePoPError:
            pass
        raise


def _safe_campaign(value: Any) -> str:
    if not isinstance(value, str) or CAMPAIGN_RE.fullmatch(value) is None:
        raise MachinePoPError('campaign id is invalid')
    return value


def _safe_id(value: Any, label: str) -> str:
    if not isinstance(value, str) or KEY_ID_RE.fullmatch(value) is None:
        raise MachinePoPError(f'{label} is invalid')
    return value


def _sha_field(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise MachinePoPError(f'{label} is not a SHA-256 identity')
    return value


def _safe_leaf(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value in {'.', '..'} or \
            Path(value).name != value or any(
                not (char.isalnum() or char in '._-') for char in value):
        raise MachinePoPError(f'{label} must be a safe basename')
    return value


def _decode_b64(value: Any, label: str, length: int) -> bytes:
    if not isinstance(value, str) or not value or len(value) > 256:
        raise MachinePoPError(f'{label} base64 is invalid')
    try:
        data = base64.b64decode(value.encode('ascii'), validate=True)
    except (ValueError, UnicodeError, base64.binascii.Error) as error:
        raise MachinePoPError(f'{label} base64 is invalid') from error
    if len(data) != length:
        raise MachinePoPError(f'{label} length is invalid')
    return data


def _timestamp(value: Any, label: str) -> str:
    if not isinstance(value, str) or ISO_UTC_RE.fullmatch(value) is None:
        raise MachinePoPError(f'{label} timestamp is invalid')
    try:
        dt.datetime.strptime(value, '%Y-%m-%dT%H:%M:%SZ')
    except ValueError as error:
        raise MachinePoPError(f'{label} timestamp is invalid') from error
    return value


def _read_json(path: Path, label: str, *, schema: str,
               limit: int = MAX_FILE_BYTES,
               readonly: bool = True) -> tuple[dict[str, Any], bytes, dict[str, int]]:
    data, identity = _read_bytes(path, label, limit=limit, readonly=readonly)
    try:
        value = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise MachinePoPError(f'{label} is not valid JSON') from error
    if not isinstance(value, dict) or value.get('schema') != schema or \
            value.get('schema_version') != 1:
        raise MachinePoPError(f'{label} schema is invalid')
    return value, data, identity


def _input_ref(path: Path, value: Mapping[str, Any], data: bytes) -> dict[str, Any]:
    return {
        'schema': value['schema'], 'path': path.name, 'file_sha256': _sha(data),
        'file_bytes': len(data), 'canonical_sha256': value['canonical_sha256'],
        'campaign_domain': value['campaign_domain'],
        'campaign_id': value['campaign_id'],
    }


def _sidecar(path: Path, data: bytes, value: Mapping[str, Any], schema: str) -> dict[str, Any]:
    return {'schema': schema, 'schema_version': 1, 'artifact_path': path.name,
            'artifact_bytes': len(data), 'artifact_sha256': _sha(data),
            'canonical_sha256': value['canonical_sha256']}


def _read_pair(path: Path, *, schema: str, sidecar_schema: str,
               label: str, limit: int = MAX_FILE_BYTES) -> tuple[dict[str, Any], bytes]:
    value, data, _ = _read_json(path, label, schema=schema, limit=limit)
    sidecar_path = path.with_name(path.name + '.sha256.json')
    side, side_data, _ = _read_json(sidecar_path, f'{label} sidecar',
                                    schema=sidecar_schema, limit=MAX_SIDECAR_BYTES)
    if set(side) != {'schema', 'schema_version', 'artifact_path',
                     'artifact_bytes', 'artifact_sha256', 'canonical_sha256'} or \
            side['artifact_path'] != path.name or side['artifact_bytes'] != len(data) or \
            side['artifact_sha256'] != _sha(data) or \
            side['canonical_sha256'] != value.get('canonical_sha256'):
        raise MachinePoPError(f'{label} sidecar binding drift')
    # Reopen both after parsing to close the parse/read TOCTOU window.
    data_again, _ = _read_bytes(path, label, limit=limit, readonly=True)
    side_again, _ = _read_bytes(sidecar_path, f'{label} sidecar',
                                limit=MAX_SIDECAR_BYTES, readonly=True)
    if data_again != data or side_again != side_data:
        raise MachinePoPError(f'{label} changed during validation')
    return value, data


def _read_raw(path: Path) -> str:
    """Read a private stable identifier without ever putting it in an error."""
    try:
        fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
        try:
            data = os.read(fd, 4097)
        finally:
            os.close(fd)
    except OSError:
        return ''
    if not data or len(data) > 4096:
        return ''
    try:
        return data.decode('utf-8').strip()
    except UnicodeDecodeError:
        return ''


def _machine_id_ok(value: Any) -> bool:
    return (isinstance(value, str) and
            MACHINE_ID_RE.fullmatch(value.strip().lower()) is not None and
            len(set(value.strip().lower())) > 1)


def _public(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping) or set(value) != set(PUBLIC_FIELDS):
        raise MachinePoPError('public machine projection shape is invalid')
    for key in ('architecture', 'cpu_model', 'kernel_release'):
        if not isinstance(value[key], str) or not value[key] or len(value[key]) > 512 or \
                any(ord(char) < 32 or ord(char) == 127 for char in value[key]):
            raise MachinePoPError('public machine text projection is invalid')
    if type(value['logical_cpu_count']) is not int or value['logical_cpu_count'] <= 0 or \
            type(value['memory_total_kb']) is not int or value['memory_total_kb'] <= 0:
        raise MachinePoPError('public machine sizing projection is invalid')
    affinity = value['observed_affinity']
    if not isinstance(affinity, list) or not affinity or affinity != sorted(set(affinity)) or \
            any(type(item) is not int or item < 0 for item in affinity):
        raise MachinePoPError('public observed affinity is invalid')
    return dict(value)


def _host_snapshot(provider: Callable[[], Mapping[str, Any]] | None = None) -> dict[str, Any]:
    if provider is not None:
        snapshot = provider()
        if not isinstance(snapshot, Mapping) or set(snapshot) != {'raw_identifiers', 'public'}:
            raise MachinePoPError('machine provider projection is invalid')
        raw = snapshot['raw_identifiers']
        if not isinstance(raw, Mapping) or set(raw) != set(RAW_PATHS):
            raise MachinePoPError('machine provider identifiers are invalid')
        public = _public(snapshot['public'])
        return {'raw_identifiers': dict(raw), 'public': public}
    raw = {label: _read_raw(path) for label, path in RAW_PATHS.items()}
    try:
        affinity = sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        affinity = []
    cpu_model = ''
    try:
        with open('/proc/cpuinfo', encoding='utf-8') as stream:
            for line in stream:
                if line.lower().startswith('model name') and ':' in line:
                    cpu_model = line.split(':', 1)[1].strip()
                    break
    except OSError:
        pass
    memory = 0
    try:
        with open('/proc/meminfo', encoding='utf-8') as stream:
            for line in stream:
                if line.startswith('MemTotal:'):
                    memory = int(line.split()[1])
                    break
    except (OSError, ValueError, IndexError):
        pass
    public = _public({
        'architecture': platform.machine(), 'cpu_model': cpu_model,
        'logical_cpu_count': os.cpu_count() or 0, 'memory_total_kb': memory,
        'kernel_release': platform.release(), 'observed_affinity': affinity,
    })
    return {'raw_identifiers': raw, 'public': public}


def _machine_digest(campaign_id: str, raw_machine_id: str) -> str:
    payload = (DOMAIN + '\0' + campaign_id + '\0machine_id\0' +
               raw_machine_id.strip().lower()).encode('utf-8')
    return _sha(payload)


def _producer() -> dict[str, str]:
    data, _ = _read_bytes(MODULE_PATH, 'machine PoP producer',
                          limit=MAX_FILE_BYTES, readonly=False)
    return {'module': MODULE_RELATIVE, 'source_sha256': _sha(data),
            'version': PRODUCER_VERSION}


def _candidate_snapshot() -> tuple[dict[str, Any], dict[str, Any]]:
    report = validate_candidate(CANDIDATE_PATH)
    if not report.get('structural_valid'):
        raise MachinePoPError('competitive r3 candidate is not structurally valid')
    data, _ = _read_bytes(CANDIDATE_PATH, 'competitive r3 candidate',
                          limit=MAX_FILE_BYTES, readonly=False)
    try:
        candidate = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise MachinePoPError('competitive r3 candidate is not valid JSON') from error
    if not isinstance(candidate, dict) or _sha(data) != report['candidate_file_sha256'] or \
            candidate.get('candidate_identity_sha256') != report['candidate_identity_sha256']:
        raise MachinePoPError('competitive r3 candidate changed during binding')
    if candidate.get('status') != 'NOT_READY' or \
            candidate.get('benchmark_eligible') is not False or \
            candidate.get('claim_eligible') is not False:
        raise MachinePoPError('competitive r3 candidate is not non-promoting')
    candidate_binding = {
        'path': CANDIDATE_REL, 'file_sha256': _sha(data),
        'candidate_identity_sha256': candidate['candidate_identity_sha256'],
        'candidate_id': candidate['candidate_id'], 'status': candidate['status'],
    }
    profile_binding = candidate.get('profile_binding')
    if not isinstance(profile_binding, Mapping) or set(profile_binding) != {
            'path', 'file_sha256', 'canonical_sha256', 'canonical_hash_kind'} or \
            profile_binding['path'] != PROFILE_REL or \
            profile_binding['canonical_hash_kind'] != PROFILE_CANONICAL_HASH_KIND:
        raise MachinePoPError('candidate profile binding is invalid')
    profile_data, _ = _read_bytes(PROFILE_PATH, 'competitive profile',
                                  limit=MAX_FILE_BYTES, readonly=False)
    try:
        profile = yaml.safe_load(profile_data.decode('utf-8'))
    except (UnicodeError, yaml.YAMLError) as error:
        raise MachinePoPError('competitive profile is not valid YAML') from error
    if not isinstance(profile, dict) or _sha(profile_data) != profile_binding['file_sha256'] or \
            canonical_profile_sha256(profile) != profile_binding['canonical_sha256']:
        raise MachinePoPError('competitive profile binding drift')
    thread = candidate.get('thread_policy_binding')
    if not isinstance(thread, Mapping) or set(thread) != {
            'status', 'required_keys', 'values', 'canonical_sha256'} or \
            thread['status'] != 'RECORDED_NOT_READY_EXTERNAL_MACHINE' or \
            thread['required_keys'] != list(THREAD_KEYS) or \
            thread['values'] != THREAD_VALUES or \
            thread['canonical_sha256'] != _canonical_hash(THREAD_VALUES):
        raise MachinePoPError('competitive seven-field thread policy drift')
    profile_contract = profile.get('competitive_slam_profile')
    evidence = (profile_contract.get('evidence_gate_v2')
                if isinstance(profile_contract, Mapping) else None)
    if (not isinstance(evidence, Mapping) or
            evidence.get('thread_policy_required_keys') != list(THREAD_KEYS)):
        raise MachinePoPError('profile thread-policy contract drift')
    return candidate, {
        'candidate_binding': candidate_binding,
        'profile_binding': dict(profile_binding),
        'thread_policy': dict(thread),
        'candidate_file_sha256': _sha(data),
    }


def _validate_key(value: Mapping[str, Any], campaign_id: str) -> bytes:
    required = {'schema', 'schema_version', 'status', 'campaign_domain',
                'campaign_id', 'key_id', 'algorithm', 'public_key_base64',
                'public_key_sha256', 'provisioning_receipt_sha256',
                'valid_from', 'valid_until', 'canonical_sha256'}
    if (set(value) != required or value.get('schema') != KEY_SCHEMA or
            value.get('schema_version') != 1 or
            value.get('status') != 'EXTERNALLY_PROVISIONED' or
            value.get('campaign_domain') != DOMAIN or
            value.get('campaign_id') != campaign_id):
        raise MachinePoPError('host key descriptor shape/campaign is invalid')
    _safe_id(value.get('key_id'), 'host key id')
    if value.get('algorithm') != ALGORITHM:
        raise MachinePoPError('host key algorithm is not Ed25519')
    public = _decode_b64(value.get('public_key_base64'), 'host public key', 32)
    if value.get('public_key_sha256') != _sha(public):
        raise MachinePoPError('host public key SHA drift')
    _sha_field(value.get('provisioning_receipt_sha256'), 'provisioning receipt')
    if not value['provisioning_receipt_sha256'].strip('0'):
        raise MachinePoPError('provisioning receipt identity is empty')
    if type(value.get('valid_from')) is not int or type(value.get('valid_until')) is not int or \
            value['valid_from'] < 0 or value['valid_until'] <= value['valid_from'] or \
            value['valid_until'] - value['valid_from'] > MAX_VALIDITY_SECONDS:
        raise MachinePoPError('host key validity window is invalid')
    if value.get('canonical_sha256') != _canonical_hash(value, 'canonical_sha256'):
        raise MachinePoPError('host key canonical identity drift')
    return public


def _key_ref(path: Path, value: Mapping[str, Any], data: bytes) -> dict[str, Any]:
    return {
        'schema': KEY_SCHEMA, 'path': path.name, 'file_sha256': _sha(data),
        'file_bytes': len(data), 'canonical_sha256': value['canonical_sha256'],
        'campaign_id': value['campaign_id'], 'key_id': value['key_id'],
        'algorithm': value['algorithm'],
        'public_key_sha256': value['public_key_sha256'],
        'provisioning_receipt_sha256': value['provisioning_receipt_sha256'],
        'valid_from': value['valid_from'], 'valid_until': value['valid_until'],
    }


def _backend() -> dict[str, str]:
    try:
        import cryptography
    except ImportError as error:
        raise MachinePoPNotReady('Ed25519 backend is unavailable') from error
    version = getattr(cryptography, '__version__', '')
    if not isinstance(version, str) or not version:
        raise MachinePoPNotReady('Ed25519 backend version is unavailable')
    return {'name': BACKEND_NAME, 'version': version,
            'implementation': BACKEND_IMPLEMENTATION}


def _policy_ref(policy_path: Path, candidate_info: Mapping[str, Any],
                *, allow_not_ready: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    policy_path = _safe_abs(policy_path, 'trust policy')
    value, data, _ = _read_json(
        policy_path, 'trust policy', schema=POLICY_SCHEMA,
        limit=MAX_FILE_BYTES, readonly=policy_path != TRUST_POLICY_PATH)
    required = {'schema', 'schema_version', 'status', 'authorized_runtime',
                'benchmark_eligible', 'campaign_domain', 'campaign_id',
                'candidate_binding', 'profile_binding', 'backend',
                'authorized_keys', 'verifier_source_sha256', 'canonical_sha256'}
    if set(value) != required or value['campaign_domain'] != DOMAIN or \
            value['campaign_id'] != CAMPAIGN_ID or value['benchmark_eligible'] is not False:
        raise MachinePoPError('trust policy shape or campaign is invalid')
    if value['canonical_sha256'] != _canonical_hash(value, 'canonical_sha256'):
        raise MachinePoPError('trust policy canonical identity drift')
    if value['candidate_binding'] != candidate_info['candidate_binding'] or \
            value['profile_binding'] != candidate_info['profile_binding']:
        raise MachinePoPError('trust policy candidate/profile binding drift')
    if policy_path == TRUST_POLICY_PATH and (
            value['status'] != 'NOT_READY' or value['authorized_keys'] != [] or
            value['authorized_runtime'] is not False):
        raise MachinePoPNotReady('checked-in trust policy cannot be promoted in place')
    if policy_path == TRUST_POLICY_PATH and not allow_not_ready:
        raise MachinePoPNotReady('checked-in competitive machine PoP policy is NOT_READY')
    backend = _backend()
    if value['backend'] != backend:
        raise MachinePoPError('trust policy Ed25519 backend drift')
    _sha_field(value['verifier_source_sha256'], 'trust policy verifier source')
    source, _ = _read_bytes(MODULE_PATH, 'machine PoP producer',
                            limit=MAX_FILE_BYTES, readonly=False)
    if value['verifier_source_sha256'] != _sha(source):
        raise MachinePoPError('trust policy verifier source drift')
    keys = value['authorized_keys']
    if (not isinstance(keys, list) or
            len({item.get('key_id') for item in keys
                 if isinstance(item, Mapping)}) != len(keys)):
        raise MachinePoPError('trust policy authorized-key list is invalid')
    for key in keys:
        if not isinstance(key, Mapping) or set(key) != {
                'key_id', 'algorithm', 'public_key_sha256', 'status',
                'valid_from', 'valid_until'} or key['algorithm'] != ALGORITHM or \
                key['status'] != 'ACTIVE':
            raise MachinePoPError('trust policy authorized key is invalid')
        _safe_id(key['key_id'], 'trust policy key id')
        _sha_field(key['public_key_sha256'], 'trust policy public-key hash')
        if type(key['valid_from']) is not int or type(key['valid_until']) is not int or \
                key['valid_until'] <= key['valid_from']:
            raise MachinePoPError('trust policy key validity is invalid')
    ref = {
        'schema': POLICY_SCHEMA, 'path': (
            TRUST_POLICY_REL if policy_path == TRUST_POLICY_PATH else str(policy_path)),
        'file_sha256': _sha(data), 'file_bytes': len(data),
        'canonical_sha256': value['canonical_sha256'], 'status': value['status'],
        'authorized_runtime': value['authorized_runtime'],
        'authorized_keys_sha256': _canonical_hash(keys),
    }
    return value, ref


def _policy_key_matches(policy: Mapping[str, Any], key: Mapping[str, Any], now: int) -> None:
    if policy['status'] != 'READY' or policy['authorized_runtime'] is not True:
        raise MachinePoPNotReady('competitive machine identity trust policy is NOT_READY')
    matches = [item for item in policy['authorized_keys']
               if item['key_id'] == key['key_id'] and
               item['public_key_sha256'] == key['public_key_sha256']]
    if len(matches) != 1:
        raise MachinePoPError('host key is not in the fixed trust allowlist')
    item = matches[0]
    if (not item['valid_from'] <= now < item['valid_until'] or
            not item['valid_from'] <= key['valid_from'] <
            key['valid_until'] <= item['valid_until']):
        raise MachinePoPError('host key validity is outside trusted policy')


def _challenge_payload(value: Mapping[str, Any]) -> bytes:
    unsigned = {key: item for key, item in value.items()
                if key not in {'challenge_payload_sha256', 'canonical_sha256', '_path'}}
    return (DOMAIN_PROOF + '\0').encode('utf-8') + _canonical(unsigned)


def _validate_challenge(value: Mapping[str, Any], candidate_info: Mapping[str, Any]) -> None:
    required = {'schema', 'schema_version', 'status', 'campaign_domain',
                'campaign_id', 'candidate_binding', 'profile_binding',
                'thread_policy', 'machine_id_sha256', 'public', 'host_public_key',
                'trust_policy', 'final_output_name', 'replay_ledger_path',
                'challenge_nonce_base64', 'challenge_nonce_sha256', 'issued_at',
                'expires_at', 'producer', 'challenge_payload_sha256',
                'canonical_sha256'}
    if set(value) != required or value.get('schema') != CHALLENGE_SCHEMA or \
            value.get('schema_version') != 1 or value.get('status') != 'CHALLENGE_PENDING' or \
            value.get('campaign_domain') != DOMAIN or value.get('campaign_id') != CAMPAIGN_ID:
        raise MachinePoPError('machine PoP challenge shape/status is invalid')
    if value['candidate_binding'] != candidate_info['candidate_binding'] or \
            value['profile_binding'] != candidate_info['profile_binding'] or \
            value['thread_policy'] != candidate_info['thread_policy']:
        raise MachinePoPError('machine PoP challenge candidate/profile/thread binding drift')
    _sha_field(value['machine_id_sha256'], 'machine-id digest')
    _public(value['public'])
    _safe_leaf(value['final_output_name'], 'challenge final output')
    ledger = _safe_abs(value['replay_ledger_path'], 'challenge replay ledger')
    _directory_identity(ledger, 'challenge replay ledger')
    _decode_b64(value['challenge_nonce_base64'], 'challenge nonce', 32)
    if value['challenge_nonce_sha256'] != _sha(
            _decode_b64(value['challenge_nonce_base64'], 'challenge nonce', 32)):
        raise MachinePoPError('challenge nonce hash drift')
    if type(value['issued_at']) is not int or type(value['expires_at']) is not int or \
            value['issued_at'] < 0 or not value['issued_at'] < value['expires_at'] or \
            value['expires_at'] - value['issued_at'] > MAX_VALIDITY_SECONDS:
        raise MachinePoPError('challenge validity window is invalid')
    _key_ref_validate(value['host_public_key'], CAMPAIGN_ID)
    _policy_ref_validate(value['trust_policy'])
    producer = value['producer']
    if (not isinstance(producer, Mapping) or
            set(producer) != {'module', 'source_sha256', 'version'} or
            producer['module'] != MODULE_RELATIVE or
            producer['version'] != PRODUCER_VERSION):
        raise MachinePoPError('machine PoP producer descriptor is invalid')
    _sha_field(producer['source_sha256'], 'machine PoP producer source')
    if value['challenge_payload_sha256'] != _sha(_challenge_payload(value)) or \
            value['canonical_sha256'] != _canonical_hash(value, 'canonical_sha256'):
        raise MachinePoPError('machine PoP challenge hash drift')


def _key_ref_validate(value: Any, campaign_id: str) -> None:
    required = {'schema', 'path', 'file_sha256', 'file_bytes', 'canonical_sha256',
                'campaign_id', 'key_id', 'algorithm', 'public_key_sha256',
                'provisioning_receipt_sha256', 'valid_from', 'valid_until'}
    if not isinstance(value, Mapping) or set(value) != required or \
            value['schema'] != KEY_SCHEMA or value['campaign_id'] != campaign_id:
        raise MachinePoPError('host key reference is invalid')
    _safe_leaf(value['path'], 'host key reference path')
    _sha_field(value['file_sha256'], 'host key file')
    _sha_field(value['canonical_sha256'], 'host key canonical')
    _sha_field(value['public_key_sha256'], 'host public key')
    _sha_field(value['provisioning_receipt_sha256'], 'provisioning receipt')
    _safe_id(value['key_id'], 'host key id')
    if value['algorithm'] != ALGORITHM or type(value['file_bytes']) is not int or \
            value['file_bytes'] <= 0 or value['file_bytes'] > MAX_KEY_BYTES or \
            type(value['valid_from']) is not int or type(value['valid_until']) is not int:
        raise MachinePoPError('host key reference values are invalid')


def _policy_ref_validate(value: Any) -> None:
    required = {'schema', 'path', 'file_sha256', 'file_bytes', 'canonical_sha256',
                'status', 'authorized_runtime', 'authorized_keys_sha256'}
    if (not isinstance(value, Mapping) or set(value) != required or
            value['schema'] != POLICY_SCHEMA):
        raise MachinePoPError('trust policy reference is invalid')
    if not isinstance(value['path'], str) or not value['path'] or '\x00' in value['path']:
        raise MachinePoPError('trust policy reference path is invalid')
    _sha_field(value['file_sha256'], 'trust policy file')
    _sha_field(value['canonical_sha256'], 'trust policy canonical')
    _sha_field(value['authorized_keys_sha256'], 'trust policy key projection')
    if (type(value['file_bytes']) is not int or value['file_bytes'] <= 0 or
            value['file_bytes'] > MAX_FILE_BYTES or
            value['status'] not in {'NOT_READY', 'READY'} or
            type(value['authorized_runtime']) is not bool):
        raise MachinePoPError('trust policy reference values are invalid')


def _validate_proof(value: Mapping[str, Any]) -> bytes:
    required = {'schema', 'schema_version', 'status', 'challenge_path',
                'challenge_file_sha256', 'challenge_canonical_sha256',
                'challenge_payload_sha256', 'key_id', 'algorithm',
                'signature_base64', 'signature_sha256', 'canonical_sha256'}
    if set(value) != required or value.get('schema') != PROOF_SCHEMA or \
            value.get('schema_version') != 1 or value.get('status') != 'EXTERNAL_SIGNATURE':
        raise MachinePoPError('machine PoP proof shape/status is invalid')
    _safe_leaf(value['challenge_path'], 'proof challenge path')
    for field, label in (('challenge_file_sha256', 'proof challenge file'),
                         ('challenge_canonical_sha256', 'proof challenge canonical'),
                         ('challenge_payload_sha256', 'proof challenge payload'),
                         ('signature_sha256', 'proof signature')):
        _sha_field(value[field], label)
    _safe_id(value['key_id'], 'proof key id')
    if value['algorithm'] != ALGORITHM:
        raise MachinePoPError('proof algorithm is invalid')
    signature = _decode_b64(value['signature_base64'], 'proof signature', 64)
    if value['signature_sha256'] != _sha(signature) or \
            value['canonical_sha256'] != _canonical_hash(value, 'canonical_sha256'):
        raise MachinePoPError('machine PoP proof hash drift')
    return signature


def _verify(public: bytes, payload: bytes, signature: bytes) -> None:
    try:
        from cryptography.exceptions import InvalidSignature
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
        Ed25519PublicKey.from_public_bytes(public).verify(signature, payload)
    except InvalidSignature as error:
        raise MachinePoPError('machine PoP signature is invalid') from error
    except (ImportError, TypeError, ValueError) as error:
        raise MachinePoPNotReady('Ed25519 verification backend is unavailable') from error


def _claim_nonce(ledger: Path, challenge: Mapping[str, Any],
                 challenge_data: bytes, claimed_at: int) -> tuple[Path, bytes, dict[str, int]]:
    _directory_identity(ledger, 'replay ledger')
    name = challenge['challenge_nonce_sha256'] + '.claim.json'
    claim_path = ledger / name
    _fresh(claim_path, 'nonce claim')
    value: dict[str, Any] = {
        'schema': 'competitive_execution_machine_identity_pop_nonce_claim_v1',
        'schema_version': 1, 'campaign_domain': DOMAIN,
        'campaign_id': CAMPAIGN_ID,
        'challenge_file_sha256': _sha(challenge_data),
        'challenge_payload_sha256': challenge['challenge_payload_sha256'],
        'final_output_name': challenge['final_output_name'],
        'nonce_sha256': challenge['challenge_nonce_sha256'],
        'claimed_at': claimed_at, 'canonical_sha256': '',
    }
    value['canonical_sha256'] = _canonical_hash(value, 'canonical_sha256')
    data = _canonical(value) + b'\n'
    try:
        identity = _write_new(claim_path, data, 'nonce claim')
    except MachinePoPError as error:
        raise MachinePoPError('nonce has already been claimed or claim failed') from error
    return claim_path, data, identity


def _claim_ref(path: Path, value: Mapping[str, Any], data: bytes) -> dict[str, Any]:
    return {'path': path.name, 'file_sha256': _sha(data), 'file_bytes': len(data),
            'canonical_sha256': value['canonical_sha256'],
            'nonce_sha256': value['nonce_sha256']}


def _identity_projection(value: Mapping[str, Any]) -> dict[str, Any]:
    return {
        'campaign_domain': value['campaign_domain'], 'campaign_id': value['campaign_id'],
        'method': value['method'], 'candidate_binding': value['candidate_binding'],
        'profile_binding': value['profile_binding'], 'thread_policy': value['thread_policy'],
        'machine_id_sha256': value['machine_id_sha256'], 'public': value['public'],
        'host_public_key': value['host_public_key'], 'challenge': value['challenge'],
        'proof': value['proof'], 'trust_policy': value['trust_policy'],
        'verified_at': value['verified_at'], 'validity': value['validity'],
        'nonce_claim': value['nonce_claim'],
    }


def _artifact_validate_shape(value: Mapping[str, Any], *, allow_synthetic: bool) -> None:
    required = {'schema', 'schema_version', 'status', 'review_status',
                'benchmark_eligible', 'claim_eligible', 'campaign_domain',
                'campaign_id', 'method', 'candidate_binding', 'profile_binding',
                'thread_policy', 'machine_id_sha256', 'public',
                'host_public_key', 'challenge', 'proof', 'trust_policy',
                'validity', 'verified_at', 'nonce_claim', 'producer',
                'identity_sha256', 'canonical_sha256', 'promotion_policy'}
    if not isinstance(value, Mapping) or set(value) != required or \
            value['schema'] != ARTIFACT_SCHEMA or value['schema_version'] != 1 or \
            value['status'] not in {'LIVE_CAPTURED', 'SYNTHETIC_TEST'} or \
            (value['status'] == 'SYNTHETIC_TEST' and not allow_synthetic) or \
            value['review_status'] != 'NOT_REVIEWED_EXTERNAL' or \
            value['benchmark_eligible'] is not False or value['claim_eligible'] is not False or \
            value['campaign_domain'] != DOMAIN or value['campaign_id'] != CAMPAIGN_ID or \
            value['method'] != METHOD:
        raise MachinePoPError('machine PoP artifact shape/status is invalid')
    _sha_field(value['machine_id_sha256'], 'artifact machine-id digest')
    _public(value['public'])
    thread = value['thread_policy']
    if (not isinstance(value['candidate_binding'], Mapping) or
            value['candidate_binding'].get('status') != 'NOT_READY' or
            not isinstance(value['profile_binding'], Mapping) or
            value['profile_binding'].get('path') != PROFILE_REL or
            not isinstance(thread, Mapping) or
            set(thread) != {'status', 'required_keys', 'values',
                            'canonical_sha256'} or
            thread['status'] != 'RECORDED_NOT_READY_EXTERNAL_MACHINE' or
            thread['required_keys'] != list(THREAD_KEYS) or
            thread['values'] != THREAD_VALUES or
            thread['canonical_sha256'] != _canonical_hash(THREAD_VALUES)):
        raise MachinePoPError('machine PoP artifact binding is invalid')
    for ref in (value['host_public_key'], value['challenge'],
                value['proof'], value['trust_policy']):
        if not isinstance(ref, Mapping):
            raise MachinePoPError('machine PoP artifact source reference is invalid')
    validity = value['validity']
    if not isinstance(validity, Mapping) or set(validity) != {'issued_at', 'expires_at'} or \
            type(validity['issued_at']) is not int or type(validity['expires_at']) is not int or \
            not validity['issued_at'] < validity['expires_at']:
        raise MachinePoPError('machine PoP artifact validity is invalid')
    if type(value['verified_at']) is not int or value['verified_at'] < validity['issued_at'] or \
            value['verified_at'] >= validity['expires_at']:
        raise MachinePoPError('machine PoP artifact verification time is invalid')
    _sha_field(value['identity_sha256'], 'machine PoP artifact identity')
    _sha_field(value['canonical_sha256'], 'machine PoP artifact canonical')
    producer = value['producer']
    if (not isinstance(producer, Mapping) or
            set(producer) != {'module', 'source_sha256', 'version'} or
            producer['module'] != MODULE_RELATIVE or
            producer['version'] != PRODUCER_VERSION):
        raise MachinePoPError('machine PoP artifact producer is invalid')
    _sha_field(producer['source_sha256'], 'machine PoP artifact producer source')
    if value['promotion_policy'] != {
            'benchmark_eligible': False, 'claim_eligible': False,
            'active_profile_switch': False, 'requires_external_review': True,
            'requires_candidate_profile_reseal': True}:
        raise MachinePoPError('machine PoP promotion policy is invalid')
    if value['identity_sha256'] != _canonical_hash(_identity_projection(value)) or \
            value['canonical_sha256'] != _canonical_hash(value, 'canonical_sha256'):
        raise MachinePoPError('machine PoP artifact hash drift')


def _source_ref(path: Path, value: Mapping[str, Any], data: bytes) -> dict[str, Any]:
    return {'schema': value['schema'], 'path': path.name, 'file_sha256': _sha(data),
            'file_bytes': len(data), 'canonical_sha256': value['canonical_sha256']}


def prepare_challenge(output: Path, *, key_descriptor: Path,
                      final_output: Path, replay_ledger: Path,
                      validity_seconds: int = 3600,
                      provider: Callable[[], Mapping[str, Any]] | None = None,
                      policy_path: Path = TRUST_POLICY_PATH,
                      _now: int | None = None) -> dict[str, Any]:
    """Create a fresh challenge; this never requires or handles a private key."""
    output = _safe_abs(output, 'challenge output')
    key_descriptor = _safe_abs(key_descriptor, 'host public-key descriptor')
    final_output = _safe_abs(final_output, 'final machine artifact')
    replay_ledger = _safe_abs(replay_ledger, 'replay ledger')
    if output.parent != key_descriptor.parent or output.parent != final_output.parent:
        raise MachinePoPError('challenge/key/final outputs must share one directory')
    if not isinstance(validity_seconds, int) or isinstance(validity_seconds, bool) or \
            not MIN_VALIDITY_SECONDS <= validity_seconds <= MAX_VALIDITY_SECONDS:
        raise MachinePoPError('challenge validity duration is invalid')
    _directory_identity(replay_ledger, 'replay ledger')
    _fresh(output, 'challenge output')
    _fresh(output.with_name(output.name + '.sha256.json'), 'challenge sidecar')
    _fresh(final_output, 'final machine artifact')
    _fresh(final_output.with_name(final_output.name + '.sha256.json'), 'final sidecar')
    candidate, candidate_info = _candidate_snapshot()
    del candidate
    policy, policy_ref = _policy_ref(
        policy_path, candidate_info, allow_not_ready=policy_path == TRUST_POLICY_PATH)
    del policy
    key_value, key_data = _read_pair(
        key_descriptor, schema=KEY_SCHEMA, sidecar_schema=KEY_SIDECAR_SCHEMA,
        label='host public-key descriptor', limit=MAX_KEY_BYTES)
    public_key = _validate_key(key_value, CAMPAIGN_ID)
    del public_key
    snapshot = _host_snapshot(provider)
    raw_machine_id = snapshot['raw_identifiers'].get('machine_id')
    if not _machine_id_ok(raw_machine_id):
        raise MachinePoPError('machine-id anchor is unavailable')
    now = int(time.time()) if _now is None else _now
    if type(now) is not int or now < 0:
        raise MachinePoPError('challenge clock is invalid')
    challenge: dict[str, Any] = {
        'schema': CHALLENGE_SCHEMA, 'schema_version': 1,
        'status': 'CHALLENGE_PENDING', 'campaign_domain': DOMAIN,
        'campaign_id': CAMPAIGN_ID,
        'candidate_binding': candidate_info['candidate_binding'],
        'profile_binding': candidate_info['profile_binding'],
        'thread_policy': candidate_info['thread_policy'],
        'machine_id_sha256': _machine_digest(CAMPAIGN_ID, raw_machine_id),
        'public': snapshot['public'],
        'host_public_key': _key_ref(key_descriptor, key_value, key_data),
        'trust_policy': policy_ref,
        'final_output_name': final_output.name,
        'replay_ledger_path': str(replay_ledger),
        'challenge_nonce_base64': base64.b64encode(secrets.token_bytes(32)).decode('ascii'),
        'challenge_nonce_sha256': '', 'issued_at': now,
        'expires_at': now + validity_seconds,
        'producer': _producer(), 'challenge_payload_sha256': '',
        'canonical_sha256': '',
    }
    nonce = _decode_b64(challenge['challenge_nonce_base64'], 'challenge nonce', 32)
    challenge['challenge_nonce_sha256'] = _sha(nonce)
    challenge['challenge_payload_sha256'] = _sha(_challenge_payload(challenge))
    challenge['canonical_sha256'] = _canonical_hash(challenge, 'canonical_sha256')
    data = _canonical(challenge) + b'\n'
    sidecar = _sidecar(output, data, challenge, KEY_SIDECAR_SCHEMA)
    _seal_pair(output, data, output.with_name(output.name + '.sha256.json'),
               _canonical(sidecar) + b'\n', 'machine PoP challenge')
    return {'status': 'CHALLENGE_PENDING', 'benchmark_eligible': False,
            'claim_eligible': False, 'challenge_path': str(output),
            'challenge_payload_sha256': challenge['challenge_payload_sha256'],
            'campaign_id': CAMPAIGN_ID, 'proof_required': True,
            'trust_policy_status': policy_ref['status']}


def _load_challenge(path: Path, candidate_info: Mapping[str, Any]) -> tuple[dict[str, Any], bytes]:
    value, data = _read_pair(path, schema=CHALLENGE_SCHEMA,
                             sidecar_schema=KEY_SIDECAR_SCHEMA,
                             label='machine PoP challenge')
    _validate_challenge(value, candidate_info)
    return value, data


def _load_proof(path: Path, challenge: Mapping[str, Any],
                challenge_data: bytes) -> tuple[dict[str, Any], bytes, bytes]:
    value, data = _read_pair(path, schema=PROOF_SCHEMA,
                             sidecar_schema=KEY_SIDECAR_SCHEMA,
                             label='machine PoP proof', limit=MAX_KEY_BYTES)
    signature = _validate_proof(value)
    if value['challenge_path'] != challenge.get('_path') or \
            value['challenge_file_sha256'] != _sha(challenge_data) or \
            value['challenge_canonical_sha256'] != challenge['canonical_sha256'] or \
            value['challenge_payload_sha256'] != challenge['challenge_payload_sha256'] or \
            value['key_id'] != challenge['host_public_key']['key_id']:
        raise MachinePoPError('machine PoP proof/challenge binding drift')
    return value, data, signature


def finalize_challenge(output: Path, *, challenge: Path, proof: Path,
                       provider: Callable[[], Mapping[str, Any]] | None = None,
                       _policy_path: Path | None = None,
                       _allow_test_policy: bool = False,
                       _now: int | None = None,
                       _status: str = 'LIVE_CAPTURED') -> dict[str, Any]:
    """Verify an external proof and seal a live/non-promoting artifact."""
    output = _safe_abs(output, 'final machine artifact')
    challenge = _safe_abs(challenge, 'machine PoP challenge')
    proof = _safe_abs(proof, 'machine PoP proof')
    if output.parent != challenge.parent or output.parent != proof.parent:
        raise MachinePoPError('final/challenge/proof must share one directory')
    if _status not in {'LIVE_CAPTURED', 'SYNTHETIC_TEST'}:
        raise MachinePoPError('machine PoP artifact status is invalid')
    if _status == 'SYNTHETIC_TEST' and not _allow_test_policy:
        raise MachinePoPError('synthetic machine identity is test-only')
    _fresh(output, 'final machine artifact')
    _fresh(output.with_name(output.name + '.sha256.json'), 'final sidecar')
    _, candidate_info = _candidate_snapshot()
    challenge_value, challenge_data = _load_challenge(challenge, candidate_info)
    challenge_value = dict(challenge_value)
    challenge_value['_path'] = challenge.name
    if output.name != challenge_value['final_output_name']:
        raise MachinePoPError('final output is not challenge-bound')
    if proof.parent != challenge.parent:
        raise MachinePoPError('proof must be in challenge directory')
    policy_path = (TRUST_POLICY_PATH if _policy_path is None else
                   _safe_abs(_policy_path, 'trust policy'))
    if _policy_path is not None and not _allow_test_policy:
        raise MachinePoPError('alternate trust policy is test-only')
    policy, policy_ref = _policy_ref(policy_path, candidate_info)
    if policy_ref != challenge_value['trust_policy']:
        raise MachinePoPError('trust policy changed after challenge')
    key_path = challenge.parent / challenge_value['host_public_key']['path']
    key_value, key_data = _read_pair(
        key_path, schema=KEY_SCHEMA, sidecar_schema=KEY_SIDECAR_SCHEMA,
        label='host public-key descriptor', limit=MAX_KEY_BYTES)
    public_key = _validate_key(key_value, CAMPAIGN_ID)
    if _key_ref(key_path, key_value, key_data) != challenge_value['host_public_key']:
        raise MachinePoPError('host public-key source changed after challenge')
    proof_value, proof_data, signature = _load_proof(proof, challenge_value, challenge_data)
    now = int(time.time()) if _now is None else _now
    if (type(now) is not int or now < challenge_value['issued_at'] or
            now >= challenge_value['expires_at']):
        raise MachinePoPError('machine PoP challenge is outside its validity window')
    if not key_value['valid_from'] <= now < key_value['valid_until']:
        raise MachinePoPError('host public-key validity window is expired or not yet active')
    _policy_key_matches(policy, key_value, now)
    _verify(public_key, _challenge_payload(challenge_value), signature)
    snapshot = _host_snapshot(provider)
    raw_machine_id = snapshot['raw_identifiers'].get('machine_id')
    if (not _machine_id_ok(raw_machine_id) or
            _machine_digest(CAMPAIGN_ID, raw_machine_id) !=
            challenge_value['machine_id_sha256'] or
            snapshot['public'] != challenge_value['public']):
        raise MachinePoPError('current host does not match the signed challenge')
    ledger = _safe_abs(challenge_value['replay_ledger_path'], 'replay ledger')
    claim_path, claim_data, _ = _claim_nonce(ledger, challenge_value, challenge_data, now)
    claim_value = json.loads(claim_data.decode('utf-8'))
    challenge_ref = {
        'schema': CHALLENGE_SCHEMA, 'path': challenge.name,
        'file_sha256': _sha(challenge_data), 'file_bytes': len(challenge_data),
        'canonical_sha256': challenge_value['canonical_sha256'],
        'challenge_payload_sha256': challenge_value['challenge_payload_sha256'],
    }
    proof_ref = {
        'schema': PROOF_SCHEMA, 'path': proof.name,
        'file_sha256': _sha(proof_data), 'file_bytes': len(proof_data),
        'canonical_sha256': proof_value['canonical_sha256'],
        'challenge_payload_sha256': proof_value['challenge_payload_sha256'],
        'signature_sha256': proof_value['signature_sha256'],
    }
    artifact: dict[str, Any] = {
        'schema': ARTIFACT_SCHEMA, 'schema_version': 1,
        'status': _status, 'review_status': 'NOT_REVIEWED_EXTERNAL',
        'benchmark_eligible': False, 'claim_eligible': False,
        'campaign_domain': DOMAIN, 'campaign_id': CAMPAIGN_ID,
        'method': METHOD,
        'candidate_binding': candidate_info['candidate_binding'],
        'profile_binding': candidate_info['profile_binding'],
        'thread_policy': candidate_info['thread_policy'],
        'machine_id_sha256': challenge_value['machine_id_sha256'],
        'public': snapshot['public'],
        'host_public_key': challenge_value['host_public_key'],
        'challenge': challenge_ref, 'proof': proof_ref,
        'trust_policy': policy_ref,
        'validity': {'issued_at': challenge_value['issued_at'],
                     'expires_at': challenge_value['expires_at']},
        'verified_at': now,
        'nonce_claim': _claim_ref(claim_path, claim_value, claim_data),
        'producer': _producer(), 'identity_sha256': '', 'canonical_sha256': '',
        'promotion_policy': {
            'benchmark_eligible': False, 'claim_eligible': False,
            'active_profile_switch': False, 'requires_external_review': True,
            'requires_candidate_profile_reseal': True,
        },
    }
    artifact['identity_sha256'] = _canonical_hash(_identity_projection(artifact))
    artifact['canonical_sha256'] = _canonical_hash(artifact, 'canonical_sha256')
    _artifact_validate_shape(artifact, allow_synthetic=_allow_test_policy)
    data = _canonical(artifact) + b'\n'
    sidecar = _sidecar(output, data, artifact, ARTIFACT_SIDECAR_SCHEMA)
    _seal_pair(output, data, output.with_name(output.name + '.sha256.json'),
               _canonical(sidecar) + b'\n', 'machine PoP artifact')
    return {'status': _status, 'review_status': 'NOT_REVIEWED_EXTERNAL',
            'benchmark_eligible': False, 'claim_eligible': False,
            'artifact_path': str(output), 'artifact_sha256': _sha(data),
            'canonical_sha256': artifact['canonical_sha256'],
            'identity_sha256': artifact['identity_sha256'],
            'live_host_match': True, 'trust_policy_status': policy['status'],
            'replay_protected': True, 'promotion_allowed': False}


def validate_challenge(path: Path) -> dict[str, Any]:
    _, info = _candidate_snapshot()
    value, data = _load_challenge(_safe_abs(path, 'machine PoP challenge'), info)
    del data
    return {'status': 'CHALLENGE_PENDING', 'benchmark_eligible': False,
            'claim_eligible': False, 'campaign_id': value['campaign_id'],
            'proof_required': True, 'live_host_match': False,
            'host_match_status': 'HOST_MATCH_NOT_CHECKED',
            'trust_policy_status': value['trust_policy']['status']}


def validate_artifact(path: Path, *, live: bool = False,
                      provider: Callable[[], Mapping[str, Any]] | None = None,
                      _policy_path: Path | None = None,
                      _allow_synthetic: bool = False,
                      _allow_external_policy: bool = False,
                      _now: int | None = None) -> dict[str, Any]:
    """Reopen an artifact; offline mode never claims current-host equality."""
    path = _safe_abs(path, 'machine PoP artifact')
    _, info = _candidate_snapshot()
    value, data = _read_pair(path, schema=ARTIFACT_SCHEMA,
                             sidecar_schema=ARTIFACT_SIDECAR_SCHEMA,
                             label='machine PoP artifact')
    _artifact_validate_shape(value, allow_synthetic=_allow_synthetic)
    if value['candidate_binding'] != info['candidate_binding'] or \
            value['profile_binding'] != info['profile_binding'] or \
            value['thread_policy'] != info['thread_policy']:
        raise MachinePoPError('machine PoP artifact candidate/profile/thread drift')
    policy_path = (TRUST_POLICY_PATH if _policy_path is None else
                   _safe_abs(_policy_path, 'trust policy'))
    if (_policy_path is not None and not _allow_synthetic and
            not _allow_external_policy):
        raise MachinePoPError('alternate trust policy is test-only')
    policy, policy_ref = _policy_ref(policy_path, info)
    if policy_ref != value['trust_policy']:
        raise MachinePoPError('machine PoP artifact trust policy source drift')
    parent = path.parent
    challenge_path = parent / value['challenge']['path']
    proof_path = parent / value['proof']['path']
    key_path = parent / value['host_public_key']['path']
    challenge, challenge_data = _load_challenge(challenge_path, info)
    challenge['_path'] = challenge_path.name
    challenge_ref = {
        'schema': CHALLENGE_SCHEMA, 'path': challenge_path.name,
        'file_sha256': _sha(challenge_data), 'file_bytes': len(challenge_data),
        'canonical_sha256': challenge['canonical_sha256'],
        'challenge_payload_sha256': challenge['challenge_payload_sha256'],
    }
    if challenge_ref != value['challenge']:
        raise MachinePoPError('machine PoP challenge source drift')
    key_value, key_data = _read_pair(
        key_path, schema=KEY_SCHEMA, sidecar_schema=KEY_SIDECAR_SCHEMA,
        label='host public-key descriptor', limit=MAX_KEY_BYTES)
    public_key = _validate_key(key_value, CAMPAIGN_ID)
    if _key_ref(key_path, key_value, key_data) != value['host_public_key']:
        raise MachinePoPError('machine PoP key source drift')
    proof_value, proof_data, signature = _load_proof(proof_path, challenge, challenge_data)
    proof_ref = {
        'schema': PROOF_SCHEMA, 'path': proof_path.name,
        'file_sha256': _sha(proof_data), 'file_bytes': len(proof_data),
        'canonical_sha256': proof_value['canonical_sha256'],
        'challenge_payload_sha256': proof_value['challenge_payload_sha256'],
        'signature_sha256': proof_value['signature_sha256'],
    }
    if proof_ref != value['proof']:
        raise MachinePoPError('machine PoP proof source drift')
    _verify(public_key, _challenge_payload(challenge), signature)
    validity = value['validity']
    if validity != {'issued_at': challenge['issued_at'], 'expires_at': challenge['expires_at']}:
        raise MachinePoPError('machine PoP validity source drift')
    ledger = _safe_abs(challenge['replay_ledger_path'], 'replay ledger')
    claim_path = ledger / (challenge['challenge_nonce_sha256'] + '.claim.json')
    claim, claim_data, _ = _read_json(
        claim_path, 'machine PoP nonce claim',
        schema='competitive_execution_machine_identity_pop_nonce_claim_v1',
        limit=MAX_LEDGER_BYTES)
    if claim['campaign_domain'] != DOMAIN or claim['campaign_id'] != CAMPAIGN_ID or \
            claim['challenge_file_sha256'] != _sha(challenge_data) or \
            claim['challenge_payload_sha256'] != challenge['challenge_payload_sha256'] or \
            claim['final_output_name'] != challenge['final_output_name'] or \
            claim['nonce_sha256'] != challenge['challenge_nonce_sha256'] or \
            value['nonce_claim'] != _claim_ref(claim_path, claim, claim_data):
        raise MachinePoPError('machine PoP nonce claim drift')
    if live:
        now = int(time.time()) if _now is None else _now
        if now < challenge['issued_at'] or now >= challenge['expires_at']:
            raise MachinePoPError('machine PoP artifact is outside current validity window')
        _policy_key_matches(policy, key_value, now)
        snapshot = _host_snapshot(provider)
        raw_machine_id = snapshot['raw_identifiers'].get('machine_id')
        if (not _machine_id_ok(raw_machine_id) or
                _machine_digest(CAMPAIGN_ID, raw_machine_id) !=
                value['machine_id_sha256'] or
                snapshot['public'] != value['public']):
            raise MachinePoPError('machine PoP artifact does not match current host')
        return {'status': 'PASS', 'campaign_id': CAMPAIGN_ID,
                'artifact_sha256': _sha(data), 'identity_sha256': value['identity_sha256'],
                'live_host_match': True, 'host_match_status': 'LIVE_HOST_MATCH_VERIFIED',
                'benchmark_eligible': False, 'claim_eligible': False}
    return {'status': 'PASS', 'campaign_id': CAMPAIGN_ID,
            'artifact_sha256': _sha(data), 'identity_sha256': value['identity_sha256'],
            'live_host_match': False, 'host_match_status': 'HOST_MATCH_NOT_CHECKED',
            'benchmark_eligible': False, 'claim_eligible': False,
            'offline_only': True}


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    prepare = sub.add_parser('prepare-challenge')
    prepare.add_argument('--output', type=Path, required=True)
    prepare.add_argument('--key-descriptor', type=Path, required=True)
    prepare.add_argument('--final-output', type=Path, required=True)
    prepare.add_argument('--replay-ledger', type=Path, required=True)
    prepare.add_argument('--validity-seconds', type=int, default=3600)
    finalize = sub.add_parser('finalize')
    finalize.add_argument('--output', type=Path, required=True)
    finalize.add_argument('--challenge', type=Path, required=True)
    finalize.add_argument('--proof', type=Path, required=True)
    validate_ch = sub.add_parser('validate-challenge')
    validate_ch.add_argument('--challenge', type=Path, required=True)
    validate_art = sub.add_parser('validate-artifact')
    validate_art.add_argument('--artifact', type=Path, required=True)
    validate_art.add_argument('--live', action='store_true')
    args = parser.parse_args()
    if args.command == 'prepare-challenge':
        result = prepare_challenge(
            args.output, key_descriptor=args.key_descriptor,
            final_output=args.final_output, replay_ledger=args.replay_ledger,
            validity_seconds=args.validity_seconds)
    elif args.command == 'finalize':
        result = finalize_challenge(args.output, challenge=args.challenge,
                                    proof=args.proof)
    elif args.command == 'validate-challenge':
        result = validate_challenge(args.challenge)
    else:
        result = validate_artifact(args.artifact, live=args.live)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(_main())
    except MachinePoPNotReady as error:
        print(f'not-ready: {error}', file=sys.stderr)
        raise SystemExit(3)
    except (MachinePoPError, CandidateError, OSError, UnicodeError,
            TypeError, ValueError, yaml.YAMLError) as error:
        print(f'error: {error}', file=sys.stderr)
        raise SystemExit(2)
