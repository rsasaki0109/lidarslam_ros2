#!/usr/bin/env python3
"""Capture and validate a non-secret r3 machine identity artifact.

Raw machine identifiers are read only inside the capture boundary and are
immediately converted to domain-separated SHA-256 values.  They are never
written to the artifact, its sidecar, CLI output, or the offline validator.
The checked-in benchmark candidate does not contain a live capture; callers
must separately review a captured artifact before a policy can become READY.
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
from typing import Any, Callable, Mapping


ROOT = Path(__file__).resolve().parents[5]
MODULE_PATH = Path(__file__)
MODULE_RELATIVE = MODULE_PATH.relative_to(ROOT).as_posix()
SCHEMA = 'glim_clean_room_r3_machine_identity_v1'
SIDECAR_SCHEMA = 'glim_clean_room_r3_machine_identity_sidecar_v1'
DOMAIN = 'lidarslam/glim-clean-room-r3/machine-identity/sha256/v1'
PRODUCER_VERSION = '1.0.0'
MAX_ARTIFACT_BYTES = 256 * 1024
MAX_SIDECAR_BYTES = 4096
CAMPAIGN_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$')
SHA_RE = re.compile(r'^[0-9a-f]{64}$')
MACHINE_ID_RE = re.compile(r'^[0-9a-f]{32}$')
DMI_UUID_RE = re.compile(r'^[0-9A-Fa-f]{8}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{12}$')
IDENTIFIER_FIELDS = ('machine_id_sha256', 'dmi_uuid_sha256', 'board_serial_sha256')
MIN_STABLE_IDENTIFIERS = 2
PUBLIC_FIELDS = (
    'architecture', 'cpu_model', 'logical_cpu_count', 'memory_total_kb',
    'kernel_release', 'toolchain',
)
TOOLCHAIN_FIELDS = ('python_version', 'python_implementation', 'python_compiler')
RAW_IDENTIFIER_PATHS = {
    'machine_id': Path('/etc/machine-id'),
    'dmi_uuid': Path('/sys/class/dmi/id/product_uuid'),
    'board_serial': Path('/sys/class/dmi/id/board_serial'),
}

# A DMI-less host may use this additive method.  It never weakens the v1
# contract: v1 still requires two distinct stable identifiers.  The keyed
# method requires the machine-id plus a public key that was provisioned by an
# external custodian and a detached proof made by the holder of its private
# key.  This module never accepts, reads, or creates private-key material.
KEYED_SCHEMA = 'glim_clean_room_r3_machine_identity_keyed_v1'
KEYED_SIDECAR_SCHEMA = 'glim_clean_room_r3_machine_identity_keyed_sidecar_v1'
KEY_DESCRIPTOR_SCHEMA = 'glim_clean_room_r3_machine_host_public_key_v1'
KEY_INPUT_SIDECAR_SCHEMA = 'glim_clean_room_r3_machine_key_input_sidecar_v1'
KEY_CHALLENGE_SCHEMA = 'glim_clean_room_r3_machine_key_challenge_v1'
KEY_PROOF_SCHEMA = 'glim_clean_room_r3_machine_key_proof_v1'
KEY_DOMAIN = 'lidarslam/glim-clean-room-r3/machine-identity-keyed/sha256/v1'
KEY_PROOF_DOMAIN = 'lidarslam/glim-clean-room-r3/machine-identity-keyed/proof/ed25519/v1'
KEY_METHOD = 'PROVISIONED_ED25519'
KEY_ALGORITHM = 'Ed25519'
KEY_ID_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$')
MAX_KEY_BYTES = 16 * 1024
MAX_KEY_INPUT_BYTES = 64 * 1024
MAX_KEY_NONCE_BYTES = 32
KEYED_PROJECTION_FIELDS = (
    'method', 'key_id', 'public_key_sha256', 'challenge_payload_sha256',
    'proof_signature_sha256',
)
CANDIDATE_PATH = ROOT / 'docker/benchmark_adapters/glim_clean_room/phase3d/r3/apt_allowlist_candidate.json'
CANDIDATE_RELATIVE = CANDIDATE_PATH.relative_to(ROOT).as_posix()


class MachineIdentityError(ValueError):
    """Raised when a machine identity artifact is unsafe or inconsistent."""


def _meaningful_identifier(value: Any, label: str) -> bool:
    if not isinstance(value, str) or not value or len(value) > 512 or \
            any(ord(char) < 32 or ord(char) == 127 for char in value):
        return False
    normalized = value.strip().lower()
    if normalized in {
            'unknown', 'none', 'null', 'n/a', 'not specified', 'default',
            'not applicable', 'not available', 'not_available',
            'to be filled by o.e.m.', 'to be filled by oem', 'default string',
            'system serial number', 'serial number', 'none provided',
            'unknown serial number'}:
        return False
    compact = normalized.replace('-', '')
    if len(set(compact)) <= 1:
        return False
    if label == 'machine_id' and MACHINE_ID_RE.fullmatch(normalized) is None:
        return False
    if label == 'dmi_uuid' and DMI_UUID_RE.fullmatch(value.strip()) is None:
        return False
    return True


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True).encode('utf-8')


def canonical_hash(value: Any, excluded: str | None = None) -> str:
    if excluded is not None and isinstance(value, Mapping):
        value = {str(key): item for key, item in value.items() if key != excluded}
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _safe_campaign(value: Any) -> str:
    if not isinstance(value, str) or CAMPAIGN_RE.fullmatch(value) is None:
        raise MachineIdentityError('campaign_id is not a safe precommitted identifier')
    return value


def _safe_abs(path: Any, label: str) -> Path:
    if not isinstance(path, str) or not path.startswith('/') or \
            Path(path).as_posix() != path or any(part in {'', '.', '..'}
                                                 for part in path[1:].split('/')):
        raise MachineIdentityError(f'{label} must be an absolute normalized path')
    return Path(path)


def _parent_no_symlink(path: Path, label: str) -> None:
    parent = Path(path).parent
    current = Path(parent.anchor)
    for component in parent.parts[1:]:
        current /= component
        try:
            info = current.lstat()
        except OSError as error:
            raise MachineIdentityError(f'{label} parent is unavailable: {error}') from error
        if stat.S_ISLNK(info.st_mode):
            raise MachineIdentityError(f'{label} parent contains a symlink')


def _read_regular(path: Path, label: str, *, limit: int,
                  allow_empty: bool = False) -> bytes:
    try:
        before = path.lstat()
    except OSError as error:
        raise MachineIdentityError(f'{label} cannot be inspected: {error}') from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or \
            before.st_nlink != 1 or before.st_size > limit or \
            (before.st_size == 0 and not allow_empty):
        raise MachineIdentityError(f'{label} is not a bounded single-link regular file')
    try:
        with path.open('rb') as stream:
            fd_before = os.fstat(stream.fileno())
            data = stream.read(limit + 1)
            fd_after = os.fstat(stream.fileno())
    except OSError as error:
        raise MachineIdentityError(f'{label} cannot be read: {error}') from error
    if (fd_before.st_dev, fd_before.st_ino) != (before.st_dev, before.st_ino) or \
            (fd_after.st_dev, fd_after.st_ino) != (before.st_dev, before.st_ino) or \
            fd_after.st_size != before.st_size or len(data) != before.st_size or \
            len(data) > limit or (not data and not allow_empty):
        raise MachineIdentityError(f'{label} changed while being read')
    return data


def _write_exclusive(path: Path, data: bytes, label: str, *, mode: int = 0o444) -> None:
    if path.exists() or path.is_symlink() or path.parent.is_symlink() or \
            not path.parent.is_dir():
        raise MachineIdentityError(f'{label} is not a fresh output')
    created: tuple[int, int] | None = None
    try:
        with path.open('xb') as stream:
            created_info = os.fstat(stream.fileno())
            created = (created_info.st_dev, created_info.st_ino)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        path.chmod(mode)
    except OSError as error:
        if created is not None:
            try:
                current = path.lstat()
                if (current.st_dev, current.st_ino) == created:
                    path.unlink()
            except (FileNotFoundError, OSError):
                pass
        raise MachineIdentityError(f'{label} cannot be sealed: {error}') from error


def _fresh_file(path: Path, label: str) -> None:
    _parent_no_symlink(path, label)
    try:
        info = path.lstat()
    except FileNotFoundError:
        return
    except OSError as error:
        raise MachineIdentityError(f'{label} cannot be inspected: {error}') from error
    raise MachineIdentityError(f'{label} must be fresh (mode={stat.S_IFMT(info.st_mode):o})')


def _seal_pair(artifact_path: Path, artifact_data: bytes,
               sidecar_path: Path, sidecar_data: bytes) -> None:
    """Seal two files as one best-effort transaction without broad cleanup."""
    _fresh_file(artifact_path, 'machine identity artifact')
    _fresh_file(sidecar_path, 'machine identity sidecar')
    parent = artifact_path.parent
    try:
        parent_before = parent.lstat()
    except OSError as error:
        raise MachineIdentityError(f'machine identity output parent unavailable: {error}') from error
    created: list[tuple[Path, tuple[int, int]]] = []
    try:
        _write_exclusive(artifact_path, artifact_data, 'machine identity artifact')
        artifact_info = artifact_path.lstat()
        created.append((artifact_path, (artifact_info.st_dev, artifact_info.st_ino)))
        parent_now = parent.lstat()
        if (parent_now.st_dev, parent_now.st_ino) != (parent_before.st_dev, parent_before.st_ino):
            raise MachineIdentityError('machine identity output parent changed during seal')
        _write_exclusive(sidecar_path, sidecar_data, 'machine identity sidecar')
        sidecar_info = sidecar_path.lstat()
        created.append((sidecar_path, (sidecar_info.st_dev, sidecar_info.st_ino)))
        parent_now = parent.lstat()
        if (parent_now.st_dev, parent_now.st_ino) != (parent_before.st_dev, parent_before.st_ino):
            raise MachineIdentityError('machine identity output parent changed during seal')
    except Exception:
        for path, identity in reversed(created):
            try:
                current = path.lstat()
                if (current.st_dev, current.st_ino) == identity:
                    path.unlink()
            except FileNotFoundError:
                pass
            except OSError:
                # A cleanup failure is intentionally visible to the caller by
                # leaving the exact path for forensic inspection; no broad
                # recursive cleanup is attempted.
                pass
        raise


def _domain_digest(campaign_id: str, label: str, raw: str) -> str:
    payload = (DOMAIN + '\0' + campaign_id + '\0' + label + '\0' + raw).encode('utf-8')
    return _sha(payload)


def _read_host_text(path: Path) -> str:
    try:
        data = _read_regular(path, f'raw identifier {path}', limit=4096, allow_empty=True)
    except MachineIdentityError:
        return ''
    try:
        return data.decode('utf-8').strip()
    except UnicodeDecodeError:
        return ''


def _read_proc_text(path: Path, label: str, limit: int) -> str:
    """Read a bounded proc pseudo-file without relying on st_size."""
    try:
        with path.open('rb') as stream:
            data = stream.read(limit + 1)
    except OSError:
        return ''
    if not data or len(data) > limit:
        return ''
    try:
        return data.decode('utf-8')
    except UnicodeDecodeError:
        return ''


def _read_cpu_model() -> str:
    try:
        data = _read_proc_text(Path('/proc/cpuinfo'), '/proc/cpuinfo', 2 * 1024 * 1024)
    except OSError:
        data = ''
    for line in data.splitlines():
        if line.lower().startswith('model name') and ':' in line:
            return line.split(':', 1)[1].strip()
    return ''


def _read_memory_kb() -> int | None:
    data = _read_proc_text(Path('/proc/meminfo'), '/proc/meminfo', 256 * 1024)
    if not data:
        return None
    for line in data.splitlines():
        if line.startswith('MemTotal:'):
            try:
                value = int(line.split()[1])
            except (IndexError, ValueError):
                return None
            return value if value > 0 else None
    return None


def _host_snapshot() -> dict[str, Any]:
    raw = {label: _read_host_text(path) for label, path in RAW_IDENTIFIER_PATHS.items()}
    public = {
        'architecture': platform.machine(),
        'cpu_model': _read_cpu_model(),
        'logical_cpu_count': os.cpu_count(),
        'memory_total_kb': _read_memory_kb(),
        'kernel_release': platform.release(),
        'toolchain': {
            'python_version': platform.python_version(),
            'python_implementation': platform.python_implementation(),
            'python_compiler': platform.python_compiler(),
        },
    }
    return {'raw_identifiers': raw, 'public': public}


def _identity_projection(artifact: Mapping[str, Any]) -> dict[str, Any]:
    return {
        'campaign_domain': artifact['campaign_domain'],
        'campaign_id': artifact['campaign_id'],
        'identifier_digests': artifact['identifier_digests'],
        'public': artifact['public'],
    }


def _producer() -> dict[str, str]:
    source = _read_regular(MODULE_PATH, 'machine identity producer',
                           limit=2 * 1024 * 1024, allow_empty=False)
    return {'module': MODULE_RELATIVE, 'source_sha256': _sha(source),
            'version': PRODUCER_VERSION}


def build_artifact(*, campaign_id: str, snapshot: Mapping[str, Any],
                   captured_at: str, status: str = 'SYNTHETIC_TEST') -> dict[str, Any]:
    """Build an artifact from a provider snapshot without serializing raw IDs.

    ``snapshot`` is an internal provider boundary.  It is intentionally not
    accepted by the CLI; tests may inject it to avoid reading host identity.
    """
    campaign_id = _safe_campaign(campaign_id)
    if status not in {'LIVE_CAPTURED', 'SYNTHETIC_TEST'}:
        raise MachineIdentityError('machine identity status is invalid')
    if not isinstance(snapshot, Mapping) or set(snapshot) != {'raw_identifiers', 'public'}:
        raise MachineIdentityError('machine snapshot shape is invalid')
    raw = snapshot['raw_identifiers']
    public = snapshot['public']
    if not isinstance(raw, Mapping) or set(raw) != {
            'machine_id', 'dmi_uuid', 'board_serial'}:
        raise MachineIdentityError('raw identifier provider shape is invalid')
    if any(raw[label] and not _meaningful_identifier(raw[label], label)
           for label in ('machine_id', 'dmi_uuid', 'board_serial')):
        raise MachineIdentityError('machine identity contains an invalid stable identifier')
    meaningful = [raw[label].strip().lower()
                  for label in ('machine_id', 'dmi_uuid', 'board_serial')
                  if _meaningful_identifier(raw[label], label)]
    if len(meaningful) < MIN_STABLE_IDENTIFIERS or \
            len(set(meaningful)) < MIN_STABLE_IDENTIFIERS:
        raise MachineIdentityError(
            'machine identity has fewer than two distinct stable identifiers')
    if not isinstance(public, Mapping) or set(public) != set(PUBLIC_FIELDS):
        raise MachineIdentityError('public machine projection shape is invalid')
    if not isinstance(public['toolchain'], Mapping) or set(public['toolchain']) != set(TOOLCHAIN_FIELDS):
        raise MachineIdentityError('toolchain projection shape is invalid')
    identifier_digests = {
        f'{label}_sha256': _domain_digest(campaign_id, label, raw[label])
        for label in ('machine_id', 'dmi_uuid', 'board_serial')
    }
    artifact: dict[str, Any] = {
        'schema': SCHEMA, 'schema_version': 1, 'status': status,
        'campaign_domain': DOMAIN, 'campaign_id': campaign_id,
        'identifier_digests': identifier_digests,
        'public': dict(public),
        'identity_sha256': '',
        'captured_at': captured_at,
        'producer': _producer(),
        'canonical_sha256': '',
    }
    artifact['identity_sha256'] = canonical_hash(_identity_projection(artifact))
    artifact['canonical_sha256'] = canonical_hash(artifact, 'canonical_sha256')
    return artifact


def _validate_timestamp(value: Any) -> None:
    if not isinstance(value, str) or not value.endswith('Z'):
        raise MachineIdentityError('captured_at must be an RFC3339 UTC timestamp')
    try:
        dt.datetime.fromisoformat(value[:-1] + '+00:00')
    except ValueError as error:
        raise MachineIdentityError('captured_at is not a valid timestamp') from error


def validate_artifact(artifact: Mapping[str, Any], *, allow_synthetic: bool = True) -> dict[str, Any]:
    required = {'schema', 'schema_version', 'status', 'campaign_domain', 'campaign_id',
                'identifier_digests', 'public', 'identity_sha256', 'captured_at',
                'producer', 'canonical_sha256'}
    if not isinstance(artifact, Mapping) or set(artifact) != required or \
            artifact.get('schema') != SCHEMA or artifact.get('schema_version') != 1:
        raise MachineIdentityError('machine identity artifact shape is invalid')
    if artifact['status'] not in {'LIVE_CAPTURED', 'SYNTHETIC_TEST'} or \
            (not allow_synthetic and artifact['status'] != 'LIVE_CAPTURED'):
        raise MachineIdentityError('machine identity artifact status is not acceptable')
    if artifact['campaign_domain'] != DOMAIN:
        raise MachineIdentityError('machine identity campaign domain drift')
    _safe_campaign(artifact['campaign_id'])
    ids = artifact['identifier_digests']
    if not isinstance(ids, Mapping) or set(ids) != set(IDENTIFIER_FIELDS) or \
            any(not isinstance(ids[key], str) or SHA_RE.fullmatch(ids[key]) is None
                for key in IDENTIFIER_FIELDS):
        raise MachineIdentityError('machine identity digest projection is invalid')
    public = artifact['public']
    if not isinstance(public, Mapping) or set(public) != set(PUBLIC_FIELDS):
        raise MachineIdentityError('machine identity public projection is invalid')
    for key in ('architecture', 'cpu_model', 'kernel_release'):
        if not isinstance(public[key], str) or not public[key] or len(public[key]) > 512:
            raise MachineIdentityError(f'public field {key} is invalid')
    if type(public['logical_cpu_count']) is not int or public['logical_cpu_count'] <= 0 or \
            type(public['memory_total_kb']) is not int or public['memory_total_kb'] <= 0:
        raise MachineIdentityError('public machine sizing fields are invalid')
    toolchain = public['toolchain']
    if not isinstance(toolchain, Mapping) or set(toolchain) != set(TOOLCHAIN_FIELDS) or \
            any(not isinstance(toolchain[key], str) or not toolchain[key] or len(toolchain[key]) > 256
                for key in TOOLCHAIN_FIELDS):
        raise MachineIdentityError('toolchain projection is invalid')
    if not isinstance(artifact['identity_sha256'], str) or \
            artifact['identity_sha256'] != canonical_hash(_identity_projection(artifact)):
        raise MachineIdentityError('machine identity projection hash drift')
    _validate_timestamp(artifact['captured_at'])
    producer = artifact['producer']
    if not isinstance(producer, Mapping) or set(producer) != {'module', 'source_sha256', 'version'} or \
            producer['module'] != MODULE_RELATIVE or producer['version'] != PRODUCER_VERSION or \
            SHA_RE.fullmatch(producer['source_sha256']) is None:
        raise MachineIdentityError('machine identity producer is invalid')
    actual_source = _sha(_read_regular(MODULE_PATH, 'machine identity producer',
                                       limit=2 * 1024 * 1024, allow_empty=False))
    if producer['source_sha256'] != actual_source:
        raise MachineIdentityError('machine identity producer source drift')
    if artifact['canonical_sha256'] != canonical_hash(artifact, 'canonical_sha256'):
        raise MachineIdentityError('machine identity canonical hash drift')
    return {'status': 'PASS', 'campaign_domain': DOMAIN,
            'campaign_id': artifact['campaign_id'],
            'identity_sha256': artifact['identity_sha256'],
            'offline_host_match_claimed': False}


def _sidecar(artifact_path: Path, data: bytes, artifact: Mapping[str, Any]) -> dict[str, Any]:
    return {'schema': SIDECAR_SCHEMA, 'schema_version': 1,
            'artifact_path': artifact_path.name, 'artifact_bytes': len(data),
            'artifact_sha256': _sha(data),
            'canonical_sha256': artifact['canonical_sha256']}


def capture_artifact(output: Path, *, campaign_id: str,
                     provider: Callable[[], Mapping[str, Any]] | None = None,
                     captured_at: str | None = None,
                     status: str = 'LIVE_CAPTURED') -> dict[str, Any]:
    output = _safe_abs(str(output), 'machine identity output')
    _parent_no_symlink(output, 'machine identity output')
    sidecar_path = output.with_name(output.name + '.sha256.json')
    _fresh_file(output, 'machine identity artifact')
    _fresh_file(sidecar_path, 'machine identity sidecar')
    snapshot = _host_snapshot() if provider is None else provider()
    timestamp = captured_at or dt.datetime.now(dt.timezone.utc).isoformat().replace('+00:00', 'Z')
    artifact = build_artifact(campaign_id=campaign_id, snapshot=snapshot,
                              captured_at=timestamp, status=status)
    validate_artifact(artifact, allow_synthetic=status == 'SYNTHETIC_TEST')
    data = canonical_bytes(artifact) + b'\n'
    sidecar = _sidecar(output, data, artifact)
    sidecar_data = canonical_bytes(sidecar) + b'\n'
    _seal_pair(output, data, sidecar_path, sidecar_data)
    return {'status': 'PASS', 'artifact_path': str(output),
            'sidecar_path': str(sidecar_path), 'artifact_sha256': _sha(data),
            'artifact_bytes': len(data), 'canonical_sha256': artifact['canonical_sha256'],
            'campaign_id': campaign_id, 'identity_sha256': artifact['identity_sha256'],
            'live_host_match': status == 'LIVE_CAPTURED'}


def validate_file(path: Path, *, live: bool = False,
                 provider: Callable[[], Mapping[str, Any]] | None = None) -> dict[str, Any]:
    path = _safe_abs(str(path), 'machine identity artifact')
    _parent_no_symlink(path, 'machine identity artifact')
    sidecar_path = path.with_name(path.name + '.sha256.json')
    for immutable_path, label in ((path, 'machine identity artifact'),
                                  (sidecar_path, 'machine identity sidecar')):
        try:
            immutable_info = immutable_path.lstat()
        except OSError as error:
            raise MachineIdentityError(f'{label} cannot be inspected: {error}') from error
        if immutable_info.st_mode & 0o222:
            raise MachineIdentityError(f'{label} must be read-only sealed evidence')
    data = _read_regular(path, 'machine identity artifact', limit=MAX_ARTIFACT_BYTES)
    try:
        artifact = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise MachineIdentityError(f'machine identity artifact is invalid JSON: {error}') from error
    result = validate_artifact(artifact, allow_synthetic=False if live else True)
    side_data = _read_regular(sidecar_path, 'machine identity sidecar',
                              limit=MAX_SIDECAR_BYTES)
    try:
        sidecar = json.loads(side_data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise MachineIdentityError(f'machine identity sidecar is invalid JSON: {error}') from error
    if not isinstance(sidecar, Mapping) or set(sidecar) != {
            'schema', 'schema_version', 'artifact_path', 'artifact_bytes',
            'artifact_sha256', 'canonical_sha256'} or \
            sidecar['schema'] != SIDECAR_SCHEMA or sidecar['schema_version'] != 1 or \
            sidecar['artifact_path'] != path.name or sidecar['artifact_bytes'] != len(data) or \
            sidecar['artifact_sha256'] != _sha(data) or \
            sidecar['canonical_sha256'] != artifact['canonical_sha256']:
        raise MachineIdentityError('machine identity sidecar binding drift')
    result.update({'artifact_path': str(path), 'artifact_sha256': _sha(data),
                   'artifact_bytes': len(data), 'canonical_sha256': artifact['canonical_sha256'],
                   'sidecar_path': str(sidecar_path)})
    if live:
        current = build_artifact(
            campaign_id=artifact['campaign_id'],
            snapshot=_host_snapshot() if provider is None else provider(),
            captured_at=artifact['captured_at'], status='LIVE_CAPTURED')
        if current['identity_sha256'] != artifact['identity_sha256'] or \
                current['public'] != artifact['public']:
            raise MachineIdentityError('machine identity does not match current host projection')
        result['live_host_match'] = True
        result['host_match_status'] = 'LIVE_HOST_MATCH_VERIFIED'
    else:
        result['live_host_match'] = False
        result['host_match_status'] = 'HOST_MATCH_NOT_CHECKED'
    return result


def _key_safe_relative(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value.startswith('/') or \
            '\\' in value or Path(value).as_posix() != value or \
            any(part in {'', '.', '..'} for part in value.split('/')):
        raise MachineIdentityError(f'{label} is not a safe relative path')
    return value


def _key_safe_id(value: Any, label: str) -> str:
    if not isinstance(value, str) or KEY_ID_RE.fullmatch(value) is None:
        raise MachineIdentityError(f'{label} is not a safe key identifier')
    return value


def _key_sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise MachineIdentityError(f'{label} is not a SHA-256 identity')
    return value


def _key_decode(value: Any, label: str, size: int) -> bytes:
    if not isinstance(value, str) or not value or len(value) > 256:
        raise MachineIdentityError(f'{label} is not bounded base64')
    try:
        decoded = base64.b64decode(value.encode('ascii'), validate=True)
    except (ValueError, UnicodeError, base64.binascii.Error) as error:
        raise MachineIdentityError(f'{label} is not valid base64') from error
    if len(decoded) != size:
        raise MachineIdentityError(f'{label} has an invalid length')
    return decoded


def _key_public(public: Any) -> dict[str, Any]:
    if not isinstance(public, Mapping) or set(public) != set(PUBLIC_FIELDS):
        raise MachineIdentityError('keyed machine public projection shape is invalid')
    for key in ('architecture', 'cpu_model', 'kernel_release'):
        if not isinstance(public[key], str) or not public[key] or len(public[key]) > 512 or \
                any(ord(char) < 32 or ord(char) == 127 for char in public[key]):
            raise MachineIdentityError(f'keyed public field {key} is invalid')
    if type(public['logical_cpu_count']) is not int or public['logical_cpu_count'] <= 0 or \
            type(public['memory_total_kb']) is not int or public['memory_total_kb'] <= 0:
        raise MachineIdentityError('keyed public sizing fields are invalid')
    toolchain = public['toolchain']
    if not isinstance(toolchain, Mapping) or set(toolchain) != set(TOOLCHAIN_FIELDS) or \
            any(not isinstance(toolchain[key], str) or not toolchain[key] or len(toolchain[key]) > 256
                for key in TOOLCHAIN_FIELDS):
        raise MachineIdentityError('keyed public toolchain projection is invalid')
    return dict(public)


def _key_parent(path: Path, label: str) -> None:
    _parent_no_symlink(path, label)


def _key_input(path: Path, label: str) -> tuple[dict[str, Any], bytes]:
    path = _safe_abs(str(path), label)
    _key_parent(path, label)
    data = _read_regular(path, label, limit=MAX_KEY_INPUT_BYTES)
    try:
        value = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise MachineIdentityError(f'{label} is not valid JSON') from error
    if not isinstance(value, Mapping) or 'canonical_sha256' not in value or \
            value['canonical_sha256'] != canonical_hash(value, 'canonical_sha256'):
        raise MachineIdentityError(f'{label} canonical identity is invalid')
    sidecar_path = path.with_name(path.name + '.sha256.json')
    sidecar_data = _read_regular(sidecar_path, f'{label} sidecar', limit=MAX_SIDECAR_BYTES)
    try:
        sidecar = json.loads(sidecar_data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise MachineIdentityError(f'{label} sidecar is not valid JSON') from error
    if not isinstance(sidecar, Mapping) or set(sidecar) != {
            'schema', 'schema_version', 'artifact_path', 'artifact_bytes',
            'artifact_sha256', 'canonical_sha256'} or \
            sidecar['schema'] != KEY_INPUT_SIDECAR_SCHEMA or \
            sidecar['schema_version'] != 1 or sidecar['artifact_path'] != path.name or \
            sidecar['artifact_bytes'] != len(data) or sidecar['artifact_sha256'] != _sha(data) or \
            sidecar['canonical_sha256'] != value['canonical_sha256']:
        raise MachineIdentityError(f'{label} sidecar binding drift')
    for immutable_path, immutable_label in ((path, label), (sidecar_path, f'{label} sidecar')):
        info = immutable_path.lstat()
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_mode & 0o222:
            raise MachineIdentityError(f'{immutable_label} is not immutable single-link evidence')
    if _read_regular(path, f'{label} recheck', limit=MAX_KEY_INPUT_BYTES) != data or \
            _read_regular(sidecar_path, f'{label} sidecar recheck', limit=MAX_SIDECAR_BYTES) != sidecar_data:
        raise MachineIdentityError(f'{label} changed during validation')
    return dict(value), data


def _key_sidecar(path: Path, data: bytes, value: Mapping[str, Any], *, schema: str) -> dict[str, Any]:
    return {'schema': schema, 'schema_version': 1, 'artifact_path': path.name,
            'artifact_bytes': len(data), 'artifact_sha256': _sha(data),
            'canonical_sha256': value['canonical_sha256']}


def _key_ref(path: Path, value: Mapping[str, Any], data: bytes) -> dict[str, Any]:
    return {'schema': value['schema'], 'path': path.name, 'file_sha256': _sha(data),
            'file_bytes': len(data), 'canonical_sha256': value['canonical_sha256'],
            'campaign_id': value['campaign_id'], 'key_id': value['key_id'],
            'algorithm': value['algorithm'], 'public_key_sha256': value['public_key_sha256'],
            'provisioning_receipt_sha256': value['provisioning_receipt_sha256']}


def _key_validate_descriptor(value: Mapping[str, Any], campaign_id: str) -> tuple[bytes, dict[str, Any]]:
    required = {'schema', 'schema_version', 'status', 'campaign_domain', 'campaign_id',
                'key_id', 'algorithm', 'public_key_base64', 'public_key_sha256',
                'provisioning_receipt_sha256', 'canonical_sha256'}
    if set(value) != required or value.get('schema') != KEY_DESCRIPTOR_SCHEMA or \
            value.get('schema_version') != 1 or value.get('status') != 'EXTERNALLY_PROVISIONED' or \
            value.get('campaign_domain') != KEY_DOMAIN or value.get('campaign_id') != campaign_id:
        raise MachineIdentityError('external host public-key descriptor is malformed')
    _safe_campaign(campaign_id)
    key_id = _key_safe_id(value.get('key_id'), 'host public key id')
    if value.get('algorithm') != KEY_ALGORITHM:
        raise MachineIdentityError('host public key algorithm is not Ed25519')
    public_key = _key_decode(value.get('public_key_base64'), 'host public key', 32)
    if _key_sha(value.get('public_key_sha256'), 'host public key') != _sha(public_key):
        raise MachineIdentityError('host public key hash drift')
    receipt = _key_sha(value.get('provisioning_receipt_sha256'), 'provisioning receipt')
    if not receipt.strip('0'):
        raise MachineIdentityError('provisioning receipt identity is empty')
    if value.get('canonical_sha256') != canonical_hash(value, 'canonical_sha256'):
        raise MachineIdentityError('host public key canonical identity drift')
    return public_key, {'schema': KEY_DESCRIPTOR_SCHEMA, 'key_id': key_id,
                        'algorithm': KEY_ALGORITHM,
                        'public_key_sha256': value['public_key_sha256'],
                        'provisioning_receipt_sha256': receipt,
                        'campaign_id': campaign_id}


def _key_snapshot(provider: Callable[[], Mapping[str, Any]] | None) -> tuple[str, dict[str, Any]]:
    snapshot = _host_snapshot() if provider is None else provider()
    if not isinstance(snapshot, Mapping) or set(snapshot) != {'raw_identifiers', 'public'}:
        raise MachineIdentityError('keyed machine provider projection is malformed')
    raw = snapshot['raw_identifiers']
    if not isinstance(raw, Mapping) or set(raw) != {'machine_id', 'dmi_uuid', 'board_serial'} or \
            not _meaningful_identifier(raw.get('machine_id'), 'machine_id'):
        raise MachineIdentityError('keyed machine-id anchor is unavailable')
    return raw['machine_id'], _key_public(snapshot['public'])


def _key_machine_digest(campaign_id: str, raw_machine_id: str) -> str:
    return _domain_digest(campaign_id, 'machine_id', raw_machine_id)


def _key_candidate_ref() -> dict[str, Any]:
    _key_parent(CANDIDATE_PATH, 'r3 candidate manifest')
    data = _read_regular(CANDIDATE_PATH, 'r3 candidate manifest', limit=MAX_KEY_INPUT_BYTES)
    try:
        candidate = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise MachineIdentityError('r3 candidate manifest is invalid JSON') from error
    if not isinstance(candidate, Mapping) or candidate.get('status') != 'OPT_IN_NOT_READY' or \
            candidate.get('benchmark_eligible') is not False or \
            candidate.get('canonical_sha256') != canonical_hash(candidate, 'canonical_sha256'):
        raise MachineIdentityError('r3 candidate manifest is not a fixed non-promoting identity')
    tool_hashes = candidate.get('tool_hashes')
    if not isinstance(tool_hashes, Mapping) or \
            tool_hashes.get(MODULE_RELATIVE) != _sha(_read_regular(
                MODULE_PATH, 'keyed machine identity producer', limit=2 * 1024 * 1024)):
        raise MachineIdentityError('r3 candidate machine producer binding drift')
    return {'path': CANDIDATE_RELATIVE, 'file_sha256': _sha(data),
            'file_bytes': len(data), 'canonical_sha256': candidate['canonical_sha256'],
            'candidate_id': candidate['candidate_id'], 'revision': candidate['revision']}


def _key_producer() -> dict[str, str]:
    _key_parent(MODULE_PATH, 'keyed machine identity producer')
    source = _read_regular(MODULE_PATH, 'keyed machine identity producer',
                           limit=2 * 1024 * 1024)
    return {'module': MODULE_RELATIVE, 'source_sha256': _sha(source),
            'version': PRODUCER_VERSION}


def _key_validate_producer(value: Any) -> None:
    if not isinstance(value, Mapping) or set(value) != {'module', 'source_sha256', 'version'} or \
            value.get('module') != MODULE_RELATIVE or value.get('version') != PRODUCER_VERSION or \
            SHA_RE.fullmatch(value.get('source_sha256', '')) is None or \
            value['source_sha256'] != _key_producer()['source_sha256']:
        raise MachineIdentityError('keyed machine identity producer source drift')


def _key_challenge_payload(value: Mapping[str, Any]) -> bytes:
    unsigned = {key: item for key, item in value.items()
                if key not in {'challenge_payload_sha256', 'canonical_sha256'}}
    return (KEY_PROOF_DOMAIN + '\0challenge\0').encode('utf-8') + canonical_bytes(unsigned)


def _key_validate_candidate_ref(value: Any) -> None:
    if not isinstance(value, Mapping) or set(value) != {
            'path', 'file_sha256', 'file_bytes', 'canonical_sha256',
            'candidate_id', 'revision'}:
        raise MachineIdentityError('keyed candidate binding is malformed')
    if value.get('path') != CANDIDATE_RELATIVE:
        raise MachineIdentityError('keyed candidate path drift')
    _key_sha(value.get('file_sha256'), 'keyed candidate file')
    _key_sha(value.get('canonical_sha256'), 'keyed candidate canonical')
    if type(value.get('file_bytes')) is not int or value['file_bytes'] <= 0 or \
            value['file_bytes'] > MAX_KEY_INPUT_BYTES or \
            not isinstance(value.get('candidate_id'), str) or not value['candidate_id'] or \
            type(value.get('revision')) is not int or value['revision'] < 1:
        raise MachineIdentityError('keyed candidate identity is invalid')


def _key_validate_challenge(value: Mapping[str, Any]) -> None:
    required = {'schema', 'schema_version', 'status', 'campaign_domain', 'campaign_id',
                'machine_id_sha256', 'public', 'candidate', 'key', 'final_output_path',
                'challenge_nonce_base64',
                'producer', 'challenge_payload_sha256', 'canonical_sha256'}
    if set(value) != required or value.get('schema') != KEY_CHALLENGE_SCHEMA or \
            value.get('schema_version') != 1 or value.get('status') != 'CHALLENGE_PENDING' or \
            value.get('campaign_domain') != KEY_DOMAIN:
        raise MachineIdentityError('keyed challenge shape is invalid')
    _safe_campaign(value.get('campaign_id'))
    _key_safe_relative(value.get('final_output_path'), 'challenge final output path')
    _key_validate_candidate_ref(value.get('candidate'))
    _key_sha(value.get('machine_id_sha256'), 'challenge machine-id digest')
    _key_public(value.get('public'))
    key = value['key']
    if not isinstance(key, Mapping) or set(key) != {
            'schema', 'path', 'file_sha256', 'file_bytes', 'canonical_sha256',
            'campaign_id', 'key_id', 'algorithm', 'public_key_sha256',
            'provisioning_receipt_sha256'} or key['schema'] != KEY_DESCRIPTOR_SCHEMA:
        raise MachineIdentityError('keyed challenge key binding is malformed')
    _key_safe_relative(key['path'], 'challenge key path')
    _key_sha(key['file_sha256'], 'challenge key file')
    if type(key['file_bytes']) is not int or key['file_bytes'] <= 0 or key['file_bytes'] > MAX_KEY_INPUT_BYTES:
        raise MachineIdentityError('challenge key size is invalid')
    _key_sha(key['canonical_sha256'], 'challenge key canonical')
    if key['campaign_id'] != value['campaign_id']:
        raise MachineIdentityError('challenge key campaign drift')
    _key_safe_id(key['key_id'], 'challenge key id')
    if key['algorithm'] != KEY_ALGORITHM:
        raise MachineIdentityError('challenge key algorithm drift')
    _key_sha(key['public_key_sha256'], 'challenge key hash')
    _key_sha(key['provisioning_receipt_sha256'], 'challenge provisioning receipt')
    nonce = _key_decode(value.get('challenge_nonce_base64'), 'challenge nonce', MAX_KEY_NONCE_BYTES)
    if len(nonce) != MAX_KEY_NONCE_BYTES:
        raise MachineIdentityError('challenge nonce length is invalid')
    _key_validate_producer(value.get('producer'))
    expected_payload = _sha(_key_challenge_payload(value))
    if value.get('challenge_payload_sha256') != expected_payload or \
            value.get('canonical_sha256') != canonical_hash(value, 'canonical_sha256'):
        raise MachineIdentityError('keyed challenge identity drift')


def _key_validate_proof(value: Mapping[str, Any]) -> bytes:
    required = {'schema', 'schema_version', 'status', 'challenge_path',
                'challenge_file_sha256', 'challenge_canonical_sha256',
                'challenge_payload_sha256', 'key_id', 'algorithm',
                'signature_base64', 'signature_sha256', 'canonical_sha256'}
    if set(value) != required or value.get('schema') != KEY_PROOF_SCHEMA or \
            value.get('schema_version') != 1 or value.get('status') != 'EXTERNAL_SIGNATURE':
        raise MachineIdentityError('keyed proof shape is invalid')
    _key_safe_relative(value.get('challenge_path'), 'proof challenge path')
    for field, label in (('challenge_file_sha256', 'proof challenge file'),
                         ('challenge_canonical_sha256', 'proof challenge canonical'),
                         ('challenge_payload_sha256', 'proof challenge payload')):
        _key_sha(value.get(field), label)
    _key_safe_id(value.get('key_id'), 'proof key id')
    if value.get('algorithm') != KEY_ALGORITHM:
        raise MachineIdentityError('proof algorithm is not Ed25519')
    signature = _key_decode(value.get('signature_base64'), 'proof signature', 64)
    if _key_sha(value.get('signature_sha256'), 'proof signature') != _sha(signature) or \
            value.get('canonical_sha256') != canonical_hash(value, 'canonical_sha256'):
        raise MachineIdentityError('keyed proof identity drift')
    return signature


def _key_verify(public_key: bytes, payload: bytes, signature: bytes,
                verifier: Callable[[bytes, bytes, bytes], bool] | None = None) -> None:
    if verifier is not None:
        try:
            accepted = verifier(public_key, payload, signature)
        except Exception as error:
            raise MachineIdentityError('injected proof verifier failed') from error
        if accepted is not True:
            raise MachineIdentityError('proof-of-possession was rejected')
        return
    try:
        from cryptography.exceptions import InvalidSignature
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
        Ed25519PublicKey.from_public_bytes(public_key).verify(signature, payload)
    except InvalidSignature as error:
        raise MachineIdentityError('proof-of-possession signature is invalid') from error
    except (ImportError, TypeError, ValueError) as error:
        raise MachineIdentityError('Ed25519 proof verification is unavailable') from error


def _key_identity_projection(value: Mapping[str, Any]) -> dict[str, Any]:
    return {
        'campaign_domain': value['campaign_domain'], 'campaign_id': value['campaign_id'],
        'method': value['method'], 'machine_id_sha256': value['machine_id_sha256'],
        'candidate': value['candidate'],
        'public_key_sha256': value['host_public_key']['public_key_sha256'],
        'challenge_payload_sha256': value['challenge']['challenge_payload_sha256'],
        'proof_signature_sha256': value['proof']['signature_sha256'],
        'public': value['public'],
    }


def _key_validate_artifact(value: Mapping[str, Any], *, allow_synthetic: bool = True) -> None:
    required = {'schema', 'schema_version', 'status', 'method', 'campaign_domain',
                'campaign_id', 'candidate', 'machine_id_sha256', 'host_public_key', 'challenge',
                'proof', 'public', 'identity_sha256', 'captured_at', 'producer',
                'canonical_sha256'}
    if not isinstance(value, Mapping) or set(value) != required or \
            value.get('schema') != KEYED_SCHEMA or value.get('schema_version') != 1 or \
            value.get('method') != KEY_METHOD or value.get('campaign_domain') != KEY_DOMAIN or \
            value.get('status') not in {'LIVE_CAPTURED', 'SYNTHETIC_TEST'} or \
            (not allow_synthetic and value.get('status') != 'LIVE_CAPTURED'):
        raise MachineIdentityError('keyed machine identity artifact shape/status is invalid')
    _safe_campaign(value.get('campaign_id'))
    _key_validate_candidate_ref(value.get('candidate'))
    _key_sha(value.get('machine_id_sha256'), 'keyed machine-id digest')
    _key_public(value.get('public'))
    key = value['host_public_key']
    key_required = {'schema', 'path', 'file_sha256', 'file_bytes', 'canonical_sha256',
                    'campaign_id', 'key_id', 'algorithm', 'public_key_sha256',
                    'provisioning_receipt_sha256'}
    if not isinstance(key, Mapping) or set(key) != key_required or key['schema'] != KEY_DESCRIPTOR_SCHEMA:
        raise MachineIdentityError('keyed host public-key binding is malformed')
    _key_safe_relative(key['path'], 'keyed host public-key path')
    _key_sha(key['file_sha256'], 'keyed host public-key file')
    _key_sha(key['canonical_sha256'], 'keyed host public-key canonical')
    _key_sha(key['public_key_sha256'], 'keyed host public-key hash')
    _key_sha(key['provisioning_receipt_sha256'], 'keyed provisioning receipt')
    if key['campaign_id'] != value['campaign_id'] or key['algorithm'] != KEY_ALGORITHM:
        raise MachineIdentityError('keyed host public-key campaign/algorithm drift')
    if type(key['file_bytes']) is not int or key['file_bytes'] <= 0 or key['file_bytes'] > MAX_KEY_INPUT_BYTES:
        raise MachineIdentityError('keyed host public-key size is invalid')
    challenge = value['challenge']
    challenge_required = {'schema', 'path', 'file_sha256', 'file_bytes', 'canonical_sha256',
                          'challenge_payload_sha256'}
    if not isinstance(challenge, Mapping) or set(challenge) != challenge_required or \
            challenge['schema'] != KEY_CHALLENGE_SCHEMA:
        raise MachineIdentityError('keyed challenge source binding is malformed')
    _key_safe_relative(challenge['path'], 'keyed challenge path')
    _key_sha(challenge['file_sha256'], 'keyed challenge file')
    _key_sha(challenge['canonical_sha256'], 'keyed challenge canonical')
    _key_sha(challenge['challenge_payload_sha256'], 'keyed challenge payload')
    if type(challenge['file_bytes']) is not int or challenge['file_bytes'] <= 0 or challenge['file_bytes'] > MAX_KEY_INPUT_BYTES:
        raise MachineIdentityError('keyed challenge size is invalid')
    proof = value['proof']
    proof_required = {'schema', 'path', 'file_sha256', 'file_bytes', 'canonical_sha256',
                      'challenge_payload_sha256', 'signature_sha256'}
    if not isinstance(proof, Mapping) or set(proof) != proof_required or proof['schema'] != KEY_PROOF_SCHEMA:
        raise MachineIdentityError('keyed proof source binding is malformed')
    _key_safe_relative(proof['path'], 'keyed proof path')
    _key_sha(proof['file_sha256'], 'keyed proof file')
    _key_sha(proof['canonical_sha256'], 'keyed proof canonical')
    _key_sha(proof['challenge_payload_sha256'], 'keyed proof payload')
    _key_sha(proof['signature_sha256'], 'keyed proof signature')
    if type(proof['file_bytes']) is not int or proof['file_bytes'] <= 0 or proof['file_bytes'] > MAX_KEY_INPUT_BYTES:
        raise MachineIdentityError('keyed proof size is invalid')
    _validate_timestamp(value['captured_at'])
    _key_validate_producer(value['producer'])
    if value.get('identity_sha256') != canonical_hash(_key_identity_projection(value)) or \
            value.get('canonical_sha256') != canonical_hash(value, 'canonical_sha256'):
        raise MachineIdentityError('keyed machine identity hash drift')


def prepare_key_challenge(output: Path, *, key_descriptor: Path, campaign_id: str,
                          final_output: Path,
                          provider: Callable[[], Mapping[str, Any]] | None = None) -> dict[str, Any]:
    """Create a challenge without accessing or generating any private key."""
    output = _safe_abs(str(output), 'keyed challenge output')
    key_descriptor = _safe_abs(str(key_descriptor), 'host public-key descriptor')
    final_output = _safe_abs(str(final_output), 'keyed final output')
    if output.parent != key_descriptor.parent or output.parent != final_output.parent:
        raise MachineIdentityError('keyed challenge inputs must share one immutable directory')
    if final_output.name in {output.name, key_descriptor.name}:
        raise MachineIdentityError('keyed final output must be distinct from challenge/key inputs')
    _fresh_file(final_output, 'keyed final output')
    _fresh_file(final_output.with_name(final_output.name + '.sha256.json'),
                'keyed final output sidecar')
    _fresh_file(output, 'keyed challenge output')
    _fresh_file(output.with_name(output.name + '.sha256.json'), 'keyed challenge sidecar')
    key_value, key_data = _key_input(key_descriptor, 'host public-key descriptor')
    public_key, key_projection = _key_validate_descriptor(key_value, campaign_id)
    del public_key
    raw_machine_id, public = _key_snapshot(provider)
    challenge: dict[str, Any] = {
        'schema': KEY_CHALLENGE_SCHEMA, 'schema_version': 1,
        'status': 'CHALLENGE_PENDING', 'campaign_domain': KEY_DOMAIN,
        'campaign_id': campaign_id,
        'machine_id_sha256': _key_machine_digest(campaign_id, raw_machine_id),
        'public': public, 'candidate': _key_candidate_ref(),
        'key': _key_ref(key_descriptor, key_value, key_data),
        'final_output_path': final_output.name,
        'challenge_nonce_base64': base64.b64encode(secrets.token_bytes(MAX_KEY_NONCE_BYTES)).decode('ascii'),
        'producer': _key_producer(), 'challenge_payload_sha256': '', 'canonical_sha256': '',
    }
    # The key projection is recomputed so a descriptor with an unexpected field
    # cannot be smuggled into a challenge even if its JSON was rehashed.
    if challenge['key']['key_id'] != key_projection['key_id'] or \
            challenge['key']['public_key_sha256'] != key_projection['public_key_sha256']:
        raise MachineIdentityError('host public-key descriptor projection drift')
    challenge['challenge_payload_sha256'] = _sha(_key_challenge_payload(challenge))
    challenge['canonical_sha256'] = canonical_hash(challenge, 'canonical_sha256')
    data = canonical_bytes(challenge) + b'\n'
    sidecar = _key_sidecar(output, data, challenge, schema=KEY_INPUT_SIDECAR_SCHEMA)
    _seal_pair(output, data, output.with_name(output.name + '.sha256.json'),
               canonical_bytes(sidecar) + b'\n')
    return {'status': 'PASS', 'challenge_path': str(output),
            'challenge_payload_sha256': challenge['challenge_payload_sha256'],
            'campaign_id': campaign_id, 'machine_id_sha256': challenge['machine_id_sha256']}


def finalize_keyed_artifact(output: Path, *, challenge: Path, proof: Path,
                           captured_at: str, status: str = 'LIVE_CAPTURED',
                           provider: Callable[[], Mapping[str, Any]] | None = None,
                           verifier: Callable[[bytes, bytes, bytes], bool] | None = None) -> dict[str, Any]:
    """Finalize an externally signed proof; this function never signs."""
    if status not in {'LIVE_CAPTURED', 'SYNTHETIC_TEST'}:
        raise MachineIdentityError('keyed final status is invalid')
    output = _safe_abs(str(output), 'keyed machine identity output')
    challenge = _safe_abs(str(challenge), 'keyed challenge input')
    proof = _safe_abs(str(proof), 'keyed proof input')
    if output.parent != challenge.parent or output.parent != proof.parent:
        raise MachineIdentityError('keyed final inputs must share one immutable directory')
    if status == 'LIVE_CAPTURED' and verifier is not None:
        raise MachineIdentityError('injected proof verifiers are test-only')
    _fresh_file(output, 'keyed machine identity output')
    _fresh_file(output.with_name(output.name + '.sha256.json'), 'keyed machine identity sidecar')
    challenge_value, challenge_data = _key_input(challenge, 'keyed challenge')
    _key_validate_challenge(challenge_value)
    if output.name != challenge_value['final_output_path']:
        raise MachineIdentityError('keyed final output is not bound by the challenge')
    if _key_candidate_ref() != challenge_value['candidate']:
        raise MachineIdentityError('keyed challenge candidate binding drift')
    key_path = challenge.parent / _key_safe_relative(challenge_value['key']['path'], 'challenge key path')
    key_value, key_data = _key_input(key_path, 'host public-key descriptor')
    public_key, key_projection = _key_validate_descriptor(key_value, challenge_value['campaign_id'])
    if _key_ref(key_path, key_value, key_data) != challenge_value['key']:
        raise MachineIdentityError('keyed challenge host public-key binding drift')
    proof_value, proof_data = _key_input(proof, 'keyed proof')
    signature = _key_validate_proof(proof_value)
    if proof_value['challenge_path'] != challenge.name or \
            proof_value['challenge_file_sha256'] != _sha(challenge_data) or \
            proof_value['challenge_canonical_sha256'] != challenge_value['canonical_sha256'] or \
            proof_value['challenge_payload_sha256'] != challenge_value['challenge_payload_sha256'] or \
            proof_value['key_id'] != key_projection['key_id']:
        raise MachineIdentityError('keyed proof challenge/key binding drift')
    raw_machine_id, public = _key_snapshot(provider)
    if _key_machine_digest(challenge_value['campaign_id'], raw_machine_id) != challenge_value['machine_id_sha256'] or \
            public != challenge_value['public']:
        raise MachineIdentityError('keyed challenge does not match current host projection')
    _key_verify(public_key, _key_challenge_payload(challenge_value), signature, verifier)
    challenge_ref = {'schema': KEY_CHALLENGE_SCHEMA, 'path': challenge.name,
                     'file_sha256': _sha(challenge_data), 'file_bytes': len(challenge_data),
                     'canonical_sha256': challenge_value['canonical_sha256'],
                     'challenge_payload_sha256': challenge_value['challenge_payload_sha256']}
    proof_ref = {'schema': KEY_PROOF_SCHEMA, 'path': proof.name,
                 'file_sha256': _sha(proof_data), 'file_bytes': len(proof_data),
                 'canonical_sha256': proof_value['canonical_sha256'],
                 'challenge_payload_sha256': proof_value['challenge_payload_sha256'],
                 'signature_sha256': proof_value['signature_sha256']}
    artifact: dict[str, Any] = {
        'schema': KEYED_SCHEMA, 'schema_version': 1, 'status': status,
        'method': KEY_METHOD, 'campaign_domain': KEY_DOMAIN,
        'campaign_id': challenge_value['campaign_id'],
        'candidate': challenge_value['candidate'],
        'machine_id_sha256': challenge_value['machine_id_sha256'],
        'host_public_key': _key_ref(key_path, key_value, key_data),
        'challenge': challenge_ref, 'proof': proof_ref, 'public': public,
        'identity_sha256': '', 'captured_at': captured_at,
        'producer': _key_producer(), 'canonical_sha256': '',
    }
    artifact['identity_sha256'] = canonical_hash(_key_identity_projection(artifact))
    artifact['canonical_sha256'] = canonical_hash(artifact, 'canonical_sha256')
    _key_validate_artifact(artifact, allow_synthetic=status == 'SYNTHETIC_TEST')
    data = canonical_bytes(artifact) + b'\n'
    sidecar = _key_sidecar(output, data, artifact, schema=KEYED_SIDECAR_SCHEMA)
    _seal_pair(output, data, output.with_name(output.name + '.sha256.json'),
               canonical_bytes(sidecar) + b'\n')
    return {'status': 'PASS', 'artifact_path': str(output),
            'sidecar_path': str(output.with_name(output.name + '.sha256.json')),
            'artifact_sha256': _sha(data), 'canonical_sha256': artifact['canonical_sha256'],
            'identity_sha256': artifact['identity_sha256'], 'campaign_id': artifact['campaign_id'],
            'method': KEY_METHOD, 'live_host_match': status == 'LIVE_CAPTURED'}


def validate_keyed_file(path: Path, *, live: bool = False,
                        provider: Callable[[], Mapping[str, Any]] | None = None,
                        verifier: Callable[[bytes, bytes, bytes], bool] | None = None) -> dict[str, Any]:
    path = _safe_abs(str(path), 'keyed machine identity artifact')
    _key_parent(path, 'keyed machine identity artifact')
    sidecar_path = path.with_name(path.name + '.sha256.json')
    for immutable_path, label in ((path, 'keyed machine identity artifact'),
                                  (sidecar_path, 'keyed machine identity sidecar')):
        info = immutable_path.lstat()
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_mode & 0o222:
            raise MachineIdentityError(f'{label} is not immutable single-link evidence')
    data = _read_regular(path, 'keyed machine identity artifact', limit=MAX_ARTIFACT_BYTES)
    try:
        artifact = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise MachineIdentityError('keyed machine identity artifact is invalid JSON') from error
    _key_validate_artifact(artifact, allow_synthetic=not live)
    if _key_candidate_ref() != artifact['candidate']:
        raise MachineIdentityError('keyed artifact candidate binding drift')
    side_data = _read_regular(sidecar_path, 'keyed machine identity sidecar', limit=MAX_SIDECAR_BYTES)
    try:
        sidecar = json.loads(side_data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise MachineIdentityError('keyed machine identity sidecar is invalid JSON') from error
    if not isinstance(sidecar, Mapping) or set(sidecar) != {
            'schema', 'schema_version', 'artifact_path', 'artifact_bytes',
            'artifact_sha256', 'canonical_sha256'} or sidecar['schema'] != KEYED_SIDECAR_SCHEMA or \
            sidecar['schema_version'] != 1 or sidecar['artifact_path'] != path.name or \
            sidecar['artifact_bytes'] != len(data) or sidecar['artifact_sha256'] != _sha(data) or \
            sidecar['canonical_sha256'] != artifact['canonical_sha256']:
        raise MachineIdentityError('keyed machine identity sidecar binding drift')
    if _read_regular(path, 'keyed machine identity artifact recheck', limit=MAX_ARTIFACT_BYTES) != data or \
            _read_regular(sidecar_path, 'keyed machine identity sidecar recheck', limit=MAX_SIDECAR_BYTES) != side_data:
        raise MachineIdentityError('keyed machine identity artifact changed during validation')
    parent = path.parent
    key_path = parent / _key_safe_relative(artifact['host_public_key']['path'], 'keyed host key path')
    challenge_path = parent / _key_safe_relative(artifact['challenge']['path'], 'keyed challenge path')
    proof_path = parent / _key_safe_relative(artifact['proof']['path'], 'keyed proof path')
    key_value, key_data = _key_input(key_path, 'host public-key descriptor')
    public_key, key_projection = _key_validate_descriptor(key_value, artifact['campaign_id'])
    if _key_ref(key_path, key_value, key_data) != artifact['host_public_key']:
        raise MachineIdentityError('keyed host public-key source drift')
    challenge_value, challenge_data = _key_input(challenge_path, 'keyed challenge')
    _key_validate_challenge(challenge_value)
    if {'schema': KEY_CHALLENGE_SCHEMA, 'path': challenge_path.name,
        'file_sha256': _sha(challenge_data), 'file_bytes': len(challenge_data),
        'canonical_sha256': challenge_value['canonical_sha256'],
        'challenge_payload_sha256': challenge_value['challenge_payload_sha256']} != artifact['challenge']:
        raise MachineIdentityError('keyed challenge source drift')
    proof_value, proof_data = _key_input(proof_path, 'keyed proof')
    signature = _key_validate_proof(proof_value)
    if {'schema': KEY_PROOF_SCHEMA, 'path': proof_path.name,
        'file_sha256': _sha(proof_data), 'file_bytes': len(proof_data),
        'canonical_sha256': proof_value['canonical_sha256'],
        'challenge_payload_sha256': proof_value['challenge_payload_sha256'],
        'signature_sha256': proof_value['signature_sha256']} != artifact['proof']:
        raise MachineIdentityError('keyed proof source drift')
    if proof_value['challenge_path'] != challenge_path.name or \
            proof_value['challenge_file_sha256'] != _sha(challenge_data) or \
            proof_value['challenge_canonical_sha256'] != challenge_value['canonical_sha256'] or \
            proof_value['challenge_payload_sha256'] != challenge_value['challenge_payload_sha256'] or \
            proof_value['key_id'] != key_projection['key_id'] or \
            challenge_value['campaign_id'] != artifact['campaign_id'] or \
            challenge_value['machine_id_sha256'] != artifact['machine_id_sha256'] or \
            challenge_value['public'] != artifact['public']:
        raise MachineIdentityError('keyed challenge/proof/artifact binding drift')
    if live and verifier is not None:
        raise MachineIdentityError('injected proof verifiers are test-only')
    _key_verify(public_key, _key_challenge_payload(challenge_value), signature, verifier)
    if live:
        raw_machine_id, public = _key_snapshot(provider)
        if _key_machine_digest(artifact['campaign_id'], raw_machine_id) != artifact['machine_id_sha256'] or \
                public != artifact['public']:
            raise MachineIdentityError('keyed machine identity does not match current host')
        return {'status': 'PASS', 'campaign_domain': KEY_DOMAIN,
                'campaign_id': artifact['campaign_id'], 'identity_sha256': artifact['identity_sha256'],
                'artifact_path': str(path), 'artifact_sha256': _sha(data),
                'canonical_sha256': artifact['canonical_sha256'], 'live_host_match': True,
                'host_match_status': 'LIVE_HOST_MATCH_VERIFIED', 'method': KEY_METHOD,
                'key_id': key_projection['key_id'], 'public_key_sha256': key_projection['public_key_sha256'],
                'challenge_payload_sha256': artifact['challenge']['challenge_payload_sha256'],
                'proof_signature_sha256': artifact['proof']['signature_sha256']}
    return {'status': 'PASS', 'campaign_domain': KEY_DOMAIN,
            'campaign_id': artifact['campaign_id'], 'identity_sha256': artifact['identity_sha256'],
            'artifact_path': str(path), 'artifact_sha256': _sha(data),
            'canonical_sha256': artifact['canonical_sha256'], 'live_host_match': False,
            'host_match_status': 'HOST_MATCH_NOT_CHECKED', 'method': KEY_METHOD,
            'key_id': key_projection['key_id'], 'public_key_sha256': key_projection['public_key_sha256'],
            'challenge_payload_sha256': artifact['challenge']['challenge_payload_sha256'],
            'proof_signature_sha256': artifact['proof']['signature_sha256']}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    capture = sub.add_parser('capture')
    capture.add_argument('--output', type=Path, required=True)
    capture.add_argument('--campaign-id', required=True)
    validate = sub.add_parser('validate')
    validate.add_argument('--artifact', type=Path, required=True)
    validate.add_argument('--live', action='store_true')
    validate.add_argument('--campaign-id')
    challenge = sub.add_parser('prepare-key-challenge')
    challenge.add_argument('--output', type=Path, required=True)
    challenge.add_argument('--final-output', type=Path, required=True)
    challenge.add_argument('--key-descriptor', type=Path, required=True)
    challenge.add_argument('--campaign-id', required=True)
    finalize = sub.add_parser('finalize-keyed')
    finalize.add_argument('--output', type=Path, required=True)
    finalize.add_argument('--challenge', type=Path, required=True)
    finalize.add_argument('--proof', type=Path, required=True)
    finalize.add_argument('--captured-at', required=True)
    finalize.add_argument('--status', choices=['LIVE_CAPTURED', 'SYNTHETIC_TEST'],
                          default='LIVE_CAPTURED')
    keyed_validate = sub.add_parser('validate-keyed')
    keyed_validate.add_argument('--artifact', type=Path, required=True)
    keyed_validate.add_argument('--live', action='store_true')
    args = parser.parse_args()
    try:
        if args.command == 'capture':
            result = capture_artifact(args.output, campaign_id=args.campaign_id)
        elif args.command == 'validate':
            result = validate_file(args.artifact, live=args.live)
            if args.campaign_id and result['campaign_id'] != _safe_campaign(args.campaign_id):
                raise MachineIdentityError('campaign identity mismatch')
        elif args.command == 'prepare-key-challenge':
            result = prepare_key_challenge(args.output, key_descriptor=args.key_descriptor,
                                           final_output=args.final_output,
                                           campaign_id=args.campaign_id)
        elif args.command == 'finalize-keyed':
            result = finalize_keyed_artifact(args.output, challenge=args.challenge,
                                             proof=args.proof, captured_at=args.captured_at,
                                             status=args.status)
        else:
            result = validate_keyed_file(args.artifact, live=args.live)
        print(json.dumps(result, sort_keys=True))
        return 0
    except (OSError, MachineIdentityError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
