#!/usr/bin/env python3
"""Verify a signed, non-promoting r3 acceptance aggregation receipt.

This is intentionally a verifier, not a signer.  The checked-in policy has no
keys and remains ``NOT_READY``.  An external custodian may return a detached
Ed25519-signed receipt, but the receipt remains review evidence only:
benchmark, claim, and active-profile promotion are always forbidden here.

Every closure is reopened through its authoritative validator.  The selection
handoff and dataset-source audit use the dedicated signed custodian packet
validator; their checked-in keyless policies still make the production path
``NOT_READY``.  The ``_adapters`` seam exists only for synthetic unit fixtures
and is unavailable to the CLI.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import re
import stat
import sys
import time
from pathlib import Path
from typing import Any, Callable, Mapping

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from scripts import prepare_competitive_execution_selection_r3_handoff as handoff  # noqa: E402,E501
from scripts import validate_competitive_execution_r3_runtime_closure_review as runtime_review  # noqa: E402,E501
from scripts import validate_competitive_execution_r3_source_worktree_review as source_review  # noqa: E402,E501
from scripts import validate_competitive_execution_r3_unsigned_closure_review as closure_review  # noqa: E402,E501
from scripts import validate_competitive_execution_selection_r3_external_review as image_review  # noqa: E402,E501
from scripts import validate_competitive_rival_source_closure_r3_custodian_response as rival_review  # noqa: E402,E501
from scripts.validate_competitive_execution_selection_r3 import (  # noqa: E402
    CANDIDATE_REL,
    ROOT,
    validate_candidate,
)


RESPONSE_SCHEMA = 'competitive_execution_selection_r3_acceptance_response_v1'
SIDECAR_SCHEMA = 'competitive_execution_selection_r3_acceptance_response_sidecar_v1'  # noqa: E501
TRUST_SCHEMA = 'competitive_execution_selection_r3_acceptance_trust_policy_v1'
RESPONSE_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_r3_acceptance_response_v1.schema.json')
SIDECAR_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_r3_acceptance_response_sidecar_v1.schema.json')  # noqa: E501
TRUST_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_r3_acceptance_trust_policy_v1.schema.json')  # noqa: E501
TRUST_POLICY_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_r3_acceptance_trust_policy.json')
FIXED_POLICY = ROOT / TRUST_POLICY_REL
SCRIPT = Path(__file__).resolve()
SCRIPT_REL = SCRIPT.relative_to(ROOT).as_posix()
CAMPAIGN_ID = 'competitive-execution-selection-2026-08-r3-candidate'
DOMAIN = 'lidarslam/competitive-execution-selection-r3/acceptance/ed25519/v1'
ALGORITHM = 'Ed25519'
BACKEND_NAME = 'python-cryptography'
BACKEND_IMPLEMENTATION = (
    'cryptography.hazmat.primitives.asymmetric.ed25519.Ed25519PublicKey.verify')  # noqa: E501
CLOSURES = (
    'selection_handoff', 'rival_source_closure', 'dataset_source_closure',
    'fresh_holdout_authorization', 'machine_identity',
    'image_toolchain_review', 'source_worktree_review')
# These are the only validator implementations that the acceptance receipt is
# allowed to name.  A receipt cannot select an arbitrary file and then claim
# that an adapter made the verification authoritative.
BUILTIN_VALIDATOR_SPECS = {
    'selection_handoff': (
        'scripts/validate_competitive_execution_r3_unsigned_closure_review.py',
        'competitive_execution_r3_selection_handoff_signed_review_v1'),
    'rival_source_closure': (
        'scripts/validate_competitive_rival_source_closure_r3_custodian_response.py',  # noqa: E501
        'competitive_rival_source_closure_r3_custodian_response_v1'),
    'dataset_source_closure': (
        'scripts/validate_competitive_execution_r3_unsigned_closure_review.py',
        'competitive_execution_r3_dataset_source_closure_signed_review_v1'),
    'fresh_holdout_authorization': (
        'scripts/validate_competitive_execution_r3_runtime_closure_review.py',
        'competitive_execution_r3_fresh_holdout_signed_review_v1'),
    'machine_identity': (
        'scripts/validate_competitive_execution_r3_runtime_closure_review.py',
        'competitive_execution_r3_machine_identity_signed_review_v1'),
    'image_toolchain_review': (
        'scripts/validate_competitive_execution_selection_r3_external_review.py',  # noqa: E501
        'competitive_execution_selection_r3_external_review_v1'),
    'source_worktree_review': (
        'scripts/validate_competitive_execution_r3_source_worktree_review.py',
        'competitive_execution_r3_source_worktree_review_v1'),
}
MAX_BYTES = 64 * 1024 * 1024
MAX_SIDECAR_BYTES = 4096
MAX_WINDOW = 30 * 24 * 60 * 60
SHA_RE = re.compile(r'^[0-9a-f]{64}$')
NONCE_RE = re.compile(r'^[A-Za-z0-9_-]{32,128}$')


class AcceptanceError(ValueError):
    """Malformed, stale, unsigned, or unsafe aggregation input."""


class AcceptanceNotReady(AcceptanceError):
    """An external closure or trust policy is not ready for aggregation."""


def _canonical(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def _hash(value: Any) -> str:
    return hashlib.sha256(
        value if isinstance(value, bytes) else _canonical(value)).hexdigest()


def _absolute(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.startswith('/') or '\x00' in value:  # noqa: E501
        raise AcceptanceError(f'{label} must be absolute')
    path = Path(value)
    if path == Path('/') or path.as_posix() != value or '..' in path.parts:
        raise AcceptanceError(f'{label} is not normalized')
    return path


def _file_identity(path: Path, label: str, *, limit: int = MAX_BYTES,
                   mode: int | None = 0o444) -> dict[str, int]:
    current = Path(path.anchor)
    for component in path.parts[1:-1]:
        current /= component
        try:
            item = os.lstat(current)
        except OSError as error:
            raise AcceptanceError(f'{label} parent is unavailable') from error
        if stat.S_ISLNK(item.st_mode) or not stat.S_ISDIR(item.st_mode):
            raise AcceptanceError(f'{label} parent is unsafe')
    try:
        item = os.lstat(path)
    except OSError as error:
        raise AcceptanceError(f'{label} is unavailable') from error
    actual_mode = stat.S_IMODE(item.st_mode)
    if (not stat.S_ISREG(item.st_mode) or item.st_nlink != 1 or
            item.st_size <= 0 or item.st_size > limit or
            (mode is not None and actual_mode != mode)):
        raise AcceptanceError(f'{label} is not an immutable bounded regular file')  # noqa: E501
    return {'device': item.st_dev, 'inode': item.st_ino, 'nlink': item.st_nlink,  # noqa: E501
            'size': item.st_size, 'mode': actual_mode}


def _read(path: Path, label: str, *, limit: int = MAX_BYTES,
          mode: int | None = 0o444) -> tuple[bytes, dict[str, int]]:
    before = _file_identity(path, label, limit=limit, mode=mode)
    fd = None
    chunks: list[bytes] = []
    total = 0
    try:
        fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
        opened = os.fstat(fd)
        opened_id = {'device': opened.st_dev, 'inode': opened.st_ino,
                     'nlink': opened.st_nlink, 'size': opened.st_size,
                     'mode': stat.S_IMODE(opened.st_mode)}
        if opened_id != before:
            raise AcceptanceError(f'{label} changed before read')
        while True:
            block = os.read(fd, min(1024 * 1024, limit + 1 - total))
            if not block:
                break
            chunks.append(block)
            total += len(block)
            if total > limit:
                raise AcceptanceError(f'{label} is oversized')
        closed = os.fstat(fd)
        closed_id = {'device': closed.st_dev, 'inode': closed.st_ino,
                     'nlink': closed.st_nlink, 'size': closed.st_size,
                     'mode': stat.S_IMODE(closed.st_mode)}
        if closed_id != before or total != before['size']:
            raise AcceptanceError(f'{label} changed during read')
    except OSError as error:
        raise AcceptanceError(f'{label} cannot be read safely') from error
    finally:
        if fd is not None:
            os.close(fd)
    if _file_identity(path, label, limit=limit, mode=mode) != before:
        raise AcceptanceError(f'{label} changed after read')
    return b''.join(chunks), before


def _json(path: Path, label: str, *, limit: int = MAX_BYTES,
          mode: int | None = 0o444) -> tuple[dict[str, Any], bytes, dict[str, int]]:  # noqa: E501
    data, identity = _read(path, label, limit=limit, mode=mode)
    try:
        value = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise AcceptanceError(f'{label} is not valid JSON') from error
    if not isinstance(value, dict):
        raise AcceptanceError(f'{label} must be an object')
    return value, data, identity


def _sidecar(path: Path, data: bytes, identity_sha: str) -> None:
    side_path = path.with_name(path.name + '.sha256.json')
    side, _side_data, _side_identity = _json(
        side_path, f'{path.name} sidecar', limit=MAX_SIDECAR_BYTES, mode=0o444)
    expected = {
        'schema': SIDECAR_SCHEMA, 'schema_version': 1,
        'artifact_path': path.name, 'artifact_bytes': len(data),
        'artifact_sha256': _hash(data), 'canonical_sha256': identity_sha,
    }
    if side != expected:
        raise AcceptanceError(f'{path.name} sidecar binding drift')


def _candidate_profile() -> tuple[dict[str, Any], dict[str, Any]]:
    report = validate_candidate(ROOT / CANDIDATE_REL)
    if report.get('status') != 'NOT_READY' or \
            report.get('benchmark_eligible') is not False or \
            report.get('claim_eligible') is not False:
        raise AcceptanceError('candidate is not structurally valid NOT_READY')
    path = ROOT / CANDIDATE_REL
    data, _identity = _read(path, 'candidate', mode=None)
    try:
        candidate = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise AcceptanceError('candidate is not valid JSON') from error
    if not isinstance(candidate, dict):
        raise AcceptanceError('candidate is not an object')
    profile = candidate.get('profile_binding')
    if not isinstance(profile, dict) or set(profile) != {
            'path', 'file_sha256', 'canonical_sha256', 'canonical_hash_kind'}:
        raise AcceptanceError('candidate profile binding is malformed')
    return {
        'path': CANDIDATE_REL,
        'file_sha256': report['candidate_file_sha256'],
        'candidate_identity_sha256': report['candidate_identity_sha256'],
        'candidate_id': report['candidate_id'], 'status': 'NOT_READY'}, dict(profile)  # noqa: E501


def _verifier() -> dict[str, Any]:
    import cryptography
    data, _identity = _read(SCRIPT, 'acceptance verifier', mode=None)
    digest = _hash(data)
    return {
        'path': SCRIPT_REL, 'file_sha256': digest,
        'implementation': 'competitive_execution_selection_r3_acceptance_v1',
        'backend': {
            'name': BACKEND_NAME,
            'version': getattr(cryptography, '__version__', ''),
            'implementation': BACKEND_IMPLEMENTATION,
            'implementation_file': SCRIPT_REL,
            'implementation_sha256': digest,
        },
    }


def _policy(path: Path, candidate: Mapping[str, Any],
            profile: Mapping[str, Any], now: int) -> tuple[dict[str, Any], dict[str, Any]]:  # noqa: E501
    path = _absolute(str(path), 'trust policy')
    if path == FIXED_POLICY:
        raise AcceptanceNotReady('checked-in acceptance trust policy is NOT_READY with zero keys')  # noqa: E501
    policy, data, _identity = _json(path, 'acceptance trust policy', mode=0o444)  # noqa: E501
    if (policy.get('schema') != TRUST_SCHEMA or
            policy.get('schema_version') != 1 or
            policy.get('policy_identity_sha256') != _hash({
                key: value for key, value in policy.items()
                if key != 'policy_identity_sha256'})):
        raise AcceptanceError('acceptance trust policy identity is invalid')
    _sidecar(path, data, policy['policy_identity_sha256'])
    expected_fields = {
        'schema', 'schema_version', 'status', 'benchmark_eligible',
        'campaign_id', 'candidate_binding', 'profile_binding',
        'response_schema_binding', 'verifier', 'authorized_keys',
        'nonce_policy', 'execution_authorized', 'policy_identity_sha256'}
    if set(policy) != expected_fields or policy['status'] != 'READY' or \
            policy['benchmark_eligible'] is not False or \
            policy['campaign_id'] != CAMPAIGN_ID or \
            policy['execution_authorized'] is not True:
        raise AcceptanceNotReady('acceptance trust policy is not strict READY')
    if (policy['candidate_binding'] != candidate or
            policy['profile_binding'] != profile or
            policy['verifier'] != _verifier()):
        raise AcceptanceError('acceptance trust policy candidate/verifier drift')  # noqa: E501
    response_schema = ROOT / RESPONSE_SCHEMA_REL
    response_schema_data, _schema_identity = _read(
        response_schema, 'acceptance response schema', mode=None)
    expected_response_ref = {
        'path': RESPONSE_SCHEMA_REL, 'file_sha256': _hash(response_schema_data),  # noqa: E501
        'schema': RESPONSE_SCHEMA}
    if policy['response_schema_binding'] != expected_response_ref:
        raise AcceptanceError('acceptance response schema binding drift')
    if policy['nonce_policy'] != {
            'minimum_length': 32, 'maximum_length': 128,
            'one_shot_ledger_required': True}:
        raise AcceptanceError('acceptance nonce policy drift')
    keys = policy['authorized_keys']
    if not isinstance(keys, list) or len(keys) != 1:
        raise AcceptanceNotReady('acceptance policy must contain one active key')  # noqa: E501
    key = keys[0]
    if not isinstance(key, dict) or set(key) != {
            'key_id', 'algorithm', 'status', 'public_key_base64',
            'public_key_sha256', 'not_before', 'not_after'} or \
            key['algorithm'] != ALGORITHM or key['status'] != 'ACTIVE':
        raise AcceptanceError('acceptance trust key shape is invalid')
    if (type(key['not_before']) is not int or
            type(key['not_after']) is not int or key['not_before'] < 0 or
            key['not_after'] <= key['not_before'] or
            now < key['not_before'] or now >= key['not_after']):
        raise AcceptanceError('acceptance trust key is outside validity')
    try:
        raw = base64.b64decode(key['public_key_base64'].encode('ascii'), validate=True)  # noqa: E501
    except (ValueError, UnicodeError) as error:
        raise AcceptanceError('acceptance trust key is invalid base64') from error  # noqa: E501
    if len(raw) != 32 or _hash(raw) != key['public_key_sha256']:
        raise AcceptanceError('acceptance trust key hash drift')
    return policy, {'path': str(path), 'file_bytes': len(data),
                    'file_sha256': _hash(data),
                    'canonical_sha256': policy['policy_identity_sha256'],
                    'schema': TRUST_SCHEMA, 'key': key, 'raw': raw}


def _payload(response: Mapping[str, Any]) -> bytes:
    signature = response.get('signature')
    if not isinstance(signature, Mapping):
        raise AcceptanceError('acceptance signature metadata is missing')
    metadata = {key: value for key, value in signature.items()
                if key not in {'signature_base64', 'payload_sha256'}}
    unsigned = {key: value for key, value in response.items()
                if key not in {'signature', 'receipt_identity_sha256'}}
    unsigned['signature_metadata'] = metadata
    return _canonical({'domain': DOMAIN, 'schema_version': 1,
                       'response': unsigned})


def _verify_signature(response: Mapping[str, Any], state: Mapping[str, Any]) -> None:  # noqa: E501
    signature = response['signature']
    key = state['key']
    required = {'algorithm', 'key_id', 'public_key_sha256', 'payload_sha256',
                'signature_base64', 'backend'}
    if (not isinstance(signature, Mapping) or set(signature) != required or
            signature['algorithm'] != ALGORITHM or
            signature['key_id'] != key['key_id'] or
            signature['public_key_sha256'] != key['public_key_sha256'] or
            signature['backend'] != state['policy']['verifier']['backend']):
        raise AcceptanceError('acceptance signature metadata drift')
    payload = _payload(response)
    if signature['payload_sha256'] != _hash(payload):
        raise AcceptanceError('acceptance signature payload drift')
    try:
        raw_sig = base64.b64decode(signature['signature_base64'].encode('ascii'),  # noqa: E501
                                   validate=True)
        from cryptography.exceptions import InvalidSignature
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey  # noqa: E501
        Ed25519PublicKey.from_public_bytes(state['raw']).verify(raw_sig, payload)  # noqa: E501
    except InvalidSignature as error:
        raise AcceptanceError('acceptance Ed25519 signature is invalid') from error  # noqa: E501
    except (TypeError, ValueError, UnicodeError) as error:
        raise AcceptanceError('acceptance Ed25519 signature cannot be verified') from error  # noqa: E501
    if len(raw_sig) != 64:
        raise AcceptanceError('acceptance Ed25519 signature length is invalid')


def _claim(ledger: Path, nonce: str, identity: str) -> None:
    if NONCE_RE.fullmatch(nonce) is None:
        raise AcceptanceError('acceptance nonce is malformed')
    ledger = _absolute(str(ledger), 'acceptance replay ledger')
    try:
        info = handoff._directory_identity(ledger, 'acceptance replay ledger')
    except Exception as error:
        raise AcceptanceError(str(error)) from error
    path = ledger / (nonce + '.acceptance.claim.json')
    if path.exists() or path.is_symlink():
        raise AcceptanceError('acceptance nonce has already been claimed')
    payload = _canonical({'schema': 'competitive_execution_selection_r3_acceptance_nonce_v1',  # noqa: E501
                          'campaign_id': CAMPAIGN_ID, 'nonce': nonce,
                          'receipt_identity_sha256': identity})
    fd = None
    owned: tuple[int, int] | None = None
    complete = False
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                     getattr(os, 'O_NOFOLLOW', 0), 0o444)
        opened = os.fstat(fd)
        owned = (opened.st_dev, opened.st_ino)
        offset = 0
        while offset < len(payload):
            count = os.write(fd, payload[offset:])
            if count <= 0:
                raise AcceptanceError('acceptance nonce claim made no progress')  # noqa: E501
            offset += count
        os.fchmod(fd, 0o444)
        os.fsync(fd)
        final = os.fstat(fd)
        if (final.st_dev, final.st_ino) != owned or final.st_nlink != 1 or \
                final.st_size != len(payload):
            raise AcceptanceError('acceptance nonce claim identity drift')
        os.close(fd)
        fd = None
        observed, _ = _read(path, 'acceptance nonce claim',
                            limit=MAX_SIDECAR_BYTES, mode=0o444)
        if observed != payload or handoff._directory_identity(
                ledger, 'acceptance replay ledger') != info:
            raise AcceptanceError('acceptance nonce claim binding drift')
        directory_fd = os.open(ledger, os.O_RDONLY |
                               getattr(os, 'O_DIRECTORY', 0) |
                               getattr(os, 'O_NOFOLLOW', 0))
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        complete = True
    except FileExistsError as error:
        raise AcceptanceError('acceptance nonce has already been claimed') from error  # noqa: E501
    except OSError as error:
        raise AcceptanceError(f'acceptance nonce claim failed: {error}') from error  # noqa: E501
    finally:
        if fd is not None:
            os.close(fd)
        if owned is not None and not complete:
            try:
                current = os.lstat(path)
                if (current.st_dev, current.st_ino) == owned and current.st_nlink == 1:  # noqa: E501
                    path.unlink()
            except FileNotFoundError:
                pass
            except OSError as error:
                raise AcceptanceError(f'acceptance nonce cleanup failed: {error}') from error  # noqa: E501


def _descriptor(entry: Mapping[str, Any], path: Path,
                data: bytes) -> None:
    if entry['path'] != str(path) or entry['file_sha256'] != _hash(data):
        raise AcceptanceError('closure path/file SHA binding drift')
    if not SHA_RE.fullmatch(entry['canonical_sha256']):
        raise AcceptanceError('closure canonical identity is malformed')


def _builtin_validator_descriptor(name: str) -> dict[str, str]:
    """Return the immutable in-tree validator identity for ``name``."""
    try:
        relative, implementation = BUILTIN_VALIDATOR_SPECS[name]
    except KeyError as error:
        raise AcceptanceError(f'unknown closure validator: {name}') from error
    path = (_ROOT / relative).resolve()
    data, _identity = _read(path, f'{name} builtin validator', mode=None)
    return {'path': str(path), 'file_sha256': _hash(data),
            'implementation': implementation}


def _projection(name: str, result: Mapping[str, Any], entry: Mapping[str, Any],
                candidate: Mapping[str, Any], profile: Mapping[str, Any]) -> dict[str, Any]:  # noqa: E501
    expected = {
        'status': 'PASS',
        'verified': True,
        'campaign_id': CAMPAIGN_ID,
        'candidate_identity_sha256': candidate['candidate_identity_sha256'],
        'profile_canonical_sha256': profile['canonical_sha256'],
        'closure_kind': name,
        'closure_identity_sha256': entry['canonical_sha256'],
        'signature_valid': True,
        'replay_protected': True,
        'benchmark_eligible': False,
        'claim_eligible': False,
    }
    if any(result.get(field) != value for field, value in expected.items()):
        raise AcceptanceNotReady(f'{name} authoritative validator is not PASS')
    return {
        'status': 'VERIFIED',
        **{key: value for key, value in expected.items() if key != 'status'},
    }


def _builtin_adapters() -> dict[str, Callable[..., dict[str, Any]]]:
    # Every closure dispatches to an authoritative validator.  The two
    # formerly unsigned closures use the dedicated signed custodian packet
    # verifier; they are still NOT_READY while their fixed policies have no
    # keys.
    return {
        'selection_handoff': _selection_adapter,
        'dataset_source_closure': _dataset_adapter,
        'rival_source_closure': _rival_adapter,
        'image_toolchain_review': _image_adapter,
        'source_worktree_review': _source_adapter,
        'machine_identity': _machine_adapter,
        'fresh_holdout_authorization': _holdout_adapter,
    }


def _bind_signed_result(name: str, raw: Mapping[str, Any],
                        entry: Mapping[str, Any],
                        context: Mapping[str, Any]) -> dict[str, Any]:
    """Bind an authoritative validator result to the exact closure slot."""
    # The rival validator predates the explicit ``signature_valid`` field,
    # but its ``verified`` result is emitted only after its Ed25519 verifier
    # returns successfully.  Do not use a missing-field default: accept this
    # one documented projection only, and require the replay ledger field for
    # every signed validator.
    signature_valid = raw.get('signature_valid') is True
    if name == 'rival_source_closure' and raw.get('verified') is True:
        signature_valid = True
    if not signature_valid or raw.get('replay_protected') is not True:
        raise AcceptanceNotReady(
            f'{name} validator did not prove signature/replay')
    if raw.get('response_identity_sha256') != entry['canonical_sha256']:
        raise AcceptanceError(
            f'{name} response identity does not match closure')
    if (raw.get('verified') is not True and
            raw.get('structural_valid') is not True):
        raise AcceptanceNotReady(
            f'{name} validator did not prove structural validity')
    result = dict(raw)
    result.update({
        'status': 'PASS', 'verified': True, 'campaign_id': CAMPAIGN_ID,
        'candidate_identity_sha256': context['candidate'][
            'candidate_identity_sha256'],
        'profile_canonical_sha256': context['profile']['canonical_sha256'],
        'closure_kind': name,
        'closure_identity_sha256': entry['canonical_sha256'],
        'signature_valid': signature_valid, 'replay_protected': True,
        'benchmark_eligible': False, 'claim_eligible': False,
    })
    return result


def _selection_adapter(name: str, path: Path, entry: Mapping[str, Any],
                       context: Mapping[str, Any]) -> dict[str, Any]:
    policy_path = _absolute(entry['trust_policy']['path'], 'selection handoff policy')  # noqa: E501
    raw = closure_review.validate_selection_response(
        path, replay_ledger=context['replay_ledger'], policy_path=policy_path,
        review_time=context['review_time'])
    return _bind_signed_result(name, raw, entry, context)


def _dataset_adapter(name: str, path: Path, entry: Mapping[str, Any],
                     context: Mapping[str, Any]) -> dict[str, Any]:
    policy_path = _absolute(entry['trust_policy']['path'], 'dataset source policy')  # noqa: E501
    raw = closure_review.validate_dataset_response(
        path, replay_ledger=context['replay_ledger'], policy_path=policy_path,
        review_time=context['review_time'])
    return _bind_signed_result(name, raw, entry, context)


def _rival_adapter(name: str, path: Path, entry: Mapping[str, Any],
                   context: Mapping[str, Any]) -> dict[str, Any]:
    policy_path = _absolute(entry['trust_policy']['path'], f'{name} policy')
    ledger = context['replay_ledger']
    raw = rival_review.validate_response(
        path, replay_ledger=ledger, _review_time=context['review_time'],
        _policy_path=policy_path)
    return _bind_signed_result(name, raw, entry, context)


def _image_adapter(name: str, path: Path, entry: Mapping[str, Any],
                   context: Mapping[str, Any]) -> dict[str, Any]:
    policy_path = _absolute(entry['trust_policy']['path'], f'{name} policy')
    raw = image_review.validate_response(
        path, replay_ledger=context['replay_ledger'],
        policy_path=policy_path, review_time=context['review_time'])
    return _bind_signed_result(name, raw, entry, context)


def _source_adapter(name: str, path: Path, entry: Mapping[str, Any],
                    context: Mapping[str, Any]) -> dict[str, Any]:
    policy_path = _absolute(entry['trust_policy']['path'], f'{name} policy')
    raw = source_review.validate_response(
        path, replay_ledger=context['replay_ledger'],
        policy_path=policy_path, review_time=context['review_time'])
    return _bind_signed_result(name, raw, entry, context)


def _machine_adapter(name: str, path: Path, entry: Mapping[str, Any],
                     context: Mapping[str, Any]) -> dict[str, Any]:
    try:
        policy_path = _absolute(
            entry['trust_policy']['path'], 'machine policy')
    except (KeyError, TypeError) as error:
        raise AcceptanceNotReady(
            'machine closure has no authoritative trust policy') from error
    raw = runtime_review.validate_machine_response(
        path, replay_ledger=context['replay_ledger'], policy_path=policy_path,
        review_time=context['review_time'])
    return _bind_signed_result(name, raw, entry, context)


def _holdout_adapter(name: str, path: Path, entry: Mapping[str, Any],
                     context: Mapping[str, Any]) -> dict[str, Any]:
    try:
        policy_path = _absolute(
            entry['trust_policy']['path'], 'holdout policy')
    except (KeyError, TypeError) as error:
        raise AcceptanceNotReady(
            'holdout closure has no authoritative trust policy') from error
    raw = runtime_review.validate_holdout_response(
        path, replay_ledger=context['replay_ledger'], policy_path=policy_path,
        review_time=context['review_time'])
    return _bind_signed_result(name, raw, entry, context)


def _validate_closure(name: str, entry: Mapping[str, Any],
                      context: Mapping[str, Any],
                      adapters: Mapping[str, Callable[..., dict[str, Any]]]) -> dict[str, Any]:  # noqa: E501
    required = {'status', 'path', 'file_sha256', 'canonical_sha256',
                'validator', 'trust_policy', 'result'}
    if set(entry) != required or entry['status'] != 'VERIFIED':
        raise AcceptanceNotReady(f'{name} closure is missing/partial/not VERIFIED')  # noqa: E501
    path = _absolute(entry['path'], f'{name} closure')
    data, _identity = _read(path, f'{name} closure', mode=0o444)
    _descriptor(entry, path, data)
    if not isinstance(entry['validator'], Mapping) or set(entry['validator']) != {  # noqa: E501
            'path', 'file_sha256', 'implementation'}:
        raise AcceptanceError(f'{name} validator identity is malformed')
    if entry['validator'] != _builtin_validator_descriptor(name):
        raise AcceptanceError(f'{name} validator is not the fixed builtin')
    if not isinstance(entry['trust_policy'], Mapping) or set(entry['trust_policy']) != {  # noqa: E501
            'path', 'file_sha256', 'canonical_sha256'}:
        raise AcceptanceError(f'{name} trust policy identity is malformed')
    policy_path = _absolute(entry['trust_policy']['path'], f'{name} policy')
    policy_value, policy_data, _policy_identity = _json(
        policy_path, f'{name} trust policy', mode=0o444)
    policy_canonical_field = (
        'policy_identity_sha256' if 'policy_identity_sha256' in policy_value
        else 'canonical_sha256')
    policy_canonical = policy_value.get(policy_canonical_field)
    if not isinstance(policy_canonical, str) or not SHA_RE.fullmatch(policy_canonical):  # noqa: E501
        raise AcceptanceError(f'{name} trust policy has no canonical identity')
    policy_payload = {key: value for key, value in policy_value.items()
                      if key != policy_canonical_field}
    policy_identity = _hash(policy_payload)
    if policy_canonical_field == 'canonical_sha256':
        policy_identity = hashlib.sha256(json.dumps(
            policy_payload, sort_keys=True, separators=(',', ':'),
            ensure_ascii=True).encode('utf-8')).hexdigest()
    if policy_canonical != policy_identity:
        raise AcceptanceError(f'{name} trust policy canonical identity drift')
    if entry['trust_policy']['file_sha256'] != _hash(policy_data) or \
            entry['trust_policy']['canonical_sha256'] != policy_canonical:
        raise AcceptanceError(f'{name} trust policy binding drift')
    adapter = adapters.get(name)
    if not callable(adapter):
        raise AcceptanceNotReady(f'{name} authoritative adapter is missing')
    try:
        result = adapter(name, path, entry, context)
    except AcceptanceError:
        raise
    except Exception as error:
        raise AcceptanceError(f'{name} authoritative validator failed: {error}') from error  # noqa: E501
    projection = _projection(name, result, entry, context['candidate'], context['profile'])  # noqa: E501
    if entry['result'] != projection:
        raise AcceptanceError(f'{name} validator result projection drift')
    return projection


def validate_receipt(response_path: Path, *, replay_ledger: Path,
                     policy_path: Path | None = None,
                     review_time: int | None = None,
                     _adapters: Mapping[str, Callable[..., dict[str, Any]]] | None = None,  # noqa: E501
                     _allow_synthetic: bool = False) -> dict[str, Any]:
    """Reopen and verify one external acceptance receipt.

    ``_adapters`` and ``_allow_synthetic`` are test-only seams; the CLI never
    exposes them.  The normal path dispatches only to fixed in-tree validators.
    """
    now = int(time.time()) if review_time is None else review_time
    if type(now) is not int or now < 0:
        raise AcceptanceError('acceptance review time is invalid')
    response_path = _absolute(str(response_path), 'acceptance response')
    response, data, _identity = _json(response_path, 'acceptance response', mode=0o444)  # noqa: E501
    if (response.get('schema') != RESPONSE_SCHEMA or
            response.get('schema_version') != 1 or
            response.get('receipt_identity_sha256') != _hash({
                key: value for key, value in response.items()
                if key != 'receipt_identity_sha256'})):
        raise AcceptanceError('acceptance response canonical identity is invalid')  # noqa: E501
    _sidecar(response_path, data, response['receipt_identity_sha256'])
    required = {
        'schema', 'schema_version', 'status', 'benchmark_eligible',
        'claim_eligible', 'active_profile_switch', 'promotion_allowed',
        'execution_authorized', 'publication_authorized',
        'sota_claim_authorized', 'requires_separate_publication_review',
        'campaign_id', 'nonce', 'issued_at', 'expires_at',
        'candidate_binding', 'profile_binding', 'required_closures',
        'closures', 'trust_policy', 'verifier', 'signature',
        'promotion_policy', 'receipt_identity_sha256'}
    if (set(response) != required or
            response['status'] != 'READY_FOR_SEPARATE_PUBLICATION_REVIEW' or
            response['benchmark_eligible'] is not False or
            response['claim_eligible'] is not False or
            response['active_profile_switch'] is not False or
            response['promotion_allowed'] is not False or
            response['execution_authorized'] is not True or
            response['publication_authorized'] is not False or
            response['sota_claim_authorized'] is not False or
            response['requires_separate_publication_review'] is not True or
            response['campaign_id'] != CAMPAIGN_ID):
        raise AcceptanceError('acceptance response shape/status is invalid')
    if (type(response['issued_at']) is not int or
            type(response['expires_at']) is not int or
            response['expires_at'] <= response['issued_at'] or
            response['expires_at'] - response['issued_at'] > MAX_WINDOW or
            now < response['issued_at'] or now >= response['expires_at'] or
            NONCE_RE.fullmatch(response['nonce']) is None):
        raise AcceptanceError('acceptance response validity or nonce is invalid')  # noqa: E501
    candidate, profile = _candidate_profile()
    if response['candidate_binding'] != candidate or response['profile_binding'] != profile:  # noqa: E501
        raise AcceptanceError('acceptance candidate/profile binding drift')
    if response['required_closures'] != list(CLOSURES) or \
            not isinstance(response['closures'], Mapping) or \
            set(response['closures']) != set(CLOSURES):
        raise AcceptanceError('acceptance closure coverage is not exact')
    if response['promotion_policy'] != {
            'benchmark_eligible': False, 'claim_eligible': False,
            'active_profile_switch': False, 'promotion_allowed': False,
            'execution_authorized': True, 'publication_authorized': False,
            'sota_claim_authorized': False,
            'requires_separate_publication_review': True,
            'offline_only': True, 'requires_acceptance_aggregator': True,
            'requires_candidate_profile_reseal': True}:
        raise AcceptanceError('acceptance promotion policy was altered')
    expected_verifier = _verifier()
    if response['verifier'] != expected_verifier:
        raise AcceptanceError('acceptance verifier source drift')
    policy_path = _absolute(str(policy_path or FIXED_POLICY), 'acceptance policy')  # noqa: E501
    policy, state = _policy(policy_path, candidate, profile, now)
    policy_ref = {'path': state['path'], 'file_bytes': state['file_bytes'],
                  'file_sha256': state['file_sha256'],
                  'canonical_sha256': state['canonical_sha256'],
                  'schema': TRUST_SCHEMA}
    if response['trust_policy'] != policy_ref:
        raise AcceptanceError('acceptance trust policy descriptor drift')
    _verify_signature(response, {**state, 'policy': policy})
    try:
        import yaml
        profile_data, _profile_identity = _read(
            ROOT / profile['path'], 'profile document', mode=None)
        profile_document = yaml.safe_load(profile_data.decode('utf-8'))
    except (ImportError, UnicodeError, ValueError, TypeError, OSError) as error:  # noqa: E501
        raise AcceptanceError(f'profile document cannot be reopened: {error}') from error  # noqa: E501
    if not isinstance(profile_document, Mapping):
        raise AcceptanceError('profile document is not a mapping')
    context = {
        'candidate': candidate, 'profile': profile,
        'profile_document': profile_document, 'review_time': now,
        'replay_ledger': _absolute(str(replay_ledger), 'acceptance replay ledger')}  # noqa: E501
    adapters = dict(_builtin_adapters())
    if _adapters is not None:
        if not _allow_synthetic:
            raise AcceptanceError('synthetic adapters are test-only')
        adapters.update(_adapters)
    projections = {
        name: _validate_closure(name, response['closures'][name], context, adapters)  # noqa: E501
        for name in CLOSURES}
    try:
        _claim(context['replay_ledger'], response['nonce'],
               response['receipt_identity_sha256'])
    except AcceptanceError:
        raise
    return {
        'status': response['status'], 'verified': True,
        'closure_status': {name: value['status'] for name, value in projections.items()},  # noqa: E501
        'receipt_identity_sha256': response['receipt_identity_sha256'],
        'candidate_identity_sha256': candidate['candidate_identity_sha256'],
        'profile_canonical_sha256': profile['canonical_sha256'],
        'review_time': now, 'signature_valid': True, 'replay_protected': True,
        'benchmark_eligible': False, 'claim_eligible': False,
        'active_profile_switch': False, 'promotion_allowed': False,
        'execution_authorized': True, 'publication_authorized': False,
        'sota_claim_authorized': False,
        'offline_only': True, 'requires_separate_publication_review': True,
    }


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--response', type=Path, required=True)
    parser.add_argument('--replay-ledger', type=Path, required=True)
    parser.add_argument('--policy', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = validate_receipt(args.response, replay_ledger=args.replay_ledger,  # noqa: E501
                                  policy_path=args.policy)
    except (AcceptanceError, OSError, ValueError, TypeError,
            json.JSONDecodeError) as error:
        print(json.dumps({'status': 'NOT_READY' if isinstance(
            error, AcceptanceNotReady) else 'INVALID',
            'benchmark_eligible': False, 'claim_eligible': False,
            'active_profile_switch': False, 'promotion_allowed': False,
            'error': str(error)}, sort_keys=True))
        return 3 if isinstance(error, AcceptanceNotReady) else 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(_main())
