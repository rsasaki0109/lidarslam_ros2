#!/usr/bin/env python3
"""Verify signed custodian packets for the two previously unsigned r3 closures.

The selection handoff and dataset-source closure did not have an authoritative
signed verifier.  This module is that verifier; it does not infer approval
from a status field.  Both public entry points reopen the signed response,
the reviewed JSON subject, the candidate/profile, and the external trust
policy before accepting an offline, non-promoting review.  The checked-in
policies contain no keys and therefore remain ``NOT_READY``.
"""

from __future__ import annotations

import base64
import hashlib
import json
import os
import re
import stat
import sys
import time
from pathlib import Path
from typing import Any, Mapping

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from scripts.validate_competitive_execution_selection_r3 import (  # noqa: E402
    CANDIDATE_REL,
    ROOT,
    validate_candidate,
)


CAMPAIGN_ID = 'competitive-execution-selection-2026-08-r3-candidate'
DOMAIN = 'lidarslam/competitive-execution-selection-r3/closure-review/ed25519/v1'  # noqa: E501
ALGORITHM = 'Ed25519'
BACKEND_NAME = 'python-cryptography'
BACKEND_IMPLEMENTATION = (
    'cryptography.hazmat.primitives.asymmetric.ed25519.Ed25519PublicKey.verify')  # noqa: E501
MODULE = Path(__file__).resolve()
MODULE_REL = MODULE.relative_to(ROOT).as_posix()
MAX_BYTES = 64 * 1024 * 1024
MAX_SIDE_BYTES = 4096
MAX_WINDOW = 30 * 24 * 60 * 60
NONCE_RE = re.compile(r'^[A-Za-z0-9_-]{32,128}$')
KINDS = ('selection_handoff', 'dataset_source_closure')
SCHEMAS = {
    'selection_handoff': (
        'competitive_execution_selection_r3_selection_handoff_review_response_v1'),  # noqa: E501
    'dataset_source_closure': (
        'competitive_execution_selection_r3_dataset_source_closure_review_response_v1'),  # noqa: E501
}
SIDECAR_SCHEMA = (
    'competitive_execution_selection_r3_closure_review_response_sidecar_v1')
TRUST_SCHEMAS = {
    'selection_handoff': (
        'competitive_execution_selection_r3_selection_handoff_review_trust_policy_v1'),  # noqa: E501
    'dataset_source_closure': (
        'competitive_execution_selection_r3_dataset_source_closure_review_trust_policy_v1'),  # noqa: E501
}
TRUST_POLICY_RELS = {
    'selection_handoff': (
        'configs/slam_benchmark_profiles/'
        'competitive_execution_selection_r3_selection_handoff_review_trust_policy.json'),  # noqa: E501
    'dataset_source_closure': (
        'configs/slam_benchmark_profiles/'
        'competitive_execution_selection_r3_dataset_source_closure_review_trust_policy.json'),  # noqa: E501
}
SCHEMA_RELS = {
    'selection_handoff': (
        'configs/slam_benchmark_profiles/'
        'competitive_execution_selection_r3_selection_handoff_review_response_v1.schema.json'),  # noqa: E501
    'dataset_source_closure': (
        'configs/slam_benchmark_profiles/'
        'competitive_execution_selection_r3_dataset_source_closure_review_response_v1.schema.json'),  # noqa: E501
}


class ClosureReviewError(ValueError):
    """Malformed, stale, untrusted, or non-promoting closure review."""


class ClosureReviewNotReady(ClosureReviewError):
    """The fixed policy or reviewed external closure is not ready."""


def _canonical(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def _hash(value: Any) -> str:
    payload = value if isinstance(value, bytes) else _canonical(value)
    return hashlib.sha256(payload).hexdigest()


def _absolute(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.startswith('/') or '\x00' in value:  # noqa: E501
        raise ClosureReviewError(f'{label} must be absolute')
    path = Path(value)
    if path == Path('/') or path.as_posix() != value or '..' in path.parts:
        raise ClosureReviewError(f'{label} is not normalized')
    return path


def _identity(path: Path, label: str, *, limit: int = MAX_BYTES,
              mode: int | None = 0o444) -> dict[str, int]:
    current = Path(path.anchor)
    for component in path.parts[1:-1]:
        current /= component
        try:
            item = os.lstat(current)
        except OSError as error:
            raise ClosureReviewError(f'{label} parent is unavailable') from error  # noqa: E501
        if stat.S_ISLNK(item.st_mode) or not stat.S_ISDIR(item.st_mode):
            raise ClosureReviewError(f'{label} parent is unsafe')
    try:
        item = os.lstat(path)
    except OSError as error:
        raise ClosureReviewError(f'{label} is unavailable') from error
    actual_mode = stat.S_IMODE(item.st_mode)
    if (not stat.S_ISREG(item.st_mode) or item.st_nlink != 1 or
            item.st_size <= 0 or item.st_size > limit or
            (mode is not None and actual_mode != mode)):
        raise ClosureReviewError(f'{label} is not a bounded immutable file')
    return {'device': item.st_dev, 'inode': item.st_ino, 'nlink': item.st_nlink,  # noqa: E501
            'size': item.st_size, 'mode': actual_mode}


def _read(path: Path, label: str, *, limit: int = MAX_BYTES,
          mode: int | None = 0o444) -> tuple[bytes, dict[str, int]]:
    before = _identity(path, label, limit=limit, mode=mode)
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
            raise ClosureReviewError(f'{label} changed before read')
        while True:
            block = os.read(fd, min(1024 * 1024, limit + 1 - total))
            if not block:
                break
            chunks.append(block)
            total += len(block)
            if total > limit:
                raise ClosureReviewError(f'{label} is oversized')
        closed = os.fstat(fd)
        closed_id = {'device': closed.st_dev, 'inode': closed.st_ino,
                     'nlink': closed.st_nlink, 'size': closed.st_size,
                     'mode': stat.S_IMODE(closed.st_mode)}
        if closed_id != before or total != before['size']:
            raise ClosureReviewError(f'{label} changed during read')
    except OSError as error:
        raise ClosureReviewError(f'{label} cannot be read safely') from error
    finally:
        if fd is not None:
            os.close(fd)
    if _identity(path, label, limit=limit, mode=mode) != before:
        raise ClosureReviewError(f'{label} changed after read')
    return b''.join(chunks), before


def _json(path: Path, label: str, *, limit: int = MAX_BYTES,
          mode: int | None = 0o444) -> tuple[dict[str, Any], bytes, dict[str, int]]:  # noqa: E501
    data, identity = _read(path, label, limit=limit, mode=mode)
    try:
        value = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ClosureReviewError(f'{label} is invalid JSON') from error
    if not isinstance(value, dict):
        raise ClosureReviewError(f'{label} must be an object')
    return value, data, identity


def _sidecar(path: Path, data: bytes, canonical_sha: str) -> None:
    side_path = path.with_name(path.name + '.sha256.json')
    side, _side_data, _side_identity = _json(
        side_path, f'{path.name} sidecar', limit=MAX_SIDE_BYTES, mode=0o444)
    expected = {
        'schema': SIDECAR_SCHEMA, 'schema_version': 1,
        'artifact_path': path.name, 'artifact_bytes': len(data),
        'artifact_sha256': _hash(data), 'canonical_sha256': canonical_sha,
    }
    if side != expected:
        raise ClosureReviewError(f'{path.name} sidecar binding drift')


def _candidate_profile() -> tuple[dict[str, Any], dict[str, Any]]:
    report = validate_candidate(ROOT / CANDIDATE_REL)
    if report.get('status') != 'NOT_READY' or \
            report.get('benchmark_eligible') is not False or \
            report.get('claim_eligible') is not False:
        raise ClosureReviewError('candidate is not structurally valid NOT_READY')  # noqa: E501
    path = ROOT / CANDIDATE_REL
    data, _ = _read(path, 'candidate', mode=None)
    try:
        candidate = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ClosureReviewError('candidate is invalid JSON') from error
    binding = {
        'path': CANDIDATE_REL, 'file_sha256': _hash(data),
        'candidate_identity_sha256': candidate['candidate_identity_sha256'],
        'candidate_id': candidate['candidate_id'], 'status': 'NOT_READY',
    }
    profile = candidate.get('profile_binding')
    if not isinstance(profile, dict) or set(profile) != {
            'path', 'file_sha256', 'canonical_sha256', 'canonical_hash_kind'}:
        raise ClosureReviewError('candidate profile binding is malformed')
    return binding, dict(profile)


def _verifier() -> dict[str, Any]:
    import cryptography
    data, _ = _read(MODULE, 'closure review verifier', mode=None)
    digest = _hash(data)
    backend = {
        'name': BACKEND_NAME, 'version': getattr(cryptography, '__version__', ''),  # noqa: E501
        'implementation': BACKEND_IMPLEMENTATION,
        'implementation_file': MODULE_REL, 'implementation_sha256': digest,
    }
    return {'path': MODULE_REL, 'file_sha256': digest,
            'implementation': 'competitive_execution_r3_closure_review_v1',
            'backend': backend}


def _policy(path: Path, kind: str, candidate: Mapping[str, Any],
            profile: Mapping[str, Any], now: int) -> tuple[dict[str, Any], dict[str, Any]]:  # noqa: E501
    path = _absolute(str(path), f'{kind} trust policy')
    fixed = ROOT / TRUST_POLICY_RELS[kind]
    if path == fixed:
        raise ClosureReviewNotReady(f'{kind} checked-in trust policy is NOT_READY')  # noqa: E501
    policy, data, _ = _json(path, f'{kind} trust policy', mode=0o444)
    expected = {key: value for key, value in policy.items()
                if key != 'policy_identity_sha256'}
    if policy.get('schema') != TRUST_SCHEMAS[kind] or \
            policy.get('schema_version') != 1 or \
            policy.get('policy_identity_sha256') != _hash(expected):
        raise ClosureReviewError(f'{kind} trust policy identity is invalid')
    _sidecar(path, data, policy['policy_identity_sha256'])
    fields = {
        'schema', 'schema_version', 'status', 'benchmark_eligible', 'campaign_id',  # noqa: E501
        'closure_kind', 'candidate_binding', 'profile_binding',
        'response_schema_binding', 'verifier', 'authorized_keys', 'nonce_policy',  # noqa: E501
        'policy_identity_sha256'}
    if set(policy) != fields or policy['status'] != 'READY' or \
            policy['benchmark_eligible'] is not False or \
            policy['campaign_id'] != CAMPAIGN_ID or policy['closure_kind'] != kind:  # noqa: E501
        raise ClosureReviewNotReady(f'{kind} trust policy is not strict READY')
    if policy['candidate_binding'] != candidate or policy['profile_binding'] != profile:  # noqa: E501
        raise ClosureReviewError(f'{kind} candidate/profile policy drift')
    expected_verifier = _verifier()
    if policy['verifier'] != expected_verifier:
        raise ClosureReviewError(f'{kind} verifier policy drift')
    schema_data, _ = _read(ROOT / SCHEMA_RELS[kind], f'{kind} response schema', mode=None)  # noqa: E501
    if policy['response_schema_binding'] != {
            'path': SCHEMA_RELS[kind], 'file_sha256': _hash(schema_data),
            'schema': SCHEMAS[kind]}:
        raise ClosureReviewError(f'{kind} response schema policy drift')
    if policy['nonce_policy'] != {
            'minimum_length': 32, 'maximum_length': 128,
            'one_shot_ledger_required': True}:
        raise ClosureReviewError(f'{kind} nonce policy drift')
    keys = policy['authorized_keys']
    if not isinstance(keys, list) or len(keys) != 1:
        raise ClosureReviewNotReady(f'{kind} trust policy has no single active key')  # noqa: E501
    key = keys[0]
    required = {'key_id', 'algorithm', 'status', 'public_key_base64',
                'public_key_sha256', 'not_before', 'not_after'}
    if not isinstance(key, dict) or set(key) != required or \
            key['algorithm'] != ALGORITHM or key['status'] != 'ACTIVE':
        raise ClosureReviewError(f'{kind} trust key shape is invalid')
    if (type(key['not_before']) is not int or
            type(key['not_after']) is not int or key['not_before'] < 0 or
            key['not_after'] <= key['not_before'] or
            now < key['not_before'] or now >= key['not_after']):
        raise ClosureReviewError(f'{kind} trust key is outside validity')
    try:
        raw = base64.b64decode(key['public_key_base64'].encode('ascii'), validate=True)  # noqa: E501
    except (ValueError, UnicodeError) as error:
        raise ClosureReviewError(f'{kind} trust key is invalid base64') from error  # noqa: E501
    if len(raw) != 32 or _hash(raw) != key['public_key_sha256']:
        raise ClosureReviewError(f'{kind} trust key hash drift')
    return policy, {'policy': policy, 'key': key, 'raw': raw,
                    'path': str(path), 'file_bytes': len(data),
                    'file_sha256': _hash(data),
                    'canonical_sha256': policy['policy_identity_sha256']}


def _payload(response: Mapping[str, Any]) -> bytes:
    signature = response.get('signature')
    if not isinstance(signature, Mapping):
        raise ClosureReviewError('closure response signature is missing')
    metadata = {key: value for key, value in signature.items()
                if key not in {'signature_base64', 'payload_sha256'}}
    unsigned = {key: value for key, value in response.items()
                if key not in {'signature', 'response_identity_sha256'}}
    unsigned['signature_metadata'] = metadata
    return _canonical({'domain': DOMAIN, 'schema_version': 1,
                       'closure_kind': response.get('closure_kind'),
                       'response': unsigned})


def _signature(response: Mapping[str, Any], state: Mapping[str, Any]) -> None:
    signature = response.get('signature')
    key = state['key']
    required = {'algorithm', 'key_id', 'public_key_sha256', 'payload_sha256',
                'signature_base64', 'backend'}
    if (not isinstance(signature, Mapping) or set(signature) != required or
            signature['algorithm'] != ALGORITHM or
            signature['key_id'] != key['key_id'] or
            signature['public_key_sha256'] != key['public_key_sha256'] or
            signature['backend'] != state['policy']['verifier']['backend']):
        raise ClosureReviewError('closure response signature metadata drift')
    payload = _payload(response)
    if signature['payload_sha256'] != _hash(payload):
        raise ClosureReviewError('closure response payload hash drift')
    try:
        raw_sig = base64.b64decode(signature['signature_base64'].encode('ascii'),  # noqa: E501
                                   validate=True)
        from cryptography.exceptions import InvalidSignature
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey  # noqa: E501
        Ed25519PublicKey.from_public_bytes(state['raw']).verify(raw_sig, payload)  # noqa: E501
    except InvalidSignature as error:
        raise ClosureReviewError('closure response signature is invalid') from error  # noqa: E501
    except (TypeError, ValueError, UnicodeError) as error:
        raise ClosureReviewError('closure response signature cannot be verified') from error  # noqa: E501
    if len(raw_sig) != 64:
        raise ClosureReviewError('closure response signature length is invalid')  # noqa: E501


def _claim(ledger: Path, nonce: str, identity: str, kind: str) -> None:
    if NONCE_RE.fullmatch(nonce) is None:
        raise ClosureReviewError('closure response nonce is malformed')
    ledger = _absolute(str(ledger), 'closure review replay ledger')
    try:
        directory = os.lstat(ledger)
    except OSError as error:
        raise ClosureReviewError('closure replay ledger is unavailable') from error  # noqa: E501
    if not stat.S_ISDIR(directory.st_mode) or stat.S_ISLNK(directory.st_mode):
        raise ClosureReviewError('closure replay ledger is not a directory')
    directory_id = (directory.st_dev, directory.st_ino)
    path = ledger / f'{kind}.{nonce}.claim.json'
    payload = _canonical({'schema': 'competitive_execution_r3_closure_review_nonce_v1',  # noqa: E501
                          'campaign_id': CAMPAIGN_ID, 'closure_kind': kind,
                          'nonce': nonce, 'response_identity_sha256': identity})  # noqa: E501
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
                raise ClosureReviewError('closure replay claim made no progress')  # noqa: E501
            offset += count
        os.fchmod(fd, 0o444)
        os.fsync(fd)
        final = os.fstat(fd)
        if (final.st_dev, final.st_ino) != owned or final.st_nlink != 1 or \
                final.st_size != len(payload):
            raise ClosureReviewError('closure replay claim identity drift')
        os.close(fd)
        fd = None
        observed, _ = _read(path, 'closure replay claim',
                            limit=MAX_SIDE_BYTES, mode=0o444)
        if observed != payload:
            raise ClosureReviewError('closure replay claim content drift')
        after = os.lstat(ledger)
        if (after.st_dev, after.st_ino) != directory_id:
            raise ClosureReviewError('closure replay ledger changed')
        complete = True
    except FileExistsError as error:
        raise ClosureReviewError('closure response nonce has already been claimed') from error  # noqa: E501
    except OSError as error:
        raise ClosureReviewError(f'closure replay claim failed: {error}') from error  # noqa: E501
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
                raise ClosureReviewError(f'closure replay cleanup failed: {error}') from error  # noqa: E501


def _subject(binding: Mapping[str, Any], kind: str) -> tuple[str, str]:
    required = {'path', 'file_bytes', 'file_sha256', 'canonical_sha256', 'schema'}  # noqa: E501
    if set(binding) != required:
        raise ClosureReviewError(f'{kind} subject binding is malformed')
    path = _absolute(binding['path'], f'{kind} subject')
    data, _ = _read(path, f'{kind} subject', mode=0o444)
    if binding['file_bytes'] != len(data) or binding['file_sha256'] != _hash(data):  # noqa: E501
        raise ClosureReviewError(f'{kind} subject file binding drift')
    try:
        document = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ClosureReviewError(f'{kind} subject is not canonical JSON') from error  # noqa: E501
    if not isinstance(document, dict) or binding['canonical_sha256'] != _hash(document):  # noqa: E501
        raise ClosureReviewError(f'{kind} subject canonical binding drift')
    if not isinstance(binding['schema'], str) or not binding['schema']:
        raise ClosureReviewError(f'{kind} subject schema is missing')
    return str(path), binding['canonical_sha256']


def _validate_response(response_path: Path, *, kind: str, replay_ledger: Path,
                       policy_path: Path | None, review_time: int | None) -> dict[str, Any]:  # noqa: E501
    if kind not in KINDS:
        raise ClosureReviewError('unknown closure kind')
    now = int(time.time()) if review_time is None else review_time
    if type(now) is not int or now < 0:
        raise ClosureReviewError('closure review time is invalid')
    response_path = _absolute(str(response_path), f'{kind} response')
    response, data, _ = _json(response_path, f'{kind} response', mode=0o444)
    identity_field = 'response_identity_sha256'
    if (response.get('schema') != SCHEMAS[kind] or
            response.get('schema_version') != 1 or
            response.get(identity_field) != _hash({
                key: value for key, value in response.items()
                if key != identity_field})):
        raise ClosureReviewError(f'{kind} response identity is invalid')
    _sidecar(response_path, data, response[identity_field])
    required = {
        'schema', 'schema_version', 'decision_status', 'review_status',
        'benchmark_eligible', 'claim_eligible', 'active_profile_switch',
        'promotion_allowed', 'campaign_id', 'closure_kind', 'nonce', 'issued_at',  # noqa: E501
        'expires_at', 'candidate_binding', 'profile_binding', 'subject_binding',  # noqa: E501
        'review_scope', 'trust_policy', 'verifier', 'signature',
        'promotion_policy', 'response_identity_sha256'}
    if set(response) != required or response['decision_status'] != 'ACCEPTED' or \
            response['review_status'] != 'REVIEWED_EXTERNAL' or \
            response['benchmark_eligible'] is not False or \
            response['claim_eligible'] is not False or \
            response['active_profile_switch'] is not False or \
            response['promotion_allowed'] is not False or \
            response['campaign_id'] != CAMPAIGN_ID or response['closure_kind'] != kind:  # noqa: E501
        raise ClosureReviewError(f'{kind} response status or shape is not promotable-safe')  # noqa: E501
    if (type(response['issued_at']) is not int or
            type(response['expires_at']) is not int or
            response['issued_at'] < 0 or
            response['expires_at'] <= response['issued_at'] or
            response['expires_at'] - response['issued_at'] > MAX_WINDOW or
            now < response['issued_at'] or now >= response['expires_at'] or
            NONCE_RE.fullmatch(response['nonce']) is None):
        raise ClosureReviewError(f'{kind} response validity or nonce is invalid')  # noqa: E501
    candidate, profile = _candidate_profile()
    if response['candidate_binding'] != candidate or response['profile_binding'] != profile:  # noqa: E501
        raise ClosureReviewError(f'{kind} candidate/profile binding drift')
    expected_scope = {
        'closure_kind': kind, 'candidate_id': candidate['candidate_id'],
        'profile_canonical_sha256': profile['canonical_sha256'],
        'decision': 'ACCEPTED', 'offline_only': True,
    }
    if response['review_scope'] != expected_scope:
        raise ClosureReviewError(f'{kind} review scope drift')
    _subject(response['subject_binding'], kind)
    if _absolute(response['trust_policy']['path'], f'{kind} policy') != \
            _absolute(str(policy_path or ROOT / TRUST_POLICY_RELS[kind]), f'{kind} policy'):  # noqa: E501
        raise ClosureReviewError(f'{kind} policy path drift')
    policy, state = _policy(_absolute(response['trust_policy']['path'], f'{kind} policy'),  # noqa: E501
                            kind, candidate, profile, now)
    policy_ref = {'path': state['path'], 'file_bytes': state['file_bytes'],
                  'file_sha256': state['file_sha256'],
                  'canonical_sha256': state['canonical_sha256'],
                  'schema': TRUST_SCHEMAS[kind]}
    if response['trust_policy'] != policy_ref:
        raise ClosureReviewError(f'{kind} response trust policy descriptor drift')  # noqa: E501
    if response['verifier'] != _verifier():
        raise ClosureReviewError(f'{kind} response verifier drift')
    _signature(response, {**state, 'policy': policy})
    _claim(_absolute(str(replay_ledger), 'closure replay ledger'),
           response['nonce'], response[identity_field], kind)
    after, after_id = _read(response_path, f'{kind} response recheck', mode=0o444)  # noqa: E501
    if after != data or after_id != _identity(response_path, f'{kind} response recheck'):  # noqa: E501
        raise ClosureReviewError(f'{kind} response changed during validation')
    return {
        'status': 'PASS', 'verified': True, 'campaign_id': CAMPAIGN_ID,
        'candidate_identity_sha256': candidate['candidate_identity_sha256'],
        'profile_canonical_sha256': profile['canonical_sha256'],
        'closure_kind': kind, 'response_identity_sha256': response[identity_field],  # noqa: E501
        'signature_valid': True, 'replay_protected': True,
        'benchmark_eligible': False, 'claim_eligible': False,
        'active_profile_switch': False, 'promotion_allowed': False,
        'offline_only': True,
    }


def validate_selection_response(response_path: Path, *, replay_ledger: Path,
                                policy_path: Path | None = None,
                                review_time: int | None = None) -> dict[str, Any]:  # noqa: E501
    """Validate a signed selection-handoff review packet."""
    return _validate_response(response_path, kind='selection_handoff',
                              replay_ledger=replay_ledger, policy_path=policy_path,  # noqa: E501
                              review_time=review_time)


def validate_dataset_response(response_path: Path, *, replay_ledger: Path,
                              policy_path: Path | None = None,
                              review_time: int | None = None) -> dict[str, Any]:  # noqa: E501
    """Validate a signed dataset-source review packet."""
    return _validate_response(response_path, kind='dataset_source_closure',
                              replay_ledger=replay_ledger, policy_path=policy_path,  # noqa: E501
                              review_time=review_time)


def _main() -> int:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind', choices=KINDS, required=True)
    parser.add_argument('--response', type=Path, required=True)
    parser.add_argument('--replay-ledger', type=Path, required=True)
    parser.add_argument('--policy', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = _validate_response(args.response, kind=args.kind,
                                    replay_ledger=args.replay_ledger,
                                    policy_path=args.policy, review_time=None)
    except (ClosureReviewError, OSError, ValueError, TypeError) as error:
        print(json.dumps({'status': 'NOT_READY' if isinstance(
            error, ClosureReviewNotReady) else 'INVALID',
            'benchmark_eligible': False, 'claim_eligible': False,
            'active_profile_switch': False, 'promotion_allowed': False,
            'error': str(error)}, sort_keys=True))
        return 3 if isinstance(error, ClosureReviewNotReady) else 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(_main())
