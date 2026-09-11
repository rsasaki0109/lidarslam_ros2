#!/usr/bin/env python3
# flake8: noqa
"""Verify a signed, non-promoting review of an r3 source-worktree snapshot.

This module contains verification only: there is no key-generation or signing
path.  The checked-in policy has no keys and remains ``NOT_READY``.  A
synthetic READY policy may be supplied by an external custodian test seam, but
even an accepted review is limited to the r3 candidate review and cannot
promote a profile, benchmark, or README claim.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
from pathlib import Path
import os
import re
import stat
import sys
import time
from typing import Any, Mapping

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from scripts import capture_competitive_execution_r3_source_worktree as source  # noqa: E402
from scripts import capture_competitive_execution_source_worktree as source_impl  # noqa: E402
from scripts import validate_competitive_execution_selection_r3_external_review as external  # noqa: E402
from scripts import prepare_competitive_execution_selection_r3_handoff as sealed  # noqa: E402
from scripts.validate_competitive_execution_selection_r3 import (  # noqa: E402
    CANDIDATE_REL,
    ROOT,
    validate_candidate,
)


RESPONSE_SCHEMA = 'competitive_execution_r3_source_worktree_review_response_v1'
SIDECAR_SCHEMA = 'competitive_execution_r3_source_worktree_review_sidecar_v1'
TRUST_SCHEMA = 'competitive_execution_r3_source_worktree_review_trust_policy_v1'
RESPONSE_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_r3_source_worktree_review_response_v1.schema.json')
TRUST_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_r3_source_worktree_review_trust_policy_v1.schema.json')
TRUST_POLICY_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_r3_source_worktree_review_trust_policy.json')
SNAPSHOT_SCHEMA_REL = source.SCHEMA_REL
SNAPSHOT_STRICT_SCHEMA_REL = source.STRICT_SCHEMA_REL
SNAPSHOT_SIDECAR_SCHEMA_REL = source.SIDECAR_SCHEMA_REL
SCRIPT = Path(__file__).resolve()
SCRIPT_REL = SCRIPT.relative_to(ROOT).as_posix()
FIXED_POLICY = ROOT / TRUST_POLICY_REL
CAMPAIGN_ID = 'competitive-execution-selection-2026-08-r3-candidate'
ALGORITHM = 'Ed25519'
BACKEND_NAME = 'python-cryptography'
BACKEND_IMPLEMENTATION = (
    'cryptography.hazmat.primitives.asymmetric.ed25519.Ed25519PublicKey.verify')
DOMAIN = 'lidarslam/competitive-execution-r3/source-worktree-review/ed25519/v1'
MAX_JSON_BYTES = 64 * 1024 * 1024
MAX_SIDE_BYTES = 4096
MAX_WINDOW = 30 * 24 * 60 * 60
NONCE_RE = re.compile(r'^[A-Za-z0-9_-]{32,128}$')
SHA_RE = re.compile(r'^[0-9a-f]{64}$')
ACCEPTED = 'ACCEPTED_FOR_R3_CANDIDATE_REVIEW_ONLY'
DECISIONS = (ACCEPTED, 'REJECTED', 'NEEDS_CLARIFICATION')


class SourceWorktreeReviewError(ValueError):
    """Malformed, stale, untrusted, or cross-campaign review input."""


class SourceWorktreeReviewNotReady(SourceWorktreeReviewError):
    """The checked-in zero-key policy cannot authorize a response."""


def _canonical(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def _hash(value: Any) -> str:
    return hashlib.sha256(value if isinstance(value, bytes) else _canonical(value)).hexdigest()


def _absolute(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.startswith('/') or '\x00' in value:
        raise SourceWorktreeReviewError(f'{label} must be absolute')
    path = Path(value)
    if path == Path('/') or path.as_posix() != value or '..' in path.parts:
        raise SourceWorktreeReviewError(f'{label} is not normalized')
    return path


def _read(path: Path, label: str, *, limit: int = MAX_JSON_BYTES,
          mode: int | None = None, immutable: bool = True) -> tuple[bytes, dict[str, int]]:
    try:
        identity = sealed._file_identity(path, label, max_bytes=limit,
                                         read_only=immutable)
    except Exception as error:
        raise SourceWorktreeReviewError(str(error)) from error
    if mode is not None and identity['mode'] != mode:
        raise SourceWorktreeReviewError(f'{label} mode is not {oct(mode)}')
    fd = None
    chunks: list[bytes] = []
    total = 0
    try:
        fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
        opened = os.fstat(fd)
        expected = (identity['device'], identity['inode'], identity['nlink'],
                    identity['size'], identity['mode'])
        before = (opened.st_dev, opened.st_ino, opened.st_nlink,
                  opened.st_size, stat.S_IMODE(opened.st_mode))
        if before != expected:
            raise SourceWorktreeReviewError(f'{label} changed before read')
        while True:
            block = os.read(fd, min(1024 * 1024, limit + 1 - total))
            if not block:
                break
            chunks.append(block)
            total += len(block)
            if total > limit:
                raise SourceWorktreeReviewError(f'{label} is oversized')
        closed = os.fstat(fd)
        after = (closed.st_dev, closed.st_ino, closed.st_nlink,
                 closed.st_size, stat.S_IMODE(closed.st_mode))
        if after != expected or total != identity['size']:
            raise SourceWorktreeReviewError(f'{label} changed during read')
    except OSError as error:
        raise SourceWorktreeReviewError(f'{label} cannot be read safely') from error
    finally:
        if fd is not None:
            os.close(fd)
    try:
        again = sealed._file_identity(path, label, max_bytes=limit,
                                      read_only=immutable)
    except Exception as error:
        raise SourceWorktreeReviewError(str(error)) from error
    if dict(again) != dict(identity):
        raise SourceWorktreeReviewError(f'{label} changed after read')
    return b''.join(chunks), identity


def _json(path: Path, label: str, *, limit: int = MAX_JSON_BYTES,
          mode: int | None = None, immutable: bool = True) -> tuple[dict[str, Any], bytes, dict[str, int]]:
    data, identity = _read(path, label, limit=limit, mode=mode,
                           immutable=immutable)
    try:
        value = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise SourceWorktreeReviewError(f'{label} is invalid JSON') from error
    if not isinstance(value, dict):
        raise SourceWorktreeReviewError(f'{label} is not an object')
    return value, data, identity


def _sidecar(path: Path, label: str, *, artifact: bytes,
             canonical_sha: str) -> tuple[dict[str, Any], bytes, dict[str, int]]:
    side_path = path.with_name(path.name + '.sha256.json')
    side, side_data, side_identity = _json(
        side_path, f'{label} sidecar', limit=MAX_SIDE_BYTES, mode=0o444)
    expected = {
        'schema': SIDECAR_SCHEMA, 'schema_version': 1,
        'artifact_path': path.name, 'artifact_bytes': len(artifact),
        'artifact_sha256': _hash(artifact), 'canonical_sha256': canonical_sha,
    }
    if side != expected:
        raise SourceWorktreeReviewError(f'{label} sidecar binding drift')
    return side, side_data, side_identity


def _response_pair(path: Path) -> tuple[dict[str, Any], bytes, dict[str, int]]:
    value, data, identity = _json(path, 'source-worktree response',
                                   mode=0o444)
    if value.get('schema') != RESPONSE_SCHEMA or value.get('schema_version') != 1 or \
            value.get('response_identity_sha256') != _hash({
                key: item for key, item in value.items()
                if key != 'response_identity_sha256'}):
        raise SourceWorktreeReviewError('response canonical identity is invalid')
    _sidecar(path, 'source-worktree response', artifact=data,
             canonical_sha=value['response_identity_sha256'])
    value2, data2, identity2 = _json(path, 'source-worktree response recheck',
                                      mode=0o444)
    if value2 != value or data2 != data or identity2 != identity:
        raise SourceWorktreeReviewError('response changed during validation')
    return value, data, identity


def _snapshot_pair(path: Path) -> tuple[dict[str, Any], bytes, dict[str, int], dict[str, Any]]:
    path = _absolute(str(path), 'snapshot')
    value, data, file_identity = _json(path, 'source-worktree snapshot',
                                        limit=MAX_JSON_BYTES, mode=0o444)
    if value.get('schema') != source.SCHEMA or value.get('schema_version') != 1 or \
            value.get('snapshot_identity_sha256') != _hash({
                key: item for key, item in value.items()
                if key != 'snapshot_identity_sha256'}):
        raise SourceWorktreeReviewError('snapshot canonical identity is invalid')
    side_path = path.with_name(path.name + '.sha256.json')
    side, side_data, side_identity = _json(
        side_path, 'source-worktree snapshot sidecar',
        limit=MAX_SIDE_BYTES, mode=0o444)
    expected = {
        'schema': source.SIDECAR_SCHEMA, 'schema_version': 1,
        'artifact_path': path.name, 'artifact_bytes': len(data),
        'artifact_sha256': _hash(data),
        'snapshot_identity_sha256': value['snapshot_identity_sha256'],
    }
    if side != expected:
        raise SourceWorktreeReviewError('snapshot sidecar binding drift')
    value2, data2, identity2 = _json(path, 'source-worktree snapshot recheck',
                                      limit=MAX_JSON_BYTES, mode=0o444)
    side2, side_data2, side_identity2 = _json(
        side_path, 'source-worktree snapshot sidecar recheck',
        limit=MAX_SIDE_BYTES, mode=0o444)
    if value2 != value or data2 != data or identity2 != file_identity or \
            side2 != side or side_data2 != side_data or side_identity2 != side_identity:
        raise SourceWorktreeReviewError('snapshot changed during validation')
    descriptor = {
        'path': str(path), 'file_bytes': len(data), 'file_sha256': _hash(data),
        'canonical_sha256': value['snapshot_identity_sha256'],
        'device': file_identity['device'], 'inode': file_identity['inode'],
        'mode': file_identity['mode'],
        'sidecar': {
            'path': str(side_path), 'file_bytes': len(side_data),
            'file_sha256': _hash(side_data),
            'device': side_identity['device'], 'inode': side_identity['inode'],
            'mode': side_identity['mode'],
        },
        'snapshot': value,
    }
    return value, data, file_identity, descriptor


def _current_contract() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    try:
        candidate, profile, source_manifest = source_impl._candidate_contract()
    except Exception as error:
        raise SourceWorktreeReviewError(str(error)) from error
    return candidate, profile, source_manifest


def _source_manifest_identity(source_manifest: Mapping[str, Any]) -> str:
    return _hash(dict(source_manifest))


def _current_context() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], str]:
    candidate, profile, source_manifest = _current_contract()
    return candidate, profile, source_manifest, _source_manifest_identity(source_manifest)


def _file_ref(path: Path, label: str, *, relative: bool = True) -> dict[str, str]:
    data, _identity = _read(path, label, immutable=False)
    return {
        'path': path.relative_to(ROOT).as_posix() if relative else str(path),
        'file_sha256': _hash(data),
    }


def _verifier() -> dict[str, Any]:
    data, _identity = _read(SCRIPT, 'source-worktree review verifier',
                             immutable=False)
    import cryptography
    digest = _hash(data)
    return {
        'path': SCRIPT_REL, 'file_sha256': digest,
        'implementation': 'competitive_execution_r3_source_worktree_review_v1',
        'backend': {
            'name': BACKEND_NAME,
            'version': getattr(cryptography, '__version__', ''),
            'implementation': BACKEND_IMPLEMENTATION,
            'implementation_file': SCRIPT_REL,
            'implementation_sha256': digest,
        },
    }


def _candidate_source_binding() -> dict[str, Any]:
    candidate, profile, source_manifest, source_identity = _current_context()
    schema_data, _ = _read(ROOT / RESPONSE_SCHEMA_REL, 'response schema',
                            immutable=False)
    return {
        'candidate': candidate,
        'profile': profile,
        'source_manifest_identity_sha256': source_identity,
        'response_schema_binding': {
            'path': RESPONSE_SCHEMA_REL, 'file_sha256': _hash(schema_data),
            'schema': RESPONSE_SCHEMA,
        },
        'source_schema_bindings': {
            'wrapper': _file_ref(ROOT / SNAPSHOT_SCHEMA_REL, 'snapshot schema'),
            'strict': _file_ref(ROOT / SNAPSHOT_STRICT_SCHEMA_REL,
                                'snapshot strict schema'),
            'sidecar': _file_ref(ROOT / SNAPSHOT_SIDECAR_SCHEMA_REL,
                                 'snapshot sidecar schema'),
        },
    }


def _policy(path: Path, expected: Mapping[str, Any], now: int) -> tuple[dict[str, Any], dict[str, Any]]:
    path = _absolute(str(path), 'trust policy')
    if path == FIXED_POLICY:
        raise SourceWorktreeReviewNotReady(
            'checked-in source-worktree trust policy is NOT_READY with zero keys')
    policy, data, identity = _json(path, 'source-worktree trust policy', mode=0o444)
    if policy.get('schema') != TRUST_SCHEMA or policy.get('schema_version') != 1 or \
            policy.get('policy_identity_sha256') != _hash({
                key: item for key, item in policy.items()
                if key != 'policy_identity_sha256'}):
        raise SourceWorktreeReviewError('trust policy canonical identity is invalid')
    _sidecar(path, 'source-worktree trust policy', artifact=data,
             canonical_sha=policy['policy_identity_sha256'])
    required = {
        'schema', 'schema_version', 'status', 'benchmark_eligible', 'campaign_id',
        'candidate_binding', 'profile_binding', 'source_manifest_identity_sha256',
        'response_schema_binding', 'source_schema_bindings', 'verifier',
        'authorized_keys', 'nonce_policy', 'policy_identity_sha256'}
    if set(policy) != required or policy['status'] != 'READY' or \
            policy['benchmark_eligible'] is not False or policy['campaign_id'] != CAMPAIGN_ID:
        raise SourceWorktreeReviewError('trust policy is not strict READY')
    if policy['candidate_binding'] != expected['candidate'] or \
            policy['profile_binding'] != expected['profile'] or \
            policy['source_manifest_identity_sha256'] != expected[
                'source_manifest_identity_sha256']:
        raise SourceWorktreeReviewError('trust policy candidate/source binding drift')
    if policy['response_schema_binding'] != expected['response_schema_binding'] or \
            policy['source_schema_bindings'] != expected['source_schema_bindings'] or \
            policy['verifier'] != _verifier():
        raise SourceWorktreeReviewError('trust policy producer/schema binding drift')
    if policy['nonce_policy'] != {
            'minimum_length': 32, 'maximum_length': 128,
            'one_shot_ledger_required': True}:
        raise SourceWorktreeReviewError('trust policy nonce contract drift')
    keys = policy['authorized_keys']
    if not isinstance(keys, list) or len(keys) != 1:
        raise SourceWorktreeReviewError('trust policy must contain one active key')
    key = keys[0]
    required_key = {
        'key_id', 'algorithm', 'status', 'public_key_base64',
        'public_key_sha256', 'not_before', 'not_after'}
    if not isinstance(key, dict) or set(key) != required_key or \
            key['algorithm'] != ALGORITHM or key['status'] != 'ACTIVE':
        raise SourceWorktreeReviewError('trust policy key descriptor is invalid')
    if type(key['not_before']) is not int or type(key['not_after']) is not int or \
            key['not_before'] < 0 or key['not_after'] <= key['not_before'] or \
            now < key['not_before'] or now >= key['not_after']:
        raise SourceWorktreeReviewError('trust policy key is outside validity')
    try:
        raw = base64.b64decode(key['public_key_base64'].encode('ascii'), validate=True)
    except (ValueError, UnicodeError) as error:
        raise SourceWorktreeReviewError('trust policy public key is invalid base64') from error
    if len(raw) != 32 or _hash(raw) != key['public_key_sha256']:
        raise SourceWorktreeReviewError('trust policy public key hash drift')
    return policy, {
        'path': str(path), 'file_bytes': len(data), 'file_sha256': _hash(data),
        'canonical_sha256': policy['policy_identity_sha256'],
        'schema': TRUST_SCHEMA, 'key': key, 'raw': raw,
    }


def _review_scope(snapshot_value: Mapping[str, Any]) -> dict[str, Any]:
    return {
        'kind': 'source_worktree',
        'candidate_profile_source_manifest_bound': True,
        'head_bound': True,
        'tracked_diff': {
            'staged_and_unstaged_binary': True,
            'sha256_bound': True, 'bytes_bound': True,
        },
        'untracked': {
            'git_exclude_standard': True,
            'path_size_sha256_projection_bound': True,
            'raw_content_received': False,
        },
        'submodules': {'status_and_revision_identity_bound': True},
        'symlinks': 'reject', 'hardlinks': 'reject',
        'dataset_gt_scorer': 'not_opened', 'network': 'none',
        'offline_only': True,
        'dirty_worktree': snapshot_value['worktree']['worktree_dirty'],
        'dirty_worktree_requires_explicit_ack': True,
    }


def build_response_template(*, snapshot_path: Path, policy_path: Path,
                            nonce: str, issued_at: int, expires_at: int,
                            decision_status: str, reviewer_note: str,
                            signature: Mapping[str, Any]) -> dict[str, Any]:
    """Build an unsigned external-review envelope; never signs it."""
    if NONCE_RE.fullmatch(nonce) is None:
        raise SourceWorktreeReviewError('review nonce is invalid')
    if type(issued_at) is not int or type(expires_at) is not int or issued_at < 0 or \
            expires_at <= issued_at or expires_at - issued_at > MAX_WINDOW:
        raise SourceWorktreeReviewError('review validity is invalid')
    if decision_status not in DECISIONS or not isinstance(reviewer_note, str) or \
            not reviewer_note.strip():
        raise SourceWorktreeReviewError('review decision/note is invalid')
    snapshot_value, _snapshot_data, _snapshot_id, snapshot_binding = _snapshot_pair(snapshot_path)
    expected = _candidate_source_binding()
    policy, policy_state = _policy(_absolute(str(policy_path), 'trust policy'), expected, issued_at)
    del policy
    response = {
        'schema': RESPONSE_SCHEMA, 'schema_version': 1,
        'decision_status': decision_status,
        'review_status': 'REVIEWED_EXTERNAL',
        'benchmark_eligible': False, 'claim_eligible': False,
        'active_profile_switch': False, 'campaign_id': CAMPAIGN_ID,
        'nonce': nonce, 'issued_at': issued_at, 'expires_at': expires_at,
        'candidate_binding': expected['candidate'],
        'profile_binding': expected['profile'],
        'source_manifest': snapshot_value['source_manifest'],
        'source_manifest_identity_sha256': expected[
            'source_manifest_identity_sha256'],
        'snapshot_binding': snapshot_binding,
        'review_scope': _review_scope(snapshot_value),
        'review_decision': {
            'status': decision_status,
            'scope_complete': True,
            'dirty_worktree_acknowledged': snapshot_value['worktree']['worktree_dirty'],
            'reviewer_note': reviewer_note,
        },
        'trust_policy': {
            'path': policy_state['path'], 'file_bytes': policy_state['file_bytes'],
            'file_sha256': policy_state['file_sha256'],
            'canonical_sha256': policy_state['canonical_sha256'],
            'schema': TRUST_SCHEMA,
        },
        'verifier': _verifier(),
        'signature': dict(signature),
        'promotion_policy': {
            'benchmark_eligible': False, 'claim_eligible': False,
            'active_profile_switch': False, 'offline_only': True,
            'candidate_review_only': True,
            'requires_acceptance_aggregator': True,
            'requires_candidate_profile_reseal': True,
        },
        'response_identity_sha256': '',
    }
    response['response_identity_sha256'] = _hash({
        key: value for key, value in response.items()
        if key != 'response_identity_sha256'})
    return response


def _payload(response: Mapping[str, Any]) -> bytes:
    signature = response.get('signature')
    if not isinstance(signature, Mapping):
        raise SourceWorktreeReviewError('signature metadata is missing')
    metadata = {key: value for key, value in signature.items()
                if key not in {'signature_base64', 'payload_sha256'}}
    unsigned = {key: value for key, value in response.items()
                if key not in {'signature', 'response_identity_sha256'}}
    unsigned['signature_metadata'] = metadata
    return _canonical({'domain': DOMAIN, 'schema_version': 1,
                       'response': unsigned})


def _verify_signature(response: Mapping[str, Any], state: Mapping[str, Any]) -> None:
    signature = response['signature']
    key = state['key']
    required = {
        'algorithm', 'key_id', 'public_key_sha256', 'payload_sha256',
        'signature_base64', 'backend'}
    if not isinstance(signature, Mapping) or set(signature) != required or \
            signature['algorithm'] != ALGORITHM or signature['key_id'] != key['key_id'] or \
            signature['public_key_sha256'] != key['public_key_sha256'] or \
            signature['backend'] != state['policy']['verifier']['backend']:
        raise SourceWorktreeReviewError('signature metadata drift')
    payload = _payload(response)
    if signature['payload_sha256'] != _hash(payload):
        raise SourceWorktreeReviewError('signature payload hash drift')
    try:
        raw_sig = base64.b64decode(signature['signature_base64'].encode('ascii'),
                                   validate=True)
        from cryptography.exceptions import InvalidSignature
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
        Ed25519PublicKey.from_public_bytes(state['raw']).verify(raw_sig, payload)
    except InvalidSignature as error:
        raise SourceWorktreeReviewError('Ed25519 signature is invalid') from error
    except (TypeError, ValueError, UnicodeError) as error:
        raise SourceWorktreeReviewError('Ed25519 signature cannot be verified') from error
    if len(raw_sig) != 64:
        raise SourceWorktreeReviewError('Ed25519 signature length is invalid')


def _validate_live_snapshot(path: Path, snapshot_value: Mapping[str, Any]) -> None:
    root = _absolute(snapshot_value['worktree']['root'], 'snapshot worktree root')
    try:
        result = source.validate_snapshot(path, root=root)
    except Exception as error:
        raise SourceWorktreeReviewError(
            f'current live worktree does not match snapshot: {error}') from error
    if result.get('status') != 'NOT_REVIEWED_EXTERNAL' or \
            result.get('structural_valid') is not True or \
            result.get('benchmark_eligible') is not False or \
            result.get('claim_eligible') is not False:
        raise SourceWorktreeReviewError('live snapshot validation is not non-promoting')


def validate_response(response_path: Path, *, replay_ledger: Path,
                      policy_path: Path | None = None,
                      review_time: int | None = None) -> dict[str, Any]:
    now = int(time.time()) if review_time is None else review_time
    if type(now) is not int or now < 0:
        raise SourceWorktreeReviewError('review time is invalid')
    response_path = _absolute(str(response_path), 'response')
    response, response_data, response_identity = _response_pair(response_path)
    required = {
        'schema', 'schema_version', 'decision_status', 'review_status',
        'benchmark_eligible', 'claim_eligible', 'active_profile_switch',
        'campaign_id', 'nonce', 'issued_at', 'expires_at', 'candidate_binding',
        'profile_binding', 'source_manifest', 'source_manifest_identity_sha256',
        'snapshot_binding', 'review_scope', 'review_decision', 'trust_policy',
        'verifier', 'signature', 'promotion_policy', 'response_identity_sha256'}
    if set(response) != required or response['decision_status'] not in DECISIONS or \
            response['review_status'] != 'REVIEWED_EXTERNAL' or \
            response['benchmark_eligible'] is not False or \
            response['claim_eligible'] is not False or \
            response['active_profile_switch'] is not False or \
            response['campaign_id'] != CAMPAIGN_ID:
        raise SourceWorktreeReviewError('response shape/status is invalid')
    if type(response['issued_at']) is not int or type(response['expires_at']) is not int or \
            response['issued_at'] < 0 or response['expires_at'] <= response['issued_at'] or \
            response['expires_at'] - response['issued_at'] > MAX_WINDOW or \
            now < response['issued_at'] or now >= response['expires_at']:
        raise SourceWorktreeReviewError('response is outside validity')
    if not isinstance(response['nonce'], str) or NONCE_RE.fullmatch(response['nonce']) is None:
        raise SourceWorktreeReviewError('response nonce is invalid')
    expected = _candidate_source_binding()
    if response['candidate_binding'] != expected['candidate'] or \
            response['profile_binding'] != expected['profile'] or \
            response['source_manifest_identity_sha256'] != expected[
                'source_manifest_identity_sha256'] or \
            response['source_manifest'] != source_impl._candidate_contract()[2]:
        raise SourceWorktreeReviewError('candidate/profile/source manifest drift')
    if response['verifier'] != _verifier():
        raise SourceWorktreeReviewError('review verifier source drift')
    if response['review_scope'] != _review_scope(response['snapshot_binding']['snapshot']):
        raise SourceWorktreeReviewError('review scope is not exact')
    decision = response['review_decision']
    snapshot_value = response['snapshot_binding']['snapshot']
    if not isinstance(decision, Mapping) or set(decision) != {
            'status', 'scope_complete', 'dirty_worktree_acknowledged',
            'reviewer_note'} or decision['status'] != response['decision_status'] or \
            decision['scope_complete'] is not True or \
            decision['dirty_worktree_acknowledged'] != snapshot_value[
                'worktree']['worktree_dirty'] or \
            not isinstance(decision['reviewer_note'], str) or \
            not decision['reviewer_note'].strip():
        raise SourceWorktreeReviewError('review decision is incomplete')
    expected_promotion = {
        'benchmark_eligible': False, 'claim_eligible': False,
        'active_profile_switch': False, 'offline_only': True,
        'candidate_review_only': True,
        'requires_acceptance_aggregator': True,
        'requires_candidate_profile_reseal': True,
    }
    if response['promotion_policy'] != expected_promotion:
        raise SourceWorktreeReviewError('response promotion policy was altered')
    snapshot_path = _absolute(response['snapshot_binding']['path'], 'snapshot')
    snapshot_value_actual, _snapshot_data, _snapshot_identity, actual_binding = _snapshot_pair(snapshot_path)
    if actual_binding != response['snapshot_binding']:
        raise SourceWorktreeReviewError('snapshot file/sidecar binding drift')
    if snapshot_value_actual != response['snapshot_binding']['snapshot']:
        raise SourceWorktreeReviewError('snapshot projection drift')
    _validate_live_snapshot(snapshot_path, snapshot_value_actual)
    # Reopen after the live recapture so a replacement of either immutable file
    # between recapture and signature verification cannot be accepted.
    _snapshot_value_after, _data_after, _id_after, binding_after = _snapshot_pair(snapshot_path)
    if binding_after != actual_binding:
        raise SourceWorktreeReviewError('snapshot changed after live validation')
    policy_path_value = _absolute(response['trust_policy']['path'], 'trust policy')
    if policy_path_value != _absolute(str(policy_path or FIXED_POLICY), 'trust policy'):
        raise SourceWorktreeReviewError('trust policy path drift')
    policy, state = _policy(policy_path_value, expected, now)
    policy_ref = {
        'path': state['path'], 'file_bytes': state['file_bytes'],
        'file_sha256': state['file_sha256'],
        'canonical_sha256': state['canonical_sha256'], 'schema': TRUST_SCHEMA,
    }
    if response['trust_policy'] != policy_ref:
        raise SourceWorktreeReviewError('trust policy descriptor drift')
    _verify_signature(response, {**state, 'policy': policy})
    try:
        external._claim(Path(replay_ledger), response['nonce'],
                         response['response_identity_sha256'])
    except external.ExternalReviewError as error:
        raise SourceWorktreeReviewError(str(error)) from error
    response_data_after, response_identity_after = _read(
        response_path, 'source-worktree response recheck', mode=0o444)
    if response_data_after != response_data or response_identity_after != response_identity:
        raise SourceWorktreeReviewError('response changed after verification')
    return {
        'status': response['decision_status'], 'structural_valid': True,
        'signature_valid': True, 'review_time': now,
        'response_identity_sha256': response['response_identity_sha256'],
        'snapshot_identity_sha256': snapshot_value_actual[
            'snapshot_identity_sha256'],
        'revision': snapshot_value_actual['worktree']['revision'],
        'benchmark_eligible': False, 'claim_eligible': False,
        'active_profile_switch': False, 'promotion_allowed': False,
        'offline_only': True, 'acceptance_aggregator_required': True,
        'replay_protected': True,
    }


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--response', type=Path, required=True)
    parser.add_argument('--replay-ledger', type=Path, required=True)
    parser.add_argument('--policy', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = validate_response(args.response,
                                   replay_ledger=args.replay_ledger,
                                   policy_path=args.policy)
    except (SourceWorktreeReviewError, OSError, ValueError, TypeError,
            json.JSONDecodeError) as error:
        print(json.dumps({'status': 'INVALID', 'error': str(error)}, sort_keys=True))
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(_main())
