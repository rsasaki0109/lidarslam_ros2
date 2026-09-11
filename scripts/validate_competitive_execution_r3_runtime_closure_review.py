#!/usr/bin/env python3
"""Verify signed r3 reviews for the holdout and machine closures.

This module is the production adapter boundary for the two closures that
cannot be represented by a status-only document.  The signed packet is
validated first as an immutable custodian review, then the referenced subject
is reopened through its authoritative validator.  The checked-in policies
have no keys and therefore remain ``NOT_READY``.
"""

from __future__ import annotations

import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from scripts import capture_competitive_execution_machine_identity_pop as machine_identity  # noqa: E501,E402
from scripts import competitive_holdout_authorization as holdout  # noqa: E501,E402
from scripts import validate_competitive_execution_r3_unsigned_closure_review as base  # noqa: E501,E402


ROOT = base.ROOT
CAMPAIGN_ID = base.CAMPAIGN_ID
DOMAIN = base.DOMAIN
ALGORITHM = base.ALGORITHM
BACKEND_NAME = base.BACKEND_NAME
BACKEND_IMPLEMENTATION = base.BACKEND_IMPLEMENTATION
MODULE = Path(__file__).resolve()
MODULE_REL = MODULE.relative_to(ROOT).as_posix()
MAX_BYTES = base.MAX_BYTES
MAX_SIDE_BYTES = base.MAX_SIDE_BYTES
MAX_WINDOW = base.MAX_WINDOW
KINDS = ('fresh_holdout_authorization', 'machine_identity')
SCHEMAS = {
    'fresh_holdout_authorization': (
        'competitive_execution_r3_fresh_holdout_closure_review_response_v1'),
    'machine_identity': (
        'competitive_execution_r3_machine_identity_closure_review_response_v1'),  # noqa: E501
}
SIDE_SCHEMA = 'competitive_execution_r3_runtime_closure_review_sidecar_v1'
TRUST_SCHEMAS = {
    'fresh_holdout_authorization': (
        'competitive_execution_r3_fresh_holdout_closure_review_trust_policy_v1'),  # noqa: E501
    'machine_identity': (
        'competitive_execution_r3_machine_identity_closure_review_trust_policy_v1'),  # noqa: E501
}
TRUST_POLICY_RELS = {
    'fresh_holdout_authorization': (
        'configs/slam_benchmark_profiles/'
        'competitive_execution_r3_fresh_holdout_closure_review_trust_policy.json'),  # noqa: E501
    'machine_identity': (
        'configs/slam_benchmark_profiles/'
        'competitive_execution_r3_machine_identity_closure_review_trust_policy.json'),  # noqa: E501
}
SCHEMA_RELS = {
    'fresh_holdout_authorization': (
        'configs/slam_benchmark_profiles/'
        'competitive_execution_r3_fresh_holdout_closure_review_response_v1.schema.json'),  # noqa: E501
    'machine_identity': (
        'configs/slam_benchmark_profiles/'
        'competitive_execution_r3_machine_identity_closure_review_response_v1.schema.json'),  # noqa: E501
}
SUBJECT_SCHEMAS = {
    'fresh_holdout_authorization': 'competitive_fresh_holdout_authorization_v1',  # noqa: E501
    'machine_identity': 'competitive_execution_machine_identity_pop_v1',
}


class RuntimeClosureReviewError(base.ClosureReviewError):
    """Malformed, stale, unsigned, or unsafe runtime closure review."""


class RuntimeClosureReviewNotReady(RuntimeClosureReviewError):
    """The fixed policy or authoritative subject is not ready."""


def _canonical(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def _hash(value: Any) -> str:
    return hashlib.sha256(
        value if isinstance(value, bytes) else _canonical(value)).hexdigest()


def _verifier() -> dict[str, Any]:
    import cryptography

    data, _identity = base._read(MODULE, 'runtime closure verifier', mode=None)
    digest = _hash(data)
    return {
        'path': MODULE_REL, 'file_sha256': digest,
        'implementation': 'competitive_execution_r3_runtime_closure_review_v1',
        'backend': {
            'name': BACKEND_NAME,
            'version': getattr(cryptography, '__version__', ''),
            'implementation': BACKEND_IMPLEMENTATION,
            'implementation_file': MODULE_REL,
            'implementation_sha256': digest,
        },
    }


def _sidecar(path: Path, data: bytes, canonical_sha: str) -> None:
    side_path = path.with_name(path.name + '.sha256.json')
    side, _side_data, _side_identity = base._json(
        side_path, f'{path.name} sidecar', limit=MAX_SIDE_BYTES, mode=0o444)
    expected = {
        'schema': SIDE_SCHEMA, 'schema_version': 1,
        'artifact_path': path.name, 'artifact_bytes': len(data),
        'artifact_sha256': _hash(data), 'canonical_sha256': canonical_sha,
    }
    if side != expected:
        raise RuntimeClosureReviewError(f'{path.name} sidecar binding drift')


def _policy(path: Path, kind: str, candidate: Mapping[str, Any],
            profile: Mapping[str, Any], now: int) -> tuple[dict[str, Any], dict[str, Any]]:  # noqa: E501
    path = base._absolute(str(path), f'{kind} trust policy')
    fixed = ROOT / TRUST_POLICY_RELS[kind]
    if path == fixed:
        raise RuntimeClosureReviewNotReady(
            f'{kind} checked-in trust policy is NOT_READY')
    policy, data, _identity = base._json(
        path, f'{kind} trust policy', mode=0o444)
    identity = policy.get('policy_identity_sha256')
    expected = {key: value for key, value in policy.items()
                if key != 'policy_identity_sha256'}
    if (policy.get('schema') != TRUST_SCHEMAS[kind] or
            policy.get('schema_version') != 1 or
            identity != _hash(expected)):
        raise RuntimeClosureReviewError(  # noqa: E501
            f'{kind} trust policy identity is invalid')
    _sidecar(path, data, identity)
    fields = {
        'schema', 'schema_version', 'status', 'benchmark_eligible', 'campaign_id',  # noqa: E501
        'closure_kind', 'candidate_binding', 'profile_binding',
        'response_schema_binding', 'verifier', 'authorized_keys', 'nonce_policy',  # noqa: E501
        'policy_identity_sha256'}
    if (set(policy) != fields or policy['status'] != 'READY' or
            policy['benchmark_eligible'] is not False or
            policy['campaign_id'] != CAMPAIGN_ID or
            policy['closure_kind'] != kind):
        raise RuntimeClosureReviewNotReady(
            f'{kind} trust policy is not strict READY')
    if (policy['candidate_binding'] != candidate or
            policy['profile_binding'] != profile):
        raise RuntimeClosureReviewError(f'{kind} candidate/profile drift')
    if policy['verifier'] != _verifier():
        raise RuntimeClosureReviewError(f'{kind} verifier policy drift')
    schema_data, _schema_id = base._read(
        ROOT / SCHEMA_RELS[kind], f'{kind} response schema', mode=None)
    if policy['response_schema_binding'] != {
            'path': SCHEMA_RELS[kind], 'file_sha256': _hash(schema_data),
            'schema': SCHEMAS[kind]}:
        raise RuntimeClosureReviewError(f'{kind} response schema drift')
    if policy['nonce_policy'] != {
            'minimum_length': 32, 'maximum_length': 128,
            'one_shot_ledger_required': True}:
        raise RuntimeClosureReviewError(f'{kind} nonce policy drift')
    keys = policy['authorized_keys']
    if not isinstance(keys, list) or len(keys) != 1:
        raise RuntimeClosureReviewNotReady(
            f'{kind} trust policy has no single active key')
    key = keys[0]
    required = {'key_id', 'algorithm', 'status', 'public_key_base64',
                'public_key_sha256', 'not_before', 'not_after'}
    if (not isinstance(key, dict) or set(key) != required or
            key['algorithm'] != ALGORITHM or key['status'] != 'ACTIVE'):
        raise RuntimeClosureReviewError(f'{kind} trust key shape is invalid')
    if (not isinstance(key.get('key_id'), str) or
            not 1 <= len(key['key_id']) <= 128 or
            not key['key_id'].isascii()):
        raise RuntimeClosureReviewError(f'{kind} trust key id is invalid')
    if (type(key['not_before']) is not int or type(key['not_after']) is not int or  # noqa: E501
            key['not_before'] < 0 or key['not_after'] <= key['not_before'] or
            now < key['not_before'] or now >= key['not_after']):
        raise RuntimeClosureReviewError(  # noqa: E501
            f'{kind} trust key is outside validity')
    if not isinstance(key.get('public_key_base64'), str):
        raise RuntimeClosureReviewError(
            f'{kind} trust key encoding is invalid')
    try:
        import base64
        raw = base64.b64decode(key['public_key_base64'].encode('ascii'),
                               validate=True)
    except (ValueError, UnicodeError) as error:
        raise RuntimeClosureReviewError(
            f'{kind} trust key is invalid base64') from error
    if len(raw) != 32 or _hash(raw) != key['public_key_sha256']:
        raise RuntimeClosureReviewError(f'{kind} trust key hash drift')
    return policy, {
        'policy': policy, 'key': key, 'raw': raw,
        'path': str(path), 'file_bytes': len(data),
        'file_sha256': _hash(data), 'canonical_sha256': identity,
    }


def _subject(path: Path, binding: Mapping[str, Any], kind: str,
             candidate: Mapping[str, Any], profile: Mapping[str, Any],
             review_time: int) -> tuple[dict[str, Any], str]:
    required = {'path', 'file_bytes', 'file_sha256', 'canonical_sha256', 'schema'}  # noqa: E501
    if (set(binding) != required or
            binding.get('schema') != SUBJECT_SCHEMAS[kind]):
        raise RuntimeClosureReviewError(f'{kind} subject binding is malformed')
    if (type(binding.get('file_bytes')) is not int or
            not 1 <= binding['file_bytes'] <= MAX_BYTES or
            not isinstance(binding.get('file_sha256'), str) or
            not isinstance(binding.get('canonical_sha256'), str)):
        raise RuntimeClosureReviewError(
            f'{kind} subject binding values are invalid')
    data, _identity = base._read(path, f'{kind} subject', mode=0o444)
    if (binding['file_bytes'] != len(data) or
            binding['file_sha256'] != _hash(data)):
        raise RuntimeClosureReviewError(f'{kind} subject file binding drift')
    try:
        document = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise RuntimeClosureReviewError(  # noqa: E501
            f'{kind} subject is invalid JSON') from error
    if (not isinstance(document, dict) or
            binding['canonical_sha256'] != _hash(document)):
        raise RuntimeClosureReviewError(f'{kind} subject canonical drift')
    if kind == 'fresh_holdout_authorization':
        _validate_holdout(document, profile, review_time)
    else:
        _validate_machine(path, document, binding, review_time)
    after_data, _after_identity = base._read(
        path, f'{kind} subject recheck', mode=0o444)
    if after_data != data:
        raise RuntimeClosureReviewError(
            f'{kind} subject changed during validation')
    return document, binding['canonical_sha256']


def _validate_holdout(document: Mapping[str, Any], profile: Mapping[str, Any],
                      review_time: int) -> None:
    try:
        import yaml
        profile_data, _profile_id = base._read(
            ROOT / profile['path'], 'competitive profile', mode=None)
        profile_document = yaml.safe_load(profile_data.decode('utf-8'))
    except (ImportError, OSError, UnicodeError, ValueError, TypeError) as error:  # noqa: E501
        raise RuntimeClosureReviewError(
            f'holdout profile cannot be reopened: {error}') from error
    contract = (  # noqa: E501
        profile_document.get('competitive_slam_profile', profile_document)
        if isinstance(profile_document, Mapping) else None)
    policy = contract.get('fresh_holdout_authorization') \
        if isinstance(contract, Mapping) else None
    result = holdout.verify_fresh_holdout_authorization(
        document, policy=policy, profile=profile_document,
        expected_profile_sha256=profile['canonical_sha256'],
        now_utc=datetime.fromtimestamp(review_time, timezone.utc))
    attestation = result.get('external_attestation_signature')
    if (result.get('status') != 'PASS' or result.get('pass') is not True or
            result.get('external_attestation_status') != 'PASS' or
            not isinstance(attestation, Mapping) or
            attestation.get('status') != 'PASS' or
            attestation.get('verified') is not True):
        raise RuntimeClosureReviewNotReady(
            'fresh holdout authoritative authorization is not signed PASS')


def _validate_machine(path: Path, document: Mapping[str, Any],
                      binding: Mapping[str, Any], review_time: int) -> None:
    trust = document.get('trust_policy')
    if (not isinstance(trust, Mapping) or
            not isinstance(trust.get('path'), str)):
        raise RuntimeClosureReviewError(
            'machine subject trust policy is missing')
    policy_path = Path(trust['path'])
    if not policy_path.is_absolute():
        policy_path = ROOT / policy_path
    result = machine_identity.validate_artifact(
        path, live=True, _policy_path=policy_path,
        _allow_external_policy=True, _now=review_time)
    if (result.get('status') != 'PASS' or
            result.get('live_host_match') is not True or
            result.get('identity_sha256') != document.get('identity_sha256') or
            result.get('artifact_sha256') != binding.get('file_sha256')):
        raise RuntimeClosureReviewNotReady(
            'machine subject authoritative live PoP validation did not PASS')


def _response_result(kind: str, response: Mapping[str, Any]) -> dict[str, Any]:
    return {
        'status': 'PASS', 'verified': True, 'campaign_id': CAMPAIGN_ID,
        'candidate_identity_sha256': response['candidate_binding'][
            'candidate_identity_sha256'],
        'profile_canonical_sha256': response['profile_binding'][
            'canonical_sha256'],
        'closure_kind': kind,
        'response_identity_sha256': response['response_identity_sha256'],
        'signature_valid': True, 'replay_protected': True,
        'benchmark_eligible': False, 'claim_eligible': False,
        'active_profile_switch': False, 'promotion_allowed': False,
        'offline_only': True,
    }


def _validate_response(response_path: Path, *, kind: str,
                       replay_ledger: Path, policy_path: Path | None,
                       review_time: int | None) -> dict[str, Any]:
    if kind not in KINDS:
        raise RuntimeClosureReviewError('unknown runtime closure kind')
    now = int(__import__('time').time()) if review_time is None else review_time  # noqa: E501
    if type(now) is not int or now < 0:
        raise RuntimeClosureReviewError(  # noqa: E501
            'runtime closure review time is invalid')
    response_path = base._absolute(str(response_path), f'{kind} response')
    response, data, _identity = base._json(
        response_path, f'{kind} response', mode=0o444)
    identity = response.get('response_identity_sha256')
    expected = {key: value for key, value in response.items()
                if key != 'response_identity_sha256'}
    if (response.get('schema') != SCHEMAS[kind] or
            response.get('schema_version') != 1 or identity != _hash(expected)):  # noqa: E501
        raise RuntimeClosureReviewError(f'{kind} response identity is invalid')
    _sidecar(response_path, data, identity)
    required = {
        'schema', 'schema_version', 'decision_status', 'review_status',
        'benchmark_eligible', 'claim_eligible', 'active_profile_switch',
        'promotion_allowed', 'campaign_id', 'closure_kind', 'nonce',
        'issued_at', 'expires_at', 'candidate_binding', 'profile_binding',
        'subject_binding', 'review_scope', 'trust_policy', 'verifier',
        'signature', 'promotion_policy', 'response_identity_sha256'}
    if (set(response) != required or response['decision_status'] != 'ACCEPTED' or  # noqa: E501
            response['review_status'] != 'REVIEWED_EXTERNAL' or
            response['benchmark_eligible'] is not False or
            response['claim_eligible'] is not False or
            response['active_profile_switch'] is not False or
            response['promotion_allowed'] is not False or
            response['campaign_id'] != CAMPAIGN_ID or
            response['closure_kind'] != kind):
        raise RuntimeClosureReviewError(f'{kind} response status is invalid')
    if (not isinstance(response.get('nonce'), str) or
            base.NONCE_RE.fullmatch(response['nonce']) is None):
        raise RuntimeClosureReviewError(f'{kind} response nonce is invalid')
    if (type(response['issued_at']) is not int or
            type(response['expires_at']) is not int or
            response['issued_at'] < 0 or response['expires_at'] <= response['issued_at'] or  # noqa: E501
            response['expires_at'] - response['issued_at'] > MAX_WINDOW or
            now < response['issued_at'] or now >= response['expires_at']):
        raise RuntimeClosureReviewError(f'{kind} response validity is invalid')
    candidate, profile = base._candidate_profile()
    if (response['candidate_binding'] != candidate or
            response['profile_binding'] != profile):
        raise RuntimeClosureReviewError(f'{kind} candidate/profile drift')
    if response['review_scope'] != {
            'closure_kind': kind, 'candidate_id': candidate['candidate_id'],
            'profile_canonical_sha256': profile['canonical_sha256'],
            'decision': 'ACCEPTED', 'offline_only': True}:
        raise RuntimeClosureReviewError(f'{kind} review scope drift')
    subject_path = base._absolute(
        response['subject_binding']['path'], f'{kind} subject')
    _subject(subject_path, response['subject_binding'], kind, candidate, profile, now)  # noqa: E501
    configured_policy = _absolute_policy(
        response['trust_policy'], kind, policy_path)
    policy, state = _policy(configured_policy, kind, candidate, profile, now)
    policy_ref = {
        'path': state['path'], 'file_bytes': state['file_bytes'],
        'file_sha256': state['file_sha256'],
        'canonical_sha256': state['canonical_sha256'],
        'schema': TRUST_SCHEMAS[kind]}
    if response['trust_policy'] != policy_ref:
        raise RuntimeClosureReviewError(  # noqa: E501
            f'{kind} trust policy descriptor drift')
    if response['verifier'] != _verifier():
        raise RuntimeClosureReviewError(f'{kind} verifier drift')
    base._signature(response, {**state, 'policy': policy})
    base._claim(base._absolute(str(replay_ledger), 'runtime replay ledger'),
                response['nonce'], identity, kind)
    after, after_id = base._read(
        response_path, f'{kind} response recheck', mode=0o444)
    if (after != data or
            after_id != base._identity(  # noqa: E501
                response_path, f'{kind} response recheck')):
        raise RuntimeClosureReviewError(  # noqa: E501
            f'{kind} response changed during validation')
    return _response_result(kind, response)


def _absolute_policy(response_ref: Mapping[str, Any], kind: str,
                     supplied: Path | None) -> Path:
    required = {'path', 'file_bytes', 'file_sha256', 'canonical_sha256', 'schema'}  # noqa: E501
    if not isinstance(response_ref, Mapping) or set(response_ref) != required:
        raise RuntimeClosureReviewError(  # noqa: E501
            f'{kind} trust policy reference is malformed')
    path = base._absolute(response_ref['path'], f'{kind} trust policy')
    if supplied is not None and path != base._absolute(str(supplied), 'trust policy'):  # noqa: E501
        raise RuntimeClosureReviewError(f'{kind} trust policy path drift')
    return path


def validate_holdout_response(response_path: Path, *, replay_ledger: Path,
                              policy_path: Path | None = None,
                              review_time: int | None = None) -> dict[str, Any]:  # noqa: E501
    """Validate a signed fresh-holdout closure review."""
    return _validate_response(
        response_path, kind='fresh_holdout_authorization',
        replay_ledger=replay_ledger, policy_path=policy_path,
        review_time=review_time)


def validate_machine_response(response_path: Path, *, replay_ledger: Path,
                              policy_path: Path | None = None,
                              review_time: int | None = None) -> dict[str, Any]:  # noqa: E501
    """Validate a signed live-machine closure review."""
    return _validate_response(
        response_path, kind='machine_identity', replay_ledger=replay_ledger,
        policy_path=policy_path, review_time=review_time)


def _main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind', choices=KINDS, required=True)
    parser.add_argument('--response', type=Path, required=True)
    parser.add_argument('--replay-ledger', type=Path, required=True)
    parser.add_argument('--policy', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = _validate_response(
            args.response, kind=args.kind, replay_ledger=args.replay_ledger,
            policy_path=args.policy, review_time=None)
    except (RuntimeClosureReviewError, OSError, ValueError,
            TypeError) as error:
        status = (
            'NOT_READY' if isinstance(error, RuntimeClosureReviewNotReady)
            else 'INVALID')
        print(json.dumps({
            'status': status, 'benchmark_eligible': False,
            'claim_eligible': False, 'active_profile_switch': False,
            'promotion_allowed': False, 'error': str(error)}, sort_keys=True))
        return 3 if status == 'NOT_READY' else 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(_main())
