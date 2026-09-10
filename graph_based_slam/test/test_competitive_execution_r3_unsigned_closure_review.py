"""Tests for signed selection and dataset closure review packets."""

from __future__ import annotations

import base64
import json
from pathlib import Path

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

import pytest

from scripts import validate_competitive_execution_r3_unsigned_closure_review as review  # noqa: E501


def _write(path: Path, value: object) -> bytes:
    data = (json.dumps(value, sort_keys=True, separators=(',', ':')) + '\n').encode()  # noqa: E501
    path.write_bytes(data)
    path.chmod(0o444)
    return data


def _side(path: Path, data: bytes, canonical: str) -> None:
    _write(path.with_name(path.name + '.sha256.json'), {
        'schema': review.SIDECAR_SCHEMA,
        'schema_version': 1,
        'artifact_path': path.name,
        'artifact_bytes': len(data),
        'artifact_sha256': review._hash(data),
        'canonical_sha256': canonical,
    })


def _fixture(tmp_path: Path, kind: str):
    now = 1_800_000_000
    candidate, profile = review._candidate_profile()
    private = Ed25519PrivateKey.generate()
    public = private.public_key().public_bytes_raw()
    verifier = review._verifier()
    schema_data = (review.ROOT / review.SCHEMA_RELS[kind]).read_bytes()
    policy = {
        'schema': review.TRUST_SCHEMAS[kind], 'schema_version': 1,
        'status': 'READY', 'benchmark_eligible': False,
        'campaign_id': review.CAMPAIGN_ID, 'closure_kind': kind,
        'candidate_binding': candidate, 'profile_binding': profile,
        'response_schema_binding': {
            'path': review.SCHEMA_RELS[kind],
            'file_sha256': review._hash(schema_data),
            'schema': review.SCHEMAS[kind],
        },
        'verifier': verifier,
        'authorized_keys': [{
            'key_id': 'synthetic-closure-custodian', 'algorithm': 'Ed25519',
            'status': 'ACTIVE',
            'public_key_base64': base64.b64encode(public).decode(),
            'public_key_sha256': review._hash(public),
            'not_before': now - 1, 'not_after': now + 10000,
        }],
        'nonce_policy': {
            'minimum_length': 32, 'maximum_length': 128,
            'one_shot_ledger_required': True,
        },
    }
    policy['policy_identity_sha256'] = review._hash(policy)
    policy_path = tmp_path / f'{kind}-policy.json'
    policy_data = _write(policy_path, policy)
    _side(policy_path, policy_data, policy['policy_identity_sha256'])
    subject = tmp_path / f'{kind}-subject.json'
    subject_data = _write(subject, {'kind': kind, 'review': 'ACCEPTED'})
    response = {
        'schema': review.SCHEMAS[kind], 'schema_version': 1,
        'decision_status': 'ACCEPTED', 'review_status': 'REVIEWED_EXTERNAL',
        'benchmark_eligible': False, 'claim_eligible': False,
        'active_profile_switch': False, 'promotion_allowed': False,
        'campaign_id': review.CAMPAIGN_ID, 'closure_kind': kind,
        'nonce': 'closure-review-nonce-000000000000000000000000',
        'issued_at': now - 1, 'expires_at': now + 1000,
        'candidate_binding': candidate, 'profile_binding': profile,
        'subject_binding': {
            'path': str(subject), 'file_bytes': len(subject_data),
            'file_sha256': review._hash(subject_data),
            'canonical_sha256': review._hash({'kind': kind, 'review': 'ACCEPTED'}),  # noqa: E501
            'schema': f'synthetic_{kind}_v1',
        },
        'review_scope': {
            'closure_kind': kind, 'candidate_id': candidate['candidate_id'],
            'profile_canonical_sha256': profile['canonical_sha256'],
            'decision': 'ACCEPTED', 'offline_only': True,
        },
        'trust_policy': {
            'path': str(policy_path), 'file_bytes': policy_path.stat().st_size,
            'file_sha256': review._hash(policy_path.read_bytes()),
            'canonical_sha256': policy['policy_identity_sha256'],
            'schema': review.TRUST_SCHEMAS[kind],
        },
        'verifier': verifier,
        'signature': {
            'algorithm': 'Ed25519', 'key_id': 'synthetic-closure-custodian',
            'public_key_sha256': policy['authorized_keys'][0]['public_key_sha256'],  # noqa: E501
            'payload_sha256': '', 'signature_base64': '',
            'backend': verifier['backend'],
        },
        'promotion_policy': {
            'benchmark_eligible': False, 'claim_eligible': False,
            'active_profile_switch': False, 'promotion_allowed': False,
            'offline_only': True, 'requires_acceptance_aggregator': True,
            'requires_candidate_profile_reseal': True,
        },
    }
    payload = review._payload(response)
    response['signature']['payload_sha256'] = review._hash(payload)
    response['signature']['signature_base64'] = base64.b64encode(
        private.sign(payload)).decode()
    response['response_identity_sha256'] = review._hash(response)
    response_path = tmp_path / f'{kind}-response.json'
    response_data = _write(response_path, response)
    _side(response_path, response_data, response['response_identity_sha256'])
    ledger = tmp_path / 'ledger'
    ledger.mkdir(mode=0o700)
    return now, response_path, policy_path, ledger


@pytest.mark.parametrize('kind,entry', [
    ('selection_handoff', 'validate_selection_response'),
    ('dataset_source_closure', 'validate_dataset_response'),
])
def test_signed_closure_review_is_verified_offline(tmp_path: Path, kind: str, entry: str):  # noqa: E501
    """Both dedicated signed closure validators verify offline evidence."""
    now, response, policy, ledger = _fixture(tmp_path, kind)
    result = getattr(review, entry)(response, replay_ledger=ledger,
                                    policy_path=policy, review_time=now)
    assert result['status'] == 'PASS'
    assert result['verified'] is True
    assert result['closure_kind'] == kind
    assert result['offline_only'] is True
    assert result['benchmark_eligible'] is False


def test_checked_in_policies_and_cross_type_are_not_ready(tmp_path: Path):
    """Keyless policies and cross-kind replay remain fail closed."""
    now, response, _policy, ledger = _fixture(tmp_path, 'selection_handoff')
    candidate, profile = review._candidate_profile()
    with pytest.raises(review.ClosureReviewNotReady):
        review._policy(review.ROOT / review.TRUST_POLICY_RELS['selection_handoff'],  # noqa: E501
                       'selection_handoff', candidate, profile, now)
    with pytest.raises(review.ClosureReviewError):
        review.validate_dataset_response(
            response, replay_ledger=ledger,
            policy_path=tmp_path / 'missing-policy.json', review_time=now)


def test_replay_and_tamper_are_rejected(tmp_path: Path):
    """A second validation of the same closure nonce is rejected."""
    now, response, policy, ledger = _fixture(tmp_path, 'dataset_source_closure')  # noqa: E501
    review.validate_dataset_response(response, replay_ledger=ledger,
                                     policy_path=policy, review_time=now)
    with pytest.raises(review.ClosureReviewError):
        review.validate_dataset_response(response, replay_ledger=ledger,
                                         policy_path=policy, review_time=now)
