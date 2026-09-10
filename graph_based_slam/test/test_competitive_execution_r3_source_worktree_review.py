#!/usr/bin/env python3
"""Adversarial tests for signed r3 source-worktree review evidence."""

from __future__ import annotations

import base64
import hashlib
import json
import subprocess
from pathlib import Path

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

from jsonschema import Draft7Validator

import pytest

from scripts import (
    capture_competitive_execution_r3_source_worktree as snapshot,
    capture_competitive_execution_source_worktree as source_impl,
    validate_competitive_execution_r3_source_worktree_review as review,
)


REVIEW_TIME = 1_756_128_000
REVISION = 'a' * 40


def _git_fixture(tmp_path: Path) -> Path:
    root = tmp_path / 'checkout'
    root.mkdir()
    subprocess.run(['git', 'init', '--quiet', root], check=True)
    subprocess.run(['git', '-C', str(root), 'config', 'user.email',
                    'fixture@example.invalid'], check=True)
    subprocess.run(['git', '-C', str(root), 'config', 'user.name',
                    'fixture'], check=True)
    (root / 'tracked.txt').write_text('fixture\n', encoding='utf-8')
    subprocess.run(['git', '-C', str(root), 'add', 'tracked.txt'], check=True)
    subprocess.run(['git', '-C', str(root), 'commit', '--quiet', '-m',
                    'fixture'], check=True)
    return root


def _write_immutable(path: Path, data: bytes) -> None:
    path.write_bytes(data)
    path.chmod(0o444)


def _write_pair(path: Path, document: dict, *, schema: str) -> None:
    payload = (json.dumps(document, sort_keys=True, indent=2) + '\n').encode()
    _write_immutable(path, payload)
    side = {
        'schema': schema, 'schema_version': 1,
        'artifact_path': path.name, 'artifact_bytes': len(payload),
        'artifact_sha256': hashlib.sha256(payload).hexdigest(),
        'canonical_sha256': document.get(
            'policy_identity_sha256',
            document.get('response_identity_sha256'),
        ),
    }
    _write_immutable(path.with_name(path.name + '.sha256.json'),
                     review._canonical(side))


@pytest.fixture()
def review_fixture(tmp_path: Path, monkeypatch):
    """Create a synthetic signed response over a temporary Git checkout."""
    root = _git_fixture(tmp_path)
    contract = source_impl._candidate_contract()
    monkeypatch.setattr(source_impl, '_candidate_contract', lambda: contract)
    monkeypatch.setattr(snapshot, '_candidate_contract', lambda: contract)
    snapshot_path = tmp_path / 'snapshot.json'
    snapshot.capture_snapshot(root=root, output=snapshot_path,
                              captured_at='2026-08-26T00:00:00Z')

    private = Ed25519PrivateKey.generate()
    public = private.public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw)
    expected = review._candidate_source_binding()
    policy = {
        'schema': review.TRUST_SCHEMA, 'schema_version': 1,
        'status': 'READY', 'benchmark_eligible': False,
        'campaign_id': review.CAMPAIGN_ID,
        'candidate_binding': expected['candidate'],
        'profile_binding': expected['profile'],
        'source_manifest_identity_sha256': expected[
            'source_manifest_identity_sha256'],
        'response_schema_binding': expected['response_schema_binding'],
        'source_schema_bindings': expected['source_schema_bindings'],
        'verifier': review._verifier(),
        'authorized_keys': [{
            'key_id': 'synthetic-source-review', 'algorithm': 'Ed25519',
            'status': 'ACTIVE',
            'public_key_base64': base64.b64encode(public).decode('ascii'),
            'public_key_sha256': hashlib.sha256(public).hexdigest(),
            'not_before': REVIEW_TIME - 10,
            'not_after': REVIEW_TIME + 10000,
        }],
        'nonce_policy': {
            'minimum_length': 32, 'maximum_length': 128,
            'one_shot_ledger_required': True,
        },
        'policy_identity_sha256': '',
    }
    policy['policy_identity_sha256'] = review._hash({
        key: value for key, value in policy.items()
        if key != 'policy_identity_sha256'})
    policy_path = tmp_path / 'ready-policy.json'
    _write_pair(policy_path, policy, schema=review.SIDECAR_SCHEMA)

    signature = {
        'algorithm': 'Ed25519', 'key_id': 'synthetic-source-review',
        'public_key_sha256': policy['authorized_keys'][0]['public_key_sha256'],
        'payload_sha256': '0' * 64, 'signature_base64': 'A' * 88,
        'backend': policy['verifier']['backend'],
    }
    response = review.build_response_template(
        snapshot_path=snapshot_path, policy_path=policy_path,
        nonce='n' * 40, issued_at=REVIEW_TIME - 1,
        expires_at=REVIEW_TIME + 1000,
        decision_status=review.ACCEPTED,
        reviewer_note='synthetic fixture only', signature=signature)
    payload = review._payload(response)
    response['signature']['payload_sha256'] = hashlib.sha256(
        payload).hexdigest()
    response['signature']['signature_base64'] = base64.b64encode(
        private.sign(payload)).decode('ascii')
    response['response_identity_sha256'] = review._hash({
        key: value for key, value in response.items()
        if key != 'response_identity_sha256'})
    response_path = tmp_path / 'response.json'
    _write_pair(response_path, response, schema=review.SIDECAR_SCHEMA)
    ledger = tmp_path / 'ledger'
    ledger.mkdir()
    return {
        'root': root, 'snapshot': snapshot_path, 'policy': policy_path,
        'response': response_path,
        'ledger': ledger,
        'response_value': response,
        'contract': contract,
    }


def _rewrite_signed_response(
        path: Path, value: dict,
        private: Ed25519PrivateKey | None = None):
    """Rewrite an adversarial fixture; production files remain immutable."""
    value['response_identity_sha256'] = review._hash({
        key: item for key, item in value.items()
        if key != 'response_identity_sha256'})
    payload = (json.dumps(value, sort_keys=True, indent=2) + '\n').encode()
    path.chmod(0o644)
    path.write_bytes(payload)
    path.chmod(0o444)
    side = path.with_name(path.name + '.sha256.json')
    side.chmod(0o644)
    side.write_bytes(review._canonical({
        'schema': review.SIDECAR_SCHEMA, 'schema_version': 1,
        'artifact_path': path.name, 'artifact_bytes': len(payload),
        'artifact_sha256': hashlib.sha256(payload).hexdigest(),
        'canonical_sha256': value['response_identity_sha256']}))
    side.chmod(0o444)


def test_signed_review_is_non_promoting_and_binds_strict_schema(
        review_fixture):
    """Accept a complete signed review while keeping promotion disabled."""
    value = review.validate_response(
        review_fixture['response'], replay_ledger=review_fixture['ledger'],
        policy_path=review_fixture['policy'], review_time=REVIEW_TIME)
    assert value['signature_valid'] is True
    assert value['promotion_allowed'] is False
    assert value['benchmark_eligible'] is False
    producer = review_fixture['response_value']['snapshot_binding'][
        'snapshot']['source_manifest']['producer_bindings']
    assert producer['snapshot_schema']['path'] == snapshot.SCHEMA_REL
    assert producer['snapshot_strict_schema']['path'] == (
        snapshot.STRICT_SCHEMA_REL)
    assert producer['snapshot_strict_schema']['file_sha256'] == hashlib.sha256(
        (snapshot.ROOT / snapshot.STRICT_SCHEMA_REL).read_bytes()).hexdigest()


def test_checked_in_zero_key_policy_is_not_authorizer(review_fixture):
    """Reject the checked-in zero-key policy as an authorizer."""
    with pytest.raises(
            review.SourceWorktreeReviewNotReady,
            match='NOT_READY|zero keys'):
        review._policy(review.FIXED_POLICY, {}, REVIEW_TIME)


def test_expired_and_wrong_signature_fail_closed(review_fixture):
    """Reject an expired response and a modified detached signature."""
    with pytest.raises(review.SourceWorktreeReviewError, match='validity'):
        review.validate_response(
            review_fixture['response'], replay_ledger=review_fixture['ledger'],
            policy_path=review_fixture['policy'],
            review_time=REVIEW_TIME + 2000)
    value = json.loads(review_fixture['response'].read_text())
    value['signature']['signature_base64'] = 'A' * 88
    _rewrite_signed_response(review_fixture['response'], value)
    with pytest.raises(review.SourceWorktreeReviewError, match='signature'):
        review.validate_response(
            review_fixture['response'], replay_ledger=review_fixture['ledger'],
            policy_path=review_fixture['policy'], review_time=REVIEW_TIME)


def test_nonce_replay_and_partial_review_fail_closed(review_fixture):
    """Reject nonce replay and incomplete review scope."""
    first = review.validate_response(
        review_fixture['response'], replay_ledger=review_fixture['ledger'],
        policy_path=review_fixture['policy'], review_time=REVIEW_TIME)
    assert first['replay_protected'] is True
    with pytest.raises(
            review.SourceWorktreeReviewError,
            match='already been claimed'):
        review.validate_response(
            review_fixture['response'], replay_ledger=review_fixture['ledger'],
            policy_path=review_fixture['policy'], review_time=REVIEW_TIME)

    value = review_fixture['response_value']
    value['review_decision']['scope_complete'] = False
    _rewrite_signed_response(review_fixture['response'], value)
    with pytest.raises(review.SourceWorktreeReviewError,
                       match='signature|response canonical|incomplete'):
        review.validate_response(
            review_fixture['response'], replay_ledger=review_fixture['ledger'],
            policy_path=review_fixture['policy'], review_time=REVIEW_TIME)


def test_strict_schema_binding_drift_is_rejected(review_fixture, monkeypatch):
    """Reject a source-manifest strict-schema hash substitution."""
    candidate, profile, source_manifest = review_fixture['contract']
    altered = json.loads(json.dumps(source_manifest))
    altered['producer_bindings']['snapshot_strict_schema'][
        'file_sha256'] = '0' * 64
    changed = (candidate, profile, altered)
    monkeypatch.setattr(source_impl, '_candidate_contract', lambda: changed)
    monkeypatch.setattr(snapshot, '_candidate_contract', lambda: changed)
    with pytest.raises(review.SourceWorktreeReviewError, match='drift'):
        review.validate_response(
            review_fixture['response'], replay_ledger=review_fixture['ledger'],
            policy_path=review_fixture['policy'], review_time=REVIEW_TIME)


def test_strict_schema_artifact_tamper_is_rejected(review_fixture):
    """Reject a snapshot artifact with a modified strict-schema binding."""
    path = review_fixture['snapshot']
    value = json.loads(path.read_text())
    value['source_manifest']['producer_bindings'][
        'snapshot_strict_schema']['file_sha256'] = '0' * 64
    value['snapshot_identity_sha256'] = review._hash({
        key: item for key, item in value.items()
        if key != 'snapshot_identity_sha256'})
    payload = (json.dumps(value, sort_keys=True, indent=2) + '\n').encode()
    path.chmod(0o644)
    path.write_bytes(payload)
    path.chmod(0o444)
    side = path.with_name(path.name + '.sha256.json')
    side.chmod(0o644)
    side.write_bytes(review._canonical({
        'schema': snapshot.SIDECAR_SCHEMA, 'schema_version': 1,
        'artifact_path': path.name, 'artifact_bytes': len(payload),
        'artifact_sha256': hashlib.sha256(payload).hexdigest(),
        'snapshot_identity_sha256': value['snapshot_identity_sha256']}))
    side.chmod(0o444)
    with pytest.raises(
            source_impl.SourceWorktreeError, match='manifest drift'):
        snapshot.validate_snapshot(
            path, root=review_fixture['root'])


def test_snapshot_path_traversal_is_rejected_before_signature(review_fixture):
    """Reject a traversal path before attempting signature verification."""
    value = json.loads(review_fixture['response'].read_text())
    value['snapshot_binding']['path'] = '/tmp/../escaped-snapshot.json'
    _rewrite_signed_response(review_fixture['response'], value)
    with pytest.raises(review.SourceWorktreeReviewError, match='normalized'):
        review.validate_response(
            review_fixture['response'], replay_ledger=review_fixture['ledger'],
            policy_path=review_fixture['policy'], review_time=REVIEW_TIME)


def test_live_worktree_revalidation_is_required(review_fixture, monkeypatch):
    """Reject a response when live worktree recapture reports drift."""
    def drift(*_args, **_kwargs):
        raise source_impl.SourceWorktreeError('fixture worktree drift')

    monkeypatch.setattr(snapshot, 'validate_snapshot', drift)
    with pytest.raises(review.SourceWorktreeReviewError, match='worktree'):
        review.validate_response(
            review_fixture['response'], replay_ledger=review_fixture['ledger'],
            policy_path=review_fixture['policy'], review_time=REVIEW_TIME)


def test_review_schema_files_are_strict():
    """Ensure all source-worktree review schemas are valid Draft 7 schemas."""
    sidecar_schema = (
        'configs/slam_benchmark_profiles/'
        'competitive_execution_r3_source_worktree_review_'
        'sidecar_v1.schema.json')
    for relative in (review.RESPONSE_SCHEMA_REL, review.TRUST_SCHEMA_REL,
                     sidecar_schema):
        schema = json.loads((review.ROOT / relative).read_text())
        Draft7Validator.check_schema(schema)
