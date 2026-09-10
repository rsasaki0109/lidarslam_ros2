#!/usr/bin/env python3
"""Focused signed image/toolchain review response tests."""

from __future__ import annotations

import base64
import hashlib
import json
from pathlib import Path
import stat

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from jsonschema import Draft7Validator
import pytest

import scripts.capture_competitive_execution_selection_r3_external as capture
import scripts.validate_competitive_execution_selection_r3_external_review as review


STAMP = '2026-08-26T00:00:00Z'
REVIEW_TIME = 1_756_128_000


def _runner(command):
    if command[1:3] == ['image', 'inspect']:
        digest = command[-1]
        payload = {
            'Id': digest,
            'RepoDigests': ['registry.example/r3@' + digest],
            'RepoTags': [], 'Created': STAMP, 'Architecture': 'amd64',
            'Os': 'linux', 'Size': 1,
        }
        return capture.CommandResult(0, json.dumps(payload).encode())
    return capture.CommandResult(0, b'probe-version\n')


def _capture(tmp_path: Path, *, probe: bool = True) -> Path:
    root = tmp_path / 'capture'
    capture.capture_external(
        output_dir=root, runner=_runner, probe_toolchain=probe,
        observed_at=STAMP)
    return root


def _policy(tmp_path: Path):
    key = Ed25519PrivateKey.generate()
    raw = key.public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw)
    _, binding, context_info = review._candidate()
    schema = review.ROOT / review.RESPONSE_SCHEMA_REL
    schema_sha = hashlib.sha256(schema.read_bytes()).hexdigest()
    policy = {
        'schema': review.TRUST_SCHEMA, 'schema_version': 1,
        'status': 'READY', 'benchmark_eligible': False,
        'candidate_binding': binding,
        'selection_binding': context_info['selection'],
        'capture_producer': review._capture_producer(),
        'response_schema_binding': {
            'path': review.RESPONSE_SCHEMA_REL, 'file_sha256': schema_sha,
            'schema': review.RESPONSE_SCHEMA,
        },
        'verifier': review._verifier(),
        'authorized_keys': [{
            'key_id': 'synthetic-custodian', 'algorithm': 'Ed25519',
            'status': 'ACTIVE',
            'public_key_base64': base64.b64encode(raw).decode('ascii'),
            'public_key_sha256': hashlib.sha256(raw).hexdigest(),
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
    path = tmp_path / 'external-policy.json'
    payload = (json.dumps(policy, indent=2, sort_keys=True) + '\n').encode()
    path.write_bytes(payload)
    path.chmod(0o444)
    sidecar = {
        'schema': review.SIDECAR_SCHEMA, 'schema_version': 1,
        'artifact_path': path.name, 'artifact_bytes': len(payload),
        'artifact_sha256': hashlib.sha256(payload).hexdigest(),
        'canonical_sha256': policy['policy_identity_sha256'],
    }
    side_path = path.with_name(path.name + '.sha256.json')
    side_path.write_bytes((review._canonical(sidecar)))
    side_path.chmod(0o444)
    return key, policy, path


def _signed_response(tmp_path: Path, *, probe: bool = True):
    root = _capture(tmp_path, probe=probe)
    private, policy, policy_path = _policy(tmp_path)
    policy_value = json.loads(policy_path.read_text())
    key_id = policy_value['authorized_keys'][0]['key_id']
    signature = {
        'algorithm': 'Ed25519', 'key_id': key_id,
        'public_key_sha256': policy_value['authorized_keys'][0]['public_key_sha256'],
        'payload_sha256': '0' * 64, 'signature_base64': 'A' * 88,
        'backend': policy_value['verifier']['backend'],
    }
    response = review.build_response_template(
        capture_root=root, policy_path=policy_path,
        nonce='n' * 40, issued_at=REVIEW_TIME - 1,
        expires_at=REVIEW_TIME + 1000, decision_status='ACCEPTED',
        reviewer_note='synthetic fixture only', signature=signature)
    # The template is intentionally unsigned.  The ephemeral signer exists only
    # in this test fixture and is never stored in the response or repository.
    response['signature']['public_key_sha256'] = policy_value[
        'authorized_keys'][0]['public_key_sha256']
    response['signature']['backend'] = policy_value['verifier']['backend']
    payload = review._payload(response)
    response['signature']['payload_sha256'] = hashlib.sha256(payload).hexdigest()
    response['signature']['signature_base64'] = base64.b64encode(
        private.sign(payload)).decode('ascii')
    response['response_identity_sha256'] = review._hash({
        key: value for key, value in response.items()
        if key != 'response_identity_sha256'})
    response_path = tmp_path / 'response.json'
    response_payload = (json.dumps(response, indent=2, sort_keys=True) + '\n').encode()
    response_path.write_bytes(response_payload)
    response_path.chmod(0o444)
    response_side = {
        'schema': review.SIDECAR_SCHEMA, 'schema_version': 1,
        'artifact_path': response_path.name, 'artifact_bytes': len(response_payload),
        'artifact_sha256': hashlib.sha256(response_payload).hexdigest(),
        'canonical_sha256': response['response_identity_sha256'],
    }
    response_path.with_name(response_path.name + '.sha256.json').write_bytes(
        review._canonical(response_side))
    response_path.with_name(response_path.name + '.sha256.json').chmod(0o444)
    return response_path, policy_path, root


def test_complete_signed_review_is_non_promoting_and_binds_all_systems(tmp_path):
    response, policy, root = _signed_response(tmp_path)
    (tmp_path / 'ledger').mkdir()
    result = review.validate_response(
        response, replay_ledger=tmp_path / 'ledger', policy_path=policy,
        review_time=REVIEW_TIME)
    assert result['signature_valid'] is True
    assert result['promotion_allowed'] is False
    assert result['benchmark_eligible'] is False
    assert result['capture_identity_sha256']
    assert set(json.loads((root / 'image_inspect.json').read_text())[
        'systems']) == set(review.SYSTEMS)
    assert {
        'image_inspect.json', 'image_inspect.json.sha256',
        'toolchain_capture.json', 'toolchain_capture.json.sha256',
        'capture_manifest.json', 'capture_manifest.json.sha256'} == {
            path.name for path in root.iterdir()}
    assert stat.S_IMODE(response.stat().st_mode) == 0o444


def test_partial_capture_cannot_receive_signed_acceptance(tmp_path):
    with pytest.raises(review.ExternalReviewError, match='partial|pending|explicitly'):
        _signed_response(tmp_path, probe=False)


def test_checked_in_zero_key_policy_is_not_an_authorizer(tmp_path):
    with pytest.raises(review.ExternalReviewNotReady, match='NOT_READY|zero keys'):
        review._policy(review.FIXED_POLICY, {}, {}, {}, 1)


def test_expired_response_and_wrong_signature_fail_closed(tmp_path):
    response, policy, _ = _signed_response(tmp_path)
    (tmp_path / 'ledger').mkdir()
    with pytest.raises(review.ExternalReviewError, match='validity'):
        review.validate_response(
            response, replay_ledger=tmp_path / 'ledger', policy_path=policy,
            review_time=REVIEW_TIME + 2000)
    document = json.loads(response.read_text())
    document['signature']['signature_base64'] = 'A' * 88
    document['response_identity_sha256'] = review._hash({
        key: value for key, value in document.items()
        if key != 'response_identity_sha256'})
    payload = (json.dumps(document, indent=2, sort_keys=True) + '\n').encode()
    response.chmod(0o644)
    response.write_bytes(payload)
    response.chmod(0o444)
    sidecar = response.with_name(response.name + '.sha256.json')
    sidecar.chmod(0o644)
    sidecar.write_bytes(review._canonical({
        'schema': review.SIDECAR_SCHEMA, 'schema_version': 1,
        'artifact_path': response.name, 'artifact_bytes': len(payload),
        'artifact_sha256': hashlib.sha256(payload).hexdigest(),
        'canonical_sha256': document['response_identity_sha256']}))
    sidecar.chmod(0o444)
    with pytest.raises(review.ExternalReviewError, match='signature'):
        review.validate_response(
            response, replay_ledger=tmp_path / 'ledger', policy_path=policy,
            review_time=REVIEW_TIME)


def test_response_nonce_is_one_shot(tmp_path):
    response, policy, _ = _signed_response(tmp_path)
    ledger = tmp_path / 'ledger'
    ledger.mkdir()
    first = review.validate_response(
        response, replay_ledger=ledger, policy_path=policy,
        review_time=REVIEW_TIME)
    assert first['replay_protected'] is True
    with pytest.raises(review.ExternalReviewError, match='already been claimed'):
        review.validate_response(
            response, replay_ledger=ledger, policy_path=policy,
            review_time=REVIEW_TIME)


def test_response_nonce_path_traversal_is_rejected(tmp_path):
    response, policy, _ = _signed_response(tmp_path)
    document = json.loads(response.read_text())
    document['nonce'] = '../' + 'n' * 38
    document['response_identity_sha256'] = review._hash({
        key: value for key, value in document.items()
        if key != 'response_identity_sha256'})
    payload = (json.dumps(document, indent=2, sort_keys=True) + '\n').encode()
    response.chmod(0o644)
    response.write_bytes(payload)
    response.chmod(0o444)
    sidecar = response.with_name(response.name + '.sha256.json')
    sidecar.chmod(0o644)
    sidecar.write_bytes(review._canonical({
        'schema': review.SIDECAR_SCHEMA, 'schema_version': 1,
        'artifact_path': response.name, 'artifact_bytes': len(payload),
        'artifact_sha256': hashlib.sha256(payload).hexdigest(),
        'canonical_sha256': document['response_identity_sha256']}))
    sidecar.chmod(0o444)
    (tmp_path / 'ledger').mkdir()
    with pytest.raises(review.ExternalReviewError, match='nonce'):
        review.validate_response(
            response, replay_ledger=tmp_path / 'ledger', policy_path=policy,
            review_time=REVIEW_TIME)


def test_capture_file_tamper_is_not_reopened_as_reviewed(tmp_path):
    response, policy, root = _signed_response(tmp_path)
    image = root / 'image_inspect.json'
    image.chmod(0o644)
    image.write_bytes(image.read_bytes() + b'\n')
    image.chmod(0o444)
    (tmp_path / 'ledger').mkdir()
    with pytest.raises(review.ExternalReviewError, match='capture'):
        review.validate_response(
            response, replay_ledger=tmp_path / 'ledger', policy_path=policy,
            review_time=REVIEW_TIME)


def test_capture_sidecar_tamper_is_not_reopened_as_reviewed(tmp_path):
    response, policy, root = _signed_response(tmp_path)
    sidecar = root / 'toolchain_capture.json.sha256'
    sidecar.chmod(0o644)
    sidecar.write_bytes(b'0' * 64 + b'  toolchain_capture.json\n')
    sidecar.chmod(0o444)
    (tmp_path / 'ledger').mkdir()
    with pytest.raises(review.ExternalReviewError, match='capture'):
        review.validate_response(
            response, replay_ledger=tmp_path / 'ledger', policy_path=policy,
            review_time=REVIEW_TIME)


def test_schema_files_are_valid(tmp_path):
    response_path, _, _ = _signed_response(tmp_path)
    for name in (
            'competitive_execution_selection_r3_external_review_response_v1.schema.json',
            'competitive_execution_selection_r3_external_review_sidecar_v1.schema.json',
            'competitive_execution_selection_r3_external_review_trust_policy_v1.schema.json'):
        schema = json.loads((review.ROOT / 'configs/slam_benchmark_profiles' / name).read_text())
        Draft7Validator.check_schema(schema)
    response_schema_path = (
        review.ROOT / 'configs/slam_benchmark_profiles' /
        ('competitive_execution_selection_r3_external_review_response_v1.'
         'schema.json'))
    response_schema = json.loads(response_schema_path.read_text())
    errors = list(Draft7Validator(response_schema).iter_errors(
        json.loads(response_path.read_text())))
    assert errors == []
