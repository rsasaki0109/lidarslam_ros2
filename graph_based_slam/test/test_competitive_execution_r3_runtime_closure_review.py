"""Adversarial tests for the signed holdout/machine closure adapters."""

from __future__ import annotations

import base64
import json
from pathlib import Path

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

from jsonschema import Draft7Validator

import pytest

from scripts import validate_competitive_execution_r3_acceptance as acceptance
from scripts import validate_competitive_execution_r3_runtime_closure_review as runtime  # noqa: E501


ROOT = runtime.ROOT


def _write(path: Path, value: object, mode: int = 0o444) -> bytes:
    data = (json.dumps(value, sort_keys=True, separators=(',', ':')) + '\n').encode()  # noqa: E501
    path.write_bytes(data)
    path.chmod(mode)
    return data


def _sidecar(path: Path, data: bytes, canonical: str) -> None:
    _write(path.with_name(path.name + '.sha256.json'), {
        'schema': runtime.SIDE_SCHEMA, 'schema_version': 1,
        'artifact_path': path.name, 'artifact_bytes': len(data),
        'artifact_sha256': runtime._hash(data), 'canonical_sha256': canonical,
    })


def _policy(tmp_path: Path, kind: str, candidate: dict, profile: dict,
            key: Ed25519PrivateKey, now: int) -> tuple[Path, dict]:
    public = key.public_key().public_bytes_raw()
    schema = runtime.SCHEMAS[kind]
    schema_rel = runtime.SCHEMA_RELS[kind]
    policy = {
        'schema': runtime.TRUST_SCHEMAS[kind], 'schema_version': 1,
        'status': 'READY', 'benchmark_eligible': False,
        'campaign_id': runtime.CAMPAIGN_ID, 'closure_kind': kind,
        'candidate_binding': candidate, 'profile_binding': profile,
        'response_schema_binding': {
            'path': schema_rel,
            'file_sha256': runtime._hash((ROOT / schema_rel).read_bytes()),
            'schema': schema,
        },
        'verifier': runtime._verifier(),
        'authorized_keys': [{
            'key_id': 'runtime-synthetic-custodian', 'algorithm': 'Ed25519',
            'status': 'ACTIVE',
            'public_key_base64': base64.b64encode(public).decode(),
            'public_key_sha256': runtime._hash(public),
            'not_before': now - 10, 'not_after': now + 10000,
        }],
        'nonce_policy': {
            'minimum_length': 32, 'maximum_length': 128,
            'one_shot_ledger_required': True,
        },
    }
    policy['policy_identity_sha256'] = runtime._hash(policy)
    path = tmp_path / f'{kind}-policy.json'
    data = _write(path, policy)
    _sidecar(path, data, policy['policy_identity_sha256'])
    return path, policy


def _subject(tmp_path: Path, kind: str, policy_path: Path) -> tuple[Path, dict]:  # noqa: E501
    if kind == 'machine_identity':
        document = {
            'schema': 'competitive_execution_machine_identity_pop_v1',
            'identity_sha256': 'a' * 64,
            'trust_policy': {'path': str(policy_path)},
        }
    else:
        document = {
            'schema': 'competitive_fresh_holdout_authorization_v1',
            'authorization': {'status': 'READY'},
        }
    path = tmp_path / f'{kind}-subject.json'
    data = _write(path, document)
    return path, {  # noqa: E501
        'path': str(path), 'file_bytes': len(data),
        'file_sha256': runtime._hash(data),
        'canonical_sha256': runtime._hash(document),
        'schema': runtime.SUBJECT_SCHEMAS[kind],
    }


def _response(tmp_path: Path, kind: str, now: int,
              policy_path: Path, policy: dict, candidate: dict, profile: dict,
              key: Ed25519PrivateKey) -> Path:
    subject_path, subject = _subject(tmp_path, kind, policy_path)
    response = {
        'schema': runtime.SCHEMAS[kind], 'schema_version': 1,
        'decision_status': 'ACCEPTED', 'review_status': 'REVIEWED_EXTERNAL',
        'benchmark_eligible': False, 'claim_eligible': False,
        'active_profile_switch': False, 'promotion_allowed': False,
        'campaign_id': runtime.CAMPAIGN_ID, 'closure_kind': kind,
        'nonce': f'runtime-{kind}-nonce-000000000000000000000000',
        'issued_at': now - 1, 'expires_at': now + 1000,
        'candidate_binding': candidate, 'profile_binding': profile,
        'subject_binding': subject,
        'review_scope': {
            'closure_kind': kind, 'candidate_id': candidate['candidate_id'],
            'profile_canonical_sha256': profile['canonical_sha256'],
            'decision': 'ACCEPTED', 'offline_only': True,
        },
        'trust_policy': {
            'path': str(policy_path), 'file_bytes': policy_path.stat().st_size,
            'file_sha256': runtime._hash(policy_path.read_bytes()),
            'canonical_sha256': policy['policy_identity_sha256'],
            'schema': runtime.TRUST_SCHEMAS[kind],
        },
        'verifier': runtime._verifier(),
        'signature': {
            'algorithm': 'Ed25519', 'key_id': policy['authorized_keys'][0]['key_id'],  # noqa: E501
            'public_key_sha256': policy['authorized_keys'][0]['public_key_sha256'],  # noqa: E501
            'payload_sha256': '', 'signature_base64': '',
            'backend': policy['verifier']['backend'],
        },
        'promotion_policy': {
            'benchmark_eligible': False, 'claim_eligible': False,
            'active_profile_switch': False, 'promotion_allowed': False,
            'offline_only': True,
        },
    }
    payload = runtime.base._payload(response)
    response['signature']['payload_sha256'] = runtime._hash(payload)
    response['signature']['signature_base64'] = base64.b64encode(
        key.sign(payload)).decode()
    response['response_identity_sha256'] = runtime._hash(response)
    path = tmp_path / f'{kind}-response.json'
    data = _write(path, response)
    _sidecar(path, data, response['response_identity_sha256'])
    del subject_path
    return path


@pytest.fixture()
def runtime_fixture(tmp_path: Path):
    """Create two signed responses and their synthetic trust policies."""
    now = 1_800_000_000
    candidate, profile = runtime.base._candidate_profile()
    keys = {kind: Ed25519PrivateKey.generate() for kind in runtime.KINDS}
    policies = {}
    responses = {}
    for kind in runtime.KINDS:
        policies[kind] = _policy(tmp_path, kind, candidate, profile,
                                 keys[kind], now)
        responses[kind] = _response(
            tmp_path, kind, now, policies[kind][0], policies[kind][1],
            candidate, profile, keys[kind])
    ledger = tmp_path / 'replay-ledger'
    ledger.mkdir(mode=0o700)
    return now, candidate, profile, keys, policies, responses, ledger


def test_runtime_wrappers_positive_with_authoritative_subjects(
        runtime_fixture, monkeypatch):
    """Both wrappers call subject validators before returning proof fields."""
    now, candidate, profile, _keys, policies, responses, ledger = runtime_fixture  # noqa: E501

    def holdout_pass(*_args, **_kwargs):
        return {
            'status': 'PASS', 'pass': True,
            'external_attestation_status': 'PASS',
            'external_attestation_signature': {
                'status': 'PASS', 'verified': True,
            },
        }

    def machine_pass(path, **_kwargs):
        document = json.loads(path.read_text())
        return {
            'status': 'PASS', 'live_host_match': True,
            'identity_sha256': document['identity_sha256'],
            'artifact_sha256': runtime._hash(path.read_bytes()),
        }

    monkeypatch.setattr(runtime.holdout,
                        'verify_fresh_holdout_authorization', holdout_pass)
    monkeypatch.setattr(runtime.machine_identity,
                        'validate_artifact', machine_pass)
    for kind in runtime.KINDS:
        result = (runtime.validate_holdout_response if kind ==
                  'fresh_holdout_authorization' else
                  runtime.validate_machine_response)(
            responses[kind], replay_ledger=ledger,
            policy_path=policies[kind][0], review_time=now)
        assert result['signature_valid'] is True
        assert result['replay_protected'] is True
        assert result['closure_kind'] == kind
        assert result['benchmark_eligible'] is False
        assert result['claim_eligible'] is False
    del candidate, profile


def test_runtime_schemas_are_strict_and_well_formed():
    """The two tagged responses and policies expose valid strict schemas."""
    relatives = [
        *runtime.SCHEMA_RELS.values(),
        'configs/slam_benchmark_profiles/'
        'competitive_execution_r3_fresh_holdout_'
        'closure_review_trust_policy_v1.schema.json',
        'configs/slam_benchmark_profiles/'
        'competitive_execution_r3_machine_identity_'
        'closure_review_trust_policy_v1.schema.json',
        'configs/slam_benchmark_profiles/'
        'competitive_execution_r3_runtime_closure_review_'
        'sidecar_v1.schema.json',
    ]
    for relative in relatives:
        schema = json.loads((ROOT / relative).read_text())
        Draft7Validator.check_schema(schema)
        assert schema.get('additionalProperties') is False
    for kind, relative in runtime.TRUST_POLICY_RELS.items():
        suffix = ('fresh_holdout' if kind == 'fresh_holdout_authorization'
                  else 'machine_identity')
        schema_rel = ('configs/slam_benchmark_profiles/'
                      'competitive_execution_r3_' + suffix +
                      '_closure_review_trust_policy_v1.schema.json')
        schema = json.loads((ROOT / schema_rel).read_text())
        policy = json.loads((ROOT / relative).read_text())
        Draft7Validator(schema).validate(policy)


def test_checked_in_runtime_policies_stay_not_ready(runtime_fixture):
    """Checked-in zero-key policies cannot be used as external trust roots."""
    now, candidate, profile, _keys, _policies, _responses, _ledger = (
        runtime_fixture)
    for kind in runtime.KINDS:
        with pytest.raises(runtime.RuntimeClosureReviewNotReady):
            runtime._policy(
                ROOT / runtime.TRUST_POLICY_RELS[kind], kind,
                candidate, profile, now)


def test_runtime_wrapper_replay_and_expiry_fail(runtime_fixture, monkeypatch):
    """Expired responses and one-shot nonce reuse fail closed."""
    now, _candidate, _profile, _keys, policies, responses, ledger = runtime_fixture  # noqa: E501
    monkeypatch.setattr(
        runtime.holdout, 'verify_fresh_holdout_authorization',
        lambda *_args, **_kwargs: {
            'status': 'PASS', 'pass': True,
            'external_attestation_status': 'PASS',
            'external_attestation_signature': {'status': 'PASS', 'verified': True},  # noqa: E501
        })
    with pytest.raises(Exception):
        runtime.validate_holdout_response(
            responses['fresh_holdout_authorization'], replay_ledger=ledger,
            policy_path=policies['fresh_holdout_authorization'][0],
            review_time=now + 2000)
    with pytest.raises(Exception):
        runtime.validate_holdout_response(
            responses['fresh_holdout_authorization'], replay_ledger=ledger,
            policy_path=policies['fresh_holdout_authorization'][0],
            review_time=now - 2)
    runtime.validate_holdout_response(
        responses['fresh_holdout_authorization'], replay_ledger=ledger,
        policy_path=policies['fresh_holdout_authorization'][0], review_time=now)  # noqa: E501
    with pytest.raises(Exception):
        runtime.validate_holdout_response(
            responses['fresh_holdout_authorization'], replay_ledger=ledger,
            policy_path=policies['fresh_holdout_authorization'][0], review_time=now)  # noqa: E501
    monkeypatch.setattr(
        runtime.machine_identity, 'validate_artifact',
        lambda *_args, **_kwargs: {
            'status': 'PASS', 'live_host_match': False,
            'identity_sha256': 'a' * 64,
        })
    with pytest.raises(Exception):
        runtime.validate_machine_response(
            responses['machine_identity'], replay_ledger=ledger,
            policy_path=policies['machine_identity'][0], review_time=now)


def test_runtime_wrapper_wrong_signer_and_subject_tamper(runtime_fixture, monkeypatch):  # noqa: E501
    """Subject tampering is rejected before the signed review is accepted."""
    now, _candidate, _profile, keys, policies, responses, ledger = runtime_fixture  # noqa: E501
    monkeypatch.setattr(
        runtime.holdout, 'verify_fresh_holdout_authorization',
        lambda *_args, **_kwargs: {
            'status': 'PASS', 'pass': True,
            'external_attestation_status': 'PASS',
            'external_attestation_signature': {'status': 'PASS', 'verified': True},  # noqa: E501
        })
    response = responses['fresh_holdout_authorization']
    document = json.loads(response.read_text())
    subject = Path(document['subject_binding']['path'])
    subject.chmod(0o644)
    subject.write_text('{"authorization":{"status":"TAMPERED"}}\n')
    subject.chmod(0o444)
    with pytest.raises(Exception):
        runtime.validate_holdout_response(
            response, replay_ledger=ledger,
            policy_path=policies['fresh_holdout_authorization'][0], review_time=now)  # noqa: E501
    del keys


def test_runtime_wrapper_wrong_signer_rejects(runtime_fixture, monkeypatch):
    """A response metadata key substitution cannot mint a new signature."""
    now, _candidate, _profile, _keys, policies, responses, ledger = runtime_fixture  # noqa: E501
    monkeypatch.setattr(
        runtime.holdout, 'verify_fresh_holdout_authorization',
        lambda *_args, **_kwargs: {
            'status': 'PASS', 'pass': True,
            'external_attestation_status': 'PASS',
            'external_attestation_signature': {
                'status': 'PASS', 'verified': True,
            },
        })
    response = responses['fresh_holdout_authorization']
    document = json.loads(response.read_text())
    document['signature']['key_id'] = 'wrong-custodian'
    document['response_identity_sha256'] = runtime._hash({
        key: value for key, value in document.items()
        if key != 'response_identity_sha256'})
    response.chmod(0o644)
    response.with_name(response.name + '.sha256.json').chmod(0o644)
    data = _write(response, document)
    _sidecar(response, data, document['response_identity_sha256'])
    with pytest.raises(Exception):
        runtime.validate_holdout_response(
            response, replay_ledger=ledger,
            policy_path=policies['fresh_holdout_authorization'][0], review_time=now)  # noqa: E501


def test_cross_kind_and_authoritative_failures_reject(runtime_fixture, monkeypatch):  # noqa: E501
    """Cross-kind replay and failed authoritative subjects are rejected."""
    now, _candidate, _profile, _keys, policies, responses, ledger = runtime_fixture  # noqa: E501
    with pytest.raises(Exception):
        runtime.validate_machine_response(
            responses['fresh_holdout_authorization'], replay_ledger=ledger,
            policy_path=policies['machine_identity'][0], review_time=now)
    monkeypatch.setattr(
        runtime.holdout, 'verify_fresh_holdout_authorization',
        lambda *_args, **_kwargs: {
            'status': 'FAIL_CLOSED', 'pass': False,
            'external_attestation_status': 'FAIL',
            'external_attestation_signature': {'status': 'FAIL', 'verified': False},  # noqa: E501
        })
    with pytest.raises(Exception):
        runtime.validate_holdout_response(
            responses['fresh_holdout_authorization'], replay_ledger=ledger,
            policy_path=policies['fresh_holdout_authorization'][0], review_time=now)  # noqa: E501


def test_aggregator_uses_runtime_builtin_descriptors(runtime_fixture):
    """The acceptance dispatcher points the two slots at the new wrappers."""
    del runtime_fixture
    assert acceptance.BUILTIN_VALIDATOR_SPECS['machine_identity'][0].endswith(
        'validate_competitive_execution_r3_runtime_closure_review.py')
    assert acceptance.BUILTIN_VALIDATOR_SPECS['fresh_holdout_authorization'][0].endswith(  # noqa: E501
        'validate_competitive_execution_r3_runtime_closure_review.py')


def test_aggregator_dispatches_runtime_wrappers(runtime_fixture, monkeypatch):
    """The production aggregator adapters execute both subject validators."""
    now, candidate, profile, _keys, policies, responses, ledger = runtime_fixture  # noqa: E501

    monkeypatch.setattr(
        runtime.holdout, 'verify_fresh_holdout_authorization',
        lambda *_args, **_kwargs: {
            'status': 'PASS', 'pass': True,
            'external_attestation_status': 'PASS',
            'external_attestation_signature': {'status': 'PASS', 'verified': True},  # noqa: E501
        })

    def machine_pass(path, **_kwargs):
        return {
            'status': 'PASS', 'live_host_match': True,
            'identity_sha256': json.loads(path.read_text())['identity_sha256'],  # noqa: E501
            'artifact_sha256': runtime._hash(path.read_bytes()),
        }

    monkeypatch.setattr(runtime.machine_identity,
                        'validate_artifact', machine_pass)
    context = {'candidate': candidate, 'profile': profile,
               'review_time': now, 'replay_ledger': ledger}
    adapters = acceptance._builtin_adapters()
    for kind in runtime.KINDS:
        response_data = json.loads(responses[kind].read_text())
        policy_path, policy = policies[kind]
        result = {
            'status': 'VERIFIED', 'verified': True,
            'campaign_id': acceptance.CAMPAIGN_ID,
            'candidate_identity_sha256': candidate[
                'candidate_identity_sha256'],
            'profile_canonical_sha256': profile['canonical_sha256'],
            'closure_kind': kind,
            'closure_identity_sha256': response_data['response_identity_sha256'],  # noqa: E501
            'signature_valid': True, 'replay_protected': True,
            'benchmark_eligible': False, 'claim_eligible': False,
        }
        raw = responses[kind].read_bytes()
        entry = {
            'status': 'VERIFIED', 'path': str(responses[kind]),
            'file_sha256': acceptance._hash(raw),
            'canonical_sha256': response_data['response_identity_sha256'],
            'validator': acceptance._builtin_validator_descriptor(kind),
            'trust_policy': {
                'path': str(policy_path), 'file_sha256': acceptance._hash(
                    policy_path.read_bytes()),
                'canonical_sha256': policy['policy_identity_sha256'],
            },
            'result': result,
        }
        projection = acceptance._validate_closure(
            kind, entry, context, adapters)
        assert projection['status'] == 'VERIFIED'
