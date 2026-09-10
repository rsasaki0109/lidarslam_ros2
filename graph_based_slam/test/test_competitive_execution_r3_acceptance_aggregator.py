"""Adversarial tests for the non-promoting r3 acceptance aggregator."""

from __future__ import annotations

import base64
import json
import os
from pathlib import Path

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

import pytest

from scripts import validate_competitive_execution_r3_acceptance as acceptance


ROOT = acceptance.ROOT


def _write(path: Path, value: object, mode: int = 0o444) -> bytes:
    data = (json.dumps(value, sort_keys=True, separators=(',', ':')) + '\n').encode()  # noqa: E501
    path.write_bytes(data)
    path.chmod(mode)
    return data


def _sidecar(path: Path, data: bytes, canonical: str) -> None:
    _write(path.with_name(path.name + '.sha256.json'), {
        'schema': acceptance.SIDECAR_SCHEMA,
        'schema_version': 1,
        'artifact_path': path.name,
        'artifact_bytes': len(data),
        'artifact_sha256': acceptance._hash(data),
        'canonical_sha256': canonical,
    })


def _policy(tmp_path: Path, candidate: dict, profile: dict,
            key: Ed25519PrivateKey, now: int) -> tuple[Path, dict]:
    public = key.public_key().public_bytes_raw()
    verifier = acceptance._verifier()
    policy = {
        'schema': acceptance.TRUST_SCHEMA,
        'schema_version': 1,
        'status': 'READY',
        'benchmark_eligible': False,
        'execution_authorized': True,
        'campaign_id': acceptance.CAMPAIGN_ID,
        'candidate_binding': candidate,
        'profile_binding': profile,
        'response_schema_binding': {
            'path': acceptance.RESPONSE_SCHEMA_REL,
            'file_sha256': acceptance._hash(
                (ROOT / acceptance.RESPONSE_SCHEMA_REL).read_bytes()),
            'schema': acceptance.RESPONSE_SCHEMA,
        },
        'verifier': verifier,
        'authorized_keys': [{
            'key_id': 'synthetic-custodian',
            'algorithm': 'Ed25519',
            'status': 'ACTIVE',
            'public_key_base64': base64.b64encode(public).decode(),
            'public_key_sha256': acceptance._hash(public),
            'not_before': now - 10,
            'not_after': now + 10000,
        }],
        'nonce_policy': {
            'minimum_length': 32, 'maximum_length': 128,
            'one_shot_ledger_required': True,
        },
    }
    policy['policy_identity_sha256'] = acceptance._hash(policy)
    path = tmp_path / 'acceptance-policy.json'
    data = _write(path, policy)
    _sidecar(path, data, policy['policy_identity_sha256'])
    return path, policy


def _response(tmp_path: Path, now: int, policy_path: Path,
              policy: dict, candidate: dict, profile: dict,
              private_key: Ed25519PrivateKey, *, closures=None) -> Path:
    closures = {} if closures is None else closures
    entries = {}
    for name in acceptance.CLOSURES:
        subject = tmp_path / f'{name}.json'
        subject_data = _write(subject, {'closure': name, 'status': 'PASS'})
        subject_sha = acceptance._hash(subject_data)
        result = {
            'status': 'VERIFIED', 'verified': True,
            'campaign_id': acceptance.CAMPAIGN_ID,
            'candidate_identity_sha256': candidate[
                'candidate_identity_sha256'],
            'profile_canonical_sha256': profile['canonical_sha256'],
            'closure_kind': name,
            'closure_identity_sha256': subject_sha,
            'signature_valid': True, 'replay_protected': True,
            'benchmark_eligible': False, 'claim_eligible': False,
        }
        entries[name] = {
            'status': 'VERIFIED',
            'path': str(subject),
            'file_sha256': acceptance._hash(subject_data),
            'canonical_sha256': subject_sha,
            'validator': {
                **acceptance._builtin_validator_descriptor(name),
            },
            'trust_policy': {
                'path': str(policy_path),
                'file_sha256': acceptance._hash(policy_path.read_bytes()),
                'canonical_sha256': policy['policy_identity_sha256'],
            },
            'result': result,
        }
    entries.update(closures)
    response = {
        'schema': acceptance.RESPONSE_SCHEMA,
        'schema_version': 1,
        'status': 'READY_FOR_SEPARATE_PUBLICATION_REVIEW',
        'benchmark_eligible': False, 'claim_eligible': False,
        'active_profile_switch': False, 'promotion_allowed': False,
        'execution_authorized': True, 'publication_authorized': False,
        'sota_claim_authorized': False,
        'requires_separate_publication_review': True,
        'campaign_id': acceptance.CAMPAIGN_ID,
        'nonce': 'synthetic-acceptance-nonce-000000000000000000000000',
        'issued_at': now - 1, 'expires_at': now + 1000,
        'candidate_binding': candidate, 'profile_binding': profile,
        'required_closures': list(acceptance.CLOSURES),
        'closures': entries,
        'trust_policy': {
            'path': str(policy_path), 'file_bytes': policy_path.stat().st_size,
            'file_sha256': acceptance._hash(policy_path.read_bytes()),
            'canonical_sha256': policy['policy_identity_sha256'],
            'schema': acceptance.TRUST_SCHEMA,
        },
        'verifier': acceptance._verifier(),
        'signature': {
            'algorithm': 'Ed25519', 'key_id': 'synthetic-custodian',
            'public_key_sha256': policy['authorized_keys'][0][
                'public_key_sha256'],
            'payload_sha256': '', 'signature_base64': '',
            'backend': policy['verifier']['backend'],
        },
        'promotion_policy': {
            'benchmark_eligible': False, 'claim_eligible': False,
            'active_profile_switch': False, 'promotion_allowed': False,
            'execution_authorized': True, 'publication_authorized': False,
            'sota_claim_authorized': False,
            'requires_separate_publication_review': True,
            'offline_only': True, 'requires_acceptance_aggregator': True,
            'requires_candidate_profile_reseal': True,
        },
    }
    payload = acceptance._payload(response)
    response['signature']['payload_sha256'] = acceptance._hash(payload)
    response['signature']['signature_base64'] = base64.b64encode(
        private_key.sign(payload)).decode()
    response['receipt_identity_sha256'] = acceptance._hash(response)
    path = tmp_path / 'acceptance-response.json'
    data = _write(path, response)
    _sidecar(path, data, response['receipt_identity_sha256'])
    return path


@pytest.fixture()
def synthetic_acceptance(tmp_path: Path):
    """Create a synthetic external policy and seven immutable closure files."""
    now = 1_800_000_000
    candidate, profile = acceptance._candidate_profile()
    key = Ed25519PrivateKey.generate()
    policy_path, policy = _policy(tmp_path, candidate, profile, key, now)
    response = _response(tmp_path, now, policy_path, policy, candidate,
                         profile, key)
    ledger = tmp_path / 'replay-ledger'
    ledger.mkdir(mode=0o700)
    return now, candidate, profile, key, policy_path, policy, response, ledger


def _synthetic_adapters():
    def adapter(_name, _path, _entry, context):
        return {
            'status': 'PASS', 'verified': True,
            'campaign_id': acceptance.CAMPAIGN_ID,
            'candidate_identity_sha256': context['candidate'][
                'candidate_identity_sha256'],
            'profile_canonical_sha256': context['profile']['canonical_sha256'],
            'closure_kind': _name,
            'closure_identity_sha256': _entry['canonical_sha256'],
            'benchmark_eligible': False, 'claim_eligible': False,
            'signature_valid': True, 'replay_protected': True,
        }

    return {name: adapter for name in acceptance.CLOSURES}


def test_checked_in_policy_is_not_ready(tmp_path: Path):
    """The checked-in keyless policy cannot authorize an aggregate."""
    candidate, profile = acceptance._candidate_profile()
    with pytest.raises(acceptance.AcceptanceNotReady):
        acceptance._policy(acceptance.FIXED_POLICY, candidate, profile, 1_800_000_000)  # noqa: E501


def test_signed_all_closures_are_verified_but_non_promoting(synthetic_acceptance):  # noqa: E501
    """A synthetic externally signed aggregate remains non-promoting."""
    now, _candidate, _profile, _key, _policy_path, _policy, response, ledger = (  # noqa: E501
        synthetic_acceptance)
    result = acceptance.validate_receipt(
        response, replay_ledger=ledger, policy_path=_policy_path,
        review_time=now, _adapters=_synthetic_adapters(), _allow_synthetic=True)  # noqa: E501
    assert result['verified'] is True
    assert result['closure_status'] == {
        name: 'VERIFIED' for name in acceptance.CLOSURES}
    assert result['benchmark_eligible'] is False
    assert result['claim_eligible'] is False
    assert result['active_profile_switch'] is False
    assert result['promotion_allowed'] is False
    assert result['execution_authorized'] is True
    assert result['publication_authorized'] is False
    assert result['sota_claim_authorized'] is False
    assert result['requires_separate_publication_review'] is True


def test_default_path_rejects_unsupported_or_unsigned_replacements(
        synthetic_acceptance):
    """The production adapter path rejects synthetic unsigned closures."""
    now, _candidate, _profile, _key, policy_path, _policy, response, ledger = (
        synthetic_acceptance)
    with pytest.raises((acceptance.AcceptanceNotReady, acceptance.AcceptanceError)):  # noqa: E501
        acceptance.validate_receipt(response, replay_ledger=ledger,
                                    policy_path=policy_path, review_time=now)


def test_cross_type_adapter_is_rejected(synthetic_acceptance):
    """A validator result for another closure kind cannot be substituted."""
    now, _candidate, _profile, _key, policy_path, _policy, response, ledger = (
        synthetic_acceptance)
    adapters = _synthetic_adapters()

    def wrong_kind(_name, _path, _entry, context):
        result = adapters['dataset_source_closure'](_name, _path, _entry, context)  # noqa: E501
        result['closure_kind'] = 'dataset_source_closure'
        return result
    adapters['selection_handoff'] = wrong_kind
    with pytest.raises(acceptance.AcceptanceError):
        acceptance.validate_receipt(response, replay_ledger=ledger,
                                    policy_path=policy_path, review_time=now,
                                    _adapters=adapters, _allow_synthetic=True)


def test_builtin_validator_descriptor_is_fixed(synthetic_acceptance):
    """A rehashed arbitrary validator cannot replace a builtin descriptor."""
    now, candidate, profile, _key, policy_path, _policy, response, ledger = (
        synthetic_acceptance)
    value = json.loads(response.read_text())
    name = 'selection_handoff'
    entry = dict(value['closures'][name])
    entry['validator'] = acceptance._verifier()
    context = {
        'candidate': candidate, 'profile': profile,
        'replay_ledger': ledger,
    }
    adapters = _synthetic_adapters()
    with pytest.raises(acceptance.AcceptanceError):
        acceptance._validate_closure(name, entry, context, adapters)


def test_projection_requires_exact_bindings(synthetic_acceptance):
    """Missing campaign/profile/identity proof is not defaulted or inferred."""
    now, _candidate, _profile, _key, policy_path, _policy, response, ledger = (
        synthetic_acceptance)
    adapters = _synthetic_adapters()
    original_selection = adapters['selection_handoff']

    def missing_binding(_name, _path, _entry, context):
        result = original_selection(
            _name, _path, _entry, context)
        result.pop('profile_canonical_sha256')
        return result

    adapters['selection_handoff'] = missing_binding
    with pytest.raises(acceptance.AcceptanceNotReady):
        acceptance.validate_receipt(
            response, replay_ledger=ledger, policy_path=policy_path,
            review_time=now, _adapters=adapters, _allow_synthetic=True)


def test_holdout_without_signed_replay_validator_is_not_ready():
    """Legacy holdout cannot mint replay protection by inference."""
    with pytest.raises(acceptance.AcceptanceNotReady):
        acceptance._holdout_adapter(
            'fresh_holdout_authorization', Path('/unused'), {}, {})


def test_machine_missing_explicit_proof_is_rejected(synthetic_acceptance):
    """A machine result lacking explicit signed/replay fields cannot pass."""
    now, _candidate, _profile, _key, policy_path, _policy, response, ledger = (
        synthetic_acceptance)
    adapters = _synthetic_adapters()
    original_machine = adapters['machine_identity']

    def fake_machine(_name, _path, _entry, context):
        result = original_machine(
            _name, _path, _entry, context)
        result.pop('signature_valid')
        return result

    adapters['machine_identity'] = fake_machine
    with pytest.raises(acceptance.AcceptanceNotReady):
        acceptance.validate_receipt(
            response, replay_ledger=ledger, policy_path=policy_path,
            review_time=now, _adapters=adapters, _allow_synthetic=True)


def test_replay_and_tamper_fail_closed(synthetic_acceptance):
    """Nonce replay is rejected even when all signed bytes are unchanged."""
    now, _candidate, _profile, _key, policy_path, _policy, response, ledger = (
        synthetic_acceptance)
    adapters = _synthetic_adapters()
    acceptance.validate_receipt(response, replay_ledger=ledger,
                                policy_path=policy_path, review_time=now,
                                _adapters=adapters, _allow_synthetic=True)
    with pytest.raises(acceptance.AcceptanceError):
        acceptance.validate_receipt(response, replay_ledger=ledger,
                                    policy_path=policy_path, review_time=now,
                                    _adapters=adapters, _allow_synthetic=True)


def test_schema_instances_and_cli_help():
    """All new schemas parse and the verifier exposes a clean CLI help path."""
    import subprocess
    for path in sorted((ROOT / 'configs/slam_benchmark_profiles').glob(
            'competitive_execution_selection_r3_*acceptance*schema.json')):
        json.loads(path.read_text())
    result = subprocess.run(
        ['python3', 'scripts/validate_competitive_execution_r3_acceptance.py',
         '--help'], cwd=ROOT, env={**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'},  # noqa: E501
        check=False, capture_output=True, text=True)
    assert result.returncode == 0
