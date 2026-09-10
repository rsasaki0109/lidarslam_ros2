#!/usr/bin/env python3
"""Adversarial tests for the additive competitive execution machine PoP path."""

from __future__ import annotations

import base64
import hashlib
import json
from pathlib import Path
import stat

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from jsonschema import Draft202012Validator
import pytest

import scripts.capture_competitive_execution_machine_identity_pop as pop


ROOT = pop.ROOT
STAMP = 1_000


def _snapshot(machine_id: str = '0123456789abcdef0123456789abcdef'):
    return {
        'raw_identifiers': {
            'machine_id': machine_id,
            'dmi_uuid': '',
            'board_serial': '',
        },
        'public': {
            'architecture': 'x86_64',
            'cpu_model': 'fixture-cpu',
            'logical_cpu_count': 8,
            'memory_total_kb': 4096,
            'kernel_release': 'fixture-kernel',
            'observed_affinity': [0, 1, 2, 3],
        },
    }


def _write_input(path: Path, value: dict) -> bytes:
    data = pop._canonical(value) + b'\n'
    if path.exists():
        path.chmod(0o644)
    path.write_bytes(data)
    path.chmod(0o444)
    sidecar = pop._sidecar(path, data, value, pop.KEY_SIDECAR_SCHEMA)
    sidecar_path = path.with_name(path.name + '.sha256.json')
    if sidecar_path.exists():
        sidecar_path.chmod(0o644)
    sidecar_data = pop._canonical(sidecar) + b'\n'
    sidecar_path.write_bytes(sidecar_data)
    sidecar_path.chmod(0o444)
    return data


def _key(tmp_path: Path, *, key_id: str = 'host-fixture', now: int = STAMP):
    private = Ed25519PrivateKey.generate()  # test-only key, never persisted
    public = private.public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw)
    value = {
        'schema': pop.KEY_SCHEMA,
        'schema_version': 1,
        'status': 'EXTERNALLY_PROVISIONED',
        'campaign_domain': pop.DOMAIN,
        'campaign_id': pop.CAMPAIGN_ID,
        'key_id': key_id,
        'algorithm': pop.ALGORITHM,
        'public_key_base64': base64.b64encode(public).decode('ascii'),
        'public_key_sha256': hashlib.sha256(public).hexdigest(),
        'provisioning_receipt_sha256': 'a' * 64,
        'valid_from': now - 100,
        'valid_until': now + 10_000,
        'canonical_sha256': '',
    }
    value['canonical_sha256'] = pop._canonical_hash(value, 'canonical_sha256')
    path = tmp_path / 'host-key.json'
    _write_input(path, value)
    return private, path, value


def _ready_policy(tmp_path: Path):
    _, info = pop._candidate_snapshot()
    private, key_path, key_value = _key(tmp_path)
    policy = {
        'schema': pop.POLICY_SCHEMA,
        'schema_version': 1,
        'status': 'READY',
        'authorized_runtime': True,
        'benchmark_eligible': False,
        'campaign_domain': pop.DOMAIN,
        'campaign_id': pop.CAMPAIGN_ID,
        'candidate_binding': info['candidate_binding'],
        'profile_binding': info['profile_binding'],
        'backend': pop._backend(),
        'authorized_keys': [{
            'key_id': key_value['key_id'],
            'algorithm': pop.ALGORITHM,
            'public_key_sha256': key_value['public_key_sha256'],
            'status': 'ACTIVE',
            'valid_from': key_value['valid_from'],
            'valid_until': key_value['valid_until'],
        }],
        'verifier_source_sha256': pop._producer()['source_sha256'],
        'canonical_sha256': '',
    }
    policy['canonical_sha256'] = pop._canonical_hash(policy, 'canonical_sha256')
    policy_path = tmp_path / 'policy.json'
    policy_path.write_bytes(pop._canonical(policy) + b'\n')
    policy_path.chmod(0o444)
    return private, key_path, policy_path


def _proof(tmp_path: Path, challenge_path: Path, private: Ed25519PrivateKey):
    challenge = json.loads(challenge_path.read_text())
    signature = private.sign(pop._challenge_payload(challenge))
    value = {
        'schema': pop.PROOF_SCHEMA,
        'schema_version': 1,
        'status': 'EXTERNAL_SIGNATURE',
        'challenge_path': challenge_path.name,
        'challenge_file_sha256': hashlib.sha256(challenge_path.read_bytes()).hexdigest(),
        'challenge_canonical_sha256': challenge['canonical_sha256'],
        'challenge_payload_sha256': challenge['challenge_payload_sha256'],
        'key_id': challenge['host_public_key']['key_id'],
        'algorithm': pop.ALGORITHM,
        'signature_base64': base64.b64encode(signature).decode('ascii'),
        'signature_sha256': hashlib.sha256(signature).hexdigest(),
        'canonical_sha256': '',
    }
    value['canonical_sha256'] = pop._canonical_hash(value, 'canonical_sha256')
    path = tmp_path / 'proof.json'
    _write_input(path, value)
    return path


def _fixture(tmp_path: Path, *, now: int = STAMP, validity: int = 1000):
    tmp_path.mkdir(parents=True, exist_ok=True)
    ledger = tmp_path / 'replay-ledger'
    ledger.mkdir()
    private, key_path, policy_path = _ready_policy(tmp_path)
    challenge_path = tmp_path / 'challenge.json'
    final_path = tmp_path / 'machine.json'
    result = pop.prepare_challenge(
        challenge_path, key_descriptor=key_path, final_output=final_path,
        replay_ledger=ledger, validity_seconds=validity,
        provider=lambda: _snapshot(), policy_path=policy_path, _now=now)
    proof_path = _proof(tmp_path, challenge_path, private)
    return {
        'private': private, 'key': key_path, 'policy': policy_path,
        'challenge': challenge_path, 'proof': proof_path, 'final': final_path,
        'ledger': ledger, 'result': result,
    }


def test_challenge_binds_candidate_profile_thread_and_hides_raw_machine_id(tmp_path):
    fixture = _fixture(tmp_path)
    challenge = json.loads(fixture['challenge'].read_text())
    assert challenge['status'] == 'CHALLENGE_PENDING'
    assert challenge['candidate_binding']['candidate_id'] == pop.CAMPAIGN_ID
    assert challenge['profile_binding']['path'] == pop.PROFILE_REL
    assert challenge['thread_policy']['required_keys'] == list(pop.THREAD_KEYS)
    assert challenge['trust_policy']['status'] == 'READY'
    assert '0123456789abcdef0123456789abcdef' not in fixture['challenge'].read_text()
    assert stat.S_IMODE(fixture['challenge'].stat().st_mode) == 0o444
    assert stat.S_IMODE(fixture['challenge'].with_name(
        'challenge.json.sha256.json').stat().st_mode) == 0o444
    assert pop.validate_challenge(fixture['challenge'])['proof_required'] is True


def test_external_proof_finalizes_and_offline_reopen_never_claims_host(tmp_path):
    fixture = _fixture(tmp_path)
    result = pop.finalize_challenge(
        fixture['final'], challenge=fixture['challenge'], proof=fixture['proof'],
        provider=lambda: _snapshot(), _policy_path=fixture['policy'],
        _allow_test_policy=True, _now=STAMP + 1, _status='SYNTHETIC_TEST')
    assert result['benchmark_eligible'] is False
    assert result['promotion_allowed'] is False
    offline = pop.validate_artifact(
        fixture['final'], _policy_path=fixture['policy'], _allow_synthetic=True)
    assert offline['live_host_match'] is False
    assert offline['host_match_status'] == 'HOST_MATCH_NOT_CHECKED'
    live = pop.validate_artifact(
        fixture['final'], live=True, provider=lambda: _snapshot(),
        _policy_path=fixture['policy'], _allow_synthetic=True, _now=STAMP + 1)
    assert live['live_host_match'] is True


def test_checked_in_empty_policy_is_not_ready_and_does_not_create_output(tmp_path):
    fixture = _fixture(tmp_path)
    output = fixture['final']
    with pytest.raises(pop.MachinePoPNotReady):
        pop.finalize_challenge(
            output, challenge=fixture['challenge'], proof=fixture['proof'],
            provider=lambda: _snapshot(), _now=STAMP + 1)
    assert not output.exists()
    assert not output.with_name('machine.json.sha256.json').exists()


@pytest.mark.parametrize('now', [STAMP - 1, STAMP + 1000])
def test_actual_validity_window_rejects_before_issue_and_at_expiry(tmp_path, now):
    fixture = _fixture(tmp_path, validity=1000)
    with pytest.raises(pop.MachinePoPError, match='validity window'):
        pop.finalize_challenge(
            fixture['final'], challenge=fixture['challenge'], proof=fixture['proof'],
            provider=lambda: _snapshot(), _policy_path=fixture['policy'],
            _allow_test_policy=True, _now=now, _status='SYNTHETIC_TEST')


def test_nonce_is_one_shot_and_claim_is_immutable(tmp_path):
    fixture = _fixture(tmp_path)
    pop.finalize_challenge(
        fixture['final'], challenge=fixture['challenge'], proof=fixture['proof'],
        provider=lambda: _snapshot(), _policy_path=fixture['policy'],
        _allow_test_policy=True, _now=STAMP + 1, _status='SYNTHETIC_TEST')
    claim = next(fixture['ledger'].glob('*.claim.json'))
    assert stat.S_IMODE(claim.stat().st_mode) == 0o444
    fixture['final'].unlink()
    fixture['final'].with_name('machine.json.sha256.json').unlink()
    with pytest.raises(pop.MachinePoPError, match='nonce'):
        pop.finalize_challenge(
            fixture['final'], challenge=fixture['challenge'], proof=fixture['proof'],
            provider=lambda: _snapshot(), _policy_path=fixture['policy'],
            _allow_test_policy=True, _now=STAMP + 2, _status='SYNTHETIC_TEST')


def test_wrong_host_is_rejected_before_nonce_claim(tmp_path):
    fixture = _fixture(tmp_path)
    with pytest.raises(pop.MachinePoPError, match='current host'):
        pop.finalize_challenge(
            fixture['final'], challenge=fixture['challenge'], proof=fixture['proof'],
            provider=lambda: _snapshot('fedcba9876543210fedcba9876543210'),
            _policy_path=fixture['policy'], _allow_test_policy=True,
            _now=STAMP + 1, _status='SYNTHETIC_TEST')
    assert not list(fixture['ledger'].glob('*.claim.json'))


def test_bad_signature_and_wrong_key_are_rejected(tmp_path):
    fixture = _fixture(tmp_path)
    proof = json.loads(fixture['proof'].read_text())
    proof['signature_base64'] = base64.b64encode(b'x' * 64).decode('ascii')
    proof['signature_sha256'] = hashlib.sha256(b'x' * 64).hexdigest()
    proof['canonical_sha256'] = pop._canonical_hash(proof, 'canonical_sha256')
    _write_input(fixture['proof'], proof)
    with pytest.raises(pop.MachinePoPError, match='signature'):
        pop.finalize_challenge(
            fixture['final'], challenge=fixture['challenge'], proof=fixture['proof'],
            provider=lambda: _snapshot(), _policy_path=fixture['policy'],
            _allow_test_policy=True, _now=STAMP + 1, _status='SYNTHETIC_TEST')


@pytest.mark.parametrize('kind', ['symlink', 'hardlink'])
def test_key_link_replacement_is_rejected(tmp_path, kind):
    fixture = _fixture(tmp_path)
    replacement = tmp_path / 'replacement.json'
    replacement.write_bytes(fixture['key'].read_bytes())
    replacement.chmod(0o444)
    fixture['key'].unlink()
    if kind == 'symlink':
        fixture['key'].symlink_to(replacement.name)
    else:
        fixture['key'].hardlink_to(replacement)
    with pytest.raises(pop.MachinePoPError):
        pop.finalize_challenge(
            fixture['final'], challenge=fixture['challenge'], proof=fixture['proof'],
            provider=lambda: _snapshot(), _policy_path=fixture['policy'],
            _allow_test_policy=True, _now=STAMP + 1, _status='SYNTHETIC_TEST')


def test_cross_campaign_and_candidate_tamper_fail_closed(tmp_path):
    fixture = _fixture(tmp_path)
    challenge = json.loads(fixture['challenge'].read_text())
    challenge['campaign_id'] = 'other-campaign'
    challenge['canonical_sha256'] = pop._canonical_hash(challenge, 'canonical_sha256')
    fixture['challenge'].chmod(0o644)
    fixture['challenge'].write_bytes(pop._canonical(challenge) + b'\n')
    fixture['challenge'].chmod(0o444)
    with pytest.raises(pop.MachinePoPError):
        pop.validate_challenge(fixture['challenge'])


def test_output_collision_and_partial_pair_are_rejected(tmp_path):
    fixture = _fixture(tmp_path)
    fixture['final'].write_text('occupied')
    with pytest.raises(pop.MachinePoPError, match='fresh'):
        pop.finalize_challenge(
            fixture['final'], challenge=fixture['challenge'], proof=fixture['proof'],
            provider=lambda: _snapshot(), _policy_path=fixture['policy'],
            _allow_test_policy=True, _now=STAMP + 1, _status='SYNTHETIC_TEST')


def test_seal_zero_progress_write_fails_closed_without_hanging(tmp_path, monkeypatch):
    output = tmp_path / 'zero-progress.json'

    def no_progress(_fd, _payload):
        return 0

    monkeypatch.setattr(pop.os, 'write', no_progress)
    with pytest.raises(pop.MachinePoPError, match='no progress'):
        pop._write_new(output, b'{"x":1}\n', 'zero-progress fixture')
    assert not output.exists()


def test_seal_rejects_symlink_parent_before_creating_output(tmp_path):
    real = tmp_path / 'real'
    real.mkdir()
    alias = tmp_path / 'alias'
    alias.symlink_to(real, target_is_directory=True)
    with pytest.raises(pop.MachinePoPError):
        pop._write_new(alias / 'output.json', b'{"x":1}\n', 'symlink parent')


def test_schema_files_and_checked_policy_are_strict(tmp_path):
    schema_paths = [
        ROOT / (
            'configs/slam_benchmark_profiles/'
            'competitive_execution_machine_host_public_key_v1.schema.json'),
        ROOT / (
            'configs/slam_benchmark_profiles/'
            'competitive_execution_machine_identity_pop_input_sidecar_v1.schema.json'),
        ROOT / (
            'configs/slam_benchmark_profiles/'
            'competitive_execution_machine_identity_pop_challenge_v1.schema.json'),
        ROOT / (
            'configs/slam_benchmark_profiles/'
            'competitive_execution_machine_identity_pop_proof_v1.schema.json'),
        ROOT / (
            'configs/slam_benchmark_profiles/'
            'competitive_execution_machine_identity_pop_v1.schema.json'),
        ROOT / (
            'configs/slam_benchmark_profiles/'
            'competitive_execution_machine_identity_pop_sidecar_v1.schema.json'),
        ROOT / (
            'configs/slam_benchmark_profiles/'
            'competitive_execution_machine_identity_pop_nonce_claim_v1.schema.json'),
        ROOT / (
            'configs/slam_benchmark_profiles/'
            'competitive_execution_machine_identity_pop_trust_policy_v1.schema.json'),
    ]
    for path in schema_paths:
        Draft202012Validator.check_schema(json.loads(path.read_text()))
    policy = json.loads(pop.TRUST_POLICY_PATH.read_text())
    schema = json.loads(schema_paths[-1].read_text())
    Draft202012Validator(schema).validate(policy)
    assert policy['status'] == 'NOT_READY'
    assert policy['authorized_keys'] == []
    assert policy['authorized_runtime'] is False


def test_direct_cli_help_is_clean_checkout_compatible():
    # Importing the module is the same bootstrap used by direct CLI execution;
    # this assertion keeps the test independent of PYTHONPATH.
    assert pop.MODULE_RELATIVE == 'scripts/capture_competitive_execution_machine_identity_pop.py'
