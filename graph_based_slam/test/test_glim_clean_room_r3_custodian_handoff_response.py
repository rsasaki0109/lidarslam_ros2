#!/usr/bin/env python3
"""Adversarial tests for the non-promoting custodian response boundary."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

from jsonschema import Draft202012Validator
import pytest


ROOT = Path(__file__).resolve().parents[2]
R3 = ROOT / 'docker/benchmark_adapters/glim_clean_room/phase3d/r3'


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RESPONSE = _load(R3 / 'apt_release_signature_handoff_response.py',
                 'r3_custodian_handoff_response_test')
AUTH_TEST = _load(
    ROOT / 'graph_based_slam/test/test_glim_clean_room_r3_apt_signature_authorization.py',
    'r3_custodian_response_authorization_fixture')
HANDOFF = RESPONSE.HANDOFF


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _write_json(path: Path, value: object, *, mode: int = 0o600) -> bytes:
    data = RESPONSE.canonical_bytes(value) + b'\n'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    path.chmod(mode)
    return data


def _source_descriptor(path: Path, schema: str) -> dict[str, object]:
    value = json.loads(path.read_text())
    data = path.read_bytes()
    canonical = value.get('canonical_sha256')
    if canonical is None:
        canonical = RESPONSE.AUTH.authorization_payload_sha256(value)
    return {'path': path.name, 'file_bytes': len(data), 'file_sha256': _sha(data),
            'canonical_sha256': canonical, 'schema': schema}


def _seal_response(path: Path, response: dict[str, object]) -> None:
    if path.exists():
        path.chmod(0o600)
    sidecar_path = path.with_name(path.name + '.sha256.json')
    if sidecar_path.exists():
        sidecar_path.chmod(0o600)
    response['canonical_sha256'] = RESPONSE.canonical_hash(response, 'canonical_sha256')
    data = _write_json(path, response, mode=0o444)
    sidecar = {'schema': RESPONSE.SIDECAR_SCHEMA, 'schema_version': 1,
               'artifact_path': path.name, 'artifact_bytes': len(data),
               'artifact_sha256': _sha(data),
               'canonical_sha256': response['canonical_sha256']}
    _write_json(sidecar_path, sidecar, mode=0o444)


def _fixture(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    fixture, plan, auth, private, policy, candidate, _ = AUTH_TEST._fixture(
        tmp_path / 'inputs', monkeypatch)
    policy_root = AUTH_TEST.AUTH.ROOT
    candidate_path = policy_root / 'candidate.json'
    policy_path = policy_root / 'policy.json'

    # Keep the synthetic key valid across the complete review-window boundary
    # tests.  The checked-in policy remains NOT_READY; this is test-only.
    policy['authorized_keys'][0]['not_after'] = 1_800_001_000
    policy['canonical_sha256'] = RESPONSE.canonical_hash(policy, 'canonical_sha256')
    policy_data = _write_json(policy_path, policy)

    # The authorization fixture is intentionally minimal.  Add the candidate
    # manifest schema fields before recomputing the signed source identities.
    candidate['schema'] = 'glim_clean_room_r3_apt_allowlist_candidate_v1'
    candidate['schema_version'] = 1
    candidate['status'] = 'OPT_IN_NOT_READY'
    candidate['authorization_policy']['file_sha256'] = _sha(policy_data)
    candidate['authorization_policy']['canonical_sha256'] = policy['canonical_sha256']
    candidate['canonical_sha256'] = RESPONSE.canonical_hash(candidate, 'canonical_sha256')
    candidate_data = _write_json(candidate_path, candidate)
    auth['candidate']['file_sha256'] = _sha(candidate_data)
    auth['candidate']['canonical_sha256'] = candidate['canonical_sha256']
    auth['policy']['file_sha256'] = _sha(policy_data)
    auth['policy']['canonical_sha256'] = policy['canonical_sha256']
    auth['issued_at'] = 1_800_000_001
    auth['expires_at'] = 1_800_000_500
    AUTH_TEST._resign(auth, private)
    auth_path = policy_root / 'authorization.json'
    auth_data = _write_json(auth_path, auth)

    plan_path = fixture['root'] / 'plan.json'
    _write_json(plan_path, plan)

    candidate_source = _source_descriptor(
        candidate_path, 'glim_clean_room_r3_apt_allowlist_candidate_v1')
    policy_source = _source_descriptor(policy_path, AUTH_TEST.AUTH.POLICY_SCHEMA)
    fixed = {
        'candidate': candidate_source,
        'policy': policy_source,
        'executor': HANDOFF._source(HANDOFF.EXECUTOR, 'signature executor'),
        'authorization_verifier': HANDOFF._source(
            HANDOFF.AUTH.IMPLEMENTATION_PATH, 'authorization verifier'),
    }
    monkeypatch.setattr(HANDOFF, '_fixed_sources', lambda: fixed)
    request_path = tmp_path / 'request.json'
    HANDOFF.build_request(
        output=request_path, plan_path=plan_path,
        machine_artifact=fixture['root'] / 'machine.json',
        root_inputs=fixture['root'], repository=fixture['repository'],
        deb_bindings=fixture['deb_bindings'], campaign_id='test-campaign',
        issued_at=1_800_000_000, expires_at=1_800_000_600, nonce='N' * 64)
    request = json.loads(request_path.read_text())

    candidate_desc = RESPONSE._descriptor(
        candidate_path, 'candidate', schema=candidate_source['schema'])[1]
    policy_desc = RESPONSE._descriptor(
        policy_path, 'policy', schema=policy_source['schema'])[1]
    auth_desc = RESPONSE._descriptor(
        auth_path, 'authorization', schema=auth['schema'])[1]
    response = {
        'schema': RESPONSE.SCHEMA, 'schema_version': 1,
        'status': 'READY_AUTHORIZATION_RESPONSE', 'benchmark_eligible': False,
        'request': RESPONSE._descriptor(
            request_path, 'request', schema=HANDOFF.SCHEMA)[1],
        'candidate': candidate_desc, 'policy': policy_desc, 'authorization': auth_desc,
        'verifier': RESPONSE._verifier_descriptor(),
        'request_binding': RESPONSE._request_binding(request),
        'authorization_binding': {
            'policy_canonical_sha256': policy['canonical_sha256'],
            'candidate_canonical_sha256': candidate['canonical_sha256'],
            'machine_method': 'LEGACY_V1',
            'authorization_issued_at': auth['issued_at'],
            'live_host_match': False,
            'authorization_payload_sha256': auth['signature']['payload_sha256'],
        },
        'machine_validation': 'OFFLINE_ARTIFACT_ONLY', 'canonical_sha256': '',
    }
    response_path = tmp_path / 'response.json'
    _seal_response(response_path, response)
    return {'fixture': fixture, 'plan': plan, 'auth': auth, 'private': private,
            'candidate': candidate, 'policy': policy, 'candidate_path': candidate_path,
            'policy_path': policy_path, 'auth_path': auth_path, 'auth_data': auth_data,
            'request': request, 'request_path': request_path,
            'response': response, 'response_path': response_path}


def test_response_schema_and_offline_positive(tmp_path, monkeypatch):
    case = _fixture(tmp_path, monkeypatch)
    value = json.loads(case['response_path'].read_text())
    Draft202012Validator(
        json.loads((R3 / 'apt_release_signature_handoff_response.schema.json').read_text())
    ).validate(value)
    result = RESPONSE.validate_response(
        case['response_path'], case['request_path'], _review_time=1_800_000_050)
    assert result['status'] == 'PASS'
    assert result['benchmark_eligible'] is False
    assert result['live_host_match'] is False
    assert result['authorization_current'] is True
    assert result['review_time'] == 1_800_000_050


@pytest.mark.parametrize(('review_time', 'passes'), [
    (1_800_000_000, False),  # before issuance
    (1_800_000_001, True),   # exactly at issuance
    (1_800_000_499, True),   # just before expiry
    (1_800_000_500, False),  # exactly at expiry
    (1_800_000_501, False),  # after expiry
])
def test_response_review_uses_verifier_time_window(
        tmp_path, monkeypatch, review_time, passes):
    case = _fixture(tmp_path, monkeypatch)
    if passes:
        result = RESPONSE.validate_response(
            case['response_path'], case['request_path'], _review_time=review_time)
        assert result['review_time'] == review_time
        assert result['authorization_current'] is True
    else:
        with pytest.raises(RESPONSE.ResponseError, match='signed authorization did not verify'):
            RESPONSE.validate_response(
                case['response_path'], case['request_path'], _review_time=review_time)


def test_response_binding_cannot_self_declare_review_time(tmp_path, monkeypatch):
    case = _fixture(tmp_path, monkeypatch)
    value = json.loads(case['response_path'].read_text())
    value['authorization_binding']['authorization_issued_at'] = 1_800_000_050
    _seal_response(case['response_path'], value)
    with pytest.raises(RESPONSE.ResponseError, match='verification binding drift'):
        RESPONSE.validate_response(
            case['response_path'], case['request_path'], _review_time=1_800_000_050)


@pytest.mark.parametrize('mutation', [
    'request', 'candidate', 'policy', 'auth_expiry', 'auth_metadata',
    'binding', 'machine_claim', 'response_symlink', 'sidecar',
])
def test_response_swap_or_scope_tamper_fails_closed(tmp_path, monkeypatch, mutation):
    case = _fixture(tmp_path, monkeypatch)
    response_path = case['response_path']
    if mutation == 'request':
        other = tmp_path / 'other-request.json'
        other.write_bytes(case['request_path'].read_bytes())
        other.chmod(0o444)
        value = json.loads(response_path.read_text())
        value['request']['path'] = str(other)
        _seal_response(response_path, value)
    elif mutation == 'candidate':
        value = json.loads(response_path.read_text())
        value['candidate']['file_sha256'] = 'a' * 64
        _seal_response(response_path, value)
    elif mutation == 'policy':
        value = json.loads(response_path.read_text())
        value['policy']['canonical_sha256'] = 'b' * 64
        _seal_response(response_path, value)
    elif mutation == 'auth_expiry':
        auth = dict(case['auth'])
        auth['expires_at'] = auth['issued_at'] + 8 * 24 * 60 * 60
        AUTH_TEST._resign(auth, case['private'])
        _write_json(case['auth_path'], auth)
    elif mutation == 'auth_metadata':
        auth = dict(case['auth'])
        auth['signature'] = dict(auth['signature'])
        auth['signature']['key_id'] = 'other-key'
        _write_json(case['auth_path'], auth)
    elif mutation == 'binding':
        value = json.loads(response_path.read_text())
        value['authorization_binding']['live_host_match'] = True
        _seal_response(response_path, value)
    elif mutation == 'machine_claim':
        value = json.loads(response_path.read_text())
        value['request_binding']['machine_identity']['validation'] = 'LIVE'
        _seal_response(response_path, value)
    elif mutation == 'response_symlink':
        target = tmp_path / 'response-target.json'
        target.write_bytes(response_path.read_bytes())
        target.chmod(0o444)
        response_path.unlink()
        response_path.symlink_to(target.name)
    else:
        sidecar = response_path.with_name(response_path.name + '.sha256.json')
        sidecar.chmod(0o600)
        sidecar.write_text(sidecar.read_text().replace(
            RESPONSE.SIDECAR_SCHEMA, 'forged-sidecar'))
        sidecar.chmod(0o444)
    with pytest.raises((RESPONSE.ResponseError, OSError, HANDOFF.HandoffError,
                        RESPONSE.AUTH.AuthorizationError)):
        RESPONSE.validate_response(response_path, case['request_path'])


def test_response_requires_immutable_sidecar_and_no_ready_checked_in_policy(tmp_path, monkeypatch):
    case = _fixture(tmp_path, monkeypatch)
    sidecar = case['response_path'].with_name(case['response_path'].name + '.sha256.json')
    sidecar.chmod(0o600)
    with pytest.raises(RESPONSE.ResponseError, match='mutable'):
        RESPONSE.validate_response(case['response_path'], case['request_path'])
    assert RESPONSE.AUTH.verify_fixed_policy_only()['status'] == 'NOT_READY'
