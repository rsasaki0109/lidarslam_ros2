#!/usr/bin/env python3
"""Synthetic-only tests for the signed rival-closure custodian boundary."""

from __future__ import annotations

import base64
import hashlib
import importlib.util
import json
from pathlib import Path

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from jsonschema import Draft202012Validator
import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/validate_competitive_rival_source_closure_r3_custodian_response.py'


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


R = _load(SCRIPT, 'r3_rival_custodian_response_test')


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _write(path: Path, data: bytes, mode: int = 0o444) -> bytes:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    path.chmod(mode)
    return data


def _write_json(path: Path, value: object, mode: int = 0o444) -> bytes:
    return _write(path, R.canonical_bytes(value), mode=mode)


def _descriptor(path: Path, schema: str, identity: str) -> dict[str, object]:
    return R._descriptor(path, path.name, schema=schema,
                         identity_field=identity)[1]


def _policy(tmp_path: Path, candidate: dict[str, object], selection: dict[str, object],
            source: dict[str, object],
            private: Ed25519PrivateKey) -> tuple[Path, dict[str, object]]:
    import cryptography

    public = private.public_key().public_bytes_raw()
    policy_path = tmp_path / 'policy.json'
    policy: dict[str, object] = {
        'schema': R.TRUST_SCHEMA, 'schema_version': 1, 'status': 'READY',
        'benchmark_eligible': False,
        'candidate_binding': {
            'path': R.CANDIDATE_REL,
            'file_sha256': source['candidate']['file_sha256'],
            'canonical_sha256': candidate['candidate_identity_sha256'],
            'candidate_id': candidate['candidate_id'],
        },
        'selection_binding': {
            'path': R.SELECTION_REL,
            'file_sha256': source['selection']['file_sha256'],
            'canonical_sha256': selection['selection_identity_sha256'],
            'selection_id': selection['selection_id'],
        },
        'response_schema_binding': {
            'path': R.RESPONSE_SCHEMA_REL,
            'file_sha256': _sha((ROOT / R.RESPONSE_SCHEMA_REL).read_bytes()),
            'schema': R.RESPONSE_SCHEMA,
        },
        'verifier': {
            'path': R.SCRIPT_REL, 'file_sha256': _sha(SCRIPT.read_bytes()),
            'implementation': 'rival_source_closure_custodian_response_v1',
        },
        'backend': {
            'name': R.BACKEND_NAME, 'version': cryptography.__version__,
            'implementation': R.BACKEND_IMPLEMENTATION,
            'implementation_file': R.SCRIPT_REL,
            'implementation_sha256': _sha(SCRIPT.read_bytes()),
        },
        'authorized_keys': [{
            'key_id': 'synthetic-custodian', 'algorithm': 'Ed25519', 'status': 'ACTIVE',
            'public_key_base64': base64.b64encode(public).decode('ascii'),
            'public_key_sha256': _sha(public), 'not_before': 1_700_000_000,
            'not_after': 2_000_000_000,
        }],
        'nonce_policy': {'minimum_length': 32, 'maximum_length': 128,
                         'one_shot_ledger_required': True},
        'policy_identity_sha256': '',
    }
    policy['policy_identity_sha256'] = R.canonical_hash(
        policy, 'policy_identity_sha256')
    _write_json(policy_path, policy)
    return policy_path, policy


def _decision(tmp_path: Path, blocker: str, index: int) -> dict[str, object]:
    del index
    evidence_path = tmp_path / f'{blocker}.LICENSE'
    evidence = _write(evidence_path,
                      (f'Synthetic immutable license text for {blocker}\\n').encode())
    descriptor = {
        'kind': 'LICENSE_TEXT', 'path': str(evidence_path),
        'file_bytes': len(evidence), 'file_sha256': _sha(evidence),
    }
    component = R.BLOCKER_COMPONENTS[blocker]
    candidate, _, _ = R._load_fixed_candidate()
    _, repository, revision = R._source_identity_for_blocker(candidate, blocker)
    return {
        'blocker_id': blocker, 'decision': 'ACCEPTED',
        'evidence': [descriptor],
        'license_identity': {
            'component': component, 'status': 'ESTABLISHED',
            'upstream_url': repository,
            'revision': revision, 'license_text': descriptor,
            'declaration_is_not_license_text': True,
        },
        'allowed_use': {'build': 'ALLOWED', 'execute': 'ALLOWED'},
        'redistribution_policy': {'source': 'CONDITIONAL', 'binary': 'CONDITIONAL'},
        'publication_policy': {
            'source': 'CONDITIONAL', 'binary': 'CONDITIONAL',
            'benchmark_claim': 'DENIED',
        },
        'review_note': f'Synthetic custodian review for {blocker}.',
    }


def _seal_response(path: Path, response: dict[str, object]) -> None:
    response['response_identity_sha256'] = R.canonical_hash(
        response, 'response_identity_sha256')
    data = _write_json(path, response)
    sidecar = {
        'schema': R.SIDECAR_SCHEMA, 'schema_version': 1,
        'artifact_path': path.name, 'artifact_bytes': len(data),
        'artifact_sha256': _sha(data),
        'canonical_sha256': response['response_identity_sha256'],
    }
    _write_json(path.with_name(path.name + '.sha256.json'), sidecar)


def _sign(response: dict[str, object], private: Ed25519PrivateKey,
          key_id: str = 'synthetic-custodian') -> None:
    public = private.public_key().public_bytes_raw()
    signature = {
        'algorithm': 'Ed25519', 'key_id': key_id,
        'public_key_sha256': _sha(public), 'payload_sha256': '',
        'signature_base64': '',
        'backend': {
            'name': R.BACKEND_NAME, 'version': __import__('cryptography').__version__,
            'implementation': R.BACKEND_IMPLEMENTATION,
            'implementation_file': R.SCRIPT_REL,
            'implementation_sha256': _sha(SCRIPT.read_bytes()),
        },
    }
    response['signature'] = signature
    signature['payload_sha256'] = _sha(R._signature_payload(response))
    signature['signature_base64'] = base64.b64encode(
        private.sign(R._signature_payload(response))).decode('ascii')


def _case(tmp_path: Path) -> dict[str, object]:
    candidate, selection, source = R._load_fixed_candidate()
    handoff_path = tmp_path / 'handoff.json'
    R._HANDOFF.prepare_handoff(output=handoff_path, root=ROOT)
    capture_dir = tmp_path / 'capture'
    R._CAPTURE.capture_legal_provenance(
        output_dir=capture_dir, root=ROOT,
        observed_at='2026-08-26T00:00:00Z')
    capture_path = capture_dir / R._CAPTURE.CAPTURE_FILE
    capture, capture_descriptor, _ = R._descriptor(
        capture_path, 'capture', schema=R.CAPTURE_SCHEMA,
        identity_field='capture_identity_sha256')
    contract = candidate['legal_capture_contract']
    capture_binding = {
        'path': str(capture_path), 'file_bytes': capture_descriptor['file_bytes'],
        'file_sha256': capture_descriptor['file_sha256'],
        'capture_identity_sha256': capture['capture_identity_sha256'],
        'schema': R.CAPTURE_SCHEMA, 'status': capture['status'],
        'review_status': capture['review_status'],
        'remote_responses_status': capture['remote_policy']['responses_status'],
        'closure_identity_sha256': capture['closure_binding']['closure_identity_sha256'],
        'selection_sha256': capture['closure_binding']['selection_sha256'],
        'producer': dict(contract['producer']), 'schema_binding': dict(contract['schema']),
    }
    private = Ed25519PrivateKey.generate()
    policy_path, policy = _policy(tmp_path, candidate, selection, source, private)
    response: dict[str, object] = {
        'schema': R.RESPONSE_SCHEMA, 'schema_version': 1,
        'decision_status': 'ACCEPTED', 'benchmark_eligible': False,
        'claim_eligible': False, 'active_profile_switch': False,
        'campaign_id': R.CAMPAIGN_ID, 'nonce': 'N' * 64,
        'issued_at': 1_800_000_000, 'expires_at': 1_800_000_600,
        'handoff': _descriptor(handoff_path, R.HANDOFF_SCHEMA,
                               'handoff_identity_sha256'),
        'candidate': source['candidate'], 'selection': source['selection'],
        'trust_policy': _descriptor(policy_path, R.TRUST_SCHEMA,
                                    'policy_identity_sha256'),
        'capture': capture_binding, 'r2_lineage': candidate['r2_lineage'],
        'blocker_decisions': [_decision(tmp_path, blocker, i)
                              for i, blocker in enumerate(R.BLOCKERS)],
        'verifier': R._expected_verifier(), 'signature': {},
        'response_identity_sha256': '',
    }
    _sign(response, private)
    response_path = tmp_path / 'response.json'
    _seal_response(response_path, response)
    return {
        'candidate': candidate, 'selection': selection, 'source': source,
        'policy_path': policy_path, 'policy': policy, 'private': private,
        'response': response, 'response_path': response_path,
        'capture_path': capture_path, 'handoff_path': handoff_path,
        'ledger': tmp_path / 'ledger',
    }


def test_schema_and_signed_positive_with_capture_not_run(tmp_path):
    case = _case(tmp_path)
    case['ledger'].mkdir()
    value = json.loads(case['response_path'].read_text())
    Draft202012Validator(json.loads(
        (ROOT / R.RESPONSE_SCHEMA_REL).read_text())).validate(value)
    result = R.validate_response(case['response_path'],
                                 replay_ledger=case['ledger'],
                                 _review_time=1_800_000_100,
                                 _policy_path=case['policy_path'])
    assert result['status'] == 'ACCEPTED'
    assert result['capture_remote_status'] == 'NOT_RUN'
    assert result['benchmark_eligible'] is False
    assert result['active_profile_switch'] is False
    assert result['legal_clearance'] is False


def test_checked_in_empty_trust_policy_is_not_ready(tmp_path):
    case = _case(tmp_path)
    case['ledger'].mkdir()
    with pytest.raises(R.CustodianResponseNotReady, match='no authorized keys'):
        R.validate_response(case['response_path'], replay_ledger=case['ledger'],
                            _review_time=1_800_000_100)


@pytest.mark.parametrize('review_time', [1_799_999_999, 1_800_000_600, 1_800_000_601])
def test_actual_review_window_is_fail_closed(tmp_path, review_time):
    case = _case(tmp_path)
    case['ledger'].mkdir()
    with pytest.raises(R.CustodianResponseError):
        R.validate_response(case['response_path'], replay_ledger=case['ledger'],
                            _review_time=review_time, _policy_path=case['policy_path'])


def test_nonce_replay_is_rejected(tmp_path):
    case = _case(tmp_path)
    case['ledger'].mkdir()
    R.validate_response(case['response_path'], replay_ledger=case['ledger'],
                        _review_time=1_800_000_100, _policy_path=case['policy_path'])
    with pytest.raises(R.CustodianResponseError, match='already been claimed'):
        R.validate_response(case['response_path'], replay_ledger=case['ledger'],
                            _review_time=1_800_000_101, _policy_path=case['policy_path'])


def test_detached_signature_bytes_tamper_is_rejected(tmp_path):
    case = _case(tmp_path)
    case['ledger'].mkdir()
    value = json.loads(case['response_path'].read_text())
    case['response_path'].chmod(0o600)
    case['response_path'].with_name(
        case['response_path'].name + '.sha256.json').chmod(0o600)
    value['signature']['signature_base64'] = base64.b64encode(b'X' * 64).decode('ascii')
    _seal_response(case['response_path'], value)
    with pytest.raises(R.CustodianResponseError, match='signature'):
        R.validate_response(case['response_path'], replay_ledger=case['ledger'],
                            _review_time=1_800_000_100, _policy_path=case['policy_path'])


def test_response_hardlink_is_rejected(tmp_path):
    case = _case(tmp_path)
    case['ledger'].mkdir()
    link = tmp_path / 'response-hardlink.json'
    link.hardlink_to(case['response_path'])
    with pytest.raises(R.CustodianResponseError, match='regular single-link'):
        R.validate_response(link, replay_ledger=case['ledger'],
                            _review_time=1_800_000_100, _policy_path=case['policy_path'])


def test_capture_remote_not_run_cannot_be_relabelled_as_ready(tmp_path):
    case = _case(tmp_path)
    case['ledger'].mkdir()
    value = json.loads(case['response_path'].read_text())
    case['response_path'].chmod(0o600)
    case['response_path'].with_name(
        case['response_path'].name + '.sha256.json').chmod(0o600)
    value['capture']['remote_responses_status'] = 'OBSERVED'
    _sign(value, case['private'])
    _seal_response(case['response_path'], value)
    with pytest.raises(R.CustodianResponseError, match='capture projection'):
        R.validate_response(case['response_path'], replay_ledger=case['ledger'],
                            _review_time=1_800_000_100, _policy_path=case['policy_path'])


def test_blocker_cannot_switch_to_another_upstream_repository(tmp_path):
    case = _case(tmp_path)
    case['ledger'].mkdir()
    value = json.loads(case['response_path'].read_text())
    case['response_path'].chmod(0o600)
    case['response_path'].with_name(
        case['response_path'].name + '.sha256.json').chmod(0o600)
    value['blocker_decisions'][0]['license_identity']['upstream_url'] = (
        'https://github.com/attacker/not-the-pinned-repository.git')
    _sign(value, case['private'])
    _seal_response(case['response_path'], value)
    with pytest.raises(R.CustodianResponseError, match='license identity'):
        R.validate_response(case['response_path'], replay_ledger=case['ledger'],
                            _review_time=1_800_000_100, _policy_path=case['policy_path'])


@pytest.mark.parametrize('mutation', [
    'key', 'candidate', 'capture', 'decision', 'declaration', 'status', 'sidecar', 'mode',
])
def test_signature_scope_and_immutable_artifacts_fail_closed(tmp_path, mutation):
    case = _case(tmp_path)
    case['ledger'].mkdir()
    response_path = case['response_path']
    response_path.chmod(0o600)
    if mutation == 'key':
        value = json.loads(response_path.read_text())
        value['signature']['key_id'] = 'other-key'
        _write_json(response_path, value)
        response_path.chmod(0o444)
    elif mutation == 'candidate':
        value = json.loads(response_path.read_text())
        value['candidate']['file_sha256'] = 'a' * 64
        _write_json(response_path, value)
    elif mutation == 'capture':
        value = json.loads(response_path.read_text())
        value['capture']['closure_identity_sha256'] = 'b' * 64
        _write_json(response_path, value)
    elif mutation == 'decision':
        value = json.loads(response_path.read_text())
        value['blocker_decisions'][1]['decision'] = 'REJECTED'
        _write_json(response_path, value)
    elif mutation == 'declaration':
        value = json.loads(response_path.read_text())
        evidence = value['blocker_decisions'][0]['evidence'][0]
        evidence['path'] = str(tmp_path / 'package.xml')
        _write(tmp_path / 'package.xml', b'<package><license>MIT</license></package>')
        _write_json(response_path, value)
    elif mutation == 'status':
        value = json.loads(response_path.read_text())
        value['decision_status'] = 'REJECTED'
        _write_json(response_path, value)
    elif mutation == 'sidecar':
        sidecar = response_path.with_name(response_path.name + '.sha256.json')
        sidecar.chmod(0o644)
        _write_json(sidecar, json.loads(sidecar.read_text()), mode=0o644)
    else:
        response_path.chmod(0o644)
    with pytest.raises((R.CustodianResponseError, OSError, ValueError)):
        R.validate_response(response_path, replay_ledger=case['ledger'],
                            _review_time=1_800_000_100, _policy_path=case['policy_path'])


def test_partial_blocker_and_license_declaration_are_rejected(tmp_path):
    case = _case(tmp_path)
    case['ledger'].mkdir()
    value = json.loads(case['response_path'].read_text())
    case['response_path'].chmod(0o600)
    value['blocker_decisions'] = value['blocker_decisions'][:3]
    _write_json(case['response_path'], value)
    with pytest.raises(R.CustodianResponseError):
        R.validate_response(case['response_path'], replay_ledger=case['ledger'],
                            _review_time=1_800_000_100, _policy_path=case['policy_path'])


def test_rejected_and_clarification_are_first_class_but_nonpromoting(tmp_path):
    for status in ('REJECTED', 'NEEDS_CLARIFICATION'):
        case = _case(tmp_path / status.lower())
        case['ledger'].mkdir()
        value = json.loads(case['response_path'].read_text())
        case['response_path'].chmod(0o600)
        case['response_path'].with_name(
            case['response_path'].name + '.sha256.json').chmod(0o600)
        value['decision_status'] = status
        for decision in value['blocker_decisions']:
            decision['decision'] = status
            note = tmp_path / status.lower() / f"{decision['blocker_id']}.NOTE"
            note_data = _write(note, b'Custodian review note; legal text unresolved.\\n')
            evidence = {'kind': 'CUSTODIAN_REVIEW_NOTE', 'path': str(note),
                        'file_bytes': len(note_data), 'file_sha256': _sha(note_data)}
            decision['evidence'] = [evidence]
            decision['license_identity']['status'] = (
                'CONFLICT' if status == 'REJECTED' else 'UNRESOLVED')
            decision['license_identity']['license_text'] = None
        _sign(value, case['private'])
        _seal_response(case['response_path'], value)
        result = R.validate_response(case['response_path'], replay_ledger=case['ledger'],
                                     _review_time=1_800_000_100,
                                     _policy_path=case['policy_path'])
        assert result['status'] == status
        assert result['benchmark_eligible'] is False


def test_cross_campaign_handoff_or_capture_is_rejected(tmp_path):
    case = _case(tmp_path)
    case['ledger'].mkdir()
    value = json.loads(case['response_path'].read_text())
    case['response_path'].chmod(0o600)
    value['campaign_id'] = 'other-campaign'
    _write_json(case['response_path'], value)
    with pytest.raises(R.CustodianResponseError):
        R.validate_response(case['response_path'], replay_ledger=case['ledger'],
                            _review_time=1_800_000_100, _policy_path=case['policy_path'])
