# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Synthetic adversarial tests for the fresh-holdout authorization ledger."""

from __future__ import annotations

import base64
import builtins
import copy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path

import cryptography
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
import jsonschema


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'competitive_holdout_authorization.py'
SPEC = importlib.util.spec_from_file_location('holdout_authorization', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


CHAIN_KINDS = [
    'precommit', 'holdout_seal', 'replay_authorization',
    'leakage_audit', 'failure_record']


def _profile():
    slots = {}
    for index in range(1, 4):
        slots[f'fresh_{index}'] = {
            'sequence': f'exp{index:02d}',
            'dataset': f'dataset-{index}',
            'input_manifest_sha256': str(index) * 64,
            'ground_truth_sha256': str(index + 3) * 64,
            'ground_truth_expected_bytes': 100 + index,
            'calibration_archive_sha256': 'a' * 64,
        }
    return {'competitive_slam_profile': {
        'datasets': {'fresh_holdout_slots': slots},
        'rivals': {
            'glim': {'revision': 'b' * 40},
            'fast_livo2': {'revision': 'c' * 40},
        },
    }}


def _policy(attestation_required=False):
    return {
        'required': True,
        'status': 'READY',
        'required_chain_kinds': CHAIN_KINDS,
        'required_systems': ['ours', 'glim', 'fast_livo2'],
        'run_count': 3,
        'release': 'Release',
        'external_attestation': {'required': attestation_required},
    }


def _base_auth():
    profile = _profile()
    systems = {
        'ours': {'revision': 'a' * 40},
        'glim': {'revision': 'b' * 40},
        'fast_livo2': {'revision': 'c' * 40},
    }
    for index, system in enumerate(systems, 1):
        systems[system].update({
            'config_sha256': str(index + 5) * 64,
            'hardware_fingerprint': 'd' * 64,
            'thread_policy_sha256': 'e' * 64,
            'release': 'Release',
        })
    slots = {}
    for slot_id, slot in profile['competitive_slam_profile']['datasets'][
            'fresh_holdout_slots'].items():
        slots[slot_id] = {
            'sequence': slot['sequence'],
            'dataset_id': slot['dataset'],
            'input_manifest_sha256': slot['input_manifest_sha256'],
            'ground_truth_sha256': slot['ground_truth_sha256'],
            'ground_truth_size_bytes': slot['ground_truth_expected_bytes'],
            'calibration_archive_sha256': slot['calibration_archive_sha256'],
        }
    precommit = {
        'dataset_ids': [slot['dataset'] for slot in
                        profile['competitive_slam_profile']['datasets'][
                            'fresh_holdout_slots'].values()],
        'holdout_slots': slots,
        'systems': systems,
        'run_count': 3,
        'scorer': {
            'revision': 'f' * 40,
            'config_sha256': '1' * 64,
            'fingerprint': '2' * 64,
        },
        'metric_gate_sha256': '3' * 64,
        'profile_sha256': '4' * 64,
        'hardware_fingerprint': 'd' * 64,
        'thread_policy_sha256': 'e' * 64,
        'release': 'Release',
    }
    gt_manifest = {
        'identity_only': True,
        'entries': [
            {'slot_id': slot_id, 'sequence': slot['sequence'],
             'ground_truth_sha256': slot['ground_truth_sha256'],
             'ground_truth_size_bytes': slot['ground_truth_size_bytes'],
             'hash_kind': 'opaque_file_sha256_only_no_parser'}
            for slot_id, slot in slots.items()
        ],
    }
    replay = {
        'role': 'replay_only', 'process_id': 'replay-process',
        'gt_mounts': False, 'gt_content_opened': False,
        'scorer_invoked': False,
    }
    scoring = {
        'role': 'scoring_only', 'process_id': 'scoring-process',
        'status': 'PASS', 'authorized_event_count': 1,
        'events': [{'event_id': 'score-1', 'authorized': True,
                    'sealed_bundle_sha256': '5' * 64}],
        'overwrite_or_retry': False,
    }
    leakage = {
        'gt_content_opened': False, 'gt_path_exposed_to_runner': False,
        'prior_gt_access': False, 'development_tuning': False,
        'result_dependent_selection': False, 'reused_holdout': False,
        'gt_available_before_precommit': False,
        'selection_before_gt_access': True,
        'failure_records_complete': True,
    }
    failure_log = {'complete': True, 'events': []}
    auth = {
        'schema_version': 1, 'status': 'READY',
        'receipt_hash_kind': MODULE.RECEIPT_HASH_KIND,
        'precommit': precommit, 'gt_manifest': gt_manifest,
        'replay': replay, 'scoring': scoring,
        'leakage_audit': leakage, 'failure_log': failure_log,
        'external_attestation': {'status': 'NOT_READY'},
    }
    _rechain(auth)
    return auth, profile


def _rechain(auth, timestamps=None):
    payloads = {
        'precommit': auth['precommit'],
        'holdout_seal': auth['gt_manifest'],
        'replay_authorization': auth['replay'],
        'leakage_audit': auth['leakage_audit'],
        'failure_record': auth['failure_log'],
    }
    previous = None
    chain = []
    for index, kind in enumerate(CHAIN_KINDS):
        receipt = {
            'sequence': index + 1,
            'kind': kind,
            'timestamp_utc': (timestamps[index] if timestamps else
                              f'2026-08-24T00:00:0{index}Z'),
            'payload_sha256': MODULE.sha256_value(payloads[kind]),
            'previous_sha256': previous,
        }
        receipt['receipt_sha256'] = MODULE.canonical_authorization_receipt_sha256(
            receipt)
        previous = receipt['receipt_sha256']
        chain.append(receipt)
    auth['receipt_chain'] = chain
    auth['chain_head_sha256'] = previous


def _verify(auth, profile=None, policy=None, document=None, now_utc=None):
    return MODULE.verify_fresh_holdout_authorization(
        document or {'authorization': auth, 'manifest_sha256': '5' * 64},
        policy=policy or _policy(), profile=profile or _profile(),
        expected_profile_sha256='4' * 64, now_utc=now_utc)


def _signed_authorization():
    auth, profile = _base_auth()
    private_key = Ed25519PrivateKey.generate()
    public_key = private_key.public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw)
    key_id = 'custodian-test-1'
    trust_store = {
        'schema_version': 1,
        'keys': [{
            'key_id': key_id,
            'algorithm': 'Ed25519',
            'public_key_base64': base64.b64encode(public_key).decode('ascii'),
            'public_key_sha256': hashlib.sha256(public_key).hexdigest(),
            'status': 'ACTIVE',
            'not_before_utc': '2026-08-01T00:00:00Z',
            'not_after_utc': '2026-09-01T00:00:00Z',
        }],
    }
    trust_store['canonical_sha256'] = MODULE.canonical_trust_store_sha256(
        trust_store)
    policy = _policy(attestation_required=True)
    policy['external_attestation']['trust_store'] = trust_store
    policy['external_attestation']['verification_backend'] = {
        'name': 'python-cryptography',
        'version': cryptography.__version__,
        'implementation': (
            'cryptography.hazmat.primitives.asymmetric.ed25519.'
            'Ed25519PublicKey.verify'),
    }
    attestation = {
        'status': 'PASS',
        'signer_id': 'independent-custodian',
        'algorithm': 'Ed25519',
        'key_id': key_id,
        'public_key_sha256': hashlib.sha256(public_key).hexdigest(),
        'issued_at_utc': '2026-08-24T00:00:00Z',
        'expires_at_utc': '2026-08-25T00:00:00Z',
    }
    auth['external_attestation'] = attestation
    auth['external_attestation_binding_sha256'] = (
        MODULE.canonical_attestation_binding_sha256(attestation))
    payload = MODULE.canonical_attestation_payload(auth)
    attestation['payload_sha256'] = hashlib.sha256(payload).hexdigest()
    attestation['signature'] = base64.b64encode(private_key.sign(payload)).decode('ascii')
    now_utc = datetime(2026, 8, 24, 12, tzinfo=timezone.utc)
    return auth, profile, policy, now_utc


def test_complete_chain_is_locally_hash_verifiable_without_gt_content():
    auth, profile = _base_auth()
    result = _verify(auth, profile)
    assert result['status'] == 'PASS'
    assert result['claim_eligible'] is True


def test_reordered_timestamps_fail_closed():
    auth, profile = _base_auth()
    _rechain(auth, [
        '2026-08-24T00:00:00Z', '2026-08-24T00:00:02Z',
        '2026-08-24T00:00:01Z', '2026-08-24T00:00:03Z',
        '2026-08-24T00:00:04Z'])
    result = _verify(auth, profile)
    assert result['pass'] is False
    assert result['status'] == 'FAIL_CLOSED'
    assert any('timestamps are not strictly increasing' in item
               for item in result['errors'])


def test_duplicate_scoring_event_is_rejected():
    auth, profile = _base_auth()
    auth['scoring']['authorized_event_count'] = 2
    auth['scoring']['events'].append(copy.deepcopy(auth['scoring']['events'][0]))
    result = _verify(auth, profile)
    assert result['pass'] is False
    assert any('exactly one authorized scoring event' in item
               for item in result['errors'])


def test_gt_path_leakage_is_rejected_before_any_content_access():
    auth, profile = _base_auth()
    auth['gt_manifest']['entries'][0]['path'] = 'slots/fresh_1/ground_truth.txt'
    result = _verify(auth, profile)
    assert result['pass'] is False
    assert any('path/content field' in item for item in result['errors'])


def test_revision_and_config_drift_are_bound_to_submitted_manifest():
    auth, profile = _base_auth()
    document = {'authorization': auth, 'manifest_sha256': '5' * 64,
                'revision': {'systems': {'ours': '9' * 40}},
                'profile': {'sha256': '9' * 64}}
    result = _verify(auth, profile, document=document)
    assert result['pass'] is False
    assert any('revision drift' in item for item in result['errors'])
    assert any('config SHA-256 drift' in item for item in result['errors'])


def test_missing_independent_attestation_remains_not_ready():
    auth, profile = _base_auth()
    result = _verify(auth, profile, policy=_policy(attestation_required=True))
    assert result['pass'] is False
    assert result['status'] == 'NOT_READY'
    assert any('independent external custodian attestation is required' in item
               for item in result['not_ready'])


def test_ed25519_attestation_is_verified_against_profile_trust_store():
    auth, profile, policy, now_utc = _signed_authorization()
    result = _verify(auth, profile, policy, now_utc=now_utc)
    assert result['status'] == 'PASS'
    assert result['claim_eligible'] is True
    assert result['external_attestation_signature']['verified'] is True
    assert result['external_attestation_signature']['key_id'] == 'custodian-test-1'
    assert result['external_attestation_signature']['backend']['name'] == (
        'python-cryptography')
    assert result['external_attestation_signature']['backend']['version'] == (
        cryptography.__version__)
    assert result['external_attestation_signature']['trust_store_sha256'] == (
        policy['external_attestation']['trust_store']['canonical_sha256'])


def test_ed25519_signature_binds_all_authorization_fields():
    auth, profile, policy, now_utc = _signed_authorization()
    auth['scoring']['events'][0]['sealed_bundle_sha256'] = '6' * 64
    result = _verify(auth, profile, policy, now_utc=now_utc)
    assert result['status'] == 'FAIL_CLOSED'
    assert any('payload SHA-256 mismatch' in item or
               'signature is invalid' in item for item in result['errors'])


def test_ed25519_attestation_metadata_binding_rejects_expiry_edit():
    auth, profile, policy, now_utc = _signed_authorization()
    auth['external_attestation']['expires_at_utc'] = '2026-08-31T00:00:00Z'
    result = _verify(auth, profile, policy, now_utc=now_utc)
    assert result['status'] == 'FAIL_CLOSED'
    assert any('metadata binding SHA-256 mismatch' in item
               for item in result['errors'])


def test_ed25519_unknown_key_is_rejected_without_self_trust():
    auth, profile, policy, now_utc = _signed_authorization()
    auth['external_attestation']['key_id'] = 'untrusted-key'
    result = _verify(auth, profile, policy, now_utc=now_utc)
    assert result['status'] == 'FAIL_CLOSED'
    assert any('not in the trusted profile store' in item
               for item in result['errors'])


def test_ed25519_revoked_key_is_rejected():
    auth, profile, policy, now_utc = _signed_authorization()
    policy['external_attestation']['trust_store']['keys'][0]['status'] = 'REVOKED'
    store = policy['external_attestation']['trust_store']
    store['canonical_sha256'] = MODULE.canonical_trust_store_sha256(store)
    result = _verify(auth, profile, policy, now_utc=now_utc)
    assert result['status'] == 'FAIL_CLOSED'
    assert any('key is revoked' in item for item in result['errors'])


def test_ed25519_expired_attestation_is_rejected():
    auth, profile, policy, _ = _signed_authorization()
    result = _verify(
        auth, profile, policy,
        now_utc=datetime(2026, 8, 26, tzinfo=timezone.utc))
    assert result['status'] == 'FAIL_CLOSED'
    assert any('expired or not yet valid' in item for item in result['errors'])


def test_ed25519_malformed_signature_is_rejected():
    auth, profile, policy, now_utc = _signed_authorization()
    auth['external_attestation']['signature'] = '%%%'
    result = _verify(auth, profile, policy, now_utc=now_utc)
    assert result['status'] == 'FAIL_CLOSED'
    assert any('signature is not canonical base64' in item
               for item in result['errors'])


def test_missing_profile_trust_store_is_not_ready_not_self_authorized():
    auth, profile, policy, now_utc = _signed_authorization()
    del policy['external_attestation']['trust_store']
    result = _verify(auth, profile, policy, now_utc=now_utc)
    assert result['status'] == 'NOT_READY'
    assert result['pass'] is False
    assert any('trust store is not configured' in item
               for item in result['not_ready'])


def test_missing_backend_contract_is_not_ready():
    auth, profile, policy, now_utc = _signed_authorization()
    del policy['external_attestation']['verification_backend']
    result = _verify(auth, profile, policy, now_utc=now_utc)
    assert result['status'] == 'NOT_READY'
    assert any('verification backend contract is missing' in item
               for item in result['not_ready'])


def test_backend_version_drift_is_fail_closed():
    auth, profile, policy, now_utc = _signed_authorization()
    policy['external_attestation']['verification_backend']['version'] = '0.0.0'
    result = _verify(auth, profile, policy, now_utc=now_utc)
    assert result['status'] == 'FAIL_CLOSED'
    assert any('backend version drift' in item for item in result['errors'])


def test_missing_cryptography_backend_is_not_ready(monkeypatch):
    auth, profile, policy, now_utc = _signed_authorization()
    original_import = builtins.__import__

    def blocked_import(name, *args, **kwargs):
        if name == 'cryptography' or name.startswith('cryptography.'):
            raise ModuleNotFoundError('synthetic missing cryptography')
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', blocked_import)
    result = _verify(auth, profile, policy, now_utc=now_utc)
    assert result['status'] == 'NOT_READY'
    assert any('Ed25519 verification dependency is unavailable' in item
               for item in result['not_ready'])


def test_attestation_schemas_validate_ephemeral_trust_store():
    auth, _, policy, _ = _signed_authorization()
    auth_schema = json.loads((ROOT / 'configs/slam_benchmark_profiles' /
                              'competitive_fresh_holdout_authorization_v1.schema.json')
                             .read_text(encoding='utf-8'))
    for name in (
            'competitive_fresh_holdout_authorization_v1.schema.json',
            'competitive_external_attestation_trust_store_v1.schema.json'):
        schema_path = ROOT / 'configs/slam_benchmark_profiles' / name
        schema = json.loads(schema_path.read_text(encoding='utf-8'))
        jsonschema.Draft202012Validator.check_schema(schema)
    assert not list(jsonschema.Draft202012Validator(auth_schema).iter_errors(auth))
    without_binding = copy.deepcopy(auth)
    without_binding.pop('external_attestation_binding_sha256')
    assert list(jsonschema.Draft202012Validator(auth_schema).iter_errors(
        without_binding))
    trust_schema = json.loads((ROOT / 'configs/slam_benchmark_profiles' /
                               'competitive_external_attestation_trust_store_v1.schema.json')
                              .read_text(encoding='utf-8'))
    jsonschema.Draft202012Validator(trust_schema).validate(
        policy['external_attestation']['trust_store'])
