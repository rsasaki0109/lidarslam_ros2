#!/usr/bin/env python3
"""Prepare and validate a non-promoting machine-PoP custodian handoff.

The packet is an unsigned review envelope.  It binds one candidate/profile,
one challenge/key/proof/artifact set, and one externally supplied READY trust
policy.  It never creates a key, accepts private key material, or switches the
checked-in NOT_READY policy.  Offline validation deliberately reports no
current-host match and no benchmark/claim eligibility.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Mapping

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from scripts import capture_competitive_execution_machine_identity_pop as pop  # noqa: E402


ROOT = _ROOT
HANDOFF_SCHEMA = 'competitive_execution_machine_identity_pop_handoff_v1'
HANDOFF_SIDECAR_SCHEMA = (
    'competitive_execution_machine_identity_pop_handoff_sidecar_v1')
POLICY_SIDECAR_SCHEMA = (
    'competitive_execution_machine_identity_pop_handoff_input_sidecar_v1')
HANDOFF_MAX_BYTES = 2 * 1024 * 1024
SIDECAR_MAX_BYTES = 4096
HANDOFF_READONLY_MODE = 0o444
HANDOFF_FIELDS = {
    'schema', 'schema_version', 'status', 'benchmark_eligible',
    'claim_eligible', 'candidate_binding', 'execution_context', 'asset_root',
    'challenge', 'host_public_key', 'proof', 'machine_artifact',
    'trust_policy', 'validity', 'replay', 'external_review',
    'promotion_policy', 'producer', 'handoff_identity_sha256',
}
PRODUCER_VERSION = '1.0.0'
MODULE_PATH = Path(__file__)
MODULE_RELATIVE = MODULE_PATH.relative_to(ROOT).as_posix()


class HandoffError(ValueError):
    """Malformed, stale, unsafe, or non-promoting handoff input."""


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _canonical(value: Any) -> bytes:
    return pop._canonical(value)


def _canonical_hash(value: Any, excluded: str | None = None) -> str:
    return pop._canonical_hash(value, excluded)


def _abs(value: Any, label: str) -> Path:
    try:
        return pop._safe_abs(value, label)
    except pop.MachinePoPError as error:
        raise HandoffError(str(error)) from error


def _sha_field(value: Any, label: str) -> str:
    try:
        return pop._sha_field(value, label)
    except pop.MachinePoPError as error:
        raise HandoffError(str(error)) from error


def _read_pair(path: Path, *, schema: str, sidecar_schema: str,
               label: str, limit: int = HANDOFF_MAX_BYTES
               ) -> tuple[dict[str, Any], bytes, bytes]:
    try:
        value, data = pop._read_pair(
            path, schema=schema, sidecar_schema=sidecar_schema,
            label=label, limit=limit)
        sidecar_path = path.with_name(path.name + '.sha256.json')
        side_data, _ = pop._read_bytes(
            sidecar_path, f'{label} sidecar', limit=SIDECAR_MAX_BYTES,
            readonly=True)
    except (OSError, pop.MachinePoPError) as error:
        raise HandoffError(str(error)) from error
    return value, data, side_data


def _source_base(path: Path, value: Mapping[str, Any], data: bytes,
                 side_data: bytes, *, relative: bool = True) -> dict[str, Any]:
    return {
        'schema': value['schema'],
        'path': path.name if relative else str(path),
        'file_sha256': _sha(data),
        'file_bytes': len(data),
        'canonical_sha256': value['canonical_sha256'],
        'sidecar_sha256': _sha(side_data),
    }


def _candidate_context() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    try:
        candidate, info = pop._candidate_snapshot()
    except (OSError, pop.MachinePoPError, ValueError, TypeError) as error:
        raise HandoffError(str(error)) from error
    closures = candidate.get('external_closures')
    source_bindings = candidate.get('source_bindings')
    system_bindings = candidate.get('system_bindings')
    required_systems = candidate.get('required_systems')
    release = candidate.get('release_binding')
    if (not isinstance(closures, Mapping) or not isinstance(source_bindings, Mapping) or
            not isinstance(system_bindings, Mapping) or
            required_systems != ['ours', 'glim', 'fast_livo2'] or
            not isinstance(release, Mapping) or release.get('release') != 'Release'):
        raise HandoffError('candidate execution/source/legal context is incomplete')
    candidate_binding = {
        'path': pop.CANDIDATE_REL,
        'file_sha256': info['candidate_file_sha256'],
        'candidate_identity_sha256': info['candidate_binding']['candidate_identity_sha256'],
        'candidate_id': info['candidate_binding']['candidate_id'],
        'status': info['candidate_binding']['status'],
        'profile_binding': info['profile_binding'],
        'thread_policy': info['thread_policy'],
        'external_closures_sha256': _canonical_hash(closures),
    }
    execution_context = {
        'required_systems': list(required_systems),
        'release': release['release'],
        'source_bindings_sha256': _canonical_hash(source_bindings),
        'system_bindings_sha256': _canonical_hash(system_bindings),
        'rival_source_closure_sha256': _canonical_hash(closures['rival_source']),
        'dataset_source_closure_sha256': _canonical_hash(closures['dataset_source']),
        'fresh_holdout_authorization_sha256': _canonical_hash(
            closures['fresh_holdout_authorization']),
        'image_identity_sha256': _canonical_hash(closures['image_identity']),
        'toolchain_identity_sha256': _canonical_hash(closures['toolchain_identity']),
        'legal_status': closures['rival_source']['legal_status'],
    }
    return candidate, candidate_binding, execution_context


def _producer() -> dict[str, str]:
    try:
        data, _ = pop._read_bytes(
            MODULE_PATH, 'machine-PoP handoff producer',
            limit=HANDOFF_MAX_BYTES, readonly=False)
    except pop.MachinePoPError as error:
        raise HandoffError(str(error)) from error
    return {'module': MODULE_RELATIVE, 'source_sha256': _sha(data),
            'version': PRODUCER_VERSION}


def _asset_root(path: Path) -> dict[str, Any]:
    try:
        identity = pop._directory_identity(path, 'machine-PoP asset root')
    except pop.MachinePoPError as error:
        raise HandoffError(str(error)) from error
    return {'path': str(path), **identity}


def _same_asset_root(path: Path, root: Path, label: str) -> None:
    if path.parent != root:
        raise HandoffError(f'{label} must be directly under the asset root')
    try:
        pop._file_identity(path, label, limit=HANDOFF_MAX_BYTES, readonly=True)
    except pop.MachinePoPError as error:
        raise HandoffError(str(error)) from error


def _key_ref(path: Path, value: Mapping[str, Any], data: bytes,
             side_data: bytes) -> dict[str, Any]:
    result = _source_base(path, value, data, side_data)
    result.update({
        'key_id': value['key_id'], 'algorithm': value['algorithm'],
        'public_key_sha256': value['public_key_sha256'],
        'provisioning_receipt_sha256': value['provisioning_receipt_sha256'],
        'valid_from': value['valid_from'], 'valid_until': value['valid_until'],
    })
    return result


def _policy_ref(path: Path, value: Mapping[str, Any], data: bytes,
                side_data: bytes) -> dict[str, Any]:
    result = _source_base(path, value, data, side_data, relative=False)
    result.update({
        'status': value['status'], 'authorized_runtime': value['authorized_runtime'],
        'authorized_keys_sha256': _canonical_hash(value['authorized_keys']),
    })
    return result


def _validate_ready_policy(policy_path: Path, policy: Mapping[str, Any],
                           policy_data: bytes, candidate_info: Mapping[str, Any],
                           key: Mapping[str, Any], issued_at: int) -> dict[str, Any]:
    if policy_path == pop.TRUST_POLICY_PATH:
        raise HandoffError('checked-in NOT_READY policy cannot be supplied as external policy')
    try:
        _, expected_ref = pop._policy_ref(policy_path, candidate_info)
    except (OSError, pop.MachinePoPError) as error:
        raise HandoffError(str(error)) from error
    if (_sha(policy_data) != expected_ref['file_sha256'] or
            policy.get('status') != 'READY' or
            policy.get('authorized_runtime') is not True or
            policy.get('benchmark_eligible') is not False or
            not isinstance(policy.get('authorized_keys'), list) or
            len(policy['authorized_keys']) != 1):
        raise HandoffError('external trust policy is not a strict READY policy')
    authorized = policy['authorized_keys'][0]
    expected = {
        'key_id': key['key_id'], 'algorithm': pop.ALGORITHM,
        'public_key_sha256': key['public_key_sha256'], 'status': 'ACTIVE',
        'valid_from': key['valid_from'], 'valid_until': key['valid_until'],
    }
    if authorized != expected:
        raise HandoffError('external trust policy key binding mismatch')
    try:
        pop._policy_key_matches(policy, key, issued_at)
    except pop.MachinePoPError as error:
        raise HandoffError(str(error)) from error
    return expected_ref


def _validate_inputs(*, challenge_path: Path, key_path: Path,
                     proof_path: Path, machine_path: Path,
                     policy_path: Path) -> dict[str, Any]:
    challenge_path = _abs(challenge_path, 'challenge')
    key_path = _abs(key_path, 'host public key')
    proof_path = _abs(proof_path, 'proof')
    machine_path = _abs(machine_path, 'machine artifact')
    policy_path = _abs(policy_path, 'external trust policy')
    if policy_path == pop.TRUST_POLICY_PATH:
        raise HandoffError('checked-in trust policy is not an external handoff input')
    root = machine_path.parent
    for path, label in ((challenge_path, 'challenge'), (key_path, 'host public key'),
                        (proof_path, 'proof'), (machine_path, 'machine artifact')):
        _same_asset_root(path, root, label)
    asset_root = _asset_root(root)
    try:
        challenge, challenge_data, challenge_side = _read_pair(
            challenge_path, schema=pop.CHALLENGE_SCHEMA,
            sidecar_schema=pop.KEY_SIDECAR_SCHEMA, label='challenge')
        key, key_data, key_side = _read_pair(
            key_path, schema=pop.KEY_SCHEMA,
            sidecar_schema=pop.KEY_SIDECAR_SCHEMA, label='host public key',
            limit=pop.MAX_KEY_BYTES)
        proof, proof_data, proof_side = _read_pair(
            proof_path, schema=pop.PROOF_SCHEMA,
            sidecar_schema=pop.KEY_SIDECAR_SCHEMA, label='proof',
            limit=pop.MAX_KEY_BYTES)
        machine, machine_data, machine_side = _read_pair(
            machine_path, schema=pop.ARTIFACT_SCHEMA,
            sidecar_schema=pop.ARTIFACT_SIDECAR_SCHEMA, label='machine artifact')
        policy, policy_data, policy_side = _read_pair(
            policy_path, schema=pop.POLICY_SCHEMA,
            sidecar_schema=POLICY_SIDECAR_SCHEMA, label='external trust policy')
    except (OSError, pop.MachinePoPError, HandoffError) as error:
        raise HandoffError(str(error)) from error
    candidate, candidate_binding, execution_context = _candidate_context()
    del candidate
    candidate_info = {
        'candidate_binding': {
            key: candidate_binding[key] for key in (
                'path', 'file_sha256', 'candidate_identity_sha256',
                'candidate_id', 'status')},
        'profile_binding': candidate_binding['profile_binding'],
        'thread_policy': candidate_binding['thread_policy'],
    }
    try:
        pop._validate_key(key, pop.CAMPAIGN_ID)
        pop._validate_challenge(challenge, candidate_info)
    except (OSError, pop.MachinePoPError, KeyError, TypeError) as error:
        raise HandoffError(str(error)) from error
    if challenge['candidate_binding'] != {
            key: candidate_binding[key] for key in (
                'path', 'file_sha256', 'candidate_identity_sha256',
                'candidate_id', 'status')} or \
            challenge['profile_binding'] != candidate_binding['profile_binding'] or \
            challenge['thread_policy'] != candidate_binding['thread_policy']:
        raise HandoffError('challenge candidate/profile/thread binding drift')
    if machine.get('status') != 'LIVE_CAPTURED' or \
            machine.get('review_status') != 'NOT_REVIEWED_EXTERNAL' or \
            machine.get('benchmark_eligible') is not False or \
            machine.get('claim_eligible') is not False:
        raise HandoffError('machine artifact is not a non-promoting LIVE_CAPTURED artifact')
    try:
        policy_ref = _validate_ready_policy(
            policy_path, policy, policy_data, candidate_info,
            key, challenge['issued_at'])
        validation = pop.validate_artifact(
            machine_path, live=False, _policy_path=policy_path,
            _allow_external_policy=True)
    except (OSError, pop.MachinePoPError, HandoffError) as error:
        raise HandoffError(str(error)) from error
    if validation.get('live_host_match') is not False or \
            validation.get('host_match_status') != 'HOST_MATCH_NOT_CHECKED':
        raise HandoffError('handoff validation cannot claim live host equality')
    # Reopen every source after the cryptographic/artifact verification.  The
    # first read establishes the document projections; this second read closes
    # the interval in which a policy, proof, or sidecar could be replaced.
    for path, schema, sidecar_schema, label, original in (
            (challenge_path, pop.CHALLENGE_SCHEMA, pop.KEY_SIDECAR_SCHEMA,
             'challenge', (challenge, challenge_data, challenge_side)),
            (key_path, pop.KEY_SCHEMA, pop.KEY_SIDECAR_SCHEMA,
             'host public key', (key, key_data, key_side)),
            (proof_path, pop.PROOF_SCHEMA, pop.KEY_SIDECAR_SCHEMA,
             'proof', (proof, proof_data, proof_side)),
            (machine_path, pop.ARTIFACT_SCHEMA, pop.ARTIFACT_SIDECAR_SCHEMA,
             'machine artifact', (machine, machine_data, machine_side)),
            (policy_path, pop.POLICY_SCHEMA, POLICY_SIDECAR_SCHEMA,
             'external trust policy', (policy, policy_data, policy_side))):
        value_again, data_again, side_again = _read_pair(
            path, schema=schema, sidecar_schema=sidecar_schema, label=label)
        if (value_again != original[0] or data_again != original[1] or
                side_again != original[2]):
            raise HandoffError(f'{label} changed during handoff validation')
    current_root = _asset_root(root)
    if current_root != asset_root:
        raise HandoffError('machine-PoP asset root changed during validation')
    _, candidate_binding_again, execution_context_again = _candidate_context()
    if (candidate_binding_again != candidate_binding or
            execution_context_again != execution_context):
        raise HandoffError('candidate/profile context changed during validation')
    artifact_nonce = machine['nonce_claim']
    claim_path = _abs(
        Path(challenge['replay_ledger_path']) / artifact_nonce['path'],
        'nonce claim')
    challenge_ref = _source_base(challenge_path, challenge, challenge_data,
                                 challenge_side)
    challenge_ref['challenge_payload_sha256'] = challenge['challenge_payload_sha256']
    key_ref = _key_ref(key_path, key, key_data, key_side)
    proof_ref = _source_base(proof_path, proof, proof_data, proof_side)
    proof_ref.update({
        'key_id': proof['key_id'],
        'challenge_payload_sha256': proof['challenge_payload_sha256'],
        'signature_sha256': proof['signature_sha256'],
    })
    artifact_ref = _source_base(machine_path, machine, machine_data, machine_side)
    artifact_ref.update({
        'identity_sha256': machine['identity_sha256'],
        'status': machine['status'], 'review_status': machine['review_status'],
    })
    policy_ref = _policy_ref(policy_path, policy, policy_data, policy_side)
    document: dict[str, Any] = {
        'schema': HANDOFF_SCHEMA, 'schema_version': 1,
        'status': 'UNSIGNED_REVIEW_REQUIRED',
        'benchmark_eligible': False, 'claim_eligible': False,
        'candidate_binding': candidate_binding,
        'execution_context': execution_context,
        'asset_root': asset_root,
        'challenge': challenge_ref, 'host_public_key': key_ref,
        'proof': proof_ref, 'machine_artifact': artifact_ref,
        'trust_policy': policy_ref,
        'validity': {'issued_at': challenge['issued_at'],
                     'expires_at': challenge['expires_at']},
        'replay': {
            'nonce_sha256': challenge['challenge_nonce_sha256'],
            'ledger_path': challenge['replay_ledger_path'],
            'claim_path': str(claim_path),
            'claim_sha256': artifact_nonce['file_sha256'],
        },
        'external_review': {
            'status': 'UNSIGNED_REVIEW_REQUIRED',
            'live_host_match': False,
            'host_match_status': 'HOST_MATCH_NOT_CHECKED',
            'key_policy_review': 'REQUIRED_EXTERNAL_CUSTODIAN',
            'auto_promoted': False,
        },
        'producer': _producer(),
        'promotion_policy': {
            'benchmark_eligible': False, 'claim_eligible': False,
            'active_profile_switch': False,
            'requires_external_custodian_review': True,
            'requires_candidate_profile_reseal': True,
            'checked_in_policy_immutable': True,
            'private_key_included': False,
        },
        'handoff_identity_sha256': '',
    }
    document['handoff_identity_sha256'] = _canonical_hash(
        document, 'handoff_identity_sha256')
    return document


def _write_pair(output: Path, document: Mapping[str, Any]) -> None:
    output = _abs(output, 'machine-PoP handoff output')
    if output.name.endswith('.sha256.json'):
        raise HandoffError('handoff output must not be a sidecar path')
    data = _canonical(document) + b'\n'
    sidecar = {
        'schema': HANDOFF_SIDECAR_SCHEMA, 'schema_version': 1,
        'artifact_path': output.name, 'artifact_bytes': len(data),
        'artifact_sha256': _sha(data),
        'canonical_sha256': document['handoff_identity_sha256'],
    }
    try:
        pop._seal_pair(
            output, data, output.with_name(output.name + '.sha256.json'),
            _canonical(sidecar) + b'\n', 'machine-PoP handoff')
    except (OSError, pop.MachinePoPError) as error:
        raise HandoffError(str(error)) from error


def prepare_handoff(*, output: Path, challenge: Path, key_descriptor: Path,
                    proof: Path, machine_artifact: Path,
                    trust_policy: Path) -> dict[str, Any]:
    output = _abs(output, 'machine-PoP handoff output')
    document = _validate_inputs(
        challenge_path=challenge, key_path=key_descriptor, proof_path=proof,
        machine_path=machine_artifact, policy_path=trust_policy)
    _write_pair(output, document)
    return validate_handoff(output)


def _read_handoff(path: Path) -> tuple[dict[str, Any], bytes, bytes]:
    path = _abs(path, 'machine-PoP handoff')
    try:
        data, identity = pop._read_bytes(
            path, 'machine-PoP handoff', limit=HANDOFF_MAX_BYTES,
            readonly=True)
        sidecar_path = path.with_name(path.name + '.sha256.json')
        side, side_identity = pop._read_bytes(
            sidecar_path, 'machine-PoP handoff sidecar',
            limit=SIDECAR_MAX_BYTES, readonly=True)
        value = json.loads(data.decode('utf-8'))
        side_value = json.loads(side.decode('utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError,
            pop.MachinePoPError) as error:
        raise HandoffError(str(error)) from error
    if not isinstance(value, dict) or not isinstance(side_value, dict):
        raise HandoffError('machine-PoP handoff pair is not JSON objects')
    if value.get('schema') != HANDOFF_SCHEMA or \
            value.get('schema_version') != 1:
        raise HandoffError('machine-PoP handoff schema identity is invalid')
    if set(side_value) != {'schema', 'schema_version', 'artifact_path',
                           'artifact_bytes', 'artifact_sha256',
                           'canonical_sha256'} or \
            side_value['schema'] != HANDOFF_SIDECAR_SCHEMA or \
            side_value['schema_version'] != 1 or \
            side_value['artifact_path'] != path.name or \
            side_value['artifact_bytes'] != len(data) or \
            side_value['artifact_sha256'] != _sha(data) or \
            side_value['canonical_sha256'] != value.get(
                'handoff_identity_sha256'):
        raise HandoffError('machine-PoP handoff sidecar binding drift')
    try:
        data_again, identity_again = pop._read_bytes(
            path, 'machine-PoP handoff', limit=HANDOFF_MAX_BYTES,
            readonly=True)
        side_again, side_identity_again = pop._read_bytes(
            sidecar_path, 'machine-PoP handoff sidecar',
            limit=SIDECAR_MAX_BYTES, readonly=True)
    except pop.MachinePoPError as error:
        raise HandoffError(str(error)) from error
    if (data_again != data or side_again != side or
            identity_again != identity or side_identity_again != side_identity):
        raise HandoffError('machine-PoP handoff changed during validation')
    if set(value) != HANDOFF_FIELDS:
        raise HandoffError('machine-PoP handoff field set is not exact')
    if value['producer'] != _producer():
        raise HandoffError('machine-PoP handoff producer source drift')
    if value['handoff_identity_sha256'] != _canonical_hash(
            value, 'handoff_identity_sha256'):
        raise HandoffError('machine-PoP handoff identity is stale')
    if value['status'] != 'UNSIGNED_REVIEW_REQUIRED' or \
            value['benchmark_eligible'] is not False or \
            value['claim_eligible'] is not False:
        raise HandoffError('machine-PoP handoff is not non-promoting')
    if value['external_review'] != {
            'status': 'UNSIGNED_REVIEW_REQUIRED', 'live_host_match': False,
            'host_match_status': 'HOST_MATCH_NOT_CHECKED',
            'key_policy_review': 'REQUIRED_EXTERNAL_CUSTODIAN',
            'auto_promoted': False}:
        raise HandoffError('machine-PoP handoff review status is invalid')
    if value['promotion_policy'] != {
            'benchmark_eligible': False, 'claim_eligible': False,
            'active_profile_switch': False,
            'requires_external_custodian_review': True,
            'requires_candidate_profile_reseal': True,
            'checked_in_policy_immutable': True, 'private_key_included': False}:
        raise HandoffError('machine-PoP handoff promotion policy is invalid')
    return value, data, side


def validate_handoff(path: Path) -> dict[str, Any]:
    path = _abs(path, 'machine-PoP handoff')
    value, data, side = _read_handoff(path)
    root_value = value['asset_root']
    if not isinstance(root_value, Mapping) or set(root_value) != {
            'path', 'device', 'inode', 'mode'}:
        raise HandoffError('machine-PoP handoff asset root is invalid')
    root = _abs(root_value['path'], 'machine-PoP asset root')
    try:
        current = pop._directory_identity(root, 'machine-PoP asset root')
    except pop.MachinePoPError as error:
        raise HandoffError(str(error)) from error
    if current != {key: root_value[key] for key in ('device', 'inode', 'mode')}:
        raise HandoffError('machine-PoP asset root identity changed')
    candidate, candidate_binding, execution_context = _candidate_context()
    del candidate
    if value['candidate_binding'] != candidate_binding or \
            value['execution_context'] != execution_context:
        raise HandoffError('machine-PoP handoff candidate/source context drift')
    challenge_path = root / value['challenge']['path']
    key_path = root / value['host_public_key']['path']
    proof_path = root / value['proof']['path']
    machine_path = root / value['machine_artifact']['path']
    policy_path = _abs(value['trust_policy']['path'], 'external trust policy')
    expected = _validate_inputs(
        challenge_path=challenge_path, key_path=key_path,
        proof_path=proof_path, machine_path=machine_path,
        policy_path=policy_path)
    if expected != value:
        raise HandoffError('machine-PoP handoff source binding drift')
    final_data, final_side = _read_handoff(path)[1:]
    if final_data != data or final_side != side:
        raise HandoffError('machine-PoP handoff changed during validation')
    return {
        'status': value['status'], 'structural_valid': True,
        'benchmark_eligible': False, 'claim_eligible': False,
        'live_host_match': False,
        'host_match_status': 'HOST_MATCH_NOT_CHECKED',
        'handoff_identity_sha256': value['handoff_identity_sha256'],
        'trust_policy_status': 'READY',
        'promotion_allowed': False,
    }


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    prepare = sub.add_parser('prepare')
    prepare.add_argument('--output', type=Path, required=True)
    prepare.add_argument('--challenge', type=Path, required=True)
    prepare.add_argument('--key-descriptor', type=Path, required=True)
    prepare.add_argument('--proof', type=Path, required=True)
    prepare.add_argument('--machine-artifact', type=Path, required=True)
    prepare.add_argument('--trust-policy', type=Path, required=True)
    validate = sub.add_parser('validate')
    validate.add_argument('--handoff', type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.command == 'prepare':
            result = prepare_handoff(
                output=args.output, challenge=args.challenge,
                key_descriptor=args.key_descriptor, proof=args.proof,
                machine_artifact=args.machine_artifact,
                trust_policy=args.trust_policy)
        else:
            result = validate_handoff(args.handoff)
    except (HandoffError, OSError, ValueError, TypeError) as error:
        print(json.dumps({'status': 'INVALID', 'error': str(error)}, indent=2))
        return 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(_main())
