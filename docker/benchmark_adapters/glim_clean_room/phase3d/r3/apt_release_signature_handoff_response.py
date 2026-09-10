#!/usr/bin/env python3
"""Review an external custodian response to one sealed r3 handoff request.

This module is a verifier only.  It never generates a policy, key, signature,
or authorization.  A response is non-promoting even when its READY policy and
Ed25519 authorization verify successfully.  The checked-in candidate remains
NOT_READY; an external promotion workflow must reseal the candidate and issue
a new handoff request before this verifier can accept a READY response.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import stat
import time
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[5]
R3 = ROOT / 'docker/benchmark_adapters/glim_clean_room/phase3d/r3'
MODULE_PATH = Path(__file__)
MODULE_RELATIVE = MODULE_PATH.relative_to(ROOT).as_posix()
SCHEMA = 'glim_clean_room_r3_custodian_handoff_response_v1'
SIDECAR_SCHEMA = 'glim_clean_room_r3_custodian_handoff_response_sidecar_v1'
DOMAIN = 'lidarslam/glim-clean-room-r3/custodian-handoff-response/sha256/v1'
MAX_JSON_BYTES = 16 * 1024 * 1024
SHA_RE = r'^[0-9a-f]{64}$'


class ResponseError(ValueError):
    """Raised when an external handoff response is not exactly bound."""


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ResponseError(f'cannot load contract: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


HANDOFF = _load('r3_response_handoff', R3 / 'apt_release_signature_handoff.py')
AUTH = _load('r3_response_authorization', R3 / 'apt_release_signature_authorization.py')
AUTH_SCHEMA_PATH = R3 / 'apt_release_signature_authorization.schema.json'
HANDOFF_SCHEMA_PATH = R3 / 'apt_release_signature_handoff.schema.json'


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True).encode('utf-8')


def canonical_hash(value: Any, excluded: str | None = None) -> str:
    if isinstance(value, Mapping):
        value = {str(key): item for key, item in value.items() if key != excluded}
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _safe_absolute(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.startswith('/') or '\x00' in value:
        raise ResponseError(f'{label} is not absolute')
    path = Path(value)
    if path.as_posix() != value or path == Path('/') or any(
            part in {'', '.', '..'} for part in value[1:].split('/')):
        raise ResponseError(f'{label} is not normalized')
    return path


def _parent_no_symlink(path: Path, label: str) -> None:
    current = Path(path.anchor)
    for component in path.parent.parts[1:]:
        current /= component
        try:
            info = current.lstat()
        except OSError as error:
            raise ResponseError(f'{label} parent is unavailable') from error
        if stat.S_ISLNK(info.st_mode):
            raise ResponseError(f'{label} contains a symlink parent')


def _regular(path: Path, label: str, *, limit: int = MAX_JSON_BYTES,
             immutable: bool = False) -> bytes:
    _parent_no_symlink(path, label)
    try:
        before = path.lstat()
    except OSError as error:
        raise ResponseError(f'{label} cannot be inspected') from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or \
            before.st_nlink != 1 or before.st_size <= 0 or before.st_size > limit:
        raise ResponseError(f'{label} is not a bounded immutable regular file')
    try:
        with path.open('rb') as stream:
            fd_before = os.fstat(stream.fileno())
            data = stream.read(limit + 1)
            fd_after = os.fstat(stream.fileno())
    except OSError as error:
        raise ResponseError(f'{label} cannot be read') from error
    if (fd_before.st_dev, fd_before.st_ino, fd_before.st_nlink) != \
            (before.st_dev, before.st_ino, before.st_nlink) or \
            (fd_after.st_dev, fd_after.st_ino, fd_after.st_nlink) != \
            (before.st_dev, before.st_ino, before.st_nlink) or \
            fd_after.st_size != before.st_size or len(data) != before.st_size or \
            len(data) > limit or (immutable and before.st_mode & 0o222):
        raise ResponseError(f'{label} changed or is mutable')
    return data


def _json(path: Path, label: str, *, immutable: bool = False) -> tuple[dict[str, Any], bytes]:
    data = _regular(path, label, immutable=immutable)
    try:
        value = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ResponseError(f'{label} is invalid JSON') from error
    if not isinstance(value, dict):
        raise ResponseError(f'{label} is not an object')
    return value, data


def _descriptor(path: Path, label: str, *, schema: str | None = None,
                immutable: bool = False) -> tuple[dict[str, Any], dict[str, Any], bytes]:
    path = _safe_absolute(str(path), f'{label} path')
    value, data = _json(path, label, immutable=immutable)
    canonical = value.get('canonical_sha256')
    if canonical is None and value.get('schema') == AUTH.AUTH_SCHEMA:
        # Authorization has a signed payload identity rather than a mutable
        # top-level canonical_sha256 field.  Keep the descriptor explicit and
        # derive it from the same domain-separated payload that is verified
        # below.
        canonical = AUTH.authorization_payload_sha256(value)
    if not isinstance(canonical, str) or canonical != (
            canonical_hash(value, 'canonical_sha256')
            if 'canonical_sha256' in value else canonical):
        raise ResponseError(f'{label} canonical identity drift')
    if schema is not None and value.get('schema') != schema:
        raise ResponseError(f'{label} schema drift')
    descriptor = {'path': str(path), 'file_bytes': len(data),
                  'file_sha256': _sha(data), 'canonical_sha256': canonical,
                  'schema': value.get('schema', '')}
    return value, descriptor, data


def _sealed(path: Path, label: str, *, schema: str,
            sidecar_schema: str) -> tuple[dict[str, Any], dict[str, Any]]:
    value, descriptor, data = _descriptor(path, label, schema=schema, immutable=True)
    sidecar_path = path.with_name(path.name + '.sha256.json')
    sidecar, side_data = _json(sidecar_path, f'{label} sidecar', immutable=True)
    if (set(sidecar) != {
            'schema', 'schema_version', 'artifact_path', 'artifact_bytes',
            'artifact_sha256', 'canonical_sha256'} or
            sidecar.get('schema') != sidecar_schema or
            sidecar.get('schema_version') != 1 or
            sidecar.get('artifact_path') != path.name or
            sidecar.get('artifact_bytes') != len(data) or
            sidecar.get('artifact_sha256') != _sha(data) or
            sidecar.get('canonical_sha256') != value.get('canonical_sha256')):
        raise ResponseError(f'{label} sidecar binding drift')
    # Reopen through the same exact path after the sidecar read to close a
    # replacement race between the descriptor and sidecar observations.
    again, again_descriptor, _ = _descriptor(
        path, f'{label} recheck', schema=schema, immutable=True)
    if again != value or again_descriptor != descriptor:
        raise ResponseError(f'{label} changed during validation')
    sidecar_again, sidecar_again_data = _json(
        sidecar_path, f'{label} sidecar recheck', immutable=True)
    if sidecar_again != sidecar or sidecar_again_data != side_data:
        raise ResponseError(f'{label} sidecar changed during validation')
    return value, descriptor


def _request_binding(request: Mapping[str, Any]) -> dict[str, Any]:
    template = request.get('authorization_template')
    if not isinstance(template, Mapping):
        raise ResponseError('handoff authorization template is missing')
    bindings = template.get('bindings')
    if not isinstance(bindings, Mapping):
        raise ResponseError('handoff authorization template bindings are missing')
    required = {
        'candidate', 'policy', 'plan', 'executor', 'trust_root', 'repository',
        'binding_identity_sha256', 'deb_bindings_sha256', 'tool', 'scope',
        'machine_identity', 'machine_fingerprint', 'workflow_sha256', 'phases_sha256',
    }
    if set(bindings) != required:
        raise ResponseError('handoff authorization template binding shape drift')
    return {
        'campaign_id': request['campaign_id'],
        'candidate': request['candidate'],
        'policy_template': request['policy'],
        'plan': request['plan'],
        'executor': request['executor'],
        'trust_root': request['trust_root'],
        'repository': request['repository'],
        'binding': request['binding'],
        'deb_bindings': request['deb_bindings'],
        'deb_bindings_sha256': request['deb_bindings_sha256'],
        'scope': request['scope'],
        'machine_identity': request['machine_identity'],
        'machine_fingerprint': (request['machine_fingerprint']
                                if 'machine_fingerprint' in request
                                else bindings['machine_fingerprint']),
        'safety': request['safety'],
        'requested_validity': request['requested_validity'],
        'nonce': request['nonce'],
        'authorization_template': template,
    }


def _verifier_descriptor() -> dict[str, Any]:
    try:
        import cryptography
    except ImportError as error:
        raise ResponseError('cryptography backend is unavailable') from error
    auth_data = _regular(AUTH.IMPLEMENTATION_PATH, 'authorization verifier')
    auth_schema_data = _regular(AUTH_SCHEMA_PATH, 'authorization schema')
    handoff_schema_data = _regular(HANDOFF_SCHEMA_PATH, 'handoff schema')
    return {
        'module': AUTH.IMPLEMENTATION_RELATIVE,
        'source_sha256': _sha(auth_data),
        'authorization_schema': AUTH.AUTH_SCHEMA,
        'authorization_schema_path': AUTH_SCHEMA_PATH.relative_to(ROOT).as_posix(),
        'authorization_schema_sha256': _sha(auth_schema_data),
        'handoff_schema': HANDOFF.SCHEMA,
        'handoff_schema_path': HANDOFF_SCHEMA_PATH.relative_to(ROOT).as_posix(),
        'handoff_schema_sha256': _sha(handoff_schema_data),
        'backend': {
            **dict(AUTH.BACKEND), 'version': getattr(cryptography, '__version__', ''),
            'implementation_sha256': _sha(auth_data)},
    }


def _policy_machine_from_request(machine: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(machine, Mapping):
        raise ResponseError('request machine identity is malformed')
    if machine.get('validation') != 'OFFLINE_ARTIFACT_ONLY':
        raise ResponseError('request machine identity is not offline-only')
    result = {key: value for key, value in machine.items() if key != 'validation'}
    result = {'status': 'PRECOMMITTED', **result}
    return result


def _relative_or_name(path: Path) -> str:
    try:
        return path.relative_to(AUTH.ROOT).as_posix()
    except ValueError:
        return path.name


def _authorization_projection(auth: Mapping[str, Any], plan: Mapping[str, Any],
                              request: Mapping[str, Any]) -> dict[str, Any]:
    template = request['authorization_template']
    bindings = template['bindings']
    expected_machine = {key: value for key, value in bindings['machine_identity'].items()
                        if key != 'validation'}
    expected_tool = {
        'path': plan['tool']['binary_path'],
        'sha256': plan['tool']['binary_sha256'],
        'version': plan['tool']['version'],
    }
    expected_executor = {'source_sha256': bindings['executor']['file_sha256']}
    expected_trust_root = {
        'path': bindings['trust_root']['path'],
        'file_sha256': bindings['trust_root']['file_sha256'],
        'canonical_sha256': bindings['trust_root']['canonical_sha256'],
        'keyring_path': plan['trust_root']['keyring_path'],
        'keyring_sha256': plan['trust_root']['keyring_sha256'],
        'key_id': bindings['trust_root']['key_id'],
        'fingerprint': bindings['trust_root']['fingerprint'],
    }
    raw_debs = [{key: item[key] for key in ('name', 'version', 'architecture', 'path')}
                for item in request['deb_bindings']]
    fixed = {
        'schema': auth['schema'], 'schema_version': auth['schema_version'],
        'status': auth['status'], 'candidate': bindings['candidate'],
        'policy': bindings['policy'],
        'plan': {'sha256': hashlib.sha256(canonical_bytes(plan)).hexdigest(),
                 'canonical_sha256': plan['canonical_sha256']},
        'executor': expected_executor, 'trust_root': expected_trust_root,
        'repository': bindings['repository'], 'binding': request['binding'],
        'deb_bindings': raw_debs,
        'deb_bindings_sha256': AUTH.canonical_hash(raw_debs),
        'tool': expected_tool, 'scope': request['scope'],
        'machine_fingerprint': bindings['machine_fingerprint'],
        'machine_identity': expected_machine,
        'network_used': request['safety']['network_used'],
        'shell': request['safety']['shell'],
        'timeout_seconds': request['safety']['timeout_seconds'],
        'resource_limits': request['safety']['resource_limits'],
        'workflow': plan['workflow'], 'phases': plan['phases'],
        'nonce': request['nonce'],
    }
    return fixed


def _check_auth_template(auth: Mapping[str, Any], request: Mapping[str, Any],
                         plan: Mapping[str, Any]) -> None:
    expected = _authorization_projection(auth, plan, request)
    for key, value in expected.items():
        if key in {'schema', 'schema_version', 'status', 'nonce', 'candidate', 'policy'}:
            continue
        if auth.get(key) != value:
            raise ResponseError(f'authorization {key} is not the handoff template projection')
    if auth.get('nonce') != request['nonce']:
        raise ResponseError('authorization nonce is not the handoff nonce')
    validity = request['requested_validity']
    if type(auth.get('issued_at')) is not int or type(auth.get('expires_at')) is not int or \
            auth['issued_at'] < validity['issued_at'] or \
            auth['expires_at'] > validity['expires_at'] or \
            auth['expires_at'] <= auth['issued_at']:
        raise ResponseError('authorization validity is outside the requested window')
    if (auth.get('issued_at') < 0 or
            auth.get('expires_at') - auth.get('issued_at') > 7 * 24 * 60 * 60):
        raise ResponseError('authorization validity window is unreasonable')
    signature = auth.get('signature')
    if not isinstance(signature, Mapping) or set(signature) != {
            'algorithm', 'key_id', 'public_key_sha256', 'payload_sha256',
            'signature_base64', 'backend'}:
        raise ResponseError('authorization signature fields are not exact')


def validate_response(response_path: Path, request_path: Path | None = None,
                      *, _review_time: int | None = None) -> dict[str, Any]:
    """Reopen and validate one response at the verifier's current wall time.

    ``_review_time`` is a private deterministic-test seam.  Production callers
    and the CLI always use the verifier process's integer wall clock; the
    response author cannot choose or seal the review time.
    """
    review_time = int(time.time()) if _review_time is None else _review_time
    if type(review_time) is not int or review_time < 0:
        raise ResponseError('review time is not a nonnegative integer')
    response, response_desc = _sealed(Path(response_path), 'handoff response', schema=SCHEMA,
                                      sidecar_schema=SIDECAR_SCHEMA)
    required = {'schema', 'schema_version', 'status', 'benchmark_eligible', 'request',
                'candidate', 'policy', 'authorization', 'verifier', 'request_binding',
                'authorization_binding', 'machine_validation', 'canonical_sha256'}
    if (set(response) != required or response['schema'] != SCHEMA or
            response['schema_version'] != 1 or
            response['status'] != 'READY_AUTHORIZATION_RESPONSE' or
            response['benchmark_eligible'] is not False or
            response['machine_validation'] != 'OFFLINE_ARTIFACT_ONLY' or
            response['canonical_sha256'] != canonical_hash(response, 'canonical_sha256')):
        raise ResponseError('response schema/status/canonical identity is invalid')
    request_ref = response['request']
    if not isinstance(request_ref, Mapping) or set(request_ref) != {
            'path', 'file_bytes', 'file_sha256', 'canonical_sha256', 'schema'} or \
            request_ref['schema'] != HANDOFF.SCHEMA:
        raise ResponseError('response request reference is incomplete')
    request_abs = _safe_absolute(request_ref['path'], 'response request')
    if request_path is not None and request_abs != _safe_absolute(
            str(request_path), 'request argument'):
        raise ResponseError('response request path does not match the requested artifact')
    request, request_desc = _sealed(request_abs, 'handoff request', schema=HANDOFF.SCHEMA,
                                    sidecar_schema=HANDOFF.SIDECAR_SCHEMA)
    if request_desc != dict(request_ref):
        raise ResponseError('response request file identity drift')
    HANDOFF.validate_request(request_abs)
    expected_request_binding = _request_binding(request)
    if response['request_binding'] != expected_request_binding:
        raise ResponseError('response request binding projection drift')
    candidate_ref = response['candidate']
    policy_ref = response['policy']
    auth_ref = response['authorization']
    for label, ref, schema in (
            ('candidate', candidate_ref, 'glim_clean_room_r3_apt_allowlist_candidate_v1'),
            ('policy', policy_ref, AUTH.POLICY_SCHEMA),
            ('authorization', auth_ref, AUTH.AUTH_SCHEMA)):
        if not isinstance(ref, Mapping) or set(ref) != {
                'path', 'file_bytes', 'file_sha256', 'canonical_sha256', 'schema'} or \
                ref['schema'] != schema:
            raise ResponseError(f'response {label} reference is incomplete')
    candidate_path = _safe_absolute(candidate_ref['path'], 'response candidate')
    policy_path = _safe_absolute(policy_ref['path'], 'response policy')
    auth_path = _safe_absolute(auth_ref['path'], 'response authorization')
    candidate, candidate_desc, _ = _descriptor(
        candidate_path, 'candidate manifest', schema=candidate_ref['schema'])
    policy, policy_desc, _ = _descriptor(
        policy_path, 'READY authorization policy', schema=policy_ref['schema'])
    auth, auth_desc, _ = _descriptor(
        auth_path, 'signed authorization', schema=auth_ref['schema'])
    if (candidate_desc != dict(candidate_ref) or policy_desc != dict(policy_ref) or
            auth_desc != dict(auth_ref)):
        raise ResponseError('response source descriptor drift')
    template = request['authorization_template']
    bindings = template['bindings']
    if candidate_ref['file_sha256'] != bindings['candidate']['file_sha256'] or \
            candidate_ref['canonical_sha256'] != bindings['candidate']['canonical_sha256'] or \
            policy_ref['file_sha256'] != bindings['policy']['file_sha256'] or \
            policy_ref['canonical_sha256'] != bindings['policy']['canonical_sha256']:
        raise ResponseError('response candidate/policy is not the handoff template identity')
    if policy.get('status') != 'READY' or policy.get('authorized_keys') == [] or \
            policy.get('schema') != AUTH.POLICY_SCHEMA:
        raise ResponseError('response policy is not READY with authorized keys')
    if policy.get('candidate_manifest') != {
            'path': _relative_or_name(candidate_path),
            'candidate_id': candidate.get('candidate_id'),
            'revision': candidate.get('revision')}:
        raise ResponseError('response policy candidate binding is not exact')
    policy_binding = candidate.get('authorization_policy')
    if not isinstance(policy_binding, Mapping) or \
            policy_binding.get('path') != _relative_or_name(policy_path):
        raise ResponseError('response candidate policy path binding is invalid')
    if (policy_binding.get('status') != 'READY' or
            policy_binding.get('authorized_runtime') is not True or
            policy_binding.get('file_sha256') != policy_ref['file_sha256'] or
            policy_binding.get('canonical_sha256') != policy_ref['canonical_sha256']):
        raise ResponseError('response candidate does not pin the READY policy')
    policy_candidate = policy.get('candidate_manifest', {})
    if (policy_candidate.get('candidate_id') != candidate.get('candidate_id') or
            policy_candidate.get('revision') != candidate.get('revision')):
        raise ResponseError('response policy candidate identity drift')
    if policy.get('machine_identity') != _policy_machine_from_request(request['machine_identity']):
        raise ResponseError('response policy machine method/projection drift')
    if policy.get('backend') != request['authorization_backend']:
        raise ResponseError('response policy authorization backend drift')
    expected_auth_candidate = {
        'path': bindings['candidate']['path'],
        'file_sha256': bindings['candidate']['file_sha256'],
        'canonical_sha256': bindings['candidate']['canonical_sha256'],
        'candidate_id': candidate['candidate_id'], 'revision': candidate['revision'],
    }
    expected_auth_policy = {
        'path': bindings['policy']['path'],
        'file_sha256': bindings['policy']['file_sha256'],
        'canonical_sha256': bindings['policy']['canonical_sha256'],
    }
    if auth.get('candidate') != expected_auth_candidate or \
            auth.get('policy') != expected_auth_policy:
        raise ResponseError('response authorization candidate/policy binding drift')
    plan_path = Path(request['scope']['root_inputs']) / HANDOFF._safe_relative(
        request['plan']['path'], 'response plan path')
    plan, plan_data = HANDOFF._json(plan_path, 'response signature plan')
    if _sha(plan_data) != request['plan']['file_sha256'] or \
            plan.get('canonical_sha256') != request['plan']['canonical_sha256']:
        raise ResponseError('response plan source identity drift')
    _check_auth_template(auth, request, plan)
    raw_debs = [{key: item[key] for key in ('name', 'version', 'architecture', 'path')}
                for item in request['deb_bindings']]
    try:
        AUTH.verify_authorization_against_policy(
            auth, policy=policy, manifest=candidate,
            candidate_file_sha=candidate_ref['file_sha256'],
            policy_file_sha=policy_ref['file_sha256'],
            candidate_path=candidate_path, policy_path=policy_path, plan=plan,
            root_inputs=Path(request['scope']['root_inputs']), repository=request['repository'],
            deb_bindings=raw_debs,
            executor_source_sha256=request['executor']['file_sha256'],
            now=review_time, require_live_machine=False)
    except Exception as error:
        raise ResponseError(f'signed authorization did not verify offline: {error}') from error
    candidate_again, candidate_again_desc, _ = _descriptor(
        candidate_path, 'candidate manifest recheck', schema=candidate_ref['schema'])
    policy_again, policy_again_desc, _ = _descriptor(
        policy_path, 'READY authorization policy recheck', schema=policy_ref['schema'])
    auth_again, auth_again_desc, _ = _descriptor(
        auth_path, 'signed authorization recheck', schema=auth_ref['schema'])
    if (candidate_again != candidate or candidate_again_desc != candidate_desc or
            policy_again != policy or policy_again_desc != policy_desc or
            auth_again != auth or auth_again_desc != auth_desc):
        raise ResponseError('response source changed during verification')
    expected_verifier = _verifier_descriptor()
    if response['verifier'] != expected_verifier:
        raise ResponseError('response verifier source/backend identity drift')
    expected_auth_binding = {
        'policy_canonical_sha256': policy['canonical_sha256'],
        'candidate_canonical_sha256': candidate['canonical_sha256'],
        'machine_method': (
            auth['machine_identity'].get('method', 'LEGACY_V1')
            if isinstance(auth['machine_identity'], Mapping) else ''),
        'authorization_issued_at': auth['issued_at'],
        'live_host_match': False,
        'authorization_payload_sha256': auth['signature']['payload_sha256'],
    }
    if response['authorization_binding'] != expected_auth_binding:
        raise ResponseError('response authorization verification binding drift')
    return {'status': 'PASS', 'response_sha256': response_desc['file_sha256'],
            'request_sha256': request_desc['file_sha256'],
            'policy_sha256': policy_desc['file_sha256'],
            'authorization_sha256': auth_desc['file_sha256'],
            'machine_method': expected_auth_binding['machine_method'],
            'authorization_issued_at': auth['issued_at'],
            'review_time': review_time, 'authorization_current': True,
            'live_host_match': False, 'benchmark_eligible': False}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    validate = sub.add_parser('validate-response')
    validate.add_argument('--response', type=Path, required=True)
    validate.add_argument('--request', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = validate_response(args.response, args.request)
    except (ResponseError, OSError, HANDOFF.HandoffError, AUTH.AuthorizationError) as error:
        print(f'error: {error}')
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
