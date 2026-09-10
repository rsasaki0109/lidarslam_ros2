#!/usr/bin/env python3
"""Verify a signed custodian decision for the additive rival-source r3 closure.

This is a verifier, not a signer or a promotion tool.  The checked-in trust
policy deliberately has no keys and remains ``NOT_READY``.  A response can be
``ACCEPTED``, ``REJECTED``, or ``NEEDS_CLARIFICATION`` as a durable external
review result, but none of those results switch the active profile, publish a
README claim, or make a benchmark eligible.

The verifier binds the immutable unsigned r3 handoff, the checked-in r3
candidate and selection, the r2 lineage, and one offline legal-capture packet.
The capture packet is allowed to say that remote retrieval was ``NOT_RUN``;
that packet is never treated as legal approval.  An ``ACCEPTED`` result must
instead contain four independent custodian decision records with immutable
evidence and license-text artifacts.  The response nonce is claimed in an
operator-supplied, fixed replay ledger before PASS is returned.  This proves
one-shot use within that ledger; copying an artifact to another ledger is not
claimed to be globally prevented.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import stat
import time
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = Path(__file__).resolve()
SCRIPT_REL = SCRIPT.relative_to(ROOT).as_posix()
CAPTURE_PRODUCER_REL = 'scripts/capture_competitive_rival_legal_provenance.py'
CAPTURE_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_rival_legal_provenance_capture_v1.schema.json')
CANDIDATE_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_rival_source_closure_2026-08-r3-candidate.json')
SELECTION_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_rival_source_closure_2026-08-r3-selection-candidate.json')
HANDOFF_SCHEMA = 'competitive_rival_source_closure_r3_handoff_v1'
CANDIDATE_SCHEMA = 'competitive_rival_source_closure_r3_candidate_v1'
SELECTION_SCHEMA = 'competitive_rival_source_closure_r3_selection_candidate_v1'
CAPTURE_SCHEMA = 'competitive_rival_legal_provenance_capture_v1'
TRUST_POLICY_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_rival_source_closure_r3_custodian_trust_policy.json')
TRUST_POLICY_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_rival_source_closure_r3_custodian_trust_policy_v1.schema.json')
RESPONSE_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_rival_source_closure_r3_custodian_response_v1.schema.json')
SIDECAR_SCHEMA_REL = (
    'configs/slam_benchmark_profiles/'
    'competitive_rival_source_closure_r3_custodian_response_sidecar_v1.schema.json')
RESPONSE_SCHEMA = 'competitive_rival_source_closure_r3_custodian_response_v1'
SIDECAR_SCHEMA = 'competitive_rival_source_closure_r3_custodian_response_sidecar_v1'
TRUST_SCHEMA = 'competitive_rival_source_closure_r3_custodian_trust_policy_v1'
DOMAIN = 'lidarslam/competitive-rival-source-closure-r3/custodian-response/ed25519/v1'
ALGORITHM = 'Ed25519'
BACKEND_NAME = 'python-cryptography'
BACKEND_IMPLEMENTATION = (
    'cryptography.hazmat.primitives.asymmetric.ed25519.Ed25519PublicKey.verify')
MAX_JSON_BYTES = 16 * 1024 * 1024
MAX_EVIDENCE_BYTES = 8 * 1024 * 1024
MAX_SIDE_BYTES = 4096
MAX_WINDOW_SECONDS = 30 * 24 * 60 * 60
SHA_RE = re.compile(r'^[0-9a-f]{64}$')
KEY_ID_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$')
NONCE_RE = re.compile(r'^[A-Za-z0-9_-]{32,128}$')
REVISION_RE = re.compile(r'^[0-9a-f]{40}$')
CAMPAIGN_ID = 'competitive-rival-source-closure-2026-08-r3-candidate'
BLOCKERS = (
    'glim_ros2_license_text_missing',
    'fast_livo2_license_declaration_conflict',
    'rpg_vikit_license_artifacts_incomplete',
    'sophus_license_artifact_missing',
)
BLOCKER_COMPONENTS = {
    'glim_ros2_license_text_missing': 'glim_ros2',
    'fast_livo2_license_declaration_conflict': 'fast_livo2',
    'rpg_vikit_license_artifacts_incomplete': 'rpg_vikit',
    'sophus_license_artifact_missing': 'sophus',
}


class CustodianResponseError(ValueError):
    """A malformed, stale, untrusted, or cross-campaign response."""


class CustodianResponseNotReady(CustodianResponseError):
    """The fixed trust policy intentionally has no active custodian key."""


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise CustodianResponseError(f'cannot load contract: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_CLOSURE = _load(
    'r3_custodian_closure',
    ROOT / 'scripts/validate_competitive_rival_source_closure_r3.py')
_HANDOFF = _load(
    'r3_custodian_handoff',
    ROOT / 'scripts/prepare_competitive_rival_source_closure_r3_handoff.py')
_CAPTURE = _load('r3_custodian_capture', ROOT / CAPTURE_PRODUCER_REL)


def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def canonical_hash(value: Any, excluded: str | None = None) -> str:
    if isinstance(value, Mapping) and excluded is not None:
        value = {str(key): item for key, item in value.items() if key != excluded}
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _sha_value(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise CustodianResponseError(f'{label} is not a lowercase SHA-256')
    return value


def _absolute(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.startswith('/') or '\x00' in value:
        raise CustodianResponseError(f'{label} must be absolute')
    path = Path(value)
    if path.as_posix() != value or path == Path('/') or any(
            part in {'', '.', '..'} for part in value[1:].split('/')):
        raise CustodianResponseError(f'{label} is not normalized')
    return path


def _parent_no_symlink(path: Path, label: str) -> None:
    current = Path(path.anchor)
    for part in path.parent.parts[1:]:
        current /= part
        try:
            info = current.lstat()
        except OSError as error:
            raise CustodianResponseError(f'{label} parent is unavailable') from error
        if stat.S_ISLNK(info.st_mode):
            raise CustodianResponseError(f'{label} contains a symlink parent')


def _regular(path: Path, label: str, *, limit: int,
             immutable: bool = False, exact_mode: int | None = None) -> bytes:
    _parent_no_symlink(path, label)
    try:
        before = path.lstat()
    except OSError as error:
        raise CustodianResponseError(f'{label} cannot be inspected') from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or \
            before.st_nlink != 1 or before.st_size <= 0 or before.st_size > limit:
        raise CustodianResponseError(f'{label} is not a bounded regular single-link file')
    if immutable and before.st_mode & 0o222:
        raise CustodianResponseError(f'{label} is mutable')
    if exact_mode is not None and stat.S_IMODE(before.st_mode) != exact_mode:
        raise CustodianResponseError(f'{label} mode is not {oct(exact_mode)}')
    try:
        with path.open('rb') as stream:
            fd_before = os.fstat(stream.fileno())
            data = stream.read(limit + 1)
            fd_after = os.fstat(stream.fileno())
    except OSError as error:
        raise CustodianResponseError(f'{label} cannot be read') from error
    identity = (before.st_dev, before.st_ino, before.st_nlink, before.st_size)
    if (
            (fd_before.st_dev, fd_before.st_ino, fd_before.st_nlink,
             fd_before.st_size) != identity or
            (fd_after.st_dev, fd_after.st_ino, fd_after.st_nlink,
             fd_after.st_size) != identity or
            len(data) != before.st_size or len(data) > limit):
        raise CustodianResponseError(f'{label} changed while being read')
    return data


def _json(path: Path, label: str, *, immutable: bool = False,
          exact_mode: int | None = None,
          limit: int = MAX_JSON_BYTES) -> tuple[dict[str, Any], bytes]:
    data = _regular(path, label, limit=limit, immutable=immutable,
                    exact_mode=exact_mode)
    try:
        value = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise CustodianResponseError(f'{label} is invalid JSON') from error
    if not isinstance(value, dict):
        raise CustodianResponseError(f'{label} is not an object')
    return value, data


def _descriptor(
        path: Path, label: str, *, schema: str, identity_field: str,
        immutable: bool = False) -> tuple[dict[str, Any], dict[str, Any], bytes]:
    value, data = _json(path, label, immutable=immutable)
    identity = value.get(identity_field)
    if not isinstance(identity, str) or identity != canonical_hash(value, identity_field):
        raise CustodianResponseError(f'{label} canonical identity drift')
    descriptor = {
        'path': str(path), 'file_bytes': len(data), 'file_sha256': _sha(data),
        'canonical_sha256': identity, 'schema': schema,
    }
    return value, descriptor, data


def _json_descriptor(path: Path, label: str, *, schema: str,
                     identity_field: str) -> dict[str, Any]:
    return _descriptor(path, label, schema=schema,
                       identity_field=identity_field)[1]


def _read_sidecar(path: Path, label: str, *, artifact_name: str,
                  artifact_bytes: int, artifact_sha: str,
                  identity_sha: str) -> tuple[dict[str, Any], bytes]:
    sidecar_path = path.with_name(path.name + '.sha256.json')
    sidecar, data = _json(
        sidecar_path, f'{label} sidecar', immutable=True,
        exact_mode=0o444, limit=MAX_SIDE_BYTES)
    required = {'schema', 'schema_version', 'artifact_path', 'artifact_bytes',
                'artifact_sha256', 'canonical_sha256'}
    if (
            set(sidecar) != required or
            sidecar.get('schema') != SIDECAR_SCHEMA or
            sidecar.get('schema_version') != 1 or
            sidecar.get('artifact_path') != artifact_name or
            sidecar.get('artifact_bytes') != artifact_bytes or
            sidecar.get('artifact_sha256') != artifact_sha or
            sidecar.get('canonical_sha256') != identity_sha):
        raise CustodianResponseError(f'{label} sidecar binding drift')
    return sidecar, data


def _sealed_response(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    response, descriptor, data = _descriptor(
        path, 'custodian response', schema=RESPONSE_SCHEMA,
        identity_field='response_identity_sha256', immutable=True)
    if stat.S_IMODE(path.lstat().st_mode) != 0o444:
        raise CustodianResponseError('custodian response must be mode 0444')
    _read_sidecar(path, 'custodian response', artifact_name=path.name,
                  artifact_bytes=len(data), artifact_sha=_sha(data),
                  identity_sha=response['response_identity_sha256'])
    again, again_desc, again_data = _descriptor(
        path, 'custodian response recheck', schema=RESPONSE_SCHEMA,
        identity_field='response_identity_sha256', immutable=True)
    if again != response or again_desc != descriptor or again_data != data:
        raise CustodianResponseError('custodian response changed during validation')
    return response, descriptor


def _binding_descriptor(path: Path, label: str, *, schema: str,
                        identity_field: str) -> tuple[dict[str, Any], dict[str, Any]]:
    value, descriptor, data = _descriptor(
        path, label, schema=schema, identity_field=identity_field)
    return value, {**descriptor, 'data_sha256': _sha(data)}


def _load_fixed_candidate() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    try:
        report = _CLOSURE.validate_checked_in(ROOT)
    except Exception as error:
        raise CustodianResponseError(f'checked-in r3 candidate is not valid: {error}') from error
    candidate_path = ROOT / CANDIDATE_REL
    selection_path = ROOT / SELECTION_REL
    candidate, candidate_desc, candidate_data = _descriptor(
        candidate_path, 'r3 candidate', schema=CANDIDATE_SCHEMA,
        identity_field='candidate_identity_sha256')
    selection, selection_desc, selection_data = _descriptor(
        selection_path, 'r3 selection', schema=SELECTION_SCHEMA,
        identity_field='selection_identity_sha256')
    if report.get('status') != 'NOT_READY' or candidate.get('status') != 'NOT_READY' or \
            selection.get('status') != 'NOT_READY' or \
            candidate.get('benchmark_eligible') is not False or \
            selection.get('benchmark_eligible') is not False or \
            candidate.get('claim_eligible') is not False or \
            selection.get('claim_eligible') is not False:
        raise CustodianResponseError('checked-in candidate/selection is promotable')
    if candidate.get('candidate_id') != CAMPAIGN_ID:
        raise CustodianResponseError('r3 candidate campaign identity drift')
    candidate_binding = selection.get('candidate_binding')
    if (
            not isinstance(candidate_binding, Mapping) or
            candidate_binding.get('candidate_id') != candidate.get('candidate_id') or
            candidate_binding.get('candidate_identity_sha256') !=
            candidate['candidate_identity_sha256'] or
            candidate_binding.get('file_sha256') != candidate_desc['file_sha256']):
        raise CustodianResponseError('r3 selection/candidate binding drift')
    return candidate, selection, {
        'candidate': candidate_desc, 'selection': selection_desc,
        'candidate_bytes': candidate_data, 'selection_bytes': selection_data,
    }


def _load_policy(policy_path: Path, *, candidate: Mapping[str, Any],
                 selection: Mapping[str, Any], source: Mapping[str, Any],
                 review_time: int) -> tuple[dict[str, Any], dict[str, Any]]:
    policy, descriptor, data = _descriptor(
        policy_path, 'custodian trust policy', schema=TRUST_SCHEMA,
        identity_field='policy_identity_sha256')
    if set(policy) != {
            'schema', 'schema_version', 'status', 'benchmark_eligible',
            'candidate_binding', 'selection_binding', 'response_schema_binding',
            'verifier', 'backend', 'authorized_keys', 'nonce_policy',
            'policy_identity_sha256'}:
        raise CustodianResponseError('trust policy field set is not exact')
    if policy.get('schema') != TRUST_SCHEMA or policy.get('schema_version') != 1 or \
            policy.get('benchmark_eligible') is not False or \
            policy.get('policy_identity_sha256') != canonical_hash(
                policy, 'policy_identity_sha256'):
        raise CustodianResponseError('trust policy shape or identity is invalid')
    expected_candidate = {
        'path': CANDIDATE_REL, 'file_sha256': source['candidate']['file_sha256'],
        'canonical_sha256': candidate['candidate_identity_sha256'],
        'candidate_id': candidate['candidate_id'],
    }
    expected_selection = {
        'path': SELECTION_REL, 'file_sha256': source['selection']['file_sha256'],
        'canonical_sha256': selection['selection_identity_sha256'],
        'selection_id': selection['selection_id'],
    }
    if policy.get('candidate_binding') != expected_candidate or \
            policy.get('selection_binding') != expected_selection:
        raise CustodianResponseError('trust policy candidate/selection pin drift')
    schema_path = ROOT / RESPONSE_SCHEMA_REL
    schema_data = _regular(schema_path, 'custodian response schema', limit=MAX_JSON_BYTES)
    expected_schema = {'path': RESPONSE_SCHEMA_REL, 'file_sha256': _sha(schema_data),
                       'schema': RESPONSE_SCHEMA}
    if policy.get('response_schema_binding') != expected_schema:
        raise CustodianResponseError('trust policy response schema binding drift')
    verifier = policy.get('verifier')
    if not isinstance(verifier, Mapping) or set(verifier) != {
            'path', 'file_sha256', 'implementation'} or \
            verifier.get('path') != SCRIPT_REL or \
            verifier.get('file_sha256') != _sha(
                _regular(SCRIPT, 'custodian verifier', limit=MAX_JSON_BYTES)) or \
            verifier.get('implementation') != 'rival_source_closure_custodian_response_v1':
        raise CustodianResponseError('trust policy verifier binding drift')
    backend = policy.get('backend')
    try:
        import cryptography
    except ImportError as error:
        raise CustodianResponseNotReady('Ed25519 backend is unavailable') from error
    if not isinstance(backend, Mapping) or set(backend) != {
            'name', 'version', 'implementation', 'implementation_file',
            'implementation_sha256'} or backend.get('name') != BACKEND_NAME or \
            backend.get('implementation') != BACKEND_IMPLEMENTATION or \
            backend.get('implementation_file') != SCRIPT_REL or \
            backend.get('implementation_sha256') != _sha(
                _regular(SCRIPT, 'custodian verifier', limit=MAX_JSON_BYTES)) or \
            backend.get('version') != getattr(cryptography, '__version__', ''):
        raise CustodianResponseError('trust policy Ed25519 backend binding drift')
    nonce_policy = policy.get('nonce_policy')
    if nonce_policy != {'minimum_length': 32, 'maximum_length': 128,
                        'one_shot_ledger_required': True}:
        raise CustodianResponseError('trust policy nonce contract drift')
    keys = policy.get('authorized_keys')
    if not isinstance(keys, list) or keys != sorted(keys, key=lambda item: item.get('key_id', '')
                                                    if isinstance(item, Mapping) else ''):
        raise CustodianResponseError('trust policy keys are not sorted')
    if policy.get('status') == 'NOT_READY':
        if keys:
            raise CustodianResponseError('NOT_READY trust policy contains keys')
        raise CustodianResponseNotReady('custodian trust policy has no authorized keys')
    if policy.get('status') != 'READY' or not keys:
        raise CustodianResponseError('trust policy is not a usable READY policy')
    parsed: dict[str, dict[str, Any]] = {}
    for index, key in enumerate(keys):
        label = f'authorized_keys[{index}]'
        if not isinstance(key, Mapping) or set(key) != {
                'key_id', 'algorithm', 'status', 'public_key_base64',
                'public_key_sha256', 'not_before', 'not_after'}:
            raise CustodianResponseError(f'{label} is malformed')
        key_id = key.get('key_id')
        if not isinstance(key_id, str) or KEY_ID_RE.fullmatch(key_id) is None or key_id in parsed:
            raise CustodianResponseError(f'{label} key_id is malformed or duplicated')
        if key.get('algorithm') != ALGORITHM or key.get('status') != 'ACTIVE':
            raise CustodianResponseError(f'{label} is not active Ed25519')
        if type(key.get('not_before')) is not int or type(key.get('not_after')) is not int or \
                key['not_before'] < 0 or key['not_after'] <= key['not_before'] or \
                review_time < key['not_before'] or review_time >= key['not_after']:
            raise CustodianResponseError(f'{label} key is outside its validity window')
        try:
            raw_key = base64.b64decode(key['public_key_base64'].encode('ascii'), validate=True)
        except (KeyError, UnicodeError, ValueError) as error:
            raise CustodianResponseError(f'{label} public key is not strict base64') from error
        if len(raw_key) != 32 or _sha(raw_key) != key.get('public_key_sha256'):
            raise CustodianResponseError(f'{label} public key hash drift')
        _sha_value(key.get('public_key_sha256'), f'{label}.public_key_sha256')
        parsed[key_id] = {**dict(key), '_raw': raw_key}
    again, again_descriptor, again_data = _descriptor(
        policy_path, 'custodian trust policy recheck', schema=TRUST_SCHEMA,
        identity_field='policy_identity_sha256')
    if again != policy or again_descriptor != descriptor or again_data != data:
        raise CustodianResponseError('custodian trust policy changed during validation')
    return policy, {'descriptor': descriptor, 'data': data, 'keys': parsed}


def _validate_ref(
        value: Any, label: str, *, schema: str, identity_field: str,
        path: Path | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
    if not isinstance(value, Mapping) or set(value) != {
            'path', 'file_bytes', 'file_sha256', 'canonical_sha256', 'schema'}:
        raise CustodianResponseError(f'{label} descriptor is incomplete')
    if value.get('schema') != schema:
        raise CustodianResponseError(f'{label} schema drift')
    ref_path = _absolute(value.get('path'), f'{label} path')
    if path is not None and ref_path != path:
        raise CustodianResponseError(f'{label} path drift')
    actual, descriptor, _ = _descriptor(ref_path, label, schema=schema,
                                        identity_field=identity_field)
    if descriptor != dict(value):
        raise CustodianResponseError(f'{label} file identity drift')
    return actual, descriptor


def _sealed_external_pair(
        path: Path, label: str, *, expected_schema: str,
        identity_field: str) -> tuple[dict[str, Any], dict[str, Any], bytes]:
    value, descriptor, data = _descriptor(path, label, schema=expected_schema,
                                          identity_field=identity_field,
                                          immutable=True)
    if stat.S_IMODE(path.lstat().st_mode) != 0o444:
        raise CustodianResponseError(f'{label} must be mode 0444')
    sidecar_path = path.with_name(path.name + '.sha256')
    sidecar_data = _regular(
        sidecar_path, f'{label} sidecar', limit=MAX_SIDE_BYTES,
        immutable=True, exact_mode=0o444)
    expected_sidecar = (f'{_sha(data)}  {path.name}\n').encode('ascii')
    if sidecar_data != expected_sidecar:
        raise CustodianResponseError(f'{label} sidecar is stale')
    again, again_desc, again_data = _descriptor(path, f'{label} recheck',
                                                schema=expected_schema,
                                                identity_field=identity_field,
                                                immutable=True)
    if again != value or again_desc != descriptor or again_data != data:
        raise CustodianResponseError(f'{label} changed during validation')
    return value, descriptor, data


def _validate_handoff(ref: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(ref, Mapping) or set(ref) != {
            'path', 'file_bytes', 'file_sha256', 'canonical_sha256', 'schema'} or \
            ref.get('schema') != 'competitive_rival_source_closure_r3_handoff_v1':
        raise CustodianResponseError('handoff descriptor is incomplete')
    path = _absolute(ref.get('path'), 'handoff path')
    value, descriptor, data = _sealed_external_pair(
        path, 'unsigned r3 handoff', expected_schema=HANDOFF_SCHEMA,
        identity_field='handoff_identity_sha256')
    if descriptor != dict(ref) or len(data) != ref.get('file_bytes'):
        raise CustodianResponseError('handoff descriptor drift')
    try:
        report = _HANDOFF.validate_handoff(path, root=ROOT)
    except Exception as error:
        raise CustodianResponseError(f'unsigned handoff is stale: {error}') from error
    if (
            report.get('status') != 'UNSIGNED_REVIEW_REQUIRED' or
            value.get('benchmark_eligible') is not False or
            value.get('claim_eligible') is not False):
        raise CustodianResponseError('handoff is promotable or malformed')
    return value


def _validate_capture(capture_ref: Mapping[str, Any], candidate: Mapping[str, Any],
                      selection: Mapping[str, Any]) -> dict[str, Any]:
    required = {
        'path', 'file_bytes', 'file_sha256', 'capture_identity_sha256',
        'schema', 'status', 'review_status', 'remote_responses_status',
        'closure_identity_sha256', 'selection_sha256', 'producer', 'schema_binding'}
    if not isinstance(capture_ref, Mapping) or set(capture_ref) != required or \
            capture_ref.get('schema') != CAPTURE_SCHEMA:
        raise CustodianResponseError('legal capture binding shape is not exact')
    path = _absolute(capture_ref.get('path'), 'legal capture path')
    if path.name != _CAPTURE.CAPTURE_FILE:
        raise CustodianResponseError('legal capture path is not the fixed packet name')
    capture, descriptor, data = _sealed_external_pair(
        path, 'legal capture packet', expected_schema=CAPTURE_SCHEMA,
        identity_field='capture_identity_sha256')
    if descriptor['file_sha256'] != capture_ref.get('file_sha256') or \
            descriptor['file_bytes'] != capture_ref.get('file_bytes') or \
            capture.get('capture_identity_sha256') != capture_ref.get('capture_identity_sha256'):
        raise CustodianResponseError('legal capture descriptor drift')
    if capture.get('status') != 'REVIEW_REQUIRED' or \
            capture.get('review_status') != 'UNSIGNED_REVIEW_REQUIRED' or \
            capture.get('benchmark_eligible') is not False or \
            capture.get('claim_eligible') is not False:
        raise CustodianResponseError('legal capture is unexpectedly promotable')
    remote = capture.get('remote_policy')
    if not isinstance(remote, Mapping) or remote.get('responses_status') != 'NOT_RUN' or \
            remote.get('network_used') is not False:
        raise CustodianResponseError('legal capture remote phase is not NOT_RUN')
    closure = capture.get('closure_binding')
    if (
            not isinstance(closure, Mapping) or
            closure.get('closure_identity_sha256') !=
            candidate['r2_lineage']['closure_identity_sha256'] or
            closure.get('selection_sha256') !=
            candidate['r2_lineage']['r2_selection_sha256'] or
            closure.get('selection_id') !=
            'competitive-execution-selection-2026-08-r2'):
        raise CustodianResponseError('legal capture closure/selection binding drift')
    contract = candidate.get('legal_capture_contract')
    if not isinstance(contract, Mapping):
        raise CustodianResponseError('candidate legal capture contract is missing')
    producer = contract.get('producer')
    schema = contract.get('schema')
    if not isinstance(producer, Mapping) or not isinstance(schema, Mapping):
        raise CustodianResponseError('candidate capture producer/schema contract is incomplete')
    producer_path = ROOT / CAPTURE_PRODUCER_REL
    schema_path = ROOT / CAPTURE_SCHEMA_REL
    producer_sha = _sha(_regular(producer_path, 'capture producer', limit=MAX_JSON_BYTES))
    schema_sha = _sha(_regular(schema_path, 'capture schema', limit=MAX_JSON_BYTES))
    expected_producer = {'path': CAPTURE_PRODUCER_REL, 'file_sha256': producer_sha,
                         'status': 'CONTRACT_BOUND_NOT_REVIEWED'}
    expected_schema = {'path': CAPTURE_SCHEMA_REL, 'file_sha256': schema_sha,
                       'status': 'CONTRACT_BOUND_NOT_REVIEWED'}
    if producer != expected_producer or schema != expected_schema:
        raise CustodianResponseError('candidate capture producer/schema drift')
    if capture_ref.get('producer') != expected_producer or \
            capture_ref.get('schema_binding') != expected_schema:
        raise CustodianResponseError('response capture producer/schema binding drift')
    try:
        _CAPTURE.validate_capture_set(output_dir=path.parent, root=ROOT)
    except Exception as error:
        raise CustodianResponseError(
            f'legal capture packet failed exact validation: {error}') from error
    again, again_desc, again_data = _sealed_external_pair(
        path, 'legal capture packet recheck', expected_schema=CAPTURE_SCHEMA,
        identity_field='capture_identity_sha256')
    if again != capture or again_desc != descriptor or again_data != data:
        raise CustodianResponseError('legal capture changed during validation')
    if capture_ref.get('status') != capture['status'] or \
            capture_ref.get('review_status') != capture['review_status'] or \
            capture_ref.get('remote_responses_status') != remote['responses_status'] or \
            capture_ref.get('closure_identity_sha256') != closure['closure_identity_sha256'] or \
            capture_ref.get('selection_sha256') != closure['selection_sha256']:
        raise CustodianResponseError('response capture projection drift')
    return capture


def _evidence_descriptor(value: Any, label: str) -> bytes:
    if not isinstance(value, Mapping) or set(value) != {
            'kind', 'path', 'file_bytes', 'file_sha256'}:
        raise CustodianResponseError(f'{label} evidence shape is not exact')
    if value.get('kind') not in {'LICENSE_TEXT', 'CUSTODIAN_REVIEW_NOTE'}:
        raise CustodianResponseError(f'{label} evidence kind is invalid')
    path = _absolute(value.get('path'), f'{label} evidence path')
    data = _regular(path, label, limit=MAX_EVIDENCE_BYTES, immutable=True,
                    exact_mode=0o444)
    if len(data) != value.get('file_bytes') or _sha(data) != value.get('file_sha256'):
        raise CustodianResponseError(f'{label} evidence identity drift')
    reread = _regular(path, f'{label} recheck', limit=MAX_EVIDENCE_BYTES,
                      immutable=True, exact_mode=0o444)
    if reread != data:
        raise CustodianResponseError(f'{label} evidence changed during validation')
    if value.get('kind') == 'LICENSE_TEXT' and path.name.lower() in {
            'package.xml', 'package.json', 'manifest.xml'}:
        raise CustodianResponseError(f'{label} package declaration is not license text')
    try:
        text = data.decode('utf-8')
    except UnicodeDecodeError as error:
        raise CustodianResponseError(f'{label} evidence is not UTF-8 text') from error
    if not text.strip() or len(text) > MAX_EVIDENCE_BYTES:
        raise CustodianResponseError(f'{label} evidence text is empty or oversized')
    return data


def _source_identity_for_blocker(
        candidate: Mapping[str, Any], blocker: str) -> tuple[str, str, str]:
    closure = candidate.get('r2_inherited_closure')
    rivals = closure.get('rivals') if isinstance(closure, Mapping) else None
    if not isinstance(rivals, Mapping):
        raise CustodianResponseError('candidate inherited source closure is missing')
    component = BLOCKER_COMPONENTS[blocker]
    matches: list[Mapping[str, Any]] = []
    for rival in rivals.values():
        if not isinstance(rival, Mapping):
            continue
        for source in rival.get('sources', []):
            if isinstance(source, Mapping) and source.get('name') == component:
                matches.append(source)
    if len(matches) != 1:
        raise CustodianResponseError(f'{blocker} does not have one fixed upstream source')
    source = matches[0]
    repository = source.get('repository_url')
    revision = source.get('revision')
    if (
            not isinstance(repository, str) or
            not repository.startswith('https://github.com/') or
            not isinstance(revision, str) or
            REVISION_RE.fullmatch(revision) is None):
        raise CustodianResponseError(f'{blocker} upstream source identity is malformed')
    return component, repository, revision


def _validate_decision(value: Any, index: int,
                       candidate: Mapping[str, Any]) -> dict[str, Any]:
    label = f'blocker_decisions[{index}]'
    required = {'blocker_id', 'decision', 'evidence', 'license_identity',
                'allowed_use', 'redistribution_policy', 'publication_policy',
                'review_note'}
    if not isinstance(value, Mapping) or set(value) != required:
        raise CustodianResponseError(f'{label} fields are not exact')
    blocker = value.get('blocker_id')
    if blocker not in BLOCKERS:
        raise CustodianResponseError(f'{label} blocker is unknown')
    if value.get('decision') not in {'ACCEPTED', 'REJECTED', 'NEEDS_CLARIFICATION'}:
        raise CustodianResponseError(f'{label} decision is invalid')
    evidence = value.get('evidence')
    if not isinstance(evidence, list) or not evidence or len(evidence) > 8:
        raise CustodianResponseError(f'{label} evidence is incomplete')
    kinds = []
    for evidence_index, item in enumerate(evidence):
        _evidence_descriptor(item, f'{label}.evidence[{evidence_index}]')
        kinds.append(item['kind'])
    if value['decision'] == 'ACCEPTED' and 'LICENSE_TEXT' not in kinds:
        raise CustodianResponseError(f'{label} accepted without immutable license text')
    if value['decision'] != 'ACCEPTED' and 'CUSTODIAN_REVIEW_NOTE' not in kinds:
        raise CustodianResponseError(f'{label} non-accepted result lacks review note evidence')
    license_identity = value.get('license_identity')
    if not isinstance(license_identity, Mapping) or set(license_identity) != {
            'component', 'status', 'upstream_url', 'revision', 'license_text',
            'declaration_is_not_license_text'}:
        raise CustodianResponseError(f'{label} license identity is incomplete')
    component, repository, revision = _source_identity_for_blocker(candidate, blocker)
    if license_identity.get('component') != component or \
            license_identity.get('status') not in {
                'ESTABLISHED', 'CONFLICT', 'MISSING', 'UNRESOLVED'} or \
            license_identity.get('upstream_url') != repository or \
            license_identity.get('revision') != revision or \
            license_identity.get('declaration_is_not_license_text') is not True:
        raise CustodianResponseError(f'{label} license identity is invalid')
    license_text = license_identity.get('license_text')
    if value['decision'] == 'ACCEPTED':
        if license_identity['status'] != 'ESTABLISHED' or not isinstance(license_text, Mapping):
            raise CustodianResponseError(f'{label} accepted license identity is not established')
        _evidence_descriptor(license_text, f'{label}.license_text')
        if license_text.get('kind') != 'LICENSE_TEXT' or license_text not in evidence:
            raise CustodianResponseError(f'{label} license text is not evidence-bound')
    elif license_text is not None:
        raise CustodianResponseError(f'{label} non-accepted license text projection is ambiguous')
    for field in ('allowed_use', 'redistribution_policy', 'publication_policy'):
        mapping = value.get(field)
        expected = {
            'build': 'ALLOWED', 'execute': 'ALLOWED',
            'source': 'ALLOWED', 'binary': 'ALLOWED', 'benchmark_claim': 'ALLOWED'}
        if not isinstance(mapping, Mapping):
            raise CustodianResponseError(f'{label}.{field} is missing')
        if field == 'allowed_use':
            keys = {'build', 'execute'}
        elif field == 'redistribution_policy':
            keys = {'source', 'binary'}
        else:
            keys = {'source', 'binary', 'benchmark_claim'}
        if set(mapping) != keys or any(mapping[key] not in {
                'ALLOWED', 'DENIED', 'CONDITIONAL', 'UNKNOWN'} for key in keys):
            raise CustodianResponseError(f'{label}.{field} is not an exact policy')
        del expected
    if not isinstance(value.get('review_note'), str) or not value['review_note'].strip() or \
            len(value['review_note']) > 4096:
        raise CustodianResponseError(f'{label} review note is invalid')
    return dict(value)


def _signature_payload(response: Mapping[str, Any]) -> bytes:
    signature = response.get('signature')
    if not isinstance(signature, Mapping):
        raise CustodianResponseError('response signature metadata is missing')
    metadata = {key: value for key, value in signature.items()
                if key not in {'signature_base64', 'payload_sha256'}}
    unsigned = {key: value for key, value in response.items()
                if key not in {'signature', 'response_identity_sha256'}}
    unsigned['signature_metadata'] = metadata
    return canonical_bytes({'domain': DOMAIN, 'schema_version': 1,
                            'response': unsigned})


def _decode_b64(value: Any, label: str, size: int) -> bytes:
    if not isinstance(value, str) or not value:
        raise CustodianResponseError(f'{label} is missing')
    try:
        decoded = base64.b64decode(value.encode('ascii'), validate=True)
    except (UnicodeError, ValueError) as error:
        raise CustodianResponseError(f'{label} is not strict base64') from error
    if len(decoded) != size:
        raise CustodianResponseError(f'{label} has invalid length')
    return decoded


def _claim_nonce(ledger: Path, nonce: str, response_identity: str) -> None:
    if not ledger.is_absolute() or ledger.as_posix() != str(ledger) or ledger == Path('/'):
        raise CustodianResponseError('replay ledger path is not absolute/normalized')
    _parent_no_symlink(ledger / 'placeholder', 'replay ledger')
    if ledger.exists() and (ledger.is_symlink() or not ledger.is_dir()):
        raise CustodianResponseError('replay ledger is not a real directory')
    if not ledger.exists():
        raise CustodianResponseError('replay ledger must be pre-created')
    if any(part in {'', '.', '..'} for part in ledger.parts[1:]):
        raise CustodianResponseError('replay ledger path traversal')
    claim = ledger / (nonce + '.claim.json')
    if claim.exists() or claim.is_symlink():
        raise CustodianResponseError('response nonce has already been claimed')
    payload = canonical_bytes({'schema': 'competitive_rival_r3_nonce_claim_v1',
                               'campaign_id': CAMPAIGN_ID, 'nonce': nonce,
                               'response_identity_sha256': response_identity})
    fd = None
    owned: tuple[int, int] | None = None
    claim_identity: tuple[int, int, int, int] | None = None
    try:
        fd = os.open(str(claim), os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o444)
        initial = os.fstat(fd)
        if not stat.S_ISREG(initial.st_mode) or initial.st_nlink != 1:
            raise CustodianResponseError('replay nonce claim is not a regular single-link file')
        owned = (initial.st_dev, initial.st_ino)
        offset = 0
        while offset < len(payload):
            written = os.write(fd, payload[offset:])
            if written <= 0:
                raise CustodianResponseError('replay nonce claim write made no progress')
            offset += written
        os.fchmod(fd, 0o444)
        os.fsync(fd)
        final = os.fstat(fd)
        claim_identity = (final.st_dev, final.st_ino, final.st_nlink, final.st_size)
        if (final.st_dev, final.st_ino) != owned or final.st_nlink != 1 or \
                final.st_size != len(payload) or stat.S_IMODE(final.st_mode) != 0o444:
            raise CustodianResponseError('replay nonce claim identity drift')
        os.close(fd)
        fd = None
        observed = _regular(claim, 'replay nonce claim', limit=MAX_SIDE_BYTES,
                            immutable=True, exact_mode=0o444)
        if observed != payload:
            raise CustodianResponseError('replay nonce claim content drift')
        parent_fd = os.open(str(ledger), os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            os.fsync(parent_fd)
        finally:
            os.close(parent_fd)
    except FileExistsError as error:
        raise CustodianResponseError('response nonce has already been claimed') from error
    except OSError as error:
        raise CustodianResponseError(f'cannot seal replay nonce claim: {error}') from error
    finally:
        if fd is not None:
            os.close(fd)
        if claim_identity is None and owned is not None:
            try:
                current = claim.lstat()
                if (current.st_dev, current.st_ino) == owned and current.st_nlink == 1:
                    claim.unlink()
            except OSError:
                pass


def _verify_signature(response: Mapping[str, Any], policy_state: Mapping[str, Any],
                      review_time: int) -> dict[str, Any]:
    signature = response.get('signature')
    if not isinstance(signature, Mapping) or set(signature) != {
            'algorithm', 'key_id', 'public_key_sha256', 'payload_sha256',
            'signature_base64', 'backend'}:
        raise CustodianResponseError('response signature fields are not exact')
    if (signature.get('algorithm') != ALGORITHM or
            signature.get('backend') != policy_state['policy']['backend']):
        raise CustodianResponseError('response signature algorithm/backend drift')
    key = policy_state['keys'].get(signature.get('key_id'))
    if key is None:
        raise CustodianResponseError('response key is not in the fixed trust policy')
    if signature.get('public_key_sha256') != key['public_key_sha256']:
        raise CustodianResponseError('response public-key identity drift')
    payload = _signature_payload(response)
    payload_sha = _sha(payload)
    if signature.get('payload_sha256') != payload_sha:
        raise CustodianResponseError('response signature payload hash drift')
    signed = _decode_b64(signature.get('signature_base64'), 'response signature', 64)
    try:
        from cryptography.exceptions import InvalidSignature
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
        Ed25519PublicKey.from_public_bytes(key['_raw']).verify(signed, payload)
    except InvalidSignature as error:
        raise CustodianResponseError('response Ed25519 signature is invalid') from error
    except (TypeError, ValueError) as error:
        raise CustodianResponseError('response Ed25519 signature cannot be verified') from error
    del review_time
    return {'key_id': key['key_id'], 'public_key_sha256': key['public_key_sha256'],
            'payload_sha256': payload_sha}


def _expected_verifier() -> dict[str, Any]:
    import cryptography
    source_sha = _sha(_regular(SCRIPT, 'custodian verifier', limit=MAX_JSON_BYTES))
    return {
        'path': SCRIPT_REL, 'file_sha256': source_sha,
        'implementation': 'rival_source_closure_custodian_response_v1',
        'backend': {
            'name': BACKEND_NAME,
            'version': getattr(cryptography, '__version__', ''),
            'implementation': BACKEND_IMPLEMENTATION,
            'implementation_file': SCRIPT_REL,
            'implementation_sha256': source_sha,
        },
    }


def validate_response(response_path: Path, *, replay_ledger: Path,
                      _review_time: int | None = None,
                      _policy_path: Path | None = None) -> dict[str, Any]:
    """Validate one signed response at the verifier's actual review time.

    ``_review_time`` and ``_policy_path`` are test-only seams.  The CLI uses
    the current wall clock and the fixed checked-in policy path.  No signer or
    arbitrary trust root is exposed by the production CLI.
    """
    review_time = int(time.time()) if _review_time is None else _review_time
    if type(review_time) is not int or review_time < 0:
        raise CustodianResponseError('review_time is not a nonnegative integer')
    response, response_desc = _sealed_response(_absolute(str(response_path), 'response'))
    required = {
        'schema', 'schema_version', 'decision_status', 'benchmark_eligible',
        'claim_eligible', 'active_profile_switch', 'campaign_id', 'nonce',
        'issued_at', 'expires_at', 'handoff', 'candidate', 'selection',
        'trust_policy', 'capture', 'r2_lineage', 'blocker_decisions',
        'signature', 'verifier', 'response_identity_sha256'}
    if set(response) != required or response.get('schema') != RESPONSE_SCHEMA or \
            response.get('schema_version') != 1 or \
            response.get('benchmark_eligible') is not False or \
            response.get('claim_eligible') is not False or \
            response.get('active_profile_switch') is not False or \
            response.get('campaign_id') != CAMPAIGN_ID or \
            response.get('response_identity_sha256') != canonical_hash(
                response, 'response_identity_sha256'):
        raise CustodianResponseError('response shape/status/identity is invalid')
    if response.get('decision_status') not in {'ACCEPTED', 'REJECTED', 'NEEDS_CLARIFICATION'}:
        raise CustodianResponseError('response decision status is invalid')
    if not isinstance(response.get('nonce'), str) or NONCE_RE.fullmatch(response['nonce']) is None:
        raise CustodianResponseError('response nonce is malformed')
    if (
            type(response.get('issued_at')) is not int or
            type(response.get('expires_at')) is not int or
            response['issued_at'] < 0 or
            response['expires_at'] <= response['issued_at'] or
            response['expires_at'] - response['issued_at'] > MAX_WINDOW_SECONDS or
            review_time < response['issued_at'] or
            review_time >= response['expires_at']):
        raise CustodianResponseError(
            'response is expired, not-yet-valid, or has an excessive window')
    candidate, selection, source = _load_fixed_candidate()
    candidate_actual, candidate_ref = _validate_ref(
        response['candidate'], 'response candidate', schema=CANDIDATE_SCHEMA,
        identity_field='candidate_identity_sha256', path=ROOT / CANDIDATE_REL)
    selection_actual, selection_ref = _validate_ref(
        response['selection'], 'response selection', schema=SELECTION_SCHEMA,
        identity_field='selection_identity_sha256', path=ROOT / SELECTION_REL)
    if candidate_actual != candidate or selection_actual != selection or \
            candidate_ref != source['candidate'] or selection_ref != source['selection']:
        raise CustodianResponseError('response candidate/selection source drift')
    if response.get('r2_lineage') != candidate.get('r2_lineage'):
        raise CustodianResponseError('response r2 lineage drift')
    _validate_handoff(response['handoff'])
    capture = _validate_capture(response['capture'], candidate, selection)
    capture_ref = response['capture']
    if capture_ref.get('capture_identity_sha256') != capture.get('capture_identity_sha256'):
        raise CustodianResponseError('response capture identity drift')
    decisions = response.get('blocker_decisions')
    if (
            not isinstance(decisions, list) or len(decisions) != len(BLOCKERS) or
            [item.get('blocker_id') for item in decisions
             if isinstance(item, Mapping)] != list(BLOCKERS)):
        raise CustodianResponseError(
            'response does not cover exactly the four legal blockers in order')
    for index, decision in enumerate(decisions):
        _validate_decision(decision, index, candidate)
    if response['decision_status'] == 'ACCEPTED' and any(
            item['decision'] != 'ACCEPTED' for item in decisions):
        raise CustodianResponseError('ACCEPTED response has a non-accepted blocker')
    if response['decision_status'] != 'ACCEPTED' and all(
            item['decision'] == 'ACCEPTED' for item in decisions):
        raise CustodianResponseError('non-accepted response has only accepted blockers')
    policy_path = _absolute(str(_policy_path or ROOT / TRUST_POLICY_REL), 'trust policy')
    policy, policy_state = _load_policy(policy_path, candidate=candidate,
                                        selection=selection, source=source,
                                        review_time=review_time)
    trust_ref = response.get('trust_policy')
    if not isinstance(trust_ref, Mapping) or trust_ref != policy_state['descriptor']:
        raise CustodianResponseError('response trust-policy descriptor drift')
    expected_verifier = _expected_verifier()
    if response.get('verifier') != expected_verifier:
        raise CustodianResponseError('response verifier identity drift')
    signature_result = _verify_signature(response, {**policy_state, 'policy': policy}, review_time)
    _claim_nonce(replay_ledger, response['nonce'], response['response_identity_sha256'])
    legal_clearance = response['decision_status'] == 'ACCEPTED' and all(
        item['allowed_use']['build'] == 'ALLOWED' and
        item['allowed_use']['execute'] == 'ALLOWED' and
        item['publication_policy']['benchmark_claim'] == 'ALLOWED'
        for item in decisions)
    return {
        'status': response['decision_status'], 'verified': True,
        'response_sha256': response_desc['file_sha256'],
        'response_identity_sha256': response['response_identity_sha256'],
        'candidate_identity_sha256': candidate['candidate_identity_sha256'],
        'selection_identity_sha256': selection['selection_identity_sha256'],
        'capture_identity_sha256': capture['capture_identity_sha256'],
        'key_id': signature_result['key_id'],
        'issued_at': response['issued_at'], 'expires_at': response['expires_at'],
        'review_time': review_time, 'replay_protected': True,
        'benchmark_eligible': False, 'claim_eligible': False,
        'active_profile_switch': False,
        'promotion': 'FORBIDDEN_UNTIL_CANDIDATE_AND_PROFILE_RESEALED',
        'capture_remote_status': capture['remote_policy']['responses_status'],
        'legal_clearance': legal_clearance,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    validate = sub.add_parser('validate-response')
    validate.add_argument('--response', type=Path, required=True)
    validate.add_argument('--replay-ledger', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = validate_response(args.response, replay_ledger=args.replay_ledger)
    except CustodianResponseNotReady as error:
        print(json.dumps({'status': 'NOT_READY', 'benchmark_eligible': False,
                          'error': str(error)}, sort_keys=True))
        return 3
    except (CustodianResponseError, OSError, ValueError, TypeError) as error:
        print(json.dumps({'status': 'INVALID', 'benchmark_eligible': False,
                          'error': str(error)}, sort_keys=True))
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
