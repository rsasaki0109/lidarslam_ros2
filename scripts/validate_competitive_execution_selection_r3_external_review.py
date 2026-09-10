#!/usr/bin/env python3
# flake8: noqa
"""Verify a signed, non-promoting review of the r3 image/toolchain capture.

There is deliberately no signer or key-generation path here.  The checked-in
trust policy has no keys and remains NOT_READY.  An external READY policy is
accepted only through the explicit validation seam used by an external
custodian handoff; even a valid response is offline review evidence and cannot
switch the active profile or make a benchmark claim eligible.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
from pathlib import Path
import os
import re
import stat
import sys
import time
from typing import Any, Mapping

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from scripts import capture_competitive_execution_selection_r3_external as capture  # noqa: E402
from scripts import prepare_competitive_execution_selection_r3_handoff as sealed  # noqa: E402
from scripts.validate_competitive_execution_selection_r3 import (  # noqa: E402
    CANDIDATE_REL,
    ROOT,
    validate_candidate,
)


RESPONSE_SCHEMA = 'competitive_execution_selection_r3_external_review_response_v1'
SIDECAR_SCHEMA = 'competitive_execution_selection_r3_external_review_sidecar_v1'
TRUST_SCHEMA = 'competitive_execution_selection_r3_external_review_trust_policy_v1'
RESPONSE_SCHEMA_REL = 'configs/slam_benchmark_profiles/competitive_execution_selection_r3_external_review_response_v1.schema.json'
TRUST_POLICY_REL = 'configs/slam_benchmark_profiles/competitive_execution_selection_r3_external_review_trust_policy.json'
TRUST_SCHEMA_REL = 'configs/slam_benchmark_profiles/competitive_execution_selection_r3_external_review_trust_policy_v1.schema.json'
CAPTURE_SCHEMA_REL = 'configs/slam_benchmark_profiles/competitive_execution_selection_r3_external_capture_v1.schema.json'
CAPTURE_SCHEMA_PATH = ROOT / CAPTURE_SCHEMA_REL
CAPTURE_SCRIPT = ROOT / 'scripts/capture_competitive_execution_selection_r3_external.py'
SCRIPT = Path(__file__).resolve()
SCRIPT_REL = SCRIPT.relative_to(ROOT).as_posix()
FIXED_POLICY = ROOT / TRUST_POLICY_REL
CAMPAIGN_ID = 'competitive-execution-selection-2026-08-r3-candidate'
SYSTEMS = ('ours', 'glim', 'fast_livo2')
PROBES = ('compiler', 'linker', 'ros_distro', 'pcl', 'eigen', 'openmp')
NOT_APPLICABLE = {'ours': (), 'glim': ('pcl',), 'fast_livo2': ()}
ALGORITHM = 'Ed25519'
BACKEND_NAME = 'python-cryptography'
BACKEND_IMPLEMENTATION = 'cryptography.hazmat.primitives.asymmetric.ed25519.Ed25519PublicKey.verify'
DOMAIN = 'lidarslam/competitive-execution-selection-r3/external-review/ed25519/v1'
MAX_BYTES = 16 * 1024 * 1024
MAX_SIDE_BYTES = 4096
MAX_WINDOW = 30 * 24 * 60 * 60
NONCE_RE = re.compile(r'^[A-Za-z0-9_-]{32,128}$')


class ExternalReviewError(ValueError):
    """Malformed, stale, untrusted, or non-promoting review input."""


class ExternalReviewNotReady(ExternalReviewError):
    """The fixed NOT_READY policy cannot authorize a response."""


def _canonical(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def _hash(value: Any) -> str:
    return hashlib.sha256(value if isinstance(value, bytes) else _canonical(value)).hexdigest()


def _absolute(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.startswith('/') or '\x00' in value:
        raise ExternalReviewError(f'{label} must be absolute')
    path = Path(value)
    if path.as_posix() != value or path == Path('/') or '..' in path.parts:
        raise ExternalReviewError(f'{label} is not normalized')
    return path


def _read(path: Path, label: str, *, limit: int = MAX_BYTES,
          immutable: bool = False, mode: int | None = None) -> tuple[bytes, dict[str, int]]:
    try:
        identity = sealed._file_identity(path, label, max_bytes=limit,
                                         read_only=immutable)
    except Exception as error:
        raise ExternalReviewError(str(error)) from error
    if mode is not None and identity['mode'] != mode:
        raise ExternalReviewError(f'{label} mode is not {oct(mode)}')
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0)
    try:
        fd = os.open(path, flags)
        before = os.fstat(fd)
        chunks: list[bytes] = []
        total = 0
        while True:
            chunk = os.read(fd, min(1024 * 1024, limit + 1 - total))
            if not chunk:
                break
            chunks.append(chunk)
            total += len(chunk)
            if total > limit:
                raise ExternalReviewError(f'{label} is oversized')
        after = os.fstat(fd)
    except OSError as error:
        raise ExternalReviewError(f'{label} cannot be read safely') from error
    finally:
        try:
            os.close(fd)
        except (UnboundLocalError, OSError):
            pass
    expected = (identity['device'], identity['inode'], identity['nlink'],
                identity['size'], identity['mode'])
    observed_before = (before.st_dev, before.st_ino, before.st_nlink,
                       before.st_size, stat.S_IMODE(before.st_mode))
    observed_after = (after.st_dev, after.st_ino, after.st_nlink,
                      after.st_size, stat.S_IMODE(after.st_mode))
    if observed_before != expected or observed_after != expected or total != identity['size']:
        raise ExternalReviewError(f'{label} changed while being read')
    again = sealed._file_identity(path, label, max_bytes=limit, read_only=immutable)
    if dict(again) != dict(identity):
        raise ExternalReviewError(f'{label} changed after being read')
    return b''.join(chunks), identity


def _json(path: Path, label: str, *, limit: int = MAX_BYTES,
          immutable: bool = False, mode: int | None = None) -> tuple[dict[str, Any], bytes, dict[str, int]]:
    data, identity = _read(path, label, limit=limit, immutable=immutable, mode=mode)
    try:
        value = json.loads(data.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ExternalReviewError(f'{label} is invalid JSON') from error
    if not isinstance(value, dict):
        raise ExternalReviewError(f'{label} is not an object')
    return value, data, identity


def _pair(path: Path, label: str, schema: str, identity_field: str,
          side_schema: str = SIDECAR_SCHEMA) -> tuple[dict[str, Any], bytes, dict[str, int]]:
    value, data, identity = _json(path, label, immutable=True, mode=0o444)
    if value.get('schema') != schema or value.get('schema_version') != 1 or \
            value.get(identity_field) != _hash({key: item for key, item in value.items()
                                                if key != identity_field}):
        raise ExternalReviewError(f'{label} schema/canonical identity is invalid')
    side_path = path.with_name(path.name + '.sha256.json')
    side, side_data, _ = _json(side_path, f'{label} sidecar', limit=MAX_SIDE_BYTES,
                                immutable=True, mode=0o444)
    expected = {'schema': side_schema, 'schema_version': 1,
                'artifact_path': path.name, 'artifact_bytes': len(data),
                'artifact_sha256': _hash(data),
                'canonical_sha256': value[identity_field]}
    if side != expected:
        raise ExternalReviewError(f'{label} sidecar binding drift')
    value2, data2, identity2 = _json(path, f'{label} recheck', immutable=True, mode=0o444)
    side2, side_data2, _ = _json(side_path, f'{label} sidecar recheck', limit=MAX_SIDE_BYTES,
                                 immutable=True, mode=0o444)
    if value2 != value or data2 != data or identity2 != identity or side2 != side or side_data2 != side_data:
        raise ExternalReviewError(f'{label} changed during validation')
    return value, data, identity


def _candidate() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    path = ROOT / CANDIDATE_REL
    report = validate_candidate(path)
    if report.get('status') != 'NOT_READY' or report.get('benchmark_eligible') is not False or \
            report.get('claim_eligible') is not False or not report.get('structural_valid'):
        raise ExternalReviewError('r3 candidate is not non-promoting')
    data, _ = _read(path, 'r3 candidate')
    candidate = json.loads(data.decode('utf-8'))
    binding = {'path': CANDIDATE_REL, 'file_sha256': _hash(data),
               'candidate_identity_sha256': candidate['candidate_identity_sha256'],
               'candidate_id': candidate['candidate_id'], 'status': candidate['status']}
    profile = candidate['profile_binding']
    r2 = candidate['r2_selection_binding']
    selection = {
        'profile_path': profile['path'], 'profile_file_sha256': profile['file_sha256'],
        'profile_canonical_sha256': profile['canonical_sha256'],
        'r2_selection_path': r2['path'], 'r2_selection_file_sha256': r2['file_sha256'],
        'r2_selection_id': r2['selection_id'], 'r2_closure_id': r2['closure_id'],
        'r2_closure_identity_sha256': r2['closure_identity_sha256']}
    context = {
        'required_systems': list(candidate['required_systems']), 'release': 'Release',
        'thread_policy_sha256': candidate['thread_policy_binding']['canonical_sha256'],
        'image_digests': {s: candidate['system_bindings'][s]['image_digest'] for s in SYSTEMS},
        'source_bindings_sha256': _hash(candidate['source_bindings']),
        'system_bindings_sha256': _hash(candidate['system_bindings'])}
    return candidate, binding, {'selection': selection, 'context': context}


def _source_ref(path: Path, label: str, relative: bool = True) -> dict[str, str]:
    data, _ = _read(path, label)
    return {'path': path.relative_to(ROOT).as_posix() if relative else str(path),
            'file_sha256': _hash(data)}


def _capture_producer() -> dict[str, str]:
    source = _source_ref(CAPTURE_SCRIPT, 'capture producer')
    schema = _source_ref(CAPTURE_SCHEMA_PATH, 'capture schema')
    return {'path': source['path'], 'file_sha256': source['file_sha256'],
            'schema_path': schema['path'], 'schema_file_sha256': schema['file_sha256'],
            'schema': capture.CAPTURE_MANIFEST_KIND}


def _capture_binding(root: Path, expected_candidate: Mapping[str, Any],
                    context: Mapping[str, Any]) -> dict[str, Any]:
    root = _absolute(str(root), 'capture root')
    root_id = sealed._directory_identity(root, 'capture root')
    try:
        capture.validate_capture_set(root, candidate_path=ROOT / CANDIDATE_REL)
    except Exception as error:
        raise ExternalReviewError(f'capture validation failed: {error}') from error
    manifest, manifest_sha = capture._read_pair(root, 'capture_manifest.json')
    image, image_sha = capture._read_pair(root, 'image_inspect.json')
    tool, tool_sha = capture._read_pair(root, 'toolchain_capture.json')
    if manifest.get('status') != 'UNREVIEWED_COMPLETE_OBSERVATION' or \
            image.get('status') != 'UNREVIEWED_COMPLETE_OBSERVATION' or \
            tool.get('status') != 'UNREVIEWED_COMPLETE_OBSERVATION' or \
            tool.get('probe_requested') is not True:
        raise ExternalReviewError('capture is partial, pending, or not explicitly probed')
    capture_candidate = {
        key: expected_candidate[key] for key in (
            'path', 'file_sha256', 'candidate_identity_sha256', 'candidate_id')}
    if manifest.get('candidate_binding') != capture_candidate or \
            image.get('candidate_binding') != capture_candidate or \
            tool.get('candidate_binding') != capture_candidate:
        raise ExternalReviewError('capture candidate binding drift')
    if manifest.get('image_inspect') != {'path': 'image_inspect.json', 'sha256': image_sha, 'status': image['status']} or manifest.get('toolchain_capture') != {'path': 'toolchain_capture.json', 'sha256': tool_sha, 'status': tool['status']}:
        raise ExternalReviewError('capture manifest file binding drift')
    systems: dict[str, Any] = {}
    for system in SYSTEMS:
        expected = context['image_digests'][system]
        item = image['systems'][system]
        observed = item.get('observed')
        if item.get('status') != 'OBSERVED' or item.get('expected_digest') != expected or item.get('returncode') != 0 or item.get('timed_out') is not False or not isinstance(observed, Mapping):
            raise ExternalReviewError(f'{system} image inspection is incomplete')
        repos = observed.get('RepoDigests')
        if observed.get('Id') != expected or observed.get('Architecture') != 'amd64' or observed.get('Os') != 'linux' or not isinstance(repos, list) or not repos or repos != sorted(set(repos)) or any(not isinstance(value, str) or not value.endswith('@' + expected) for value in repos):
            raise ExternalReviewError(f'{system} image digest/platform/repository binding is invalid')
        titem = tool['systems'][system]
        if titem.get('status') != 'OBSERVED' or titem.get('probe_requested') is not True or titem.get('image_digest') != expected or titem.get('not_applicable_fields') != list(capture.NOT_APPLICABLE_FIELDS_BY_SYSTEM[system]):
            raise ExternalReviewError(f'{system} toolchain is incomplete')
        probes = titem.get('probes')
        if set(probes or {}) != set(capture.TOOLCHAIN_PROBES):
            raise ExternalReviewError(f'{system} toolchain probe coverage is incomplete')
        values: dict[str, Any] = {}
        for name in PROBES:
            probe = probes[name]
            if name in NOT_APPLICABLE[system]:
                if probe.get('value') != 'not_applicable' or probe.get('command') is not None or probe.get('returncode') != 0 or probe.get('timed_out') is not False or probe.get('reason') != 'not_applicable':
                    raise ExternalReviewError(f'{system} {name} N/A projection is invalid')
                values[name] = 'not_applicable'
            else:
                if not isinstance(probe, Mapping) or probe.get('returncode') != 0 or probe.get('timed_out') is not False or not isinstance(probe.get('value'), str) or not probe['value'] or probe.get('reason') is not None:
                    raise ExternalReviewError(f'{system} {name} probe is pending or failed')
                values[name] = probe['value']
        if titem.get('fingerprint') != _hash(values):
            raise ExternalReviewError(f'{system} toolchain fingerprint drift')
        systems[system] = {'image': {'digest': expected, 'architecture': 'amd64', 'os': 'linux', 'repo_digests': repos}, 'toolchain': {'fingerprint': titem['fingerprint'], 'not_applicable_fields': list(NOT_APPLICABLE[system]), 'values': values}}
    root_after = sealed._directory_identity(root, 'capture root')
    if dict(root_after) != dict(root_id):
        raise ExternalReviewError('capture root changed during validation')
    def file_ref(name: str, value: Mapping[str, Any], sha: str, field: str) -> dict[str, Any]:
        path = root / name
        data, identity = _read(path, f'capture {name}', limit=MAX_BYTES, immutable=True, mode=0o444)
        sidecar_path = path.with_name(path.name + '.sha256')
        sidecar_data, sidecar_identity = _read(
            sidecar_path, f'capture {name} sidecar', limit=256,
            immutable=True, mode=0o444)
        expected_sidecar = (sha + '  ' + path.name + '\n').encode('ascii')
        if sidecar_data != expected_sidecar:
            raise ExternalReviewError(f'capture {name} sidecar binding drift')
        return {
            'path': str(path), 'file_bytes': len(data), 'file_sha256': sha,
            'canonical_sha256': value[field], 'device': identity['device'],
            'inode': identity['inode'], 'mode': identity['mode'],
            'sidecar': {
                'path': str(sidecar_path), 'file_bytes': len(sidecar_data),
                'file_sha256': _hash(sidecar_data),
                'device': sidecar_identity['device'],
                'inode': sidecar_identity['inode'],
                'mode': sidecar_identity['mode'],
            },
        }
    return {'root': str(root), 'root_device': root_id['device'], 'root_inode': root_id['inode'], 'root_mode': root_id['mode'], 'manifest': file_ref('capture_manifest.json', manifest, manifest_sha, 'capture_identity_sha256'), 'image': file_ref('image_inspect.json', image, image_sha, 'artifact_identity_sha256'), 'toolchain': file_ref('toolchain_capture.json', tool, tool_sha, 'artifact_identity_sha256'), 'status': manifest['status'], 'review_status': manifest['review_status'], 'systems': systems}


def _policy(path: Path, candidate_binding: Mapping[str, Any], selection: Mapping[str, Any], producer: Mapping[str, Any], now: int) -> tuple[dict[str, Any], dict[str, Any]]:
    path = _absolute(str(path), 'trust policy')
    if path == FIXED_POLICY:
        raise ExternalReviewNotReady('checked-in trust policy is NOT_READY with zero keys')
    policy, data, identity = _pair(path, 'external trust policy', TRUST_SCHEMA, 'policy_identity_sha256')
    required = {'schema', 'schema_version', 'status', 'benchmark_eligible', 'candidate_binding', 'selection_binding', 'capture_producer', 'response_schema_binding', 'verifier', 'authorized_keys', 'nonce_policy', 'policy_identity_sha256'}
    if set(policy) != required or policy.get('schema') != TRUST_SCHEMA or policy.get('schema_version') != 1 or policy.get('status') != 'READY' or policy.get('benchmark_eligible') is not False or policy.get('candidate_binding') != dict(candidate_binding) or policy.get('selection_binding') != dict(selection) or policy.get('capture_producer') != dict(producer):
        raise ExternalReviewError('external trust policy is stale or not strict READY')
    schema_data, _ = _read(ROOT / RESPONSE_SCHEMA_REL, 'response schema')
    if policy.get('response_schema_binding') != {'path': RESPONSE_SCHEMA_REL, 'file_sha256': _hash(schema_data), 'schema': RESPONSE_SCHEMA}:
        raise ExternalReviewError('external response schema binding drift')
    expected_verifier = _verifier()
    if policy.get('verifier') != expected_verifier:
        raise ExternalReviewError('external verifier/backend binding drift')
    if policy.get('nonce_policy') != {'minimum_length': 32, 'maximum_length': 128, 'one_shot_ledger_required': True}:
        raise ExternalReviewError('external nonce policy drift')
    keys = policy.get('authorized_keys')
    if not isinstance(keys, list) or len(keys) != 1:
        raise ExternalReviewError('external policy must contain exactly one active key')
    key = keys[0]
    if set(key) != {'key_id', 'algorithm', 'status', 'public_key_base64', 'public_key_sha256', 'not_before', 'not_after'} or key['algorithm'] != ALGORITHM or key['status'] != 'ACTIVE':
        raise ExternalReviewError('external key descriptor is invalid')
    if type(key['not_before']) is not int or type(key['not_after']) is not int or now < key['not_before'] or now >= key['not_after']:
        raise ExternalReviewError('external key is outside validity')
    try:
        raw = base64.b64decode(key['public_key_base64'].encode('ascii'), validate=True)
    except (ValueError, UnicodeError) as error:
        raise ExternalReviewError('external public key is not strict base64') from error
    if len(raw) != 32 or _hash(raw) != key['public_key_sha256']:
        raise ExternalReviewError('external public key hash drift')
    return policy, {'path': str(path), 'file_bytes': len(data), 'file_sha256': _hash(data), 'canonical_sha256': policy['policy_identity_sha256'], 'schema': TRUST_SCHEMA, 'key': key, 'raw': raw}


def _verifier() -> dict[str, Any]:
    data, _ = _read(SCRIPT, 'external review verifier')
    import cryptography
    source_hash = _hash(data)
    return {'path': SCRIPT_REL, 'file_sha256': source_hash, 'implementation': 'competitive_execution_selection_r3_external_review_v1', 'backend': {'name': BACKEND_NAME, 'version': getattr(cryptography, '__version__', ''), 'implementation': BACKEND_IMPLEMENTATION, 'implementation_file': SCRIPT_REL, 'implementation_sha256': source_hash}}


def _review_scope(context: Mapping[str, Any]) -> dict[str, Any]:
    return {
        'required_systems': list(SYSTEMS),
        'release': 'Release',
        'thread_policy_sha256': context['thread_policy_sha256'],
        'image': {
            'platform': {'architecture': 'amd64', 'os': 'linux'},
            'exact_digest_binding': True, 'repo_digests_required': True,
        },
        'toolchain': {
            'probe_requested': True, 'network': 'none',
            'read_only_root': True, 'shell': False,
            'fields': list(PROBES),
            'not_applicable_fields': {
                system: list(NOT_APPLICABLE[system]) for system in SYSTEMS},
        },
        'candidate_status': 'NOT_READY',
        'offline_host_match': 'HOST_MATCH_NOT_CLAIMED',
    }


def build_response_template(*, capture_root: Path, policy_path: Path,
                            nonce: str, issued_at: int, expires_at: int,
                            decision_status: str, reviewer_note: str,
                            signature: Mapping[str, Any]) -> dict[str, Any]:
    """Build an unsigned envelope for a custodian; never creates a signature."""
    candidate, candidate_binding, context_info = _candidate()
    context = context_info['context']
    if not isinstance(nonce, str) or NONCE_RE.fullmatch(nonce) is None:
        raise ExternalReviewError('response nonce is invalid')
    if type(issued_at) is not int or type(expires_at) is not int or \
            issued_at < 0 or expires_at <= issued_at or \
            expires_at - issued_at > MAX_WINDOW:
        raise ExternalReviewError('response validity is invalid')
    if decision_status not in {'ACCEPTED', 'REJECTED', 'NEEDS_CLARIFICATION'}:
        raise ExternalReviewError('review decision is invalid')
    if not isinstance(reviewer_note, str) or not reviewer_note.strip():
        raise ExternalReviewError('reviewer note is invalid')
    policy_path = _absolute(str(policy_path), 'external trust policy')
    policy, policy_data, _policy_file_identity = _pair(
        policy_path, 'external trust policy', TRUST_SCHEMA,
        'policy_identity_sha256')
    capture_binding = _capture_binding(
        capture_root, candidate_binding, context)
    response = {
        'schema': RESPONSE_SCHEMA, 'schema_version': 1,
        'decision_status': decision_status, 'review_status': 'REVIEWED_EXTERNAL',
        'benchmark_eligible': False, 'claim_eligible': False,
        'active_profile_switch': False, 'campaign_id': CAMPAIGN_ID,
        'nonce': nonce, 'issued_at': issued_at, 'expires_at': expires_at,
        'candidate_binding': candidate_binding,
        'selection_binding': context_info['selection'],
        'capture_binding': capture_binding,
        'capture_producer': _capture_producer(),
        'review_scope': _review_scope(context),
        'review_decision': {
            'status': decision_status, 'image_review': decision_status,
            'toolchain_review': decision_status, 'reviewer_note': reviewer_note,
        },
        'trust_policy': {
            'path': str(policy_path), 'file_bytes': len(policy_data),
            'file_sha256': _hash(policy_data),
            'canonical_sha256': policy['policy_identity_sha256'],
            'schema': TRUST_SCHEMA,
        },
        'verifier': _verifier(), 'signature': dict(signature),
        'promotion_policy': {
            'benchmark_eligible': False, 'claim_eligible': False,
            'active_profile_switch': False,
            'requires_candidate_profile_reseal': True,
            'requires_external_custodian_review': True, 'offline_only': True,
        },
        'response_identity_sha256': '',
    }
    del candidate, _policy_file_identity
    response['response_identity_sha256'] = _hash({
        key: value for key, value in response.items()
        if key != 'response_identity_sha256'})
    return response


def _payload(response: Mapping[str, Any]) -> bytes:
    signature = response.get('signature')
    if not isinstance(signature, Mapping):
        raise ExternalReviewError('signature metadata is missing')
    metadata = {key: value for key, value in signature.items() if key not in {'signature_base64', 'payload_sha256'}}
    unsigned = {key: value for key, value in response.items() if key not in {'signature', 'response_identity_sha256'}}
    unsigned['signature_metadata'] = metadata
    return _canonical({'domain': DOMAIN, 'schema_version': 1, 'response': unsigned})


def _verify_signature(response: Mapping[str, Any], state: Mapping[str, Any]) -> None:
    signature = response['signature']; key = state['key']
    if set(signature) != {'algorithm', 'key_id', 'public_key_sha256', 'payload_sha256', 'signature_base64', 'backend'} or signature['algorithm'] != ALGORITHM or signature['key_id'] != key['key_id'] or signature['public_key_sha256'] != key['public_key_sha256'] or signature['backend'] != state['policy']['verifier']['backend']:
        raise ExternalReviewError('signature key/algorithm/backend drift')
    payload = _payload(response)
    if signature['payload_sha256'] != _hash(payload):
        raise ExternalReviewError('signature payload hash drift')
    try:
        raw_sig = base64.b64decode(signature['signature_base64'].encode('ascii'), validate=True)
        from cryptography.exceptions import InvalidSignature
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
        Ed25519PublicKey.from_public_bytes(state['raw']).verify(raw_sig, payload)
    except InvalidSignature as error:
        raise ExternalReviewError('Ed25519 signature is invalid') from error
    except (TypeError, ValueError, UnicodeError) as error:
        raise ExternalReviewError('Ed25519 signature cannot be verified') from error
    if len(raw_sig) != 64:
        raise ExternalReviewError('Ed25519 signature length is invalid')


def _claim(ledger: Path, nonce: str, response_identity: str) -> None:
    if NONCE_RE.fullmatch(nonce) is None:
        raise ExternalReviewError('response nonce is not path-safe')
    ledger = _absolute(str(ledger), 'replay ledger')
    info = sealed._directory_identity(ledger, 'replay ledger')
    path = ledger / (nonce + '.claim.json')
    if path.exists() or path.is_symlink():
        raise ExternalReviewError('response nonce has already been claimed')
    data = _canonical({'schema': 'competitive_execution_selection_r3_nonce_claim_v1', 'campaign_id': CAMPAIGN_ID, 'nonce': nonce, 'response_identity_sha256': response_identity})
    fd = None; owned = None; complete = False
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0), 0o444)
        opened = os.fstat(fd); owned = (opened.st_dev, opened.st_ino)
        offset = 0
        while offset < len(data):
            count = os.write(fd, data[offset:])
            if count <= 0: raise ExternalReviewError('nonce claim made no progress')
            offset += count
        os.fchmod(fd, 0o444); os.fsync(fd)
        final = os.fstat(fd)
        if (final.st_dev, final.st_ino) != owned or final.st_nlink != 1 or final.st_size != len(data):
            raise ExternalReviewError('nonce claim identity drift')
        os.close(fd); fd = None
        observed, _ = _read(path, 'nonce claim', limit=MAX_SIDE_BYTES, immutable=True, mode=0o444)
        if observed != data: raise ExternalReviewError('nonce claim bytes drift')
        after = sealed._directory_identity(ledger, 'replay ledger')
        if dict(after) != dict(info): raise ExternalReviewError('replay ledger changed')
        directory_fd = os.open(ledger, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0) | getattr(os, 'O_NOFOLLOW', 0))
        try: os.fsync(directory_fd)
        finally: os.close(directory_fd)
        complete = True
    except FileExistsError as error:
        raise ExternalReviewError('response nonce has already been claimed') from error
    except OSError as error:
        raise ExternalReviewError(f'cannot claim response nonce: {error}') from error
    finally:
        if fd is not None: os.close(fd)
        if owned is not None and not complete:
            try:
                current = path.lstat()
                if (current.st_dev, current.st_ino) == owned and current.st_nlink == 1: path.unlink()
            except FileNotFoundError: pass
            except OSError as error: raise ExternalReviewError(f'nonce cleanup failed: {error}') from error


def validate_response(response_path: Path, *, replay_ledger: Path, policy_path: Path | None = None, review_time: int | None = None) -> dict[str, Any]:
    now = int(time.time()) if review_time is None else review_time
    if type(now) is not int or now < 0: raise ExternalReviewError('review time is invalid')
    response_path = _absolute(str(response_path), 'response')
    response, data, identity = _pair(response_path, 'external review response', RESPONSE_SCHEMA, 'response_identity_sha256')
    required = {'schema', 'schema_version', 'decision_status', 'review_status', 'benchmark_eligible', 'claim_eligible', 'active_profile_switch', 'campaign_id', 'nonce', 'issued_at', 'expires_at', 'candidate_binding', 'selection_binding', 'capture_binding', 'capture_producer', 'review_scope', 'review_decision', 'trust_policy', 'verifier', 'signature', 'promotion_policy', 'response_identity_sha256'}
    if set(response) != required or response['decision_status'] not in {'ACCEPTED', 'REJECTED', 'NEEDS_CLARIFICATION'} or response['review_status'] != 'REVIEWED_EXTERNAL' or response['benchmark_eligible'] is not False or response['claim_eligible'] is not False or response['active_profile_switch'] is not False or response['campaign_id'] != CAMPAIGN_ID:
        raise ExternalReviewError('response shape/status is invalid')
    if type(response['issued_at']) is not int or type(response['expires_at']) is not int or response['issued_at'] < 0 or response['expires_at'] <= response['issued_at'] or response['expires_at'] - response['issued_at'] > MAX_WINDOW or now < response['issued_at'] or now >= response['expires_at']:
        raise ExternalReviewError('response is outside validity')
    if not isinstance(response['nonce'], str) or NONCE_RE.fullmatch(response['nonce']) is None:
        raise ExternalReviewError('response nonce is invalid')
    candidate, binding, context = _candidate()
    if response['candidate_binding'] != binding or response['selection_binding'] != context['selection']:
        raise ExternalReviewError('candidate/profile/campaign binding drift')
    expected_producer = _capture_producer()
    if response['capture_producer'] != expected_producer or response['verifier'] != _verifier():
        raise ExternalReviewError('producer/verifier source drift')
    expected_scope = {'required_systems': list(SYSTEMS), 'release': 'Release', 'thread_policy_sha256': context['context']['thread_policy_sha256'], 'image': {'platform': {'architecture': 'amd64', 'os': 'linux'}, 'exact_digest_binding': True, 'repo_digests_required': True}, 'toolchain': {'probe_requested': True, 'network': 'none', 'read_only_root': True, 'shell': False, 'fields': list(PROBES), 'not_applicable_fields': {s: list(NOT_APPLICABLE[s]) for s in SYSTEMS}}, 'candidate_status': 'NOT_READY', 'offline_host_match': 'HOST_MATCH_NOT_CLAIMED'}
    if response['review_scope'] != expected_scope:
        raise ExternalReviewError('review scope is not exact')
    decision = response['review_decision']
    if not isinstance(decision, Mapping) or set(decision) != {'status', 'image_review', 'toolchain_review', 'reviewer_note'} or decision['status'] != response['decision_status'] or decision['image_review'] != response['decision_status'] or decision['toolchain_review'] != response['decision_status'] or not isinstance(decision['reviewer_note'], str) or not decision['reviewer_note'].strip():
        raise ExternalReviewError('review decision is not exact')
    if response['promotion_policy'] != {'benchmark_eligible': False, 'claim_eligible': False, 'active_profile_switch': False, 'requires_candidate_profile_reseal': True, 'requires_external_custodian_review': True, 'offline_only': True}:
        raise ExternalReviewError('response promotion policy was altered')
    cap = _capture_binding(Path(response['capture_binding']['root']), binding, context['context'])
    if response['capture_binding'] != cap:
        raise ExternalReviewError('capture manifest/image/toolchain identity drift')
    if _absolute(response['trust_policy']['path'], 'trust policy') != _absolute(str(policy_path or FIXED_POLICY), 'trust policy'):
        raise ExternalReviewError('trust policy path drift')
    policy, state = _policy(_absolute(str(policy_path or FIXED_POLICY), 'trust policy'), binding, context['selection'], expected_producer, now)
    policy_ref = {'path': state['path'], 'file_bytes': state['file_bytes'], 'file_sha256': state['file_sha256'], 'canonical_sha256': state['canonical_sha256'], 'schema': TRUST_SCHEMA}
    if response['trust_policy'] != policy_ref:
        raise ExternalReviewError('trust policy descriptor drift')
    _verify_signature(response, {**state, 'policy': policy})
    _claim(Path(replay_ledger), response['nonce'], response['response_identity_sha256'])
    data2, identity2 = _read(response_path, 'response recheck', immutable=True, mode=0o444)
    if data2 != data or identity2 != identity:
        raise ExternalReviewError('response changed after verification')
    return {'status': response['decision_status'], 'structural_valid': True, 'signature_valid': True, 'review_time': now, 'response_identity_sha256': response['response_identity_sha256'], 'capture_identity_sha256': cap['manifest']['canonical_sha256'], 'benchmark_eligible': False, 'claim_eligible': False, 'active_profile_switch': False, 'promotion_allowed': False, 'offline_only': True, 'replay_protected': True}


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--response', type=Path, required=True)
    parser.add_argument('--replay-ledger', type=Path, required=True)
    parser.add_argument('--policy', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = validate_response(args.response, replay_ledger=args.replay_ledger, policy_path=args.policy)
    except (ExternalReviewError, OSError, ValueError, TypeError, json.JSONDecodeError) as error:
        print(json.dumps({'status': 'INVALID', 'error': str(error)}, indent=2))
        return 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(_main())
