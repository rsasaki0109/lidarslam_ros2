#!/usr/bin/env python3
"""Locally verify the pre-registered fresh-holdout authorization contract.

This module deliberately validates metadata only.  It never opens a bag,
ground-truth file, trajectory, or scorer artifact.  A claim can use the
result only after the pre-commit/holdout/replay/leakage/failure receipt chain
is hash-consistent, the replay and scoring roles are separated, and one
authorized scoring event (or an explicit invalidation) is recorded.  An
independent custodian attestation is intentionally not synthesized here.
"""

from __future__ import annotations

import base64
from datetime import datetime, timezone
import hashlib
import json
import re
from typing import Any, Mapping


SCHEMA_VERSION = 1
RECEIPT_HASH_KIND = 'canonical_authorization_receipt_sha256_v1'
_SHA256_RE = re.compile(r'^[0-9a-fA-F]{64}$')
_REVISION_RE = re.compile(r'^[0-9a-fA-F]{40}$')
_DEFAULT_CHAIN_KINDS = (
    'precommit', 'holdout_seal', 'replay_authorization',
    'leakage_audit', 'failure_record')
_DEFAULT_REQUIRED_PRECOMMIT_FIELDS = (
    'dataset_ids', 'holdout_slots', 'systems', 'run_count', 'scorer',
    'metric_gate_sha256', 'profile_sha256', 'hardware_fingerprint',
    'thread_policy_sha256', 'release')
_GT_IDENTITY_KEYS = {
    'identity_only', 'entries', 'slot_id', 'sequence',
    'ground_truth_sha256', 'ground_truth_size_bytes', 'sha256',
    'size_bytes', 'hash_kind',
}
_LEAKAGE_FALSE_FIELDS = (
    'gt_content_opened', 'gt_path_exposed_to_runner', 'prior_gt_access',
    'development_tuning', 'result_dependent_selection', 'reused_holdout',
    'gt_available_before_precommit')
_ATTESTATION_DOMAIN = (
    'lidarslam/competitive-fresh-holdout-authorization/ed25519/v1')
_ATTESTATION_ALGORITHM = 'Ed25519'
_BACKEND_NAME = 'python-cryptography'
_BACKEND_IMPLEMENTATION = (
    'cryptography.hazmat.primitives.asymmetric.ed25519.Ed25519PublicKey.verify')
_KEY_ID_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$')


def _canonical_json(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(',', ':'), ensure_ascii=True
    ).encode('utf-8')


def sha256_value(value: Any) -> str:
    """Hash a JSON-compatible value using the authorization encoding."""
    return hashlib.sha256(_canonical_json(value)).hexdigest()


def canonical_authorization_receipt_sha256(receipt: Mapping[str, Any]) -> str:
    """Return the hash covered by one chain receipt.

    Only the receipt's self field is excluded.  A signature, when an
    independently supplied receipt contains one, is therefore also bound by
    the hash; this function does not manufacture or verify that signature.
    """
    payload = {
        str(key): value for key, value in receipt.items()
        if key != 'receipt_sha256'
    }
    return sha256_value(payload)


def canonical_attestation_payload(auth: Mapping[str, Any]) -> bytes:
    """Return the domain-separated bytes covered by the custodian signature."""
    authorization = {
        str(key): value for key, value in auth.items()
        if key != 'external_attestation'
    }
    return _canonical_json({
        'domain': _ATTESTATION_DOMAIN,
        'schema_version': SCHEMA_VERSION,
        'authorization': authorization,
    })


def canonical_attestation_payload_sha256(auth: Mapping[str, Any]) -> str:
    """Return the SHA-256 identity of the detached-signature payload."""
    return hashlib.sha256(canonical_attestation_payload(auth)).hexdigest()


def canonical_attestation_binding_sha256(attestation: Mapping[str, Any]) -> str:
    """Hash signed-attestation metadata without circular signature fields."""
    payload = {
        str(key): value for key, value in attestation.items()
        if key not in {'signature', 'payload_sha256'}
    }
    return sha256_value(payload)


def canonical_trust_store_sha256(store: Mapping[str, Any]) -> str:
    """Hash a versioned trust store without its self-identity field."""
    payload = {
        str(key): value for key, value in store.items()
        if key != 'canonical_sha256'
    }
    return sha256_value(payload)


def _profile_document(profile: Mapping[str, Any] | None) -> Mapping[str, Any] | None:
    if not isinstance(profile, Mapping):
        return None
    document = profile.get('competitive_slam_profile', profile)
    return document if isinstance(document, Mapping) else None


def _sha(value: Any, label: str, errors: list[str]) -> bool:
    if not isinstance(value, str) or _SHA256_RE.fullmatch(value) is None:
        errors.append(f'{label} must be lowercase 64-hex SHA-256')
        return False
    if value != value.lower():
        errors.append(f'{label} must use lowercase hexadecimal')
        return False
    return True


def _revision(value: Any, label: str, errors: list[str]) -> bool:
    if not isinstance(value, str) or _REVISION_RE.fullmatch(value) is None:
        errors.append(f'{label} must be a pinned 40-hex revision')
        return False
    return True


def _positive_int(value: Any, label: str, errors: list[str]) -> bool:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        errors.append(f'{label} must be a positive integer')
        return False
    return True


def _slot_ground_truth_size(slot: Mapping[str, Any]) -> Any:
    value = slot.get('ground_truth_expected_bytes')
    if value is not None:
        return value
    frozen = slot.get('frozen_identity')
    if isinstance(frozen, Mapping):
        gt = frozen.get('ground_truth')
        if isinstance(gt, Mapping):
            return gt.get('bytes')
    return None


def _profile_fresh_slots(profile: Mapping[str, Any] | None) -> dict[str, Mapping[str, Any]]:
    document = _profile_document(profile)
    if document is None:
        return {}
    datasets = document.get('datasets')
    if not isinstance(datasets, Mapping):
        return {}
    slots = datasets.get('fresh_holdout_slots')
    if not isinstance(slots, Mapping):
        return {}
    return {
        str(key): value for key, value in slots.items()
        if isinstance(value, Mapping)
    }


def _path_like(value: Any) -> bool:
    if not isinstance(value, str):
        return False
    return (
        '/' in value or '\\' in value or '://' in value or
        value.startswith(('.', '~')))


def _validate_gt_manifest(gt_manifest: Any, errors: list[str]) -> bool:
    if not isinstance(gt_manifest, Mapping):
        errors.append('authorization.gt_manifest is missing or malformed')
        return False
    valid = True
    if gt_manifest.get('identity_only') is not True:
        errors.append('authorization.gt_manifest.identity_only must be true')
        valid = False
    for key in gt_manifest:
        if str(key) not in _GT_IDENTITY_KEYS:
            errors.append(
                'authorization.gt_manifest exposes a non-identity field: '
                f'{key}')
            valid = False
    entries = gt_manifest.get('entries')
    if not isinstance(entries, list) or not entries:
        errors.append('authorization.gt_manifest.entries must be a non-empty list')
        return False
    seen: set[str] = set()
    for index, entry in enumerate(entries):
        if not isinstance(entry, Mapping):
            errors.append(f'authorization.gt_manifest.entries[{index}] is malformed')
            valid = False
            continue
        if {str(key) for key in entry} - _GT_IDENTITY_KEYS:
            errors.append(
                f'authorization.gt_manifest.entries[{index}] contains a path/content field')
            valid = False
        slot_id = entry.get('slot_id')
        if not isinstance(slot_id, str) or not slot_id or slot_id in seen:
            errors.append(
                f'authorization.gt_manifest.entries[{index}].slot_id is missing or duplicated')
            valid = False
        else:
            seen.add(slot_id)
        if not _sha(entry.get('ground_truth_sha256', entry.get('sha256')),
                    f'authorization.gt_manifest.entries[{index}].ground_truth_sha256',
                    errors):
            valid = False
        if not _positive_int(
                entry.get('ground_truth_size_bytes', entry.get('size_bytes')),
                f'authorization.gt_manifest.entries[{index}].ground_truth_size_bytes',
                errors):
            valid = False
        for key, value in entry.items():
            if str(key) not in {'slot_id', 'sequence', 'ground_truth_sha256',
                                'ground_truth_size_bytes', 'sha256',
                                'size_bytes', 'hash_kind'}:
                valid = False
            if _path_like(value):
                errors.append(
                    f'authorization.gt_manifest.entries[{index}] contains a path-like value')
                valid = False
    return valid


def _validate_chain(
        auth: Mapping[str, Any], policy: Mapping[str, Any],
        errors: list[str], not_ready: list[str]) -> bool:
    chain = auth.get('receipt_chain')
    if not isinstance(chain, list) or not chain:
        not_ready.append('authorization.receipt_chain is missing')
        return False
    configured_kinds = policy.get(
        'required_chain_kinds', list(_DEFAULT_CHAIN_KINDS))
    if not isinstance(configured_kinds, list) or not all(
            isinstance(item, str) and item for item in configured_kinds):
        errors.append('fresh-holdout authorization required_chain_kinds is malformed')
        configured_kinds = list(_DEFAULT_CHAIN_KINDS)
    required_kinds = list(configured_kinds)
    observed_kinds: list[str] = []
    observed_hashes: set[str] = set()
    previous_time: datetime | None = None
    previous_hash: str | None = None
    valid = True
    payloads = {
        'precommit': auth.get('precommit'),
        'holdout_seal': auth.get('gt_manifest'),
        'replay_authorization': auth.get('replay'),
        'leakage_audit': auth.get('leakage_audit'),
        'failure_record': auth.get('failure_log'),
    }
    for index, raw in enumerate(chain):
        if not isinstance(raw, Mapping):
            errors.append(f'authorization.receipt_chain[{index}] is malformed')
            valid = False
            continue
        sequence = raw.get('sequence')
        if sequence != index + 1:
            errors.append('authorization receipt sequence is not contiguous')
            valid = False
        kind = raw.get('kind')
        if not isinstance(kind, str) or not kind:
            errors.append(f'authorization.receipt_chain[{index}].kind is missing')
            valid = False
        else:
            observed_kinds.append(kind)
            if kind in observed_kinds[:-1]:
                errors.append(f'duplicate authorization receipt kind: {kind}')
                valid = False
        timestamp = raw.get('timestamp_utc')
        parsed_time: datetime | None = None
        if not isinstance(timestamp, str):
            errors.append('authorization receipt timestamp_utc is missing')
            valid = False
        else:
            try:
                parsed_time = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
                if parsed_time.tzinfo is None:
                    raise ValueError('timestamp must include timezone')
                parsed_time = parsed_time.astimezone(timezone.utc)
            except ValueError:
                errors.append('authorization receipt timestamp_utc is invalid')
                valid = False
        if parsed_time is not None and previous_time is not None and parsed_time <= previous_time:
            errors.append('authorization receipt timestamps are not strictly increasing')
            valid = False
        if parsed_time is not None:
            previous_time = parsed_time
        previous_declared = raw.get('previous_sha256')
        if index == 0:
            if previous_declared is not None:
                errors.append('first authorization receipt previous_sha256 must be null')
                valid = False
        elif previous_declared != previous_hash:
            errors.append('authorization receipt hash chain predecessor mismatch')
            valid = False
        receipt_hash = raw.get('receipt_sha256')
        if not _sha(receipt_hash, f'authorization.receipt_chain[{index}].receipt_sha256', errors):
            valid = False
        else:
            if receipt_hash in observed_hashes:
                errors.append('authorization receipt hash is duplicated')
                valid = False
            observed_hashes.add(receipt_hash)
            if canonical_authorization_receipt_sha256(raw) != receipt_hash:
                errors.append('authorization receipt self SHA-256 mismatch')
                valid = False
        previous_hash = receipt_hash if isinstance(receipt_hash, str) else None
        payload_sha = raw.get('payload_sha256')
        if not _sha(payload_sha, f'authorization.receipt_chain[{index}].payload_sha256', errors):
            valid = False
        elif isinstance(kind, str) and kind in payloads and payloads[kind] is not None:
            if payload_sha != sha256_value(payloads[kind]):
                errors.append(f'authorization receipt payload mismatch: {kind}')
                valid = False
    expected_order = [kind for kind in required_kinds if kind in observed_kinds]
    if observed_kinds != expected_order:
        errors.append('authorization receipt kinds are reordered or undeclared')
        valid = False
    missing = [kind for kind in required_kinds if kind not in observed_kinds]
    if missing:
        not_ready.append('authorization receipt kinds are missing: ' + ', '.join(missing))
        valid = False
    head = auth.get('chain_head_sha256')
    if not _sha(head, 'authorization.chain_head_sha256', errors):
        valid = False
    elif previous_hash != head:
        errors.append('authorization.chain_head_sha256 does not match the receipt chain')
        valid = False
    return valid


def _validate_precommit(
        auth: Mapping[str, Any], policy: Mapping[str, Any],
        profile: Mapping[str, Any] | None, expected_profile_sha256: str | None,
        errors: list[str], not_ready: list[str]) -> bool:
    precommit = auth.get('precommit')
    if not isinstance(precommit, Mapping):
        not_ready.append('authorization.precommit is missing')
        return False
    required_fields = policy.get(
        'required_precommit_fields', list(_DEFAULT_REQUIRED_PRECOMMIT_FIELDS))
    if not isinstance(required_fields, list) or not all(
            isinstance(item, str) for item in required_fields):
        errors.append('fresh-holdout authorization required_precommit_fields is malformed')
        required_fields = list(_DEFAULT_REQUIRED_PRECOMMIT_FIELDS)
    valid = True
    for field in required_fields:
        if field not in precommit:
            not_ready.append(f'authorization.precommit.{field} is missing')
            valid = False
    profile_slots = _profile_fresh_slots(profile)
    slots = precommit.get('holdout_slots')
    if not isinstance(slots, Mapping) or not slots:
        not_ready.append('authorization.precommit.holdout_slots is missing')
        valid = False
        slots = {}
    dataset_ids = precommit.get('dataset_ids')
    if not isinstance(dataset_ids, list) or not dataset_ids or any(
            not isinstance(item, str) or not item for item in dataset_ids):
        errors.append('authorization.precommit.dataset_ids must be non-empty text list')
        valid = False
    elif len(set(dataset_ids)) != len(dataset_ids):
        errors.append('authorization.precommit.dataset_ids contains duplicates')
        valid = False
    if isinstance(dataset_ids, list) and isinstance(slots, Mapping):
        slot_dataset_ids = [
            item.get('dataset_id', item.get('dataset'))
            for item in slots.values() if isinstance(item, Mapping)]
        if slot_dataset_ids and sorted(str(item) for item in slot_dataset_ids) != sorted(
                str(item) for item in dataset_ids):
            errors.append('authorization precommit dataset IDs do not match slot identities')
            valid = False
    if profile_slots and {str(key) for key in slots} != set(profile_slots):
        errors.append('authorization precommit fresh slot IDs do not match profile')
        valid = False
    for slot_id, observed in slots.items():
        if not isinstance(observed, Mapping):
            errors.append(f'authorization.precommit.holdout_slots.{slot_id} is malformed')
            valid = False
            continue
        for key, value in observed.items():
            if ('ground_truth' in str(key).lower() and
                    any(token in str(key).lower() for token in ('path', 'url', 'uri', 'content'))):
                errors.append('authorization precommit exposes a ground-truth path/content field')
                valid = False
            if _path_like(value) and 'ground_truth' in str(key).lower():
                errors.append('authorization precommit exposes a path-like GT value')
                valid = False
        expected = profile_slots.get(str(slot_id))
        if expected is not None:
            comparisons = {
                'sequence': expected.get('sequence'),
                'dataset_id': expected.get('dataset'),
                'input_manifest_sha256': expected.get('input_manifest_sha256'),
                'ground_truth_sha256': expected.get('ground_truth_sha256'),
                'calibration_archive_sha256': expected.get('calibration_archive_sha256'),
                'ground_truth_size_bytes': _slot_ground_truth_size(expected),
            }
            for field, expected_value in comparisons.items():
                if expected_value is None:
                    continue
                if observed.get(field) != expected_value:
                    errors.append(f'authorization precommit slot drift: {slot_id}.{field}')
                    valid = False
        for field in ('input_manifest_sha256', 'ground_truth_sha256',
                      'calibration_archive_sha256'):
            if not _sha(observed.get(field),
                        f'authorization.precommit.holdout_slots.{slot_id}.{field}',
                        errors):
                valid = False
        if not _positive_int(
                observed.get('ground_truth_size_bytes'),
                f'authorization.precommit.holdout_slots.{slot_id}.ground_truth_size_bytes',
                errors):
            valid = False
    run_count = precommit.get('run_count')
    expected_runs = policy.get('run_count')
    if not _positive_int(run_count, 'authorization.precommit.run_count', errors):
        valid = False
    elif isinstance(expected_runs, int) and run_count != expected_runs:
        errors.append('authorization precommit run_count does not match profile')
        valid = False
    systems = precommit.get('systems')
    expected_systems = policy.get('required_systems', [])
    if not isinstance(systems, Mapping) or not systems:
        errors.append('authorization.precommit.systems is missing')
        valid = False
        systems = {}
    if (isinstance(expected_systems, list) and expected_systems and
            set(systems) != {str(item) for item in expected_systems}):
        errors.append('authorization precommit systems do not match profile')
        valid = False
    for system, identity in systems.items():
        if not isinstance(identity, Mapping):
            errors.append(f'authorization.precommit.systems.{system} is malformed')
            valid = False
            continue
        for field in ('revision', 'config_sha256', 'hardware_fingerprint',
                      'thread_policy_sha256'):
            value = identity.get(field)
            checker = _revision if field == 'revision' else _sha
            if not checker(value, f'authorization.precommit.systems.{system}.{field}', errors):
                valid = False
        if identity.get('release') != policy.get('release', 'Release'):
            errors.append(
                f'authorization.precommit.systems.{system}.release is not pinned Release')
            valid = False
    scorer = precommit.get('scorer')
    if not isinstance(scorer, Mapping):
        errors.append('authorization.precommit.scorer is missing')
        valid = False
    else:
        if not _revision(
                scorer.get('revision'), 'authorization.precommit.scorer.revision', errors):
            valid = False
        for field in ('config_sha256', 'fingerprint'):
            if not _sha(scorer.get(field), f'authorization.precommit.scorer.{field}', errors):
                valid = False
    for field in ('metric_gate_sha256', 'profile_sha256',
                  'hardware_fingerprint', 'thread_policy_sha256'):
        if not _sha(precommit.get(field), f'authorization.precommit.{field}', errors):
            valid = False
    declared_profile_sha = precommit.get('profile_sha256')
    if expected_profile_sha256 is not None and declared_profile_sha != expected_profile_sha256:
        errors.append('authorization precommit profile SHA-256 drift')
        valid = False
    if precommit.get('release') != policy.get('release', 'Release'):
        errors.append('authorization.precommit.release is not exactly Release')
        valid = False
    return valid


def _validate_roles_and_scoring(
        auth: Mapping[str, Any], manifest: Mapping[str, Any] | None,
        errors: list[str], not_ready: list[str]) -> bool:
    valid = True
    replay = auth.get('replay')
    scoring = auth.get('scoring')
    if not isinstance(replay, Mapping):
        errors.append('authorization.replay is missing')
        valid = False
        replay = {}
    if replay.get('role') != 'replay_only':
        errors.append('authorization.replay.role must be replay_only')
        valid = False
    for field in ('gt_mounts', 'gt_content_opened', 'scorer_invoked'):
        if replay.get(field) is not False:
            errors.append(f'authorization.replay.{field} must be false')
            valid = False
    replay_process = replay.get('process_id')
    if not isinstance(replay_process, str) or not replay_process:
        errors.append('authorization.replay.process_id is missing')
        valid = False
    if not isinstance(scoring, Mapping):
        errors.append('authorization.scoring is missing')
        valid = False
        scoring = {}
    if scoring.get('role') != 'scoring_only':
        errors.append('authorization.scoring.role must be scoring_only')
        valid = False
    scoring_process = scoring.get('process_id')
    if not isinstance(scoring_process, str) or not scoring_process:
        errors.append('authorization.scoring.process_id is missing')
        valid = False
    if replay_process and scoring_process and replay_process == scoring_process:
        errors.append('replay and scoring process identities must be distinct')
        valid = False
    events = scoring.get('events')
    event_count = scoring.get('authorized_event_count')
    if not isinstance(events, list) or not isinstance(event_count, int):
        errors.append('authorization.scoring events/count are malformed')
        valid = False
        events = []
        event_count = -1
    if scoring.get('overwrite_or_retry') is not False:
        errors.append('authorization.scoring.overwrite_or_retry must be false')
        valid = False
    status = scoring.get('status')
    if status in {'PASS', 'COMPLETE'}:
        if event_count != 1 or len(events) != 1:
            errors.append('exactly one authorized scoring event is required per sealed bundle')
            valid = False
        elif not isinstance(events[0], Mapping) or events[0].get('authorized') is not True:
            errors.append('the scoring event is not authorized')
            valid = False
        else:
            event_bundle_sha = events[0].get('sealed_bundle_sha256')
            if not _sha(
                    event_bundle_sha,
                    'authorization.scoring event sealed_bundle_sha256', errors):
                valid = False
            elif isinstance(manifest, Mapping) and isinstance(
                    manifest.get('manifest_sha256'), str) and event_bundle_sha != manifest.get(
                        'manifest_sha256'):
                errors.append('scoring event is bound to a different sealed bundle')
                valid = False
    elif status == 'INVALIDATED':
        invalidation = scoring.get('invalidation')
        if event_count != 0 or events:
            errors.append('invalidated scoring must not retain an authorized event')
            valid = False
        if not isinstance(invalidation, Mapping) or not isinstance(
                invalidation.get('reason'), str) or not invalidation.get('reason'):
            errors.append('explicit scoring invalidation reason is required')
            valid = False
        not_ready.append('sealed bundle was explicitly invalidated')
    else:
        not_ready.append('authorization.scoring has no completed authorized event')
    if isinstance(events, list):
        event_ids = [item.get('event_id') for item in events if isinstance(item, Mapping)]
        if len(event_ids) != len(set(event_ids)):
            errors.append('duplicate scoring event IDs are forbidden')
            valid = False
    return valid


def _validate_document_bindings(
        auth: Mapping[str, Any], document: Mapping[str, Any] | None,
        profile: Mapping[str, Any] | None, errors: list[str]) -> bool:
    """Bind the precommit identities to the submitted manifest/evidence."""
    if not isinstance(document, Mapping):
        return True
    precommit = auth.get('precommit')
    if not isinstance(precommit, Mapping):
        return False
    systems = precommit.get('systems')
    if not isinstance(systems, Mapping):
        return False
    valid = True
    revision_document = document.get('revision')
    if isinstance(revision_document, Mapping) and isinstance(
            revision_document.get('systems'), Mapping):
        observed = revision_document['systems']
        for system, identity in systems.items():
            if isinstance(identity, Mapping) and system in observed:
                if observed.get(system) != identity.get('revision'):
                    errors.append(
                        f'authorization precommit revision drift for system {system}')
                    valid = False
    evidence_systems = document.get('systems')
    if isinstance(evidence_systems, Mapping):
        for system, identity in systems.items():
            record = evidence_systems.get(system)
            provenance = (record.get('provenance') if isinstance(record, Mapping)
                          else None)
            if not isinstance(identity, Mapping) or not isinstance(provenance, Mapping):
                continue
            for field in ('revision', 'config_sha256', 'hardware_fingerprint'):
                if field in provenance and provenance.get(field) != identity.get(field):
                    errors.append(
                        f'authorization precommit {field} drift for system {system}')
                    valid = False
            if 'thread_policy' in provenance and isinstance(
                    provenance['thread_policy'], Mapping):
                if sha256_value(provenance['thread_policy']) != identity.get(
                        'thread_policy_sha256'):
                    errors.append(
                        f'authorization precommit thread policy drift for system {system}')
                    valid = False
    declared_scorer = document.get('scorer')
    precommit_scorer = precommit.get('scorer')
    if isinstance(declared_scorer, Mapping) and isinstance(precommit_scorer, Mapping):
        if (declared_scorer.get('fingerprint') is not None and
                declared_scorer.get('fingerprint') != precommit_scorer.get('fingerprint')):
            errors.append('authorization precommit scorer fingerprint drift')
            valid = False
    declared_profile = document.get('profile')
    ours_identity = systems.get('ours')
    if isinstance(declared_profile, Mapping) and isinstance(ours_identity, Mapping):
        if (declared_profile.get('sha256') is not None and
                declared_profile.get('sha256') != ours_identity.get('config_sha256')):
            errors.append('authorization precommit ours config SHA-256 drift')
            valid = False
    profile_document = _profile_document(profile)
    rivals = profile_document.get('rivals') if isinstance(profile_document, Mapping) else None
    if isinstance(rivals, Mapping):
        for system, identity in systems.items():
            rival = rivals.get(system)
            if isinstance(rival, Mapping) and isinstance(identity, Mapping):
                if rival.get('revision') != identity.get('revision'):
                    errors.append(
                        f'authorization precommit revision does not match profile for {system}')
                    valid = False
    return valid


def _validate_leakage(
        auth: Mapping[str, Any], errors: list[str], not_ready: list[str]) -> bool:
    leakage = auth.get('leakage_audit')
    if not isinstance(leakage, Mapping):
        not_ready.append('authorization.leakage_audit is missing')
        return False
    valid = True
    for field in _LEAKAGE_FALSE_FIELDS:
        if leakage.get(field) is not False:
            errors.append(f'authorization.leakage_audit.{field} must be false')
            valid = False
    if leakage.get('selection_before_gt_access') is not True:
        errors.append('authorization.leakage_audit.selection_before_gt_access must be true')
        valid = False
    if leakage.get('failure_records_complete') is not True:
        errors.append('authorization.leakage_audit.failure_records_complete must be true')
        valid = False
    failure_log = auth.get('failure_log')
    if not isinstance(failure_log, Mapping) or failure_log.get('complete') is not True:
        errors.append('authorization.failure_log.complete must be true')
        valid = False
    elif not isinstance(failure_log.get('events'), list):
        errors.append('authorization.failure_log.events must be a list')
        valid = False
    return valid


def _decode_base64(
        value: Any, label: str, expected_size: int, errors: list[str]) -> bytes | None:
    if not isinstance(value, str) or not value or len(value) > 4096:
        errors.append(f'{label} must be bounded base64 text')
        return None
    try:
        decoded = base64.b64decode(value.encode('ascii'), validate=True)
    except (UnicodeEncodeError, ValueError):
        errors.append(f'{label} is not canonical base64')
        return None
    if base64.b64encode(decoded).decode('ascii') != value:
        errors.append(f'{label} is not canonical base64')
        return None
    if len(decoded) != expected_size:
        errors.append(f'{label} must decode to exactly {expected_size} bytes')
        return None
    return decoded


def _parse_attestation_time(
        value: Any, label: str, errors: list[str]) -> datetime | None:
    if not isinstance(value, str) or not value.endswith('Z'):
        errors.append(f'{label} must be an RFC3339 UTC timestamp ending in Z')
        return None
    try:
        parsed = datetime.fromisoformat(value[:-1] + '+00:00')
    except ValueError:
        errors.append(f'{label} is not a valid RFC3339 UTC timestamp')
        return None
    if parsed.tzinfo is None:
        errors.append(f'{label} must include UTC timezone')
        return None
    return parsed.astimezone(timezone.utc)


def _validate_trust_store(
        attestation_policy: Mapping[str, Any], errors: list[str],
        not_ready: list[str], now_utc: datetime) -> dict[str, dict[str, Any]] | None:
    store = attestation_policy.get('trust_store')
    if store is None:
        not_ready.append(
            'external attestation trust store is not configured in the profile')
        return None
    if not isinstance(store, Mapping):
        errors.append('external attestation trust store is malformed')
        return None
    if {str(key) for key in store} - {
            'schema_version', 'canonical_sha256', 'keys'}:
        errors.append('external attestation trust store contains unknown fields')
    if store.get('schema_version') != 1:
        errors.append('external attestation trust store schema_version must be 1')
    declared_store_sha = store.get('canonical_sha256')
    store_sha_valid = _sha(
        declared_store_sha, 'external attestation trust store canonical_sha256', errors)
    if store_sha_valid and declared_store_sha != canonical_trust_store_sha256(store):
        errors.append('external attestation trust store canonical SHA-256 mismatch')
        store_sha_valid = False
    raw_keys = store.get('keys')
    if not isinstance(raw_keys, list) or not raw_keys:
        errors.append('external attestation trust store keys must be non-empty list')
        return None
    keys: dict[str, dict[str, Any]] = {}
    valid = store_sha_valid
    for index, raw_key in enumerate(raw_keys):
        label = f'external attestation trust store keys[{index}]'
        if not isinstance(raw_key, Mapping):
            errors.append(f'{label} is malformed')
            valid = False
            continue
        if {str(key) for key in raw_key} - {
                'key_id', 'algorithm', 'public_key_base64', 'public_key_sha256',
                'status', 'not_before_utc', 'not_after_utc'}:
            errors.append(f'{label} contains unknown fields')
            valid = False
        key_id = raw_key.get('key_id')
        if not isinstance(key_id, str) or _KEY_ID_RE.fullmatch(key_id) is None:
            errors.append(f'{label}.key_id is invalid')
            valid = False
            continue
        if key_id in keys:
            errors.append(f'duplicate external attestation trust-store key_id: {key_id}')
            valid = False
            continue
        if raw_key.get('algorithm') != _ATTESTATION_ALGORITHM:
            errors.append(f'{label}.algorithm must be {_ATTESTATION_ALGORITHM}')
            valid = False
        key_status = raw_key.get('status')
        if key_status not in {'ACTIVE', 'REVOKED'}:
            errors.append(f'{label}.status must be ACTIVE or REVOKED')
            valid = False
        key_before = _parse_attestation_time(
            raw_key.get('not_before_utc'), f'{label}.not_before_utc', errors)
        key_after = _parse_attestation_time(
            raw_key.get('not_after_utc'), f'{label}.not_after_utc', errors)
        if key_before is not None and key_after is not None and key_after <= key_before:
            errors.append(f'{label} validity interval is empty')
            valid = False
        public_key = _decode_base64(
            raw_key.get('public_key_base64'), f'{label}.public_key_base64', 32, errors)
        public_sha_valid = _sha(
            raw_key.get('public_key_sha256'), f'{label}.public_key_sha256', errors)
        if public_key is not None and public_sha_valid and hashlib.sha256(
                public_key).hexdigest() != raw_key.get('public_key_sha256'):
            errors.append(f'{label}.public_key_sha256 does not match key bytes')
            valid = False
        if public_key is None or key_before is None or key_after is None:
            valid = False
        keys[key_id] = {
            'algorithm': raw_key.get('algorithm'),
            'status': key_status,
            'not_before_utc': key_before,
            'not_after_utc': key_after,
            'public_key': public_key,
            'public_key_sha256': raw_key.get('public_key_sha256'),
        }
    if not valid:
        return None
    return keys


def _validate_external_attestation(
        auth: Mapping[str, Any], policy: Mapping[str, Any],
        errors: list[str], not_ready: list[str],
        now_utc: datetime | None = None) -> dict[str, Any]:
    attestation_policy = policy.get('external_attestation')
    if not isinstance(attestation_policy, Mapping):
        attestation_policy = {}
    required = attestation_policy.get('required', True) is True
    if not required:
        return {'status': 'NOT_REQUIRED', 'verified': False}
    attestation = auth.get('external_attestation')
    if not isinstance(attestation, Mapping):
        not_ready.append(
            'independent external custodian attestation is required: '
            'attestation mapping is missing')
        return {'status': 'NOT_READY', 'verified': False}
    if attestation.get('status') != 'PASS':
        if attestation.get('status') == 'NOT_READY':
            not_ready.append(
                'independent external custodian attestation is required: '
                'attestation is NOT_READY')
        else:
            errors.append('external attestation status must be PASS or NOT_READY')
        return {'status': 'NOT_READY', 'verified': False}
    backend_policy = attestation_policy.get('verification_backend')
    if not isinstance(backend_policy, Mapping):
        not_ready.append(
            'external attestation verification backend contract is missing from the profile')
        return {'status': 'NOT_READY', 'verified': False}
    backend_shape_valid = True
    if {str(key) for key in backend_policy} - {
            'name', 'version', 'implementation'}:
        errors.append('external attestation verification backend has unknown fields')
        backend_shape_valid = False
    if backend_policy.get('name') != _BACKEND_NAME:
        errors.append(
            f'external attestation verification backend name must be {_BACKEND_NAME}')
        backend_shape_valid = False
    backend_version = backend_policy.get('version')
    if not isinstance(backend_version, str) or not backend_version:
        errors.append('external attestation verification backend version is missing')
        backend_shape_valid = False
    if backend_policy.get('implementation') != _BACKEND_IMPLEMENTATION:
        errors.append('external attestation verification backend implementation is invalid')
        backend_shape_valid = False
    backend_identity = {
        'name': _BACKEND_NAME,
        'version': backend_version,
        'implementation': _BACKEND_IMPLEMENTATION,
    }
    if not backend_shape_valid:
        return {'status': 'FAIL_CLOSED', 'verified': False,
                'backend': backend_identity}
    attestation_shape_valid = True
    if {str(key) for key in attestation} - {
            'status', 'signer_id', 'algorithm', 'key_id', 'signature',
            'public_key_sha256', 'payload_sha256', 'issued_at_utc',
            'expires_at_utc'}:
        errors.append('external attestation contains unknown fields')
        attestation_shape_valid = False
    signer_id = attestation.get('signer_id')
    if not isinstance(signer_id, str) or not signer_id:
        errors.append('external attestation signer_id must be non-empty text')
        attestation_shape_valid = False
    if now_utc is None:
        now_utc = datetime.now(timezone.utc)
    if now_utc.tzinfo is None:
        errors.append('external attestation verification time must include timezone')
        return {'status': 'FAIL_CLOSED', 'verified': False}
    now_utc = now_utc.astimezone(timezone.utc)
    verification_time = now_utc.isoformat().replace('+00:00', 'Z')
    keys = _validate_trust_store(attestation_policy, errors, not_ready, now_utc)
    if keys is None:
        return {'status': 'NOT_READY' if not errors else 'FAIL_CLOSED', 'verified': False}
    valid = attestation_shape_valid
    if attestation.get('algorithm') != _ATTESTATION_ALGORITHM:
        errors.append(f'external attestation algorithm must be {_ATTESTATION_ALGORITHM}')
        valid = False
    key_id = attestation.get('key_id')
    if not isinstance(key_id, str) or _KEY_ID_RE.fullmatch(key_id) is None:
        errors.append('external attestation key_id is invalid')
        valid = False
        key = None
    else:
        key = keys.get(key_id)
        if key is None:
            errors.append('external attestation key_id is not in the trusted profile store')
            valid = False
    if key is not None:
        if key['status'] != 'ACTIVE':
            errors.append('external attestation key is revoked')
            valid = False
        if now_utc < key['not_before_utc'] or now_utc >= key['not_after_utc']:
            errors.append('external attestation trust anchor is expired or not yet valid')
            valid = False
        declared_key_sha = attestation.get('public_key_sha256')
        if not _sha(declared_key_sha, 'external attestation public_key_sha256', errors):
            valid = False
        elif declared_key_sha != key['public_key_sha256']:
            errors.append('external attestation public key SHA-256 does not match trust anchor')
            valid = False
    issued = _parse_attestation_time(
        attestation.get('issued_at_utc'), 'external attestation issued_at_utc', errors)
    expires = _parse_attestation_time(
        attestation.get('expires_at_utc'), 'external attestation expires_at_utc', errors)
    if issued is None or expires is None:
        valid = False
    else:
        if expires <= issued:
            errors.append('external attestation validity interval is empty')
            valid = False
        if key is not None and (
                issued < key['not_before_utc'] or expires > key['not_after_utc']):
            errors.append('external attestation interval exceeds trust-anchor validity')
            valid = False
        if now_utc < issued or now_utc >= expires:
            errors.append('external attestation is expired or not yet valid')
            valid = False
    payload_sha = attestation.get('payload_sha256')
    computed_payload_sha = canonical_attestation_payload_sha256(auth)
    binding_sha = auth.get('external_attestation_binding_sha256')
    if not _sha(
            binding_sha, 'authorization.external_attestation_binding_sha256', errors):
        valid = False
    elif binding_sha != canonical_attestation_binding_sha256(attestation):
        errors.append('external attestation metadata binding SHA-256 mismatch')
        valid = False
    if not _sha(payload_sha, 'external attestation payload_sha256', errors):
        valid = False
    elif payload_sha != computed_payload_sha:
        errors.append('external attestation payload SHA-256 mismatch')
        valid = False
    signature = _decode_base64(
        attestation.get('signature'), 'external attestation signature', 64, errors)
    if signature is None:
        valid = False
    if key is None or key.get('public_key') is None:
        valid = False
    if not valid:
        return {'status': 'FAIL_CLOSED', 'verified': False,
                'payload_sha256': computed_payload_sha,
                'backend': backend_identity,
                'verified_at_utc': verification_time}
    try:
        import cryptography
        from cryptography.exceptions import InvalidSignature
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
    except ImportError:
        not_ready.append(
            'Ed25519 verification dependency is unavailable; authorization is NOT_READY')
        return {'status': 'NOT_READY', 'verified': False,
                'payload_sha256': computed_payload_sha,
                'backend': {
                    **backend_identity,
                    'version': None,
                },
                'verified_at_utc': verification_time}
    installed_version = getattr(cryptography, '__version__', None)
    if installed_version != backend_version:
        errors.append(
            'external attestation verification backend version drift: '
            f'profile={backend_version} installed={installed_version}')
        return {'status': 'FAIL_CLOSED', 'verified': False,
                'backend': {
                    **backend_identity,
                    'installed_version': installed_version,
                },
                'verified_at_utc': verification_time}
    try:
        Ed25519PublicKey.from_public_bytes(key['public_key']).verify(
            signature, canonical_attestation_payload(auth))
    except InvalidSignature:
        errors.append('external attestation Ed25519 signature is invalid')
        return {'status': 'FAIL_CLOSED', 'verified': False,
                'payload_sha256': computed_payload_sha,
                'backend': backend_identity,
                'verified_at_utc': verification_time}
    except (TypeError, ValueError) as exc:
        errors.append(f'external attestation Ed25519 verification failed: {exc}')
        return {'status': 'FAIL_CLOSED', 'verified': False,
                'payload_sha256': computed_payload_sha,
                'backend': backend_identity,
                'verified_at_utc': verification_time}
    return {
        'status': 'PASS', 'verified': True, 'key_id': key_id,
        'algorithm': _ATTESTATION_ALGORITHM,
        'payload_sha256': computed_payload_sha,
        'trust_store_sha256': attestation_policy['trust_store'].get(
            'canonical_sha256'),
        'public_key_sha256': key['public_key_sha256'],
        'backend': backend_identity,
        'verified_at_utc': verification_time,
    }


def verify_fresh_holdout_authorization(
        document: Mapping[str, Any] | None,
        *,
        policy: Mapping[str, Any] | None = None,
        profile: Mapping[str, Any] | None = None,
        expected_profile_sha256: str | None = None,
        now_utc: datetime | None = None,
) -> dict[str, Any]:
    """Verify an authorization mapping without opening any data artifact.

    ``document`` may be a bundle manifest or a schema-v2 evidence receipt.
    A profile policy with ``required: false`` is explicitly report-only and
    returns ``NOT_REQUIRED``.  A required policy whose external attestation is
    absent returns ``NOT_READY`` with the exact outstanding requirement.
    """
    configured = dict(policy) if isinstance(policy, Mapping) else {}
    if configured.get('required') is not True:
        return {
            'schema_version': SCHEMA_VERSION,
            'status': 'NOT_REQUIRED',
            'pass': True,
            'claim_eligible': True,
            'required': False,
            'errors': [],
            'not_ready': [],
        }
    errors: list[str] = []
    not_ready: list[str] = []
    if not isinstance(document, Mapping):
        not_ready.append('authorization source document is missing')
        auth: Mapping[str, Any] = {}
    else:
        raw_auth = document.get('authorization')
        if not isinstance(raw_auth, Mapping):
            not_ready.append('authorization mapping is missing')
            auth = {}
        else:
            auth = raw_auth
    if auth:
        if auth.get('schema_version') != SCHEMA_VERSION:
            errors.append('authorization.schema_version must be 1')
        if auth.get('status') != 'READY':
            not_ready.append('authorization status is not READY')
        if auth.get('receipt_hash_kind', RECEIPT_HASH_KIND) != RECEIPT_HASH_KIND:
            errors.append('authorization receipt_hash_kind is invalid')
        _validate_precommit(auth, configured, profile, expected_profile_sha256,
                            errors, not_ready)
        _validate_document_bindings(auth, document, profile, errors)
        _validate_gt_manifest(auth.get('gt_manifest'), errors)
        _validate_chain(auth, configured, errors, not_ready)
        _validate_leakage(auth, errors, not_ready)
        _validate_roles_and_scoring(auth, document, errors, not_ready)
        attestation_result = _validate_external_attestation(
            auth, configured, errors, not_ready, now_utc)
    else:
        attestation_result = {'status': 'NOT_READY', 'verified': False}
    policy_status = configured.get('status')
    if policy_status != 'READY':
        not_ready.append(
            'profile fresh-holdout authorization policy is NOT_READY: '
            + str(configured.get('status_reason', 'external attestation is absent')))
    passed = not errors and not not_ready and bool(auth)
    status = 'PASS' if passed else ('NOT_READY' if not errors else 'FAIL_CLOSED')
    return {
        'schema_version': SCHEMA_VERSION,
        'status': status,
        'pass': passed,
        'claim_eligible': passed,
        'required': True,
        'errors': errors,
        'not_ready': not_ready,
        'receipt_chain_head_sha256': auth.get('chain_head_sha256'),
        'external_attestation_status': (
            auth.get('external_attestation', {}).get('status')
            if isinstance(auth.get('external_attestation'), Mapping) else None),
        'external_attestation_signature': attestation_result,
    }
