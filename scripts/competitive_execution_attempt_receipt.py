#!/usr/bin/env python3
"""Canonical, GT-blind per-attempt execution receipt contract.

The receipt is the identity boundary for one benchmark attempt.  It binds the
scheduled system/dataset/repetition to the frozen profile/selection/input,
container and command identity, exit/completion state, and output hashes.  A
receipt hash is computed over every field except its own hash, using canonical
JSON.  This module never opens a dataset, ground-truth file, or scorer.
"""

from __future__ import annotations

import hashlib
import json
import re
from typing import Any, Mapping


SCHEMA_VERSION = 1
RECEIPT_KIND = 'competitive_execution_attempt_receipt_v1'
HASH_KIND = 'canonical_execution_attempt_receipt_sha256_v1'
_SHA256_RE = re.compile(r'^[0-9a-f]{64}$')


class ExecutionReceiptError(ValueError):
    """A malformed or unbound execution receipt."""


def _canonical_payload(value: Mapping[str, Any]) -> dict[str, Any]:
    return {
        str(key): item for key, item in value.items()
        if key != 'execution_receipt_sha256'
    }


def canonical_bytes(value: Mapping[str, Any]) -> bytes:
    """Return canonical receipt bytes, excluding the self-hash field."""
    return (json.dumps(_canonical_payload(value), sort_keys=True,
                       separators=(',', ':'), ensure_ascii=True) + '\n').encode()


def receipt_sha256(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or _SHA256_RE.fullmatch(value) is None:
        raise ExecutionReceiptError(f'{label} must be lowercase 64-hex')
    return value


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ExecutionReceiptError(f'{label} must be a mapping')
    return value


def _reject_gt_tokens(value: Any, label: str, *, proof: bool = False) -> None:
    """Reject explicit GT/scorer paths or content markers in a receipt."""
    if isinstance(value, Mapping):
        for key, item in value.items():
            key_text = str(key).lower()
            if key_text == 'gt_blind_proof':
                _reject_gt_tokens(item, label, proof=True)
                continue
            if any(token in key_text for token in (
                    'ground_truth', 'ground-truth', 'gt_path', 'gt_root',
                    'gt_content', 'scorer_path', 'scorer_input')):
                # GT-blind proof fields are allowed only as boolean claims;
                # paths/content must never enter an execution receipt.
                if not proof and key_text not in {
                        'ground_truth_content_opened', 'scorer_invoked'}:
                    raise ExecutionReceiptError(
                        f'{label} contains forbidden GT/scorer field {key}')
            _reject_gt_tokens(item, label, proof=proof)
    elif isinstance(value, list):
        for item in value:
            _reject_gt_tokens(item, label, proof=proof)
    elif isinstance(value, str):
        lowered = value.lower()
        if any(token in lowered for token in (
                '/ground_truth', '/ground-truth', 'ground_truth/',
                'ground-truth/', 'gt_content', 'scorer_input')):
            raise ExecutionReceiptError(
                f'{label} contains forbidden GT/scorer path/content marker')


def validate_receipt(
        value: Mapping[str, Any], *, expected_system: str | None = None,
        expected_dataset: str | None = None,
        expected_run_index: int | None = None,
        expected_campaign_id: str | None = None,
        require_success: bool = False,
        expected_sha256: str | None = None) -> dict[str, Any]:
    """Validate one receipt and return its immutable identity projection."""
    _mapping(value, 'execution receipt')
    if value.get('schema_version') != SCHEMA_VERSION or \
            value.get('receipt_kind') != RECEIPT_KIND:
        raise ExecutionReceiptError('execution receipt schema identity is invalid')
    receipt_hash = _sha(value.get('execution_receipt_sha256'),
                        'execution_receipt_sha256')
    if receipt_sha256(value) != receipt_hash:
        raise ExecutionReceiptError('execution receipt self-hash mismatch')
    if expected_sha256 is not None and receipt_hash != _sha(
            expected_sha256, 'expected execution receipt SHA'):
        raise ExecutionReceiptError('execution receipt SHA differs from run identity')
    campaign_id = _sha(value.get('campaign_id'), 'campaign_id')
    if expected_campaign_id is not None and campaign_id != _sha(
            expected_campaign_id, 'expected campaign_id'):
        raise ExecutionReceiptError('execution receipt campaign identity mismatch')
    schedule = _mapping(value.get('schedule'), 'execution receipt schedule')
    system = schedule.get('system')
    dataset = schedule.get('sequence', schedule.get('dataset'))
    repetition = schedule.get('repetition')
    if not isinstance(system, str) or not system or not isinstance(dataset, str) or \
            not dataset or isinstance(repetition, bool) or \
            not isinstance(repetition, int) or repetition < 1:
        raise ExecutionReceiptError('execution receipt schedule identity is invalid')
    if expected_system is not None and system != expected_system:
        raise ExecutionReceiptError('execution receipt system mismatch')
    if expected_dataset is not None and dataset != expected_dataset:
        raise ExecutionReceiptError('execution receipt dataset mismatch')
    if expected_run_index is not None and repetition != expected_run_index:
        raise ExecutionReceiptError('execution receipt run_index mismatch')
    identity = _mapping(value.get('identity'), 'execution receipt identity')
    for field in (
            'profile_canonical_sha256', 'execution_receipt_file_sha256',
            'selection_receipt_file_sha256', 'image_digest'):
        if field not in identity:
            raise ExecutionReceiptError(f'execution identity.{field} is missing')
    _sha(identity['profile_canonical_sha256'], 'execution identity profile')
    _sha(identity['execution_receipt_file_sha256'],
         'execution identity selection receipt')
    _sha(identity['selection_receipt_file_sha256'],
         'execution identity holdout receipt')
    if not isinstance(identity['image_digest'], str) or not identity['image_digest']:
        raise ExecutionReceiptError('execution identity image digest is missing')
    if not isinstance(value.get('argv'), list) or not value['argv'] or \
            any(not isinstance(item, str) or not item for item in value['argv']):
        raise ExecutionReceiptError('execution receipt argv is invalid')
    if not isinstance(value.get('mounts'), list):
        raise ExecutionReceiptError('execution receipt mounts are missing')
    execution = _mapping(value.get('execution'), 'execution receipt execution')
    exit_status = execution.get('exit_status')
    if isinstance(exit_status, bool) or not isinstance(exit_status, int):
        raise ExecutionReceiptError('execution exit_status is invalid')
    if (require_success and
            (exit_status != 0 or execution.get('timed_out') is not False)):
        raise ExecutionReceiptError(
            'execution receipt does not prove a successful, non-timeout attempt')
    completion = _mapping(value.get('completion'), 'execution receipt completion')
    if not isinstance(completion.get('complete'), bool):
        raise ExecutionReceiptError('execution completion.complete is invalid')
    if require_success and completion.get('complete') is not True:
        raise ExecutionReceiptError('execution receipt is incomplete')
    artifacts = _mapping(value.get('artifact_hashes'),
                         'execution receipt artifact_hashes')
    if require_success and not artifacts:
        raise ExecutionReceiptError('execution receipt artifact_hashes is empty')
    for key, item in artifacts.items():
        if not isinstance(key, str) or not isinstance(item, str) or \
                _SHA256_RE.fullmatch(item) is None:
            raise ExecutionReceiptError('execution artifact hash projection is invalid')
    _sha(value.get('output_tree_sha256'), 'execution output tree SHA')
    proof = _mapping(value.get('gt_blind_proof'), 'execution GT-blind proof')
    if proof.get('ground_truth_content_opened') is not False or \
            proof.get('scorer_invoked') is not False:
        raise ExecutionReceiptError('execution receipt is not explicitly GT-blind')
    _reject_gt_tokens(value, 'execution receipt')
    return {
        'campaign_id': campaign_id,
        'system': system,
        'dataset': dataset,
        'run_index': repetition,
        'execution_receipt_sha256': receipt_hash,
        'receipt_hash_kind': HASH_KIND,
        'complete': completion['complete'],
    }


def seal_receipt(value: Mapping[str, Any]) -> dict[str, Any]:
    """Return a copy with schema identity and canonical self-hash sealed."""
    sealed = dict(value)
    sealed['schema_version'] = SCHEMA_VERSION
    sealed['receipt_kind'] = RECEIPT_KIND
    sealed['receipt_hash_kind'] = HASH_KIND
    sealed['execution_receipt_sha256'] = receipt_sha256(sealed)
    validate_receipt(sealed)
    return sealed


if __name__ == "__main__":
    raise SystemExit("competitive_execution_attempt_receipt is a library module; import it instead of running it directly.")
