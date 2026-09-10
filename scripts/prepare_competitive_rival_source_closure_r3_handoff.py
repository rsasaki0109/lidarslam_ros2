#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""Prepare or validate an unsigned r3 rival-source review handoff.

This is a request envelope only.  It contains no signer, key, license grant,
or reviewed external packet.  In particular, a legal capture packet produced
by the offline capture tool is not promoted to ``REVIEWED_EXTERNAL`` here.
The pair is sealed with the same no-follow, immutable 0444 contract as the
execution-selection handoff utility.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Mapping

_SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(_SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(_SOURCE_ROOT))

from scripts.prepare_competitive_execution_selection_r3_handoff import (  # noqa: E402
    _read_immutable,
    _require_output_path,
    _same_identity,
    _write_sealed_pair as _write_immutable_pair,
    CandidateError as ImmutableError,
    MAX_HANDOFF_BYTES,
    MAX_HANDOFF_SIDECAR_BYTES,
)
from scripts.validate_competitive_rival_source_closure_r3 import (  # noqa: E402
    CANDIDATE_REL,
    CandidateError,
    canonical_sha256,
    ROOT,
    SELECTION_REL,
    validate_checked_in,
)


HANDOFF_KIND = 'competitive_rival_source_closure_r3_handoff_v1'
HANDOFF_SIDECAR_SUFFIX = '.sha256'
HANDOFF_FIELDS = frozenset({
    'schema_version', 'handoff_kind', 'status', 'benchmark_eligible',
    'claim_eligible', 'candidate_binding', 'selection_binding',
    'r2_lineage', 'legal_review', 'promotion_policy',
    'handoff_identity_sha256',
})
LEGAL_BLOCKERS = [
    'glim_ros2_license_text_missing',
    'fast_livo2_license_declaration_conflict',
    'rpg_vikit_license_artifacts_incomplete',
    'sophus_license_artifact_missing',
    'external_custodian_legal_review_required',
]


def _canonical_without_identity(value: Mapping[str, Any]) -> str:
    return canonical_sha256({k: v for k, v in value.items()
                             if k != 'handoff_identity_sha256'})


def _json(path: Path, label: str) -> Mapping[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CandidateError(f'{label} is unreadable') from exc
    if not isinstance(value, Mapping):
        raise CandidateError(f'{label} must be an object')
    return value


def _checked_documents(
        root: Path = ROOT) -> tuple[Mapping[str, Any], Mapping[str, Any], dict[str, Any]]:
    report = validate_checked_in(root)
    candidate = _json(root / CANDIDATE_REL, 'r3 candidate')
    selection = _json(root / SELECTION_REL, 'r3 selection')
    return candidate, selection, report


def _document(
        candidate: Mapping[str, Any], selection: Mapping[str, Any],
        root: Path = ROOT) -> dict[str, Any]:
    candidate_bytes = (root / CANDIDATE_REL).read_bytes()
    selection_bytes = (root / SELECTION_REL).read_bytes()
    lineage = candidate.get('r2_lineage')
    if not isinstance(lineage, Mapping):
        raise CandidateError('candidate r2 lineage is missing')
    result: dict[str, Any] = {
        'schema_version': 1,
        'handoff_kind': HANDOFF_KIND,
        'status': 'UNSIGNED_REVIEW_REQUIRED',
        'benchmark_eligible': False,
        'claim_eligible': False,
        'candidate_binding': {
            'path': CANDIDATE_REL,
            'file_sha256': hashlib.sha256(candidate_bytes).hexdigest(),
            'candidate_identity_sha256': candidate.get('candidate_identity_sha256'),
            'candidate_id': candidate.get('candidate_id'),
        },
        'selection_binding': {
            'path': SELECTION_REL,
            'file_sha256': hashlib.sha256(selection_bytes).hexdigest(),
            'selection_identity_sha256': selection.get('selection_identity_sha256'),
            'selection_id': selection.get('selection_id'),
        },
        'r2_lineage': dict(lineage),
        'legal_review': {
            'status': 'NOT_PROVIDED',
            'required_blockers': LEGAL_BLOCKERS,
            'external_packet': None,
            'auto_review': False,
        },
        'promotion_policy': {
            'active_profile_switch': False,
            'r2_immutable': True,
            'requires_reviewed_external_packet': True,
            'requires_candidate_and_profile_reseal': True,
            'signer_or_private_key_included': False,
            'benchmark_eligible': False,
        },
        'handoff_identity_sha256': '',
    }
    result['handoff_identity_sha256'] = _canonical_without_identity(result)
    return result


def _write_pair(output: Path, document: Mapping[str, Any]) -> tuple[Path, Path]:
    output = _require_output_path(output, 'r3 rival-source handoff')
    try:
        return _write_immutable_pair(output, document)
    except ImmutableError as exc:
        raise CandidateError(str(exc)) from exc


def prepare_handoff(*, output: Path, root: Path = ROOT) -> dict[str, Any]:
    candidate, selection, report = _checked_documents(root)
    del report
    document = _document(candidate, selection, root)
    _write_pair(output, document)
    return validate_handoff(output, root=root)


def validate_handoff(path: Path, *, root: Path = ROOT) -> dict[str, Any]:
    path = _require_output_path(path, 'r3 rival-source handoff')
    try:
        payload, identity = _read_immutable(
            path, 'r3 rival-source handoff', max_bytes=MAX_HANDOFF_BYTES)
        sidecar_path = path.with_name(path.name + HANDOFF_SIDECAR_SUFFIX)
        sidecar, sidecar_identity = _read_immutable(
            sidecar_path, 'r3 rival-source handoff sidecar',
            max_bytes=MAX_HANDOFF_SIDECAR_BYTES)
    except ImmutableError as exc:
        raise CandidateError(str(exc)) from exc
    expected_sidecar = (f'{hashlib.sha256(payload).hexdigest()}  {path.name}\n').encode()
    if sidecar != expected_sidecar:
        raise CandidateError('r3 rival-source handoff sidecar is stale')
    document = _json_bytes(payload, 'r3 rival-source handoff')
    if set(document) != HANDOFF_FIELDS:
        raise CandidateError('r3 rival-source handoff field set is not exact')
    if document.get('schema_version') != 1 or document.get(
            'handoff_kind') != HANDOFF_KIND:
        raise CandidateError('r3 rival-source handoff schema identity is invalid')
    if document.get('status') != 'UNSIGNED_REVIEW_REQUIRED' or document.get(
            'benchmark_eligible') is not False or document.get(
            'claim_eligible') is not False:
        raise CandidateError('r3 rival-source handoff cannot claim promotion')
    if _canonical_without_identity(document) != document.get('handoff_identity_sha256'):
        raise CandidateError('r3 rival-source handoff identity is stale')
    candidate, selection, _ = _checked_documents(root)
    expected = _document(candidate, selection, root)
    # The review request is a canonical projection of the checked-in inputs;
    # no caller-supplied path or legal packet can be substituted.
    if dict(document) != expected:
        raise CandidateError('r3 rival-source handoff is stale or cross-campaign')
    legal = document.get('legal_review')
    if not isinstance(legal, Mapping) or legal.get('status') != 'NOT_PROVIDED' or \
            legal.get('external_packet') is not None or legal.get('auto_review') is not False or \
            legal.get('required_blockers') != LEGAL_BLOCKERS:
        raise CandidateError('r3 legal review packet was incorrectly auto-promoted')
    final_payload, final_identity = _read_immutable(
        path, 'r3 rival-source handoff', max_bytes=MAX_HANDOFF_BYTES)
    final_sidecar, final_sidecar_identity = _read_immutable(
        sidecar_path, 'r3 rival-source handoff sidecar',
        max_bytes=MAX_HANDOFF_SIDECAR_BYTES)
    if not _same_identity(identity, final_identity) or not _same_identity(
            sidecar_identity, final_sidecar_identity) or final_payload != payload or \
            final_sidecar != sidecar:
        raise CandidateError('r3 rival-source handoff changed during validation')
    return {
        'status': document['status'],
        'structural_valid': True,
        'benchmark_eligible': False,
        'claim_eligible': False,
        'handoff_identity_sha256': document['handoff_identity_sha256'],
        'legal_review_status': 'NOT_PROVIDED',
    }


def _json_bytes(payload: bytes, label: str) -> Mapping[str, Any]:
    try:
        value = json.loads(payload.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise CandidateError(f'{label} is not valid JSON') from exc
    if not isinstance(value, Mapping):
        raise CandidateError(f'{label} must be an object')
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    prepare = sub.add_parser('prepare')
    prepare.add_argument('--output', type=Path, required=True)
    prepare.add_argument('--root', type=Path, default=ROOT)
    validate = sub.add_parser('validate')
    validate.add_argument('--handoff', type=Path, required=True)
    validate.add_argument('--root', type=Path, default=ROOT)
    args = parser.parse_args()
    try:
        result = (prepare_handoff(output=args.output, root=args.root)
                  if args.command == 'prepare'
                  else validate_handoff(args.handoff, root=args.root))
    except (CandidateError, ImmutableError, OSError, ValueError, TypeError) as exc:
        print(json.dumps({'status': 'INVALID', 'error': str(exc)}, indent=2))
        return 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
