#!/usr/bin/env python3
"""Prepare, but never directly apply, an external first-map ledger entry."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import re
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

from check_external_first_map_readiness import (
    DEFAULT_LEDGER,
    DEFAULT_SCHEMA as DEFAULT_LEDGER_SCHEMA,
    LedgerError,
    validate_ledger_payload,
)

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RECEIPT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'first-map-validation-receipt-v1.schema.json'
)
DEFAULT_REPORT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'external-first-map-acceptance-v1.schema.json'
)
REPORT_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/'
    'external-first-map-acceptance-v1.schema.json'
)
REQUIRED_RECEIPT_CHECKS = {
    'manifest_succeeded',
    'lifecycle_complete',
    'runner_exit_zero',
    'diagnosis_success',
    'autoware_verification_pass',
    'diagnosis_bound_to_manifest',
    'verify_log_bound_to_manifest',
}
KNOWN_MAINTAINERS = {'@rsasaki0109'}
REVIEW_COMMENT_PATTERN = re.compile(
    r'^https://github\.com/rsasaki0109/lidar_slam_ros2/issues/'
    r'[1-9][0-9]*#issuecomment-[1-9][0-9]*$'
)
REVIEW_PULL_PATTERN = re.compile(
    r'^https://github\.com/rsasaki0109/lidar_slam_ros2/pull/'
    r'[1-9][0-9]*(?:#(?:issuecomment|discussion_r)-[1-9][0-9]*)?$'
)


class AcceptanceError(ValueError):
    """A proposed external validation cannot be accepted safely."""


def _load_object(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise AcceptanceError(
            f'cannot read JSON object {path}: {exc}'
        ) from exc
    if not isinstance(payload, dict):
        raise AcceptanceError(f'JSON root must be an object: {path}')
    return payload


def _validate_schema(
    payload: dict[str, Any],
    schema: dict[str, Any],
    *,
    label: str,
) -> None:
    try:
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(
            schema,
            format_checker=jsonschema.FormatChecker(),
        ).validate(payload)
    except (jsonschema.SchemaError, jsonschema.ValidationError) as exc:
        location = (
            '.'.join(str(item) for item in exc.absolute_path) or '<root>'
        )
        raise AcceptanceError(
            f'{label} schema validation failed at {location}: {exc.message}'
        ) from exc


def _canonical_sha256(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(',', ':'),
    ).encode()
    return hashlib.sha256(encoded).hexdigest()


def _parse_datetime(value: str) -> datetime:
    return datetime.fromisoformat(value.replace('Z', '+00:00'))


def _validate_receipt(receipt: dict[str, Any]) -> None:
    if receipt['status'] != 'PASS':
        raise AcceptanceError('receipt status must be PASS')
    checks = {item['id']: item for item in receipt['checks']}
    missing = sorted(REQUIRED_RECEIPT_CHECKS - checks.keys())
    if missing:
        raise AcceptanceError(
            'receipt is missing required checks: ' + ', '.join(missing)
        )
    failed = sorted(
        item['id'] for item in receipt['checks'] if not item['passed']
    )
    if failed:
        raise AcceptanceError(
            'receipt has failed required checks: ' + ', '.join(failed)
        )


def _validate_cross_file_contract(
    entry: dict[str, Any],
    receipt: dict[str, Any],
    maintainers: set[str],
) -> None:
    if entry['verification'] != receipt['verification']:
        raise AcceptanceError(
            'entry verification must exactly match receipt verification'
        )
    if receipt['evidence']['manifest']['sha256'] != (
        receipt['verification']['manifest_sha256']
    ):
        raise AcceptanceError(
            'receipt manifest evidence hash does not match verification'
        )
    product_version = receipt['run']['product_version']
    git_commit = receipt['run']['git_commit']
    release_ref = entry['release_ref']
    version_refs = {product_version, f'v{product_version}'}
    is_digest = re.search(r'(?:^|@)sha256:[0-9a-f]{64}$', release_ref)
    if (
        release_ref not in version_refs
        and release_ref != git_commit
        and is_digest is None
    ):
        raise AcceptanceError(
            'release_ref must match the receipt product version, git commit, '
            'or an immutable sha256 image digest'
        )

    if entry['reporter'].casefold() in {
        maintainer.casefold() for maintainer in maintainers
    }:
        raise AcceptanceError('a known maintainer cannot count as independent')
    if entry['acceptance']['reviewed_by'].casefold() not in {
        maintainer.casefold() for maintainer in maintainers
    }:
        raise AcceptanceError('reviewed_by must identify a known maintainer')
    if _parse_datetime(entry['acceptance']['reviewed_at']) < (
        _parse_datetime(entry['submitted_at'])
    ):
        raise AcceptanceError('reviewed_at cannot precede submitted_at')

    review_url = entry['acceptance']['review_url']
    is_issue_review = REVIEW_COMMENT_PATTERN.fullmatch(review_url)
    if is_issue_review and not review_url.startswith(
        f"{entry['issue_url']}#issuecomment-"
    ):
        raise AcceptanceError(
            'issue review_url must be a comment on the submitted issue'
        )
    if not (is_issue_review or REVIEW_PULL_PATTERN.fullmatch(review_url)):
        raise AcceptanceError(
            'review_url must identify a public repository issue comment or PR'
        )


def prepare_acceptance(
    ledger: dict[str, Any],
    entry: dict[str, Any],
    receipt: dict[str, Any],
    ledger_schema: dict[str, Any],
    receipt_schema: dict[str, Any],
    report_schema: dict[str, Any],
    maintainers: set[str] = KNOWN_MAINTAINERS,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Validate all evidence and return a report plus proposed ledger."""
    try:
        current = validate_ledger_payload(ledger, ledger_schema)
    except LedgerError as exc:
        raise AcceptanceError(f'current ledger is invalid: {exc}') from exc
    proposed_ledger = copy.deepcopy(ledger)
    proposed_ledger['validations'].append(copy.deepcopy(entry))
    try:
        proposed = validate_ledger_payload(proposed_ledger, ledger_schema)
    except LedgerError as exc:
        raise AcceptanceError(f'proposed ledger is invalid: {exc}') from exc

    _validate_schema(receipt, receipt_schema, label='receipt')
    _validate_receipt(receipt)
    _validate_cross_file_contract(entry, receipt, maintainers)

    report = {
        'schema_version': 1,
        'schema_uri': REPORT_SCHEMA_URI,
        'status': 'READY_TO_PROPOSE',
        'validation_id': entry['id'],
        'current_accepted_validations': current['accepted_validations'],
        'proposed_accepted_validations': proposed['accepted_validations'],
        'remaining_validations': proposed['remaining_validations'],
        'ledger_sha256': _canonical_sha256(ledger),
        'proposed_ledger_sha256': _canonical_sha256(proposed_ledger),
        'receipt_manifest_sha256': receipt['verification']['manifest_sha256'],
        'proposal_written_to': None,
    }
    _validate_schema(report, report_schema, label='acceptance report')
    return report, proposed_ledger


def _write_new_json(path: Path, payload: dict[str, Any]) -> None:
    text = json.dumps(payload, indent=2, sort_keys=True) + '\n'
    try:
        with path.open('x', encoding='utf-8') as output:
            output.write(text)
    except OSError as exc:
        raise AcceptanceError(
            f'cannot create proposal without overwriting {path}: {exc}'
        ) from exc


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Validate one reviewed first-map entry against its receipt and '
            'prepare a non-overwriting ledger proposal.'
        ),
    )
    parser.add_argument('--entry', type=Path, required=True)
    parser.add_argument('--receipt', type=Path, required=True)
    parser.add_argument('--ledger', type=Path, default=DEFAULT_LEDGER)
    parser.add_argument(
        '--ledger-schema',
        type=Path,
        default=DEFAULT_LEDGER_SCHEMA,
    )
    parser.add_argument(
        '--receipt-schema',
        type=Path,
        default=DEFAULT_RECEIPT_SCHEMA,
    )
    parser.add_argument(
        '--report-schema',
        type=Path,
        default=DEFAULT_REPORT_SCHEMA,
    )
    parser.add_argument(
        '--output-ledger',
        type=Path,
        help=(
            'Create a proposed ledger at a new path. The tracked ledger and '
            'existing files are never overwritten.'
        ),
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """CLI entrypoint; invalid evidence and unsafe output both exit 2."""
    args = _parse_args(argv)
    try:
        ledger = _load_object(args.ledger)
        entry = _load_object(args.entry)
        receipt = _load_object(args.receipt)
        ledger_schema = _load_object(args.ledger_schema)
        receipt_schema = _load_object(args.receipt_schema)
        report_schema = _load_object(args.report_schema)
        report, proposal = prepare_acceptance(
            ledger,
            entry,
            receipt,
            ledger_schema,
            receipt_schema,
            report_schema,
        )
        if args.output_ledger:
            if args.output_ledger.resolve() == args.ledger.resolve():
                raise AcceptanceError(
                    'output-ledger must not be the authoritative input ledger'
                )
            _write_new_json(args.output_ledger, proposal)
            report['proposal_written_to'] = str(args.output_ledger)
            _validate_schema(
                report,
                report_schema,
                label='acceptance report',
            )
    except AcceptanceError as exc:
        print(f'external first-map acceptance blocked: {exc}', file=sys.stderr)
        return 2

    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
