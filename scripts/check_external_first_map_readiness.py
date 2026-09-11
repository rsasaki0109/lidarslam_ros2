#!/usr/bin/env python3
"""Validate and summarize the independent first-map adoption ledger."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LEDGER = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'external-first-map-validations.json'
)
DEFAULT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'external-first-map-validations-v1.schema.json'
)


class LedgerError(ValueError):
    """The tracked adoption ledger is invalid or internally inconsistent."""


def _load_object(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise LedgerError(f'cannot read JSON object {path}: {exc}') from exc
    if not isinstance(payload, dict):
        raise LedgerError(f'JSON root must be an object: {path}')
    return payload


def _require_unique(
    values: list[str],
    *,
    field: str,
    casefold: bool = False,
) -> None:
    normalized = [value.casefold() if casefold else value for value in values]
    duplicates = sorted({
        value for value in normalized if normalized.count(value) > 1
    })
    if duplicates:
        raise LedgerError(
            f'duplicate {field} values are not independent evidence: '
            + ', '.join(duplicates)
        )


def validate_ledger(
    ledger_path: Path = DEFAULT_LEDGER,
    schema_path: Path = DEFAULT_SCHEMA,
) -> dict[str, Any]:
    """Return a deterministic readiness report for one valid ledger."""
    ledger = _load_object(ledger_path)
    schema = _load_object(schema_path)
    return validate_ledger_payload(ledger, schema)


def validate_ledger_payload(
    ledger: dict[str, Any],
    schema: dict[str, Any],
) -> dict[str, Any]:
    """Return a readiness report for an in-memory ledger proposal."""
    try:
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(
            schema,
            format_checker=jsonschema.FormatChecker(),
        ).validate(ledger)
    except (jsonschema.SchemaError, jsonschema.ValidationError) as exc:
        location = (
            '.'.join(str(item) for item in exc.absolute_path) or '<root>'
        )
        raise LedgerError(
            f'schema validation failed at {location}: {exc.message}'
        ) from exc

    validations = ledger['validations']
    _require_unique(
        [item['id'] for item in validations],
        field='validation id',
    )
    _require_unique(
        [item['reporter'] for item in validations],
        field='reporter',
        casefold=True,
    )
    _require_unique(
        [item['issue_url'] for item in validations],
        field='issue URL',
    )
    _require_unique(
        [
            item['verification']['manifest_sha256']
            for item in validations
        ],
        field='run manifest SHA-256',
    )
    for item in validations:
        findings = item['findings']
        acceptance = item['acceptance']
        finding_status = acceptance['findings_status']
        resolution_urls = acceptance['resolution_urls']
        if findings:
            if (
                finding_status not in {'resolved', 'documented'}
                or not resolution_urls
            ):
                raise LedgerError(
                    f"validation {item['id']} has findings without a "
                    'public resolution or documentation link'
                )
        elif finding_status != 'no-findings' or resolution_urls:
            raise LedgerError(
                f"validation {item['id']} declares no findings but has an "
                'inconsistent findings disposition'
            )

    required = ledger['required_validations']
    accepted = len(validations)
    path_counts = {
        path: sum(
            item['documentation_path'] == path
            for item in validations
        )
        for path in (
            'docker-first-map',
            'source-quickstart',
            'own-bag',
        )
    }
    return {
        'schema_version': 1,
        'status': 'READY' if accepted >= required else 'NOT_READY',
        'required_validations': required,
        'accepted_validations': accepted,
        'remaining_validations': max(required - accepted, 0),
        'distinct_reporters': accepted,
        'documentation_path_counts': path_counts,
        'validation_ids': [item['id'] for item in validations],
    }


def render_markdown(report: dict[str, Any]) -> str:
    """Render a compact reviewer-facing readiness summary."""
    counts = report['documentation_path_counts']
    validation_ids = report['validation_ids']
    lines = [
        '# External first-map readiness',
        '',
        f"- Status: **{report['status']}**",
        (
            '- Accepted independent validations: '
            f"**{report['accepted_validations']} / "
            f"{report['required_validations']}**"
        ),
        f"- Remaining: **{report['remaining_validations']}**",
        f"- Distinct reporters: **{report['distinct_reporters']}**",
        '',
        '## Documentation paths',
        '',
        '| Path | Accepted reports |',
        '| --- | ---: |',
        f"| Docker First Map | {counts['docker-first-map']} |",
        f"| Source quickstart | {counts['source-quickstart']} |",
        f"| Own-bag golden path | {counts['own-bag']} |",
        '',
        '## Accepted validation IDs',
        '',
    ]
    if validation_ids:
        lines.extend(
            f'- `{validation_id}`' for validation_id in validation_ids
        )
    else:
        lines.append('- None yet.')
    return '\n'.join(lines) + '\n'


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Validate the tracked independent first-map ledger and report '
            'whether the v1.0 three-user gate is complete.'
        ),
    )
    parser.add_argument(
        '--ledger',
        type=Path,
        default=DEFAULT_LEDGER,
        help='Validation ledger JSON.',
    )
    parser.add_argument(
        '--schema',
        type=Path,
        default=DEFAULT_SCHEMA,
        help='Ledger JSON schema.',
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Print the readiness report as JSON instead of Markdown.',
    )
    parser.add_argument(
        '--output-json',
        type=Path,
        help='Also write the readiness report as JSON.',
    )
    parser.add_argument(
        '--output-markdown',
        type=Path,
        help='Also write the readiness report as Markdown.',
    )
    parser.add_argument(
        '--require-complete',
        action='store_true',
        help='Exit 1 unless three accepted independent validations exist.',
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """CLI entrypoint with stable validation/readiness exit behavior."""
    args = _parse_args(argv)
    try:
        report = validate_ledger(args.ledger, args.schema)
    except LedgerError as exc:
        print(f'external first-map ledger invalid: {exc}', file=sys.stderr)
        return 2

    json_text = json.dumps(report, indent=2, sort_keys=True) + '\n'
    markdown_text = render_markdown(report)
    if args.output_json:
        _write_text(args.output_json, json_text)
    if args.output_markdown:
        _write_text(args.output_markdown, markdown_text)
    print(json_text if args.json else markdown_text, end='')
    if args.require_complete and report['status'] != 'READY':
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
