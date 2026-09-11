#!/usr/bin/env python3
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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
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

"""Audit the fixed Humble/Jazzy Docker/source onboarding trial matrix."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path, PurePosixPath
from typing import Any

from check_onboarding_trial import (
    TrialError,
    apply_measurement_supplement,
    evaluate_trial,
    read_validation_receipt,
)

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = (
    REPO_ROOT / 'docs' / 'schemas'
    / 'onboarding-trial-matrix-v1.schema.json'
)
EVIDENCE_INDEX_PATH = (
    REPO_ROOT / 'docs' / 'contracts'
    / 'g0-onboarding-matrix-evidence-v1.json'
)
EVIDENCE_INDEX_SCHEMA_PATH = (
    REPO_ROOT / 'docs' / 'schemas'
    / 'onboarding-matrix-evidence-index-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/onboarding-trial-matrix-v1.schema.json'
)
MATRIX_ID = 'g0-humble-jazzy-docker-source'
DATASET_ID = 'mid360-public-zenodo-14841855'
DATASET_BYTES = 517088133
EVIDENCE_INDEX_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/onboarding-matrix-evidence-index-v1.schema.json'
)
ROW_CONTRACTS = (
    {
        'row_id': 'docker-humble',
        'route': 'docker',
        'ros_distro': 'humble',
        'documentation_path': 'docker-first-map',
        'os_family': 'ubuntu-22.04',
        'revision_kind': 'image-digest',
    },
    {
        'row_id': 'docker-jazzy',
        'route': 'docker',
        'ros_distro': 'jazzy',
        'documentation_path': 'docker-first-map',
        'os_family': 'ubuntu-24.04',
        'revision_kind': 'image-digest',
    },
    {
        'row_id': 'source-humble',
        'route': 'source',
        'ros_distro': 'humble',
        'documentation_path': 'source-quickstart',
        'os_family': 'ubuntu-22.04',
        'revision_kind': 'git-commit',
    },
    {
        'row_id': 'source-jazzy',
        'route': 'source',
        'ros_distro': 'jazzy',
        'documentation_path': 'source-quickstart',
        'os_family': 'ubuntu-24.04',
        'revision_kind': 'git-commit',
    },
)
CONTRACT_BY_KEY = {
    (row['route'], row['ros_distro']): row for row in ROW_CONTRACTS
}
EXPECTED_ROW_IDS = tuple(row['row_id'] for row in ROW_CONTRACTS)


class MatrixError(ValueError):
    """The matrix cannot be evaluated without inventing evidence."""


class MatrixRecord(dict[str, Any]):
    """A trial record plus exact bounded evidence loaded by the index."""

    def __init__(
        self,
        record: dict[str, Any],
        *,
        validation_receipt_bytes: bytes | None = None,
    ) -> None:
        """Keep byte evidence out of the schema-validated record mapping."""
        super().__init__(record)
        self.validation_receipt_bytes = validation_receipt_bytes


def _load_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise MatrixError(f'cannot read trial record {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise MatrixError(f'trial record root is not an object: {path}')
    return value


def _route(record: dict[str, Any]) -> str:
    documentation_path = record.get('documentation_path')
    if documentation_path == 'docker-first-map':
        return 'docker'
    if documentation_path == 'source-quickstart':
        return 'source'
    raise MatrixError(
        'fixed matrix accepts only docker-first-map and source-quickstart '
        f'records, found {documentation_path!r}'
    )


def _index_file(
    relative: str,
    *,
    repo_root: Path,
    label: str,
) -> Path:
    path = PurePosixPath(relative)
    if (
        str(path) != relative
        or relative.startswith('/')
        or '..' in path.parts
        or '\\' in relative
    ):
        raise MatrixError(
            f'evidence index contains an unsafe {label} path: {relative!r}'
        )
    candidate = repo_root / relative
    if candidate.is_symlink() or not candidate.is_file():
        raise MatrixError(
            f'evidence index {label} is not a regular file: {relative}'
        )
    return candidate


def load_evidence_index(
    index_path: Path = EVIDENCE_INDEX_PATH,
    repo_root: Path = REPO_ROOT,
) -> list[dict[str, Any]]:
    """Load reviewed records named by the checked-in four-row index."""
    index = _load_object(index_path)
    schema = _load_object(EVIDENCE_INDEX_SCHEMA_PATH)
    try:
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(schema).validate(index)
    except (jsonschema.SchemaError, jsonschema.ValidationError) as exc:
        raise MatrixError(
            f'evidence index schema failed: {exc.message}') from exc
    if index['schema_uri'] != EVIDENCE_INDEX_SCHEMA_URI:
        raise MatrixError('evidence index uses an unsupported schema URI')
    row_ids = tuple(row['row_id'] for row in index['rows'])
    if row_ids != EXPECTED_ROW_IDS:
        raise MatrixError(
            'evidence index rows must use the fixed matrix order: '
            + ', '.join(EXPECTED_ROW_IDS))

    records = []
    for row in index['rows']:
        relative = row['record_path']
        if relative is None:
            if row.get('measurement_supplement_path') is not None:
                raise MatrixError(
                    f"evidence index row {row['row_id']} has a supplement "
                    'without a base record'
                )
            if row.get('validation_receipt_path') is not None:
                raise MatrixError(
                    f"evidence index row {row['row_id']} has a validation "
                    'receipt without a base record'
                )
            continue
        candidate = _index_file(
            relative,
            repo_root=repo_root,
            label='record',
        )
        record_bytes = candidate.read_bytes()
        record = _load_object(candidate)
        supplement_relative = row.get('measurement_supplement_path')
        if supplement_relative is not None:
            supplement_candidate = _index_file(
                supplement_relative,
                repo_root=repo_root,
                label='measurement supplement',
            )
            try:
                record = apply_measurement_supplement(
                    record,
                    _load_object(supplement_candidate),
                    record_bytes=record_bytes,
                )
            except TrialError as exc:
                raise MatrixError(
                    f'measurement supplement invalid for {row["row_id"]}: '
                    f'{exc}'
                ) from exc
        receipt_bytes = None
        receipt_relative = row.get('validation_receipt_path')
        if receipt_relative is not None:
            receipt_candidate = _index_file(
                receipt_relative,
                repo_root=repo_root,
                label='validation receipt',
            )
            try:
                receipt_bytes = read_validation_receipt(receipt_candidate)
            except TrialError as exc:
                raise MatrixError(
                    f'validation receipt invalid for {row["row_id"]}: '
                    f'{exc}'
                ) from exc
        route = _route(record)
        key = (route, record.get('environment', {}).get('ros_distro'))
        contract = CONTRACT_BY_KEY.get(key)
        if contract is None or contract['row_id'] != row['row_id']:
            raise MatrixError(
                f"evidence index row {row['row_id']} points to a different "
                'matrix row')
        records.append(MatrixRecord(
            record,
            validation_receipt_bytes=receipt_bytes,
        ))
    return records


def _validate_row_contract(
    record: dict[str, Any],
    contract: dict[str, str],
) -> None:
    trial_id = record['trial_id']
    environment = record['environment']
    revision = environment['revision']
    expected = {
        'documentation_path': contract['documentation_path'],
        'environment.ros_distro': contract['ros_distro'],
        'environment.architecture': 'x86_64',
        'environment.os_family': contract['os_family'],
        'environment.revision.kind': contract['revision_kind'],
        'input.dataset_class': 'fixed-public',
        'input.dataset_id': DATASET_ID,
    }
    actual = {
        'documentation_path': record['documentation_path'],
        'environment.ros_distro': environment['ros_distro'],
        'environment.architecture': environment['architecture'],
        'environment.os_family': environment['os_family'],
        'environment.revision.kind': revision['kind'],
        'input.dataset_class': record['input']['dataset_class'],
        'input.dataset_id': record['input']['dataset_id'],
    }
    mismatches = [
        f'{field}={actual[field]!r} (expected {value!r})'
        for field, value in expected.items()
        if actual[field] != value
    ]
    if mismatches:
        raise MatrixError(
            f'{trial_id} violates {contract["row_id"]}: '
            + '; '.join(mismatches)
        )
    if (
        record['outcome']['status'] == 'PASS'
        and record['input']['download_bytes'] != DATASET_BYTES
    ):
        raise MatrixError(
            f'{trial_id} PASS must use the full fixed dataset byte identity '
            f'{DATASET_BYTES}, found '
            f'{record["input"]["download_bytes"]!r}'
        )


def _matrix_row(
    contract: dict[str, str],
    record: dict[str, Any] | None,
    trial_report: dict[str, Any] | None,
) -> dict[str, Any]:
    if record is None or trial_report is None:
        return {
            'row_id': contract['row_id'],
            'route': contract['route'],
            'ros_distro': contract['ros_distro'],
            'present': False,
            'trial_id': None,
            'outcome_status': 'MISSING',
            'measurement_status': 'MISSING',
            'comparable': False,
            'comparability_blockers': ['record_missing'],
            'finding_codes': [],
        }
    return {
        'row_id': contract['row_id'],
        'route': contract['route'],
        'ros_distro': contract['ros_distro'],
        'present': True,
        'trial_id': record['trial_id'],
        'outcome_status': trial_report['outcome_status'],
        'measurement_status': trial_report['measurement_status'],
        'comparable': trial_report['comparable'],
        'comparability_blockers': trial_report['comparability_blockers'],
        'finding_codes': record['outcome']['finding_codes'],
    }


def _actions(
    rows: list[dict[str, Any]],
    activation_gate: bool,
    product_version_aligned: bool,
) -> list[str]:
    actions: list[str] = []
    missing = [row['row_id'] for row in rows if not row['present']]
    if missing:
        actions.append('Capture missing matrix rows: ' + ', '.join(missing))
    for row in rows:
        if not row['present'] or row['comparable']:
            continue
        details = list(row['comparability_blockers'])
        details.extend(row['finding_codes'])
        actions.append(
            f'Resolve {row["row_id"]}: ' + ', '.join(details)
        )
    if not product_version_aligned:
        actions.append(
            'Align all matrix rows to one product version before comparing '
            'Docker and source routes.'
        )
    if not missing and not activation_gate:
        actions.append(
            'Require at least one comparable Docker PASS and one comparable '
            'source PASS before the activation gate.'
        )
    return actions


def evaluate_matrix(records: list[dict[str, Any]]) -> dict[str, Any]:
    """Validate records and evaluate the two documented matrix gates."""
    selected: dict[tuple[str, str], tuple[dict[str, Any], dict[str, Any]]] = {}
    trial_ids: set[str] = set()
    product_versions: set[str] = set()
    source_revisions: set[str] = set()

    for record in records:
        try:
            trial_report = evaluate_trial(
                record,
                validation_receipt_bytes=getattr(
                    record,
                    'validation_receipt_bytes',
                    None,
                ),
                require_evidence_binding=True,
            )
        except TrialError as exc:
            label = record.get('trial_id', '<unknown>')
            raise MatrixError(f'invalid trial {label}: {exc}') from exc
        trial_id = record['trial_id']
        if trial_id in trial_ids:
            raise MatrixError(f'duplicate trial_id: {trial_id}')
        trial_ids.add(trial_id)
        route = _route(record)
        distro = record['environment']['ros_distro']
        key = (route, distro)
        contract = CONTRACT_BY_KEY.get(key)
        if contract is None:
            raise MatrixError(f'unexpected matrix row: {route}-{distro}')
        if key in selected:
            raise MatrixError(f'duplicate matrix row: {contract["row_id"]}')
        _validate_row_contract(record, contract)
        selected[key] = (record, trial_report)
        product_versions.add(record['environment']['product_version'])
        if route == 'source':
            source_revisions.add(
                record['environment']['revision']['value']
            )

    if len(source_revisions) > 1:
        raise MatrixError(
            'source rows disagree on Git commit: '
            + ', '.join(sorted(source_revisions))
        )

    rows = []
    for contract in ROW_CONTRACTS:
        item = selected.get((contract['route'], contract['ros_distro']))
        record, report = item if item is not None else (None, None)
        rows.append(_matrix_row(contract, record, report))

    present_rows = sum(row['present'] for row in rows)
    pass_rows = sum(row['outcome_status'] == 'PASS' for row in rows)
    comparable_rows = sum(row['comparable'] for row in rows)
    docker_comparable = sum(
        row['comparable'] and row['route'] == 'docker' for row in rows
    )
    source_comparable = sum(
        row['comparable'] and row['route'] == 'source' for row in rows
    )
    product_version_aligned = len(product_versions) <= 1
    matrix_complete = present_rows == len(ROW_CONTRACTS)
    activation_gate = (
        matrix_complete
        and docker_comparable >= 1
        and source_comparable >= 1
        and product_version_aligned
    )
    all_rows_comparable = (
        matrix_complete
        and comparable_rows == len(ROW_CONTRACTS)
        and product_version_aligned
    )
    if not matrix_complete:
        status = 'INCOMPLETE'
    elif all_rows_comparable:
        status = 'ALL_ROWS_COMPARABLE'
    elif activation_gate:
        status = 'ACTIVATION_GATE_PASS'
    else:
        status = 'BLOCKED'

    result = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'matrix_id': MATRIX_ID,
        'dataset_id': DATASET_ID,
        'product_versions': sorted(product_versions),
        'source_revisions': sorted(source_revisions),
        'rows': rows,
        'summary': {
            'required_rows': len(ROW_CONTRACTS),
            'present_rows': present_rows,
            'pass_rows': pass_rows,
            'comparable_rows': comparable_rows,
            'docker_comparable_rows': docker_comparable,
            'source_comparable_rows': source_comparable,
            'product_version_aligned': product_version_aligned,
            'matrix_complete': matrix_complete,
            'activation_gate': activation_gate,
            'all_rows_comparable': all_rows_comparable,
        },
        'decision': {
            'status': status,
            'actions': _actions(
                rows, activation_gate, product_version_aligned
            ),
        },
    }
    try:
        schema = _load_object(SCHEMA_PATH)
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(schema).validate(result)
    except (jsonschema.SchemaError, jsonschema.ValidationError) as exc:
        raise MatrixError(
            f'matrix report schema failed: {exc.message}'
        ) from exc
    return result


def render_markdown(report: dict[str, Any]) -> str:
    """Render a compact matrix gate summary."""
    lines = [
        '# Onboarding trial matrix audit',
        '',
        f"- Matrix status: **{report['decision']['status']}**",
        (
            '- Activation gate: **PASS**'
            if report['summary']['activation_gate']
            else '- Activation gate: **FAIL**'
        ),
        (
            '- All rows comparable: **YES**'
            if report['summary']['all_rows_comparable']
            else '- All rows comparable: **NO**'
        ),
        '',
        '| Row | Outcome | Measurements | Comparable |',
        '| --- | --- | --- | --- |',
    ]
    for row in report['rows']:
        lines.append(
            f"| `{row['row_id']}` | {row['outcome_status']} | "
            f"{row['measurement_status']} | "
            f"{'YES' if row['comparable'] else 'NO'} |"
        )
    lines.extend(['', '## Required actions', ''])
    actions = report['decision']['actions']
    lines.extend(f'- {action}' for action in actions)
    if not actions:
        lines.append('- None for this matrix gate.')
    return '\n'.join(lines) + '\n'


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('records', nargs='*', type=Path)
    parser.add_argument(
        '--evidence-index',
        type=Path,
        help=(
            'Use a reviewed evidence index instead of explicit records; '
            'the checked-in G0 index is used when neither is supplied.'
        ),
    )
    parser.add_argument(
        '--validation-receipt',
        action='append',
        type=Path,
        default=[],
        help=(
            'Exact first-map validation receipt for the corresponding '
            'explicit record; repeat once per record in the same order.'
        ),
    )
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--require-activation-gate', action='store_true')
    parser.add_argument('--require-all-comparable', action='store_true')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """CLI entrypoint; invalid evidence exits 2 and unmet gates exit 1."""
    args = _parse_args(argv)
    try:
        if args.records and args.evidence_index:
            raise MatrixError(
                'explicit records and --evidence-index are mutually exclusive')
        if args.validation_receipt and not args.records:
            raise MatrixError(
                '--validation-receipt requires explicit trial records'
            )
        if args.validation_receipt and (
            len(args.validation_receipt) != len(args.records)
        ):
            raise MatrixError(
                'repeat --validation-receipt exactly once per explicit '
                'trial record'
            )
        if args.records:
            receipts = (
                [read_validation_receipt(path)
                 for path in args.validation_receipt]
                if args.validation_receipt else [None] * len(args.records)
            )
            records = [
                MatrixRecord(
                    _load_object(path),
                    validation_receipt_bytes=receipt,
                )
                for path, receipt in zip(args.records, receipts)
            ]
        else:
            records = load_evidence_index(
                args.evidence_index or EVIDENCE_INDEX_PATH)
        report = evaluate_matrix(records)
    except (MatrixError, TrialError) as exc:
        print(f'onboarding matrix error: {exc}', file=sys.stderr)
        return 2
    rendered_json = json.dumps(report, indent=2, sort_keys=True) + '\n'
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(rendered_json, encoding='utf-8')
    if args.json:
        print(rendered_json, end='')
    else:
        print(render_markdown(report), end='')
    if args.require_all_comparable:
        return 0 if report['summary']['all_rows_comparable'] else 1
    if args.require_activation_gate:
        return 0 if report['summary']['activation_gate'] else 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
