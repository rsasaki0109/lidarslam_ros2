#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Validate one privacy-bounded first-map onboarding trial record."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'onboarding-trial-v1.schema.json'
)
SUPPLEMENT_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas'
    / 'onboarding-measurement-supplement-v1.schema.json'
)
VALIDATION_RECEIPT_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas'
    / 'first-map-validation-receipt-v1.schema.json'
)
MAX_VALIDATION_RECEIPT_BYTES = 1024 * 1024
MEASUREMENT_PATHS = (
    'input.download_bytes',
    'measurements.workflow_download_bytes',
    'measurements.wall_time_sec',
    'measurements.active_operator_time_sec',
    'measurements.command_count',
    'measurements.peak_disk_bytes',
    'measurements.output_bytes',
)
SUPPLEMENT_FIELDS = (
    ('input_download_bytes', ('input', 'download_bytes')),
    ('workflow_download_bytes', ('measurements', 'workflow_download_bytes')),
    ('wall_time_sec', ('measurements', 'wall_time_sec')),
    ('active_operator_time_sec', (
        'measurements', 'active_operator_time_sec')),
    ('command_count', ('measurements', 'command_count')),
    ('peak_disk_bytes', ('measurements', 'peak_disk_bytes')),
    ('output_bytes', ('measurements', 'output_bytes')),
)
SUPPLEMENT_SOURCE_BY_FIELD = {
    'input_download_bytes': 'observer-log',
    'workflow_download_bytes': 'observer-log',
    'wall_time_sec': 'observer-log',
    'active_operator_time_sec': 'operator-observation',
    'command_count': 'operator-observation',
    'peak_disk_bytes': 'dedicated-filesystem-sampler',
    'output_bytes': 'observer-log',
}
IMMUTABLE_REVISION_KINDS = {'git-commit', 'image-digest'}


class TrialError(ValueError):
    """The onboarding trial record is structurally or semantically invalid."""


def _load_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise TrialError(f'cannot read JSON object {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise TrialError(f'JSON root must be an object: {path}')
    return value


def _validate_schema(
    record: dict[str, Any],
    schema: dict[str, Any],
    *,
    label: str = 'onboarding trial',
) -> None:
    try:
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(
            schema,
            format_checker=jsonschema.FormatChecker(),
        ).validate(record)
    except (jsonschema.SchemaError, jsonschema.ValidationError) as exc:
        location = '.'.join(str(item) for item in exc.absolute_path)
        raise TrialError(
            f'{label} schema failed at '
            f'{location or "<root>"}: {exc.message}'
        ) from exc


def read_validation_receipt(path: Path) -> bytes:
    """Read one bounded, regular first-map receipt without following links."""
    try:
        if path.is_symlink() or not path.is_file():
            raise TrialError(
                f'validation receipt is not a regular file: {path}'
            )
        size = path.stat().st_size
        if size < 1 or size > MAX_VALIDATION_RECEIPT_BYTES:
            raise TrialError(
                'validation receipt must be between 1 and '
                f'{MAX_VALIDATION_RECEIPT_BYTES} bytes: {path}'
            )
        return path.read_bytes()
    except OSError as exc:
        raise TrialError(
            f'cannot read validation receipt {path}: {exc}'
        ) from exc


def _validate_validation_receipt(
    record: dict[str, Any],
    payload: bytes,
) -> None:
    """Bind a trial claim to the exact privacy-bounded product receipt."""
    if not payload or len(payload) > MAX_VALIDATION_RECEIPT_BYTES:
        raise TrialError(
            'validation receipt bytes must be between 1 and '
            f'{MAX_VALIDATION_RECEIPT_BYTES}'
        )
    try:
        receipt = json.loads(payload.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise TrialError('validation receipt is not readable JSON') from exc
    if not isinstance(receipt, dict):
        raise TrialError('validation receipt JSON root is not an object')
    _validate_schema(
        receipt,
        _load_object(VALIDATION_RECEIPT_SCHEMA),
        label='first-map validation receipt',
    )

    expected_sha256 = record['evidence']['receipt_sha256']
    actual_sha256 = hashlib.sha256(payload).hexdigest()
    if expected_sha256 is None or actual_sha256 != expected_sha256:
        raise TrialError(
            'validation receipt SHA-256 does not match the trial record'
        )
    receipt_passed = receipt['status'] == 'PASS'
    record_receipt_passed = record['outcome']['receipt_status'] == 'PASS'
    if receipt_passed != record_receipt_passed:
        raise TrialError(
            'validation receipt status does not match the trial record'
        )
    if (
        receipt['verification']['manifest_sha256']
        != record['evidence']['manifest_sha256']
    ):
        raise TrialError(
            'validation receipt manifest SHA-256 does not match the '
            'trial record'
        )

    environment = record['environment']
    receipt_run = receipt['run']
    if receipt_run['product_version'] != environment['product_version']:
        raise TrialError(
            'validation receipt product version does not match the '
            'trial record'
        )
    if receipt_run['profile_id'] != 'rko_lio_graph_mid360_preset':
        raise TrialError(
            'validation receipt does not identify the fixed first-map profile'
        )
    revision = environment['revision']
    expected_commit = None
    if revision['kind'] == 'git-commit':
        expected_commit = revision['value']
    else:
        candidate = record['evidence'].get('candidate_image_set')
        if candidate is not None:
            expected_commit = candidate['source_commit']
    if (
        expected_commit is not None
        and receipt_run['git_commit'] != expected_commit
    ):
        raise TrialError(
            'validation receipt source commit does not match the trial '
            'identity'
        )


def _path_value(record: dict[str, Any], dotted_path: str) -> Any:
    value: Any = record
    for item in dotted_path.split('.'):
        value = value[item]
    return value


def _nested_value(record: dict[str, Any], path: tuple[str, ...]) -> Any:
    value: Any = record
    for item in path:
        value = value[item]
    return value


def _set_nested_value(
    record: dict[str, Any],
    path: tuple[str, ...],
    value: Any,
) -> None:
    target: Any = record
    for item in path[:-1]:
        target = target[item]
    target[path[-1]] = value


def apply_measurement_supplement(
    record: dict[str, Any],
    supplement: dict[str, Any],
    *,
    record_bytes: bytes,
    schema: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Apply observed missing measurements without rewriting the base record.

    The supplement is bound to the exact bytes of the original record. A
    non-null supplement value may fill only a null base field; it can never
    overwrite an observed value or silently replace a trial identity.
    """
    contract = schema or _load_object(SUPPLEMENT_SCHEMA)
    _validate_schema(supplement, contract)
    if supplement['trial_id'] != record['trial_id']:
        raise TrialError(
            'measurement supplement trial_id does not match the base record')
    actual_sha256 = hashlib.sha256(record_bytes).hexdigest()
    if supplement['base_record_sha256'] != actual_sha256:
        raise TrialError(
            'measurement supplement base_record_sha256 does not match the '
            'exact base record bytes')
    try:
        raw_record = json.loads(record_bytes.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise TrialError('base record bytes are not valid JSON') from exc
    if raw_record != record:
        raise TrialError(
            'base record value does not match the bytes bound by the '
            'measurement supplement'
        )

    merged = copy.deepcopy(record)
    values = supplement['measurements']
    sources = supplement['measurement_sources']
    for field, path in SUPPLEMENT_FIELDS:
        value = values[field]
        source = sources[field]
        if value is None:
            if source != 'not-supplemented':
                raise TrialError(
                    f'{field} is null but measurement source is {source!r}')
            continue
        if source == 'not-supplemented':
            raise TrialError(
                f'{field} has a value but is marked not-supplemented')
        expected_source = SUPPLEMENT_SOURCE_BY_FIELD[field]
        if source != expected_source:
            raise TrialError(
                f'{field} must use measurement source '
                f'{expected_source!r}, found {source!r}'
            )
        if _nested_value(merged, path) is not None:
            raise TrialError(
                f'{field} supplement would overwrite an observed base value')
        _set_nested_value(merged, path, value)
    return merged


def _validate_measurements(record: dict[str, Any]) -> None:
    dataset_download = record['input']['download_bytes']
    measurements = record['measurements']
    workflow_download = measurements['workflow_download_bytes']
    wall_time = measurements['wall_time_sec']
    active_time = measurements['active_operator_time_sec']
    peak_disk = measurements['peak_disk_bytes']
    output_bytes = measurements['output_bytes']
    if wall_time is not None and wall_time <= 0:
        raise TrialError('wall_time_sec must be greater than zero when known')
    if (
        dataset_download is not None
        and workflow_download is not None
        and workflow_download < dataset_download
    ):
        raise TrialError(
            'workflow_download_bytes cannot be less than input.download_bytes'
        )
    if (
        wall_time is not None
        and active_time is not None
        and active_time > wall_time
    ):
        raise TrialError(
            'active_operator_time_sec cannot exceed wall_time_sec'
        )
    if (
        peak_disk is not None
        and output_bytes is not None
        and output_bytes > peak_disk
    ):
        raise TrialError('output_bytes cannot exceed peak_disk_bytes')


def _validate_evidence(record: dict[str, Any]) -> None:
    outcome = record['outcome']
    evidence = record['evidence']
    manifest_available = outcome['manifest_status'] != 'missing'
    receipt_available = outcome['receipt_status'] != 'NOT_CREATED'
    if manifest_available != (evidence['manifest_sha256'] is not None):
        raise TrialError(
            'manifest status and manifest_sha256 availability disagree')
    if receipt_available != (evidence['receipt_sha256'] is not None):
        raise TrialError(
            'receipt status and receipt_sha256 availability disagree')


def _validate_outcome(record: dict[str, Any]) -> None:
    outcome = record['outcome']
    if outcome['status'] == 'PASS':
        expected = {
            'runner_exit_code': 0,
            'manifest_status': 'succeeded',
            'diagnosis_status': 'success',
            'verifier_status': 'PASS',
            'receipt_status': 'PASS',
            'undocumented_manual_steps': 0,
            'failure_stage': 'none',
        }
        inconsistent = [
            key for key, value in expected.items()
            if outcome[key] != value
        ]
        if inconsistent:
            raise TrialError(
                'PASS trial has inconsistent outcome fields: '
                + ', '.join(inconsistent)
            )
    else:
        if outcome['failure_stage'] == 'none':
            raise TrialError('FAIL trial must identify a failure_stage')
        if not outcome['finding_codes']:
            raise TrialError(
                'FAIL trial must include at least one finding code'
            )


def evaluate_trial(
    record: dict[str, Any],
    schema: dict[str, Any] | None = None,
    *,
    validation_receipt_bytes: bytes | None = None,
    require_evidence_binding: bool = True,
) -> dict[str, Any]:
    """Validate one record and return a fail-closed comparability report.

    Trusted probe internals may disable the retained-receipt requirement while
    constructing a record. Public checkers and matrix gates keep it enabled.
    """
    contract = schema or _load_object(DEFAULT_SCHEMA)
    _validate_schema(record, contract)
    _validate_measurements(record)
    _validate_evidence(record)
    _validate_outcome(record)
    if validation_receipt_bytes is not None:
        _validate_validation_receipt(record, validation_receipt_bytes)

    missing = [
        path for path in MEASUREMENT_PATHS
        if _path_value(record, path) is None
    ]
    blockers: list[str] = []
    if missing:
        blockers.append('measurements_incomplete')
    if not record['environment']['clean_start']:
        blockers.append('environment_not_clean')
    if (
        record['environment']['revision']['kind']
        not in IMMUTABLE_REVISION_KINDS
    ):
        blockers.append('revision_not_immutable')
    if record['outcome']['status'] != 'PASS':
        blockers.append('outcome_failed')
    if record['outcome']['undocumented_manual_steps']:
        blockers.append('undocumented_manual_steps')
    if (
        require_evidence_binding
        and record['outcome']['status'] == 'PASS'
        and validation_receipt_bytes is None
    ):
        blockers.append('validation_receipt_unbound')

    return {
        'schema_version': 1,
        'trial_id': record['trial_id'],
        'outcome_status': record['outcome']['status'],
        'measurement_status': 'INCOMPLETE' if missing else 'COMPLETE',
        'comparable': not blockers,
        'missing_measurements': missing,
        'comparability_blockers': blockers,
    }


def render_markdown(report: dict[str, Any]) -> str:
    """Render a compact, reviewer-facing trial audit."""
    lines = [
        '# Onboarding trial audit',
        '',
        f"- Trial: `{report['trial_id']}`",
        f"- Outcome: **{report['outcome_status']}**",
        f"- Measurements: **{report['measurement_status']}**",
        (
            '- Comparable onboarding baseline: **YES**'
            if report['comparable']
            else '- Comparable onboarding baseline: **NO**'
        ),
        '',
        '## Missing measurements',
        '',
    ]
    if report['missing_measurements']:
        lines.extend(
            f'- `{item}`' for item in report['missing_measurements'])
    else:
        lines.append('- None.')
    lines.extend(['', '## Comparability blockers', ''])
    if report['comparability_blockers']:
        lines.extend(
            f'- `{item}`' for item in report['comparability_blockers'])
    else:
        lines.append('- None.')
    return '\n'.join(lines) + '\n'


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Validate a first-map onboarding trial and report whether it is '
            'complete enough for cross-path and cross-release comparison.'
        ),
    )
    parser.add_argument('record', type=Path)
    parser.add_argument('--schema', type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument(
        '--supplement',
        type=Path,
        help=(
            'Apply a SHA-bound onboarding measurement supplement before '
            'evaluating this record.'
        ),
    )
    parser.add_argument(
        '--validation-receipt',
        type=Path,
        help=(
            'Bind comparability to the exact privacy-bounded first-map '
            'validation receipt retained by the probe.'
        ),
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Print the comparability report as JSON.',
    )
    parser.add_argument(
        '--require-comparable',
        action='store_true',
        help='Exit 1 when the valid trial is not a comparable PASS baseline.',
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """CLI entrypoint; invalid records exit 2 and unmet gates exit 1."""
    args = _parse_args(argv)
    try:
        try:
            record_bytes = args.record.read_bytes()
            record_value = json.loads(record_bytes.decode('utf-8'))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise TrialError(
                f'cannot read JSON object {args.record}: {exc}'
            ) from exc
        if not isinstance(record_value, dict):
            raise TrialError('record JSON root must be an object')
        supplement_id = None
        if args.supplement is not None:
            supplement = _load_object(args.supplement)
            record_value = apply_measurement_supplement(
                record_value,
                supplement,
                record_bytes=record_bytes,
            )
            supplement_id = supplement['supplement_id']
        report = evaluate_trial(
            record_value,
            _load_object(args.schema),
            validation_receipt_bytes=(
                None if args.validation_receipt is None
                else read_validation_receipt(args.validation_receipt)
            ),
            require_evidence_binding=True,
        )
    except TrialError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        if supplement_id is not None:
            report['measurement_supplement_id'] = supplement_id
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        if supplement_id is not None:
            print(f'Measurement supplement: `{supplement_id}`')
        print(render_markdown(report), end='')
    if args.require_comparable and not report['comparable']:
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
