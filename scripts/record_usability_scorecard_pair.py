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

"""Record one prepared lidar_slam_ros2 / GLIM usability scorecard pair.

The recorder accepts only untouched fail-closed worksheets, derives command
counts and task outcomes, validates both completed records, and publishes the
pair with one atomic directory rename. Missing observations remain explicit;
they never become comparable evidence. No network or remote mutation occurs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shlex
import shutil
import sys
import tempfile
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from check_usability_scorecard import (  # noqa: E402
    PAIR_FIELDS,
    PRODUCT_IDS,
    ScorecardError,
    TASK_CONTRACTS,
    evaluate_scorecard,
    validate_trial,
)

from prepare_usability_scorecard_pair import (  # noqa: E402
    PREPARATION_RECEIPT_NAME,
    validate_preparation_receipt,
)


MEASUREMENT_FIELDS = (
    'wall_time_sec',
    'active_operator_time_sec',
    'workflow_download_bytes',
    'peak_disk_bytes',
    'failure_count',
    'output_bytes',
)
INTEGER_MEASUREMENTS = {
    'workflow_download_bytes',
    'peak_disk_bytes',
    'failure_count',
    'output_bytes',
}
SLUG = re.compile(r'^[a-z0-9][a-z0-9._-]{2,79}$')
OBSERVATION_KEYS = {
    'task_id',
    'exact_commands',
    'measurements',
    'checks',
    'undocumented_manual_steps',
    'finding_codes',
    'transcript_sha256',
    'public_url',
}
PREPARATION_ARCHIVE_DIR = 'preparation'


def _load_object_payload(
    path: Path,
    label: str,
) -> tuple[dict[str, Any], bytes]:
    try:
        payload = path.read_bytes()
        value = json.loads(payload)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ScorecardError(f'cannot read {label} {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise ScorecardError(f'{label} root must be an object: {path}')
    return value, payload


def _load_object(path: Path, label: str) -> dict[str, Any]:
    return _load_object_payload(path, label)[0]


def _ensure_untouched_worksheet(record: Mapping[str, Any]) -> None:
    """Reject stale or partially edited input before recording a new run."""
    for task in record['tasks']:
        task_id = task['task_id']
        untouched = (
            task['exact_commands'] == []
            and all(
                value is None for value in task['measurements'].values()
            )
            and all(not item['passed'] for item in task['checks'])
            and task['outcome'] == {
                'status': 'FAIL',
                'undocumented_manual_steps': 0,
                'finding_codes': ['not-recorded'],
            }
            and task['evidence'] == {
                'transcript_sha256': None,
                'public_url': None,
            }
        )
        if not untouched:
            raise ScorecardError(
                f'{record["product"]["id"]} {task_id} is not an untouched '
                'prepared worksheet')


def _load_pair(
    paths: Sequence[Path],
) -> tuple[dict[str, dict[str, Any]], bytes, dict[str, bytes]]:
    if len(paths) != len(PRODUCT_IDS):
        raise ScorecardError('provide exactly two --record worksheets')
    if any(path.is_symlink() for path in paths):
        raise ScorecardError('prepared worksheets must not be symbolic links')
    loaded = [_load_object_payload(path, 'worksheet') for path in paths]
    records = [record for record, _ in loaded]
    worksheet_payloads = [payload for _, payload in loaded]
    for record in records:
        validate_trial(record)
        _ensure_untouched_worksheet(record)
    selected = {record['product']['id']: record for record in records}
    if set(selected) != set(PRODUCT_IDS) or len(selected) != len(records):
        raise ScorecardError(
            'worksheets must contain one lidar_slam_ros2 and one GLIM record')

    left = selected[PRODUCT_IDS[0]]
    right = selected[PRODUCT_IDS[1]]
    if left['trial_id'] == right['trial_id']:
        raise ScorecardError('paired trial IDs must be different')
    for field in PAIR_FIELDS:
        if left['environment'][field] != right['environment'][field]:
            raise ScorecardError(f'paired environment {field} differs')
    for field in ('class', 'cohort_id', 'first_attempt'):
        if left['operator'][field] != right['operator'][field]:
            raise ScorecardError(f'paired operator {field} differs')
    if {
        left['operator']['product_order'],
        right['operator']['product_order'],
    } != {'first', 'second'}:
        raise ScorecardError(
            'paired product order must contain first and second')
    for left_task, right_task in zip(left['tasks'], right['tasks']):
        if left_task['input_id'] != right_task['input_id']:
            raise ScorecardError(
                f"paired input differs for {left_task['task_id']}")

    try:
        parents = {path.parent.resolve(strict=True) for path in paths}
    except OSError as exc:
        raise ScorecardError(
            f'cannot resolve prepared worksheet directory: {exc}'
        ) from exc
    if len(parents) != 1:
        raise ScorecardError(
            'prepared worksheets must share one receipt directory'
        )
    parent = next(iter(parents))
    receipt_path = parent / PREPARATION_RECEIPT_NAME
    if receipt_path.is_symlink():
        raise ScorecardError('preparation receipt must not be a symbolic link')
    receipt, receipt_payload = _load_object_payload(
        receipt_path,
        'preparation receipt',
    )
    ordered_records = [selected[product_id] for product_id in PRODUCT_IDS]
    validate_preparation_receipt(receipt, ordered_records)
    entries = {item['filename']: item for item in receipt['files']}
    if set(entries) != {path.name for path in paths}:
        raise ScorecardError(
            'preparation receipt does not name the selected worksheets'
        )
    for path, payload in zip(paths, worksheet_payloads):
        actual_sha256 = hashlib.sha256(payload).hexdigest()
        if actual_sha256 != entries[path.name]['sha256']:
            raise ScorecardError(
                f'prepared worksheet bytes differ from receipt: {path.name}'
            )
    prepared_worksheets = {
        path.name: payload for path, payload in zip(paths, worksheet_payloads)
    }
    return selected, receipt_payload, prepared_worksheets


def _validate_metric(name: str, value: Any, task_id: str) -> None:
    if value is None:
        return
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScorecardError(f'{task_id} {name} must be a number or null')
    if not math.isfinite(value) or value < 0:
        raise ScorecardError(
            f'{task_id} {name} must be finite and non-negative')
    if name in INTEGER_MEASUREMENTS and not isinstance(value, int):
        raise ScorecardError(f'{task_id} {name} must be an integer or null')


def _validate_task_observation(
    value: Any,
    contract: Mapping[str, Any],
) -> dict[str, Any]:
    task_id = contract['task_id']
    if not isinstance(value, dict):
        raise ScorecardError(f'{task_id} observation must be an object')
    unknown = set(value) - OBSERVATION_KEYS
    missing = OBSERVATION_KEYS - set(value)
    if unknown or missing:
        raise ScorecardError(
            f'{task_id} observation keys differ; missing={sorted(missing)}, '
            f'unknown={sorted(unknown)}')
    if value['task_id'] != task_id:
        raise ScorecardError(f'expected observation for {task_id}')

    commands = value['exact_commands']
    if not isinstance(commands, list) or not all(
        isinstance(command, str) and command for command in commands
    ):
        raise ScorecardError(
            f'{task_id} exact_commands must be a list of non-empty strings')
    if len(commands) > 20:
        raise ScorecardError(f'{task_id} has more than 20 commands')

    measurements = value['measurements']
    if not isinstance(measurements, dict):
        raise ScorecardError(f'{task_id} measurements must be an object')
    unknown_metrics = set(measurements) - set(MEASUREMENT_FIELDS)
    if unknown_metrics:
        raise ScorecardError(
            f'{task_id} has unknown measurements: '
            + ', '.join(sorted(unknown_metrics)))
    for name, metric in measurements.items():
        _validate_metric(name, metric, task_id)

    checks = value['checks']
    if not isinstance(checks, dict) or set(checks) != set(contract['checks']):
        raise ScorecardError(
            f'{task_id} checks must be: '
            + ', '.join(contract['checks']))
    if not all(item is None or isinstance(item, bool)
               for item in checks.values()):
        raise ScorecardError(f'{task_id} checks must be true, false, or null')

    steps = value['undocumented_manual_steps']
    if steps is not None and (
        isinstance(steps, bool) or not isinstance(steps, int) or steps < 0
    ):
        raise ScorecardError(
            f'{task_id} undocumented_manual_steps must be an integer or null')
    codes = value['finding_codes']
    if not isinstance(codes, list) or len(codes) > 20:
        raise ScorecardError(f'{task_id} finding_codes must be a list')
    if len(set(codes)) != len(codes) or not all(
        isinstance(code, str) and SLUG.fullmatch(code) for code in codes
    ):
        raise ScorecardError(
            f'{task_id} finding_codes must be unique lowercase slugs')
    if 'not-recorded' in codes:
        raise ScorecardError(
            f'{task_id} not-recorded is reserved for the recorder')
    return value


def _load_observations(path: Path) -> dict[str, list[dict[str, Any]]]:
    root = _load_object(path, 'observations')
    if set(root) != {'schema_version', 'products'}:
        raise ScorecardError(
            'observations must contain only schema_version and products')
    if root['schema_version'] != 1:
        raise ScorecardError('unsupported observations schema_version')
    products = root['products']
    if not isinstance(products, dict) or set(products) != set(PRODUCT_IDS):
        raise ScorecardError(
            'observations must contain lidar_slam_ros2 and glim')

    validated: dict[str, list[dict[str, Any]]] = {}
    for product_id in PRODUCT_IDS:
        product = products[product_id]
        if not isinstance(product, dict) or set(product) != {'tasks'}:
            raise ScorecardError(
                f'{product_id} observations must contain only tasks')
        tasks = product['tasks']
        if not isinstance(tasks, list) or len(tasks) != len(TASK_CONTRACTS):
            raise ScorecardError(
                f'{product_id} must contain {len(TASK_CONTRACTS)} tasks')
        validated[product_id] = [
            _validate_task_observation(value, contract)
            for value, contract in zip(tasks, TASK_CONTRACTS)
        ]
    return validated


def _prompt_line(prompt: str) -> str:
    sys.stderr.write(prompt)
    sys.stderr.flush()
    try:
        return input().strip()
    except EOFError:
        return ''


def _prompt_metric(name: str, integer: bool) -> int | float | None:
    while True:
        value = _prompt_line(f'  {name} (blank = not observed): ')
        if not value:
            return None
        try:
            parsed: int | float = int(value) if integer else float(value)
        except ValueError:
            print('  Enter a non-negative number or leave blank.',
                  file=sys.stderr)
            continue
        if isinstance(parsed, float) and not math.isfinite(parsed):
            print('  Enter a finite number or leave blank.', file=sys.stderr)
            continue
        if parsed < 0:
            print('  Enter a non-negative number or leave blank.',
                  file=sys.stderr)
            continue
        return parsed


def _prompt_check(check_id: str) -> bool | None:
    while True:
        value = _prompt_line(
            f'  {check_id} [y/n, blank = not observed]: ').lower()
        if not value:
            return None
        if value in {'y', 'yes'}:
            return True
        if value in {'n', 'no'}:
            return False
        print('  Enter y, n, or leave blank.', file=sys.stderr)


def _prompt_task(contract: Mapping[str, Any]) -> dict[str, Any]:
    task_id = contract['task_id']
    print(f'\nTask: {task_id}', file=sys.stderr)
    print('  Enter each operator-submitted command in order.',
          file=sys.stderr)
    commands = []
    while len(commands) < 20:
        command = _prompt_line(
            f'  command {len(commands) + 1} (blank = done): ')
        if not command:
            break
        commands.append(command)

    measurements: dict[str, int | float | None] = {}
    for name in contract['required_metrics']:
        if name == 'command_count':
            continue
        measurements[name] = _prompt_metric(
            name, name in INTEGER_MEASUREMENTS)
    checks = {
        check_id: _prompt_check(check_id)
        for check_id in contract['checks']
    }
    steps = _prompt_metric('undocumented_manual_steps', True)
    codes_text = _prompt_line(
        '  finding codes (comma-separated lowercase slugs; blank = none): ')
    codes = [code.strip() for code in codes_text.split(',') if code.strip()]
    transcript = _prompt_line(
        '  transcript SHA-256 (blank = not observed): ') or None
    public_url = _prompt_line(
        '  public evidence URL (blank = private/not published): ') or None
    return {
        'task_id': task_id,
        'exact_commands': commands,
        'measurements': measurements,
        'checks': checks,
        'undocumented_manual_steps': steps,
        'finding_codes': codes,
        'transcript_sha256': transcript,
        'public_url': public_url,
    }


def _collect_interactive(
    records: Mapping[str, Mapping[str, Any]],
) -> dict[str, list[dict[str, Any]]]:
    if not sys.stdin.isatty():
        raise ScorecardError(
            'interactive recording requires a TTY; use --observations for '
            'non-interactive input')
    print(
        'Record only direct observations from this exact paired trial. '
        'Blank values remain incomplete and never count as evidence.',
        file=sys.stderr,
    )
    ordered = sorted(
        PRODUCT_IDS,
        key=lambda product_id: (
            records[product_id]['operator']['product_order'] != 'first'
        ),
    )
    result = {}
    for product_id in ordered:
        record = records[product_id]
        print(
            f'\nProduct: {product_id} '
            f'({record["operator"]["product_order"]})\n'
            f'Docs: {record["product"]["documentation_root_url"]}',
            file=sys.stderr,
        )
        result[product_id] = [
            _validate_task_observation(_prompt_task(contract), contract)
            for contract in TASK_CONTRACTS
        ]
    return result


def _apply_observations(
    records: Mapping[str, dict[str, Any]],
    observations: Mapping[str, Sequence[Mapping[str, Any]]],
) -> list[dict[str, Any]]:
    completed = []
    for product_id in PRODUCT_IDS:
        record = deepcopy(records[product_id])
        for task, observation, contract in zip(
            record['tasks'], observations[product_id], TASK_CONTRACTS
        ):
            task['exact_commands'] = list(observation['exact_commands'])
            task['measurements'] = {
                name: None for name in task['measurements']
            }
            task['measurements'].update(observation['measurements'])
            task['measurements']['command_count'] = len(
                task['exact_commands'])
            observed_checks = observation['checks']
            task['checks'] = [
                {
                    'id': check_id,
                    'passed': observed_checks[check_id] is True,
                }
                for check_id in contract['checks']
            ]
            task['outcome']['status'] = (
                'PASS' if all(item['passed'] for item in task['checks'])
                else 'FAIL'
            )
            steps = observation['undocumented_manual_steps']
            task['outcome']['undocumented_manual_steps'] = (
                0 if steps is None else steps
            )
            codes = list(observation['finding_codes'])
            required_metrics = (
                name for name in contract['required_metrics']
                if name != 'command_count'
            )
            incomplete = (
                any(
                    task['measurements'][name] is None
                    for name in required_metrics
                )
                or any(value is None for value in observed_checks.values())
                or steps is None
                or observation['transcript_sha256'] is None
            )
            if incomplete:
                codes.append('not-recorded')
            task['outcome']['finding_codes'] = codes
            task['evidence'] = {
                'transcript_sha256': observation['transcript_sha256'],
                'public_url': observation['public_url'],
            }
        validate_trial(record)
        completed.append(record)
    return completed


def _write_pair_atomic(
    output_dir: Path,
    records: Sequence[Mapping[str, Any]],
    preparation_receipt: bytes,
    prepared_worksheets: Mapping[str, bytes],
) -> tuple[list[Path], Path, list[Path]]:
    if output_dir.exists() or output_dir.is_symlink():
        raise ScorecardError(
            f'refusing to overwrite output directory: {output_dir}')
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(
        prefix=f'.{output_dir.name}.', dir=output_dir.parent))
    names = [f'{record["trial_id"]}.json' for record in records]
    try:
        for name, record in zip(names, records):
            payload = json.dumps(record, indent=2, sort_keys=True) + '\n'
            (temporary / name).write_text(payload, encoding='utf-8')
        preparation_dir = temporary / PREPARATION_ARCHIVE_DIR
        preparation_dir.mkdir()
        for name, payload in prepared_worksheets.items():
            (preparation_dir / name).write_bytes(payload)
        (preparation_dir / PREPARATION_RECEIPT_NAME).write_bytes(
            preparation_receipt
        )
        os.replace(temporary, output_dir)
    except (OSError, TypeError) as exc:
        shutil.rmtree(temporary, ignore_errors=True)
        raise ScorecardError(f'cannot publish recorded pair: {exc}') from exc
    archived_paths = [
        output_dir / PREPARATION_ARCHIVE_DIR / name
        for name in prepared_worksheets
    ]
    return (
        [output_dir / name for name in names],
        output_dir / PREPARATION_ARCHIVE_DIR / PREPARATION_RECEIPT_NAME,
        archived_paths,
    )


def _validation_command(
    paths: Sequence[Path],
    preparation_receipt: Path,
) -> str:
    command = ['python3', 'scripts/check_usability_scorecard.py']
    for path in paths:
        command.extend(('--record', str(path)))
    command.extend(('--preparation-receipt', str(preparation_receipt)))
    command.append('--json')
    return shlex.join(command)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--record', action='append', type=Path, required=True,
        help='Prepared worksheet; provide once per product.',
    )
    parser.add_argument(
        '--observations', type=Path,
        help='Non-interactive paired observation JSON; otherwise prompt.',
    )
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--json', action='store_true')
    parser.add_argument(
        '--require-ready', action='store_true',
        help='Exit 1 after writing when the pair is not READY.',
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Record and atomically publish one validated local pair."""
    args = _parser().parse_args(argv)
    try:
        (
            prepared,
            preparation_receipt,
            prepared_worksheets,
        ) = _load_pair(args.record)
        observations = (
            _load_observations(args.observations)
            if args.observations is not None
            else _collect_interactive(prepared)
        )
        records = _apply_observations(prepared, observations)
        report = evaluate_scorecard(records)
        paths, receipt_path, archived_paths = _write_pair_atomic(
            args.output_dir,
            records,
            preparation_receipt,
            prepared_worksheets,
        )
    except (OSError, ScorecardError) as exc:
        print(f'usability pair recorder error: {exc}', file=sys.stderr)
        return 2

    result = {
        'status': report['status'],
        'summary': report['summary'],
        'files': [str(path) for path in paths],
        'preparation_receipt': str(receipt_path),
        'preparation_receipt_sha256': hashlib.sha256(
            preparation_receipt
        ).hexdigest(),
        'prepared_worksheets': [str(path) for path in archived_paths],
        'validation_command': _validation_command(paths, receipt_path),
        'remote_mutations_performed': False,
        'automatic_winner_claim_authorized': False,
    }
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(f'Recorded paired scorecard: {result["status"]}')
        for path in paths:
            print(f'  {path}')
        print('Validate with:')
        print(f'  {result["validation_command"]}')
    if args.require_ready and report['status'] != 'READY':
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
