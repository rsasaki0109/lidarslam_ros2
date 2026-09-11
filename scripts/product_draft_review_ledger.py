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

"""Prepare and validate an anonymous exact-tip Draft review ledger."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Sequence
import uuid

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
ROUTING_SCRIPT = REPO_ROOT / 'scripts' / 'check_product_draft_review_routing.py'
PUBLICATION_SCRIPT = REPO_ROOT / 'scripts' / 'check_publication_slice_plan.py'
ROUTING_CONTRACT_PATH = (
    REPO_ROOT / 'docs' / 'contracts'
    / 'product-draft-review-routing-v1.json'
)
PUBLICATION_PLAN_PATH = (
    REPO_ROOT / 'docs' / 'evidence' / 'growth'
    / 'g0-publication-slice-plan-2026-08-12.json'
)
LEDGER_SCHEMA_PATH = (
    REPO_ROOT / 'docs' / 'schemas'
    / 'product-draft-review-ledger-v1.schema.json'
)
REPORT_SCHEMA_PATH = (
    REPO_ROOT / 'docs' / 'schemas'
    / 'product-draft-review-ledger-report-v1.schema.json'
)
LEDGER_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/'
    'product-draft-review-ledger-v1.schema.json'
)
REPORT_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/'
    'product-draft-review-ledger-report-v1.schema.json'
)
REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
PULL_REQUEST = 427
LEDGER_SCOPE = 'anonymous-product-draft-review-ledger'
REPORT_SCOPE = 'anonymous-product-draft-review-ledger-report'
MAX_JSON_BYTES = 2 * 1024 * 1024
EXPECTED_LANE_IDS = (
    'R1-runtime-safety',
    'R2-operator-ux',
    'R3-distribution',
    'R4-integration-publication',
)
FINDING_SEVERITIES = ('BLOCKER', 'NOTE')
FINDING_CODES = (
    'correctness-risk',
    'safety-risk',
    'operator-ux-gap',
    'documentation-gap',
    'verification-failure',
    'distribution-blocker',
    'publication-boundary',
    'maintainability-note',
)
PRIVATE_DETAIL_PATTERNS = (
    re.compile(r'@'),
    re.compile(r'(?i)\b(?:https?|file)://'),
    re.compile(r'(?i)\bwww\.'),
    re.compile(r'(?i)(?:^|\s)(?:/home/|/Users/|~[/\\])'),
    re.compile(r'(?i)(?:^|\s)[A-Z]:\\'),
    re.compile(r'[<>]'),
)
AUTHORITY = {
    'identities_collected': False,
    'review_commands_executed_by_tool': False,
    'github_reviewer_requests_authorized': False,
    'github_reviews_authorized': False,
    'mark_ready_authorized': False,
    'merge_authorized': False,
    'remote_mutations_performed': False,
}


class ReviewLedgerError(RuntimeError):
    """Raised when local review evidence is unsafe, stale, or incomplete."""


def _canonical_payload(value: dict[str, Any]) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True) + '\n').encode()


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _load_json_bytes(
    path: Path,
    label: str,
    *,
    require_canonical: bool = False,
) -> tuple[dict[str, Any], bytes]:
    if path.is_symlink():
        raise ReviewLedgerError(f'{label} must not be a symlink')
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise ReviewLedgerError(f'{label} cannot be read: {exc}') from exc
    if len(payload) > MAX_JSON_BYTES:
        raise ReviewLedgerError(f'{label} exceeds the size limit')
    try:
        value = json.loads(payload.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ReviewLedgerError(f'{label} is not valid UTF-8 JSON: {exc}') from exc
    if not isinstance(value, dict):
        raise ReviewLedgerError(f'{label} must be a JSON object')
    if require_canonical and payload != _canonical_payload(value):
        raise ReviewLedgerError(
            f'{label} is not in canonical recorder format'
        )
    return value, payload


def _validate_schema(
    value: dict[str, Any],
    schema_path: Path,
    label: str,
) -> None:
    schema, _ = _load_json_bytes(schema_path, f'{label} schema')
    try:
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(schema).validate(value)
    except jsonschema.SchemaError as exc:
        raise ReviewLedgerError(
            f'{label} schema is invalid: {exc.message}'
        ) from exc
    except jsonschema.ValidationError as exc:
        location = '.'.join(str(item) for item in exc.absolute_path)
        raise ReviewLedgerError(
            f'{label} failed at {location or "<root>"}: {exc.message}'
        ) from exc


def _run_json(
    command: list[str],
    label: str,
    *,
    runner: Any = subprocess.run,
) -> dict[str, Any]:
    result = runner(
        command,
        cwd=REPO_ROOT,
        capture_output=True,
        check=False,
    )
    if len(result.stdout) > MAX_JSON_BYTES:
        raise ReviewLedgerError(f'{label} output exceeds the size limit')
    if result.returncode != 0:
        detail = result.stderr.decode('utf-8', errors='replace').strip()
        raise ReviewLedgerError(
            f'{label} failed: {detail or result.returncode}'
        )
    try:
        value = json.loads(result.stdout.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ReviewLedgerError(f'{label} output is invalid JSON: {exc}') from exc
    if not isinstance(value, dict):
        raise ReviewLedgerError(f'{label} output must be an object')
    return value


def collect_review_context(*, runner: Any = subprocess.run) -> dict[str, Any]:
    """Bind routing and exact slice paths to one clean local candidate."""
    routing = _run_json(
        [sys.executable, str(ROUTING_SCRIPT), '--json'],
        'review routing',
        runner=runner,
    )
    plan = _run_json(
        [sys.executable, str(PUBLICATION_SCRIPT), '--json'],
        'publication plan',
        runner=runner,
    )
    source, _ = _load_json_bytes(PUBLICATION_PLAN_PATH, 'publication source')
    raw_slices = source.get('review_slices')
    raw_lanes = routing.get('lanes')
    if (
        routing.get('status') != 'READY_LOCAL_ONLY'
        or routing.get('worktree_clean') is not True
        or routing.get('uncommitted_path_count') != 0
        or routing.get('authority') != {
            'commands_executed': False,
            'github_reviewer_requests_authorized': False,
            'github_reviews_authorized': False,
            'mark_ready_authorized': False,
            'merge_authorized': False,
            'remote_mutations_performed': False,
        }
        or plan.get('status') != 'PLAN_VALID_LOCAL_ONLY'
        or plan.get('worktree_clean') is not True
        or plan.get('uncommitted_path_count') != 0
        or plan.get('local_tip_sha') != routing.get('exact_head')
        or plan.get('review_coverage_complete') is not True
        or not isinstance(raw_slices, list)
        or len(raw_slices) != 7
        or not isinstance(raw_lanes, list)
        or len(raw_lanes) != len(EXPECTED_LANE_IDS)
    ):
        raise ReviewLedgerError(
            'review ledger requires one clean, exact, no-write routing context'
        )
    paths_by_slice: dict[str, tuple[str, ...]] = {}
    for raw_slice in raw_slices:
        if not isinstance(raw_slice, dict):
            raise ReviewLedgerError('publication source contains a malformed slice')
        slice_id = raw_slice.get('id')
        paths = raw_slice.get('paths')
        if (
            not isinstance(slice_id, str)
            or not isinstance(paths, list)
            or not paths
            or not all(isinstance(path, str) and path for path in paths)
            or paths != sorted(paths)
            or len(paths) != len(set(paths))
        ):
            raise ReviewLedgerError(
                f'publication source slice {slice_id!r} has unsafe paths'
            )
        paths_by_slice[slice_id] = tuple(paths)
    lanes: list[dict[str, Any]] = []
    for order, (raw_lane, expected_id) in enumerate(
        zip(raw_lanes, EXPECTED_LANE_IDS),
        start=1,
    ):
        if not isinstance(raw_lane, dict):
            raise ReviewLedgerError(f'review lane {expected_id} is malformed')
        slice_ids = raw_lane.get('slice_ids')
        dependencies = raw_lane.get('depends_on_lanes')
        if (
            raw_lane.get('id') != expected_id
            or raw_lane.get('order') != order
            or not isinstance(slice_ids, list)
            or not slice_ids
            or not all(slice_id in paths_by_slice for slice_id in slice_ids)
            or not isinstance(dependencies, list)
            or not all(item in EXPECTED_LANE_IDS for item in dependencies)
            or raw_lane.get('path_count')
            != sum(len(paths_by_slice[item]) for item in slice_ids)
        ):
            raise ReviewLedgerError(f'review lane {expected_id} is invalid')
        lanes.append({
            'id': expected_id,
            'order': order,
            'slice_ids': tuple(slice_ids),
            'depends_on_lanes': tuple(dependencies),
        })
    _, routing_contract_payload = _load_json_bytes(
        ROUTING_CONTRACT_PATH,
        'review routing contract',
    )
    return {
        'exact_head': routing['exact_head'],
        'worktree_clean': True,
        'routing_contract_sha256': _sha256(routing_contract_payload),
        'lanes': lanes,
        'paths_by_slice': paths_by_slice,
    }


def prepare_ledger(context: dict[str, Any]) -> dict[str, Any]:
    """Create one empty identity-free ledger bound to the exact clean tip."""
    ledger = {
        'schema_version': 1,
        'schema_uri': LEDGER_SCHEMA_URI,
        'repository': REPOSITORY,
        'pull_request': PULL_REQUEST,
        'scope': LEDGER_SCOPE,
        'exact_head': context['exact_head'],
        'routing_contract_sha256': context['routing_contract_sha256'],
        'events': [],
        'authority': dict(AUTHORITY),
    }
    _validate_schema(ledger, LEDGER_SCHEMA_PATH, 'review ledger')
    return ledger


def _safe_detail(detail: str) -> bool:
    return (
        bool(detail)
        and len(detail) <= 240
        and '\n' not in detail
        and '\r' not in detail
        and not any(pattern.search(detail) for pattern in PRIVATE_DETAIL_PATTERNS)
    )


def _lane_map(context: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {lane['id']: lane for lane in context['lanes']}


def _latest_events(ledger: dict[str, Any]) -> dict[str, dict[str, Any]]:
    latest: dict[str, dict[str, Any]] = {}
    for event in ledger['events']:
        latest[event['lane_id']] = event
    return latest


def _validate_ledger_semantics(
    ledger: dict[str, Any],
    context: dict[str, Any],
) -> None:
    if (
        ledger.get('exact_head') != context.get('exact_head')
        or ledger.get('routing_contract_sha256')
        != context.get('routing_contract_sha256')
        or ledger.get('authority') != AUTHORITY
    ):
        raise ReviewLedgerError(
            'review ledger is stale or claims unsafe authority'
        )
    lanes = _lane_map(context)
    latest: dict[str, dict[str, Any]] = {}
    for sequence, event in enumerate(ledger['events'], start=1):
        lane = lanes.get(event['lane_id'])
        if (
            event['sequence'] != sequence
            or lane is None
            or event['slice_ids'] != list(lane['slice_ids'])
        ):
            raise ReviewLedgerError(
                f'review event {sequence} has invalid lane scope or sequence'
            )
        later_reviewed = [
            item['id']
            for item in context['lanes']
            if item['order'] > lane['order'] and item['id'] in latest
        ]
        if event['lane_id'] in latest and later_reviewed:
            raise ReviewLedgerError(
                f'review event {sequence} would stale downstream lanes '
                f'{later_reviewed}'
            )
        unmet = [
            dependency for dependency in lane['depends_on_lanes']
            if latest.get(dependency, {}).get('outcome') != 'PASS'
        ]
        if unmet:
            raise ReviewLedgerError(
                f'review event {sequence} has unmet lane dependencies {unmet}'
            )
        blocker_count = 0
        for finding_index, finding in enumerate(event['findings'], start=1):
            expected_finding_id = f'E{sequence:03d}-F{finding_index:02d}'
            slice_id = finding['slice_id']
            if (
                finding['id'] != expected_finding_id
                or slice_id not in lane['slice_ids']
                or finding['path']
                not in context['paths_by_slice'].get(slice_id, ())
                or not _safe_detail(finding['detail'])
            ):
                raise ReviewLedgerError(
                    f'review finding {expected_finding_id} is unsafe or out of scope'
                )
            blocker_count += finding['severity'] == 'BLOCKER'
        if (
            event['outcome'] == 'PASS'
            and (
                event['verification_status'] != 'PASS'
                or blocker_count != 0
            )
        ):
            raise ReviewLedgerError(
                f'review event {sequence} cannot claim PASS'
            )
        if event['outcome'] == 'BLOCKED' and blocker_count == 0:
            raise ReviewLedgerError(
                f'review event {sequence} needs an actionable blocker'
            )
        latest[event['lane_id']] = event


def build_report(
    ledger: dict[str, Any],
    context: dict[str, Any],
    *,
    ledger_payload: bytes | None = None,
) -> dict[str, Any]:
    """Validate append-only events and derive the current lane state."""
    _validate_schema(ledger, LEDGER_SCHEMA_PATH, 'review ledger')
    _validate_ledger_semantics(ledger, context)
    payload = ledger_payload or _canonical_payload(ledger)
    latest = _latest_events(ledger)
    current_lanes: list[dict[str, Any]] = []
    for lane in context['lanes']:
        event = latest.get(lane['id'])
        findings = event['findings'] if event else []
        current_lanes.append({
            'id': lane['id'],
            'order': lane['order'],
            'slice_ids': list(lane['slice_ids']),
            'status': event['outcome'] if event else 'NOT_REVIEWED',
            'verification_status': (
                event['verification_status'] if event else 'NOT_RECORDED'
            ),
            'latest_event_sequence': event['sequence'] if event else None,
            'finding_count': len(findings),
            'blocker_count': sum(
                finding['severity'] == 'BLOCKER' for finding in findings
            ),
        })
    passing_lane_count = sum(
        lane['status'] == 'PASS' for lane in current_lanes
    )
    blocked_lane_count = sum(
        lane['status'] == 'BLOCKED' for lane in current_lanes
    )
    reviewed_lane_count = passing_lane_count + blocked_lane_count
    if reviewed_lane_count == 0:
        status = 'EMPTY_LOCAL_LEDGER'
    elif blocked_lane_count:
        status = 'BLOCKED_LOCAL_REVIEW'
    elif passing_lane_count == len(EXPECTED_LANE_IDS):
        status = 'COMPLETE_LOCAL_REVIEW'
    else:
        status = 'IN_PROGRESS_LOCAL_REVIEW'
    next_lane_id = next(
        (lane['id'] for lane in current_lanes if lane['status'] != 'PASS'),
        None,
    )
    current_events = list(latest.values())
    report = {
        'schema_version': 1,
        'schema_uri': REPORT_SCHEMA_URI,
        'repository': REPOSITORY,
        'pull_request': PULL_REQUEST,
        'scope': REPORT_SCOPE,
        'status': status,
        'exact_head': ledger['exact_head'],
        'ledger_sha256': _sha256(payload),
        'routing_contract_sha256': ledger['routing_contract_sha256'],
        'worktree_clean': True,
        'event_count': len(ledger['events']),
        'reviewed_lane_count': reviewed_lane_count,
        'passing_lane_count': passing_lane_count,
        'blocked_lane_count': blocked_lane_count,
        'historical_finding_count': sum(
            len(event['findings']) for event in ledger['events']
        ),
        'current_finding_count': sum(
            len(event['findings']) for event in current_events
        ),
        'open_blocker_count': sum(
            finding['severity'] == 'BLOCKER'
            for event in current_events
            for finding in event['findings']
        ),
        'current_lanes': current_lanes,
        'next_lane_id': next_lane_id,
        'authority': dict(AUTHORITY),
    }
    _validate_schema(report, REPORT_SCHEMA_PATH, 'review ledger report')
    return report


def append_event(
    ledger: dict[str, Any],
    context: dict[str, Any],
    *,
    lane_id: str,
    outcome: str,
    verification_status: str,
    raw_findings: list[list[str]],
) -> dict[str, Any]:
    """Append one complete lane outcome without storing reviewer identity."""
    build_report(ledger, context)
    lanes = _lane_map(context)
    lane = lanes.get(lane_id)
    if lane is None:
        raise ReviewLedgerError(f'unknown review lane: {lane_id}')
    sequence = len(ledger['events']) + 1
    findings: list[dict[str, Any]] = []
    for index, raw in enumerate(raw_findings, start=1):
        severity, code, slice_id, path, detail = raw
        if severity not in FINDING_SEVERITIES or code not in FINDING_CODES:
            raise ReviewLedgerError(
                f'review finding {index} has an unsupported severity or code'
            )
        findings.append({
            'id': f'E{sequence:03d}-F{index:02d}',
            'severity': severity,
            'code': code,
            'slice_id': slice_id,
            'path': path,
            'detail': detail,
        })
    updated = json.loads(json.dumps(ledger))
    updated['events'].append({
        'sequence': sequence,
        'lane_id': lane_id,
        'slice_ids': list(lane['slice_ids']),
        'outcome': outcome,
        'verification_status': verification_status,
        'findings': findings,
    })
    build_report(updated, context)
    return updated


def _outside_repository(path: Path) -> bool:
    try:
        path.resolve().relative_to(REPO_ROOT.resolve())
    except ValueError:
        return True
    return False


def _write_ledger(path: Path, ledger: dict[str, Any], *, replace: bool) -> None:
    if not _outside_repository(path):
        raise ReviewLedgerError(
            'review ledger output must stay outside the source repository'
        )
    if not path.parent.is_dir():
        raise ReviewLedgerError('review ledger output parent does not exist')
    if path.is_symlink():
        raise ReviewLedgerError('review ledger output must not be a symlink')
    if not replace and path.exists():
        raise ReviewLedgerError('review ledger output already exists')
    if replace and (not path.is_file() or path.is_symlink()):
        raise ReviewLedgerError('review ledger update target is not a regular file')
    payload = _canonical_payload(ledger)
    temporary = path.with_name(f'.{path.name}.{uuid.uuid4().hex}.tmp')
    try:
        with temporary.open('xb') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except OSError as exc:
        temporary.unlink(missing_ok=True)
        raise ReviewLedgerError(f'review ledger write failed: {exc}') from exc


def load_ledger(path: Path) -> tuple[dict[str, Any], bytes]:
    return _load_json_bytes(path, 'review ledger', require_canonical=True)


def render_card(report: dict[str, Any], ledger: dict[str, Any]) -> str:
    """Render current anonymous lane results and one safe next action."""
    lines = [
        '# Product Draft review ledger',
        '',
        f"- Status: **{report['status']}**",
        f"- Exact head: `{report['exact_head']}`",
        f"- Ledger SHA-256: `{report['ledger_sha256']}`",
        (
            '- Lanes: '
            f"{report['passing_lane_count']} pass / "
            f"{report['blocked_lane_count']} blocked / "
            f"{4 - report['reviewed_lane_count']} not reviewed"
        ),
        f"- Current blockers: {report['open_blocker_count']}",
        '- Reviewer identities collected: none',
        '- GitHub review submitted: no',
        '',
        '| Lane | Slices | State | Verification | Findings | Blockers |',
        '| --- | --- | --- | --- | ---: | ---: |',
    ]
    for lane in report['current_lanes']:
        lines.append(
            f"| `{lane['id']}` | {', '.join(lane['slice_ids'])} | "
            f"{lane['status']} | {lane['verification_status']} | "
            f"{lane['finding_count']} | {lane['blocker_count']} |"
        )
    latest = _latest_events(ledger)
    current_findings = [
        finding
        for lane in report['current_lanes']
        for finding in latest.get(lane['id'], {}).get('findings', [])
    ]
    if current_findings:
        lines.extend(['', 'Current findings:'])
        for finding in current_findings:
            lines.append(
                f"- `{finding['id']}` {finding['severity']} "
                f"`{finding['code']}` in `{finding['path']}`: "
                f"{finding['detail']}"
            )
    lines.extend(['', 'Next action:'])
    if report['status'] == 'COMPLETE_LOCAL_REVIEW':
        lines.append(
            'Review this local evidence separately before any GitHub review, '
            'mark-ready, or merge decision.'
        )
    elif report['status'] == 'BLOCKED_LOCAL_REVIEW':
        lines.append(
            f"Resolve and rerun `{report['next_lane_id']}`; append a new "
            'PASS or BLOCKED event instead of editing history.'
        )
    else:
        lines.append(
            'Render the next bounded lane: '
            '`python3 scripts/check_product_draft_review_routing.py '
            f"--lane {report['next_lane_id']}`."
        )
    lines.append(
        'Boundary: local evidence only; the tool runs no review command and '
        'grants no reviewer-request, submitted-review, mark-ready, merge, or '
        'remote-write authority.'
    )
    return '\n'.join(lines)


def _add_output_flags(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--require-complete', action='store_true')


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Record anonymous, exact-tip product Draft lane review evidence.'
        )
    )
    subparsers = parser.add_subparsers(dest='command', required=True)
    prepare = subparsers.add_parser('prepare')
    prepare.add_argument('--output', type=Path, required=True)
    _add_output_flags(prepare)
    record = subparsers.add_parser('record')
    record.add_argument('--ledger', type=Path, required=True)
    record.add_argument('--lane', choices=EXPECTED_LANE_IDS, required=True)
    record.add_argument('--outcome', choices=('PASS', 'BLOCKED'), required=True)
    record.add_argument(
        '--verification-status',
        choices=('PASS', 'FAIL'),
        required=True,
    )
    record.add_argument(
        '--finding',
        action='append',
        nargs=5,
        default=[],
        metavar=('SEVERITY', 'CODE', 'SLICE', 'PATH', 'DETAIL'),
    )
    _add_output_flags(record)
    check = subparsers.add_parser('check')
    check.add_argument('--ledger', type=Path, required=True)
    _add_output_flags(check)
    return parser.parse_args(argv)


def _emit(
    report: dict[str, Any],
    ledger: dict[str, Any],
    *,
    as_json: bool,
) -> None:
    if as_json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(render_card(report, ledger))


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        context = collect_review_context()
        if args.command == 'prepare':
            path = args.output.expanduser().resolve()
            ledger = prepare_ledger(context)
            _write_ledger(path, ledger, replace=False)
            report = build_report(
                ledger,
                context,
                ledger_payload=_canonical_payload(ledger),
            )
        else:
            path = args.ledger.expanduser().resolve()
            ledger, payload = load_ledger(path)
            if args.command == 'record':
                ledger = append_event(
                    ledger,
                    context,
                    lane_id=args.lane,
                    outcome=args.outcome,
                    verification_status=args.verification_status,
                    raw_findings=args.finding,
                )
                _write_ledger(path, ledger, replace=True)
                payload = _canonical_payload(ledger)
            report = build_report(ledger, context, ledger_payload=payload)
        _emit(report, ledger, as_json=args.json)
        if args.require_complete and report['status'] != 'COMPLETE_LOCAL_REVIEW':
            return 2
    except ReviewLedgerError as exc:
        print(f'product Draft review ledger failed: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
