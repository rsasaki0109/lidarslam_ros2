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

"""Validate privacy-safe role routing for the large product Draft review."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Sequence

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = (
    REPO_ROOT / 'docs' / 'contracts'
    / 'product-draft-review-routing-v1.json'
)
CONTRACT_SCHEMA_PATH = (
    REPO_ROOT / 'docs' / 'schemas'
    / 'product-draft-review-routing-v1.schema.json'
)
REPORT_SCHEMA_PATH = (
    REPO_ROOT / 'docs' / 'schemas'
    / 'product-draft-review-routing-report-v1.schema.json'
)
REPORT_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/'
    'product-draft-review-routing-report-v1.schema.json'
)
REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
PULL_REQUEST = 427
SCOPE = 'role-based-product-draft-review-routing'
MAX_OVERVIEW_BYTES = 2 * 1024 * 1024
SHA_PATTERN = re.compile(r'^[0-9a-f]{40}$')
EXPECTED_LANE_IDS = (
    'R1-runtime-safety',
    'R2-operator-ux',
    'R3-distribution',
    'R4-integration-publication',
)
EXPECTED_SLICE_IDS = (
    'S1-runtime-safety',
    'S2-first-map-foundation',
    'S3-map-lifecycle',
    'S4-source-onboarding',
    'S5-distribution-readiness',
    'S6-product-shell-integration',
    'S7-publication-control',
)
OVERVIEW_COMMAND = (
    'python3 scripts/check_publication_slice_plan.py --overview --json'
)
SLICE_COMMAND_PREFIX = (
    'python3 scripts/check_publication_slice_plan.py --slice '
)


class ReviewRoutingError(RuntimeError):
    """Raised when routing cannot be bound to the exact review overview."""


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ReviewRoutingError(f'{label} cannot be loaded: {exc}') from exc
    if not isinstance(value, dict):
        raise ReviewRoutingError(f'{label} must be a JSON object')
    return value


def _validate(
    instance: dict[str, Any],
    schema_path: Path,
    label: str,
) -> None:
    schema = _load_json(schema_path, f'{label} schema')
    try:
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(schema).validate(instance)
    except jsonschema.SchemaError as exc:
        raise ReviewRoutingError(
            f'{label} schema is invalid: {exc.message}'
        ) from exc
    except jsonschema.ValidationError as exc:
        location = '.'.join(str(item) for item in exc.absolute_path)
        raise ReviewRoutingError(
            f'{label} failed at {location or "<root>"}: {exc.message}'
        ) from exc


def collect_publication_overview() -> dict[str, Any]:
    """Run the existing local overview without executing displayed checks."""
    result = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / 'scripts' / 'check_publication_slice_plan.py'),
            '--overview',
            '--json',
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        check=False,
    )
    if len(result.stdout) > MAX_OVERVIEW_BYTES:
        raise ReviewRoutingError('publication overview output is too large')
    if result.returncode != 0:
        detail = result.stderr.decode('utf-8', errors='replace').strip()
        raise ReviewRoutingError(
            f'publication overview failed: {detail or result.returncode}'
        )
    try:
        overview = json.loads(result.stdout.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ReviewRoutingError(
            f'publication overview is not valid JSON: {exc}'
        ) from exc
    if not isinstance(overview, dict):
        raise ReviewRoutingError('publication overview must be an object')
    return overview


def _positive_integer(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, int)
        and value > 0
    )


def build_report(
    contract: dict[str, Any],
    overview: dict[str, Any],
) -> dict[str, Any]:
    """Bind every validated slice to one capability lane and exact tip."""
    _validate(contract, CONTRACT_SCHEMA_PATH, 'review routing contract')
    candidate = overview.get('candidate')
    raw_slices = overview.get('review_slices')
    if (
        overview.get('status')
        != 'PR_REVIEW_OVERVIEW_READY_LOCAL_ONLY'
        or overview.get('commands_executed') is not False
        or overview.get('github_writes_authorized') is not False
        or overview.get('remote_mutations_performed') is not False
        or not isinstance(candidate, dict)
        or not isinstance(raw_slices, list)
        or len(raw_slices) != len(EXPECTED_SLICE_IDS)
        or candidate.get('review_coverage_complete') is not True
        or candidate.get('merge_commit_count') != 0
        or not isinstance(candidate.get('worktree_clean'), bool)
        or isinstance(candidate.get('uncommitted_path_count'), bool)
        or not isinstance(candidate.get('uncommitted_path_count'), int)
        or candidate.get('uncommitted_path_count') < 0
    ):
        raise ReviewRoutingError(
            'review routing requires one bounded, no-write exact overview'
        )
    exact_head = candidate.get('local_tip_sha')
    if (
        not isinstance(exact_head, str)
        or SHA_PATTERN.fullmatch(exact_head) is None
    ):
        raise ReviewRoutingError('review routing exact head is invalid')

    slices: dict[str, dict[str, Any]] = {}
    for order, (raw_slice, expected_id) in enumerate(
        zip(raw_slices, EXPECTED_SLICE_IDS),
        start=1,
    ):
        if not isinstance(raw_slice, dict):
            raise ReviewRoutingError('review overview has a malformed slice')
        depends_on = raw_slice.get('depends_on')
        if (
            raw_slice.get('id') != expected_id
            or raw_slice.get('order') != order
            or not _positive_integer(raw_slice.get('path_count'))
            or not _positive_integer(raw_slice.get('verification_count'))
            or not isinstance(depends_on, list)
            or not all(item in EXPECTED_SLICE_IDS for item in depends_on)
            or len(depends_on) != len(set(depends_on))
        ):
            raise ReviewRoutingError(
                f'review overview slice {expected_id} is invalid'
            )
        slices[expected_id] = raw_slice

    raw_lanes = contract['lanes']
    assigned_slice_ids = [
        slice_id
        for lane in raw_lanes
        for slice_id in lane['slice_ids']
    ]
    missing = sorted(set(EXPECTED_SLICE_IDS) - set(assigned_slice_ids))
    duplicates = sorted({
        slice_id for slice_id in assigned_slice_ids
        if assigned_slice_ids.count(slice_id) > 1
    })
    extra = sorted(set(assigned_slice_ids) - set(EXPECTED_SLICE_IDS))
    if missing or duplicates or extra:
        raise ReviewRoutingError(
            'review lanes do not assign every slice exactly once: '
            f'missing={missing}, duplicates={duplicates}, extra={extra}'
        )
    slice_to_lane = {
        slice_id: lane['id']
        for lane in raw_lanes
        for slice_id in lane['slice_ids']
    }
    lane_order = {lane['id']: lane['order'] for lane in raw_lanes}
    normalized_lanes: list[dict[str, Any]] = []
    for order, (lane, expected_id) in enumerate(
        zip(raw_lanes, EXPECTED_LANE_IDS),
        start=1,
    ):
        if lane['id'] != expected_id or lane['order'] != order:
            raise ReviewRoutingError(
                f'review lane {expected_id} order or identity is invalid'
            )
        required_lane_dependencies = sorted({
            slice_to_lane[dependency]
            for slice_id in lane['slice_ids']
            for dependency in slices[slice_id]['depends_on']
            if slice_to_lane[dependency] != lane['id']
        }, key=lane_order.__getitem__)
        if lane['depends_on_lanes'] != required_lane_dependencies or any(
            lane_order[dependency] >= order
            for dependency in lane['depends_on_lanes']
        ):
            raise ReviewRoutingError(
                f'review lane {expected_id} dependencies are not exact'
            )
        path_count = sum(
            slices[slice_id]['path_count']
            for slice_id in lane['slice_ids']
        )
        verification_count = sum(
            slices[slice_id]['verification_count']
            for slice_id in lane['slice_ids']
        )
        if (
            path_count != lane['expected_path_count']
            or verification_count
            != lane['expected_verification_count']
        ):
            raise ReviewRoutingError(
                f'review lane {expected_id} budget is stale'
            )
        normalized_lanes.append({
            'id': lane['id'],
            'order': lane['order'],
            'title': lane['title'],
            'capability': lane['capability'],
            'slice_ids': list(lane['slice_ids']),
            'depends_on_lanes': list(lane['depends_on_lanes']),
            'path_count': path_count,
            'verification_count': verification_count,
            'slice_commands': [
                f'{SLICE_COMMAND_PREFIX}{slice_id}'
                for slice_id in lane['slice_ids']
            ],
        })
    total_paths = sum(item['path_count'] for item in normalized_lanes)
    total_verifications = sum(
        item['verification_count'] for item in normalized_lanes
    )
    if (
        total_paths != candidate.get('follow_up_path_count')
        or len(assigned_slice_ids) != candidate.get('slice_count')
    ):
        raise ReviewRoutingError(
            'review lane totals contradict the exact candidate overview'
        )
    policy = contract['policy']
    authority = contract['authority']
    report = {
        'schema_version': 1,
        'schema_uri': REPORT_SCHEMA_URI,
        'repository': REPOSITORY,
        'pull_request': PULL_REQUEST,
        'scope': SCOPE,
        'status': (
            'READY_LOCAL_ONLY'
            if candidate['worktree_clean']
            else 'PREPARED_DIRTY_WORKTREE'
        ),
        'exact_head': exact_head,
        'worktree_clean': candidate['worktree_clean'],
        'uncommitted_path_count': candidate['uncommitted_path_count'],
        'advisory_reviewer_target': policy['advisory_reviewer_target'],
        'advisory_target_is_merge_gate': (
            policy['advisory_target_is_merge_gate']
        ),
        'max_parallel_active_lanes': policy[
            'max_parallel_active_lanes'
        ],
        'lane_completion_order_required': policy[
            'lane_completion_order_required'
        ],
        'final_decision_role': policy['final_decision_role'],
        'lanes': normalized_lanes,
        'summary': {
            'lane_count': len(normalized_lanes),
            'slice_count': len(assigned_slice_ids),
            'path_count': total_paths,
            'verification_count': total_verifications,
            'unassigned_slice_count': len(missing),
            'duplicate_slice_count': len(duplicates),
        },
        'authority': {
            'commands_executed': False,
            'github_reviewer_requests_authorized': authority[
                'github_reviewer_requests_authorized'
            ],
            'github_reviews_authorized': authority[
                'github_reviews_authorized'
            ],
            'mark_ready_authorized': authority['mark_ready_authorized'],
            'merge_authorized': authority['merge_authorized'],
            'remote_mutations_performed': authority[
                'remote_mutations_performed'
            ],
        },
    }
    _validate(report, REPORT_SCHEMA_PATH, 'review routing report')
    return report


def render_card(
    report: dict[str, Any],
    *,
    lane_id: str | None = None,
) -> str:
    """Render capability lanes without naming or requesting reviewers."""
    selected = [
        lane for lane in report['lanes']
        if lane_id is None or lane['id'] == lane_id
    ]
    if not selected:
        raise ReviewRoutingError(f'unknown review lane: {lane_id}')
    lines = [
        '# Product Draft review routing',
        '',
        f"- Status: **{report['status']}**",
        f"- Exact head: `{report['exact_head']}`",
        (
            '- Worktree clean: '
            f"{'yes' if report['worktree_clean'] else 'no'}"
        ),
        (
            '- Advisory reviewer target: '
            f"{report['advisory_reviewer_target']} "
            '(target only; not a merge gate)'
        ),
        (
            '- Maximum active lanes: '
            f"{report['max_parallel_active_lanes']}"
        ),
        f"- Final decision role: `{report['final_decision_role']}`",
        '- Reviewer identities collected: none',
        '- GitHub reviewer requests authorized: no',
        '',
        '| Lane | Scope | Paths | Checks | Depends on | Capability |',
        '| --- | --- | ---: | ---: | --- | --- |',
    ]
    for lane in selected:
        dependencies = ', '.join(lane['depends_on_lanes']) or 'none'
        scope = ', '.join(lane['slice_ids'])
        lines.append(
            f"| `{lane['id']}` | {scope} | {lane['path_count']} | "
            f"{lane['verification_count']} | {dependencies} | "
            f"{lane['capability']} |"
        )
    for lane in selected:
        lines.extend(['', f"## {lane['id']} — {lane['title']}"])
        lines.append(f"Capability: {lane['capability']}")
        lines.append('Local review commands (not executed):')
        lines.extend(f'- `{command}`' for command in lane['slice_commands'])
    first = selected[0]
    next_action = (
        f"Review `{first['id']}` locally or identify a consenting reviewer "
        'with the displayed capability.'
        if report['worktree_clean']
        else (
            'Inspect `git status --short`; do not use this dirty routing '
            'packet for a public review request.'
        )
    )
    lines.extend([
        '',
        'Next action:',
        next_action,
        (
            'Boundary: local routing only; no identity collection, reviewer '
            'request, submitted review, mark-ready, merge, or remote write.'
        ),
    ])
    return '\n'.join(lines)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Validate role-based, no-write routing for product Draft review.'
        )
    )
    parser.add_argument('--contract', type=Path, default=CONTRACT_PATH)
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--lane', choices=EXPECTED_LANE_IDS)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        contract = _load_json(args.contract, 'review routing contract')
        overview = collect_publication_overview()
        report = build_report(contract, overview)
        if args.json:
            print(json.dumps(report, indent=2, sort_keys=True))
        else:
            print(render_card(report, lane_id=args.lane))
    except ReviewRoutingError as exc:
        print(f'product Draft review routing failed: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
