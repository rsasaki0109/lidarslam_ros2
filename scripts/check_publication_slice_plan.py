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
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
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

"""Fail closed when a local PR follow-up is not completely review-sliced."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shlex
import subprocess
import sys
from typing import Any, Sequence

import jsonschema


sys.dont_write_bytecode = True

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLAN = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'growth'
    / 'g0-publication-slice-plan-2026-08-12.json'
)
DEFAULT_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'publication-slice-plan-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/publication-slice-plan-v1.schema.json'
)
ROS_SOURCE_PREFIX = (
    'source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" && '
)
ROS_REQUIRED_VERIFICATION_MARKERS = (
    'test_sensor_setup_wizard.py',
    'test_lidarslam_product_cli.py',
    'colcon test --packages-select',
)
PACKAGE_TEST_ROOTS = (
    'lidarslam/test/',
    'graph_based_slam/test/',
)
REMOTE_WRITE_VERIFICATION_FRAGMENTS = (
    'git push',
    'docker push',
    'oras push',
    'gh pr create',
    'gh pr comment',
    'gh pr edit',
    'gh pr merge',
    'gh pr ready',
    'gh pr review',
    'gh issue create',
    'gh issue comment',
    'gh issue edit',
    'gh release create',
    'gh workflow run',
    'gh api -x post',
    'gh api -x put',
    'gh api -x patch',
    'gh api -x delete',
    'gh api --method post',
    'gh api --method put',
    'gh api --method patch',
    'gh api --method delete',
    'curl -x post',
    'curl -x put',
    'curl -x patch',
    'curl -x delete',
)


class PlanError(ValueError):
    """The publication plan cannot be trusted."""


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise PlanError(f'cannot read {label} {path}: {exc}') from exc
    if not isinstance(payload, dict):
        raise PlanError(f'{label} must be a JSON object')
    return payload


def _schema_error_path(error: jsonschema.ValidationError) -> str:
    path = '.'.join(str(item) for item in error.absolute_path)
    return path or '<root>'


def _run_git(arguments: Sequence[str]) -> list[str]:
    command = ['git', *arguments]
    try:
        result = subprocess.run(
            command,
            cwd=REPO_ROOT,
            check=False,
            capture_output=True,
            text=True,
            timeout=60,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise PlanError(
            f'cannot execute read-only Git inspection: {exc}'
        ) from exc
    if result.returncode != 0:
        detail = result.stderr.strip() or 'git returned no error text'
        raise PlanError(f'read-only Git inspection failed: {detail}')
    return [line for line in result.stdout.splitlines() if line]


def candidate_paths(base_sha: str) -> list[str]:
    """Return every tracked or untracked path in the follow-up candidate."""
    tracked = _run_git([
        'diff',
        '--name-only',
        '--diff-filter=ACDMRTUXB',
        base_sha,
        '--',
    ])
    untracked = _run_git(['ls-files', '--others', '--exclude-standard'])
    return sorted(set(tracked + untracked))


def review_paths(
    base_sha: str,
    candidate_ref: str,
    diff_mode: str,
) -> list[str]:
    """Return paths from either an active worktree or a frozen commit range."""
    if candidate_ref == 'HEAD' and diff_mode == 'worktree':
        return candidate_paths(base_sha)
    if diff_mode != 'two-dot':
        raise PlanError(
            'a frozen review endpoint requires two-dot diff mode'
        )
    return committed_paths(base_sha, candidate_ref, diff_mode)


def path_inventory_sha256(paths: Sequence[str]) -> str:
    """Hash a canonical newline-terminated path inventory."""
    payload = ''.join(f'{path}\n' for path in paths).encode('utf-8')
    return hashlib.sha256(payload).hexdigest()


def committed_paths(
    start_sha: str,
    end_sha: str,
    diff_mode: str,
) -> list[str]:
    """Return the canonical path inventory for one committed review phase."""
    separators = {
        'three-dot': '...',
        'two-dot': '..',
    }
    try:
        separator = separators[diff_mode]
    except KeyError as exc:
        raise PlanError(
            f'unsupported committed diff mode: {diff_mode}'
        ) from exc
    paths = _run_git([
        'diff',
        '--name-only',
        '--diff-filter=ACDMRTUXB',
        f'{start_sha}{separator}{end_sha}',
        '--',
    ])
    return sorted(set(paths))


def _git_diff_numstat(
    start_sha: str,
    end_sha: str,
    diff_mode: str,
) -> dict[str, tuple[int | None, int | None]]:
    """Return exact per-path line counts for one read-only Git range."""
    separators = {
        'three-dot': '...',
        'two-dot': '..',
    }
    if diff_mode == 'worktree':
        revision = start_sha
    else:
        try:
            revision = f'{start_sha}{separators[diff_mode]}{end_sha}'
        except KeyError as exc:
            raise PlanError(
                f'unsupported numstat diff mode: {diff_mode}'
            ) from exc

    rows: dict[str, tuple[int | None, int | None]] = {}
    for line in _run_git(['diff', '--numstat', revision, '--']):
        parts = line.split('\t', 2)
        if len(parts) != 3:
            raise PlanError(f'malformed Git numstat row: {line!r}')
        additions_raw, deletions_raw, path = parts
        _validate_repo_path(path)
        if path in rows:
            raise PlanError(f'duplicate Git numstat path: {path}')
        if additions_raw == '-' and deletions_raw == '-':
            rows[path] = (None, None)
        elif additions_raw.isdigit() and deletions_raw.isdigit():
            rows[path] = (int(additions_raw), int(deletions_raw))
        else:
            raise PlanError(f'malformed Git numstat counts: {line!r}')
    return rows


def _validate_numstat_inventory(
    rows: dict[str, tuple[int | None, int | None]],
    *,
    expected_path_count: int,
    expected_paths_sha256: str,
    label: str,
) -> None:
    """Bind derived review effort to the same exact path inventory."""
    paths = sorted(rows)
    if len(paths) != expected_path_count:
        raise PlanError(f'{label} numstat path count is stale')
    if path_inventory_sha256(paths) != expected_paths_sha256:
        raise PlanError(f'{label} numstat path inventory is stale')


def _build_review_budget(
    paths: Sequence[str],
    rows: dict[str, tuple[int | None, int | None]],
    *,
    hotspot_limit: int = 3,
) -> dict[str, Any]:
    """Summarize bounded review effort and the largest textual deltas."""
    canonical_paths = sorted(set(paths))
    if len(canonical_paths) != len(paths):
        raise PlanError('review budget paths must be unique')
    unresolved_paths = [path for path in canonical_paths if path not in rows]
    binary_paths = [
        path
        for path in canonical_paths
        if path in rows and rows[path] == (None, None)
    ]
    text_rows = [
        (path, rows[path][0], rows[path][1])
        for path in canonical_paths
        if path in rows and rows[path] != (None, None)
    ]
    additions = sum(int(item[1]) for item in text_rows)
    deletions = sum(int(item[2]) for item in text_rows)
    ranked = sorted(
        text_rows,
        key=lambda item: (
            -(int(item[1]) + int(item[2])),
            -int(item[1]),
            -int(item[2]),
            item[0],
        ),
    )
    hotspots = [
        {
            'path': path,
            'additions': int(path_additions),
            'deletions': int(path_deletions),
            'changed_lines': int(path_additions) + int(path_deletions),
        }
        for path, path_additions, path_deletions in ranked[:hotspot_limit]
    ]
    return {
        'path_count': len(canonical_paths),
        'text_path_count': len(text_rows),
        'binary_path_count': len(binary_paths),
        'binary_paths': binary_paths,
        'unresolved_path_count': len(unresolved_paths),
        'unresolved_paths': unresolved_paths,
        'additions': additions,
        'deletions': deletions,
        'changed_lines': additions + deletions,
        'hotspots': hotspots,
    }


def _validate_repo_path(path: str) -> None:
    candidate = PurePosixPath(path)
    if not path or path.startswith('/') or '\\' in path:
        raise PlanError(f'invalid repository-relative path: {path!r}')
    if '\n' in path or '\r' in path or '..' in candidate.parts:
        raise PlanError(f'unsafe repository-relative path: {path!r}')
    if str(candidate) != path or path.startswith('./'):
        raise PlanError(f'non-canonical repository-relative path: {path!r}')


def _validate_verification_command(slice_id: str, command: str) -> None:
    """Require copy-ready, bounded review commands without remote writes."""
    if '\n' in command or '\r' in command or len(command) > 2000:
        raise PlanError(
            f'{slice_id} contains an unsafe verification command'
        )
    normalized = command.lower()
    if any(
        fragment in normalized
        for fragment in REMOTE_WRITE_VERIFICATION_FRAGMENTS
    ):
        raise PlanError(
            f'{slice_id} verification cannot perform a remote write'
        )
    if 'pytest' in command and '-p no:cacheprovider' not in command:
        raise PlanError(
            f'{slice_id} pytest verification must disable its cache'
        )
    package_roots = [
        root for root in PACKAGE_TEST_ROOTS if root in command
    ]
    if len(package_roots) > 1:
        raise PlanError(
            f'{slice_id} must run package test roots in separate processes'
        )
    if (
        any(marker in command for marker in ROS_REQUIRED_VERIFICATION_MARKERS)
        and not command.startswith(ROS_SOURCE_PREFIX)
    ):
        raise PlanError(
            f'{slice_id} ROS-dependent verification must source ROS first'
        )
    if 'colcon test --packages-select' in command:
        try:
            tokens = shlex.split(command)
        except ValueError as exc:
            raise PlanError(
                f'{slice_id} verification command has invalid shell quoting'
            ) from exc
        test_index = next(
            (
                index for index in range(len(tokens) - 1)
                if tokens[index:index + 2] == ['colcon', 'test']
            ),
            None,
        )
        build_index = next(
            (
                index for index in range(len(tokens) - 1)
                if tokens[index:index + 2] == ['colcon', 'build']
            ),
            None,
        )

        def option_values(option: str) -> set[str]:
            try:
                index = tokens.index(option)
            except ValueError:
                return set()
            values = []
            for token in tokens[index + 1:]:
                if token == '&&' or token.startswith('-'):
                    break
                values.append(token)
            return set(values)

        tested_packages = option_values('--packages-select')
        built_packages = option_values('--packages-up-to')
        if (
            test_index is None
            or build_index is None
            or build_index >= test_index
            or not tested_packages
            or not tested_packages <= built_packages
        ):
            raise PlanError(
                f'{slice_id} colcon verification must build all tested '
                'packages first with --packages-up-to'
            )


def _validate_candidate_lineage(
    base_sha: str,
    public_baseline_sha: str,
    candidate_ref: str,
) -> tuple[str, int]:
    """Require the frozen public baseline between the PR base and candidate."""
    try:
        base_to_public = _run_git([
            'merge-base', base_sha, public_baseline_sha,
        ])
        public_to_local = _run_git([
            'merge-base', public_baseline_sha, candidate_ref,
        ])
        local_tip = _run_git(['rev-parse', candidate_ref])
        follow_up_count = _run_git([
            'rev-list', '--count',
            f'{public_baseline_sha}..{candidate_ref}',
        ])
    except PlanError as exc:
        raise PlanError('candidate lineage cannot be verified') from exc
    if base_to_public != [base_sha]:
        raise PlanError(
            'public review baseline does not descend from the PR base'
        )
    if public_to_local != [public_baseline_sha]:
        raise PlanError(
            'local candidate does not descend from public review baseline'
        )
    if len(local_tip) != 1 or len(local_tip[0]) != 40:
        raise PlanError('local candidate tip could not be resolved exactly')
    if len(follow_up_count) != 1 or not follow_up_count[0].isdigit():
        raise PlanError('follow-up commit count could not be resolved')
    return local_tip[0], int(follow_up_count[0])


def _validate_review_record(label: str, path: str) -> None:
    """Require a safe, current, regular-file record for each review phase."""
    _validate_repo_path(path)
    record = REPO_ROOT / path
    if record.is_symlink() or not record.is_file():
        raise PlanError(f'{label} review_record is not a regular file: {path}')


def _validate_fixed_review_phase(
    label: str,
    phase: dict[str, Any],
) -> list[str]:
    """Validate one immutable committed phase and return its exact paths."""
    start_sha = phase['start_sha']
    end_sha = phase['end_sha']
    try:
        merge_base = _run_git(['merge-base', start_sha, end_sha])
    except PlanError as exc:
        raise PlanError(f'{label} lineage cannot be verified') from exc
    if merge_base != [start_sha]:
        raise PlanError(f'{label} end does not descend from its start')

    commit_count = _run_git([
        'rev-list', '--count', f'{start_sha}..{end_sha}',
    ])
    if commit_count != [str(phase['expected_commit_count'])]:
        raise PlanError(f'{label} expected_commit_count is stale')

    paths = committed_paths(start_sha, end_sha, phase['diff_mode'])
    if len(paths) != phase['expected_path_count']:
        raise PlanError(f'{label} expected_path_count is stale')
    if path_inventory_sha256(paths) != phase['expected_paths_sha256']:
        raise PlanError(f'{label} expected_paths_sha256 is stale')
    try:
        _run_git(['diff', '--check', f'{start_sha}..{end_sha}', '--'])
    except PlanError as exc:
        raise PlanError(f'{label} exact range fails git diff --check') from exc
    _validate_review_record(label, phase['review_record'])
    return paths


def _validate_whole_pr_review(
    plan: dict[str, Any],
    follow_up_paths: Sequence[str],
) -> dict[str, Any]:
    """Prove that sequential review phases cover the final whole-PR diff."""
    contract = plan['whole_pr_review']
    initial = contract['initial_review']
    bridge = contract['bridge_review']
    follow_up = contract['follow_up_review']
    candidate_ref = follow_up['end_ref']
    candidate = plan['candidate']

    if initial['start_sha'] != contract['base_sha']:
        raise PlanError('initial review does not start at the whole-PR base')
    if initial['end_sha'] != bridge['start_sha']:
        raise PlanError('initial and bridge review ranges are not contiguous')
    if bridge['end_sha'] != follow_up['start_sha']:
        raise PlanError(
            'bridge and follow-up review ranges are not contiguous'
        )
    if follow_up['start_sha'] != candidate['base_sha']:
        raise PlanError(
            'follow-up review does not start at the slice-plan base'
        )

    initial_paths = _validate_fixed_review_phase('initial review', initial)
    bridge_paths = _validate_fixed_review_phase('bridge review', bridge)

    allowed_bridge_paths = bridge['allowed_paths']
    if allowed_bridge_paths != sorted(allowed_bridge_paths):
        raise PlanError('bridge review allowed_paths must be sorted')
    for path in allowed_bridge_paths:
        _validate_repo_path(path)
    if bridge_paths != allowed_bridge_paths:
        missing = sorted(set(bridge_paths) - set(allowed_bridge_paths))
        stale = sorted(set(allowed_bridge_paths) - set(bridge_paths))
        raise PlanError(
            'bridge review allowlist mismatch: '
            f'missing_from_allowlist={missing}, absent_from_bridge={stale}'
        )

    canonical_follow_up = sorted(set(follow_up_paths))
    _validate_review_record(
        'follow-up review', follow_up['review_record']
    )
    if initial['review_record'] not in initial_paths:
        raise PlanError(
            'initial review record is outside the initial review inventory'
        )
    for label, record in (
        ('bridge review', bridge['review_record']),
        ('follow-up review', follow_up['review_record']),
    ):
        if record not in canonical_follow_up:
            raise PlanError(
                f'{label} record is outside the follow-up review inventory'
            )

    whole_paths = review_paths(
        contract['base_sha'], candidate_ref, follow_up['diff_mode']
    )
    whole_digest = path_inventory_sha256(whole_paths)
    if contract['expected_path_count'] != len(whole_paths):
        raise PlanError('whole-PR expected_path_count is stale')
    if contract['expected_paths_sha256'] != whole_digest:
        raise PlanError('whole-PR expected_paths_sha256 is stale')

    phase_sets = [
        set(initial_paths),
        set(bridge_paths),
        set(canonical_follow_up),
    ]
    covered_paths = set().union(*phase_sets)
    whole_set = set(whole_paths)
    uncovered = sorted(whole_set - covered_paths)
    extraneous = sorted(covered_paths - whole_set)
    if uncovered or extraneous:
        raise PlanError(
            'whole-PR review coverage mismatch: '
            f'uncovered={uncovered}, extraneous={extraneous}'
        )

    merge_commits = _run_git([
        'rev-list',
        '--min-parents=2',
        f"{contract['base_sha']}..{candidate_ref}",
    ])
    if merge_commits:
        raise PlanError(
            f'whole-PR history contains merge commits: {merge_commits}'
        )

    follow_up_commit_count_raw = _run_git([
        'rev-list', '--count',
        f"{follow_up['start_sha']}..{candidate_ref}",
    ])
    whole_pr_commit_count_raw = _run_git([
        'rev-list', '--count', f"{contract['base_sha']}..{candidate_ref}",
    ])
    if (
        len(follow_up_commit_count_raw) != 1
        or not follow_up_commit_count_raw[0].isdigit()
        or len(whole_pr_commit_count_raw) != 1
        or not whole_pr_commit_count_raw[0].isdigit()
    ):
        raise PlanError('whole-PR review commit counts could not be resolved')
    follow_up_review_commit_count = int(follow_up_commit_count_raw[0])
    whole_pr_commit_count = int(whole_pr_commit_count_raw[0])
    composed_commit_count = (
        initial['expected_commit_count']
        + bridge['expected_commit_count']
        + follow_up_review_commit_count
    )
    if whole_pr_commit_count != composed_commit_count:
        raise PlanError('whole-PR review commit ranges do not compose exactly')

    memberships: dict[str, int] = {}
    for phase_paths in phase_sets:
        for path in phase_paths:
            memberships[path] = memberships.get(path, 0) + 1
    overlap_path_count = sum(
        count > 1 for count in memberships.values()
    )
    overlap_membership_count = sum(
        count - 1 for count in memberships.values()
    )
    return {
        'whole_pr_base_sha': contract['base_sha'],
        'whole_pr_path_count': len(whole_paths),
        'whole_pr_paths_sha256': whole_digest,
        'review_phase_count': 3,
        'review_phase_ids': [
            'initial_review',
            'bridge_review',
            'follow_up_review',
        ],
        'review_coverage_complete': True,
        'whole_pr_commit_count': whole_pr_commit_count,
        'initial_review_commit_count': initial['expected_commit_count'],
        'initial_review_path_count': len(initial_paths),
        'bridge_review_commit_count': bridge['expected_commit_count'],
        'bridge_path_count': len(bridge_paths),
        'follow_up_review_commit_count': follow_up_review_commit_count,
        'overlap_path_count': overlap_path_count,
        'overlap_membership_count': overlap_membership_count,
        'uncovered_path_count': 0,
        'extraneous_phase_path_count': 0,
        'merge_commit_count': 0,
    }


def validate_plan(
    plan: dict[str, Any],
    schema: dict[str, Any],
    actual_paths: Sequence[str],
) -> dict[str, Any]:
    """Validate schema, dependency order, exact coverage, and authority."""
    validator = jsonschema.Draft7Validator(schema)
    errors = sorted(
        validator.iter_errors(plan),
        key=lambda item: [str(part) for part in item.absolute_path],
    )
    if errors:
        first = errors[0]
        raise PlanError(
            f'schema validation failed at {_schema_error_path(first)}: '
            f'{first.message}')
    if plan['schema_uri'] != SCHEMA_URI:
        raise PlanError('plan schema_uri is not the supported v1 URI')

    candidate = plan['candidate']
    follow_up = plan['whole_pr_review']['follow_up_review']
    candidate_ref = follow_up['end_ref']
    local_tip_sha, follow_up_commit_count = _validate_candidate_lineage(
        candidate['base_sha'],
        candidate['public_head_sha'],
        candidate_ref,
    )
    worktree_status = _run_git(['status', '--short'])
    slices = plan['review_slices']
    slice_ids = [item['id'] for item in slices]
    orders = [item['order'] for item in slices]
    if len(slice_ids) != len(set(slice_ids)):
        raise PlanError('review slice ids must not contain duplicates')
    if orders != list(range(1, len(slices) + 1)):
        raise PlanError('review slice orders must be consecutive from one')

    seen_ids: set[str] = set()
    planned_paths: list[str] = []
    for item in slices:
        dependencies = item['depends_on']
        if dependencies != sorted(dependencies):
            raise PlanError(f"{item['id']} dependencies must be sorted")
        unknown = set(dependencies) - seen_ids
        if unknown:
            raise PlanError(
                f"{item['id']} depends on unknown or later slices: "
                f'{sorted(unknown)}')
        paths = item['paths']
        if paths != sorted(paths):
            raise PlanError(f"{item['id']} paths must be sorted")
        if len(paths) != len(set(paths)):
            raise PlanError(f"{item['id']} contains duplicate paths")
        for path in paths:
            _validate_repo_path(path)
        for command in item['verification']:
            _validate_verification_command(item['id'], command)
        planned_paths.extend(paths)
        seen_ids.add(item['id'])

    duplicates = sorted({
        path for path in planned_paths if planned_paths.count(path) > 1
    })
    if duplicates:
        raise PlanError(f'paths assigned to multiple slices: {duplicates}')

    canonical_planned = sorted(planned_paths)
    canonical_actual = sorted(set(actual_paths))
    missing = sorted(set(canonical_actual) - set(canonical_planned))
    stale = sorted(set(canonical_planned) - set(canonical_actual))
    if missing or stale:
        raise PlanError(
            'candidate path coverage mismatch: '
            f'missing_from_plan={missing}, absent_from_candidate={stale}')

    actual_digest = path_inventory_sha256(canonical_actual)
    if candidate['expected_path_count'] != len(canonical_actual):
        raise PlanError('candidate expected_path_count is stale')
    if candidate['expected_paths_sha256'] != actual_digest:
        raise PlanError('candidate expected_paths_sha256 is stale')

    whole_pr_report = _validate_whole_pr_review(plan, canonical_actual)

    authority = plan['authority']
    if authority['github_writes_authorized']:
        raise PlanError('a local review plan cannot authorize GitHub writes')
    if authority['remote_mutations_performed']:
        raise PlanError('a local review plan cannot claim remote mutations')

    return {
        'status': 'PLAN_VALID_LOCAL_ONLY',
        'base_sha': candidate['base_sha'],
        'public_baseline_sha': candidate['public_head_sha'],
        'local_tip_sha': local_tip_sha,
        'follow_up_commit_count': follow_up_commit_count,
        'worktree_clean': not worktree_status,
        'uncommitted_path_count': len(worktree_status),
        'scope': candidate['scope'],
        'pull_request': candidate['pull_request'],
        'path_count': len(canonical_actual),
        'paths_sha256': actual_digest,
        'slice_count': len(slices),
        'slice_ids': slice_ids,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
        **whole_pr_report,
    }


def build_slice_review_report(
    plan: dict[str, Any],
    validation_report: dict[str, Any],
    slice_id: str,
) -> dict[str, Any]:
    """Return one exact, read-only review focus from a validated plan."""
    review_slice = next(
        (item for item in plan['review_slices'] if item['id'] == slice_id),
        None,
    )
    if review_slice is None:
        available = ', '.join(item['id'] for item in plan['review_slices'])
        raise PlanError(
            f'unknown review slice {slice_id!r}; '
            f'available slices: {available}'
        )

    follow_up_rows = _git_diff_numstat(
        validation_report['base_sha'],
        validation_report['local_tip_sha'],
        plan['whole_pr_review']['follow_up_review']['diff_mode'],
    )
    _validate_numstat_inventory(
        follow_up_rows,
        expected_path_count=validation_report['path_count'],
        expected_paths_sha256=validation_report['paths_sha256'],
        label='follow-up review',
    )

    return {
        'status': 'SLICE_REVIEW_READY_LOCAL_ONLY',
        'candidate': {
            'base_sha': validation_report['base_sha'],
            'public_baseline_sha': validation_report['public_baseline_sha'],
            'local_tip_sha': validation_report['local_tip_sha'],
            'follow_up_commit_count': validation_report[
                'follow_up_commit_count'
            ],
            'pull_request': validation_report['pull_request'],
            'slice_count': validation_report['slice_count'],
            'worktree_clean': validation_report['worktree_clean'],
            'uncommitted_path_count': validation_report[
                'uncommitted_path_count'
            ],
            'whole_pr_base_sha': validation_report['whole_pr_base_sha'],
            'whole_pr_path_count': validation_report['whole_pr_path_count'],
            'review_phase_count': validation_report['review_phase_count'],
            'review_coverage_complete': validation_report[
                'review_coverage_complete'
            ],
            'bridge_path_count': validation_report['bridge_path_count'],
        },
        'review_slice': {
            'id': review_slice['id'],
            'order': review_slice['order'],
            'title': review_slice['title'],
            'review_outcome': review_slice['review_outcome'],
            'depends_on': list(review_slice['depends_on']),
            'path_count': len(review_slice['paths']),
            'paths': list(review_slice['paths']),
            'verification': list(review_slice['verification']),
            'publication_gate': review_slice['publication_gate'],
            'review_budget': _build_review_budget(
                review_slice['paths'],
                follow_up_rows,
            ),
        },
        'commands_executed': False,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }


def build_pr_review_overview_report(
    plan: dict[str, Any],
    validation_report: dict[str, Any],
) -> dict[str, Any]:
    """Return one bounded overview of the complete validated PR review."""
    contract = plan['whole_pr_review']
    initial = contract['initial_review']
    bridge = contract['bridge_review']
    follow_up = contract['follow_up_review']
    candidate = plan['candidate']
    initial_rows = _git_diff_numstat(
        initial['start_sha'],
        initial['end_sha'],
        initial['diff_mode'],
    )
    _validate_numstat_inventory(
        initial_rows,
        expected_path_count=initial['expected_path_count'],
        expected_paths_sha256=initial['expected_paths_sha256'],
        label='initial review',
    )
    bridge_rows = _git_diff_numstat(
        bridge['start_sha'],
        bridge['end_sha'],
        bridge['diff_mode'],
    )
    _validate_numstat_inventory(
        bridge_rows,
        expected_path_count=bridge['expected_path_count'],
        expected_paths_sha256=bridge['expected_paths_sha256'],
        label='bridge review',
    )
    follow_up_rows = _git_diff_numstat(
        follow_up['start_sha'],
        validation_report['local_tip_sha'],
        follow_up['diff_mode'],
    )
    _validate_numstat_inventory(
        follow_up_rows,
        expected_path_count=validation_report['path_count'],
        expected_paths_sha256=validation_report['paths_sha256'],
        label='follow-up review',
    )
    whole_pr_rows = _git_diff_numstat(
        contract['base_sha'],
        validation_report['local_tip_sha'],
        follow_up['diff_mode'],
    )
    _validate_numstat_inventory(
        whole_pr_rows,
        expected_path_count=validation_report['whole_pr_path_count'],
        expected_paths_sha256=validation_report['whole_pr_paths_sha256'],
        label='whole-PR review',
    )
    phase_budgets = {
        'P0-initial-review': _build_review_budget(
            sorted(initial_rows), initial_rows,
        ),
        'P1-ci-bridge': _build_review_budget(
            sorted(bridge_rows), bridge_rows,
        ),
        'P2-follow-up-slices': _build_review_budget(
            sorted(follow_up_rows), follow_up_rows,
        ),
    }
    return {
        'status': 'PR_REVIEW_OVERVIEW_READY_LOCAL_ONLY',
        'candidate': {
            'repository': candidate['repository'],
            'pull_request': validation_report['pull_request'],
            'whole_pr_base_sha': validation_report['whole_pr_base_sha'],
            'local_tip_sha': validation_report['local_tip_sha'],
            'public_baseline_sha': validation_report['public_baseline_sha'],
            'whole_pr_commit_count': validation_report[
                'whole_pr_commit_count'
            ],
            'whole_pr_path_count': validation_report['whole_pr_path_count'],
            'whole_pr_paths_sha256': validation_report[
                'whole_pr_paths_sha256'
            ],
            'follow_up_path_count': validation_report['path_count'],
            'review_phase_count': validation_report['review_phase_count'],
            'review_coverage_complete': validation_report[
                'review_coverage_complete'
            ],
            'overlap_path_count': validation_report['overlap_path_count'],
            'uncovered_path_count': validation_report[
                'uncovered_path_count'
            ],
            'extraneous_phase_path_count': validation_report[
                'extraneous_phase_path_count'
            ],
            'merge_commit_count': validation_report['merge_commit_count'],
            'slice_count': validation_report['slice_count'],
            'worktree_clean': validation_report['worktree_clean'],
            'uncommitted_path_count': validation_report[
                'uncommitted_path_count'
            ],
            'whole_pr_review_budget': _build_review_budget(
                sorted(whole_pr_rows), whole_pr_rows,
            ),
        },
        'review_phases': [
            {
                'id': 'P0-initial-review',
                'order': 1,
                'start_sha': initial['start_sha'],
                'end_sha': initial['end_sha'],
                'diff_mode': initial['diff_mode'],
                'commit_count': validation_report[
                    'initial_review_commit_count'
                ],
                'path_count': validation_report[
                    'initial_review_path_count'
                ],
                'review_record': initial['review_record'],
                'review_budget': phase_budgets['P0-initial-review'],
            },
            {
                'id': 'P1-ci-bridge',
                'order': 2,
                'start_sha': bridge['start_sha'],
                'end_sha': bridge['end_sha'],
                'diff_mode': bridge['diff_mode'],
                'commit_count': validation_report[
                    'bridge_review_commit_count'
                ],
                'path_count': validation_report['bridge_path_count'],
                'review_record': bridge['review_record'],
                'review_budget': phase_budgets['P1-ci-bridge'],
            },
            {
                'id': 'P2-follow-up-slices',
                'order': 3,
                'start_sha': follow_up['start_sha'],
                'end_sha': validation_report['local_tip_sha'],
                'diff_mode': follow_up['diff_mode'],
                'commit_count': validation_report[
                    'follow_up_review_commit_count'
                ],
                'path_count': validation_report['path_count'],
                'review_record': follow_up['review_record'],
                'review_budget': phase_budgets['P2-follow-up-slices'],
            },
        ],
        'review_slices': [
            {
                'id': item['id'],
                'order': item['order'],
                'title': item['title'],
                'depends_on': list(item['depends_on']),
                'path_count': len(item['paths']),
                'verification_count': len(item['verification']),
                'publication_gate': item['publication_gate'],
                'review_budget': _build_review_budget(
                    item['paths'], follow_up_rows,
                ),
            }
            for item in plan['review_slices']
        ],
        'slice_command_template': (
            'python3 scripts/check_publication_slice_plan.py --slice <ID>'
        ),
        'commands_executed': False,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }


def _format_review_delta(budget: dict[str, Any]) -> str:
    return f"+{budget['additions']:,}/-{budget['deletions']:,}"


def _format_largest_hotspot(budget: dict[str, Any]) -> str:
    hotspots = budget['hotspots']
    if not hotspots:
        return 'none'
    hotspot = hotspots[0]
    return (
        f"`{hotspot['path']}` "
        f"(+{hotspot['additions']:,}/-{hotspot['deletions']:,})"
    )


def render_slice_review_card(report: dict[str, Any]) -> str:
    """Render one copy-ready, human-facing publication review card."""
    candidate = report['candidate']
    review_slice = report['review_slice']
    budget = review_slice['review_budget']
    dependencies = review_slice['depends_on']
    lines = [
        f"# {review_slice['id']}: {review_slice['title']}",
        '',
        (
            f"- Review order: {review_slice['order']} of "
            f"{candidate['slice_count']}"
        ),
        f"- Paths: {review_slice['path_count']}",
        f'- Text delta: {_format_review_delta(budget)}',
        f"- Binary paths: {budget['binary_path_count']}",
        f"- Unresolved numstat paths: {budget['unresolved_path_count']}",
        f"- Depends on: {', '.join(dependencies) if dependencies else 'none'}",
        f"- Publication gate: {review_slice['publication_gate']}",
        f"- Frozen public review baseline: {candidate['public_baseline_sha']}",
        f"- Local HEAD: {candidate['local_tip_sha']}",
        f"- Whole-PR base: {candidate['whole_pr_base_sha']}",
        f"- Whole-PR paths: {candidate['whole_pr_path_count']}",
        f"- Sequential review phases: {candidate['review_phase_count']}",
        (
            '- Whole-PR review coverage complete: '
            f"{'yes' if candidate['review_coverage_complete'] else 'no'}"
        ),
        f"- CI bridge paths: {candidate['bridge_path_count']}",
        (
            '- Follow-up commits after baseline: '
            f"{candidate['follow_up_commit_count']}"
        ),
        f"- Worktree clean: {'yes' if candidate['worktree_clean'] else 'no'}",
        (
            '- Uncommitted paths: '
            f"{candidate['uncommitted_path_count']}"
        ),
        '- Commands executed by this card: no',
        '- GitHub write authorized: no',
        '',
        'Review outcome:',
        review_slice['review_outcome'],
        '',
        'Review hotspots (largest textual deltas):',
        *(
            f"{index}. `{hotspot['path']}` "
            f"(+{hotspot['additions']:,}/-{hotspot['deletions']:,})"
            for index, hotspot in enumerate(budget['hotspots'], start=1)
        ),
        *(
            (
                '',
                'Binary paths requiring manifest/content review:',
                *(f'- `{path}`' for path in budget['binary_paths']),
            )
            if budget['binary_paths']
            else ()
        ),
        '',
        'Paths:',
        *(f'- {path}' for path in review_slice['paths']),
        '',
        'Verification commands:',
    ]
    for command in review_slice['verification']:
        lines.extend(['', '```bash', command, '```'])
    lines.extend([
        '',
        (
            'Next action: Start with the bounded hotspots, review the '
            'remaining paths in order, then run the listed commands and '
            'record their results without treating this card as publication '
            'approval.'
        ),
    ])
    return '\n'.join(lines)


def render_pr_review_overview_card(report: dict[str, Any]) -> str:
    """Render a compact, copy-ready overview of the complete PR review."""
    candidate = report['candidate']
    whole_budget = candidate['whole_pr_review_budget']
    lines = [
        f"# PR #{candidate['pull_request']} review overview",
        '',
        f"- Repository: {candidate['repository']}",
        f"- Whole-PR base: {candidate['whole_pr_base_sha']}",
        f"- Exact local tip: {candidate['local_tip_sha']}",
        f"- Frozen public review baseline: {candidate['public_baseline_sha']}",
        f"- Commits: {candidate['whole_pr_commit_count']}",
        f"- Whole-PR paths: {candidate['whole_pr_path_count']}",
        f'- Whole-PR text delta: {_format_review_delta(whole_budget)}',
        f"- Whole-PR binary paths: {whole_budget['binary_path_count']}",
        (
            '- Whole-PR unresolved numstat paths: '
            f"{whole_budget['unresolved_path_count']}"
        ),
        f"- Follow-up paths: {candidate['follow_up_path_count']}",
        f"- Sequential review phases: {candidate['review_phase_count']}",
        f"- Review slices: {candidate['slice_count']}",
        (
            '- Whole-PR review coverage complete: '
            f"{'yes' if candidate['review_coverage_complete'] else 'no'}"
        ),
        f"- Overlapping paths across phases: {candidate['overlap_path_count']}",
        f"- Uncovered paths: {candidate['uncovered_path_count']}",
        (
            '- Extraneous phase paths: '
            f"{candidate['extraneous_phase_path_count']}"
        ),
        f"- Merge commits: {candidate['merge_commit_count']}",
        f"- Worktree clean: {'yes' if candidate['worktree_clean'] else 'no'}",
        f"- Uncommitted paths: {candidate['uncommitted_path_count']}",
        '- Commands executed by this card: no',
        '- GitHub write authorized: no',
        '',
        '## Sequential coverage',
        '',
        (
            '| Phase | Range | Mode | Commits | Paths | Text delta | Binary '
            '| Largest text path | Review record |'
        ),
        (
            '| --- | --- | --- | ---: | ---: | ---: | ---: | --- | --- |'
        ),
    ]
    for phase in report['review_phases']:
        budget = phase['review_budget']
        range_text = (
            f"`{phase['start_sha'][:7]}..{phase['end_sha'][:7]}`"
        )
        lines.append(
            f"| {phase['id']} | {range_text} | {phase['diff_mode']} | "
            f"{phase['commit_count']} | {phase['path_count']} | "
            f'{_format_review_delta(budget)} | '
            f"{budget['binary_path_count']} | "
            f'{_format_largest_hotspot(budget)} | '
            f"[{phase['review_record']}]({phase['review_record']}) |"
        )
    lines.extend([
        '',
        '## Review slices',
        '',
        (
            '| Order | Slice | Focus | Paths | Text delta | Binary | Largest '
            'text path | Checks | Gate | Depends on |'
        ),
        (
            '| ---: | --- | --- | ---: | ---: | ---: | --- | ---: | --- '
            '| --- |'
        ),
    ])
    for review_slice in report['review_slices']:
        dependencies = ', '.join(review_slice['depends_on']) or 'none'
        budget = review_slice['review_budget']
        lines.append(
            f"| {review_slice['order']} | {review_slice['id']} | "
            f"{review_slice['title']} | {review_slice['path_count']} | "
            f'{_format_review_delta(budget)} | '
            f"{budget['binary_path_count']} | "
            f'{_format_largest_hotspot(budget)} | '
            f"{review_slice['verification_count']} | "
            f"{review_slice['publication_gate']} | {dependencies} |"
        )
    lines.extend([
        '',
        'Render one exact slice:',
        '',
        '```bash',
        report['slice_command_template'],
        '```',
        '',
        (
            'Next action: Review the three phases and then the seven slices '
            'in order; run each slice card without treating this overview as '
            'push, review-submission, mark-ready, or merge approval.'
        ),
    ])
    return '\n'.join(lines)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--schema', type=Path, default=DEFAULT_SCHEMA)
    output_mode = parser.add_mutually_exclusive_group()
    output_mode.add_argument(
        '--slice',
        metavar='ID',
        help='Render one validated review slice without running its commands.',
    )
    output_mode.add_argument(
        '--overview',
        action='store_true',
        help='Render one bounded overview of every review phase and slice.',
    )
    parser.add_argument('--json', action='store_true')
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        plan = _load_json(args.plan, 'plan')
        schema = _load_json(args.schema, 'schema')
        candidate = plan.get('candidate', {})
        follow_up = plan.get('whole_pr_review', {}).get(
            'follow_up_review', {}
        )
        candidate_ref = follow_up.get('end_ref', '')
        diff_mode = follow_up.get('diff_mode')
        paths = review_paths(
            candidate.get('base_sha', ''), candidate_ref, diff_mode
        )
        report = validate_plan(plan, schema, paths)
        if args.slice:
            report = build_slice_review_report(plan, report, args.slice)
        elif args.overview:
            report = build_pr_review_overview_report(plan, report)
    except PlanError as exc:
        if args.json:
            print(json.dumps({
                'status': 'PLAN_INVALID',
                'error': str(exc),
                'remote_mutations_performed': False,
            }, sort_keys=True))
        else:
            print(f'ERROR: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    elif args.slice:
        print(render_slice_review_card(report))
    elif args.overview:
        print(render_pr_review_overview_card(report))
    else:
        print(
            'PASS: publication slice plan covers '
            f"{report['path_count']} paths in {report['slice_count']} slices")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
