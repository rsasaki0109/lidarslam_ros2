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

"""Render one fail-closed, read-only G0 readiness dashboard.

The dashboard composes the existing local publication-plan, onboarding
matrix, validator-cohort, and v1-readiness checkers. It does not replace any
checker, execute a trial, or write remote state. Product-PR, protected-
environment, and published-release audits are opt-in because they perform
network reads; absence is reported as ``NOT_CHECKED`` rather than a pass.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Callable, Sequence
import urllib.error
import urllib.request

import jsonschema

try:
    from github_api_auth import github_api_authorization
except ModuleNotFoundError:  # pragma: no cover - importlib test path
    from scripts.github_api_auth import github_api_authorization


REPO_ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = (
    REPO_ROOT / 'docs' / 'schemas' / 'g0-readiness-report-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/g0-readiness-report-v1.schema.json'
)
REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
CURRENT_PACKET = 'docs/evidence/growth/g0-current-action-packet-2026-08-14.md'
PRODUCT_PR_NUMBER = 427
PRODUCT_PR_URL = f'https://github.com/{REPOSITORY}/pull/{PRODUCT_PR_NUMBER}'
PRODUCT_GITHUB_URL = f'https://github.com/{REPOSITORY}'
PRODUCT_REPOSITORY_URL = f'https://github.com/{REPOSITORY}.git'
PRODUCT_PR_BASE = 'develop'
PRODUCT_PR_HEAD = 'agent/product-g0-guided-ux'
PRODUCT_PR_VERIFY_COMMAND = (
    'python3 scripts/check_g0_readiness.py --include-product-draft --json'
)
PUBLICATION_REVIEW_OVERVIEW_COMMAND = (
    'python3 scripts/check_publication_slice_plan.py --overview'
)
PUBLICATION_REVIEW_SLICE_TEMPLATE = (
    'python3 scripts/check_publication_slice_plan.py --slice <ID>'
)
MAX_GITHUB_JSON_BYTES = 2 * 1024 * 1024
MAX_CHECK_RUNS = 100
MAX_PRODUCT_DESCRIPTION_BYTES = 64 * 1024
SHA_PATTERN = re.compile(r'^[0-9a-f]{40}$')
DIGEST_PATTERN = re.compile(r'^[0-9a-f]{64}$')
VERSION_PATTERN = re.compile(
    r'^[0-9]+\.[0-9]+\.[0-9]+(?:[-+][A-Za-z0-9.-]+)?$'
)
SAFE_REVIEW_TITLE_PATTERN = re.compile(
    r'^[A-Za-z0-9][A-Za-z0-9 ,&/+\-]{0,159}$'
)
EXPECTED_REVIEW_PHASES = (
    ('P0-initial-review', 'three-dot'),
    ('P1-ci-bridge', 'two-dot'),
    ('P2-follow-up-slices', 'two-dot'),
)
EXPECTED_REVIEW_SLICES = (
    'S1-runtime-safety',
    'S2-first-map-foundation',
    'S3-map-lifecycle',
    'S4-source-onboarding',
    'S5-distribution-readiness',
    'S6-product-shell-integration',
    'S7-publication-control',
)
EXPECTED_REVIEW_LANES = (
    'R1-runtime-safety',
    'R2-operator-ux',
    'R3-distribution',
    'R4-integration-publication',
)
EXPECTED_REVIEW_LANE_DEPENDENCIES = (
    (),
    ('R1-runtime-safety',),
    ('R1-runtime-safety', 'R2-operator-ux'),
    ('R1-runtime-safety', 'R2-operator-ux', 'R3-distribution'),
)
EXPECTED_REVIEW_LANE_SLICES = (
    ('S1-runtime-safety', 'S2-first-map-foundation'),
    ('S3-map-lifecycle', 'S4-source-onboarding'),
    ('S5-distribution-readiness',),
    ('S6-product-shell-integration', 'S7-publication-control'),
)
REVIEW_PUBLICATION_GATES = frozenset({
    'PUBLIC_CI',
    'LOCAL_REVIEW',
    'MAINTAINER_DECISION',
})
REQUIRED_SUCCESS_CHECKS = frozenset({
    'build (humble)',
    'build (jazzy)',
    'candidate gate contract',
    'docs and release metadata',
    'humble default workflow',
    'humble v0.6.0 to candidate',
    'jazzy default workflow',
    'jazzy v0.6.0 to candidate',
    'release readiness',
    'release readiness threshold guard',
})
REQUIRED_SKIPPED_CHECKS = frozenset({
    'authorize immutable candidate request',
    'build and push (${{ matrix.ros_distro }})',
    'publish immutable digest (${{ matrix.ros_distro }})',
    'verify immutable candidate pair',
})
ACCEPTED_EXTRA_CONCLUSIONS = frozenset({'success', 'neutral', 'skipped'})
DEFAULT_RELEASE_VERSION = (
    REPO_ROOT / 'VERSION'
).read_text(encoding='utf-8').strip()
CANDIDATE_ENVIRONMENT_SETTINGS_URL = (
    f'https://github.com/{REPOSITORY}/settings/environments'
)
CANDIDATE_ENVIRONMENT_VERIFY_COMMAND = (
    'python3 scripts/check_candidate_environment.py --json --require-ready'
)
PUBLIC_TRANSITION_AUDITS = (
    'product_draft',
    'candidate_environment',
    'published_release',
)
CANDIDATE_HANDOFF_KINDS = {
    'CREATE_AND_REVIEW_ENVIRONMENT',
    'REPAIR_AND_REVIEW_ENVIRONMENT',
    'RESTORE_READ_ACCESS',
    'REVIEW_E2_SEPARATELY',
}
CANDIDATE_HANDOFF_AUTHORITIES = {
    'repository-settings-admin',
    'read-access',
    'separate-e2-approval',
}
CANDIDATE_HANDOFF_BY_STATUS = {
    'READY': (
        'REVIEW_E2_SEPARATELY',
        'separate-e2-approval',
        False,
        None,
    ),
    'ABSENT': (
        'CREATE_AND_REVIEW_ENVIRONMENT',
        'repository-settings-admin',
        True,
        CANDIDATE_ENVIRONMENT_SETTINGS_URL,
    ),
    'MISCONFIGURED': (
        'REPAIR_AND_REVIEW_ENVIRONMENT',
        'repository-settings-admin',
        True,
        CANDIDATE_ENVIRONMENT_SETTINGS_URL,
    ),
    'BLOCKED': (
        'RESTORE_READ_ACCESS',
        'read-access',
        False,
        None,
    ),
}

COHORT_GATE_GUIDANCE = {
    'public_revision': (
        'name one exact public commit for the selected product version'
    ),
    'public_revision_resolvable': (
        'verify that the exact source commit resolves from the public '
        'repository'
    ),
    'comparable_docker_row': (
        'record one clean Docker PASS at that version with all seven '
        'measurements, including active time, command count, and isolated '
        'peak disk'
    ),
    'comparable_source_row': (
        'record one clean source PASS at that version with all seven '
        'measurements, including active time and command count'
    ),
    'canonical_documentation_path': (
        'select the public Docker First Map or source quickstart route used '
        'by the cohort'
    ),
    'canonical_documentation_url': (
        'bind that route to its canonical documentation URL and route fragment'
    ),
    'canonical_documentation_provenance': (
        'verify the deployed page manifest, exact source revision, and page '
        'SHA-256 with check_public_docs_deployment.py'
    ),
    'canonical_runtime_ref': (
        'bind Docker to an immutable GHCR digest or source to the exact '
        'public commit'
    ),
    'copy_ready_handoff_public': (
        'ensure the public revision contains the copy-ready first-map handoff'
    ),
}


class G0ReadinessError(ValueError):
    """The dashboard cannot safely summarize a checker result."""


Runner = Callable[..., subprocess.CompletedProcess[str]]
GithubFetcher = Callable[[str], tuple[int, dict[str, Any] | None]]
AncestorChecker = Callable[[str, str], bool | None]


def _cohort_gate_guidance(gate: str) -> str:
    """Return bounded human guidance while preserving the machine gate ID."""
    return COHORT_GATE_GUIDANCE.get(
        gate,
        'inspect the first-map cohort contract for the exact missing '
        'prerequisite',
    )


def _checker_command(script: str, *arguments: str) -> list[str]:
    return [sys.executable, str(REPO_ROOT / 'scripts' / script), *arguments]


def _run_json(
    script: str,
    arguments: Sequence[str] = (),
    *,
    runner: Runner = subprocess.run,
) -> dict[str, Any]:
    """Run one existing checker and require a JSON object on stdout.

    The checkers use exit code 1 for an unmet gate, which is still a valid
    report. Exit code 2 or malformed output is an audit error and never gets
    converted into a synthetic HOLD result.
    """
    result = runner(
        _checker_command(script, *arguments),
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode not in (0, 1):
        detail = result.stderr.strip() or 'checker returned no diagnostic'
        raise G0ReadinessError(
            f'{script} failed with exit {result.returncode}: {detail}'
        )
    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise G0ReadinessError(
            f'{script} did not emit valid JSON: {exc}'
        ) from exc
    if not isinstance(payload, dict):
        raise G0ReadinessError(f'{script} JSON root is not an object')
    return payload


def _github_json(path: str) -> tuple[int, dict[str, Any] | None]:
    """Perform one bounded GitHub GET for the optional product-PR audit."""
    url = f'https://api.github.com/{path}'
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'lidarslam-g0-product-pr-audit/1',
        'X-GitHub-Api-Version': '2022-11-28',
    }
    headers.update(github_api_authorization(url, method='GET'))
    request = urllib.request.Request(url, headers=headers, method='GET')
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            status = response.status
            payload = response.read(MAX_GITHUB_JSON_BYTES + 1)
    except urllib.error.HTTPError as exc:
        status = exc.code
        payload = exc.read(MAX_GITHUB_JSON_BYTES + 1)
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise G0ReadinessError(
            f'cannot read GitHub product PR API: {exc}'
        ) from exc
    if len(payload) > MAX_GITHUB_JSON_BYTES:
        raise G0ReadinessError(
            'GitHub product PR API response exceeds the byte limit'
        )
    if not payload:
        return status, None
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise G0ReadinessError(
            'GitHub product PR API returned invalid JSON'
        ) from exc
    if not isinstance(value, dict):
        raise G0ReadinessError(
            'GitHub product PR API JSON root is not an object'
        )
    return status, value


def _local_head() -> str:
    """Resolve the exact local review tip without consulting a remote."""
    result = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    head = result.stdout.strip()
    if result.returncode != 0 or SHA_PATTERN.fullmatch(head) is None:
        detail = result.stderr.strip() or 'HEAD is not a full commit ID'
        raise G0ReadinessError(f'cannot resolve local product tip: {detail}')
    return head


def _is_local_ancestor(ancestor: str, descendant: str) -> bool | None:
    """Check local commit ancestry without reading or updating a remote."""
    result = subprocess.run(
        ['git', 'merge-base', '--is-ancestor', ancestor, descendant],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode == 0:
        return True
    if result.returncode == 1:
        return False
    return None


def _product_draft_result(
    *,
    status: str,
    state: str,
    is_draft: bool | None,
    merged: bool | None,
    mergeable: bool | None,
    local_head: str,
    remote_head: str | None,
    head_matches_local: bool | None,
    observed_check_count: int,
    passing_check_count: int,
    skipped_check_count: int,
    pending_check_count: int,
    failing_check_count: int,
    required_checks_complete: bool | None,
    blockers: list[str],
    decision_state: str,
    non_force_update_possible: bool | None = None,
    remote_description_sha256: str | None = None,
) -> dict[str, Any]:
    """Build one bounded PR result that can never authorize a write."""
    return {
        'status': status,
        'pull_request': PRODUCT_PR_NUMBER,
        'url': PRODUCT_PR_URL,
        'state': state,
        'is_draft': is_draft,
        'merged': merged,
        'mergeable': mergeable,
        'base_ref': PRODUCT_PR_BASE,
        'head_ref': PRODUCT_PR_HEAD,
        'local_head': local_head,
        'remote_head': remote_head,
        'head_matches_local': head_matches_local,
        'non_force_update_possible': non_force_update_possible,
        'remote_description_sha256': remote_description_sha256,
        'observed_check_count': observed_check_count,
        'passing_check_count': passing_check_count,
        'skipped_check_count': skipped_check_count,
        'pending_check_count': pending_check_count,
        'failing_check_count': failing_check_count,
        'required_checks_complete': required_checks_complete,
        'blockers': blockers,
        'decision_state': decision_state,
        'authority': {
            'network_reads_performed': True,
            'github_writes_authorized': False,
            'merge_authorized': False,
            'remote_mutations_performed': False,
        },
    }


def _blocked_product_draft(
    *,
    local_head: str,
    detail: str,
    state: str = 'UNKNOWN',
    is_draft: bool | None = None,
    merged: bool | None = None,
    mergeable: bool | None = None,
    remote_head: str | None = None,
    head_matches_local: bool | None = None,
    observed_check_count: int = 0,
    passing_check_count: int = 0,
    skipped_check_count: int = 0,
    pending_check_count: int = 0,
    failing_check_count: int = 0,
    non_force_update_possible: bool | None = None,
    remote_description_sha256: str | None = None,
) -> dict[str, Any]:
    """Return a fail-closed PR state without leaking remote free text."""
    return _product_draft_result(
        status='BLOCKED',
        state=state,
        is_draft=is_draft,
        merged=merged,
        mergeable=mergeable,
        local_head=local_head,
        remote_head=remote_head,
        head_matches_local=head_matches_local,
        observed_check_count=observed_check_count,
        passing_check_count=passing_check_count,
        skipped_check_count=skipped_check_count,
        pending_check_count=pending_check_count,
        failing_check_count=failing_check_count,
        required_checks_complete=False,
        blockers=[detail],
        decision_state='HOLD',
        non_force_update_possible=non_force_update_possible,
        remote_description_sha256=remote_description_sha256,
    )


def _object(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise G0ReadinessError(f'{label} must be an object')
    return value


def audit_product_draft(
    *,
    fetcher: GithubFetcher = _github_json,
    local_head: str | None = None,
    ancestor_checker: AncestorChecker = _is_local_ancestor,
) -> dict[str, Any]:
    """Audit exact Draft PR identity and check runs through GitHub GETs."""
    exact_local_head = local_head if local_head is not None else _local_head()
    if SHA_PATTERN.fullmatch(exact_local_head) is None:
        raise G0ReadinessError('local product tip is not a full commit ID')

    try:
        pull_status, pull = fetcher(
            f'repos/{REPOSITORY}/pulls/{PRODUCT_PR_NUMBER}'
        )
    except G0ReadinessError as exc:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=str(exc),
        )
    if pull_status != 200 or pull is None:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=f'Product PR is not readable (HTTP {pull_status}).',
        )

    raw_description = pull.get('body')
    if raw_description is None:
        normalized_description = ''
    elif isinstance(raw_description, str):
        normalized_description = raw_description.replace('\r\n', '\n')
    else:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail='Product PR description is not text or null.',
        )
    if len(normalized_description.encode('utf-8')) > (
        MAX_PRODUCT_DESCRIPTION_BYTES
    ):
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail='Product PR description exceeds the audit size limit.',
        )
    remote_description_sha256 = hashlib.sha256(
        normalized_description.encode('utf-8')
    ).hexdigest()

    try:
        head = _object(pull.get('head'), 'product PR head')
        base = _object(pull.get('base'), 'product PR base')
        head_repo = _object(head.get('repo'), 'product PR head repository')
        base_repo = _object(base.get('repo'), 'product PR base repository')
    except G0ReadinessError as exc:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=str(exc),
        )

    remote_head = head.get('sha')
    raw_state = pull.get('state')
    state = raw_state.upper() if raw_state in ('open', 'closed') else 'UNKNOWN'
    is_draft = pull.get('draft')
    merged = pull.get('merged')
    mergeable = pull.get('mergeable')
    head_matches_local = (
        remote_head == exact_local_head
        if isinstance(remote_head, str)
        else None
    )
    identity_valid = (
        pull.get('number') == PRODUCT_PR_NUMBER
        and pull.get('html_url') == PRODUCT_PR_URL
        and raw_state in ('open', 'closed')
        and isinstance(is_draft, bool)
        and isinstance(merged, bool)
        and isinstance(remote_head, str)
        and SHA_PATTERN.fullmatch(remote_head) is not None
        and head.get('ref') == PRODUCT_PR_HEAD
        and base.get('ref') == PRODUCT_PR_BASE
        and head_repo.get('full_name') == REPOSITORY
        and base_repo.get('full_name') == REPOSITORY
    )
    if not identity_valid:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail='Product PR identity or branch contract is invalid.',
            state=state,
            is_draft=is_draft if isinstance(is_draft, bool) else None,
            merged=merged if isinstance(merged, bool) else None,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=(
                remote_head
                if isinstance(remote_head, str)
                and SHA_PATTERN.fullmatch(remote_head) is not None
                else None
            ),
            head_matches_local=head_matches_local,
        )
    if not head_matches_local:
        non_force_update_possible = ancestor_checker(
            remote_head,
            exact_local_head,
        )
        if (
            non_force_update_possible is not None
            and not isinstance(non_force_update_possible, bool)
        ):
            raise G0ReadinessError(
                'product Draft ancestor checker returned an invalid result'
            )
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail='Local and public product PR heads do not match.',
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=False,
            non_force_update_possible=non_force_update_possible,
            remote_description_sha256=remote_description_sha256,
        )
    if raw_state == 'closed' and not merged:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail='Product PR is closed without being merged.',
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=True,
        )
    if raw_state == 'open' and mergeable is not True:
        detail = (
            'Product PR mergeability is still unknown.'
            if mergeable is None
            else 'Product PR is not mergeable.'
        )
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=detail,
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=True,
        )

    try:
        checks_status, checks = fetcher(
            f'repos/{REPOSITORY}/commits/{remote_head}/check-runs?per_page=100'
        )
    except G0ReadinessError as exc:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=str(exc),
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=True,
        )
    if checks_status != 200 or checks is None:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=(
                f'Exact-head checks are not readable '
                f'(HTTP {checks_status}).'
            ),
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=True,
        )

    runs = checks.get('check_runs')
    total_count = checks.get('total_count')
    if (
        not isinstance(runs, list)
        or isinstance(total_count, bool)
        or not isinstance(total_count, int)
        or total_count != len(runs)
        or total_count > MAX_CHECK_RUNS
    ):
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail='Exact-head check-run inventory is invalid or truncated.',
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=True,
        )

    latest_by_name: dict[str, dict[str, Any]] = {}
    for raw_run in runs:
        if not isinstance(raw_run, dict):
            return _blocked_product_draft(
                local_head=exact_local_head,
                detail='Exact-head check-run inventory contains a bad item.',
                state=state,
                is_draft=is_draft,
                merged=merged,
                mergeable=(
                    mergeable if isinstance(mergeable, bool) else None
                ),
                remote_head=remote_head,
                head_matches_local=True,
            )
        name = raw_run.get('name')
        run_id = raw_run.get('id')
        if (
            not isinstance(name, str)
            or not name
            or len(name) > 200
            or '\n' in name
            or '\r' in name
            or isinstance(run_id, bool)
            or not isinstance(run_id, int)
            or run_id <= 0
        ):
            return _blocked_product_draft(
                local_head=exact_local_head,
                detail='Exact-head check-run identity is invalid.',
                state=state,
                is_draft=is_draft,
                merged=merged,
                mergeable=(
                    mergeable if isinstance(mergeable, bool) else None
                ),
                remote_head=remote_head,
                head_matches_local=True,
            )
        current = latest_by_name.get(name)
        if current is None or run_id > current['id']:
            latest_by_name[name] = raw_run

    passing = 0
    skipped = 0
    pending = 0
    failing = 0
    for run in latest_by_name.values():
        status_value = run.get('status')
        conclusion = run.get('conclusion')
        if status_value != 'completed':
            pending += 1
        elif conclusion in ('success', 'neutral'):
            passing += 1
        elif conclusion == 'skipped':
            skipped += 1
        else:
            failing += 1

    check_blockers: list[str] = []
    missing_success = REQUIRED_SUCCESS_CHECKS - latest_by_name.keys()
    missing_skipped = REQUIRED_SKIPPED_CHECKS - latest_by_name.keys()
    if missing_success:
        check_blockers.append(
            f'{len(missing_success)} required successful checks are missing.'
        )
    if missing_skipped:
        check_blockers.append(
            f'{len(missing_skipped)} expected non-publication skips are '
            'missing.'
        )
    wrong_success = sum(
        1 for name in REQUIRED_SUCCESS_CHECKS
        if name in latest_by_name
        and (
            latest_by_name[name].get('status') != 'completed'
            or latest_by_name[name].get('conclusion') != 'success'
        )
    )
    wrong_skipped = sum(
        1 for name in REQUIRED_SKIPPED_CHECKS
        if name in latest_by_name
        and (
            latest_by_name[name].get('status') != 'completed'
            or latest_by_name[name].get('conclusion') != 'skipped'
        )
    )
    if wrong_success:
        check_blockers.append(
            f'{wrong_success} required successful checks are not successful.'
        )
    if wrong_skipped:
        check_blockers.append(
            f'{wrong_skipped} expected non-publication jobs are not skipped.'
        )
    extra_bad = sum(
        1 for name, run in latest_by_name.items()
        if name not in REQUIRED_SUCCESS_CHECKS
        and name not in REQUIRED_SKIPPED_CHECKS
        and (
            run.get('status') != 'completed'
            or run.get('conclusion') not in ACCEPTED_EXTRA_CONCLUSIONS
        )
    )
    if extra_bad:
        check_blockers.append(
            f'{extra_bad} additional exact-head checks are not complete.'
        )
    if check_blockers:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=' '.join(check_blockers),
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=True,
            observed_check_count=len(latest_by_name),
            passing_check_count=passing,
            skipped_check_count=skipped,
            pending_check_count=pending,
            failing_check_count=failing,
        )

    if merged:
        result_status = 'MERGED'
        decision_state = 'MERGED'
    elif is_draft:
        result_status = 'DRAFT_REVIEW_REQUIRED'
        decision_state = 'REVIEW_DRAFT'
    else:
        result_status = 'READY_FOR_SEPARATE_MERGE_REVIEW'
        decision_state = 'READY_FOR_SEPARATE_MERGE_REVIEW'
    return _product_draft_result(
        status=result_status,
        state=state,
        is_draft=is_draft,
        merged=merged,
        mergeable=(mergeable if isinstance(mergeable, bool) else None),
        local_head=exact_local_head,
        remote_head=remote_head,
        head_matches_local=True,
        observed_check_count=len(latest_by_name),
        passing_check_count=passing,
        skipped_check_count=skipped,
        pending_check_count=pending,
        failing_check_count=failing,
        required_checks_complete=True,
        blockers=[],
        decision_state=decision_state,
        remote_description_sha256=remote_description_sha256,
    )


def collect_checker_reports(
    *,
    product_draft_review_ledger: Path | None = None,
    include_product_draft: bool = False,
    include_candidate_environment: bool = False,
    include_published_release: bool = False,
    published_release_version: str = DEFAULT_RELEASE_VERSION,
    runner: Runner = subprocess.run,
) -> dict[str, dict[str, Any] | None]:
    """Collect existing checker reports without changing their semantics."""
    reports: dict[str, dict[str, Any] | None] = {
        'publication_plan': _run_json(
            'check_publication_slice_plan.py',
            ('--json',),
            runner=runner,
        ),
        'publication_overview': _run_json(
            'check_publication_slice_plan.py',
            ('--overview', '--json'),
            runner=runner,
        ),
        'product_draft_review_routing': _run_json(
            'check_product_draft_review_routing.py',
            ('--json',),
            runner=runner,
        ),
        'product_draft_review_ledger': None,
        'onboarding_matrix': _run_json(
            'check_onboarding_trial_matrix.py',
            ('--json',),
            runner=runner,
        ),
        'first_map_cohort': _run_json(
            'first_map_validator_cohort.py',
            ('--json',),
            runner=runner,
        ),
        'v1_readiness': _run_json(
            'check_v1_readiness.py',
            ('--json',),
            runner=runner,
        ),
        'product_draft': None,
        'candidate_environment': None,
        'published_release': None,
    }
    if product_draft_review_ledger is not None:
        reports['product_draft_review_ledger'] = _run_json(
            'product_draft_review_ledger.py',
            (
                'check',
                '--ledger',
                str(product_draft_review_ledger),
                '--json',
            ),
            runner=runner,
        )
    if include_product_draft:
        reports['product_draft'] = audit_product_draft()
    if include_candidate_environment:
        reports['candidate_environment'] = _run_json(
            'check_candidate_environment.py',
            ('--json',),
            runner=runner,
        )
    if include_published_release:
        reports['published_release'] = _run_json(
            'check_published_release.py',
            ('--version', published_release_version, '--json'),
            runner=runner,
        )
    return reports


def _publication_review_navigation_summary(
    report: dict[str, Any],
    plan: dict[str, Any],
) -> dict[str, Any]:
    """Normalize exact phase links and bounded slice labels for reviewers."""
    candidate = report.get('candidate')
    phases = report.get('review_phases')
    slices = report.get('review_slices')
    if (
        report.get('status') != 'PR_REVIEW_OVERVIEW_READY_LOCAL_ONLY'
        or report.get('commands_executed') is not False
        or report.get('github_writes_authorized') is not False
        or report.get('remote_mutations_performed') is not False
        or report.get('slice_command_template')
        != PUBLICATION_REVIEW_SLICE_TEMPLATE
        or not isinstance(candidate, dict)
        or not isinstance(phases, list)
        or not isinstance(slices, list)
    ):
        raise G0ReadinessError(
            'publication review overview is unsafe or incomplete'
        )
    candidate_contract = {
        'local_tip_sha': plan.get('local_tip_sha'),
        'whole_pr_commit_count': plan.get('whole_pr_commit_count'),
        'whole_pr_path_count': plan.get('whole_pr_path_count'),
        'follow_up_path_count': plan.get('path_count'),
        'review_phase_count': plan.get('review_phase_count'),
        'slice_count': plan.get('slice_count'),
        'review_coverage_complete': True,
        'merge_commit_count': 0,
        'worktree_clean': plan.get('worktree_clean'),
        'uncommitted_path_count': plan.get('uncommitted_path_count'),
    }
    if any(
        candidate.get(field) != expected
        for field, expected in candidate_contract.items()
    ):
        raise G0ReadinessError(
            'publication review overview contradicts the publication plan'
        )
    whole_pr_base_sha = candidate.get('whole_pr_base_sha')
    if (
        not isinstance(whole_pr_base_sha, str)
        or SHA_PATTERN.fullmatch(whole_pr_base_sha) is None
        or len(phases) != len(EXPECTED_REVIEW_PHASES)
        or len(slices) != len(EXPECTED_REVIEW_SLICES)
    ):
        raise G0ReadinessError(
            'publication review overview has invalid coverage identity'
        )

    normalized_phases: list[dict[str, Any]] = []
    previous_end = whole_pr_base_sha
    for order, (raw, expected) in enumerate(
        zip(phases, EXPECTED_REVIEW_PHASES),
        start=1,
    ):
        expected_id, expected_mode = expected
        if not isinstance(raw, dict):
            raise G0ReadinessError(
                'publication review overview has a malformed phase'
            )
        start_sha = raw.get('start_sha')
        end_sha = raw.get('end_sha')
        commit_count = raw.get('commit_count')
        path_count = raw.get('path_count')
        if (
            raw.get('id') != expected_id
            or raw.get('order') != order
            or raw.get('diff_mode') != expected_mode
            or not isinstance(start_sha, str)
            or SHA_PATTERN.fullmatch(start_sha) is None
            or not isinstance(end_sha, str)
            or SHA_PATTERN.fullmatch(end_sha) is None
            or start_sha != previous_end
            or isinstance(commit_count, bool)
            or not isinstance(commit_count, int)
            or commit_count < 1
            or isinstance(path_count, bool)
            or not isinstance(path_count, int)
            or path_count < 1
        ):
            raise G0ReadinessError(
                f'publication review phase {expected_id} is invalid'
            )
        normalized_phases.append({
            'id': expected_id,
            'order': order,
            'start_sha': start_sha,
            'end_sha': end_sha,
            'commit_count': commit_count,
            'path_count': path_count,
            'compare_url': (
                f'{PRODUCT_GITHUB_URL}/compare/{start_sha}...{end_sha}'
            ),
        })
        previous_end = end_sha
    if (
        previous_end != plan.get('local_tip_sha')
        or sum(item['commit_count'] for item in normalized_phases)
        != plan.get('whole_pr_commit_count')
        or normalized_phases[-1]['commit_count']
        != plan.get('follow_up_review_commit_count')
        or normalized_phases[-1]['path_count'] != plan.get('path_count')
    ):
        raise G0ReadinessError(
            'publication review phase composition is not exact'
        )

    normalized_slices: list[dict[str, Any]] = []
    for order, (raw, expected_id) in enumerate(
        zip(slices, EXPECTED_REVIEW_SLICES),
        start=1,
    ):
        if not isinstance(raw, dict):
            raise G0ReadinessError(
                'publication review overview has a malformed slice'
            )
        title = raw.get('title')
        path_count = raw.get('path_count')
        verification_count = raw.get('verification_count')
        publication_gate = raw.get('publication_gate')
        if (
            raw.get('id') != expected_id
            or raw.get('order') != order
            or not isinstance(title, str)
            or SAFE_REVIEW_TITLE_PATTERN.fullmatch(title) is None
            or isinstance(path_count, bool)
            or not isinstance(path_count, int)
            or path_count < 1
            or isinstance(verification_count, bool)
            or not isinstance(verification_count, int)
            or verification_count < 1
            or publication_gate not in REVIEW_PUBLICATION_GATES
        ):
            raise G0ReadinessError(
                f'publication review slice {expected_id} is invalid'
            )
        normalized_slices.append({
            'id': expected_id,
            'order': order,
            'title': title,
            'path_count': path_count,
            'verification_count': verification_count,
            'publication_gate': publication_gate,
        })
    if sum(item['path_count'] for item in normalized_slices) != plan.get(
        'path_count'
    ):
        raise G0ReadinessError(
            'publication review slices do not compose the follow-up scope'
        )
    return {
        'status': 'READY_LOCAL_ONLY',
        'exact_head': plan['local_tip_sha'],
        'whole_pr_base_sha': whole_pr_base_sha,
        'phase_count': len(normalized_phases),
        'slice_count': len(normalized_slices),
        'phases': normalized_phases,
        'slices': normalized_slices,
        'overview_command': PUBLICATION_REVIEW_OVERVIEW_COMMAND,
        'slice_command_template': PUBLICATION_REVIEW_SLICE_TEMPLATE,
        'commands_executed': False,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }


def _review_routing_summary(
    report: dict[str, Any],
    plan: dict[str, Any],
    navigation: dict[str, Any],
) -> dict[str, Any]:
    """Normalize privacy-safe capability lanes against the exact review map."""
    expected_status = (
        'READY_LOCAL_ONLY'
        if plan.get('worktree_clean') is True
        and plan.get('uncommitted_path_count') == 0
        else 'PREPARED_DIRTY_WORKTREE'
    )
    expected_authority = {
        'commands_executed': False,
        'github_reviewer_requests_authorized': False,
        'github_reviews_authorized': False,
        'mark_ready_authorized': False,
        'merge_authorized': False,
        'remote_mutations_performed': False,
    }
    policy = {
        'advisory_reviewer_target': 2,
        'advisory_target_is_merge_gate': False,
        'max_parallel_active_lanes': 2,
        'lane_completion_order_required': True,
        'final_decision_role': 'lead-maintainer',
    }
    if (
        report.get('status') != expected_status
        or report.get('exact_head') != plan.get('local_tip_sha')
        or report.get('exact_head') != navigation.get('exact_head')
        or report.get('worktree_clean') != plan.get('worktree_clean')
        or report.get('uncommitted_path_count')
        != plan.get('uncommitted_path_count')
        or any(report.get(field) != value for field, value in policy.items())
        or report.get('authority') != expected_authority
    ):
        raise G0ReadinessError(
            'product Draft review routing contradicts the exact local plan'
        )
    raw_lanes = report.get('lanes')
    raw_summary = report.get('summary')
    slices = navigation.get('slices')
    if (
        not isinstance(raw_lanes, list)
        or len(raw_lanes) != len(EXPECTED_REVIEW_LANES)
        or not isinstance(raw_summary, dict)
        or not isinstance(slices, list)
    ):
        raise G0ReadinessError(
            'product Draft review routing is incomplete'
        )
    slice_by_id = {
        item.get('id'): item
        for item in slices
        if isinstance(item, dict)
    }
    if set(slice_by_id) != set(EXPECTED_REVIEW_SLICES):
        raise G0ReadinessError(
            'product Draft review routing received an invalid slice map'
        )

    normalized_lanes: list[dict[str, Any]] = []
    assigned_slices: list[str] = []
    for order, (raw, lane_id, dependencies, slice_ids) in enumerate(
        zip(
            raw_lanes,
            EXPECTED_REVIEW_LANES,
            EXPECTED_REVIEW_LANE_DEPENDENCIES,
            EXPECTED_REVIEW_LANE_SLICES,
        ),
        start=1,
    ):
        if not isinstance(raw, dict):
            raise G0ReadinessError(
                f'product Draft review lane {lane_id} is malformed'
            )
        title = raw.get('title')
        capability = raw.get('capability')
        expected_path_count = sum(
            slice_by_id[slice_id]['path_count'] for slice_id in slice_ids
        )
        expected_verification_count = sum(
            slice_by_id[slice_id]['verification_count']
            for slice_id in slice_ids
        )
        expected_commands = [
            'python3 scripts/check_publication_slice_plan.py '
            f'--slice {slice_id}'
            for slice_id in slice_ids
        ]
        if (
            raw.get('id') != lane_id
            or raw.get('order') != order
            or not isinstance(title, str)
            or SAFE_REVIEW_TITLE_PATTERN.fullmatch(title) is None
            or not isinstance(capability, str)
            or SAFE_REVIEW_TITLE_PATTERN.fullmatch(capability) is None
            or raw.get('slice_ids') != list(slice_ids)
            or raw.get('depends_on_lanes') != list(dependencies)
            or raw.get('path_count') != expected_path_count
            or raw.get('verification_count')
            != expected_verification_count
            or raw.get('slice_commands') != expected_commands
        ):
            raise G0ReadinessError(
                f'product Draft review lane {lane_id} is invalid'
            )
        assigned_slices.extend(slice_ids)
        normalized_lanes.append({
            'id': lane_id,
            'order': order,
            'title': title,
            'capability': capability,
            'slice_ids': list(slice_ids),
            'depends_on_lanes': list(dependencies),
            'path_count': expected_path_count,
            'verification_count': expected_verification_count,
            'slice_commands': expected_commands,
        })

    expected_summary = {
        'lane_count': len(EXPECTED_REVIEW_LANES),
        'slice_count': len(EXPECTED_REVIEW_SLICES),
        'path_count': sum(item['path_count'] for item in slices),
        'verification_count': sum(
            item['verification_count'] for item in slices
        ),
        'unassigned_slice_count': 0,
        'duplicate_slice_count': 0,
    }
    if (
        assigned_slices != list(EXPECTED_REVIEW_SLICES)
        or raw_summary != expected_summary
    ):
        raise G0ReadinessError(
            'product Draft review lanes do not cover the review map exactly'
        )
    return {
        'status': expected_status,
        'exact_head': report['exact_head'],
        'worktree_clean': report['worktree_clean'],
        'uncommitted_path_count': report['uncommitted_path_count'],
        'policy': policy,
        'lanes': normalized_lanes,
        'summary': expected_summary,
        'authority': expected_authority,
    }


def _review_ledger_summary(
    report: dict[str, Any] | None,
    plan: dict[str, Any],
    routing: dict[str, Any],
) -> dict[str, Any]:
    """Normalize optional anonymous review evidence without upgrading authority."""
    authority = {
        'identities_collected': False,
        'review_commands_executed_by_tool': False,
        'github_reviewer_requests_authorized': False,
        'github_reviews_authorized': False,
        'mark_ready_authorized': False,
        'merge_authorized': False,
        'remote_mutations_performed': False,
    }
    if report is None:
        return {
            'status': 'NOT_CHECKED',
            'exact_head': None,
            'ledger_sha256': None,
            'routing_contract_sha256': None,
            'worktree_clean': None,
            'event_count': 0,
            'reviewed_lane_count': 0,
            'passing_lane_count': 0,
            'blocked_lane_count': 0,
            'historical_finding_count': 0,
            'current_finding_count': 0,
            'open_blocker_count': 0,
            'current_lanes': [],
            'next_lane_id': None,
            'authority': authority,
        }
    status = report.get('status')
    allowed_statuses = {
        'EMPTY_LOCAL_LEDGER',
        'IN_PROGRESS_LOCAL_REVIEW',
        'BLOCKED_LOCAL_REVIEW',
        'COMPLETE_LOCAL_REVIEW',
    }
    digest_fields = (
        report.get('ledger_sha256'),
        report.get('routing_contract_sha256'),
    )
    raw_lanes = report.get('current_lanes')
    if (
        status not in allowed_statuses
        or report.get('exact_head') != plan.get('local_tip_sha')
        or report.get('exact_head') != routing.get('exact_head')
        or report.get('worktree_clean') is not True
        or any(
            not isinstance(value, str)
            or DIGEST_PATTERN.fullmatch(value) is None
            for value in digest_fields
        )
        or report.get('authority') != authority
        or not isinstance(raw_lanes, list)
        or len(raw_lanes) != len(EXPECTED_REVIEW_LANES)
    ):
        raise G0ReadinessError(
            'product Draft review ledger is stale, unsafe, or incomplete'
        )
    normalized_lanes: list[dict[str, Any]] = []
    for order, (raw_lane, routing_lane, expected_id) in enumerate(
        zip(raw_lanes, routing['lanes'], EXPECTED_REVIEW_LANES),
        start=1,
    ):
        if not isinstance(raw_lane, dict):
            raise G0ReadinessError(
                f'product Draft review ledger lane {expected_id} is malformed'
            )
        lane_status = raw_lane.get('status')
        verification_status = raw_lane.get('verification_status')
        latest_sequence = raw_lane.get('latest_event_sequence')
        finding_count = raw_lane.get('finding_count')
        blocker_count = raw_lane.get('blocker_count')
        if (
            raw_lane.get('id') != expected_id
            or raw_lane.get('order') != order
            or raw_lane.get('slice_ids') != routing_lane.get('slice_ids')
            or lane_status not in ('NOT_REVIEWED', 'PASS', 'BLOCKED')
            or verification_status not in ('NOT_RECORDED', 'PASS', 'FAIL')
            or (
                latest_sequence is not None
                and (
                    isinstance(latest_sequence, bool)
                    or not isinstance(latest_sequence, int)
                    or latest_sequence < 1
                    or latest_sequence > 100
                )
            )
            or isinstance(finding_count, bool)
            or not isinstance(finding_count, int)
            or not 0 <= finding_count <= 10
            or isinstance(blocker_count, bool)
            or not isinstance(blocker_count, int)
            or not 0 <= blocker_count <= finding_count
            or (
                lane_status == 'NOT_REVIEWED'
                and (
                    verification_status != 'NOT_RECORDED'
                    or latest_sequence is not None
                    or finding_count != 0
                    or blocker_count != 0
                )
            )
            or (
                lane_status == 'PASS'
                and (
                    verification_status != 'PASS'
                    or latest_sequence is None
                    or blocker_count != 0
                )
            )
            or (
                lane_status == 'BLOCKED'
                and (latest_sequence is None or blocker_count < 1)
            )
        ):
            raise G0ReadinessError(
                f'product Draft review ledger lane {expected_id} is invalid'
            )
        normalized_lanes.append({
            'id': expected_id,
            'order': order,
            'slice_ids': list(raw_lane['slice_ids']),
            'status': lane_status,
            'verification_status': verification_status,
            'latest_event_sequence': latest_sequence,
            'finding_count': finding_count,
            'blocker_count': blocker_count,
        })
    integer_fields = (
        'event_count',
        'reviewed_lane_count',
        'passing_lane_count',
        'blocked_lane_count',
        'historical_finding_count',
        'current_finding_count',
        'open_blocker_count',
    )
    if any(
        isinstance(report.get(field), bool)
        or not isinstance(report.get(field), int)
        or report[field] < 0
        for field in integer_fields
    ):
        raise G0ReadinessError(
            'product Draft review ledger has invalid summary counts'
        )
    expected_passing = sum(
        lane['status'] == 'PASS' for lane in normalized_lanes
    )
    expected_blocked = sum(
        lane['status'] == 'BLOCKED' for lane in normalized_lanes
    )
    expected_reviewed = expected_passing + expected_blocked
    expected_current_findings = sum(
        lane['finding_count'] for lane in normalized_lanes
    )
    expected_blockers = sum(
        lane['blocker_count'] for lane in normalized_lanes
    )
    expected_next_lane = next(
        (lane['id'] for lane in normalized_lanes if lane['status'] != 'PASS'),
        None,
    )
    if (
        report['reviewed_lane_count'] != expected_reviewed
        or report['passing_lane_count'] != expected_passing
        or report['blocked_lane_count'] != expected_blocked
        or report['current_finding_count'] != expected_current_findings
        or report['open_blocker_count'] != expected_blockers
        or report['event_count'] < expected_reviewed
        or report['historical_finding_count'] < expected_current_findings
        or report.get('next_lane_id') != expected_next_lane
        or (status == 'EMPTY_LOCAL_LEDGER' and expected_reviewed != 0)
        or (
            status == 'IN_PROGRESS_LOCAL_REVIEW'
            and not (0 < expected_passing < len(EXPECTED_REVIEW_LANES))
        )
        or (status == 'BLOCKED_LOCAL_REVIEW' and expected_blocked < 1)
        or (
            status == 'COMPLETE_LOCAL_REVIEW'
            and expected_passing != len(EXPECTED_REVIEW_LANES)
        )
    ):
        raise G0ReadinessError(
            'product Draft review ledger summary contradicts its lanes'
        )
    return {
        'status': status,
        'exact_head': report['exact_head'],
        'ledger_sha256': report['ledger_sha256'],
        'routing_contract_sha256': report['routing_contract_sha256'],
        'worktree_clean': True,
        **{field: report[field] for field in integer_fields},
        'current_lanes': normalized_lanes,
        'next_lane_id': expected_next_lane,
        'authority': authority,
    }


def _matrix_summary(report: dict[str, Any]) -> dict[str, Any]:
    summary = report.get('summary')
    decision = report.get('decision')
    if not isinstance(summary, dict) or not isinstance(decision, dict):
        raise G0ReadinessError('onboarding matrix report is incomplete')
    return {
        'status': decision.get('status'),
        'product_versions': report.get('product_versions', []),
        'present_rows': summary.get('present_rows'),
        'pass_rows': summary.get('pass_rows'),
        'comparable_rows': summary.get('comparable_rows'),
        'docker_comparable_rows': summary.get('docker_comparable_rows'),
        'source_comparable_rows': summary.get('source_comparable_rows'),
        'product_version_aligned': summary.get('product_version_aligned'),
        'activation_gate': summary.get('activation_gate'),
        'actions': decision.get('actions', []),
    }


def _cohort_summary(report: dict[str, Any]) -> dict[str, Any]:
    required = (
        'status',
        'launch_status',
        'attempt_count',
        'accepted_validations',
        'accepted_target',
        'pending_launch_gates',
    )
    if any(field not in report for field in required):
        raise G0ReadinessError('first-map cohort report is incomplete')
    pending_launch_gates = report['pending_launch_gates']
    if not isinstance(pending_launch_gates, list) or not all(
        isinstance(gate, str)
        and gate
        and '\n' not in gate
        and '\r' not in gate
        for gate in pending_launch_gates
    ):
        raise G0ReadinessError(
            'first-map cohort contains unsafe pending launch gate fields'
        )
    return {field: report[field] for field in required}


def _v1_summary(report: dict[str, Any]) -> dict[str, Any]:
    summary = report.get('summary')
    if not isinstance(summary, dict):
        raise G0ReadinessError('v1 readiness report is incomplete')
    gates = report.get('gates', [])
    incomplete_gate_details: list[dict[str, Any]] = []
    for gate in gates:
        if gate.get('status') != 'INCOMPLETE':
            continue
        gate_id = gate.get('id')
        title = gate.get('title')
        detail = gate.get('detail')
        blockers = gate.get('blockers', [])
        if not all(isinstance(value, str) and value for value in (
            gate_id, title, detail,
        )) or not isinstance(blockers, list) or not all(
            isinstance(item, str) and item for item in blockers
        ):
            raise G0ReadinessError(
                'v1 readiness contains an incomplete gate without safe '
                'display fields'
            )
        incomplete_gate_details.append({
            'id': gate_id,
            'title': title,
            'detail': detail,
            'blockers': blockers,
        })
    return {
        'status': report.get('status'),
        'complete': summary.get('complete'),
        'incomplete': summary.get('incomplete'),
        'total': summary.get('total'),
        'incomplete_gates': [
            gate.get('id')
            for gate in gates
            if gate.get('status') == 'INCOMPLETE'
        ],
        'incomplete_gate_details': incomplete_gate_details,
    }


def _published_summary(
    report: dict[str, Any] | None,
    version: str,
) -> dict[str, Any]:
    if VERSION_PATTERN.fullmatch(version) is None:
        raise G0ReadinessError(
            'published release version must be one safe semantic version'
        )
    if report is None:
        return {
            'status': 'NOT_CHECKED',
            'version': version,
            'tag_present': None,
            'image_statuses': [],
        }
    observed_version = report.get('expected_version')
    if observed_version != version:
        raise G0ReadinessError(
            'published release report version does not match the requested '
            'transition version'
        )
    return {
        'status': report.get('status'),
        'version': observed_version,
        'tag_present': report.get('remote', {}).get('tag_present'),
        'image_statuses': [
            {'tag': image.get('tag'), 'status': image.get('status')}
            for image in report.get('images', [])
        ],
    }


def _product_draft_summary(
    report: dict[str, Any] | None,
) -> dict[str, Any]:
    """Keep an optional exact-head PR audit distinct from a review pass."""
    if report is None:
        return {
            'status': 'NOT_CHECKED',
            'pull_request': PRODUCT_PR_NUMBER,
            'url': PRODUCT_PR_URL,
            'state': 'NOT_CHECKED',
            'is_draft': None,
            'merged': None,
            'mergeable': None,
            'base_ref': PRODUCT_PR_BASE,
            'head_ref': PRODUCT_PR_HEAD,
            'local_head': None,
            'remote_head': None,
            'head_matches_local': None,
            'non_force_update_possible': None,
            'remote_description_sha256': None,
            'observed_check_count': 0,
            'passing_check_count': 0,
            'skipped_check_count': 0,
            'pending_check_count': 0,
            'failing_check_count': 0,
            'required_checks_complete': None,
            'blockers': [],
            'decision_state': 'NOT_CHECKED',
            'merge_authorized': False,
        }
    authority = report.get('authority')
    if not isinstance(authority, dict) or authority != {
        'network_reads_performed': True,
        'github_writes_authorized': False,
        'merge_authorized': False,
        'remote_mutations_performed': False,
    }:
        raise G0ReadinessError(
            'product Draft audit claims unsafe or incomplete authority'
        )
    if (
        report.get('pull_request') != PRODUCT_PR_NUMBER
        or report.get('url') != PRODUCT_PR_URL
        or report.get('base_ref') != PRODUCT_PR_BASE
        or report.get('head_ref') != PRODUCT_PR_HEAD
    ):
        raise G0ReadinessError('product Draft audit identity is invalid')
    blockers = report.get('blockers')
    if (
        not isinstance(blockers, list)
        or len(blockers) > 3
        or not all(
            isinstance(item, str)
            and item
            and len(item) <= 1000
            and '\n' not in item
            and '\r' not in item
            for item in blockers
        )
    ):
        raise G0ReadinessError('product Draft audit blockers are unsafe')
    non_force_update_possible = report.get('non_force_update_possible')
    if (
        non_force_update_possible is not None
        and not isinstance(non_force_update_possible, bool)
    ):
        raise G0ReadinessError(
            'product Draft non-force update state is invalid'
        )
    remote_description_sha256 = report.get(
        'remote_description_sha256'
    )
    if (
        remote_description_sha256 is not None
        and (
            not isinstance(remote_description_sha256, str)
            or DIGEST_PATTERN.fullmatch(remote_description_sha256) is None
        )
    ):
        raise G0ReadinessError(
            'product Draft description digest is invalid'
        )
    if isinstance(non_force_update_possible, bool) and (
        report.get('status') != 'BLOCKED'
        or report.get('head_matches_local') is not False
        or not isinstance(report.get('local_head'), str)
        or SHA_PATTERN.fullmatch(report['local_head']) is None
        or not isinstance(report.get('remote_head'), str)
        or SHA_PATTERN.fullmatch(report['remote_head']) is None
    ):
        raise G0ReadinessError(
            'product Draft non-force update claim contradicts its state'
        )
    fields = (
        'status',
        'pull_request',
        'url',
        'state',
        'is_draft',
        'merged',
        'mergeable',
        'base_ref',
        'head_ref',
        'local_head',
        'remote_head',
        'head_matches_local',
        'non_force_update_possible',
        'remote_description_sha256',
        'observed_check_count',
        'passing_check_count',
        'skipped_check_count',
        'pending_check_count',
        'failing_check_count',
        'required_checks_complete',
        'decision_state',
    )
    summary = {field: report.get(field) for field in fields}
    summary['blockers'] = list(blockers)
    summary['merge_authorized'] = False
    return summary


def _product_draft_update_handoff(
    product_draft: dict[str, Any],
) -> dict[str, Any]:
    """Build an exact, non-executing fast-forward branch handoff."""
    local_head = product_draft['local_head']
    remote_head = product_draft['remote_head']
    if (
        product_draft['status'] != 'BLOCKED'
        or product_draft['head_matches_local'] is not False
        or product_draft['non_force_update_possible'] is not True
        or not isinstance(local_head, str)
        or SHA_PATTERN.fullmatch(local_head) is None
        or not isinstance(remote_head, str)
        or SHA_PATTERN.fullmatch(remote_head) is None
    ):
        raise G0ReadinessError(
            'product Draft update handoff requires verified fast-forward tips'
        )
    return {
        'kind': 'NON_FORCE_PR_BRANCH_UPDATE',
        'authority_required': (
            'separate-exact-tip-non-force-push-approval'
        ),
        'external_write_required': True,
        'pull_request': PRODUCT_PR_NUMBER,
        'url': PRODUCT_PR_URL,
        'repository_url': PRODUCT_REPOSITORY_URL,
        'head_ref': PRODUCT_PR_HEAD,
        'public_head': remote_head,
        'local_head': local_head,
        'fast_forward_verified': True,
        'non_force_only': True,
        'push_authorized': False,
        'force_push_authorized': False,
        'steps': [
            (
                f'Confirm PR #{PRODUCT_PR_NUMBER} still has public head '
                f'{remote_head} and the reviewed local tip is {local_head}.'
            ),
            (
                f'Obtain separate exact-tip authority to update '
                f'{PRODUCT_PR_HEAD} from {remote_head} to {local_head} '
                'without force.'
            ),
            (
                'After an authorized external update, rerun the GET-only '
                'exact-head audit below.'
            ),
        ],
        'verification_command': PRODUCT_PR_VERIFY_COMMAND,
        'writes_performed': False,
    }


def _product_draft_description_body(
    plan: dict[str, Any],
    navigation: dict[str, Any],
    routing: dict[str, Any],
    matrix: dict[str, Any],
    cohort: dict[str, Any],
    v1: dict[str, Any],
    exact_head: str,
) -> str:
    """Render the canonical reviewer-facing Draft description."""
    positive_counts = (
        plan.get('whole_pr_commit_count'),
        plan.get('whole_pr_path_count'),
        plan.get('review_phase_count'),
        plan.get('follow_up_review_commit_count'),
        plan.get('path_count'),
        plan.get('slice_count'),
        cohort.get('accepted_target'),
        v1.get('total'),
    )
    nonnegative_counts = (
        matrix.get('present_rows'),
        matrix.get('comparable_rows'),
        cohort.get('accepted_validations'),
        v1.get('complete'),
    )
    if (
        not isinstance(exact_head, str)
        or SHA_PATTERN.fullmatch(exact_head) is None
        or plan.get('status') != 'PLAN_VALID_LOCAL_ONLY'
        or plan.get('local_tip_sha') != exact_head
        or plan.get('review_coverage_complete') is not True
        or plan.get('worktree_clean') is not True
        or plan.get('uncommitted_path_count') != 0
        or any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < 1
            for value in positive_counts
        )
        or any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < 0
            for value in nonnegative_counts
        )
        or matrix['present_rows'] > 4
        or matrix['comparable_rows'] > 4
        or matrix['comparable_rows'] > matrix['present_rows']
        or cohort['accepted_validations'] > cohort['accepted_target']
        or v1['complete'] > v1['total']
        or navigation.get('status') != 'READY_LOCAL_ONLY'
        or navigation.get('exact_head') != exact_head
        or navigation.get('phase_count') != plan.get('review_phase_count')
        or navigation.get('slice_count') != plan.get('slice_count')
        or navigation.get('commands_executed') is not False
        or navigation.get('github_writes_authorized') is not False
        or navigation.get('remote_mutations_performed') is not False
        or routing.get('status') != 'READY_LOCAL_ONLY'
        or routing.get('exact_head') != exact_head
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
    ):
        raise G0ReadinessError(
            'Draft description requires one clean exact candidate packet'
        )
    phases = navigation.get('phases')
    slices = navigation.get('slices')
    lanes = routing.get('lanes')
    routing_policy = routing.get('policy')
    if (
        not isinstance(phases, list)
        or len(phases) != plan['review_phase_count']
        or not isinstance(slices, list)
        or len(slices) != plan['slice_count']
        or not isinstance(lanes, list)
        or len(lanes) != len(EXPECTED_REVIEW_LANES)
        or not isinstance(routing_policy, dict)
        or routing_policy.get('advisory_reviewer_target') != 2
        or routing_policy.get('advisory_target_is_merge_gate') is not False
    ):
        raise G0ReadinessError(
            'Draft description review navigation is incomplete'
        )
    phase_lines = []
    for phase in phases:
        phase_label = phase['id'].split('-', maxsplit=1)[0]
        phase_lines.append(
            f"| `{phase_label}` | {phase['commit_count']} commits / "
            f"{phase['path_count']} paths | "
            f"[Open exact diff]({phase['compare_url']}) |"
        )
    slice_lines = []
    for review_slice in slices:
        slice_label = review_slice['id'].split('-', maxsplit=1)[0]
        slice_lines.append(
            f"| `{slice_label}` | {review_slice['title']} | "
            f"{review_slice['path_count']} | "
            f"{review_slice['verification_count']} | "
            f"`{review_slice['publication_gate']}` |"
        )
    lane_lines = []
    for lane in lanes:
        lane_label = lane['id'].split('-', maxsplit=1)[0]
        scope = ', '.join(
            slice_id.split('-', maxsplit=1)[0]
            for slice_id in lane['slice_ids']
        )
        lane_lines.append(
            f"| `{lane_label}` | {scope} | {lane['path_count']} | "
            f"{lane['verification_count']} | {lane['capability']} |"
        )
    lines = [
        '## Review intent',
        '',
        (
            'This Draft makes the rosbag2-to-Autoware point-cloud map '
            'workflow safer and easier to operate: guided commands, '
            'preflight and rollback behavior, recovery guidance, and '
            'machine-checked quality gates.'
        ),
        '',
        (
            'It stays Draft while maintainers review the bounded phases '
            'below. Green CI is evidence for one exact head, not merge '
            'authority.'
        ),
        '',
        '## Exact review candidate',
        '',
        f'- Candidate head: `{exact_head}`',
        (
            '- Whole PR review: '
            f"**{plan['whole_pr_commit_count']} commits / "
            f"{plan['whole_pr_path_count']} paths / "
            f"{plan['review_phase_count']} phases**"
        ),
        (
            '- P2 follow-up review: '
            f"**{plan['follow_up_review_commit_count']} commits / "
            f"{plan['path_count']} paths / {plan['slice_count']} slices**"
        ),
        '',
        (
            'After any branch update, CI must rerun on the candidate head '
            'before review conclusions are recorded.'
        ),
        '',
        '## Exact review map',
        '',
        '| Phase | Scope | GitHub diff |',
        '| --- | ---: | --- |',
        *phase_lines,
        '',
        '| Slice | Focus | Paths | Checks | Gate |',
        '| --- | --- | ---: | ---: | --- |',
        *slice_lines,
        '',
        (
            'Local bounded detail: '
            f'`{PUBLICATION_REVIEW_OVERVIEW_COMMAND}` then '
            f'`{PUBLICATION_REVIEW_SLICE_TEMPLATE}`.'
        ),
        '',
        '## Review roles',
        '',
        '| Lane | Scope | Paths | Checks | Capability |',
        '| --- | --- | ---: | ---: | --- |',
        *lane_lines,
        '',
        (
            'Advisory reviewer target: **2** (target only; not a merge '
            'gate). Identities collected: none.'
        ),
        '',
        '## Review order',
        '',
        '1. Open the exact P0, P1, then P2 links above.',
        '2. Confirm each link matches the displayed contiguous lineage.',
        (
            '3. Render S1 through S7 locally in dependency order and run '
            'each displayed verification group.'
        ),
        (
            '4. Record findings separately; review submission, mark-ready, '
            'and merge remain independent decisions.'
        ),
        '',
        '## Current external evidence',
        '',
        f"- v1 readiness: **{v1['complete']}/{v1['total']} complete**",
        (
            '- Onboarding trials: '
            f"**{matrix['present_rows']}/4 present, "
            f"{matrix['comparable_rows']}/4 comparable**"
        ),
        (
            '- Independent first-map validations: '
            f"**{cohort['accepted_validations']}/"
            f"{cohort['accepted_target']} accepted**"
        ),
        '',
        (
            'These are evidence gates, not PR merge gates. Distribution '
            'and independent adoption remain external work.'
        ),
        '',
        '## Authority boundary',
        '',
        (
            'This description does not authorize a push, review '
            'submission, mark-ready transition, merge, release, deployment, '
            'or community outreach.'
        ),
    ]
    return '\n'.join(lines)


def _product_draft_description_refresh_handoff(
    plan: dict[str, Any],
    navigation: dict[str, Any],
    routing: dict[str, Any],
    matrix: dict[str, Any],
    cohort: dict[str, Any],
    v1: dict[str, Any],
    product_draft: dict[str, Any],
) -> dict[str, Any]:
    """Bind a no-write PR-description refresh to one clean exact tip."""
    desired_head = product_draft.get('local_head')
    observed_public_head = product_draft.get('remote_head')
    after_branch_update_required = (
        product_draft.get('head_matches_local') is False
    )
    state_is_eligible = (
        product_draft.get('status') == 'DRAFT_REVIEW_REQUIRED'
        and product_draft.get('head_matches_local') is True
        and product_draft.get('is_draft') is True
    ) or (
        product_draft.get('status') == 'BLOCKED'
        and after_branch_update_required
        and product_draft.get('non_force_update_possible') is True
        and product_draft.get('is_draft') is True
    )
    if (
        not state_is_eligible
        or not isinstance(desired_head, str)
        or SHA_PATTERN.fullmatch(desired_head) is None
        or not isinstance(observed_public_head, str)
        or SHA_PATTERN.fullmatch(observed_public_head) is None
    ):
        raise G0ReadinessError(
            'Draft description handoff requires one eligible exact Draft'
        )
    body = _product_draft_description_body(
        plan,
        navigation,
        routing,
        matrix,
        cohort,
        v1,
        desired_head,
    )
    body_sha256 = hashlib.sha256(body.encode('utf-8')).hexdigest()
    observed_body_sha256 = product_draft.get(
        'remote_description_sha256'
    )
    if (
        observed_body_sha256 is not None
        and (
            not isinstance(observed_body_sha256, str)
            or DIGEST_PATTERN.fullmatch(observed_body_sha256) is None
        )
    ):
        raise G0ReadinessError(
            'Draft description handoff received an invalid observed digest'
        )
    first_step = (
        'Complete only the separately authorized non-force branch update '
        'and its GET-only exact-head verification first.'
        if after_branch_update_required
        else (
            f'Confirm PR #{PRODUCT_PR_NUMBER} is still Draft at exact head '
            f'{desired_head}.'
        )
    )
    return {
        'kind': 'REFRESH_EXACT_DRAFT_DESCRIPTION',
        'authority_required': (
            'separate-exact-tip-pr-description-update-approval'
        ),
        'external_write_required': True,
        'pull_request': PRODUCT_PR_NUMBER,
        'url': PRODUCT_PR_URL,
        'observed_public_head': observed_public_head,
        'desired_head': desired_head,
        'after_branch_update_required': after_branch_update_required,
        'clean_tip_verified': True,
        'observed_body_sha256': observed_body_sha256,
        'body_sha256': body_sha256,
        'body_character_count': len(body),
        'body_line_count': len(body.splitlines()),
        'body_matches_observed': (
            observed_body_sha256 == body_sha256
            if observed_body_sha256 is not None
            else None
        ),
        'body': body,
        'keep_draft_required': True,
        'description_update_authorized': False,
        'review_submission_authorized': False,
        'mark_ready_authorized': False,
        'merge_authorized': False,
        'steps': [
            first_step,
            (
                'Obtain separate authority for this exact desired head and '
                f'description SHA-256 {body_sha256}.'
            ),
            (
                'Replace only the PR description with the exact body below '
                'and keep the PR in Draft state.'
            ),
            (
                'Rerun the GET-only audit and verify the public head and '
                'description digest before starting review.'
            ),
        ],
        'verification_command': PRODUCT_PR_VERIFY_COMMAND,
        'writes_performed': False,
    }


def _product_draft_review_handoff(
    plan: dict[str, Any],
    product_draft: dict[str, Any],
) -> dict[str, Any]:
    """Build one exact, non-executing Draft review sequence."""
    exact_head = product_draft['remote_head']
    counts = (
        plan['whole_pr_path_count'],
        plan['review_phase_count'],
        plan['slice_count'],
    )
    if (
        product_draft['status'] != 'DRAFT_REVIEW_REQUIRED'
        or product_draft['head_matches_local'] is not True
        or product_draft['is_draft'] is not True
        or product_draft['mergeable'] is not True
        or product_draft['required_checks_complete'] is not True
        or not isinstance(exact_head, str)
        or SHA_PATTERN.fullmatch(exact_head) is None
        or product_draft['local_head'] != exact_head
        or plan['status'] != 'PLAN_VALID_LOCAL_ONLY'
        or plan['review_coverage_complete'] is not True
        or plan['worktree_clean'] is not True
        or plan['uncommitted_path_count'] != 0
        or any(not isinstance(count, int) or count < 1 for count in counts)
    ):
        raise G0ReadinessError(
            'product Draft review handoff requires one clean exact green tip'
        )
    return {
        'kind': 'EXACT_DRAFT_REVIEW_SEQUENCE',
        'external_write_required': False,
        'pull_request': PRODUCT_PR_NUMBER,
        'url': PRODUCT_PR_URL,
        'exact_head': exact_head,
        'whole_pr_path_count': plan['whole_pr_path_count'],
        'review_phase_count': plan['review_phase_count'],
        'slice_count': plan['slice_count'],
        'overview_command': PUBLICATION_REVIEW_OVERVIEW_COMMAND,
        'slice_command_template': PUBLICATION_REVIEW_SLICE_TEMPLATE,
        'steps': [
            'Render the exact overview and confirm a clean matching tip.',
            'Review P0, P1, then P2 using their bounded hotspots.',
            (
                'Review S1 through S7 in dependency order and run each '
                'displayed verification group.'
            ),
            (
                'Record findings separately; review submission, mark-ready, '
                'and merge remain separate GitHub decisions.'
            ),
        ],
        'commands_executed': False,
        'github_review_submitted': False,
        'mark_ready_authorized': False,
        'merge_authorized': False,
        'writes_performed': False,
    }


def _candidate_environment_summary(
    report: dict[str, Any] | None,
) -> dict[str, Any]:
    """Keep an optional live environment audit distinct from a pass."""
    if report is None:
        return {
            'status': 'NOT_CHECKED',
            'environment': 'candidate-images',
            'target_present': None,
            'required_reviewer_count': None,
            'prevent_self_review': None,
            'deployment_branch_policy': None,
            'decision_state': 'NOT_CHECKED',
            'blockers': [],
            'operator_handoff': None,
        }
    observed = report.get('observed')
    decision = report.get('decision')
    findings = report.get('findings')
    authority = report.get('authority')
    handoff = report.get('operator_handoff')
    if (
        not isinstance(observed, dict)
        or not isinstance(decision, dict)
        or not isinstance(findings, list)
        or not isinstance(authority, dict)
        or not isinstance(handoff, dict)
    ):
        raise G0ReadinessError(
            'candidate environment report is incomplete'
        )
    if decision.get('dispatch_authorized') is not False or any(
        authority.get(field) is not False
        for field in (
            'github_writes_authorized',
            'environment_writes_authorized',
            'artifact_publication_authorized',
            'remote_mutations_performed',
        )
    ):
        raise G0ReadinessError(
            'candidate environment report claims remote-write authority'
        )
    if (
        handoff.get('kind') not in CANDIDATE_HANDOFF_KINDS
        or handoff.get('authority_required')
        not in CANDIDATE_HANDOFF_AUTHORITIES
        or not isinstance(handoff.get('external_write_required'), bool)
        or handoff.get('writes_performed') is not False
        or handoff.get('verification_command')
        != CANDIDATE_ENVIRONMENT_VERIFY_COMMAND
    ):
        raise G0ReadinessError(
            'candidate environment operator handoff is unsafe'
        )
    settings_url = handoff.get('settings_url')
    if settings_url not in (None, CANDIDATE_ENVIRONMENT_SETTINGS_URL):
        raise G0ReadinessError(
            'candidate environment operator handoff has an untrusted URL'
        )
    expected_handoff = CANDIDATE_HANDOFF_BY_STATUS.get(report.get('status'))
    observed_handoff = (
        handoff.get('kind'),
        handoff.get('authority_required'),
        handoff.get('external_write_required'),
        settings_url,
    )
    if expected_handoff is None or observed_handoff != expected_handoff:
        raise G0ReadinessError(
            'candidate environment operator handoff contradicts its status'
        )
    steps = handoff.get('steps')
    if (
        not isinstance(steps, list)
        or not 2 <= len(steps) <= 5
        or len(steps) != len(set(steps))
        or not all(
            isinstance(step, str)
            and step
            and len(step) <= 300
            and '\n' not in step
            and '\r' not in step
            for step in steps
        )
    ):
        raise G0ReadinessError(
            'candidate environment operator handoff has unsafe steps'
        )
    target = observed.get('target')
    if target is not None and not isinstance(target, dict):
        raise G0ReadinessError(
            'candidate environment target must be an object or null'
        )
    blockers: list[str] = []
    for finding in findings:
        if not isinstance(finding, dict):
            raise G0ReadinessError(
                'candidate environment finding must be an object'
            )
        detail = finding.get('detail')
        if not isinstance(detail, str) or not detail:
            raise G0ReadinessError(
                'candidate environment finding has no safe detail'
            )
        blockers.append(detail)
    return {
        'status': report.get('status'),
        'environment': report.get('environment'),
        'target_present': observed.get('target_present'),
        'required_reviewer_count': (
            target.get('required_reviewer_count') if target else None
        ),
        'prevent_self_review': (
            target.get('prevent_self_review') if target else None
        ),
        'deployment_branch_policy': (
            target.get('deployment_branch_policy') if target else None
        ),
        'decision_state': decision.get('state'),
        'blockers': blockers,
        'operator_handoff': {
            'kind': handoff['kind'],
            'authority_required': handoff['authority_required'],
            'external_write_required': handoff['external_write_required'],
            'settings_url': settings_url,
            'steps': list(steps),
            'verification_command': handoff['verification_command'],
            'writes_performed': False,
        },
    }


def _identity_alternatives(published: dict[str, Any]) -> list[dict[str, str]]:
    """Describe safe identity choices without selecting or publishing one."""
    version = published['version']
    publication_status = (
        'AVAILABLE_FOR_READ_ONLY_PREFLIGHT'
        if published['status'] == 'PUBLISHED'
        else 'REQUIRES_EXTERNAL_PUBLICATION'
    )
    rebuild_status = (
        'READY_FOR_FRESH_PACKET'
        if published['status'] == 'PUBLISHED'
        else 'BLOCKED_UNTIL_PUBLISHED'
    )
    return [
        {
            'id': 'continue-current-candidate',
            'title': f'Continue the current candidate v{version}',
            'status': publication_status,
            'command': (
                'python3 scripts/check_published_release.py '
                f'--version {version} --json'
            ),
            'write_boundary': (
                'read-only preflight; release, tag, and image publication '
                'remain separate'
            ),
        },
        {
            'id': 'rebuild-against-published-version',
            'title': 'Rebuild all rows against one existing public version',
            'status': rebuild_status,
            'command': (
                'python3 scripts/check_published_release.py '
                f'--version {version} --json --require-published | '
                'python3 scripts/prepare_onboarding_matrix_packet.py '
                '--published-release-report - --render'
            ),
            'write_boundary': (
                'local plan only; run a fresh source preflight and never '
                'reuse mixed-version measurements'
            ),
        },
    ]


def _public_transition_handoff(
    published: dict[str, Any],
) -> dict[str, Any]:
    """Bind one safe audit-to-fresh-packet transition for mixed rows."""
    version = published['version']
    status_by_release = {
        'NOT_CHECKED': 'AUDIT_REQUIRED',
        'NOT_PUBLISHED': 'PUBLICATION_REQUIRED',
        'BLOCKED': 'AUDIT_BLOCKED',
        'PUBLISHED': 'READY_FOR_FRESH_MATRIX_PACKET',
    }
    release_status = published['status']
    if release_status not in status_by_release:
        raise G0ReadinessError(
            f'unsupported published release status: {release_status}'
        )
    audit_command = (
        'python3 scripts/check_g0_readiness.py '
        '--include-public-transition '
        f'--published-release-version {version} --json'
    )
    packet_command = (
        'python3 scripts/check_published_release.py '
        f'--version {version} --json --require-published | '
        'python3 scripts/prepare_onboarding_matrix_packet.py '
        '--published-release-report - --render'
    )
    return {
        'kind': 'READ_ONLY_PUBLIC_PRODUCT_TRANSITION',
        'status': status_by_release[release_status],
        'target_version': version,
        'observed_release_status': release_status,
        'audits': list(PUBLIC_TRANSITION_AUDITS),
        'audit_command': audit_command,
        'post_publication_packet_command': packet_command,
        'packet_generation_eligible': release_status == 'PUBLISHED',
        'published_identity_required': True,
        'mixed_version_measurements_reusable': False,
        'network_reads_required': True,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }


def _next_action(
    plan: dict[str, Any],
    navigation: dict[str, Any],
    routing: dict[str, Any],
    matrix: dict[str, Any],
    cohort: dict[str, Any],
    v1: dict[str, Any],
    product_draft: dict[str, Any],
    candidate_environment: dict[str, Any],
    published: dict[str, Any],
) -> dict[str, Any]:
    """Choose one safe next action in dependency order."""
    if (
        plan['status'] != 'PLAN_VALID_LOCAL_ONLY'
        or plan['review_coverage_complete'] is not True
    ):
        return {
            'id': 'repair-publication-plan',
            'title': 'Repair the local publication inventory',
            'reason': (
                'The exact candidate path plan or composed whole-PR review '
                'coverage is not valid.'
            ),
            'command': (
                'python3 scripts/check_publication_slice_plan.py --json'
            ),
            'write_boundary': 'read-only',
        }
    if (
        product_draft['status'] == 'NOT_CHECKED'
        and candidate_environment['status'] != 'NOT_CHECKED'
    ):
        return {
            'id': 'inspect-product-draft',
            'title': 'Inspect the product Draft before repository settings',
            'reason': (
                'The candidate environment was inspected before the product '
                'PR merge state was established.'
            ),
            'command': PRODUCT_PR_VERIFY_COMMAND,
            'write_boundary': (
                'GitHub GETs only; marking ready, merging, settings changes, '
                'and E2 dispatch remain separate decisions'
            ),
        }
    if product_draft['status'] == 'BLOCKED':
        blocker = '; '.join(product_draft['blockers']) or (
            'The exact product Draft state could not be established.'
        )
        if (
            product_draft['head_matches_local'] is False
            and product_draft['non_force_update_possible'] is True
        ):
            local_head = product_draft['local_head']
            remote_head = product_draft['remote_head']
            if (
                plan['worktree_clean'] is not True
                or plan['uncommitted_path_count'] != 0
            ):
                return {
                    'id': 'restore-clean-draft-update-worktree',
                    'title': (
                        'Restore a clean worktree before preparing the Draft '
                        'update'
                    ),
                    'reason': (
                        'The local publication plan has '
                        f"{plan['uncommitted_path_count']} uncommitted paths, "
                        'so an exact branch-and-description handoff cannot '
                        f'be bound to local tip {local_head}.'
                    ),
                    'command': 'git status --short',
                    'write_boundary': (
                        'read-only local inspection; no cleanup, commit, '
                        'push, PR edit, mark-ready, or merge is authorized'
                    ),
                }
            return {
                'id': 'review-product-draft-branch-update',
                'title': 'Review the exact non-force Draft branch update',
                'reason': (
                    f'Public head {remote_head} differs from local tip '
                    f'{local_head}; local ancestry proves that this exact '
                    'transition is a fast-forward.'
                ),
                'command': (
                    f'git merge-base --is-ancestor {remote_head} '
                    f'{local_head}'
                ),
                'write_boundary': (
                    'read-only ancestry check; push requires separate '
                    'exact-tip authority and force push is forbidden'
                ),
                'product_draft_update_handoff': (
                    _product_draft_update_handoff(product_draft)
                ),
                'product_draft_description_refresh_handoff': (
                    _product_draft_description_refresh_handoff(
                        plan,
                        navigation,
                        routing,
                        matrix,
                        cohort,
                        v1,
                        product_draft,
                    )
                ),
            }
        if product_draft['head_matches_local'] is False:
            local_head = product_draft['local_head']
            remote_head = product_draft['remote_head']
            if product_draft['non_force_update_possible'] is False:
                return {
                    'id': 'inspect-product-draft-divergence',
                    'title': 'Inspect the divergent Draft branch history',
                    'reason': (
                        f'Public head {remote_head} is not an ancestor of '
                        f'local tip {local_head}; a non-force update is not '
                        'currently possible.'
                    ),
                    'command': f'git merge-base {remote_head} {local_head}',
                    'write_boundary': (
                        'read-only history inspection; no push, force push, '
                        'PR state change, or merge is authorized'
                    ),
                }
            return {
                'id': 'restore-product-draft-lineage-evidence',
                'title': 'Restore local evidence for Draft branch lineage',
                'reason': (
                    f'Public head {remote_head} and local tip {local_head} '
                    'differ, but the local object database cannot yet prove '
                    'whether a non-force update is possible.'
                ),
                'command': (
                    f'git fetch --no-tags {PRODUCT_REPOSITORY_URL} '
                    f'{PRODUCT_PR_HEAD} && '
                    f'git merge-base --is-ancestor {remote_head} {local_head}'
                ),
                'write_boundary': (
                    'network read and local Git metadata only; no remote '
                    'write, PR state change, or merge is authorized'
                ),
            }
        return {
            'id': 'repair-product-draft-audit',
            'title': 'Restore an exact product Draft audit',
            'reason': blocker,
            'command': PRODUCT_PR_VERIFY_COMMAND,
            'write_boundary': (
                'GitHub GETs only; no PR state change or merge is authorized'
            ),
        }
    if product_draft['status'] == 'DRAFT_REVIEW_REQUIRED':
        if (
            plan['worktree_clean'] is not True
            or plan['uncommitted_path_count'] != 0
        ):
            return {
                'id': 'restore-clean-draft-review-worktree',
                'title': (
                    'Restore a clean worktree before exact Draft review'
                ),
                'reason': (
                    'The local publication plan has '
                    f"{plan['uncommitted_path_count']} uncommitted paths, "
                    'so its rendered review budget would not describe exact '
                    f"public head {product_draft['remote_head']}."
                ),
                'command': 'git status --short',
                'write_boundary': (
                    'read-only local inspection; no file cleanup, commit, '
                    'push, review submission, mark-ready, or merge is '
                    'authorized'
                ),
            }
        description_handoff = (
            _product_draft_description_refresh_handoff(
                plan,
                navigation,
                routing,
                matrix,
                cohort,
                v1,
                product_draft,
            )
        )
        if description_handoff['body_matches_observed'] is not True:
            return {
                'id': 'review-product-draft-description-refresh',
                'title': 'Review the exact Draft description refresh',
                'reason': (
                    f"PR #{product_draft['pull_request']} is exact and green "
                    f"at {product_draft['remote_head']}, but its observed "
                    'description digest does not match the clean local '
                    'review packet.'
                ),
                'command': PRODUCT_PR_VERIFY_COMMAND,
                'write_boundary': (
                    'GitHub GETs and local rendering only; editing the PR '
                    'description requires separate exact-tip authority and '
                    'the PR must remain Draft'
                ),
                'product_draft_description_refresh_handoff': (
                    description_handoff
                ),
            }
        return {
            'id': 'review-product-draft',
            'title': 'Review the exact Draft from one bounded overview',
            'reason': (
                f"PR #{product_draft['pull_request']} is an exact, mergeable "
                f"Draft at {product_draft['remote_head']} with "
                f"{product_draft['passing_check_count']} passing checks and "
                f"{product_draft['skipped_check_count']} intentional skips; "
                f"the local plan covers {plan['whole_pr_path_count']} paths "
                f"in {plan['review_phase_count']} phases and "
                f"{plan['slice_count']} slices."
            ),
            'command': PUBLICATION_REVIEW_OVERVIEW_COMMAND,
            'write_boundary': (
                'read-only local review cards; review submission, marking '
                'ready and merging remain separate GitHub decisions'
            ),
            'product_draft_review_handoff': (
                _product_draft_review_handoff(plan, product_draft)
            ),
        }
    if (
        product_draft['status']
        == 'READY_FOR_SEPARATE_MERGE_REVIEW'
    ):
        return {
            'id': 'review-product-merge',
            'title': 'Review the exact PR merge separately',
            'reason': (
                f"PR #{product_draft['pull_request']} is no longer Draft, "
                'but the read-only audit cannot authorize or perform a merge.'
            ),
            'command': PRODUCT_PR_VERIFY_COMMAND,
            'write_boundary': (
                'GitHub GETs only; merge remains a separate maintainer '
                'decision'
            ),
        }
    if (
        not matrix['product_version_aligned']
        and published['status'] != 'PUBLISHED'
        and candidate_environment['status'] not in ('NOT_CHECKED', 'READY')
    ):
        blocker = '; '.join(candidate_environment['blockers']) or (
            'The protected candidate environment is not ready.'
        )
        return {
            'id': 'review-candidate-environment',
            'title': 'Review the protected candidate environment',
            'reason': (
                f"Environment state is {candidate_environment['status']}: "
                f'{blocker}'
            ),
            'command': (
                'python3 scripts/check_candidate_environment.py '
                '--json --require-ready'
            ),
            'write_boundary': (
                'read-only audit; repository settings and E2 dispatch remain '
                'separate decisions'
            ),
            'operator_handoff': candidate_environment['operator_handoff'],
        }
    if not matrix['product_version_aligned']:
        versions = ', '.join(matrix['product_versions']) or 'multiple versions'
        transition = _public_transition_handoff(published)
        packet_ready = transition['packet_generation_eligible']
        if packet_ready:
            title = 'Prepare one fresh same-version matrix packet'
            command = transition['post_publication_packet_command']
            write_boundary = (
                'local fresh-row plan only; trial execution and evidence '
                'replacement remain separate'
            )
        else:
            title = 'Resolve one public product version before measuring'
            command = transition['audit_command']
            write_boundary = (
                'read-only complete transition audit; branch, environment, '
                'release, tag, and image writes remain separate'
            )
        return {
            'id': 'align-public-product-version',
            'title': title,
            'reason': (
                f'The reviewed rows use {versions}; do not attach human '
                'measurements to mixed-version rows. The target publication '
                f'audit is currently {published["status"]}.'
            ),
            'command': command,
            'alternatives': _identity_alternatives(published),
            'public_transition_handoff': transition,
            'write_boundary': write_boundary,
        }
    if not matrix['activation_gate']:
        reason = '; '.join(matrix['actions']) or (
            'The Docker/source onboarding matrix has not reached its '
            'activation gate.'
        )
        return {
            'id': 'complete-comparable-onboarding',
            'title': 'Complete same-version measured Docker/source rows',
            'reason': reason,
            'command': (
                'python3 scripts/check_onboarding_trial_matrix.py --json'
            ),
            'write_boundary': (
                'read-only audit; trial execution remains separate'
            ),
        }
    if cohort['launch_status'] != 'READY_FOR_NEXT_ATTEMPT':
        return {
            'id': 'review-cohort-launch-gates',
            'title': 'Review the independent first-map cohort launch gates',
            'reason': '; '.join(
                f'{gate}: {_cohort_gate_guidance(gate)}'
                for gate in cohort['pending_launch_gates']
            ) or (
                f"Cohort state is {cohort['status']}."
            ),
            'command': 'python3 scripts/first_map_validator_cohort.py --json',
            'write_boundary': (
                'read-only; community recruitment remains unauthorized'
            ),
        }
    if v1['status'] != 'READY':
        return {
            'id': 'close-v1-readiness',
            'title': 'Resolve the remaining v1 readiness gates',
            'reason': ', '.join(v1['incomplete_gates']) or (
                f"v1 readiness is {v1['status']}."
            ),
            'command': 'python3 scripts/check_v1_readiness.py --json',
            'write_boundary': (
                'read-only audit; release and adoption decisions remain '
                'separate'
            ),
        }
    if published['status'] != 'PUBLISHED':
        return {
            'id': 'inspect-release-publication',
            'title': 'Inspect or decide the stable release publication gate',
            'reason': (
                'The published-release state is '
                f"{published['status']}; no publication is implied."
            ),
            'command': (
                'python3 scripts/check_published_release.py '
                f"--version {published['version']} --json"
            ),
            'write_boundary': (
                'read-only audit; tag/release/image publication is separate'
            ),
        }
    return {
        'id': 'review-external-gates',
        'title': 'Review the external G0 transition packet',
        'reason': 'Local gates are ready for a separate maintainer decision.',
        'command': (
            'python3 scripts/check_g0_readiness.py '
            '--include-published-release'
        ),
        'write_boundary': 'read-only; no GitHub or community mutation',
    }


def build_report(
    reports: dict[str, dict[str, Any] | None],
    *,
    published_release_version: str = DEFAULT_RELEASE_VERSION,
) -> dict[str, Any]:
    """Build and schema-validate a stable, privacy-safe dashboard report."""
    plan_report = reports.get('publication_plan')
    overview_report = reports.get('publication_overview')
    routing_report = reports.get('product_draft_review_routing')
    matrix_report = reports.get('onboarding_matrix')
    cohort_report = reports.get('first_map_cohort')
    v1_report = reports.get('v1_readiness')
    if not all(
        isinstance(item, dict)
        for item in (
            plan_report,
            overview_report,
            routing_report,
            matrix_report,
            cohort_report,
            v1_report,
        )
    ):
        raise G0ReadinessError(
            'the six local G0 checker reports are required'
        )

    plan = {
        'status': plan_report.get('status'),
        'local_tip_sha': plan_report.get('local_tip_sha'),
        'whole_pr_commit_count': plan_report.get('whole_pr_commit_count'),
        'follow_up_review_commit_count': plan_report.get(
            'follow_up_review_commit_count'
        ),
        'path_count': plan_report.get('path_count'),
        'slice_count': plan_report.get('slice_count'),
        'whole_pr_path_count': plan_report.get('whole_pr_path_count'),
        'review_phase_count': plan_report.get('review_phase_count'),
        'review_coverage_complete': plan_report.get(
            'review_coverage_complete'
        ),
        'bridge_path_count': plan_report.get('bridge_path_count'),
        'worktree_clean': plan_report.get('worktree_clean'),
        'uncommitted_path_count': plan_report.get('uncommitted_path_count'),
    }
    navigation = _publication_review_navigation_summary(
        overview_report,
        plan,
    )
    routing = _review_routing_summary(
        routing_report,
        plan,
        navigation,
    )
    review_ledger = _review_ledger_summary(
        reports.get('product_draft_review_ledger'),
        plan,
        routing,
    )
    matrix = _matrix_summary(matrix_report)
    cohort = _cohort_summary(cohort_report)
    v1 = _v1_summary(v1_report)
    published = _published_summary(
        reports.get('published_release'),
        published_release_version,
    )
    candidate_environment = _candidate_environment_summary(
        reports.get('candidate_environment')
    )
    product_draft = _product_draft_summary(reports.get('product_draft'))
    authority = {
        'network_reads_performed': (
            reports.get('published_release') is not None
            or reports.get('candidate_environment') is not None
            or reports.get('product_draft') is not None
        ),
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }
    local_error = (
        plan['status'] != 'PLAN_VALID_LOCAL_ONLY'
        or plan['review_coverage_complete'] is not True
    )
    local_ready = (
        not local_error
        and bool(matrix['activation_gate'])
        and cohort['launch_status'] == 'READY_FOR_NEXT_ATTEMPT'
        and v1['status'] == 'READY'
        and published['status'] == 'PUBLISHED'
        and product_draft['status'] in ('NOT_CHECKED', 'MERGED')
    )
    status = 'READY_FOR_REVIEW' if local_ready else 'HOLD'
    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'repository': REPOSITORY,
        'scope': 'local-g0-readiness',
        'status': status,
        'authority': authority,
        'current_packet': {
            'path': CURRENT_PACKET,
            'supersedes_historical_snapshot': True,
        },
        'checks': {
            'publication_plan': plan,
            'publication_review_navigation': navigation,
            'product_draft_review_routing': routing,
            'product_draft_review_ledger': review_ledger,
            'onboarding_matrix': matrix,
            'first_map_cohort': cohort,
            'v1_readiness': v1,
            'product_draft': product_draft,
            'candidate_environment': candidate_environment,
            'published_release': published,
        },
        'next_action': _next_action(
            plan,
            navigation,
            routing,
            matrix,
            cohort,
            v1,
            product_draft,
            candidate_environment,
            published,
        ),
    }
    try:
        schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(schema).validate(report)
    except (OSError, json.JSONDecodeError, jsonschema.SchemaError) as exc:
        raise G0ReadinessError(
            f'dashboard schema cannot be loaded: {exc}'
        ) from exc
    except jsonschema.ValidationError as exc:
        location = '.'.join(str(item) for item in exc.absolute_path)
        raise G0ReadinessError(
            f'dashboard schema failed at {location or "<root>"}: {exc.message}'
        ) from exc
    return report


def render_card(report: dict[str, Any]) -> str:
    """Render one concise card with exactly one next action."""
    checks = report['checks']
    plan = checks['publication_plan']
    navigation = checks['publication_review_navigation']
    routing = checks['product_draft_review_routing']
    review_ledger = checks['product_draft_review_ledger']
    matrix = checks['onboarding_matrix']
    cohort = checks['first_map_cohort']
    v1 = checks['v1_readiness']
    product_draft = checks['product_draft']
    candidate_environment = checks['candidate_environment']
    published = checks['published_release']
    lines = [
        '# G0 readiness',
        '',
        f"- Overall: **{report['status']}**",
        '- Scope: local, read-only',
        (
            '- Network reads: **yes**'
            if report['authority']['network_reads_performed']
            else '- Network reads: **no**'
        ),
        '- GitHub/community writes: **no**',
        '',
        '| Gate | Status | Evidence summary |',
        '| --- | --- | --- |',
        (
            f"| publication plan | {plan['status']} | "
            f"{plan['path_count']} paths / {plan['slice_count']} slices; "
            f"whole PR {plan['whole_pr_path_count']} paths / "
            f"{plan['review_phase_count']} phases; "
            'review coverage complete: '
            f"{str(plan['review_coverage_complete']).lower()}; "
            f"worktree clean: {str(plan['worktree_clean']).lower()} |"
        ),
        (
            f"| review navigation | {navigation['status']} | "
            f"{navigation['phase_count']} exact compare links / "
            f"{navigation['slice_count']} bounded slices; "
            'commands executed: false |'
        ),
        (
            f"| review roles | {routing['status']} | "
            f"{routing['summary']['lane_count']} capability lanes / "
            'advisory target 2; identities: none; reviewer requests: false |'
        ),
        (
            f"| review ledger | {review_ledger['status']} | "
            f"{review_ledger['passing_lane_count']} pass / "
            f"{review_ledger['blocked_lane_count']} blocked / "
            f"{review_ledger['open_blocker_count']} open blockers; "
            'identities: none; GitHub review submitted: false |'
        ),
        (
            f"| onboarding matrix | {matrix['status']} | "
            f"{matrix['present_rows']}/4 present, "
            f"{matrix['comparable_rows']}/4 comparable, "
            f"activation: {str(matrix['activation_gate']).lower()} |"
        ),
        (
            f"| first-map cohort | {cohort['launch_status']} | "
            f"accepted {cohort['accepted_validations']}/"
            f"{cohort['accepted_target']}; "
            f"attempts {cohort['attempt_count']} |"
        ),
        (
            f"| v1 readiness | {v1['status']} | "
            f"{v1['complete']}/{v1['total']} complete |"
        ),
        (
            f"| product Draft PR #{product_draft['pull_request']} | "
            f"{product_draft['status']} | "
            f"head match: {str(product_draft['head_matches_local']).lower()}; "
            f"checks {product_draft['passing_check_count']} pass / "
            f"{product_draft['skipped_check_count']} skip / "
            f"{product_draft['failing_check_count']} fail; "
            'merge authorized: false |'
        ),
        (
            '| candidate environment | '
            f"{candidate_environment['status']} | "
            f"{candidate_environment['environment']}; "
            'dispatch authorized: false |'
        ),
        (
            f"| published release | {published['status']} | "
            f"v{published['version']} |"
        ),
    ]
    if v1['incomplete_gate_details']:
        lines.extend(['', 'v1 blockers:'])
        for gate in v1['incomplete_gate_details']:
            lines.append(
                f"- {gate['title']} (`{gate['id']}`): {gate['detail']}"
            )
            lines.extend(f'  - {blocker}' for blocker in gate['blockers'])
    if cohort['pending_launch_gates']:
        lines.extend(['', 'first-map cohort blockers:'])
        lines.extend(
            f'- `{gate}` — {_cohort_gate_guidance(gate)}'
            for gate in cohort['pending_launch_gates']
        )
    lines.extend([
        '',
        'Next action:',
        f"{report['next_action']['title']}",
        f"Reason: {report['next_action']['reason']}",
        f"Command: `{report['next_action']['command']}`",
        f"Boundary: {report['next_action']['write_boundary']}",
    ])
    alternatives = report['next_action'].get('alternatives', [])
    if alternatives:
        lines.extend(['', 'Choices (no write):'])
        for alternative in alternatives:
            lines.extend([
                f"- {alternative['title']} — **{alternative['status']}**",
                f"  Command: `{alternative['command']}`",
                f"  Boundary: {alternative['write_boundary']}",
            ])
    transition = report['next_action'].get('public_transition_handoff')
    if transition is not None:
        lines.extend([
            '',
            'Public product transition (not executed):',
            f"- Status: **{transition['status']}**",
            f"- Target: `v{transition['target_version']}`",
            '- Read together: '
            + ', '.join(
                f'`{audit}`' for audit in transition['audits']
            ),
            '- Fresh matrix packet eligible: '
            + (
                'yes'
                if transition['packet_generation_eligible'] else 'no'
            ),
            f"- Complete audit: `{transition['audit_command']}`",
            '- After publication: '
            f"`{transition['post_publication_packet_command']}`",
            '- Mixed-version measurements reusable: no',
            '- GitHub writes performed or authorized: no',
        ])
    handoff = report['next_action'].get('operator_handoff')
    if handoff is not None:
        lines.extend([
            '',
            'Operator handoff (not executed):',
            f"- Authority required: {handoff['authority_required']}",
        ])
        if handoff['settings_url'] is not None:
            lines.append(f"- Settings: {handoff['settings_url']}")
        lines.extend(
            f'{index}. {step}'
            for index, step in enumerate(handoff['steps'], start=1)
        )
        lines.append(
            'Read-only verification: '
            f"`{handoff['verification_command']}`"
        )
        lines.append('- Environment writes performed: no')
    draft_handoff = report['next_action'].get(
        'product_draft_update_handoff'
    )
    if draft_handoff is not None:
        lines.extend([
            '',
            'Draft branch update handoff (not executed):',
            f"- Authority required: {draft_handoff['authority_required']}",
            f"- Repository: `{draft_handoff['repository_url']}`",
            f"- Head branch: `{draft_handoff['head_ref']}`",
            f"- Public head: `{draft_handoff['public_head']}`",
            f"- Local tip: `{draft_handoff['local_head']}`",
            '- Fast-forward verified: yes',
            '- Non-force only: yes',
        ])
        lines.extend(
            f'{index}. {step}'
            for index, step in enumerate(draft_handoff['steps'], start=1)
        )
        lines.append(
            'Post-update GET-only verification: '
            f"`{draft_handoff['verification_command']}`"
        )
        lines.append('- Pushes performed: no')
    description_handoff = report['next_action'].get(
        'product_draft_description_refresh_handoff'
    )
    if description_handoff is not None:
        observed_digest = (
            description_handoff['observed_body_sha256'] or 'not-observed'
        )
        lines.extend([
            '',
            'Draft description refresh handoff (not executed):',
            (
                '- Authority required: '
                f"{description_handoff['authority_required']}"
            ),
            (
                '- Observed public head: '
                f"`{description_handoff['observed_public_head']}`"
            ),
            f"- Desired head: `{description_handoff['desired_head']}`",
            f'- Observed body SHA-256: `{observed_digest}`',
            (
                '- Desired body SHA-256: '
                f"`{description_handoff['body_sha256']}`"
            ),
            (
                '- Desired body size: '
                f"{description_handoff['body_character_count']} characters / "
                f"{description_handoff['body_line_count']} lines"
            ),
            (
                '- Branch update required first: '
                f"{'yes' if description_handoff['after_branch_update_required'] else 'no'}"
            ),
        ])
        lines.extend(
            f'{index}. {step}'
            for index, step in enumerate(
                description_handoff['steps'], start=1
            )
        )
        lines.extend([
            'Exact desired PR description:',
            '```markdown',
            description_handoff['body'],
            '```',
            (
                'Post-refresh GET-only verification: '
                f"`{description_handoff['verification_command']}`"
            ),
            '- Description updates performed: no',
            '- Mark-ready authorized: no',
            '- Merge authorized: no',
        ])
    review_handoff = report['next_action'].get(
        'product_draft_review_handoff'
    )
    if review_handoff is not None:
        lines.extend([
            '',
            'Draft review sequence (not executed):',
            f"- Exact head: `{review_handoff['exact_head']}`",
            (
                '- Coverage: '
                f"{review_handoff['whole_pr_path_count']} paths / "
                f"{review_handoff['review_phase_count']} phases / "
                f"{review_handoff['slice_count']} slices"
            ),
            f"- Overview: `{review_handoff['overview_command']}`",
            (
                '- Slice template: '
                f"`{review_handoff['slice_command_template']}`"
            ),
        ])
        lines.extend(
            f'{index}. {step}'
            for index, step in enumerate(
                review_handoff['steps'], start=1
            )
        )
        lines.extend([
            '- Commands executed: no',
            '- GitHub review submitted: no',
            '- Mark-ready authorized: no',
            '- Merge authorized: no',
        ])
    lines.extend(['', f"Current packet: `{report['current_packet']['path']}`"])
    return '\n'.join(lines) + '\n'


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse dashboard CLI options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json', action='store_true')
    parser.add_argument(
        '--product-draft-review-ledger',
        type=Path,
        help=(
            'Also validate one local anonymous exact-tip review ledger; '
            'the file path is not retained in the report.'
        ),
    )
    parser.add_argument(
        '--include-public-transition',
        action='store_true',
        help=(
            'Run the read-only product Draft, protected environment, and '
            'published-release audits together in dependency order.'
        ),
    )
    parser.add_argument(
        '--include-product-draft',
        action='store_true',
        help='Also run the read-only exact-head Draft PR audit.',
    )
    parser.add_argument(
        '--include-candidate-environment',
        action='store_true',
        help='Also run the read-only protected-environment audit.',
    )
    parser.add_argument(
        '--include-published-release',
        action='store_true',
        help='Also run the read-only remote v0.9.1 publication audit.',
    )
    parser.add_argument(
        '--published-release-version',
        default=DEFAULT_RELEASE_VERSION,
    )
    parser.add_argument(
        '--require-ready',
        action='store_true',
        help='Exit 1 unless all summarized gates are ready for review.',
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Run the dashboard without performing any writes."""
    args = parse_args(argv)
    try:
        reports = collect_checker_reports(
            product_draft_review_ledger=(
                args.product_draft_review_ledger.expanduser().resolve()
                if args.product_draft_review_ledger is not None
                else None
            ),
            include_product_draft=(
                args.include_product_draft
                or args.include_public_transition
            ),
            include_candidate_environment=(
                args.include_candidate_environment
                or args.include_public_transition
            ),
            include_published_release=(
                args.include_published_release
                or args.include_public_transition
            ),
            published_release_version=args.published_release_version,
        )
        report = build_report(
            reports,
            published_release_version=args.published_release_version,
        )
    except (G0ReadinessError, OSError) as exc:
        print(f'G0 readiness audit error: {exc}', file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(render_card(report), end='')
    if args.require_ready and report['status'] != 'READY_FOR_REVIEW':
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
