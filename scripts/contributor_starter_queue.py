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

"""Inspect and verify the bounded local contributor starter queue."""

from __future__ import annotations

import argparse
import base64
import binascii
import hashlib
import json
import os
import re
import shlex
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any, Sequence

import jsonschema


sys.dont_write_bytecode = True

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_QUEUE = (
    REPO_ROOT / 'docs' / 'contracts' / 'contributor-starter-queue-v1.json'
)
DEFAULT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'contributor-starter-queue-v1.schema.json'
)
DEFAULT_NEXT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'contributor-next-action-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/contributor-starter-queue-v1.schema.json'
)
NEXT_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/contributor-next-action-v1.schema.json'
)
EXPECTED_TASK_IDS = tuple(f'starter-C{index}' for index in range(5, 10))
COMMON_LABELS = {'good first issue', 'help wanted'}
DOMAIN_LABELS = {'bug', 'documentation'}

DOCS_STRICT_CHECKS = (
    ('python3', '-m', 'mkdocs', 'build', '--strict'),
)
MID360_EMPTY_FRAME_CHECKS = (
    (
        'python3',
        '-m',
        'pytest',
        '-q',
        'graph_based_slam/test/test_mid360_robot_tools.py',
        '-k',
        'frame',
    ),
    (
        'python3',
        '-m',
        'flake8',
        '--select=E9,F63,F7,F82',
        'scripts/lidarslam_tools/mid360_preflight.py',
        'graph_based_slam/test/test_mid360_robot_tools.py',
    ),
    ('git', 'diff', '--check'),
)
PROFILE_CHECKS = {
    'docs-strict': DOCS_STRICT_CHECKS,
    'mid360-empty-frame': MID360_EMPTY_FRAME_CHECKS,
}
GITHUB_PAGE_SIZE = 100
GITHUB_MAX_PAGES = 20
GITHUB_REPOSITORY_PATTERN = re.compile(
    r'^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$'
)
PUBLICATION_GATE_COMMANDS = {
    'first-map-validator-cohort-v1': (
        'python3',
        'scripts/first_map_validator_cohort.py',
        '--json',
    ),
}
LOCAL_TASK_PUBLICATION_RECHECK_COMMAND = [
    'python3',
    'scripts/contributor_starter_queue.py',
    '--next',
    '--json',
]


class QueueError(ValueError):
    """The local starter queue cannot be trusted."""


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise QueueError(f'cannot read {label} {path}: {exc}') from exc
    if not isinstance(payload, dict):
        raise QueueError(f'{label} must be a JSON object')
    return payload


def _schema_error_path(error: jsonschema.ValidationError) -> str:
    path = '.'.join(str(item) for item in error.absolute_path)
    return path or '<root>'


def _validate_repo_path(path: str) -> None:
    candidate = PurePosixPath(path)
    if not path or path.startswith('/') or '\\' in path:
        raise QueueError(f'invalid repository-relative path: {path!r}')
    if '\n' in path or '\r' in path or '..' in candidate.parts:
        raise QueueError(f'unsafe repository-relative path: {path!r}')
    if str(candidate) != path or path.startswith('./'):
        raise QueueError(f'non-canonical repository-relative path: {path!r}')


def _declared_checks(task: dict[str, Any]) -> tuple[tuple[str, ...], ...]:
    return tuple(tuple(item['argv']) for item in task['focused_checks'])


def _validate_task(task: dict[str, Any], repo_root: Path) -> None:
    task_id = task['id']
    if task['labels'] != sorted(task['labels']):
        raise QueueError(f'{task_id} labels must be sorted')
    labels = set(task['labels'])
    if not COMMON_LABELS.issubset(labels):
        raise QueueError(f'{task_id} is missing required starter labels')
    domain = labels - COMMON_LABELS
    if len(domain) != 1 or not domain.issubset(DOMAIN_LABELS):
        raise QueueError(f'{task_id} must have exactly one domain label')

    if task['source_issue_numbers'] != sorted(task['source_issue_numbers']):
        raise QueueError(f'{task_id} source issue numbers must be sorted')
    if task['allowed_paths'] != sorted(task['allowed_paths']):
        raise QueueError(f'{task_id} allowed paths must be sorted')

    for path in task['allowed_paths']:
        _validate_repo_path(path)
        candidate = repo_root / path
        if not candidate.is_file() or candidate.is_symlink():
            raise QueueError(
                f'{task_id} allowed path is not a regular file: {path}')

    profile = task['check_profile']
    expected_checks = PROFILE_CHECKS.get(profile)
    if expected_checks is None:
        raise QueueError(f'{task_id} uses unsupported check profile {profile}')
    if _declared_checks(task) != expected_checks:
        raise QueueError(
            f'{task_id} focused checks do not match profile {profile}')

    check_ids = [item['id'] for item in task['focused_checks']]
    if len(check_ids) != len(set(check_ids)):
        raise QueueError(f'{task_id} focused check ids contain duplicates')
    for command in expected_checks:
        if any('\x00' in argument or '\n' in argument for argument in command):
            raise QueueError(f'{task_id} focused check contains unsafe text')

    allowed = set(task['allowed_paths'])
    for probe in task['gap_probes']:
        _validate_repo_path(probe['path'])
        if probe['path'] not in allowed:
            raise QueueError(
                f'{task_id} gap probe is outside its path scope: '
                f"{probe['path']}")


def _task_readiness(task: dict[str, Any], repo_root: Path) -> dict[str, Any]:
    failures = []
    contents: dict[str, str] = {}
    for probe in task['gap_probes']:
        path = probe['path']
        if path not in contents:
            contents[path] = (repo_root / path).read_text(encoding='utf-8')
        present = probe['value'] in contents[path]
        passed = present if probe['mode'] == 'contains' else not present
        if not passed:
            failures.append(probe['failure_detail'])
    return {
        'id': task['id'],
        'status': 'READY' if not failures else 'STALE',
        'reasons': failures,
    }


def _validate_published_issue_dependencies(
    dependencies: Sequence[dict[str, Any]],
) -> None:
    """Keep every executable publication gate on a fixed local allowlist."""
    ids = [item['id'] for item in dependencies]
    if len(ids) != len(set(ids)):
        raise QueueError('published issue dependency ids must be unique')
    identities = [
        (item['issue_number'], item['issue_title'].casefold())
        for item in dependencies
    ]
    if len(identities) != len(set(identities)):
        raise QueueError(
            'published issue dependency identities must be unique')
    for dependency in dependencies:
        expected = PUBLICATION_GATE_COMMANDS.get(dependency['id'])
        command = tuple(dependency['check_command'])
        if expected is None or command != expected:
            raise QueueError(
                f"{dependency['id']} uses an unsupported gate command")


def validate_queue(
    queue: dict[str, Any],
    schema: dict[str, Any],
    repo_root: Path = REPO_ROOT,
) -> dict[str, Any]:
    """Validate schema, scope, authority, duplicate audit, and local drift."""
    validator = jsonschema.Draft7Validator(schema)
    errors = sorted(
        validator.iter_errors(queue),
        key=lambda item: [str(part) for part in item.absolute_path],
    )
    if errors:
        first = errors[0]
        raise QueueError(
            f'schema validation failed at {_schema_error_path(first)}: '
            f'{first.message}')
    if queue['schema_uri'] != SCHEMA_URI:
        raise QueueError('queue schema_uri is not the supported v1 URI')

    tasks = queue['tasks']
    task_ids = tuple(task['id'] for task in tasks)
    if task_ids != EXPECTED_TASK_IDS:
        raise QueueError(
            'task ids must be the ordered bounded set '
            f'{list(EXPECTED_TASK_IDS)}')
    for task in tasks:
        _validate_task(task, repo_root)

    _validate_published_issue_dependencies(
        queue['published_issue_dependencies'])
    expected_local_dependency = {
        'id': 'product-draft-public-queue-v1',
        'pull_request_number': 427,
        'base_ref': 'develop',
        'public_queue_path': (
            'docs/contracts/contributor-starter-queue-v1.json'
        ),
    }
    if queue['local_task_publication_dependency'] != (
        expected_local_dependency
    ):
        raise QueueError(
            'local task publication dependency must bind PR #427, develop, '
            'and the canonical queue path'
        )

    audit = queue['remote_duplicate_audit']
    if audit['open_pull_request_count'] != len(
        audit['open_pull_request_numbers']
    ):
        raise QueueError(
            'open pull request count does not match its inventory')
    audited_ids = tuple(item['task_id'] for item in audit['task_matches'])
    if audited_ids != EXPECTED_TASK_IDS:
        raise QueueError('duplicate audit must cover the ordered task set')
    duplicate_matches = {
        item['task_id']: item['matching_open_pull_requests']
        for item in audit['task_matches']
        if item['matching_open_pull_requests']
    }
    if duplicate_matches:
        raise QueueError(
            'open pull request duplicates require review: '
            f'{duplicate_matches}')

    authority = queue['authority']
    if any((
        authority['github_writes_authorized'],
        authority['issues_published'],
        authority['remote_mutations_performed'],
    )):
        raise QueueError(
            'a local starter queue cannot authorize GitHub writes')

    readiness = [_task_readiness(task, repo_root) for task in tasks]
    stale = [item for item in readiness if item['status'] == 'STALE']
    return {
        'status': (
            'QUEUE_READY_LOCAL_ONLY'
            if not stale else 'QUEUE_STALE_LOCAL_ONLY'
        ),
        'queue_id': queue['queue_id'],
        'publication_status': queue['status'],
        'task_count': len(tasks),
        'ready_task_ids': [
            item['id'] for item in readiness if item['status'] == 'READY'
        ],
        'stale_tasks': stale,
        'remote_duplicate_audit': {
            'checked_at': audit['checked_at'],
            'open_pull_request_count': audit['open_pull_request_count'],
            'matching_task_pull_requests': 0,
            'recheck_before_publication': True,
        },
        'authority': {
            'github_writes_authorized': False,
            'issues_published': False,
            'remote_mutations_performed': False,
        },
    }


def evaluate(
    queue_path: Path = DEFAULT_QUEUE,
    schema_path: Path = DEFAULT_SCHEMA,
    repo_root: Path = REPO_ROOT,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Load and validate the queue, returning the source and report."""
    queue = _load_json(queue_path, 'queue')
    schema = _load_json(schema_path, 'schema')
    return queue, validate_queue(queue, schema, repo_root)


def _find_task(queue: dict[str, Any], task_id: str) -> dict[str, Any]:
    for task in queue['tasks']:
        if task['id'] == task_id:
            return task
    raise QueueError(f'unknown starter task: {task_id}')


def _require_render_ready(
    report: dict[str, Any],
    task_id: str,
) -> None:
    stale = {
        item['id']: item['reasons'] for item in report['stale_tasks']
    }
    if task_id in stale:
        raise QueueError(
            f'{task_id} is stale and must be reviewed before rendering: '
            f"{'; '.join(stale[task_id])}")


def _bullet_lines(items: Sequence[str]) -> list[str]:
    return [f'- {item}' for item in items]


def render_task_markdown(task: dict[str, Any]) -> str:
    """Render one copy-ready issue body without publishing it."""
    issue_links = ', '.join(
        f'#{number}' for number in task['source_issue_numbers'])
    lines = [
        f"# {task['title']}",
        '',
        '> Coordination gate: this task is prepared locally but is not a '
        'published GitHub issue. Start only after a maintainer publishes the '
        'issue or confirms that it is still unclaimed.',
        '',
        '## Outcome',
        '',
        task['outcome'],
        '',
        f"Estimate in a prepared environment: **{task['estimate_minutes']} "
        'minutes**.',
        '',
        f'Related support issues: {issue_links}.',
        '',
        '## Allowed files',
        '',
        *_bullet_lines([f'`{path}`' for path in task['allowed_paths']]),
        '',
        '## Acceptance',
        '',
        *_bullet_lines(task['acceptance']),
        '',
        '## Non-goals',
        '',
        *_bullet_lines(task['non_goals']),
        '',
        '## Focused checks',
        '',
        '```bash',
        *(shlex.join(check['argv']) for check in task['focused_checks']),
        '```',
        '',
        'No private data or hardware is required.',
        '',
        'Labels: ' + ', '.join(f'`{label}`' for label in task['labels']),
    ]
    return '\n'.join(lines) + '\n'


def _verification_commands(
    task: dict[str, Any],
    temporary_root: Path,
) -> list[tuple[str, list[str]]]:
    profile = task['check_profile']
    if profile == 'docs-strict':
        return [(
            task['focused_checks'][0]['id'],
            [
                sys.executable,
                '-m',
                'mkdocs',
                'build',
                '--strict',
                '--site-dir',
                str(temporary_root / 'site'),
            ],
        )]
    if profile == 'mid360-empty-frame':
        return [
            (item['id'], list(command))
            for item, command in zip(
                task['focused_checks'], MID360_EMPTY_FRAME_CHECKS)
        ]
    raise QueueError(f'unsupported verification profile: {profile}')


def verify_task(
    task: dict[str, Any],
    repo_root: Path = REPO_ROOT,
) -> dict[str, Any]:
    """Run only the built-in command profile selected by a validated task."""
    environment = os.environ.copy()
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    environment['PYTEST_ADDOPTS'] = '-p no:cacheprovider'
    checks = []
    with tempfile.TemporaryDirectory(prefix='lidarslam-starter-') as temp_dir:
        commands = _verification_commands(task, Path(temp_dir))
        for (check_id, command), declared in zip(
            commands, task['focused_checks']
        ):
            try:
                result = subprocess.run(
                    command,
                    cwd=repo_root,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=300,
                    env=environment,
                )
                returncode = result.returncode
            except (OSError, subprocess.TimeoutExpired):
                returncode = 124
            checks.append({
                'id': check_id,
                'argv': declared['argv'],
                'status': 'PASS' if returncode == 0 else 'FAIL',
                'returncode': returncode,
            })
            if returncode != 0:
                break
    return {
        'status': (
            'FOCUSED_CHECKS_PASSED'
            if all(item['status'] == 'PASS' for item in checks)
            else 'FOCUSED_CHECKS_FAILED'
        ),
        'task_id': task['id'],
        'check_profile': task['check_profile'],
        'checks': checks,
        'acceptance_review_still_required': True,
        'workspace_artifacts_written': False,
        'remote_mutations_performed': False,
    }


def _github_get_all(repository: str, resource: str) -> list[dict[str, Any]]:
    """Read all open issues or pulls through the authenticated gh CLI."""
    if GITHUB_REPOSITORY_PATTERN.fullmatch(repository) is None:
        raise QueueError(f'invalid GitHub repository: {repository!r}')
    if resource not in {'issues', 'pulls'}:
        raise QueueError(f'unsupported GitHub resource: {resource}')

    records: list[dict[str, Any]] = []
    for page in range(1, GITHUB_MAX_PAGES + 1):
        endpoint = (
            f'/repos/{repository}/{resource}?state=open&'
            f'per_page={GITHUB_PAGE_SIZE}&page={page}'
        )
        command = [
            'gh',
            'api',
            '--method',
            'GET',
            '-H',
            'Accept: application/vnd.github+json',
            '-H',
            'X-GitHub-Api-Version: 2022-11-28',
            endpoint,
        ]
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
            raise QueueError(
                f'cannot read open GitHub {resource}: {exc}'
            ) from exc
        if result.returncode != 0:
            detail = result.stderr.strip() or 'gh api returned no error text'
            raise QueueError(
                f'cannot read open GitHub {resource}: {detail}'
            )
        try:
            page_records = json.loads(result.stdout)
        except json.JSONDecodeError as exc:
            raise QueueError(
                f'GitHub {resource} response is not valid JSON'
            ) from exc
        if not isinstance(page_records, list):
            raise QueueError(f'GitHub {resource} response must be an array')
        if not all(isinstance(item, dict) for item in page_records):
            raise QueueError(
                f'GitHub {resource} response contains a non-object record'
            )
        records.extend(page_records)
        if len(page_records) < GITHUB_PAGE_SIZE:
            return records
    raise QueueError(
        f'GitHub {resource} pagination exceeded {GITHUB_MAX_PAGES} pages; '
        'refusing to report partial availability'
    )


def _github_get_object(
    repository: str,
    endpoint_suffix: str,
    label: str,
    *,
    allow_not_found: bool = False,
) -> dict[str, Any] | None:
    """Read one bounded GitHub object with no mutation capability."""
    if GITHUB_REPOSITORY_PATTERN.fullmatch(repository) is None:
        raise QueueError(f'invalid GitHub repository: {repository!r}')
    endpoint = f'/repos/{repository}/{endpoint_suffix}'
    command = [
        'gh',
        'api',
        '--method',
        'GET',
        '-H',
        'Accept: application/vnd.github+json',
        '-H',
        'X-GitHub-Api-Version: 2022-11-28',
        endpoint,
    ]
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
        raise QueueError(f'cannot read GitHub {label}: {exc}') from exc
    if result.returncode != 0:
        detail = result.stderr.strip() or 'gh api returned no error text'
        if allow_not_found and 'HTTP 404' in detail:
            return None
        raise QueueError(f'cannot read GitHub {label}: {detail}')
    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise QueueError(
            f'GitHub {label} response is not valid JSON'
        ) from exc
    if not isinstance(payload, dict):
        raise QueueError(f'GitHub {label} response must be an object')
    return payload


def _github_get_product_pull(
    repository: str,
    pull_request_number: int,
) -> dict[str, Any]:
    """Read the declared product pull request through one GET request."""
    if (
        isinstance(pull_request_number, bool)
        or not isinstance(pull_request_number, int)
        or pull_request_number < 1
    ):
        raise QueueError('product pull request number must be positive')
    payload = _github_get_object(
        repository,
        f'pulls/{pull_request_number}',
        f'pull request #{pull_request_number}',
    )
    assert payload is not None
    return payload


def _github_get_public_queue(
    repository: str,
    path: str,
    ref: str,
) -> dict[str, Any] | None:
    """Read and decode the declared public queue JSON, accepting only 404."""
    _validate_repo_path(path)
    if ref != 'develop':
        raise QueueError('public queue ref must be develop')
    record = _github_get_object(
        repository,
        f'contents/{path}?ref={ref}',
        f'public queue {path}@{ref}',
        allow_not_found=True,
    )
    if record is None:
        return None
    if record.get('type') != 'file' or record.get('encoding') != 'base64':
        raise QueueError('public queue response is not one base64 file')
    content = record.get('content')
    if not isinstance(content, str):
        raise QueueError('public queue response has no base64 content')
    try:
        decoded = base64.b64decode(''.join(content.split()), validate=True)
        payload = json.loads(decoded.decode('utf-8'))
    except (
        binascii.Error,
        UnicodeDecodeError,
        json.JSONDecodeError,
    ) as exc:
        raise QueueError(
            'public queue content is not valid UTF-8 JSON'
        ) from exc
    if not isinstance(payload, dict):
        raise QueueError('public queue content must be a JSON object')
    return payload


def _public_issue(record: dict[str, Any]) -> dict[str, Any]:
    number = record.get('number')
    title = record.get('title')
    url = record.get('html_url')
    if (
        not isinstance(number, int)
        or isinstance(number, bool)
        or number < 1
        or not isinstance(title, str)
        or not title
        or not isinstance(url, str)
        or not url.startswith('https://github.com/')
    ):
        raise QueueError('GitHub issue or pull has invalid public identity')
    return {'number': number, 'title': title, 'url': url}


def _label_names(record: dict[str, Any]) -> set[str]:
    labels = record.get('labels', [])
    if not isinstance(labels, list):
        raise QueueError('GitHub issue labels must be an array')
    names: set[str] = set()
    for label in labels:
        if (
            not isinstance(label, dict)
            or not isinstance(label.get('name'), str)
        ):
            raise QueueError('GitHub issue label has invalid public identity')
        names.add(label['name'].casefold())
    return names


def _github_timestamp(value: Any, label: str) -> datetime:
    if not isinstance(value, str):
        raise QueueError(f'{label} must be an ISO-8601 timestamp')
    try:
        parsed = datetime.fromisoformat(value.replace('Z', '+00:00'))
    except ValueError as exc:
        raise QueueError(f'{label} must be an ISO-8601 timestamp') from exc
    if parsed.tzinfo is None:
        raise QueueError(f'{label} must include a UTC offset')
    return parsed.astimezone(timezone.utc)


def _matches_task_pull(
    task: dict[str, Any],
    pull: dict[str, Any],
    query: str,
) -> bool:
    title = pull.get('title')
    body = pull.get('body')
    if (
        not isinstance(title, str)
        or body is not None and not isinstance(body, str)
    ):
        raise QueueError('GitHub pull has invalid searchable text')
    searchable = f'{title}\n{body or ""}'.casefold()
    if task['title'].casefold() in searchable:
        return True
    terms = [item for item in query.casefold().split() if item]
    return bool(terms) and all(term in searchable for term in terms)


def _requires_duplicate_recheck(
    pull: dict[str, Any],
    audit: dict[str, Any],
    task_audit: dict[str, Any],
) -> bool:
    number = pull.get('number')
    if not isinstance(number, int) or isinstance(number, bool) or number < 1:
        raise QueueError('GitHub pull has invalid number')
    if number not in audit['open_pull_request_numbers']:
        return True
    if number in task_audit['matching_open_pull_requests']:
        return True
    updated_at = _github_timestamp(
        pull.get('updated_at'),
        f'GitHub pull #{number}.updated_at',
    )
    checked_at = _github_timestamp(
        audit['checked_at'],
        'remote_duplicate_audit.checked_at',
    )
    return updated_at > checked_at


def _collect_publication_gate(
    dependency: dict[str, Any],
) -> dict[str, Any]:
    """Evaluate a fixed local dependency without remote authority."""
    command = tuple(dependency['check_command'])
    expected = PUBLICATION_GATE_COMMANDS.get(dependency['id'])
    if expected is None or command != expected:
        raise QueueError(
            f"{dependency['id']} uses an unsupported gate command")
    environment = os.environ.copy()
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    try:
        result = subprocess.run(
            list(command),
            cwd=REPO_ROOT,
            check=False,
            capture_output=True,
            text=True,
            timeout=60,
            env=environment,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise QueueError(
            f"cannot evaluate publication gate {dependency['id']}: {exc}"
        ) from exc
    if result.returncode != 0:
        raise QueueError(
            f"publication gate {dependency['id']} did not produce a valid "
            'state report'
        )
    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise QueueError(
            f"publication gate {dependency['id']} returned invalid JSON"
        ) from exc
    if not isinstance(payload, dict):
        raise QueueError(
            f"publication gate {dependency['id']} report must be an object"
        )

    status = payload.get('status')
    permitted = payload.get('next_attempt_permitted_by_state')
    pending = payload.get('pending_launch_gates')
    stop_conditions = payload.get('stop_conditions')
    if (
        not isinstance(status, str)
        or not status
        or not isinstance(permitted, bool)
        or not isinstance(pending, list)
        or not all(isinstance(item, str) and item for item in pending)
        or not isinstance(stop_conditions, list)
        or not all(
            isinstance(item, str) and item for item in stop_conditions
        )
    ):
        raise QueueError(
            f"publication gate {dependency['id']} report is incomplete"
        )
    authority_fields = (
        'community_posts_authorized',
        'github_writes_authorized',
        'remote_mutations_performed',
    )
    if any(payload.get(field) is not False for field in authority_fields):
        raise QueueError(
            f"publication gate {dependency['id']} violates the no-write "
            'boundary'
        )

    eligible = (
        status in dependency['eligible_statuses']
        and permitted
    )
    reasons = list(dict.fromkeys([*pending, *stop_conditions]))
    if not eligible and not reasons:
        reasons = [status]
    return {
        'id': dependency['id'],
        'status': status,
        'eligible': eligible,
        'blocking_reasons': reasons,
        'check_command': list(command),
        'remote_mutations_performed': False,
    }


def collect_publication_gates(
    queue: dict[str, Any],
) -> list[dict[str, Any]]:
    """Evaluate every declared public-starter dependency in source order."""
    return [
        _collect_publication_gate(dependency)
        for dependency in queue['published_issue_dependencies']
    ]


def build_local_task_publication_gate(
    queue: dict[str, Any],
    pull: dict[str, Any],
    public_queue: dict[str, Any] | None,
) -> dict[str, Any]:
    """Require the product merge and exact public queue before issue review."""
    dependency = queue['local_task_publication_dependency']
    number = dependency['pull_request_number']
    base_ref = dependency['base_ref']
    path = dependency['public_queue_path']
    if pull.get('number') != number:
        raise QueueError('product pull response has the wrong number')
    state = pull.get('state')
    draft = pull.get('draft')
    merged = pull.get('merged')
    url = pull.get('html_url')
    pull_base = pull.get('base')
    pull_head = pull.get('head')
    head_sha = pull_head.get('sha') if isinstance(pull_head, dict) else None
    base_repository = (
        pull_base.get('repo') if isinstance(pull_base, dict) else None
    )
    if (
        state not in {'open', 'closed'}
        or not isinstance(draft, bool)
        or not isinstance(merged, bool)
        or not isinstance(url, str)
        or url != f'https://github.com/{queue["repository"]}/pull/{number}'
        or not isinstance(pull_base, dict)
        or pull_base.get('ref') != base_ref
        or not isinstance(base_repository, dict)
        or base_repository.get('full_name') != queue['repository']
        or not isinstance(head_sha, str)
        or re.fullmatch(r'[0-9a-f]{40}', head_sha) is None
        or (merged and state != 'closed')
        or (merged and draft)
    ):
        raise QueueError('product pull response has invalid bounded identity')

    expected_queue_sha256 = _canonical_sha256(queue)
    observed_queue_sha256 = (
        None if public_queue is None else _canonical_sha256(public_queue)
    )
    if public_queue is None:
        public_queue_status = 'ABSENT'
    elif observed_queue_sha256 == expected_queue_sha256:
        public_queue_status = 'MATCH'
    else:
        public_queue_status = 'DRIFT'

    blocking_reasons = []
    if not merged:
        blocking_reasons.append('product_pr_not_merged')
    if public_queue_status == 'ABSENT':
        blocking_reasons.append('public_queue_absent')
    elif public_queue_status == 'DRIFT':
        blocking_reasons.append('public_queue_drift')
    eligible = not blocking_reasons
    if eligible:
        status = 'READY'
    elif state == 'closed' and not merged:
        status = 'PRODUCT_PR_CLOSED_UNMERGED'
    elif not merged:
        status = 'WAITING_FOR_PRODUCT_MERGE'
    elif public_queue_status == 'ABSENT':
        status = 'PUBLIC_QUEUE_ABSENT'
    else:
        status = 'PUBLIC_QUEUE_DRIFT'

    return {
        'id': dependency['id'],
        'status': status,
        'eligible': eligible,
        'pull_request_number': number,
        'pull_request_url': url,
        'pull_request_state': state.upper(),
        'is_draft': draft,
        'merged': merged,
        'head_sha': head_sha,
        'target_branch': base_ref,
        'public_queue_path': path,
        'public_queue_status': public_queue_status,
        'expected_queue_sha256': expected_queue_sha256,
        'observed_queue_sha256': observed_queue_sha256,
        'blocking_reasons': blocking_reasons,
        'recheck_command': LOCAL_TASK_PUBLICATION_RECHECK_COMMAND,
        'github_requests': 'GET_ONLY',
        'remote_mutations_performed': False,
    }


def _validate_next_report(report: dict[str, Any]) -> None:
    schema = _load_json(DEFAULT_NEXT_SCHEMA, 'next-action schema')
    try:
        jsonschema.Draft7Validator.check_schema(schema)
        errors = sorted(
            jsonschema.Draft7Validator(schema).iter_errors(report),
            key=lambda item: [str(part) for part in item.absolute_path],
        )
    except jsonschema.SchemaError as exc:
        raise QueueError(
            f'next-action schema is invalid: {exc.message}'
        ) from exc
    if errors:
        first = errors[0]
        raise QueueError(
            'next-action report schema validation failed at '
            f'{_schema_error_path(first)}: {first.message}'
        )
    local_gate = report['local_task_publication_gate']
    gate_ready = local_gate['status'] == 'READY'
    if local_gate['eligible'] is not gate_ready:
        raise QueueError(
            'local task publication gate status and eligibility disagree'
        )
    if gate_ready != (not local_gate['blocking_reasons']):
        raise QueueError(
            'local task publication gate blockers disagree with readiness'
        )
    maintainer = report['maintainer_next']
    handoff = report['maintainer_publication_handoff']
    selected_actions = {
        'PREPARE_LOCAL_TASK_FOR_POST_MERGE',
        'REVIEW_AND_PUBLISH_LOCAL_TASK',
    }
    selected = maintainer['action'] in selected_actions
    if selected != (handoff is not None):
        raise QueueError(
            'maintainer publication handoff must exist exactly when one '
            'local task is selected'
        )
    if handoff is None:
        return
    if (
        maintainer['action'] == 'REVIEW_AND_PUBLISH_LOCAL_TASK'
        and not gate_ready
    ):
        raise QueueError('local task publication action requires a ready gate')
    if (
        maintainer['action'] == 'PREPARE_LOCAL_TASK_FOR_POST_MERGE'
        and gate_ready
    ):
        raise QueueError('post-merge preparation cannot carry a ready gate')
    if handoff['task_id'] != maintainer['task_id']:
        raise QueueError(
            'maintainer publication handoff task does not match next action'
        )
    body_digest = hashlib.sha256(
        handoff['issue_body'].encode('utf-8')
    ).hexdigest()
    if handoff['issue_body_sha256'] != body_digest:
        raise QueueError('maintainer publication issue body digest is stale')
    if handoff['labels'] != sorted(handoff['labels']):
        raise QueueError('maintainer publication labels must be sorted')
    if handoff['publication_gate_status'] != local_gate['status']:
        raise QueueError('maintainer handoff publication gate status is stale')
    if handoff['public_base_ready'] is not gate_ready:
        raise QueueError('maintainer handoff public-base readiness is stale')
    expected_kind = (
        'REVIEW_LOCAL_TASK_FOR_SEPARATE_PUBLICATION'
        if gate_ready else
        'PREPARE_LOCAL_TASK_FOR_POST_MERGE'
    )
    if handoff['kind'] != expected_kind:
        raise QueueError('maintainer handoff kind disagrees with public gate')


def _canonical_sha256(value: Any) -> str:
    """Hash one JSON-compatible value independently of source formatting."""
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    ).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _build_maintainer_publication_handoff(
    queue: dict[str, Any],
    task_id: str,
    local_gate: dict[str, Any],
) -> dict[str, Any]:
    """Bind one live-selected local task without granting issue authority."""
    task = _find_task(queue, task_id)
    rendered = render_task_markdown(task)
    heading = f"# {task['title']}\n\n"
    if not rendered.startswith(heading):
        raise QueueError(
            f'{task_id} rendered task does not start with its exact title'
        )
    issue_body = rendered[len(heading):]
    public_base_ready = local_gate['eligible']
    return {
        'kind': (
            'REVIEW_LOCAL_TASK_FOR_SEPARATE_PUBLICATION'
            if public_base_ready else
            'PREPARE_LOCAL_TASK_FOR_POST_MERGE'
        ),
        'task_id': task_id,
        'repository': queue['repository'],
        'title': task['title'],
        'labels': task['labels'],
        'issue_body': issue_body,
        'issue_body_sha256': hashlib.sha256(
            issue_body.encode('utf-8')
        ).hexdigest(),
        'task_sha256': _canonical_sha256(task),
        'queue_sha256': _canonical_sha256(queue),
        'live_duplicate_count': 0,
        'publication_gate_status': local_gate['status'],
        'public_base_ready': public_base_ready,
        'external_write_required': True,
        'maintainer_confirmation_required': True,
        'issue_creation_authorized': False,
        'writes_performed': False,
    }


def _dependency_for_issue(
    queue: dict[str, Any],
    issue: dict[str, Any],
) -> dict[str, Any] | None:
    for dependency in queue['published_issue_dependencies']:
        if (
            issue['number'] == dependency['issue_number']
            or issue['title'].casefold()
            == dependency['issue_title'].casefold()
        ):
            return dependency
    return None


def build_next_report(
    queue: dict[str, Any],
    local_report: dict[str, Any],
    issues: Sequence[dict[str, Any]],
    pulls: Sequence[dict[str, Any]],
    publication_gates: Sequence[dict[str, Any]],
    local_task_publication_gate: dict[str, Any],
) -> dict[str, Any]:
    """Combine local readiness with privacy-bounded live availability."""
    expected_gate_ids = [
        item['id'] for item in queue['published_issue_dependencies']
    ]
    gate_ids = [item.get('id') for item in publication_gates]
    if gate_ids != expected_gate_ids:
        raise QueueError(
            'publication gate reports must cover declared dependencies in '
            'source order'
        )
    for dependency, gate in zip(
        queue['published_issue_dependencies'], publication_gates
    ):
        expected_eligible = gate.get('status') in dependency[
            'eligible_statuses'
        ]
        if gate.get('eligible') is not expected_eligible:
            raise QueueError(
                f"publication gate {dependency['id']} eligibility disagrees "
                'with its declared statuses'
            )
        reasons = gate.get('blocking_reasons')
        if (
            not isinstance(reasons, list)
            or expected_eligible and reasons
            or not expected_eligible and not reasons
        ):
            raise QueueError(
                f"publication gate {dependency['id']} blocking reasons "
                'disagree with eligibility'
            )
    gates_by_id = {item['id']: item for item in publication_gates}
    if (
        local_task_publication_gate.get('id')
        != queue['local_task_publication_dependency']['id']
        or not isinstance(
            local_task_publication_gate.get('eligible'),
            bool,
        )
        or local_task_publication_gate.get('github_requests') != 'GET_ONLY'
        or local_task_publication_gate.get(
            'remote_mutations_performed'
        ) is not False
        or local_task_publication_gate.get('pull_request_number')
        != queue['local_task_publication_dependency'][
            'pull_request_number'
        ]
        or local_task_publication_gate.get('target_branch')
        != queue['local_task_publication_dependency']['base_ref']
        or local_task_publication_gate.get('public_queue_path')
        != queue['local_task_publication_dependency']['public_queue_path']
        or local_task_publication_gate.get('expected_queue_sha256')
        != _canonical_sha256(queue)
    ):
        raise QueueError('local task publication gate is incomplete or unsafe')
    open_issues = [item for item in issues if 'pull_request' not in item]
    public_issues = [_public_issue(item) for item in open_issues]
    public_pulls = [_public_issue(item) for item in pulls]
    good_first_issues = [
        public
        for raw, public in zip(open_issues, public_issues)
        if 'good first issue' in _label_names(raw)
    ]
    eligible_good_first_issues = []
    blocked_good_first_issues = []
    for issue in good_first_issues:
        dependency = _dependency_for_issue(queue, issue)
        if dependency is None:
            eligible_good_first_issues.append(issue)
            continue
        gate = gates_by_id[dependency['id']]
        if gate['eligible']:
            eligible_good_first_issues.append(issue)
        else:
            blocked_good_first_issues.append({**issue, 'gate': gate})
    published_tasks = []
    for task in queue['tasks']:
        matches = [
            item for item in public_issues
            if item['title'].casefold() == task['title'].casefold()
        ]
        for item in matches:
            published_tasks.append({'task_id': task['id'], **item})

    audit = queue['remote_duplicate_audit']
    audit_matches = {
        item['task_id']: item for item in audit['task_matches']
    }
    potential_duplicates = []
    for task in queue['tasks']:
        task_audit = audit_matches[task['id']]
        matches = [
            public
            for raw, public in zip(pulls, public_pulls)
            if _requires_duplicate_recheck(raw, audit, task_audit)
            and _matches_task_pull(task, raw, task_audit['query'])
        ]
        if matches:
            potential_duplicates.append({
                'task_id': task['id'],
                'pull_requests': matches,
            })

    published_ids = {item['task_id'] for item in published_tasks}
    duplicate_ids = {item['task_id'] for item in potential_duplicates}
    unpublished_ready = [
        task_id for task_id in local_report['ready_task_ids']
        if task_id not in published_ids
    ]
    publishable = [
        task_id for task_id in unpublished_ready
        if task_id not in duplicate_ids
    ]

    if published_tasks:
        first = published_tasks[0]
        status = 'PUBLISHED_QUEUE_TASK_AVAILABLE'
        contributor_next = {
            'action': 'REVIEW_PUBLISHED_QUEUE_TASK',
            'task_id': first['task_id'],
            'issue_number': first['number'],
            'url': first['url'],
        }
    elif eligible_good_first_issues:
        first = eligible_good_first_issues[0]
        status = 'PUBLISHED_GOOD_FIRST_ISSUE_AVAILABLE'
        contributor_next = {
            'action': 'REVIEW_PUBLISHED_GOOD_FIRST_ISSUE',
            'issue_number': first['number'],
            'url': first['url'],
        }
    elif blocked_good_first_issues:
        status = 'PUBLISHED_GOOD_FIRST_ISSUES_BLOCKED'
        contributor_next = {
            'action': 'WAIT_FOR_READY_PUBLISHED_STARTER',
            'url': (
                f"https://github.com/{queue['repository']}/issues?"
                'q=is%3Aissue+is%3Aopen+label%3A%22good+first+issue%22'
            ),
        }
    else:
        status = 'MAINTAINER_PUBLICATION_REQUIRED'
        contributor_next = {
            'action': 'WAIT_FOR_PUBLISHED_STARTER',
            'url': (
                f"https://github.com/{queue['repository']}/issues?"
                'q=is%3Aissue+is%3Aopen+label%3A%22good+first+issue%22'
            ),
        }

    if potential_duplicates:
        maintainer_next = {'action': 'REVIEW_POTENTIAL_PULL_DUPLICATE'}
    elif publishable:
        task_id = publishable[0]
        preview_command = [
            'python3',
            'scripts/contributor_starter_queue.py',
            '--task',
            task_id,
        ]
        if local_task_publication_gate['eligible']:
            maintainer_next = {
                'action': 'REVIEW_AND_PUBLISH_LOCAL_TASK',
                'task_id': task_id,
                'preview_command': preview_command,
            }
        else:
            maintainer_next = {
                'action': 'PREPARE_LOCAL_TASK_FOR_POST_MERGE',
                'task_id': task_id,
                'gate_status': local_task_publication_gate['status'],
                'preview_command': preview_command,
                'recheck_command': LOCAL_TASK_PUBLICATION_RECHECK_COMMAND,
            }
    elif blocked_good_first_issues:
        blocked = blocked_good_first_issues[0]
        gate = blocked['gate']
        maintainer_next = {
            'action': 'REVIEW_BLOCKED_PUBLISHED_STARTER',
            'issue_number': blocked['number'],
            'gate_id': gate['id'],
            'gate_status': gate['status'],
            'review_command': gate['check_command'],
        }
    else:
        maintainer_next = {'action': 'REVIEW_QUEUE_STATE'}

    maintainer_publication_handoff = None
    if maintainer_next['action'] in {
        'PREPARE_LOCAL_TASK_FOR_POST_MERGE',
        'REVIEW_AND_PUBLISH_LOCAL_TASK',
    }:
        maintainer_publication_handoff = _build_maintainer_publication_handoff(
            queue,
            maintainer_next['task_id'],
            local_task_publication_gate,
        )

    report = {
        'schema_version': 1,
        'schema_uri': NEXT_SCHEMA_URI,
        'status': status,
        'repository': queue['repository'],
        'local_queue_status': local_report['status'],
        'local_publication_status': local_report['publication_status'],
        'open_issue_count': len(open_issues),
        'open_pull_request_count': len(pulls),
        'publication_gates': list(publication_gates),
        'local_task_publication_gate': local_task_publication_gate,
        'published_good_first_issues': good_first_issues,
        'eligible_good_first_issues': eligible_good_first_issues,
        'blocked_good_first_issues': blocked_good_first_issues,
        'published_queue_tasks': published_tasks,
        'unpublished_ready_task_ids': unpublished_ready,
        'potential_pull_duplicates': potential_duplicates,
        'contributor_next': contributor_next,
        'maintainer_next': maintainer_next,
        'maintainer_publication_handoff': maintainer_publication_handoff,
        'authority': {
            'github_requests': 'GET_ONLY',
            'github_writes_authorized': False,
            'remote_mutations_performed': False,
        },
    }
    _validate_next_report(report)
    return report


def collect_next_report(
    queue: dict[str, Any],
    local_report: dict[str, Any],
) -> dict[str, Any]:
    """Collect the current read-only GitHub status for the next-step card."""
    issues = _github_get_all(queue['repository'], 'issues')
    pulls = _github_get_all(queue['repository'], 'pulls')
    publication_gates = collect_publication_gates(queue)
    dependency = queue['local_task_publication_dependency']
    product_pull = _github_get_product_pull(
        queue['repository'],
        dependency['pull_request_number'],
    )
    public_queue = _github_get_public_queue(
        queue['repository'],
        dependency['public_queue_path'],
        dependency['base_ref'],
    )
    local_task_gate = build_local_task_publication_gate(
        queue,
        product_pull,
        public_queue,
    )
    return build_next_report(
        queue,
        local_report,
        issues,
        pulls,
        publication_gates,
        local_task_gate,
    )


def render_next_report(report: dict[str, Any]) -> str:
    """Render the live contributor and maintainer next steps."""
    lines = [f"Contributor next step — {report['status']}"]
    local_gate = report['local_task_publication_gate']
    reasons = ', '.join(local_gate['blocking_reasons']) or 'none'
    lines.append(
        'Local task public base: '
        f"{local_gate['status']} (PR #{local_gate['pull_request_number']}; "
        f"queue {local_gate['public_queue_status']}; {reasons})"
    )
    issues = report['published_good_first_issues']
    eligible = report['eligible_good_first_issues']
    blocked = report['blocked_good_first_issues']
    lines.append(
        f'Published good first issues: {len(issues)} '
        f'({len(eligible)} ready, {len(blocked)} blocked)'
    )
    for issue in eligible:
        lines.append(
            f"- READY #{issue['number']} {issue['title']} — {issue['url']}"
        )
    for issue in blocked:
        gate = issue['gate']
        reasons = ', '.join(gate['blocking_reasons'])
        lines.append(
            f"- BLOCKED #{issue['number']} {issue['title']} — "
            f"{gate['status']} ({reasons}) — {issue['url']}"
        )
    lines.append(
        'Local 30-minute queue: '
        f"{len(report['unpublished_ready_task_ids'])} ready but unpublished"
    )
    contributor = report['contributor_next']
    if contributor['action'] == 'REVIEW_PUBLISHED_QUEUE_TASK':
        lines.append(
            'Contributor: review and claim the published queue task at '
            f"{contributor['url']}"
        )
    elif contributor['action'] == 'REVIEW_PUBLISHED_GOOD_FIRST_ISSUE':
        lines.append(
            'Contributor: review the published starter scope at '
            f"{contributor['url']}"
        )
    elif contributor['action'] == 'WAIT_FOR_READY_PUBLISHED_STARTER':
        lines.append(
            'Contributor: no published starter is ready; do not start a '
            'blocked cohort task or a local queue task yet.'
        )
    else:
        lines.append(
            'Contributor: wait for a published starter; do not start a local '
            'queue task yet.'
        )
    maintainer = report['maintainer_next']
    if maintainer['action'] == 'REVIEW_BLOCKED_PUBLISHED_STARTER':
        lines.append(
            'Maintainer: inspect the blocked public starter gate with '
            f"`{shlex.join(maintainer['review_command'])}`; do not recruit "
            'from the issue while the gate is closed.'
        )
    elif maintainer['action'] == 'REVIEW_AND_PUBLISH_LOCAL_TASK':
        lines.append(
            'Maintainer: review the next bounded task with '
            f"`{shlex.join(maintainer['preview_command'])}`"
        )
    elif maintainer['action'] == 'PREPARE_LOCAL_TASK_FOR_POST_MERGE':
        lines.append(
            'Maintainer: prepare the bounded task with '
            f"`{shlex.join(maintainer['preview_command'])}`; do not publish "
            'until the local task gate is READY. Recheck with '
            f"`{shlex.join(maintainer['recheck_command'])}`"
        )
    elif maintainer['action'] == 'REVIEW_POTENTIAL_PULL_DUPLICATE':
        lines.append(
            'Maintainer: review the potential open-PR duplicates before '
            'publishing any local task.'
        )
    handoff = report['maintainer_publication_handoff']
    if handoff is not None:
        lines.extend([
            'Publication handoff: exact local body for '
            f"{handoff['task_id']}; SHA-256 "
            f"{handoff['issue_body_sha256']}.",
            'Public base ready: '
            f"{'yes' if handoff['public_base_ready'] else 'no'} "
            f"({handoff['publication_gate_status']}).",
            'GitHub issue creation authorized: no; maintainer confirmation '
            'and a separate external write are required.',
        ])
    lines.append(
        f"Remote check: {report['open_pull_request_count']} open "
        f"PR{'s' if report['open_pull_request_count'] != 1 else ''}; "
        f"{len(report['potential_pull_duplicates'])} potential queue matches."
    )
    lines.append(
        'Read-only: no GitHub issue, pull request, or label was changed.'
    )
    return '\n'.join(lines) + '\n'


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--queue', type=Path, default=DEFAULT_QUEUE)
    parser.add_argument('--schema', type=Path, default=DEFAULT_SCHEMA)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--list', action='store_true')
    action.add_argument('--task', choices=EXPECTED_TASK_IDS)
    action.add_argument('--verify', choices=EXPECTED_TASK_IDS)
    action.add_argument(
        '--next',
        action='store_true',
        help='show one live, read-only contributor and maintainer next step',
    )
    parser.add_argument('--json', action='store_true')
    return parser.parse_args(argv)


def _print_list(
    queue: dict[str, Any],
    report: dict[str, Any],
) -> None:
    stale_ids = {item['id'] for item in report['stale_tasks']}
    print('Local starter queue — PREPARED_NOT_PUBLISHED')
    print('Wait for a published issue or explicit maintainer confirmation.')
    for task in queue['tasks']:
        readiness = 'STALE' if task['id'] in stale_ids else 'READY'
        print(
            f"{task['id']} [{readiness}] {task['estimate_minutes']} min — "
            f"{task['title']}")


def main(argv: Sequence[str] | None = None) -> int:
    """Run the selected read-only inspection or fixed verification action."""
    args = parse_args(argv)
    try:
        queue, report = evaluate(args.queue, args.schema)
        if args.next:
            next_report = collect_next_report(queue, report)
            if args.json:
                print(json.dumps(next_report, indent=2, sort_keys=True))
            else:
                print(render_next_report(next_report), end='')
            return 0
        if args.verify:
            task = _find_task(queue, args.verify)
            verification = verify_task(task)
            if args.json:
                print(json.dumps(verification, indent=2, sort_keys=True))
            else:
                for check in verification['checks']:
                    print(f"{check['status']}: {shlex.join(check['argv'])}")
                print(verification['status'])
            return (
                0
                if verification['status'] == 'FOCUSED_CHECKS_PASSED'
                else 1
            )
        if args.task:
            task = _find_task(queue, args.task)
            _require_render_ready(report, args.task)
            if args.json:
                print(json.dumps({
                    'queue_status': report['status'],
                    'task': task,
                    'authority': report['authority'],
                }, indent=2, sort_keys=True))
            else:
                print(render_task_markdown(task), end='')
            return 0
        if args.list:
            if args.json:
                print(json.dumps({
                    'queue_status': report['status'],
                    'publication_status': report['publication_status'],
                    'tasks': [
                        {
                            'id': task['id'],
                            'title': task['title'],
                            'estimate_minutes': task['estimate_minutes'],
                        }
                        for task in queue['tasks']
                    ],
                    'authority': report['authority'],
                }, indent=2, sort_keys=True))
            else:
                _print_list(queue, report)
            return 0
        if args.json:
            print(json.dumps(report, indent=2, sort_keys=True))
        else:
            print(
                f"PASS: {report['task_count']} local starter tasks are "
                'bounded, unclaimed, and ready for maintainer review')
            print('No GitHub issue was created or changed.')
        return 0 if report['status'] == 'QUEUE_READY_LOCAL_ONLY' else 1
    except QueueError as exc:
        if args.json:
            print(json.dumps({
                'status': 'QUEUE_INVALID',
                'error': str(exc),
                'remote_mutations_performed': False,
            }, sort_keys=True))
        else:
            print(f'ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
