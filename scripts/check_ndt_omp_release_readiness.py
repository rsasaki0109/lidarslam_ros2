#!/usr/bin/env python3
"""Read-only, fail-closed preflight for the first ndt_omp_ros2 release."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Any
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET

import jsonschema

try:
    from github_api_auth import github_api_authorization
except ModuleNotFoundError:  # pragma: no cover - importlib test path
    from scripts.github_api_auth import github_api_authorization


REPO_ROOT = Path(__file__).resolve().parents[1]
SUBMODULE = Path('Thirdparty/ndt_omp_ros2')
EXPECTED_COMMIT = '8b77fa5a6cdcad45bf35918361c892b6d94a287e'
EXPECTED_VERSION = '0.1.0'
SCHEMA_PATH = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'ndt-omp-release-readiness-v2.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/ndt-omp-release-readiness-v2.schema.json'
)
SOURCE_REPOSITORY = 'rsasaki0109/ndt_omp_ros2'
RELEASE_REPOSITORY = 'rsasaki0109/ndt_omp_ros2-release'
ROSDISTRO_REPOSITORY = 'ros/rosdistro'
DISTROS = ('humble', 'jazzy')
ROSDISTRO_PULL_REQUESTS = {
    'humble': 52949,
    'jazzy': 52950,
}
ROSDISTRO_PULL_REQUEST_HEADS = {
    'humble': 'c375b1c8e92d14e58a4c10e023920763645fe5c7',
    'jazzy': 'ef7e147af917eee64f4569a528dd98400004cadd',
}
ROSDISTRO_REVIEW_URLS = {
    'humble': (
        'https://github.com/ros/rosdistro/pull/52949'
        '#pullrequestreview-4857900792'
    ),
    'jazzy': (
        'https://github.com/ros/rosdistro/pull/52950'
        '#pullrequestreview-4857894506'
    ),
}
UPSTREAM_REPOSITORY = 'koide3/ndt_omp'
UPSTREAM_FORK_REPOSITORY = 'rsasaki0109/ndt_omp_ros2'
UPSTREAM_BASE_BRANCH = 'master'
UPSTREAM_CANDIDATE_COMMIT = '618f02f6b50a8590b81f48b4fee5b6cfc8d3f3ea'
UPSTREAM_PULL_REQUEST_URL = re.compile(
    r'https://github\.com/koide3/ndt_omp/pull/([1-9][0-9]*)')
ACTIONABLE_REVIEW = re.compile(
    r'(?:\?|\b(?:please|could you|would you|how does|how do|why|what is|'
    r'what are|same question|needs? to|must)\b)',
    re.IGNORECASE,
)
PASSING_CHECK_CONCLUSIONS = frozenset({'success', 'neutral', 'skipped'})


class PreflightError(ValueError):
    """The release candidate or remote inspection could not be trusted."""


def _run(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ['git', *args],
        cwd=repo,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise PreflightError(
            f"git {' '.join(args)} failed in {repo}: "
            f'{result.stderr.strip() or result.stdout.strip()}'
        )
    return result.stdout.strip()


def _check(check_id: str, passed: bool, detail: str) -> dict[str, str]:
    return {
        'id': check_id,
        'status': 'PASS' if passed else 'FAIL',
        'detail': detail,
    }


def inspect_local(repo_root: Path = REPO_ROOT) -> dict[str, Any]:
    """Inspect the pinned submodule without changing either repository."""
    repo_root = repo_root.resolve()
    submodule = repo_root / SUBMODULE
    checks: list[dict[str, str]] = []

    try:
        stage = _run(repo_root, 'ls-files', '--stage', '--', SUBMODULE.as_posix())
        fields = stage.split()
        gitlink = fields[1] if len(fields) >= 2 and fields[0] == '160000' else ''
        checks.append(_check(
            'parent-gitlink',
            gitlink == EXPECTED_COMMIT,
            f'expected {EXPECTED_COMMIT}; found {gitlink or "no gitlink"}',
        ))
    except PreflightError as exc:
        gitlink = ''
        checks.append(_check('parent-gitlink', False, str(exc)))

    if not submodule.is_dir():
        checks.append(_check(
            'submodule-present',
            False,
            f'missing directory {SUBMODULE.as_posix()}',
        ))
        return {
            'ready': False,
            'gitlink_commit': gitlink,
            'head_commit': '',
            'package_version': '',
            'checks': checks,
        }

    checks.append(_check(
        'submodule-present',
        True,
        f'{SUBMODULE.as_posix()} is initialized',
    ))
    try:
        head = _run(submodule, 'rev-parse', 'HEAD')
        checks.append(_check(
            'candidate-commit',
            head == EXPECTED_COMMIT,
            f'expected {EXPECTED_COMMIT}; found {head}',
        ))
        dirty = _run(submodule, 'status', '--porcelain')
        checks.append(_check(
            'clean-worktree',
            not dirty,
            'submodule worktree is clean' if not dirty else
            'submodule has uncommitted changes',
        ))
    except PreflightError as exc:
        head = ''
        checks.append(_check('submodule-git-state', False, str(exc)))

    package_name = ''
    package_version = ''
    package_license = ''
    try:
        package = ET.parse(submodule / 'package.xml').getroot()
        package_name = (package.findtext('name') or '').strip()
        package_version = (package.findtext('version') or '').strip()
        licenses = [
            (node.text or '').strip() for node in package.findall('license')
        ]
        package_license = ','.join(licenses)
        checks.extend([
            _check(
                'package-name',
                package_name == 'ndt_omp_ros2',
                f'expected ndt_omp_ros2; found {package_name or "empty"}',
            ),
            _check(
                'package-version',
                package_version == EXPECTED_VERSION,
                f'expected {EXPECTED_VERSION}; '
                f'found {package_version or "empty"}',
            ),
            _check(
                'package-license',
                licenses == ['BSD-2-Clause'],
                f'expected BSD-2-Clause; found {package_license or "empty"}',
            ),
        ])
    except (OSError, ET.ParseError) as exc:
        checks.append(_check('package-metadata', False, str(exc)))

    required_text = {
        'changelog-entry': (
            'CHANGELOG.rst',
            f'{EXPECTED_VERSION} (',
        ),
        'cmake-library-install': (
            'CMakeLists.txt',
            '  TARGETS\n    ndt_omp\n  EXPORT export_ndt_omp',
        ),
        'cmake-target-export': (
            'CMakeLists.txt',
            'ament_export_targets(export_ndt_omp HAS_LIBRARY_TARGET)',
        ),
        'bloom-preflight': (
            'scripts/check_bloom_release.py',
            "'bloom-generate'",
        ),
        'bloom-schema': (
            'schemas/bloom-release-v1.schema.json',
            '"$schema"',
        ),
        'release-ci': (
            '.github/workflows/ci.yml',
            'check_bloom_release.py',
        ),
    }
    for check_id, (relative, marker) in required_text.items():
        path = submodule / relative
        try:
            content = path.read_text(encoding='utf-8')
            checks.append(_check(
                check_id,
                marker in content,
                f'{relative} contains required marker' if marker in content
                else f'{relative} lacks required marker {marker!r}',
            ))
        except OSError as exc:
            checks.append(_check(check_id, False, f'{relative}: {exc}'))

    return {
        'ready': all(item['status'] == 'PASS' for item in checks),
        'gitlink_commit': gitlink,
        'head_commit': head,
        'package_version': package_version,
        'checks': checks,
    }


def _request_text(url: str) -> tuple[int, str]:
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'lidarslam-ndt-release-preflight/1',
    }
    headers.update(github_api_authorization(url, method='GET'))
    request = urllib.request.Request(
        url,
        headers=headers,
    )
    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            return response.status, response.read().decode('utf-8')
    except urllib.error.HTTPError as exc:
        if exc.code == 404:
            return 404, ''
        raise PreflightError(f'HTTP {exc.code} while reading {url}') from exc
    except (urllib.error.URLError, TimeoutError, UnicodeDecodeError) as exc:
        raise PreflightError(f'cannot read {url}: {exc}') from exc


def _empty_check_runs() -> dict[str, Any]:
    return {
        'inspected': False,
        'total_count': None,
        'passing_count': None,
        'pending_count': None,
        'failing_count': None,
        'runs': [],
    }


def _empty_pull_requests() -> dict[str, dict[str, Any]]:
    return {
        distro: {
            'number': ROSDISTRO_PULL_REQUESTS[distro],
            'url': (
                'https://github.com/ros/rosdistro/pull/'
                f'{ROSDISTRO_PULL_REQUESTS[distro]}'
            ),
            'state': None,
            'merged': None,
            'mergeable': None,
            'head_sha': None,
            'updated_at': None,
            'latest_actionable_review': {
                'url': None,
                'author': None,
                'created_at': None,
            },
            'response_pending': None,
            'check_runs': _empty_check_runs(),
        }
        for distro in DISTROS
    }


def _request_json(url: str) -> Any:
    status, body = _request_text(url)
    if status != 200:
        raise PreflightError(f'unexpected HTTP {status} while reading {url}')
    try:
        return json.loads(body)
    except json.JSONDecodeError as exc:
        raise PreflightError(f'invalid JSON while reading {url}: {exc}') from exc


def _activity_user(item: dict[str, Any]) -> tuple[str, bool]:
    user = item.get('user')
    if not isinstance(user, dict):
        return '', False
    login = user.get('login')
    user_type = user.get('type')
    if not isinstance(login, str):
        return '', False
    is_bot = user_type == 'Bot' or login.endswith('[bot]')
    return login, is_bot


def _inspect_check_runs(
    api_root: str,
    head_sha: str,
    number: int,
) -> dict[str, Any]:
    payload = _request_json(
        f'{api_root}/commits/{head_sha}/check-runs?per_page=100')
    if not isinstance(payload, dict):
        raise PreflightError(
            f'pull request #{number} check-runs response is not an object')
    total_count = payload.get('total_count')
    raw_runs = payload.get('check_runs')
    if (
        not isinstance(total_count, int)
        or isinstance(total_count, bool)
        or not isinstance(raw_runs, list)
    ):
        raise PreflightError(
            f'pull request #{number} check-runs response is incomplete')
    if total_count != len(raw_runs):
        raise PreflightError(
            f'pull request #{number} check-runs response was truncated: '
            f'expected {total_count}; received {len(raw_runs)}')

    runs: list[dict[str, Any]] = []
    passing_count = 0
    pending_count = 0
    failing_count = 0
    for item in raw_runs:
        if not isinstance(item, dict):
            raise PreflightError(
                f'pull request #{number} contains an invalid check run')
        name = item.get('name')
        status = item.get('status')
        conclusion = item.get('conclusion')
        details_url = item.get('details_url')
        if (
            not isinstance(name, str)
            or not name
            or not isinstance(status, str)
            or not status
            or conclusion is not None and not isinstance(conclusion, str)
            or details_url is not None and not isinstance(details_url, str)
        ):
            raise PreflightError(
                f'pull request #{number} contains an incomplete check run')

        if status != 'completed':
            classification = 'PENDING'
            pending_count += 1
        elif conclusion in PASSING_CHECK_CONCLUSIONS:
            classification = 'PASSING'
            passing_count += 1
        else:
            classification = 'FAILING'
            failing_count += 1
        runs.append({
            'name': name,
            'status': status,
            'conclusion': conclusion,
            'details_url': details_url,
            'classification': classification,
        })

    runs.sort(key=lambda item: (
        item['classification'], item['name'], item['details_url'] or ''))
    return {
        'inspected': True,
        'total_count': total_count,
        'passing_count': passing_count,
        'pending_count': pending_count,
        'failing_count': failing_count,
        'runs': runs,
    }


def _inspect_pull_request(distro: str) -> dict[str, Any]:
    number = ROSDISTRO_PULL_REQUESTS[distro]
    api_root = f'https://api.github.com/repos/{ROSDISTRO_REPOSITORY}'
    payload = _request_json(f'{api_root}/pulls/{number}')
    if not isinstance(payload, dict):
        raise PreflightError(f'pull request #{number} response is not an object')
    author, _ = _activity_user(payload)
    if not author:
        raise PreflightError(f'pull request #{number} has no author identity')

    activity_specs = (
        (f'{api_root}/pulls/{number}/reviews?per_page=100', 'submitted_at'),
        (f'{api_root}/issues/{number}/comments?per_page=100', 'created_at'),
        (f'{api_root}/pulls/{number}/comments?per_page=100', 'created_at'),
    )
    actionable: list[dict[str, str]] = []
    author_responses: list[str] = []
    for url, timestamp_field in activity_specs:
        items = _request_json(url)
        if not isinstance(items, list):
            raise PreflightError(
                f'pull request #{number} activity response is not a list')
        for item in items:
            if not isinstance(item, dict):
                continue
            body = item.get('body')
            timestamp = item.get(timestamp_field)
            login, is_bot = _activity_user(item)
            if (
                not isinstance(body, str)
                or not body.strip()
                or not isinstance(timestamp, str)
                or not login
            ):
                continue
            if login == author:
                author_responses.append(timestamp)
                continue
            review_state = item.get('state')
            is_actionable = (
                review_state == 'CHANGES_REQUESTED'
                or ACTIONABLE_REVIEW.search(body) is not None
            )
            if is_bot or not is_actionable:
                continue
            html_url = item.get('html_url')
            if not isinstance(html_url, str) or not html_url.startswith(
                    'https://github.com/'):
                continue
            actionable.append({
                'url': html_url,
                'author': login,
                'created_at': timestamp,
            })

    latest = max(actionable, key=lambda item: item['created_at']) \
        if actionable else None
    response_pending = bool(
        latest is not None
        and not any(
            timestamp > latest['created_at']
            for timestamp in author_responses
        )
        and payload.get('merged') is not True
    )
    state = payload.get('state')
    merged = payload.get('merged')
    mergeable = payload.get('mergeable')
    head = payload.get('head')
    head_sha = head.get('sha') if isinstance(head, dict) else None
    updated_at = payload.get('updated_at')
    html_url = payload.get('html_url')
    if state not in {'open', 'closed'} or not isinstance(merged, bool):
        raise PreflightError(f'pull request #{number} has invalid state fields')
    if mergeable is not None and not isinstance(mergeable, bool):
        raise PreflightError(f'pull request #{number} has invalid mergeable field')
    if not isinstance(head_sha, str) or not isinstance(updated_at, str):
        raise PreflightError(f'pull request #{number} has incomplete identity')
    if not isinstance(html_url, str):
        raise PreflightError(f'pull request #{number} has no public URL')
    check_runs = _inspect_check_runs(api_root, head_sha, number)
    return {
        'number': number,
        'url': html_url,
        'state': state,
        'merged': merged,
        'mergeable': mergeable,
        'head_sha': head_sha,
        'updated_at': updated_at,
        'latest_actionable_review': latest or {
            'url': None,
            'author': None,
            'created_at': None,
        },
        'response_pending': response_pending,
        'check_runs': check_runs,
    }


def inspect_remote() -> dict[str, Any]:
    """Read public GitHub and rosdistro state without writing."""
    errors: list[str] = []
    origin_commit: str | None = None
    source_tag_present: bool | None = None
    release_repository_present: bool | None = None
    rosdistro: dict[str, bool | None] = {distro: None for distro in DISTROS}
    pull_requests = _empty_pull_requests()

    try:
        status, body = _request_text(
            f'https://api.github.com/repos/{SOURCE_REPOSITORY}/'
            'git/ref/heads/humble'
        )
        if status != 200:
            raise PreflightError('source branch humble unexpectedly returned 404')
        payload = json.loads(body)
        origin_commit = payload['object']['sha']
    except (PreflightError, json.JSONDecodeError, KeyError, TypeError) as exc:
        errors.append(f'origin branch: {exc}')

    try:
        status, _ = _request_text(
            f'https://api.github.com/repos/{SOURCE_REPOSITORY}/'
            f'git/ref/tags/{EXPECTED_VERSION}'
        )
        source_tag_present = status == 200
    except PreflightError as exc:
        errors.append(f'source tag: {exc}')

    try:
        status, _ = _request_text(
            f'https://api.github.com/repos/{RELEASE_REPOSITORY}'
        )
        release_repository_present = status == 200
    except PreflightError as exc:
        errors.append(f'release repository: {exc}')

    for distro in DISTROS:
        try:
            status, body = _request_text(
                'https://raw.githubusercontent.com/ros/rosdistro/'
                f'master/{distro}/distribution.yaml'
            )
            if status != 200:
                raise PreflightError(
                    f'{distro} distribution unexpectedly returned 404')
            rosdistro[distro] = any(
                line == '  ndt_omp_ros2:'
                for line in body.splitlines()
            )
        except PreflightError as exc:
            errors.append(f'rosdistro {distro}: {exc}')

    if source_tag_present is True and release_repository_present is True:
        for distro in DISTROS:
            try:
                pull_requests[distro] = _inspect_pull_request(distro)
            except PreflightError as exc:
                errors.append(f'rosdistro PR {distro}: {exc}')

    return {
        'errors': errors,
        'origin_branch_commit': origin_commit,
        'source_tag_present': source_tag_present,
        'release_repository_present': release_repository_present,
        'rosdistro': rosdistro,
        'pull_requests': pull_requests,
    }


def _inspect_upstream_pull_request(url: str) -> dict[str, Any]:
    """Read one proposed canonical Draft PR without changing GitHub state."""
    match = UPSTREAM_PULL_REQUEST_URL.fullmatch(url)
    if match is None:
        raise PreflightError(
            'upstream PR URL must match '
            'https://github.com/koide3/ndt_omp/pull/<number>')
    number = int(match.group(1))
    payload = _request_json(
        f'https://api.github.com/repos/{UPSTREAM_REPOSITORY}/pulls/{number}')
    if not isinstance(payload, dict):
        raise PreflightError('upstream pull request response is not an object')

    head = payload.get('head')
    base = payload.get('base')
    head_repo = head.get('repo') if isinstance(head, dict) else None
    base_repo = base.get('repo') if isinstance(base, dict) else None
    result = {
        'number': number,
        'url': payload.get('html_url'),
        'state': payload.get('state'),
        'draft': payload.get('draft'),
        'merged': payload.get('merged'),
        'head_sha': head.get('sha') if isinstance(head, dict) else None,
        'head_repository': (
            head_repo.get('full_name')
            if isinstance(head_repo, dict) else None
        ),
        'base_branch': base.get('ref') if isinstance(base, dict) else None,
        'base_repository': (
            base_repo.get('full_name')
            if isinstance(base_repo, dict) else None
        ),
    }
    if result['url'] != url:
        raise PreflightError(
            f'upstream pull request canonical URL drifted: {result["url"]!r}')
    if result['state'] not in {'open', 'closed'}:
        raise PreflightError('upstream pull request has invalid state')
    if not isinstance(result['draft'], bool):
        raise PreflightError('upstream pull request has invalid draft state')
    if not isinstance(result['merged'], bool):
        raise PreflightError('upstream pull request has invalid merged state')
    return result


def _empty_upstream_pull_request(url: str | None) -> dict[str, Any]:
    return {
        'number': None,
        'url': url,
        'state': None,
        'draft': None,
        'merged': None,
        'head_sha': None,
        'head_repository': None,
        'base_branch': None,
        'base_repository': None,
    }


def build_review_response_packet(
    report: dict[str, Any],
    upstream_pr_url: str | None,
    *,
    upstream_pull_request: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Unlock copy-ready rosdistro replies only after exact identity checks."""
    blockers: list[str] = []
    remote = report['remote']
    if report['mode'] != 'online' or remote.get('inspected') is not True:
        blockers.append('Run the online release audit before preparing replies.')
    if report['local'].get('ready') is not True:
        blockers.append('The pinned ndt_omp_ros2 release candidate is not ready.')
    if remote.get('errors'):
        blockers.append('Remote inspection contains errors.')
    if remote.get('source_tag_present') is not True:
        blockers.append('The pinned ndt_omp_ros2 source tag is not present.')
    if remote.get('release_repository_present') is not True:
        blockers.append('The ndt_omp_ros2 release repository is not present.')

    response_targets: dict[str, dict[str, Any]] = {}
    for distro in DISTROS:
        pull_request = remote.get('pull_requests', {}).get(distro, {})
        expected_number = ROSDISTRO_PULL_REQUESTS[distro]
        expected_url = (
            f'https://github.com/ros/rosdistro/pull/{expected_number}')
        expected_head = ROSDISTRO_PULL_REQUEST_HEADS[distro]
        expected_review = ROSDISTRO_REVIEW_URLS[distro]
        response_targets[distro] = {
            'number': pull_request.get('number'),
            'url': pull_request.get('url'),
            'head_sha': pull_request.get('head_sha'),
            'review_url': pull_request.get(
                'latest_actionable_review', {}).get('url'),
            'response': None,
        }
        if remote.get('rosdistro', {}).get(distro) is not False:
            blockers.append(
                f'{distro} rosdistro publication state is no longer absent.')
        if (
            pull_request.get('number') != expected_number
            or pull_request.get('url') != expected_url
            or pull_request.get('state') != 'open'
            or pull_request.get('merged') is not False
            or pull_request.get('head_sha') != expected_head
        ):
            blockers.append(
                f'{distro} rosdistro PR identity or exact head drifted.')
        if (
            pull_request.get('response_pending') is not True
            or pull_request.get('latest_actionable_review', {}).get('url')
            != expected_review
        ):
            blockers.append(
                f'{distro} actionable review is absent, answered, or changed.')

    inspected_upstream = _empty_upstream_pull_request(upstream_pr_url)
    if not upstream_pr_url:
        blockers.append('A verified canonical upstream Draft PR URL is required.')
    else:
        try:
            inspected_upstream = (
                _inspect_upstream_pull_request(upstream_pr_url)
                if upstream_pull_request is None
                else dict(upstream_pull_request)
            )
        except PreflightError as exc:
            blockers.append(f'Canonical upstream PR inspection failed: {exc}')

    expected_upstream = {
        'url': upstream_pr_url,
        'state': 'open',
        'draft': True,
        'merged': False,
        'head_sha': UPSTREAM_CANDIDATE_COMMIT,
        'head_repository': UPSTREAM_FORK_REPOSITORY,
        'base_branch': UPSTREAM_BASE_BRANCH,
        'base_repository': UPSTREAM_REPOSITORY,
    }
    if upstream_pr_url and any(
        inspected_upstream.get(field) != value
        for field, value in expected_upstream.items()
    ):
        blockers.append(
            'Canonical upstream PR is not the expected open Draft at the '
            'exact candidate commit and repository boundary.')

    if not blockers:
        jazzy_response = (
            'Thanks for catching this. `ndt_omp_ros2` is a downstream ROS 2 '
            'fork of `koide3/ndt_omp`, not an independent implementation. It '
            "carries four APIs used by lidarslam's optional IMU/translation "
            'priors and adaptive correspondence threshold, plus buildfarm '
            'packaging work. However, the current candidate still installs '
            'the same `include/pclomp/*` headers and `libndt_omp.so` as '
            "Humble's released `ndt_omp`, so the differently named Debian "
            'packages are not safely co-installable. I do not want these PRs '
            'merged as-is. My preferred correction is to upstream the '
            'required APIs and consume/release the canonical `ndt_omp` '
            'package for Humble and Jazzy. The focused upstream work is '
            f'tracked in Draft PR {upstream_pr_url}. If those '
            'project-specific APIs are declined upstream, I will instead '
            "fully namespace the fork's package, headers, symbols, library, "
            'and CMake target, then replace these Bloom registrations. I will '
            'report the selected collision-free path here before requesting '
            'another merge review. The current red rosdep check also remains '
            'a hard gate; any replacement registration will be generated '
            'from current rosdistro `master` and must be fully green.'
        )
        humble_response = (
            'The same lineage and co-installation issue applies here as in '
            'Jazzy #52950. `ndt_omp_ros2` is a downstream fork that overlaps '
            'the canonical `ndt_omp` headers and library, so please do not '
            'merge this registration as-is. I am pursuing the collision-free '
            f'canonical path in Draft PR {upstream_pr_url} and will '
            'return with the accepted resolution before requesting another '
            'rosdistro review. Any replacement PR will be generated from '
            'current rosdistro `master` and must pass its complete check '
            'suite.'
        )
        response_targets['jazzy']['response'] = jazzy_response
        response_targets['humble']['response'] = humble_response

    return {
        'packet_version': 1,
        'status': (
            'READY_FOR_MAINTAINER_POST' if not blockers else 'BLOCKED'),
        'authority': {
            'github_writes_authorized': False,
            'remote_mutations_performed': False,
        },
        'upstream_candidate_commit': UPSTREAM_CANDIDATE_COMMIT,
        'upstream_pull_request': inspected_upstream,
        'rosdistro_pull_requests': response_targets,
        'blockers': blockers,
    }


def _unanswered_review_action(
    distro: str,
    pull_request: dict[str, Any],
) -> str:
    review = pull_request['latest_actionable_review']
    return (
        f'Respond to the unanswered human review for ros/rosdistro PR '
        f'#{pull_request["number"]} ({distro}) at {review["url"]}. Explain '
        'the upstream lineage, required API delta, and collision-free '
        'convergence plan; do not describe this as wait-only or rerun Bloom.'
    )


def evaluate_readiness(
    *,
    repo_root: Path = REPO_ROOT,
    offline: bool = False,
    local: dict[str, Any] | None = None,
    remote: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Return a deterministic report from local and optional remote state."""
    local = inspect_local(repo_root) if local is None else local
    actions: list[str] = []

    if not local['ready']:
        status = 'BLOCKED'
        actions.append('Repair every failed local candidate check before tagging.')
        remote_report = {
            'inspected': False,
            'errors': [],
            'origin_branch_commit': None,
            'source_tag_present': None,
            'release_repository_present': None,
            'rosdistro': {distro: None for distro in DISTROS},
            'pull_requests': _empty_pull_requests(),
        }
    elif offline:
        status = 'LOCAL_READY'
        actions.append(
            'Run again without --offline before creating the source tag.')
        remote_report = {
            'inspected': False,
            'errors': [],
            'origin_branch_commit': None,
            'source_tag_present': None,
            'release_repository_present': None,
            'rosdistro': {distro: None for distro in DISTROS},
            'pull_requests': _empty_pull_requests(),
        }
    else:
        remote_report = inspect_remote() if remote is None else dict(remote)
        remote_report.setdefault('pull_requests', _empty_pull_requests())
        remote_report['inspected'] = True
        artifacts = [
            remote_report.get('source_tag_present'),
            remote_report.get('release_repository_present'),
            remote_report.get('rosdistro', {}).get('humble'),
            remote_report.get('rosdistro', {}).get('jazzy'),
        ]
        remote_failed = (
            bool(remote_report.get('errors'))
            or remote_report.get('origin_branch_commit') != EXPECTED_COMMIT
            or any(value is None for value in artifacts)
        )
        generated_prs_required = (
            remote_report.get('source_tag_present') is True
            and remote_report.get('release_repository_present') is True
            and not all(remote_report.get('rosdistro', {}).values())
        )
        if generated_prs_required:
            for distro in DISTROS:
                if remote_report['rosdistro'][distro]:
                    continue
                pull_request = remote_report['pull_requests'].get(distro, {})
                if (
                    pull_request.get('state') is None
                    or pull_request.get('merged') is None
                    or pull_request.get('response_pending') is None
                ):
                    remote_failed = True
                check_runs = pull_request.get('check_runs')
                count_fields = (
                    'total_count',
                    'passing_count',
                    'pending_count',
                    'failing_count',
                )
                if (
                    not isinstance(check_runs, dict)
                    or check_runs.get('inspected') is not True
                    or not isinstance(check_runs.get('runs'), list)
                    or not all(
                        isinstance(check_runs.get(field), int)
                        and not isinstance(check_runs.get(field), bool)
                        for field in count_fields
                    )
                ):
                    remote_failed = True
                elif (
                    check_runs['total_count'] != len(check_runs.get('runs', []))
                    or check_runs['total_count'] != sum(
                        check_runs[field] for field in count_fields[1:])
                ):
                    remote_failed = True
        closed_unmerged = any(
            not remote_report['rosdistro'][distro]
            and remote_report['pull_requests'][distro]['state'] == 'closed'
            and remote_report['pull_requests'][distro]['merged'] is False
            for distro in DISTROS
        ) if not remote_failed else False
        unmergeable = any(
            not remote_report['rosdistro'][distro]
            and remote_report['pull_requests'][distro]['state'] == 'open'
            and remote_report['pull_requests'][distro]['mergeable'] is False
            for distro in DISTROS
        ) if not remote_failed else False
        mergeability_pending = any(
            not remote_report['rosdistro'][distro]
            and remote_report['pull_requests'][distro]['state'] == 'open'
            and remote_report['pull_requests'][distro]['mergeable'] is None
            for distro in DISTROS
        ) if not remote_failed else False
        review_pending = any(
            not remote_report['rosdistro'][distro]
            and remote_report['pull_requests'][distro]['response_pending']
            is True
            for distro in DISTROS
        ) if not remote_failed else False
        failing_checks = any(
            not remote_report['rosdistro'][distro]
            and remote_report['pull_requests'][distro]['state'] == 'open'
            and remote_report['pull_requests'][distro]['check_runs'][
                'failing_count'] > 0
            for distro in DISTROS
        ) if not remote_failed else False
        pending_checks = any(
            not remote_report['rosdistro'][distro]
            and remote_report['pull_requests'][distro]['state'] == 'open'
            and remote_report['pull_requests'][distro]['check_runs'][
                'pending_count'] > 0
            for distro in DISTROS
        ) if not remote_failed else False
        missing_checks = any(
            not remote_report['rosdistro'][distro]
            and remote_report['pull_requests'][distro]['state'] == 'open'
            and remote_report['pull_requests'][distro]['check_runs'][
                'total_count'] == 0
            for distro in DISTROS
        ) if not remote_failed else False
        if remote_failed:
            status = 'BLOCKED'
            actions.append(
                'Resolve remote inspection failures or candidate drift; '
                'do not publish from this report.')
        elif all(value is False for value in artifacts):
            status = 'READY_TO_TAG'
            actions.append(
                'Create and push source tag 0.1.0, then follow the Bloom runbook.')
        elif all(value is True for value in artifacts):
            status = 'RELEASED'
            actions.append(
                'Proceed with the lidarslam_ros2 distribution gate.')
        elif closed_unmerged:
            status = 'BLOCKED'
            for distro in DISTROS:
                pull_request = remote_report['pull_requests'][distro]
                if (
                    not remote_report['rosdistro'][distro]
                    and pull_request['state'] == 'closed'
                    and pull_request['merged'] is False
                ):
                    actions.append(
                        f'ros/rosdistro PR #{pull_request["number"]} '
                        f'({distro}) closed without merge. Resolve the '
                        'review outcome before creating replacement release '
                        'state.')
        elif unmergeable:
            status = 'BLOCKED'
            for distro in DISTROS:
                pull_request = remote_report['pull_requests'][distro]
                if (
                    not remote_report['rosdistro'][distro]
                    and pull_request['state'] == 'open'
                    and pull_request['mergeable'] is False
                ):
                    actions.append(
                        f'ros/rosdistro PR #{pull_request["number"]} '
                        f'({distro}) is not mergeable at exact head '
                        f'{pull_request["head_sha"]}. Resolve the base '
                        'conflict and rerun this audit; do not merge or '
                        'rerun Bloom from this state.')
            for distro in DISTROS:
                pull_request = remote_report['pull_requests'][distro]
                if (
                    not remote_report['rosdistro'][distro]
                    and pull_request['response_pending'] is True
                ):
                    actions.append(_unanswered_review_action(
                        distro, pull_request))
        elif failing_checks or pending_checks or missing_checks:
            status = 'BLOCKED'
            for distro in DISTROS:
                if remote_report['rosdistro'][distro]:
                    continue
                pull_request = remote_report['pull_requests'][distro]
                if pull_request['state'] != 'open':
                    continue
                check_runs = pull_request['check_runs']
                if check_runs['failing_count']:
                    failed_names = ', '.join(
                        item['name'] for item in check_runs['runs']
                        if item['classification'] == 'FAILING'
                    )
                    actions.append(
                        f'ros/rosdistro PR #{pull_request["number"]} '
                        f'({distro}) has {check_runs["failing_count"]} '
                        f'failing check run at exact head '
                        f'{pull_request["head_sha"]}: {failed_names}. '
                        'After selecting the collision-free convergence '
                        'path, refresh or recreate the generated PR from '
                        'current rosdistro master and require all checks to '
                        'pass; do not merge, rerun Bloom, or claim green CI '
                        'from this state.')
                if check_runs['pending_count']:
                    actions.append(
                        f'ros/rosdistro PR #{pull_request["number"]} '
                        f'({distro}) has {check_runs["pending_count"]} '
                        'pending check run at exact head '
                        f'{pull_request["head_sha"]}. Rerun this read-only '
                        'audit after completion; do not treat the PR as '
                        'green or wait-only yet.')
                if check_runs['total_count'] == 0:
                    actions.append(
                        f'ros/rosdistro PR #{pull_request["number"]} '
                        f'({distro}) has no check-run evidence at exact head '
                        f'{pull_request["head_sha"]}. Obtain a complete '
                        'green check suite before requesting merge.')
            for distro in DISTROS:
                pull_request = remote_report['pull_requests'][distro]
                if (
                    not remote_report['rosdistro'][distro]
                    and pull_request['response_pending'] is True
                ):
                    actions.append(_unanswered_review_action(
                        distro, pull_request))
        elif review_pending:
            status = 'REVIEW_REQUIRED'
            for distro in DISTROS:
                pull_request = remote_report['pull_requests'][distro]
                if (
                    not remote_report['rosdistro'][distro]
                    and pull_request['response_pending'] is True
                ):
                    actions.append(_unanswered_review_action(
                        distro, pull_request))
        elif mergeability_pending:
            status = 'BLOCKED'
            for distro in DISTROS:
                pull_request = remote_report['pull_requests'][distro]
                if (
                    not remote_report['rosdistro'][distro]
                    and pull_request['state'] == 'open'
                    and pull_request['mergeable'] is None
                ):
                    actions.append(
                        f'GitHub has not resolved mergeability for '
                        f'ros/rosdistro PR #{pull_request["number"]} '
                        f'({distro}) at exact head '
                        f'{pull_request["head_sha"]}. Rerun the read-only '
                        'audit before treating the PR as wait-only or '
                        'mergeable.')
        else:
            status = 'IN_PROGRESS'
            if not remote_report['source_tag_present']:
                actions.append('Create source tag 0.1.0.')
            if not remote_report['release_repository_present']:
                actions.append('Create and initialize ndt_omp_ros2-release.')
            for distro in DISTROS:
                if not remote_report['rosdistro'][distro]:
                    if (
                        remote_report['source_tag_present']
                        and remote_report['release_repository_present']
                    ):
                        pr_number = ROSDISTRO_PULL_REQUESTS[distro]
                        actions.append(
                            f'Wait for ros/rosdistro PR #{pr_number} '
                            f'({distro}) to merge; do not recreate the source '
                            'tag or rerun Bloom while this generated PR is '
                            'current.'
                        )
                    else:
                        actions.append(
                            f'After the source tag and release repository are '
                            f'ready, Bloom-release ndt_omp_ros2 for {distro} '
                            'and merge the generated rosdistro entry.'
                        )

    report = {
        'schema_version': 2,
        'schema_uri': SCHEMA_URI,
        'package': 'ndt_omp_ros2',
        'candidate': {
            'version': EXPECTED_VERSION,
            'commit': EXPECTED_COMMIT,
        },
        'mode': 'offline' if offline else 'online',
        'status': status,
        'local': local,
        'remote': remote_report,
        'actions': actions,
    }
    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.Draft7Validator(schema).validate(report)
    return report


def _summary(report: dict[str, Any]) -> str:
    lines = [
        f"NDT release preflight: {report['status']}",
        f"Candidate: {report['candidate']['version']} "
        f"({report['candidate']['commit']})",
    ]
    for item in report['local']['checks']:
        lines.append(f"  [{item['status']}] {item['id']}: {item['detail']}")
    if report['remote']['inspected']:
        remote = report['remote']
        lines.extend([
            f"Remote branch: {remote['origin_branch_commit']}",
            f"Source tag present: {remote['source_tag_present']}",
            f'Release repository present: '
            f"{remote['release_repository_present']}",
            f"rosdistro: {remote['rosdistro']}",
        ])
        for distro in DISTROS:
            pull_request = remote['pull_requests'][distro]
            lines.append(
                f"rosdistro PR {distro}: #{pull_request['number']} "
                f"state={pull_request['state']} merged="
                f"{pull_request['merged']} mergeable="
                f"{pull_request['mergeable']} response_pending="
                f"{pull_request['response_pending']} checks="
                f"{pull_request['check_runs']['passing_count']}/"
                f"{pull_request['check_runs']['total_count']} passing "
                f"pending={pull_request['check_runs']['pending_count']} "
                f"failing={pull_request['check_runs']['failing_count']}")
        lines.extend(f'  [ERROR] {error}' for error in remote['errors'])
    lines.extend(f'Next: {action}' for action in report['actions'])
    return '\n'.join(lines)


def _review_response_summary(packet: dict[str, Any]) -> str:
    lines = [
        f"NDT rosdistro review response packet: {packet['status']}",
        'GitHub writes authorized: false',
        'Remote mutations performed: false',
        f"Upstream candidate: {packet['upstream_candidate_commit']}",
        f"Upstream Draft PR: {packet['upstream_pull_request']['url']}",
    ]
    if packet['blockers']:
        lines.extend(f'  [BLOCKER] {item}' for item in packet['blockers'])
        lines.append('No copy-ready response was emitted.')
        return '\n'.join(lines)

    for distro in ('jazzy', 'humble'):
        target = packet['rosdistro_pull_requests'][distro]
        lines.extend([
            '',
            f"{distro.title()} PR #{target['number']} response",
            f"Target: {target['url']}",
            f"Review: {target['review_url']}",
            f"Exact head: {target['head_sha']}",
            '',
            target['response'],
        ])
    lines.extend([
        '',
        'This packet is copy-ready evidence only; posting remains a separate '
        'maintainer action.',
    ])
    return '\n'.join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--offline',
        action='store_true',
        help='validate only the pinned local release candidate',
    )
    parser.add_argument('--json', action='store_true', help='print JSON')
    parser.add_argument('--output-json', type=Path)
    parser.add_argument(
        '--review-response-packet',
        action='store_true',
        help=(
            'render fail-closed rosdistro review replies instead of the '
            'release report; no GitHub write is performed'
        ),
    )
    parser.add_argument(
        '--upstream-pr-url',
        help=(
            'canonical koide3/ndt_omp Draft PR URL to verify before '
            'unlocking review replies'
        ),
    )
    strict = parser.add_mutually_exclusive_group()
    strict.add_argument('--require-ready-to-tag', action='store_true')
    strict.add_argument('--require-released', action='store_true')
    strict.add_argument(
        '--require-review-response-ready',
        action='store_true',
        help='exit nonzero unless the exact review response packet is ready',
    )
    args = parser.parse_args(argv)
    packet_mode = (
        args.review_response_packet or args.require_review_response_ready)
    if args.upstream_pr_url and not packet_mode:
        parser.error(
            '--upstream-pr-url requires --review-response-packet or '
            '--require-review-response-ready')
    try:
        report = evaluate_readiness(offline=args.offline)
        output = report
        if packet_mode:
            output = build_review_response_packet(
                report, args.upstream_pr_url)
        rendered = json.dumps(output, indent=2, sort_keys=True) + '\n'
        if args.output_json:
            args.output_json.write_text(rendered, encoding='utf-8')
        summary = (
            _review_response_summary(output)
            if packet_mode else _summary(report)
        )
        print(rendered if args.json else summary)
    except (
        OSError,
        PreflightError,
        json.JSONDecodeError,
        jsonschema.SchemaError,
        jsonschema.ValidationError,
    ) as exc:
        print(f'ndt release preflight error: {exc}', file=sys.stderr)
        return 2

    if args.require_ready_to_tag and report['status'] != 'READY_TO_TAG':
        return 1
    if args.require_released and report['status'] != 'RELEASED':
        return 1
    if (
        args.require_review_response_ready
        and output['status'] != 'READY_FOR_MAINTAINER_POST'
    ):
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
