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
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Validate the collision-free canonical ndt_omp convergence bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys
from typing import Any, Sequence
import urllib.error
import urllib.parse
import urllib.request

import jsonschema

try:
    from github_api_auth import (
        github_api_authorization,
        is_github_api_url,
    )
except ModuleNotFoundError:  # pragma: no cover - importlib test path
    from scripts.github_api_auth import (
        github_api_authorization,
        is_github_api_url,
    )


sys.dont_write_bytecode = True

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = (
    REPO_ROOT / 'docs' / 'contracts' / 'canonical-ndt-convergence-v1.json'
)
CONTRACT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'canonical-ndt-convergence-contract-v1.schema.json'
)
REPORT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'canonical-ndt-convergence-readiness-v1.schema.json'
)
REPORT_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/canonical-ndt-convergence-readiness-v1.schema.json'
)
PATCH_HEADER = re.compile(r'^diff --git a/(.+) b/(.+)$')


class ConvergenceError(ValueError):
    """The convergence contract or artifact cannot be trusted."""


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ConvergenceError(f'cannot read {label} {path}: {exc}') from exc
    if not isinstance(payload, dict):
        raise ConvergenceError(f'{label} must be a JSON object')
    return payload


def _schema_error_path(error: jsonschema.ValidationError) -> str:
    path = '.'.join(str(item) for item in error.absolute_path)
    return path or '<root>'


def _validate_schema(
    payload: dict[str, Any],
    schema: dict[str, Any],
    label: str,
) -> None:
    jsonschema.Draft7Validator.check_schema(schema)
    errors = sorted(
        jsonschema.Draft7Validator(schema).iter_errors(payload),
        key=lambda item: [str(part) for part in item.absolute_path],
    )
    if errors:
        first = errors[0]
        raise ConvergenceError(
            f'{label} schema failed at {_schema_error_path(first)}: '
            f'{first.message}')


def _validate_repo_path(value: str) -> None:
    candidate = PurePosixPath(value)
    if (
        not value
        or value.startswith('/')
        or '\\' in value
        or '\n' in value
        or '\r' in value
        or '..' in candidate.parts
        or str(candidate) != value
        or value.startswith('./')
    ):
        raise ConvergenceError(
            f'unsafe or non-canonical repository path: {value!r}')


def _artifact_path(repo_root: Path, relative: str) -> Path:
    _validate_repo_path(relative)
    root = repo_root.resolve()
    path = (root / relative).resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ConvergenceError(
            f'artifact escapes repository root: {relative}') from exc
    if not path.is_file():
        raise ConvergenceError(f'artifact is not a regular file: {relative}')
    return path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _run(
    command: Sequence[str],
    *,
    cwd: Path,
    label: str,
) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            list(command),
            cwd=cwd,
            check=False,
            capture_output=True,
            text=True,
            timeout=60,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ConvergenceError(f'cannot run {label}: {exc}') from exc


def _apply_check(repo: Path, patch: Path) -> tuple[bool, str]:
    result = _run(
        ['git', 'apply', '--check', '--binary', str(patch)],
        cwd=repo,
        label='read-only git apply check',
    )
    detail = result.stderr.strip() or result.stdout.strip()
    if result.returncode == 0:
        return True, 'patch applies without modifying the checkout'
    return False, detail or f'git apply returned {result.returncode}'


def _git_text(repo: Path, *arguments: str) -> str:
    result = _run(
        ['git', *arguments],
        cwd=repo,
        label=f"git {' '.join(arguments)}",
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise ConvergenceError(
            f"git {' '.join(arguments)} failed: "
            f'{detail or result.returncode}')
    return result.stdout.strip()


def _git_diff_sha256(repo: Path, base: str, commit: str) -> str:
    try:
        result = subprocess.run(
            ['git', 'diff', '--binary', f'{base}..{commit}'],
            cwd=repo,
            check=False,
            capture_output=True,
            timeout=60,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ConvergenceError(
            f'cannot hash candidate git diff: {exc}') from exc
    if result.returncode != 0:
        detail = result.stderr.decode('utf-8', errors='replace').strip()
        raise ConvergenceError(
            f'candidate git diff failed: {detail or result.returncode}')
    return hashlib.sha256(result.stdout).hexdigest()


def _request_json(url: str, *, allow_404: bool = False) -> tuple[int, Any]:
    if not is_github_api_url(url):
        raise ConvergenceError(f'refusing non-GitHub API URL: {url}')
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'lidarslam-canonical-ndt-preflight/1',
    }
    headers.update(github_api_authorization(url, method='GET'))
    request = urllib.request.Request(url, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            status = response.status
            body = response.read().decode('utf-8')
    except urllib.error.HTTPError as exc:
        if allow_404 and exc.code == 404:
            return 404, None
        raise ConvergenceError(f'HTTP {exc.code} while reading {url}') from exc
    except (urllib.error.URLError, TimeoutError, UnicodeDecodeError) as exc:
        raise ConvergenceError(f'cannot read {url}: {exc}') from exc
    if status != 200:
        raise ConvergenceError(f'unexpected HTTP {status} while reading {url}')
    try:
        return status, json.loads(body)
    except json.JSONDecodeError as exc:
        raise ConvergenceError(
            f'invalid JSON while reading {url}: {exc}') from exc


def _inspect_publication_state(contract: dict[str, Any]) -> dict[str, Any]:
    publication = contract['publication']
    upstream_repository = publication['upstream_repository']
    fork_repository = publication['fork_repository']
    proposed_branch = publication['proposed_branch']
    api_root = 'https://api.github.com/repos'
    report: dict[str, Any] = {
        'inspected': True,
        'upstream_repository': upstream_repository,
        'fork_repository': fork_repository,
        'proposed_branch': proposed_branch,
        'errors': [],
        'upstream_head_sha': None,
        'fork_is_expected': None,
        'proposed_branch_present': None,
        'open_pr_count': None,
        'duplicate_prs': [],
    }

    try:
        _, payload = _request_json(
            f'{api_root}/{upstream_repository}/git/ref/heads/'
            f'{urllib.parse.quote(contract["upstream"]["branch"], safe="")}')
        sha = payload.get('object', {}).get('sha') \
            if isinstance(payload, dict) else None
        if not isinstance(sha, str):
            raise ConvergenceError(
                'upstream branch response has no commit SHA')
        report['upstream_head_sha'] = sha
    except ConvergenceError as exc:
        report['errors'].append(f'upstream branch: {exc}')

    try:
        _, payload = _request_json(f'{api_root}/{fork_repository}')
        parent = payload.get('parent') if isinstance(payload, dict) else None
        parent_name = (
            parent.get('full_name') if isinstance(parent, dict) else None)
        report['fork_is_expected'] = bool(
            isinstance(payload, dict)
            and payload.get('fork') is True
            and parent_name == upstream_repository
        )
    except ConvergenceError as exc:
        report['errors'].append(f'fork identity: {exc}')

    try:
        status, payload = _request_json(
            f'{api_root}/{fork_repository}/git/ref/heads/'
            f'{urllib.parse.quote(proposed_branch, safe="")}',
            allow_404=True,
        )
        if status == 404:
            report['proposed_branch_present'] = False
        elif isinstance(payload, dict):
            report['proposed_branch_present'] = True
        else:
            raise ConvergenceError('proposed branch response is malformed')
    except ConvergenceError as exc:
        report['errors'].append(f'proposed branch: {exc}')

    try:
        open_prs: list[dict[str, Any]] = []
        encoded_branch = urllib.parse.quote(
            contract['upstream']['branch'], safe='')
        for page in range(1, 11):
            _, payload = _request_json(
                f'{api_root}/{upstream_repository}/pulls?state=open&'
                f'base={encoded_branch}'
                f'&per_page=100&page={page}')
            if not isinstance(payload, list):
                raise ConvergenceError(
                    'open pull request response is not a list')
            if not all(isinstance(item, dict) for item in payload):
                raise ConvergenceError('open pull request list is malformed')
            open_prs.extend(payload)
            if len(payload) < 100:
                break
        else:
            raise ConvergenceError(
                'open pull request pagination exceeded 1000')

        report['open_pr_count'] = len(open_prs)
        terms = [
            item.lower() for item in publication['duplicate_search_terms']]
        duplicates = []
        for pull_request in open_prs:
            number = pull_request.get('number')
            title = pull_request.get('title')
            body = pull_request.get('body') or ''
            url = pull_request.get('html_url')
            head = pull_request.get('head')
            if (
                not isinstance(number, int)
                or not isinstance(title, str)
                or not isinstance(body, str)
                or not isinstance(url, str)
                or not isinstance(head, dict)
            ):
                raise ConvergenceError(
                    'open pull request identity is incomplete')
            head_ref = head.get('ref')
            head_label = head.get('label')
            head_sha = head.get('sha')
            head_repo = head.get('repo')
            head_repo_name = (
                head_repo.get('full_name')
                if isinstance(head_repo, dict) else None
            )
            if not all(isinstance(item, str) and item for item in (
                    head_ref, head_label, head_sha)):
                raise ConvergenceError('open pull request head is incomplete')
            searchable = f'{title}\n{body}\n{head_ref}'.lower()
            branch_match = (
                head_ref == proposed_branch
                and head_repo_name == fork_repository
            )
            if branch_match or any(term in searchable for term in terms):
                duplicates.append({
                    'number': number,
                    'title': title,
                    'url': url,
                    'head_label': head_label,
                    'head_sha': head_sha,
                })
        report['duplicate_prs'] = sorted(
            duplicates, key=lambda item: item['number'])
    except ConvergenceError as exc:
        report['errors'].append(f'open pull requests: {exc}')

    return report


def _parse_patch(path: Path) -> dict[str, Any]:
    try:
        content = path.read_text(encoding='utf-8')
    except (OSError, UnicodeDecodeError) as exc:
        raise ConvergenceError(f'cannot read patch {path.name}: {exc}') from exc
    if 'GIT binary patch' in content or '\x00' in content:
        raise ConvergenceError(f'binary patch is not permitted: {path.name}')

    paths: list[str] = []
    added: dict[str, list[str]] = {}
    deleted: dict[str, list[str]] = {}
    current: str | None = None
    in_hunk = False
    for line in content.splitlines():
        header = PATCH_HEADER.match(line)
        if header:
            left, right = header.groups()
            if left != right:
                raise ConvergenceError(
                    f'rename patches are not permitted: {left} -> {right}')
            _validate_repo_path(left)
            if left in added:
                raise ConvergenceError(f'duplicate patch path: {left}')
            current = left
            paths.append(left)
            added[left] = []
            deleted[left] = []
            in_hunk = False
            continue
        if line.startswith('@@ '):
            if current is None:
                raise ConvergenceError('patch hunk appears before a file header')
            in_hunk = True
            continue
        if not in_hunk or current is None:
            continue
        if line.startswith('+') and not line.startswith('+++'):
            added[current].append(line[1:])
        elif line.startswith('-') and not line.startswith('---'):
            deleted[current].append(line[1:])

    if not paths:
        raise ConvergenceError(f'patch contains no paths: {path.name}')
    return {
        'content': content,
        'paths': paths,
        'added': added,
        'deleted': deleted,
        'additions': sum(len(lines) for lines in added.values()),
        'deletions': sum(len(lines) for lines in deleted.values()),
    }


def _check(check_id: str, passed: bool, detail: str) -> dict[str, str]:
    return {
        'id': check_id,
        'status': 'PASS' if passed else 'FAIL',
        'detail': detail,
    }


def _not_checked(check_id: str, detail: str) -> dict[str, str]:
    return {'id': check_id, 'status': 'NOT_CHECKED', 'detail': detail}


def _draft_pr_handoff(
    contract: dict[str, Any],
    publication_report: dict[str, Any],
) -> dict[str, Any]:
    """Bind one no-write handoff to the exact verified Draft PR candidate."""
    publication = contract['publication']
    upstream = contract['upstream']
    draft_pr = publication['draft_pr']
    upstream_repository = publication['upstream_repository']
    fork_repository = publication['fork_repository']
    base_branch = upstream['branch']
    head_branch = publication['proposed_branch']
    candidate_commit = publication['candidate_commit']
    return {
        'kind': 'CREATE_CANONICAL_NDT_DRAFT_PR',
        'external_write_required': True,
        'upstream_repository': upstream_repository,
        'base_branch': base_branch,
        'fork_repository': fork_repository,
        'head_branch': head_branch,
        'upstream_base_commit': upstream['base_commit'],
        'candidate_commit': candidate_commit,
        'candidate_subject': publication['candidate_subject'],
        'title': draft_pr['title'],
        'body': draft_pr['body'],
        'preflight': {
            'upstream_base_verified': True,
            'fork_identity_verified': True,
            'branch_absent_verified': True,
            'open_pr_count': publication_report['open_pr_count'],
            'duplicate_pr_count': len(publication_report['duplicate_prs']),
        },
        'constraints': {
            'draft_required': draft_pr['draft_required'],
            'create_only': True,
            'non_force_only': True,
            'push_authorized': False,
            'pr_creation_authorized': False,
            'force_push_authorized': False,
            'mark_ready_authorized': False,
            'merge_authorized': False,
        },
        'steps': [
            'Obtain separate authority for the exact candidate commit and '
            'create-only branch plus Draft PR scope.',
            f'Create {fork_repository}:{head_branch} at exact commit '
            f'{candidate_commit} with a non-force push; abort if the branch '
            'now exists or any verified identity has drifted.',
            f'Open a Draft PR to {upstream_repository}:{base_branch} with '
            'the exact title and body in this handoff; do not mark it ready '
            'or merge it.',
            'GET-only verify the public Draft identity and exact head, then '
            'provide its canonical URL to the rosdistro response packet.',
        ],
        'writes_performed': False,
    }


def _validate_artifact(
    *,
    repo_root: Path,
    artifact: dict[str, Any],
    label: str,
    checks: list[dict[str, str]],
) -> tuple[Path, dict[str, Any]]:
    path = _artifact_path(repo_root, artifact['path'])
    observed_hash = _sha256(path)
    checks.append(_check(
        f'{label}-sha256',
        observed_hash == artifact['sha256'],
        f"expected {artifact['sha256']}; found {observed_hash}",
    ))
    parsed = _parse_patch(path)
    expected_paths = artifact['paths']
    paths_match = (
        expected_paths == sorted(expected_paths)
        and parsed['paths'] == expected_paths
    )
    checks.append(_check(
        f'{label}-paths',
        paths_match,
        f'expected {expected_paths}; found {parsed["paths"]}',
    ))
    stats_match = (
        parsed['additions'] == artifact['additions']
        and parsed['deletions'] == artifact['deletions']
    )
    checks.append(_check(
        f'{label}-stats',
        stats_match,
        f"expected +{artifact['additions']}/-{artifact['deletions']}; "
        f"found +{parsed['additions']}/-{parsed['deletions']}",
    ))
    return path, parsed


def evaluate(
    *,
    repo_root: Path = REPO_ROOT,
    contract_path: Path = DEFAULT_CONTRACT,
    contract_schema_path: Path = CONTRACT_SCHEMA,
    report_schema_path: Path = REPORT_SCHEMA,
    upstream_checkout: Path | None = None,
    candidate_checkout: Path | None = None,
    online: bool = False,
    publication_state: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Validate artifacts, exact checkouts, and optional public PR state."""
    repo_root = repo_root.resolve()
    contract = _load_json(contract_path, 'convergence contract')
    contract_schema = _load_json(contract_schema_path, 'contract schema')
    _validate_schema(contract, contract_schema, 'contract')
    checks: list[dict[str, str]] = [_check(
        'contract-schema', True, 'contract matches the v1 schema')]

    upstream_patch, upstream = _validate_artifact(
        repo_root=repo_root,
        artifact=contract['upstream']['patch'],
        label='upstream-patch',
        checks=checks,
    )
    required_markers = contract['upstream']['required_api_markers']
    added_upstream = '\n'.join(
        line for lines in upstream['added'].values() for line in lines)
    missing_markers = [
        marker for marker in required_markers if marker not in added_upstream]
    checks.append(_check(
        'upstream-api-surface',
        not missing_markers,
        'all required APIs are present' if not missing_markers else
        f'missing API markers: {missing_markers}',
    ))
    missing_tests = [
        name for name in contract['upstream']['focused_tests']
        if name not in added_upstream
    ]
    checks.append(_check(
        'upstream-focused-tests',
        not missing_tests,
        'all focused test cases are present' if not missing_tests else
        f'missing focused tests: {missing_tests}',
    ))

    parent_patch, parent = _validate_artifact(
        repo_root=repo_root,
        artifact=contract['parent_transition']['patch'],
        label='parent-patch',
        checks=checks,
    )
    parent_applies, parent_detail = _apply_check(repo_root, parent_patch)
    checks.append(_check(
        'parent-patch-applies', parent_applies, parent_detail))

    before = contract['parent_transition']['provider_before']
    after = contract['parent_transition']['provider_after']
    for replacement in contract['parent_transition']['consumer_replacements']:
        relative = replacement['path']
        source_path = _artifact_path(repo_root, relative)
        source = source_path.read_text(encoding='utf-8')
        current_count = source.count(before)
        deleted_count = sum(
            line.count(before) for line in parent['deleted'][relative])
        added_before_count = sum(
            line.count(before) for line in parent['added'][relative])
        added_after_count = sum(
            line.count(after) for line in parent['added'][relative])
        virtual_after_count = (
            current_count - deleted_count + added_before_count)
        expected = replacement['before_count']
        passed = (
            current_count == expected
            and deleted_count == expected
            and added_before_count == 0
            and added_after_count == expected
            and virtual_after_count == replacement['after_count']
        )
        checks.append(_check(
            f'consumer-{relative}',
            passed,
            f'current={current_count}, deleted={deleted_count}, '
            f'canonical-added={added_after_count}, '
            f'virtual-fork-remaining={virtual_after_count}',
        ))

    spelling = contract['parent_transition']['spelling_correction']
    spelling_path = spelling['path']
    spelling_source = _artifact_path(repo_root, spelling_path).read_text(
        encoding='utf-8')
    spelling_ok = (
        spelling_source.count(spelling['before']) == 1
        and spelling_source.count(spelling['after']) == 0
        and sum(
            line.count(spelling['before'])
            for line in parent['deleted'][spelling_path]
        ) == 1
        and sum(
            line.count(spelling['after'])
            for line in parent['added'][spelling_path]
        ) == 1
    )
    checks.append(_check(
        'parent-api-spelling-transition',
        spelling_ok,
        'fork-only misspelling is replaced by the canonical PCL setter',
    ))

    collision = contract['collision']
    collision_ok = (
        collision['existing_package'] == after
        and collision['fork_package'] == before
        and collision['safe_resolution'] == 'UPSTREAM_CONVERGENCE'
        and len(collision['conflicting_install_surfaces']) >= 2
    )
    checks.append(_check(
        'collision-resolution',
        collision_ok,
        'canonical provider replaces the colliding fork dependency',
    ))
    authority = contract['authority']
    authority_ok = (
        authority['github_writes_authorized'] is False
        and authority['remote_mutations_performed'] is False
    )
    checks.append(_check(
        'authority-boundary',
        authority_ok,
        'contract authorizes no GitHub write or remote mutation',
    ))

    checkout_report: dict[str, Any] = {
        'inspected': upstream_checkout is not None,
        'commit': None,
        'clean': None,
        'patch_applies': None,
    }
    if upstream_checkout is None:
        checks.extend([
            _not_checked(
                'upstream-checkout-commit',
                'pass --upstream-checkout to verify the exact base'),
            _not_checked(
                'upstream-checkout-clean',
                'pass --upstream-checkout to verify cleanliness'),
            _not_checked(
                'upstream-patch-applies',
                'pass --upstream-checkout to run read-only apply check'),
        ])
    else:
        checkout = upstream_checkout.resolve()
        try:
            commit = _git_text(checkout, 'rev-parse', 'HEAD')
            dirty = _git_text(
                checkout, 'status', '--porcelain', '--untracked-files=all')
            clean = not dirty
            applies, detail = _apply_check(checkout, upstream_patch)
        except ConvergenceError as exc:
            commit = ''
            clean = False
            applies = False
            detail = str(exc)
        checkout_report.update({
            'commit': commit or None,
            'clean': clean,
            'patch_applies': applies,
        })
        expected_commit = contract['upstream']['base_commit']
        checks.extend([
            _check(
                'upstream-checkout-commit',
                commit == expected_commit,
                f'expected {expected_commit}; found {commit or "unavailable"}',
            ),
            _check(
                'upstream-checkout-clean',
                clean,
                'checkout is clean' if clean else 'checkout is not clean',
            ),
            _check('upstream-patch-applies', applies, detail),
        ])

    candidate_report: dict[str, Any] = {
        'inspected': candidate_checkout is not None,
        'commit': None,
        'parent_commit': None,
        'clean': None,
        'subject': None,
        'patch_matches': None,
    }
    if candidate_checkout is None:
        checks.extend([
            _not_checked(
                'candidate-checkout-commit',
                'pass --candidate-checkout to verify the exact local commit'),
            _not_checked(
                'candidate-checkout-parent',
                'pass --candidate-checkout to verify the exact parent'),
            _not_checked(
                'candidate-checkout-clean',
                'pass --candidate-checkout to verify cleanliness'),
            _not_checked(
                'candidate-checkout-subject',
                'pass --candidate-checkout to verify the commit subject'),
            _not_checked(
                'candidate-checkout-patch',
                'pass --candidate-checkout to bind the commit to the patch'),
        ])
    else:
        candidate = candidate_checkout.resolve()
        publication = contract['publication']
        expected_commit = publication['candidate_commit']
        expected_parent = contract['upstream']['base_commit']
        try:
            commit = _git_text(candidate, 'rev-parse', 'HEAD')
            parents = _git_text(candidate, 'show', '-s', '--format=%P', 'HEAD')
            parent_commit = parents if re.fullmatch(r'[0-9a-f]{40}', parents) \
                else None
            dirty = _git_text(
                candidate, 'status', '--porcelain', '--untracked-files=all')
            clean = not dirty
            subject = _git_text(candidate, 'show', '-s', '--format=%s', 'HEAD')
            diff_hash = _git_diff_sha256(candidate, expected_parent, commit)
            patch_matches = (
                diff_hash == contract['upstream']['patch']['sha256'])
        except ConvergenceError as exc:
            commit = ''
            parent_commit = None
            clean = False
            subject = ''
            patch_matches = False
            diff_hash = str(exc)
        candidate_report.update({
            'commit': commit or None,
            'parent_commit': parent_commit,
            'clean': clean,
            'subject': subject or None,
            'patch_matches': patch_matches,
        })
        checks.extend([
            _check(
                'candidate-checkout-commit',
                commit == expected_commit,
                f'expected {expected_commit}; found {commit or "unavailable"}',
            ),
            _check(
                'candidate-checkout-parent',
                parent_commit == expected_parent,
                f'expected {expected_parent}; '
                f'found {parent_commit or "unavailable"}',
            ),
            _check(
                'candidate-checkout-clean',
                clean,
                'candidate checkout is clean' if clean else
                'candidate checkout is not clean',
            ),
            _check(
                'candidate-checkout-subject',
                subject == publication['candidate_subject'],
                f'expected {publication["candidate_subject"]!r}; '
                f'found {subject or "unavailable"!r}',
            ),
            _check(
                'candidate-checkout-patch',
                patch_matches,
                'candidate diff matches the checked-in patch SHA-256'
                if patch_matches else
                f'candidate diff hash mismatch: {diff_hash}',
            ),
        ])

    publication = contract['publication']
    publication_report: dict[str, Any] = {
        'inspected': False,
        'upstream_repository': publication['upstream_repository'],
        'fork_repository': publication['fork_repository'],
        'proposed_branch': publication['proposed_branch'],
        'errors': [],
        'upstream_head_sha': None,
        'fork_is_expected': None,
        'proposed_branch_present': None,
        'open_pr_count': None,
        'duplicate_prs': [],
    }
    if not online:
        checks.extend([
            _not_checked(
                'publication-remote-inspection',
                'pass --online to inspect current GitHub state'),
            _not_checked(
                'publication-upstream-base',
                'pass --online to verify the current upstream base'),
            _not_checked(
                'publication-fork-identity',
                'pass --online to verify the expected GitHub fork'),
            _not_checked(
                'publication-branch-absent',
                'pass --online to verify the proposed branch is absent'),
            _not_checked(
                'publication-no-duplicate-pr',
                'pass --online to search current open upstream PRs'),
        ])
    else:
        publication_report = dict(
            _inspect_publication_state(contract)
            if publication_state is None else publication_state
        )
        publication_report['inspected'] = True
        errors = publication_report.get('errors')
        errors_ok = isinstance(errors, list) and not errors
        upstream_head = publication_report.get('upstream_head_sha')
        fork_is_expected = publication_report.get('fork_is_expected')
        branch_present = publication_report.get('proposed_branch_present')
        duplicates = publication_report.get('duplicate_prs')
        open_pr_count = publication_report.get('open_pr_count')
        checks.extend([
            _check(
                'publication-remote-inspection',
                errors_ok,
                'GitHub inspection completed without errors' if errors_ok else
                f'GitHub inspection errors: {errors}',
            ),
            _check(
                'publication-upstream-base',
                upstream_head == contract['upstream']['base_commit'],
                f'expected {contract["upstream"]["base_commit"]}; '
                f'found {upstream_head or "unavailable"}',
            ),
            _check(
                'publication-fork-identity',
                fork_is_expected is True,
                'fork parent is canonical koide3/ndt_omp' if
                fork_is_expected is True else 'fork identity is not trusted',
            ),
            _check(
                'publication-branch-absent',
                branch_present is False,
                'proposed branch is absent and safe for a non-force push' if
                branch_present is False else
                'proposed branch is present or could not be inspected',
            ),
            _check(
                'publication-no-duplicate-pr',
                (
                    isinstance(open_pr_count, int)
                    and isinstance(duplicates, list)
                    and not duplicates
                ),
                f'checked {open_pr_count} open PRs; no duplicate found'
                if isinstance(duplicates, list) and not duplicates else
                f'duplicate or untrusted PR state: {duplicates}',
            ),
        ])

    failed = [item for item in checks if item['status'] == 'FAIL']
    if failed:
        status = 'BLOCKED'
        actions = [
            'Repair every failed convergence check before proposing any '
            'upstream or rosdistro mutation.'
        ]
    elif upstream_checkout is None:
        status = 'ARTIFACTS_READY'
        actions = [
            'Run again with --upstream-checkout pointing to a clean exact '
            'koide3/ndt_omp base before upstream review.'
        ]
    elif candidate_checkout is None:
        status = 'READY_FOR_UPSTREAM_REVIEW'
        actions = [
            'Verify the exact clean candidate commit with '
            '--candidate-checkout before publication preflight.'
        ]
    elif not online:
        status = 'READY_FOR_UPSTREAM_REVIEW'
        actions = [
            'Run again with --online immediately before any separately '
            'authorized Draft PR publication.'
        ]
    else:
        status = 'READY_FOR_DRAFT_PR'
        actions = [
            'Use draft_pr_handoff for the exact create-only transition only '
            'after separate authority. This report authorizes no GitHub '
            'write.'
        ]

    draft_pr_handoff = (
        _draft_pr_handoff(contract, publication_report)
        if status == 'READY_FOR_DRAFT_PR' else None
    )

    report = {
        'schema_version': 1,
        'schema_uri': REPORT_SCHEMA_URI,
        'contract_id': contract['contract_id'],
        'mode': (
            'draft-preflight' if online else
            'upstream-checkout' if upstream_checkout is not None else
            'artifacts'
        ),
        'status': status,
        'checks': checks,
        'upstream_checkout': checkout_report,
        'candidate_checkout': candidate_report,
        'publication_preflight': publication_report,
        'draft_pr_handoff': draft_pr_handoff,
        'actions': actions,
        'authority': {
            'github_writes_authorized': False,
            'remote_mutations_performed': False,
        },
    }
    report_schema = _load_json(report_schema_path, 'readiness schema')
    _validate_schema(report, report_schema, 'readiness report')
    return report


def _summary(report: dict[str, Any]) -> str:
    passed = sum(item['status'] == 'PASS' for item in report['checks'])
    failed = sum(item['status'] == 'FAIL' for item in report['checks'])
    not_checked = sum(
        item['status'] == 'NOT_CHECKED' for item in report['checks'])
    lines = [
        f"Canonical NDT convergence: {report['status']}",
        f'Checks: {passed} passed, {failed} failed, '
        f'{not_checked} not checked',
    ]
    for item in report['checks']:
        if item['status'] == 'FAIL':
            lines.append(f"- [{item['id']}] {item['detail']}")
    handoff = report['draft_pr_handoff']
    if handoff is not None:
        lines.extend([
            'Draft PR handoff: '
            f"{handoff['fork_repository']}:{handoff['head_branch']} -> "
            f"{handoff['upstream_repository']}:{handoff['base_branch']}",
            f"Exact candidate: {handoff['candidate_commit']}",
            f"Draft title: {handoff['title']}",
            'External write required: yes; authorized by this report: no',
        ])
    lines.extend(f'- {action}' for action in report['actions'])
    lines.append('Remote mutations performed: no')
    return '\n'.join(lines) + '\n'


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--contract',
        type=Path,
        default=DEFAULT_CONTRACT,
        help='Convergence contract JSON (default: repository v1 contract).',
    )
    parser.add_argument(
        '--upstream-checkout',
        type=Path,
        help=(
            'Clean koide3/ndt_omp checkout at the exact contract base; only '
            'read-only Git inspection and patch apply checking are performed.'
        ),
    )
    parser.add_argument(
        '--candidate-checkout',
        type=Path,
        help=(
            'Clean local checkout at the exact candidate commit; its parent, '
            'subject, and binary diff hash are verified without mutation.'
        ),
    )
    parser.add_argument(
        '--online',
        action='store_true',
        help=(
            'Read current GitHub upstream/fork/branch/open-PR state. An '
            'explicit GITHUB_TOKEN or active gh auth credential is scoped '
            'to exact api.github.com GET requests.'
        ),
    )
    strict = parser.add_mutually_exclusive_group()
    strict.add_argument(
        '--require-ready-for-upstream-review',
        action='store_true',
        help='Exit nonzero unless the exact clean upstream checkout passes.',
    )
    strict.add_argument(
        '--require-ready-for-draft-pr',
        action='store_true',
        help=(
            'Exit nonzero unless both exact checkouts and current GitHub '
            'read-only publication state pass.'
        ),
    )
    parser.add_argument(
        '--json', action='store_true', help='Print the schema-v1 JSON report.')
    parser.add_argument(
        '--output-json',
        type=Path,
        help='Create a new report file; existing paths are never overwritten.',
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    options = _parser().parse_args(argv)
    try:
        report = evaluate(
            contract_path=options.contract,
            upstream_checkout=options.upstream_checkout,
            candidate_checkout=options.candidate_checkout,
            online=options.online,
        )
        rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
        if options.output_json is not None:
            try:
                with options.output_json.open('x', encoding='utf-8') as stream:
                    stream.write(rendered)
            except OSError as exc:
                raise ConvergenceError(
                    f'cannot create report {options.output_json}: {exc}') from exc
    except ConvergenceError as exc:
        print(f'canonical NDT convergence blocked: {exc}', file=sys.stderr)
        return 2

    print(rendered if options.json else _summary(report), end='')
    if report['status'] == 'BLOCKED':
        return 1
    if (
        options.require_ready_for_upstream_review
        and report['status'] not in {
            'READY_FOR_UPSTREAM_REVIEW', 'READY_FOR_DRAFT_PR'}
    ):
        return 1
    if (
        options.require_ready_for_draft_pr
        and report['status'] != 'READY_FOR_DRAFT_PR'
    ):
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
