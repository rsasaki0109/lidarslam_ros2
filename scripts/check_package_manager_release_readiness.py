#!/usr/bin/env python3
"""Audit the public Humble/Jazzy main-channel package-manager release gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
from typing import Any
import urllib.error
import urllib.parse
import urllib.request

import jsonschema

try:
    from github_api_auth import github_api_authorization
except ModuleNotFoundError:  # pragma: no cover - importlib test path
    from scripts.github_api_auth import github_api_authorization


REPO_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
WORKFLOW_PATH = '.github/workflows/package-manager-install-upgrade.yml'
WORKFLOW_NAME = 'package-manager-install-upgrade.yml'
DISTROS = ('humble', 'jazzy')
SCHEMA_PATH = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'package-manager-release-readiness-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/package-manager-release-readiness-v1.schema.json'
)
SEMVER = re.compile(r'^[0-9]+\.[0-9]+\.[0-9]+$')
SHA = re.compile(r'^[a-f0-9]{40}$')
MAX_RESPONSE_BYTES = 2 * 1024 * 1024


class PackageManagerReleaseError(ValueError):
    """The public package-manager release state could not be trusted."""


def expected_run_name(version: str) -> str:
    """Return the immutable identity encoded by the workflow run name."""
    if SEMVER.fullmatch(version) is None:
        raise PackageManagerReleaseError(
            f'expected MAJOR.MINOR.PATCH version, found {version!r}')
    return (
        f'package-manager / v{version} / {version} / main / clean-install'
    )


def _resolve_source_ref(version: str) -> dict[str, Any]:
    """Resolve one lightweight or annotated release tag to its commit."""
    source_ref = f'v{version}'
    encoded_ref = urllib.parse.quote(f'tags/{source_ref}', safe='/')
    ref_url = (
        f'https://api.github.com/repos/{REPOSITORY}/git/ref/{encoded_ref}'
    )
    payload = _request_json(ref_url, allow_not_found=True)
    if payload is None:
        return {
            'ref': source_ref,
            'resolved': False,
            'commit_sha': None,
        }
    target = payload.get('object')
    for _depth in range(5):
        if not isinstance(target, dict):
            raise PackageManagerReleaseError(
                f'GitHub ref response for {source_ref} has no object')
        object_type = target.get('type')
        object_sha = target.get('sha')
        if object_type == 'commit':
            if (
                not isinstance(object_sha, str)
                or SHA.fullmatch(object_sha) is None
            ):
                raise PackageManagerReleaseError(
                    f'GitHub ref response for {source_ref} has no '
                    'valid commit SHA')
            return {
                'ref': source_ref,
                'resolved': True,
                'commit_sha': object_sha,
            }
        if object_type != 'tag':
            raise PackageManagerReleaseError(
                f'GitHub ref {source_ref} points to unsupported object '
                f'type {object_type!r}')
        tag_url = target.get('url')
        trusted_prefix = (
            f'https://api.github.com/repos/{REPOSITORY}/git/tags/'
        )
        if (
            not isinstance(tag_url, str)
            or not isinstance(object_sha, str)
            or SHA.fullmatch(object_sha) is None
            or tag_url != f'{trusted_prefix}{object_sha}'
        ):
            raise PackageManagerReleaseError(
                f'GitHub annotated tag {source_ref} has no trusted object URL')
        tag_payload = _request_json(tag_url)
        if tag_payload is None:  # pragma: no cover - not allowed here
            raise PackageManagerReleaseError(
                f'GitHub annotated tag {source_ref} was unexpectedly absent')
        target = tag_payload.get('object')
    raise PackageManagerReleaseError(
        f'GitHub annotated tag {source_ref} exceeds the dereference limit')


def _request_json(
    url: str,
    *,
    allow_not_found: bool = False,
) -> dict[str, Any] | None:
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'lidarslam-package-manager-release-audit/1',
        'X-GitHub-Api-Version': '2022-11-28',
    }
    headers.update(github_api_authorization(url, method='GET'))
    request = urllib.request.Request(
        url,
        headers=headers,
    )
    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            length_text = response.headers.get('Content-Length')
            if length_text is not None and int(length_text) > MAX_RESPONSE_BYTES:
                raise PackageManagerReleaseError(
                    f'remote response is too large: {length_text} bytes')
            payload = response.read(MAX_RESPONSE_BYTES + 1)
            if len(payload) > MAX_RESPONSE_BYTES:
                raise PackageManagerReleaseError(
                    'remote response exceeds the download limit')
    except urllib.error.HTTPError as exc:
        if allow_not_found and exc.code == 404:
            return None
        raise PackageManagerReleaseError(
            f'GitHub API request failed for {url}: {exc}') from exc
    except (
        OSError,
        ValueError,
        urllib.error.URLError,
    ) as exc:
        raise PackageManagerReleaseError(
            f'GitHub API request failed for {url}: {exc}') from exc
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PackageManagerReleaseError(
            f'GitHub API returned invalid JSON for {url}: {exc}') from exc
    if not isinstance(value, dict):
        raise PackageManagerReleaseError(
            f'GitHub API root is not an object for {url}')
    return value


def inspect_remote(version: str) -> dict[str, Any]:
    """Inspect the source ref and workflow dispatches without GitHub writes."""
    run_name = expected_run_name(version)
    source_ref = f'v{version}'
    source_state: dict[str, Any] = {
        'ref': source_ref,
        'resolved': False,
        'commit_sha': None,
    }
    query = urllib.parse.urlencode({
        'event': 'workflow_dispatch',
        'per_page': 100,
    })
    runs_url = (
        f'https://api.github.com/repos/{REPOSITORY}/actions/workflows/'
        f'{WORKFLOW_NAME}/runs?{query}'
    )
    try:
        source_state = _resolve_source_ref(version)
        payload = _request_json(runs_url)
        if payload is None:  # pragma: no cover - not allowed for this request
            raise PackageManagerReleaseError(
                'GitHub workflow-runs response was unexpectedly absent')
        runs = payload.get('workflow_runs')
        if not isinstance(runs, list):
            raise PackageManagerReleaseError(
                'GitHub workflow-runs response has no workflow_runs array')
        candidates = [
            run for run in runs
            if (
                isinstance(run, dict)
                and run.get('display_title') == run_name
            )
        ]
        inspected: list[dict[str, Any]] = []
        for run in candidates:
            jobs_url = run.get('jobs_url')
            if not isinstance(jobs_url, str):
                raise PackageManagerReleaseError(
                    'matching workflow run has no jobs_url')
            jobs_payload = _request_json(jobs_url)
            if jobs_payload is None:  # pragma: no cover - not allowed here
                raise PackageManagerReleaseError(
                    'GitHub jobs response was unexpectedly absent')
            jobs = jobs_payload.get('jobs')
            if not isinstance(jobs, list):
                raise PackageManagerReleaseError(
                    'GitHub jobs response has no jobs array')
            inspected.append({
                'id': run.get('id'),
                'html_url': run.get('html_url'),
                'display_title': run.get('display_title'),
                'event': run.get('event'),
                'status': run.get('status'),
                'conclusion': run.get('conclusion'),
                'workflow_path': run.get('path'),
                'head_sha': run.get('head_sha'),
                'jobs': [
                    {
                        'name': job.get('name'),
                        'status': job.get('status'),
                        'conclusion': job.get('conclusion'),
                    }
                    for job in jobs
                    if isinstance(job, dict)
                ],
            })
        return {
            'inspected': True,
            'errors': [],
            'source_ref': source_state,
            'runs': inspected,
        }
    except PackageManagerReleaseError as exc:
        return {
            'inspected': False,
            'errors': [str(exc)],
            'source_ref': source_state,
            'runs': [],
        }


def _check(check_id: str, passed: bool, detail: str) -> dict[str, str]:
    return {
        'id': check_id,
        'status': 'PASS' if passed else 'FAIL',
        'detail': detail,
    }


def evaluate_readiness(
    *,
    version: str,
    snapshot: dict[str, Any],
) -> dict[str, Any]:
    """Evaluate an injected or live workflow snapshot fail-closed."""
    run_name = expected_run_name(version)
    errors = snapshot.get('errors', [])
    runs = snapshot.get('runs', [])
    if not isinstance(errors, list) or not all(
        isinstance(error, str) and error for error in errors
    ):
        raise PackageManagerReleaseError(
            'snapshot errors must be an array of non-empty strings')
    if not isinstance(runs, list) or not all(
        isinstance(run, dict) for run in runs
    ):
        raise PackageManagerReleaseError(
            'snapshot runs must be an array of objects')

    expected_source_ref = f'v{version}'
    source_ref = snapshot.get('source_ref')
    if not isinstance(source_ref, dict):
        raise PackageManagerReleaseError(
            'snapshot source_ref must be an object')
    if source_ref.get('ref') != expected_source_ref:
        raise PackageManagerReleaseError(
            f'expected source ref {expected_source_ref!r}, found '
            f"{source_ref.get('ref')!r}")
    source_resolved = source_ref.get('resolved')
    source_commit_sha = source_ref.get('commit_sha')
    if not isinstance(source_resolved, bool):
        raise PackageManagerReleaseError(
            'snapshot source_ref.resolved must be a boolean')
    if source_resolved:
        if (
            not isinstance(source_commit_sha, str)
            or SHA.fullmatch(source_commit_sha) is None
        ):
            raise PackageManagerReleaseError(
                'resolved source_ref must contain a 40-character commit SHA')
    elif source_commit_sha is not None:
        raise PackageManagerReleaseError(
            'unresolved source_ref must have a null commit_sha')

    assessments: list[dict[str, Any]] = []
    for run in runs:
        jobs = run.get('jobs', [])
        if not isinstance(jobs, list):
            continue
        job_state = {
            job.get('name'): (
                job.get('status'),
                job.get('conclusion'),
            )
            for job in jobs
            if isinstance(job, dict)
        }
        candidate_checks = [
            _check(
                'immutable-run-name',
                run.get('display_title') == run_name,
                f"expected {run_name!r}; found {run.get('display_title')!r}",
            ),
            _check(
                'workflow-dispatch',
                run.get('event') == 'workflow_dispatch',
                f"event={run.get('event')!r}",
            ),
            _check(
                'trusted-workflow-path',
                run.get('workflow_path') == WORKFLOW_PATH,
                f"workflow_path={run.get('workflow_path')!r}",
            ),
            _check(
                'completed-successfully',
                (
                    run.get('status') == 'completed'
                    and run.get('conclusion') == 'success'
                ),
                (
                    f"status={run.get('status')!r}, "
                    f"conclusion={run.get('conclusion')!r}"
                ),
            ),
            _check(
                'workflow-head-identity',
                (
                    source_resolved
                    and run.get('head_sha') == source_commit_sha
                ),
                (
                    f'expected source commit {source_commit_sha!r}; '
                    f"found head_sha={run.get('head_sha')!r}"
                ),
            ),
        ]
        for distro in DISTROS:
            name = f'{distro} clean-install'
            state = job_state.get(name)
            candidate_checks.append(_check(
                f'{distro}-main-clean-install',
                state == ('completed', 'success'),
                f'job {name!r}: {state!r}',
            ))
        identity_trusted = all(
            candidate_checks[index]['status'] == 'PASS'
            for index in (0, 1, 2, 4)
        )
        assessments.append({
            'run': run,
            'checks': candidate_checks,
            'identity_trusted': identity_trusted,
            'ready': all(
                check['status'] == 'PASS'
                for check in candidate_checks
            ),
        })

    selected_assessment = next(
        (item for item in assessments if item['ready']),
        None,
    )
    selected = (
        selected_assessment['run']
        if selected_assessment is not None
        else None
    )
    checks: list[dict[str, str]] = (
        selected_assessment['checks']
        if selected_assessment is not None
        else []
    )

    if errors:
        status = 'BLOCKED'
        actions = [
            'Restore trusted read-only access to the public GitHub Actions API.'
        ]
    elif not source_resolved:
        status = 'SOURCE_REF_MISSING'
        actions = [
            (
                f'Do not dispatch the package-manager workflow: '
                f'{expected_source_ref} does not resolve to an immutable '
                'public commit. Publish it only through the reviewed release '
                'gate, then rerun this audit.'
            )
        ]
    elif selected is not None:
        status = 'READY'
        actions = []
    else:
        trusted = [
            item for item in assessments
            if item['identity_trusted']
        ]
        running = next(
            (
                item for item in trusted
                if item['run'].get('status') != 'completed'
            ),
            None,
        )
        failed = next(iter(trusted), None)
        if running is not None:
            checks = running['checks']
            run_url = running['run'].get('html_url')
            status = 'RUNNING'
            actions = [
                f'Wait for the exact package-manager run to finish: {run_url}'
            ]
        elif failed is not None:
            checks = failed['checks']
            run_url = failed['run'].get('html_url')
            status = 'FAILED'
            actions = [
                (
                    f'Inspect the failed exact package-manager run at '
                    f'{run_url}; fix the recorded failure before rerunning it.'
                )
            ]
        elif assessments:
            checks = assessments[0]['checks']
            status = 'BLOCKED'
            actions = [
                (
                    'Do not reuse the matching workflow history: its event, '
                    'workflow path, completion state, or source commit does '
                    'not match the immutable candidate.'
                )
            ]
        else:
            status = 'NOT_RUN'
            dispatch = (
                f'gh workflow run {WORKFLOW_NAME} --repo {REPOSITORY} '
                f'--ref {expected_source_ref} '
                f'-f source_ref={expected_source_ref} '
                f'-f target_version={version} '
                '-f target_channel=main -f mode=clean-install'
            )
            actions = [
                (
                    'First require main-channel dependency readiness with '
                    '`python3 scripts/check_ros_apt_dependency_readiness.py '
                    '--require main` and confirm the exact product packages '
                    'are present in both supported distributions.'
                ),
                f'Dispatch the exact immutable candidate with `{dispatch}`.',
            ]

    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'status': status,
        'candidate': {
            'product_version': version,
            'source_ref': f'v{version}',
            'apt_channel': 'main',
            'mode': 'clean-install',
            'run_name': run_name,
        },
        'source_ref': source_ref,
        'remote': {
            'inspected': snapshot.get('inspected') is True,
            'errors': errors,
            'matching_runs': len(runs),
        },
        'selected_run': selected,
        'checks': checks,
        'actions': actions,
    }
    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.Draft7Validator(schema).validate(report)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version', required=True)
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--require-ready', action='store_true')
    args = parser.parse_args(argv)
    try:
        report = evaluate_readiness(
            version=args.version,
            snapshot=inspect_remote(args.version),
        )
    except (
        OSError,
        PackageManagerReleaseError,
        json.JSONDecodeError,
        jsonschema.SchemaError,
        jsonschema.ValidationError,
    ) as exc:
        print(
            f'package-manager release readiness error: {exc}',
            file=sys.stderr,
        )
        return 2
    rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(rendered, encoding='utf-8')
    if args.json:
        print(rendered, end='')
    else:
        print(
            f"Package-manager release gate: {report['status']}\n"
            f"Expected run: {report['candidate']['run_name']}"
        )
        for action in report['actions']:
            print(f'  - {action}')
    if args.require_ready and report['status'] != 'READY':
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
