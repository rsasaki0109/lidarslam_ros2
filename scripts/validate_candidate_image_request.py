#!/usr/bin/env python3
"""Authorize one exact, digest-only candidate-image publication request.

This validator is intended to run from the trusted default-branch checkout in
the candidate-image workflow.  It consumes already-downloaded GitHub API
documents, performs no network request, and emits one auditable authorization
record only when every identity and exact-head CI boundary agrees.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
from typing import Any, Sequence

from check_candidate_environment import validate_candidate_environment
from product_schema import load_json_object, validate_contract


REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
DEFAULT_BRANCH = 'develop'
EVENT_NAME = 'repository_dispatch'
EVENT_ACTION = 'e2-publish-candidate-image'
APPROVAL = 'E2_IMMUTABLE_DIGEST_ONLY'
SCHEMA_NAME = 'candidate-image-request-v1.schema.json'
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/candidate-image-request-v1.schema.json'
)
COMMIT_RE = re.compile(r'^[0-9a-f]{40}$')
VERSION_RE = re.compile(r'^[0-9]+\.[0-9]+\.[0-9]+$')
ACTOR_RE = re.compile(r'^[A-Za-z0-9](?:[A-Za-z0-9-]{0,38})$')
MAINTAINER_ROLES = {'admin', 'maintain'}
REQUIRED_SUCCESS_CHECKS = {
    'build (humble)',
    'build (jazzy)',
    'docs and release metadata',
    'humble default workflow',
    'humble v0.6.0 to candidate',
    'jazzy default workflow',
    'jazzy v0.6.0 to candidate',
    'release readiness',
    'release readiness threshold guard',
}
ALLOWED_SKIPPED_CHECKS = {
    'build and push (${{ matrix.ros_distro }})',
    'authorize immutable candidate request',
    'publish immutable digest (${{ matrix.ros_distro }})',
    'verify immutable candidate pair',
}


def _object(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f'{label} must be an object')
    return value


def _string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f'{label} must be a non-empty string')
    return value


def _validate_pull_request(
    pull_request: dict[str, Any],
    *,
    repository: str,
    source_pr: int,
    source_commit: str,
) -> None:
    if pull_request.get('number') != source_pr:
        raise ValueError('pull request number does not match the request')
    if pull_request.get('state') != 'open':
        raise ValueError('candidate pull request must remain open')
    if pull_request.get('mergeable') is not True:
        raise ValueError('candidate pull request must be mergeable')

    base = _object(pull_request.get('base'), 'pull request base')
    head = _object(pull_request.get('head'), 'pull request head')
    base_repository = _object(base.get('repo'), 'pull request base repo')
    head_repository = _object(head.get('repo'), 'pull request head repo')
    if base.get('ref') != DEFAULT_BRANCH:
        raise ValueError(
            f'candidate pull request must target {DEFAULT_BRANCH}'
        )
    if base_repository.get('full_name') != repository:
        raise ValueError('pull request base repository does not match')
    if head_repository.get('full_name') != repository:
        raise ValueError(
            'candidate must be a same-repository pull request'
        )
    if head.get('sha') != source_commit:
        raise ValueError('candidate commit is not the exact pull request head')


def _validate_check_runs(
    check_runs_document: dict[str, Any],
) -> tuple[list[str], list[str]]:
    runs = check_runs_document.get('check_runs')
    if not isinstance(runs, list) or not runs:
        raise ValueError('exact candidate commit has no check runs')
    declared_count = check_runs_document.get('total_count')
    if declared_count != len(runs):
        raise ValueError(
            'check-run response is incomplete or truncated; request '
            'filter=latest&per_page=100'
        )
    if len(runs) > 100:
        raise ValueError('check-run response exceeds the bounded audit limit')

    successful: list[str] = []
    skipped: list[str] = []
    seen: set[str] = set()
    for index, raw_run in enumerate(runs):
        run = _object(raw_run, f'check run {index}')
        name = _string(run.get('name'), f'check run {index} name')
        if name in seen:
            raise ValueError(f'duplicate latest check-run name: {name}')
        seen.add(name)
        if run.get('status') != 'completed':
            raise ValueError(f'check run is not completed: {name}')
        conclusion = run.get('conclusion')
        if conclusion == 'success':
            successful.append(name)
        elif conclusion == 'skipped':
            if name not in ALLOWED_SKIPPED_CHECKS:
                raise ValueError(f'unexpected skipped check run: {name}')
            skipped.append(name)
        else:
            raise ValueError(
                f'check run {name} concluded {conclusion!r}, not success'
            )

    missing = sorted(REQUIRED_SUCCESS_CHECKS.difference(successful))
    if missing:
        raise ValueError(
            'exact candidate commit is missing required successful checks: '
            + ', '.join(missing)
        )
    return sorted(successful), sorted(skipped)


def validate_candidate_image_request(
    *,
    event_name: str,
    event_action: str,
    repository: str,
    workflow_branch_ref: str,
    workflow_gate_commit: str,
    default_branch: str,
    source_pr: int,
    source_commit: str,
    product_version: str,
    candidate_version: str,
    requested_by: str,
    actor_role: str,
    approval: str,
    pull_request: dict[str, Any],
    check_runs_document: dict[str, Any],
    environment: dict[str, Any],
    branch_policies_document: dict[str, Any],
) -> dict[str, Any]:
    """Validate one request and return its schema-backed authorization."""
    if event_name != EVENT_NAME or event_action != EVENT_ACTION:
        raise ValueError('candidate publication requires its dedicated event')
    if repository != REPOSITORY:
        raise ValueError('candidate publication repository does not match')
    if default_branch != DEFAULT_BRANCH:
        raise ValueError('repository default branch does not match the gate')
    if workflow_branch_ref != f'refs/heads/{DEFAULT_BRANCH}':
        raise ValueError(
            'candidate workflow must execute from the default branch'
        )
    if COMMIT_RE.fullmatch(workflow_gate_commit) is None:
        raise ValueError(
            'workflow_gate_commit must be one lowercase 40-character SHA'
        )
    if approval != APPROVAL:
        raise ValueError('explicit immutable-digest E2 approval is missing')
    if actor_role not in MAINTAINER_ROLES:
        raise ValueError('candidate publication requires maintain role')
    if ACTOR_RE.fullmatch(requested_by) is None:
        raise ValueError('requested_by is not a valid GitHub login')
    if not isinstance(source_pr, int) or isinstance(source_pr, bool):
        raise ValueError('source_pr must be an integer')
    if source_pr < 1:
        raise ValueError('source_pr must be positive')
    if COMMIT_RE.fullmatch(source_commit) is None:
        raise ValueError(
            'source_commit must be one lowercase 40-character SHA'
        )
    if VERSION_RE.fullmatch(product_version) is None:
        raise ValueError('product_version must be x.y.z')
    if candidate_version != product_version:
        raise ValueError('candidate VERSION does not match product_version')

    _validate_pull_request(
        pull_request,
        repository=repository,
        source_pr=source_pr,
        source_commit=source_commit,
    )
    successful, skipped = _validate_check_runs(check_runs_document)
    environment_report = validate_candidate_environment(
        environment,
        branch_policies_document,
    )
    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'status': 'AUTHORIZED',
        'publication_mode': 'digest_only',
        'repository': repository,
        'event_name': event_name,
        'event_action': event_action,
        'default_branch': default_branch,
        'workflow_branch_ref': workflow_branch_ref,
        'workflow_gate_commit': workflow_gate_commit,
        'source_pr': source_pr,
        'source_commit': source_commit,
        'product_version': product_version,
        'requested_by': requested_by,
        'actor_role': actor_role,
        'required_success_checks': sorted(REQUIRED_SUCCESS_CHECKS),
        'observed_successful_checks': successful,
        'observed_skipped_checks': skipped,
        'environment': environment_report,
        'authority': {
            'package_write_authorized_for_digest_job': True,
            'tag_creation_authorized': False,
            'moving_tag_mutation_authorized': False,
            'release_mutation_authorized': False,
        },
    }
    validate_contract(report, SCHEMA_NAME)
    return report


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--event-name', required=True)
    parser.add_argument('--event-action', required=True)
    parser.add_argument('--repository', required=True)
    parser.add_argument('--workflow-branch-ref', required=True)
    parser.add_argument('--workflow-gate-commit', required=True)
    parser.add_argument('--default-branch', required=True)
    parser.add_argument('--source-pr', required=True, type=int)
    parser.add_argument('--source-commit', required=True)
    parser.add_argument('--product-version', required=True)
    parser.add_argument('--candidate-version-file', required=True, type=Path)
    parser.add_argument('--requested-by', required=True)
    parser.add_argument('--actor-role', required=True)
    parser.add_argument('--approval', required=True)
    parser.add_argument('--pull-request-json', required=True, type=Path)
    parser.add_argument('--check-runs-json', required=True, type=Path)
    parser.add_argument('--environment-json', required=True, type=Path)
    parser.add_argument('--branch-policies-json', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Validate downloaded evidence and write one exclusive report."""
    args = _parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(
                f'refusing to overwrite candidate request: {args.output}'
            )
        candidate_version = args.candidate_version_file.read_text(
            encoding='utf-8'
        ).strip()
        report = validate_candidate_image_request(
            event_name=args.event_name,
            event_action=args.event_action,
            repository=args.repository,
            workflow_branch_ref=args.workflow_branch_ref,
            workflow_gate_commit=args.workflow_gate_commit,
            default_branch=args.default_branch,
            source_pr=args.source_pr,
            source_commit=args.source_commit,
            product_version=args.product_version,
            candidate_version=candidate_version,
            requested_by=args.requested_by,
            actor_role=args.actor_role,
            approval=args.approval,
            pull_request=load_json_object(
                args.pull_request_json,
                'candidate pull request',
            ),
            check_runs_document=load_json_object(
                args.check_runs_json,
                'candidate check runs',
            ),
            environment=load_json_object(
                args.environment_json,
                'candidate environment',
            ),
            branch_policies_document=load_json_object(
                args.branch_policies_json,
                'candidate deployment branch policies',
            ),
        )
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write('\n')
    except (OSError, ValueError) as exc:
        print(f'candidate image request error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
