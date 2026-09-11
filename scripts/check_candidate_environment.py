#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
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

"""Read-only, fail-closed audit of the candidate-images environment."""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any, Callable, Sequence
import urllib.error
import urllib.request

from product_schema import validate_contract

try:
    from github_api_auth import github_api_authorization
except ModuleNotFoundError:  # pragma: no cover - importlib test path
    from scripts.github_api_auth import github_api_authorization


REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
ENVIRONMENT_NAME = 'candidate-images'
DEFAULT_BRANCH = 'develop'
SCHEMA_NAME = 'candidate-environment-readiness-v1.schema.json'
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/candidate-environment-readiness-v1.schema.json'
)
MAX_JSON_BYTES = 1024 * 1024
MAX_ENVIRONMENTS = 100
ALLOWED_RULE_TYPES = {'required_reviewers', 'branch_policy'}
SETTINGS_URL = (
    f'https://github.com/{REPOSITORY}/settings/environments'
)
VERIFY_COMMAND = (
    'python3 scripts/check_candidate_environment.py --json --require-ready'
)


class CandidateEnvironmentError(ValueError):
    """The remote environment state cannot be audited safely."""


Fetcher = Callable[[str], tuple[int, dict[str, Any] | None]]


def _object(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise CandidateEnvironmentError(f'{label} must be an object')
    return value


def _bounded_collection(
    document: dict[str, Any],
    *,
    field: str,
    label: str,
) -> list[Any]:
    values = document.get(field)
    if not isinstance(values, list):
        raise CandidateEnvironmentError(f'{label} are missing')
    total_count = document.get('total_count')
    if total_count != len(values):
        raise CandidateEnvironmentError(f'{label} response is truncated')
    if len(values) > MAX_ENVIRONMENTS:
        raise CandidateEnvironmentError(
            f'{label} exceed the bounded audit limit'
        )
    return values


def validate_candidate_environment(
    environment: dict[str, Any],
    branch_policies_document: dict[str, Any],
) -> dict[str, Any]:
    """Validate the exact environment contract shared with publication."""
    if environment.get('name') != ENVIRONMENT_NAME:
        raise CandidateEnvironmentError(
            'candidate-images environment is not configured'
        )
    rules = environment.get('protection_rules')
    if not isinstance(rules, list):
        raise CandidateEnvironmentError(
            'candidate environment protection rules are missing'
        )
    if not all(isinstance(rule, dict) for rule in rules):
        raise CandidateEnvironmentError(
            'candidate environment has a malformed protection rule'
        )
    rule_types = [rule.get('type') for rule in rules]
    if any(rule_type not in ALLOWED_RULE_TYPES for rule_type in rule_types):
        raise CandidateEnvironmentError(
            'candidate environment has an unknown protection rule'
        )
    reviewer_rules = [
        rule for rule in rules if rule.get('type') == 'required_reviewers'
    ]
    if len(reviewer_rules) != 1:
        raise CandidateEnvironmentError(
            'candidate environment requires exactly one reviewer rule'
        )
    if rule_types.count('branch_policy') != 1:
        raise CandidateEnvironmentError(
            'candidate environment requires exactly one branch-policy rule'
        )
    reviewers = reviewer_rules[0].get('reviewers')
    if not isinstance(reviewers, list) or not reviewers:
        raise CandidateEnvironmentError(
            'candidate environment has no required reviewer'
        )
    if len(reviewers) > 6:
        raise CandidateEnvironmentError(
            'candidate environment reviewer set is unbounded'
        )
    for reviewer in reviewers:
        reviewer_object = _object(
            reviewer, 'candidate environment reviewer'
        )
        if reviewer_object.get('type') not in ('User', 'Team'):
            raise CandidateEnvironmentError(
                'candidate environment reviewer type is unsupported'
            )
        _object(
            reviewer_object.get('reviewer'),
            'candidate environment reviewer identity',
        )
    if reviewer_rules[0].get('prevent_self_review') is not True:
        raise CandidateEnvironmentError(
            'candidate environment must prevent self-review'
        )

    deployment_policy = _object(
        environment.get('deployment_branch_policy'),
        'environment deployment branch policy',
    )
    if deployment_policy.get('protected_branches') is not False:
        raise CandidateEnvironmentError(
            'candidate environment must use a custom develop-only policy'
        )
    if deployment_policy.get('custom_branch_policies') is not True:
        raise CandidateEnvironmentError(
            'candidate environment must enable custom branch policies'
        )
    policies = _bounded_collection(
        branch_policies_document,
        field='branch_policies',
        label='candidate deployment branch policies',
    )
    if len(policies) != 1:
        raise CandidateEnvironmentError(
            'candidate environment must allow exactly one deployment branch'
        )
    policy = _object(policies[0], 'candidate deployment branch policy')
    if policy.get('name') != DEFAULT_BRANCH:
        raise CandidateEnvironmentError(
            'candidate environment must allow develop only'
        )
    if policy.get('type') not in (None, 'branch'):
        raise CandidateEnvironmentError(
            'candidate environment policy must target a branch'
        )
    return {
        'name': ENVIRONMENT_NAME,
        'required_reviewer_count': len(reviewers),
        'prevent_self_review': True,
        'deployment_branch_policy': 'develop_only',
    }


def _github_json(path: str) -> tuple[int, dict[str, Any] | None]:
    url = f'https://api.github.com/{path}'
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'lidarslam-candidate-environment-audit/1',
        'X-GitHub-Api-Version': '2022-11-28',
    }
    headers.update(github_api_authorization(url, method='GET'))
    request = urllib.request.Request(url, headers=headers, method='GET')
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            status = response.status
            payload = response.read(MAX_JSON_BYTES + 1)
    except urllib.error.HTTPError as exc:
        status = exc.code
        payload = exc.read(MAX_JSON_BYTES + 1)
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise CandidateEnvironmentError(
            f'cannot read GitHub environment API: {exc}'
        ) from exc
    if len(payload) > MAX_JSON_BYTES:
        raise CandidateEnvironmentError(
            'GitHub environment API response exceeds the byte limit'
        )
    if not payload:
        return status, None
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CandidateEnvironmentError(
            'GitHub environment API returned invalid JSON'
        ) from exc
    if not isinstance(value, dict):
        raise CandidateEnvironmentError(
            'GitHub environment API JSON root is not an object'
        )
    return status, value


def _report(
    *,
    status: str,
    environment_names: list[str],
    target: dict[str, Any] | None,
    finding_id: str | None,
    detail: str,
) -> dict[str, Any]:
    ready = status == 'READY'
    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'repository': REPOSITORY,
        'environment': ENVIRONMENT_NAME,
        'scope': 'read-only-candidate-environment-preflight',
        'status': status,
        'requirements': {
            'required_reviewer_count_minimum': 1,
            'required_reviewer_count_maximum': 6,
            'prevent_self_review': True,
            'deployment_branch_policy': 'develop_only',
            'allowed_protection_rule_types': sorted(ALLOWED_RULE_TYPES),
        },
        'observed': {
            'environment_count': len(environment_names),
            'environment_names': environment_names,
            'target_present': ENVIRONMENT_NAME in environment_names,
            'target': target,
        },
        'findings': (
            [] if finding_id is None else [{
                'id': finding_id,
                'severity': 'BLOCKER',
                'detail': detail,
            }]
        ),
        'authority': {
            'network_reads_performed': True,
            'github_writes_authorized': False,
            'environment_writes_authorized': False,
            'artifact_publication_authorized': False,
            'remote_mutations_performed': False,
        },
        'operator_handoff': _operator_handoff(status),
        'decision': {
            'state': (
                'READY_FOR_SEPARATE_E2_REVIEW' if ready else 'HOLD'
            ),
            'dispatch_authorized': False,
            'next_action': detail,
        },
    }
    validate_contract(report, SCHEMA_NAME)
    return report


def _operator_handoff(status: str) -> dict[str, Any]:
    """Return one bounded, status-specific handoff without doing the work."""
    if status == 'ABSENT':
        return {
            'kind': 'CREATE_AND_REVIEW_ENVIRONMENT',
            'authority_required': 'repository-settings-admin',
            'external_write_required': True,
            'settings_url': SETTINGS_URL,
            'steps': [
                'Create an environment named candidate-images.',
                'Add 1–6 trusted User or Team required reviewers.',
                'Enable Prevent self-review.',
                (
                    'Select custom deployment branches and allow exactly '
                    'the develop branch.'
                ),
                'Have an independent maintainer review the saved settings.',
            ],
            'verification_command': VERIFY_COMMAND,
            'writes_performed': False,
        }
    if status == 'MISCONFIGURED':
        return {
            'kind': 'REPAIR_AND_REVIEW_ENVIRONMENT',
            'authority_required': 'repository-settings-admin',
            'external_write_required': True,
            'settings_url': SETTINGS_URL,
            'steps': [
                'Open the existing candidate-images environment.',
                (
                    'Keep one required-reviewer rule with 1–6 User or Team '
                    'reviewers.'
                ),
                'Enable Prevent self-review.',
                (
                    'Use custom deployment branches with exactly one develop '
                    'branch policy and no other protection-rule type.'
                ),
                'Have an independent maintainer review the repaired settings.',
            ],
            'verification_command': VERIFY_COMMAND,
            'writes_performed': False,
        }
    if status == 'BLOCKED':
        return {
            'kind': 'RESTORE_READ_ACCESS',
            'authority_required': 'read-access',
            'external_write_required': False,
            'settings_url': None,
            'steps': [
                (
                    'Run gh auth status and confirm repository environment '
                    'read access.'
                ),
                (
                    'If the active gh credential lacks access, provide a '
                    'read-capable GITHUB_TOKEN only to the verification '
                    'command.'
                ),
                (
                    'Retry the audit; do not change environment settings from '
                    'incomplete evidence.'
                ),
            ],
            'verification_command': VERIFY_COMMAND,
            'writes_performed': False,
        }
    if status == 'READY':
        return {
            'kind': 'REVIEW_E2_SEPARATELY',
            'authority_required': 'separate-e2-approval',
            'external_write_required': False,
            'settings_url': None,
            'steps': [
                (
                    'Review the exact mergeable PR head, VERSION, required '
                    'checks, and immutable digest-only scope.'
                ),
                (
                    "Keep environment approval separate from the requester's "
                    'E2 decision.'
                ),
            ],
            'verification_command': VERIFY_COMMAND,
            'writes_performed': False,
        }
    raise CandidateEnvironmentError(
        f'unsupported candidate environment status {status!r}'
    )


def render_human(report: dict[str, Any]) -> str:
    """Render one copy-ready operator handoff without implying authority."""
    handoff = report['operator_handoff']
    lines = [
        f"Candidate environment: {report['status']}",
        f"Why: {report['decision']['next_action']}",
        '',
        'Operator handoff (not executed):',
        f"Authority required: {handoff['authority_required']}",
    ]
    if handoff['settings_url'] is not None:
        lines.append(f"Settings: {handoff['settings_url']}")
    lines.extend(
        f'{index}. {step}'
        for index, step in enumerate(handoff['steps'], start=1)
    )
    lines.extend([
        'Verify (read-only):',
        f"  {handoff['verification_command']}",
        'Environment writes performed: no',
        'GitHub writes authorized: no',
        'E2 dispatch authorized: no',
    ])
    return '\n'.join(lines) + '\n'


def audit_candidate_environment(
    *,
    fetcher: Fetcher = _github_json,
) -> dict[str, Any]:
    """Inspect the complete live environment inventory without mutation."""
    inventory_path = f'repos/{REPOSITORY}/environments?per_page=100'
    try:
        inventory_status, inventory = fetcher(inventory_path)
        if inventory_status != 200 or inventory is None:
            raise CandidateEnvironmentError(
                'GitHub environment inventory is not readable '
                f'(HTTP {inventory_status})'
            )
        environments = _bounded_collection(
            inventory,
            field='environments',
            label='repository environments',
        )
        names: list[str] = []
        for raw_environment in environments:
            item = _object(raw_environment, 'repository environment')
            name = item.get('name')
            if not isinstance(name, str) or not name:
                raise CandidateEnvironmentError(
                    'repository environment has no valid name'
                )
            names.append(name)
        if len(names) != len(set(names)):
            raise CandidateEnvironmentError(
                'repository environment inventory contains duplicate names'
            )
        names.sort()
        if ENVIRONMENT_NAME not in names:
            return _report(
                status='ABSENT',
                environment_names=names,
                target=None,
                finding_id='candidate-environment-absent',
                detail=(
                    'Configure and independently review the protected '
                    'candidate-images environment before any E2 dispatch.'
                ),
            )

        environment_path = (
            f'repos/{REPOSITORY}/environments/{ENVIRONMENT_NAME}'
        )
        policy_path = (
            f'{environment_path}/deployment-branch-policies?per_page=100'
        )
        environment_status, environment = fetcher(environment_path)
        policy_status, policies = fetcher(policy_path)
        if environment_status != 200 or environment is None:
            raise CandidateEnvironmentError(
                'listed candidate environment is not readable '
                f'(HTTP {environment_status})'
            )
        if policy_status != 200 or policies is None:
            raise CandidateEnvironmentError(
                'candidate deployment policies are not readable '
                f'(HTTP {policy_status})'
            )
        try:
            target = validate_candidate_environment(environment, policies)
        except CandidateEnvironmentError as exc:
            return _report(
                status='MISCONFIGURED',
                environment_names=names,
                target=None,
                finding_id='candidate-environment-misconfigured',
                detail=str(exc),
            )
        return _report(
            status='READY',
            environment_names=names,
            target=target,
            finding_id=None,
            detail=(
                'Review the exact PR head and E2 scope separately; this '
                'read-only report does not authorize a dispatch.'
            ),
        )
    except CandidateEnvironmentError as exc:
        return _report(
            status='BLOCKED',
            environment_names=[],
            target=None,
            finding_id='candidate-environment-audit-blocked',
            detail=str(exc),
        )


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json', action='store_true')
    parser.add_argument(
        '--require-ready',
        action='store_true',
        help='Exit 1 unless the exact protected environment is ready.',
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Run the network-read-only audit and render one bounded result."""
    args = _parse_args(argv)
    try:
        report = audit_candidate_environment()
    except (CandidateEnvironmentError, OSError, ValueError) as exc:
        print(f'candidate environment audit error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(render_human(report), end='')
    if args.require_ready and report['status'] != 'READY':
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
