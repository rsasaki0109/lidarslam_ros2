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

"""Tests for the read-only candidate environment preflight."""

from __future__ import annotations

import copy
import importlib.util
import json
import pathlib
import sys

import jsonschema

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
SCRIPT = SCRIPTS / 'check_candidate_environment.py'
SCHEMA = (
    ROOT / 'docs' / 'schemas'
    / 'candidate-environment-readiness-v1.schema.json'
)
SPEC = importlib.util.spec_from_file_location(
    'check_candidate_environment_test', SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
sys.path.insert(0, str(SCRIPTS))
try:
    CHECKER = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(CHECKER)
finally:
    sys.path.remove(str(SCRIPTS))


INVENTORY_PATH = (
    'repos/rsasaki0109/lidar_slam_ros2/environments?per_page=100'
)
ENVIRONMENT_PATH = (
    'repos/rsasaki0109/lidar_slam_ros2/environments/candidate-images'
)
POLICIES_PATH = (
    f'{ENVIRONMENT_PATH}/deployment-branch-policies?per_page=100'
)


def _inventory(*names: str) -> dict:
    return {
        'total_count': len(names),
        'environments': [{'name': name} for name in names],
    }


def _environment() -> dict:
    return {
        'name': 'candidate-images',
        'protection_rules': [
            {
                'id': 1,
                'type': 'required_reviewers',
                'prevent_self_review': True,
                'reviewers': [{
                    'type': 'User',
                    'reviewer': {'login': 'release-reviewer'},
                }],
            },
            {'id': 2, 'type': 'branch_policy'},
        ],
        'deployment_branch_policy': {
            'protected_branches': False,
            'custom_branch_policies': True,
        },
    }


def _policies(name: str = 'develop') -> dict:
    return {
        'total_count': 1,
        'branch_policies': [{'id': 1, 'name': name, 'type': 'branch'}],
    }


class FakeFetcher:
    """Return exact synthetic API documents while recording request order."""

    def __init__(self, responses: dict[str, tuple[int, dict | None]]):
        """Store endpoint responses for one deterministic audit."""
        self.responses = responses
        self.calls: list[str] = []

    def __call__(self, path: str):
        """Return one configured response and record the endpoint path."""
        self.calls.append(path)
        return self.responses[path]


def _ready_fetcher(
    *,
    environment: dict | None = None,
    policies: dict | None = None,
) -> FakeFetcher:
    return FakeFetcher({
        INVENTORY_PATH: (200, _inventory('candidate-images')),
        ENVIRONMENT_PATH: (200, environment or _environment()),
        POLICIES_PATH: (200, policies or _policies()),
    })


def test_ready_environment_is_exact_and_never_authorizes_dispatch():
    """A ready preflight remains read-only and cannot authorize dispatch."""
    fetcher = _ready_fetcher()

    report = CHECKER.audit_candidate_environment(fetcher=fetcher)

    assert report['status'] == 'READY'
    assert fetcher.calls == [
        INVENTORY_PATH,
        ENVIRONMENT_PATH,
        POLICIES_PATH,
    ]
    assert report['observed']['target'] == {
        'name': 'candidate-images',
        'required_reviewer_count': 1,
        'prevent_self_review': True,
        'deployment_branch_policy': 'develop_only',
    }
    assert report['decision'] == {
        'state': 'READY_FOR_SEPARATE_E2_REVIEW',
        'dispatch_authorized': False,
        'next_action': (
            'Review the exact PR head and E2 scope separately; this '
            'read-only report does not authorize a dispatch.'
        ),
    }
    assert report['authority']['github_writes_authorized'] is False
    assert report['authority']['remote_mutations_performed'] is False
    assert report['operator_handoff']['kind'] == 'REVIEW_E2_SEPARATELY'
    assert report['operator_handoff']['external_write_required'] is False
    assert report['operator_handoff']['settings_url'] is None
    assert report['operator_handoff']['writes_performed'] is False


def test_complete_inventory_proves_absence_without_follow_up_requests():
    """A complete inventory can prove absence without interpreting a 404."""
    fetcher = FakeFetcher({
        INVENTORY_PATH: (200, _inventory('production')),
    })

    report = CHECKER.audit_candidate_environment(fetcher=fetcher)

    assert report['status'] == 'ABSENT'
    assert fetcher.calls == [INVENTORY_PATH]
    assert report['observed']['environment_names'] == ['production']
    assert report['findings'][0]['id'] == 'candidate-environment-absent'
    handoff = report['operator_handoff']
    assert handoff['kind'] == 'CREATE_AND_REVIEW_ENVIRONMENT'
    assert handoff['authority_required'] == 'repository-settings-admin'
    assert handoff['external_write_required'] is True
    assert handoff['settings_url'].endswith('/settings/environments')
    assert any('Prevent self-review' in step for step in handoff['steps'])
    assert any('develop branch' in step for step in handoff['steps'])
    assert '--require-ready' in handoff['verification_command']
    assert handoff['writes_performed'] is False

    card = CHECKER.render_human(report)
    assert 'Operator handoff (not executed):' in card
    assert 'Authority required: repository-settings-admin' in card
    assert '1. Create an environment named candidate-images.' in card
    assert 'Environment writes performed: no' in card
    assert 'E2 dispatch authorized: no' in card


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        ('self-review', 'prevent self-review'),
        ('unknown-rule', 'unknown protection rule'),
        ('wrong-branch', 'allow develop only'),
        ('truncated-policies', 'response is truncated'),
    ],
)
def test_unsafe_or_ambiguous_protection_is_misconfigured(
    mutation: str,
    message: str,
):
    """Every weaker or unknown protection state must fail closed."""
    environment = _environment()
    policies = _policies()
    if mutation == 'self-review':
        environment['protection_rules'][0]['prevent_self_review'] = False
    elif mutation == 'unknown-rule':
        environment['protection_rules'].append({
            'id': 3,
            'type': 'wait_timer',
        })
    elif mutation == 'wrong-branch':
        policies = _policies('release/*')
    else:
        policies['total_count'] = 2

    report = CHECKER.audit_candidate_environment(
        fetcher=_ready_fetcher(
            environment=environment,
            policies=policies,
        )
    )

    assert report['status'] == 'MISCONFIGURED'
    assert message in report['decision']['next_action']
    assert report['observed']['target'] is None
    assert report['operator_handoff']['kind'] == (
        'REPAIR_AND_REVIEW_ENVIRONMENT'
    )


def test_incomplete_inventory_or_api_failure_is_blocked_not_absent():
    """Incomplete or inaccessible evidence is BLOCKED, never ABSENT."""
    truncated = _inventory('candidate-images')
    truncated['total_count'] = 2
    report = CHECKER.audit_candidate_environment(fetcher=FakeFetcher({
        INVENTORY_PATH: (200, truncated),
    }))
    assert report['status'] == 'BLOCKED'
    assert 'truncated' in report['decision']['next_action']
    assert report['operator_handoff']['kind'] == 'RESTORE_READ_ACCESS'
    assert report['operator_handoff']['settings_url'] is None
    assert report['operator_handoff']['external_write_required'] is False
    assert 'do not change environment settings' in ' '.join(
        report['operator_handoff']['steps']
    )

    report = CHECKER.audit_candidate_environment(fetcher=FakeFetcher({
        INVENTORY_PATH: (403, {'message': 'forbidden'}),
    }))
    assert report['status'] == 'BLOCKED'
    assert 'HTTP 403' in report['decision']['next_action']


def test_schema_rejects_any_claimed_write_or_dispatch_authority():
    """The persisted contract cannot acquire remote-write authority."""
    schema = json.loads(SCHEMA.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(schema)
    report = CHECKER.audit_candidate_environment(fetcher=_ready_fetcher())
    jsonschema.Draft7Validator(schema).validate(report)

    unsafe = copy.deepcopy(report)
    unsafe['decision']['dispatch_authorized'] = True
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.Draft7Validator(schema).validate(unsafe)

    unsafe = copy.deepcopy(report)
    unsafe['authority']['remote_mutations_performed'] = True
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.Draft7Validator(schema).validate(unsafe)

    inconsistent = copy.deepcopy(report)
    inconsistent['status'] = 'ABSENT'
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.Draft7Validator(schema).validate(inconsistent)

    unsafe = copy.deepcopy(report)
    unsafe['operator_handoff']['writes_performed'] = True
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.Draft7Validator(schema).validate(unsafe)

    inconsistent = copy.deepcopy(report)
    inconsistent['operator_handoff']['kind'] = (
        'CREATE_AND_REVIEW_ENVIRONMENT'
    )
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.Draft7Validator(schema).validate(inconsistent)


def test_github_transport_is_get_only_and_has_no_request_body(monkeypatch):
    """The live preflight cannot mutate GitHub through its HTTP transport."""
    captured = {}

    class Response:
        """Provide one bounded JSON response as a context manager."""

        status = 200

        def __enter__(self):
            """Return this synthetic response."""
            return self

        def __exit__(self, *_args):
            """Close the synthetic response without suppressing errors."""
            return False

        @staticmethod
        def read(_limit):
            """Return one empty repository environment inventory."""
            return b'{"total_count": 0, "environments": []}'

    def urlopen(request, *, timeout):
        captured['request'] = request
        captured['timeout'] = timeout
        return Response()

    monkeypatch.setattr(CHECKER.urllib.request, 'urlopen', urlopen)
    monkeypatch.setenv('GITHUB_TOKEN', 'test-token')

    status, payload = CHECKER._github_json(INVENTORY_PATH)

    request = captured['request']
    assert status == 200
    assert payload == {'total_count': 0, 'environments': []}
    assert request.get_method() == 'GET'
    assert request.data is None
    assert request.get_header('Authorization') == 'Bearer test-token'
    assert captured['timeout'] == 30


def test_checker_is_shipped_in_the_release_evidence_tooling():
    """A release bundle retains the exact preflight used by maintainers."""
    builder = (SCRIPTS / 'build_release_bundle.py').read_text(
        encoding='utf-8'
    )

    assert "'scripts/check_candidate_environment.py'" in builder
    assert SCHEMA.is_file()
