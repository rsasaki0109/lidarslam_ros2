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

"""Tests for privacy-bounded weekly project growth snapshots."""

from __future__ import annotations

from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path

import jsonschema
import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'collect_growth_snapshot.py'
SPEC = importlib.util.spec_from_file_location('collect_growth_snapshot', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
GROWTH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GROWTH)


CAPTURED_TEXT = '2026-08-10T00:00:00Z'
CAPTURED = datetime(2026, 8, 10, tzinfo=timezone.utc)


def _user(login: str, user_type: str = 'User') -> dict[str, str]:
    return {'login': login, 'type': user_type}


def _github_records() -> dict[str, object]:
    return {
        'repository': {
            'stargazers_count': 837,
            'forks_count': 172,
            'subscribers_count': 12,
        },
        'views': {'count': 500, 'uniques': 299, 'views': []},
        'clones': {'count': 900, 'uniques': 259, 'clones': []},
        'referrers': [
            {'referrer': 'google.com', 'count': 20, 'uniques': 10},
            {'referrer': 'autoware.org', 'count': 7, 'uniques': 4},
            {'referrer': 'docs.tier4.jp', 'count': 3, 'uniques': 2},
        ],
        'release': {
            'tag_name': 'v0.9.0',
            'published_at': '2026-07-31T00:00:00Z',
            'assets': [
                {
                    'name': 'lidarslam_ros2_v0.9.0_release_bundle.tar.gz',
                    'download_count': 8,
                },
                {'name': 'release-image-jazzy.json', 'download_count': 4},
            ],
        },
        'pulls': [
            {
                'user': _user('external-a'),
                'created_at': '2026-08-01T00:00:00Z',
                'merged_at': None,
            },
            {
                'user': _user('external-b'),
                'created_at': '2026-06-01T00:00:00Z',
                'merged_at': '2026-07-01T00:00:00Z',
            },
            {
                'user': _user('external-old'),
                'created_at': '2026-01-01T00:00:00Z',
                'merged_at': '2026-01-02T00:00:00Z',
            },
            {
                'user': _user('rsasaki0109'),
                'created_at': '2026-08-02T00:00:00Z',
                'merged_at': '2026-08-03T00:00:00Z',
            },
            {
                'user': _user('dependabot[bot]', 'Bot'),
                'created_at': '2026-08-02T00:00:00Z',
                'merged_at': '2026-08-03T00:00:00Z',
            },
        ],
        'issues': [
            {
                'number': 1,
                'state': 'open',
                'labels': [],
                'user': _user('map-user-a'),
                'created_at': '2026-08-01T00:00:00Z',
                'body': 'raw issue body must not be retained',
            },
            {
                'number': 2,
                'state': 'open',
                'labels': [{'name': 'bug'}],
                'user': _user('map-user-b'),
                'created_at': '2026-08-05T00:00:00Z',
            },
            {
                'number': 3,
                'state': 'open',
                'labels': [],
                'user': _user('rsasaki0109'),
                'created_at': '2026-08-04T00:00:00Z',
            },
            {
                'number': 4,
                'state': 'closed',
                'labels': [{'name': 'question'}],
                'user': _user('map-user-c'),
                'created_at': '2026-07-01T00:00:00Z',
            },
            {
                'number': 5,
                'state': 'open',
                'labels': [],
                'user': _user('map-user-d'),
                'created_at': '2026-08-03T00:00:00Z',
                'pull_request': {'url': 'private raw URL'},
            },
        ],
        'comments': {
            1: [
                {
                    'user': _user('rsasaki0109'),
                    'created_at': '2026-08-02T00:00:00Z',
                    'body': 'raw maintainer comment must not be retained',
                },
            ],
            2: [],
            4: [
                {
                    'user': _user('another-user'),
                    'created_at': '2026-07-01T06:00:00Z',
                },
                {
                    'user': _user('rsasaki0109'),
                    'created_at': '2026-07-01T12:00:00Z',
                },
            ],
        },
    }


class FakeApi:
    source = 'fixture'

    def __init__(self, records: dict[str, object]) -> None:
        self.records = records

    def get(self, endpoint, params=None):
        del params
        if endpoint.endswith('/traffic/views'):
            return self.records['views']
        if endpoint.endswith('/traffic/clones'):
            return self.records['clones']
        if endpoint.endswith('/traffic/popular/referrers'):
            return self.records['referrers']
        if endpoint.endswith('/releases/latest'):
            return self.records['release']
        return self.records['repository']

    def get_all(self, endpoint, params=None):
        del params
        if endpoint.endswith('/pulls'):
            return self.records['pulls']
        if endpoint.endswith('/issues'):
            return self.records['issues']
        number = int(endpoint.rsplit('/', 2)[-2])
        return self.records['comments'][number]


def _product_reports(captured_at=None):
    del captured_at
    return (
        {
            'accepted_validations': 0,
            'required_validations': 3,
            'remaining_validations': 3,
        },
        {
            'cohort_id': 'g1-first-map-cohort-2026-08',
            'status': 'WAITING_FOR_PUBLIC_GATES',
            'phase': 'initial',
            'attempt_count': 0,
            'terminal_attempt_count': 0,
            'active_attempt_count': 0,
            'review_wip_count': 0,
            'successful_first_map_count': 0,
            'accepted_validations': 0,
            'accepted_target': 3,
            'completion_rate': None,
            'median_active_operator_minutes': None,
            'operational_signals_fresh': False,
            'next_attempt_permitted_by_state': False,
            'stop_conditions': [],
        },
        {
            'status': 'NOT_READY',
            'summary': {'complete': 8, 'incomplete': 2, 'total': 10},
        },
    )


def _collect(records=None):
    first_map, cohort, readiness = _product_reports()
    return GROWTH.collect_snapshot(
        api=FakeApi(records or _github_records()),
        captured_at=CAPTURED,
        maintainers={'rsasaki0109'},
        first_map_report=first_map,
        cohort_report=cohort,
        readiness_report=readiness,
        annotations=['v0.9 onboarding baseline'],
    )


def test_snapshot_aggregates_growth_activation_and_community_metrics():
    snapshot = _collect()

    assert snapshot['goal'] == {
        'target_stars': 1000,
        'stars_to_goal': 163,
    }
    assert snapshot['github']['stars'] == 837
    assert snapshot['github']['forks'] == 172
    assert snapshot['github']['subscribers'] == 12
    assert snapshot['github']['traffic_14d'] == {
        'views': {'total': 500, 'unique': 299},
        'clones': {'total': 900, 'unique': 259},
        'referrals': {
            'top_referrer_unique_sum': 16,
            'autoware_tier4_unique_sum': 6,
        },
    }
    assert snapshot['github']['latest_release'] == {
        'tag': 'v0.9.0',
        'published_at': '2026-07-31T00:00:00Z',
        'total_asset_downloads': 12,
        'primary_bundle': {
            'name': 'lidarslam_ros2_v0.9.0_release_bundle.tar.gz',
            'downloads': 8,
        },
    }
    assert snapshot['github']['community'] == {
        'open_issues': 3,
        'untriaged_open_issues': 2,
        'external_prs_90d': 2,
        'external_merged_contributors_180d': 1,
        'issue_first_response_90d': {
            'scope': 'external_non_bot_issues',
            'eligible': 3,
            'responded': 2,
            'unanswered': 1,
            'median_hours': 18.0,
        },
    }
    assert snapshot['product'] == {
        'external_first_maps': {
            'accepted': 0,
            'required': 3,
            'remaining': 3,
            'cohort': {
                'id': 'g1-first-map-cohort-2026-08',
                'status': 'WAITING_FOR_PUBLIC_GATES',
                'phase': 'initial',
                'attempted': 0,
                'terminal': 0,
                'active': 0,
                'review_wip': 0,
                'successful': 0,
                'accepted': 0,
                'target': 3,
                'completion_rate': None,
                'median_active_operator_minutes': None,
                'operational_signals_fresh': False,
                'next_attempt_permitted_by_state': False,
                'stop_conditions': [],
            },
        },
        'v1_readiness': {
            'status': 'NOT_READY',
            'complete': 8,
            'incomplete': 2,
            'total': 10,
        },
    }


def test_snapshot_validates_against_the_published_schema():
    snapshot = _collect()
    schema = json.loads(GROWTH.DEFAULT_SCHEMA.read_text(encoding='utf-8'))

    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.Draft7Validator(
        schema,
        format_checker=jsonschema.FormatChecker(),
    ).validate(snapshot)


def test_historical_snapshot_remains_valid_without_cohort_extension():
    snapshot = json.loads(
        (ROOT / 'docs/evidence/growth/2026-08-10.json').read_text(
            encoding='utf-8'
        )
    )
    schema = json.loads(GROWTH.DEFAULT_SCHEMA.read_text(encoding='utf-8'))

    jsonschema.Draft7Validator(
        schema,
        format_checker=jsonschema.FormatChecker(),
    ).validate(snapshot)


def test_live_local_product_metrics_include_fail_closed_cohort_state():
    first_map, cohort, readiness = GROWTH.collect_product_metrics(
        repo_root=ROOT,
        captured_at=datetime(2026, 8, 13, tzinfo=timezone.utc),
    )

    assert first_map['accepted_validations'] == 0
    assert cohort['status'] == 'WAITING_FOR_PUBLIC_GATES'
    assert cohort['attempt_count'] == 0
    assert cohort['github_writes_authorized'] is False
    assert readiness['status'] == 'NOT_READY'


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        (
            lambda first_map, cohort: cohort.update(completion_rate=1.1),
            'completion_rate',
        ),
        (
            lambda first_map, cohort: cohort.update(
                terminal_attempt_count=1,
                median_active_operator_minutes=5,
            ),
            'equal attempted',
        ),
        (
            lambda first_map, cohort: cohort.update(
                accepted_validations=1,
            ),
            'counts conflict',
        ),
        (
            lambda first_map, cohort: cohort.update(accepted_target=4),
            'target differs',
        ),
        (
            lambda first_map, cohort: cohort.update(
                next_attempt_permitted_by_state=True,
            ),
            'decision conflicts',
        ),
    ],
)
def test_malformed_cohort_metrics_fail_closed(mutation, message):
    first_map, cohort, readiness = _product_reports()
    mutation(first_map, cohort)

    with pytest.raises(GROWTH.SnapshotError, match=message):
        GROWTH.collect_snapshot(
            api=FakeApi(_github_records()),
            captured_at=CAPTURED,
            maintainers={'rsasaki0109'},
            first_map_report=first_map,
            cohort_report=cohort,
            readiness_report=readiness,
        )


def test_snapshot_never_writes_raw_users_referrers_urls_or_text():
    rendered = json.dumps(_collect(), sort_keys=True)

    for private_value in (
        'external-a',
        'external-b',
        'map-user-a',
        'another-user',
        'google.com',
        'autoware.org',
        'docs.tier4.jp',
        'private raw URL',
        'raw issue body',
        'raw maintainer comment',
    ):
        assert private_value not in rendered
    assert json.loads(rendered)['privacy'] == {
        'personal_identifiers_written': False,
        'raw_records_written': False,
    }


def test_no_maintainer_response_is_reported_as_null_not_zero():
    records = _github_records()
    records['comments'] = {1: [], 2: [], 4: []}

    response = _collect(records)['github']['community'][
        'issue_first_response_90d'
    ]

    assert response == {
        'scope': 'external_non_bot_issues',
        'eligible': 3,
        'responded': 0,
        'unanswered': 3,
        'median_hours': None,
    }


def test_multiple_primary_release_bundles_fail_closed():
    records = _github_records()
    records['release']['assets'].append({
        'name': 'lidarslam_ros2_v0.9.1_release_bundle.tar.gz',
        'download_count': 1,
    })

    with pytest.raises(
        GROWTH.SnapshotError,
        match='multiple primary release bundles',
    ):
        _collect(records)


def test_naive_capture_timestamp_is_rejected():
    with pytest.raises(GROWTH.SnapshotError, match='UTC offset'):
        GROWTH._parse_timestamp('2026-08-10T00:00:00', 'capture')


def _write_fixture_dir(path: Path, records: dict[str, object]) -> None:
    payloads = {
        'repository.json': records['repository'],
        'traffic-views.json': records['views'],
        'traffic-clones.json': records['clones'],
        'traffic-referrers.json': records['referrers'],
        'latest-release.json': records['release'],
        'pulls.json': records['pulls'],
        'issues.json': records['issues'],
    }
    payloads.update({
        f'issue-comments-{number}.json': comments
        for number, comments in records['comments'].items()
    })
    path.mkdir()
    for filename, payload in payloads.items():
        (path / filename).write_text(
            json.dumps(payload),
            encoding='utf-8',
        )


def test_fixture_cli_creates_immutable_snapshot(
    tmp_path,
    monkeypatch,
    capsys,
):
    fixture_dir = tmp_path / 'fixtures'
    _write_fixture_dir(fixture_dir, _github_records())
    output = tmp_path / 'growth.json'
    monkeypatch.setattr(GROWTH, 'collect_product_metrics', _product_reports)
    command = [
        '--fixture-dir', str(fixture_dir),
        '--captured-at', CAPTURED_TEXT,
        '--output', str(output),
    ]

    assert GROWTH.main(command) == 0
    assert output.is_file()
    assert json.loads(output.read_text(encoding='utf-8'))['source'][
        'github'
    ] == 'fixture'
    assert GROWTH.main(command) == 2
    captured = capsys.readouterr()
    assert 'without overwriting' in captured.err
