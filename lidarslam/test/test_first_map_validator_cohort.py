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

"""Tests for the bounded independent first-map cohort contract."""

from __future__ import annotations

import copy
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
SCRIPT = ROOT / 'scripts' / 'first_map_validator_cohort.py'
CONTRACT = (
    ROOT / 'docs' / 'contracts' / 'first-map-validator-cohort-v1.json'
)
SCHEMA = ROOT / 'docs' / 'schemas' / 'first-map-validator-cohort-v1.schema.json'
STATE = (
    ROOT
    / 'docs'
    / 'evidence'
    / 'growth'
    / 'first-map-validator-cohort-state.json'
)
STATE_SCHEMA = (
    ROOT
    / 'docs'
    / 'schemas'
    / 'first-map-validator-cohort-state-v1.schema.json'
)
ACCEPTED_LEDGER = (
    ROOT / 'docs' / 'evidence' / 'external-first-map-validations.json'
)
ACCEPTED_SCHEMA = (
    ROOT
    / 'docs'
    / 'schemas'
    / 'external-first-map-validations-v1.schema.json'
)


def _load_module():
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location('cohort_contract', SCRIPT)
        assert spec is not None
        assert spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


def _payload(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def _ready_contract() -> dict:
    contract = _payload(CONTRACT)
    contract['status'] = 'COPY_READY_NOT_AUTHORIZED'
    gates = contract['launch_gates']
    gates.update({
        'public_revision': 'a' * 40,
        'public_revision_resolvable': True,
        'comparable_docker_row': True,
        'comparable_source_row': True,
        'canonical_documentation_path': 'docker-first-map',
        'canonical_documentation_url': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'getting-started.html#docker-first-map-no-ros-2-workspace'
        ),
        'canonical_documentation_provenance': {
            'manifest_url': (
                'https://rsasaki0109.github.io/lidar_slam_ros2/'
                'docs-deployment-v1.json'
            ),
            'source_revision': 'a' * 40,
            'page_sha256': 'd' * 64,
        },
        'canonical_runtime_ref': (
            'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:' + 'b' * 64
        ),
        'copy_ready_handoff_public': True,
    })
    return contract


def _state() -> dict:
    return copy.deepcopy(_payload(STATE))


def _observed_state() -> dict:
    state = _state()
    state['operational_signals'].update({
        'observed_at': '2026-08-12T15:00:00Z',
        'audit_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/pull/427'
        ),
        'public_source_or_documentation_drift': False,
        'supported_p0_open': False,
        'failed_release_gate': False,
        'privacy_or_safety_incident': False,
    })
    return state


def _attempt(
    number: int,
    status: str,
    *,
    minutes: float = 8,
    blocker_id: str | None = None,
    accepted_validation_id: str | None = None,
) -> dict:
    active = status == 'active'
    failed = status in {'reported-fail', 'closed-fail', 'abandoned'}
    return {
        'id': f'attempt-{number:03d}',
        'status': status,
        'started_at': f'2026-08-{number:02d}T10:00:00Z',
        'completed_at': (
            None if active else f'2026-08-{number:02d}T11:00:00Z'
        ),
        'documentation_path': 'docker-first-map',
        'documentation_page_sha256': 'd' * 64,
        'public_revision': 'a' * 40,
        'runtime_ref': (
            'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:' + 'b' * 64
        ),
        'report_url': (
            None
            if active
            else 'https://github.com/rsasaki0109/lidar_slam_ros2/issues/'
            f'{500 + number}'
        ),
        'active_operator_minutes': None if active else minutes,
        'blocker_id': blocker_id if failed else None,
        'blocker_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/issues/'
            f'{600 + number}'
            if failed else None
        ),
        'accepted_validation_id': accepted_validation_id,
    }


def _accepted_validation(number: int) -> dict:
    return {
        'id': f'validation-{number:03d}',
        'reporter': f'@external-user-{number}',
        'issue_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/issues/'
            f'{500 + number}'
        ),
        'submitted_at': f'2026-08-{number:02d}T11:00:00Z',
        'independent_attestation': True,
        'documentation_path': 'docker-first-map',
        'release_ref': (
            'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:' + 'b' * 64
        ),
        'environment': {
            'os': 'Ubuntu 24.04',
            'architecture': 'amd64',
            'ros_distro': 'container-managed',
            'install_method': 'ghcr',
        },
        'exact_command': 'docker run --rm immutable-first-map-image',
        'result': 'passed',
        'verification': {
            'manifest_status': 'succeeded',
            'diagnosis_status': 'success',
            'autoware_status': 'PASS',
            'manifest_sha256': f'{number:064x}',
        },
        'findings': [],
        'acceptance': {
            'status': 'accepted',
            'reviewed_by': '@rsasaki0109',
            'reviewed_at': f'2026-08-{number:02d}T12:00:00Z',
            'review_url': (
                'https://github.com/rsasaki0109/lidar_slam_ros2/issues/'
                f'{500 + number}#issuecomment-{9000 + number}'
            ),
            'findings_status': 'no-findings',
            'resolution_urls': [],
        },
    }


def _evaluate(module, contract=None, state=None, ledger=None):
    return module.evaluate_state(
        contract or _ready_contract(),
        _payload(SCHEMA),
        state or _observed_state(),
        _payload(STATE_SCHEMA),
        ledger or _payload(ACCEPTED_LEDGER),
        _payload(ACCEPTED_SCHEMA),
        now=datetime(2026, 8, 12, 16, 0, tzinfo=timezone.utc),
    )


def test_tracked_contract_is_valid_waiting_and_read_only():
    module = _load_module()

    report = module.validate_contract(_payload(CONTRACT), _payload(SCHEMA))

    assert report['status'] == 'WAITING_FOR_PUBLIC_GATES'
    assert report['copy_ready'] is False
    assert report['pending_launch_gates'] == [
        'comparable_docker_row',
        'comparable_source_row',
        'copy_ready_handoff_public',
    ]
    assert report['accepted_target'] == 3
    assert report['initial_attempt_cap'] == 5
    assert report['hard_attempt_cap'] == 10
    assert report['max_concurrent_attempts'] == 2
    assert report['community_posts_authorized'] is False
    assert report['github_writes_authorized'] is False
    assert report['remote_mutations_performed'] is False


def test_tracked_operational_state_is_empty_waiting_and_private():
    module = _load_module()

    report = _evaluate(
        module,
        contract=_payload(CONTRACT),
        state=_state(),
    )

    assert report['status'] == 'WAITING_FOR_PUBLIC_GATES'
    assert report['attempt_count'] == 0
    assert report['accepted_validations'] == 0
    assert report['stop_conditions'] == []
    assert report['next_attempt_permitted_by_state'] is False
    assert 'reporter' not in json.dumps(_state())


def test_human_summary_names_the_boundary_and_one_next_action():
    module = _load_module()
    report = _evaluate(
        module,
        contract=_payload(CONTRACT),
        state=_state(),
    )

    rendered = module.render_state_summary(report)

    assert 'Status: **WAITING_FOR_PUBLIC_GATES**' in rendered
    assert 'Accepted: **0 / 3**' in rendered
    assert 'Completion rate: not measured' in rendered
    assert 'GitHub/community write authorized: no' in rendered
    assert 'Pending launch gates: comparable_docker_row' in rendered
    assert 'copy_ready_handoff_public' in rendered
    assert 'Run the missing clean comparable Docker/source matrix rows' in (
        rendered
    )
    assert 'Publish the reviewed candidate' not in rendered
    assert 'reporter' not in rendered


def test_every_operational_status_has_exactly_one_next_action():
    module = _load_module()

    assert set(module.NEXT_ACTIONS) == {
        'WAITING_FOR_PUBLIC_GATES',
        'WAITING_FOR_OPERATIONAL_SIGNALS',
        'PAUSED_REPAIR',
        'TARGET_MET',
        'HARD_CAP_REVIEW',
        'INITIAL_BATCH_REVIEW',
        'CAPACITY_FULL',
        'READY_FOR_NEXT_ATTEMPT',
    }
    assert all(action.strip() for action in module.NEXT_ACTIONS.values())


def test_ready_observed_empty_state_permits_one_attempt_but_no_write():
    module = _load_module()

    report = _evaluate(module)

    assert report['status'] == 'READY_FOR_NEXT_ATTEMPT'
    assert report['operational_signals_fresh'] is True
    assert report['next_attempt_permitted_by_state'] is True
    assert report['community_posts_authorized'] is False
    assert report['github_writes_authorized'] is False
    assert report['remote_mutations_performed'] is False


def test_two_active_attempts_fill_capacity_and_three_are_rejected():
    module = _load_module()
    state = _observed_state()
    state['attempts'] = [_attempt(1, 'active'), _attempt(2, 'active')]

    report = _evaluate(module, state=state)

    assert report['status'] == 'CAPACITY_FULL'
    assert report['active_attempt_count'] == 2
    state['attempts'].append(_attempt(3, 'active'))
    with pytest.raises(module.CohortError, match='two-attempt WIP'):
        _evaluate(module, state=state)

    state['attempts'] = [
        _attempt(1, 'reported-pass'),
        _attempt(2, 'active'),
    ]
    report = _evaluate(module, state=state)
    assert report['status'] == 'CAPACITY_FULL'
    assert report['review_wip_count'] == 2
    state['attempts'].append(_attempt(3, 'active'))
    with pytest.raises(module.CohortError, match='combined WIP'):
        _evaluate(module, state=state)


def test_active_attempt_must_use_the_current_canonical_route():
    module = _load_module()
    state = _observed_state()
    attempt = _attempt(1, 'active')
    attempt['public_revision'] = 'c' * 40
    state['attempts'] = [attempt]

    with pytest.raises(module.CohortError, match='canonical path'):
        _evaluate(module, state=state)


def test_active_attempt_must_use_the_exact_public_page_bytes():
    module = _load_module()
    state = _observed_state()
    attempt = _attempt(1, 'active')
    attempt['documentation_page_sha256'] = 'e' * 64
    state['attempts'] = [attempt]

    with pytest.raises(module.CohortError, match='canonical path'):
        _evaluate(module, state=state)


def test_documentation_provenance_must_match_public_revision():
    module = _load_module()
    contract = _ready_contract()
    contract['launch_gates']['canonical_documentation_provenance'][
        'source_revision'
    ] = 'c' * 40

    with pytest.raises(
        module.CohortError,
        match='must match the public revision',
    ):
        module.validate_contract(contract, _payload(SCHEMA))


def test_two_unreviewed_reports_pause_recruitment():
    module = _load_module()
    state = _observed_state()
    state['attempts'] = [
        _attempt(1, 'reported-pass'),
        _attempt(2, 'reported-pass'),
    ]

    report = _evaluate(module, state=state)

    assert report['status'] == 'PAUSED_REPAIR'
    assert report['unreviewed_report_count'] == 2
    assert 'two-unreviewed-receipts' in report['stop_conditions']


def test_repeated_public_blocker_pauses_recruitment():
    module = _load_module()
    state = _observed_state()
    state['attempts'] = [
        _attempt(1, 'closed-fail', blocker_id='missing-runtime-package'),
        _attempt(2, 'closed-fail', blocker_id='missing-runtime-package'),
    ]

    report = _evaluate(module, state=state)

    assert report['status'] == 'PAUSED_REPAIR'
    assert report['repeated_blocker_ids'] == ['missing-runtime-package']
    assert 'two-attempts-share-one-blocker' in report['stop_conditions']


@pytest.mark.parametrize(
    ('signal', 'stop_id'),
    [
        (
            'public_source_or_documentation_drift',
            'public-source-or-documentation-drift',
        ),
        ('supported_p0_open', 'supported-p0-open'),
        ('failed_release_gate', 'failed-release-gate'),
        ('privacy_or_safety_incident', 'privacy-or-safety-incident'),
    ],
)
def test_each_operational_signal_stops_recruitment(
    signal: str,
    stop_id: str,
):
    module = _load_module()
    state = _observed_state()
    state['operational_signals'][signal] = True

    report = _evaluate(module, state=state)

    assert report['status'] == 'PAUSED_REPAIR'
    assert report['stop_conditions'] == [stop_id]


def test_initial_five_attempt_batch_requires_review_before_extension():
    module = _load_module()
    state = _observed_state()
    state['attempts'] = [
        _attempt(number, 'closed-pass') for number in range(1, 6)
    ]

    report = _evaluate(module, state=state)

    assert report['status'] == 'INITIAL_BATCH_REVIEW'
    state['phase'] = 'extended'
    with pytest.raises(module.CohortError, match='public decision URL'):
        _evaluate(module, state=state)


def test_attempt_ten_enforces_completion_and_median_time_thresholds():
    module = _load_module()
    state = _observed_state()
    state['phase'] = 'extended'
    state['extension_decision_url'] = (
        'https://github.com/rsasaki0109/lidar_slam_ros2/issues/'
        '422#issuecomment-9000'
    )
    state['attempts'] = [
        _attempt(number, 'closed-pass', minutes=12)
        if number <= 7
        else _attempt(
            number,
            'closed-fail',
            minutes=12,
            blocker_id=f'unique-blocker-{number}',
        )
        for number in range(1, 11)
    ]

    report = _evaluate(module, state=state)

    assert report['completion_rate'] == pytest.approx(0.7)
    assert report['median_active_operator_minutes'] == 12
    assert 'completion-below-80-percent-at-attempt-10' in (
        report['stop_conditions']
    )
    assert 'median-active-time-above-10-minutes-at-attempt-10' in (
        report['stop_conditions']
    )


def test_accepted_attempt_must_match_the_authoritative_ledger():
    module = _load_module()
    state = _observed_state()
    state['attempts'] = [
        _attempt(
            1,
            'accepted-pass',
            accepted_validation_id='validation-001',
        )
    ]
    ledger = _payload(ACCEPTED_LEDGER)
    ledger['validations'] = [_accepted_validation(1)]

    report = _evaluate(module, state=state, ledger=ledger)

    assert report['accepted_validations'] == 1
    state['attempts'][0]['report_url'] = (
        'https://github.com/rsasaki0109/lidar_slam_ros2/issues/999'
    )
    with pytest.raises(module.CohortError, match='accepted ledger evidence'):
        _evaluate(module, state=state, ledger=ledger)
    state['attempts'][0]['report_url'] = (
        'https://github.com/rsasaki0109/lidar_slam_ros2/issues/501'
    )
    state['attempts'][0]['accepted_validation_id'] = 'validation-002'
    with pytest.raises(module.CohortError, match='absent from'):
        _evaluate(module, state=state, ledger=ledger)

    state['attempts'] = []
    report = _evaluate(module, state=state, ledger=ledger)
    assert report['accepted_validations'] == 0


def test_three_ledger_bound_acceptances_meet_the_cohort_target():
    module = _load_module()
    state = _observed_state()
    state['attempts'] = [
        _attempt(
            number,
            'accepted-pass',
            accepted_validation_id=f'validation-{number:03d}',
        )
        for number in range(1, 4)
    ]
    ledger = _payload(ACCEPTED_LEDGER)
    ledger['validations'] = [
        _accepted_validation(number) for number in range(1, 4)
    ]

    report = _evaluate(module, state=state, ledger=ledger)

    assert report['status'] == 'TARGET_MET'
    assert report['accepted_validations'] == 3
    assert report['next_attempt_permitted_by_state'] is False


def test_stale_or_future_operational_signals_fail_closed():
    module = _load_module()
    state = _observed_state()
    state['operational_signals']['observed_at'] = '2026-08-10T15:59:59Z'

    report = _evaluate(module, state=state)

    assert report['status'] == 'WAITING_FOR_OPERATIONAL_SIGNALS'
    assert report['operational_signals_fresh'] is False
    state['operational_signals']['observed_at'] = '2026-08-12T16:06:00Z'
    with pytest.raises(module.CohortError, match='future-dated'):
        _evaluate(module, state=state)


def test_attempt_ids_must_be_consecutive_and_ordered():
    module = _load_module()
    state = _observed_state()
    state['attempts'] = [_attempt(2, 'active')]

    with pytest.raises(module.CohortError, match='consecutive and ordered'):
        _evaluate(module, state=state)


def test_state_rejects_partial_signals_and_identity_fields():
    module = _load_module()
    state = _state()
    state['operational_signals']['supported_p0_open'] = False
    with pytest.raises(module.CohortError, match='wholly observed'):
        _evaluate(module, state=state)

    state = _observed_state()
    attempt = _attempt(1, 'active')
    attempt['reporter'] = '@must-not-be-stored'
    state['attempts'] = [attempt]
    with pytest.raises(module.CohortError, match='state schema validation'):
        _evaluate(module, state=state)


def test_render_is_blocked_until_all_public_product_gates_pass():
    module = _load_module()

    with pytest.raises(module.CohortError, match='public revision'):
        module.render_recruitment(_payload(CONTRACT))

    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--render', '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1
    assert json.loads(result.stdout)['status'] == (
        'COHORT_INVALID_OR_BLOCKED'
    )


def test_cli_render_is_blocked_by_unobserved_operational_state(tmp_path):
    contract_path = tmp_path / 'ready-contract.json'
    contract_path.write_text(
        json.dumps(_ready_contract()),
        encoding='utf-8',
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--contract',
            str(contract_path),
            '--render',
            '--json',
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    report = json.loads(result.stdout)
    assert report['status'] == 'COHORT_INVALID_OR_BLOCKED'
    assert 'WAITING_FOR_OPERATIONAL_SIGNALS' in report['error']


def test_default_cli_prints_the_human_summary_without_mutation():
    result = subprocess.run(
        [sys.executable, str(SCRIPT)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert '# Independent first-map cohort' in result.stdout
    assert 'WAITING_FOR_PUBLIC_GATES' in result.stdout
    assert 'GitHub/community write authorized: no' in result.stdout
    assert result.stderr == ''


def test_ready_contract_renders_bounded_privacy_first_recruitment():
    module = _load_module()
    contract = _ready_contract()

    report = module.validate_contract(contract, _payload(SCHEMA))
    rendered = module.render_recruitment(contract)

    assert report['status'] == 'COPY_READY_NOT_AUTHORIZED'
    assert report['copy_ready'] is True
    assert report['pending_launch_gates'] == []
    assert 'a' * 40 in rendered
    assert (
        'Exact product identity: '
        'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:' + 'b' * 64
    ) in rendered
    assert 'Public page SHA-256: ' + 'd' * 64 in rendered
    assert 'at most 5 attempts' in rendered
    assert 'no more than 2 at once' in rendered
    assert 'not a lidarslam_ros2 maintainer' in rendered
    assert 'Please report both PASS and FAIL outcomes' in rendered
    assert 'Attach only the reviewed' in rendered
    assert 'Do not upload a bag, map, trajectory' in rendered
    assert 'No product telemetry is used' in rendered
    assert 'explicit community-write approval' in rendered


@pytest.mark.parametrize(
    ('path', 'value', 'match'),
    [
        (
            ('authority', 'community_posts_authorized'),
            True,
            'schema validation',
        ),
        (
            ('capacity', 'max_concurrent_attempts'),
            3,
            'schema validation',
        ),
        (
            ('status',),
            'COPY_READY_NOT_AUTHORIZED',
            'status must be WAITING_FOR_PUBLIC_GATES',
        ),
        (
            ('launch_gates', 'canonical_documentation_path'),
            None,
            'path, URL, and provenance must be set together',
        ),
    ],
)
def test_contract_rejects_authority_capacity_and_state_drift(
    path: tuple[str, ...],
    value,
    match: str,
):
    module = _load_module()
    contract = copy.deepcopy(_payload(CONTRACT))
    target = contract
    for part in path[:-1]:
        target = target[part]
    target[path[-1]] = value

    with pytest.raises(module.CohortError, match=match):
        module.validate_contract(contract, _payload(SCHEMA))
    with pytest.raises(module.CohortError, match='recruitment text is blocked'):
        module.render_recruitment(contract)


def test_every_required_stop_condition_is_fixed():
    contract = _payload(CONTRACT)
    assert set(contract['stop_conditions']) == {
        'public-source-or-documentation-drift',
        'supported-p0-open',
        'failed-release-gate',
        'two-unreviewed-receipts',
        'two-attempts-share-one-blocker',
        'privacy-or-safety-incident',
        'completion-below-80-percent-at-attempt-10',
        'median-active-time-above-10-minutes-at-attempt-10',
    }


@pytest.mark.parametrize(
    ('path', 'url', 'runtime_ref', 'match'),
    [
        (
            'docker-first-map',
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'getting-started.html#docker-first-map-no-ros-2-workspace',
            'a' * 40,
            'Docker path requires an immutable GHCR digest',
        ),
        (
            'source-quickstart',
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'getting-started.html#1-install-and-build-from-source',
            'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:' + 'b' * 64,
            'source path runtime identity must equal the public revision',
        ),
    ],
)
def test_runtime_identity_must_match_the_selected_path(
    path: str,
    url: str,
    runtime_ref: str,
    match: str,
):
    module = _load_module()
    contract = _ready_contract()
    gates = contract['launch_gates']
    gates['canonical_documentation_path'] = path
    gates['canonical_documentation_url'] = url
    gates['canonical_runtime_ref'] = runtime_ref

    with pytest.raises(module.CohortError, match=match):
        module.validate_contract(contract, _payload(SCHEMA))
