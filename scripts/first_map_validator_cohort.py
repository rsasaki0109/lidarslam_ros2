#!/usr/bin/env python3
"""Validate or render the bounded independent first-map cohort packet."""

from __future__ import annotations

import argparse
import copy
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
from statistics import median
import sys
from typing import Any, Sequence

from check_external_first_map_readiness import (
    LedgerError,
    validate_ledger_payload,
)

import jsonschema


sys.dont_write_bytecode = True

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = (
    REPO_ROOT / 'docs' / 'contracts' / 'first-map-validator-cohort-v1.json'
)
DEFAULT_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'first-map-validator-cohort-v1.schema.json'
)
DEFAULT_STATE = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'growth'
    / 'first-map-validator-cohort-state.json'
)
DEFAULT_STATE_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'first-map-validator-cohort-state-v1.schema.json'
)
DEFAULT_ACCEPTED_LEDGER = (
    REPO_ROOT / 'docs' / 'evidence' / 'external-first-map-validations.json'
)
DEFAULT_ACCEPTED_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'external-first-map-validations-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/'
    'first-map-validator-cohort-v1.schema.json'
)
ISSUE_FORM_URL = (
    'https://github.com/rsasaki0109/lidar_slam_ros2/issues/new'
    '?template=first-map-validation.yml'
)
NEXT_ACTIONS = {
    'WAITING_FOR_PUBLIC_GATES': (
        'Complete the remaining public launch gates in reported order before '
        'recording an attempt.'
    ),
    'WAITING_FOR_OPERATIONAL_SIGNALS': (
        'Record one complete public operational-signal audit no older than '
        '48 hours; do not recruit or start an attempt yet.'
    ),
    'PAUSED_REPAIR': (
        'Stop new attempts, repair every listed stop condition publicly, '
        'rerun the affected clean matrix row, and refresh the signal audit.'
    ),
    'TARGET_MET': (
        'Stop this cohort, retain the accepted evidence, and move to the v1 '
        'readiness decision without recruiting extra participants.'
    ),
    'HARD_CAP_REVIEW': (
        'Do not start attempt 11; review the ten-attempt completion, timing, '
        'acceptance, and blocker evidence before choosing a new cohort.'
    ),
    'INITIAL_BATCH_REVIEW': (
        'Review the first five attempts; continuing requires an exact public '
        'extension-decision comment and an updated proposed state.'
    ),
    'CAPACITY_FULL': (
        'Finish active work or review a reported outcome before admitting '
        'another attempt.'
    ),
    'READY_FOR_NEXT_ATTEMPT': (
        'The state layer permits one next attempt within WIP; recruitment or '
        'a public post still requires separate explicit authorization.'
    ),
}


class CohortError(ValueError):
    """The cohort contract is invalid or not copy-ready."""


def _load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise CohortError(f'cannot read {label} {path}: {exc}') from exc
    if not isinstance(payload, dict):
        raise CohortError(f'{label} root must be an object')
    return payload


def _schema_path(error: jsonschema.ValidationError) -> str:
    return '.'.join(str(item) for item in error.absolute_path) or '<root>'


def _validate_schema(
    payload: dict[str, Any],
    schema: dict[str, Any],
    label: str,
) -> None:
    try:
        jsonschema.Draft7Validator.check_schema(schema)
        errors = sorted(
            jsonschema.Draft7Validator(
                schema,
                format_checker=jsonschema.FormatChecker(),
            ).iter_errors(payload),
            key=lambda item: [str(part) for part in item.absolute_path],
        )
    except jsonschema.SchemaError as exc:
        raise CohortError(f'{label} schema is invalid: {exc.message}') from exc
    if errors:
        first = errors[0]
        raise CohortError(
            f'{label} schema validation failed at {_schema_path(first)}: '
            f'{first.message}'
        )


def _parse_datetime(value: str) -> datetime:
    parsed = datetime.fromisoformat(value.replace('Z', '+00:00'))
    if parsed.tzinfo is None:
        raise CohortError('timestamps must include a UTC offset')
    return parsed.astimezone(timezone.utc)


def _ready_gates(gates: dict[str, Any]) -> bool:
    path = gates['canonical_documentation_path']
    documentation_provenance = gates[
        'canonical_documentation_provenance'
    ]
    runtime_ref = gates['canonical_runtime_ref']
    identity_matches_path = (
        path == 'docker-first-map'
        and isinstance(runtime_ref, str)
        and runtime_ref.startswith('ghcr.io/')
    ) or (
        path == 'source-quickstart'
        and runtime_ref == gates['public_revision']
    )
    return (
        gates['public_revision'] is not None
        and gates['public_revision_resolvable']
        and gates['comparable_docker_row']
        and gates['comparable_source_row']
        and gates['canonical_documentation_path'] is not None
        and gates['canonical_documentation_url'] is not None
        and documentation_provenance is not None
        and documentation_provenance['source_revision']
        == gates['public_revision']
        and gates['canonical_runtime_ref'] is not None
        and gates['copy_ready_handoff_public']
        and identity_matches_path
    )


def _pending_launch_gates(gates: dict[str, Any]) -> list[str]:
    """Return stable field names for every launch prerequisite still open."""
    pending = []
    if gates['public_revision'] is None:
        pending.append('public_revision')
    if not gates['public_revision_resolvable']:
        pending.append('public_revision_resolvable')
    if not gates['comparable_docker_row']:
        pending.append('comparable_docker_row')
    if not gates['comparable_source_row']:
        pending.append('comparable_source_row')
    if gates['canonical_documentation_path'] is None:
        pending.append('canonical_documentation_path')
    if gates['canonical_documentation_url'] is None:
        pending.append('canonical_documentation_url')
    if gates['canonical_documentation_provenance'] is None:
        pending.append('canonical_documentation_provenance')
    if gates['canonical_runtime_ref'] is None:
        pending.append('canonical_runtime_ref')
    if not gates['copy_ready_handoff_public']:
        pending.append('copy_ready_handoff_public')
    return pending


def validate_contract(
    contract: dict[str, Any],
    schema: dict[str, Any],
) -> dict[str, Any]:
    """Validate schema, launch state, capacity, and no-write boundaries."""
    _validate_schema(contract, schema, 'contract')
    if contract['schema_uri'] != SCHEMA_URI:
        raise CohortError('contract schema_uri is not the supported v1 URI')

    capacity = contract['capacity']
    if not (
        capacity['accepted_target']
        <= capacity['initial_attempt_cap']
        <= capacity['hard_attempt_cap']
    ):
        raise CohortError('cohort attempt capacity is inconsistent')
    if capacity['max_concurrent_attempts'] > 2:
        raise CohortError('cohort exceeds the two-attempt review WIP limit')

    gates = contract['launch_gates']
    documentation_path = gates['canonical_documentation_path']
    documentation_url = gates['canonical_documentation_url']
    documentation_provenance = gates[
        'canonical_documentation_provenance'
    ]
    runtime_ref = gates['canonical_runtime_ref']
    documentation_values = (
        documentation_path,
        documentation_url,
        documentation_provenance,
    )
    if any(value is None for value in documentation_values) and any(
        value is not None for value in documentation_values
    ):
        raise CohortError(
            'canonical documentation path, URL, and provenance must be set '
            'together'
        )
    expected_fragments = {
        'docker-first-map': '#docker-first-map-no-ros-2-workspace',
        'source-quickstart': '#1-install-and-build-from-source',
    }
    if documentation_path is not None and expected_fragments[
        documentation_path
    ] not in documentation_url:
        raise CohortError(
            'canonical documentation URL does not match the selected path'
        )
    if runtime_ref is not None and documentation_path is None:
        raise CohortError(
            'a canonical runtime identity requires a selected public path'
        )
    if (
        documentation_provenance is not None
        and documentation_provenance['source_revision']
        != gates['public_revision']
    ):
        raise CohortError(
            'canonical documentation provenance must match the public revision'
        )
    if (
        documentation_path == 'docker-first-map'
        and runtime_ref is not None
        and not runtime_ref.startswith('ghcr.io/')
    ):
        raise CohortError(
            'the Docker path requires an immutable GHCR digest identity'
        )
    if (
        documentation_path == 'source-quickstart'
        and runtime_ref is not None
        and runtime_ref != gates['public_revision']
    ):
        raise CohortError(
            'the source path runtime identity must equal the public revision'
        )
    if gates['public_revision_resolvable'] and (
        gates['public_revision'] is None
    ):
        raise CohortError(
            'a resolvable public revision requires an exact commit'
        )

    ready = _ready_gates(gates)
    expected_status = (
        'COPY_READY_NOT_AUTHORIZED'
        if ready else 'WAITING_FOR_PUBLIC_GATES'
    )
    if contract['status'] != expected_status:
        raise CohortError(
            f'status must be {expected_status} for the current launch gates'
        )

    authority = contract['authority']
    if any(authority.values()):
        raise CohortError('the local cohort packet cannot authorize a write')

    return {
        'schema_version': 1,
        'cohort_id': contract['cohort_id'],
        'status': contract['status'],
        'copy_ready': ready,
        'pending_launch_gates': _pending_launch_gates(gates),
        'accepted_target': capacity['accepted_target'],
        'initial_attempt_cap': capacity['initial_attempt_cap'],
        'hard_attempt_cap': capacity['hard_attempt_cap'],
        'max_concurrent_attempts': capacity['max_concurrent_attempts'],
        'community_posts_authorized': False,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }


def _validate_attempt(
    attempt: dict[str, Any],
    contract: dict[str, Any],
) -> None:
    status = attempt['status']
    is_active = status == 'active'
    completed_at = attempt['completed_at']
    minutes = attempt['active_operator_minutes']
    report_url = attempt['report_url']
    blocker_id = attempt['blocker_id']
    blocker_url = attempt['blocker_url']
    accepted_id = attempt['accepted_validation_id']

    if is_active:
        if any(value is not None for value in (
            completed_at,
            minutes,
            report_url,
            blocker_id,
            blocker_url,
            accepted_id,
        )):
            raise CohortError(
                f"{attempt['id']} active attempt contains terminal evidence"
            )
        gates = contract['launch_gates']
        if not _ready_gates(gates):
            raise CohortError(
                f"{attempt['id']} cannot be active before public gates pass"
            )
        if (
            attempt['documentation_path']
            != gates['canonical_documentation_path']
            or attempt['documentation_page_sha256']
            != gates['canonical_documentation_provenance']['page_sha256']
            or attempt['public_revision'] != gates['public_revision']
            or attempt['runtime_ref'] != gates['canonical_runtime_ref']
        ):
            raise CohortError(
                f"{attempt['id']} active route differs from the canonical path"
            )
        return

    if completed_at is None or minutes is None or report_url is None:
        raise CohortError(
            f"{attempt['id']} terminal attempt lacks completion evidence"
        )
    if _parse_datetime(completed_at) < _parse_datetime(attempt['started_at']):
        raise CohortError(
            f"{attempt['id']} completion precedes its start"
        )

    failure_statuses = {'reported-fail', 'closed-fail', 'abandoned'}
    if status in failure_statuses:
        if blocker_id is None or blocker_url is None:
            raise CohortError(
                f"{attempt['id']} failed attempt lacks a public blocker"
            )
    elif blocker_id is not None or blocker_url is not None:
        raise CohortError(
            f"{attempt['id']} passing attempt cannot claim a blocker"
        )

    if status == 'accepted-pass':
        if accepted_id is None:
            raise CohortError(
                f"{attempt['id']} accepted pass lacks a validation id"
            )
    elif accepted_id is not None:
        raise CohortError(
            f"{attempt['id']} unaccepted attempt has a validation id"
        )

    path = attempt['documentation_path']
    runtime_ref = attempt['runtime_ref']
    if path == 'docker-first-map' and not runtime_ref.startswith('ghcr.io/'):
        raise CohortError(
            f"{attempt['id']} Docker attempt lacks an immutable image digest"
        )
    if (
        path == 'source-quickstart'
        and runtime_ref != attempt['public_revision']
    ):
        raise CohortError(
            f"{attempt['id']} source identity differs from its revision"
        )


def evaluate_state(
    contract: dict[str, Any],
    contract_schema: dict[str, Any],
    state: dict[str, Any],
    state_schema: dict[str, Any],
    accepted_ledger: dict[str, Any],
    accepted_schema: dict[str, Any],
    *,
    now: datetime | None = None,
) -> dict[str, Any]:
    """Validate cohort operations and derive the only safe next state."""
    launch = validate_contract(contract, contract_schema)
    _validate_schema(state, state_schema, 'state')
    if state['cohort_id'] != contract['cohort_id']:
        raise CohortError('state cohort_id differs from the launch contract')
    if any(state['authority'].values()):
        raise CohortError('the cohort state cannot authorize a write')

    try:
        accepted = validate_ledger_payload(accepted_ledger, accepted_schema)
    except LedgerError as exc:
        raise CohortError(f'accepted validation ledger is invalid: {exc}') from exc

    attempts = state['attempts']
    capacity = contract['capacity']
    if state['phase'] == 'initial':
        if state['extension_decision_url'] is not None:
            raise CohortError('initial phase cannot claim an extension decision')
        if len(attempts) > capacity['initial_attempt_cap']:
            raise CohortError('initial phase exceeds its five-attempt cap')
    else:
        if state['extension_decision_url'] is None:
            raise CohortError('extended phase requires a public decision URL')
        if len(attempts) < capacity['initial_attempt_cap']:
            raise CohortError('extended phase cannot begin before initial review')

    ids = [item['id'] for item in attempts]
    if len(ids) != len(set(ids)):
        raise CohortError('cohort attempt ids must be unique')
    expected_ids = [
        f'attempt-{number:03d}' for number in range(1, len(attempts) + 1)
    ]
    if ids != expected_ids:
        raise CohortError('cohort attempt ids must be consecutive and ordered')
    report_urls = [
        item['report_url'] for item in attempts
        if item['report_url'] is not None
    ]
    if len(report_urls) != len(set(report_urls)):
        raise CohortError('public report URLs must be unique')
    start_times = [_parse_datetime(item['started_at']) for item in attempts]
    if start_times != sorted(start_times):
        raise CohortError('attempts must be ordered by started_at')
    for attempt in attempts:
        _validate_attempt(attempt, contract)

    accepted_ids = [
        item['accepted_validation_id']
        for item in attempts
        if item['accepted_validation_id'] is not None
    ]
    if len(accepted_ids) != len(set(accepted_ids)):
        raise CohortError('accepted validation ids must be unique')
    missing_accepted_ids = sorted(
        set(accepted_ids) - set(accepted['validation_ids'])
    )
    if missing_accepted_ids:
        raise CohortError(
            'cohort accepted ids are absent from the authoritative ledger: '
            + ', '.join(missing_accepted_ids)
        )
    accepted_by_id = {
        item['id']: item for item in accepted_ledger['validations']
    }
    for attempt in attempts:
        accepted_id = attempt['accepted_validation_id']
        if accepted_id is None:
            continue
        entry = accepted_by_id[accepted_id]
        if (
            attempt['report_url'] != entry['issue_url']
            or attempt['documentation_path'] != entry['documentation_path']
            or attempt['runtime_ref'] != entry['release_ref']
        ):
            raise CohortError(
                f"{attempt['id']} differs from accepted ledger evidence"
            )

    signals = state['operational_signals']
    signal_values = [
        signals['public_source_or_documentation_drift'],
        signals['supported_p0_open'],
        signals['failed_release_gate'],
        signals['privacy_or_safety_incident'],
    ]
    signals_available = all(value is not None for value in signal_values)
    signals_empty = all(value is None for value in signal_values)
    audit_available = (
        signals['observed_at'] is not None
        and signals['audit_url'] is not None
    )
    if not (
        (signals_available and audit_available)
        or (signals_empty and not audit_available)
    ):
        raise CohortError(
            'operational signals must be wholly observed or wholly unset'
        )
    current_time = (now or datetime.now(timezone.utc)).astimezone(timezone.utc)
    signals_fresh = False
    if signals_available:
        observed_at = _parse_datetime(signals['observed_at'])
        if observed_at > current_time + timedelta(minutes=5):
            raise CohortError('operational signals cannot be future-dated')
        max_age = timedelta(
            hours=contract['service_levels'][
                'operational_signal_max_age_hours'
            ]
        )
        signals_fresh = current_time - observed_at <= max_age

    active_count = sum(item['status'] == 'active' for item in attempts)
    if active_count > capacity['max_concurrent_attempts']:
        raise CohortError('active attempts exceed the two-attempt WIP limit')
    unreviewed_count = sum(
        item['status'] in {'reported-pass', 'reported-fail'}
        for item in attempts
    )
    review_wip_count = active_count + unreviewed_count
    if review_wip_count > capacity['max_concurrent_attempts']:
        raise CohortError(
            'active attempts and unreviewed reports exceed combined WIP'
        )
    terminal = [item for item in attempts if item['status'] != 'active']
    successful = sum(
        item['status'] in {'reported-pass', 'accepted-pass', 'closed-pass'}
        for item in attempts
    )
    completion_rate = (
        successful / len(attempts) if attempts else None
    )
    median_minutes = (
        float(median(item['active_operator_minutes'] for item in terminal))
        if terminal else None
    )

    blocker_counts: dict[str, int] = {}
    for item in attempts:
        blocker_id = item['blocker_id']
        if blocker_id is not None:
            blocker_counts[blocker_id] = blocker_counts.get(blocker_id, 0) + 1
    repeated_blockers = sorted(
        blocker for blocker, count in blocker_counts.items() if count >= 2
    )

    stop_conditions: list[str] = []
    signal_stop_map = {
        'public_source_or_documentation_drift': (
            'public-source-or-documentation-drift'
        ),
        'supported_p0_open': 'supported-p0-open',
        'failed_release_gate': 'failed-release-gate',
        'privacy_or_safety_incident': 'privacy-or-safety-incident',
    }
    if signals_available:
        stop_conditions.extend(
            stop_id for key, stop_id in signal_stop_map.items()
            if signals[key]
        )
    if unreviewed_count >= 2:
        stop_conditions.append('two-unreviewed-receipts')
    if repeated_blockers:
        stop_conditions.append('two-attempts-share-one-blocker')
    at_assessment = len(attempts) == capacity['assessment_attempt_count']
    assessment_complete = at_assessment and len(terminal) == len(attempts)
    if assessment_complete and completion_rate is not None:
        if completion_rate < capacity['minimum_completion_rate']:
            stop_conditions.append(
                'completion-below-80-percent-at-attempt-10'
            )
        if (
            median_minutes is not None
            and median_minutes
            > capacity['maximum_median_active_operator_minutes']
        ):
            stop_conditions.append(
                'median-active-time-above-10-minutes-at-attempt-10'
            )
    stop_conditions.sort()

    accepted_count = len(accepted_ids)
    if not launch['copy_ready']:
        status = 'WAITING_FOR_PUBLIC_GATES'
    elif not signals_available or not signals_fresh:
        status = 'WAITING_FOR_OPERATIONAL_SIGNALS'
    elif stop_conditions:
        status = 'PAUSED_REPAIR'
    elif accepted_count >= capacity['accepted_target']:
        status = 'TARGET_MET'
    elif len(attempts) >= capacity['hard_attempt_cap']:
        status = 'HARD_CAP_REVIEW'
    elif (
        state['phase'] == 'initial'
        and len(attempts) >= capacity['initial_attempt_cap']
    ):
        status = 'INITIAL_BATCH_REVIEW'
    elif review_wip_count >= capacity['max_concurrent_attempts']:
        status = 'CAPACITY_FULL'
    else:
        status = 'READY_FOR_NEXT_ATTEMPT'

    return {
        'schema_version': 1,
        'cohort_id': contract['cohort_id'],
        'status': status,
        'launch_status': launch['status'],
        'copy_ready': launch['copy_ready'],
        'pending_launch_gates': launch['pending_launch_gates'],
        'phase': state['phase'],
        'attempt_count': len(attempts),
        'terminal_attempt_count': len(terminal),
        'active_attempt_count': active_count,
        'unreviewed_report_count': unreviewed_count,
        'review_wip_count': review_wip_count,
        'operational_signals_fresh': signals_fresh,
        'successful_first_map_count': successful,
        'accepted_validations': accepted_count,
        'accepted_target': capacity['accepted_target'],
        'completion_rate': completion_rate,
        'median_active_operator_minutes': median_minutes,
        'repeated_blocker_ids': repeated_blockers,
        'stop_conditions': stop_conditions,
        'next_attempt_permitted_by_state': (
            status == 'READY_FOR_NEXT_ATTEMPT'
        ),
        'community_posts_authorized': False,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }


def render_state_summary(report: dict[str, Any]) -> str:
    """Render a privacy-safe operator summary with one explicit next action."""
    status = report['status']
    if status not in NEXT_ACTIONS:
        raise CohortError(f'cannot render unsupported cohort status: {status}')
    completion_rate = report['completion_rate']
    completion_text = (
        'not measured'
        if completion_rate is None
        else f'{completion_rate * 100:.1f}%'
    )
    median_minutes = report['median_active_operator_minutes']
    median_text = (
        'not measured'
        if median_minutes is None
        else f'{median_minutes:g} minutes'
    )
    stop_conditions = report['stop_conditions']
    stop_text = ', '.join(stop_conditions) if stop_conditions else 'none'
    pending_gates = report['pending_launch_gates']
    pending_text = ', '.join(pending_gates) if pending_gates else 'none'
    yes_no = {True: 'yes', False: 'no'}
    next_action = NEXT_ACTIONS[status]
    if status == 'WAITING_FOR_PUBLIC_GATES':
        pending = set(pending_gates)
        if pending & {'public_revision', 'public_revision_resolvable'}:
            next_action = (
                'Publish one reviewed exact revision and verify that it is '
                'publicly resolvable before provisioning a trial host.'
            )
        elif pending & {'comparable_docker_row', 'comparable_source_row'}:
            next_action = (
                'Run the missing clean comparable Docker/source matrix rows; '
                'do not recruit while either route lacks measured evidence.'
            )
        elif pending & {
            'canonical_documentation_path',
            'canonical_documentation_url',
            'canonical_documentation_provenance',
            'canonical_runtime_ref',
        }:
            next_action = (
                'Select the lower-burden comparable PASS as the canonical '
                'path and bind its verified public page bytes and immutable '
                'runtime identity.'
            )
        elif 'copy_ready_handoff_public' in pending:
            next_action = (
                'Publish and verify the copy-ready first-map handoff at the '
                'selected immutable product identity.'
            )
    return '\n'.join([
        '# Independent first-map cohort',
        '',
        f'- Status: **{status}**',
        (
            '- Accepted: '
            f"**{report['accepted_validations']} / "
            f"{report['accepted_target']}**"
        ),
        (
            '- Attempts: '
            f"{report['attempt_count']} total; "
            f"{report['active_attempt_count']} active; "
            f"{report['review_wip_count']} review WIP"
        ),
        f'- Completion rate: {completion_text}',
        f'- Median active operator time: {median_text}',
        (
            '- Operational signals fresh: '
            f"{yes_no[report['operational_signals_fresh']]}"
        ),
        f'- Stop conditions: {stop_text}',
        f'- Pending launch gates: {pending_text}',
        (
            '- Next attempt permitted by state: '
            f"{yes_no[report['next_attempt_permitted_by_state']]}"
        ),
        '- GitHub/community write authorized: no',
        '',
        f'Next action: {next_action}',
    ]) + '\n'


def render_recruitment(contract: dict[str, Any]) -> str:
    """Render copy-ready public text only after every product gate passes."""
    if not _ready_gates(contract['launch_gates']):
        raise CohortError(
            'recruitment text is blocked until the public revision, '
            'comparable Docker/source rows, canonical documentation '
            'provenance/runtime identity, and public copy-ready handoff all '
            'pass'
        )
    gates = contract['launch_gates']
    capacity = contract['capacity']
    service = contract['service_levels']
    return '\n'.join([
        'Help validate the public lidarslam_ros2 first-map path',
        '',
        'We are inviting a small first cohort of independent ROS 2 users to '
        'run one public first-map path without private maintainer guidance.',
        '',
        f"Public path: {gates['canonical_documentation_url']}",
        (
            'Public page SHA-256: '
            f"{gates['canonical_documentation_provenance']['page_sha256']}"
        ),
        f"Exact source revision: {gates['public_revision']}",
        f"Exact product identity: {gates['canonical_runtime_ref']}",
        f"Tracking issue: {gates['tracking_issue_url']}",
        '',
        'You are eligible if you are not a lidarslam_ros2 maintainer and can '
        'follow only the public documentation. Please report both PASS and '
        'FAIL outcomes; a failure is useful product evidence.',
        '',
        'After the attempt, run `lidarslam-map report '
        '<session-or-demo-output>` '
        'and use its copy-ready fields. Attach only the reviewed '
        '`first_map_validation_receipt.json` to this form:',
        ISSUE_FORM_URL,
        '',
        'Do not upload a bag, map, trajectory, raw log, parameters, precise '
        'location, or private-place screenshot. No product telemetry is used.',
        '',
        f"We will begin with at most {capacity['initial_attempt_cap']} attempts "
        f"and no more than {capacity['max_concurrent_attempts']} at once. "
        'We aim to acknowledge a report within '
        f"{service['acknowledgement_business_days']} business days and finish "
        'evidence review within '
        f"{service['evidence_review_business_days']} business days.",
        '',
        'Posting this text still requires explicit community-write approval.',
    ])


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the read-only launch and operating-state interface."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract', type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument('--schema', type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument('--state', type=Path, default=DEFAULT_STATE)
    parser.add_argument('--state-schema', type=Path, default=DEFAULT_STATE_SCHEMA)
    parser.add_argument(
        '--accepted-ledger',
        type=Path,
        default=DEFAULT_ACCEPTED_LEDGER,
    )
    parser.add_argument(
        '--accepted-schema',
        type=Path,
        default=DEFAULT_ACCEPTED_SCHEMA,
    )
    parser.add_argument('--json', action='store_true')
    parser.add_argument(
        '--render',
        action='store_true',
        help='Render recruitment text only when every public launch gate passes.',
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Validate the cohort state or render bounded recruitment text."""
    args = parse_args(argv)
    try:
        contract = _load_object(args.contract, 'contract')
        schema = _load_object(args.schema, 'schema')
        state = _load_object(args.state, 'state')
        state_schema = _load_object(args.state_schema, 'state schema')
        accepted_ledger = _load_object(args.accepted_ledger, 'accepted ledger')
        accepted_schema = _load_object(args.accepted_schema, 'accepted schema')
        report = evaluate_state(
            contract,
            schema,
            state,
            state_schema,
            accepted_ledger,
            accepted_schema,
        )
        if args.render:
            if report['status'] != 'READY_FOR_NEXT_ATTEMPT':
                raise CohortError(
                    'recruitment text is blocked by cohort operational state: '
                    f"{report['status']}"
                )
            print(render_recruitment(copy.deepcopy(contract)))
            return 0
    except CohortError as exc:
        if args.json:
            print(json.dumps({
                'status': 'COHORT_INVALID_OR_BLOCKED',
                'error': str(exc),
                'remote_mutations_performed': False,
            }, sort_keys=True))
        else:
            print(f'first-map cohort blocked: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(render_state_summary(report), end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
