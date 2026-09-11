#!/usr/bin/env python3
"""Run one exact candidate onboarding row from a prepared handoff.

The command validates the complete handoff, performs the row-specific public
preflight, bootstraps a content-bound Docker observer image when required,
delegates to the existing machine probe, and publishes one new local evidence
directory atomically. It never invents human measurements or performs a
GitHub, registry, release, issue, or community write.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Sequence

from audit_candidate_image_set import audit_candidate_bundle

from check_onboarding_trial import (
    TrialError,
    evaluate_trial,
    read_validation_receipt,
)

from prepare_candidate_trial import (
    CandidateTrialPreparationError,
    PREPARATION_FILENAME,
    load_candidate_trial_handoff,
)

from product_schema import validate_contract


REPO_ROOT = Path(__file__).resolve().parents[1]
EXECUTION_SCHEMA = 'candidate-trial-execution-v1.schema.json'
EXECUTION_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/candidate-trial-execution-v1.schema.json'
)
REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
SOURCE_REPOSITORY_URL = 'https://github.com/rsasaki0109/lidar_slam_ros2.git'
ROW_IDS = (
    'docker-humble',
    'docker-jazzy',
    'source-humble',
    'source-jazzy',
)
DOCKER_OS_VERSION = {'humble': '22.04', 'jazzy': '24.04'}
OBSERVER_DOCKERFILE = (
    REPO_ROOT / 'docker' / 'onboarding-trial-host.Dockerfile'
)
PREFLIGHT_FILENAME = 'row-preflight.json'
TRIAL_RECORD_FILENAME = 'trial-record.json'
TRIAL_AUDIT_FILENAME = 'trial-audit.json'
VALIDATION_RECEIPT_FILENAME = 'first-map-validation-receipt.json'
PRIVATE_DIRECTORY = 'private'
EXECUTION_FILENAME = 'execution.json'
MAX_RECORD_BYTES = 1024 * 1024
TRIAL_ID_RE = re.compile(r'^[a-z0-9][a-z0-9._-]{2,79}$')
INTERFACE_RE = re.compile(r'^[A-Za-z0-9_.-]+$')
Runner = Callable[..., subprocess.CompletedProcess[str]]


class CandidateTrialExecutionError(ValueError):
    """The candidate row cannot be run without guessing or unsafe reuse."""

    exit_code = 2


def _utc_now() -> str:
    return (
        datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace('+00:00', 'Z')
    )


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    try:
        return _sha256_bytes(path.read_bytes())
    except OSError as exc:
        raise CandidateTrialExecutionError(
            f'cannot hash retained output {path.name}: {exc}'
        ) from exc


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    with path.open('x', encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write('\n')


def _contains(parent: Path, child: Path) -> bool:
    try:
        child.relative_to(parent)
    except ValueError:
        return False
    return True


def _validate_paths(
    handoff_dir: Path,
    output_dir: Path,
    disk_scope: Path,
) -> tuple[Path, Path, Path, Path]:
    """Resolve all operator paths before any network or trial activity."""
    try:
        if handoff_dir.is_symlink() or not handoff_dir.is_dir():
            raise CandidateTrialExecutionError(
                'handoff directory must be one existing real directory'
            )
        handoff = handoff_dir.resolve(strict=True)
    except OSError as exc:
        raise CandidateTrialExecutionError(
            f'handoff directory cannot be resolved: {exc}'
        ) from exc
    if os.path.lexists(output_dir):
        raise CandidateTrialExecutionError(
            f'refusing to overwrite trial output: {output_dir}'
        )
    parent = output_dir.parent
    try:
        if parent.is_symlink() or not parent.is_dir():
            raise CandidateTrialExecutionError(
                'output parent must be one existing non-symlink directory'
            )
        output_parent = parent.resolve(strict=True)
    except OSError as exc:
        raise CandidateTrialExecutionError(
            f'output parent cannot be resolved: {exc}'
        ) from exc
    output = output_parent / output_dir.name
    try:
        if disk_scope.is_symlink() or not disk_scope.is_dir():
            raise CandidateTrialExecutionError(
                'disk scope must be one existing real directory'
            )
        scope = disk_scope.resolve(strict=True)
    except OSError as exc:
        raise CandidateTrialExecutionError(
            f'disk scope cannot be resolved: {exc}'
        ) from exc

    checkout = REPO_ROOT.resolve()
    if _contains(checkout, handoff):
        raise CandidateTrialExecutionError(
            'candidate handoff must be outside the product checkout'
        )
    if _contains(checkout, output):
        raise CandidateTrialExecutionError(
            'trial output must be outside the product checkout'
        )
    if _contains(handoff, output) or _contains(output, handoff):
        raise CandidateTrialExecutionError(
            'candidate handoff and trial output must not overlap'
        )
    return handoff, output, output_parent, scope


def _measurement_capture(
    requested: str,
    *,
    interactive: bool,
) -> dict[str, str]:
    if requested == 'prompt':
        if not interactive:
            raise CandidateTrialExecutionError(
                '--human-measurements prompt requires an interactive terminal'
            )
        return {
            'requested_mode': requested,
            'resolved_mode': 'prompt',
            'resolution': 'explicit',
        }
    if requested == 'unknown':
        return {
            'requested_mode': requested,
            'resolved_mode': 'unknown',
            'resolution': 'explicit',
        }
    return {
        'requested_mode': 'auto',
        'resolved_mode': 'prompt' if interactive else 'unknown',
        'resolution': (
            'interactive-terminal' if interactive else 'non-interactive'
        ),
    }


def _default_trial_id(row_id: str, started_at: str) -> str:
    try:
        timestamp = datetime.fromisoformat(started_at.replace('Z', '+00:00'))
    except ValueError as exc:
        raise CandidateTrialExecutionError(
            'started_at must be an ISO 8601 timestamp'
        ) from exc
    return f'g0-{row_id}-{timestamp:%Y%m%dt%H%M%Sz}'


def _select_row(packet: dict[str, Any], row_id: str) -> dict[str, Any]:
    rows = [row for row in packet['rows'] if row['row_id'] == row_id]
    if len(rows) != 1:
        raise CandidateTrialExecutionError(
            f'observer packet does not contain row {row_id!r} exactly once'
        )
    return rows[0]


def _run_read(
    command: list[str],
    *,
    runner: Runner,
) -> subprocess.CompletedProcess[str]:
    try:
        return runner(
            command,
            capture_output=True,
            text=True,
            check=False,
            timeout=120,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise CandidateTrialExecutionError(
            f'preflight command could not run: {exc}'
        ) from exc


def _source_preflight(
    row: dict[str, Any],
    *,
    runner: Runner,
) -> dict[str, Any]:
    command = [
        sys.executable,
        str(REPO_ROOT / 'scripts' / 'run_source_onboarding_probe.py'),
        '--public-preflight',
        '--source-commit',
        row['identity']['value'],
        '--product-version',
        row['product_version'],
    ]
    result = _run_read(command, runner=runner)
    try:
        report = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise CandidateTrialExecutionError(
            'source public preflight did not return JSON'
        ) from exc
    if not isinstance(report, dict):
        raise CandidateTrialExecutionError(
            'source public preflight JSON root is not an object'
        )
    required = {
        'schema_version',
        'status',
        'repository',
        'source_commit',
        'product_version',
        'network_requested',
        'writes_performed',
        'finding_codes',
    }
    if not required.issubset(report):
        raise CandidateTrialExecutionError(
            'source public preflight report is incomplete'
        )
    status = report['status']
    expected_exit = {'READY': 0, 'NOT_READY': 1}.get(status)
    if expected_exit is None or result.returncode != expected_exit:
        raise CandidateTrialExecutionError(
            'source public preflight status and exit code disagree'
        )
    identity_ok = (
        report['schema_version'] == 1
        and report['repository'] == SOURCE_REPOSITORY_URL
        and report['source_commit'] == row['identity']['value']
        and report['product_version'] == row['product_version']
        and report['network_requested'] is True
        and report['writes_performed'] is False
        and isinstance(report['finding_codes'], list)
        and all(
            isinstance(item, str) and TRIAL_ID_RE.fullmatch(item)
            for item in report['finding_codes']
        )
    )
    if not identity_ok:
        raise CandidateTrialExecutionError(
            'source public preflight identity is inconsistent'
        )
    if status == 'READY' and report['finding_codes']:
        raise CandidateTrialExecutionError(
            'READY source preflight contains finding codes'
        )
    if status == 'NOT_READY' and not report['finding_codes']:
        raise CandidateTrialExecutionError(
            'NOT_READY source preflight has no finding code'
        )
    return report


def _run_preflight(
    handoff: dict[str, Any],
    row: dict[str, Any],
    *,
    runner: Runner,
) -> tuple[dict[str, Any], str, str, list[str]]:
    if row['route'] == 'docker':
        report = audit_candidate_bundle(
            handoff['evidence_bundle'],
            remote=True,
            runner=runner,
        )
        status = report['status']
        if status not in {'REMOTE_AUDIT_PASS', 'REMOTE_AUDIT_FAIL'}:
            raise CandidateTrialExecutionError(
                f'unexpected candidate remote audit status: {status}'
            )
        return (
            report,
            'candidate-remote-audit',
            status,
            report['findings'],
        )
    report = _source_preflight(row, runner=runner)
    return (
        report,
        'source-public',
        report['status'],
        report['finding_codes'],
    )


def _measurement_arguments(capture: dict[str, str]) -> list[str]:
    if capture['resolved_mode'] == 'prompt':
        return ['--prompt-human-measurements']
    return ['--record-human-measurements-unknown']


def _docker_observer_identity(row: dict[str, Any]) -> tuple[str, str]:
    """Bind automatic observer bootstrap to the reviewed Dockerfile bytes."""
    try:
        if (
            OBSERVER_DOCKERFILE.is_symlink()
            or not OBSERVER_DOCKERFILE.is_file()
        ):
            raise CandidateTrialExecutionError(
                'observer Dockerfile must be one regular non-symlink file'
            )
        recipe_sha256 = _sha256_bytes(OBSERVER_DOCKERFILE.read_bytes())
    except OSError as exc:
        raise CandidateTrialExecutionError(
            f'cannot inspect observer Dockerfile: {exc}'
        ) from exc
    os_version = DOCKER_OS_VERSION[row['ros_distro']]
    image = (
        f'lidarslam-onboarding-trial-host:{os_version}-'
        f'{recipe_sha256[:12]}'
    )
    return image, recipe_sha256


def _docker_probe_command(
    handoff: dict[str, Any],
    row: dict[str, Any],
    *,
    trial_id: str,
    record: Path,
    private: Path,
    disk_scope: Path,
    timeout_sec: float,
    capture: dict[str, str],
) -> list[str]:
    preparation = handoff['preparation']
    observer_image, observer_recipe_sha256 = _docker_observer_identity(row)
    return [
        sys.executable,
        str(REPO_ROOT / 'scripts' / 'run_docker_onboarding_probe.py'),
        '--trial-id',
        trial_id,
        '--ros-distro',
        row['ros_distro'],
        '--candidate-image-ref',
        row['identity']['immutable_ref'],
        '--image-digest',
        row['identity']['value'],
        '--product-version',
        row['product_version'],
        '--candidate-image-set-sha256',
        preparation['candidate_set_sha256'],
        '--candidate-evidence-bundle-sha256',
        preparation['candidate_bundle_sha256'],
        '--candidate-source-pr',
        str(preparation['source_pr']),
        '--candidate-source-commit',
        preparation['source_commit'],
        '--candidate-workflow-run-url',
        preparation['workflow_run_url'],
        '--record',
        str(record),
        '--validation-receipt-output',
        str(record.parent / VALIDATION_RECEIPT_FILENAME),
        '--temp-parent',
        str(private),
        '--disk-scope',
        str(disk_scope),
        '--timeout-sec',
        str(timeout_sec),
        '--acknowledge-dedicated-filesystem',
        '--allow-privileged-container-host',
        '--build-observer-image-if-missing',
        '--observer-image',
        observer_image,
        '--observer-recipe-sha256',
        observer_recipe_sha256,
        *_measurement_arguments(capture),
    ]


def _source_probe_command(
    handoff: dict[str, Any],
    row: dict[str, Any],
    *,
    trial_id: str,
    record: Path,
    private: Path,
    disk_scope: Path,
    timeout_sec: float,
    capture: dict[str, str],
    network_interface: str | None,
) -> list[str]:
    trial_root = private / 'trial'
    observer_parent = private / 'observer'
    trial_root.mkdir()
    observer_parent.mkdir()
    command = [
        sys.executable,
        str(REPO_ROOT / 'scripts' / 'run_source_onboarding_probe.py'),
        '--trial-id',
        trial_id,
        '--ros-distro',
        row['ros_distro'],
        '--source-commit',
        handoff['preparation']['source_commit'],
        '--product-version',
        row['product_version'],
        '--trial-root',
        str(trial_root),
        '--observer-parent',
        str(observer_parent),
        '--disk-scope',
        str(disk_scope),
        '--record',
        str(record),
        '--validation-receipt-output',
        str(record.parent / VALIDATION_RECEIPT_FILENAME),
        '--timeout-sec',
        str(timeout_sec),
        '--acknowledge-disposable-host',
        '--acknowledge-isolated-network',
        *_measurement_arguments(capture),
    ]
    if network_interface is not None:
        command.extend(['--network-interface', network_interface])
    return command


def _run_probe(
    command: list[str],
    *,
    runner: Runner,
) -> subprocess.CompletedProcess[str]:
    try:
        # Keep stdout reserved for this wrapper's optional JSON receipt while
        # preserving live product/probe output on the operator's terminal.
        return runner(command, check=False, stdout=sys.stderr)
    except (OSError, subprocess.SubprocessError) as exc:
        print(f'candidate trial probe error: {exc}', file=sys.stderr)
        return subprocess.CompletedProcess(command, 125)


def _read_trial_record(path: Path) -> tuple[dict[str, Any], bytes]:
    try:
        if path.is_symlink() or not path.is_file():
            raise CandidateTrialExecutionError(
                'probe did not create one regular bounded trial record'
            )
        size = path.stat().st_size
        if size < 1 or size > MAX_RECORD_BYTES:
            raise CandidateTrialExecutionError(
                f'probe trial record must be 1..{MAX_RECORD_BYTES} bytes'
            )
        payload = path.read_bytes()
        record = json.loads(payload.decode('utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CandidateTrialExecutionError(
            f'probe trial record is not readable JSON: {exc}'
        ) from exc
    if not isinstance(record, dict):
        raise CandidateTrialExecutionError(
            'probe trial record JSON root is not an object'
        )
    return record, payload


def _validate_trial_identity(
    record: dict[str, Any],
    handoff: dict[str, Any],
    row: dict[str, Any],
    trial_id: str,
) -> None:
    expected_documentation = (
        'docker-first-map' if row['route'] == 'docker'
        else 'source-quickstart'
    )
    environment = record['environment']
    expected_identity = {
        'kind': row['identity']['kind'],
        'value': row['identity']['value'],
    }
    checks = (
        (record['trial_id'] == trial_id, 'trial-id-mismatch'),
        (
            record['operator_class'] == 'maintainer',
            'operator-class-mismatch',
        ),
        (
            record['documentation_path'] == expected_documentation,
            'route-mismatch',
        ),
        (environment['ros_distro'] == row['ros_distro'], 'distro-mismatch'),
        (environment['os_family'] == row['os_family'], 'os-mismatch'),
        (environment['architecture'] == 'x86_64', 'architecture-mismatch'),
        (
            environment['product_version'] == row['product_version'],
            'product-version-mismatch',
        ),
        (environment['revision'] == expected_identity, 'identity-mismatch'),
        (
            record['input']['dataset_class'] == 'fixed-public',
            'dataset-class-mismatch',
        ),
        (
            record['input']['dataset_id']
            == 'mid360-public-zenodo-14841855',
            'dataset-mismatch',
        ),
    )
    failed = [finding for passed, finding in checks if not passed]
    if row['route'] == 'docker':
        preparation = handoff['preparation']
        expected_candidate = {
            'sha256': preparation['candidate_set_sha256'],
            'bundle_sha256': preparation['candidate_bundle_sha256'],
            'source_pr': preparation['source_pr'],
            'source_commit': preparation['source_commit'],
            'product_version': preparation['product_version'],
            'workflow_run_url': preparation['workflow_run_url'],
            'immutable_ref': row['identity']['immutable_ref'],
        }
        if record['evidence'].get('candidate_image_set') != expected_candidate:
            failed.append('candidate-evidence-mismatch')
    elif 'candidate_image_set' in record['evidence']:
        failed.append('unexpected-candidate-evidence')
    if failed:
        raise CandidateTrialExecutionError(
            'probe trial identity is inconsistent: ' + ', '.join(failed)
        )


def _trial_state(
    *,
    attempted: bool,
    probe_exit_code: int | None,
    record_status: str,
    record_sha256: str | None = None,
    report: dict[str, Any] | None = None,
    finding_codes: list[str] | None = None,
) -> dict[str, Any]:
    return {
        'attempted': attempted,
        'probe_exit_code': probe_exit_code,
        'record_status': record_status,
        'record_sha256': record_sha256,
        'outcome_status': None if report is None else report['outcome_status'],
        'measurement_status': (
            None if report is None else report['measurement_status']
        ),
        'comparable': None if report is None else report['comparable'],
        'missing_measurements': (
            [] if report is None else report['missing_measurements']
        ),
        'comparability_blockers': (
            [] if report is None else report['comparability_blockers']
        ),
        'finding_codes': sorted(set(finding_codes or [])),
    }


def _row_receipt(row: dict[str, Any]) -> dict[str, Any]:
    return {
        'row_id': row['row_id'],
        'route': row['route'],
        'ros_distro': row['ros_distro'],
        'os_family': row['os_family'],
        'product_version': row['product_version'],
        'identity': {
            'kind': row['identity']['kind'],
            'value': row['identity']['value'],
            'immutable_ref': row['identity']['immutable_ref'],
        },
    }


def _build_execution(
    *,
    status: str,
    started_at: str,
    completed_at: str,
    trial_id: str,
    handoff: dict[str, Any],
    row: dict[str, Any],
    capture: dict[str, str],
    preflight_kind: str,
    preflight_status: str,
    preflight_sha256: str,
    preflight_findings: list[str],
    trial: dict[str, Any],
    validation_receipt_available: bool = False,
) -> dict[str, Any]:
    preparation = handoff['preparation']
    trial_available = trial['record_status'] == 'AVAILABLE'
    audit_available = trial_available and trial['outcome_status'] is not None
    private_available = trial['attempted']
    bounded_outputs = [PREFLIGHT_FILENAME]
    if trial_available:
        bounded_outputs.append(TRIAL_RECORD_FILENAME)
    if audit_available:
        bounded_outputs.append(TRIAL_AUDIT_FILENAME)
    if validation_receipt_available:
        bounded_outputs.append(VALIDATION_RECEIPT_FILENAME)
    bounded_outputs.append(EXECUTION_FILENAME)
    receipt = {
        'schema_version': 1,
        'schema_uri': EXECUTION_SCHEMA_URI,
        'status': status,
        'started_at': started_at,
        'completed_at': completed_at,
        'repository': REPOSITORY,
        'trial_id': trial_id,
        'handoff': {
            'preparation_sha256': _sha256_bytes(
                handoff['metadata_bytes'][PREPARATION_FILENAME]
            ),
            'workflow_run_url': preparation['workflow_run_url'],
            'run_id': preparation['run_id'],
            'source_pr': preparation['source_pr'],
            'source_commit': preparation['source_commit'],
            'product_version': preparation['product_version'],
            'candidate_bundle_sha256': (
                preparation['candidate_bundle_sha256']
            ),
            'candidate_set_sha256': preparation['candidate_set_sha256'],
            'artifact_expires_at': preparation['artifact_expires_at'],
        },
        'row': _row_receipt(row),
        'measurement_capture': capture,
        'preflight': {
            'kind': preflight_kind,
            'status': preflight_status,
            'report_sha256': preflight_sha256,
            'finding_codes': sorted(set(preflight_findings)),
        },
        'trial': trial,
        'outputs': {
            'preflight_json': PREFLIGHT_FILENAME,
            'trial_record': (
                TRIAL_RECORD_FILENAME if trial_available else None
            ),
            'trial_audit': TRIAL_AUDIT_FILENAME if audit_available else None,
            'validation_receipt': (
                VALIDATION_RECEIPT_FILENAME
                if validation_receipt_available else None
            ),
            'private_evidence_directory': (
                PRIVATE_DIRECTORY if private_available else None
            ),
            'execution_json': EXECUTION_FILENAME,
        },
        'sharing': {
            'bounded_outputs': bounded_outputs,
            'private_evidence_shareable': False,
            'review_before_sharing': True,
        },
        'authority': {
            'dedicated_trial_host_acknowledged': True,
            'network_reads_performed': True,
            'local_files_written': True,
            'atomic_output_publication': True,
            'trial_executed': trial['attempted'],
            'privileged_container_authorized': (
                trial['attempted'] and row['route'] == 'docker'
            ),
            'source_host_mutation_authorized': (
                trial['attempted'] and row['route'] == 'source'
            ),
            'github_writes_authorized': False,
            'registry_writes_authorized': False,
            'release_writes_authorized': False,
            'community_writes_authorized': False,
            'remote_mutations_performed': False,
        },
    }
    validate_contract(receipt, EXECUTION_SCHEMA)
    return receipt


def _quarantine_record(record: Path, private: Path) -> str:
    """Move an untrusted record under private evidence when possible."""
    if record.is_symlink():
        record.unlink(missing_ok=True)
        return 'NOT_CREATED'
    if record.is_file():
        os.replace(record, private / 'untrusted-trial-record.json')
        return 'QUARANTINED'
    return 'NOT_CREATED'


def _quarantine_validation_receipt(receipt: Path, private: Path) -> None:
    if receipt.is_symlink():
        receipt.unlink(missing_ok=True)
    elif os.path.lexists(receipt):
        os.replace(
            receipt,
            private / 'untrusted-first-map-validation-receipt',
        )


def _publish_execution(
    staging: Path,
    output: Path,
    receipt: dict[str, Any],
) -> None:
    _write_json(staging / EXECUTION_FILENAME, receipt)
    os.replace(staging, output)


def run_candidate_trial(
    handoff_dir: Path,
    row_id: str,
    output_dir: Path,
    *,
    acknowledge_dedicated_trial_host: bool,
    trial_id: str | None = None,
    human_measurements: str = 'auto',
    disk_scope: Path = Path('/'),
    timeout_sec: float = 7200.0,
    network_interface: str | None = None,
    read_runner: Runner = subprocess.run,
    probe_runner: Runner = subprocess.run,
    interactive: bool | None = None,
    started_at: str | None = None,
) -> tuple[dict[str, Any], int]:
    """Run one row and return its immutable execution receipt and exit code."""
    if not acknowledge_dedicated_trial_host:
        raise CandidateTrialExecutionError(
            '--acknowledge-dedicated-trial-host is required; source rows '
            'modify the disposable host and Docker rows use a privileged '
            'container plus a dedicated filesystem measurement'
        )
    if row_id not in ROW_IDS:
        raise CandidateTrialExecutionError(
            'row must be one of: ' + ', '.join(ROW_IDS)
        )
    if human_measurements not in {'auto', 'prompt', 'unknown'}:
        raise CandidateTrialExecutionError(
            'human measurement mode must be auto, prompt, or unknown'
        )
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise CandidateTrialExecutionError(
            'timeout must be greater than zero'
        )
    handoff_path, output, parent, scope = _validate_paths(
        handoff_dir, output_dir, disk_scope
    )
    capture = _measurement_capture(
        human_measurements,
        interactive=sys.stdin.isatty() if interactive is None else interactive,
    )
    start = started_at or _utc_now()
    resolved_trial_id = trial_id or _default_trial_id(row_id, start)
    if TRIAL_ID_RE.fullmatch(resolved_trial_id) is None:
        raise CandidateTrialExecutionError(
            'trial ID must be a privacy-bounded 3..80 character lower-case '
            'slug'
        )

    try:
        handoff = load_candidate_trial_handoff(handoff_path)
    except CandidateTrialPreparationError as exc:
        raise CandidateTrialExecutionError(
            f'candidate handoff is invalid: {exc}'
        ) from exc
    row = _select_row(handoff['packet'], row_id)
    if network_interface is not None:
        if row['route'] != 'source':
            raise CandidateTrialExecutionError(
                '--network-interface is available only for a source row'
            )
        if INTERFACE_RE.fullmatch(network_interface) is None:
            raise CandidateTrialExecutionError(
                'network interface name is unsafe'
            )
    staging = Path(tempfile.mkdtemp(
        prefix=f'.{output.name}.running-',
        dir=parent,
    ))
    try:
        preflight_report, preflight_kind, preflight_status, findings = (
            _run_preflight(handoff, row, runner=read_runner)
        )
        _write_json(staging / PREFLIGHT_FILENAME, preflight_report)
        preflight_sha256 = _sha256_file(staging / PREFLIGHT_FILENAME)
        preflight_pass = preflight_status in {
            'REMOTE_AUDIT_PASS', 'READY',
        }
        if not preflight_pass:
            trial = _trial_state(
                attempted=False,
                probe_exit_code=None,
                record_status='NOT_CREATED',
            )
            receipt = _build_execution(
                status='PREFLIGHT_BLOCKED',
                started_at=start,
                completed_at=_utc_now(),
                trial_id=resolved_trial_id,
                handoff=handoff,
                row=row,
                capture=capture,
                preflight_kind=preflight_kind,
                preflight_status=preflight_status,
                preflight_sha256=preflight_sha256,
                preflight_findings=findings,
                trial=trial,
            )
            _publish_execution(staging, output, receipt)
            return receipt, 1

        private = staging / PRIVATE_DIRECTORY
        private.mkdir(mode=0o700)
        record_path = staging / TRIAL_RECORD_FILENAME
        validation_receipt_path = staging / VALIDATION_RECEIPT_FILENAME
        if row['route'] == 'docker':
            command = _docker_probe_command(
                handoff,
                row,
                trial_id=resolved_trial_id,
                record=record_path,
                private=private,
                disk_scope=scope,
                timeout_sec=timeout_sec,
                capture=capture,
            )
        else:
            command = _source_probe_command(
                handoff,
                row,
                trial_id=resolved_trial_id,
                record=record_path,
                private=private,
                disk_scope=scope,
                timeout_sec=timeout_sec,
                capture=capture,
                network_interface=network_interface,
            )
        probe = _run_probe(command, runner=probe_runner)

        record_was_created = os.path.lexists(record_path)
        try:
            record, record_bytes = _read_trial_record(record_path)
            validation_receipt_bytes = (
                read_validation_receipt(validation_receipt_path)
                if os.path.lexists(validation_receipt_path) else None
            )
            report = evaluate_trial(
                record,
                validation_receipt_bytes=validation_receipt_bytes,
                require_evidence_binding=True,
            )
            _validate_trial_identity(
                record, handoff, row, resolved_trial_id
            )
        except (CandidateTrialExecutionError, TrialError, KeyError) as exc:
            print(f'candidate trial harness error: {exc}', file=sys.stderr)
            status = _quarantine_record(record_path, private)
            _quarantine_validation_receipt(
                validation_receipt_path,
                private,
            )
            quarantined = private / 'untrusted-trial-record.json'
            record_sha256 = (
                _sha256_file(quarantined)
                if status == 'QUARANTINED' else None
            )
            trial = _trial_state(
                attempted=True,
                probe_exit_code=probe.returncode,
                record_status=status,
                record_sha256=record_sha256,
                finding_codes=[
                    'probe-record-invalid' if record_was_created
                    else 'probe-record-not-created'
                ],
            )
            receipt = _build_execution(
                status='HARNESS_ERROR',
                started_at=start,
                completed_at=_utc_now(),
                trial_id=resolved_trial_id,
                handoff=handoff,
                row=row,
                capture=capture,
                preflight_kind=preflight_kind,
                preflight_status=preflight_status,
                preflight_sha256=preflight_sha256,
                preflight_findings=findings,
                trial=trial,
            )
            _publish_execution(staging, output, receipt)
            return receipt, 2

        _write_json(staging / TRIAL_AUDIT_FILENAME, report)
        expected_exit = 0 if record['outcome']['status'] == 'PASS' else 1
        exit_matches = probe.returncode == expected_exit
        trial = _trial_state(
            attempted=True,
            probe_exit_code=probe.returncode,
            record_status='AVAILABLE',
            record_sha256=_sha256_bytes(record_bytes),
            report=report,
            finding_codes=(
                [] if exit_matches else ['probe-exit-contract-mismatch']
            ),
        )
        execution_status = (
            'TRIAL_RECORDED' if exit_matches else 'HARNESS_ERROR'
        )
        receipt = _build_execution(
            status=execution_status,
            started_at=start,
            completed_at=_utc_now(),
            trial_id=resolved_trial_id,
            handoff=handoff,
            row=row,
            capture=capture,
            preflight_kind=preflight_kind,
            preflight_status=preflight_status,
            preflight_sha256=preflight_sha256,
            preflight_findings=findings,
            trial=trial,
            validation_receipt_available=(
                validation_receipt_bytes is not None
            ),
        )
        _publish_execution(staging, output, receipt)
        if not exit_matches:
            return receipt, 2
        return receipt, expected_exit
    except CandidateTrialExecutionError:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    except (OSError, ValueError) as exc:
        shutil.rmtree(staging, ignore_errors=True)
        raise CandidateTrialExecutionError(str(exc)) from exc
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--handoff-dir', required=True, type=Path)
    parser.add_argument('--row', required=True, choices=ROW_IDS)
    parser.add_argument(
        '--output-dir',
        required=True,
        type=Path,
        help='new atomic evidence directory outside the product checkout',
    )
    parser.add_argument(
        '--acknowledge-dedicated-trial-host',
        action='store_true',
        help=(
            'confirm this is an isolated disposable trial host with a '
            'dedicated measured filesystem; also permit source host changes '
            'or the reviewed Docker observer bootstrap and privileged '
            'container required by the selected row'
        ),
    )
    parser.add_argument(
        '--trial-id',
        help='optional privacy-bounded slug; defaults to row plus UTC time',
    )
    parser.add_argument(
        '--human-measurements',
        choices=('auto', 'prompt', 'unknown'),
        default='auto',
        help=(
            'auto prompts only on a TTY; non-interactive execution records '
            'human measurements as unknown and therefore non-comparable'
        ),
    )
    parser.add_argument('--disk-scope', type=Path, default=Path('/'))
    parser.add_argument('--timeout-sec', type=float, default=7200.0)
    parser.add_argument(
        '--network-interface',
        help='optional isolated interface override for a source row',
    )
    parser.add_argument('--json', action='store_true')
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Run one candidate row; preserve blocked and failed evidence."""
    args = _parse_args(argv)
    try:
        receipt, exit_code = run_candidate_trial(
            args.handoff_dir,
            args.row,
            args.output_dir,
            acknowledge_dedicated_trial_host=(
                args.acknowledge_dedicated_trial_host
            ),
            trial_id=args.trial_id,
            human_measurements=args.human_measurements,
            disk_scope=args.disk_scope,
            timeout_sec=args.timeout_sec,
            network_interface=args.network_interface,
        )
    except CandidateTrialExecutionError as exc:
        print(f'candidate trial execution error: {exc}', file=sys.stderr)
        return exc.exit_code

    if args.json:
        print(json.dumps(receipt, indent=2, sort_keys=True))
    else:
        print(f"Candidate trial status: {receipt['status']}")
        print(f"Row: {receipt['row']['row_id']}")
        print(f'Evidence: {args.output_dir}')
        if receipt['status'] == 'TRIAL_RECORDED':
            print(
                'Outcome: '
                f"{receipt['trial']['outcome_status']} / "
                f"measurements {receipt['trial']['measurement_status']}"
            )
            print(
                'Comparable: '
                + ('YES' if receipt['trial']['comparable'] else 'NO')
            )
        elif receipt['status'] == 'PREFLIGHT_BLOCKED':
            print(
                'Trial was not started because preflight status was '
                f"{receipt['preflight']['status']}."
            )
        else:
            print('Probe evidence was retained for harness diagnosis.')
        print('No GitHub, registry, release, issue, or community write ran.')
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
