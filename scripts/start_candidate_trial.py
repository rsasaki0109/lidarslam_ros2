#!/usr/bin/env python3
"""Prepare and run one exact candidate trial as one atomic local session.

The command starts from one candidate-image Actions run URL, authenticates its
four-file handoff, runs one structured Docker/source row, and publishes the
handoff plus bounded execution evidence under one new session directory.  It
performs no GitHub, registry, release, issue, or community write.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import shlex
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from prepare_candidate_trial import (
    CandidateTrialPreparationError,
    EXPECTED_HANDOFF_ENTRIES,
    PREPARATION_FILENAME,
    RUN_URL_RE,
    prepare_candidate_trial,
)

from product_schema import validate_contract

from run_candidate_trial import (
    CandidateTrialExecutionError,
    EXECUTION_FILENAME,
    INTERFACE_RE,
    REPOSITORY,
    REPO_ROOT,
    ROW_IDS,
    TRIAL_ID_RE,
    run_candidate_trial,
)


SESSION_SCHEMA = 'candidate-trial-session-v1.schema.json'
SESSION_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/candidate-trial-session-v1.schema.json'
)
SESSION_FILENAME = 'session.json'
READINESS_SCHEMA = 'candidate-trial-readiness-v1.schema.json'
READINESS_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/candidate-trial-readiness-v1.schema.json'
)
HANDOFF_DIRECTORY = 'handoff'
EXECUTION_DIRECTORY = 'execution'
MAX_CHILD_RECEIPT_BYTES = 1024 * 1024
MINIMUM_FREE_BYTES = 8 * 1024 ** 3
EXPECTED_OS_VERSION = {'humble': '22.04', 'jazzy': '24.04'}
DOCKER_ENDPOINT = 'unix:///var/run/docker.sock'
Runner = Callable[..., subprocess.CompletedProcess[str]]


class CandidateTrialSessionError(ValueError):
    """A complete candidate trial session cannot be created safely."""

    def __init__(self, message: str, *, exit_code: int = 2) -> None:
        """Keep the stage-specific process exit code with the error."""
        super().__init__(message)
        self.exit_code = exit_code


def _utc_now() -> str:
    return (
        datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace('+00:00', 'Z')
    )


def _load_child_receipt(
    path: Path,
    label: str,
) -> tuple[dict[str, Any], str]:
    """Load and hash the exact bounded child receipt retained on disk."""
    try:
        if path.is_symlink() or not path.is_file():
            raise CandidateTrialSessionError(
                f'{label} is not one regular non-symlink file'
            )
        size = path.stat().st_size
        if size < 1 or size > MAX_CHILD_RECEIPT_BYTES:
            raise CandidateTrialSessionError(
                f'{label} must be 1..{MAX_CHILD_RECEIPT_BYTES} bytes'
            )
        payload = path.read_bytes()
        receipt = json.loads(payload.decode('utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CandidateTrialSessionError(
            f'cannot read {label}: {exc}'
        ) from exc
    if not isinstance(receipt, dict):
        raise CandidateTrialSessionError(f'{label} JSON root is not an object')
    return receipt, hashlib.sha256(payload).hexdigest()


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


def _validate_request(
    workflow_run_url: str,
    output_dir: Path,
    row_id: str,
    *,
    acknowledge_dedicated_trial_host: bool,
    trial_id: str | None,
    human_measurements: str,
    disk_scope: Path,
    network_interface: str | None,
    interactive: bool,
    timeout_sec: float,
) -> tuple[Path, Path]:
    """Validate every local mutation boundary before staging any output."""
    if RUN_URL_RE.fullmatch(workflow_run_url) is None:
        raise CandidateTrialSessionError(
            'workflow run URL must be the exact rsasaki0109/lidar_slam_ros2 '
            'Actions run URL'
        )
    if not acknowledge_dedicated_trial_host:
        raise CandidateTrialSessionError(
            '--acknowledge-dedicated-trial-host is required; source rows '
            'modify the disposable host and Docker rows build/use a reviewed '
            'observer image plus a privileged isolated container'
        )
    for label, path in (
        ('session output', output_dir),
        ('disk scope', disk_scope),
    ):
        if '\n' in str(path) or '\r' in str(path):
            raise CandidateTrialSessionError(
                f'{label} must not contain a line break'
            )
    if row_id not in ROW_IDS:
        raise CandidateTrialSessionError(
            'row must be one of: ' + ', '.join(ROW_IDS)
        )
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise CandidateTrialSessionError(
            'timeout must be finite and greater than zero'
        )
    if trial_id is not None and TRIAL_ID_RE.fullmatch(trial_id) is None:
        raise CandidateTrialSessionError(
            'trial ID must be a privacy-bounded 3..80 character lower-case '
            'slug'
        )
    if human_measurements not in {'auto', 'prompt', 'unknown'}:
        raise CandidateTrialSessionError(
            'human measurement mode must be auto, prompt, or unknown'
        )
    if human_measurements == 'prompt' and not interactive:
        raise CandidateTrialSessionError(
            '--human-measurements prompt requires an interactive terminal'
        )
    route = row_id.split('-', 1)[0]
    if network_interface is not None:
        if route != 'source':
            raise CandidateTrialSessionError(
                '--network-interface is available only for a source row'
            )
        if INTERFACE_RE.fullmatch(network_interface) is None:
            raise CandidateTrialSessionError(
                'network interface name is unsafe'
            )
    try:
        if disk_scope.is_symlink() or not disk_scope.is_dir():
            raise CandidateTrialSessionError(
                'disk scope must be one existing real directory'
            )
        disk_scope.resolve(strict=True)
    except OSError as exc:
        raise CandidateTrialSessionError(
            f'disk scope cannot be resolved: {exc}'
        ) from exc
    if os.path.lexists(output_dir):
        raise CandidateTrialSessionError(
            f'refusing to overwrite candidate trial session: {output_dir}'
        )
    parent = output_dir.parent
    try:
        if parent.is_symlink() or not parent.is_dir():
            raise CandidateTrialSessionError(
                'session output parent must be one existing non-symlink '
                'directory'
            )
        resolved_parent = parent.resolve(strict=True)
    except OSError as exc:
        raise CandidateTrialSessionError(
            f'session output parent cannot be resolved: {exc}'
        ) from exc
    output = resolved_parent / output_dir.name
    if _contains(REPO_ROOT.resolve(), output):
        raise CandidateTrialSessionError(
            'candidate trial session must be outside the product checkout'
        )
    return output, resolved_parent


def _read_host_os(path: Path) -> tuple[str, str]:
    """Return the bounded OS ID and version from one local os-release."""
    try:
        lines = path.read_text(encoding='utf-8').splitlines()
    except OSError as exc:
        raise CandidateTrialSessionError(
            f'cannot read host OS identity: {exc}'
        ) from exc
    values: dict[str, str] = {}
    for line in lines:
        key, separator, value = line.partition('=')
        if separator and key in {'ID', 'VERSION_ID'}:
            values[key] = value.strip().strip('"\'')
    if not values.get('ID') or not values.get('VERSION_ID'):
        raise CandidateTrialSessionError(
            'host OS identity must contain ID and VERSION_ID'
        )
    return values['ID'], values['VERSION_ID']


def _check(
    check_id: str,
    status: str,
    message: str,
    next_action: str,
) -> dict[str, str]:
    """Build one stable, path-free readiness finding."""
    def _bounded_line(value: str) -> str:
        line = value.replace('\r', '\\r').replace('\n', '\\n')
        return line if len(line) <= 500 else line[:497] + '...'

    return {
        'id': check_id,
        'status': status,
        'message': _bounded_line(message),
        'next_action': _bounded_line(next_action),
    }


def _bounded_token(value: str) -> str | None:
    """Keep one local observed token safe for a single-line status card."""
    allowed = frozenset(
        'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._+-'
    )
    if not 1 <= len(value) <= 80 or any(char not in allowed for char in value):
        return None
    return value


def _local_read(
    command: list[str],
    *,
    runner: Runner,
) -> subprocess.CompletedProcess[str]:
    """Run one bounded local inspection command without requesting writes."""
    try:
        return runner(
            command,
            capture_output=True,
            text=True,
            check=False,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return subprocess.CompletedProcess(command, 125, '', str(exc))


def _command_for_request(
    workflow_run_url: str,
    row_id: str,
    output_dir: Path,
    *,
    check_readiness: bool,
    acknowledge_dedicated_trial_host: bool,
    trial_id: str | None,
    human_measurements: str,
    disk_scope: Path,
    timeout_sec: float,
    network_interface: str | None,
) -> str:
    """Render one shell-safe copy-ready command for the same request."""
    rendered_values = (
        workflow_run_url,
        row_id,
        str(output_dir),
        '' if trial_id is None else trial_id,
        human_measurements,
        str(disk_scope),
        '' if network_interface is None else network_interface,
    )
    if any('\n' in value or '\r' in value for value in rendered_values):
        return 'python3 scripts/start_candidate_trial.py --help'
    command = [
        'python3',
        'scripts/start_candidate_trial.py',
        '--workflow-run-url',
        workflow_run_url,
        '--row',
        row_id,
        '--output-dir',
        str(output_dir),
        '--human-measurements',
        human_measurements,
        '--disk-scope',
        str(disk_scope),
        '--timeout-sec',
        f'{timeout_sec:g}',
    ]
    if network_interface is not None:
        command.extend(['--network-interface', network_interface])
    if trial_id is not None:
        command.extend(['--trial-id', trial_id])
    if acknowledge_dedicated_trial_host:
        command.append('--acknowledge-dedicated-trial-host')
    if check_readiness:
        command.append('--check-readiness')
    return shlex.join(command)


def inspect_candidate_trial_readiness(
    workflow_run_url: str,
    row_id: str,
    output_dir: Path,
    *,
    acknowledge_dedicated_trial_host: bool,
    trial_id: str | None = None,
    human_measurements: str = 'auto',
    disk_scope: Path = Path('/'),
    timeout_sec: float = 7200.0,
    network_interface: str | None = None,
    environment: Mapping[str, str] | None = None,
    command_lookup: Callable[[str], str | None] = shutil.which,
    command_runner: Runner = subprocess.run,
    machine_reader: Callable[[], str] = platform.machine,
    disk_usage: Callable[[Path], Any] = shutil.disk_usage,
    user_id_reader: Callable[[], int] = os.geteuid,
    os_release_path: Path = Path('/etc/os-release'),
    ros_root: Path = Path('/opt/ros'),
    network_root: Path = Path('/sys/class/net'),
    interactive: bool | None = None,
    checked_at: str | None = None,
) -> dict[str, Any]:
    """Inspect one host without network access, writes, or trial execution."""
    resolved_interactive = (
        sys.stdin.isatty() if interactive is None else interactive
    )
    env = dict(os.environ if environment is None else environment)
    checks: list[dict[str, str]] = []

    try:
        _validate_request(
            workflow_run_url,
            output_dir,
            row_id,
            acknowledge_dedicated_trial_host=True,
            trial_id=trial_id,
            human_measurements=human_measurements,
            disk_scope=disk_scope,
            network_interface=network_interface,
            interactive=resolved_interactive,
            timeout_sec=timeout_sec,
        )
    except CandidateTrialSessionError as exc:
        checks.append(_check(
            'request-contract',
            'BLOCKED',
            str(exc),
            'Correct the reported argument or destination, then run the '
            'readiness check again.',
        ))
    else:
        checks.append(_check(
            'request-contract',
            'PASS',
            'The exact run URL, row options, and new output destination are '
            'safe to inspect.',
            'Keep this exact candidate identity and output destination.',
        ))

    distro = row_id.rsplit('-', 1)[-1] if row_id in ROW_IDS else 'unknown'
    route = row_id.split('-', 1)[0] if row_id in ROW_IDS else 'unknown'
    expected_os = EXPECTED_OS_VERSION.get(distro)
    try:
        os_id, os_version = _read_host_os(os_release_path)
    except CandidateTrialSessionError as exc:
        checks.append(_check(
            'host-operating-system',
            'BLOCKED',
            str(exc),
            'Use a disposable Ubuntu 22.04/Humble or 24.04/Jazzy host.',
        ))
    else:
        if os_id == 'ubuntu' and os_version == expected_os:
            checks.append(_check(
                'host-operating-system',
                'PASS',
                f'Host is Ubuntu {os_version}, matching the {distro} row.',
                'Keep this host image unchanged until the trial completes.',
            ))
        else:
            observed_os = (
                f'{os_id} {os_version}'
                if _bounded_token(os_id) and _bounded_token(os_version)
                else 'an unsupported local identity'
            )
            checks.append(_check(
                'host-operating-system',
                'BLOCKED',
                f'{distro} requires Ubuntu {expected_os}; found '
                f'{observed_os}.',
                f'Recreate the disposable host with Ubuntu {expected_os}.',
            ))

    architecture = machine_reader()
    if architecture == 'x86_64':
        checks.append(_check(
            'host-architecture',
            'PASS',
            'Host architecture is x86_64.',
            'Use this same host for the measured row.',
        ))
    else:
        observed_architecture = (
            architecture
            if _bounded_token(architecture) is not None
            else 'an unsupported local architecture'
        )
        checks.append(_check(
            'host-architecture',
            'BLOCKED',
            'The onboarding matrix requires x86_64; found '
            f'{observed_architecture}.',
            'Move the row to a disposable x86_64 host.',
        ))

    try:
        scope = disk_scope.resolve(strict=True)
        parent = output_dir.parent.resolve(strict=True)
        if scope.is_symlink() or not scope.is_dir():
            raise CandidateTrialSessionError(
                'disk scope must be one existing real directory'
            )
        scope_device = scope.stat().st_dev
        measured_paths = [parent]
        if route == 'source':
            measured_paths.extend([Path('/usr'), Path('/var')])
        if any(path.stat().st_dev != scope_device for path in measured_paths):
            raise CandidateTrialSessionError(
                'disk scope does not cover every measured trial path on one '
                'filesystem'
            )
        available = int(disk_usage(scope).free)
        if available < MINIMUM_FREE_BYTES:
            raise CandidateTrialSessionError(
                f'only {available / 1024 ** 3:.1f} GiB is free; at least '
                '8 GiB is required before the fixed demo'
            )
    except (CandidateTrialSessionError, OSError, ValueError) as exc:
        checks.append(_check(
            'measured-filesystem',
            'BLOCKED',
            str(exc),
            'Select one disposable filesystem that contains the session and '
            'all measured paths with at least 8 GiB free.',
        ))
    else:
        checks.append(_check(
            'measured-filesystem',
            'PASS',
            f'The measured filesystem contains the row paths and has '
            f'{available / 1024 ** 3:.1f} GiB free.',
            'Keep unrelated workloads off this filesystem during the trial.',
        ))

    if route == 'docker':
        docker = command_lookup('docker')
        overrides = [
            name for name in ('DOCKER_HOST', 'DOCKER_CONTEXT')
            if env.get(name)
        ]
        problem: str | None = None
        server_version = ''
        if docker is None:
            problem = 'the docker command is unavailable'
        elif overrides:
            problem = (
                'Docker endpoint override is set: ' + ', '.join(overrides)
            )
        else:
            context = _local_read(
                [docker, '--host', DOCKER_ENDPOINT, 'context', 'show'],
                runner=command_runner,
            )
            endpoint = _local_read(
                [
                    docker,
                    '--host',
                    DOCKER_ENDPOINT,
                    'context',
                    'inspect',
                    'default',
                    '--format',
                    '{{.Endpoints.docker.Host}}',
                ],
                runner=command_runner,
            )
            version = _local_read(
                [
                    docker,
                    '--host',
                    DOCKER_ENDPOINT,
                    'version',
                    '--format',
                    '{{.Server.Version}}',
                ],
                runner=command_runner,
            )
            if context.returncode != 0:
                problem = 'the local Docker context cannot be inspected'
            elif context.stdout.strip() != 'default':
                problem = 'the active Docker context is not default'
            elif endpoint.returncode != 0:
                problem = 'the default Docker endpoint cannot be inspected'
            elif endpoint.stdout.strip() != DOCKER_ENDPOINT:
                problem = 'the default Docker context is not the local socket'
            elif version.returncode != 0 or not version.stdout.strip():
                problem = 'the local Docker daemon is unreachable'
            elif _bounded_token(version.stdout.strip()) is None:
                problem = 'the local Docker daemon returned an unsafe version'
            else:
                server_version = version.stdout.strip()
        if problem is None:
            checks.append(_check(
                'route-runtime',
                'PASS',
                f'Local Docker daemon {server_version} is reachable through '
                'the reviewed default socket.',
                'Do not switch Docker context before the trial.',
            ))
        else:
            checks.append(_check(
                'route-runtime',
                'BLOCKED',
                problem + '.',
                'Install/start local Docker, select its default context, and '
                'remove DOCKER_HOST/DOCKER_CONTEXT overrides.',
            ))
    elif route == 'source':
        required = ['bash', 'df', 'du', 'git', 'ip', 'apt-get']
        if user_id_reader() != 0:
            required.append('sudo')
        missing = [name for name in required if command_lookup(name) is None]
        setup = ros_root / distro / 'setup.bash'
        if missing:
            checks.append(_check(
                'route-runtime',
                'BLOCKED',
                'Required source observer commands are missing: '
                + ', '.join(missing) + '.',
                'Install the missing base tools on the disposable host.',
            ))
        elif not setup.is_file():
            checks.append(_check(
                'route-runtime',
                'BLOCKED',
                f'ROS 2 {distro} setup is missing.',
                f'Install ROS 2 {distro} under /opt/ros before the trial.',
            ))
        else:
            checks.append(_check(
                'route-runtime',
                'PASS',
                f'ROS 2 {distro} and every source observer command are '
                'available.',
                'Keep this base ROS installation unchanged until the trial.',
            ))

        interface = network_interface
        network_problem: str | None = None
        if interface is None:
            ip_command = command_lookup('ip')
            if ip_command is None:
                network_problem = 'the ip command is unavailable'
            else:
                route_result = _local_read(
                    [ip_command, '-o', 'route', 'show', 'default'],
                    runner=command_runner,
                )
                fields = route_result.stdout.split()
                if route_result.returncode != 0:
                    network_problem = 'the default network route is unreadable'
                else:
                    try:
                        interface = fields[fields.index('dev') + 1]
                    except (ValueError, IndexError):
                        network_problem = (
                            'the default network interface cannot be resolved'
                        )
        if interface is not None and INTERFACE_RE.fullmatch(interface) is None:
            network_problem = 'the network interface name is unsafe'
        if network_problem is None and interface is not None:
            counter = network_root / interface / 'statistics' / 'rx_bytes'
            try:
                rx_bytes = int(counter.read_text(encoding='utf-8').strip())
                if rx_bytes < 0:
                    raise ValueError('negative counter')
            except (OSError, ValueError):
                network_problem = (
                    'the selected interface RX counter is unreadable'
                )
        if network_problem is None and interface is not None:
            checks.append(_check(
                'network-measurement',
                'PASS',
                f'RX measurement is available on interface {interface}.',
                'Keep unrelated traffic off this interface during the trial.',
            ))
        else:
            checks.append(_check(
                'network-measurement',
                'BLOCKED',
                (network_problem or 'no network interface was selected') + '.',
                'Select an isolated interface with a readable RX counter.',
            ))

    if human_measurements == 'prompt' and not resolved_interactive:
        checks.append(_check(
            'human-observer-measurements',
            'BLOCKED',
            'Prompted human measurements require an interactive terminal.',
            'Open a terminal with a neutral observer and rerun this check.',
        ))
    elif human_measurements == 'unknown' or (
        human_measurements == 'auto' and not resolved_interactive
    ):
        checks.append(_check(
            'human-observer-measurements',
            'WARNING',
            'The trial can run, but active time and command count will be '
            'unknown and the row cannot be comparable.',
            'Run from an interactive terminal with --human-measurements '
            'prompt and a neutral observer.',
        ))
    else:
        checks.append(_check(
            'human-observer-measurements',
            'PASS',
            'The interactive run will request neutral-observer active time '
            'and command count.',
            'Prepare a paused stopwatch and human command log.',
        ))

    if acknowledge_dedicated_trial_host:
        checks.append(_check(
            'dedicated-host-confirmation',
            'PASS',
            'The operator confirmed the disposable host, isolated '
            'filesystem, and route-specific mutation boundary.',
            'Destroy or clean up only this named disposable environment after '
            'archiving the evidence.',
        ))
    else:
        route_detail = (
            'privileged nested Docker use'
            if route == 'docker' else 'APT/build host mutation'
        )
        checks.append(_check(
            'dedicated-host-confirmation',
            'CONFIRMATION_REQUIRED',
            f'Machine checks cannot prove that the host is disposable, its '
            f'filesystem/network are isolated, or {route_detail} is '
            'acceptable.',
            'Review the isolation checklist; acknowledge only when every '
            'statement is true.',
        ))

    counts = {
        status: sum(item['status'] == status for item in checks)
        for status in ('PASS', 'BLOCKED', 'CONFIRMATION_REQUIRED', 'WARNING')
    }
    comparable = counts['WARNING'] == 0 and not any(
        item['id'] == 'human-observer-measurements'
        and item['status'] == 'BLOCKED'
        for item in checks
    )
    execution_ready = (
        counts['BLOCKED'] == 0 and counts['CONFIRMATION_REQUIRED'] == 0
    )
    if counts['BLOCKED']:
        status = 'BLOCKED'
    elif counts['CONFIRMATION_REQUIRED']:
        status = 'CONFIRMATION_REQUIRED'
    elif not comparable:
        status = 'READY_NONCOMPARABLE'
    else:
        status = 'READY'

    if status == 'BLOCKED':
        next_id = 'fix-blockers-and-recheck'
        next_title = 'Fix each blocked check, then inspect again'
        next_command = _command_for_request(
            workflow_run_url,
            row_id,
            output_dir,
            check_readiness=True,
            acknowledge_dedicated_trial_host=(
                acknowledge_dedicated_trial_host
            ),
            trial_id=trial_id,
            human_measurements=human_measurements,
            disk_scope=disk_scope,
            timeout_sec=timeout_sec,
            network_interface=network_interface,
        )
        boundary = 'read-only local inspection; no network or file write'
    elif status == 'READY_NONCOMPARABLE':
        next_id = 'prepare-neutral-observer'
        next_title = 'Open an interactive terminal with a neutral observer'
        next_command = _command_for_request(
            workflow_run_url,
            row_id,
            output_dir,
            check_readiness=True,
            acknowledge_dedicated_trial_host=True,
            trial_id=trial_id,
            human_measurements='prompt',
            disk_scope=disk_scope,
            timeout_sec=timeout_sec,
            network_interface=network_interface,
        )
        boundary = 'read-only local inspection; no network or file write'
    else:
        next_id = (
            'review-isolation-and-start'
            if status == 'CONFIRMATION_REQUIRED' else 'start-candidate-trial'
        )
        next_title = (
            'Review the isolation checklist, then start this exact row'
            if status == 'CONFIRMATION_REQUIRED'
            else 'Start this exact candidate row'
        )
        next_command = _command_for_request(
            workflow_run_url,
            row_id,
            output_dir,
            check_readiness=False,
            acknowledge_dedicated_trial_host=True,
            trial_id=trial_id,
            human_measurements=(
                'prompt' if counts['WARNING'] else human_measurements
            ),
            disk_scope=disk_scope,
            timeout_sec=timeout_sec,
            network_interface=network_interface,
        )
        boundary = (
            'bounded public network reads and one new local evidence '
            'directory; no remote mutation'
        )

    report = {
        'schema_version': 1,
        'schema_uri': READINESS_SCHEMA_URI,
        'status': status,
        'checked_at': checked_at or _utc_now(),
        'repository': REPOSITORY,
        'row': {
            'row_id': row_id,
            'route': route,
            'ros_distro': distro,
            'expected_os_family': (
                None if expected_os is None else f'ubuntu-{expected_os}'
            ),
            'required_architecture': 'x86_64',
        },
        'request': {
            'workflow_run_url': workflow_run_url,
            'output_directory_exists': os.path.lexists(output_dir),
            'human_measurements': human_measurements,
            'interactive_terminal': resolved_interactive,
            'dedicated_trial_host_acknowledged': (
                acknowledge_dedicated_trial_host
            ),
            'minimum_free_bytes': MINIMUM_FREE_BYTES,
        },
        'summary': {
            'pass_count': counts['PASS'],
            'blocker_count': counts['BLOCKED'],
            'confirmation_count': counts['CONFIRMATION_REQUIRED'],
            'warning_count': counts['WARNING'],
            'execution_ready': execution_ready,
            'comparable_measurements_planned': comparable,
        },
        'checks': checks,
        'next_action': {
            'id': next_id,
            'title': next_title,
            'command': next_command,
            'write_boundary': boundary,
        },
        'authority': {
            'network_reads_performed': False,
            'local_files_written': False,
            'trial_executed': False,
            'github_writes_authorized': False,
            'registry_writes_authorized': False,
            'release_writes_authorized': False,
            'community_writes_authorized': False,
            'remote_mutations_performed': False,
        },
    }
    validate_contract(report, READINESS_SCHEMA)
    return report


def _render_readiness(report: dict[str, Any]) -> None:
    """Render one concise card with stable findings and an exact next step."""
    print(f"Candidate trial readiness: {report['status']}")
    row = report['row']
    print(
        f"Row: {row['row_id']} ({row['expected_os_family']}, "
        f"{row['required_architecture']})"
    )
    for item in report['checks']:
        print(f"[{item['status']}] {item['id']}: {item['message']}")
        if item['status'] != 'PASS':
            print(f"  Fix: {item['next_action']}")
    action = report['next_action']
    print(f"Next: {action['title']}")
    print(f"  {action['command']}")
    print(f"Boundary: {action['write_boundary']}")
    print('Readiness inspection performed no network access or write.')


def _build_session(
    *,
    started_at: str,
    completed_at: str,
    workflow_run_url: str,
    preparation: dict[str, Any],
    preparation_sha256: str,
    execution: dict[str, Any],
    execution_sha256: str,
) -> dict[str, Any]:
    handoff_entries = [
        f'{HANDOFF_DIRECTORY}/{name}'
        for name in sorted(EXPECTED_HANDOFF_ENTRIES)
    ]
    execution_entries = [
        f'{EXECUTION_DIRECTORY}/{name}'
        for name in execution['sharing']['bounded_outputs']
    ]
    private = execution['outputs']['private_evidence_directory']
    receipt = {
        'schema_version': 1,
        'schema_uri': SESSION_SCHEMA_URI,
        'status': execution['status'],
        'started_at': started_at,
        'completed_at': completed_at,
        'repository': REPOSITORY,
        'workflow_run_url': workflow_run_url,
        'row': execution['row'],
        'handoff': {
            'directory': HANDOFF_DIRECTORY,
            'preparation_json': (
                f'{HANDOFF_DIRECTORY}/{PREPARATION_FILENAME}'
            ),
            'preparation_sha256': preparation_sha256,
            'status': preparation['status'],
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
        'execution': {
            'directory': EXECUTION_DIRECTORY,
            'execution_json': (
                f'{EXECUTION_DIRECTORY}/{EXECUTION_FILENAME}'
            ),
            'execution_sha256': execution_sha256,
            'status': execution['status'],
            'trial_id': execution['trial_id'],
            'outcome_status': execution['trial']['outcome_status'],
            'measurement_status': execution['trial']['measurement_status'],
            'comparable': execution['trial']['comparable'],
        },
        'outputs': {
            'handoff_directory': HANDOFF_DIRECTORY,
            'execution_directory': EXECUTION_DIRECTORY,
            'session_json': SESSION_FILENAME,
        },
        'sharing': {
            'bounded_outputs': [
                SESSION_FILENAME,
                *handoff_entries,
                *execution_entries,
            ],
            'private_evidence_directory': (
                None if private is None
                else f'{EXECUTION_DIRECTORY}/{private}'
            ),
            'private_evidence_shareable': False,
            'review_before_sharing': True,
        },
        'authority': {
            'network_reads_performed': True,
            'local_files_written': True,
            'atomic_output_publication': True,
            'candidate_handoff_prepared': True,
            'trial_executed': execution['authority']['trial_executed'],
            'docker_observer_bootstrap_requested': (
                execution['row']['route'] == 'docker'
            ),
            'github_writes_authorized': False,
            'registry_writes_authorized': False,
            'release_writes_authorized': False,
            'community_writes_authorized': False,
            'remote_mutations_performed': False,
        },
    }
    validate_contract(receipt, SESSION_SCHEMA)
    return receipt


def start_candidate_trial(
    workflow_run_url: str,
    row_id: str,
    output_dir: Path,
    *,
    acknowledge_dedicated_trial_host: bool,
    trial_id: str | None = None,
    human_measurements: str = 'auto',
    disk_scope: Path = Path('/'),
    timeout_sec: float = 7200.0,
    network_interface: str | None = None,
    preparation_runner: Runner = subprocess.run,
    read_runner: Runner = subprocess.run,
    probe_runner: Runner = subprocess.run,
    interactive: bool | None = None,
    started_at: str | None = None,
) -> tuple[dict[str, Any], int]:
    """Prepare, execute, and publish one complete candidate trial session."""
    resolved_interactive = (
        sys.stdin.isatty() if interactive is None else interactive
    )
    output, parent = _validate_request(
        workflow_run_url,
        output_dir,
        row_id,
        acknowledge_dedicated_trial_host=(
            acknowledge_dedicated_trial_host
        ),
        trial_id=trial_id,
        human_measurements=human_measurements,
        disk_scope=disk_scope,
        network_interface=network_interface,
        interactive=resolved_interactive,
        timeout_sec=timeout_sec,
    )
    start = started_at or _utc_now()
    staging = Path(tempfile.mkdtemp(
        prefix=f'.{output.name}.session-',
        dir=parent,
    ))
    try:
        handoff_dir = staging / HANDOFF_DIRECTORY
        execution_dir = staging / EXECUTION_DIRECTORY
        preparation = prepare_candidate_trial(
            workflow_run_url,
            handoff_dir,
            runner=preparation_runner,
            prepared_at=start,
        )
        execution, exit_code = run_candidate_trial(
            handoff_dir,
            row_id,
            execution_dir,
            acknowledge_dedicated_trial_host=True,
            trial_id=trial_id,
            human_measurements=human_measurements,
            disk_scope=disk_scope,
            timeout_sec=timeout_sec,
            network_interface=network_interface,
            read_runner=read_runner,
            probe_runner=probe_runner,
            interactive=resolved_interactive,
            started_at=start,
        )
        retained_preparation, preparation_sha256 = _load_child_receipt(
            handoff_dir / PREPARATION_FILENAME,
            'retained preparation receipt',
        )
        if retained_preparation != preparation:
            raise CandidateTrialSessionError(
                'retained preparation receipt differs from the delegated '
                'result'
            )
        retained_execution, execution_sha256 = _load_child_receipt(
            execution_dir / EXECUTION_FILENAME,
            'retained execution receipt',
        )
        if retained_execution != execution:
            raise CandidateTrialSessionError(
                'retained execution receipt differs from the delegated '
                'result'
            )
        receipt = _build_session(
            started_at=start,
            completed_at=_utc_now(),
            workflow_run_url=workflow_run_url,
            preparation=retained_preparation,
            preparation_sha256=preparation_sha256,
            execution=retained_execution,
            execution_sha256=execution_sha256,
        )
        _write_json(staging / SESSION_FILENAME, receipt)
        os.replace(staging, output)
        return receipt, exit_code
    except CandidateTrialPreparationError as exc:
        shutil.rmtree(staging, ignore_errors=True)
        raise CandidateTrialSessionError(
            f'candidate handoff preparation failed: {exc}',
            exit_code=exc.exit_code,
        ) from exc
    except CandidateTrialExecutionError as exc:
        shutil.rmtree(staging, ignore_errors=True)
        raise CandidateTrialSessionError(
            f'candidate row execution failed before evidence publication: '
            f'{exc}',
            exit_code=exc.exit_code,
        ) from exc
    except CandidateTrialSessionError:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    except (OSError, ValueError) as exc:
        shutil.rmtree(staging, ignore_errors=True)
        raise CandidateTrialSessionError(str(exc)) from exc
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--workflow-run-url',
        required=True,
        help='exact successful candidate-image Actions run URL',
    )
    parser.add_argument('--row', required=True, choices=ROW_IDS)
    parser.add_argument(
        '--output-dir',
        required=True,
        type=Path,
        help=(
            'new atomic session directory outside the checkout; contains '
            'handoff/, execution/, and session.json'
        ),
    )
    parser.add_argument(
        '--acknowledge-dedicated-trial-host',
        action='store_true',
        help=(
            'confirm an isolated disposable trial host and dedicated '
            'measured filesystem; permit source changes or reviewed Docker '
            'bootstrap and the privileged nested host for the selected row'
        ),
    )
    parser.add_argument('--trial-id')
    parser.add_argument(
        '--human-measurements',
        choices=('auto', 'prompt', 'unknown'),
        default='auto',
    )
    parser.add_argument('--disk-scope', type=Path, default=Path('/'))
    parser.add_argument('--timeout-sec', type=float, default=7200.0)
    parser.add_argument(
        '--network-interface',
        help='optional isolated interface override for a source row',
    )
    parser.add_argument(
        '--check-readiness',
        action='store_true',
        help=(
            'inspect the local host and print one exact next command without '
            'network access, writes, or trial execution'
        ),
    )
    parser.add_argument('--json', action='store_true')
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Inspect readiness or run one complete evidence-retaining session."""
    args = _parse_args(argv)
    if args.check_readiness:
        try:
            report = inspect_candidate_trial_readiness(
                args.workflow_run_url,
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
        except (CandidateTrialSessionError, OSError, ValueError) as exc:
            print(f'candidate trial readiness error: {exc}', file=sys.stderr)
            return 2
        if args.json:
            print(json.dumps(report, indent=2, sort_keys=True))
        else:
            _render_readiness(report)
        return 0 if report['status'] == 'READY' else 1

    try:
        receipt, exit_code = start_candidate_trial(
            args.workflow_run_url,
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
    except CandidateTrialSessionError as exc:
        print(f'candidate trial session error: {exc}', file=sys.stderr)
        return exc.exit_code

    if args.json:
        print(json.dumps(receipt, indent=2, sort_keys=True))
    else:
        print(f"Candidate trial session: {receipt['status']}")
        print(f"Row: {receipt['row']['row_id']}")
        print(f'Session evidence: {args.output_dir}')
        print(f'Handoff: {args.output_dir / HANDOFF_DIRECTORY}')
        print(f'Execution: {args.output_dir / EXECUTION_DIRECTORY}')
        if receipt['execution']['outcome_status'] is not None:
            print(
                'Outcome: '
                f"{receipt['execution']['outcome_status']} / "
                f"measurements {receipt['execution']['measurement_status']}"
            )
            print(
                'Comparable: '
                + ('YES' if receipt['execution']['comparable'] else 'NO')
            )
        print('No GitHub, registry, release, issue, or community write ran.')
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
