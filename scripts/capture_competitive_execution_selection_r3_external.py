#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""Capture unreviewed r3 image/toolchain observations without promotion.

This utility is deliberately narrower than the r3 handoff verifier.  It
reopens the checked-in candidate and observes only the three pinned local
image identities.  Container toolchain probes are opt-in and use a fixed,
network-disabled, read-only Docker argv.  Every result remains an unreviewed
observation; no status emitted by this module is ``READY`` or
``REVIEWED_EXTERNAL``.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
from typing import Any, Callable, Mapping

_SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(_SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(_SOURCE_ROOT))

from scripts.prepare_competitive_execution_selection_r3_handoff import (  # noqa: E402
    _exclusive_write,
    _read_immutable,
    _remove_owned,
    MAX_HANDOFF_BYTES,
)
from scripts.validate_competitive_execution_selection_r3 import (  # noqa: E402
    CANDIDATE_REL,
    CandidateError,
    ROOT,
    validate_candidate,
)


CAPTURE_SCHEMA_VERSION = 1
CAPTURE_MANIFEST_KIND = (
    'competitive_execution_selection_r3_external_capture_manifest_v1')
IMAGE_ARTIFACT_KIND = 'competitive_execution_selection_r3_image_inspect_v1'
TOOLCHAIN_ARTIFACT_KIND = 'competitive_execution_selection_r3_toolchain_capture_v1'
CAPTURE_SIDECAR_SUFFIX = '.sha256'
SYSTEMS = ('ours', 'glim', 'fast_livo2')
NOT_APPLICABLE_FIELDS_BY_SYSTEM = {
    'ours': frozenset(),
    'glim': frozenset({'pcl'}),
    'fast_livo2': frozenset(),
}
IMAGE_DIGEST_RE = re.compile(r'^sha256:[0-9a-f]{64}$')
ISO_UTC_RE = re.compile(
    r'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z$')
MAX_COMMAND_OUTPUT_BYTES = 1024 * 1024
MAX_VALUE_BYTES = 512
COMMAND_TIMEOUT_SECONDS = 30

TOOLCHAIN_PROBES: dict[str, tuple[str, ...]] = {
    'compiler': ('c++', '--version'),
    'linker': ('ld', '--version'),
    'ros_distro': ('printenv', 'ROS_DISTRO'),
    'pcl': ('dpkg-query', '-W', '-f=${Version}', 'libpcl-dev'),
    'eigen': ('dpkg-query', '-W', '-f=${Version}', 'libeigen3-dev'),
    'openmp': ('dpkg-query', '-W', '-f=${Version}', 'libgomp1'),
}

IMAGE_DOCUMENT_FIELDS = {
    'schema_version', 'artifact_kind', 'status', 'review_status',
    'benchmark_eligible', 'claim_eligible', 'observed_at',
    'candidate_binding', 'safety', 'systems', 'blockers',
    'artifact_identity_sha256'}
TOOLCHAIN_DOCUMENT_FIELDS = IMAGE_DOCUMENT_FIELDS | {'probe_requested'}
MANIFEST_FIELDS = {
    'schema_version', 'manifest_kind', 'status', 'review_status',
    'benchmark_eligible', 'claim_eligible', 'observed_at',
    'candidate_binding', 'safety', 'image_inspect', 'toolchain_capture',
    'blockers', 'capture_identity_sha256'}


class CaptureError(ValueError):
    """A stale, unsafe, malformed, or non-promoting capture."""


@dataclass(frozen=True)
class CommandResult:
    """Bounded command result used by the real and synthetic runners."""

    returncode: int
    stdout: bytes = b''
    stderr: bytes = b''
    timed_out: bool = False


Runner = Callable[[list[str]], CommandResult]


def _canonical(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(',', ':'),
                       ensure_ascii=True) + '\n').encode('utf-8')


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _identity(value: Mapping[str, Any], field: str) -> str:
    payload = {key: item for key, item in value.items() if key != field}
    return _sha256(_canonical(payload))


def _safe_absolute(path: Path, label: str) -> Path:
    if not path.is_absolute() or '..' in path.parts:
        raise CaptureError(f'{label} must be absolute and traversal-free')
    current = Path(path.anchor)
    for component in path.parts[1:-1]:
        current /= component
        if current.is_symlink():
            raise CaptureError(f'{label} has a symlinked parent')
    if path.name in ('', '.', '..') or any(
            char.isspace() or ord(char) < 32 for char in path.name):
        raise CaptureError(f'{label} basename is unsafe')
    return path


def _directory_identity(path: Path, label: str) -> tuple[int, int, int]:
    path = _safe_absolute(path, label)
    try:
        info = os.lstat(path)
    except OSError as exc:
        raise CaptureError(f'{label} cannot be inspected: {exc}') from exc
    if not stat.S_ISDIR(info.st_mode):
        raise CaptureError(f'{label} is not a directory')
    return info.st_dev, info.st_ino, stat.S_IMODE(info.st_mode)


def _read_text(value: bytes, label: str, *, max_bytes: int) -> str:
    if len(value) > max_bytes:
        raise CaptureError(f'{label} exceeds the bounded output size')
    try:
        text = value.decode('utf-8')
    except UnicodeDecodeError as exc:
        raise CaptureError(f'{label} is not UTF-8') from exc
    return text


def _normalize_result(value: Any) -> CommandResult:
    if isinstance(value, CommandResult):
        result = value
    elif isinstance(value, Mapping):
        result = CommandResult(
            returncode=value.get('returncode', 127),
            stdout=value.get('stdout', b''), stderr=value.get('stderr', b''),
            timed_out=bool(value.get('timed_out', False)))
    else:
        raise CaptureError('command runner returned an unsupported result')
    if isinstance(result.stdout, str):
        stdout = result.stdout.encode('utf-8')
    else:
        stdout = bytes(result.stdout)
    if isinstance(result.stderr, str):
        stderr = result.stderr.encode('utf-8')
    else:
        stderr = bytes(result.stderr)
    if (isinstance(result.returncode, bool) or
            not isinstance(result.returncode, int)):
        raise CaptureError('command result returncode is not an integer')
    return CommandResult(result.returncode, stdout, stderr, result.timed_out)


def run_command(command: list[str]) -> CommandResult:
    """Run fixed argv with no shell and bounded post-capture output."""
    if not command or any(not isinstance(item, str) or not item for item in command):
        raise CaptureError('command argv is not exact')
    try:
        completed = subprocess.run(
            command, check=False, shell=False, capture_output=True,
            timeout=COMMAND_TIMEOUT_SECONDS)
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or b''
        stderr = exc.stderr or b''
        return CommandResult(124, stdout, stderr, timed_out=True)
    except OSError as exc:
        return CommandResult(127, b'', str(exc).encode('utf-8'), timed_out=False)
    return CommandResult(completed.returncode, completed.stdout,
                         completed.stderr, timed_out=False)


def _candidate_snapshot(candidate_path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    report = validate_candidate(candidate_path)
    if not report.get('structural_valid'):
        raise CaptureError(f'candidate is not structurally valid: {report.get("error")}')
    try:
        payload = candidate_path.read_bytes()
        candidate = json.loads(payload.decode('utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CaptureError(f'candidate cannot be reopened: {exc}') from exc
    if _sha256(payload) != report['candidate_file_sha256']:
        raise CaptureError('candidate changed between validation and capture')
    if not isinstance(candidate, dict):
        raise CaptureError('candidate JSON is not an object')
    binding = {
        'path': CANDIDATE_REL,
        'file_sha256': report['candidate_file_sha256'],
        'candidate_identity_sha256': report['candidate_identity_sha256'],
        'candidate_id': report['candidate_id'],
    }
    systems = candidate.get('system_bindings')
    if not isinstance(systems, dict) or set(systems) != set(SYSTEMS):
        raise CaptureError('candidate system binding set is not exact')
    for system in SYSTEMS:
        item = systems[system]
        if not isinstance(item, dict) or set(item) != {
                'profile_image_pointer', 'image_digest', 'image_status',
                'toolchain_fingerprint', 'toolchain_status', 'release'}:
            raise CaptureError(f'candidate {system} binding shape is not exact')
        if IMAGE_DIGEST_RE.fullmatch(item['image_digest']) is None:
            raise CaptureError(f'candidate {system} image digest is invalid')
    return candidate, {'binding': binding, 'systems': systems}


def _image_command(digest: str) -> list[str]:
    return ['docker', 'image', 'inspect', '--format', '{{json .}}', digest]


def _toolchain_command(digest: str, fragment: tuple[str, ...]) -> list[str]:
    return [
        'docker', 'run', '--rm', '--pull=never', '--network', 'none',
        '--read-only', '--entrypoint', fragment[0], digest, *fragment[1:],
    ]


def _generic_reason(result: CommandResult) -> str:
    if result.timed_out:
        return 'probe_timeout'
    if result.returncode != 0:
        return 'probe_nonzero_exit'
    return 'probe_output_invalid'


def _inspect_payload(result: CommandResult, expected: str) -> tuple[str, dict[str, Any] | None]:
    if result.timed_out or result.returncode != 0:
        return _generic_reason(result), None
    text = _read_text(result.stdout, 'docker image inspect stdout',
                      max_bytes=MAX_COMMAND_OUTPUT_BYTES)
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        return 'inspect_json_invalid', None
    if not isinstance(value, dict):
        return 'inspect_json_not_object', None
    observed_id = value.get('Id')
    if observed_id != expected:
        return 'image_id_mismatch', {
            'Id': observed_id if isinstance(observed_id, str) else None,
        }
    repo_digests = value.get('RepoDigests', [])
    repo_tags = value.get('RepoTags', [])
    if not isinstance(repo_digests, list) or not all(
            isinstance(item, str) and len(item) <= MAX_VALUE_BYTES
            for item in repo_digests):
        return 'inspect_repo_digests_invalid', None
    if not isinstance(repo_tags, list) or not all(
            isinstance(item, str) and len(item) <= MAX_VALUE_BYTES
            for item in repo_tags):
        return 'inspect_repo_tags_invalid', None
    created = value.get('Created')
    architecture = value.get('Architecture')
    operating_system = value.get('Os')
    if created is not None and not isinstance(created, str):
        return 'inspect_created_invalid', None
    if architecture is not None and not isinstance(architecture, str):
        return 'inspect_architecture_invalid', None
    if operating_system is not None and not isinstance(operating_system, str):
        return 'inspect_os_invalid', None
    size = value.get('Size')
    if size is not None and (isinstance(size, bool) or not isinstance(size, int)):
        return 'inspect_size_invalid', None
    return '', {
        'Id': expected,
        'RepoDigests': sorted(repo_digests),
        'RepoTags': sorted(repo_tags),
        'Created': created,
        'Architecture': architecture,
        'Os': operating_system,
        'Size': size,
    }


def _capture_image(system: str, expected: str, runner: Runner) -> dict[str, Any]:
    command = _image_command(expected)
    try:
        result = _normalize_result(runner(command))
        reason, observed = _inspect_payload(result, expected)
    except (CaptureError, OSError, UnicodeError) as exc:
        result = CommandResult(127, b'', b'', False)
        reason, observed = f'probe_runner_error:{type(exc).__name__}', None
    status = 'OBSERVED' if not reason else 'PENDING'
    return {
        'status': status,
        'expected_digest': expected,
        'command': command,
        'returncode': result.returncode,
        'timed_out': result.timed_out,
        'reason': reason or None,
        'observed': observed,
    }


def _first_value(stdout: bytes) -> tuple[str | None, str]:
    if len(stdout) > MAX_COMMAND_OUTPUT_BYTES:
        return None, 'probe_output_too_large'
    text = _read_text(stdout, 'toolchain probe stdout',
                      max_bytes=MAX_COMMAND_OUTPUT_BYTES)
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if not lines:
        return None, 'probe_output_empty'
    value = lines[0]
    if len(value.encode('utf-8')) > MAX_VALUE_BYTES or any(
            ord(char) < 32 and char not in '\t' for char in value):
        return None, 'probe_value_invalid'
    return value, ''


def _capture_toolchain(
        expected: str, image: Mapping[str, Any], requested: bool,
        runner: Runner, not_applicable_fields: set[str]) -> dict[str, Any]:
    if not requested:
        return {
            'status': 'NOT_REQUESTED', 'probe_requested': False,
            'image_digest': expected, 'probes': {}, 'fingerprint': None,
            'not_applicable_fields': sorted(not_applicable_fields),
            'reason': 'explicit_probe_option_not_supplied',
        }
    if image['status'] != 'OBSERVED':
        return {
            'status': 'PENDING', 'probe_requested': True,
            'image_digest': expected, 'probes': {}, 'fingerprint': None,
            'not_applicable_fields': sorted(not_applicable_fields),
            'reason': 'image_inspect_not_observed',
        }
    probes: dict[str, Any] = {}
    values: dict[str, str] = {}
    for name, fragment in TOOLCHAIN_PROBES.items():
        if name in not_applicable_fields:
            values[name] = 'not_applicable'
            probes[name] = {
                'command': None,
                'returncode': 0,
                'timed_out': False,
                'value': 'not_applicable',
                'stdout_sha256': _sha256(b''),
                'reason': 'not_applicable',
            }
            continue
        command = _toolchain_command(expected, fragment)
        try:
            result = _normalize_result(runner(command))
            value, reason = _first_value(result.stdout) \
                if result.returncode == 0 and not result.timed_out \
                else (None, _generic_reason(result))
        except (CaptureError, OSError, UnicodeError) as exc:
            result = CommandResult(127, b'', b'', False)
            value, reason = None, f'probe_runner_error:{type(exc).__name__}'
        if result.returncode == 0 and not result.timed_out and value:
            values[name] = value
        probes[name] = {
            'command': command,
            'returncode': result.returncode,
            'timed_out': result.timed_out,
            'value': value,
            'stdout_sha256': _sha256(result.stdout),
            'reason': reason or None,
        }
    complete = len(values) == len(TOOLCHAIN_PROBES)
    return {
        'status': 'OBSERVED' if complete else 'PENDING',
        'probe_requested': True,
        'image_digest': expected,
        'probes': probes,
        'fingerprint': _sha256(_canonical(values)) if complete else None,
        'not_applicable_fields': sorted(not_applicable_fields),
        'reason': None if complete else 'one_or_more_probes_pending',
    }


def _timestamp(value: str | None) -> str:
    if value is None:
        return dt.datetime.now(dt.timezone.utc).replace(
            microsecond=0).isoformat().replace('+00:00', 'Z')
    if ISO_UTC_RE.fullmatch(value) is None:
        raise CaptureError('observed_at must be an RFC3339 UTC timestamp')
    try:
        parsed = dt.datetime.fromisoformat(value.replace('Z', '+00:00'))
    except ValueError as exc:
        raise CaptureError('observed_at is not a valid timestamp') from exc
    if parsed.tzinfo is None:
        raise CaptureError('observed_at must contain UTC')
    return value


def _candidate_doc(binding: Mapping[str, Any]) -> dict[str, Any]:
    return {
        'path': binding['path'],
        'file_sha256': binding['file_sha256'],
        'candidate_identity_sha256': binding['candidate_identity_sha256'],
        'candidate_id': binding['candidate_id'],
    }


def _status_for(items: Mapping[str, Mapping[str, Any]]) -> str:
    return ('UNREVIEWED_COMPLETE_OBSERVATION'
            if all(item['status'] in {'OBSERVED', 'UNREVIEWED_COMPLETE_OBSERVATION'}
                   for item in items.values())
            else 'PARTIAL_OBSERVATION')


def _artifact_identity(document: Mapping[str, Any]) -> str:
    return _identity(document, 'artifact_identity_sha256')


def capture_external(
        *, output_dir: Path, candidate_path: Path = ROOT / CANDIDATE_REL,
        probe_toolchain: bool = False, runner: Runner | None = None,
        observed_at: str | None = None) -> dict[str, Any]:
    """Capture image/toolchain observations into a fresh immutable directory."""
    output_dir = _safe_absolute(output_dir, 'capture output directory')
    if output_dir.exists() or output_dir.is_symlink():
        raise CaptureError('capture output directory must be fresh and absent')
    candidate, candidate_info = _candidate_snapshot(candidate_path)
    del candidate
    parent = output_dir.parent
    _safe_absolute(parent, 'capture output parent')
    parent.mkdir(parents=True, exist_ok=True)
    if parent.is_symlink():
        raise CaptureError('capture output parent is a symlink')
    try:
        output_dir.mkdir(mode=0o700)
    except OSError as exc:
        raise CaptureError(f'capture output directory cannot be created: {exc}') from exc
    _directory_identity(output_dir, 'capture output directory')
    runner = runner or run_command
    binding = candidate_info['binding']
    timestamp = _timestamp(observed_at)
    image_systems: dict[str, Any] = {}
    toolchain_systems: dict[str, Any] = {}
    blockers = ['capture_is_observation_only', 'external_custodian_review_required']
    for system in SYSTEMS:
        expected = candidate_info['systems'][system]['image_digest']
        image = _capture_image(system, expected, runner)
        image_systems[system] = image
        if image['status'] != 'OBSERVED':
            blockers.append(f'{system}_image_inspect_pending')
        toolchain = _capture_toolchain(
            expected, image, probe_toolchain, runner,
            set(NOT_APPLICABLE_FIELDS_BY_SYSTEM[system]))
        toolchain_systems[system] = toolchain
        if toolchain['status'] != 'OBSERVED':
            blockers.append(f'{system}_toolchain_pending')
    image_status = _status_for(image_systems)
    toolchain_status = _status_for(toolchain_systems)
    image_document: dict[str, Any] = {
        'schema_version': CAPTURE_SCHEMA_VERSION,
        'artifact_kind': IMAGE_ARTIFACT_KIND,
        'status': image_status,
        'review_status': 'NOT_REVIEWED_EXTERNAL',
        'benchmark_eligible': False,
        'claim_eligible': False,
        'observed_at': timestamp,
        'candidate_binding': _candidate_doc(binding),
        'safety': {
            'docker_image_inspect_only': True,
            'pull': False,
            'build': False,
            'network': False,
            'shell': False,
        },
        'systems': image_systems,
        'blockers': sorted(set(blockers)),
    }
    image_document['artifact_identity_sha256'] = _artifact_identity(image_document)
    toolchain_document: dict[str, Any] = {
        'schema_version': CAPTURE_SCHEMA_VERSION,
        'artifact_kind': TOOLCHAIN_ARTIFACT_KIND,
        'status': toolchain_status,
        'review_status': 'NOT_REVIEWED_EXTERNAL',
        'benchmark_eligible': False,
        'claim_eligible': False,
        'observed_at': timestamp,
        'candidate_binding': _candidate_doc(binding),
        'probe_requested': probe_toolchain,
        'safety': {
            'explicit_opt_in': probe_toolchain,
            'pull': False,
            'build': False,
            'network': 'none',
            'read_only_root': True,
            'shell': False,
        },
        'systems': toolchain_systems,
        'blockers': sorted(set(blockers)),
    }
    toolchain_document['artifact_identity_sha256'] = _artifact_identity(
        toolchain_document)
    files: list[tuple[Path, dict[str, int]]] = []

    def write_pair(
            name: str, document: Mapping[str, Any]
    ) -> tuple[str, list[tuple[Path, dict[str, int]]]]:
        path = output_dir / name
        payload = (
            json.dumps(document, indent=2, sort_keys=True, ensure_ascii=True)
            + '\n').encode('utf-8')
        identity = _exclusive_write(path, payload)
        pair = [(path, identity)]
        try:
            sidecar = path.with_name(path.name + CAPTURE_SIDECAR_SUFFIX)
            side_identity = _exclusive_write(
                sidecar, (_sha256(payload) + '  ' + path.name + '\n').encode())
            pair.append((sidecar, side_identity))
        except Exception:
            _remove_owned(path, identity, 'capture pair rollback')
            raise
        return _sha256(payload), pair

    try:
        image_sha, image_files = write_pair('image_inspect.json', image_document)
        files.extend(image_files)
        tool_sha, tool_files = write_pair('toolchain_capture.json', toolchain_document)
        files.extend(tool_files)
        manifest: dict[str, Any] = {
            'schema_version': CAPTURE_SCHEMA_VERSION,
            'manifest_kind': CAPTURE_MANIFEST_KIND,
            'status': ('UNREVIEWED_COMPLETE_OBSERVATION'
                       if image_status == 'UNREVIEWED_COMPLETE_OBSERVATION' and
                       toolchain_status == 'UNREVIEWED_COMPLETE_OBSERVATION'
                       else 'PARTIAL_OBSERVATION'),
            'review_status': 'NOT_REVIEWED_EXTERNAL',
            'benchmark_eligible': False,
            'claim_eligible': False,
            'observed_at': timestamp,
            'candidate_binding': _candidate_doc(binding),
            'safety': {
                'capture_only': True,
                'promotion': False,
                'network_used': False,
                'shell_used': False,
            },
            'image_inspect': {
                'path': 'image_inspect.json', 'sha256': image_sha,
                'status': image_status,
            },
            'toolchain_capture': {
                'path': 'toolchain_capture.json', 'sha256': tool_sha,
                'status': toolchain_status,
            },
            'blockers': sorted(set(blockers)),
        }
        manifest['capture_identity_sha256'] = _identity(
            manifest, 'capture_identity_sha256')
        _, manifest_files = write_pair('capture_manifest.json', manifest)
        files.extend(manifest_files)
    except Exception:
        for path, identity in reversed(files):
            _remove_owned(path, identity, 'capture set rollback')
        raise
    validate_capture_set(output_dir, candidate_path=candidate_path)
    return manifest


def _read_pair(directory: Path, name: str) -> tuple[dict[str, Any], str]:
    path = directory / name
    try:
        payload, _ = _read_immutable(
            path, f'capture {name}', max_bytes=MAX_HANDOFF_BYTES)
        sidecar, _ = _read_immutable(
            path.with_name(path.name + CAPTURE_SIDECAR_SUFFIX),
            f'capture {name} sidecar', max_bytes=256)
    except CandidateError as exc:
        raise CaptureError(str(exc)) from exc
    expected = (_sha256(payload) + '  ' + path.name + '\n').encode()
    if sidecar != expected:
        raise CaptureError(f'capture {name} sidecar is stale')
    try:
        final_payload, _ = _read_immutable(
            path, f'capture {name}', max_bytes=MAX_HANDOFF_BYTES)
        final_sidecar, _ = _read_immutable(
            path.with_name(path.name + CAPTURE_SIDECAR_SUFFIX),
            f'capture {name} sidecar', max_bytes=256)
    except CandidateError as exc:
        raise CaptureError(str(exc)) from exc
    if final_payload != payload or final_sidecar != sidecar:
        raise CaptureError(f'capture {name} changed during validation')
    try:
        value = json.loads(payload.decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise CaptureError(f'capture {name} JSON is invalid') from exc
    if not isinstance(value, dict):
        raise CaptureError(f'capture {name} is not an object')
    return value, _sha256(payload)


def _validate_candidate_binding(value: Any, expected: Mapping[str, Any]) -> None:
    if not isinstance(value, Mapping) or set(value) != set(expected) or \
            dict(value) != dict(expected):
        raise CaptureError('capture candidate binding is stale or swapped')


def _validate_image_document(
        value: Mapping[str, Any], expected_binding: Mapping[str, Any],
        expected_systems: Mapping[str, Any]) -> None:
    if set(value) != IMAGE_DOCUMENT_FIELDS:
        raise CaptureError('image artifact field set is not exact')
    if value['schema_version'] != CAPTURE_SCHEMA_VERSION or \
            value['artifact_kind'] != IMAGE_ARTIFACT_KIND or \
            value['review_status'] != 'NOT_REVIEWED_EXTERNAL' or \
            value['benchmark_eligible'] is not False or \
            value['claim_eligible'] is not False:
        raise CaptureError('image artifact status or schema is invalid')
    _validate_candidate_binding(value.get('candidate_binding'), expected_binding)
    _validate_system_images(value.get('systems'), expected_systems)
    safety = value.get('safety')
    if safety != {
            'docker_image_inspect_only': True, 'pull': False, 'build': False,
            'network': False, 'shell': False}:
        raise CaptureError('image safety contract is not exact')
    _validate_observed_at(value.get('observed_at'))
    if value['status'] not in {'PARTIAL_OBSERVATION',
                               'UNREVIEWED_COMPLETE_OBSERVATION'}:
        raise CaptureError('image artifact may not claim READY or reviewed status')
    if not isinstance(value.get('blockers'), list) or not value['blockers']:
        raise CaptureError('image artifact blockers are missing')
    if _artifact_identity(value) != value['artifact_identity_sha256']:
        raise CaptureError('image artifact identity mismatch')


def _validate_system_images(value: Any, expected_systems: Mapping[str, Any]) -> None:
    if not isinstance(value, Mapping) or set(value) != set(SYSTEMS):
        raise CaptureError('image system coverage is not exact')
    for system in SYSTEMS:
        item = value[system]
        if not isinstance(item, Mapping) or set(item) != {
                'status', 'expected_digest', 'command', 'returncode',
                'timed_out', 'reason', 'observed'}:
            raise CaptureError(f'image {system} shape is not exact')
        digest = expected_systems[system]['image_digest']
        if item['expected_digest'] != digest or item['command'] != _image_command(digest):
            raise CaptureError(f'image {system} command/digest drifted')
        if (isinstance(item['returncode'], bool) or
                not isinstance(item['returncode'], int) or
                not isinstance(item['timed_out'], bool)):
            raise CaptureError(f'image {system} result type is invalid')
        if item['status'] == 'OBSERVED':
            if item['returncode'] != 0 or item['timed_out'] or \
                    not isinstance(item['observed'], Mapping) or \
                    item['observed'].get('Id') != digest or item['reason'] is not None:
                raise CaptureError(f'image {system} observed result is inconsistent')
        elif item['status'] == 'PENDING':
            if not isinstance(item['reason'], str) or not item['reason'] or \
                    item['observed'] is not None and not isinstance(item['observed'], Mapping):
                raise CaptureError(f'image {system} pending result is inconsistent')
        else:
            raise CaptureError(f'image {system} has an invalid status')


def _validate_observed_at(value: Any) -> None:
    if not isinstance(value, str):
        raise CaptureError('capture timestamp is invalid')
    _timestamp(value)


def _validate_toolchain_document(
        value: Mapping[str, Any], expected_binding: Mapping[str, Any],
        expected_systems: Mapping[str, Any]) -> None:
    if set(value) != TOOLCHAIN_DOCUMENT_FIELDS:
        raise CaptureError('toolchain artifact field set is not exact')
    if value['schema_version'] != CAPTURE_SCHEMA_VERSION or \
            value['artifact_kind'] != TOOLCHAIN_ARTIFACT_KIND or \
            value['review_status'] != 'NOT_REVIEWED_EXTERNAL' or \
            value['benchmark_eligible'] is not False or \
            value['claim_eligible'] is not False:
        raise CaptureError('toolchain artifact status or schema is invalid')
    _validate_candidate_binding(value.get('candidate_binding'), expected_binding)
    requested = value.get('probe_requested')
    if not isinstance(requested, bool):
        raise CaptureError('toolchain probe_requested is not boolean')
    safety = value.get('safety')
    if not isinstance(safety, Mapping) or set(safety) != {
            'explicit_opt_in', 'pull', 'build', 'network', 'read_only_root', 'shell'} or \
            safety['explicit_opt_in'] != requested or safety['pull'] is not False or \
            safety['build'] is not False or safety['network'] != 'none' or \
            safety['read_only_root'] is not True or safety['shell'] is not False:
        raise CaptureError('toolchain safety contract is not exact')
    _validate_observed_at(value.get('observed_at'))
    systems = value.get('systems')
    if not isinstance(systems, Mapping) or set(systems) != set(SYSTEMS):
        raise CaptureError('toolchain system coverage is not exact')
    for system in SYSTEMS:
        _validate_toolchain_system(
            systems[system], expected_systems[system]['image_digest'], requested,
            NOT_APPLICABLE_FIELDS_BY_SYSTEM[system])
    if value['status'] not in {'PARTIAL_OBSERVATION',
                               'UNREVIEWED_COMPLETE_OBSERVATION'}:
        raise CaptureError('toolchain artifact may not claim READY or reviewed status')
    if not isinstance(value.get('blockers'), list) or not value['blockers']:
        raise CaptureError('toolchain artifact blockers are missing')
    if _artifact_identity(value) != value['artifact_identity_sha256']:
        raise CaptureError('toolchain artifact identity mismatch')


def _validate_toolchain_system(
        value: Any, digest: str, requested: bool,
        allowed_not_applicable: set[str] | frozenset[str]) -> None:
    if not isinstance(value, Mapping) or set(value) != {
            'status', 'probe_requested', 'image_digest', 'probes',
            'fingerprint', 'not_applicable_fields', 'reason'}:
        raise CaptureError('toolchain system shape is not exact')
    if value['image_digest'] != digest or value['probe_requested'] != requested:
        raise CaptureError('toolchain image/opt-in binding drifted')
    if value['not_applicable_fields'] != sorted(allowed_not_applicable):
        raise CaptureError('toolchain not-applicable field policy drifted')
    if not requested:
        if value['status'] != 'NOT_REQUESTED' or value['probes'] != {} or \
                value['fingerprint'] is not None:
            raise CaptureError('toolchain was observed without explicit opt-in')
        return
    if value['status'] == 'PENDING' and \
            value['reason'] == 'image_inspect_not_observed':
        if value['probes'] != {} or value['fingerprint'] is not None:
            raise CaptureError('image-pending toolchain result is inconsistent')
        return
    if not isinstance(value['probes'], Mapping) or \
            set(value['probes']) != set(TOOLCHAIN_PROBES):
        raise CaptureError('requested toolchain probe coverage is not exact')
    complete = True
    values: dict[str, str] = {}
    if value['status'] == 'PENDING':
        if not isinstance(value['reason'], str) or not value['reason']:
            raise CaptureError('pending toolchain result lacks reason')
    elif value['status'] == 'OBSERVED':
        if value['reason'] is not None or not isinstance(value['fingerprint'], str):
            raise CaptureError('complete toolchain result is inconsistent')
    else:
        raise CaptureError('toolchain status is invalid')
    for name in TOOLCHAIN_PROBES:
        probe = value['probes'][name]
        if name not in TOOLCHAIN_PROBES or not isinstance(probe, Mapping) or \
                set(probe) != {
                    'command', 'returncode', 'timed_out', 'value',
                    'stdout_sha256', 'reason'}:
            raise CaptureError('toolchain probe shape is not exact')
        if name in allowed_not_applicable:
            if probe != {
                    'command': None, 'returncode': 0, 'timed_out': False,
                    'value': 'not_applicable', 'stdout_sha256': _sha256(b''),
                    'reason': 'not_applicable'}:
                raise CaptureError(f'toolchain {name} not-applicable projection drifted')
            values[name] = 'not_applicable'
            continue
        if probe['command'] != _toolchain_command(digest, TOOLCHAIN_PROBES[name]):
            raise CaptureError(f'toolchain {name} command drifted')
        if isinstance(probe['returncode'], bool) or \
                not isinstance(probe['returncode'], int) or \
                not isinstance(probe['timed_out'], bool):
            raise CaptureError(f'toolchain {name} result type is invalid')
        if not isinstance(probe['stdout_sha256'], str) or \
                re.fullmatch(r'[0-9a-f]{64}', probe['stdout_sha256']) is None:
            raise CaptureError(f'toolchain {name} stdout hash is invalid')
        if probe['value'] is not None and not isinstance(probe['value'], str):
            raise CaptureError(f'toolchain {name} value type is invalid')
        if probe['reason'] is not None and not isinstance(probe['reason'], str):
            raise CaptureError(f'toolchain {name} reason type is invalid')
        if probe['returncode'] != 0 or probe['timed_out'] or \
                not isinstance(probe['value'], str) or not probe['value'] or \
                probe['reason'] is not None:
            complete = False
        else:
            values[name] = probe['value']
    if value['status'] == 'OBSERVED':
        if not complete or value['fingerprint'] != _sha256(_canonical(values)):
            raise CaptureError('complete toolchain fingerprint is inconsistent')
    elif complete or value['fingerprint'] is not None:
        raise CaptureError('pending toolchain result is inconsistent')


def validate_capture_set(
        output_dir: Path, *, candidate_path: Path = ROOT / CANDIDATE_REL) -> dict[str, Any]:
    output_dir = _safe_absolute(output_dir, 'capture output directory')
    _directory_identity(output_dir, 'capture output directory')
    expected_names = {
        'image_inspect.json', 'image_inspect.json.sha256',
        'toolchain_capture.json', 'toolchain_capture.json.sha256',
        'capture_manifest.json', 'capture_manifest.json.sha256'}
    actual_names = {path.name for path in output_dir.iterdir()}
    if actual_names != expected_names:
        raise CaptureError('capture output contains missing or extra files')
    candidate, candidate_info = _candidate_snapshot(candidate_path)
    del candidate
    image, image_sha = _read_pair(output_dir, 'image_inspect.json')
    toolchain, toolchain_sha = _read_pair(output_dir, 'toolchain_capture.json')
    manifest, _ = _read_pair(output_dir, 'capture_manifest.json')
    expected = candidate_info['binding']
    _validate_image_document(image, expected, candidate_info['systems'])
    _validate_toolchain_document(toolchain, expected, candidate_info['systems'])
    if set(manifest) != MANIFEST_FIELDS or \
            manifest['schema_version'] != CAPTURE_SCHEMA_VERSION or \
            manifest['manifest_kind'] != CAPTURE_MANIFEST_KIND or \
            manifest['review_status'] != 'NOT_REVIEWED_EXTERNAL' or \
            manifest['benchmark_eligible'] is not False or \
            manifest['claim_eligible'] is not False:
        raise CaptureError('capture manifest schema/status is invalid')
    _validate_candidate_binding(manifest.get('candidate_binding'), expected)
    _validate_observed_at(manifest.get('observed_at'))
    if manifest['observed_at'] != image['observed_at'] or \
            manifest['observed_at'] != toolchain['observed_at']:
        raise CaptureError('capture timestamps are not identical')
    if manifest['image_inspect'] != {
            'path': 'image_inspect.json', 'sha256': image_sha,
            'status': image['status']} or manifest['toolchain_capture'] != {
            'path': 'toolchain_capture.json', 'sha256': toolchain_sha,
            'status': toolchain['status']}:
        raise CaptureError('capture manifest artifact binding is stale')
    if manifest['status'] not in {
            'PARTIAL_OBSERVATION', 'UNREVIEWED_COMPLETE_OBSERVATION'}:
        raise CaptureError('capture manifest may not claim READY/reviewed')
    if not isinstance(manifest.get('blockers'), list) or not manifest['blockers']:
        raise CaptureError('capture manifest blockers are missing')
    safety = manifest.get('safety')
    if safety != {
            'capture_only': True, 'promotion': False,
            'network_used': False, 'shell_used': False}:
        raise CaptureError('capture manifest safety contract is not exact')
    if _identity(manifest, 'capture_identity_sha256') != \
            manifest['capture_identity_sha256']:
        raise CaptureError('capture manifest identity mismatch')
    complete = image['status'] == 'UNREVIEWED_COMPLETE_OBSERVATION' and \
        toolchain['status'] == 'UNREVIEWED_COMPLETE_OBSERVATION'
    expected_status = ('UNREVIEWED_COMPLETE_OBSERVATION'
                       if complete else 'PARTIAL_OBSERVATION')
    if manifest['status'] != expected_status:
        raise CaptureError('capture manifest status is inconsistent')
    return {
        'status': manifest['status'], 'review_status': manifest['review_status'],
        'benchmark_eligible': False, 'claim_eligible': False,
        'candidate_identity_sha256': expected['candidate_identity_sha256'],
        'image_status': image['status'], 'toolchain_status': toolchain['status'],
    }


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    capture = sub.add_parser('capture')
    capture.add_argument('--candidate', type=Path, default=ROOT / CANDIDATE_REL)
    capture.add_argument('--output-dir', type=Path, required=True)
    capture.add_argument('--probe-toolchain', action='store_true')
    validate = sub.add_parser('validate')
    validate.add_argument('--candidate', type=Path, default=ROOT / CANDIDATE_REL)
    validate.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'capture':
        result = capture_external(
            output_dir=args.output_dir, candidate_path=args.candidate,
            probe_toolchain=args.probe_toolchain)
    else:
        result = validate_capture_set(args.output_dir, candidate_path=args.candidate)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(_main())
    except (CaptureError, CandidateError, OSError, UnicodeError,
            json.JSONDecodeError, TypeError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(2)
