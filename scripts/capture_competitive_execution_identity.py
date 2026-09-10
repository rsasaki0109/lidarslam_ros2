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
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
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

"""Capture and finalize competitive execution identity without changing a receipt.

The checked-in execution-selection receipt is a reviewed contract.  This tool
creates an observation artifact beside it; it never edits the receipt and it
never turns pending values into ``ready`` by inference.  ``capture`` performs
read-only machine, worktree, environment, and locally available container
probes.  ``finalize`` verifies that the observation belongs to the current
receipt and emits a fail-closed decision.  A future operator may use that
decision to update the reviewed receipt explicitly after a clean commit and
reproducible builds; no automatic receipt mutation is provided.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
from typing import Any

import yaml

# Direct source-checkout execution puts ``scripts/`` (not the checkout root)
# on ``sys.path``.  Bootstrap only the exact canonical checkout that owns this
# file; installed package execution does not satisfy these source markers and
# therefore keeps its normal package resolution.
_SCRIPT_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if (
        (_SCRIPT_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file()
        and (_SCRIPT_SOURCE_ROOT / 'scripts' / 'benchmark_phase_contract.py').is_file()
        and str(_SCRIPT_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SCRIPT_SOURCE_ROOT))

try:
    from lidarslam_benchmark_tools.competitive_identity_hash import (
        PROFILE_CANONICAL_HASH_KIND, canonical_profile_sha256)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.competitive_identity_hash import (  # type: ignore[no-redef]
        PROFILE_CANONICAL_HASH_KIND, canonical_profile_sha256)

try:
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (
        current_rival_source_closure_identity,
        validate_rival_source_closure_receipt_identity)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (  # type: ignore[no-redef]
        current_rival_source_closure_identity,
        validate_rival_source_closure_receipt_identity)


try:
    from lidarslam_benchmark_tools import package_root
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    def package_root() -> Path:
        return Path(__file__).resolve().parents[1]


ROOT = package_root()
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
DEFAULT_RECEIPT = ROOT / (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_2026-08.yaml')
SHA256_RE = re.compile(r'^[0-9a-fA-F]{64}$')
REVISION_RE = re.compile(r'^[0-9a-fA-F]{40}$')
SYSTEMS = ('ours', 'glim', 'fast_livo2')
NOT_APPLICABLE_FIELDS_BY_SYSTEM = {
    'ours': frozenset(),
    'glim': frozenset({'pcl'}),
    'fast_livo2': frozenset(),
}
THREAD_KEYS = (
    'cpu_affinity', 'max_threads', 'omp_num_threads',
    'openblas_num_threads', 'mkl_num_threads', 'tbb_num_threads',
    'accelerator_policy')
THREAD_ENV = {
    'omp_num_threads': 'OMP_NUM_THREADS',
    'openblas_num_threads': 'OPENBLAS_NUM_THREADS',
    'mkl_num_threads': 'MKL_NUM_THREADS',
    'tbb_num_threads': 'TBB_NUM_THREADS',
}
TOOLCHAIN_COMMANDS = {
    'compiler': ['c++', '--version'],
    'linker': ['ld', '--version'],
    'ros_distro': ['bash', '-lc', 'printf %s "${ROS_DISTRO-}"'],
    'pcl': ['dpkg-query', '-W', '-f=${Version}', 'libpcl-dev'],
    'eigen': ['dpkg-query', '-W', '-f=${Version}', 'libeigen3-dev'],
    'openmp': [
        'bash', '-lc',
        "dpkg-query -W -f='${Version}' libomp-dev 2>/dev/null || "
        "dpkg-query -W -f='${Version}' libgomp1 2>/dev/null",
    ],
}
CONTAINER_TOOLCHAIN_COMMANDS = {
    # Each command is invoked with --pull=never, --network=none, and
    # --read-only by capture_container_toolchain().  Keep these as argv
    # fragments so no receipt-controlled value is interpreted by a shell.
    'compiler': (['c++', '--version']),
    'linker': (['ld', '--version']),
    'ros_distro': (['sh', '-c', 'printf %s "${ROS_DISTRO-}"']),
    'pcl': (['dpkg-query', '-W', '-f=${Version}', 'libpcl-dev']),
    'eigen': (['dpkg-query', '-W', '-f=${Version}', 'libeigen3-dev']),
    'openmp': (['sh', '-c',
                "dpkg-query -W -f='${Version}' libomp-dev 2>/dev/null || "
                "dpkg-query -W -f='${Version}' libgomp1 2>/dev/null"]),
}


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def canonical_hash(value: Any) -> str:
    return sha256_bytes(json.dumps(
        value, sort_keys=True, separators=(',', ':'), ensure_ascii=True).encode())


def run_read_only(command: list[str], *, cwd: Path | None = None) -> tuple[int, str, str]:
    """Run a bounded read-only probe and return its result without raising."""
    try:
        completed = subprocess.run(
            command, cwd=str(cwd) if cwd else None, check=False,
            capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 127, '', str(exc)
    return completed.returncode, completed.stdout.strip(), completed.stderr.strip()


def _git_probe(root: Path, args: list[str]) -> tuple[int, str, str]:
    return run_read_only(['git', '-C', str(root), *args])


def capture_git_provenance(root: Path) -> dict[str, Any]:
    revision_rc, revision, revision_error = _git_probe(root, ['rev-parse', 'HEAD'])
    status_rc, status, status_error = _git_probe(
        root, ['status', '--porcelain=v1', '--untracked-files=all'])
    diff_rc, diff, diff_error = _git_probe(root, ['diff', '--binary', '--no-ext-diff'])
    cached_rc, cached, cached_error = _git_probe(
        root, ['diff', '--cached', '--binary', '--no-ext-diff'])
    untracked_rc, untracked, untracked_error = _git_probe(
        root, ['ls-files', '--others', '--exclude-standard'])
    untracked_rows: list[dict[str, str]] = []
    for relative in sorted(path for path in untracked.splitlines() if path):
        candidate = root / relative
        if candidate.is_file():
            untracked_rows.append({'path': relative, 'sha256': sha256_file(candidate)})
        elif candidate.is_symlink():
            untracked_rows.append({'path': relative, 'symlink': os.readlink(candidate)})
        else:
            untracked_rows.append({'path': relative, 'kind': 'other'})
    tracked_diff_sha = sha256_bytes((diff + '\n' + cached).encode()) \
        if diff_rc == 0 and cached_rc == 0 else None
    untracked_sha = canonical_hash(untracked_rows) if untracked_rc == 0 else None
    dirty = bool(status) if status_rc == 0 else None
    clean_provenance = None
    if revision_rc == 0 and tracked_diff_sha and untracked_sha is not None:
        clean_provenance = canonical_hash({
            'revision': revision,
            'tracked_diff_sha256': tracked_diff_sha,
            'untracked_content_sha256': untracked_sha,
            'worktree_dirty': dirty,
        })
    errors = [item for item in (
        revision_error, status_error, diff_error, cached_error, untracked_error)
              if item]
    return {
        'revision': revision if revision_rc == 0 else None,
        'worktree_dirty': dirty,
        'tracked_diff_sha256': tracked_diff_sha,
        'untracked_content_sha256': untracked_sha,
        'clean_provenance_sha256': clean_provenance,
        'status': 'observed' if not errors else 'INCOMPLETE',
        'errors': errors,
    }


def capture_machine() -> dict[str, Any]:
    """Capture the same non-secret machine fields used by benchmark runners."""
    private_values: list[str] = []
    for path in (Path('/etc/machine-id'), Path('/sys/class/dmi/id/product_uuid'),
                 Path('/sys/class/dmi/id/board_serial')):
        try:
            value = path.read_text(encoding='utf-8').strip()
        except OSError:
            value = ''
        if value:
            private_values.append(value)
    cpu_model = ''
    try:
        for line in Path('/proc/cpuinfo').read_text(encoding='utf-8').splitlines():
            if line.lower().startswith('model name') and ':' in line:
                cpu_model = line.split(':', 1)[1].strip()
                break
    except OSError:
        pass
    memory_total_kb = None
    try:
        for line in Path('/proc/meminfo').read_text(encoding='utf-8').splitlines():
            if line.startswith('MemTotal:'):
                memory_total_kb = int(line.split()[1])
                break
    except (OSError, ValueError, IndexError):
        pass
    public = {
        'architecture': platform.machine(),
        'cpu_model': cpu_model,
        'logical_cpu_count': os.cpu_count(),
        'memory_total_kb': memory_total_kb,
    }
    machine_id = canonical_hash({'private_identifiers': private_values, **public})
    return {'machine_id': machine_id, **public}


def _positive_env(name: str) -> int | None:
    value = os.environ.get(name)
    if value is None or not value.strip():
        return None
    try:
        parsed = int(value)
    except ValueError:
        return None
    return parsed if parsed > 0 else None


def capture_thread_policy() -> dict[str, Any]:
    try:
        affinity = sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        affinity = None
    policy: dict[str, Any] = {
        'cpu_affinity': affinity,
        'max_threads': len(affinity) if affinity else None,
        **{key: _positive_env(env) for key, env in THREAD_ENV.items()},
        'accelerator_policy': os.environ.get('LIDARSLAM_ACCELERATOR_POLICY') or None,
    }
    policy['status'] = (
        'observed' if all(policy.get(key) is not None for key in THREAD_KEYS)
        else 'pending_manual_or_environment_policy')
    policy['environment'] = {
        env: os.environ.get(env) for env in THREAD_ENV.values()
    }
    policy['canonical_sha256'] = canonical_hash(
        {key: policy.get(key) for key in THREAD_KEYS})
    return policy


def capture_toolchain() -> dict[str, Any]:
    observed: dict[str, Any] = {}
    for name, command in TOOLCHAIN_COMMANDS.items():
        rc, stdout, stderr = run_read_only(command)
        observed[name] = {
            'command': command,
            'returncode': rc,
            'value': stdout if rc == 0 else None,
            'error': stderr if rc != 0 else None,
        }
    fingerprint = canonical_hash({
        name: row['value'] for name, row in sorted(observed.items())})
    complete = all(row['returncode'] == 0 and row['value'] for row in observed.values())
    return {
        'status': 'observed_host_only' if complete else 'pending_toolchain_probe',
        'fingerprint': fingerprint if complete else None,
        'observed': observed,
        'scope': 'host_observation_not_system_container_identity',
        'probe_manifest': {
            'commands': TOOLCHAIN_COMMANDS,
            'no_clone_build_or_download': True,
        },
    }


def probe_container(image_tag: Any) -> dict[str, Any]:
    if not isinstance(image_tag, str) or not image_tag:
        return {'status': 'pending_image_selection', 'image_tag': None, 'image_digest': None}
    if shutil.which('docker') is None:
        return {'status': 'pending_docker_unavailable', 'image_tag': image_tag,
                'image_digest': None}
    rc, stdout, stderr = run_read_only(
        ['docker', 'image', 'inspect', '--format', '{{.Id}}', image_tag])
    digest = stdout.strip() if rc == 0 else None
    if not isinstance(digest, str) or not re.fullmatch(r'sha256:[0-9a-fA-F]{64}', digest):
        return {
            'status': 'pending_build', 'image_tag': image_tag,
            'image_digest': None, 'probe_error': stderr or 'image not found'}
    return {
        'status': 'observed',
        'image_tag': image_tag, 'image_digest': digest,
        'probe_manifest': {
            'command': ['docker', 'image', 'inspect', '--format', '{{.Id}}', image_tag],
            'no_pull_build_or_download': True,
        },
    }


def capture_container_toolchain(
        image_tag: str, image_digest: str,
        not_applicable_fields: set[str] | None = None) -> dict[str, Any]:
    """Read toolchain identity from an already-local image, never pulling it."""
    not_applicable_fields = not_applicable_fields or set()
    unknown_fields = not_applicable_fields.difference(CONTAINER_TOOLCHAIN_COMMANDS)
    if unknown_fields:
        return {
            'status': 'pending_toolchain_probe',
            'fingerprint': None,
            'observed': None,
            'scope': 'system_container',
            'image_tag': image_tag,
            'image_digest': image_digest,
            'probe_manifest': {
                'reason': 'unknown not_applicable_fields; docker run not attempted',
                'unknown_not_applicable_fields': sorted(unknown_fields),
                'no_clone_build_or_download': True,
            },
        }
    if (not isinstance(image_tag, str) or not image_tag or
            not isinstance(image_digest, str) or
            re.fullmatch(r'sha256:[0-9a-fA-F]{64}', image_digest) is None):
        return {
            'status': 'pending_toolchain_probe',
            'fingerprint': None,
            'observed': None,
            'scope': 'system_container',
            'image_tag': image_tag,
            'image_digest': image_digest,
            'probe_manifest': {
                'reason': 'invalid local image digest binding; docker run not attempted',
                'no_clone_build_or_download': True,
            },
        }
    observed: dict[str, Any] = {}
    commands: dict[str, list[str] | None] = {}
    for name, fragment in CONTAINER_TOOLCHAIN_COMMANDS.items():
        if name in not_applicable_fields:
            commands[name] = None
            observed[name] = {
                'command': None,
                'returncode': 0,
                'value': 'not_applicable',
                'error': None,
                'status': 'not_applicable',
            }
            continue
        command = [
            'docker', 'run', '--rm', '--pull=never', '--network', 'none',
            '--read-only', '--entrypoint', fragment[0], image_digest, *fragment[1:],
        ]
        commands[name] = command
        rc, stdout, stderr = run_read_only(command)
        first_line = stdout.splitlines()[0].strip() if stdout.splitlines() else ''
        observed[name] = {
            'command': command,
            'returncode': rc,
            'value': first_line if rc == 0 and first_line else None,
            'error': stderr if rc != 0 else None,
        }
    values = {
        name: row['value'] for name, row in sorted(observed.items())
    }
    complete = all(row['returncode'] == 0 and row['value'] for row in observed.values())
    return {
        'status': 'observed' if complete else 'pending_toolchain_probe',
        'fingerprint': canonical_hash(values) if complete else None,
        'observed': observed,
        'scope': 'system_container',
        'image_tag': image_tag,
        'image_digest': image_digest,
        'not_applicable_fields': sorted(not_applicable_fields),
        'probe_manifest': {
            'commands': commands,
            'flags': ['--pull=never', '--network=none', '--read-only'],
            'no_clone_build_or_download': True,
        },
    }


def _resolve_path(root: Path, value: Any) -> Path | None:
    if not isinstance(value, str) or not value:
        return None
    path = Path(value)
    return path if path.is_absolute() else root / path


def _file_hash(root: Path, path_value: Any) -> str | None:
    if not isinstance(path_value, str) or not path_value:
        return None
    path = _resolve_path(root, path_value)
    if path is None:
        return None
    return sha256_file(path) if path.is_file() else None


def _thread_values(policy: Any) -> dict[str, Any] | None:
    if not isinstance(policy, dict) or any(key not in policy for key in THREAD_KEYS):
        return None
    return {key: policy.get(key) for key in THREAD_KEYS}


def _thread_values_valid(values: dict[str, Any] | None) -> bool:
    if values is None:
        return False
    affinity = values.get('cpu_affinity')
    if (not isinstance(affinity, list) or not affinity or
            any(isinstance(item, bool) or not isinstance(item, int) or item < 0
                for item in affinity)):
        return False
    if (isinstance(values.get('max_threads'), bool) or
            not isinstance(values.get('max_threads'), int) or
            values['max_threads'] <= 0):
        return False
    for key in THREAD_KEYS[2:6]:
        value = values.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            return False
    return isinstance(values.get('accelerator_policy'), str) and bool(
        values['accelerator_policy'])


def _record_blocker(blockers: list[str], message: str) -> None:
    if message not in blockers:
        blockers.append(message)


def compare_capture_to_receipt(
        receipt: dict[str, Any], profile: dict[str, Any], capture: dict[str, Any],
        root: Path = ROOT, *, receipt_path: Path | None = None,
        profile_path: Path | None = None) -> dict[str, Any]:
    """Compare measured observations with every reviewed identity field."""
    errors: list[str] = []
    blockers: list[str] = []
    checks: dict[str, dict[str, Any]] = {}
    receipt_ready = receipt.get('status') in {'ready', 'frozen'}

    def report_mismatch(message: str) -> None:
        # A pending reviewed contract cannot be called tampered merely because
        # today's observation differs from its unresolved placeholders.  Once
        # the contract is ready/frozen, the same difference is an invalid
        # measurement and must fail hard.
        (errors if receipt_ready else blockers).append(message)

    def check(name: str, passed: bool, evidence: Any) -> None:
        checks[name] = {'pass': bool(passed), 'evidence': evidence}

    contract = profile.get('competitive_slam_profile', profile)
    policy = contract.get('evidence_gate_v2', {})
    receipt_path = (receipt_path or _resolve_path(
        root, policy.get('execution_selection_receipt_path')))
    profile_path = profile_path or DEFAULT_PROFILE
    receipt_path = receipt_path.resolve() if receipt_path else None
    profile_path = profile_path.resolve()
    current_receipt_sha = (
        sha256_file(receipt_path) if receipt_path and receipt_path.is_file() else None)
    try:
        current_profile_sha = canonical_profile_sha256(profile)
    except ValueError as exc:
        errors.append(str(exc))
        current_profile_sha = None
    current_profile_file_sha = (
        sha256_file(profile_path) if profile_path.is_file() else None)
    source_receipt = capture.get('source_receipt', {})
    source_profile = capture.get('source_profile', {})
    captured_receipt_path = _resolve_path(root, source_receipt.get('path'))
    captured_profile_path = _resolve_path(root, source_profile.get('path'))
    path_ok = (captured_receipt_path is not None and receipt_path is not None and
               captured_receipt_path.resolve() == receipt_path.resolve() and
               captured_profile_path is not None and
               captured_profile_path.resolve() == profile_path.resolve())
    if not path_ok:
        errors.append('capture source receipt/profile path does not match explicit inputs')
    if current_receipt_sha is None:
        errors.append('current execution-selection receipt is missing')
    elif source_receipt.get('sha256') != current_receipt_sha:
        errors.append('capture receipt SHA does not match current receipt')
    if current_profile_sha is None:
        errors.append('current competitive profile is missing')
    elif source_profile.get('sha256') != current_profile_sha:
        errors.append('capture canonical profile SHA does not match current profile')
    if source_profile.get('sha256_kind') != PROFILE_CANONICAL_HASH_KIND:
        errors.append(
            'capture profile SHA kind is not '
            f'{PROFILE_CANONICAL_HASH_KIND}')
    if (current_profile_file_sha is not None and
            source_profile.get('file_sha256') != current_profile_file_sha):
        errors.append('capture profile file SHA does not match current profile file')
    declared_profile_receipt = policy.get('execution_selection_receipt_sha256')
    if (current_receipt_sha is not None and isinstance(declared_profile_receipt, str) and
            current_receipt_sha != declared_profile_receipt.lower()):
        errors.append('profile execution-selection receipt SHA does not match file')
    check('source_ownership', path_ok and current_receipt_sha is not None and
          source_receipt.get('sha256') == current_receipt_sha and
          current_profile_sha is not None and
          source_profile.get('sha256') == current_profile_sha and
          source_profile.get('sha256_kind') == PROFILE_CANONICAL_HASH_KIND and
          (current_profile_file_sha is None or
           source_profile.get('file_sha256') == current_profile_file_sha), {
              'receipt_path': str(receipt_path) if receipt_path else None,
              'profile_path': str(profile_path),
              'receipt_sha256': current_receipt_sha,
              'profile_sha256': current_profile_sha,
              'profile_sha256_kind': PROFILE_CANONICAL_HASH_KIND,
              'profile_file_sha256': current_profile_file_sha,
          })

    if receipt.get('schema_version') != 1:
        errors.append('execution-selection receipt schema_version must be 1')
    if receipt.get('receipt_kind') != 'competitive_execution_selection':
        errors.append(
            'execution-selection receipt_kind is not competitive_execution_selection')
    if receipt.get('status') not in {'ready', 'frozen'}:
        _record_blocker(blockers, f"source receipt status is {receipt.get('status')!r}")
    fresh_slots = contract.get('datasets', {}).get('fresh_holdout_slots', {})
    if isinstance(fresh_slots, dict):
        for slot_id, slot in fresh_slots.items():
            if not isinstance(slot, dict) or slot.get('status') not in {
                    'frozen_unopened', 'frozen'}:
                _record_blocker(
                    blockers,
                    f'fresh holdout slot {slot_id} is not frozen_unopened/frozen')

    common = receipt.get('common_identity', {})
    if not isinstance(common, dict):
        errors.append('receipt.common_identity must be a mapping')
        common = {}
    closure_policy = policy.get('rival_source_closure', {})
    closure_identity = None
    closure_ok = True
    if isinstance(closure_policy, dict) and closure_policy.get('required') is True:
        try:
            closure_identity = current_rival_source_closure_identity(
                {'competitive_slam_profile': contract}, root=root)
            closure_errors = validate_rival_source_closure_receipt_identity(
                receipt, closure_identity)
        except (OSError, ValueError, TypeError, UnicodeError, yaml.YAMLError) as exc:
            closure_errors = [f'current rival source closure identity is invalid: {exc}']
        if closure_errors:
            errors.extend(closure_errors)
            errors.append(
                'execution-selection receipt is not eligible: missing or '
                'mismatched rival source closure identity')
            closure_ok = False
        observed_capture_closure = capture.get('rival_source_closure')
        if observed_capture_closure != closure_identity:
            errors.append(
                'capture rival source closure identity is missing or differs '
                'from the current closure')
            closure_ok = False
    check('rival_source_closure_identity', closure_ok, {
        'required': isinstance(closure_policy, dict) and
        closure_policy.get('required') is True,
        'expected': closure_identity,
        'receipt': common.get('rival_source_closure'),
        'capture': capture.get('rival_source_closure'),
    })
    expected_profile_sha = common.get('profile_sha256')
    expected_profile_kind = common.get('profile_sha256_kind')
    profile_hash_ok = True
    if expected_profile_sha is None:
        _record_blocker(blockers, 'profile canonical SHA is unresolved')
        profile_hash_ok = False
    elif not isinstance(expected_profile_sha, str) or not SHA256_RE.fullmatch(
            expected_profile_sha):
        report_mismatch('profile canonical SHA is invalid')
        profile_hash_ok = False
    elif current_profile_sha != expected_profile_sha.lower():
        report_mismatch('profile canonical SHA mismatch')
        profile_hash_ok = False
    if expected_profile_kind is None:
        _record_blocker(blockers, 'profile canonical SHA kind is unresolved')
        profile_hash_ok = False
    elif expected_profile_kind != PROFILE_CANONICAL_HASH_KIND:
        report_mismatch('profile canonical SHA kind mismatch')
        profile_hash_ok = False
    check('profile_canonical_hash_match', profile_hash_ok, {
        'expected_sha256': expected_profile_sha,
        'observed_sha256': current_profile_sha,
        'expected_kind': expected_profile_kind,
        'required_kind': PROFILE_CANONICAL_HASH_KIND,
    })
    expected_machine = common.get('machine_fingerprint', {})
    observed_machine = capture.get('machine_fingerprint', {})
    machine_ok = True
    if not isinstance(expected_machine, dict) or not isinstance(observed_machine, dict):
        _record_blocker(blockers, 'machine fingerprint observation is missing')
        machine_ok = False
    else:
        for field in ('machine_id', 'sha256'):
            expected = expected_machine.get(field)
            observed = (observed_machine.get('machine_id') if field == 'machine_id'
                        else observed_machine.get('file_sha256'))
            valid_expected = (isinstance(expected, str) and bool(expected) and
                              (field != 'sha256' or SHA256_RE.fullmatch(expected)))
            if not valid_expected:
                _record_blocker(blockers, f'machine fingerprint expected {field} is unresolved')
                machine_ok = False
            elif observed != expected:
                report_mismatch(f'machine fingerprint {field} mismatch')
                machine_ok = False
        if expected_machine.get('status') not in {'ready', 'frozen'}:
            _record_blocker(blockers, 'machine fingerprint receipt status is not ready/frozen')
            machine_ok = False
    check('machine_fingerprint_match', machine_ok, {
        'expected': expected_machine,
        'observed': observed_machine,
    })

    expected_thread = common.get('thread_policy', {})
    observed_thread = capture.get('thread_policy', {})
    thread_ok = True
    expected_values = _thread_values(expected_thread)
    observed_values = _thread_values(observed_thread)
    if expected_values is None or observed_values is None:
        _record_blocker(blockers, 'thread policy keys are unresolved')
        thread_ok = False
    else:
        if expected_thread.get('status') not in {'ready', 'frozen'}:
            _record_blocker(blockers, 'thread policy receipt status is not ready/frozen')
            thread_ok = False
        if observed_thread.get('status') not in {'ready', 'observed'}:
            _record_blocker(blockers, 'thread policy capture status is not ready/observed')
            thread_ok = False
        if (not _thread_values_valid(expected_values) or
                not _thread_values_valid(observed_values)):
            _record_blocker(blockers, 'thread policy contains invalid or unresolved values')
            thread_ok = False
        if expected_values != observed_values:
            report_mismatch('canonical thread policy mismatch')
            thread_ok = False
        expected_thread_hash = expected_thread.get('canonical_sha256')
        if expected_thread_hash is not None:
            observed_thread_hash = canonical_hash(observed_values)
            if (not isinstance(expected_thread_hash, str) or
                    not SHA256_RE.fullmatch(expected_thread_hash)):
                _record_blocker(blockers, 'thread policy canonical SHA is invalid')
                thread_ok = False
            elif expected_thread_hash.lower() != observed_thread_hash:
                report_mismatch('thread policy canonical SHA mismatch')
                thread_ok = False
    check('thread_policy_match', thread_ok, {
        'expected': expected_values,
        'observed': observed_values,
        'expected_canonical_sha256': canonical_hash(expected_values)
        if expected_values else None,
        'observed_canonical_sha256': canonical_hash(observed_values)
        if observed_values else None,
    })

    expected_systems = receipt.get('systems', {})
    observed_systems = capture.get('systems', {})
    systems_ok = True
    system_checks: dict[str, bool] = {}
    for system in SYSTEMS:
        expected = expected_systems.get(system, {}) if isinstance(expected_systems, dict) else {}
        observed = observed_systems.get(system, {}) if isinstance(observed_systems, dict) else {}
        system_ok = True
        expected_repo = expected.get('repository', {})
        observed_repo = observed.get('repository', {})
        if not isinstance(expected_repo, dict) or not isinstance(observed_repo, dict):
            _record_blocker(blockers, f'{system} repository observation is missing')
            system_ok = False
        else:
            if expected_repo.get('revision_status') not in {'ready', 'frozen', 'pinned'}:
                _record_blocker(blockers, f'{system} revision status is not ready/frozen/pinned')
                system_ok = False
            if observed_repo.get('status') == 'INVALID':
                errors.append(f'{system} repository observation is invalid')
                system_ok = False
            if observed_repo.get('status') not in {'observed', 'ready', 'frozen'}:
                _record_blocker(blockers, f'{system} repository capture is not complete')
                system_ok = False
            for field in ('revision', 'worktree_dirty', 'tracked_diff_sha256',
                          'untracked_content_sha256', 'clean_provenance_sha256'):
                expected_value = expected_repo.get(field)
                observed_value = observed_repo.get(field)
                valid_expected = expected_value is not None
                if field == 'revision':
                    valid_expected = (isinstance(expected_value, str) and
                                      REVISION_RE.fullmatch(expected_value) is not None)
                elif field.endswith('_sha256'):
                    valid_expected = (isinstance(expected_value, str) and
                                      SHA256_RE.fullmatch(expected_value) is not None)
                elif field == 'worktree_dirty':
                    valid_expected = isinstance(expected_value, bool)
                if not valid_expected:
                    _record_blocker(blockers, f'{system} repository {field} is unresolved')
                    system_ok = False
                elif expected_value != observed_value:
                    report_mismatch(f'{system} repository {field} mismatch')
                    system_ok = False
            if observed_repo.get('worktree_dirty') is not False:
                _record_blocker(blockers, f'{system} worktree is not clean')
                system_ok = False

        expected_container = expected.get('container', {})
        observed_container = observed.get('container', {})
        if not isinstance(expected_container, dict) or not isinstance(observed_container, dict):
            _record_blocker(blockers, f'{system} container observation is missing')
            system_ok = False
            expected_container = expected_container if isinstance(expected_container, dict) else {}
            observed_container = observed_container if isinstance(observed_container, dict) else {}
        else:
            if expected_container.get('status') not in {'ready', 'frozen'}:
                _record_blocker(blockers, f'{system} container status is not ready/frozen')
                system_ok = False
            if observed_container.get('status') not in {'observed', 'ready', 'frozen'}:
                _record_blocker(blockers, f'{system} container probe is not complete')
                system_ok = False
            for field in ('image_tag', 'image_digest'):
                expected_value = expected_container.get(field)
                observed_value = observed_container.get(field)
                valid_expected = (isinstance(expected_value, str) and bool(expected_value))
                if field == 'image_digest':
                    valid_expected = (isinstance(expected_value, str) and
                                      re.fullmatch(r'sha256:[0-9a-fA-F]{64}',
                                                   expected_value) is not None)
                if not valid_expected:
                    _record_blocker(blockers, f'{system} container {field} is unresolved')
                    system_ok = False
                elif expected_value != observed_value:
                    report_mismatch(f'{system} container {field} mismatch')
                    system_ok = False

        expected_toolchain = expected.get('toolchain', {})
        observed_toolchain = observed.get('toolchain', {})
        if not isinstance(expected_toolchain, dict) or not isinstance(observed_toolchain, dict):
            _record_blocker(blockers, f'{system} toolchain observation is missing')
            system_ok = False
            expected_toolchain = expected_toolchain if isinstance(expected_toolchain, dict) else {}
            observed_toolchain = observed_toolchain if isinstance(observed_toolchain, dict) else {}
        else:
            if expected_toolchain.get('status') not in {'ready', 'frozen'}:
                _record_blocker(blockers, f'{system} toolchain status is not ready/frozen')
                system_ok = False
            if observed_toolchain.get('status') not in {'observed', 'ready', 'frozen'}:
                _record_blocker(blockers, f'{system} toolchain probe is not complete')
                system_ok = False
            expected_fingerprint = expected_toolchain.get('fingerprint')
            observed_fingerprint = observed_toolchain.get('fingerprint')
            if not isinstance(expected_fingerprint, str) or not SHA256_RE.fullmatch(
                    expected_fingerprint):
                _record_blocker(blockers, f'{system} toolchain fingerprint is unresolved')
                system_ok = False
            elif expected_fingerprint != observed_fingerprint:
                report_mismatch(f'{system} toolchain fingerprint mismatch')
                system_ok = False
            expected_fields = expected_toolchain.get('observed')
            observed_fields = observed_toolchain.get('observed')
            if (not isinstance(expected_fields, dict) or
                    any(key not in expected_fields or expected_fields[key] in (None, '')
                        for key in TOOLCHAIN_COMMANDS)):
                _record_blocker(blockers, f'{system} toolchain fields are unresolved')
                system_ok = False
            elif (not isinstance(observed_fields, dict) or
                  any(key not in observed_fields or observed_fields[key] in (None, '')
                      for key in TOOLCHAIN_COMMANDS)):
                _record_blocker(blockers, f'{system} toolchain observation fields are unresolved')
                system_ok = False
            elif expected_fields != observed_fields:
                report_mismatch(
                    f'{system} toolchain compiler/linker/ROS/PCL/Eigen/OpenMP mismatch')
                system_ok = False
            expected_toolchain_digest = expected_toolchain.get('image_digest')
            observed_toolchain_digest = observed_toolchain.get('image_digest')
            expected_container_digest = expected_container.get('image_digest')
            expected_scope = expected_toolchain.get('scope', 'system_container')
            if not isinstance(expected_scope, str) or not expected_scope:
                _record_blocker(blockers, f'{system} toolchain scope is unresolved')
                system_ok = False
            elif observed_toolchain.get('scope') != expected_scope:
                _record_blocker(
                    blockers,
                    f'{system} toolchain observation scope does not match {expected_scope}')
                system_ok = False
            elif expected_scope == 'system_container':
                if (not isinstance(expected_toolchain_digest, str) or
                        not re.fullmatch(r'sha256:[0-9a-fA-F]{64}',
                                         expected_toolchain_digest)):
                    _record_blocker(
                        blockers,
                        f'{system} toolchain image digest is unresolved')
                    system_ok = False
                elif expected_toolchain_digest != expected_container_digest:
                    report_mismatch(
                        f'{system} toolchain image digest does not match container')
                    system_ok = False
                if observed_toolchain_digest != observed_container.get('image_digest'):
                    report_mismatch(
                        f'{system} observed toolchain image digest does not match container')
                    system_ok = False
        system_checks[system] = system_ok
        systems_ok = systems_ok and system_ok
    check('systems_identity_match', systems_ok, {'per_system': system_checks})
    status = 'INVALID' if errors else ('INCOMPLETE' if blockers else 'PASS')
    return {
        'status': status,
        'pass': status == 'PASS',
        'errors': errors,
        'blockers': sorted(set(blockers)),
        'checks': checks,
        'current_receipt_sha256': current_receipt_sha,
        'current_profile_sha256': current_profile_sha,
        'thread_policy_canonical_sha256': (
            canonical_hash(observed_values) if observed_values else None),
    }


def capture_identity(receipt: dict[str, Any], profile: dict[str, Any],
                     root: Path = ROOT, *, receipt_path: Path | None = None,
                     profile_path: Path | None = None,
                     system_sources: dict[str, Path] | None = None,
                     image_overrides: dict[str, str] | None = None) -> dict[str, Any]:
    """Capture observations while preserving all reviewed receipt decisions."""
    receipt_bytes = json.dumps(receipt, sort_keys=True, separators=(',', ':')).encode()
    root = root.resolve()
    policy = profile.get('competitive_slam_profile', profile).get('evidence_gate_v2', {})
    receipt_path = (receipt_path or _resolve_path(
        root, policy.get('execution_selection_receipt_path')) or DEFAULT_RECEIPT).resolve()
    profile_path = (profile_path or DEFAULT_PROFILE).resolve()
    system_sources = dict(system_sources or {})
    if 'ours' not in system_sources:
        system_sources['ours'] = root
    image_overrides = dict(image_overrides or {})
    closure_identity = None
    closure_policy = policy.get('rival_source_closure', {})
    if isinstance(closure_policy, dict) and closure_policy.get('required') is True:
        try:
            closure_identity = current_rival_source_closure_identity(
                {'competitive_slam_profile': profile.get(
                    'competitive_slam_profile', profile)}, root=root)
        except (OSError, ValueError, TypeError, UnicodeError, yaml.YAMLError):
            closure_identity = None
    machine = capture_machine()
    machine_path = _resolve_path(
        root, receipt.get('common_identity', {}).get('machine_fingerprint', {}).get('path'))
    machine['file_sha256'] = (
        sha256_file(machine_path)
        if machine_path and machine_path.is_file() else None)
    thread = capture_thread_policy()
    systems: dict[str, Any] = {}
    receipt_systems = receipt.get('systems', {})
    for system in SYSTEMS:
        item = receipt_systems.get(system, {}) if isinstance(receipt_systems, dict) else {}
        repository = item.get('repository', {}) if isinstance(item, dict) else {}
        container = item.get('container', {}) if isinstance(item, dict) else {}
        source_path = system_sources.get(system)
        if source_path is not None:
            source_path = Path(source_path).resolve()
            repository_observation = capture_git_provenance(source_path)
            repository_observation['source_path'] = str(source_path)
            expected_revision = repository.get('revision')
            repository_observation['revision_expected'] = expected_revision
            repository_observation['revision_match'] = (
                expected_revision is not None and
                repository_observation.get('revision') == expected_revision)
            if not repository_observation['revision_match']:
                repository_observation.setdefault('warnings', []).append(
                    f'{system} source revision does not match receipt; review required')
        else:
            repository_observation = {
                'revision': repository.get('revision'),
                'worktree_dirty': None,
                'status': 'pending_external_source_checkout',
            }
        image_tag = image_overrides.get(system, container.get('image_tag'))
        container_observation = probe_container(image_tag)
        if (container_observation.get('status') in {'observed', 'ready', 'frozen'} and
                isinstance(container_observation.get('image_tag'), str) and
                isinstance(container_observation.get('image_digest'), str)):
            expected_toolchain = item.get('toolchain', {}) \
                if isinstance(item, dict) else {}
            not_applicable_fields = set(
                expected_toolchain.get('not_applicable_fields', [])) \
                if isinstance(expected_toolchain, dict) else set()
            forbidden_not_applicable = not_applicable_fields.difference(
                NOT_APPLICABLE_FIELDS_BY_SYSTEM[system])
            if forbidden_not_applicable:
                toolchain_observation = {
                    'status': 'INVALID',
                    'fingerprint': None,
                    'observed': None,
                    'scope': 'system_container',
                    'image_tag': container_observation['image_tag'],
                    'image_digest': container_observation['image_digest'],
                    'probe_manifest': {
                        'reason': 'not_applicable field is not allowed for system',
                        'forbidden_not_applicable_fields': sorted(
                            forbidden_not_applicable),
                        'docker_run_attempted': False,
                        'no_clone_build_or_download': True,
                    },
                }
            elif not_applicable_fields:
                toolchain_observation = capture_container_toolchain(
                    container_observation['image_tag'],
                    container_observation['image_digest'],
                    not_applicable_fields)
            else:
                toolchain_observation = capture_container_toolchain(
                    container_observation['image_tag'],
                    container_observation['image_digest'])
        else:
            toolchain_observation = {
                'status': 'pending_system_container_probe',
                'fingerprint': None,
                'observed': None,
                'scope': 'system_container_probe_not_run',
                'probe_manifest': {
                    'required_commands': CONTAINER_TOOLCHAIN_COMMANDS,
                    'expected_scope': 'system_container',
                    'reason': (
                        'no complete local image observation; source bindings only provide '
                        'Git provenance; no clone/build/download performed'),
                },
            }
        systems[system] = {
            'repository': repository_observation,
            'container': container_observation,
            'toolchain': toolchain_observation,
            'receipt_statuses': {
                'revision_status': repository.get('revision_status'),
                'container_status': container.get('status'),
                'toolchain_status': (item.get('toolchain', {}).get('status')
                                     if isinstance(item, dict) else None),
            },
        }
    capture = {
        'schema_version': 1,
        'receipt_kind': 'competitive_execution_identity_capture',
        'rival_source_closure': closure_identity,
        'status': 'pending_observation',
        'pass': False,
        'captured_at': dt.datetime.now(dt.timezone.utc).isoformat(),
        'source_receipt': {
            'path': str(receipt_path.resolve()),
            'sha256': sha256_file(receipt_path) if receipt_path.is_file() else None,
            'declared_status': receipt.get('status'),
            'canonical_input_sha256': sha256_bytes(receipt_bytes),
        },
        'source_profile': {
            'path': str(profile_path.resolve()),
            'sha256': canonical_profile_sha256(profile),
            'sha256_kind': PROFILE_CANONICAL_HASH_KIND,
            'file_sha256': sha256_file(profile_path) if profile_path.is_file() else None,
        },
        'machine_fingerprint': machine,
        'thread_policy': thread,
        'systems': systems,
        'fresh_data_access': {
            'downloaded': False,
            'ground_truth_opened': False,
            'raw_bag_opened': False,
        },
        'receipt_mutation': {
            'performed': False,
            'ready_or_frozen_inferred': False,
        },
    }
    comparison = compare_capture_to_receipt(
        receipt, profile, capture, root, receipt_path=receipt_path,
        profile_path=profile_path)
    capture.update({
        'status': comparison['status'],
        'pass': comparison['pass'],
        'errors': comparison['errors'],
        'blockers': comparison['blockers'],
        'checks': comparison['checks'],
        'current_receipt_sha256': comparison['current_receipt_sha256'],
        'current_profile_sha256': comparison['current_profile_sha256'],
        'thread_policy_canonical_sha256': comparison[
            'thread_policy_canonical_sha256'],
    })
    return capture


def finalize_identity(
        receipt: dict[str, Any], profile: dict[str, Any], capture: dict[str, Any],
        root: Path = ROOT, *, receipt_path: Path | None = None,
        profile_path: Path | None = None) -> dict[str, Any]:
    """Finalize only an observation that still matches every source value.

    This is deliberately a verifier, not a promoter: it never edits the
    reviewed receipt.  In particular, a ``PASS`` result is possible for a
    complete synthetic/clean capture, while the checked-in pending receipt
    remains ``INCOMPLETE`` until an operator explicitly updates it.
    """
    root = root.resolve()
    if receipt_path is None:
        policy = profile.get('competitive_slam_profile', profile).get(
            'evidence_gate_v2', {})
        receipt_path = _resolve_path(
            root, policy.get('execution_selection_receipt_path')) or DEFAULT_RECEIPT
    receipt_path = receipt_path.resolve()
    profile_path = (profile_path or DEFAULT_PROFILE).resolve()
    comparison = compare_capture_to_receipt(
        receipt, profile, capture, root, receipt_path=receipt_path,
        profile_path=profile_path)
    errors = list(comparison.get('errors', []))
    blockers = list(comparison.get('blockers', []))
    captured_receipt_sha = capture.get('source_receipt', {}).get('sha256')
    current_receipt_sha = comparison.get('current_receipt_sha256')
    if (current_receipt_sha is None or captured_receipt_sha != current_receipt_sha):
        errors.append('capture belongs to a different execution-selection receipt')
    captured_profile_sha = capture.get('source_profile', {}).get('sha256')
    current_profile_sha = comparison.get('current_profile_sha256')
    if current_profile_sha is None or captured_profile_sha != current_profile_sha:
        errors.append('capture belongs to a different competitive profile')
    if capture.get('receipt_mutation', {}).get('performed') is True:
        errors.append('capture artifact claims receipt mutation; refusing finalize')
    fresh_access = capture.get('fresh_data_access', {})
    for field, label in (
            ('raw_bag_opened', 'raw bags'),
            ('ground_truth_opened', 'ground truth')):
        if fresh_access.get(field) is not False:
            errors.append(f'capture must not open {label}')
    if capture.get('status') != 'PASS':
        _record_blocker(blockers, 'capture status is not PASS; no values may be promoted')

    status = 'INVALID' if errors else ('INCOMPLETE' if blockers else 'PASS')
    return {
        'schema_version': 1,
        'receipt_kind': 'competitive_execution_identity_finalize_result',
        'status': status,
        'pass': status == 'PASS',
        'errors': sorted(set(errors)),
        'blockers': sorted(set(blockers)),
        'checks': comparison.get('checks', {}),
        'source_receipt_sha256': current_receipt_sha,
        'capture_receipt_sha256': captured_receipt_sha,
        'source_profile_sha256': current_profile_sha,
        'capture_profile_sha256': captured_profile_sha,
        'rival_source_closure': capture.get('rival_source_closure'),
        'thread_policy_canonical_sha256': comparison.get(
            'thread_policy_canonical_sha256'),
        'receipt_mutation': {'performed': False, 'automatic_promotion': False},
        'fresh_data_access': {
            'downloaded': False,
            'ground_truth_opened': False,
            'raw_bag_opened': False,
        },
    }


def _load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'{path} must contain a YAML mapping')
    return value


def _parse_bindings(values: list[str], option: str, *, allow_paths: bool) -> dict[str, Any]:
    """Parse repeatable ``system=value`` CLI bindings without touching inputs."""
    bindings: dict[str, Any] = {}
    for value in values:
        if '=' not in value:
            raise ValueError(f'{option} expects SYSTEM=VALUE, got {value!r}')
        system, bound = value.split('=', 1)
        if system not in SYSTEMS or not bound:
            raise ValueError(
                f'{option} system must be one of {", ".join(SYSTEMS)} and value nonempty')
        if system in bindings:
            raise ValueError(f'duplicate {option} binding for {system}')
        bindings[system] = Path(bound) if allow_paths else bound
    return bindings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('capture', 'finalize'))
    parser.add_argument('--receipt', type=Path, default=DEFAULT_RECEIPT)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--capture', type=Path,
                        help='capture YAML/JSON used by finalize mode')
    parser.add_argument(
        '--source', action='append', default=[], metavar='SYSTEM=PATH',
        help='optional local source checkout binding; no clone/build is performed')
    parser.add_argument(
        '--image', action='append', default=[], metavar='SYSTEM=TAG',
        help='optional local container image tag for read-only docker inspect')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--yaml-output', type=Path)
    args = parser.parse_args()
    receipt_path = args.receipt.resolve()
    profile_path = args.profile.resolve()
    receipt = _load_yaml(receipt_path)
    profile = _load_yaml(profile_path)
    source_bindings = _parse_bindings(args.source, '--source', allow_paths=True)
    image_bindings = _parse_bindings(args.image, '--image', allow_paths=False)
    if args.mode == 'capture':
        result = capture_identity(
            receipt, profile, receipt_path=receipt_path, profile_path=profile_path,
            system_sources=source_bindings, image_overrides=image_bindings)
    else:
        if args.capture is None:
            parser.error('--capture is required for finalize mode')
        result = finalize_identity(
            receipt, profile, _load_yaml(args.capture), receipt_path=receipt_path,
            profile_path=profile_path)
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n',
                           encoding='utf-8')
    if args.yaml_output:
        if args.yaml_output.exists():
            raise ValueError(f'refusing to overwrite: {args.yaml_output}')
        args.yaml_output.parent.mkdir(parents=True, exist_ok=True)
        args.yaml_output.write_text(yaml.safe_dump(result, sort_keys=True),
                                    encoding='utf-8')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError,
            json.JSONDecodeError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
