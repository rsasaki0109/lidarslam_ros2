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

"""Fail-closed preflight for the competitive benchmark execution identity.

This checker only validates the preregistration receipt.  It does not build
containers, download datasets, inspect ground truth, or run a benchmark.
Missing values are reported as ``INCOMPLETE``; malformed or mismatched values
are ``INVALID``.  A pending receipt can therefore be committed before the
environment is built without being mistaken for a runnable comparison.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Any

import yaml

# Direct source-checkout execution puts ``scripts/`` (not the checkout root)
# on ``sys.path``.  Bootstrap only the exact canonical checkout that owns this
# file; installed package execution keeps normal package resolution.
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
        validate_rival_source_closure_receipt_identity,
        verify_rival_source_closure)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (
        current_rival_source_closure_identity,
        validate_rival_source_closure_receipt_identity,
        verify_rival_source_closure)

try:
    from lidarslam_benchmark_tools.check_competitive_dataset_source_closure import (
        verify_dataset_source_closure)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.check_competitive_dataset_source_closure import (
        verify_dataset_source_closure)


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
COMMIT_RE = re.compile(r'^[0-9a-fA-F]{40}$')
CONTAINER_DIGEST_RE = re.compile(r'^sha256:[0-9a-fA-F]{64}$')
THREAD_KEYS = (
    'cpu_affinity', 'max_threads', 'omp_num_threads',
    'openblas_num_threads', 'mkl_num_threads', 'tbb_num_threads',
    'accelerator_policy')
SYSTEMS = ('ours', 'glim', 'fast_livo2')
M6A5_MEMORY_VERSION = 'm6a5-cgroup-v2-memory-v1'
M6A5_MEMORY_SCOPE = 'container_cgroup_v2'
M6A5_MEMORY_PRODUCER_FILES = {
    'helper': 'scripts/container_memory_evidence.sh',
    'ours_wrapper': 'scripts/ours_container_gt_blind_run.sh',
    'glim_wrapper': 'scripts/glim_container_run.sh',
    'fast_wrapper': 'scripts/fast_livo2_container_run.sh',
    'driver': 'scripts/run_competitive_gt_blind_benchmark.py',
}
M6A7_PROCESS_RSS_VERSION = 'm6a7-container-process-rss-v1'
M6A7_PROCESS_RSS_SCOPE = 'container_pid_namespace_proc_status'
M6A7_PROCESS_RSS_METRIC = 'aggregate_process_tree_peak_rss_bytes'
M6A7_PROCESS_RSS_DEFINITION = (
    'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted')
M6A7_STATIC_KEYS = (
    'schema_version', 'measurement_version', 'measurement_scope',
    'primary_metric', 'primary_metric_definition', 'sampler_interval_ms',
    'sampler_scheduler_nice', 'memory_max', 'oom_delta_required',
    'docker_client_comparable', 'prior_failed_audit_lineage_required')
M6A7_FAILED_AUDIT_ROOTS = (
    '/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a7_20260822/'
    'v3_final_audit_20260822',
    '/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a7_20260822/'
    'v3_final_audit_retry_20260822',
    '/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a7_20260822/'
    'v3_final_audit_pass_20260822',
    '/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a7_20260822/'
    'v3_final_audit_pass2_20260822',
    '/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a7_20260822/'
    'v3_final_audit_tool_20260822',
    '/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a7_20260822/'
    'v3_final_audit_tool_retry_20260822')


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sha256_tree(path: Path) -> str:
    """Hash relative file names and file bytes using the repository contract."""
    digest = hashlib.sha256()
    for candidate in sorted(item for item in path.rglob('*') if item.is_file()):
        digest.update(candidate.relative_to(path).as_posix().encode())
        digest.update(b'\0')
        with candidate.open('rb') as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


def _resolve(root: Path, value: Any) -> Path | None:
    if not isinstance(value, str) or not value:
        return None
    path = Path(value)
    return path if path.is_absolute() else root / path


def _hash_path(root: Path, value: Any) -> str | None:
    path = _resolve(root, value)
    if path is None or not path.exists():
        return None
    return sha256_tree(path) if path.is_dir() else sha256_file(path)


def _is_sha(value: Any) -> bool:
    return isinstance(value, str) and SHA256_RE.fullmatch(value) is not None


def _is_sha256_digest(value: Any) -> bool:
    return isinstance(value, str) and CONTAINER_DIGEST_RE.fullmatch(value) is not None


def _nonempty(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _canonical_hash(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(',', ':'),
                         ensure_ascii=True).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _add_missing(incomplete: list[str], path: str) -> None:
    incomplete.append(path)


def _check_hash_file(root: Path, value: Any, label: str,
                     errors: list[str], incomplete: list[str]) -> bool:
    path = _resolve(root, value.get('path') if isinstance(value, dict) else None)
    expected = value.get('sha256') if isinstance(value, dict) else None
    if path is None or expected is None:
        _add_missing(incomplete, f'{label}.path_or_sha256')
        return False
    if not _is_sha(expected):
        errors.append(f'{label}.sha256 must be a 64-hex SHA-256')
        return False
    if not path.is_file():
        errors.append(f'{label}.path is not an existing file: {path}')
        return False
    actual = sha256_file(path)
    if actual.lower() != expected.lower():
        errors.append(f'{label}.sha256 does not match {path}')
        return False
    return True


def _check_config(root: Path, config: Any, label: str,
                  errors: list[str], incomplete: list[str], *,
                  container_digest: Any = None) -> bool:
    if not isinstance(config, dict):
        errors.append(f'{label} must be a mapping')
        return False
    expected = config.get('sha256')
    if expected is None:
        _add_missing(incomplete, f'{label}.path_or_sha256')
        return False
    if not _is_sha(expected):
        errors.append(f'{label}.sha256 must be a 64-hex SHA-256')
        return False
    hash_kind = config.get('hash_kind', 'file_sha256')
    if hash_kind == 'external_container_file_sha256':
        path_value = config.get('path')
        valid = True
        if (not isinstance(path_value, str) or not path_value or
                not Path(path_value).is_absolute()):
            errors.append(f'{label}.path must be an absolute container path')
            valid = False
        if config.get('path_kind') != 'external_container_absolute_path':
            errors.append(
                f'{label}.path_kind must be external_container_absolute_path')
            valid = False
        status = config.get('status')
        if status not in {'observed', 'ready', 'frozen'}:
            if status is None or 'pending' in str(status):
                _add_missing(incomplete, f'{label}.status is not observed')
            else:
                errors.append(f'{label}.status must be observed/ready/frozen')
            valid = False
        observed_digest = config.get('container_image_digest')
        if not _is_sha256_digest(observed_digest):
            _add_missing(incomplete, f'{label}.container_image_digest')
            valid = False
        elif not _is_sha256_digest(container_digest):
            _add_missing(incomplete, f'{label}.container_digest_binding')
            valid = False
        elif observed_digest != container_digest:
            errors.append(f'{label}.container_image_digest does not match image')
            valid = False
        return valid
    path = _resolve(root, config.get('path'))
    if path is None:
        _add_missing(incomplete, f'{label}.path_or_sha256')
        return False
    if not path.exists():
        errors.append(f'{label}.path does not exist: {path}')
        return False
    actual = sha256_tree(path) if path.is_dir() else sha256_file(path)
    if actual.lower() != expected.lower():
        errors.append(f'{label}.sha256 does not match {path}')
        return False
    return True


def _check_thread_policy(value: Any, label: str, errors: list[str],
                         incomplete: list[str]) -> bool:
    if not isinstance(value, dict):
        _add_missing(incomplete, f'{label} mapping')
        return False
    valid = True
    for key in THREAD_KEYS:
        if key not in value or value[key] is None:
            _add_missing(incomplete, f'{label}.{key}')
            valid = False
            continue
        item = value[key]
        if key == 'cpu_affinity':
            if (not isinstance(item, list) or not item or
                    any(isinstance(cpu, bool) or not isinstance(cpu, int)
                        or cpu < 0 for cpu in item)):
                errors.append(f'{label}.{key} must be a non-empty integer list')
                valid = False
        elif key == 'accelerator_policy':
            if not _nonempty(item):
                errors.append(f'{label}.{key} must be a non-empty string')
                valid = False
        elif isinstance(item, bool) or not isinstance(item, int) or item <= 0:
            errors.append(f'{label}.{key} must be a positive integer')
            valid = False
    return valid


def _check_m6a5_memory_contract(receipt: dict[str, Any], root: Path,
                                errors: list[str],
                                incomplete: list[str]) -> tuple[bool, dict[str, Any]]:
    """Validate cgroup-memory evidence and immutable campaign lineage."""
    contract = receipt.get('m6a5_memory_contract')
    if not isinstance(contract, dict):
        _add_missing(incomplete, 'receipt.m6a5_memory_contract')
        return False, {'status': 'missing'}
    valid = True

    def require_equal(key: str, expected: Any) -> None:
        nonlocal valid
        actual = contract.get(key)
        if actual != expected:
            if actual is None:
                _add_missing(incomplete, f'receipt.m6a5_memory_contract.{key}')
            else:
                errors.append(
                    f'receipt.m6a5_memory_contract.{key} must be {expected!r}')
            valid = False

    require_equal('schema_version', 1)
    require_equal('measurement_version', M6A5_MEMORY_VERSION)
    require_equal('measurement_scope', M6A5_MEMORY_SCOPE)
    require_equal('children_included', True)
    require_equal('atomic_output', True)
    require_equal('readability_scope', 'OUT_DIR_only')
    require_equal('peak_field', 'container_cgroup_peak_bytes')
    require_equal('comparative_rss_field', 'container_cgroup_peak_bytes')
    require_equal('docker_client_diagnostic_field', 'docker_client_peak_rss_kb')
    require_equal('docker_client_comparable', False)
    require_equal('memory_max_unlimited_valid', True)

    producer_files = contract.get('producer_files')
    if not isinstance(producer_files, dict):
        _add_missing(incomplete, 'receipt.m6a5_memory_contract.producer_files')
        valid = False
    else:
        for name, expected_path in M6A5_MEMORY_PRODUCER_FILES.items():
            item = producer_files.get(name)
            if not isinstance(item, dict):
                _add_missing(incomplete, f'memory producer {name}')
                valid = False
                continue
            if item.get('path') != expected_path:
                errors.append(f'memory producer {name} path is not {expected_path}')
                valid = False
            valid = _check_hash_file(root, item, f'memory producer {name}',
                                     errors, incomplete) and valid

    summary = contract.get('smoke_summary')
    valid = _check_hash_file(root, summary, 'm6a5_memory_smoke_summary',
                             errors, incomplete) and valid
    if isinstance(summary, dict) and summary.get('status') != 'pass':
        errors.append('m6a5_memory_smoke_summary.status must be pass')
        valid = False
    delta = contract.get('known_allocation_peak_delta_bytes')
    if isinstance(delta, bool) or not isinstance(delta, int) or delta <= 0:
        if delta is None:
            _add_missing(
                incomplete,
                'receipt.m6a5_memory_contract.known_allocation_peak_delta_bytes')
        else:
            errors.append(
                'm6a5_memory_contract.known_allocation_peak_delta_bytes must be positive')
        valid = False

    lineage = contract.get('campaign_lineage')
    if not isinstance(lineage, dict):
        _add_missing(incomplete, 'receipt.m6a5_memory_contract.campaign_lineage')
        valid = False
        lineage = {}
    for name in ('campaign1_completion', 'campaign2_partial'):
        item = lineage.get(name)
        lineage_ok = _check_hash_file(
            root, item, f'm6a5 campaign lineage {name}', errors, incomplete)
        if not isinstance(item, dict) or item.get('immutable') is not True:
            errors.append(f'm6a5 campaign lineage {name} must be immutable')
            lineage_ok = False
        if isinstance(item, dict) and item.get('status') != 'INCOMPLETE':
            errors.append(f'm6a5 campaign lineage {name} must remain INCOMPLETE')
            lineage_ok = False
        valid = lineage_ok and valid
    return valid, {
        'measurement_version': contract.get('measurement_version'),
        'measurement_scope': contract.get('measurement_scope'),
        'comparative_rss_field': contract.get('comparative_rss_field'),
        'docker_client_comparable': contract.get('docker_client_comparable'),
        'smoke_summary_sha256': summary.get('sha256')
        if isinstance(summary, dict) else None,
        'known_allocation_peak_delta_bytes': delta,
        'lineage': lineage,
    }


def _load_json_file(path: Path, label: str, errors: list[str],
                    incomplete: list[str]) -> dict[str, Any] | None:
    """Read a machine receipt without interpreting any benchmark input."""
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        errors.append(f'{label} is not valid JSON: {exc}')
        return None
    if not isinstance(value, dict):
        errors.append(f'{label} must contain a JSON object')
        return None
    return value


def _check_m6a7_process_rss_contract(
        receipt: dict[str, Any], profile: dict[str, Any], root: Path,
        errors: list[str], incomplete: list[str]) -> tuple[bool, dict[str, Any]]:
    """Validate the audited process-tree RSS contract used by campaign4.

    The audit and normalized receipt are external machine artifacts.  They
    contain only run metadata and aggregate measurements; this function never
    opens a frozen bag, calibration file, or ground-truth path.
    """
    contract = receipt.get('m6a7_process_rss_contract')
    profile_contract = profile.get('competitive_slam_profile', profile).get(
        'evidence_gate_v2', {}).get('m6a7_process_rss_contract')
    if not isinstance(contract, dict):
        _add_missing(incomplete, 'receipt.m6a7_process_rss_contract')
        return False, {'status': 'missing'}
    if not isinstance(profile_contract, dict):
        _add_missing(incomplete, 'profile.evidence_gate_v2.m6a7_process_rss_contract')
        return False, {'status': 'profile_contract_missing'}
    valid = True
    for key in M6A7_STATIC_KEYS:
        expected = profile_contract.get(key)
        actual = contract.get(key)
        if actual != expected:
            if actual is None:
                _add_missing(incomplete, f'receipt.m6a7_process_rss_contract.{key}')
            else:
                errors.append(
                    f'receipt.m6a7_process_rss_contract.{key} does not match profile')
            valid = False
    expected_values = {
        'schema_version': 1,
        'measurement_version': M6A7_PROCESS_RSS_VERSION,
        'measurement_scope': M6A7_PROCESS_RSS_SCOPE,
        'primary_metric': M6A7_PROCESS_RSS_METRIC,
        'primary_metric_definition': M6A7_PROCESS_RSS_DEFINITION,
        'sampler_interval_ms': 250,
        'sampler_scheduler_nice': 10,
        'memory_max': 'max',
        'oom_delta_required': 0,
        'docker_client_comparable': False,
        'prior_failed_audit_lineage_required': True,
    }
    for key, expected in expected_values.items():
        if contract.get(key) != expected:
            errors.append(f'm6a7_process_rss_contract.{key} must be {expected!r}')
            valid = False
    if contract.get('status') != 'PASS':
        errors.append('m6a7_process_rss_contract.status must be PASS')
        valid = False

    def artifact(name: str) -> tuple[dict[str, Any] | None, Path | None]:
        nonlocal valid
        item = contract.get(name)
        if not isinstance(item, dict):
            _add_missing(incomplete, f'receipt.m6a7_process_rss_contract.{name}')
            valid = False
            return None, None
        path = _resolve(root, item.get('path'))
        expected_sha = item.get('sha256')
        if path is None or not _is_sha(expected_sha):
            if path is None or expected_sha is None:
                _add_missing(incomplete,
                             f'receipt.m6a7_process_rss_contract.{name}.path_or_sha256')
            else:
                errors.append(f'm6a7 {name}.sha256 must be a 64-hex SHA-256')
            valid = False
            return item, path
        if not path.is_file() or sha256_file(path).lower() != expected_sha.lower():
            errors.append(f'm6a7 {name}.sha256 does not match {path}')
            valid = False
        return item, path

    audit_ref, audit_path = artifact('final_audit')
    receipt_ref, receipt_path = artifact('final_receipt')
    summary_ref, summary_path = artifact('summary')
    audit = _load_json_file(audit_path, 'm6a7 final audit', errors, incomplete) \
        if audit_path is not None and audit_path.is_file() else None
    audited_receipt = _load_json_file(
        receipt_path, 'm6a7 final receipt', errors, incomplete) \
        if receipt_path is not None and receipt_path.is_file() else None
    summary = _load_json_file(summary_path, 'm6a7 summary', errors, incomplete) \
        if summary_path is not None and summary_path.is_file() else None
    if audit is not None:
        if audit.get('status') != 'PASS' or audit.get('campaign4_started') is not False:
            errors.append('m6a7 final audit is not PASS or records campaign4')
            valid = False
        if audit.get('gt_accessed') is not False or audit.get('scorer_accessed') is not False:
            errors.append('m6a7 final audit is not GT/scorer blind')
            valid = False
        schedule = audit.get('schedule')
        if not isinstance(schedule, dict) or schedule.get('all_complete') is not True or \
                schedule.get('pairs') != 20 or schedule.get('runs') != 40 or \
                schedule.get('expected_order') != schedule.get('observed_order') or \
                len(schedule.get('observed_order', [])) != 40:
            errors.append('m6a7 final audit schedule is incomplete or mismatched')
            valid = False
        if audit.get('read_only_inputs') is not True or \
                audit.get('machine_checks') != {
                    'docker_empty': True, 'sampler_empty': True}:
            errors.append('m6a7 final audit machine/read-only contract is invalid')
            valid = False
        rows = schedule.get('rows', []) if isinstance(schedule, dict) else []
        if not isinstance(rows, list) or len(rows) != 40:
            errors.append('m6a7 final audit row count is not 40')
            valid = False
        else:
            for row in rows:
                command = row.get('command') if isinstance(row, dict) else None
                if not isinstance(row, dict) or row.get('docker_exit_status') != 0 or \
                        row.get('host_time_exit_status') != 0 or \
                        not isinstance(command, dict) or \
                        command.get('network_none') is not True or \
                        command.get('memory_cap') is not False or \
                        command.get('external_network') is not False or \
                        command.get('cpuset') != '0-7':
                    errors.append('m6a7 final audit contains an invalid run row')
                    valid = False
        if audit.get('receipt_sha256') is not None and receipt_ref is not None and \
                audit.get('receipt_sha256') != receipt_ref.get('sha256'):
            errors.append('m6a7 final audit receipt SHA binding is invalid')
            valid = False
    if audited_receipt is not None:
        if audited_receipt.get('status') != 'PASS' or \
                audited_receipt.get('campaign4_started') is not False or \
                audited_receipt.get('gt_accessed') is not False or \
                audited_receipt.get('scorer_accessed') is not False:
            errors.append('m6a7 normalized receipt is not PASS/blind')
            valid = False
        contract_data = audited_receipt.get('contract')
        if not isinstance(contract_data, dict) or \
                contract_data.get('aggregate_rss_definition') != M6A7_PROCESS_RSS_DEFINITION or \
                contract_data.get('memory_max') != 'max' or \
                contract_data.get('docker_client_rss_comparable') is not False:
            errors.append('m6a7 normalized receipt metric contract is invalid')
            valid = False
        if audit_ref is not None and audited_receipt.get('audit_sha256') != \
                audit_ref.get('sha256'):
            errors.append('m6a7 normalized receipt audit SHA binding is invalid')
            valid = False
    if summary is not None:
        if summary.get('overall_status') != 'PASS' or \
                summary.get('gt_accessed') is not False or \
                summary.get('scorer_accessed') is not False or \
                summary.get('frozen_performance_bags_accessed') is not False or \
                summary.get('campaign4_started') is not False:
            errors.append('m6a7 summary is not PASS/blind')
            valid = False
        overhead = summary.get('overhead')
        gate = overhead.get('gate') if isinstance(overhead, dict) else None
        separation = summary.get('allocation_cache_separation')
        if not isinstance(gate, dict) or gate.get('median_abs_pass') is not True or \
                gate.get('bootstrap_95_upper_pass') is not True or \
                not isinstance(separation, dict) or \
                separation.get('allocation_pass') is not True or \
                separation.get('cache_separation_pass') is not True:
            errors.append('m6a7 summary measurement gates are not PASS')
            valid = False
        signal = summary.get('signal_smoke')
        wrappers = summary.get('wrapper_smoke')
        if not isinstance(signal, dict) or signal.get('docker_remnants') != 'none' or \
                not isinstance(wrappers, dict) or wrappers.get('all_pass') is not True or \
                wrappers.get('docker_remnants') != 'none':
            errors.append('m6a7 signal/wrapper smoke is not PASS')
            valid = False
    lineage = contract.get('lineage')
    roots = lineage.get('prior_failed_audit_roots') \
        if isinstance(lineage, dict) else None
    if (not isinstance(lineage, dict) or
            lineage.get('prior_failed_audits_retained') is not True or
            lineage.get('campaign4_authorized') is not False or
            not isinstance(roots, dict) or roots.get('status') != 'FAIL_CLOSED' or
            roots.get('immutable') is not True or
            roots.get('roots') != list(M6A7_FAILED_AUDIT_ROOTS) or
            any(not Path(item).is_dir() for item in roots.get('roots', []))):
        errors.append('m6a7 prior failed-audit lineage is missing or mutable')
        valid = False
    return valid, {
        'status': contract.get('status'),
        'primary_metric': contract.get('primary_metric'),
        'metric_definition': contract.get('primary_metric_definition'),
        'memory_max': contract.get('memory_max'),
        'oom_delta_required': contract.get('oom_delta_required'),
        'docker_client_comparable': contract.get('docker_client_comparable'),
        'final_audit_sha256': audit_ref.get('sha256') if audit_ref else None,
        'final_receipt_sha256': receipt_ref.get('sha256') if receipt_ref else None,
        'summary_sha256': summary_ref.get('sha256') if summary_ref else None,
        'schedule': contract.get('schedule'),
        'blind_scope': contract.get('blind_scope'),
        'prior_failed_audit_lineage': lineage,
    }


def evaluate(receipt: dict[str, Any], profile: dict[str, Any],
             root: Path = ROOT) -> dict[str, Any]:
    """Return a JSON/YAML-safe preflight result without mutating inputs."""
    errors: list[str] = []
    incomplete: list[str] = []
    checks: dict[str, dict[str, Any]] = {}

    def check(name: str, passed: bool, evidence: Any) -> None:
        checks[name] = {'pass': bool(passed), 'evidence': evidence}

    contract = profile.get('competitive_slam_profile', profile)
    policy = contract.get('evidence_gate_v2', {})
    receipt_path_value = policy.get('execution_selection_receipt_path')
    receipt_expected_sha = policy.get('execution_selection_receipt_sha256')
    receipt_path = _resolve(root, receipt_path_value)
    receipt_path_ok = receipt_path is not None and receipt_path.is_file()
    if receipt_expected_sha is None:
        _add_missing(incomplete, 'profile.execution_selection_receipt_sha256')
    elif not _is_sha(receipt_expected_sha):
        errors.append('profile.execution_selection_receipt_sha256 must be 64-hex')
    if not receipt_path_ok:
        _add_missing(incomplete, 'profile.execution_selection_receipt_path')
    actual_receipt_sha = sha256_file(receipt_path) if receipt_path_ok else None
    if (_is_sha(receipt_expected_sha) and actual_receipt_sha is not None and
            actual_receipt_sha.lower() != receipt_expected_sha.lower()):
        errors.append('execution selection receipt SHA does not match profile')
    receipt_path_check = (receipt_path_ok and _is_sha(receipt_expected_sha) and
                          actual_receipt_sha is not None and
                          actual_receipt_sha.lower() == receipt_expected_sha.lower())
    check('receipt_path_and_sha256', receipt_path_check, {
        'path': receipt_path_value,
        'expected_sha256': receipt_expected_sha,
        'actual_sha256': actual_receipt_sha,
    })

    if not isinstance(receipt, dict):
        errors.append('execution selection receipt must be a mapping')
        receipt = {}
    if receipt.get('schema_version') != 1:
        errors.append('execution selection receipt schema_version must be 1')
    if receipt.get('receipt_kind') != 'competitive_execution_selection':
        errors.append('execution selection receipt_kind is not competitive_execution_selection')
    status = receipt.get('status')
    if status not in {'ready', 'frozen'}:
        if status is None or 'pending' in str(status):
            _add_missing(incomplete, f'receipt.status is not ready: {status!r}')
        else:
            errors.append(f'receipt.status must be ready/frozen: {status!r}')
    check('receipt_status_ready', status in {'ready', 'frozen'}, {
        'status': status,
        'ready_statuses': ['frozen', 'ready'],
    })

    release = receipt.get('release')
    if release != 'Release':
        if release is None:
            _add_missing(incomplete, 'receipt.release')
        else:
            errors.append('receipt.release must be exactly Release')
    check('release_exact', release == 'Release', {'release': release})

    common = receipt.get('common_identity')
    if not isinstance(common, dict):
        errors.append('receipt.common_identity must be a mapping')
        common = {}
    closure_identity: dict[str, Any] | None = None
    closure_contract = (policy.get('rival_source_closure')
                        if isinstance(policy, dict) else None)
    closure_identity_ok = True
    if isinstance(closure_contract, dict) and closure_contract.get('required') is True:
        try:
            closure_identity = current_rival_source_closure_identity(
                profile, root=root)
            closure_errors = validate_rival_source_closure_receipt_identity(
                receipt, closure_identity)
        except (OSError, ValueError, TypeError, UnicodeError, yaml.YAMLError) as exc:
            closure_errors = [f'current rival source closure identity is invalid: {exc}']
            closure_identity = None
        if closure_errors:
            errors.extend(closure_errors)
            errors.append(
                'execution receipt is not eligible: missing or mismatched '
                'rival source closure identity')
            closure_identity_ok = False
    check('rival_source_closure_identity', closure_identity_ok, {
        'expected': closure_identity,
        'receipt': common.get('rival_source_closure'),
    })
    try:
        computed_profile_sha = canonical_profile_sha256(profile)
    except ValueError as exc:
        errors.append(str(exc))
        computed_profile_sha = None
    declared_profile_sha = common.get('profile_sha256')
    declared_profile_kind = common.get('profile_sha256_kind')
    profile_hash_ok = True
    if declared_profile_sha is None:
        _add_missing(incomplete, 'common_identity.profile_sha256')
        profile_hash_ok = False
    elif not _is_sha(declared_profile_sha):
        errors.append('common_identity.profile_sha256 must be 64-hex')
        profile_hash_ok = False
    if declared_profile_kind is None:
        _add_missing(incomplete, 'common_identity.profile_sha256_kind')
        profile_hash_ok = False
    elif declared_profile_kind != PROFILE_CANONICAL_HASH_KIND:
        errors.append(
            'common_identity.profile_sha256_kind must be '
            f'{PROFILE_CANONICAL_HASH_KIND}')
        profile_hash_ok = False
    if (profile_hash_ok and computed_profile_sha is not None and
            declared_profile_sha.lower() != computed_profile_sha):
        errors.append('common_identity.profile_sha256 does not match canonical profile')
        profile_hash_ok = False
    check('profile_canonical_hash', profile_hash_ok and computed_profile_sha is not None, {
        'hash_kind': declared_profile_kind,
        'expected_hash_kind': PROFILE_CANONICAL_HASH_KIND,
        'declared_sha256': declared_profile_sha,
        'computed_sha256': computed_profile_sha,
        'excluded_path': '.'.join(
            ('competitive_slam_profile', 'evidence_gate_v2',
             'execution_selection_receipt_sha256')),
    })
    scorer = common.get('scorer')
    scorer_ok = isinstance(scorer, dict)
    if not scorer_ok:
        errors.append('receipt.common_identity.scorer must be a mapping')
        scorer = {}
    scorer_files = scorer.get('files') or {}
    scorer_files_ok = isinstance(scorer_files, dict)
    if not scorer_files_ok:
        errors.append('common_identity.scorer.files must be a mapping')
        scorer_files = {}
    scorer_payload: list[dict[str, Any]] = []
    for name, item in sorted(scorer_files.items()):
        file_ok = _check_hash_file(root, item, f'scorer.{name}', errors,
                                   incomplete)
        scorer_files_ok = file_ok and scorer_files_ok
        if file_ok and isinstance(item, dict):
            actual = _hash_path(root, item.get('path'))
            scorer_payload.append({
                'name': name,
                'path': item.get('path'),
                'sha256': actual,
                'policy': item.get('policy'),
            })
    computed_scorer_fingerprint = (
        _canonical_hash(scorer_payload) if scorer_files_ok else None)
    scorer_fingerprint = scorer.get('canonical_fingerprint')
    if scorer_fingerprint is None:
        _add_missing(incomplete, 'common_identity.scorer.canonical_fingerprint')
    elif not _is_sha(scorer_fingerprint):
        errors.append('common_identity.scorer.canonical_fingerprint must be 64-hex')
    if (computed_scorer_fingerprint is not None and
            isinstance(scorer_fingerprint, str) and
            computed_scorer_fingerprint != scorer_fingerprint.lower()):
        errors.append('common_identity.scorer.canonical_fingerprint does not match '
                      'the canonical scorer file payload')
    check('scorer_files_and_fingerprint', scorer_ok and scorer_files_ok and
          _is_sha(scorer_fingerprint) and
          computed_scorer_fingerprint == scorer_fingerprint.lower(), {
              'canonical_fingerprint': scorer_fingerprint,
              'computed_fingerprint': computed_scorer_fingerprint,
              'canonical_payload': scorer_payload,
              'file_count': len(scorer_files),
          })

    machine = common.get('machine_fingerprint')
    machine_ok = _check_hash_file(root, machine, 'machine_fingerprint', errors,
                                  incomplete)
    machine_status = machine.get('status') if isinstance(machine, dict) else None
    if machine_status not in {'ready', 'frozen'}:
        if machine_status is None or str(machine_status).startswith('pending') or \
                machine_status == 'available_but_refresh_required_before_run':
            _add_missing(incomplete, 'machine_fingerprint.status is not refreshed')
        else:
            errors.append('machine_fingerprint.status must be ready or frozen')
        machine_ok = False
    check('machine_fingerprint', machine_ok, {
        'path': machine.get('path') if isinstance(machine, dict) else None,
        'status': machine_status,
    })

    thread = common.get('thread_policy')
    common_thread_ok = _check_thread_policy(thread, 'common_identity.thread_policy',
                                            errors, incomplete)
    thread_status = thread.get('status') if isinstance(thread, dict) else None
    if thread_status not in {'ready', 'frozen'}:
        if thread_status is None or str(thread_status).startswith('pending'):
            _add_missing(incomplete, 'common_identity.thread_policy.status is not ready')
        else:
            errors.append('common_identity.thread_policy.status must be ready or frozen')
        common_thread_ok = False
    thread_policy_canonical_hash = (
        _canonical_hash({key: thread.get(key) for key in THREAD_KEYS})
        if common_thread_ok and isinstance(thread, dict) else None)
    check('thread_policy_complete', common_thread_ok, {
        'required_keys': list(THREAD_KEYS),
        'status': thread_status,
        'canonical_sha256': thread_policy_canonical_hash,
        'policy': thread,
    })

    systems = receipt.get('systems')
    if not isinstance(systems, dict):
        errors.append('receipt.systems must be a mapping')
        systems = {}
    systems_ok = True
    per_system_results: dict[str, bool] = {}
    for system in SYSTEMS:
        per_system_ok = True
        item = systems.get(system)
        if not isinstance(item, dict):
            _add_missing(incomplete, f'systems.{system}')
            per_system_ok = False
            per_system_results[system] = per_system_ok
            continue
        repository = item.get('repository') or {}
        revision = repository.get('revision')
        if not _nonempty(repository.get('url')):
            _add_missing(incomplete, f'{system}.repository.url')
            per_system_ok = False
        if revision is None:
            _add_missing(incomplete, f'{system}.repository.revision')
            per_system_ok = False
        elif not isinstance(revision, str) or not COMMIT_RE.fullmatch(revision):
            errors.append(f'{system}.repository.revision must be a 40-hex commit')
            per_system_ok = False
        revision_status = repository.get('revision_status')
        if revision_status not in {'ready', 'frozen', 'pinned'}:
            if revision_status is None or 'pending' in str(revision_status):
                _add_missing(incomplete, f'{system}.repository.revision_status is not ready')
            else:
                errors.append(f'{system}.repository.revision_status is not ready/frozen/pinned')
            per_system_ok = False
        worktree_dirty = repository.get('worktree_dirty')
        if worktree_dirty is None:
            _add_missing(incomplete, f'{system}.repository.worktree_dirty')
            per_system_ok = False
        elif worktree_dirty is not False:
            _add_missing(incomplete, f'{system}.repository.worktree must be clean')
            per_system_ok = False
        for clean_key in ('tracked_diff_sha256', 'untracked_content_sha256',
                          'clean_provenance_sha256'):
            clean_value = repository.get(clean_key)
            if worktree_dirty is False and not _is_sha(clean_value):
                _add_missing(incomplete, f'{system}.repository.{clean_key}')
                per_system_ok = False
        container = item.get('container') or {}
        for index, config in enumerate(item.get('configs', [])):
            per_system_ok = (_check_config(root, config, f'{system}.configs[{index}]',
                                           errors, incomplete,
                                           container_digest=container.get('image_digest'))
                             and per_system_ok)
        runner = item.get('runner')
        per_system_ok = (_check_hash_file(root, runner, f'{system}.runner', errors,
                                          incomplete) and per_system_ok)
        container_status = container.get('status')
        if container_status not in {'ready', 'frozen'}:
            if container_status is None or 'pending' in str(container_status):
                _add_missing(incomplete, f'{system}.container.status is not ready')
            else:
                errors.append(f'{system}.container.status must be ready or frozen')
            per_system_ok = False
        tag = container.get('image_tag')
        digest = container.get('image_digest')
        if tag is None or digest is None:
            _add_missing(incomplete, f'{system}.container.image_tag_or_digest')
            per_system_ok = False
        else:
            if not _nonempty(tag):
                errors.append(f'{system}.container.image_tag must be non-empty')
                per_system_ok = False
            if not isinstance(digest, str) or not CONTAINER_DIGEST_RE.fullmatch(digest):
                errors.append(f'{system}.container.image_digest must be sha256:<64hex>')
                per_system_ok = False
        toolchain = item.get('toolchain') or {}
        toolchain_status = toolchain.get('status')
        if toolchain_status not in {'ready', 'frozen'}:
            if toolchain_status is None or 'pending' in str(toolchain_status):
                _add_missing(incomplete, f'{system}.toolchain.status is not ready')
            else:
                errors.append(f'{system}.toolchain.status must be ready or frozen')
            per_system_ok = False
        fingerprint = toolchain.get('fingerprint')
        if fingerprint is None:
            _add_missing(incomplete, f'{system}.toolchain.fingerprint')
            per_system_ok = False
        elif not _is_sha(fingerprint):
            errors.append(f'{system}.toolchain.fingerprint must be 64-hex')
            per_system_ok = False
        if item.get('release') != 'Release':
            if item.get('release') is None:
                _add_missing(incomplete, f'{system}.release')
            else:
                errors.append(f'{system}.release must be exactly Release')
            per_system_ok = False
        ref = item.get('thread_policy_ref')
        if not _nonempty(ref):
            _add_missing(incomplete, f'{system}.thread_policy_ref')
            per_system_ok = False
        per_system_results[system] = per_system_ok
        systems_ok = systems_ok and per_system_ok
        check(f'system_{system}', per_system_ok, {
            'revision': revision,
            'revision_status': revision_status,
            'worktree_dirty': worktree_dirty,
            'container_digest': digest,
            'container_status': container_status,
            'toolchain_fingerprint': fingerprint,
            'toolchain_status': toolchain_status,
        })
    check('all_systems_pinned_and_resolved', systems_ok, {
        'systems': list(SYSTEMS),
        'per_system': per_system_results,
        'expected_thread_policy_keys': list(THREAD_KEYS),
    })
    memory_ok, memory_evidence = _check_m6a5_memory_contract(
        receipt, root, errors, incomplete)
    check('m6a5_memory_contract_and_lineage', memory_ok, memory_evidence)
    process_rss_ok, process_rss_evidence = _check_m6a7_process_rss_contract(
        receipt, profile, root, errors, incomplete)
    check('m6a7_process_rss_contract', process_rss_ok, process_rss_evidence)
    rival_source_closure = verify_rival_source_closure(
        profile, root=root, receipt=receipt)
    rival_source_closure_ok = rival_source_closure.get('pass') is True
    check('rival_source_closure', rival_source_closure_ok,
          rival_source_closure)
    if not rival_source_closure_ok:
        if rival_source_closure.get('status') == 'NOT_READY':
            incomplete.extend(
                'rival source closure: ' + str(item)
                for item in rival_source_closure.get('not_ready', [])
            )
            incomplete.extend(
                'rival source closure incomplete: ' + str(item)
                for item in rival_source_closure.get('incomplete', [])
            )
        else:
            errors.extend(
                'rival source closure: ' + str(item)
                for item in rival_source_closure.get('errors', [])
            )
            incomplete.extend(
                'rival source closure incomplete: ' + str(item)
                for item in rival_source_closure.get('incomplete', [])
            )

    dataset_source_closure = verify_dataset_source_closure(profile, root=root)
    dataset_source_closure_ok = dataset_source_closure.get('pass') is True
    check('dataset_source_closure', dataset_source_closure_ok,
          dataset_source_closure)
    if not dataset_source_closure_ok:
        if dataset_source_closure.get('status') == 'NOT_READY':
            incomplete.extend(
                'dataset source closure: ' + str(item)
                for item in dataset_source_closure.get('not_ready', [])
            )
        else:
            errors.extend(
                'dataset source closure: ' + str(item)
                for item in dataset_source_closure.get('errors', [])
            )

    status = 'INVALID' if errors else ('INCOMPLETE' if incomplete else 'PASS')
    return {
        'schema_version': 1,
        'receipt_kind': 'competitive_execution_preflight_result',
        'status': status,
        'pass': status == 'PASS',
        'errors': errors + incomplete,
        'checks': checks,
        'receipt_status': receipt.get('status'),
        'canonical_scorer_fingerprint': computed_scorer_fingerprint,
        'thread_policy_canonical_sha256': thread_policy_canonical_hash,
        'm6a5_memory_contract': memory_evidence,
        'm6a7_process_rss_contract': process_rss_evidence,
        'rival_source_closure': rival_source_closure,
        'dataset_source_closure': dataset_source_closure,
        'profile_execution_selection_receipt': {
            'path': receipt_path_value,
            'expected_sha256': receipt_expected_sha,
            'actual_sha256': actual_receipt_sha,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--receipt', type=Path, default=DEFAULT_RECEIPT)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--yaml-output', type=Path)
    args = parser.parse_args()
    receipt = yaml.safe_load(args.receipt.read_text(encoding='utf-8'))
    profile = yaml.safe_load(args.profile.read_text(encoding='utf-8'))
    result = evaluate(receipt, profile)
    profile_file_sha256 = sha256_file(args.profile)
    result['identity'] = {
        'profile_sha256': canonical_profile_sha256(profile),
        'profile_sha256_kind': PROFILE_CANONICAL_HASH_KIND,
        'profile_file_sha256': profile_file_sha256,
        'execution_receipt_sha256': result['profile_execution_selection_receipt'].get(
            'actual_sha256'),
        'canonical_scorer_fingerprint': result.get(
            'canonical_scorer_fingerprint'),
        'thread_policy_canonical_sha256': result.get(
            'thread_policy_canonical_sha256'),
        'rival_source_closure': result['checks'].get(
            'rival_source_closure_identity', {}).get('evidence', {}).get(
                'expected'),
        'dataset_source_closure_sha256': result.get(
            'dataset_source_closure', {}).get(
                'dataset_source_closure_sha256'),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n',
                           encoding='utf-8')
    yaml_output = args.yaml_output
    if yaml_output is not None:
        yaml_output.parent.mkdir(parents=True, exist_ok=True)
        yaml_output.write_text(yaml.safe_dump(result, sort_keys=True),
                               encoding='utf-8')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError,
            json.JSONDecodeError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
