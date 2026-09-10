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

"""Single-process fail-closed launcher for the M6a10 fixed10-v7 attempt.

The launcher owns the whole preflight-to-run transition.  It reserves a new
attempt root, invokes the quiescence checker exactly once, verifies the
resulting receipt, and immediately starts the fixed Docker argv from this
process.  There is no shell interpolation, retry, or second command for a
human to run between PASS and ``Popen``.  Every terminal path gets a distinct
immutable receipt; an existing root or receipt is never overwritten.

This module is intentionally not a benchmark driver in tests: tests inject a
quiescence writer and a process factory.  The production command remains
GT-blind, no-map, network-isolated, and has no taskset/cpuset option.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import datetime as dt
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Any, Callable, Sequence


CONTRACT_ID = 'm6a10-v2a-ours-rko-unpaced-ack-fixed10-v7'
SCHEMA_VERSION = 1
IMAGE_TAG = 'm6a10-v2a-fixed10-v2-lidarslam-ours:jazzy'
IMAGE_DIGEST = (
    'sha256:385b6eeda3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69')
EXPECTED_INPUT_TREE_SHA256 = (
    '0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb926')
EXPECTED_QUIESCENCE_SHA256 = (
    'acf8b2e65e7744dd9cfd4127ac5ec12fb381c5a7c1430910b3e769ea259fe85d')
EXPECTED_WRAPPER_SHA256 = (
    '806857afec65c16105446e75fb32450e2b0efe1116268ce4412ffb44559db8a6')
EXPECTED_RUNNER_SHA256 = (
    'd82df94521189d97fabd0b8f672b2b61a239a417f7e3220ddb4ab12a65ef450f')
EXPECTED_LAUNCH_SHA256 = (
    'd45545717f90f6877b5f281fc5623df04b824f7a236d2fe73b91c2dd3714371c')
EXPECTED_PHASE_CONTRACT_SHA256 = (
    'c4c917b2a0d5765bea7783acaf4f2a626d9a77442a8da81bb27ac4457de3313c')
MAX_PREFLIGHT_TO_RUN_GAP_SECONDS = 2.0
MAX_PREFLIGHT_TO_RUN_GAP_NS = int(MAX_PREFLIGHT_TO_RUN_GAP_SECONDS * 1e9)
EXPECTED_IMAGE_LABELS = {
    'benchmark.ours.launch_sha256': EXPECTED_LAUNCH_SHA256,
    'benchmark.rko_lio.initialized': 'true',
    'benchmark.ours.revision': '866f733677e92ecb08d67126e463da99dd140d46',
}
DEFAULT_INPUT_ROOT = Path(
    '/media/sasaki/aiueo1/datasets/ntu_viral_release/'
    'tnp_01_m6a10_v2a_sync_materialization_v1_ros2')
DEFAULT_DRAIN_TIMEOUT_SECONDS = 30
DEFAULT_SAMPLE_SECONDS = 5.0
DEFAULT_MAX_BUSY_PERCENT = 5.0
DEFAULT_MAX_LOAD_PER_CPU = 0.5


class LaunchError(RuntimeError):
    """A fail-closed launcher error with a machine-readable kind."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


@dataclass(frozen=True)
class LaunchConfig:
    root: Path
    repo_root: Path
    input_root: Path = DEFAULT_INPUT_ROOT
    image_tag: str = IMAGE_TAG
    image_digest: str = IMAGE_DIGEST
    quiescence_script: Path = Path('scripts/check_m6a10_quiescence.py')
    sample_seconds: float = DEFAULT_SAMPLE_SECONDS
    max_busy_percent: float = DEFAULT_MAX_BUSY_PERCENT
    max_load_per_cpu: float = DEFAULT_MAX_LOAD_PER_CPU
    expected_input_tree_sha256: str | None = EXPECTED_INPUT_TREE_SHA256


def _utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def tree_sha256(path: Path) -> str:
    """Hash relative paths and bytes without loading the input into memory."""
    if not path.is_dir() or path.is_symlink():
        raise LaunchError('INPUT_TREE_INVALID', f'input tree is invalid: {path}')
    files = sorted(item for item in path.rglob('*')
                   if item.is_file() and not item.is_symlink())
    if not files:
        raise LaunchError('INPUT_TREE_EMPTY', f'input tree is empty: {path}')
    digest = hashlib.sha256()
    for item in files:
        digest.update(item.relative_to(path).as_posix().encode('utf-8'))
        digest.update(b'\0')
        with item.open('rb') as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


def _repo_file(config: LaunchConfig, relative_or_absolute: Path) -> Path:
    return (relative_or_absolute if relative_or_absolute.is_absolute()
            else config.repo_root / relative_or_absolute)


def _inspect_image(config: LaunchConfig) -> dict[str, Any]:
    """Inspect the local tag before quiescence; never pull or build."""
    command = ['docker', 'image', 'inspect', config.image_tag]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        raise LaunchError('IMAGE_INSPECT_FAILURE', completed.stderr.strip() or
                          'docker image inspect failed')
    try:
        documents = json.loads(completed.stdout)
        document = documents[0]
        labels = document.get('Config', {}).get('Labels') or {}
    except (IndexError, TypeError, AttributeError, json.JSONDecodeError) as error:
        raise LaunchError(
            'IMAGE_INSPECT_INVALID',
            'docker image inspect JSON is invalid') from error
    if document.get('Id') != config.image_digest:
        raise LaunchError('IMAGE_IDENTITY_MISMATCH', 'local image Id differs from pinned digest')
    for key, expected in EXPECTED_IMAGE_LABELS.items():
        if labels.get(key) != expected:
            raise LaunchError('IMAGE_LABEL_MISMATCH', f'image label mismatch: {key}')
    return {'id': document.get('Id'), 'repo_tags': document.get('RepoTags', []),
            'labels': labels}


def validate_preflight_identity(
        config: LaunchConfig,
        *, image_probe: Callable[[LaunchConfig], dict[str, Any]] | None = None,
        expected_input_tree_sha256: str | None = None) -> dict[str, Any]:
    """Bind every mounted source and the local image before quiescence."""
    repo = _absolute_directory(config.repo_root, 'repository root')
    input_root = _absolute_directory(config.input_root, 'input root')
    if not repo.is_dir() or not input_root.is_dir():
        raise LaunchError('INPUT_OR_REPO_MISSING', 'repository or input root is missing')
    expected_tree = (config.expected_input_tree_sha256 if
                     expected_input_tree_sha256 is None else expected_input_tree_sha256)
    observed_tree = tree_sha256(input_root)
    if expected_tree is not None and observed_tree != expected_tree:
        raise LaunchError('INPUT_TREE_IDENTITY_MISMATCH', 'input tree SHA differs from v7 pin')
    source_paths = {
        'quiescence_script': _repo_file(config, config.quiescence_script),
        'wrapper': repo / 'scripts' / 'ours_container_gt_blind_run.sh',
        'runner': repo / 'scripts' / 'run_rko_lio_graph_benchmark.sh',
        'launch': repo / 'lidarslam' / 'launch' / 'rko_lio_slam.launch.py',
        'phase_contract': repo / 'scripts' / 'benchmark_phase_contract.py',
    }
    observed_sources = {}
    expected_sources = {
        'quiescence_script': EXPECTED_QUIESCENCE_SHA256,
        'wrapper': EXPECTED_WRAPPER_SHA256,
        'runner': EXPECTED_RUNNER_SHA256,
        'launch': EXPECTED_LAUNCH_SHA256,
        'phase_contract': EXPECTED_PHASE_CONTRACT_SHA256,
    }
    for name, path in source_paths.items():
        if not path.is_file() or path.is_symlink():
            raise LaunchError('SOURCE_IDENTITY_MISSING', f'{name} source is invalid: {path}')
        observed = file_sha256(path)
        observed_sources[name] = {'path': str(path), 'sha256': observed}
        if observed != expected_sources[name]:
            raise LaunchError('SOURCE_IDENTITY_MISMATCH', f'{name} SHA differs from v7 pin')
    image = (image_probe or _inspect_image)(config)
    return {'input_tree_sha256': observed_tree, 'sources': observed_sources,
            'image': image}


def _json_bytes(value: dict[str, Any]) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True) + '\n').encode('utf-8')


def atomic_json(path: Path, value: dict[str, Any]) -> str:
    """Create one JSON receipt without ever replacing an existing file."""
    if path.exists() or path.is_symlink():
        raise LaunchError('RECEIPT_OVERWRITE', f'refusing to overwrite {path}')
    part = path.with_name(path.name + '.part')
    if part.exists() or part.is_symlink():
        raise LaunchError('RECEIPT_OVERWRITE', f'stale receipt part exists: {part}')
    payload = _json_bytes(value)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        with os.fdopen(fd, 'wb') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(part, path)
    except Exception:
        try:
            part.unlink()
        except FileNotFoundError:
            pass
        raise
    return hashlib.sha256(payload).hexdigest()


def _absolute_directory(path: Path, label: str) -> Path:
    if not path.is_absolute():
        raise LaunchError('PATH_NOT_ABSOLUTE', f'{label} must be absolute: {path}')
    if path.is_symlink():
        raise LaunchError('PATH_SYMLINK', f'{label} must not be a symlink: {path}')
    if path == Path('/'):
        raise LaunchError('PATH_TOO_BROAD', f'{label} may not be /')
    return path


def _reject_path_overlap(first: Path, second: Path) -> None:
    first_resolved = first.resolve(strict=False)
    second_resolved = second.resolve(strict=False)
    if (first_resolved == second_resolved or
            first_resolved in second_resolved.parents or
            second_resolved in first_resolved.parents):
        raise LaunchError('ROOT_OVERLAP', 'attempt and input roots overlap')


def reserve_attempt_root(config: LaunchConfig) -> dict[str, Any]:
    """Reserve a new, empty root and atomically write its immutable marker."""
    root = _absolute_directory(config.root, 'attempt root')
    repo = _absolute_directory(config.repo_root, 'repository root')
    input_root = _absolute_directory(config.input_root, 'input root')
    if not repo.is_dir() or not input_root.is_dir():
        raise LaunchError('INPUT_OR_REPO_MISSING', 'repository or input root is missing')
    _reject_path_overlap(root, input_root)
    _reject_path_overlap(root, repo)
    if root.exists():
        if not root.is_dir():
            raise LaunchError('ROOT_NOT_DIRECTORY', f'attempt root is not a directory: {root}')
        if any(root.iterdir()):
            raise LaunchError('ROOT_NOT_EMPTY', f'attempt root is not empty: {root}')
    else:
        parent = root.parent
        if parent.is_symlink() or not parent.is_dir():
            raise LaunchError('ROOT_PARENT_INVALID', f'attempt root parent is invalid: {parent}')
        try:
            root.mkdir()
        except FileExistsError as error:
            raise LaunchError(
                'ROOT_RACE',
                f'attempt root appeared during reservation: {root}') from error
    marker = {
        'schema_version': SCHEMA_VERSION,
        'contract_id': CONTRACT_ID,
        'status': 'RESERVED_EMPTY_ROOT',
        'created_at_utc': _utc_now(),
        'root': str(root),
        'repo_root': str(repo),
        'input_root': str(input_root),
        'launcher_pid': os.getpid(),
        'launcher_path': str(Path(__file__).resolve()),
        'launcher_sha256': file_sha256(Path(__file__).resolve()),
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    marker_sha = atomic_json(root / 'attempt_root_marker.json', marker)
    marker['receipt_sha256'] = marker_sha
    return marker


def _validated_arg(value: str, label: str) -> str:
    if (not isinstance(value, str) or not value or '\x00' in value or
            '\n' in value or '\r' in value):
        raise LaunchError('ARGUMENT_INJECTION', f'invalid {label}')
    return value


def build_runner_argv(config: LaunchConfig, output_dir: Path) -> list[str]:
    """Build the fixed Docker argv; never invoke a shell or accept free argv."""
    if config.image_tag != IMAGE_TAG or config.image_digest != IMAGE_DIGEST:
        raise LaunchError('IMAGE_IDENTITY_MISMATCH', 'v7 image identity is not pinned')
    repo = _absolute_directory(config.repo_root, 'repository root')
    input_root = _absolute_directory(config.input_root, 'input root')
    output_dir = _absolute_directory(output_dir, 'output directory')
    if not repo.is_dir() or not input_root.is_dir():
        raise LaunchError('INPUT_OR_REPO_MISSING', 'runner input or repository is missing')
    image_ref = _validated_arg(config.image_digest, 'image digest')
    values = {
        'repo': str(repo), 'input': str(input_root), 'output': str(output_dir),
        'image': image_ref,
    }
    for label, value in values.items():
        _validated_arg(value, label)
    env = [
        ('ROS_DOMAIN_ID', '229'), ('OMP_NUM_THREADS', '8'),
        ('OPENBLAS_NUM_THREADS', '8'), ('MKL_NUM_THREADS', '8'),
        ('TBB_NUM_THREADS', '8'),
        ('M6A10_PHASE_CONTRACT_VERSION', 'm6a10-online-compute-v2'),
        ('M6A10_PHASE_MODE', 'unpaced_ack'),
        ('M6A10_DRAIN_TIMEOUT_SECONDS', str(DEFAULT_DRAIN_TIMEOUT_SECONDS)),
        ('M6A10_SKIP_MAP_SAVE', '1'), ('M6A10_BENCHMARK_NO_MAP_ARTIFACTS', '1'),
        ('LIDARSLAM_WS_ROOT', '/opt/ours_ws'),
        ('RKO_PARAM', '/runner/lidarslam/param/rko_lio_ntu_viral.yaml'),
        ('BAG_PATH', '/input/canonical_ros2'), ('OUT_DIR', '/out'),
        ('LIDAR_TOPIC', '/os1_cloud_node1/points'), ('IMU_TOPIC', '/imu/imu'),
    ]
    argv = [
        '/usr/bin/time', '-v', '-o', str(config.root / 'time-v.txt'),
        'docker', 'run', '--rm', '--init', '--pull=never', '--network', 'none',
        '--read-only', '--tmpfs', '/tmp:rw,noexec,nosuid,size=1024m',
        '--shm-size', '512m',
    ]
    for key, value in env:
        argv.extend(['-e', f'{key}={value}'])
    argv.extend([
        '-v', f'{repo}:/runner:ro',
        '-v', f'{input_root}:/input/canonical_ros2:ro',
        '-v', f'{output_dir}:/out:rw',
        '--entrypoint', '/bin/bash', image_ref,
        '/runner/scripts/ours_container_gt_blind_run.sh',
    ])
    if any(item in ('taskset', '--cpuset-cpus', '--cpus') for item in argv):
        raise LaunchError('CPU_POLICY_MISMATCH', 'taskset/cpuset is forbidden in v7 argv')
    if any('ground_truth' in item.lower() or 'scorer' in item.lower() for item in argv):
        raise LaunchError('GT_SCORER_ARGUMENT', 'GT/scorer material is forbidden in v7 argv')
    return argv


def _quiescence_command(config: LaunchConfig, receipt: Path) -> list[str]:
    script = _repo_file(config, config.quiescence_script)
    if not script.is_file() or script.is_symlink():
        raise LaunchError('QUIESCENCE_SCRIPT_INVALID', f'invalid quiescence script: {script}')
    return [
        sys.executable, str(script), '--output', str(receipt),
        '--sample-seconds', str(config.sample_seconds),
        '--max-busy-percent', str(config.max_busy_percent),
        '--max-load-per-cpu', str(config.max_load_per_cpu),
    ]


def _load_pass_receipt(path: Path) -> tuple[dict[str, Any], str]:
    if not path.is_file() or path.is_symlink():
        raise LaunchError('PREFLIGHT_RECEIPT_MISSING', f'missing quiescence receipt: {path}')
    digest = file_sha256(path)
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise LaunchError('PREFLIGHT_RECEIPT_INVALID', 'quiescence receipt is not JSON') from error
    if (value.get('schema_version') != 1 or
            value.get('contract_version') != 'm6a10-quiescence-v1' or
            value.get('status') != 'PASS' or
            value.get('runner_start_allowed') is not True):
        raise LaunchError('PREFLIGHT_FAIL_CLOSED', 'quiescence did not authorize runner start')
    if value.get('ground_truth_content_opened') is not False or \
            value.get('scorer_invoked') is not False:
        raise LaunchError('PREFLIGHT_SAFETY_INVALID', 'quiescence safety flags are invalid')
    observation = value.get('observation')
    if not isinstance(observation, dict) or \
            observation.get('sample_seconds') != DEFAULT_SAMPLE_SECONDS or \
            observation.get('nproc', 0) <= 0 or \
            observation.get('proc_race_skips') != 0 or \
            observation.get('forbidden_processes') != []:
        raise LaunchError('PREFLIGHT_OBSERVATION_INVALID', 'quiescence observation is incomplete')
    limits = observation.get('limits')
    checks = observation.get('checks')
    load = observation.get('loadavg')
    cpu = observation.get('cpu')
    if (not isinstance(limits, dict) or
            limits.get('max_cpu_busy_percent') != DEFAULT_MAX_BUSY_PERCENT or
            limits.get('max_load1_per_cpu') != DEFAULT_MAX_LOAD_PER_CPU or
            not isinstance(checks, dict) or not checks or
            any(value is not True for value in checks.values()) or
            not isinstance(load, dict) or not math.isfinite(
                float(load.get('load1_per_cpu', -1))) or
            float(load['load1_per_cpu']) > DEFAULT_MAX_LOAD_PER_CPU or
            not isinstance(cpu, dict) or
            not math.isfinite(float(cpu.get('busy_percent', -1))) or
            float(cpu['busy_percent']) > DEFAULT_MAX_BUSY_PERCENT):
        raise LaunchError('PREFLIGHT_OBSERVATION_INVALID', 'quiescence limits/checks are invalid')
    return value, digest


def _write_closure(root: Path, *, kind: str, message: str,
                   preflight: dict[str, Any] | None = None,
                   extra: dict[str, Any] | None = None,
                   status: str = 'FAIL_CLOSED') -> str:
    value: dict[str, Any] = {
        'schema_version': SCHEMA_VERSION,
        'contract_id': CONTRACT_ID,
        'status': status,
        'failure_kind': kind,
        'message': message,
        'closed_at_utc': _utc_now(),
        'preflight': preflight,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'retry_count': 0,
    }
    if extra:
        value.update(extra)
    return atomic_json(root / 'closure_receipt.json', value)


REQUIRED_OUTPUT_FILES = (
    'phase_evidence.json', 'consumer_evidence.json',
    'container_process_rss.json', 'container_memory.json',
    'gt_blind_run.json', 'traj_raw.tum', 'traj_corrected.tum')
FORBIDDEN_OUTPUT_TOKENS = (
    'map.pcd', 'map_bundle.yaml', 'map_projector_info.yaml',
    'degeneracy_report.yaml', 'pose_graph.g2o', 'trajectory_optimized.tum',
    'loop_edges.csv', 'pointcloud_map', 'ground_truth', 'scorer')


def _json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise LaunchError('COMPLETION_EVIDENCE_INVALID', f'invalid JSON: {path}') from error
    if not isinstance(value, dict):
        raise LaunchError('COMPLETION_EVIDENCE_INVALID', f'JSON object required: {path}')
    return value


def _phase_contract_module(repo_root: Path) -> Any:
    path = repo_root / 'scripts' / 'benchmark_phase_contract.py'
    spec = importlib.util.spec_from_file_location('m6a10_v7_phase_contract', path)
    if spec is None or spec.loader is None:
        raise LaunchError('COMPLETION_VALIDATOR_MISSING', 'phase contract module cannot load')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def validate_completed_output(output_dir: Path, *, repo_root: Path) -> dict[str, Any]:
    """Require host-visible, GT-blind completion before returning success."""
    if not output_dir.is_dir() or output_dir.is_symlink():
        raise LaunchError('COMPLETION_OUTPUT_MISSING', 'runner output directory is missing')
    files = [item for item in output_dir.rglob('*') if item.is_file()]
    if any(item.name.endswith('.part') for item in files):
        raise LaunchError('COMPLETION_PARTIAL_OUTPUT', 'output contains a .part file')
    for item in files:
        lowered = item.relative_to(output_dir).as_posix().lower()
        if any(token in lowered for token in FORBIDDEN_OUTPUT_TOKENS):
            raise LaunchError('COMPLETION_FORBIDDEN_MAP', f'forbidden output: {lowered}')
    missing = [name for name in REQUIRED_OUTPUT_FILES
               if not (output_dir / name).is_file()]
    if missing:
        raise LaunchError('COMPLETION_REQUIRED_OUTPUT_MISSING', ','.join(missing))
    consumer = _json_object(output_dir / 'consumer_evidence.json')
    phase = _json_object(output_dir / 'phase_evidence.json')
    memory = _json_object(output_dir / 'container_memory.json')
    rss = _json_object(output_dir / 'container_process_rss.json')
    blind = _json_object(output_dir / 'gt_blind_run.json')
    if (blind.get('ground_truth_accessed') is not False or
            blind.get('scorer_invoked') is not False or
            blind.get('ground_truth_content_opened', False) is not False or
            'ground_truth' in str(blind.get('bag_path', '')).lower()):
        raise LaunchError('COMPLETION_GT_BLIND_INVALID', 'GT-blind proof is not strict')
    if memory.get('status') != 'pass' or memory.get('atomic') is not True:
        raise LaunchError('COMPLETION_MEMORY_INVALID', 'container memory evidence is invalid')
    if rss.get('status') != 'pass' or rss.get('atomic') is not True:
        raise LaunchError('COMPLETION_RSS_INVALID', 'process RSS evidence is invalid')
    phase_contract = _phase_contract_module(repo_root)
    try:
        phase_result = phase_contract.validate_evidence_v2(
            phase, maximum_online_rtf=1.0,
            maximum_end_gap_seconds=0.25,
            maximum_callback_latency_seconds=0.25,
            maximum_backlog_messages=0, require_pass=True)
        consumer_result = phase_contract.validate_consumer_evidence_v2(
            consumer, maximum_end_gap_seconds=0.25,
            maximum_callback_latency_seconds=0.25,
            maximum_backlog_messages=0)
    except (ValueError, TypeError, KeyError) as error:
        raise LaunchError('COMPLETION_PHASE_INVALID', str(error)) from error
    if phase_result.get('status') != 'pass' or consumer_result.get('status') != 'pass':
        raise LaunchError('COMPLETION_PHASE_INVALID', 'phase/consumer validator did not pass')
    for name in ('traj_raw.tum', 'traj_corrected.tum'):
        if (output_dir / name).stat().st_size == 0:
            raise LaunchError('COMPLETION_TRAJECTORY_EMPTY', name)
    return {
        'status': 'PASS', 'required_files': list(REQUIRED_OUTPUT_FILES),
        'phase_status': phase_result.get('status'),
        'consumer_status': consumer_result.get('status'),
        'forbidden_map_paths': [], 'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }


def validate_host_time_report(path: Path) -> dict[str, Any]:
    """Validate the GNU time report before a zero exit can complete."""
    if not path.is_file() or path.is_symlink():
        raise LaunchError('HOST_TIME_INVALID', 'GNU time report is missing')
    text = path.read_text(encoding='utf-8', errors='replace')
    patterns = {
        'wall_seconds': r'Elapsed \(wall clock\) time .*?:\s*([0-9:.]+)',
        'user_seconds': r'User time \(seconds\):\s*([0-9.]+)',
        'sys_seconds': r'System time \(seconds\):\s*([0-9.]+)',
        'peak_rss_kb': r'Maximum resident set size \(kbytes\):\s*(\d+)',
    }
    values: dict[str, Any] = {}
    for key, pattern in patterns.items():
        match = re.search(pattern, text)
        if match is None:
            raise LaunchError('HOST_TIME_INVALID', f'GNU time field missing: {key}')
        raw = match.group(1)
        if key == 'wall_seconds':
            pieces = [float(item) for item in raw.split(':')]
            values[key] = sum(
                value * (60 ** index)
                for index, value in enumerate(reversed(pieces)))
        else:
            values[key] = float(raw)
        if not math.isfinite(float(values[key])) or float(values[key]) < 0:
            raise LaunchError('HOST_TIME_INVALID', f'GNU time field invalid: {key}')
    values['sha256'] = file_sha256(path)
    return values


def run_v7(
        config: LaunchConfig, *,
        quiescence_runner: Callable[[LaunchConfig, Path], int] | None = None,
        process_factory: Callable[..., Any] = subprocess.Popen,
        now: Callable[[], str] = _utc_now,
        wait: Callable[[Any], int] | None = None,
        identity_validator: Callable[[LaunchConfig], dict[str, Any]] | None = None,
        completion_validator: Callable[[Path], dict[str, Any]] | None = None) -> int:
    """Run one attempt, returning distinct fail-closed exit classes."""
    marker: dict[str, Any] | None = None
    root = config.root
    child_holder: dict[str, Any] = {}
    received_signal: dict[str, int | None] = {'signum': None}

    def forward_signal(signum: int, _frame: Any) -> None:
        received_signal['signum'] = signum
        child = child_holder.get('process')
        if child is not None:
            try:
                child.send_signal(signum)
            except OSError:
                pass

    old_signal_handlers: dict[int, Any] = {}
    try:
        marker = reserve_attempt_root(config)
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_signal_handlers[signum] = signal.signal(signum, forward_signal)
        identity_check = identity_validator or validate_preflight_identity
        try:
            identity = identity_check(config)
        except LaunchError as error:
            _write_closure(root, kind='IDENTITY_PREFLIGHT_FAIL_CLOSED',
                           message=str(error))
            return 11
        q_path = root / 'quiescence.json'
        runner = quiescence_runner or _run_quiescence
        q_returncode = runner(config, q_path)
        quiescence_completed_ns = time.monotonic_ns()
        if received_signal['signum'] is not None:
            _write_closure(
                root, kind='LAUNCHER_SIGNAL',
                message='signal received during quiescence preflight',
                extra={'signal': int(received_signal['signum'])})
            return 128 + int(received_signal['signum'])
        try:
            preflight, q_sha = _load_pass_receipt(q_path)
        except LaunchError as error:
            _write_closure(
                root, kind='PREFLIGHT_FAIL_CLOSED', message=str(error),
                extra={'quiescence_returncode': q_returncode})
            return 10
        if q_returncode != 0:
            _write_closure(
                root, kind='PREFLIGHT_FAIL_CLOSED',
                message='quiescence command returned nonzero despite PASS receipt',
                preflight={'sha256': q_sha},
                extra={'quiescence_returncode': q_returncode})
            return 10
        output_dir = root / 'out'
        if output_dir.exists() or output_dir.is_symlink():
            raise LaunchError('OUTPUT_OVERWRITE', f'output directory already exists: {output_dir}')
        output_dir.mkdir()
        argv = build_runner_argv(config, output_dir)
        command_sha = hashlib.sha256(_json_bytes({'argv': argv})).hexdigest()
        launch_attempt = {
            'schema_version': SCHEMA_VERSION,
            'contract_id': CONTRACT_ID,
            'status': 'START_ATTEMPTED',
            'start_attempted': True,
            'requested_at_utc': now(),
            'parent_pid': os.getpid(),
            'command_sha256': command_sha,
            'argv': argv,
            'quiescence_receipt_sha256': q_sha,
            'quiescence_completed_monotonic_ns': quiescence_completed_ns,
            'maximum_preflight_to_run_gap_seconds': MAX_PREFLIGHT_TO_RUN_GAP_SECONDS,
            'identity': identity,
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
        }
        atomic_json(root / 'launch_attempt.json', launch_attempt)
        stdout_path = root / 'runner.stdout.log'
        stderr_path = root / 'runner.stderr.log'
        stdout = stdout_path.open('x', encoding='utf-8')
        stderr = stderr_path.open('x', encoding='utf-8')
        pre_popen_ns = time.monotonic_ns()
        pre_popen_gap_ns = pre_popen_ns - quiescence_completed_ns
        if pre_popen_gap_ns > MAX_PREFLIGHT_TO_RUN_GAP_NS:
            stdout.close()
            stderr.close()
            _write_closure(
                root, kind='PREFLIGHT_RUN_GAP_FAIL_CLOSED',
                message='runner start gap exceeded before Popen',
                preflight={'sha256': q_sha},
                extra={'gap_ns': pre_popen_gap_ns,
                       'maximum_gap_ns': MAX_PREFLIGHT_TO_RUN_GAP_NS})
            return 22
        try:
            child = process_factory(argv, stdin=subprocess.DEVNULL,
                                    stdout=stdout, stderr=stderr,
                                    close_fds=True)
        except Exception as error:
            stdout.close()
            stderr.close()
            _write_closure(root, kind='RUNNER_START_FAILURE', message=str(error),
                           preflight={'sha256': q_sha})
            return 20
        child_holder['process'] = child
        runner_started_ns = time.monotonic_ns()
        gap_ns = runner_started_ns - quiescence_completed_ns
        started = {
            'schema_version': SCHEMA_VERSION,
            'contract_id': CONTRACT_ID,
            'status': 'RUNNER_STARTED',
            'started_at_utc': now(),
            'runner_started_monotonic_ns': runner_started_ns,
            'preflight_to_run_gap_ns': gap_ns,
            'maximum_preflight_to_run_gap_ns': MAX_PREFLIGHT_TO_RUN_GAP_NS,
            'parent_pid': os.getpid(),
            'child_pid': int(child.pid),
            'command_sha256': command_sha,
            'quiescence_receipt_sha256': q_sha,
            'identity': identity,
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
        }
        try:
            atomic_json(root / 'launch_started.json', started)
        except Exception as error:
            child.terminate()
            child.wait()
            stdout.close()
            stderr.close()
            _write_closure(root, kind='LAUNCH_RECEIPT_FAILURE', message=str(error),
                           preflight={'sha256': q_sha},
                           extra={'child_pid': int(child.pid)})
            return 21
        if gap_ns > MAX_PREFLIGHT_TO_RUN_GAP_NS:
            try:
                child.terminate()
                child.wait()
            finally:
                stdout.close()
                stderr.close()
            _write_closure(
                root, kind='PREFLIGHT_RUN_GAP_FAIL_CLOSED',
                message='runner start exceeded the preregistered continuity gap',
                preflight={'sha256': q_sha},
                extra={'child_pid': int(child.pid), 'gap_ns': gap_ns,
                       'maximum_gap_ns': MAX_PREFLIGHT_TO_RUN_GAP_NS})
            return 22
        try:
            returncode = wait(child) if wait is not None else child.wait()
        except KeyboardInterrupt:
            try:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait()
                except BaseException:
                    pass
            finally:
                stdout.close()
                stderr.close()
            _write_closure(root, kind='LAUNCHER_SIGNAL', message='SIGINT received',
                           preflight={'sha256': q_sha},
                           extra={'child_pid': int(child.pid), 'signal': 'SIGINT'})
            return 130
        except BaseException as error:
            try:
                child.terminate()
                child.wait()
            finally:
                stdout.close()
                stderr.close()
            _write_closure(root, kind='LAUNCHER_EXCEPTION', message=repr(error),
                           preflight={'sha256': q_sha},
                           extra={'child_pid': int(child.pid)})
            return 70
        stdout.close()
        stderr.close()
        if returncode == 0:
            try:
                host_time = validate_host_time_report(root / 'time-v.txt')
                completion = (completion_validator or
                              (lambda path: validate_completed_output(
                                  path, repo_root=config.repo_root)))(output_dir)
            except Exception as error:
                _write_closure(
                    root, kind='COMPLETION_CONTRACT_FAIL_CLOSED', message=str(error),
                    preflight={'sha256': q_sha},
                    extra={
                        'child_pid': int(child.pid),
                        'runner_returncode': returncode,
                        'host_time_report_sha256': (
                            file_sha256(root / 'time-v.txt')
                            if (root / 'time-v.txt').is_file() else None),
                    })
                return 32
            status = 'COMPLETED'
            code = 0
        elif isinstance(returncode, int) and returncode < 0:
            status = 'RUNNER_SIGNAL_EXIT'
            code = 30
        else:
            status = 'RUNNER_EXITED_NONZERO'
            code = 31
        if received_signal['signum'] is not None:
            status = 'FAIL_CLOSED'
            status_kind = 'LAUNCHER_SIGNAL'
            code = 128 + int(received_signal['signum'])
        else:
            status_kind = status
        _write_closure(
            root, kind=status_kind, message=f'runner return code: {returncode}',
            preflight={'sha256': q_sha},
            extra={'child_pid': int(child.pid), 'runner_returncode': returncode,
                   'completion': completion if returncode == 0 else None,
                   'host_time_report_sha256': (
                       host_time['sha256'] if returncode == 0 else
                       (file_sha256(root / 'time-v.txt')
                        if (root / 'time-v.txt').is_file() else None))},
            status=status)
        return code
    except LaunchError as error:
        if marker is not None:
            try:
                _write_closure(root, kind=error.kind, message=str(error))
            except LaunchError:
                pass
        return 2
    except BaseException as error:
        if marker is not None:
            try:
                _write_closure(root, kind='LAUNCHER_EXCEPTION', message=repr(error))
            except LaunchError:
                pass
        return 70
    finally:
        for signum, handler in old_signal_handlers.items():
            signal.signal(signum, handler)


def _run_quiescence(config: LaunchConfig, receipt: Path) -> int:
    command = _quiescence_command(config, receipt)
    completed = subprocess.run(command, check=False)
    return int(completed.returncode)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--repo-root', type=Path,
                        default=Path(__file__).resolve().parents[1])
    parser.add_argument('--input-root', type=Path, default=DEFAULT_INPUT_ROOT)
    parser.add_argument('--quiescence-script', type=Path,
                        default=Path(__file__).with_name('check_m6a10_quiescence.py'))
    parser.add_argument('--sample-seconds', type=float, default=DEFAULT_SAMPLE_SECONDS)
    parser.add_argument('--max-busy-percent', type=float, default=DEFAULT_MAX_BUSY_PERCENT)
    parser.add_argument('--max-load-per-cpu', type=float, default=DEFAULT_MAX_LOAD_PER_CPU)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    config = LaunchConfig(
        root=args.root, repo_root=args.repo_root, input_root=args.input_root,
        quiescence_script=args.quiescence_script,
        sample_seconds=args.sample_seconds,
        max_busy_percent=args.max_busy_percent,
        max_load_per_cpu=args.max_load_per_cpu)
    return run_v7(config)


if __name__ == '__main__':
    sys.exit(main())
