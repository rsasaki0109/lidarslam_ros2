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

"""Plan and run the M6a fresh-holdout comparison without opening ground truth.

The default action is a read-only dry-run.  ``--preflight`` additionally
checks every frozen input identity and immutable local image.  Actual replay
requires ``--execute`` and is deliberately a separate action so a plan cannot
accidentally become a performance run.  No scorer, GT parser, or map-quality
tool is called by this driver.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Iterable

import yaml

# Direct source-checkout invocation must resolve the canonical package without
# requiring PYTHONPATH.  Installed entry points use the package normally.
_SCRIPT_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if (
        (_SCRIPT_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file()
        and (_SCRIPT_SOURCE_ROOT / 'scripts' / 'benchmark_phase_contract.py').is_file()
        and str(_SCRIPT_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SCRIPT_SOURCE_ROOT))

try:
    from lidarslam_benchmark_tools.competitive_identity_hash import (
        canonical_profile_sha256)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.competitive_identity_hash import (
        canonical_profile_sha256)

try:
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (
        current_rival_source_closure_identity,
        validate_rival_source_closure_receipt_identity)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (
        current_rival_source_closure_identity,
        validate_rival_source_closure_receipt_identity)

try:
    from lidarslam_benchmark_tools.benchmark_phase_contract import (
        CONTRACT_VERSION as PHASE_CONTRACT_VERSION,
        CONTRACT_VERSION_V2 as PHASE_CONTRACT_VERSION_V2,
        PhaseContractError as PhaseValidationError,
        validate_evidence as validate_phase_evidence,
        validate_evidence_v2 as validate_phase_evidence_v2,
    )
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.benchmark_phase_contract import (
        CONTRACT_VERSION as PHASE_CONTRACT_VERSION,
        CONTRACT_VERSION_V2 as PHASE_CONTRACT_VERSION_V2,
        PhaseContractError as PhaseValidationError,
        validate_evidence as validate_phase_evidence,
        validate_evidence_v2 as validate_phase_evidence_v2,
    )

try:
    from lidarslam_benchmark_tools.competitive_execution_attempt_receipt import (
        seal_receipt,
    )
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.competitive_execution_attempt_receipt import (
        seal_receipt,
    )


try:
    from lidarslam_benchmark_tools import package_root, resolve_benchmark_resource
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    def package_root() -> Path:
        return Path(__file__).resolve().parents[1]

    def resolve_benchmark_resource(resource: str, *, executable: bool = False) -> Path:
        """Resolve a source-checkout benchmark resource without fallback."""
        relative = Path(resource)
        if relative.is_absolute() or '..' in relative.parts:
            raise RuntimeError(f'unsafe benchmark resource: {resource!r}')
        if relative.parts and relative.parts[0] == 'scripts':
            relative = Path(*relative.parts[1:])
        candidate = (Path(__file__).resolve().parent / relative).resolve(strict=True)
        source_root = Path(__file__).resolve().parent
        if source_root not in candidate.parents or not candidate.is_file():
            raise RuntimeError(f'benchmark resource is outside source scripts: {resource!r}')
        if executable and not os.access(candidate, os.X_OK):
            raise RuntimeError(f'benchmark resource is not executable: {resource!r}')
        return candidate


ROOT = package_root()
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
DEFAULT_RECEIPT = ROOT / (
    'configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml')
DEFAULT_SELECTION = ROOT / (
    'configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml')
SYSTEM_ORDER = ('ours', 'glim_cpu', 'fast_livo2')
SLOT_ORDER = ('fresh_1', 'fresh_2', 'fresh_3')
SYSTEM_RECEIPT_NAMES = {
    'ours': 'ours', 'glim_cpu': 'glim', 'fast_livo2': 'fast_livo2'}
EXPECTED_LABELS = {
    'ours': ('benchmark.ours.revision', 'benchmark.rko_lio.initialized'),
    'glim_cpu': ('benchmark.glim.revision', 'benchmark.glim_ros2.revision'),
    'fast_livo2': ('benchmark.fast_livo2.revision',),
}
THREAD_ENV = {
    'OMP_NUM_THREADS': '8',
    'OPENBLAS_NUM_THREADS': '8',
    'MKL_NUM_THREADS': '8',
    'TBB_NUM_THREADS': '8',
}
DENY_WORDS = ('ground_truth', 'ground-truth', 'ape', 'scorer', 'map_quality')
SHA_RE = re.compile(r'^[0-9a-f]{64}$')
CONTAINER_SHA_RE = re.compile(r'^sha256:[0-9a-f]{64}$')
MEMORY_EVIDENCE_NAME = 'container_memory.json'
PROCESS_RSS_EVIDENCE_NAME = 'container_process_rss.json'
PROCESS_RSS_MEASUREMENT_VERSION = 'm6a7-container-process-rss-v1'
PHASE_EVIDENCE_NAME = 'phase_evidence.json'
PHASE_CONTRACT_FILE = resolve_benchmark_resource(
    'scripts/benchmark_phase_contract.py')
PHASE_HELPER_FILE = resolve_benchmark_resource(
    'scripts/container_phase_evidence.sh')
PROCESS_RSS_HELPER_FILE = resolve_benchmark_resource(
    'scripts/sample_container_process_rss.py')
MEMORY_HELPER_FILE = resolve_benchmark_resource(
    'scripts/container_memory_evidence.py')
PHASE_CONTRACT_CHOICES = ('v1', 'v2-paced', 'v2-unpaced')
PROCESS_RSS_PRIMARY_METRIC = 'aggregate_process_tree_peak_rss_bytes'
PROCESS_RSS_METRIC_DEFINITION = (
    'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted')
M6A7_REQUIRED_CONTRACT = {
    'schema_version': 1,
    'measurement_version': PROCESS_RSS_MEASUREMENT_VERSION,
    'measurement_scope': 'container_pid_namespace_proc_status',
    'primary_metric': PROCESS_RSS_PRIMARY_METRIC,
    'primary_metric_definition': PROCESS_RSS_METRIC_DEFINITION,
    'sampler_interval_ms': 250,
    'sampler_scheduler_nice': 10,
    'memory_max': 'max',
    'oom_delta_required': 0,
    'docker_client_comparable': False,
    'prior_failed_audit_lineage_required': True,
}


class ContractError(ValueError):
    """Raised when a plan or attempt would violate the frozen contract."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sha256_tree(path: Path) -> str:
    if not path.is_dir():
        raise ContractError(f'input tree is not a directory: {path}')
    digest = hashlib.sha256()
    files = sorted(item for item in path.rglob('*') if item.is_file())
    if not files:
        raise ContractError(f'input tree is empty: {path}')
    for item in files:
        digest.update(item.relative_to(path).as_posix().encode('utf-8'))
        digest.update(b'\0')
        with item.open('rb') as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


def canonical_sha(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(',', ':'),
                         ensure_ascii=True).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ContractError(f'{path}: expected a mapping')
    return value


def resolve_inside(root: Path, value: str, *, label: str) -> Path:
    path = Path(value)
    resolved = (path if path.is_absolute() else root / path).resolve(strict=True)
    root_resolved = root.resolve(strict=True)
    if not resolved.is_relative_to(root_resolved):
        raise ContractError(f'{label} escapes managed root: {value}')
    return resolved


def path_contains(parent: Path, child: Path) -> bool:
    try:
        child.resolve(strict=False).relative_to(parent.resolve(strict=False))
        return True
    except ValueError:
        return False


def assert_roots_are_disjoint(input_root: Path, output_root: Path) -> None:
    input_real = input_root.resolve(strict=True)
    output_real = output_root.resolve(strict=False)
    if input_real == output_real or path_contains(input_real, output_real) or \
            path_contains(output_real, input_real):
        raise ContractError('input and output roots must be disjoint and non-containing')


def schedule() -> list[dict[str, Any]]:
    result = []
    index = 1
    for system in SYSTEM_ORDER:
        for slot in SLOT_ORDER:
            for repetition in range(1, 4):
                result.append({'schedule_index': index, 'system': system,
                               'slot': slot, 'repetition': repetition})
                index += 1
    return result


def schedule_sha(value: Iterable[dict[str, Any]] | None = None) -> str:
    return canonical_sha(list(schedule() if value is None else value))


def selection_path(receipt: dict[str, Any], profile: dict[str, Any]) -> Path:
    common = receipt.get('common_identity')
    if not isinstance(common, dict):
        raise ContractError('receipt.common_identity is missing')
    value = common.get('fresh_holdout_selection_receipt_path')
    if not isinstance(value, str):
        value = profile.get('evidence_gate_v2', {}).get(
            'fresh_holdout_selection_receipt_path')
    if not isinstance(value, str):
        return DEFAULT_SELECTION
    return (ROOT / value).resolve() if not Path(value).is_absolute() else Path(value).resolve()


def _slot_identity(profile: dict[str, Any], slot_id: str) -> dict[str, Any]:
    slots = profile.get('datasets', {}).get('fresh_holdout_slots', {})
    slot = slots.get(slot_id)
    if not isinstance(slot, dict):
        raise ContractError(f'missing fresh holdout slot: {slot_id}')
    if slot.get('status') != 'frozen_unopened':
        raise ContractError(f'{slot_id} is not frozen_unopened')
    identity = slot.get('frozen_identity')
    if not isinstance(identity, dict):
        raise ContractError(f'{slot_id}.frozen_identity is missing')
    return slot


def resolve_slot(input_root: Path, profile: dict[str, Any], slot_id: str) -> dict[str, Any]:
    slot = _slot_identity(profile, slot_id)
    identity = slot['frozen_identity']
    raw = identity.get('raw_bag_path')
    if not isinstance(raw, str):
        raw = identity.get('raw_bag', {}).get('path')
    if not isinstance(raw, str):
        raise ContractError(f'{slot_id}: raw bag path missing')
    manifest_value = identity.get('manifest_path')
    if not isinstance(manifest_value, str):
        raise ContractError(f'{slot_id}: frozen manifest path missing')
    sequence = slot.get('sequence')
    if not isinstance(sequence, str):
        raise ContractError(f'{slot_id}: sequence missing')
    canonical = (input_root / 'slots' / sequence / 'canonical_ros2').resolve(strict=True)
    raw_path = resolve_inside(input_root, raw, label=f'{slot_id}.raw_bag_path')
    manifest = resolve_inside(input_root, manifest_value, label=f'{slot_id}.manifest_path')
    if not raw_path.is_file() or not manifest.is_file():
        raise ContractError(f'{slot_id}: raw bag or manifest is not a file')
    manifest_doc = load_yaml(manifest) if manifest.suffix in ('.yaml', '.yml') else json.loads(
        manifest.read_text(encoding='utf-8'))
    if not isinstance(manifest_doc, dict) or manifest_doc.get('status') != 'frozen_unopened':
        raise ContractError(f'{slot_id}: managed manifest is not frozen_unopened')
    expected_manifest_sha = identity.get('manifest_file_sha256')
    if expected_manifest_sha is None:
        expected_manifest_sha = identity.get('manifest_file_sha256')
    if not isinstance(expected_manifest_sha, str) or not SHA_RE.fullmatch(expected_manifest_sha):
        raise ContractError(f'{slot_id}: manifest SHA is missing or malformed')
    if sha256_file(manifest) != expected_manifest_sha:
        raise ContractError(f'{slot_id}: frozen manifest SHA mismatch')
    manifest_raw = manifest_doc.get('raw_bag')
    manifest_ros = manifest_doc.get('canonical_rosbag2')
    manifest_calibration = manifest_doc.get('calibration')
    expected_raw_sha = slot.get('bag_sha256') or identity.get('raw_bag_sha256') or \
        identity.get('raw_bag', {}).get('sha256')
    expected_ros_sha = identity.get('canonical_rosbag2_tree_sha256')
    expected_semantic_sha = identity.get('semantic_equivalence_sha256')
    expected_input_sha = slot.get('input_manifest_sha256')
    expected_calibration_sha = slot.get('calibration_archive_sha256')
    if not isinstance(manifest_raw, dict) or manifest_raw.get('sha256') != expected_raw_sha:
        raise ContractError(f'{slot_id}: raw bag manifest identity mismatch')
    if not isinstance(manifest_ros, dict) or \
            manifest_ros.get('canonical_rosbag2_tree_sha256') != expected_ros_sha or \
            manifest_ros.get('semantic_equivalence_sha256') != expected_semantic_sha:
        raise ContractError(f'{slot_id}: canonical/semantic manifest identity mismatch')
    if manifest_doc.get('semantic_equivalence_sha256') != expected_semantic_sha:
        raise ContractError(f'{slot_id}: semantic manifest identity mismatch')
    if manifest_doc.get('input_manifest_sha256') != expected_input_sha:
        raise ContractError(f'{slot_id}: input manifest identity mismatch')
    if not isinstance(manifest_calibration, dict) or \
            manifest_calibration.get('sha256') != expected_calibration_sha:
        raise ContractError(f'{slot_id}: calibration manifest identity mismatch')
    gt_value = identity.get('ground_truth_path')
    if not isinstance(gt_value, str):
        gt_value = identity.get('ground_truth', {}).get('path')
    if not isinstance(gt_value, str):
        raise ContractError(f'{slot_id}: ground-truth path missing from frozen identity')
    gt_path = resolve_inside(input_root, gt_value, label=f'{slot_id}.ground_truth_path')
    gt_stat = gt_path.stat()  # metadata only; contents are never opened or hashed
    raw_sha = expected_raw_sha
    raw_bytes = slot.get('bag_expected_bytes') or identity.get('raw_bag_bytes') or \
        identity.get('raw_bag', {}).get('bytes')
    return {
        'slot': slot_id, 'sequence': sequence, 'raw_path': raw_path,
        'canonical_path': canonical, 'manifest_path': manifest,
        'manifest_sha256': expected_manifest_sha, 'gt_realpath': gt_path,
        'gt_device': gt_stat.st_dev, 'gt_inode': gt_stat.st_ino,
        'raw_sha256': raw_sha, 'raw_bytes': raw_bytes,
        'canonical_tree_sha256': expected_ros_sha,
        'semantic_sha256': expected_semantic_sha,
        'input_manifest_sha256': expected_input_sha,
        'calibration_sha256': expected_calibration_sha,
    }


def image_ref_and_labels(
        system: str, receipt: dict[str, Any], *, inspect: bool
        ) -> tuple[str, dict[str, str], str]:
    name = SYSTEM_RECEIPT_NAMES[system]
    container = receipt.get('systems', {}).get(name, {}).get('container', {})
    tag = container.get('image_tag')
    digest = container.get('image_digest')
    if (not isinstance(tag, str) or not isinstance(digest, str) or
            not CONTAINER_SHA_RE.fullmatch(digest)):
        raise ContractError(f'{system}: immutable image tag/digest is incomplete')
    ref = f'{tag}@{digest}'
    if not inspect:
        return ref, {}, digest
    try:
        result = subprocess.run(
            ['docker', 'image', 'inspect', ref], check=True, text=True,
            capture_output=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        raise ContractError(f'{system}: immutable image unavailable: {ref}') from exc
    documents = json.loads(result.stdout)
    if not documents:
        raise ContractError(f'{system}: docker inspect returned no image: {ref}')
    document = documents[0]
    image_id = document.get('Id')
    if image_id != digest:
        raise ContractError(f'{system}: inspected image ID differs from receipt digest')
    labels = document.get('Config', {}).get('Labels') or {}
    expected = receipt.get('systems', {}).get(name, {}).get('repository', {})
    for label in EXPECTED_LABELS[system]:
        if label.endswith('initialized'):
            if labels.get(label) != 'true':
                raise ContractError(f'{system}: RKO-LIO image is not initialized')
        else:
            key = label
            expected_value = expected.get('revision') if system == 'ours' else None
            if system == 'glim_cpu':
                expected_value = (receipt['systems']['glim']['repository'].get(
                    'ros2_revision') if label == 'benchmark.glim_ros2.revision' else
                    receipt['systems']['glim']['repository'].get('revision'))
            if system == 'fast_livo2':
                expected_value = receipt['systems']['fast_livo2']['repository'].get('revision')
            if expected_value is not None and labels.get(key) != expected_value:
                raise ContractError(f'{system}: image label {key} does not match receipt')
    if system == 'ours':
        recipe = container.get('recipe')
        submodules = recipe.get('submodules') if isinstance(recipe, dict) else None
        rko = submodules.get('rko_lio') if isinstance(submodules, dict) else None
        ndt = submodules.get('ndt_omp_ros2') if isinstance(submodules, dict) else None
        expected_labels = {
            'benchmark.rko_lio.gitlink_revision':
                rko.get('gitlink_revision') if isinstance(rko, dict) else None,
            'benchmark.rko_lio.archive_sha256':
                rko.get('archive_sha256') if isinstance(rko, dict) else None,
            'benchmark.ndt_omp.submodule_revision':
                ndt.get('revision') if isinstance(ndt, dict) else None,
        }
        for label, expected_value in expected_labels.items():
            if not isinstance(expected_value, str) or labels.get(label) != expected_value:
                raise ContractError(f'ours: image label {label} does not match receipt')
    return ref, {str(key): str(value) for key, value in labels.items()}, digest


def _slot_mounts(item: dict[str, Any], system: str) -> list[tuple[Path, str, str]]:
    if system in ('ours', 'glim_cpu'):
        return [(item['canonical_path'], '/input/canonical_ros2', 'ro')]
    return [(item['raw_path'], '/input/raw_input.bag', 'ro')]


def _guard_gt(
        item: dict[str, Any], mounts: list[tuple[Path, str, str]],
        argv: list[str], env: dict[str, str]) -> dict[str, Any]:
    gt_path = item['gt_realpath']
    for source, _, _ in mounts:
        if path_contains(source, gt_path):
            raise ContractError(f'GT is reachable through {source}')
    serialized = '\0'.join(argv + [f'{key}={value}' for key, value in sorted(env.items())])
    lowered = serialized.lower()
    if any(word in lowered for word in DENY_WORDS):
        raise ContractError('GT/scorer/map-quality token leaked into container argv or env')
    if str(gt_path) in serialized:
        raise ContractError('frozen GT realpath leaked into container command')
    return {'ground_truth_reachable': False, 'gt_device': item['gt_device'],
            'gt_inode': item['gt_inode'], 'mount_sources': [str(x[0]) for x in mounts]}


def docker_command(
        system: str, item: dict[str, Any], image_ref: str,
        output_dir: Path, schedule_item: dict[str, Any],
        *, phase_contract: str = 'v1'
        ) -> tuple[list[str], dict[str, str]]:
    if phase_contract not in PHASE_CONTRACT_CHOICES:
        raise ContractError(f'unsupported phase contract selection: {phase_contract}')
    run_id = f'{system}-{schedule_item["schedule_index"]:03d}'
    env = dict(THREAD_ENV)
    phase_version = (PHASE_CONTRACT_VERSION_V2
                     if phase_contract != 'v1' else PHASE_CONTRACT_VERSION)
    phase_mode = ('paced_1x' if phase_contract != 'v2-unpaced' else 'unpaced_ack')
    env.update({'ROS_DOMAIN_ID': str(200 + schedule_item['schedule_index']),
                'RUN_NAME': run_id, 'GT_BLIND': '1',
                'M6A10_PHASE_CONTRACT_VERSION': phase_version,
                'M6A10_PHASE_MODE': phase_mode,
                'M6A10_MAX_END_GAP_SECONDS': '0.25',
                'M6A10_MAX_CALLBACK_LATENCY_SECONDS': '0.25',
                'M6A10_MAX_BACKLOG_MESSAGES': '0'})
    if system == 'glim_cpu':
        env.update({'ROS_HOME': '/out/ros_home', 'ROS_LOG_DIR': '/out/ros_log'})
    elif system == 'fast_livo2':
        env.update({'ROS_MASTER_URI': 'http://127.0.0.1:11311',
                    'ROS_IP': '127.0.0.1', 'ROS_HOSTNAME': '127.0.0.1'})
    common = [
        'taskset', '--cpu-list', '0-7', '/usr/bin/time', '-v',
        '-o', str(output_dir / 'host_time.txt'), 'docker', 'run', '--rm', '--init',
        '--network', 'none', '--cpuset-cpus', '0-7', '--read-only',
        '--tmpfs', '/tmp:rw,noexec,nosuid,size=1024m', '--shm-size', '512m']
    for key, value in env.items():
        common.extend(['-e', f'{key}={value}'])
    common.extend(['-v', f'{ROOT}:/runner:ro', '-v', f'{output_dir}:/out:rw'])
    if system == 'ours':
        common.extend([
            '-e', 'BAG_PATH=/input/canonical_ros2',
            '-e', 'OUT_DIR=/out', '-e', 'LIDAR_TOPIC=/hesai/pandar',
            '-e', 'IMU_TOPIC=/alphasense/imu', '-e', 'LIDARSLAM_WS_ROOT=/opt/ours_ws',
            '-v', f"{item['canonical_path']}:/input/canonical_ros2:ro",
            '--entrypoint', '/bin/bash', image_ref,
            '/runner/scripts/ours_container_gt_blind_run.sh'])
    elif system == 'glim_cpu':
        common.extend([
            '-e', 'BAG_PATH=/input/canonical_ros2',
            '-v', f"{item['canonical_path']}:/input/canonical_ros2:ro",
            '--entrypoint', '/bin/bash', image_ref,
            '/runner/scripts/glim_container_gt_blind_run.sh'])
    else:
        common.extend([
            '-e', 'BAG_PATH=/input/raw_input.bag', '-e', 'RATE=1.0',
            '-e', 'SHUTDOWN_GRACE_SECONDS=5', '-e', 'SAVE_MAP=1',
            '-v', f"{item['raw_path']}:/input/raw_input.bag:ro",
            '-v', f'{output_dir}/fast_log:/bench/FAST-LIVO2/Log:rw',
            '--entrypoint', '/bin/bash', image_ref,
            '/runner/scripts/fast_livo2_container_gt_blind_run.sh'])
    return common, env


def system_identity(receipt: dict[str, Any], system: str) -> dict[str, Any]:
    name = SYSTEM_RECEIPT_NAMES[system]
    document = receipt.get('systems', {}).get(name)
    if not isinstance(document, dict):
        raise ContractError(f'{system}: receipt system identity is missing')
    repository = document.get('repository')
    container = document.get('container')
    toolchain = document.get('toolchain')
    if not isinstance(repository, dict) or not isinstance(container, dict) or \
            not isinstance(toolchain, dict):
        raise ContractError(f'{system}: repository/container/toolchain identity is incomplete')
    revision = repository.get('revision')
    config = document.get('configs')
    if not isinstance(revision, str) or not isinstance(config, list):
        raise ContractError(f'{system}: revision/config identity is incomplete')
    return {
        'revision': revision,
        'revision_status': repository.get('revision_status'),
        'container_digest': container.get('image_digest'),
        'toolchain_fingerprint': toolchain.get('fingerprint'),
        'config': config,
        'runner': document.get('runner'),
        'recipe': container.get('recipe'),
    }


def m6a7_contract_identity(receipt: dict[str, Any]) -> dict[str, Any]:
    """Return the immutable process-RSS contract required by campaign4.

    This checks receipt metadata only.  The external M6a7 audit is separately
    hashed and referenced; no input, calibration, or ground-truth path is
    opened here.
    """
    contract = receipt.get('m6a7_process_rss_contract')
    if not isinstance(contract, dict):
        raise ContractError('receipt.m6a7_process_rss_contract is missing')
    if contract.get('status') != 'PASS':
        raise ContractError('M6a7 process-RSS audit is not PASS')
    for key, expected in M6A7_REQUIRED_CONTRACT.items():
        if contract.get(key) != expected:
            raise ContractError(
                f'M6a7 process-RSS contract {key} is not {expected!r}')
    result = {key: contract[key] for key in M6A7_REQUIRED_CONTRACT}
    for name in ('final_audit', 'final_receipt', 'summary'):
        item = contract.get(name)
        if not isinstance(item, dict) or not isinstance(item.get('path'), str) or \
                not SHA_RE.fullmatch(str(item.get('sha256', ''))):
            raise ContractError(f'M6a7 {name} path/SHA is missing')
        path = Path(item['path'])
        if not path.is_file() or sha256_file(path) != item['sha256']:
            raise ContractError(f'M6a7 {name} path/SHA does not match')
        result[name] = {'path': str(path), 'sha256': item['sha256']}
    schedule = contract.get('schedule')
    if not isinstance(schedule, dict) or schedule.get('order') != 'AB_BA_alternating' or \
            schedule.get('pairs') != 20 or schedule.get('runs') != 40 or \
            schedule.get('all_complete') is not True:
        raise ContractError('M6a7 schedule is not the complete 20-pair/40-run audit')
    blind_scope = contract.get('blind_scope')
    if not isinstance(blind_scope, dict) or \
            blind_scope.get('ground_truth_content_opened') is not False or \
            blind_scope.get('scorer_invoked') is not False or \
            blind_scope.get('campaign4_started') is not False:
        raise ContractError('M6a7 audit is not GT/scorer/campaign4 blind')
    lineage = contract.get('lineage')
    failed = lineage.get('prior_failed_audit_roots') \
        if isinstance(lineage, dict) else None
    roots = failed.get('roots') if isinstance(failed, dict) else None
    if (not isinstance(lineage, dict) or
            lineage.get('prior_failed_audits_retained') is not True or
            lineage.get('campaign4_authorized') is not False or
            not isinstance(failed, dict) or failed.get('status') != 'FAIL_CLOSED' or
            failed.get('immutable') is not True or not isinstance(roots, list) or
            any(not Path(item).is_dir() for item in roots)):
        raise ContractError('M6a7 prior failed-audit lineage is not retained')
    result['schedule'] = dict(schedule)
    result['blind_scope'] = dict(blind_scope)
    result['lineage'] = dict(lineage)
    return result


def verify_input_identity(item: dict[str, Any], checked_slots: set[str]) -> None:
    """Verify one frozen slot at most once during a multi-run preflight.

    The schedule deliberately repeats each slot for every system and
    repetition.  Re-reading the same raw/canonical trees for every attempt
    would multiply a large, read-only hash operation by nine without adding
    evidence; the slot identity is immutable for the entire plan.
    """
    slot = item['slot']
    if slot in checked_slots:
        return
    actual_raw_bytes = item['raw_path'].stat().st_size
    if actual_raw_bytes != item['raw_bytes']:
        raise ContractError(f'{slot}: raw bag byte count mismatch')
    if sha256_file(item['raw_path']) != item['raw_sha256']:
        raise ContractError(f'{slot}: raw bag SHA mismatch')
    if sha256_tree(item['canonical_path']) != item['canonical_tree_sha256']:
        raise ContractError(f'{slot}: canonical ROS2 tree SHA mismatch')
    checked_slots.add(slot)


def build_plan(profile_doc: dict[str, Any], receipt: dict[str, Any], selection: dict[str, Any],
               input_root: Path, profile_path: Path, receipt_path: Path,
               selection_path_value: Path, *, inspect_images: bool,
               phase_contract: str = 'v1') -> dict[str, Any]:
    if phase_contract not in PHASE_CONTRACT_CHOICES:
        raise ContractError(f'unsupported phase contract selection: {phase_contract}')
    profile = profile_doc.get('competitive_slam_profile', profile_doc)
    if not isinstance(profile, dict):
        raise ContractError('profile has no competitive_slam_profile')
    if receipt.get('status') not in ('ready', 'frozen'):
        raise ContractError('execution identity receipt is not ready/frozen')
    try:
        closure_identity = current_rival_source_closure_identity(
            {'competitive_slam_profile': profile}, root=ROOT)
    except (OSError, ValueError, TypeError, UnicodeError, yaml.YAMLError) as exc:
        raise ContractError(f'current rival source closure is invalid: {exc}') from exc
    closure_errors = validate_rival_source_closure_receipt_identity(
        receipt, closure_identity)
    if closure_errors:
        raise ContractError(
            'execution identity receipt is not bound to current r2 rival source '
            'closure: ' + '; '.join(closure_errors))
    if selection.get('status') != 'frozen_unopened':
        raise ContractError('fresh selection receipt is not frozen_unopened')
    profile_sha = canonical_profile_sha256(profile_doc)
    receipt_sha = sha256_file(receipt_path)
    selection_sha = sha256_file(selection_path_value)
    expected_selection_sha = receipt.get('common_identity', {}).get(
        'fresh_holdout_selection_receipt_sha256')
    if expected_selection_sha != selection_sha:
        raise ContractError('selection receipt SHA differs from execution receipt')
    expected_profile_sha = receipt.get('common_identity', {}).get('profile_sha256')
    if expected_profile_sha != profile_sha:
        raise ContractError('profile canonical SHA differs from execution receipt')
    process_rss_contract = m6a7_contract_identity(receipt)
    image_info = {}
    for system in SYSTEM_ORDER:
        image_info[system] = image_ref_and_labels(
            system, receipt, inspect=inspect_images)
    plans = []
    checked_input_slots: set[str] = set()
    for schedule_item in schedule():
        item = resolve_slot(input_root, profile, schedule_item['slot'])
        if inspect_images:
            verify_input_identity(item, checked_input_slots)
        image_ref, labels, image_digest = image_info[schedule_item['system']]
        execution_identity = system_identity(receipt, schedule_item['system'])
        output_placeholder = Path('/M6A_OUTPUT_PLACEHOLDER')
        command, env = docker_command(
            schedule_item['system'], item, image_ref, output_placeholder,
            schedule_item, phase_contract=phase_contract)
        mounts = _slot_mounts(item, schedule_item['system'])
        guard = _guard_gt(item, mounts, command, env)
        plans.append({
            **schedule_item, 'sequence': item['sequence'],
            'rival_source_closure': closure_identity,
            'image_ref': image_ref, 'image_digest': image_digest,
            'image_labels': labels, 'argv': command, 'env': env,
            'execution_identity': execution_identity,
            'mounts': [{'source': str(source), 'target': target, 'mode': mode}
                       for source, target, mode in mounts],
            'input': {
                'raw_bag_sha256': item['raw_sha256'],
                'raw_bag_bytes': item['raw_bytes'],
                'canonical_rosbag2_tree_sha256': item['canonical_tree_sha256'],
                'semantic_equivalence_sha256': item['semantic_sha256'],
                'input_manifest_sha256': item['input_manifest_sha256'],
                'calibration_tree_sha256': item['calibration_sha256'],
                'manifest_file_sha256': item['manifest_sha256'],
            }, 'gt_blind_guard': guard,
        })
    identity = {
        'rival_source_closure': closure_identity,
        'profile_canonical_sha256': profile_sha,
        'execution_receipt_file_sha256': receipt_sha,
        'selection_receipt_file_sha256': selection_sha,
        'schedule_sha256': schedule_sha(),
        'systems': list(SYSTEM_ORDER), 'slots': list(SLOT_ORDER),
        'repetitions': 3, 'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'm6a7_process_rss_contract': process_rss_contract,
        'm6a10_phase_contract': {
            'schema_version': 1,
            'contract_version': PHASE_CONTRACT_VERSION,
            'maximum_online_compute_rtf': 1.0,
            'phase_contract_sha256': sha256_file(PHASE_CONTRACT_FILE),
            'phase_helper_sha256': sha256_file(PHASE_HELPER_FILE),
            'primary_metric': 'runtime.online_compute_rtf',
            'wall_metric': 'runtime.wall_realtime_factor',
            'wall_metric_role': 'diagnostic_only',
        },
        'm6a10_phase_contract_v2': {
            'schema_version': 2,
            'contract_version': PHASE_CONTRACT_VERSION_V2,
            'phase_selection': phase_contract,
            'paced_primary_metric': 'runtime.paced_followability_passed',
            'unpaced_primary_metric': 'runtime.unpaced_throughput_gate_passed',
            'wall_metric_role': 'diagnostic_only',
            'requires_consumer_ack': True,
            'publisher_counts_are_not_acks': True,
        },
    }
    identity['campaign_id'] = canonical_sha({
        'profile_canonical_sha256': profile_sha,
        'execution_receipt_file_sha256': receipt_sha,
        'selection_receipt_file_sha256': selection_sha,
        'rival_source_closure': closure_identity,
        'schedule_sha256': identity['schedule_sha256'],
        'systems': identity['systems'],
        'slots': identity['slots'],
        'repetitions': identity['repetitions'],
        'm6a7_process_rss_contract': process_rss_contract,
        'm6a10_phase_contract_v2': identity['m6a10_phase_contract_v2'],
    })
    return {'schema_version': 1, 'kind': 'm6a_gt_blind_plan',
            'status': 'preflight_ready' if inspect_images else 'planned',
            'identity': identity,
            'paths': {'input_root': str(input_root.resolve()),
                      'profile': str(profile_path), 'receipt': str(receipt_path),
                      'selection': str(selection_path_value)},
            'attempts': plans}


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def parse_time_report(path: Path) -> dict[str, float | int | None]:
    if not path.is_file():
        return {'wall_seconds': None, 'user_seconds': None,
                'sys_seconds': None, 'docker_client_peak_rss_kb': None}
    text = path.read_text(errors='replace')
    elapsed = re.search(r'Elapsed \(wall clock\) time .*?:\s*([0-9:.]+)', text)
    user = re.search(r'User time \(seconds\):\s*([0-9.]+)', text)
    system = re.search(r'System time \(seconds\):\s*([0-9.]+)', text)
    rss = re.search(r'Maximum resident set size \(kbytes\):\s*(\d+)', text)
    wall = None
    if elapsed:
        values = [float(item) for item in elapsed.group(1).split(':')]
        wall = sum(value * (60 ** index) for index, value in
                   enumerate(reversed(values)))
    return {'wall_seconds': wall,
            'user_seconds': float(user.group(1)) if user else None,
            'sys_seconds': float(system.group(1)) if system else None,
            'docker_client_peak_rss_kb': int(rss.group(1)) if rss else None}


def parse_phase_evidence(
        path: Path, *, expected_contract_version: str | None = None,
        expected_phase_mode: str | None = None) -> dict[str, Any]:
    """Validate wrapper phase evidence without opening GT or scorer inputs."""
    value = _load_json_object(path)
    if value is None:
        return {'valid': False, 'reason': 'missing_or_unreadable_phase_evidence'}
    if (expected_contract_version is not None and
            value.get('contract_version') != expected_contract_version):
        return {
            'valid': False,
            'reason': 'phase contract version does not match scheduled mode',
            'expected_contract_version': expected_contract_version,
            'observed_contract_version': value.get('contract_version'),
            'document': value,
        }
    if (expected_phase_mode is not None and
            value.get('phase_mode') != expected_phase_mode):
        return {
            'valid': False,
            'reason': 'phase mode does not match scheduled mode',
            'expected_phase_mode': expected_phase_mode,
            'observed_phase_mode': value.get('phase_mode'),
            'document': value,
        }
    try:
        validator = (validate_phase_evidence_v2
                     if value.get('contract_version') == PHASE_CONTRACT_VERSION_V2
                     else validate_phase_evidence)
        checked = validator(value, maximum_online_rtf=1.0, require_pass=True)
    except (PhaseValidationError, TypeError, ValueError) as error:
        return {'valid': False, 'reason': str(error), 'document': value}
    checked['valid'] = True
    checked['reason'] = ''
    return checked


def _load_json_object(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def parse_process_rss_evidence(path: Path) -> dict[str, Any]:
    """Validate process-tree RSS; client ``time -v`` is never accepted here."""
    value = _load_json_object(path)
    if value is None:
        return {'valid': False, 'reason': 'missing_or_unreadable_process_rss'}
    result = dict(value)
    result['valid'] = False
    if value.get('measurement_version') != PROCESS_RSS_MEASUREMENT_VERSION or \
            value.get('measurement_scope') != 'container_pid_namespace_proc_status':
        result['reason'] = 'process_rss_version_or_scope_invalid'
        return result
    if value.get('status') != 'pass' or value.get('atomic') is not True or \
            value.get('sampler_excluded') is not True:
        result['reason'] = 'process_rss_sampler_status_invalid'
        return result
    sampler_pid = value.get('sampler_pid')
    first_stamp = value.get('first_sample_monotonic_ns')
    last_stamp = value.get('last_sample_monotonic_ns')
    if any(isinstance(item, bool) or not isinstance(item, int) or item < 0
           for item in (sampler_pid, first_stamp, last_stamp)) or \
            sampler_pid == 0 or last_stamp < first_stamp:
        result['reason'] = 'process_rss_timestamps_invalid'
        return result
    peak = value.get('peak')
    required = ('vmrss_bytes', 'rss_anon_bytes', 'rss_file_bytes',
                'rss_shmem_bytes', 'process_count')
    if not isinstance(peak, dict) or any(
            isinstance(peak.get(key), bool) or not isinstance(peak.get(key), int) or
            peak.get(key) < 0 for key in required):
        result['reason'] = 'process_rss_peak_fields_invalid'
        return result
    aggregate = value.get(PROCESS_RSS_PRIMARY_METRIC)
    if value.get('primary_metric') != PROCESS_RSS_PRIMARY_METRIC or \
            value.get('primary_metric_definition') != \
            PROCESS_RSS_METRIC_DEFINITION or \
            isinstance(aggregate, bool) or not isinstance(aggregate, int) or \
            aggregate < 0 or aggregate != peak['vmrss_bytes']:
        result['reason'] = 'process_rss_primary_metric_invalid'
        return result
    sample_count = value.get('sample_count')
    if isinstance(sample_count, bool) or not isinstance(sample_count, int) or \
            sample_count < 2:
        result['reason'] = 'process_rss_sample_count_invalid'
        return result
    for key in ('sample_errors', 'pid_race_skips', 'missed_intervals'):
        if isinstance(value.get(key), bool) or not isinstance(value.get(key), int) or \
                value.get(key) < 0:
            result['reason'] = f'process_rss_{key}_invalid'
            return result
    thresholds = value.get('thresholds')
    if not isinstance(thresholds, dict):
        result['reason'] = 'process_rss_thresholds_missing'
        return result
    min_samples = thresholds.get('min_samples')
    if isinstance(min_samples, bool) or not isinstance(min_samples, int) or \
            min_samples < 1 or sample_count < min_samples:
        result['reason'] = 'process_rss_min_sample_threshold_invalid'
        return result
    max_errors = thresholds.get('max_errors')
    max_races = thresholds.get('max_race_skips')
    if any(isinstance(item, bool) or not isinstance(item, int) or item < 0
           for item in (max_errors, max_races)) or \
            value['sample_errors'] > max_errors or value['pid_race_skips'] > max_races:
        result['reason'] = 'process_rss_error_threshold_exceeded'
        return result
    if value.get('missed_intervals') != 0:
        result['reason'] = 'process_rss_missed_interval'
        return result
    jitter = value.get('interval_jitter_percent')
    max_jitter = thresholds.get('max_jitter_percent')
    if isinstance(jitter, bool) or not isinstance(jitter, (int, float)) or \
            not math.isfinite(float(jitter)) or jitter < 0 or \
            isinstance(max_jitter, bool) or not isinstance(max_jitter, (int, float)) or \
            not math.isfinite(float(max_jitter)) or max_jitter < 0 or \
            jitter > max_jitter:
        result['reason'] = 'process_rss_jitter_invalid'
        return result
    result['valid'] = True
    result['reason'] = ''
    result[PROCESS_RSS_PRIMARY_METRIC] = aggregate
    # Keep the old key as a read-only compatibility alias. New receipts must
    # use the explicit aggregate metric name above.
    result['process_tree_peak_rss_bytes'] = aggregate
    return result


def parse_container_memory_evidence(path: Path) -> dict[str, Any]:
    """Validate cgroup diagnostics and its linked process RSS evidence."""
    value = _load_json_object(path)
    if value is None:
        return {'valid': False, 'reason': 'missing_or_unreadable_memory_evidence'}
    result = dict(value)
    result['valid'] = False
    if value.get('status') != 'pass':
        result['reason'] = 'memory_measurement_status_not_pass'
        return result
    if value.get('measurement_version') != 'm6a7-container-memory-v2' or \
            value.get('measurement_scope') != 'container_cgroup_v2_with_pid_rss':
        result['reason'] = 'memory_measurement_scope_or_version_invalid'
        return result
    if value.get('cgroup_version') != 2 or \
            value.get('children_included') is not True or \
            value.get('atomic') is not True:
        result['reason'] = 'memory_cgroup_scope_or_atomic_contract_invalid'
        return result
    if not value.get('cgroup_path') or not value.get('proc_self_cgroup'):
        result['reason'] = 'memory_cgroup_path_missing'
        return result
    readability = value.get('output_readability')
    if not isinstance(readability, dict) or readability.get('status') != 'pass':
        result['reason'] = 'output_tree_readability_contract_invalid'
        return result
    required = ('container_cgroup_peak_bytes', 'memory_current_bytes')
    values = [value.get(key) for key in required]
    if any(isinstance(item, bool) or not isinstance(item, int) or item < 0
           for item in values):
        result['reason'] = 'memory_measurement_value_missing_or_non_numeric'
        return result
    peak, current = values
    max_raw = value.get('memory_max_raw')
    max_bytes = value.get('memory_max_bytes')
    if max_raw == 'max':
        if value.get('memory_max_unlimited') is not True or max_bytes is not None:
            result['reason'] = 'unlimited_memory_max_fields_inconsistent'
            return result
    elif isinstance(max_raw, str) and re.fullmatch(r'[0-9]+', max_raw):
        if value.get('memory_max_unlimited') is not False or \
                isinstance(max_bytes, bool) or not isinstance(max_bytes, int) or \
                max_bytes <= 0 or int(max_raw) != max_bytes or current > max_bytes:
            result['reason'] = 'numeric_memory_max_fields_inconsistent'
            return result
    else:
        result['reason'] = 'memory_max_missing_or_non_numeric'
        return result
    if peak < current:
        result['reason'] = 'memory_measurement_range_invalid'
        return result
    events = value.get('cgroup_events')
    if not isinstance(events, dict) or events.get('status') != 'pass' or \
            events.get('oom_free') is not True:
        result['reason'] = 'cgroup_events_or_oom_contract_invalid'
        return result
    for name in ('memory_events', 'memory_events_local', 'memory_pressure'):
        record = events.get(name)
        if not isinstance(record, dict) or not all(
                isinstance(record.get(key), dict)
                for key in ('baseline', 'final', 'delta')):
            result['reason'] = f'cgroup_event_{name}_record_invalid'
            return result
        if set(record['baseline']) != set(record['final']) or \
                set(record['baseline']) != set(record['delta']):
            result['reason'] = f'cgroup_event_{name}_keys_invalid'
            return result
        if name == 'memory_pressure' and set(record['baseline']) != {'some', 'full'}:
            result['reason'] = 'cgroup_pressure_keys_invalid'
            return result
        for field in record['baseline']:
            before = record['baseline'][field]
            after = record['final'][field]
            difference = record['delta'][field]
            if isinstance(before, dict):
                if not isinstance(after, dict) or not isinstance(difference, dict) or \
                        set(before) != set(after) or set(before) != set(difference):
                    result['reason'] = f'cgroup_event_{name}_nested_invalid'
                    return result
                for nested in before:
                    values = (before[nested], after[nested], difference[nested])
                    if any(isinstance(item, bool) or
                           not isinstance(item, (int, float)) or
                           not math.isfinite(float(item)) or item < 0
                           for item in values):
                        result['reason'] = f'cgroup_event_{name}_value_invalid'
                        return result
            else:
                values = (before, after, difference)
                if any(isinstance(item, bool) or
                       not isinstance(item, (int, float)) or
                       not math.isfinite(float(item)) or item < 0
                       for item in values):
                    result['reason'] = f'cgroup_event_{name}_value_invalid'
                    return result
    oom_delta = events.get('oom_delta')
    if not isinstance(oom_delta, dict) or any(
            isinstance(item, bool) or not isinstance(item, int) or item != 0
            for item in oom_delta.values()):
        result['reason'] = 'cgroup_oom_delta_invalid'
        return result
    embedded = value.get('process_rss_evidence')
    if not isinstance(embedded, dict):
        result['reason'] = 'embedded_process_rss_missing'
        return result
    if value.get('primary_metric') != PROCESS_RSS_PRIMARY_METRIC or \
            value.get('primary_metric_definition') != \
            PROCESS_RSS_METRIC_DEFINITION or \
            value.get('process_rss_metric_definition') != \
            PROCESS_RSS_METRIC_DEFINITION:
        result['reason'] = 'memory_primary_metric_contract_invalid'
        return result
    embedded_path = path.with_name(PROCESS_RSS_EVIDENCE_NAME)
    process = parse_process_rss_evidence(embedded_path)
    if not process['valid']:
        result['reason'] = f'process_rss_evidence_invalid:{process["reason"]}'
        return result
    aggregate = value.get(PROCESS_RSS_PRIMARY_METRIC)
    if isinstance(aggregate, bool) or not isinstance(aggregate, int) or \
            aggregate < 0 or \
            embedded.get('peak', {}).get('vmrss_bytes') != aggregate or \
            process.get(PROCESS_RSS_PRIMARY_METRIC) != aggregate or \
            value.get('process_tree_peak_rss_bytes') != aggregate:
        result['reason'] = 'process_rss_embedded_or_primary_mismatch'
        return result
    result['valid'] = True
    result['reason'] = ''
    result[PROCESS_RSS_PRIMARY_METRIC] = process[PROCESS_RSS_PRIMARY_METRIC]
    result['process_tree_peak_rss_bytes'] = process[PROCESS_RSS_PRIMARY_METRIC]
    return result


def comparison_rss_bytes(attempt: dict[str, Any]) -> int:
    """Return the only RSS value permitted for comparison gates."""
    value = attempt.get(PROCESS_RSS_PRIMARY_METRIC)
    if value is None:
        value = attempt.get('process_tree_peak_rss_bytes')
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ContractError('process-tree peak RSS is missing or invalid')
    return value


def output_tree_hash(path: Path) -> str | None:
    files = sorted(item for item in path.rglob('*') if item.is_file())
    if not files:
        return None
    digest = hashlib.sha256()
    for item in files:
        digest.update(item.relative_to(path).as_posix().encode('utf-8'))
        digest.update(b'\0')
        digest.update(hashlib.sha256(item.read_bytes()).digest())
    return digest.hexdigest()


def artifact_hashes(path: Path) -> dict[str, str]:
    result = {}
    for item in sorted(path.rglob('*')):
        if not item.is_file() or item.name in {'host_time.txt', 'stdout.log', 'stderr.log'}:
            continue
        if item.name in {'traj_raw.tum', 'traj_corrected.tum', 'traj_lidar.txt',
                         'odometry.csv'} or item.suffix.lower() in {'.pcd', '.ply'}:
            result[item.relative_to(path).as_posix()] = sha256_file(item)
    return result


def write_json_atomic(path: Path, value: Any) -> None:
    """Publish a root-level failure record without exposing a partial JSON."""
    if path.exists():
        raise ContractError(f'failure artifact already exists: {path}')
    part = path.with_name(path.name + '.part')
    if part.exists():
        raise ContractError(f'failure artifact part already exists: {part}')
    part.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n',
                    encoding='utf-8')
    os.replace(part, path)


def write_driver_failure(output_root: Path, plan: dict[str, Any],
                         part: Path, started: dt.datetime,
                         error: OSError) -> Path:
    index = plan['schedule_index']
    failure_path = output_root / f'attempt_{index:03d}.driver_failure.json'
    failure = {
        'schema_version': 1,
        'kind': 'm6a_gt_blind_driver_failure',
        'status': 'INCOMPLETE',
        'failure_stage': 'attempt_artifact_finalization',
        'schedule': {key: plan[key] for key in
                     ('schedule_index', 'system', 'slot', 'sequence',
                      'repetition')},
        'started_at': started.isoformat(),
        'observed_at': dt.datetime.now(dt.timezone.utc).isoformat(),
        'error_type': type(error).__name__,
        'error': str(error),
        'attempt_part': str(part),
        'attempt_final': str(output_root / f'attempt_{index:03d}'),
        'attempt_finalized': False,
        'preserve_part': True,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    write_json_atomic(failure_path, failure)
    return failure_path


def expected_outputs(system: str, output: Path,
                     phase_contract_version: str | None = None) -> list[Path]:
    memory = output / MEMORY_EVIDENCE_NAME
    process_rss = output / PROCESS_RSS_EVIDENCE_NAME
    phase = output / PHASE_EVIDENCE_NAME
    consumer = output / 'consumer_evidence.json'
    if system == 'ours':
        expected = [output / 'traj_raw.tum', memory, process_rss, phase]
        if phase_contract_version == PHASE_CONTRACT_VERSION_V2:
            expected.append(consumer)
        return expected
    if system == 'glim_cpu':
        return [output / 'dump' / 'traj_lidar.txt', memory, process_rss, phase]
    return [output / 'odometry.csv', memory, process_rss, phase]


def run_attempt(plan: dict[str, Any], output_root: Path,
                timeout_seconds: float) -> dict[str, Any]:
    index = plan['schedule_index']
    final = output_root / f'attempt_{index:03d}'
    part = output_root / f'attempt_{index:03d}.part'
    if final.exists() or part.exists():
        raise ContractError(f'attempt already exists: {index}')
    part.mkdir(parents=True)
    command = list(plan['argv'])
    command = [token.replace('/M6A_OUTPUT_PLACEHOLDER', str(part))
               for token in command]
    plan['argv'] = command
    started = dt.datetime.now(dt.timezone.utc)
    exit_status = None
    timed_out = False
    signal = None
    try:
        with (part / 'stdout.log').open('w') as stdout, \
                (part / 'stderr.log').open('w') as stderr:
            try:
                completed = subprocess.run(
                    command, cwd=ROOT, stdout=stdout, stderr=stderr,
                    check=False, timeout=timeout_seconds)
                exit_status = completed.returncode
            except subprocess.TimeoutExpired:
                timed_out = True
                exit_status = 124
    except OSError as exc:
        (part / 'driver_error.txt').write_text(str(exc) + '\n', encoding='utf-8')
        exit_status = 127
    finished = dt.datetime.now(dt.timezone.utc)
    expected_phase_contract_version = plan.get('env', {}).get(
        'M6A10_PHASE_CONTRACT_VERSION')
    expected = expected_outputs(plan['system'], part,
                                expected_phase_contract_version)
    proof = dict(plan['gt_blind_guard'])
    proof.update({'ground_truth_content_opened': False, 'scorer_invoked': False,
                  'argv_gt_free': True, 'env_gt_free': True, 'mounts_gt_free': True})
    try:
        write_json(part / 'gt_blind_proof.json', proof)
        process_rss_evidence = parse_process_rss_evidence(
            part / PROCESS_RSS_EVIDENCE_NAME)
        memory_evidence = parse_container_memory_evidence(
            part / MEMORY_EVIDENCE_NAME)
        expected_phase_mode = (plan.get('env', {}).get('M6A10_PHASE_MODE')
                               if expected_phase_contract_version ==
                               PHASE_CONTRACT_VERSION_V2 else None)
        phase_evidence = parse_phase_evidence(
            part / PHASE_EVIDENCE_NAME,
            expected_contract_version=expected_phase_contract_version,
            expected_phase_mode=expected_phase_mode)
        complete = not timed_out and exit_status == 0 and \
            all(path.is_file() for path in expected) and \
            process_rss_evidence['valid'] and memory_evidence['valid'] and \
            phase_evidence['valid']
        timing = parse_time_report(part / 'host_time.txt')
        phase_document = phase_evidence.get('document', phase_evidence)
        input_duration = phase_document.get('input_duration_seconds') \
            if isinstance(phase_document, dict) else None
        wall_rtf = None
        if isinstance(input_duration, (int, float)) and not isinstance(
                input_duration, bool) and input_duration > 0 and \
                isinstance(timing.get('wall_seconds'), (int, float)):
            wall_rtf = timing['wall_seconds'] / float(input_duration)
        phase_runtime = phase_evidence.get('runtime', {})
        v2_contract = (phase_document.get('contract_version') ==
                       PHASE_CONTRACT_VERSION_V2
                       if isinstance(phase_document, dict) else False)
        if v2_contract:
            phase_gate_passed = bool(
                phase_runtime.get('paced_followability_passed') or
                phase_runtime.get('unpaced_throughput_gate_passed'))
        else:
            phase_gate_passed = bool(
                phase_runtime.get('online_compute_rtf_gate_passed', False))
        report = {
            'schema_version': 1, 'kind': 'm6a_gt_blind_attempt',
            'schedule': {key: plan[key] for key in
                         ('schedule_index', 'system', 'slot', 'sequence',
                          'repetition')},
            'identity': {
                'campaign_id': plan.get('campaign_id'),
                'profile_canonical_sha256': plan.get('profile_canonical_sha256'),
                'execution_receipt_file_sha256': plan.get(
                    'execution_receipt_file_sha256'),
                'selection_receipt_file_sha256': plan.get(
                    'selection_receipt_file_sha256'),
                'image_digest': plan['image_digest'],
                'execution': plan['execution_identity'], 'input': plan['input']},
            'argv': command, 'env': plan['env'], 'mounts': plan['mounts'],
            'started_at': started.isoformat(), 'finished_at': finished.isoformat(),
            'execution': {
                'exit_status': exit_status, 'timed_out': timed_out,
                'signal': signal},
            'completion': {
                'complete': complete,
                'expected_outputs': [str(path.relative_to(part))
                                     for path in expected],
                'memory_evidence_valid': memory_evidence['valid'],
                'process_rss_evidence_valid': process_rss_evidence['valid'],
                'phase_evidence_valid': phase_evidence['valid'],
                'online_compute_rtf_gate_passed': phase_gate_passed,
                'phase_primary_gate_passed': phase_gate_passed},
            'timing': timing,
            'phase_evidence': phase_evidence,
            'runtime': {
                'phase_contract_version': phase_runtime.get(
                    'phase_contract_version'),
                'phase_mode': phase_runtime.get('phase_mode'),
                'online_compute_rtf': phase_runtime.get('online_compute_rtf'),
                'paced_followability_passed': phase_runtime.get(
                    'paced_followability_passed'),
                'unpaced_throughput_gate_passed': phase_runtime.get(
                    'unpaced_throughput_gate_passed'),
                'wall_realtime_factor': wall_rtf,
                'wall_realtime_factor_role': 'diagnostic_only',
            },
            'docker_client_peak_rss_kb': timing.get('docker_client_peak_rss_kb'),
            PROCESS_RSS_PRIMARY_METRIC: process_rss_evidence.get(
                PROCESS_RSS_PRIMARY_METRIC),
            'process_tree_peak_rss_bytes': process_rss_evidence.get(
                PROCESS_RSS_PRIMARY_METRIC),
            'comparison_rss_metric': PROCESS_RSS_PRIMARY_METRIC,
            'process_rss_measurement_contract': {
                'measurement_version': PROCESS_RSS_MEASUREMENT_VERSION,
                'measurement_scope': 'container_pid_namespace_proc_status',
                'primary_metric': PROCESS_RSS_PRIMARY_METRIC,
                'primary_metric_definition': PROCESS_RSS_METRIC_DEFINITION,
                'sampler_script_sha256': sha256_file(
                    PROCESS_RSS_HELPER_FILE),
                'memory_helper_script_sha256': sha256_file(
                    MEMORY_HELPER_FILE),
            },
            'process_rss_evidence_file_sha256': sha256_file(
                part / PROCESS_RSS_EVIDENCE_NAME)
            if (part / PROCESS_RSS_EVIDENCE_NAME).is_file() else None,
            'container_memory_evidence_file_sha256': sha256_file(
                part / MEMORY_EVIDENCE_NAME)
            if (part / MEMORY_EVIDENCE_NAME).is_file() else None,
            'container_cgroup_peak_bytes': memory_evidence.get(
                'container_cgroup_peak_bytes'),
            'container_cgroup_total_peak_bytes': memory_evidence.get(
                'container_cgroup_total_peak_bytes'),
            'cgroup_events': memory_evidence.get('cgroup_events'),
            'memory_evidence': memory_evidence,
            'artifact_hashes': artifact_hashes(part),
            'output_tree_sha256': output_tree_hash(part),
            'gt_blind_proof': proof,
            'campaign_id': plan.get('campaign_id') or canonical_sha({
                'schedule': plan.get('schedule'),
                'system': plan.get('system'),
                'sequence': plan.get('sequence'),
                'repetition': plan.get('repetition'),
                'identity': plan.get('execution_identity'),
                'profile_canonical_sha256': plan.get(
                    'profile_canonical_sha256'),
                'selection_receipt_file_sha256': plan.get(
                    'selection_receipt_file_sha256'),
            }),
        }
        report = seal_receipt(report)
        write_json(part / 'attempt.json', report)
        # This is deliberately outside the self-hashed attempt JSON.  The
        # canonical receipt identity above is stable across formatting; the
        # external run index/completion record binds the exact pretty-file
        # bytes copied into a claim bundle.
        plan['_execution_receipt_file_sha256'] = sha256_file(
            part / 'attempt.json')
        os.replace(part, final)
        return report
    except OSError as error:
        try:
            failure_path = write_driver_failure(output_root, plan, part, started,
                                                error)
        except OSError as failure_error:
            raise ContractError(
                f'attempt {index} finalization failed: {error}; '
                f'failure artifact also failed: {failure_error}') from error
        raise ContractError(
            f'attempt {index} finalization failed; preserved {part}; '
            f'failure artifact: {failure_path}: {error}') from error


def root_marker(output_root: Path, identity: dict[str, Any]) -> Path:
    return output_root / '.m6a_gt_blind_root.json'


def check_or_create_marker(output_root: Path, identity: dict[str, Any], *, create: bool) -> None:
    marker = root_marker(output_root, identity)
    if marker.exists():
        observed = load_yaml(marker) if marker.suffix in ('.yaml', '.yml') else json.loads(
            marker.read_text(encoding='utf-8'))
        if observed != identity:
            raise ContractError('results root identity marker mismatch')
    elif any(output_root.iterdir()):
        raise ContractError('results root contains unowned files')
    elif create:
        write_json(marker, identity)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-root', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--receipt', type=Path, default=DEFAULT_RECEIPT)
    parser.add_argument('--selection', type=Path, default=DEFAULT_SELECTION)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--preflight', action='store_true')
    parser.add_argument('--execute', action='store_true',
                        help='run replays; never combine with --dry-run')
    parser.add_argument('--phase-contract', choices=PHASE_CONTRACT_CHOICES,
                        default='v1',
                        help='phase evidence contract (v1 remains the default)')
    parser.add_argument('--plan-output', type=Path)
    parser.add_argument('--timeout-seconds', type=float, default=7200.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.dry_run and (args.preflight or args.execute):
        raise ContractError('--dry-run cannot be combined with --preflight/--execute')
    if args.execute and args.dry_run:
        raise ContractError('--execute cannot be combined with --dry-run')
    args.input_root = args.input_root.resolve(strict=True)
    args.output_root = args.output_root.resolve(strict=False)
    args.profile = args.profile.resolve(strict=True)
    args.receipt = args.receipt.resolve(strict=True)
    args.selection = args.selection.resolve(strict=True)
    assert_roots_are_disjoint(args.input_root, args.output_root)
    profile_doc = load_yaml(args.profile)
    receipt = load_yaml(args.receipt)
    selection = load_yaml(args.selection)
    plan = build_plan(profile_doc, receipt, selection, args.input_root,
                      args.profile, args.receipt, args.selection,
                      inspect_images=args.preflight or args.execute,
                      phase_contract=args.phase_contract)
    plan['paths']['output_root'] = str(args.output_root)
    if args.plan_output:
        args.plan_output.parent.mkdir(parents=True, exist_ok=True)
        write_json(args.plan_output, plan)
    else:
        print(json.dumps(plan, indent=2, sort_keys=True))
    if not args.execute:
        return 0
    args.output_root.mkdir(parents=True, exist_ok=False)
    check_or_create_marker(args.output_root, plan['identity'], create=True)
    reports = []
    execution_receipt_files = []
    for item in plan['attempts']:
        item.update({key: value for key, value in plan['identity'].items()
                     if key.endswith('sha256')})
        item['campaign_id'] = plan['identity']['campaign_id']
        print(f"M6a attempt {item['schedule_index']}/27: "
              f"{item['system']} {item['slot']} r{item['repetition']}", flush=True)
        report = run_attempt(item, args.output_root, args.timeout_seconds)
        reports.append(report)
        execution_receipt_files.append({
            'schedule_index': item['schedule_index'],
            'system': item['system'], 'sequence': item['sequence'],
            'repetition': item['repetition'],
            'campaign_id': plan['identity']['campaign_id'],
            'execution_receipt_sha256': report['execution_receipt_sha256'],
            'execution_receipt_file_sha256': item.get(
                '_execution_receipt_file_sha256'),
        })
    completion = {
        'schema_version': 1, 'kind': 'm6a_gt_blind_completion',
        'status': 'PASS' if all(report['completion']['complete'] for report in reports)
        else 'INCOMPLETE', 'identity': plan['identity'], 'attempts': reports,
        'execution_receipt_files': execution_receipt_files,
        'ground_truth_content_opened': False, 'scorer_invoked': False,
    }
    write_json(args.output_root / 'completion_manifest.json', completion)
    return 0 if completion['status'] == 'PASS' else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ContractError, OSError, KeyError, TypeError, json.JSONDecodeError,
            yaml.YAMLError, subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
