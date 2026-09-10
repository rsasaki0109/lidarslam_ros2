#!/usr/bin/env python3
"""Fail-closed v10 formal launcher and lifecycle gate.

The launcher binds the v10 profile, source patch, image, fixed bag metadata,
immutable gate receipts, and runtime source bytes.  Its production path uses
read-only identity probes, one quiescence subprocess, one fixed Docker
``Popen``, diagnostics-first watchdog supervision, host binding/composition,
and stopped-only cleanup.  Tests retain explicit injection seams; no shell,
retry, ground-truth, or scorer path is accepted.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Any, Callable, Mapping, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]

CONTRACT_ID = 'm6a10-v2c-fast-livo2-terminal-support-context-v10'
LAUNCHER_CONTRACT_ID = 'm6a10-v2c-fast-livo2-v10-formal-lifecycle-v1'
SCHEMA_VERSION = 1
QUIESCENCE_CONTRACT_VERSION = 'm6a10-quiescence-v1'

PROFILE_PATH = Path('configs/slam_benchmark_profiles/fast_livo2_m6a10_v10_formal.yaml')
PROFILE_SHA256 = (
    'f706f41b0a985347ff98e4953532590c7a9c8c2d75db355e53cfb558f95bc459')
EXPECTED_PROFILE_SHA256 = PROFILE_SHA256
V10_PROFILE_SHA256 = PROFILE_SHA256
V10_PATCH_PATH = Path(
    'docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch')
V10_PATCH_SHA256 = (
    '155bae4e37eac7d6220baedd861b9110b6139ee2b384d5b90db78ac24a6733c6')
EXPECTED_PATCH_SHA256 = V10_PATCH_SHA256
V10_DELTA_PATCH_SHA256 = V10_PATCH_SHA256

WRAPPER_PATH = Path('scripts/fast_livo2_m6a10_v10_container_run.sh')
WRAPPER_SHA256 = (
    '678b46402e441fcdc306140f104a878c564ec3e0e644957501bae431e041535b')
FEEDER_PATH = Path('scripts/fast_livo2_m6a10_feeder.py')
FEEDER_CONTAINER_PATH = '/runner/scripts/fast_livo2_m6a10_feeder.py'
FEEDER_SHA256 = (
    '1ea1c9bb9c625795c51c4b8c4b6aa81364370da17f8cac562f46d00af9691831')
QUIESCENCE_SCRIPT = Path('scripts/check_m6a10_quiescence.py')
QUIESCENCE_SCRIPT_SHA256 = (
    'acf8b2e65e7744dd9cfd4127ac5ec12fb381c5a7c1430910b3e769ea259fe85d')
MAX_PREFLIGHT_TO_RUN_GAP_SECONDS = 2.0
MAX_PREFLIGHT_TO_RUN_GAP_NS = int(MAX_PREFLIGHT_TO_RUN_GAP_SECONDS * 1e9)
MAX_CONTAINER_RUNTIME_SECONDS = 1200.0
CONTAINER_STATS_INTERVAL_SECONDS = 5.0
CONTAINER_STOP_GRACE_SECONDS = 30.0
CONTAINER_STOP_WAIT_SECONDS = 45.0
IMAGE_TAG = 'm6a10-v2c-v10-callback-v3-20260823t094100z-fast-livo2-benchmark:ros1-pinned'
IMAGE_ID = 'sha256:3e087acc5ef116f03357a73927c18b2059068b093ae1cce6cb41c1baf1fbf759'
IMAGE_DIGEST = IMAGE_ID
IMAGE_REF = f'{IMAGE_TAG}@{IMAGE_ID}'

BAG_PATH = Path(
    '/media/sasaki/aiueo1/datasets/ntu_viral_release/'
    'tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag')
BAG_BYTES = 11290464091
BAG_SHA256 = '5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310'
EXPECTED_MESSAGES = 236687
EXPECTED_TOPIC_COUNTS = {'lidar': 5793, 'imu': 225102, 'image': 5792}
REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS = 1623491515.148352

# Public aliases make the immutable contract easy to audit without opening
# the bag or importing a replay implementation.
EXPECTED_BAG_BYTES = BAG_BYTES
EXPECTED_BAG_SHA256 = BAG_SHA256
EXPECTED_END_TIMESTAMP_SECONDS = REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS
EXPECTED_END_TIMESTAMP = REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS
BAG_EXPECTED_TOPIC_COUNTS = EXPECTED_TOPIC_COUNTS

LIDAR_TOPIC = '/os1_cloud_node1/points'
IMU_TOPIC = '/imu/imu'
IMAGE_TOPIC = '/left/image_raw'

PHASE_CONTRACT_VERSION = 'm6a10-online-compute-v3-terminal-support-context'
PHASE_MODE = 'unpaced_ack'
EXPECTED_END_GAP_SECONDS = '0.25'
EXPECTED_MIN_TERMINAL_POLL_WALL_SECONDS = '0.05'
EXPECTED_CALLBACK_LATENCY_SECONDS = '0.25'
EXPECTED_BACKLOG_MESSAGES = '1'
FEEDER_TIMEOUT_SECONDS = '60'
TERMINAL_DRAIN_TIMEOUT_SECONDS = '120'
STARTUP_TIMEOUT_SECONDS = '60'
TRAJECTORY_TIMEOUT_SECONDS = '120'
RPC_TIMEOUT_SECONDS = '10'
SENSOR_DURATION_SECONDS = 579.278127298
EXPECTED_SENSOR_DURATION_SECONDS = SENSOR_DURATION_SECONDS
TIMING_SCHEMA_VERSION = 1
TIMING_CONTRACT_VERSION = 'm6a10-online-compute-v3-timing-v1'
TIMING_FILENAME = 'online_compute_timing.json'

EVIDENCE_RECEIPT_SHA256 = {
    'build_identity': 'a92e1d283f6787229c21be96b4f9f87564e13cf6ad8edda019ef096868139b96',
    'synthetic_gates': 'e41a44df9db8ed807aa14c2cc3540ba58f1ba0956e88be3b4151a9c3db4415f7',
    'no_input_handshake': 'a8b1b39fd1ad6807d328063b98c70037d3bca1d1c6bec8b68347c807ef7517a7',
    'binder_compositor': '8d65a05b581f9ddfbefbc7f88f29e0b002828911842a067dc3e0d19c7c778347',
}
EVIDENCE_RECEIPT_SHAS = EVIDENCE_RECEIPT_SHA256

EVIDENCE_RECEIPT_PATHS = {
    'build_identity': Path(
        '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/'
        'fast_livo2_v2c_v10_callback_v3_rebuild2_20260823T094100Z_agentv10callback/'
        'build/build_identity.receipt.json'),
    'synthetic_gates': Path(
        '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/'
        'fast_livo2_v2c_v10_callback_v3_rebuild2_20260823T094100Z_agentv10callback/'
        'synthetic/synthetic_cpp_gates.receipt.json'),
    'no_input_handshake': Path(
        '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/'
        'fast_livo2_v2c_v10_callback_v3_rebuild2_20260823T094100Z_agentv10callback/'
        'handshake/dual_service_handshake.receipt.json'),
    'binder_compositor': Path(
        '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/'
        'fast_livo2_v2c_v10_callback_v3_rebuild2_20260823T094100Z_agentv10callback/'
        'host_gates/binder_compositor.receipt.json'),
}

AUTHORIZED_ATTEMPT_ROOT = Path(
    '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/'
    'fast_livo2_v2c_v10_formal_replay3_20260823T112240Z_agentv10formal')
AUTHORIZATION_PATH = Path(
    '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/'
    'fast_livo2_v2c_v10_formal_authorization3_20260823T112240Z/'
    'formal_replay_authorization.receipt.json')
AUTHORIZATION_SHA256 = (
    '40f585ef747f5915cb20629001eeef176bd5f3d7b6a95feb19b93506b4da596f')
AUTHORIZATION_CONTRACT_ID = 'm6a10-v2c-v10-formal-replay-authorization-v1'
AUTHORIZATION_RECEIPT_KIND = 'm6a10_v10_formal_replay_authorization'
BINDER_PATH = Path('scripts/bind_fast_livo2_v10_consumer_evidence.py')
BINDER_SHA256 = (
    'f8b81fdb17b7ee516cf1f7b0c24b79bfa6a0f2a3cf4f5dd199216534325f40e6')
COMPOSITOR_PATH = Path('scripts/compose_fast_livo2_terminal_evidence.py')
COMPOSITOR_SHA256 = (
    'b706084392d858fb4c61f341e088c994996659a757a7ef209e9104851998fdfe')


class LaunchError(RuntimeError):
    """A machine-readable fail-closed launcher error."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


@dataclass(frozen=True)
class FormalConfig:
    """Independent v10 configuration; no v8/v9 defaults are inherited."""

    root: Path
    repo_root: Path = ROOT
    bag_path: Path = BAG_PATH
    image_tag: str = IMAGE_TAG
    image_id: str = IMAGE_ID
    profile_path: Path = PROFILE_PATH
    patch_path: Path = V10_PATCH_PATH
    quiescence_script: Path = QUIESCENCE_SCRIPT
    watchdog_seconds: float = MAX_CONTAINER_RUNTIME_SECONDS
    stats_interval_seconds: float = CONTAINER_STATS_INTERVAL_SECONDS


# Compatibility spelling used by the older launcher tests, while retaining an
# independent implementation and configuration class.
LaunchConfig = FormalConfig


def _utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _json_bytes(value: Mapping[str, Any]) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True) + '\n').encode('utf-8')


def _atomic_bytes(path: Path, payload: bytes, *, kind: str = 'RECEIPT_OVERWRITE') -> str:
    """Create immutable bytes exactly once, with fsync and no replacement."""
    if path.exists() or path.is_symlink():
        raise LaunchError(kind, f'refusing to overwrite {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    except FileExistsError as error:
        raise LaunchError(kind, f'refusing to overwrite {path}') from error
    with os.fdopen(fd, 'wb') as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    return hashlib.sha256(payload).hexdigest()


def atomic_json(path: Path, value: Mapping[str, Any]) -> str:
    """Create one immutable JSON receipt and refuse every overwrite."""
    return _atomic_bytes(path, _json_bytes(value))


def atomic_sha256_sidecar(path: Path, digest: str) -> Path:
    """Seal the conventional SHA-256 sidecar for one immutable receipt."""
    sidecar = path.with_name(path.name + '.sha256')
    payload = f'{digest}  {path.name}\n'.encode('ascii')
    _atomic_bytes(sidecar, payload)
    return sidecar


def _absolute_path(path: Path, label: str) -> Path:
    if not path.is_absolute():
        raise LaunchError('PATH_NOT_ABSOLUTE', f'{label} must be absolute: {path}')
    if path.is_symlink():
        raise LaunchError('PATH_SYMLINK', f'{label} must not be a symlink: {path}')
    if path == Path('/'):
        raise LaunchError('PATH_TOO_BROAD', f'{label} may not be /')
    return path


def _repo_path(config: FormalConfig, path: Path) -> Path:
    return path if path.is_absolute() else config.repo_root / path


def _default_image_probe(config: FormalConfig) -> Mapping[str, Any]:
    """Inspect the pinned local image without pulling, building, or running."""
    command = (
        'docker', 'image', 'inspect', '--format', '{{json .}}', config.image_id)
    try:
        completed = subprocess.run(
            command, check=False, capture_output=True, text=True)
    except (OSError, subprocess.SubprocessError) as error:
        raise LaunchError('IMAGE_INSPECT_FAILURE', str(error)) from error
    if completed.returncode != 0:
        raise LaunchError(
            'IMAGE_INSPECT_FAILURE', completed.stderr.strip() or 'docker image inspect failed')
    try:
        document = json.loads(completed.stdout.strip())
    except (UnicodeError, json.JSONDecodeError) as error:
        raise LaunchError('IMAGE_INSPECT_INVALID', 'docker image inspect JSON is invalid') from error
    if isinstance(document, list):
        if len(document) != 1:
            raise LaunchError('IMAGE_INSPECT_INVALID', 'docker image inspect returned wrong count')
        document = document[0]
    if not isinstance(document, Mapping):
        raise LaunchError('IMAGE_INSPECT_INVALID', 'docker image inspect result is not an object')
    labels = document.get('Config', {}).get('Labels', {}) if isinstance(
        document.get('Config', {}), Mapping) else {}
    if not isinstance(labels, Mapping):
        labels = {}
    repo_tags = document.get('RepoTags', [])
    if not isinstance(repo_tags, list):
        repo_tags = []
    if IMAGE_TAG not in repo_tags:
        raise LaunchError('IMAGE_IDENTITY_MISMATCH', 'local image tag is not bound to the pinned ID')
    return {
        'tag': IMAGE_TAG,
        'id': document.get('Id'),
        'repo_tags': list(repo_tags),
        'labels': dict(labels),
        'docker_inspected': True,
    }


def _default_bag_probe(config: FormalConfig) -> Mapping[str, Any]:
    """Stat and stream-hash exactly the pinned bag, without parsing messages."""
    bag = _absolute_path(config.bag_path, 'bag path')
    if bag != BAG_PATH:
        raise LaunchError('BAG_IDENTITY_MISMATCH', 'bag path differs from pinned v10 input')
    try:
        stat = bag.lstat()
    except OSError as error:
        raise LaunchError('BAG_PROBE_FAILURE', f'cannot stat bag: {bag}') from error
    if bag.is_symlink() or not bag.is_file():
        raise LaunchError('BAG_PROBE_FAILURE', 'bag is not a regular non-symlink file')
    if stat.st_size != BAG_BYTES:
        raise LaunchError('BAG_IDENTITY_MISMATCH', 'bag byte count differs from pin')
    observed_sha = file_sha256(bag)
    if observed_sha != BAG_SHA256:
        raise LaunchError('BAG_IDENTITY_MISMATCH', 'bag SHA-256 differs from pin')
    return {
        'path': str(bag),
        'bytes': stat.st_size,
        'sha256': observed_sha,
        'expected_messages': EXPECTED_MESSAGES,
        'expected_topic_counts': dict(EXPECTED_TOPIC_COUNTS),
        'required_evaluation_end_timestamp_seconds':
            REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS,
        'opened': True,
        'streaming_sha256': True,
    }


def _reject_overlap(first: Path, second: Path) -> None:
    first_resolved = first.resolve(strict=False)
    second_resolved = second.resolve(strict=False)
    if (first_resolved == second_resolved or
            first_resolved in second_resolved.parents or
            second_resolved in first_resolved.parents):
        raise LaunchError('ROOT_OVERLAP', f'paths overlap: {first} and {second}')


def _validated_text(value: str, label: str) -> str:
    if not isinstance(value, str) or not value or any(
            char in value for char in ('\x00', '\n', '\r')):
        raise LaunchError('ARGUMENT_INJECTION', f'invalid {label}')
    return value


def _receipt_value(receipt: Mapping[str, Any], path: str, key: str) -> Any:
    """Read one required receipt field and fail closed when it is absent."""
    value: Any = receipt
    for component in path.split('.'):
        if not isinstance(value, Mapping) or component not in value:
            raise LaunchError(
                'EVIDENCE_RECEIPT_INVALID',
                f'{key}: missing receipt field {path}')
        value = value[component]
    return value


def verify_evidence_receipts() -> dict[str, Any]:
    """Verify the four immutable gates before any bag probe can run.

    Receipt verification is deliberately host-side and read-only.  It checks
    the actual receipt bytes, final image identity, profile/patch bindings,
    and each receipt's safety contract.  No input bag, GT, or scorer path is
    opened here.
    """
    safety_contracts: dict[str, tuple[tuple[str, str, Any], ...]] = {
        'build_identity': (
            ('build_contract', 'formal_replay_started', False),
            ('build_contract', 'input_mount_performed', False),
            ('build_contract', 'ground_truth_mount_forbidden', True),
            ('build_contract', 'scorer_mount_forbidden', True),
            ('build_contract', 'network', 'none'),
        ),
        'synthetic_gates': (
            ('command_contract', 'bag_opened', False),
            ('command_contract', 'formal_replay_started', False),
            ('command_contract', 'ground_truth_content_opened', False),
            ('command_contract', 'input_mount_performed', False),
            ('command_contract', 'scorer_invoked', False),
            ('command_contract', 'network', 'none'),
            ('command_contract', 'rootfs', 'read_only'),
        ),
        'no_input_handshake': (
            ('container_security', 'formal_replay_started', False),
            ('container_security', 'ground_truth_mount_performed', False),
            ('container_security', 'input_mount_performed', False),
            ('container_security', 'network', 'none'),
            ('container_security', 'rootfs', 'read_only'),
            ('safety', 'bag_opened', False),
            ('safety', 'ground_truth_content_opened', False),
            ('safety', 'map_saved', False),
            ('safety', 'scorer_invoked', False),
        ),
        'binder_compositor': (
            ('safety', 'bag_opened', False),
            ('safety', 'formal_replay_started', False),
            ('safety', 'ground_truth_content_opened', False),
            ('safety', 'scorer_invoked', False),
        ),
    }
    verified: dict[str, Any] = {}
    for key, expected_sha in EVIDENCE_RECEIPT_SHA256.items():
        if key not in EVIDENCE_RECEIPT_PATHS or key not in safety_contracts:
            raise LaunchError('EVIDENCE_RECEIPT_INVALID', f'unexpected receipt key: {key}')
        path = EVIDENCE_RECEIPT_PATHS[key]
        _absolute_path(path, f'{key} receipt')
        if not path.is_file() or path.is_symlink():
            raise LaunchError(
                'EVIDENCE_RECEIPT_MISSING', f'{key} receipt is not a regular file: {path}')
        observed_sha = file_sha256(path)
        if observed_sha != expected_sha:
            raise LaunchError(
                'EVIDENCE_RECEIPT_SHA_MISMATCH',
                f'{key} receipt SHA differs from immutable pin')
        try:
            receipt = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise LaunchError(
                'EVIDENCE_RECEIPT_INVALID', f'{key} receipt is not valid JSON') from error
        if not isinstance(receipt, Mapping):
            raise LaunchError('EVIDENCE_RECEIPT_INVALID', f'{key} receipt is not an object')
        if _receipt_value(receipt, 'status', key) != 'PASS':
            raise LaunchError('EVIDENCE_RECEIPT_INVALID', f'{key} receipt is not PASS')
        image = _receipt_value(receipt, 'image', key)
        if _receipt_value(image, 'tag', key) != IMAGE_TAG:
            raise LaunchError('EVIDENCE_RECEIPT_IMAGE_MISMATCH', f'{key} image tag mismatch')
        if _receipt_value(image, 'id', key) != IMAGE_ID:
            raise LaunchError('EVIDENCE_RECEIPT_IMAGE_MISMATCH', f'{key} image ID mismatch')
        source_hashes = receipt.get('source_hashes', {})
        if not isinstance(source_hashes, Mapping):
            raise LaunchError('EVIDENCE_RECEIPT_INVALID', f'{key} source hashes are invalid')
        profile_sha = receipt.get('profile_sha256', source_hashes.get('profile_sha256'))
        patch_sha = receipt.get('v10_patch_sha256', source_hashes.get('v10_patch_sha256'))
        if profile_sha != PROFILE_SHA256:
            raise LaunchError('EVIDENCE_RECEIPT_PROFILE_MISMATCH', f'{key} profile SHA mismatch')
        if patch_sha != V10_PATCH_SHA256:
            raise LaunchError('EVIDENCE_RECEIPT_PATCH_MISMATCH', f'{key} patch SHA mismatch')
        for section, field, expected in safety_contracts[key]:
            observed = _receipt_value(receipt, f'{section}.{field}', key)
            matches = observed is expected if isinstance(expected, bool) else observed == expected
            if not matches:
                raise LaunchError(
                    'EVIDENCE_RECEIPT_SAFETY_INVALID',
                    f'{key} safety field {section}.{field} is not {expected!r}')
        verified[key] = {
            'path': str(path),
            'sha256': observed_sha,
            'status': receipt['status'],
            'image': {'tag': image['tag'], 'id': image['id']},
            'profile_sha256': profile_sha,
            'v10_patch_sha256': patch_sha,
            'safety_verified': True,
        }
    if set(verified) != set(EVIDENCE_RECEIPT_PATHS):
        raise LaunchError('EVIDENCE_RECEIPT_INVALID', 'receipt set is incomplete')
    return verified


def _authorization_value(
        receipt: Mapping[str, Any], path: str, expected: Any) -> Any:
    value: Any = receipt
    for component in path.split('.'):
        if not isinstance(value, Mapping) or component not in value:
            raise LaunchError(
                'AUTHORIZATION_INVALID', f'missing authorization field {path}')
        value = value[component]
    matches = value is expected if isinstance(expected, bool) else value == expected
    if not matches:
        raise LaunchError(
            'AUTHORIZATION_FIELD_MISMATCH',
            f'authorization field {path} differs from the pinned value')
    return value


def verify_formal_authorization(config: FormalConfig) -> dict[str, Any]:
    """Verify the additive authorization for exactly one formal attempt.

    The authorization is an immutable external receipt.  It does not mutate
    the profile's formal-replay-forbidden flag and intentionally excludes the
    launcher hash to avoid a self-reference cycle.
    """
    root = _absolute_path(config.root, 'attempt root')
    if root != AUTHORIZED_ATTEMPT_ROOT:
        raise LaunchError(
            'AUTHORIZATION_ROOT_MISMATCH',
            'formal authorization is bound to a different attempt root')
    if config.bag_path != BAG_PATH:
        raise LaunchError(
            'AUTHORIZATION_INPUT_MISMATCH',
            'formal authorization is bound to a different input path')
    _absolute_path(AUTHORIZATION_PATH, 'formal authorization receipt')
    sidecar = AUTHORIZATION_PATH.with_name(AUTHORIZATION_PATH.name + '.sha256')
    if AUTHORIZATION_PATH.is_symlink() or not AUTHORIZATION_PATH.is_file():
        raise LaunchError('AUTHORIZATION_MISSING', 'formal authorization receipt is not regular')
    if sidecar.is_symlink() or not sidecar.is_file():
        raise LaunchError('AUTHORIZATION_SIDECAR_MISSING', 'formal authorization sidecar is missing')
    observed_sha = file_sha256(AUTHORIZATION_PATH)
    if observed_sha != AUTHORIZATION_SHA256:
        raise LaunchError('AUTHORIZATION_SHA_MISMATCH', 'formal authorization SHA differs from pin')
    try:
        sidecar_text = sidecar.read_text(encoding='ascii')
    except (OSError, UnicodeError) as error:
        raise LaunchError('AUTHORIZATION_SIDECAR_INVALID', 'authorization sidecar is unreadable') from error
    expected_sidecar = f'{AUTHORIZATION_SHA256}  {AUTHORIZATION_PATH.name}\n'
    if sidecar_text != expected_sidecar:
        raise LaunchError('AUTHORIZATION_SIDECAR_INVALID', 'authorization sidecar content differs')
    try:
        receipt = json.loads(AUTHORIZATION_PATH.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise LaunchError('AUTHORIZATION_INVALID', 'formal authorization JSON is invalid') from error
    if not isinstance(receipt, Mapping):
        raise LaunchError('AUTHORIZATION_INVALID', 'formal authorization is not an object')

    for path, expected in (
            ('schema_version', SCHEMA_VERSION),
            ('contract_id', AUTHORIZATION_CONTRACT_ID),
            ('receipt_kind', AUTHORIZATION_RECEIPT_KIND),
            ('status', 'AUTHORIZED'),
            ('attempt_index', 3),
            ('retry_of_same_root', False),
            ('authorization.scope', 'exact_attempt_root_only'),
            ('authorization.attempt_root', str(AUTHORIZED_ATTEMPT_ROOT)),
            ('authorization.profile_immutable', True),
            ('authorization.profile_formal_replay_forbidden', True),
            ('authorization.formal_replay_forbidden_superseded_for_this_attempt_only', True),
            ('authorization.additive_authorization_only', True),
            ('authorization.readme_authorized', False),
            ('authorization.ground_truth_scoring_authorized', False),
            ('input.path', str(BAG_PATH)),
            ('input.bytes', BAG_BYTES),
            ('input.sha256', BAG_SHA256),
            ('input.expected_messages', EXPECTED_MESSAGES),
            ('input.expected_topic_counts', EXPECTED_TOPIC_COUNTS),
            ('input.required_evaluation_end_timestamp_seconds',
             REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS),
            ('input.sensor_duration_seconds', SENSOR_DURATION_SECONDS),
            ('profile.path', str(PROFILE_PATH)),
            ('profile.sha256', PROFILE_SHA256),
            ('profile.formal_replay_forbidden', True),
            ('patch.path', str(V10_PATCH_PATH)),
            ('patch.sha256', V10_PATCH_SHA256),
            ('image.tag', IMAGE_TAG),
            ('image.id', IMAGE_ID),
            ('execution.watchdog_seconds', MAX_CONTAINER_RUNTIME_SECONDS),
            ('execution.runner_start_count', 1),
            ('execution.one_start', True),
            ('execution.replay_count', 1),
            ('execution.retry_count', 0),
            ('execution.no_retry', True),
            ('execution.manual_stop_allowed', False),
            ('execution.watchdog_safety_stop_allowed', True),
            ('execution.ground_truth_content_opened', False),
            ('execution.scorer_invoked', False),
            ('execution.map_saved', False),
            ('execution.network', 'none'),
            ('execution.rootfs', 'read_only'),
            ('safety.ground_truth_mount', False),
            ('safety.scorer_mount', False),
            ('safety.ground_truth_content_opened', False),
            ('safety.scorer_invoked', False),
            ('safety.map_saved', False),
            ('safety.manual_stop', False),
            ('safety.retry', False),
            ('prior_attempt.attempt_root',
             '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/'
             'fast_livo2_v2c_v10_formal_replay2_20260823T111124Z_agentv10formal'),
            ('prior_attempt.closure_path',
             '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/'
             'fast_livo2_v2c_v10_formal_replay2_20260823T111124Z_agentv10formal/'
             'closure_receipt.json'),
            ('prior_attempt.closure_sha256',
             'ec9459f4f76bf46873c1bd4e8f90299310e0449b4d82435dd0af3110cc2f5446'),
            ('prior_attempt.cause', 'feeder_missing'),
            ('prior_attempt.exit_code', 1),
            ('prior_attempt.container_status',
             'container_created_then_natural_exit_cleanup_removed'),
            ('prior_attempt.retry_of_same_root', False),
            ('authorized_delta.old_launcher_sha256',
             'eb9b397985cc5239c60721974ecab6d9365b32db68d254f0d44266d822d88287'),
            ('authorized_delta.reviewed_launcher_sha256',
             '9309d43202749eb52d9869cdd9ac660495221e4397e5263f9e523c6aa3ed0b52'),
            ('authorized_delta.reason', 'add_pinned_readonly_feeder_bind'),
            ('authorized_delta.output_mount', 'type=bind,dst=/out,readonly=false'),
            ('authorized_delta.input_mount', 'type=bind,dst=/input/ntu_viral.bag,readonly'),
            ('authorized_delta.wrapper_mount', 'type=bind,dst=/runner/v10_runtime.sh,readonly'),
            ('authorized_delta.feeder_mount',
             'type=bind,dst=/runner/scripts/fast_livo2_m6a10_feeder.py,readonly')):
        _authorization_value(receipt, path, expected)

    gate_receipts = receipt.get('gate_receipts')
    if not isinstance(gate_receipts, Mapping) or \
            set(gate_receipts) != set(EVIDENCE_RECEIPT_PATHS):
        raise LaunchError('AUTHORIZATION_GATE_SET_INVALID', 'authorization gate receipt set differs')
    for key, expected_sha in EVIDENCE_RECEIPT_SHA256.items():
        entry = gate_receipts.get(key)
        if not isinstance(entry, Mapping):
            raise LaunchError('AUTHORIZATION_GATE_INVALID', f'{key} authorization gate is missing')
        if entry.get('path') != str(EVIDENCE_RECEIPT_PATHS[key]) or \
                entry.get('sha256') != expected_sha or entry.get('status') != 'PASS':
            raise LaunchError('AUTHORIZATION_GATE_INVALID', f'{key} authorization gate differs')

    source_contract = {
        'wrapper': (str(WRAPPER_PATH), WRAPPER_SHA256),
        'feeder': (str(FEEDER_PATH), FEEDER_SHA256),
        'binder': (str(BINDER_PATH), BINDER_SHA256),
        'compositor': (str(COMPOSITOR_PATH), COMPOSITOR_SHA256),
        'quiescence': (str(QUIESCENCE_SCRIPT), QUIESCENCE_SCRIPT_SHA256),
    }
    sources = receipt.get('sources')
    if not isinstance(sources, Mapping) or set(sources) != set(source_contract):
        raise LaunchError('AUTHORIZATION_SOURCE_SET_INVALID', 'authorization source set differs')
    for name, (expected_path, expected_sha) in source_contract.items():
        entry = sources.get(name)
        if not isinstance(entry, Mapping) or entry.get('path') != expected_path or \
                entry.get('sha256') != expected_sha:
            raise LaunchError('AUTHORIZATION_SOURCE_INVALID', f'{name} authorization source differs')
    return {
        'path': str(AUTHORIZATION_PATH),
        'sha256': observed_sha,
        'status': receipt['status'],
        'contract_id': receipt['contract_id'],
        'attempt_root': str(AUTHORIZED_ATTEMPT_ROOT),
        'attempt_index': 3,
        'retry_of_same_root': False,
        'prior_attempt': dict(receipt['prior_attempt']),
        'authorized_delta': dict(receipt['authorized_delta']),
        'replay_count': 1,
        'runner_start_count': 1,
        'retry_count': 0,
        'manual_stop_allowed': False,
        'watchdog_seconds': MAX_CONTAINER_RUNTIME_SECONDS,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'map_saved': False,
    }


def _bag_contract(metadata: Optional[Mapping[str, Any]] = None) -> dict[str, Any]:
    """Return declared bag identity; this function never opens the bag."""
    declared = {
        'path': str(BAG_PATH),
        'bytes': BAG_BYTES,
        'sha256': BAG_SHA256,
        'expected_messages': EXPECTED_MESSAGES,
        'expected_topic_counts': dict(EXPECTED_TOPIC_COUNTS),
        'required_evaluation_end_timestamp_seconds':
            REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS,
        'opened': False,
    }
    if metadata is None:
        return declared
    observed = dict(metadata)
    checks = {
        'path': str(BAG_PATH),
        'bytes': BAG_BYTES,
        'sha256': BAG_SHA256,
        'expected_messages': EXPECTED_MESSAGES,
        'expected_topic_counts': EXPECTED_TOPIC_COUNTS,
        'required_evaluation_end_timestamp_seconds':
            REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS,
    }
    for key, expected in checks.items():
        if observed.get(key) != expected:
            raise LaunchError('BAG_IDENTITY_MISMATCH', f'bag metadata mismatch: {key}')
    declared['probe'] = observed
    return declared


def validate_preflight_identity(
        config: FormalConfig,
        *,
        image_probe: Optional[Callable[[FormalConfig], Mapping[str, Any]]] = None,
        bag_probe: Optional[Callable[[FormalConfig], Mapping[str, Any]]] = None,
        authorization_validator: Optional[Callable[[FormalConfig], Mapping[str, Any]]] = None,
) -> dict[str, Any]:
    """Verify source/image declarations; probes remain injectable.

    ``image_probe`` and ``bag_probe`` are dependency-injection seams.  The
    default path intentionally remains read-only and declaration-only; the
    production validator below supplies the inspect/stat/hash probes.
    """
    repo = _absolute_path(config.repo_root, 'repository root')
    if not repo.is_dir():
        raise LaunchError('REPOSITORY_MISSING', f'repository root is missing: {repo}')
    if config.image_tag != IMAGE_TAG or config.image_id != IMAGE_ID:
        raise LaunchError('IMAGE_IDENTITY_MISMATCH', 'v10 image tag or ID is not pinned')

    profile = _repo_path(config, config.profile_path)
    patch = _repo_path(config, config.patch_path)
    if not profile.is_file() or profile.is_symlink():
        raise LaunchError('PROFILE_IDENTITY_MISSING', f'profile is invalid: {profile}')
    if not patch.is_file() or patch.is_symlink():
        raise LaunchError('PATCH_IDENTITY_MISSING', f'patch is invalid: {patch}')
    profile_sha = file_sha256(profile)
    patch_sha = file_sha256(patch)
    if profile_sha != PROFILE_SHA256:
        raise LaunchError('PROFILE_IDENTITY_MISMATCH', 'v10 profile SHA differs from pin')
    if patch_sha != V10_PATCH_SHA256:
        raise LaunchError('PATCH_IDENTITY_MISMATCH', 'v10 patch SHA differs from pin')

    wrapper = _repo_path(config, WRAPPER_PATH)
    feeder = _repo_path(config, FEEDER_PATH)
    quiescence = _repo_path(config, config.quiescence_script)
    for source, label, expected in (
            (wrapper, 'wrapper', WRAPPER_SHA256),
            (feeder, 'feeder', FEEDER_SHA256),
            (quiescence, 'quiescence script', QUIESCENCE_SCRIPT_SHA256)):
        if not source.is_file() or source.is_symlink():
            raise LaunchError('SOURCE_IDENTITY_MISSING', f'{label} is invalid: {source}')
        if file_sha256(source) != expected:
            raise LaunchError('SOURCE_IDENTITY_MISMATCH', f'{label} SHA differs from pin')

    # Verify all immutable gate receipts before accepting any bag metadata
    # probe.  The receipt verifier is strictly host-side and never opens the
    # declared input path.
    evidence_receipts = verify_evidence_receipts()

    authorization = (authorization_validator or verify_formal_authorization)(config)

    if image_probe is None:
        image = {'tag': IMAGE_TAG, 'id': IMAGE_ID, 'docker_inspected': False}
    else:
        image = dict(image_probe(config))
        if image.get('id') != IMAGE_ID:
            raise LaunchError('IMAGE_IDENTITY_MISMATCH', 'probed image ID differs from pin')
        if image.get('tag', IMAGE_TAG) != IMAGE_TAG:
            raise LaunchError('IMAGE_IDENTITY_MISMATCH', 'probed image tag differs from pin')
    bag = _bag_contract(bag_probe(config) if bag_probe is not None else None)
    return {
        'status': 'PASS',
        'contract_id': CONTRACT_ID,
        'profile': {'path': str(profile), 'sha256': profile_sha},
        'patch': {'path': str(patch), 'sha256': patch_sha},
        'sources': {
            'launcher': {'path': str(Path(__file__).resolve()),
                         'sha256': file_sha256(Path(__file__).resolve())},
            'wrapper': {'path': str(wrapper), 'sha256': WRAPPER_SHA256},
            'feeder': {'path': str(feeder), 'sha256': FEEDER_SHA256},
            'quiescence': {'path': str(quiescence), 'sha256': QUIESCENCE_SCRIPT_SHA256},
        },
        'image': image,
        'bag': bag,
        'authorization': dict(authorization),
        'evidence_receipts': evidence_receipts,
        'evidence_receipt_sha256': dict(EVIDENCE_RECEIPT_SHA256),
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'docker_inspected': bool(image.get('docker_inspected', False)),
    }


def validate_production_identity(config: FormalConfig) -> dict[str, Any]:
    """Run the pinned read-only image inspect and exact bag probe."""
    return validate_preflight_identity(
        config, image_probe=_default_image_probe, bag_probe=_default_bag_probe)


def reserve_attempt_root(config: FormalConfig) -> dict[str, Any]:
    """Reserve one genuinely fresh root; never reuse or overwrite it."""
    root = _absolute_path(config.root, 'attempt root')
    repo = _absolute_path(config.repo_root, 'repository root')
    bag = _absolute_path(config.bag_path, 'bag path')
    _reject_overlap(root, repo)
    _reject_overlap(root, bag)
    if not repo.is_dir():
        raise LaunchError('REPOSITORY_MISSING', f'repository root is missing: {repo}')
    if root.exists() or root.is_symlink():
        raise LaunchError('ROOT_ALREADY_EXISTS', f'attempt root must be fresh: {root}')
    if not root.parent.is_dir() or root.parent.is_symlink():
        raise LaunchError('ROOT_PARENT_INVALID', f'attempt root parent is invalid: {root.parent}')
    try:
        root.mkdir()
    except FileExistsError as error:
        raise LaunchError('ROOT_RACE', f'attempt root appeared during reservation: {root}') from error
    marker = {
        'schema_version': SCHEMA_VERSION,
        'contract_id': CONTRACT_ID,
        'launcher_contract_id': LAUNCHER_CONTRACT_ID,
        'status': 'RESERVED_EMPTY_ROOT',
        'created_at_utc': _utc_now(),
        'root': str(root),
        'repo_root': str(repo),
        'bag_path': str(bag),
        'bag_opened': False,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'launcher_path': str(Path(__file__).resolve()),
        'launcher_sha256': file_sha256(Path(__file__).resolve()),
    }
    marker_sha = atomic_json(root / 'attempt_root_marker.json', marker)
    marker['receipt_sha256'] = marker_sha
    return marker


def _closure_bindings(identity: Optional[Mapping[str, Any]] = None) -> dict[str, Any]:
    """Return non-secret input/source/image/evidence bindings for closure."""
    fallback = {
        'input': {
            'path': str(BAG_PATH), 'bytes': BAG_BYTES, 'sha256': BAG_SHA256,
            'expected_messages': EXPECTED_MESSAGES,
            'expected_topic_counts': dict(EXPECTED_TOPIC_COUNTS),
            'required_evaluation_end_timestamp_seconds':
                REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS,
        },
        'profile': {'path': str(PROFILE_PATH), 'sha256': PROFILE_SHA256},
        'patch': {'path': str(V10_PATCH_PATH), 'sha256': V10_PATCH_SHA256},
        'image': {'tag': IMAGE_TAG, 'id': IMAGE_ID},
        'sources': {
            'launcher': {'path': str(Path(__file__).resolve()),
                         'sha256': file_sha256(Path(__file__).resolve())},
            'wrapper': {'path': str(WRAPPER_PATH), 'sha256': WRAPPER_SHA256},
            'feeder': {'path': str(FEEDER_PATH), 'sha256': FEEDER_SHA256},
            'quiescence': {
                'path': str(QUIESCENCE_SCRIPT), 'sha256': QUIESCENCE_SCRIPT_SHA256},
        },
        'evidence': {
            key: {'path': str(EVIDENCE_RECEIPT_PATHS[key]), 'sha256': digest}
            for key, digest in EVIDENCE_RECEIPT_SHA256.items()
        },
        'authorization': {
            'path': str(AUTHORIZATION_PATH), 'sha256': AUTHORIZATION_SHA256,
            'status': 'AUTHORIZED', 'attempt_root': str(AUTHORIZED_ATTEMPT_ROOT),
        },
    }
    if identity is None:
        return fallback
    input_binding = dict(identity.get('bag') or fallback['input'])
    if _identity_input_opened(identity):
        input_binding['opened'] = True
        input_binding['input_opened'] = True
    return {
        'input': input_binding,
        'profile': dict(identity.get('profile') or fallback['profile']),
        'patch': dict(identity.get('patch') or fallback['patch']),
        'image': dict(identity.get('image') or fallback['image']),
        'sources': dict(identity.get('sources') or fallback['sources']),
        'evidence': dict(identity.get('evidence_receipts') or fallback['evidence']),
        'authorization': dict(identity.get('authorization') or fallback['authorization']),
    }


def _identity_input_opened(identity: Optional[Mapping[str, Any]]) -> bool:
    """Only a completed production streaming probe opens the input bag."""
    if not isinstance(identity, Mapping):
        return False
    bag = identity.get('bag')
    probe = bag.get('probe') if isinstance(bag, Mapping) else None
    return isinstance(probe, Mapping) and probe.get('opened') is True


def _write_closure(root: Path, *, kind: str, message: str,
                   extra: Optional[Mapping[str, Any]] = None,
                   status: str = 'FAIL_CLOSED',
                   identity: Optional[Mapping[str, Any]] = None,
                   command_sha256: Optional[str] = None) -> str:
    bindings = _closure_bindings(identity)
    input_opened = _identity_input_opened(identity)
    value: dict[str, Any] = {
        'schema_version': SCHEMA_VERSION,
        'contract_id': CONTRACT_ID,
        'launcher_contract_id': LAUNCHER_CONTRACT_ID,
        'status': status,
        'terminal_status': status,
        'failure_kind': None if status == 'PASS' else kind,
        'message': message,
        'closed_at_utc': _utc_now(),
        'runner_start_attempted': False,
        'bag_opened': input_opened,
        'input_opened': input_opened,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'map_saved': False,
        'manual_retry': False,
        'retry_count': 0,
        'command_sha256': command_sha256,
        'bindings': bindings,
        'input': dict(bindings['input']),
        'profile': dict(bindings['profile']),
        'patch': dict(bindings['patch']),
        'image': dict(bindings['image']),
        'sources': dict(bindings['sources']),
        'evidence': dict(bindings['evidence']),
        'authorization': dict(bindings['authorization']),
        'result': {
            'expected_topic_counts': dict(EXPECTED_TOPIC_COUNTS),
            'published_topic_counts': None,
            'received_topic_counts': None,
            'acknowledged_topic_counts': None,
            'required_evaluation_end_timestamp_seconds':
                REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS,
            'rtf': None,
            'rtf_available': False,
        },
        'process': None,
        'container': None,
        'cleanup': None,
        'timing': None,
    }
    if extra:
        value.update(dict(extra))
    evidence_value = value.get('evidence')
    if isinstance(evidence_value, Mapping):
        timing_value = evidence_value.get('timing')
        if isinstance(timing_value, Mapping):
            value['timing'] = dict(timing_value)
            bindings_value = dict(value.get('bindings') or {})
            bindings_value['timing'] = dict(timing_value)
            value['bindings'] = bindings_value
    result = value.get('result')
    if isinstance(result, Mapping):
        value['counts'] = {
            'expected': result.get('expected_topic_counts'),
            'published': result.get('published_topic_counts'),
            'received': result.get('received_topic_counts'),
            'acknowledged': result.get('acknowledged_topic_counts'),
        }
        value['required_evaluation_end_timestamp_seconds'] = result.get(
            'required_evaluation_end_timestamp_seconds')
        value['rtf'] = result.get('rtf')
    receipt_path = root / 'closure_receipt.json'
    sidecar_path = receipt_path.with_name(receipt_path.name + '.sha256')
    if (receipt_path.exists() or receipt_path.is_symlink() or
            sidecar_path.exists() or sidecar_path.is_symlink()):
        raise LaunchError('RECEIPT_OVERWRITE', f'refusing to overwrite closure receipt: {root}')
    receipt_sha = atomic_json(receipt_path, value)
    atomic_sha256_sidecar(receipt_path, receipt_sha)
    return receipt_sha


def _load_pass_quiescence(path: Path) -> tuple[dict[str, Any], str]:
    if not path.is_file() or path.is_symlink():
        raise LaunchError('PREFLIGHT_RECEIPT_MISSING', f'missing quiescence receipt: {path}')
    digest = file_sha256(path)
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise LaunchError('PREFLIGHT_RECEIPT_INVALID', 'quiescence receipt is invalid') from error
    if (value.get('schema_version') != SCHEMA_VERSION or
            value.get('contract_version') != QUIESCENCE_CONTRACT_VERSION or
            value.get('status') != 'PASS' or
            value.get('runner_start_allowed') is not True):
        raise LaunchError('PREFLIGHT_FAIL_CLOSED', 'quiescence did not authorize runner start')
    if (value.get('ground_truth_content_opened') is not False or
            value.get('scorer_invoked') is not False):
        raise LaunchError('PREFLIGHT_SAFETY_INVALID', 'quiescence safety flags are invalid')
    observation = value.get('observation')
    if (not isinstance(observation, Mapping) or
            observation.get('sample_seconds') != 5.0 or
            not isinstance(observation.get('nproc'), int) or
            observation.get('nproc', 0) <= 0 or
            observation.get('proc_race_skips') != 0 or
            observation.get('forbidden_processes') != []):
        raise LaunchError('PREFLIGHT_OBSERVATION_INVALID', 'quiescence observation is incomplete')
    checks = observation.get('checks')
    limits = observation.get('limits')
    cpu = observation.get('cpu')
    load = observation.get('loadavg')
    if (not isinstance(checks, Mapping) or not checks or
            any(item is not True for item in checks.values()) or
            not isinstance(limits, Mapping) or
            limits.get('max_cpu_busy_percent') != 5.0 or
            limits.get('max_load1_per_cpu') != 0.5 or
            not isinstance(cpu, Mapping) or
            not isinstance(load, Mapping)):
        raise LaunchError('PREFLIGHT_OBSERVATION_INVALID', 'quiescence limits/checks are invalid')
    try:
        busy = float(cpu.get('busy_percent'))
        load_per_cpu = float(load.get('load1_per_cpu'))
    except (TypeError, ValueError) as error:
        raise LaunchError('PREFLIGHT_OBSERVATION_INVALID', 'quiescence CPU/load values are invalid') from error
    if (not math.isfinite(busy) or busy > 5.0 or
            not math.isfinite(load_per_cpu) or load_per_cpu > 0.5):
        raise LaunchError('PREFLIGHT_OBSERVATION_INVALID', 'quiescence CPU/load limits exceeded')
    return value, digest


def _quiescence_command(config: FormalConfig, receipt: Path) -> tuple[str, ...]:
    script = _repo_path(config, config.quiescence_script)
    if not script.is_file() or script.is_symlink():
        raise LaunchError('QUIESCENCE_SCRIPT_INVALID', f'invalid quiescence script: {script}')
    if file_sha256(script) != QUIESCENCE_SCRIPT_SHA256:
        raise LaunchError('QUIESCENCE_SCRIPT_MISMATCH', 'quiescence script bytes differ from pin')
    return (
        sys.executable, str(script), '--output', str(receipt),
        '--sample-seconds', '5.0', '--max-busy-percent', '5.0',
        '--max-load-per-cpu', '0.5')


def _run_quiescence(config: FormalConfig, receipt: Path) -> int:
    """Run the read-only quiescence checker exactly once."""
    command = _quiescence_command(config, receipt)
    completed = subprocess.run(command, check=False)
    return int(completed.returncode)


def _container_name(config: FormalConfig, output: Path) -> str:
    """Derive one validated name from the reserved attempt/output roots."""
    seed = f'{config.root.resolve(strict=False)}::{output.resolve(strict=False)}'
    suffix = hashlib.sha256(seed.encode('utf-8')).hexdigest()[:20]
    name = f'm6a10-v10-formal-{suffix}'
    if re.fullmatch(r'[a-z0-9][a-z0-9_.-]{0,127}', name) is None:
        raise LaunchError('CONTAINER_NAME_INVALID', 'derived container name is invalid')
    return name


def build_safe_docker_argv(config: FormalConfig, output_dir: Path) -> list[str]:
    """Return the exact production v10 Docker argv without invoking Docker."""
    if config.image_tag != IMAGE_TAG or config.image_id != IMAGE_ID:
        raise LaunchError('IMAGE_IDENTITY_MISMATCH', 'v10 image identity is not pinned')
    bag = _absolute_path(config.bag_path, 'bag path')
    if bag != BAG_PATH:
        raise LaunchError('BAG_IDENTITY_MISMATCH', 'production argv requires the pinned bag path')
    output = _absolute_path(output_dir, 'output directory')
    if output.exists() or output.is_symlink():
        raise LaunchError('OUTPUT_OVERWRITE', f'output directory must be fresh: {output}')
    if not output.parent.is_dir() or output.parent.is_symlink():
        raise LaunchError('OUTPUT_PARENT_INVALID', f'output parent is invalid: {output.parent}')
    repo = _absolute_path(config.repo_root, 'repository root')
    wrapper = _absolute_path(_repo_path(config, WRAPPER_PATH), 'v10 wrapper source')
    feeder = _absolute_path(_repo_path(config, FEEDER_PATH), 'v10 feeder source')
    if not wrapper.is_file() or wrapper.is_symlink():
        raise LaunchError('WRAPPER_IDENTITY_MISSING', f'wrapper is invalid: {wrapper}')
    if file_sha256(wrapper) != WRAPPER_SHA256:
        raise LaunchError('WRAPPER_IDENTITY_MISMATCH', 'v10 wrapper bytes differ from pin')
    if not feeder.is_file() or feeder.is_symlink():
        raise LaunchError('FEEDER_IDENTITY_MISSING', f'feeder is invalid: {feeder}')
    if file_sha256(feeder) != FEEDER_SHA256:
        raise LaunchError('FEEDER_IDENTITY_MISMATCH', 'v10 feeder bytes differ from pin')
    _reject_overlap(output, bag)
    _reject_overlap(output, repo)
    for value, label in ((str(bag), 'bag path'), (str(output), 'output path'),
                         (str(wrapper), 'wrapper path'), (str(feeder), 'feeder path')):
        _validated_text(value, label)
    lowered = f'{bag} {output} {wrapper} {feeder}'.lower()
    if any(forbidden in lowered for forbidden in ('ground_truth', 'scorer', 'map')):
        raise LaunchError('GT_SCORER_ARGUMENT', 'GT/scorer/map paths are forbidden')
    image_ref = _validated_text(config.image_id, 'image ID')
    container_name = _container_name(config, output)
    raw_evidence = '/out/consumer_evidence.json'
    callback_evidence = '/out/callback_consumer_evidence.json'
    if raw_evidence == callback_evidence:
        raise LaunchError('EVIDENCE_PATH_ALIAS', 'terminal and callback evidence paths must differ')
    env = [
        'OUT_DIR=/out',
        'BAG_PATH=/input/ntu_viral.bag',
        f'M6A10_PROFILE_PATH={PROFILE_PATH}',
        f'M6A10_PROFILE_SHA256={PROFILE_SHA256}',
        f'M6A10_WRAPPER_SHA256={WRAPPER_SHA256}',
        f'M6A10_BAG_BYTES={BAG_BYTES}',
        f'M6A10_BAG_SHA256={BAG_SHA256}',
        f'M6A10_FAST_EXPECTED_MESSAGES={EXPECTED_MESSAGES}',
        f'M6A10_FAST_EXPECTED_LIDAR_MESSAGES={EXPECTED_TOPIC_COUNTS["lidar"]}',
        f'M6A10_FAST_EXPECTED_IMU_MESSAGES={EXPECTED_TOPIC_COUNTS["imu"]}',
        f'M6A10_FAST_EXPECTED_IMAGE_MESSAGES={EXPECTED_TOPIC_COUNTS["image"]}',
        f'M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS={REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS}',
        f'M6A10_PHASE_CONTRACT_VERSION={PHASE_CONTRACT_VERSION}',
        f'M6A10_PHASE_MODE={PHASE_MODE}',
        f'M6A10_CONSUMER_EVIDENCE={callback_evidence}',
        f'M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE={raw_evidence}',
        f'M6A10_ONLINE_TIMING_EVIDENCE=/out/{TIMING_FILENAME}',
        f'M6A10_SENSOR_DURATION_SECONDS={SENSOR_DURATION_SECONDS:.9f}',
        f'M6A10_TIMING_CONTRACT_VERSION={TIMING_CONTRACT_VERSION}',
        f'M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS={EXPECTED_CALLBACK_LATENCY_SECONDS}',
        f'M6A10_FAST_MAX_BACKLOG_MESSAGES={EXPECTED_BACKLOG_MESSAGES}',
        f'M6A10_FAST_MAX_END_GAP_SECONDS={EXPECTED_END_GAP_SECONDS}',
        f'M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS={EXPECTED_MIN_TERMINAL_POLL_WALL_SECONDS}',
        f'M6A10_FAST_FEEDER_SHA256={FEEDER_SHA256}',
        f'M6A10_FEEDER_TIMEOUT_SECONDS={FEEDER_TIMEOUT_SECONDS}',
        f'M6A10_TERMINAL_DRAIN_TIMEOUT_SECONDS={TERMINAL_DRAIN_TIMEOUT_SECONDS}',
        f'M6A10_STARTUP_TIMEOUT_SECONDS={STARTUP_TIMEOUT_SECONDS}',
        f'M6A10_TRAJECTORY_TIMEOUT_SECONDS={TRAJECTORY_TIMEOUT_SECONDS}',
        f'M6A10_RPC_TIMEOUT_SECONDS={RPC_TIMEOUT_SECONDS}',
    ]
    argv = [
        'docker', 'run',
        '--name', container_name,
        '--pull=never', '--init', '--network', 'none', '--read-only',
        '--tmpfs', '/tmp:rw,noexec,nosuid,size=128m',
        '--tmpfs', '/root/.ros:rw,noexec,nosuid,size=32m',
        '--mount', f'type=bind,src={bag},dst=/input/ntu_viral.bag,readonly',
        '--mount', f'type=bind,src={output},dst=/out,readonly=false',
        '--mount', f'type=bind,src={wrapper},dst=/runner/v10_runtime.sh,readonly',
        '--mount', f'type=bind,src={feeder},dst={FEEDER_CONTAINER_PATH},readonly',
    ]
    for item in env:
        argv.extend(['--env', item])
    argv.extend(['--entrypoint', '/runner/v10_runtime.sh', image_ref])
    forbidden = (
        '--rm', '--privileged', '--cap-add', '--cap-drop', '--device',
        '--pid=host', '--ipc=host', '--uts=host', '--userns=host',
        '/bin/sh', '/bin/bash', '-c', '-lc', 'rosbag', 'ground_truth',
        'scorer', 'map_save',
    )
    rendered = ' '.join(argv).lower()
    rendered_forbidden = ('/bin/sh', '/bin/bash', 'rosbag', 'ground_truth', 'scorer', 'map_save')
    if (any(item in argv for item in forbidden) or
            any(item in rendered for item in rendered_forbidden)):
        raise LaunchError('UNSAFE_DOCKER_ARG', 'unsafe Docker capability or shell in fixed argv')
    if IMAGE_TAG in argv or len([item for item in argv if item == '--mount']) != 4:
        raise LaunchError('UNSAFE_DOCKER_ARG', 'argv image or mount contract is invalid')
    if len([item for item in argv if item == '--tmpfs']) != 2:
        raise LaunchError('UNSAFE_DOCKER_ARG', 'argv tmpfs contract is invalid')
    if argv[-1] != IMAGE_ID or argv[-2:] != ['/runner/v10_runtime.sh', IMAGE_ID]:
        raise LaunchError('UNSAFE_DOCKER_ARG', 'entrypoint must run directly from the image ID')
    return argv


# Alias used by callers that share the older launcher naming convention.
build_runner_argv = build_safe_docker_argv


def _memory_usage_bytes(value: str) -> Optional[int]:
    match = re.match(r'^\s*([0-9]+(?:\.[0-9]+)?)\s*([kmgt]?i?b)\b',
                     value or '', re.IGNORECASE)
    if match is None:
        return None
    units = {
        'b': 1, 'kb': 1000, 'kib': 1024,
        'mb': 1000 ** 2, 'mib': 1024 ** 2,
        'gb': 1000 ** 3, 'gib': 1024 ** 3,
        'tb': 1000 ** 4, 'tib': 1024 ** 4,
    }
    multiplier = units.get(match.group(2).lower())
    return None if multiplier is None else int(float(match.group(1)) * multiplier)


def _docker_container_inspect(container_name: str) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            ('docker', 'inspect', container_name), check=False,
            capture_output=True, text=True)
    except (OSError, subprocess.SubprocessError) as error:
        return {'status': 'inspect_error', 'error': str(error)}
    stderr_sha = hashlib.sha256(completed.stderr.encode()).hexdigest()
    if completed.returncode != 0:
        lowered = completed.stderr.lower()
        return {
            'status': 'not_found' if 'no such' in lowered else 'inspect_error',
            'returncode': completed.returncode, 'stderr_sha256': stderr_sha,
        }
    try:
        documents = json.loads(completed.stdout)
        item = documents[0]
        state = item.get('State') or {}
    except (IndexError, TypeError, AttributeError, json.JSONDecodeError) as error:
        return {
            'status': 'invalid_inspect_json',
            'stdout_sha256': hashlib.sha256(completed.stdout.encode()).hexdigest(),
            'error': str(error),
        }
    if not isinstance(item, Mapping) or not isinstance(state, Mapping):
        return {'status': 'invalid_inspect_json'}
    return {
        'status': 'present', 'id': item.get('Id'), 'name': item.get('Name'),
        'running': state.get('Running'), 'state': state.get('Status'),
        'exit_code': state.get('ExitCode'), 'oom_killed': state.get('OOMKilled'),
        'started_at': state.get('StartedAt'), 'finished_at': state.get('FinishedAt'),
    }


def _docker_container_stats(container_name: str) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            ('docker', 'stats', '--no-stream', '--format', '{{json .}}',
             container_name), check=False, capture_output=True, text=True)
    except (OSError, subprocess.SubprocessError) as error:
        return {'status': 'stats_error', 'error': str(error)}
    raw = completed.stdout.strip()
    if completed.returncode != 0:
        return {
            'status': 'stats_unavailable', 'returncode': completed.returncode,
            'stderr_sha256': hashlib.sha256(completed.stderr.encode()).hexdigest(),
        }
    try:
        document = json.loads(raw.splitlines()[0])
    except (IndexError, json.JSONDecodeError) as error:
        return {
            'status': 'invalid_stats_json',
            'stdout_sha256': hashlib.sha256(raw.encode()).hexdigest(),
            'error': str(error),
        }
    if not isinstance(document, Mapping):
        return {'status': 'invalid_stats_json'}
    memory = document.get('MemUsage', '')
    return {
        'status': 'present', 'memory_usage': memory,
        'memory_usage_bytes': _memory_usage_bytes(memory),
        'cpu_percent': document.get('CPUPerc'), 'pids': document.get('PIDs'),
        'raw_sha256': hashlib.sha256(raw.encode()).hexdigest(),
    }


def _docker_container_top(container_name: str) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            ('docker', 'top', container_name, '-eo',
             'pid,ppid,stat,etime,pcpu,pmem,args'), check=False,
            capture_output=True, text=True)
    except (OSError, subprocess.SubprocessError) as error:
        return {'status': 'top_error', 'error': str(error)}
    if completed.returncode != 0:
        return {
            'status': 'top_unavailable', 'returncode': completed.returncode,
            'stderr_sha256': hashlib.sha256(completed.stderr.encode()).hexdigest(),
        }
    return {
        'status': 'present',
        'stdout_sha256': hashlib.sha256(completed.stdout.encode()).hexdigest(),
    }


def _docker_container_stop(container_name: str) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            ('docker', 'stop', '--time', str(int(CONTAINER_STOP_GRACE_SECONDS)),
             container_name), check=False, capture_output=True, text=True)
    except (OSError, subprocess.SubprocessError) as error:
        return {'status': 'stop_error', 'error': str(error)}
    return {
        'status': 'requested', 'returncode': completed.returncode,
        'stdout_sha256': hashlib.sha256(completed.stdout.encode()).hexdigest(),
        'stderr_sha256': hashlib.sha256(completed.stderr.encode()).hexdigest(),
    }


def _docker_container_remove(container_name: str) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            ('docker', 'rm', container_name), check=False,
            capture_output=True, text=True)
    except (OSError, subprocess.SubprocessError) as error:
        return {'status': 'remove_error', 'error': str(error)}
    return {
        'status': 'removed' if completed.returncode == 0 else 'remove_failed',
        'returncode': completed.returncode,
        'stdout_sha256': hashlib.sha256(completed.stdout.encode()).hexdigest(),
        'stderr_sha256': hashlib.sha256(completed.stderr.encode()).hexdigest(),
    }


def _capture_container_diagnostics(
        container_name: str,
        inspect: Callable[[str], Mapping[str, Any]],
        stats: Callable[[str], Mapping[str, Any]],
        top: Callable[[str], Mapping[str, Any]]) -> dict[str, Any]:
    def capture(function: Callable[[str], Mapping[str, Any]]) -> dict[str, Any]:
        try:
            value = function(container_name)
            return dict(value)
        except BaseException as error:
            return {'status': 'diagnostic_error', 'error': repr(error)}
    inspected = capture(inspect)
    sampled = capture(stats)
    process_table = capture(top)
    return {
        'inspect': inspected,
        'state': {
            key: inspected.get(key)
            for key in ('state', 'running', 'exit_code', 'oom_killed', 'started_at', 'finished_at')
        },
        'stats': sampled,
        'top': process_table,
        'oom_killed': inspected.get('oom_killed'),
        'captured_at_utc': _utc_now(),
    }


def _remove_verified_exited_container(
        container_name: str,
        inspect: Callable[[str], Mapping[str, Any]],
        remove: Callable[[str], Mapping[str, Any]]) -> dict[str, Any]:
    """Remove only after a fresh inspect proves the container is stopped."""
    try:
        before = dict(inspect(container_name))
    except BaseException as error:
        return {'status': 'not_removed_unverified', 'error': repr(error)}
    if before.get('status') == 'not_found':
        return {'status': 'not_present', 'before': before}
    if before.get('status') != 'present' or before.get('running') is not False:
        return {'status': 'not_removed_running_or_unverified', 'before': before}
    try:
        removed = dict(remove(container_name))
    except BaseException as error:
        return {'status': 'remove_error', 'before': before, 'error': repr(error)}
    try:
        after = dict(inspect(container_name))
    except BaseException as error:
        return {'status': 'remove_unverified', 'before': before, 'remove': removed,
                'error': repr(error)}
    removed['before'] = before
    removed['after'] = after
    removed['status'] = (
        'removed' if removed.get('status') == 'removed' and
        after.get('status') == 'not_found' else 'remove_unverified')
    return removed


def _supervise_container(
        process: Any, *, container_name: str, watchdog_seconds: float,
        stats_interval_seconds: float,
        inspect: Callable[[str], Mapping[str, Any]],
        stats: Callable[[str], Mapping[str, Any]],
        top: Callable[[str], Mapping[str, Any]],
        stop: Callable[[str], Mapping[str, Any]],
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep) -> tuple[int, dict[str, Any]]:
    """Supervise exactly one container process, with diagnostics-first timeout."""
    if watchdog_seconds <= 0 or stats_interval_seconds <= 0:
        raise LaunchError('WATCHDOG_CONFIGURATION_INVALID', 'watchdog/stat intervals must be positive')
    started = clock()
    next_stats = started
    peak_rss: Optional[int] = None
    samples = 0
    stats_samples = 0
    last_inspect: Mapping[str, Any] = {'status': 'not_sampled'}
    last_stats: Mapping[str, Any] = {'status': 'not_sampled'}
    watchdog_triggered = False
    stop_result: Optional[Mapping[str, Any]] = None
    while process.poll() is None:
        now = clock()
        last_inspect = inspect(container_name)
        samples += 1
        if now >= next_stats:
            last_stats = stats(container_name)
            stats_samples += 1
            memory = last_stats.get('memory_usage_bytes')
            if isinstance(memory, int) and (peak_rss is None or memory > peak_rss):
                peak_rss = memory
            next_stats = now + stats_interval_seconds
        if now - started >= watchdog_seconds:
            watchdog_triggered = True
            diagnostics = _capture_container_diagnostics(container_name, inspect, stats, top)
            # Only the watchdog may request a stop, and only after diagnostics.
            stop_result = stop(container_name)
            try:
                process.wait(timeout=CONTAINER_STOP_WAIT_SECONDS)
            except TypeError:
                process.wait()
            except (subprocess.TimeoutExpired, OSError) as error:
                stop_result = {**dict(stop_result or {}), 'wait_error': repr(error)}
            return_code = process.poll()
            if return_code is None:
                return_code = 124
            final = {
                'status': 'watchdog_timeout', 'returncode': int(return_code),
                'watchdog_triggered': True, 'samples': samples,
                'stats_samples': stats_samples, 'peak_rss_bytes': peak_rss,
                'peak_rss': peak_rss,
                'oom_killed': diagnostics.get('oom_killed'),
                'diagnostics_before_stop': diagnostics,
                'stop': dict(stop_result or {}),
                'diagnostics_after_stop': _capture_container_diagnostics(
                    container_name, inspect, stats, top),
            }
            return int(return_code), final
        sleep(min(1.0, max(0.0, next_stats - now)))
    return_code = process.wait()
    diagnostics = _capture_container_diagnostics(container_name, inspect, stats, top)
    final_memory = diagnostics.get('stats', {}).get('memory_usage_bytes')
    if isinstance(final_memory, int) and (peak_rss is None or final_memory > peak_rss):
        peak_rss = final_memory
    final = {
        'status': 'natural_exit', 'returncode': int(return_code),
        'watchdog_triggered': watchdog_triggered, 'samples': samples,
        'stats_samples': stats_samples, 'peak_rss_bytes': peak_rss,
        'peak_rss': peak_rss,
        'oom_killed': diagnostics.get('oom_killed'),
        'diagnostics_before_cleanup': diagnostics,
        'last_inspect': dict(last_inspect), 'last_stats': dict(last_stats),
    }
    return int(return_code), final


def capture_identity_receipt(
        config: FormalConfig, output: Path, *,
        image_probe: Optional[Callable[[FormalConfig], Mapping[str, Any]]] = None,
        bag_probe: Optional[Callable[[FormalConfig], Mapping[str, Any]]] = None,
        authorization_validator: Optional[Callable[[FormalConfig], Mapping[str, Any]]] = None,
) -> dict[str, Any]:
    """Write an immutable source/image declaration without opening the bag."""
    identity = validate_preflight_identity(
        config, image_probe=image_probe, bag_probe=bag_probe,
        authorization_validator=authorization_validator)
    receipt = {
        'schema_version': SCHEMA_VERSION,
        'receipt_kind': 'm6a10_v10_formal_identity_preflight',
        'contract_id': CONTRACT_ID,
        'launcher_contract_id': LAUNCHER_CONTRACT_ID,
        'status': 'PASS',
        'launcher': {
            'path': str(Path(__file__).resolve()),
            'sha256': file_sha256(Path(__file__).resolve()),
        },
        'identity': identity,
        'execution': {
            'runner_start_attempted': False,
            'quiescence_started': False,
            'docker_run_started': False,
            'bag_opened': False,
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
        },
    }
    receipt_sha = atomic_json(output, receipt)
    return {'path': str(output), 'sha256': receipt_sha, 'receipt': receipt}


def _load_script_module(name: str, path: Path) -> Any:
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise LaunchError('SOURCE_MODULE_INVALID', f'cannot load module: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _default_binder(raw_path: Path, output_path: Path,
                    profile_path: Path) -> Mapping[str, Any]:
    module = _load_script_module(
        'fast_livo2_v10_binder_runtime', ROOT / 'scripts' /
        'bind_fast_livo2_v10_consumer_evidence.py')
    return module.bind_consumer_evidence(
        raw_path=raw_path, output_path=output_path, profile_path=profile_path)


def _default_compositor(profile_path: Path, consumer_path: Path,
                        feeder_path: Path, output_path: Path) -> Mapping[str, Any]:
    module = _load_script_module(
        'fast_livo2_v10_compositor_runtime', ROOT / 'scripts' /
        'compose_fast_livo2_terminal_evidence.py')
    return module.compose(
        profile_path=profile_path, consumer_path=consumer_path,
        feeder_path=feeder_path, output_path=output_path)


def validate_online_timing(
        path: Path,
        *, expected_sensor_duration: float = SENSOR_DURATION_SECONDS) -> dict[str, Any]:
    """Validate wrapper-owned timing and derive the closure RTF.

    The compositor is deliberately not an authority for runtime.  This file
    is produced by the mounted wrapper after terminal finalization, and the
    host independently checks its immutable boundary and safety fields.
    """
    path = Path(path)
    staging = path.with_name(path.name + '.part')
    if staging.exists() or staging.is_symlink():
        raise LaunchError('TIMING_STAGING_PRESENT', f'timing staging file exists: {staging}')
    if path.is_symlink() or not path.is_file():
        raise LaunchError('MISSING_TIMING_EVIDENCE', f'online timing is missing: {path}')
    try:
        payload = path.read_bytes()
        value = json.loads(payload.decode('utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise LaunchError('TIMING_INVALID', 'online timing JSON is invalid') from error
    if not isinstance(value, Mapping):
        raise LaunchError('TIMING_INVALID', 'online timing is not an object')
    if (value.get('schema_version') != TIMING_SCHEMA_VERSION or
            value.get('contract_version') != TIMING_CONTRACT_VERSION or
            value.get('status') != 'PASS'):
        raise LaunchError('TIMING_CONTRACT_INVALID', 'online timing schema/contract/status mismatch')

    start = value.get('input_start_monotonic_ns', value.get('start_monotonic_ns'))
    end = value.get('drain_end_monotonic_ns', value.get('end_monotonic_ns'))
    for field, item in (('input_start_monotonic_ns', start),
                        ('drain_end_monotonic_ns', end)):
        if (isinstance(item, bool) or not isinstance(item, int) or item < 0):
            raise LaunchError('TIMING_INVALID', f'{field} is not a nonnegative integer')
    if 'start_monotonic_ns' in value and value.get('start_monotonic_ns') != start:
        raise LaunchError('TIMING_INVALID', 'timing start aliases differ')
    if 'end_monotonic_ns' in value and value.get('end_monotonic_ns') != end:
        raise LaunchError('TIMING_INVALID', 'timing end aliases differ')
    if end <= start:
        raise LaunchError('TIMING_INVALID', 'timing end does not follow start')
    try:
        duration = float(value.get('duration_seconds'))
        sensor_duration = float(value.get('sensor_duration_seconds'))
        rtf = float(value.get('online_compute_rtf'))
    except (TypeError, ValueError) as error:
        raise LaunchError('TIMING_INVALID', 'timing numeric fields are invalid') from error
    expected_duration = (end - start) / 1.0e9
    if (not math.isfinite(duration) or duration < 0.0 or
            not math.isclose(duration, expected_duration, rel_tol=0.0, abs_tol=1.0e-9)):
        raise LaunchError('TIMING_INVALID', 'timing duration does not match monotonic bounds')
    if (not math.isfinite(sensor_duration) or sensor_duration != float(expected_sensor_duration) or
            sensor_duration <= 0.0):
        raise LaunchError('TIMING_SENSOR_DURATION_MISMATCH', 'sensor duration is not exact')
    expected_rtf = duration / sensor_duration
    if (not math.isfinite(rtf) or rtf < 0.0 or
            not math.isclose(rtf, expected_rtf, rel_tol=0.0, abs_tol=1.0e-12)):
        raise LaunchError('TIMING_RTF_INVALID', 'online compute RTF is not derived from timing')
    if (value.get('ground_truth_content_opened') is not False or
            value.get('scorer_invoked') is not False):
        raise LaunchError('TIMING_SAFETY_INVALID', 'timing safety fields are not false')
    return {
        'schema_version': TIMING_SCHEMA_VERSION,
        'contract_version': TIMING_CONTRACT_VERSION,
        'status': value.get('status'),
        'path': str(path),
        'sha256': hashlib.sha256(payload).hexdigest(),
        'input_start_monotonic_ns': start,
        'drain_end_monotonic_ns': end,
        'duration_seconds': duration,
        'sensor_duration_seconds': sensor_duration,
        'online_compute_rtf': rtf,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }


def _terminal_metrics(document: Mapping[str, Any]) -> dict[str, Any]:
    counts = document.get('counts')
    if not isinstance(counts, Mapping):
        raise LaunchError('COMPOSITOR_REJECT', 'composed evidence counts are missing')
    bound_counts: dict[str, Any] = {}
    for name in ('expected', 'published', 'received', 'acknowledged'):
        value = counts.get(name)
        if not isinstance(value, Mapping) or dict(value) != EXPECTED_TOPIC_COUNTS:
            raise LaunchError('COMPOSITOR_REJECT', f'composed {name} counts are not exact')
        bound_counts[name] = dict(value)
    required_end = document.get('required_evaluation_end_timestamp_seconds')
    if (isinstance(required_end, bool) or not isinstance(required_end, (int, float)) or
            float(required_end) != REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS):
        raise LaunchError('COMPOSITOR_REJECT', 'composed required end is not exact')
    if (document.get('ground_truth_content_opened') is not False or
            document.get('scorer_invoked') is not False):
        raise LaunchError('COMPOSITOR_SAFETY_INVALID', 'composed safety flags are not false')
    return {
        'expected_topic_counts': bound_counts['expected'],
        'published_topic_counts': bound_counts['published'],
        'received_topic_counts': bound_counts['received'],
        'acknowledged_topic_counts': bound_counts['acknowledged'],
        'required_evaluation_end_timestamp_seconds': float(required_end),
        'rtf': None, 'rtf_available': False,
    }


def _bind_and_compose(
        output_dir: Path, config: FormalConfig,
        binder: Callable[[Path, Path, Path], Mapping[str, Any]],
        compositor: Callable[[Path, Path, Path, Path], Mapping[str, Any]]) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    raw_path = output_dir / 'consumer_evidence.json'
    feeder_path = output_dir / 'feeder_receipt.json'
    timing_path = output_dir / TIMING_FILENAME
    bound_path = output_dir / 'consumer_evidence.host_bound.json'
    composed_path = output_dir / 'terminal_evidence.v3.json'
    if raw_path.is_symlink() or not raw_path.is_file():
        raise LaunchError('MISSING_RAW_EVIDENCE', 'raw mapper evidence is missing')
    if feeder_path.is_symlink() or not feeder_path.is_file():
        raise LaunchError('MISSING_FEEDER_EVIDENCE', 'feeder evidence is missing')
    timing = validate_online_timing(timing_path)
    try:
        bound_receipt = dict(binder(raw_path, bound_path, _repo_path(config, config.profile_path)))
    except LaunchError:
        raise
    except BaseException as error:
        raise LaunchError('BINDER_REJECT', repr(error)) from error
    if bound_receipt.get('status') != 'PASS' or bound_path.is_symlink() or not bound_path.is_file():
        raise LaunchError('BINDER_REJECT', 'binder did not publish validated PASS evidence')
    try:
        composed = dict(compositor(
            profile_path=_repo_path(config, config.profile_path),
            consumer_path=bound_path, feeder_path=feeder_path,
            output_path=composed_path))
    except LaunchError:
        raise
    except BaseException as error:
        raise LaunchError('COMPOSITOR_REJECT', repr(error)) from error
    if (composed.get('status') != 'pass' or composed.get('schema_version') != 3 or
            composed.get('contract_version') != PHASE_CONTRACT_VERSION or
            composed_path.is_symlink() or not composed_path.is_file()):
        raise LaunchError('COMPOSITOR_REJECT', 'compositor did not publish schema-v3 PASS evidence')
    metrics = _terminal_metrics(composed)
    metrics['rtf'] = timing['online_compute_rtf']
    metrics['rtf_available'] = True
    metrics['timing'] = timing
    evidence = {
        'raw': {'path': str(raw_path), 'sha256': file_sha256(raw_path)},
        'feeder': {'path': str(feeder_path), 'sha256': file_sha256(feeder_path)},
        'bound': {
            'path': str(bound_path), 'sha256': file_sha256(bound_path),
            'receipt_sha256': hashlib.sha256(
                _json_bytes({key: value for key, value in bound_receipt.items()
                             if key != 'document'})).hexdigest(),
        },
        'composed': {'path': str(composed_path), 'sha256': file_sha256(composed_path)},
        'timing': timing,
    }
    return composed, evidence, metrics


def run_formal(
        config: FormalConfig, *,
        identity_validator: Optional[Callable[[FormalConfig], Mapping[str, Any]]] = None,
        quiescence_runner: Optional[Callable[[FormalConfig, Path], int]] = None,
        runner_hook: Optional[Callable[[Sequence[str], FormalConfig], Optional[int]]] = None,
        runner: Optional[Callable[[Sequence[str], FormalConfig], Optional[int]]] = None,
        process_factory: Optional[Callable[..., Any]] = None,
        container_inspect: Optional[Callable[[str], Mapping[str, Any]]] = None,
        container_stats: Optional[Callable[[str], Mapping[str, Any]]] = None,
        container_top: Optional[Callable[[str], Mapping[str, Any]]] = None,
        container_stop: Optional[Callable[[str], Mapping[str, Any]]] = None,
        container_remove: Optional[Callable[[str], Mapping[str, Any]]] = None,
        binder: Optional[Callable[[Path, Path, Path], Mapping[str, Any]]] = None,
        compositor: Optional[Callable[[Path, Path, Path, Path], Mapping[str, Any]]] = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
) -> int:
    """Run one production lifecycle, or the legacy injected hook seam.

    The default path performs read-only identity probes, exactly one
    quiescence subprocess, one same-process ``Popen`` of the fixed argv, and
    one supervised container lifecycle.  There is no retry.  Every reserved
    root gets one immutable closure receipt and SHA sidecar.
    """
    marker: Optional[Mapping[str, Any]] = None
    identity: Optional[dict[str, Any]] = None
    closure_written = False
    root = config.root

    def close(kind: str, message: str, *, status: str = 'FAIL_CLOSED',
              extra: Optional[Mapping[str, Any]] = None,
              command_sha256: Optional[str] = None) -> None:
        nonlocal closure_written
        if closure_written:
            return
        _write_closure(
            root, kind=kind, message=message, status=status,
            identity=identity, extra=extra, command_sha256=command_sha256)
        closure_written = True

    try:
        marker = reserve_attempt_root(config)
    except LaunchError:
        return 11

    try:
        identity_check = identity_validator or validate_production_identity
        identity = dict(identity_check(config))
        atomic_json(root / 'identity_preflight.json', {
            'schema_version': SCHEMA_VERSION,
            'receipt_kind': 'm6a10_v10_formal_identity_preflight',
            'contract_id': CONTRACT_ID,
            'status': 'PASS',
            'identity': identity,
            'runner_start_attempted': False,
            'bag_opened': _identity_input_opened(identity),
            'input_opened': _identity_input_opened(identity),
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
        })
    except (LaunchError, OSError, TypeError, ValueError) as error:
        try:
            close('IDENTITY_PREFLIGHT_FAIL_CLOSED', str(error))
        except (LaunchError, OSError):
            pass
        return 11

    if runner_hook is not None and runner is not None:
        close('RUNNER_HOOK_AMBIGUOUS', 'provide only one injected runner hook')
        return 12

    quiescence_path = root / 'quiescence.json'
    quiescence_rc: Optional[int] = None
    try:
        # The selected callable is invoked exactly once.
        quiescence_rc = int((quiescence_runner or _run_quiescence)(config, quiescence_path))
        preflight, quiescence_sha = _load_pass_quiescence(quiescence_path)
        if quiescence_rc != 0:
            raise LaunchError(
                'PREFLIGHT_FAIL_CLOSED',
                'quiescence returned nonzero despite a PASS receipt')
    except (LaunchError, OSError, TypeError, ValueError) as error:
        try:
            close(
                'PREFLIGHT_FAIL_CLOSED', str(error),
                extra={'quiescence_returncode': quiescence_rc})
        except (LaunchError, OSError):
            pass
        return 10

    # Measure continuity immediately after the PASS receipt is read.  The
    # production Popen below is the next process-start operation.
    quiescence_completed_ns = time.monotonic_ns()

    injected_runner = runner_hook or runner
    try:
        output_dir = root / 'out'
        if output_dir.exists() or output_dir.is_symlink():
            raise LaunchError('OUTPUT_OVERWRITE', f'output directory already exists: {output_dir}')
        argv = build_safe_docker_argv(config, output_dir)
        try:
            output_dir.mkdir()
        except FileExistsError as error:
            raise LaunchError(
                'OUTPUT_OVERWRITE', f'output directory appeared during reservation: {output_dir}') from error
        command_sha = hashlib.sha256(_json_bytes({'argv': argv})).hexdigest()
        atomic_json(root / 'launch_attempt.json', {
            'schema_version': SCHEMA_VERSION,
            'contract_id': CONTRACT_ID,
            'status': 'START_ATTEMPTED',
            'command_sha256': command_sha,
            'quiescence_receipt_sha256': quiescence_sha,
            'quiescence': preflight,
            'quiescence_completed_monotonic_ns': quiescence_completed_ns,
            'maximum_preflight_to_run_gap_ns': MAX_PREFLIGHT_TO_RUN_GAP_NS,
            'runner_start_attempted': True,
            'bag_opened': False,
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
        })
        if injected_runner is not None:
            # Compatibility seam for unit tests and callers that deliberately
            # do not start Docker.  It remains fail-closed and has no retry.
            pre_popen_gap_ns = time.monotonic_ns() - quiescence_completed_ns
            if pre_popen_gap_ns > MAX_PREFLIGHT_TO_RUN_GAP_NS:
                raise LaunchError('PREFLIGHT_RUN_GAP_FAIL_CLOSED',
                                  'runner start gap exceeded before injected hook')
            result = injected_runner(tuple(argv), config)
            return_code = 0 if result is None else int(result)
            atomic_json(root / 'runner_hook_receipt.json', {
                'schema_version': SCHEMA_VERSION,
                'contract_id': CONTRACT_ID,
                'status': 'INJECTED_RUNNER_HOOK_RETURNED',
                'returncode': return_code,
                'docker_run_started': False,
                'bag_opened': False,
                'ground_truth_content_opened': False,
                'scorer_invoked': False,
            })
            close(
                'INJECTED_RUNNER_HOOK_RETURNED',
                f'injected runner returned {return_code}',
                command_sha256=command_sha,
                extra={
                    'runner_start_attempted': True,
                    'quiescence_receipt_sha256': quiescence_sha,
                    'process': {'returncode': return_code, 'injected': True},
                    'injected_result': {'status': 'not_composed', 'failure_kind': 'INJECTED_HOOK'},
                })
            return return_code

        preflight_check_ns = time.monotonic_ns()
        pre_popen_gap_ns = preflight_check_ns - quiescence_completed_ns
        if pre_popen_gap_ns > MAX_PREFLIGHT_TO_RUN_GAP_NS:
            raise LaunchError(
                'PREFLIGHT_RUN_GAP_FAIL_CLOSED',
                'runner start gap exceeded before Popen')
        stdout_path = root / 'runner.stdout.log'
        stderr_path = root / 'runner.stderr.log'
        stdout = stdout_path.open('x', encoding='utf-8')
        stderr = stderr_path.open('x', encoding='utf-8')
        factory = process_factory or subprocess.Popen
        process: Any = None
        runner_start_boundary_ns = time.monotonic_ns()
        boundary_gap_ns = runner_start_boundary_ns - quiescence_completed_ns
        if boundary_gap_ns > MAX_PREFLIGHT_TO_RUN_GAP_NS:
            stdout.close()
            stderr.close()
            raise LaunchError(
                'PREFLIGHT_RUN_GAP_FAIL_CLOSED',
                'runner start boundary exceeded before Popen')
        try:
            # Exactly one same-process Popen; no retry or shell.
            process = factory(
                argv, stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr,
                close_fds=True)
        except BaseException as error:
            stdout.close()
            stderr.close()
            close(
                'RUNNER_START_FAILURE', repr(error), command_sha256=command_sha,
                extra={
                    'runner_start_attempted': True,
                    'quiescence_receipt_sha256': quiescence_sha,
                    'process': {'started': False, 'error': repr(error)},
                })
            return 20
        invocation_gap_ns = time.monotonic_ns() - runner_start_boundary_ns
        container_name = _container_name(config, output_dir)
        inspect_fn = container_inspect or _docker_container_inspect
        stats_fn = container_stats or _docker_container_stats
        top_fn = container_top or _docker_container_top
        stop_fn = container_stop or _docker_container_stop
        remove_fn = container_remove or _docker_container_remove
        stop_state = {'requested': False}

        def stop_once(name: str) -> Mapping[str, Any]:
            if stop_state['requested']:
                return {'status': 'already_requested'}
            stop_state['requested'] = True
            return stop_fn(name)

        try:
            lifecycle_returncode, lifecycle = _supervise_container(
                process, container_name=container_name,
                watchdog_seconds=config.watchdog_seconds,
                stats_interval_seconds=config.stats_interval_seconds,
                inspect=inspect_fn,
                stats=stats_fn,
                top=top_fn,
                stop=stop_once,
                clock=clock, sleep=sleep)
        except BaseException as error:
            try:
                stdout.close()
                stderr.close()
            finally:
                diagnostics = _capture_container_diagnostics(
                    container_name, inspect_fn, stats_fn, top_fn)
                try:
                    stop_result = dict(stop_once(container_name))
                except BaseException as stop_error:
                    stop_result = {'status': 'stop_error', 'error': repr(stop_error)}
                try:
                    process.wait(timeout=CONTAINER_STOP_WAIT_SECONDS)
                except TypeError:
                    try:
                        process.wait()
                    except BaseException as wait_error:
                        stop_result = {**stop_result, 'wait_error': repr(wait_error)}
                except (subprocess.TimeoutExpired, OSError) as wait_error:
                    stop_result = {**stop_result, 'wait_error': repr(wait_error)}
                except BaseException as wait_error:
                    stop_result = {**stop_result, 'wait_error': repr(wait_error)}
                after_stop = _capture_container_diagnostics(
                    container_name, inspect_fn, stats_fn, top_fn)
                cleanup = _remove_verified_exited_container(
                    container_name, inspect_fn, remove_fn)
                close(
                    'CONTAINER_SUPERVISION_FAILURE', repr(error),
                    command_sha256=command_sha,
                    extra={'runner_start_attempted': True,
                           'quiescence_receipt_sha256': quiescence_sha,
                           'preflight_to_run_gap_ns': boundary_gap_ns,
                           'popen_invocation_gap_ns': invocation_gap_ns,
                           'process': {'pid': getattr(process, 'pid', None),
                                       'started': True, 'error': repr(error)},
                           'container': {
                               'diagnostics_before_stop': diagnostics,
                               'stop': stop_result,
                               'diagnostics_after_stop': after_stop,
                           },
                           'cleanup': cleanup})
            return 70
        stdout.close()
        stderr.close()
        process_summary = {
            'pid': getattr(process, 'pid', None),
            'returncode': lifecycle_returncode,
            'started': True,
            'preflight_to_run_gap_ns': boundary_gap_ns,
            'popen_invocation_gap_ns': invocation_gap_ns,
            'stdout_path': str(stdout_path),
            'stderr_path': str(stderr_path),
            'stdout_sha256': file_sha256(stdout_path) if stdout_path.is_file() else None,
            'stderr_sha256': file_sha256(stderr_path) if stderr_path.is_file() else None,
        }
        container_summary = dict(lifecycle)
        container_name = _container_name(config, output_dir)
        inspect_fn = container_inspect or _docker_container_inspect
        remove_fn = container_remove or _docker_container_remove
        cleanup: Optional[Mapping[str, Any]] = None
        evidence: Optional[Mapping[str, Any]] = None
        metrics: dict[str, Any] = {
            'expected_topic_counts': dict(EXPECTED_TOPIC_COUNTS),
            'published_topic_counts': None,
            'received_topic_counts': None,
            'acknowledged_topic_counts': None,
            'required_evaluation_end_timestamp_seconds':
                REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS,
            'rtf': None, 'rtf_available': False,
        }
        diagnostics = lifecycle.get('diagnostics_before_cleanup') or \
            lifecycle.get('diagnostics_after_stop')
        oom_killed = bool(
            isinstance(diagnostics, Mapping) and diagnostics.get('oom_killed') is True)
        if lifecycle.get('watchdog_triggered') is True:
            cleanup = _remove_verified_exited_container(container_name, inspect_fn, remove_fn)
            close(
                'CONTAINER_WATCHDOG_TIMEOUT', 'container watchdog timeout',
                command_sha256=command_sha,
                extra={
                    'runner_start_attempted': True,
                    'quiescence_receipt_sha256': quiescence_sha,
                    'preflight_to_run_gap_ns': boundary_gap_ns,
                    'process': process_summary, 'container': container_summary,
                    'cleanup': cleanup,
                })
            return 124
        if lifecycle_returncode != 0:
            cleanup = _remove_verified_exited_container(container_name, inspect_fn, remove_fn)
            kind = 'CONTAINER_OOM' if oom_killed else 'CONTAINER_PROCESS_FAILURE'
            close(
                kind, f'container returned {lifecycle_returncode}',
                command_sha256=command_sha,
                extra={
                    'runner_start_attempted': True,
                    'quiescence_receipt_sha256': quiescence_sha,
                    'preflight_to_run_gap_ns': boundary_gap_ns,
                    'process': process_summary, 'container': container_summary,
                    'cleanup': cleanup,
                })
            return 31 if not oom_killed else 32
        try:
            composed, evidence, metrics = _bind_and_compose(
                output_dir, config, binder or _default_binder,
                compositor or _default_compositor)
        except LaunchError as error:
            cleanup = _remove_verified_exited_container(container_name, inspect_fn, remove_fn)
            close(
                error.kind, str(error), command_sha256=command_sha,
                extra={
                    'runner_start_attempted': True,
                    'quiescence_receipt_sha256': quiescence_sha,
                    'preflight_to_run_gap_ns': boundary_gap_ns,
                    'process': process_summary, 'container': container_summary,
                    'cleanup': cleanup, 'evidence': evidence,
                })
            return 32
        cleanup = _remove_verified_exited_container(container_name, inspect_fn, remove_fn)
        if cleanup.get('status') not in ('removed', 'not_present'):
            close(
                'CLEANUP_FAIL_CLOSED',
                'container cleanup was not verified stopped-only removal',
                command_sha256=command_sha,
                extra={
                    'runner_start_attempted': True,
                    'quiescence_receipt_sha256': quiescence_sha,
                    'preflight_to_run_gap_ns': boundary_gap_ns,
                    'process': process_summary, 'container': container_summary,
                    'cleanup': cleanup, 'evidence': evidence, 'result': metrics,
                })
            return 33
        close(
            'NATURAL_PASS', 'validated schema-v3 terminal evidence', status='PASS',
            command_sha256=command_sha,
            extra={
                'runner_start_attempted': True,
                'quiescence_receipt_sha256': quiescence_sha,
                'preflight_to_run_gap_ns': boundary_gap_ns,
                'process': process_summary, 'container': container_summary,
                'cleanup': cleanup, 'evidence': evidence, 'result': metrics,
                'composed_status': 'pass',
            })
        return 0
    except (LaunchError, OSError, TypeError, ValueError) as error:
        try:
            close(error.kind if isinstance(error, LaunchError)
                  else 'RUNNER_LIFECYCLE_FAIL_CLOSED', str(error))
        except (LaunchError, OSError):
            pass
        return 12
    except BaseException as error:
        try:
            close('LAUNCHER_EXCEPTION', repr(error))
        except (LaunchError, OSError):
            pass
        return 70


# Explicit aliases for a future caller while keeping the v10 API independent.
run_v10 = run_formal
run_fast_livo2_m6a10_v10_formal = run_formal


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--repo-root', type=Path, default=ROOT)
    parser.add_argument('--bag', type=Path, default=BAG_PATH)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    config = FormalConfig(
        root=args.root, repo_root=args.repo_root, bag_path=args.bag)
    # No replay implementation is installed yet; this exits fail-closed after
    # reserving/recording the preflight state instead of invoking Docker.
    return run_formal(config)


if __name__ == '__main__':
    sys.exit(main())
