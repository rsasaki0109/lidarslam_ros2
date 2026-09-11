#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""Live, input-free service-handshake preflight for FAST-LIVO2 M6a10 v8.

This gate starts the built v8 mapper with no bag, no host source mount, and no
ground-truth/scorer mount.  It proves that the four M6a10 Trigger services
respond while the mapper is idle under simulated time, including the
no-callback ACK rejection and the zero-message EOF/finalize path.  It is a
build/identity gate only; it does not replay a bag or measure performance.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Callable, Mapping, Optional, Sequence

import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v8_formal.yaml'
CONTRACT_ID = 'm6a10-v2c-fast-livo2-service-queue-v8'
HANDSHAKE_CONTRACT_ID = 'm6a10-v2c-fast-livo2-service-queue-handshake-v1'
RECEIPT_KIND = 'fast_livo2_m6a10_v8_live_service_handshake_preflight'
VARIANT = 'v8'
BASE_PATCH_SHA256 = (
    '33f30a40ad54db5eea331a87b8b86f32410aac2980e37e4cb6d5027100095297')
V8_DELTA_PATCH_SHA256 = (
    'e4a3b2b9d981ec0f740365695f53a662797aaf9400e1d7186551687143c4fd87')
IMAGE_TAG = 'm6a10-v2c-v8-service-queue-fast-livo2-benchmark:ros1-pinned'
SERVICE_NAMES = {
    'status': '/m6a10/consumer_status',
    'eof': '/m6a10/consumer_eof',
    'finalize': '/m6a10/consumer_finalize',
    'ack': '/m6a10/consumer_ack',
}


class PreflightError(RuntimeError):
    """A fail-closed handshake or identity error."""


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _run(command: Sequence[str], *, timeout: int = 120) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            list(command), check=False, capture_output=True, text=True,
            timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise PreflightError(f'command failed or timed out: {command[0]}') from exc


def load_profile(path: Path = PROFILE_PATH) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    try:
        phase = document['competitive_slam_profile']['m6a10_fast_livo2_v2c_v8']
    except (KeyError, TypeError) as exc:
        raise PreflightError('v8 profile phase is missing') from exc
    if phase.get('contract_id') != CONTRACT_ID:
        raise PreflightError('v8 profile contract_id mismatch')
    if phase.get('status') != 'preregistered_not_built':
        raise PreflightError('v8 profile must remain preregistered_not_built')
    if phase.get('result') is not None:
        raise PreflightError('v8 profile result must remain null')
    source = phase.get('source') or {}
    if source.get('base_patch_sha256') != BASE_PATCH_SHA256:
        raise PreflightError('v7 base patch binding mismatch')
    if source.get('v8_delta_patch_sha256') != V8_DELTA_PATCH_SHA256:
        raise PreflightError('v8 delta patch binding mismatch')
    return phase


def handshake_script() -> str:
    """Return the bounded in-container idle handshake script.

    It intentionally contains no input path and the command generated below
    contains no Docker mount.  ``use_sim_time`` is enabled before launch so a
    mapper without the v8 service spinner reproduces the v7 idle condition.
    """
    return r'''set -euo pipefail
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1
export ROS_HOME=/tmp/m6a10_ros_home ROS_LOG_DIR=/tmp/m6a10_ros_logs
export M6A10_PHASE_CONTRACT_VERSION=m6a10-online-compute-v2
export M6A10_PHASE_MODE=unpaced_ack
export M6A10_CONSUMER_EVIDENCE=/tmp/m6a10_consumer_evidence.json
export M6A10_FAST_EXPECTED_MESSAGES=0
export M6A10_FAST_EXPECTED_LIDAR_MESSAGES=0
export M6A10_FAST_EXPECTED_IMU_MESSAGES=0
export M6A10_FAST_EXPECTED_IMAGE_MESSAGES=0
export M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS=0.25
export M6A10_FAST_MAX_BACKLOG_MESSAGES=1
export M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS=-1
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
. /opt/ros/noetic/setup.bash
. /opt/fast_livo_ws/devel/setup.bash

roscore >/tmp/m6a10_roscore.log 2>&1 &
ROSCORE_PID=$!
MAPPER_PID=''
cleanup() {
  status=$?
  set +e
  if [[ -n "$MAPPER_PID" ]]; then kill -TERM "$MAPPER_PID" >/dev/null 2>&1 || true; fi
  kill -TERM "$ROSCORE_PID" >/dev/null 2>&1 || true
  if [[ -n "$MAPPER_PID" ]]; then wait "$MAPPER_PID" >/dev/null 2>&1 || true; fi
  wait "$ROSCORE_PID" >/dev/null 2>&1 || true
  exit "$status"
}
trap cleanup EXIT INT TERM

for _ in $(seq 1 100); do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.1
done
rosparam list >/dev/null 2>&1
rosparam set use_sim_time true
roslaunch fast_livo mapping_ouster_ntu.launch rviz:=false \
  >/tmp/m6a10_mapper.log 2>&1 &
MAPPER_PID=$!

for _ in $(seq 1 300); do
  listed=1
  for service in /m6a10/consumer_status /m6a10/consumer_eof \
      /m6a10/consumer_finalize /m6a10/consumer_ack; do
    rosservice list 2>/dev/null | grep -Fx "$service" >/dev/null || listed=0
  done
  if [[ "$listed" == 1 ]]; then break; fi
  if ! kill -0 "$MAPPER_PID" >/dev/null 2>&1; then exit 21; fi
  sleep 0.1
done
for service in /m6a10/consumer_status /m6a10/consumer_eof \
    /m6a10/consumer_finalize /m6a10/consumer_ack; do
  rosservice list 2>/dev/null | grep -Fx "$service" >/dev/null
done

status_response=$(timeout 5 rosservice call /m6a10/consumer_status "{}")
printf '%s\n' "$status_response" | grep -F 'success: True' >/dev/null
ack_response=$(timeout 5 rosservice call /m6a10/consumer_ack "{}")
printf '%s\n' "$ack_response" | grep -F 'success: False' >/dev/null
eof_response=$(timeout 5 rosservice call /m6a10/consumer_eof "{}")
printf '%s\n' "$eof_response" | grep -F 'success: True' >/dev/null
test -s /tmp/m6a10_consumer_evidence.json.eof.json
finalize_response=$(timeout 5 rosservice call /m6a10/consumer_finalize "{}")
printf '%s\n' "$finalize_response" | grep -F 'success: True' >/dev/null
test -s /tmp/m6a10_consumer_evidence.json

python3 - "$status_response" "$ack_response" "$eof_response" "$finalize_response" <<'PY'
import json
import sys

print('M6A10_HANDSHAKE ' + json.dumps({
    'status': 'PASS',
    'contract_id': 'm6a10-v2c-fast-livo2-service-queue-handshake-v1',
    'services': {
        'status': 'success: True' in sys.argv[1],
        'ack_rejected_without_callback': 'success: False' in sys.argv[2],
        'eof': 'success: True' in sys.argv[3],
        'finalize': 'success: True' in sys.argv[4],
    },
    'received_messages': 0,
    'acknowledged_messages': 0,
    'input_mount_performed': False,
    'ground_truth_content_opened': False,
    'scorer_invoked': False,
}, sort_keys=True, separators=(',', ':')))
PY
'''


def handshake_command(image: str) -> list[str]:
    command = [
        'docker', 'run', '--pull=never', '--rm', '--init', '--network', 'none',
        '--read-only', '--tmpfs', '/tmp:rw,noexec,nosuid,size=64m',
        '--tmpfs', '/root/.ros:rw,noexec,nosuid,size=32m', '--entrypoint',
        '/bin/bash', image, '-lc', handshake_script(),
    ]
    if any(token in command for token in ('-v', '--volume', '--mount')):
        raise PreflightError('handshake command must not mount host paths')
    if '--network' not in command or command[command.index('--network') + 1] != 'none':
        raise PreflightError('handshake command must use network none')
    if '--read-only' not in command:
        raise PreflightError('handshake command must use a read-only rootfs')
    return command


def inspect_image(image: str, runner: Callable[..., subprocess.CompletedProcess[str]] = _run) -> dict[str, Any]:
    completed = runner(['docker', 'image', 'inspect', image], timeout=30)
    if completed.returncode != 0:
        raise PreflightError(completed.stderr.strip() or 'docker image inspect failed')
    try:
        documents = json.loads(completed.stdout)
        document = documents[0] if isinstance(documents, list) else documents
    except (IndexError, TypeError, json.JSONDecodeError) as exc:
        raise PreflightError('docker image inspect JSON is invalid') from exc
    labels = document.get('Config', {}).get('Labels') or {}
    required = {
        'benchmark.fast_livo2.m6a10_variant': VARIANT,
        'benchmark.fast_livo2.m6a10_patch_sha256': BASE_PATCH_SHA256,
        'benchmark.fast_livo2.m6a10_base_patch_sha256': BASE_PATCH_SHA256,
        'benchmark.fast_livo2.m6a10_v8_delta_patch_sha256': V8_DELTA_PATCH_SHA256,
        'benchmark.fast_livo2.m6a10_consumer_contract': CONTRACT_ID,
        'benchmark.fast_livo2.m6a10_service_handshake_contract': HANDSHAKE_CONTRACT_ID,
    }
    for key, expected in required.items():
        if labels.get(key) != expected:
            raise PreflightError(f'image label mismatch: {key}')
    return {
        'id': document.get('Id'),
        'repo_digests': document.get('RepoDigests', []),
        'labels': labels,
        'inspect_sha256': hashlib.sha256(completed.stdout.encode()).hexdigest(),
    }


def parse_handshake(stdout: str) -> dict[str, Any]:
    prefix = 'M6A10_HANDSHAKE '
    records = [json.loads(line[len(prefix):]) for line in stdout.splitlines()
               if line.startswith(prefix)]
    if len(records) != 1:
        raise PreflightError('handshake did not emit exactly one result record')
    result = records[0]
    if result.get('status') != 'PASS' or result.get('contract_id') != HANDSHAKE_CONTRACT_ID:
        raise PreflightError('handshake result is not PASS for the v8 contract')
    services = result.get('services')
    if not isinstance(services, Mapping) or not all(services.values()):
        raise PreflightError('one or more live service checks failed')
    if result.get('received_messages') != 0 or result.get('acknowledged_messages') != 0:
        raise PreflightError('input-free handshake observed non-zero messages')
    for field in ('input_mount_performed', 'ground_truth_content_opened', 'scorer_invoked'):
        if result.get(field) is not False:
            raise PreflightError(f'handshake safety field is not false: {field}')
    return result


def run_handshake(image: str, *, timeout: int = 120,
                  runner: Callable[..., subprocess.CompletedProcess[str]] = _run) -> dict[str, Any]:
    command = handshake_command(image)
    completed = runner(command, timeout=timeout)
    if completed.returncode != 0:
        raise PreflightError(
            'live no-input service handshake failed: ' + completed.stderr.strip())
    return {
        'command': command,
        'exit_status': completed.returncode,
        'result': parse_handshake(completed.stdout),
        'stdout_sha256': hashlib.sha256(completed.stdout.encode()).hexdigest(),
        'stderr_sha256': hashlib.sha256(completed.stderr.encode()).hexdigest(),
    }


def build_receipt(phase: Mapping[str, Any], image: Mapping[str, Any],
                  handshake: Mapping[str, Any]) -> dict[str, Any]:
    return {
        'schema_version': 1,
        'receipt_kind': RECEIPT_KIND,
        'contract_id': CONTRACT_ID,
        'status': 'PASS',
        'identity': {
            'variant': VARIANT,
            'base_patch_sha256': BASE_PATCH_SHA256,
            'v8_delta_patch_sha256': V8_DELTA_PATCH_SHA256,
            'image': dict(image),
            'profile_path': str(PROFILE_PATH),
            'preflight_script': {
                'path': str(Path(__file__).resolve().relative_to(ROOT)),
                'sha256': file_sha256(Path(__file__).resolve()),
            },
        },
        'handshake': dict(handshake),
        'execution_mount_plan': {
            'network': 'none',
            'rootfs': 'read_only',
            'input_mount_performed': False,
            'observed_host_mounts': [],
            'ground_truth_mount_forbidden': True,
            'scorer_mount_forbidden': True,
        },
        'safety': {
            'bag_replay_started': False,
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
            'input_content_read': False,
        },
        'runtime_result': None,
        'created_at': dt.datetime.now(dt.timezone.utc).isoformat(),
        'profile_contract_snapshot': {
            'contract_id': phase['contract_id'],
            'status': phase['status'],
            'result': phase['result'],
        },
    }


def atomic_write(path: Path, value: Mapping[str, Any]) -> str:
    if path.exists() or path.is_symlink():
        raise PreflightError(f'refusing to overwrite existing receipt: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    staging = path.with_name(path.name + '.part')
    if staging.exists() or staging.is_symlink():
        raise PreflightError(f'stale receipt staging file exists: {staging}')
    payload = (json.dumps(value, indent=2, sort_keys=True) + '\n').encode()
    with staging.open('xb') as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(staging, path)
    return hashlib.sha256(payload).hexdigest()


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', default=IMAGE_TAG)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=int, default=120)
    args = parser.parse_args(argv)
    try:
        phase = load_profile()
        image = inspect_image(args.image)
        handshake = run_handshake(args.image, timeout=args.timeout)
        receipt = build_receipt(phase, image, handshake)
        receipt_sha = atomic_write(args.output, receipt)
        print(json.dumps({'status': 'PASS', 'path': str(args.output),
                          'sha256': receipt_sha, 'image_id': image['id']},
                         sort_keys=True))
        return 0
    except Exception as exc:
        failure = {
            'schema_version': 1, 'receipt_kind': RECEIPT_KIND,
            'contract_id': CONTRACT_ID, 'status': 'INVALID',
            'failure': {'type': type(exc).__name__, 'message': str(exc)},
            'safety': {
                'bag_replay_started': False,
                'ground_truth_content_opened': False,
                'scorer_invoked': False,
                'input_content_read': False,
            },
        }
        try:
            receipt_sha = atomic_write(args.output, failure)
        except Exception:
            receipt_sha = None
        print(json.dumps({'status': 'INVALID', 'path': str(args.output),
                          'sha256': receipt_sha, 'error': str(exc)},
                         sort_keys=True), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
