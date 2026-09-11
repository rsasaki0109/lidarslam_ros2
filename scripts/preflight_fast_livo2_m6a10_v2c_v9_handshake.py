#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""Input-free v9 callback-dispatch preflight for FAST-LIVO2.

The gate starts the built mapper with no host mounts and publishes exactly one
minimal, valid Ouster ``sensor_msgs/PointCloud2`` on the configured LiDAR
topic. It proves publisher return, one consumer callback, one ACK, duplicate
ACK rejection, topic isolation, and bounded callback latency. The synthetic
message is deliberately not a valid estimator frame (the mapper's IMU/image
prerequisites are not supplied), so this is a callback-dispatch proof only.
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
PROFILE_PATH = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v9_formal.yaml'
PROFILE_KEY = 'm6a10_fast_livo2_v2c_v9'
CONTRACT_ID = 'm6a10-v2c-fast-livo2-wallrate-v9'
HANDSHAKE_CONTRACT_ID = 'm6a10-v2c-fast-livo2-synthetic-lidar-handshake-v1'
RECEIPT_KIND = 'fast_livo2_m6a10_v9_synthetic_lidar_handshake_preflight'
VARIANT = 'v9'
BASE_PATCH_SHA256 = (
    '33f30a40ad54db5eea331a87b8b86f32410aac2980e37e4cb6d5027100095297')
V8_DELTA_PATCH_SHA256 = (
    'e4a3b2b9d981ec0f740365695f53a662797aaf9400e1d7186551687143c4fd87')
V9_DELTA_PATCH_SHA256 = (
    '1cf3067a634a34d5c3a2126f9f495fcb675068cb94edf562eb7e84b1e470146b')
IMAGE_TAG = 'm6a10-v2c-v9-wallrate-fast-livo2-benchmark:ros1-pinned'
LIDAR_TOPIC = '/os1_cloud_node1/points'
SERVICE_NAMES = {
    'status': '/m6a10/consumer_status',
    'ack': '/m6a10/consumer_ack',
    'eof': '/m6a10/consumer_eof',
}


class PreflightError(RuntimeError):
    """A fail-closed identity, handshake, or safety error."""


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
        phase = document['competitive_slam_profile'][PROFILE_KEY]
    except (KeyError, TypeError) as exc:
        raise PreflightError('v9 profile phase is missing') from exc
    if phase.get('contract_id') != CONTRACT_ID:
        raise PreflightError('v9 profile contract_id mismatch')
    if phase.get('status') != 'preregistered_not_built':
        raise PreflightError('v9 profile must remain preregistered_not_built')
    if phase.get('result') is not None:
        raise PreflightError('v9 profile result must remain null')
    source = phase.get('source') or {}
    if source.get('base_patch_sha256') != BASE_PATCH_SHA256:
        raise PreflightError('base patch binding mismatch')
    if source.get('v8_delta_patch_sha256') != V8_DELTA_PATCH_SHA256:
        raise PreflightError('v8 delta patch binding mismatch')
    if source.get('v9_delta_patch_sha256') != V9_DELTA_PATCH_SHA256:
        raise PreflightError('v9 delta patch binding mismatch')
    if source.get('patch_apply_order') != ['base_patch', 'v8_delta_patch', 'v9_delta_patch']:
        raise PreflightError('v9 patch apply order is not explicit')
    return phase


def handshake_script() -> str:
    """Return the bounded no-mount synthetic callback handshake."""
    return r'''set -euo pipefail
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1
export ROS_HOME=/tmp/m6a10_ros_home ROS_LOG_DIR=/tmp/m6a10_ros_logs
export M6A10_PHASE_CONTRACT_VERSION=m6a10-online-compute-v2
export M6A10_PHASE_MODE=unpaced_ack
export M6A10_CONSUMER_EVIDENCE=/tmp/m6a10_v9_consumer_evidence.json
export M6A10_FAST_EXPECTED_MESSAGES=1
export M6A10_FAST_EXPECTED_LIDAR_MESSAGES=1
export M6A10_FAST_EXPECTED_IMU_MESSAGES=0
export M6A10_FAST_EXPECTED_IMAGE_MESSAGES=0
export M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS=0.25
export M6A10_FAST_MAX_BACKLOG_MESSAGES=1
export M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS=-1
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
. /opt/ros/noetic/setup.bash
. /opt/fast_livo_ws/devel/setup.bash

roscore >/tmp/m6a10_v9_roscore.log 2>&1 &
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
  >/tmp/m6a10_v9_mapper.log 2>&1 &
MAPPER_PID=$!

for _ in $(seq 1 300); do
  listed=1
  for service in /m6a10/consumer_status /m6a10/consumer_eof /m6a10/consumer_ack; do
    rosservice list 2>/dev/null | grep -Fx "$service" >/dev/null || listed=0
  done
  if [[ "$listed" == 1 ]]; then break; fi
  if ! kill -0 "$MAPPER_PID" >/dev/null 2>&1; then exit 21; fi
  sleep 0.1
done
for service in /m6a10/consumer_status /m6a10/consumer_eof /m6a10/consumer_ack; do
  rosservice list 2>/dev/null | grep -Fx "$service" >/dev/null
done
''' + r'''
# The second half runs after the mapper services are ready.
python3 - <<'PY'
import json
import struct
import time

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

topic = '/os1_cloud_node1/points'
marker = '/tmp/m6a10_v9_publish_marker.json'
rospy.init_node('m6a10_v9_synthetic_lidar_publisher', anonymous=True,
                disable_signals=True)
publisher = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=False)
deadline = time.monotonic() + 5.0
while publisher.get_num_connections() < 1 and time.monotonic() < deadline:
    time.sleep(0.01)
if publisher.get_num_connections() < 1:
    raise RuntimeError('synthetic lidar publisher did not connect')

# These offsets match the EIGEN-aligned ouster_ros::Point registration in
# FAST-LIVO2's preprocess.h (x/y/z + padding, then intensity/t/etc.).
fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('intensity', 16, PointField.FLOAT32, 1),
    PointField('t', 20, PointField.UINT32, 1),
    PointField('reflectivity', 24, PointField.UINT16, 1),
    PointField('ring', 26, PointField.UINT8, 1),
    PointField('ambient', 28, PointField.UINT16, 1),
    PointField('range', 32, PointField.UINT32, 1),
]
payload = bytearray(36)
struct.pack_into('<f', payload, 0, 2.0)
struct.pack_into('<f', payload, 4, 0.0)
struct.pack_into('<f', payload, 8, 0.0)
struct.pack_into('<f', payload, 16, 1.0)
struct.pack_into('<I', payload, 20, 0)
struct.pack_into('<H', payload, 24, 1)
struct.pack_into('<B', payload, 26, 0)
struct.pack_into('<H', payload, 28, 0)
struct.pack_into('<I', payload, 32, 2000)
message = PointCloud2(
    header=Header(stamp=rospy.Time.from_sec(1.0), frame_id='os_sensor'),
    height=1, width=1, fields=fields, is_bigendian=False,
    point_step=36, row_step=36, data=bytes(payload), is_dense=True)
publisher.publish(message)
with open(marker, 'x', encoding='utf-8') as stream:
    json.dump({'publisher_returned': True, 'topic': topic, 'width': 1,
               'point_step': 36, 'field_names': [field.name for field in fields]},
              stream, sort_keys=True)
    stream.write('\n')
PY

# Poll only consumer-owned counters and use the EOF sidecar for the final
# bounded evidence.  No publisher connection count is treated as an ACK.
python3 - <<'PY'
import json
import subprocess
import time
import yaml

def call(name):
    completed = subprocess.run(['rosservice', 'call', name, '{}'], check=False,
                               capture_output=True, text=True, timeout=5)
    if completed.returncode != 0:
        raise RuntimeError('service failed: ' + name + ': ' + completed.stderr)
    return yaml.safe_load(completed.stdout) or {}

deadline = time.monotonic() + 5.0
status = {}
while time.monotonic() < deadline:
    response = call('/m6a10/consumer_status')
    try:
        status = json.loads(response['message'])
    except (KeyError, TypeError, json.JSONDecodeError):
        time.sleep(0.01)
        continue
    consumer = status.get('consumer') or {}
    counts = consumer.get('received_topic_counts') or {}
    if (consumer.get('received_messages') == 1 and counts ==
            {'lidar': 1, 'imu': 0, 'image': 0}):
        break
    time.sleep(0.01)
else:
    raise RuntimeError('synthetic callback was not observed before deadline')

ack = call('/m6a10/consumer_ack')
if ack.get('success') is not True:
    raise RuntimeError('ACK for accepted synthetic callback was rejected')
duplicate = call('/m6a10/consumer_ack')
if duplicate.get('success') is not False:
    raise RuntimeError('duplicate ACK was accepted')
eof = call('/m6a10/consumer_eof')
if eof.get('success') is not True:
    raise RuntimeError('synthetic EOF evidence write failed')
with open('/tmp/m6a10_v9_consumer_evidence.json.eof.json', encoding='utf-8') as stream:
    evidence = json.load(stream)
consumer = evidence.get('consumer') or {}
counts = consumer.get('received_topic_counts') or {}
if (consumer.get('received_messages') != 1 or
        consumer.get('acked_messages') != 1 or
        counts != {'lidar': 1, 'imu': 0, 'image': 0} or
        consumer.get('dropped_messages') != 0 or
        consumer.get('queue_overflow') != 0 or
        consumer.get('processing_failures') != 0 or
        float(consumer.get('maximum_callback_latency_seconds', 1.0)) > 0.25):
    raise RuntimeError('synthetic callback evidence is not exact and bounded')
with open('/tmp/m6a10_v9_publish_marker.json', encoding='utf-8') as stream:
    marker = json.load(stream)
print('M6A10_HANDSHAKE ' + json.dumps({
    'status': 'PASS',
    'contract_id': 'm6a10-v2c-fast-livo2-synthetic-lidar-handshake-v1',
    'services': {'status': True, 'ack': True, 'duplicate_ack_rejected': True,
                 'eof': True},
    'publisher_returned': marker.get('publisher_returned') is True,
    'published_topic': marker.get('topic'),
    'published_count': 1,
    'received_messages': consumer.get('received_messages'),
    'received_topic_counts': counts,
    'acknowledged_messages': consumer.get('acked_messages'),
    'dropped_messages': consumer.get('dropped_messages'),
    'queue_overflow': consumer.get('queue_overflow'),
    'processing_failures': consumer.get('processing_failures'),
    'maximum_callback_latency_seconds': consumer.get(
        'maximum_callback_latency_seconds'),
    'estimator_claim': False,
    'estimator_prerequisites_supplied': False,
    'finalize_not_attempted_due_to_uninitialized_estimator_buffers': True,
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
    if any(value in command for value in ('-v', '--volume', '--mount')):
        raise PreflightError('handshake command must not mount host paths')
    if command[command.index('--network') + 1] != 'none':
        raise PreflightError('handshake command must use network none')
    if '--read-only' not in command:
        raise PreflightError('handshake command must use a read-only rootfs')
    return command


def inspect_image(
        image: str,
        runner: Callable[..., subprocess.CompletedProcess[str]] = _run,
) -> dict[str, Any]:
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
        'benchmark.fast_livo2.m6a10_v9_delta_patch_sha256': V9_DELTA_PATCH_SHA256,
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
        raise PreflightError('handshake result is not PASS for the v9 contract')
    services = result.get('services')
    if not isinstance(services, Mapping) or not all(services.values()):
        raise PreflightError('one or more synthetic service checks failed')
    if result.get('publisher_returned') is not True or result.get('published_topic') != LIDAR_TOPIC:
        raise PreflightError('publisher return/topic proof is missing')
    if result.get('published_count') != 1 or result.get('received_messages') != 1:
        raise PreflightError('synthetic total count is not exactly one')
    if result.get('received_topic_counts') != {'lidar': 1, 'imu': 0, 'image': 0}:
        raise PreflightError('synthetic cross-topic counts are not isolated')
    if result.get('acknowledged_messages') != 1:
        raise PreflightError('synthetic ACK count is not exactly one')
    if result.get('dropped_messages') != 0 or result.get('queue_overflow') != 0:
        raise PreflightError('synthetic callback reported a drop or overflow')
    if result.get('processing_failures') != 0:
        raise PreflightError('synthetic callback reported a processing failure')
    if float(result.get('maximum_callback_latency_seconds', 1.0)) > 0.25:
        raise PreflightError('synthetic callback exceeded the 250 ms bound')
    if result.get('estimator_claim') is not False or result.get(
            'estimator_prerequisites_supplied') is not False:
        raise PreflightError('synthetic preflight made an estimator claim')
    for field in ('input_mount_performed', 'ground_truth_content_opened', 'scorer_invoked'):
        if result.get(field) is not False:
            raise PreflightError(f'handshake safety field is not false: {field}')
    return result


def run_handshake(
        image: str, *, timeout: int = 120,
        runner: Callable[..., subprocess.CompletedProcess[str]] = _run,
) -> dict[str, Any]:
    command = handshake_command(image)
    completed = runner(command, timeout=timeout)
    if completed.returncode != 0:
        raise PreflightError(
            'live synthetic callback handshake failed: ' + completed.stderr.strip())
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
            'v9_delta_patch_sha256': V9_DELTA_PATCH_SHA256,
            'image': dict(image),
            'profile_key': PROFILE_KEY,
            'profile_path': str(PROFILE_PATH),
            'preflight_script': {
                'path': str(Path(__file__).resolve().relative_to(ROOT)),
                'sha256': file_sha256(Path(__file__).resolve()),
            },
        },
        'handshake': dict(handshake),
        'synthetic_scope': {
            'topic': LIDAR_TOPIC,
            'message_type': 'sensor_msgs/PointCloud2',
            'point_layout': 'ouster_ros::Point fields x y z intensity t reflectivity ring ambient range',
            'published_messages': 1,
            'imu_messages': 0,
            'image_messages': 0,
            'estimator_claim': False,
            'limitation': 'callback dispatch only; no IMU/image prerequisites or estimator result',
        },
        'execution_mount_plan': {
            'network': 'none', 'rootfs': 'read_only',
            'input_mount_performed': False, 'observed_host_mounts': [],
            'ground_truth_mount_forbidden': True, 'scorer_mount_forbidden': True,
        },
        'safety': {
            'bag_replay_started': False, 'ground_truth_content_opened': False,
            'scorer_invoked': False, 'input_content_read': False,
        },
        'runtime_result': None,
        'created_at': dt.datetime.now(dt.timezone.utc).isoformat(),
        'profile_contract_snapshot': {
            'profile_key': PROFILE_KEY,
            'contract_id': phase['contract_id'], 'status': phase['status'],
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
    parser.add_argument('--profile', type=Path, default=PROFILE_PATH)
    args = parser.parse_args(argv)
    try:
        phase = load_profile(args.profile)
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
            'safety': {'bag_replay_started': False,
                       'ground_truth_content_opened': False,
                       'scorer_invoked': False, 'input_content_read': False},
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
