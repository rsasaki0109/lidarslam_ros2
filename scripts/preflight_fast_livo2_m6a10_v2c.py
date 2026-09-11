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

"""Read-only FAST-LIVO2 v2c execution-identity preflight.

The preflight deliberately does not mount the benchmark bag.  It verifies the
immutable image, host-owned source/runner bindings, and installed image
markers in a network-isolated, read-only container.  A successful receipt is
an identity gate, not a replay or performance observation.
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
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
CONTRACT_ID = 'm6a10-v2c-fast-livo2-single-inflight-v1'
RECEIPT_KIND = 'fast_livo2_m6a10_v2c_execution_identity_preflight'
IMAGE_TAG = 'm6a10-v2c-20260823-fast-livo2-fixed2:competitive-v1'
IMAGE_ID = (
    'sha256:89f6baeca8bc3691ed8465d0e19fd962cce708053cd28c8a4eebabb5b841f0ea')
EXPECTED_SOURCE_REVISION = '0d2c0346107b75b59934975adec9a6eeeb913c64'
EXPECTED_PATCH_SHA = (
    '33f30a40ad54db5eea331a87b8b86f32410aac2980e37e4cb6d5027100095297')
EXPECTED_FEEDER_SHA = (
    'a15d41381320dc237db2c9ab704b610f0079b51a39a808347e84d3a10928b248')
EXPECTED_RUNNER_SHA = (
    'add465a330b159f0da594c24178686095475b950b53731464da8cc2668a9ec19')
EXPECTED_WRAPPER_SHA = (
    'f4236ec79659becacb12dc5f76c2d7ef41bc83be7616df88c1ee6ebc60e65d20')
EXPECTED_RECIPE_SHA = (
    '3518159755396232c100a6cac71865ea79de81f45061239aa23883d1234fc257')
EXPECTED_BUILD_ENTRYPOINT_SHA = (
    '20fa4e760efe38091fae7bb7503b72545b17ad9cc0e572ced68da47753d57937')
EXPECTED_BUILD_RECEIPT_SHA = (
    'db29eab240852d6f7fd926f51aefc66547c72fb0e8a1193b978e3d34b65c8926')
EXPECTED_CONFIG_SHA = (
    'c8f94f130e599b928c3f02c3f3d3b2009ae01df76aec32f6ac96b6a987311ef3')
EXPECTED_TOPICS = {
    'lidar': '/os1_cloud_node1/points',
    'imu': '/imu/imu',
    'image': '/left/image_raw',
}
EXPECTED_COUNTS = {'lidar': 5793, 'imu': 225102, 'image': 5792}


class PreflightError(RuntimeError):
    """A fail-closed identity error."""


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def run(command: list[str], *, timeout: int = 120) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            command, check=False, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise PreflightError(
            f'command failed to start or timed out: {command[0]}') from exc


def load_phase() -> dict[str, Any]:
    document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    phase = document['competitive_slam_profile']['m6a10_fast_livo2_v2c']
    if phase.get('contract_id') != CONTRACT_ID:
        raise PreflightError('profile contract_id mismatch')
    if phase.get('status') != 'build_passed_not_executed':
        raise PreflightError('profile status is not build_passed_not_executed')
    if phase.get('result') is not None:
        raise PreflightError('profile result must remain null before replay')
    return phase


def verify_host_bindings(phase: dict[str, Any]) -> dict[str, Any]:
    paths = {
        'patch': ROOT / phase['source']['patch_path'],
        'feeder': ROOT / phase['feeder']['path'],
        'runner': ROOT / phase['runner']['path'],
        'wrapper': ROOT / phase['runner']['phase_entrypoint_path'],
        'recipe': ROOT / phase['execution']['recipe']['path'],
        'build_entrypoint': ROOT / phase['execution']['build_entrypoint']['path'],
    }
    expected = {
        'patch': EXPECTED_PATCH_SHA,
        'feeder': EXPECTED_FEEDER_SHA,
        'runner': EXPECTED_RUNNER_SHA,
        'wrapper': EXPECTED_WRAPPER_SHA,
        'recipe': EXPECTED_RECIPE_SHA,
        'build_entrypoint': EXPECTED_BUILD_ENTRYPOINT_SHA,
    }
    observed = {}
    for name, path in paths.items():
        if not path.is_file() or path.is_symlink():
            raise PreflightError(f'host binding is not a regular file: {path}')
        value = file_sha256(path)
        observed[name] = {
            'path': str(path.relative_to(ROOT)), 'sha256': value}
        if value != expected[name]:
            raise PreflightError(f'{name} SHA mismatch: {value}')

    receipt_path = Path(phase['execution']['build_receipt_path'])
    if file_sha256(receipt_path) != EXPECTED_BUILD_RECEIPT_SHA:
        raise PreflightError('authoritative build receipt SHA mismatch')
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    if receipt.get('status') != 'PASS':
        raise PreflightError('authoritative build receipt is not PASS')
    if receipt.get('image', {}).get('id') != IMAGE_ID:
        raise PreflightError('authoritative build receipt image ID mismatch')
    if receipt.get('source', {}).get('revision') != EXPECTED_SOURCE_REVISION:
        raise PreflightError('authoritative source revision mismatch')
    if receipt.get('source', {}).get('patch_sha256') != EXPECTED_PATCH_SHA:
        raise PreflightError('authoritative patch SHA mismatch')
    if receipt.get('source', {}).get('config_sha256') != EXPECTED_CONFIG_SHA:
        raise PreflightError('authoritative config SHA mismatch')
    return {
        'files': observed,
        'build_receipt': {
            'path': str(receipt_path), 'sha256': EXPECTED_BUILD_RECEIPT_SHA,
            'status': receipt['status'],
        },
    }


def inspect_image(phase: dict[str, Any]) -> dict[str, Any]:
    completed = run(['docker', 'image', 'inspect', IMAGE_TAG])
    if completed.returncode != 0:
        raise PreflightError(
            f'docker image inspect failed: {completed.stderr.strip()}')
    try:
        document = json.loads(completed.stdout)
        image = (document[0] if isinstance(document, list)
                 and len(document) == 1 else document)
    except json.JSONDecodeError as exc:
        raise PreflightError('docker image inspect did not return JSON') from exc
    if image.get('Id') != IMAGE_ID:
        raise PreflightError(f'image ID mismatch: {image.get("Id")}')
    labels = image.get('Config', {}).get('Labels') or {}
    required = {
        'benchmark.fast_livo2.revision': EXPECTED_SOURCE_REVISION,
        'benchmark.fast_livo2.m6a10_patch_sha256': EXPECTED_PATCH_SHA,
        'benchmark.fast_livo2.m6a10_feeder_sha256': EXPECTED_FEEDER_SHA,
        'benchmark.fast_livo2.m6a10_consumer_contract': CONTRACT_ID,
        'benchmark.fast_livo2.m6a10_consumer_capability':
            'callback_acceptance_eof_service_v1',
        'benchmark.fast_livo2.m6a10_queue_overflow_capability':
            'single_inflight_exact_count_observed',
    }
    for key, expected in required.items():
        if labels.get(key) != expected:
            raise PreflightError(f'image label mismatch: {key}')
    expected_inspect_sha = phase['execution']['image_inspect_sha256']
    inspect_sha = hashlib.sha256(completed.stdout.encode()).hexdigest()
    if inspect_sha != expected_inspect_sha:
        raise PreflightError('image inspect SHA differs from profile')
    labels_sha = hashlib.sha256(json.dumps(
        labels, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return {
        'id': image['Id'], 'repo_digests': image.get('RepoDigests', []),
        'size_bytes': image.get('Size'), 'labels': labels,
        'inspect_sha256': inspect_sha, 'labels_sha256': labels_sha,
    }


def probe_command(image: str) -> list[str]:
    probe = r"""set -eu
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_HOSTNAME=127.0.0.1 ROS_IP=127.0.0.1
. /opt/ros/noetic/setup.bash
src=/opt/fast_livo_ws/src/FAST-LIVO2
cfg="$src/config/NTU_VIRAL.yaml"
bin=/opt/fast_livo_ws/devel/lib/fast_livo/fastlivo_mapping
header="$src/include/m6a10_consumer_evidence.h"
test -d "$src"; test -f "$cfg"; test -f "$header"; test -x "$bin"
rev=$(git -C "$src" rev-parse HEAD)
config_sha=$(sha256sum "$cfg" | awk '{print $1}')
header_sha=$(sha256sum "$header" | awk '{print $1}')
test "$rev" = '0d2c0346107b75b59934975adec9a6eeeb913c64'
test "$config_sha" = 'c8f94f130e599b928c3f02c3f3d3b2009ae01df76aec32f6ac96b6a987311ef3'
grep -q 'img_topic: "/left/image_raw"' "$cfg"
grep -q 'lid_topic: "/os1_cloud_node1/points"' "$cfg"
grep -q 'imu_topic: "/imu/imu"' "$cfg"
grep -q 'pcd_save_en: false' "$cfg"
grep -q \
  'm6a10_queue_size = m6a10_consumer_evidence.enabled() ? 1U : 200000U' \
  "$src/src/LIVMapper.cpp"
grep -q 'm6a10_ack_service' "$src/src/LIVMapper.cpp"
grep -q 'm6a10_eof_service' "$src/src/LIVMapper.cpp"
grep -q 'm6a10_finalize_service' "$src/src/LIVMapper.cpp"
grep -q 'if (!m6a10_consumer_evidence.enabled()) savePCD();' "$src/src/LIVMapper.cpp"
! ldd "$bin" 2>&1 | grep -q 'not found'
command -v roscore >/dev/null
command -v rosbag >/dev/null
    command -v rospack >/dev/null
    python3 -c 'import rosbag, rospy, std_srvs.srv'
    compiler=$(g++ --version | head -1)
    linker=$(ld --version | head -1)
    ros_distro=$(rosversion -d)
    pcl=$(dpkg-query -W -f='${Version}' libpcl-dev)
    eigen=$(dpkg-query -W -f='${Version}' libeigen3-dev)
    openmp=$(dpkg-query -W -f='${Version}' libomp-dev)
    python3 - "$rev" "$(git -C "$src" status --porcelain)" \
      "$config_sha" "$(stat -c %s "$cfg")" "$header_sha" \
      "$compiler" "$linker" "$ros_distro" "$pcl" "$eigen" "$openmp" <<'PY'
import json
import sys

print(json.dumps({
    'source_revision': sys.argv[1],
    'worktree_status': sys.argv[2],
    'config_sha256': sys.argv[3],
    'config_bytes': int(sys.argv[4]),
    'consumer_header_sha256': sys.argv[5],
    'binary_present': True,
    'ldd_missing': False,
    'config_topics': {
        'lidar': '/os1_cloud_node1/points',
        'imu': '/imu/imu',
        'image': '/left/image_raw',
    },
    'config_map_save_enabled': False,
    'markers': {
        'ack_service': True,
        'eof_service': True,
        'finalize_service': True,
        'single_inflight_queue': True,
        'no_map_guard': True,
    },
    'ros_prerequisites': {
        'roscore': True,
        'rosbag': True,
        'rospack': True,
        'python_imports': True,
    },
    'toolchain': {
        'compiler': sys.argv[6],
        'linker': sys.argv[7],
        'ros_distro': sys.argv[8],
        'pcl': sys.argv[9],
        'eigen': sys.argv[10],
        'openmp': sys.argv[11],
    },
}, sort_keys=True, separators=(',', ':')))
PY
"""
    return [
        'docker', 'run', '--pull=never', '--rm', '--init', '--network', 'none',
        '--read-only', '--tmpfs', '/tmp:rw,noexec,nosuid,size=32m',
        '--tmpfs', '/root/.ros:rw,noexec,nosuid,size=32m', '--entrypoint',
        '/bin/bash', image, '-lc', probe]


def run_image_probe(image: dict[str, Any]) -> dict[str, Any]:
    command = probe_command(image['id'])
    completed = run(command, timeout=180)
    if completed.returncode != 0:
        raise PreflightError(
            'network-none read-only image probe failed: '
            + completed.stderr.strip())
    try:
        result = json.loads(completed.stdout.strip())
    except json.JSONDecodeError as exc:
        raise PreflightError('image probe did not return JSON') from exc
    if result.get('source_revision') != EXPECTED_SOURCE_REVISION:
        raise PreflightError('probe source revision mismatch')
    if result.get('config_sha256') != EXPECTED_CONFIG_SHA:
        raise PreflightError('probe config SHA mismatch')
    if result.get('config_topics') != EXPECTED_TOPICS:
        raise PreflightError('probe config topics mismatch')
    if not result.get('binary_present') or result.get('ldd_missing'):
        raise PreflightError('probe binary/linker contract failed')
    if result.get('config_map_save_enabled') is not False:
        raise PreflightError('probe map-save configuration is enabled')
    if not all(result.get('markers', {}).values()):
        raise PreflightError('probe installed marker contract incomplete')
    if not all(result.get('ros_prerequisites', {}).values()):
        raise PreflightError('probe ROS prerequisite contract incomplete')
    if not all(result.get('toolchain', {}).values()):
        raise PreflightError('probe toolchain contract incomplete')
    return {
        'command': command, 'exit_status': completed.returncode,
        'result': result,
        'stdout_sha256': hashlib.sha256(
            completed.stdout.encode()).hexdigest(),
        'stderr_sha256': hashlib.sha256(
            completed.stderr.encode()).hexdigest(),
    }


def build_receipt(phase: dict[str, Any], *, host: dict[str, Any],
                  image: dict[str, Any], probe: dict[str, Any]) -> dict[str, Any]:
    input_contract = phase['input']
    return {
        'schema_version': 1,
        'receipt_kind': RECEIPT_KIND,
        'contract_id': CONTRACT_ID,
        'status': 'PASS',
        'identity': {
            'source_revision': EXPECTED_SOURCE_REVISION,
            'patch_sha256': EXPECTED_PATCH_SHA,
            'image_tag': IMAGE_TAG,
            'image_id': IMAGE_ID,
            'authoritative_build_receipt': host['build_receipt'],
            'host_bindings': host['files'],
            'image_inspect_sha256': image['inspect_sha256'],
            'image_labels_sha256': image['labels_sha256'],
            'installed_probe': probe,
            'preflight_script': {
                'path': 'scripts/preflight_fast_livo2_m6a10_v2c.py',
                'sha256': file_sha256(Path(__file__).resolve()),
            },
        },
        'execution_mount_plan': {
            'network': 'none', 'rootfs': 'read_only',
            'raw_input_source': input_contract['path'],
            'raw_input_target': '/input/raw_input.bag',
            'raw_input_mode': 'ro',
            'input_mount_performed': False,
            'parent_asset_mount_forbidden': True,
            'ground_truth_mount_forbidden': True,
            'scorer_mount_forbidden': True,
            'observed_probe_mounts': [],
            'probe_command_contract': probe['command'],
        },
        'consumer_contract': {
            'topics': EXPECTED_TOPICS,
            'expected_topic_counts': EXPECTED_COUNTS,
            'expected_messages': sum(EXPECTED_COUNTS.values()),
            'ack_backpressure_verified': True,
            'queue_overflow_capability': 'single_inflight_exact_count_observed',
            'eof_service': '/m6a10/consumer_eof',
            'finalize_service': '/m6a10/consumer_finalize',
            'no_map_guard_verified': True,
        },
        'input_identity_expected_only': {
            'path': input_contract['path'],
            'bytes': input_contract['bytes'],
            'sha256': input_contract['sha256'],
            'content_read_during_preflight': False,
        },
        'safety': {
            'bag_replay_started': False,
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
            'retry': 0,
        },
        'runtime_result': None,
        'created_at': dt.datetime.now(dt.timezone.utc).isoformat(),
    }


def atomic_write(path: Path, value: dict[str, Any]) -> str:
    if path.exists() or path.is_symlink():
        raise PreflightError(f'refusing to overwrite existing receipt: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + '.part')
    if part.exists() or part.is_symlink():
        raise PreflightError(f'stale receipt staging file exists: {part}')
    payload = (json.dumps(value, indent=2, sort_keys=True) + '\n').encode()
    with part.open('xb') as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(part, path)
    return hashlib.sha256(payload).hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        phase = load_phase()
        host = verify_host_bindings(phase)
        image = inspect_image(phase)
        probe = run_image_probe(image)
        receipt = build_receipt(phase, host=host, image=image, probe=probe)
        receipt_sha = atomic_write(args.output, receipt)
        print(json.dumps({
            'status': receipt['status'], 'path': str(args.output),
            'sha256': receipt_sha, 'image_id': IMAGE_ID,
        }, sort_keys=True))
        return 0
    except Exception as exc:
        failure = {
            'schema_version': 1, 'receipt_kind': RECEIPT_KIND,
            'contract_id': CONTRACT_ID, 'status': 'INVALID',
            'failure': {'type': type(exc).__name__, 'message': str(exc)},
            'safety': {
                'bag_replay_started': False,
                'ground_truth_content_opened': False,
                'scorer_invoked': False, 'retry': 0,
            },
        }
        try:
            receipt_sha = atomic_write(args.output, failure)
        except Exception:
            receipt_sha = None
        print(json.dumps({
            'status': 'INVALID', 'path': str(args.output),
            'sha256': receipt_sha, 'error': str(exc),
        }, sort_keys=True), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
