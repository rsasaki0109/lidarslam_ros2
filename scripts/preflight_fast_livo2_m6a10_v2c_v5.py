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

"""Read-only identity preflight for the FAST-LIVO2 v5 image."""

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
PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
CONTRACT = 'm6a10-v2c-fast-livo2-observable-supervision-v1'
IMAGE_OBSERVABILITY = 'm6a10-fast-livo2-observable-supervision-v1'
KIND = 'fast_livo2_m6a10_v2c_v5_execution_identity_preflight'
SOURCE_REV = '0d2c0346107b75b59934975adec9a6eeeb913c64'
RPG_REV = '6c886c8e5d83997806e00294826d528cea3581dd'
PATCH_SHA = '33f30a40ad54db5eea331a87b8b86f32410aac2980e37e4cb6d5027100095297'
CONFIG_SHA = 'c8f94f130e599b928c3f02c3f3d3b2009ae01df76aec32f6ac96b6a987311ef3'


class PreflightError(RuntimeError):
    """A fail-closed identity error."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def command(argv: list[str], timeout: int = 180) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(argv, check=False, capture_output=True,
                              text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise PreflightError(f'command failed or timed out: {argv[0]}') from exc


def phase() -> dict[str, Any]:
    document = yaml.safe_load(PROFILE.read_text(encoding='utf-8'))
    retry = document['competitive_slam_profile']['m6a10_fast_livo2_v2c'][
        'retry_v5']
    if retry.get('contract_id') != CONTRACT:
        raise PreflightError('contract mismatch')
    if retry.get('status') != 'build_passed_not_executed':
        raise PreflightError('retry is not build_passed_not_executed')
    if retry.get('result') is not None or retry.get('runner_start_attempted'):
        raise PreflightError('retry has already started or has a result')
    if retry.get('bag_replay_started') or retry.get('gt_content_opened'):
        raise PreflightError('input or GT activity is already recorded')
    return retry


def verify_files(retry: dict[str, Any]) -> dict[str, Any]:
    source = retry['source']
    paths = {
        'patch': ROOT / source['patch_path'],
        'feeder': ROOT / retry['feeder']['path'],
        'runner': ROOT / retry['runner']['path'],
        'wrapper': ROOT / retry['container_entrypoint']['path'],
        'recipe': ROOT / retry['build']['recipe']['path'],
        'build_entrypoint': ROOT / retry['build']['entrypoint']['path'],
        'preflight': Path(__file__).resolve(),
    }
    expected = {
        'patch': source['patch_sha256'],
        'feeder': retry['feeder']['sha256'],
        'runner': retry['runner']['sha256'],
        'wrapper': retry['container_entrypoint']['sha256'],
        'recipe': retry['build']['recipe']['sha256'],
        'build_entrypoint': retry['build']['entrypoint']['sha256'],
        'preflight': retry['preflight']['script_sha256'],
    }
    observed: dict[str, Any] = {}
    for name, path in paths.items():
        if not path.is_file() or path.is_symlink():
            raise PreflightError(f'{name} is not a regular file')
        actual = sha256(path)
        observed[name] = {'path': str(path.relative_to(ROOT)),
                          'sha256': actual}
        if expected[name] and actual != expected[name]:
            raise PreflightError(f'{name} SHA mismatch: {actual}')
    build_path = Path(retry['build']['receipt_path'])
    if sha256(build_path) != retry['build']['receipt_sha256']:
        raise PreflightError('build receipt SHA mismatch')
    build = json.loads(build_path.read_text(encoding='utf-8'))
    if build.get('status') != 'PASS' or not build.get('authoritative'):
        raise PreflightError('build receipt is not authoritative PASS')
    if build.get('image', {}).get('image_id') != retry['image']['image_id']:
        raise PreflightError('build receipt image ID mismatch')
    if build.get('image', {}).get('tag') != retry['image']['tag']:
        raise PreflightError('build receipt image tag mismatch')
    return {'files': observed,
            'build_receipt': {'path': str(build_path),
                              'sha256': retry['build']['receipt_sha256'],
                              'status': build['status']}}


def inspect_image(retry: dict[str, Any]) -> dict[str, Any]:
    image_ref = retry['image']['tag']
    result = command(['docker', 'image', 'inspect', image_ref])
    if result.returncode:
        raise PreflightError(result.stderr.strip() or 'docker inspect failed')
    try:
        document = json.loads(result.stdout)
        image = document[0] if isinstance(document, list) else document
    except (json.JSONDecodeError, IndexError) as exc:
        raise PreflightError('docker inspect did not return one image') from exc
    if image.get('Id') != retry['image']['image_id']:
        raise PreflightError('image ID mismatch')
    labels = image.get('Config', {}).get('Labels') or {}
    required = {
        'benchmark.fast_livo2.revision': SOURCE_REV,
        'benchmark.rpg_vikit.revision': RPG_REV,
        'benchmark.fast_livo2.m6a10_patch_sha256': PATCH_SHA,
        'benchmark.fast_livo2.m6a10_feeder_sha256': retry['feeder']['sha256'],
        'benchmark.fast_livo2.m6a10_observability_contract': IMAGE_OBSERVABILITY,
        'benchmark.fast_livo2.cpu_flags_policy': 'portable_x86_64_v1',
        'benchmark.cpu_policy': 'cpu_only;threads=8;native_flags=disabled',
    }
    for key, expected in required.items():
        if labels.get(key) != expected:
            raise PreflightError(f'label mismatch: {key}')
    observed_inspect_sha = hashlib.sha256(result.stdout.encode()).hexdigest()
    if observed_inspect_sha != retry['image']['image_inspect_sha256']:
        raise PreflightError('image inspect SHA mismatch')
    observed_labels_sha = hashlib.sha256(json.dumps(
        labels, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    expected_labels_sha = retry['image'].get(
        'labels_canonical_sha256', retry['image']['labels_sha256'])
    if observed_labels_sha != expected_labels_sha:
        raise PreflightError('image labels SHA mismatch')
    return {'id': image['Id'], 'repo_digests': image.get('RepoDigests', []),
            'size_bytes': image.get('Size'), 'labels': labels,
            'inspect_sha256': observed_inspect_sha,
            'labels_sha256': observed_labels_sha}


def probe(image_id: str) -> dict[str, Any]:
    script = r'''set -eu
export ROS_MASTER_URI=http://127.0.0.1:11311
. /opt/ros/noetic/setup.bash
. /opt/fast_livo_ws/devel/setup.bash
src=/opt/fast_livo_ws/src/FAST-LIVO2
rpg=/opt/fast_livo_ws/src/rpg_vikit
cfg="$src/config/NTU_VIRAL.yaml"
bin=/opt/fast_livo_ws/devel/lib/fast_livo/fastlivo_mapping
test "$(git -C "$src" rev-parse HEAD)" = '0d2c0346107b75b59934975adec9a6eeeb913c64'
test "$(git -C "$rpg" rev-parse HEAD)" = '6c886c8e5d83997806e00294826d528cea3581dd'
test "$(sha256sum "$cfg" | awk '{print $1}')" = 'c8f94f130e599b928c3f02c3f3d3b2009ae01df76aec32f6ac96b6a987311ef3'
test -x "$bin"
grep -q 'pcd_save_en: false' "$cfg"
grep -q 'm6a10_queue_size = m6a10_consumer_evidence.enabled() ? 1U : 200000U' "$src/src/LIVMapper.cpp"
grep -q 'm6a10_ack_service' "$src/src/LIVMapper.cpp"
grep -q 'm6a10_eof_service' "$src/src/LIVMapper.cpp"
grep -q 'm6a10_finalize_service' "$src/src/LIVMapper.cpp"
grep -q 'if (!m6a10_consumer_evidence.enabled()) savePCD();' "$src/src/LIVMapper.cpp"
! ldd "$bin" 2>&1 | grep -q 'not found'
flags=$(find /opt/fast_livo_ws/build -type f -name flags.make -exec cat {} +)
if printf '%s' "$flags" | grep -Eq -- '-march=(native|tigerlake)|-mcpu=native|-mtune=native|-m(avx512|avx2)'; then exit 17; fi
python3 - <<'PY'
import json
print(json.dumps({'source_revision': '0d2c0346107b75b59934975adec9a6eeeb913c64',
                  'rpg_revision': '6c886c8e5d83997806e00294826d528cea3581dd',
                  'config_sha256': 'c8f94f130e599b928c3f02c3f3d3b2009ae01df76aec32f6ac96b6a987311ef3',
                  'ldd_missing': False, 'active_forbidden_compile_flags': False,
                  'config_map_save_enabled': False,
                  'markers': {'ack_service': True, 'eof_service': True,
                              'finalize_service': True,
                              'single_inflight_queue': True,
                              'no_map_guard': True}},
                 sort_keys=True, separators=(',', ':')))
PY
'''
    argv = ['docker', 'run', '--pull=never', '--rm', '--init', '--network',
            'none', '--read-only', '--tmpfs', '/tmp:rw,noexec,nosuid,size=32m',
            '--tmpfs', '/root/.ros:rw,noexec,nosuid,size=32m', '--entrypoint',
            '/bin/bash', image_id, '-lc', script]
    result = command(argv)
    if result.returncode:
        raise PreflightError('read-only image probe failed: '
                             + result.stderr.strip())
    try:
        payload = json.loads(result.stdout.strip())
    except json.JSONDecodeError as exc:
        raise PreflightError('image probe did not return JSON') from exc
    if payload.get('config_sha256') != CONFIG_SHA:
        raise PreflightError('probe config SHA mismatch')
    if payload.get('ldd_missing') or payload.get('active_forbidden_compile_flags'):
        raise PreflightError('probe binary/CPU contract failed')
    if payload.get('config_map_save_enabled') is not False:
        raise PreflightError('map-save configuration enabled')
    if not all(payload.get('markers', {}).values()):
        raise PreflightError('installed marker contract incomplete')
    return {'command': argv, 'exit_status': result.returncode,
            'result': payload,
            'stdout_sha256': hashlib.sha256(result.stdout.encode()).hexdigest(),
            'stderr_sha256': hashlib.sha256(result.stderr.encode()).hexdigest()}


def atomic_write(path: Path, value: dict[str, Any]) -> str:
    if path.exists() or path.is_symlink():
        raise PreflightError(f'refusing to overwrite {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + '.part')
    if part.exists() or part.is_symlink():
        raise PreflightError(f'stale staging exists: {part}')
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
        retry = phase()
        files = verify_files(retry)
        image = inspect_image(retry)
        image_probe = probe(image['id'])
        receipt = {
            'schema_version': 1, 'receipt_kind': KIND,
            'contract_id': CONTRACT, 'status': 'PASS',
            'identity': {'image': image, 'host_bindings': files['files'],
                         'build_receipt': files['build_receipt'],
                         'probe': image_probe,
                         'preflight_script_sha256': sha256(Path(__file__))},
            'execution_mount_plan': {
                'network': 'none', 'rootfs': 'read_only',
                'input_mount_performed': False, 'observed_probe_mounts': [],
                'ground_truth_mount_forbidden': True,
                'scorer_mount_forbidden': True},
            'safety': {'bag_replay_started': False,
                       'ground_truth_content_opened': False,
                       'scorer_invoked': False, 'input_content_read': False,
                       'retry': 0},
            'runtime_result': None,
            'created_at': dt.datetime.now(dt.timezone.utc).isoformat(),
        }
        receipt_sha = atomic_write(args.output, receipt)
        print(json.dumps({'status': 'PASS', 'path': str(args.output),
                          'sha256': receipt_sha}, sort_keys=True))
        return 0
    except Exception as exc:
        failure = {'schema_version': 1, 'receipt_kind': KIND,
                   'contract_id': CONTRACT, 'status': 'INVALID',
                   'failure': {'type': type(exc).__name__, 'message': str(exc)},
                   'safety': {'bag_replay_started': False,
                              'ground_truth_content_opened': False,
                              'scorer_invoked': False,
                              'input_content_read': False, 'retry': 0}}
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
