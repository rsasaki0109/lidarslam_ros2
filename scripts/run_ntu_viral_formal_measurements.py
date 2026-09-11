#!/usr/bin/env python3
"""Run one frozen NTU-VIRAL sequence through all formal raw measurements."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

import yaml

from competitive_candidate_provenance import verify_candidate_manifest


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def docker_image_id(image: str) -> str:
    return subprocess.run(
        ['docker', 'image', 'inspect', image, '--format', '{{.Id}}'],
        check=True, text=True, capture_output=True).stdout.strip()


def validate_container_images(path: Path) -> dict[str, Any]:
    records = json.loads(path.read_text())
    required = {'glim_cpu', 'fast_livo2'}
    if set(records) != required:
        raise ValueError(
            f'container manifest must contain exactly {sorted(required)}')
    for name in sorted(required):
        record = records[name]
        image = record.get('image')
        expected = record.get('image_id')
        if not isinstance(image, str) or not isinstance(expected, str):
            raise ValueError(f'invalid container manifest record: {name}')
        actual = docker_image_id(image)
        if actual != expected:
            raise ValueError(
                f'frozen container image mismatch for {name}: '
                f'expected {expected}, got {actual}')
    return records


def commands(args: argparse.Namespace, probe_rate: float) -> list[dict[str, Any]]:
    common_ours = [
        sys.executable, str(ROOT / 'scripts/run_ours_competitive_benchmark.py'),
        '--bag', str(args.ros2_bag), '--reference-tum', str(args.reference),
        '--reference-meta', str(args.reference_meta),
        '--profile', str(args.profile), '--input-manifest', str(args.manifest),
        '--candidate-manifest', str(args.candidate_manifest),
        '--lidarslam-param', str(ROOT / 'lidarslam/param/lidarslam.yaml'),
        '--lidar-topic', '/os1_cloud_node1/points',
        '--imu-topic', '/imu/imu', '--base-frame', 'base_link',
        '--reference-source', 'ntu_viral_official_csv', '--runs', '3',
        '--save-maps']
    common_fast = [
        sys.executable, str(ROOT / 'scripts/run_fast_livo2_benchmark.py'),
        '--asset-root', str(args.fast_asset_root), '--bag', str(args.ros1_bag),
        '--profile', str(args.profile), '--input-manifest', str(args.manifest),
        '--reference-meta', str(args.reference_meta),
        '--mapping-launch', str(
            ROOT / 'configs/fast_livo2/mapping_ouster_ntu_viral.launch'),
        '--mapping-map-launch', str(
            ROOT / 'configs/fast_livo2/mapping_ouster_ntu_viral_map.launch'),
        '--trajectory-source-frame', 'imu', '--runs', '3',
        '--image', args.fast_image]
    return [
        {'name': 'glim_rival', 'command': [
            sys.executable, str(ROOT / 'scripts/run_glim_benchmark.py'),
            '--bag', str(args.ros2_bag),
            '--output', str(args.output / 'glim_rival'),
            '--profile', str(args.profile), '--input-manifest', str(args.manifest),
            '--reference-meta', str(args.reference_meta),
            '--config-dir', str(ROOT / 'configs/glim/ntu_viral_ouster_cpu'),
            '--runs', '3', '--save-maps', '--image', args.glim_image]},
        {'name': 'ours_lio', 'command': [
            *common_ours, '--output', str(args.output / 'ours_lio'),
            '--rko-param', str(ROOT / 'lidarslam/param/rko_lio_ntu_viral.yaml')]},
        {'name': 'fast_rival_baseline', 'command': [
            *common_fast, '--output', str(args.output / 'fast_rival_baseline'),
            '--rate', '1.0', '--save-map']},
        {'name': 'ours_liv', 'command': [
            *common_ours, '--output', str(args.output / 'ours_liv'),
            '--rko-param', str(
                ROOT / 'lidarslam/param/rko_lio_ntu_viral_direct_visual.yaml'),
            '--camera-topic', '/left/image_raw']},
        {'name': 'fast_rival_processing_probe', 'command': [
            *common_fast,
            '--output', str(args.output / 'fast_rival_processing_probe'),
            '--rate', str(probe_rate)]},
    ]


def validate_contract(args: argparse.Namespace) -> tuple[dict[str, Any], float]:
    profile = yaml.safe_load(args.profile.read_text())['competitive_slam_profile']
    manifest = json.loads(args.manifest.read_text())
    if manifest.get('status') != 'frozen':
        raise ValueError('input manifest is not frozen')
    sequence = manifest.get('sequence')
    slot_rows = profile['datasets']['holdout_slots']
    matching = [row for row in slot_rows.values()
                if row.get('sequence') == sequence]
    if len(matching) != 1 or matching[0].get('status') != 'frozen':
        raise ValueError('sequence does not identify one frozen profile slot')
    if matching[0]['input_manifest_sha256'] != sha256(args.manifest):
        raise ValueError('profile/input-manifest SHA mismatch')
    candidate = verify_candidate_manifest(ROOT, args.candidate_manifest)
    frozen_candidate = json.loads(args.candidate_manifest.read_text())
    by_track = frozen_candidate.get('rko_params_by_track') or {}
    expected_params = {
        'glim_cpu_lidar_imu': ROOT / 'lidarslam/param/rko_lio_ntu_viral.yaml',
        'fast_livo2_lidar_imu_visual': (
            ROOT / 'lidarslam/param/rko_lio_ntu_viral_direct_visual.yaml')}
    for track, path in expected_params.items():
        if by_track.get(track, {}).get('sha256') != sha256(path):
            raise ValueError(f'candidate track parameter mismatch: {track}')
    containers = validate_container_images(args.container_images)
    args.glim_image = containers['glim_cpu']['image']
    args.fast_image = containers['fast_livo2']['image']
    runtime = profile['runtime_policy']
    if int(runtime['fast_livo2_processing_probe_repetitions']) != 3:
        raise ValueError('formal FAST processing probe must use three repetitions')
    return {
        'sequence': sequence, 'profile_sha256': sha256(args.profile),
        'input_manifest_sha256': sha256(args.manifest),
        'container_images_path': str(args.container_images),
        'container_images_sha256': sha256(args.container_images),
        'container_images': containers,
        'candidate': candidate,
    }, float(runtime['fast_livo2_processing_probe_rate'])


def load_is_quiet(maximum: float) -> tuple[bool, float]:
    load1 = os.getloadavg()[0]
    return load1 <= maximum, load1


def _cpu_totals() -> tuple[int, int]:
    fields = [int(value) for value in Path('/proc/stat').read_text().splitlines()[
        0].split()[1:]]
    idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
    return sum(fields), idle


def cpu_busy_percent(sample_seconds: float = 2.0) -> float:
    total_a, idle_a = _cpu_totals()
    time.sleep(sample_seconds)
    total_b, idle_b = _cpu_totals()
    elapsed = total_b - total_a
    if elapsed <= 0:
        raise RuntimeError('failed to sample CPU utilization')
    return 100.0 * (1.0 - (idle_b - idle_a) / elapsed)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros1-bag', type=Path, required=True)
    parser.add_argument('--ros2-bag', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--reference-meta', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--candidate-manifest', type=Path, required=True)
    parser.add_argument('--container-images', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--fast-asset-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--maximum-load1', type=float, default=2.0)
    parser.add_argument('--maximum-cpu-busy-percent', type=float, default=10.0)
    args = parser.parse_args()
    for name in ('ros1_bag', 'ros2_bag', 'reference', 'reference_meta',
                 'manifest', 'candidate_manifest', 'container_images', 'profile',
                 'fast_asset_root', 'output'):
        setattr(args, name, getattr(args, name).resolve())
    if args.output.exists():
        raise ValueError(f'refusing to overwrite formal output: {args.output}')
    quiet, load1 = load_is_quiet(args.maximum_load1)
    if not quiet:
        raise RuntimeError(
            f'machine is not quiescent: load1={load1:.3f} '
            f'maximum={args.maximum_load1:.3f}')
    initial_busy = cpu_busy_percent()
    if initial_busy > args.maximum_cpu_busy_percent:
        raise RuntimeError(
            f'machine CPU is not quiescent: busy={initial_busy:.2f}% '
            f'maximum={args.maximum_cpu_busy_percent:.2f}%')
    contract, probe_rate = validate_contract(args)
    args.output.mkdir(parents=True)
    subprocess.run([
        sys.executable, str(ROOT / 'scripts/capture_benchmark_machine_fingerprint.py'),
        '--output', str(args.output / 'machine.json')], check=True)
    report = {
        'schema_version': 1, 'status': 'running', 'contract': contract,
        'maximum_load1': args.maximum_load1, 'initial_load1': load1,
        'maximum_cpu_busy_percent': args.maximum_cpu_busy_percent,
        'initial_cpu_busy_percent': initial_busy,
        'stages': []}
    status_path = args.output / 'orchestration.json'
    status_path.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    all_passed = True
    for stage in commands(args, probe_rate):
        busy = cpu_busy_percent()
        row = {
            'name': stage['name'], 'command': stage['command'],
            'load1_before': os.getloadavg()[0], 'cpu_busy_percent_before': busy,
            'started_at': dt.datetime.now(dt.timezone.utc).isoformat()}
        if busy > args.maximum_cpu_busy_percent:
            row.update({'returncode': None, 'status': 'aborted_external_load'})
            all_passed = False
            report['stages'].append(row)
            break
        completed = subprocess.run(stage['command'], cwd=ROOT, check=False)
        row.update({
            'returncode': completed.returncode,
            'status': 'complete' if completed.returncode == 0 else 'failed',
            'finished_at': dt.datetime.now(dt.timezone.utc).isoformat()})
        all_passed &= completed.returncode == 0
        report['stages'].append(row)
        status_path.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    report['status'] = 'complete' if all_passed and len(report['stages']) == 5 \
        else 'incomplete'
    status_path.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    return 0 if report['status'] == 'complete' else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, KeyError, TypeError,
            json.JSONDecodeError, yaml.YAMLError,
            subprocess.CalledProcessError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
