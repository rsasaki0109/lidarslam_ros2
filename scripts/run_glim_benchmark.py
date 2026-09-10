#!/usr/bin/env python3
"""Run the pinned official GLIM CPU track repeatedly in Docker."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any

_SCRIPT_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if (
        (_SCRIPT_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file()
        and str(_SCRIPT_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SCRIPT_SOURCE_ROOT))

from lidarslam_benchmark_tools.run_fast_livo2_benchmark import (
    benchmark_machine_fingerprint, parse_time_report, read_int,
    current_competitive_closure_identity, validate_frozen_input_manifest)
import yaml


ROOT = _SCRIPT_SOURCE_ROOT
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'


def sha256_tree(path: Path) -> str:
    """Use the synchronized-tail materializer's canonical tree identity.

    The fixed NTU input and its profile were materialized with
    ``relative_path_size_content_sha256_v1``.  Reusing that helper avoids a
    second, incompatible tree-hash definition in the replay runner.
    """
    from lidarslam_benchmark_tools.materialize_m6a10_synchronized_tail import sha256_tree as canonical

    return str(canonical(path)['sha256'])


def bag_bounds(metadata_path: Path) -> tuple[float, float]:
    info = yaml.safe_load(metadata_path.read_text())['rosbag2_bagfile_information']
    start = info['starting_time']['nanoseconds_since_epoch'] * 1e-9
    duration = info['duration']['nanoseconds'] * 1e-9
    return start, start + duration


def trajectory_info(path: Path) -> dict[str, Any]:
    rows = []
    if path.exists():
        for line in path.read_text(errors='replace').splitlines():
            if not line or line.startswith('#'):
                continue
            fields = line.split()
            if len(fields) >= 8:
                rows.append(fields)
    return {
        'samples': len(rows),
        'first_stamp': float(rows[0][0]) if rows else None,
        'last_stamp': float(rows[-1][0]) if rows else None,
    }


def image_labels(image: str) -> dict[str, str]:
    output = subprocess.run(
        ['docker', 'image', 'inspect', image], check=True, text=True,
        capture_output=True).stdout
    inspection = json.loads(output)[0]
    return inspection['Id'], inspection['Config'].get('Labels') or {}


def validate_glim_v2_binding(
        phase: dict[str, Any], *, bag_path: Path, bag_hash: str,
        config_path: Path, config_hash: str, labels: dict[str, str],
        glim_revision: str, glim_ros2_revision: str) -> None:
    """Fail closed unless every v2 input/build identity is preregistered."""
    expected_input = Path(phase['input']['path']).resolve()
    if bag_path.resolve() != expected_input:
        raise ValueError('GLIM v2 bag path is not the preregistered input')
    if bag_hash != phase['input']['tree_sha256']:
        raise ValueError('GLIM v2 bag tree SHA differs from preregistration')
    expected_config = (ROOT / phase['config']['path']).resolve()
    if config_path.resolve() != expected_config:
        raise ValueError('GLIM v2 config path is not preregistered')
    if config_hash != phase['config']['tree_sha256']:
        raise ValueError('GLIM v2 config tree SHA differs from preregistration')
    expected_labels = {
        'benchmark.glim.revision': glim_revision,
        'benchmark.glim_ros2.revision': glim_ros2_revision,
        'benchmark.glim.build_with_cv_bridge': str(
            phase['build']['build_with_cv_bridge']),
        'benchmark.glim.m6a10_core_patch_sha256':
            phase['source']['core_patch_sha256'],
        'benchmark.glim.m6a10_ros2_patch_sha256':
            phase['source']['ros2_patch_sha256'],
    }
    mismatches = {
        key: (labels.get(key), expected)
        for key, expected in expected_labels.items()
        if labels.get(key) != expected}
    if mismatches:
        raise RuntimeError(f'container v2 labels mismatch: {mismatches}')


def glim_v2_environment(phase: dict[str, Any], mode: str,
                        required_end_timestamp: float) -> dict[str, str]:
    """Build the complete opt-in GLIM v2 environment from the preregistration."""
    if phase.get('status') != 'preregistered_not_executed' or \
            phase.get('result') is not None:
        raise ValueError('GLIM v2 phase identity is not preregistered')
    input_identity = phase.get('input', {})
    phase_identity = phase.get('phase', {})
    build_identity = phase.get('build', {})
    expected = {
        'M6A10_GLIM_EXPECTED_MESSAGES': input_identity.get(
            'expected_messages'),
        'M6A10_GLIM_EXPECTED_IMU_MESSAGES': input_identity.get(
            'imu_messages'),
        'M6A10_GLIM_EXPECTED_POINTS_MESSAGES': input_identity.get(
            'lidar_messages'),
        'M6A10_GLIM_EXPECTED_IMAGE_MESSAGES': input_identity.get(
            'image_messages'),
        'M6A10_GLIM_REQUIRED_END_TIMESTAMP_SECONDS': required_end_timestamp,
        'M6A10_GLIM_MAX_CALLBACK_LATENCY_NS': phase_identity.get(
            'maximum_callback_latency_nanoseconds'),
        'M6A10_GLIM_MAX_CALLBACK_LATENCY_SECONDS': phase_identity.get(
            'maximum_callback_latency_seconds'),
        'M6A10_GLIM_MAX_BACKLOG_MESSAGES': phase_identity.get(
            'maximum_backlog_messages'),
    }
    if any(value is None for value in expected.values()):
        raise ValueError('GLIM v2 phase environment is incomplete')
    return {
        'GLIM_PROFILE': 'ntu_viral_cpu',
        'M6A10_PHASE_CONTRACT_VERSION': 'm6a10-online-compute-v2',
        'M6A10_PHASE_MODE': mode,
        'M6A10_GLIM_BUILD_WITH_CV_BRIDGE': str(
            build_identity.get('build_with_cv_bridge')),
        **{key: str(value) for key, value in expected.items()},
    }


def build_docker_command(args: argparse.Namespace, run_dir: Path, index: int,
                         bag_end: float, phase: dict[str, Any] | None
                         ) -> list[str]:
    """Build the isolated Docker argv, keeping v2 input mounts narrow."""
    # A v2 replay receives only the canonical ROS2 bag directory.  Mounting
    # its parent would make sibling assets (including GT) reachable even when
    # BAG_PATH names only the selected directory.  Preserve the legacy v1
    # layout for historical runs, but make the v2 source/target unambiguous.
    if phase is not None:
        input_env = 'BAG_PATH=/data'
        input_mount = f'{args.bag}:/data:ro'
    else:
        input_env = f'BAG_PATH=/data/{args.bag.name}'
        input_mount = f'{args.bag.parent}:/data:ro'
    isolation = (['--network', 'none', '--read-only',
                  '--tmpfs', '/tmp:rw,noexec,nosuid,size=1024m',
                  '--shm-size', '512m'] if phase is not None else [])
    command = [
        'docker', 'run', '--pull=never', '--rm', '--init', *isolation, '--name',
        f'glim-cpu-bench-{index}-{os.getpid()}',
        '-e', input_env,
        '-v', input_mount,
        '-v', f'{ROOT}:/runner:ro', '-v', f'{run_dir}:/out',
    ]
    if phase is not None:
        for key, value in glim_v2_environment(
                phase, args.phase_mode, bag_end).items():
            command.extend(['-e', f'{key}={value}'])
    command.extend([
        args.image, '/bin/bash', '/runner/scripts/glim_container_run.sh'])
    return command


def prepare_output_directory(output: Path, phase: dict[str, Any] | None) -> None:
    """Create a fresh run root, preserving only a passed v2 quiescence receipt."""
    if not output.exists():
        output.mkdir(parents=True)
        return
    if phase is None or not output.is_dir():
        raise ValueError(f'refusing to reuse output root: {output}')
    entries = {entry.name for entry in output.iterdir()}
    if entries != {'quiescence.json'}:
        raise ValueError(
            'v2 output root must contain only the preregistered quiescence receipt')
    try:
        quiescence = json.loads((output / 'quiescence.json').read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError('quiescence receipt is unreadable') from error
    if (quiescence.get('status') != 'PASS' or
            quiescence.get('runner_start_allowed') is not True):
        raise ValueError('quiescence receipt is not a PASS start gate')


def run_once(args: argparse.Namespace, output: Path, index: int,
             shared: dict[str, Any], duration: float, bag_end: float,
             phase: dict[str, Any] | None) -> dict[str, Any]:
    run_dir = output / f'run_{index:02d}'
    run_dir.mkdir(parents=True, exist_ok=False)
    command = build_docker_command(args, run_dir, index, bag_end, phase)
    started = dt.datetime.now(dt.timezone.utc)
    with (run_dir / 'container_stdout.log').open('w') as stdout, \
            (run_dir / 'container_stderr.log').open('w') as stderr:
        completed = subprocess.run(command, stdout=stdout, stderr=stderr,
                                   check=False)
    finished = dt.datetime.now(dt.timezone.utc)
    process_time = parse_time_report(run_dir / 'process_time.txt')
    trajectory = trajectory_info(run_dir / 'dump/traj_lidar.txt')
    end_gap = (None if trajectory['last_stamp'] is None
               else bag_end - trajectory['last_stamp'])
    processing_rtf = (float(process_time['wall_seconds']) / duration
                      if process_time.get('wall_seconds') else None)
    process_exit = read_int(run_dir / 'process_exit_status.txt')
    consumer = None
    eof_sidecar = None
    if phase is not None:
        consumer_path = run_dir / 'consumer_evidence.json'
        eof_path = run_dir / 'consumer_evidence.json.eof.json'
        if consumer_path.is_file():
            consumer = json.loads(consumer_path.read_text(encoding='utf-8'))
        if eof_path.is_file():
            eof_sidecar = json.loads(eof_path.read_text(encoding='utf-8'))
    complete = (completed.returncode == 0 and process_exit == 0 and
                trajectory['samples'] > 0 and end_gap is not None and
                end_gap <= args.maximum_end_gap_seconds and
                (phase is None or (consumer and consumer.get('status') == 'pass' and
                                   eof_sidecar and eof_sidecar.get('status') == 'eof')))
    report = {
        'schema_version': 1, 'system': 'glim_cpu', 'run_index': index,
        'started_at': started.isoformat(), 'finished_at': finished.isoformat(),
        'provenance': shared,
        'execution': {'container_exit_status': completed.returncode,
                      'process_exit_status': process_exit},
        'completion': {'trajectory_complete': complete,
                       'trajectory_end_gap_seconds': end_gap,
                       'process_exit_status': process_exit},
        'trajectory': trajectory,
        'consumer_evidence': consumer,
        'consumer_eof_boundary': eof_sidecar,
        'runtime': {'bag_duration_seconds': duration,
                    'processing_realtime_factor': processing_rtf,
                    **process_time},
        'mount_contract': {
            'input_source': str(args.bag),
            'input_target': '/data',
            'input_options': 'ro',
            'parent_mount_forbidden': phase is not None,
            'bag_path_in_container': '/data' if phase is not None
            else f'/data/{args.bag.name}',
        },
    }
    (run_dir / 'run.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--input-manifest', type=Path)
    parser.add_argument('--image', default='glim-cpu-benchmark:competitive-v1')
    parser.add_argument('--runs', type=int)
    parser.add_argument('--maximum-end-gap-seconds', type=float, default=0.25)
    parser.add_argument('--phase-contract', choices=('v1', 'v2'), default='v1')
    parser.add_argument(
        '--phase-id', default='m6a10_glim_v2b',
        help='profile contract key used with --phase-contract v2')
    parser.add_argument('--phase-mode', choices=('paced_1x', 'unpaced_ack'),
                        default='paced_1x')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.bag, args.output = args.bag.resolve(), args.output.resolve()
    contract = yaml.safe_load(args.profile.read_text())['competitive_slam_profile']
    closure_identity = current_competitive_closure_identity(contract)
    runs = contract['repetitions'] if args.runs is None else args.runs
    if runs < 1 or not (args.bag / 'metadata.yaml').exists():
        raise ValueError('positive runs and a ROS2 bag directory are required')
    phase = None
    if args.phase_contract == 'v2':
        phase = contract.get(args.phase_id)
        if not isinstance(phase, dict):
            raise ValueError(f'profile has no GLIM v2 phase: {args.phase_id}')
    image_id, labels = image_labels(args.image)
    rival = contract['rivals']['glim']
    expected_labels = {
        'benchmark.glim.revision': rival['revision'],
        'benchmark.glim_ros2.revision': rival['ros2_revision'],
    }
    if phase is not None:
        expected_labels.update({
            'benchmark.glim.build_with_cv_bridge':
            str(phase['build']['build_with_cv_bridge']),
            'benchmark.glim.m6a10_core_patch_sha256':
            phase['source']['core_patch_sha256'],
            'benchmark.glim.m6a10_ros2_patch_sha256':
            phase['source']['ros2_patch_sha256'],
        })
    mismatches = {key: (labels.get(key), expected)
                  for key, expected in expected_labels.items()
                  if labels.get(key) != expected}
    if mismatches:
        raise RuntimeError(f'container revision labels mismatch: {mismatches}')
    start, end = bag_bounds(args.bag / 'metadata.yaml')
    bag_hash = sha256_tree(args.bag)
    manifest = validate_frozen_input_manifest(
        args.input_manifest, 'canonical_rosbag2_tree_sha256', bag_hash)
    config_path = (ROOT / phase['config']['path'] if phase is not None else
                   ROOT / 'configs/glim/hilti2022_cpu')
    config_hash = sha256_tree(config_path)
    if phase is not None:
        validate_glim_v2_binding(
            phase, bag_path=args.bag, bag_hash=bag_hash,
            config_path=config_path, config_hash=config_hash, labels=labels,
            glim_revision=rival['revision'],
            glim_ros2_revision=rival['ros2_revision'])
    prepare_output_directory(args.output, phase)
    shared = {
        'profile': contract['name'], 'image': args.image, 'image_id': image_id,
        'rival_source_closure': closure_identity,
        'machine': benchmark_machine_fingerprint(),
        'image_labels': labels, 'bag_path': str(args.bag),
        'bag_sha256': bag_hash, 'input_manifest': manifest,
        'config_path': str(config_path),
        'config_sha256': config_hash,
    }
    reports = []
    for index in range(1, runs + 1):
        print(f'GLIM CPU repetition {index}/{runs}', flush=True)
        reports.append(run_once(
            args, args.output, index, shared, end - start, end, phase))
    summary = {
        'schema_version': 1, 'system': 'glim_cpu', 'requested_runs': runs,
        'completed_trajectories': sum(
            report['completion']['trajectory_complete'] for report in reports),
        'clean_exits': sum(
            report['completion']['process_exit_status'] == 0 for report in reports),
        'runs': reports,
    }
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({key: summary[key] for key in (
        'requested_runs', 'completed_trajectories', 'clean_exits')}, indent=2))
    return 0 if summary['completed_trajectories'] == runs else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, KeyError,
            subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
