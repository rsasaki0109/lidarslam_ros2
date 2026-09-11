#!/usr/bin/env python3
"""Run the measured RTK-SLAM accuracy path without per-sequence hand tuning."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / 'configs/slam_benchmark_profiles/rtk_slam_accuracy_v1.yaml'
BENCHMARK = ROOT / 'scripts/run_rko_lio_graph_benchmark.sh'
REFERENCE_GENERATOR = ROOT / 'scripts/generate_rtk_slam_reference.py'
DEFAULT_SEQUENCE = 'construction_seq2'


def load_contract(path: Path = PROFILE) -> dict[str, Any]:
    """Load and validate the repository-owned suite contract."""
    document = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    contract = document.get('rtk_slam_accuracy_suite')
    if not isinstance(contract, dict):
        raise ValueError(f'{path}: missing rtk_slam_accuracy_suite mapping')
    if contract.get('schema_version') != 1:
        raise ValueError(f'{path}: unsupported schema_version')
    sensor = contract.get('sensor')
    runtime = contract.get('runtime')
    sequences = contract.get('sequences')
    if not isinstance(sensor, dict) or not isinstance(runtime, dict):
        raise ValueError(f'{path}: sensor and runtime mappings are required')
    if not isinstance(sequences, dict) or not sequences:
        raise ValueError(f'{path}: at least one sequence is required')

    required_sensor = ('lidar_topic', 'imu_topic', 'base_frame')
    required_runtime = (
        'quiescence_secs', 'offline_timeout_secs', 'save_timeout_secs')
    required_sequence = (
        'bag', 'checkpoints', 'checkpoint_count', 'rko_param',
        'completion_end_margin_secs')
    for key in required_sensor:
        if not isinstance(sensor.get(key), str) or not sensor[key]:
            raise ValueError(f'{path}: sensor.{key} must be a non-empty string')
    for key in required_runtime:
        if not isinstance(runtime.get(key), int) or runtime[key] <= 0:
            raise ValueError(f'{path}: runtime.{key} must be a positive integer')
    for name, sequence in sequences.items():
        if not isinstance(sequence, dict):
            raise ValueError(f'{path}: sequences.{name} must be a mapping')
        missing = [key for key in required_sequence if key not in sequence]
        if missing:
            raise ValueError(f'{path}: sequences.{name} missing {missing}')
        if not isinstance(sequence['checkpoint_count'], int) or \
                sequence['checkpoint_count'] <= 0:
            raise ValueError(
                f'{path}: sequences.{name}.checkpoint_count must be positive')
        margin = sequence['completion_end_margin_secs']
        if isinstance(margin, bool) or not isinstance(margin, (int, float)) or \
                margin < 0:
            raise ValueError(
                f'{path}: sequences.{name}.completion_end_margin_secs '
                'must be non-negative')
    return contract


def selected_sequences(requested: list[str], contract: dict[str, Any]) -> list[str]:
    """Expand ``all`` and reject unknown or duplicate sequence selections."""
    available = list(contract['sequences'])
    if not requested:
        return [DEFAULT_SEQUENCE]
    if 'all' in requested:
        if len(requested) != 1:
            raise ValueError("'all' cannot be combined with another --sequence")
        return available
    unknown = [name for name in requested if name not in available]
    if unknown:
        raise ValueError(
            f"unknown sequence(s): {', '.join(unknown)}; "
            f"choices: {', '.join(available)}, all")
    if len(set(requested)) != len(requested):
        raise ValueError('duplicate --sequence values are not allowed')
    return requested


def resolve_inputs(
        dataset_root: Path, name: str, sequence: dict[str, Any],
        contract: dict[str, Any]) -> dict[str, Path]:
    """Resolve and preflight all dataset and repository inputs for one run."""
    paths = {
        'bag': dataset_root / sequence['bag'],
        'checkpoints': dataset_root / sequence['checkpoints'],
        'rko_param': ROOT / sequence['rko_param'],
        'lidarslam_param': ROOT / contract['lidarslam_param'],
    }
    missing = []
    if not (paths['bag'] / 'metadata.yaml').is_file():
        missing.append(f"ROS2 bag metadata: {paths['bag'] / 'metadata.yaml'}")
    for key in ('checkpoints', 'rko_param', 'lidarslam_param'):
        if not paths[key].is_file():
            missing.append(f'{key}: {paths[key]}')
    if missing:
        raise ValueError(f'{name}: missing input(s):\n  ' + '\n  '.join(missing))
    return {key: path.resolve() for key, path in paths.items()}


def commands_for(
        name: str, sequence: dict[str, Any], contract: dict[str, Any],
        paths: dict[str, Path], output_root: Path,
        save_maps: bool) -> tuple[list[str], list[str]]:
    """Build the reference and benchmark commands for one sequence."""
    reference_dir = output_root / 'reference'
    reference_tum = reference_dir / f'{name}_gt.tum'
    reference_meta = reference_dir / f'{name}_reference.json'
    sequence_output = output_root / name
    generate = [
        sys.executable, str(REFERENCE_GENERATOR),
        '--checkpoints', str(paths['checkpoints']),
        '--sequence', name,
        '--out', str(reference_tum),
        '--write-meta', str(reference_meta),
    ]
    sensor = contract['sensor']
    runtime = contract['runtime']
    benchmark = [
        'bash', str(BENCHMARK),
        '--bag', str(paths['bag']),
        '--lidar-topic', sensor['lidar_topic'],
        '--imu-topic', sensor['imu_topic'],
        '--base-frame', sensor['base_frame'],
        '--lidarslam-param', str(paths['lidarslam_param']),
        '--rko-param', str(paths['rko_param']),
        '--reference-tum', str(reference_tum),
        '--reference-meta', str(reference_meta),
        '--skip-reference-gen',
        '--reference-source', f'rtk_slam_{name}_gt',
        '--completion-end-margin-secs',
        str(sequence['completion_end_margin_secs']),
        '--quiescence-secs', str(runtime['quiescence_secs']),
        '--offline-timeout-secs', str(runtime['offline_timeout_secs']),
        '--save-timeout-secs', str(runtime['save_timeout_secs']),
        '--run-name', f'rtk_slam_{name}',
        '--output-dir', str(sequence_output),
    ]
    if not save_maps:
        benchmark.append('--skip-map-save')
    return generate, benchmark


def summarize_run(name: str, output_root: Path, expected_pairs: int) -> dict[str, Any]:
    """Read the canonical metrics artifact and retain suite-level essentials."""
    metrics_path = output_root / name / 'metrics.json'
    metrics = json.loads(metrics_path.read_text(encoding='utf-8'))
    raw_ape = metrics.get('evo', {}).get('raw_ape', {})
    pairs = raw_ape.get('pairs')
    if pairs != expected_pairs:
        raise ValueError(
            f'{name}: expected {expected_pairs} checkpoint pairs, got {pairs}')
    return {
        'sequence': name,
        'metrics_path': str(metrics_path.resolve()),
        'ape_rmse_gt_m': raw_ape.get('rmse'),
        'ape_median_gt_m': raw_ape.get('median'),
        'checkpoint_pairs': pairs,
        'completion': metrics.get('completion'),
        'git_commit': metrics.get('provenance', {}).get(
            'software', {}).get('git_commit'),
        'git_dirty': metrics.get('provenance', {}).get(
            'software', {}).get('git_dirty'),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--dataset-root', type=Path, default=ROOT / 'datasets/rtk_slam',
        help='root containing ros2/ and rtk_slam_eval/ (default: '
             'datasets/rtk_slam)')
    parser.add_argument(
        '--sequence', action='append', default=[],
        help=f'sequence to run; repeat for several, or use all '
             f'(default: {DEFAULT_SEQUENCE})')
    parser.add_argument(
        '--output-dir', type=Path,
        help='new output root (default: output/rtk_slam_accuracy_<timestamp>)')
    parser.add_argument(
        '--save-maps', action='store_true',
        help='save and verify map bundles in addition to trajectory accuracy')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='validate inputs and print exact commands without running ROS')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    contract = load_contract()
    names = selected_sequences(args.sequence, contract)
    dataset_root = args.dataset_root.expanduser().resolve()
    output_root = (
        args.output_dir.expanduser().resolve() if args.output_dir else
        ROOT / 'output' /
        f"rtk_slam_accuracy_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    resolved = {
        name: resolve_inputs(
            dataset_root, name, contract['sequences'][name], contract)
        for name in names
    }
    plans = {
        name: commands_for(
            name, contract['sequences'][name], contract, resolved[name],
            output_root, args.save_maps)
        for name in names
    }

    print(f"suite: {contract['name']}")
    print(f'sequences: {", ".join(names)}')
    print(f'output: {output_root}')
    for name in names:
        print(f'\n[{name}]')
        for command in plans[name]:
            print(shlex.join(command))
    if args.dry_run:
        return 0
    if output_root.exists():
        raise ValueError(f'output directory already exists: {output_root}')
    output_root.mkdir(parents=True)
    plan = {
        'schema_version': 1,
        'suite': contract['name'],
        'profile_path': str(PROFILE),
        'profile_sha256': hashlib.sha256(PROFILE.read_bytes()).hexdigest(),
        'dataset_root': str(dataset_root),
        'attribution': contract['attribution'],
        'save_maps': args.save_maps,
        'sequences': [
            {
                'sequence': name,
                'expected_checkpoint_pairs':
                    contract['sequences'][name]['checkpoint_count'],
                'commands': plans[name],
            }
            for name in names
        ],
    }
    plan_path = output_root / 'suite_plan.json'
    plan_path.write_text(json.dumps(plan, indent=2) + '\n', encoding='utf-8')

    results = []
    for name in names:
        print(f'\nRunning {name}...', flush=True)
        generate, benchmark = plans[name]
        subprocess.run(generate, cwd=ROOT, check=True)
        subprocess.run(benchmark, cwd=ROOT, check=True)
        results.append(summarize_run(
            name, output_root,
            contract['sequences'][name]['checkpoint_count']))

    summary = {
        'schema_version': 1,
        'suite': contract['name'],
        'profile_path': str(PROFILE),
        'profile_sha256': plan['profile_sha256'],
        'plan_path': str(plan_path),
        'dataset_root': str(dataset_root),
        'attribution': contract['attribution'],
        'sequences': results,
    }
    summary_path = output_root / 'suite_summary.json'
    summary_path.write_text(json.dumps(summary, indent=2) + '\n',
                            encoding='utf-8')
    print(f'\ndone: {summary_path}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, yaml.YAMLError,
            json.JSONDecodeError, subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        raise SystemExit(2)
