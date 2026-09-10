#!/usr/bin/env python3
"""Run repeated in-workspace RKO-LIO + graph SLAM competition baselines."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

# Direct ``python scripts/<tool>.py`` execution puts only ``scripts/`` on
# sys.path.  Add the checkout root so the canonical source package can project
# that scripts tree as ``lidarslam_benchmark_tools``.
_SCRIPT_SOURCE_ROOT = Path(__file__).resolve().parent.parent
if (
        (_SCRIPT_SOURCE_ROOT / 'lidarslam_benchmark_tools' / '__init__.py').is_file()
        and str(_SCRIPT_SOURCE_ROOT) not in sys.path):
    sys.path.insert(0, str(_SCRIPT_SOURCE_ROOT))

from lidarslam_benchmark_tools.run_fast_livo2_benchmark import (
    benchmark_machine_fingerprint, parse_time_report,
    current_competitive_closure_identity, validate_frozen_input_manifest)
from lidarslam_benchmark_tools.run_glim_benchmark import bag_bounds, sha256_tree, trajectory_info
from lidarslam_benchmark_tools.competitive_candidate_provenance import verify_candidate_manifest
import yaml


ROOT = _SCRIPT_SOURCE_ROOT
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def trajectory_determinism(reports: list[dict[str, Any]]) -> dict[str, Any]:
    """Summarize exact frontend trajectory identity across completed runs."""
    hashes = [report['trajectory'].get('sha256') for report in reports]
    comparable = bool(reports) and all(hashes)
    return {
        'comparable_runs': len(hashes) if comparable else 0,
        'trajectory_byte_identical': comparable and len(set(hashes)) == 1,
        'trajectory_sha256_by_run': hashes,
    }


def git_provenance() -> dict[str, Any]:
    revision = subprocess.run(
        ['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], check=True,
        text=True, capture_output=True).stdout.strip()
    diff = subprocess.run(
        ['git', '-C', str(ROOT), 'diff', '--binary', 'HEAD'], check=True,
        capture_output=True).stdout
    status = subprocess.run(
        ['git', '-C', str(ROOT), 'status', '--porcelain'], check=True,
        text=True, capture_output=True).stdout
    return {'revision': revision, 'dirty': bool(status.strip()),
            'diff_sha256': hashlib.sha256(diff).hexdigest()}


def run_once(args: argparse.Namespace, output: Path, index: int,
             shared: dict[str, Any], duration: float, bag_end: float) -> dict[str, Any]:
    run_dir = output / f'run_{index:02d}'
    artifacts = run_dir / 'artifacts'
    run_dir.mkdir(parents=True, exist_ok=False)
    command = [
        '/usr/bin/time', '-v', '-o', str(run_dir / 'process_time.txt'),
        'bash', str(ROOT / 'scripts/run_rko_lio_graph_benchmark.sh'),
        '--bag', str(args.bag), '--lidar-topic', '/hesai/pandar',
        '--imu-topic', '/alphasense/imu', '--rko-param', str(args.rko_param),
        '--lidarslam-param', str(args.lidarslam_param),
        '--reference-tum', str(args.reference_tum),
        '--reference-meta', str(args.reference_meta), '--skip-reference-gen',
        '--reference-source', args.reference_source,
        '--quiescence-secs', str(args.quiescence_secs),
        '--completion-end-margin-secs', '0.25',
        '--offline-timeout-secs', str(args.offline_timeout_secs),
        '--save-timeout-secs', str(args.save_timeout_secs),
        '--run-name', f'competitive_run_{index:02d}',
        '--output-dir', str(artifacts)]
    if not args.save_maps:
        command.append('--skip-map-save')
    with (run_dir / 'stdout.log').open('w') as stdout, \
            (run_dir / 'stderr.log').open('w') as stderr:
        completed = subprocess.run(command, cwd=ROOT, stdout=stdout,
                                   stderr=stderr, check=False)
    trajectory = trajectory_info(artifacts / 'traj_raw.tum')
    trajectory['sha256'] = (
        file_hash(artifacts / 'traj_raw.tum')
        if (artifacts / 'traj_raw.tum').is_file() else None)
    end_gap = (None if trajectory['last_stamp'] is None
               else bag_end - trajectory['last_stamp'])
    timing = parse_time_report(run_dir / 'process_time.txt')
    metrics_path = artifacts / 'metrics.json'
    metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() else {}
    map_quality = None
    if args.save_maps and (artifacts / 'map.pcd').exists():
        map_quality_dir = run_dir / 'map_quality'
        map_command = [
            'bash', str(ROOT / 'scripts/run_map_quality_check.sh'),
            '--input', str(artifacts / 'map.pcd'),
            '--output-dir', str(map_quality_dir), '--runs', '1',
            '--downsample', '0.1', '--setup', str(ROOT / 'install/setup.bash')]
        map_completed = subprocess.run(
            map_command, cwd=ROOT, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, check=False)
        report_path = map_quality_dir / 'run1/map_quality_report.yaml'
        if map_completed.returncode == 0 and report_path.exists():
            map_quality = yaml.safe_load(report_path.read_text())['map_quality_report']
    processing_rtf = metrics.get('lidarslam', {}).get('rtf')
    complete = (completed.returncode == 0 and trajectory['samples'] > 0 and
                end_gap is not None and end_gap <= 0.25)
    if args.save_maps and map_quality is None:
        complete = False
    report = {
        'schema_version': 1, 'system': 'lidarslam_ros2', 'run_index': index,
        'provenance': shared,
        'execution': {'process_exit_status': completed.returncode},
        'completion': {'trajectory_complete': complete,
                       'trajectory_end_gap_seconds': end_gap,
                       'process_exit_status': completed.returncode},
        'trajectory': trajectory,
        'runtime': {'bag_duration_seconds': duration,
                    'processing_realtime_factor': processing_rtf, **timing},
    }
    if map_quality is not None:
        report['mapping'] = map_quality
    (run_dir / 'run.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--reference-tum', type=Path, required=True)
    parser.add_argument('--reference-meta', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--input-manifest', type=Path)
    parser.add_argument('--candidate-manifest', type=Path)
    parser.add_argument('--reference-source', default='hilti2022_common_gt')
    parser.add_argument('--quiescence-secs', type=int, default=120)
    parser.add_argument('--offline-timeout-secs', type=int, default=3600)
    parser.add_argument('--save-timeout-secs', type=int, default=600)
    parser.add_argument('--save-maps', action='store_true')
    parser.add_argument('--runs', type=int)
    parser.add_argument('--rko-param', type=Path,
                        default=ROOT / 'configs/hilti2022/rko_lio_hilti2022_pandar.yaml')
    parser.add_argument('--lidarslam-param', type=Path,
                        default=ROOT / 'lidarslam/param/lidarslam.yaml')
    args = parser.parse_args()
    for name in ('bag', 'reference_tum', 'reference_meta', 'output',
                 'rko_param', 'lidarslam_param'):
        setattr(args, name, getattr(args, name).resolve())
    contract = yaml.safe_load(args.profile.read_text())['competitive_slam_profile']
    closure_identity = current_competitive_closure_identity(contract)
    runs = contract['repetitions'] if args.runs is None else args.runs
    if runs < 1 or not (args.bag / 'metadata.yaml').exists():
        raise ValueError('positive runs and a ROS2 bag directory are required')
    args.output.mkdir(parents=True, exist_ok=False)
    start, end = bag_bounds(args.bag / 'metadata.yaml')
    bag_hash = sha256_tree(args.bag)
    manifest = validate_frozen_input_manifest(
        args.input_manifest, 'canonical_rosbag2_tree_sha256', bag_hash)
    candidate = (None if args.candidate_manifest is None else
                 verify_candidate_manifest(ROOT, args.candidate_manifest))
    shared = {
        'profile': contract['name'], 'repository': git_provenance(),
        'rival_source_closure': closure_identity,
        'machine': benchmark_machine_fingerprint(),
        'bag_path': str(args.bag), 'bag_sha256': bag_hash,
        'input_manifest': manifest,
        'candidate_manifest': candidate,
        'reference_path': str(args.reference_tum),
        'reference_sha256': file_hash(args.reference_tum),
        'rko_param_path': str(args.rko_param),
        'rko_param_sha256': file_hash(args.rko_param),
        'lidarslam_param_path': str(args.lidarslam_param),
        'lidarslam_param_sha256': file_hash(args.lidarslam_param),
        'completion_end_margin_seconds': 0.25,
    }
    reports = []
    for index in range(1, runs + 1):
        print(f'lidarslam_ros2 repetition {index}/{runs}', flush=True)
        reports.append(run_once(args, args.output, index, shared, end - start, end))
    determinism = trajectory_determinism(reports)
    summary = {
        'schema_version': 1, 'system': 'lidarslam_ros2', 'requested_runs': runs,
        'completed_trajectories': sum(
            report['completion']['trajectory_complete'] for report in reports),
        'clean_exits': sum(
            report['completion']['process_exit_status'] == 0 for report in reports),
        'determinism': determinism,
        'runs': reports,
    }
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({key: summary[key] for key in (
        'requested_runs', 'completed_trajectories', 'clean_exits',
        'determinism')}, indent=2))
    return 0 if (summary['completed_trajectories'] == runs and
                 determinism['trajectory_byte_identical']) else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError,
            subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
