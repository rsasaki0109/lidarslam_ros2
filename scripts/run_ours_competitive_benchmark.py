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
    benchmark_machine_fingerprint, evaluate_map_quality, parse_time_report,
    current_competitive_closure_identity, validate_frozen_input_manifest)
from lidarslam_benchmark_tools.run_glim_benchmark import bag_bounds, sha256_tree, trajectory_info
from lidarslam_benchmark_tools.competitive_candidate_provenance import verify_candidate_manifest
import yaml


ROOT = _SCRIPT_SOURCE_ROOT
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


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


def sensor_arguments(args: argparse.Namespace) -> list[str]:
    """Return the explicitly frozen sensor interface passed to RKO-LIO."""
    return [
        '--lidar-topic', args.lidar_topic,
        '--imu-topic', args.imu_topic,
        '--base-frame', args.base_frame,
    ]


def visual_sensor_contract(rko_param: Path, camera_topic: str) -> dict[str, Any]:
    """Validate and expose the camera interface hidden inside the RKO YAML."""
    params = yaml.safe_load(rko_param.read_text()) or {}
    enabled = bool(params.get('direct_visual_frontend', False))
    configured_topic = str(params.get('visual_image_topic', '')) if enabled else ''
    if enabled and not camera_topic:
        raise ValueError(
            '--camera-topic is required when direct_visual_frontend is enabled')
    if camera_topic and not enabled:
        raise ValueError(
            '--camera-topic was supplied but direct_visual_frontend is disabled')
    if enabled and configured_topic != camera_topic:
        raise ValueError(
            f'camera topic mismatch: CLI={camera_topic}, RKO YAML={configured_topic}')
    return {
        'enabled': enabled,
        'camera_topic': configured_topic or None,
        'distortion_model': (
            params.get('visual_camera_distortion_model', 'equidistant')
            if enabled else None),
    }


def base_to_prism_contract(reference_meta: Path) -> dict[str, Any]:
    metadata = json.loads(reference_meta.read_text())
    for frame in ('base', 'body', 'imu'):
        for suffix in ('reference', 'prism'):
            offset = metadata.get(f'{frame}_to_{suffix}_translation_m')
            if isinstance(offset, dict) and all(axis in offset for axis in 'xyz'):
                return {
                    'source_frame': frame,
                    'target_frame': metadata.get(
                        'reference_point_frame',
                        'leica_prism' if suffix == 'prism' else 'reference_point'),
                    'offset_m': {axis: float(offset[axis]) for axis in 'xyz'},
                }
    raise ValueError(
        'RKO-LIO trajectory is base-frame pose but reference metadata lacks '
        'base/body/imu_to_reference_translation_m (or legacy '
        '*_to_prism_translation_m)')


def run_once(args: argparse.Namespace, output: Path, index: int,
             shared: dict[str, Any], duration: float, bag_end: float) -> dict[str, Any]:
    run_dir = output / f'run_{index:02d}'
    artifacts = run_dir / 'artifacts'
    run_dir.mkdir(parents=True, exist_ok=False)
    command = [
        '/usr/bin/time', '-v', '-o', str(run_dir / 'process_time.txt'),
        'bash', str(ROOT / 'scripts/run_rko_lio_graph_benchmark.sh'),
        '--bag', str(args.bag), *sensor_arguments(args),
        '--rko-param', str(args.rko_param),
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
    scoring_trajectory = trajectory_info(artifacts / 'traj_raw_prism.tum')
    end_gap = (None if trajectory['last_stamp'] is None
               else bag_end - trajectory['last_stamp'])
    timing = parse_time_report(run_dir / 'process_time.txt')
    metrics_path = artifacts / 'metrics.json'
    metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() else {}
    map_quality = None
    if args.save_maps and (artifacts / 'map.pcd').exists():
        map_quality = evaluate_map_quality(artifacts / 'map.pcd', run_dir)
    processing_rtf = metrics.get('lidarslam', {}).get('rtf')
    complete = (completed.returncode == 0 and trajectory['samples'] > 0 and
                scoring_trajectory['samples'] == trajectory['samples'] and
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
        'scoring_trajectory': scoring_trajectory,
        'trajectory_contract': args.trajectory_contract,
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
    parser.add_argument('--lidar-topic', default='/hesai/pandar')
    parser.add_argument('--imu-topic', default='/alphasense/imu')
    parser.add_argument('--camera-topic', default='')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--rko-param', type=Path,
                        default=ROOT / 'configs/hilti2022/rko_lio_hilti2022_pandar.yaml')
    parser.add_argument('--lidarslam-param', type=Path,
                        default=ROOT / 'lidarslam/param/lidarslam.yaml')
    args = parser.parse_args()
    for name in ('bag', 'reference_tum', 'reference_meta', 'output',
                 'rko_param', 'lidarslam_param'):
        setattr(args, name, getattr(args, name).resolve())
    visual_interface = visual_sensor_contract(args.rko_param, args.camera_topic)
    args.trajectory_contract = base_to_prism_contract(args.reference_meta)
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
        'reference_metadata_path': str(args.reference_meta),
        'reference_metadata_sha256': file_hash(args.reference_meta),
        'trajectory_contract': args.trajectory_contract,
        'sensor_interface': {
            'lidar_topic': args.lidar_topic,
            'imu_topic': args.imu_topic,
            'base_frame': args.base_frame,
            'visual': visual_interface,
        },
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
    summary = {
        'schema_version': 1, 'system': 'lidarslam_ros2', 'requested_runs': runs,
        'completed_trajectories': sum(
            report['completion']['trajectory_complete'] for report in reports),
        'clean_exits': sum(
            report['completion']['process_exit_status'] == 0 for report in reports),
        'completed_maps': sum('mapping' in report for report in reports),
        'runs': reports,
    }
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({key: summary[key] for key in (
        'requested_runs', 'completed_trajectories', 'clean_exits')}, indent=2))
    return 0 if summary['completed_trajectories'] == runs else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError,
            subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
