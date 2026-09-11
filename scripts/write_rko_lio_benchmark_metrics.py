#!/usr/bin/env python3

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any

from benchmark_provenance import bag_identity, file_identity, software_identity

import yaml


REPO_ROOT = Path(__file__).resolve().parent.parent
VERIFY_SCRIPT = REPO_ROOT / 'scripts' / 'verify_autoware_map.py'


def _load_verify_module():
    spec = importlib.util.spec_from_file_location(
        'verify_autoware_map',
        VERIFY_SCRIPT,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError('failed to load verify_autoware_map.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


VERIFY_MODULE = _load_verify_module()
MapVerifier = VERIFY_MODULE.MapVerifier


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _read_pose_count(path: Path) -> int:
    if not path.is_file():
        return 0
    return sum(
        1
        for line in path.read_text(encoding='utf-8', errors='replace').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    )


def _parse_ape_report(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    metrics: dict[str, Any] = {'path': str(path)}
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if ':' not in line:
            continue
        key, raw_value = line.split(':', 1)
        key = key.strip()
        value = raw_value.strip()
        if key == 'alignment':
            metrics[key] = value
            continue
        try:
            numeric = float(value)
        except ValueError:
            continue
        if key == 'pairs':
            metrics[key] = int(numeric)
        else:
            metrics[key] = numeric
    return metrics


def _bag_duration_seconds(metadata_path: Path) -> float | None:
    if not metadata_path.is_file():
        return None
    lines = metadata_path.read_text(encoding='utf-8', errors='replace').splitlines()
    in_duration = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('duration:'):
            in_duration = True
            continue
        if in_duration and stripped.startswith('nanoseconds:'):
            try:
                nanoseconds = int(stripped.split(':', 1)[1].strip())
            except ValueError:
                return None
            return nanoseconds / 1e9
        if in_duration and stripped and not line.startswith(' '):
            break
    return None


def _bag_topic_message_count(metadata_path: Path, topic: str) -> int | None:
    """Return the recorded message count for a topic from rosbag2 metadata."""
    if not metadata_path.is_file():
        return None
    try:
        data = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
        topics = data['rosbag2_bagfile_information']['topics_with_message_count']
        for entry in topics:
            metadata = entry.get('topic_metadata', {})
            if metadata.get('name') == topic:
                return int(entry['message_count'])
    except (KeyError, TypeError, ValueError, yaml.YAMLError):
        return None
    return None


def _read_reference_meta(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    return _read_json(path)


def _verify_map(pointcloud_dir: Path) -> dict[str, Any] | None:
    if not pointcloud_dir.is_dir():
        return None
    verifier = MapVerifier(str(pointcloud_dir))
    ok = verifier.run()
    return {
        'ok': ok,
        'passes': verifier.passes,
        'warnings': verifier.warnings,
        'failures': verifier.failures,
    }


def _fmt_path(path: Path | None) -> str:
    return str(path) if path is not None else ''


def _trajectory_offset(metadata: dict[str, Any], frame: str) -> dict[str, Any] | None:
    """Return the generic reference-point offset, with legacy prism fallback."""
    return (
        metadata.get(f'{frame}_to_reference_translation_m')
        or metadata.get(f'{frame}_to_prism_translation_m')
    )


def _infer_reference_kind(source: str, metadata: dict[str, Any]) -> str:
    explicit = metadata.get('kind')
    if explicit:
        return str(explicit)
    lowered = source.strip().lower()
    if 'gt' in lowered or 'ground_truth' in lowered:
        return 'ground_truth'
    if 'glim' in lowered or 'cross' in lowered:
        return 'cross_validation'
    return 'unknown'


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Write a metrics.json for an RKO-LIO + graph_based_slam benchmark '
            'run so it can be consumed by the repo reporting tools.'
        ),
    )
    parser.add_argument('--out-dir', required=True, help='Benchmark output directory')
    parser.add_argument('--bag', required=True, help='rosbag2 directory used for the run')
    parser.add_argument('--reference-tum', required=True, help='Reference TUM trajectory')
    parser.add_argument(
        '--reference-meta',
        default='',
        help='Optional JSON sidecar emitted by generate_ntu_viral_tnp01_reference.py',
    )
    parser.add_argument(
        '--reference-source',
        default='leica_prism_gt',
        help='Reference source label stored in metrics.json',
    )
    parser.add_argument(
        '--trajectory-source-frame',
        default='base',
        choices=('base', 'body', 'imu', 'lidar'),
        help='Local frame represented by the input trajectory pose origin.',
    )
    parser.add_argument(
        '--points-topic',
        default='/os1_cloud_node1/points',
        help='LiDAR topic used for the run',
    )
    parser.add_argument(
        '--imu-topic',
        default='/imu/imu',
        help='IMU topic used for the run',
    )
    parser.add_argument(
        '--lidarslam-param',
        default='lidarslam/param/lidarslam.yaml',
        help='graph_based_slam parameter YAML',
    )
    parser.add_argument(
        '--rko-param',
        default='lidarslam/param/rko_lio_ntu_viral.yaml',
        help='RKO-LIO parameter YAML',
    )
    parser.add_argument(
        '--run-name',
        default='',
        help='RKO-LIO run name tag',
    )
    parser.add_argument(
        '--raw-tum',
        default='',
        help='Raw odometry TUM path (default: auto-detect in out-dir)',
    )
    parser.add_argument(
        '--corrected-tum',
        default='',
        help='Corrected path TUM path (default: auto-detect in out-dir)',
    )
    parser.add_argument(
        '--raw-ape',
        default='',
        help='Raw APE report path (default: auto-detect in out-dir)',
    )
    parser.add_argument(
        '--corrected-ape',
        default='',
        help='Corrected APE report path (default: auto-detect in out-dir)',
    )
    parser.add_argument(
        '--launch-log',
        default='',
        help='Launch log path (default: <out-dir>/slam.launch.log)',
    )
    parser.add_argument(
        '--wall-sec',
        type=float,
        default=None,
        help='Measured wall time for the full benchmark run',
    )
    parser.add_argument(
        '--completion-reason',
        default='',
        help='Authoritative reason the replay was considered complete.',
    )
    parser.add_argument(
        '--completion-end-margin-secs',
        type=float,
        default=None,
        help='Allowed trajectory-to-bag-end gap for completion fallback.',
    )
    parser.add_argument(
        '--started-at',
        default='',
        help='Optional ISO-8601 start timestamp',
    )
    parser.add_argument(
        '--started-at-unix',
        type=int,
        default=None,
        help='Optional unix start timestamp',
    )
    parser.add_argument(
        '--metrics-out',
        default='',
        help='Output metrics path (default: <out-dir>/metrics.json)',
    )
    parser.add_argument(
        '--skip-map-verify',
        action='store_true',
        help=(
            'Do not inspect pointcloud_map, even if a partial directory exists. '
            'Use for trajectory-only runs that intentionally skip map_save.'
        ),
    )
    parser.add_argument(
        '--parameter-file',
        action='append',
        default=[],
        help='Effective parameter file to identify; repeat for multiple files.',
    )
    parser.add_argument(
        '--runtime-artifact',
        action='append',
        default=[],
        metavar='LABEL=PATH',
        help='Runtime executable/library to identify; repeat for multiple artifacts.',
    )
    parser.add_argument(
        '--benchmark-harness',
        default='',
        help='Benchmark wrapper/script to identify for release provenance.',
    )
    parser.add_argument(
        '--pipeline',
        default='rko_lio',
        choices=('rko_lio', 'lo', 'small_gicp'),
        help=(
            'rko_lio: RKO-LIO frontend; '
            'lo: scanmatcher LiDAR-only frontend; '
            'small_gicp: small_gicp ICP/GICP odometry frontend.'
        ),
    )
    parser.add_argument(
        '--robot-frame-id',
        default='base_link',
        help='Robot frame label stored in metrics (LO / visualization).',
    )
    parser.add_argument(
        '--raw-path-topic',
        default='/path',
        help='Scanmatcher Path topic (LO pipeline).',
    )
    parser.add_argument(
        '--corrected-path-topic',
        default='/modified_path',
        help='Graph Path topic (LO pipeline).',
    )
    args = parser.parse_args()

    out_dir = Path(args.out_dir).expanduser().resolve()
    bag_path = Path(args.bag).expanduser().resolve()
    reference_tum = Path(args.reference_tum).expanduser().resolve()
    reference_meta = (
        Path(args.reference_meta).expanduser().resolve()
        if args.reference_meta else None
    )
    lidarslam_param = Path(args.lidarslam_param).expanduser().resolve()
    rko_param = Path(args.rko_param).expanduser().resolve()
    metrics_path = (
        Path(args.metrics_out).expanduser().resolve()
        if args.metrics_out else out_dir / 'metrics.json'
    )

    raw_tum = (
        Path(args.raw_tum).expanduser().resolve()
        if args.raw_tum else out_dir / 'traj_raw_prism.tum'
    )
    corrected_tum = (
        Path(args.corrected_tum).expanduser().resolve()
        if args.corrected_tum else out_dir / 'traj_corrected_prism.tum'
    )
    if not raw_tum.is_file():
        raw_tum = out_dir / 'traj_raw.tum'
    if not corrected_tum.is_file():
        corrected_tum = out_dir / 'traj_corrected.tum'

    raw_ape_path = (
        Path(args.raw_ape).expanduser().resolve()
        if args.raw_ape else out_dir / 'ape_raw_vs_gt.txt'
    )
    corrected_ape_path = (
        Path(args.corrected_ape).expanduser().resolve()
        if args.corrected_ape else out_dir / 'ape_corrected_vs_gt.txt'
    )
    launch_log = (
        Path(args.launch_log).expanduser().resolve()
        if args.launch_log else out_dir / 'slam.launch.log'
    )

    bag_duration_sec = _bag_duration_seconds(bag_path / 'metadata.yaml')
    input_points_messages = _bag_topic_message_count(
        bag_path / 'metadata.yaml', args.points_topic)
    reference_meta_data = _read_reference_meta(reference_meta) if reference_meta else {}
    trajectory_offset = _trajectory_offset(
        reference_meta_data, args.trajectory_source_frame)
    reference_source = reference_meta_data.get('source', args.reference_source)
    raw_ape = _parse_ape_report(raw_ape_path)
    corrected_ape = _parse_ape_report(corrected_ape_path)
    map_verify = (
        None if args.skip_map_verify
        else _verify_map(out_dir / 'pointcloud_map')
    )

    corrected_success = corrected_tum.is_file() and corrected_ape is not None
    raw_success = raw_tum.is_file() and raw_ape is not None
    wall_sec = args.wall_sec
    rtf = None
    if wall_sec is not None and bag_duration_sec and bag_duration_sec > 0.0:
        rtf = wall_sec / bag_duration_sec
    raw_output_pose_count = _read_pose_count(raw_tum)
    raw_output_pose_ratio = (
        raw_output_pose_count / input_points_messages
        if input_points_messages and input_points_messages > 0 else None
    )

    if args.pipeline in ('lo', 'small_gicp'):
        frames: dict[str, str] = {
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'robot_frame_id': args.robot_frame_id,
            'points_frame_id': args.robot_frame_id,
        }
    else:
        frames = {
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'robot_frame_id': 'os_sensor',
            'points_frame_id': 'os_sensor',
        }

    metrics: dict[str, Any] = {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/benchmark-metrics-v1.schema.json'
        ),
        'started_at': args.started_at or None,
        'started_at_unix': args.started_at_unix,
        'pipeline': args.pipeline,
        'out_dir': str(out_dir),
        'bag_path': str(bag_path),
        'bag_duration_sec': bag_duration_sec,
        'points_topic': args.points_topic,
        'imu_topic': args.imu_topic,
        'completion': {
            'reason': args.completion_reason or None,
            'end_margin_sec': args.completion_end_margin_secs,
            'input_points_messages': input_points_messages,
            'raw_output_pose_count': raw_output_pose_count,
            'raw_output_pose_ratio': raw_output_pose_ratio,
        },
        'frames': frames,
        'reference': {
            'source': reference_source,
            'kind': _infer_reference_kind(reference_source, reference_meta_data),
            'tum_path': str(reference_tum),
            'topic': reference_meta_data.get('topic', '/leica/pose/relative'),
            'meta_path': _fmt_path(reference_meta),
            'source_bag': reference_meta_data.get('source_bag', ''),
        },
        'lidarslam': {
            'success': corrected_success or raw_success,
            'wall_sec': wall_sec,
            'rtf': rtf,
            'tum_path': str(corrected_tum if corrected_tum.is_file() else raw_tum),
            'tum_lines': _read_pose_count(
                corrected_tum if corrected_tum.is_file() else raw_tum),
            'log_path': str(launch_log) if launch_log.is_file() else '',
            'param_path': str(lidarslam_param),
            'out_dir': str(out_dir),
        },
        'glim': {
            'available': False,
            'success': False,
            'reference_source': reference_meta_data.get('source', args.reference_source),
        },
        'rko_lio': (
            {
                'available': False,
                'note': f'{args.pipeline} pipeline does not use RKO-LIO.',
            }
            if args.pipeline != 'rko_lio'
            else {
                'available': True,
                'run_name': args.run_name or out_dir.name,
                'param_path': str(rko_param),
                'raw_tum_path': str(raw_tum) if raw_tum.is_file() else '',
                'raw_tum_lines': _read_pose_count(raw_tum),
                'raw_ape': raw_ape,
                'corrected_tum_path': str(corrected_tum) if corrected_tum.is_file() else '',
                'corrected_tum_lines': _read_pose_count(corrected_tum),
                'corrected_ape': corrected_ape,
                'reference_meta_path': _fmt_path(reference_meta),
                'trajectory_source_frame': args.trajectory_source_frame,
                'trajectory_to_prism_offset_m': trajectory_offset,
                'prism_offset_m': trajectory_offset,
            }
        ),
        'scanmatcher_lo': (
            {
                'lidarslam_param_path': str(lidarslam_param),
                'raw_path_topic': args.raw_path_topic,
                'corrected_path_topic': args.corrected_path_topic,
                'raw_tum_path': str(raw_tum) if raw_tum.is_file() else '',
                'raw_tum_lines': _read_pose_count(raw_tum),
                'raw_ape': raw_ape,
                'corrected_tum_path': str(corrected_tum) if corrected_tum.is_file() else '',
                'corrected_tum_lines': _read_pose_count(corrected_tum),
                'corrected_ape': corrected_ape,
                'reference_meta_path': _fmt_path(reference_meta),
                'prism_offset_m': reference_meta_data.get('lidar_to_prism_translation_m'),
            }
            if args.pipeline == 'lo'
            else {
                'available': False,
            }
        ),
        'small_gicp_lo': (
            {
                'available': True,
                'frontend_param_path': str(rko_param),
                'raw_odom_topic': '/odom',
                'raw_tum_path': str(raw_tum) if raw_tum.is_file() else '',
                'raw_tum_lines': _read_pose_count(raw_tum),
                'raw_ape': raw_ape,
                'corrected_tum_path': str(corrected_tum) if corrected_tum.is_file() else '',
                'corrected_tum_lines': _read_pose_count(corrected_tum),
                'corrected_ape': corrected_ape,
                'reference_meta_path': _fmt_path(reference_meta),
                'prism_offset_m': reference_meta_data.get('lidar_to_prism_translation_m'),
            }
            if args.pipeline == 'small_gicp'
            else {
                'available': False,
            }
        ),
        'graph_based_slam': {
            'corrected_path_available': corrected_tum.is_file(),
            'map_projector_info_path': str(out_dir / 'map_projector_info.yaml')
            if (
                not args.skip_map_verify
                and (out_dir / 'map_projector_info.yaml').is_file()
            ) else '',
            'pointcloud_map_dir': str(out_dir / 'pointcloud_map')
            if (
                not args.skip_map_verify
                and (out_dir / 'pointcloud_map').is_dir()
            ) else '',
            'map_verify': map_verify,
        },
        'evo': {
            'ape_log_path': str(corrected_ape_path) if corrected_ape_path.is_file() else '',
            'ape': corrected_ape,
            'raw_ape_log_path': str(raw_ape_path) if raw_ape_path.is_file() else '',
            'raw_ape': raw_ape,
        },
    }

    if args.runtime_artifact:
        runtime_artifacts: list[tuple[str, Path]] = []
        for value in args.runtime_artifact:
            label, separator, path = value.partition('=')
            if not separator or not label or not path:
                parser.error('--runtime-artifact must be LABEL=PATH')
            runtime_artifacts.append(
                (label, Path(path).expanduser().resolve()))
        parameter_files = [
            Path(path).expanduser().resolve()
            for path in (
                args.parameter_file
                or [str(lidarslam_param), str(rko_param)]
            )
        ]
        harness_path = (
            Path(args.benchmark_harness).expanduser().resolve()
            if args.benchmark_harness else Path(__file__).resolve()
        )
        metrics['provenance'] = {
            'input': {
                'bag': bag_identity(bag_path),
                'reference_trajectory': file_identity(reference_tum),
            },
            'software': software_identity(
                REPO_ROOT,
                parameter_files=parameter_files,
                runtime_artifacts=runtime_artifacts,
                benchmark_harness=harness_path,
                metrics_writer=Path(__file__).resolve(),
            ),
        }

    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    metrics_path.write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    print(metrics_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
