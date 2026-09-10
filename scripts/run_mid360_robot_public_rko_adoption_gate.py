#!/usr/bin/env python3
"""Run the public MID-360 RKO-LIO sweep-quality-config adoption gate."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_rko_adoption_gate import (
    RKO_ADOPTION_GATE_JSON,
    RKO_ADOPTION_GATE_MARKDOWN,
    RkoAdoptionGateOptions,
    RkoAdoptionGateRunner,
    render_rko_adoption_gate_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_public_rko_quality_report import RkoQualityGateThresholds
from lidarslam_benchmark_tools.mid360_robot_public_rko_sweep import (
    RKO_SWEEP_JSON,
    default_rko_sweep_cases,
    parse_rko_sweep_case,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPO_ROOT / 'output' / 'mid360_public' / 'rko_sweep'
DEFAULT_SWEEP = DEFAULT_OUTPUT_DIR / RKO_SWEEP_JSON
DEFAULT_CONFIG = (
    REPO_ROOT
    / 'configs'
    / 'mid360_robot'
    / 'rko_lio_mid360_low_voxel_no_deskew.yaml'
)
DEFAULT_BAG = (
    REPO_ROOT
    / 'datasets'
    / 'mid360_public_segments'
    / 'hard_pointcloud_mid360_outdoor_kidnap_a'
    / 'segment_002'
    / 'rosbag2'
)
DEFAULT_BASE_RKO_PARAM = REPO_ROOT / 'configs' / 'mid360_robot' / 'rko_lio_mid360_no_deskew.yaml'
DEFAULT_LIDARSLAM_PARAM = REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam_mid360_rko_graph.yaml'


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Run the public MID-360 RKO-LIO adoption gate.'
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        '--from-existing',
        dest='mode',
        action='store_const',
        const='existing',
        help='Use an existing sweep manifest. This is the default.',
    )
    mode.add_argument(
        '--run',
        dest='mode',
        action='store_const',
        const='run',
        help='Run the public RKO-LIO sweep before quality/adoption checks.',
    )
    parser.set_defaults(mode='existing')
    parser.add_argument('--sweep', default='', help='Existing mid360_robot_public_rko_sweep.json.')
    parser.add_argument('--output-dir', default='', help='Gate output directory.')
    parser.add_argument('--config', default=str(DEFAULT_CONFIG), help='Tracked RKO-LIO YAML.')
    parser.add_argument(
        '--allow-non-best',
        action='store_true',
        help='Accept any matching gate-pass case instead of requiring the top-ranked case.',
    )
    parser.add_argument('--bag', default=str(DEFAULT_BAG), help='rosbag2 directory for --run.')
    parser.add_argument('--base-rko-param', default=str(DEFAULT_BASE_RKO_PARAM))
    parser.add_argument('--lidarslam-param', default=str(DEFAULT_LIDARSLAM_PARAM))
    parser.add_argument('--lidar-topic', default='/livox/points')
    parser.add_argument('--imu-topic', default='/livox/imu')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--lidar-frame', default='livox_frame')
    parser.add_argument('--imu-frame', default='livox_frame')
    parser.add_argument(
        '--case',
        action='append',
        default=[],
        help='Repeatable sweep case spec used only with --run.',
    )
    parser.add_argument('--limit', type=int, default=0, help='Maximum sweep cases in --run mode.')
    parser.add_argument('--allow-existing-output', action='store_true')
    parser.add_argument('--run-timeout-sec', type=int, default=90)
    parser.add_argument('--save-timeout-secs', type=int, default=60)
    parser.add_argument('--startup-timeout-secs', type=int, default=30)
    parser.add_argument('--offline-quiet-log-secs', type=int, default=0)
    parser.add_argument('--min-trajectory-poses', type=int, default=200)
    parser.add_argument('--min-trajectory-duration-sec', type=float, default=60.0)
    parser.add_argument('--min-path-length-m', type=float, default=10.0)
    parser.add_argument('--max-step-m', type=float, default=5.0)
    parser.add_argument('--min-map-points', type=int, default=100000)
    parser.add_argument('--min-map-tiles', type=int, default=10)
    parser.add_argument('--max-runtime-sec', type=float, default=120.0)
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    output_dir = _resolve_output_dir(args)
    sweep_path = _resolve_sweep_path(args, output_dir)
    try:
        cases = tuple(parse_rko_sweep_case(spec) for spec in args.case)
        if not cases:
            cases = default_rko_sweep_cases()
        thresholds = RkoQualityGateThresholds(
            min_trajectory_poses=max(0, int(args.min_trajectory_poses)),
            min_trajectory_duration_sec=max(0.0, float(args.min_trajectory_duration_sec)),
            min_path_length_m=max(0.0, float(args.min_path_length_m)),
            max_step_m=max(0.0, float(args.max_step_m)),
            min_map_points=max(0, int(args.min_map_points)),
            min_map_tiles=max(0, int(args.min_map_tiles)),
            max_runtime_sec=max(0.0, float(args.max_runtime_sec)),
        )
        report = RkoAdoptionGateRunner(
            RkoAdoptionGateOptions(
                repo_root=REPO_ROOT,
                output_dir=output_dir,
                config_path=Path(args.config),
                sweep_path=sweep_path,
                mode=args.mode,
                require_best=not args.allow_non_best,
                thresholds=thresholds,
                bag_path=Path(args.bag),
                base_rko_param=Path(args.base_rko_param),
                lidarslam_param=Path(args.lidarslam_param),
                lidar_topic=args.lidar_topic,
                imu_topic=args.imu_topic,
                base_frame=args.base_frame,
                lidar_frame=args.lidar_frame,
                imu_frame=args.imu_frame,
                cases=cases,
                limit=max(0, int(args.limit)),
                allow_existing_output=args.allow_existing_output,
                run_timeout_sec=max(0, int(args.run_timeout_sec)),
                save_timeout_secs=max(1, int(args.save_timeout_secs)),
                startup_timeout_secs=max(1, int(args.startup_timeout_secs)),
                offline_quiet_log_secs=max(0, int(args.offline_quiet_log_secs)),
            )
        ).run()
    except Exception as exc:
        print(f'failed to run public MID-360 RKO adoption gate: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_rko_adoption_gate_markdown(report))
        artifacts = report.get('artifacts') or {}
        print(f'{RKO_ADOPTION_GATE_JSON}: {artifacts.get("gate_json", "")}')
        print(f'{RKO_ADOPTION_GATE_MARKDOWN}: {artifacts.get("gate_markdown", "")}')
    return 0 if report['status'] == 'PASS' else 1


def _resolve_output_dir(args: argparse.Namespace) -> Path:
    if args.output_dir:
        return Path(args.output_dir).expanduser().resolve()
    if args.mode == 'existing' and args.sweep:
        return Path(args.sweep).expanduser().resolve().parent
    return DEFAULT_OUTPUT_DIR


def _resolve_sweep_path(args: argparse.Namespace, output_dir: Path) -> Path:
    if args.sweep:
        return Path(args.sweep).expanduser().resolve()
    if args.mode == 'existing':
        if args.output_dir:
            return output_dir / RKO_SWEEP_JSON
        return DEFAULT_SWEEP
    return output_dir / RKO_SWEEP_JSON


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
