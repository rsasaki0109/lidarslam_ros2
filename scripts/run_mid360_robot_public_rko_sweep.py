#!/usr/bin/env python3
"""Plan or run an RKO-LIO parameter sweep for public MID-360 bag segments."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_rko_sweep import (
    RkoSweepBuilder,
    RkoSweepOptions,
    RkoSweepRunOptions,
    default_rko_sweep_cases,
    parse_rko_sweep_case,
    render_rko_sweep_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BAG = (
    REPO_ROOT
    / 'datasets'
    / 'mid360_public_segments'
    / 'hard_pointcloud_mid360_outdoor_kidnap_a'
    / 'segment_002'
    / 'rosbag2'
)
DEFAULT_OUTPUT_DIR = REPO_ROOT / 'output' / 'mid360_public' / 'rko_sweep'
DEFAULT_BASE_RKO_PARAM = REPO_ROOT / 'configs' / 'mid360_robot' / 'rko_lio_mid360_no_deskew.yaml'
DEFAULT_LIDARSLAM_PARAM = REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam_mid360_rko_graph.yaml'


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Plan or run RKO-LIO frontend parameter sweeps on public MID-360 data.'
    )
    parser.add_argument(
        '--bag',
        default=str(DEFAULT_BAG),
        help='rosbag2 directory to run. Defaults to the clipped Hard dataset segment.',
    )
    parser.add_argument(
        '--output-dir',
        default=str(DEFAULT_OUTPUT_DIR),
        help='Sweep artifact and per-case output directory.',
    )
    parser.add_argument(
        '--base-rko-param',
        default=str(DEFAULT_BASE_RKO_PARAM),
        help='Base RKO-LIO YAML to copy before applying case overrides.',
    )
    parser.add_argument(
        '--lidarslam-param',
        default=str(DEFAULT_LIDARSLAM_PARAM),
        help='graph_based_slam parameter YAML.',
    )
    parser.add_argument('--lidar-topic', default='/livox/points')
    parser.add_argument('--imu-topic', default='/livox/imu')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--lidar-frame', default='livox_frame')
    parser.add_argument('--imu-frame', default='livox_frame')
    parser.add_argument(
        '--case',
        action='append',
        default=[],
        help=(
            'Repeatable case spec, e.g. '
            'half:voxel_size=0.5,min_range=1.0,double_downsample=true,deskew=false. '
            'Defaults to a small voxel/min-range/double-downsample sweep.'
        ),
    )
    parser.add_argument('--limit', type=int, default=0, help='Maximum cases to include.')
    parser.add_argument(
        '--allow-existing-output',
        action='store_true',
        help='Allow per-case directories with existing runtime outputs.',
    )
    parser.add_argument(
        '--run',
        action='store_true',
        help='Execute runnable cases. Default only writes configs and a sweep manifest.',
    )
    parser.add_argument(
        '--run-timeout-sec',
        type=int,
        default=90,
        help='Wrapper timeout for each case when --run is set.',
    )
    parser.add_argument(
        '--save-timeout-secs',
        type=int,
        default=60,
        help='Map-save wait timeout passed to the dogfood wrapper.',
    )
    parser.add_argument(
        '--startup-timeout-secs',
        type=int,
        default=30,
        help='Startup wait timeout passed to the dogfood wrapper.',
    )
    parser.add_argument(
        '--offline-quiet-log-secs',
        type=int,
        default=0,
        help=(
            'Treat an unchanged launch log after first odom/cloud as offline completion '
            'after N seconds. Default 0 disables this fallback.'
        ),
    )
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    try:
        cases = tuple(parse_rko_sweep_case(spec) for spec in args.case)
        if not cases:
            cases = default_rko_sweep_cases()
        options = RkoSweepOptions(
            repo_root=REPO_ROOT,
            bag_path=Path(args.bag),
            output_dir=Path(args.output_dir),
            base_rko_param=Path(args.base_rko_param),
            lidarslam_param=Path(args.lidarslam_param),
            lidar_topic=args.lidar_topic,
            imu_topic=args.imu_topic,
            base_frame=args.base_frame,
            lidar_frame=args.lidar_frame,
            imu_frame=args.imu_frame,
            save_timeout_secs=max(1, int(args.save_timeout_secs)),
            startup_timeout_secs=max(1, int(args.startup_timeout_secs)),
            offline_quiet_log_secs=max(0, int(args.offline_quiet_log_secs)),
            allow_existing_output=args.allow_existing_output,
            limit=max(0, int(args.limit)),
        )
        builder = RkoSweepBuilder(options=options, cases=cases)
        manifest = builder.build(
            run=args.run,
            run_options=RkoSweepRunOptions(timeout_sec=max(0, int(args.run_timeout_sec))),
        )
        paths = builder.write(manifest)
    except Exception as exc:
        print(f'failed to build public MID-360 RKO sweep: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(manifest))
    else:
        print(render_rko_sweep_markdown(manifest))
        print(f"RKO sweep JSON: {paths['json']}")
        print(f"RKO sweep Markdown: {paths['markdown']}")
    return 1 if manifest['status'] in ('FAIL', 'BLOCKED') else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
