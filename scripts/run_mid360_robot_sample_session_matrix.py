#!/usr/bin/env python3
"""CLI for the MID-360 sample-session QA matrix."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_sample_session_matrix import (
    DEFAULT_MATRIX_SCENARIOS,
    Mid360SampleSessionMatrixRunner,
    SampleSessionMatrixOptions,
    render_matrix_markdown,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = REPO_ROOT / 'configs' / 'mid360_robot' / 'livox_mid360_default.yaml'


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run all MID-360 sample-session QA scenarios and summarize expected statuses.'
    )
    parser.add_argument('--robot-profile', default=str(DEFAULT_PROFILE))
    parser.add_argument(
        '--bag-root',
        default=str(REPO_ROOT / 'output' / 'mid360_robot_sample_session_matrix' / 'bags'),
    )
    parser.add_argument(
        '--output-dir',
        default=str(REPO_ROOT / 'output' / 'mid360_robot_sample_session_matrix'),
    )
    parser.add_argument('--run-id-prefix', default='mid360_matrix')
    parser.add_argument(
        '--scenario',
        action='append',
        default=[],
        help='Scenario to run. Can be passed more than once. Defaults to the full matrix.',
    )
    parser.add_argument('--duration-sec', type=float, default=5.0)
    parser.add_argument('--pointcloud-rate-hz', type=float, default=10.0)
    parser.add_argument('--imu-rate-hz', type=float, default=100.0)
    parser.add_argument('--point-count', type=int, default=32)
    parser.add_argument('--force', action='store_true', help='Overwrite existing sample bags.')
    parser.add_argument('--json', action='store_true', help='Print matrix JSON.')
    return parser.parse_args()


def options_from_args(args: argparse.Namespace) -> SampleSessionMatrixOptions:
    return SampleSessionMatrixOptions(
        profile_path=Path(args.robot_profile),
        bag_root=Path(args.bag_root),
        output_dir=Path(args.output_dir),
        run_id_prefix=args.run_id_prefix,
        scenarios=tuple(args.scenario) if args.scenario else DEFAULT_MATRIX_SCENARIOS,
        duration_sec=args.duration_sec,
        pointcloud_rate_hz=args.pointcloud_rate_hz,
        imu_rate_hz=args.imu_rate_hz,
        point_count=args.point_count,
        force=args.force,
    )


def main() -> int:
    args = parse_args()
    try:
        report = Mid360SampleSessionMatrixRunner(REPO_ROOT).run(options_from_args(args))
    except Exception as exc:
        print(f'failed to run MID-360 sample session matrix: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(render_matrix_markdown(report))
    return 1 if report['status'] == 'FAIL' else 0


if __name__ == '__main__':
    raise SystemExit(main())
