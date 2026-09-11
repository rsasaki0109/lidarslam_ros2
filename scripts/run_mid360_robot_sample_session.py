#!/usr/bin/env python3
"""CLI for running a synthetic MID-360 robot field session."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_sample_session import (
    Mid360SampleSessionRunner,
    SAMPLE_SESSION_SCENARIOS,
    SampleSessionOptions,
    render_sample_session_markdown,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = REPO_ROOT / 'configs' / 'mid360_robot' / 'livox_mid360_default.yaml'


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Generate a MID-360 sample bag and run the normal session reports.'
    )
    parser.add_argument('--robot-profile', default=str(DEFAULT_PROFILE))
    parser.add_argument(
        '--bag-root',
        default=str(REPO_ROOT / 'output' / 'mid360_robot_sample_session' / 'bags'),
        help='Directory where sample bag and recording sidecars are written.',
    )
    parser.add_argument('--run-id', default='mid360_sample_session')
    parser.add_argument(
        '--output-dir',
        default=str(REPO_ROOT / 'output' / 'mid360_robot_sample_session'),
        help='Directory for sample session reports and dashboard.',
    )
    parser.add_argument('--duration-sec', type=float, default=5.0)
    parser.add_argument('--pointcloud-rate-hz', type=float, default=10.0)
    parser.add_argument('--imu-rate-hz', type=float, default=100.0)
    parser.add_argument('--point-count', type=int, default=32)
    parser.add_argument(
        '--scenario',
        choices=SAMPLE_SESSION_SCENARIOS,
        default='pass',
        help='Synthetic QA scenario to generate.',
    )
    parser.add_argument('--force', action='store_true', help='Overwrite an existing sample bag.')
    parser.add_argument('--json', action='store_true', help='Print sample session JSON.')
    return parser.parse_args()


def options_from_args(args: argparse.Namespace) -> SampleSessionOptions:
    return SampleSessionOptions(
        profile_path=Path(args.robot_profile),
        bag_root=Path(args.bag_root),
        output_dir=Path(args.output_dir),
        run_id=args.run_id,
        duration_sec=args.duration_sec,
        pointcloud_rate_hz=args.pointcloud_rate_hz,
        imu_rate_hz=args.imu_rate_hz,
        point_count=args.point_count,
        scenario=args.scenario,
        force=args.force,
    )


def main() -> int:
    args = parse_args()
    try:
        report = Mid360SampleSessionRunner(REPO_ROOT).run(options_from_args(args))
    except Exception as exc:
        print(f'failed to run MID-360 sample session: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(render_sample_session_markdown(report))
        print(f"Session dashboard HTML: {report['dashboard_html_path']}")
    return 1 if report['status'] == 'FAIL' else 0


if __name__ == '__main__':
    raise SystemExit(main())
