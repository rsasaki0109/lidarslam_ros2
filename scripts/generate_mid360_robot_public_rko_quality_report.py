#!/usr/bin/env python3
"""Generate a quality dashboard from a public MID-360 RKO-LIO sweep manifest."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_rko_quality_report import (
    RKO_QUALITY_HTML,
    RKO_QUALITY_JSON,
    RKO_QUALITY_MARKDOWN,
    RkoQualityGateThresholds,
    RkoQualityReportBuilder,
    render_rko_quality_markdown,
    write_rko_quality_report,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SWEEP = (
    REPO_ROOT
    / 'output'
    / 'mid360_public'
    / 'rko_sweep'
    / 'mid360_robot_public_rko_sweep.json'
)


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Build a map and trajectory quality report for a MID-360 public RKO sweep.'
    )
    parser.add_argument(
        '--sweep',
        default=str(DEFAULT_SWEEP),
        help='Path to mid360_robot_public_rko_sweep.json.',
    )
    parser.add_argument(
        '--output-dir',
        default='',
        help='Directory for quality report outputs. Defaults to the sweep manifest directory.',
    )
    parser.add_argument(
        '--min-trajectory-poses',
        type=int,
        default=200,
        help='Minimum valid TUM trajectory poses for the quality gate.',
    )
    parser.add_argument(
        '--min-trajectory-duration-sec',
        type=float,
        default=60.0,
        help='Minimum trajectory duration covered by the output TUM trajectory.',
    )
    parser.add_argument(
        '--min-path-length-m',
        type=float,
        default=10.0,
        help='Minimum trajectory path length for the quality gate.',
    )
    parser.add_argument(
        '--max-step-m',
        type=float,
        default=5.0,
        help='Maximum adjacent trajectory step for the quality gate.',
    )
    parser.add_argument(
        '--min-map-points',
        type=int,
        default=100000,
        help='Minimum total point count across referenced pointcloud_map tiles.',
    )
    parser.add_argument(
        '--min-map-tiles',
        type=int,
        default=10,
        help='Minimum referenced pointcloud_map tile count for the quality gate.',
    )
    parser.add_argument(
        '--max-runtime-sec',
        type=float,
        default=120.0,
        help='Maximum wrapper runtime for the quality gate.',
    )
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    sweep_path = Path(args.sweep).expanduser().resolve()
    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir else sweep_path.parent
    )
    try:
        thresholds = RkoQualityGateThresholds(
            min_trajectory_poses=max(0, int(args.min_trajectory_poses)),
            min_trajectory_duration_sec=max(0.0, float(args.min_trajectory_duration_sec)),
            min_path_length_m=max(0.0, float(args.min_path_length_m)),
            max_step_m=max(0.0, float(args.max_step_m)),
            min_map_points=max(0, int(args.min_map_points)),
            min_map_tiles=max(0, int(args.min_map_tiles)),
            max_runtime_sec=max(0.0, float(args.max_runtime_sec)),
        )
        report = RkoQualityReportBuilder(
            sweep_path=sweep_path,
            thresholds=thresholds,
        ).build_report()
        paths = write_rko_quality_report(report, output_dir)
    except Exception as exc:
        print(f'failed to generate public MID-360 RKO quality report: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_rko_quality_markdown(report))
        print(f'{RKO_QUALITY_JSON}: {paths["json"]}')
        print(f'{RKO_QUALITY_MARKDOWN}: {paths["markdown"]}')
        print(f'{RKO_QUALITY_HTML}: {paths["html"]}')
    return 1 if report['status'] in ('EMPTY',) else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
