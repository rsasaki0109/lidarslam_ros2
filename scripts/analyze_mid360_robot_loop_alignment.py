#!/usr/bin/env python3
"""CLI for MID-360 loop-alignment cloud analysis."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_loop_alignment_analyzer import (
    LOOP_ALIGNMENT_JSON,
    LOOP_ALIGNMENT_MARKDOWN,
    LoopAlignmentOptions,
    LoopAlignmentThresholds,
    Mid360LoopAlignmentAnalyzer,
    render_loop_alignment_markdown,
    write_loop_alignment_report,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Analyze trajectory/cloud evidence for MID-360 loop alignment.'
    )
    parser.add_argument('run_dir', help='Map run output directory or pointcloud_map parent.')
    parser.add_argument('--pointcloud-map', default='', help='Override pointcloud_map directory.')
    parser.add_argument('--trajectory', default='', help='TUM trajectory path. Auto-detected when omitted.')
    parser.add_argument('--output-dir', default='', help='Directory for loop-alignment report artifacts.')
    parser.add_argument('--loop-search-radius-m', type=float, default=2.0)
    parser.add_argument('--max-loop-distance-m', type=float, default=1.0)
    parser.add_argument('--min-index-separation', type=int, default=50)
    parser.add_argument('--min-time-separation-sec', type=float, default=20.0)
    parser.add_argument('--min-loop-candidates', type=int, default=1)
    parser.add_argument('--cloud-radius-m', type=float, default=8.0)
    parser.add_argument('--voxel-size-m', type=float, default=0.5)
    parser.add_argument('--min-local-points', type=int, default=80)
    parser.add_argument('--max-connected-components', type=int, default=3)
    parser.add_argument('--min-largest-component-ratio', type=float, default=0.6)
    parser.add_argument('--max-points-per-tile', type=int, default=5000)
    parser.add_argument('--max-total-points', type=int, default=200000)
    parser.add_argument('--max-loop-candidates', type=int, default=20)
    parser.add_argument('--write', action='store_true', help='Write JSON/Markdown artifacts.')
    parser.add_argument('--json', action='store_true', help='Print machine-readable JSON.')
    return parser.parse_args()


def _options_from_args(args: argparse.Namespace) -> LoopAlignmentOptions:
    return LoopAlignmentOptions(
        run_dir=Path(args.run_dir).expanduser().resolve(),
        pointcloud_map_dir=(
            Path(args.pointcloud_map).expanduser().resolve()
            if args.pointcloud_map else None
        ),
        trajectory_path=Path(args.trajectory).expanduser().resolve() if args.trajectory else None,
        output_dir=Path(args.output_dir).expanduser().resolve() if args.output_dir else None,
        thresholds=LoopAlignmentThresholds(
            loop_search_radius_m=max(0.0, float(args.loop_search_radius_m)),
            max_loop_distance_m=max(0.0, float(args.max_loop_distance_m)),
            min_index_separation=max(1, int(args.min_index_separation)),
            min_time_separation_sec=max(0.0, float(args.min_time_separation_sec)),
            min_loop_candidates=max(0, int(args.min_loop_candidates)),
            cloud_radius_m=max(0.0, float(args.cloud_radius_m)),
            voxel_size_m=max(0.01, float(args.voxel_size_m)),
            min_local_points=max(0, int(args.min_local_points)),
            max_connected_components=max(1, int(args.max_connected_components)),
            min_largest_component_ratio=max(0.0, min(1.0, float(args.min_largest_component_ratio))),
        ),
        max_points_per_tile=max(1, int(args.max_points_per_tile)),
        max_total_points=max(1, int(args.max_total_points)),
        max_loop_candidates=max(1, int(args.max_loop_candidates)),
    )


def main() -> int:
    """Entry point."""
    args = parse_args()
    try:
        options = _options_from_args(args)
        report = Mid360LoopAlignmentAnalyzer().analyze(options)
        output_dir = options.output_dir or options.run_dir
        if args.write:
            paths = write_loop_alignment_report(report, output_dir)
        else:
            paths = {}
    except Exception as exc:
        print(f'failed to analyze MID-360 loop alignment: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_loop_alignment_markdown(report))
        if paths:
            print(f'{LOOP_ALIGNMENT_JSON}: {paths["json"]}')
            print(f'{LOOP_ALIGNMENT_MARKDOWN}: {paths["markdown"]}')
    return 1 if report['status'] == 'FAIL' else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
