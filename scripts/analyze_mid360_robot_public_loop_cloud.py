#!/usr/bin/env python3
"""CLI for public MID-360 GT loop cloud-overlap analysis."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_loop_cloud_analyzer import (
    PUBLIC_LOOP_CLOUD_ANALYSIS_JSON,
    PUBLIC_LOOP_CLOUD_ANALYSIS_MARKDOWN,
    LoopCloudAnalysisOptions,
    PublicLoopCloudAnalyzer,
    render_public_loop_cloud_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOOP_CANDIDATES = (
    REPO_ROOT
    / 'output'
    / 'mid360_public'
    / 'loop_candidates'
    / 'mid360_robot_public_loop_candidates.json'
)
DEFAULT_GT_ZIP = (
    REPO_ROOT
    / 'datasets'
    / 'mid360_public'
    / 'hard_pointcloud_mid360_outdoor_kidnap_a'
    / 'archives'
    / 'gt.zip'
)
DEFAULT_BAG = REPO_ROOT / 'datasets' / 'mid360_public_loops' / 'outdoor_kidnap_raw' / 'rosbag2'
DEFAULT_OUTPUT_DIR = REPO_ROOT / 'output' / 'mid360_public' / 'outdoor_kidnap_loop_cloud'


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description=(
            'Compare real PointCloud2 windows around a public GT loop candidate after '
            'transforming both windows into the GT trajectory frame.'
        )
    )
    parser.add_argument('--loop-candidates', default=str(DEFAULT_LOOP_CANDIDATES))
    parser.add_argument('--gt-zip', default=str(DEFAULT_GT_ZIP))
    parser.add_argument('--bag', default=str(DEFAULT_BAG))
    parser.add_argument('--output-dir', default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument('--sequence-id', default='outdoor_kidnap')
    parser.add_argument('--pointcloud-topic', default='/livox/points')
    parser.add_argument('--candidate-index', type=int, default=0)
    parser.add_argument('--window-sec', type=float, default=1.0)
    parser.add_argument('--voxel-size-m', type=float, default=0.5)
    parser.add_argument('--max-scans-per-window', type=int, default=30)
    parser.add_argument('--max-points-per-scan', type=int, default=4000)
    parser.add_argument('--max-points-per-cloud', type=int, default=30000)
    parser.add_argument('--min-range-m', type=float, default=1.0)
    parser.add_argument('--max-range-m', type=float, default=80.0)
    parser.add_argument('--pass-median-nn-m', type=float, default=1.0)
    parser.add_argument('--pass-coverage-within-1m', type=float, default=0.30)
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    options = LoopCloudAnalysisOptions(
        loop_candidates_json=Path(args.loop_candidates),
        gt_zip=Path(args.gt_zip),
        bag_path=Path(args.bag),
        output_dir=Path(args.output_dir),
        sequence_id=str(args.sequence_id),
        pointcloud_topic=str(args.pointcloud_topic),
        candidate_index=max(0, int(args.candidate_index)),
        window_sec=max(0.0, float(args.window_sec)),
        voxel_size_m=max(0.01, float(args.voxel_size_m)),
        max_scans_per_window=max(1, int(args.max_scans_per_window)),
        max_points_per_scan=max(1, int(args.max_points_per_scan)),
        max_points_per_cloud=max(1, int(args.max_points_per_cloud)),
        min_range_m=max(0.0, float(args.min_range_m)),
        max_range_m=max(0.0, float(args.max_range_m)),
        pass_median_nn_m=max(0.0, float(args.pass_median_nn_m)),
        pass_coverage_within_1m=max(0.0, min(1.0, float(args.pass_coverage_within_1m))),
    )
    try:
        report = PublicLoopCloudAnalyzer().analyze(options)
    except Exception as exc:
        print(f'failed to analyze public MID-360 loop cloud overlap: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_public_loop_cloud_markdown(report))
        output_dir = Path(report['output_dir'])
        print(f'{PUBLIC_LOOP_CLOUD_ANALYSIS_JSON}: {output_dir / PUBLIC_LOOP_CLOUD_ANALYSIS_JSON}')
        print(
            f'{PUBLIC_LOOP_CLOUD_ANALYSIS_MARKDOWN}: '
            f'{output_dir / PUBLIC_LOOP_CLOUD_ANALYSIS_MARKDOWN}'
        )
    return 1 if report['status'] == 'FAIL' else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
