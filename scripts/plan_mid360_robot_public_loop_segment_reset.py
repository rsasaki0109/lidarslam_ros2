#!/usr/bin/env python3
"""CLI for planning public MID-360 loop segment-reset runs."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_loop_segment_reset import (
    PUBLIC_LOOP_SEGMENT_RESET_JSON,
    PUBLIC_LOOP_SEGMENT_RESET_MARKDOWN,
    LoopSegmentResetOptions,
    LoopSegmentResetPlanner,
    render_loop_segment_reset_markdown,
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
DEFAULT_BAG = REPO_ROOT / 'datasets' / 'mid360_public_loops' / 'outdoor_kidnap_raw' / 'rosbag2'
DEFAULT_OUTPUT_DIR = REPO_ROOT / 'output' / 'mid360_public' / 'outdoor_kidnap_segment_reset_plan'
DEFAULT_CLIP_ROOT = REPO_ROOT / 'datasets' / 'mid360_public_loop_segments'
DEFAULT_RKO_OUTPUT_ROOT = REPO_ROOT / 'output' / 'mid360_public' / 'outdoor_kidnap_segment_reset'


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description=(
            'Plan RKO-LIO segment-reset runs for a public MID-360 loop candidate. '
            'The planner maps GT loop endpoints to RKO-safe PointCloud2 scan segments.'
        ),
    )
    parser.add_argument('--loop-candidates', default=str(DEFAULT_LOOP_CANDIDATES))
    parser.add_argument('--bag', default=str(DEFAULT_BAG))
    parser.add_argument('--output-dir', default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument('--sequence-id', default='outdoor_kidnap')
    parser.add_argument('--pointcloud-topic', default='/livox/points')
    parser.add_argument('--imu-topic', default='/livox/imu')
    parser.add_argument('--candidate-index', type=int, default=0)
    parser.add_argument('--max-scan-gap-sec', type=float, default=10000.0)
    parser.add_argument('--min-segment-duration-sec', type=float, default=5.0)
    parser.add_argument('--max-scans', type=int, default=0)
    parser.add_argument('--min-keypoints', type=int, default=10)
    parser.add_argument('--voxel-size', type=float, default=1.0)
    parser.add_argument('--min-range', type=float, default=1.0)
    parser.add_argument('--max-range', type=float, default=100.0)
    parser.add_argument('--clip-output-root', default=str(DEFAULT_CLIP_ROOT))
    parser.add_argument('--rko-output-root', default=str(DEFAULT_RKO_OUTPUT_ROOT))
    parser.add_argument('--lidarslam-param', default='lidarslam/param/lidarslam_mid360_rko_graph.yaml')
    parser.add_argument('--rko-param', default='configs/mid360_robot/rko_lio_mid360_kidnap_tolerant.yaml')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--lidar-frame', default='livox_frame')
    parser.add_argument('--imu-frame', default='livox_frame')
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    options = LoopSegmentResetOptions(
        loop_candidates_json=Path(args.loop_candidates),
        bag_path=Path(args.bag),
        output_dir=Path(args.output_dir),
        sequence_id=str(args.sequence_id),
        pointcloud_topic=str(args.pointcloud_topic),
        imu_topic=str(args.imu_topic),
        candidate_index=max(0, int(args.candidate_index)),
        max_scan_gap_sec=max(0.0, float(args.max_scan_gap_sec)),
        min_segment_duration_sec=max(0.0, float(args.min_segment_duration_sec)),
        max_scans=max(0, int(args.max_scans)),
        min_keypoints=max(0, int(args.min_keypoints)),
        voxel_size=max(0.01, float(args.voxel_size)),
        min_range=max(0.0, float(args.min_range)),
        max_range=max(0.0, float(args.max_range)),
        clip_output_root=Path(args.clip_output_root),
        rko_output_root=Path(args.rko_output_root),
        lidarslam_param=Path(args.lidarslam_param),
        rko_param=Path(args.rko_param),
        base_frame=str(args.base_frame),
        lidar_frame=str(args.lidar_frame),
        imu_frame=str(args.imu_frame),
    )
    try:
        report = LoopSegmentResetPlanner().plan(options)
    except Exception as exc:
        print(f'failed to plan public MID-360 loop segment reset: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_loop_segment_reset_markdown(report))
        artifacts = report.get('artifacts') or {}
        print(f'{PUBLIC_LOOP_SEGMENT_RESET_JSON}: {artifacts.get("plan_json", "")}')
        print(f'{PUBLIC_LOOP_SEGMENT_RESET_MARKDOWN}: {artifacts.get("plan_markdown", "")}')
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
