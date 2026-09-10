#!/usr/bin/env python3
"""CLI for reset-segment public MID-360 map cloud alignment."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_segment_map_cloud_alignment import (
    PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_JSON,
    PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_MARKDOWN,
    SegmentMapCloudAlignmentOptions,
    PublicSegmentMapCloudAlignmentAnalyzer,
    render_segment_map_cloud_alignment_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_START_RUN = REPO_ROOT / 'output' / 'mid360_public' / 'outdoor_kidnap_segment_reset' / 'segment_000'
DEFAULT_END_RUN = REPO_ROOT / 'output' / 'mid360_public' / 'outdoor_kidnap_segment_reset' / 'segment_012'
DEFAULT_OUTPUT_DIR = REPO_ROOT / 'output' / 'mid360_public' / 'outdoor_kidnap_segment_reset_alignment'
DEFAULT_LOOP_SEGMENT_PLAN = (
    REPO_ROOT
    / 'output'
    / 'mid360_public'
    / 'outdoor_kidnap_segment_reset_plan'
    / 'mid360_robot_public_loop_segment_reset.json'
)


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description=(
            'Align the reset-based start/end public MID-360 segment maps and gate '
            'the loop drift using cloud nearest-neighbor metrics.'
        ),
    )
    parser.add_argument('--start-run-dir', default=str(DEFAULT_START_RUN))
    parser.add_argument('--end-run-dir', default=str(DEFAULT_END_RUN))
    parser.add_argument('--output-dir', default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument('--start-pointcloud-map-dir')
    parser.add_argument('--end-pointcloud-map-dir')
    parser.add_argument('--start-trajectory')
    parser.add_argument('--end-trajectory')
    parser.add_argument('--loop-segment-plan', default=str(DEFAULT_LOOP_SEGMENT_PLAN))
    parser.add_argument('--start-center-stamp', type=float)
    parser.add_argument('--end-center-stamp', type=float)
    parser.add_argument('--crop-radius-m', type=float, default=20.0)
    parser.add_argument('--voxel-size-m', type=float, default=0.5)
    parser.add_argument('--max-points-per-tile', type=int, default=4000)
    parser.add_argument('--max-points-per-cloud', type=int, default=30000)
    parser.add_argument('--icp-max-iterations', type=int, default=40)
    parser.add_argument('--icp-trim-fraction', type=float, default=0.70)
    parser.add_argument('--icp-yaw-samples', type=int, default=36)
    parser.add_argument('--icp-convergence-translation-m', type=float, default=1e-3)
    parser.add_argument('--icp-convergence-rotation-deg', type=float, default=0.05)
    parser.add_argument('--pass-median-nn-m', type=float, default=1.0)
    parser.add_argument('--pass-p90-nn-m', type=float, default=2.5)
    parser.add_argument('--pass-coverage-within-1m', type=float, default=0.35)
    parser.add_argument('--min-cloud-points', type=int, default=200)
    parser.add_argument('--ply-max-points-per-cloud', type=int, default=15000)
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    plan_stamps = _plan_stamps(Path(args.loop_segment_plan))
    start_center_stamp = (
        float(args.start_center_stamp)
        if args.start_center_stamp is not None
        else plan_stamps.get('start')
    )
    end_center_stamp = (
        float(args.end_center_stamp)
        if args.end_center_stamp is not None
        else plan_stamps.get('end')
    )
    options = SegmentMapCloudAlignmentOptions(
        start_run_dir=Path(args.start_run_dir),
        end_run_dir=Path(args.end_run_dir),
        output_dir=Path(args.output_dir),
        start_pointcloud_map_dir=Path(args.start_pointcloud_map_dir)
        if args.start_pointcloud_map_dir else None,
        end_pointcloud_map_dir=Path(args.end_pointcloud_map_dir)
        if args.end_pointcloud_map_dir else None,
        start_trajectory_path=Path(args.start_trajectory) if args.start_trajectory else None,
        end_trajectory_path=Path(args.end_trajectory) if args.end_trajectory else None,
        start_center_stamp=start_center_stamp,
        end_center_stamp=end_center_stamp,
        crop_radius_m=max(0.0, float(args.crop_radius_m)),
        voxel_size_m=max(0.01, float(args.voxel_size_m)),
        max_points_per_tile=max(1, int(args.max_points_per_tile)),
        max_points_per_cloud=max(1, int(args.max_points_per_cloud)),
        icp_max_iterations=max(1, int(args.icp_max_iterations)),
        icp_trim_fraction=max(0.05, min(1.0, float(args.icp_trim_fraction))),
        icp_yaw_samples=max(1, int(args.icp_yaw_samples)),
        icp_convergence_translation_m=max(0.0, float(args.icp_convergence_translation_m)),
        icp_convergence_rotation_deg=max(0.0, float(args.icp_convergence_rotation_deg)),
        pass_median_nn_m=max(0.0, float(args.pass_median_nn_m)),
        pass_p90_nn_m=max(0.0, float(args.pass_p90_nn_m)),
        pass_coverage_within_1m=max(0.0, min(1.0, float(args.pass_coverage_within_1m))),
        min_cloud_points=max(1, int(args.min_cloud_points)),
        ply_max_points_per_cloud=max(1, int(args.ply_max_points_per_cloud)),
    )
    try:
        report = PublicSegmentMapCloudAlignmentAnalyzer().analyze(options)
    except Exception as exc:
        print(f'failed to align public MID-360 segment maps: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_segment_map_cloud_alignment_markdown(report))
        output_dir = Path(report['output_dir'])
        print(
            f'{PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_JSON}: '
            f'{output_dir / PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_JSON}'
        )
        print(
            f'{PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_MARKDOWN}: '
            f'{output_dir / PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_MARKDOWN}'
        )
    return 0 if report['status'] == 'PASS' else 1


def _plan_stamps(path: Path) -> dict[str, float]:
    if not path.is_file():
        return {}
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return {}
    reset_pair = payload.get('reset_pair') or {}
    stamps = {}
    for key in ('start', 'end'):
        endpoint = reset_pair.get(key) or {}
        if endpoint.get('candidate_stamp') is not None:
            stamps[key] = float(endpoint['candidate_stamp'])
    return stamps


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
