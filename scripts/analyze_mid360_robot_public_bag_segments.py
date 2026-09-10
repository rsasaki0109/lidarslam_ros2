#!/usr/bin/env python3
"""Analyze contiguous public MID-360 bag segments."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_bag_segments import (
    PUBLIC_BAG_SEGMENTS_JSON,
    PublicBagSegmentOptions,
    PublicBagSegmentReportBuilder,
    render_public_bag_segments_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_public_dataset_map_runner import PUBLIC_DATASET_MAP_CANDIDATES_JSON
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPO_ROOT / 'output' / 'mid360_public'
DEFAULT_MANIFEST = DEFAULT_OUTPUT_DIR / PUBLIC_DATASET_MAP_CANDIDATES_JSON


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Find contiguous RKO-LIO-safe segments in public MID-360 bags.'
    )
    parser.add_argument(
        '--manifest',
        default=str(DEFAULT_MANIFEST),
        help='Map candidate manifest JSON.',
    )
    parser.add_argument(
        '--output-dir',
        default=str(DEFAULT_OUTPUT_DIR),
        help='Directory for public bag segment artifacts.',
    )
    parser.add_argument(
        '--datasets',
        default='',
        help='Comma-separated dataset ids. Defaults to all candidates in the manifest.',
    )
    parser.add_argument(
        '--max-scan-gap-sec',
        type=float,
        default=1.0,
        help='Maximum adjacent RKO-LIO scan timestamp delta allowed inside one segment.',
    )
    parser.add_argument(
        '--min-segment-duration-sec',
        type=float,
        default=5.0,
        help='Minimum duration for a recommended segment.',
    )
    parser.add_argument(
        '--max-scans',
        type=int,
        default=0,
        help='Limit PointCloud2 scans read per dataset. Default 0 reads all scans.',
    )
    parser.add_argument(
        '--min-keypoints',
        type=int,
        default=10,
        help='Minimum estimated RKO-LIO keypoints required for a scan inside a segment.',
    )
    parser.add_argument(
        '--write',
        action='store_true',
        help=f'Write {PUBLIC_BAG_SEGMENTS_JSON} and Markdown artifacts.',
    )
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    options = PublicBagSegmentOptions(
        max_scan_gap_sec=max(0.0, float(args.max_scan_gap_sec)),
        min_segment_duration_sec=max(0.0, float(args.min_segment_duration_sec)),
        max_scans=max(0, int(args.max_scans)),
        min_keypoints=max(0, int(args.min_keypoints)),
    )
    try:
        builder = PublicBagSegmentReportBuilder(
            manifest_path=Path(args.manifest),
            output_dir=Path(args.output_dir),
        )
        report = builder.build(
            dataset_ids=tuple(_dataset_ids(args.datasets)),
            options=options,
        )
        paths = builder.write(report) if args.write else {}
    except Exception as exc:
        print(f'failed to analyze public MID-360 bag segments: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_public_bag_segments_markdown(report))
        if paths:
            print(f"Public bag segments JSON: {paths['json']}")
            print(f"Public bag segments Markdown: {paths['markdown']}")
    return 1 if report['status'] == 'FAIL' else 0


def _dataset_ids(value: str) -> list[str]:
    return [item.strip() for item in value.split(',') if item.strip()]


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
