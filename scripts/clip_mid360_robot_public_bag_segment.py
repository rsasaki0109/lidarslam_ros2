#!/usr/bin/env python3
"""Clip a public MID-360 bag segment selected by segment analysis."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_bag_clip import (
    PublicBagSegmentClipOptions,
    PublicBagSegmentClipper,
    render_public_bag_segment_clip_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_public_bag_segments import PUBLIC_BAG_SEGMENTS_JSON
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SEGMENTS = REPO_ROOT / 'output' / 'mid360_public' / PUBLIC_BAG_SEGMENTS_JSON
DEFAULT_OUTPUT_ROOT = REPO_ROOT / 'datasets' / 'mid360_public_segments'


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Clip one public MID-360 bag segment for a focused retry.'
    )
    parser.add_argument('--segments', default=str(DEFAULT_SEGMENTS), help='Bag segment report JSON.')
    parser.add_argument('--dataset', required=True, help='Dataset id to clip.')
    parser.add_argument(
        '--segment',
        default='',
        help='Segment id. Defaults to the recommended segment.',
    )
    parser.add_argument(
        '--output-root',
        default=str(DEFAULT_OUTPUT_ROOT),
        help='Root directory for clipped public bag segments.',
    )
    parser.add_argument(
        '--margin-sec',
        type=float,
        default=0.0,
        help='Extra receive-time margin around the selected segment. Default 0 keeps the RKO-safe span exact.',
    )
    parser.add_argument(
        '--no-tf',
        action='store_true',
        help='Do not copy /tf_static or /tf if present.',
    )
    parser.add_argument('--force', action='store_true', help='Replace an existing clip output.')
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    try:
        summary = PublicBagSegmentClipper(Path(args.segments)).clip(
            PublicBagSegmentClipOptions(
                dataset_id=args.dataset,
                segment_id=args.segment,
                output_root=Path(args.output_root),
                margin_sec=max(0.0, float(args.margin_sec)),
                force=bool(args.force),
                include_tf=not args.no_tf,
            )
        )
    except Exception as exc:
        print(f'failed to clip public MID-360 bag segment: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(summary))
    else:
        print(render_public_bag_segment_clip_markdown(summary))
    return 0 if summary.get('status') == 'PASS' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
