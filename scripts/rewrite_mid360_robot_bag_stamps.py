#!/usr/bin/env python3
"""CLI for rewriting PointCloud2 / Imu header.stamp to bag receive time."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_bag_stamp_rewriter import (
    BAG_STAMP_REWRITER_JSON,
    BAG_STAMP_REWRITER_MARKDOWN,
    DEFAULT_STAMP_MSGTYPES,
    BagStampRewriter,
    BagStampRewriterOptions,
    render_bag_stamp_rewriter_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description=(
            'Rewrite PointCloud2 / Imu header.stamp in a rosbag2 to match the rosbag2 '
            'receive timestamp. Useful when upstream MID-360 recordings have corrupted '
            'PointCloud2 header stamps that trigger RKO-LIO Δt errors.'
        ),
    )
    parser.add_argument(
        '--input-bag',
        required=True,
        help='Path to an existing rosbag2 directory.',
    )
    parser.add_argument(
        '--output-bag',
        required=True,
        help='Path to the rewritten rosbag2 directory to create.',
    )
    parser.add_argument(
        '--rewrite-msgtype',
        action='append',
        default=[],
        help=(
            'Repeatable. ROS msg type whose header.stamp should be replaced by the '
            'bag receive time. Defaults to PointCloud2 and Imu.'
        ),
    )
    parser.add_argument(
        '--rewrite-topic',
        action='append',
        default=[],
        help='Repeatable. Topic name whose header.stamp should be replaced.',
    )
    parser.add_argument(
        '--force',
        action='store_true',
        help='Replace an existing output bag directory.',
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Print JSON instead of Markdown.',
    )
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    msgtypes = tuple(args.rewrite_msgtype) if args.rewrite_msgtype else DEFAULT_STAMP_MSGTYPES
    options = BagStampRewriterOptions(
        input_bag=Path(args.input_bag).expanduser().resolve(),
        output_bag=Path(args.output_bag).expanduser().resolve(),
        rewrite_msgtypes=msgtypes,
        rewrite_topics=tuple(args.rewrite_topic),
        force=bool(args.force),
    )
    try:
        summary = BagStampRewriter().rewrite(options)
    except Exception as exc:
        print(f'failed to rewrite MID-360 bag stamps: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(summary))
    else:
        print(render_bag_stamp_rewriter_markdown(summary))
        sidecar_dir = Path(summary['output_bag']).parent
        print(f'{BAG_STAMP_REWRITER_JSON}: {sidecar_dir / BAG_STAMP_REWRITER_JSON}')
        print(f'{BAG_STAMP_REWRITER_MARKDOWN}: {sidecar_dir / BAG_STAMP_REWRITER_MARKDOWN}')
    return 0 if summary['status'] == 'PASS' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
