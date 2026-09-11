#!/usr/bin/env python3
"""CLI for raw sqlite merge of public MID-360 split rosbag2 directories."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_split_bag_merge import (
    PUBLIC_SPLIT_BAG_MERGE_JSON,
    PUBLIC_SPLIT_BAG_MERGE_MARKDOWN,
    SplitBagMergeOptions,
    SplitBagMerger,
    render_split_bag_merge_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description=(
            'Merge public MID-360 split rosbag2 sqlite directories without '
            'deserializing messages. This preserves custom Livox topics that may '
            'not be available in the local Python typestore.'
        ),
    )
    parser.add_argument(
        '--input-bag',
        action='append',
        required=True,
        help='Path to an input rosbag2 directory (repeat in playback order).',
    )
    parser.add_argument(
        '--output-bag',
        required=True,
        help='Path to the merged rosbag2 directory to create.',
    )
    parser.add_argument(
        '--output-db-name',
        default='merged_0.db3',
        help='SQLite filename to write inside the output bag.',
    )
    parser.add_argument('--force', action='store_true', help='Replace an existing output bag.')
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    options = SplitBagMergeOptions(
        input_bags=tuple(Path(bag).expanduser().resolve() for bag in args.input_bag),
        output_bag=Path(args.output_bag).expanduser().resolve(),
        force=bool(args.force),
        output_db_name=str(args.output_db_name),
    )
    try:
        report = SplitBagMerger().merge(options)
    except Exception as exc:
        print(f'failed to merge MID-360 public split bags: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_split_bag_merge_markdown(report))
        output_bag = Path(report['output_bag'])
        print(f'{PUBLIC_SPLIT_BAG_MERGE_JSON}: {output_bag / PUBLIC_SPLIT_BAG_MERGE_JSON}')
        print(
            f'{PUBLIC_SPLIT_BAG_MERGE_MARKDOWN}: '
            f'{output_bag / PUBLIC_SPLIT_BAG_MERGE_MARKDOWN}'
        )
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
