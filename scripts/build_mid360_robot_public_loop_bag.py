#!/usr/bin/env python3
"""CLI for merging public MID-360 split bags into one focused loop bag."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_loop_bag import (
    PUBLIC_LOOP_BAG_JSON,
    PUBLIC_LOOP_BAG_MARKDOWN,
    PublicLoopBagBuilder,
    PublicLoopBagOptions,
    render_public_loop_bag_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / 'datasets' / 'mid360_public_loops'


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Merge multiple public MID-360 split bags into one focused loop rosbag2.'
    )
    parser.add_argument(
        '--input-bag',
        action='append',
        required=True,
        help='Path to an input rosbag2 directory (repeat to add more, in order).',
    )
    parser.add_argument(
        '--output-bag',
        required=True,
        help='Path to the merged rosbag2 directory to create.',
    )
    parser.add_argument(
        '--topic',
        action='append',
        default=[],
        help='Topic to copy (repeat). Defaults to all topics.',
    )
    parser.add_argument('--no-tf', action='store_true', help='Skip /tf and /tf_static topics.')
    parser.add_argument(
        '--time-window-sec',
        nargs=2,
        type=float,
        metavar=('START_SEC', 'END_SEC'),
        default=None,
        help='Restrict to a window relative to the first message of the merged bag.',
    )
    parser.add_argument('--force', action='store_true', help='Replace an existing output bag.')
    parser.add_argument(
        '--allow-rewind',
        action='store_true',
        help='Allow input bags whose timestamps overlap previous bags (default rejects).',
    )
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    options = PublicLoopBagOptions(
        input_bags=tuple(Path(bag).expanduser().resolve() for bag in args.input_bag),
        output_bag=Path(args.output_bag).expanduser().resolve(),
        topics=tuple(args.topic),
        include_tf=not args.no_tf,
        time_window_sec=tuple(args.time_window_sec) if args.time_window_sec else None,
        force=bool(args.force),
        enforce_monotonic=not args.allow_rewind,
    )
    try:
        summary = PublicLoopBagBuilder().build(options)
    except Exception as exc:
        print(f'failed to build MID-360 public loop bag: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(summary))
    else:
        print(render_public_loop_bag_markdown(summary))
        sidecar_dir = Path(summary['output_bag']).parent
        print(f"{PUBLIC_LOOP_BAG_JSON}: {sidecar_dir / PUBLIC_LOOP_BAG_JSON}")
        print(f"{PUBLIC_LOOP_BAG_MARKDOWN}: {sidecar_dir / PUBLIC_LOOP_BAG_MARKDOWN}")
    return 0 if summary['status'] == 'PASS' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
