#!/usr/bin/env python3
"""Merge verified map sessions into one non-destructive map project."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Sequence

from map_merge import MapMergeError, MergeOptions, merge_map_sessions


EX_USAGE = 2
EX_SOFTWARE = 70


def _positive_float(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('must be a number') from exc
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError('must be finite and greater than zero')
    return parsed


def _positive_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('must be an integer') from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError('must be greater than zero')
    return parsed


def _fraction(value: str) -> float:
    parsed = _positive_float(value)
    if parsed > 1.0:
        raise argparse.ArgumentTypeError('must not exceed 1.0')
    return parsed


def _initial_transform(value: str) -> tuple[int, tuple[float, float, float, float]]:
    try:
        index_text, values_text = value.split(':', 1)
        index = int(index_text)
        values = tuple(float(item) for item in values_text.split(','))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            'expected <index:tx,ty,tz,yaw_deg>'
        ) from exc
    if index <= 0 or len(values) != 4 or not all(math.isfinite(item) for item in values):
        raise argparse.ArgumentTypeError(
            'expected a non-anchor index and four finite values: '
            '<index:tx,ty,tz,yaw_deg>'
        )
    return index, values


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the stable multi-session project command."""
    command = os.environ.get('LIDARSLAM_CLI_COMMAND', 'lidarslam-map merge')
    parser = argparse.ArgumentParser(
        prog=command,
        description=(
            'Validate, align, and combine two or more completed map outputs into '
            'one verified map project without modifying its sources.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'The first map is the anchor frame. Every later map must overlap an '
            'already aligned session and use the same PCD fields and projector.\n\n'
            'Example:\n'
            f'  {command} output/day1 output/day2 --output-dir output/site_project'
        ),
    )
    parser.add_argument(
        'map_outputs',
        nargs='+',
        metavar='<map_outputs>',
        help='Completed source map outputs; first is the anchor (minimum: two).',
    )
    parser.add_argument('--help-all', action='help', help='Show all alignment controls.')
    parser.add_argument(
        '--output-dir',
        required=True,
        metavar='<dir>',
        help='New project directory; it must not exist.',
    )
    parser.add_argument(
        '--merge-voxel-size',
        type=_positive_float,
        default=0.20,
        metavar='<m>',
        help='Final cross-session deduplication voxel size (default: 0.20 m).',
    )
    parser.add_argument(
        '--alignment-voxel-size',
        type=_positive_float,
        default=0.50,
        metavar='<m>',
        help='Point spacing used by automatic alignment (default: 0.50 m).',
    )
    parser.add_argument(
        '--max-alignment-points',
        type=_positive_int,
        default=12000,
        metavar='<count>',
        help='Deterministic point cap per alignment cloud (default: 12000).',
    )
    parser.add_argument(
        '--max-median-error',
        type=_positive_float,
        default=1.0,
        metavar='<m>',
        help='Maximum trimmed overlap median (default: 1.0 m).',
    )
    parser.add_argument(
        '--max-p90-error',
        type=_positive_float,
        default=2.5,
        metavar='<m>',
        help='Maximum trimmed overlap p90 (default: 2.5 m).',
    )
    parser.add_argument(
        '--min-overlap',
        type=_fraction,
        default=0.15,
        metavar='<ratio>',
        help='Minimum one-direction coverage within 1 m (default: 0.15).',
    )
    parser.add_argument(
        '--initial-transform',
        action='append',
        type=_initial_transform,
        default=[],
        metavar='<index:tx,ty,tz,yaw_deg>',
        help='Optional gravity-aligned starting transform; repeat per session.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Run source validation and alignment without writing a project.',
    )
    parser.add_argument('--json', action='store_true', help='Print the receipt as JSON.')
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Build and report one verified multi-session map project."""
    args = parse_args(argv)
    if len(args.map_outputs) < 2:
        print('error: merge requires at least two map outputs', file=sys.stderr)
        return EX_USAGE
    transforms = {}
    for index, transform in args.initial_transform:
        if index >= len(args.map_outputs):
            print(
                f'error: initial transform index {index} is outside the '
                f'{len(args.map_outputs)} supplied sessions',
                file=sys.stderr,
            )
            return EX_USAGE
        if index in transforms:
            print(f'error: duplicate initial transform for session {index}', file=sys.stderr)
            return EX_USAGE
        transforms[index] = transform
    try:
        receipt = merge_map_sessions(
            source_dirs=[Path(value) for value in args.map_outputs],
            output_dir=Path(args.output_dir),
            options=MergeOptions(
                merge_voxel_size_m=args.merge_voxel_size,
                alignment_voxel_size_m=args.alignment_voxel_size,
                max_alignment_points=args.max_alignment_points,
                max_overlap_median_m=args.max_median_error,
                max_overlap_p90_m=args.max_p90_error,
                min_overlap_within_1m=args.min_overlap,
            ),
            initial_transforms=transforms,
            dry_run=bool(args.dry_run),
        )
    except MapMergeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return EX_USAGE
    except OSError as exc:
        print(f'error: map merge failed: {exc}', file=sys.stderr)
        return EX_SOFTWARE
    if args.json:
        print(json.dumps(receipt, indent=2, sort_keys=True))
    elif receipt['status'] == 'DRY_RUN':
        print('Multi-session map project is ready to publish')
        print(f"  sessions: {receipt['session_count']}")
        print(f"  source points: {receipt['source_points']}")
        print(f"  merged points: {receipt['merged_points']}")
        print(f"  candidate: {receipt['output_dir']}")
    else:
        print('Multi-session map project verified')
        print(f"  project: {receipt['output_dir']}")
        print(f"  sessions: {receipt['session_count']}")
        print(f"  merged points: {receipt['merged_points']}")
        print(f"  duplicate points removed: {receipt['duplicate_points_removed']}")
        print(f"  receipt: {Path(receipt['output_dir']) / 'map_merge_receipt.json'}")
        print(f"  next: lidarslam-map view {receipt['output_dir']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
