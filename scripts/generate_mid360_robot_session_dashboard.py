#!/usr/bin/env python3
"""Generate a static HTML dashboard for MID-360 robot session artifacts."""

from __future__ import annotations

import argparse
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_dashboard import DASHBOARD_HTML, write_dashboard


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Generate a MID-360 robot session HTML dashboard.'
    )
    parser.add_argument('output_dir', help='Directory containing MID-360 robot session JSON artifacts.')
    parser.add_argument(
        '--output',
        help=f'HTML output path. Defaults to <output_dir>/{DASHBOARD_HTML}.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve() if args.output else None
    path = write_dashboard(output_dir, output_path)
    print(f'MID-360 session dashboard: {path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
