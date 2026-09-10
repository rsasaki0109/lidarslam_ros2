#!/usr/bin/env python3
"""Check Jetson host readiness before a MID-360 robot mapping run."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    from lidarslam_benchmark_tools.jetson_mid360_host_tools import (
        HostReadinessOptions,
        JetsonHostReadiness,
        payload_to_json,
    )
except ModuleNotFoundError:
    # Direct source-checkout invocation puts this script's directory on
    # sys.path, while the installed package exposes the same canonical module
    # under lidarslam_benchmark_tools.  Keep the two execution surfaces
    # identical without maintaining a second implementation.
    from jetson_mid360_host_tools import (  # type: ignore[no-redef]
        HostReadinessOptions,
        JetsonHostReadiness,
        payload_to_json,
    )


REPO_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Check Jetson host readiness for MID-360 robot mapping.'
    )
    parser.add_argument(
        '--output-dir',
        default=str(REPO_ROOT / 'output' / 'jetson_mid360_host_readiness'),
        help='Directory for host-readiness report files.',
    )
    parser.add_argument(
        '--bag-dir',
        help='Bag recording directory or mount point to check for free space.',
    )
    parser.add_argument(
        '--expected-bag-minutes',
        type=float,
        default=10.0,
        help='Expected recording duration for storage estimate.',
    )
    parser.add_argument(
        '--estimated-bag-mbps',
        type=float,
        default=50.0,
        help='Estimated rosbag2 write rate in MB/s for storage estimate.',
    )
    parser.add_argument(
        '--bag-reserve-gb',
        type=float,
        default=5.0,
        help='Extra free-space reserve for bag recording.',
    )
    parser.add_argument(
        '--min-bag-free-gb',
        type=float,
        default=20.0,
        help='Minimum free space required on --bag-dir.',
    )
    parser.add_argument(
        '--min-output-free-gb',
        type=float,
        default=5.0,
        help='Minimum free space required for map output.',
    )
    parser.add_argument(
        '--thermal-warn-c',
        type=float,
        default=75.0,
        help='Warn when a thermal zone is at or above this temperature.',
    )
    parser.add_argument(
        '--thermal-fail-c',
        type=float,
        default=85.0,
        help='Fail when a thermal zone is at or above this temperature.',
    )
    parser.add_argument(
        '--min-memory-available-gb',
        type=float,
        default=1.0,
        help='Minimum MemAvailable required before mapping.',
    )
    parser.add_argument(
        '--host-root',
        default='/',
        help='Host filesystem root to inspect; useful for tests and chroots.',
    )
    parser.add_argument('--json', action='store_true', help='Print report JSON.')
    return parser.parse_args()


def _options_from_args(args: argparse.Namespace) -> HostReadinessOptions:
    return HostReadinessOptions(
        output_dir=Path(args.output_dir).expanduser().resolve(),
        bag_dir=Path(args.bag_dir).expanduser().resolve() if args.bag_dir else None,
        expected_bag_minutes=args.expected_bag_minutes,
        estimated_bag_mbps=args.estimated_bag_mbps,
        bag_reserve_gb=args.bag_reserve_gb,
        min_bag_free_gb=args.min_bag_free_gb,
        min_output_free_gb=args.min_output_free_gb,
        thermal_warn_c=args.thermal_warn_c,
        thermal_fail_c=args.thermal_fail_c,
        min_memory_available_gb=args.min_memory_available_gb,
    )


def main() -> int:
    args = parse_args()
    options = _options_from_args(args)
    checker = JetsonHostReadiness(host_root=Path(args.host_root).expanduser().resolve())
    report = checker.build_report(options)
    paths = checker.write(report, options.output_dir)

    if args.json:
        print(payload_to_json(report))
    else:
        print(checker.render_markdown(report))
        print(f"Host readiness JSON: {paths['json']}")
        print(f"Host readiness Markdown: {paths['markdown']}")

    return 1 if report['status'] == 'FAIL' else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
