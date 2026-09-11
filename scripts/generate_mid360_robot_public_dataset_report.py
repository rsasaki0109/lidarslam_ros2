#!/usr/bin/env python3
"""Generate a comparison report for public MID-360 dataset intake artifacts."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_dataset_report import (
    PublicDatasetReportBuilder,
    render_public_dataset_report_markdown,
    write_public_dataset_report,
)
from lidarslam_benchmark_tools.mid360_robot_public_datasets import public_dataset_registry
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET_ROOT = REPO_ROOT / 'datasets' / 'mid360_public'
DEFAULT_OUTPUT_ROOT = REPO_ROOT / 'output' / 'mid360_public'


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Compare public MID-360 intake/readiness artifacts.'
    )
    parser.add_argument(
        '--dataset-root',
        default=str(DEFAULT_DATASET_ROOT),
        help='Root directory where public dataset intake artifacts live.',
    )
    parser.add_argument(
        '--output-root',
        default=str(DEFAULT_OUTPUT_ROOT),
        help='Root directory where per-dataset readiness/check artifacts live.',
    )
    parser.add_argument(
        '--output-dir',
        default=str(DEFAULT_OUTPUT_ROOT),
        help='Directory where comparison JSON/Markdown/HTML are written.',
    )
    parser.add_argument(
        '--datasets',
        default='',
        help='Comma-separated dataset ids. Defaults to all registered public datasets.',
    )
    parser.add_argument(
        '--map-sweep',
        action='append',
        default=[],
        help=(
            'Optional MID-360 public RKO sweep manifest. Repeatable. '
            'Defaults to discovered mid360_robot_public_rko_sweep.json files under output-root.'
        ),
    )
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    dataset_ids = _dataset_ids(args.datasets)
    try:
        report = PublicDatasetReportBuilder(
            dataset_root=Path(args.dataset_root),
            output_root=Path(args.output_root),
            dataset_ids=dataset_ids,
            map_sweep_paths=[Path(item) for item in args.map_sweep] if args.map_sweep else None,
        ).build_report()
        paths = write_public_dataset_report(report, Path(args.output_dir))
    except Exception as exc:
        print(f'failed to generate public MID-360 dataset report: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_public_dataset_report_markdown(report))
        print(f"Public dataset report JSON: {paths['json']}")
        print(f"Public dataset report Markdown: {paths['markdown']}")
        print(f"Public dataset report HTML: {paths['html']}")
    return 1 if report['status'] == 'FAIL' else 0


def _dataset_ids(value: str) -> list[str]:
    if not value.strip():
        return [dataset.id for dataset in public_dataset_registry()]
    return [item.strip() for item in value.split(',') if item.strip()]


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
