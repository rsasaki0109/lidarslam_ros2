#!/usr/bin/env python3
"""Diagnose public MID-360 map-run outputs."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_dataset_map_runner import PUBLIC_DATASET_MAP_CANDIDATES_JSON
from lidarslam_benchmark_tools.mid360_robot_public_map_run_diagnosis import (
    PublicDatasetMapRunDiagnosisBuilder,
    render_public_map_run_diagnosis_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPO_ROOT / 'output' / 'mid360_public'
DEFAULT_MANIFEST = DEFAULT_OUTPUT_DIR / PUBLIC_DATASET_MAP_CANDIDATES_JSON


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Diagnose public MID-360 map candidate run artifacts.'
    )
    parser.add_argument(
        '--manifest',
        default=str(DEFAULT_MANIFEST),
        help='Map candidate manifest JSON.',
    )
    parser.add_argument(
        '--output-dir',
        default=str(DEFAULT_OUTPUT_DIR),
        help='Directory for public map-run diagnosis artifacts.',
    )
    parser.add_argument(
        '--datasets',
        default='',
        help='Comma-separated dataset ids. Defaults to all candidates in the manifest.',
    )
    parser.add_argument(
        '--write',
        action='store_true',
        help='Write diagnosis JSON and Markdown artifacts.',
    )
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    try:
        builder = PublicDatasetMapRunDiagnosisBuilder(
            manifest_path=Path(args.manifest),
            output_dir=Path(args.output_dir),
        )
        report = builder.build(dataset_ids=tuple(_dataset_ids(args.datasets)))
        paths = builder.write(report) if args.write else {}
    except Exception as exc:
        print(f'failed to diagnose public MID-360 map run: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_public_map_run_diagnosis_markdown(report))
        if paths:
            print(f"Public map-run diagnosis JSON: {paths['json']}")
            print(f"Public map-run diagnosis Markdown: {paths['markdown']}")
    return 0


def _dataset_ids(value: str) -> list[str]:
    return [item.strip() for item in value.split(',') if item.strip()]


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
