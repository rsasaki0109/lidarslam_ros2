#!/usr/bin/env python3
"""Select and optionally run map candidates from the public MID-360 report."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_dataset_map_runner import (
    PublicDatasetMapRunOptions,
    PublicDatasetMapRunner,
    PublicDatasetMapSafetyOptions,
    PublicDatasetMapSelectionOptions,
    render_map_candidates_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_public_dataset_report import PUBLIC_DATASET_REPORT_JSON
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPO_ROOT / 'output' / 'mid360_public'
DEFAULT_REPORT = DEFAULT_OUTPUT_DIR / PUBLIC_DATASET_REPORT_JSON


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Select runnable public MID-360 map candidates.'
    )
    parser.add_argument(
        '--report',
        default=str(DEFAULT_REPORT),
        help='Public dataset comparison report JSON.',
    )
    parser.add_argument(
        '--output-dir',
        default=str(DEFAULT_OUTPUT_DIR),
        help='Directory for map candidate manifest artifacts.',
    )
    parser.add_argument(
        '--datasets',
        default='',
        help='Comma-separated dataset ids. Defaults to all datasets in the report.',
    )
    parser.add_argument(
        '--pass-only',
        action='store_true',
        help='Only select PASS rows. By default WARN rows are allowed when launch-ready.',
    )
    parser.add_argument('--limit', type=int, default=0, help='Maximum selected candidates.')
    parser.add_argument(
        '--min-free-gb',
        type=float,
        default=5.0,
        help='Free-space reserve required after estimated output size.',
    )
    parser.add_argument(
        '--runtime-scale',
        type=float,
        default=1.5,
        help='Runtime estimate multiplier applied to bag duration before fixed overhead.',
    )
    parser.add_argument(
        '--output-size-ratio',
        type=float,
        default=1.25,
        help='Estimated map output size as a ratio of bag directory size.',
    )
    parser.add_argument(
        '--allow-existing-map-output',
        action='store_true',
        help='Do not block when map outputs already exist under the candidate output directory.',
    )
    parser.add_argument(
        '--run',
        action='store_true',
        help='Execute selected map commands. Default only writes the candidate manifest.',
    )
    parser.add_argument(
        '--run-timeout-sec',
        type=int,
        default=0,
        help='Timeout for each executed map command. Default 0 disables the wrapper timeout.',
    )
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    options = PublicDatasetMapSelectionOptions(
        dataset_ids=tuple(_dataset_ids(args.datasets)),
        allow_warn=not args.pass_only,
        limit=max(0, int(args.limit)),
    )
    safety_options = PublicDatasetMapSafetyOptions(
        min_free_bytes=max(0, int(args.min_free_gb * 1024 * 1024 * 1024)),
        runtime_scale=max(0.1, float(args.runtime_scale)),
        output_size_ratio=max(0.0, float(args.output_size_ratio)),
        allow_existing_map_output=args.allow_existing_map_output,
    )
    run_options = PublicDatasetMapRunOptions(
        timeout_sec=max(0, int(args.run_timeout_sec)),
    )
    try:
        runner = PublicDatasetMapRunner(
            report_path=Path(args.report),
            output_dir=Path(args.output_dir),
        )
        manifest = runner.build_manifest(
            options=options,
            safety_options=safety_options,
            run_options=run_options,
            run=args.run,
        )
        paths = runner.write_manifest(manifest)
    except Exception as exc:
        print(f'failed to build public MID-360 map candidates: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(manifest))
    else:
        print(render_map_candidates_markdown(manifest))
        print(f"Map candidates JSON: {paths['json']}")
        print(f"Map candidates Markdown: {paths['markdown']}")
    return 1 if manifest['status'] == 'FAIL' else 0


def _dataset_ids(value: str) -> list[str]:
    return [item.strip() for item in value.split(',') if item.strip()]


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
