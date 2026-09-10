#!/usr/bin/env python3
"""Check that a tracked MID-360 RKO-LIO config is backed by real-data QA."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_rko_config_adoption import (
    RKO_CONFIG_ADOPTION_JSON,
    RKO_CONFIG_ADOPTION_MARKDOWN,
    RkoConfigAdoptionChecker,
    render_rko_config_adoption_markdown,
    write_rko_config_adoption_report,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_QUALITY_REPORT = (
    REPO_ROOT
    / 'output'
    / 'mid360_public'
    / 'rko_sweep'
    / 'mid360_robot_public_rko_quality_report.json'
)
DEFAULT_CONFIG = (
    REPO_ROOT
    / 'configs'
    / 'mid360_robot'
    / 'rko_lio_mid360_low_voxel_no_deskew.yaml'
)


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Validate a tracked RKO-LIO MID-360 config against public real-data QA.'
    )
    parser.add_argument(
        '--quality-report',
        default=str(DEFAULT_QUALITY_REPORT),
        help='Path to mid360_robot_public_rko_quality_report.json.',
    )
    parser.add_argument(
        '--config',
        default=str(DEFAULT_CONFIG),
        help='Tracked RKO-LIO YAML to validate.',
    )
    parser.add_argument(
        '--output-dir',
        default='',
        help='Directory for adoption report outputs. Defaults to the quality report directory.',
    )
    parser.add_argument(
        '--require-best',
        action='store_true',
        help='Require the config to match the top-ranked gate-pass case.',
    )
    parser.add_argument(
        '--tolerance',
        type=float,
        default=1e-6,
        help='Float comparison tolerance for config parameters.',
    )
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    quality_report_path = Path(args.quality_report).expanduser().resolve()
    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else quality_report_path.parent
    )
    try:
        report = RkoConfigAdoptionChecker(
            quality_report_path=quality_report_path,
            config_path=Path(args.config),
            require_best=args.require_best,
            tolerance=max(0.0, float(args.tolerance)),
        ).build_report()
        paths = write_rko_config_adoption_report(report, output_dir)
    except Exception as exc:
        print(f'failed to check MID-360 RKO-LIO config adoption: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_rko_config_adoption_markdown(report))
        print(f'{RKO_CONFIG_ADOPTION_JSON}: {paths["json"]}')
        print(f'{RKO_CONFIG_ADOPTION_MARKDOWN}: {paths["markdown"]}')
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
