#!/usr/bin/env python3
"""Import and optionally recheck a MID-360 production-candidate bundle."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_production_candidate_bundle_import import (
    BUNDLE_IMPORT_JSON,
    BUNDLE_IMPORT_MARKDOWN,
    ImportOptions,
    Mid360ProductionCandidateBundleImporter,
    render_import_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Import a MID-360 production-candidate bundle and optionally recheck it.'
    )
    parser.add_argument('bundle', help='Bundle tar.gz/tgz or staged bundle directory.')
    parser.add_argument(
        '--output-dir',
        default='',
        help='Directory where bundle contents are imported. Defaults beside the bundle.',
    )
    parser.add_argument('--recheck', action='store_true', help='Re-run production readiness from imported artifacts.')
    parser.add_argument('--verify', action='store_true', help='Fail if the imported bundle is incomplete.')
    parser.add_argument('--force', action='store_true', help='Overwrite an existing output directory.')
    parser.add_argument(
        '--bag-root',
        default='',
        help='Working bag root for recheck plan sidecars. Defaults to <bundle>/recheck_recording.',
    )
    parser.add_argument(
        '--min-bag-duration-sec',
        type=float,
        default=None,
        help='Override the production gate duration threshold during --recheck.',
    )
    parser.add_argument('--json', action='store_true', help='Print machine-readable import report JSON.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    try:
        report = Mid360ProductionCandidateBundleImporter().import_bundle(
            ImportOptions(
                bundle_path=Path(args.bundle),
                output_dir=Path(args.output_dir) if args.output_dir else None,
                recheck=args.recheck,
                verify=args.verify,
                force=args.force,
                bag_root=Path(args.bag_root) if args.bag_root else None,
                min_bag_duration_sec=args.min_bag_duration_sec,
            ),
            quiet=args.json,
        )
    except Exception as exc:
        print(f'failed to import MID-360 production-candidate bundle: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_import_markdown(report))
        bundle_dir = Path(report['bundle_dir'])
        print(f'{BUNDLE_IMPORT_JSON}: {bundle_dir / BUNDLE_IMPORT_JSON}')
        print(f'{BUNDLE_IMPORT_MARKDOWN}: {bundle_dir / BUNDLE_IMPORT_MARKDOWN}')

    if report['status'] == 'FAIL':
        return 1
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
