#!/usr/bin/env python3
"""Export a portable MID-360 production-candidate artifact bundle."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_production_candidate_bundle import (
    BUNDLE_MANIFEST_JSON,
    BUNDLE_MANIFEST_MARKDOWN,
    BundleOptions,
    Mid360ProductionCandidateBundleExporter,
    render_bundle_markdown,
    verify_bundle_manifest,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Export MID-360 production-candidate artifacts as a tar.gz bundle.'
    )
    parser.add_argument(
        'artifact_dir',
        help='Directory containing production-candidate session artifacts.',
    )
    parser.add_argument(
        '--output',
        default='',
        help='Output .tar.gz path or staging directory. Defaults next to artifact_dir.',
    )
    parser.add_argument('--label', default='', help='Bundle label stored in the manifest.')
    parser.add_argument('--verify', action='store_true', help='Fail if required bundle artifacts are missing.')
    parser.add_argument('--force', action='store_true', help='Overwrite an existing bundle dir/tarball.')
    parser.add_argument('--json', action='store_true', help='Print machine-readable bundle manifest JSON.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    try:
        manifest = Mid360ProductionCandidateBundleExporter().export(
            BundleOptions(
                artifact_dir=Path(args.artifact_dir),
                output_path=Path(args.output) if args.output else None,
                label=args.label,
                force=args.force,
            )
        )
    except Exception as exc:
        print(f'failed to export MID-360 production-candidate bundle: {exc}', file=sys.stderr)
        return 1

    ok, errors = verify_bundle_manifest(manifest)
    if args.json:
        print(payload_to_json(manifest))
    else:
        print(render_bundle_markdown(manifest))
        print(f'{BUNDLE_MANIFEST_JSON}: {Path(manifest["bundle_dir"]) / BUNDLE_MANIFEST_JSON}')
        print(f'{BUNDLE_MANIFEST_MARKDOWN}: {Path(manifest["bundle_dir"]) / BUNDLE_MANIFEST_MARKDOWN}')
        if manifest.get('tarball_path'):
            print(f'Tarball: {manifest["tarball_path"]}')
        if errors:
            print('Bundle verification errors:', file=sys.stderr)
            for error in errors:
                print(f'- {error}', file=sys.stderr)

    if args.verify and not ok:
        return 1
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
