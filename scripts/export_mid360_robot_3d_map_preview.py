#!/usr/bin/env python3
"""CLI for exporting a browser-ready MID-360 3D map preview."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_3d_map_preview import (
    MAP_PREVIEW_HTML,
    MAP_PREVIEW_JSON,
    MAP_PREVIEW_OVERLAY_JSON,
    MAP_PREVIEW_PLY,
    MapPreviewOptions,
    Mid360MapPreviewExporter,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Export a one-file browser preview for a MID-360 map run.'
    )
    parser.add_argument('run_dir', help='Map run output directory or pointcloud_map parent.')
    parser.add_argument('--pointcloud-map', default='', help='Override pointcloud_map directory.')
    parser.add_argument('--trajectory', default='', help='TUM trajectory path. Auto-detected when omitted.')
    parser.add_argument('--loop-alignment', default='', help='Loop-alignment JSON. Auto-detected when omitted.')
    parser.add_argument('--output-dir', default='', help='Directory for preview artifacts.')
    parser.add_argument('--max-points', type=int, default=50000)
    parser.add_argument('--max-points-per-tile', type=int, default=5000)
    parser.add_argument('--html-max-points', type=int, default=15000)
    parser.add_argument('--max-trajectory-poses', type=int, default=2000)
    parser.add_argument('--max-loop-candidates', type=int, default=20)
    parser.add_argument('--json', action='store_true', help='Print machine-readable JSON.')
    return parser.parse_args()


def _options_from_args(args: argparse.Namespace) -> MapPreviewOptions:
    return MapPreviewOptions(
        run_dir=Path(args.run_dir).expanduser().resolve(),
        pointcloud_map_dir=(
            Path(args.pointcloud_map).expanduser().resolve()
            if args.pointcloud_map else None
        ),
        trajectory_path=Path(args.trajectory).expanduser().resolve() if args.trajectory else None,
        loop_alignment_path=(
            Path(args.loop_alignment).expanduser().resolve()
            if args.loop_alignment else None
        ),
        output_dir=Path(args.output_dir).expanduser().resolve() if args.output_dir else None,
        max_points=max(1, int(args.max_points)),
        max_points_per_tile=max(1, int(args.max_points_per_tile)),
        html_max_points=max(1, int(args.html_max_points)),
        max_trajectory_poses=max(1, int(args.max_trajectory_poses)),
        max_loop_candidates=max(1, int(args.max_loop_candidates)),
    )


def main() -> int:
    """Entry point."""
    args = parse_args()
    try:
        manifest = Mid360MapPreviewExporter().export(_options_from_args(args))
    except Exception as exc:
        print(f'failed to export MID-360 3D map preview: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(manifest, indent=2, sort_keys=True))
    else:
        artifacts = manifest['artifacts']
        print('lidar_slam 3D map preview exported')
        print(f'- status: {manifest["status"]}')
        print(f'- {MAP_PREVIEW_HTML}: {artifacts["html"]}')
        print(f'- {MAP_PREVIEW_PLY}: {artifacts["ply"]}')
        print(f'- {MAP_PREVIEW_OVERLAY_JSON}: {artifacts["overlay_json"]}')
        print(f'- {MAP_PREVIEW_JSON}: {artifacts["manifest_json"]}')
    return 1 if manifest['status'] == 'FAIL' else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
