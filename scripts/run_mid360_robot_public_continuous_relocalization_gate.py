#!/usr/bin/env python3
"""CLI for the public MID-360 continuous RKO-LIO relocalization gate."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_continuous_relocalization_gate import (
    ContinuousRelocalizationGate,
    ContinuousRelocalizationGateOptions,
    DEFAULT_PUBLIC_LOOP_END_STAMP_SEC,
    DEFAULT_PUBLIC_LOOP_START_STAMP_SEC,
    render_continuous_relocalization_gate_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUN_DIR = (
    REPO_ROOT
    / 'output'
    / 'mid360_public'
    / 'outdoor_kidnap_ab_rko_kidnap_relocalization_final'
)
DEFAULT_OUTPUT_DIR = REPO_ROOT / 'output' / 'mid360_public' / 'continuous_relocalization_gate'
DEFAULT_TRACKED_RKO_CONFIG = REPO_ROOT / 'configs' / 'mid360_robot' / 'rko_lio_mid360_kidnap_tolerant.yaml'


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Gate public MID-360 continuous RKO-LIO kidnap relocalization evidence.'
    )
    parser.add_argument('--run-dir', default=str(DEFAULT_RUN_DIR), help='Continuous RKO-LIO run directory.')
    parser.add_argument(
        '--loop-alignment',
        default='',
        help='Loop alignment JSON. Defaults to <run-dir>/mid360_robot_loop_alignment.json.',
    )
    parser.add_argument(
        '--tracked-rko-config',
        default=str(DEFAULT_TRACKED_RKO_CONFIG),
        help='Tracked RKO kidnap recovery config YAML.',
    )
    parser.add_argument('--output-dir', default=str(DEFAULT_OUTPUT_DIR), help='Gate report output directory.')
    parser.add_argument('--min-rko-poses', type=int, default=1000)
    parser.add_argument('--min-trajectory-duration-sec', type=float, default=500.0)
    parser.add_argument('--min-relocalization-events', type=int, default=1)
    parser.add_argument('--max-loop-distance-m', type=float, default=1.0)
    parser.add_argument(
        '--gt-loop-start-stamp-sec',
        type=float,
        default=DEFAULT_PUBLIC_LOOP_START_STAMP_SEC,
    )
    parser.add_argument(
        '--gt-loop-end-stamp-sec',
        type=float,
        default=DEFAULT_PUBLIC_LOOP_END_STAMP_SEC,
    )
    parser.add_argument('--max-public-endpoint-distance-m', type=float, default=5.0)
    parser.add_argument('--max-public-endpoint-stamp-error-sec', type=float, default=1.0)
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Run the continuous relocalization gate."""
    args = parse_args()
    run_dir = Path(args.run_dir).expanduser().resolve()
    loop_alignment = (
        Path(args.loop_alignment).expanduser().resolve()
        if args.loop_alignment else run_dir / 'mid360_robot_loop_alignment.json'
    )
    try:
        report = ContinuousRelocalizationGate().build_report(
            ContinuousRelocalizationGateOptions(
                run_dir=run_dir,
                output_dir=Path(args.output_dir),
                loop_alignment_json=loop_alignment,
                tracked_rko_config=Path(args.tracked_rko_config),
                min_rko_poses=args.min_rko_poses,
                min_trajectory_duration_sec=args.min_trajectory_duration_sec,
                min_relocalization_events=args.min_relocalization_events,
                max_loop_distance_m=args.max_loop_distance_m,
                gt_loop_start_stamp_sec=args.gt_loop_start_stamp_sec,
                gt_loop_end_stamp_sec=args.gt_loop_end_stamp_sec,
                max_public_endpoint_distance_m=args.max_public_endpoint_distance_m,
                max_public_endpoint_stamp_error_sec=args.max_public_endpoint_stamp_error_sec,
            )
        )
    except Exception as exc:
        print(f'failed to build continuous relocalization gate: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_continuous_relocalization_gate_markdown(report))
    return 0 if report.get('status') == 'PASS' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
