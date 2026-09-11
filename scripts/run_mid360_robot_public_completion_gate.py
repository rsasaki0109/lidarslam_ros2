#!/usr/bin/env python3
"""CLI for the public MID-360 completion gate."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_public_completion_gate import (
    PUBLIC_COMPLETION_GATE_JSON,
    PUBLIC_COMPLETION_GATE_MARKDOWN,
    PublicCompletionGate,
    PublicCompletionGateOptions,
    render_public_completion_gate_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPO_ROOT / 'output' / 'mid360_public' / 'completion_gate'
DEFAULT_LOOP_CLOUD = (
    REPO_ROOT
    / 'output'
    / 'mid360_public'
    / 'outdoor_kidnap_loop_cloud'
    / 'mid360_robot_public_loop_cloud_analysis.json'
)
DEFAULT_SEGMENT_PLAN = (
    REPO_ROOT
    / 'output'
    / 'mid360_public'
    / 'outdoor_kidnap_segment_reset_plan'
    / 'mid360_robot_public_loop_segment_reset.json'
)
DEFAULT_SEGMENT_ROOT = REPO_ROOT / 'output' / 'mid360_public' / 'outdoor_kidnap_segment_reset'
DEFAULT_ALIGNMENT = (
    REPO_ROOT
    / 'output'
    / 'mid360_public'
    / 'outdoor_kidnap_segment_reset_alignment'
    / 'mid360_robot_public_segment_map_cloud_alignment.json'
)
DEFAULT_ADOPTION_GATE = (
    REPO_ROOT
    / 'output'
    / 'mid360_public'
    / 'rko_sweep_no_quiet_all'
    / 'mid360_robot_public_rko_adoption_gate.json'
)
DEFAULT_DASHBOARD = (
    REPO_ROOT
    / 'output'
    / 'mid360_public'
    / 'outdoor_kidnap_segment_reset_alignment'
    / 'mid360_robot_session_dashboard.html'
)


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Build the public MID-360 segment-reset completion gate.'
    )
    parser.add_argument('--output-dir', default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument('--loop-cloud', default=str(DEFAULT_LOOP_CLOUD))
    parser.add_argument('--segment-reset-plan', default=str(DEFAULT_SEGMENT_PLAN))
    parser.add_argument('--start-run-dir', default=str(DEFAULT_SEGMENT_ROOT / 'segment_000'))
    parser.add_argument('--end-run-dir', default=str(DEFAULT_SEGMENT_ROOT / 'segment_012'))
    parser.add_argument('--segment-map-alignment', default=str(DEFAULT_ALIGNMENT))
    parser.add_argument('--adoption-gate', default=str(DEFAULT_ADOPTION_GATE))
    parser.add_argument('--dashboard-html', default=str(DEFAULT_DASHBOARD))
    parser.add_argument('--min-segment-rko-poses', type=int, default=50)
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    options = PublicCompletionGateOptions(
        repo_root=REPO_ROOT,
        output_dir=Path(args.output_dir),
        loop_cloud_json=Path(args.loop_cloud),
        segment_reset_plan_json=Path(args.segment_reset_plan),
        start_run_dir=Path(args.start_run_dir),
        end_run_dir=Path(args.end_run_dir),
        segment_map_alignment_json=Path(args.segment_map_alignment),
        adoption_gate_json=Path(args.adoption_gate),
        dashboard_html=Path(args.dashboard_html),
        min_segment_rko_poses=max(1, int(args.min_segment_rko_poses)),
    )
    try:
        report = PublicCompletionGate().build_report(options)
    except Exception as exc:
        print(f'failed to build public MID-360 completion gate: {exc}', file=sys.stderr)
        return 1
    if args.json:
        print(payload_to_json(report))
    else:
        print(render_public_completion_gate_markdown(report))
        output_dir = Path(report['output_dir'])
        print(f'{PUBLIC_COMPLETION_GATE_JSON}: {output_dir / PUBLIC_COMPLETION_GATE_JSON}')
        print(f'{PUBLIC_COMPLETION_GATE_MARKDOWN}: {output_dir / PUBLIC_COMPLETION_GATE_MARKDOWN}')
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
