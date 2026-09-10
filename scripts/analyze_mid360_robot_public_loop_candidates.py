#!/usr/bin/env python3
"""CLI for MID-360 public loop-candidate analysis."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_loop_alignment_analyzer import LoopAlignmentThresholds
from lidarslam_benchmark_tools.mid360_robot_public_loop_candidates import (
    PUBLIC_LOOP_CANDIDATES_JSON,
    PUBLIC_LOOP_CANDIDATES_MARKDOWN,
    PublicLoopCandidateAnalyzer,
    PublicLoopCandidateOptions,
    render_public_loop_candidate_markdown,
    write_public_loop_candidate_report,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET_ROOT = REPO_ROOT / 'datasets' / 'mid360_public'
DEFAULT_OUTPUT_DIR = REPO_ROOT / 'output' / 'mid360_public' / 'loop_candidates'


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Analyze public GT trajectories for MID-360 loop-candidate bags.'
    )
    parser.add_argument('--dataset-root', default=str(DEFAULT_DATASET_ROOT))
    parser.add_argument('--gt-zip', default='', help='Existing Hard Point Cloud gt.zip path.')
    parser.add_argument('--download-gt', action='store_true', help='Download gt.zip if missing.')
    parser.add_argument('--skip-md5', action='store_true', help='Skip gt.zip MD5 verification.')
    parser.add_argument('--include-indoor', action='store_true', help='Include non-MID360 indoor GT files.')
    parser.add_argument('--output-dir', default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument('--loop-search-radius-m', type=float, default=2.0)
    parser.add_argument('--max-loop-distance-m', type=float, default=1.0)
    parser.add_argument('--min-index-separation', type=int, default=50)
    parser.add_argument('--min-time-separation-sec', type=float, default=20.0)
    parser.add_argument('--min-loop-candidates', type=int, default=1)
    parser.add_argument('--max-loop-candidates', type=int, default=20)
    parser.add_argument('--write', action='store_true', help='Write JSON/Markdown artifacts.')
    parser.add_argument('--json', action='store_true', help='Print machine-readable JSON.')
    return parser.parse_args()


def _options_from_args(args: argparse.Namespace) -> PublicLoopCandidateOptions:
    return PublicLoopCandidateOptions(
        dataset_root=Path(args.dataset_root).expanduser().resolve(),
        output_dir=Path(args.output_dir).expanduser().resolve(),
        gt_zip=Path(args.gt_zip).expanduser().resolve() if args.gt_zip else None,
        download_gt=bool(args.download_gt),
        verify_md5=not args.skip_md5,
        include_indoor=bool(args.include_indoor),
        thresholds=LoopAlignmentThresholds(
            loop_search_radius_m=max(0.0, float(args.loop_search_radius_m)),
            max_loop_distance_m=max(0.0, float(args.max_loop_distance_m)),
            min_index_separation=max(1, int(args.min_index_separation)),
            min_time_separation_sec=max(0.0, float(args.min_time_separation_sec)),
            min_loop_candidates=max(0, int(args.min_loop_candidates)),
        ),
        max_loop_candidates=max(1, int(args.max_loop_candidates)),
    )


def main() -> int:
    """Entry point."""
    args = parse_args()
    try:
        options = _options_from_args(args)
        report = PublicLoopCandidateAnalyzer().analyze(options)
        if args.write:
            paths = write_public_loop_candidate_report(report, options.output_dir)
        else:
            paths = {}
    except Exception as exc:
        print(f'failed to analyze MID-360 public loop candidates: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_public_loop_candidate_markdown(report))
        if paths:
            print(f'{PUBLIC_LOOP_CANDIDATES_JSON}: {paths["json"]}')
            print(f'{PUBLIC_LOOP_CANDIDATES_MARKDOWN}: {paths["markdown"]}')
    return 1 if report['status'] == 'FAIL' else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
