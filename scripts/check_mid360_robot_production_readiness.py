#!/usr/bin/env python3
"""Check production readiness for a Jetson MID-360 robot mapping deployment."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_production_readiness import (
    PRODUCTION_READINESS_JSON,
    PRODUCTION_READINESS_MARKDOWN,
    Mid360ProductionReadinessGate,
    ProductionReadinessInputs,
    ProductionReadinessThresholds,
    render_production_readiness_markdown,
    write_production_readiness_report,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ARTIFACT_DIR = REPO_ROOT / 'output' / 'mid360_robot_test'
DEFAULT_PUBLIC_ADOPTION_GATE = (
    REPO_ROOT
    / 'output'
    / 'mid360_public'
    / 'rko_sweep'
    / 'mid360_robot_public_rko_adoption_gate.json'
)


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Gate production readiness for Jetson MID-360 robot mapping.'
    )
    parser.add_argument(
        '--artifact-dir',
        default=str(DEFAULT_ARTIFACT_DIR),
        help='Directory containing host/readiness/recording/map artifacts.',
    )
    parser.add_argument('--host-readiness', default='', help='jetson_mid360_host_readiness.json')
    parser.add_argument('--recording-check', default='', help='mid360_robot_recording_check.json')
    parser.add_argument('--readiness', default='', help='mid360_robot_readiness.json')
    parser.add_argument('--map-diagnosis', default='', help='autoware_map_diagnosis.json')
    parser.add_argument(
        '--adoption-gate',
        default=str(DEFAULT_PUBLIC_ADOPTION_GATE),
        help='mid360_robot_public_rko_adoption_gate.json from public real-data evidence.',
    )
    parser.add_argument(
        '--output-dir',
        default='',
        help='Directory for production-readiness JSON/Markdown. Defaults to --artifact-dir.',
    )
    parser.add_argument('--min-bag-duration-sec', type=float, default=600.0)
    parser.add_argument('--min-pointcloud-hz', type=float, default=5.0)
    parser.add_argument('--min-imu-hz', type=float, default=50.0)
    parser.add_argument('--allow-warnings', action='store_true')
    parser.add_argument('--allow-public-bag', action='store_true')
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    artifact_dir = Path(args.artifact_dir).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else artifact_dir
    inputs = ProductionReadinessInputs(
        host_readiness=_path_arg(
            args.host_readiness, artifact_dir / 'jetson_mid360_host_readiness.json',
        ),
        recording_check=_path_arg(
            args.recording_check, artifact_dir / 'mid360_robot_recording_check.json',
        ),
        readiness=_path_arg(args.readiness, artifact_dir / 'mid360_robot_readiness.json'),
        map_diagnosis=_path_arg(args.map_diagnosis, artifact_dir / 'autoware_map_diagnosis.json'),
        adoption_gate=Path(args.adoption_gate).expanduser().resolve(),
        output_dir=output_dir,
    )
    thresholds = ProductionReadinessThresholds(
        min_bag_duration_sec=max(0.0, float(args.min_bag_duration_sec)),
        min_pointcloud_hz=max(0.0, float(args.min_pointcloud_hz)),
        min_imu_hz=max(0.0, float(args.min_imu_hz)),
        allow_warnings=bool(args.allow_warnings),
        allow_public_bag=bool(args.allow_public_bag),
    )
    try:
        report = Mid360ProductionReadinessGate(inputs, thresholds).build_report()
        paths = write_production_readiness_report(report, output_dir)
    except Exception as exc:
        print(f'failed to check MID-360 production readiness: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_production_readiness_markdown(report))
        print(f'{PRODUCTION_READINESS_JSON}: {paths["json"]}')
        print(f'{PRODUCTION_READINESS_MARKDOWN}: {paths["markdown"]}')
    return 0 if report['status'] == 'PASS' else 1


def _path_arg(value: str, default: Path) -> Path:
    return Path(value).expanduser().resolve() if value else default.expanduser().resolve()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
