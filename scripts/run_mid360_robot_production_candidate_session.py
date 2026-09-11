#!/usr/bin/env python3
"""CLI for the MID-360 robot production-candidate session runner."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_production_candidate_session import (
    DEFAULT_PUBLIC_RKO_CONFIG,
    Mid360ProductionCandidateSessionRunner,
    PRODUCTION_CANDIDATE_SESSION_JSON,
    PRODUCTION_CANDIDATE_SESSION_MARKDOWN,
    ProductionCandidateSessionOptions,
    render_production_candidate_session_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = REPO_ROOT / 'configs' / 'mid360_robot' / 'livox_mid360_default.yaml'


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Plan or run a production-candidate MID-360 robot mapping session.'
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        '--run',
        dest='execute',
        action='store_true',
        help='Execute recording, post-check, mapping, public gate, and production gate.',
    )
    mode.add_argument(
        '--dry-run',
        dest='execute',
        action='store_false',
        help='Only write plans and commands. This is the default.',
    )
    parser.set_defaults(execute=False)
    parser.add_argument(
        '--robot-profile',
        default=str(DEFAULT_PROFILE),
        help='Robot profile YAML with expected MID-360 topics and frames.',
    )
    parser.add_argument(
        '--bag-root',
        required=True,
        help='Directory where rosbag2 output and recording sidecars are written.',
    )
    parser.add_argument('--output-dir', default='', help='Directory for session artifacts.')
    parser.add_argument('--run-id', default='', help='Recording run id and bag directory name.')
    parser.add_argument(
        '--duration-sec',
        default='600',
        help='Recording duration and production-gate minimum duration default.',
    )
    parser.add_argument('--extra-topic', action='append', default=[], help='Additional topic to record.')
    parser.add_argument('--no-tf', action='store_true', help='Do not record /tf.')
    parser.add_argument('--no-tf-static', action='store_true', help='Do not record /tf_static.')
    parser.add_argument('--storage-id', default='', help='rosbag2 storage id.')
    parser.add_argument('--max-cache-size', default='', help='ros2 bag record --max-cache-size.')
    parser.add_argument('--compression-mode', default='', help='ros2 bag record compression mode.')
    parser.add_argument('--compression-format', default='', help='ros2 bag record compression format.')
    parser.add_argument('--host-root', default='/', help='Host filesystem root for Jetson readiness.')
    parser.add_argument(
        '--skip-host-readiness',
        action='store_true',
        help='Skip Jetson host readiness. Intended only for partial rehearsals.',
    )
    parser.add_argument('--record-only', action='store_true', help='Stop after recording.')
    parser.add_argument('--skip-map', action='store_true', help='Skip mapping and production readiness.')
    parser.add_argument(
        '--skip-public-gate',
        action='store_true',
        help='Do not regenerate public RKO adoption evidence.',
    )
    parser.add_argument(
        '--from-existing-artifacts',
        action='store_true',
        help='Reuse existing host, recording, map, and adoption artifacts and run only the production gate.',
    )
    parser.add_argument(
        '--public-rko-run',
        action='store_true',
        help='Run the public clipped-bag RKO sweep before the public adoption gate.',
    )
    parser.add_argument(
        '--public-rko-sweep',
        default='',
        help='Existing public RKO sweep JSON for the adoption gate.',
    )
    parser.add_argument(
        '--public-rko-config',
        default=str(DEFAULT_PUBLIC_RKO_CONFIG),
        help='Tracked RKO-LIO config checked by the public adoption gate.',
    )
    parser.add_argument(
        '--public-rko-output-dir',
        default='',
        help='Directory for public RKO adoption-gate artifacts.',
    )
    parser.add_argument(
        '--adoption-gate',
        default='',
        help='Existing public RKO adoption-gate JSON for production readiness.',
    )
    parser.add_argument(
        '--segment-map-alignment',
        default='',
        help='Optional segment-map cloud-alignment JSON to surface in dashboard and bundle.',
    )
    parser.add_argument(
        '--allow-non-best',
        action='store_true',
        help='Accept any matching public gate-pass case instead of requiring the top-ranked case.',
    )
    parser.add_argument('--map-run-name', default='', help='Optional RKO-LIO map run_name.')
    parser.add_argument('--map-save-timeout-secs', default='', help='Timeout waiting for map files.')
    parser.add_argument('--map-startup-timeout-secs', default='', help='Timeout waiting for SLAM startup.')
    parser.add_argument('--min-bag-duration-sec', type=float, default=None)
    parser.add_argument('--min-pointcloud-hz', type=float, default=5.0)
    parser.add_argument('--min-imu-hz', type=float, default=50.0)
    parser.add_argument('--allow-warnings', action='store_true')
    parser.add_argument('--allow-public-bag', action='store_true')
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def _options_from_args(args: argparse.Namespace) -> ProductionCandidateSessionOptions:
    min_duration = args.min_bag_duration_sec
    if min_duration is None:
        min_duration = _duration_as_float(args.duration_sec, default=600.0)
    return ProductionCandidateSessionOptions(
        profile_path=Path(args.robot_profile).expanduser().resolve(),
        bag_root=Path(args.bag_root).expanduser().resolve(),
        output_dir=Path(args.output_dir).expanduser().resolve() if args.output_dir else None,
        run_id=args.run_id,
        duration_sec=args.duration_sec,
        include_tf=not args.no_tf,
        include_tf_static=not args.no_tf_static,
        extra_topics=tuple(args.extra_topic or []),
        storage_id=args.storage_id,
        max_cache_size=args.max_cache_size,
        compression_mode=args.compression_mode,
        compression_format=args.compression_format,
        host_root=Path(args.host_root).expanduser().resolve(),
        skip_host_readiness=args.skip_host_readiness,
        record_only=args.record_only,
        skip_map=args.skip_map,
        skip_public_gate=args.skip_public_gate,
        from_existing_artifacts=args.from_existing_artifacts,
        execute=args.execute,
        public_rko_run=args.public_rko_run,
        public_rko_sweep=Path(args.public_rko_sweep).expanduser().resolve() if args.public_rko_sweep else None,
        public_rko_config=Path(args.public_rko_config).expanduser().resolve(),
        public_rko_output_dir=(
            Path(args.public_rko_output_dir).expanduser().resolve()
            if args.public_rko_output_dir else None
        ),
        adoption_gate=Path(args.adoption_gate).expanduser().resolve() if args.adoption_gate else None,
        segment_map_alignment=(
            Path(args.segment_map_alignment).expanduser().resolve()
            if args.segment_map_alignment else None
        ),
        allow_non_best=args.allow_non_best,
        map_run_name=args.map_run_name,
        map_save_timeout_secs=args.map_save_timeout_secs,
        map_startup_timeout_secs=args.map_startup_timeout_secs,
        min_bag_duration_sec=max(0.0, float(min_duration)),
        min_pointcloud_hz=max(0.0, float(args.min_pointcloud_hz)),
        min_imu_hz=max(0.0, float(args.min_imu_hz)),
        allow_warnings=args.allow_warnings,
        allow_public_bag=args.allow_public_bag,
    )


def _duration_as_float(value: str, *, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return default


def main() -> int:
    """Entry point."""
    args = parse_args()
    try:
        report = Mid360ProductionCandidateSessionRunner(REPO_ROOT).run(
            _options_from_args(args),
            quiet=args.json,
        )
    except Exception as exc:
        print(f'failed to run MID-360 production-candidate session: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_production_candidate_session_markdown(report))
        print(f'{PRODUCTION_CANDIDATE_SESSION_JSON}: {report["production_candidate_session_json_path"]}')
        print(f'{PRODUCTION_CANDIDATE_SESSION_MARKDOWN}: {report["production_candidate_session_markdown_path"]}')
    return 1 if report['status'] == 'FAIL' else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
