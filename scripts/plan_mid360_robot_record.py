#!/usr/bin/env python3
"""Plan or run MID-360 robot rosbag2 recording."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_record_tools import (
    Mid360RecordManifestWriter,
    Mid360RobotRecordPlanner,
    RecordOptions,
    record_plan_to_json,
)
from lidarslam_benchmark_tools.mid360_robot_tools import RobotProfileLoader


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = REPO_ROOT / 'configs' / 'mid360_robot' / 'livox_mid360_default.yaml'


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Plan or run ros2 bag recording for a MID-360 robot.'
    )
    parser.add_argument(
        '--robot-profile',
        default=str(DEFAULT_PROFILE),
        help='Robot profile YAML with expected MID-360 topics.',
    )
    parser.add_argument(
        '--bag-root',
        default=str(REPO_ROOT / 'output' / 'mid360_robot_bags'),
        help='Directory where rosbag2 output and recording plan files are written.',
    )
    parser.add_argument('--run-id', default='', help='Recording run id and bag directory name.')
    parser.add_argument(
        '--duration-sec',
        default='',
        help='Optional timeout in seconds for ros2 bag record.',
    )
    parser.add_argument(
        '--extra-topic',
        action='append',
        default=[],
        help='Additional topic to record. Can be passed more than once.',
    )
    parser.add_argument('--no-tf', action='store_true', help='Do not record /tf.')
    parser.add_argument('--no-tf-static', action='store_true', help='Do not record /tf_static.')
    parser.add_argument('--storage-id', default='', help='rosbag2 storage id.')
    parser.add_argument('--max-cache-size', default='', help='ros2 bag record --max-cache-size.')
    parser.add_argument('--compression-mode', default='', help='ros2 bag record compression mode.')
    parser.add_argument(
        '--compression-format', default='',
        help='ros2 bag record compression format.',
    )
    parser.add_argument(
        '--dry-run', action='store_true',
        help='Write plan and print command only.',
    )
    parser.add_argument(
        '--json', action='store_true',
        help='Print machine-readable recording plan JSON.',
    )
    return parser.parse_args()


def _options_from_args(args: argparse.Namespace) -> RecordOptions:
    return RecordOptions(
        bag_root=Path(args.bag_root).expanduser().resolve(),
        run_id=args.run_id,
        include_tf=not args.no_tf,
        include_tf_static=not args.no_tf_static,
        extra_topics=tuple(args.extra_topic or []),
        storage_id=args.storage_id,
        max_cache_size=args.max_cache_size,
        compression_mode=args.compression_mode,
        compression_format=args.compression_format,
        duration_sec=args.duration_sec,
    )


def _print_human_report(plan_payload: dict[str, object], paths: dict[str, Path]) -> None:
    print('MID-360 robot recording plan:')
    print(f"  run_id: {plan_payload['run_id']}")
    print(f"  bag_path: {plan_payload['bag_path']}")
    print('  topics:')
    for topic in plan_payload['topics']:
        print(f'    - {topic}')
    print('Record command:')
    print(f"  {plan_payload['command_shell']}")
    print(f"Recording plan JSON: {paths['json']}", file=sys.stderr)
    print(f"Recording plan Markdown: {paths['markdown']}", file=sys.stderr)
    print(f"Profile snapshot: {paths['profile']}", file=sys.stderr)


def main() -> int:
    args = parse_args()
    profile = RobotProfileLoader().load(Path(args.robot_profile).expanduser().resolve())
    plan = Mid360RobotRecordPlanner().build_plan(profile, _options_from_args(args))
    writer = Mid360RecordManifestWriter()
    paths = writer.write(profile, plan)
    plan_payload = writer.build_manifest(profile, plan)

    if args.json:
        print(record_plan_to_json(profile, plan))
    else:
        _print_human_report(plan_payload, paths)

    if args.dry_run:
        return 0

    try:
        subprocess.run(plan.command, check=True, cwd=REPO_ROOT)
    except subprocess.CalledProcessError as exc:
        if args.duration_sec and exc.returncode == 124:
            return 0
        raise
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
