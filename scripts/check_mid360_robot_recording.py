#!/usr/bin/env python3
"""Check a recorded MID-360 robot bag and prepare the map dry-run plan."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_recording_check_tools import (
    Mid360RecordingCheckReporter,
    RecordingCheckInputs,
    auto_profile_snapshot_path,
    auto_record_plan_path,
    build_readiness_for_recording,
    load_recording_plan,
    write_readiness_artifacts,
)
from lidarslam_benchmark_tools.mid360_robot_tools import (
    Mid360ReadinessReporter,
    RobotProfileLoader,
    payload_to_json,
    resolve_robot_frames,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = REPO_ROOT / 'configs' / 'mid360_robot' / 'livox_mid360_default.yaml'


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Post-check a recorded MID-360 robot rosbag2.'
    )
    parser.add_argument('--bag', required=True, help='Recorded rosbag2 directory.')
    parser.add_argument(
        '--robot-profile',
        default='',
        help='Robot profile YAML. Defaults to <bag_parent>/<bag_name>_profile.yaml when present.',
    )
    parser.add_argument(
        '--record-plan',
        default='',
        help='Recording plan JSON. Defaults to <bag_parent>/<bag_name>_record_plan.json.',
    )
    parser.add_argument('--base-frame', default='', help='Override robot body frame.')
    parser.add_argument('--lidar-frame', default='', help='Override MID-360 LiDAR frame.')
    parser.add_argument('--imu-frame', default='', help='Override MID-360 IMU frame.')
    parser.add_argument(
        '--output-dir',
        help='Directory for recording check, readiness, and run-plan outputs.',
    )
    parser.add_argument('--json', action='store_true', help='Print recording-check JSON.')
    return parser.parse_args()


def _output_dir(args: argparse.Namespace, bag_path: Path) -> Path:
    if args.output_dir:
        return Path(args.output_dir).expanduser().resolve()
    return REPO_ROOT / 'output' / f'mid360_robot_recording_check_{bag_path.name}'


def _profile_path(args: argparse.Namespace, bag_path: Path) -> Path:
    if args.robot_profile:
        return Path(args.robot_profile).expanduser().resolve()
    snapshot = auto_profile_snapshot_path(bag_path)
    if snapshot.is_file():
        return snapshot.resolve()
    return DEFAULT_PROFILE.resolve()


def _record_plan_path(args: argparse.Namespace, bag_path: Path) -> Path:
    if args.record_plan:
        return Path(args.record_plan).expanduser().resolve()
    return auto_record_plan_path(bag_path).resolve()


def main() -> int:
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    output_dir = _output_dir(args, bag_path)
    profile_path: Path | None = None
    record_plan_path: Path | None = None
    reporter = Mid360RecordingCheckReporter()

    try:
        profile_path = _profile_path(args, bag_path)
        record_plan_path = _record_plan_path(args, bag_path)
        profile = RobotProfileLoader().load(profile_path)
        frames = resolve_robot_frames(
            base_frame=args.base_frame,
            lidar_frame=args.lidar_frame,
            imu_frame=args.imu_frame,
            profile=profile,
        )
        recording_plan = load_recording_plan(record_plan_path)
        inputs = RecordingCheckInputs(
            bag_path=bag_path,
            output_dir=output_dir,
            profile_path=profile_path,
            record_plan_path=record_plan_path,
        )

        try:
            payload, readiness_report, _ = build_readiness_for_recording(
                repo_root=REPO_ROOT,
                bag_path=bag_path,
                output_dir=output_dir,
                profile=profile,
                frames=frames,
            )
        except Exception as exc:
            payload = {}
            readiness_report = Mid360ReadinessReporter().build_error_report(
                bag_path=bag_path,
                output_dir=output_dir,
                message=str(exc),
            )

        paths = write_readiness_artifacts(payload, readiness_report, output_dir)
        report = reporter.build_report(
            inputs=inputs,
            profile=profile,
            recording_plan=recording_plan,
            readiness_report=readiness_report,
            payload=payload,
            paths=paths,
        )
    except Exception as exc:
        report = reporter.build_error_report(
            bag_path=bag_path,
            output_dir=output_dir,
            profile_path=profile_path,
            record_plan_path=record_plan_path,
            message=str(exc),
        )

    paths = reporter.write(report, output_dir)
    if args.json:
        print(payload_to_json(report))
    else:
        print(reporter.render_markdown(report))
        print(f"Recording check JSON: {paths['json']}")
        print(f"Recording check Markdown: {paths['markdown']}")

    return 1 if report['status'] == 'FAIL' else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
