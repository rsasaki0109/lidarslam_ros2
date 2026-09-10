#!/usr/bin/env python3
"""Check whether a MID-360 robot bag/profile is ready for mapping."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_tools import (
    AutowarePreflightAdapter,
    MapRunOptions,
    Mid360MapRunPlanner,
    Mid360ReadinessReporter,
    Mid360RobotPreflight,
    Mid360RunManifestWriter,
    RobotProfile,
    RobotProfileLoader,
    payload_to_json,
    resolve_robot_frames,
)


REPO_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Check pre-run readiness for MID-360 robot mapping.'
    )
    parser.add_argument('bag', help='Path to a rosbag2 directory that contains metadata.yaml.')
    parser.add_argument('--robot-profile', help='Robot profile YAML for expected topics and frames.')
    parser.add_argument('--base-frame', default='', help='Override robot body frame.')
    parser.add_argument('--lidar-frame', default='', help='Override MID-360 LiDAR frame.')
    parser.add_argument('--imu-frame', default='', help='Override MID-360 IMU frame.')
    parser.add_argument('--output-dir', help='Directory for readiness and run-plan outputs.')
    parser.add_argument('--write-manifest', action='store_true', help='Also write run-plan manifest.')
    parser.add_argument('--json', action='store_true', help='Print readiness JSON.')
    return parser.parse_args()


def _load_profile(args: argparse.Namespace) -> RobotProfile | None:
    if not args.robot_profile:
        return None
    return RobotProfileLoader().load(Path(args.robot_profile).expanduser().resolve())


def _output_dir(args: argparse.Namespace, bag_path: Path) -> Path:
    if args.output_dir:
        return Path(args.output_dir).expanduser().resolve()
    return REPO_ROOT / 'output' / f'mid360_robot_readiness_{bag_path.name}'


def build_readiness_payload(args: argparse.Namespace) -> tuple[dict[str, object], dict[str, object]]:
    bag_path = Path(args.bag).expanduser().resolve()
    profile = _load_profile(args)
    frames = resolve_robot_frames(
        base_frame=args.base_frame,
        lidar_frame=args.lidar_frame,
        imu_frame=args.imu_frame,
        profile=profile,
    )
    output_dir = _output_dir(args, bag_path)
    preflight = Mid360RobotPreflight(AutowarePreflightAdapter(REPO_ROOT))
    preflight_payload = preflight.build_payload(bag_path, frames, profile=profile)
    payload: dict[str, object] = {'preflight': preflight_payload}
    plan_error = ''

    try:
        plan = Mid360MapRunPlanner(REPO_ROOT).build_plan(
            bag_path=bag_path,
            payload=preflight_payload,
            frames=frames,
            options=MapRunOptions(output_dir=output_dir),
        )
        payload['plan'] = plan.to_dict()
    except ValueError as exc:
        plan_error = str(exc)

    report = Mid360ReadinessReporter().build_report(
        payload=payload,
        output_dir=output_dir,
        plan_error=plan_error,
    )
    return payload, report


def main() -> int:
    args = parse_args()
    reporter = Mid360ReadinessReporter()
    bag_path = Path(args.bag).expanduser().resolve()
    output_dir = _output_dir(args, bag_path)
    try:
        payload, report = build_readiness_payload(args)
    except Exception as exc:
        report = reporter.build_error_report(
            bag_path=bag_path,
            output_dir=output_dir,
            message=str(exc),
        )
        payload = {}

    output_dir = Path(report['output_dir'])
    paths = reporter.write(report, output_dir)
    if args.write_manifest and 'plan' in payload:
        manifest_paths = Mid360RunManifestWriter().write(payload)
        print(f"Manifest JSON: {manifest_paths['json']}", file=sys.stderr)
        print(f"Manifest Markdown: {manifest_paths['markdown']}", file=sys.stderr)

    if args.json:
        print(payload_to_json(report))
    else:
        print(reporter.render_markdown(report))
        print(f"Readiness JSON: {paths['json']}")
        print(f"Readiness Markdown: {paths['markdown']}")

    return 1 if report['status'] == 'FAIL' else 0


if __name__ == '__main__':
    raise SystemExit(main())
