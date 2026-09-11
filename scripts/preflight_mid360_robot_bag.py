#!/usr/bin/env python3
"""Preflight a Livox MID-360 robot bag for the Jetson legged-robot path."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_tools import (
    AutowarePreflightAdapter,
    Mid360RobotPreflight,
    RobotFrames,
    RobotProfileLoader,
    payload_to_json,
)

REPO_ROOT = Path(__file__).resolve().parents[1]


def build_mid360_robot_payload(
    bag_path: Path,
    base_frame: str,
    lidar_frame: str,
    imu_frame: str,
    robot_profile: Path | None = None,
) -> dict[str, Any]:
    """Compatibility wrapper for tests and external imports."""
    profile = RobotProfileLoader().load(robot_profile) if robot_profile else None
    return _build_preflight().build_payload(
        bag_path,
        RobotFrames(
            base_frame=base_frame or (profile.frames.base_frame if profile else 'base_link'),
            lidar_frame=lidar_frame or (profile.frames.lidar_frame if profile else 'livox_frame'),
            imu_frame=imu_frame or (profile.frames.imu_frame if profile else 'livox_frame'),
        ),
        profile=profile,
    )


def render_text_report(payload: dict[str, Any]) -> str:
    """Compatibility wrapper for tests and external imports."""
    return _build_preflight().render_text_report(payload)


def _build_preflight() -> Mid360RobotPreflight:
    return Mid360RobotPreflight(AutowarePreflightAdapter(REPO_ROOT))


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Inspect a MID-360 robot rosbag2 and emit the legged-robot launch path.'
    )
    parser.add_argument('bag', help='Path to a rosbag2 directory that contains metadata.yaml.')
    parser.add_argument(
        '--robot-profile',
        help='Robot profile YAML for expected topics and frames.',
    )
    parser.add_argument('--base-frame', default='', help='Robot body frame.')
    parser.add_argument('--lidar-frame', default='', help='MID-360 LiDAR frame.')
    parser.add_argument('--imu-frame', default='', help='MID-360 IMU frame.')
    parser.add_argument('--json', action='store_true', help='Emit machine-readable JSON.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    payload = build_mid360_robot_payload(
        Path(args.bag).expanduser().resolve(),
        args.base_frame,
        args.lidar_frame,
        args.imu_frame,
        Path(args.robot_profile).expanduser().resolve() if args.robot_profile else None,
    )
    if args.json:
        print(payload_to_json(payload))
    else:
        print(render_text_report(payload))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
