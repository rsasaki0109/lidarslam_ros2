#!/usr/bin/env python3
"""Validate and print a MID-360 robot profile."""

from __future__ import annotations

import argparse
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_tools import (
    RobotProfileLoader,
    payload_to_json,
    render_robot_profile_report,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Validate a MID-360 robot profile YAML.')
    parser.add_argument('profile', help='Path to a robot profile YAML.')
    parser.add_argument('--json', action='store_true', help='Emit normalized profile JSON.')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    profile = RobotProfileLoader().load(Path(args.profile).expanduser().resolve())
    if args.json:
        print(payload_to_json(profile.to_dict()))
    else:
        print(render_robot_profile_report(profile))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
