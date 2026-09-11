#!/usr/bin/env python3
"""CLI for generating a small MID-360-style rosbag2 test input."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from lidarslam_benchmark_tools.mid360_robot_sample_bag import (
    Mid360SampleBagWriter,
    SampleBagConfig,
    render_summary,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Generate a synthetic MID-360 robot rosbag2 with PointCloud2, Imu, and TF.'
    )
    parser.add_argument('output', help='Output rosbag2 directory.')
    parser.add_argument('--duration-sec', type=float, default=5.0)
    parser.add_argument('--pointcloud-rate-hz', type=float, default=10.0)
    parser.add_argument('--imu-rate-hz', type=float, default=100.0)
    parser.add_argument('--point-count', type=int, default=32)
    parser.add_argument('--pointcloud-topic', default='/livox/lidar')
    parser.add_argument('--imu-topic', default='/livox/imu')
    parser.add_argument('--tf-static-topic', default='/tf_static')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--lidar-frame', default='livox_frame')
    parser.add_argument('--imu-frame', default='livox_frame')
    parser.add_argument('--no-tf-static', action='store_true', help='Do not write /tf_static.')
    parser.add_argument('--force', action='store_true', help='Overwrite an existing output bag.')
    parser.add_argument('--json', action='store_true', help='Print JSON summary.')
    return parser.parse_args()


def config_from_args(args: argparse.Namespace) -> SampleBagConfig:
    return SampleBagConfig(
        output_path=Path(args.output),
        duration_sec=args.duration_sec,
        pointcloud_rate_hz=args.pointcloud_rate_hz,
        imu_rate_hz=args.imu_rate_hz,
        point_count=args.point_count,
        pointcloud_topic=args.pointcloud_topic,
        imu_topic=args.imu_topic,
        tf_static_topic=args.tf_static_topic,
        base_frame=args.base_frame,
        lidar_frame=args.lidar_frame,
        imu_frame=args.imu_frame,
        write_tf_static=not args.no_tf_static,
        force=args.force,
    )


def main() -> int:
    args = parse_args()
    try:
        summary = Mid360SampleBagWriter(config_from_args(args)).write()
    except Exception as exc:
        print(f'failed to generate sample bag: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(summary.to_dict(), indent=2, sort_keys=True))
    else:
        print(render_summary(summary))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
