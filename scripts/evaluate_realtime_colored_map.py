#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""Capture one realtime coloured-map message and report RGB coverage."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def decode_packed_rgb(values: np.ndarray) -> np.ndarray:
    """Decode PCL packed RGB stored as float32 or uint32."""
    packed = np.asarray(values)
    if packed.dtype.kind == 'f':
        packed = np.ascontiguousarray(packed, dtype=np.float32).view(np.uint32)
    else:
        packed = packed.astype(np.uint32, copy=False)
    return np.stack([
        (packed >> 16) & 0xff,
        (packed >> 8) & 0xff,
        packed & 0xff,
    ], axis=1).astype(np.uint8)


def summarize(rgb: np.ndarray, default_rgb=(128, 128, 128)) -> dict:
    """Summarize confirmed colour coverage and chroma."""
    colours = np.asarray(rgb, dtype=np.uint8)
    confirmed = np.any(colours != np.asarray(default_rgb, dtype=np.uint8), axis=1)
    chroma = np.ptp(colours[confirmed].astype(np.int16), axis=1)
    return {
        'points': int(len(colours)),
        'confirmed': int(confirmed.sum()),
        'coverage': float(confirmed.mean()) if len(confirmed) else 0.0,
        'chroma_mean': float(chroma.mean()) if len(chroma) else 0.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/colored_map')
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    rclpy.init()
    node = Node('evaluate_realtime_colored_map')
    result = {}

    def callback(message: PointCloud2) -> None:
        rows = point_cloud2.read_points_numpy(
            message, field_names=('rgb',), skip_nans=False)
        report = summarize(decode_packed_rgb(rows.reshape(-1)))
        report.update({
            'topic': args.topic,
            'frame_id': message.header.frame_id,
            'stamp': message.header.stamp.sec +
            message.header.stamp.nanosec * 1.0e-9,
        })
        result.update(report)

    node.create_subscription(
        PointCloud2, args.topic, callback, qos_profile_sensor_data)
    deadline = node.get_clock().now().nanoseconds * 1.0e-9 + args.timeout
    while rclpy.ok() and not result:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.get_clock().now().nanoseconds * 1.0e-9 >= deadline:
            break
    node.destroy_node()
    rclpy.shutdown()
    if not result:
        raise SystemExit(f'no {args.topic} message within {args.timeout:g}s')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
