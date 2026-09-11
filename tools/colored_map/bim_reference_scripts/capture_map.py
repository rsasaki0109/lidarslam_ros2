#!/usr/bin/env python3
"""Subscribe to /map (SLAM world PointCloud2), write the latest to .ply on every
callback (last write wins — no fragile SIGINT-save). Also the subscriber that
triggers map publishing."""
import sys
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

OUT = sys.argv[1]
PREFIX = sys.argv[2] if len(sys.argv) > 2 else 'slam'
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pointcloud_io as pcio  # noqa: E402


class Cap(Node):
    def __init__(self):
        super().__init__('map_capture')
        self.count = 0
        self.max_points = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, '/map', self.cb, qos)
        print('map_capture: subscribed to /map', flush=True)

    def cb(self, msg):
        self.count += 1
        pts = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(np.float64)
        if len(xyz) >= self.max_points:
            self.max_points = len(xyz)
            pcio.write_ply(OUT + '/' + PREFIX + '_map.ply', xyz, None)
            print(f'map_capture: msg #{self.count}, wrote {len(xyz)} pts', flush=True)
        else:
            print(f'map_capture: msg #{self.count}, skipped stale map '
                  f'({len(xyz)} < {self.max_points} pts)', flush=True)


def main():
    rclpy.init()
    node = Cap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
