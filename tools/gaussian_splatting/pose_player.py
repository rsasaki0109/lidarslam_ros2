#!/usr/bin/env python3
"""Replay a camera trajectory as ROS 2 ego poses for the 3DGS sensor sim.

Phase 2 building block: a stand-in for the localiser. It walks the camera poses
of a ``transforms.json`` (the very trajectory the 3DGS scene was built from) and
publishes them as ``nav_msgs/Odometry`` on the pose topic the sensor-sim node
subscribes to. With the node's ``extrinsic`` and ``align`` left at identity this
closes the loop in the model's own frame, so the rendered stream is a faithful
trajectory flythrough -- the same path a live Autoware ``kinematic_state`` would
drive once a real driving scene (3DGS + NDT map + lanelet2 in one frame) is wired
in its place.

The pose maths (``viewmat_to_pos_quat``: world->camera viewmat to the camera's
world position + xyzw quaternion) is numpy-only and unit tested on CPU; the node
itself needs rclpy.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np  # noqa: E402
from lidarslam_benchmark_tools.gaussian_splatting.render_path import matrix_to_quat_xyzw  # noqa: E402
from lidarslam_benchmark_tools.gaussian_splatting.train_gsplat import load_transforms


# --------------------------------------------------------------------------- #
# Pure pose maths (no rclpy)
# --------------------------------------------------------------------------- #
def viewmat_to_pos_quat(viewmat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Camera world position and xyzw orientation from a world->camera viewmat.

    The camera-to-world pose is ``inv(viewmat)``; its translation is the camera
    centre in world and its rotation becomes the published orientation. Feeding
    this back through the sensor-sim node with identity extrinsic/align recovers
    the original viewmat exactly.
    """
    c2w = np.linalg.inv(np.asarray(viewmat, dtype=float))
    return c2w[:3, 3].copy(), matrix_to_quat_xyzw(c2w[:3, :3])


# --------------------------------------------------------------------------- #
# Replay node (rclpy; imported lazily)
# --------------------------------------------------------------------------- #
def _build_node():
    """Construct the node class lazily (so the module imports without rclpy)."""
    import rclpy
    from rclpy.node import Node

    class PosePlayer(Node):
        """Publish the transforms.json camera poses as Odometry on a timer."""

        def __init__(self):
            super().__init__('pose_player')
            self.declare_parameter('transforms', '')
            self.declare_parameter('topic', '/ego_odom')
            self.declare_parameter('rate', 10.0)
            self.declare_parameter('frame_id', 'map')
            self.declare_parameter('child_frame_id', 'base_link')
            self.declare_parameter('loop', True)

            transforms = self.get_parameter('transforms').value
            if not transforms:
                raise RuntimeError('parameter "transforms" is required')
            ds = load_transforms(transforms)
            self.poses = [viewmat_to_pos_quat(vm) for vm in ds['viewmats']]
            self.frame_id = self.get_parameter('frame_id').value
            self.child_frame_id = self.get_parameter('child_frame_id').value
            self.loop = bool(self.get_parameter('loop').value)
            self.idx = 0

            from nav_msgs.msg import Odometry

            self._odom_cls = Odometry
            self.pub = self.create_publisher(Odometry, self.get_parameter('topic').value, 1)
            rate = float(self.get_parameter('rate').value)
            self.timer = self.create_timer(1.0 / rate, self._tick)
            self.get_logger().info(
                f'replaying {len(self.poses)} poses at {rate} Hz '
                f'(loop={self.loop})')

        def _tick(self):
            if self.idx >= len(self.poses):
                if not self.loop:
                    self.get_logger().info('trajectory finished')
                    self.timer.cancel()
                    return
                self.idx = 0
            pos, quat = self.poses[self.idx]
            msg = self._odom_cls()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.child_frame_id = self.child_frame_id
            msg.pose.pose.position.x = float(pos[0])
            msg.pose.pose.position.y = float(pos[1])
            msg.pose.pose.position.z = float(pos[2])
            msg.pose.pose.orientation.x = float(quat[0])
            msg.pose.pose.orientation.y = float(quat[1])
            msg.pose.pose.orientation.z = float(quat[2])
            msg.pose.pose.orientation.w = float(quat[3])
            self.pub.publish(msg)
            self.idx += 1

    return rclpy, PosePlayer


def main(argv=None):
    """CLI entry point: spin the pose-replay node."""
    rclpy, PosePlayer = _build_node()
    rclpy.init(args=argv)
    node = PosePlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
