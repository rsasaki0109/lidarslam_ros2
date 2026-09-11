#!/usr/bin/env python3
r"""Closed-loop 3DGS camera sensor simulator as a ROS 2 node (gsplat, Apache-2.0).

Phase 2 of the 3DGS-as-sim2real track: put a LiDAR-primed 3DGS scene in the
loop so a real-image-trained stack (e.g. Autoware perception) drives against a
photoreal render instead of a synthetic one. The node subscribes to the ego
pose, renders the scene from the matching camera viewpoint with a resident
``GaussianRenderer``, and publishes ``sensor_msgs/Image`` + ``CameraInfo``.

Frames: the ego pose is ``world<-base_link``; the static ``base_link<-camera``
extrinsic (``extrinsic`` param, 16 row-major floats) must target the OpenCV
optical frame the model was trained in (+x right, +y down, +z forward). If the
localiser's world frame differs from the model's, set ``align`` (16 floats,
``model_world<-pose_world``). Phase 0 found driving-scale scenes tolerate ~±1 m
lateral deviation before the render degrades -- see
``docs/research/3dgs-sim2real-gap-phase0.md``.

Run (after sourcing the workspace and with a CUDA GPU):
    python3 tools/gaussian_splatting/sensor_sim_node.py --ros-args \\
        -p ply:=output/koide_3dgs_firstlight/gsplat/pc_sh1_15k.ply \\
        -p transforms:=output/koide_3dgs_firstlight/gsplat/transforms.json \\
        -p scale:=0.5 -p pose_topic:=/localization/kinematic_state
"""

from __future__ import annotations

from pathlib import Path

from lidarslam_benchmark_tools.gaussian_splatting.gaussian_renderer import (
    GaussianRenderer, pose_to_viewmat, transform_from_pos_quat)
import numpy as np
from lidarslam_benchmark_tools.gaussian_splatting.render_path import scale_intrinsics  # noqa: E402
from lidarslam_benchmark_tools.gaussian_splatting.train_gsplat import load_transforms  # noqa: E402


def _mat4(values, name: str) -> np.ndarray:
    """Validate a 16-element row-major list into a 4x4 matrix."""
    arr = np.asarray(values, dtype=float)
    if arr.size != 16:
        raise ValueError(f'{name} must have 16 elements, got {arr.size}')
    return arr.reshape(4, 4)


def _camera_info(K: np.ndarray, width: int, height: int, frame_id: str):
    """Build a plumb_bob CameraInfo with no distortion from intrinsics K."""
    from sensor_msgs.msg import CameraInfo

    info = CameraInfo()
    info.width = int(width)
    info.height = int(height)
    info.distortion_model = 'plumb_bob'
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.k = [float(v) for v in K.reshape(-1)]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [K[0, 0], K[0, 1], K[0, 2], 0.0,
              K[1, 0], K[1, 1], K[1, 2], 0.0,
              0.0, 0.0, 1.0, 0.0]
    info.header.frame_id = frame_id
    return info


def _build_node():
    """Construct the node class lazily (so the module imports without rclpy)."""
    import rclpy
    from rclpy.node import Node

    class SensorSimNode(Node):
        """Render the resident 3DGS scene at each incoming ego pose."""

        def __init__(self):
            super().__init__('gaussian_sensor_sim')
            self.declare_parameter('ply', '')
            self.declare_parameter('transforms', '')
            self.declare_parameter('scale', 0.5)
            self.declare_parameter('pose_topic', '/localization/kinematic_state')
            self.declare_parameter('pose_type', 'odometry')  # or 'pose_stamped'
            self.declare_parameter('image_topic', '/sensor_sim/image_raw')
            self.declare_parameter('camera_info_topic', '/sensor_sim/camera_info')
            self.declare_parameter('frame_id', 'camera_optical')
            self.declare_parameter('extrinsic', list(np.eye(4).reshape(-1)))
            self.declare_parameter('align', list(np.eye(4).reshape(-1)))

            ply = self.get_parameter('ply').value
            transforms = self.get_parameter('transforms').value
            if not ply or not transforms:
                raise RuntimeError('parameters "ply" and "transforms" are required')
            scale = float(self.get_parameter('scale').value)
            self.frame_id = self.get_parameter('frame_id').value
            self.t_base_cam = _mat4(self.get_parameter('extrinsic').value, 'extrinsic')
            self.t_align = _mat4(self.get_parameter('align').value, 'align')

            ds = load_transforms(transforms)
            self.K, self.width, self.height = scale_intrinsics(
                ds['K'], ds['width'], ds['height'], scale)
            self.renderer = GaussianRenderer(ply)
            self.get_logger().info(
                f'loaded {self.renderer.num_gaussians} gaussians, rendering '
                f'{self.width}x{self.height} (sh_degree={self.renderer.sh_degree})')

            from sensor_msgs.msg import CameraInfo, Image
            from cv_bridge import CvBridge

            self.bridge = CvBridge()
            self.pub_img = self.create_publisher(
                Image, self.get_parameter('image_topic').value, 1)
            self.pub_info = self.create_publisher(
                CameraInfo, self.get_parameter('camera_info_topic').value, 1)

            topic = self.get_parameter('pose_topic').value
            if self.get_parameter('pose_type').value == 'pose_stamped':
                from geometry_msgs.msg import PoseStamped

                self.create_subscription(PoseStamped, topic,
                                         self._on_pose_stamped, 10)
            else:
                from nav_msgs.msg import Odometry

                self.create_subscription(Odometry, topic, self._on_odometry, 10)
            self.get_logger().info(f'subscribed to {topic}, publishing renders')

        def _on_odometry(self, msg):
            self._render_and_publish(msg.pose.pose, msg.header.stamp)

        def _on_pose_stamped(self, msg):
            self._render_and_publish(msg.pose, msg.header.stamp)

        def _render_and_publish(self, pose, stamp):
            p, q = pose.position, pose.orientation
            t_world_base = transform_from_pos_quat(
                [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            viewmat = pose_to_viewmat(t_world_base, self.t_base_cam, self.t_align)
            rgb = self.renderer.render(viewmat, self.K, self.width, self.height)
            img = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
            img.header.stamp = stamp
            img.header.frame_id = self.frame_id
            info = _camera_info(self.K, self.width, self.height, self.frame_id)
            info.header.stamp = stamp
            self.pub_img.publish(img)
            self.pub_info.publish(info)

    return rclpy, SensorSimNode


def main(argv=None):
    """CLI entry point: spin the sensor-sim node."""
    rclpy, SensorSimNode = _build_node()
    rclpy.init(args=argv)
    node = SensorSimNode()
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
