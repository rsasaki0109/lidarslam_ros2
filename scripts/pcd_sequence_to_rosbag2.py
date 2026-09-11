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
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Convert a binary XYZI PCD sequence and KITTI matrices to rosbag2 + TUM."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import shutil

import numpy as np


def read_binary_xyzi_pcd(path: Path) -> tuple[int, bytes]:
    """Return point count and packed float32 XYZI bytes from a binary PCD."""
    payload = path.read_bytes()
    marker = b'DATA binary\n'
    split = payload.find(marker)
    if split < 0:
        raise ValueError(f'{path}: only DATA binary PCD is supported')
    header = payload[:split].decode('ascii', errors='strict').splitlines()
    fields = sizes = types = counts = None
    points = None
    for line in header:
        key, *values = line.split()
        if key == 'FIELDS':
            fields = values
        elif key == 'SIZE':
            sizes = values
        elif key == 'TYPE':
            types = values
        elif key == 'COUNT':
            counts = values
        elif key == 'POINTS':
            points = int(values[0])
    expected = (['x', 'y', 'z', 'intensity'], ['4'] * 4, ['F'] * 4, ['1'] * 4)
    if (fields, sizes, types, counts) != expected or points is None:
        raise ValueError(f'{path}: expected packed float32 x y z intensity fields')
    data = payload[split + len(marker):]
    if len(data) != points * 16:
        raise ValueError(f'{path}: {len(data)} data bytes for {points} XYZI points')
    return points, data


def stride_xyzi(points: int, packed: bytes, stride: int) -> tuple[int, bytes]:
    """Apply deterministic acquisition-order stride sampling to packed XYZI."""
    if stride <= 1:
        return points, packed
    array = np.frombuffer(packed, dtype='<f4').reshape(points, 4)[::stride].copy()
    return int(array.shape[0]), array.tobytes()


def quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to normalized XYZW quaternion."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        q = [0.0, 0.0, 0.0, 0.0]
        s = math.sqrt(max(0.0, 1.0 + rotation[i, i] - rotation[j, j] - rotation[k, k])) * 2.0
        q[i] = 0.25 * s
        q[3] = (rotation[k, j] - rotation[j, k]) / s
        q[j] = (rotation[j, i] + rotation[i, j]) / s
        q[k] = (rotation[k, i] + rotation[i, k]) / s
        qx, qy, qz, qw = q
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    return qx/norm, qy/norm, qz/norm, qw/norm


def matrix_line_to_transform(
    line: str, body_from_matrix_frame: np.ndarray | None = None
) -> np.ndarray:
    values = np.array([float(value) for value in line.split()], dtype=np.float64)
    if values.size != 12:
        raise ValueError('ground-truth matrix row must contain 12 values')
    transform = np.eye(4)
    transform[:3, :4] = values.reshape(3, 4)
    if body_from_matrix_frame is not None:
        transform = transform @ body_from_matrix_frame
    return transform


def matrix_line_to_tum(
    stamp: float, line: str, body_from_matrix_frame: np.ndarray | None = None
) -> str:
    transform = matrix_line_to_transform(line, body_from_matrix_frame)
    qx, qy, qz, qw = quaternion_xyzw(transform[:3, :3])
    x, y, z = transform[:3, 3]
    return (f'{stamp:.9f} {x:.9f} {y:.9f} {z:.9f} '
            f'{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}')


def load_kitti_camera_to_velodyne(calib_path: Path) -> np.ndarray:
    """Return camera_T_velodyne inverse for KITTI odometry pose conversion."""
    for line in calib_path.read_text(encoding='utf-8').splitlines():
        if ':' not in line:
            continue
        key, values = line.split(':', 1)
        if key.strip() not in ('Tr', 'Tr_velo_to_cam'):
            continue
        raw = np.array([float(value) for value in values.split()], dtype=np.float64)
        if raw.size != 12:
            raise ValueError(f'{calib_path}: {key} must contain 12 values')
        camera_from_velodyne = np.eye(4)
        camera_from_velodyne[:3, :4] = raw.reshape(3, 4)
        return np.linalg.inv(camera_from_velodyne)
    raise ValueError(f'{calib_path}: missing Tr or Tr_velo_to_cam')


def ground_truth_body_transform(
    gt_frame: str, calib_path: Path | None
) -> np.ndarray | None:
    """Resolve an explicit GT-frame contract without double calibration."""
    if gt_frame == 'camera':
        if calib_path is None:
            raise ValueError('--gt-frame camera requires --calib')
        return load_kitti_camera_to_velodyne(calib_path)
    if calib_path is not None:
        raise ValueError('--calib requires --gt-frame camera')
    return None


def load_frames(pcd_dir: Path, timestamp_scale: float) -> list[tuple[Path, float]]:
    timestamp_path = pcd_dir / 'frame_timestamps.csv'
    rows = list(csv.DictReader(timestamp_path.open(newline='')))
    frames = []
    for row in rows:
        frame = int(row['frame_idx'])
        cloud = pcd_dir / f'{frame:08d}' / 'cloud.pcd'
        if not cloud.is_file():
            raise FileNotFoundError(cloud)
        frames.append((cloud, float(row['timestamp']) * timestamp_scale))
    return frames


def write_bag(frames: list[tuple[Path, float]], output: Path, topic: str,
              frame_id: str, force: bool, point_stride: int = 1,
              estimate_lines: list[str] | None = None,
              odom_topic: str = '/rko_lio/odometry') -> None:
    try:
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError('rosbags is required (pip install rosbags)') from exc
    if output.exists():
        if not force:
            raise FileExistsError(f'{output} exists; pass --force')
        shutil.rmtree(output)
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    Header = typestore.types['std_msgs/msg/Header']
    Time = typestore.types['builtin_interfaces/msg/Time']
    PointField = typestore.types['sensor_msgs/msg/PointField']
    PointCloud2 = typestore.types['sensor_msgs/msg/PointCloud2']
    Odometry = typestore.types['nav_msgs/msg/Odometry']
    PoseWithCovariance = typestore.types['geometry_msgs/msg/PoseWithCovariance']
    Pose = typestore.types['geometry_msgs/msg/Pose']
    Point = typestore.types['geometry_msgs/msg/Point']
    Quaternion = typestore.types['geometry_msgs/msg/Quaternion']
    TwistWithCovariance = typestore.types['geometry_msgs/msg/TwistWithCovariance']
    Twist = typestore.types['geometry_msgs/msg/Twist']
    Vector3 = typestore.types['geometry_msgs/msg/Vector3']
    fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
              for name, offset in zip(('x', 'y', 'z', 'intensity'), (0, 4, 8, 12))]
    with Writer(output, version=8) as writer:
        connection = writer.add_connection(
            topic, 'sensor_msgs/msg/PointCloud2', typestore=typestore)
        odom_connection = None
        if estimate_lines is not None:
            odom_connection = writer.add_connection(
                odom_topic, 'nav_msgs/msg/Odometry', typestore=typestore)
        for index, (path, stamp) in enumerate(frames):
            points, packed = read_binary_xyzi_pcd(path)
            points, packed = stride_xyzi(points, packed, point_stride)
            sec = int(math.floor(stamp))
            nanosec = int(round((stamp - sec) * 1e9))
            stamp_ns = sec * 1_000_000_000 + nanosec
            message = PointCloud2(
                header=Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id=frame_id),
                height=1, width=points, fields=fields, is_bigendian=False,
                point_step=16, row_step=points * 16,
                data=np.frombuffer(packed, dtype=np.uint8).copy(), is_dense=False)
            if odom_connection is not None and estimate_lines is not None:
                transform = matrix_line_to_transform(estimate_lines[index])
                qx, qy, qz, qw = quaternion_xyzw(transform[:3, :3])
                covariance = np.zeros(36, dtype=np.float64)
                odom = Odometry(
                    header=Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id='map'),
                    child_frame_id=frame_id,
                    pose=PoseWithCovariance(
                        pose=Pose(
                            position=Point(
                                x=float(transform[0, 3]), y=float(transform[1, 3]),
                                z=float(transform[2, 3])),
                            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw)),
                        covariance=covariance.copy()),
                    twist=TwistWithCovariance(
                        twist=Twist(
                            linear=Vector3(x=0.0, y=0.0, z=0.0),
                            angular=Vector3(x=0.0, y=0.0, z=0.0)),
                        covariance=covariance.copy()))
                writer.write(
                    odom_connection, stamp_ns,
                    typestore.serialize_cdr(odom, 'nav_msgs/msg/Odometry'))
            writer.write(connection, stamp_ns,
                         typestore.serialize_cdr(message, 'sensor_msgs/msg/PointCloud2'))
            if (index + 1) % 500 == 0:
                print(f'wrote {index + 1}/{len(frames)} clouds')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd-dir', type=Path, required=True)
    parser.add_argument('--gt-matrices', type=Path, required=True)
    parser.add_argument(
        '--estimate-matrices', type=Path,
        help='optional 3x4 LiDAR estimate rows written as estimate.tum')
    parser.add_argument(
        '--calib', type=Path,
        help=('KITTI calib.txt; only with --gt-frame camera. Do not pass for '
              'Localization Zoo csv_lidar_pose/evaluated_gt matrices.'))
    parser.add_argument(
        '--gt-frame', choices=('lidar', 'camera'), default='lidar',
        help=('frame represented by --gt-matrices (default: lidar). Camera '
              'requires --calib; lidar forbids it to prevent double conversion.'))
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--timestamp-scale', type=float, default=0.1)
    parser.add_argument('--topic', default='/points')
    parser.add_argument('--odom-topic', default='/rko_lio/odometry')
    parser.add_argument('--frame-id', default='velodyne')
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument(
        '--point-stride', type=int, default=1,
        help='keep every Nth input point in acquisition order (default: 1)')
    parser.add_argument('--force', action='store_true')
    parser.add_argument(
        '--ground-truth-only', action='store_true',
        help='rewrite ground_truth.tum without rebuilding the rosbag')
    args = parser.parse_args()
    if args.timestamp_scale <= 0.0 or args.max_frames < 0 or args.point_stride < 1:
        parser.error('timestamp scale/stride must be positive and max frames non-negative')
    try:
        body_from_matrix_frame = ground_truth_body_transform(
            args.gt_frame, args.calib)
    except ValueError as error:
        parser.error(str(error))
    frames = load_frames(args.pcd_dir, args.timestamp_scale)
    gt_lines = [line for line in args.gt_matrices.read_text().splitlines() if line.strip()]
    if len(frames) != len(gt_lines):
        parser.error(f'{len(frames)} frames != {len(gt_lines)} GT poses')
    if args.max_frames:
        frames = frames[:args.max_frames]
        gt_lines = gt_lines[:args.max_frames]
    estimate_lines = None
    if args.estimate_matrices:
        estimate_lines = [
            line for line in args.estimate_matrices.read_text().splitlines() if line.strip()]
        if args.max_frames:
            estimate_lines = estimate_lines[:args.max_frames]
        if len(estimate_lines) != len(frames):
            parser.error(
                f'{len(estimate_lines)} estimate poses != {len(frames)} selected frames')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    bag = args.output_dir / 'rosbag2'
    tum = args.output_dir / 'ground_truth.tum'
    if not args.ground_truth_only:
        write_bag(
            frames, bag, args.topic, args.frame_id, args.force, args.point_stride,
            estimate_lines, args.odom_topic)
    tum.write_text('\n'.join(
        matrix_line_to_tum(stamp, line, body_from_matrix_frame)
        for (_, stamp), line in zip(frames, gt_lines)) + '\n')
    if estimate_lines is not None:
        estimate_tum = args.output_dir / 'estimate.tum'
        estimate_tum.write_text('\n'.join(
            matrix_line_to_tum(stamp, line)
            for (_, stamp), line in zip(frames, estimate_lines)) + '\n')
        print(f'estimate: {estimate_tum}')
    if not args.ground_truth_only:
        print(f'rosbag2: {bag}')
    print(f'ground truth: {tum}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
