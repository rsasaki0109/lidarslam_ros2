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
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
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

"""Convert Applanix GSOF49/50 messages into an Imu-only rosbag2 sidecar."""

from __future__ import annotations

import argparse
import math
import shutil
from pathlib import Path

import numpy as np

DEFAULT_ANGULAR_VELOCITY_VARIANCE = 0.1
DEFAULT_YAW_RATE_VARIANCE = 10.0
DEFAULT_LINEAR_ACCELERATION_VARIANCE = 0.1


def sec_nsec_from_ns(stamp_ns: int) -> tuple[int, int]:
    """Split a nanosecond stamp into ROS Time fields."""
    return stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000


def import_rosbags_modules():
    """Import rosbags lazily so helper tests do not require the package."""
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            'rosbags is required to convert Applanix bags into Imu bags',
        ) from exc
    return AnyReader, Writer, Stores, get_typestore, get_types_from_msg


def _default_applanix_msg_dirs(repo_root: Path) -> list[Path]:
    return [
        repo_root / 'Thirdparty' / 'applanix' / 'applanix_msgs' / 'msg',
        repo_root / 'applanix_msgs' / 'msg',
        Path('/tmp/applanix/applanix_msgs/msg'),
    ]


def resolve_applanix_msg_dir(requested: Path | None, repo_root: Path) -> Path:
    """Locate applanix_msgs message definitions."""
    candidates = [requested] if requested is not None else _default_applanix_msg_dirs(repo_root)
    for candidate in candidates:
        if candidate is not None and candidate.is_dir():
            return candidate
    raise RuntimeError(
        'could not find applanix_msgs message definitions; pass '
        '--applanix-msg-dir or clone https://github.com/autowarefoundation/applanix.git',
    )


def load_typestore_with_applanix(msg_dir: Path):
    """Build a typestore that knows both standard ROS 2 and applanix_msgs."""
    _, _, Stores, get_typestore, get_types_from_msg = import_rosbags_modules()
    typestore = get_typestore(Stores.LATEST)
    for path in sorted(msg_dir.glob('*.msg')):
        text = path.read_text(encoding='utf-8')
        msg_name = f'applanix_msgs/msg/{path.stem}'
        typestore.register(get_types_from_msg(text, msg_name))
    return typestore


def deg2rad(angle_deg: float) -> float:
    """Convert degrees to radians."""
    return angle_deg * math.pi / 180.0


def rotation_matrix_from_rpy(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> np.ndarray:
    """Create a rotation matrix from intrinsic XYZ roll/pitch/yaw."""
    sr = math.sin(roll_rad)
    cr = math.cos(roll_rad)
    sp = math.sin(pitch_rad)
    cp = math.cos(pitch_rad)
    sy = math.sin(yaw_rad)
    cy = math.cos(yaw_rad)

    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def quaternion_xyzw_from_rotation_matrix(
    rotation: np.ndarray,
) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a normalized XYZW quaternion."""
    trace = float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(
            1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2],
        ) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(
            1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2],
        ) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(
            1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1],
        ) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def applanix_attitude_to_ros_quaternion_xyzw(
    roll_deg: float,
    pitch_deg: float,
    heading_deg: float,
) -> tuple[float, float, float, float]:
    """Convert Applanix roll/pitch/heading into ROS ENU base_link quaternion."""
    applanix_rotation = rotation_matrix_from_rpy(
        deg2rad(roll_deg),
        deg2rad(pitch_deg),
        deg2rad(heading_deg),
    )
    enu_to_ned = np.array(
        [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
        dtype=np.float64,
    )
    applanix_to_ros = np.array(
        [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
        dtype=np.float64,
    )
    corrected_rotation = enu_to_ned @ applanix_rotation @ applanix_to_ros
    return quaternion_xyzw_from_rotation_matrix(corrected_rotation)


def applanix_angular_velocity_to_ros_rad_s(
    ang_rate_long_deg_s: float,
    ang_rate_trans_deg_s: float,
    ang_rate_down_deg_s: float,
) -> tuple[float, float, float]:
    """Convert Applanix angular rates into ROS ENU base_link axes."""
    return (
        deg2rad(ang_rate_long_deg_s),
        -deg2rad(ang_rate_trans_deg_s),
        -deg2rad(ang_rate_down_deg_s),
    )


def applanix_linear_acceleration_to_ros_m_s2(
    acc_long_m_s2: float,
    acc_trans_m_s2: float,
    acc_down_m_s2: float,
) -> tuple[float, float, float]:
    """Convert Applanix linear acceleration into ROS ENU base_link axes."""
    return (
        float(acc_long_m_s2),
        -float(acc_trans_m_s2),
        -float(acc_down_m_s2),
    )


def orientation_covariance_from_applanix_rms(
    roll_rms_deg: float,
    pitch_rms_deg: float,
    heading_rms_deg: float,
) -> np.ndarray:
    """Build orientation covariance from Applanix attitude RMS in degrees."""
    covariance = np.zeros(9, dtype=np.float64)
    covariance[0] = deg2rad(float(roll_rms_deg)) ** 2
    covariance[4] = deg2rad(float(pitch_rms_deg)) ** 2
    covariance[8] = deg2rad(float(heading_rms_deg)) ** 2
    return covariance


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Convert Applanix GSOF49/50 messages into an Imu-only rosbag2.',
    )
    parser.add_argument('--input', required=True, help='Input rosbag2 directory.')
    parser.add_argument('--output', required=True, help='Output rosbag2 directory.')
    parser.add_argument(
        '--gsof49-topic',
        default='/lvx_client/gsof/ins_solution_49',
        help='Applanix NavigationSolutionGsof49 topic.',
    )
    parser.add_argument(
        '--gsof50-topic',
        default='/lvx_client/gsof/ins_solution_rms_50',
        help='Applanix NavigationPerformanceGsof50 topic.',
    )
    parser.add_argument(
        '--output-topic',
        default='/imu',
        help='Output Imu topic.',
    )
    parser.add_argument(
        '--frame-id',
        default='base_link',
        help='Header frame_id for converted Imu messages.',
    )
    parser.add_argument(
        '--max-rms-age-sec',
        type=float,
        default=1.0,
        help=(
            'Maximum age of the latest GSOF50 sample before orientation '
            'covariance is treated as unknown.'
        ),
    )
    parser.add_argument(
        '--applanix-msg-dir',
        type=Path,
        default=None,
        help='Path to applanix_msgs/msg containing *.msg definitions.',
    )
    parser.add_argument(
        '--force',
        action='store_true',
        help='Remove the output directory if it already exists.',
    )
    return parser.parse_args()


def main() -> int:
    """Convert GSOF messages into an Imu sidecar rosbag2."""
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    if not input_path.is_dir():
        raise SystemExit(f'input bag not found: {input_path}')
    if args.max_rms_age_sec < 0.0:
        raise SystemExit('--max-rms-age-sec must be >= 0')
    if output_path.exists():
        if not args.force:
            raise SystemExit(f'output already exists: {output_path}')
        shutil.rmtree(output_path)

    repo_root = Path(__file__).resolve().parents[1]
    msg_dir = resolve_applanix_msg_dir(args.applanix_msg_dir, repo_root)
    AnyReader, Writer, _, _, _ = import_rosbags_modules()
    typestore = load_typestore_with_applanix(msg_dir)
    imu_cls = typestore.types['sensor_msgs/msg/Imu']
    vector3_cls = typestore.types['geometry_msgs/msg/Vector3']
    quaternion_cls = typestore.types['geometry_msgs/msg/Quaternion']
    header_cls = typestore.types['std_msgs/msg/Header']
    time_cls = typestore.types['builtin_interfaces/msg/Time']

    latest_rms_msg = None
    latest_rms_stamp_ns = None
    written = 0
    covariance_known = 0
    covariance_unknown = 0
    skipped_invalid = 0

    with AnyReader([input_path], default_typestore=typestore) as reader, Writer(
        output_path,
        version=8,
    ) as writer:
        connections = {conn.topic: conn for conn in reader.connections}
        if args.gsof49_topic not in connections:
            raise SystemExit(f'missing GSOF49 topic: {args.gsof49_topic}')
        gsof49_conn = connections[args.gsof49_topic]
        gsof50_conn = connections.get(args.gsof50_topic)

        output_conn = writer.add_connection(
            args.output_topic,
            'sensor_msgs/msg/Imu',
            typestore=typestore,
        )

        selected_connections = [gsof49_conn]
        if gsof50_conn is not None:
            selected_connections.append(gsof50_conn)

        for conn, stamp_ns, raw in reader.messages(connections=selected_connections):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == args.gsof50_topic:
                latest_rms_msg = msg
                latest_rms_stamp_ns = stamp_ns
                continue

            values = (
                float(msg.roll),
                float(msg.pitch),
                float(msg.heading),
                float(msg.ang_rate_long),
                float(msg.ang_rate_trans),
                float(msg.ang_rate_down),
                float(msg.acc_long),
                float(msg.acc_trans),
                float(msg.acc_down),
            )
            if not all(math.isfinite(value) for value in values):
                skipped_invalid += 1
                continue

            orientation_covariance = np.zeros(9, dtype=np.float64)
            angular_velocity_covariance = np.zeros(9, dtype=np.float64)
            linear_acceleration_covariance = np.zeros(9, dtype=np.float64)

            if latest_rms_msg is not None and latest_rms_stamp_ns is not None:
                age_sec = max(0.0, (stamp_ns - latest_rms_stamp_ns) * 1e-9)
                if age_sec <= args.max_rms_age_sec:
                    orientation_covariance = orientation_covariance_from_applanix_rms(
                        latest_rms_msg.attitude_rms_error_roll,
                        latest_rms_msg.attitude_rms_error_pitch,
                        latest_rms_msg.attitude_rms_error_heading,
                    )
                    covariance_known += 1
                else:
                    covariance_unknown += 1
            else:
                covariance_unknown += 1

            angular_velocity_covariance[0] = DEFAULT_ANGULAR_VELOCITY_VARIANCE
            angular_velocity_covariance[4] = DEFAULT_ANGULAR_VELOCITY_VARIANCE
            angular_velocity_covariance[8] = DEFAULT_YAW_RATE_VARIANCE
            linear_acceleration_covariance[0] = DEFAULT_LINEAR_ACCELERATION_VARIANCE
            linear_acceleration_covariance[4] = DEFAULT_LINEAR_ACCELERATION_VARIANCE
            linear_acceleration_covariance[8] = DEFAULT_LINEAR_ACCELERATION_VARIANCE

            qx, qy, qz, qw = applanix_attitude_to_ros_quaternion_xyzw(
                roll_deg=float(msg.roll),
                pitch_deg=float(msg.pitch),
                heading_deg=float(msg.heading),
            )
            wx, wy, wz = applanix_angular_velocity_to_ros_rad_s(
                ang_rate_long_deg_s=float(msg.ang_rate_long),
                ang_rate_trans_deg_s=float(msg.ang_rate_trans),
                ang_rate_down_deg_s=float(msg.ang_rate_down),
            )
            ax, ay, az = applanix_linear_acceleration_to_ros_m_s2(
                acc_long_m_s2=float(msg.acc_long),
                acc_trans_m_s2=float(msg.acc_trans),
                acc_down_m_s2=float(msg.acc_down),
            )

            header_sec, header_nanosec = sec_nsec_from_ns(stamp_ns)
            imu_msg = imu_cls(
                header=header_cls(
                    stamp=time_cls(sec=header_sec, nanosec=header_nanosec),
                    frame_id=args.frame_id,
                ),
                orientation=quaternion_cls(x=qx, y=qy, z=qz, w=qw),
                orientation_covariance=orientation_covariance,
                angular_velocity=vector3_cls(x=wx, y=wy, z=wz),
                angular_velocity_covariance=angular_velocity_covariance,
                linear_acceleration=vector3_cls(x=ax, y=ay, z=az),
                linear_acceleration_covariance=linear_acceleration_covariance,
            )
            writer.write(
                output_conn,
                stamp_ns,
                typestore.serialize_cdr(imu_msg, 'sensor_msgs/msg/Imu'),
            )
            written += 1

    print(f'wrote {output_path}')
    print(f'output_topic: {args.output_topic}')
    print(f'written_messages: {written}')
    print(f'covariance_known: {covariance_known}')
    print(f'covariance_unknown: {covariance_unknown}')
    print(f'skipped_invalid: {skipped_invalid}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
