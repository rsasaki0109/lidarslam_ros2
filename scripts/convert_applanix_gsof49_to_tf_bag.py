#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Convert Applanix GSOF49 messages into a /tf rosbag2 sidecar."""

from __future__ import annotations

import argparse
import math
import shutil
from pathlib import Path


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def sec_nsec_from_ns(stamp_ns: int) -> tuple[int, int]:
    """Split a nanosecond stamp into ROS Time fields."""
    return stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000


def import_rosbags_modules():
    """Import rosbags lazily so helper tests stay lightweight."""
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            'rosbags is required to convert Applanix bags into TF bags',
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


def lla_to_ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> tuple[float, float, float]:
    """Convert WGS84 LLA into ECEF."""
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (radius + altitude_m) * cos_lat * cos_lon
    y = (radius + altitude_m) * cos_lat * sin_lon
    z = (radius * (1.0 - WGS84_E2) + altitude_m) * sin_lat
    return x, y, z


def ecef_to_enu(
    x: float,
    y: float,
    z: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    origin_altitude_m: float,
) -> tuple[float, float, float]:
    """Convert ECEF into local ENU relative to the origin LLA."""
    ox, oy, oz = lla_to_ecef(
        origin_latitude_deg,
        origin_longitude_deg,
        origin_altitude_m,
    )
    dx = x - ox
    dy = y - oy
    dz = z - oz

    lat0 = math.radians(origin_latitude_deg)
    lon0 = math.radians(origin_longitude_deg)
    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    east = -sin_lon0 * dx + cos_lon0 * dy
    north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
    return east, north, up


def lla_to_enu(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    origin_altitude_m: float,
) -> tuple[float, float, float]:
    """Convert WGS84 LLA into local ENU relative to the origin LLA."""
    return ecef_to_enu(
        *lla_to_ecef(latitude_deg, longitude_deg, altitude_m),
        origin_latitude_deg,
        origin_longitude_deg,
        origin_altitude_m,
    )


def heading_deg_to_enu_yaw_deg(heading_deg: float) -> float:
    """Convert north-clockwise heading into ENU yaw."""
    yaw_deg = 90.0 - heading_deg
    while yaw_deg <= -180.0:
        yaw_deg += 360.0
    while yaw_deg > 180.0:
        yaw_deg -= 360.0
    return yaw_deg


def rpy_deg_to_quaternion(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> tuple[float, float, float, float]:
    """Convert roll/pitch/yaw degrees into an XYZW quaternion."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def integrate_planar_velocity(
    east_m: float,
    north_m: float,
    velocity_east_mps: float,
    velocity_north_mps: float,
    previous_stamp_ns: int | None,
    current_stamp_ns: int,
) -> tuple[float, float]:
    """Integrate planar velocity between usable GSOF49 samples."""
    if previous_stamp_ns is None:
        return east_m, north_m
    dt = max(0.0, (current_stamp_ns - previous_stamp_ns) * 1e-9)
    return (
        east_m + velocity_east_mps * dt,
        north_m + velocity_north_mps * dt,
    )


def is_usable_fix(
    *,
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    imu_alignment: int,
    gnss_status: int,
    aligned_value: int,
    fix_not_available_value: int,
    gnss_unknown_value: int,
) -> tuple[bool, str]:
    """Return whether a GSOF49 sample is usable as a reference pose."""
    if not math.isfinite(latitude_deg) or not math.isfinite(longitude_deg):
        return False, 'non_finite_lla'
    if abs(latitude_deg) < 1e-9 and abs(longitude_deg) < 1e-9:
        return False, 'zero_origin'
    if not math.isfinite(altitude_m):
        return False, 'non_finite_altitude'
    if imu_alignment < aligned_value:
        return False, 'imu_not_aligned'
    if gnss_status == fix_not_available_value:
        return False, 'fix_not_available'
    if gnss_status == gnss_unknown_value:
        return False, 'gnss_unknown'
    return True, 'ok'


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Convert Applanix GSOF49 messages into a /tf rosbag2 sidecar.',
    )
    parser.add_argument('--input', required=True, help='Input rosbag2 directory.')
    parser.add_argument('--output', required=True, help='Output rosbag2 directory.')
    parser.add_argument(
        '--topic',
        default='/lvx_client/gsof/ins_solution_49',
        help='Applanix NavigationSolutionGsof49 topic.',
    )
    parser.add_argument(
        '--output-topic',
        default='/tf',
        help='Output TF topic (default: /tf).',
    )
    parser.add_argument(
        '--odom-frame-id',
        default='odom',
        help='Parent frame_id for the generated TF (default: odom).',
    )
    parser.add_argument(
        '--child-frame-id',
        default='base_link',
        help='Child frame_id for the generated TF (default: base_link).',
    )
    parser.add_argument(
        '--planar',
        action='store_true',
        help='Publish planar odom only (zero z, roll, and pitch).',
    )
    parser.add_argument(
        '--integrate-velocity-planar',
        action='store_true',
        help='Integrate planar NED velocity into a smoother odom trajectory instead of using absolute LLA.',
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
    """Convert GSOF49 into a TF sidecar rosbag2."""
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    if not input_path.is_dir():
        raise SystemExit(f'input bag not found: {input_path}')
    if output_path.exists():
        if not args.force:
            raise SystemExit(f'output already exists: {output_path}')
        shutil.rmtree(output_path)

    repo_root = Path(__file__).resolve().parents[1]
    msg_dir = resolve_applanix_msg_dir(args.applanix_msg_dir, repo_root)
    AnyReader, Writer, _, _, _ = import_rosbags_modules()
    typestore = load_typestore_with_applanix(msg_dir)
    tf_message_cls = typestore.types['tf2_msgs/msg/TFMessage']
    transform_stamped_cls = typestore.types['geometry_msgs/msg/TransformStamped']
    transform_cls = typestore.types['geometry_msgs/msg/Transform']
    vector3_cls = typestore.types['geometry_msgs/msg/Vector3']
    quaternion_cls = typestore.types['geometry_msgs/msg/Quaternion']
    header_cls = typestore.types['std_msgs/msg/Header']
    time_cls = typestore.types['builtin_interfaces/msg/Time']

    origin = None
    integrated_east = 0.0
    integrated_north = 0.0
    previous_stamp_ns = None
    written = 0
    skipped = 0

    with AnyReader([input_path], default_typestore=typestore) as reader, Writer(
        output_path,
        version=8,
    ) as writer:
        connections = {conn.topic: conn for conn in reader.connections}
        if args.topic not in connections:
            raise SystemExit(f'missing GSOF49 topic: {args.topic}')
        gsof49_conn = connections[args.topic]
        output_conn = writer.add_connection(
            args.output_topic,
            'tf2_msgs/msg/TFMessage',
            typestore=typestore,
        )

        for conn, stamp_ns, raw in reader.messages(connections=[gsof49_conn]):
            msg = reader.deserialize(raw, conn.msgtype)
            status = msg.status
            lla = msg.lla
            usable, _ = is_usable_fix(
                latitude_deg=float(lla.latitude),
                longitude_deg=float(lla.longitude),
                altitude_m=float(lla.altitude),
                imu_alignment=int(status.imu_alignment),
                gnss_status=int(status.gnss),
                aligned_value=int(status.ALIGNED),
                fix_not_available_value=int(status.FIX_NOT_AVAILABLE),
                gnss_unknown_value=int(status.GNSS_UNKNOWN),
            )
            if not usable:
                skipped += 1
                continue

            if args.integrate_velocity_planar:
                velocity = msg.velocity
                integrated_east, integrated_north = integrate_planar_velocity(
                    east_m=integrated_east,
                    north_m=integrated_north,
                    velocity_east_mps=float(velocity.east),
                    velocity_north_mps=float(velocity.north),
                    previous_stamp_ns=previous_stamp_ns,
                    current_stamp_ns=stamp_ns,
                )
                east = integrated_east
                north = integrated_north
                up = 0.0
            else:
                if origin is None:
                    origin = (
                        float(lla.latitude),
                        float(lla.longitude),
                        float(lla.altitude),
                    )

                east, north, up = lla_to_enu(
                    latitude_deg=float(lla.latitude),
                    longitude_deg=float(lla.longitude),
                    altitude_m=float(lla.altitude),
                    origin_latitude_deg=origin[0],
                    origin_longitude_deg=origin[1],
                    origin_altitude_m=origin[2],
                )
            roll_deg = float(msg.roll)
            pitch_deg = float(msg.pitch)
            yaw_deg = heading_deg_to_enu_yaw_deg(float(msg.heading))
            if args.planar:
                up = 0.0
                roll_deg = 0.0
                pitch_deg = 0.0
            qx, qy, qz, qw = rpy_deg_to_quaternion(
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
                yaw_deg=yaw_deg,
            )
            sec, nanosec = sec_nsec_from_ns(stamp_ns)
            tf_msg = tf_message_cls(
                transforms=[
                    transform_stamped_cls(
                        header=header_cls(
                            stamp=time_cls(sec=sec, nanosec=nanosec),
                            frame_id=args.odom_frame_id,
                        ),
                        child_frame_id=args.child_frame_id,
                        transform=transform_cls(
                            translation=vector3_cls(x=east, y=north, z=up),
                            rotation=quaternion_cls(x=qx, y=qy, z=qz, w=qw),
                        ),
                    ),
                ],
            )
            writer.write(
                output_conn,
                stamp_ns,
                typestore.serialize_cdr(tf_msg, 'tf2_msgs/msg/TFMessage'),
            )
            previous_stamp_ns = stamp_ns
            written += 1

    print(f'wrote {output_path}')
    print(f'output_topic: {args.output_topic}')
    print(f'written_messages: {written}')
    print(f'skipped_invalid: {skipped}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
