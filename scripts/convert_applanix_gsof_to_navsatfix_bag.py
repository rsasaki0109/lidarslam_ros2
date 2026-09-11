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

"""Convert Applanix GSOF49/50 messages into a NavSatFix-only rosbag2."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import numpy as np

APPLANIX_FIX_NOT_AVAILABLE = 0
APPLANIX_GNSS_SPS_MODE = 1
APPLANIX_DIFFERENTIAL_GPS_SPS = 2
APPLANIX_GPS_PPS_MODE = 3
APPLANIX_FIXED_RTK_MODE = 4
APPLANIX_FLOAT_RTK = 5
APPLANIX_DIRECT_GEOREFERENCING_MODE = 6
APPLANIX_GNSS_UNKNOWN = 7

NAVSAT_STATUS_NO_FIX = -1
NAVSAT_STATUS_FIX = 0
NAVSAT_STATUS_SBAS_FIX = 1
NAVSAT_STATUS_GBAS_FIX = 2
NAVSAT_STATUS_UNKNOWN = -2

NAVSAT_FIX_COVARIANCE_TYPE_UNKNOWN = 0
NAVSAT_FIX_COVARIANCE_TYPE_DIAGONAL_KNOWN = 2


def sec_nsec_from_ns(stamp_ns: int) -> tuple[int, int]:
    """Split a nanosecond stamp into ROS Time fields."""
    return stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000


def import_rosbags_modules():
    """Import rosbags lazily so pure helper tests do not require the package."""
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            'rosbags is required to convert Applanix bags into NavSatFix bags',
        ) from exc
    return AnyReader, Writer, Stores, get_typestore, get_types_from_msg


def applanix_gnss_status_to_navsat_status(gnss_status: int) -> int:
    """Map Applanix GNSS quality to the nearest NavSatStatus bucket."""
    if gnss_status in (APPLANIX_FIX_NOT_AVAILABLE,):
        return NAVSAT_STATUS_NO_FIX
    if gnss_status in (APPLANIX_GNSS_UNKNOWN,):
        return NAVSAT_STATUS_UNKNOWN
    if gnss_status in (
        APPLANIX_FIXED_RTK_MODE,
        APPLANIX_FLOAT_RTK,
        APPLANIX_DIRECT_GEOREFERENCING_MODE,
    ):
        return NAVSAT_STATUS_GBAS_FIX
    if gnss_status in (APPLANIX_DIFFERENTIAL_GPS_SPS,):
        return NAVSAT_STATUS_SBAS_FIX
    return NAVSAT_STATUS_FIX


def covariance_from_applanix_rms(
    east_rms_m: float,
    north_rms_m: float,
    down_rms_m: float,
) -> tuple[np.ndarray, int]:
    """Convert Applanix NED RMS values into NavSatFix ENU covariance."""
    covariance = np.zeros(9, dtype=np.float64)
    covariance[0] = east_rms_m * east_rms_m
    covariance[4] = north_rms_m * north_rms_m
    covariance[8] = down_rms_m * down_rms_m
    return covariance, NAVSAT_FIX_COVARIANCE_TYPE_DIAGONAL_KNOWN


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


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Convert Applanix GSOF49/50 messages into a NavSatFix-only rosbag2.',
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
        default='/gnss/fix',
        help='Output NavSatFix topic.',
    )
    parser.add_argument(
        '--frame-id',
        default='gnss',
        help='Header frame_id for converted NavSatFix messages.',
    )
    parser.add_argument(
        '--max-rms-age-sec',
        type=float,
        default=1.0,
        help='Maximum age of the latest GSOF50 sample before covariance is treated as unknown.',
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
    """Convert GSOF messages into a NavSatFix sidecar rosbag2."""
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
    navsat_fix_cls = typestore.types['sensor_msgs/msg/NavSatFix']
    navsat_status_cls = typestore.types['sensor_msgs/msg/NavSatStatus']
    header_cls = typestore.types['std_msgs/msg/Header']
    time_cls = typestore.types['builtin_interfaces/msg/Time']

    latest_rms_msg = None
    latest_rms_stamp_ns = None
    written = 0
    skipped_no_fix = 0
    covariance_known = 0
    covariance_unknown = 0

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
            'sensor_msgs/msg/NavSatFix',
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

            gnss_status = int(msg.status.gnss)
            if gnss_status in (APPLANIX_FIX_NOT_AVAILABLE, APPLANIX_GNSS_UNKNOWN):
                skipped_no_fix += 1
                continue

            covariance = np.zeros(9, dtype=np.float64)
            covariance_type = NAVSAT_FIX_COVARIANCE_TYPE_UNKNOWN
            if latest_rms_msg is not None and latest_rms_stamp_ns is not None:
                age_sec = max(0.0, (stamp_ns - latest_rms_stamp_ns) * 1e-9)
                if age_sec <= args.max_rms_age_sec:
                    covariance, covariance_type = covariance_from_applanix_rms(
                        east_rms_m=float(latest_rms_msg.pos_rms_error.east),
                        north_rms_m=float(latest_rms_msg.pos_rms_error.north),
                        down_rms_m=float(latest_rms_msg.pos_rms_error.down),
                    )

            if covariance_type == NAVSAT_FIX_COVARIANCE_TYPE_UNKNOWN:
                covariance_unknown += 1
            else:
                covariance_known += 1

            # Use the rosbag timestamp for the published NavSatFix header so the
            # converted sidecar stays aligned with the bag's PointCloud2/IMU
            # timestamps. Some Applanix raw messages carry a different internal
            # time base in msg.header.stamp.
            header_sec, header_nanosec = sec_nsec_from_ns(stamp_ns)
            navsat_msg = navsat_fix_cls(
                header=header_cls(
                    stamp=time_cls(sec=header_sec, nanosec=header_nanosec),
                    frame_id=args.frame_id,
                ),
                status=navsat_status_cls(
                    status=applanix_gnss_status_to_navsat_status(gnss_status),
                    service=navsat_status_cls.SERVICE_GPS,
                ),
                latitude=float(msg.lla.latitude),
                longitude=float(msg.lla.longitude),
                altitude=float(msg.lla.altitude),
                position_covariance=covariance,
                position_covariance_type=covariance_type,
            )
            writer.write(
                output_conn,
                stamp_ns,
                typestore.serialize_cdr(navsat_msg, 'sensor_msgs/msg/NavSatFix'),
            )
            written += 1

    print(f'wrote {output_path}')
    print(f'output_topic: {args.output_topic}')
    print(f'written_messages: {written}')
    print(f'skipped_no_fix: {skipped_no_fix}')
    print(f'covariance_known: {covariance_known}')
    print(f'covariance_unknown: {covariance_unknown}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
