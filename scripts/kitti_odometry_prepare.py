#!/usr/bin/env python3
"""Prepare KITTI Odometry data for lidarslam RKO-LIO offline benchmark.

Reads a standard cvlibs layout::

  <dataset>/
    sequences/<id>/{velodyne/*.bin,times.txt,calib.txt}
    poses/<id>.txt   # training sequences only (00-10)

Outputs:
  - rosbag2 with sensor_msgs/PointCloud2 + sensor_msgs/Imu
  - Ground-truth TUM in Velodyne frame (for APE, training poses only)
  - Reference JSON (zero lidar->prism offset; compatible with benchmark scripts)
  - RKO-LIO YAML with IMU extrinsics from Tr_imu_to_velo when present

KITTI Odometry velodyne packs do not ship IMU. This tool synthesizes high-rate
IMU messages (near-static gyro / gravity-aligned acceleration) so the offline
node's IMU synchronisation gate is satisfied when deskew is disabled.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path

import numpy as np


def _import_rosbags():
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError('rosbags is required (pip install rosbags)') from exc
    return AnyReader, Writer, Stores, get_typestore


def parse_kitti_calib(calib_path: Path) -> dict[str, np.ndarray]:
    """Parse KITTI calib.txt into 3x4 matrices (Tr_velo_to_cam, Tr_imu_to_velo)."""
    out: dict[str, np.ndarray] = {}
    for line in calib_path.read_text(encoding='utf-8', errors='replace').splitlines():
        line = line.strip()
        if ':' not in line:
            continue
        key, rest = line.split(':', 1)
        key = key.strip()
        vals = np.array([float(x) for x in rest.split()], dtype=np.float64)
        if vals.size == 12:
            out[key] = vals.reshape(3, 4)
    return out


def mat34_to_mat4(tr: np.ndarray) -> np.ndarray:
    """3x4 [R|t] -> 4x4 homogeneous."""
    m = np.eye(4, dtype=np.float64)
    m[:3, :4] = tr
    return m


def rotation_matrix_to_quaternion_xyzw(R: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation matrix -> (qx, qy, qz, qw) unit quaternion."""
    m00, m01, m02 = float(R[0, 0]), float(R[0, 1]), float(R[0, 2])
    m10, m11, m12 = float(R[1, 0]), float(R[1, 1]), float(R[1, 2])
    m20, m21, m22 = float(R[2, 0]), float(R[2, 1]), float(R[2, 2])
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (m21 - m12) * s
        qy = (m02 - m20) * s
        qz = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / n, qy / n, qz / n, qw / n)


def se3_to_rko_extrinsic(T: np.ndarray) -> list[float]:
    """4x4 SE3 -> [qx,qy,qz,qw, x,y,z] for rko_lio YAML."""
    R = T[:3, :3]
    t = T[:3, 3]
    qx, qy, qz, qw = rotation_matrix_to_quaternion_xyzw(R)
    return [qx, qy, qz, qw, float(t[0]), float(t[1]), float(t[2])]


def world_T_velo_from_kitti_pose(
    pose_row: np.ndarray,
    T_cam_velo: np.ndarray,
) -> np.ndarray:
    """KITTI pose row (3x4 world from cam) -> 4x4 world from velodyne."""
    T_w_c = np.eye(4, dtype=np.float64)
    T_w_c[:3, :4] = pose_row.reshape(3, 4)
    return T_w_c @ T_cam_velo


def tum_line_from_T(
    stamp_sec: float,
    T: np.ndarray,
) -> str:
    """Single TUM row: t x y z qx qy qz qw (world_T_body)."""
    R = T[:3, :3]
    t = T[:3, 3]
    qx, qy, qz, qw = rotation_matrix_to_quaternion_xyzw(R)
    return (
        f'{stamp_sec:.9f} '
        f'{float(t[0]):.9f} {float(t[1]):.9f} {float(t[2]):.9f} '
        f'{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}'
    )


def read_velodyne_bin(path: Path) -> np.ndarray:
    """KITTI velodyne .bin -> Nx4 float32 (x,y,z,intensity)."""
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size % 4 != 0:
        raise ValueError(f'invalid velodyne bin size: {path}')
    return raw.reshape(-1, 4)


def imu_stamp_grid(prev_t: float, end_t: float, period_s: float) -> list[float]:
    """Inclusive-exclusive stamp list (prev_t, end_t] with floor stepping."""
    if end_t <= prev_t:
        return []
    out: list[float] = []
    t = prev_t + period_s
    while t <= end_t + 1e-9:
        out.append(float(t))
        t += period_s
    # guarantee at least one sample strictly after prev_t
    if not out:
        out.append(float(min(prev_t + period_s, end_t)))
    return out


def write_rosbag2(
    *,
    sequence_dir: Path,
    bag_out: Path,
    lidar_topic: str,
    imu_topic: str,
    imu_frame_id: str,
    imu_period_s: float,
    force: bool,
    lidar_only: bool = False,
) -> int:
    AnyReader, Writer, Stores, get_typestore = _import_rosbags()
    del AnyReader

    times_path = sequence_dir / 'times.txt'
    if not times_path.is_file():
        raise SystemExit(f'missing {times_path}')
    time_lines = [
        float(line.strip())
        for line in times_path.read_text(encoding='utf-8').splitlines()
        if line.strip()
    ]
    velo_dir = sequence_dir / 'velodyne'
    if not velo_dir.is_dir():
        raise SystemExit(f'missing {velo_dir}')

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    Header = typestore.types['std_msgs/msg/Header']
    Time = typestore.types['builtin_interfaces/msg/Time']
    PointField = typestore.types['sensor_msgs/msg/PointField']
    PointCloud2 = typestore.types['sensor_msgs/msg/PointCloud2']
    Imu = typestore.types['sensor_msgs/msg/Imu']
    Quaternion = typestore.types['geometry_msgs/msg/Quaternion']
    Vector3 = typestore.types['geometry_msgs/msg/Vector3']

    if bag_out.exists():
        if not force:
            raise SystemExit(f'output bag exists (use --force): {bag_out}')
        shutil.rmtree(bag_out)

    imu_cov = np.full(9, 0.0, dtype=np.float64)
    imu_cov[0] = -1.0

    messages: list[tuple[int, str, object]] = []

    prev_t = time_lines[0] - 0.2
    total_clouds = 0
    for idx, t_sec in enumerate(time_lines):
        bin_name = f'{idx:06d}.bin'
        bin_path = velo_dir / bin_name
        if not bin_path.is_file():
            raise SystemExit(f'missing {bin_path}')
        if not lidar_only:
            pc_end = float(t_sec)
            for s in imu_stamp_grid(prev_t, pc_end + 0.002, imu_period_s):
                sec = int(math.floor(s))
                nsec = int(round((s - sec) * 1e9))
                if nsec >= 1_000_000_000:
                    sec += 1
                    nsec -= 1_000_000_000
                stamp_ns = sec * 1_000_000_000 + nsec
                hdr = Header(
                    stamp=Time(sec=sec, nanosec=nsec),
                    frame_id=imu_frame_id,
                )
                imu_msg = Imu(
                    header=hdr,
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    orientation_covariance=imu_cov,
                    angular_velocity=Vector3(x=0.0, y=0.0, z=0.0),
                    angular_velocity_covariance=imu_cov,
                    linear_acceleration=Vector3(x=0.0, y=0.0, z=9.80665),
                    linear_acceleration_covariance=imu_cov,
                )
                messages.append((stamp_ns, 'imu', imu_msg))
            prev_t = pc_end

        pts = read_velodyne_bin(bin_path)
        n = int(pts.shape[0])
        if n == 0:
            continue
        point_step = 16
        row_step = point_step * n
        data = np.zeros(row_step, dtype=np.uint8)
        view = data.view(dtype=np.float32).reshape(n, 4)
        view[:, :] = pts.astype(np.float32, copy=False)

        sec = int(math.floor(t_sec))
        nsec = int(round((t_sec - sec) * 1e9))
        if nsec >= 1_000_000_000:
            sec += 1
            nsec -= 1_000_000_000
        stamp_ns = sec * 1_000_000_000 + nsec
        pf_x = PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1)
        pf_y = PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1)
        pf_z = PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        pf_i = PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        cloud = PointCloud2(
            header=Header(
                stamp=Time(sec=sec, nanosec=nsec),
                frame_id='velodyne',
            ),
            height=1,
            width=n,
            fields=[pf_x, pf_y, pf_z, pf_i],
            is_bigendian=False,
            point_step=point_step,
            row_step=row_step,
            data=data,
            is_dense=False,
        )
        messages.append((stamp_ns, 'lidar', cloud))
        total_clouds += 1

    if not lidar_only:
        messages.sort(key=lambda x: (x[0], 0 if x[1] == 'imu' else 1))

    with Writer(bag_out, version=8) as writer:
        conn_lidar = writer.add_connection(
            lidar_topic,
            'sensor_msgs/msg/PointCloud2',
            typestore=typestore,
        )
        conn_imu = None
        if not lidar_only:
            conn_imu = writer.add_connection(
                imu_topic,
                'sensor_msgs/msg/Imu',
                typestore=typestore,
            )
        for stamp_ns, kind, msg in messages:
            if kind == 'imu' and conn_imu is not None:
                buf = typestore.serialize_cdr(msg, 'sensor_msgs/msg/Imu')
                writer.write(conn_imu, stamp_ns, buf)
            elif kind == 'lidar':
                buf = typestore.serialize_cdr(msg, 'sensor_msgs/msg/PointCloud2')
                writer.write(conn_lidar, stamp_ns, buf)

    return total_clouds


def write_rko_yaml(
    *,
    template_path: Path,
    T_imu_velo: np.ndarray | None,
    out_path: Path,
) -> None:
    text = template_path.read_text(encoding='utf-8')
    if T_imu_velo is not None:
        # p_velo = R p_imu + t  => extrinsic imu->base(velo) for RKO
        extr = se3_to_rko_extrinsic(T_imu_velo)
        rep = (
            'extrinsic_imu2base_quat_xyzw_xyz: ['
            + ', '.join(f'{v:.12g}' for v in extr)
            + ']'
        )
        lines = text.splitlines()
        out_lines: list[str] = []
        replaced = False
        for line in lines:
            if line.strip().startswith('extrinsic_imu2base_quat_xyzw_xyz:'):
                out_lines.append(rep)
                replaced = True
            else:
                out_lines.append(line)
        if not replaced:
            out_lines.insert(0, rep)
        text = '\n'.join(out_lines) + '\n'
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text, encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description='Prepare KITTI Odometry for lidarslam benchmark')
    parser.add_argument('--dataset', required=True, type=Path, help='KITTI odometry dataset root')
    parser.add_argument('--sequence', required=True, help='Sequence id, e.g. 00')
    parser.add_argument('--output-dir', required=True, type=Path, help='Directory for artifacts')
    parser.add_argument(
        '--poses',
        type=Path,
        default=None,
        help='Explicit poses.txt (default: <dataset>/poses/<seq>.txt)',
    )
    parser.add_argument('--lidar-topic', default='/kitti/velodyne/points')
    parser.add_argument('--imu-topic', default='/kitti/imu/sample')
    parser.add_argument('--imu-period-ms', type=float, default=5.0)
    parser.add_argument('--rko-template', type=Path, default=None)
    parser.add_argument(
        '--lidar-only-bag',
        action='store_true',
        help='Write only Velodyne PointCloud2 (for LiDAR-only / lo_slam benchmarks).',
    )
    parser.add_argument('--force', action='store_true')
    args = parser.parse_args()

    dataset = args.dataset.expanduser().resolve()
    seq = args.sequence.strip()
    if not seq:
        raise SystemExit('empty --sequence')
    sequence_dir = dataset / 'sequences' / seq
    if not sequence_dir.is_dir():
        raise SystemExit(f'sequence dir not found: {sequence_dir}')

    calib_path = sequence_dir / 'calib.txt'
    if not calib_path.is_file():
        raise SystemExit(f'missing calib: {calib_path}')

    calib = parse_kitti_calib(calib_path)
    for key in ('Tr_velo_to_cam', 'Tr'):
        if key in calib:
            T_velo_cam = mat34_to_mat4(calib[key])
            break
    else:
        raise SystemExit('calib missing Tr_velo_to_cam')

    T_cam_velo = np.linalg.inv(T_velo_cam)

    T_imu_velo: np.ndarray | None
    if 'Tr_imu_to_velo' in calib:
        T_imu_velo = mat34_to_mat4(calib['Tr_imu_to_velo'])
        imu_frame_id = 'imu'
    else:
        T_imu_velo = None
        imu_frame_id = 'velodyne'

    repo_root = Path(__file__).resolve().parents[1]
    tmpl = args.rko_template or (repo_root / 'lidarslam/param/rko_lio_kitti_odometry.yaml')

    out_dir = args.output_dir.expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    bag_dir = out_dir / f'kitti_seq{seq}_rosbag2'
    tum_path = out_dir / f'kitti_seq{seq}_gt_velo.tum'
    meta_path = out_dir / f'kitti_seq{seq}_reference.json'
    rko_out = out_dir / f'kitti_seq{seq}_rko_lio.yaml'

    imu_period_s = max(1e-3, float(args.imu_period_ms) * 1e-3)
    n_clouds = write_rosbag2(
        sequence_dir=sequence_dir,
        bag_out=bag_dir,
        lidar_topic=args.lidar_topic,
        imu_topic=args.imu_topic,
        imu_frame_id=imu_frame_id,
        imu_period_s=imu_period_s,
        force=args.force,
        lidar_only=args.lidar_only_bag,
    )

    poses_path = args.poses.expanduser().resolve() if args.poses else dataset / 'poses' / f'{seq}.txt'
    pose_rows = 0
    if poses_path.is_file():
        pose_lines = [
            ln
            for ln in poses_path.read_text(encoding='utf-8').splitlines()
            if ln.strip() and not ln.lstrip().startswith('#')
        ]
        times_path = sequence_dir / 'times.txt'
        time_vals = [
            float(ln.strip())
            for ln in times_path.read_text(encoding='utf-8').splitlines()
            if ln.strip()
        ]
        if len(pose_lines) != len(time_vals):
            raise SystemExit(
                f'pose count {len(pose_lines)} != times count {len(time_vals)} '
                f'({poses_path} vs {times_path})',
            )
        tum_lines: list[str] = []
        for ln, tv in zip(pose_lines, time_vals):
            vals = np.array([float(x) for x in ln.split()], dtype=np.float64)
            if vals.size != 12:
                raise SystemExit(f'expected 12 values per pose line, got {vals.size}')
            T_w_v = world_T_velo_from_kitti_pose(vals, T_cam_velo)
            tum_lines.append(tum_line_from_T(float(tv), T_w_v))
        tum_path.write_text('\n'.join(tum_lines) + '\n', encoding='utf-8')
        pose_rows = len(tum_lines)
    else:
        tum_path = Path('')

    write_rko_yaml(template_path=tmpl, T_imu_velo=T_imu_velo, out_path=rko_out)

    meta = {
        'reference_tum_path': str(tum_path) if tum_path.is_file() else '',
        'source': 'kitti_odometry_gt_velo',
        'sequence': seq,
        'dataset_root': str(dataset),
        'poses_path': str(poses_path) if poses_path.is_file() else '',
        'pose_count': pose_rows,
        'lidar_to_prism_translation_m': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'lidar_topic': args.lidar_topic,
        'imu_topic': args.imu_topic,
        'cloud_frames': n_clouds,
        'imu_synthetic': not args.lidar_only_bag,
        'lidar_only_bag': bool(args.lidar_only_bag),
    }
    meta_path.write_text(json.dumps(meta, indent=2) + '\n', encoding='utf-8')

    print(f'rosbag2: {bag_dir}')
    print(f'rko_yaml: {rko_out}')
    print(f'reference_meta: {meta_path}')
    if tum_path.is_file():
        print(f'reference_tum: {tum_path}')
    else:
        print('reference_tum: (skipped -- no poses file; training GT only for 00-10)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
