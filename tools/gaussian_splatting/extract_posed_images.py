#!/usr/bin/env python3
"""Extract posed images from a rosbag2 for the 3DGS map deliverable.

Reads a rosbag2 (image topic + ``camera_info`` topic), resolves each image's
``world <- camera_optical`` pose from a SLAM TUM trajectory and a static
``body <- camera_optical`` extrinsic, writes the images plus a Nerfstudio
``transforms.json`` that gsplat (Apache-2.0) can train on.

Design: the pose/extrinsic/association logic lives in pure, numpy-only
functions (``resolve_world_T_camera``, ``parse_extrinsic_dict``,
``ros_stamp_to_seconds``) so it runs in the ament pytest harness with no ROS.
The rosbag2 reading and image decoding (``rosbag2_py`` / ``cv_bridge``) are
imported lazily inside ``main`` and the reader helpers, so importing this
module for testing never requires a ROS environment.

See ``docs/research/3dgs-postprocess-map-design.md``.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

import lidarslam_benchmark_tools.gaussian_splatting.posed_images as pi


# --------------------------------------------------------------------------- #
# Pure helpers (numpy-only, ROS-free, unit-tested)
# --------------------------------------------------------------------------- #
def ros_stamp_to_seconds(sec: int, nanosec: int) -> float:
    """Convert a ROS ``builtin_interfaces/Time`` to float seconds."""
    return float(sec) + float(nanosec) * 1e-9


# Channel count per supported sensor_msgs/Image encoding.
_ENCODING_CHANNELS = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}


def decode_image(encoding: str, height: int, width: int, step: int,
                 data: bytes) -> np.ndarray:
    """Decode a raw ``sensor_msgs/Image`` payload to a canonical RGB uint8 array.

    Returns ``(H, W, 3)`` for colour encodings and ``(H, W)`` for ``mono8``.
    Avoids cv_bridge entirely (which is not numpy-2 compatible here). Handles
    row padding via ``step`` and converts BGR(A) to RGB. Raises ``ValueError``
    on an unsupported encoding or a payload shorter than ``step * height``.
    """
    enc = encoding.lower()
    if enc not in _ENCODING_CHANNELS:
        raise ValueError(f'unsupported image encoding {encoding!r}')
    channels = _ENCODING_CHANNELS[enc]
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if buf.size < step * height:
        raise ValueError(
            f'image payload {buf.size} < step*height {step * height}'
        )
    rows = buf[: step * height].reshape(height, step)
    img = rows[:, : width * channels].reshape(height, width, channels)
    if enc in ('bgr8', 'bgra8'):
        img = img[:, :, [2, 1, 0]]  # BGR(A) -> RGB, drop alpha
    elif enc == 'rgba8':
        img = img[:, :, :3]
    elif enc == 'mono8':
        return img[:, :, 0]
    return np.ascontiguousarray(img)


def compute_clock_offset(cam_header: float, cam_bagtime: float,
                         ref_header: float, ref_bagtime: float) -> float:
    """Offset to add to camera header stamps to reach the reference clock.

    Some bags carry sensors on independent uptime clocks (e.g. a Livox LiDAR
    and a camera whose ``header.stamp`` bases differ by tens of seconds). Each
    sensor's ``header - bag_receive_time`` is a near-constant skew; the offset
    that maps camera stamps onto the trajectory/reference clock is the
    difference of those skews.
    """
    skew_cam = cam_header - cam_bagtime
    skew_ref = ref_header - ref_bagtime
    return skew_ref - skew_cam


@dataclass(frozen=True)
class ClockCorrection:
    """Affine correction added to camera stamps at a bag receive time."""

    offset: float
    drift: float = 0.0
    origin: float = 0.0

    def apply(self, header_stamp: float, bag_time: float) -> float:
        return header_stamp + self.offset + self.drift * (bag_time - self.origin)


def _robust_clock_line(samples: np.ndarray, origin: float) -> tuple[float, float]:
    """Fit ``header-bagtime = intercept + slope*(bagtime-origin)`` robustly."""
    values = np.asarray(samples, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 2 or len(values) < 2:
        raise ValueError('clock samples must be Nx2 (header, bagtime), N >= 2')
    x = values[:, 1] - origin
    y = values[:, 0] - values[:, 1]
    keep = np.isfinite(x) & np.isfinite(y)
    if keep.sum() < 2:
        raise ValueError('clock samples need at least two finite rows')
    for _ in range(3):
        design = np.column_stack((np.ones(keep.sum()), x[keep]))
        intercept, slope = np.linalg.lstsq(design, y[keep], rcond=None)[0]
        residual = y - (intercept + slope * x)
        median = np.median(residual[keep])
        mad = np.median(np.abs(residual[keep] - median))
        if mad <= 1.0e-12:
            break
        next_keep = keep & (np.abs(residual - median) <= 4.5 * 1.4826 * mad)
        if next_keep.sum() < 2 or np.array_equal(next_keep, keep):
            break
        keep = next_keep
    return float(intercept), float(slope)


def estimate_clock_correction(cam_samples: np.ndarray,
                              ref_samples: np.ndarray) -> ClockCorrection:
    """Estimate camera-to-reference clock offset and drift from many samples."""
    cam = np.asarray(cam_samples, dtype=np.float64)
    ref = np.asarray(ref_samples, dtype=np.float64)
    if cam.ndim != 2 or ref.ndim != 2 or cam.shape[1:] != (2,) or ref.shape[1:] != (2,):
        raise ValueError('camera and reference samples must be Nx2')
    origin = float(np.median(np.concatenate((cam[:, 1], ref[:, 1]))))
    cam_offset, cam_drift = _robust_clock_line(cam, origin)
    ref_offset, ref_drift = _robust_clock_line(ref, origin)
    return ClockCorrection(ref_offset - cam_offset, ref_drift - cam_drift, origin)


def parse_extrinsic_dict(data: dict) -> np.ndarray:
    """Build a 4x4 ``body <- camera_optical`` matrix from a config dict.

    Accepts either a ``matrix`` (4x4 nested list) or a
    ``translation`` + ``rotation_xyzw`` pair. Raises ``ValueError`` otherwise.
    """
    if 'matrix' in data:
        m = np.asarray(data['matrix'], dtype=float)
        if m.shape != (4, 4):
            raise ValueError(f'extrinsic matrix must be 4x4, got {m.shape}')
        return m
    if 'translation' in data and 'rotation_xyzw' in data:
        return pi.make_transform(data['translation'], data['rotation_xyzw'])
    if 'results' in data and 'T_lidar_camera' in data['results']:
        # direct_visual_lidar_calibration stores lidar/body <- camera, exactly
        # the direction this extractor needs.
        values = data['results']['T_lidar_camera']
        if len(values) != 7:
            raise ValueError('T_lidar_camera must contain 7 values')
        return pi.make_transform(values[:3], values[3:])
    raise ValueError(
        'extrinsic must provide matrix, translation+rotation_xyzw, or '
        'results.T_lidar_camera'
    )


def load_intrinsics_yaml(path: str | Path) -> pi.CameraIntrinsics:
    """Parse a camera intrinsics YAML (NTU VIRAL / Kalibr-style PINHOLE).

    Reads ``image_width/height``, ``projection_parameters`` (fx,fy,cx,cy) and
    ``distortion_parameters`` (k1,k2,p1,p2). Tolerant of the OpenCV ``%YAML:1.0``
    header and ``!!opencv-matrix`` tags (we only need the scalar fields, pulled
    by regex), so ``yaml.safe_load`` is not required.
    """
    import re

    # Kalibr camchain format (HILTI, many visual-inertial datasets). Select the
    # first camera in file order; single-camera files naturally behave the same.
    import yaml
    try:
        parsed = yaml.safe_load(Path(path).read_text())
    except yaml.YAMLError:
        parsed = None
    if isinstance(parsed, dict):
        cameras = [value for key, value in parsed.items()
                   if str(key).startswith('cam') and isinstance(value, dict)]
        if cameras and 'intrinsics' in cameras[0] and 'resolution' in cameras[0]:
            camera = cameras[0]
            intr = camera['intrinsics']
            resolution = camera['resolution']
            if len(intr) != 4 or len(resolution) != 2:
                raise ValueError(f'{path}: invalid Kalibr intrinsics/resolution')
            return pi.CameraIntrinsics(
                width=int(resolution[0]), height=int(resolution[1]),
                fx=float(intr[0]), fy=float(intr[1]),
                cx=float(intr[2]), cy=float(intr[3]),
                distortion=tuple(float(x) for x in camera.get(
                    'distortion_coeffs', [])),
                distortion_model=str(camera.get('distortion_model', 'plumb_bob')),
            )

    text = Path(path).read_text()

    def grab(key: str, default: Optional[float] = None,
             after: Optional[str] = None) -> float:
        # Scope the search to the text after ``after`` (e.g. the
        # ``projection_parameters`` section header) so a stereo YAML's second
        # camera, or a rectification block reusing the same field names, cannot
        # win by appearing earlier in document order.
        scope = text
        if after is not None:
            anchor = re.search(rf'\b{after}\b', text)
            if anchor is not None:
                scope = text[anchor.end():]
        m = re.search(rf'\b{key}\s*:\s*([-+0-9.eE]+)', scope)
        if m is None:
            if default is None:
                raise ValueError(f'{path}: missing intrinsics field {key!r}')
            return default
        return float(m.group(1))

    return pi.CameraIntrinsics(
        width=int(grab('image_width')),
        height=int(grab('image_height')),
        fx=grab('fx', after='projection_parameters'),
        fy=grab('fy', after='projection_parameters'),
        cx=grab('cx', after='projection_parameters'),
        cy=grab('cy', after='projection_parameters'),
        distortion=(grab('k1', 0.0, after='distortion_parameters'),
                    grab('k2', 0.0, after='distortion_parameters'),
                    grab('p1', 0.0, after='distortion_parameters'),
                    grab('p2', 0.0, after='distortion_parameters'), 0.0),
    )


def load_extrinsic(path: Optional[str | Path]) -> np.ndarray:
    """Load ``body <- camera_optical`` from a YAML file, or identity if None."""
    if path is None:
        return np.eye(4)
    import yaml  # lazy: PyYAML present in ROS env, not needed for identity

    data = yaml.safe_load(Path(path).read_text())
    return parse_extrinsic_dict(data)


def load_kalibr_body_camera_extrinsic(camchain_path: str | Path,
                                      lidar_calibration_path: str | Path, *,
                                      camera_key: str = 'cam0',
                                      lidar_key: str = 'PandarXT-32') -> np.ndarray:
    """Load ``body/IMU <- camera`` for a body-frame SLAM trajectory.

    Kalibr stores ``T_cam_imu`` (camera <- IMU). The HILTI-style LiDAR file
    is validated here because the paired pipeline also uses its ``IMU <-
    LiDAR`` transform when accumulating scans. Camera poses, however, must be
    composed with the frame represented by the SLAM trajectory (IMU/body), not
    with the raw LiDAR frame. Therefore the required transform is simply the
    inverse Kalibr transform.
    """
    import yaml

    camchain = yaml.safe_load(Path(camchain_path).read_text())
    if camera_key not in camchain or 'T_cam_imu' not in camchain[camera_key]:
        raise ValueError(f'{camchain_path}: missing {camera_key}.T_cam_imu')
    camera_T_imu = np.asarray(
        camchain[camera_key]['T_cam_imu'], dtype=np.float64)
    if camera_T_imu.shape != (4, 4):
        raise ValueError(f'{camchain_path}: {camera_key}.T_cam_imu must be 4x4')

    load_parented_sensor_extrinsic(lidar_calibration_path, lidar_key)
    return np.linalg.inv(camera_T_imu)


def compose_kalibr_lidar_extrinsic(camchain_path: str | Path,
                                   lidar_calibration_path: str | Path, *,
                                   camera_key: str = 'cam0',
                                   lidar_key: str = 'PandarXT-32') -> np.ndarray:
    """Compatibility alias for ``load_kalibr_body_camera_extrinsic``."""
    return load_kalibr_body_camera_extrinsic(
        camchain_path, lidar_calibration_path,
        camera_key=camera_key, lidar_key=lidar_key)


def load_parented_sensor_extrinsic(path: str | Path,
                                   sensor_key: str) -> np.ndarray:
    """Load ``parent <- sensor`` from a HILTI-style sensor-tree YAML."""
    import yaml

    doc = yaml.safe_load(Path(path).read_text())
    sensors = doc.get('sensors', {}) if isinstance(doc, dict) else {}
    if sensor_key not in sensors or 'extrinsics' not in sensors[sensor_key]:
        raise ValueError(f'{path}: missing sensor {sensor_key}')
    extrinsic = sensors[sensor_key]['extrinsics']
    if 'translation' not in extrinsic or 'quaternion' not in extrinsic:
        raise ValueError(f'{path}: incomplete {sensor_key} extrinsic')
    return pi.make_transform(extrinsic['translation'], extrinsic['quaternion'])


def resolve_world_T_camera(
    stamp: float,
    samples: Sequence[pi.TrajectorySample],
    body_T_camera_optical: np.ndarray,
    *,
    max_extrapolation: float = 0.0,
    time_offset: float = 0.0,
) -> Optional[np.ndarray]:
    """Resolve ``world <- camera_optical`` for an image stamp.

    ``time_offset`` (seconds) is added to ``stamp`` before lookup to absorb a
    known camera/LiDAR clock skew. Returns ``None`` when the (offset) stamp
    falls outside the trajectory beyond ``max_extrapolation`` so the caller can
    drop the frame instead of inventing a pose.
    """
    try:
        world_T_body = pi.interpolate_pose(
            samples, stamp + time_offset, max_extrapolation=max_extrapolation
        )
    except ValueError:
        return None
    return pi.compose_world_T_camera(world_T_body, body_T_camera_optical)


# --------------------------------------------------------------------------- #
# ROS I/O (lazy imports; only exercised with a real bag)
# --------------------------------------------------------------------------- #
def _bag_is_file_compressed(bag_path: str | Path) -> bool:
    """Return True if the bag's metadata declares FILE-level compression (zstd)."""
    meta = Path(bag_path) / 'metadata.yaml'
    if not meta.is_file():
        return False
    text = meta.read_text(errors='replace')
    for line in text.splitlines():
        s = line.strip()
        if s.startswith('compression_mode:'):
            return s.split(':', 1)[1].strip().strip('"').upper() == 'FILE'
    return False


def _topic_type(bag_path: str | Path, topic: str) -> str:
    """Topic message type from the bag metadata (empty string if unknown)."""
    meta = Path(bag_path) / 'metadata.yaml'
    if not meta.exists():
        return ''
    import yaml

    info = yaml.safe_load(meta.read_text())['rosbag2_bagfile_information']
    for entry in info.get('topics_with_message_count', []):
        tm = entry['topic_metadata']
        if tm['name'] == topic:
            return tm['type']
    return ''


def decode_compressed_image(fmt: str, data: bytes) -> np.ndarray:
    """Decode a ``sensor_msgs/CompressedImage`` payload to RGB uint8.

    cv_bridge-produced jpegs encode the pre-compression channel order in
    ``format`` (e.g. ``"bgr8; jpeg compressed bgr8"``); decoding such a payload
    yields swapped channels, so honour a leading ``bgr`` tag.
    """
    import imageio.v3 as iio

    img = iio.imread(bytes(data))
    if img.ndim == 3 and img.shape[2] >= 3 and \
            fmt.lower().split(';')[0].strip().startswith('bgr'):
        img = img[:, :, [2, 1, 0]]
    return np.ascontiguousarray(img[:, :, :3] if img.ndim == 3 else img)


def _open_reader(bag_path: str | Path):
    import rosbag2_py

    storage = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='')
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )
    # FILE-compressed bags (Autoware Leo Drive, etc.) need the compression
    # reader; the plain SequentialReader would try to open the .zstd as sqlite.
    if _bag_is_file_compressed(bag_path):
        reader = rosbag2_py.SequentialCompressionReader()
    else:
        reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    return reader


def read_camera_intrinsics(bag_path: str | Path, topic: str) -> pi.CameraIntrinsics:
    """Read the first ``CameraInfo`` on ``topic`` into ``CameraIntrinsics``."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo

    reader = _open_reader(bag_path)
    while reader.has_next():
        tname, raw, _ = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(raw, CameraInfo)
        return pi.CameraIntrinsics.from_camera_info(
            msg.width, msg.height, list(msg.k), list(msg.d)
        )
    raise RuntimeError(f'no CameraInfo found on topic {topic!r}')


def _first_header_and_bagtime(bag_path: str | Path, topic: str,
                              msg_type) -> tuple[float, float]:
    """Return the (header_stamp_s, bag_receive_s) of the first ``topic`` msg."""
    from rclpy.serialization import deserialize_message

    reader = _open_reader(bag_path)
    while reader.has_next():
        tname, raw, bagt = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(raw, msg_type)
        header = ros_stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec)
        return header, bagt * 1e-9
    raise RuntimeError(f'no message found on topic {topic!r}')


def _clock_samples(bag_path: str | Path, topic: str, msg_type,
                   max_samples: int = 256,
                   min_interval: float = 0.5) -> np.ndarray:
    """Read clock pairs spread across a topic without decoding image payloads."""
    from rclpy.serialization import deserialize_message

    reader = _open_reader(bag_path)
    rows = []
    last_bagtime = -np.inf
    while reader.has_next() and len(rows) < max_samples:
        tname, raw, bagt = reader.read_next()
        bagtime = bagt * 1.0e-9
        if tname != topic or bagtime - last_bagtime < min_interval:
            continue
        msg = deserialize_message(raw, msg_type)
        header = ros_stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec)
        rows.append((header, bagtime))
        last_bagtime = bagtime
    if len(rows) < 2:
        raise RuntimeError(f'fewer than two clock samples on topic {topic!r}')
    return np.asarray(rows, dtype=np.float64)


def resolve_clock_correction(args: argparse.Namespace) -> ClockCorrection:
    """Resolve fixed or robust affine camera-to-reference clock correction."""
    adjustment = float(getattr(args, 'time_offset_adjustment', 0.0))
    if str(args.time_offset).lower() != 'auto':
        return ClockCorrection(float(args.time_offset) + adjustment)
    if not args.clock_reference_topic:
        raise ValueError('--time-offset auto requires --clock-reference-topic')
    from sensor_msgs.msg import CompressedImage, Image, PointCloud2

    cam_type = (CompressedImage
                if _topic_type(args.bag, args.camera_topic).endswith('CompressedImage')
                else Image)
    cam_samples = _clock_samples(args.bag, args.camera_topic, cam_type)
    ref_samples = _clock_samples(args.bag, args.clock_reference_topic, PointCloud2)
    correction = estimate_clock_correction(cam_samples, ref_samples)
    correction = ClockCorrection(
        correction.offset + adjustment, correction.drift, correction.origin)
    print(f'auto clock correction: offset={correction.offset:.6f}s '
          f'drift={correction.drift * 1.0e6:.3f}ppm '
          f'adjustment={adjustment:+.6f}s '
          f'(camera -> {args.clock_reference_topic})')
    return correction


def resolve_time_offset(args: argparse.Namespace) -> float:
    """Backward-compatible fixed offset accessor; auto returns its intercept."""
    return resolve_clock_correction(args).offset


def extract(args: argparse.Namespace) -> dict:
    """Run the full extraction and return a small summary dict."""
    import imageio.v3 as iio
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage, Image

    compressed = _topic_type(args.bag, args.camera_topic).endswith('CompressedImage')
    msg_cls = CompressedImage if compressed else Image
    samples = pi.read_tum_trajectory(args.traj)
    body_T_cam = load_extrinsic(args.extrinsic)
    if args.intrinsics_yaml:
        intrinsics = load_intrinsics_yaml(args.intrinsics_yaml)
    else:
        intrinsics = read_camera_intrinsics(args.bag, args.camera_info_topic)
    clock_correction = resolve_clock_correction(args)

    undistort_map = None
    out_intrinsics = intrinsics
    if args.undistort:
        import cv2
        k = np.array([[intrinsics.fx, 0, intrinsics.cx],
                      [0, intrinsics.fy, intrinsics.cy], [0, 0, 1.0]])
        size = (intrinsics.width, intrinsics.height)
        if intrinsics.distortion_model in ('equidistant', 'fisheye'):
            d = np.array((list(intrinsics.distortion) + [0] * 4)[:4], dtype=float)
            new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                k, d, size, np.eye(3), balance=0.0)
            undistort_map = cv2.fisheye.initUndistortRectifyMap(
                k, d, np.eye(3), new_k, size, cv2.CV_16SC2)
        else:
            d = np.array((list(intrinsics.distortion) + [0] * 5)[:5], dtype=float)
            new_k, _ = cv2.getOptimalNewCameraMatrix(k, d, size, 0, size)
            undistort_map = cv2.initUndistortRectifyMap(
                k, d, None, new_k, size, cv2.CV_16SC2)
        out_intrinsics = pi.CameraIntrinsics(
            intrinsics.width, intrinsics.height,
            float(new_k[0, 0]), float(new_k[1, 1]),
            float(new_k[0, 2]), float(new_k[1, 2]))

    out_dir = Path(args.out)
    images_dir = out_dir / 'images'
    images_dir.mkdir(parents=True, exist_ok=True)

    import rosbag2_py
    reader = _open_reader(args.bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.camera_topic]))
    frames: list[pi.PosedImage] = []
    seen = 0
    dropped = 0
    t0 = None
    while reader.has_next():
        tname, raw, bagt = reader.read_next()
        if tname != args.camera_topic:
            continue
        rel_t = 0.0 if t0 is None else (bagt * 1e-9 - t0)
        if t0 is None:
            t0 = bagt * 1e-9
        if args.end_time >= 0 and rel_t > args.end_time:
            break  # camera stamps are monotonic, no later frame qualifies
        if rel_t < args.start_time:
            continue
        if args.stride > 1 and seen % args.stride != 0:
            seen += 1
            continue
        msg = deserialize_message(raw, msg_cls)
        stamp = ros_stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec)
        corrected_stamp = clock_correction.apply(stamp, bagt * 1.0e-9)
        world_T_cam = resolve_world_T_camera(
            corrected_stamp, samples, body_T_cam,
            max_extrapolation=args.max_extrapolation,
        )
        if world_T_cam is None:
            dropped += 1
            seen += 1
            continue
        rel = f'images/{len(frames):05d}.png'
        if compressed:
            rgb = decode_compressed_image(msg.format, msg.data)
        else:
            rgb = decode_image(msg.encoding, msg.height, msg.width, msg.step, msg.data)
        if undistort_map is not None:
            import cv2
            rgb = cv2.remap(rgb, undistort_map[0], undistort_map[1], cv2.INTER_LINEAR)
        iio.imwrite(str(out_dir / rel), rgb)
        frames.append(pi.PosedImage(rel, world_T_cam, corrected_stamp))
        seen += 1

    if not frames:
        # Fail loudly here rather than writing an empty transforms.json that
        # only blows up later as an opaque torch.stack([]) in train_gsplat.
        raise RuntimeError(
            f'no image resolved a pose ({dropped} dropped): the camera stamps '
            'do not overlap the trajectory. Check --time-offset / '
            '--clock-reference-topic, --extrinsic, and that the bag and TUM '
            'trajectory cover the same interval.')
    pi.write_transforms(out_dir / 'transforms.json', out_intrinsics, frames)
    return {'kept': len(frames), 'dropped': dropped, 'out': str(out_dir)}


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--bag', required=True, help='rosbag2 directory')
    p.add_argument('--traj', required=True, help='SLAM trajectory (TUM, world<-body)')
    p.add_argument('--camera-topic', default='/image')
    p.add_argument('--camera-info-topic', default='/camera_info')
    p.add_argument('--intrinsics-yaml', default=None,
                   help='camera intrinsics YAML (NTU/Kalibr); overrides bag camera_info')
    p.add_argument('--undistort', action='store_true',
                   help='undistort images to a pinhole model (gsplat is pinhole)')
    p.add_argument('--start-time', type=float, default=0.0,
                   help='keep images at/after this many seconds from bag start')
    p.add_argument('--end-time', type=float, default=-1.0,
                   help='keep images up to this many seconds from bag start (-1 = all)')
    p.add_argument('--extrinsic', default=None,
                   help='YAML with body<-camera_optical (matrix or translation+rotation_xyzw); '
                        'identity if omitted')
    p.add_argument('--out', required=True, help='output directory')
    p.add_argument('--max-extrapolation', type=float, default=0.05,
                   help='seconds an image stamp may fall outside the trajectory')
    p.add_argument('--time-offset', default='0.0',
                   help='seconds added to image stamps, or "auto" to align the '
                        'camera clock to --clock-reference-topic via bag receive time')
    p.add_argument('--time-offset-adjustment', type=float, default=0.0,
                   help='seconds added after fixed or auto clock correction; '
                        'intended for measured synchronization ablations')
    p.add_argument('--clock-reference-topic', default=None,
                   help='PointCloud2 topic whose clock the trajectory uses '
                        '(required for --time-offset auto)')
    p.add_argument('--stride', type=int, default=1, help='keep every Nth image')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    if args.extrinsic is None:
        print('warning: no --extrinsic given; using identity body<-camera_optical')
    summary = extract(args)
    print(f"wrote {summary['kept']} frames ({summary['dropped']} dropped) to {summary['out']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
