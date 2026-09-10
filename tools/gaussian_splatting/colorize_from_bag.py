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

"""Colour one real LiDAR scan from a time-matched camera image in a rosbag2.

A LiDAR scan and a camera on the same rig share a *static* extrinsic, so a
single time-matched (cloud, image) pair can be coloured by projection with no
SLAM and no map frame: pick a cloud, pick the nearest image in time, resolve
``camera_optical <- lidar`` from ``/tf`` + ``/tf_static`` or an explicit
7-value calibration (they are rigidly mounted, so the transform is
time-independent), undistort the image, and hand
the pair to the offline colorizer
(``pointcloud_io.colorize_by_projection_robust`` — occlusion-aware, bilinear,
depth-weighted). This is the productionised form of the real-data experiment in
``docs`` and validates the whole colour path on real sensors.

Design mirrors ``extract_posed_images.py``: the pure geometry helpers
(``nearest_index``, ``transform_msg_to_matrix``) are numpy-only and unit-tested
in the ament pytest harness, while the rosbag2 / tf2 / OpenCV imports are lazy
inside ``main`` so importing this module never needs a ROS environment.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

import numpy as np

import lidarslam_benchmark_tools.gaussian_splatting.pointcloud_io as pcio
import lidarslam_benchmark_tools.gaussian_splatting.posed_images as pi


# --------------------------------------------------------------------------- #
# Pure helpers (numpy only — no ROS)
# --------------------------------------------------------------------------- #
def nearest_index(sorted_stamps: Sequence[int], target: int) -> int:
    """Index of the entry in ascending ``sorted_stamps`` closest to ``target``."""
    arr = np.asarray(sorted_stamps)
    if arr.size == 0:
        raise ValueError('no stamps to match against')
    return int(np.argmin(np.abs(arr.astype(np.int64) - int(target))))


def select_synced_time(pc_stamps: Sequence[int], image_stamps: Sequence[int],
                       time_frac: float, search_radius: int = 2
                       ) -> tuple[int, int]:
    """Select a mutually-nearest cloud/image pair near ``time_frac``.

    Around the requested position, examine the nearest image plus
    ``search_radius`` neighbours on each side. Each image is paired with its
    nearest cloud; the smallest time offset wins (proximity to the requested
    time breaks ties). This reduces motion-induced colour offset without
    moving to an unrelated part of the bag.
    """
    if not pc_stamps:
        raise ValueError('no point-cloud stamps')
    if not image_stamps:
        raise ValueError('no image stamps')
    frac = min(max(float(time_frac), 0.0), 1.0)
    target = pc_stamps[min(int(len(pc_stamps) * frac), len(pc_stamps) - 1)]
    center = nearest_index(image_stamps, target)
    radius = max(int(search_radius), 0)
    candidates = []
    for index in range(max(0, center - radius),
                       min(len(image_stamps), center + radius + 1)):
        image_time = int(image_stamps[index])
        pc_time = int(pc_stamps[nearest_index(pc_stamps, image_time)])
        candidates.append((abs(pc_time - image_time),
                           abs(pc_time - target), pc_time, image_time))
    _, _, pc_time, image_time = min(candidates)
    return pc_time, image_time


def transform_msg_to_matrix(translation, rotation) -> np.ndarray:
    """Build a 4x4 ``target <- source`` matrix from a geometry_msgs Transform.

    ``translation`` has ``.x/.y/.z`` and ``rotation`` has ``.x/.y/.z/.w`` — the
    fields of a ``geometry_msgs/Transform``. Reuses ``posed_images.make_transform``
    so the maths matches the rest of the toolchain.
    """
    t = [translation.x, translation.y, translation.z]
    q = [rotation.x, rotation.y, rotation.z, rotation.w]
    return pi.make_transform(t, q)


def extrinsic_matrix(values=None, path=None) -> np.ndarray | None:
    """Load a ``camera_optical <- lidar`` transform from CLI values or JSON.

    The compact form is ``tx ty tz qx qy qz qw``. A JSON file may contain that
    seven-value list, an object with ``translation`` and ``rotation_xyzw``
    arrays, or an official direct_visual_lidar_calibration ``calib.json``.
    The latter stores the opposite transform and is inverted automatically.
    Exactly one source may be supplied. ``None`` means use bag TF.
    """
    if values is not None and path is not None:
        raise ValueError('use either extrinsic values or an extrinsic file, not both')
    invert = False
    if path is not None:
        try:
            data = json.loads(Path(path).read_text())
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f'failed to read extrinsic JSON {path!s}: {exc}') from exc
        if isinstance(data, dict) and 'results' in data and \
                'T_lidar_camera' in data['results']:
            values = data['results']['T_lidar_camera']
            invert = True  # official result is lidar <- camera
        elif isinstance(data, dict):
            if 'translation' not in data or 'rotation_xyzw' not in data:
                raise ValueError(
                    'extrinsic JSON needs translation/rotation_xyzw or '
                    'results/T_lidar_camera')
            values = [*data['translation'], *data['rotation_xyzw']]
        else:
            values = data
    if values is None:
        return None
    try:
        vals = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError('extrinsic values must be numeric') from exc
    if vals.shape != (7,):
        raise ValueError(
            'extrinsic must have 7 values: tx ty tz qx qy qz qw')
    if not np.all(np.isfinite(vals)):
        raise ValueError('extrinsic values must be finite')
    quat_norm = float(np.linalg.norm(vals[3:]))
    if quat_norm <= 1e-12:
        raise ValueError('extrinsic quaternion must be non-zero')
    matrix = pi.make_transform(vals[:3], vals[3:] / quat_norm)
    return np.linalg.inv(matrix) if invert else matrix


def projection_diagnostics(points, world_to_cam, K, width, height, *,
                           zbuf_bin=4, depth_tol=0.15) -> dict:
    """Project points and report in-frame/visible samples for an overlay.

    Returns arrays ``indices``, ``u``, ``v``, ``depth`` and ``visible``. The
    coarse z-buffer matches the production colorizer, so hidden points can be
    distinguished from points that genuinely contributed colour.
    """
    xyz = np.asarray(points, dtype=np.float64)
    transform = np.asarray(world_to_cam, dtype=np.float64)
    intr = np.asarray(K, dtype=np.float64).reshape(3, 3)
    if zbuf_bin < 1:
        raise ValueError('zbuf_bin must be >= 1')
    cam = xyz @ transform[:3, :3].T + transform[:3, 3]
    depth = cam[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = np.nan_to_num(intr[0, 0] * cam[:, 0] / depth + intr[0, 2], nan=-1.0)
        v = np.nan_to_num(intr[1, 1] * cam[:, 1] / depth + intr[1, 2], nan=-1.0)
    ui = np.round(u).astype(np.int64)
    vi = np.round(v).astype(np.int64)
    in_frame = ((depth > 1e-6) & (ui >= 0) & (ui < width) &
                (vi >= 0) & (vi < height))
    indices = np.flatnonzero(in_frame)
    if indices.size == 0:
        return {
            'indices': indices, 'u': u[indices], 'v': v[indices],
            'depth': depth[indices], 'visible': np.zeros(0, dtype=bool),
        }
    cols = (int(width) + zbuf_bin - 1) // zbuf_bin
    rows = (int(height) + zbuf_bin - 1) // zbuf_bin
    cells = (vi[indices] // zbuf_bin) * cols + ui[indices] // zbuf_bin
    zbuf = np.full(rows * cols, np.inf, dtype=np.float64)
    np.minimum.at(zbuf, cells, depth[indices])
    visible = depth[indices] <= zbuf[cells] + depth_tol + 0.02 * depth[indices]
    return {
        'indices': indices, 'u': u[indices], 'v': v[indices],
        'depth': depth[indices], 'visible': visible,
    }


def _write_diagnostic_overlay(path, rgb_image, diagnostics, *, pair_dt_ms,
                              total_points) -> Path:
    """Write a depth-coloured projection overlay with a compact status panel."""
    import cv2

    canvas = np.ascontiguousarray(np.asarray(rgb_image, dtype=np.uint8).copy())
    u = np.round(diagnostics['u']).astype(np.int64)
    v = np.round(diagnostics['v']).astype(np.int64)
    depth = diagnostics['depth']
    visible = diagnostics['visible']
    # Occluded samples remain visible as small magenta dots for diagnosis.
    for x, y in zip(u[~visible], v[~visible]):
        cv2.circle(canvas, (int(x), int(y)), 1, (255, 0, 255), -1)
    if visible.any():
        visible_depth = depth[visible]
        near, far = np.percentile(visible_depth, [5, 95])
        scale = np.clip((visible_depth - near) / max(far - near, 1e-6), 0.0, 1.0)
        # Near=red, middle=green, far=blue (RGB).
        colours = np.column_stack([
            255.0 * (1.0 - scale),
            255.0 * (1.0 - np.abs(2.0 * scale - 1.0)),
            255.0 * scale,
        ]).astype(np.uint8)
        for x, y, colour in zip(u[visible], v[visible], colours):
            cv2.circle(canvas, (int(x), int(y)), 2,
                       tuple(int(c) for c in colour), -1)
    in_frame = len(diagnostics['indices'])
    n_visible = int(visible.sum())
    lines = [
        f'points: {total_points}  in-frame: {in_frame}  visible: {n_visible}',
        f'pair dt: {pair_dt_ms:.1f} ms  depth: near red -> far blue',
        'magenta: rejected by occlusion z-buffer',
    ]
    cv2.rectangle(canvas, (8, 8), (850, 92), (0, 0, 0), -1)
    for row, line in enumerate(lines):
        cv2.putText(canvas, line, (20, 34 + row * 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2,
                    cv2.LINE_AA)
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), canvas[:, :, ::-1]):
        raise RuntimeError(f'failed to write diagnostic overlay {output}')
    return output


def merge_colorings(per_camera, default_rgb=(80, 80, 80)
                    ) -> tuple[np.ndarray, np.ndarray]:
    """Fuse several cameras' colourings of one cloud into a single colour.

    ``per_camera`` is a list of ``(rgb uint8 (N,3), seen bool (N,), counts
    (N,))`` — the ``return_counts=True`` output of
    ``colorize_by_projection_robust`` for each camera. Each point's final colour
    is the per-camera mean weighted by that camera's sample count, so a camera
    that saw a point in more views (or at all) contributes proportionally and a
    point seen by two overlapping cameras blends both. Points no camera saw keep
    ``default_rgb``. Returns ``(rgb uint8 (N,3), seen bool (N,))``.
    """
    if not per_camera:
        raise ValueError('need at least one camera colouring to merge')
    n = per_camera[0][0].shape[0]
    acc = np.zeros((n, 3), dtype=np.float64)
    sum_w = np.zeros(n, dtype=np.float64)
    for rgb, _seen, counts in per_camera:
        w = np.asarray(counts, dtype=np.float64)
        acc += w[:, None] * np.asarray(rgb, dtype=np.float64)
        sum_w += w
    seen = sum_w > 0
    out = np.tile(np.asarray(default_rgb, dtype=np.uint8), (n, 1))
    out[seen] = np.round(acc[seen] / sum_w[seen, None]).astype(np.uint8)
    return out, seen


def colour_statistics(rgb: np.ndarray, seen: np.ndarray) -> dict:
    """Summarise whether visible point colours contain real chroma."""
    colours = np.asarray(rgb, dtype=np.uint8)
    mask = np.asarray(seen, dtype=bool)
    if colours.shape != (len(mask), 3):
        raise ValueError('rgb and seen shapes must match')
    selected = colours[mask]
    if not len(selected):
        return {'mean_channel_range': 0.0, 'chromatic_fraction_10': 0.0,
                'unique_colours': 0}
    channel_range = np.ptp(selected.astype(np.int16), axis=1)
    return {
        'mean_channel_range': float(np.mean(channel_range)),
        'chromatic_fraction_10': float(np.mean(channel_range >= 10)),
        'unique_colours': int(len(np.unique(selected, axis=0))),
    }


# --------------------------------------------------------------------------- #
# rosbag2 + tf2 (lazy ROS imports live inside these)
# --------------------------------------------------------------------------- #
def _collect(bag_path, pc_topic, cameras, need_tf=True):
    """One pass over the bag: tf buffer, each camera's CameraInfo, and stamps.

    ``cameras`` is a list of ``(image_topic, info_topic, optical_frame)``.
    Returns ``(buf, infos, pc_stamps, img_stamps, types)`` where ``infos`` and
    ``img_stamps`` are keyed by info_topic / image_topic respectively. With
    ``need_tf=False``, TF topics are optional and ``buf`` is ``None``.
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import tf2_ros
    import rclpy.duration
    from lidarslam_benchmark_tools.gaussian_splatting.extract_posed_images import _open_reader

    reader = _open_reader(bag_path)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if need_tf and ('/tf' not in types or '/tf_static' not in types):
        raise RuntimeError(
            'bag has no /tf and /tf_static; pass --extrinsic or '
            '--extrinsic-file with camera_optical <- lidar calibration')
    tf_cls = get_message(types['/tf']) if '/tf' in types else None
    tf_static_cls = get_message(types['/tf_static']) if '/tf_static' in types else None
    buf = None
    if need_tf:
        try:
            buf = tf2_ros.BufferCore(rclpy.duration.Duration(seconds=3600))
        except TypeError:  # older signature
            buf = tf2_ros.BufferCore()

    info_topics = {c[1] for c in cameras}
    image_topics = {c[0] for c in cameras}
    infos: dict = {}
    pc_stamps: list[int] = []
    img_stamps: dict = {t: [] for t in image_topics}
    while reader.has_next():
        topic, raw, bagt = reader.read_next()
        if need_tf and topic == '/tf_static':
            for tr in deserialize_message(raw, tf_static_cls).transforms:
                buf.set_transform_static(tr, 'bag')
        elif need_tf and topic == '/tf':
            for tr in deserialize_message(raw, tf_cls).transforms:
                buf.set_transform(tr, 'bag')
        elif topic in info_topics and topic not in infos:
            infos[topic] = deserialize_message(raw, get_message(types[topic]))
        elif topic == pc_topic:
            pc_stamps.append(bagt)
        elif topic in image_topics:
            img_stamps[topic].append(bagt)
    for c in cameras:
        if c[1] not in infos:
            raise RuntimeError(f'no CameraInfo on {c[1]!r}')
        if not img_stamps[c[0]]:
            raise RuntimeError(f'no images on {c[0]!r}')
    if not pc_stamps:
        raise RuntimeError('bag has no point-cloud messages')
    return (buf, infos, sorted(pc_stamps),
            {t: sorted(s) for t, s in img_stamps.items()}, types)


def _grab_messages(bag_path, wanted, types):
    """Second pass: deserialize each ``(topic, bag_time)`` in ``wanted`` -> msg."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from lidarslam_benchmark_tools.gaussian_splatting.extract_posed_images import _open_reader

    out = {key: None for key in wanted}
    remaining = len(wanted)
    reader = _open_reader(bag_path)
    while reader.has_next() and remaining > 0:
        topic, raw, bagt = reader.read_next()
        key = (topic, bagt)
        if key in out and out[key] is None:
            out[key] = deserialize_message(raw, get_message(types[topic]))
            remaining -= 1
    missing = [k for k, v in out.items() if v is None]
    if missing:
        raise RuntimeError(f'failed to re-read messages: {missing}')
    return out


def _image_to_rgb(img_msg, K, D, undistort):
    """Decode an sensor_msgs/Image (rgb8/bgr8) to an RGB array, optionally undistort."""
    enc = img_msg.encoding
    arr = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
        img_msg.height, img_msg.width, -1)
    if enc == 'bgr8':
        rgb = arr[:, :, ::-1]
    elif enc == 'rgb8':
        rgb = arr[:, :, :3]
    elif enc in ('bgra8',):
        rgb = arr[:, :, [2, 1, 0]]
    elif enc in ('rgba8',):
        rgb = arr[:, :, :3]
    elif enc == 'mono8':
        rgb = np.repeat(arr[:, :, :1], 3, axis=2)
    else:
        raise RuntimeError(f'unsupported image encoding {enc!r}')
    rgb = np.ascontiguousarray(rgb)
    if undistort and np.any(np.asarray(D) != 0.0):
        import cv2
        rgb = cv2.undistort(rgb, np.asarray(K, dtype=np.float64).reshape(3, 3),
                            np.asarray(D, dtype=np.float64))
    return rgb


def _read_xyz(pc_msg) -> np.ndarray:
    from sensor_msgs_py import point_cloud2 as pc2
    pts = pc2.read_points(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True)
    return np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(np.float64)


def _camera_list(args) -> list:
    """Assemble the ``(image_topic, info_topic, optical_frame)`` camera list."""
    cams = [(args.image_topic, args.camera_info_topic, args.camera_optical_frame)]
    for extra in (args.extra_camera or []):
        cams.append((extra[0], extra[1], extra[2]))
    return cams


def colorize_bag_frame(args) -> dict:
    """Run the whole extract -> colorize -> write flow; return a summary dict."""
    from rclpy.time import Time

    cameras = _camera_list(args)
    manual_extrinsic = extrinsic_matrix(
        getattr(args, 'extrinsic', None), getattr(args, 'extrinsic_file', None))
    if manual_extrinsic is not None and len(cameras) != 1:
        raise ValueError(
            'manual extrinsic currently supports one camera; remove --extra-camera')
    buf, infos, pc_stamps, img_stamps, types = _collect(
        args.bag, args.pc_topic, cameras, need_tf=manual_extrinsic is None)

    primary_image_topic = cameras[0][0]
    pc_time, primary_image_time = select_synced_time(
        pc_stamps, img_stamps[primary_image_topic], args.time_frac,
        getattr(args, 'sync_search_radius', 2))

    # Choose each camera's nearest-in-time image and gather every message.
    per_cam_time = {}
    wanted = {(args.pc_topic, pc_time)}
    max_pair_dt_ms = getattr(args, 'max_pair_dt_ms', 100.0)
    for img_topic, info_topic, _frame in cameras:
        t = primary_image_time if img_topic == primary_image_topic else \
            img_stamps[img_topic][nearest_index(img_stamps[img_topic], pc_time)]
        pair_dt_ms = abs(t - pc_time) / 1e6
        if max_pair_dt_ms > 0 and pair_dt_ms > max_pair_dt_ms:
            raise RuntimeError(
                f'nearest image on {img_topic!r} is {pair_dt_ms:.1f} ms from '
                f'the cloud (limit {max_pair_dt_ms:.1f} ms); choose another '
                '--time-frac or relax --max-pair-dt-ms')
        per_cam_time[img_topic] = t
        wanted.add((img_topic, t))
    msgs = _grab_messages(args.bag, wanted, types)

    xyz = _read_xyz(msgs[(args.pc_topic, pc_time)])

    per_camera = []
    cam_stats = []
    overlay_context = None
    for img_topic, info_topic, frame in cameras:
        info = infos[info_topic]
        K = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        D = np.asarray(info.d, dtype=np.float64)
        W, H = int(info.width), int(info.height)
        if manual_extrinsic is None:
            tf = buf.lookup_transform_core(frame, args.base_frame, Time().to_msg())
            world_to_cam = transform_msg_to_matrix(
                tf.transform.translation, tf.transform.rotation)
        else:
            world_to_cam = manual_extrinsic
        rgb_img = _image_to_rgb(
            msgs[(img_topic, per_cam_time[img_topic])], K, D, not args.no_undistort)
        colors, seen, counts = pcio.colorize_by_projection_robust(
            xyz, world_to_cam[None], K, [rgb_img], W, H,
            default_rgb=tuple(args.default_rgb),
            zbuf_bin=args.zbuf_bin, depth_tol=args.depth_tol,
            normalize_exposure=args.normalize_exposure,
            interp=args.interp, edge_threshold=args.edge_threshold,
            return_counts=True)
        per_camera.append((colors, seen, counts))
        stats = {
            'image_topic': img_topic, 'colored': int(seen.sum()),
            'pair_dt_ms': abs(per_cam_time[img_topic] - pc_time) / 1e6}
        cam_stats.append(stats)
        if img_topic == primary_image_topic:
            overlay_context = (rgb_img, world_to_cam, K, W, H, stats)

    colors, seen = merge_colorings(per_camera, tuple(args.default_rgb))
    n_seen = int(seen.sum())

    out_prefix = Path(args.out_prefix)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    full = pcio.write_ply(out_prefix.with_name(out_prefix.name + '_full.ply'),
                          xyz, colors)
    seen_ply = pcio.write_ply(out_prefix.with_name(out_prefix.name + '_seen.ply'),
                              xyz[seen], colors[seen])
    overlay_path = None
    requested_overlay = getattr(args, 'diagnostic_overlay', None)
    if requested_overlay is not None and overlay_context is not None:
        rgb_img, world_to_cam, K, W, H, stats = overlay_context
        diagnostics = projection_diagnostics(xyz, world_to_cam, K, W, H)
        overlay_path = _write_diagnostic_overlay(
            requested_overlay, rgb_img, diagnostics,
            pair_dt_ms=stats['pair_dt_ms'], total_points=len(xyz))
    return {
        'pc_frames': len(pc_stamps), 'cameras': cam_stats,
        'points': len(xyz), 'colored': n_seen,
        'colored_frac': n_seen / max(1, len(xyz)),
        'colour_statistics': colour_statistics(colors, seen),
        'projection_config': {
            'zbuf_bin': args.zbuf_bin,
            'depth_tol_m': args.depth_tol,
            'interp': args.interp,
            'edge_threshold': args.edge_threshold,
        },
        'full_ply': str(full), 'seen_ply': str(seen_ply),
        'diagnostic_overlay': str(overlay_path) if overlay_path else None,
    }


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('bag', help='rosbag2 directory')
    p.add_argument('out_prefix', help='output path prefix (_full.ply / _seen.ply)')
    p.add_argument('--pc-topic', default='/sensing/lidar/concatenated/pointcloud')
    p.add_argument('--image-topic', default='/lucid_vision/camera_0/raw_image')
    p.add_argument('--camera-info-topic',
                   default='/lucid_vision/camera_0/camera_info')
    p.add_argument('--base-frame', default='base_link',
                   help='frame the point cloud is expressed in')
    p.add_argument('--camera-optical-frame', default='camera_top/camera_optical_link',
                   help='camera OPTICAL frame (REP-103 z-forward) to project into')
    extrinsic = p.add_mutually_exclusive_group()
    extrinsic.add_argument(
        '--extrinsic', type=float, nargs=7,
        metavar=('TX', 'TY', 'TZ', 'QX', 'QY', 'QZ', 'QW'),
        help='camera_optical <- lidar transform; allows bags without TF')
    extrinsic.add_argument(
        '--extrinsic-file', type=Path,
        help='JSON transform or direct_visual_lidar_calibration calib.json')
    p.add_argument('--extra-camera', nargs=3, action='append',
                   metavar=('IMAGE_TOPIC', 'INFO_TOPIC', 'OPTICAL_FRAME'),
                   help='add another camera to fuse (repeatable); more coverage')
    p.add_argument('--time-frac', type=float, default=0.6,
                   help='pick the cloud at this fraction through the bag [0,1]')
    p.add_argument('--max-pair-dt-ms', type=float, default=100.0,
                   help='reject image/cloud pairs farther apart (<=0 disables)')
    p.add_argument('--sync-search-radius', type=int, default=2,
                   help='neighbouring images per side searched for best sync')
    p.add_argument('--no-undistort', action='store_true',
                   help='skip plumb_bob undistortion (needs OpenCV otherwise)')
    p.add_argument('--normalize-exposure', action='store_true',
                   help='rescale image luminance (harmless for a single view)')
    p.add_argument('--zbuf-bin', type=int, default=1,
                   help='occlusion z-buffer pixel bin size (default: 1)')
    p.add_argument('--depth-tol', type=float, default=0.15,
                   help='base visible-depth tolerance in metres (default: 0.15)')
    p.add_argument('--interp', choices=('nearest', 'bilinear', 'edge-aware'),
                   default='edge-aware', help='image sampling mode')
    p.add_argument('--edge-threshold', type=float, default=48.0,
                   help='RGB range where edge-aware sampling snaps to nearest')
    p.add_argument('--default-rgb', type=int, nargs=3, default=(80, 80, 80),
                   help='colour for points no camera saw')
    p.add_argument('--diagnostic-overlay', type=Path,
                   help='write depth/occlusion projection overlay PNG')
    p.add_argument('--report', type=Path, help='write machine-readable summary JSON')
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    s = colorize_bag_frame(args)
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(s, indent=2) + '\n')
    print(f"pc_frames={s['pc_frames']}  cameras={len(s['cameras'])}")
    for c in s['cameras']:
        print(f"  {c['image_topic']}: {c['colored']} pts  (dt={c['pair_dt_ms']:.1f}ms)")
    print(f"merged coloured {s['colored']}/{s['points']} "
          f"({100.0 * s['colored_frac']:.1f}%)")
    stats = s['colour_statistics']
    print(f"real RGB: channel range={stats['mean_channel_range']:.1f}, "
          f"chromatic>=10={100.0 * stats['chromatic_fraction_10']:.1f}%")
    print(f"wrote {s['full_ply']}\n      {s['seen_ply']}")
    if s['diagnostic_overlay']:
        print(f"      {s['diagnostic_overlay']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
