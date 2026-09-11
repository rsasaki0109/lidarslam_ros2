#!/usr/bin/env python3
"""Render a third-person flythrough of the SLAM point-cloud map along the
estimated trajectory.

The camera rides the full walked loop at constant speed (the pose stream is
resampled uniformly by arc length, so capture stand-stills are skipped
naturally) and follows the current trajectory point from behind and above,
keeping the subject — the magenta trajectory line and the map around it —
centred in frame. Points above the local walking height are cut away so the
indoor scene reads as a cutaway bird's-eye view instead of a ceiling wall.

Unlike the photoreal 3DGS pane (which only holds up next to its training
views; from a third-person viewpoint the trained gaussians render as
confetti noise), the point-cloud map renders from any viewpoint, so this is
the "actually travel through the map" companion to
``render_slam_3dgs_sidebyside.py`` and reuses its height-colouring,
trajectory-line, minimap and fade helpers. With ``--color-mode rgb`` and an
init cloud colorized by ``build_lidar_init.py --color-transforms`` the map
is drawn in the real camera-projected colours instead of the height ramp. The pure path helpers
(``moving_average_edge``, ``resample_equal_arclength``, ``smooth_tangents``,
``third_person_path``, ``build_viewmats``, ``ceiling_cut_mask``) are
numpy-only and unit tested on CPU; rendering needs CUDA + torch + gsplat.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
from typing import Optional, Sequence

import numpy as np

# The renderer and transforms loader stay on the 3DGS side; make their
# directory importable regardless of which directory hosts the caller.
_GS_DIR = Path(__file__).resolve().parents[1] / 'gaussian_splatting'
if str(_GS_DIR) not in sys.path:
    sys.path.append(str(_GS_DIR))

import pointcloud_io as pcio  # noqa: E402
from render_path import render_frames, scale_intrinsics, write_videos  # noqa: E402
from render_slam_3dgs_sidebyside import (  # noqa: E402
    annotate_frames,
    fade_weights,
    height_colormap,
    mask_far_from,
    merge_gaussians,
    minimap_points,
    nearest_polyline_index,
    points_to_gaussians,
    resample_polyline,
)
from train_gsplat import load_transforms  # noqa: E402

WORLD_UP = np.array([0.0, 0.0, 1.0])


# --------------------------------------------------------------------------- #
# Pure helpers (no torch/CUDA)
# --------------------------------------------------------------------------- #
def moving_average_edge(x: np.ndarray, window: int) -> np.ndarray:
    """Moving average over the first axis with edge padding (odd window)."""
    x = np.asarray(x, dtype=np.float64)
    if window <= 1 or len(x) <= 1:
        return x.copy()
    window = int(window)
    if window % 2 == 0:
        window += 1
    pad = window // 2
    xp = np.pad(x, [(pad, pad), (0, 0)], mode='edge')
    kernel = np.ones(window) / float(window)
    out = np.empty_like(x)
    for c in range(x.shape[1]):
        out[:, c] = np.convolve(xp[:, c], kernel, mode='valid')
    return out


def resample_equal_arclength(points: np.ndarray, count: int) -> tuple[np.ndarray, float]:
    """Resample an ordered position polyline to ``count`` equal-arc-length samples.

    Stand-still stretches contribute zero arc length, so they collapse to a
    single sample instead of freezing the ride: zero-length segments are
    dropped before interpolation, which also keeps the cumulative arc length
    strictly increasing. Returns the resampled path and its total length in
    metres.
    """
    points = np.asarray(points, dtype=np.float64)
    if len(points) < 2:
        raise ValueError('need at least two positions to build a flythrough path')
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.concatenate([[True], seg > 1.0e-9])
    points = points[keep]
    if len(points) < 2:
        raise ValueError('the trajectory never moves; cannot build a flythrough path')
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])
    targets = np.linspace(0.0, s[-1], int(count))
    out = np.stack([np.interp(targets, s, points[:, c]) for c in range(3)], axis=1)
    return out, float(s[-1])


def resample_cinematic_arclength(points: np.ndarray, count: int,
                                 corner_slowdown: float) -> tuple[np.ndarray, float]:
    """Resample while spending more frames at changes of heading."""
    if corner_slowdown < 0.0:
        raise ValueError('corner_slowdown must be >= 0')
    if corner_slowdown == 0.0:
        return resample_equal_arclength(points, count)
    dense, total = resample_equal_arclength(points, max(int(count) * 8, 256))
    delta = np.diff(dense, axis=0)
    length = np.linalg.norm(delta, axis=1)
    direction = delta / np.maximum(length[:, None], 1.0e-9)
    turn = np.zeros(len(length))
    if len(direction) > 1:
        turn[1:] = np.arccos(np.clip(
            np.sum(direction[1:] * direction[:-1], axis=1), -1.0, 1.0))
    turn = moving_average_edge(turn[:, None], 9)[:, 0]
    timeline = np.concatenate([[0.0], np.cumsum(
        length * (1.0 + float(corner_slowdown) * turn / np.pi))])
    targets = np.linspace(0.0, timeline[-1], int(count))
    out = np.stack([
        np.interp(targets, timeline, dense[:, axis]) for axis in range(3)
    ], axis=1)
    return out, total


def look_ahead_targets(path: np.ndarray, distance: float) -> np.ndarray:
    """Return points ``distance`` metres ahead along an ordered path."""
    path = np.asarray(path, dtype=np.float64)
    if distance <= 0.0:
        return path.copy()
    arc = np.concatenate([[0.0], np.cumsum(
        np.linalg.norm(np.diff(path, axis=0), axis=1))])
    target_arc = np.minimum(arc + float(distance), arc[-1])
    return np.stack([
        np.interp(target_arc, arc, path[:, axis]) for axis in range(3)
    ], axis=1)


def smooth_tangents(path: np.ndarray, window: int = 15) -> np.ndarray:
    """Unit travel directions along ``path`` (central differences, smoothed).

    Zero-length tangents (repeated samples) inherit the previous direction so
    the result is always finite and unit length.
    """
    path = np.asarray(path, dtype=np.float64)
    t = np.empty_like(path)
    t[0] = path[1] - path[0]
    t[-1] = path[-1] - path[-2]
    t[1:-1] = path[2:] - path[:-2]
    t = moving_average_edge(t, window)
    out = np.empty_like(t)
    prev = np.array([1.0, 0.0, 0.0])
    for i, v in enumerate(t):
        n = float(np.linalg.norm(v))
        prev = v / n if n > 1.0e-10 else prev
        out[i] = prev
    return out


def third_person_path(ride: np.ndarray, tangents: np.ndarray, *, follow_back: float,
                      lift: float, look_up: float = 0.8,
                      eye_smooth_window: int = 31,
                      targets: Optional[np.ndarray] = None) -> tuple[np.ndarray, np.ndarray]:
    """Follow-camera eyes and forward directions for a ride along ``ride``.

    The camera sits ``follow_back`` metres behind (horizontally, along the
    travel direction) and ``lift`` metres above the current ride point, and
    looks at the ride point raised by ``look_up`` — so the subject stays
    centred and the camera never faces away from the map.
    """
    ride = np.asarray(ride, dtype=np.float64)
    horiz = tangents - np.outer(tangents @ WORLD_UP, WORLD_UP)
    horiz /= np.maximum(np.linalg.norm(horiz, axis=1, keepdims=True), 1.0e-9)
    eyes = ride - horiz * float(follow_back) + WORLD_UP * float(lift)
    eyes = moving_average_edge(eyes, eye_smooth_window)
    focus = ride if targets is None else np.asarray(targets, dtype=np.float64)
    forwards = focus + WORLD_UP * float(look_up) - eyes
    forwards /= np.maximum(np.linalg.norm(forwards, axis=1, keepdims=True), 1.0e-9)
    return eyes, forwards


def build_viewmats(eyes: np.ndarray, forwards: np.ndarray) -> np.ndarray:
    """World-to-camera matrices (OpenCV: x right, y down, z forward).

    ``right = forward x up`` — the reversed cross product would roll the
    camera 180 degrees (upside-down and mirrored).
    """
    viewmats = np.empty((len(eyes), 4, 4), dtype=np.float32)
    prev_right = np.array([1.0, 0.0, 0.0])
    for i, (eye, forward) in enumerate(zip(eyes, forwards)):
        right = np.cross(forward, WORLD_UP)
        n = float(np.linalg.norm(right))
        if n > 1.0e-8:
            right = right / n
        else:
            # Near-vertical forward: keep the previous heading, re-projected
            # onto the plane orthogonal to the current forward.
            right = prev_right - forward * float(prev_right @ forward)
            right /= np.linalg.norm(right)
        down = np.cross(forward, right)
        down /= np.linalg.norm(down)
        c2w = np.eye(4)
        c2w[:3, 0] = right
        c2w[:3, 1] = down
        c2w[:3, 2] = forward
        c2w[:3, 3] = eye
        viewmats[i] = np.linalg.inv(c2w).astype(np.float32)
        prev_right = right
    return viewmats


def enhance_colors(rgb01: np.ndarray, *, saturation: float = 1.25,
                   percentiles: tuple[float, float] = (0.5, 99.8),
                   gamma: float = 1.0) -> np.ndarray:
    """Mild saturation/contrast/gamma boost for camera-projected point colours.

    Multi-view averaging plus indoor auto-exposure washes the projected
    colours out; this stretches them back without clipping more than the
    given percentiles. Input and output are float RGB in [0, 1].
    """
    rgb01 = np.asarray(rgb01, dtype=np.float64)
    lum = rgb01.mean(axis=1, keepdims=True)
    out = np.clip(lum + (rgb01 - lum) * float(saturation), 0.0, 1.0)
    lo, hi = np.percentile(out, list(percentiles))
    if not (np.isfinite(lo) and np.isfinite(hi)) or hi - lo < 1.0e-3:
        return out  # near-uniform colours: stretching would amplify noise
    return np.clip((out - lo) / (hi - lo), 0.0, 1.0) ** float(gamma)


def uncolored_mask(rgb_u8: np.ndarray, default_rgb=(128, 128, 128)) -> np.ndarray:
    """True for points still carrying the colorizer's default (unseen) colour.

    ``build_lidar_init.py --color-transforms`` paints points no camera ever
    saw with ``default_rgb``; those grey points smear the cutaway view and are
    dropped in rgb colour mode.
    """
    return np.all(np.asarray(rgb_u8) == np.asarray(default_rgb), axis=1)


def ceiling_cut_mask(xyz: np.ndarray, ride_positions: np.ndarray, height: float,
                     chunk: int = 50000) -> np.ndarray:
    """Keep-mask of points at most ``height`` above the *local* walking height.

    Each point is thresholded against the nearest (in XY) ride position, so the
    cut follows ramps instead of slicing the whole map at one global z. The XY
    nearest-neighbour lookup assumes a single walking level: multi-storey
    trajectories that overlap in XY would pick an arbitrary level's height.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    ref = np.asarray(ride_positions, dtype=np.float64)
    keep = np.empty(len(xyz), dtype=bool)
    for s0 in range(0, len(xyz), chunk):
        part = xyz[s0:s0 + chunk]
        d2 = ((part[:, None, :2] - ref[None, :, :2]) ** 2).sum(axis=2)
        zref = ref[np.argmin(d2, axis=1), 2]
        keep[s0:s0 + chunk] = part[:, 2] <= zref + float(height)
    return keep


# --------------------------------------------------------------------------- #
# Scene assembly + rendering (CUDA beyond this point)
# --------------------------------------------------------------------------- #
def build_scene(args: argparse.Namespace) -> dict:
    """Load poses, build the equal-speed follow-camera path and trajectory line."""
    dataset = load_transforms(args.transforms)
    K, width, height = scale_intrinsics(dataset['K'], dataset['width'],
                                        dataset['height'], args.scale)
    c2ws = np.linalg.inv(np.asarray(dataset['viewmats'], dtype=np.float64))
    raw_positions = c2ws[:, :3, 3]

    cinematic = args.camera_preset == 'cinematic'
    path_window = 11 if cinematic else 7
    tangent_window = 21 if cinematic else 15
    eye_window = 41 if cinematic else 31
    follow_back = (4.5 if cinematic and args.follow_back == 5.5
                   else args.follow_back)
    lift = 4.3 if cinematic and args.cam_lift == 5.5 else args.cam_lift
    look_up = 0.6 if cinematic and args.look_up == 0.8 else args.look_up
    look_ahead = 1.2 if cinematic and args.look_ahead == 0.0 else args.look_ahead
    corner_slowdown = (2.0 if cinematic and args.corner_slowdown == 0.0
                       else args.corner_slowdown)
    ride, total_len = resample_cinematic_arclength(
        moving_average_edge(raw_positions, path_window), args.frames,
        corner_slowdown)
    tangents = smooth_tangents(ride, tangent_window)
    targets = look_ahead_targets(ride, look_ahead)
    eyes, forwards = third_person_path(ride, tangents,
                                       follow_back=follow_back, lift=lift,
                                       look_up=look_up,
                                       eye_smooth_window=eye_window,
                                       targets=targets)
    line = resample_polyline(raw_positions, 0.05)
    line[:, 2] -= 0.5
    return {
        'K': np.asarray(K, dtype=np.float32), 'width': int(width), 'height': int(height),
        'raw_positions': raw_positions, 'ride': ride, 'eyes': eyes,
        'viewmats': build_viewmats(eyes, forwards), 'line': line,
        'total_len': total_len,
        'camera_parameters': {
            'preset': args.camera_preset, 'follow_back': follow_back,
            'lift': lift, 'look_up': look_up, 'look_ahead': look_ahead,
            'corner_slowdown': corner_slowdown,
        },
    }


def render_flythrough(scene: dict, args: argparse.Namespace,
                      frame_indices: np.ndarray) -> np.ndarray:
    """Render the selected frames (map + per-frame culled trajectory line)."""
    xyz, rgb = pcio.read_ply_xyz(args.pointcloud)
    keep = np.ones(len(xyz), dtype=bool)
    if args.ceiling_cut > 0.0:
        keep &= ceiling_cut_mask(xyz, scene['raw_positions'], args.ceiling_cut)
    if args.color_mode == 'rgb':
        if rgb is None:
            raise SystemExit('--color-mode rgb needs a coloured ply (rebuild the '
                             'init cloud with build_lidar_init.py --color-transforms)')
        keep &= ~uncolored_mask(rgb)
    print(f'ceiling cut / colour filter: {len(xyz)} -> {int(keep.sum())} points')
    if not keep.any():
        raise SystemExit('no map points survive the ceiling cut / colour filter '
                         '(is the ply colorized and does it overlap the trajectory?)')
    xyz = xyz[keep]
    if args.color_mode == 'rgb':
        kept_rgb = rgb[keep]
        if args.render_voxel > 0.0:
            xyz, kept_rgb = pcio.voxel_downsample(
                xyz, args.render_voxel, kept_rgb)
            print(f'render voxel {args.render_voxel:g} m -> {len(xyz)} points')
        colors = enhance_colors(kept_rgb.astype(np.float64) / 255.0,
                                saturation=args.saturation)
    else:
        if args.render_voxel > 0.0:
            xyz, _ = pcio.voxel_downsample(xyz, args.render_voxel)
            print(f'render voxel {args.render_voxel:g} m -> {len(xyz)} points')
        colors = height_colormap(xyz[:, 2])
    cloud = points_to_gaussians(xyz, colors, args.point_size, 0.95)
    surface_normals = None
    if args.surface_splat:
        surface_normals = pcio.estimate_voxel_normals(
            xyz, voxel=args.surface_normal_voxel)
    line = scene['line']
    line_rgb = np.tile(np.array([1.0, 40.0 / 255.0, 220.0 / 255.0]),
                       (len(line), 1))

    def render_one(j: int) -> np.ndarray:
        eye = scene['eyes'][j]
        keep_map = mask_far_from(cloud['means'], eye, 3.0)
        culled = {key: (val[keep_map] if isinstance(val, np.ndarray) else val)
                  for key, val in cloud.items()}
        keep_line = mask_far_from(line, eye, 2.5)
        traj = points_to_gaussians(line[keep_line], line_rgb[keep_line], 0.02, 0.99)
        merged = merge_gaussians(culled, traj)
        if surface_normals is not None:
            merged['surface_normals'] = np.concatenate([
                surface_normals[keep_map], np.zeros((len(traj['means']), 3))
            ], axis=0)
        return render_frames(merged,
                             scene['viewmats'][j][None], scene['K'],
                             scene['width'], scene['height'],
                             device=args.device,
                             soft_edge_px=args.soft_edge_px,
                             surface_splat=args.surface_splat,
                             surface_aspect_limit=args.surface_aspect_limit)[0]

    indices = [int(index) for index in frame_indices]
    if args.render_workers <= 1:
        rendered = [render_one(index) for index in indices]
    else:
        with ThreadPoolExecutor(max_workers=args.render_workers) as pool:
            rendered = list(pool.map(render_one, indices))
    return np.stack(rendered)


def flythrough_metrics(frames: np.ndarray) -> dict:
    """Return display-space coverage and temporal luminance diagnostics."""
    data = np.asarray(frames, dtype=np.uint8)
    occupied = np.any(data > 3, axis=3)
    black_fraction = float(1.0 - occupied.mean())
    luminance = np.tensordot(
        data.astype(np.float32), np.asarray([0.299, 0.587, 0.114]),
        axes=([-1], [0]))
    frame_luminance = np.asarray([
        np.median(lum[mask]) if mask.any() else 0.0
        for lum, mask in zip(luminance, occupied)
    ], dtype=np.float64)
    delta = np.abs(np.diff(frame_luminance)) / 255.0
    trend = moving_average_edge(frame_luminance[:, None], 9)[:, 0]
    flicker = np.abs(frame_luminance - trend) / 255.0
    return {
        'frames': int(len(data)),
        'black_pixel_fraction': black_fraction,
        'occupied_pixel_fraction': 1.0 - black_fraction,
        'frame_luminance_median': float(np.median(frame_luminance)),
        'temporal_luminance_delta_median': float(np.median(delta)) if len(delta) else 0.0,
        'temporal_luminance_delta_p90': float(np.percentile(delta, 90)) if len(delta) else 0.0,
        'temporal_flicker_median': float(np.median(flicker)),
        'temporal_flicker_p90': float(np.percentile(flicker, 90)),
    }


def write_metrics(path: Optional[str], frames: np.ndarray, scene: dict) -> None:
    """Write optional JSON metrics beside a rendered candidate."""
    if path is None:
        return
    report = flythrough_metrics(frames)
    report['camera'] = scene['camera_parameters']
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + '\n')
    print(f'wrote {out}')


def annotate(frames: np.ndarray, scene: dict, frame_indices: np.ndarray,
             label: str) -> np.ndarray:
    """Stamp the pane label and the travelled/ahead minimap onto every frame."""
    line = scene['line']
    progress = np.array([nearest_polyline_index(line, scene['ride'][j])
                         for j in frame_indices])
    size = 132
    minimap = {
        'px': minimap_points(line[:, :2], size, 12),
        'progress': progress,
        'start': int(progress[0]),
        'origin': (frames.shape[2] - size - 10, frames.shape[1] - size - 10),
        'size': size,
        'past_rgb': (255, 40, 220),
        'ahead_rgb': (165, 165, 165),
    }
    return annotate_frames(frames, [(0, label)], minimap=minimap)


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--pointcloud', required=True,
                   help='LiDAR map .ply (e.g. the lidar_init cloud '
                        'build_lidar_init.py accumulated with the SLAM trajectory)')
    p.add_argument('--transforms', required=True,
                   help='transforms.json giving intrinsics + the SLAM-estimated '
                        'poses whose positions define the ride path')
    p.add_argument('--mp4', default=None, help='output mp4 path')
    p.add_argument('--gif', default=None, help='optional downscaled GIF output')
    p.add_argument('--frames', type=int, default=300)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--gif-fps', type=int, default=12)
    p.add_argument('--gif-scale', type=float, default=0.5)
    p.add_argument('--scale', type=float, default=1.0,
                   help='render-resolution scale relative to the training images')
    p.add_argument('--point-size', type=float, default=0.03,
                   help='map point Gaussian sigma in metres (bigger than the '
                        'side-by-side default: the camera travels close to the cloud; '
                        'use ~0.018 for a dense camera-coloured cloud)')
    p.add_argument('--render-voxel', type=float, default=0.0,
                   help='display-only voxel thinning in metres; 0 keeps all points')
    p.add_argument('--render-workers', type=int, default=1,
                   help='parallel CPU frames; 1 preserves serial rendering')
    p.add_argument('--color-mode', choices=('height', 'rgb'), default='height',
                   help='height = cold-to-warm height ramp; rgb = the ply\'s own '
                        'camera-projected colours (photoreal-coloured map; needs '
                        'build_lidar_init.py --color-transforms), enhanced via '
                        '--saturation and with unseen grey points dropped')
    p.add_argument('--saturation', type=float, default=1.25,
                   help='saturation boost for --color-mode rgb (1.0 = off)')
    p.add_argument('--cam-lift', type=float, default=5.5,
                   help='camera height above the ride point in metres')
    p.add_argument('--follow-back', type=float, default=5.5,
                   help='camera distance behind the ride point in metres')
    p.add_argument('--camera-preset', choices=('legacy', 'cinematic'),
                   default='legacy',
                   help='cinematic moves closer, looks ahead, and slows at turns')
    p.add_argument('--look-ahead', type=float, default=0.0,
                   help='focus this many metres ahead along the trajectory')
    p.add_argument('--look-up', type=float, default=0.8,
                   help='vertical focus offset above the trajectory (m)')
    p.add_argument('--corner-slowdown', type=float, default=0.0,
                   help='extra frame density around turns; 0 keeps equal speed')
    p.add_argument('--ceiling-cut', type=float, default=2.3,
                   help='drop map points more than this many metres above the '
                        'local walking height (<= 0 disables the cutaway)')
    p.add_argument('--loop-fade', type=int, default=12,
                   help='fade the first/last N frames from/to black so the '
                        'one-way ride loops without a hard jump cut')
    p.add_argument('--label', default='LiDAR SLAM map + trajectory (full loop)')
    p.add_argument('--test-grid', type=int, default=0,
                   help='render only N evenly spaced frames into a grid PNG '
                        '(--test-grid-out) instead of a video')
    p.add_argument('--test-grid-out', default='output/map_flythrough_testgrid.png')
    p.add_argument('--dry-run', action='store_true',
                   help='build the camera path, print sanity stats and exit')
    p.add_argument('--device', default='cuda')
    p.add_argument('--soft-edge-px', type=float, default=0.0,
                   help='CPU-only outer disc fade width in output pixels; 0 off')
    p.add_argument('--surface-splat', action='store_true',
                   help='CPU-only normal-aligned elliptical map splats')
    p.add_argument('--surface-aspect-limit', type=float, default=3.0)
    p.add_argument('--surface-normal-voxel', type=float, default=0.12)
    p.add_argument('--metrics-out', default=None,
                   help='optional JSON black-pixel and temporal-flicker report')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    if args.frames < 2:
        raise SystemExit('--frames must be at least 2')
    if args.render_workers < 1:
        raise SystemExit('--render-workers must be at least 1')
    if args.surface_normal_voxel <= 0.0:
        raise SystemExit('--surface-normal-voxel must be positive')
    if args.surface_aspect_limit < 1.0:
        raise SystemExit('--surface-aspect-limit must be at least 1')
    if not (args.dry_run or args.test_grid > 0 or args.mp4 or args.gif):
        raise SystemExit('nothing to do: pass --mp4 and/or --gif')

    scene = build_scene(args)
    if args.dry_run:
        step = np.linalg.norm(np.diff(scene['ride'], axis=0), axis=1)
        vm = scene['viewmats']
        print(f'viewmats: {vm.shape}, NaN: {bool(np.isnan(vm).any())}')
        print(f'ride length: {scene["total_len"]:.2f} m, step min/max/mean: '
              f'{step.min():.4f}/{step.max():.4f}/{step.mean():.4f} m')
        return 0

    if args.test_grid > 0:
        idx = np.unique(np.linspace(0, args.frames - 1, args.test_grid).astype(int))
        frames = annotate(render_flythrough(scene, args, idx), scene, idx, args.label)
        from PIL import Image
        n, h, w = frames.shape[:3]
        cols = min(4, n)
        rows = (n + cols - 1) // cols
        grid = Image.new('RGB', (cols * w, rows * h))
        for i in range(n):
            grid.paste(Image.fromarray(frames[i]), ((i % cols) * w, (i // cols) * h))
        out = Path(args.test_grid_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        grid.save(out)
        print(f'wrote {out}')
        write_metrics(args.metrics_out, frames, scene)
        return 0

    idx = np.arange(args.frames)
    frames = annotate(render_flythrough(scene, args, idx), scene, idx, args.label)
    if args.loop_fade > 0:
        w = fade_weights(len(frames), args.loop_fade)
        frames = (frames.astype(np.float32) * w[:, None, None, None]).astype(np.uint8)
    write_metrics(args.metrics_out, frames, scene)
    write_videos(frames, list(range(len(frames))), fps=args.fps,
                 mp4=Path(args.mp4) if args.mp4 else None,
                 gif=Path(args.gif) if args.gif else None,
                 gif_fps=args.gif_fps, gif_scale=args.gif_scale)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
