#!/usr/bin/env python3
"""Render a synced side-by-side "LiDAR SLAM map | 3DGS" flythrough video.

Left pane: the LiDAR point-cloud map the SLAM stack built (height-coloured)
with the estimated trajectory drawn as a line; right pane: the trained 3DGS
scene. Both panes are rendered along the same smoothed camera path through
the SLAM-estimated poses, so the video shows "LiDAR SLAM result" and
"photoreal 3DGS" from identical viewpoints, frame by frame.

The point cloud and trajectory reuse the gsplat rasteriser of the 3DGS pane
by wrapping each point as a tiny isotropic Gaussian -- no extra renderer or
dependency. The pure helpers (``height_colormap``, ``resample_polyline``,
``points_to_gaussians``, ``merge_gaussians``, ``hstack_panes``) are
numpy-only and unit tested on CPU; rendering needs CUDA + torch + gsplat.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

import lidarslam_benchmark_tools.gaussian_splatting.pointcloud_io as pcio
from lidarslam_benchmark_tools.gaussian_splatting.render_path import (
    load_gaussian_ply, path_through_views, ping_pong_indices, render_frames,
    scale_intrinsics, write_videos)
from lidarslam_benchmark_tools.gaussian_splatting.train_gsplat import load_transforms


# Cold-to-warm height ramp (low -> high): indigo, blue, teal, green, yellow, red.
# The lowest stop stays bright enough that the floor doesn't vanish into the
# black background.
HEIGHT_RAMP = np.array([
    [0.27, 0.15, 0.55],
    [0.13, 0.37, 0.66],
    [0.13, 0.72, 0.67],
    [0.37, 0.85, 0.30],
    [0.95, 0.78, 0.19],
    [0.94, 0.33, 0.15],
])


# --------------------------------------------------------------------------- #
# Pure helpers (no torch/CUDA)
# --------------------------------------------------------------------------- #
def height_colormap(values: np.ndarray, lo: Optional[float] = None,
                    hi: Optional[float] = None) -> np.ndarray:
    """Map scalars to RGB (N,3 in 0..1) over the cold-to-warm ``HEIGHT_RAMP``.

    ``lo``/``hi`` default to the 2nd/98th percentiles so a few outlier points
    (ceiling fixtures, stray returns) don't compress the ramp for the rest.
    """
    v = np.asarray(values, dtype=np.float64)
    lo = float(np.percentile(v, 2.0)) if lo is None else float(lo)
    hi = float(np.percentile(v, 98.0)) if hi is None else float(hi)
    if hi <= lo:
        hi = lo + 1e-6
    t = np.clip((v - lo) / (hi - lo), 0.0, 1.0)
    x = t * (len(HEIGHT_RAMP) - 1)
    i = np.minimum(x.astype(int), len(HEIGHT_RAMP) - 2)
    a = (x - i)[:, None]
    return HEIGHT_RAMP[i] * (1.0 - a) + HEIGHT_RAMP[i + 1] * a


def resample_polyline(points: np.ndarray, spacing: float) -> np.ndarray:
    """Resample an ordered polyline to ~evenly spaced points (keeps endpoints)."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] < 2:
        return pts.copy()
    if spacing <= 0.0:
        raise ValueError('spacing must be positive')
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 0.0:
        return pts[:1].copy()
    si = np.linspace(0.0, s[-1], max(2, int(round(s[-1] / spacing)) + 1))
    return np.stack([np.interp(si, s, pts[:, a]) for a in range(3)], axis=1)


def points_to_gaussians(xyz: np.ndarray, colors_rgb: np.ndarray, size: float,
                        opacity: float = 0.95) -> dict:
    """Wrap raw points as tiny isotropic Gaussians for ``render_frames``.

    Returns the same dict layout as ``load_gaussian_ply`` (``sh_rest`` None),
    so the point-cloud pane renders through the identical gsplat path as the
    3DGS pane. ``size`` is the Gaussian sigma in metres.
    """
    if size <= 0.0:
        raise ValueError('size must be positive')
    if not 0.0 < opacity < 1.0:
        raise ValueError('opacity must be in (0, 1)')
    xyz = np.asarray(xyz, dtype=np.float32)
    n = xyz.shape[0]
    quats = np.zeros((n, 4), dtype=np.float32)
    quats[:, 0] = 1.0  # wxyz identity
    return {
        'means': xyz,
        'scales_log': np.full((n, 3), np.log(size), dtype=np.float32),
        'quats': quats,
        'opacities_logit': np.full(n, np.log(opacity / (1.0 - opacity)),
                                   dtype=np.float32),
        'colors_rgb': np.asarray(colors_rgb, dtype=np.float32),
        'sh_rest': None,
    }


def merge_gaussians(a: dict, b: dict) -> dict:
    """Concatenate two band-0-only Gaussian dicts (``sh_rest`` must be None)."""
    if a['sh_rest'] is not None or b['sh_rest'] is not None:
        raise ValueError('merge_gaussians only supports sh_rest=None sets')
    out = {key: np.concatenate([a[key], b[key]], axis=0)
           for key in ('means', 'scales_log', 'quats', 'opacities_logit', 'colors_rgb')}
    out['sh_rest'] = None
    return out


def nearest_polyline_index(line: np.ndarray, pos: np.ndarray) -> int:
    """Index of the polyline point nearest to ``pos`` (walk progress lookup).

    The walking window never revisits itself, so nearest-point is a robust
    "how far along the trajectory is the camera" measure.
    """
    line = np.asarray(line, dtype=np.float64)
    return int(np.argmin(np.linalg.norm(
        line - np.asarray(pos, dtype=np.float64), axis=1)))


def fade_weights(count: int, fade: int) -> np.ndarray:
    """Per-frame brightness weights for a fade-out/in loop seam.

    The first and last ``fade`` frames ramp from/to near-black so a one-way
    ride loops with a brief dip instead of a hard jump cut. ``fade`` 0 returns
    all ones.
    """
    w = np.ones(count)
    if fade > 0:
        if 2 * fade > count:
            raise ValueError('fade longer than half the sequence')
        ramp = np.linspace(0.0, 1.0, fade + 1)[1:]
        w[:fade] = ramp
        w[count - fade:] = ramp[::-1]
    return w


def minimap_points(xy: np.ndarray, size: int, margin: int) -> np.ndarray:
    """Map world XY points into a ``size`` x ``size`` pixel box (top-down).

    Aspect ratio is preserved (the shorter axis is centred) and the world Y
    axis points up on the map (image rows grow downward).
    """
    xy = np.asarray(xy, dtype=np.float64)
    mn = xy.min(axis=0)
    span = xy.max(axis=0) - mn
    scale = (size - 2.0 * margin) / max(float(span.max()), 1e-9)
    offset = margin + (size - 2.0 * margin - span * scale) / 2.0
    px = offset + (xy - mn) * scale
    px[:, 1] = size - px[:, 1]
    return px


def mask_far_from(points: np.ndarray, centre: np.ndarray,
                  radius: float) -> np.ndarray:
    """Boolean mask of ``points`` farther than ``radius`` from ``centre``.

    The camera flies along the trajectory itself, so trajectory Gaussians
    right next to the camera project as huge blobs that wash out the frame;
    each frame culls the line within this radius of its own camera centre.
    """
    pts = np.asarray(points, dtype=np.float64)
    return np.linalg.norm(pts - np.asarray(centre, dtype=np.float64),
                          axis=1) > radius


def hstack_panes(left: np.ndarray, right: np.ndarray, divider: int = 2,
                 divider_rgb: Sequence[int] = (255, 255, 255)) -> np.ndarray:
    """Horizontally stack two (F,H,W,3) frame stacks with a divider column."""
    left = np.asarray(left)
    right = np.asarray(right)
    if left.shape[0] != right.shape[0] or left.shape[1] != right.shape[1]:
        raise ValueError('panes must share frame count and height')
    if divider <= 0:
        return np.concatenate([left, right], axis=2)
    div = np.tile(np.asarray(divider_rgb, dtype=np.uint8)[None, None, None, :],
                  (left.shape[0], left.shape[1], divider, 1))
    return np.concatenate([left, div, right], axis=2)


# --------------------------------------------------------------------------- #
# Annotations (PIL; cosmetic only)
# --------------------------------------------------------------------------- #
def annotate_frames(frames: np.ndarray, labels: Sequence[tuple[int, str]], *,
                    minimap: Optional[dict] = None, pad: int = 8,
                    font_size: int = 17) -> np.ndarray:
    """Stamp pane labels and an optional top-down minimap onto every frame.

    ``minimap`` (all keys required): ``px`` (N,2 pixel polyline from
    ``minimap_points``), ``progress`` (per-frame index into the polyline of
    the camera position), ``start`` (polyline index where the ride begins;
    the bright travelled segment grows from there), ``origin`` (x, y of the
    map box), ``size``, ``past_rgb``, ``ahead_rgb``. Edits ``frames`` in
    place.
    """
    from PIL import Image, ImageDraw, ImageFont

    try:
        font = ImageFont.truetype(
            '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', font_size)
    except OSError:
        font = ImageFont.load_default()
    for i in range(frames.shape[0]):
        img = Image.fromarray(frames[i]).convert('RGBA')
        if minimap is not None:
            box = Image.new('RGBA', (minimap['size'], minimap['size']),
                            (0, 0, 0, 150))
            mdraw = ImageDraw.Draw(box)
            pts = [tuple(p) for p in minimap['px']]
            k = int(minimap['progress'][i])
            start = int(minimap['start'])
            if len(pts) > 1:
                mdraw.line(pts, fill=tuple(minimap['ahead_rgb']), width=2)
            if k > start:
                mdraw.line(pts[start:k + 1], fill=tuple(minimap['past_rgb']),
                           width=3)
            cx, cy = pts[min(k, len(pts) - 1)]
            mdraw.ellipse([cx - 4, cy - 4, cx + 4, cy + 4],
                          fill=(255, 255, 255), outline=tuple(minimap['past_rgb']))
            img.alpha_composite(box, dest=tuple(minimap['origin']))
        draw = ImageDraw.Draw(img)
        for x, text in labels:
            draw.text((x + pad, pad), text, font=font, fill=(255, 255, 255),
                      stroke_width=2, stroke_fill=(0, 0, 0))
        frames[i] = np.asarray(img.convert('RGB'))
    return frames


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--ply', required=True, help='trained INRIA 3DGS .ply (right pane)')
    p.add_argument('--pointcloud', required=True,
                   help='LiDAR map .ply for the left pane (e.g. the lidar_init '
                        'cloud build_lidar_init.py accumulated with the SLAM '
                        'trajectory)')
    p.add_argument('--transforms', required=True,
                   help='transforms.json giving intrinsics + the keyframe poses '
                        'the camera path flies through')
    p.add_argument('--traj-transforms', default=None,
                   help='transforms.json whose camera centres draw the trajectory '
                        'line (default: --transforms; pass the full-window file '
                        'to show the whole walked loop)')
    p.add_argument('--mp4', default=None, help='output mp4 path')
    p.add_argument('--gif', default=None, help='optional downscaled GIF output')
    p.add_argument('--frames', type=int, default=240)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--gif-fps', type=int, default=12)
    p.add_argument('--gif-scale', type=float, default=0.5)
    p.add_argument('--smooth-window', type=int, default=5)
    p.add_argument('--ping-pong', action='store_true')
    p.add_argument('--scale', type=float, default=1.0,
                   help='render-resolution scale relative to the training images')
    p.add_argument('--point-size', type=float, default=0.008,
                   help='map point Gaussian sigma in metres')
    p.add_argument('--point-opacity', type=float, default=0.95)
    p.add_argument('--traj-radius', type=float, default=0.04,
                   help='trajectory line Gaussian sigma in metres')
    p.add_argument('--traj-spacing', type=float, default=0.05,
                   help='trajectory resample spacing in metres')
    p.add_argument('--traj-z-offset', type=float, default=-0.4,
                   help='drop the trajectory line below the camera height so it '
                        'reads as a path instead of blocking the view (metres)')
    p.add_argument('--traj-cull-radius', type=float, default=1.0,
                   help='per frame, hide trajectory points within this distance '
                        'of the camera (they would blob over the whole view)')
    p.add_argument('--traj-color', default='255,40,220',
                   help='trajectory RGB as r,g,b in 0..255 (magenta by default: '
                        'the only hue not used by the height ramp)')
    p.add_argument('--loop-fade', type=int, default=0,
                   help='fade the first/last N frames from/to black so a '
                        'one-way ride loops without a hard jump cut')
    p.add_argument('--no-minimap', action='store_true',
                   help='disable the top-down trajectory minimap inset')
    p.add_argument('--minimap-size', type=int, default=132)
    p.add_argument('--label-left', default='LiDAR SLAM map + trajectory')
    p.add_argument('--label-right', default='3DGS render')
    p.add_argument('--no-labels', action='store_true')
    p.add_argument('--device', default='cuda')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    if args.mp4 is None and args.gif is None:
        raise SystemExit('nothing to do: pass --mp4 and/or --gif')

    dataset = load_transforms(args.transforms)
    c2ws = [np.linalg.inv(vm) for vm in dataset['viewmats']]
    path = path_through_views(c2ws, args.frames, smooth_window=args.smooth_window)
    viewmats = np.stack([np.linalg.inv(p) for p in path])
    K, width, height = scale_intrinsics(dataset['K'], dataset['width'],
                                        dataset['height'], args.scale)

    # Left pane: SLAM point-cloud map (height-coloured) + estimated trajectory.
    xyz, _ = pcio.read_ply_xyz(args.pointcloud)
    cloud = points_to_gaussians(xyz, height_colormap(xyz[:, 2]),
                                args.point_size, args.point_opacity)
    traj_ds = (load_transforms(args.traj_transforms)
               if args.traj_transforms else dataset)
    centres = np.stack([np.linalg.inv(vm)[:3, 3] for vm in traj_ds['viewmats']])
    line = resample_polyline(centres, args.traj_spacing)
    line[:, 2] += args.traj_z_offset
    rgb = np.array([float(c) / 255.0 for c in args.traj_color.split(',')])
    traj_rgb = np.tile(rgb, (line.shape[0], 1))
    print(f'left pane: {xyz.shape[0]} map points + {line.shape[0]} trajectory points')
    # Walk progress per frame, for the minimap's travelled/ahead split. The
    # first-person pane keeps a single bright line: the camera only ever sees
    # the part of the trajectory ahead of it, so a progress split there would
    # just dim the whole visible line.
    progress = np.array([nearest_polyline_index(line, p[:3, 3]) for p in path])
    # Render the left pane frame by frame: the trajectory near the current
    # camera must be culled per frame (see mask_far_from).
    left = np.empty((len(viewmats), height, width, 3), dtype=np.uint8)
    for i, vm in enumerate(viewmats):
        keep = mask_far_from(line, path[i][:3, 3], args.traj_cull_radius)
        traj = points_to_gaussians(line[keep], traj_rgb[keep],
                                   args.traj_radius, 0.99)
        left[i] = render_frames(merge_gaussians(cloud, traj), vm[None], K,
                                width, height, device=args.device)[0]

    # Right pane: the trained 3DGS scene from the identical camera path.
    right = render_frames(load_gaussian_ply(args.ply), viewmats, K, width, height,
                          device=args.device)

    frames = hstack_panes(left, right)
    labels = ([] if args.no_labels
              else [(0, args.label_left), (width + 2, args.label_right)])
    minimap = None
    if not args.no_minimap:
        size = args.minimap_size
        minimap = {
            'px': minimap_points(line[:, :2], size, 12),
            'progress': progress,
            'start': int(progress[0]),
            'origin': (frames.shape[2] - size - 10,
                       frames.shape[1] - size - 10),
            'size': size,
            'past_rgb': tuple(int(c) for c in args.traj_color.split(',')),
            'ahead_rgb': (165, 165, 165),
        }
    if labels or minimap is not None:
        annotate_frames(frames, labels, minimap=minimap)
    order = (ping_pong_indices(args.frames) if args.ping_pong
             else list(range(args.frames)))
    seq = frames[np.asarray(order)]
    if args.loop_fade > 0:
        w = fade_weights(len(seq), args.loop_fade)
        seq = (seq.astype(np.float32) * w[:, None, None, None]).astype(np.uint8)
    write_videos(seq, list(range(len(seq))), fps=args.fps,
                 mp4=Path(args.mp4) if args.mp4 else None,
                 gif=Path(args.gif) if args.gif else None,
                 gif_fps=args.gif_fps, gif_scale=args.gif_scale)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
