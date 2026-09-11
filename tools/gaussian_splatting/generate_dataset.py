#!/usr/bin/env python3
"""Generate an open-loop RGB-D perception dataset from a 3DGS scene (gsplat, Apache-2.0).

Phase 1 of the 3DGS-as-sim2real track. Phase 0 measured the *valid viewpoint
range* (how far the ego may deviate from the recorded trajectory before the
render breaks); this tool turns a trained LiDAR-primed scene into a labelled
RGB-D dataset *within* that range, ready to feed a perception model offline.

For every reference view it renders the recorded pose plus a handful of
jittered poses (lateral / vertical camera offsets, sampled deterministically
inside ``--max-lateral`` / ``--max-vertical`` so the augmentation never leaves
the valid range). Each frame is written as an 8-bit RGB png and a 16-bit depth
png (millimetres by default), with a nerfstudio-compatible ``transforms.json``
holding the intrinsics and per-frame camera-to-world pose. Because the model is
LiDAR-primed the depth is metric, so the dataset carries true range labels.

The pure helpers (depth quantisation, jitter planning, pose offsetting, the
manifest builder) are numpy-only and unit tested on CPU; rendering needs
CUDA + torch + gsplat.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

import lidarslam_benchmark_tools.gaussian_splatting.posed_images as pi
from lidarslam_benchmark_tools.gaussian_splatting.render_path import load_gaussian_ply, render_rgbd_frames, scale_intrinsics
from lidarslam_benchmark_tools.gaussian_splatting.sim2real_gap import offset_viewmat, select_views
from lidarslam_benchmark_tools.gaussian_splatting.train_gsplat import load_transforms


# --------------------------------------------------------------------------- #
# Pure helpers (no torch/CUDA)
# --------------------------------------------------------------------------- #
def depth_to_uint16(depth_m: np.ndarray, *, depth_scale: float = 0.001,
                    max_depth: float = 0.0) -> np.ndarray:
    """Quantise a metric depth map to uint16 units of ``depth_scale`` metres.

    With the default ``depth_scale`` of 0.001 the stored integers are
    millimetres (the common RGB-D / KITTI convention). Non-finite and negative
    samples become 0 (= "no return"); values past ``max_depth`` (when > 0) and
    past the uint16 ceiling are clamped.
    """
    if depth_scale <= 0.0:
        raise ValueError('depth_scale must be positive')
    d = np.asarray(depth_m, dtype=np.float64)
    d = np.where(np.isfinite(d) & (d > 0.0), d, 0.0)
    if max_depth > 0.0:
        d = np.minimum(d, max_depth)
    u = np.rint(d / depth_scale)
    return np.minimum(u, 65535.0).astype(np.uint16)


def uint16_to_depth(u16: np.ndarray, *, depth_scale: float = 0.001) -> np.ndarray:
    """Inverse of :func:`depth_to_uint16`: uint16 units back to metric depth."""
    return np.asarray(u16, dtype=np.float64) * depth_scale


def plan_jitters(n_aug: int, *, max_lateral: float, max_vertical: float,
                 seed: int = 0) -> list[tuple[float, float]]:
    """Deterministic ``(dx_right, dy_down)`` camera offsets, the recorded pose first.

    Always returns ``(0.0, 0.0)`` as the first entry (the recorded view) followed
    by ``n_aug`` samples drawn uniformly from the box
    ``[-max_lateral, max_lateral] x [-max_vertical, max_vertical]``. Seeded, so a
    given ``(n_aug, ranges, seed)`` always yields the same set -- the dataset is
    reproducible byte-for-byte.
    """
    if n_aug < 0:
        raise ValueError('n_aug must be non-negative')
    out = [(0.0, 0.0)]
    if n_aug == 0:
        return out
    rng = np.random.default_rng(seed)
    dx = rng.uniform(-max_lateral, max_lateral, n_aug)
    dy = rng.uniform(-max_vertical, max_vertical, n_aug)
    out.extend((float(x), float(y)) for x, y in zip(dx, dy))
    return out


def jittered_viewmat(viewmat: np.ndarray, dx_right: float,
                     dy_down: float) -> np.ndarray:
    """Offset a world->camera view along its local right (x) then down (y) axes."""
    return offset_viewmat(offset_viewmat(viewmat, dx_right, 'x'), dy_down, 'y')


def c2w_to_opengl(viewmat: np.ndarray) -> np.ndarray:
    """Camera-to-world in OpenGL convention (nerfstudio ``transform_matrix``).

    Inverts the gsplat OpenCV world->camera ``viewmat`` and applies the
    OpenCV->OpenGL optical flip, matching what ``train_gsplat.load_transforms``
    expects to read back.
    """
    c2w_cv = np.linalg.inv(np.asarray(viewmat, dtype=float))
    return c2w_cv @ pi.ROS_OPTICAL_TO_OPENGL


def build_manifest(K: np.ndarray, width: int, height: int,
                   frames: Sequence[dict], *, depth_scale: float) -> dict:
    """Assemble a nerfstudio-style ``transforms.json`` dict with depth metadata."""
    K = np.asarray(K, dtype=float)
    return {
        'w': int(width),
        'h': int(height),
        'fl_x': float(K[0, 0]),
        'fl_y': float(K[1, 1]),
        'cx': float(K[0, 2]),
        'cy': float(K[1, 2]),
        'camera_model': 'OPENCV',
        'depth_scale': float(depth_scale),
        'depth_unit': 'metres = pixel * depth_scale',
        'frames': list(frames),
    }


# --------------------------------------------------------------------------- #
# Generation (torch + gsplat; rendering imported lazily by render_rgbd_frames)
# --------------------------------------------------------------------------- #
def generate(ply: Path, transforms: Path, out_dir: Path, *,
             n_views: int, n_aug: int, max_lateral: float, max_vertical: float,
             scale: float, depth_scale: float, max_depth: float, seed: int,
             device: str = 'cuda') -> dict:
    """Render the RGB-D dataset and write rgb/, depth/ and transforms.json."""
    import imageio.v2 as imageio

    dataset = load_transforms(transforms)
    gaussians = load_gaussian_ply(ply)
    K, width, height = scale_intrinsics(dataset['K'], dataset['width'],
                                        dataset['height'], scale)
    views = select_views(len(dataset['viewmats']), n_views)
    jitters = plan_jitters(n_aug, max_lateral=max_lateral,
                           max_vertical=max_vertical, seed=seed)

    viewmats, records = [], []
    for gi in views:
        base = dataset['viewmats'][gi]
        for ji, (dx, dy) in enumerate(jitters):
            vm = jittered_viewmat(base, dx, dy)
            stem = f'view_{gi:04d}_j{ji:02d}'
            viewmats.append(vm)
            records.append({'stem': stem, 'view': int(gi), 'jitter': [dx, dy],
                            'viewmat': vm})

    (out_dir / 'rgb').mkdir(parents=True, exist_ok=True)
    (out_dir / 'depth').mkdir(parents=True, exist_ok=True)
    rgb, depth = render_rgbd_frames(gaussians, np.stack(viewmats), K, width,
                                    height, device=device)

    frames = []
    for i, rec in enumerate(records):
        stem = rec['stem']
        imageio.imwrite(out_dir / 'rgb' / f'{stem}.png', rgb[i])
        d16 = depth_to_uint16(depth[i], depth_scale=depth_scale,
                              max_depth=max_depth)
        imageio.imwrite(out_dir / 'depth' / f'{stem}.png', d16)
        frames.append({
            'file_path': f'rgb/{stem}.png',
            'depth_file_path': f'depth/{stem}.png',
            'transform_matrix': c2w_to_opengl(rec['viewmat']).tolist(),
            'source_view': rec['view'],
            'jitter_rightdown_m': rec['jitter'],
        })

    manifest = build_manifest(K, width, height, frames, depth_scale=depth_scale)
    (out_dir / 'transforms.json').write_text(json.dumps(manifest, indent=2))
    valid = depth[depth > 0.0]
    return {
        'frames': len(frames),
        'views': len(views),
        'jitters_per_view': len(jitters),
        'render_size': [width, height],
        'depth_scale': depth_scale,
        'depth_coverage': float(np.mean(depth > 0.0)),
        'depth_p50_m': float(np.median(valid)) if valid.size else 0.0,
        'depth_p95_m': float(np.percentile(valid, 95)) if valid.size else 0.0,
    }


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--ply', required=True, help='trained INRIA 3DGS .ply')
    p.add_argument('--transforms', required=True,
                   help='transforms.json the model was trained from')
    p.add_argument('--out', required=True, help='output dataset directory')
    p.add_argument('--views', type=int, default=0,
                   help='evenly spaced reference views (0 = all)')
    p.add_argument('--aug', type=int, default=0,
                   help='jittered poses per view, inside the valid range '
                        '(0 = recorded pose only)')
    p.add_argument('--max-lateral', type=float, default=0.5,
                   help='max camera-right jitter in metres (keep within the '
                        'Phase 0 valid viewpoint range for the scene)')
    p.add_argument('--max-vertical', type=float, default=0.1,
                   help='max camera-down jitter in metres')
    p.add_argument('--scale', type=float, default=0.5,
                   help='render-resolution scale vs the training images')
    p.add_argument('--depth-scale', type=float, default=0.001,
                   help='metres per stored uint16 depth unit (0.001 = mm)')
    p.add_argument('--max-depth', type=float, default=0.0,
                   help='clip depth to this many metres (0 = no clip)')
    p.add_argument('--seed', type=int, default=0,
                   help='jitter RNG seed (dataset is reproducible)')
    p.add_argument('--device', default='cuda')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    stats = generate(Path(args.ply), Path(args.transforms), Path(args.out),
                     n_views=args.views, n_aug=args.aug,
                     max_lateral=args.max_lateral, max_vertical=args.max_vertical,
                     scale=args.scale, depth_scale=args.depth_scale,
                     max_depth=args.max_depth, seed=args.seed, device=args.device)
    print(f"wrote {stats['frames']} RGB-D frames "
          f"({stats['views']} views x {stats['jitters_per_view']}) to {args.out}")
    print(f"  size {stats['render_size'][0]}x{stats['render_size'][1]}, "
          f"depth coverage {stats['depth_coverage']:.2f}, "
          f"median {stats['depth_p50_m']:.2f} m, p95 {stats['depth_p95_m']:.2f} m")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
