#!/usr/bin/env python3
"""Render a flythrough video from a trained 3DGS ``.ply`` (gsplat, Apache-2.0).

Loads an INRIA-layout Gaussian ``.ply`` (as written by ``train_gsplat.py``)
plus the ``transforms.json`` it was trained from, builds a smooth camera path
through the training views, renders each frame with the gsplat rasteriser, and
writes an mp4 (and optionally a downscaled GIF for README embedding).

The pure helpers (``load_gaussian_ply``, ``path_through_views``,
``matrix_to_quat_xyzw``, ``infer_sh_degree``, ``ping_pong_indices``) are
numpy-only and unit tested on CPU; rendering requires CUDA + torch + gsplat.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

import lidarslam_benchmark_tools.gaussian_splatting.posed_images as pi
from lidarslam_benchmark_tools.gaussian_splatting.train_gsplat import load_transforms, SH_C0


# --------------------------------------------------------------------------- #
# Pure helpers (no torch/CUDA)
# --------------------------------------------------------------------------- #
def load_gaussian_ply(path: str | Path) -> dict:
    """Read an INRIA-layout binary 3DGS ``.ply`` back into numpy arrays.

    Inverse of ``train_gsplat.export_ply``: returns a dict with ``means``,
    ``scales_log``, ``quats``, ``opacities_logit``, ``colors_rgb`` (0..1, from
    ``f_dc``), and ``sh_rest`` ((N, K-1, 3) from the channel-major ``f_rest_*``
    fields, or ``None`` when the file has only band 0).
    """
    path = Path(path)
    with open(path, 'rb') as fh:
        names: list[str] = []
        n = 0
        for raw in iter(fh.readline, b''):
            line = raw.decode('ascii').strip()
            if line.startswith('element vertex'):
                n = int(line.split()[-1])
            elif line.startswith('property'):
                kind, name = line.split()[1:3]
                if kind != 'float':
                    raise ValueError(f'unsupported property type {kind} in {path}')
                names.append(name)
            elif line == 'end_header':
                break
        arr = np.frombuffer(fh.read(4 * n * len(names)), dtype=np.float32)
    arr = arr.reshape(n, len(names))
    col = {name: arr[:, i] for i, name in enumerate(names)}
    f_dc = np.stack([col['f_dc_0'], col['f_dc_1'], col['f_dc_2']], axis=1)
    k_rest = sum(1 for name in names if name.startswith('f_rest_')) // 3
    sh_rest = None
    if k_rest:
        # INRIA channel-major: all coeffs of R, then G, then B.
        rest = np.stack([col[f'f_rest_{i}'] for i in range(3 * k_rest)], axis=1)
        sh_rest = rest.reshape(n, 3, k_rest).transpose(0, 2, 1)
    return {
        'means': np.stack([col['x'], col['y'], col['z']], axis=1),
        'scales_log': np.stack([col[f'scale_{i}'] for i in range(3)], axis=1),
        'quats': np.stack([col[f'rot_{i}'] for i in range(4)], axis=1),
        'opacities_logit': col['opacity'].copy(),
        'colors_rgb': f_dc * SH_C0 + 0.5,
        'sh_rest': sh_rest,
    }


def infer_sh_degree(sh_rest: Optional[np.ndarray]) -> Optional[int]:
    """SH degree from the higher-band coefficient count ((deg+1)^2 - 1)."""
    if sh_rest is None or sh_rest.shape[1] == 0:
        return None
    deg = int(round(np.sqrt(sh_rest.shape[1] + 1))) - 1
    if (deg + 1) ** 2 - 1 != sh_rest.shape[1]:
        raise ValueError(f'f_rest count {sh_rest.shape[1]} is not (deg+1)^2-1')
    return deg


def matrix_to_quat_xyzw(rot: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a unit xyzw quaternion (Shepperd)."""
    rot = np.asarray(rot, dtype=float)
    tr = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] >= rot[1, 1] and rot[0, 0] >= rot[2, 2]:
        s = np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] >= rot[2, 2]:
        s = np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return pi.quat_normalize(np.array([x, y, z, w]))


def path_through_views(c2ws: Sequence[np.ndarray], count: int, *,
                       smooth_window: int = 5) -> list[np.ndarray]:
    """Sample ``count`` camera-to-world poses along the ordered keyframe poses.

    Translation is piecewise-linearly interpolated then box-smoothed with
    ``smooth_window`` (odd, reflected padding; 1 disables smoothing) to take
    handheld jitter out of the ride; rotation is per-segment SLERP. Endpoint
    rotations match the first/last keyframes exactly.
    """
    if len(c2ws) < 2:
        raise ValueError('need at least two keyframe poses')
    if count < 2:
        raise ValueError('need at least two output frames')
    if smooth_window < 1 or smooth_window % 2 == 0:
        raise ValueError('smooth_window must be a positive odd integer')
    keys = np.stack([np.asarray(m, dtype=float)[:3, 3] for m in c2ws])
    quats = [matrix_to_quat_xyzw(np.asarray(m, dtype=float)[:3, :3]) for m in c2ws]
    ts = np.linspace(0.0, len(c2ws) - 1.0, count)
    idx = np.minimum(ts.astype(int), len(c2ws) - 2)
    alpha = ts - idx
    pos = keys[idx] * (1.0 - alpha)[:, None] + keys[idx + 1] * alpha[:, None]
    if smooth_window > 1:
        half = smooth_window // 2
        padded = np.concatenate([pos[half:0:-1], pos, pos[-2:-2 - half:-1]])
        kernel = np.full(smooth_window, 1.0 / smooth_window)
        pos = np.stack(
            [np.convolve(padded[:, a], kernel, mode='valid') for a in range(3)],
            axis=1)
    poses: list[np.ndarray] = []
    for k in range(count):
        i = int(idx[k])
        c2w = np.eye(4)
        c2w[:3, :3] = pi.quat_to_matrix(pi.quat_slerp(quats[i], quats[i + 1],
                                                      float(alpha[k])))
        c2w[:3, 3] = pos[k]
        poses.append(c2w)
    return poses


def ping_pong_indices(count: int) -> list[int]:
    """Frame order for a seamless forward-then-backward loop (no dup ends)."""
    if count < 2:
        return list(range(count))
    return list(range(count)) + list(range(count - 2, 0, -1))


def scale_intrinsics(K: np.ndarray, width: int, height: int,
                     scale: float) -> tuple[np.ndarray, int, int]:
    """Scale the pinhole intrinsics and image size (rounded to even pixels)."""
    if scale <= 0.0:
        raise ValueError('scale must be positive')
    w = max(2, int(round(width * scale)) // 2 * 2)
    h = max(2, int(round(height * scale)) // 2 * 2)
    out = np.asarray(K, dtype=float).copy()
    out[0, :] *= w / width
    out[1, :] *= h / height
    return out, w, h


# --------------------------------------------------------------------------- #
# Rendering (torch + gsplat; imported lazily)
# --------------------------------------------------------------------------- #
def render_frames(gaussians: dict, viewmats: np.ndarray, K: np.ndarray,
                  width: int, height: int, *, device: str = 'cuda') -> np.ndarray:
    """Rasterise every w2c view; returns uint8 frames (F, H, W, 3)."""
    import torch
    import torch.nn.functional as F

    from gsplat import rasterization

    dev = torch.device(device)
    means = torch.tensor(gaussians['means'], dtype=torch.float32, device=dev)
    quats = F.normalize(
        torch.tensor(gaussians['quats'], dtype=torch.float32, device=dev), dim=-1)
    scales = torch.exp(
        torch.tensor(gaussians['scales_log'], dtype=torch.float32, device=dev))
    opac = torch.sigmoid(
        torch.tensor(gaussians['opacities_logit'], dtype=torch.float32, device=dev))
    sh_degree = infer_sh_degree(gaussians['sh_rest'])
    if sh_degree is None:
        colors = torch.tensor(np.clip(gaussians['colors_rgb'], 0.0, 1.0),
                              dtype=torch.float32, device=dev)
    else:
        sh0 = (gaussians['colors_rgb'] - 0.5) / SH_C0
        sh = np.concatenate([sh0[:, None, :], gaussians['sh_rest']], axis=1)
        colors = torch.tensor(sh, dtype=torch.float32, device=dev)
    kmat = torch.tensor(K, dtype=torch.float32, device=dev)[None]
    frames = np.empty((len(viewmats), height, width, 3), dtype=np.uint8)
    with torch.no_grad():
        for i, vm in enumerate(viewmats):
            vmt = torch.tensor(vm, dtype=torch.float32, device=dev)[None]
            out, _, _ = rasterization(means, quats, scales, opac, colors, vmt,
                                      kmat, width, height, sh_degree=sh_degree,
                                      packed=False)
            frames[i] = (out[0].clamp(0.0, 1.0) * 255.0).to(torch.uint8).cpu().numpy()
    return frames


def render_rgbd_frames(gaussians: dict, viewmats: np.ndarray, K: np.ndarray,
                       width: int, height: int, *,
                       device: str = 'cuda') -> tuple[np.ndarray, np.ndarray]:
    """Rasterise every w2c view returning RGB and expected depth.

    Returns ``(rgb, depth)`` where ``rgb`` is uint8 ``(F, H, W, 3)`` and
    ``depth`` is float32 ``(F, H, W)`` of the gsplat *expected* depth (per-pixel
    opacity-weighted ray distance, in the Gaussian model's coordinate units).
    For a LiDAR-primed model those units are metric, so the depth is a usable
    range image for open-loop perception data generation (Phase 1).
    """
    import torch
    import torch.nn.functional as F

    from gsplat import rasterization

    dev = torch.device(device)
    means = torch.tensor(gaussians['means'], dtype=torch.float32, device=dev)
    quats = F.normalize(
        torch.tensor(gaussians['quats'], dtype=torch.float32, device=dev), dim=-1)
    scales = torch.exp(
        torch.tensor(gaussians['scales_log'], dtype=torch.float32, device=dev))
    opac = torch.sigmoid(
        torch.tensor(gaussians['opacities_logit'], dtype=torch.float32, device=dev))
    sh_degree = infer_sh_degree(gaussians['sh_rest'])
    if sh_degree is None:
        colors = torch.tensor(np.clip(gaussians['colors_rgb'], 0.0, 1.0),
                              dtype=torch.float32, device=dev)
    else:
        sh0 = (gaussians['colors_rgb'] - 0.5) / SH_C0
        sh = np.concatenate([sh0[:, None, :], gaussians['sh_rest']], axis=1)
        colors = torch.tensor(sh, dtype=torch.float32, device=dev)
    kmat = torch.tensor(K, dtype=torch.float32, device=dev)[None]
    rgb = np.empty((len(viewmats), height, width, 3), dtype=np.uint8)
    depth = np.empty((len(viewmats), height, width), dtype=np.float32)
    with torch.no_grad():
        for i, vm in enumerate(viewmats):
            vmt = torch.tensor(vm, dtype=torch.float32, device=dev)[None]
            out, _, _ = rasterization(means, quats, scales, opac, colors, vmt,
                                      kmat, width, height, sh_degree=sh_degree,
                                      packed=False, render_mode='RGB+ED')
            rgb[i] = (out[0, ..., :3].clamp(0.0, 1.0) * 255.0).to(
                torch.uint8).cpu().numpy()
            depth[i] = out[0, ..., 3].cpu().numpy()
    return rgb, depth


def write_videos(frames: np.ndarray, order: Sequence[int], *, fps: int,
                 mp4: Optional[Path], gif: Optional[Path], gif_fps: int,
                 gif_scale: float) -> None:
    """Write the rendered frames as mp4 and/or a downscaled GIF."""
    import imageio.v2 as imageio

    if mp4 is not None:
        mp4.parent.mkdir(parents=True, exist_ok=True)
        with imageio.get_writer(mp4, fps=fps, codec='libx264', quality=8,
                                macro_block_size=2) as writer:
            for i in order:
                writer.append_data(frames[i])
        print(f'wrote {mp4}')
    if gif is not None:
        gif.parent.mkdir(parents=True, exist_ok=True)
        step = max(1, round(fps / gif_fps))
        sub = [i for k, i in enumerate(order) if k % step == 0]
        h = int(frames.shape[1] * gif_scale) // 2 * 2
        w = int(frames.shape[2] * gif_scale) // 2 * 2
        try:
            from PIL import Image

            small = [np.asarray(Image.fromarray(frames[i]).resize((w, h)))
                     for i in sub]
        except ImportError:  # nearest-neighbour fallback without Pillow
            ys = (np.arange(h) / gif_scale).astype(int)
            xs = (np.arange(w) / gif_scale).astype(int)
            small = [frames[i][ys][:, xs] for i in sub]
        imageio.mimsave(gif, small, fps=gif_fps, loop=0)
        print(f'wrote {gif}')


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--ply', required=True, help='trained INRIA 3DGS .ply')
    p.add_argument('--transforms', required=True,
                   help='transforms.json the model was trained from (camera '
                        'intrinsics + keyframe poses for the path)')
    p.add_argument('--mp4', default=None, help='output mp4 path')
    p.add_argument('--gif', default=None, help='optional downscaled GIF output')
    p.add_argument('--frames', type=int, default=240,
                   help='rendered frames along the path (default 240)')
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--gif-fps', type=int, default=12)
    p.add_argument('--gif-scale', type=float, default=0.5)
    p.add_argument('--smooth-window', type=int, default=5,
                   help='odd box-filter width over the camera positions '
                        '(1 = no smoothing)')
    p.add_argument('--ping-pong', action='store_true',
                   help='append the reversed frames for a seamless loop')
    p.add_argument('--scale', type=float, default=1.0,
                   help='render-resolution scale relative to the training '
                        'images (e.g. 0.25)')
    p.add_argument('--rotate', type=int, default=0, choices=(0, 90, 180, 270),
                   help='rotate output frames clockwise (for sideways-mounted '
                        'cameras)')
    p.add_argument('--device', default='cuda')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    if args.mp4 is None and args.gif is None:
        raise SystemExit('nothing to do: pass --mp4 and/or --gif')
    dataset = load_transforms(args.transforms)
    gaussians = load_gaussian_ply(args.ply)
    c2ws = [np.linalg.inv(vm) for vm in dataset['viewmats']]
    path = path_through_views(c2ws, args.frames, smooth_window=args.smooth_window)
    viewmats = np.stack([np.linalg.inv(p) for p in path])
    K, width, height = scale_intrinsics(dataset['K'], dataset['width'],
                                        dataset['height'], args.scale)
    print(f'{gaussians["means"].shape[0]} gaussians, '
          f'sh_degree={infer_sh_degree(gaussians["sh_rest"])}, '
          f'{len(path)} frames at {width}x{height}')
    frames = render_frames(gaussians, viewmats, K, width, height,
                           device=args.device)
    if args.rotate:
        # np.rot90 is counter-clockwise; --rotate is clockwise degrees.
        frames = np.ascontiguousarray(
            np.rot90(frames, k=-args.rotate // 90, axes=(1, 2)))
    order = ping_pong_indices(args.frames) if args.ping_pong else list(range(args.frames))
    write_videos(frames, order, fps=args.fps,
                 mp4=Path(args.mp4) if args.mp4 else None,
                 gif=Path(args.gif) if args.gif else None,
                 gif_fps=args.gif_fps, gif_scale=args.gif_scale)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
