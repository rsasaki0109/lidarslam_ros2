#!/usr/bin/env python3
"""Opt-in GPU self-test: prove the gsplat training path works end-to-end.

Renders a synthetic scene of Gaussians from a ring of cameras, serialises the
views through the real ``posed_images.write_transforms`` writer, then runs
``train_gsplat`` to reconstruct from scratch and exports a ``.ply``. Asserts
the photometric loss drops, so a green run means the whole render -> transforms
-> train -> export chain is healthy on this machine.

Requires a CUDA device + torch + gsplat. Not part of the ament/CI suite (those
runners are CPU-only); run it manually on a GPU host:

    python3 tools/gaussian_splatting/selftest_gpu.py --out /tmp/gsplat_selftest
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

import lidarslam_benchmark_tools.gaussian_splatting.posed_images as pi
import lidarslam_benchmark_tools.gaussian_splatting.train_gsplat as tg


def run(out_dir: str | Path, *, iters: int = 800, views: int = 12,
        width: int = 160, height: int = 120, device: str = 'cuda') -> dict:
    """Render synthetic views, reconstruct, and return a summary dict."""
    import torch
    import torch.nn.functional as F
    import imageio.v3 as iio
    from gsplat import rasterization

    out_dir = Path(out_dir)
    (out_dir / 'images').mkdir(parents=True, exist_ok=True)
    dev = torch.device(device)

    fx = fy = float(height)
    K = torch.tensor([[fx, 0.0, width / 2], [0.0, fy, height / 2],
                      [0.0, 0.0, 1.0]], device=dev)[None]
    intr = pi.CameraIntrinsics(width, height, fx, fy, width / 2, height / 2)

    rng = np.random.default_rng(42)
    n = 400
    means = torch.tensor(rng.normal(scale=0.4, size=(n, 3)).astype(np.float32), device=dev)
    quats = F.normalize(torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=dev).repeat(n, 1), dim=-1)
    scales = torch.full((n, 3), 0.05, device=dev)
    opac = torch.full((n,), 0.9, device=dev)
    cols = torch.tensor(rng.uniform(size=(n, 3)).astype(np.float32), device=dev)

    frames = []
    for i, c2w in enumerate(tg.looks_at_poses(radius=3.0, count=views, height=0.5)):
        viewmat = torch.tensor(np.linalg.inv(c2w), dtype=torch.float32, device=dev)[None]
        with torch.no_grad():
            img, _, _ = rasterization(means, quats, scales, opac, cols,
                                      viewmat, K, width, height)
        arr = (img[0].clamp(0, 1).cpu().numpy() * 255).astype(np.uint8)
        rel = f'images/{i:03d}.png'
        iio.imwrite(out_dir / rel, arr)
        frames.append(pi.PosedImage(rel, np.asarray(c2w), float(i)))

    pi.write_transforms(out_dir / 'transforms.json', intr, frames)

    dataset = tg.load_transforms(out_dir / 'transforms.json')
    params = tg.train(dataset, num_init=4000, iters=iters, lr=1.5e-2,
                      device=device, log_every=max(1, iters // 4))
    ply = tg.export_ply(out_dir / 'recon.ply', params['means'],
                        params['scales_log'], params['quats'],
                        params['opacities_logit'], params['colors_rgb'])
    hist = params['loss_history']
    return {
        'views': len(frames),
        'loss_first': hist[0],
        'loss_last': hist[-1],
        'ratio': hist[-1] / hist[0],
        'ply': str(ply),
        'ply_bytes': Path(ply).stat().st_size,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point; exits non-zero if the loss did not drop."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--out', default='/tmp/gsplat_selftest')
    p.add_argument('--iters', type=int, default=800)
    p.add_argument('--device', default='cuda')
    args = p.parse_args(argv)

    summary = run(args.out, iters=args.iters, device=args.device)
    print(f"views={summary['views']} loss {summary['loss_first']:.6f} -> "
          f"{summary['loss_last']:.6f} (ratio {summary['ratio']:.3f}) "
          f"ply={summary['ply_bytes']}B")
    ok = summary['ratio'] < 0.5
    print('SELFTEST PASS' if ok else 'SELFTEST FAIL (loss did not drop)')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
