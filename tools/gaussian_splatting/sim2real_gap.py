#!/usr/bin/env python3
"""Measure the sim2real gap of a LiDAR-primed 3DGS scene (gsplat, Apache-2.0).

Phase 0 of the 3DGS-as-sim2real track. The question this answers is the
prerequisite for every downstream use (open-loop data gen, closed-loop
Autoware-in-the-loop, RL): *does a model trained on real images still work on
3DGS renders, and out to what viewpoint offset does it keep working?*

The harness does two things per reference view:

1. **Reconstruction fidelity** at the training pose (offset 0): render vs the
   real image -> PSNR / SSIM, plus optional object-detector agreement (how many
   real detections survive on the render, matched by IoU). This is the literal
   sim2real gap at the recorded trajectory.

2. **Extrapolation degradation** as the camera is stepped sideways off the
   recorded pose (a lateral ego deviation, the thing a closed-loop sim must
   survive): each offset render is scored against the offset-0 render
   (appearance stability via SSIM), plus a sharpness ratio and a bright-floater
   fraction that flags the characteristic 3DGS blow-up, and -- if a detector is
   enabled -- detection retention. The offset where these fall off a cliff is
   the *valid viewpoint range* for using this scene as a simulator.

The pure helpers (pose offset math, PSNR/SSIM, box IoU matching, montage) are
numpy-only and unit tested on CPU; rendering needs CUDA + torch + gsplat and an
optional detector needs ``ultralytics``.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from lidarslam_benchmark_tools.gaussian_splatting.render_path import load_gaussian_ply, render_frames, scale_intrinsics
from lidarslam_benchmark_tools.gaussian_splatting.train_gsplat import load_transforms

_AXES = {'x': 0, 'y': 1, 'z': 2}


# --------------------------------------------------------------------------- #
# Pure helpers (no torch/CUDA/detector)
# --------------------------------------------------------------------------- #
def offset_viewmat(viewmat: np.ndarray, offset_m: float, axis: str) -> np.ndarray:
    """Translate the camera along one of its *local* axes by ``offset_m``.

    ``viewmat`` is world->camera ``[R | t]`` (so the camera centre in world is
    ``C = -R^T t``). Moving the centre by ``d_world`` gives ``t' = t - R d_world``;
    for a camera-local offset ``o`` (``+x`` = right, ``+y`` = down, ``+z`` =
    forward in the usual pinhole convention) this collapses to ``t' = t - o``,
    since ``R d_world = R R^T o = o``. Rotation is untouched, so the camera slides
    sideways while keeping its heading -- exactly a lateral ego deviation.
    """
    if axis not in _AXES:
        raise ValueError(f'axis must be one of x/y/z, got {axis!r}')
    out = np.asarray(viewmat, dtype=float).copy()
    out[_AXES[axis], 3] -= float(offset_m)
    return out


def sweep_viewmats(viewmats: Sequence[np.ndarray], offset_m: float,
                   axis: str) -> np.ndarray:
    """Apply ``offset_viewmat`` to every view; returns stacked (N, 4, 4)."""
    return np.stack([offset_viewmat(vm, offset_m, axis) for vm in viewmats])


def to_gray(rgb: np.ndarray) -> np.ndarray:
    """Rec.601 luma of an (H, W, 3) image as float64 in the input's range."""
    a = np.asarray(rgb, dtype=np.float64)
    return a[..., 0] * 0.299 + a[..., 1] * 0.587 + a[..., 2] * 0.114


def psnr(a: np.ndarray, b: np.ndarray, *, peak: float = 255.0) -> float:
    """Peak signal-to-noise ratio between two same-shape uint8/float images."""
    mse = float(np.mean((np.asarray(a, dtype=np.float64)
                         - np.asarray(b, dtype=np.float64)) ** 2))
    if mse <= 0.0:
        return float('inf')
    return float(10.0 * np.log10(peak * peak / mse))


def _box_mean(img: np.ndarray, win: int) -> np.ndarray:
    """Separable local mean with a shrinking window at the borders (exact)."""
    def along(a: np.ndarray, axis: int) -> np.ndarray:
        a = np.moveaxis(a, axis, -1)
        n = a.shape[-1]
        cs = np.concatenate([np.zeros(a.shape[:-1] + (1,)),
                             np.cumsum(a, axis=-1)], axis=-1)
        idx = np.arange(n)
        lo = np.maximum(0, idx - win // 2)
        hi = np.minimum(n, idx + win // 2 + 1)
        out = (cs[..., hi] - cs[..., lo]) / (hi - lo)
        return np.moveaxis(out, -1, axis)

    return along(along(np.asarray(img, dtype=np.float64), -1), -2)


def ssim(a: np.ndarray, b: np.ndarray, *, win: int = 7, peak: float = 255.0) -> float:
    """Mean structural similarity over a uniform window (grayscale)."""
    ga, gb = to_gray(a), to_gray(b)
    c1 = (0.01 * peak) ** 2
    c2 = (0.03 * peak) ** 2
    mu_a, mu_b = _box_mean(ga, win), _box_mean(gb, win)
    mu_a2, mu_b2, mu_ab = mu_a * mu_a, mu_b * mu_b, mu_a * mu_b
    var_a = _box_mean(ga * ga, win) - mu_a2
    var_b = _box_mean(gb * gb, win) - mu_b2
    cov = _box_mean(ga * gb, win) - mu_ab
    smap = (((2 * mu_ab + c1) * (2 * cov + c2))
            / ((mu_a2 + mu_b2 + c1) * (var_a + var_b + c2)))
    return float(smap.mean())


def sharpness(img: np.ndarray) -> float:
    """Mean squared gradient magnitude of the luma -- a focus/detail proxy."""
    g = to_gray(img)
    gy, gx = np.gradient(g)
    return float(np.mean(gx * gx + gy * gy))


def floater_fraction(img: np.ndarray, *, thresh: float = 250.0) -> float:
    """Fraction of near-white pixels -- a proxy for 3DGS blow-up under extrapolation."""
    return float(np.mean(to_gray(img) >= thresh))


def box_iou(a: Sequence[float], b: Sequence[float]) -> float:
    """Intersection-over-union of two ``[x1, y1, x2, y2]`` boxes."""
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - inter
    return float(inter / union) if union > 0.0 else 0.0


def match_detections(ref: Sequence[dict], qry: Sequence[dict], *,
                     iou_thresh: float = 0.5) -> int:
    """Greedy one-to-one count of ``qry`` boxes matching a same-class ``ref`` box.

    Each detection is ``{'cls': int, 'box': [x1, y1, x2, y2], 'conf': float}``.
    Matches are taken highest-IoU first; every ref box is used at most once.
    """
    pairs = []
    for qi, q in enumerate(qry):
        for ri, r in enumerate(ref):
            if q['cls'] != r['cls']:
                continue
            iou = box_iou(q['box'], r['box'])
            if iou >= iou_thresh:
                pairs.append((iou, qi, ri))
    pairs.sort(reverse=True)
    used_q: set = set()
    used_r: set = set()
    matched = 0
    for _, qi, ri in pairs:
        if qi in used_q or ri in used_r:
            continue
        used_q.add(qi)
        used_r.add(ri)
        matched += 1
    return matched


def montage(rows: Sequence[Sequence[np.ndarray]], *, pad: int = 4,
            bg: int = 32) -> np.ndarray:
    """Tile a grid of equal-size uint8 RGB images into one contact sheet."""
    h, w = rows[0][0].shape[:2]
    ncol = max(len(r) for r in rows)
    out_h = len(rows) * h + (len(rows) + 1) * pad
    out_w = ncol * w + (ncol + 1) * pad
    sheet = np.full((out_h, out_w, 3), bg, dtype=np.uint8)
    for ri, row in enumerate(rows):
        y = pad + ri * (h + pad)
        for ci, im in enumerate(row):
            x = pad + ci * (w + pad)
            sheet[y:y + h, x:x + w] = im
    return sheet


# --------------------------------------------------------------------------- #
# IO + detector (lazy / optional)
# --------------------------------------------------------------------------- #
def load_real_image(path: Path, width: int, height: int) -> np.ndarray:
    """Read a real image and resize it to the render size (uint8 RGB)."""
    import imageio.v2 as imageio

    img = np.asarray(imageio.imread(path))
    if img.ndim == 2:
        img = np.repeat(img[..., None], 3, axis=2)
    img = img[..., :3]
    if img.shape[1] != width or img.shape[0] != height:
        from PIL import Image

        img = np.asarray(Image.fromarray(img).resize((width, height)))
    return np.ascontiguousarray(img, dtype=np.uint8)


class Detector:
    """Thin wrapper over an ultralytics YOLO model (lazy, optional)."""

    def __init__(self, weights: str, *, conf: float = 0.25):
        from ultralytics import YOLO

        self.model = YOLO(weights)
        self.conf = conf

    def __call__(self, img: np.ndarray) -> list:
        res = self.model.predict(img[..., ::-1], conf=self.conf, verbose=False)[0]
        out = []
        for b in res.boxes:
            out.append({'cls': int(b.cls.item()),
                        'box': [float(v) for v in b.xyxy[0].tolist()],
                        'conf': float(b.conf.item())})
        return out


# --------------------------------------------------------------------------- #
# Harness
# --------------------------------------------------------------------------- #
def select_views(n_total: int, n_want: int) -> list[int]:
    """Evenly spaced reference-view indices (all of them when n_want <= 0)."""
    if n_want <= 0 or n_want >= n_total:
        return list(range(n_total))
    return [int(round(i)) for i in np.linspace(0, n_total - 1, n_want)]


def run(ply: Path, transforms: Path, *, offsets: Sequence[float], axis: str,
        scale: float, n_views: int, detector: Optional[Detector],
        out_dir: Path, device: str = 'cuda') -> dict:
    """Render the offset sweep, score every view, and write artefacts."""
    dataset = load_transforms(transforms)
    gaussians = load_gaussian_ply(ply)
    K, width, height = scale_intrinsics(dataset['K'], dataset['width'],
                                        dataset['height'], scale)
    views = select_views(len(dataset['viewmats']), n_views)
    base_vms = [dataset['viewmats'][i] for i in views]
    reals = [load_real_image(dataset['image_paths'][i], width, height) for i in views]

    # offset -> (V, H, W, 3) renders, with 0.0 first so it is the reference.
    ordered = sorted(set(offsets), key=lambda o: (abs(o), o))
    renders: dict[float, np.ndarray] = {}
    for off in ordered:
        renders[off] = render_frames(gaussians, sweep_viewmats(base_vms, off, axis),
                                     K, width, height, device=device)

    out_dir.mkdir(parents=True, exist_ok=True)
    per_view = []
    for vi, gi in enumerate(views):
        base = renders[0.0][vi]
        real = reals[vi]
        det_real = detector(real) if detector else None
        det_base = detector(base) if detector else None
        base_sharp = max(sharpness(base), 1e-9)
        rec = {'view': int(gi),
               'recon_psnr': psnr(real, base),
               'recon_ssim': ssim(real, base)}
        if det_real is not None:
            rec['real_dets'] = len(det_real)
            rec['render_dets'] = len(det_base)
            rec['recon_det_agree'] = (match_detections(det_real, det_base)
                                      / max(len(det_real), 1))
        sweep = {}
        for off in ordered:
            r = renders[off][vi]
            entry = {'ssim_vs_base': ssim(base, r),
                     'sharpness_ratio': sharpness(r) / base_sharp,
                     'floater_frac': floater_fraction(r)}
            if det_real is not None:
                det_r = detector(r)
                entry['dets'] = len(det_r)
                entry['det_retain'] = ((match_detections(det_base, det_r)
                                        / max(len(det_base), 1)) if off else 1.0)
            sweep[f'{off:+.2f}'] = entry
        rec['sweep'] = sweep
        per_view.append(rec)

        import imageio.v2 as imageio
        sheet = montage([[real] + [renders[o][vi] for o in ordered]])
        imageio.imwrite(out_dir / f'view_{gi:03d}.png', sheet)

    summary = _summarise(per_view, ordered)
    report = {'ply': str(ply), 'transforms': str(transforms), 'axis': axis,
              'scale': scale, 'render_size': [width, height],
              'offsets': ordered, 'n_views': len(views),
              'detector': bool(detector), 'per_view': per_view,
              'summary': summary}
    (out_dir / 'metrics.json').write_text(json.dumps(report, indent=2))
    return report


def _summarise(per_view: list[dict], offsets: Sequence[float]) -> dict:
    """Mean recon fidelity and per-offset degradation across all views."""
    def mean(key: str, src: list) -> float:
        vals = [s[key] for s in src if key in s and np.isfinite(s[key])]
        return float(np.mean(vals)) if vals else float('nan')

    out = {'recon_psnr': mean('recon_psnr', per_view),
           'recon_ssim': mean('recon_ssim', per_view),
           'per_offset': {}}
    if any('recon_det_agree' in v for v in per_view):
        out['recon_det_agree'] = mean('recon_det_agree', per_view)
    for off in offsets:
        key = f'{off:+.2f}'
        cells = [v['sweep'][key] for v in per_view]
        row = {'ssim_vs_base': mean('ssim_vs_base', cells),
               'sharpness_ratio': mean('sharpness_ratio', cells),
               'floater_frac': mean('floater_frac', cells)}
        if any('det_retain' in c for c in cells):
            row['det_retain'] = mean('det_retain', cells)
        out['per_offset'][key] = row
    return out


def parse_offsets(text: str) -> list[float]:
    """Parse a comma-separated offset list (metres); 0 is always included."""
    vals = {0.0}
    for tok in text.split(','):
        tok = tok.strip()
        if tok:
            vals.add(float(tok))
    return sorted(vals, key=lambda o: (abs(o), o))


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--ply', required=True, help='trained INRIA 3DGS .ply')
    p.add_argument('--transforms', required=True,
                   help='transforms.json the model was trained from')
    p.add_argument('--out', required=True, help='output directory')
    p.add_argument('--offsets', default='-1.0,-0.5,-0.25,0.25,0.5,1.0',
                   help='comma-separated lateral offsets in metres (0 implied)')
    p.add_argument('--axis', default='x', choices=('x', 'y', 'z'),
                   help='camera-local offset axis (x=right, default)')
    p.add_argument('--scale', type=float, default=0.25,
                   help='render-resolution scale vs the training images')
    p.add_argument('--views', type=int, default=0,
                   help='evenly spaced reference views to score (0 = all)')
    p.add_argument('--detector', default='',
                   help='ultralytics weights for the sim2real detection gap '
                        "(e.g. 'yolov8n.pt'); empty disables it")
    p.add_argument('--det-conf', type=float, default=0.25,
                   help='detector confidence threshold')
    p.add_argument('--device', default='cuda')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    det = Detector(args.detector, conf=args.det_conf) if args.detector else None
    report = run(Path(args.ply), Path(args.transforms),
                 offsets=parse_offsets(args.offsets), axis=args.axis,
                 scale=args.scale, n_views=args.views, detector=det,
                 out_dir=Path(args.out), device=args.device)
    s = report['summary']
    print(f"recon: PSNR {s['recon_psnr']:.2f} dB, SSIM {s['recon_ssim']:.3f}"
          + (f", det-agree {s['recon_det_agree']:.2f}"
             if 'recon_det_agree' in s else ''))
    for off, row in s['per_offset'].items():
        line = (f"  off {off} m: ssim_vs_base {row['ssim_vs_base']:.3f}, "
                f"sharp {row['sharpness_ratio']:.2f}, "
                f"floater {row['floater_frac']:.3f}")
        if 'det_retain' in row:
            line += f", det-retain {row['det_retain']:.2f}"
        print(line)
    print(f'wrote {args.out}/metrics.json + per-view montages')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
