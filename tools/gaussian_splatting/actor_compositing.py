#!/usr/bin/env python3
"""Composite dynamic actors into a 3DGS scene with correct occlusion (gsplat, Apache-2.0).

Phase 3 of the 3DGS-as-sim2real track (the actor-insertion half; RL is out of
scope here). A static 3DGS scene has no moving traffic, and Phase 0 found that
the indoor/test-track scenes on hand contain no COCO-class objects at all, so a
real-image detector never fires on the renders. This tool inserts a moving
actor into the scene so that gap can finally be exercised:

* **box actor** -- a synthetic solid of Gaussians, rasterised from the scene
  camera and composited by a per-pixel depth test (the actor is only drawn where
  it is nearer than the scene), giving geometrically correct occlusion. This is
  the photoreal-geometry demo of a moving object.
* **sprite actor** -- a real-photo RGBA cutout billboarded into the scene at a
  chosen camera-local position, scaled by its depth and occluded by the scene
  depth. Because the sprite is a real object (e.g. a person cut from a stock
  image), a COCO detector *can* fire on it, so this drives the detection-gap
  measurement: how reliably is a real object detected once embedded in a 3DGS
  render, and how does scene occlusion degrade it.

Either way a per-frame ground-truth 2D box is exported (from the projected actor
extent), so a detector's output can be scored against a known label.

The pure helpers (actor construction, rigid placement, Gaussian transforms,
projection, depth-tested compositing, nearest-neighbour sprite resize) are
numpy-only and unit tested on CPU; rasterising the scene/box needs CUDA + torch
+ gsplat and the optional detector needs ultralytics.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from lidarslam_benchmark_tools.gaussian_splatting.render_path import load_gaussian_ply, matrix_to_quat_xyzw, scale_intrinsics
from lidarslam_benchmark_tools.gaussian_splatting.train_gsplat import load_transforms


# --------------------------------------------------------------------------- #
# Pure helpers (no torch/CUDA)
# --------------------------------------------------------------------------- #
def logit(p: float) -> float:
    """Inverse sigmoid, clamped away from the asymptotes."""
    p = min(max(float(p), 1e-6), 1.0 - 1e-6)
    return float(np.log(p / (1.0 - p)))


def make_box_actor(size_xyz: Sequence[float], *, spacing: float = 0.1,
                   color_rgb: Sequence[float] = (0.85, 0.1, 0.1),
                   opacity: float = 0.99) -> dict:
    """Build a solid box of Gaussians centred on x/y, resting on the ground (z in [0, h]).

    Returns a gaussians dict in the same layout as ``load_gaussian_ply`` (band-0
    colour only, identity ``wxyz`` quats), deterministic for a given size and
    spacing so the demo and tests are reproducible.
    """
    sx, sy, sz = (float(v) for v in size_xyz)
    if min(sx, sy, sz) <= 0.0 or spacing <= 0.0:
        raise ValueError('box dimensions and spacing must be positive')
    xs = np.arange(-sx / 2.0, sx / 2.0 + 1e-9, spacing)
    ys = np.arange(-sy / 2.0, sy / 2.0 + 1e-9, spacing)
    zs = np.arange(0.0, sz + 1e-9, spacing)
    gx, gy, gz = np.meshgrid(xs, ys, zs, indexing='ij')
    means = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
    n = means.shape[0]
    quats = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))  # wxyz identity
    scales_log = np.full((n, 3), np.log(spacing * 0.75))
    return {
        'means': means.astype(np.float64),
        'scales_log': scales_log,
        'quats': quats.astype(np.float64),
        'opacities_logit': np.full(n, logit(opacity)),
        'colors_rgb': np.tile(np.asarray(color_rgb, dtype=float), (n, 1)),
        'sh_rest': None,
    }


def crop_gaussians(g: dict, center_xy: Sequence[float], half_extent_xy: float,
                   z_range: Sequence[float]) -> dict:
    """Subset a gaussians dict to an axis-aligned box around ``center_xy``.

    Keeps Gaussians whose mean lies within ``half_extent_xy`` of ``center_xy`` in
    x and y and within ``z_range`` (inclusive). Every per-Gaussian array is
    indexed the same way (including ``sh_rest`` when present), so the result is a
    valid standalone model -- used to mint a photoreal volumetric actor by
    cutting a compact object out of a trained scene.
    """
    means = np.asarray(g['means'], dtype=float)
    cx, cy = float(center_xy[0]), float(center_xy[1])
    zlo, zhi = float(z_range[0]), float(z_range[1])
    keep = ((np.abs(means[:, 0] - cx) <= half_extent_xy)
            & (np.abs(means[:, 1] - cy) <= half_extent_xy)
            & (means[:, 2] >= zlo) & (means[:, 2] <= zhi))
    if not np.any(keep):
        raise ValueError('crop box contains no Gaussians')
    out = {k: (np.asarray(v)[keep] if k != 'sh_rest' else
               (None if v is None else np.asarray(v)[keep]))
           for k, v in g.items()}
    return out


_UP_TO_Z = {
    'z': np.eye(3),
    'y': np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]),
    'x': np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]),
}


def reorient_up_to_z(g: dict, up_axis: str) -> dict:
    """Rotate a gaussians dict so the given world up axis maps to +z.

    Assets trained in a different convention (e.g. the y-up Tanks&Temples
    scenes) must be reoriented before they sit upright under the z-up actor
    convention (``recenter_gaussians`` grounds on min-z, ``rigid_from_pos_yaw``
    yaws about z).
    """
    if up_axis not in _UP_TO_Z:
        raise ValueError(f'up_axis must be one of x/y/z, got {up_axis!r}')
    rot = _UP_TO_Z[up_axis]
    if up_axis == 'z':
        return dict(g)
    t = np.eye(4)
    t[:3, :3] = rot
    return transform_gaussians(g, t)


def recenter_gaussians(g: dict) -> dict:
    """Translate a gaussians dict so its x/y centroid is 0 and it rests on z=0.

    A pure translation (orientations untouched), so an arbitrary actor model can
    be placed with the same ``rigid_from_pos_yaw`` convention as the box actor.
    """
    means = np.asarray(g['means'], dtype=float)
    shift = np.array([means[:, 0].mean(), means[:, 1].mean(), means[:, 2].min()])
    out = dict(g)
    out['means'] = means - shift
    return out


def _rotate_quats_wxyz(quats_wxyz: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """Apply a world rotation ``rot`` to a batch of ``wxyz`` Gaussian quaternions.

    Vectorised: the orientation update ``rot @ R_g`` corresponds to the Hamilton
    product ``q_rot (x) q_g``, computed for all Gaussians at once (a Python loop
    is far too slow for the ~10^6 Gaussians of a real scene crop).
    """
    q = np.asarray(quats_wxyz, dtype=float)
    rx = matrix_to_quat_xyzw(rot)               # xyzw quaternion of rot
    rw, rxx, ryy, rzz = rx[3], rx[0], rx[1], rx[2]
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    out = np.empty_like(q)
    out[:, 0] = rw * w - rxx * x - ryy * y - rzz * z
    out[:, 1] = rw * x + rxx * w + ryy * z - rzz * y
    out[:, 2] = rw * y - rxx * z + ryy * w + rzz * x
    out[:, 3] = rw * z + rxx * y - ryy * x + rzz * w
    return out


def transform_gaussians(g: dict, transform: np.ndarray) -> dict:
    """Rigidly move a gaussians dict by a 4x4 ``transform`` (means + orientations)."""
    t = np.asarray(transform, dtype=float)
    rot, trans = t[:3, :3], t[:3, 3]
    out = dict(g)
    out['means'] = (np.asarray(g['means'], dtype=float) @ rot.T) + trans
    out['quats'] = _rotate_quats_wxyz(g['quats'], rot)
    return out


def rigid_from_pos_yaw(pos: Sequence[float], yaw: float,
                       up_axis: int = 2) -> np.ndarray:
    """4x4 transform: translate to ``pos`` and rotate ``yaw`` rad about ``up_axis``."""
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.eye(3)
    a, b = [i for i in range(3) if i != up_axis]
    rot[a, a], rot[a, b] = c, -s
    rot[b, a], rot[b, b] = s, c
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = np.asarray(pos, dtype=float)
    return t


def actor_world_poses(c2w: np.ndarray, laterals: Sequence[float], *,
                      distance: float, drop: float) -> list[np.ndarray]:
    """Actor world positions in front of a camera, swept across its local x.

    ``c2w`` is an OpenCV camera-to-world pose; the actor sits ``distance`` m
    ahead (+z), ``drop`` m below (+y down), and at each lateral offset (+x
    right). Returns one world position per lateral -- the actor crossing the
    field of view as it moves.
    """
    c2w = np.asarray(c2w, dtype=float)
    rot, eye = c2w[:3, :3], c2w[:3, 3]
    out = []
    for lat in laterals:
        local = np.array([float(lat), float(drop), float(distance)])
        out.append(eye + rot @ local)
    return out


def project_points(points_world: np.ndarray, viewmat: np.ndarray,
                   K: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Project world points to pixels; returns ``(uv (N, 2), z (N,))`` in camera z."""
    pts = np.asarray(points_world, dtype=float)
    cam = (np.asarray(viewmat, dtype=float)
           @ np.concatenate([pts, np.ones((len(pts), 1))], axis=1).T).T[:, :3]
    z = cam[:, 2]
    safe = np.where(np.abs(z) < 1e-9, 1e-9, z)
    uv = (np.asarray(K, dtype=float) @ (cam / safe[:, None]).T).T[:, :2]
    return uv, z


def points_to_bbox(uv: np.ndarray, z: np.ndarray, width: int,
                   height: int) -> Optional[list[int]]:
    """Axis-aligned ``[x1, y1, x2, y2]`` of the in-front points, clipped to image.

    Returns ``None`` when nothing projects in front of the camera or the extent
    falls entirely outside the frame.
    """
    front = np.asarray(z) > 1e-6
    if not np.any(front):
        return None
    pix = np.asarray(uv)[front]
    x1, y1 = pix.min(axis=0)
    x2, y2 = pix.max(axis=0)
    x1, x2 = max(0, int(np.floor(x1))), min(width, int(np.ceil(x2)))
    y1, y2 = max(0, int(np.floor(y1))), min(height, int(np.ceil(y2)))
    if x2 <= x1 or y2 <= y1:
        return None
    return [x1, y1, x2, y2]


def composite_depth(scene_rgb: np.ndarray, scene_depth: np.ndarray,
                    actor_rgb: np.ndarray, actor_depth: np.ndarray,
                    actor_alpha: np.ndarray, *, alpha_thresh: float = 0.5
                    ) -> tuple[np.ndarray, np.ndarray]:
    """Z-tested composite of an actor render over a scene render.

    The actor wins a pixel where its coverage exceeds ``alpha_thresh`` and it is
    nearer than the scene (or the scene has no return, depth 0). Returns the
    composited uint8 RGB and the boolean actor mask.
    """
    sd = np.asarray(scene_depth, dtype=float)
    ad = np.asarray(actor_depth, dtype=float)
    nearer = (ad < sd) | (sd <= 0.0)
    mask = (np.asarray(actor_alpha) >= alpha_thresh) & (ad > 0.0) & nearer
    out = np.asarray(scene_rgb).copy()
    out[mask] = np.asarray(actor_rgb)[mask]
    return out, mask


def resize_nearest(img: np.ndarray, out_w: int, out_h: int) -> np.ndarray:
    """Nearest-neighbour resize of an ``(H, W, C)`` image (pure numpy, testable)."""
    if out_w <= 0 or out_h <= 0:
        raise ValueError('output size must be positive')
    h, w = img.shape[:2]
    ys = np.minimum((np.arange(out_h) * h / out_h).astype(int), h - 1)
    xs = np.minimum((np.arange(out_w) * w / out_w).astype(int), w - 1)
    return img[ys][:, xs]


def paste_sprite(scene_rgb: np.ndarray, scene_depth: np.ndarray,
                 sprite_rgba: np.ndarray, center_uv: Sequence[float],
                 height_px: int, actor_depth: float, *, alpha_thresh: int = 128
                 ) -> tuple[np.ndarray, Optional[list[int]]]:
    """Billboard an RGBA sprite into the scene at ``center_uv``, scaled and z-tested.

    The sprite is resized to ``height_px`` tall (aspect preserved), centred at
    ``center_uv``, and blended where its alpha exceeds ``alpha_thresh`` and the
    scene is farther than ``actor_depth`` (or empty). Returns the composited
    uint8 RGB and the actor's clipped ``[x1, y1, x2, y2]`` (or ``None`` if the
    sprite lands fully off-frame).
    """
    sh, sw = sprite_rgba.shape[:2]
    out_h = max(1, int(height_px))
    out_w = max(1, int(round(sw * out_h / sh)))
    spr = resize_nearest(sprite_rgba, out_w, out_h)
    H, W = scene_rgb.shape[:2]
    cx, cy = float(center_uv[0]), float(center_uv[1])
    x0, y0 = int(round(cx - out_w / 2.0)), int(round(cy - out_h / 2.0))
    sx0, sy0 = max(0, -x0), max(0, -y0)
    dx0, dy0 = max(0, x0), max(0, y0)
    dx1, dy1 = min(W, x0 + out_w), min(H, y0 + out_h)
    out = np.asarray(scene_rgb).copy()
    if dx1 <= dx0 or dy1 <= dy0:
        return out, None
    region = spr[sy0:sy0 + (dy1 - dy0), sx0:sx0 + (dx1 - dx0)]
    sd = np.asarray(scene_depth, dtype=float)[dy0:dy1, dx0:dx1]
    cover = (region[..., 3] >= alpha_thresh) & ((sd > actor_depth) | (sd <= 0.0))
    dst = out[dy0:dy1, dx0:dx1]
    dst[cover] = region[..., :3][cover]
    out[dy0:dy1, dx0:dx1] = dst
    if not np.any(cover):
        return out, None
    ys, xs = np.where(cover)
    return out, [dx0 + int(xs.min()), dy0 + int(ys.min()),
                 dx0 + int(xs.max()) + 1, dy0 + int(ys.max()) + 1]


def linspace_sym(half_range: float, count: int) -> list[float]:
    """``count`` evenly spaced values across ``[-half_range, +half_range]``."""
    if count < 1:
        raise ValueError('count must be positive')
    if count == 1:
        return [0.0]
    return [float(v) for v in np.linspace(-half_range, half_range, count)]


# --------------------------------------------------------------------------- #
# Rasterisation (torch + gsplat; imported lazily)
# --------------------------------------------------------------------------- #
def rasterize_rgbda(gaussians: dict, viewmat: np.ndarray, K: np.ndarray,
                    width: int, height: int, *, device: str = 'cuda'
                    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Rasterise one view returning ``(rgb uint8, depth float32, alpha float32)``."""
    import torch
    import torch.nn.functional as F

    from gsplat import rasterization
    from lidarslam_benchmark_tools.gaussian_splatting.render_path import infer_sh_degree
    from lidarslam_benchmark_tools.gaussian_splatting.train_gsplat import SH_C0

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
    vmt = torch.tensor(np.asarray(viewmat, dtype=np.float32), device=dev)[None]
    with torch.no_grad():
        out, alpha, _ = rasterization(means, quats, scales, opac, colors, vmt,
                                      kmat, width, height, sh_degree=sh_degree,
                                      packed=False, render_mode='RGB+ED')
        rgb = (out[0, ..., :3].clamp(0.0, 1.0) * 255.0).to(torch.uint8).cpu().numpy()
        depth = out[0, ..., 3].cpu().numpy()
        acc = alpha[0, ..., 0].cpu().numpy()
    return rgb, depth, acc


# --------------------------------------------------------------------------- #
# Driver
# --------------------------------------------------------------------------- #
def load_sprite(path: Path) -> np.ndarray:
    """Load an RGBA sprite (adds a fully opaque alpha channel if missing)."""
    import imageio.v2 as imageio

    img = np.asarray(imageio.imread(path))
    if img.ndim == 2:
        img = np.repeat(img[..., None], 3, axis=2)
    if img.shape[2] == 3:
        a = np.full(img.shape[:2] + (1,), 255, dtype=img.dtype)
        img = np.concatenate([img, a], axis=2)
    return np.ascontiguousarray(img[..., :4], dtype=np.uint8)


def run(ply: Path, transforms: Path, out_dir: Path, *, view: int, frames: int,
        mode: str, box_size: Sequence[float], sprite: Optional[Path],
        actor_ply: Optional[Path], yaw: float, distance: float, lateral: float,
        drop: float, sprite_height_m: float, scale: float, detector,
        out_fps: int, device: str = 'cuda') -> dict:
    """Render the actor crossing one scene view and write video + per-frame labels."""
    import imageio.v2 as imageio

    from lidarslam_benchmark_tools.gaussian_splatting.sim2real_gap import box_iou

    dataset = load_transforms(transforms)
    scene = load_gaussian_ply(ply)
    K, width, height = scale_intrinsics(dataset['K'], dataset['width'],
                                        dataset['height'], scale)
    vm = dataset['viewmats'][view]
    c2w = np.linalg.inv(vm)
    laterals = linspace_sym(lateral, frames)
    poses = actor_world_poses(c2w, laterals, distance=distance, drop=drop)

    scene_rgb, scene_depth, _ = rasterize_rgbda(scene, vm, K, width, height,
                                                device=device)
    if mode == 'box':
        actor_g = make_box_actor(box_size)
    elif mode == 'ply':
        actor_g = recenter_gaussians(load_gaussian_ply(actor_ply))
    else:
        actor_g = None
    spr = load_sprite(sprite) if mode == 'sprite' else None
    # The heading is fixed across the sweep, so rotate the actor once (a Python
    # quat loop over ~10^6 Gaussians per frame would be hopeless); each frame
    # then only translates the means -- cheap even for a full scene crop.
    actor_rot = (transform_gaussians(actor_g, rigid_from_pos_yaw([0.0, 0.0, 0.0],
                 yaw)) if actor_g is not None else None)

    out_dir.mkdir(parents=True, exist_ok=True)
    composited, labels, per_frame = [], [], []
    for i, pos in enumerate(poses):
        if actor_rot is not None:  # volumetric actor (box or arbitrary 3DGS ply)
            placed = dict(actor_rot)
            placed['means'] = actor_rot['means'] + np.asarray(pos, dtype=float)
            arb, adp, aac = rasterize_rgbda(placed, vm, K, width, height,
                                            device=device)
            frame, mask = composite_depth(scene_rgb, scene_depth, arb, adp, aac)
            uv, z = project_points(placed['means'], vm, K)
            gt = points_to_bbox(uv, z, width, height)
        else:
            uv, z = project_points(pos[None], vm, K)
            depth_c = float(z[0])
            height_px = int(round(K[1, 1] * sprite_height_m / max(depth_c, 1e-3)))
            frame, gt = paste_sprite(scene_rgb, scene_depth, spr, uv[0],
                                     height_px, depth_c)
        composited.append(frame)
        labels.append(gt)
        rec = {'frame': i, 'gt_bbox': gt}
        if detector is not None and gt is not None:
            dets = detector(frame)
            best = max((box_iou(gt, d['box']) for d in dets), default=0.0)
            hit = best >= 0.5
            rec.update({'n_dets': len(dets), 'best_iou': round(float(best), 3),
                        'hit': bool(hit)})
        per_frame.append(rec)

    mp4 = out_dir / 'actor.mp4'
    with imageio.get_writer(mp4, fps=out_fps, codec='libx264', quality=8,
                            macro_block_size=2) as wr:
        for f in composited:
            wr.append_data(f)

    report = {'ply': str(ply), 'mode': mode, 'view': view, 'frames': frames,
              'render_size': [width, height], 'per_frame': per_frame}
    if detector is not None:
        scored = [r for r in per_frame if 'hit' in r]
        report['detection_recall'] = (sum(r['hit'] for r in scored) / len(scored)
                                      if scored else 0.0)
        report['mean_best_iou'] = (float(np.mean([r['best_iou'] for r in scored]))
                                   if scored else 0.0)
    (out_dir / 'actor_labels.json').write_text(json.dumps(report, indent=2))
    report['mp4'] = str(mp4)
    return report


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--ply', required=True, help='trained INRIA 3DGS scene .ply')
    p.add_argument('--transforms', required=True,
                   help='transforms.json the scene was trained from')
    p.add_argument('--out', required=True, help='output directory')
    p.add_argument('--view', type=int, default=0,
                   help='scene camera index to stage the actor in front of')
    p.add_argument('--frames', type=int, default=48,
                   help='frames as the actor crosses the view')
    p.add_argument('--mode', default='box', choices=('box', 'sprite', 'ply'))
    p.add_argument('--box-size', default='0.6,0.6,1.7',
                   help='box actor w,d,h in metres (default a standing person)')
    p.add_argument('--sprite', default='',
                   help='RGBA cutout for --mode sprite (real object)')
    p.add_argument('--actor-ply', default='',
                   help='trained 3DGS .ply used as a volumetric actor for '
                        '--mode ply (e.g. an object cut from a scene with '
                        'crop_actor_ply.py); recentred onto the ground')
    p.add_argument('--yaw', type=float, default=0.0,
                   help='actor heading in radians (volumetric actor modes)')
    p.add_argument('--distance', type=float, default=4.0,
                   help='actor distance ahead of the camera (m)')
    p.add_argument('--lateral', type=float, default=2.0,
                   help='half-width of the lateral sweep across the view (m)')
    p.add_argument('--drop', type=float, default=0.8,
                   help='actor drop below the camera (m, +y down)')
    p.add_argument('--sprite-height-m', type=float, default=1.7,
                   help='real-world height the sprite represents (m)')
    p.add_argument('--scale', type=float, default=0.5,
                   help='render-resolution scale vs the training images')
    p.add_argument('--detector', default='',
                   help="ultralytics weights to score detection (e.g. 'yolov8n.pt')")
    p.add_argument('--det-conf', type=float, default=0.25)
    p.add_argument('--fps', type=int, default=12)
    p.add_argument('--device', default='cuda')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    if args.mode == 'sprite' and not args.sprite:
        raise SystemExit('--mode sprite requires --sprite')
    if args.mode == 'ply' and not args.actor_ply:
        raise SystemExit('--mode ply requires --actor-ply')
    detector = None
    if args.detector:
        from lidarslam_benchmark_tools.gaussian_splatting.sim2real_gap import Detector

        detector = Detector(args.detector, conf=args.det_conf)
    box_size = [float(v) for v in args.box_size.split(',')]
    report = run(Path(args.ply), Path(args.transforms), Path(args.out),
                 view=args.view, frames=args.frames, mode=args.mode,
                 box_size=box_size,
                 sprite=Path(args.sprite) if args.sprite else None,
                 actor_ply=Path(args.actor_ply) if args.actor_ply else None,
                 yaw=args.yaw, distance=args.distance, lateral=args.lateral,
                 drop=args.drop, sprite_height_m=args.sprite_height_m,
                 scale=args.scale, detector=detector, out_fps=args.fps,
                 device=args.device)
    print(f"wrote {report['mp4']} ({report['frames']} frames, mode {report['mode']})")
    labelled = sum(1 for r in report['per_frame'] if r['gt_bbox'] is not None)
    print(f"  {labelled}/{report['frames']} frames have a visible actor label")
    if 'detection_recall' in report:
        print(f"  detection recall {report['detection_recall']:.2f}, "
              f"mean best IoU {report['mean_best_iou']:.2f}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
