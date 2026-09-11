#!/usr/bin/env python3
"""Train a 3D Gaussian Splatting model from posed images (gsplat, Apache-2.0).

Consumes a Nerfstudio ``transforms.json`` (as produced by
``extract_posed_images.py``) plus the referenced images, optimises a set of
3D Gaussians with the gsplat CUDA rasteriser, and exports a standard INRIA
3DGS ``.ply`` that SuperSplat / other viewers can open.

This is the GPU half of the pipeline in
``docs/research/3dgs-postprocess-map-design.md``. It is opt-in and requires a
CUDA device + torch + gsplat; importing the pure helpers
(``load_transforms``, ``looks_at_poses``) does not.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

import lidarslam_benchmark_tools.gaussian_splatting.posed_images as pi

# SH band-0 constant: f_dc = (rgb - 0.5) / C0 for the INRIA .ply layout.
SH_C0 = 0.28209479177387814


# --------------------------------------------------------------------------- #
# Pure helpers (no torch/CUDA)
# --------------------------------------------------------------------------- #
def load_transforms(path: str | Path) -> dict:
    """Load a transforms.json into intrinsics + per-frame OpenCV w2c poses.

    Returns a dict with ``K`` (3x3), ``width``, ``height``, ``image_paths``
    (resolved), ``viewmats`` (list of 4x4 world->camera, OpenCV/gsplat
    convention), and ``groups`` (per-frame int index from the optional ``bag``
    frame key; all zeros when absent — used by per-group pose refinement when
    merging multi-session captures). Optional timestamps, dynamic-mask paths,
    and root spatiotemporal-calibration metadata are retained for geometry-aware
    RGB fusion. The stored ``transform_matrix`` is OpenGL c2w, so we undo the
    ``ROS_OPTICAL_TO_OPENGL`` flip and invert to get the OpenCV w2c gsplat wants.
    """
    path = Path(path)
    doc = json.loads(path.read_text())
    fx, fy = doc['fl_x'], doc['fl_y']
    cx, cy = doc['cx'], doc['cy']
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    image_paths: list[Path] = []
    viewmats: list[np.ndarray] = []
    groups: list[int] = []
    timestamps: list[float] = []
    dynamic_mask_paths: list[Optional[Path]] = []
    group_ids: dict = {}
    for fr in doc['frames']:
        c2w_gl = np.asarray(fr['transform_matrix'], dtype=float)
        c2w_cv = c2w_gl @ pi.ROS_OPTICAL_TO_OPENGL  # OpenGL -> OpenCV camera
        viewmats.append(np.linalg.inv(c2w_cv))
        image_paths.append((path.parent / fr['file_path']).resolve())
        groups.append(group_ids.setdefault(fr.get('bag', ''), len(group_ids)))
        timestamps.append(float(fr.get('timestamp', np.nan)))
        mask_path = fr.get('dynamic_mask_path')
        dynamic_mask_paths.append(
            (path.parent / mask_path).resolve() if mask_path else None)
    return {
        'K': K,
        'width': int(doc['w']),
        'height': int(doc['h']),
        'image_paths': image_paths,
        'viewmats': viewmats,
        'groups': groups,
        'timestamps': np.asarray(timestamps, dtype=np.float64),
        'dynamic_mask_paths': dynamic_mask_paths,
        'spatiotemporal_calibration':
            doc.get('spatiotemporal_calibration'),
    }


def looks_at_poses(radius: float, count: int, *, height: float = 0.0) -> list[np.ndarray]:
    """Generate ``count`` OpenCV camera-to-world poses on a ring looking at origin.

    Used by the synthetic self-test; deterministic (no RNG) so it is unit
    testable. Cameras sit on a circle of ``radius`` at ``height`` and point at
    the origin with +z forward (OpenCV optical convention).
    """
    poses: list[np.ndarray] = []
    for i in range(count):
        ang = 2.0 * np.pi * i / count
        eye = np.array([radius * np.cos(ang), radius * np.sin(ang), height])
        forward = -eye / np.linalg.norm(eye)              # +z points at origin
        up_hint = np.array([0.0, 0.0, 1.0])
        right = np.cross(up_hint, forward)
        right /= np.linalg.norm(right)
        down = np.cross(forward, right)                   # +y is down in OpenCV
        c2w = np.eye(4)
        c2w[:3, 0] = right
        c2w[:3, 1] = down
        c2w[:3, 2] = forward
        c2w[:3, 3] = eye
        poses.append(c2w)
    return poses


# --------------------------------------------------------------------------- #
# Gaussian parameter container + INRIA .ply export (numpy only)
# --------------------------------------------------------------------------- #
def axis_angle_to_matrix(omega: np.ndarray) -> np.ndarray:
    """Rodrigues: a 3-vector axis-angle (rad) to a 3x3 rotation matrix."""
    omega = np.asarray(omega, dtype=float)
    theta = float(np.linalg.norm(omega))
    if theta < 1e-12:
        return np.eye(3)
    k = omega / theta
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * kx + (1 - np.cos(theta)) * (kx @ kx)


def export_ply(path: str | Path, means: np.ndarray, scales_log: np.ndarray,
               quats: np.ndarray, opacities_logit: np.ndarray,
               colors_rgb: np.ndarray, sh_rest: Optional[np.ndarray] = None) -> Path:
    """Write a standard INRIA 3DGS binary ``.ply``.

    ``scales_log`` and ``opacities_logit`` are stored raw (log / logit), as the
    3DGS format expects; ``colors_rgb`` (0..1) become ``f_dc`` via SH band 0.
    ``sh_rest`` (N, K-1, 3), if given, writes the higher SH bands as ``f_rest_*``
    in INRIA channel-major order (all coeffs of R, then G, then B).
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    n = means.shape[0]
    f_dc = (colors_rgb - 0.5) / SH_C0
    fields = [
        ('x', means[:, 0]), ('y', means[:, 1]), ('z', means[:, 2]),
        ('nx', np.zeros(n)), ('ny', np.zeros(n)), ('nz', np.zeros(n)),
        ('f_dc_0', f_dc[:, 0]), ('f_dc_1', f_dc[:, 1]), ('f_dc_2', f_dc[:, 2]),
    ]
    if sh_rest is not None and sh_rest.shape[1] > 0:
        k_rest = sh_rest.shape[1]
        for c in range(3):
            for k in range(k_rest):
                fields.append((f'f_rest_{c * k_rest + k}', sh_rest[:, k, c]))
    fields += [
        ('opacity', opacities_logit),
        ('scale_0', scales_log[:, 0]), ('scale_1', scales_log[:, 1]),
        ('scale_2', scales_log[:, 2]),
        ('rot_0', quats[:, 0]), ('rot_1', quats[:, 1]),
        ('rot_2', quats[:, 2]), ('rot_3', quats[:, 3]),
    ]
    header = 'ply\nformat binary_little_endian 1.0\n'
    header += f'element vertex {n}\n'
    header += ''.join(f'property float {name}\n' for name, _ in fields)
    header += 'end_header\n'
    arr = np.empty((n, len(fields)), dtype=np.float32)
    for i, (_, col) in enumerate(fields):
        arr[:, i] = col
    with open(path, 'wb') as fh:
        fh.write(header.encode('ascii'))
        fh.write(arr.tobytes())
    return path


# --------------------------------------------------------------------------- #
# Training (torch + gsplat; imported lazily)
# --------------------------------------------------------------------------- #
def make_ssim(device, *, channels: int = 3, window_size: int = 11,
              sigma: float = 1.5):
    """Build a differentiable SSIM(a, b) for ``(H,W,C)`` tensors in ``0..1``.

    Returns a closure computing the mean SSIM with a Gaussian window, matching
    the INRIA 3DGS structural term. Used both as the ``1 - SSIM`` training loss
    (``ssim_lambda``) and for end-of-run evaluation. torch is imported lazily so
    importing this module stays GPU-free.
    """
    import torch
    import torch.nn.functional as F

    coords = torch.arange(window_size, dtype=torch.float32, device=device)
    coords = coords - (window_size - 1) / 2.0
    g = torch.exp(-(coords ** 2) / (2.0 * sigma ** 2))
    g = g / g.sum()
    win = (g[:, None] * g[None, :])[None, None]
    win = win.expand(channels, 1, window_size, window_size).contiguous()
    pad = window_size // 2
    c1, c2 = 0.01 ** 2, 0.03 ** 2

    def ssim(a, b):
        x = a.permute(2, 0, 1)[None]
        y = b.permute(2, 0, 1)[None]
        mu_x = F.conv2d(x, win, padding=pad, groups=channels)
        mu_y = F.conv2d(y, win, padding=pad, groups=channels)
        mu_x2, mu_y2, mu_xy = mu_x * mu_x, mu_y * mu_y, mu_x * mu_y
        sig_x2 = F.conv2d(x * x, win, padding=pad, groups=channels) - mu_x2
        sig_y2 = F.conv2d(y * y, win, padding=pad, groups=channels) - mu_y2
        sig_xy = F.conv2d(x * y, win, padding=pad, groups=channels) - mu_xy
        ssim_map = ((2 * mu_xy + c1) * (2 * sig_xy + c2)) / (
            (mu_x2 + mu_y2 + c1) * (sig_x2 + sig_y2 + c2))
        return ssim_map.mean()

    return ssim


def _photometric_loss(render, gt, ssim_fn, ssim_lambda):
    """INRIA-style data term: pure MSE when ``ssim_lambda<=0``, else L1+D-SSIM.

    Returns ``(loss, mse)`` -- ``mse`` is always reported for PSNR continuity
    with earlier runs, regardless of which term is optimised.
    """
    import torch.nn.functional as F

    mse = F.mse_loss(render, gt)
    if ssim_fn is None or ssim_lambda <= 0.0:
        return mse, mse
    l1 = F.l1_loss(render, gt)
    loss = (1.0 - ssim_lambda) * l1 + ssim_lambda * (1.0 - ssim_fn(render, gt))
    return loss, mse


def _eval_views(render_fn, gts, ssim_fn) -> dict:
    """Mean PSNR (dB) and SSIM over all views, computed under no_grad."""
    import torch

    mses, ssims = [], []
    with torch.no_grad():
        for i in range(gts.shape[0]):
            r = render_fn(i)
            mses.append(float(torch.mean((r - gts[i]) ** 2)))
            ssims.append(float(ssim_fn(r, gts[i])))
    mse = sum(mses) / max(len(mses), 1)
    psnr = float('inf') if mse <= 0 else -10.0 * float(np.log10(mse))
    return {'psnr': psnr, 'ssim': sum(ssims) / max(len(ssims), 1), 'mse': mse}


def train(dataset: dict, *, init_points: Optional[np.ndarray] = None,
          init_colors: Optional[np.ndarray] = None,
          num_init: int = 20000, iters: int = 2000, lr: float = 1e-2,
          device: str = 'cuda', log_every: int = 200,
          ssim_lambda: float = 0.0) -> dict:
    """Optimise Gaussians to reconstruct the dataset images. Returns numpy params.

    ``init_points`` (N,3, e.g. a LiDAR map) seeds the means; otherwise points
    are sampled in the cameras' bounding sphere. ``init_colors`` (N,3 in 0..1)
    optionally seeds the per-Gaussian colour. The result dict holds ``means``,
    ``scales_log``, ``quats``, ``opacities_logit``, ``colors_rgb``, and the
    ``loss_history``.
    """
    import torch
    import torch.nn.functional as F
    import imageio.v3 as iio
    from gsplat import rasterization

    dev = torch.device(device)
    K = torch.tensor(dataset['K'], dtype=torch.float32, device=dev)[None]
    W, H = dataset['width'], dataset['height']
    viewmats = torch.tensor(np.stack(dataset['viewmats']), dtype=torch.float32, device=dev)
    gts = []
    for p in dataset['image_paths']:
        img = np.asarray(iio.imread(p), dtype=np.float32) / 255.0
        if img.ndim == 2:
            img = np.stack([img] * 3, axis=-1)
        gts.append(torch.tensor(img[..., :3], device=dev))
    gts = torch.stack(gts)  # (C, H, W, 3)

    # Seed Gaussians via the shared initialiser (same logic as train_densify),
    # so the fixed-count and densify paths can never drift apart.
    seed, extent = _seed_params(dataset, init_points, init_colors, num_init, dev)
    means = torch.nn.Parameter(seed['means'].to(dev))
    scales = torch.nn.Parameter(seed['scales'].to(dev))
    quats = torch.nn.Parameter(seed['quats'].to(dev))
    opacities = torch.nn.Parameter(seed['opacities'].to(dev))
    colors = torch.nn.Parameter(seed['colors'].to(dev))

    opt = torch.optim.Adam([
        {'params': [means], 'lr': lr * extent},
        {'params': [scales], 'lr': lr},
        {'params': [quats], 'lr': lr},
        {'params': [opacities], 'lr': lr * 3},
        {'params': [colors], 'lr': lr * 3},
    ])

    ssim_fn = make_ssim(dev)

    def render_view(i):
        out, _, _ = rasterization(
            means, F.normalize(quats, dim=-1), torch.exp(scales),
            torch.sigmoid(opacities), torch.sigmoid(colors),
            viewmats[i:i + 1], K, W, H,
        )
        return out[0]

    loss_history: list[float] = []
    for it in range(iters):
        idx = it % viewmats.shape[0]
        loss, mse = _photometric_loss(render_view(idx), gts[idx], ssim_fn, ssim_lambda)
        opt.zero_grad()
        loss.backward()
        opt.step()
        # Only sync the loss to the host when we actually record/print it; a
        # per-iter .cpu() would force a GPU->CPU stall every step. The final
        # iter is always recorded so loss_history[-1] is the converged mse.
        if it == iters - 1 or (log_every and it % log_every == 0):
            loss_history.append(float(mse.detach().cpu()))
            if log_every:
                print(f'iter {it:5d}  mse {loss_history[-1]:.6f}', flush=True)

    metrics = _eval_views(render_view, gts, ssim_fn)
    return {
        'means': means.detach().cpu().numpy(),
        'scales_log': scales.detach().cpu().numpy(),
        'quats': F.normalize(quats, dim=-1).detach().cpu().numpy(),
        'opacities_logit': opacities.detach().cpu().numpy(),
        'colors_rgb': torch.sigmoid(colors).detach().cpu().numpy(),
        'loss_history': loss_history,
        'psnr': metrics['psnr'], 'ssim': metrics['ssim'],
    }


def knn_scale_log(points: np.ndarray, k: int = 3) -> np.ndarray:
    """Per-point isotropic log-scale from mean distance to ``k`` nearest neighbours.

    The INRIA 3DGS scale init: a Gaussian's initial size should match the local
    point spacing, so dense regions get small splats and sparse regions large
    ones -- far better than a single scene-wide scale when seeding from a LiDAR
    cloud of non-uniform density. Returns ``(N,3)`` log-scales (isotropic).
    Falls back to the global-spacing estimate if scipy/KDTree is unavailable.
    """
    pts = np.asarray(points, dtype=np.float64)
    n = pts.shape[0]
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        # scipy genuinely absent (e.g. CI): fall back to a global-spacing
        # estimate. Only ImportError is swallowed -- a real failure inside the
        # query below (OOM, bad shapes) must surface, not silently degrade to a
        # uniform scale that defeats the point of --knn-scale-init.
        import warnings
        warnings.warn('scipy.spatial unavailable; knn_scale_log falls back to '
                      'a single global spacing (per-point k-NN scale disabled)')
        extent = float(np.linalg.norm(pts - pts.mean(axis=0), axis=1).max()) + 1e-3
        mean_d = np.full(n, extent / max(n, 1) ** (1 / 3) * 0.5)
    else:
        tree = cKDTree(pts)
        d, _ = tree.query(pts, k=min(k + 1, n))  # col 0 is self (dist 0)
        mean_d = d[:, 1:].mean(axis=1) if d.ndim == 2 and d.shape[1] > 1 else d.ravel()
    mean_d = np.clip(mean_d, 1e-4, None)
    return np.log(mean_d).astype(np.float32)[:, None].repeat(3, axis=1)


def _seed_params(dataset: dict, init_points, init_colors, num_init, device,
                 knn_scale: bool = False):
    """Build the initial raw Gaussian parameters + scene extent (shared init)."""
    import torch
    import numpy as _np

    cam_centers = _np.stack([_np.linalg.inv(v)[:3, 3] for v in dataset['viewmats']])
    center = cam_centers.mean(axis=0)
    extent = float(_np.linalg.norm(cam_centers - center, axis=1).max()) + 1e-3
    if init_points is not None and len(init_points) > 0:
        means0 = _np.asarray(init_points, dtype=_np.float32)
    else:
        rng = _np.random.default_rng(0)
        means0 = (center + rng.normal(scale=extent * 0.5, size=(num_init, 3))).astype(_np.float32)
    n = means0.shape[0]
    scale_log = float(_np.log(extent / max(n, 1) ** (1 / 3) * 0.5))
    if init_colors is not None and len(init_colors) == n:
        c0 = _np.clip(_np.asarray(init_colors, dtype=_np.float32), 1e-4, 1 - 1e-4)
        colors0 = torch.logit(torch.tensor(c0))
    else:
        colors0 = torch.zeros((n, 3))
    if knn_scale and init_points is not None and len(init_points) > 0:
        scales0 = torch.tensor(knn_scale_log(means0))
    else:
        scales0 = torch.full((n, 3), scale_log)
    return {
        'means': torch.tensor(means0),
        'scales': scales0,
        'quats': torch.tensor([[1.0, 0.0, 0.0, 0.0]]).repeat(n, 1),
        'opacities': torch.full((n,), 0.1),
        'colors': colors0,
    }, extent


def _se3_exp_torch(tau):
    """Differentiable SO(3)+t exp: tau=[omega(3), t(3)] -> 4x4 (torch).

    Uses the unnormalised hat(omega) so the coefficients ``sin(theta)/theta``
    and ``(1-cos)/theta^2`` carry the angle -- this stays differentiable in
    omega at theta=0 (an eps-guarded sqrt), unlike a normalise-by-theta path
    which would zero the rotation gradient at the origin.
    """
    import torch

    omega, t = tau[:3], tau[3:]
    z = torch.zeros((), device=tau.device, dtype=tau.dtype)
    w = torch.stack([
        torch.stack([z, -omega[2], omega[1]]),
        torch.stack([omega[2], z, -omega[0]]),
        torch.stack([-omega[1], omega[0], z]),
    ])
    theta2 = torch.dot(omega, omega)
    theta = torch.sqrt(theta2 + 1e-12)
    a = torch.sin(theta) / theta
    b = (1 - torch.cos(theta)) / (theta2 + 1e-12)
    eye = torch.eye(3, device=tau.device, dtype=tau.dtype)
    R = eye + a * w + b * (w @ w)
    M = torch.eye(4, device=tau.device, dtype=tau.dtype)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def train_densify(dataset: dict, *, init_points=None, init_colors=None,
                  num_init: int = 20000, iters: int = 3000, lr: float = 1e-2,
                  device: str = 'cuda', log_every: int = 200,
                  optimize_extrinsic: bool = False,
                  optimize_pose_groups: bool = False,
                  optimize_exposure: bool = False,
                  ssim_lambda: float = 0.0, knn_scale: bool = False,
                  sh_degree: Optional[int] = None,
                  antialiased: bool = False, mcmc: bool = False,
                  mcmc_cap: int = 500000,
                  lidar_depth_lambda: float = 0.0) -> dict:
    """Train with gsplat DefaultStrategy adaptive density control (densify/prune).

    Same I/O contract as ``train`` but the Gaussian count grows/shrinks via the
    strategy, which sharpens detail beyond the fixed-count ``train``. When
    ``optimize_extrinsic`` is set, a single shared 6-DoF SE(3) correction is
    co-optimised photometrically and returned as ``extrinsic_delta`` -- this
    recovers the camera<-LiDAR lever arm/rotation that a frame-convention
    approximation omits (all frames share the same extrinsic error, so one
    left-multiplied SE(3) on every view matrix corrects it).

    When ``optimize_pose_groups`` is set, one world-side SE(3) correction per
    ``dataset['groups']`` value (e.g. per merged capture session) is
    co-optimised instead: registration error between sessions sits on the world
    side, so each group's correction right-multiplies its view matrices. Group
    0 is held fixed as the gauge; the refined per-group matrices are returned
    as ``pose_group_mats``.

    ``optimize_exposure`` adds a per-group affine colour compensation
    (per-channel gain + bias applied to the render inside the loss only),
    absorbing auto-exposure/white-balance differences between merged sessions.
    Group 0 is the reference, so the exported Gaussians keep its exposure.

    ``lidar_depth_lambda`` (>0) adds a LiDAR depth-supervision term: the init
    cloud is projected into every view as a sparse ground-truth depth map and
    the rendered expected depth (gsplat ``RGB+ED``) is pulled towards it with an
    L1 loss at those pixels. The LiDAR carries metric geometry, so this anchors
    the Gaussians to the real surface and curbs the view-dependent floaters /
    collapse that photometric-only training leaves on sparse captures. Requires
    ``init_points`` (the LiDAR cloud) -- raises ``ValueError`` without them, and
    is mutually exclusive with the pose-optimisation levers (the GT depth is
    projected at fixed poses, and trusting vs distrusting the poses is
    contradictory).
    """
    if lidar_depth_lambda > 0.0:
        if init_points is None or len(init_points) == 0:
            raise ValueError('lidar_depth_lambda>0 requires an init point cloud '
                             '(--init-ply) to supervise depth against')
        if optimize_extrinsic or optimize_pose_groups:
            # The depth GT is projected once from the given viewmats; pose
            # self-calibration moves the cameras under it, so the rendered depth
            # and GT would drift into different frames. The two levers are also
            # contradictory in intent (depth supervision trusts the LiDAR
            # geometry; pose refinement distrusts the poses). Forbid the combo.
            raise ValueError('lidar_depth_lambda is incompatible with '
                             '--optimize-extrinsic / --optimize-pose-groups '
                             '(GT depth is projected at fixed poses)')
    import torch
    import torch.nn.functional as F
    import imageio.v3 as iio
    from gsplat import rasterization, DefaultStrategy, MCMCStrategy

    dev = torch.device(device)
    seed, extent = _seed_params(dataset, init_points, init_colors, num_init, dev,
                                knn_scale=knn_scale)
    # INRIA-style position LR: 1.6e-4 * scene_extent at the default lr=1e-2,
    # decayed exponentially to 1% over the run (constant lr*extent is ~60x too
    # hot and visibly churns large scenes -- walking captures never sharpen).
    lrs = {'means': lr * 0.016 * extent, 'scales': lr, 'quats': lr,
           'opacities': lr * 3, 'colors': lr * 3}
    if sh_degree is not None:
        # Split the flat colour into SH coefficients: a band-0 DC term (seeded
        # from the init colour) plus higher bands (zero, lower LR), matching the
        # INRIA convention so the rasteriser models view-dependent appearance.
        n = seed['colors'].shape[0]
        dc_rgb = torch.sigmoid(seed.pop('colors'))
        seed['sh0'] = (dc_rgb - 0.5) / SH_C0
        k_rest = (sh_degree + 1) ** 2 - 1
        if k_rest > 0:
            seed['shN'] = torch.zeros((n, k_rest, 3))
        lrs.pop('colors')
        lrs['sh0'] = lr * 3
        lrs['shN'] = lr * 3 / 20.0
    params = torch.nn.ParameterDict(
        {k: torch.nn.Parameter(v.to(dev)) for k, v in seed.items()}
    )
    optimizers = {
        k: torch.optim.Adam([{'params': [params[k]], 'lr': lrs[k]}])
        for k in params
    }
    means_sched = torch.optim.lr_scheduler.ExponentialLR(
        optimizers['means'], gamma=0.01 ** (1.0 / max(iters, 1)))

    K = torch.tensor(dataset['K'], dtype=torch.float32, device=dev)[None]
    W, H = dataset['width'], dataset['height']
    viewmats = torch.tensor(np.stack(dataset['viewmats']), dtype=torch.float32, device=dev)
    gts = []
    for p in dataset['image_paths']:
        img = np.asarray(iio.imread(p), dtype=np.float32) / 255.0
        if img.ndim == 2:
            img = np.stack([img] * 3, axis=-1)
        gts.append(torch.tensor(img[..., :3], device=dev))
    gts = torch.stack(gts)

    # LiDAR depth supervision: project the init cloud into every view as a
    # sparse GT depth map (flattened pixel index + metric z), kept on-device for
    # a cheap gather in the loss. Built once; only the current view is used per
    # iter. Empty views (no points project) simply contribute no depth term.
    depth_gt = None
    if lidar_depth_lambda > 0.0:
        import lidarslam_benchmark_tools.gaussian_splatting.pointcloud_io as _pcio
        maps = _pcio.project_depth_maps(
            np.asarray(init_points, dtype=np.float64),
            dataset['viewmats'], dataset['K'], W, H)
        depth_gt = [(torch.tensor(pix, dtype=torch.long, device=dev),
                     torch.tensor(d, dtype=torch.float32, device=dev))
                    for pix, d in maps]
        n_sup = sum(int(p.numel()) for p, _ in depth_gt)
        if n_sup == 0:
            # No init point projects into any view -- almost always a bad
            # extrinsic or a cloud/camera frame mismatch. Fail loudly rather
            # than train a silent photometric-only model.
            raise ValueError('lidar_depth_lambda>0 but the init cloud projects '
                             'into 0 pixels across all views (check extrinsics/'
                             'frames)')
        print(f'LiDAR depth supervision: {n_sup} GT pixels over '
              f'{len(depth_gt)} views (lambda={lidar_depth_lambda})', flush=True)

    if mcmc:
        # MCMC keeps a fixed Gaussian budget (cap_max) and relocates dead/low-
        # opacity splats by a Metropolis-Hastings move instead of clone/split.
        # Needs L1 opacity+scale regularisation in the loss and the means LR in
        # step_post_backward. Often beats clone/split densification per budget.
        strategy = MCMCStrategy(
            cap_max=mcmc_cap, refine_start_iter=max(100, iters // 10),
            refine_stop_iter=int(iters * 0.85), refine_every=100, verbose=False,
        )
        strategy.check_sanity(params, optimizers)
        state = strategy.initialize_state()
    else:
        strategy = DefaultStrategy(
            refine_start_iter=max(100, iters // 10),
            refine_stop_iter=int(iters * 0.85),
            # Opacity reset helps long (30k-iter) runs prune floaters but
            # destabilises short runs; disable by pushing it past the horizon.
            refine_every=100, reset_every=iters + 1, verbose=False,
        )
        strategy.check_sanity(params, optimizers)
        state = strategy.initialize_state(scene_scale=extent)

    tau = torch.zeros(6, device=dev, requires_grad=optimize_extrinsic)
    ext_opt = (torch.optim.Adam([tau], lr=lr * 0.1)
               if optimize_extrinsic else None)

    groups = list(dataset.get('groups') or [0] * viewmats.shape[0])
    n_groups = max(groups) + 1
    # One world-side SE(3) correction per capture session; group 0 is the
    # gauge and stays identity (its tau row simply never receives gradient).
    taus_g = [torch.zeros(6, device=dev,
                          requires_grad=optimize_pose_groups and g > 0)
              for g in range(n_groups)]
    grp_opt = (torch.optim.Adam(taus_g[1:], lr=lr * 0.1)
               if optimize_pose_groups and n_groups > 1 else None)

    # Per-group affine colour compensation: render' = render*(1+g) + b inside
    # the loss only. Group 0 is the exposure reference (parameters frozen).
    expo = [torch.zeros(6, device=dev,
                        requires_grad=optimize_exposure and g > 0)
            for g in range(n_groups)]
    exp_opt = (torch.optim.Adam(expo[1:], lr=lr)
               if optimize_exposure and n_groups > 1 else None)

    ssim_fn = make_ssim(dev)

    def _colors_arg():
        if sh_degree is None:
            return torch.sigmoid(params['colors'])
        sh = params['sh0'][:, None, :]
        if 'shN' in params:
            sh = torch.cat([sh, params['shN']], dim=1)
        return sh

    rasterize_mode = 'antialiased' if antialiased else 'classic'

    def render_view(i, vm=None, *, with_depth=False):
        out, _, info = rasterization(
            params['means'], F.normalize(params['quats'], dim=-1),
            torch.exp(params['scales']), torch.sigmoid(params['opacities']),
            _colors_arg(),
            viewmats[i:i + 1] if vm is None else vm, K, W, H,
            sh_degree=sh_degree, rasterize_mode=rasterize_mode, packed=False,
            render_mode='RGB+ED' if with_depth else 'RGB',
        )
        if with_depth:
            # RGB+ED packs expected depth as the trailing channel: (H,W,4).
            assert out.shape[-1] == 4, f'expected RGB+ED (4 ch), got {out.shape}'
            return out[0][..., :3], out[0][..., 3], info
        return out[0], info

    loss_history: list[float] = []
    for it in range(iters):
        idx = it % viewmats.shape[0]
        vm = viewmats[idx:idx + 1]
        if optimize_pose_groups:
            vm = (vm[0] @ _se3_exp_torch(taus_g[groups[idx]]))[None]
        if optimize_extrinsic:
            vm = (_se3_exp_torch(tau) @ vm[0])[None]
        if depth_gt is not None:
            render, depth, info = render_view(idx, vm, with_depth=True)
        else:
            render, info = render_view(idx, vm)
        if not mcmc:
            strategy.step_pre_backward(params, optimizers, state, it, info)
        if optimize_exposure:
            e = expo[groups[idx]]
            render = render * (1.0 + e[:3]) + e[3:]
        loss, mse = _photometric_loss(render, gts[idx], ssim_fn, ssim_lambda)
        if depth_gt is not None:
            pix, dgt = depth_gt[idx]
            if pix.numel() > 0:
                # Pull the rendered expected depth toward the LiDAR depth at the
                # sparse projected pixels (metric L1).
                dr = depth.reshape(-1)[pix]
                loss = loss + lidar_depth_lambda * F.l1_loss(dr, dgt)
        if optimize_exposure:
            # keep the affine compensation near identity: it should absorb
            # auto-exposure drift, not repaint inconsistent geometry
            loss = loss + 0.1 * (expo[groups[idx]] ** 2).sum()
        if mcmc:
            # MCMC paper regularisers (gsplat defaults): keep opacities/scales small.
            loss = (loss + 0.01 * torch.sigmoid(params['opacities']).abs().mean()
                    + 0.01 * torch.exp(params['scales']).abs().mean())
        loss.backward()
        if not mcmc:
            strategy.step_post_backward(params, optimizers, state, it, info,
                                        packed=False)
        for opt in optimizers.values():
            opt.step()
            opt.zero_grad(set_to_none=True)
        means_sched.step()
        if ext_opt is not None:
            ext_opt.step()
            ext_opt.zero_grad(set_to_none=True)
        if grp_opt is not None:
            grp_opt.step()
            grp_opt.zero_grad(set_to_none=True)
        if exp_opt is not None:
            exp_opt.step()
            exp_opt.zero_grad(set_to_none=True)
        if mcmc:
            # MCMC relocates after the optimiser step and needs the means LR.
            strategy.step_post_backward(params, optimizers, state, it, info,
                                        lr=optimizers['means'].param_groups[0]['lr'])
        # Sync mse to the host only when recorded/printed (avoid a per-iter
        # GPU->CPU stall); the final iter is always recorded.
        if it == iters - 1 or (log_every and it % log_every == 0):
            loss_history.append(float(mse.detach().cpu()))
            if log_every:
                print(f'iter {it:5d}  mse {loss_history[-1]:.6f}  '
                      f'gaussians {params["means"].shape[0]}', flush=True)

    eval_vm = None
    if optimize_extrinsic or optimize_pose_groups:
        eval_vm = viewmats
        if optimize_pose_groups:
            eval_vm = torch.stack([
                eval_vm[i] @ _se3_exp_torch(taus_g[groups[i]].detach())
                for i in range(eval_vm.shape[0])])
        if optimize_extrinsic:
            eval_vm = _se3_exp_torch(tau.detach()) @ eval_vm[..., :, :]

    def _eval_render(i):
        out = render_view(i, None if eval_vm is None else eval_vm[i:i + 1])[0]
        if optimize_exposure:
            e = expo[groups[i]].detach()
            out = out * (1.0 + e[:3]) + e[3:]
        return out

    metrics = _eval_views(_eval_render, gts, ssim_fn)
    depth_mae = None
    if depth_gt is not None:
        # Median absolute error (m) between rendered and LiDAR depth at the
        # supervised pixels -- a geometry-fidelity readout independent of PSNR.
        errs = []
        with torch.no_grad():
            for i in range(viewmats.shape[0]):
                pix, dgt = depth_gt[i]
                if pix.numel() == 0:
                    continue
                vm = None if eval_vm is None else eval_vm[i:i + 1]
                dr = render_view(i, vm, with_depth=True)[1].reshape(-1)[pix]
                errs.append(torch.abs(dr - dgt))
        if errs:
            depth_mae = float(torch.median(torch.cat(errs)).cpu())
            print(f'LiDAR depth median abs error: {depth_mae:.4f} m', flush=True)
    if sh_degree is None:
        colors_rgb = torch.sigmoid(params['colors']).detach().cpu().numpy()
        sh_rest = None
    else:
        # f_dc round-trips sh0 through export_ply's (rgb-0.5)/C0; higher bands go
        # out verbatim as f_rest.
        colors_rgb = (params['sh0'] * SH_C0 + 0.5).detach().cpu().numpy()
        sh_rest = (params['shN'].detach().cpu().numpy()
                   if 'shN' in params else None)
    out = {
        'means': params['means'].detach().cpu().numpy(),
        'scales_log': params['scales'].detach().cpu().numpy(),
        'quats': F.normalize(params['quats'], dim=-1).detach().cpu().numpy(),
        'opacities_logit': params['opacities'].detach().cpu().numpy(),
        'colors_rgb': colors_rgb,
        'sh_rest': sh_rest,
        'loss_history': loss_history,
        'psnr': metrics['psnr'], 'ssim': metrics['ssim'],
        'depth_mae': depth_mae,
    }
    if optimize_extrinsic:
        # viewmat_refined = M @ viewmat with M = exp(tau); equivalently the
        # camera<-body correction is delta = inv(M), so body<-cam gains inv(M).
        m = _se3_exp_torch(tau.detach()).cpu().numpy()
        out['extrinsic_delta'] = np.linalg.inv(m)  # right-multiply onto body_T_cam
        out['tau'] = tau.detach().cpu().numpy()
    if optimize_pose_groups:
        # viewmat_refined = viewmat @ N_g with N_g = exp(tau_g): a world-side
        # registration correction for every view of group g.
        out['pose_group_mats'] = np.stack(
            [_se3_exp_torch(t.detach()).cpu().numpy() for t in taus_g])
        out['pose_group_taus'] = np.stack(
            [t.detach().cpu().numpy() for t in taus_g])
    if optimize_exposure:
        out['exposure_groups'] = np.stack(
            [e.detach().cpu().numpy() for e in expo])
    return out


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--transforms', required=True, help='transforms.json path')
    p.add_argument('--out', required=True, help='output .ply path')
    p.add_argument('--init-ply', default=None,
                   help='LiDAR-primed init cloud (xyz[+rgb]); random init if omitted')
    p.add_argument('--iters', type=int, default=2000)
    p.add_argument('--num-init', type=int, default=20000)
    p.add_argument('--lr', type=float, default=1e-2)
    p.add_argument('--device', default='cuda')
    p.add_argument('--ssim-lambda', type=float, default=0.2,
                   help='weight of the D-SSIM term (INRIA loss = (1-l)*L1 + '
                        'l*(1-SSIM)); 0 = pure MSE (legacy behaviour)')
    p.add_argument('--knn-scale-init', action='store_true',
                   help='seed per-Gaussian scale from k-NN spacing of the init '
                        'cloud (INRIA-style) instead of one scene-wide scale')
    p.add_argument('--sh-degree', type=int, default=None,
                   help='spherical-harmonics degree for view-dependent colour '
                        '(e.g. 3); omitted = flat band-0 colour (implies --densify)')
    p.add_argument('--antialiased', action='store_true',
                   help="gsplat 'antialiased' rasterize mode (opacity-compensated "
                        'screen-space filter; reduces aliasing, implies --densify)')
    p.add_argument('--mcmc', action='store_true',
                   help='use gsplat MCMCStrategy (fixed budget + relocation) '
                        'instead of clone/split DefaultStrategy (implies --densify)')
    p.add_argument('--mcmc-cap', type=int, default=500000,
                   help='max Gaussian budget for --mcmc (default 500000)')
    p.add_argument('--lidar-depth-lambda', type=float, default=0.0,
                   help='weight of the LiDAR depth-supervision L1 term; projects '
                        'the --init-ply cloud into each view as sparse GT depth '
                        'and anchors the rendered depth to it (implies --densify, '
                        'needs --init-ply; 0 = off)')
    p.add_argument('--densify', action='store_true',
                   help='use gsplat DefaultStrategy adaptive density control')
    p.add_argument('--optimize-extrinsic', action='store_true',
                   help='co-optimise a shared 6-DoF camera extrinsic correction '
                        '(implies --densify); writes <out>.extrinsic.yaml')
    p.add_argument('--optimize-pose-groups', action='store_true',
                   help='co-optimise one world-side SE(3) per "bag" group in '
                        'transforms.json (multi-session registration refine, '
                        'implies --densify); writes <transforms>_refined.json')
    p.add_argument('--optimize-exposure', action='store_true',
                   help='co-optimise per-"bag"-group affine colour gain/bias '
                        '(auto-exposure/WB compensation for multi-session '
                        'merges; group 0 is the reference, implies --densify)')
    p.add_argument('--extrinsic', default=None,
                   help='base body<-camera extrinsic YAML to compose the '
                        'recovered correction onto (for --optimize-extrinsic)')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    dataset = load_transforms(args.transforms)
    print(f'loaded {len(dataset["image_paths"])} views @ {dataset["width"]}x{dataset["height"]}')
    init_points = None
    init_colors = None
    if args.init_ply:
        import lidarslam_benchmark_tools.gaussian_splatting.pointcloud_io as pcio
        init_points, rgb = pcio.read_ply_xyz(args.init_ply)
        init_colors = None if rgb is None else rgb.astype(np.float32) / 255.0
        print(f'LiDAR-primed init: {len(init_points)} points from {args.init_ply}')
    if (args.densify or args.optimize_extrinsic or args.optimize_pose_groups
            or args.optimize_exposure or args.sh_degree is not None
            or args.antialiased or args.mcmc or args.lidar_depth_lambda > 0.0):
        params = train_densify(
            dataset, init_points=init_points, init_colors=init_colors,
            num_init=args.num_init, iters=args.iters, lr=args.lr,
            device=args.device, optimize_extrinsic=args.optimize_extrinsic,
            optimize_pose_groups=args.optimize_pose_groups,
            optimize_exposure=args.optimize_exposure,
            ssim_lambda=args.ssim_lambda, knn_scale=args.knn_scale_init,
            sh_degree=args.sh_degree, antialiased=args.antialiased,
            mcmc=args.mcmc, mcmc_cap=args.mcmc_cap,
            lidar_depth_lambda=args.lidar_depth_lambda)
    else:
        params = train(dataset, init_points=init_points, init_colors=init_colors,
                       num_init=args.num_init, iters=args.iters,
                       lr=args.lr, device=args.device, ssim_lambda=args.ssim_lambda)
    out = export_ply(args.out, params['means'], params['scales_log'],
                     params['quats'], params['opacities_logit'],
                     params['colors_rgb'], params.get('sh_rest'))
    dmae = params.get('depth_mae')
    print(f'final mse {params["loss_history"][-1]:.6f}  '
          f'PSNR {params.get("psnr", float("nan")):.2f} dB  '
          f'SSIM {params.get("ssim", float("nan")):.4f}'
          + (f'  depthMAE {dmae:.4f} m' if dmae is not None else '')
          + f' -> {out}')
    if 'extrinsic_delta' in params:
        import yaml
        from lidarslam_benchmark_tools.gaussian_splatting.extract_posed_images import parse_extrinsic_dict
        base = np.eye(4)
        if args.extrinsic:
            # Reuse the extractor's parser so both 'matrix' and
            # 'translation'+'rotation_xyzw' forms compose (the extractor accepts
            # either, so reading only ['matrix'] here would KeyError at the very
            # end of training on the translation/quaternion form).
            base = parse_extrinsic_dict(
                yaml.safe_load(Path(args.extrinsic).read_text()))
        refined = base @ params['extrinsic_delta']
        ext_path = Path(str(out) + '.extrinsic.yaml')
        ext_path.write_text(yaml.safe_dump(
            {'matrix': refined.tolist(),
             'note': 'photometrically self-calibrated body<-camera extrinsic'}))
        print(f'recovered extrinsic tau={np.round(params["tau"], 4).tolist()} '
              f'-> {ext_path}')
    if 'pose_group_mats' in params:
        # Bake the refined per-group world registration back into a
        # transforms.json so downstream renderers see the corrected poses:
        # w2c' = w2c @ N_g, and the stored matrix is OpenGL c2w.
        doc = json.loads(Path(args.transforms).read_text())
        flip = pi.ROS_OPTICAL_TO_OPENGL
        group_ids: dict = {}
        for fr in doc['frames']:
            g = group_ids.setdefault(fr.get('bag', ''), len(group_ids))
            w2c = np.linalg.inv(np.asarray(fr['transform_matrix']) @ flip)
            c2w_cv = np.linalg.inv(w2c @ params['pose_group_mats'][g])
            fr['transform_matrix'] = (c2w_cv @ flip).tolist()
        ref_path = Path(args.transforms).with_name(
            Path(args.transforms).stem + '_refined.json')
        ref_path.write_text(json.dumps(doc))
        norms = np.linalg.norm(params['pose_group_taus'][:, 3:], axis=1)
        print(f'pose-group |t| corrections: {np.round(norms, 3).tolist()} m '
              f'-> {ref_path}')
    if 'exposure_groups' in params:
        gains = 1.0 + params['exposure_groups'][:, :3].mean(axis=1)
        print(f'exposure gains per group: {np.round(gains, 3).tolist()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
