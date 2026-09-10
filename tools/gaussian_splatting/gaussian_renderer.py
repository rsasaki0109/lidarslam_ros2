#!/usr/bin/env python3
"""Persistent 3DGS renderer for closed-loop sensor simulation (gsplat, Apache-2.0).

Phase 2 of the 3DGS-as-sim2real track. ``render_path.render_frames`` re-uploads
every Gaussian to the GPU on each call, which is fine for a one-shot flythrough
but wasteful in a closed loop. ``GaussianRenderer`` uploads the model once and
keeps the tensors resident, so each ``render(...)`` is just a rasterisation --
fast enough for an Autoware-in-the-loop camera (see scripts: 30+ FPS at
practical resolutions on a GPU).

The pose maths (``pose_to_viewmat`` and the camera-frame conventions) are
numpy-only and unit tested on CPU; the renderer itself needs CUDA + torch +
gsplat and is exercised by the sensor-sim node / benchmarks.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

import lidarslam_benchmark_tools.gaussian_splatting.posed_images as pi
from lidarslam_benchmark_tools.gaussian_splatting.render_path import infer_sh_degree, load_gaussian_ply
from lidarslam_benchmark_tools.gaussian_splatting.train_gsplat import SH_C0


# --------------------------------------------------------------------------- #
# Pure pose maths (no torch/CUDA)
# --------------------------------------------------------------------------- #
def pose_to_viewmat(t_world_base: np.ndarray, t_base_cam: np.ndarray,
                    t_align: Optional[np.ndarray] = None) -> np.ndarray:
    """World->camera matrix (gsplat viewmat) from an ego pose and extrinsics.

    ``t_world_base`` is the ego pose ``world<-base_link`` (as published by a
    localiser), ``t_base_cam`` the static extrinsic ``base_link<-camera_optical``
    (OpenCV optical frame: +x right, +y down, +z forward, matching the training
    ``transforms.json``), and ``t_align`` an optional ``model_world<-pose_world``
    rigid transform for when the localiser's world frame differs from the frame
    the 3DGS model was built in (identity when they coincide).

    The camera pose in the model frame is ``T = t_align @ t_world_base @
    t_base_cam`` and the viewmat the rasteriser wants is its inverse.
    """
    t_align = np.eye(4) if t_align is None else np.asarray(t_align, dtype=float)
    cam_in_world = t_align @ np.asarray(t_world_base, dtype=float) \
        @ np.asarray(t_base_cam, dtype=float)
    return np.linalg.inv(cam_in_world)


def transform_from_pos_quat(pos, quat_xyzw) -> np.ndarray:
    """4x4 rigid transform from a translation and an xyzw quaternion (ROS order)."""
    return pi.make_transform(pos, quat_xyzw)


# --------------------------------------------------------------------------- #
# Resident renderer (torch + gsplat; imported lazily)
# --------------------------------------------------------------------------- #
class GaussianRenderer:
    """Hold a trained 3DGS model resident on the GPU and rasterise novel views."""

    def __init__(self, ply: str | Path, *, device: str = 'cuda'):
        import torch
        import torch.nn.functional as F

        self._torch = torch
        g = load_gaussian_ply(ply)
        dev = torch.device(device)
        self.device = dev
        self.num_gaussians = int(g['means'].shape[0])
        self._means = torch.tensor(g['means'], dtype=torch.float32, device=dev)
        self._quats = F.normalize(
            torch.tensor(g['quats'], dtype=torch.float32, device=dev), dim=-1)
        self._scales = torch.exp(
            torch.tensor(g['scales_log'], dtype=torch.float32, device=dev))
        self._opac = torch.sigmoid(
            torch.tensor(g['opacities_logit'], dtype=torch.float32, device=dev))
        self.sh_degree = infer_sh_degree(g['sh_rest'])
        if self.sh_degree is None:
            self._colors = torch.tensor(np.clip(g['colors_rgb'], 0.0, 1.0),
                                        dtype=torch.float32, device=dev)
        else:
            sh0 = (g['colors_rgb'] - 0.5) / SH_C0
            sh = np.concatenate([sh0[:, None, :], g['sh_rest']], axis=1)
            self._colors = torch.tensor(sh, dtype=torch.float32, device=dev)

    def render(self, viewmat: np.ndarray, K: np.ndarray, width: int,
               height: int) -> np.ndarray:
        """Rasterise one world->camera view; returns a uint8 (H, W, 3) RGB image."""
        torch = self._torch
        from gsplat import rasterization

        vmt = torch.tensor(np.asarray(viewmat, dtype=np.float32),
                           device=self.device)[None]
        kmat = torch.tensor(np.asarray(K, dtype=np.float32),
                            device=self.device)[None]
        with torch.no_grad():
            out, _, _ = rasterization(self._means, self._quats, self._scales,
                                      self._opac, self._colors, vmt, kmat,
                                      width, height, sh_degree=self.sh_degree,
                                      packed=False)
            frame = (out[0].clamp(0.0, 1.0) * 255.0).to(torch.uint8)
        return frame.cpu().numpy()
