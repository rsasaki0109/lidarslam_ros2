#!/usr/bin/env python3
"""Reconstruct a coloured surface mesh from a coloured SLAM point cloud.

Second export off the coloured-cloud "hub" (after ``map_export.py``'s GIS
bridge): turn the cloud into a triangle mesh that keeps its per-point colour, so
the map reads as a surface instead of dots — for README hero shots, clash /
coverage checks, and as the geometric stepping stone toward CAD/BIM.

Unlike ``pointcloud_io`` (numpy-only) this needs ``open3d``; it is imported
lazily so merely importing this module — or the rest of the toolchain — never
requires open3d. Two reconstructors:

* ``poisson`` — watertight Screened Poisson; best for dense, closed-ish scans.
  Extrapolates into unseen space, so low-density vertices are trimmed
  (``density_quantile``).
* ``bpa`` — Ball-Pivoting; stays on the measured surface (no invented geometry),
  better for walls / open scenes. Needs a radius near the point spacing.

See roadmap §12 and ``docs/research/3dgs-postprocess-map-design.md``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence

import numpy as np


def _o3d():
    try:
        import open3d as o3d  # noqa: F401
    except ImportError as exc:  # pragma: no cover - env-dependent
        raise ImportError(
            'mesh_export needs open3d (`pip install open3d`); '
            'the numpy-only GIS/thin exports in map_export.py do not.') from exc
    return o3d


def make_cloud(xyz: np.ndarray, rgb: Optional[np.ndarray] = None,
               *, estimate_normals: bool = True, normal_radius: float = 0.5,
               normal_max_nn: int = 30):
    """Build an open3d point cloud (colours in [0,1], optional normals)."""
    o3d = _o3d()
    xyz = np.asarray(xyz, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f'xyz must be (N,3), got {xyz.shape}')
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    if rgb is not None:
        rgb = np.asarray(rgb, dtype=np.float64)
        if rgb.shape != xyz.shape:
            raise ValueError('rgb must match xyz shape')
        pcd.colors = o3d.utility.Vector3dVector(np.clip(rgb / 255.0, 0.0, 1.0))
    if estimate_normals:
        o3d.geometry.PointCloud.estimate_normals(
            pcd, search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=normal_radius, max_nn=normal_max_nn))
    return pcd


def reconstruct_mesh(xyz: np.ndarray, rgb: Optional[np.ndarray] = None, *,
                     method: str = 'poisson', depth: int = 9,
                     density_quantile: float = 0.02,
                     radii: Optional[Sequence[float]] = None,
                     normal_radius: float = 0.5):
    """Reconstruct a coloured ``open3d`` ``TriangleMesh`` from a cloud.

    ``method='poisson'`` runs Screened Poisson at ``depth`` then trims the
    ``density_quantile`` lowest-density vertices (0 keeps all) to cut the
    balloon of extrapolated geometry Poisson invents beyond the samples.
    ``method='bpa'`` runs Ball-Pivoting over ``radii`` (metres; defaults to a
    spread around the median nearest-neighbour spacing) and never invents
    geometry. Per-point colour is carried onto the mesh vertices either way.
    """
    o3d = _o3d()
    pcd = make_cloud(xyz, rgb, estimate_normals=True, normal_radius=normal_radius)

    if method == 'poisson':
        mesh, densities = \
            o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                pcd, depth=depth)
        if density_quantile > 0.0 and len(densities) > 0:
            thresh = np.quantile(np.asarray(densities), density_quantile)
            mesh.remove_vertices_by_mask(np.asarray(densities) < thresh)
    elif method == 'bpa':
        if radii is None:
            d = np.asarray(pcd.compute_nearest_neighbor_distance())
            base = float(np.median(d)) if d.size else 0.1
            radii = [base, base * 2.0, base * 4.0]
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, o3d.utility.DoubleVector(list(radii)))
    else:
        raise ValueError(f"method must be 'poisson' or 'bpa', got {method!r}")

    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh


def write_mesh(path: str | Path, mesh) -> Path:
    """Write a mesh to ``path`` (.ply / .obj — format from the suffix)."""
    o3d = _o3d()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok = o3d.io.write_triangle_mesh(str(path), mesh)
    if not ok:
        raise IOError(f'open3d failed to write mesh to {path}')
    return path


def _build_arg_parser():
    import argparse
    p = argparse.ArgumentParser(
        description='Reconstruct a coloured mesh from a coloured PLY cloud.')
    p.add_argument('input', help='input .ply (xyz [+ rgb])')
    p.add_argument('output', help='output mesh (.ply / .obj)')
    p.add_argument('--method', choices=['poisson', 'bpa'], default='poisson')
    p.add_argument('--depth', type=int, default=9,
                   help='Poisson octree depth (higher = finer, slower)')
    p.add_argument('--density-quantile', type=float, default=0.02,
                   help='Poisson: trim this fraction of lowest-density verts')
    p.add_argument('--radii', type=float, nargs='+', default=None,
                   help='BPA ball radii [m] (default: from point spacing)')
    p.add_argument('--normal-radius', type=float, default=0.5,
                   help='normal-estimation search radius [m]')
    p.add_argument('--thin-voxel', type=float, default=0.0,
                   help='voxel-downsample input before meshing [m] (0=off)')
    return p


def main(argv=None) -> int:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from pointcloud_io import read_ply_xyz, voxel_downsample
    args = _build_arg_parser().parse_args(argv)
    xyz, rgb = read_ply_xyz(args.input)
    xyz, rgb = voxel_downsample(xyz, args.thin_voxel, rgb)
    mesh = reconstruct_mesh(xyz, rgb, method=args.method, depth=args.depth,
                            density_quantile=args.density_quantile,
                            radii=args.radii, normal_radius=args.normal_radius)
    out = write_mesh(args.output, mesh)
    print(f'wrote {out} ({len(mesh.triangles)} triangles, '
          f'{len(mesh.vertices)} vertices, '
          f'coloured={mesh.has_vertex_colors()})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
