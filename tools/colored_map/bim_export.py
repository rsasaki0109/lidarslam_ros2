#!/usr/bin/env python3
"""Export a SLAM point cloud to IFC (the CAD/BIM-native format).

Last export off the coloured-cloud "hub" (after ``map_export.py`` delimited
text, ``mesh_export.py`` meshes and ``las_export.py`` LAS/LAZ): turn the raw
geometry into *structured* building elements. Planar surfaces are extracted by
sequential RANSAC, classified by their normal into horizontal slabs (floor /
ceiling) and vertical walls, fitted with an oriented rectangle, and emitted as
solid IFC elements (``IfcSlab`` / ``IfcWall`` / ``IfcBuildingElementProxy``)
inside a proper spatial hierarchy — an .ifc that Revit / ArchiCAD / BlenderBIM
/ FreeCAD open natively.

The plane maths (RANSAC fit, normal classification, oriented rectangle, solid
box tessellation) is pure numpy and unit-tested; ``ifcopenshell`` is imported
lazily in the writer only, so the numpy-only exports never depend on it.

Coordinates stay in the SLAM local ENU frame (metres), matching the other
exports and the lanelet2 map.

See roadmap §14 (CAD/BIM) and ``docs/research/3dgs-postprocess-map-design.md``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional
import html
import json

import numpy as np


# --------------------------------------------------------------------------- #
# Pure geometry (numpy only, unit-tested — no ifcopenshell)
# --------------------------------------------------------------------------- #
def refit_plane(points: np.ndarray) -> tuple[np.ndarray, float]:
    """Least-squares plane through ``points`` (PCA). Returns ``(normal, d)``
    with a unit normal and ``normal . x + d = 0`` (so ``d = -normal . centroid``).
    """
    pts = np.asarray(points, dtype=np.float64)
    centroid = pts.mean(axis=0)
    # Smallest-variance eigenvector of the centred covariance is the normal.
    cov = np.cov((pts - centroid).T)
    _, vecs = np.linalg.eigh(cov)
    normal = vecs[:, 0]
    normal = normal / (np.linalg.norm(normal) + 1e-12)
    d = float(-normal.dot(centroid))
    return normal, d


def fit_plane_ransac(points: np.ndarray, *, threshold: float = 0.1,
                     iterations: int = 100, min_inliers: int = 50,
                     orient: Optional[str] = None, up=(0.0, 0.0, 1.0),
                     horiz_dot: float = 0.85, vert_dot: float = 0.35,
                     seed: int = 0):
    """RANSAC-fit the dominant plane in ``points``.

    Samples triples, scores by the count of points within ``threshold`` metres,
    then refines the winner by least squares over its inliers. Returns
    ``(normal, d, inlier_mask)`` (unit normal, ``normal . x + d = 0``) or
    ``None`` if no plane reaches ``min_inliers``.

    ``orient`` restricts the search to a target orientation — ``'vertical'``
    only accepts candidate normals with ``|n . up| <= vert_dot`` (walls),
    ``'horizontal'`` only ``|n . up| >= horiz_dot`` (slabs). This lets a
    vertical pass surface walls even when horizontal floors hold far more
    points (the RANSAC-by-count bias that hides walls in multi-storey scans).
    """
    pts = np.asarray(points, dtype=np.float64)
    n = len(pts)
    if n < 3:
        return None
    up = np.asarray(up, dtype=np.float64)
    up = up / (np.linalg.norm(up) + 1e-12)

    def _orient_ok(nrm):
        if orient is None:
            return True
        a = abs(float(nrm.dot(up)))
        if orient == 'vertical':
            return a <= vert_dot
        if orient == 'horizontal':
            return a >= horiz_dot
        return True

    rng = np.random.default_rng(seed)
    best_mask = None
    best_count = 0
    for _ in range(iterations):
        idx = rng.choice(n, size=3, replace=False)
        p0, p1, p2 = pts[idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue  # collinear sample
        normal = normal / norm
        if not _orient_ok(normal):
            continue  # wrong orientation for this pass
        dist = np.abs((pts - p0).dot(normal))
        mask = dist < threshold
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_mask = mask
    if best_mask is None or best_count < min_inliers:
        return None
    # Refine the plane over all inliers, then re-select inliers once.
    normal, d = refit_plane(pts[best_mask])
    dist = np.abs(pts.dot(normal) + d)
    mask = dist < threshold
    if int(mask.sum()) < min_inliers:
        return None
    normal, d = refit_plane(pts[mask])
    return normal, d, mask


def classify_plane(normal: np.ndarray, *, up=(0.0, 0.0, 1.0),
                   horiz_dot: float = 0.85, vert_dot: float = 0.35) -> str:
    """Classify a plane by its normal: ``'horizontal'`` (slab: floor/ceiling),
    ``'vertical'`` (wall) or ``'other'`` (ramp/clutter).

    Uses ``|normal . up|``: ~1 → horizontal, ~0 → vertical.
    """
    n = np.asarray(normal, dtype=np.float64)
    u = np.asarray(up, dtype=np.float64)
    n = n / (np.linalg.norm(n) + 1e-12)
    u = u / (np.linalg.norm(u) + 1e-12)
    a = abs(float(n.dot(u)))
    if a >= horiz_dot:
        return 'horizontal'
    if a <= vert_dot:
        return 'vertical'
    return 'other'


def plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two orthonormal in-plane axes ``(u, v)`` spanning the plane ``normal``."""
    n = np.asarray(normal, dtype=np.float64)
    n = n / (np.linalg.norm(n) + 1e-12)
    # A helper axis least aligned with n keeps the cross product well-conditioned.
    helper = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, helper)
    u = u / (np.linalg.norm(u) + 1e-12)
    v = np.cross(n, u)
    v = v / (np.linalg.norm(v) + 1e-12)
    return u, v


def oriented_rectangle(points: np.ndarray, normal: np.ndarray):
    """Axis-aligned (in the plane basis) rectangle enclosing ``points``.

    Returns ``(corners, size, thickness)``: ``corners`` is a ``(4,3)`` array of
    the rectangle's 3D corners in order, ``size`` is ``(width, height)`` in the
    plane, ``thickness`` is the point spread along the normal (twice the max
    absolute offset).
    """
    pts = np.asarray(points, dtype=np.float64)
    centroid = pts.mean(axis=0)
    u, v = plane_basis(normal)
    rel = pts - centroid
    au = rel.dot(u)
    av = rel.dot(v)
    an = rel.dot(np.asarray(normal, dtype=np.float64)
                 / (np.linalg.norm(normal) + 1e-12))
    umin, umax = au.min(), au.max()
    vmin, vmax = av.min(), av.max()
    corners_uv = [(umin, vmin), (umax, vmin), (umax, vmax), (umin, vmax)]
    corners = np.array([centroid + cu * u + cv * v for cu, cv in corners_uv])
    thickness = 2.0 * float(np.abs(an).max())
    return corners, (float(umax - umin), float(vmax - vmin)), thickness


def box_from_rectangle(corners: np.ndarray, normal: np.ndarray,
                       thickness: float):
    """Extrude a rectangle into a closed box solid centred on the plane.

    Returns ``(vertices, faces)``: ``vertices`` is ``(8,3)`` (rectangle offset
    by +-thickness/2 along the normal), ``faces`` is 6 quads (as index tuples)
    winding bottom, top, then the four sides.
    """
    corners = np.asarray(corners, dtype=np.float64)
    n = np.asarray(normal, dtype=np.float64)
    n = n / (np.linalg.norm(n) + 1e-12)
    half = max(thickness, 1e-3) / 2.0
    bottom = corners - n * half
    top = corners + n * half
    vertices = np.vstack([bottom, top])
    faces = [
        (0, 1, 2, 3),        # bottom
        (4, 5, 6, 7),        # top
        (0, 1, 5, 4),        # sides
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 0, 4, 7),
    ]
    return vertices, faces


def extrude_polygon(corners: np.ndarray, height: float):
    """Extrude a simple horizontal polygon upward into a closed prism mesh."""
    base = np.asarray(corners, dtype=np.float64)
    if base.ndim != 2 or base.shape[1] != 3 or len(base) < 3:
        raise ValueError('polygon requires at least three 3D corners')
    if height <= 0.0:
        raise ValueError('polygon extrusion height must be positive')
    top = base + np.array([0.0, 0.0, float(height)])
    count = len(base)
    faces = [tuple(range(count - 1, -1, -1)),
             tuple(range(count, 2 * count))]
    faces.extend((i, (i + 1) % count, (i + 1) % count + count, i + count)
                 for i in range(count))
    return np.vstack([base, top]), faces


def voxel_density_filter(points: np.ndarray, voxel: float = 0.2,
                         min_count: int = 4) -> np.ndarray:
    """Drop isolated noise points: keep only those in voxels holding at least
    ``min_count`` points. Pure numpy (voxel-hash histogram); cleans the sparse
    floaters a SLAM map accumulates without touching dense real surfaces.

    Returns the boolean keep-mask over ``points``.
    """
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) == 0:
        return np.zeros(0, dtype=bool)
    keys = np.floor(pts / voxel).astype(np.int64)
    # Pack the 3 voxel indices into one hashable key via a 1-D view.
    uniq, inv, counts = np.unique(keys, axis=0, return_inverse=True,
                                  return_counts=True)
    return counts[inv.ravel()] >= min_count


def _binary_dilate(grid: np.ndarray) -> np.ndarray:
    """4-connected binary dilation of a 2-D boolean grid."""
    out = grid.copy()
    out[1:, :] |= grid[:-1, :]
    out[:-1, :] |= grid[1:, :]
    out[:, 1:] |= grid[:, :-1]
    out[:, :-1] |= grid[:, 1:]
    return out


def _binary_erode(grid: np.ndarray) -> np.ndarray:
    """4-connected binary erosion of a 2-D boolean grid (border counts empty)."""
    out = grid.copy()
    out[1:, :] &= grid[:-1, :]
    out[:-1, :] &= grid[1:, :]
    out[:, 1:] &= grid[:, :-1]
    out[:, :-1] &= grid[:, 1:]
    return out


def _binary_close(grid: np.ndarray, iters: int) -> np.ndarray:
    """Morphological closing: ``iters`` dilations then ``iters`` erosions.
    Fills gaps up to ``iters`` cells wide (patchy scan coverage) while leaving
    larger real openings intact."""
    g = grid
    for _ in range(iters):
        g = _binary_dilate(g)
    for _ in range(iters):
        g = _binary_erode(g)
    return g


def _wall_axes(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """In-plane ``(horiz, vert)`` axes for a wall: ``vert`` is the basis vector
    most aligned with world +z (oriented upward), ``horiz`` the other."""
    u, v = plane_basis(normal)
    vert, horiz = (u, v) if abs(u[2]) >= abs(v[2]) else (v, u)
    if vert[2] < 0:
        vert = -vert
    return horiz, vert


def detect_openings(points: np.ndarray, normal: np.ndarray, *, cell: float = 0.15,
                    min_width: float = 0.6, min_height: float = 1.2,
                    door_min_height: float = 1.8, fill_ratio: float = 0.55,
                    occ_min: int = 1, close_iter: int = 0) -> list[dict]:
    """Find rectangular openings (doors / windows) punched through a wall plane.

    Projects the wall inliers onto ``(horiz, vert)`` plane axes, rasterises an
    occupancy grid, and returns the empty connected regions that are enclosed by
    wall on their two sides and top (so ragged scan edges, which touch the grid
    border, are ignored). A region open at the wall base is a ``'door'``, a
    fully-enclosed one a ``'window'``. Each result dict has ``kind``, ``corners``
    (4,3), ``size`` (width, height) and ``sill`` (height of the lower edge above
    the wall base). ``fill_ratio`` rejects non-rectangular blobs (a real opening
    fills most of its bounding box).
    """
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) < 3:
        return []
    horiz, vert = _wall_axes(normal)
    centroid = pts.mean(axis=0)
    ph = (pts - centroid).dot(horiz)
    pv = (pts - centroid).dot(vert)
    hmin, hmax = ph.min(), ph.max()
    vmin, vmax = pv.min(), pv.max()
    nh = max(1, int(np.ceil((hmax - hmin) / cell)))
    nv = max(1, int(np.ceil((vmax - vmin) / cell)))
    occ = np.zeros((nh, nv), dtype=np.int32)
    ih = np.clip(((ph - hmin) / cell).astype(int), 0, nh - 1)
    iv = np.clip(((pv - vmin) / cell).astype(int), 0, nv - 1)
    np.add.at(occ, (ih, iv), 1)
    occupied = occ >= occ_min
    if close_iter > 0:
        occupied = _binary_close(occupied, close_iter)
    empty = ~occupied

    seen = np.zeros_like(empty, dtype=bool)
    openings: list[dict] = []
    for si in range(nh):
        for sj in range(nv):
            if not empty[si, sj] or seen[si, sj]:
                continue
            comp = []
            stack = [(si, sj)]
            seen[si, sj] = True
            touch = {'left': False, 'right': False, 'top': False, 'bottom': False}
            while stack:
                i, j = stack.pop()
                comp.append((i, j))
                if i == 0:
                    touch['left'] = True
                if i == nh - 1:
                    touch['right'] = True
                if j == nv - 1:
                    touch['top'] = True
                if j == 0:
                    touch['bottom'] = True
                for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                    if 0 <= a < nh and 0 <= b < nv and empty[a, b] and not seen[a, b]:
                        seen[a, b] = True
                        stack.append((a, b))
            # Enclosed on both sides and the top -> candidate opening.
            if touch['left'] or touch['right'] or touch['top']:
                continue
            ci = np.array([c[0] for c in comp])
            cj = np.array([c[1] for c in comp])
            i0, i1, j0, j1 = ci.min(), ci.max(), cj.min(), cj.max()
            width = (i1 - i0 + 1) * cell
            height = (j1 - j0 + 1) * cell
            bbox_cells = (i1 - i0 + 1) * (j1 - j0 + 1)
            if width < min_width or height < min_height:
                continue
            if len(comp) < fill_ratio * bbox_cells:
                continue
            kind = 'door' if (touch['bottom'] and height >= door_min_height) \
                else 'window'
            h0 = hmin + i0 * cell
            h1 = hmin + (i1 + 1) * cell
            v0 = vmin + j0 * cell
            v1 = vmin + (j1 + 1) * cell
            corners = np.array([
                centroid + h0 * horiz + v0 * vert,
                centroid + h1 * horiz + v0 * vert,
                centroid + h1 * horiz + v1 * vert,
                centroid + h0 * horiz + v1 * vert,
            ])
            openings.append({
                'kind': kind,
                'corners': corners,
                'size': (float(width), float(height)),
                'sill': float(v0 - vmin),
            })
    return openings


def largest_plane_patch(points: np.ndarray, normal: np.ndarray,
                        cell: float = 1.0) -> np.ndarray:
    """Boolean mask selecting the largest spatially-contiguous cluster of
    ``points`` within their plane.

    Projects the points onto the plane basis, buckets them into a ``cell``-metre
    grid, and returns the points falling in the largest 4-connected component of
    occupied cells. This rejects the spurious "slice through 3D clutter" planes
    that raw RANSAC inlier counts favour on large outdoor scenes — a real floor
    or facade is one contiguous patch, a horizontal slice through foliage is not.
    """
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) == 0:
        return np.zeros(0, dtype=bool)
    u, v = plane_basis(normal)
    ci = np.floor(pts.dot(u) / cell).astype(np.int64)
    cj = np.floor(pts.dot(v) / cell).astype(np.int64)
    cells: dict[tuple[int, int], list[int]] = {}
    for k in range(len(pts)):
        cells.setdefault((int(ci[k]), int(cj[k])), []).append(k)
    occupied = set(cells.keys())
    seen: set[tuple[int, int]] = set()
    best: list[tuple[int, int]] = []
    for start in occupied:
        if start in seen:
            continue
        comp = []
        stack = [start]
        seen.add(start)
        while stack:
            c = stack.pop()
            comp.append(c)
            i0, j0 = c
            for nb in ((i0 + 1, j0), (i0 - 1, j0), (i0, j0 + 1), (i0, j0 - 1)):
                if nb in occupied and nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
        if len(comp) > len(best):
            best = comp
    mask = np.zeros(len(pts), dtype=bool)
    for c in best:
        mask[cells[c]] = True
    return mask


def extract_planes(points: np.ndarray, *, colors: Optional[np.ndarray] = None,
                   threshold: float = 0.1, max_planes: int = 12,
                   min_inliers: int = 300, min_remaining: int = 500,
                   iterations: int = 120, patch_cell: float = 1.0,
                   min_patch: Optional[int] = None, find_openings: bool = False,
                   opening_cell: float = 0.15, opening_close: int = 0,
                   orient: Optional[str] = None, seed: int = 0) -> list[dict]:
    """Peel dominant planes out of ``points`` by sequential RANSAC.

    Each RANSAC fit is reduced to its largest contiguous patch
    (:func:`largest_plane_patch`); only patches of at least ``min_patch`` points
    (default ``min_inliers``) are recorded as planes and removed. Spurious
    scattered slices are discarded so the loop keeps making progress without
    logging them. Returns a list of dicts: ``normal``, ``d``, ``indices`` (into
    the original array), ``centroid``, ``kind`` (from :func:`classify_plane`),
    ``corners``, ``size`` and ``thickness``, roughly largest-first. When
    ``colors`` (a matching ``(N,3)`` uint8 array) is given, each plane also gets
    a ``color`` = mean RGB of its inliers (uint8), carried into the IFC style.
    """
    pts = np.asarray(points, dtype=np.float64)
    rgb = None if colors is None else np.asarray(colors)
    min_patch = min_inliers if min_patch is None else min_patch
    remaining = np.arange(len(pts))
    planes: list[dict] = []
    for _ in range(max_planes * 3):
        if len(planes) >= max_planes:
            break
        if len(remaining) < max(min_remaining, min_inliers):
            break
        res = fit_plane_ransac(pts[remaining], threshold=threshold,
                               iterations=iterations, min_inliers=min_inliers,
                               orient=orient, seed=seed + len(planes))
        if res is None:
            break
        normal, d, mask = res
        inlier_pos = np.where(mask)[0]                 # positions into remaining
        global_idx = remaining[inlier_pos]
        patch = largest_plane_patch(pts[global_idx], normal, patch_cell)
        if int(patch.sum()) >= min_patch:
            patch_idx = global_idx[patch]
            inliers = pts[patch_idx]
            normal, d = refit_plane(inliers)           # tighten to the real patch
            corners, size, thickness = oriented_rectangle(inliers, normal)
            plane = {
                'normal': normal,
                'd': d,
                'indices': patch_idx,
                'centroid': inliers.mean(axis=0),
                'kind': classify_plane(normal),
                'corners': corners,
                'size': size,
                'thickness': thickness,
            }
            if rgb is not None:
                plane['color'] = rgb[patch_idx].mean(axis=0).round().astype(
                    np.uint8)
            if find_openings and plane['kind'] == 'vertical':
                plane['openings'] = detect_openings(
                    inliers, normal, cell=opening_cell,
                    close_iter=opening_close)
            planes.append(plane)
            keep = np.ones(len(remaining), dtype=bool)
            keep[inlier_pos[patch]] = False
            remaining = remaining[keep]
        else:
            # Spurious slice: drop its inliers to progress, but do not record it.
            remaining = remaining[~mask]
    return planes


def extract_building(points: np.ndarray, *, colors: Optional[np.ndarray] = None,
                     threshold: float = 0.1, floor_planes: int = 6,
                     wall_planes: int = 10, min_inliers: int = 300,
                     patch_cell: float = 0.5, find_openings: bool = False,
                     opening_cell: float = 0.15, opening_close: int = 0,
                     seed: int = 0) -> list[dict]:
    """Extract building elements with a separate horizontal and vertical pass.

    Raw sequential RANSAC ranks planes by inlier count, so the many horizontal
    slabs of a multi-storey scan crowd out the walls. This runs a horizontal
    pass first, *removes* those slab points, then runs an orientation-targeted
    vertical pass on the remainder — where the walls are no longer a rare
    minority, so RANSAC finds them. Returns the merged list (slabs then walls).
    """
    pts = np.asarray(points, dtype=np.float64)
    rgb = None if colors is None else np.asarray(colors)
    floors = extract_planes(pts, colors=rgb, threshold=threshold,
                            max_planes=floor_planes, min_inliers=min_inliers,
                            patch_cell=patch_cell, orient='horizontal', seed=seed)
    used = np.concatenate([f['indices'] for f in floors]) if floors \
        else np.array([], dtype=int)
    keep = np.ones(len(pts), dtype=bool)
    keep[used] = False
    rem_idx = np.where(keep)[0]
    if len(rem_idx) < min_inliers:
        return floors
    rem_rgb = None if rgb is None else rgb[rem_idx]
    walls = extract_planes(pts[rem_idx], colors=rem_rgb, threshold=threshold,
                           max_planes=wall_planes, min_inliers=min_inliers,
                           patch_cell=patch_cell, orient='vertical',
                           find_openings=find_openings, opening_cell=opening_cell,
                           opening_close=opening_close, seed=seed + 1000)
    for w in walls:                          # remap local indices to the original
        w['indices'] = rem_idx[w['indices']]
    return floors + walls


def principal_axis(walls: list[dict], up=(0.0, 0.0, 1.0)) -> float:
    """Dominant horizontal wall direction (radians in ``[0, pi/2)``).

    Manhattan walls sit at 0/90/180/270 degrees to a common building axis, so
    reducing each wall's horizontal-normal angle modulo 90 degrees collapses
    them onto that axis; a point-count-weighted circular mean (period pi/2)
    returns it robustly.
    """
    up = np.asarray(up, dtype=np.float64)
    up = up / (np.linalg.norm(up) + 1e-12)
    ang4 = []
    wts = []
    for w in walls:
        n = np.asarray(w['normal'], dtype=np.float64)
        h = n - n.dot(up) * up            # horizontal component of the normal
        if np.linalg.norm(h) < 1e-6:
            continue
        theta = np.arctan2(h[1], h[0]) % (np.pi / 2.0)
        ang4.append(theta * 4.0)          # map period pi/2 -> 2pi for averaging
        wts.append(float(len(w.get('indices', [1]))))
    if not ang4:
        return 0.0
    ang4 = np.asarray(ang4)
    wts = np.asarray(wts)
    mean = np.arctan2((wts * np.sin(ang4)).sum(),
                      (wts * np.cos(ang4)).sum()) / 4.0
    return float(mean % (np.pi / 2.0))


def regularize_walls(planes: list[dict], points: np.ndarray,
                     up=(0.0, 0.0, 1.0), *, adaptive: bool = False) -> list[dict]:
    """Snap wall normals to the building's Manhattan grid (0/90 degrees to
    :func:`principal_axis`) and re-fit each wall's plane + rectangle to the
    snapped, perfectly-vertical normal. Slabs pass through untouched. Returns a
    new plane list; ``points`` supplies each wall's inliers (via ``indices``).
    """
    pts = np.asarray(points, dtype=np.float64)
    up = np.asarray(up, dtype=np.float64)
    up = up / (np.linalg.norm(up) + 1e-12)
    walls = [p for p in planes if p['kind'] == 'vertical']
    if not walls:
        return list(planes)
    theta0 = principal_axis(walls, up)
    out = []
    for p in planes:
        if p['kind'] != 'vertical':
            out.append(p)
            continue
        n = np.asarray(p['normal'], dtype=np.float64)
        theta = np.arctan2(n[1], n[0])
        k = round((theta - theta0) / (np.pi / 2.0))
        theta_s = theta0 + k * (np.pi / 2.0)
        n_s = np.array([np.cos(theta_s), np.sin(theta_s), 0.0])
        if n_s.dot([n[0], n[1], 0.0]) < 0:      # keep the original facing
            n_s = -n_s
        inliers = pts[p['indices']]
        observed_n = np.asarray(p['normal'], dtype=np.float64)
        observed_d = float(p['d'])
        observed_rmse = float(np.sqrt(np.mean(
            (inliers.dot(observed_n) + observed_d) ** 2)))

        def candidate(candidate_n):
            candidate_n = candidate_n / (np.linalg.norm(candidate_n) + 1e-12)
            candidate_d = float(-inliers.mean(axis=0).dot(candidate_n))
            candidate_rmse = float(np.sqrt(np.mean(
                (inliers.dot(candidate_n) + candidate_d) ** 2)))
            return candidate_n, candidate_d, candidate_rmse

        full_n, full_d, full_rmse = candidate(n_s)
        chosen_n, d_s, chosen_rmse, decision = full_n, full_d, full_rmse, 'Full snap'
        if adaptive:
            # Half-angle blend on the horizontal unit circle.
            blend = observed_n.copy(); blend[2] = 0.0
            blend /= np.linalg.norm(blend) + 1e-12
            if blend.dot(full_n) < 0:
                blend = -blend
            soft_n, soft_d, soft_rmse = candidate(blend + full_n)
            full_limit = max(0.08, observed_rmse * 2.0)
            soft_limit = max(0.12, observed_rmse * 2.5)
            if full_rmse <= full_limit:
                decision = 'Full snap'
            elif soft_rmse <= soft_limit:
                chosen_n, d_s, chosen_rmse, decision = (
                    soft_n, soft_d, soft_rmse, 'Soft snap')
            else:
                chosen_n, d_s, chosen_rmse, decision = (
                    observed_n, observed_d, observed_rmse, 'Keep observed')
        corners, size, thickness = oriented_rectangle(inliers, chosen_n)
        q = dict(p)
        q['observed_normal'] = observed_n.copy()
        q['observed_d'] = observed_d
        observed_angle = float(np.degrees(np.arctan2(observed_n[1], observed_n[0])))
        adopted_angle = float(np.degrees(np.arctan2(chosen_n[1], chosen_n[0])))
        angle_change = abs((adopted_angle - observed_angle + 90.0) % 180.0 - 90.0)
        q['regularization'] = {
            'decision': decision, 'observed_angle_deg': observed_angle,
            'adopted_angle_deg': adopted_angle, 'angle_change_deg': angle_change,
            'observed_rmse_m': observed_rmse, 'adopted_rmse_m': chosen_rmse,
            'full_snap_rmse_m': full_rmse,
        }
        q.update(normal=chosen_n, d=d_s, corners=corners, size=size,
                 thickness=thickness)
        out.append(q)
    return out


def _wall_footprint_segment(wall: dict) -> tuple[np.ndarray, np.ndarray]:
    """The two horizontal end-points (xy) of a vertical wall's rectangle."""
    xy = np.asarray(wall['corners'], dtype=np.float64)[:, :2]
    best = (0, 1)
    bd = -1.0
    for i in range(4):
        for j in range(i + 1, 4):
            d = np.linalg.norm(xy[i] - xy[j])
            if d > bd:
                bd = d
                best = (i, j)
    return xy[best[0]], xy[best[1]]


def _line_intersection(a: np.ndarray, b: np.ndarray, c: np.ndarray,
                       d: np.ndarray) -> Optional[tuple[np.ndarray, float, float]]:
    """Return infinite-line intersection and parameters on ``ab`` / ``cd``."""
    u, v = b - a, d - c
    cross = float(u[0] * v[1] - u[1] * v[0])
    if abs(cross) < 1e-12:
        return None
    ca = c - a
    t = float((ca[0] * v[1] - ca[1] * v[0]) / cross)
    s = float((ca[0] * u[1] - ca[1] * u[0]) / cross)
    return a + t * u, t, s


def build_wall_graph(planes: list[dict], *, intersection_gap: float = 0.5,
                     merge_distance: float = 0.10,
                     min_intersection_angle_deg: float = 15.0) -> dict:
    """Build a planar graph from wall endpoints and supporting-line crossings.

    Intersections may extend each observed segment by at most
    ``intersection_gap``. Every wall is split at accepted intersections, so
    T-junctions become real graph nodes and adjacent rooms can share an edge.
    Edges retain their observed/extended fraction for topology scoring.
    """
    if intersection_gap < 0.0 or merge_distance < 0.0:
        raise ValueError('graph distances must be non-negative')
    walls = [p for p in planes if p['kind'] == 'vertical']
    segments = [_wall_footprint_segment(w) for w in walls]
    candidates = [[(0.0, a.copy()), (1.0, b.copy())] for a, b in segments]
    min_sine = float(np.sin(np.deg2rad(min_intersection_angle_deg)))

    def extension_distance(parameter, length):
        return max(-parameter, 0.0, parameter - 1.0) * length

    for i, (a, b) in enumerate(segments):
        u = b - a
        lu = float(np.linalg.norm(u))
        if lu < 1e-9:
            continue
        for j in range(i + 1, len(segments)):
            c, d = segments[j]
            v = d - c
            lv = float(np.linalg.norm(v))
            if lv < 1e-9:
                continue
            sine = abs(float(u[0] * v[1] - u[1] * v[0])) / (lu * lv)
            if sine < min_sine:
                continue
            result = _line_intersection(a, b, c, d)
            if result is None:
                continue
            point, ti, tj = result
            if (extension_distance(ti, lu) <= intersection_gap and
                    extension_distance(tj, lv) <= intersection_gap):
                candidates[i].append((ti, point.copy()))
                candidates[j].append((tj, point.copy()))

    nodes: list[np.ndarray] = []

    def node_id(point):
        if nodes:
            distances = np.linalg.norm(np.asarray(nodes) - point, axis=1)
            closest = int(np.argmin(distances))
            if distances[closest] <= merge_distance:
                # Keep graph construction deterministic while reducing endpoint
                # noise: progressively average clustered observations.
                nodes[closest] = (nodes[closest] + point) / 2.0
                return closest
        nodes.append(point.copy())
        return len(nodes) - 1

    edge_map: dict[tuple[int, int], dict] = {}
    for wall_index, ((a, b), wall, wall_candidates) in enumerate(
            zip(segments, walls, candidates)):
        length = float(np.linalg.norm(b - a))
        observed_lo, observed_hi = 0.0, 1.0
        adjustment = wall.get('topology_adjustment', {})
        if adjustment.get('observed_footprint_xy') and length > 1e-9:
            observed_ends = np.asarray(adjustment['observed_footprint_xy'],
                                       dtype=np.float64)
            direction = (b - a) / length
            observed_parameters = (observed_ends - a).dot(direction) / length
            observed_lo, observed_hi = (float(observed_parameters.min()),
                                         float(observed_parameters.max()))
        ordered = sorted(wall_candidates, key=lambda item: item[0])
        compact = []
        for parameter, point in ordered:
            if compact and np.linalg.norm(point - compact[-1][1]) <= 1e-9:
                continue
            compact.append((parameter, point))
        for (t0, p0), (t1, p1) in zip(compact, compact[1:]):
            edge_length = float(np.linalg.norm(p1 - p0))
            if edge_length <= 1e-9:
                continue
            n0, n1 = node_id(p0), node_id(p1)
            if n0 == n1:
                continue
            observed_parameter = max(
                0.0, min(t1, observed_hi) - max(t0, observed_lo))
            observed_length = (0.0 if wall.get('synthetic') else
                               min(edge_length, observed_parameter * length))
            edge = {
                'nodes': (n0, n1), 'wall_index': wall_index,
                'length': edge_length,
                'observed_fraction': float(observed_length / edge_length),
                'extension_length': float(max(0.0, edge_length - observed_length)),
                'synthetic_wall': bool(wall.get('synthetic')),
                'confidence': float(wall.get('confidence', 0.0)),
            }
            key = tuple(sorted((n0, n1)))
            previous = edge_map.get(key)
            if previous is None or edge['observed_fraction'] > previous['observed_fraction']:
                edge_map[key] = edge

    z_values = [np.asarray(w['corners'])[:, 2] for w in walls]
    return {
        'nodes': np.asarray(nodes, dtype=np.float64).reshape((-1, 2)),
        'edges': list(edge_map.values()),
        'floor_z': float(min(z.min() for z in z_values)) if z_values else 0.0,
        'ceiling_z': float(max(z.max() for z in z_values)) if z_values else 0.0,
    }


def extract_room_cycles(graph: dict, *, min_area: float = 2.0) -> list[dict]:
    """Extract bounded planar faces from a wall graph as room candidates.

    A half-edge face walk returns each elementary bounded cycle once. The
    exterior face has clockwise winding and is discarded. Candidates retain
    boundary observation and extension measurements for later acceptance or
    repair decisions.
    """
    nodes = np.asarray(graph.get('nodes', []), dtype=np.float64).reshape((-1, 2))
    edges = graph.get('edges', [])
    adjacency: dict[int, list[int]] = {i: [] for i in range(len(nodes))}
    edge_lookup = {}
    for edge in edges:
        a, b = edge['nodes']
        if b not in adjacency[a]: adjacency[a].append(b)
        if a not in adjacency[b]: adjacency[b].append(a)
        edge_lookup[tuple(sorted((a, b)))] = edge
    for node, neighbours in adjacency.items():
        neighbours.sort(key=lambda other: float(np.arctan2(
            nodes[other, 1] - nodes[node, 1], nodes[other, 0] - nodes[node, 0])))

    visited = set()
    cycles = []
    max_steps = max(1, len(edges) * 2 + 1)
    for edge in edges:
        for start in (edge['nodes'], edge['nodes'][::-1]):
            start = tuple(start)
            if start in visited:
                continue
            directed = start
            node_ids = []
            for _ in range(max_steps):
                if directed in visited and directed != start:
                    node_ids = []
                    break
                visited.add(directed)
                u, v = directed
                node_ids.append(u)
                neighbours = adjacency.get(v, [])
                if len(neighbours) < 1 or u not in neighbours:
                    node_ids = []
                    break
                # Clockwise from the reverse edge keeps the traversed face on
                # the left. Positive signed area is therefore a bounded face.
                position = neighbours.index(u)
                directed = (v, neighbours[(position - 1) % len(neighbours)])
                if directed == start:
                    break
            else:
                node_ids = []
            if len(node_ids) < 3 or directed != start:
                continue
            polygon = nodes[node_ids]
            signed_area = 0.5 * float(np.sum(
                polygon[:, 0] * np.roll(polygon[:, 1], -1) -
                polygon[:, 1] * np.roll(polygon[:, 0], -1)))
            if signed_area < min_area:
                continue
            boundary = [edge_lookup[tuple(sorted((node_ids[i],
                        node_ids[(i + 1) % len(node_ids)])))]
                        for i in range(len(node_ids))]
            perimeter = sum(e['length'] for e in boundary)
            observed = sum(e['length'] * e['observed_fraction'] for e in boundary)
            cycles.append({
                'node_ids': tuple(node_ids), 'polygon_xy': polygon.copy(),
                'area': signed_area, 'perimeter': float(perimeter),
                'observed_boundary_ratio': float(observed / max(perimeter, 1e-12)),
                'extension_length': float(sum(e['extension_length'] for e in boundary)),
                'synthetic_edge_count': int(sum(e['synthetic_wall'] for e in boundary)),
                'floor_z': float(graph.get('floor_z', 0.0)),
                'height': float(graph.get('ceiling_z', 0.0) - graph.get('floor_z', 0.0)),
            })
    return sorted(cycles, key=lambda cycle: cycle['area'], reverse=True)


def rooms_from_wall_cycles(cycles: list[dict]) -> list[dict]:
    """Convert accepted wall-graph faces into polygonal room candidates."""
    rooms = []
    for cycle in cycles:
        polygon = np.asarray(cycle['polygon_xy'], dtype=np.float64)
        floor_z = float(cycle.get('floor_z', 0.0))
        corners = np.c_[polygon, np.full(len(polygon), floor_z)]
        rooms.append({
            'corners': corners,
            'height': float(cycle.get('height', 0.0)),
            'area': float(cycle['area']),
            'name': f'Room {len(rooms) + 1}',
            'number': str(len(rooms) + 1),
            'generation_method': 'LiDAR wall topology cycle',
            'topology_metrics': {
                'observed_boundary_ratio': float(cycle['observed_boundary_ratio']),
                'extension_length_m': float(cycle['extension_length']),
                'synthetic_edge_count': int(cycle['synthetic_edge_count']),
                'vertices': int(len(polygon)),
            },
        })
    return rooms


def derive_room_candidates(planes: list[dict], *, room_close: int = 3,
                           min_area: float = 2.0,
                           min_observed_boundary: float = 0.85) -> list[dict]:
    """Prefer evidence-backed polygonal graph faces, with raster fallback."""
    graph = build_wall_graph(planes, intersection_gap=0.0)
    cycles = [cycle for cycle in extract_room_cycles(graph, min_area=min_area)
              if cycle['observed_boundary_ratio'] >= min_observed_boundary]
    if cycles:
        return rooms_from_wall_cycles(cycles)
    return reconstruct_rooms(planes, close_iter=room_close)


def _apply_topology_intersections(planes: list[dict], graph: dict,
                                  cycles: list[dict]) -> list[dict]:
    """Extend observed wall rectangles only along accepted cycle boundaries."""
    nodes = np.asarray(graph['nodes'])
    cycle_edges = set()
    cycle_count_by_wall: dict[int, int] = {}
    edge_by_key = {tuple(sorted(edge['nodes'])): edge for edge in graph['edges']}
    for cycle in cycles:
        ids = cycle['node_ids']
        keys = {tuple(sorted((ids[i], ids[(i + 1) % len(ids)])))
                for i in range(len(ids))}
        cycle_edges.update(keys)
        for wall_index in {edge_by_key[key]['wall_index'] for key in keys}:
            cycle_count_by_wall[wall_index] = cycle_count_by_wall.get(
                wall_index, 0) + 1
    selected: dict[int, list[np.ndarray]] = {}
    for edge in graph['edges']:
        if tuple(sorted(edge['nodes'])) not in cycle_edges:
            continue
        selected.setdefault(edge['wall_index'], []).extend(
            [nodes[edge['nodes'][0]], nodes[edge['nodes'][1]]])

    out = [dict(plane) for plane in planes]
    plane_ids = [i for i, plane in enumerate(out) if plane['kind'] == 'vertical']
    for wall_index, points in selected.items():
        plane_id = plane_ids[wall_index]
        wall = out[plane_id]
        if wall.get('synthetic'):
            continue
        a, b = _wall_footprint_segment(wall)
        delta = b - a
        length = float(np.linalg.norm(delta))
        if length < 1e-9:
            continue
        direction = delta / length
        parameters = [float((point - a).dot(direction)) for point in points]
        lo, hi = min(0.0, min(parameters)), max(length, max(parameters))
        p0, p1 = a + direction * lo, a + direction * hi
        extension = -lo + (hi - length)
        if extension <= 1e-9:
            continue
        old = np.asarray(wall['corners'], dtype=np.float64)
        z0, z1 = float(old[:, 2].min()), float(old[:, 2].max())
        corners = np.array([[p0[0], p0[1], z0], [p1[0], p1[1], z0],
                            [p1[0], p1[1], z1], [p0[0], p0[1], z1]])
        wall['corners'] = corners
        wall['centroid'] = corners.mean(axis=0)
        wall['size'] = (float(np.linalg.norm(p1 - p0)), float(z1 - z0))
        wall['topology_adjustment'] = {
            'decision': 'Extend to accepted room-cycle intersection',
            'extension_length_m': float(extension),
            'observed_footprint_xy': [a.tolist(), b.tolist()],
            'accepted_cycle_count': int(cycle_count_by_wall.get(wall_index, 0)),
        }
    return out


def optimize_wall_topology(planes: list[dict], *, intersection_gap: float = 0.5,
                           repair_gap: float = 0.75,
                           min_room_area: float = 2.0,
                           min_observed_boundary: float = 0.85
                           ) -> tuple[list[dict], list[dict]]:
    """Condition wall extension and repair on evidence-backed room cycles.

    Supporting-line intersections are adopted only when they participate in a
    sufficiently observed bounded face. Short synthetic repairs are considered
    one at a time and retained only when they create an additional acceptable
    room face. This prevents generic nearest-end joining from closing doors or
    construction openings without topological evidence.
    """
    graph = build_wall_graph(planes, intersection_gap=intersection_gap)
    cycles = [cycle for cycle in extract_room_cycles(graph, min_area=min_room_area)
              if cycle['observed_boundary_ratio'] >= min_observed_boundary]
    out = _apply_topology_intersections(planes, graph, cycles)

    accepted_count = len(cycles)
    for repair in propose_wall_repairs(out, max_gap=repair_gap):
        trial = out + [repair]
        trial_graph = build_wall_graph(trial, intersection_gap=intersection_gap)
        trial_cycles = [
            cycle for cycle in extract_room_cycles(trial_graph, min_area=min_room_area)
            if cycle['observed_boundary_ratio'] >= min_observed_boundary]
        if len(trial_cycles) <= accepted_count:
            continue
        repair = dict(repair)
        repair['topology_validation'] = {
            'decision': 'Accepted: creates evidence-backed room cycle',
            'room_cycles_before': accepted_count,
            'room_cycles_after': len(trial_cycles),
            'min_observed_boundary': float(min_observed_boundary),
        }
        out.append(repair)
        cycles = trial_cycles
        accepted_count = len(cycles)
    return out, cycles


def snap_wall_corners(planes: list[dict], *, max_gap: float = 0.5,
                      max_parallel_dot: float = 0.35) -> list[dict]:
    """Extend nearby wall ends to a shared, well-conditioned intersection.

    Only nearly perpendicular footprint segments are considered.  Their line
    intersection replaces an end point when it lies within ``max_gap`` of an
    end of *both* walls.  This closes small gaps left by independently fitted
    rectangles without joining distant or parallel walls.
    """
    out = [dict(p) for p in planes]
    wall_ids = [i for i, p in enumerate(out) if p['kind'] == 'vertical']
    segments = {}
    for i in wall_ids:
        a, b = _wall_footprint_segment(out[i])
        segments[i] = [a.copy(), b.copy()]

    for pos, i in enumerate(wall_ids):
        for j in wall_ids[pos + 1:]:
            a, b = segments[i]
            c, d = segments[j]
            u, v = b - a, d - c
            lu, lv = np.linalg.norm(u), np.linalg.norm(v)
            if lu < 1e-9 or lv < 1e-9:
                continue
            if abs(float(u.dot(v) / (lu * lv))) > max_parallel_dot:
                continue
            cross = u[0] * v[1] - u[1] * v[0]
            if abs(cross) < 1e-9:
                continue
            ca = c - a
            t = (ca[0] * v[1] - ca[1] * v[0]) / cross
            intersection = a + t * u
            ei = int(np.argmin([np.linalg.norm(intersection - a),
                                np.linalg.norm(intersection - b)]))
            ej = int(np.argmin([np.linalg.norm(intersection - c),
                                np.linalg.norm(intersection - d)]))
            if (np.linalg.norm(intersection - segments[i][ei]) <= max_gap and
                    np.linalg.norm(intersection - segments[j][ej]) <= max_gap):
                segments[i][ei] = intersection.copy()
                segments[j][ej] = intersection.copy()

    for i in wall_ids:
        p0, p1 = segments[i]
        old = np.asarray(out[i]['corners'], dtype=np.float64)
        z0, z1 = float(old[:, 2].min()), float(old[:, 2].max())
        corners = np.array([[p0[0], p0[1], z0], [p1[0], p1[1], z0],
                            [p1[0], p1[1], z1], [p0[0], p0[1], z1]])
        out[i]['corners'] = corners
        out[i]['centroid'] = corners.mean(axis=0)
        size = out[i].get('size', (np.linalg.norm(p1 - p0), z1 - z0))
        out[i]['size'] = (float(np.linalg.norm(p1 - p0)), float(size[1]))
    return out


def propose_wall_repairs(planes: list[dict], *, min_gap: float = 0.3,
                         max_gap: float = 0.75) -> list[dict]:
    """Propose short synthetic walls between mutually-nearest dangling ends.

    Exact/already-snapped corners are ignored. Each endpoint is used at most
    once, and only when both endpoints select one another as their closest end;
    this deliberately favours a few high-confidence repairs over aggressive
    closure of real doors or unfinished construction openings.
    """
    walls = [p for p in planes if p['kind'] == 'vertical' and not p.get('synthetic')]
    ends = []
    for wi, wall in enumerate(walls):
        for point in _wall_footprint_segment(wall):
            ends.append((wi, point, wall))
    nearest = {}
    for i, (wi, point, _) in enumerate(ends):
        choices = [(np.linalg.norm(point - other), j)
                   for j, (wj, other, _) in enumerate(ends) if wi != wj]
        if choices:
            distance, j = min(choices)
            if min_gap < distance <= max_gap:
                nearest[i] = j
    repairs = []
    for i, j in nearest.items():
        if i >= j or nearest.get(j) != i:
            continue
        _, a, wa = ends[i]; _, b, wb = ends[j]
        za = np.asarray(wa['corners'])[:, 2]
        zb = np.asarray(wb['corners'])[:, 2]
        z0, z1 = max(float(za.min()), float(zb.min())), min(float(za.max()), float(zb.max()))
        if z1 <= z0:
            continue
        direction = b - a
        length = float(np.linalg.norm(direction))
        normal = np.array([-direction[1] / length, direction[0] / length, 0.0])
        corners = np.array([[a[0], a[1], z0], [b[0], b[1], z0],
                            [b[0], b[1], z1], [a[0], a[1], z1]])
        repairs.append({
            'kind': 'vertical', 'normal': normal,
            'd': float(-np.r_[((a + b) / 2.0), 0.0].dot(normal)),
            'corners': corners, 'centroid': corners.mean(axis=0),
            'size': (length, z1 - z0), 'thickness': 0.15,
            'indices': np.array([], dtype=int), 'synthetic': True,
            'repair_reason': 'mutually-nearest dangling wall ends',
        })
    return repairs


def evaluate_element_fit(plane: dict, points: np.ndarray, *,
                         cell: float = 0.25,
                         distance_threshold: float = 0.10) -> dict:
    """Measure point-to-element Coverage, Distance and Distribution.

    ``coverage_ratio`` is the fraction of rectangular surface cells containing
    at least one supporting point within ``distance_threshold`` of the model.
    ``distance_*`` measures all supporting points whose projection falls on the
    element. ``distribution_ratio`` is normalized Shannon entropy over occupied
    cells and therefore measures evenness independently from surface coverage.

    This is an observation-quality metric, not a visibility-aware survey
    certificate.  The explicit components are retained so later ray/occlusion
    information can refine coverage without changing the public report schema.
    """
    if cell <= 0.0:
        raise ValueError('cell must be positive')
    if distance_threshold <= 0.0:
        raise ValueError('distance_threshold must be positive')

    pts = np.asarray(points, dtype=np.float64)
    indices = np.asarray(plane.get('indices', []), dtype=int)
    synthetic = bool(plane.get('synthetic'))
    corners = np.asarray(plane.get('corners', []), dtype=np.float64)
    empty = {
        'coverage_ratio': 0.0,
        'distribution_ratio': 0.0,
        'distance_rmse_m': None,
        'distance_p95_m': None,
        'matched_points': 0,
        'occupied_cells': 0,
        'total_cells': 0,
        'cell_m': float(cell),
        'distance_threshold_m': float(distance_threshold),
        'visibility_model': 'unavailable',
    }
    if synthetic or len(indices) == 0 or corners.shape != (4, 3):
        return empty
    if np.any(indices < 0) or np.any(indices >= len(pts)):
        raise IndexError('plane indices are outside the point array')

    normal = np.asarray(plane['normal'], dtype=np.float64)
    normal /= np.linalg.norm(normal) + 1e-12
    u, v = plane_basis(normal)
    cu, cv = corners.dot(u), corners.dot(v)
    u0, u1 = float(cu.min()), float(cu.max())
    v0, v1 = float(cv.min()), float(cv.max())
    nu = max(1, int(np.ceil(max(u1 - u0, 1e-9) / cell)))
    nv = max(1, int(np.ceil(max(v1 - v0, 1e-9) / cell)))
    total_cells = nu * nv

    observed = pts[indices]
    ou, ov = observed.dot(u), observed.dot(v)
    tolerance = cell * 0.5
    on_rectangle = ((ou >= u0 - tolerance) & (ou <= u1 + tolerance) &
                    (ov >= v0 - tolerance) & (ov <= v1 + tolerance))
    observed = observed[on_rectangle]
    ou, ov = ou[on_rectangle], ov[on_rectangle]
    if len(observed) == 0:
        return {**empty, 'total_cells': total_cells}

    residual = np.abs(observed.dot(normal) + float(plane['d']))
    near = residual <= distance_threshold
    iu = np.clip(((ou[near] - u0) / cell).astype(int), 0, nu - 1)
    iv = np.clip(((ov[near] - v0) / cell).astype(int), 0, nv - 1)
    counts = np.zeros(total_cells, dtype=np.int64)
    if len(iu):
        np.add.at(counts, iu * nv + iv, 1)
    occupied = counts[counts > 0]
    coverage = len(occupied) / total_cells
    if len(occupied) <= 1:
        distribution = 0.0
    else:
        probabilities = occupied / occupied.sum()
        entropy = -float(np.sum(probabilities * np.log(probabilities)))
        distribution = entropy / np.log(len(occupied))
    return {
        'coverage_ratio': float(coverage),
        'distribution_ratio': float(distribution),
        'distance_rmse_m': float(np.sqrt(np.mean(residual ** 2))),
        'distance_p95_m': float(np.percentile(residual, 95)),
        'matched_points': int(len(observed)),
        'occupied_cells': int(len(occupied)),
        'total_cells': int(total_cells),
        'cell_m': float(cell),
        'distance_threshold_m': float(distance_threshold),
        'visibility_model': 'unavailable',
    }


def add_element_fit_metrics(planes: list[dict], points: np.ndarray, *,
                            cell: float = 0.25,
                            distance_threshold: float = 0.10) -> list[dict]:
    """Attach :func:`evaluate_element_fit` results to every BIM element."""
    return [dict(plane, element_fit=evaluate_element_fit(
        plane, points, cell=cell, distance_threshold=distance_threshold))
            for plane in planes]


def add_plane_confidence(planes: list[dict], points: np.ndarray) -> list[dict]:
    """Attach an explainable 0--100 observation-confidence score to planes.

    The score combines plane residual (40%), supporting point count (35%), and
    observed point density over fitted area (25%). It is a QA ranking, not a
    survey-accuracy certificate. Synthetic repairs receive a fixed low score.
    """
    pts = np.asarray(points, dtype=np.float64)
    out = []
    for plane in planes:
        q = dict(plane)
        if q.get('synthetic'):
            score, support, rmse, density = 25.0, 0, None, 0.0
            model_rmse = None
            provenance = 'inferred'
        else:
            idx = np.asarray(q.get('indices', []), dtype=int)
            observed = pts[idx] if len(idx) else np.empty((0, 3))
            support = len(observed)
            observed_n = np.asarray(q.get('observed_normal', q['normal']))
            observed_d = float(q.get('observed_d', q['d']))
            observed_residual = np.abs(observed.dot(observed_n) + observed_d)
            model_residual = np.abs(observed.dot(q['normal']) + float(q['d']))
            rmse = float(np.sqrt(np.mean(observed_residual ** 2))) if support else None
            model_rmse = float(np.sqrt(np.mean(model_residual ** 2))) if support else None
            size = q.get('size', (1.0, 1.0))
            area = max(float(size[0]) * float(size[1]), 1e-6)
            density = support / area
            support_score = 1.0 - np.exp(-support / 1000.0)
            residual_score = np.exp(-((1.0 if rmse is None else rmse) / 0.08) ** 2)
            density_score = 1.0 - np.exp(-density / 100.0)
            score = 100.0 * (0.35 * support_score + 0.40 * residual_score +
                             0.25 * density_score)
            provenance = 'observed'
        level = 'High' if score >= 75 else ('Medium' if score >= 50 else 'Low')
        q.update(confidence=float(round(score, 1)), confidence_level=level,
                 provenance=provenance, confidence_metrics={
                     'support_points': int(support),
                     'plane_rmse_m': None if rmse is None else float(rmse),
                     'observation_rmse_m': None if rmse is None else float(rmse),
                     'model_rmse_m': None if model_rmse is None else float(model_rmse),
                     'point_density_m2': float(density),
                 })
        out.append(q)
    return out


def add_wall_quality_segments(planes: list[dict], points: np.ndarray, *,
                              segment_length: float = 2.0) -> list[dict]:
    """Attach local QA measurements along each wall footprint."""
    pts = np.asarray(points, dtype=np.float64)
    out = []
    for plane in planes:
        q = dict(plane)
        if q['kind'] != 'vertical':
            out.append(q)
            continue
        a, b = _wall_footprint_segment(q)
        delta = b - a
        length = float(np.linalg.norm(delta))
        if length < 1e-9:
            q['quality_segments'] = []
            out.append(q)
            continue
        direction = delta / length
        count = max(1, int(np.ceil(length / segment_length)))
        idx = np.asarray(q.get('indices', []), dtype=int)
        observed = pts[idx] if len(idx) else np.empty((0, 3))
        along = (observed[:, :2] - a).dot(direction) if len(observed) else np.array([])
        segments = []
        for k in range(count):
            t0, t1 = length * k / count, length * (k + 1) / count
            mask = (along >= t0 - 1e-9) & (along <= t1 + (1e-9 if k == count - 1 else 0))
            local = observed[mask]
            support = len(local)
            if q.get('synthetic'):
                rmse, density, score = None, 0.0, 25.0
                model_rmse = None
            else:
                observed_n = np.asarray(q.get('observed_normal', q['normal']))
                observed_d = float(q.get('observed_d', q['d']))
                observed_residual = np.abs(local.dot(observed_n) + observed_d)
                model_residual = np.abs(local.dot(q['normal']) + float(q['d']))
                rmse = float(np.sqrt(np.mean(observed_residual ** 2))) if support else None
                model_rmse = float(np.sqrt(np.mean(model_residual ** 2))) if support else None
                height = max(float(q.get('size', (length, 1.0))[1]), 0.1)
                density = support / max((t1 - t0) * height, 1e-6)
                support_score = 1.0 - np.exp(-support / 200.0)
                residual_score = np.exp(-((1.0 if rmse is None else rmse) / 0.08) ** 2)
                density_score = 1.0 - np.exp(-density / 100.0)
                score = 100.0 * (0.30 * support_score + 0.50 * residual_score +
                                 0.20 * density_score)
            level = 'High' if score >= 75 else ('Medium' if score >= 50 else 'Low')
            if support < 50:
                recommendation = '再計測（観測点不足）'
            elif rmse is not None and rmse > 0.15:
                recommendation = 'SLAMドリフト確認'
            elif model_rmse is not None and model_rmse > 0.15:
                recommendation = '正規化を緩和・局所再フィット'
            elif level == 'Low':
                recommendation = '追加計測または再フィット'
            else:
                recommendation = '利用可能'
            segments.append({
                'start': a + direction * t0, 'end': a + direction * t1,
                'length': t1 - t0, 'support_points': support,
                'plane_rmse_m': rmse, 'point_density_m2': float(density),
                'observation_rmse_m': rmse, 'model_rmse_m': model_rmse,
                'score': float(round(score, 1)), 'level': level,
                'recommendation': recommendation,
            })
        q['quality_segments'] = segments
        out.append(q)
    return out


def reconstruct_rooms(planes: list[dict], *, cell: float = 0.2,
                      close_iter: int = 3, min_cells: int = 25,
                      up=(0.0, 0.0, 1.0)) -> list[dict]:
    """Reconstruct enclosed rooms from wall footprints.

    Rasterises the wall segments into a floor-plan grid, morphologically closes
    it to bridge doorways / scan gaps, then returns every empty connected region
    that does **not** touch the plan border — an interior room. Each result has
    ``corners`` (4,3 footprint at the wall base), ``height`` and ``area``. Same
    enclosed-region idea as :func:`detect_openings`, applied to the plan.
    """
    up = np.asarray(up, dtype=np.float64)
    up = up / (np.linalg.norm(up) + 1e-12)
    walls = [p for p in planes if p['kind'] == 'vertical']
    if not walls:
        return []
    corners_xy = np.vstack([np.asarray(w['corners'])[:, :2] for w in walls])
    zs = np.concatenate([np.asarray(w['corners'])[:, 2] for w in walls])
    z_floor, z_ceiling = float(zs.min()), float(zs.max())
    pad = cell * 3.0
    xmin, ymin = corners_xy.min(axis=0) - pad
    xmax, ymax = corners_xy.max(axis=0) + pad
    nx = int(np.ceil((xmax - xmin) / cell)) + 1
    ny = int(np.ceil((ymax - ymin) / cell)) + 1
    grid = np.zeros((nx, ny), dtype=bool)
    for w in walls:
        p0, p1 = _wall_footprint_segment(w)
        steps = int(np.linalg.norm(p1 - p0) / cell * 2.0) + 2
        for t in np.linspace(0.0, 1.0, steps):
            p = p0 + (p1 - p0) * t
            i = int((p[0] - xmin) / cell)
            j = int((p[1] - ymin) / cell)
            if 0 <= i < nx and 0 <= j < ny:
                grid[i, j] = True
    # Dilate (not full closing) to seal doorways/scan gaps: for room detection we
    # only need the walls to form a closed barrier — erosion could re-sever a
    # freshly-bridged thin wall line, so keep the thickened walls.
    for _ in range(close_iter):
        grid = _binary_dilate(grid)
    empty = ~grid

    seen = np.zeros_like(empty)
    rooms: list[dict] = []
    for si in range(nx):
        for sj in range(ny):
            if not empty[si, sj] or seen[si, sj]:
                continue
            comp = []
            stack = [(si, sj)]
            seen[si, sj] = True
            border = False
            while stack:
                i, j = stack.pop()
                comp.append((i, j))
                if i in (0, nx - 1) or j in (0, ny - 1):
                    border = True
                for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                    if 0 <= a < nx and 0 <= b < ny and empty[a, b] \
                            and not seen[a, b]:
                        seen[a, b] = True
                        stack.append((a, b))
            if border or len(comp) < min_cells:
                continue                       # exterior, or too small
            ci = np.array([c[0] for c in comp])
            cj = np.array([c[1] for c in comp])
            x0 = xmin + ci.min() * cell
            x1 = xmin + (ci.max() + 1) * cell
            y0 = ymin + cj.min() * cell
            y1 = ymin + (cj.max() + 1) * cell
            corners = np.array([
                [x0, y0, z_floor], [x1, y0, z_floor],
                [x1, y1, z_floor], [x0, y1, z_floor],
            ])
            rooms.append({
                'corners': corners,
                'height': z_ceiling - z_floor,
                'area': len(comp) * cell * cell,
                'name': f'Room {len(rooms) + 1}',
                'number': str(len(rooms) + 1),
                'generation_method': 'LiDAR wall enclosure',
            })
    return rooms


def assess_rooms(rooms: list[dict], planes: list[dict], *,
                 boundary_tolerance: float = 0.4) -> list[dict]:
    """Score reconstructed spaces and classify Confirmed/Candidate/Rejected."""
    walls = [p for p in planes if p['kind'] == 'vertical']
    real_walls = [p for p in walls if not p.get('synthetic')]
    stable_rooms = reconstruct_rooms(real_walls) if real_walls else []

    def point_segment_distance(point, a, b):
        ab = b - a
        t = np.clip((point - a).dot(ab) / (ab.dot(ab) + 1e-12), 0.0, 1.0)
        return float(np.linalg.norm(point - (a + t * ab)))

    def boundary_samples(room):
        c = np.asarray(room['corners'])[:, :2]
        samples = []
        for i in range(len(c)):
            a, b = c[i], c[(i + 1) % len(c)]
            count = max(2, int(np.linalg.norm(b - a) / 0.25) + 1)
            samples.extend(a + (b - a) * t for t in np.linspace(0, 1, count))
        return samples

    out = []
    for index, room in enumerate(rooms):
        q = dict(room)
        corners = np.asarray(q['corners'])[:, :2]
        width = float(np.ptp(corners[:, 0]))
        depth = float(np.ptp(corners[:, 1]))
        min_width = min(width, depth)
        samples = boundary_samples(q)
        covered = observed = 0
        for point in samples:
            hits = [(point_segment_distance(point, *_wall_footprint_segment(w)), w)
                    for w in walls]
            if hits:
                distance, wall = min(hits, key=lambda item: item[0])
                if distance <= boundary_tolerance:
                    covered += 1
                    observed += not wall.get('synthetic', False)
        boundary_coverage = covered / max(len(samples), 1)
        observed_coverage = observed / max(covered, 1)
        centre = corners.mean(axis=0)
        stable = any(np.linalg.norm(
            np.asarray(r['corners'])[:, :2].mean(axis=0) - centre) < 0.75
                     for r in stable_rooms)
        openings = 0
        for wall in walls:
            for opening in wall.get('openings', []):
                op = np.asarray(opening['corners'])[:, :2].mean(axis=0)
                if min(point_segment_distance(
                        op, corners[i], corners[(i + 1) % len(corners)])
                       for i in range(len(corners))) <= boundary_tolerance:
                    openings += 1
        geometry_score = (1.0 if q['area'] >= 6.0 and min_width >= 1.8 and
                          2.0 <= q['height'] <= 6.0 else 0.35)
        score = 100.0 * (0.25 * geometry_score + 0.40 * boundary_coverage +
                         0.20 * observed_coverage + 0.10 * float(stable) +
                         0.05 * min(openings, 1))
        impossible_geometry = q['area'] < 2.0 or min_width < 1.2 or q['height'] < 1.8
        status = ('Rejected' if impossible_geometry else
                  ('Confirmed' if score >= 75 else
                   ('Candidate' if score >= 45 else 'Rejected')))
        reasons = []
        if geometry_score < 1.0: reasons.append('寸法または面積が非典型')
        if boundary_coverage < 0.75: reasons.append('境界壁の観測不足')
        if observed_coverage < 0.8: reasons.append('推定壁への依存')
        if not stable: reasons.append('補完なしでは閉じない')
        if not openings: reasons.append('出入口未検出')
        q.update(name=q.get('name', f'Room {index + 1}'),
                 room_confidence=float(round(score, 1)), room_status=status,
                 validation_metrics={
                     'width_m': width, 'depth_m': depth,
                     'boundary_coverage': boundary_coverage,
                     'observed_wall_fraction': observed_coverage,
                     'stable_without_repairs': stable, 'openings': openings,
                     'topology': q.get('topology_metrics', {}),
                 }, validation_reasons=reasons)
        out.append(q)
    return out


def _count_dangling_wall_ends(planes: list[dict], tolerance: float = 0.3) -> int:
    endpoints = [point for wall in planes if wall['kind'] == 'vertical'
                 for point in _wall_footprint_segment(wall)]
    return sum(not any(np.linalg.norm(point - other) <= tolerance
                       for j, other in enumerate(endpoints) if i != j)
               for i, point in enumerate(endpoints))


def _fmt_optional_pct(value: Optional[float]) -> str:
    return '-' if value is None else f'{value:.0%}'


def _fmt_optional_fixed(value: Optional[float], decimals: int = 4) -> str:
    return '-' if value is None else f'{value:.{decimals}f}'


def build_bim_metrics(planes: list[dict], rooms: Optional[list[dict]] = None,
                      *, source: str = '', settings: Optional[dict] = None) -> dict:
    """Build a deterministic, machine-readable BIM QA manifest."""
    rooms = (assess_rooms(derive_room_candidates(planes), planes)
             if rooms is None else rooms)
    walls = [plane for plane in planes if plane['kind'] == 'vertical']
    openings = [opening for wall in walls for opening in wall.get('openings', [])]
    accepted = [room for room in rooms if room['room_status'] != 'Rejected']
    rejected = [room for room in rooms if room['room_status'] == 'Rejected']
    observed_fits = [plane['element_fit'] for plane in planes
                     if not plane.get('synthetic') and plane.get('element_fit')]

    def values(key):
        return [float(fit[key]) for fit in observed_fits if fit.get(key) is not None]

    def aggregate(key):
        data = values(key)
        return ({'min': min(data), 'mean': float(np.mean(data)), 'max': max(data)}
                if data else {'min': None, 'mean': None, 'max': None})

    elements = []
    for index, plane in enumerate(planes):
        fit = plane.get('element_fit', {})
        elements.append({
            'index': index + 1,
            'kind': plane['kind'],
            'provenance': plane.get('provenance',
                                    'inferred' if plane.get('synthetic') else 'observed'),
            'confidence': plane.get('confidence'),
            'confidence_level': plane.get('confidence_level'),
            'coverage_ratio': fit.get('coverage_ratio'),
            'distribution_ratio': fit.get('distribution_ratio'),
            'distance_rmse_m': fit.get('distance_rmse_m'),
            'distance_p95_m': fit.get('distance_p95_m'),
            'support_points': plane.get('confidence_metrics', {}).get('support_points'),
            'topology_adjusted': bool(plane.get('topology_adjustment')),
        })
    return {
        'schema_version': 1,
        'source': source,
        'settings': settings or {},
        'summary': {
            'slabs': sum(plane['kind'] == 'horizontal' for plane in planes),
            'observed_walls': sum(not wall.get('synthetic') for wall in walls),
            'synthetic_walls': sum(bool(wall.get('synthetic')) for wall in walls),
            'doors': sum(opening.get('kind') == 'door' for opening in openings),
            'windows': sum(opening.get('kind') == 'window' for opening in openings),
            'accepted_rooms': len(accepted),
            'rejected_room_candidates': len(rejected),
            'dangling_wall_ends': _count_dangling_wall_ends(planes),
            'topology_adjusted_walls': sum(
                bool(plane.get('topology_adjustment')) for plane in planes),
        },
        'element_fit': {
            'coverage_ratio': aggregate('coverage_ratio'),
            'distribution_ratio': aggregate('distribution_ratio'),
            'distance_rmse_m': aggregate('distance_rmse_m'),
            'distance_p95_m': aggregate('distance_p95_m'),
        },
        'elements': elements,
        'rooms': [{
            'name': room['name'], 'area_m2': float(room['area']),
            'status': room['room_status'],
            'confidence': float(room['room_confidence']),
            'generation_method': room.get('generation_method', ''),
            'validation_metrics': room.get('validation_metrics', {}),
            'validation_reasons': room.get('validation_reasons', []),
        } for room in rooms],
    }


def write_bim_metrics(planes: list[dict], path: str | Path,
                      rooms: Optional[list[dict]] = None, *, source: str = '',
                      settings: Optional[dict] = None) -> Path:
    """Write :func:`build_bim_metrics` as stable UTF-8 JSON."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(build_bim_metrics(
        planes, rooms, source=source, settings=settings), ensure_ascii=False,
        indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return path


def write_html_report(planes: list[dict], path: str | Path, *,
                      source: str = '', ifc_path: str = '',
                      settings: Optional[dict] = None) -> Path:
    """Write a dependency-free BIM QA report with an inline SVG floor plan."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    walls = [p for p in planes if p['kind'] == 'vertical']
    real_walls = [p for p in walls if not p.get('synthetic')]
    repair_walls = [p for p in walls if p.get('synthetic')]
    segments = [_wall_footprint_segment(w) for w in walls]
    endpoints = [p for segment in segments for p in segment]
    dangling = []
    for i, point in enumerate(endpoints):
        distances = [np.linalg.norm(point - other) for j, other in enumerate(endpoints)
                     if j != i]
        if not distances or min(distances) > 0.3:
            dangling.append(point)
    rooms = assess_rooms(derive_room_candidates(planes), planes)
    accepted_rooms = [r for r in rooms if r['room_status'] != 'Rejected']
    rejected_rooms = [r for r in rooms if r['room_status'] == 'Rejected']
    openings = [op for p in walls for op in p.get('openings', [])]
    all_xy = np.vstack(endpoints) if endpoints else np.array([[0., 0.], [1., 1.]])
    lo, hi = all_xy.min(axis=0), all_xy.max(axis=0)
    span = np.maximum(hi - lo, 1.0)
    width, height, pad = 900, 600, 35
    scale = min((width - 2 * pad) / span[0], (height - 2 * pad) / span[1])

    def xy(point):
        x = pad + (point[0] - lo[0]) * scale
        y = height - pad - (point[1] - lo[1]) * scale
        return float(x), float(y)

    svg = []
    for wall, (a, b) in zip(walls, segments):
        quality = wall.get('quality_segments', [])
        if quality and not wall.get('synthetic'):
            for seg in quality:
                x1, y1 = xy(seg['start']); x2, y2 = xy(seg['end'])
                svg.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" '
                           f'y2="{y2:.1f}" class="q{seg["level"].lower()}"/>')
        else:
            x1, y1 = xy(a); x2, y2 = xy(b)
            svg.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" '
                       f'y2="{y2:.1f}" class="{"repair" if wall.get("synthetic") else "wall"}"/>')
    for op in openings:
        c = np.asarray(op['corners'])[:, :2].mean(axis=0)
        x, y = xy(c)
        cls = 'door' if op.get('kind') == 'door' else 'window'
        svg.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="6" class="{cls}"/>')
    for point in dangling:
        x, y = xy(point)
        svg.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="7" class="gap"/>')
    counts = {
        '床・水平面': sum(p['kind'] == 'horizontal' for p in planes),
        '壁': len(real_walls), '補完壁': len(repair_walls),
        'ドア': sum(o.get('kind') == 'door' for o in openings),
        '窓': sum(o.get('kind') == 'window' for o in openings),
        '部屋': len(accepted_rooms), 'Rejected部屋候補': len(rejected_rooms),
        '未接続の壁端': len(dangling),
    }
    confidence_counts = {level: sum(p.get('confidence_level') == level for p in planes)
                         for level in ('High', 'Medium', 'Low')}
    counts.update({f'信頼度 {k}': v for k, v in confidence_counts.items()})
    cards = ''.join(f'<div class="card"><b>{html.escape(k)}</b><span>{v}</span></div>'
                    for k, v in counts.items())
    setting_rows = ''.join(
        f'<tr><th>{html.escape(str(k))}</th><td>{html.escape(str(v))}</td></tr>'
        for k, v in (settings or {}).items())
    room_rows = ''.join(
        f'<tr><td>{html.escape(r["name"])}</td><td>{r["area"]:.1f}</td>'
        f'<td>{r["room_confidence"]}</td><td>{r["room_status"]}</td>'
        f'<td>{r["validation_metrics"]["boundary_coverage"]:.0%}</td>'
        f'<td>{html.escape("、".join(r["validation_reasons"]) or "問題なし")}</td></tr>'
        for r in rooms)
    confidence_rows = ''.join(
        f'<tr><td>{i + 1}</td><td>{html.escape(p["kind"])}</td>'
        f'<td>{html.escape(p.get("provenance", "unknown"))}</td>'
        f'<td>{p.get("confidence", "-")}</td><td>{html.escape(p.get("confidence_level", "-"))}</td>'
        f'<td>{p.get("confidence_metrics", {}).get("support_points", "-")}</td>'
        f'<td>{_fmt_optional_pct(p.get("element_fit", {}).get("coverage_ratio"))}</td>'
        f'<td>{_fmt_optional_pct(p.get("element_fit", {}).get("distribution_ratio"))}</td>'
        f'<td>{_fmt_optional_fixed(p.get("element_fit", {}).get("distance_rmse_m"))}</td>'
        f'<td>{_fmt_optional_fixed(p.get("element_fit", {}).get("distance_p95_m"))}</td></tr>'
        for i, p in enumerate(planes))
    local_segments = [(wi, si, seg) for wi, wall in enumerate(walls)
                      for si, seg in enumerate(wall.get('quality_segments', []))]
    issue_rows = ''.join(
        f'<tr><td>Wall {wi + 1}</td><td>{si + 1}</td><td>{seg["score"]}</td>'
        f'<td>{seg["level"]}</td><td>{seg["support_points"]}</td>'
        f'<td>{_fmt_optional_fixed(seg["observation_rmse_m"], 3)}</td>'
        f'<td>{_fmt_optional_fixed(seg["model_rmse_m"], 3)}</td>'
        f'<td>{html.escape(seg["recommendation"])}</td></tr>'
        for wi, si, seg in local_segments if seg['level'] != 'High')
    regularization_rows = ''.join(
        f'<tr><td>Wall {i + 1}</td><td>{html.escape(r["decision"])}</td>'
        f'<td>{r["observed_angle_deg"]:.1f}</td><td>{r["adopted_angle_deg"]:.1f}</td>'
        f'<td>{r["angle_change_deg"]:.1f}</td><td>{r["observed_rmse_m"]:.3f}</td>'
        f'<td>{r["adopted_rmse_m"]:.3f}</td><td>{r["full_snap_rmse_m"]:.3f}</td></tr>'
        for i, wall in enumerate(real_walls) if (r := wall.get('regularization')))
    diagnosis = ('部屋候補は見つかりましたが、品質基準を満たさずRejectedになりました。'
                 if rejected_rooms and not accepted_rooms else
                 ('壁が閉ループを形成していないため、部屋を復元できませんでした。'
                  if not rooms and walls else
                 ('壁が検出されていません。抽出設定または点群を確認してください。'
                  if not walls else '閉じた壁領域から部屋を復元できました。')))
    doc = f'''<!doctype html><html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>BIM QA Report</title>
<style>body{{font-family:system-ui,sans-serif;margin:0;background:#f5f7fa;color:#17202a}}
main{{max-width:1000px;margin:auto;padding:24px}}h1{{margin-bottom:4px}}.muted{{color:#657786}}
.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:12px;margin:20px 0}}
.card{{background:white;border-radius:10px;padding:15px;box-shadow:0 1px 4px #ccd}}
.card span{{display:block;font-size:1.8rem;font-weight:700;margin-top:5px}}section{{background:white;padding:18px;
border-radius:10px;margin:16px 0}}svg{{width:100%;height:auto;background:#fbfcfd;border:1px solid #dde}}
.wall{{stroke:#263238;stroke-width:4;stroke-linecap:round}}.repair{{stroke:#fb8c00;stroke-width:4;stroke-dasharray:10 7}}
.qhigh{{stroke:#2e7d32;stroke-width:5}}.qmedium{{stroke:#f9a825;stroke-width:5}}.qlow{{stroke:#c62828;stroke-width:5}}
.gap{{fill:#e53935;opacity:.85}}
.door{{fill:#43a047}}.window{{fill:#1e88e5}}table{{border-collapse:collapse;width:100%}}
th,td{{text-align:left;border-bottom:1px solid #eee;padding:8px}}.warn{{border-left:5px solid #f9a825}}
a{{color:#1565c0}}code{{overflow-wrap:anywhere}}</style></head><body><main>
<h1>LiDARSLAM BIM QA Report</h1><p class="muted">入力: <code>{html.escape(source)}</code></p>
<div class="cards">{cards}</div><section><h2>上面図</h2>
<p><span style="color:#2e7d32">緑: High</span>　<span style="color:#f9a825">黄: Medium</span>　<span style="color:#c62828">赤: Low</span>　<span style="color:#fb8c00">破線: 補完壁</span>　● 未接続端</p>
<svg viewBox="0 0 {width} {height}" role="img">{''.join(svg)}</svg></section>
<section class="warn"><h2>診断</h2><p>{diagnosis}</p></section>
<section><h2>要素別の信頼度</h2><p class="muted">支持点と、面グリッドのCoverage・点分布のDistribution・モデル面までのDistanceによるQA指標（測量精度の証明ではありません）。</p>
<table><tr><th>#</th><th>種類</th><th>由来</th><th>スコア</th><th>区分</th><th>支持点</th><th>Coverage</th><th>Distribution</th><th>Distance RMSE [m]</th><th>Distance P95 [m]</th></tr>{confidence_rows}</table></section>
<section><h2>要対応の壁区間</h2><table><tr><th>壁</th><th>区間</th><th>スコア</th><th>区分</th><th>支持点</th><th>観測RMSE [m]</th><th>モデルRMSE [m]</th><th>推奨対応</th></tr>{issue_rows or '<tr><td colspan="8">要対応区間はありません</td></tr>'}</table></section>
<section><h2>壁の適応型正規化</h2><table><tr><th>壁</th><th>判断</th><th>元角度</th><th>採用角度</th><th>変更量</th><th>観測RMSE</th><th>採用RMSE</th><th>Full RMSE</th></tr>{regularization_rows or '<tr><td colspan="8">正規化情報はありません</td></tr>'}</table></section>
<section><h2>部屋の妥当性</h2><table><tr><th>部屋</th><th>面積 [m²]</th><th>スコア</th><th>判定</th><th>境界</th><th>理由</th></tr>{room_rows or '<tr><td colspan="6">部屋候補はありません</td></tr>'}</table></section>
<section><h2>実行設定</h2><table>{setting_rows or '<tr><td>設定情報なし</td></tr>'}</table></section>
<section><h2>成果物</h2><p><a href="{html.escape(Path(ifc_path).name)}">IFCを開く</a></p></section>
</main></body></html>'''
    path.write_text(doc, encoding='utf-8')
    return path


# --------------------------------------------------------------------------- #
# IFC writer (lazy ifcopenshell)
# --------------------------------------------------------------------------- #
_IFC_CLASS = {
    'horizontal': 'IfcSlab',
    'vertical': 'IfcWall',
    'other': 'IfcBuildingElementProxy',
}


def _ifcopenshell():
    try:
        import ifcopenshell  # noqa: F401
        import ifcopenshell.api  # noqa: F401
    except ImportError as exc:  # pragma: no cover - env-dependent
        raise ImportError(
            'bim_export needs ifcopenshell (`pip install ifcopenshell`). The '
            'numpy-only exports (map_export/las_export) do not.') from exc
    return ifcopenshell


def _element_name(kind: str, centroid: np.ndarray, i: int) -> str:
    if kind == 'horizontal':
        # Split slabs into floor/ceiling by height for readability only.
        return f'Slab {i}'
    if kind == 'vertical':
        return f'Wall {i}'
    return f'Surface {i}'


def write_ifc(planes: list[dict], path: str | Path, *,
              project_name: str = 'LIDARSLAM BIM',
              min_thickness: float = 0.15,
              rooms: Optional[list[dict]] = None) -> Path:
    """Write extracted ``planes`` (from :func:`extract_planes`) to an IFC4 file.

    Each plane becomes a solid element (``IfcSlab`` for horizontal, ``IfcWall``
    for vertical, ``IfcBuildingElementProxy`` otherwise) with a tessellated box
    representation, placed inside a Project/Site/Building/Storey hierarchy.
    Returns the output path.
    """
    ifcopenshell = _ifcopenshell()
    from ifcopenshell.api import run

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    model = run('project.create_file')
    project = run('root.create_entity', model, ifc_class='IfcProject',
                  name=project_name)
    run('unit.assign_unit', model, length={'is_metric': True, 'raw': 'METERS'})
    ctx = run('context.add_context', model, context_type='Model')
    body = run('context.add_context', model, context_type='Model',
               context_identifier='Body', target_view='MODEL_VIEW', parent=ctx)

    site = run('root.create_entity', model, ifc_class='IfcSite', name='Site')
    building = run('root.create_entity', model, ifc_class='IfcBuilding',
                   name='Building')
    storey = run('root.create_entity', model, ifc_class='IfcBuildingStorey',
                 name='Storey')
    run('aggregate.assign_object', model, products=[site], relating_object=project)
    run('aggregate.assign_object', model, products=[building], relating_object=site)
    run('aggregate.assign_object', model, products=[storey],
        relating_object=building)

    for i, pl in enumerate(planes):
        kind = pl['kind']
        thickness = max(pl['thickness'], min_thickness)
        verts, faces = box_from_rectangle(pl['corners'], pl['normal'], thickness)
        element = run('root.create_entity', model,
                      ifc_class=_IFC_CLASS[kind],
                      name=_element_name(kind, pl['centroid'], i))
        rep = run('geometry.add_mesh_representation', model, context=body,
                  vertices=[[tuple(map(float, v)) for v in verts]],
                  faces=[faces])
        run('geometry.assign_representation', model, product=element,
            representation=rep)
        run('spatial.assign_container', model, products=[element],
            relating_structure=storey)
        if pl.get('synthetic'):
            pset = run('pset.add_pset', model, product=element,
                       name='Pset_LIDARSLAMRepair')
            repair_properties = {
                'IsSynthetic': True,
                'Reason': str(pl.get('repair_reason', 'wall gap repair')),
            }
            validation = pl.get('topology_validation', {})
            if validation:
                repair_properties.update({
                    'TopologyDecision': str(validation['decision']),
                    'RoomCyclesBefore': int(validation['room_cycles_before']),
                    'RoomCyclesAfter': int(validation['room_cycles_after']),
                    'MinimumObservedBoundary': float(
                        validation['min_observed_boundary']),
                })
            run('pset.edit_pset', model, pset=pset,
                properties=repair_properties)
        if pl.get('topology_adjustment'):
            adjustment = pl['topology_adjustment']
            pset = run('pset.add_pset', model, product=element,
                       name='Pset_LIDARSLAMTopology')
            run('pset.edit_pset', model, pset=pset, properties={
                'Decision': str(adjustment['decision']),
                'ExtensionLength': float(adjustment['extension_length_m']),
                'AcceptedCycleCount': int(
                    adjustment['accepted_cycle_count']),
            })
        if 'confidence' in pl:
            metrics = pl.get('confidence_metrics', {})
            pset = run('pset.add_pset', model, product=element,
                       name='Pset_LIDARSLAMConfidence')
            properties = {
                'Score': float(pl['confidence']),
                'Level': str(pl['confidence_level']),
                'Provenance': str(pl.get('provenance', 'unknown')),
                'SupportPoints': int(metrics.get('support_points', 0)),
                'PointDensity': float(metrics.get('point_density_m2', 0.0)),
                'Method': 'support(35%)+plane RMSE(40%)+density(25%)',
            }
            if metrics.get('plane_rmse_m') is not None:
                properties['PlaneRMSE'] = float(metrics['plane_rmse_m'])
                properties['ObservationRMSE'] = float(metrics['observation_rmse_m'])
            if metrics.get('model_rmse_m') is not None:
                properties['ModelRMSE'] = float(metrics['model_rmse_m'])
            segments = pl.get('quality_segments', [])
            if segments:
                worst_i, worst = min(enumerate(segments), key=lambda item: item[1]['score'])
                properties.update({
                    'WorstSegment': int(worst_i + 1),
                    'WorstSegmentScore': float(worst['score']),
                    'WorstSegmentLevel': str(worst['level']),
                    'RecommendedAction': str(worst['recommendation']),
                })
                if worst.get('observation_rmse_m') is not None:
                    properties['WorstObservationRMSE'] = float(worst['observation_rmse_m'])
                if worst.get('model_rmse_m') is not None:
                    properties['WorstModelRMSE'] = float(worst['model_rmse_m'])
            run('pset.edit_pset', model, pset=pset, properties=properties)
        if pl.get('element_fit'):
            fit = pl['element_fit']
            pset = run('pset.add_pset', model, product=element,
                       name='Pset_LIDARSLAMElementFit')
            properties = {
                'CoverageRatio': float(fit['coverage_ratio']),
                'DistributionRatio': float(fit['distribution_ratio']),
                'MatchedPoints': int(fit['matched_points']),
                'OccupiedCells': int(fit['occupied_cells']),
                'TotalCells': int(fit['total_cells']),
                'GridCellSize': float(fit['cell_m']),
                'DistanceThreshold': float(fit['distance_threshold_m']),
                'VisibilityModel': str(fit['visibility_model']),
            }
            if fit['distance_rmse_m'] is not None:
                properties['DistanceRMSE'] = float(fit['distance_rmse_m'])
                properties['DistanceP95'] = float(fit['distance_p95_m'])
            run('pset.edit_pset', model, pset=pset, properties=properties)
        if pl.get('regularization'):
            reg = pl['regularization']
            pset = run('pset.add_pset', model, product=element,
                       name='Pset_LIDARSLAMRegularization')
            run('pset.edit_pset', model, pset=pset, properties={
                'Decision': str(reg['decision']),
                'ObservedAngle': float(reg['observed_angle_deg']),
                'AdoptedAngle': float(reg['adopted_angle_deg']),
                'AngleChange': float(reg['angle_change_deg']),
                'ObservedRMSE': float(reg['observed_rmse_m']),
                'AdoptedRMSE': float(reg['adopted_rmse_m']),
                'FullSnapRMSE': float(reg['full_snap_rmse_m']),
            })
        color = pl.get('color')
        if color is not None:
            _apply_surface_colour(run, model, rep, color)
        for k, op in enumerate(pl.get('openings', [])):
            _add_opening(run, model, body, storey, element, pl['normal'],
                         thickness, op, f'{i}.{k}')

    for r, room in enumerate(rooms or []):
        if room.get('room_status') == 'Rejected':
            continue
        number = str(room.get('number', r + 1))
        name = str(room.get('name', f'Room {number}'))
        space = run('root.create_entity', model, ifc_class='IfcSpace',
                    name=name)
        rv, rf = extrude_polygon(
            room['corners'], max(room['height'], min_thickness))
        rrep = run('geometry.add_mesh_representation', model, context=body,
                   vertices=[[tuple(map(float, v)) for v in rv]], faces=[rf])
        run('geometry.assign_representation', model, product=space,
            representation=rrep)
        run('aggregate.assign_object', model, products=[space],
            relating_object=storey)
        pset = run('pset.add_pset', model, product=space,
                   name='Pset_LIDARSLAMSpace')
        space_properties = {
            'RoomNumber': number,
            'NetFloorArea': float(room['area']),
            'CeilingHeight': float(room['height']),
            'StoreyName': str(room.get('storey', storey.Name)),
            'GenerationMethod': str(room.get(
                'generation_method', 'LiDAR wall enclosure')),
            'ValidationStatus': str(room.get('room_status', 'Unassessed')),
            'ConfidenceScore': float(room.get('room_confidence', 0.0)),
            'ValidationReasons': '; '.join(room.get('validation_reasons', [])),
        }
        topology = room.get('topology_metrics', {})
        if topology:
            space_properties.update({
                'TopologyObservedBoundary': float(
                    topology['observed_boundary_ratio']),
                'TopologyExtensionLength': float(
                    topology['extension_length_m']),
                'TopologySyntheticEdges': int(
                    topology['synthetic_edge_count']),
                'PolygonVertices': int(topology['vertices']),
            })
        run('pset.edit_pset', model, pset=pset,
            properties=space_properties)

    model.write(str(path))
    return path


_FILLING_CLASS = {'door': 'IfcDoor', 'window': 'IfcWindow'}


def _add_opening(run, model, body, storey, wall, normal, wall_thickness, op,
                 tag) -> None:
    """Punch an ``IfcOpeningElement`` through ``wall`` at ``op``'s rectangle and
    fill it with an ``IfcDoor`` / ``IfcWindow`` (IfcRelVoids/IfcRelFills)."""
    n = np.asarray(normal, dtype=np.float64)
    n = n / (np.linalg.norm(n) + 1e-12)
    # Opening: a box a bit thicker than the wall so the void cuts cleanly through.
    ov, of = box_from_rectangle(op['corners'], n, wall_thickness * 2.0)
    opening = run('root.create_entity', model, ifc_class='IfcOpeningElement',
                  name=f'Opening {tag}')
    orep = run('geometry.add_mesh_representation', model, context=body,
               vertices=[[tuple(map(float, v)) for v in ov]], faces=[of])
    run('geometry.assign_representation', model, product=opening,
        representation=orep)
    run('feature.add_feature', model, feature=opening, element=wall)

    fill_class = _FILLING_CLASS[op['kind']]
    fv, ff = box_from_rectangle(op['corners'], n, wall_thickness)
    filling = run('root.create_entity', model, ifc_class=fill_class,
                  name=f'{op["kind"].capitalize()} {tag}')
    frep = run('geometry.add_mesh_representation', model, context=body,
               vertices=[[tuple(map(float, v)) for v in fv]], faces=[ff])
    run('geometry.assign_representation', model, product=filling,
        representation=frep)
    run('spatial.assign_container', model, products=[filling],
        relating_structure=storey)
    run('feature.add_filling', model, opening=opening, element=filling)


def _apply_surface_colour(run, model, representation, rgb) -> None:
    """Attach an ``IfcSurfaceStyleShading`` (from a uint8 ``rgb``) to a shape
    representation so BIM viewers render the element in its point-cloud colour.
    """
    r, g, b = (float(c) / 255.0 for c in rgb[:3])
    style = run('style.add_style', model)
    run('style.add_surface_style', model, style=style,
        ifc_class='IfcSurfaceStyleShading',
        attributes={'SurfaceColour': {'Name': None, 'Red': r, 'Green': g,
                                      'Blue': b}})
    run('style.assign_representation_styles', model,
        shape_representation=representation, styles=[style])


def extract_and_export(xyz: np.ndarray, path: str | Path, *,
                       rgb: Optional[np.ndarray] = None,
                       threshold: float = 0.1, max_planes: int = 12,
                       min_inliers: int = 300, thin_voxel: float = 0.0,
                       min_thickness: float = 0.15,
                       find_openings: bool = False, opening_close: int = 0,
                       denoise_voxel: float = 0.0, denoise_min_count: int = 4,
                       building: bool = False, regularize: bool = False,
                       adaptive_regularize: bool = False,
                       corner_snap: float = 0.5,
                       repair_walls: bool = False, repair_gap: float = 0.75,
                       rooms: bool = False,
                       room_close: int = 3) -> tuple[Path, list[dict]]:
    """Extract planes from ``xyz`` and write them to an IFC file.

    Returns ``(path, planes)``. With ``thin_voxel > 0`` the cloud is
    voxel-downsampled first (shared code path with the other exports; ``rgb`` is
    carried through the thinning). When ``rgb`` is given each IFC element is
    styled with its inliers' mean colour. With ``find_openings`` each wall's
    doors/windows are detected and emitted as IfcDoor/IfcWindow voiding the wall.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    rgb = None if rgb is None else np.asarray(rgb, dtype=np.uint8)
    if denoise_voxel > 0.0:
        keep = voxel_density_filter(xyz, denoise_voxel, denoise_min_count)
        xyz = xyz[keep]
        rgb = None if rgb is None else rgb[keep]
    if thin_voxel > 0.0:
        from pointcloud_io import voxel_downsample
        xyz, rgb = voxel_downsample(xyz, thin_voxel, rgb)
    if building:
        planes = extract_building(xyz, colors=rgb, threshold=threshold,
                                  wall_planes=max_planes, min_inliers=min_inliers,
                                  find_openings=find_openings,
                                  opening_close=opening_close)
    else:
        planes = extract_planes(xyz, colors=rgb, threshold=threshold,
                                max_planes=max_planes, min_inliers=min_inliers,
                                find_openings=find_openings,
                                opening_close=opening_close)
    topology_cycles = []
    if regularize:
        planes = regularize_walls(planes, xyz, adaptive=adaptive_regularize)
    if (regularize and corner_snap > 0.0) or repair_walls:
        planes, topology_cycles = optimize_wall_topology(
            planes,
            intersection_gap=corner_snap if regularize else 0.0,
            repair_gap=repair_gap if repair_walls else 0.0)
    planes = add_element_fit_metrics(planes, xyz)
    planes = add_plane_confidence(planes, xyz)
    planes = add_wall_quality_segments(planes, xyz)
    room_candidates = (rooms_from_wall_cycles(topology_cycles)
                       if topology_cycles else
                       reconstruct_rooms(planes, close_iter=room_close))
    room_list = assess_rooms(room_candidates, planes) if rooms else None
    out = write_ifc(planes, path, min_thickness=min_thickness, rooms=room_list)
    return out, planes


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def _build_arg_parser():
    import argparse
    p = argparse.ArgumentParser(
        description='Extract planar building elements from a PLY/PCD cloud and '
                    'export them to IFC (CAD/BIM).')
    p.add_argument('input', help='input .ply/.pcd (xyz [+ rgb])')
    p.add_argument('output', nargs='?',
                   help='output .ifc (default: input name with .ifc)')
    p.add_argument('--indoor', action='store_true',
                   help='recommended indoor preset: walls, denoise, openings, '
                        'regularization and rooms')
    p.add_argument('--threshold', type=float, default=0.1,
                   help='RANSAC inlier distance [m] (default 0.1)')
    p.add_argument('--max-planes', type=int, default=None,
                   help='maximum planes (default: 12, indoor: 20)')
    p.add_argument('--min-inliers', type=int, default=None,
                   help='minimum plane inliers (default: 300, indoor: 600)')
    p.add_argument('--thin-voxel', type=float, default=None,
                   help='voxel size [m] to downsample before fitting (0=off)')
    p.add_argument('--min-thickness', type=float, default=0.15,
                   help='minimum solid thickness [m] for thin planes')
    p.add_argument('--no-color', action='store_true',
                   help='ignore PLY colours (no IfcStyledItem surface colour)')
    p.add_argument('--openings', action='store_true',
                   help='detect doors/windows in walls -> IfcDoor/IfcWindow')
    p.add_argument('--opening-close', type=int, default=None,
                   help='morphological closing iters to bridge patchy scans '
                        'before opening detection (noisy SLAM maps)')
    p.add_argument('--denoise-voxel', type=float, default=None,
                   help='voxel size [m] for density denoise (0=off)')
    p.add_argument('--denoise-min-count', type=int, default=4,
                   help='min points per voxel to keep in denoise')
    p.add_argument('--building', action='store_true',
                   help='orientation-separated extraction (dedicated wall pass '
                        'so walls survive multi-storey floor-heavy scans)')
    p.add_argument('--regularize', action='store_true',
                   help='snap wall normals to the building Manhattan grid')
    p.add_argument('--adaptive-regularize', action='store_true',
                   help='choose full/soft/no snap per wall from RMSE degradation')
    p.add_argument('--corner-snap', type=float, default=0.5,
                   help='max wall-end gap [m] closed after regularization')
    p.add_argument('--rooms', action='store_true',
                   help='reconstruct enclosed rooms as IfcSpace')
    p.add_argument('--repair-walls', action='store_true',
                   help='add conservative synthetic walls between nearby ends')
    p.add_argument('--repair-gap', type=float, default=0.75,
                   help='maximum synthetic wall length [m]')
    p.add_argument('--report', nargs='?', const='auto',
                   help='write an HTML QA report (default path beside IFC)')
    p.add_argument('--metrics-json', nargs='?', const='auto',
                   help='write machine-readable BIM QA metrics (automatically '
                        'enabled beside IFC when --report is used)')
    return p


def _apply_cli_defaults(args):
    """Apply normal/indoor defaults while preserving explicit CLI values."""
    args.output = args.output or str(Path(args.input).with_suffix('.ifc'))
    args.max_planes = args.max_planes if args.max_planes is not None \
        else (20 if args.indoor else 12)
    args.min_inliers = args.min_inliers if args.min_inliers is not None \
        else (600 if args.indoor else 300)
    args.thin_voxel = args.thin_voxel if args.thin_voxel is not None \
        else (0.1 if args.indoor else 0.0)
    args.denoise_voxel = args.denoise_voxel if args.denoise_voxel is not None \
        else (0.15 if args.indoor else 0.0)
    args.opening_close = args.opening_close if args.opening_close is not None \
        else (1 if args.indoor else 0)
    if args.indoor:
        args.building = args.regularize = args.openings = args.rooms = True
    if args.adaptive_regularize:
        args.regularize = True
    return args


def main(argv=None) -> int:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from pointcloud_io import read_point_cloud_xyz
    args = _apply_cli_defaults(_build_arg_parser().parse_args(argv))
    print(f'[1/3] Reading point cloud: {args.input}', flush=True)
    xyz, rgb = read_point_cloud_xyz(args.input)
    print(f'      {len(xyz):,} points', flush=True)
    if args.no_color:
        rgb = None
    mode = 'indoor preset' if args.indoor else 'custom settings'
    print(f'[2/3] Extracting BIM elements ({mode})...', flush=True)
    out, planes = extract_and_export(
        xyz, args.output, rgb=rgb, threshold=args.threshold,
        max_planes=args.max_planes, min_inliers=args.min_inliers,
        thin_voxel=args.thin_voxel, min_thickness=args.min_thickness,
        find_openings=args.openings, opening_close=args.opening_close,
        denoise_voxel=args.denoise_voxel,
        denoise_min_count=args.denoise_min_count, building=args.building,
        regularize=args.regularize,
        adaptive_regularize=args.adaptive_regularize,
        corner_snap=args.corner_snap,
        repair_walls=args.repair_walls, repair_gap=args.repair_gap,
        rooms=args.rooms)
    kinds: dict[str, int] = {}
    for pl in planes:
        kinds[pl['kind']] = kinds.get(pl['kind'], 0) + 1
    n_doors = sum(op.get('kind') == 'door' for pl in planes
                  for op in pl.get('openings', []))
    n_windows = sum(op.get('kind') == 'window' for pl in planes
                    for op in pl.get('openings', []))
    assessed_rooms = (assess_rooms(derive_room_candidates(planes), planes)
                      if args.rooms else [])
    n_rooms = sum(r['room_status'] != 'Rejected' for r in assessed_rooms)
    n_rejected_rooms = sum(r['room_status'] == 'Rejected' for r in assessed_rooms)
    settings = {
        'Preset': mode, 'Max planes': args.max_planes,
        'RANSAC threshold [m]': args.threshold,
        'Corner snap [m]': args.corner_snap,
        'Adaptive regularize': args.adaptive_regularize,
        'Repair walls': args.repair_walls,
        'Repair gap [m]': args.repair_gap,
        'Thin voxel [m]': args.thin_voxel,
        'Denoise voxel [m]': args.denoise_voxel,
    }
    report_path = None
    if args.report:
        report_path = (Path(out).with_name(Path(out).stem + '_report.html')
                       if args.report == 'auto' else Path(args.report))
        write_html_report(planes, report_path, source=str(args.input),
                          ifc_path=str(out), settings=settings)
    metrics_path = None
    metrics_option = args.metrics_json or ('auto' if args.report else None)
    if metrics_option:
        metrics_path = (Path(out).with_name(Path(out).stem + '_metrics.json')
                        if metrics_option == 'auto' else Path(metrics_option))
        write_bim_metrics(planes, metrics_path, assessed_rooms,
                          source=str(args.input), settings=settings)
    print('[3/3] Complete', flush=True)
    print(f'      Output : {out}')
    print(f'      Slabs  : {kinds.get("horizontal", 0)}')
    print(f'      Walls  : {kinds.get("vertical", 0)}')
    print(f'      Doors  : {n_doors}')
    print(f'      Windows: {n_windows}')
    print(f'      Rooms  : {n_rooms}')
    if n_rejected_rooms:
        print(f'      Rejected room candidates: {n_rejected_rooms}')
    if report_path:
        print(f'      Report : {report_path}')
    if metrics_path:
        print(f'      Metrics: {metrics_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
