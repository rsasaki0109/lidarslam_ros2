#!/usr/bin/env python3
"""Export a coloured SLAM point cloud into GIS-friendly formats (numpy only).

The SLAM pipeline produces a coloured, *georeferenced* cloud in a local ENU
frame (x=east, y=north, z=up) plus a WGS84 origin (see ``local_to_wgs84`` in
``scripts/simple_lanelet2_generator.py``). This module is the "export hub" that
turns that hub cloud into downstream deliverables. Step 1 (this file) is the
dependency-free GIS bridge: a colour-carrying delimited-text export that QGIS /
CloudCompare / ArcGIS open directly as a lon/lat point layer, plus sidecar
files documenting the coordinate reference system.

Heavier exports (LAS/LAZ via ``laspy``, meshes via ``open3d``) live in separate
functions/modules so this one stays numpy-only and importable everywhere. See
``docs/research/3dgs-postprocess-map-design.md`` and roadmap §12.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import numpy as np

# WGS84 ellipsoid — identical constants to scripts/simple_lanelet2_generator.py
# so a cloud and its lanelet2 map share one georeferencing convention.
_A = 6_378_137.0  # semi-major axis [m]
_F = 1.0 / 298.257223563
_B = _A * (1.0 - _F)  # semi-minor axis
_E2 = 1.0 - (_B * _B) / (_A * _A)

# EPSG:4326 (WGS84 geographic lon/lat) as OGC WKT — written as a .prj sidecar so
# the CRS of the exported lon/lat columns is self-documenting.
WGS84_WKT = (
    'GEOGCS["WGS 84",DATUM["WGS_1984",'
    'SPHEROID["WGS 84",6378137,298.257223563,AUTHORITY["EPSG","7030"]],'
    'AUTHORITY["EPSG","6326"]],'
    'PRIMEM["Greenwich",0,AUTHORITY["EPSG","8901"]],'
    'UNIT["degree",0.0174532925199433,AUTHORITY["EPSG","9122"]],'
    'AUTHORITY["EPSG","4326"]]'
)


def _radius_of_curvature_n(lat_rad: float) -> float:
    sin_lat = math.sin(lat_rad)
    return _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)


def enu_to_wgs84(x: np.ndarray, y: np.ndarray, z: np.ndarray,
                 origin_lat: float, origin_lon: float
                 ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert local ENU (x=east, y=north, z=up) offsets [m] to WGS84.

    Uses the same flat-Earth linearisation about the origin as the lanelet2
    generator, so a point cloud and its lanelet2 map land on the same spot.
    Returns ``(lon, lat, ele)`` in degrees / degrees / metres. Accurate to a few
    cm over the ~km-scale extents these local SLAM maps cover.
    """
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    z = np.asarray(z, dtype=np.float64)
    lat0 = math.radians(origin_lat)
    lon0 = math.radians(origin_lon)
    n0 = _radius_of_curvature_n(lat0)
    m0 = _A * (1.0 - _E2) / (1.0 - _E2 * math.sin(lat0) ** 2) ** 1.5

    lat = np.degrees(lat0 + y / m0)
    lon = np.degrees(lon0 + x / (n0 * math.cos(lat0)))
    return lon, lat, z.copy()


def write_gis_csv(path: str | Path, xyz: np.ndarray,
                  rgb: Optional[np.ndarray] = None, *,
                  origin_lat: Optional[float] = None,
                  origin_lon: Optional[float] = None,
                  thin_voxel: float = 0.0,
                  precision: int = 3) -> Path:
    """Write a coloured cloud as QGIS-openable delimited text (+ CRS sidecars).

    With ``origin_lat``/``origin_lon`` the local ENU coordinates are also
    reprojected to WGS84 lon/lat and the columns become
    ``east,north,z,lon,lat,ele[,red,green,blue]`` — QGIS' "Add Delimited Text
    Layer" reads the ``lon``/``lat`` pair as EPSG:4326 point geometry, colour
    included. Without an origin the local frame is written verbatim
    (``x,y,z[,red,green,blue]``).

    ``thin_voxel > 0`` voxel-downsamples first (delegating to
    ``pointcloud_io.voxel_downsample``) so the *thinned* GIS export and the full
    one share one code path. A ``.csvt`` (column types) and, when georeferenced,
    a ``.prj`` (EPSG:4326 WKT) sidecar are written next to the CSV.

    Returns the CSV path.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    xyz = np.asarray(xyz, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f'xyz must be (N,3), got {xyz.shape}')
    if rgb is not None:
        rgb = np.asarray(rgb, dtype=np.uint8)
        if rgb.shape != xyz.shape:
            raise ValueError('rgb must match xyz shape')

    if thin_voxel > 0.0:
        from pointcloud_io import voxel_downsample
        xyz, rgb = voxel_downsample(xyz, thin_voxel, rgb)

    geo = origin_lat is not None and origin_lon is not None
    if geo:
        lon, lat, ele = enu_to_wgs84(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                                     float(origin_lat), float(origin_lon))
        cols = [xyz[:, 0], xyz[:, 1], xyz[:, 2], lon, lat, ele]
        names = ['east', 'north', 'z', 'lon', 'lat', 'ele']
        types = ['Real'] * 6
    else:
        cols = [xyz[:, 0], xyz[:, 1], xyz[:, 2]]
        names = ['x', 'y', 'z']
        types = ['Real'] * 3

    fmts = ['%.8f' if nm in ('lon', 'lat') else f'%.{precision}f' for nm in names]
    if rgb is not None:
        cols += [rgb[:, 0], rgb[:, 1], rgb[:, 2]]
        names += ['red', 'green', 'blue']
        types += ['Integer'] * 3
        fmts += ['%d', '%d', '%d']

    data = np.column_stack(cols)
    with open(path, 'w') as fh:
        fh.write(','.join(names) + '\n')
        np.savetxt(fh, data, delimiter=',', fmt=fmts)

    # QGIS reads .csvt (same basename) for column types; .prj documents the CRS.
    path.with_suffix('.csvt').write_text(
        ','.join(f'"{t}"' for t in types) + '\n')
    if geo:
        path.with_suffix('.prj').write_text(WGS84_WKT + '\n')
    return path


def _build_arg_parser():
    import argparse
    p = argparse.ArgumentParser(
        description='Export a coloured PLY cloud to GIS delimited text.')
    p.add_argument('input', help='input .ply (xyz [+ rgb])')
    p.add_argument('output', help='output .csv (sidecars written alongside)')
    p.add_argument('--origin-lat', type=float, default=None,
                   help='WGS84 origin latitude [deg]; enables lon/lat columns')
    p.add_argument('--origin-lon', type=float, default=None,
                   help='WGS84 origin longitude [deg]')
    p.add_argument('--thin-voxel', type=float, default=0.0,
                   help='voxel size [m] to downsample before export (0=off)')
    return p


def main(argv=None) -> int:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from pointcloud_io import read_ply_xyz
    args = _build_arg_parser().parse_args(argv)
    xyz, rgb = read_ply_xyz(args.input)
    out = write_gis_csv(args.output, xyz, rgb,
                        origin_lat=args.origin_lat,
                        origin_lon=args.origin_lon,
                        thin_voxel=args.thin_voxel)
    n = 'unknown' if xyz is None else len(xyz)
    print(f'wrote {out} ({n} input points, '
          f'{"georeferenced" if args.origin_lat is not None else "local frame"})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
