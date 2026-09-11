#!/usr/bin/env python3
"""Export a coloured SLAM point cloud to LAS / LAZ (the GIS-native format).

Third export off the coloured-cloud "hub" (after ``map_export.py`` delimited
text and ``mesh_export.py`` meshes): write LAS 1.2 point-format-2 (XYZ +
intensity + 16-bit RGB), the format CloudCompare / QGIS / ArcGIS / PDAL open
natively, with real colour and mm precision.

Coordinates stay in the SLAM local ENU frame (metres) — the project's
georeferencing convention is ``projector_type: local`` (local metres + a WGS84
origin), matching the lanelet2 map. When an origin is given it is embedded as a
LAS VLR so the cloud carries its Earth anchor without pretending metres are
degrees. ``laspy`` is imported lazily so the numpy-only exports never need it;
LAZ additionally needs a backend (``lazrs`` or ``laszip``).

See roadmap §12/§13 and ``docs/research/3dgs-postprocess-map-design.md``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

_ORIGIN_USER_ID = 'LIDARSLAM'
_ORIGIN_RECORD_ID = 2


def _laspy():
    try:
        import laspy  # noqa: F401
    except ImportError as exc:  # pragma: no cover - env-dependent
        raise ImportError(
            'las_export needs laspy (`pip install laspy`; add `lazrs` for '
            '.laz). The numpy-only exports in map_export.py do not.') from exc
    return laspy


def write_las(path: str | Path, xyz: np.ndarray,
              rgb: Optional[np.ndarray] = None, *,
              intensity: Optional[np.ndarray] = None,
              origin_lat: Optional[float] = None,
              origin_lon: Optional[float] = None,
              thin_voxel: float = 0.0, scale: float = 0.001) -> Path:
    """Write ``xyz`` (+ optional ``rgb``) to a LAS/LAZ file (format from suffix).

    ``rgb`` (uint8) is stored as LAS 16-bit colour (``*257`` so 255->65535).
    ``scale`` is the coordinate quantum in metres (0.001 = mm). Offsets are set
    to the cloud minimum so the int32 storage keeps full precision. With
    ``thin_voxel > 0`` the cloud is voxel-downsampled first (shared code path
    with the other exports). ``origin_lat``/``origin_lon``, if given, are
    embedded as a ``LIDARSLAM`` VLR recording the local frame's WGS84 origin.

    A ``.laz`` suffix triggers LAZ compression (needs a ``lazrs``/``laszip``
    backend). Returns the output path.
    """
    laspy = _laspy()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    xyz = np.asarray(xyz, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f'xyz must be (N,3), got {xyz.shape}')
    if rgb is not None:
        rgb = np.asarray(rgb, dtype=np.uint8)
        if rgb.shape != xyz.shape:
            raise ValueError('rgb must match xyz shape')
    if intensity is not None:
        intensity = np.asarray(intensity)
        if intensity.shape[0] != xyz.shape[0]:
            raise ValueError('intensity must have one value per point')

    if thin_voxel > 0.0:
        from pointcloud_io import voxel_downsample
        # voxel_downsample carries rgb; intensity is dropped when thinning.
        xyz, rgb = voxel_downsample(xyz, thin_voxel, rgb)
        intensity = None

    header = laspy.LasHeader(point_format=2, version='1.2')
    header.scales = [scale, scale, scale]
    if xyz.shape[0] > 0:
        header.offsets = np.floor(xyz.min(axis=0))
    else:
        header.offsets = [0.0, 0.0, 0.0]
    if origin_lat is not None and origin_lon is not None:
        blob = f'enu_origin_wgs84 lat={origin_lat!r} lon={origin_lon!r}'.encode()
        header.vlrs.append(laspy.vlrs.VLR(
            user_id=_ORIGIN_USER_ID, record_id=_ORIGIN_RECORD_ID,
            description='ENU local-frame WGS84 origin', record_data=blob))

    las = laspy.LasData(header)
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    if intensity is not None:
        las.intensity = np.clip(intensity, 0, 65535).astype(np.uint16)
    if rgb is not None:
        las.red = rgb[:, 0].astype(np.uint16) * 257
        las.green = rgb[:, 1].astype(np.uint16) * 257
        las.blue = rgb[:, 2].astype(np.uint16) * 257

    las.write(str(path))
    return path


def read_origin(path: str | Path) -> Optional[tuple[float, float]]:
    """Return the ``(lat, lon)`` embedded by :func:`write_las`, or ``None``."""
    laspy = _laspy()
    with laspy.open(str(path)) as fh:
        for vlr in fh.header.vlrs:
            if (vlr.user_id.strip('\x00') == _ORIGIN_USER_ID
                    and vlr.record_id == _ORIGIN_RECORD_ID):
                text = bytes(vlr.record_data).decode('utf-8', 'replace')
                lat = float(text.split('lat=')[1].split()[0])
                lon = float(text.split('lon=')[1].split()[0])
                return lat, lon
    return None


def _build_arg_parser():
    import argparse
    p = argparse.ArgumentParser(
        description='Export a coloured PLY cloud to LAS/LAZ for GIS.')
    p.add_argument('input', help='input .ply (xyz [+ rgb])')
    p.add_argument('output', help='output .las / .laz')
    p.add_argument('--origin-lat', type=float, default=None,
                   help='WGS84 origin latitude [deg] to embed as a VLR')
    p.add_argument('--origin-lon', type=float, default=None,
                   help='WGS84 origin longitude [deg] to embed as a VLR')
    p.add_argument('--thin-voxel', type=float, default=0.0,
                   help='voxel size [m] to downsample before export (0=off)')
    p.add_argument('--scale', type=float, default=0.001,
                   help='coordinate quantum [m] (default 0.001 = mm)')
    return p


def main(argv=None) -> int:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from pointcloud_io import read_ply_xyz
    args = _build_arg_parser().parse_args(argv)
    xyz, rgb = read_ply_xyz(args.input)
    out = write_las(args.output, xyz, rgb, origin_lat=args.origin_lat,
                    origin_lon=args.origin_lon, thin_voxel=args.thin_voxel,
                    scale=args.scale)
    print(f'wrote {out} ({0 if xyz is None else len(xyz)} input points, '
          f'coloured={rgb is not None}, '
          f'origin={"yes" if args.origin_lat is not None else "no"})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
