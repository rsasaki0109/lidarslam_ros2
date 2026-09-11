"""Canonical package view of the Gaussian-splatting benchmark helpers.

The implementation remains in ``tools/gaussian_splatting`` in a source
checkout and is projected into this subpackage by the CMake install helper.
No ``sys.path`` mutation or duplicate source tree is used.
"""

from __future__ import annotations

from pathlib import Path


_PACKAGE_DIR = Path(__file__).resolve().parent
_SOURCE_DIR = _PACKAGE_DIR.parents[1] / 'tools' / 'gaussian_splatting'
if (_SOURCE_DIR / 'pointcloud_io.py').is_file():
    __path__.append(str(_SOURCE_DIR))
# develop relocated canonical sources to tools/colored_map/ (old directory
# holds deprecation shims). Expose the canonical tree for submodule lookup.
_COLORED_DIR = _PACKAGE_DIR.parents[1] / 'tools' / 'colored_map'
if (_COLORED_DIR / 'pointcloud_io.py').is_file():
    __path__.append(str(_COLORED_DIR))
