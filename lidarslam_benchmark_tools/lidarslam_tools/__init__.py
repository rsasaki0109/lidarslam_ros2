"""Canonical package view of the shared benchmark report/runtime helpers."""

from __future__ import annotations

from pathlib import Path


_PACKAGE_DIR = Path(__file__).resolve().parent
_SOURCE_DIR = _PACKAGE_DIR.parents[1] / 'scripts' / 'lidarslam_tools'
if (_SOURCE_DIR / '__init__.py').is_file():
    __path__.append(str(_SOURCE_DIR))

from .serialization import payload_to_json

__all__ = ['payload_to_json']
