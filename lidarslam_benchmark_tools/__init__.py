"""Installed Python surface for the repository's benchmark/evidence tools.

The implementation files remain in the single canonical ``scripts/`` source
tree.  ``graph_based_slam`` projects that tree into this package at install
time; it does not maintain a second hand-copied source tree.  A source-tree
package path is supported for unit tests and direct development, while
``require_installed_surface`` lets release checks reject an accidental source
checkout fallback.
"""

from __future__ import annotations

import os
from pathlib import Path
import importlib.util
from pathlib import PurePosixPath
from typing import Final


PACKAGE_NAME: Final[str] = 'lidarslam_benchmark_tools'
PACKAGE_VERSION: Final[str] = '0.1.0'
PACKAGE_SURFACE_SCHEMA: Final[str] = 'lidarslam-benchmark-python-surface-v1'

_PACKAGE_DIR = Path(__file__).resolve().parent
_SOURCE_SCRIPTS_DIR = _PACKAGE_DIR.parent / 'scripts'
_SOURCE_SURFACE = _SOURCE_SCRIPTS_DIR.is_dir() and (
    _SOURCE_SCRIPTS_DIR / 'verify_autoware_map.py'
).is_file()

# In a source checkout the package contains only this metadata file.  Extend
# the package path to the canonical scripts tree instead of copying wrappers
# or maintaining divergent module implementations.
if _SOURCE_SURFACE:
    __path__.append(str(_SOURCE_SCRIPTS_DIR))


def is_source_surface() -> bool:
    """Return whether imports are resolving from the repository checkout."""
    return _SOURCE_SURFACE


def package_root() -> Path:
    """Return the source root or installed package share root.

    Installed CMake packages place profiles/configuration under
    ``<prefix>/share/graph_based_slam``.  Keeping this lookup here gives the
    benchmark modules one deterministic root in both source and installed
    execution without consulting ``sys.path`` or a repository checkout.
    """
    if _SOURCE_SURFACE:
        return _SOURCE_SCRIPTS_DIR.parent
    for candidate in (_PACKAGE_DIR, *_PACKAGE_DIR.parents):
        installed_root = candidate / 'share' / 'graph_based_slam'
        if installed_root.is_dir():
            return installed_root
    raise RuntimeError(
        'installed benchmark package has no graph_based_slam share root; '
        'refusing repository-path fallback'
    )


def require_installed_surface() -> Path:
    """Require the CMake-installed surface and return its package root.

    This is intentionally explicit rather than enforced at import time: source
    unit tests and direct development tools may still import the canonical
    scripts tree.  Release/clean-install smoke tests call this guard before
    exercising installed benchmark/evidence modules.
    """
    if _SOURCE_SURFACE:
        raise RuntimeError(
            'benchmark Python import resolved to repository scripts; '
            'use the installed lidarslam_benchmark_tools package'
        )
    return package_root()


def module_path(module_name: str) -> Path:
    """Resolve one installed benchmark module without using repository paths.

    Historical launchers load an earlier immutable implementation by file
    path.  ``importlib`` is the only source of truth for that path once the
    package is installed; this prevents a clean install from accidentally
    reaching back into a checkout or an old ``scripts/`` tree.
    """
    if not module_name.isidentifier():
        raise ValueError(f'invalid benchmark module name: {module_name!r}')
    qualified = f'{PACKAGE_NAME}.{module_name}'
    spec = importlib.util.find_spec(qualified)
    if spec is None or not spec.origin or spec.origin in {'built-in', 'frozen'}:
        raise RuntimeError(f'benchmark module is not installed: {qualified}')
    path = Path(spec.origin).resolve()
    if path.suffix != '.py' or not path.is_file():
        raise RuntimeError(f'benchmark module has no regular Python source: {path}')
    return path


def resolve_benchmark_resource(resource: str, *, executable: bool = False) -> Path:
    """Resolve one canonical benchmark resource in source or install mode.

    Resource names are repository-relative (for example,
    ``scripts/sample_container_process_rss.py``).  In a source checkout the
    name is resolved below ``scripts/``; in an installed package it is
    resolved below the installed package directory.  The resolver rejects
    absolute/traversal names, symlinks, missing files, and paths that resolve
    outside the selected canonical root so a caller cannot silently fall back
    to an old checkout or an ambiguous ``share/.../scripts`` tree.
    """
    if (not isinstance(resource, str) or not resource or '\\' in resource or
            '\x00' in resource):
        raise RuntimeError(f'invalid benchmark resource name: {resource!r}')
    raw_parts = resource.split('/')
    if any(part in {'', '.', '..'} for part in raw_parts):
        raise RuntimeError(
            f'benchmark resource must be a safe relative path: {resource!r}')
    parsed = PurePosixPath(resource)
    if parsed.is_absolute() or any(part in {'', '.', '..'} for part in parsed.parts):
        raise RuntimeError(f'benchmark resource must be a safe relative path: {resource!r}')
    parts = parsed.parts
    if parts and parts[0] == 'scripts':
        parts = parts[1:]
    if not parts:
        raise RuntimeError(f'benchmark resource is empty: {resource!r}')
    root = _SOURCE_SCRIPTS_DIR if _SOURCE_SURFACE else _PACKAGE_DIR
    root = root.resolve(strict=True)
    candidate = root.joinpath(*parts)
    if candidate.is_symlink() or not candidate.is_file():
        raise RuntimeError(f'benchmark resource is missing or non-regular: {resource!r}')
    resolved = candidate.resolve(strict=True)
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise RuntimeError(f'benchmark resource escapes canonical root: {resource!r}') from exc
    if resolved.is_symlink() or not resolved.is_file():
        raise RuntimeError(f'benchmark resource is not a regular file: {resource!r}')
    if executable and not os.access(resolved, os.X_OK):
        raise RuntimeError(f'benchmark resource is not executable: {resource!r}')
    return resolved


__all__ = [
    'PACKAGE_NAME',
    'PACKAGE_VERSION',
    'PACKAGE_SURFACE_SCHEMA',
    'is_source_surface',
    'package_root',
    'module_path',
    'resolve_benchmark_resource',
    'require_installed_surface',
]
