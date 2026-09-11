#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Derive a fail-closed apt package set for an installed ROS 2 product."""

from __future__ import annotations

import argparse
import importlib.util
import json
import re
import shutil
import subprocess
from collections import defaultdict
from pathlib import Path
from typing import Callable, Iterable, Sequence


ELF_MAGIC = b'\x7fELF'
PACKAGE_PATTERN = re.compile(r'^[a-z0-9][a-z0-9+.-]*(?::[a-z0-9]+)?$')
FORBIDDEN_RUNTIME_PACKAGES = {
    'build-essential',
    'cmake',
    'git',
    'python3-colcon-common-extensions',
    'python3-rosdep',
}
DEFAULT_COMMANDS = (
    'awk',
    'basename',
    'bash',
    'chown',
    'cp',
    'curl',
    'date',
    'df',
    'dirname',
    'find',
    'grep',
    'head',
    'id',
    'md5sum',
    'mkdir',
    'mv',
    'pgrep',
    'pkill',
    'ps',
    'python3',
    'readlink',
    'realpath',
    'rm',
    'ros2',
    'sed',
    'setsid',
    'sha256sum',
    'sleep',
    'sort',
    'stat',
    'tail',
    'tar',
    'tee',
    'timeout',
    'tr',
    'wc',
)
DEFAULT_MODULES = (
    'ament_index_python.packages',
    'imageio',
    'jsonschema',
    'launch',
    'launch_ros',
    'numpy',
    'rclpy',
    'rosbag2_py',
    'rosidl_runtime_py.utilities',
    'scipy',
    'sensor_msgs.msg',
    'tf2_ros',
    'yaml',
)
DEFAULT_PACKAGES = (
    'ca-certificates',
    'time',
)
MERGED_USR_DIRECTORY_PAIRS = (
    (Path('/bin'), Path('/usr/bin')),
    (Path('/sbin'), Path('/usr/sbin')),
    (Path('/lib'), Path('/usr/lib')),
    (Path('/lib64'), Path('/usr/lib64')),
)


class RuntimePackageError(RuntimeError):
    """Raised when the runtime closure cannot be proven safely."""


def required_ros_packages(ros_distro: str) -> tuple[str, ...]:
    """Return ROS plugins loaded through CLI or plugin discovery at runtime."""
    return (
        f'ros-{ros_distro}-ros2launch',
        f'ros-{ros_distro}-rosbag2',
        f'ros-{ros_distro}-rosbag2-storage-mcap',
    )


def _run(arguments: Sequence[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(arguments),
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )


def _is_elf(path: Path) -> bool:
    try:
        with path.open('rb') as stream:
            return stream.read(len(ELF_MAGIC)) == ELF_MAGIC
    except OSError:
        return False


def elf_files(root: Path) -> list[Path]:
    """Return deterministic, unique ELF paths under an install tree."""
    if not root.is_dir() or root.is_symlink():
        raise RuntimePackageError(f'install root is missing or unsafe: {root}')
    rows: dict[Path, Path] = {}
    for path in sorted(root.rglob('*')):
        if not path.is_file() or not _is_elf(path):
            continue
        try:
            resolved = path.resolve(strict=True)
            resolved.relative_to(root.resolve(strict=True))
        except (OSError, ValueError) as exc:
            raise RuntimePackageError(
                f'ELF path escapes install root: {path}'
            ) from exc
        rows.setdefault(resolved, path)
    if not rows:
        raise RuntimePackageError(
            f'install root contains no ELF files: {root}'
        )
    return [rows[key] for key in sorted(rows)]


def parse_ldd_output(output: str, label: str) -> set[Path]:
    """Extract absolute dependencies and reject unresolved libraries."""
    dependencies: set[Path] = set()
    missing = []
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if '=> not found' in line:
            missing.append(line.split('=>', 1)[0].strip())
            continue
        if '=>' in line:
            candidate = line.split('=>', 1)[1].strip().split()[0]
        else:
            candidate = line.split()[0]
        if candidate.startswith('/'):
            dependencies.add(Path(candidate))
    if missing:
        raise RuntimePackageError(
            f'{label} has unresolved shared libraries: {sorted(missing)}'
        )
    return dependencies


def linked_libraries(
    paths: Iterable[Path],
    runner: Callable[[Sequence[str]], subprocess.CompletedProcess[str]] = _run,
) -> set[Path]:
    """Resolve the dynamic-library closure reported by ldd."""
    libraries: set[Path] = set()
    for path in paths:
        result = runner(('ldd', str(path)))
        detail = (result.stdout + '\n' + result.stderr).strip()
        if result.returncode != 0:
            if (
                'not a dynamic executable' in detail
                or 'statically linked' in detail
            ):
                continue
            raise RuntimePackageError(
                f'ldd failed for {path}: {detail or result.returncode}'
            )
        libraries.update(parse_ldd_output(result.stdout, str(path)))
    return libraries


def parse_owner_output(output: str) -> set[str]:
    """Parse package names from dpkg-query -S output."""
    packages = set()
    for line in output.splitlines():
        if line.startswith('diversion by '):
            continue
        if ': ' not in line:
            continue
        owners = line.split(': ', 1)[0]
        for package in owners.split(', '):
            package = package.strip()
            if not PACKAGE_PATTERN.fullmatch(package):
                raise RuntimePackageError(
                    f'dpkg returned an unsafe package name: {package!r}'
                )
            packages.add(package)
    return packages


def owner_path_candidates(
    path: Path,
    directory_pairs: Iterable[tuple[Path, Path]] | None = None,
) -> list[Path]:
    """Return package-query paths, including verified merged-/usr aliases."""
    candidates: list[Path] = []

    def add(candidate: Path) -> None:
        if candidate not in candidates:
            candidates.append(candidate)

    add(path)
    try:
        add(path.resolve(strict=True))
    except OSError:
        pass

    pairs = (
        MERGED_USR_DIRECTORY_PAIRS
        if directory_pairs is None
        else directory_pairs
    )
    for legacy_root, usr_root in pairs:
        try:
            legacy_resolved = legacy_root.resolve(strict=True)
            usr_resolved = usr_root.resolve(strict=True)
            if legacy_resolved != usr_resolved:
                continue
        except OSError:
            continue
        for candidate in tuple(candidates):
            try:
                relative = candidate.relative_to(usr_root)
            except ValueError:
                continue
            legacy_candidate = legacy_root / relative
            try:
                if (
                    legacy_candidate.resolve(strict=True)
                    != candidate.resolve(strict=True)
                ):
                    continue
            except OSError:
                continue
            add(legacy_candidate)
    return candidates


def package_owners(
    path: Path,
    runner: Callable[[Sequence[str]], subprocess.CompletedProcess[str]] = _run,
) -> set[str]:
    """Return dpkg owners for a path, including merged-/usr aliases."""
    for candidate in owner_path_candidates(path):
        result = runner(('dpkg-query', '-S', str(candidate)))
        if result.returncode == 0:
            packages = parse_owner_output(result.stdout)
            if packages:
                return packages
    raise RuntimePackageError(f'no installed Debian package owns {path}')


def _installed_package(
    package: str,
    runner: Callable[[Sequence[str]], subprocess.CompletedProcess[str]] = _run,
) -> bool:
    if not PACKAGE_PATTERN.fullmatch(package):
        raise RuntimePackageError(f'unsafe explicit package name: {package!r}')
    result = runner((
        'dpkg-query', '-W', '-f=${db:Status-Status}', package,
    ))
    return result.returncode == 0 and result.stdout.strip() == 'installed'


def _module_path(module: str) -> Path:
    spec = importlib.util.find_spec(module)
    if (
        spec is None
        or not spec.origin
        or spec.origin in {'built-in', 'frozen'}
    ):
        raise RuntimePackageError(
            f'Python runtime module is unavailable: {module}'
        )
    path = Path(spec.origin)
    if not path.is_file():
        raise RuntimePackageError(
            'Python runtime module has no package-owned file: '
            f'{module}: {path}'
        )
    return path


def _command_path(command: str) -> Path:
    path = shutil.which(command)
    if not path:
        raise RuntimePackageError(f'runtime command is unavailable: {command}')
    return Path(path)


def collect_packages(
    install_root: Path,
    ros_distro: str,
    commands: Iterable[str] = DEFAULT_COMMANDS,
    modules: Iterable[str] = DEFAULT_MODULES,
    explicit_packages: Iterable[str] = DEFAULT_PACKAGES,
) -> tuple[list[str], dict[str, object]]:
    """Collect package names and a deterministic audit report."""
    if not re.fullmatch(r'[a-z][a-z0-9_]{2,19}', ros_distro):
        raise RuntimePackageError(f'invalid ROS distribution: {ros_distro!r}')
    reasons: dict[str, set[str]] = defaultdict(set)
    executables = elf_files(install_root)
    libraries = linked_libraries(executables)
    install_root_resolved = install_root.resolve(strict=True)
    external_libraries = []
    internal_libraries = []
    for library in sorted(libraries):
        try:
            library.resolve(strict=True).relative_to(install_root_resolved)
        except (OSError, ValueError):
            external_libraries.append(library)
        else:
            internal_libraries.append(library)

    for library in external_libraries:
        for package in package_owners(library):
            reasons[package].add(f'linked-library:{library}')
    for command in sorted(set(commands)):
        path = _command_path(command)
        for package in package_owners(path):
            reasons[package].add(f'command:{command}')
    for module in sorted(set(modules)):
        path = _module_path(module)
        for package in package_owners(path):
            reasons[package].add(f'python-module:{module}')

    candidates = set(explicit_packages)
    candidates.update(required_ros_packages(ros_distro))
    sqlite_candidates = (
        f'ros-{ros_distro}-rosbag2-storage-sqlite3',
        f'ros-{ros_distro}-rosbag2-storage-default-plugins',
    )
    installed_sqlite = [
        package for package in sqlite_candidates if _installed_package(package)
    ]
    if not installed_sqlite:
        raise RuntimePackageError(
            'no installed rosbag2 sqlite storage package was found'
        )
    candidates.add(installed_sqlite[0])
    for package in sorted(candidates):
        if not _installed_package(package):
            raise RuntimePackageError(
                f'explicit runtime package is not installed: {package}'
            )
        reasons[package].add('explicit-runtime-contract')

    packages = sorted(reasons)
    forbidden = sorted(FORBIDDEN_RUNTIME_PACKAGES.intersection(packages))
    if forbidden:
        raise RuntimePackageError(
            f'build-only packages entered runtime closure: {forbidden}'
        )
    report: dict[str, object] = {
        'schema_version': 1,
        'ros_distro': ros_distro,
        'elf_file_count': len(executables),
        'linked_library_count': len(libraries),
        'internal_linked_library_count': len(internal_libraries),
        'external_linked_library_count': len(external_libraries),
        'package_count': len(packages),
        'development_packages': [
            package for package in packages
            if package.split(':', 1)[0].endswith('-dev')
        ],
        'forbidden_build_packages': forbidden,
        'packages': packages,
        'reasons': {
            package: sorted(reasons[package]) for package in packages
        },
    }
    return packages, report


def parse_args() -> argparse.Namespace:
    """Parse the fixed-path runtime closure CLI contract."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install-root', type=Path, required=True)
    parser.add_argument('--ros-distro', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    """Collect and write a deterministic runtime package closure."""
    args = parse_args()
    packages, report = collect_packages(
        args.install_root.resolve(), args.ros_distro
    )
    args.output.write_text('\n'.join(packages) + '\n', encoding='utf-8')
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    print(f'runtime packages: {len(packages)}')
    print(f'ELF files scanned: {report["elf_file_count"]}')
    print(f'linked libraries: {report["linked_library_count"]}')
    print(f'development packages: {report["development_packages"]}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except RuntimePackageError as exc:
        raise SystemExit(f'runtime package collection failed: {exc}') from exc
