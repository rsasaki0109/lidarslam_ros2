#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Check the product installation, or inspect one rosbag2 when provided."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import sys
from typing import Any, Callable, Mapping, Sequence


sys.dont_write_bytecode = True

SCRIPT_DIR = Path(__file__).resolve().parent
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/system-doctor-v1.schema.json'
)
SUPPORTED_ROS_DISTROS = ('humble', 'jazzy')
DEFAULT_MIN_FREE_SPACE_GIB = 8.0
GIB = 1024 ** 3


class DoctorError(ValueError):
    """The read-only system diagnosis could not be completed."""


def _gib_shortfall(byte_count: int) -> str:
    """Format a positive byte shortfall without understating it."""
    hundredths = (byte_count * 100 + GIB - 1) // GIB
    return f'{hundredths / 100:.2f} GiB'


def _source_layout(script_dir: Path) -> bool:
    root = script_dir.parent
    return (
        (root / 'VERSION').is_file()
        and (root / 'lidarslam' / 'package.xml').is_file()
    )


def _product_paths(script_dir: Path) -> dict[str, Path | bool]:
    source_layout = _source_layout(script_dir)
    if source_layout:
        product_root = script_dir.parent
        manifest = product_root / 'lidarslam' / 'product-runtime-files.txt'
        version = product_root / 'VERSION'
    else:
        product_root = script_dir.parent
        manifest = product_root / 'product-runtime-files.txt'
        version = product_root / 'VERSION'
    return {
        'source_layout': source_layout,
        'product_root': product_root,
        'manifest': manifest,
        'version': version,
        'runtime_root': script_dir,
    }


def _read_runtime_inventory(manifest: Path) -> list[str]:
    try:
        lines = manifest.read_text(encoding='utf-8').splitlines()
    except OSError as exc:
        raise DoctorError(f'cannot read product runtime inventory: {exc}') from exc
    names = sorted({
        line.strip()
        for line in lines
        if line.strip() and not line.lstrip().startswith('#')
    })
    if not names:
        raise DoctorError('product runtime inventory is empty')
    for name in names:
        path = Path(name)
        if path.is_absolute() or '..' in path.parts or path.name != name:
            raise DoctorError(
                f'product runtime inventory contains an unsafe name: {name!r}')
    return names


def _read_version(path: Path) -> str:
    try:
        version = path.read_text(encoding='utf-8').strip()
    except OSError as exc:
        raise DoctorError(f'cannot read product version: {exc}') from exc
    if not version:
        raise DoctorError('product version is empty')
    return version


def _module_available(name: str) -> bool:
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, AttributeError, ValueError):
        return False


def _existing_storage_root(path: Path) -> Path:
    candidate = path.expanduser().resolve(strict=False)
    while not candidate.exists():
        parent = candidate.parent
        if parent == candidate:
            raise DoctorError('cannot find an existing parent for demo storage')
        candidate = parent
    if not candidate.is_dir():
        candidate = candidate.parent
    return candidate


def _local_install_markers(product_root: Path) -> list[Path]:
    candidates = [
        product_root / 'install' / 'share' / 'lidarslam' / 'product' / 'VERSION',
        product_root / 'install' / 'lidarslam' / 'share' / 'lidarslam'
        / 'product' / 'VERSION',
        product_root.parent / 'install' / 'share' / 'lidarslam'
        / 'product' / 'VERSION',
        product_root.parent / 'install' / 'lidarslam' / 'share'
        / 'lidarslam' / 'product' / 'VERSION',
    ]
    if product_root.parent.name == 'src':
        workspace = product_root.parent.parent
        candidates.extend([
            workspace / 'install' / 'share' / 'lidarslam' / 'product'
            / 'VERSION',
            workspace / 'install' / 'lidarslam' / 'share' / 'lidarslam'
            / 'product' / 'VERSION',
        ])
    return candidates


def _installed_prefix_detected(
    *,
    source_layout: bool,
    product_root: Path,
    environment: Mapping[str, str],
) -> bool:
    if not source_layout:
        return True
    if any(path.is_file() for path in _local_install_markers(product_root)):
        return True
    for value in environment.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if not value:
            continue
        marker = Path(value) / 'share' / 'lidarslam' / 'product' / 'VERSION'
        if marker.is_file():
            return True
    return False


def _finding(code: str, message: str, next_action: str) -> dict[str, str]:
    return {
        'code': code,
        'message': message,
        'next_action': next_action,
    }


def _select_next_action(
    findings: Sequence[Mapping[str, str]],
) -> dict[str, str] | None:
    """Select the first dependency-ordered recovery without hiding findings."""
    if not findings:
        return None
    selected = findings[0]
    return {
        'code': selected['code'],
        'reason': selected['message'],
        'action': selected['next_action'],
    }


def build_system_report(
    *,
    script_dir: Path = SCRIPT_DIR,
    demo_dir: Path,
    min_free_space_gib: float,
    environment: Mapping[str, str] | None = None,
    command_lookup: Callable[[str], str | None] = shutil.which,
    module_available: Callable[[str], bool] = _module_available,
    disk_usage: Callable[[Path], Any] = shutil.disk_usage,
) -> dict[str, Any]:
    """Return one privacy-bounded, read-only installation diagnosis."""
    if not min_free_space_gib > 0:
        raise DoctorError('minimum free space must be greater than zero')
    env = dict(os.environ if environment is None else environment)
    paths = _product_paths(script_dir)
    source_layout = bool(paths['source_layout'])
    product_root = Path(paths['product_root'])
    runtime_root = Path(paths['runtime_root'])
    runtime_inventory = _read_runtime_inventory(Path(paths['manifest']))
    missing_runtime = [
        name for name in runtime_inventory if not (runtime_root / name).is_file()
    ]
    version = _read_version(Path(paths['version']))
    installed_prefix = _installed_prefix_detected(
        source_layout=source_layout,
        product_root=product_root,
        environment=env,
    )
    ros_distro = env.get('ROS_DISTRO') or None
    ros_supported = ros_distro in SUPPORTED_ROS_DISTROS
    ros2_cli = command_lookup('ros2') is not None
    rosbag2_python = module_available('rosbag2_py')
    storage_root = _existing_storage_root(demo_dir)
    available_bytes = int(disk_usage(storage_root).free)
    minimum_bytes = int(min_free_space_gib * GIB)
    storage_sufficient = available_bytes >= minimum_bytes
    additional_bytes_required = max(minimum_bytes - available_bytes, 0)

    findings: list[dict[str, str]] = []
    if missing_runtime:
        findings.append(_finding(
            'product-runtime-incomplete',
            f'{len(missing_runtime)} installed product helper(s) are missing.',
            'Rebuild or reinstall this exact lidar_slam_ros2 version, then run '
            'lidarslam-map doctor again.',
        ))
    if source_layout and not installed_prefix:
        findings.append(_finding(
            'source-build-required',
            'This source checkout has no matching lidar_slam_ros2 install.',
            'Run bash scripts/source_quickstart.sh --build-only, then use the '
            'absolute lidarslam-map path it prints.',
        ))
    if ros_distro is None:
        findings.append(_finding(
            'ros-environment-missing',
            'No supported ROS 2 environment is active for this command.',
            'Use the absolute installed lidarslam-map launcher, or source '
            '/opt/ros/humble/setup.bash or /opt/ros/jazzy/setup.bash.',
        ))
    elif not ros_supported:
        findings.append(_finding(
            'ros-distro-unsupported',
            f'ROS_DISTRO={ros_distro} is outside the Humble/Jazzy contract.',
            'Open a fresh terminal and use a Humble or Jazzy installation.',
        ))
    if not ros2_cli:
        findings.append(_finding(
            'ros2-cli-missing',
            'The ros2 command is unavailable.',
            'Install the selected ROS 2 desktop/base tools and run this doctor '
            'again from the supported environment.',
        ))
    if not rosbag2_python:
        findings.append(_finding(
            'rosbag2-python-missing',
            'The rosbag2_py runtime needed for safe bag inspection is unavailable.',
            'Install the rosbag2 Python package for the selected ROS distro, '
            'then run lidarslam-map doctor again.',
        ))
    if not storage_sufficient:
        shortfall = _gib_shortfall(additional_bytes_required)
        findings.append(_finding(
            'demo-storage-low',
            f'The selected filesystem needs at least {shortfall} more free '
            f'space for the fixed demo ({min_free_space_gib:g} GiB minimum).',
            f'Free at least {shortfall} on the selected filesystem, then run: '
            'lidarslam-map doctor',
        ))

    return {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'mode': 'system',
        'status': 'ready' if not findings else 'action_required',
        'version': version,
        'product': {
            'layout': 'source' if source_layout else 'installed',
            'runtime_file_count': len(runtime_inventory),
            'missing_runtime_files': missing_runtime,
            'installed_prefix_detected': installed_prefix,
        },
        'ros': {
            'distro': ros_distro,
            'supported': ros_supported,
            'ros2_cli_available': ros2_cli,
            'rosbag2_python_available': rosbag2_python,
        },
        'storage': {
            'available_bytes': available_bytes,
            'minimum_free_bytes': minimum_bytes,
            'additional_bytes_required': additional_bytes_required,
            'sufficient_for_fixed_demo': storage_sufficient,
        },
        'findings': findings,
        'next_action': _select_next_action(findings),
        'commands': {
            'system_doctor': 'lidarslam-map doctor',
            'bag_doctor': 'lidarslam-map doctor /path/to/rosbag2',
            'fixed_demo': 'lidarslam-map demo',
            'own_bag': 'lidarslam-map start /path/to/rosbag2',
        },
        'network_accessed': False,
        'writes_performed': False,
    }


def render_system_report(report: Mapping[str, Any]) -> str:
    status = str(report['status']).replace('_', ' ').upper()
    product = report['product']
    ros = report['ros']
    storage = report['storage']
    distro = ros['distro'] or 'not active'
    lines = [
        'lidarslam-map doctor — installation and demo readiness',
        f'Status:       {status}',
        f"Version:      {report['version']}",
        f"Product:      {product['layout']} "
        f"({product['runtime_file_count']} runtime helpers)",
        f'ROS 2:        {distro}',
        f"ros2 CLI:     {'ready' if ros['ros2_cli_available'] else 'missing'}",
        'Bag reader:   '
        f"{'ready' if ros['rosbag2_python_available'] else 'missing'}",
        'Demo storage: '
        f"{storage['available_bytes'] / GIB:.1f} GiB free; "
        f"{storage['minimum_free_bytes'] / GIB:.1f} GiB required"
        + (
            '; free '
            f"{_gib_shortfall(storage['additional_bytes_required'])} more"
            if storage['additional_bytes_required'] else
            ''
        ),
    ]
    findings = report['findings']
    if findings:
        next_action = report['next_action']
        lines.extend([
            '',
            'Do this now:',
            f"  {next_action['action']}",
            f"  Why: [{next_action['code']}] {next_action['reason']}",
        ])
        remaining = findings[1:]
        if remaining:
            lines.extend([
                '',
                'Other checks detected:',
            ])
        for finding in remaining:
            lines.append(f"  [{finding['code']}] {finding['message']}")
        if remaining:
            lines.append(
                '  Rerun doctor after the first action; it will choose the '
                'next blocker.'
            )
    else:
        commands = report['commands']
        lines.extend([
            '',
            'Ready for the verified paths:',
            f"  Fixed demo: {commands['fixed_demo']}",
            f"  Own bag:    {commands['own_bag']}",
            '  No launch-file or YAML edits are needed for the own-bag path.',
            (
                '  It detects the recorded inputs and stops with a reason '
                'code when no maintained path is safe.'
            ),
        ])
    lines.extend([
        '',
        'To inspect topics, frames, timestamps, and sensor compatibility:',
        f"  {report['commands']['bag_doctor']}",
        '',
        'This check used no network and wrote no files.',
    ])
    return '\n'.join(lines)


def _parser() -> argparse.ArgumentParser:
    profile_registry = SCRIPT_DIR / 'product_profiles.py'
    try:
        profile_help = runpy.run_path(str(profile_registry))['PROFILE_HELP']
    except (ImportError, KeyError, OSError, RuntimeError, TypeError) as exc:
        raise DoctorError(
            f'cannot load maintained profile help: {exc}') from exc
    profile_lines = '\n'.join(
        f'  {profile_id}: {description}'
        for profile_id, description in profile_help
    )
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Check this lidar_slam_ros2 installation without a bag, or inspect '
            'one rosbag2 directory when provided.'
        ),
        epilog=(
            'Without rosbag2_dir, doctor checks the installed product surface, '
            'supported ROS environment, bag reader, and fixed-demo storage. '
            'It uses no network and writes no files. With rosbag2_dir, it '
            'checks topics, fields, frames, timestamps, and maintained profiles.\n\n'
            'Maintained bag profiles:\n'
            f'{profile_lines}'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        'rosbag2_dir',
        nargs='?',
        type=Path,
        help='Optional directory containing metadata.yaml.',
    )
    output_group = parser.add_mutually_exclusive_group()
    output_group.add_argument(
        '--json',
        action='store_true',
        help='Emit machine-readable JSON.',
    )
    output_group.add_argument(
        '--public-json',
        action='store_true',
        help=(
            'With a bag, emit path-free, topic/frame-name-free evidence for '
            'a reviewed public issue.'
        ),
    )
    parser.add_argument(
        '--demo-dir',
        type=Path,
        default=Path.cwd(),
        metavar='<dir>',
        help='Filesystem to check for fixed-demo storage (system mode only).',
    )
    parser.add_argument(
        '--min-free-space-gib',
        type=float,
        default=DEFAULT_MIN_FREE_SPACE_GIB,
        metavar='<GiB>',
        help='Required fixed-demo free space (default: 8; system mode only).',
    )
    parser.add_argument(
        '--help-all',
        action='store_true',
        help='Show this help; doctor has no hidden advanced options.',
    )
    return parser


def _system_option_requested(argv: Sequence[str]) -> bool:
    return any(
        item in {'--demo-dir', '--min-free-space-gib'}
        or item.startswith('--demo-dir=')
        or item.startswith('--min-free-space-gib=')
        for item in argv
    )


def _delegate_bag_doctor(
    bag: Path,
    *,
    json_output: bool,
    public_json_output: bool = False,
) -> int:
    helper = SCRIPT_DIR / 'preflight_autoware_map_bag.py'
    if not helper.is_file():
        raise DoctorError(f'bag preflight helper is missing: {helper.name}')
    command = [sys.executable, str(helper), str(bag)]
    if json_output:
        command.append('--json')
    elif public_json_output:
        command.append('--public-json')
    try:
        return subprocess.run(command, check=False).returncode
    except OSError as exc:
        raise DoctorError(f'cannot start bag preflight: {exc}') from exc


def main(argv: Sequence[str] | None = None) -> int:
    raw_args = list(sys.argv[1:] if argv is None else argv)
    try:
        parser = _parser()
    except DoctorError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 70
    args = parser.parse_args(raw_args)
    if args.help_all:
        if len(raw_args) != 1:
            parser.error('--help-all cannot be combined with other arguments')
        parser.print_help()
        return 0
    if args.rosbag2_dir is not None:
        if _system_option_requested(raw_args):
            parser.error(
                '--demo-dir and --min-free-space-gib apply only when no '
                'rosbag2_dir is provided')
        try:
            return _delegate_bag_doctor(
                args.rosbag2_dir,
                json_output=args.json,
                public_json_output=args.public_json,
            )
        except DoctorError as exc:
            print(f'error: {exc}', file=sys.stderr)
            return 70
    if args.public_json:
        parser.error('--public-json requires rosbag2_dir')
    try:
        report = build_system_report(
            demo_dir=args.demo_dir,
            min_free_space_gib=args.min_free_space_gib,
        )
    except DoctorError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 70
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(render_system_report(report))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
