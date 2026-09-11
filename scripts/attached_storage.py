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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
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

"""Discover or resolve attached Linux storage without changing mount state."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
from typing import Any, Iterable, Sequence


class StorageError(ValueError):
    """Raised when an explicitly selected storage device is not usable."""


def _optional_text(value: Any) -> str | None:
    if not isinstance(value, str):
        return None
    printable = ''.join(
        character if character.isprintable() else ' '
        for character in value
    )
    cleaned = ' '.join(printable.split())
    return cleaned or None


def _valid_device_path(value: Any) -> bool:
    return (
        isinstance(value, str)
        and value.startswith('/dev/')
        and value.isprintable()
        and not any(character.isspace() for character in value)
    )


def _is_mounted(value: Any) -> bool:
    values = value if isinstance(value, list) else [value]
    return any(_optional_text(item) is not None for item in values)


def _flatten_devices(items: Any) -> Iterable[dict[str, Any]]:
    if not isinstance(items, list):
        return
    for item in items:
        if not isinstance(item, dict):
            continue
        yield item
        yield from _flatten_devices(item.get('children'))


def _lsblk_document(
    command: list[str],
    *,
    runner: Any = None,
) -> dict[str, Any] | None:
    run = runner or subprocess.run
    try:
        result = run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
        if result.returncode != 0:
            return None
        document = json.loads(result.stdout)
    except (
        json.JSONDecodeError,
        OSError,
        subprocess.SubprocessError,
        TypeError,
    ):
        return None
    return document if isinstance(document, dict) else None


def discover_unmounted_candidates(
    minimum_bytes: int,
    *,
    runner: Any = None,
    executable: str | None = None,
) -> list[dict[str, Any]]:
    """Return suitable hotplug filesystems without mounting or probing them."""
    if isinstance(minimum_bytes, bool) or minimum_bytes < 1:
        raise StorageError('--minimum-bytes must be a positive integer')
    lsblk = executable or shutil.which('lsblk')
    if lsblk is None:
        return []
    document = _lsblk_document(
        [
            lsblk,
            '--json',
            '--bytes',
            '--output',
            (
                'PATH,PKNAME,TYPE,FSTYPE,SIZE,MOUNTPOINTS,LABEL,'
                'MODEL,TRAN,RO,RM,HOTPLUG'
            ),
        ],
        runner=runner,
    )
    if document is None:
        return []
    devices = list(_flatten_devices(document.get('blockdevices')))
    by_name = {
        Path(path).name: item
        for item in devices
        if _valid_device_path(path := item.get('path'))
    }
    candidates = []
    for item in devices:
        path = item.get('path')
        filesystem = _optional_text(item.get('fstype'))
        size = item.get('size')
        if (
            not _valid_device_path(path)
            or filesystem is None
            or isinstance(size, bool)
            or not isinstance(size, int)
            or size < minimum_bytes
            or item.get('ro') is not False
            or _is_mounted(item.get('mountpoints'))
        ):
            continue
        parent_name = _optional_text(item.get('pkname'))
        parent = by_name.get(Path(parent_name).name, {}) if parent_name else {}
        transport = _optional_text(item.get('tran')) or _optional_text(
            parent.get('tran')
        )
        hotplug = item.get('hotplug') is True or parent.get('hotplug') is True
        if not hotplug and transport != 'usb':
            continue
        candidates.append({
            'device': path,
            'filesystem': filesystem,
            'partition_bytes': size,
            'label': _optional_text(item.get('label')),
            'model': (
                _optional_text(item.get('model'))
                or _optional_text(parent.get('model'))
            ),
            'transport': transport,
            'capacity_status': 'UNVERIFIED_UNTIL_MOUNTED',
        })
    return sorted(
        candidates,
        key=lambda candidate: (
            -candidate['partition_bytes'], candidate['device']
        ),
    )


def device_mountpoint(
    device: str,
    *,
    runner: Any = None,
    executable: str | None = None,
) -> Path:
    """Resolve exactly one current mountpoint without changing mount state."""
    if not _valid_device_path(device):
        raise StorageError(
            f'--device must be an absolute printable /dev path: {device}'
        )
    lsblk = executable or shutil.which('lsblk')
    if lsblk is None:
        raise StorageError('cannot resolve --device because lsblk is unavailable')
    document = _lsblk_document(
        [lsblk, '--json', '--output', 'PATH,MOUNTPOINTS', device],
        runner=runner,
    )
    if document is None:
        raise StorageError(f'cannot inspect destination device {device}')
    mountpoints = []
    for item in _flatten_devices(document.get('blockdevices')):
        if item.get('path') != device:
            continue
        values = item.get('mountpoints')
        if not isinstance(values, list):
            values = [values]
        for value in values:
            if value is None:
                continue
            if (
                not isinstance(value, str)
                or not value
                or not value.isprintable()
                or not Path(value).is_absolute()
            ):
                raise StorageError(
                    f'destination device returned an invalid mountpoint: '
                    f'{device}'
                )
            mountpoints.append(value)
    mountpoints = sorted(set(mountpoints))
    if not mountpoints:
        mount_command = shlex.join(['udisksctl', 'mount', '-b', device])
        raise StorageError(
            f'destination device is not mounted: {device}. Run '
            f'{mount_command}, then retry the same command.'
        )
    if len(mountpoints) != 1:
        raise StorageError(
            f'destination device has ambiguous mountpoints: {device}: '
            f'{mountpoints}'
        )
    return Path(mountpoints[0]).resolve()


def candidate_summary(candidate: dict[str, Any]) -> str:
    """Render one sanitized, single-line candidate description."""
    details = []
    if candidate.get('model'):
        details.append(candidate['model'])
    details.extend([
        candidate['filesystem'],
        f"{candidate['partition_bytes']} bytes",
    ])
    if candidate.get('label'):
        details.append(f"label {candidate['label']}")
    return f"{candidate['device']} ({', '.join(details)})"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    discover = subparsers.add_parser(
        'discover',
        help='list sufficiently large attached-but-unmounted filesystems',
    )
    discover.add_argument('--minimum-bytes', required=True, type=int)
    discover.add_argument(
        '--format',
        choices=('json', 'lines'),
        default='json',
        help=(
            "'lines' emits the first device and its display text on separate "
            'lines for a Bash caller'
        ),
    )
    mountpoint = subparsers.add_parser(
        'mountpoint',
        help='print the one current mountpoint for an explicit device',
    )
    mountpoint.add_argument('--device', required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == 'discover':
            candidates = discover_unmounted_candidates(args.minimum_bytes)
            if args.format == 'lines':
                if candidates:
                    print(candidates[0]['device'])
                    print(candidate_summary(candidates[0]))
            else:
                print(json.dumps(candidates, indent=2, sort_keys=True))
            return 0
        print(device_mountpoint(args.device))
        return 0
    except StorageError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
