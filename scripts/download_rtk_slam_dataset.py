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

"""Acquire exact, resumable RTK-SLAM benchmark inputs without surprises."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
from typing import Any, Sequence


HF_REPO = 'Willyzw/rtk-slam-dataset'
HF_REVISION = '87619d2da3f345109b9a2b0d3a192a8596b4d2d3'
HF_RESOLVE = (
    f'https://huggingface.co/datasets/{HF_REPO}/resolve/{HF_REVISION}'
)
EVAL_REPO_URL = 'https://github.com/Willyzw/rtk-slam-eval.git'
EVAL_REPO_COMMIT = 'f2921a58caf5a87c1f4f73b48c6f2a5e35f92924'
# A pinned checkout measured 92,874,519 bytes including its Git directory.
# Keep headroom for filesystem and Git-version variance; the separate reserve
# below still applies to the complete acquisition plan.
EVAL_ASSETS_PLANNING_BYTES = 150_000_000
MINIMUM_SPACE_RESERVE_BYTES = 1_000_000_000

# Exact identities from the official Hugging Face tree API at HF_REVISION.
# The large-file ``lfs.oid`` values are SHA-256 digests. Metadata SHA-256
# values were computed from the immutable revision URLs.
SEQUENCES: dict[str, dict[str, dict[str, Any]]] = {
    'construction_seq2': {
        'db3': {
            'relative_path': (
                'ros2/construction_seq2/construction_seq2.db3'
            ),
            'bytes': 10_656_112_640,
            'sha256': (
                '9e808703a57d7be6afa6a37abb8f5d65'
                'c6566f71f4864cd4c24cb01f6ab82af5'
            ),
        },
        'metadata': {
            'relative_path': 'ros2/construction_seq2/metadata.yaml',
            'bytes': 1_930,
            'sha256': (
                '2cc6cb1e4a53b2d1c371499e489582b8'
                '8737b1dbce79f01df4b6c811e43db8ff'
            ),
        },
    },
    'construction_seq1': {
        'db3': {
            'relative_path': (
                'ros2/construction_seq1/construction_seq1.db3'
            ),
            'bytes': 13_180_936_192,
            'sha256': (
                'adf7e5e8f8d73a0a3a0c09f80d846ca3'
                'a88446809ee391a818260d4bd3d03a7a'
            ),
        },
        'metadata': {
            'relative_path': 'ros2/construction_seq1/metadata.yaml',
            'bytes': 1_930,
            'sha256': (
                'ad9b7f8b01862305740abd83e671dbc1c'
                '080c8d6e2b82a9e05b1152306382318'
            ),
        },
    },
    'stadtgarten_seq2': {
        'db3': {
            'relative_path': 'ros2/stadtgarten_seq2/stadtgarten_seq2.db3',
            'bytes': 16_793_665_536,
            'sha256': (
                'd303eeaa773ae1606ddbefb38c509009d'
                '9ef9c81a7d4bc6b68869db4287d27e2'
            ),
        },
        'metadata': {
            'relative_path': 'ros2/stadtgarten_seq2/metadata.yaml',
            'bytes': 1_928,
            'sha256': (
                '9538fe9ca6b3d496bd35eb13519e38136'
                'a4a86c2a857f1f0215a2ded6a62037c'
            ),
        },
    },
    'stadtgarten_seq1': {
        'db3': {
            'relative_path': 'ros2/stadtgarten_seq1/stadtgarten_seq1.db3',
            'bytes': 30_263_574_528,
            'sha256': (
                '6f674fff7182e54d2aa12cac36b0be36'
                'd022f67e8624b3df4d1acf8018fa6e5b'
            ),
        },
        'metadata': {
            'relative_path': 'ros2/stadtgarten_seq1/metadata.yaml',
            'bytes': 1_933,
            'sha256': (
                'b1fa518aead0436fd574db48b3425d3f'
                '1fdcb4a9e26a752f1be0733ed9af3aae'
            ),
        },
    },
}

EVAL_REQUIRED_PATHS = tuple(
    f'ground_truth/{sequence}.csv' for sequence in SEQUENCES
)


class AcquisitionError(RuntimeError):
    """An input cannot be acquired without operator action."""


def _human_gb(num_bytes: int) -> str:
    return f'{num_bytes / 1e9:.1f} GB'


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _absolute_path(raw_path: str) -> Path:
    return Path(os.path.abspath(os.path.expanduser(raw_path)))


def _path_exists(path: Path) -> bool:
    return os.path.lexists(path)


def _require_directory(path: Path, label: str) -> None:
    if path.is_symlink():
        raise AcquisitionError(f'{label} must not be a symlink: {path}')
    if _path_exists(path) and not path.is_dir():
        raise AcquisitionError(f'{label} is not a directory: {path}')


def _validate_destination_layout(
    dest_root: Path,
    sequences: Sequence[str],
    include_eval: bool,
) -> None:
    _require_directory(dest_root, 'destination')
    ros2_root = dest_root / 'ros2'
    if sequences:
        _require_directory(ros2_root, 'ROS2 destination')
    for sequence in sequences:
        _require_directory(
            ros2_root / sequence,
            f'{sequence} destination',
        )
    if include_eval:
        target = dest_root / 'rtk_slam_eval'
        if target.is_symlink():
            raise AcquisitionError(
                f'eval-assets destination must not be a symlink: {target}'
            )
        if _path_exists(target) and not target.is_dir():
            raise AcquisitionError(
                f'eval-assets destination is not a directory: {target}'
            )


def _available_bytes(path: Path) -> int:
    probe = path
    while not probe.exists() and probe.parent != probe:
        probe = probe.parent
    try:
        return shutil.disk_usage(probe).free
    except OSError as exc:
        raise AcquisitionError(
            f'cannot determine free space for destination {path}: {exc}'
        ) from exc


def _optional_text(value: Any) -> str | None:
    if not isinstance(value, str):
        return None
    printable = ''.join(
        character if character.isprintable() else ' '
        for character in value
    )
    cleaned = ' '.join(printable.split())
    return cleaned or None


def _is_mounted(value: Any) -> bool:
    if isinstance(value, list):
        return any(_optional_text(item) is not None for item in value)
    return _optional_text(value) is not None


def _discover_unmounted_storage_candidates(
    required_bytes: int,
    *,
    runner: Any = None,
    executable: str | None = None,
) -> list[dict[str, Any]]:
    """Return large attached filesystems without mounting or probing them."""
    lsblk = executable or shutil.which('lsblk')
    if lsblk is None:
        return []
    run = runner or subprocess.run
    try:
        result = run(
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
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
        if result.returncode != 0:
            return []
        document = json.loads(result.stdout)
    except (
        json.JSONDecodeError,
        OSError,
        subprocess.SubprocessError,
        TypeError,
    ):
        return []

    if not isinstance(document, dict):
        return []
    devices = document.get('blockdevices')
    if not isinstance(devices, list):
        return []
    by_name = {
        Path(path).name: item
        for item in devices
        if isinstance(item, dict)
        if isinstance((path := item.get('path')), str)
    }
    candidates = []
    for item in devices:
        if not isinstance(item, dict):
            continue
        path = item.get('path')
        filesystem = _optional_text(item.get('fstype'))
        size = item.get('size')
        if (
            not isinstance(path, str)
            or not path.startswith('/dev/')
            or not path.isprintable()
            or any(character.isspace() for character in path)
            or filesystem is None
            or isinstance(size, bool)
            or not isinstance(size, int)
            or size < required_bytes
            or item.get('ro') is not False
            or _is_mounted(item.get('mountpoints'))
        ):
            continue
        parent = by_name.get(item.get('pkname'), {})
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
        key=lambda item: (-item['partition_bytes'], item['device']),
    )


def _device_mountpoint(
    device: str,
    *,
    runner: Any = None,
    executable: str | None = None,
) -> Path:
    """Resolve one mounted block device without changing mount state."""
    if (
        not device.startswith('/dev/')
        or not device.isprintable()
        or any(character.isspace() for character in device)
    ):
        raise AcquisitionError(
            f'--dest-device must be an absolute /dev path: {device}'
        )
    lsblk = executable or shutil.which('lsblk')
    if lsblk is None:
        raise AcquisitionError(
            'cannot resolve --dest-device because lsblk is unavailable'
        )
    run = runner or subprocess.run
    try:
        result = run(
            [
                lsblk,
                '--json',
                '--output',
                'PATH,MOUNTPOINTS',
                device,
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
        if result.returncode != 0:
            raise AcquisitionError(
                f'cannot inspect destination device {device}'
            )
        document = json.loads(result.stdout)
    except AcquisitionError:
        raise
    except (
        json.JSONDecodeError,
        OSError,
        subprocess.SubprocessError,
        TypeError,
    ) as exc:
        raise AcquisitionError(
            f'cannot inspect destination device {device}: {exc}'
        ) from exc
    if not isinstance(document, dict):
        raise AcquisitionError(
            f'cannot inspect destination device {device}: invalid lsblk JSON'
        )
    mountpoints = []
    for item in document.get('blockdevices', []):
        if not isinstance(item, dict) or item.get('path') != device:
            continue
        values = item.get('mountpoints')
        if not isinstance(values, list):
            values = [values]
        mountpoints.extend(
            text for value in values
            if (text := _optional_text(value)) is not None
        )
    mountpoints = sorted(set(mountpoints))
    if not mountpoints:
        mount_command = shlex.join(['udisksctl', 'mount', '-b', device])
        raise AcquisitionError(
            f'destination device is not mounted: {device}. Run '
            f'{mount_command}, then retry the same command.'
        )
    if len(mountpoints) != 1:
        raise AcquisitionError(
            f'destination device has ambiguous mountpoints: {device}: '
            f'{mountpoints}'
        )
    return Path(mountpoints[0]).resolve()


def _storage_recovery(
    required_bytes: int,
    sequences: Sequence[str],
    include_eval: bool,
) -> dict[str, Any]:
    candidates = _discover_unmounted_storage_candidates(required_bytes)
    destination_args = (
        ['--dest-device', candidates[0]['device']]
        if candidates else ['--dest', '/mnt/large/rtk_slam']
    )
    base_command = [
        'python3',
        'scripts/download_rtk_slam_dataset.py',
        *_selection_arguments(sequences, include_eval),
        *destination_args,
    ]
    preflight_command = shlex.join([*base_command, '--dry-run'])
    live_command = shlex.join(base_command)
    mount_command = None
    if candidates and shutil.which('udisksctl') is not None:
        mount_command = shlex.join([
            'udisksctl', 'mount', '-b', candidates[0]['device'],
        ])
    if mount_command is not None:
        next_action = mount_command
    elif candidates:
        next_action = (
            f"mount attached filesystem {candidates[0]['device']}, then run: "
            f'{preflight_command}'
        )
    else:
        next_action = (
            f'mount a filesystem with at least {required_bytes} free bytes, '
            f'then run: {preflight_command}'
        )
    return {
        'minimum_free_bytes': required_bytes,
        'unmounted_candidates': candidates,
        'mount_command': mount_command,
        'preflight_command': preflight_command,
        'live_command': live_command,
        'next_action': next_action,
    }


def _artifact_plan(
    sequence: str,
    kind: str,
    artifact: dict[str, Any],
    dest_root: Path,
) -> dict[str, Any]:
    relative_path = str(artifact['relative_path'])
    target = dest_root / relative_path
    expected_bytes = int(artifact['bytes'])
    expected_sha256 = str(artifact['sha256'])

    if target.is_symlink():
        raise AcquisitionError(
            f'{sequence} {kind} must not be a symlink: {target}'
        )
    if _path_exists(target) and not target.is_file():
        raise AcquisitionError(
            f'{sequence} {kind} is not a regular file: {target}'
        )

    existing_bytes = target.stat().st_size if target.is_file() else 0
    if existing_bytes > expected_bytes:
        raise AcquisitionError(
            f'{sequence} {kind} is larger than the official file: '
            f'expected {expected_bytes} bytes, got {existing_bytes}: '
            f'{target}. '
            'Move the file aside and rerun.'
        )

    if existing_bytes == expected_bytes:
        actual_sha256 = _sha256(target)
        if actual_sha256 != expected_sha256:
            raise AcquisitionError(
                f'{sequence} {kind} SHA-256 mismatch: expected '
                f'{expected_sha256}, got {actual_sha256}: {target}. '
                'Move the file aside and rerun.'
            )
        action = 'reuse-verified'
    elif existing_bytes:
        action = 'resume'
    else:
        action = 'download'

    return {
        'sequence': sequence,
        'kind': kind,
        'path': str(target),
        'source_url': f'{HF_RESOLVE}/{relative_path}',
        'expected_bytes': expected_bytes,
        'expected_sha256': expected_sha256,
        'existing_bytes': existing_bytes,
        'transfer_bytes': expected_bytes - existing_bytes,
        'action': action,
    }


def _git_output(arguments: Sequence[str], cwd: Path | None = None) -> str:
    try:
        result = subprocess.run(
            ['git', *arguments],
            cwd=cwd,
            check=False,
            capture_output=True,
            text=True,
            timeout=60,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise AcquisitionError(f'cannot inspect eval assets with git: {exc}') \
            from exc
    if result.returncode != 0:
        detail = result.stderr.strip() or 'git returned no error text'
        raise AcquisitionError(f'cannot inspect eval assets: {detail}')
    return result.stdout.strip()


def _verify_eval_assets(target: Path) -> None:
    head = _git_output(['rev-parse', 'HEAD'], cwd=target)
    if head != EVAL_REPO_COMMIT:
        raise AcquisitionError(
            f'eval assets are at {head}, expected {EVAL_REPO_COMMIT}: '
            f'{target}. Choose a different --dest or move this checkout aside.'
        )
    dirty = _git_output(
        ['status', '--porcelain', '--untracked-files=no'],
        cwd=target,
    )
    if dirty:
        raise AcquisitionError(
            f'eval assets contain modified tracked files: {target}. '
            'Choose a different --dest or restore the pinned checkout.'
        )
    for relative_path in EVAL_REQUIRED_PATHS:
        path = target / relative_path
        if path.is_symlink() or not path.is_file():
            raise AcquisitionError(
                f'pinned eval assets are missing a required regular file: '
                f'{path}'
            )


def _eval_plan(dest_root: Path, include_eval: bool) -> dict[str, Any]:
    target = dest_root / 'rtk_slam_eval'
    if not include_eval:
        return {
            'requested': False,
            'action': 'skip',
            'path': str(target),
            'repository': EVAL_REPO_URL,
            'commit': EVAL_REPO_COMMIT,
            'planning_bytes': 0,
        }
    if target.is_dir():
        git_path = target / '.git'
        if git_path.is_symlink() or not git_path.is_dir():
            raise AcquisitionError(
                f'eval-assets directory is not a Git checkout: {target}'
            )
        _verify_eval_assets(target)
        action = 'reuse-verified'
        planning_bytes = 0
    else:
        action = 'fetch-pinned'
        planning_bytes = EVAL_ASSETS_PLANNING_BYTES
    return {
        'requested': True,
        'action': action,
        'path': str(target),
        'repository': EVAL_REPO_URL,
        'commit': EVAL_REPO_COMMIT,
        'planning_bytes': planning_bytes,
    }


def _selection_arguments(
    sequences: Sequence[str],
    include_eval: bool,
) -> list[str]:
    if not sequences:
        return ['--eval-assets-only']
    value = 'all' if list(sequences) == list(SEQUENCES) else sequences[0]
    arguments = ['--sequence', value]
    if include_eval:
        arguments.append('--eval-assets')
    return arguments


def build_plan(
    dest_root: Path,
    sequences: Sequence[str],
    include_eval: bool,
) -> dict[str, Any]:
    """Build a read-only acquisition and storage plan."""
    _validate_destination_layout(dest_root, sequences, include_eval)
    files = [
        _artifact_plan(sequence, kind, artifact, dest_root)
        for sequence in sequences
        for kind, artifact in SEQUENCES[sequence].items()
    ]
    eval_assets = _eval_plan(dest_root, include_eval)
    payload_bytes = sum(item['transfer_bytes'] for item in files)
    payload_bytes += eval_assets['planning_bytes']
    reserve_bytes = 0
    if payload_bytes:
        reserve_bytes = max(
            MINIMUM_SPACE_RESERVE_BYTES,
            (payload_bytes + 9) // 10,
        )
    required_peak_bytes = payload_bytes + reserve_bytes
    observed_free_bytes = _available_bytes(dest_root)
    shortage_bytes = max(required_peak_bytes - observed_free_bytes, 0)
    status = 'READY' if shortage_bytes == 0 else 'BLOCKED_INSUFFICIENT_SPACE'
    recovery = (
        _storage_recovery(required_peak_bytes, sequences, include_eval)
        if shortage_bytes else None
    )
    return {
        'schema_version': 1,
        'tool': 'download_rtk_slam_dataset',
        'dry_run': True,
        'side_effects_started': False,
        'dataset': {
            'repository': HF_REPO,
            'revision': HF_REVISION,
            'license': 'CC-BY-4.0',
        },
        'selected_sequences': list(sequences),
        'destination': str(dest_root),
        'files': files,
        'eval_assets': eval_assets,
        'storage': {
            'payload_remaining_bytes': payload_bytes,
            'reserve_bytes': reserve_bytes,
            'required_peak_bytes': required_peak_bytes,
            'observed_free_bytes': observed_free_bytes,
            'additional_bytes_required': shortage_bytes,
        },
        'storage_recovery': recovery,
        'status': status,
        'next_action': (
            recovery['next_action']
            if recovery is not None
            else 'run again without --dry-run'
        ),
    }


def _print_text_plan(plan: dict[str, Any]) -> None:
    print('RTK-SLAM acquisition plan')
    print(f'source:      {HF_REPO}@{HF_REVISION}')
    print(f"dest:        {plan['destination']}")
    for item in plan['files']:
        print(
            f"{item['sequence']}/{item['kind']}: "
            f"{item['action']}; {item['existing_bytes']} / "
            f"{item['expected_bytes']} bytes; SHA-256 "
            f"{item['expected_sha256']}"
        )
    eval_assets = plan['eval_assets']
    if eval_assets['requested']:
        print(
            'eval-assets: '
            f"{eval_assets['action']}; commit {eval_assets['commit']}"
        )
    storage = plan['storage']
    print(
        'space:       '
        f"{storage['required_peak_bytes']} bytes "
        f"({_human_gb(storage['required_peak_bytes'])}) additional required "
        '(remaining payload + reserve)'
    )
    print(
        'available:   '
        f"{storage['observed_free_bytes']} bytes "
        f"({_human_gb(storage['observed_free_bytes'])})"
    )
    print(
        'shortfall:   '
        f"{storage['additional_bytes_required']} bytes "
        f"({_human_gb(storage['additional_bytes_required'])})"
    )
    recovery = plan['storage_recovery']
    if recovery is not None:
        for candidate in recovery['unmounted_candidates']:
            description = candidate['model'] or candidate['label'] or 'device'
            print(
                'attached:    '
                f"{candidate['device']} ({description}, "
                f"{candidate['filesystem']}, "
                f"{candidate['partition_bytes']} bytes); unmounted, free "
                'space unverified'
            )
        if recovery['mount_command'] is not None:
            print(f"mount:       {recovery['mount_command']}")
        print(f"preflight:   {recovery['preflight_command']}")
        print(f"after READY: {recovery['live_command']}")
    print(f"status:      {plan['status']}")
    print(f"next:        {plan['next_action']}")


def _verify_artifact(item: dict[str, Any]) -> None:
    path = Path(item['path'])
    if path.is_symlink() or not path.is_file():
        raise AcquisitionError(
            f'download did not create a regular file: {path}'
        )
    actual_bytes = path.stat().st_size
    if actual_bytes != item['expected_bytes']:
        raise AcquisitionError(
            f"{item['sequence']} {item['kind']} size mismatch after download: "
            f"expected {item['expected_bytes']} bytes, got {actual_bytes}: "
            f'{path}. The partial file was retained for a retry.'
        )
    actual_sha256 = _sha256(path)
    if actual_sha256 != item['expected_sha256']:
        raise AcquisitionError(
            f"{item['sequence']} {item['kind']} SHA-256 mismatch after "
            f"download: expected {item['expected_sha256']}, got "
            f'{actual_sha256}: {path}. Move the file aside and rerun.'
        )
    print(
        f'identity: PASS {path} ({actual_bytes} bytes, SHA-256 '
        f'{actual_sha256})'
    )


def _download(item: dict[str, Any]) -> None:
    path = Path(item['path'])
    path.parent.mkdir(parents=True, exist_ok=True)
    print(f"downloading: {item['source_url']}\n  -> {path}")
    subprocess.run(
        [
            'wget',
            '--continue',
            '--progress=dot:giga',
            '--output-document',
            str(path),
            item['source_url'],
        ],
        check=True,
    )


def _run_command(command: Sequence[str]) -> None:
    subprocess.run(list(command), check=True)


def fetch_eval_assets(item: dict[str, Any]) -> None:
    """Fetch the eval repository at one immutable detached commit."""
    target = Path(item['path'])
    if item['action'] == 'reuse-verified':
        print(f'eval-assets identity: PASS {target}@{EVAL_REPO_COMMIT}')
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    _run_command(['git', 'init', '--quiet', str(target)])
    _run_command([
        'git', '-C', str(target), 'remote', 'add', 'origin', EVAL_REPO_URL,
    ])
    _run_command([
        'git', '-C', str(target), 'fetch', '--depth', '1',
        'origin', EVAL_REPO_COMMIT,
    ])
    _run_command([
        'git', '-C', str(target), 'checkout', '--quiet', '--detach',
        EVAL_REPO_COMMIT,
    ])
    _verify_eval_assets(target)
    print(f'eval-assets identity: PASS {target}@{EVAL_REPO_COMMIT}')


def _require_tools(plan: dict[str, Any]) -> None:
    if any(item['action'] != 'reuse-verified' for item in plan['files']):
        if shutil.which('wget') is None:
            raise AcquisitionError(
                'required command not found: wget. Install wget, then rerun.'
            )
    if plan['eval_assets']['action'] == 'fetch-pinned':
        if shutil.which('git') is None:
            raise AcquisitionError(
                'required command not found: git. Install git, then rerun.'
            )


def execute_plan(plan: dict[str, Any]) -> None:
    """Execute a READY plan and verify every resulting identity."""
    _require_tools(plan)
    for item in plan['files']:
        if item['action'] == 'reuse-verified':
            print(
                f"identity: PASS {item['path']} "
                f"({item['expected_bytes']} bytes, SHA-256 "
                f"{item['expected_sha256']})"
            )
            continue
        _download(item)
        _verify_artifact(item)
    if plan['eval_assets']['requested']:
        fetch_eval_assets(plan['eval_assets'])


def _print_sequence_list() -> None:
    print(f'RTK-SLAM exact inputs at {HF_REVISION}')
    for sequence, artifacts in SEQUENCES.items():
        db3 = artifacts['db3']
        print(
            f"{sequence:20s} {db3['bytes']} bytes "
            f"({_human_gb(db3['bytes'])}) SHA-256 {db3['sha256']}"
        )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Download exact public RTK-SLAM bags and/or pinned eval assets.'
        ),
    )
    parser.add_argument(
        '--sequence',
        choices=[*SEQUENCES, 'all'],
        default='construction_seq2',
        help=(
            "ROS2 sequence to fetch, or 'all'. The default is the smallest "
            'sequence, construction_seq2.'
        ),
    )
    destination = parser.add_mutually_exclusive_group()
    destination.add_argument(
        '--dest',
        default=None,
        help=(
            'Destination root (default: datasets/rtk_slam, which is '
            'gitignored)'
        ),
    )
    destination.add_argument(
        '--dest-device',
        help=(
            'Mounted /dev path; resolve its current mountpoint and use the '
            'rtk_slam subdirectory'
        ),
    )
    parser.add_argument(
        '--eval-assets',
        action='store_true',
        help='Also fetch the pinned ground-truth and trajectory repository',
    )
    parser.add_argument(
        '--eval-assets-only',
        action='store_true',
        help='Fetch only pinned eval assets; skip the large bag download',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help=(
            'Print the exact local acquisition plan; use no network or writes'
        ),
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Emit the dry-run plan as JSON (requires --dry-run)',
    )
    parser.add_argument(
        '--list',
        action='store_true',
        help=(
            'List exact sequence identities, then exit without network or '
            'writes'
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point."""
    parser = _parser()
    args = parser.parse_args(argv)
    if args.json and not args.dry_run:
        parser.error('--json requires --dry-run')
    if args.list:
        if (
            args.dry_run
            or args.json
            or args.eval_assets
            or args.eval_assets_only
            or args.dest is not None
            or args.dest_device is not None
        ):
            parser.error('--list cannot be combined with acquisition options')
        _print_sequence_list()
        return 0

    sequences = [] if args.eval_assets_only else (
        list(SEQUENCES) if args.sequence == 'all' else [args.sequence]
    )
    include_eval = args.eval_assets or args.eval_assets_only
    try:
        dest_root = (
            _device_mountpoint(args.dest_device) / 'rtk_slam'
            if args.dest_device is not None
            else _absolute_path(args.dest or 'datasets/rtk_slam')
        )
        plan = build_plan(dest_root, sequences, include_eval)
    except AcquisitionError as exc:
        print(f'error: {exc}', file=sys.stderr)
        print(
            'hint: inspect the write-free plan with --dry-run, or choose a '
            'different --dest.',
            file=sys.stderr,
        )
        return 2

    if args.json:
        print(json.dumps(plan, indent=2, sort_keys=True))
    else:
        _print_text_plan(plan)

    if args.dry_run:
        print(
            'dry-run:     no files, network requests, Git fetches, or '
            'downloads were started',
            file=sys.stderr if args.json else sys.stdout,
        )
        return 0
    if plan['status'] != 'READY':
        print(
            'error: insufficient free space; no files, network requests, Git '
            'fetches, or downloads were started',
            file=sys.stderr,
        )
        return 2

    try:
        execute_plan(plan)
    except (AcquisitionError, OSError, subprocess.CalledProcessError) as exc:
        print(f'error: RTK-SLAM acquisition failed: {exc}', file=sys.stderr)
        return 1
    print(
        'done. Build references with scripts/generate_rtk_slam_reference.py '
        'and run scripts/run_rtk_slam_accuracy_suite.py.'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
