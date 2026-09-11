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
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""CLI regression tests for the NTU VIRAL demo dataset downloader."""

from __future__ import annotations

import json
import os
import pathlib
import shutil
import subprocess


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
DOWNLOAD_SCRIPT = REPO_ROOT / 'scripts' / 'download_ntu_viral_tnp01.sh'
BASH = shutil.which('bash')
assert BASH is not None


def _run_download(
    *args: str,
    env: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [BASH, str(DOWNLOAD_SCRIPT), *args],
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )


def _combined_output(result: subprocess.CompletedProcess[str]) -> str:
    return result.stdout + result.stderr


def _fake_lsblk(fake_bin: pathlib.Path, document: object) -> None:
    script = fake_bin / 'lsblk'
    payload = json.dumps(document)
    script.write_text(
        '#!/bin/sh\n'
        f"printf '%s\\n' '{payload}'\n",
        encoding='utf-8',
    )
    script.chmod(0o755)


def _fake_df(fake_bin: pathlib.Path, available_bytes: int = 1) -> None:
    script = fake_bin / 'df'
    script.write_text(
        '#!/bin/sh\n'
        "printf 'Filesystem 1-blocks Used Available Capacity Mounted on\\n'\n"
        f"printf 'fake 1000000 999999 {available_bytes} 100%% /\\n'\n",
        encoding='utf-8',
    )
    script.chmod(0o755)


def test_download_help_exits_successfully():
    result = _run_download('--help')
    output = _combined_output(result)

    assert result.returncode == 0
    assert 'download_ntu_viral_tnp01.sh' in output
    assert '--dry-run' in output
    assert '--dest-device' in output
    assert '--no-restamp' in output


def test_download_dry_run_is_write_and_network_free(tmp_path: pathlib.Path):
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    _fake_df(fake_bin)
    dest = tmp_path / 'dataset'
    env = dict(os.environ)
    env['PATH'] = f'{fake_bin}:{env["PATH"]}'

    result = _run_download('--dest', str(dest), '--dry-run', env=env)
    output = _combined_output(result)

    assert result.returncode == 0, output
    assert 'NTU VIRAL tnp_01 acquisition plan' in output
    assert '8736253605 bytes' in output
    assert '82588ea4f29e311447f3d716865a022b' in output
    assert '49209878965 bytes' in output
    assert 'shortfall:' in output
    assert 'no files, network requests, conversions, or downloads' in output
    assert not dest.exists()


def test_download_fails_before_network_when_space_is_insufficient(
    tmp_path: pathlib.Path,
):
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    _fake_df(fake_bin)
    dest = tmp_path / 'dataset'
    env = dict(os.environ)
    env['PATH'] = f'{fake_bin}:{env["PATH"]}'

    result = _run_download(
        '--dest',
        str(dest),
        '--no-convert',
        '--no-restamp',
        env=env,
    )
    output = _combined_output(result)

    assert result.returncode == 2
    assert 'BLOCKED_INSUFFICIENT_SPACE' in output
    assert 'insufficient free space' in result.stderr
    assert 'downloading zip' not in output
    assert not dest.exists()


def test_low_space_discovers_attached_device_and_preserves_options(
    tmp_path: pathlib.Path,
):
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    _fake_df(fake_bin)
    _fake_lsblk(fake_bin, {
        'blockdevices': [
            {
                'path': '/dev/sdz',
                'pkname': None,
                'fstype': None,
                'size': 2_000_000_000_000,
                'mountpoints': [None],
                'label': None,
                'model': 'Portable SSD',
                'tran': 'usb',
                'ro': False,
                'hotplug': True,
            },
            {
                'path': '/dev/sdz1',
                'pkname': 'sdz',
                'fstype': 'ext4',
                'size': 1_999_000_000_000,
                'mountpoints': [None],
                'label': 'bench',
                'model': None,
                'tran': None,
                'ro': False,
                'hotplug': True,
            },
        ],
    })
    env = dict(os.environ)
    env['PATH'] = f'{fake_bin}:{env["PATH"]}'
    dest = tmp_path / 'dataset'

    result = _run_download(
        '--dest', str(dest), '--keep-zip', '--no-convert', '--no-restamp',
        '--dry-run', env=env,
    )
    output = _combined_output(result)

    assert result.returncode == 0, output
    assert (
        'attached:    /dev/sdz1 '
        '(Portable SSD, ext4, 1999000000000 bytes, label bench); '
        'unmounted, free space unverified'
    ) in output
    assert (
        'preflight:   bash scripts/download_ntu_viral_tnp01.sh '
        '--dest-device /dev/sdz1 --keep-zip --no-convert --no-restamp '
        '--dry-run'
    ) in output
    assert (
        'after READY: bash scripts/download_ntu_viral_tnp01.sh '
        '--dest-device /dev/sdz1 --keep-zip --no-convert --no-restamp'
    ) in output
    assert not dest.exists()


def test_dest_device_requires_mount_before_planning(tmp_path: pathlib.Path):
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    _fake_lsblk(fake_bin, {
        'blockdevices': [{
            'path': '/dev/sdz1',
            'mountpoints': [None],
        }],
    })
    env = dict(os.environ)
    env['PATH'] = f'{fake_bin}:{env["PATH"]}'

    result = _run_download(
        '--dest-device', '/dev/sdz1', '--dry-run', env=env,
    )
    output = _combined_output(result)

    assert result.returncode == 2
    assert 'destination device is not mounted: /dev/sdz1' in output
    assert 'udisksctl mount -b /dev/sdz1' in output
    assert 'NTU VIRAL tnp_01 acquisition plan' not in output


def test_dest_device_resolves_mountpoint_and_appends_dataset_directory(
    tmp_path: pathlib.Path,
):
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    mountpoint = tmp_path / 'mounted volume'
    mountpoint.mkdir()
    _fake_lsblk(fake_bin, {
        'blockdevices': [{
            'path': '/dev/sdz1',
            'mountpoints': [str(mountpoint)],
        }],
    })
    env = dict(os.environ)
    env['PATH'] = f'{fake_bin}:{env["PATH"]}'

    result = _run_download(
        '--dest-device', '/dev/sdz1', '--dry-run', env=env,
    )
    output = _combined_output(result)

    assert result.returncode == 0, output
    assert f'dest:        {mountpoint}/ntu_viral' in output
    assert not (mountpoint / 'ntu_viral').exists()


def test_download_rejects_dest_and_dest_device_combination(
    tmp_path: pathlib.Path,
):
    result = _run_download(
        '--dest', str(tmp_path), '--dest-device', '/dev/sdz1', '--dry-run',
    )

    assert result.returncode == 2
    assert '--dest and --dest-device are mutually exclusive' in result.stderr


def test_download_rejects_cached_archive_with_wrong_identity(
    tmp_path: pathlib.Path,
):
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    fake_df = fake_bin / 'df'
    fake_df.write_text(
        '#!/bin/sh\n'
        "printf 'Filesystem 1-blocks Used Available Capacity Mounted on\\n'\n"
        "printf 'fake 100000000000 1 99999999999 1%% /\\n'\n",
        encoding='utf-8',
    )
    fake_df.chmod(0o755)
    required_commands = (
        'awk', 'dirname', 'find', 'head', 'md5sum', 'realpath', 'stat',
    )
    for command_name in required_commands:
        command_path = shutil.which(command_name)
        assert command_path is not None
        (fake_bin / command_name).symlink_to(command_path)
    env = dict(os.environ)
    env['PATH'] = str(fake_bin)
    dest = tmp_path / 'dataset'
    dest.mkdir()
    (dest / 'tnp_01.zip').write_bytes(b'not the official archive')

    result = _run_download(
        '--dest',
        str(dest),
        '--no-convert',
        '--no-restamp',
        env=env,
    )

    assert result.returncode == 1
    assert 'official archive size mismatch' in result.stderr
    assert 'extracting zip' not in _combined_output(result)


def test_download_reuses_extracted_bag_without_requiring_archive(
    tmp_path: pathlib.Path,
):
    dest = tmp_path / 'dataset'
    extracted = dest / 'tnp_01' / 'tnp_01'
    extracted.mkdir(parents=True)
    (extracted / 'tnp_01.bag').write_bytes(b'present')

    result = _run_download(
        '--dest',
        str(dest),
        '--no-convert',
        '--no-restamp',
    )
    output = _combined_output(result)

    assert result.returncode == 0, output
    assert 'archive:     not-needed' in output
    assert 'rosbag1:     reuse' in output
    assert 'space:       0.0 GB additional required' in output
    assert 'downloading zip' not in output
    assert not (dest / 'tnp_01.zip').exists()


def test_download_rejects_missing_dest_value_before_dependency_checks():
    result = _run_download('--dest', '--keep-zip')

    assert result.returncode == 2
    assert 'error: option requires a value: --dest' in result.stderr
    assert 'shift' not in result.stderr
    assert 'required command not found' not in result.stderr


def test_download_rejects_unknown_option_with_help_hint():
    result = _run_download('--bogus')

    assert result.returncode == 2
    assert 'error: unknown option: --bogus' in result.stderr
    assert 'download_ntu_viral_tnp01.sh --help' in result.stderr
    assert 'downloading zip' not in _combined_output(result)


def test_download_rejects_destination_file_before_download(
    tmp_path: pathlib.Path,
):
    dest_file = tmp_path / 'not_a_directory'
    dest_file.write_text('', encoding='utf-8')

    result = _run_download(
        '--dest',
        str(dest_file),
        '--no-convert',
        '--no-restamp',
    )

    assert result.returncode == 2
    assert 'error: destination path is not a directory:' in result.stderr
    assert 'downloading zip' not in _combined_output(result)


def test_download_rejects_no_convert_restamp_without_existing_rosbag2(
    tmp_path: pathlib.Path,
):
    result = _run_download('--dest', str(tmp_path / 'dataset'), '--no-convert')

    assert result.returncode == 2
    assert '--no-convert requires existing rosbag2 metadata' in result.stderr
    assert 'rosbags-convert' not in result.stderr
    assert 'downloading zip' not in _combined_output(result)
