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

"""Regression tests for read-only attached-storage discovery."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'attached_storage.py'
SPEC = importlib.util.spec_from_file_location('attached_storage', SCRIPT)
STORAGE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(STORAGE)


def _runner(document: object, *, returncode: int = 0):
    def run(command, **kwargs):
        return type('Result', (), {
            'returncode': returncode,
            'stdout': json.dumps(document),
        })()

    return run


def test_discovery_selects_large_usb_filesystem_and_sanitizes_text():
    document = {
        'blockdevices': [
            {
                'path': '/dev/sdz',
                'pkname': None,
                'fstype': None,
                'size': 2_000_000,
                'mountpoints': [None],
                'label': None,
                'model': 'Portable\nSSD',
                'tran': 'usb',
                'ro': False,
                'hotplug': True,
            },
            {
                'path': '/dev/sdz1',
                'pkname': 'sdz',
                'fstype': 'ext4',
                'size': 1_999_000,
                'mountpoints': [None],
                'label': 'bench\x1b[31m',
                'model': None,
                'tran': None,
                'ro': False,
                'hotplug': True,
            },
            {
                'path': '/dev/nvme0n1p3',
                'pkname': 'nvme0n1',
                'fstype': 'ntfs',
                'size': 5_000_000,
                'mountpoints': [None],
                'label': None,
                'model': None,
                'tran': 'nvme',
                'ro': False,
                'hotplug': False,
            },
        ],
    }

    candidates = STORAGE.discover_unmounted_candidates(
        1_000_000,
        runner=_runner(document),
        executable='/usr/bin/lsblk',
    )

    assert candidates == [{
        'device': '/dev/sdz1',
        'filesystem': 'ext4',
        'partition_bytes': 1_999_000,
        'label': 'bench [31m',
        'model': 'Portable SSD',
        'transport': 'usb',
        'capacity_status': 'UNVERIFIED_UNTIL_MOUNTED',
    }]
    assert '\n' not in STORAGE.candidate_summary(candidates[0])
    assert '\x1b' not in STORAGE.candidate_summary(candidates[0])


@pytest.mark.parametrize('document', [None, [], {'blockdevices': 'bad'}])
def test_discovery_treats_malformed_documents_as_no_candidate(document):
    assert STORAGE.discover_unmounted_candidates(
        1,
        runner=_runner(document),
        executable='/usr/bin/lsblk',
    ) == []


def test_mountpoint_resolves_one_exact_path():
    document = {
        'blockdevices': [{
            'path': '/dev/sdz1',
            'mountpoints': ['/media/operator/bench'],
        }],
    }

    assert STORAGE.device_mountpoint(
        '/dev/sdz1',
        runner=_runner(document),
        executable='/usr/bin/lsblk',
    ) == Path('/media/operator/bench')


def test_mountpoint_requires_user_mount_and_rejects_ambiguous_paths():
    unmounted = {'blockdevices': [{
        'path': '/dev/sdz1', 'mountpoints': [None],
    }]}
    with pytest.raises(STORAGE.StorageError, match='udisksctl mount'):
        STORAGE.device_mountpoint(
            '/dev/sdz1',
            runner=_runner(unmounted),
            executable='/usr/bin/lsblk',
        )

    ambiguous = {'blockdevices': [{
        'path': '/dev/sdz1',
        'mountpoints': ['/media/a', '/media/b'],
    }]}
    with pytest.raises(STORAGE.StorageError, match='ambiguous mountpoints'):
        STORAGE.device_mountpoint(
            '/dev/sdz1',
            runner=_runner(ambiguous),
            executable='/usr/bin/lsblk',
        )

    invalid = {'blockdevices': [{
        'path': '/dev/sdz1',
        'mountpoints': ['/media/bench\nspoof'],
    }]}
    with pytest.raises(STORAGE.StorageError, match='invalid mountpoint'):
        STORAGE.device_mountpoint(
            '/dev/sdz1',
            runner=_runner(invalid),
            executable='/usr/bin/lsblk',
        )


def test_mountpoint_rejects_untrusted_device_text_before_lsblk():
    with pytest.raises(STORAGE.StorageError, match='printable /dev path'):
        STORAGE.device_mountpoint('/dev/sdz1\nspoof')
