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

"""Tests for exact live identity of a published onboarding release."""

from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
SCRIPT = SCRIPTS / 'check_published_onboarding_identity.py'
VERSION = '0.9.1'
COMMIT = 'a' * 40
HUMBLE_DIGEST = 'sha256:' + 'b' * 64
JAZZY_DIGEST = 'sha256:' + 'c' * 64


def _module():
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            'published_onboarding_identity', SCRIPT
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


def _release(status='PUBLISHED'):
    return {
        'status': status,
        'remote': {
            'tag_commit': COMMIT if status != 'NOT_PUBLISHED' else None,
        },
        'images': [
            {
                'tag': (
                    'ghcr.io/rsasaki0109/lidar_slam_ros2:'
                    'v0.9.1-humble'
                ),
                'status': 'PUBLISHED',
                'digest': HUMBLE_DIGEST,
            },
            {
                'tag': (
                    'ghcr.io/rsasaki0109/lidar_slam_ros2:'
                    'v0.9.1-jazzy'
                ),
                'status': 'PUBLISHED',
                'digest': JAZZY_DIGEST,
            },
        ],
    }


def _audit(module, release):
    return module.audit_identity(
        VERSION,
        COMMIT,
        HUMBLE_DIGEST,
        JAZZY_DIGEST,
        release_auditor=lambda version: release,
    )


def test_exact_published_commit_and_digests_are_ready():
    module = _module()
    report = _audit(module, _release())

    assert report['status'] == 'READY'
    assert report['finding_codes'] == []
    assert all(report['checks'].values())
    assert report['network_requested'] is True
    assert report['writes_performed'] is False


def test_stale_packet_commit_and_digest_are_not_ready():
    module = _module()
    release = _release()
    release['remote']['tag_commit'] = 'd' * 40
    release['images'][1]['digest'] = 'sha256:' + 'e' * 64
    report = _audit(module, release)

    assert report['status'] == 'NOT_READY'
    assert report['finding_codes'] == [
        'source-commit-mismatch',
        'jazzy-digest-mismatch',
    ]
    assert report['checks']['source_commit_matches'] is False
    assert report['checks']['jazzy_digest_matches'] is False


def test_missing_release_is_not_ready_without_inventing_identity():
    module = _module()
    release = _release('NOT_PUBLISHED')
    release['images'] = []
    report = _audit(module, release)

    assert report['status'] == 'NOT_READY'
    assert report['finding_codes'] == ['release-not-published']
    assert report['observed']['source_commit'] is None
    assert report['observed']['docker_digests'] == {
        'humble': None,
        'jazzy': None,
    }


def test_blocked_release_audit_remains_distinct():
    module = _module()
    report = _audit(module, _release('BLOCKED'))

    assert report['status'] == 'BLOCKED'
    assert report['finding_codes'] == ['release-audit-blocked']


def test_duplicate_or_mislabeled_release_images_fail_closed():
    module = _module()
    release = _release()
    changed = copy.deepcopy(release['images'][0])
    changed['digest'] = JAZZY_DIGEST
    release['images'][1] = changed
    report = _audit(module, release)

    assert report['status'] == 'NOT_READY'
    assert 'release-image-contract-invalid' in report['finding_codes']
    assert 'jazzy-digest-mismatch' in report['finding_codes']
