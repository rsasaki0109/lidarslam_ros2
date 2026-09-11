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
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
# OF SUCH DAMAGE.

"""Tests for the public ROS apt dependency readiness audit."""

from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_ros_apt_dependency_readiness.py'
SPEC = importlib.util.spec_from_file_location('ros_apt_readiness', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def _snapshot(main_version=None, testing_version=None):
    def channel(version):
        return {
            package: ([] if version is None else [f'{minimum}-1fixture'])
            for package, minimum in AUDIT.DEPENDENCY_MINIMUMS.items()
        }

    return {
        'inspected': True,
        'errors': [],
        'distros': {
            distro: {
                'main': channel(main_version),
                'testing': channel(testing_version),
            }
            for distro in AUDIT.DISTROS
        },
    }


def test_missing_testing_dependency_is_in_progress():
    """Unpublished required packages must keep the testing gate closed."""
    report = AUDIT.evaluate_readiness(_snapshot())

    assert report['status'] == 'IN_PROGRESS'
    assert report['channels']['main']['ready'] is False
    assert report['channels']['testing']['ready'] is False


def test_testing_dependencies_are_ready_before_main_sync():
    """Both distro testing channels can pass before their normal sync."""
    report = AUDIT.evaluate_readiness(
        _snapshot(testing_version='present')
    )

    assert report['status'] == 'TESTING_READY'
    assert report['channels']['testing']['ready'] is True
    assert report['channels']['main']['ready'] is False


def test_main_dependencies_are_ready_after_sync():
    """A main-ready report requires both channels and both distributions."""
    report = AUDIT.evaluate_readiness(
        _snapshot(main_version='present', testing_version='present')
    )

    assert report['status'] == 'MAIN_READY'
    assert all(check['status'] == 'PASS' for check in report['checks'])


def test_old_dependency_version_fails_minimum():
    """Published versions below the product minimum must not pass."""
    snapshot = _snapshot(testing_version='present')
    snapshot['distros']['jazzy']['testing']['rko-lio'] = [
        '0.3.0-1noble.fixture'
    ]

    report = AUDIT.evaluate_readiness(snapshot)

    assert report['status'] == 'IN_PROGRESS'
    check = next(
        item for item in report['checks']
        if item['id'] == 'jazzy-testing-rko-lio'
    )
    assert check['status'] == 'FAIL'


def test_probe_error_is_blocked():
    """Network or Docker errors are distinct from unpublished packages."""
    report = AUDIT.evaluate_readiness({
        'inspected': False,
        'errors': ['Docker is unavailable'],
        'distros': {},
    })

    assert report['status'] == 'BLOCKED'
    assert report['remote']['errors'] == ['Docker is unavailable']
