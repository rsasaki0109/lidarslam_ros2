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

"""Keep bundled dependency metadata aligned with runtime requirements."""

from __future__ import annotations

from pathlib import Path
import re
import xml.etree.ElementTree as ET

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
PRODUCT_PACKAGE = REPO_ROOT / 'lidarslam' / 'package.xml'
RKO_PACKAGE = REPO_ROOT / 'Thirdparty' / 'rko_lio' / 'package.xml'
PACKAGE_VERSION = re.compile(r'^[0-9]+\.[0-9]+\.[0-9]+$')
OFFICIAL_RKO_BASELINE = (0, 3, 2)


def _version_tuple(value: str, source: Path) -> tuple[int, int, int]:
    assert PACKAGE_VERSION.fullmatch(value), (
        f'{source}: expected a three-component numeric package version, '
        f'got {value!r}'
    )
    return tuple(int(component) for component in value.split('.'))


def _package_version(path: Path) -> tuple[int, int, int]:
    version = ET.parse(path).getroot().findtext('version')
    assert version is not None, f'{path}: missing package version'
    return _version_tuple(version.strip(), path)


def _runtime_minimum(path: Path, dependency_name: str) -> tuple[int, int, int]:
    root = ET.parse(path).getroot()
    dependencies = [
        dependency
        for tag in ('depend', 'exec_depend')
        for dependency in root.findall(tag)
        if (dependency.text or '').strip() == dependency_name
    ]
    assert len(dependencies) == 1, (
        f'{path}: expected exactly one runtime dependency named '
        f'{dependency_name!r}, found {len(dependencies)}'
    )
    minimum = dependencies[0].get('version_gte')
    assert minimum is not None, (
        f'{path}: {dependency_name} must declare version_gte'
    )
    return _version_tuple(minimum.strip(), path)


def test_bundled_rko_lio_satisfies_declared_runtime_minimum():
    if not RKO_PACKAGE.is_file():
        pytest.skip('release source layout does not bundle the RKO-LIO submodule')

    bundled = _package_version(RKO_PACKAGE)
    required = _runtime_minimum(PRODUCT_PACKAGE, 'rko_lio')

    assert required >= OFFICIAL_RKO_BASELINE, (
        'the declared RKO-LIO runtime minimum must not regress below the '
        'official v0.3.2 product baseline'
    )
    assert bundled >= required, (
        'bundled RKO-LIO package version '
        f'{".".join(map(str, bundled))} is older than the declared runtime '
        f'minimum {".".join(map(str, required))}'
    )


@pytest.mark.parametrize(
    ('bundled', 'required', 'aligned'),
    [
        ((0, 3, 2), (0, 3, 2), True),
        ((0, 4, 0), (0, 3, 2), True),
        ((0, 3, 1), (0, 3, 2), False),
        ((0, 2, 99), (0, 3, 0), False),
    ],
)
def test_numeric_package_version_ordering(bundled, required, aligned):
    assert (bundled >= required) is aligned
