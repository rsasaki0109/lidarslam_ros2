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
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Tests for stable v0.9 release-candidate publication."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'release_channel.py'
SPEC = importlib.util.spec_from_file_location('release_channel', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
CHANNEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHANNEL)


@pytest.mark.parametrize('version', ('0.1.0', '0.7.0', '0.8.99'))
def test_versions_before_v09_are_prereleases(version):
    assert CHANNEL.release_channel(version) == 'prerelease'


@pytest.mark.parametrize('version', ('0.9.0', '0.9.7', '1.0.0', '2.1.3'))
def test_v09_and_later_are_stable_publications(version):
    assert CHANNEL.release_channel(version) == 'stable'


@pytest.mark.parametrize('version', ('v0.9.0', '0.9', '00.9.0', '0.9.0-rc1'))
def test_noncanonical_versions_are_rejected(version):
    with pytest.raises(ValueError, match='canonical MAJOR.MINOR.PATCH'):
        CHANNEL.release_channel(version)


def test_cli_reports_current_stable_candidate():
    version = (ROOT / 'VERSION').read_text(encoding='utf-8').strip()
    result = subprocess.run(
        [sys.executable, str(SCRIPT), version],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert result.stdout == 'stable\n'
    assert result.stderr == ''


def test_release_workflow_uses_classified_channel():
    workflow = (
        ROOT / '.github' / 'workflows' / 'release.yml'
    ).read_text(encoding='utf-8')

    assert 'scripts/release_channel.py "${VERSION}"' in workflow
    assert 'prerelease=${PRERELEASE}' in workflow
    assert "needs.metadata.outputs.prerelease == 'true'" in workflow


def test_github_workflows_reject_duplicate_yaml_keys():
    """Workflow mappings must not rely on parser-specific duplicate-key rules."""

    class UniqueKeyLoader(yaml.SafeLoader):
        pass

    def construct_unique_mapping(loader, node, deep=False):
        loader.flatten_mapping(node)
        mapping = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if key in mapping:
                raise ValueError(
                    f'duplicate YAML key {key!r} at line '
                    f'{key_node.start_mark.line + 1}'
                )
            mapping[key] = loader.construct_object(value_node, deep=deep)
        return mapping

    UniqueKeyLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        construct_unique_mapping,
    )
    workflows = sorted((ROOT / '.github' / 'workflows').glob('*.yml'))
    assert workflows
    for path in workflows:
        try:
            yaml.load(
                path.read_text(encoding='utf-8'),
                Loader=UniqueKeyLoader,
            )
        except (ValueError, yaml.YAMLError) as exc:
            pytest.fail(f'{path.relative_to(ROOT)}: {exc}')
