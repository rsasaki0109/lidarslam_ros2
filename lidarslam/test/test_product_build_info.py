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

"""Tests for deterministic installed-product source provenance."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
GENERATOR_PATH = REPO_ROOT / 'scripts' / 'generate_product_build_info.py'
RUNNER_PATH = REPO_ROOT / 'scripts' / 'run_autoware_map_from_bag.py'


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _git(repo: Path, *args: str) -> str:
    completed = subprocess.run(
        ['git', *args],
        cwd=repo,
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def _repository(tmp_path: Path) -> tuple[Path, str]:
    repo = tmp_path / 'source'
    repo.mkdir()
    _git(repo, 'init')
    _git(repo, 'config', 'user.name', 'Product Test')
    _git(repo, 'config', 'user.email', 'product-test@example.invalid')
    (repo / 'tracked.txt').write_text('clean\n', encoding='utf-8')
    _git(repo, 'add', 'tracked.txt')
    _git(repo, 'commit', '-m', 'fixture')
    return repo, _git(repo, 'rev-parse', 'HEAD')


def test_generator_captures_clean_and_dirty_checkout_state(tmp_path: Path):
    """Tracked edits should change dirty state while untracked files do not."""
    generator = _load(GENERATOR_PATH, 'generate_product_build_info')
    repo, revision = _repository(tmp_path)

    assert generator.build_info(repo) == {
        'schema_version': 1,
        'revision': revision,
        'dirty': False,
        'source': 'git',
    }

    (repo / 'untracked.txt').write_text('ignored\n', encoding='utf-8')
    assert generator.build_info(repo)['dirty'] is False
    (repo / 'tracked.txt').write_text('changed\n', encoding='utf-8')
    assert generator.build_info(repo)['dirty'] is True


def test_generator_scopes_safe_directory_to_source(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """Container builds should not require a global Git configuration."""
    generator = _load(GENERATOR_PATH, 'generate_product_build_info')
    source = tmp_path / 'source'
    source.mkdir()
    observed = {}

    def fake_run(command, **kwargs):
        observed['command'] = command
        observed['kwargs'] = kwargs
        return subprocess.CompletedProcess(command, 0, '', '')

    monkeypatch.setattr(generator.subprocess, 'run', fake_run)
    generator._run_git(source, 'rev-parse', 'HEAD')

    assert observed['command'] == [
        'git',
        '-c',
        f'safe.directory={source}',
        'rev-parse',
        'HEAD',
    ]
    assert observed['kwargs']['cwd'] == source


def test_generator_supports_validated_override_and_unknown_archive(
    tmp_path: Path,
):
    """Git-free builds should use explicit identity or report unknown."""
    generator = _load(GENERATOR_PATH, 'generate_product_build_info')
    source = tmp_path / 'archive'
    source.mkdir()
    revision = 'a' * 40

    assert generator.build_info(source) == {
        'schema_version': 1,
        'revision': None,
        'dirty': None,
        'source': 'unknown',
    }
    assert generator.build_info(source, revision, 'false') == {
        'schema_version': 1,
        'revision': revision,
        'dirty': False,
        'source': 'override',
    }
    with pytest.raises(ValueError, match='40 hexadecimal'):
        generator.build_info(source, 'not-a-revision', 'false')
    with pytest.raises(ValueError, match='requires a source revision'):
        generator.build_info(source, None, 'false')


def test_generator_output_is_deterministic_and_atomic(tmp_path: Path):
    """Identical provenance should not rewrite the installed metadata."""
    generator = _load(GENERATOR_PATH, 'generate_product_build_info')
    output = tmp_path / 'product-build-info.json'
    payload = {
        'schema_version': 1,
        'revision': 'b' * 40,
        'dirty': False,
        'source': 'override',
    }

    generator.write_build_info(output, payload)
    first_stat = output.stat()
    generator.write_build_info(output, payload)

    assert output.stat().st_mtime_ns == first_stat.st_mtime_ns
    assert output.stat().st_ino == first_stat.st_ino
    assert json.loads(output.read_text(encoding='utf-8')) == payload
    assert not output.with_name(f'.{output.name}.tmp').exists()


def test_installed_runner_reads_valid_build_info_and_rejects_corruption(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """Installed manifests should use only a validated build identity."""
    runner = _load(RUNNER_PATH, 'run_autoware_map_from_bag_installed')
    build_info = tmp_path / 'product-build-info.json'
    monkeypatch.setattr(runner, 'SOURCE_LAYOUT', False)
    monkeypatch.setattr(runner, 'PRODUCT_BUILD_INFO_PATH', build_info)

    build_info.write_text(
        json.dumps({
            'schema_version': 1,
            'revision': 'c' * 40,
            'dirty': False,
            'source': 'override',
        }),
        encoding='utf-8',
    )
    assert runner._git_state() == {
        'commit': 'c' * 40,
        'dirty': False,
    }

    build_info.write_text(
        json.dumps({
            'schema_version': 1,
            'revision': 'c' * 40,
            'dirty': False,
            'source': 'invented',
        }),
        encoding='utf-8',
    )
    assert runner._git_state() == {'commit': None, 'dirty': None}
    build_info.write_text('{"revision":"wrong"}\n', encoding='utf-8')
    assert runner._git_state() == {'commit': None, 'dirty': None}
    build_info.write_text('not-json\n', encoding='utf-8')
    assert runner._git_state() == {'commit': None, 'dirty': None}
