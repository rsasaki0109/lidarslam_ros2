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

"""Tests for the local first-map verification package audit."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_first_map_verification_package.py'


def _load_module():
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        spec = importlib.util.spec_from_file_location(
            'first_map_verification_package_audit',
            SCRIPT,
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPT.parent))


def test_current_checkout_is_ready_and_side_effect_free():
    module = _load_module()
    before = (ROOT / 'README.md').read_bytes()
    report = module.audit_package(ROOT)
    after = (ROOT / 'README.md').read_bytes()

    assert report['status'] == 'READY'
    assert all(report['checks'].values())
    assert report['finding_codes'] == []
    assert report['network_requested'] is False
    assert report['writes_performed'] is False
    assert before == after


def test_audit_fails_closed_for_an_incomplete_root(tmp_path: Path):
    module = _load_module()
    report = module.audit_package(tmp_path)

    assert report['status'] == 'NOT_READY'
    assert report['checks']['required_files_present'] is False
    assert 'required-file-contract-mismatch' in report['finding_codes']
    assert report['network_requested'] is False
    assert report['writes_performed'] is False


def test_json_cli_emits_schema_valid_ready_report():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report['status'] == 'READY'
    assert report['scope'] == 'local-first-map-verification-package'
    assert report['checks']['release_bundle_contract'] is True


def test_json_cli_returns_not_ready_for_an_incomplete_root(tmp_path: Path):
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--root',
            str(tmp_path),
            '--json',
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1
    report = json.loads(result.stdout)
    assert report['status'] == 'NOT_READY'
    assert 'required-file-contract-mismatch' in report['finding_codes']


def test_release_bundle_contains_the_complete_verification_package():
    module = _load_module()
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        from build_release_bundle import release_bundle_paths
    finally:
        sys.path.remove(str(SCRIPT.parent))

    bundled_paths = set(release_bundle_paths(ROOT, 'v0.9.1'))
    assert set(module.REQUIRED_FILES).issubset(bundled_paths)


def test_release_checklist_runs_the_package_audit():
    releasing = (ROOT / 'RELEASING.md').read_text(encoding='utf-8')
    assert (
        'python3 scripts/check_first_map_verification_package.py --json'
        in releasing
    )
