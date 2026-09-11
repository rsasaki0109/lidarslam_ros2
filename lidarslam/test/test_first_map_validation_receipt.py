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

"""Tests for privacy-bounded external first-map validation receipts."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPO_ROOT / 'scripts' / 'first_map_validation_receipt.py'
RUNNER_PATH = REPO_ROOT / 'scripts' / 'run_autoware_map_from_bag.py'
CLI_PATH = REPO_ROOT / 'scripts' / 'create_first_map_validation_receipt.py'
SCHEMA_PATH = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'first-map-validation-receipt-v1.schema.json'
)
PRIVATE_PATH = '/private/customer/site-42/input.bag'
PRIVATE_COMMAND = f'ros2 bag play {PRIVATE_PATH}'
ISSUE_TEMPLATE = (
    REPO_ROOT / '.github' / 'ISSUE_TEMPLATE' / 'first-map-validation.yml'
)


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_run(run_dir: Path, *, diagnosis_status: str = 'success') -> None:
    run_dir.mkdir()
    diagnosis_path = run_dir / 'autoware_map_diagnosis.json'
    verify_path = run_dir / 'verify_autoware_map.log'
    diagnosis_path.write_text(
        json.dumps({
            'schema_version': 1,
            'run_dir': '/private/customer/site-42/output',
            'status': diagnosis_status,
            'verify': {
                'result': 'PASS' if diagnosis_status == 'success' else 'FAIL',
            },
        }),
        encoding='utf-8',
    )
    verify_path.write_text(
        'RESULT: PASS -- map is Autoware-compatible\n'
        'PASS: 8 | WARN: 0 | FAIL: 0\n'
        f'internal source: {PRIVATE_PATH}\n',
        encoding='utf-8',
    )
    manifest = {
        'schema_version': 2,
        'run_id': '02fc84de-c5a2-40d6-9533-af72f89b664b',
        'status': 'succeeded',
        'lifecycle': {
            'stage': 'complete',
            'runner_exit_code': 0,
        },
        'input': {
            'bag_path': PRIVATE_PATH,
        },
        'software': {
            'product_version': '0.7.0',
            'git_commit': 'a' * 40,
        },
        'profile': {
            'id': 'rko_lio_graph_mid360_preset',
        },
        'execution': {
            'command_shell': PRIVATE_COMMAND,
        },
        'output': {
            'artifact_checksums': [
                {
                    'path': diagnosis_path.name,
                    'sha256': _sha256(diagnosis_path),
                },
                {
                    'path': verify_path.name,
                    'sha256': _sha256(verify_path),
                },
            ],
        },
    }
    (run_dir / 'run_manifest.json').write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def test_passing_receipt_is_schema_valid_and_privacy_bounded(tmp_path: Path):
    """A passing receipt should omit private source data."""
    module = _load_module(MODULE_PATH, 'first_map_validation_receipt_pass')
    run_dir = tmp_path / 'run'
    _write_run(run_dir)

    receipt = module.build_receipt(run_dir)

    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator(schema).validate(receipt)
    assert receipt['status'] == 'PASS'
    assert all(check['passed'] for check in receipt['checks'])
    assert receipt['verification'] == {
        'manifest_status': 'succeeded',
        'diagnosis_status': 'success',
        'autoware_status': 'PASS',
        'manifest_sha256': _sha256(run_dir / 'run_manifest.json'),
    }
    serialized = json.dumps(receipt)
    markdown = module.render_markdown(receipt)
    for private_value in (PRIVATE_PATH, PRIVATE_COMMAND, str(run_dir)):
        assert private_value not in serialized
        assert private_value not in markdown
    assert 'manifest_status=succeeded' in markdown
    assert 'autoware_status=PASS' in markdown
    assert module.VALIDATION_ISSUE_URL in markdown
    assert 'Both PASS and FAIL reports are useful.' in markdown


def test_submission_link_is_shared_by_receipt_and_runner():
    receipt_module = _load_module(
        MODULE_PATH,
        'first_map_validation_submission_link',
    )
    runner_module = _load_module(
        RUNNER_PATH,
        'run_autoware_map_submission_link',
    )

    assert ISSUE_TEMPLATE.is_file()
    assert (
        receipt_module.VALIDATION_ISSUE_URL
        == runner_module.VALIDATION_ISSUE_URL
    )
    assert receipt_module.VALIDATION_ISSUE_URL.endswith(
        '?template=first-map-validation.yml'
    )


def test_receipt_fails_when_frozen_diagnosis_identity_is_changed(
    tmp_path: Path,
):
    """Tampering with a frozen diagnosis must fail its binding check."""
    module = _load_module(MODULE_PATH, 'first_map_validation_receipt_tamper')
    run_dir = tmp_path / 'run'
    _write_run(run_dir)
    diagnosis_path = run_dir / 'autoware_map_diagnosis.json'
    diagnosis = json.loads(diagnosis_path.read_text(encoding='utf-8'))
    diagnosis['new_unbound_field'] = True
    diagnosis_path.write_text(json.dumps(diagnosis), encoding='utf-8')

    receipt = module.build_receipt(run_dir)

    assert receipt['status'] == 'FAIL'
    checks = {check['id']: check for check in receipt['checks']}
    assert checks['diagnosis_bound_to_manifest']['passed'] is False
    assert checks['diagnosis_bound_to_manifest']['observed'] == (
        'missing-or-mismatched'
    )


def test_verifier_result_requires_a_bounded_result_line(tmp_path: Path):
    """Accept the real verifier suffix without accepting status-like text."""
    module = _load_module(MODULE_PATH, 'first_map_validation_result_line')
    verify_path = tmp_path / 'verify_autoware_map.log'

    verify_path.write_text(
        'RESULT: PASS -- map is Autoware-compatible\n',
        encoding='utf-8',
    )
    assert module._verify_result(verify_path) == 'PASS'

    verify_path.write_text(
        'RESULT: PASSING\n',
        encoding='utf-8',
    )
    assert module._verify_result(verify_path) == 'unknown'


def test_terminal_failure_with_no_verify_log_still_has_shareable_receipt(
    tmp_path: Path,
):
    """Terminal failures should retain safe evidence even without a map."""
    module = _load_module(MODULE_PATH, 'first_map_validation_receipt_missing')
    run_dir = tmp_path / 'run'
    _write_run(run_dir, diagnosis_status='runtime_failed')
    (run_dir / 'verify_autoware_map.log').unlink()
    manifest_path = run_dir / 'run_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    manifest['status'] = 'failed'
    manifest['lifecycle']['runner_exit_code'] = 17
    manifest['output']['artifact_checksums'] = [
        identity
        for identity in manifest['output']['artifact_checksums']
        if identity['path'] != 'verify_autoware_map.log'
    ]
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )

    receipt = module.build_receipt(run_dir)

    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator(schema).validate(receipt)
    assert receipt['status'] == 'FAIL'
    assert receipt['verification']['autoware_status'] == 'missing'
    assert receipt['evidence']['verify_log'] == {
        'filename': 'verify_autoware_map.log',
        'available': False,
        'sha256': None,
    }
    serialized = json.dumps(receipt)
    assert PRIVATE_PATH not in serialized
    assert str(run_dir) not in serialized


def test_cli_writes_both_receipts_and_uses_stable_exit_codes(tmp_path: Path):
    """The CLI should write both formats and preserve exit meaning."""
    run_dir = tmp_path / 'run'
    _write_run(run_dir)

    passing = subprocess.run(
        ['python3', str(CLI_PATH), str(run_dir), '--write', '--json'],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert passing.returncode == 0, passing.stderr
    assert json.loads(passing.stdout)['status'] == 'PASS'
    assert (run_dir / 'first_map_validation_receipt.json').is_file()
    markdown_path = run_dir / 'first_map_validation_receipt.md'
    assert markdown_path.is_file()
    markdown = markdown_path.read_text(encoding='utf-8')
    assert 'Open the Independent First-map Validation issue form' in markdown
    assert '?template=first-map-validation.yml' in markdown
    assert 'attach that JSON file to the issue' in markdown
    assert 'Do not attach the manifest, map, logs, bag' in markdown

    missing = subprocess.run(
        ['python3', str(CLI_PATH), str(tmp_path / 'missing')],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert missing.returncode == 2
    assert 'run directory does not exist' in missing.stderr


def test_receipts_are_derived_and_excluded_from_manifest_artifact_hashes(
    tmp_path: Path,
):
    """Derived receipt files must not create a manifest hash cycle."""
    receipt_module = _load_module(
        MODULE_PATH,
        'first_map_validation_receipt_derived',
    )
    runner = _load_module(RUNNER_PATH, 'run_autoware_map_receipt_exclusion')
    run_dir = tmp_path / 'run'
    _write_run(run_dir)
    receipt_module.write_receipt(run_dir)

    identities = runner._artifact_checksums(run_dir)
    paths = {identity['path'] for identity in identities}

    assert 'autoware_map_diagnosis.json' in paths
    assert 'verify_autoware_map.log' in paths
    assert 'run_manifest.json' not in paths
    assert 'first_map_validation_receipt.json' not in paths
    assert 'first_map_validation_receipt.md' not in paths
