#!/usr/bin/env python3
"""Tests for the non-promoting r3 acceptance preflight checker."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from jsonschema import Draft7Validator

import pytest

import scripts.check_competitive_execution_r3_acceptance as checker


def _manifest(tmp_path: Path, mutate=None) -> Path:
    value = json.loads((checker.ROOT / checker.MANIFEST_REL).read_text())
    if mutate:
        mutate(value)
    value['manifest_identity_sha256'] = checker._sha({
        key: item for key, item in value.items()
        if key != 'manifest_identity_sha256'})
    path = tmp_path / 'manifest.json'
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')
    return path


def test_checked_in_manifest_is_not_ready():
    """The checked-in seven-slot manifest remains a non-promoting preflight."""
    result = checker.validate_manifest()
    assert result['status'] == 'NOT_READY'
    assert result['benchmark_eligible'] is False
    assert result['claim_eligible'] is False
    assert result['missing_closures'] == list(checker.REQUIRED_CLOSURES)


def test_self_rehashed_ready_edit_is_rejected(tmp_path):
    """A self-rehashed READY edit cannot promote the checked-in manifest."""
    path = _manifest(tmp_path, lambda value: value.update({'status': 'READY'}))
    with pytest.raises(checker.AcceptanceManifestError, match='promotion'):
        checker.validate_manifest(path)


def test_candidate_binding_drift_is_rejected(tmp_path):
    """A candidate identity change is rejected with a new manifest hash."""
    path = _manifest(
        tmp_path, lambda value: value['candidate_binding'].__setitem__(
            'file_sha256', '0' * 64))
    with pytest.raises(checker.AcceptanceManifestError, match='candidate'):
        checker.validate_manifest(path)


@pytest.mark.parametrize('change', ['extra', 'missing'])
def test_closure_coverage_must_be_exact(tmp_path, change):
    """Every required closure slot is exact."""
    def mutate(value):
        if change == 'extra':
            value['closures']['invented'] = {
                'status': 'PENDING', 'path': None, 'sha256': None}
        else:
            del value['closures'][checker.REQUIRED_CLOSURES[0]]
    path = _manifest(tmp_path, mutate)
    with pytest.raises(checker.AcceptanceManifestError, match='closure'):
        checker.validate_manifest(path)


def test_cli_help_and_not_ready_exit():
    """The CLI exposes help and a NOT_READY result."""
    command = [sys.executable, str(checker.SCRIPT if hasattr(checker, 'SCRIPT')
                                   else Path(checker.__file__)), '--help']
    assert subprocess.run(
        command, check=False, capture_output=True).returncode == 0
    command = [sys.executable, str(Path(checker.__file__))]
    result = subprocess.run(
        command, check=False, capture_output=True, text=True)
    assert result.returncode == 3
    assert json.loads(result.stdout)['status'] == 'NOT_READY'


def test_schema_and_checked_in_identity():
    """The manifest schema and checked-in canonical identity remain bound."""
    schema_path = checker.ROOT / checker.SCHEMA_REL
    schema = json.loads(schema_path.read_text())
    Draft7Validator.check_schema(schema)
    manifest = json.loads((checker.ROOT / checker.MANIFEST_REL).read_text())
    assert manifest['manifest_identity_sha256'] == checker._sha({
        key: item for key, item in manifest.items()
        if key != 'manifest_identity_sha256'})
