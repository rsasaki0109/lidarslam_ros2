#!/usr/bin/env python3
"""Adversarial tests for unreviewed r3 image/toolchain capture."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import stat

from jsonschema import Draft7Validator, ValidationError
import pytest

import scripts.capture_competitive_execution_selection_r3_external as capture_module
from scripts.capture_competitive_execution_selection_r3_external import (
    capture_external,
    CAPTURE_MANIFEST_KIND,
    CaptureError,
    CommandResult,
    SYSTEMS,
    validate_capture_set,
)
from scripts.validate_competitive_execution_selection_r3 import (
    CANDIDATE_REL,
    ROOT,
)


CANDIDATE = ROOT / CANDIDATE_REL
STAMP = '2026-08-26T00:00:00Z'


def _fake_runner(*, image_status: int = 0, wrong_image: bool = False):
    commands: list[list[str]] = []

    def run(command: list[str]) -> CommandResult:
        commands.append(command)
        if command[0:3] == ['docker', 'image', 'inspect']:
            digest = command[-1]
            observed = 'sha256:' + '0' * 64 if wrong_image else digest
            payload = {
                'Id': observed,
                'RepoDigests': [],
                'RepoTags': [],
                'Created': '2026-08-26T00:00:00Z',
                'Architecture': 'amd64',
                'Os': 'linux',
                'Size': 1,
            }
            return CommandResult(image_status, json.dumps(payload).encode())
        return CommandResult(0, b'probe-version\n')

    return commands, run


def _rewrite_pair(path: Path, document: dict) -> None:
    payload = (json.dumps(document, indent=2, sort_keys=True) + '\n').encode()
    path.chmod(0o644)
    path.write_bytes(payload)
    path.chmod(0o444)
    sidecar = path.with_name(path.name + '.sha256')
    sidecar.chmod(0o644)
    sidecar.write_text(hashlib.sha256(payload).hexdigest() + '  ' + path.name + '\n')
    sidecar.chmod(0o444)


def test_partial_image_only_capture_is_immutable_and_not_reviewed(tmp_path: Path):
    commands, runner = _fake_runner()
    output = tmp_path / 'capture'
    result = capture_external(output_dir=output, runner=runner, observed_at=STAMP)
    assert result['manifest_kind'] == CAPTURE_MANIFEST_KIND
    assert result['status'] == 'PARTIAL_OBSERVATION'
    assert result['review_status'] == 'NOT_REVIEWED_EXTERNAL'
    assert result['benchmark_eligible'] is False
    assert validate_capture_set(output)['status'] == 'PARTIAL_OBSERVATION'
    assert all(stat.S_IMODE(path.stat().st_mode) == 0o444
               for path in output.iterdir())
    assert all(command[0:3] == ['docker', 'image', 'inspect']
               for command in commands)


def test_opt_in_toolchain_uses_exact_network_none_read_only_argv(tmp_path: Path):
    commands, runner = _fake_runner()
    output = tmp_path / 'capture'
    result = capture_external(
        output_dir=output, runner=runner, probe_toolchain=True,
        observed_at=STAMP)
    assert result['status'] == 'UNREVIEWED_COMPLETE_OBSERVATION'
    validation = validate_capture_set(output)
    assert validation['toolchain_status'] == 'UNREVIEWED_COMPLETE_OBSERVATION'
    image_commands = [item for item in commands if item[1:3] == ['image', 'inspect']]
    tool_commands = [item for item in commands if item[1] == 'run']
    assert len(image_commands) == len(SYSTEMS)
    expected_probe_count = sum(
        len(capture_module.TOOLCHAIN_PROBES) - len(
            capture_module.NOT_APPLICABLE_FIELDS_BY_SYSTEM[system])
        for system in SYSTEMS)
    assert len(tool_commands) == expected_probe_count
    toolchain = json.loads((output / 'toolchain_capture.json').read_text())
    assert toolchain['systems']['glim']['not_applicable_fields'] == ['pcl']
    assert toolchain['systems']['glim']['probes']['pcl'] == {
        'command': None, 'returncode': 0, 'timed_out': False,
        'value': 'not_applicable',
        'stdout_sha256': capture_module._sha256(b''),
        'reason': 'not_applicable',
    }
    assert toolchain['systems']['ours']['not_applicable_fields'] == []
    assert toolchain['systems']['fast_livo2']['not_applicable_fields'] == []
    for command in tool_commands:
        assert command[0:8] == [
            'docker', 'run', '--rm', '--pull=never', '--network', 'none',
            '--read-only', '--entrypoint']
        assert 'build' not in command
        assert 'sh' not in command
        assert command[0:2] == ['docker', 'run']


def test_missing_image_keeps_toolchain_pending_and_does_not_run_container(tmp_path: Path):
    commands, runner = _fake_runner(image_status=1)
    output = tmp_path / 'capture'
    result = capture_external(
        output_dir=output, runner=runner, probe_toolchain=True,
        observed_at=STAMP)
    assert result['status'] == 'PARTIAL_OBSERVATION'
    assert validate_capture_set(output)['toolchain_status'] == 'PARTIAL_OBSERVATION'
    assert all(command[1:3] == ['image', 'inspect'] for command in commands)


def test_digest_mismatch_is_pending_not_reviewed(tmp_path: Path):
    commands, runner = _fake_runner(wrong_image=True)
    output = tmp_path / 'capture'
    capture_external(output_dir=output, runner=runner, observed_at=STAMP)
    image = json.loads((output / 'image_inspect.json').read_text())
    assert all(item['status'] == 'PENDING' for item in image['systems'].values())
    assert all(item['reason'] == 'image_id_mismatch'
               for item in image['systems'].values())


def test_capture_rejects_output_collision_and_symlink_parent(tmp_path: Path):
    existing = tmp_path / 'existing'
    existing.mkdir()
    with pytest.raises(CaptureError, match='fresh and absent'):
        capture_external(output_dir=existing)
    real_parent = tmp_path / 'real'
    real_parent.mkdir()
    link_parent = tmp_path / 'link'
    link_parent.symlink_to(real_parent, target_is_directory=True)
    with pytest.raises(CaptureError, match='symlink'):
        capture_external(output_dir=link_parent / 'capture')


def test_capture_rejects_hardlink_and_extra_files(tmp_path: Path):
    commands, runner = _fake_runner()
    output = tmp_path / 'capture'
    capture_external(output_dir=output, runner=runner, observed_at=STAMP)
    alias = tmp_path / 'alias.json'
    os.link(output / 'image_inspect.json', alias)
    with pytest.raises(CaptureError, match='single-link'):
        validate_capture_set(output)
    alias.unlink()
    (output / 'extra.json').write_text('{}')
    (output / 'extra.json').chmod(0o444)
    with pytest.raises(CaptureError, match='missing or extra'):
        validate_capture_set(output)


def test_partial_pair_failure_removes_only_owned_files(tmp_path: Path, monkeypatch):
    commands, runner = _fake_runner()
    output = tmp_path / 'capture'
    original = capture_module._exclusive_write
    calls = 0

    def fail_toolchain_sidecar(path: Path, payload: bytes):
        nonlocal calls
        calls += 1
        if calls == 4:
            raise OSError('injected sidecar failure')
        return original(path, payload)

    monkeypatch.setattr(capture_module, '_exclusive_write', fail_toolchain_sidecar)
    with pytest.raises(OSError, match='injected sidecar failure'):
        capture_external(output_dir=output, runner=runner, observed_at=STAMP)
    assert output.is_dir()
    assert list(output.iterdir()) == []


def test_self_rehashed_candidate_binding_is_rejected(tmp_path: Path):
    commands, runner = _fake_runner()
    output = tmp_path / 'capture'
    capture_external(output_dir=output, runner=runner, observed_at=STAMP)
    image_path = output / 'image_inspect.json'
    image = json.loads(image_path.read_text())
    image['candidate_binding']['candidate_id'] = 'other-campaign'
    image['artifact_identity_sha256'] = capture_module._artifact_identity(image)
    _rewrite_pair(image_path, image)
    with pytest.raises(CaptureError, match='candidate binding'):
        validate_capture_set(output)


def test_capture_mutation_between_reads_is_rejected(tmp_path: Path, monkeypatch):
    commands, runner = _fake_runner()
    output = tmp_path / 'capture'
    capture_external(output_dir=output, runner=runner, observed_at=STAMP)
    original = capture_module._read_immutable
    image_path = output / 'image_inspect.json'
    mutated = False

    def mutate(path: Path, label: str, **kwargs):
        nonlocal mutated
        result = original(path, label, **kwargs)
        if path == image_path and not mutated:
            mutated = True
            image_path.chmod(0o644)
            image_path.write_bytes(result[0] + b'\nmutation')
            image_path.chmod(0o444)
        return result

    monkeypatch.setattr(capture_module, '_read_immutable', mutate)
    with pytest.raises(CaptureError, match='sidecar|changed'):
        validate_capture_set(output)


def test_toolchain_probe_failure_is_truthful_pending(tmp_path: Path):
    commands, image_runner = _fake_runner()

    def runner(command: list[str]) -> CommandResult:
        if command[1] == 'run':
            return CommandResult(1, b'', b'probe failed')
        return image_runner(command)

    output = tmp_path / 'capture'
    capture_external(
        output_dir=output, runner=runner, probe_toolchain=True,
        observed_at=STAMP)
    toolchain = json.loads((output / 'toolchain_capture.json').read_text())
    assert toolchain['status'] == 'PARTIAL_OBSERVATION'
    assert all(item['status'] == 'PENDING'
               for item in toolchain['systems'].values())


def test_not_applicable_policy_is_system_bound(tmp_path: Path):
    commands, runner = _fake_runner()
    output = tmp_path / 'capture'
    capture_external(
        output_dir=output, runner=runner, probe_toolchain=True,
        observed_at=STAMP)
    tool_path = output / 'toolchain_capture.json'
    toolchain = json.loads(tool_path.read_text())
    toolchain['systems']['ours']['not_applicable_fields'] = ['pcl']
    toolchain['systems']['ours']['probes']['pcl'] = {
        'command': None, 'returncode': 0, 'timed_out': False,
        'value': 'not_applicable',
        'stdout_sha256': capture_module._sha256(b''),
        'reason': 'not_applicable',
    }
    toolchain['artifact_identity_sha256'] = capture_module._artifact_identity(
        toolchain)
    _rewrite_pair(tool_path, toolchain)
    tool_payload = (json.dumps(toolchain, indent=2, sort_keys=True) + '\n').encode()
    manifest_path = output / 'capture_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['toolchain_capture']['sha256'] = hashlib.sha256(tool_payload).hexdigest()
    manifest['capture_identity_sha256'] = capture_module._identity(
        manifest, 'capture_identity_sha256')
    _rewrite_pair(manifest_path, manifest)
    with pytest.raises(CaptureError, match='not-applicable'):
        validate_capture_set(output)


def test_validate_reopens_candidate_at_capture_end(
        tmp_path: Path, monkeypatch):
    commands, runner = _fake_runner()
    output = tmp_path / 'capture'
    capture_external(output_dir=output, runner=runner, observed_at=STAMP)
    original = capture_module.validate_candidate
    calls = 0

    def stale_report(path):
        nonlocal calls
        report = original(path)
        calls += 1
        if calls == 1:
            report = dict(report)
            report['candidate_file_sha256'] = '0' * 64
        return report

    monkeypatch.setattr(capture_module, 'validate_candidate', stale_report)
    with pytest.raises(CaptureError, match='candidate changed'):
        validate_capture_set(output)


def test_capture_documents_match_strict_schema(tmp_path: Path):
    commands, runner = _fake_runner()
    output = tmp_path / 'capture'
    capture_external(
        output_dir=output, runner=runner, probe_toolchain=True,
        observed_at=STAMP)
    schema_path = ROOT / 'configs/slam_benchmark_profiles/' \
        'competitive_execution_selection_r3_external_capture_v1.schema.json'
    schema = json.loads(schema_path.read_text())
    Draft7Validator.check_schema(schema)
    validator = Draft7Validator(schema)
    for name in ('image_inspect.json', 'toolchain_capture.json',
                 'capture_manifest.json'):
        validator.validate(json.loads((output / name).read_text()))
    invalid_toolchain = json.loads(
        (output / 'toolchain_capture.json').read_text())
    invalid_toolchain['systems']['ours']['not_applicable_fields'] = ['pcl']
    with pytest.raises(ValidationError):
        validator.validate(invalid_toolchain)
