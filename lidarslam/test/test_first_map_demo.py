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

"""Tests for the stable one-command public first-map demo."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'first_map_demo.py'
RECEIPT_BUILDER = REPO_ROOT / 'scripts' / 'first_map_validation_receipt.py'
SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'first-map-demo-plan-v1.schema.json'
)


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _sparse_file(path: Path, size: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('wb') as stream:
        stream.truncate(size)


def _write_verified_output(path: Path) -> Path:
    path.mkdir(parents=True)
    diagnosis = path / 'autoware_map_diagnosis.json'
    verify_log = path / 'verify_autoware_map.log'
    manifest_path = path / 'run_manifest.json'
    diagnosis.write_text(json.dumps({'status': 'success'}), encoding='utf-8')
    verify_log.write_text('RESULT: PASS\n', encoding='utf-8')
    manifest = {
        'schema_version': 2,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/run-manifest-v2.schema.json'
        ),
        'run_id': '12345678-1234-4234-8234-123456789abc',
        'status': 'succeeded',
        'lifecycle': {
            'stage': 'complete',
            'resume_count': 0,
            'verification_enabled': True,
            'runner_exit_code': 0,
            'last_error': None,
        },
        'input': {
            'bag_path': '/fixture/public-bag',
            'metadata_path': '/fixture/public-bag/metadata.yaml',
            'metadata_size_bytes': 42,
            'metadata_sha256': 'a' * 64,
            'storage_identifier': 'sqlite3',
            'storage_files': [],
            'identity_algorithm': 'sha256',
        },
        'software': {
            'product_version': '0.9.0',
            'git_commit': 'b' * 40,
            'git_dirty': False,
            'package_versions': {'lidarslam': '0.9.0'},
            'ros_distro': 'jazzy',
        },
        'profile': {
            'id': 'rko_lio_graph_mid360_preset',
            'label': 'MID-360 public demo',
        },
        'execution': {
            'argv': ['lidarslam-map', 'run', '/fixture/public-bag'],
            'command_shell': 'lidarslam-map run <public-bag>',
            'started_at': '2026-08-12T00:00:00Z',
            'finished_at': '2026-08-12T00:01:00Z',
            'exit_code': 0,
        },
        'output': {
            'requested_dir': str(path),
            'working_dir': f'{path}.partial',
            'finalized': True,
            'diagnosis_status': 'success',
            'artifact_checksums': [
                {
                    'path': diagnosis.name,
                    'size_bytes': diagnosis.stat().st_size,
                    'sha256': _digest(diagnosis),
                },
                {
                    'path': verify_log.name,
                    'size_bytes': verify_log.stat().st_size,
                    'sha256': _digest(verify_log),
                },
            ],
        },
    }
    manifest_path.write_text(json.dumps(manifest), encoding='utf-8')
    builder = _load(RECEIPT_BUILDER, f'demo_receipt_{len(str(path))}')
    receipt = builder.build_receipt(path)
    (path / 'first_map_validation_receipt.json').write_text(
        json.dumps(receipt),
        encoding='utf-8',
    )
    return path


def _write_resumable_output(
    output: Path,
    *,
    stage: str = 'verified',
    workflow_exit_code: int = 0,
) -> Path:
    partial = output.with_name(f'{output.name}.partial')
    partial.mkdir(parents=True)
    manifest = {
        'schema_version': 2,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/run-manifest-v2.schema.json'
        ),
        'run_id': '12345678-1234-4234-8234-123456789abc',
        'status': (
            'succeeded' if workflow_exit_code == 0 else 'failed'
        ),
        'lifecycle': {
            'stage': stage,
            'resume_count': 0,
            'verification_enabled': True,
            'runner_exit_code': None,
            'last_error': None,
        },
        'input': {
            'bag_path': '/fixture/public-bag',
            'metadata_path': '/fixture/public-bag/metadata.yaml',
            'metadata_size_bytes': 42,
            'metadata_sha256': 'a' * 64,
            'storage_identifier': 'sqlite3',
            'storage_files': [],
            'identity_algorithm': 'sha256',
        },
        'software': {
            'product_version': '0.9.0',
            'git_commit': 'b' * 40,
            'git_dirty': False,
            'package_versions': {'lidarslam': '0.9.0'},
            'ros_distro': 'jazzy',
        },
        'profile': {
            'id': 'rko_lio_graph_mid360_preset',
            'label': 'MID-360 public demo',
        },
        'execution': {
            'argv': ['lidarslam-map', 'run', '/fixture/public-bag'],
            'command_shell': 'lidarslam-map run <public-bag>',
            'started_at': '2026-08-12T00:00:00Z',
            'finished_at': '2026-08-12T00:01:00Z',
            'exit_code': workflow_exit_code,
        },
        'output': {
            'requested_dir': str(output),
            'working_dir': str(partial),
            'finalized': False,
            'artifact_checksums': [],
        },
    }
    (partial / 'run_manifest.json').write_text(
        json.dumps(manifest),
        encoding='utf-8',
    )
    return partial


def _enough_space(monkeypatch, module) -> None:
    monkeypatch.setattr(
        module.shutil,
        'disk_usage',
        lambda path: SimpleNamespace(
            total=100 * 1024**3,
            used=10 * 1024**3,
            free=90 * 1024**3,
        ),
    )


def test_dry_run_json_is_schema_valid_copy_ready_and_read_only(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'first_map_demo_dry_run')
    _enough_space(monkeypatch, module)
    work = tmp_path / 'new demo workspace'
    monkeypatch.setenv('LIDARSLAM_CLI_COMMAND', 'lidarslam-map demo')

    assert module.main([
        str(work), '--viewer', 'none', '--dry-run', '--json',
    ]) == 0
    plan = json.loads(capsys.readouterr().out)

    jsonschema.validate(
        plan,
        json.loads(SCHEMA.read_text(encoding='utf-8')),
    )
    assert plan['status'] == 'ready'
    assert plan['ready'] is True
    assert plan['mode'] == 'fresh'
    assert plan['cache']['download_required'] is True
    assert plan['cache']['status'] == 'download_required'
    assert plan['output']['status'] == 'absent'
    assert plan['steps'] == [
        'download_and_verify_dataset',
        'build_and_verify_map',
    ]
    assert plan['command']['argv'][:3] == [
        'lidarslam-map',
        'demo',
        str(work.resolve()),
    ]
    assert plan['storage']['volumes'][0][
        'additional_bytes_required'
    ] == 0
    assert not work.exists()


def test_dry_run_output_is_exclusive_and_does_not_overwrite(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """The saved human or JSON plan refuses a second write."""
    module = _load(SCRIPT, 'first_map_demo_output')
    _enough_space(monkeypatch, module)
    work = tmp_path / 'work'
    human_output = tmp_path / 'observer-plan.md'
    output = tmp_path / 'observer-plan.json'
    human_args = [
        str(work),
        '--viewer',
        'none',
        '--dry-run',
        '--output',
        str(human_output),
    ]
    args = [
        str(work),
        '--viewer',
        'none',
        '--dry-run',
        '--json',
        '--output',
        str(output),
    ]

    assert module.main(human_args) == 0
    assert human_output.read_text(encoding='utf-8').startswith(
        'Public first-map demo plan\n'
    )
    assert module.main(args) == 0
    original = output.read_bytes()
    assert json.loads(original)['status'] == 'ready'
    assert module.main(args) == 2
    assert output.read_bytes() == original
    assert 'File exists' in capsys.readouterr().err
    assert not work.exists()


def test_dry_run_output_requires_dry_run(tmp_path: Path, capsys):
    """Live execution cannot redirect its progress into a plan file."""
    module = _load(SCRIPT, 'first_map_demo_output_guard')
    output = tmp_path / 'plan.txt'

    assert module.main(['--output', str(output)]) == 2
    assert '[demo-output-requires-dry-run]' in capsys.readouterr().err
    assert not output.exists()


def test_safe_terminal_partial_offers_copy_ready_demo_resume(
    monkeypatch,
    tmp_path: Path,
):
    module = _load(SCRIPT, 'first_map_demo_resume_offer')
    _enough_space(monkeypatch, module)
    work = tmp_path / 'work'
    output = work / 'output' / 'mid360_demo'
    partial = _write_resumable_output(output, stage='verified')

    plan = module.build_demo_plan(module.parse_args([
        str(work), '--viewer', 'none',
    ]))

    assert plan['status'] == 'not_ready'
    assert plan['output'] == {
        'status': 'resumable',
        'receipt_sha256': None,
        'lifecycle_stage': 'verified',
        'resume_available': True,
        'run_dir': str(partial),
    }
    assert plan['findings'][0]['code'] == 'resume-available'
    assert plan['findings'][0]['next_action'].endswith('--resume')


def test_resume_dry_run_is_schema_valid_and_never_restarts_mapping(
    monkeypatch,
    tmp_path: Path,
):
    module = _load(SCRIPT, 'first_map_demo_resume_plan')
    _enough_space(monkeypatch, module)
    work = tmp_path / 'work'
    output = work / 'output' / 'mid360_demo'
    _write_resumable_output(output, stage='diagnosed')

    plan = module.build_demo_plan(module.parse_args([
        str(work), '--viewer', 'none', '--resume',
    ]))

    jsonschema.validate(
        plan,
        json.loads(SCHEMA.read_text(encoding='utf-8')),
    )
    assert plan['status'] == 'resume_ready'
    assert plan['ready'] is True
    assert plan['mode'] == 'resume_postprocessing'
    assert plan['steps'][-1] == 'resume_and_verify_map'
    assert plan['command']['argv'][-1] == '--resume'
    assert 'build_and_verify_map' not in plan['steps']


def test_resume_accepts_failed_postprocessing_after_atomic_rename(
    monkeypatch,
    tmp_path: Path,
):
    module = _load(SCRIPT, 'first_map_demo_resume_final')
    _enough_space(monkeypatch, module)
    work = tmp_path / 'work'
    output = work / 'output' / 'mid360_demo'
    partial = _write_resumable_output(output, stage='checksumming')
    manifest_path = partial / 'run_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    manifest['lifecycle']['runner_exit_code'] = 70
    manifest['lifecycle']['last_error'] = 'injected receipt interruption'
    manifest_path.write_text(json.dumps(manifest), encoding='utf-8')
    partial.rename(output)

    plan = module.build_demo_plan(module.parse_args([
        str(work), '--viewer', 'none', '--resume',
    ]))

    assert plan['status'] == 'resume_ready'
    assert plan['output']['status'] == 'resumable'
    assert plan['output']['run_dir'] == str(output)


def test_resume_rejects_running_or_missing_state_without_delegation(
    monkeypatch,
    tmp_path: Path,
):
    module = _load(SCRIPT, 'first_map_demo_resume_unsafe')
    _enough_space(monkeypatch, module)
    work = tmp_path / 'work'
    output = work / 'output' / 'mid360_demo'
    partial = _write_resumable_output(
        output,
        stage='workflow_running',
    )
    manifest_path = partial / 'run_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    manifest['status'] = 'running'
    manifest['execution']['finished_at'] = None
    manifest['execution']['exit_code'] = None
    manifest_path.write_text(json.dumps(manifest), encoding='utf-8')

    def unexpected_run(*args, **kwargs):
        raise AssertionError('unsafe resume must not delegate')

    monkeypatch.setattr(module.subprocess, 'run', unexpected_run)

    assert module.main([
        str(work), '--viewer', 'none', '--resume',
    ]) == 2


def test_plan_detects_cached_bag_and_deduplicates_one_filesystem(
    monkeypatch,
    tmp_path: Path,
):
    module = _load(SCRIPT, 'first_map_demo_cache')
    _enough_space(monkeypatch, module)
    work = tmp_path / 'work'
    bag = (
        work / 'datasets' / 'mid360_public' / 'driving_slam_mid360'
        / 'extracted' / module.BAG_NAME / module.BAG_NAME
    )
    bag.mkdir(parents=True)
    _sparse_file(bag / 'metadata.yaml', module.METADATA_SIZE_BYTES)
    _sparse_file(bag / module.STORAGE_NAME, module.STORAGE_SIZE_BYTES)
    _sparse_file(
        work / 'datasets' / 'mid360_public' / 'driving_slam_mid360'
        / 'archives' / module.ARCHIVE_NAME,
        module.ARCHIVE_SIZE_BYTES,
    )

    plan = module.build_demo_plan(module.parse_args([
        str(work), '--viewer', 'none',
    ]))

    assert plan['cache'] == {
        'status': 'prepared_unverified',
        'bag_path': str(bag.resolve()),
        'download_required': False,
    }
    assert plan['steps'][0] == 'verify_prepared_dataset_cache'
    assert len(plan['storage']['volumes']) == 1


def test_plan_fails_closed_for_low_space_overlap_symlink_and_partial(
    monkeypatch,
    tmp_path: Path,
):
    module = _load(SCRIPT, 'first_map_demo_boundaries')
    monkeypatch.setattr(
        module.shutil,
        'disk_usage',
        lambda path: SimpleNamespace(total=1024, used=1023, free=1),
    )
    work = tmp_path / 'work'
    real_cache = tmp_path / 'real-cache'
    real_cache.mkdir()
    linked_cache = tmp_path / 'linked-cache'
    linked_cache.symlink_to(real_cache, target_is_directory=True)
    output = real_cache / 'map'
    partial = output.with_name(f'{output.name}.partial')
    partial.mkdir()

    plan = module.build_demo_plan(module.parse_args([
        str(work),
        '--data-dir',
        str(linked_cache),
        '--output-dir',
        str(output),
        '--viewer',
        'none',
    ]))
    codes = {item['code'] for item in plan['findings']}

    assert plan['status'] == 'not_ready'
    assert plan['ready'] is False
    assert {
        'path-symlink',
        'path-overlap',
        'partial-output',
        'insufficient-free-space',
    } <= codes
    volume = plan['storage']['volumes'][0]
    assert volume['additional_bytes_required'] == 8 * module.GIB - 1
    storage_finding = next(
        item for item in plan['findings']
        if item['code'] == 'insufficient-free-space'
    )
    assert '8.00 GiB more is needed' in storage_finding['message']
    assert plan['command']['shell'] in storage_finding['next_action']
    assert '<' not in storage_finding['next_action']


def test_low_storage_retry_preserves_shell_quoted_demo_command(
    monkeypatch,
    tmp_path: Path,
):
    module = _load(SCRIPT, 'first_map_demo_storage_retry')
    monkeypatch.setattr(
        module.shutil,
        'disk_usage',
        lambda path: SimpleNamespace(
            total=10 * module.GIB,
            used=3 * module.GIB + 1,
            free=7 * module.GIB - 1,
        ),
    )
    work = tmp_path / 'demo workspace'

    plan = module.build_demo_plan(module.parse_args([
        str(work), '--viewer', 'none',
    ]))

    storage_finding = next(
        item for item in plan['findings']
        if item['code'] == 'insufficient-free-space'
    )
    assert plan['storage']['volumes'][0][
        'additional_bytes_required'
    ] == module.GIB + 1
    assert '1.01 GiB more is needed' in storage_finding['message']
    assert storage_finding['next_action'].endswith(plan['command']['shell'])
    assert f"'{work.resolve()}'" in storage_finding['next_action']


def test_verified_output_is_rebuilt_and_tamper_detected(tmp_path: Path):
    module = _load(SCRIPT, 'first_map_demo_verified_evidence')
    output = _write_verified_output(tmp_path / 'verified')

    verified, receipt_sha256 = module._verified_output(output)

    assert verified is True
    assert receipt_sha256 == _digest(
        output / 'first_map_validation_receipt.json'
    )
    (output / 'verify_autoware_map.log').write_text(
        'RESULT: FAIL\n',
        encoding='utf-8',
    )
    assert module._verified_output(output) == (False, None)


def test_existing_verified_output_is_reused_without_mapping(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'first_map_demo_reuse')
    _enough_space(monkeypatch, module)
    work = tmp_path / 'work'
    output = _write_verified_output(work / 'output' / 'mid360_demo')

    def unexpected_run(*args, **kwargs):
        raise AssertionError('verified output must not be mapped again')

    monkeypatch.setattr(module.subprocess, 'run', unexpected_run)

    assert module.main([str(work), '--viewer', 'none']) == 0
    terminal = capsys.readouterr()
    assert 'Verified demo output reused.' in terminal.out
    assert f'Map output: {output}' in terminal.out


def test_demo_delegates_to_canonical_script_and_verifies_postcondition(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'first_map_demo_delegate')
    _enough_space(monkeypatch, module)
    work = tmp_path / 'work'
    output = work / 'output' / 'mid360_demo'
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        _write_verified_output(output)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, 'run', fake_run)

    assert module.main([str(work), '--viewer', 'none']) == 0
    terminal = capsys.readouterr()
    assert calls[0][0] == ['/bin/bash', str(module.CANONICAL_DEMO)]
    assert calls[0][1]['env']['DEMO_DATA_DIR'] == str(
        work / 'datasets' / 'mid360_public'
    )
    assert calls[0][1]['env']['DEMO_OUTPUT_DIR'] == str(output)
    assert calls[0][1]['env']['DEMO_RESUME'] == '0'
    assert 'First map verified.' in terminal.out
    assert 'Verifier: PASS' in terminal.out


def test_demo_resume_delegates_only_terminal_postprocessing(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'first_map_demo_resume_delegate')
    _enough_space(monkeypatch, module)
    work = tmp_path / 'work'
    output = work / 'output' / 'mid360_demo'
    partial = _write_resumable_output(output, stage='finalized')
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        shutil.rmtree(partial)
        _write_verified_output(output)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, 'run', fake_run)

    assert module.main([
        str(work), '--viewer', 'none', '--resume',
    ]) == 0
    terminal = capsys.readouterr()
    assert calls[0][0] == ['/bin/bash', str(module.CANONICAL_DEMO)]
    assert calls[0][1]['env']['DEMO_RESUME'] == '1'
    assert 'resume_and_verify_map' in terminal.out
    assert 'First map verified.' in terminal.out


def test_resume_failure_prints_durable_stage_and_same_command(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'first_map_demo_resume_failure')
    _enough_space(monkeypatch, module)
    work = tmp_path / 'work'
    output = work / 'output' / 'mid360_demo'
    _write_resumable_output(output, stage='checksumming')
    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda command, **kwargs: subprocess.CompletedProcess(command, 70),
    )

    assert module.main([
        str(work), '--viewer', 'none', '--resume',
    ]) == 70
    terminal = capsys.readouterr()
    assert 'Last durable stage: checksumming' in terminal.err
    assert 'only terminal post-processing will resume' in terminal.err
    assert terminal.err.count('--resume') == 1
    assert 'Retained evidence was not deleted' in terminal.err


def test_demo_propagates_mapping_failure_and_rejects_false_success(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'first_map_demo_failures')
    _enough_space(monkeypatch, module)
    work = tmp_path / 'work'

    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda command, **kwargs: subprocess.CompletedProcess(command, 17),
    )
    assert module.main([str(work), '--viewer', 'none']) == 17
    first_failure = capsys.readouterr()
    assert 'Last durable stage: no durable stage recorded' in (
        first_failure.err
    )
    assert 'Fresh retry:' in first_failure.err
    assert 'mid360_demo-retry-' not in first_failure.err

    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda command, **kwargs: subprocess.CompletedProcess(command, 0),
    )
    assert module.main([str(work), '--viewer', 'none']) == 1
    assert '[demo-evidence-invalid]' in capsys.readouterr().err


def test_browser_failure_does_not_replace_verified_map_success(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'first_map_demo_viewer')
    _enough_space(monkeypatch, module)
    work = tmp_path / 'work'
    _write_verified_output(work / 'output' / 'mid360_demo')
    monkeypatch.setattr(
        module.shutil,
        'which',
        lambda name: '/prefix/lidarslam-map',
    )
    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda command, **kwargs: subprocess.CompletedProcess(command, 9),
    )

    assert module.main([str(work)]) == 0
    terminal = capsys.readouterr()
    assert '[demo-viewer-failed]' in terminal.err
    assert 'Verified demo output reused.' in terminal.out


def test_json_requires_dry_run_and_writes_nothing(tmp_path: Path, capsys):
    module = _load(SCRIPT, 'first_map_demo_json_guard')
    work = tmp_path / 'work'

    assert module.main([str(work), '--json']) == 2
    assert '[demo-json-requires-dry-run]' in capsys.readouterr().err
    assert not work.exists()
