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
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
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

"""Regression tests for bounded-filesystem real-data evidence."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import jsonschema
import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_bounded_filesystem_exhaustion.py'
SCHEMA_PATH = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'bounded-filesystem-exhaustion-v1.schema.json'
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        'run_bounded_filesystem_exhaustion',
        SCRIPT_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_evaluate_state_requires_failed_manifest_and_real_enospc_signature():
    module = _load_module()
    state = {
        'product_exit_code': 1,
        'filesystem': {
            'capacity_bytes': 32 * 1024 * 1024,
            'available_bytes': 900_000,
            'used_bytes': 32 * 1024 * 1024 - 900_000,
        },
        'manifest': {
            'status': 'failed',
            'lifecycle': {
                'stage': 'complete',
                'runner_exit_code': 1,
            },
        },
        'diagnosis': {
            'status': 'runtime_failed',
            'problem_hints': [
                (
                    'The output filesystem ran out of writable space or quota. '
                    'Preserve the run evidence, free storage, and rerun.'
                ),
            ],
        },
        'storage_signatures': [
            {
                'path': 'slam.launch.log',
                'line': 12,
                'text': 'raw_fallocate(length=1140644) returned 28.',
            },
        ],
    }

    checks = module.evaluate_state(
        state,
        32 * 1024 * 1024,
        container_exit_code=1,
        timed_out=False,
        harness_commit='a' * 40,
        harness_dirty=False,
        image_revision='a' * 40,
    )

    assert len(checks) == 10
    assert all(item['passed'] for item in checks)

    mismatched_image_checks = module.evaluate_state(
        state,
        32 * 1024 * 1024,
        container_exit_code=1,
        timed_out=False,
        harness_commit='a' * 40,
        harness_dirty=False,
        image_revision='c' * 40,
    )
    assert not next(
        item for item in mismatched_image_checks
        if item['id'] == 'runtime_matches_harness_revision'
    )['passed']

    not_nearly_full = {
        **state,
        'filesystem': {
            'capacity_bytes': 32 * 1024 * 1024,
            'available_bytes': 4 * 1024 * 1024,
            'used_bytes': 28 * 1024 * 1024,
        },
    }
    capacity_checks = module.evaluate_state(
        not_nearly_full,
        32 * 1024 * 1024,
        container_exit_code=1,
        timed_out=False,
        harness_commit='a' * 40,
        harness_dirty=False,
        image_revision='a' * 40,
    )
    assert not next(
        item for item in capacity_checks
        if item['id'] == 'filesystem_nearly_exhausted'
    )['passed']

    false_success = {
        **state,
        'manifest': {
            'status': 'succeeded',
            'lifecycle': {'stage': 'complete', 'runner_exit_code': 0},
        },
        'diagnosis': {'status': 'success', 'problem_hints': []},
    }
    failed_checks = module.evaluate_state(
        false_success,
        32 * 1024 * 1024,
        container_exit_code=0,
        timed_out=False,
        harness_commit='a' * 40,
        harness_dirty=False,
        image_revision='a' * 40,
    )
    failed_ids = {
        item['id'] for item in failed_checks if not item['passed']
    }
    assert 'product_failed_closed' in failed_ids
    assert 'manifest_records_failed_terminal_state' in failed_ids
    assert 'diagnosis_identifies_storage_exhaustion' in failed_ids
    assert 'success_not_claimed' in failed_ids


def test_collect_state_copies_only_text_evidence(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = _load_module()
    output_root = tmp_path / 'bounded'
    run_dir = output_root / 'run'
    evidence_dir = tmp_path / 'evidence'
    run_dir.mkdir(parents=True)
    evidence_dir.mkdir()
    (run_dir / 'run_manifest.json').write_text(
        json.dumps({
            'status': 'failed',
            'lifecycle': {'stage': 'complete', 'runner_exit_code': 1},
        }),
        encoding='utf-8',
    )
    (run_dir / 'autoware_map_diagnosis.json').write_text(
        json.dumps({
            'status': 'runtime_failed',
            'problem_hints': [
                'The output filesystem ran out of writable space or quota.',
            ],
        }),
        encoding='utf-8',
    )
    (run_dir / 'slam.launch.log').write_text(
        'raw_fallocate(length=1000) returned 28.\n',
        encoding='utf-8',
    )
    (run_dir / 'map.pcd').write_bytes(b'geometry must not be copied')
    monkeypatch.setattr(
        module,
        '_bounded_filesystem_state',
        lambda _path: {
            'capacity_bytes': 32 * 1024 * 1024,
            'available_bytes': 512 * 1024,
            'used_bytes': 32 * 1024 * 1024 - 512 * 1024,
        },
    )

    state = module.collect_state(output_root, evidence_dir, 1)

    assert state['output_layout'] == 'final'
    assert state['manifest']['status'] == 'failed'
    assert state['diagnosis']['status'] == 'runtime_failed'
    assert state['storage_signatures'][0]['path'] == 'slam.launch.log'
    assert 'captured/slam.launch.log' in state['captured_files']
    assert not (evidence_dir / 'captured' / 'map.pcd').exists()
    assert (evidence_dir / 'bounded_state.json').is_file()


def test_report_schema_accepts_terminal_pass_shape():
    module = _load_module()
    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(schema)
    report = {
        'schema_version': 1,
        'schema_uri': module.SCHEMA_URI,
        'status': 'passed',
        'started_at': '2026-07-28T00:00:00Z',
        'finished_at': '2026-07-28T00:02:00Z',
        'duration_sec': 120.0,
        'hardware_label': 'test-machine',
        'image': {
            'reference': 'example.invalid/image@sha256:' + 'a' * 64,
            'image_id': 'sha256:' + 'b' * 64,
            'repo_digests': ['example.invalid/image@sha256:' + 'a' * 64],
            'source_revision': 'c' * 40,
        },
        'harness': {
            'commit': 'd' * 40,
            'dirty': False,
            'script_sha256': 'e' * 64,
            'runtime_payload': {
                'files': [{
                    'path': 'scripts/runtime.py',
                    'sha256': '2' * 64,
                }],
                'sha256': '3' * 64,
            },
        },
        'input': {
            'bag_name': 'public-bag',
            'metadata_sha256': 'f' * 64,
            'storage_identifier': 'sqlite3',
            'message_count': 10,
            'storage_files': [{
                'path': 'bag_0.db3',
                'size_bytes': 100,
                'sha256': '1' * 64,
            }],
        },
        'bounded_filesystem': {
            'type': 'docker_tmpfs',
            'limit_bytes': 32 * 1024 * 1024,
            'mount_path': '/bounded-output',
        },
        'execution': {
            'container_exit_code': 1,
            'timed_out': False,
            'timeout_secs': 600,
            'container_log_sha256': '2' * 64,
        },
        'observed': {
            'product_exit_code': 1,
            'output_layout': 'final',
            'filesystem': {
                'capacity_bytes': 32 * 1024 * 1024,
                'available_bytes': 900_000,
                'used_bytes': 32 * 1024 * 1024 - 900_000,
            },
            'manifest': {'status': 'failed'},
            'diagnosis': {'status': 'runtime_failed'},
            'storage_signatures': [{
                'path': 'slam.launch.log',
                'line': 12,
                'text': 'raw_fallocate(length=1140644) returned 28.',
            }],
            'captured_files': [
                'captured/run_manifest.json',
                'captured/autoware_map_diagnosis.json',
                'captured/slam.launch.log',
            ],
        },
        'checks': [
            {'id': check_id, 'passed': True, 'observed': 'ok'}
            for check_id in (
                'harness_revision_clean',
                'runtime_matches_harness_revision',
                'run_completed_within_timeout',
                'product_failed_closed',
                'tmpfs_capacity_enforced',
                'real_enospc_signature_observed',
                'filesystem_nearly_exhausted',
                'manifest_records_failed_terminal_state',
                'diagnosis_identifies_storage_exhaustion',
                'success_not_claimed',
            )
        ],
    }

    jsonschema.validate(report, schema)

    missing_observation = {**report, 'observed': {}}
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(missing_observation, schema)

    duplicate_check = {
        **report,
        'checks': report['checks'][:-1] + [report['checks'][0]],
    }
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(duplicate_check, schema)

    false_pass = {
        **report,
        'checks': [
            {**check, 'passed': False}
            if check['id'] == 'product_failed_closed'
            else check
            for check in report['checks']
        ],
    }
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(false_pass, schema)

    successful_product_exit = {
        **report,
        'execution': {**report['execution'], 'container_exit_code': 0},
    }
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(successful_product_exit, schema)
