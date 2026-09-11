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

"""Tests for evidence-backed local map-session comparison."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import jsonschema
import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'session_compare.py'
WIZARD = REPO_ROOT / 'scripts' / 'sensor_setup_wizard.py'
COMPARISON_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' /
    'map-session-comparison-v1.schema.json'
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


def _write_fixture(
    root: Path,
    name: str,
    *,
    created_at: str,
    topic: str = '/livox/lidar',
    parameter_text: str = 'mapping:\n  resolution: 0.2\n',
    metadata_sha: str = '1' * 64,
    label: str = 'Livox MID-360 · RKO-LIO graph',
) -> Path:
    wizard = _load(WIZARD, f'session_compare_wizard_{len(name)}')
    bundle = root / name
    bundle.mkdir(parents=True)
    bag = bundle / 'demo_bag'
    bag.mkdir()
    map_output = bundle / 'map'
    map_output.mkdir()
    (map_output / 'pointcloud_map').mkdir()
    params = bundle / 'params'
    params.mkdir()
    parameter = params / 'graph.yaml'
    parameter.write_text(parameter_text, encoding='utf-8')
    profile = {
        'id': 'rko_lio_graph_mid360_preset',
        'label': label,
    }
    setup = {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/sensor-setup-v1.schema.json'
        ),
        'status': 'ready',
        'created_at': created_at,
        'bundle_path': str(bundle),
        'input': {
            'bag_path': str(bag),
            'metadata_path': str(bag / 'metadata.yaml'),
            'metadata_size_bytes': 100,
            'metadata_sha256': metadata_sha,
            'storage_identifier': 'sqlite3',
            'storage_files': [],
            'identity_algorithm': 'sha256',
        },
        'profile': profile,
        'topics': {
            'lidar': topic,
            'lidar_type': 'sensor_msgs/msg/PointCloud2',
            'imu': '/livox/imu',
            'gnss': None,
            'navigation': None,
            'navigation_quality': None,
        },
        'frames': {
            'base': {'id': 'base_link', 'source': 'cli_or_default'},
            'lidar': {'id': 'livox_frame', 'source': 'bag_header'},
            'imu': {'id': 'livox_imu', 'source': 'bag_header'},
        },
        'pointcloud': {
            'inspection_status': 'compatible',
            'timestamp_field': 'offset_time',
            'fields': [],
        },
        'timestamp_order': {'status': 'monotonic'},
        'calibration': {
            'required': True,
            'source': 'accepted_profile_extrinsics',
            'lidar_to_base_quat_xyzw_xyz': [0, 0, 0, 1, 0, 0, 0],
            'imu_to_base_quat_xyzw_xyz': [0, 0, 0, 1, 0, 0, 0],
        },
        'parameters': [
            {
                'role': 'graph_backend',
                'source_path': '/tracked/lidarslam.yaml',
                'bundle_path': 'params/graph.yaml',
                'size_bytes': parameter.stat().st_size,
                'sha256': _digest(parameter),
            }
        ],
        'run': {
            'output_dir': str(map_output),
            'argv': [
                'lidarslam-map',
                'run',
                str(bag),
                '--output-dir',
                str(map_output),
            ],
            'command_shell': (
                f'lidarslam-map run {bag} --output-dir {map_output}'
            ),
        },
    }
    (bundle / 'sensor_setup.json').write_text(
        json.dumps(setup),
        encoding='utf-8',
    )
    manifest = {
        'bundle_path': str(bundle),
        'input': {'bag_path': str(bag)},
        'profile': profile,
        'verification': {'mode': 'required'},
        'run': {
            'output_dir': str(map_output),
            'argv': setup['run']['argv'],
        },
    }
    session = wizard._session_index_payload(
        type('Args', (), {'verification': 'required'})(),
        manifest,
        runner_exit_code=0,
    )
    session['created_at'] = created_at
    (bundle / 'session.json').write_text(
        json.dumps(session),
        encoding='utf-8',
    )
    (bundle / 'session.html').write_text(
        '<!doctype html><title>session fixture</title>\n',
        encoding='utf-8',
    )
    return bundle


def _row(payload: dict, row_id: str) -> dict:
    return next(
        item for item in payload['comparisons'] if item['id'] == row_id
    )


def test_comparison_reports_same_different_and_unavailable_without_winner(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'session_compare_rows')
    root = tmp_path / 'output'
    root.mkdir()
    left = _write_fixture(
        root,
        'left',
        created_at='2026-08-12T01:00:00Z',
    )
    right = _write_fixture(
        root,
        'right',
        created_at='2026-08-12T02:00:00Z',
        topic='/livox/points_changed',
        parameter_text='mapping:\n  resolution: 0.4\n',
    )

    payload = module.build_comparison(str(left), str(right))

    schema = json.loads(COMPARISON_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.validate(payload, schema)
    duplicate_id = json.loads(json.dumps(payload))
    duplicate_id['comparisons'][-1]['id'] = 'session_status'
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(duplicate_id, schema)
    assert payload['policy'] == {
        'numeric_score': False,
        'winner_selected': False,
        'missing_evidence': 'unavailable_not_inferred',
    }
    assert payload['summary']['total'] == 14
    assert sum(
        payload['summary'][key]
        for key in ('same', 'different', 'unavailable')
    ) == 14
    assert _row(payload, 'session_status')['result'] == 'same'
    assert _row(payload, 'profile')['result'] == 'same'
    assert _row(payload, 'input_identity')['result'] == 'same'
    assert _row(payload, 'topics')['result'] == 'different'
    assert _row(payload, 'parameters')['result'] == 'different'


def test_missing_or_stale_setup_is_unavailable_instead_of_inferred(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'session_compare_stale')
    root = tmp_path / 'output'
    root.mkdir()
    left = _write_fixture(
        root,
        'left',
        created_at='2026-08-12T01:00:00Z',
    )
    right = _write_fixture(
        root,
        'right',
        created_at='2026-08-12T02:00:00Z',
    )
    (right / 'params/graph.yaml').write_text(
        'mapping:\n  resolution: 99\n',
        encoding='utf-8',
    )

    stale = module.build_comparison(str(left), str(right))
    assert stale['right']['setup_source']['status'] == 'invalid'
    for row_id in (
        'profile',
        'input_identity',
        'topics',
        'frames',
        'calibration',
        'parameters',
    ):
        assert _row(stale, row_id)['result'] == 'unavailable'

    (right / 'params/graph.yaml').write_text(
        'mapping:\n  resolution: 0.2\n',
        encoding='utf-8',
    )
    parameter = right / 'params/graph.yaml'
    real_parameter = right / 'params/graph-real.yaml'
    parameter.rename(real_parameter)
    parameter.symlink_to(real_parameter.name)
    linked = module.build_comparison(str(left), str(right))
    assert linked['right']['setup_source']['status'] == 'invalid'
    assert _row(linked, 'parameters')['result'] == 'unavailable'
    parameter.unlink()
    real_parameter.rename(parameter)

    (right / 'sensor_setup.json').unlink()
    missing = module.build_comparison(str(left), str(right))
    assert missing['right']['setup_source']['status'] == 'invalid'
    assert _row(missing, 'topics')['result'] == 'unavailable'


def test_comparison_rejects_same_session_bundle_and_session_symlink(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'session_compare_identity')
    root = tmp_path / 'output'
    root.mkdir()
    left = _write_fixture(
        root,
        'left',
        created_at='2026-08-12T01:00:00Z',
    )
    right = _write_fixture(
        root,
        'right',
        created_at='2026-08-12T02:00:00Z',
    )

    with pytest.raises(ValueError, match='same-session'):
        module.build_comparison(str(left), str(left))

    (right / 'session.json').unlink()
    (right / 'session.json').symlink_to(left / 'session.json')
    with pytest.raises(ValueError, match='symlinked'):
        module.build_comparison(str(left), str(right))

    linked_bundle = root / 'linked-bundle'
    linked_bundle.symlink_to(left, target_is_directory=True)
    with pytest.raises(ValueError, match='bundle may not be a symlink'):
        module.build_comparison(str(linked_bundle), str(left))


def test_html_escapes_operator_text_and_links_only_regular_pages(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'session_compare_escape')
    root = tmp_path / 'output'
    root.mkdir()
    left = _write_fixture(
        root,
        '<svg onload=alert(1)>',
        created_at='2026-08-12T01:00:00Z',
        label='<img src=x onerror=alert(2)>',
    )
    right = _write_fixture(
        root,
        'right',
        created_at='2026-08-12T02:00:00Z',
    )
    protected = tmp_path / 'protected.html'
    protected.write_text('private', encoding='utf-8')
    (right / 'session.html').unlink()
    (right / 'session.html').symlink_to(protected)

    payload = module.build_comparison(str(left), str(right))
    rendered = module.render_comparison_html(payload)

    assert '<svg onload=alert(1)>' not in rendered
    assert '&lt;svg onload=alert(1)&gt;' in rendered
    assert '<img src=x onerror=alert(2)>' not in rendered
    assert '&lt;img src=x onerror=alert(2)&gt;' in rendered
    assert '<script src=' not in rendered
    assert '<link ' not in rendered
    assert rendered.count('Open session') == 1
    assert 'Page unavailable' in rendered


def test_writer_refuses_symlinks_and_unrecognized_existing_files(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'session_compare_writer')
    root = tmp_path / 'output'
    root.mkdir()
    left = _write_fixture(
        root,
        'left',
        created_at='2026-08-12T01:00:00Z',
    )
    right = _write_fixture(
        root,
        'right',
        created_at='2026-08-12T02:00:00Z',
    )
    payload = module.build_comparison(str(left), str(right))
    protected = tmp_path / 'protected.html'
    protected.write_text('keep me', encoding='utf-8')
    linked = tmp_path / 'linked.html'
    linked.symlink_to(protected)

    with pytest.raises(OSError, match='symlink'):
        module.write_comparison_html(linked, payload)
    with pytest.raises(OSError, match='non-comparison'):
        module.write_comparison_html(protected, payload)
    assert protected.read_text(encoding='utf-8') == 'keep me'

    generated = tmp_path / 'comparison.html'
    assert module.write_comparison_html(generated, payload) == generated
    assert module.write_comparison_html(generated, payload) == generated

    protected_temp = tmp_path / 'protected-temp.html'
    protected_temp.write_text('keep me too', encoding='utf-8')
    legacy_temp = tmp_path / f'.fresh.html.{module.os.getpid()}.tmp'
    legacy_temp.symlink_to(protected_temp)
    fresh = tmp_path / 'fresh.html'
    assert module.write_comparison_html(fresh, payload) == fresh
    assert protected_temp.read_text(encoding='utf-8') == 'keep me too'
    assert legacy_temp.is_symlink()


def test_json_mode_is_read_only_and_browser_failure_does_not_fail_comparison(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'session_compare_modes')
    root = tmp_path / 'output'
    root.mkdir()
    left = _write_fixture(
        root,
        'left',
        created_at='2026-08-12T01:00:00Z',
    )
    right = _write_fixture(
        root,
        'right',
        created_at='2026-08-12T02:00:00Z',
    )
    output = tmp_path / 'report.html'

    assert module.main([
        str(left), str(right), '--output', str(output), '--json'
    ]) == 0
    json.loads(capsys.readouterr().out)
    assert not output.exists()

    def fail(_path: Path) -> bool:
        raise RuntimeError('injected browser failure')

    monkeypatch.setattr(module, '_open_comparison', fail)
    assert module.main([
        str(left), str(right), '--output', str(output)
    ]) == 0
    captured = capsys.readouterr()
    assert output.is_file()
    assert '[comparison-open-failed]' in captured.err
    assert 'injected browser failure' in captured.err


def test_main_returns_stable_usage_error_for_same_session(
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'session_compare_same_main')
    root = tmp_path / 'output'
    root.mkdir()
    session = _write_fixture(
        root,
        'session',
        created_at='2026-08-12T01:00:00Z',
    )

    assert module.main([str(session), str(session), '--json']) == 2
    assert '[same-session]' in capsys.readouterr().err
