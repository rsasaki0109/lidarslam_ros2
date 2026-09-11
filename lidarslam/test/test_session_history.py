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

"""Tests for the bounded local map-session history."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'session_history.py'
WIZARD = REPO_ROOT / 'scripts' / 'sensor_setup_wizard.py'
CATALOG_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' /
    'map-session-catalog-v1.schema.json'
)


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_session(
    root: Path,
    name: str,
    *,
    status: str,
    created_at: str,
    label: str = 'Livox MID-360 · RKO-LIO graph',
) -> Path:
    wizard = _load(WIZARD, f'session_history_wizard_{name}')
    bundle = root / name
    bundle.mkdir(parents=True)
    verification = 'off' if status == 'unverified' else 'required'
    manifest = {
        'bundle_path': str(bundle),
        'input': {'bag_path': str(bundle / 'demo_bag')},
        'profile': {
            'id': 'rko_lio_graph_mid360_preset',
            'label': label,
        },
        'verification': {'mode': verification},
        'run': {
            'output_dir': str(bundle / 'map'),
            'argv': [
                'lidarslam-map',
                'run',
                str(bundle / 'demo_bag'),
                '--output-dir',
                str(bundle / 'map'),
                '--verification',
                verification,
            ],
        },
    }
    if status == 'running':
        payload = wizard._session_index_payload(
            type('Args', (), {'verification': verification})(),
            manifest,
            runner_exit_code=None,
            running_stage='workflow_running',
            active_run_dir=bundle / 'map',
        )
    else:
        payload = wizard._session_index_payload(
            type('Args', (), {'verification': verification})(),
            manifest,
            runner_exit_code=0,
        )
    payload['created_at'] = created_at
    (bundle / 'session.json').write_text(
        json.dumps(payload),
        encoding='utf-8',
    )
    (bundle / 'session.html').write_text(
        '<!doctype html><title>session fixture</title>\n',
        encoding='utf-8',
    )
    return bundle


def test_catalog_is_sorted_bounded_filtered_and_schema_valid(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'session_history_sorted')
    root = tmp_path / 'output'
    root.mkdir()
    newest = _write_session(
        root,
        'newest',
        status='verified',
        created_at='2026-08-12T03:00:00Z',
    )
    older = _write_session(
        root,
        'older',
        status='running',
        created_at='2026-08-12T02:00:00Z',
    )
    invalid = root / 'invalid'
    invalid.mkdir()
    (invalid / 'session.json').write_text('{broken', encoding='utf-8')
    linked_file = root / 'linked-file'
    linked_file.mkdir()
    (linked_file / 'session.json').symlink_to(newest / 'session.json')
    (root / 'linked-bundle').symlink_to(newest, target_is_directory=True)
    (older / 'session.html').unlink()
    (older / 'session.html').symlink_to(newest / 'session.html')
    oversized = root / 'oversized'
    oversized.mkdir()
    (oversized / 'session.json').write_text(
        'x' * (module.MAX_SESSION_BYTES + 1),
        encoding='utf-8',
    )
    nested = root / 'group' / 'nested'
    nested.mkdir(parents=True)
    (nested / 'session.json').write_text(
        (newest / 'session.json').read_text(encoding='utf-8'),
        encoding='utf-8',
    )

    payload = module.build_catalog(root, status='all', limit=1)

    jsonschema.validate(
        payload,
        json.loads(CATALOG_SCHEMA.read_text(encoding='utf-8')),
    )
    assert payload['source']['scan_depth'] == 1
    assert payload['source']['follows_symlinks'] is False
    assert payload['summary'] == {
        'candidates': 5,
        'valid': 2,
        'skipped_invalid': 3,
        'displayed': 1,
    }
    assert [item['status'] for item in payload['sessions']] == ['verified']
    assert payload['sessions'][0]['bundle_path'] == str(newest.resolve())

    running = module.build_catalog(root, status='running', limit=20)
    assert running['summary']['valid'] == 2
    assert running['summary']['displayed'] == 1
    assert running['sessions'][0]['status'] == 'running'
    assert running['sessions'][0]['page_path'] is None


def test_catalog_never_displays_more_than_two_hundred_sessions(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'session_history_maximum')
    root = tmp_path / 'output'
    root.mkdir()
    fixture = _write_session(
        root,
        'fixture',
        status='verified',
        created_at='2026-08-12T03:00:00Z',
    )
    rendered = (fixture / 'session.json').read_text(encoding='utf-8')
    for index in range(204):
        bundle = root / f'session-{index:03d}'
        bundle.mkdir()
        (bundle / 'session.json').write_text(rendered, encoding='utf-8')

    payload = module.build_catalog(root, status='all', limit=200)

    assert payload['summary']['candidates'] == 205
    assert payload['summary']['valid'] == 205
    assert payload['summary']['displayed'] == 200
    assert len(payload['sessions']) == 200


def test_catalog_html_escapes_session_text_and_has_no_network_dependency(
    monkeypatch,
    tmp_path: Path,
):
    module = _load(SCRIPT, 'session_history_escape')
    monkeypatch.setenv('LIDARSLAM_CLI_COMMAND', 'lidarslam-map sessions')
    root = tmp_path / 'output'
    root.mkdir()
    hostile = _write_session(
        root,
        'hostile',
        status='verified',
        created_at='2026-08-12T03:00:00Z',
        label='<img src=x onerror=alert(1)>',
    )
    hostile.rename(root / '<svg onload=alert(2)>')
    payload = module.build_catalog(root, status='all', limit=20)

    rendered = module.render_catalog_html(payload)

    assert '<img src=x onerror=alert(1)>' not in rendered
    assert '&lt;img src=x onerror=alert(1)&gt;' in rendered
    assert '<svg onload=alert(2)>' not in rendered
    assert '&lt;svg onload=alert(2)&gt;' in rendered
    assert '<script src=' not in rendered
    assert '<link ' not in rendered
    assert 'file://' in rendered
    assert 'Open session' in rendered
    assert 'Select for comparison' in rendered
    assert 'data-prefix="lidarslam-map compare"' in rendered
    assert 'Copy compare command' in rendered
    assert 'lidarslam-map support' in rendered
    assert 'Copy support command' in rendered
    assert 'class="copy-support button"' in rendered
    assert 'button.dataset.command' in rendered
    assert 'navigator.clipboard.writeText' in rendered


def test_verified_pass_card_has_copy_ready_share_action(
    monkeypatch,
    tmp_path: Path,
):
    module = _load(SCRIPT, 'session_history_share_action')
    monkeypatch.setenv('LIDARSLAM_CLI_COMMAND', 'lidarslam-map sessions')
    entry = {
        'bundle_path': str(tmp_path / 'verified-session'),
        'session_name': 'verified-session',
        'session_path': str(tmp_path / 'verified-session/session.json'),
        'page_path': None,
        'created_at': '2026-08-12T03:00:00Z',
        'status': 'verified',
        'profile': {'label': 'Livox MID-360 · RKO-LIO graph'},
        'verification': {'mode': 'required', 'result': 'PASS'},
        'quality': {'overall': 'pass', 'headline': 'All evidence passed'},
        'summary': {'title': 'Verified map', 'message': 'Ready'},
        'bag_path': str(tmp_path / 'bag'),
        'map_output': str(tmp_path / 'map'),
        'recommended_action': None,
    }

    rendered = module._render_session_card(entry)

    assert 'Prepare a first-map report' in rendered
    assert 'lidarslam-map report' in rendered
    assert 'Copy report command' in rendered
    assert 'class="copy-report button"' in rendered

    nonpass = dict(entry)
    nonpass['quality'] = {
        'overall': 'action_required',
        'headline': 'Needs attention',
    }
    nonpass_rendered = module._render_session_card(nonpass)
    assert 'Prepare a first-map report' not in nonpass_rendered
    assert 'Copy report command' not in nonpass_rendered


def test_terminal_history_prints_share_or_recovery_next_action(
    monkeypatch,
    tmp_path: Path,
):
    module = _load(SCRIPT, 'session_history_terminal_actions')
    monkeypatch.setenv('LIDARSLAM_CLI_COMMAND', 'lidarslam-map sessions')
    payload = {
        'summary': {
            'displayed': 2,
            'valid': 2,
            'skipped_invalid': 0,
        },
        'sessions': [
            {
                'bundle_path': str(tmp_path / 'verified'),
                'status': 'verified',
                'quality': {'overall': 'pass'},
                'created_at': '2026-08-12T03:00:00Z',
                'profile': {'label': 'verified profile'},
                'summary': {
                    'title': 'Map ready',
                    'message': 'Review the verified result.',
                },
                'page_path': None,
                'recommended_action': None,
            },
            {
                'bundle_path': str(tmp_path / 'failed'),
                'status': 'action_required',
                'quality': {'overall': 'action_required'},
                'created_at': '2026-08-12T04:00:00Z',
                'profile': {'label': 'failed profile'},
                'summary': {
                    'title': 'Mapping needs attention.',
                    'message': 'The retained run stopped before verification.',
                },
                'page_path': None,
                'recommended_action': {
                    'command': 'lidarslam-map support /tmp/failed',
                },
            },
        ],
    }

    rendered = module._render_terminal(payload)

    assert (
        'Report: lidarslam-map report '
        f'{tmp_path / "verified"}'
    ) in rendered
    assert (
        'Details: Mapping needs attention. — '
        'The retained run stopped before verification.'
    ) in rendered
    assert 'Next: lidarslam-map support /tmp/failed' in rendered


def test_terminal_history_compacts_retained_summary_text(
    tmp_path: Path,
):
    """Stored newlines must not break the copy-ready terminal layout."""
    module = _load(SCRIPT, 'session_history_terminal_details')
    entry = {
        'status': 'action_required',
        'quality': {'overall': 'action_required'},
        'summary': {
            'title': 'Needs\nattention',
            'message': 'Retry\r\nwith the retained setup.',
        },
    }

    assert module._terminal_summary(entry) == (
        'Needs attention — Retry with the retained setup.'
    )


def test_json_mode_is_read_only_and_browser_mode_opens_catalog(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'session_history_modes')
    root = tmp_path / 'output'
    root.mkdir()
    _write_session(
        root,
        'verified',
        status='verified',
        created_at='2026-08-12T03:00:00Z',
    )

    assert module.main([str(root), '--json']) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload['summary']['displayed'] == 1
    assert not (root / 'sessions.html').exists()

    opened: list[Path] = []
    monkeypatch.setattr(
        module,
        '_open_catalog',
        lambda path: opened.append(path) or True,
    )
    assert module.main([str(root), '--viewer', 'browser']) == 0
    output = capsys.readouterr().out
    assert opened == [root / 'sessions.html']
    assert 'Session history opened in the browser.' in output
    assert (root / 'sessions.html').is_file()


def test_catalog_refuses_to_replace_a_symlink(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'session_history_report_symlink')
    root = tmp_path / 'output'
    root.mkdir()
    target = tmp_path / 'protected.html'
    target.write_text('keep me', encoding='utf-8')
    (root / 'sessions.html').symlink_to(target)
    payload = module.build_catalog(root, status='all', limit=20)

    try:
        module.write_catalog_html(root, payload)
    except OSError as exc:
        assert 'refusing to replace symlink' in str(exc)
    else:
        raise AssertionError('catalog writer followed a symlink')
    assert target.read_text(encoding='utf-8') == 'keep me'


def test_catalog_writer_does_not_follow_predictable_temp_symlink(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'session_history_temp_symlink')
    root = tmp_path / 'output'
    root.mkdir()
    protected = tmp_path / 'protected.html'
    protected.write_text('keep me', encoding='utf-8')
    legacy_temp = root / f'.sessions.html.{module.os.getpid()}.tmp'
    legacy_temp.symlink_to(protected)
    payload = module.build_catalog(root, status='all', limit=20)

    assert module.write_catalog_html(root, payload) == root / 'sessions.html'
    assert protected.read_text(encoding='utf-8') == 'keep me'
    assert legacy_temp.is_symlink()


def test_missing_roots_have_safe_default_and_explicit_behavior(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'session_history_missing')
    monkeypatch.setattr(module, 'WORK_ROOT', tmp_path)

    assert module.main(['--viewer', 'none']) == 0
    output = capsys.readouterr().out
    assert 'No matching sessions.' in output
    assert not (tmp_path / 'output').exists()

    missing = tmp_path / 'explicitly-missing'
    assert module.main([str(missing), '--viewer', 'none']) == 2
    error = capsys.readouterr().err
    assert '[session-history-unavailable]' in error
    assert str(missing) in error


def test_main_warns_without_replacing_success_when_browser_open_fails(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'session_history_open_failure')
    root = tmp_path / 'output'
    root.mkdir()
    _write_session(
        root,
        'verified',
        status='verified',
        created_at='2026-08-12T03:00:00Z',
    )

    def fail(_path: Path) -> bool:
        raise RuntimeError('injected browser failure')

    monkeypatch.setattr(module, '_open_catalog', fail)

    assert module.main([str(root)]) == 0
    error = capsys.readouterr().err
    assert '[session-history-open-failed]' in error
    assert 'injected browser failure' in error
