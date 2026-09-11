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

"""Tests for the privacy-bounded session support attachment."""

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import zipfile

import jsonschema
import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'support_bundle.py'
COMPARE_TEST = REPO_ROOT / 'lidarslam' / 'test' / 'test_session_compare.py'
WIZARD_TEST = REPO_ROOT / 'lidarslam' / 'test' / 'test_sensor_setup_wizard.py'
WIZARD = REPO_ROOT / 'scripts' / 'sensor_setup_wizard.py'
RECEIPT = REPO_ROOT / 'scripts' / 'first_map_validation_receipt.py'
SCHEMA = REPO_ROOT / 'docs' / 'schemas' / 'support-bundle-v1.schema.json'
DIAGNOSIS_SCHEMA = REPO_ROOT / 'docs' / 'schemas' / 'diagnosis-v1.schema.json'
HANDOFF_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'first-map-handoff-v1.schema.json'
)


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _fixture(tmp_path: Path, name: str = 'private-site') -> Path:
    helpers = _load(COMPARE_TEST, f'support_fixture_{len(name)}')
    root = tmp_path / 'private operator output'
    root.mkdir(exist_ok=True)
    return helpers._write_fixture(
        root,
        name,
        created_at='2026-08-12T06:00:00Z',
    )


def _rewrite(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload), encoding='utf-8')


def _validation_fixture(tmp_path: Path) -> Path:
    bundle = _fixture(tmp_path, 'verified-first-map')
    helpers = _load(WIZARD_TEST, 'support_validation_helpers')
    wizard = _load(WIZARD, 'support_validation_wizard')
    receipt_module = _load(RECEIPT, 'support_validation_receipt')
    map_output = bundle / 'map'
    receipt_path = helpers._write_validation_receipt(map_output)
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    (map_output / 'first_map_validation_receipt.md').write_text(
        receipt_module.render_markdown(receipt),
        encoding='utf-8',
    )
    setup = json.loads(
        (bundle / 'sensor_setup.json').read_text(encoding='utf-8')
    )
    manifest = {
        'bundle_path': str(bundle),
        'input': {'bag_path': setup['input']['bag_path']},
        'profile': setup['profile'],
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
    _rewrite(bundle / 'session.json', session)
    return bundle


def test_report_is_schema_valid_and_redacts_paths_commands_and_secrets(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'support_bundle_privacy')
    bundle = _fixture(tmp_path)
    setup_path = bundle / 'sensor_setup.json'
    session_path = bundle / 'session.json'
    setup = json.loads(setup_path.read_text(encoding='utf-8'))
    session = json.loads(session_path.read_text(encoding='utf-8'))
    secret = 'secret-token-value-123'
    private_label = '/home/operator/customer-alpha'
    setup['profile']['label'] = private_label
    session['profile']['label'] = private_label
    setup['run']['argv'].extend(['--api-token', secret])
    setup['run']['command_shell'] += f' --api-token {secret}'
    _rewrite(setup_path, setup)
    _rewrite(session_path, session)

    report = module.build_support_report(str(bundle))
    rendered = json.dumps(report, sort_keys=True)
    schema = json.loads(SCHEMA.read_text(encoding='utf-8'))

    jsonschema.validate(report, schema)
    assert report['privacy'] == {
        'contains_map_geometry': False,
        'contains_raw_sensor_data': False,
        'contains_raw_logs': False,
        'contains_parameter_contents': False,
        'local_paths_redacted': True,
        'command_secrets_redacted': True,
        'review_before_sharing': True,
    }
    assert str(tmp_path) not in rendered
    assert str(bundle) not in rendered
    assert private_label not in rendered
    assert secret not in rendered
    assert '<bag>' in report['setup']['run_command']['redacted']
    assert '<map-output>' in report['setup']['run_command']['redacted']
    assert '<redacted-secret>' in report['setup']['run_command']['redacted']


def test_missing_stale_or_symlinked_setup_is_reported_without_inference(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'support_bundle_setup_state')
    stale = _fixture(tmp_path, 'stale')
    parameter = stale / 'params/graph.yaml'
    parameter.write_text('changed: true\n', encoding='utf-8')

    stale_report = module.build_support_report(str(stale))

    assert stale_report['setup']['status'] == 'invalid'
    assert stale_report['setup']['run_command'] is None
    assert stale_report['setup']['parameters'] == []

    missing = _fixture(tmp_path, 'missing')
    (missing / 'sensor_setup.json').unlink()
    missing_report = module.build_support_report(str(missing))
    assert missing_report['setup']['status'] == 'invalid'

    linked = _fixture(tmp_path, 'linked')
    setup_path = linked / 'sensor_setup.json'
    real_setup = linked / 'real-sensor-setup.json'
    setup_path.rename(real_setup)
    setup_path.symlink_to(real_setup.name)
    linked_report = module.build_support_report(str(linked))
    assert linked_report['setup']['status'] == 'invalid'
    setup_artifact = next(
        item for item in linked_report['artifacts']
        if item['name'] == 'setup_manifest'
    )
    assert setup_artifact['current_state'] == 'symlink'


def test_diagnosis_projection_keeps_states_and_counts_but_not_messages(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'support_bundle_diagnosis')
    bundle = _fixture(tmp_path)
    session_path = bundle / 'session.json'
    session = json.loads(session_path.read_text(encoding='utf-8'))
    diagnosis_path = bundle / 'map/autoware_map_diagnosis.json'
    private_hint = f'private path: {bundle}/customer-map.pcd'
    _rewrite(diagnosis_path, {
        'status': 'verify_failed',
        'verify': {'result': 'FAIL'},
        'projector_type': 'Local',
        'symptom_triage': {
            'symptom': 'map-spins-or-spirals',
            'code': 'map-spins-or-spirals',
            'basis': 'USER_REPORTED_NOT_AUTOMATICALLY_DIAGNOSED',
            'title': private_hint,
            'checks': [private_hint],
            'next_commands': [f'lidarslam-map inspect {bundle}'],
            'avoid': [private_hint],
        },
        'problem_hints': [private_hint, 'another hint'],
        'suggested_next_steps': ['inspect private data'],
    })
    session['artifacts']['diagnosis_json'] = str(diagnosis_path)
    _rewrite(session_path, session)

    report = module.build_support_report(str(bundle))
    rendered = json.dumps(report)

    assert report['diagnosis'] == {
        'evidence_status': 'regular_file',
        'status': 'verify_failed',
        'verify_result': 'FAIL',
        'projector_type': 'Local',
        'problem_hint_count': 2,
        'suggested_step_count': 1,
        'reported_symptom': {
            'code': 'map-spins-or-spirals',
            'basis': 'USER_REPORTED_NOT_AUTOMATICALLY_DIAGNOSED',
        },
    }
    assert private_hint not in rendered
    assert 'inspect private data' not in rendered
    issue_body = module.support_members(report)['issue-body.md']
    assert 'Reported visual symptom: `map-spins-or-spirals`' in issue_body
    assert 'not an automatically diagnosed root cause' in issue_body


@pytest.mark.parametrize('field,value', [
    ('code', 'unknown-map-problem'),
    ('code', 'map-stops-early'),
    ('basis', 'AUTOMATIC_ROOT_CAUSE'),
])
def test_diagnosis_projection_rejects_unbounded_symptom_claims(
    tmp_path: Path,
    field: str,
    value: str,
):
    module = _load(SCRIPT, f'support_bundle_bad_symptom_{field}')
    bundle = _fixture(tmp_path, field)
    session_path = bundle / 'session.json'
    session = json.loads(session_path.read_text(encoding='utf-8'))
    diagnosis_path = bundle / 'map/autoware_map_diagnosis.json'
    symptom = {
        'symptom': 'map-is-not-visible',
        'code': 'map-is-not-visible',
        'basis': 'USER_REPORTED_NOT_AUTOMATICALLY_DIAGNOSED',
        'private_note': f'customer site at {bundle}',
    }
    symptom[field] = value
    _rewrite(diagnosis_path, {
        'status': 'verify_failed',
        'symptom_triage': symptom,
    })
    session['artifacts']['diagnosis_json'] = str(diagnosis_path)
    _rewrite(session_path, session)

    report = module.build_support_report(str(bundle))
    rendered = json.dumps(report)

    assert report['diagnosis']['evidence_status'] == 'invalid_contract'
    assert report['diagnosis']['reported_symptom'] is None
    assert 'customer site' not in rendered


def test_reported_symptom_codes_match_the_diagnosis_and_support_schemas():
    module = _load(SCRIPT, 'support_bundle_symptom_contract')
    diagnosis_schema = json.loads(DIAGNOSIS_SCHEMA.read_text(encoding='utf-8'))
    support_schema = json.loads(SCHEMA.read_text(encoding='utf-8'))
    diagnosis_symptom = diagnosis_schema['properties']['symptom_triage']
    support_symptom = support_schema['definitions']['diagnosis'][
        'properties'
    ]['reported_symptom']['oneOf'][1]

    assert module.REPORTED_SYMPTOM_CODES == frozenset(
        diagnosis_symptom['properties']['code']['enum']
    )
    assert module.REPORTED_SYMPTOM_CODES == frozenset(
        support_symptom['properties']['code']['enum']
    )
    assert module.REPORTED_SYMPTOM_BASIS == (
        diagnosis_symptom['properties']['basis']['const']
    )
    assert module.REPORTED_SYMPTOM_BASIS == (
        support_symptom['properties']['basis']['const']
    )


def test_artifact_symlink_and_outside_path_are_never_read(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'support_bundle_artifact_boundary')
    bundle = _fixture(tmp_path)
    session_path = bundle / 'session.json'
    session = json.loads(session_path.read_text(encoding='utf-8'))
    protected = tmp_path / 'autoware_map_diagnosis.json'
    protected.write_text('TOP-SECRET-CONTENT', encoding='utf-8')
    linked = bundle / 'map/autoware_map_diagnosis.json'
    linked.symlink_to(protected)
    session['artifacts']['diagnosis_json'] = str(linked)
    session['artifacts']['run_manifest'] = str(
        tmp_path / 'run_manifest.json'
    )
    _rewrite(session_path, session)

    report = module.build_support_report(str(bundle))
    rendered = json.dumps(report)
    artifacts = {item['name']: item for item in report['artifacts']}

    assert artifacts['diagnosis_json']['current_state'] == 'symlink'
    assert artifacts['run_manifest']['current_state'] == (
        'outside_evidence_roots'
    )
    assert 'TOP-SECRET-CONTENT' not in rendered


def test_zip_has_only_fixed_sanitized_members_and_refuses_overwrite(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'support_bundle_zip')
    bundle = _fixture(tmp_path)
    report = module.build_support_report(str(bundle))
    output = tmp_path / 'support.zip'

    assert module.write_support_zip(output, report) == output
    with zipfile.ZipFile(output) as archive:
        assert archive.namelist() == [
            'README.txt',
            'issue-body.md',
            'support-report.json',
        ]
        members = {
            name: archive.read(name).decode('utf-8')
            for name in archive.namelist()
        }
    jsonschema.validate(
        json.loads(members['support-report.json']),
        json.loads(SCHEMA.read_text(encoding='utf-8')),
    )
    combined = '\n'.join(members.values())
    assert str(tmp_path) not in combined
    assert '"review_before_sharing": true' in combined
    assert 'Review every member before sharing' in members['README.txt']

    with pytest.raises(OSError, match='existing'):
        module.write_support_zip(output, report)
    protected = tmp_path / 'protected.zip'
    protected.write_text('keep', encoding='utf-8')
    linked = tmp_path / 'linked.zip'
    linked.symlink_to(protected)
    with pytest.raises(OSError, match='symlink'):
        module.write_support_zip(linked, report)
    assert protected.read_text(encoding='utf-8') == 'keep'


def test_zip_does_not_replace_a_target_created_during_finalization(
    monkeypatch,
    tmp_path: Path,
):
    module = _load(SCRIPT, 'support_bundle_output_race')
    bundle = _fixture(tmp_path)
    report = module.build_support_report(str(bundle))
    output = tmp_path / 'raced-support.zip'
    real_link = module.os.link

    def create_competing_target(source, destination):
        Path(destination).write_text('competitor', encoding='utf-8')
        return real_link(source, destination)

    monkeypatch.setattr(module.os, 'link', create_competing_target)

    with pytest.raises(OSError):
        module.write_support_zip(output, report)
    assert output.read_text(encoding='utf-8') == 'competitor'


def test_json_mode_is_read_only_and_zip_mode_prints_review_warning(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'support_bundle_modes')
    bundle = _fixture(tmp_path)
    monkeypatch.setattr(module, '_utc_now', lambda: '2026-08-12T06:30:00Z')

    assert module.main([str(bundle), '--json']) == 0
    json_output = capsys.readouterr()
    assert json.loads(json_output.out)['schema_version'] == 1
    assert not list(bundle.parent.glob('lidarslam-support-*.zip'))

    output = tmp_path / 'explicit-support.zip'
    assert module.main([
        str(bundle), '--output', str(output),
    ]) == 0
    terminal = capsys.readouterr()
    assert output.is_file()
    assert 'Review all three ZIP members' in terminal.out


def test_first_map_handoff_revalidates_pass_evidence_without_writing(
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'support_first_map_handoff')
    bundle = _validation_fixture(tmp_path)
    before = {
        path.relative_to(bundle): (
            path.stat().st_size,
            path.stat().st_mtime_ns,
        )
        for path in bundle.rglob('*')
        if path.is_file()
    }

    handoff = module.build_first_map_handoff(str(bundle))
    assert handoff['status'] == 'READY_FOR_REVIEW'
    assert handoff['receipt_status'] == 'PASS'
    assert handoff['form_fields'] == {
        'result': 'PASS — verified first map completed',
        'release_ref': 'a' * 40,
    }
    assert handoff['verification_summary'].splitlines()[:3] == [
        'manifest_status=succeeded',
        'diagnosis_status=success',
        'autoware_status=PASS',
    ]
    assert handoff['issue_url'].endswith(
        '?template=first-map-validation.yml'
    )
    assert handoff['privacy'] == {
        'contains_map_geometry': False,
        'contains_private_paths': True,
        'contains_exact_command': False,
        'review_before_sharing': True,
    }
    assert module.main([str(bundle), '--first-map']) == 0
    terminal = capsys.readouterr().out
    assert 'First-map report: READY FOR REVIEW' in terminal
    assert 'Copy into the issue form:' in terminal
    assert 'Result: PASS — verified first map completed' in terminal
    assert f'Release, commit, or image digest: {"a" * 40}' in terminal
    assert 'Verification summary:' in terminal
    assert 'Complete before submitting:' in terminal
    assert 'Public documentation path: <Docker First Map' in terminal
    expected_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    assert (
        f'Environment: Linux / x86_64 / ROS 2 {expected_distro}' in terminal
    )
    assert 'Redacted command shape: <executable and options' in terminal
    assert 'Findings: <what was unclear, slow, surprising' in terminal
    assert 'use REDACTED for credentials' in terminal
    assert 'host/user names' in terminal
    assert str(bundle / 'map/first_map_validation_receipt.json') in terminal
    assert 'never attach the map, bag, manifest, logs' in terminal
    assert len(terminal.splitlines()) <= 23
    assert '```text' not in terminal
    assert 'Readable receipt:' not in terminal
    after = {
        path.relative_to(bundle): (
            path.stat().st_size,
            path.stat().st_mtime_ns,
        )
        for path in bundle.rglob('*')
        if path.is_file()
    }
    assert after == before
    assert not list(bundle.parent.glob('lidarslam-support-*.zip'))


def test_first_map_json_is_schema_valid_and_read_only(
    tmp_path: Path,
    capsys,
):
    """Structured handoff output validates without changing the session."""
    module = _load(SCRIPT, 'support_first_map_handoff_json')
    bundle = _validation_fixture(tmp_path)
    before = {
        path.relative_to(bundle): (
            path.stat().st_size,
            path.stat().st_mtime_ns,
        )
        for path in bundle.rglob('*')
        if path.is_file()
    }

    assert module.main([str(bundle), '--first-map', '--json']) == 0
    payload = json.loads(capsys.readouterr().out)
    schema = json.loads(HANDOFF_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.validate(payload, schema)
    assert payload['status'] == 'READY_FOR_REVIEW'
    assert payload['receipt_status'] == 'PASS'
    assert payload['privacy']['contains_private_paths'] is True
    assert payload['operator_supplied_fields'] == [
        'documentation_path',
        'environment_details',
        'exact_command',
        'findings',
    ]
    assert set(payload['environment_hints']) == {
        'os_family',
        'architecture',
        'ros_distro',
    }
    after = {
        path.relative_to(bundle): (
            path.stat().st_size,
            path.stat().st_mtime_ns,
        )
        for path in bundle.rglob('*')
        if path.is_file()
    }
    assert after == before
    assert not list(bundle.parent.glob('lidarslam-support-*.zip'))


def test_first_map_handoff_accepts_fixed_demo_output_without_session_index(
    tmp_path: Path,
    capsys,
):
    """Docker/source fixed demos expose receipt evidence without session.json."""
    module = _load(SCRIPT, 'support_fixed_demo_handoff')
    bundle = _validation_fixture(tmp_path)
    (bundle / 'session.json').unlink()
    demo_output = bundle / 'map'

    handoff = module.build_first_map_handoff(str(demo_output))
    assert handoff['status'] == 'READY_FOR_REVIEW'
    assert handoff['receipt_status'] == 'PASS'
    assert handoff['form_fields']['release_ref'] == 'a' * 40

    assert module.main([str(demo_output), '--first-map', '--json']) == 0
    payload = json.loads(capsys.readouterr().out)
    schema = json.loads(HANDOFF_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.validate(payload, schema)
    assert payload['run'] == handoff['run']
    assert payload['verification_summary'] == handoff['verification_summary']


def test_report_mode_is_a_focused_read_only_alias(
    tmp_path: Path,
    monkeypatch,
    capsys,
):
    """The first-class report command hides ZIP-only support options."""
    module = _load(SCRIPT, 'support_first_map_report_mode')
    bundle = _validation_fixture(tmp_path)
    monkeypatch.setenv(module.SUPPORT_MODE_ENV, 'first-map-report')

    args = module.parse_args([str(bundle), '--json'])
    assert args.first_map is True
    assert args.output is None
    assert module.main([str(bundle)]) == 0
    output = capsys.readouterr().out
    assert 'First-map report: READY FOR REVIEW' in output
    assert not list(bundle.parent.glob('lidarslam-support-*.zip'))

    with pytest.raises(SystemExit) as rejected:
        module.parse_args([str(bundle), '--output', str(tmp_path / 'x.zip')])
    assert rejected.value.code == 2


def test_first_map_handoff_uses_product_version_when_commit_is_unavailable(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'support_first_map_version_fallback')
    bundle = _validation_fixture(tmp_path)
    receipt_path = bundle / 'map/first_map_validation_receipt.json'
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    receipt['run']['git_commit'] = None
    _rewrite(receipt_path, receipt)

    handoff = module.build_first_map_handoff(str(bundle))

    assert handoff['run']['git_commit'] == 'unknown'
    assert handoff['form_fields']['release_ref'] == (
        receipt['run']['product_version']
    )


def test_first_map_handoff_rejects_stale_symlinked_and_nonpass_evidence(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'support_first_map_rejections')
    stale = _validation_fixture(tmp_path)
    diagnosis = stale / 'map/autoware_map_diagnosis.json'
    diagnosis.write_text('{"status":"runtime_failed"}', encoding='utf-8')
    with pytest.raises(ValueError, match='no longer matches'):
        module.build_first_map_handoff(str(stale))

    linked_root = tmp_path / 'linked-case'
    linked_root.mkdir()
    linked = _validation_fixture(linked_root)
    receipt = linked / 'map/first_map_validation_receipt.json'
    real_receipt = linked / 'map/retained-receipt.json'
    receipt.rename(real_receipt)
    receipt.symlink_to(real_receipt.name)
    with pytest.raises(ValueError, match='not a regular'):
        module.build_first_map_handoff(str(linked))

    incomplete_root = tmp_path / 'incomplete-case'
    incomplete_root.mkdir()
    incomplete = _fixture(incomplete_root, 'no-pass-receipt')
    with pytest.raises(ValueError, match='verified session with PASS'):
        module.build_first_map_handoff(str(incomplete))


def test_first_map_mode_rejects_writing_but_allows_structured_json(
    tmp_path: Path,
    capsys,
):
    module = _load(SCRIPT, 'support_first_map_option_boundary')
    bundle = _validation_fixture(tmp_path)
    output = tmp_path / 'must-not-exist.zip'

    assert module.main([
        str(bundle), '--first-map', '--output', str(output),
    ]) == 2
    assert '[invalid-usage]' in capsys.readouterr().err
    assert not output.exists()
    assert module.main([str(bundle), '--first-map', '--json']) == 0
    assert json.loads(capsys.readouterr().out)['status'] == 'READY_FOR_REVIEW'


def test_rejects_bundle_session_symlinks_oversized_json_and_bad_output(
    tmp_path: Path,
):
    module = _load(SCRIPT, 'support_bundle_rejections')
    bundle = _fixture(tmp_path)
    linked_bundle = tmp_path / 'linked-bundle'
    linked_bundle.symlink_to(bundle, target_is_directory=True)
    with pytest.raises(ValueError, match='bundle may not be a symlink'):
        module.build_support_report(str(linked_bundle))

    real_session = bundle / 'real-session.json'
    session_path = bundle / 'session.json'
    session_path.rename(real_session)
    session_path.symlink_to(real_session.name)
    with pytest.raises(ValueError, match='symlinked'):
        module.build_support_report(str(bundle))
    session_path.unlink()
    real_session.rename(session_path)

    session_path.write_text(
        session_path.read_text(encoding='utf-8')
        + (' ' * (2 * 1024 * 1024)),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='oversized'):
        module.build_support_report(str(bundle))

    safe_bundle = _fixture(tmp_path, 'safe-output')
    report = module.build_support_report(str(safe_bundle))
    with pytest.raises(OSError, match='.zip suffix'):
        module.write_support_zip(tmp_path / 'support.txt', report)
