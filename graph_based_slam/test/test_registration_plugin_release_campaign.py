#!/usr/bin/env python3
"""Synthetic tests for the fixed four-leg release campaign orchestrator."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

from jsonschema import Draft202012Validator
import pytest


REPO = Path(__file__).resolve().parents[2]
PROFILE = REPO / 'configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json'
SCRIPT = REPO / 'scripts/run_registration_plugin_release_campaign.py'


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def campaign_module():
    return _load(SCRIPT, 'registration_plugin_release_campaign_test')


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _fixture(campaign_module, monkeypatch, tmp_path):
    profile, release, profile_path = campaign_module._load_profile(PROFILE)
    snapshot = {
        'profile': {'path': str(profile_path), 'sha256': '1' * 64},
        'source': {
            'status': 'PASS', 'hash_kind': 'relative_path_content_sha256_v1',
            'files': [{'path': 'synthetic.py', 'expected_sha256': '6' * 64,
                       'status': 'PASS', 'actual_sha256': '6' * 64,
                       'size_bytes': 1}],
            'manifest_sha256': '2' * 64, 'mismatches': [],
        },
        'git': {'revision': '3' * 40, 'head_tree': '4' * 40,
                'dirty': True, 'dirty_tree_sha256': '5' * 64,
                'dirty_components': {'status': ' M synthetic.py',
                                     'diff_sha256': '7' * 64, 'untracked': []}},
    }
    monkeypatch.setattr(campaign_module, '_source_snapshot',
                        lambda repo, value, path: snapshot)

    def image_probe(reference):
        image = next(item['image'] for item in release['distros']
                     if item['image']['reference'] == reference)
        return {'Id': image['digest'], 'RepoDigests': [reference],
                'Os': 'linux', 'Architecture': 'amd64'}

    def container_probe(_name):
        return None

    return profile, release, profile_path, snapshot, image_probe, container_probe


def _legacy_seal(module, path: Path, value: dict):
    result = module.audit.seal_receipt(path, value)
    return Path(result[0]), Path(result[1])


def _fake_leg(module, snapshot, campaign, calls, *, fail_index=None,
              tamper_sidecar=False):
    def execute(row, child, command):
        calls.append((row['index'], tuple(command)))
        child.mkdir()
        if row['index'] == fail_index:
            raise module.CampaignError('FAKE_LEG_FAILURE', 'synthetic leg failure')
        inner_path = child / 'registration_plugin_release.receipt.json'
        inner = {
            'schema_version': 1, 'contract_version': 'registration-plugin-release-matrix-v1',
            'status': 'PASS', 'distro': row['distro'],
            'dependency_leg': row['dependency_leg'], 'campaign_set': campaign,
            'profile': snapshot['profile'],
            'repository': {'source_manifest': snapshot['source']},
        }
        _legacy_seal(module, inner_path, inner)
        inner_data = inner_path.read_bytes()
        host_path = child / 'registration_plugin_host.receipt.json'
        host = {
            'schema': 'registration-plugin-host-release-leg-v1', 'schema_version': 1,
            'status': 'PASS', 'failure': None, 'distro': row['distro'],
            'dependency_leg': row['dependency_leg'], 'campaign_set': campaign,
            'profile': snapshot['profile'],
            'source': {'manifest_sha256': snapshot['source']['manifest_sha256']},
            'container': {'name': row['container_name'], 'start_count': 1},
            'cleanup': {'post_remove_absent': True},
            'safety': {'bag_opened': False, 'ground_truth_content_opened': False,
                       'scorer_invoked': False, 'map_saved': False,
                       'formal_replay_started': False},
            'runner_receipt_binding': {
                'status': 'PASS', 'path_relative': 'registration_plugin_release.receipt.json',
                'sha256': _sha(inner_data),
            },
        }
        _legacy_seal(module, host_path, host)
        if tamper_sidecar:
            sidecar = Path(str(host_path) + '.sha256')
            sidecar.chmod(0o644)
            sidecar.write_text('0' * 64 + '  ' + host_path.name + '\n', encoding='ascii')
            sidecar.chmod(0o444)
        return {'status': 'PASS', 'closure': {'path': str(host_path)}}

    return execute


def _fake_summary(module, snapshot, campaign, calls):
    def summarize(profile_path, inner_paths, output):
        calls.append((str(profile_path), tuple(str(path) for path in inner_paths)))
        value = {
            'schema_version': 1,
            'contract_version': 'registration-plugin-release-matrix-summary-v1',
            'status': 'PASS', 'profile': snapshot['profile'],
            'source_manifest_sha256': snapshot['source']['manifest_sha256'],
            'campaign_set': campaign,
            'rows': [{'distro': path.parent.name.split('-')[3],
                      'dependency_leg': path.parent.name.split('-')[4]}
                     for path in inner_paths],
        }
        _legacy_seal(module, output, value)
        return {'status': 'PASS'}

    return summarize


def test_preflight_only_is_non_runtime_and_does_not_create_campaign_root(
        campaign_module, monkeypatch, tmp_path):
    _profile, _release, _profile_path, _snapshot, _image, _container = _fixture(
        campaign_module, monkeypatch, tmp_path)
    campaign_root = tmp_path / 'registration-plugin-release-campaign-test'
    output = tmp_path / 'preflight.json'
    report, seal = campaign_module.run_preflight(
        REPO, PROFILE, campaign_root, output)
    assert report['status'] == 'PREFLIGHT_NOT_RUNTIME_VALIDATED'
    assert report['benchmark_eligible'] is False
    assert not campaign_root.exists()
    assert Path(seal['path']).is_file()
    assert output.stat().st_mode & 0o777 == 0o444


def test_source_manifest_accepts_sealed_optional_v2_snapshot(campaign_module):
    source = {
        'status': 'PASS', 'hash_kind': 'relative_path_content_sha256_v1',
        'files': [{'path': 'synthetic.py', 'expected_sha256': '6' * 64,
                   'status': 'PASS', 'actual_sha256': '6' * 64,
                   'size_bytes': 1}],
        'manifest_sha256': '2' * 64, 'mismatches': [],
        'v2_snapshot': {'status': 'NOT_CONFIGURED', 'checked': False,
                        'promotion': 'NOT_AUTHORIZED'},
    }
    campaign_module._validate_source_manifest(source)
    source['v2_snapshot']['checked'] = True
    with pytest.raises(campaign_module.CampaignError) as error:
        campaign_module._validate_source_manifest(source)
    assert error.value.kind == 'CAMPAIGN_IDENTITY_INVALID'


def test_preflight_output_outside_safe_roots_is_rejected(
        campaign_module, monkeypatch, tmp_path):
    _profile, _release, _profile_path, _snapshot, _image, _container = _fixture(
        campaign_module, monkeypatch, tmp_path)
    with pytest.raises(campaign_module.CampaignError, match='below /tmp'):
        campaign_module.run_preflight(
            REPO, PROFILE, tmp_path / 'campaign',
            Path('/var/tmp/registration-plugin-preflight.json'))


def test_campaign_runs_exact_rows_once_and_summarizes_only_after_four_pass(
        campaign_module, monkeypatch, tmp_path):
    _profile, release, _profile_path, snapshot, image, container = _fixture(
        campaign_module, monkeypatch, tmp_path)
    leg_calls = []
    summary_calls = []
    root = tmp_path / 'registration-plugin-release-campaign-test-pass'
    report, seal = campaign_module.run_campaign(
        REPO, PROFILE, root, image_probe=image, container_probe=container,
        leg_executor=_fake_leg(campaign_module, snapshot, release['campaign_set'], leg_calls),
        summary_executor=_fake_summary(
            campaign_module, snapshot, release['campaign_set'], summary_calls),
        require_storage=False, now=lambda: 1_800_000_000.0)
    assert report['status'] == 'PASS'
    assert report['summary']['status'] == 'PASS'
    assert [item[0] for item in leg_calls] == [0, 1, 2, 3]
    assert len(summary_calls) == 1
    assert report['partial_rows'] == []
    assert report['partial_row_count'] == 0
    assert Path(seal['path']).stat().st_mode & 0o777 == 0o444
    value, _ = campaign_module._read_sealed(
        Path(seal['path']), campaign_module.SCHEMA, 'campaign receipt')
    assert value['canonical_sha256'] == campaign_module._canonical_hash(value)


def test_failed_leg_stops_later_rows_and_does_not_invoke_summary(
        campaign_module, monkeypatch, tmp_path):
    _profile, release, _profile_path, snapshot, image, container = _fixture(
        campaign_module, monkeypatch, tmp_path)
    leg_calls = []
    summary_calls = []
    root = tmp_path / 'registration-plugin-release-campaign-test-fail'
    report, _seal = campaign_module.run_campaign(
        REPO, PROFILE, root,
        image_probe=image, container_probe=container,
        leg_executor=_fake_leg(campaign_module, snapshot, release['campaign_set'],
                               leg_calls, fail_index=1),
        summary_executor=_fake_summary(
            campaign_module, snapshot, release['campaign_set'], summary_calls),
        require_storage=False, now=lambda: 1_800_000_000.0)
    assert report['status'] == 'PARTIAL_FAILURE'
    assert report['summary']['status'] == 'NOT_RUN'
    assert [item[0] for item in leg_calls] == [0, 1]
    assert [item['status'] for item in report['rows']] == [
        'PASS', 'FAIL_CLOSED', 'NOT_STARTED', 'NOT_STARTED']
    assert report['partial_row_count'] == 1
    assert len(report['partial_rows']) == 1
    partial = report['partial_rows'][0]
    assert partial['index'] == 1
    assert partial['status'] == 'FAIL_CLOSED'
    partial_path = root / partial['path_relative']
    assert partial_path.is_file()
    assert partial_path.stat().st_mode & 0o777 == 0o444
    partial_value, partial_desc = campaign_module._read_sealed(
        partial_path, campaign_module.PARTIAL_RECEIPT_SCHEMA,
        'test partial receipt')
    assert partial_value['canonical_sha256'] == partial['canonical_sha256']
    assert partial_desc['file_sha256'] == partial['file_sha256']
    assert partial_desc['sidecar_sha256'] == partial['sidecar_sha256']
    assert not summary_calls


def test_partial_row_schema_and_adversarial_projection_are_fail_closed(
        campaign_module, monkeypatch, tmp_path):
    _profile, release, _profile_path, snapshot, image, container = _fixture(
        campaign_module, monkeypatch, tmp_path)
    root = tmp_path / 'registration-plugin-release-campaign-partial-contract'
    report, _seal = campaign_module.run_campaign(
        REPO, PROFILE, root, image_probe=image, container_probe=container,
        leg_executor=_fake_leg(campaign_module, snapshot, release['campaign_set'], [],
                               fail_index=1),
        require_storage=False, now=lambda: 1_800_000_000.0)
    partial = report['partial_rows'][0]
    partial_path = root / partial['path_relative']
    partial_value, _ = campaign_module._read_sealed(
        partial_path, campaign_module.PARTIAL_RECEIPT_SCHEMA,
        'schema test partial receipt')
    schema_path = REPO / 'configs/slam_benchmark_profiles/' \
        'registration_plugin_release_child_partial_v1.schema.json'
    schema = json.loads(schema_path.read_text(encoding='utf-8'))
    Draft202012Validator.check_schema(schema)
    Draft202012Validator(schema).validate(partial_value)

    def assert_rejected(mutate):
        candidate = json.loads(json.dumps(report))
        mutate(candidate)
        with pytest.raises(campaign_module.CampaignError):
            campaign_module._validate_campaign_report(candidate)

    assert_rejected(lambda value: (
        value.__setitem__('partial_rows', []),
        value.__setitem__('partial_row_count', 0)))
    assert_rejected(lambda value: (
        value['partial_rows'].append(json.loads(json.dumps(partial))),
        value.__setitem__('partial_row_count', 2)))
    assert_rejected(lambda value: value['partial_rows'][0].__setitem__(
        'path_relative', 'wrong/partial.receipt.json'))
    assert_rejected(lambda value: value['partial_rows'][0].__setitem__(
        'status', 'PASS'))
    assert_rejected(lambda value: value['partial_rows'][0].__setitem__(
        'file_sha256', 'f' * 64))
    assert_rejected(lambda value: value['partial_rows'][0].__setitem__(
        'failure', {'kind': 'FAKE', 'phase': 'leg', 'row': 0,
                    'message': 'wrong row'}))

    rows_empty = json.loads(json.dumps(report))
    rows_empty['partial_rows'] = []
    rows_empty['partial_row_count'] = 0
    with pytest.raises(campaign_module.CampaignError,
                       match='unreferenced|missing'):
        campaign_module._validate_partial_rows(
            rows_empty, [], rows_empty['campaign_set'], rows_empty['profile'],
            rows_empty['source_manifest'], rows_empty['git'])


def test_source_drift_aborts_before_next_row(
        campaign_module, monkeypatch, tmp_path):
    profile, release, profile_path, snapshot, image, container = _fixture(
        campaign_module, monkeypatch, tmp_path)
    calls = {'count': 0}

    def drifting_snapshot(repo, value, path):
        calls['count'] += 1
        if calls['count'] >= 4:
            changed = json.loads(json.dumps(snapshot))
            changed['source']['manifest_sha256'] = 'f' * 64
            return changed
        return snapshot

    monkeypatch.setattr(campaign_module, '_source_snapshot', drifting_snapshot)
    leg_calls = []
    report, _seal = campaign_module.run_campaign(
        REPO, PROFILE, tmp_path / 'registration-plugin-release-campaign-test-drift',
        image_probe=image, container_probe=container,
        leg_executor=_fake_leg(campaign_module, snapshot, release['campaign_set'], leg_calls),
        require_storage=False, now=lambda: 1_800_000_000.0)
    assert report['status'] == 'PARTIAL_FAILURE'
    assert report['failure']['kind'] == 'SAFETY_IDENTITY_DRIFT'
    assert [item[0] for item in leg_calls] == [0]
    assert report['summary']['status'] == 'NOT_RUN'


def test_failure_after_root_reservation_is_sealed(
        campaign_module, monkeypatch, tmp_path):
    _profile, release, _profile_path, snapshot, image, container = _fixture(
        campaign_module, monkeypatch, tmp_path)
    calls = {'count': 0}

    def drift_after_preflight(repo, value, path):
        calls['count'] += 1
        if calls['count'] == 1:
            return snapshot
        changed = json.loads(json.dumps(snapshot))
        changed['source']['manifest_sha256'] = 'e' * 64
        return changed

    monkeypatch.setattr(campaign_module, '_source_snapshot', drift_after_preflight)
    root = tmp_path / 'registration-plugin-release-campaign-root-failure'
    report, seal = campaign_module.run_campaign(
        REPO, PROFILE, root, image_probe=image, container_probe=container,
        require_storage=False, now=lambda: 1_800_000_000.0)
    assert report['status'] == 'FAIL_CLOSED'
    assert report['failure']['phase'] == 'before_first_row'
    assert root.is_dir()
    assert Path(seal['path']).is_file()
    assert [row['status'] for row in report['rows']] == [
        'FAIL_CLOSED', 'NOT_STARTED', 'NOT_STARTED', 'NOT_STARTED']
    assert report['safety']['docker']['state'] == 'NOT_STARTED'


def test_attempted_docker_failure_is_not_reported_as_not_started(
        campaign_module, monkeypatch, tmp_path):
    _profile, release, _profile_path, snapshot, image, container = _fixture(
        campaign_module, monkeypatch, tmp_path)

    def attempted_leg(row, child, command):
        child.mkdir()
        host_path = child / 'registration_plugin_host.receipt.json'
        host = {
            'schema': 'registration-plugin-host-release-leg-v1', 'schema_version': 1,
            'status': 'FAIL_CLOSED', 'failure': {'kind': 'RUNNER_FAILED'},
            'distro': row['distro'], 'dependency_leg': row['dependency_leg'],
            'campaign_set': release['campaign_set'], 'profile': snapshot['profile'],
            'source': {'manifest_sha256': snapshot['source']['manifest_sha256']},
            'container': {'name': row['container_name'], 'start_count': 1},
            'cleanup': {'post_remove_absent': True},
            'safety': {'bag_opened': False, 'ground_truth_content_opened': False,
                       'scorer_invoked': False, 'map_saved': False,
                       'formal_replay_started': False},
            'runner_receipt_binding': {'status': 'NOT_REACHED'},
        }
        _legacy_seal(campaign_module, host_path, host)
        return {'status': 'FAIL_CLOSED', 'closure': {'path': str(host_path)}}

    report, _seal = campaign_module.run_campaign(
        REPO, PROFILE, tmp_path / 'registration-plugin-release-campaign-attempt',
        image_probe=image, container_probe=container, leg_executor=attempted_leg,
        require_storage=False, now=lambda: 1_800_000_000.0)
    assert report['status'] == 'PARTIAL_FAILURE'
    assert report['safety']['docker']['state'] == 'CONFIRMED'
    assert report['safety']['build_test']['state'] == 'ATTEMPTED'


def test_campaign_invariants_reject_forged_shape_and_status(
        campaign_module, monkeypatch, tmp_path):
    _profile, release, _profile_path, snapshot, image, container = _fixture(
        campaign_module, monkeypatch, tmp_path)
    report, _seal = campaign_module.run_campaign(
        REPO, PROFILE, tmp_path / 'registration-plugin-release-campaign-invariants',
        image_probe=image, container_probe=container,
        leg_executor=_fake_leg(campaign_module, snapshot, release['campaign_set'], []),
        summary_executor=_fake_summary(campaign_module, snapshot, release['campaign_set'], []),
        require_storage=False, now=lambda: 1_800_000_000.0)
    assert report['status'] == 'PASS'
    for mutate in (
            lambda value: value['rows'].reverse(),
            lambda value: value['rows'].__setitem__(1, json.loads(json.dumps(value['rows'][0]))),
            lambda value: value['campaign_set'].__setitem__('extra', True),
            lambda value: value['safety']['docker'].__setitem__('state', 'NOT_STARTED'),
            lambda value: value['safety'].__setitem__('bag_opened', True),
            lambda value: value['rows'][3].__setitem__('status', 'NOT_STARTED'),
            lambda value: (value.__setitem__('status', 'PARTIAL_FAILURE'),
                           value.__setitem__('failure', {
                               'kind': 'FAKE', 'phase': 'leg', 'message': 'fake'}),
                           value['summary'].__setitem__('status', 'PASS')),
            lambda value: value['rows'][0].__setitem__(
                'ended_at_unix', value['rows'][0]['started_at_unix'] - 1),
            lambda value: value.__setitem__('ended_at_unix', value['started_at_unix'] - 1)):
        candidate = json.loads(json.dumps(report))
        mutate(candidate)
        with pytest.raises(campaign_module.CampaignError):
            campaign_module._validate_campaign_report(candidate)


def test_collision_and_tampered_host_receipt_fail_before_promotion(
        campaign_module, monkeypatch, tmp_path):
    _profile, release, _profile_path, snapshot, image, _container = _fixture(
        campaign_module, monkeypatch, tmp_path)

    def collision(_name):
        return {'Id': 'existing'}

    with pytest.raises(campaign_module.CampaignError, match='already exists'):
        campaign_module.build_preflight(
            REPO, PROFILE, tmp_path / 'registration-plugin-release-campaign-collision',
            image_probe=image, container_probe=collision)

    leg_calls = []
    summary_calls = []
    report, _seal = campaign_module.run_campaign(
        REPO, PROFILE, tmp_path / 'registration-plugin-release-campaign-test-tamper',
        image_probe=image, container_probe=lambda _name: None,
        leg_executor=_fake_leg(campaign_module, snapshot, release['campaign_set'],
                               leg_calls, tamper_sidecar=True),
        summary_executor=_fake_summary(
            campaign_module, snapshot, release['campaign_set'], summary_calls),
        require_storage=False, now=lambda: 1_800_000_000.0)
    assert report['status'] == 'PARTIAL_FAILURE'
    assert report['summary']['status'] == 'NOT_RUN'
    assert not summary_calls


def test_seal_pair_failure_removes_only_owned_partial_files(campaign_module,
                                                            monkeypatch,
                                                            tmp_path):
    target = tmp_path / 'partial.receipt.json'

    def fail_chmod(*_args):
        raise OSError('synthetic chmod failure')

    monkeypatch.setattr(campaign_module.os, 'chmod', fail_chmod)
    with pytest.raises(OSError, match='synthetic chmod failure'):
        campaign_module._seal_pair(target, {
            'schema': campaign_module.SCHEMA, 'schema_version': 1,
            'status': 'PREFLIGHT_NOT_RUNTIME_VALIDATED',
        })
    assert not target.exists()
    assert not Path(str(target) + '.sha256').exists()
