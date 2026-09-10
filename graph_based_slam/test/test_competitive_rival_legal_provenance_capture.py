# Copyright 2026 Sasaki
# All rights reserved.

"""Adversarial tests for the non-promoting rival legal capture packet."""

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path

import jsonschema
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


CAPTURE = _load(
    'competitive_rival_legal_provenance_capture',
    ROOT / 'scripts' / 'capture_competitive_rival_legal_provenance.py',
)
FIXTURE = _load(
    'competitive_rival_source_closure_fixture',
    ROOT / 'graph_based_slam' / 'test' /
    'test_competitive_rival_source_closure.py',
)


def _prepared(tmp_path: Path) -> tuple[dict, Path, Path]:
    source_root = tmp_path / 'source'
    source_root.mkdir()
    profile, _ = FIXTURE._fixture(source_root)
    profile_path = source_root / 'profile.yaml'
    profile_path.write_text(yaml.safe_dump(profile, sort_keys=True), encoding='utf-8')
    return profile, source_root, profile_path


def _schema() -> dict:
    return json.loads((ROOT / 'configs' / 'slam_benchmark_profiles' /
                       'competitive_rival_legal_provenance_capture_v1.schema.json').read_text(
                           encoding='utf-8'))


def _write_capture(tmp_path: Path) -> tuple[dict, Path, Path, Path]:
    profile, source_root, profile_path = _prepared(tmp_path)
    output = tmp_path / 'capture'
    document = CAPTURE.capture_legal_provenance(
        output_dir=output, root=source_root, profile_path=profile_path,
        observed_at='2026-08-25T20:06:15Z')
    return document, source_root, profile_path, output


def _reseal_fixture_selection(profile: dict, source_root: Path) -> None:
    closure = profile['competitive_slam_profile']['evidence_gate_v2'][
        'rival_source_closure']
    selection_path = source_root / 'selection-r2.yaml'
    selection = yaml.safe_load(selection_path.read_text(encoding='utf-8'))
    selection['closure_identity_sha256'] = \
        FIXTURE.MODULE.canonical_rival_source_closure_identity(closure)
    selection_path.write_text(yaml.safe_dump(selection, sort_keys=True),
                              encoding='utf-8')
    closure['active_selection']['sha256'] = FIXTURE.MODULE.sha256_file(selection_path)


def test_capture_is_immutable_nonpromoting_and_schema_valid(tmp_path):
    document, _, _, output = _write_capture(tmp_path)
    jsonschema.Draft202012Validator(_schema()).validate(document)
    assert document['status'] == 'REVIEW_REQUIRED'
    assert document['review_status'] == 'UNSIGNED_REVIEW_REQUIRED'
    assert document['benchmark_eligible'] is False
    assert document['claim_eligible'] is False
    assert document['remote_policy']['responses_status'] == 'NOT_RUN'
    assert CAPTURE.validate_capture_set(
        output_dir=output, root=tmp_path / 'source',
        profile_path=tmp_path / 'source' / 'profile.yaml')['status'] == 'REVIEW_REQUIRED'
    assert {path.name for path in output.iterdir()} == {
        CAPTURE.CAPTURE_FILE, CAPTURE.CAPTURE_SIDECAR}
    for path in output.iterdir():
        assert path.stat().st_nlink == 1
        assert path.stat().st_mode & 0o777 == 0o444


def test_pinned_archive_config_is_bound_without_local_observation(tmp_path):
    profile, source_root, profile_path = _prepared(tmp_path)
    closure = profile['competitive_slam_profile']['evidence_gate_v2'][
        'rival_source_closure']
    fast_source = closure['rivals']['fast_livo2']['sources'][0]
    archive_artifact = {
        'role': 'benchmark_hilti22_config',
        'path': 'config/HILTI22.yaml',
        'hash_kind': 'upstream_archive_file_sha256',
        'sha256': 'd' * 64,
        'status': 'READY',
    }
    fast_source['archive_artifacts'] = [archive_artifact]
    closure['rivals']['fast_livo2']['recipe']['configs'][0] = {
        'path': archive_artifact['path'],
        'path_kind': 'pinned_upstream_archive_relative_path',
        'hash_kind': archive_artifact['hash_kind'],
        'sha256': archive_artifact['sha256'],
        'source': 'fast_livo2',
        'role': archive_artifact['role'],
        'status': 'READY',
    }
    _reseal_fixture_selection(profile, source_root)
    profile_path.write_text(yaml.safe_dump(profile, sort_keys=True),
                            encoding='utf-8')
    document = CAPTURE.build_capture_document(
        root=source_root, profile_path=profile_path,
        observed_at='2026-08-25T20:06:15Z')
    recipe = document['systems']['fast_livo2']['recipe']
    pinned = [item for item in recipe['artifacts']
              if item['status'] == CAPTURE.PINNED_ARCHIVE_STATUS]
    assert len(pinned) == 1
    assert pinned[0]['path'] == 'config/HILTI22.yaml'
    assert pinned[0]['actual_sha256'] is None
    assert pinned[0]['observation'] == 'NOT_LOCALLY_OBSERVED'
    assert recipe['status'] == 'CURRENT_BYTES_MATCH_DECLARATIONS'
    jsonschema.Draft202012Validator(_schema()).validate(document)


def test_pinned_archive_kind_and_binding_must_match_source(tmp_path):
    profile, source_root, profile_path = _prepared(tmp_path)
    closure = profile['competitive_slam_profile']['evidence_gate_v2'][
        'rival_source_closure']
    source = closure['rivals']['fast_livo2']['sources'][0]
    source['archive_artifacts'] = [{
        'role': 'benchmark_hilti22_config',
        'path': 'config/HILTI22.yaml',
        'hash_kind': 'upstream_archive_file_sha256',
        'sha256': 'd' * 64,
        'status': 'READY',
    }]
    descriptor = {
        'path': 'config/HILTI22.yaml',
        'path_kind': 'unknown_path_kind',
        'hash_kind': 'upstream_archive_file_sha256',
        'sha256': 'd' * 64,
        'source': 'fast_livo2',
        'role': 'benchmark_hilti22_config',
        'status': 'READY',
    }
    closure['rivals']['fast_livo2']['recipe']['configs'][0] = descriptor
    _reseal_fixture_selection(profile, source_root)
    profile_path.write_text(yaml.safe_dump(profile, sort_keys=True),
                            encoding='utf-8')
    with pytest.raises(CAPTURE.LegalCaptureError):
        CAPTURE.build_capture_document(
            root=source_root, profile_path=profile_path,
            observed_at='2026-08-25T20:06:15Z')

    descriptor['path_kind'] = CAPTURE.PINNED_ARCHIVE_PATH_KIND
    descriptor['role'] = 'wrong-role'
    _reseal_fixture_selection(profile, source_root)
    profile_path.write_text(yaml.safe_dump(profile, sort_keys=True),
                            encoding='utf-8')
    with pytest.raises(CAPTURE.LegalCaptureError):
        CAPTURE.build_capture_document(
            root=source_root, profile_path=profile_path,
            observed_at='2026-08-25T20:06:15Z')


def test_remote_observation_requires_http_200_and_safe_headers():
    request = {
        'kind': 'archive',
        'url': 'https://github.com/koide3/glim/archive/' + 'a' * 40 + '.tar.gz',
        'expected_sha256': 'b' * 64,
    }
    observation = {
        **request,
        'response_status': 'OBSERVED',
        'final_url': request['url'],
        'http_status': 200,
        'redirect_count': 0,
        'body_size': 4,
        'body_sha256': 'b' * 64,
        'headers': {'content-type': 'application/gzip'},
        'reason': None,
    }
    CAPTURE.validate_remote_observation(request, observation)
    for mutation in (
        {'http_status': 206},
        {'http_status': 500},
        {'headers': {'set-cookie': 'secret'}},
        {'headers': {'content-type': 'bad\nvalue'}},
        {'redirect_count': 1, 'final_url': 'https://evil.example/x'},
    ):
        candidate = json.loads(json.dumps(observation))
        candidate.update(mutation)
        with pytest.raises(CAPTURE.LegalCaptureError):
            CAPTURE.validate_remote_observation(request, candidate)


def test_not_run_remote_observation_cannot_contain_observed_fields():
    request = {
        'kind': 'commit',
        'url': 'https://github.com/koide3/glim/commit/' + 'a' * 40,
        'expected_sha256': None,
    }
    observation = {
        **request,
        'response_status': 'NOT_RUN',
        'final_url': None,
        'http_status': None,
        'redirect_count': 0,
        'body_size': None,
        'body_sha256': None,
        'headers': {},
        'reason': 'network_not_used_in_capture',
    }
    CAPTURE.validate_remote_observation(request, observation)
    observed = dict(observation, http_status=200)
    with pytest.raises(CAPTURE.LegalCaptureError):
        CAPTURE.validate_remote_observation(request, observed)


def test_pending_remote_observation_cannot_smuggle_partial_bytes():
    request = {
        'kind': 'raw',
        'url': 'https://raw.githubusercontent.com/koide3/glim/' +
        'a' * 40 + '/LICENSE',
        'expected_sha256': 'b' * 64,
    }
    observation = {
        **request,
        'response_status': 'PENDING',
        'final_url': None,
        'http_status': None,
        'redirect_count': 0,
        'body_size': None,
        'body_sha256': None,
        'headers': {},
        'reason': 'custodian_review_pending',
    }
    CAPTURE.validate_remote_observation(request, observation)
    partial = dict(observation, body_size=1)
    with pytest.raises(CAPTURE.LegalCaptureError):
        CAPTURE.validate_remote_observation(request, partial)


def test_current_recipe_drift_is_not_silently_accepted(tmp_path):
    _, source_root, profile_path, output = _write_capture(tmp_path)
    (source_root / 'runner.py').write_text('# drift\n', encoding='utf-8')
    with pytest.raises(CAPTURE.LegalCaptureError):
        CAPTURE.validate_capture_set(
            output_dir=output, root=source_root, profile_path=profile_path)


def test_invalid_official_url_fails_before_packet_creation(tmp_path):
    profile, source_root, profile_path = _prepared(tmp_path)
    rival = profile['competitive_slam_profile']['evidence_gate_v2'][
        'rival_source_closure']['rivals']['glim']
    rival['sources'][0]['repository_url'] = 'https://example.invalid/glim.git'
    profile_path.write_text(yaml.safe_dump(profile, sort_keys=True), encoding='utf-8')
    with pytest.raises(CAPTURE.LegalCaptureError):
        CAPTURE.capture_legal_provenance(
            output_dir=tmp_path / 'capture', root=source_root,
            profile_path=profile_path, observed_at='2026-08-25T20:06:15Z')
    assert not (tmp_path / 'capture').exists()


def test_collision_extra_and_hardlink_are_rejected(tmp_path):
    _, source_root, profile_path, output = _write_capture(tmp_path)
    with pytest.raises(CAPTURE.LegalCaptureError):
        CAPTURE.capture_legal_provenance(
            output_dir=output, root=source_root, profile_path=profile_path)
    (output / 'extra').write_text('extra\n', encoding='utf-8')
    with pytest.raises(CAPTURE.LegalCaptureError):
        CAPTURE.validate_capture_set(
            output_dir=output, root=source_root, profile_path=profile_path)
    (output / 'extra').unlink()
    os.link(output / CAPTURE.CAPTURE_FILE, output / 'linked')
    (output / CAPTURE.CAPTURE_SIDECAR).unlink()
    os.rename(output / 'linked', output / CAPTURE.CAPTURE_SIDECAR)
    with pytest.raises(CAPTURE.LegalCaptureError):
        CAPTURE.validate_capture_set(
            output_dir=output, root=source_root, profile_path=profile_path)


def test_self_rehashed_ready_status_is_rejected(tmp_path):
    document, source_root, profile_path, output = _write_capture(tmp_path)
    payload_path = output / CAPTURE.CAPTURE_FILE
    payload_path.chmod(0o644)
    document['status'] = 'READY'
    payload = (json.dumps(document, indent=2, sort_keys=True) + '\n').encode()
    payload_path.write_bytes(payload)
    payload_path.chmod(0o444)
    sidecar = output / CAPTURE.CAPTURE_SIDECAR
    sidecar.chmod(0o644)
    sidecar.write_text(CAPTURE._sha(payload) + '  ' + CAPTURE.CAPTURE_FILE + '\n',
                       encoding='ascii')
    sidecar.chmod(0o444)
    with pytest.raises(CAPTURE.LegalCaptureError):
        CAPTURE.validate_capture_set(
            output_dir=output, root=source_root, profile_path=profile_path)


def test_symlink_parent_is_rejected(tmp_path):
    _, source_root, profile_path = _prepared(tmp_path)
    real_parent = tmp_path / 'real-parent'
    real_parent.mkdir()
    link_parent = tmp_path / 'link-parent'
    link_parent.symlink_to(real_parent, target_is_directory=True)
    with pytest.raises(CAPTURE.LegalCaptureError):
        CAPTURE.capture_legal_provenance(
            output_dir=link_parent / 'capture', root=source_root,
            profile_path=profile_path)


def test_document_shape_rejects_unknown_fields_and_ready_projection(tmp_path):
    document, _, _, _ = _write_capture(tmp_path)
    extra = dict(document, unexpected=True)
    with pytest.raises(CAPTURE.LegalCaptureError):
        CAPTURE._validate_document_shape(extra)
    ready = dict(document)
    ready['status'] = 'READY'
    with pytest.raises(CAPTURE.LegalCaptureError):
        CAPTURE._validate_document_shape(ready)
