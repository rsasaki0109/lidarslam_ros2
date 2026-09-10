#!/usr/bin/env python3
"""Focused tests for the additive, non-promoting competitive r3 candidate."""

from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path
import stat

import pytest

import scripts.prepare_competitive_execution_selection_r3_handoff as handoff_module
from scripts.prepare_competitive_execution_selection_r3_handoff import (
    CandidateError,
    prepare_handoff,
    REQUIRED_EXTERNAL_ARTIFACTS,
    validate_handoff,
)
from scripts.validate_competitive_execution_selection_r3 import (
    _validate_selection_binding,
    _validate_source_bindings,
    CANDIDATE_REL,
    ROOT,
    validate_candidate,
)
import yaml


CANDIDATE = ROOT / CANDIDATE_REL


def _candidate_document() -> dict:
    return json.loads(CANDIDATE.read_text(encoding='utf-8'))


def test_r3_candidate_is_structurally_valid_but_not_ready_and_active_profile_unchanged():
    report = validate_candidate(CANDIDATE)
    assert report['structural_valid'] is True
    assert report['status'] == 'NOT_READY'
    assert report['benchmark_eligible'] is False
    assert report['claim_eligible'] is False
    assert report['active_profile_switched'] is False
    assert 'active_profile_points_to_historical_execution_receipt' in report['blockers']
    profile = yaml.safe_load(
        (ROOT / 'configs/slam_benchmark_profiles/'
         'competitive_slam_v1.yaml').read_text())
    assert profile['competitive_slam_profile']['evidence_gate_v2'][
        'execution_selection_receipt_path'].endswith(
            'competitive_execution_selection_2026-08.yaml')


def test_current_source_hash_drift_is_rejected_without_resealing_candidate():
    candidate = _candidate_document()
    candidate['source_bindings']['runner']['ours']['sha256'] = '0' * 64
    with pytest.raises(CandidateError, match='stale'):
        _validate_source_bindings(candidate, ROOT)


def test_r2_closure_or_selection_swap_is_rejected():
    candidate = _candidate_document()
    candidate['r2_selection_binding']['closure_identity_sha256'] = '0' * 64
    profile = yaml.safe_load(
        (ROOT / 'configs/slam_benchmark_profiles/'
         'competitive_slam_v1.yaml').read_text())
    with pytest.raises(CandidateError):
        _validate_selection_binding(candidate, ROOT, profile)


def test_historical_receipt_is_explicitly_rejected_and_not_promoted():
    report = validate_candidate(CANDIDATE)
    assert report['structural_valid'] is True
    old = _candidate_document()['prior_campaign_lineage']
    assert old['promotion_status'] == 'REJECTED_HISTORICAL_INVALID'
    assert old['superseded_by'] == 'competitive-execution-selection-2026-08-r2'


def test_unsigned_handoff_binds_candidate_and_lists_every_external_blocker(tmp_path: Path):
    output = tmp_path / 'handoff.json'
    prepare_handoff(output=output)
    result = validate_handoff(output)
    assert result['structural_valid'] is True
    assert result['status'] == 'UNSIGNED_REVIEW_REQUIRED'
    assert result['benchmark_eligible'] is False
    assert set(result['missing_external_artifacts']) == set(REQUIRED_EXTERNAL_ARTIFACTS)
    assert output.with_name(output.name + '.sha256').is_file()


def test_handoff_self_rehash_cannot_change_candidate_binding(tmp_path: Path):
    output = tmp_path / 'handoff.json'
    prepare_handoff(output=output)
    document = json.loads(output.read_text())
    document['candidate_binding']['candidate_id'] = 'other-campaign'
    document['handoff_identity_sha256'] = hashlib.sha256(
        (json.dumps({k: v for k, v in document.items()
                     if k != 'handoff_identity_sha256'}, sort_keys=True,
                    separators=(',', ':'), ensure_ascii=True) + '\n').encode()).hexdigest()
    output.chmod(0o644)
    output.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
    output.chmod(0o444)
    sidecar = output.with_name(output.name + '.sha256')
    sidecar.chmod(0o644)
    sidecar.write_text(
        hashlib.sha256(output.read_bytes()).hexdigest() + '  ' + output.name + '\n')
    sidecar.chmod(0o444)
    with pytest.raises(CandidateError, match='candidate binding'):
        validate_handoff(output)


def test_handoff_sidecar_detects_byte_tamper(tmp_path: Path):
    output = tmp_path / 'handoff.json'
    prepare_handoff(output=output)
    output.chmod(0o644)
    output.write_bytes(output.read_bytes() + b' tamper\n')
    output.chmod(0o444)
    with pytest.raises(CandidateError, match='sidecar'):
        validate_handoff(output)


def test_reviewed_external_manifest_requires_exact_coverage_and_bytes(tmp_path: Path):
    artifact_root = tmp_path / 'reviewed'
    artifact_root.mkdir()
    artifacts = {}
    for kind in REQUIRED_EXTERNAL_ARTIFACTS:
        name = kind + '.json'
        path = artifact_root / name
        path.write_text(json.dumps({'kind': kind}, sort_keys=True) + '\n')
        path.chmod(0o444)
        artifacts[kind] = {
            'status': 'REVIEWED_EXTERNAL', 'path': name,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    manifest = artifact_root / 'manifest.json'
    manifest.write_text(json.dumps({
        'schema_version': 1,
        'manifest_kind': 'competitive_execution_selection_r3_external_artifacts_v1',
        'artifacts': artifacts,
    }, indent=2, sort_keys=True) + '\n')
    manifest.chmod(0o444)
    output = tmp_path / 'reviewed-handoff.json'
    prepare_handoff(output=output, external_manifest=manifest,
                    external_root=artifact_root)
    result = validate_handoff(output)
    assert result['structural_valid'] is True
    assert result['missing_external_artifacts'] == []


def test_cross_campaign_external_manifest_is_rejected(tmp_path: Path):
    artifact_root = tmp_path / 'reviewed'
    artifact_root.mkdir()
    artifacts = {}
    for kind in REQUIRED_EXTERNAL_ARTIFACTS:
        path = artifact_root / (kind + '.json')
        path.write_text('{}\n')
        path.chmod(0o444)
        artifacts[kind] = {
            'status': 'REVIEWED_EXTERNAL', 'path': path.name,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    manifest = artifact_root / 'manifest.json'
    manifest.write_text(json.dumps({
        'schema_version': 1,
        'manifest_kind': 'competitive_execution_selection_r3_external_artifacts_v1',
        'artifacts': artifacts,
    }, sort_keys=True) + '\n')
    manifest.chmod(0o444)
    output = tmp_path / 'handoff.json'
    prepare_handoff(output=output, external_manifest=manifest,
                    external_root=artifact_root)
    document = json.loads(output.read_text())
    document['selection_binding']['r2_closure_id'] = 'other-closure'
    document['handoff_identity_sha256'] = hashlib.sha256(
        (json.dumps({k: v for k, v in document.items()
                     if k != 'handoff_identity_sha256'}, sort_keys=True,
                    separators=(',', ':'), ensure_ascii=True) + '\n').encode()).hexdigest()
    output.chmod(0o644)
    output.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
    output.chmod(0o444)
    sidecar = output.with_name(output.name + '.sha256')
    sidecar.chmod(0o644)
    sidecar.write_text(
        hashlib.sha256(output.read_bytes()).hexdigest() + '  ' + output.name + '\n')
    sidecar.chmod(0o444)
    with pytest.raises(CandidateError, match='selection/profile'):
        validate_handoff(output)


def test_candidate_schema_is_strict():
    jsonschema = pytest.importorskip('jsonschema')
    schema = json.loads((ROOT / 'configs/slam_benchmark_profiles/'
                        'competitive_execution_selection_r3_candidate_v1.schema.json').read_text())
    candidate = _candidate_document()
    jsonschema.Draft7Validator(schema).validate(candidate)
    extra = copy.deepcopy(candidate)
    extra['unexpected'] = True
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.Draft7Validator(schema).validate(extra)


def test_handoff_pair_is_read_only_and_single_link(tmp_path: Path):
    output = tmp_path / 'handoff.json'
    prepare_handoff(output=output)
    sidecar = output.with_name(output.name + '.sha256')
    assert stat.S_IMODE(output.stat().st_mode) == 0o444
    assert stat.S_IMODE(sidecar.stat().st_mode) == 0o444
    alias = tmp_path / 'handoff-alias.json'
    os.link(output, alias)
    with pytest.raises(CandidateError, match='single-link'):
        validate_handoff(output)


def test_handoff_validator_rejects_writable_pair(tmp_path: Path):
    output = tmp_path / 'handoff.json'
    prepare_handoff(output=output)
    output.chmod(0o644)
    with pytest.raises(CandidateError, match='immutable mode 0444'):
        validate_handoff(output)
    output.chmod(0o444)
    output.with_name(output.name + '.sha256').chmod(0o644)
    with pytest.raises(CandidateError, match='immutable mode 0444'):
        validate_handoff(output)


def test_handoff_output_collision_and_symlink_parent_are_rejected(tmp_path: Path):
    output = tmp_path / 'handoff.json'
    prepare_handoff(output=output)
    with pytest.raises(CandidateError, match='already exists'):
        prepare_handoff(output=output)
    real_parent = tmp_path / 'real-parent'
    real_parent.mkdir()
    symlink_parent = tmp_path / 'symlink-parent'
    symlink_parent.symlink_to(real_parent, target_is_directory=True)
    with pytest.raises(CandidateError, match='symlink'):
        prepare_handoff(output=symlink_parent / 'handoff.json')


def test_partial_pair_seal_removes_only_owned_output(tmp_path: Path, monkeypatch):
    output = tmp_path / 'handoff.json'
    original = handoff_module._exclusive_write
    calls = 0

    def fail_sidecar(path: Path, payload: bytes):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError('injected sidecar failure')
        return original(path, payload)

    monkeypatch.setattr(handoff_module, '_exclusive_write', fail_sidecar)
    with pytest.raises(OSError, match='injected sidecar failure'):
        prepare_handoff(output=output)
    assert not output.exists()
    assert not output.with_name(output.name + '.sha256').exists()


def test_handoff_mutation_between_reads_is_rejected(tmp_path: Path, monkeypatch):
    output = tmp_path / 'handoff.json'
    prepare_handoff(output=output)
    original = handoff_module._read_immutable
    mutated = False

    def mutate_after_first_read(path: Path, label: str, **kwargs):
        nonlocal mutated
        result = original(path, label, **kwargs)
        if path == output and not mutated:
            mutated = True
            output.chmod(0o644)
            output.write_bytes(result[0] + b' mutation\n')
            output.chmod(0o444)
        return result

    monkeypatch.setattr(handoff_module, '_read_immutable', mutate_after_first_read)
    with pytest.raises(CandidateError, match='changed'):
        validate_handoff(output)


def test_external_manifest_swap_is_rejected(tmp_path: Path):
    artifact_root = tmp_path / 'reviewed'
    artifact_root.mkdir()
    artifacts = {}
    for kind in REQUIRED_EXTERNAL_ARTIFACTS:
        path = artifact_root / (kind + '.json')
        path.write_text('{}\n')
        path.chmod(0o444)
        artifacts[kind] = {
            'status': 'REVIEWED_EXTERNAL', 'path': path.name,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    manifest = artifact_root / 'manifest.json'
    manifest.write_text(json.dumps({
        'schema_version': 1,
        'manifest_kind': 'competitive_execution_selection_r3_external_artifacts_v1',
        'artifacts': artifacts,
    }, sort_keys=True) + '\n')
    manifest.chmod(0o444)
    output = tmp_path / 'handoff.json'
    prepare_handoff(output=output, external_manifest=manifest,
                    external_root=artifact_root)
    backup = artifact_root / 'manifest-copy.json'
    manifest.rename(backup)
    manifest.symlink_to(backup)
    with pytest.raises(CandidateError, match='symlink'):
        validate_handoff(output)
