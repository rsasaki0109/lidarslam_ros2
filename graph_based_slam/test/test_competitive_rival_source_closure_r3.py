#!/usr/bin/env python3
"""Adversarial tests for the additive rival-source closure r3 candidate."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import jsonschema
import pytest

import scripts.prepare_competitive_rival_source_closure_r3_handoff as handoff
import scripts.validate_competitive_rival_source_closure_r3 as candidate_module
import yaml


ROOT = Path(__file__).resolve().parents[2]
CANDIDATE_PATH = ROOT / candidate_module.CANDIDATE_REL
SELECTION_PATH = ROOT / candidate_module.SELECTION_REL


def _json(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def test_candidate_and_selection_are_not_ready_but_canonical_and_schema_bound():
    report = candidate_module.validate_checked_in(ROOT)
    assert report['status'] == 'NOT_READY'
    assert report['benchmark_eligible'] is False
    assert report['claim_eligible'] is False
    candidate = _json(CANDIDATE_PATH)
    selection = _json(SELECTION_PATH)
    assert (candidate['candidate_identity_sha256'] ==
            candidate_module.candidate_identity_sha256(candidate))
    assert (selection['selection_identity_sha256'] ==
            candidate_module.selection_identity_sha256(selection))
    for relative in (
        'competitive_rival_source_closure_r3_candidate_v1.schema.json',
        'competitive_rival_source_closure_r3_selection_candidate_v1.schema.json',
        'competitive_rival_source_closure_r3_handoff_v1.schema.json',
    ):
        schema = _json(ROOT / 'configs/slam_benchmark_profiles' / relative)
        jsonschema.Draft202012Validator.check_schema(schema)


def test_r2_closure_is_exactly_inherited_and_runner_drift_is_only_rebound():
    candidate = _json(CANDIDATE_PATH)
    profile = yaml.safe_load((ROOT / candidate_module.PROFILE_REL).read_text())
    closure = profile['competitive_slam_profile']['evidence_gate_v2'][
        'rival_source_closure']
    assert candidate['r2_inherited_closure'] == closure
    assert candidate['r2_lineage']['closure_revision'] == 2
    for system in ('glim', 'fast_livo2'):
        binding = candidate['current_runner_bindings'][system]
        assert binding['status'] == 'CURRENT_BYTES_REBOUND'
        assert binding['current_sha256'] == hashlib.sha256(
            (ROOT / binding['path']).read_bytes()).hexdigest()
        assert binding['current_sha256'] != binding['r2_sha256']


def test_legal_blockers_and_capture_contract_cannot_be_auto_reviewed():
    candidate = _json(CANDIDATE_PATH)
    assert candidate['legal_status']['status'] == 'NOT_READY'
    assert candidate['legal_status']['external_review_required'] is True
    assert candidate['legal_status']['package_metadata_is_not_license_text'] is True
    assert candidate['legal_capture_contract']['auto_review'] is False
    assert candidate['promotion_policy']['active_profile_switch'] is False
    assert candidate['promotion_policy']['requires_external_reviewed_packet'] is True
    assert candidate['source_provenance']['remote_observation'] == 'NOT_RUN'


def test_candidate_self_rehash_or_inherited_closure_swap_fails():
    candidate = _json(CANDIDATE_PATH)
    mutated = copy.deepcopy(candidate)
    mutated['current_runner_bindings']['glim']['current_sha256'] = '0' * 64
    with pytest.raises(candidate_module.CandidateError):
        candidate_module.validate_candidate_document(mutated, ROOT)
    mutated = copy.deepcopy(candidate)
    mutated['r2_inherited_closure']['status'] = 'READY'
    with pytest.raises(candidate_module.CandidateError):
        candidate_module.validate_candidate_document(mutated, ROOT)


def test_selection_self_rehash_or_cross_candidate_binding_fails():
    candidate = _json(CANDIDATE_PATH)
    selection = _json(SELECTION_PATH)
    mutated = copy.deepcopy(selection)
    mutated['candidate_binding']['candidate_identity_sha256'] = 'f' * 64
    with pytest.raises(candidate_module.CandidateError):
        candidate_module.validate_selection_document(
            mutated, ROOT, candidate=candidate,
            candidate_file_sha256=hashlib.sha256(CANDIDATE_PATH.read_bytes()).hexdigest())
    mutated = copy.deepcopy(selection)
    mutated['status'] = 'READY'
    with pytest.raises(candidate_module.CandidateError):
        candidate_module.validate_selection_document(
            mutated, ROOT, candidate=candidate,
            candidate_file_sha256=hashlib.sha256(CANDIDATE_PATH.read_bytes()).hexdigest())


def test_unsigned_handoff_binds_both_candidates_and_no_external_packet(tmp_path: Path):
    output = tmp_path / 'rival-source-handoff.json'
    result = handoff.prepare_handoff(output=output)
    assert result['status'] == 'UNSIGNED_REVIEW_REQUIRED'
    assert result['legal_review_status'] == 'NOT_PROVIDED'
    assert result['benchmark_eligible'] is False
    assert output.stat().st_mode & 0o777 == 0o444
    assert output.with_name(output.name + '.sha256').stat().st_mode & 0o777 == 0o444
    assert handoff.validate_handoff(output)['structural_valid'] is True


def test_handoff_self_rehash_cannot_change_selection_or_review_status(tmp_path: Path):
    output = tmp_path / 'rival-source-handoff.json'
    handoff.prepare_handoff(output=output)
    document = _json(output)
    document['selection_binding']['selection_id'] = 'other-campaign'
    document['handoff_identity_sha256'] = handoff._canonical_without_identity(document)
    output.chmod(0o644)
    output.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
    output.chmod(0o444)
    sidecar = output.with_name(output.name + '.sha256')
    sidecar.chmod(0o644)
    sidecar.write_text(hashlib.sha256(output.read_bytes()).hexdigest() +
                       '  ' + output.name + '\n')
    sidecar.chmod(0o444)
    with pytest.raises(handoff.CandidateError):
        handoff.validate_handoff(output)


def test_handoff_cannot_overwrite_collision_or_accept_symlink(tmp_path: Path):
    output = tmp_path / 'rival-source-handoff.json'
    output.write_text('collision\n')
    with pytest.raises(handoff.CandidateError):
        handoff.prepare_handoff(output=output)
    output.unlink()
    target = tmp_path / 'target.json'
    target.write_text('target\n')
    output.symlink_to(target)
    with pytest.raises(handoff.CandidateError):
        handoff.prepare_handoff(output=output)


def test_archive_relative_binding_is_not_claimed_as_local_observation():
    candidate = _json(CANDIDATE_PATH)
    entries = candidate['current_recipe_bindings']['fast_livo2']['artifacts']
    pinned = [entry for entry in entries if entry['role'] == 'config[1]']
    assert len(pinned) == 1
    assert pinned[0]['status'] == 'PINNED_UPSTREAM_ARCHIVE_UNOBSERVED'
    assert pinned[0]['current_sha256'] is None


def test_checked_in_r2_and_active_profile_bytes_are_not_rewritten():
    profile = yaml.safe_load((ROOT / candidate_module.PROFILE_REL).read_text())
    closure = profile['competitive_slam_profile']['evidence_gate_v2'][
        'rival_source_closure']
    assert closure['closure_revision'] == 2
    assert closure['status'] == 'NOT_READY'
    assert closure['active_selection']['status'] == 'CURRENT'
    assert candidate_module.R2_SELECTION_REL.endswith('r2.yaml')
