#!/usr/bin/env python3
"""Adversarial tests for the external machine-PoP handoff packet."""

from __future__ import annotations

import json
from pathlib import Path
import stat

from graph_based_slam.test.test_competitive_execution_machine_identity_pop import (
    _fixture,
    _snapshot,
    STAMP,
)
from jsonschema import Draft202012Validator
import pytest

import scripts.capture_competitive_execution_machine_identity_pop as pop
import scripts.prepare_competitive_execution_machine_identity_pop_handoff as handoff


def _external_policy_sidecar(path: Path) -> None:
    value = json.loads(path.read_text())
    data = pop._canonical(value) + b'\n'
    path.chmod(0o644)
    path.write_bytes(data)
    path.chmod(0o444)
    sidecar = pop._sidecar(
        path, data, value, handoff.POLICY_SIDECAR_SCHEMA)
    sidecar_path = path.with_name(path.name + '.sha256.json')
    if sidecar_path.exists():
        sidecar_path.chmod(0o644)
    sidecar_data = pop._canonical(sidecar) + b'\n'
    sidecar_path.write_bytes(sidecar_data)
    sidecar_path.chmod(0o444)


def _live_fixture(tmp_path: Path):
    fixture = _fixture(tmp_path)
    _external_policy_sidecar(fixture['policy'])
    pop.finalize_challenge(
        fixture['final'], challenge=fixture['challenge'], proof=fixture['proof'],
        provider=lambda: _snapshot(), _policy_path=fixture['policy'],
        _allow_test_policy=True, _now=STAMP + 1, _status='LIVE_CAPTURED')
    return fixture


def _prepare(tmp_path: Path):
    fixture = _live_fixture(tmp_path)
    output = tmp_path / 'machine-handoff.json'
    result = handoff.prepare_handoff(
        output=output, challenge=fixture['challenge'],
        key_descriptor=fixture['key'], proof=fixture['proof'],
        machine_artifact=fixture['final'], trust_policy=fixture['policy'])
    return fixture, output, result


def test_external_ready_policy_and_pop_sources_are_bound_offline(tmp_path):
    fixture, output, result = _prepare(tmp_path)
    assert result['structural_valid'] is True
    assert result['trust_policy_status'] == 'READY'
    assert result['live_host_match'] is False
    assert result['promotion_allowed'] is False
    assert stat.S_IMODE(output.stat().st_mode) == 0o444
    assert stat.S_IMODE(output.with_name(
        output.name + '.sha256.json').stat().st_mode) == 0o444
    packet = output.read_text()
    assert 'PRIVATE KEY' not in packet
    schema_path = pop.ROOT / 'configs/slam_benchmark_profiles/' \
        'competitive_execution_machine_identity_pop_handoff_v1.schema.json'
    schema = json.loads(schema_path.read_text())
    Draft202012Validator(schema).validate(json.loads(packet))
    assert handoff.validate_handoff(output)['host_match_status'] == (
        'HOST_MATCH_NOT_CHECKED')


def test_checked_in_policy_cannot_be_used_as_external_ready_policy(tmp_path):
    fixture = _live_fixture(tmp_path)
    output = tmp_path / 'rejected.json'
    with pytest.raises(handoff.HandoffError, match='checked-in'):
        handoff.prepare_handoff(
            output=output, challenge=fixture['challenge'],
            key_descriptor=fixture['key'], proof=fixture['proof'],
            machine_artifact=fixture['final'],
            trust_policy=pop.TRUST_POLICY_PATH)
    assert not output.exists()


def test_synthetic_artifact_cannot_enter_external_handoff(tmp_path):
    fixture = _fixture(tmp_path)
    _external_policy_sidecar(fixture['policy'])
    pop.finalize_challenge(
        fixture['final'], challenge=fixture['challenge'], proof=fixture['proof'],
        provider=lambda: _snapshot(), _policy_path=fixture['policy'],
        _allow_test_policy=True, _now=STAMP + 1, _status='SYNTHETIC_TEST')
    with pytest.raises(handoff.HandoffError, match='LIVE_CAPTURED'):
        handoff.prepare_handoff(
            output=tmp_path / 'machine-handoff.json',
            challenge=fixture['challenge'], key_descriptor=fixture['key'],
            proof=fixture['proof'], machine_artifact=fixture['final'],
            trust_policy=fixture['policy'])


def test_ready_policy_key_swap_is_not_accepted_as_custodian_handoff(tmp_path):
    fixture = _live_fixture(tmp_path)
    policy = json.loads(fixture['policy'].read_text())
    policy['authorized_keys'][0]['public_key_sha256'] = 'b' * 64
    policy['canonical_sha256'] = pop._canonical_hash(policy, 'canonical_sha256')
    fixture['policy'].chmod(0o644)
    fixture['policy'].write_bytes(pop._canonical(policy) + b'\n')
    fixture['policy'].chmod(0o444)
    _external_policy_sidecar(fixture['policy'])
    with pytest.raises(handoff.HandoffError, match='key binding'):
        handoff.prepare_handoff(
            output=tmp_path / 'machine-handoff.json',
            challenge=fixture['challenge'], key_descriptor=fixture['key'],
            proof=fixture['proof'], machine_artifact=fixture['final'],
            trust_policy=fixture['policy'])


def test_packet_tamper_and_policy_sidecar_mutation_fail_closed(tmp_path):
    fixture, output, _ = _prepare(tmp_path)
    packet = json.loads(output.read_text())
    packet['external_review']['auto_promoted'] = True
    output.chmod(0o644)
    output.write_bytes(pop._canonical(packet) + b'\n')
    output.chmod(0o444)
    with pytest.raises(handoff.HandoffError):
        handoff.validate_handoff(output)

    # Rebuild a fresh packet, then make the external policy sidecar writable.
    fixture, output, _ = _prepare(tmp_path / 'sidecar')
    policy_sidecar = fixture['policy'].with_name(
        fixture['policy'].name + '.sha256.json')
    policy_sidecar.chmod(0o644)
    with pytest.raises(handoff.HandoffError, match='0444'):
        handoff.validate_handoff(output)


def test_handoff_output_collision_and_asset_root_symlink_fail_closed(tmp_path):
    fixture = _live_fixture(tmp_path)
    output = tmp_path / 'collision.json'
    output.write_text('occupied')
    with pytest.raises(handoff.HandoffError):
        handoff.prepare_handoff(
            output=output, challenge=fixture['challenge'],
            key_descriptor=fixture['key'], proof=fixture['proof'],
            machine_artifact=fixture['final'], trust_policy=fixture['policy'])

    alias = tmp_path / 'alias'
    alias.symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(handoff.HandoffError):
        handoff.prepare_handoff(
            output=alias / 'handoff.json', challenge=fixture['challenge'],
            key_descriptor=fixture['key'], proof=fixture['proof'],
            machine_artifact=fixture['final'], trust_policy=fixture['policy'])
