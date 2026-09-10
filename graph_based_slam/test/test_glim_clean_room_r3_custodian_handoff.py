#!/usr/bin/env python3
"""Adversarial tests for the unsigned r3 custodian handoff boundary."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parents[2]
R3 = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


HANDOFF = _load(R3 / "apt_release_signature_handoff.py", "r3_custodian_handoff_test")
EXEC_TEST = _load(ROOT / "graph_based_slam/test/test_glim_clean_room_r3_apt_signature_executor.py",
                  "r3_custodian_handoff_executor_fixture")


def _write_json(path: Path, value: object):
    data = HANDOFF.canonical_bytes(value) + b"\n"
    path.write_bytes(data)
    path.chmod(0o444)
    return data


def _fixture(tmp_path: Path):
    fixture = EXEC_TEST._fixture(tmp_path / "inputs")
    plan = EXEC_TEST._plan(fixture, tmp_path / "fresh-output")
    plan_path = fixture["root"] / "plan.json"
    _write_json(plan_path, plan)
    machine_path = fixture["root"] / "machine.json"
    snapshot = {
        "raw_identifiers": {"machine_id": "0123456789abcdef0123456789abcdef",
                            "dmi_uuid": "01234567-89ab-cdef-0123-456789abcdef",
                            "board_serial": "HANDOFF-SERIAL-1234"},
        "public": {"architecture": "x86_64", "cpu_model": "fixture-cpu",
                    "logical_cpu_count": 4, "memory_total_kb": 2048,
                    "kernel_release": "fixture-kernel",
                    "toolchain": {"python_version": "3.12.0",
                                   "python_implementation": "CPython",
                                   "python_compiler": "fixture"}},
    }
    HANDOFF.MACHINE.capture_artifact(
        machine_path, campaign_id="campaign-a", provider=lambda: snapshot,
        captured_at="2027-01-01T00:00:00Z", status="LIVE_CAPTURED")
    output = tmp_path / "handoff.json"
    result = HANDOFF.build_request(
        output=output, plan_path=plan_path, machine_artifact=machine_path,
        root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"], campaign_id="campaign-a",
        issued_at=1_800_000_000, expires_at=1_800_000_600, nonce="N" * 64)
    return fixture, plan_path, machine_path, output, result


def test_handoff_positive_is_unsigned_and_reopens_all_bindings(tmp_path):
    _, _, _, output, result = _fixture(tmp_path)
    assert result["benchmark_eligible"] is False
    reopened = HANDOFF.validate_request(output)
    assert reopened["status"] == "PASS"
    assert reopened["offline_host_match_claimed"] is False
    request = json.loads(output.read_text())
    Draft202012Validator(json.loads(
        (R3 / "apt_release_signature_handoff.schema.json").read_text())).validate(request)
    assert "HANDOFF-SERIAL-1234" not in output.read_text()


@pytest.mark.parametrize("mutation", ["canonical", "custodian", "scope", "campaign", "candidate"])
def test_handoff_tamper_or_replay_fails_closed(tmp_path, mutation):
    _, _, _, output, _ = _fixture(tmp_path)
    request = json.loads(output.read_text())
    if mutation == "canonical":
        request["machine_identity"]["identity_sha256"] = "f" * 64
    elif mutation == "custodian":
        request["custodian"]["public_key_base64"] = "Zm9yZ2Vk"
    elif mutation == "scope":
        request["scope"]["output_root"] = "/tmp/other-output"
    elif mutation == "campaign":
        request["campaign_id"] = "other-campaign"
    else:
        request["candidate"]["file_sha256"] = "a" * 64
    request["canonical_sha256"] = HANDOFF.canonical_hash(request, "canonical_sha256")
    output.chmod(0o600)
    output.write_bytes(HANDOFF.canonical_bytes(request) + b"\n")
    output.chmod(0o444)
    with pytest.raises(HANDOFF.HandoffError):
        HANDOFF.validate_request(output)


def test_machine_replacement_and_symlink_are_rejected(tmp_path):
    _, _, machine_path, output, _ = _fixture(tmp_path)
    request = json.loads(output.read_text())
    target = machine_path.with_name("machine-target.json")
    target.write_bytes(machine_path.read_bytes())
    target.chmod(0o444)
    machine_path.unlink()
    machine_path.symlink_to(target.name)
    with pytest.raises(HANDOFF.HandoffError):
        HANDOFF.validate_request(output)
    del request


def test_synthetic_machine_artifact_cannot_enter_handoff(tmp_path):
    fixture = EXEC_TEST._fixture(tmp_path / "inputs")
    plan = EXEC_TEST._plan(fixture, tmp_path / "fresh-output")
    plan_path = fixture["root"] / "plan.json"
    _write_json(plan_path, plan)
    machine_path = fixture["root"] / "machine.json"
    snapshot = {"raw_identifiers": {"machine_id": "0123456789abcdef0123456789abcdef",
                                    "dmi_uuid": "01234567-89ab-cdef-0123-456789abcdef",
                                    "board_serial": "SYNTHETIC-SERIAL-1234"},
                "public": {"architecture": "x86_64", "cpu_model": "cpu",
                            "logical_cpu_count": 2, "memory_total_kb": 1,
                            "kernel_release": "kernel",
                            "toolchain": {"python_version": "3", "python_implementation": "CPython",
                                           "python_compiler": "compiler"}}}
    HANDOFF.MACHINE.capture_artifact(machine_path, campaign_id="campaign-a",
                                     provider=lambda: snapshot,
                                     captured_at="2027-01-01T00:00:00Z", status="SYNTHETIC_TEST")
    with pytest.raises(HANDOFF.HandoffError, match="live-capture"):
        HANDOFF.build_request(
            output=tmp_path / "handoff.json", plan_path=plan_path,
            machine_artifact=machine_path, root_inputs=fixture["root"],
            repository=fixture["repository"], deb_bindings=fixture["deb_bindings"],
            campaign_id="campaign-a", issued_at=1_800_000_000,
            expires_at=1_800_000_600, nonce="N" * 64)


def test_prepare_output_is_fresh_and_sidecar_is_bound(tmp_path):
    fixture, plan_path, machine_path, output, _ = _fixture(tmp_path)
    with pytest.raises(HANDOFF.HandoffError, match="fresh"):
        HANDOFF.build_request(
            output=output, plan_path=plan_path,
            machine_artifact=machine_path, root_inputs=fixture["root"],
            repository=fixture["repository"], deb_bindings=fixture["deb_bindings"],
            campaign_id="campaign-a", issued_at=1_800_000_000,
            expires_at=1_800_000_600, nonce="N" * 64)


def test_machine_descriptor_rechecks_validation_identity_after_read(tmp_path, monkeypatch):
    fixture, _, machine_path, _, _ = _fixture(tmp_path)
    original = HANDOFF.MACHINE.validate_file

    def forged_validation(path, **kwargs):
        result = original(path, **kwargs)
        result["artifact_sha256"] = "f" * 64
        return result

    monkeypatch.setattr(HANDOFF.MACHINE, "validate_file", forged_validation)
    with pytest.raises(HANDOFF.HandoffError, match="changed between validation"):
        HANDOFF._machine_descriptor(machine_path, fixture["root"], "campaign-a")


def test_fixed_policy_backend_is_reopened_as_a_complete_descriptor(tmp_path, monkeypatch):
    _fixture(tmp_path)
    original = HANDOFF._backend_descriptor

    def forged_backend():
        value = original()
        value["version"] = "forged"
        return value

    monkeypatch.setattr(HANDOFF, "_backend_descriptor", forged_backend)
    with pytest.raises(HANDOFF.HandoffError, match="backend source drift"):
        HANDOFF._fixed_sources()
