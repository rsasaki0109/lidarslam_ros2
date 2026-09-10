#!/usr/bin/env python3
"""Adversarial tests for the non-secret r3 machine identity boundary."""

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


MACHINE = _load(R3 / "apt_machine_identity.py", "r3_machine_identity_test_contract")


def _snapshot():
    return {
        "raw_identifiers": {
            "machine_id": "0123456789abcdef0123456789abcdef",
            "dmi_uuid": "01234567-89ab-cdef-0123-456789abcdef",
            "board_serial": "SERIAL-SECRET-SENTINEL",
        },
        "public": {
            "architecture": "x86_64", "cpu_model": "fixture-cpu",
            "logical_cpu_count": 4, "memory_total_kb": 2048,
            "kernel_release": "fixture-kernel",
            "toolchain": {"python_version": "3.12.0",
                           "python_implementation": "CPython",
                           "python_compiler": "fixture-compiler"},
        },
    }


def _write(path: Path, data: bytes):
    path.write_bytes(data)
    path.chmod(0o600)


def _sealed(tmp_path: Path, *, status: str = "LIVE_CAPTURED"):
    tmp_path.mkdir(parents=True, exist_ok=True)
    artifact = MACHINE.build_artifact(campaign_id="campaign-a", snapshot=_snapshot(),
                                      captured_at="2027-01-01T00:00:00Z", status=status)
    path = tmp_path / "machine.json"
    data = MACHINE.canonical_bytes(artifact) + b"\n"
    _write(path, data)
    sidecar = MACHINE._sidecar(path, data, artifact)
    sidecar_path = path.with_name(path.name + ".sha256.json")
    _write(sidecar_path, MACHINE.canonical_bytes(sidecar) + b"\n")
    path.chmod(0o444)
    sidecar_path.chmod(0o444)
    return path, artifact, data


def test_artifact_hashes_only_stable_ids_and_never_raw_values(tmp_path):
    path, artifact, data = _sealed(tmp_path)
    assert b"0123456789abcdef0123456789abcdef" not in data
    assert b"01234567-89ab-cdef-0123-456789abcdef" not in data
    assert b"SERIAL-SECRET-SENTINEL" not in data
    assert MACHINE.validate_file(path)["live_host_match"] is False
    assert MACHINE.validate_file(path)["identity_sha256"] == artifact["identity_sha256"]


def test_schema_and_sidecar_are_strict(tmp_path):
    path, artifact, _ = _sealed(tmp_path)
    schema = json.loads((R3 / "apt_machine_identity.schema.json").read_text())
    Draft202012Validator(schema).validate(artifact)
    sidecar = json.loads(path.with_name(path.name + ".sha256.json").read_text())
    side_schema = json.loads((R3 / "apt_machine_identity_sidecar.schema.json").read_text())
    Draft202012Validator(side_schema).validate(sidecar)
    artifact["unexpected"] = True
    with pytest.raises(MACHINE.MachineIdentityError):
        MACHINE.validate_artifact(artifact)


def test_missing_stable_identifier_and_missing_public_projection_fail(tmp_path):
    snapshot = _snapshot()
    snapshot["raw_identifiers"]["dmi_uuid"] = ""
    snapshot["raw_identifiers"]["board_serial"] = ""
    with pytest.raises(MACHINE.MachineIdentityError, match="stable identifiers"):
        MACHINE.build_artifact(campaign_id="campaign-a", snapshot=snapshot,
                               captured_at="2027-01-01T00:00:00Z")
    snapshot = _snapshot()
    snapshot["public"]["cpu_model"] = ""
    with pytest.raises(MACHINE.MachineIdentityError):
        MACHINE.validate_artifact(MACHINE.build_artifact(
            campaign_id="campaign-a", snapshot=snapshot,
            captured_at="2027-01-01T00:00:00Z"))


@pytest.mark.parametrize("placeholder", [
    "to be filled by o.e.m.", "default string", "system serial number",
    "not applicable", "not available", "unknown serial number",
])
def test_dmi_placeholders_are_not_stable_identifiers(placeholder):
    snapshot = _snapshot()
    snapshot["raw_identifiers"]["board_serial"] = placeholder
    with pytest.raises(MACHINE.MachineIdentityError, match="invalid stable identifier"):
        MACHINE.build_artifact(campaign_id="campaign-a", snapshot=snapshot,
                               captured_at="2027-01-01T00:00:00Z")


def test_duplicate_stable_identifier_values_do_not_satisfy_minimum():
    snapshot = _snapshot()
    snapshot["raw_identifiers"]["dmi_uuid"] = ""
    snapshot["raw_identifiers"]["board_serial"] = snapshot["raw_identifiers"]["machine_id"]
    with pytest.raises(MACHINE.MachineIdentityError, match="distinct stable identifiers"):
        MACHINE.build_artifact(campaign_id="campaign-a", snapshot=snapshot,
                               captured_at="2027-01-01T00:00:00Z")


def test_tamper_canonical_or_sidecar_fails(tmp_path):
    path, artifact, _ = _sealed(tmp_path)
    artifact["public"]["cpu_model"] = "tampered"
    path.chmod(0o600)
    path.write_bytes(MACHINE.canonical_bytes(artifact) + b"\n")
    with pytest.raises(MACHINE.MachineIdentityError):
        MACHINE.validate_file(path)
    path, _, _ = _sealed(tmp_path / "second")
    sidecar = path.with_name(path.name + ".sha256.json")
    sidecar.chmod(0o600)
    value = json.loads(sidecar.read_text())
    value["artifact_bytes"] += 1
    sidecar.write_bytes(MACHINE.canonical_bytes(value) + b"\n")
    with pytest.raises(MACHINE.MachineIdentityError):
        MACHINE.validate_file(path)


@pytest.mark.parametrize("kind", ["symlink", "hardlink"])
def test_artifact_symlink_and_hardlink_fail(tmp_path, kind):
    path, _, data = _sealed(tmp_path)
    replacement = tmp_path / "replacement.json"
    if kind == "symlink":
        path.unlink()
        path.symlink_to(replacement)
        _write(replacement, data)
    else:
        path.unlink()
        path.hardlink_to(tmp_path / "machine.json.sha256.json")
    with pytest.raises(MACHINE.MachineIdentityError):
        MACHINE.validate_file(path)


def test_live_mode_is_distinct_and_compares_provider(tmp_path):
    path, _, _ = _sealed(tmp_path, status="LIVE_CAPTURED")
    with pytest.raises(MACHINE.MachineIdentityError, match="does not match"):
        MACHINE.validate_file(path, live=True, provider=lambda: {
            **_snapshot(), "public": {**_snapshot()["public"], "cpu_model": "other"}})
    assert MACHINE.validate_file(path, live=True,
                                 provider=lambda: _snapshot())["live_host_match"] is True


def test_capture_requires_fresh_pair_and_cleans_partial_sidecar(tmp_path, monkeypatch):
    output = tmp_path / "captured.json"
    provider = _snapshot
    MACHINE.capture_artifact(output, campaign_id="campaign-a", provider=provider,
                             captured_at="2027-01-01T00:00:00Z", status="SYNTHETIC_TEST")
    with pytest.raises(MACHINE.MachineIdentityError, match="fresh"):
        MACHINE.capture_artifact(output, campaign_id="campaign-a", provider=provider,
                                 captured_at="2027-01-01T00:00:00Z", status="SYNTHETIC_TEST")
    other = tmp_path / "other.json"
    original = MACHINE._write_exclusive
    calls = {"count": 0}

    def fail_on_sidecar(path, data, label, **kwargs):
        calls["count"] += 1
        if calls["count"] == 2:
            raise MACHINE.MachineIdentityError("synthetic sidecar failure")
        return original(path, data, label, **kwargs)

    monkeypatch.setattr(MACHINE, "_write_exclusive", fail_on_sidecar)
    with pytest.raises(MACHINE.MachineIdentityError, match="synthetic sidecar"):
        MACHINE.capture_artifact(other, campaign_id="campaign-a", provider=provider,
                                 captured_at="2027-01-01T00:00:00Z", status="SYNTHETIC_TEST")
    assert not other.exists()
    assert not other.with_name(other.name + ".sha256.json").exists()
