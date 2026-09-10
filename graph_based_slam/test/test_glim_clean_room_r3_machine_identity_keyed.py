#!/usr/bin/env python3
"""Adversarial tests for the DMI-less keyed r3 machine identity path."""

from __future__ import annotations

import base64
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parents[2]
R3 = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MACHINE = _load(R3 / "apt_machine_identity.py", "r3_machine_identity_keyed_test")


def _snapshot(machine_id: str = "0123456789abcdef0123456789abcdef"):
    return {
        "raw_identifiers": {
            "machine_id": machine_id,
            "dmi_uuid": "",
            "board_serial": "",
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


def _write_input(path: Path, value: dict[str, object]) -> bytes:
    data = MACHINE.canonical_bytes(value) + b"\n"
    path.write_bytes(data)
    path.chmod(0o444)
    sidecar = MACHINE._key_sidecar(
        path, data, value, schema=MACHINE.KEY_INPUT_SIDECAR_SCHEMA)
    sidecar_path = path.with_name(path.name + ".sha256.json")
    sidecar_data = MACHINE.canonical_bytes(sidecar) + b"\n"
    sidecar_path.write_bytes(sidecar_data)
    sidecar_path.chmod(0o444)
    return data


def _key_descriptor(private: Ed25519PrivateKey, campaign_id: str = "campaign-a"):
    public = private.public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw)
    value = {
        "schema": MACHINE.KEY_DESCRIPTOR_SCHEMA, "schema_version": 1,
        "status": "EXTERNALLY_PROVISIONED", "campaign_domain": MACHINE.KEY_DOMAIN,
        "campaign_id": campaign_id, "key_id": "host-key-fixture",
        "algorithm": MACHINE.KEY_ALGORITHM,
        "public_key_base64": base64.b64encode(public).decode("ascii"),
        "public_key_sha256": hashlib.sha256(public).hexdigest(),
        "provisioning_receipt_sha256": "c" * 64, "canonical_sha256": "",
    }
    value["canonical_sha256"] = MACHINE.canonical_hash(value, "canonical_sha256")
    return value, public


def _proof(challenge_path: Path, challenge: dict[str, object], private: Ed25519PrivateKey):
    signature = private.sign(MACHINE._key_challenge_payload(challenge))
    value = {
        "schema": MACHINE.KEY_PROOF_SCHEMA, "schema_version": 1,
        "status": "EXTERNAL_SIGNATURE", "challenge_path": challenge_path.name,
        "challenge_file_sha256": hashlib.sha256(challenge_path.read_bytes()).hexdigest(),
        "challenge_canonical_sha256": challenge["canonical_sha256"],
        "challenge_payload_sha256": challenge["challenge_payload_sha256"],
        "key_id": challenge["key"]["key_id"], "algorithm": MACHINE.KEY_ALGORITHM,
        "signature_base64": base64.b64encode(signature).decode("ascii"),
        "signature_sha256": hashlib.sha256(signature).hexdigest(),
        "canonical_sha256": "",
    }
    value["canonical_sha256"] = MACHINE.canonical_hash(value, "canonical_sha256")
    return value


def _fixture(tmp_path: Path, *, campaign_id: str = "campaign-a",
             final_name: str = "machine-keyed.json", finalize: bool = True):
    tmp_path.mkdir(parents=True, exist_ok=True)
    private = Ed25519PrivateKey.generate()  # test-only, memory-only fixture key
    key_value, _ = _key_descriptor(private, campaign_id)
    key_path = tmp_path / "host-key.json"
    _write_input(key_path, key_value)
    snapshot = _snapshot()
    challenge_path = tmp_path / "challenge.json"
    MACHINE.prepare_key_challenge(
        challenge_path, key_descriptor=key_path, campaign_id=campaign_id,
        final_output=tmp_path / final_name,
        provider=lambda: snapshot)
    challenge = json.loads(challenge_path.read_text())
    proof_path = tmp_path / "proof.json"
    _write_input(proof_path, _proof(challenge_path, challenge, private))
    artifact_path = tmp_path / final_name
    result = None
    if finalize:
        result = MACHINE.finalize_keyed_artifact(
            artifact_path, challenge=challenge_path, proof=proof_path,
            captured_at="2027-01-01T00:00:00Z", status="LIVE_CAPTURED",
            provider=lambda: snapshot)
    return {
        "private": private, "snapshot": snapshot, "key_path": key_path,
        "challenge_path": challenge_path, "proof_path": proof_path,
        "artifact_path": artifact_path, "result": result,
    }


def test_dmi_less_two_phase_artifact_is_live_revalidated_and_offline_is_explicit(tmp_path):
    fixture = _fixture(tmp_path)
    artifact = fixture["artifact_path"]
    assert MACHINE.validate_keyed_file(artifact)["live_host_match"] is False
    live = MACHINE.validate_keyed_file(
        artifact, live=True, provider=lambda: fixture["snapshot"])
    assert live["live_host_match"] is True
    value = json.loads(artifact.read_text())
    assert value["method"] == MACHINE.KEY_METHOD
    assert value["status"] == "LIVE_CAPTURED"
    assert "0123456789abcdef0123456789abcdef" not in artifact.read_text()


def test_keyed_inputs_and_artifact_match_strict_schemas(tmp_path):
    _fixture(tmp_path)
    schema_names = {
        "machine-keyed.json": "apt_machine_identity_keyed.schema.json",
        "host-key.json": "apt_machine_host_public_key.schema.json",
        "challenge.json": "apt_machine_key_challenge.schema.json",
        "proof.json": "apt_machine_key_proof.schema.json",
    }
    for filename, schema_name in schema_names.items():
        value = json.loads((tmp_path / filename).read_text())
        schema = json.loads((R3 / schema_name).read_text())
        Draft202012Validator.check_schema(schema)
        Draft202012Validator(schema).validate(value)
    for filename in ("machine-keyed.json",):
        sidecar = json.loads((tmp_path / (filename + ".sha256.json")).read_text())
        schema = json.loads((R3 / "apt_machine_identity_keyed_sidecar.schema.json").read_text())
        Draft202012Validator(schema).validate(sidecar)
    for filename in ("host-key.json", "challenge.json", "proof.json"):
        sidecar = json.loads((tmp_path / (filename + ".sha256.json")).read_text())
        schema = json.loads((R3 / "apt_machine_key_input_sidecar.schema.json").read_text())
        Draft202012Validator(schema).validate(sidecar)


def test_existing_v1_still_fails_closed_with_only_machine_id(tmp_path):
    snapshot = _snapshot()
    with pytest.raises(MACHINE.MachineIdentityError, match="distinct stable identifiers"):
        MACHINE.build_artifact(campaign_id="campaign-a", snapshot=snapshot,
                               captured_at="2027-01-01T00:00:00Z")


def test_production_path_has_no_private_key_generation_or_signing():
    source = (R3 / "apt_machine_identity.py").read_text()
    assert "Ed25519PrivateKey" not in source
    assert "private_bytes" not in source
    assert ".sign(" not in source


def test_invalid_proof_and_cross_host_finalize_fail_closed(tmp_path):
    fixture = _fixture(tmp_path / "bad-proof", final_name="bad.json", finalize=False)
    proof = json.loads(fixture["proof_path"].read_text())
    proof["signature_base64"] = base64.b64encode(b"x" * 64).decode("ascii")
    proof["signature_sha256"] = hashlib.sha256(b"x" * 64).hexdigest()
    proof["canonical_sha256"] = MACHINE.canonical_hash(proof, "canonical_sha256")
    proof_path = tmp_path / "bad-proof" / "bad-proof.json"
    _write_input(proof_path, proof)
    with pytest.raises(MACHINE.MachineIdentityError, match="proof"):
        MACHINE.finalize_keyed_artifact(
            tmp_path / "bad-proof" / "bad.json", challenge=fixture["challenge_path"],
            proof=proof_path, captured_at="2027-01-01T00:00:00Z",
            provider=lambda: fixture["snapshot"])
    wrong_host = _fixture(tmp_path / "wrong-host", final_name="wrong-host.json", finalize=False)
    with pytest.raises(MACHINE.MachineIdentityError, match="current host projection"):
        MACHINE.finalize_keyed_artifact(
            tmp_path / "wrong-host" / "wrong-host.json", challenge=wrong_host["challenge_path"],
            proof=wrong_host["proof_path"], captured_at="2027-01-01T00:00:00Z",
            provider=lambda: _snapshot("fedcba9876543210fedcba9876543210"))


def test_campaign_replay_and_challenge_tamper_fail_closed(tmp_path):
    fixture = _fixture(tmp_path)
    challenge = json.loads(fixture["challenge_path"].read_text())
    challenge["campaign_id"] = "other-campaign"
    challenge["canonical_sha256"] = MACHINE.canonical_hash(challenge, "canonical_sha256")
    challenge_path = tmp_path / "replayed-challenge.json"
    _write_input(challenge_path, challenge)
    with pytest.raises(MACHINE.MachineIdentityError):
        MACHINE.finalize_keyed_artifact(
            tmp_path / "replayed.json", challenge=challenge_path,
            proof=fixture["proof_path"], captured_at="2027-01-01T00:00:00Z",
            provider=lambda: fixture["snapshot"])


@pytest.mark.parametrize("kind", ["symlink", "hardlink"])
def test_final_artifact_and_inputs_reject_link_replacement(tmp_path, kind):
    fixture = _fixture(tmp_path)
    target = tmp_path / "replacement.json"
    target.write_bytes(fixture["artifact_path"].read_bytes())
    target.chmod(0o444)
    fixture["artifact_path"].unlink()
    if kind == "symlink":
        fixture["artifact_path"].symlink_to(target.name)
    else:
        fixture["artifact_path"].hardlink_to(target)
    with pytest.raises(MACHINE.MachineIdentityError):
        MACHINE.validate_keyed_file(fixture["artifact_path"])


def test_challenge_is_fresh_and_nonce_is_not_reused(tmp_path):
    private = Ed25519PrivateKey.generate()
    key_value, _ = _key_descriptor(private)
    key_path = tmp_path / "host-key.json"
    _write_input(key_path, key_value)
    snapshot = _snapshot()
    first = tmp_path / "first.json"
    second = tmp_path / "second.json"
    MACHINE.prepare_key_challenge(first, key_descriptor=key_path,
                                  final_output=tmp_path / "first-final.json",
                                  campaign_id="campaign-a", provider=lambda: snapshot)
    MACHINE.prepare_key_challenge(second, key_descriptor=key_path,
                                  final_output=tmp_path / "second-final.json",
                                  campaign_id="campaign-a", provider=lambda: snapshot)
    first_nonce = json.loads(first.read_text())["challenge_nonce_base64"]
    second_nonce = json.loads(second.read_text())["challenge_nonce_base64"]
    assert first_nonce != second_nonce
    with pytest.raises(MACHINE.MachineIdentityError, match="fresh"):
        MACHINE.prepare_key_challenge(first, key_descriptor=key_path,
                                      final_output=tmp_path / "first-final.json",
                                      campaign_id="campaign-a", provider=lambda: snapshot)


def test_signed_challenge_cannot_be_replayed_to_a_different_output(tmp_path):
    fixture = _fixture(tmp_path)
    with pytest.raises(MACHINE.MachineIdentityError, match="bound by the challenge"):
        MACHINE.finalize_keyed_artifact(
            tmp_path / "replayed-output.json", challenge=fixture["challenge_path"],
            proof=fixture["proof_path"], captured_at="2027-01-01T00:00:00Z",
            provider=lambda: fixture["snapshot"])


def test_candidate_binding_tamper_is_not_fixed_by_rehashing_artifact(tmp_path):
    fixture = _fixture(tmp_path)
    artifact_path = fixture["artifact_path"]
    value = json.loads(artifact_path.read_text())
    value["candidate"]["file_sha256"] = "f" * 64
    value["identity_sha256"] = MACHINE.canonical_hash(
        MACHINE._key_identity_projection(value))
    value["canonical_sha256"] = MACHINE.canonical_hash(value, "canonical_sha256")
    artifact_path.chmod(0o600)
    artifact_path.write_bytes(MACHINE.canonical_bytes(value) + b"\n")
    artifact_path.chmod(0o444)
    with pytest.raises(MACHINE.MachineIdentityError, match="candidate binding"):
        MACHINE.validate_keyed_file(artifact_path)
