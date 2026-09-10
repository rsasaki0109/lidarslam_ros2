#!/usr/bin/env python3
"""Adversarial integration tests for the keyed machine authorization branch."""

from __future__ import annotations

import base64
import hashlib
import importlib.util
import json
from pathlib import Path

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from jsonschema import Draft202012Validator
import pytest


ROOT = Path(__file__).resolve().parents[2]
R3 = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


AUTH_TEST = _load(
    ROOT / "graph_based_slam/test/test_glim_clean_room_r3_apt_signature_authorization.py",
    "r3_keyed_authorization_base_fixture")
AUTH = AUTH_TEST.AUTH
MACHINE = AUTH_TEST.MACHINE
EXEC = AUTH_TEST.EXEC
HANDOFF = _load(
    R3 / "apt_release_signature_handoff.py", "r3_keyed_authorization_handoff")


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _write_immutable(path: Path, value: dict[str, object], *, sidecar_schema: str) -> bytes:
    data = MACHINE.canonical_bytes(value) + b"\n"
    path.write_bytes(data)
    path.chmod(0o444)
    sidecar = MACHINE._key_sidecar(path, data, value, schema=sidecar_schema)
    sidecar_path = path.with_name(path.name + ".sha256.json")
    sidecar_data = MACHINE.canonical_bytes(sidecar) + b"\n"
    sidecar_path.write_bytes(sidecar_data)
    sidecar_path.chmod(0o444)
    return data


def _snapshot():
    return {
        "raw_identifiers": {
            "machine_id": "0123456789abcdef0123456789abcdef",
            "dmi_uuid": "", "board_serial": "",
        },
        "public": {
            "architecture": "x86_64", "cpu_model": "keyed-fixture",
            "logical_cpu_count": 8, "memory_total_kb": 4096,
            "kernel_release": "keyed-kernel",
            "toolchain": {"python_version": "3.12.0",
                           "python_implementation": "CPython",
                           "python_compiler": "keyed-compiler"},
        },
    }


def _key_descriptor(path: Path, private: Ed25519PrivateKey, campaign_id: str) -> dict[str, object]:
    public = private.public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw)
    value: dict[str, object] = {
        "schema": MACHINE.KEY_DESCRIPTOR_SCHEMA, "schema_version": 1,
        "status": "EXTERNALLY_PROVISIONED", "campaign_domain": MACHINE.KEY_DOMAIN,
        "campaign_id": campaign_id, "key_id": "host-key-auth-fixture",
        "algorithm": MACHINE.KEY_ALGORITHM,
        "public_key_base64": base64.b64encode(public).decode("ascii"),
        "public_key_sha256": _sha(public), "provisioning_receipt_sha256": "c" * 64,
        "canonical_sha256": "",
    }
    value["canonical_sha256"] = MACHINE.canonical_hash(value, "canonical_sha256")
    _write_immutable(path, value, sidecar_schema=MACHINE.KEY_INPUT_SIDECAR_SCHEMA)
    return value


def _make_keyed_machine(base, tmp_path: Path, monkeypatch):
    fixture, plan, auth, auth_private, policy, candidate, _ = base
    snapshot = _snapshot()
    candidate_path = AUTH.CANDIDATE_MANIFEST_PATH
    monkeypatch.setattr(MACHINE, "CANDIDATE_PATH", candidate_path)
    monkeypatch.setattr(MACHINE, "CANDIDATE_RELATIVE", "candidate.json")
    monkeypatch.setattr(AUTH.MACHINE, "CANDIDATE_PATH", candidate_path)
    monkeypatch.setattr(AUTH.MACHINE, "CANDIDATE_RELATIVE", "candidate.json")
    monkeypatch.setattr(EXEC.AUTHORIZATION.MACHINE, "CANDIDATE_PATH", candidate_path)
    monkeypatch.setattr(EXEC.AUTHORIZATION.MACHINE, "CANDIDATE_RELATIVE", "candidate.json")
    production_candidate = json.loads(
        (R3 / "apt_allowlist_candidate.json").read_text(encoding="utf-8"))
    production_candidate["candidate_id"] = "test-candidate"
    production_candidate["revision"] = 1
    production_candidate["authorization_policy"] = {
        "schema": AUTH.POLICY_SCHEMA, "path": "policy.json",
        "file_sha256": "", "canonical_sha256": "",
        "status": "READY", "authorized_runtime": True,
    }
    production_candidate["canonical_sha256"] = AUTH.canonical_hash(
        production_candidate, "canonical_sha256")
    candidate_data = AUTH_TEST._write_json(candidate_path, production_candidate)
    assert json.loads(candidate_path.read_text())["candidate_id"] == "test-candidate"
    assert MACHINE._key_candidate_ref()["candidate_id"] == "test-candidate"
    private = Ed25519PrivateKey.generate()
    key_path = fixture["root"] / "host-key.json"
    _key_descriptor(key_path, private, "test-campaign")
    challenge_path = fixture["root"] / "machine-challenge.json"
    final_path = fixture["root"] / "machine-keyed.json"
    MACHINE.prepare_key_challenge(
        challenge_path, key_descriptor=key_path, campaign_id="test-campaign",
        final_output=final_path, provider=lambda: snapshot)
    challenge = json.loads(challenge_path.read_text(encoding="utf-8"))
    signature = private.sign(MACHINE._key_challenge_payload(challenge))
    proof = {
        "schema": MACHINE.KEY_PROOF_SCHEMA, "schema_version": 1,
        "status": "EXTERNAL_SIGNATURE", "challenge_path": challenge_path.name,
        "challenge_file_sha256": _sha(challenge_path.read_bytes()),
        "challenge_canonical_sha256": challenge["canonical_sha256"],
        "challenge_payload_sha256": challenge["challenge_payload_sha256"],
        "key_id": challenge["key"]["key_id"], "algorithm": MACHINE.KEY_ALGORITHM,
        "signature_base64": base64.b64encode(signature).decode("ascii"),
        "signature_sha256": _sha(signature), "canonical_sha256": "",
    }
    proof["canonical_sha256"] = MACHINE.canonical_hash(proof, "canonical_sha256")
    proof_path = fixture["root"] / "machine-proof.json"
    _write_immutable(proof_path, proof, sidecar_schema=MACHINE.KEY_INPUT_SIDECAR_SCHEMA)
    MACHINE.finalize_keyed_artifact(
        final_path, challenge=challenge_path, proof=proof_path,
        captured_at="2027-01-01T00:00:00Z", status="LIVE_CAPTURED",
        provider=lambda: snapshot)
    machine = json.loads(final_path.read_text(encoding="utf-8"))
    assert AUTH.MACHINE._key_candidate_ref() == machine["candidate"]
    machine_data = final_path.read_bytes()
    keyed_policy = {
        "status": "PRECOMMITTED", "method": machine["method"],
        "schema": machine["schema"], "path": "machine-keyed.json",
        "file_sha256": _sha(machine_data), "canonical_sha256": machine["canonical_sha256"],
        "campaign_domain": machine["campaign_domain"], "campaign_id": machine["campaign_id"],
        "identity_sha256": machine["identity_sha256"],
        "host_public_key": machine["host_public_key"], "challenge": machine["challenge"],
        "proof": machine["proof"],
    }
    policy["machine_identity"] = keyed_policy
    policy["canonical_sha256"] = AUTH.canonical_hash(policy, "canonical_sha256")
    policy_data = AUTH_TEST._write_json(AUTH.POLICY_PATH, policy)
    # A keyed artifact is bound to one immutable candidate snapshot.  The
    # checked-in candidate's policy back-reference is a separate release
    # reseal concern (otherwise policy->artifact->candidate forms a hash
    # cycle); this synthetic READY seam supplies the already reopened pair.
    candidate_data = candidate_path.read_bytes()
    auth["policy"] = {"path": "policy.json", "file_sha256": _sha(policy_data),
                       "canonical_sha256": policy["canonical_sha256"]}
    auth["candidate"] = {**auth["candidate"], "file_sha256": _sha(candidate_data),
                          "canonical_sha256": production_candidate["canonical_sha256"]}
    auth["machine_fingerprint"] = _sha(machine_data)
    auth["machine_identity"] = {key: value for key, value in keyed_policy.items()
                                 if key != "status"}
    AUTH_TEST._resign(auth, auth_private)
    candidate_value = json.loads(candidate_data.decode("utf-8"))
    monkeypatch.setattr(
        AUTH, "_load_fixed_policy",
        lambda: (policy, candidate_value, _sha(candidate_data), _sha(policy_data)))
    monkeypatch.setattr(AUTH.MACHINE, "_host_snapshot", lambda: snapshot)
    monkeypatch.setattr(EXEC.AUTHORIZATION.MACHINE, "_host_snapshot", lambda: snapshot)
    return fixture, plan, auth, snapshot, machine, keyed_policy


def test_keyed_method_is_verified_live_then_reopened_offline(tmp_path, monkeypatch):
    base = AUTH_TEST._fixture(tmp_path / "base", monkeypatch)
    fixture, plan, auth, snapshot, machine, keyed_policy = _make_keyed_machine(
        base, tmp_path, monkeypatch)
    result = AUTH.verify_authorization(
        auth, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"],
        executor_source_sha256=_sha(EXEC.EXECUTOR_PATH.read_bytes()),
        now=1_800_000_000, machine_provider=lambda: snapshot)
    assert result["machine_method"] == MACHINE.KEY_METHOD
    assert result["live_host_match"] is True
    assert result["machine_identity"]["host_public_key"] == machine["host_public_key"]
    offline = AUTH.verify_authorization(
        auth, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"],
        executor_source_sha256=_sha(EXEC.EXECUTOR_PATH.read_bytes()),
        now=1_800_000_000, machine_provider=lambda: snapshot,
        require_live_machine=False)
    assert offline["machine_method"] == MACHINE.KEY_METHOD
    assert offline["live_host_match"] is False
    assert keyed_policy["status"] == "PRECOMMITTED"


def test_keyed_branch_rejects_legacy_or_swapped_method_projection(tmp_path, monkeypatch):
    base = AUTH_TEST._fixture(tmp_path / "base", monkeypatch)
    fixture, plan, auth, snapshot, machine, _ = _make_keyed_machine(
        base, tmp_path, monkeypatch)
    auth["machine_identity"]["method"] = "LEGACY_V1"
    AUTH_TEST._resign(auth, base[3])
    with pytest.raises(AUTH.AuthorizationError):
        AUTH.verify_authorization(
            auth, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"],
            executor_source_sha256=_sha(EXEC.EXECUTOR_PATH.read_bytes()),
            now=1_800_000_000, machine_provider=lambda: snapshot)
    assert machine["method"] == MACHINE.KEY_METHOD


def test_keyed_runtime_receipt_reopens_with_offline_machine_validation(tmp_path, monkeypatch):
    base = AUTH_TEST._fixture(tmp_path / "base", monkeypatch)
    fixture, plan, auth, snapshot, _, _ = _make_keyed_machine(
        base, tmp_path, monkeypatch)
    policy = AUTH.POLICY_PATH.read_bytes()
    candidate = AUTH.CANDIDATE_MANIFEST_PATH.read_bytes()
    policy_value = json.loads(policy.decode("utf-8"))
    candidate_value = json.loads(candidate.decode("utf-8"))
    monkeypatch.setattr(
        EXEC.AUTHORIZATION, "_load_fixed_policy",
        lambda: (policy_value, candidate_value, _sha(candidate), _sha(policy)))

    class NoProcessRunner:
        def run(self, argv, **kwargs):
            del kwargs
            if argv[-1] == "--version":
                stdout = b"gpg (GnuPG) 2.4.synthetic\n"
            elif "--show-keys" in argv:
                stdout = (f"pub:-:rsa4096:1:{'A' * 16}:1700000000:0:::::::\n"
                          f"fpr:::::::::{'A' * 40}:\n"
                          f"sub:-:rsa4096:1:{'B' * 16}:1700000000:0:::::::\n"
                          f"fpr:::::::::{'B' * 40}:\n").encode()
            else:
                stdout = ("[GNUPG:] NEWSIG\n[GNUPG:] KEY_CONSIDERED " + "A" * 40 +
                          " 0\n[GNUPG:] GOODSIG " + "B" * 16 +
                          " Synthetic signer\n[GNUPG:] VALIDSIG " + "B" * 40 +
                          " 2026-01-01 1767225600 0 1 0 1 10 00 " + "A" * 40 +
                          "\n[GNUPG:] TRUST_ULTIMATE 0 pgp\n").encode()
            return {"returncode": 0, "timed_out": False, "signal": None,
                    "stdout": stdout, "stderr": b""}

    monkeypatch.setattr(EXEC, "SubprocessRunner", NoProcessRunner)
    monkeypatch.setattr(EXEC.time, "time", lambda: 1_800_000_000)
    values = iter((10, 25))
    result = EXEC.execute_signature_plan(
        plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"], runner=None, authorization=auth,
        clock=lambda: next(values))
    assert result["status"] == "PASS"
    receipt_root = Path(plan["scope"]["output_root"])
    receipt = json.loads((receipt_root / "apt-release-signature.executor.receipt.json").read_text())
    assert receipt["authorization_machine"]["artifact"]["method"] == MACHINE.KEY_METHOD
    assert receipt["authorization_machine"]["live_host_match"] is True
    executor_schema = json.loads(
        (R3 / "apt_release_signature_executor.schema.json").read_text())
    Draft202012Validator(executor_schema).validate(receipt)
    mixed = json.loads(json.dumps(receipt))
    mixed["authorization_machine"]["artifact"]["method"] = "LEGACY_V1"
    with pytest.raises(Exception):
        Draft202012Validator(executor_schema).validate(mixed)
    assert EXEC.validate_executor_receipt(
        receipt_root, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"])["status"] == "PASS"
    del snapshot


def test_keyed_authorization_schema_accepts_exact_union_and_rejects_downgrade():
    schema = json.loads((R3 / "apt_release_signature_authorization.schema.json").read_text())
    Draft202012Validator.check_schema(schema)
    machine = {"schema": MACHINE.SCHEMA, "path": "machine.json",
               "file_sha256": "a" * 64, "canonical_sha256": "b" * 64,
               "campaign_domain": MACHINE.DOMAIN, "campaign_id": "campaign-a",
               "identity_sha256": "c" * 64}
    # The branch schema itself is exact; a keyed tag cannot be mixed with the
    # legacy projection, even when all hashes look syntactically valid.
    keyed = dict(machine)
    keyed["method"] = MACHINE.KEY_METHOD
    with pytest.raises(Exception):
        Draft202012Validator(schema).validate({"machine_identity": keyed})


def test_keyed_live_artifact_enters_handoff_as_offline_only(tmp_path, monkeypatch):
    base = AUTH_TEST._fixture(tmp_path / "base", monkeypatch)
    fixture, _, _, _, _, _ = _make_keyed_machine(base, tmp_path, monkeypatch)
    for machine_module in (HANDOFF.MACHINE, HANDOFF.AUTH.MACHINE):
        monkeypatch.setattr(machine_module, "CANDIDATE_PATH",
                            AUTH.CANDIDATE_MANIFEST_PATH)
        monkeypatch.setattr(machine_module, "CANDIDATE_RELATIVE", "candidate.json")
    descriptor = HANDOFF._machine_descriptor(
        fixture["root"] / "machine-keyed.json", fixture["root"], "test-campaign")
    assert descriptor["method"] == MACHINE.KEY_METHOD
    assert descriptor["validation"] == "OFFLINE_ARTIFACT_ONLY"
