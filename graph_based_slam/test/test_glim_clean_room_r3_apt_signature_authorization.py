#!/usr/bin/env python3
"""Adversarial tests for the r3 Ed25519 host-authorization boundary."""

from __future__ import annotations

import base64
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

from jsonschema import Draft202012Validator
import pytest
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey


ROOT = Path(__file__).resolve().parents[2]
R3 = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


AUTH = _load(R3 / "apt_release_signature_authorization.py", "r3_host_authorization_test")
MACHINE = _load(R3 / "apt_machine_identity.py", "r3_machine_identity_test")
EXEC_TEST = _load(ROOT / "graph_based_slam/test/test_glim_clean_room_r3_apt_signature_executor.py",
                  "r3_host_authorization_executor_fixture")
EXEC = EXEC_TEST.EXEC


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _write_json(path: Path, value: object) -> bytes:
    data = AUTH.canonical_bytes(value) + b"\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    path.chmod(0o600)
    return data


def _canonical_with_self(value: dict[str, object]) -> dict[str, object]:
    result = dict(value)
    result["canonical_sha256"] = AUTH.canonical_hash(result, "canonical_sha256")
    return result


def _fixture(tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
             authorization_now: int | None = None):
    fixture = EXEC_TEST._fixture(tmp_path / "inputs")
    plan = EXEC_TEST._plan(fixture, tmp_path / "output")
    policy_root = tmp_path / "policy"
    policy_root.mkdir()
    monkeypatch.setattr(AUTH, "ROOT", policy_root)
    monkeypatch.setattr(AUTH, "R3", policy_root)
    candidate_path = policy_root / "candidate.json"
    policy_path = policy_root / "policy.json"
    machine_snapshot = {
        "raw_identifiers": {
            "machine_id": "0123456789abcdef0123456789abcdef",
            "dmi_uuid": "01234567-89ab-cdef-0123-456789abcdef",
            "board_serial": "BOARD-SERIAL-1234",
        },
        "public": {
            "architecture": "x86_64", "cpu_model": "synthetic-cpu",
            "logical_cpu_count": 8, "memory_total_kb": 1024,
            "kernel_release": "synthetic-kernel",
            "toolchain": {"python_version": "3.12.0",
                           "python_implementation": "CPython",
                           "python_compiler": "synthetic-compiler"},
        },
    }
    machine = MACHINE.build_artifact(
        campaign_id="test-campaign",
        captured_at="2027-01-01T00:00:00Z",
        snapshot=machine_snapshot, status="LIVE_CAPTURED")
    machine_path = fixture["root"] / "machine.json"
    machine_data = _write_json(machine_path, machine)
    machine_sidecar = MACHINE._sidecar(machine_path, machine_data, machine)
    sidecar_path = machine_path.with_name(machine_path.name + ".sha256.json")
    _write_json(sidecar_path, machine_sidecar)
    machine_path.chmod(0o444)
    sidecar_path.chmod(0o444)
    private = Ed25519PrivateKey.generate()
    public = private.public_key().public_bytes_raw()
    now = 1_800_000_000 if authorization_now is None else authorization_now
    policy = {
        "schema": AUTH.POLICY_SCHEMA, "schema_version": 1, "status": "READY",
        "candidate_manifest": {"path": "candidate.json", "candidate_id": "test-candidate",
                               "revision": 1},
        "backend": {**AUTH.BACKEND, "version": "46.0.5",
                     "implementation_sha256": _sha(AUTH.IMPLEMENTATION_PATH.read_bytes())},
        "machine_identity": {"status": "PRECOMMITTED", "schema": MACHINE.SCHEMA,
                              "path": "machine.json", "file_sha256": _sha(machine_data),
                              "canonical_sha256": machine["canonical_sha256"],
                              "campaign_domain": MACHINE.DOMAIN,
                              "campaign_id": machine["campaign_id"],
                              "identity_sha256": machine["identity_sha256"]},
        "authorized_keys": [{"key_id": "custodian-test", "algorithm": "Ed25519",
                              "status": "ACTIVE", "public_key_base64": base64.b64encode(public).decode(),
                              "public_key_sha256": _sha(public), "not_before": now - 100,
                              "not_after": now + 100}],
        "canonical_sha256": "",
    }
    policy = _canonical_with_self(policy)
    policy_data = _write_json(policy_path, policy)
    candidate = {
        "candidate_id": "test-candidate", "revision": 1,
        "authorization_policy": {"schema": AUTH.POLICY_SCHEMA, "path": "policy.json",
                                  "file_sha256": _sha(policy_data),
                                  "canonical_sha256": policy["canonical_sha256"],
                                  "status": "READY", "authorized_runtime": True},
        "canonical_sha256": "",
    }
    candidate = _canonical_with_self(candidate)
    candidate_data = _write_json(candidate_path, candidate)
    monkeypatch.setattr(AUTH, "CANDIDATE_MANIFEST_PATH", candidate_path)
    monkeypatch.setattr(AUTH, "POLICY_PATH", policy_path)
    # The executor loads a second module instance.  Bind both instances to
    # the same synthetic fixed paths so the runtime path exercises the exact
    # production reopen boundary instead of a test-only verifier shortcut.
    monkeypatch.setattr(EXEC.AUTHORIZATION, "ROOT", policy_root)
    monkeypatch.setattr(EXEC.AUTHORIZATION, "R3", policy_root)
    monkeypatch.setattr(EXEC.AUTHORIZATION, "CANDIDATE_MANIFEST_PATH", candidate_path)
    monkeypatch.setattr(EXEC.AUTHORIZATION, "POLICY_PATH", policy_path)
    monkeypatch.setattr(AUTH.MACHINE, "_host_snapshot", lambda: machine_snapshot)
    monkeypatch.setattr(EXEC.AUTHORIZATION.MACHINE, "_host_snapshot", lambda: machine_snapshot)
    context = AUTH._expected_context(
        plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"],
        executor_source_sha256=_sha(EXEC.EXECUTOR_PATH.read_bytes()))
    auth = {
        "schema": AUTH.AUTH_SCHEMA, "schema_version": 1,
        "status": "HOST_RUNTIME_AUTHORIZATION",
        "candidate": {"path": "candidate.json", "file_sha256": _sha(candidate_data),
                      "canonical_sha256": candidate["canonical_sha256"],
                      "candidate_id": "test-candidate", "revision": 1},
        "policy": {"path": "policy.json", "file_sha256": _sha(policy_data),
                   "canonical_sha256": policy["canonical_sha256"]},
        "plan": {"sha256": context["plan_sha256"],
                 "canonical_sha256": context["plan_canonical_sha256"]},
        "executor": {"source_sha256": context["executor_source_sha256"]},
        "trust_root": {"path": plan["trust_root"]["path"],
                       "file_sha256": plan["trust_root"]["file_sha256"],
                       "canonical_sha256": plan["trust_root"]["canonical_sha256"],
                       "keyring_path": plan["trust_root"]["keyring_path"],
                       "keyring_sha256": plan["trust_root"]["keyring_sha256"],
                       "key_id": context["trust"]["key_id"],
                       "fingerprint": context["trust"]["fingerprint"]},
        "repository": dict(fixture["repository"]), "binding": context["binding"],
        "deb_bindings": fixture["deb_bindings"],
        "deb_bindings_sha256": AUTH.canonical_hash(fixture["deb_bindings"]),
        "tool": {"path": context["executable"], "sha256": context["executable_sha256"],
                 "version": context["trust"]["verifier"]["version"]},
        "scope": {"root_inputs": str(fixture["root"]),
                  "homedir": plan["scope"]["homedir"],
                  "output_root": plan["scope"]["output_root"]},
        "machine_fingerprint": _sha(machine_data),
        "machine_identity": {"schema": MACHINE.SCHEMA, "path": "machine.json",
                              "file_sha256": _sha(machine_data),
                              "canonical_sha256": machine["canonical_sha256"],
                              "campaign_domain": MACHINE.DOMAIN,
                              "campaign_id": machine["campaign_id"],
                              "identity_sha256": machine["identity_sha256"]},
        "issued_at": now - 1, "expires_at": now + 50,
        "nonce": "N" * 64, "network_used": False, "shell": False,
        "timeout_seconds": AUTH.TIMEOUT_SECONDS, "resource_limits": dict(AUTH.RESOURCE_LIMITS),
        "workflow": context["workflow"], "phases": context["phases"],
        "signature": {"algorithm": AUTH.ALGORITHM, "key_id": "custodian-test",
                      "public_key_sha256": _sha(public), "payload_sha256": "",
                      "signature_base64": "", "backend": policy["backend"]},
    }
    auth["signature"]["payload_sha256"] = AUTH.authorization_payload_sha256(auth)
    signed = private.sign(AUTH.authorization_payload(auth))
    auth["signature"]["signature_base64"] = base64.b64encode(signed).decode()
    return fixture, plan, auth, private, policy, candidate, machine_data


def _resign(auth: dict[str, object], private: Ed25519PrivateKey) -> None:
    signature = auth["signature"]
    assert isinstance(signature, dict)
    signature["payload_sha256"] = AUTH.authorization_payload_sha256(auth)
    signature["signature_base64"] = base64.b64encode(
        private.sign(AUTH.authorization_payload(auth))).decode()


def test_checked_in_policy_is_not_ready_and_cannot_authorize():
    result = AUTH.verify_fixed_policy_only()
    assert result["status"] == "NOT_READY"
    assert result["benchmark_eligible"] is False


def test_valid_authorization_binds_all_runtime_inputs(tmp_path, monkeypatch):
    fixture, plan, auth, _, _, _, _ = _fixture(tmp_path, monkeypatch)
    result = AUTH.verify_authorization(
        auth, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"],
        executor_source_sha256=_sha(EXEC.EXECUTOR_PATH.read_bytes()), now=1_800_000_000)
    assert result["status"] == "PASS"
    assert result["verified"] is True


def test_authorization_requires_live_machine_match_before_runtime(tmp_path, monkeypatch):
    fixture, plan, auth, _, _, _, _ = _fixture(tmp_path, monkeypatch)
    original = AUTH.MACHINE._host_snapshot
    monkeypatch.setattr(AUTH.MACHINE, "_host_snapshot", lambda: {
        **original(), "public": {**original()["public"], "cpu_model": "different-host"}})
    with pytest.raises(AUTH.AuthorizationError, match="does not match current host"):
        AUTH.verify_authorization(
            auth, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"],
            executor_source_sha256=_sha(EXEC.EXECUTOR_PATH.read_bytes()), now=1_800_000_000)


@pytest.mark.parametrize("mutation", [
    "signature", "key_id", "plan", "repository", "machine", "source", "expiry", "backend"])
def test_authorization_metadata_or_signature_tamper_fails_closed(tmp_path, monkeypatch, mutation):
    fixture, plan, auth, private, policy, _, machine_data = _fixture(tmp_path, monkeypatch)
    if mutation == "signature":
        auth["signature"]["signature_base64"] = base64.b64encode(b"x" * 64).decode()
    elif mutation == "key_id":
        auth["signature"]["key_id"] = "other"
    elif mutation == "plan":
        auth["plan"]["canonical_sha256"] = "f" * 64
    elif mutation == "repository":
        auth["repository"]["url"] = "https://other.invalid/jazzy"
    elif mutation == "machine":
        auth["machine_fingerprint"] = "a" * 64
    elif mutation == "source":
        auth["executor"]["source_sha256"] = "b" * 64
    elif mutation == "expiry":
        auth["expires_at"] = 1_800_000_000
    else:
        auth["signature"]["backend"] = {**auth["signature"]["backend"], "version": "0.0.0"}
    # A modified payload must be re-signed to prove metadata binding, except
    # for an invalid detached signature; both paths must fail closed.
    if mutation not in {"signature", "key_id", "backend"}:
        auth["signature"]["payload_sha256"] = AUTH.authorization_payload_sha256(auth)
        auth["signature"]["signature_base64"] = base64.b64encode(
            private.sign(AUTH.authorization_payload(auth))).decode()
    with pytest.raises(AUTH.AuthorizationError):
        AUTH.verify_authorization(
            auth, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"],
            executor_source_sha256=_sha(EXEC.EXECUTOR_PATH.read_bytes()), now=1_800_000_000)


def test_revoked_or_forged_key_and_policy_replay_fail_closed(tmp_path, monkeypatch):
    fixture, plan, auth, private, policy, candidate, _ = _fixture(tmp_path, monkeypatch)
    policy["authorized_keys"][0]["status"] = "REVOKED"
    policy["canonical_sha256"] = AUTH.canonical_hash(policy, "canonical_sha256")
    policy_path = AUTH.POLICY_PATH
    policy_data = _write_json(policy_path, policy)
    candidate["authorization_policy"]["file_sha256"] = _sha(policy_data)
    candidate["authorization_policy"]["canonical_sha256"] = policy["canonical_sha256"]
    candidate["canonical_sha256"] = AUTH.canonical_hash(candidate, "canonical_sha256")
    _write_json(AUTH.CANDIDATE_MANIFEST_PATH, candidate)
    with pytest.raises(AUTH.AuthorizationError):
        AUTH.verify_authorization(
            auth, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"],
            executor_source_sha256=_sha(EXEC.EXECUTOR_PATH.read_bytes()), now=1_800_000_000)


def test_nonce_claim_is_one_shot_before_output_root(tmp_path, monkeypatch):
    fixture, plan, auth, _, _, _, _ = _fixture(tmp_path, monkeypatch)
    output = tmp_path / "new-output"
    claim = EXEC._claim_authorization_nonce(auth, output, auth["issued_at"] + 1)
    assert not output.exists()
    assert Path(claim["path"]).is_file()
    with pytest.raises(EXEC.ExecutorError, match="already consumed"):
        EXEC._claim_authorization_nonce(auth, output, auth["issued_at"] + 1)


def test_subprocess_execution_requires_authorization_before_output_creation(tmp_path):
    fixture = EXEC_TEST._fixture(tmp_path / "inputs")
    output = tmp_path / "output"
    plan = EXEC_TEST._plan(fixture, output)
    with pytest.raises(EXEC.ExecutorError, match="independent host authorization"):
        EXEC.execute_signature_plan(
            plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"], runner=None)
    assert not output.exists()


def test_fixture_runner_cannot_bypass_authorization_or_claim_output(tmp_path):
    fixture = EXEC_TEST._fixture(tmp_path / "inputs")
    output = tmp_path / "output"
    plan = EXEC_TEST._plan(fixture, output)
    with pytest.raises(EXEC.ExecutorError, match="fixture runners"):
        EXEC.execute_signature_plan(
            plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"],
            runner=EXEC_TEST._FakeRunner("A" * 40, "B" * 40), authorization={})
    assert not output.exists()


@pytest.mark.parametrize("mutation", [
    "not_yet_valid", "scope_output", "scope_homedir", "tool_sha", "tool_version",
    "deb", "release", "packages", "candidate_path", "policy_path",
])
def test_authorization_rejects_scope_tool_and_repository_drift(tmp_path, monkeypatch, mutation):
    fixture, plan, auth, private, _, _, _ = _fixture(tmp_path, monkeypatch)
    verification_time = 1_800_000_000
    if mutation == "not_yet_valid":
        auth["issued_at"] = verification_time + 1
    elif mutation == "scope_output":
        auth["scope"]["output_root"] = str(tmp_path / "other-output")
    elif mutation == "scope_homedir":
        auth["scope"]["homedir"] = str(tmp_path / "other-gpg")
    elif mutation == "tool_sha":
        auth["tool"]["sha256"] = "a" * 64
    elif mutation == "tool_version":
        auth["tool"]["version"] = "0.0.synthetic"
    elif mutation == "deb":
        auth["deb_bindings"][0]["path"] = "other.deb"
    elif mutation == "release":
        auth["repository"]["release_sha256"] = "a" * 64
    elif mutation == "packages":
        auth["repository"]["packages_sha256"] = "b" * 64
    elif mutation == "candidate_path":
        auth["candidate"]["path"] = "policy.json"
    else:
        auth["policy"]["path"] = "candidate.json"
    _resign(auth, private)
    with pytest.raises(AUTH.AuthorizationError):
        AUTH.verify_authorization(
            auth, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"],
            executor_source_sha256=_sha(EXEC.EXECUTOR_PATH.read_bytes()), now=verification_time)


@pytest.mark.parametrize("kind", ["symlink", "hardlink"])
def test_machine_identity_artifact_must_be_single_link_nonsymlink(tmp_path, monkeypatch, kind):
    fixture, plan, auth, private, _, _, machine_data = _fixture(tmp_path, monkeypatch)
    machine_path = fixture["root"] / "machine.json"
    if kind == "symlink":
        replacement = fixture["root"] / "machine-target.json"
        replacement.write_bytes(machine_data)
        machine_path.unlink()
        machine_path.symlink_to(replacement.name)
    else:
        replacement = fixture["root"] / "machine-target.json"
        replacement.write_bytes(machine_data)
        machine_path.unlink()
        os.link(replacement, machine_path)
    _resign(auth, private)
    with pytest.raises(AUTH.AuthorizationError):
        AUTH.verify_authorization(
            auth, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"],
            executor_source_sha256=_sha(EXEC.EXECUTOR_PATH.read_bytes()), now=1_800_000_000)


def test_alternate_self_signed_policy_cannot_replace_fixed_policy(tmp_path, monkeypatch):
    fixture, plan, auth, private, policy, _, _ = _fixture(tmp_path, monkeypatch)
    alternate = dict(policy)
    alternate["authorized_keys"] = []
    alternate["canonical_sha256"] = AUTH.canonical_hash(alternate, "canonical_sha256")
    _write_json(AUTH.POLICY_PATH, alternate)
    _resign(auth, private)
    with pytest.raises(AUTH.AuthorizationError):
        AUTH.verify_authorization(
            auth, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"],
            executor_source_sha256=_sha(EXEC.EXECUTOR_PATH.read_bytes()), now=1_800_000_000)


def test_explicit_subprocess_runner_with_invalid_authorization_rejected_pre_output(
        tmp_path, monkeypatch):
    fixture, plan, _, _, _, _, _ = _fixture(tmp_path, monkeypatch)
    output = tmp_path / "runtime-output"
    with pytest.raises(EXEC.ExecutorError, match="host runtime authorization"):
        EXEC.execute_signature_plan(
            plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"], runner=EXEC.SubprocessRunner(), authorization={})
    assert not output.exists()


def test_expired_authorization_is_rejected_before_first_runtime_output(tmp_path, monkeypatch):
    verification_time = int(time.time())
    fixture, plan, auth, private, _, _, _ = _fixture(
        tmp_path, monkeypatch, authorization_now=verification_time)
    auth["issued_at"] = verification_time - 10
    auth["expires_at"] = verification_time - 1
    _resign(auth, private)
    output = Path(plan["scope"]["output_root"])
    with pytest.raises(EXEC.ExecutorError, match="outside its signed validity window"):
        EXEC.execute_signature_plan(
            plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"], runner=None, authorization=auth)
    assert not output.exists()


def test_authorized_runtime_receipt_reopens_and_promotion_gate_accepts_synthetic_runner(
        tmp_path, monkeypatch):
    verification_time = int(time.time())
    fixture, plan, auth, _, _, _, _ = _fixture(
        tmp_path, monkeypatch, authorization_now=verification_time)

    class NoProcessAuthorizedRunner:
        def run(self, argv, **kwargs):
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

    monkeypatch.setattr(EXEC, "SubprocessRunner", NoProcessAuthorizedRunner)
    output = Path(plan["scope"]["output_root"])
    values = iter((10, 25))
    result = EXEC.execute_signature_plan(
        plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"], runner=None, authorization=auth,
        clock=lambda: next(values))
    assert result["status"] == "PASS"
    assert result["execution_mode"] == "HOST_AUTHORIZED_SUBPROCESS"
    assert result["signature_runtime"] == "RUNTIME_VERIFIED"
    receipt = json.loads((output / "apt-release-signature.executor.receipt.json").read_text())
    Draft202012Validator(json.loads(
        (R3 / "apt_release_signature_executor.schema.json").read_text())).validate(receipt)
    assert receipt["benchmark_eligible"] is True
    assert EXEC.validate_executor_receipt(
        output, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"])["status"] == "PASS"
    # Reopening uses the sealed execution time, so a later wall-clock expiry
    # does not make an already-authorized receipt unreproducible.
    monkeypatch.setattr(EXEC.time, "time", lambda: verification_time + 10_000)
    assert EXEC.validate_executor_receipt(
        output, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"])["status"] == "PASS"
    promotion = _load(R3 / "apt_signature_promotion.py", "r3_authorized_promotion_test")
    assert promotion._validate_promotable_receipt(output)["promotion"] == "HOST_AUTHORIZED_RUNTIME"
    receipt["authorization_verification_time"] += 1
    receipt["canonical_sha256"] = EXEC.SIG.canonical_hash(receipt, "canonical_sha256")
    receipt_path = output / "apt-release-signature.executor.receipt.json"
    sidecar = output / "apt-release-signature.executor.receipt.json.sha256"
    receipt_path.chmod(0o600)
    sidecar.chmod(0o600)
    data = EXEC.SIG.canonical_bytes(receipt) + b"\n"
    receipt_path.write_bytes(data)
    sidecar.write_text(f"{_sha(data)}  {receipt_path.name}\n")
    with pytest.raises(EXEC.ExecutorError):
        EXEC.validate_executor_receipt(
            output, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"])


def test_authorized_runtime_failure_is_not_eligible_or_promotable(tmp_path, monkeypatch):
    verification_time = int(time.time())
    fixture, plan, auth, _, _, _, _ = _fixture(
        tmp_path, monkeypatch, authorization_now=verification_time)

    class NoProcessFailingRunner:
        def __init__(self):
            self.delegate = EXEC_TEST._FakeRunner("A" * 40, "B" * 40)

        def run(self, argv, **kwargs):
            result = self.delegate.run(argv, **kwargs)
            if argv[-1] != "--version":
                result["returncode"] = 1
            return result

    monkeypatch.setattr(EXEC, "SubprocessRunner", NoProcessFailingRunner)
    output = Path(plan["scope"]["output_root"])
    values = iter((10, 25))
    result = EXEC.execute_signature_plan(
        plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"], runner=None, authorization=auth,
        clock=lambda: next(values))
    assert result["status"] == "PASS"
    assert result["outcome"] == "FAIL"
    receipt = json.loads((output / "apt-release-signature.executor.receipt.json").read_text())
    Draft202012Validator(json.loads(
        (R3 / "apt_release_signature_executor.schema.json").read_text())).validate(receipt)
    assert receipt["benchmark_eligible"] is False
    assert receipt["promotion"] == "FORBIDDEN"
    EXEC.validate_executor_receipt(
        output, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"])
    promotion = _load(R3 / "apt_signature_promotion.py", "r3_runtime_failure_promotion_test")
    with pytest.raises(promotion.SignaturePromotionError):
        promotion._validate_promotable_receipt(output)
