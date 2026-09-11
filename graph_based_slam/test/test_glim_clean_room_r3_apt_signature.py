#!/usr/bin/env python3
"""Synthetic tests for the non-promoting r3 OpenPGP verification contract."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

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


SIG = _load(R3 / "apt_release_signature.py", "r3_apt_signature_test")
PLAN = _load(ROOT / "scripts/plan_glim_clean_room_r3_apt_signature.py",
             "r3_apt_signature_plan_test")


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and not path.is_symlink():
        path.chmod(0o600)
    path.write_bytes(data)


def _json(path: Path, value: object) -> None:
    _write(path, SIG.canonical_bytes(value) + b"\n")


def _fixture(tmp_path: Path):
    fingerprint = "A" * 40
    sub_fingerprint = "B" * 40
    key_id = fingerprint[-16:]
    sub_key_id = sub_fingerprint[-16:]
    keyring = b"synthetic-keyring-bytes\n"
    keyring_path = tmp_path / "keyring.gpg"
    _write(keyring_path, keyring)
    trust = {
        "schema": SIG.TRUST_SCHEMA, "schema_version": 1, "status": "PRECOMMITTED",
        "key_id": key_id, "fingerprint": fingerprint, "keyring_path": "keyring.gpg",
        "keyring_bytes": len(keyring), "keyring_sha256": _sha(keyring),
        "verification_time": 1800000000,
        "validity": {"primary_fingerprint": fingerprint, "created_at": 1700000000,
                      "expires_at": 0, "revoked": False,
                      "subkeys": [{"fingerprint": sub_fingerprint, "created_at": 1700000000,
                                   "expires_at": 0, "revoked": False}]},
        "verifier": {"name": "gpg", "version": "2.4.synthetic", "binary_sha256": "c" * 64,
                     "binary_path": "/usr/bin/gpg", "status_protocol": "gpg-status-fd-v1"},
    }
    trust["canonical_sha256"] = SIG.canonical_hash(trust)
    trust_path = tmp_path / "trust-root.json"
    _json(trust_path, trust)

    package = b"Package: demo\nVersion: 1\nArchitecture: amd64\nFilename: pool/demo.deb\nSize: 9\nSHA256: " + \
        _sha(b"demo-deb\n").encode() + b"\n\n"
    packages_path = tmp_path / "Packages"
    _write(packages_path, package)
    release = (b"Suite: jazzy\nSHA256:\n " + _sha(package).encode() + b" " +
               str(len(package)).encode() + b" main/binary-amd64/Packages\n")
    release_path = tmp_path / "InRelease"
    _write(release_path, release)
    deb_path = tmp_path / "demo.deb"
    _write(deb_path, b"demo-deb\n")
    status = (f"[GNUPG:] NEWSIG\n[GNUPG:] KEY_CONSIDERED {fingerprint} 0\n"
              f"[GNUPG:] GOODSIG {sub_key_id} Synthetic signer\n"
              f"[GNUPG:] VALIDSIG {sub_fingerprint} 2026-01-01 1767225600 0 1 0 1 10 00 {fingerprint}\n"
              "[GNUPG:] TRUST_ULTIMATE 0 pgp\n").encode()
    status_path = tmp_path / "synthetic-gpg-status.log"
    _write(status_path, status)
    inventory = (f"pub:-:rsa4096:1:{key_id}:1700000000:0:::::::\n"
                 f"fpr:::::::::{fingerprint}:\n"
                 f"sub:-:rsa4096:1:{sub_key_id}:1700000000:0:::::::\n"
                 f"fpr:::::::::{sub_fingerprint}:\n").encode()
    inventory_path = tmp_path / "synthetic-gpg-inventory.log"
    _write(inventory_path, inventory)
    repository = {
        "url": "https://apt.example.invalid/jazzy",
        "release_url": "https://apt.example.invalid/jazzy/InRelease",
        "release_path": "var/lib/apt/lists/apt.example.invalid_jazzy_InRelease",
        "release_sha256": _sha(release),
        "packages_url": "https://apt.example.invalid/jazzy/main/binary-amd64/Packages",
        "packages_path": "var/lib/apt/lists/apt.example.invalid_jazzy_main_binary-amd64_Packages",
        "packages_sha256": _sha(package),
    }
    return {"root": tmp_path, "trust": trust_path, "release": release_path,
            "packages": packages_path, "deb": deb_path, "status": status_path,
            "inventory": inventory_path, "repository": repository,
            "deb_bindings": [{"name": "demo", "version": "1", "architecture": "amd64",
                              "path": "demo.deb"}], "homedir": tmp_path / "fresh-gpg"}


def _seal(fixture, output: Path):
    command = SIG.build_verify_argv(
        homedir=fixture["homedir"], keyring=fixture["root"] / "keyring.gpg",
        release=fixture["release"], release_kind="INRELEASE", executable="/usr/bin/gpg")
    phases = [{"ordinal": index, "phase": phase,
               "status": "INJECTED_VERIFIER" if phase == "signature_verify" else
                         "SEALED_FIXTURE" if phase == "seal_receipt" else
                         "NOT_RUN" if phase == "cleanup" else "INJECTED_REOPEN"}
              for index, phase in enumerate(SIG.PHASES)]
    return SIG.seal_signature_receipt(
        root=fixture["root"], trust_root_path=fixture["trust"],
        release_path=fixture["release"], packages_path=fixture["packages"],
        repository=fixture["repository"], deb_bindings=fixture["deb_bindings"],
        homedir=fixture["homedir"], release_kind="INRELEASE", signature_path=None,
        verifier_log_path=fixture["status"], key_inventory_path=fixture["inventory"],
        command=command, verifier_exit_status=0, phases=phases, output_root=output)


def test_synthetic_signature_receipt_and_plan_are_nonpromoting(tmp_path):
    fixture = _fixture(tmp_path)
    result = _seal(fixture, tmp_path / "receipt")
    assert result["status"] == "PASS"
    receipt = json.loads((tmp_path / "receipt/apt-release-signature.receipt.json").read_text())
    Draft202012Validator(json.loads((R3 / "apt_release_signature.schema.json").read_text())).validate(receipt)
    plan = PLAN.build_plan(
        root=fixture["root"], trust_root_path=fixture["trust"], release_path=fixture["release"],
        packages_path=fixture["packages"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"], homedir=fixture["homedir"],
        release_kind="INRELEASE", signature_path=None, output_root=tmp_path / "plan-output")
    assert PLAN.validate_plan(plan, root=fixture["root"], repository=fixture["repository"],
                              deb_bindings=fixture["deb_bindings"])["status"] == "PASS"
    Draft202012Validator(json.loads((R3 / "apt_release_signature_plan.schema.json").read_text())).validate(plan)
    assert [item["phase"] for item in plan["workflow"]] == list(PLAN.HOST_PHASES)


def test_status_primary_fingerprint_uses_field_ten_and_rejects_unknown(tmp_path):
    fixture = _fixture(tmp_path)
    trust = SIG.validate_trust_root(fixture["root"], fixture["trust"])
    good = fixture["status"].read_bytes()
    assert SIG.validate_status_output(good, trust)["status"] == "PASS"
    bad = good.replace(b"00 AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", b"00 CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC")
    with pytest.raises(SIG.SignatureError):
        SIG.validate_status_output(bad, trust)
    with pytest.raises(SIG.SignatureError):
        SIG.validate_status_output(good + b"[GNUPG:] UNKNOWN x\n", trust)


def test_signing_subkey_and_inventory_are_bound(tmp_path):
    fixture = _fixture(tmp_path)
    trust = SIG.validate_trust_root(fixture["root"], fixture["trust"])
    inventory = SIG.validate_key_inventory_output(fixture["inventory"].read_bytes(), trust)
    assert inventory["primary"]["fingerprint"] == trust["fingerprint"]
    assert inventory["subkeys"][0]["fingerprint"] == "B" * 40
    assert SIG.build_key_inventory_argv(homedir=fixture["homedir"],
                                        keyring=fixture["root"] / "keyring.gpg",
                                        executable="/usr/bin/gpg")[-2] == "--show-keys"


@pytest.mark.parametrize("mutation", ["command", "inventory", "status", "release", "keyring"])
def test_receipt_tamper_or_drift_is_rejected(tmp_path, mutation):
    fixture = _fixture(tmp_path)
    output = tmp_path / "receipt"
    _seal(fixture, output)
    receipt_path = output / "apt-release-signature.receipt.json"
    receipt_path.chmod(0o600)
    receipt = json.loads(receipt_path.read_text())
    if mutation == "command":
        receipt["verifier"]["command"][-1] = "/tmp/other"
    elif mutation == "inventory":
        fixture["inventory"].write_bytes(fixture["inventory"].read_bytes().replace(b"1700000000", b"1700000001"))
    elif mutation == "status":
        fixture["status"].write_bytes(fixture["status"].read_bytes().replace(b"TRUST_ULTIMATE", b"REVKEYSIG"))
    elif mutation == "release":
        fixture["release"].write_bytes(fixture["release"].read_bytes() + b"tamper")
    else:
        fixture["root"].joinpath("keyring.gpg").write_bytes(b"changed-keyring\n")
    receipt["canonical_sha256"] = SIG.canonical_hash(receipt, "canonical_sha256")
    data = SIG.canonical_bytes(receipt) + b"\n"
    receipt_path.write_bytes(data)
    sidecar_path = output / "apt-release-signature.receipt.json.sha256"
    sidecar_path.chmod(0o600)
    sidecar_path.write_text(
        _sha(data) + "  apt-release-signature.receipt.json\n")
    with pytest.raises(SIG.SignatureError):
        SIG.validate_receipt(output, root_inputs=fixture["root"], trust_root_path=fixture["trust"],
                             release_path=fixture["release"], packages_path=fixture["packages"],
                             repository=fixture["repository"], deb_bindings=fixture["deb_bindings"],
                             homedir=fixture["homedir"], release_kind="INRELEASE", signature_path=None)


def test_expired_revoked_and_missing_inventory_fail_closed(tmp_path):
    fixture = _fixture(tmp_path)
    trust = json.loads(fixture["trust"].read_text())
    trust["validity"]["subkeys"][0]["revoked"] = True
    trust["canonical_sha256"] = SIG.canonical_hash(trust, "canonical_sha256")
    _json(fixture["trust"], trust)
    with pytest.raises(SIG.SignatureError):
        SIG.validate_trust_root(fixture["root"], fixture["trust"])
    fixture = _fixture(tmp_path / "missing")
    fixture["inventory"].unlink()
    with pytest.raises(SIG.SignatureError):
        _seal(fixture, fixture["root"] / "receipt")


def test_plan_rejects_rehashed_command_and_checks_source_root(tmp_path):
    fixture = _fixture(tmp_path)
    plan = PLAN.build_plan(
        root=fixture["root"], trust_root_path=fixture["trust"], release_path=fixture["release"],
        packages_path=fixture["packages"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"], homedir=fixture["homedir"],
        release_kind="INRELEASE", signature_path=None, output_root=tmp_path / "plan-output")
    assert PLAN.validate_plan(plan, root=fixture["root"], repository=fixture["repository"],
                              deb_bindings=fixture["deb_bindings"])["status"] == "PASS"
    tampered = json.loads(json.dumps(plan))
    tampered["workflow"][4]["argv"].insert(tampered["workflow"][4]["argv"].index("--verify"),
                                             "--safe-but-unknown")
    tampered["workflow"][4]["argv_sha256"] = hashlib.sha256(
        SIG.canonical_bytes(tampered["workflow"][4]["argv"])).hexdigest()
    tampered["canonical_sha256"] = SIG.canonical_hash(tampered, "canonical_sha256")
    with pytest.raises(PLAN.SignaturePlanError):
        PLAN.validate_plan(tampered, root=fixture["root"], repository=fixture["repository"],
                           deb_bindings=fixture["deb_bindings"])


def test_input_symlink_parent_and_hardlink_are_rejected(tmp_path):
    fixture = _fixture(tmp_path)
    trust = json.loads(fixture["trust"].read_text())
    trust["keyring_path"] = "alias-keyring.gpg"
    trust["canonical_sha256"] = SIG.canonical_hash(trust, "canonical_sha256")
    _json(fixture["trust"], trust)
    fixture["root"].joinpath("alias-keyring.gpg").symlink_to("keyring.gpg")
    with pytest.raises(SIG.SignatureError, match="symlink"):
        SIG.validate_trust_root(fixture["root"], fixture["trust"])

    fixture = _fixture(tmp_path / "parent")
    fixture["root"].joinpath("real").mkdir()
    fixture["root"].joinpath("real/InRelease").write_bytes(fixture["release"].read_bytes())
    fixture["root"].joinpath("release-parent").symlink_to("real", target_is_directory=True)
    with pytest.raises(SIG.SignatureError, match="symlink"):
        SIG._binding(fixture["root"], fixture["root"] / "release-parent/InRelease",
                     fixture["packages"], fixture["repository"], fixture["deb_bindings"])

    fixture = _fixture(tmp_path / "hardlink")
    fixture["root"].joinpath("demo-hard.deb").hardlink_to(fixture["deb"])
    bindings = [{"name": "demo", "version": "1", "architecture": "amd64",
                 "path": "demo-hard.deb"}]
    with pytest.raises(SIG.SignatureError, match="single-link"):
        SIG._binding(fixture["root"], fixture["release"], fixture["packages"],
                     fixture["repository"], bindings)

    root_fixture = _fixture(tmp_path / "root-target")
    root_link = tmp_path / "root-link"
    root_link.symlink_to(root_fixture["root"], target_is_directory=True)
    with pytest.raises(SIG.SignatureError, match="non-symlink"):
        SIG.validate_trust_root(root_link, root_fixture["trust"])
