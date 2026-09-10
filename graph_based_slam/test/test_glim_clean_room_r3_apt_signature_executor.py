#!/usr/bin/env python3
"""Synthetic tests for the non-promoting r3 signature executor candidate."""

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


SIG = _load(R3 / "apt_release_signature.py", "r3_executor_signature_contract")
PLAN = _load(ROOT / "scripts/plan_glim_clean_room_r3_apt_signature.py",
             "r3_executor_signature_planner")
EXEC = _load(R3 / "apt_release_signature_executor.py", "r3_signature_executor")


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _write(path: Path, data: bytes, mode: int = 0o600) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    path.chmod(mode)


def _json(path: Path, value: object) -> None:
    _write(path, SIG.canonical_bytes(value) + b"\n")


def _fixture(tmp_path: Path):
    fingerprint = "A" * 40
    sub_fingerprint = "B" * 40
    key_id = fingerprint[-16:]
    keyring = b"synthetic-keyring-bytes\n"
    _write(tmp_path / "keyring.gpg", keyring)
    executable = tmp_path / "bin" / "gpg"
    executable_bytes = b"synthetic-gpg-binary-v1\n"
    _write(executable, executable_bytes, 0o755)
    trust = {
        "schema": SIG.TRUST_SCHEMA, "schema_version": 1, "status": "PRECOMMITTED",
        "key_id": key_id, "fingerprint": fingerprint, "keyring_path": "keyring.gpg",
        "keyring_bytes": len(keyring), "keyring_sha256": _sha(keyring),
        "verification_time": 1800000000,
        "validity": {"primary_fingerprint": fingerprint, "created_at": 1700000000,
                      "expires_at": 0, "revoked": False,
                      "subkeys": [{"fingerprint": sub_fingerprint, "created_at": 1700000000,
                                   "expires_at": 0, "revoked": False}]},
        "verifier": {"name": "gpg", "version": "2.4.synthetic",
                     "binary_sha256": _sha(executable_bytes),
                     "binary_path": str(executable), "status_protocol": "gpg-status-fd-v1"},
    }
    trust["canonical_sha256"] = SIG.canonical_hash(trust)
    _json(tmp_path / "trust-root.json", trust)
    package = (b"Package: demo\nVersion: 1\nArchitecture: amd64\n"
               b"Filename: pool/demo.deb\nSize: 9\nSHA256: " +
               _sha(b"demo-deb\n").encode() + b"\n\n")
    _write(tmp_path / "Packages", package)
    release = (b"Suite: jazzy\nSHA256:\n " + _sha(package).encode() + b" " +
               str(len(package)).encode() + b" main/binary-amd64/Packages\n")
    _write(tmp_path / "InRelease", release)
    _write(tmp_path / "demo.deb", b"demo-deb\n")
    repository = {
        "url": "https://apt.example.invalid/jazzy",
        "release_url": "https://apt.example.invalid/jazzy/InRelease",
        "release_path": "var/lib/apt/lists/example_InRelease",
        "release_sha256": _sha(release),
        "packages_url": "https://apt.example.invalid/jazzy/main/binary-amd64/Packages",
        "packages_path": "var/lib/apt/lists/example_main_binary-amd64_Packages",
        "packages_sha256": _sha(package),
    }
    return {"root": tmp_path, "trust": tmp_path / "trust-root.json",
            "release": tmp_path / "InRelease", "packages": tmp_path / "Packages",
            "repository": repository,
            "deb_bindings": [{"name": "demo", "version": "1", "architecture": "amd64",
                              "path": "demo.deb"}], "executable": executable}


def _plan(fixture, output: Path):
    return PLAN.build_plan(root=fixture["root"], trust_root_path=fixture["trust"],
                           release_path=fixture["release"], packages_path=fixture["packages"],
                           repository=fixture["repository"], deb_bindings=fixture["deb_bindings"],
                           homedir=fixture["root"] / "fresh-gpg", release_kind="INRELEASE",
                           signature_path=None, output_root=output)


class _FakeRunner:
    def __init__(self, fingerprint: str, subkey: str, *, fail_phase: str | None = None,
                 timeout_phase: str | None = None):
        self.fingerprint = fingerprint
        self.subkey = subkey
        self.fail_phase = fail_phase
        self.timeout_phase = timeout_phase
        self.calls: list[dict[str, object]] = []

    def run(self, argv, **kwargs):
        phase = "version" if argv[-1] == "--version" else \
            "inventory" if "--show-keys" in argv else "verify"
        self.calls.append({"phase": phase, "argv": list(argv), **kwargs})
        if phase == "version":
            stdout = b"gpg (GnuPG) 2.4.synthetic\n"
        elif phase == "inventory":
            stdout = (f"pub:-:rsa4096:1:{self.fingerprint[-16:]}:1700000000:0:::::::\n"
                      f"fpr:::::::::{self.fingerprint}:\n"
                      f"sub:-:rsa4096:1:{self.subkey[-16:]}:1700000000:0:::::::\n"
                      f"fpr:::::::::{self.subkey}:\n").encode()
        else:
            stdout = (f"[GNUPG:] NEWSIG\n[GNUPG:] KEY_CONSIDERED {self.fingerprint} 0\n"
                      f"[GNUPG:] GOODSIG {self.subkey[-16:]} Synthetic signer\n"
                      f"[GNUPG:] VALIDSIG {self.subkey} 2026-01-01 1767225600 0 1 0 1 10 00 {self.fingerprint}\n"
                      "[GNUPG:] TRUST_ULTIMATE 0 pgp\n").encode()
        return {"returncode": 1 if phase == self.fail_phase else 0,
                "timed_out": phase == self.timeout_phase, "signal": None,
                "stdout": stdout, "stderr": b""}


def test_executor_fake_runner_seals_nonpromoting_pass_and_reopens(tmp_path):
    fixture = _fixture(tmp_path / "inputs")
    output = tmp_path / "output"
    plan = _plan(fixture, output)
    runner = _FakeRunner("A" * 40, "B" * 40)
    clock_values = iter((10, 25))
    result = EXEC.execute_signature_plan(
        plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"], runner=runner, clock=lambda: next(clock_values))
    assert result["status"] == "PASS"
    assert result["outcome"] == "PASS"
    assert result["benchmark_eligible"] is False
    assert not (fixture["root"] / "fresh-gpg").exists()
    receipt = json.loads((output / "apt-release-signature.executor.receipt.json").read_text())
    Draft202012Validator(json.loads(
        (R3 / "apt_release_signature_executor.schema.json").read_text())).validate(receipt)
    assert receipt["execution_mode"] == "INJECTED_FIXTURE_ONLY"
    assert [item["phase"] for item in receipt["commands"]] == [
        "tool_version", "key-inventory", "signature-verify"]
    assert [item["phase"] for item in receipt["phases"]] == list(EXEC.PHASES)
    assert all(item["shell"] is False for item in receipt["commands"])
    assert all(item["stdin"] == "DEVNULL" for item in receipt["commands"])
    assert EXEC.validate_executor_receipt(
        output, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"])["status"] == "PASS"


@pytest.mark.parametrize("mode", ["failure", "timeout"])
def test_executor_seals_failed_attempt_and_never_promotes(tmp_path, mode):
    fixture = _fixture(tmp_path / "inputs")
    output = tmp_path / "output"
    plan = _plan(fixture, output)
    runner = _FakeRunner("A" * 40, "B" * 40,
                         fail_phase="verify" if mode == "failure" else None,
                         timeout_phase="verify" if mode == "timeout" else None)
    values = iter((20, 35))
    result = EXEC.execute_signature_plan(
        plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"], runner=runner, clock=lambda: next(values))
    assert result["status"] == "PASS"
    assert result["outcome"] == "FAIL"
    receipt = json.loads((output / "apt-release-signature.executor.receipt.json").read_text())
    assert receipt["benchmark_eligible"] is False
    assert receipt["promotion"] == "FORBIDDEN"
    assert receipt["phases"][3]["status"] == "PASS"
    assert receipt["phases"][4]["status"] == "FAIL"
    assert not (fixture["root"] / "fresh-gpg").exists()


def test_executor_rejects_preexisting_output_and_ambient_binary_fallback(tmp_path):
    fixture = _fixture(tmp_path / "inputs")
    output = tmp_path / "output"
    output.mkdir()
    plan = _plan(fixture, tmp_path / "fresh-output")
    plan["scope"]["output_root"] = str(output)
    plan["canonical_sha256"] = SIG.canonical_hash(plan, "canonical_sha256")
    with pytest.raises(EXEC.ExecutorError):
        EXEC.execute_signature_plan(plan=plan, root_inputs=fixture["root"],
                                    repository=fixture["repository"],
                                    deb_bindings=fixture["deb_bindings"],
                                    runner=_FakeRunner("A" * 40, "B" * 40))
    trust = json.loads(fixture["trust"].read_text())
    del trust["verifier"]["binary_path"]
    trust["canonical_sha256"] = SIG.canonical_hash(trust, "canonical_sha256")
    _json(fixture["trust"], trust)
    with pytest.raises(SIG.SignatureError):
        SIG.validate_trust_root(fixture["root"], fixture["trust"])


def test_executor_rejects_explicit_subprocess_runner_without_authorization(tmp_path):
    fixture = _fixture(tmp_path / "inputs")
    plan = _plan(fixture, tmp_path / "output")
    with pytest.raises(EXEC.ExecutorError, match="authorization"):
        EXEC.execute_signature_plan(
            plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"], runner=EXEC.SubprocessRunner())


def test_executor_rejects_good_version_with_nonzero_exit(tmp_path):
    fixture = _fixture(tmp_path / "inputs")
    output = tmp_path / "output"
    plan = _plan(fixture, output)

    class BadVersion(_FakeRunner):
        def run(self, argv, **kwargs):
            result = super().run(argv, **kwargs)
            if argv[-1] == "--version":
                result["returncode"] = 1
            return result

    result = EXEC.execute_signature_plan(
        plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"], runner=BadVersion("A" * 40, "B" * 40),
        clock=iter((1, 2)).__next__)
    assert result["outcome"] == "FAIL"
    receipt = json.loads((output / "apt-release-signature.executor.receipt.json").read_text())
    assert receipt["phases"][0]["status"] == "PASS"
    assert receipt["commands"][0]["returncode"] == 1
    assert receipt["phases"][3]["status"] == "NOT_RUN"


def test_executor_strong_reopen_rejects_self_rehashed_command_and_cleanup(tmp_path):
    fixture = _fixture(tmp_path / "inputs")
    output = tmp_path / "output"
    plan = _plan(fixture, output)
    EXEC.execute_signature_plan(
        plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"], runner=_FakeRunner("A" * 40, "B" * 40),
        clock=iter((1, 2)).__next__)
    path = output / "apt-release-signature.executor.receipt.json"
    sidecar = output / "apt-release-signature.executor.receipt.json.sha256"
    receipt = json.loads(path.read_text())
    receipt["commands"] = []
    receipt["canonical_sha256"] = SIG.canonical_hash(receipt, "canonical_sha256")
    data = SIG.canonical_bytes(receipt) + b"\n"
    path.chmod(0o600)
    sidecar.chmod(0o600)
    path.write_bytes(data)
    sidecar.write_text(f"{_sha(data)}  {path.name}\n")
    with pytest.raises(EXEC.ExecutorError):
        EXEC.validate_executor_receipt(
            output, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"])


def test_executor_rejects_output_root_swap_and_resource_limit_drift(tmp_path):
    fixture = _fixture(tmp_path / "inputs")
    output = tmp_path / "output"
    plan = _plan(fixture, output)

    class SwapRunner(_FakeRunner):
        def run(self, argv, **kwargs):
            if not self.calls:
                old = output.with_name("output-old")
                output.rename(old)
                output.mkdir(mode=0o700)
            return super().run(argv, **kwargs)

    with pytest.raises(EXEC.ExecutorError):
        EXEC.execute_signature_plan(
            plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"], runner=SwapRunner("A" * 40, "B" * 40),
            clock=iter((1, 2)).__next__)
    runner = EXEC.SubprocessRunner()
    with pytest.raises(EXEC.ExecutorError, match="fixed contract"):
        runner.run(["/definitely/not-run", "--version"], env={"HOME": "x"},
                   cwd=fixture["root"], stdin=None, stdout_path=tmp_path / "out",
                   stderr_path=tmp_path / "err", timeout=30,
                   resource_limits={"RLIMIT_CORE": 0, "RLIMIT_FSIZE": 1}, shell=False)


@pytest.mark.parametrize("field", ["execution_mode", "cleanup"])
def test_executor_reopen_rejects_nonpromoting_receipt_field_tamper(tmp_path, field):
    fixture = _fixture(tmp_path / "inputs")
    output = tmp_path / "output"
    plan = _plan(fixture, output)
    EXEC.execute_signature_plan(
        plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
        deb_bindings=fixture["deb_bindings"], runner=_FakeRunner("A" * 40, "B" * 40),
        clock=iter((1, 2)).__next__)
    path = output / "apt-release-signature.executor.receipt.json"
    sidecar = output / "apt-release-signature.executor.receipt.json.sha256"
    receipt = json.loads(path.read_text())
    if field == "execution_mode":
        receipt[field] = "HOST_EXECUTED"
    else:
        receipt[field]["post_absent"] = False
    receipt["canonical_sha256"] = SIG.canonical_hash(receipt, "canonical_sha256")
    data = SIG.canonical_bytes(receipt) + b"\n"
    path.chmod(0o600)
    sidecar.chmod(0o600)
    path.write_bytes(data)
    sidecar.write_text(f"{_sha(data)}  {path.name}\n")
    with pytest.raises(EXEC.ExecutorError):
        EXEC.validate_executor_receipt(
            output, plan=plan, root_inputs=fixture["root"], repository=fixture["repository"],
            deb_bindings=fixture["deb_bindings"])
