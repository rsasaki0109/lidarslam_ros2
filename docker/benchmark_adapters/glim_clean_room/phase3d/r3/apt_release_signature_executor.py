#!/usr/bin/env python3
"""Bounded host executor candidate for the r3 OpenPGP signature contract.

This module is deliberately opt-in and non-promoting.  It accepts only a
fully revalidated plan, uses an exact absolute GnuPG identity from the trust
root, and records every command, output byte, phase transition, and cleanup
result.  The default runner is implemented with ``shell=False`` for future
host use, but this repository task never invokes it: tests inject a small
bounded runner and the resulting receipt remains
``IMPLEMENTED_NOT_RUNTIME_VALIDATED``/``benchmark_eligible=false``.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import resource
import stat
import subprocess
import time
from typing import Any, Mapping


MODULE_PATH = Path(__file__).with_name("apt_release_signature.py")
EXECUTOR_PATH = Path(__file__)
AUTHORIZATION_PATH = Path(__file__).with_name("apt_release_signature_authorization.py")
PLANNER_PATH = Path(__file__).parents[5] / "scripts" / "plan_glim_clean_room_r3_apt_signature.py"
SCHEMA = "glim_clean_room_r3_apt_signature_executor_receipt_v1"
MAX_RECEIPT_BYTES = 2 * 1024 * 1024
MAX_LOG_BYTES = 256 * 1024
MAX_TOOL_BYTES = 32 * 1024 * 1024
COMMAND_TIMEOUT_SECONDS = 30
PHASES = ("trust_root_reopen", "release_reopen", "homedir_freshness",
          "key_inventory", "signature_verify", "binding_reopen",
          "seal_receipt", "cleanup")
FORBIDDEN = frozenset({"sh", "bash", "-c", "-lc", "curl", "wget", "nc", "socat",
                       "--auto-key-retrieve", "--auto-key-import", "--auto-key-locate",
                       "--use-agent", "--gpg-agent"})
FIXED_ENV_NAMES = ("HOME", "GNUPGHOME", "LANG", "LC_ALL", "TZ")
RESOURCE_LIMITS = {"RLIMIT_CORE": 0, "RLIMIT_FSIZE": MAX_LOG_BYTES}


class ExecutorError(ValueError):
    """Raised when plan or host state is unsafe to execute or seal."""


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ExecutorError(f"cannot load contract module: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SIG = _load(MODULE_PATH, "r3_apt_signature_executor_contract")
PLAN = _load(PLANNER_PATH, "r3_apt_signature_executor_planner")
AUTHORIZATION = _load(AUTHORIZATION_PATH, "r3_apt_signature_host_authorization")


def _canonical(value: Any) -> bytes:
    return SIG.canonical_bytes(value)


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _safe_abs(value: Any, label: str) -> Path:
    parts = value[1:].split("/") if isinstance(value, str) and value.startswith("/") else []
    if not isinstance(value, str) or not value.startswith("/") or \
            Path(value).as_posix() != value or any(part in {"", ".", ".."}
                                                   for part in parts):
        raise ExecutorError(f"{label} must be an absolute normalized path")
    return Path(value)


def _parent_no_symlink(path: Path, label: str) -> None:
    SIG._ensure_parent_no_symlink(path, label)


def _bounded(path: Path, label: str, *, limit: int = MAX_LOG_BYTES,
             allow_empty: bool = True) -> bytes:
    try:
        before = path.lstat()
    except OSError as error:
        raise ExecutorError(f"{label} cannot be inspected: {error}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or \
            before.st_nlink != 1 or before.st_size > limit or \
            (before.st_size == 0 and not allow_empty):
        raise ExecutorError(f"{label} is not a bounded single-link regular file")
    try:
        with path.open("rb") as stream:
            fd_before = os.fstat(stream.fileno())
            data = stream.read(limit + 1)
            fd_after = os.fstat(stream.fileno())
    except OSError as error:
        raise ExecutorError(f"{label} cannot be read: {error}") from error
    if (fd_before.st_dev, fd_before.st_ino) != (before.st_dev, before.st_ino) or \
            (fd_after.st_dev, fd_after.st_ino) != (before.st_dev, before.st_ino) or \
            fd_after.st_size != before.st_size or len(data) != before.st_size or \
            len(data) > limit or (not data and not allow_empty):
        raise ExecutorError(f"{label} changed while being read")
    return data


def _write_exclusive(path: Path, data: bytes, label: str) -> None:
    if path.exists() or path.is_symlink() or not path.parent.is_dir() or \
            path.parent.is_symlink() or len(data) > MAX_LOG_BYTES:
        raise ExecutorError(f"{label} is not a fresh bounded output")
    try:
        with path.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
    except OSError as error:
        raise ExecutorError(f"cannot write {label}: {error}") from error


def _identity(path: Path, root: Path, label: str, *, limit: int = MAX_LOG_BYTES,
              allow_empty: bool = True) -> dict[str, Any]:
    data = _bounded(path, label, limit=limit, allow_empty=allow_empty)
    try:
        relative = path.relative_to(root).as_posix()
    except ValueError as error:
        raise ExecutorError(f"{label} escapes output root") from error
    if not relative or relative.startswith("../") or "/../" in f"/{relative}":
        raise ExecutorError(f"{label} has unsafe relative path")
    return {"path": relative, "bytes": len(data), "sha256": _sha(data)}


def _fresh_directory(path: Path, label: str) -> dict[str, int]:
    _safe_abs(str(path), label)
    _parent_no_symlink(path, label)
    if path.exists() or path.is_symlink():
        raise ExecutorError(f"{label} must not pre-exist")
    try:
        path.mkdir(mode=0o700)
        info = path.lstat()
    except OSError as error:
        raise ExecutorError(f"cannot create fresh {label}: {error}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode) or \
            stat.S_IMODE(info.st_mode) != 0o700:
        raise ExecutorError(f"fresh {label} has unsafe type or mode")
    return {"device": info.st_dev, "inode": info.st_ino, "mode": stat.S_IMODE(info.st_mode)}


def _assert_dir_identity(path: Path, identity: Mapping[str, int], label: str) -> None:
    try:
        info = path.lstat()
    except OSError as error:
        raise ExecutorError(f"{label} disappeared before cleanup: {error}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode) or \
            info.st_dev != identity["device"] or info.st_ino != identity["inode"]:
        raise ExecutorError(f"{label} identity changed")


def _remove_fresh_directory(path: Path, identity: Mapping[str, int], label: str) -> None:
    _assert_dir_identity(path, identity, label)
    for child in sorted(path.iterdir(), key=lambda item: item.name, reverse=True):
        info = child.lstat()
        if stat.S_ISLNK(info.st_mode) or (stat.S_ISREG(info.st_mode) and info.st_nlink != 1):
            raise ExecutorError(f"{label} contains unsafe cleanup entry: {child.name}")
        if stat.S_ISDIR(info.st_mode):
            _remove_fresh_directory(child, {"device": info.st_dev, "inode": info.st_ino}, label)
        elif stat.S_ISREG(info.st_mode):
            child.unlink()
        else:
            raise ExecutorError(f"{label} contains a special file: {child.name}")
    path.rmdir()
    if path.exists() or path.is_symlink():
        raise ExecutorError(f"{label} cleanup did not remove directory")


def _checked_executable(path: str, expected_sha: str) -> dict[str, Any]:
    executable = _safe_abs(path, "GnuPG executable")
    _parent_no_symlink(executable, "GnuPG executable")
    data = _bounded(executable, "GnuPG executable", limit=MAX_TOOL_BYTES, allow_empty=False)
    try:
        info = executable.lstat()
    except OSError as error:
        raise ExecutorError(f"GnuPG executable cannot be inspected: {error}") from error
    if not (info.st_mode & stat.S_IXUSR) or _sha(data) != expected_sha:
        raise ExecutorError("GnuPG executable identity or execute bit drift")
    return {"path": str(executable), "bytes": len(data), "sha256": _sha(data),
            "mode": stat.S_IMODE(info.st_mode)}


def _check_argv(argv: Any, executable: str, label: str) -> list[str]:
    if not isinstance(argv, list) or not argv or any(not isinstance(item, str) or not item
                                                     for item in argv):
        raise ExecutorError(f"{label} argv is malformed")
    if argv[0] != executable or any(item in FORBIDDEN for item in argv):
        raise ExecutorError(f"{label} argv is not the fixed no-shell contract")
    return list(argv)


def _fixed_env(homedir: Path) -> dict[str, str]:
    return {"HOME": str(homedir), "GNUPGHOME": str(homedir),
            "LANG": "C", "LC_ALL": "C", "TZ": "UTC"}


def _normalize_result(result: Any, stdout_path: Path, stderr_path: Path,
                      label: str) -> dict[str, Any]:
    if not isinstance(result, Mapping):
        raise ExecutorError(f"{label} runner result is not an object")
    allowed = {"returncode", "timed_out", "signal", "stdout", "stderr"}
    if any(key not in allowed for key in result):
        raise ExecutorError(f"{label} runner result contains unknown fields")
    returncode = result.get("returncode")
    if type(returncode) is not int or returncode < -256 or returncode > 255:
        raise ExecutorError(f"{label} runner return code is invalid")
    timed_out = result.get("timed_out", False)
    if type(timed_out) is not bool:
        raise ExecutorError(f"{label} runner timeout flag is invalid")
    signal_value = result.get("signal")
    if signal_value is not None and (type(signal_value) is not int or signal_value <= 0):
        raise ExecutorError(f"{label} runner signal is invalid")
    for key, path in (("stdout", stdout_path), ("stderr", stderr_path)):
        value = result.get(key)
        if value is not None:
            if not isinstance(value, bytes) or len(value) > MAX_LOG_BYTES:
                raise ExecutorError(f"{label} {key} bytes are invalid")
            if path.exists() or path.is_symlink():
                if _bounded(path, f"{label} {key} log") != value:
                    raise ExecutorError(f"{label} {key} log differs from runner bytes")
            else:
                _write_exclusive(path, value, f"{label} {key} log")
        elif not path.exists() and not path.is_symlink():
            raise ExecutorError(f"{label} did not produce {key} log")
    return {"returncode": returncode, "timed_out": timed_out,
            "signal": signal_value}


class SubprocessRunner:
    """The future production runner; no command uses a shell or ambient env."""

    @staticmethod
    def _limits() -> None:
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        resource.setrlimit(resource.RLIMIT_FSIZE, (MAX_LOG_BYTES, MAX_LOG_BYTES))

    def run(self, argv: list[str], *, env: Mapping[str, str], cwd: Path,
            stdin: Any, stdout_path: Path, stderr_path: Path, timeout: int,
            resource_limits: Mapping[str, int], shell: bool) -> dict[str, Any]:
        if shell is not False or stdin is not subprocess.DEVNULL or \
                dict(env).keys() != set(FIXED_ENV_NAMES) or timeout != COMMAND_TIMEOUT_SECONDS or \
                dict(resource_limits) != RESOURCE_LIMITS:
            raise ExecutorError("subprocess runner invocation is outside fixed contract")
        _write_parent = stdout_path.parent
        if not _write_parent.is_dir() or _write_parent.is_symlink():
            raise ExecutorError("subprocess log parent is unsafe")
        try:
            with stdout_path.open("xb") as stdout, stderr_path.open("xb") as stderr:
                process = subprocess.Popen(argv, shell=False, cwd=str(cwd), env=dict(env),
                                           stdin=subprocess.DEVNULL, stdout=stdout,
                                           stderr=stderr, preexec_fn=self._limits,
                                           close_fds=True)
                try:
                    process.wait(timeout=timeout)
                    return {"returncode": process.returncode, "timed_out": False,
                            "signal": (-process.returncode if process.returncode < 0 else None)}
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                    return {"returncode": process.returncode, "timed_out": True,
                            "signal": (-process.returncode if process.returncode < 0 else None)}
        except OSError as error:
            raise ExecutorError(f"subprocess runner failed: {error}") from error


def _command_record(*, phase: str, argv: list[str], env: Mapping[str, str], cwd: Path,
                    stdout_path: Path, stderr_path: Path, result: Mapping[str, Any],
                    output_root: Path) -> dict[str, Any]:
    stdout = _identity(stdout_path, output_root, f"{phase} stdout")
    stderr = _identity(stderr_path, output_root, f"{phase} stderr")
    env_data = dict(sorted(env.items()))
    return {"phase": phase, "argv": argv, "argv_sha256": _sha(_canonical(argv)),
            "env": env_data, "env_sha256": _sha(_canonical(env_data)),
            "cwd": str(cwd), "stdin": "DEVNULL", "shell": False,
            "timeout_seconds": COMMAND_TIMEOUT_SECONDS,
            "resource_limits": dict(sorted(RESOURCE_LIMITS.items())),
            "stdout": stdout, "stderr": stderr,
            "returncode": result["returncode"], "timed_out": result["timed_out"],
            "signal": result["signal"]}


def _version(value: bytes, expected: str) -> str:
    if not value or len(value) > MAX_LOG_BYTES:
        raise ExecutorError("GnuPG version output is empty or oversized")
    try:
        text = value.decode("ascii")
    except UnicodeDecodeError as error:
        raise ExecutorError("GnuPG version output is not ASCII") from error
    lines = [line for line in text.splitlines() if line]
    if not lines or expected not in lines[0] or any(ord(char) < 32 and char not in "\n\r\t"
                                                   for char in text):
        raise ExecutorError("GnuPG version output does not match trust-root version")
    return lines[0]


def _receipt_write(output_root: Path, receipt: dict[str, Any]) -> tuple[Path, str]:
    receipt["canonical_sha256"] = SIG.canonical_hash(receipt, "canonical_sha256")
    data = _canonical(receipt) + b"\n"
    path = output_root / "apt-release-signature.executor.receipt.json"
    sidecar = output_root / "apt-release-signature.executor.receipt.json.sha256"
    _write_exclusive(path, data, "executor receipt")
    path.chmod(0o444)
    sidecar_data = f"{_sha(data)}  {path.name}\n".encode("ascii")
    _write_exclusive(sidecar, sidecar_data, "executor receipt sidecar")
    sidecar.chmod(0o444)
    return path, _sha(data)


def _phase_rows() -> list[dict[str, Any]]:
    return [{"ordinal": index, "phase": phase, "status": "NOT_RUN"}
            for index, phase in enumerate(PHASES)]


def _verify_host_authorization(authorization: Mapping[str, Any], *, plan: Mapping[str, Any],
                                root_inputs: Path, repository: Mapping[str, Any],
                                deb_bindings: list[Mapping[str, Any]],
                                verification_time: int | None = None,
                                require_live_machine: bool = True) -> dict[str, Any]:
    if not isinstance(authorization, Mapping):
        raise ExecutorError("host runtime authorization is missing or malformed")
    if verification_time is None:
        verification_time = int(time.time())
    issued_at = authorization.get("issued_at")
    expires_at = authorization.get("expires_at")
    if type(issued_at) is not int or type(expires_at) is not int or \
            type(verification_time) is not int or verification_time < 0 or \
            verification_time < issued_at or verification_time >= expires_at:
        raise ExecutorError("host runtime authorization is outside its signed validity window")
    try:
        projection = AUTHORIZATION.verify_authorization(
            authorization, plan=plan, root_inputs=root_inputs, repository=repository,
            deb_bindings=deb_bindings,
            executor_source_sha256=_sha(EXECUTOR_PATH.read_bytes()), now=verification_time,
            require_live_machine=require_live_machine)
        if projection.get("verification_time") != verification_time:
            raise ExecutorError("authorization verification time projection drift")
        return projection
    except Exception as error:
        raise ExecutorError(f"host runtime authorization rejected: {error}") from error


def _claim_authorization_nonce(authorization: Mapping[str, Any], output_root: Path,
                               verification_time: int) -> dict[str, Any]:
    """Atomically consume one signed nonce before creating the output root."""
    nonce = authorization.get("nonce") if isinstance(authorization, Mapping) else None
    signature = authorization.get("signature") if isinstance(authorization, Mapping) else None
    payload_sha = signature.get("payload_sha256") if isinstance(signature, Mapping) else None
    if not isinstance(nonce, str) or not isinstance(payload_sha, str):
        raise ExecutorError("authorization nonce claim is incomplete")
    parent = output_root.parent
    _parent_no_symlink(output_root, "authorization nonce parent")
    try:
        parent_info = parent.lstat()
    except OSError as error:
        raise ExecutorError(f"authorization nonce parent is unavailable: {error}") from error
    if stat.S_ISLNK(parent_info.st_mode) or not stat.S_ISDIR(parent_info.st_mode):
        raise ExecutorError("authorization nonce parent is not a real directory")
    if output_root.exists() or output_root.is_symlink():
        raise ExecutorError("authorization output root must be absent before nonce claim")
    token = _sha(_canonical({"nonce": nonce, "payload_sha256": payload_sha}))
    claim_path = parent / f".glim-r3-authorization-{token}.claim"
    claim = {"schema": "glim_clean_room_r3_apt_signature_nonce_claim_v1",
             "nonce": nonce, "payload_sha256": payload_sha,
             "output_root": str(output_root), "verification_time": verification_time}
    data = _canonical(claim) + b"\n"
    try:
        with claim_path.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
    except FileExistsError as error:
        raise ExecutorError("authorization nonce was already consumed") from error
    except OSError as error:
        raise ExecutorError(f"authorization nonce claim could not be sealed: {error}") from error
    claim_path.chmod(0o444)
    return {"path": str(claim_path), "bytes": len(data), "sha256": _sha(data),
            "parent_device": parent_info.st_dev, "parent_inode": parent_info.st_ino}


def execute_signature_plan(*, plan: Mapping[str, Any], root_inputs: Path,
                           repository: Mapping[str, Any],
                           deb_bindings: list[Mapping[str, Any]],
                           runner: Any = None, clock: Any = None,
                           authorization: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Execute a fully validated plan using a bounded runner and seal its result.

    ``root_inputs`` contains only already captured trust/Release/Packages/deb
    inputs.  Output and GnuPG home paths come from the plan and must be fresh.
    A command failure returns a sealed ``outcome=FAIL`` result; preflight or
    cleanup identity failures raise and never claim a receipt was valid.
    """
    if not isinstance(plan, Mapping):
        raise ExecutorError("signature plan must be an object")
    root = SIG._checked_root(Path(root_inputs))
    try:
        PLAN.validate_plan(plan, root=root, repository=repository, deb_bindings=deb_bindings)
    except (ValueError, OSError, KeyError, TypeError) as error:
        raise ExecutorError(f"signature plan full validation failed: {error}") from error
    scope = plan.get("scope")
    trust_projection = plan.get("trust_root")
    if not isinstance(scope, Mapping) or not isinstance(trust_projection, Mapping):
        raise ExecutorError("validated plan has no scope/trust projection")
    trust_path = root / SIG._safe_relative(plan["trust_root"]["path"], "trust-root path")
    trust = SIG.validate_trust_root(root, trust_path)
    executable = _safe_abs(trust["verifier"]["binary_path"], "GnuPG executable")
    if str(executable) != trust_projection["verifier_binary_path"]:
        raise ExecutorError("plan/trust executable identity drift")
    tool = _checked_executable(str(executable), trust["verifier"]["binary_sha256"])
    release = root / SIG._safe_relative(scope["release_path"], "Release path")
    packages = root / SIG._safe_relative(scope["packages_path"], "Packages path")
    signature = root / SIG._safe_relative(scope["signature_path"], "signature path") \
        if scope["signature_path"] else None
    homedir = _safe_abs(scope["homedir"], "GnuPG homedir")
    output_root = _safe_abs(scope["output_root"], "executor output root")
    if homedir == output_root or output_root == root or homedir == root:
        raise ExecutorError("executor roots must be distinct from input root")
    # The planner has already fixed these source identities; reopen once more
    # before any output state is created so a stale plan cannot be promoted.
    binding = SIG._binding(root, release, packages, repository, deb_bindings,
                           release_kind=scope["release_kind"], signature_path=signature)
    if binding["binding_identity_sha256"] != scope["binding_identity_sha256"]:
        raise ExecutorError("plan binding identity drift")
    runtime_authorized = False
    authorization_verification_time: int | None = None
    nonce_claim: dict[str, Any] | None = None
    if runner is None or isinstance(runner, SubprocessRunner):
        if authorization is None:
            raise ExecutorError(
                "real GnuPG execution requires an independent host authorization")
        actual_authorization_time = int(time.time())
        authorization_projection = _verify_host_authorization(
            authorization, plan=plan, root_inputs=root, repository=repository,
            deb_bindings=deb_bindings, verification_time=actual_authorization_time,
            require_live_machine=True)
        authorization_verification_time = actual_authorization_time
        runtime_authorized = True
        runner = SubprocessRunner()
    elif authorization is not None:
        raise ExecutorError(
            "injected fixture runners cannot be paired with host runtime authorization")
    if clock is not None and not callable(clock):
        raise ExecutorError("clock must be callable")
    now = clock or time.monotonic_ns
    try:
        start_ns = now()
    except Exception as error:
        raise ExecutorError(f"clock preflight failed: {error}") from error
    if type(start_ns) is not int or start_ns < 0:
        raise ExecutorError("clock preflight did not return a nonnegative integer")
    if runtime_authorized:
        nonce_claim = _claim_authorization_nonce(
            authorization, output_root, authorization_verification_time)
    output_identity = _fresh_directory(output_root, "executor output root")
    phases = _phase_rows()
    commands: list[dict[str, Any]] = []
    error_text = ""
    outcome = "PASS"
    cleanup = {"requested": True, "removed": False, "post_absent": False,
               "identity": None, "error": ""}
    homedir_identity: dict[str, int] | None = None
    fake_runner = not runtime_authorized

    def mark(index: int, status: str) -> None:
        phases[index]["status"] = status

    def run_command(phase: str, argv: list[str], label: str) -> dict[str, Any]:
        stdout_path = output_root / f"{phase}.stdout"
        stderr_path = output_root / f"{phase}.stderr"
        env = _fixed_env(homedir)
        _check_argv(argv, str(executable), label)
        _assert_dir_identity(output_root, output_identity, "executor output root")
        result = runner.run(argv, env=env, cwd=root, stdin=subprocess.DEVNULL,
                            stdout_path=stdout_path, stderr_path=stderr_path,
                            timeout=COMMAND_TIMEOUT_SECONDS,
                            resource_limits=RESOURCE_LIMITS, shell=False)
        normalized = _normalize_result(result, stdout_path, stderr_path, label)
        _assert_dir_identity(output_root, output_identity, "executor output root")
        commands.append(_command_record(phase=phase, argv=argv, env=env, cwd=root,
                                        stdout_path=stdout_path, stderr_path=stderr_path,
                                        result=normalized, output_root=output_root))
        if normalized["timed_out"] or normalized["signal"] is not None or \
                normalized["returncode"] != 0:
            raise ExecutorError(f"{label} did not complete successfully")
        return normalized

    try:
        mark(0, "PASS")
        SIG._regular(release, "Release input", limit=SIG.MAX_TEXT_BYTES)
        SIG._regular(packages, "Packages input", limit=SIG.MAX_TEXT_BYTES)
        mark(1, "PASS")
        homedir_identity = _fresh_directory(homedir, "GnuPG homedir")
        cleanup["identity"] = homedir_identity
        mark(2, "PASS")
        env = _fixed_env(homedir)
        _assert_dir_identity(output_root, output_identity, "executor output root")
        version_result = runner.run([str(executable), "--version"], env=env, cwd=root,
                                    stdin=subprocess.DEVNULL,
                                    stdout_path=output_root / "tool-version.stdout",
                                    stderr_path=output_root / "tool-version.stderr",
                                    timeout=COMMAND_TIMEOUT_SECONDS,
                                    resource_limits=RESOURCE_LIMITS, shell=False)
        version_result = _normalize_result(version_result, output_root / "tool-version.stdout",
                                            output_root / "tool-version.stderr", "tool version")
        _assert_dir_identity(output_root, output_identity, "executor output root")
        commands.append(_command_record(phase="tool_version",
                                        argv=[str(executable), "--version"], env=env, cwd=root,
                                        stdout_path=output_root / "tool-version.stdout",
                                        stderr_path=output_root / "tool-version.stderr",
                                        result=version_result, output_root=output_root))
        version_stdout = _bounded(output_root / "tool-version.stdout", "tool version stdout")
        version = _version(version_stdout, trust["verifier"]["version"])
        if version_result["returncode"] != 0 or version_result["timed_out"] or \
                version_result["signal"] is not None:
            raise ExecutorError("GnuPG version command did not complete successfully")
        inventory_argv = SIG.build_key_inventory_argv(
            homedir=homedir, keyring=root / trust["keyring_path"], executable=str(executable))
        mark(3, "RUNNING")
        run_command("key-inventory", inventory_argv, "key inventory")
        inventory_data = _bounded(output_root / "key_inventory.stdout", "key inventory output",
                                  allow_empty=False) if (output_root / "key_inventory.stdout").exists() \
            else _bounded(output_root / "key-inventory.stdout", "key inventory output",
                          allow_empty=False)
        inventory = SIG.validate_key_inventory_output(inventory_data, trust)
        mark(3, "PASS")
        verify_argv = SIG.build_verify_argv(
            homedir=homedir, keyring=root / trust["keyring_path"], release=release,
            release_kind=scope["release_kind"], signature=signature,
            executable=str(executable))
        mark(4, "RUNNING")
        run_command("signature-verify", verify_argv, "signature verification")
        status_data = _bounded(output_root / "signature-verify.stdout",
                               "signature verification status", allow_empty=False)
        status = SIG.validate_status_output(status_data, trust)
        mark(4, "PASS")
        mark(5, "RUNNING")
        binding = SIG._binding(root, release, packages, repository, deb_bindings,
                               release_kind=scope["release_kind"], signature_path=signature)
        mark(5, "PASS")
    except Exception as error:  # seal the truthful failed attempt below
        outcome = "FAIL"
        error_text = f"{type(error).__name__}: {error}"[:1024]
        for row in phases:
            if row["status"] == "RUNNING":
                row["status"] = "FAIL"
                break
    finally:
        if homedir_identity is not None:
            try:
                _remove_fresh_directory(homedir, homedir_identity, "GnuPG homedir")
                cleanup["removed"] = True
                cleanup["post_absent"] = not homedir.exists() and not homedir.is_symlink()
            except Exception as error:
                cleanup["error"] = f"{type(error).__name__}: {error}"[:512]
                outcome = "FAIL"
                error_text = error_text or cleanup["error"]
        else:
            cleanup["error"] = "GnuPG homedir was not created"
            cleanup["post_absent"] = not homedir.exists() and not homedir.is_symlink()
            outcome = "FAIL"
            error_text = error_text or cleanup["error"]
    if not cleanup["post_absent"]:
        outcome = "FAIL"
        error_text = error_text or "GnuPG homedir cleanup did not prove absence"
    if outcome == "PASS":
        mark(6, "PASS")
    else:
        if phases[6]["status"] == "NOT_RUN":
            phases[6]["status"] = "NOT_RUN"
    if cleanup["post_absent"]:
        mark(7, "PASS")
    else:
        mark(7, "FAIL")
    end_ns = now()
    if type(start_ns) is not int or type(end_ns) is not int or end_ns < start_ns:
        outcome = "FAIL"
        error_text = error_text or "clock source is non-monotonic"
    receipt = {
        "schema": SCHEMA, "schema_version": 1, "status": "SEALED_EXECUTOR_RESULT",
        "candidate_status": "IMPLEMENTED_NOT_RUNTIME_VALIDATED" if fake_runner else
        "HOST_AUTHORIZED_SUBPROCESS",
        "benchmark_eligible": (not fake_runner) and outcome == "PASS",
        "signature_runtime": "NOT_RUN" if fake_runner else
        ("RUNTIME_VERIFIED" if outcome == "PASS" else "RUNTIME_FAILED"),
        "execution_mode": "INJECTED_FIXTURE_ONLY" if fake_runner else "HOST_AUTHORIZED_SUBPROCESS",
        "outcome": outcome,
        "promotion": "HOST_AUTHORIZED_RUNTIME" if not fake_runner and outcome == "PASS" else "FORBIDDEN",
        "error": error_text,
        "plan_sha256": _sha(_canonical(plan)),
        "output_root_identity": output_identity,
        "input": {"root": str(root), "trust_root_path": plan["trust_root"]["path"],
                  "trust_root_sha256": plan["trust_root"]["file_sha256"],
                  "release_path": scope["release_path"], "release_sha256": scope["release_sha256"],
                  "packages_path": scope["packages_path"], "packages_sha256": scope["packages_sha256"],
                  "release_kind": scope["release_kind"],
                  "signature_path": scope["signature_path"],
                  "signature_sha256": scope["signature_sha256"],
                  "binding_identity_sha256": binding["binding_identity_sha256"]},
        "tool": {**tool, "version": trust["verifier"]["version"],
                 "version_line": locals().get("version", ""),
                 "command": [str(executable), "--version"],
                 "command_sha256": _sha(_canonical([str(executable), "--version"]))},
        "commands": commands, "phases": phases,
        "binding": binding if outcome == "PASS" else None,
        "inventory": locals().get("inventory", None),
        "signature": locals().get("status", None),
        "cleanup": cleanup,
        "clock": {"source": "monotonic_ns", "start": start_ns, "end": end_ns,
                  "duration": max(0, end_ns - start_ns)},
        "outer_executor": {"status": "IMPLEMENTED_NOT_RUNTIME_VALIDATED" if fake_runner else
                            "HOST_AUTHORIZED_SUBPROCESS",
                            "entrypoint": "apt_release_signature_executor.execute_signature_plan",
                            "source_sha256": _sha(EXECUTOR_PATH.read_bytes())},
        "canonical_sha256": "",
    }
    if not fake_runner:
        receipt["authorization"] = authorization
        receipt["nonce_claim"] = nonce_claim
        receipt["authorization_verification_time"] = authorization_verification_time
        receipt["authorization_machine"] = {
            "artifact": authorization_projection["machine_identity"],
            "live_host_match": authorization_projection["live_host_match"]}
    _assert_dir_identity(output_root, output_identity, "executor output root")
    receipt_path, receipt_sha = _receipt_write(output_root, receipt)
    validated = validate_executor_receipt(
        output_root, plan=plan, root_inputs=root, repository=repository,
        deb_bindings=deb_bindings)
    validated.update({"outcome": outcome, "receipt_path": str(receipt_path),
                      "receipt_sha256": receipt_sha})
    return validated


def _validate_reopened_plan(plan: Mapping[str, Any], root: Path,
                            repository: Mapping[str, Any],
                            deb_bindings: list[Mapping[str, Any]]) -> dict[str, Any]:
    """Reopen plan inputs after execution without requiring fresh output paths."""
    try:
        PLAN.validate_plan_static(plan)
    except (ValueError, OSError, KeyError, TypeError) as error:
        raise ExecutorError(f"plan static identity is invalid: {error}") from error
    scope = plan["scope"]
    trust_path = root / SIG._safe_relative(plan["trust_root"]["path"], "trust-root path")
    trust = SIG.validate_trust_root(root, trust_path)
    trust_data = SIG._regular(trust_path, "trust-root manifest", limit=SIG.MAX_JSON_BYTES)
    if plan["trust_root"]["file_sha256"] != _sha(trust_data) or \
            plan["trust_root"]["file_bytes"] != len(trust_data) or \
            plan["trust_root"]["canonical_sha256"] != trust["canonical_sha256"] or \
            plan["trust_root"]["verifier_binary_path"] != trust["verifier"]["binary_path"]:
        raise ExecutorError("plan trust-root projection drift")
    expected_tool = {"binary_path": trust["verifier"]["binary_path"],
                     "binary_sha256": trust["verifier"]["binary_sha256"],
                     "version": trust["verifier"]["version"],
                     "argv": [trust["verifier"]["binary_path"], "--version"],
                     "argv_sha256": _sha(_canonical(
                         [trust["verifier"]["binary_path"], "--version"]))}
    if plan.get("tool") != expected_tool:
        raise ExecutorError("plan tool preflight projection drift")
    release = root / SIG._safe_relative(scope["release_path"], "Release path")
    packages = root / SIG._safe_relative(scope["packages_path"], "Packages path")
    release_data = SIG._regular(release, "Release input", limit=SIG.MAX_TEXT_BYTES)
    packages_data = SIG._regular(packages, "Packages input", limit=SIG.MAX_TEXT_BYTES)
    if len(release_data) != scope["release_bytes"] or _sha(release_data) != scope["release_sha256"] or \
            len(packages_data) != scope["packages_bytes"] or _sha(packages_data) != scope["packages_sha256"]:
        raise ExecutorError("plan source projection drift")
    signature = root / SIG._safe_relative(scope["signature_path"], "signature path") \
        if scope["signature_path"] else None
    binding = SIG._binding(root, release, packages, repository, deb_bindings,
                           release_kind=scope["release_kind"], signature_path=signature)
    if binding["binding_identity_sha256"] != scope["binding_identity_sha256"]:
        raise ExecutorError("plan binding projection drift")
    executable = trust["verifier"]["binary_path"]
    expected_inventory = SIG.build_key_inventory_argv(
        homedir=Path(scope["homedir"]), keyring=root / trust["keyring_path"],
        executable=executable)
    expected_verify = SIG.build_verify_argv(
        homedir=Path(scope["homedir"]), keyring=root / trust["keyring_path"],
        release=release, release_kind=scope["release_kind"], signature=signature,
        executable=executable)
    workflow = plan["workflow"]
    if workflow[3]["argv"] != expected_inventory or workflow[4]["argv"] != expected_verify or \
            workflow[4].get("inventory_argv") != expected_inventory:
        raise ExecutorError("plan command projection drift")
    expected_outer = {"status": "IMPLEMENTED_NOT_RUNTIME_VALIDATED",
                      "entrypoint": "apt_release_signature_executor.execute_signature_plan",
                      "sha256": _sha(EXECUTOR_PATH.read_bytes())}
    if plan["outer_executor"] != expected_outer:
        raise ExecutorError("plan executor source identity drift")
    return {"trust": trust, "trust_path": trust_path, "release": release,
            "packages": packages, "signature": signature, "binding": binding,
            "executable": executable, "scope": scope,
            "expected_inventory": expected_inventory, "expected_verify": expected_verify}


def validate_executor_receipt(output_root: Path, *, plan: Mapping[str, Any],
                              root_inputs: Path, repository: Mapping[str, Any],
                              deb_bindings: list[Mapping[str, Any]]) -> dict[str, Any]:
    """Reopen a sealed executor receipt and revalidate all source projections."""
    root = SIG._checked_root(Path(root_inputs))
    projected = _validate_reopened_plan(plan, root, repository, deb_bindings)
    output_root = SIG._checked_root(Path(output_root), "executor output root")
    if Path(projected["scope"]["output_root"]) != output_root:
        raise ExecutorError("receipt output root is not plan-bound")
    path = output_root / "apt-release-signature.executor.receipt.json"
    sidecar = output_root / "apt-release-signature.executor.receipt.json.sha256"
    data = _bounded(path, "executor receipt", limit=MAX_RECEIPT_BYTES, allow_empty=False)
    side = _bounded(sidecar, "executor receipt sidecar", limit=512, allow_empty=False)
    if side.decode("ascii").strip().split() != [_sha(data), path.name]:
        raise ExecutorError("executor receipt sidecar mismatch")
    allowed_outputs = {path.name, sidecar.name, "tool-version.stdout", "tool-version.stderr",
                       "key-inventory.stdout", "key-inventory.stderr",
                       "signature-verify.stdout", "signature-verify.stderr"}
    for child in output_root.iterdir():
        info = child.lstat()
        if child.name not in allowed_outputs or stat.S_ISLNK(info.st_mode) or \
                not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise ExecutorError("executor output contains an undeclared or unsafe artifact")
    try:
        receipt = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ExecutorError(f"executor receipt is invalid JSON: {error}") from error
    base_required = {"schema", "schema_version", "status", "candidate_status", "benchmark_eligible",
                "signature_runtime", "execution_mode", "outcome", "promotion", "error",
                "plan_sha256", "output_root_identity", "input", "tool", "commands", "phases", "binding", "inventory",
                "signature", "cleanup", "clock", "outer_executor", "canonical_sha256"}
    if not isinstance(receipt, Mapping) or set(receipt) not in (
            base_required, base_required | {"authorization", "nonce_claim",
                                             "authorization_verification_time",
                                             "authorization_machine"}) or \
            receipt["schema"] != SCHEMA or receipt["schema_version"] != 1 or \
            receipt["status"] != "SEALED_EXECUTOR_RESULT" or receipt["outcome"] not in {"PASS", "FAIL"}:
        raise ExecutorError("executor receipt status is not exact non-promoting state")
    runtime_receipt = receipt["execution_mode"] == "HOST_AUTHORIZED_SUBPROCESS"
    if runtime_receipt:
        if set(receipt) != base_required | {"authorization", "nonce_claim",
                                            "authorization_verification_time",
                                            "authorization_machine"} or \
                receipt["candidate_status"] != "HOST_AUTHORIZED_SUBPROCESS" or \
                receipt["signature_runtime"] not in {"RUNTIME_VERIFIED", "RUNTIME_FAILED"} or \
                (receipt["outcome"] == "PASS" and
                 (receipt["benchmark_eligible"] is not True or
                  receipt["promotion"] != "HOST_AUTHORIZED_RUNTIME")) or \
                (receipt["outcome"] == "FAIL" and
                 (receipt["benchmark_eligible"] is not False or
                  receipt["promotion"] != "FORBIDDEN")):
            raise ExecutorError("authorized runtime receipt state is not exact")
    elif receipt["candidate_status"] != "IMPLEMENTED_NOT_RUNTIME_VALIDATED" or \
            receipt["benchmark_eligible"] is not False or receipt["signature_runtime"] != "NOT_RUN" or \
            receipt["execution_mode"] != "INJECTED_FIXTURE_ONLY" or \
            receipt["promotion"] != "FORBIDDEN":
        raise ExecutorError("executor receipt status is not exact non-promoting state")
    if (receipt["outcome"] == "PASS" and receipt["error"] != "") or \
            (receipt["outcome"] == "FAIL" and not receipt["error"]):
        raise ExecutorError("executor outcome/error projection is inconsistent")
    if receipt["canonical_sha256"] != SIG.canonical_hash(receipt, "canonical_sha256"):
        raise ExecutorError("executor receipt canonical identity drift")
    if receipt["plan_sha256"] != _sha(_canonical(plan)):
        raise ExecutorError("executor plan identity is invalid")
    if runtime_receipt:
        verification_time = receipt["authorization_verification_time"]
        if type(verification_time) is not int or verification_time < 0:
            raise ExecutorError("runtime authorization verification time is invalid")
        reopened_authorization = _verify_host_authorization(
            receipt["authorization"], plan=plan, root_inputs=root,
            repository=repository, deb_bindings=deb_bindings,
            verification_time=verification_time, require_live_machine=False)
        if receipt["authorization_machine"] != {
                "artifact": reopened_authorization["machine_identity"],
                "live_host_match": True} or reopened_authorization["live_host_match"] is not False:
            raise ExecutorError("sealed live machine verification evidence drift")
        claim = receipt["nonce_claim"]
        if not isinstance(claim, Mapping) or set(claim) != {
                "path", "bytes", "sha256", "parent_device", "parent_inode"}:
            raise ExecutorError("runtime receipt nonce claim is incomplete")
        claim_path = _safe_abs(claim["path"], "authorization nonce claim path")
        expected_token = _sha(_canonical({
            "nonce": receipt["authorization"]["nonce"],
            "payload_sha256": receipt["authorization"]["signature"]["payload_sha256"]}))
        expected_claim = output_root.parent / f".glim-r3-authorization-{expected_token}.claim"
        if claim_path != expected_claim:
            raise ExecutorError("runtime receipt nonce claim path drift")
        _parent_no_symlink(output_root, "runtime nonce claim parent")
        parent_info = output_root.parent.lstat()
        if stat.S_ISLNK(parent_info.st_mode) or not stat.S_ISDIR(parent_info.st_mode) or \
                claim["parent_device"] != parent_info.st_dev or \
                claim["parent_inode"] != parent_info.st_ino:
            raise ExecutorError("runtime receipt nonce claim parent identity drift")
        claim_data = _bounded(claim_path, "authorization nonce claim", limit=1024,
                              allow_empty=False)
        if claim["bytes"] != len(claim_data) or claim["sha256"] != _sha(claim_data):
            raise ExecutorError("runtime receipt nonce claim bytes drift")
        try:
            claim_document = json.loads(claim_data.decode("utf-8"))
        except (UnicodeError, json.JSONDecodeError) as error:
            raise ExecutorError(f"runtime receipt nonce claim is invalid: {error}") from error
        expected_document = {
            "schema": "glim_clean_room_r3_apt_signature_nonce_claim_v1",
            "nonce": receipt["authorization"]["nonce"],
            "payload_sha256": receipt["authorization"]["signature"]["payload_sha256"],
            "output_root": str(output_root),
            "verification_time": verification_time}
        if claim_document != expected_document:
            raise ExecutorError("runtime receipt nonce claim content drift")
        if receipt["authorization"]["issued_at"] > verification_time or \
                verification_time >= receipt["authorization"]["expires_at"]:
            raise ExecutorError("runtime authorization receipt time is outside signed validity")
    output_info = output_root.lstat()
    expected_output_identity = {"device": output_info.st_dev, "inode": output_info.st_ino,
                                "mode": stat.S_IMODE(output_info.st_mode)}
    if receipt["output_root_identity"] != expected_output_identity:
        raise ExecutorError("executor output-root identity drift")
    if set(receipt["output_root_identity"]) != {"device", "inode", "mode"} or \
            receipt["output_root_identity"]["mode"] != 0o700:
        raise ExecutorError("executor output-root identity is invalid")
    expected_input = {
        "root": str(root), "trust_root_path": plan["trust_root"]["path"],
        "trust_root_sha256": plan["trust_root"]["file_sha256"],
        "release_path": projected["scope"]["release_path"],
        "release_sha256": projected["scope"]["release_sha256"],
        "packages_path": projected["scope"]["packages_path"],
        "packages_sha256": projected["scope"]["packages_sha256"],
        "release_kind": projected["scope"]["release_kind"],
        "signature_path": projected["scope"]["signature_path"],
        "signature_sha256": projected["scope"]["signature_sha256"],
        "binding_identity_sha256": projected["binding"]["binding_identity_sha256"],
    }
    if receipt["input"] != expected_input:
        raise ExecutorError("executor input projection drift")
    executable = projected["executable"]
    tool = _checked_executable(executable, projected["trust"]["verifier"]["binary_sha256"])
    if not isinstance(receipt["tool"], Mapping) or set(receipt["tool"]) != {
            "path", "bytes", "sha256", "mode", "version", "version_line", "command",
            "command_sha256"} or receipt["tool"]["path"] != executable or \
            receipt["tool"]["bytes"] != tool["bytes"] or receipt["tool"]["sha256"] != tool["sha256"] or \
            receipt["tool"]["mode"] != tool["mode"] or \
            receipt["tool"]["version"] != projected["trust"]["verifier"]["version"] or \
            receipt["tool"]["command"] != [executable, "--version"] or \
            receipt["tool"]["command_sha256"] != _sha(_canonical([executable, "--version"])):
        raise ExecutorError("executor tool identity drift")
    expected_outer = {"status": "HOST_AUTHORIZED_SUBPROCESS" if runtime_receipt else
                      "IMPLEMENTED_NOT_RUNTIME_VALIDATED",
                      "entrypoint": "apt_release_signature_executor.execute_signature_plan",
                      "source_sha256": _sha(EXECUTOR_PATH.read_bytes())}
    if receipt["outer_executor"] != expected_outer:
        raise ExecutorError("executor source identity drift")
    phases = receipt["phases"]
    if not isinstance(phases, list) or len(phases) != len(PHASES) or \
            [row.get("phase") for row in phases] != list(PHASES) or \
            [row.get("ordinal") for row in phases] != list(range(len(PHASES))):
        raise ExecutorError("executor phase order is invalid")
    for row in phases:
        if row.get("status") not in {"PASS", "FAIL", "NOT_RUN"}:
            raise ExecutorError("executor phase status is invalid")
    homedir = Path(projected["scope"]["homedir"])
    if not isinstance(receipt["cleanup"], Mapping) or \
            set(receipt["cleanup"]) != {"requested", "removed", "post_absent", "identity", "error"} or \
            receipt["cleanup"].get("requested") is not True or \
            type(receipt["cleanup"].get("removed")) is not bool or \
            type(receipt["cleanup"].get("post_absent")) is not bool or \
            not isinstance(receipt["cleanup"].get("error"), str) or \
            len(receipt["cleanup"]["error"]) > 512:
        raise ExecutorError("executor cleanup did not prove homedir absence")
    cleanup_identity = receipt["cleanup"].get("identity")
    if cleanup_identity is not None and (not isinstance(cleanup_identity, Mapping) or
            set(cleanup_identity) != {"device", "inode", "mode"} or
            type(cleanup_identity["device"]) is not int or type(cleanup_identity["inode"]) is not int or
            cleanup_identity["device"] < 0 or cleanup_identity["inode"] <= 0 or
            cleanup_identity["mode"] != 0o700):
        raise ExecutorError("executor cleanup identity is invalid")
    actual_absent = not homedir.exists() and not homedir.is_symlink()
    if receipt["cleanup"]["post_absent"] != actual_absent:
        raise ExecutorError("executor cleanup absence claim does not match host state")
    if receipt["outcome"] == "PASS" and \
            (receipt["cleanup"]["removed"] is not True or
             receipt["cleanup"]["post_absent"] is not True or receipt["cleanup"]["error"] != ""):
        raise ExecutorError("executor PASS has incomplete cleanup evidence")
    clock = receipt["clock"]
    if not isinstance(clock, Mapping) or set(clock) != {"source", "start", "end", "duration"} or \
            clock["source"] != "monotonic_ns" or type(clock["start"]) is not int or \
            type(clock["end"]) is not int or type(clock["duration"]) is not int or \
            clock["start"] < 0 or clock["end"] < clock["start"] or \
            clock["duration"] != clock["end"] - clock["start"]:
        raise ExecutorError("executor clock evidence is invalid")
    if not isinstance(receipt["commands"], list) or not receipt["commands"]:
        raise ExecutorError("executor command evidence is empty")
    command_phases = [item.get("phase") for item in receipt["commands"]]
    if command_phases[0] != "tool_version" or command_phases[1:] not in (
            [], ["key-inventory"], ["key-inventory", "signature-verify"]):
        raise ExecutorError("executor command order is invalid")
    if receipt["outcome"] == "PASS" and command_phases != [
            "tool_version", "key-inventory", "signature-verify"]:
        raise ExecutorError("executor PASS is missing a command")
    expected_commands = {
        "tool_version": [executable, "--version"],
        "key-inventory": projected["expected_inventory"],
        "signature-verify": projected["expected_verify"],
    }
    expected_env = _fixed_env(homedir)
    if receipt["outcome"] == "PASS":
        if any(row["status"] != "PASS" for row in phases):
            raise ExecutorError("executor PASS has incomplete phase evidence")
        if receipt["binding"] is None or receipt["signature"] is None or receipt["inventory"] is None:
            raise ExecutorError("executor PASS is missing verified outputs")
        if any(command.get("returncode") != 0 or command.get("timed_out") or
               command.get("signal") is not None for command in receipt["commands"]):
            raise ExecutorError("executor PASS has unsuccessful command evidence")
    for command in receipt["commands"]:
        if not isinstance(command, Mapping) or command.get("shell") is not False or \
                command.get("stdin") != "DEVNULL" or command.get("argv_sha256") != \
                _sha(_canonical(command.get("argv"))) or \
                command.get("argv") != expected_commands.get(command.get("phase")) or \
                command.get("env") != expected_env or \
                command.get("env_sha256") != _sha(_canonical(expected_env)) or \
                command.get("cwd") != str(root) or command.get("timeout_seconds") != COMMAND_TIMEOUT_SECONDS or \
                command.get("resource_limits") != RESOURCE_LIMITS:
            raise ExecutorError("executor command evidence is not canonical")
        for key in ("stdout", "stderr"):
            log = command.get(key)
            if not isinstance(log, Mapping) or set(log) != {"path", "bytes", "sha256"}:
                raise ExecutorError("executor command log identity is incomplete")
            log_path = output_root / SIG._safe_relative(log["path"], "executor log path")
            actual = _identity(log_path, output_root, "executor command log")
            if actual != dict(log):
                raise ExecutorError("executor command log bytes/SHA drift")
    version_command = next(command for command in receipt["commands"]
                           if command["phase"] == "tool_version")
    version_data = _bounded(output_root / version_command["stdout"]["path"],
                            "tool version stdout", allow_empty=False)
    version_line = _version(version_data, projected["trust"]["verifier"]["version"])
    if receipt["outcome"] == "PASS" and receipt["tool"]["version_line"] != version_line:
        raise ExecutorError("tool version evidence drift")
    if receipt["outcome"] == "PASS" and (version_command["returncode"] != 0 or
                                          version_command["timed_out"] or
                                          version_command["signal"] is not None):
        raise ExecutorError("tool version evidence drift")
    inventory_command = next((command for command in receipt["commands"]
                              if command["phase"] == "key-inventory"), None)
    verify_command = next((command for command in receipt["commands"]
                           if command["phase"] == "signature-verify"), None)
    if inventory_command is not None and inventory_command["returncode"] == 0 and \
            not inventory_command["timed_out"] and inventory_command["signal"] is None:
        inventory_data = _bounded(output_root / inventory_command["stdout"]["path"],
                                  "key inventory output", allow_empty=False)
        inventory = SIG.validate_key_inventory_output(inventory_data, projected["trust"])
        if receipt["inventory"] != inventory:
            raise ExecutorError("executor key inventory projection drift")
    if verify_command is not None and verify_command["returncode"] == 0 and \
            not verify_command["timed_out"] and verify_command["signal"] is None:
        status_data = _bounded(output_root / verify_command["stdout"]["path"],
                               "signature status output", allow_empty=False)
        status = SIG.validate_status_output(status_data, projected["trust"])
        if receipt["signature"] != status:
            raise ExecutorError("executor signature projection drift")
    if receipt["outcome"] == "PASS" and receipt["binding"] != projected["binding"]:
        raise ExecutorError("executor binding projection drift")
    return {"status": "PASS", "outcome": receipt["outcome"],
            "receipt_sha256": _sha(data),
            "benchmark_eligible": receipt["benchmark_eligible"],
            "execution_mode": receipt["execution_mode"],
            "signature_runtime": receipt["signature_runtime"]}


def validate_executor_receipt_static(output_root: Path) -> dict[str, Any]:
    """Classify a receipt without source/plan revalidation; never runtime-ready."""
    output_root = SIG._checked_root(Path(output_root), "executor output root")
    path = output_root / "apt-release-signature.executor.receipt.json"
    data = _bounded(path, "executor receipt", limit=MAX_RECEIPT_BYTES, allow_empty=False)
    try:
        receipt = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ExecutorError(f"executor receipt is invalid JSON: {error}") from error
    if not isinstance(receipt, Mapping) or receipt.get("schema") != SCHEMA or \
            receipt.get("benchmark_eligible") is not False or \
            receipt.get("candidate_status") != "IMPLEMENTED_NOT_RUNTIME_VALIDATED":
        raise ExecutorError("receipt is not an explicit non-promoting candidate")
    if receipt.get("canonical_sha256") != SIG.canonical_hash(receipt, "canonical_sha256"):
        raise ExecutorError("receipt canonical identity drift")
    return {"status": "NOT_RUNTIME_VALIDATED", "benchmark_eligible": False,
            "receipt_sha256": _sha(data)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--help-contract", action="store_true")
    args = parser.parse_args()
    if args.help_contract:
        print(__doc__)
        return 0
    parser.error("runtime signature execution requires explicit injected API inputs")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
