#!/usr/bin/env python3
"""Plan (but never execute) the r3 apt Release signature verification contract.

The planner emits bounded argv and immutable input identities for a future
host-owned executor.  It deliberately has no subprocess, filesystem-writing,
network, Docker, apt, or GnuPG execution path.  A plan is therefore
``OPT_IN_NOT_READY`` until an outer host receipt proves the same phases.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[1]
R3 = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"
MODULE_PATH = R3 / "apt_release_signature.py"
EXECUTOR_PATH = R3 / "apt_release_signature_executor.py"
HOST_PHASES = ("trust_root_reopen", "release_reopen", "homedir_freshness",
              "key_inventory", "signature_verify", "binding_reopen",
              "seal_receipt", "cleanup")


class SignaturePlanError(ValueError):
    """Raised for an unsafe or incomplete signature verification plan."""


def _module():
    spec = importlib.util.spec_from_file_location("r3_apt_release_signature", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise SignaturePlanError("signature contract module cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _canonical(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")


def _sha_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _relative(root: Path, path: Path, label: str, limit: int) -> tuple[str, bytes]:
    module = _module()
    return module._input_relative(root, path, label, limit=limit)


def _sha_file(path: Path, label: str) -> str:
    module = _module()
    return _sha_bytes(module._regular(path, label, limit=module.MAX_JSON_BYTES))


def _executor_projection() -> dict[str, str]:
    return {"status": "IMPLEMENTED_NOT_RUNTIME_VALIDATED",
            "entrypoint": "apt_release_signature_executor.execute_signature_plan",
            "sha256": _sha_file(EXECUTOR_PATH, "signature executor source")}


def build_plan(*, root: Path, trust_root_path: Path, release_path: Path,
               packages_path: Path, repository: Mapping[str, Any],
               deb_bindings: list[Mapping[str, Any]], homedir: Path,
               release_kind: str, signature_path: Path | None,
               output_root: Path) -> dict[str, Any]:
    """Build a deterministic, non-executable plan from already sealed bytes."""
    module = _module()
    root = _module()._checked_root(Path(root))
    trust_root_path = Path(trust_root_path)
    trust = module.validate_trust_root(root, trust_root_path)
    trust_relative, trust_data = _relative(root, trust_root_path, "trust-root manifest",
                                            module.MAX_JSON_BYTES)
    release_relative, release_data = _relative(root, release_path, "Release/InRelease",
                                                module.MAX_TEXT_BYTES)
    packages_relative, packages_data = _relative(root, packages_path, "Packages index",
                                                 module.MAX_TEXT_BYTES)
    signature_relative = ""
    signature_data = b""
    if signature_path is not None:
        signature_relative, signature_data = _relative(
            root, signature_path, "detached Release signature", module.MAX_TEXT_BYTES)
    if not Path(output_root).is_absolute() or Path(output_root).exists() or \
            Path(output_root).is_symlink() or not Path(output_root).parent.is_dir() or \
            Path(output_root).parent.is_symlink():
        raise SignaturePlanError("signature output root must be a fresh absolute path")
    homedir = Path(homedir)
    if not homedir.is_absolute() or homedir.exists() or homedir.is_symlink() or \
            not homedir.parent.is_dir() or homedir.parent.is_symlink():
        raise SignaturePlanError("GnuPG homedir must be a fresh absolute path")
    binding = module._binding(root, Path(release_path), Path(packages_path), repository,
                              deb_bindings, release_kind=release_kind,
                              signature_path=signature_path)
    command = module.build_verify_argv(
        homedir=homedir, keyring=root / trust["keyring_path"], release=release_path,
        release_kind=release_kind, signature=signature_path,
        executable=trust["verifier"]["binary_path"])
    inventory_command = module.build_key_inventory_argv(
        homedir=homedir, keyring=root / trust["keyring_path"],
        executable=trust["verifier"]["binary_path"])
    workflow = [
        {"ordinal": 0, "phase": "trust_root_reopen",
         "argv": ["host-contract", "reopen-trust-root", trust_relative],
         "argv_sha256": _sha_bytes(_canonical(["host-contract", "reopen-trust-root",
                                                trust_relative])),
         "expected": "NOT_RUN", "network": "DISALLOWED"},
        {"ordinal": 1, "phase": "release_reopen",
         "argv": ["host-contract", "reopen-release", release_relative,
                   packages_relative],
         "argv_sha256": _sha_bytes(_canonical(["host-contract", "reopen-release",
                                                release_relative, packages_relative])),
         "expected": "NOT_RUN", "network": "DISALLOWED"},
        {"ordinal": 2, "phase": "homedir_freshness",
         "argv": ["host-contract", "require-fresh-directory", str(homedir.resolve())],
         "argv_sha256": _sha_bytes(_canonical(["host-contract", "require-fresh-directory",
                                                str(homedir.resolve())])),
         "expected": "NOT_RUN", "network": "DISALLOWED"},
        {"ordinal": 3, "phase": "key_inventory", "argv": inventory_command,
         "argv_sha256": _sha_bytes(_canonical(inventory_command)),
         "expected": "NOT_RUN", "network": "DISALLOWED"},
        {"ordinal": 4, "phase": "signature_verify", "argv": command,
         "argv_sha256": _sha_bytes(_canonical(command)), "expected": "NOT_RUN",
         "network": "DISALLOWED", "inventory_argv": inventory_command,
         "inventory_argv_sha256": _sha_bytes(_canonical(inventory_command))},
        {"ordinal": 5, "phase": "binding_reopen",
         "argv": ["host-contract", "reopen-release-packages-debs",
                   binding["binding_identity_sha256"]],
         "argv_sha256": _sha_bytes(_canonical(["host-contract",
                                                "reopen-release-packages-debs",
                                                binding["binding_identity_sha256"]])),
         "expected": "NOT_RUN", "network": "DISALLOWED"},
        {"ordinal": 6, "phase": "seal_receipt",
         "argv": ["host-contract", "seal-signature-receipt", str(Path(output_root).resolve())],
         "argv_sha256": _sha_bytes(_canonical(["host-contract", "seal-signature-receipt",
                                                str(Path(output_root).resolve())])),
         "expected": "NOT_RUN", "network": "DISALLOWED"},
        {"ordinal": 7, "phase": "cleanup",
         "argv": ["host-contract", "cleanup-verification-state", "--policy",
                   "never_runtime"],
         "argv_sha256": _sha_bytes(_canonical(["host-contract",
                                                "cleanup-verification-state", "--policy",
                                                "never_runtime"])),
         "expected": "NOT_RUN", "network": "DISALLOWED"},
    ]
    phases = [{"ordinal": item["ordinal"], "phase": item["phase"], "status": "NOT_RUN"}
              for item in workflow]
    plan = {
        "schema": module.PLAN_SCHEMA, "schema_version": 1,
        "status": "OPT_IN_NOT_READY", "benchmark_eligible": False,
        "execution": "NOT_RUN", "signature_runtime": "NOT_RUN",
        "trust_root": {"path": trust_relative, "file_bytes": len(trust_data),
                       "file_sha256": _sha_bytes(trust_data),
                       "canonical_sha256": trust["canonical_sha256"],
                       "key_id": trust["key_id"], "fingerprint": trust["fingerprint"],
                       "keyring_sha256": trust["keyring_sha256"],
                       "keyring_path": trust["keyring_path"],
                       "verifier_binary_path": trust["verifier"]["binary_path"],
                       "verifier_binary_sha256": trust["verifier"]["binary_sha256"],
                       "verification_time": trust["verification_time"]},
        "tool": {"binary_path": trust["verifier"]["binary_path"],
                 "binary_sha256": trust["verifier"]["binary_sha256"],
                 "version": trust["verifier"]["version"],
                 "argv": [trust["verifier"]["binary_path"], "--version"],
                 "argv_sha256": _sha_bytes(_canonical(
                     [trust["verifier"]["binary_path"], "--version"]))},
        "scope": {"release_kind": release_kind, "release_path": release_relative,
                  "release_bytes": len(release_data), "release_sha256": _sha_bytes(release_data),
                  "packages_path": packages_relative, "packages_bytes": len(packages_data),
                  "packages_sha256": _sha_bytes(packages_data),
                  "signature_path": signature_relative,
                  "signature_bytes": len(signature_data),
                  "signature_sha256": _sha_bytes(signature_data) if signature_data else "",
                  "binding_identity_sha256": binding["binding_identity_sha256"],
                  "homedir": str(homedir.resolve()),
                  "output_root": str(Path(output_root).resolve())},
        "workflow": workflow, "phases": phases,
        "outer_executor": _executor_projection(),
        "review_gate": {"custodian_signature_required": True,
                         "network_execution": "NOT_RUN", "collector_input_allowed": False},
        "canonical_sha256": "",
    }
    plan["canonical_sha256"] = _sha_bytes(_canonical({key: value for key, value in plan.items()
                                                       if key != "canonical_sha256"}))
    return plan


def validate_plan(plan: Mapping[str, Any], *, root: Path | None = None,
                  repository: Mapping[str, Any] | None = None,
                  deb_bindings: list[Mapping[str, Any]] | None = None) -> dict[str, Any]:
    if root is None or repository is None or deb_bindings is None:
        raise SignaturePlanError(
            "full signature-plan validation requires root, repository, and deb bindings; "
            "static plan checks are NOT_RUNTIME_VALIDATED")
    module = _module()
    if not isinstance(plan, Mapping) or plan.get("schema") != module.PLAN_SCHEMA or \
            plan.get("schema_version") != 1 or plan.get("status") != "OPT_IN_NOT_READY" or \
            plan.get("benchmark_eligible") is not False or plan.get("execution") != "NOT_RUN" or \
            plan.get("signature_runtime") != "NOT_RUN":
        raise SignaturePlanError("signature plan is not explicitly non-promoting")
    if plan.get("outer_executor") != _executor_projection():
        raise SignaturePlanError("signature plan executor identity drift")
    workflow = plan.get("workflow")
    phases = plan.get("phases")
    if not isinstance(workflow, list) or [item.get("phase") for item in workflow] != list(HOST_PHASES) or \
            not isinstance(phases, list) or [item.get("phase") for item in phases] != list(HOST_PHASES):
        raise SignaturePlanError("signature plan phases are missing or reordered")
    for ordinal, item in enumerate(workflow):
        if item.get("ordinal") != ordinal or item.get("expected") != "NOT_RUN" or \
                item.get("network") != "DISALLOWED" or not isinstance(item.get("argv"), list) or \
                item.get("argv_sha256") != _sha_bytes(_canonical(item["argv"])):
            raise SignaturePlanError("signature plan workflow identity drift")
        if any(token in {"sh", "bash", "-c", "-lc", "curl", "wget", "nc", "socat"}
               for token in item["argv"]):
            raise SignaturePlanError("signature plan contains a shell/network token")
    for ordinal, item in enumerate(phases):
        if item != {"ordinal": ordinal, "phase": HOST_PHASES[ordinal], "status": "NOT_RUN"}:
            raise SignaturePlanError("signature plan phase status is not NOT_RUN")
    inventory_item = workflow[3]
    signature_item = workflow[4]
    command = signature_item.get("argv")
    inventory_command = inventory_item.get("argv")
    if not isinstance(command, list) or not isinstance(inventory_command, list):
        raise SignaturePlanError("signature plan verifier argv is absent")
    scope = plan.get("scope")
    trust_projection = plan.get("trust_root")
    tool_projection = plan.get("tool")
    if not isinstance(scope, Mapping) or not isinstance(trust_projection, Mapping) or \
            not isinstance(tool_projection, Mapping):
        raise SignaturePlanError("signature plan scope/trust projection is absent")
    if set(tool_projection) != {"binary_path", "binary_sha256", "version", "argv", "argv_sha256"} or \
            tool_projection["binary_path"] != trust_projection["verifier_binary_path"] or \
            tool_projection["binary_sha256"] != trust_projection["verifier_binary_sha256"] or \
            tool_projection["argv"] != [tool_projection["binary_path"], "--version"] or \
            tool_projection["argv_sha256"] != _sha_bytes(_canonical(tool_projection["argv"] )):
        raise SignaturePlanError("signature plan tool preflight projection drift")
    expected_executable = trust_projection["verifier_binary_path"]
    if command[:1] != [expected_executable] or "--no-default-keyring" not in command or \
            "--no-auto-key-retrieve" not in command or "--status-fd" not in command:
        raise SignaturePlanError("signature plan verifier argv is not the fixed GnuPG contract")
    if inventory_command[:1] != [expected_executable] or "--no-default-keyring" not in inventory_command or \
            "--with-colons" not in inventory_command or "--with-subkey-fingerprint" not in inventory_command:
        raise SignaturePlanError("signature plan inventory argv is not the fixed GnuPG contract")
    if signature_item.get("inventory_argv") != inventory_command or \
            signature_item.get("inventory_argv_sha256") != _sha_bytes(_canonical(inventory_command)):
        raise SignaturePlanError("signature plan inventory command cross-binding drift")
    try:
        keyring_index = command.index("--keyring") + 1
        keyring_absolute = command[keyring_index]
        keyring_suffix = "/" + trust_projection["keyring_path"]
        if not isinstance(keyring_absolute, str) or not keyring_absolute.endswith(keyring_suffix):
            raise SignaturePlanError("signature plan keyring path is not trust-root bound")
        input_prefix = keyring_absolute[:-len(keyring_suffix)]
        expected_release_path = input_prefix + "/" + scope["release_path"]
        verify_index = command.index("--verify")
        if command[command.index("--homedir") + 1] != scope["homedir"] or \
                ((scope["release_kind"] == "INRELEASE" and
                  command[verify_index + 1] != expected_release_path) or
                 (scope["release_kind"] == "RELEASE_DETACHED" and
                  command[verify_index + 2] != expected_release_path)):
            raise SignaturePlanError("signature plan Release/homedir argv binding drift")
        if inventory_command[inventory_command.index("--homedir") + 1] != scope["homedir"] or \
                inventory_command[-1] != keyring_absolute:
            raise SignaturePlanError("signature plan key inventory argv binding drift")
        if workflow[2]["argv"][-1] != scope["homedir"] or \
                workflow[6]["argv"][-1] != scope["output_root"]:
            raise SignaturePlanError("signature plan freshness/seal path drift")
    except (IndexError, ValueError, KeyError, TypeError) as error:
        raise SignaturePlanError("signature plan argv projection is malformed") from error
    if root is not None:
        root = Path(root).resolve()
        trust_path = root / plan["trust_root"]["path"]
        trust = module.validate_trust_root(root, trust_path)
        if _sha_file(trust_path, "signature plan trust root") != plan["trust_root"]["file_sha256"] or \
                trust["canonical_sha256"] != plan["trust_root"]["canonical_sha256"] or \
                trust["keyring_sha256"] != plan["trust_root"]["keyring_sha256"] or \
                trust["keyring_path"] != plan["trust_root"]["keyring_path"] or \
                trust["verifier"]["binary_sha256"] != plan["trust_root"]["verifier_binary_sha256"]:
            raise SignaturePlanError("signature plan trust-root bytes/identity drift")
        if plan["tool"] != {
                "binary_path": trust["verifier"]["binary_path"],
                "binary_sha256": trust["verifier"]["binary_sha256"],
                "version": trust["verifier"]["version"],
                "argv": [trust["verifier"]["binary_path"], "--version"],
                "argv_sha256": _sha_bytes(_canonical(
                    [trust["verifier"]["binary_path"], "--version"]))}:
            raise SignaturePlanError("signature plan tool source identity drift")
        release = root / scope["release_path"]
        packages = root / scope["packages_path"]
        release_data = module._regular(release, "signature plan Release", limit=module.MAX_TEXT_BYTES)
        packages_data = module._regular(packages, "signature plan Packages", limit=module.MAX_TEXT_BYTES)
        if len(release_data) != scope["release_bytes"] or _sha_bytes(release_data) != scope["release_sha256"] or \
                len(packages_data) != scope["packages_bytes"] or _sha_bytes(packages_data) != scope["packages_sha256"]:
            raise SignaturePlanError("signature plan source bytes/identity drift")
        executable = trust["verifier"]["binary_path"]
        expected_command = module.build_verify_argv(
            homedir=Path(scope["homedir"]), keyring=root / trust["keyring_path"],
            release=release, release_kind=scope["release_kind"],
            signature=(root / scope["signature_path"]) if scope["signature_path"] else None,
            executable=executable)
        try:
            module.validate_verify_argv(command, expected_command)
        except (ValueError, TypeError) as error:
            raise SignaturePlanError("signature plan verifier argv does not match source scope") from error
        expected_inventory = module.build_key_inventory_argv(
            homedir=Path(scope["homedir"]), keyring=root / trust["keyring_path"],
            executable=executable)
        try:
            module.validate_key_inventory_argv(inventory_command, expected_inventory)
        except (ValueError, TypeError) as error:
            raise SignaturePlanError("signature plan key inventory argv does not match source scope") from error
        if repository is not None and deb_bindings is not None:
            binding = module._binding(
                root, release, packages, repository, deb_bindings,
                release_kind=scope["release_kind"],
                signature_path=(root / scope["signature_path"]) if scope["signature_path"] else None)
            if binding["binding_identity_sha256"] != scope["binding_identity_sha256"]:
                raise SignaturePlanError("signature plan Release/Packages/deb binding drift")
        output_root = Path(scope["output_root"])
        if output_root.exists() or output_root.is_symlink() or output_root.parent.is_symlink():
            raise SignaturePlanError("signature plan output root is not fresh")
    if plan.get("canonical_sha256") != _sha_bytes(_canonical(
            {key: value for key, value in plan.items() if key != "canonical_sha256"})):
        raise SignaturePlanError("signature plan canonical identity drift")
    return {"status": "PASS", "benchmark_eligible": False,
            "plan_sha256": _sha_bytes(_canonical(plan))}


def validate_plan_static(plan: Mapping[str, Any]) -> dict[str, Any]:
    """Only classify a sealed plan without claiming source revalidation."""
    module = _module()
    if not isinstance(plan, Mapping) or plan.get("schema") != module.PLAN_SCHEMA or \
            plan.get("schema_version") != 1 or plan.get("status") != "OPT_IN_NOT_READY" or \
            plan.get("benchmark_eligible") is not False or plan.get("execution") != "NOT_RUN":
        raise SignaturePlanError("static plan status is not explicitly non-promoting")
    if plan.get("outer_executor") != _executor_projection():
        raise SignaturePlanError("static plan executor identity drift")
    tool = plan.get("tool")
    trust = plan.get("trust_root")
    if not isinstance(tool, Mapping) or not isinstance(trust, Mapping) or \
            set(tool) != {"binary_path", "binary_sha256", "version", "argv", "argv_sha256"} or \
            tool.get("binary_path") != trust.get("verifier_binary_path") or \
            tool.get("binary_sha256") != trust.get("verifier_binary_sha256") or \
            tool.get("argv") != [tool.get("binary_path"), "--version"] or \
            tool.get("argv_sha256") != _sha_bytes(_canonical(tool.get("argv"))):
        raise SignaturePlanError("static plan tool preflight projection drift")
    if plan.get("canonical_sha256") != _sha_bytes(_canonical(
            {key: value for key, value in plan.items() if key != "canonical_sha256"})):
        raise SignaturePlanError("static plan canonical identity drift")
    return {"status": "NOT_RUNTIME_VALIDATED", "benchmark_eligible": False,
            "plan_sha256": _sha_bytes(_canonical(plan))}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--help-contract", action="store_true")
    args = parser.parse_args()
    if args.help_contract:
        print(__doc__)
        return 0
    parser.error("runtime signature planning requires explicit fixture API inputs")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
