#!/usr/bin/env python3
"""Plan and validate the one exact offline Docker build command for r3.

The planner is deliberately non-executing.  It accepts only a fresh build
context whose dependency directory was already verified by the r3 prefetch
tool, and it emits an identity binding the candidate, manifest, recipe, tool,
prefetch receipt, and exact argv.  A separate operator may later execute this
argv after reviewing the sealed prefetch receipt.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[1]
CANDIDATE_ROOT = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"
MANIFEST_NAME = "candidate_manifest.json"
OFFLINE_MANIFEST_NAME = "offline_dependency_manifest.json"
RECIPE_NAME = "Dockerfile"
PREFETCH_TOOL_NAME = "dependency_prefetch.py"
PREFETCH_RECEIPT_NAME = "prefetch.receipt.json"
MAX_FILE_BYTES = 8 * 1024 * 1024
SHA_RE = re.compile(r"^[0-9a-f]{64}$")
TAG_RE = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,127}(?::[a-z0-9][a-z0-9_.-]{0,127})?$")


class OfflineBuildError(ValueError):
    """Raised when an offline build plan cannot be proven exact."""


def sha256_file(path: Path, *, max_bytes: int = MAX_FILE_BYTES) -> str:
    _regular(path, "file", max_bytes=max_bytes)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular(path: Path, label: str, *, max_bytes: int) -> None:
    if path.is_symlink() or not path.is_file():
        raise OfflineBuildError(f"{label} is not a regular file: {path}")
    stat_result = path.stat()
    if stat_result.st_nlink != 1:
        raise OfflineBuildError(f"{label} is a hardlink or shared inode: {path}")
    if stat_result.st_size <= 0 or stat_result.st_size > max_bytes:
        raise OfflineBuildError(f"{label} has invalid size: {path}")


def _read_json(path: Path) -> dict[str, Any]:
    _regular(path, "JSON", max_bytes=MAX_FILE_BYTES)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise OfflineBuildError(f"invalid JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise OfflineBuildError(f"JSON root is not an object: {path}")
    return value


def _canonical(value: Mapping[str, Any]) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode(
        "utf-8")


def _context_file(context: Path, relative: str) -> Path:
    candidate = context / relative
    try:
        candidate.relative_to(context)
    except ValueError as error:
        raise OfflineBuildError(f"context path escapes root: {relative}") from error
    return candidate


def _validate_prefetch_receipt(context: Path) -> tuple[Path, dict[str, Any]]:
    receipt_path = _context_file(context, f"prefetch/{PREFETCH_RECEIPT_NAME}")
    receipt = _read_json(receipt_path)
    if receipt.get("receipt_kind") != "glim_clean_room_r3_prefetch_receipt_v1" or \
            receipt.get("status") != "PASS" or receipt.get("network_used") is not False:
        raise OfflineBuildError("prefetch receipt is not an offline PASS")
    if not isinstance(receipt.get("manifest_sha256"), str) or not SHA_RE.fullmatch(
            receipt["manifest_sha256"]):
        raise OfflineBuildError("prefetch receipt manifest identity is invalid")
    return receipt_path, receipt


def _verify_prefetch_bytes(context: Path, offline_path: Path) -> dict[str, Any]:
    tool_path = CANDIDATE_ROOT / "dependency_prefetch.py"
    spec = importlib.util.spec_from_file_location("glim_r3_prefetch_for_plan", tool_path)
    if spec is None or spec.loader is None:
        raise OfflineBuildError("offline prefetch verifier cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    try:
        return module.verify_prefetch_root(context / "prefetch", offline_path)
    except (OSError, ValueError, TypeError) as error:
        raise OfflineBuildError(f"prefetch bytes are not independently verified: {error}") from error


def validate_context(context: Path) -> dict[str, Any]:
    if context.is_symlink() or not context.is_dir():
        raise OfflineBuildError(f"build context is not a regular directory: {context}")
    required = {
        "phase3d/r3/candidate_manifest.json",
        "phase3d/r3/offline_dependency_manifest.json",
        "phase3d/r3/Dockerfile",
        "phase3d/r3/dependency_prefetch.py",
        "prefetch/prefetch.receipt.json",
    }
    for relative in sorted(required):
        _regular(_context_file(context, relative), "build context input",
                 max_bytes=MAX_FILE_BYTES)
    manifest_path = _context_file(context, "phase3d/r3/candidate_manifest.json")
    offline_path = _context_file(context, "phase3d/r3/offline_dependency_manifest.json")
    manifest = _read_json(manifest_path)
    offline = _read_json(offline_path)
    if manifest.get("status") != "OPT_IN_NOT_READY" or \
            manifest.get("benchmark_eligible") is not False:
        raise OfflineBuildError("candidate build context is not opt-in/not-ready")
    if offline.get("status") != "READY" or \
            offline.get("apt_deb_closure", {}).get("status") != "READY":
        raise OfflineBuildError("offline dependency manifest is not reviewed READY")
    offline_sha = sha256_file(offline_path)
    closure_binding = manifest.get("offline_dependency_closure", {})
    if closure_binding.get("manifest_sha256") != offline_sha:
        raise OfflineBuildError("candidate and offline dependency manifest are mixed")
    receipt_path, receipt = _validate_prefetch_receipt(context)
    if receipt.get("manifest_sha256") != offline_sha:
        raise OfflineBuildError("prefetch receipt is bound to another dependency manifest")
    _verify_prefetch_bytes(context, offline_path)
    recipe = _context_file(context, "phase3d/r3/Dockerfile")
    tool = _context_file(context, "phase3d/r3/dependency_prefetch.py")
    build_contract = manifest.get("offline_build_contract", {})
    if build_contract.get("recipe_sha256") != sha256_file(recipe) or \
            build_contract.get("prefetch_tool_sha256") != sha256_file(tool) or \
            build_contract.get("planner_sha256") != sha256_file(
                ROOT / "scripts/plan_glim_clean_room_r3_offline_build.py"):
        raise OfflineBuildError("candidate recipe/tool/planner identity drift")
    recipe_text = recipe.read_text(encoding="utf-8")
    for forbidden in ("ADD https://", "apt-get", "curl ", "wget ", "--network host",
                      "--privileged", "latest"):
        if forbidden.lower() in recipe_text.lower():
            raise OfflineBuildError(f"network/package-manager recipe operation: {forbidden}")
    if "COPY prefetch/" not in recipe_text or "--network" not in recipe_text:
        raise OfflineBuildError("recipe does not consume the verified prefetch contract")
    return {
        "manifest": manifest,
        "offline_manifest": offline,
        "manifest_path": manifest_path,
        "offline_manifest_path": offline_path,
        "recipe_path": recipe,
        "prefetch_tool_path": tool,
        "prefetch_receipt_path": receipt_path,
        "prefetch_receipt": receipt,
    }


def exact_build_argv(context: Path, tag: str) -> list[str]:
    if not TAG_RE.fullmatch(tag):
        raise OfflineBuildError(f"unsafe image tag: {tag}")
    validate_context(context)
    dockerfile = _context_file(context, "phase3d/r3/Dockerfile")
    return [
        "docker", "build", "--network", "none", "--pull=false",
        "--progress=plain", "--build-arg", "JOBS=2", "-f", str(dockerfile),
        "-t", tag, str(context),
    ]


def validate_exact_build_argv(argv: list[str], *, context: Path, tag: str) -> None:
    expected = exact_build_argv(context, tag)
    if argv != expected:
        raise OfflineBuildError("Docker argv is not the exact offline build command")
    if any(token in {"--privileged", "--network=host", "host", "--pull"} for token in argv):
        raise OfflineBuildError("unsafe network or privilege token in Docker argv")


def plan(context: Path, tag: str) -> dict[str, Any]:
    checked = validate_context(context)
    argv = exact_build_argv(context, tag)
    identity = {
        "candidate_id": checked["manifest"]["candidate_id"],
        "candidate_manifest_sha256": sha256_file(checked["manifest_path"]),
        "offline_dependency_manifest_sha256": sha256_file(
            checked["offline_manifest_path"]),
        "recipe_sha256": sha256_file(checked["recipe_path"]),
        "prefetch_tool_sha256": sha256_file(checked["prefetch_tool_path"]),
        "prefetch_receipt_sha256": sha256_file(checked["prefetch_receipt_path"]),
        "build_argv_sha256": hashlib.sha256(_canonical({"argv": argv})).hexdigest(),
        "network": "none",
        "pull": False,
        "benchmark_eligible": False,
    }
    return {
        "schema_version": 1,
        "plan_kind": "glim_clean_room_r3_offline_build_plan_v1",
        "status": "PASS",
        "execute": False,
        "argv": argv,
        "identity": identity,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--context", type=Path, required=True)
    parser.add_argument("--tag", required=True)
    parser.add_argument("--json", action="store_true")
    return parser


def main() -> int:
    args = _parser().parse_args()
    try:
        result = plan(args.context, args.tag)
    except (OSError, OfflineBuildError, TypeError, ValueError) as error:
        result = {"status": "FAIL_CLOSED", "reason": str(error)}
        print(json.dumps(result, sort_keys=True))
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
