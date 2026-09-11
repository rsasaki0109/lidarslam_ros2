#!/usr/bin/env python3
"""Verify the host-owned boundary around the contract-only apt capture.

This module does not execute Docker or inspect a daemon.  It reopens bounded
host logs, parses the image and post-disconnect inspect records, verifies the
fixed planner workflow and cleanup evidence, and optionally reopens a nested
capture root.  The capture producer itself is deliberately marked
``CONTRACT_ONLY`` until a real in-container collector exists.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import stat
from typing import Any, Mapping


SCHEMA = "glim_clean_room_r3_apt_outer_receipt_v1"
SCHEMA_VERSION = 1
OUTER_RECEIPT = "apt-closure.outer.receipt.json"
MAX_LOG_BYTES = 4 * 1024 * 1024
MAX_RECEIPT_BYTES = 8 * 1024 * 1024
SHA_RE = re.compile(r"^[0-9a-f]{64}$")
IMAGE_RE = re.compile(r"^ros@sha256:[0-9a-f]{64}$")
DIGEST_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
ROOT = Path(__file__).resolve().parents[5]
PLANNER_PATH = ROOT / "scripts/plan_glim_clean_room_r3_apt_capture.py"
SCHEMA_PATH = Path(__file__).with_name("apt_closure_outer.schema.json")
CAPTURE_PATH = Path(__file__).with_name("apt_closure_capture.py")
CAPTURE_SCHEMA_PATH = Path(__file__).with_name("apt_closure_capture.schema.json")
PINNED_BASE_IMAGE = "ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f"


class OuterCaptureError(ValueError):
    """Raised when host-side capture evidence is not independently bound."""


def _capture_module():
    path = Path(__file__).with_name("apt_closure_capture.py")
    spec = importlib.util.spec_from_file_location("r3_apt_capture_for_outer", path)
    if spec is None or spec.loader is None:
        raise OuterCaptureError("nested capture verifier cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CAPTURE = _capture_module()


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")


def canonical_hash(value: Mapping[str, Any], excluded: str | None = None) -> str:
    projection = dict(value)
    if excluded is not None:
        projection.pop(excluded, None)
    return hashlib.sha256(canonical_bytes(projection)).hexdigest()


def _sha_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def verify_collector_plan_sources(plan: Mapping[str, Any]) -> dict[str, Any]:
    """Verify the six fixed mounts before a future candidate invocation.

    The plan is an outer host planning object, not a container input.  In
    particular, no plan, Docker inspect record, staged-deb directory, or base
    status snapshot is mounted into the collector.  Those bytes are generated
    under the writable output mount by fixed workflow phases and are later
    bound by the outer host receipt.
    """
    required_schema = "glim_clean_room_r3_apt_collector_plan_v2"
    if plan.get("schema") != required_schema or plan.get("schema_version") != 1 or \
            plan.get("status") != "REVIEW_REQUIRED" or plan.get("execute") is not False or \
            plan.get("benchmark_eligible") is not False:
        raise OuterCaptureError("collector plan is not the fixed v2 review plan")
    if "host_binding" in plan or "authority" in plan:
        raise OuterCaptureError("collector plan must not expose inner host authority")
    if plan.get("plan_identity_sha256") != canonical_hash(plan, excluded="plan_identity_sha256"):
        raise OuterCaptureError("collector plan identity is not canonical")
    mounts = plan.get("mounts")
    expected_destinations = [
        "/workspace/capture", "/opt/apt_closure_collector.py",
        "/opt/apt_closure_capture.py", "/opt/apt_closure_capture.schema.json",
        "/opt/package-allowlist.json", "/opt/apt-acquisition-ledger.json"]
    if not isinstance(mounts, list) or len(mounts) != 6 or \
            [item.get("destination") for item in mounts] != expected_destinations:
        raise OuterCaptureError("collector plan mount destinations are not exact")
    if plan.get("mounts_sha256") != canonical_hash({"mounts": mounts}):
        raise OuterCaptureError("collector plan mount identity drift")
    workflow = plan.get("workflow")
    if not isinstance(workflow, list) or len(workflow) != 18 or \
            workflow[5].get("phase") != "container_run":
        raise OuterCaptureError("collector plan lacks the fixed container-run phase")
    run_argv = workflow[5].get("argv")
    if not isinstance(run_argv, list) or "--mount" not in run_argv:
        raise OuterCaptureError("collector container argv lacks fixed bind mounts")
    argv_mounts = []
    index = 0
    while index < len(run_argv):
        if run_argv[index] != "--mount":
            index += 1
            continue
        if index + 1 >= len(run_argv) or not isinstance(run_argv[index + 1], str):
            raise OuterCaptureError("collector container mount argv is incomplete")
        argv_mounts.append(run_argv[index + 1])
        index += 2
    expected_argv_mounts = [
        f"type=bind,src={item['source']},dst={item['destination']},"
        f"{'ro' if item['read_only'] else 'rw'}" for item in mounts]
    if argv_mounts != expected_argv_mounts:
        raise OuterCaptureError("container argv mount binding drifted")
    for item in mounts:
        if not isinstance(item, Mapping) or set(item) != {
                "source", "destination", "read_only", "sha256"} or \
                item["read_only"] != (item["destination"] != "/workspace/capture"):
            raise OuterCaptureError("collector plan mount fields or modes drifted")
        source = Path(item["source"])
        if item["destination"] == "/workspace/capture":
            # This is the only source that is expected to be absent at plan
            # time; the fixed output_root_create phase creates it atomically.
            if source.exists() or source.is_symlink() or item["sha256"] != "0" * 64:
                raise OuterCaptureError("collector output mount is not a fresh placeholder")
            continue
        digest, _data = _sha_file(source, max_bytes=MAX_RECEIPT_BYTES)
        if digest != item["sha256"]:
            raise OuterCaptureError(f"collector mount byte identity drift: {item['destination']}")
    if plan.get("output_root") != mounts[0]["source"] or \
            plan.get("allowlist_path") != mounts[4]["source"] or \
            plan.get("ledger_input_path") != mounts[5]["source"] or \
            plan.get("ledger_input_sha256") != mounts[5]["sha256"]:
        raise OuterCaptureError("collector plan input path/hash binding drift")
    return {"status": "PASS", "authority": "OUTER_RECEIPT_ONLY",
            "mount_count": len(mounts), "plan_identity_sha256": plan["plan_identity_sha256"]}


def _sha_file(path: Path, *, max_bytes: int, allow_empty: bool = False,
              mode: int | None = None) -> tuple[str, bytes]:
    try:
        info = path.lstat()
    except OSError as error:
        raise OuterCaptureError(f"host log cannot be inspected: {path}: {error}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or \
            info.st_size > max_bytes or (not allow_empty and info.st_size == 0) or \
            (mode is not None and stat.S_IMODE(info.st_mode) != mode):
        raise OuterCaptureError(f"host log is not a bounded single-link regular file: {path}")
    data = path.read_bytes()
    if len(data) != info.st_size:
        raise OuterCaptureError(f"host log changed while reading: {path}")
    return _sha_bytes(data), data


def _safe_relative(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value.startswith("/") or \
            "\\" in value or any(part in ("", ".", "..") for part in value.split("/")) or \
            Path(value).as_posix() != value:
        raise OuterCaptureError(f"{label} is not a normalized relative path")
    return value


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise OuterCaptureError(f"{label} is not a lowercase SHA-256")
    return value


def _read_json(path: Path, label: str, *, max_bytes: int) -> dict[str, Any]:
    _digest, data = _sha_file(path, max_bytes=max_bytes)
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise OuterCaptureError(f"{label} is invalid JSON: {error}") from error
    if not isinstance(value, dict):
        raise OuterCaptureError(f"{label} is not an object")
    return value


def _plan_id(plan: Mapping[str, Any]) -> str:
    projection = dict(plan)
    projection.pop("plan_identity_sha256", None)
    binding = dict(projection.get("outer_binding", {}))
    binding.pop("plan_identity_sha256", None)
    binding.pop("receipt_sha256", None)
    projection["outer_binding"] = binding
    return canonical_hash(projection)


def _load_planner():
    spec = importlib.util.spec_from_file_location("r3_apt_planner_for_outer", PLANNER_PATH)
    if spec is None or spec.loader is None:
        raise OuterCaptureError("fixed planner cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _check_plan(plan: Mapping[str, Any]) -> None:
    if plan.get("schema") != "glim_clean_room_r3_apt_capture_plan_v2" or \
            plan.get("schema_version") != 2 or plan.get("status") != "REVIEW_REQUIRED" or \
            plan.get("execute") is not False or plan.get("capture_mode") != "CONTRACT_ONLY" or \
            plan.get("benchmark_eligible") is not False or plan.get("network_used") is not True:
        raise OuterCaptureError("plan is not the fixed contract-only review plan")
    if plan.get("image_reference") != PINNED_BASE_IMAGE or \
            _plan_id(plan) != plan.get("plan_identity_sha256"):
        raise OuterCaptureError("plan identity is not canonical")
    workflow = plan.get("workflow")
    if not isinstance(workflow, list) or len(workflow) != 14 or \
            plan.get("workflow_sha256") != canonical_hash({"workflow": workflow}):
        raise OuterCaptureError("plan workflow identity is missing or drifted")
    if plan.get("outer_binding", {}).get("plan_identity_sha256") != plan.get(
            "plan_identity_sha256"):
        raise OuterCaptureError("plan outer binding is incomplete")
    owner = plan.get("host_owner")
    if not isinstance(owner, Mapping) or set(owner) != {"uid", "gid"} or \
            type(owner.get("uid")) is not int or type(owner.get("gid")) is not int or \
            owner["uid"] < 0 or owner["gid"] < 0:
        raise OuterCaptureError("plan host owner identity is incomplete")
    if not isinstance(plan.get("package_allowlist"), Mapping):
        raise OuterCaptureError("plan package allowlist is missing")
    CAPTURE._validate_allowlist(plan["package_allowlist"])
    if plan.get("package_allowlist_sha256") != CAPTURE.canonical_hash(plan["package_allowlist"]):
        raise OuterCaptureError("plan package allowlist identity drift")
    if plan.get("capture_command_sha256") != canonical_hash({"argv": plan["capture_command"]}) or \
            plan.get("provisioning_command_sha256") != canonical_hash(
                {"argv": plan["provisioning_command"]}):
        raise OuterCaptureError("plan inner command identity drift")
    if _sha_file(PLANNER_PATH, max_bytes=MAX_RECEIPT_BYTES)[0] != plan.get("planner_sha256") or \
            _sha_file(CAPTURE_PATH, max_bytes=MAX_RECEIPT_BYTES)[0] != plan.get("capture_tool_sha256") or \
            _sha_file(CAPTURE_SCHEMA_PATH, max_bytes=MAX_RECEIPT_BYTES)[0] != plan.get(
                "capture_schema_sha256") or \
            _sha_file(Path(__file__), max_bytes=MAX_RECEIPT_BYTES)[0] != plan.get(
                "outer_tool_sha256") or \
            _sha_file(SCHEMA_PATH, max_bytes=MAX_RECEIPT_BYTES)[0] != plan.get(
                "outer_schema_sha256"):
        raise OuterCaptureError("capture tool/schema/planner identity drift")
    allowlist_path = Path(plan.get("allowlist_path", ""))
    allowlist_sha, allowlist_bytes = _sha_file(allowlist_path, max_bytes=MAX_RECEIPT_BYTES)
    if allowlist_sha != plan.get("allowlist_file_sha256"):
        raise OuterCaptureError("host allowlist byte identity drift")
    try:
        allowlist_document = json.loads(allowlist_bytes.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise OuterCaptureError(f"host allowlist is invalid JSON: {error}") from error
    if allowlist_document != plan["package_allowlist"]:
        raise OuterCaptureError("host allowlist content differs from planned canonical set")
    mounts = plan.get("mounts")
    if not isinstance(mounts, list) or len(mounts) != 3 or \
            mounts[2].get("source") != str(allowlist_path.resolve()):
        raise OuterCaptureError("host allowlist mount is not the planned immutable file")
    planner = _load_planner()
    expected_commands = planner._fixed_commands(
        image_reference=plan["image_reference"], container_name=plan["container_name"],
        mounts=mounts, allowlist=plan["package_allowlist"])
    expected_workflow = planner._workflow(expected_commands)
    if plan["workflow"] != expected_workflow or \
            plan.get("container_argv") != expected_commands["container_run"] or \
            plan.get("provisioning_sequence") != [expected_commands["provisioning_update"],
                                                   expected_commands["provisioning_install"]] or \
            plan.get("capture_command") != expected_commands["inner_capture"]:
        raise OuterCaptureError("plan contains a non-fixed command sequence")


def _expected_mounts(plan: Mapping[str, Any]) -> list[dict[str, Any]]:
    mounts = plan.get("mounts")
    if not isinstance(mounts, list) or len(mounts) != 3:
        raise OuterCaptureError("plan must contain exactly three fixed mounts")
    result = []
    for item in mounts:
        if not isinstance(item, Mapping) or set(item) != {"source", "destination", "read_only"} or \
                not isinstance(item["source"], str) or not item["source"].startswith("/") or \
                not isinstance(item["destination"], str) or \
                not isinstance(item["read_only"], bool):
            raise OuterCaptureError("plan mount is not exact")
        result.append({
            "Type": "bind", "Source": item["source"],
            "Destination": item["destination"], "RW": not item["read_only"],
            "Mode": "rw" if not item["read_only"] else "ro",
        })
    if [item["Destination"] for item in result] != [
            "/workspace/capture", "/opt/apt_closure_capture.py", "/opt/package-allowlist.json"]:
        raise OuterCaptureError("plan mount destinations are not exact")
    return result


def _image_projection(value: Mapping[str, Any], image_reference: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise OuterCaptureError("host image inspect is not an object")
    projection = {key: value.get(key) for key in ("Id", "RepoDigests", "Os", "Architecture")}
    if DIGEST_RE.fullmatch(str(projection["Id"])) is None or \
            projection["Os"] != "linux" or projection["Architecture"] != "amd64" or \
            not isinstance(projection["RepoDigests"], list) or \
            projection["RepoDigests"] != sorted(set(projection["RepoDigests"])) or \
            image_reference not in projection["RepoDigests"]:
        raise OuterCaptureError("host image inspect does not prove the pinned image")
    return projection


def _absence(data: bytes, name: str) -> None:
    if data.decode("utf-8") != f"Error: No such container: {name}\n":
        raise OuterCaptureError("container absence log is not the exact Docker absence result")


def _record_logs(root: Path, record: Mapping[str, Any]) -> tuple[bytes, bytes]:
    stdout = _safe_relative(record.get("stdout"), "stdout log path")
    stderr = _safe_relative(record.get("stderr"), "stderr log path")
    stdout_sha, stdout_data = _sha_file(root / stdout, max_bytes=MAX_LOG_BYTES, allow_empty=True)
    stderr_sha, stderr_data = _sha_file(root / stderr, max_bytes=MAX_LOG_BYTES, allow_empty=True)
    if record.get("stdout_sha256") != stdout_sha or record.get("stderr_sha256") != stderr_sha or \
            record.get("stdout_bytes") != len(stdout_data) or record.get("stderr_bytes") != len(stderr_data):
        raise OuterCaptureError("host log sidecar identity mismatch")
    return stdout_data, stderr_data


def _parse_lstat(data: bytes, path: str, owner: Mapping[str, Any]) -> dict[str, Any]:
    try:
        tokens = data.decode("utf-8").strip().split("\t")
    except UnicodeError as error:
        raise OuterCaptureError(f"lstat output is not UTF-8: {error}") from error
    if len(tokens) != 5 or tokens[0] != "directory" or tokens[1] not in ("700", "0700"):
        raise OuterCaptureError("output root lstat is not a 0700 directory")
    try:
        uid, gid, nlink = (int(value) for value in tokens[2:])
    except ValueError as error:
        raise OuterCaptureError("output root lstat owner/link fields are invalid") from error
    if uid != owner["uid"] or gid != owner["gid"] or nlink < 1:
        raise OuterCaptureError("output root lstat owner or link identity drift")
    return {"path": path, "file_type": tokens[0], "mode": "0700",
            "uid": uid, "gid": gid, "nlink": nlink}


def _actual_lstat(path: Path, owner: Mapping[str, Any], label: str) -> dict[str, Any]:
    try:
        info = path.lstat()
    except OSError as error:
        raise OuterCaptureError(f"{label} cannot be inspected: {error}") from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode) or \
            stat.S_IMODE(info.st_mode) != 0o700 or info.st_uid != owner["uid"] or \
            info.st_gid != owner["gid"] or info.st_nlink < 1:
        raise OuterCaptureError(f"{label} is not the host-owned fresh 0700 directory")
    return {"path": str(path), "file_type": "directory", "mode": "0700",
            "uid": int(info.st_uid), "gid": int(info.st_gid), "nlink": int(info.st_nlink)}


def _verify_post_inspect(value: Mapping[str, Any], plan: Mapping[str, Any], image: Mapping[str, Any]) -> None:
    if value.get("Image") != image["Id"]:
        raise OuterCaptureError("post-disconnect inspect image ID drift")
    mounts = value.get("Mounts")
    expected = _expected_mounts(plan)
    if not isinstance(mounts, list) or len(mounts) != len(expected):
        raise OuterCaptureError("post-disconnect inspect mount count drift")
    observed = []
    for item in mounts:
        if not isinstance(item, Mapping):
            raise OuterCaptureError("post-disconnect mount is not an object")
        observed.append({key: item.get(key) for key in ("Type", "Source", "Destination", "RW", "Mode")})
    if sorted(observed, key=lambda item: str(item["Destination"])) != sorted(
            expected, key=lambda item: str(item["Destination"])):
        raise OuterCaptureError("post-disconnect inspect mounts drift")
    networks = value.get("NetworkSettings", {}).get("Networks")
    if networks != {}:
        raise OuterCaptureError("post-disconnect inspect still reports a network")


def _walk(root: Path) -> set[str]:
    info = root.lstat()
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
        raise OuterCaptureError("outer evidence root is not a directory")
    files: set[str] = set()
    for item in root.rglob("*"):
        item_info = item.lstat()
        if stat.S_ISLNK(item_info.st_mode) or \
                (stat.S_ISREG(item_info.st_mode) and item_info.st_nlink != 1):
            raise OuterCaptureError("outer evidence contains symlink or hardlink")
        if stat.S_ISREG(item_info.st_mode):
            files.add(item.relative_to(root).as_posix())
        elif not stat.S_ISDIR(item_info.st_mode):
            raise OuterCaptureError("outer evidence contains a special file")
    return files


def verify_outer_root(root: Path, plan: Mapping[str, Any]) -> dict[str, Any]:
    """Reopen host logs and receipt, independently binding nested capture bytes."""
    _check_plan(plan)
    root = Path(root)
    if str(root) != plan.get("outer_root"):
        raise OuterCaptureError("outer root does not match plan")
    receipt_path = root / OUTER_RECEIPT
    sidecar_path = root / (OUTER_RECEIPT + ".sha256")
    receipt_sha, receipt_bytes = _sha_file(receipt_path, max_bytes=MAX_RECEIPT_BYTES, mode=0o444)
    _side_sha, sidecar_bytes = _sha_file(sidecar_path, max_bytes=512, mode=0o444)
    if sidecar_bytes.decode("ascii").strip().split() != [receipt_sha, OUTER_RECEIPT]:
        raise OuterCaptureError("outer receipt sidecar mismatch")
    try:
        receipt = json.loads(receipt_bytes.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise OuterCaptureError(f"outer receipt is invalid JSON: {error}") from error
    if not isinstance(receipt, Mapping) or set(receipt) != {
            "schema", "schema_version", "status", "benchmark_eligible", "capture_mode",
            "plan_identity_sha256", "image_reference", "container_name", "records",
            "output_root_lstat", "outer_root_lstat", "image_inspect",
            "post_disconnect_inspect", "cleanup", "inner_payload", "outer_identity_sha256"}:
        raise OuterCaptureError("outer receipt fields are incomplete or contain extras")
    if receipt["schema"] != SCHEMA or receipt["schema_version"] != SCHEMA_VERSION or \
            receipt["status"] != "REVIEW_REQUIRED" or receipt["benchmark_eligible"] is not False or \
            receipt["capture_mode"] != "CONTRACT_ONLY":
        raise OuterCaptureError("outer receipt is not a non-promoting contract-only receipt")
    if receipt["plan_identity_sha256"] != plan["plan_identity_sha256"] or \
            receipt["image_reference"] != plan["image_reference"] or \
            receipt["container_name"] != plan["container_name"]:
        raise OuterCaptureError("outer receipt plan/container binding mismatch")
    if receipt["outer_identity_sha256"] != canonical_hash(receipt, "outer_identity_sha256"):
        raise OuterCaptureError("outer receipt identity is not canonical")
    workflow = plan["workflow"]
    records = receipt["records"]
    if not isinstance(records, list) or len(records) != len(workflow):
        raise OuterCaptureError("outer command record count mismatch")
    expected_log_files = {OUTER_RECEIPT, OUTER_RECEIPT + ".sha256"}
    parsed: dict[str, tuple[bytes, bytes]] = {}
    for index, (record, expected) in enumerate(zip(records, workflow)):
        if not isinstance(record, Mapping) or set(record) != {
                "phase", "ordinal", "argv", "exit_status", "stdout", "stderr",
                "stdout_sha256", "stderr_sha256", "stdout_bytes", "stderr_bytes"}:
            raise OuterCaptureError("outer command record is incomplete")
        if record["phase"] != expected["phase"] or record["ordinal"] != index or \
                record["argv"] != expected["argv"] or \
                record["exit_status"] != expected["expected_exit_status"]:
            raise OuterCaptureError("outer command order/argv/exit drift")
        stdout = _safe_relative(record["stdout"], "stdout log path")
        stderr = _safe_relative(record["stderr"], "stderr log path")
        if stdout in expected_log_files or stderr in expected_log_files or stdout == stderr:
            raise OuterCaptureError("outer command log path collision")
        expected_log_files.update((stdout, stderr))
        parsed[record["phase"]] = _record_logs(root, record)
    if _walk(root) != expected_log_files:
        raise OuterCaptureError("outer evidence has missing or extra files")
    owner = plan["host_owner"]
    output_lstat = _parse_lstat(parsed["output_root_lstat"][0], plan["output_root"], owner)
    if receipt["output_root_lstat"] != output_lstat or \
            _actual_lstat(Path(plan["output_root"]), owner, "output root") != output_lstat:
        raise OuterCaptureError("output root creation/lstat evidence drift")
    outer_lstat = _actual_lstat(root, owner, "outer root")
    if receipt["outer_root_lstat"] != outer_lstat:
        raise OuterCaptureError("outer root creation/lstat evidence drift")
    image = _image_projection(json.loads(parsed["image_inspect"][0].decode("utf-8")),
                              plan["image_reference"])
    _absence(parsed["preexisting_name_inspect"][1], plan["container_name"])
    _absence(parsed["post_remove_inspect"][1], plan["container_name"])
    _verify_post_inspect(json.loads(parsed["post_disconnect_inspect"][0].decode("utf-8")),
                         plan, image)
    if receipt["image_inspect"] != image:
        raise OuterCaptureError("outer receipt image projection is not host-log derived")
    post_projection = json.loads(parsed["post_disconnect_inspect"][0].decode("utf-8"))
    if receipt["post_disconnect_inspect"] != {
            "Image": post_projection.get("Image"),
            "Mounts": post_projection.get("Mounts"),
            "NetworkSettings": {"Networks": post_projection.get(
                "NetworkSettings", {}).get("Networks")}}:
        raise OuterCaptureError("outer receipt post-inspect projection is not host-log derived")
    if receipt["cleanup"] != {
            "stop_exit_status": 0, "remove_exit_status": 0,
            "post_remove_absent": True}:
        raise OuterCaptureError("cleanup receipt is not the fixed successful cleanup projection")
    inner = receipt["inner_payload"]
    if not isinstance(inner, Mapping) or set(inner) != {
            "status", "capture_root", "receipt_sha256"}:
        raise OuterCaptureError("inner payload binding is incomplete")
    if inner["status"] == "PRESENT":
        if inner["capture_root"] != plan["output_root"]:
            raise OuterCaptureError("inner capture root drift")
        capture_root = Path(inner["capture_root"])
        result = CAPTURE.verify_capture_root(capture_root, {
            "image_reference": plan["image_reference"],
            "argv": plan["container_argv"],
            "mounts": plan["mounts"],
            "output_root_lstat": output_lstat,
            "outer_root_lstat": outer_lstat,
            "image_inspect": image,
            "plan_identity_sha256": plan["plan_identity_sha256"],
            "receipt_sha256": inner["receipt_sha256"],
        })
        if result["receipt_sha256"] != inner["receipt_sha256"]:
            raise OuterCaptureError("inner receipt binding mismatch")
        inner_receipt = CAPTURE._read_json(
            capture_root / CAPTURE.CAPTURE_RECEIPT, "inner capture receipt")
        if inner_receipt.get("provisioning", {}).get("command") != \
                plan["provisioning_command"] or \
                inner_receipt.get("provisioning", {}).get("command_sha256") != \
                plan["provisioning_command_sha256"]:
            raise OuterCaptureError("inner provisioning command is not the fixed plan command")
    elif inner != {"status": "CONTRACT_ONLY_NOT_RUN", "capture_root": None,
                   "receipt_sha256": None}:
        raise OuterCaptureError("inner payload is neither verified nor contract-only")
    return {
        "status": receipt["status"], "benchmark_eligible": False,
        "capture_mode": receipt["capture_mode"], "outer_receipt_sha256": receipt_sha,
        "plan_identity_sha256": plan["plan_identity_sha256"],
        "image_id": image["Id"], "command_count": len(records),
        "inner_payload_status": inner["status"], "root": str(root),
    }


def seal_outer_receipt(root: Path, plan: Mapping[str, Any],
                       outputs: Mapping[str, tuple[bytes, bytes]],
                       *, inner_capture_root: Path | None = None,
                       inner_receipt_sha256: str | None = None) -> dict[str, Any]:
    """Seal synthetic/host-collected command logs; never overwrite a root."""
    _check_plan(plan)
    root = Path(root)
    if root.exists() or root.is_symlink() or not root.parent.is_dir():
        raise OuterCaptureError("outer evidence root must be fresh")
    root.mkdir(parents=True)
    root.chmod(0o700)
    try:
        records = []
        for expected in plan["workflow"]:
            phase = expected["phase"]
            if phase not in outputs:
                raise OuterCaptureError(f"missing output for phase {phase}")
            stdout, stderr = outputs[phase]
            if not isinstance(stdout, bytes) or not isinstance(stderr, bytes) or \
                    len(stdout) > MAX_LOG_BYTES or len(stderr) > MAX_LOG_BYTES:
                raise OuterCaptureError(f"invalid output bytes for phase {phase}")
            stdout_path = Path("logs") / f"{expected['ordinal']:02d}-{phase}.stdout"
            stderr_path = Path("logs") / f"{expected['ordinal']:02d}-{phase}.stderr"
            (root / stdout_path).parent.mkdir(parents=True, exist_ok=True)
            (root / stdout_path).write_bytes(stdout)
            (root / stderr_path).write_bytes(stderr)
            (root / stdout_path).chmod(0o444)
            (root / stderr_path).chmod(0o444)
            records.append({
                "phase": phase, "ordinal": expected["ordinal"],
                "argv": expected["argv"], "exit_status": expected["expected_exit_status"],
                "stdout": stdout_path.as_posix(), "stderr": stderr_path.as_posix(),
                "stdout_sha256": _sha_bytes(stdout), "stderr_sha256": _sha_bytes(stderr),
                "stdout_bytes": len(stdout), "stderr_bytes": len(stderr),
            })
        image = _image_projection(json.loads(outputs["image_inspect"][0].decode("utf-8")),
                                  plan["image_reference"])
        output_lstat = _parse_lstat(outputs["output_root_lstat"][0],
                                    plan["output_root"], plan["host_owner"])
        if _actual_lstat(Path(plan["output_root"]), plan["host_owner"],
                         "output root") != output_lstat:
            raise OuterCaptureError("output root creation/lstat evidence drift")
        outer_lstat = _actual_lstat(root, plan["host_owner"], "outer root")
        post = json.loads(outputs["post_disconnect_inspect"][0].decode("utf-8"))
        receipt = {
            "schema": SCHEMA, "schema_version": SCHEMA_VERSION,
            "status": "REVIEW_REQUIRED", "benchmark_eligible": False,
            "capture_mode": "CONTRACT_ONLY",
            "plan_identity_sha256": plan["plan_identity_sha256"],
            "image_reference": plan["image_reference"],
            "container_name": plan["container_name"], "records": records,
            "output_root_lstat": output_lstat,
            "outer_root_lstat": outer_lstat,
            "image_inspect": image,
            "post_disconnect_inspect": {
                "Image": post.get("Image"), "Mounts": post.get("Mounts"),
                "NetworkSettings": {"Networks": post.get(
                    "NetworkSettings", {}).get("Networks")}},
            "cleanup": {"stop_exit_status": 0, "remove_exit_status": 0,
                        "post_remove_absent": True},
            "inner_payload": {
                "status": "PRESENT" if inner_capture_root is not None else "CONTRACT_ONLY_NOT_RUN",
                "capture_root": str(inner_capture_root) if inner_capture_root is not None else None,
                "receipt_sha256": inner_receipt_sha256,
            },
        }
        receipt["outer_identity_sha256"] = canonical_hash(receipt)
        receipt_bytes = (json.dumps(receipt, sort_keys=True, separators=(",", ":")) + "\n").encode()
        receipt_path = root / OUTER_RECEIPT
        sidecar_path = root / (OUTER_RECEIPT + ".sha256")
        with receipt_path.open("xb") as stream:
            stream.write(receipt_bytes)
        with sidecar_path.open("xb") as stream:
            stream.write((f"{_sha_bytes(receipt_bytes)}  {OUTER_RECEIPT}\n").encode())
        receipt_path.chmod(0o444)
        sidecar_path.chmod(0o444)
        return verify_outer_root(root, plan)
    except Exception:
        raise


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    verify = sub.add_parser("verify")
    verify.add_argument("--root", type=Path, required=True)
    verify.add_argument("--plan", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        plan = _read_json(args.plan, "plan", max_bytes=MAX_RECEIPT_BYTES)
        result = verify_outer_root(args.root, plan)
        print(json.dumps(result, sort_keys=True))
        return 0
    except (OSError, OuterCaptureError, TypeError, ValueError) as error:
        print(json.dumps({"status": "FAIL_CLOSED", "reason": str(error)}, sort_keys=True))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
