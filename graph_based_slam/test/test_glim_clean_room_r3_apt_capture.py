#!/usr/bin/env python3
"""Adversarial synthetic tests for the GLIM r3 apt/deb capture boundary."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parents[2]
CAPTURE_PATH = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3/apt_closure_capture.py"
PLANNER_PATH = ROOT / "scripts/plan_glim_clean_room_r3_apt_capture.py"
OUTER_PATH = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3/apt_closure_outer.py"
OUTER_SCHEMA_PATH = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3/apt_closure_outer.schema.json"
PRODUCTION_MANIFEST = ROOT / (
    "docker/benchmark_adapters/glim_clean_room/phase3d/r3/"
    "offline_dependency_manifest.json")


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CAPTURE = _load(CAPTURE_PATH, "glim_r3_apt_closure_capture")
PLANNER = _load(PLANNER_PATH, "glim_r3_apt_closure_plan")
OUTER = _load(OUTER_PATH, "glim_r3_apt_closure_outer")


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _fixture(tmp_path: Path):
    source = tmp_path / "payload-source"
    source.mkdir(parents=True)
    base = json.loads(PRODUCTION_MANIFEST.read_text(encoding="utf-8"))["base_image"]["reference"]
    image_inspect = {
        "Id": "sha256:" + "b" * 64,
        "RepoDigests": [base],
        "Os": "linux",
        "Architecture": "amd64",
    }
    build_key = {"name": "build-tool", "version": "1.0-1", "architecture": "amd64"}
    runtime_key = {"name": "runtime-lib", "version": "2.0-1", "architecture": "amd64"}
    allowlist = {
        "build": [build_key], "runtime": [runtime_key],
        "union": sorted([build_key, runtime_key],
                         key=lambda item: (item["name"], item["version"], item["architecture"])),
    }
    allowlist["canonical_sha256"] = CAPTURE.canonical_hash(allowlist)
    allowlist_path = tmp_path / "package-allowlist.json"
    allowlist_path.write_text(json.dumps(allowlist, sort_keys=True, separators=(",", ":")) + "\n",
                              encoding="utf-8")
    command = ["apt-get", "install", "--yes", "--no-install-recommends",
               "build-tool=1.0-1", "runtime-lib=2.0-1"]
    output = tmp_path / "capture-output"
    plan = PLANNER.plan_capture(
        output_root=output, allowlist=allowlist, allowlist_path=allowlist_path)

    def add_artifact(path: str, role: str, data: bytes, source_name: str, url: str | None = None):
        (source / source_name).write_bytes(data)
        record = {
            "path": path, "role": role, "source_path": source_name,
            "bytes": len(data), "sha256": _sha(data),
        }
        if url is not None:
            record["source_url"] = url
        return record

    source_url = "https://apt.example.invalid/ubuntu.sources"
    release_url = "https://apt.example.invalid/ubuntu/InRelease"
    rosdep_url = "https://ros.example.invalid/rosdep/sources.yaml"
    package_defs = [
        (build_key, "build-tool_1.0-1_amd64.deb", "MIT"),
        (runtime_key, "runtime-lib_2.0-1_amd64.deb", "BSD-3-Clause"),
    ]
    artifacts = [
        add_artifact("artifacts/apt/InRelease", "apt_release", b"InRelease\n",
                      "apt-inrelease", release_url),
        add_artifact("artifacts/apt/sources.list", "apt_source", b"deb fixture main\n",
                      "apt-sources", source_url),
        add_artifact("artifacts/rosdep/cache.yaml", "rosdep_cache", b"cache: fixture\n",
                      "rosdep-cache"),
        add_artifact("artifacts/rosdep/sources.yaml", "rosdep_source", b"yaml fixture\n",
                      "rosdep-sources", rosdep_url),
    ]
    packages = []
    for key, filename, license_identity in package_defs:
        deb_path = "artifacts/deb/" + filename
        license_path = "artifacts/license/" + key["name"] + ".txt"
        deb = add_artifact(deb_path, "deb", (filename + "\n").encode(),
                           filename, "https://packages.example.invalid/" + filename)
        license_bytes = (license_identity + "\n").encode()
        license_record = add_artifact(license_path, "license", license_bytes,
                                      key["name"] + "-license")
        artifacts.extend([deb, license_record])
        packages.append({
            **key,
            "status": "install ok installed",
            "filename": filename,
            "url": "https://packages.example.invalid/" + filename,
            "sha256": deb["sha256"], "bytes": deb["bytes"],
            "artifact_path": deb_path,
            "license": {
                "path": "usr/share/doc/" + key["name"] + "/copyright",
                "identity": license_identity,
                "sha256": license_record["sha256"],
                "bytes": license_record["bytes"],
                "artifact_path": license_path,
            },
        })
    artifacts.sort(key=lambda item: item["path"])
    packages.sort(key=lambda item: (item["name"], item["version"], item["architecture"]))
    repository = {
        "url": "https://apt.example.invalid/ubuntu",
        "release_or_inrelease_url": release_url,
        "release_or_inrelease_sha256": next(item["sha256"] for item in artifacts
                                             if item["role"] == "apt_release"),
        "release_or_inrelease_bytes": next(item["bytes"] for item in artifacts
                                            if item["role"] == "apt_release"),
    }
    apt_sources = [{
        "url": source_url, "artifact_path": "artifacts/apt/sources.list",
        "sha256": next(item["sha256"] for item in artifacts if item["role"] == "apt_source"),
        "bytes": next(item["bytes"] for item in artifacts if item["role"] == "apt_source"),
    }]
    releases = [{
        "url": release_url, "artifact_path": "artifacts/apt/InRelease",
        "sha256": repository["release_or_inrelease_sha256"],
        "bytes": repository["release_or_inrelease_bytes"],
    }]
    rosdep_sources = [{
        "url": rosdep_url, "artifact_path": "artifacts/rosdep/sources.yaml",
        "sha256": next(item["sha256"] for item in artifacts if item["role"] == "rosdep_source"),
        "bytes": next(item["bytes"] for item in artifacts if item["role"] == "rosdep_source"),
    }]
    rosdep_cache = [{
        "artifact_path": "artifacts/rosdep/cache.yaml",
        "sha256": next(item["sha256"] for item in artifacts if item["role"] == "rosdep_cache"),
        "bytes": next(item["bytes"] for item in artifacts if item["role"] == "rosdep_cache"),
    }]
    archive_identity = CAPTURE._load_dependency_prefetch()._archive_identity(
        json.loads(PRODUCTION_MANIFEST.read_text(encoding="utf-8")))
    payload = {
        "schema": CAPTURE.SCHEMA, "schema_version": 1,
        "status": "REVIEW_REQUIRED", "benchmark_eligible": False,
        "capture_id": "synthetic-r3-apt-closure-001",
        "base_image": {"reference": base, "digest": base.replace("ros@", ""),
                       "distribution": "jazzy", "platform": "linux/amd64"},
        "network": {"network_used": True, "phase": "PROVISIONING_CONNECTED",
                    "build_test_network": "NOT_STARTED", "disconnect_required": True},
        "provisioning": {"command": command,
                          "command_sha256": plan["provisioning_command_sha256"],
                          "exit_status": 0},
        "container": {
            "image_reference": base, "image_inspect": image_inspect,
            "argv": plan["container_argv"], "mounts": plan["mounts"],
            "network_mode": "bridge", "network_used": True,
            "phase_trace": [
                {"phase": "image_inspect", "network_used": False},
                {"phase": "container_run", "network_used": True},
                {"phase": "payload_capture", "network_used": True},
                {"phase": "disconnect_pending", "network_used": True},
            ],
        },
        "package_allowlist": allowlist,
        "dpkg_packages_sorted": packages,
        "repositories": [repository],
        "apt_source_files_sha256": apt_sources,
        "apt_release_or_inrelease": releases,
        "rosdep_sources_sha256": rosdep_sources,
        "rosdep_cache_files": rosdep_cache,
        "artifacts": artifacts,
        "archive_set_identity_sha256": archive_identity,
        "plan_identity_sha256": plan["plan_identity_sha256"],
        "custodian_review": {"status": "UNSIGNED_PLACEHOLDER", "signature": "", "required": True},
    }
    return source, output, plan, payload


def _expected(plan, payload, receipt_sha):
    return {
        "image_reference": payload["base_image"]["reference"],
        "argv": plan["container_argv"], "mounts": plan["mounts"],
        "image_inspect": payload["container"]["image_inspect"],
        "plan_identity_sha256": plan["plan_identity_sha256"],
        "receipt_sha256": receipt_sha,
    }


def test_capture_review_seals_raw_apt_deb_license_bytes_and_proposal(tmp_path):
    source, output, plan, payload = _fixture(tmp_path)
    result = CAPTURE.seal_capture(source, output, payload)
    assert result["status"] == "REVIEW_REQUIRED"
    assert result["benchmark_eligible"] is False
    expected = _expected(plan, payload, result["receipt_sha256"])
    assert CAPTURE.verify_capture_root(output, expected)["status"] == "REVIEW_REQUIRED"
    proposal_path = tmp_path / "proposal.json"
    proposal = CAPTURE.compose_proposal(output, PRODUCTION_MANIFEST, expected, proposal_path)
    assert proposal["status"] == "PROPOSED_REVIEW_REQUIRED"
    assert CAPTURE.validate_proposal(proposal_path)["benchmark_eligible"] is False
    production = json.loads(PRODUCTION_MANIFEST.read_text(encoding="utf-8"))
    assert production["status"] == "NOT_READY_APT_CLOSURE_UNSEALED"
    assert production["apt_deb_closure"]["status"] == "NOT_READY_APT_CAPTURE_REQUIRED"


def test_capture_rejects_fake_image_network_lie_allowlist_and_nonfresh(tmp_path):
    source, output, plan, payload = _fixture(tmp_path)
    fake = copy.deepcopy(payload)
    fake["container"]["image_inspect"]["RepoDigests"] = [
        "ros@sha256:" + "c" * 64]
    with pytest.raises(CAPTURE.CaptureError, match="pinned RepoDigest"):
        CAPTURE.validate_payload(fake)
    fake = copy.deepcopy(payload)
    fake["network"]["network_used"] = False
    with pytest.raises(CAPTURE.CaptureError, match="network phase"):
        CAPTURE.validate_payload(fake)
    fake = copy.deepcopy(payload)
    fake["package_allowlist"]["union"] = [fake["package_allowlist"]["union"][0]]
    with pytest.raises(CAPTURE.CaptureError, match="union"):
        CAPTURE.validate_payload(fake)
    CAPTURE.seal_capture(source, output, payload)
    with pytest.raises(CAPTURE.CaptureError, match="fresh"):
        CAPTURE.seal_capture(source, output, payload)


def test_capture_rejects_missing_release_license_deb_and_extra_or_mixed_files(tmp_path):
    source, output, plan, payload = _fixture(tmp_path)
    missing = copy.deepcopy(payload)
    missing["apt_release_or_inrelease"] = []
    with pytest.raises(CAPTURE.CaptureError, match="non-empty"):
        CAPTURE.validate_payload(missing)
    missing = copy.deepcopy(payload)
    missing["dpkg_packages_sorted"][0]["license"]["artifact_path"] = "missing-license"
    with pytest.raises(CAPTURE.CaptureError, match="artifact binding"):
        CAPTURE.validate_payload(missing)
    missing = copy.deepcopy(payload)
    missing["dpkg_packages_sorted"][0]["artifact_path"] = "missing.deb"
    with pytest.raises(CAPTURE.CaptureError, match="artifact binding"):
        CAPTURE.validate_payload(missing)
    result = CAPTURE.seal_capture(source, output, payload)
    extra = output / "extra.json"
    extra.write_text("extra\n", encoding="utf-8")
    extra.chmod(0o444)
    expected = _expected(plan, payload, result["receipt_sha256"])
    with pytest.raises(CAPTURE.CaptureError, match="extra"):
        CAPTURE.verify_capture_root(output, expected)


def test_capture_outer_receipt_binding_rejects_self_rehash_and_mount_drift(tmp_path):
    source, output, plan, payload = _fixture(tmp_path)
    result = CAPTURE.seal_capture(source, output, payload)
    expected = _expected(plan, payload, result["receipt_sha256"])
    receipt_path = output / CAPTURE.CAPTURE_RECEIPT
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    receipt["capture_id"] = "tampered-capture"
    receipt["closure_identity_sha256"] = CAPTURE.canonical_hash(receipt, "closure_identity_sha256")
    payload_bytes = (json.dumps(receipt, sort_keys=True, separators=(",", ":")) + "\n").encode()
    receipt_path.chmod(0o644)
    receipt_path.write_bytes(payload_bytes)
    receipt_path.chmod(0o444)
    sidecar = output / (CAPTURE.CAPTURE_RECEIPT + ".sha256")
    sidecar.chmod(0o644)
    sidecar.write_text(CAPTURE.sha256_bytes(payload_bytes) + "  " + CAPTURE.CAPTURE_RECEIPT + "\n",
                       encoding="ascii")
    sidecar.chmod(0o444)
    with pytest.raises(CAPTURE.CaptureError, match="outer host receipt binding"):
        CAPTURE.verify_capture_root(output, expected)

    source, output, plan, payload = _fixture(tmp_path / "mount")
    payload["container"]["mounts"][0]["read_only"] = True
    with pytest.raises(CAPTURE.CaptureError, match="mount"):
        CAPTURE.validate_payload(payload)


def test_capture_planner_is_networked_only_for_provisioning_and_never_executes(tmp_path):
    source, output, plan, payload = _fixture(tmp_path)
    assert plan["status"] == "REVIEW_REQUIRED"
    assert plan["execute"] is False
    assert plan["network_used"] is True
    assert plan["build_test_network"] == "NOT_STARTED"
    assert [item["phase"] for item in plan["workflow"]] == [
        "image_inspect", "output_root_absent_check", "output_root_create",
        "output_root_lstat", "preexisting_name_inspect", "container_run",
        "provisioning_update", "provisioning_install", "capture", "disconnect",
        "post_disconnect_inspect", "stop", "remove", "post_remove_inspect"]
    assert plan["workflow"][2]["argv"] == [
        "mkdir", "--mode", "0700", "--", plan["output_root"]]
    assert plan["workflow"][2]["expected_exit_status"] == 0
    assert PLANNER._plan_identity(plan) == plan["plan_identity_sha256"]
    assert plan["container_argv"][0:10] == [
        "docker", "run", "--name", plan["container_name"], "--network", "bridge",
        "--platform", "linux/amd64", "--entrypoint", "sleep"]
    assert [item["destination"] for item in plan["mounts"]] == [
        "/workspace/capture", "/opt/apt_closure_capture.py", "/opt/package-allowlist.json"]
    with pytest.raises(PLANNER.PlanError, match="pinned"):
        PLANNER.plan_capture(
            output_root=tmp_path / "wrong-image", allowlist=payload["package_allowlist"],
            allowlist_path=tmp_path / "package-allowlist.json",
            image_reference="ros@sha256:" + "d" * 64)


def test_capture_rejects_mixed_plan_and_capture_roots(tmp_path):
    source_a, output_a, plan_a, payload_a = _fixture(tmp_path / "a")
    result_a = CAPTURE.seal_capture(source_a, output_a, payload_a)
    source_b, output_b, _plan_b, _payload_b = _fixture(tmp_path / "b")
    result_b = CAPTURE.seal_capture(source_b, output_b, _payload_b)
    expected_a = _expected(plan_a, payload_a, result_a["receipt_sha256"])
    expected_a["receipt_sha256"] = result_b["receipt_sha256"]
    with pytest.raises(CAPTURE.CaptureError, match="outer host (?:plan/image/mount|receipt) binding"):
        CAPTURE.verify_capture_root(output_b, expected_a)


def _outer_outputs(plan, image_id="sha256:" + "b" * 64):
    image = {
        "Id": image_id,
        "RepoDigests": [plan["image_reference"]],
        "Os": "linux",
        "Architecture": "amd64",
    }
    mounts = []
    for mount in plan["mounts"]:
        mounts.append({
            "Type": "bind", "Source": mount["source"],
            "Destination": mount["destination"],
            "RW": not mount["read_only"],
            "Mode": "rw" if not mount["read_only"] else "ro",
        })
    post = {"Image": image_id, "Mounts": mounts,
            "NetworkSettings": {"Networks": {}}}
    outputs = {}
    for record in plan["workflow"]:
        phase = record["phase"]
        if phase == "image_inspect":
            outputs[phase] = ((json.dumps(image, sort_keys=True) + "\n").encode(), b"")
        elif phase == "output_root_lstat":
            owner = plan["host_owner"]
            nlink = Path(plan["output_root"]).lstat().st_nlink
            outputs[phase] = (f"directory\t700\t{owner['uid']}\t{owner['gid']}\t{nlink}\n".encode(), b"")
        elif phase in ("preexisting_name_inspect", "post_remove_inspect"):
            outputs[phase] = (b"", f"Error: No such container: {plan['container_name']}\n".encode())
        elif phase == "post_disconnect_inspect":
            outputs[phase] = ((json.dumps(post, sort_keys=True) + "\n").encode(), b"")
        else:
            outputs[phase] = (b"", b"")
    return outputs


def _rewrite_outer_receipt(root, mutate):
    path = root / OUTER.OUTER_RECEIPT
    receipt = json.loads(path.read_text(encoding="utf-8"))
    mutate(receipt)
    receipt["outer_identity_sha256"] = OUTER.canonical_hash(receipt, "outer_identity_sha256")
    data = (json.dumps(receipt, sort_keys=True, separators=(",", ":")) + "\n").encode()
    path.chmod(0o644)
    path.write_bytes(data)
    path.chmod(0o444)
    sidecar = root / (OUTER.OUTER_RECEIPT + ".sha256")
    sidecar.chmod(0o644)
    sidecar.write_text(OUTER._sha_bytes(data) + "  " + OUTER.OUTER_RECEIPT + "\n",
                       encoding="ascii")
    sidecar.chmod(0o444)


def test_outer_host_receipt_reopens_logs_and_binds_inner_payload(tmp_path):
    source, output, plan, payload = _fixture(tmp_path)
    inner = CAPTURE.seal_capture(source, output, payload)
    output.chmod(0o700)
    outer_root = Path(plan["outer_root"])
    result = OUTER.seal_outer_receipt(
        outer_root, plan, _outer_outputs(plan), inner_capture_root=output,
        inner_receipt_sha256=inner["receipt_sha256"])
    assert result["status"] == "REVIEW_REQUIRED"
    assert result["inner_payload_status"] == "PRESENT"
    assert OUTER.verify_outer_root(outer_root, plan)["benchmark_eligible"] is False
    outer_schema = json.loads(OUTER_SCHEMA_PATH.read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(outer_schema)
    outer_receipt = json.loads((outer_root / OUTER.OUTER_RECEIPT).read_text(encoding="utf-8"))
    assert list(Draft202012Validator(outer_schema).iter_errors(outer_receipt)) == []


def test_outer_rejects_arbitrary_commands_host_mismatch_and_reordered_steps(tmp_path):
    source, output, plan, payload = _fixture(tmp_path)
    inner = CAPTURE.seal_capture(source, output, payload)
    output.chmod(0o700)
    with pytest.raises(PLANNER.PlanError, match="arbitrary"):
        PLANNER.plan_capture(
            output_root=tmp_path / "other", allowlist=payload["package_allowlist"],
            allowlist_path=tmp_path / "package-allowlist.json",
            provisioning_command=["sh", "-c", "apt-get update; evil"],
        )
    outer_root = Path(plan["outer_root"])
    OUTER.seal_outer_receipt(
        outer_root, plan, _outer_outputs(plan), inner_capture_root=output,
        inner_receipt_sha256=inner["receipt_sha256"])
    image_log = outer_root / "logs/00-image_inspect.stdout"
    image_log.chmod(0o644)
    image_log.write_text(json.dumps({
        "Id": "sha256:" + "c" * 64,
        "RepoDigests": ["ros@sha256:" + "d" * 64], "Os": "linux", "Architecture": "amd64",
    }) + "\n", encoding="utf-8")
    image_log.chmod(0o444)
    wrong_image_bytes = image_log.read_bytes()
    _rewrite_outer_receipt(outer_root, lambda receipt: (
        receipt["records"][0].update({
            "stdout_sha256": OUTER._sha_bytes(wrong_image_bytes),
            "stdout_bytes": len(wrong_image_bytes),
        }), receipt.__setitem__("image_inspect", {
            "Id": "sha256:" + "c" * 64,
            "RepoDigests": ["ros@sha256:" + "d" * 64],
            "Os": "linux", "Architecture": "amd64",
        })))
    with pytest.raises(OUTER.OuterCaptureError, match="pinned image"):
        OUTER.verify_outer_root(outer_root, plan)

    # Rebuild a fresh outer root, then alter only the sealed record ordering.
    outer_root = Path(str(plan["outer_root"]) + "-reordered")
    plan_reordered = copy.deepcopy(plan)
    plan_reordered["outer_root"] = str(outer_root)
    plan_reordered["outer_binding"]["outer_root"] = str(outer_root)
    plan_reordered["plan_identity_sha256"] = PLANNER._plan_identity(plan_reordered)
    plan_reordered["outer_binding"]["plan_identity_sha256"] = plan_reordered["plan_identity_sha256"]
    OUTER.seal_outer_receipt(
        outer_root, plan_reordered, _outer_outputs(plan_reordered))
    _rewrite_outer_receipt(outer_root, lambda receipt: receipt["records"].__setitem__(0, receipt["records"][1]))
    with pytest.raises(OUTER.OuterCaptureError, match="order/argv/exit"):
        OUTER.verify_outer_root(outer_root, plan_reordered)
    _rewrite_outer_receipt(outer_root, lambda receipt: receipt["records"][2].__setitem__(
        "phase", "container_run"))
    with pytest.raises(OUTER.OuterCaptureError, match="order/argv/exit"):
        OUTER.verify_outer_root(outer_root, plan_reordered)


def test_outer_rejects_network_or_mount_mismatch_and_missing_cleanup(tmp_path):
    source, output, plan, payload = _fixture(tmp_path)
    inner = CAPTURE.seal_capture(source, output, payload)
    output.chmod(0o700)
    outer_root = Path(plan["outer_root"])
    OUTER.seal_outer_receipt(
        outer_root, plan, _outer_outputs(plan), inner_capture_root=output,
        inner_receipt_sha256=inner["receipt_sha256"])
    outer_root.chmod(0o755)
    with pytest.raises(OUTER.OuterCaptureError, match="outer root"):
        OUTER.verify_outer_root(outer_root, plan)
    outer_root.chmod(0o700)
    output.chmod(0o755)
    with pytest.raises(OUTER.OuterCaptureError, match="output root"):
        OUTER.verify_outer_root(outer_root, plan)
    output.chmod(0o700)
    real_output = output.with_name("capture-output-real")
    output.rename(real_output)
    output.symlink_to(real_output, target_is_directory=True)
    with pytest.raises(OUTER.OuterCaptureError, match="output root"):
        OUTER.verify_outer_root(outer_root, plan)
    output.unlink()
    real_output.rename(output)
    post_log = outer_root / "logs/10-post_disconnect_inspect.stdout"
    post = json.loads(post_log.read_text(encoding="utf-8"))
    post["NetworkSettings"]["Networks"] = {"bridge": {}}
    post_log.chmod(0o644)
    post_log.write_text(json.dumps(post, sort_keys=True) + "\n", encoding="utf-8")
    post_log.chmod(0o444)
    with pytest.raises(OUTER.OuterCaptureError, match="host log sidecar"):
        OUTER.verify_outer_root(outer_root, plan)

    outer_root = Path(str(plan["outer_root"]) + "-cleanup")
    plan_cleanup = copy.deepcopy(plan)
    plan_cleanup["outer_root"] = str(outer_root)
    plan_cleanup["outer_binding"]["outer_root"] = str(outer_root)
    plan_cleanup["plan_identity_sha256"] = PLANNER._plan_identity(plan_cleanup)
    plan_cleanup["outer_binding"]["plan_identity_sha256"] = plan_cleanup["plan_identity_sha256"]
    OUTER.seal_outer_receipt(
        outer_root, plan_cleanup, _outer_outputs(plan_cleanup))
    _rewrite_outer_receipt(outer_root, lambda receipt: receipt["cleanup"].__setitem__(
        "remove_exit_status", 1))
    with pytest.raises(OUTER.OuterCaptureError, match="cleanup"):
        OUTER.verify_outer_root(outer_root, plan_cleanup)
