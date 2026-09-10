#!/usr/bin/env python3
"""Adversarial tests for the Phase 3d r3 offline dependency boundary."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil

import pytest


ROOT = Path(__file__).resolve().parents[2]


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PREFETCH = _load(
    ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3/dependency_prefetch.py",
    "glim_r3_dependency_prefetch")
PLANNER = _load(
    ROOT / "scripts/plan_glim_clean_room_r3_offline_build.py",
    "glim_r3_offline_planner")


def _ready_manifest(tmp_path: Path) -> tuple[Path, dict]:
    tmp_path.mkdir(parents=True, exist_ok=True)
    source = json.loads((ROOT / (
        "docker/benchmark_adapters/glim_clean_room/phase3d/r3/"
        "offline_dependency_manifest.json")).read_text(encoding="utf-8"))
    source["status"] = "READY"
    archive_bytes = {}
    for archive in source["archives"]:
        payload = (archive["name"] + "-fixture\n").encode("utf-8")
        archive["bytes"] = len(payload)
        archive["sha256"] = hashlib.sha256(payload).hexdigest()
        archive_bytes[archive["filename"]] = payload
    deb_payload = b"synthetic-deb-bytes\n"
    deb_sha = hashlib.sha256(deb_payload).hexdigest()
    apt = source["apt_deb_closure"]
    package = {
        "name": "synthetic-build-dependency",
        "version": "1.0-1",
        "architecture": "amd64",
        "status": "installed",
        "filename": "synthetic-build-dependency_1.0-1_amd64.deb",
        "url": "https://packages.example.invalid/synthetic-build-dependency_1.0-1_amd64.deb",
        "sha256": deb_sha,
        "bytes": len(deb_payload),
        "license": {
            "path": "usr/share/doc/synthetic-build-dependency/copyright",
            "identity": "Synthetic test license",
            "sha256": "1" * 64,
        },
    }
    repository = {
        "url": "https://packages.example.invalid/ubuntu",
        "release_or_inrelease_url": "https://packages.example.invalid/ubuntu/InRelease",
        "release_or_inrelease_sha256": "2" * 64,
        "release_or_inrelease_bytes": 64,
    }
    apt.clear()
    apt.update({
        "schema_version": 1,
        "status": "READY",
        "base_image_reference": source["base_image"]["reference"],
        "base_image_digest": source["base_image"]["reference"],
        "platform": "linux/amd64",
        "ros_distribution": "jazzy",
        "provenance_policy": "synthetic fixture only",
        "required_fields": sorted(PREFETCH.READY_REQUIRED_FIELDS),
        "dpkg_packages_sorted": [package],
        "repositories": [repository],
        "apt_source_files_sha256": [{
            "path": "etc/apt/sources.list.d/synthetic.list",
            "sha256": "3" * 64,
            "bytes": 32,
        }],
        "apt_release_or_inrelease": [{
            "url": repository["release_or_inrelease_url"],
            "sha256": repository["release_or_inrelease_sha256"],
            "bytes": repository["release_or_inrelease_bytes"],
        }],
        "rosdep_sources_sha256": [{
            "url": "https://index.example.invalid/rosdep/sources.yaml",
            "sha256": "4" * 64,
            "bytes": 48,
        }],
        "rosdep_cache_sha256": "5" * 64,
        "install_command_sha256": "6" * 64,
        "install_exit_status": 0,
    })
    apt["archive_set_identity_sha256"] = PREFETCH._archive_identity(source)
    apt["dpkg_set_identity_sha256"] = PREFETCH._set_identity(
        [package], "name/version/architecture")
    apt["repository_set_identity_sha256"] = PREFETCH._set_identity(
        [repository], "url/release_or_inrelease_url")
    apt["closure_identity_sha256"] = PREFETCH._identity(apt)
    path = tmp_path / "offline_dependency_manifest.json"
    path.write_text(json.dumps(source, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    return path, {"manifest": source, "archive_bytes": archive_bytes,
                   "deb_filename": package["filename"], "deb_bytes": deb_payload}


def _source_root(tmp_path: Path, manifest_data: dict) -> Path:
    root = tmp_path / "source"
    root.mkdir(parents=True)
    for name, payload in manifest_data["archive_bytes"].items():
        (root / name).write_bytes(payload)
    apt = manifest_data["manifest"]["apt_deb_closure"]
    (root / "apt-deb-closure.json").write_text(json.dumps({
        **apt,
    }, sort_keys=True) + "\n", encoding="utf-8")
    (root / manifest_data["deb_filename"]).write_bytes(manifest_data["deb_bytes"])
    return root


def test_production_manifest_rejects_unsealed_apt_closure(tmp_path):
    manifest = ROOT / (
        "docker/benchmark_adapters/glim_clean_room/phase3d/r3/"
        "offline_dependency_manifest.json")
    with pytest.raises(PREFETCH.PrefetchError, match="APT_CLOSURE_NOT_READY"):
        PREFETCH.prefetch_local(tmp_path / "unused", tmp_path / "output", manifest)


def test_ready_apt_closure_requires_nonempty_sorted_unique_provenance(tmp_path):
    manifest_path, data = _ready_manifest(tmp_path)
    broken = copy.deepcopy(data["manifest"])
    broken["apt_deb_closure"]["dpkg_packages_sorted"] = []
    manifest_path.write_text(json.dumps(broken, sort_keys=True, indent=2) + "\n",
                             encoding="utf-8")
    with pytest.raises(PREFETCH.PrefetchError, match="non-empty"):
        PREFETCH.prefetch_local(tmp_path / "missing-source", tmp_path / "output", manifest_path)

    manifest_path, data = _ready_manifest(tmp_path / "repositories")
    broken = copy.deepcopy(data["manifest"])
    broken["apt_deb_closure"]["repositories"] = []
    manifest_path.write_text(json.dumps(broken, sort_keys=True, indent=2) + "\n",
                             encoding="utf-8")
    with pytest.raises(PREFETCH.PrefetchError, match="non-empty"):
        PREFETCH.prefetch_local(tmp_path / "repositories-source", tmp_path / "output", manifest_path)

    manifest_path, data = _ready_manifest(tmp_path / "duplicate")
    broken = copy.deepcopy(data["manifest"])
    broken["apt_deb_closure"]["dpkg_packages_sorted"] *= 2
    manifest_path.write_text(json.dumps(broken, sort_keys=True, indent=2) + "\n",
                             encoding="utf-8")
    with pytest.raises(PREFETCH.PrefetchError, match="duplicates"):
        PREFETCH.prefetch_local(tmp_path / "duplicate-source", tmp_path / "output", manifest_path)

    manifest_path, data = _ready_manifest(tmp_path / "unsorted")
    broken = copy.deepcopy(data["manifest"])
    second_repository = copy.deepcopy(
        broken["apt_deb_closure"]["repositories"][0])
    second_repository["url"] = "https://aaa.example.invalid/ubuntu"
    second_repository["release_or_inrelease_url"] = (
        "https://aaa.example.invalid/ubuntu/InRelease")
    broken["apt_deb_closure"]["repositories"] = [
        broken["apt_deb_closure"]["repositories"][0], second_repository]
    manifest_path.write_text(json.dumps(broken, sort_keys=True, indent=2) + "\n",
                             encoding="utf-8")
    with pytest.raises(PREFETCH.PrefetchError, match="not canonically sorted"):
        PREFETCH.prefetch_local(tmp_path / "unsorted-source", tmp_path / "output", manifest_path)

    manifest_path, data = _ready_manifest(tmp_path / "missing-field")
    broken = copy.deepcopy(data["manifest"])
    del broken["apt_deb_closure"]["install_command_sha256"]
    manifest_path.write_text(json.dumps(broken, sort_keys=True, indent=2) + "\n",
                             encoding="utf-8")
    with pytest.raises(PREFETCH.PrefetchError, match="install command"):
        PREFETCH.prefetch_local(tmp_path / "missing-field-source", tmp_path / "output", manifest_path)


def test_ready_apt_closure_rejects_self_rehash_and_receipt_identity_tamper(tmp_path):
    manifest_path, data = _ready_manifest(tmp_path)
    source = _source_root(tmp_path, data)
    output = tmp_path / "output"
    PREFETCH.prefetch_local(source, output, manifest_path)
    closure_path = output / "apt-deb-closure.json"
    closure = json.loads(closure_path.read_text(encoding="utf-8"))
    closure["dpkg_packages_sorted"][0]["version"] = "9.9-9"
    closure["closure_identity_sha256"] = PREFETCH._identity(closure)
    closure_path.write_text(json.dumps(closure, sort_keys=True, indent=2) + "\n",
                           encoding="utf-8")
    with pytest.raises(PREFETCH.PrefetchError, match="mixed or mutated"):
        PREFETCH.verify_prefetch_root(output, manifest_path)

    manifest_path, data = _ready_manifest(tmp_path / "receipt")
    source = _source_root(tmp_path / "receipt-source", data)
    output = tmp_path / "receipt-output"
    PREFETCH.prefetch_local(source, output, manifest_path)
    receipt_path = output / "prefetch.receipt.json"
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    receipt["dpkg_set_identity_sha256"] = "f" * 64
    receipt_path.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n",
                           encoding="utf-8")
    with pytest.raises(PREFETCH.PrefetchError, match="canonical identity"):
        PREFETCH.verify_prefetch_root(output, manifest_path)

    manifest_path, data = _ready_manifest(tmp_path / "schema")
    broken = copy.deepcopy(data["manifest"])
    broken["apt_deb_closure"]["schema_version"] = 2
    manifest_path.write_text(json.dumps(broken, sort_keys=True, indent=2) + "\n",
                             encoding="utf-8")
    with pytest.raises(PREFETCH.PrefetchError, match="schema version"):
        PREFETCH.prefetch_local(tmp_path / "schema-source", tmp_path / "schema-output",
                                manifest_path)


def test_prefetch_copy_and_verify_bind_exact_bytes(tmp_path):
    manifest_path, data = _ready_manifest(tmp_path)
    source = _source_root(tmp_path, data)
    output = tmp_path / "prefetched"
    result = PREFETCH.prefetch_local(source, output, manifest_path)
    assert result["status"] == "PASS"
    assert PREFETCH.verify_prefetch_root(output, manifest_path)["status"] == "PASS"
    assert (output / "prefetch.receipt.json").is_file()


def test_prefetch_rejects_missing_extra_and_mixed_set(tmp_path):
    manifest_path, data = _ready_manifest(tmp_path)
    source = _source_root(tmp_path, data)
    missing = next(iter(data["archive_bytes"]))
    (source / missing).unlink()
    with pytest.raises(PREFETCH.PrefetchError, match="file set mismatch"):
        PREFETCH.prefetch_local(source, tmp_path / "missing-output", manifest_path)

    source = _source_root(tmp_path / "extra-fixture", data)
    (source / "unexpected.bin").write_bytes(b"extra")
    with pytest.raises(PREFETCH.PrefetchError, match="file set mismatch"):
        PREFETCH.prefetch_local(source, tmp_path / "extra-output", manifest_path)

    mixed_manifest_path, mixed_data = _ready_manifest(tmp_path / "mixed-manifest")
    mixed_source = _source_root(tmp_path / "mixed-source", mixed_data)
    mixed_manifest = json.loads(mixed_manifest_path.read_text(encoding="utf-8"))
    mixed_manifest["archives"][0]["sha256"] = "0" * 64
    mixed_manifest_path.write_text(
        json.dumps(mixed_manifest, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    with pytest.raises(PREFETCH.PrefetchError, match="archive (set identity drift|byte identity)"):
        PREFETCH.prefetch_local(mixed_source, tmp_path / "mixed-output", mixed_manifest_path)


def test_prefetch_rejects_symlink_hardlink_and_mutation(tmp_path):
    manifest_path, data = _ready_manifest(tmp_path)
    source = _source_root(tmp_path, data)
    target = next(iter(data["archive_bytes"]))
    original = source / target
    original.unlink()
    (source / target).symlink_to("other")
    with pytest.raises(PREFETCH.PrefetchError, match="non-regular entry"):
        PREFETCH.prefetch_local(source, tmp_path / "symlink-output", manifest_path)

    source = _source_root(tmp_path / "hardlink-fixture", data)
    target = next(iter(data["archive_bytes"]))
    (source / target).unlink()
    (source / target).hardlink_to(source / "apt-deb-closure.json")
    with pytest.raises(PREFETCH.PrefetchError, match="hardlink"):
        PREFETCH.prefetch_local(source, tmp_path / "hardlink-output", manifest_path)

    source = _source_root(tmp_path / "valid-fixture", data)
    output = tmp_path / "valid-output"
    PREFETCH.prefetch_local(source, output, manifest_path)
    archive = next(output.glob("*.tar.*"))
    archive.write_bytes(archive.read_bytes() + b"tamper")
    with pytest.raises(PREFETCH.PrefetchError, match="archive byte identity"):
        PREFETCH.verify_prefetch_root(output, manifest_path)


def _build_context(tmp_path: Path) -> Path:
    context = tmp_path / "context"
    r3 = context / "phase3d/r3"
    r3.mkdir(parents=True)
    manifest_path, data = _ready_manifest(tmp_path / "ready")
    source = _source_root(tmp_path / "prefetch-source", data)
    prefetch = context / "prefetch"
    PREFETCH.prefetch_local(source, prefetch, manifest_path)
    shutil.copy2(manifest_path, r3 / "offline_dependency_manifest.json")
    for name in ("Dockerfile", "dependency_prefetch.py"):
        shutil.copy2(ROOT / f"docker/benchmark_adapters/glim_clean_room/phase3d/r3/{name}",
                     r3 / name)
    shutil.copy2(ROOT / (
        "docker/benchmark_adapters/glim_clean_room/phase3d/r3/"
        "offline_dependency_manifest.schema.json"), r3 / "offline_dependency_manifest.schema.json")
    candidate = json.loads((ROOT / (
        "docker/benchmark_adapters/glim_clean_room/phase3d/r3/"
        "candidate_manifest.json")).read_text(encoding="utf-8"))
    candidate["offline_dependency_closure"]["manifest_sha256"] = PREFETCH.sha256_file(
        manifest_path)
    candidate["offline_build_contract"]["recipe_sha256"] = PLANNER.sha256_file(
        r3 / "Dockerfile")
    candidate["offline_build_contract"]["prefetch_tool_sha256"] = PLANNER.sha256_file(
        r3 / "dependency_prefetch.py")
    candidate["offline_build_contract"]["planner_sha256"] = PLANNER.sha256_file(
        ROOT / "scripts/plan_glim_clean_room_r3_offline_build.py")
    (r3 / "candidate_manifest.json").write_text(
        json.dumps(candidate, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    return context


def test_offline_build_plan_is_exact_and_network_mutation_fails(tmp_path):
    context = _build_context(tmp_path)
    plan = PLANNER.plan(context, "candidate:test")
    assert plan["status"] == "PASS"
    assert plan["execute"] is False
    assert plan["argv"][0:6] == [
        "docker", "build", "--network", "none", "--pull=false", "--progress=plain"]
    assert plan["identity"]["benchmark_eligible"] is False
    bad = list(plan["argv"])
    bad[bad.index("none")] = "host"
    with pytest.raises(PLANNER.OfflineBuildError, match="exact offline"):
        PLANNER.validate_exact_build_argv(bad, context=context, tag="candidate:test")
    bad = list(plan["argv"])
    bad.insert(2, "--privileged")
    with pytest.raises(PLANNER.OfflineBuildError, match="exact offline"):
        PLANNER.validate_exact_build_argv(bad, context=context, tag="candidate:test")


def test_offline_build_plan_rejects_mixed_manifest_and_receipt(tmp_path):
    context = _build_context(tmp_path)
    candidate_path = context / "phase3d/r3/candidate_manifest.json"
    candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
    candidate["offline_dependency_closure"]["manifest_sha256"] = "f" * 64
    candidate_path.write_text(json.dumps(candidate, sort_keys=True, indent=2) + "\n",
                              encoding="utf-8")
    with pytest.raises(PLANNER.OfflineBuildError, match="mixed"):
        PLANNER.plan(context, "candidate:test")
