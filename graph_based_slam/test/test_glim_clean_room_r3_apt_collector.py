#!/usr/bin/env python3
"""Synthetic tests for the non-promoting r3 apt collector candidate."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parents[2]
R3 = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


COLLECTOR = _load(R3 / "apt_closure_collector.py", "r3_apt_collector_test")
PLANNER = _load(ROOT / "scripts/plan_glim_clean_room_r3_apt_collector.py",
                "r3_apt_collector_planner_test")
VALIDATOR = _load(ROOT / "scripts/validate_glim_clean_room_r3_apt_collector.py",
                  "r3_apt_collector_validator_test")
OUTER = _load(R3 / "apt_closure_outer.py", "r3_apt_collector_outer_test")


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def _json(path: Path, value: object) -> None:
    _write(path, (json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode())


def _fixture(tmp_path: Path):
    root = tmp_path / "container-root"
    root.mkdir(parents=True)
    base_status = (
        "Package: base-image-runtime\nVersion: 1.0-1\nArchitecture: amd64\n"
        "Status: install ok installed\n\n")
    new_records = [("build-tool", "1.0-1", "amd64"), ("runtime-lib", "2.0-1", "amd64")]
    current_status = base_status + "".join(
        f"Package: {name}\nVersion: {version}\nArchitecture: {arch}\n"
        "Status: install ok installed\n\n" for name, version, arch in new_records)
    # Before provisioning only the base image package is installed.  The
    # fixed snapshot phase captures this byte set; the helper later advances
    # the fixture to ``current_status`` after the synthetic install phase.
    _write(root / "var/lib/dpkg/status", base_status.encode())
    deb_payloads = {}
    for name, version, arch in new_records:
        filename = f"{name}_{version}_{arch}.deb"
        data = (filename + "\nsynthetic deb\n").encode()
        deb_payloads[filename] = data
        _write(root / "var/cache/apt/archives" / filename, data)
        _write(root / f"usr/share/doc/{name}/copyright", f"Copyright {name}\n".encode())
    _write(root / "etc/apt/sources.list", b"deb https://apt.example.invalid/ubuntu main\n")
    _write(root / "etc/apt/sources.list.d/extra.list",
           b"deb https://apt.example.invalid/extra main\n")
    release_path = "var/lib/apt/lists/apt.example.invalid_InRelease"
    _write(root / release_path, b"Suite: synthetic\n")
    packages_path = "var/lib/apt/lists/apt.example.invalid_main_binary-amd64_Packages"
    packages_bytes = b"Package: synthetic\n"
    _write(root / "etc/ros/rosdep/sources.list.d/20-default.list",
           b"yaml https://ros.example.invalid/index.yaml\n")
    _write(root / "var/lib/ros/rosdep/sources.cache", b"cache: synthetic\n")

    keys = [{"name": name, "version": version, "architecture": arch}
            for name, version, arch in new_records]
    allowlist = {"build": [keys[0]], "runtime": [keys[1]], "union": sorted(
        keys, key=lambda item: (item["name"], item["version"], item["architecture"]))}
    allowlist["canonical_sha256"] = COLLECTOR.canonical_hash(allowlist)
    allowlist_path = tmp_path / "package-allowlist.json"
    _json(allowlist_path, allowlist)

    repository_url = "https://apt.example.invalid/ubuntu"
    release_url = "https://apt.example.invalid/ubuntu/InRelease"
    release_sha = _sha(b"Suite: synthetic\n")
    packages_sha = _sha(packages_bytes)
    entries = []
    for name, version, arch in new_records:
        filename = f"{name}_{version}_{arch}.deb"
        entries.append({
            "name": name, "version": version, "architecture": arch,
            "filename": filename, "url": "https://apt.example.invalid/pool/" + filename,
            "bytes": len(deb_payloads[filename]), "sha256": _sha(deb_payloads[filename]),
            "repository_url": repository_url, "release_url": release_url,
            "release_sha256": release_sha, "packages_url": repository_url + "/main/binary-amd64/Packages",
            "packages_path": packages_path, "packages_sha256": packages_sha,
        })
    entries.sort(key=lambda item: (item["name"], item["version"], item["architecture"]))
    ledger = {
        "schema": "glim_clean_room_r3_apt_acquisition_ledger_v1", "schema_version": 1,
        "status": "SEALED_REVIEW_REQUIRED", "entries": entries,
        "repositories": [{"url": repository_url, "release_url": release_url,
                           "release_path": release_path, "release_sha256": release_sha,
                           "packages_url": repository_url + "/main/binary-amd64/Packages",
                           "packages_path": packages_path, "packages_sha256": packages_sha}],
        "apt_source_urls": [
            {"path": "etc/apt/sources.list", "url": "https://apt.example.invalid/sources.list"},
            {"path": "etc/apt/sources.list.d/extra.list",
             "url": "https://apt.example.invalid/extra.list"}],
        "rosdep_source_urls": [{"path": "etc/ros/rosdep/sources.list.d/20-default.list",
                                "url": "https://ros.example.invalid/20-default.list"}],
    }
    ledger["canonical_sha256"] = COLLECTOR.canonical_hash(ledger)
    ledger_path = tmp_path / "apt-acquisition-ledger.json"
    _json(ledger_path, ledger)

    output_root = tmp_path / "host-output"
    plan = PLANNER.plan_collector(output_root=output_root, allowlist=allowlist,
                                  allowlist_path=allowlist_path, ledger_input_path=ledger_path)
    commands = {item["phase"]: item["argv"] for item in plan["workflow"]}

    def dpkg_reader(path: Path) -> dict[str, str]:
        name, version, arch = path.name[:-4].rsplit("_", 2)
        return {"name": name, "version": version, "architecture": arch}

    return {"root": root, "base_status": base_status, "current_status": current_status,
            "allowlist": allowlist, "allowlist_path": allowlist_path,
            "ledger": ledger, "ledger_path": ledger_path, "output_root": output_root,
            "plan": plan, "commands": commands, "dpkg_reader": dpkg_reader,
            "deb_payloads": deb_payloads}


def _prepare_generated_outputs(fixture):
    capture_root = fixture["output_root"]
    capture_root.mkdir(mode=0o700)
    COLLECTOR.snapshot_base(
        container_root=fixture["root"],
        output_path=capture_root / "base-dpkg-status.snapshot")
    COLLECTOR.stage_debs(
        container_root=fixture["root"], ledger_path=fixture["ledger_path"],
        output_root=capture_root / "staged-debs")
    # The fixture models the fixed workflow boundary: snapshot occurs before
    # provisioning, then the installed status gains the allowlisted packages.
    _write(fixture["root"] / "var/lib/dpkg/status", fixture["current_status"].encode())
    return capture_root / "collector-output"


def _collect(fixture, output: Path):
    return COLLECTOR.collect_payload(
        container_root=fixture["root"], allowlist_path=fixture["allowlist_path"],
        ledger_path=fixture["ledger_path"],
        base_status_path=fixture["output_root"] / "base-dpkg-status.snapshot",
        staged_deb_root=fixture["output_root"] / "staged-debs", output_root=output,
        dpkg_reader=fixture["dpkg_reader"])


def _outer_binding(fixture):
    mounts = fixture["plan"]["mounts"]
    return {
        "image_reference": fixture["plan"]["image_reference"],
        "argv": fixture["plan"]["workflow"][5]["argv"],
        "mounts": [{key: item[key] for key in ("source", "destination", "read_only")}
                   for item in (mounts[0], mounts[2], mounts[4])],
        "image_inspect": {"Id": "sha256:" + "b" * 64,
                           "RepoDigests": [fixture["plan"]["image_reference"]],
                           "Os": "linux", "Architecture": "amd64"},
        "plan_identity_sha256": fixture["plan"]["plan_identity_sha256"],
    }


def test_candidate_schema_and_six_mount_workflow(tmp_path):
    fixture = _fixture(tmp_path)
    assert [item["phase"] for item in fixture["plan"]["workflow"]] == [
        "image_inspect", "output_root_absent_check", "output_root_create",
        "output_root_lstat", "preexisting_name_inspect", "container_run",
        "provisioning_update", "acquisition_ledger", "base_status_snapshot",
        "download_only", "stage_downloads", "provisioning_install", "collector",
        "disconnect", "post_disconnect_inspect", "stop", "remove", "post_remove_inspect"]
    assert len(fixture["plan"]["mounts"]) == 6
    assert all(item["destination"] not in {
        "/opt/base-dpkg-status", "/opt/staged-debs", "/opt/apt-collector-plan.json"}
               for item in fixture["plan"]["mounts"])
    document = json.loads((R3 / "apt_closure_collector_candidate.json").read_text())
    schema = json.loads((R3 / "apt_closure_collector_candidate.schema.json").read_text())
    Draft202012Validator(schema).validate(document)
    plan_schema = json.loads((R3 / "apt_closure_collector_plan.schema.json").read_text())
    Draft202012Validator(plan_schema).validate(fixture["plan"])
    assert VALIDATOR.validate()["status"] == "PASS"
    assert OUTER.verify_collector_plan_sources(fixture["plan"])["status"] == "PASS"


def test_collector_v2_reopens_package_policy_and_rejects_unsupported_evidence(tmp_path):
    fixture = _fixture(tmp_path)
    ledger = json.loads(json.dumps(fixture["ledger"]))
    ledger["source_manifest_schema"] = COLLECTOR.SOURCE_SCHEMA_V2
    ledger["source_manifest_schema_version"] = 2
    ledger["packages_format_policy"] = dict(COLLECTOR.PACKAGES_FORMAT_POLICY)
    ledger["apt_source_urls"] = [
        {"path": item["path"], "urls": [item["url"]]}
        for item in ledger["apt_source_urls"]
    ]
    ledger["source_entries"] = [
        {
            "path": item["path"],
            "ordinal": 0,
            "format": "legacy",
            "types": ["deb"],
            "uris": list(item["urls"]),
            "suites": ["main"],
            "components": ["main"],
            "options": [],
            "signed_by": {"kind": "none"},
            "line_start": 1,
            "line_end": 1,
        }
        for item in ledger["apt_source_urls"]
    ]
    ledger["source_entries"].sort(key=lambda item: (item["path"], item["ordinal"]))
    ledger["canonical_sha256"] = COLLECTOR.canonical_hash(
        {key: value for key, value in ledger.items() if key != "canonical_sha256"})
    reopened = COLLECTOR._validate_ledger(ledger)
    assert reopened["packages_format_policy"] == COLLECTOR.PACKAGES_FORMAT_POLICY

    inactive = json.loads(json.dumps(ledger))
    inactive["apt_source_urls"].append({
        "path": "etc/apt/sources.list.d/inactive.list", "urls": []})
    inactive["apt_source_urls"].sort(key=lambda item: item["path"])
    inactive["canonical_sha256"] = COLLECTOR.canonical_hash(
        {key: value for key, value in inactive.items()
         if key != "canonical_sha256"})
    assert COLLECTOR._validate_ledger(inactive)["apt_source_urls"][-1]["urls"] == []

    missing_policy = json.loads(json.dumps(ledger))
    del missing_policy["packages_format_policy"]
    with pytest.raises(COLLECTOR.CollectorError, match="schema/status"):
        COLLECTOR._validate_ledger(missing_policy)

    drifted_policy = json.loads(json.dumps(ledger))
    drifted_policy["packages_format_policy"]["selection_order"] = ["plain"]
    with pytest.raises(COLLECTOR.CollectorError, match="Packages format policy"):
        COLLECTOR._validate_ledger(drifted_policy)

    unsupported = json.loads(json.dumps(ledger))
    unsupported["package_evidence"] = []
    with pytest.raises(COLLECTOR.CollectorError, match="schema/status"):
        COLLECTOR._validate_ledger(unsupported)


def test_outer_rejects_missing_or_mutated_read_only_source(tmp_path):
    fixture = _fixture(tmp_path)
    fixture["ledger_path"].unlink()
    with pytest.raises(OUTER.OuterCaptureError, match="cannot be inspected|mount"):
        OUTER.verify_collector_plan_sources(fixture["plan"])
    fixture = _fixture(tmp_path / "mutated")
    fixture["ledger_path"].write_bytes(b"mutated\n")
    with pytest.raises(OUTER.OuterCaptureError, match="byte identity|canonical"):
        OUTER.verify_collector_plan_sources(fixture["plan"])


def test_workflow_generates_snapshot_and_stage_before_cache_cleanup(tmp_path):
    fixture = _fixture(tmp_path)
    assert not fixture["output_root"].exists()
    output = _prepare_generated_outputs(fixture)
    for path in (fixture["root"] / "var/cache/apt/archives").iterdir():
        path.unlink()
    result = _collect(fixture, output)
    assert result["benchmark_eligible"] is False
    receipt_schema = json.loads((R3 / "apt_closure_collector.schema.json").read_text())
    receipt = json.loads((output / COLLECTOR.RECEIPT_NAME).read_text())
    Draft202012Validator(receipt_schema).validate(receipt)
    payload = json.loads((output / COLLECTOR.PAYLOAD_NAME).read_text())
    assert all(item["source_path"].startswith("raw/staged-debs/")
               for item in payload["artifacts"] if item["role"] == "deb")
    assert not ({"base_image", "network", "provisioning", "container", "plan_identity_sha256"}
                & set(payload))
    sealed = COLLECTOR.seal_to_capture(output, tmp_path / "capture-output", _outer_binding(fixture))
    assert sealed["status"] == "REVIEW_REQUIRED"


def test_collector_promotion_mode_requires_strong_signature_index(tmp_path):
    fixture = _fixture(tmp_path)
    output = _prepare_generated_outputs(fixture)
    _collect(fixture, output)
    binding = _outer_binding(fixture)
    binding["collector_input_allowed"] = True
    with pytest.raises(COLLECTOR.CollectorError, match="signature receipt index"):
        COLLECTOR.seal_to_capture(output, tmp_path / "capture-output", binding)


def test_precreated_generated_outputs_and_wrong_paths_rejected(tmp_path):
    fixture = _fixture(tmp_path)
    fixture["output_root"].mkdir(mode=0o700)
    (fixture["output_root"] / "base-dpkg-status.snapshot").write_bytes(b"precreated")
    (fixture["output_root"] / "staged-debs").mkdir()
    with pytest.raises(COLLECTOR.CollectorError, match="fresh"):
        COLLECTOR.snapshot_base(container_root=fixture["root"],
                                 output_path=fixture["output_root"] / "base-dpkg-status.snapshot")
    with pytest.raises(COLLECTOR.CollectorError, match="fresh"):
        COLLECTOR.stage_debs(container_root=fixture["root"], ledger_path=fixture["ledger_path"],
                             output_root=fixture["output_root"] / "staged-debs")
    with pytest.raises(COLLECTOR.CollectorError, match="fixed workflow"):
        COLLECTOR.collect_payload(
            container_root=fixture["root"], allowlist_path=fixture["allowlist_path"],
            ledger_path=fixture["ledger_path"], base_status_path=tmp_path / "elsewhere",
            staged_deb_root=tmp_path / "elsewhere-stage",
            output_root=fixture["output_root"] / "collector-output",
            dpkg_reader=fixture["dpkg_reader"])


def test_inner_payload_rejects_host_identity_injection(tmp_path):
    fixture = _fixture(tmp_path)
    output = _prepare_generated_outputs(fixture)
    result = _collect(fixture, output)
    payload_path = output / COLLECTOR.PAYLOAD_NAME
    payload = json.loads(payload_path.read_text())
    payload["image_inspect"] = {"Id": "sha256:" + "a" * 64}
    payload_path.chmod(0o644)
    payload_path.write_text(json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n")
    with pytest.raises(COLLECTOR.CollectorError, match="host identity|fields|sidecar"):
        COLLECTOR.verify_collector_output(output)
    assert result["status"] == "REVIEW_REQUIRED"


def test_staged_deb_symlink_hardlink_and_manifest_drift_rejected(tmp_path):
    fixture = _fixture(tmp_path)
    _prepare_generated_outputs(fixture)
    first = next(iter(fixture["deb_payloads"]))
    staged = fixture["output_root"] / "staged-debs" / first
    staged.unlink()
    staged.symlink_to(fixture["root"] / "var/lib/dpkg/status")
    with pytest.raises(COLLECTOR.CollectorError, match="unsafe|identity|single-link"):
        _collect(fixture, fixture["output_root"] / "collector-output")


def test_snapshot_and_stage_are_fresh_and_cache_independent(tmp_path):
    fixture = _fixture(tmp_path)
    output = _prepare_generated_outputs(fixture)
    assert (fixture["output_root"] / "base-dpkg-status.snapshot").is_file()
    assert (fixture["output_root"] / "staged-debs" / "staged-debs.manifest.json").is_file()
    with pytest.raises(COLLECTOR.CollectorError, match="fresh"):
        COLLECTOR.snapshot_base(container_root=fixture["root"],
                                 output_path=fixture["output_root"] / "base-dpkg-status.snapshot")
    with pytest.raises(COLLECTOR.CollectorError, match="fresh"):
        COLLECTOR.stage_debs(container_root=fixture["root"], ledger_path=fixture["ledger_path"],
                             output_root=fixture["output_root"] / "staged-debs")
    for path in (fixture["root"] / "var/cache/apt/archives").iterdir():
        path.unlink()
    assert _collect(fixture, output)["status"] == "REVIEW_REQUIRED"


def test_collector_binds_release_and_deb_artifact_urls_exactly(tmp_path):
    fixture = _fixture(tmp_path)
    output = _prepare_generated_outputs(fixture)
    _collect(fixture, output)
    payload = json.loads((output / COLLECTOR.PAYLOAD_NAME).read_text())
    artifacts = {item["path"]: item for item in payload["artifacts"]}
    release_url = fixture["ledger"]["repositories"][0]["release_url"]
    release_artifacts = [item for item in artifacts.values()
                         if item["role"] == "apt_release"]
    assert len(release_artifacts) == 1
    assert release_artifacts[0]["source_url"] == release_url
    for package in payload["dpkg_packages_sorted"]:
        assert artifacts[package["artifact_path"]]["source_url"] == package["url"]
    assert COLLECTOR._validate_inner_payload(payload)["status"] == "REVIEW_REQUIRED"

    wrong_url = json.loads(json.dumps(payload))
    wrong_url["artifacts"][next(
        index for index, item in enumerate(wrong_url["artifacts"])
        if item["role"] == "apt_release")]["source_url"] = release_url + "/wrong"
    with pytest.raises(COLLECTOR.CollectorError, match="URL binding drift"):
        COLLECTOR._validate_inner_payload(wrong_url)


def test_collector_rejects_release_record_omission_extra_and_duplicate(tmp_path):
    fixture = _fixture(tmp_path)
    output = _prepare_generated_outputs(fixture)
    _collect(fixture, output)
    payload = json.loads((output / COLLECTOR.PAYLOAD_NAME).read_text())

    missing = json.loads(json.dumps(payload))
    missing["apt_release_or_inrelease"] = []
    with pytest.raises(COLLECTOR.CollectorError, match="non-empty"):
        COLLECTOR._validate_inner_payload(missing)

    duplicate = json.loads(json.dumps(payload))
    duplicate["apt_release_or_inrelease"].append(
        json.loads(json.dumps(duplicate["apt_release_or_inrelease"][0])))
    with pytest.raises(COLLECTOR.CollectorError, match="duplicate"):
        COLLECTOR._validate_inner_payload(duplicate)

    extra = json.loads(json.dumps(payload))
    extra_artifact = dict(next(item for item in extra["artifacts"]
                               if item["role"] == "apt_release"))
    extra_artifact["path"] = "raw/extra-release"
    extra_artifact["source_path"] = "raw/extra-release"
    extra["artifacts"].append(extra_artifact)
    extra["artifacts"].sort(key=lambda item: item["path"])
    with pytest.raises(COLLECTOR.CollectorError, match="artifact set"):
        COLLECTOR._validate_inner_payload(extra)
