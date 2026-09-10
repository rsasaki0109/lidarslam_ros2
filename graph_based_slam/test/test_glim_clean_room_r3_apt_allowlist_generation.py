#!/usr/bin/env python3
"""Synthetic-only tests for the non-promoting r3 apt allowlist candidate."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import copy
from pathlib import Path
import subprocess

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


COMPOSER = _load(R3 / "apt_allowlist_composer.py", "r3_apt_allowlist_composer_test")
PLANNER = _load(ROOT / "scripts/plan_glim_clean_room_r3_apt_allowlist.py",
                "r3_apt_allowlist_planner_test")
VALIDATOR = _load(ROOT / "scripts/validate_glim_clean_room_r3_apt_allowlist.py",
                  "r3_apt_allowlist_validator_test")


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    # Mutation cases intentionally reopen a sealed synthetic fixture.  The
    # production composer keeps sealed outputs immutable; test-only mutation
    # explicitly changes the mode before replacing bytes.
    if path.exists() and not path.is_symlink():
        path.chmod(0o600)
    path.write_bytes(data)


def _json(path: Path, value: object) -> None:
    _write(path, COMPOSER.canonical_bytes(value) + b"\n")


def _fixture(tmp_path: Path):
    requirements = {
        "schema": COMPOSER.REQUIREMENTS_SCHEMA, "schema_version": 1,
        "status": "PRECOMMITTED",
        "build": [{"name": "build-tool", "architecture": "amd64"}],
        "runtime": [{"name": "runtime-lib", "architecture": "amd64"}],
    }
    requirements["canonical_sha256"] = COMPOSER.canonical_hash(requirements)
    requirements_path = tmp_path / "package-requirements.json"
    _json(requirements_path, requirements)
    base_raw_path = tmp_path / "base-status.raw.tsv"
    _write(base_raw_path, b"base-runtime\t1.0-1\tamd64\tinstall ok installed\n")
    base_path = tmp_path / "base-status.json"
    COMPOSER.seal_base_status_snapshot(raw_path=base_raw_path, output_path=base_path)
    repo = {"url": "https://apt.example.invalid/jazzy",
            "release_url": "https://apt.example.invalid/jazzy/InRelease",
            "release_path": "var/lib/apt/lists/apt.example.invalid_jazzy_InRelease",
            "release_sha256": "0" * 64,
            "packages_url": "https://apt.example.invalid/jazzy/main/binary-amd64/Packages",
            "packages_path": "var/lib/apt/lists/apt.example.invalid_jazzy_main_binary-amd64_Packages",
            "packages_sha256": ""}
    packages = {
        "build": {"name": "build-tool", "version": "1.0-1", "architecture": "amd64",
                   "filename": "build-tool_1.0-1_amd64.deb", **repo,
                   "repository_url": repo["url"],
                   "url": "https://apt.example.invalid/jazzy/pool/build-tool_1.0-1_amd64.deb",
                   "bytes": len(b"build-tool deb\n"), "sha256": _sha(b"build-tool deb\n"),
                   "essential": False, "pre_depends": [], "multiarch": "same", "base_installed": False},
        "runtime": {"name": "runtime-lib", "version": "2.0-1", "architecture": "amd64",
                     "filename": "runtime-lib_2.0-1_amd64.deb", **repo,
                     "repository_url": repo["url"],
                     "url": "https://apt.example.invalid/jazzy/pool/runtime-lib_2.0-1_amd64.deb",
                     "bytes": len(b"runtime-lib deb\n"), "sha256": _sha(b"runtime-lib deb\n"),
                     "essential": False, "pre_depends": ["base-runtime"], "multiarch": "same", "base_installed": False}}
    deb_roots = {}
    for role, package in packages.items():
        root = tmp_path / f"{role}-debs"
        root.mkdir()
        _write(root / package["filename"], (role + ("-tool" if role == "build" else "-lib") + " deb\n").encode())
        # Match the declared bytes/content exactly; this avoids a fake hash.
        content = b"build-tool deb\n" if role == "build" else b"runtime-lib deb\n"
        _write(root / package["filename"], content)
        deb_roots[role] = root
    packages_index = (
        "Package: build-tool\nVersion: 1.0-1\nArchitecture: amd64\n"
        "Filename: pool/build-tool_1.0-1_amd64.deb\nSize: 15\n"
        f"SHA256: {_sha(b'build-tool deb\n')}\n\n"
        "Package: runtime-lib\nVersion: 2.0-1\nArchitecture: amd64\n"
        "Filename: pool/runtime-lib_2.0-1_amd64.deb\nSize: 16\n"
        f"SHA256: {_sha(b'runtime-lib deb\n')}\n\n").encode()
    repo["packages_sha256"] = _sha(packages_index)
    release_bytes = ("Suite: jazzy\nSHA256:\n " + repo["packages_sha256"] +
                     " " + str(len(packages_index)) +
                     " main/binary-amd64/Packages\n").encode()
    repo["release_sha256"] = _sha(release_bytes)
    for package in packages.values():
        package["release_sha256"] = repo["release_sha256"]
    source_root = tmp_path / "apt-root"
    source_files = [
        ("etc/apt/sources.list", "apt_source", "https://apt.example.invalid/sources.list", b"deb https://apt.example.invalid/jazzy main\n"),
        (repo["release_path"], "apt_release", repo["release_url"], release_bytes),
        (repo["packages_path"], "apt_packages", repo["packages_url"], packages_index),
        ("etc/ros/rosdep/sources.list.d/20-default.list", "rosdep_source",
         "https://ros.example.invalid/20-default.list", b"yaml https://ros.example.invalid/index.yaml\n"),
    ]
    source_doc = {"schema": COMPOSER.SOURCE_SCHEMA, "schema_version": 1,
                  "status": "SEALED_REVIEW_REQUIRED", "files": [],
                  "repositories": [repo]}
    for path, role, url, content in source_files:
        _write(source_root / path, content)
        source_doc["files"].append({"path": path, "role": role, "url": url,
                                    "bytes": len(content), "sha256": _sha(content)})
    source_doc["files"].sort(key=lambda item: item["path"])
    source_doc["canonical_sha256"] = COMPOSER.canonical_hash(source_doc)
    source_manifest_path = tmp_path / "apt-source-manifest.json"
    _json(source_manifest_path, source_doc)
    def dpkg_reader(path: Path) -> dict[str, str]:
        if path.name.startswith("build-tool"):
            return {"name": "build-tool", "version": "1.0-1", "architecture": "amd64"}
        return {"name": "runtime-lib", "version": "2.0-1", "architecture": "amd64"}

    resolver_paths = {}
    resolver_log_paths = {}
    for role in ("build", "runtime"):
        package = packages[role]
        raw_log_path = tmp_path / f"{role}-print-uris.log"
        _write(raw_log_path, f"'{package['url']}' {package['filename']} {package['bytes']} SHA256:{package['sha256']}\n".encode())
        resolver_log_paths[role] = raw_log_path
        cache_path = f"/workspace/capture/apt-cache/{role}"
        command = ["apt-get", "--print-uris", "--download-only", "--yes",
                   "--no-install-recommends", "-o", f"Dir::Cache::archives={cache_path}",
                   "install", package["name"]]
        resolver_path = tmp_path / f"{role}-resolver.json"
        COMPOSER.build_resolver_output(
            role=role, raw_log_path=raw_log_path, downloaded_root=deb_roots[role],
            requirements_path=requirements_path, base_status_path=base_path,
            source_root=source_root, source_manifest_path=source_manifest_path,
            command=command, output_path=resolver_path, dpkg_reader=dpkg_reader)
        resolver_paths[role] = resolver_path

    return {"requirements": requirements, "requirements_path": requirements_path,
            "base_path": base_path, "resolver_paths": resolver_paths,
            "resolver_log_paths": resolver_log_paths, "base_raw_path": base_raw_path,
            "source_root": source_root, "source_manifest_path": source_manifest_path,
            "deb_roots": deb_roots, "dpkg_reader": dpkg_reader, "packages": packages,
            "repo": repo}


def _compose(fixture, output: Path):
    return COMPOSER.compose_proposal(
        requirements_path=fixture["requirements_path"], base_status_path=fixture["base_path"],
        build_resolver_path=fixture["resolver_paths"]["build"],
        runtime_resolver_path=fixture["resolver_paths"]["runtime"],
        build_resolver_log_path=fixture["resolver_log_paths"]["build"],
        runtime_resolver_log_path=fixture["resolver_log_paths"]["runtime"],
        source_root=fixture["source_root"], source_manifest_path=fixture["source_manifest_path"],
        build_deb_root=fixture["deb_roots"]["build"], runtime_deb_root=fixture["deb_roots"]["runtime"],
        output_root=output, dpkg_reader=fixture["dpkg_reader"])


def _reseal_release_fixture(fixture, release_bytes: bytes, *, ambiguous_path: bool = False) -> None:
    release_path = fixture["source_root"] / fixture["repo"]["release_path"]
    _write(release_path, release_bytes)
    source = json.loads(fixture["source_manifest_path"].read_text())
    repository = source["repositories"][0]
    repository["release_sha256"] = _sha(release_bytes)
    for item in source["files"]:
        if item["path"] == fixture["repo"]["release_path"]:
            item["bytes"] = len(release_bytes)
            item["sha256"] = _sha(release_bytes)
        if item["role"] == "apt_packages" and ambiguous_path:
            item["url"] = "https://other.example.invalid/jazzy/main/binary-amd64/Packages"
    if ambiguous_path:
        repository["packages_url"] = "https://other.example.invalid/jazzy/main/binary-amd64/Packages"
    source["canonical_sha256"] = COMPOSER.canonical_hash(source, "canonical_sha256")
    _json(fixture["source_manifest_path"], source)


def _build_with_fixture_source(fixture, output: Path) -> None:
    COMPOSER.build_resolver_output(
        role="build", raw_log_path=fixture["resolver_log_paths"]["build"],
        downloaded_root=fixture["deb_roots"]["build"],
        requirements_path=fixture["requirements_path"], base_status_path=fixture["base_path"],
        source_root=fixture["source_root"], source_manifest_path=fixture["source_manifest_path"],
        command=["apt-get", "--print-uris", "--download-only", "--yes", "install", "build-tool"],
        output_path=output, dpkg_reader=fixture["dpkg_reader"])


@pytest.mark.parametrize("mode,pattern", [
    ("missing", "exactly one SHA256"),
    ("duplicate", "duplicate paths"),
    ("ambiguous_path", "not under the repository Release URL"),
])
def test_release_packages_sha256_binding_is_strict(tmp_path, mode, pattern):
    fixture = _fixture(tmp_path)
    packages_sha = fixture["repo"]["packages_sha256"]
    packages_size = (fixture["source_root"] / fixture["repo"]["packages_path"]).stat().st_size
    line = f" {packages_sha} {packages_size} main/binary-amd64/Packages\n".encode()
    if mode == "missing":
        release = b"Suite: jazzy\n"
    elif mode == "duplicate":
        release = b"Suite: jazzy\nSHA256:\n" + line + line
    else:
        release = b"Suite: jazzy\nSHA256:\n" + line
    _reseal_release_fixture(fixture, release, ambiguous_path=mode == "ambiguous_path")
    with pytest.raises(COMPOSER.ComposerError, match=pattern):
        _build_with_fixture_source(fixture, tmp_path / "rejected-resolver.json")


def test_release_declaration_bound_is_separate_and_supports_by_hash(tmp_path):
    fixture = _fixture(tmp_path)
    packages = (fixture["source_root"] / fixture["repo"]["packages_path"]).read_bytes()
    package_digest = fixture["repo"]["packages_sha256"]
    by_hash = "main/binary-amd64/by-hash/SHA256/" + "c" * 64
    release = (
        "Suite: jazzy\nSHA256:\n"
        " {} 829119597 Contents-amd64\n"
        " {} 5869093136 Contents-arm64\n"
        " {} 7240371125 Contents-all\n"
        " {} 0 {}\n"
        " {} {} main/binary-amd64/Packages\n"
    ).format("a" * 64, "a" * 64, "a" * 64, "b" * 64, by_hash,
             package_digest, len(packages)).encode()
    repository = dict(fixture["repo"])
    repository["release_sha256"] = _sha(release)
    COMPOSER._validate_release_packages_binding(release, packages, repository)
    assert COMPOSER.MAX_RELEASE_DECLARED_BYTES > COMPOSER.MAX_TEXT_BYTES
    assert COMPOSER.MAX_RELEASE_DECLARED_BYTES == 17179869184
    assert COMPOSER.MAX_TEXT_BYTES == 64 * 1024 * 1024
    assert COMPOSER.MAX_DECODED_PACKAGES_BYTES == 256 * 1024 * 1024
    assert COMPOSER.PACKAGES_FORMAT_POLICY["compressed_max_bytes"] == \
        COMPOSER.MAX_TEXT_BYTES
    assert COMPOSER.PACKAGES_FORMAT_POLICY["decoded_max_bytes"] == \
        COMPOSER.MAX_DECODED_PACKAGES_BYTES
    assert COMPOSER.MAX_DEB_BYTES == 512 * 1024 * 1024


@pytest.mark.parametrize("size_text", [
    "-1", "+1", "01", "1x", "17179869185",
    "999999999999999999999999999999999999999999999999999999999999999999",
    "１２",
])
def test_release_declaration_size_grammar_is_fail_closed(tmp_path, size_text):
    fixture = _fixture(tmp_path)
    packages = (fixture["source_root"] / fixture["repo"]["packages_path"]).read_bytes()
    release = ("Suite: jazzy\nSHA256:\n {} {} Contents-amd64\n"
               " {} {} main/binary-amd64/Packages\n").format(
                   "a" * 64, size_text, fixture["repo"]["packages_sha256"], len(packages)).encode()
    repository = dict(fixture["repo"])
    repository["release_sha256"] = _sha(release)
    with pytest.raises(COMPOSER.ComposerError, match="size"):
        COMPOSER._validate_release_packages_binding(release, packages, repository)


def test_release_declaration_accepts_clear_signed_and_rejects_conflicting_duplicate(tmp_path):
    fixture = _fixture(tmp_path)
    packages = (fixture["source_root"] / fixture["repo"]["packages_path"]).read_bytes()
    plain = ("Suite: jazzy\nSHA256:\n {} {} main/binary-amd64/Packages\n"
             ).format(fixture["repo"]["packages_sha256"], len(packages))
    signed = ("-----BEGIN PGP SIGNED MESSAGE-----\nHash: SHA256\n\n" + plain +
              "-----BEGIN PGP SIGNATURE-----\nVersion: synthetic\n\n"
              "-----END PGP SIGNATURE-----\n").encode()
    repository = dict(fixture["repo"])
    repository["release_sha256"] = _sha(signed)
    COMPOSER._validate_release_packages_binding(signed, packages, repository)
    duplicate = ("Suite: jazzy\nSHA256:\n {} {} main/binary-amd64/Packages\n"
                 " {} {} main/binary-amd64/Packages\n").format(
                     fixture["repo"]["packages_sha256"], len(packages),
                     "a" * 64, len(packages)).encode()
    repository["release_sha256"] = _sha(duplicate)
    with pytest.raises(COMPOSER.ComposerError, match="duplicate"):
        COMPOSER._validate_release_packages_binding(duplicate, packages, repository)


def test_requirements_plan_and_schemas_are_versionless_and_nonexecuting(tmp_path):
    fixture = _fixture(tmp_path)
    plan = PLANNER.plan(requirements_path=fixture["requirements_path"],
                        output_root=tmp_path / "plan-output")
    assert plan["status"] == "PROPOSED_REVIEW_REQUIRED"
    assert plan["execution"] == "NOT_RUN"
    assert plan["benchmark_eligible"] is False
    assert plan["host_executor"] == {
        "status": "ABSENT_FAIL_CLOSED", "entrypoint": "", "sha256": "0" * 64}
    assert "/workspace/apt/build-cache" != "/workspace/apt/runtime-cache"
    phases = [entry["phase"] for entry in plan["workflow"]]
    assert phases.index("build_resolver") < phases.index("build_download") < phases.index("build_stage")
    assert phases.index("runtime_resolver") < phases.index("runtime_download") < phases.index("runtime_stage")
    assert all("bash" not in entry["argv"] and "-lc" not in entry["argv"] for entry in plan["workflow"])
    Draft202012Validator(json.loads((R3 / "apt_allowlist_generation_plan.schema.json").read_text())).validate(plan)
    for schema_name in ("apt_package_requirements.schema.json", "apt_resolver_output.schema.json",
                        "apt_allowlist_generation_proposal.schema.json",
                        "apt_allowlist_generation_plan.schema.json",
                        "apt_allowlist_generation_outer_receipt.schema.json",
                        "apt_allowlist_review.schema.json"):
        schema = json.loads((R3 / schema_name).read_text())
        Draft202012Validator.check_schema(schema)
    Draft202012Validator(json.loads((R3 / "apt_package_requirements.schema.json").read_text())).validate(fixture["requirements"])


def test_fixed_workflow_has_real_container_mounts_and_host_paths(tmp_path):
    fixture = _fixture(tmp_path)
    plan = PLANNER.plan(requirements_path=fixture["requirements_path"],
                        output_root=tmp_path / "plan-output")
    PLANNER.validate_fixed_workflow(plan)
    phases = plan["workflow"]
    names = [item["phase"] for item in phases]
    run_index = names.index("container_run")
    assert run_index < min(item["ordinal"] for item in phases
                           if item["argv"][:2] == ["docker", "exec"])
    run = phases[run_index]["argv"]
    assert "--mount" in run
    assert f"src={plan['fresh_roots']['output']}" in " ".join(run)
    assert all(item["source_state"] == "EXISTS"
               for item in plan["mounts"] if item["destination"] != "/workspace/capture")
    assert plan["mounts"][0]["source_state"] == "CREATED_BY_WORKFLOW"
    assert plan["fresh_roots"]["build_archive"] != plan["fresh_roots"]["runtime_archive"]
    for item in phases:
        for key in ("host_path", "host_stdout_path", "host_argv_path"):
            if key in item:
                assert "/workspace/" not in item[key]
        for value in item.get("host_paths", []):
            assert value.startswith("/") and "/workspace/" not in value
    for role in ("build", "runtime"):
        resolver = phases[names.index(f"{role}_resolver")]["argv"]
        assert resolver.count("install") == 1
        assert resolver[resolver.index("install") + 1:]
        download = phases[names.index(f"{role}_download")]["argv"]
        assert download.count("install") == 1
    reordered = copy.deepcopy(plan)
    reordered["workflow"][5], reordered["workflow"][6] = (
        reordered["workflow"][6], reordered["workflow"][5])
    with pytest.raises(PLANNER.PlanError, match="phase order|container creation"):
        PLANNER.validate_fixed_workflow(reordered)
    omitted = copy.deepcopy(plan)
    omitted["workflow"].pop(5)
    with pytest.raises(PLANNER.PlanError):
        PLANNER.validate_fixed_workflow(omitted)


@pytest.mark.parametrize("missing_key", ["raw", "base", "deb"])
def test_runtime_generated_inputs_are_required_before_composition(tmp_path, missing_key):
    fixture = _fixture(tmp_path)
    if missing_key == "raw":
        fixture["resolver_log_paths"]["build"].unlink()
    elif missing_key == "base":
        fixture["base_path"].unlink()
    else:
        next(fixture["deb_roots"]["build"].iterdir()).unlink()
    with pytest.raises(COMPOSER.ComposerError):
        _compose(fixture, tmp_path / f"missing-{missing_key}-proposal")


def test_runtime_composer_cli_exposes_raw_resolver_path_without_execution():
    result = subprocess.run(
        ["python3", str(R3 / "apt_allowlist_composer.py"), "--help"],
        check=False, capture_output=True, text=True,
        env={"PATH": "/usr/bin:/bin", "PYTHONDONTWRITEBYTECODE": "1"})
    assert result.returncode == 0
    assert "build-resolver" in result.stdout


def test_candidate_manifest_binds_tools_and_stays_nonpromoting():
    manifest = json.loads((R3 / "apt_allowlist_candidate.json").read_text())
    Draft202012Validator(json.loads((R3 / "apt_allowlist_candidate.schema.json").read_text())).validate(manifest)
    Draft202012Validator(json.loads((R3 / "apt_allowlist_candidate.schema.json").read_text())).validate(
        {**manifest,
         "tool_hashes": {key: "0" * 64 for key in manifest["tool_hashes"]},
         "schemas": {key: "0" * 64 for key in manifest["schemas"]},
         "canonical_sha256": "0" * 64})
    assert manifest["status"] == "OPT_IN_NOT_READY"
    assert manifest["benchmark_eligible"] is False
    assert VALIDATOR.validate()["status"] == "PASS"


def test_fixture_composes_and_reopens_review_only_outputs(tmp_path):
    fixture = _fixture(tmp_path)
    result = _compose(fixture, tmp_path / "proposal")
    assert result["status"] == "PROPOSED_REVIEW_REQUIRED"
    assert result["benchmark_eligible"] is False
    assert COMPOSER.validate_proposal(tmp_path / "proposal")["status"] == "PASS"
    proposal = json.loads((tmp_path / "proposal/apt-allowlist-generation-proposal.json").read_text())
    Draft202012Validator(json.loads((R3 / "apt_allowlist_generation_proposal.schema.json").read_text())).validate(proposal)
    assert proposal["package_allowlist"]["union"] == [
        {"name": "build-tool", "version": "1.0-1", "architecture": "amd64"},
        {"name": "runtime-lib", "version": "2.0-1", "architecture": "amd64"}]
    assert proposal["materialization"]["collector_input"] is False
    assert not (tmp_path / "proposal" / "review.json").exists()


def test_unsigned_or_mismatched_review_cannot_materialize(tmp_path):
    fixture = _fixture(tmp_path)
    _compose(fixture, tmp_path / "proposal")
    proposal_sha = _sha((tmp_path / "proposal/apt-allowlist-generation-proposal.json").read_bytes())
    review = {"schema": COMPOSER.REVIEW_SCHEMA, "schema_version": 1,
              "status": "REVIEWED_FOR_COLLECTOR", "proposal_sha256": proposal_sha,
              "custodian_key_id": "custodian-1", "signature": "",
              "signature_receipt_index_path": str(tmp_path / "signature-index.json"),
              "signature_receipt_index_sha256": "0" * 64}
    review["canonical_sha256"] = COMPOSER.canonical_hash(review)
    review_path = tmp_path / "review.json"
    _json(review_path, review)
    with pytest.raises(COMPOSER.ComposerError, match="non-empty"):
        COMPOSER.materialize_reviewed(proposal_root=tmp_path / "proposal", review_path=review_path,
                                      output_root=tmp_path / "materialized")
    review["signature"] = "detached-test-only-signature"
    review["signature_receipt_index_sha256"] = _sha(b"{}\n")
    review["canonical_sha256"] = COMPOSER.canonical_hash(review, "canonical_sha256")
    _write(tmp_path / "signature-index.json", b"{}\n")
    _json(review_path, review)
    with pytest.raises(COMPOSER.ComposerError, match="signature receipt index|strong per-repository"):
        COMPOSER.materialize_reviewed(proposal_root=tmp_path / "proposal", review_path=review_path,
                                      output_root=tmp_path / "materialized")


@pytest.mark.parametrize("mutation,pattern", [
    ("duplicate_uri", "duplicates"), ("md5", "SHA256 or an MD5"),
    ("percent_name", "filename|basename"), ("uri_spoof", "URI set|URI/package"),
    ("resolver_nonzero", "did not exit"), ("resolver_rehash", "control identity"),
    ("missing_transitive", "absent from resolved"), ("release_drift", "Release binding"),
    ("base_mutation", "base.*drift"), ("version_drift", "control identity"),
])
def test_resolver_and_base_adversaries_fail_closed(tmp_path, mutation, pattern):
    fixture = _fixture(tmp_path)
    role = "build"
    path = fixture["resolver_paths"][role]
    doc = json.loads(path.read_text())
    if mutation == "duplicate_uri":
        doc["uri_lines"].append(doc["uri_lines"][0])
    elif mutation == "md5":
        doc["uri_lines"][0] = doc["uri_lines"][0].replace("SHA256:", "MD5:")
    elif mutation == "percent_name":
        doc["uri_lines"][0] = doc["uri_lines"][0].replace("build-tool_1.0-1_amd64.deb", "wrong%2Edeb")
    elif mutation == "uri_spoof":
        doc["uri_lines"][0] = doc["uri_lines"][0].replace("build-tool_1.0-1_amd64.deb", "runtime-lib_2.0-1_amd64.deb")
    elif mutation == "resolver_nonzero":
        doc["exit_status"] = 1
    elif mutation == "resolver_rehash":
        doc["packages"][0]["version"] = "9.9-1"
    elif mutation == "missing_transitive":
        doc["packages"][0]["name"] = "transitive-missing"
    elif mutation == "release_drift":
        doc["packages"][0]["release_sha256"] = "0" * 64
    elif mutation == "base_mutation":
        base = json.loads(fixture["base_path"].read_text())
        base["packages"][0]["version"] = "9.9-1"
        base["canonical_sha256"] = COMPOSER.canonical_hash(base)
        _json(fixture["base_path"], base)
    elif mutation == "version_drift":
        runtime_path = fixture["resolver_paths"]["runtime"]
        runtime = json.loads(runtime_path.read_text())
        runtime["packages"][0]["version"] = "9.9-1"
        runtime["canonical_sha256"] = COMPOSER.canonical_hash(runtime, "canonical_sha256")
        _json(runtime_path, runtime)
    if mutation != "base_mutation" and mutation != "version_drift":
        doc["canonical_sha256"] = COMPOSER.canonical_hash(doc, "canonical_sha256")
        _json(path, doc)
    with pytest.raises(COMPOSER.ComposerError, match=pattern):
        _compose(fixture, tmp_path / "proposal")


def test_md5_uri_is_only_a_locator_and_packages_sha_binds_download(tmp_path):
    fixture = _fixture(tmp_path)
    role = "build"
    raw = fixture["resolver_log_paths"][role]
    original = raw.read_text()
    digest = hashlib.md5(fixture["deb_roots"][role].joinpath(
        "build-tool_1.0-1_amd64.deb").read_bytes()).hexdigest()
    raw_md5 = tmp_path / "build-print-uris-md5.log"
    _write(raw_md5, original.replace("SHA256:", "MD5Sum:").replace(
        fixture["packages"][role]["sha256"], digest).encode())
    output = tmp_path / "build-md5-resolver.json"
    COMPOSER.build_resolver_output(
        role=role, raw_log_path=raw_md5, downloaded_root=fixture["deb_roots"][role],
        requirements_path=fixture["requirements_path"], base_status_path=fixture["base_path"],
        source_root=fixture["source_root"], source_manifest_path=fixture["source_manifest_path"],
        command=["apt-get", "--print-uris", "--download-only", "--yes", "install", "build-tool"],
        output_path=output, dpkg_reader=fixture["dpkg_reader"])
    resolver = json.loads(output.read_text())
    Draft202012Validator(json.loads((R3 / "apt_resolver_output.schema.json").read_text())).validate(resolver)
    package = resolver["packages"][0]
    assert package["uri_digest_algorithm"] == "md5"
    assert package["uri_digest"] == digest
    assert package["sha256"] == fixture["packages"][role]["sha256"]


def test_md5_locator_cannot_authorize_same_size_mutated_deb(tmp_path):
    fixture = _fixture(tmp_path)
    role = "build"
    raw = fixture["resolver_log_paths"][role]
    original = raw.read_text()
    digest = hashlib.md5(fixture["deb_roots"][role].joinpath(
        "build-tool_1.0-1_amd64.deb").read_bytes()).hexdigest()
    raw_md5 = tmp_path / "build-print-uris-md5.log"
    _write(raw_md5, original.replace("SHA256:", "MD5Sum:").replace(
        fixture["packages"][role]["sha256"], digest).encode())
    deb = fixture["deb_roots"][role] / "build-tool_1.0-1_amd64.deb"
    deb.write_bytes(b"X" * len(deb.read_bytes()))
    with pytest.raises(COMPOSER.ComposerError, match="Packages SHA256"):
        COMPOSER.build_resolver_output(
            role=role, raw_log_path=raw_md5, downloaded_root=fixture["deb_roots"][role],
            requirements_path=fixture["requirements_path"], base_status_path=fixture["base_path"],
            source_root=fixture["source_root"], source_manifest_path=fixture["source_manifest_path"],
            command=["apt-get", "--print-uris", "--download-only", "--yes", "install", "build-tool"],
            output_path=tmp_path / "mutated-md5-resolver.json", dpkg_reader=fixture["dpkg_reader"])


def test_plan_executor_boundary_is_fail_closed(tmp_path):
    fixture = _fixture(tmp_path)
    plan = PLANNER.plan(requirements_path=fixture["requirements_path"],
                        output_root=tmp_path / "plan-output")
    altered = copy.deepcopy(plan)
    altered["host_executor"]["status"] = "IMPLEMENTED"
    with pytest.raises(PLANNER.PlanError, match="executor"):
        PLANNER.validate_fixed_workflow(altered)


def test_deb_control_hash_extra_and_symlink_fail_closed(tmp_path):
    fixture = _fixture(tmp_path)
    _write(fixture["deb_roots"]["build"] / "extra.deb", b"extra")
    with pytest.raises(COMPOSER.ComposerError, match="extra"):
        _compose(fixture, tmp_path / "proposal")
    fixture = _fixture(tmp_path / "symlink")
    deb = fixture["deb_roots"]["runtime"] / "runtime-lib_2.0-1_amd64.deb"
    deb.unlink()
    deb.symlink_to(fixture["deb_roots"]["build"] / "build-tool_1.0-1_amd64.deb")
    with pytest.raises(COMPOSER.ComposerError, match="symlink"):
        _compose(fixture, tmp_path / "symlink-proposal")


def test_deb_bytes_and_source_release_mutation_fail_closed(tmp_path):
    fixture = _fixture(tmp_path)
    deb = fixture["deb_roots"]["build"] / "build-tool_1.0-1_amd64.deb"
    deb.write_bytes(b"mutated deb bytes\n")
    with pytest.raises(COMPOSER.ComposerError, match="bytes/SHA"):
        _compose(fixture, tmp_path / "deb-mutation")
    fixture = _fixture(tmp_path / "release")
    release = fixture["source_root"] / fixture["repo"]["release_path"]
    release.write_bytes(b"tampered Release\n")
    with pytest.raises(COMPOSER.ComposerError, match="byte/SHA"):
        _compose(fixture, tmp_path / "release-mutation")


def test_predepends_must_be_in_resolved_or_base(tmp_path):
    fixture = _fixture(tmp_path)
    resolver_path = fixture["resolver_paths"]["runtime"]
    resolver = json.loads(resolver_path.read_text())
    resolver["packages"][0]["pre_depends"] = ["not-in-closure"]
    resolver["canonical_sha256"] = COMPOSER.canonical_hash(resolver, "canonical_sha256")
    _json(resolver_path, resolver)
    with pytest.raises(COMPOSER.ComposerError, match="Pre-Depends"):
        _compose(fixture, tmp_path / "predepends")


def test_base_installed_identity_is_not_acquired_or_ledgered(tmp_path):
    fixture = _fixture(tmp_path)
    resolver_path = fixture["resolver_paths"]["build"]
    resolver = json.loads(resolver_path.read_text())
    base_package = {"name": "base-runtime", "version": "1.0-1", "architecture": "amd64",
                    "filename": "", "url": "", "bytes": 0, "sha256": "",
                    "repository_url": fixture["repo"]["url"], "release_url": fixture["repo"]["release_url"],
                    "release_path": fixture["repo"]["release_path"],
                    "release_sha256": fixture["repo"]["release_sha256"],
                    "packages_url": fixture["repo"]["packages_url"],
                    "packages_path": fixture["repo"]["packages_path"],
                    "packages_sha256": fixture["repo"]["packages_sha256"],
                    "uri_digest_algorithm": "none", "uri_digest": "", "essential": True,
                    "pre_depends": [], "multiarch": "same", "base_installed": True}
    resolver["packages"] = [base_package, resolver["packages"][0]]
    resolver["canonical_sha256"] = COMPOSER.canonical_hash(resolver, "canonical_sha256")
    _json(resolver_path, resolver)
    result = _compose(fixture, tmp_path / "base-installed")
    proposal = json.loads((tmp_path / "base-installed/apt-allowlist-generation-proposal.json").read_text())
    assert all(item["name"] != "base-runtime" for item in proposal["package_allowlist"]["union"])
    assert result["benchmark_eligible"] is False


def test_source_checkout_tools_are_fixture_only_and_have_no_execution_api():
    for path in (R3 / "apt_allowlist_composer.py", ROOT / "scripts/plan_glim_clean_room_r3_apt_allowlist.py"):
        text = path.read_text()
        assert "subprocess" not in text
        assert "urllib.request" not in text
        assert "socket" not in text


def test_uri_parser_binds_percent_encoding_and_rejects_ambiguous_urls():
    digest = "a" * 64
    line = ("'https://apt.example.invalid/pool/build-tool_1.0-1_amd64%2Edeb' "
            f"build-tool_1.0-1_amd64.deb 15 SHA256:{digest}")
    assert COMPOSER.parse_print_uris([line], "fixture")[0]["filename"] == \
        "build-tool_1.0-1_amd64.deb"
    with pytest.raises(COMPOSER.ComposerError, match="ambiguous"):
        COMPOSER.parse_print_uris([line.replace("%2Edeb'", "%2Edeb?token=1'")], "fixture")

    epoch_line = (
        "'https://apt.example.invalid/pool/libcap2-bin_2.44-1_amd64.deb' "
        f"libcap2-bin_1%3a2.44-1_amd64.deb 15 SHA256:{digest}")
    assert COMPOSER.parse_print_uris([epoch_line], "fixture")[0]["filename"] == \
        "libcap2-bin_1%3a2.44-1_amd64.deb"
    with pytest.raises(COMPOSER.ComposerError, match="basename/percent encoding"):
        COMPOSER.parse_print_uris(
            [epoch_line.replace("_2.44-1_", "_2.45-1_")], "fixture")

    zeta = line.replace("build-tool", "zeta-tool")
    with pytest.raises(COMPOSER.ComposerError, match="canonically sorted"):
        COMPOSER.parse_print_uris([zeta, line], "fixture")
    parsed = COMPOSER.parse_print_uris(
        [zeta, line], "fixture", require_canonical=False)
    assert [item["filename"] for item in parsed] == [
        "zeta-tool_1.0-1_amd64.deb", "build-tool_1.0-1_amd64.deb"]


def test_same_name_different_version_or_architecture_is_explicitly_rejected(tmp_path):
    fixture = _fixture(tmp_path)
    with pytest.raises(COMPOSER.ComposerError, match="different versions"):
        altered = dict(fixture["packages"]["runtime"])
        altered["name"] = "build-tool"
        altered["version"] = "9.9-1"
        COMPOSER._check_cross_role({
            "build": [fixture["packages"]["build"]],
            "runtime": [altered],
        })


def test_live_apt_source_descriptor_modes_are_exact():
    descriptor = {
        "path": "/etc/apt/sources.list.d/ros2.sources",
        "bytes": 1793,
        "sha256": "a" * 64,
        "mode": 0o777,
        "uid": 0,
        "gid": 0,
        "nlink": 1,
        "device": 79,
        "inode": 123,
    }
    COMPOSER._validate_descriptor_stat(
        descriptor, "symlink", absolute_path=True,
        allowed_modes=frozenset({0o777}))

    target = dict(descriptor, path="/usr/share/ros-apt-source/ros2.sources",
                  mode=0o644)
    COMPOSER._validate_descriptor_stat(
        target, "target", absolute_path=True,
        allowed_modes=frozenset({0o444, 0o644}))

    for mode in (0o775, 0o644, 0o444):
        unsafe = dict(descriptor, mode=mode)
        with pytest.raises(COMPOSER.ComposerError, match="metadata is unsafe"):
            COMPOSER._validate_descriptor_stat(
                unsafe, "symlink", absolute_path=True,
                allowed_modes=frozenset({0o777}))


def test_container_transport_source_is_not_confused_with_host_namespace():
    descriptor = {
        "path": "/etc/apt/sources.list.d/ros2.sources",
        "logical_path": "etc/apt/sources.list.d/ros2.sources",
    }
    assert COMPOSER._is_container_transport_source(descriptor) is True
    descriptor["path"] = "/tmp/container/etc/apt/sources.list.d/ros2.sources"
    assert COMPOSER._is_container_transport_source(descriptor) is False


def test_v2_parser_retains_comment_only_source_as_inactive_file():
    assert COMPOSER._parse_source_entries(
        b"# Sources moved to ubuntu.sources\n",
        "inactive apt source", COMPOSER.APT_SOURCE_URL_POLICY) == []
    assert COMPOSER._source_snapshot_urls(
        [], is_v2=True, role="apt_source") == []


@pytest.mark.parametrize("urls,is_v2,role", [
    ([], False, "apt_source"),
    ([], True, "apt_release"),
    (["https://b.invalid", "https://a.invalid"], True, "apt_source"),
    (["https://a.invalid", "https://a.invalid"], True, "apt_source"),
    ([1], True, "apt_source"),
])
def test_source_snapshot_url_projection_rejects_noncanonical_or_active_empty(
        urls, is_v2, role):
    with pytest.raises(COMPOSER.ComposerError, match="sorted and unique"):
        COMPOSER._source_snapshot_urls(urls, is_v2=is_v2, role=role)


def test_v2_parser_ignores_deb822_comments_with_field_colons():
    records = COMPOSER._parse_source_entries(
        b"# Types: deb-src disabled\n"
        b"Types: deb\n"
        b"URIs: https://apt.example.invalid/ubuntu\n"
        b"Suites: jazzy\n"
        b"# Components: duplicate-looking comment\n"
        b"Components: main\n",
        "commented apt source", COMPOSER.APT_SOURCE_URL_POLICY)
    assert len(records) == 1
    assert records[0]["line_start"] == 2
    assert records[0]["line_end"] == 6


def test_packages_reader_accepts_only_release_bound_signed_empty(tmp_path):
    root = tmp_path / "source"
    path = root / "indices/Packages-empty"
    _write(path, b"")
    path.chmod(0o444)
    repository = {
        "packages_path": "indices/Packages-empty",
        "packages_sha256": COMPOSER.EMPTY_PACKAGES_SHA256,
    }
    assert COMPOSER._read_packages_index(root, repository) == {}

    repository["packages_sha256"] = "a" * 64
    with pytest.raises(COMPOSER.ComposerError, match="bounded single-link"):
        COMPOSER._read_packages_index(root, repository)


def test_packages_reader_does_not_reject_unselected_large_declaration(tmp_path):
    root = tmp_path / "source"
    data = (
        "Package: unrelated-large\nVersion: 1\nArchitecture: amd64\n"
        "Filename: pool/unrelated-large_1_amd64.deb\n"
        f"Size: {COMPOSER.MAX_DEB_BYTES + 1}\nSHA256: {'a' * 64}\n\n"
    ).encode()
    path = root / "indices/Packages"
    _write(path, data)
    path.chmod(0o444)
    repository = {
        "packages_path": "indices/Packages",
        "packages_sha256": _sha(data),
    }
    records = COMPOSER._read_packages_index(root, repository)
    assert records[("unrelated-large", "1", "amd64")]["bytes"] == \
        COMPOSER.MAX_DEB_BYTES + 1


def test_packages_reader_accepts_empty_optional_value_with_continuation(tmp_path):
    root = tmp_path / "source"
    data = (
        "Package: btm\nVersion: 0.9.6-4\nArchitecture: amd64\n"
        "Filename: pool/universe/b/btm/btm_0.9.6-4_amd64.deb\n"
        f"Size: 1607224\nSHA256: {'a' * 64}\n"
        "X-Cargo-Built-Using:\n"
        " rust-addr2line (= 0.21.0-2), rust-adler (= 1.0.2-2)\n"
    ).encode("ascii")
    path = root / "indices/Packages"
    _write(path, data)
    path.chmod(0o444)
    repository = {
        "packages_path": "indices/Packages",
        "packages_sha256": _sha(data),
    }

    records = COMPOSER._read_packages_index(root, repository)

    assert ("btm", "0.9.6-4", "amd64") in records


@pytest.mark.parametrize("invalid", [
    "X-Cargo-Built-Using:\n",
    "Package:\n continued\n",
    "Bad_Field:\n continued\n",
])
def test_packages_reader_rejects_unbound_or_identity_empty_fields(tmp_path, invalid):
    root = tmp_path / "source"
    data = (
        "Package: valid\nVersion: 1.0\nArchitecture: amd64\n"
        "Filename: pool/v/valid.deb\nSize: 1\n"
        f"SHA256: {'a' * 64}\n{invalid}"
    ).encode("ascii")
    path = root / "indices/Packages"
    _write(path, data)
    path.chmod(0o444)
    repository = {
        "packages_path": "indices/Packages",
        "packages_sha256": _sha(data),
    }

    with pytest.raises(COMPOSER.ComposerError, match="malformed|duplicate/empty"):
        COMPOSER._read_packages_index(root, repository)


def test_download_housekeeping_must_be_exact_and_empty(tmp_path):
    root = tmp_path / "debs"
    root.mkdir(mode=0o700)
    partial = root / "partial"
    partial.mkdir(mode=0o700)
    lock = root / "lock"
    _write(lock, b"")
    lock.chmod(0o640)
    deb = root / "demo_1_amd64.deb"
    _write(deb, b"deb")
    children = COMPOSER._exclude_download_housekeeping(
        list(root.iterdir()), root.lstat(), "build")
    assert children == [deb]

    _write(partial / "residue", b"unexpected")
    with pytest.raises(COMPOSER.ComposerError, match="partial directory is unsafe"):
        COMPOSER._exclude_download_housekeeping(
            list(root.iterdir()), root.lstat(), "build")

    partial_file = partial / "residue"
    partial_file.chmod(0o600)
    partial_file.unlink()
    lock.chmod(0o600)
    with pytest.raises(COMPOSER.ComposerError, match="downloaded lock is unsafe"):
        COMPOSER._exclude_download_housekeeping(
            list(root.iterdir()), root.lstat(), "build")


def test_top_level_requirement_may_be_satisfied_by_sealed_base_status():
    requirements = {
        "build": [{"name": "cmake", "architecture": "amd64"}],
    }
    base = {
        ("cmake", "3.22.1", "amd64"): {
            "name": "cmake", "version": "3.22.1", "architecture": "amd64",
        },
    }
    COMPOSER._check_resolved_requirements(
        requirements, [], base, "build")
    with pytest.raises(COMPOSER.ComposerError, match="resolved/base set"):
        COMPOSER._check_resolved_requirements(
            requirements, [], {}, "build")

    all_arch_requirement = {
        "build": [{
            "name": "python3-colcon-common-extensions",
            "architecture": "amd64",
        }],
    }
    all_arch_package = {
        "name": "python3-colcon-common-extensions", "version": "1",
        "architecture": "all", "base_installed": False, "pre_depends": [],
    }
    COMPOSER._check_resolved_requirements(
        all_arch_requirement, [all_arch_package], {}, "build")


def test_deb_reopen_accepts_fixed_reader_security_metadata(tmp_path):
    root = tmp_path / "debs"
    root.mkdir(mode=0o700)
    data = b"synthetic deb"
    path = root / "demo_1_all.deb"
    _write(path, data)
    path.chmod(0o444)
    package = {
        "name": "demo", "version": "1", "architecture": "all",
        "filename": path.name, "bytes": len(data), "sha256": _sha(data),
        "base_installed": False,
    }

    def reader(_path):
        return {
            "name": "demo", "version": "1", "architecture": "all",
            "essential": False, "pre_depends": [], "multiarch": "foreign",
        }

    COMPOSER._validate_deb_root(root, [package], "build", reader)
