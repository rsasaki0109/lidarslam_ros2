#!/usr/bin/env python3
"""Tests for the review-only dependency-closure capture executor."""

from __future__ import annotations

import importlib.util
import copy
import json
import lzma
import os
from pathlib import Path

from jsonschema import Draft202012Validator
import pytest


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "scripts/capture_registration_plugin_dependency_closure.py"
CLOSURE_MODULE_PATH = ROOT / "scripts/registration_plugin_dependency_closure.py"
RECEIPT_SCHEMA_PATH = ROOT / (
    "configs/slam_benchmark_profiles/"
    "registration_plugin_dependency_capture_executor_v1.schema.json"
)
SOURCE_SCHEMA_PATH = ROOT / (
    "configs/slam_benchmark_profiles/"
    "registration_plugin_apt_source_snapshot_v2.schema.json"
)
COMPOSER_PATH = ROOT / (
    "docker/benchmark_adapters/glim_clean_room/phase3d/r3/"
    "apt_allowlist_composer.py"
)


def _load():
    spec = importlib.util.spec_from_file_location(
        "capture_registration_plugin_dependency_closure", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_closure():
    spec = importlib.util.spec_from_file_location(
        "registration_plugin_dependency_closure_for_capture_test", CLOSURE_MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_composer():
    spec = importlib.util.spec_from_file_location(
        "registration_plugin_apt_allowlist_composer_for_capture_test",
        COMPOSER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _receipt_validator():
    schema = json.loads(RECEIPT_SCHEMA_PATH.read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(schema)
    return Draft202012Validator(schema)


def _source_validator():
    schema = json.loads(SOURCE_SCHEMA_PATH.read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(schema)
    return Draft202012Validator(schema)


@pytest.fixture(scope="module")
def capture():
    return _load()


def test_descriptor_stat_distinguishes_exact_symlink_mode(capture):
    descriptor = {
        "path": "/capture/etc/apt/sources.list.d/ros2.sources",
        "bytes": 1,
        "sha256": "0" * 64,
        "mode": 0o777,
        "uid": 0,
        "gid": 0,
        "nlink": 1,
        "device": 1,
        "inode": 1,
    }

    capture._validate_descriptor_stat(
        descriptor, "exact symlink", absolute_path=True, exact_symlink=True)
    with pytest.raises(capture.CaptureError):
        capture._validate_descriptor_stat(
            descriptor, "regular file", absolute_path=True)

    for invalid_mode in (0o755, 0o775, 0o666):
        invalid = dict(descriptor, mode=invalid_mode)
        with pytest.raises(capture.CaptureError):
            capture._validate_descriptor_stat(
                invalid, "exact symlink", absolute_path=True, exact_symlink=True)

    regular = dict(descriptor, mode=0o644)
    capture._validate_descriptor_stat(
        regular, "regular file", absolute_path=True)


def test_verified_inline_keyring_binds_declared_and_observed_fingerprint(capture):
    signer = "A" * 40
    inline = {"kind": "inline"}
    capture._bind_verified_keyring_signer(inline, signer)
    assert inline == {
        "kind": "inline", "fingerprint": signer, "signer_fingerprint": signer}

    path = {"kind": "path"}
    capture._bind_verified_keyring_signer(path, signer)
    assert path == {"kind": "path", "signer_fingerprint": signer}

    with pytest.raises(capture.CaptureError) as error:
        capture._bind_verified_keyring_signer({"kind": "inline"}, "not-a-fingerprint")
    assert error.value.kind == "KEYRING_SIGNER_BINDING_INVALID"


def test_inline_observed_fingerprint_does_not_change_input_keyring_identity(capture):
    digest = "b" * 64
    descriptor = {
        "kind": "inline", "source": "inline", "path": "apt-keyrings/key.asc",
        "bytes": 1, "sha256": digest, "normalized_armor_bytes": 1,
        "normalized_armor_sha256": digest, "dearmored_path": "apt-keyrings/key.gpg",
        "dearmored_bytes": 1, "dearmored_sha256": "c" * 64,
    }
    source_entry = {
        "kind": "inline", "source": "inline", "sha256": digest,
        "fingerprint": "A" * 40,
    }
    original = dict(source_entry)
    original.pop("fingerprint")
    assert capture._source_entry_keyring_identity(source_entry, descriptor) == (
        capture._keyring_identity("inline:" + digest, original, descriptor))

    path_entry = {
        "kind": "path", "source": "/usr/share/keyrings/example.gpg",
        "fingerprint": "A" * 40,
    }
    assert capture._source_entry_keyring_identity(path_entry, descriptor) == (
        capture._keyring_identity(path_entry["source"], path_entry, descriptor))


def test_signed_package_index_allows_identical_cross_suite_attestations(capture):
    data = b"exact deb"
    digest = capture.sha256_bytes(data)
    identity = ("demo", "1.0", "amd64")
    package = {
        "filename": "pool/main/d/demo/demo_1.0_amd64.deb",
        "bytes": len(data), "sha256": digest,
    }

    def index(suite, candidate=package):
        return {
            "repository": {
                "url": "https://archive.example.invalid/ubuntu",
                "release_url": "https://archive.example.invalid/ubuntu/dists/{}/InRelease".format(suite),
                "packages_url": "https://archive.example.invalid/ubuntu/dists/{}/main/Packages".format(suite),
                "packages_path": "indices/Packages-{}".format(suite),
            },
            "packages": {identity: candidate},
        }

    uri = {
        "url": "https://archive.example.invalid/ubuntu/" + package["filename"],
        "filename": "demo_1.0_amd64.deb",
        "url_filename": "demo_1.0_amd64.deb",
    }
    selected, observed = capture._select_signed_package_index(
        [index("jammy-updates"), index("jammy")], uri, identity, data)
    assert selected["repository"]["release_url"].endswith("/jammy-updates/InRelease")
    assert observed == package

    conflicting = dict(package, sha256="f" * 64)
    selected, _ = capture._select_signed_package_index(
        [index("jammy", conflicting), index("jammy-updates")], uri, identity, data)
    assert selected["repository"]["release_url"].endswith("/jammy-updates/InRelease")

    with pytest.raises(capture.CaptureError) as error:
        capture._select_signed_package_index(
            [index("jammy", conflicting)], uri, identity, data)
    assert error.value.kind == "PACKAGE_INDEX_BINDING_DRIFT"


def test_repository_index_storage_key_distinguishes_release_components(capture):
    base = "https://archive.example.invalid/ubuntu"
    release = base + "/dists/jammy/InRelease"
    main = capture._repository_index_storage_key(
        base, release, base + "/dists/jammy/main/Packages")
    universe = capture._repository_index_storage_key(
        base, release, base + "/dists/jammy/universe/Packages")
    assert main != universe
    assert len({main, universe}) == 2


def _synthetic_inputs(module, distro, dependency_leg):
    digest = "sha256:" + "a" * 64
    dependency = {
        "name": "fast_gicp",
        "archive_url": "https://archives.example.invalid/fast_gicp.tar.gz",
        "archive_sha256": "b" * 64,
        "official_url": "https://github.example.invalid/fast_gicp",
        "commit": "c" * 40,
        "archive_top_level": "fast_gicp-main",
        "source_tree_sha256": "d" * 64,
        "license_path": "LICENSE",
        "license_sha256": "e" * 64,
    }
    leg = {"status": "READY_PINNED", "dependencies": [dependency]}
    if dependency_leg == "absent":
        leg = {"status": "ABSENT", "dependencies": []}
    image = {
        "reference": "docker.io/library/ros:{}-base@{}".format(distro, digest),
        "digest": digest,
    }
    rows = [{"distro": name, "dependency_leg": leg_name}
            for name, leg_name in module.ROWS]
    campaign_set = {
        "schema": "registration-plugin-campaign-set-v1",
        "schema_version": 1,
        "campaign_id": "synthetic-campaign",
        "rows": rows,
        "row_set_sha256": "1" * 64,
        "identity_sha256": "2" * 64,
    }
    return {
        "profile": {"campaign_set": campaign_set},
        "profile_path": "/tmp/synthetic-profile.json",
        "profile_sha256": "f" * 64,
        "release": {"campaign_set": campaign_set},
        "source": {
            "status": "PASS",
            "hash_kind": "relative_path_content_sha256_v1",
            "files": [{
                "path": "synthetic.py", "expected_sha256": "0" * 64,
                "status": "PASS", "actual_sha256": "0" * 64, "size_bytes": 1,
            }],
            "manifest_sha256": "1" * 64,
            "mismatches": [],
        },
        "image": image,
        "leg": leg,
    }


def _plan(capture, tmp_path, monkeypatch, distro="humble", dependency_leg="absent"):
    repo = tmp_path / "repo"
    repo.mkdir()
    profile = tmp_path / "profile.json"
    profile.write_text("{}\n", encoding="utf-8")
    monkeypatch.setattr(
        capture, "_profile_inputs",
        lambda repo_root, profile_path, selected_distro, selected_leg:
        _synthetic_inputs(capture, selected_distro, selected_leg))
    return capture.build_plan(
        repo, profile, distro, dependency_leg, tmp_path / "output")


def _image_output(plan):
    image = plan["image"]
    return json.dumps([{
        "Id": image["digest"], "RepoDigests": [image["reference"]],
        "Os": "linux", "Architecture": "amd64",
    }], separators=(",", ":")).encode("utf-8")


def _runner(capture, plan, output_root, *, fail_phase=None, cleanup_ok=True):
    calls = []
    inspect_count = 0

    def run(argv, timeout):
        del timeout
        nonlocal inspect_count
        calls.append(list(argv))
        if tuple(argv) == tuple(plan["phase_argv"]["preexisting_container_check"]):
            inspect_count += 1
            if inspect_count == 1:
                return 1, b"", (
                    "error: no such object: {}\n".format(plan["container_name"])
                ).encode("ascii"), False, None
            if inspect_count == 2:
                return 0, b"{}\n", b"", False, None
            if inspect_count == 3 and cleanup_ok:
                return 0, b"{}\n", b"", False, None
            if inspect_count >= 4:
                return 1, b"", (
                    "error: no such object: {}\n".format(plan["container_name"])
                ).encode("ascii"), False, None
            return 1, b"", b"daemon failure\n", False, None
        phase_for_argv = {
            tuple(value): phase for phase, value in plan["phase_argv"].items()
        }
        phase = phase_for_argv.get(tuple(argv))
        if phase == "image_inspect":
            return 0, _image_output(plan), b"", False, None
        if argv[:3] == ["docker", "stop", plan["container_name"]]:
            return (0, b"", b"", False, None) if cleanup_ok else (1, b"", b"stop\n", False, None)
        if argv[:3] == ["docker", "rm", plan["container_name"]]:
            return 0, b"", b"", False, None
        if phase == "base_status_snapshot":
            data = b"base-package\t1.0\tamd64\tinstall ok installed\n"
            return (2, data, b"failed\n", False, None) if fail_phase == phase else (0, data, b"", False, None)
        if phase in {"apt_source_snapshot", "dependency_capture"}:
            name = "in-container-apt-source.json" if phase == "apt_source_snapshot" else "in-container-dependency.json"
            value = {
                "schema": capture.IN_CONTAINER_SCHEMA,
                "schema_version": capture.IN_CONTAINER_SCHEMA_VERSION,
                "phase": phase, "distro": plan["distro"],
                "status": "CAPTURED_REVIEW_REQUIRED",
                "directory_contract": capture._snapshot_capture_directory_layout(
                    output_root, include_prefetch=False),
            }
            value["canonical_sha256"] = capture.canonical_hash(value)
            capture._seal_pair(output_root, name, value)
        if phase == fail_phase:
            return 2, b"", b"phase failed\n", False, None
        return 0, (phase + " ok\n").encode("ascii"), b"", False, None

    return run, calls


def test_plan_has_fixed_image_and_inner_resolver_projection(capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch)
    assert plan["image"]["reference"].endswith("@" + plan["image"]["digest"])
    assert plan["run_argv"][-3:] == ["tail", "-f", "/dev/null"]
    assert plan["run_argv"].count("--pull=never") == 1
    assert plan["run_argv"].count("--network") == 1
    assert plan["phase_argv"]["resolver_build"][3] == "apt-get"
    assert "install" in plan["phase_argv"]["resolver_build"]
    assert "bash" not in plan["phase_argv"]["resolver_build"]
    for phase in ("resolver_build", "resolver_runtime"):
        assert plan["phase_argv"][phase].count("--quiet=2") == 1
        assert plan["phase_argv"][phase].index("--quiet=2") < (
            plan["phase_argv"][phase].index("--print-uris"))
    for phase in ("resolver_build", "download_build", "resolver_runtime",
                  "download_runtime", "dependency_install"):
        assert plan["phase_argv"][phase].count("--no-upgrade") == 1
    install = plan["phase_argv"]["dependency_install"]
    assert install.count("--no-download") == 1
    assert "Dir::Cache::archives=/workspace/evidence-parent/debs-build" in install
    assert install[install.index("APT::Sandbox::User=root") - 1] == "-o"
    assert install.count("APT::Sandbox::User=root") == 1
    assert "debs-runtime" not in " ".join(install)


def test_jazzy_plan_passes_distro_to_offline_prepare(capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch, distro="jazzy")
    argv = plan["phase_argv"]["rosdep_prepare"]

    assert argv[argv.index("--distro") + 1] == "jazzy"
    assert plan["rosdep_prepare_argv"]["argv"] == argv


def test_provisioning_roots_close_project_rosdeps_for_each_distro(capture):
    common = {"libpcl-dev", "python3-yaml", *capture.BOOTSTRAP_PACKAGES}
    for distro in ("humble", "jazzy"):
        packages = capture._provisioning_packages(distro)
        assert packages == tuple(sorted(set(packages)))
        assert common.issubset(packages)
        assert {
            "ros-{}-{}".format(distro, suffix)
            for suffix in capture.PROJECT_ROS_APT_SUFFIXES
        }.issubset(packages)
        other = "jazzy" if distro == "humble" else "humble"
        assert not any(name.startswith("ros-{}-".format(other))
                       for name in packages)


def test_rosdep_resolution_reuses_exact_disconnected_prepare_environment(
        capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch)
    argv = plan["phase_argv"]["rosdep_resolution"]
    assert argv == capture._rosdep_resolution_phase_argv(
        plan["container_name"], plan["distro"]
    )
    assert argv[:2] == ["docker", "exec"]
    environment = {}
    index = 2
    while argv[index] == "--env":
        key, value = argv[index + 1].split("=", 1)
        environment[key] = value
        index += 2
    assert environment == capture.ROSDEP_PREPARE.EXPECTED_NETWORK_ENV
    assert argv[index] == plan["container_name"]
    assert argv[index + 1:index + 4] == ["rosdep", "install", "-r"]
    assert argv[argv.index("--sources-cache-dir") + 1] == (
        capture.ROSDEP_PREPARE.EXPECTED_GENERATED_ROSDEP_CACHE
    )
    assert argv[-2:] == ["--from-paths", "/workspace/src/graph_based_slam"]


def test_capture_layout_precreates_apt_partial_directories(capture, tmp_path):
    root = tmp_path / "root"
    capture._fresh_root(root, "test root")
    contract = capture._create_capture_directory_layout(
        root, include_prefetch=False
    )
    paths = set(contract["paths"])
    assert {"debs-build/partial", "debs-runtime/partial"} <= paths
    assert (root / "debs-build/partial").is_dir()
    assert (root / "debs-runtime/partial").is_dir()
    # The parent link counts are captured only after the complete managed
    # tree exists, so apt may use these directories without creating a new
    # child and drifting the directory contract.
    before = {item["path"]: item for item in contract["before"]}
    assert before["debs-build"]["nlink"] == 3
    assert before["debs-runtime"]["nlink"] == 3


def _restore_fixture(capture, tmp_path, monkeypatch):
    """Create the host-owned restore references used by direct phase tests."""
    tmp_path.mkdir(parents=True, exist_ok=True)
    plan = _plan(capture, tmp_path, monkeypatch)
    root = Path(plan["output_root"])
    capture._fresh_root(root, "restore owner test root")
    capture._create_capture_directory_layout(root, include_prefetch=False)
    plan["_restore_owner_records"] = {}
    return plan, root


@pytest.mark.parametrize("distro,uid", [("humble", 100), ("jazzy", 42)])
def test_restore_owner_transient_identity_is_distro_image_bound(
        capture, tmp_path, monkeypatch, distro, uid):
    base = tmp_path / distro
    base.mkdir()
    plan = _plan(capture, base, monkeypatch, distro=distro)
    identity = capture._apt_transient_owner_binding(plan)
    assert identity["schema"] == "registration-plugin-apt-transient-owner-binding-v2"
    assert identity["distro"] == distro
    assert identity["uid"] == uid
    assert identity["gid"] == 0
    assert identity["profile_sha256"] == plan["profile_sha256"]
    assert identity["image_digest"] == plan["image"]["digest"]


def test_restore_owner_phases_use_ordered_ro_reference_binds_and_pass(
        capture, tmp_path, monkeypatch):
    plan, root = _restore_fixture(capture, tmp_path, monkeypatch)
    expected_roles = ["build", "runtime"]
    assert [item["role"] for item in plan["restore_owner_reference_mounts"]] == expected_roles
    assert all(item["read_only"] and item["type"] == "bind"
               for item in plan["restore_owner_reference_mounts"])
    assert plan["phase_argv"]["restore_build_partial_owner"] == [
        "docker", "exec", plan["container_name"], "chown",
        "--reference=/workspace/restore-owner-reference/build-partial",
        "/workspace/evidence-parent/debs-build/partial",
    ]
    for phase in capture.RESTORE_OWNER_PHASES:
        record, evidence = capture._run_restore_owner_phase(
            plan, root, phase, lambda argv, timeout: (0, b"", b"", False, None))
        assert record["phase"] == phase
        assert evidence["status"] == "PASS"
        assert evidence["reference_mount"]["read_only"] is True
        assert evidence["children_before"] == []
        assert evidence["children_after"] == []
        assert evidence["target_after"]["uid"] == os.getuid()
        assert evidence["target_after"]["gid"] == os.getgid()


def test_restore_owner_runtime_mount_accepts_only_complete_fixed_spec(
        capture, tmp_path, monkeypatch):
    plan, root = _restore_fixture(capture, tmp_path, monkeypatch)
    values = []
    for index, fixed in enumerate(plan["restore_owner_reference_mounts"]):
        descriptor = capture._restore_reference_descriptor(
            Path(fixed["source"]), "restore runtime test {}".format(index))
        values.append(dict(
            fixed, pre=descriptor, post=copy.deepcopy(descriptor),
            runtime=copy.deepcopy(fixed)))

    capture._validate_restore_owner_mount_bindings(
        values, root, required=True, reached=True, runtime_required=True)

    drifted = copy.deepcopy(values)
    del drifted[0]["runtime"]["role"]
    with pytest.raises(capture.CaptureError) as error:
        capture._validate_restore_owner_mount_bindings(
            drifted, root, required=True, reached=True, runtime_required=True)
    assert error.value.kind == "RESTORE_OWNER_RUNTIME_INVALID"


def test_restore_owner_command_failure_is_retained_as_failed_evidence(
        capture, tmp_path, monkeypatch):
    plan, root = _restore_fixture(capture, tmp_path, monkeypatch)
    record, evidence = capture._run_restore_owner_phase(
        plan, root, "restore_build_partial_owner",
        lambda argv, timeout: (2, b"", b"chown failed\n", False, None))
    assert record["returncode"] == 2
    assert record["observed_returncode"] == 2
    assert evidence["status"] == "FAILED"
    assert evidence["target_before"] == evidence["target_after"]
    assert evidence["reference_before"] == evidence["reference_after"]


def test_restore_owner_rejects_missing_reference_or_extra_child(
        capture, tmp_path, monkeypatch):
    plan, root = _restore_fixture(capture, tmp_path, monkeypatch)
    reference = Path(plan["restore_owner_reference_mounts"][0]["source"])
    reference.rmdir()
    with pytest.raises(capture.CaptureError):
        capture._run_restore_owner_phase(
            plan, root, "restore_build_partial_owner",
            lambda argv, timeout: (0, b"", b"", False, None))

    plan, root = _restore_fixture(capture, tmp_path / "extra", monkeypatch)
    child = root / "debs-build" / "partial" / "unexpected"
    child.write_bytes(b"unexpected")
    with pytest.raises(capture.CaptureError):
        capture._run_restore_owner_phase(
            plan, root, "restore_build_partial_owner",
            lambda argv, timeout: (0, b"", b"", False, None))


def test_restore_owner_rejects_command_mode_owner_and_projection_drift(
        capture, tmp_path, monkeypatch):
    plan, root = _restore_fixture(capture, tmp_path / "mode", monkeypatch)
    tampered = copy.deepcopy(plan)
    tampered["phase_argv"]["restore_build_partial_owner"][-1] = "/workspace/evidence-parent/escape"
    with pytest.raises(capture.CaptureError):
        capture._run_restore_owner_phase(
            tampered, root, "restore_build_partial_owner",
            lambda argv, timeout: (0, b"", b"", False, None))

    plan, root = _restore_fixture(capture, tmp_path / "drift", monkeypatch)
    target = root / "debs-build" / "partial"
    target.chmod(0o755)
    with pytest.raises(capture.CaptureError):
        capture._run_restore_owner_phase(
            plan, root, "restore_build_partial_owner",
            lambda argv, timeout: (0, b"", b"", False, None))

    plan, root = _restore_fixture(capture, tmp_path, monkeypatch)
    record, evidence = capture._run_restore_owner_phase(
        plan, root, "restore_build_partial_owner",
        lambda argv, timeout: (0, b"", b"", False, None))
    drifted = copy.deepcopy(evidence)
    drifted["target_after"]["uid"] += 1
    with pytest.raises(capture.CaptureError):
        capture._validate_restore_owner_phase_record(
            drifted, root, plan, record)


def test_restore_owner_exact_transient_owner_uses_lstat_without_pre_scan(
        capture, tmp_path, monkeypatch):
    """An _apt-owned 0700 target is observed by lstat and restored in-container."""
    plan, root = _restore_fixture(capture, tmp_path / "transient", monkeypatch)
    target = root / "debs-build" / "partial"
    presealed = capture._restore_directory_snapshot(target, "test transient presealed")
    plan["_restore_owner_target_before"] = {"build": copy.deepcopy(presealed)}
    original_lstat = capture._restore_directory_lstat_snapshot
    original_scandir = capture.os.scandir
    restoring = {"started": False, "pre_scan": False}

    def fake_lstat(path, label):
        observed = original_lstat(path, label)
        if "restore_build_partial_owner target before" in label:
            observed["uid"] = plan["apt_transient_owner"]["uid"]
            observed["gid"] = plan["apt_transient_owner"]["gid"]
        return observed

    def fake_scandir(path):
        if str(path) == str(target) and not restoring["started"]:
            restoring["pre_scan"] = True
            raise PermissionError("_apt target is unreadable")
        return original_scandir(path)

    monkeypatch.setattr(capture, "_restore_directory_lstat_snapshot", fake_lstat)
    monkeypatch.setattr(capture.os, "scandir", fake_scandir)

    def runner(argv, timeout):
        del argv, timeout
        restoring["started"] = True
        return 0, b"", b"", False, None

    record, evidence = capture._run_restore_owner_phase(
        plan, root, "restore_build_partial_owner", runner)
    assert record["observed_returncode"] == 0
    assert evidence["status"] == "PASS"
    assert evidence["target_before_observation"] == "lstat_only_presealed_empty"
    assert evidence["apt_identity"]["uid"] == plan["apt_transient_owner"]["uid"]
    assert restoring["pre_scan"] is False


def test_restore_owner_rejects_non_profile_transient_owner(
        capture, tmp_path, monkeypatch):
    plan, root = _restore_fixture(capture, tmp_path / "wrong-owner", monkeypatch)
    original_lstat = capture._restore_directory_lstat_snapshot

    def wrong_owner(path, label):
        observed = original_lstat(path, label)
        if "restore_build_partial_owner target before" in label:
            observed["uid"] = plan["apt_transient_owner"]["uid"] + 1
            observed["gid"] = 0
        return observed

    monkeypatch.setattr(capture, "_restore_directory_lstat_snapshot", wrong_owner)
    called = []

    def runner(argv, timeout):
        called.append((argv, timeout))
        return 0, b"", b"", False, None

    with pytest.raises(capture.CaptureError) as error:
        capture._run_restore_owner_phase(
            plan, root, "restore_build_partial_owner", runner)
    assert error.value.kind == "RESTORE_OWNER_TRANSIENT_OWNER_INVALID"
    assert called == []


def test_restore_owner_rejects_extra_child_created_after_restore(
        capture, tmp_path, monkeypatch):
    plan, root = _restore_fixture(capture, tmp_path / "post-extra", monkeypatch)
    target = root / "debs-build" / "partial"
    child = target / "created-after-restore"

    def runner(argv, timeout):
        del argv, timeout
        child.write_bytes(b"unexpected")
        return 0, b"", b"", False, None

    record, evidence = capture._run_restore_owner_phase(
        plan, root, "restore_build_partial_owner", runner)
    assert record["observed_returncode"] == 0
    assert record["returncode"] == 1
    assert evidence["status"] == "FAILED"
    assert evidence["children_after"][0]["name"] == child.name
    child.unlink()


def test_prepare_plan_binds_ordered_mounts_bundle_discovery_and_argv(
        capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch)
    mounts = plan["rosdep_prepare_mounts"]
    assert len(mounts) == 8
    assert [item["target"] for item in mounts] == [
        capture.ROSDEP_PREPARE_CONTAINER_ROOT,
        capture.ROSDEP_PREPARE_CONTAINER_DISCOVERY_ROOT,
        capture.ROSDEP_PREPARE.EXPECTED_LOCAL_SOURCE_ROOT,
        "/opt/registration-plugin/rosdistro",
        capture.ROSDEP_PREPARE.EXPECTED_ROSDEP_CACHE,
        capture.ROSDEP_PREPARE.EXPECTED_ROS_HOME,
        capture.ROSDEP_PREPARE.EXPECTED_SOURCE_LIST_DIR,
        "/opt/registration-plugin/rosdep-output-cache",
    ]
    assert all(item["type"] == "bind" for item in mounts)
    assert [item["read_only"] for item in mounts] == [True, True, True,
                                                       False, False, False,
                                                       False, False]
    assert plan["rosdep_prepare_bundle"] == plan["rosdep_prepare_contract"]["input_bundle"]
    assert plan["rosdep_prepare_discovery"] == plan["rosdep_prepare_contract"]["discovery"]
    assert plan["rosdep_prepare_argv"] == capture._prepare_argv_metadata(plan)
    assert plan["rosdep_prepare_argv"]["script_path"] == (
        capture.ROSDEP_PREPARE_CONTAINER_TOOL)


def _materialized_prepare_mounts(capture, plan, root):
    capture._create_capture_directory_layout(root, include_prefetch=False)
    plan["_rosdep_prepare_mounts_before"] = (
        capture._prepare_mount_descriptor_snapshot(plan, root, "test pre"))
    plan["_rosdep_prepare_mounts_after"] = (
        capture._prepare_mount_descriptor_snapshot(plan, root, "test post"))
    return capture._prepare_mount_bindings(plan, root)


@pytest.mark.parametrize("mutator", [
    lambda value: value.pop(),
    lambda value: value.append(copy.deepcopy(value[-1])),
    lambda value: value.reverse(),
    lambda value: value[0].__setitem__("target", value[1]["target"]),
    lambda value: value[0].__setitem__("source", "/tmp/escape"),
])
def test_prepare_mount_readback_rejects_missing_extra_reordered_or_swapped(
        capture, tmp_path, monkeypatch, mutator):
    plan = _plan(capture, tmp_path, monkeypatch)
    root = Path(plan["output_root"])
    capture._fresh_root(root, "test root")
    mounts = _materialized_prepare_mounts(capture, plan, root)
    mutator(mounts)
    with pytest.raises(capture.CaptureError):
        capture._validate_prepare_mount_bindings(
            mounts, root, required=False, prepare_reached=False)


def test_prepare_mount_readback_rejects_metadata_and_runtime_drift(
        capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch)
    root = Path(plan["output_root"])
    capture._fresh_root(root, "test root")
    mounts = _materialized_prepare_mounts(capture, plan, root)
    drifted = copy.deepcopy(mounts)
    drifted[0]["post"] = dict(drifted[0]["post"],
                                inode=drifted[0]["post"]["inode"] + 1)
    with pytest.raises(capture.CaptureError):
        capture._validate_prepare_mount_bindings(
            drifted, root, required=False, prepare_reached=False)

    runtime_drifted = copy.deepcopy(mounts)
    runtime_drifted[0]["runtime"] = dict(runtime_drifted[0]["pre"], type="bind")
    with pytest.raises(capture.CaptureError):
        capture._validate_prepare_mount_bindings(
            runtime_drifted, root, required=False, prepare_reached=False)


def test_prepare_mount_readback_rejects_symlink_source(
        capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch)
    root = Path(plan["output_root"])
    capture._fresh_root(root, "test root")
    mounts = _materialized_prepare_mounts(capture, plan, root)
    source = root / capture.ROSDEP_PREPARE_INPUT_RELATIVE
    source.rmdir()
    source.symlink_to(root)
    with pytest.raises(capture.CaptureError):
        capture._validate_prepare_mount_bindings(
            mounts, root, required=False, prepare_reached=False)


def test_prepare_input_readback_rejects_hardlink_replacement(
        capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch)
    profile = Path(plan["profile_path"])
    plan["profile_sha256"] = capture.sha256_bytes(profile.read_bytes())
    root = Path(plan["output_root"])
    capture._fresh_root(root, "test root")
    capture._create_capture_directory_layout(root, include_prefetch=False)
    binding = capture._materialize_prepare_input(root, plan)
    destination = root / capture.ROSDEP_PREPARE_INPUT_RELATIVE / "prepare_registration_plugin_rosdep.py"
    peer = root / capture.ROSDEP_PREPARE_INPUT_RELATIVE / "registration_plugin_rosdep_prepare_v1.schema.json"
    destination.unlink()
    os.link(peer, destination)
    with pytest.raises(capture.CaptureError):
        capture._verify_prepare_input_binding(root, plan, binding)


def _prepare_binding_pair_fixture(capture, tmp_path, monkeypatch):
    """Materialize one host binding and its container-path projection."""
    plan = _plan(capture, tmp_path, monkeypatch)
    profile = Path(plan["profile_path"])
    plan["profile_sha256"] = capture.sha256_bytes(profile.read_bytes())
    root = Path(plan["output_root"])
    capture._fresh_root(root, "prepare pair test root")
    capture._create_capture_directory_layout(root, include_prefetch=False)
    host = capture._materialize_prepare_input(root, plan)
    transport = copy.deepcopy(host)
    transport["mount"]["source"] = capture.ROSDEP_PREPARE_CONTAINER_ROOT
    descriptor_fields = (
        "path", "bytes", "sha256", "mode", "uid", "gid", "nlink",
        "device", "inode",
    )
    for item in transport["files"]:
        copied = item["before"]
        descriptor = {
            key: copied[key]
            for key in descriptor_fields
        }
        descriptor["path"] = item["container_path"]
        item["source"] = dict(descriptor)
        item["before"] = dict(descriptor)
        item["after"] = dict(descriptor)
    transport["identity_sha256"] = capture.canonical_hash(
        transport, "identity_sha256")
    return plan, root, host, transport


def test_prepare_input_pair_accepts_host_and_transport_namespaces(
        capture, tmp_path, monkeypatch):
    plan, root, host, transport = _prepare_binding_pair_fixture(
        capture, tmp_path, monkeypatch)
    capture._validate_prepare_input_binding_pair(
        host, transport, capture_root=root)
    assert host["mount"]["source"] == str(
        root / capture.ROSDEP_PREPARE_INPUT_RELATIVE)
    assert transport["mount"]["source"] == capture.ROSDEP_PREPARE_CONTAINER_ROOT
    del plan


def test_prepare_projection_reopens_precreated_host_bindings(
        capture, tmp_path, monkeypatch):
    root = tmp_path / "capture-root"
    prepare_root = root / capture.ROSDEP_PREPARE_RELATIVE
    prepare_root.mkdir(parents=True)
    receipt_path = prepare_root / capture.ROSDEP_PREPARE_RECEIPT_NAME
    receipt_path.write_text("{}", encoding="utf-8")
    plan = {
        "rosdep_prepare_contract": {
            "required": True,
            "discovery": {"root": str(tmp_path / "discovery")},
        },
        "_rosdep_prepare_input": {},
    }
    observed = {}

    def validate(*args, **kwargs):
        observed.update(kwargs)
        raise capture.ROSDEP_PREPARE.PrepareError("STOP", "after options")

    monkeypatch.setattr(
        capture.ROSDEP_PREPARE, "validate_prepare_receipt", validate)
    with pytest.raises(capture.CaptureError) as error:
        capture._rosdep_prepare_projection(root, plan, allow_partial=True)
    assert error.value.kind == "ROSDEP_PREPARE_RECEIPT_INVALID"
    assert observed["allow_host_bindings"] is True
    assert observed["reopen_input_binding"] is False


@pytest.mark.parametrize("mutation", [
    "null_host", "null_transport", "transport_path", "transport_sha",
    "mount_target", "host_source",
])
def test_prepare_input_pair_rejects_null_path_sha_and_mount_cross_wire(
        capture, tmp_path, monkeypatch, mutation):
    _, root, host, transport = _prepare_binding_pair_fixture(
        capture, tmp_path, monkeypatch)
    if mutation == "null_host":
        host = None
    elif mutation == "null_transport":
        transport = None
    elif mutation == "transport_path":
        transport["files"][0]["container_path"] = (
            capture.ROSDEP_PREPARE_CONTAINER_ROOT + "/other.py")
        transport["identity_sha256"] = capture.canonical_hash(
            transport, "identity_sha256")
    elif mutation == "transport_sha":
        transport["files"][0]["source"]["sha256"] = "0" * 64
        transport["identity_sha256"] = capture.canonical_hash(
            transport, "identity_sha256")
    elif mutation == "mount_target":
        transport["mount"]["target"] = "/opt/registration-plugin/other"
        transport["identity_sha256"] = capture.canonical_hash(
            transport, "identity_sha256")
    else:
        host["mount"]["source"] = str(root / "wrong-input")
        host["identity_sha256"] = capture.canonical_hash(
            host, "identity_sha256")
    with pytest.raises(capture.CaptureError) as error:
        capture._validate_prepare_input_binding_pair(
            host, transport, capture_root=root)
    assert error.value.kind == "PREPARE_INPUT_BINDING_INVALID"


def _prepare_root_envelope(capture, root, *, status="REVIEW_REQUIRED"):
    logical_path = str(root / capture.ROSDEP_PREPARE_RELATIVE)
    transport_path = capture.ROSDEP_PREPARE_CONTAINER_OUTPUT_ROOT
    metadata = {
        "type": "directory", "mode": 0o700, "uid": 1000, "gid": 1000,
        "nlink": 2, "device": 123, "inode": 456,
    }
    logical = {"path": logical_path, **metadata}
    transport = {"path": transport_path, **metadata}
    return {
        "status": status, "root": logical, "transport_root": transport,
        "root_mount": {
            "source": logical_path, "target": transport_path,
            "read_only": False, "type": "bind", "noexec": False,
            "identity": {
                key: transport[key]
                for key in ("device", "inode", "uid", "gid", "mode", "nlink")
            },
        },
    }


@pytest.mark.parametrize("status", [
    "REVIEW_REQUIRED", "PARTIAL_FAILURE_REVIEW_REQUIRED",
])
def test_prepare_root_envelope_accepts_success_and_partial(
        capture, tmp_path, status):
    value = _prepare_root_envelope(capture, tmp_path, status=status)
    capture._validate_prepare_root_binding(value, tmp_path)


@pytest.mark.parametrize("mutation", [
    "null", "logical_path", "transport_path", "metadata", "mount",
])
def test_prepare_root_envelope_rejects_cross_namespace_drift(
        capture, tmp_path, mutation):
    value = _prepare_root_envelope(capture, tmp_path)
    if mutation == "null":
        value["transport_root"] = None
    elif mutation == "logical_path":
        value["root"]["path"] = str(tmp_path / "other")
    elif mutation == "transport_path":
        value["transport_root"]["path"] = "/workspace/other"
    elif mutation == "metadata":
        value["transport_root"]["inode"] += 1
    else:
        value["root_mount"]["target"] = "/workspace/other"
    with pytest.raises(capture.CaptureError) as error:
        capture._validate_prepare_root_binding(value, tmp_path)
    assert error.value.kind == "PREPARE_ROOT_BINDING_INVALID"


def test_capture_preserves_exact_profile_bound_http_apt_urls(capture):
    allowed = (
        "http://archive.ubuntu.com/ubuntu",
        "http://archive.ubuntu.com:80/ubuntu/dists/jazzy/InRelease",
        "http://packages.ros.org/ros2/ubuntu/dists/jazzy/InRelease",
        "http://security.ubuntu.com/ubuntu/pool/main/x/example.deb",
    )
    for value in allowed:
        assert capture._capture_url(value, "APT URL") == value

    rejected = (
        "http://archive.ubuntu.com.evil/ubuntu",
        "http://archive.ubuntu.com/ubuntuish",
        "http://archive.ubuntu.com:8080/ubuntu",
        "http://user@archive.ubuntu.com/ubuntu",
        "http://archive.ubuntu.com/ubuntu?x=1",
        "http://archive.ubuntu.com/ubuntu#fragment",
        "http://unknown.invalid/ubuntu",
        "https://apt.example.invalid/ubuntu?x=1",
    )
    for value in rejected:
        with pytest.raises(capture.CaptureError) as error:
            capture._capture_url(value, "APT URL")
        assert error.value.kind == "APT_URL_INVALID"
    with pytest.raises(capture.CaptureError) as error:
        capture._https_url("http://archive.ubuntu.com/ubuntu/archive.tar.gz",
                           "archive URL")
    assert error.value.kind == "HTTPS_URL_INVALID"
    assert capture.VERIFICATION_CHAIN == {
        "release_signature": "gpgv_evidence_required_before_closure",
        "release_to_packages": "signed_release_sha256_size_exact",
        "packages_to_deb": "packages_sha256_size_and_dpkg_metadata_exact",
    }


def test_capture_success_seals_partial_safe_projection(capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch)
    output_root = Path(plan["output_root"])

    def compose(candidate_plan, root, records, optional_prefetch):
        assert candidate_plan["plan_identity_sha256"] == plan["plan_identity_sha256"]
        assert len(records) == len(capture.PHASES)
        assert optional_prefetch == {"status": "NOT_APPLICABLE"}
        value = {"schema": "synthetic-capture-v1", "status": "REVIEW_REQUIRED"}
        value["canonical_sha256"] = capture.canonical_hash(value)
        capture._seal_pair(root, capture.CAPTURE_INPUT_NAME, value)
        return {"status": "REVIEW_REQUIRED"}

    monkeypatch.setattr(capture, "_compose_candidate", compose)
    runner, calls = _runner(capture, plan, output_root)
    result = capture.capture_dependency_closure(
        Path(plan["repo_root"]), Path(plan["profile_path"]), plan["distro"],
        plan["dependency_leg"], output_root, runner=runner, clock=lambda: 10)
    assert result["status"] == "REVIEW_REQUIRED"
    assert result["outcome"] == "PASS_REVIEW_REQUIRED"
    assert result["benchmark_eligible"] is False
    assert len(calls) >= len(capture.PHASES)
    receipt = json.loads((output_root / capture.RECEIPT_NAME).read_text(encoding="utf-8"))
    assert not list(_receipt_validator().iter_errors(receipt))
    assert _load_closure().validate_capture_executor_receipt(output_root)["status"] == (
        "REVIEW_REQUIRED")
    assert set(receipt["artifacts"]) == {
        "capture_input", "base_status", "apt_source_capture", "dependency_capture",
    }
    assert (output_root / capture.RECEIPT_NAME).stat().st_mode & 0o777 == 0o444
    assert (output_root / (capture.RECEIPT_NAME + ".sha256")).stat().st_nlink == 1


def test_capture_partial_failure_reopens_truthfully(capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch)
    output_root = Path(plan["output_root"])
    runner, _ = _runner(capture, plan, output_root, fail_phase="apt_update")
    result = capture.capture_dependency_closure(
        Path(plan["repo_root"]), Path(plan["profile_path"]), plan["distro"],
        plan["dependency_leg"], output_root, runner=runner, clock=lambda: 20)
    assert result["status"] == "PARTIAL_FAILURE_REVIEW_REQUIRED"
    receipt = json.loads((output_root / capture.RECEIPT_NAME).read_text(encoding="utf-8"))
    assert not list(_receipt_validator().iter_errors(receipt))
    assert receipt["outcome"] == "PARTIAL_FAILURE"
    assert receipt["error"]["kind"] == "PHASE_FAILED"
    assert [item["phase"] for item in receipt["phases"]] == [
        "image_inspect", "preexisting_container_check", "container_start", "apt_update"
    ]
    assert capture.validate_receipt(output_root, expected_plan=plan)["benchmark_eligible"] is False


def test_phase_prefix_rejects_premature_future_rosdep_receipt(
        capture, tmp_path, monkeypatch):
    """A failed APT prefix cannot claim the later prepare receipt."""
    plan = _plan(capture, tmp_path, monkeypatch)
    output_root = Path(plan["output_root"])
    runner, _ = _runner(capture, plan, output_root, fail_phase="apt_update")
    capture.capture_dependency_closure(
        Path(plan["repo_root"]), Path(plan["profile_path"]), plan["distro"],
        plan["dependency_leg"], output_root, runner=runner, clock=lambda: 21)
    prepare_root = output_root / capture.ROSDEP_PREPARE_RELATIVE
    prepare_root.mkdir(exist_ok=True)
    (prepare_root / capture.ROSDEP_PREPARE_RECEIPT_NAME).write_bytes(b"future\n")
    with pytest.raises(capture.CaptureError) as error:
        capture.validate_receipt(output_root, expected_plan=plan)
    assert error.value.kind == "PHASE_FUTURE_EVIDENCE_INVALID"


def test_phase_prefix_rejects_future_artifact_claim(
        capture, tmp_path, monkeypatch):
    """A future top-level artifact is invalid even with a fresh sidecar."""
    plan = _plan(capture, tmp_path, monkeypatch)
    output_root = Path(plan["output_root"])
    runner, _ = _runner(capture, plan, output_root, fail_phase="apt_update")
    capture.capture_dependency_closure(
        Path(plan["repo_root"]), Path(plan["profile_path"]), plan["distro"],
        plan["dependency_leg"], output_root, runner=runner, clock=lambda: 22)
    capture._seal_pair(output_root, "requirements.json", {"future": True})
    receipt_path = output_root / capture.RECEIPT_NAME
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    receipt["artifacts"]["requirements"] = capture._descriptor(
        output_root / "requirements.json", "requirements", capture.MAX_JSON_BYTES,
        sidecar=True)
    receipt["canonical_sha256"] = capture.canonical_hash(receipt)
    _rewrite_sealed_json(capture, receipt_path, receipt)
    with pytest.raises(capture.CaptureError) as error:
        capture.validate_receipt(output_root, expected_plan=plan)
    assert error.value.kind == "PHASE_FUTURE_EVIDENCE_INVALID"


def test_phase_prefix_requires_reached_artifact(
        capture, tmp_path, monkeypatch):
    """Removing a reached phase artifact is not an ordinary partial row."""
    plan = _plan(capture, tmp_path, monkeypatch)
    output_root = Path(plan["output_root"])

    def compose(candidate_plan, root, records, optional_prefetch):
        del candidate_plan, records, optional_prefetch
        value = {"schema": "synthetic-capture-v1", "status": "REVIEW_REQUIRED"}
        value["canonical_sha256"] = capture.canonical_hash(value)
        capture._seal_pair(root, capture.CAPTURE_INPUT_NAME, value)
        return {"status": "REVIEW_REQUIRED"}

    monkeypatch.setattr(capture, "_compose_candidate", compose)
    runner, _ = _runner(capture, plan, output_root)
    capture.capture_dependency_closure(
        Path(plan["repo_root"]), Path(plan["profile_path"]), plan["distro"],
        plan["dependency_leg"], output_root, runner=runner, clock=lambda: 23)
    (output_root / "base-status.json").unlink()
    (output_root / "base-status.json.sha256").unlink()
    with pytest.raises(capture.CaptureError) as error:
        capture.validate_receipt(output_root, expected_plan=plan)
    assert error.value.kind == "PHASE_EVIDENCE_MISSING"


def test_phase_prefix_rejects_skip_or_reorder(
        capture, tmp_path, monkeypatch):
    """Phase-aware validation still requires the fixed ordered prefix."""
    plan = _plan(capture, tmp_path, monkeypatch)
    output_root = Path(plan["output_root"])
    monkeypatch.setattr(
        capture, "_compose_candidate",
        lambda p, r, records, optional: capture._seal_pair(
            r, capture.CAPTURE_INPUT_NAME, {"x": 1}))
    runner, _ = _runner(capture, plan, output_root)
    capture.capture_dependency_closure(
        Path(plan["repo_root"]), Path(plan["profile_path"]), plan["distro"],
        plan["dependency_leg"], output_root, runner=runner, clock=lambda: 24)
    receipt_path = output_root / capture.RECEIPT_NAME
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    receipt["phases"][0], receipt["phases"][1] = (
        receipt["phases"][1], receipt["phases"][0])
    receipt["canonical_sha256"] = capture.canonical_hash(receipt)
    _rewrite_sealed_json(capture, receipt_path, receipt)
    with pytest.raises(capture.CaptureError) as error:
        capture.validate_receipt(output_root, expected_plan=plan)
    assert error.value.kind == "PHASE_ORDER_INVALID"


def test_phase_prefix_rejects_tampered_reached_log(
        capture, tmp_path, monkeypatch):
    """A reached phase log is independently reopened before acceptance."""
    plan = _plan(capture, tmp_path, monkeypatch)
    output_root = Path(plan["output_root"])
    monkeypatch.setattr(
        capture, "_compose_candidate",
        lambda p, r, records, optional: capture._seal_pair(
            r, capture.CAPTURE_INPUT_NAME, {"x": 1}))
    runner, _ = _runner(capture, plan, output_root)
    capture.capture_dependency_closure(
        Path(plan["repo_root"]), Path(plan["profile_path"]), plan["distro"],
        plan["dependency_leg"], output_root, runner=runner, clock=lambda: 25)
    log = output_root / "logs" / "image_inspect.stdout"
    log.chmod(0o644)
    log.write_bytes(b"tampered\n")
    log.chmod(0o444)
    with pytest.raises(capture.CaptureError) as error:
        capture.validate_receipt(output_root, expected_plan=plan)
    assert error.value.kind == "PHASE_LOG_DRIFT"


def test_prefetch_receives_explicit_fetcher(capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch, dependency_leg="present")
    root = tmp_path / "capture-root"
    capture.EVIDENCE_DIRECTORIES.create_fresh_root(root)
    capture.EVIDENCE_DIRECTORIES.create_layout(root, ("logs",))
    prefetch_root = root / "prefetch-root"
    capture.EVIDENCE_DIRECTORIES.create_fresh_root(prefetch_root)
    capture.EVIDENCE_DIRECTORIES.create_layout(
        prefetch_root, ("prefetch", "prefetch/archives"))
    seen = []

    def fake_prefetch(*args, **kwargs):
        seen.append(kwargs["fetcher"])
        return {"receipt_sha256": "a" * 64, "manifest_sha256": "b" * 64}

    monkeypatch.setattr(capture.PREFETCH, "prefetch_pinned_archives", fake_prefetch)
    fetcher = object()
    record, optional = capture._run_archive_prefetch(plan, root, fetcher)
    assert record["phase"] == "archive_prefetch"
    assert seen == [fetcher]
    assert optional["status"] == "SEALED_PREFETCH"


def test_capture_projection_uses_relative_logs_and_inner_commands(capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch)
    root = Path(plan["output_root"])
    capture.EVIDENCE_DIRECTORIES.create_fresh_root(root)
    capture.EVIDENCE_DIRECTORIES.create_layout(root, ("logs",))
    records = []
    for phase in capture.PHASES:
        result = capture._phase_record(plan, root, phase, (0, b"ok\n", b"", False, None))
        records.append(result)
    value = capture._write_capture_input(
        root, plan, records, {"status": "NOT_APPLICABLE"})
    assert value["resolvers"]["build"]["command"][0] == "apt-get"
    assert value["resolvers"]["runtime"]["command"][0] == "apt-get"
    assert set(value["phase_records"][0]) == {
        "phase", "argv", "argv_sha256", "returncode", "network_used",
        "timeout_seconds", "attempts", "stdout_path", "stderr_path",
    }
    capture.CLOSURE._validate_capture(value)


def test_fresh_root_and_artifact_sidecar_are_fail_closed(capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch)
    output_root = Path(plan["output_root"])
    output_root.mkdir()
    runner, _ = _runner(capture, plan, output_root)
    with pytest.raises(capture.CaptureError) as error:
        capture.capture_dependency_closure(
            Path(plan["repo_root"]), Path(plan["profile_path"]), plan["distro"],
            plan["dependency_leg"], output_root, runner=runner)
    assert error.value.kind == "ROOT_NOT_FRESH"

    output_root.rmdir()
    monkeypatch.setattr(capture, "_compose_candidate", lambda p, r, records, optional: (
        capture._seal_pair(r, capture.CAPTURE_INPUT_NAME, {"x": 1})
    ))
    runner, _ = _runner(capture, plan, output_root)
    capture.capture_dependency_closure(
        Path(plan["repo_root"]), Path(plan["profile_path"]), plan["distro"],
        plan["dependency_leg"], output_root, runner=runner, clock=lambda: 30)
    sidecar = output_root / (capture.CAPTURE_INPUT_NAME + ".sha256")
    sidecar.chmod(0o644)
    sidecar.write_text("0" * 64 + "  capture-input.json\n", encoding="ascii")
    sidecar.chmod(0o444)
    with pytest.raises(capture.CaptureError) as error:
        capture.validate_receipt(output_root)
    assert error.value.kind in {"ARTIFACT_BINDING_INVALID", "SIDECAR_MISMATCH", "DESCRIPTOR_DRIFT"}


def test_absence_parser_accepts_only_documented_empty_forms(capture):
    name = "registration-plugin-capture-humble-absent-test"
    lowercase = ("error: no such object: {}\n".format(name)).encode()
    uppercase = ("Error: No such object: {}".format(name)).encode()
    assert capture._missing_container_output(b"[]\n", lowercase, name)
    assert capture._missing_container_output(b"", lowercase, name)
    assert capture._missing_container_output(b"[]\n", uppercase, name)
    assert capture._inspect_absence(1, b"[]\n", lowercase, name) is None
    assert not capture._missing_container_output(b"[{}]\n", lowercase, name)
    assert not capture._missing_container_output(
        b"[]\n", ("error: no such object: other\n").encode(), name)
    assert not capture._missing_container_output(b"[]\nextra\n", lowercase, name)
    assert not capture._missing_container_output(
        b"[]\n", ("{}\nextra".format(lowercase.decode())).encode(), name)
    with pytest.raises(capture.CaptureError):
        capture._inspect_absence(0, b"[]\n", lowercase, name)
    with pytest.raises(capture.CaptureError):
        capture._inspect_absence(1, b"[]\n", b"daemon failure\n", name)


def test_gpgv_227_real_informational_sequence_is_strictly_bound(capture):
    fingerprint = "F6ECB3762474EDA9D21B7022871920D1991BC93C"
    stdout = (
        "[GNUPG:] NEWSIG\n"
        "[GNUPG:] KEY_CONSIDERED {} 0\n"
        "[GNUPG:] SIG_ID HrKrdZjGCBwk+Qp/9/ZuVELIzaQ 2022-04-21 1650561399\n"
        "[GNUPG:] GOODSIG 871920D1991BC93C Ubuntu Archive Automatic Signing Key\n"
        "[GNUPG:] VALIDSIG {} 2022-04-21 1650561399 0 4 0 1 10 01 {}\n"
        "[GNUPG:] VERIFICATION_COMPLIANCE_MODE 23\n"
    ).format(fingerprint, fingerprint, fingerprint).encode("ascii")
    status = capture._parse_gpgv_status(stdout, b"", "real gpgv 2.2.27")
    assert status["status_sequence"] == [
        "NEWSIG", "KEY_CONSIDERED", "SIG_ID", "GOODSIG", "VALIDSIG",
        "VERIFICATION_COMPLIANCE_MODE",
    ]
    assert status["key_considered"] == {"fingerprint": fingerprint, "flags": 0}
    assert status["verification_compliance_mode"] == 23
    assert status["validsig"]["primary_fingerprint"] == fingerprint


@pytest.mark.parametrize("informational_line,expected_key,expected_mode", [
    ("[GNUPG:] KEY_CONSIDERED {fingerprint} 0\n", True, None),
    ("[GNUPG:] VERIFICATION_COMPLIANCE_MODE 23\n", False, 23),
])
def test_gpgv_informational_tags_are_independently_optional(
        capture, informational_line, expected_key, expected_mode):
    fingerprint = "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
    before_proof = informational_line if "KEY_CONSIDERED" in informational_line else ""
    after_proof = informational_line if "COMPLIANCE_MODE" in informational_line else ""
    stdout = (
        "[GNUPG:] NEWSIG\n"
        + before_proof
        + "[GNUPG:] SIG_ID id 2026-08-28 0\n"
        + "[GNUPG:] GOODSIG {} Open Robotics signer\n".format(fingerprint[-16:])
        + "[GNUPG:] VALIDSIG {} 2026-08-28 0 0 4 0 1 10 01 {}\n".format(
            fingerprint, fingerprint)
        + after_proof
    ).format(fingerprint=fingerprint).encode("ascii")
    status = capture._parse_gpgv_status(stdout, b"", "independent info tag")
    assert (status["key_considered"] is not None) is expected_key
    assert status["verification_compliance_mode"] == expected_mode
    assert status["goodsig"]["key_id"] == fingerprint[-16:]
    assert status["validsig"]["fingerprint"] == fingerprint


@pytest.mark.parametrize("mutation,expected_kind", [
    ("[GNUPG:] KEY_CONSIDERED {} 0 extra\n", "GPGV_KEY_CONSIDERED_INVALID"),
    ("[GNUPG:] KEY_CONSIDERED {} -1\n", "GPGV_KEY_CONSIDERED_INVALID"),
    ("[GNUPG:] KEY_CONSIDERED {} 01\n", "GPGV_KEY_CONSIDERED_INVALID"),
    ("[GNUPG:] VERIFICATION_COMPLIANCE_MODE 01\n", "GPGV_COMPLIANCE_MODE_INVALID"),
    ("duplicate-key", "GPGV_STATUS_DUPLICATE"),
    ("duplicate-compliance", "GPGV_STATUS_DUPLICATE"),
])
def test_gpgv_informational_tags_reject_shape_order_and_duplicates(
        capture, mutation, expected_kind):
    fingerprint = "A" * 40
    base = (
        "[GNUPG:] NEWSIG\n"
        "[GNUPG:] KEY_CONSIDERED {} 0\n"
        "[GNUPG:] SIG_ID id 2026-08-26 0\n"
        "[GNUPG:] GOODSIG {} Synthetic signer\n"
        "[GNUPG:] VALIDSIG {} 2026-08-26 0 4 0 1 10 01 00 {}\n"
        "[GNUPG:] VERIFICATION_COMPLIANCE_MODE 23\n"
    ).format(fingerprint, fingerprint[-16:], fingerprint, fingerprint)
    key_line = "[GNUPG:] KEY_CONSIDERED {} 0\n".format(fingerprint)
    compliance_line = "[GNUPG:] VERIFICATION_COMPLIANCE_MODE 23\n"
    if mutation == "duplicate-key":
        rendered = base.replace(key_line, key_line + key_line)
    elif mutation == "duplicate-compliance":
        rendered = base.replace(compliance_line, compliance_line + compliance_line)
    elif "KEY_CONSIDERED" in mutation:
        rendered = base.replace(
            key_line, mutation.format(fingerprint))
    elif "VERIFICATION_COMPLIANCE_MODE" in mutation:
        rendered = base.replace(
            compliance_line, mutation)
    else:
        rendered = base
    with pytest.raises(capture.CaptureError) as error:
        capture._parse_gpgv_status(rendered.encode("ascii"), b"", "bad info tag")
    assert error.value.kind == expected_kind


def test_gpgv_informational_fingerprint_must_match_primary(capture):
    fingerprint = "A" * 40
    other = "B" * 40
    stdout = (
        "[GNUPG:] NEWSIG\n"
        "[GNUPG:] KEY_CONSIDERED {} 0\n"
        "[GNUPG:] GOODSIG {} Synthetic signer\n"
        "[GNUPG:] VALIDSIG {} 2026-08-26 0 4 0 1 10 01 00 {}\n"
        "[GNUPG:] VERIFICATION_COMPLIANCE_MODE 23\n"
    ).format(other, fingerprint[-16:], fingerprint, fingerprint).encode("ascii")
    with pytest.raises(capture.CaptureError) as error:
        capture._parse_gpgv_status(stdout, b"", "mismatched considered key")
    assert error.value.kind == "GPGV_KEY_CONSIDERED_MISMATCH"


def test_receipt_schema_rejects_promotion_and_unknown_fields(capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch)
    root = Path(plan["output_root"])
    monkeypatch.setattr(capture, "_compose_candidate", lambda p, r, records, optional:
                        capture._seal_pair(r, capture.CAPTURE_INPUT_NAME, {"x": 1}))
    runner, _ = _runner(capture, plan, root)
    capture.capture_dependency_closure(
        Path(plan["repo_root"]), Path(plan["profile_path"]), plan["distro"],
        plan["dependency_leg"], root, runner=runner, clock=lambda: 31)
    value = json.loads((root / capture.RECEIPT_NAME).read_text(encoding="utf-8"))
    validator = _receipt_validator()
    value["unexpected"] = True
    assert list(validator.iter_errors(value))
    value.pop("unexpected")
    value["benchmark_eligible"] = True
    assert list(validator.iter_errors(value))

    original = json.loads((root / capture.RECEIPT_NAME).read_text(encoding="utf-8"))
    receipt_path = root / capture.RECEIPT_NAME
    sidecar_path = root / (capture.RECEIPT_NAME + ".sha256")
    assert original["plan"]["gpgv_status_policy"] == capture.GPGV_STATUS_POLICY

    policy_drift = json.loads(json.dumps(original))
    policy_drift["plan"]["gpgv_status_policy"]["proof"] = "status_tags_only"
    policy_drift["canonical_sha256"] = capture.canonical_hash(policy_drift)

    def _write_sealed(candidate):
        payload = capture.canonical_bytes(candidate) + b"\n"
        receipt_path.chmod(0o644)
        receipt_path.write_bytes(payload)
        receipt_path.chmod(0o444)
        sidecar_path.chmod(0o644)
        sidecar_path.write_text(
            "{}  {}\n".format(capture.sha256_bytes(payload), receipt_path.name),
            encoding="ascii")
        sidecar_path.chmod(0o444)

    _write_sealed(policy_drift)
    try:
        with pytest.raises(capture.CaptureError) as error:
            capture.validate_receipt(root, expected_plan=plan)
        assert error.value.kind == "PLAN_BINDING_INVALID"
    finally:
        _write_sealed(original)

    def _assert_artifact_rejected(mutator):
        candidate = json.loads(json.dumps(original))
        mutator(candidate["artifacts"])
        candidate["canonical_sha256"] = capture.canonical_hash(candidate)
        _write_sealed(candidate)
        try:
            with pytest.raises(capture.CaptureError) as error:
                capture.validate_receipt(root, expected_plan=plan)
            assert error.value.kind == "ARTIFACT_BINDING_INVALID"
        finally:
            _write_sealed(original)

    _assert_artifact_rejected(lambda artifacts: artifacts.pop("base_status"))
    _assert_artifact_rejected(
        lambda artifacts: artifacts.update({"unexpected": artifacts["capture_input"]}))
    _assert_artifact_rejected(
        lambda artifacts: artifacts.__setitem__(
            "base_status", artifacts["apt_source_capture"]))


def test_receipt_schema_binds_fixed_phase_prefix(capture, tmp_path, monkeypatch):
    """The JSON schema rejects phase skips even before custom readback."""
    plan = _plan(capture, tmp_path, monkeypatch)
    root = Path(plan["output_root"])
    monkeypatch.setattr(
        capture, "_compose_candidate",
        lambda p, r, records, optional: capture._seal_pair(
            r, capture.CAPTURE_INPUT_NAME, {"x": 1}))
    runner, _ = _runner(capture, plan, root)
    capture.capture_dependency_closure(
        Path(plan["repo_root"]), Path(plan["profile_path"]), plan["distro"],
        plan["dependency_leg"], root, runner=runner, clock=lambda: 33)
    receipt = json.loads((root / capture.RECEIPT_NAME).read_text(encoding="utf-8"))
    validator = _receipt_validator()
    assert not list(validator.iter_errors(receipt))
    reordered = json.loads(json.dumps(receipt))
    reordered["phases"][0], reordered["phases"][1] = (
        reordered["phases"][1], reordered["phases"][0])
    assert list(validator.iter_errors(reordered))
    skipped = json.loads(json.dumps(receipt))
    skipped["phases"][5]["phase"] = "rosdep_prepare"
    assert list(validator.iter_errors(skipped))


def test_runtime_boundary_keeps_capture_receipt_non_promoting(capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch)
    root = Path(plan["output_root"])
    monkeypatch.setattr(capture, "_compose_candidate", lambda p, r, records, optional:
                        capture._seal_pair(r, capture.CAPTURE_INPUT_NAME, {"x": 1}))
    runner, _ = _runner(capture, plan, root)
    capture.capture_dependency_closure(
        Path(plan["repo_root"]), Path(plan["profile_path"]), plan["distro"],
        plan["dependency_leg"], root, runner=runner, clock=lambda: 32)
    closure = _load_closure()
    monkeypatch.setattr(closure, "validate_closure_root", lambda *args, **kwargs: {})
    with pytest.raises(closure.ClosureError) as error:
        closure.validate_runtime_closure(
            root, distro=plan["distro"], image_digest=plan["image"]["digest"],
            dependency_leg=plan["dependency_leg"], capture_receipt_root=root)
    assert error.value.kind == "CLOSURE_REVIEW_REQUIRED"


def _mkdir(path):
    path.mkdir(parents=True, exist_ok=True)
    return path


def _write_fixture(path, data):
    _mkdir(path.parent)
    path.write_bytes(data)
    path.chmod(0o644)
    return path


def _rewrite_sealed_json(capture, path, value):
    """Rewrite a test-only sealed JSON pair after an explicit mutation."""
    payload = capture.canonical_bytes(value) + b"\n"
    sidecar = path.with_name(path.name + ".sha256")
    path.chmod(0o644)
    path.write_bytes(payload)
    path.chmod(0o444)
    sidecar.chmod(0o644)
    sidecar.write_text(
        "{}  {}\n".format(capture.sha256_bytes(payload), path.name),
        encoding="ascii")
    sidecar.chmod(0o444)


def _signed_container_fixture(capture, tmp_path, *, include_prefetch=False):
    container = tmp_path / "container"
    output = tmp_path / "output"
    _mkdir(container)
    capture.EVIDENCE_DIRECTORIES.create_fresh_root(output)
    capture._create_capture_directory_layout(
        output, include_prefetch=include_prefetch)
    source = b"Types: deb\nURIs: https://mirror.example.invalid/ubuntu\nSuites: jazzy\nComponents: main\nSigned-By: /usr/share/keyrings/ros.gpg\nArchitectures: amd64\n"
    _write_fixture(container / "etc/apt/sources.list.d/ros2.sources", source)
    _write_fixture(container / "usr/share/keyrings/ros.gpg", b"synthetic-keyring\n")
    release_name = "mirror.example.invalid_ubuntu_dists_jazzy_Release"
    packages_name = "mirror.example.invalid_ubuntu_dists_jazzy_main_binary-amd64_Packages"
    deb = b"synthetic-deb"
    deb_sha = capture.sha256_bytes(deb)
    packages = ("Package: demo\nVersion: 1.0\nArchitecture: amd64\n"
                "Filename: pool/main/d/demo/demo_1.0_amd64.deb\n"
                "Size: {}\nSHA256: {}\n\n").format(len(deb), deb_sha).encode()
    package_sha = capture.sha256_bytes(packages)
    release = ("Origin: synthetic\nSuite: jazzy\nSHA256:\n"
               " {} {} main/binary-amd64/Packages\n").format(
                   package_sha, len(packages)).encode()
    _write_fixture(container / "var/lib/apt/lists" / release_name, release)
    _write_fixture(container / "var/lib/apt/lists" / (release_name + ".gpg"), b"detached-signature\n")
    _write_fixture(container / "var/lib/apt/lists" / packages_name, packages)
    _write_fixture(container / "etc/ros/rosdep/sources.list.d/20-default.list",
                   b"yaml https://ros.example.invalid/rosdep.yaml jazzy\n")
    _write_fixture(container / "usr/share/doc/demo/copyright", b"Demo license\n")
    (container / "usr/share/doc/demo").chmod(0o755)
    return container, output, release_name, packages_name, deb, deb_sha


def _release_spec_fixture(capture, url, suite):
    """Build an in-memory repository spec and its exact Release evidence."""
    repository = {
        "url": url, "suite": suite, "component": "main",
        "signed_by": {"kind": "none"},
    }
    release = Path("/var/lib/apt/lists/{}_InRelease".format(suite))
    release_data = "Suite: {}\nSHA256:\n".format(suite).encode("ascii")
    release_kind = "inrelease"
    release_url, _ = capture._repository_urls(repository, release_kind, "plain")
    identity = capture._release_artifact_identity(
        repository, release, release_kind, release_data)
    output = {
        "path": "apt-source/indices/InRelease-{}".format(identity[:24]),
        "bytes": len(release_data), "sha256": capture.sha256_bytes(release_data),
        "mode": 0o444, "uid": 1000, "gid": 1000, "nlink": 1,
        "device": 1, "inode": 1,
    }
    record = {
        "path": "{}_InRelease".format(suite), "role": "apt_release",
        "url": release_url, "bytes": len(release_data),
        "sha256": capture.sha256_bytes(release_data), "source": str(release),
        "output": output,
    }
    evidence = {
        "path": record["path"], "source": str(release), "url": release_url,
        "repository_url": url, "kind": release_kind, "identity": identity,
        "bytes": len(release_data), "sha256": capture.sha256_bytes(release_data),
        "output": output, "signature": None,
    }
    return {
        "spec": {"repository": repository, "release": release,
                 "release_kind": release_kind, "release_identity": identity,
                 "release_record": record, "release_data": release_data,
                 "signed_by": {"kind": "none"}},
        "evidence": evidence,
    }


@pytest.mark.parametrize("url,suite", [
    ("https://archive.ubuntu.com/ubuntu", "jammy"),
    ("https://packages.ros.org/ros2/ubuntu", "jazzy"),
])
def test_repository_release_spec_binds_repository_and_suite_exactly(
        capture, url, suite):
    fixture = _release_spec_fixture(capture, url, suite)
    identity, record, data = capture._validate_repository_release_spec(
        fixture["spec"], [fixture["evidence"]])
    assert identity == fixture["spec"]["release_identity"]
    assert record == fixture["spec"]["release_record"]
    assert data == fixture["spec"]["release_data"]


def test_repository_release_spec_rejects_cross_repository_and_suite_evidence(capture):
    ubuntu = _release_spec_fixture(
        capture, "https://archive.ubuntu.com/ubuntu", "jammy")
    ros = _release_spec_fixture(
        capture, "https://packages.ros.org/ros2/ubuntu", "jazzy")
    with pytest.raises(capture.CaptureError) as error:
        capture._validate_repository_release_spec(
            ubuntu["spec"], [ros["evidence"]])
    assert error.value.kind == "GPG_RELEASE_IDENTITY_CONFLICT"

    updates = _release_spec_fixture(
        capture, "https://archive.ubuntu.com/ubuntu", "jammy-updates")
    with pytest.raises(capture.CaptureError) as error:
        capture._validate_repository_release_spec(
            ubuntu["spec"], [updates["evidence"]])
    assert error.value.kind == "GPG_RELEASE_IDENTITY_CONFLICT"


def _gpgv_runner(capture, *, fingerprint="A" * 40, bad=False, dpkg=False):
    calls = []

    def run(argv, timeout):
        del timeout
        calls.append(list(argv))
        if argv == ["gpgv", "--version"]:
            return 0, b"gpgv (GnuPG) 2.4.4\n", b"", False, None
        if argv and argv[0] == "gpgv":
            if bad:
                return 1, b"[GNUPG:] BADSIG 12345678 bad\n", b"", False, None
            keyid = fingerprint[-16:]
            status = ("[GNUPG:] NEWSIG\n"
                      "[GNUPG:] GOODSIG {} Synthetic signer\n"
                      "[GNUPG:] VALIDSIG {} 2026-08-26 0 0 1 10 00 8 00\n").format(
                          keyid, fingerprint)
            return 0, status.encode(), b"", False, None
        if argv and argv[0] == "dpkg-deb" and dpkg:
            return (0, b"Package: demo\nVersion: 1.0\nArchitecture: amd64\n"
                    b"Essential: no\nMulti-Arch: same\n", b"", False, None)
        raise AssertionError("unexpected fixture command: {}".format(argv))

    return run, calls


def test_in_container_apt_source_captures_signed_index_graph(capture, tmp_path):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)
    runner, calls = _gpgv_runner(capture)
    result = capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)
    assert result["status"] == "CAPTURED_REVIEW_REQUIRED"
    assert result["schema"] == capture.IN_CONTAINER_SCHEMA
    assert (output / "source-manifest.json").is_file()
    assert (output / "signature-evidence.json").is_file()
    assert any(argv and argv[0] == "gpgv" for argv in calls)
    source_manifest = json.loads(
        (output / "source-manifest.json").read_text(encoding="utf-8"))
    assert not list(_source_validator().iter_errors(source_manifest))
    assert {item["role"] for item in source_manifest["files"]} == {
        "apt_source", "apt_release", "apt_packages"}
    assert source_manifest["gpgv_status_policy"] == capture.GPGV_STATUS_POLICY
    assert source_manifest["canonical_sha256"] == capture.canonical_hash(source_manifest)
    assert (output / "source-manifest.json").stat().st_mode & 0o777 == 0o444
    evidence = result["release_evidence"]
    assert len(evidence) == 1
    assert evidence[0]["kind"] == "detached"
    assert evidence[0]["signature"] is not None
    assert evidence[0]["output"]["path"].startswith("apt-source/indices/")
    assert evidence[0]["signature"]["output"]["path"].startswith("signatures/")
    assert capture._validate_release_evidence(output, evidence, require_nonempty=True) == evidence


def test_v2_source_snapshot_seals_comment_only_inactive_source(capture, tmp_path):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)
    inactive = _write_fixture(
        container / "etc/apt/sources.list",
        b"# Ubuntu sources moved to /etc/apt/sources.list.d/ubuntu.sources\n")
    runner, _ = _gpgv_runner(capture)
    result = capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)

    manifest = json.loads(
        (output / "source-manifest.json").read_text(encoding="utf-8"))
    record = next(item for item in manifest["files"]
                  if item.get("path") == "etc/apt/sources.list")
    assert record["urls"] == []
    assert record["sha256"] == capture.sha256_bytes(inactive.read_bytes())
    assert all(item["path"] != "etc/apt/sources.list"
               for item in manifest["source_entries"])
    assert capture._read_source_manifest(output)[0]["files"] == manifest["files"]
    assert result["status"] == "CAPTURED_REVIEW_REQUIRED"


def test_deb822_source_comments_cannot_be_parsed_as_duplicate_fields(capture):
    data = (
        b"# Types: deb-src is intentionally disabled\n"
        b"Types: deb\n"
        b"URIs: https://apt.example.invalid/ubuntu\n"
        b"Suites: jazzy\n"
        b"# Components: duplicate-looking comment\n"
        b"Components: main\n")
    records, keyrings = capture._source_urls(
        data, "commented deb822", policy=capture.APT_SOURCE_URL_POLICY)
    assert len(records) == 1
    assert records[0]["line_start"] == 2
    assert records[0]["line_end"] == 6
    assert records[0]["components"] == ["main"]
    assert keyrings == []


def test_in_container_contract_reports_complete_optional_prefetch_layout(
        capture, tmp_path):
    container, output, _, _, _, _ = _signed_container_fixture(
        capture, tmp_path, include_prefetch=True)
    runner, _ = _gpgv_runner(capture)
    result = capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)
    paths = result["directory_contract"]["paths"]
    assert all(relative in paths
               for relative in capture.CAPTURE_PREFETCH_DIRECTORY_RELATIVES)


def test_in_container_contract_rejects_partial_optional_prefetch_layout(
        capture, tmp_path):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)
    (output / capture.CAPTURE_PREFETCH_DIRECTORY_RELATIVES[0]).mkdir(mode=0o700)
    runner, _ = _gpgv_runner(capture)
    with pytest.raises(capture.CaptureError) as error:
        capture._capture_in_container(
            "apt-source", output, "jazzy", container_root=container,
            command_runner=runner)
    assert error.value.kind == "DIRECTORY_OPEN_FAILED"


def test_v2_source_snapshot_rejects_legacy_rosdep_record(capture, tmp_path):
    """A v2 APT snapshot cannot smuggle rosdep evidence into its files."""
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)
    runner, _ = _gpgv_runner(capture)
    capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)
    manifest_path = output / "source-manifest.json"
    original = json.loads(manifest_path.read_text(encoding="utf-8"))
    altered = json.loads(json.dumps(original))
    release = next(item for item in altered["files"]
                   if item["role"] == "apt_release")
    release["role"] = "rosdep_source"
    _rewrite_sealed_json(capture, manifest_path, altered)
    try:
        with pytest.raises(capture.CaptureError) as error:
            capture._read_source_manifest(output)
        assert error.value.kind in {
            "SOURCE_MANIFEST_INVALID", "SOURCE_MANIFEST_ROLE_INVALID",
        }
    finally:
        _rewrite_sealed_json(capture, manifest_path, original)


def test_rosdep_prepare_output_nonempty_check_is_post_prepare_only(capture):
    """Only a completed prepare receipt may claim rosdep source/cache data."""
    capture._validate_rosdep_prepare_outputs({
        "source_list": {"bytes": 1}, "source_bindings": [{}],
        "cache_tree": {"bytes": 1}, "cache_entries": [{}],
    })
    for field, value, kind in (
            ("source_list", {"bytes": 0}, "ROSDEP_SOURCE_EMPTY"),
            ("source_bindings", [], "ROSDEP_SOURCE_EMPTY"),
            ("cache_tree", {"bytes": 0}, "ROSDEP_CACHE_EMPTY"),
            ("cache_entries", [], "ROSDEP_CACHE_EMPTY")):
        evidence = {
            "source_list": {"bytes": 1}, "source_bindings": [{}],
            "cache_tree": {"bytes": 1}, "cache_entries": [{}],
        }
        evidence[field] = value
        with pytest.raises(capture.CaptureError) as error:
            capture._validate_rosdep_prepare_outputs(evidence)
        assert error.value.kind == kind


@pytest.mark.parametrize("mutation", ["missing", "drift", "extra"])
def test_v2_source_manifest_requires_exact_gpgv_policy(capture, tmp_path, mutation):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)
    runner, _ = _gpgv_runner(capture)
    capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)
    manifest_path = output / "source-manifest.json"
    original = json.loads(manifest_path.read_text(encoding="utf-8"))
    candidate = json.loads(json.dumps(original))
    if mutation == "missing":
        candidate.pop("gpgv_status_policy")
    elif mutation == "drift":
        candidate["gpgv_status_policy"]["proof"] = "status_tags_only"
    else:
        candidate["gpgv_status_policy_extra"] = True
    candidate["canonical_sha256"] = capture.canonical_hash(candidate)
    _rewrite_sealed_json(capture, manifest_path, candidate)
    try:
        with pytest.raises(capture.CaptureError) as error:
            capture._read_source_manifest(output)
        assert error.value.kind in {
            "SOURCE_MANIFEST_INVALID", "SOURCE_MANIFEST_GPGV_POLICY_INVALID",
        }
    finally:
        _rewrite_sealed_json(capture, manifest_path, original)


def test_v2_source_manifest_is_accepted_by_the_composer_validator(capture, tmp_path):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)
    runner, _ = _gpgv_runner(capture)
    capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)
    composer = _load_composer()
    manifest_path = output / "source-manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    result, manifest_sha = composer._validate_source_snapshot(
        output / capture.APT_SOURCE_RELATIVE, manifest_path, manifest["repositories"])
    assert result["gpgv_status_policy"] == composer.GPGV_STATUS_POLICY
    assert manifest_sha == capture.sha256_bytes(manifest_path.read_bytes())


def test_v2_keyring_descriptor_rejects_legacy_shape_and_scope_substitution(
        capture, tmp_path):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)
    runner, _ = _gpgv_runner(capture)
    capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)
    manifest_path = output / "source-manifest.json"
    original = json.loads(manifest_path.read_text(encoding="utf-8"))

    legacy = json.loads(json.dumps(original))
    legacy["keyrings"] = [{
        key: item[key] for key in ("path", "kind", "source", "bytes", "sha256")
    } for item in legacy["keyrings"]]
    legacy["canonical_sha256"] = capture.canonical_hash(legacy)
    _rewrite_sealed_json(capture, manifest_path, legacy)
    with pytest.raises(capture.CaptureError) as error:
        capture._read_source_manifest(output)
    assert error.value.kind == "KEYRING_DESCRIPTOR_INVALID"

    substituted = json.loads(json.dumps(original))
    substituted["source_entries"][0]["signed_by"]["artifact_path"] = (
        "apt-keyrings/not-the-bound-key.gpg")
    substituted["canonical_sha256"] = capture.canonical_hash(substituted)
    _rewrite_sealed_json(capture, manifest_path, substituted)
    with pytest.raises(capture.CaptureError) as error:
        capture._read_source_manifest(output)
    assert error.value.kind in {
        "SOURCE_MANIFEST_ENTRIES_DRIFT", "SOURCE_MANIFEST_KEYRING_BINDING_MISSING",
    }


def test_deb822_inline_key_dot_continuation_is_unfolded(capture):
    source = ("Types: deb\nURIs: https://packages.ros.org/ros2/ubuntu\n"
              "Suites: jazzy\nComponents: main\n"
              "Signed-By: -----BEGIN PGP PUBLIC KEY BLOCK-----\n"
              " Version: synthetic\n .\n AQID\n"
              " -----END PGP PUBLIC KEY BLOCK-----\n").encode("ascii")
    records, _ = capture._source_urls(source, "inline source")
    signed_by = records[0]["signed_by"]
    assert signed_by["kind"] == "inline"
    assert signed_by["source"] == "inline"
    assert b"\n.\n" not in signed_by["data"]
    assert signed_by["dearmored_data"] == b"\x01\x02\x03"


def test_inline_key_armor_rejects_malformed_base64(capture):
    with pytest.raises(capture.CaptureError) as error:
        capture._parse_signed_by(
            "-----BEGIN PGP PUBLIC KEY BLOCK-----\n"
            "not-base64!\n"
            "-----END PGP PUBLIC KEY BLOCK-----\n", "inline key")
    assert error.value.kind == "APT_INLINE_KEY_INVALID"


def test_inline_gpgv_uses_only_dearmored_keyring(capture, tmp_path):
    output = tmp_path / "output"
    capture.EVIDENCE_DIRECTORIES.create_fresh_root(output)
    capture._create_capture_directory_layout(output, include_prefetch=False)
    armor = b"-----BEGIN PGP PUBLIC KEY BLOCK-----\nsynthetic\n"
    binary = b"synthetic-binary-keyring\n"
    armor_descriptor = capture._copy_capture_file(
        output, "apt-keyrings/inline-test.asc", armor)
    binary_descriptor = capture._copy_capture_file(
        output, "apt-keyrings/dearmored-test.gpg", binary)
    fingerprint = "A" * 40
    keyring = {
        "path": "apt-keyrings/inline-test.asc", "kind": "inline",
        "source": "inline", "bytes": len(armor),
        "sha256": armor_descriptor["sha256"],
        "normalized_armor_bytes": len(armor),
        "normalized_armor_sha256": armor_descriptor["sha256"],
        "dearmored_path": "apt-keyrings/dearmored-test.gpg",
        "dearmored_bytes": len(binary),
        "dearmored_sha256": binary_descriptor["sha256"],
        "primary_fingerprints": [fingerprint],
        "subkey_fingerprints": [],
        "expected_signer_fingerprints": [fingerprint],
        "inventory": {"status": "SYNTHETIC_TEST_ONLY"},
        "fingerprint": fingerprint, "signer_fingerprint": fingerprint,
    }
    runner, calls = _gpgv_runner(capture, fingerprint=fingerprint)
    evidence = capture._verify_release_signature(
        None, Path("/var/lib/apt/lists/ros-InRelease"), "inrelease", [],
        runner, output, 0, inline_keyrings=[keyring])
    gpgv_calls = [call for call in calls if call and call[0] == "gpgv"]
    assert len(gpgv_calls) == 1
    assert gpgv_calls[0][4] == str(output / keyring["dearmored_path"])
    assert str(output / keyring["path"]) not in gpgv_calls[0]
    assert capture._validate_gpgv_evidence(
        output, [evidence], require_nonempty=True) == [evidence]

    armor_command = json.loads(json.dumps(evidence))
    armor_command["command"][4] = str(output / keyring["path"])
    armor_command["command_sha256"] = capture._command_hash(
        armor_command["command"])
    with pytest.raises(capture.CaptureError) as error:
        capture._validate_gpgv_evidence(
            output, [armor_command], require_nonempty=True)
    assert error.value.kind == "GPGV_EVIDENCE_KEYRING_INVALID"


def test_release_size_policy_accepts_large_zero_and_by_hash_entries(capture):
    digest = "a" * 64
    by_hash = "main/binary-amd64/by-hash/SHA256/" + "b" * 64
    release = (
        "Suite: jazzy\nSHA256:\n"
        " {} 829119597 Contents-amd64\n"
        " {} 5869093136 Contents-arm64\n"
        " {} 7240371125 Contents-all\n"
        " {} 0 {}\n"
        " {} {} Contents-i386\n"
    ).format(digest, digest, digest, digest, by_hash, digest,
             capture.MAX_RELEASE_DECLARED_BYTES).encode("ascii")
    entries = capture._release_sha256_entries(release, "large Release")
    assert entries["Contents-amd64"] == (digest, 829119597)
    assert entries["Contents-arm64"] == (digest, 5869093136)
    assert entries["Contents-all"] == (digest, 7240371125)
    assert entries[by_hash] == (digest, 0)
    assert entries["Contents-i386"] == (
        digest, capture.MAX_RELEASE_DECLARED_BYTES)
    assert capture._release_declared_size("0", "zero") == 0
    assert capture._release_declared_size(
        str(capture.MAX_RELEASE_DECLARED_BYTES), "bound") == (
            capture.MAX_RELEASE_DECLARED_BYTES)
    # The signed declaration bound is deliberately independent of the
    # materialized Packages/index limit.
    assert capture.MAX_RELEASE_DECLARED_BYTES > capture.MAX_CAPTURE_FILE_BYTES
    assert capture.MAX_RELEASE_DECLARED_BYTES == 17179869184
    assert capture.MAX_CAPTURE_FILE_BYTES == 64 * 1024 * 1024
    assert capture.MAX_CAPTURE_DEB_BYTES == 512 * 1024 * 1024


def test_signed_acquire_by_hash_selects_digest_url(capture, tmp_path,
                                                   monkeypatch):
    decoded = b"Package: demo\nVersion: 1\nArchitecture: amd64\n"
    raw = lzma.compress(decoded, format=lzma.FORMAT_XZ)
    raw_sha = capture.sha256_bytes(raw)
    repository = {
        "url": "http://archive.ubuntu.com/ubuntu",
        "suite": "jammy-updates", "component": "universe",
    }
    plain = "universe/binary-amd64/Packages"
    compressed = plain + ".xz"
    release = (
        "Acquire-By-Hash: yes\nSHA256:\n"
        " {} {} {}\n {} {} {}\n"
    ).format(raw_sha, len(raw), compressed,
             capture.sha256_bytes(decoded), len(decoded), plain).encode("ascii")
    requested = []

    def fetch(url, _timeout):
        requested.append(url)
        return raw

    monkeypatch.setattr(
        capture, "_packages_candidate_names",
        lambda *_args: {"xz": [], "gz": [], "plain": []})
    selected = capture._select_packages_index(
        tmp_path, repository, "inrelease",
        capture._release_sha256_entries(release, "synthetic"),
        capture.APT_SOURCE_URL_POLICY, fetch,
        release_identity="a" * 64,
        release_record={"path": "indices/InRelease-" + "b" * 24},
        release_data=release)
    expected = (
        "http://archive.ubuntu.com/ubuntu/dists/jammy-updates/universe/"
        "binary-amd64/by-hash/SHA256/" + raw_sha)
    assert requested == [expected]
    assert selected["packages_url"] == expected


@pytest.mark.parametrize("header", [
    b"Acquire-By-Hash: maybe\n",
    b"Acquire-By-Hash: yes\nAcquire-By-Hash: yes\n",
    b"Acquire-By-Hash: yes \n",
])
def test_acquire_by_hash_rejects_malformed_or_duplicate_signed_flags(
        capture, header):
    with pytest.raises(capture.CaptureError) as error:
        capture._release_acquire_by_hash(header, "synthetic Release")
    assert error.value.kind == "RELEASE_BY_HASH_INVALID"


@pytest.mark.parametrize("token", [
    "-1", "+1", "01", "1x", "17179869185",
    "999999999999999999999999999999999999999999999999999999999999999999",
    "１２",
])
def test_release_size_policy_rejects_noncanonical_or_overbound_tokens(capture, token):
    digest = "a" * 64
    release = ("Suite: jazzy\nSHA256:\n {} {} main/binary-amd64/Packages\n"
               ).format(digest, token).encode("utf-8")
    with pytest.raises(capture.CaptureError) as error:
        capture._release_sha256_entries(release, "invalid Release")
    assert error.value.kind == "RELEASE_SHA256_SIZE_INVALID"


def test_release_declared_size_rejects_whitespace_before_conversion(capture):
    for token in (" 1", "1 ", "\t1", "1\t"):
        with pytest.raises(capture.CaptureError) as error:
            capture._release_declared_size(token, "whitespace")
        assert error.value.kind == "RELEASE_SHA256_SIZE_INVALID"


def test_release_declaration_bound_does_not_change_download_limits(capture):
    assert capture.PACKAGES_FORMAT_POLICY["compressed_max_bytes"] == (
        capture.MAX_CAPTURE_FILE_BYTES)
    assert capture.PACKAGES_FORMAT_POLICY["decoded_max_bytes"] == (
        capture.MAX_DECODED_PACKAGES_BYTES)
    assert capture.MAX_CAPTURE_FILE_BYTES == 64 * 1024 * 1024
    assert capture.MAX_DECODED_PACKAGES_BYTES == 256 * 1024 * 1024
    assert capture.MAX_CAPTURE_DEB_BYTES == 512 * 1024 * 1024


def test_packages_catalog_size_is_independent_from_selected_deb_limit(capture):
    digest = "a" * 64
    catalog_size = capture.MAX_CAPTURE_DEB_BYTES + 1

    def stanza(size):
        return (
            "Package: unrelated-large-package\n"
            "Version: 1.0\n"
            "Architecture: amd64\n"
            "Filename: pool/u/unrelated-large-package.deb\n"
            "Size: {}\n"
            "SHA256: {}\n".format(size, digest)
        ).encode("ascii")

    records = capture._packages_records(stanza(catalog_size), "signed catalog")
    assert records[("unrelated-large-package", "1.0", "amd64")]["bytes"] == (
        catalog_size)

    with pytest.raises(capture.CaptureError) as error:
        capture._packages_records(
            stanza(capture.MAX_RELEASE_DECLARED_BYTES + 1), "signed catalog")
    assert error.value.kind == "PACKAGES_SIZE_INVALID"

    with pytest.raises(capture.CaptureError) as error:
        capture._parse_print_uri(
            "https://example.invalid/unrelated-large-package.deb "
            "unrelated-large-package.deb {} SHA256:{}".format(catalog_size, digest),
            "selected package")
    assert error.value.kind == "RESOLVER_SIZE_INVALID"


def test_packages_parser_accepts_empty_optional_value_with_continuation(capture):
    stanza = (
        "Package: btm\n"
        "Version: 0.9.6-4\n"
        "Architecture: amd64\n"
        "Filename: pool/universe/b/btm/btm_0.9.6-4_amd64.deb\n"
        "Size: 1607224\n"
        "SHA256: " + "a" * 64 + "\n"
        "X-Cargo-Built-Using:\n"
        " rust-addr2line (= 0.21.0-2), rust-adler (= 1.0.2-2)\n"
    ).encode("ascii")

    records = capture._packages_records(stanza, "noble universe")

    assert ("btm", "0.9.6-4", "amd64") in records


@pytest.mark.parametrize("invalid", [
    "X-Cargo-Built-Using:\n",
    "Package:\n continued\n",
    "Bad_Field:\n continued\n",
])
def test_packages_parser_rejects_unbound_or_identity_empty_fields(capture, invalid):
    stanza = (
        "Package: valid\n"
        "Version: 1.0\n"
        "Architecture: amd64\n"
        "Filename: pool/v/valid.deb\n"
        "Size: 1\n"
        "SHA256: " + "a" * 64 + "\n" + invalid
    ).encode("ascii")

    with pytest.raises(capture.CaptureError) as error:
        capture._packages_records(stanza, "invalid optional field")
    assert error.value.kind in {"PACKAGES_FIELD_INVALID", "PACKAGES_FIELD_DUPLICATE"}


def test_resolver_separates_epoch_cache_name_from_pool_url_name(capture):
    line = (
        "'http://archive.ubuntu.com/ubuntu/pool/main/libc/libcap2/"
        "libcap2-bin_2.44-1ubuntu0.22.04.3_amd64.deb' "
        "libcap2-bin_1%3a2.44-1ubuntu0.22.04.3_amd64.deb 26018 "
        "MD5Sum:" + "a" * 32)
    uri = capture._parse_print_uri(line, "epoch package")
    assert uri["url_filename"] == "libcap2-bin_2.44-1ubuntu0.22.04.3_amd64.deb"
    assert uri["filename"] == "libcap2-bin_1%3a2.44-1ubuntu0.22.04.3_amd64.deb"
    assert capture._resolver_cache_filename(
        ("libcap2-bin", "1:2.44-1ubuntu0.22.04.3", "amd64")) == uri["filename"]


def test_resolver_canonicalizes_graph_order_but_rejects_duplicates(capture):
    digest = "a" * 32
    first = (
        "'http://archive.ubuntu.com/ubuntu/pool/z/zeta.deb' zeta.deb 2 MD5Sum:" +
        digest)
    second = (
        "'http://archive.ubuntu.com/ubuntu/pool/a/alpha.deb' alpha.deb 1 MD5Sum:" +
        digest)
    uris = capture._canonical_resolver_uris([first, second], "build")
    assert [item["filename"] for item in uris] == ["alpha.deb", "zeta.deb"]
    projected = [capture._canonical_resolver_line(item) for item in uris]
    assert [capture._parse_print_uri(line, "projected")["filename"]
            for line in projected] == ["alpha.deb", "zeta.deb"]
    with pytest.raises(capture.CaptureError) as error:
        capture._canonical_resolver_uris([first, first], "build")
    assert error.value.kind == "RESOLVER_URI_SET_INVALID"


def test_apt_cache_layout_allows_only_debs_lock_and_empty_partial(capture, tmp_path):
    root = tmp_path / "cache"
    root.mkdir()
    deb = root / "demo.deb"
    deb.write_bytes(b"deb")
    deb.chmod(0o644)
    lock = root / "lock"
    lock.write_bytes(b"")
    lock.chmod(0o640)
    partial = root / "partial"
    partial.mkdir(mode=0o700)
    partial.chmod(0o700)
    assert capture._apt_cache_debs(root, {"demo.deb"}, "build") == [deb]

    extra = root / "unexpected"
    extra.write_bytes(b"")
    extra.chmod(0o644)
    with pytest.raises(capture.CaptureError):
        capture._apt_cache_debs(root, {"demo.deb"}, "build")
    extra.unlink()

    residue = partial / "residue"
    residue.write_bytes(b"")
    with pytest.raises(capture.CaptureError):
        capture._apt_cache_debs(root, {"demo.deb"}, "build")


def test_package_copyright_records_bounded_relative_doc_alias(capture, tmp_path):
    root = tmp_path / "container"
    doc = root / "usr/share/doc"
    target = doc / "shared-doc"
    target.mkdir(parents=True)
    target.chmod(0o755)
    copyright_path = target / "copyright"
    copyright_path.write_bytes(b"license\n")
    copyright_path.chmod(0o644)
    (doc / "demo").symlink_to("shared-doc")

    data, descriptor, resolved, aliases, status = capture._package_copyright(root, "demo")
    assert data == b"license\n"
    assert status == "PRESENT"
    assert resolved == copyright_path
    assert descriptor["sha256"] == capture.sha256_bytes(data)
    assert aliases[0]["path"] == str(doc / "demo")
    assert aliases[0]["target"] == "shared-doc"

    (doc / "shared-doc").rename(doc / "shared-doc-real")
    (doc / "shared-doc").symlink_to("demo")
    with pytest.raises(capture.CaptureError) as error:
        capture._package_copyright(root, "demo")
    assert error.value.kind == "COPYRIGHT_ALIAS_CYCLE"


def test_package_copyright_records_explicit_dpkg_inventory_absence(capture, tmp_path):
    root = tmp_path / "container"
    package_doc = root / "usr/share/doc/demo"
    package_doc.mkdir(parents=True)
    package_doc.chmod(0o755)
    inventory = root / "var/lib/dpkg/info/demo.list"
    inventory.parent.mkdir(parents=True)
    inventory.write_bytes(b"/.\n/usr\n/usr/share\n/usr/share/doc/demo\n")
    inventory.chmod(0o644)

    data, descriptor, resolved, aliases, status = capture._package_copyright(root, "demo")
    assert data == inventory.read_bytes()
    assert descriptor["sha256"] == capture.sha256_bytes(data)
    assert resolved == inventory
    assert aliases == []
    assert status == "ABSENT_FROM_DPKG_FILE_INVENTORY"

    inventory.write_bytes(data + b"/usr/share/doc/demo/copyright\n")
    with pytest.raises(capture.CaptureError) as error:
        capture._package_copyright(root, "demo")
    assert error.value.kind == "COPYRIGHT_INVENTORY_DRIFT"


@pytest.mark.parametrize("line", [
    " {digest} 1 main/binary-amd64/Packages\n"
    " {digest} 2 main/binary-amd64/Packages\n",
    " {digest} 1 main/binary-amd64/Packages\n"
    " {other} 1 main/binary-amd64/Packages\n",
])
def test_release_size_policy_rejects_duplicate_or_conflicting_entries(capture, line):
    digest = "a" * 64
    other = "b" * 64
    rendered = line.format(digest=digest, other=other)
    release = ("Suite: jazzy\nSHA256:\n" + rendered).encode("ascii")
    with pytest.raises(capture.CaptureError) as error:
        capture._release_sha256_entries(release, "duplicate Release")
    assert error.value.kind == "RELEASE_SHA256_DUPLICATE"


def test_release_size_policy_accepts_clear_signed_payload(capture):
    digest = "a" * 64
    cleartext = ("Suite: jazzy\nSHA256:\n"
                 " {} 829119597 Contents-amd64\n".format(digest))
    signed = ("-----BEGIN PGP SIGNED MESSAGE-----\nHash: SHA256\n\n" +
              cleartext +
              "-----BEGIN PGP SIGNATURE-----\nVersion: synthetic\n\n"
              "-----END PGP SIGNATURE-----\n").encode("ascii")
    assert capture._release_sha256_entries(signed, "clear-signed Release") == {
        "Contents-amd64": (digest, 829119597)}


def test_release_evidence_is_written_before_declaration_parse_failure(capture, tmp_path):
    container, output, release_name, _, _, _ = _signed_container_fixture(capture, tmp_path)
    release_path = container / "var/lib/apt/lists" / release_name
    digest = "a" * 64
    invalid = ("Suite: jazzy\nSHA256:\n {} {} main/binary-amd64/Packages\n"
               ).format(digest, capture.MAX_RELEASE_DECLARED_BYTES + 1).encode("ascii")
    _write_fixture(release_path, invalid)
    runner, _ = _gpgv_runner(capture)
    with pytest.raises(capture.CaptureError) as error:
        capture._capture_in_container(
            "apt-source", output, "jazzy", container_root=container,
            command_runner=runner)
    assert error.value.kind == "RELEASE_SHA256_SIZE_INVALID"
    failure = json.loads(
        (output / "in-container-apt-source.json").read_text(encoding="utf-8"))
    evidence = failure["release_evidence"]
    assert evidence[0]["sha256"] == capture.sha256_bytes(invalid)
    assert capture._validate_release_evidence(output, evidence, require_nonempty=True) == evidence
    raw_path = output / evidence[0]["output"]["path"]
    raw_path.chmod(0o644)
    raw_path.write_bytes(invalid + b"tampered")
    raw_path.chmod(0o444)
    with pytest.raises(capture.CaptureError) as drift:
        capture._validate_release_evidence(output, evidence, require_nonempty=True)
    assert drift.value.kind == "RELEASE_EVIDENCE_DRIFT"


def test_partial_gpgv_failure_projects_outer_release_evidence_first(capture, tmp_path):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)
    base_runner, _ = _gpgv_runner(capture)

    def runner(argv, timeout):
        result = base_runner(argv, timeout)
        if argv and argv[0] == "gpgv" and argv != ["gpgv", "--version"]:
            return result[0], result[1] + b"[GNUPG:] UNKNOWN informational\n", result[2], result[3], result[4]
        return result

    with pytest.raises(capture.CaptureError) as error:
        capture._capture_in_container(
            "apt-source", output, "jazzy", container_root=container,
            command_runner=runner)
    assert error.value.kind == "GPGV_STATUS_UNEXPECTED_TAG"
    failure = json.loads(
        (output / "in-container-apt-source.json").read_text(encoding="utf-8"))
    assert failure["status"] == "FAILED_REVIEW_REQUIRED"
    assert failure["failure"]["kind"] == "GPGV_STATUS_UNEXPECTED_TAG"
    assert failure["release_evidence"]
    assert capture._validate_inner_directory_binding(
        output / "in-container-apt-source.json", failure["directory_contract"])


def test_clear_signed_and_detached_release_evidence_are_distinct(capture, tmp_path):
    container, output, release_name, _, _, _ = _signed_container_fixture(
        capture, tmp_path)
    lists = container / "var/lib/apt/lists"
    release_path = lists / release_name
    release = release_path.read_bytes()
    inrelease_name = release_name.replace("_Release", "_InRelease")
    release_path.rename(lists / inrelease_name)
    (lists / (release_name + ".gpg")).unlink()
    clear = ("-----BEGIN PGP SIGNED MESSAGE-----\nHash: SHA256\n\n" +
             release.decode("ascii") +
             "-----BEGIN PGP SIGNATURE-----\nVersion: synthetic\n\n"
             "-----END PGP SIGNATURE-----\n").encode("ascii")
    _write_fixture(lists / inrelease_name, clear)
    runner, _ = _gpgv_runner(capture)
    result = capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)
    assert result["release_evidence"][0]["kind"] == "inrelease"
    assert result["release_evidence"][0]["signature"] is None


def test_materialized_index_limit_remains_separate_from_release_declaration(capture):
    original = capture.MAX_CAPTURE_FILE_BYTES
    try:
        capture.MAX_CAPTURE_FILE_BYTES = 8
        with pytest.raises(capture.CaptureError) as error:
            capture._decode_packages_index(Path("Packages"), b"123456789", lambda *_: None)
        assert error.value.kind == "PACKAGES_INDEX_INVALID"
    finally:
        capture.MAX_CAPTURE_FILE_BYTES = original


def test_decoded_packages_limit_is_independent_from_compressed_input_limit(capture):
    original = capture.MAX_DECODED_PACKAGES_BYTES
    try:
        capture.MAX_DECODED_PACKAGES_BYTES = 1024
        accepted = b"Package: demo\n" + b"x" * 900
        assert capture._decode_packages_index(
            Path("Packages.xz"), lzma.compress(accepted), expected_format="xz"
        )[0] == accepted
        rejected = b"Package: demo\n" + b"x" * 1100
        with pytest.raises(capture.CaptureError) as error:
            capture._decode_packages_index(
                Path("Packages.xz"), lzma.compress(rejected), expected_format="xz"
            )
        assert error.value.kind == "PACKAGES_INDEX_INVALID"
    finally:
        capture.MAX_DECODED_PACKAGES_BYTES = original


def test_xz_terminal_empty_stream_policy_binds_real_two_stream_fixture(capture):
    payload = b"Package: demo\nVersion: 1.0\nArchitecture: amd64\n"
    first = lzma.compress(payload, format=lzma.FORMAT_XZ)
    terminal = lzma.compress(b"", format=lzma.FORMAT_XZ)
    decoded, streams = capture._decode_xz_stream_details(
        first + terminal, "two-stream Packages.xz")
    assert decoded == payload
    assert len(terminal) == 32
    assert streams == [
        {"index": 0, "compressed_offset": 0, "compressed_bytes": len(first),
         "compressed_sha256": capture.sha256_bytes(first),
         "decoded_bytes": len(payload)},
        {"index": 1, "compressed_offset": len(first), "compressed_bytes": 32,
         "compressed_sha256": capture.sha256_bytes(terminal),
         "decoded_bytes": 0},
    ]


def test_signed_empty_packages_accepts_jammy_backports_restricted_fixture(capture):
    """The signed zero-entry Ubuntu index has a narrow, explicit exception."""
    raw = lzma.compress(b"", format=lzma.FORMAT_XZ)
    decoded, streams = capture._decode_xz_stream_details(
        raw, "jammy-backports restricted Packages.xz",
        allow_signed_empty=True)
    assert decoded == b""
    assert len(raw) == 32
    assert capture.sha256_bytes(raw) == (
        "0040f94d11d0039505328a90b2ff48968db873e9e7967307631bf40ef5679275")
    assert streams == [{
        "index": 0, "compressed_offset": 0, "compressed_bytes": 32,
        "compressed_sha256": capture.sha256_bytes(raw), "decoded_bytes": 0,
    }]
    value = {
        "domain": capture.PACKAGE_ARTIFACT_IDENTITY_DOMAIN,
        "repository_url": "http://archive.ubuntu.com/ubuntu/",
        "suite": "jammy-backports", "component": "restricted",
        "architecture": "amd64", "release_kind": "inrelease",
        "release_identity": "d4cc84935f67c0884d0bd7b02dac24cb27ff02f91b8562650231d90f08880ca4",
        "release_url": "http://archive.ubuntu.com/ubuntu/dists/jammy-backports/InRelease",
        "release_record_path": "indices/archive.ubuntu.com_ubuntu_dists_jammy-backports_InRelease-d4cc84935f67c0884d0bd7b0",
        "release_file_sha256": "9944bc3e053fb069a1e557b0a1724dad034353171b14f84dd48cf0b1249b5a01",
        "release_file_bytes": 127185,
        "signed_compressed_path": "restricted/binary-amd64/Packages.xz",
        "signed_plain_path": "restricted/binary-amd64/Packages",
        "format": "xz",
        "packages_url": "http://archive.ubuntu.com/ubuntu/dists/jammy-backports/restricted/binary-amd64/Packages.xz",
        "compressed_sha256": capture.sha256_bytes(raw), "compressed_bytes": 32,
        "decoded_sha256": capture.EMPTY_PACKAGES_SHA256, "decoded_bytes": 0,
        "signed_empty": True,
    }
    value["identity"] = capture.canonical_hash(value, "identity")
    assert capture._validate_package_artifact_identity(
        value, "jammy-backports restricted") == value


@pytest.mark.parametrize("mutation", [
    {"signed_empty": False},
    {"decoded_sha256": "f" * 64},
    {"format": "plain"},
    {"compressed_bytes": 31},
])
def test_signed_empty_packages_rejects_implicit_or_mismatched_zero(capture, mutation):
    value = {
        "domain": capture.PACKAGE_ARTIFACT_IDENTITY_DOMAIN,
        "repository_url": "https://mirror.example.invalid/ubuntu",
        "suite": "jammy-backports", "component": "restricted",
        "architecture": "amd64", "release_kind": "inrelease",
        "release_identity": "a" * 64,
        "release_url": "https://mirror.example.invalid/ubuntu/dists/jammy-backports/InRelease",
        "release_record_path": "indices/InRelease-" + "b" * 24,
        "release_file_sha256": "c" * 64, "release_file_bytes": 1,
        "signed_compressed_path": "restricted/binary-amd64/Packages.xz",
        "signed_plain_path": "restricted/binary-amd64/Packages",
        "format": "xz", "packages_url": "https://mirror.example.invalid/ubuntu/dists/jammy-backports/restricted/binary-amd64/Packages.xz",
        "compressed_sha256": "d" * 64, "compressed_bytes": 32,
        "decoded_sha256": capture.EMPTY_PACKAGES_SHA256, "decoded_bytes": 0,
        "signed_empty": True,
    }
    value.update(mutation)
    value["identity"] = capture.canonical_hash(value, "identity")
    with pytest.raises(capture.CaptureError) as error:
        capture._validate_package_artifact_identity(value, "signed empty mutation")
    assert error.value.kind == "PACKAGES_ARTIFACT_IDENTITY_INVALID"


def test_signed_empty_decoder_rejects_gzip_plain_and_extra_xz(capture):
    empty_xz = lzma.compress(b"", format=lzma.FORMAT_XZ)
    with pytest.raises(capture.CaptureError):
        capture._decode_xz_stream_details(
            empty_xz + empty_xz, "two empty streams", allow_signed_empty=True)
    with pytest.raises(capture.CaptureError):
        capture._decode_packages_index_details(
            Path("Packages.gz"), b"", expected_format="gz",
            allow_signed_empty=True)
    with pytest.raises(capture.CaptureError):
        capture._decode_packages_index_details(
            Path("Packages"), b"", expected_format="plain",
            allow_signed_empty=True)


@pytest.mark.parametrize("suffix", [
    lzma.compress(b"data", format=lzma.FORMAT_XZ),
    lzma.compress(b"", format=lzma.FORMAT_XZ) +
    lzma.compress(b"", format=lzma.FORMAT_XZ),
    b"\x00\x00\x00\x00",
    b"junk",
])
def test_xz_terminal_empty_stream_policy_rejects_unapproved_tail(capture, suffix):
    first = lzma.compress(b"Packages\n", format=lzma.FORMAT_XZ)
    with pytest.raises(capture.CaptureError):
        capture._decode_xz_stream(first + suffix, "invalid Packages.xz")


def test_package_evidence_has_no_legacy_path_and_rejects_extra_field(capture, tmp_path):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)
    runner, _ = _gpgv_runner(capture)
    result = capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)
    item = result["package_evidence"][0]
    assert "path" not in item
    assert "stream_descriptors" in item
    invalid = dict(item)
    invalid["path"] = None
    with pytest.raises(capture.CaptureError) as error:
        capture._validate_package_evidence(output, [invalid], require_decoded=True)
    assert error.value.kind == "PACKAGES_EVIDENCE_INVALID"


def test_partial_xz_decode_preserves_only_release_prefix(
        capture, tmp_path):
    container, output, release_name, packages_name, _, _ = (
        _signed_container_fixture(capture, tmp_path))
    lists = container / "var/lib/apt/lists"
    packages = (lists / packages_name).read_bytes()
    (lists / packages_name).unlink()
    first = lzma.compress(packages, format=lzma.FORMAT_XZ)
    empty = lzma.compress(b"", format=lzma.FORMAT_XZ)
    invalid = first + empty + empty
    _write_fixture(lists / (packages_name + ".xz"), invalid)
    release = ("Origin: synthetic\nSuite: jazzy\nSHA256:\n"
               " {} {} main/binary-amd64/Packages.xz\n"
               " {} {} main/binary-amd64/Packages\n").format(
                   capture.sha256_bytes(invalid), len(invalid),
                   capture.sha256_bytes(packages), len(packages)).encode()
    _write_fixture(lists / release_name, release)
    runner, _ = _gpgv_runner(capture)
    with pytest.raises(capture.CaptureError) as error:
        capture._capture_in_container(
            "apt-source", output, "jazzy", container_root=container,
            command_runner=runner)
    assert error.value.kind == "PACKAGES_TRAILING_DATA"
    failure = json.loads(
        (output / "in-container-apt-source.json").read_text(encoding="utf-8"))
    assert failure["release_evidence"]
    assert "package_evidence" not in failure
    assert "signature_evidence" not in failure
    assert failure["signature_references"] == []
    assert failure["signature_count"] == 0
    assert failure["reference_map"] == {
        field: {} for field in capture.REFERENCE_MAP_FIELDS}
    assert failure["reference_counts"] == capture._reference_projection(
        failure["release_evidence"], [], [], 0)[1]
    assert failure["reference_counts"]["release_evidence"] == 1
    assert failure["reference_counts"]["package_evidence"] == 0
    assert capture._validate_inner_directory_binding(
        output / "in-container-apt-source.json", failure["directory_contract"])


def test_release_evidence_missing_extra_and_signature_tamper_reject(capture, tmp_path):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)
    runner, _ = _gpgv_runner(capture)
    result = capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)
    evidence = result["release_evidence"]
    with pytest.raises(capture.CaptureError):
        capture._validate_release_evidence(output, [], require_nonempty=True)
    with pytest.raises(capture.CaptureError):
        capture._validate_release_evidence(output, evidence + evidence)
    altered = json.loads(json.dumps(evidence))
    altered[0]["signature"]["sha256"] = "0" * 64
    with pytest.raises(capture.CaptureError) as error:
        capture._validate_release_evidence(output, altered)
    assert error.value.kind == "RELEASE_EVIDENCE_DRIFT"


def test_in_container_dependency_binds_md5_locator_to_deb_and_control(capture, tmp_path):
    container, output, _, _, deb, deb_sha = _signed_container_fixture(capture, tmp_path)
    runner, _ = _gpgv_runner(capture, dpkg=True)
    capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)
    _mkdir(output / "logs")
    build_log = _write_fixture(output / "logs/resolver_build.stdout", (
        "https://mirror.example.invalid/ubuntu/pool/main/d/demo/demo_1.0_amd64.deb "
        "demo_1.0_amd64.deb 13 MD5Sum:00000000000000000000000000000000\n").encode())
    runtime_log = _write_fixture(output / "logs/resolver_runtime.stdout", (
        "https://mirror.example.invalid/ubuntu/pool/main/d/demo/demo_1.0_amd64.deb "
        "demo_1.0_amd64.deb 13 MD5Sum:00000000000000000000000000000000\n").encode())
    build_log.chmod(0o444)
    runtime_log.chmod(0o444)
    _mkdir(output / "debs-build")
    _mkdir(output / "debs-runtime")
    for role in ("build", "runtime"):
        lock = _write_fixture(output / "debs-{}".format(role) / "lock", b"")
        lock.chmod(0o640)
        partial = output / "debs-{}".format(role) / "partial"
        partial.mkdir(mode=0o700, exist_ok=True)
        partial.chmod(0o700)
    _write_fixture(output / "debs-build/demo_1.0_amd64.deb", deb)
    _write_fixture(output / "debs-runtime/demo_1.0_amd64.deb", deb)
    base = {"schema": "glim_clean_room_r3_base_dpkg_status_v1", "schema_version": 1,
            "status": "SEALED_PREINSTALL", "packages": [{
                "name": "base", "version": "1.0", "architecture": "amd64",
                "status": "install ok installed"}]}
    base["canonical_sha256"] = capture.canonical_hash(base)
    capture._seal_pair(output, "base-status.json", base)
    rosdep_log = _write_fixture(output / "logs/rosdep_resolution.stdout", b"rosdep resolved demo\n")
    rosdep_log.chmod(0o444)
    result = capture._capture_in_container(
        "dependency", output, "jazzy", container_root=container,
        command_runner=runner)
    assert result["status"] == "CAPTURED_REVIEW_REQUIRED"
    assert len(result["packages"]) == 2
    assert result["packages"][0]["sha256"] == deb_sha
    assert result["packages"][0]["uri_digest_algorithm"] == "md5"
    assert (output / "captured-debs/build/demo_1.0_amd64.deb").stat().st_mode & 0o777 == 0o444
    assert result["packages"][0]["copyright"]["sha256"] == capture.sha256_bytes(b"Demo license\n")
    assert (output / "in-container-dependency.json").is_file()
    reopened = capture._validate_inner_directory_binding(
        output / "in-container-dependency.json", result["directory_contract"])
    assert reopened["phase"] == "dependency"


def test_in_container_rejects_bad_release_signature_and_seals_failure(capture, tmp_path):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)
    runner, _ = _gpgv_runner(capture, bad=True)
    with pytest.raises(capture.CaptureError) as error:
        capture._capture_in_container(
            "apt-source", output, "jazzy", container_root=container,
            command_runner=runner)
    assert error.value.kind == "GPGV_VERIFY_FAILED"
    failure = json.loads(
        (output / "in-container-apt-source.json").read_text(encoding="utf-8"))
    assert failure["status"] == "FAILED_REVIEW_REQUIRED"
    assert failure["failure"]["kind"] == "GPGV_VERIFY_FAILED"
    assert "IN_CONTAINER_CAPTURE_INCOMPLETE" not in (output / "in-container-apt-source.json").read_text()


def test_in_container_rejects_release_index_tamper(capture, tmp_path):
    container, output, _, packages_name, _, _ = _signed_container_fixture(capture, tmp_path)
    packages = container / "var/lib/apt/lists" / packages_name
    packages.write_bytes(packages.read_bytes().replace(b"Package: demo", b"Package: drift"))
    runner, _ = _gpgv_runner(capture)
    with pytest.raises(capture.CaptureError) as error:
        capture._capture_in_container(
            "apt-source", output, "jazzy", container_root=container,
            command_runner=runner)
    assert error.value.kind in {"PACKAGES_RELEASE_BINDING_DRIFT", "PACKAGES_RELEASE_ENTRY_MISSING"}


def test_package_identity_uses_each_repository_spec_not_last_discovery_value(
        capture, tmp_path):
    container, output, release_name, packages_name, _, _ = (
        _signed_container_fixture(capture, tmp_path))
    source_path = container / "etc/apt/sources.list.d/ros2.sources"
    source_path.write_bytes(source_path.read_bytes().replace(
        b"Suites: jazzy", b"Suites: jazzy jammy-updates"))
    source_path.chmod(0o644)
    lists = container / "var/lib/apt/lists"
    packages = (lists / packages_name).read_bytes()
    package_sha = capture.sha256_bytes(packages)
    second_prefix = capture._apt_list_prefix(
        "https://mirror.example.invalid/ubuntu", "jammy-updates")
    second_release_name = second_prefix + "_Release"
    second_packages_name = second_prefix + "_main_binary-amd64_Packages"
    _write_fixture(lists / second_packages_name, packages)
    _write_fixture(
        lists / (second_release_name + ".gpg"), b"detached-signature\n")
    second_release = (
        "Origin: synthetic\nSuite: jammy-updates\nSHA256:\n"
        " {} {} main/binary-amd64/Packages\n").format(
            package_sha, len(packages)).encode("ascii")
    _write_fixture(lists / second_release_name, second_release)
    runner, _ = _gpgv_runner(capture)

    result = capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)

    assert len(result["release_evidence"]) == 2
    assert {item["artifact_identity"]["suite"]
            for item in result["package_evidence"]} == {"jazzy", "jammy-updates"}
    release_by_suite = {
        item["url"].split("/dists/", 1)[1].split("/", 1)[0]: item
        for item in result["release_evidence"]
    }
    for item in result["package_evidence"]:
        suite = item["artifact_identity"]["suite"]
        assert item["artifact_identity"]["release_identity"] == (
            release_by_suite[suite]["identity"])
        assert item["release_record_path"] == release_by_suite[suite]["path"]


def test_package_failure_partial_reopen_has_release_only_projection(
        capture, tmp_path, monkeypatch):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)
    source_path = container / "etc/apt/sources.list.d/ros2.sources"
    source_path.write_bytes(source_path.read_bytes().replace(
        b"Suites: jazzy", b"Suites: jazzy jammy jammy-updates noble focal"))
    source_path.chmod(0o644)
    lists = container / "var/lib/apt/lists"
    packages_path = next(lists.glob("*_main_binary-amd64_Packages"))
    packages = packages_path.read_bytes()
    package_sha = capture.sha256_bytes(packages)
    repository_url = "https://mirror.example.invalid/ubuntu"
    for suite in ("jammy", "jammy-updates", "noble", "focal"):
        prefix = capture._apt_list_prefix(repository_url, suite)
        _write_fixture(
            lists / (prefix + "_main_binary-amd64_Packages"), packages)
        release = (
            "Origin: synthetic\nSuite: {}\nSHA256:\n"
                " {} {} main/binary-amd64/Packages\n").format(
                suite, package_sha, len(packages)).encode("ascii")
        _write_fixture(lists / (prefix + "_Release"), release)
        _write_fixture(
            lists / (prefix + "_Release.gpg"), b"detached-signature\n")

    def fail_packages(*args, **kwargs):
        del args, kwargs
        raise capture.CaptureError(
            "PACKAGES_ARTIFACT_BINDING_INVALID", "synthetic package failure")

    monkeypatch.setattr(capture, "_select_packages_index", fail_packages)
    runner, _ = _gpgv_runner(capture)
    with pytest.raises(capture.CaptureError) as error:
        capture._capture_in_container(
            "apt-source", output, "jazzy", container_root=container,
            command_runner=runner)
    assert error.value.kind == "PACKAGES_ARTIFACT_BINDING_INVALID"
    failure = json.loads(
        (output / "in-container-apt-source.json").read_text(encoding="utf-8"))
    assert len(failure["release_evidence"]) == 5
    assert "package_evidence" not in failure
    assert "signature_evidence" not in failure
    assert failure["signature_references"] == []
    assert failure["signature_count"] == 0
    expected_map, expected_counts = capture._reference_projection(
        failure["release_evidence"], [], [], 0)
    assert failure["reference_map"] == expected_map
    assert failure["reference_counts"] == expected_counts
    reopened = capture._validate_inner_directory_binding(
        output / "in-container-apt-source.json", failure["directory_contract"])
    assert reopened["reference_counts"] == expected_counts


def test_missing_rosdep_is_deferred_until_prepare(capture, tmp_path):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)
    (container / "etc/ros/rosdep/sources.list.d/20-default.list").unlink()
    runner, _ = _gpgv_runner(capture)

    result = capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)
    assert result["status"] == "CAPTURED_REVIEW_REQUIRED"
    manifest = json.loads((output / "source-manifest.json").read_text())
    assert {item["role"] for item in manifest["files"]} == {
        "apt_source", "apt_release", "apt_packages"}
    assert "rosdep_source_files" not in result
    assert "rosdep_cache_files" not in result


def test_missing_rosdep_does_not_change_apt_projection(capture, tmp_path):
    container, output, _, packages_name, _, _ = _signed_container_fixture(
        capture, tmp_path)
    source_path = container / "etc/apt/sources.list.d/ros2.sources"
    source = (
        "Types: deb\n"
        "URIs: https://mirror.example.invalid/ubuntu\n"
        "Suites: jazzy jammy jammy-updates noble focal\n"
        "Components: main universe restricted\n"
        "Signed-By: /usr/share/keyrings/ros.gpg\n"
        "Architectures: amd64\n"
    ).encode("ascii")
    source_path.write_bytes(source)
    source_path.chmod(0o644)
    _write_fixture(
        container / "etc/apt/sources.list.d/extra-jazzy.sources",
        source.replace(
            b"Suites: jazzy jammy jammy-updates noble focal",
            b"Suites: jazzy")
            .replace(b"Components: main universe restricted", b"Components: multiverse"))
    _write_fixture(
        container / "etc/apt/sources.list.d/extra-jammy.sources",
        source.replace(
            b"Suites: jazzy jammy jammy-updates noble focal",
            b"Suites: jammy")
            .replace(b"Components: main universe restricted", b"Components: multiverse"))
    (container / "etc/ros/rosdep/sources.list.d/20-default.list").unlink()

    lists = container / "var/lib/apt/lists"
    packages = (lists / packages_name).read_bytes()
    package_sha = capture.sha256_bytes(packages)
    repository_url = "https://mirror.example.invalid/ubuntu"
    components_by_suite = {
        "jazzy": ("main", "universe", "restricted", "multiverse"),
        "jammy": ("main", "universe", "restricted", "multiverse"),
        "jammy-updates": ("main", "universe", "restricted"),
        "noble": ("main", "universe", "restricted"),
        "focal": ("main", "universe", "restricted"),
    }
    for suite, components in components_by_suite.items():
        prefix = capture._apt_list_prefix(repository_url, suite)
        entries = []
        for component in components:
            package_file = prefix + "_{}_binary-amd64_Packages".format(component)
            _write_fixture(lists / package_file, packages)
            entries.append(" {} {} {}/binary-amd64/Packages".format(
                package_sha, len(packages), component))
        release = ("Origin: synthetic\nSuite: {}\nSHA256:\n{}\n"
                   .format(suite, "\n".join(entries))).encode("ascii")
        _write_fixture(lists / (prefix + "_Release"), release)
        _write_fixture(lists / (prefix + "_Release.gpg"), b"detached-signature\n")

    runner, _ = _gpgv_runner(capture)
    result = capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)
    manifest = json.loads((output / "source-manifest.json").read_text())
    assert result["status"] == "CAPTURED_REVIEW_REQUIRED"
    assert len(result["release_evidence"]) == 5
    assert len(result["package_evidence"]) == 17
    assert {item["role"] for item in manifest["files"]} == {
        "apt_source", "apt_release", "apt_packages"}


def test_no_signature_projection_rejects_unbacked_signer_references(
        capture, tmp_path, monkeypatch):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)

    def fail_packages(*args, **kwargs):
        del args, kwargs
        raise capture.CaptureError(
            "PACKAGES_ARTIFACT_BINDING_INVALID", "synthetic package failure")

    monkeypatch.setattr(capture, "_select_packages_index", fail_packages)
    runner, _ = _gpgv_runner(capture)
    with pytest.raises(capture.CaptureError):
        capture._capture_in_container(
            "apt-source", output, "jazzy", container_root=container,
            command_runner=runner)
    path = output / "in-container-apt-source.json"
    original = json.loads(path.read_text(encoding="utf-8"))
    altered = json.loads(json.dumps(original))
    altered["signature_references"] = [{"repository_index": 0}]
    altered["signature_count"] = 1
    _rewrite_sealed_json(capture, path, altered)
    try:
        with pytest.raises(capture.CaptureError) as error:
            capture._validate_inner_directory_binding(
                path, original["directory_contract"])
        assert error.value.kind == "SIGNATURE_EVIDENCE_MISSING"
        assert error.value.release_evidence == original["release_evidence"]
        assert error.value.package_evidence == original.get("package_evidence", [])
        assert error.value.signature_references == []
        assert error.value.signature_count == 0
        expected_map, expected_counts = capture._reference_projection(
            original["release_evidence"], original.get("package_evidence", []), [], 0)
        assert error.value.reference_map == expected_map
        assert error.value.reference_counts == expected_counts
    finally:
        _rewrite_sealed_json(capture, path, original)


def test_late_partial_projection_is_identical_in_inner_and_outer_receipts(
        capture, tmp_path, monkeypatch):
    plan = _plan(capture, tmp_path, monkeypatch)
    outer_root = Path(plan["output_root"])
    fixture_root = tmp_path / "container-fixture"
    fixture_root.mkdir()
    container, _, _, _, _, _ = _signed_container_fixture(capture, fixture_root)
    inner_runner, _ = _gpgv_runner(capture)
    original_run_phase = capture._run_phase

    def run_phase(plan_value, root, phase, runner):
        if phase != "apt_source_snapshot":
            return original_run_phase(plan_value, root, phase, runner)
        capture._capture_in_container(
            "apt-source", root, plan_value["distro"],
            container_root=container, command_runner=inner_runner)
        return capture._phase_record(
            plan_value, root, phase,
            (2, b"apt source stopped after sealed evidence\n", b"", False, None))

    monkeypatch.setattr(capture, "_run_phase", run_phase)
    outer_runner, _ = _runner(capture, plan, outer_root)
    result = capture.capture_dependency_closure(
        Path(plan["repo_root"]), Path(plan["profile_path"]), plan["distro"],
        plan["dependency_leg"], outer_root, runner=outer_runner, clock=lambda: 30)
    assert result["status"] == "PARTIAL_FAILURE_REVIEW_REQUIRED"
    receipt = json.loads(
        (outer_root / capture.RECEIPT_NAME).read_text(encoding="utf-8"))
    inner = json.loads(
        (outer_root / "in-container-apt-source.json").read_text(encoding="utf-8"))
    assert receipt["release_evidence"] == inner["release_evidence"]
    assert receipt["package_evidence"] == inner["package_evidence"]
    assert receipt["signature_references"] == inner["signature_references"]
    assert receipt["signature_count"] == inner["signature_count"]
    assert receipt["reference_map"] == inner["reference_map"]
    assert receipt["reference_counts"] == inner["reference_counts"]
    assert capture.validate_receipt(outer_root, expected_plan=plan)[
        "benchmark_eligible"] is False


def test_no_signature_projection_rejects_null_counts_with_prefix(
        capture, tmp_path, monkeypatch):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)

    def fail_packages(*args, **kwargs):
        del args, kwargs
        raise capture.CaptureError(
            "PACKAGES_ARTIFACT_BINDING_INVALID", "synthetic package failure")

    monkeypatch.setattr(capture, "_select_packages_index", fail_packages)
    runner, _ = _gpgv_runner(capture)
    with pytest.raises(capture.CaptureError):
        capture._capture_in_container(
            "apt-source", output, "jazzy", container_root=container,
            command_runner=runner)
    path = output / "in-container-apt-source.json"
    original = json.loads(path.read_text(encoding="utf-8"))
    altered = json.loads(json.dumps(original))
    altered["reference_counts"] = None
    _rewrite_sealed_json(capture, path, altered)
    try:
        with pytest.raises(capture.CaptureError) as error:
            capture._validate_inner_directory_binding(
                path, original["directory_contract"])
        assert error.value.kind == "REFERENCE_COUNT_INVALID"
        expected_map, expected_counts = capture._reference_projection(
            original["release_evidence"], original.get("package_evidence", []), [], 0)
        assert error.value.reference_map == expected_map
        assert error.value.reference_counts == expected_counts
    finally:
        _rewrite_sealed_json(capture, path, original)


def test_release_only_partial_projection_rejects_tampered_map(
        capture, tmp_path, monkeypatch):
    container, output, _, _, _, _ = _signed_container_fixture(capture, tmp_path)

    def fail_packages(*args, **kwargs):
        del args, kwargs
        raise capture.CaptureError(
            "PACKAGES_ARTIFACT_BINDING_INVALID", "synthetic package failure")

    monkeypatch.setattr(capture, "_select_packages_index", fail_packages)
    runner, _ = _gpgv_runner(capture)
    with pytest.raises(capture.CaptureError):
        capture._capture_in_container(
            "apt-source", output, "jazzy", container_root=container,
            command_runner=runner)
    path = output / "in-container-apt-source.json"
    original = json.loads(path.read_text(encoding="utf-8"))
    altered = json.loads(json.dumps(original))
    altered["reference_map"]["release"]["0"] = "a" * 64
    _rewrite_sealed_json(capture, path, altered)
    try:
        with pytest.raises(capture.CaptureError) as error:
            capture._validate_inner_directory_binding(
                path, original["directory_contract"])
        assert error.value.kind in {
            "REFERENCE_PROJECTION_BINDING_INVALID", "REFERENCE_COUNT_INVALID",
        }
        expected_map, expected_counts = capture._reference_projection(
            original["release_evidence"], [], [], 0)
        assert error.value.release_evidence == original["release_evidence"]
        assert error.value.package_evidence == []
        assert error.value.signature_references == []
        assert error.value.signature_count == 0
        assert error.value.reference_map == expected_map
        assert error.value.reference_counts == expected_counts
    finally:
        _rewrite_sealed_json(capture, path, original)


def test_shared_release_is_verified_once_and_referenced_by_each_component(
        capture, tmp_path):
    container, output, release_name, packages_name, _, _ = (
        _signed_container_fixture(capture, tmp_path))
    source_path = container / "etc/apt/sources.list.d/ros2.sources"
    source_path.write_bytes(source_path.read_bytes().replace(
        b"Components: main", b"Components: main universe"))
    source_path.chmod(0o644)
    lists = container / "var/lib/apt/lists"
    packages = (lists / packages_name).read_bytes()
    universe_name = packages_name.replace("_main_", "_universe_")
    _write_fixture(lists / universe_name, packages)
    package_sha = capture.sha256_bytes(packages)
    release = ("Origin: synthetic\nSuite: jazzy\nSHA256:\n"
               " {} {} main/binary-amd64/Packages\n"
               " {} {} universe/binary-amd64/Packages\n").format(
                   package_sha, len(packages), package_sha, len(packages)).encode()
    _write_fixture(lists / release_name, release)
    runner, calls = _gpgv_runner(capture)

    result = capture._capture_in_container(
        "apt-source", output, "jazzy", container_root=container,
        command_runner=runner)

    signature_calls = [argv for argv in calls
                       if argv and argv[0] == "gpgv" and
                       argv != ["gpgv", "--version"]]
    assert len(signature_calls) == 1
    assert len(result["release_evidence"]) == 1
    assert result["signature_count"] == 1
    assert len(result["signature_references"]) == 2
    assert len(result["package_evidence"]) == 2
    assert len({item["artifact_identity"]["identity"]
                for item in result["package_evidence"]}) == 2
    assert len({item["output"]["path"] for item in result["package_evidence"]}) == 2
    assert len(set(result["reference_map"]["packages"].values())) == 2
    assert len({item["identity"] for item in result["signature_references"]}) == 1
    signature_doc = json.loads(
        (output / "signature-evidence.json").read_text(encoding="utf-8"))
    assert len(signature_doc["repositories"]) == 1
    assert len(signature_doc["signature_references"]) == 2
    assert capture._validate_signature_references(
        result["signature_references"], 1,
        [{"identity": result["signature_references"][0]["identity"]}],
        require_complete=True)


def test_signature_failure_cache_retains_first_result_and_isolation(capture):
    identity = "a" * 64
    evidence = {"identity": identity, "exit_status": 1,
                "stderr_sha256": "b" * 64}
    cache = {}
    first_error = capture.CaptureError(
        "GPGV_VERIFY_FAILED", "first attempt", signature_evidence=[evidence])
    capture._cache_signature_failure(cache, identity, first_error)
    first = capture._cached_signature(cache, identity, "cached failure")
    first["evidence"][0]["stderr_sha256"] = "c" * 64
    second = capture._cached_signature(cache, identity, "cached failure")
    assert second["status"] == "FAILED"
    assert second["failure_kind"] == "GPGV_VERIFY_FAILED"
    assert second["evidence"][0]["stderr_sha256"] == "b" * 64


def test_signature_success_cache_rejects_conflict_and_is_mutation_isolated(capture):
    identity = "d" * 64
    cache = {}
    capture._cache_signature_success(
        cache, identity, {"identity": identity, "exit_status": 0})
    returned = capture._cached_signature(cache, identity, "cached success")
    returned["evidence"]["exit_status"] = 1
    assert capture._cached_signature(
        cache, identity, "cached success")["evidence"]["exit_status"] == 0
    with pytest.raises(capture.CaptureError) as error:
        capture._cache_signature_success(
            cache, identity, {"identity": identity, "exit_status": 2})
    assert error.value.kind == "GPG_RELEASE_IDENTITY_CONFLICT"


def test_identity_prefix_collision_is_rejected_before_write(capture):
    prefix = "e" * 24
    first = prefix + "0" * 40
    second = prefix + "1" * 40
    registry = {}
    capture._claim_identity_artifact(registry, "logs/gpgv-{}.stdout".format(prefix), first)
    with pytest.raises(capture.CaptureError) as error:
        capture._claim_identity_artifact(
            registry, "logs/gpgv-{}.stdout".format(prefix), second)
    assert error.value.kind == "ARTIFACT_IDENTITY_PREFIX_COLLISION"


def _package_identity_fixture(capture, suite="jammy", component="main"):
    repository = {
        "url": "https://mirror.example.invalid/ubuntu",
        "suite": suite,
        "component": component,
    }
    release_data = ("Origin: synthetic\nSuite: {}\n".format(suite)).encode("ascii")
    package_relative = "{}/binary-amd64/Packages".format(component)
    raw_relative = package_relative + ".xz"
    packages_url = (
        "https://mirror.example.invalid/ubuntu/dists/{}/{}/binary-amd64/"
        "Packages.xz".format(suite, component))
    return capture._package_artifact_identity(
        repository, "inrelease", "a" * 64,
        "indices/InRelease-{}".format("b" * 24), release_data,
        package_relative, raw_relative, "xz", packages_url,
        "c" * 64, 123, "d" * 64, 456)


def test_package_identity_separates_suites_and_dedupes_exact_identity(capture):
    jammy = _package_identity_fixture(capture, "jammy")
    updates = _package_identity_fixture(capture, "jammy-updates")
    assert jammy["identity"] != updates["identity"]
    assert capture._package_artifact_name("Packages.xz", jammy["identity"]) != \
        capture._package_artifact_name("Packages.xz", updates["identity"])
    registry = {}
    path = "raw-indexes/Packages.xz-{}".format(jammy["identity"][:24])
    capture._claim_identity_artifact(registry, path, jammy["identity"])
    capture._claim_identity_artifact(registry, path, jammy["identity"])
    with pytest.raises(capture.CaptureError) as error:
        capture._claim_identity_artifact(registry, path, updates["identity"])
    assert error.value.kind == "ARTIFACT_IDENTITY_PREFIX_COLLISION"


def test_package_identity_uses_dedicated_decoded_size_bound(capture):
    identity = _package_identity_fixture(capture)
    identity["decoded_bytes"] = capture.MAX_CAPTURE_FILE_BYTES + 1
    identity["identity"] = capture.canonical_hash(identity, "identity")
    assert capture._validate_package_artifact_identity(
        identity, "package identity"
    )["decoded_bytes"] == capture.MAX_CAPTURE_FILE_BYTES + 1

    identity["decoded_bytes"] = capture.MAX_DECODED_PACKAGES_BYTES + 1
    identity["identity"] = capture.canonical_hash(identity, "identity")
    with pytest.raises(capture.CaptureError) as error:
        capture._validate_package_artifact_identity(identity, "package identity")
    assert error.value.kind == "PACKAGES_ARTIFACT_IDENTITY_INVALID"


@pytest.mark.parametrize("field", [
    "repository_url", "signed_plain_path", "format", "compressed_sha256",
    "compressed_bytes", "decoded_sha256", "decoded_bytes",
])
def test_package_identity_rejects_repository_path_and_hash_substitution(capture, field):
    identity = _package_identity_fixture(capture)
    altered = json.loads(json.dumps(identity))
    if field in {"compressed_bytes", "decoded_bytes"}:
        altered[field] += 1
    elif field.endswith("sha256"):
        altered[field] = "e" * 64
    elif field == "format":
        altered[field] = "plain"
    elif field == "repository_url":
        altered[field] = "https://other.example.invalid/ubuntu"
    else:
        altered[field] = "other/binary-amd64/Packages"
    with pytest.raises(capture.CaptureError) as error:
        capture._validate_package_artifact_identity(altered, "package identity")
    assert error.value.kind == "PACKAGES_ARTIFACT_IDENTITY_INVALID"


def test_reference_projection_rejects_null_counts_instead_of_empty_zero(capture):
    reference_map, counts = capture._reference_projection([], [], [], 0)
    counts["package_evidence"] = None
    with pytest.raises(capture.CaptureError) as error:
        capture._validate_reference_projection(
            reference_map, counts, [], [], [], 0)
    assert error.value.kind == "REFERENCE_COUNT_INVALID"


def test_release_artifact_identity_and_name_are_full_identity_bound(capture):
    first = "f" * 64
    second = "f" * 24 + "0" * 40
    registry = {}
    path = "apt-source/indices/InRelease-{}".format("f" * 24)
    capture._claim_identity_artifact(
        registry, path[len("apt-source/"):], first, "Release artifact")
    with pytest.raises(capture.CaptureError) as error:
        capture._claim_identity_artifact(
            registry, path[len("apt-source/"):], second, "Release artifact")
    assert error.value.kind == "ARTIFACT_IDENTITY_PREFIX_COLLISION"


def _campaign_test_bindings(capture, tmp_path, monkeypatch):
    """Install a synthetic profile boundary for campaign-only unit tests."""
    profile_path = tmp_path / "campaign-profile.json"
    profile_path.write_text("{}\n", encoding="utf-8")
    campaign_set = {
        "schema": "registration-plugin-campaign-set-v1",
        "schema_version": 1,
        "campaign_id": "synthetic-capture-campaign",
        "rows": [
            {"distro": distro, "dependency_leg": leg}
            for distro, leg in capture.ROWS
        ],
        "row_set_sha256": "1" * 64,
        "identity_sha256": "2" * 64,
    }
    profile = {"campaign_set": campaign_set}
    release = {
        "campaign_set": campaign_set,
        "dependency_capture_campaign_aggregation": (
            capture._campaign_aggregation_contract()
        ),
    }
    monkeypatch.setattr(
        capture.AUDIT, "_load_profile",
        lambda path: (profile, profile_path),
    )
    monkeypatch.setattr(
        capture.AUDIT, "_validate_release_matrix_profile",
        lambda value: release,
    )
    return profile_path, profile, release


def _campaign_partial_fixture(capture, tmp_path, monkeypatch,
                              root_name="capture-campaign"):
    profile_path, profile, release = _campaign_test_bindings(
        capture, tmp_path, monkeypatch)

    def fail_child(*args, **kwargs):
        del args, kwargs
        raise capture.CaptureError("PHASE_FAILED", "synthetic child failure")

    monkeypatch.setattr(capture, "capture_dependency_closure", fail_child)
    root = tmp_path / root_name
    result = capture.capture_campaign(
        tmp_path / "repo", profile_path, root, clock=lambda: 100)
    return root, profile_path, profile, release, result


def _campaign_schema_validator(filename):
    path = ROOT / "configs/slam_benchmark_profiles" / filename
    schema = json.loads(path.read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(schema)
    return Draft202012Validator(schema)


def test_capture_campaign_started_failure_is_sealed_in_partial_rows(
        capture, tmp_path, monkeypatch):
    root, profile_path, _, _, result = _campaign_partial_fixture(
        capture, tmp_path, monkeypatch)
    assert result["status"] == "PARTIAL_FAILURE_REVIEW_REQUIRED"
    assert result["outcome"] == "PARTIAL_FAILURE"
    assert result["rows"] == []
    assert result["partial_row_count"] == 1
    assert [state["status"] for state in result["row_states"]] == [
        "FAIL_CLOSED", "NOT_STARTED", "NOT_STARTED", "NOT_STARTED"
    ]
    receipt = json.loads(
        (root / capture.CAMPAIGN_RECEIPT_NAME).read_text(encoding="utf-8"))
    partial_path = root / "row-1-humble-absent" / (
        capture.CAMPAIGN_PARTIAL_RECEIPT_NAME)
    partial = json.loads(partial_path.read_text(encoding="utf-8"))
    assert not list(_campaign_schema_validator(
        "registration_plugin_dependency_capture_campaign_v1.schema.json"
    ).iter_errors(receipt))
    assert not list(_campaign_schema_validator(
        "registration_plugin_dependency_capture_child_partial_v1.schema.json"
    ).iter_errors(partial))
    reopened = capture.validate_campaign_receipt(
        root, expected_profile_path=profile_path)
    assert reopened["partial_row_count"] == 1
    assert reopened["rows"] == []


def test_capture_campaign_rejects_unreferenced_extra_root_entry(
        capture, tmp_path, monkeypatch):
    root, profile_path, _, _, _ = _campaign_partial_fixture(
        capture, tmp_path, monkeypatch)
    extra = root / "unreferenced-child"
    extra.mkdir()
    with pytest.raises(capture.CaptureError) as error:
        capture.validate_campaign_receipt(
            root, expected_profile_path=profile_path)
    assert error.value.kind == "CAMPAIGN_ROOT_EXTRA_ENTRY"


@pytest.mark.parametrize("mutation,expected_kind", [
    ("clear_partial", "CAMPAIGN_ROW_MEMBERSHIP_INVALID"),
    ("escape_path", "CAMPAIGN_PROJECTION_INVALID"),
])
def test_capture_campaign_rejects_partial_row_membership_tamper(
        capture, tmp_path, monkeypatch, mutation, expected_kind):
    root, profile_path, _, _, _ = _campaign_partial_fixture(
        capture, tmp_path, monkeypatch)
    path = root / capture.CAMPAIGN_RECEIPT_NAME
    value = json.loads(path.read_text(encoding="utf-8"))
    if mutation == "clear_partial":
        value["partial_rows"] = []
        value["partial_row_count"] = 0
    else:
        value["partial_rows"][0]["path_relative"] = "../escape.json"
    value["canonical_sha256"] = capture.canonical_hash(value)
    _rewrite_sealed_json(capture, path, value)
    with pytest.raises(capture.CaptureError) as error:
        capture.validate_campaign_receipt(
            root, expected_profile_path=profile_path)
    assert error.value.kind == expected_kind


@pytest.mark.parametrize("mutation,expected_kind", [
    ("file_hash", "CAMPAIGN_PROJECTION_HASH_INVALID"),
    ("status", "CAMPAIGN_PROJECTION_INVALID"),
    ("count", "CAMPAIGN_CARDINALITY_INVALID"),
    ("duplicate", "CAMPAIGN_PARTIAL_ROWS_ORDER_INVALID"),
    ("rows_double", "CAMPAIGN_PROJECTION_INVALID"),
    ("state_order", "CAMPAIGN_ROW_STATE_INVALID"),
])
def test_capture_campaign_rejects_partial_projection_conflicts(
        capture, tmp_path, monkeypatch, mutation, expected_kind):
    root, profile_path, _, _, _ = _campaign_partial_fixture(
        capture, tmp_path, monkeypatch)
    path = root / capture.CAMPAIGN_RECEIPT_NAME
    value = json.loads(path.read_text(encoding="utf-8"))
    if mutation == "file_hash":
        value["partial_rows"][0]["file_sha256"] = "0" * 64
    elif mutation == "status":
        value["partial_rows"][0]["status"] = "PASS_REVIEW_REQUIRED"
    elif mutation == "count":
        value["partial_row_count"] = 2
    elif mutation == "duplicate":
        value["partial_rows"].append(
            json.loads(json.dumps(value["partial_rows"][0])))
        value["partial_row_count"] = 2
    elif mutation == "rows_double":
        value["rows"].append(
            json.loads(json.dumps(value["partial_rows"][0])))
    else:
        value["row_states"][0], value["row_states"][1] = (
            value["row_states"][1], value["row_states"][0])
    value["canonical_sha256"] = capture.canonical_hash(value)
    _rewrite_sealed_json(capture, path, value)
    with pytest.raises(capture.CaptureError) as error:
        capture.validate_campaign_receipt(
            root, expected_profile_path=profile_path)
    assert error.value.kind == expected_kind


def test_capture_campaign_rejects_missing_or_symlink_partial_receipt(
        capture, tmp_path, monkeypatch):
    root, profile_path, _, _, _ = _campaign_partial_fixture(
        capture, tmp_path, monkeypatch, root_name="missing-campaign")
    partial = root / "row-1-humble-absent" / capture.CAMPAIGN_PARTIAL_RECEIPT_NAME
    sidecar = partial.with_name(partial.name + ".sha256")
    partial.unlink()
    sidecar.unlink()
    with pytest.raises(capture.CaptureError):
        capture.validate_campaign_receipt(root, expected_profile_path=profile_path)

    root, profile_path, _, _, _ = _campaign_partial_fixture(
        capture, tmp_path, monkeypatch, root_name="symlink-campaign")
    partial = root / "row-1-humble-absent" / capture.CAMPAIGN_PARTIAL_RECEIPT_NAME
    outside = tmp_path / "outside-partial.json"
    outside.write_text("{}\n", encoding="utf-8")
    partial.unlink()
    partial.symlink_to(outside)
    with pytest.raises(capture.CaptureError) as error:
        capture.validate_campaign_receipt(root, expected_profile_path=profile_path)
    assert error.value.kind == "CAMPAIGN_PROJECTION_PATH_INVALID"


def test_capture_campaign_rejects_started_child_without_partial_reference(
        capture, tmp_path, monkeypatch):
    root, profile_path, _, _, _ = _campaign_partial_fixture(
        capture, tmp_path, monkeypatch)
    started_child = root / "row-3-jazzy-absent"
    started_child.mkdir()
    (started_child / "unexpected.log").write_text("started\n", encoding="utf-8")
    with pytest.raises(capture.CaptureError) as error:
        capture.validate_campaign_receipt(root, expected_profile_path=profile_path)
    assert error.value.kind == "CAMPAIGN_STARTED_CHILD_UNREFERENCED"


def test_existing_composer_output_gets_fresh_exact_sidecar(capture, tmp_path):
    output = tmp_path / "build-resolver.json"
    output.write_bytes(b'{"status":"SEALED_REVIEW_REQUIRED"}\n')
    output.chmod(0o444)
    sealed = capture._seal_existing_sidecar(output, "build resolver")
    sidecar = output.with_name(output.name + ".sha256")
    assert sidecar.read_text(encoding="ascii").split() == [
        sealed["file"]["sha256"], output.name]
    assert sidecar.stat().st_mode & 0o777 == 0o444

    with pytest.raises(capture.CaptureError) as error:
        capture._seal_existing_sidecar(output, "build resolver")
    assert error.value.kind == "OUTPUT_COLLISION_OR_SIZE"
