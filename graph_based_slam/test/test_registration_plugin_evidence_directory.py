#!/usr/bin/env python3
"""Adversarial tests for host-owned registration-plugin evidence layouts."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
DIRECTORY_PATH = ROOT / "scripts/registration_plugin_evidence_directory.py"
CAPTURE_PATH = ROOT / "scripts/capture_registration_plugin_dependency_closure.py"
LEG_PATH = ROOT / "scripts/run_registration_plugin_release_leg.py"
MATRIX_PATH = ROOT / "scripts/run_registration_plugin_release_matrix.py"
PREFETCH_PATH = ROOT / "scripts/registration_plugin_dependency_prefetch.py"

if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def directories():
    return _load(DIRECTORY_PATH, "registration_plugin_evidence_directory_test")


@pytest.fixture(scope="module")
def capture():
    return _load(CAPTURE_PATH, "capture_registration_plugin_dependency_closure_directory_test")


@pytest.fixture(scope="module")
def prefetch():
    return _load(PREFETCH_PATH, "registration_plugin_dependency_prefetch_directory_test")


@pytest.fixture(scope="module")
def leg():
    return _load(LEG_PATH, "registration_plugin_release_leg_directory_test")


@pytest.fixture(scope="module")
def matrix():
    return _load(MATRIX_PATH, "registration_plugin_release_matrix_directory_test")


def _layout(directories, tmp_path, paths=("logs", "work", "work/src")):
    tmp_path.mkdir(parents=True, exist_ok=True)
    root = tmp_path / "root"
    root_identity = directories.create_fresh_root(root)
    contract = directories.create_layout(root, paths)
    return root, root_identity, contract


def test_fresh_layout_binds_exact_owner_mode_inode_and_links(directories, tmp_path):
    root, root_identity, contract = _layout(directories, tmp_path)
    assert root_identity["path"] == str(root)
    assert root_identity["uid"] == os.getuid()
    assert root_identity["gid"] == os.getgid()
    assert root_identity["mode"] == 0o700
    assert root_identity["nlink"] >= 2
    assert contract["schema"] == directories.SCHEMA

    after = directories.verify_snapshot(root, contract)
    assert all(item["mode"] == 0o700 for item in after)
    assert all(item["uid"] == os.getuid() and item["gid"] == os.getgid()
               for item in after)
    assert all(item["nlink"] >= 2 for item in after)
    finalized = directories.finalize(contract, after)
    assert finalized["status"] == "PASS"
    assert finalized["after"] == after


def test_root_collision_and_symlink_parent_are_rejected(directories, tmp_path):
    existing = tmp_path / "existing"
    existing.mkdir()
    with pytest.raises(directories.EvidenceDirectoryError) as collision:
        directories.create_fresh_root(existing)
    assert collision.value.kind == "ROOT_NOT_FRESH"

    real_parent = tmp_path / "real-parent"
    real_parent.mkdir()
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(real_parent, target_is_directory=True)
    with pytest.raises(directories.EvidenceDirectoryError) as linked:
        directories.create_fresh_root(linked_parent / "child")
    assert linked.value.kind == "DIRECTORY_OPEN_FAILED"


def test_layout_collision_and_unsafe_paths_are_rejected(directories, tmp_path):
    root, _, _ = _layout(directories, tmp_path, paths=("logs",))
    with pytest.raises(directories.EvidenceDirectoryError) as collision:
        directories.create_layout(root, ("logs",))
    assert collision.value.kind == "DIRECTORY_LAYOUT_NOT_FRESH"

    with pytest.raises(directories.EvidenceDirectoryError) as unsafe:
        directories.create_layout(root, ("../escape",))
    assert unsafe.value.kind == "PATH_INVALID"


def test_mode_owner_and_nlink_drift_is_rejected_without_repair(directories, tmp_path):
    root, _, contract = _layout(directories, tmp_path)
    logs = root / "logs"
    logs.chmod(0o755)
    with pytest.raises(directories.EvidenceDirectoryError) as mode:
        directories.verify_snapshot(root, contract)
    assert mode.value.kind == "DIRECTORY_MODE_DRIFT"
    assert logs.stat().st_mode & 0o777 == 0o755

    logs.chmod(0o700)
    with pytest.raises(directories.EvidenceDirectoryError) as owner:
        directories.verify_snapshot(root, contract, owner=(os.getuid() + 1, os.getgid()))
    assert owner.value.kind == "DIRECTORY_OWNER_DRIFT"

    (root / "work" / "new-child").mkdir()
    with pytest.raises(directories.EvidenceDirectoryError) as link_count:
        directories.verify_snapshot(root, contract)
    assert link_count.value.kind == "DIRECTORY_IDENTITY_DRIFT"


def test_named_safe_root_directory_addition_is_exactly_bound(directories, tmp_path):
    root, _, contract = _layout(directories, tmp_path)
    closure = root / "closure"
    closure.mkdir(mode=0o700)

    after = directories.verify_snapshot(
        root, contract, allowed_root_directory_additions=["closure"])
    finalized = directories.finalize(
        contract, after, allowed_root_directory_additions=["closure"])
    assert finalized["allowed_root_directory_additions"] == ["closure"]

    (root / "unexpected").mkdir(mode=0o700)
    with pytest.raises(directories.EvidenceDirectoryError) as unexpected:
        directories.verify_snapshot(
            root, contract, allowed_root_directory_additions=["closure"])
    assert unexpected.value.kind == "DIRECTORY_IDENTITY_DRIFT"


def test_root_directory_addition_rejects_symlink_mode_and_managed_name(
        directories, tmp_path):
    root, _, contract = _layout(directories, tmp_path)
    target = tmp_path / "target"
    target.mkdir(mode=0o700)
    (root / "closure").symlink_to(target, target_is_directory=True)
    with pytest.raises(directories.EvidenceDirectoryError) as linked:
        directories.verify_snapshot(
            root, contract, allowed_root_directory_additions=["closure"])
    assert linked.value.kind == "DIRECTORY_OPEN_FAILED"

    (root / "closure").unlink()
    (root / "closure").mkdir(mode=0o755)
    with pytest.raises(directories.EvidenceDirectoryError) as mode:
        directories.verify_snapshot(
            root, contract, allowed_root_directory_additions=["closure"])
    assert mode.value.kind == "DIRECTORY_MODE_DRIFT"

    with pytest.raises(directories.EvidenceDirectoryError) as managed:
        directories.verify_snapshot(
            root, contract, allowed_root_directory_additions=["logs"])
    assert managed.value.kind == "DIRECTORY_CONTRACT_INVALID"


def test_replaced_directory_symlink_and_inode_are_rejected(directories, tmp_path):
    root, _, contract = _layout(directories, tmp_path)
    logs = root / "logs"
    moved = root / "logs-original"
    logs.rename(moved)
    logs.symlink_to(moved, target_is_directory=True)
    with pytest.raises(directories.EvidenceDirectoryError):
        directories.verify_snapshot(root, contract)

    root2, _, contract2 = _layout(directories, tmp_path / "second")
    work = root2 / "work"
    work.rename(root2 / "work-original")
    work.mkdir(mode=0o700)
    with pytest.raises(directories.EvidenceDirectoryError) as inode:
        directories.verify_snapshot(root2, contract2)
    assert inode.value.kind == "DIRECTORY_IDENTITY_DRIFT"


def test_partial_layout_failure_does_not_report_pass(directories, tmp_path, monkeypatch):
    root = tmp_path / "partial"
    directories.create_fresh_root(root)
    original = directories._mkdir_fresh
    calls = {"count": 0}

    def fail_after_one(path, relative, expected_mode=directories.DIRECTORY_MODE,
                       expected_owner=None):
        calls["count"] += 1
        if calls["count"] == 2:
            raise directories.EvidenceDirectoryError("INJECTED_FAILURE", relative)
        return original(path, relative, expected_mode, expected_owner)

    monkeypatch.setattr(directories, "_mkdir_fresh", fail_after_one)
    with pytest.raises(directories.EvidenceDirectoryError) as failure:
        directories.create_layout(root, ("a", "b"))
    assert failure.value.kind == "INJECTED_FAILURE"
    assert (root / "a").is_dir()
    assert not (root / "a").is_symlink()
    assert (root / "a").stat().st_mode & 0o777 == 0o700


def test_capture_layout_has_no_dynamic_directory_creation(capture, tmp_path):
    root = tmp_path / "output"
    capture.EVIDENCE_DIRECTORIES.create_fresh_root(root)
    with pytest.raises(capture.CaptureError) as error:
        capture._capture_in_container("apt-source", root, "jazzy")
    assert error.value.kind in {
        "DIRECTORY_OPEN_FAILED", "DIRECTORY_LAYOUT_INVALID", "DIRECTORY_OWNER_DRIFT",
    }
    assert list(root.iterdir()) == []


def test_capture_layout_contract_rejects_posthoc_mode_change(capture, tmp_path):
    root = tmp_path / "output"
    capture.EVIDENCE_DIRECTORIES.create_fresh_root(root)
    contract = capture._create_capture_directory_layout(root, include_prefetch=False)
    assert contract["paths"] == sorted(capture.CAPTURE_DIRECTORY_RELATIVES)
    (root / "logs").chmod(0o755)
    with pytest.raises(capture.CaptureError) as error:
        capture._verify_capture_directory_layout(root, contract)
    assert error.value.kind == "DIRECTORY_MODE_DRIFT"
    assert (root / "logs").stat().st_mode & 0o777 == 0o755


def test_capture_layout_records_only_terminal_closure(capture, tmp_path):
    root = tmp_path / "output"
    capture.EVIDENCE_DIRECTORIES.create_fresh_root(root)
    contract = capture._create_capture_directory_layout(root, include_prefetch=False)
    (root / "closure").mkdir(mode=0o700)

    finalized = capture._verify_capture_directory_layout(
        root, contract, allow_terminal_closure=True)
    assert finalized["allowed_root_directory_additions"] == ["closure"]
    reopened = capture._verify_capture_directory_layout(root, finalized)
    assert reopened == finalized

    (root / "other").mkdir(mode=0o700)
    with pytest.raises(capture.CaptureError) as unexpected:
        capture._verify_capture_directory_layout(root, finalized)
    assert unexpected.value.kind == "DIRECTORY_IDENTITY_DRIFT"


def test_prefetch_host_contract_rejects_missing_precreated_directories(prefetch, tmp_path):
    root = tmp_path / "prefetch"
    root.mkdir(mode=0o700)
    with pytest.raises(prefetch.PrefetchError) as error:
        prefetch._check_root_shape(
            root, [], allowed_host_directories={"prefetch", "prefetch/archives"},
            include_prefetch_dirs=True)
    assert error.value.kind == "PREFETCH_ROOT_ALLOWLIST_MISMATCH"


def test_release_leg_creates_and_reopens_host_layout_without_repair(leg, directories, tmp_path):
    root = tmp_path / "leg-root"
    directories.create_fresh_root(root)
    plan = {
        "evidence_child": str(root / "row"),
        "leg": {"dependencies": [{"name": "fast_gicp"}]},
    }
    contract = leg._host_directory_layout(plan, root)
    expected = directories.launcher_paths("row", ["fast_gicp"], include_prefetch=True)
    assert contract["paths"] == sorted(expected)
    assert (root / "row" / "prefetch" / "archives").is_dir()
    finalized = leg._verify_host_directory_layout(root, contract)
    assert finalized["status"] == "PASS"
    assert finalized["after"] == directories.verify_snapshot(root, contract)

    work = root / "row" / "work"
    work.chmod(0o755)
    with pytest.raises(leg.LauncherError) as error:
        leg._verify_host_directory_layout(root, contract)
    assert error.value.kind == "DIRECTORY_MODE_DRIFT"
    assert work.stat().st_mode & 0o777 == 0o755


def test_release_leg_rejects_preexisting_root_and_replacement(leg, directories, tmp_path):
    existing = tmp_path / "existing-leg"
    existing.mkdir()
    with pytest.raises(leg.LauncherError) as error:
        leg._reserve_root(existing)
    assert error.value.kind == "ROOT_NOT_FRESH"

    root = tmp_path / "replace-leg"
    directories.create_fresh_root(root)
    plan = {"evidence_child": str(root / "row"), "leg": {"dependencies": []}}
    contract = leg._host_directory_layout(plan, root)
    row = root / "row"
    moved = root / "row-original"
    row.rename(moved)
    row.symlink_to(moved, target_is_directory=True)
    with pytest.raises(leg.LauncherError):
        leg._verify_host_directory_layout(root, contract)
    assert row.is_symlink()


def test_release_matrix_uses_exact_host_layout_and_rejects_link_drift(
        matrix, directories, tmp_path):
    root = tmp_path / "matrix-root"
    directories.create_fresh_root(root)
    dependencies = [{"name": "fast_gicp"}]
    paths = directories.release_runner_paths(["fast_gicp"], include_prefetch=True)
    directories.create_layout(root, paths)
    contract = matrix._runner_directory_contract(root, dependencies, include_prefetch=True)
    assert contract["paths"] == sorted(paths)
    finalized = matrix._verify_runner_directory_contract(root, contract)
    assert finalized["status"] == "PASS"

    work = root / "work"
    (work / "unexpected").mkdir(mode=0o700)
    with pytest.raises(matrix.ReleaseGateError) as error:
        matrix._verify_runner_directory_contract(root, contract)
    assert error.value.kind == "DIRECTORY_IDENTITY_DRIFT"
    assert (work / "unexpected").is_dir()


def test_prefetch_accepts_only_helper_precreated_dirs(prefetch, directories, tmp_path):
    root = tmp_path / "prefetch-root"
    directories.create_fresh_root(root)
    directories.create_layout(root, ("prefetch", "prefetch/archives"))
    payload = b"synthetic archive\n"
    dependency = {
        "name": "fast_gicp",
        "official_url": "https://example.invalid/fast_gicp.git",
        "commit": "a" * 40,
        "archive_url": "https://example.invalid/fast_gicp.tar.gz",
        "archive_sha256": prefetch._sha256_bytes(payload),
        "archive_top_level": "fast_gicp-main",
        "source_tree_sha256": "b" * 64,
        "license_path": "LICENSE",
        "license_sha256": "c" * 64,
    }
    prefetch._write_sealed(root / prefetch.DEPENDENCY_ENVIRONMENT_NAME,
                           b'{"schema":"synthetic"}\n')

    def fetch(url, maximum):
        assert url == dependency["archive_url"]
        assert maximum == prefetch.MAX_ARCHIVE_BYTES
        return {"payload": payload, "final_url": url, "redirects": [], "status": 200,
                "headers": {}}

    result = prefetch.prefetch_pinned_archives(
        [dependency], root, "jazzy", "sha256:" + "d" * 64,
        fetcher=fetch, allowed_host_directories={"prefetch", "prefetch/archives"})
    assert result["status"] == "PASS"
    assert (root / "prefetch" / "archives" / "fast_gicp.tar.gz").is_file()
    assert (root / "prefetch" / "archives" / "fast_gicp.tar.gz").stat().st_mode & 0o777 == 0o444
