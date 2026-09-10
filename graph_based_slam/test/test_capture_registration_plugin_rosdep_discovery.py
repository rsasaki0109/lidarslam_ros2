#!/usr/bin/env python3
"""Focused tests for the review-only rosdep discovery collector."""

from __future__ import annotations

import importlib.util
import io
import os
from pathlib import Path
import tarfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "scripts/capture_registration_plugin_rosdep_discovery.py"


def _load():
    spec = importlib.util.spec_from_file_location(
        "capture_registration_plugin_rosdep_discovery_test", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_seal_reopen_is_immutable_and_sidecar_bound(tmp_path):
    module = _load()
    path = tmp_path / "artifacts" / "evidence.txt"
    path.parent.mkdir()
    descriptor = module.seal_bytes(path, b"discovery\n")
    assert path.stat().st_mode & 0o777 == 0o444
    assert path.stat().st_nlink == 1
    assert Path(descriptor["sidecar"]).read_text(encoding="ascii").split() == [
        descriptor["sha256"], path.name,
    ]
    assert module.reopen_artifact(path, descriptor) == descriptor
    with pytest.raises(module.DiscoveryError, match="fresh"):
        module.seal_bytes(path, b"changed\n")


def test_github_policy_rejects_ambiguous_urls():
    module = _load()
    commit = "a" * 40
    assert module.validate_archive_url(
        "https://codeload.github.com/ros/rosdistro/tar.gz/" + commit)
    with pytest.raises(module.DiscoveryError):
        module.validate_archive_url(
            "https://codeload.github.com.evil.invalid/ros/rosdistro/tar.gz/" + commit)
    with pytest.raises(module.DiscoveryError):
        module.validate_archive_url(
            "https://codeload.github.com/ros/rosdistro/tar.gz/" + commit + "?x=1")


def test_package_records_require_exact_hash_size_and_uri():
    module = _load()
    output = (
        "Package: python3-rosdep\nVersion: 0.26.0-1\nArchitecture: all\n"
        "Filename: pool/main/p/python3-rosdep/python3-rosdep_0.26.0-1_all.deb\n"
        "Size: 3320\nSHA256: " + "a" * 64 + "\n\n"
    ).encode("ascii")
    records = module.parse_apt_package_records(output, "http://packages.example.invalid")
    assert records == [{
        "name": "python3-rosdep",
        "version": "0.26.0-1",
        "architecture": "all",
        "filename": "pool/main/p/python3-rosdep/python3-rosdep_0.26.0-1_all.deb",
        "size_bytes": 3320,
        "sha256": "a" * 64,
        "uri": "http://packages.example.invalid/pool/main/p/python3-rosdep/"
        "python3-rosdep_0.26.0-1_all.deb",
    }]
    with pytest.raises(module.DiscoveryError):
        module.parse_apt_package_records(
            output.replace(b"Size: 3320", b"Size: -1"),
            "http://packages.example.invalid")


def _package_item(module, name="python3-rosdistro-modules", version="1.0.1-1"):
    payload = (name + "=" + version + "\n").encode("ascii")
    filename = "pool/main/p/" + name + "/" + name + "_" + version + "_all.deb"
    return {
        "name": name,
        "version": version,
        "architecture": "all",
        "filename": filename,
        "uri": "http://packages.ros.org/ros2/ubuntu/" + filename,
        "size_bytes": len(payload),
        "sha256": module.sha256_bytes(payload),
    }, payload


def test_exact_specs_include_preinstalled_version_and_reinstall_contract():
    module = _load()
    rows = [
        {"name": name, "version": "1.0.1-1" if name.endswith("modules") else "0.26.0-1"}
        for name in module.ALLOWED_PACKAGE_NAMES
    ]
    specs = module._exact_package_specs(rows)
    assert "python3-rosdistro-modules=1.0.1-1" in specs
    assert specs == sorted(specs)


def test_uri_set_requires_exact_version_filename_and_no_extras():
    module = _load()
    item, _ = _package_item(module)
    uri = {item["filename"].rsplit("/", 1)[-1]: item["uri"]}
    assert module._validate_uri_set([item], uri) == uri
    with pytest.raises(module.DiscoveryError, match="missing="):
        module._validate_uri_set([item], {})
    with pytest.raises(module.DiscoveryError, match="extra="):
        module._validate_uri_set([item], {**uri, "extra.deb": item["uri"]})


def test_downloaded_deb_is_bound_to_signed_record_and_collision_is_rejected(tmp_path):
    module = _load()
    root = tmp_path / "root"
    root.mkdir()
    item, payload = _package_item(module)
    filename = item["filename"].rsplit("/", 1)[-1]
    descriptors = module._bind_downloaded_debs(root, [item], {filename: payload})
    assert filename in descriptors
    module.reopen_artifact(root / "artifacts/apt/debs" / filename, descriptors[filename])
    with pytest.raises(module.DiscoveryError, match="fresh artifact"):
        module._bind_downloaded_debs(root, [item], {filename: payload})


def test_downloaded_deb_rejects_missing_extra_and_wrong_bytes(tmp_path):
    module = _load()
    root = tmp_path / "root"
    root.mkdir()
    item, payload = _package_item(module)
    filename = item["filename"].rsplit("/", 1)[-1]
    with pytest.raises(module.DiscoveryError, match="missing="):
        module._bind_downloaded_debs(root, [item], {})
    with pytest.raises(module.DiscoveryError, match="extra="):
        module._bind_downloaded_debs(root, [item], {filename: payload, "extra.deb": payload})
    with pytest.raises(module.DiscoveryError):
        module._bind_downloaded_debs(root, [item], {filename: payload + b"tamper"})


def test_cache_inventory_rejects_nonregular_or_duplicate_entries():
    module = _load()
    inventory = (
        b"/tmp/cache/partial\tdirectory\t700\t0\t0\t2\t4096\t1\t2\n"
        b"/tmp/cache/pkg.deb\tregular file\t644\t0\t0\t1\t3\t1\t3\n"
        b"/tmp/cache/lock\tregular empty file\t640\t0\t0\t1\t0\t1\t4\n"
    )
    parsed = module._parse_cache_inventory(inventory)
    assert parsed["partial"]["type"] == "directory"
    assert parsed["partial"]["mode"] == 0o700
    assert parsed["lock"]["mode"] == 0o640
    assert parsed["pkg.deb"]["nlink"] == 1
    with pytest.raises(module.DiscoveryError):
        module._parse_cache_inventory(
            b"/tmp/cache/pkg.deb\tsymbolic link\t777\t0\t0\t1\t3\t1\t3\n"
        )
    with pytest.raises(module.DiscoveryError):
        module._parse_cache_inventory(
            b"/tmp/cache/pkg.deb\\tregular file\\t644\\t0\\t0\\t1\\t3\\t1\\t3\n"
        )
    with pytest.raises(module.DiscoveryError):
        module._parse_cache_inventory(
            b"/tmp/cache/pkg.deb\tregular file\t448\t0\t0\t1\t3\t1\t3\n"
        )
    with pytest.raises(module.DiscoveryError):
        module._parse_cache_inventory(
            b"/tmp/cache/pkg.deb\tregular file\t0644\t0\t0\t1\t3\t1\t3\n"
        )
    with pytest.raises(module.DiscoveryError):
        module._parse_cache_inventory(inventory + inventory.splitlines()[1] + b"\n")


def test_cache_metadata_allows_only_empty_lock_and_partial(tmp_path):
    module = _load()
    item, _ = _package_item(module)
    filename = item["filename"].rsplit("/", 1)[-1]
    inventory = {
        "lock": {
            "name": "lock", "path": "/tmp/cache/lock",
            "type": "regular empty file", "mode": 0o640,
            "uid": 0, "gid": 0, "nlink": 1, "bytes": 0,
            "device": 1, "inode": 4,
        },
        "partial": {
            "name": "partial", "path": "/tmp/cache/partial",
            "type": "directory", "mode": 0o700,
            "uid": 100, "gid": 0, "nlink": 2, "bytes": 4096,
            "device": 1, "inode": 5,
        },
        filename: {
            "name": filename, "path": "/tmp/cache/" + filename,
            "type": "regular file", "mode": 0o644,
            "uid": 0, "gid": 0, "nlink": 1, "bytes": item["size_bytes"],
            "device": 1, "inode": 6,
        },
    }
    result = module._validate_cache_metadata(
        inventory, [item], module.sha256_bytes(b"")
    )
    assert result["lock"]["bytes"] == 0
    assert module._validate_cache_package_set(inventory, [item]) == (
        {filename}, {"lock", "partial"}
    )
    bad_lock = {name: dict(value) for name, value in inventory.items()}
    bad_lock["lock"]["mode"] = 0o644
    with pytest.raises(module.DiscoveryError):
        module._validate_cache_metadata(
            bad_lock, [item], module.sha256_bytes(b"")
        )
    bad_extra = dict(inventory)
    bad_extra["unexpected"] = dict(inventory[filename])
    with pytest.raises(module.DiscoveryError):
        module._validate_cache_metadata(
            bad_extra, [item], module.sha256_bytes(b"")
        )
    bad_partial = {name: dict(value) for name, value in inventory.items()}
    bad_partial["partial"]["type"] = "symbolic link"
    with pytest.raises(module.DiscoveryError):
        module._validate_cache_metadata(
            bad_partial, [item], module.sha256_bytes(b"")
        )


def test_cache_lifecycle_allows_partial_cleanup_but_rejects_identity_drift():
    module = _load()
    before = {
        "path": "/tmp/cache",
        "type": "directory",
        "mode": 0o700,
        "uid": 0,
        "gid": 0,
        "nlink": 3,
        "bytes": 4096,
        "device": 7,
        "inode": 8,
    }
    after = dict(before, nlink=2)
    assert module._validate_cache_lifecycle(
        before, after, {"lock", "partial"}, {}
    ) is None
    with pytest.raises(module.DiscoveryError, match="/tmp/cache"):
        module._validate_cache_lifecycle(
            dict(before, inode=9), after, {"lock", "partial"}, {}
        )
    with pytest.raises(module.DiscoveryError, match="/tmp/cache"):
        module._validate_cache_lifecycle(
            before, dict(after, device=8), {"lock", "partial"}, {}
        )
    with pytest.raises(module.DiscoveryError, match="/tmp/cache"):
        module._validate_cache_lifecycle(
            dict(before, nlink=4), after, {"lock", "partial"}, {}
        )
    with pytest.raises(module.DiscoveryError, match="/tmp/cache"):
        module._validate_cache_lifecycle(
            before, dict(after, nlink=3), {"lock", "partial"}, {}
        )
    with pytest.raises(module.DiscoveryError, match="/tmp/cache"):
        module._validate_cache_lifecycle(
            before, after, {"lock", "partial"}, {"unexpected": {}}
        )
    no_partial_before = dict(before, nlink=2)
    no_partial_after = dict(after, nlink=2)
    assert module._validate_cache_lifecycle(
        no_partial_before, no_partial_after, {"lock"}, {}
    ) is None
    with pytest.raises(module.DiscoveryError, match="/tmp/cache"):
        module._validate_cache_lifecycle(
            before, after, {"lock"}, {}
        )
    with pytest.raises(module.DiscoveryError, match="/tmp/cache"):
        module._validate_cache_lifecycle(
            no_partial_before, no_partial_after, {"lock", "partial"}, {}
        )
    with pytest.raises(module.DiscoveryError, match="/tmp/cache"):
        module._validate_cache_lifecycle(
            no_partial_before, no_partial_after, {"partial"}, {}
        )
    with pytest.raises(module.DiscoveryError, match="/tmp/cache"):
        module._validate_cache_lifecycle(
            before, after, {"lock", "partial"}, {"pkg.deb": {}}
        )


def _source_removal_fixture(module):
    item, _ = _package_item(module)
    filename = item["filename"].rsplit("/", 1)[-1]
    baseline = {
        "path": "/tmp/cache/" + filename,
        "type": "regular file",
        "mode": 0o644,
        "uid": 0,
        "gid": 0,
        "nlink": 1,
        "bytes": item["size_bytes"],
        "device": 1,
        "inode": 6,
    }
    evidence = {
        **baseline,
        "name": filename,
        "sha256": item["sha256"],
        "fsynced": True,
        "post_absent": True,
        "removed": True,
    }
    return item, filename, baseline, evidence


def test_source_removal_evidence_binds_hash_inode_and_link_count():
    module = _load()
    item, _, baseline, evidence = _source_removal_fixture(module)
    assert module._validate_source_removal_evidence(
        evidence, "/tmp/cache", item, baseline
    ) == evidence
    with pytest.raises(module.DiscoveryError):
        module._validate_source_removal_evidence(
            {**evidence, "sha256": "b" * 64},
            "/tmp/cache", item, baseline,
        )
    with pytest.raises(module.DiscoveryError):
        module._validate_source_removal_evidence(
            {**evidence, "inode": 99},
            "/tmp/cache", item, baseline,
        )
    with pytest.raises(module.DiscoveryError):
        module._validate_source_removal_evidence(
            {**evidence, "nlink": 2},
            "/tmp/cache", item, baseline,
        )
    with pytest.raises(module.DiscoveryError):
        module._validate_source_removal_evidence(
            {**evidence, "unexpected": True},
            "/tmp/cache", item, baseline,
        )


def test_source_removal_command_preserves_partial_and_exact_cache_name(
    monkeypatch, tmp_path
):
    module = _load()
    item, filename, baseline, evidence = _source_removal_fixture(module)

    class RecordingCommand:
        root = tmp_path

        def __init__(self, returncode=0):
            self.returncode = returncode
            self.calls = []

        def run(self, argv, label, *, timeout):
            self.calls.append((list(argv), label, timeout))
            return {
                "returncode": self.returncode,
                "timed_out": False,
                "oversized": False,
                "stdout": {"path": "source.stdout"},
                "stderr": {"path": "source.stderr"},
                "record": {"path": "source.json"},
            }

    monkeypatch.setattr(
        module,
        "_read_record_stream",
        lambda root, record, stream: (
            module.canonical_bytes(evidence) if stream == "stdout" else b""
        ),
    )
    command = RecordingCommand()
    partial_state = {"cache_details": {"source_removals": []}}
    result = module._remove_verified_cache_deb(
        tmp_path,
        command,
        "owned-container",
        "/tmp/cache",
        item,
        baseline,
        index=0,
        partial_state=partial_state,
    )
    assert result["filename"] == filename
    assert result["source"] == evidence
    assert command.calls[0][0][:4] == [
        "docker", "exec", "owned-container", "python3",
    ]
    assert command.calls[0][0][-4:] == [
        "/tmp/cache", filename, str(item["size_bytes"]), item["sha256"],
    ]
    assert partial_state["cache_details"]["source_removals"][0]["status"] == (
        "REMOVED"
    )

    failed = RecordingCommand(returncode=1)
    failed_state = {"cache_details": {"source_removals": []}}
    with pytest.raises(module.DiscoveryError, match="apt_deb_source_remove"):
        module._remove_verified_cache_deb(
            tmp_path,
            failed,
            "owned-container",
            "/tmp/cache",
            item,
            baseline,
            index=0,
            partial_state=failed_state,
        )
    assert failed_state["cache_details"]["source_removals"][0]["status"] == (
        "FAILED"
    )


def test_late_cache_failure_partial_receipt_keeps_deb_descriptors(tmp_path):
    module = _load()
    root = tmp_path / "registration-plugin-rosdep-discovery-20260827-07"
    artifact = root / "artifacts" / "apt" / "debs" / "pkg.deb"
    artifact.parent.mkdir(parents=True)
    descriptor = module.seal_bytes(artifact, b"deb")
    receipt = {"apt": {"status": "NOT_CAPTURED", "packages": []}}
    module._attach_partial_deb_evidence(
        receipt, root, {"pkg.deb": descriptor},
        {"path": "/tmp/cache", "identity_before": {"nlink": 3}},
    )
    assert receipt["apt"]["status"] == "PARTIAL_REVIEW_REQUIRED"
    assert receipt["apt"]["partial_cache"]["path"] == "/tmp/cache"
    assert receipt["apt"]["partial_deb_artifacts"] == [{
        **descriptor,
        "path": "artifacts/apt/debs/pkg.deb",
        "sidecar": "artifacts/apt/debs/pkg.deb.sha256",
    }]
    module.reopen_artifact(artifact, descriptor)


def test_cleanup_container_uses_exact_name_after_artifact_projection(
    monkeypatch, tmp_path
):
    module = _load()

    class RecordingCommand:
        root = tmp_path

        def __init__(self):
            self.calls = []

        def run(self, argv, label, *, timeout):
            self.calls.append((list(argv), label, timeout))
            return {
                "returncode": 0,
                "timed_out": False,
                "oversized": False,
                "record": {"path": label},
            }

    monkeypatch.setattr(
        module, "_read_record_stream", lambda root, record, stream: b""
    )
    command = RecordingCommand()
    container_name = "registration-plugin-discovery-owned"
    for artifact_name in ("first.deb", "second.deb"):
        assert artifact_name
    result = module._cleanup_container(command, container_name)
    assert result["status"] == "PASS"
    assert result["post_remove_absent"] is True
    assert command.calls[0][0] == ["docker", "rm", "-f", container_name]
    assert command.calls[1][0] == [
        "docker", "ps", "-a", "--filter",
        "name=^{}$".format(container_name), "--format", "{{.Names}}",
    ]


def test_cleanup_container_stderr_is_a_failed_cleanup_proof(monkeypatch, tmp_path):
    module = _load()

    class RecordingCommand:
        root = tmp_path

        def __init__(self):
            self.calls = []

        def run(self, argv, label, *, timeout):
            self.calls.append((list(argv), label, timeout))
            return {
                "returncode": 0,
                "timed_out": False,
                "oversized": False,
                "record": {"path": label},
            }

    def output(root, record, stream):
        del root, record
        return b"unexpected" if stream == "stderr" else b""

    monkeypatch.setattr(module, "_read_record_stream", output)
    command = RecordingCommand()
    container_name = "registration-plugin-discovery-owned"
    result = module._cleanup_container(command, container_name)
    assert result["status"] == "FAIL_CLOSED"
    assert command.calls[0][0][3] == container_name
    assert command.calls[1][0][4] == "name=^{}$".format(container_name)


def test_discovery_id_is_bound_to_root_basename():
    module = _load()
    root = Path(
        "/tmp/registration-plugin-rosdep-discovery-20260827-06"
    )
    assert module._discovery_id_for_root(root) == root.name
    with pytest.raises(module.DiscoveryError) as exc_info:
        module._discovery_id_for_root(
            Path("/tmp/registration-plugin-rosdep-discovery-20260827-00")
        )
    assert exc_info.value.kind == "DISCOVERY_ID_INVALID"
    with pytest.raises(module.DiscoveryError) as exc_info:
        module._discovery_id_for_root(
            Path("/tmp/registration-plugin-rosdep-discovery-20261327-06")
        )
    assert exc_info.value.kind == "DISCOVERY_ID_INVALID"
    with pytest.raises(module.DiscoveryError) as exc_info:
        module._discovery_id_for_root(Path("/tmp/discovery-06"))
    assert exc_info.value.kind == "DISCOVERY_ID_INVALID"


def test_archive_selection_rejects_links_and_traversal(tmp_path):
    module = _load()
    archive = tmp_path / "fixture.tar.gz"
    with tarfile.open(archive, "w:gz") as stream:
        data = b"yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml\n"
        info = tarfile.TarInfo("rosdistro-a/rosdep/sources.list.d/20-default.list")
        info.size = len(data)
        stream.addfile(info, io.BytesIO(data))
        bad = tarfile.TarInfo("rosdistro-a/../escape.yaml")
        bad.size = 1
        stream.addfile(bad, io.BytesIO(b"x"))
    with tarfile.open(archive, "r:gz") as stream:
        members = list(stream)
    with pytest.raises(module.DiscoveryError, match="escape"):
        for member in members:
            module._archive_member_safe(member)


def test_rosdistro_output_layout_is_fresh_complete_and_bound(tmp_path):
    module = _load()
    root = tmp_path / "root"
    root.mkdir()
    layout = module._create_rosdistro_output_layout(root)
    assert layout["status"] == "PRECREATED"
    assert layout["allowed_directories"] == list(
        module.ROSDISTRO_OUTPUT_DIRECTORIES
    )
    assert {
        "artifacts/apt", "artifacts/apt/debs",
        "artifacts/rosdistro/files/releases",
        "artifacts/rosdistro/files/rosdep",
    } <= set(layout["directories"])
    finalized = module._finalize_rosdistro_output_layout(root, layout)
    assert finalized["status"] == "VERIFIED"
    assert set(finalized["directories"]) == set(
        module.ROSDISTRO_OUTPUT_DIRECTORIES
    )
    module._validate_output_layout_receipt(root, finalized)


def test_rosdistro_output_layout_rejects_collision_missing_replacement_and_escape(
    tmp_path,
):
    module = _load()
    collision_root = tmp_path / "collision"
    collision_root.mkdir()
    (collision_root / "artifacts").mkdir()
    with pytest.raises(module.DiscoveryError):
        module._create_rosdistro_output_layout(collision_root)

    root = tmp_path / "root"
    root.mkdir()
    layout = module._create_rosdistro_output_layout(root)
    releases = root / "artifacts/rosdistro/files/releases"
    releases.rmdir()
    with pytest.raises(module.DiscoveryError):
        module._require_rosdistro_output_directory(
            root, "artifacts/rosdistro/files/releases", layout
        )
    with pytest.raises(module.DiscoveryError):
        module._require_rosdistro_output_directory(
            root, "artifacts/rosdistro/files/../escape", layout
        )

    outside = tmp_path / "outside"
    outside.mkdir()
    releases.symlink_to(outside, target_is_directory=True)
    with pytest.raises(module.DiscoveryError):
        module._finalize_rosdistro_output_layout(root, layout)

    releases.unlink()
    releases.mkdir()
    unexpected = root / "artifacts/rosdistro/files/unexpected"
    unexpected.mkdir()
    with pytest.raises(module.DiscoveryError, match="unexpected"):
        module._finalize_rosdistro_output_layout(root, layout)

    root = tmp_path / "apt-unknown"
    root.mkdir()
    layout = module._create_rosdistro_output_layout(root)
    unknown = root / "artifacts/apt/unknown"
    unknown.mkdir()
    with pytest.raises(module.DiscoveryError, match="unknown"):
        module._finalize_rosdistro_output_layout(root, layout)


def test_output_layout_readback_uses_sealed_host_owner_inside_root_container(
    monkeypatch, tmp_path
):
    module = _load()
    root = tmp_path / "root"
    root.mkdir()
    host_owner = (os.getuid(), os.getgid())
    layout = module._create_rosdistro_output_layout(root)
    finalized = module._finalize_rosdistro_output_layout(root, layout)

    # A validator running in the prepare container observes uid/gid 0.  The
    # sealed root owner, rather than that process identity, is authoritative.
    monkeypatch.setattr(module.os, "getuid", lambda: 0)
    monkeypatch.setattr(module.os, "getgid", lambda: 0)
    module._validate_output_layout_receipt(
        root, finalized, expected_owner=host_owner
    )


def test_output_layout_rejects_current_uid_substitution(monkeypatch, tmp_path):
    module = _load()
    root = tmp_path / "root"
    root.mkdir()
    host_owner = (os.getuid(), os.getgid())
    layout = module._create_rosdistro_output_layout(root)
    finalized = module._finalize_rosdistro_output_layout(root, layout)
    substituted = {
        key: {
            entry_key: (
                dict(descriptor, uid=0, gid=0)
                if isinstance(descriptor, dict)
                else descriptor
            )
            for entry_key, descriptor in entry.items()
        }
        for key, entry in finalized["directories"].items()
    }
    tampered = dict(finalized, directories=substituted)
    monkeypatch.setattr(module.os, "getuid", lambda: 0)
    monkeypatch.setattr(module.os, "getgid", lambda: 0)
    with pytest.raises(module.DiscoveryError, match="artifacts"):
        module._validate_output_layout_receipt(
            root, tampered, expected_owner=host_owner
        )


def test_discovery_root_owner_is_sealed_and_not_current_container_uid(
    monkeypatch, tmp_path
):
    module = _load()
    root = tmp_path / "registration-plugin-rosdep-discovery-20260827-91"
    root_identity = module.create_fresh_root(root)
    profile = {
        "path": "/tmp/profile.json",
        "sha256": "a" * 64,
    }
    receipt = module._base_receipt(
        root_identity,
        profile,
        "discovery-owner-test",
        root.name,
    )
    receipt["output_layout"] = module._create_rosdistro_output_layout(root)
    receipt["output_layout"] = module._finalize_rosdistro_output_layout(
        root, receipt["output_layout"]
    )
    receipt["outcome"] = "PASS_REVIEW_REQUIRED"
    receipt["ended_at_unix"] = receipt["started_at_unix"]
    receipt["canonical_sha256"] = module.canonical_hash(receipt)
    module.seal_bytes(
        root / module.RECEIPT_NAME,
        module.canonical_bytes(receipt) + b"\n",
    )
    monkeypatch.setattr(module.os, "getuid", lambda: 0)
    monkeypatch.setattr(module.os, "getgid", lambda: 0)
    reopened = module.validate_receipt(root)
    assert reopened["root"]["uid"] == os.stat(root).st_uid
    assert reopened["root"]["gid"] == os.stat(root).st_gid


def test_receipt_status_never_promotes():
    module = _load()
    value = module._base_receipt(
        {"path": "/tmp/registration-plugin-rosdep-discovery-20260827-06",
         "device": 1, "inode": 2,
         "uid": 1000, "gid": 1000, "mode": 0o700, "nlink": 2},
        {"path": "/tmp/profile", "sha256": "a" * 64,
         "distro": "humble", "image_digest": module.IMAGE_DIGEST},
        "container", "registration-plugin-rosdep-discovery-20260827-06",
    )
    assert value["status"] == "REVIEW_REQUIRED"
    assert value["benchmark_eligible"] is False
    assert value["promotion"] == "FORBIDDEN_UNTIL_SIGNED_REVIEW"
