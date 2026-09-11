#!/usr/bin/env python3
"""Focused tests for the isolated GnuPG inventory capture contract."""

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import shutil
import socket
import tempfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "scripts/capture_registration_plugin_dependency_closure.py"


def _load():
    spec = importlib.util.spec_from_file_location(
        "capture_registration_plugin_gpg_inventory_test", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write(path: Path, data: bytes, mode: int = 0o444) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    path.chmod(mode)


def _inventory_stdout(primary: str = "A" * 40, subkey: str = "B" * 40) -> bytes:
    return (f"pub:-:rsa4096:1:{primary[-16:]}:1700000000:0:::::::\n"
            f"fpr:::::::::{primary}:\n"
            f"sub:-:rsa4096:1:{subkey[-16:]}:1700000000:0:::::::\n"
            f"fpr:::::::::{subkey}:\n").encode()


def _fixture(module, tmp_path: Path, monkeypatch):
    output = tmp_path / "output"
    module.EVIDENCE_DIRECTORIES.create_fresh_root(output)
    module._create_capture_directory_layout(output, include_prefetch=False)
    keyring = b"deterministic-keyring-bytes\n"
    copied = module._copy_capture_file(output, "apt-keyrings/input.gpg", keyring)

    container = tmp_path / "container"
    _write(container / "usr/bin/gpg", b"fake-gpg-2.2.27\n")
    descriptor = {
        "path": "apt-keyrings/input.gpg", "kind": "path",
        "source": "/usr/share/keyrings/input.gpg", "bytes": len(keyring),
        "sha256": module.sha256_bytes(keyring),
        "normalized_armor_bytes": None, "normalized_armor_sha256": None,
        "dearmored_path": "apt-keyrings/input.gpg",
        "dearmored_bytes": len(keyring),
        "dearmored_sha256": module.sha256_bytes(keyring),
        "primary_fingerprints": [], "subkey_fingerprints": [],
        "expected_signer_fingerprints": [], "inventory": {"status": "PENDING"},
    }
    assert copied["mode"] == 0o444
    return output, container, descriptor


class _Runner:
    def __init__(self, module, *, inventory_result=None, version=None,
                 residue=False, residue_name=None, residue_kind=None):
        self.module = module
        self.inventory_result = inventory_result
        self.version = version or module.GPG_INVENTORY_VERSION
        self.residue = residue
        self.residue_name = residue_name
        self.residue_kind = residue_kind
        self.calls = []

    def __call__(self, argv, timeout):
        self.calls.append((list(argv), timeout))
        if argv[-1] == "--version":
            return 0, (self.version + "\n").encode(), b"", False, None
        homedir = Path(argv[argv.index("--homedir") + 1])
        (homedir / "pubring.kbx").write_bytes(b"pubring metadata\n")
        (homedir / "pubring.kbx").chmod(self.module.GPG_INVENTORY_METADATA_MODE)
        (homedir / "trustdb.gpg").write_bytes(b"trustdb metadata\n")
        (homedir / "trustdb.gpg").chmod(self.module.GPG_INVENTORY_METADATA_MODE)
        if self.residue or self.residue_name is not None:
            name = self.residue_name or "s"
            path = homedir / name
            if self.residue_kind == "fifo" or (self.residue and name == "s"):
                os.mkfifo(path)
            elif self.residue_kind == "symlink":
                path.symlink_to(homedir / "pubring.kbx")
            elif self.residue_kind == "hardlink":
                os.link(homedir / "pubring.kbx", path)
            elif self.residue_kind == "socket":
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                try:
                    sock.bind(str(path))
                finally:
                    sock.close()
            elif self.residue_kind == "directory":
                path.mkdir()
            elif self.residue_kind == "mode":
                path.write_bytes(b"wrong mode\n")
                path.chmod(0o644)
            elif self.residue_kind == "zero":
                path.write_bytes(b"")
                path.chmod(self.module.GPG_INVENTORY_METADATA_MODE)
            elif self.residue_kind == "oversize":
                path.write_bytes(b"x")
                with path.open("r+b") as stream:
                    stream.truncate(self.module.GPG_INVENTORY_METADATA_MAX_BYTES + 1)
                path.chmod(self.module.GPG_INVENTORY_METADATA_MODE)
            else:
                path.write_bytes(b"unexpected metadata\n")
                path.chmod(self.module.GPG_INVENTORY_METADATA_MODE)
        if self.inventory_result is not None:
            return self.inventory_result
        return 0, _inventory_stdout(), b"", False, None


def test_verified_inventory_uses_exact_isolated_command_and_reopens(
        tmp_path, monkeypatch):
    module = _load()
    output, container, descriptor = _fixture(module, tmp_path, monkeypatch)
    runner = _Runner(module)

    module._key_inventory(runner, output, descriptor, 0, container)

    assert len(runner.calls) == 2
    version_argv, version_timeout = runner.calls[0]
    inventory_argv, inventory_timeout = runner.calls[1]
    assert version_argv == [module.GPG_INVENTORY_EXECUTABLE, "--version"]
    assert version_timeout == 30
    assert inventory_argv == module._gpg_inventory_argv(
        Path(descriptor["inventory"]["homedir"]["path"]),
        output / descriptor["dearmored_path"])
    assert inventory_timeout == 120
    assert "--homedir" in inventory_argv
    assert "--no-default-keyring" in inventory_argv
    assert "--no-auto-key-retrieve" in inventory_argv
    assert "--no-auto-key-import" in inventory_argv
    assert "--no-auto-key-locate" in inventory_argv
    assert "--no-autostart" in inventory_argv
    assert "HOME" not in module._fixed_env()
    assert "GNUPGHOME" not in module._fixed_env()
    assert descriptor["inventory"]["status"] == "VERIFIED"
    assert descriptor["inventory"]["homedir"]["post_absent"] is True
    assert not Path(descriptor["inventory"]["homedir"]["path"]).exists()
    descriptor["signer_fingerprint"] = "A" * 40
    module._validate_keyring_descriptor(output, descriptor, "verified keyring")


@pytest.mark.parametrize("mutation", [
    lambda argv: argv[:15] + ["--show-keys", argv[16]],
    lambda argv: argv[:9] + ["/tmp/other-home"] + argv[10:],
    lambda argv: argv[:3] + ["--keyserver", "evil"] + argv[3:],
    lambda argv: argv + ["--no-options"],
])
def test_inventory_argv_missing_extra_reordered_or_prohibited_rejected(
        tmp_path, mutation):
    module = _load()
    homedir = tmp_path / ("glim-clean-room-r3-gpg-inventory-" + "a" * 24)
    keyring = tmp_path / "apt-keyrings/input.gpg"
    argv = module._gpg_inventory_argv(homedir, keyring)
    with pytest.raises(module.CaptureError):
        module._validate_gpg_inventory_argv(mutation(argv), homedir,
                                            "apt-keyrings/input.gpg", "argv")


def test_hostile_home_is_not_passed_and_gpg_244_is_exactly_supported(
        tmp_path, monkeypatch):
    module = _load()
    output, container, descriptor = _fixture(module, tmp_path, monkeypatch)
    monkeypatch.setenv("HOME", str(tmp_path / "hostile-home"))
    monkeypatch.setenv("GNUPGHOME", str(tmp_path / "hostile-gnupg"))
    runner = _Runner(module, version="gpg (GnuPG) 2.4.4")

    module._key_inventory(runner, output, descriptor, 0, container)

    assert descriptor["inventory"]["status"] == "VERIFIED"
    assert descriptor["inventory"]["tool"]["version"] == "gpg (GnuPG) 2.4.4"
    assert descriptor["inventory"]["homedir"]["post_absent"] is True
    descriptor["signer_fingerprint"] = "A" * 40
    module._validate_keyring_descriptor(output, descriptor, "gpg 2.4.4 inventory")


def test_unallowlisted_inventory_version_is_partial(tmp_path, monkeypatch):
    module = _load()
    output, container, descriptor = _fixture(module, tmp_path, monkeypatch)
    runner = _Runner(module, version="gpg (GnuPG) 2.5.0")

    with pytest.raises(module.CaptureError) as error:
        module._key_inventory(runner, output, descriptor, 0, container)

    assert error.value.kind == "GPG_KEY_INVENTORY_VERSION_INVALID"
    assert descriptor["inventory"]["status"] == "FAILED"
    assert descriptor["inventory"]["failure_kind"] == error.value.kind
    assert descriptor["inventory"]["version_stdout"] is not None
    assert descriptor["inventory"]["homedir"]["post_absent"] is True
    module._validate_keyring_descriptor(
        output, descriptor, "version failure", allow_pending=True,
        partial_failure_kind=error.value.kind)


def test_inventory_nonzero_result_preserves_bounded_diagnostics_and_reopens(
        tmp_path, monkeypatch):
    module = _load()
    output, container, descriptor = _fixture(module, tmp_path, monkeypatch)
    runner = _Runner(module, inventory_result=(1, b"", b"gpg failed\n", False, None))

    with pytest.raises(module.CaptureError) as error:
        module._key_inventory(runner, output, descriptor, 0, container)

    assert error.value.kind == "GPG_KEY_INVENTORY_FAILED"
    inventory = descriptor["inventory"]
    assert inventory["status"] == "FAILED"
    assert inventory["exit_status"] == 1
    assert inventory["stdout"] is not None
    assert inventory["stderr"] is not None
    module._validate_keyring_descriptor(
        output, descriptor, "command failure", allow_pending=True,
        partial_failure_kind=error.value.kind)


def test_inventory_residue_socket_is_not_repaired_or_marked_verified(
        tmp_path, monkeypatch):
    module = _load()
    output, container, descriptor = _fixture(module, tmp_path, monkeypatch)
    runner = _Runner(module, residue=True)

    try:
        with pytest.raises(module.CaptureError) as error:
            module._key_inventory(runner, output, descriptor, 0, container)

        assert error.value.kind in {
            "GPG_KEY_INVENTORY_RESIDUE", "GPG_KEY_INVENTORY_CLEANUP_FAILED",
        }
        assert descriptor["inventory"]["status"] == "FAILED"
        assert descriptor["inventory"]["homedir"]["cleanup_status"] == "FAIL_CLOSED"
        assert descriptor["inventory"]["homedir"]["post_absent"] is False
    finally:
        homedir = Path(descriptor["inventory"]["homedir"]["path"])
        (homedir / "s").unlink(missing_ok=True)
        for name in module.GPG_INVENTORY_METADATA_NAMES:
            (homedir / name).unlink(missing_ok=True)
        homedir.rmdir()


def _remove_fixture_homedir(path: Path) -> None:
    for child in list(path.iterdir()):
        if child.is_dir() and not child.is_symlink():
            child.rmdir()
        else:
            child.unlink()
    path.rmdir()


@pytest.mark.parametrize("residue_name,residue_kind", [
    ("unknown", "regular"),
    ("alias", "symlink"),
    ("alias", "hardlink"),
    ("s", "socket"),
    ("fifo", "fifo"),
    ("nested", "directory"),
    ("wrong-mode", "mode"),
    ("empty", "zero"),
    ("oversize", "oversize"),
])
def test_inventory_metadata_residue_is_fail_closed(
        tmp_path, residue_name, residue_kind):
    module = _load()
    owned_short_root = None
    fixture_root = tmp_path
    if residue_kind == "socket":
        owned_short_root = Path(tempfile.mkdtemp(prefix="gpi-", dir="/tmp"))
        fixture_root = owned_short_root
    try:
        output, container, descriptor = _fixture(module, fixture_root, None)
        runner = _Runner(module, residue_name=residue_name, residue_kind=residue_kind)
        with pytest.raises(module.CaptureError) as error:
            module._key_inventory(runner, output, descriptor, 0, container)
        assert error.value.kind in {
            "GPG_KEY_INVENTORY_RESIDUE", "GPG_KEY_INVENTORY_CLEANUP_FAILED",
            "GPG_KEY_INVENTORY_HOMEDIR_CHANGED",
        }
        inventory = descriptor["inventory"]
        metadata = inventory["metadata_residue"]
        assert metadata["status"] == "REJECTED"
        assert metadata["failure_reason"] is not None
        assert inventory["homedir"]["cleanup_status"] == "FAIL_CLOSED"
        assert inventory["homedir"]["post_absent"] is False
    finally:
        if "descriptor" in locals() and "inventory" in descriptor:
            _remove_fixture_homedir(Path(descriptor["inventory"]["homedir"]["path"]))
        if owned_short_root is not None:
            shutil.rmtree(owned_short_root)


def test_inventory_metadata_descriptor_is_complete_and_stderr_bound(
        tmp_path, monkeypatch):
    module = _load()
    output, container, descriptor = _fixture(module, tmp_path, monkeypatch)
    module._key_inventory(_Runner(module), output, descriptor, 0, container)
    inventory = descriptor["inventory"]
    metadata = inventory["metadata_residue"]
    assert metadata["status"] == "EXACT"
    assert metadata["creation_evidence"] == inventory["stderr"]
    assert metadata["expected_names"] == list(module.GPG_INVENTORY_METADATA_NAMES)
    assert [entry["name"] for entry in metadata["entries"]] == [
        "pubring.kbx", "trustdb.gpg"
    ]
    for entry in metadata["entries"]:
        assert set(entry) == {
            "name", "type", "mode", "uid", "gid", "nlink", "size", "device",
            "inode", "sha256", "xattrs", "status", "reason",
        }
        assert entry["status"] == "ACCEPTED"
        assert entry["mode"] == module.GPG_INVENTORY_METADATA_MODE
        assert entry["nlink"] == 1
        assert entry["sha256"]


def test_inventory_metadata_tamper_and_owner_drift_fail_reopen(tmp_path, monkeypatch):
    module = _load()
    output, container, descriptor = _fixture(module, tmp_path, monkeypatch)
    module._key_inventory(_Runner(module), output, descriptor, 0, container)
    metadata = descriptor["inventory"]["metadata_residue"]
    metadata["entries"][0]["size"] += 1
    with pytest.raises(module.CaptureError):
        module._validate_keyring_descriptor(output, descriptor, "metadata tamper")
    metadata["entries"][0]["size"] -= 1
    metadata["entries"][0]["uid"] += 1
    with pytest.raises(module.CaptureError):
        module._validate_keyring_descriptor(output, descriptor, "metadata owner")


def test_inventory_metadata_cleanup_failure_is_not_repaired(tmp_path, monkeypatch):
    module = _load()
    output, container, descriptor = _fixture(module, tmp_path, monkeypatch)
    original_rmdir = module.os.rmdir

    def fail_rmdir(*args, **kwargs):
        raise OSError("injected cleanup failure")

    monkeypatch.setattr(module.os, "rmdir", fail_rmdir)
    with pytest.raises(module.CaptureError):
        module._key_inventory(_Runner(module), output, descriptor, 0, container)
    assert descriptor["inventory"]["homedir"]["cleanup_status"] == "FAIL_CLOSED"
    monkeypatch.setattr(module.os, "rmdir", original_rmdir)
    _remove_fixture_homedir(Path(descriptor["inventory"]["homedir"]["path"]))


@pytest.mark.parametrize("bad", [
    b"pub:r:rsa4096:1:AAAAAAAAAAAAAAAA:0:0:::::::\nfpr:::::::::" + b"A" * 40 + b":\n",
    b"pub:-:rsa4096:1:AAAAAAAAAAAAAAAA:0:0:::::::\nfpr:::::::::" + b"A" * 39 + b":\n",
    b"pub:-:rsa4096:1:AAAAAAAAAAAAAAAA:0:0:::::::\nfpr:::::::::" + b"A" * 40 + b":\n"
    b"fpr:::::::::" + b"A" * 40 + b":\n",
    b"evil:record\n",
])
def test_inventory_parser_rejects_revoked_malformed_duplicate_or_unknown(bad):
    module = _load()
    with pytest.raises(module.CaptureError):
        module._parse_gpg_inventory(bad, b"", "bad inventory")


def test_verified_inventory_log_tamper_fails_reopen(tmp_path, monkeypatch):
    module = _load()
    output, container, descriptor = _fixture(module, tmp_path, monkeypatch)
    runner = _Runner(module)
    module._key_inventory(runner, output, descriptor, 0, container)
    log = output / descriptor["inventory"]["stdout"]["path"]
    log.chmod(0o600)
    log.write_bytes(b"tampered\n")
    with pytest.raises(module.CaptureError):
        module._validate_keyring_descriptor(output, descriptor, "tampered")


def test_inventory_status_shapes_match_source_schema(tmp_path, monkeypatch):
    module = _load()
    output, container, descriptor = _fixture(module, tmp_path, monkeypatch)
    module._key_inventory(_Runner(module), output, descriptor, 0, container)
    schema_path = ROOT / "configs/slam_benchmark_profiles/registration_plugin_apt_source_snapshot_v2.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    inventory = json.loads(json.dumps(descriptor["inventory"]))
    from jsonschema import Draft202012Validator, RefResolver
    resolver = RefResolver.from_schema(schema)
    Draft202012Validator(schema["$defs"]["keyringInventory"], resolver=resolver).validate(
        inventory)


def test_inventory_parent_is_dedicated_fresh_empty_bind(tmp_path, monkeypatch):
    module = _load()
    output, _, _ = _fixture(module, tmp_path, monkeypatch)
    parent = output / module.GPG_INVENTORY_WORK_RELATIVE
    child_name = module.GPG_INVENTORY_HOMEDIR_PREFIX + "a" * 24
    (parent / child_name).mkdir()
    with pytest.raises(module.CaptureError) as error:
        module._gpg_inventory_homedir(output, 0)
    assert error.value.kind in {
        "GPG_KEY_INVENTORY_HOMEDIR_COLLISION",
        "GPG_KEY_INVENTORY_HOMEDIR_PARENT_NOT_EMPTY",
    }
    (parent / child_name).rmdir()
    (parent / "unexpected").write_bytes(b"residue")
    with pytest.raises(module.CaptureError) as error:
        module._gpg_inventory_homedir(output, 0)
    assert error.value.kind == "GPG_KEY_INVENTORY_HOMEDIR_PARENT_NOT_EMPTY"
    (parent / "unexpected").unlink()


def test_inventory_parent_metadata_is_checked_by_outer_host_contract(
        tmp_path, monkeypatch):
    module = _load()
    output, _, _ = _fixture(module, tmp_path, monkeypatch)
    parent = output / module.GPG_INVENTORY_WORK_RELATIVE
    parent.chmod(0o755)
    with pytest.raises(module.CaptureError):
        module._validate_gpg_inventory_parent(output, "changed parent")
    parent.chmod(0o700)
    module._validate_gpg_inventory_parent(output, "restored parent")


def test_old_tmp_parent_identity_shape_is_rejected():
    module = _load()
    old = {
        "path": "/tmp/glim-clean-room-r3-gpg-inventory-" + "a" * 24,
        "status": "FRESH", "mode": 0o700, "uid": 0, "gid": 0,
        "nlink": 2, "device": 1, "inode": 1,
        "parent_device": 1, "parent_inode": 1,
        "post_absent": True, "cleanup_status": "PASS",
    }
    with pytest.raises(module.CaptureError):
        module._validate_inventory_homedir(old, "old parent shape")


def test_failed_inventory_status_shape_is_strictly_reopenable(tmp_path, monkeypatch):
    module = _load()
    output, container, descriptor = _fixture(module, tmp_path, monkeypatch)
    with pytest.raises(module.CaptureError):
        module._key_inventory(
            _Runner(module, version="gpg (GnuPG) 2.5.0"), output, descriptor, 0, container)
    schema_path = ROOT / "configs/slam_benchmark_profiles/registration_plugin_apt_source_snapshot_v2.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    inventory = json.loads(json.dumps(descriptor["inventory"]))
    from jsonschema import Draft202012Validator, RefResolver
    resolver = RefResolver.from_schema(schema)
    Draft202012Validator(schema["$defs"]["keyringInventory"], resolver=resolver).validate(
        inventory)
