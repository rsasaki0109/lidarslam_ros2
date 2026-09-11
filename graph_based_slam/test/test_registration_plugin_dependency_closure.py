#!/usr/bin/env python3
"""Synthetic tests for the registration-plugin dependency closure boundary."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

from jsonschema import Draft202012Validator
import pytest


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "scripts/registration_plugin_dependency_closure.py"


def _load():
    spec = importlib.util.spec_from_file_location("registration_plugin_dependency_closure", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def closure():
    return _load()


def _canonical(module, value):
    return module.canonical_bytes(value)


def _write(path: Path, payload: bytes, mode=0o444):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)
    path.chmod(mode)


def _seal(module, path: Path, value):
    payload = _canonical(module, value) + b"\n"
    _write(path, payload)
    _write(Path(str(path) + ".sha256"),
           (hashlib.sha256(payload).hexdigest() + "  " + path.name + "\n").encode("ascii"))


class _FakeApt:
    """Minimal apt output seam; real apt parsing is covered by its own tests."""

    @staticmethod
    def compose_proposal(**kwargs):
        output = Path(kwargs["output_root"])
        output.mkdir(mode=0o700)
        ledger = {
            "repositories": [{
                "url": "https://apt.example.invalid/jazzy",
                "release_url": "https://apt.example.invalid/jazzy/InRelease",
                "release_path": "release",
                "release_sha256": "a" * 64,
                "packages_url": "https://apt.example.invalid/jazzy/main/Packages",
                "packages_path": "packages",
                "packages_sha256": "b" * 64,
            }],
            "entries": [],
            "apt_source_urls": [{"path": "source.list", "url": "https://apt.example.invalid/source.list"}],
            "rosdep_source_urls": [{"path": "rosdep.list", "url": "https://ros.example.invalid/index.yaml"}],
        }
        proposal = {
            "schema": "fake-apt-proposal-v1",
            "status": "PROPOSED_REVIEW_REQUIRED",
            "apt_acquisition_ledger": ledger,
            "closure_identity_sha256": "c" * 64,
        }
        for filename, value in (
                ("apt-allowlist-generation-proposal.json", proposal),
                ("package-allowlist.json", {"schema": "fake-allowlist-v1"}),
                ("apt-acquisition-ledger.json", ledger)):
            payload = _canonical(_load(), value) + b"\n"
            _write(output / filename, payload, mode=0o600)
            _write(output / (filename + ".sha256"),
                   (hashlib.sha256(payload).hexdigest() + "  " + filename + "\n").encode("ascii"),
                   mode=0o600)
        return {"status": "PROPOSED_REVIEW_REQUIRED"}

    @staticmethod
    def validate_proposal(output):
        del output
        return {"status": "PASS"}


def _phase_records(module, root: Path, dependency_leg="absent"):
    records = []
    for index, phase in enumerate(module.EXPECTED_PHASES):
        log_stdout = root / "logs" / (phase + ".stdout")
        log_stderr = root / "logs" / (phase + ".stderr")
        _write(log_stdout, (phase + " stdout\n").encode("ascii"))
        _write(log_stderr, b"\n")
        if phase == "archive_prefetch":
            argv = ["true"]
        elif phase in {"image_inspect", "preexisting_container_check", "container_start",
                       "network_disconnect", "post_disconnect_inspect"}:
            argv = ["docker", phase]
        elif phase in {"apt_update", "resolver_build", "download_build", "resolver_runtime",
                       "download_runtime", "dependency_install"}:
            argv = ["apt-get", phase]
        elif phase == "rosdep_resolution":
            argv = ["rosdep", "resolve"]
        else:
            argv = ["python3", phase]
        records.append({
            "phase": phase, "argv": argv,
            "argv_sha256": hashlib.sha256(_canonical(module, argv)).hexdigest(),
            "returncode": 0,
            "network_used": module.PHASE_NETWORK[phase]
            if module.PHASE_NETWORK[phase] is not None else dependency_leg == "present",
            "timeout_seconds": 60, "attempts": 1,
            "stdout_path": str(log_stdout.relative_to(root)),
            "stderr_path": str(log_stderr.relative_to(root)),
        })
    return records


def _fixture(module, tmp_path: Path):
    capture_root = tmp_path / "capture"
    source_root = capture_root / "apt-source"
    _write(source_root / "source.list", b"deb https://apt.example.invalid/jazzy main\n")
    _write(capture_root / "requirements.json", b"{}\n")
    _write(capture_root / "base-status.json", b"{}\n")
    _write(capture_root / "source-manifest.json", b"{}\n")
    resolver_paths = {}
    log_paths = {}
    deb_paths = {}
    for role in module.ROLES:
        resolver_paths[role] = "{}-resolver.json".format(role)
        log_paths[role] = "{}-resolver.log".format(role)
        deb_paths[role] = "{}-debs".format(role)
        _write(capture_root / resolver_paths[role], b"{}\n")
        _write(capture_root / log_paths[role], b"resolver\n")
        _write(capture_root / deb_paths[role] / (role + ".deb"), (role + " deb\n").encode())
    phases = _phase_records(module, capture_root)
    capture = {
        "schema": module.CAPTURE_SCHEMA, "schema_version": 1,
        "status": "CAPTURED_REVIEW_REQUIRED", "distro": "jazzy",
        "dependency_leg": "absent", "image_digest": "sha256:" + "d" * 64,
        "platform": module.PLATFORM,
        "requirements_path": "requirements.json", "base_status_path": "base-status.json",
        "source_root": "apt-source", "source_manifest_path": "source-manifest.json",
        "resolvers": {role: {"resolver_path": resolver_paths[role],
                              "raw_log_path": log_paths[role],
                              "deb_root_path": deb_paths[role],
                              "command": ["apt-get", "--print-uris", "install", role],
                              "exit_status": 0} for role in module.ROLES},
        "phase_records": phases, "optional_prefetch": {"status": "NOT_APPLICABLE"},
        "gpgv_status_policy": dict(module.GPGV_STATUS_POLICY),
    }
    capture["canonical_sha256"] = module.canonical_hash(capture)
    _seal(module, capture_root / "capture-input.json", capture)
    return capture_root


def test_capture_input_rejects_shell_fallback_and_phase_reordering(closure):
    records = []
    for phase in closure.EXPECTED_PHASES:
        records.append({
            "phase": phase, "argv": ["true"],
            "argv_sha256": hashlib.sha256(_canonical(closure, ["true"])).hexdigest(),
            "returncode": 0, "network_used": False, "timeout_seconds": 1,
            "attempts": 1, "stdout_path": "x", "stderr_path": "y",
        })
    records[0]["argv"] = ["bash", "-lc", "apt-get update"]
    records[0]["argv_sha256"] = hashlib.sha256(
        _canonical(closure, records[0]["argv"])).hexdigest()
    with pytest.raises(closure.ClosureError) as error:
        closure._validate_phase_records(records, "absent")
    assert error.value.kind == "NETWORK_FALLBACK_COMMAND"
    records[0]["argv"] = ["true"]
    records[0]["argv_sha256"] = hashlib.sha256(_canonical(closure, ["true"])).hexdigest()
    records[0], records[1] = records[1], records[0]
    with pytest.raises(closure.ClosureError) as error:
        closure._validate_phase_records(records, "absent")
    assert error.value.kind == "PHASE_ORDER_INVALID"


def test_capture_input_requires_nonempty_sealed_prefetch_for_present(closure):
    value = {
        "schema": closure.CAPTURE_SCHEMA, "schema_version": 1,
        "status": "CAPTURED_REVIEW_REQUIRED", "distro": "jazzy",
        "dependency_leg": "present", "image_digest": "sha256:" + "d" * 64,
        "platform": closure.PLATFORM, "requirements_path": "requirements.json",
        "base_status_path": "base-status.json", "source_root": "source",
        "source_manifest_path": "source-manifest.json",
        "gpgv_status_policy": dict(closure.GPGV_STATUS_POLICY),
        "resolvers": {role: {"resolver_path": "{}-r".format(role),
                              "raw_log_path": "{}-l".format(role),
                              "deb_root_path": "{}-d".format(role),
                              "command": ["apt-get", "install", role], "exit_status": 0}
                     for role in closure.ROLES},
        "phase_records": [],
        "optional_prefetch": {"status": "SEALED_PREFETCH", "root_path": "prefetch",
                               "dependencies": []},
    }
    value["canonical_sha256"] = closure.canonical_hash(value)
    with pytest.raises(closure.ClosureError) as error:
        closure._validate_capture(value)
    assert error.value.kind in {"PHASE_ORDER_INVALID", "CAPTURE_PREFETCH_INVALID"}


def test_capture_input_requires_exact_gpgv_status_policy(closure, tmp_path):
    capture_root = _fixture(closure, tmp_path)
    original = json.loads(
        (capture_root / "capture-input.json").read_text(encoding="utf-8"))

    missing = dict(original)
    missing.pop("gpgv_status_policy")
    missing["canonical_sha256"] = closure.canonical_hash(missing)
    with pytest.raises(closure.ClosureError) as error:
        closure._validate_capture(missing)
    assert error.value.kind == "CAPTURE_SCHEMA_INVALID"

    drifted = json.loads(json.dumps(original))
    drifted["gpgv_status_policy"]["proof"] = "status_tags_only"
    drifted["canonical_sha256"] = closure.canonical_hash(drifted)
    with pytest.raises(closure.ClosureError) as error:
        closure._validate_capture(drifted)
    assert error.value.kind == "GPGV_STATUS_POLICY_INVALID"

    extra = json.loads(json.dumps(original))
    extra["gpgv_status_policy_extra"] = True
    extra["canonical_sha256"] = closure.canonical_hash(extra)
    with pytest.raises(closure.ClosureError) as error:
        closure._validate_capture(extra)
    assert error.value.kind == "CAPTURE_SCHEMA_INVALID"


def test_capture_input_rejects_aliased_paths_and_leg_network_drift(closure, tmp_path):
    capture_root = _fixture(closure, tmp_path)
    value = json.loads((capture_root / "capture-input.json").read_text(encoding="utf-8"))
    value["base_status_path"] = value["requirements_path"]
    value["canonical_sha256"] = closure.canonical_hash(value)
    with pytest.raises(closure.ClosureError) as error:
        closure._validate_capture(value)
    assert error.value.kind == "CAPTURE_PATH_ALIAS"

    value = json.loads((capture_root / "capture-input.json").read_text(encoding="utf-8"))
    value["dependency_leg"] = "present"
    value["optional_prefetch"] = {"status": "SEALED_PREFETCH", "root_path": "prefetch",
                                   "dependencies": [{"name": "fast_gicp"}]}
    value["canonical_sha256"] = closure.canonical_hash(value)
    with pytest.raises(closure.ClosureError) as error:
        closure._validate_capture(value)
    assert error.value.kind == "PHASE_NETWORK_INVALID"


def test_compose_reopens_inputs_and_leaves_review_required(closure, tmp_path, monkeypatch):
    capture_root = _fixture(closure, tmp_path)
    monkeypatch.setattr(closure, "APT", _FakeApt)
    output_root = tmp_path / "closure"
    result = closure.compose_closure(capture_root=capture_root, output_root=output_root,
                                     dpkg_reader=lambda path: {"name": path.stem,
                                                                "version": "1.0-1",
                                                                "architecture": "amd64"})
    assert result["status"] == closure.REVIEW_STATUS
    assert result["benchmark_eligible"] is False
    assert (output_root / "registration-plugin-dependency-closure.json").stat().st_mode & 0o777 == 0o444
    with pytest.raises(closure.ClosureError) as error:
        closure.validate_runtime_closure(output_root, distro="jazzy",
                                         image_digest="sha256:" + "d" * 64,
                                         dependency_leg="absent")
    assert error.value.kind == "CLOSURE_REVIEW_REQUIRED"


def test_closure_rejects_mutated_bound_input(closure, tmp_path, monkeypatch):
    capture_root = _fixture(closure, tmp_path)
    monkeypatch.setattr(closure, "APT", _FakeApt)
    output_root = tmp_path / "closure"
    closure.compose_closure(capture_root=capture_root, output_root=output_root,
                            dpkg_reader=lambda path: {"name": path.stem,
                                                       "version": "1.0-1",
                                                       "architecture": "amd64"})
    target = capture_root / "requirements.json"
    target.chmod(0o644)
    target.write_bytes(b"changed\n")
    target.chmod(0o444)
    with pytest.raises(closure.ClosureError) as error:
        closure.validate_closure_root(output_root, distro="jazzy",
                                      image_digest="sha256:" + "d" * 64,
                                      dependency_leg="absent")
    assert error.value.kind in {"DESCRIPTOR_DRIFT", "FILE_CHANGED"}


def test_closure_recomputes_phase_and_url_bindings(closure, tmp_path, monkeypatch):
    capture_root = _fixture(closure, tmp_path)
    monkeypatch.setattr(closure, "APT", _FakeApt)
    output_root = tmp_path / "closure"
    closure.compose_closure(capture_root=capture_root, output_root=output_root,
                            dpkg_reader=lambda path: {"name": path.stem,
                                                       "version": "1.0-1",
                                                       "architecture": "amd64"})
    closure_path = output_root / closure.CLOSURE_FILENAME
    value = json.loads(closure_path.read_text(encoding="utf-8"))
    original_phase = json.loads(json.dumps(value["phase_records"][0]))
    value["phase_records"][0]["argv"] = ["true"]
    value["phase_records"][0]["argv_sha256"] = hashlib.sha256(
        _canonical(closure, ["true"])).hexdigest()
    value["canonical_sha256"] = closure.canonical_hash(value)
    closure_path.chmod(0o644)
    Path(str(closure_path) + ".sha256").chmod(0o644)
    _seal(closure, closure_path, value)
    with pytest.raises(closure.ClosureError) as error:
        closure.validate_closure_root(output_root, distro="jazzy",
                                      image_digest="sha256:" + "d" * 64,
                                      dependency_leg="absent")
    assert error.value.kind == "PHASE_BINDING_INVALID"

    value = json.loads(closure_path.read_text(encoding="utf-8"))
    value["phase_records"][0] = original_phase
    value["requested_urls"].append("https://apt.example.invalid/extra")
    value["requested_urls"].sort()
    value["canonical_sha256"] = closure.canonical_hash(value)
    closure_path.chmod(0o644)
    Path(str(closure_path) + ".sha256").chmod(0o644)
    _seal(closure, closure_path, value)
    with pytest.raises(closure.ClosureError) as error:
        closure.validate_closure_root(output_root, distro="jazzy",
                                      image_digest="sha256:" + "d" * 64,
                                      dependency_leg="absent")
    assert error.value.kind in {"DESCRIPTOR_DRIFT", "CLOSURE_URL_BINDING_INVALID"}


def test_closure_rejects_extra_output_artifact(closure, tmp_path, monkeypatch):
    capture_root = _fixture(closure, tmp_path)
    monkeypatch.setattr(closure, "APT", _FakeApt)
    output_root = tmp_path / "closure"
    closure.compose_closure(capture_root=capture_root, output_root=output_root,
                            dpkg_reader=lambda path: {"name": path.stem,
                                                       "version": "1.0-1",
                                                       "architecture": "amd64"})
    _write(output_root / "unexpected.log", b"not bound\n")
    with pytest.raises(closure.ClosureError) as error:
        closure.validate_closure_root(output_root, distro="jazzy",
                                      image_digest="sha256:" + "d" * 64,
                                      dependency_leg="absent")
    assert error.value.kind == "OUTPUT_ALLOWLIST_MISMATCH"


def test_closure_rejects_output_symlink_and_collision(closure, tmp_path, monkeypatch):
    capture_root = _fixture(closure, tmp_path)
    monkeypatch.setattr(closure, "APT", _FakeApt)
    output_root = tmp_path / "closure"
    closure.compose_closure(capture_root=capture_root, output_root=output_root,
                            dpkg_reader=lambda path: {"name": path.stem,
                                                       "version": "1.0-1",
                                                       "architecture": "amd64"})
    (output_root / "unexpected-target").write_bytes(b"target\n")
    (output_root / "unexpected-target").chmod(0o444)
    (output_root / "unexpected-link").symlink_to(output_root / "unexpected-target")
    with pytest.raises(closure.ClosureError) as error:
        closure.validate_closure_root(output_root, distro="jazzy",
                                      image_digest="sha256:" + "d" * 64,
                                      dependency_leg="absent")
    assert error.value.kind == "OUTPUT_SYMLINK"
    with pytest.raises(closure.ClosureError) as error:
        closure.compose_closure(capture_root=capture_root,
                                output_root=output_root,
                                dpkg_reader=lambda path: {"name": path.stem,
                                                           "version": "1.0-1",
                                                           "architecture": "amd64"})
    assert error.value.kind == "OUTPUT_ROOT_NOT_FRESH"


def test_phase_limits_and_https_are_fail_closed(closure):
    item = {
        "phase": "image_inspect", "argv": ["docker", "image", "inspect"],
        "argv_sha256": hashlib.sha256(_canonical(
            closure, ["docker", "image", "inspect"])).hexdigest(),
        "returncode": 0, "network_used": False, "timeout_seconds": 601,
        "attempts": 1, "stdout_path": "stdout", "stderr_path": "stderr",
    }
    with pytest.raises(closure.ClosureError) as error:
        closure._validate_phase_record(item, 0, "absent")
    assert error.value.kind == "PHASE_LIMIT_INVALID"
    with pytest.raises(closure.ClosureError):
        closure._https("http://apt.example.invalid/Release", "release URL")
    with pytest.raises(closure.ClosureError):
        closure._https("https://apt.example.invalid/Release?redirect=1", "release URL")


def test_profile_bound_http_apt_allowlist_is_exact(closure):
    allowed = (
        "http://archive.ubuntu.com/ubuntu",
        "http://archive.ubuntu.com:80/ubuntu/dists/jazzy/InRelease",
        "http://packages.ros.org/ros2/ubuntu/dists/jazzy/InRelease",
        "http://security.ubuntu.com/ubuntu/pool/main/x/example.deb",
    )
    for value in allowed:
        assert closure._apt_or_https(value, "APT URL") == value

    rejected = (
        "http://archive.ubuntu.com.evil/ubuntu",
        "http://archive.ubuntu.com/ubuntuish",
        "http://archive.ubuntu.com:8080/ubuntu",
        "http://user@archive.ubuntu.com/ubuntu",
        "http://archive.ubuntu.com/ubuntu?x=1",
        "http://archive.ubuntu.com/ubuntu#fragment",
        "http://unknown.invalid/ubuntu",
        "http://archive.ubuntu.com/ubuntu\nextra",
    )
    for value in rejected:
        with pytest.raises(closure.ClosureError) as error:
            closure._apt_or_https(value, "APT URL")
        assert error.value.kind == "URL_INVALID"


def test_sealed_closure_schema_is_strict(closure, tmp_path, monkeypatch):
    capture_root = _fixture(closure, tmp_path)
    monkeypatch.setattr(closure, "APT", _FakeApt)
    output_root = tmp_path / "closure"
    closure.compose_closure(capture_root=capture_root, output_root=output_root,
                            dpkg_reader=lambda path: {"name": path.stem,
                                                       "version": "1.0-1",
                                                       "architecture": "amd64"})
    schema_path = ROOT / "configs/slam_benchmark_profiles/registration_plugin_dependency_closure_v1.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(schema)
    value = json.loads((output_root / closure.CLOSURE_FILENAME).read_text(encoding="utf-8"))
    Draft202012Validator(schema).validate(value)
    assert value["capture_executor"]["schema"] == (
        "registration-plugin-dependency-capture-executor-v1")
    assert value["capture_executor"]["capture_input"] == value["capture_input"]
    assert value["apt_source_url_policy"] == closure.APT_SOURCE_URL_POLICY
    assert value["verification_chain"] == closure.VERIFICATION_CHAIN
    value["unexpected"] = True
    with pytest.raises(Exception):
        Draft202012Validator(schema).validate(value)


def test_runtime_policy_requires_no_network_and_no_pull(closure):
    assert closure.EXPECTED_PHASES[-3:] == (
        "dependency_capture", "archive_prefetch", "post_disconnect_inspect")
    assert closure.EXPECTED_PHASES[13:16] == (
        "network_disconnect", "rosdep_prepare", "rosdep_resolution")
    assert closure.FORBIDDEN_COMMAND_TOKENS.issuperset({"curl", "wget", "git", "bash"})


def test_signed_empty_source_tree_requires_explicit_opt_in(closure, tmp_path):
    root = tmp_path / "apt-source"
    _write(root / "indices/Packages-signed-empty", b"")
    with pytest.raises(closure.ClosureError) as error:
        closure._tree_descriptor(root, "ordinary tree")
    assert error.value.kind == "FILE_INVALID"

    descriptor = closure._tree_descriptor(
        root, "apt source snapshot", allow_empty_files=True)
    assert descriptor["files"][0]["bytes"] == 0
    assert closure._verify_tree(
        descriptor, "apt source snapshot", allow_empty_files=True) == descriptor


def test_apt_source_tree_uses_decoded_packages_bound(closure, tmp_path, monkeypatch):
    root = tmp_path / "apt-source"
    _write(root / "indices/Packages-large", b"x" * 9)
    monkeypatch.setattr(closure, "MAX_TEXT_BYTES", 8)
    monkeypatch.setattr(closure, "MAX_DECODED_PACKAGES_BYTES", 32)

    with pytest.raises(closure.ClosureError) as error:
        closure._tree_descriptor(root, "ordinary tree", maximum=closure.MAX_TEXT_BYTES)
    assert error.value.kind == "FILE_INVALID"

    descriptor = closure._tree_descriptor(
        root, "apt source snapshot",
        maximum=closure.MAX_DECODED_PACKAGES_BYTES, allow_empty_files=True)
    assert descriptor["files"][0]["bytes"] == 9
    assert closure._verify_tree(
        descriptor, "apt source snapshot", allow_empty_files=True,
        maximum=closure.MAX_DECODED_PACKAGES_BYTES) == descriptor


def test_deb_tree_accepts_safe_apt_inputs_and_excludes_housekeeping(
        closure, tmp_path):
    root = tmp_path / "debs"
    root.mkdir(mode=0o700)
    partial = root / "partial"
    partial.mkdir(mode=0o700)
    lock = root / "lock"
    _write(lock, b"", mode=0o640)
    deb = root / "demo_1_amd64.deb"
    _write(deb, b"deb bytes", mode=0o644)
    descriptor = closure._deb_tree_descriptor(root, "build downloaded debs")
    assert descriptor["directories"] == []
    assert [item["path"] for item in descriptor["files"]] == [deb.name]
    assert closure._verify_deb_tree(
        descriptor, "build downloaded debs") == descriptor

    extra = root / "not-a-deb"
    _write(extra, b"unexpected", mode=0o644)
    with pytest.raises(closure.ClosureError) as error:
        closure._deb_tree_descriptor(root, "build downloaded debs")
    assert error.value.kind == "TREE_FILE_INVALID"


def test_requested_urls_expands_v2_apt_source_url_sets(closure, tmp_path):
    apt_root = tmp_path / "apt"
    proposal = {
        "apt_acquisition_ledger": {
            "source_manifest_schema": closure.APT_SOURCE_SCHEMA_V2,
            "repositories": [], "entries": [], "rosdep_source_urls": [],
            "apt_source_urls": [{
                "path": "etc/apt/sources.list",
                "urls": [
                    "http://archive.ubuntu.com/ubuntu/",
                    "http://security.ubuntu.com/ubuntu/",
                ],
            }, {
                "path": "etc/apt/sources.list.d/inactive.list",
                "urls": [],
            }],
        },
    }
    _write(
        apt_root / "apt-allowlist-generation-proposal.json",
        _canonical(closure, proposal) + b"\n")
    requested, final = closure._requested_urls(apt_root)
    assert requested == final == [
        "http://archive.ubuntu.com/ubuntu/",
        "http://security.ubuntu.com/ubuntu/",
    ]
    assert closure._validate_closure_url_token(requested[0]) == requested[0]
    archive = "optional-dependency:" + "a" * 64
    assert closure._validate_closure_url_token(archive) == archive
    with pytest.raises(closure.ClosureError):
        closure._validate_closure_url_token("optional-dependency:not-a-sha")
