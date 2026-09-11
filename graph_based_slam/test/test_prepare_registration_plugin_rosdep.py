#!/usr/bin/env python3
"""Adversarial tests for the offline rosdep discovery consumer."""

from __future__ import annotations

import importlib.util
import copy
import json
import os
from pathlib import Path
import shutil
from types import SimpleNamespace

import jsonschema
import pytest


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "scripts/prepare_registration_plugin_rosdep.py"
SCHEMA_PATH = ROOT / (
    "configs/slam_benchmark_profiles/"
    "registration_plugin_rosdep_prepare_v1.schema.json"
)


def _load():
    spec = importlib.util.spec_from_file_location(
        "prepare_registration_plugin_rosdep_test", MODULE_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _json_bytes(value):
    return json.dumps(
        value, ensure_ascii=True, sort_keys=True, separators=(",", ":")
    ).encode("utf-8") + b"\n"


def _sync_fixture_receipt(module, root, value, monkeypatch):
    value["root"]["path"] = str(root)
    info = root.lstat()
    value["root"].update({
        "device": info.st_dev,
        "inode": info.st_ino,
        "mode": info.st_mode & 0o777,
        "uid": info.st_uid,
        "gid": info.st_gid,
        "nlink": info.st_nlink,
    })
    value["canonical_sha256"] = module.canonical_hash(value)
    payload = _json_bytes(value)
    receipt = root / module.DISCOVERY_RECEIPT_NAME
    receipt.chmod(0o600)
    receipt.write_bytes(payload)
    receipt.chmod(0o444)
    sidecar = Path(str(receipt) + ".sha256")
    sidecar.chmod(0o600)
    sidecar.write_text(
        module._sha256(payload) + " " + receipt.name + "\n",
        encoding="ascii",
    )
    sidecar.chmod(0o444)
    monkeypatch.setattr(module, "EXPECTED_RECEIPT_SHA256", module._sha256(payload))
    monkeypatch.setattr(
        module, "EXPECTED_DISCOVERY_CANONICAL_SHA256",
        value["canonical_sha256"],
    )


@pytest.fixture
def fixture_module():
    return _load()


@pytest.fixture
def discovery_copy(fixture_module, monkeypatch, tmp_path):
    module = fixture_module
    source = module.DISCOVERY_ROOT
    if not source.is_dir():
        pytest.skip("sealed discovery-11 evidence is not mounted")
    root = tmp_path / module.DISCOVERY_ID
    shutil.copytree(source, root)
    receipt_path = root / module.DISCOVERY_RECEIPT_NAME
    value = json.loads(receipt_path.read_text(encoding="utf-8"))
    _sync_fixture_receipt(module, root, value, monkeypatch)
    monkeypatch.setattr(module, "DISCOVERY_ROOT", root)
    monkeypatch.setattr(module, "PINNED_DISCOVERY_ROOT", root)
    profile = json.loads(module.PROFILE_PATH.read_text(encoding="utf-8"))
    policy = dict(
        profile["release_matrix"]["dependency_closure"][
            "consumer_profile_binding"
        ]
    )
    policy["discovery_allowlist"] = module._discovery_allowlist()
    monkeypatch.setattr(module, "_load_consumer_profile_policy", lambda _: policy)

    def load_receipt(ignored_root):
        return json.loads(receipt_path.read_text(encoding="utf-8"))

    monkeypatch.setattr(
        module,
        "_discovery_module",
        lambda _logical_profile_path=None: SimpleNamespace(
            validate_receipt=load_receipt
        ),
    )
    return module, root, value


def _runner(module, calls, failure=None, version="0.26.0"):
    versions = "".join(
        "{}\t{}\t{}\tinstall ok installed\n".format(
            item["name"], item["version"], item["architecture"]
        )
        for item in module.EXPECTED_PACKAGES
    ).encode("ascii")

    def run(argv, env, timeout, network):
        calls.append((list(argv), dict(env), timeout, network))
        assert network == "none"
        label = argv[1] if len(argv) > 1 else argv[0]
        if failure == label:
            return {
                "returncode": 7,
                "stdout": b"",
                "stderr": b"synthetic failure",
                "network_attempts": [],
            }
        if argv[:2] == ["dpkg-query", "--show"]:
            return {
                "returncode": 0, "stdout": versions, "stderr": b"",
                "network_attempts": [],
            }
        if argv[:2] == ["dpkg-query", "--listfiles"]:
            package = argv[2]
            return {
                "returncode": 0,
                "stdout": ("/usr/lib/python3/{}.py\n".format(package)).encode(),
                "stderr": b"",
                "network_attempts": [],
            }
        if argv[:2] == ["rosdep", "--version"]:
            output = (version + "\n").encode("ascii")
        elif argv[0] == "find":
            output = b"index.yaml\tf\t4\n"
        else:
            output = b"offline-ok\n"
        return {
            "returncode": 0, "stdout": output, "stderr": b"",
            "network_attempts": [],
        }

    return run


def _prepare(module, root, tmp_path, runner):
    tmp_path.mkdir(parents=True, exist_ok=True)
    output = tmp_path / "prepare-output"
    return output, module.prepare_rosdep(
        output, discovery_root=root, runner=runner,
        clock=iter((100, 101)).__next__,
    )


def test_exact_discovery11_binding_is_reopened(fixture_module):
    module = fixture_module
    if not module.DISCOVERY_ROOT.is_dir():
        pytest.skip("sealed discovery-11 evidence is not mounted")
    binding = module.validate_discovery_receipt()
    assert binding["receipt"]["sha256"] == (
        "c9bfbd31a61a95a9263e8b46f7ffdb99f4353daa672ed8bd85d647065b7ac780"
    )
    assert len(binding["packages"]) == 4
    assert len(binding["selected_files"]) == 8
    assert binding["rosdistro"]["commit"] == (
        "2667056d0320bb77edf1e28fec13d56aa8d325e2"
    )


def test_mock_prepare_is_offline_and_nonpromoting(discovery_copy, tmp_path):
    module, root, _ = discovery_copy
    calls = []
    output, receipt = _prepare(module, root, tmp_path, _runner(module, calls))
    assert receipt["discovery_profile_sha256"] == module.DISCOVERY_PROFILE_SHA256
    assert receipt["consumer_profile_sha256"] == module._profile_sha(
        module.PROFILE_PATH
    )
    assert receipt["consumer_profile_sha256"] != receipt["discovery_profile_sha256"]
    assert receipt["consumer_profile"] == {
        "path": str(module.PROFILE_PATH),
        "sha256": receipt["consumer_profile_sha256"],
    }
    assert receipt["status"] == "REVIEW_REQUIRED"
    assert receipt["benchmark_eligible"] is False
    assert receipt["claim_eligible"] is False
    assert receipt["active_profile_switch"] is False
    assert receipt["promotion"] == "FORBIDDEN_UNTIL_SIGNED_REVIEW"
    assert [item["status"] for item in receipt["phases"]] == ["PASS", "PASS"]
    assert receipt["input_binding"]["schema"] == (
        "registration-plugin-rosdep-prepare-input-v1"
    )
    assert len(receipt["input_binding"]["files"]) == 4
    assert all(
        item["source"] == item["before"] == item["after"]
        for item in receipt["input_binding"]["files"]
    )
    dpkg_versions = next(
        command for command in receipt["phases"][0]["commands"]
        if command["label"] == "dpkg_versions"
    )
    assert dpkg_versions["argv"][:3] == [
        "dpkg-query", "--show",
        "--showformat=${Package}\\t${Version}\\t${Architecture}\\t${Status}\\n",
    ]
    rosdep_update = next(
        command for command in receipt["phases"][1]["commands"]
        if command["label"] == "rosdep_update"
    )
    assert rosdep_update["argv"] == [
        "rosdep", "update",
    ]
    rosdep_resolve = next(
        command for command in receipt["phases"][1]["commands"]
        if command["label"] == "rosdep_resolve"
    )
    assert rosdep_resolve["argv"] == [
        "rosdep", "resolve", "--rosdistro", "humble",
        "--sources-cache-dir", module.EXPECTED_GENERATED_ROSDEP_CACHE,
        *module.EXPECTED_ROSDEP_KEYS,
    ]
    offline_index = output / "artifacts/rosdep/index-v4-offline.yaml"
    assert offline_index.read_bytes() == module.OFFLINE_INDEX_BYTES
    distribution = output / "artifacts/rosdep/humble-distribution.yaml"
    assert distribution.stat().st_size == module.EXPECTED_HUMBLE_DISTRIBUTION_BYTES
    assert module._sha256(distribution.read_bytes()) == (
        module.EXPECTED_HUMBLE_DISTRIBUTION_SHA256
    )
    assert receipt["phases"][1]["distribution"]["path"] == (
        "artifacts/rosdep/humble-distribution.yaml"
    )
    assert b"  libg2o:\n" in distribution.read_bytes()
    assert receipt["phases"][1]["env"]["ROSDISTRO_INDEX_URL"] == (
        "file://" + module.EXPECTED_OFFLINE_INDEX
    )
    assert all(item[3] == "none" for item in calls)
    source = output / "artifacts/rosdep/sources.list.d/20-local.list"
    source_text = source.read_text(encoding="utf-8")
    assert all(
        line.split()[1].startswith("file:///")
        for line in source_text.splitlines()
    )
    assert "http://" not in source_text
    assert "https://" not in source_text
    assert module.validate_prepare_receipt(output, discovery_root=root) == receipt
    with pytest.raises(module.PrepareError, match="signed external review"):
        module.reject_promotion(receipt)


def test_prepare_separates_logical_and_transport_roots_and_uses_fixed_discovery_paths(
    discovery_copy, tmp_path
):
    module, root, _ = discovery_copy
    calls = []
    output = tmp_path / "transport-output"
    logical = tmp_path / "logical-host" / "rosdep-prepare"
    receipt = module.prepare_rosdep(
        output, discovery_root=root, logical_output_root=logical,
        runner=_runner(module, calls), clock=iter((100, 101)).__next__,
    )

    assert receipt["root"]["path"] == str(logical)
    assert receipt["transport_root"]["path"] == str(output)
    assert receipt["root_mount"] == {
        "source": str(logical), "target": str(output),
        "read_only": False, "type": "bind", "noexec": False,
        "identity": {
            key: receipt["transport_root"][key]
            for key in ("device", "inode", "uid", "gid", "mode", "nlink")
        },
    }
    assert all(call[1].get("PATH") == module.EXPECTED_NETWORK_ENV["PATH"]
               for call in calls if call[0][0] in {"rosdep", "find"})
    assert all(call[1].get("PATH") == module.EXPECTED_INSTALL_ENV["PATH"]
               for call in calls if call[0][0] in {"dpkg", "dpkg-query"})
    dpkg_install = next(call[0] for call in calls
                        if call[0][:2] == ["dpkg", "--install"])
    assert all(path.startswith(str(module.DISCOVERY_TRANSPORT_ROOT) + "/")
               for path in dpkg_install[3:])
    assert all(str(root) not in path for path in dpkg_install[3:])


def test_discovery_transport_projection_reopens_fixed_namespace_and_rejects_drift(
    discovery_copy,
):
    module, root, _ = discovery_copy
    binding = module.validate_discovery_receipt(root)
    assert binding["receipt"]["transport_descriptor"]["path"] == str(
        module.DISCOVERY_TRANSPORT_ROOT / module.DISCOVERY_RECEIPT_NAME
    )
    assert binding["receipt"]["transport_descriptor"]["sidecar"]["path"] == (
        str(module.DISCOVERY_TRANSPORT_ROOT / module.DISCOVERY_RECEIPT_NAME)
        + ".sha256"
    )
    projection = module._discovery_transport_projection(
        binding, module.DISCOVERY_TRANSPORT_ROOT,
        logical_root=root, actual_root=root,
    )
    assert projection["mount"] == {
        "source": str(root), "target": str(module.DISCOVERY_TRANSPORT_ROOT),
        "read_only": True, "type": "bind", "noexec": False,
    }
    assert projection["artifact_count"] == len(projection["artifacts"])
    for item in projection["artifacts"]:
        assert item["transport_path"] == (
            str(module.DISCOVERY_TRANSPORT_ROOT) + "/" + item["relative_path"]
        )
        assert item["transport_descriptor"]["path"] == item["transport_path"]
        assert item["transport_descriptor"]["sha256"] == item["descriptor"]["sha256"]
    altered = copy.deepcopy(projection)
    altered["artifacts"][0]["transport_path"] += "-substitution"
    with pytest.raises(module.PrepareError, match="TRANSPORT_BINDING"):
        module._validate_discovery_transport_projection(
            altered, binding, actual_root=root
        )


def test_prepare_receipt_matches_strict_schema(discovery_copy, tmp_path):
    module, root, _ = discovery_copy
    output, receipt = _prepare(module, root, tmp_path, _runner(module, []))
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.Draft202012Validator(schema).validate(receipt)
    assert output.joinpath(
        "registration_plugin_rosdep_prepare.receipt.json.sha256"
    ).stat().st_mode & 0o777 == 0o444


def test_partial_failure_is_sealed_and_reopenable(discovery_copy, tmp_path):
    module, root, _ = discovery_copy
    calls = []
    output, receipt = _prepare(
        module, root, tmp_path, _runner(module, calls, failure="update")
    )
    assert receipt["status"] == "PARTIAL_FAILURE_REVIEW_REQUIRED"
    assert receipt["failure"]["phase"] == "rosdep_prepare"
    assert receipt["phases"][0]["status"] == "PASS"
    assert receipt["phases"][1]["status"] == "FAILED"
    assert receipt["input_binding"]["files"]
    assert module._validate_prepare_input_binding(receipt["input_binding"]) is None
    assert receipt["phases"][1]["source_list"]["path"].endswith("20-local.list")
    assert module.validate_prepare_receipt(
        output, discovery_root=root
    )["failure"] == receipt["failure"]
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator(schema).validate(receipt)


def test_input_binding_is_common_and_required_on_both_outcomes(
    discovery_copy, tmp_path
):
    module, root, _ = discovery_copy
    success_output, success = _prepare(
        module, root, tmp_path / "success", _runner(module, [])
    )
    failure_output, failure = _prepare(
        module, root, tmp_path / "failure",
        _runner(module, [], failure="dpkg_versions"),
    )
    assert success["input_binding"]
    assert failure["input_binding"]
    assert success["input_binding"] == failure["input_binding"]
    assert module.validate_prepare_receipt(
        success_output, discovery_root=root
    )["input_binding"] == success["input_binding"]
    assert module.validate_prepare_receipt(
        failure_output, discovery_root=root
    )["input_binding"] == failure["input_binding"]


@pytest.mark.parametrize(
    "mutation,expected",
    [
        (lambda value: value.pop("input_binding"), "fields"),
        (lambda value: value.__setitem__("input_binding", None), "fields"),
        (
            lambda value: value["input_binding"].__setitem__(
                "identity_sha256", "0" * 64
            ),
            "identity",
        ),
        (
            lambda value: value["input_binding"]["files"][0].__setitem__(
                "container_path", "/opt/registration-plugin/evil.py"
            ),
            "fields",
        ),
    ],
)
def test_input_binding_tamper_is_rejected(discovery_copy, tmp_path, mutation, expected):
    module, root, _ = discovery_copy
    _, receipt = _prepare(module, root, tmp_path, _runner(module, []))
    altered = copy.deepcopy(receipt)
    mutation(altered)
    with pytest.raises(module.PrepareError, match="INPUT_BINDING|RECEIPT"):
        module._validate_prepare_input_binding(altered.get("input_binding"))


def test_root_must_be_the_pinned_discovery11_root(fixture_module, tmp_path):
    module = fixture_module
    with pytest.raises(module.PrepareError, match="NOT_PINNED"):
        module.validate_discovery_receipt(tmp_path / module.DISCOVERY_ID)


def test_receipt_canonical_mismatch_is_rejected(discovery_copy):
    module, root, value = discovery_copy
    value["canonical_sha256"] = "0" * 64
    receipt = root / module.DISCOVERY_RECEIPT_NAME
    receipt.chmod(0o600)
    receipt.write_bytes(_json_bytes(value))
    receipt.chmod(0o444)
    with pytest.raises(module.PrepareError, match="canonical"):
        module.validate_discovery_receipt(root)


def test_receipt_sidecar_mismatch_is_rejected(discovery_copy):
    module, root, _ = discovery_copy
    sidecar = root / (module.DISCOVERY_RECEIPT_NAME + ".sha256")
    sidecar.chmod(0o600)
    sidecar.write_text("0" * 64 + " wrong.json\n", encoding="ascii")
    sidecar.chmod(0o444)
    with pytest.raises(module.PrepareError, match="SIDECAR"):
        module.validate_discovery_receipt(root)


def test_missing_and_extra_debs_are_rejected(discovery_copy, monkeypatch):
    module, root, value = discovery_copy
    value["apt"]["packages"].pop()
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="package count"):
        module.validate_discovery_receipt(root)
    value["apt"]["packages"].append(
        dict(module._discovery_module().validate_receipt(root)["apt"]["packages"][0])
    )
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="BINDING"):
        module.validate_discovery_receipt(root)


def test_deb_hash_size_mode_and_link_are_rechecked(discovery_copy):
    module, root, value = discovery_copy
    artifact = value["apt"]["packages"][0]["deb_artifact"]
    path = root / artifact["path"]
    original = path.read_bytes()
    path.chmod(0o600)
    path.write_bytes(original[:-1] + b"X")
    path.chmod(0o444)
    with pytest.raises(
        module.PrepareError, match="SIDECAR|BINDING|METADATA"
    ):
        module.validate_discovery_receipt(root)


def test_deb_hardlink_is_rejected(discovery_copy):
    module, root, value = discovery_copy
    first = root / value["apt"]["packages"][0]["deb_artifact"]["path"]
    second = root / value["apt"]["packages"][1]["deb_artifact"]["path"]
    payload = first.read_bytes()
    first.unlink()
    os.link(second, first)
    assert first.read_bytes() != payload or first.stat().st_nlink == 2
    with pytest.raises(module.PrepareError, match="ARTIFACT|METADATA"):
        module.validate_discovery_receipt(root)


def test_archive_commit_url_and_selected_file_are_exact(discovery_copy, monkeypatch):
    module, root, value = discovery_copy
    value["rosdistro"]["archive"]["requested_url"] = (
        "https://codeload.github.com/ros/rosdistro/tar.gz/master"
    )
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="archive"):
        module.validate_discovery_receipt(root)

    value["rosdistro"]["archive"]["requested_url"] = (
        "https://codeload.github.com/ros/rosdistro/tar.gz/" + module.EXPECTED_COMMIT
    )
    value["rosdistro"]["archive_details"]["selected_files"][0]["sha256"] = "0" * 64
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="FILE"):
        module.validate_discovery_receipt(root)


def test_commit_addressed_source_record_rejects_mutable_substitution(
    discovery_copy, monkeypatch
):
    module, root, value = discovery_copy
    value["rosdistro"]["archive_details"]["source_records"][0]["commit_url"] = (
        "https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml"
    )
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="selected graph"):
        module.validate_discovery_receipt(root)


def test_selected_artifact_symlink_is_rejected(discovery_copy):
    module, root, value = discovery_copy
    artifact = value["rosdistro"]["archive_details"]["selected_files"][0]["artifact"]
    path = root / artifact["path"]
    saved = path.read_bytes()
    path.unlink()
    path.symlink_to(Path("/etc/passwd"))
    with pytest.raises(module.PrepareError, match="ARTIFACT"):
        module.validate_discovery_receipt(root)
    path.unlink()
    path.write_bytes(saved)
    path.chmod(0o444)


def test_selected_path_traversal_is_rejected(discovery_copy, monkeypatch):
    module, root, value = discovery_copy
    selected = value["rosdistro"]["archive_details"]["selected_files"][0]
    selected["relative_path"] = "../outside.yaml"
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="FILE_BINDING|PATH"):
        module.validate_discovery_receipt(root)


def test_source_bindings_use_only_fixed_local_file_uris(discovery_copy):
    module, root, _ = discovery_copy
    binding = module.validate_discovery_receipt(root)
    text, records = module._source_bindings(binding)
    assert len(records) == 4
    assert {record["type"] for record in records} == {"yaml"}
    assert all(
        "file:///opt/registration-plugin/rosdistro/files/" in line
        for line in text.splitlines()
    )
    assert not any(url in text for url in ("http://", "https://", "master"))


def test_humble_distribution_is_exactly_derived_from_pinned_archive(discovery_copy):
    module, root, _ = discovery_copy
    binding = module.validate_discovery_receipt(root)
    payload = module._humble_distribution_bytes(binding, root)
    assert len(payload) == module.EXPECTED_HUMBLE_DISTRIBUTION_BYTES
    assert module._sha256(payload) == module.EXPECTED_HUMBLE_DISTRIBUTION_SHA256
    assert module.EXPECTED_ROSDEP_KEYS[0] == "libg2o"
    assert b"  libg2o:\n" in payload
    assert b"version: 2020.5.29-4\n" in payload


def test_jazzy_prepare_uses_jazzy_distribution_and_resolver(discovery_copy, tmp_path):
    module, root, _ = discovery_copy
    calls = []
    output = tmp_path / "jazzy-prepare"

    receipt = module.prepare_rosdep(
        output,
        discovery_root=root,
        runner=_runner(module, calls),
        clock=iter((100, 101)).__next__,
        distro="jazzy",
    )

    resolve = next(
        command for command in receipt["phases"][1]["commands"]
        if command["label"] == "rosdep_resolve"
    )
    assert resolve["argv"][2:4] == ["--rosdistro", "jazzy"]
    distribution = output / "artifacts/rosdep/jazzy-distribution.yaml"
    spec = module.OFFLINE_DISTRIBUTIONS["jazzy"]
    assert distribution.stat().st_size == spec["bytes"]
    assert module._sha256(distribution.read_bytes()) == spec["sha256"]
    offline_index = (
        output / "artifacts/rosdep/index-v4-offline.yaml"
    ).read_bytes()
    assert b"  jazzy:\n" in offline_index
    assert b"artifacts/rosdep/jazzy-distribution.yaml\n" in offline_index
    assert b"  humble:\n" not in offline_index


def test_source_record_path_escape_is_rejected(discovery_copy):
    module, root, _ = discovery_copy
    binding = module.validate_discovery_receipt(root)
    binding["source_records"][0]["relative_path"] = "../escape.yaml"
    with pytest.raises(module.PrepareError, match="PATH"):
        module._source_bindings(binding)


def test_hostile_runner_environment_is_not_accepted(discovery_copy, tmp_path):
    module, root, _ = discovery_copy

    def hostile(argv, env, timeout, network):
        if argv[0] == "rosdep":
            assert env["HOME"] == "/nonexistent"
            assert env["ROS_HOME"] == module.EXPECTED_ROS_HOME
        return _runner(module, [])(argv, env, timeout, network)

    _prepare(module, root, tmp_path, hostile)


def test_network_attempts_and_url_fallback_are_rejected(discovery_copy, tmp_path):
    module, root, _ = discovery_copy

    def attempted(argv, env, timeout, network):
        return {
            "returncode": 0, "stdout": b"", "stderr": b"",
            "network_attempts": ["https://evil.invalid"],
        }

    output, receipt = _prepare(module, root, tmp_path, attempted)
    assert receipt["status"] == "PARTIAL_FAILURE_REVIEW_REQUIRED"
    assert receipt["failure"]["kind"] == "RUNNER_RESULT_INVALID"
    assert output.is_dir()

    def url_output(argv, env, timeout, network):
        result = _runner(module, [])(argv, env, timeout, network)
        if argv[0] == "rosdep":
            result["stdout"] = b"https://unexpected.invalid\n"
        return result

    output, receipt = _prepare(module, root, tmp_path / "url", url_output)
    assert receipt["failure"]["kind"] == "NETWORK_FALLBACK_OBSERVED"


def test_exact_jazzy_rosdep_deprecation_warning_is_not_a_network_attempt(
    discovery_copy, tmp_path
):
    module, root, _ = discovery_copy

    def jazzy_warning(argv, env, timeout, network):
        result = _runner(module, [])(argv, env, timeout, network)
        if argv == ["rosdep", "--version"]:
            result["stderr"] = module.ROSDEP_PKG_RESOURCES_DEPRECATION_WARNING
        return result

    _, receipt = _prepare(module, root, tmp_path, jazzy_warning)
    assert receipt["status"] == "REVIEW_REQUIRED"
    version_command = receipt["phases"][1]["commands"][0]
    assert version_command["label"] == "rosdep_version"
    assert version_command["stderr"]["bytes"] == len(
        module.ROSDEP_PKG_RESOURCES_DEPRECATION_WARNING
    )


def test_exact_jazzy_rosdep_warning_prefix_is_allowed_for_update(
    discovery_copy, tmp_path
):
    module, root, _ = discovery_copy

    def jazzy_warning(argv, env, timeout, network):
        result = _runner(module, [])(argv, env, timeout, network)
        if argv == ["rosdep", "update"]:
            result["stderr"] = (
                module.ROSDEP_PKG_RESOURCES_DEPRECATION_WARNING
                + b"local cache updated\n"
            )
        return result

    _, receipt = _prepare(module, root, tmp_path, jazzy_warning)
    assert receipt["status"] == "REVIEW_REQUIRED"
    update_command = receipt["phases"][1]["commands"][1]
    assert update_command["label"] == "rosdep_update"
    assert update_command["stderr"]["bytes"] > len(
        module.ROSDEP_PKG_RESOURCES_DEPRECATION_WARNING
    )


def test_modified_jazzy_rosdep_deprecation_warning_remains_fail_closed(
    discovery_copy, tmp_path
):
    module, root, _ = discovery_copy

    def modified_warning(argv, env, timeout, network):
        result = _runner(module, [])(argv, env, timeout, network)
        if argv == ["rosdep", "--version"]:
            result["stderr"] = (
                module.ROSDEP_PKG_RESOURCES_DEPRECATION_WARNING
                + b"https://unexpected.invalid\n"
            )
        return result

    _, receipt = _prepare(module, root, tmp_path, modified_warning)
    assert receipt["status"] == "PARTIAL_FAILURE_REVIEW_REQUIRED"
    assert receipt["failure"]["kind"] == "NETWORK_FALLBACK_OBSERVED"


def test_rosdep_version_mismatch_is_partial(discovery_copy, tmp_path):
    module, root, _ = discovery_copy
    output, receipt = _prepare(
        module, root, tmp_path, _runner(module, [], version="0.25.0")
    )
    assert receipt["status"] == "PARTIAL_FAILURE_REVIEW_REQUIRED"
    assert receipt["failure"]["kind"] == "ROSDEP_VERSION_INVALID"


def test_dpkg_version_and_file_mismatch_are_partial(discovery_copy, tmp_path):
    module, root, _ = discovery_copy
    calls = []

    def bad_versions(argv, env, timeout, network):
        result = _runner(module, calls)(argv, env, timeout, network)
        if argv[:2] == ["dpkg-query", "--show"]:
            result["stdout"] = result["stdout"].replace(b"0.26.0-1", b"9.9.9")
        return result

    output, receipt = _prepare(module, root, tmp_path / "versions", bad_versions)
    assert receipt["failure"]["kind"] == "DPKG_VERSION_BINDING_INVALID"

    def bad_files(argv, env, timeout, network):
        result = _runner(module, calls)(argv, env, timeout, network)
        if argv[:2] == ["dpkg-query", "--listfiles"]:
            result["stdout"] = b"../escape\n"
        return result

    output, receipt = _prepare(module, root, tmp_path / "files", bad_files)
    assert receipt["failure"]["kind"] == "DPKG_FILE_PATH_INVALID"
    assert output.is_dir()


def test_dpkg_file_root_marker_is_normalized_and_other_dot_paths_rejected(
        fixture_module):
    package = {"name": "python3-rosdep"}
    assert fixture_module._parse_files(
        b"/.\n/usr\n/usr/bin/rosdep\n", package
    ) == ["/", "/usr", "/usr/bin/rosdep"]
    for invalid in (b"/usr/./bin\n", b"/usr/../etc\n", b"../escape\n"):
        with pytest.raises(fixture_module.PrepareError) as error:
            fixture_module._parse_files(invalid, package)
        assert error.value.kind == "DPKG_FILE_PATH_INVALID"


def test_cache_unknown_escape_duplicate_and_oversize_are_rejected(
    discovery_copy, tmp_path
):
    module, root, _ = discovery_copy

    def cache_result(line):
        def run(argv, env, timeout, network):
            result = _runner(module, [])(argv, env, timeout, network)
            if argv[0] == "find":
                result["stdout"] = line
            return result
        return run

    for index, line in enumerate(
        (
            b"../escape\tf\t1\n",
            b"x\tx\t1\n",
            b"x\tf\t01\n",
            b"x\tf\t1\nx\tf\t1\n",
        )
    ):
        output, receipt = _prepare(
            module, root, tmp_path / "cache-{}".format(index),
            cache_result(line)
        )
        assert receipt["status"] == "PARTIAL_FAILURE_REVIEW_REQUIRED"
        assert receipt["failure"]["kind"] in {
            "ROSDEP_CACHE_INVALID", "PATH_INVALID"
        }
        assert output.is_dir()


def test_command_allowlist_rejects_network_and_hostile_environment(fixture_module):
    module = fixture_module
    with pytest.raises(module.PrepareError, match="COMMAND_POLICY"):
        module._validate_offline_command(
            ["curl", "https://evil.invalid"], module.EXPECTED_NETWORK_ENV, "bad"
        )
    hostile = dict(module.EXPECTED_NETWORK_ENV, HOME="/root")
    with pytest.raises(module.PrepareError, match="COMMAND_ENV"):
        module._validate_offline_command(
            ["rosdep", "update"], hostile, "bad-env"
        )


def test_discovery_artifact_index_rejects_extra_record(discovery_copy, monkeypatch):
    module, root, value = discovery_copy
    value["artifacts"].append(dict(value["artifacts"][0]))
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="ARTIFACT_INDEX"):
        module.validate_discovery_receipt(root)


def test_output_collision_is_fail_closed(discovery_copy, tmp_path):
    module, root, _ = discovery_copy
    output = tmp_path / "collision"
    output.mkdir()
    with pytest.raises(module.PrepareError, match="collision"):
        module.prepare_rosdep(
            output, discovery_root=root, runner=_runner(module, [])
        )


def test_runner_is_required_and_no_default_network_path(discovery_copy, tmp_path):
    module, root, _ = discovery_copy
    with pytest.raises(module.PrepareError, match="RUNNER_REQUIRED"):
        module.prepare_rosdep(tmp_path / "no-runner", discovery_root=root)


def test_profile_path_is_pinned_exactly(discovery_copy, tmp_path):
    """A discovery packet cannot be rebound to another profile path."""
    module, root, _ = discovery_copy
    with pytest.raises(module.PrepareError, match="PROFILE_NOT_PINNED"):
        module.validate_discovery_receipt(root, profile_path=tmp_path / "profile")


def test_discovery_binding_separates_logical_profile_from_transport_path(
    discovery_copy, monkeypatch, tmp_path
):
    """A staged container profile must not replace the producer identity."""
    module, root, value = discovery_copy
    logical = str(module.REPOSITORY_PROFILE_PATH)
    transport = tmp_path / "bundle" / "consumer-profile.json"
    monkeypatch.setattr(module, "PROFILE_PATH", transport)

    binding = module._binding_projection(value, root)

    assert binding["profile"] == {
        "path": logical,
        "sha256": module.DISCOVERY_PROFILE_SHA256,
    }
    assert binding["profile"]["path"] != str(transport)


def test_discovery_binding_rejects_transport_profile_cross_wire(
    discovery_copy, monkeypatch, tmp_path
):
    """The receipt's producer profile cannot be rebound to a staged path."""
    module, root, value = discovery_copy
    logical = str(module.REPOSITORY_PROFILE_PATH)
    transport = tmp_path / "bundle" / "consumer-profile.json"
    altered = copy.deepcopy(value)
    altered["profile"]["path"] = str(transport)
    altered["canonical_sha256"] = module.canonical_hash(altered)
    monkeypatch.setattr(
        module, "EXPECTED_DISCOVERY_CANONICAL_SHA256", altered["canonical_sha256"]
    )

    with pytest.raises(module.PrepareError, match="DISCOVERY_PROFILE_INVALID"):
        module._binding_projection(
            altered, root, logical_profile_path=logical
        )


def test_prepare_schema_rejects_transport_profile_path(discovery_copy, tmp_path):
    """The prepare schema keeps the current logical profile identity typed."""
    module, root, _ = discovery_copy
    _, receipt = _prepare(module, root, tmp_path, _runner(module, []))
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    validator = jsonschema.Draft202012Validator(schema)
    altered = copy.deepcopy(receipt)
    altered["consumer_profile"]["path"] = str(
        tmp_path / "bundle" / "consumer-profile.json"
    )

    with pytest.raises(jsonschema.ValidationError):
        validator.validate(altered)


@pytest.mark.parametrize(
    "location,field,replacement",
    [
        ("image", "digest", "sha256:" + "0" * 64),
        ("image", "platform", "linux/arm64"),
        ("observed", "repo_digests", ["ros@sha256:" + "0" * 64]),
    ],
)
def test_image_identity_fields_are_exact(
    discovery_copy, monkeypatch, location, field, replacement
):
    """The discovery image digest, platform, and repository digest all bind."""
    module, root, value = discovery_copy
    if location == "observed":
        value["image"]["observed"][field] = replacement
    else:
        value["image"][field] = replacement
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="DISCOVERY_IMAGE"):
        module.validate_discovery_receipt(root)


@pytest.mark.parametrize(
    "field,replacement",
    [
        ("name", "python3-not-rosdep"),
        ("version", "9.9.9-1"),
        ("architecture", "amd64"),
        ("filename", "pool/main/evil.deb"),
        ("size_bytes", 1),
        ("sha256", "0" * 64),
        ("uri", "http://evil.invalid/rosdep.deb"),
        ("source", "untrusted-field"),
    ],
)
def test_each_expected_deb_binding_field_is_exact(
    discovery_copy, monkeypatch, field, replacement
):
    """Every selected package field is pinned to discovery-11."""
    module, root, value = discovery_copy
    value["apt"]["packages"][0][field] = replacement
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="DISCOVERY_PACKAGE"):
        module.validate_discovery_receipt(root)


def test_missing_deb_file_is_rejected(discovery_copy):
    """A receipt cannot retain a package after its sealed deb disappears."""
    module, root, value = discovery_copy
    path = root / value["apt"]["packages"][0]["deb_artifact"]["path"]
    path.unlink()
    with pytest.raises(module.PrepareError, match="ARTIFACT|DISCOVERY"):
        module.validate_discovery_receipt(root)


@pytest.mark.parametrize("field,replacement", [("bytes", 1), ("nlink", 2)])
def test_deb_descriptor_size_and_link_bindings_are_exact(
    discovery_copy, monkeypatch, field, replacement
):
    """Package artifact metadata cannot be rewritten in the receipt."""
    module, root, value = discovery_copy
    artifact = value["apt"]["packages"][0]["deb_artifact"]
    artifact[field] = replacement
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="ARTIFACT|DISCOVERY_PACKAGE"):
        module.validate_discovery_receipt(root)


def test_deb_artifact_path_binding_is_exact(discovery_copy, monkeypatch):
    """A package descriptor cannot point at a different relative artifact."""
    module, root, value = discovery_copy
    value["apt"]["packages"][0]["deb_artifact"]["path"] = (
        "artifacts/apt/debs/other.deb"
    )
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="ARTIFACT|DISCOVERY_PACKAGE"):
        module.validate_discovery_receipt(root)


@pytest.mark.parametrize(
    "field,replacement",
    [
        ("requested_url", "https://codeload.github.com/ros/rosdistro/tar.gz/master"),
        ("final_url", "https://evil.invalid/rosdistro.tar.gz"),
        ("redirects", ["https://evil.invalid/redirect"]),
        ("attempts", 2),
    ],
)
def test_archive_url_redirect_and_attempt_policy_is_exact(
    discovery_copy, monkeypatch, field, replacement
):
    """Mutable refs, redirects, and retry counts are not accepted."""
    module, root, value = discovery_copy
    value["rosdistro"]["archive"][field] = replacement
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="DISCOVERY_ARCHIVE"):
        module.validate_discovery_receipt(root)


@pytest.mark.parametrize(
    "field,replacement",
    [
        ("relative_path", "rosdep/other.yaml"),
        ("bytes", 1),
        ("sha256", "0" * 64),
        ("archive_member", "rosdistro-master/rosdep/base.yaml"),
    ],
)
def test_all_selected_file_identity_fields_are_exact(
    discovery_copy, monkeypatch, field, replacement
):
    """Each of the eight commit-addressed selected files is fully bound."""
    module, root, value = discovery_copy
    value["rosdistro"]["archive_details"]["selected_files"][0][field] = (
        replacement
    )
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="DISCOVERY_FILE"):
        module.validate_discovery_receipt(root)


def test_selected_file_missing_and_sidecar_substitution_are_rejected(
    discovery_copy, monkeypatch
):
    """Selected-file absence and descriptor substitution both fail closed."""
    module, root, value = discovery_copy
    selected = value["rosdistro"]["archive_details"]["selected_files"][0]
    path = root / selected["artifact"]["path"]
    path.unlink()
    with pytest.raises(module.PrepareError, match="ARTIFACT|DISCOVERY_FILE"):
        module.validate_discovery_receipt(root)

    source = root / "artifacts/rosdistro/files/index-v4.yaml"
    source.write_bytes(b"not the selected file")
    source.chmod(0o444)
    selected["artifact"]["sidecar"] = selected["artifact"]["path"] + ".bad"
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="FILE|ARTIFACT"):
        module.validate_discovery_receipt(root)


@pytest.mark.parametrize(
    "field,replacement",
    [
        (
            "commit_url",
            "https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml",
        ),
        (
            "requested_url",
            "https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml",
        ),
        ("type", "yaml-unknown"),
        ("line", 999),
    ],
)
def test_source_record_provenance_is_commit_addressed(
    discovery_copy, monkeypatch, field, replacement
):
    """Source records must retain exact commit URLs, type, and provenance line."""
    module, root, value = discovery_copy
    value["rosdistro"]["archive_details"]["source_records"][0][field] = (
        replacement
    )
    _sync_fixture_receipt(module, root, value, monkeypatch)
    with pytest.raises(module.PrepareError, match="DISCOVERY_(SOURCE|ROSDISTRO)"):
        module.validate_discovery_receipt(root)


def test_source_bindings_preserve_exact_record_order_and_distribution(
    discovery_copy,
):
    """The local source list keeps pinned YAML rules and excludes gbpdistro."""
    module, root, _ = discovery_copy
    binding = module.validate_discovery_receipt(root)
    text, projections = module._source_bindings(binding)
    lines = text.splitlines()
    enabled = [
        record for record in binding["source_records"]
        if record["type"] in module.ENABLED_SOURCE_TYPES
    ]
    assert len(lines) == len(enabled)
    assert len(projections) == len(lines)
    for line, record in zip(lines, enabled):
        fields = line.split()
        assert fields[0] == record["type"]
        assert fields[1].startswith("file:///opt/registration-plugin/rosdistro/files/")
        if "distribution" in record:
            assert fields[2] == record["distribution"]
        else:
            assert len(fields) == 2
    assert "gbpdistro" not in text


@pytest.mark.parametrize(
    "command,hostile_environment",
    [
        (["rosdep", "update"], True),
        (["find", "/tmp", "-exec", "curl", "{}", ";"], False),
    ],
)
def test_offline_command_policy_rejects_host_or_network_escape(
    fixture_module, command, hostile_environment
):
    """The runner allowlist rejects hostile environments and command tokens."""
    module = fixture_module
    if hostile_environment:
        environment = dict(module.EXPECTED_NETWORK_ENV, HOME="/root")
    else:
        environment = module.EXPECTED_NETWORK_ENV
    with pytest.raises(module.PrepareError, match="COMMAND_(ENV|POLICY)"):
        module._validate_offline_command(command, environment, "adversarial")


@pytest.mark.parametrize(
    "variable,value",
    [
        ("HOME", "/root"),
        ("ROS_HOME", "/tmp/ros-home"),
        ("ROSDEP_SOURCE_PATH", "/tmp/sources"),
        ("ROSDEP_CACHE_DIR", "/tmp/cache"),
        ("NO_PROXY", ""),
    ],
)
def test_each_network_environment_binding_is_exact(fixture_module, variable, value):
    """Every fixed offline environment variable is independently pinned."""
    module = fixture_module
    environment = dict(module.EXPECTED_NETWORK_ENV)
    environment[variable] = value
    with pytest.raises(module.PrepareError, match="COMMAND_ENV_INVALID"):
        module._validate_offline_command(
            ["rosdep", "update"], environment, "environment"
        )


@pytest.mark.parametrize(
    "result",
    [
        None,
        {},
        {"returncode": 0, "stdout": b"", "stderr": b"", "network_attempts": ["x"]},
        {"returncode": "0", "stdout": b"", "stderr": b"", "network_attempts": []},
        {"returncode": 0, "stdout": 1, "stderr": b"", "network_attempts": []},
    ],
)
def test_runner_result_shape_is_strict(fixture_module, result):
    """Malformed injected runner output cannot become evidence."""
    with pytest.raises(fixture_module.PrepareError, match="RUNNER_RESULT_INVALID"):
        fixture_module._normalize_runner_result(result, "shape")


def test_stderr_network_fallback_becomes_partial(discovery_copy, tmp_path):
    """A URL in command stderr is also a forbidden offline fallback."""
    module, root, _ = discovery_copy

    def runner(argv, env, timeout, network):
        result = _runner(module, [])(argv, env, timeout, network)
        if argv[0] == "rosdep":
            result["stderr"] = b"fallback https://unexpected.invalid"
        return result

    output, receipt = _prepare(module, root, tmp_path, runner)
    assert receipt["status"] == "PARTIAL_FAILURE_REVIEW_REQUIRED"
    assert receipt["failure"]["kind"] == "NETWORK_FALLBACK_OBSERVED"
    assert module.validate_prepare_receipt(output, discovery_root=root) == receipt


def test_rosdep_resolve_failure_is_partial_and_reopenable(discovery_copy, tmp_path):
    """A late resolve failure preserves the sealed prepare prefix."""
    module, root, _ = discovery_copy
    output, receipt = _prepare(
        module, root, tmp_path, _runner(module, [], failure="resolve")
    )
    assert receipt["failure"]["phase"] == "rosdep_prepare"
    assert receipt["summary"]["dependency_install"] == "PASS"
    assert receipt["summary"]["rosdep_prepare"] == "FAILED"
    assert module.validate_prepare_receipt(output, discovery_root=root)["failure"] == (
        receipt["failure"]
    )


def test_runner_exception_is_truthful_partial(discovery_copy, tmp_path):
    """An injected runner exception is represented as a failed phase."""
    module, root, _ = discovery_copy

    def runner(argv, env, timeout, network):
        if argv[:2] == ["dpkg-query", "--show"]:
            raise RuntimeError("synthetic runner crash")
        return _runner(module, [])(argv, env, timeout, network)

    output, receipt = _prepare(module, root, tmp_path, runner)
    assert output.is_dir()
    assert receipt["status"] == "PARTIAL_FAILURE_REVIEW_REQUIRED"
    assert receipt["failure"]["kind"] == "RUNNER_FAILED"
    assert module.validate_prepare_receipt(output, discovery_root=root) == receipt


def test_prepare_receipt_artifact_tamper_is_rejected(discovery_copy, tmp_path):
    """Reopen validation detects mutation after a successful prepare."""
    module, root, _ = discovery_copy
    output, receipt = _prepare(module, root, tmp_path, _runner(module, []))
    artifact = output / receipt["phases"][1]["source_list"]["path"]
    artifact.chmod(0o600)
    artifact.write_bytes(artifact.read_bytes() + b"tamper")
    artifact.chmod(0o444)
    with pytest.raises(module.PrepareError, match="PREPARE|SIDECAR|ARTIFACT"):
        module.validate_prepare_receipt(output, discovery_root=root)


def test_prepare_receipt_extra_output_is_rejected(discovery_copy, tmp_path):
    """Unrecorded files are not silently ignored by output readback."""
    module, root, _ = discovery_copy
    output, _ = _prepare(module, root, tmp_path, _runner(module, []))
    extra = output / "artifacts/rosdep/cache/unrecorded"
    extra.write_bytes(b"extra")
    extra.chmod(0o444)
    with pytest.raises(module.PrepareError, match="ALLOWLIST"):
        module.validate_prepare_receipt(output, discovery_root=root)


def test_prepare_root_symlink_and_file_collisions_are_rejected(
    discovery_copy, tmp_path
):
    """Fresh output roots reject symlink and non-directory collisions."""
    module, root, _ = discovery_copy
    symlink = tmp_path / "symlink"
    symlink.symlink_to(tmp_path)
    with pytest.raises(module.PrepareError, match="PATH|COLLISION"):
        module.prepare_rosdep(symlink, discovery_root=root, runner=_runner(module, []))
    regular = tmp_path / "regular"
    regular.write_bytes(b"occupied")
    with pytest.raises(module.PrepareError, match="COLLISION"):
        module.prepare_rosdep(regular, discovery_root=root, runner=_runner(module, []))


def _complete_precreated_prepare_root(module, tmp_path):
    """Build the exact host-owned tree expected by production prepare."""
    root = tmp_path / "precreated"
    root.mkdir(mode=0o700)
    for relative in sorted(
        module.PREPARE_PRECREATED_DIRECTORY_RELATIVES,
        key=lambda item: (item.count("/"), item),
    ):
        path = root / relative
        path.mkdir(mode=0o700)
        path.chmod(0o700)
    for name in sorted(module.PREPARE_INPUT_FILE_NAMES):
        path = root / "input" / name
        payload = ("fixture:" + name).encode("ascii")
        path.write_bytes(payload)
        path.chmod(0o444)
        sidecar = Path(str(path) + ".sha256")
        sidecar.write_text(
            module._sha256(payload) + "  " + path.name + "\n",
            encoding="ascii",
        )
        sidecar.chmod(0o444)
    identity = module._root_descriptor(root)
    return root, identity


def test_precreated_tree_rejects_runtime_created_nested_directory(
    fixture_module, tmp_path
):
    module = fixture_module
    root, identity = _complete_precreated_prepare_root(module, tmp_path)
    snapshot = module._precreated_directory_snapshot(root, identity)
    child = root / "work/rosdistro/files/created-by-runtime"
    child.mkdir(mode=0o700)
    with pytest.raises(module.PrepareError, match="UNEXPECTED"):
        module._verify_precreated_directory_snapshot(root, snapshot, identity)


def test_precreated_tree_is_complete_and_prepare_never_calls_mkdir(
    discovery_copy, tmp_path, monkeypatch
):
    module, discovery_root, _ = discovery_copy
    root, _ = _complete_precreated_prepare_root(module, tmp_path)
    monkeypatch.setattr(
        module, "_mkdir",
        lambda *_args, **_kwargs: pytest.fail("precreated prepare called _mkdir"),
    )
    receipt = module.prepare_rosdep(
        root,
        discovery_root=discovery_root,
        runner=_runner(module, []),
        clock=iter((100, 101)).__next__,
        precreated=True,
    )
    assert receipt["status"] == "REVIEW_REQUIRED"
    assert module.validate_prepare_receipt(
        root, discovery_root=discovery_root, allow_host_bindings=True
    ) == receipt


def test_cli_help_is_available_without_execution(fixture_module, capsys):
    """The validator CLI exposes help without opening a runner or network."""
    with pytest.raises(SystemExit) as exc:
        fixture_module._parser().parse_args(["--help"])
    assert exc.value.code == 0
    assert "validate-discovery" in capsys.readouterr().out
