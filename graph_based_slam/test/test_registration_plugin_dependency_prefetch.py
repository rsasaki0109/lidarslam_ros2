#!/usr/bin/env python3
"""Synthetic tests for the host prefetch/runner handoff contract."""

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[2]
PREFETCH = REPO / "scripts/registration_plugin_dependency_prefetch.py"
RUNNER = REPO / "scripts/run_registration_plugin_release_matrix.py"
WORKFLOW = REPO / ".github/workflows/main.yml"


def _load():
    spec = importlib.util.spec_from_file_location("dependency_prefetch", str(PREFETCH))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def prefetch():
    return _load()


def _dependency(payload=b"synthetic archive\n"):
    digest = hashlib.sha256(payload).hexdigest()
    return {
        "name": "fast_gicp",
        "official_url": "https://github.com/SMRT-AIST/fast_gicp.git",
        "commit": "a" * 40,
        "archive_url": "https://codeload.github.com/SMRT-AIST/fast_gicp/tar.gz/" + "a" * 40,
        "archive_sha256": digest,
        "archive_top_level": "fast_gicp-" + "a" * 40,
        "source_tree_sha256": "b" * 64,
        "license_path": "LICENSE",
        "license_sha256": "c" * 64,
    }


def _seed_environment(prefetch, root):
    root.mkdir()
    payload = json.dumps({"schema": "synthetic"}, sort_keys=True).encode("utf-8")
    path = root / prefetch.DEPENDENCY_ENVIRONMENT_NAME
    path.write_bytes(payload)
    sidecar = Path(str(path) + ".sha256")
    sidecar.write_text("{}  {}\n".format(hashlib.sha256(payload).hexdigest(), path.name), encoding="ascii")
    path.chmod(0o444)
    sidecar.chmod(0o444)


def _fetcher(payload, calls):
    def fetch(url, maximum):
        calls.append(url)
        assert len(payload) <= maximum
        return {"payload": payload, "final_url": url, "redirects": [], "status": 200,
                "headers": {"content-type": "application/gzip"}}
    return fetch


def test_prefetch_fetches_then_seals_and_reopens_all_bytes(prefetch, tmp_path):
    payload = b"synthetic archive\n"
    dependency = _dependency(payload)
    root = tmp_path / "registration-plugin-release-jazzy-present"
    _seed_environment(prefetch, root)
    calls = []
    result = prefetch.prefetch_pinned_archives(
        [dependency], root, "jazzy", "sha256:" + "d" * 64,
        authority="HOST_PROMOTION", fetcher=_fetcher(payload, calls))
    assert calls == [dependency["archive_url"]]
    assert result["status"] == "PASS"
    assert result["records"][0]["archive_sha256"] == hashlib.sha256(payload).hexdigest()
    reopened = prefetch.validate_prefetch_root(
        root, [dependency], "jazzy", "sha256:" + "d" * 64,
        authority="HOST_PROMOTION")
    assert reopened["receipt_sha256"] == result["receipt_sha256"]


def test_prefetch_rejects_missing_or_extra_root_entries(prefetch, tmp_path):
    payload = b"synthetic archive\n"
    dependency = _dependency(payload)
    root = tmp_path / "registration-plugin-release-jazzy-present"
    _seed_environment(prefetch, root)
    (root / "unexpected.txt").write_bytes(b"x")
    (root / "unexpected.txt").chmod(0o444)
    with pytest.raises(prefetch.PrefetchError) as error:
        prefetch.prefetch_pinned_archives(
            [dependency], root, "jazzy", "sha256:" + "d" * 64,
            fetcher=_fetcher(payload, []))
    assert error.value.kind == "PREFETCH_ROOT_ALLOWLIST_MISMATCH"


def test_prefetch_rejects_redirect_to_different_host(prefetch, tmp_path):
    payload = b"synthetic archive\n"
    dependency = _dependency(payload)
    root = tmp_path / "registration-plugin-release-jazzy-present"
    _seed_environment(prefetch, root)

    def fetch(url, maximum):
        del maximum
        return {"payload": payload, "final_url": "https://evil.example/archive",
                "redirects": ["https://evil.example/archive"], "status": 200, "headers": {}}

    with pytest.raises(prefetch.PrefetchError) as error:
        prefetch.prefetch_pinned_archives([dependency], root, "jazzy", "sha256:" + "d" * 64,
                                          fetcher=fetch)
    assert error.value.kind == "PREFETCH_REDIRECT_HOST_MISMATCH"


def test_prefetch_rejects_archive_mutation_and_no_retry_overwrite(prefetch, tmp_path):
    payload = b"synthetic archive\n"
    dependency = _dependency(payload)
    root = tmp_path / "registration-plugin-release-jazzy-present"
    _seed_environment(prefetch, root)
    prefetch.prefetch_pinned_archives(
        [dependency], root, "jazzy", "sha256:" + "d" * 64,
        fetcher=_fetcher(payload, []))
    archive = root / "prefetch/archives/fast_gicp.tar.gz"
    archive.chmod(0o644)
    archive.write_bytes(b"tampered")
    archive.chmod(0o444)
    with pytest.raises(prefetch.PrefetchError):
        prefetch.validate_prefetch_root(root, [dependency], "jazzy", "sha256:" + "d" * 64)
    with pytest.raises(prefetch.PrefetchError) as error:
        prefetch.prefetch_pinned_archives(
            [dependency], root, "jazzy", "sha256:" + "d" * 64,
            fetcher=_fetcher(payload, []))
    assert error.value.kind == "PREFETCH_ROOT_ALLOWLIST_MISMATCH"


def test_prefetch_rejects_hardlink_and_symlink_entries(prefetch, tmp_path):
    payload = b"synthetic archive\n"
    dependency = _dependency(payload)
    root = tmp_path / "registration-plugin-release-jazzy-present"
    _seed_environment(prefetch, root)
    (root / "prefetch").mkdir()
    (root / "prefetch/archives").mkdir()
    source = root / "source"
    source.write_bytes(payload)
    source.chmod(0o444)
    (root / "prefetch/archives/fast_gicp.tar.gz").symlink_to(source)
    with pytest.raises(prefetch.PrefetchError) as error:
        prefetch.validate_prefetch_root(root, [dependency], "jazzy", "sha256:" + "d" * 64)
    assert error.value.kind == "PREFETCH_LINK_FORBIDDEN"


def test_absent_contract_has_no_prefetch_allowlist(prefetch):
    names = prefetch.expected_file_paths([], include_prefetch=False)
    assert names == frozenset({
        "dependency_environment.receipt.json",
        "dependency_environment.receipt.json.sha256",
    })
    assert prefetch.PREFETCH_RECEIPT_NAME not in names


def test_prefetch_rejects_non_https_or_query_urls(prefetch, tmp_path):
    payload = b"synthetic archive\n"
    dependency = _dependency(payload)
    dependency["archive_url"] = "https://codeload.github.com/archive?moving=1"
    root = tmp_path / "registration-plugin-release-jazzy-present"
    _seed_environment(prefetch, root)
    with pytest.raises(prefetch.PrefetchError) as error:
        prefetch.prefetch_pinned_archives([dependency], root, "jazzy", "sha256:" + "d" * 64,
                                          fetcher=_fetcher(payload, []))
    assert error.value.kind == "PREFETCH_URL_INVALID"


@pytest.mark.parametrize("headers,kind", [
    ({"authorization": "Bearer secret"}, "PREFETCH_RESPONSE_HEADER_FORBIDDEN"),
    ({"x-forwarded-for": "10.0.0.1"}, "PREFETCH_RESPONSE_HEADER_FORBIDDEN"),
    ({"Content-Type": "application/gzip"}, "PREFETCH_RESPONSE_HEADER_FORBIDDEN"),
    ({"content-type": "ok\r\nAuthorization: leaked"}, "PREFETCH_RESPONSE_HEADER_INVALID"),
])
def test_response_headers_are_allowlisted_and_control_safe(prefetch, headers, kind):
    with pytest.raises(prefetch.PrefetchError) as error:
        prefetch._normalize_fetch_result({
            "payload": b"archive",
            "final_url": "https://codeload.github.com/archive",
            "redirects": [], "status": 200, "headers": headers,
        }, "https://codeload.github.com/archive")
    assert error.value.kind == kind


def test_response_headers_normalize_only_safe_metadata(prefetch):
    result = prefetch._normalize_fetch_result({
        "payload": b"archive",
        "final_url": "https://codeload.github.com/archive",
        "redirects": [], "status": 200,
        "headers": {"etag": "abc", "content-type": "application/gzip"},
    }, "https://codeload.github.com/archive")
    assert result["headers"] == {
        "content-type": "application/gzip", "etag": "abc",
    }


def test_in_container_runner_has_no_archive_fetch_callsite(prefetch):
    del prefetch
    text = RUNNER.read_text(encoding="utf-8")
    assert "urlopen(" not in text
    assert "urlretrieve(" not in text
    assert '["curl"' not in text
    assert "prefetched_official_archive_only" in text


def test_in_container_present_materialization_requires_sealed_binding(prefetch, tmp_path):
    del prefetch
    spec = importlib.util.spec_from_file_location("release_runner", str(RUNNER))
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    with pytest.raises(runner.ReleaseGateError) as error:
        runner._materialize_pinned_dependencies(
            [_dependency()], tmp_path / "work", {}, tmp_path / "logs")
    assert error.value.kind == "PREFETCH_BINDING_MISSING"


def test_ci_functional_path_seeds_prefetch_and_cannot_be_promotion_authority():
    text = WORKFLOW.read_text(encoding="utf-8")
    assert "capture_registration_plugin_dependency_environment.py" in text
    assert "registration_plugin_dependency_prefetch.py" in text
    assert "FUNCTIONAL_CI_NON_PROMOTING" in text
    assert "REGISTRATION_PLUGIN_BUILD_TEST_NETWORK_CONNECTED: \"true\"" in text
    assert "The release runner" in text
    assert "summarize_registration_plugin_release_matrix.py" not in text.split(
        "registration-plugin-release-matrix-summary:", 1
    )[1].split("release-readiness:", 1)[0]
