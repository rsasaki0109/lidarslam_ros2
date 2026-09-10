#!/usr/bin/env python3
"""Synthetic adversarial tests for the r3 per-repository promotion gate."""

from __future__ import annotations

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


PROMOTION = _load(R3 / "apt_signature_promotion.py", "r3_signature_promotion_test")


def _repo(prefix: str) -> dict[str, str]:
    return {
        "url": f"https://{prefix}.example.invalid/jazzy",
        "release_url": f"https://{prefix}.example.invalid/jazzy/InRelease",
        "release_path": f"lists/{prefix}-InRelease",
        "release_sha256": "a" * 64,
        "packages_url": f"https://{prefix}.example.invalid/jazzy/main/Packages",
        "packages_path": f"lists/{prefix}-Packages",
        "packages_sha256": "b" * 64,
    }


def _ledger(repositories: list[dict[str, str]]) -> dict[str, object]:
    entries = []
    for index, repository in enumerate(repositories):
        entries.append({
            "name": f"pkg-{index}", "version": "1.0-1", "architecture": "amd64",
            "filename": f"pkg-{index}_1.0-1_amd64.deb", "url": repository["url"] + "/pool/pkg.deb",
            "bytes": 1, "sha256": "c" * 64,
            "repository_url": repository["url"], "release_url": repository["release_url"],
            "release_sha256": repository["release_sha256"], "packages_url": repository["packages_url"],
            "packages_path": repository["packages_path"], "packages_sha256": repository["packages_sha256"],
        })
    return {"entries": entries}


def _entry(repository: dict[str, str], index: int = 0, *, root: str | None = None) -> dict[str, object]:
    root = root or f"/tmp/receipt-{index}"
    return {
        "repository": repository,
        "receipt_root": root,
        "receipt_file_sha256": "d" * 64,
        "plan_path": f"/tmp/plan-{index}.json",
        "plan_file_sha256": "e" * 64,
        "plan_identity_sha256": "f" * 64,
        "root_inputs": f"/tmp/inputs-{index}",
        "deb_bindings": [{"name": f"pkg-{index}", "version": "1.0-1",
                          "architecture": "amd64", "path": f"debs/pkg-{index}.deb"}],
    }


def _index(path: Path, entries: list[dict[str, object]], proposal: str = "1" * 64) -> Path:
    value: dict[str, object] = {
        "schema": PROMOTION.INDEX_SCHEMA, "schema_version": 1,
        "status": "REVIEW_REQUIRED", "benchmark_eligible": False,
        "proposal_sha256": proposal, "entries": entries,
    }
    value["canonical_sha256"] = PROMOTION.canonical_hash(value)
    path.write_bytes(PROMOTION.canonical_bytes(value) + b"\n")
    return path


def test_index_schema_is_valid_and_nonpromoting():
    schema = json.loads((R3 / "apt_signature_receipt_index.schema.json").read_text())
    Draft202012Validator.check_schema(schema)
    repository = _repo("one")
    value = {
        "schema": PROMOTION.INDEX_SCHEMA, "schema_version": 1,
        "status": "REVIEW_REQUIRED", "benchmark_eligible": False,
        "proposal_sha256": "1" * 64, "entries": [_entry(repository)],
    }
    value["canonical_sha256"] = PROMOTION.canonical_hash(value)
    Draft202012Validator(schema).validate(value)


@pytest.mark.parametrize("mode,pattern", [
    ("missing", "missing|extra"),
    ("duplicate", "duplicate"),
    ("extra", "extra|cross-campaign"),
])
def test_missing_duplicate_extra_repository_receipts_fail_closed(tmp_path, mode, pattern):
    repositories = [_repo("one"), _repo("two")]
    entries = [_entry(repositories[0], 0)]
    if mode == "missing":
        entries = []
    elif mode == "duplicate":
        entries = [_entry(repositories[0], 0), _entry(repositories[0], 1)]
    else:
        entries = [_entry(repositories[0], 0), _entry(_repo("other"), 1)]
    index_path = _index(tmp_path / "index.json", entries)
    with pytest.raises(PROMOTION.SignaturePromotionError, match=pattern):
        PROMOTION.validate_receipt_index(index_path, expected_repositories=repositories,
                                         ledger=_ledger(repositories), proposal_sha256="1" * 64)


def test_receipt_root_replay_across_repositories_is_rejected(tmp_path):
    repositories = [_repo("one"), _repo("two")]
    entries = [_entry(repositories[0], 0, root="/tmp/shared-receipt"),
               _entry(repositories[1], 1, root="/tmp/shared-receipt")]
    index_path = _index(tmp_path / "index.json", entries)
    with pytest.raises(PROMOTION.SignaturePromotionError, match="replayed"):
        PROMOTION.validate_receipt_index(index_path, expected_repositories=repositories,
                                         ledger=_ledger(repositories), proposal_sha256="1" * 64)


def test_fixture_only_or_self_rehashed_receipt_can_never_promote(tmp_path):
    root = tmp_path / "receipt"
    root.mkdir()
    receipt = {
        "schema": "glim_clean_room_r3_apt_signature_executor_v1",
        "candidate_status": "IMPLEMENTED_NOT_RUNTIME_VALIDATED",
        "signature_runtime": "NOT_RUN", "execution_mode": "INJECTED_FIXTURE_ONLY",
        "promotion": "FORBIDDEN", "benchmark_eligible": False,
        "canonical_sha256": "",
    }
    receipt["canonical_sha256"] = PROMOTION.canonical_hash(receipt, "canonical_sha256")
    (root / PROMOTION.RECEIPT_NAME).write_bytes(PROMOTION.canonical_bytes(receipt) + b"\n")
    with pytest.raises(PROMOTION.SignaturePromotionError, match="fixture-only|NOT_RUNTIME"):
        PROMOTION._validate_promotable_receipt(root)
