#!/usr/bin/env python3
"""Offline authorizer for a sealed NTU VIRAL acquisition candidate.

Stage B consumes only a sealed Stage A candidate.  It reopens every byte and
receipt sidecar, verifies the preregistered role/source contract, and emits a
deterministic *proposal*.  It never edits the profile or turns a candidate
receipt into a reviewed pin; a separate human/reviewer action is required.
Ground-truth bytes are hashed as opaque role-separated inputs and no GT path is
placed in the runner input manifest.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path, PurePosixPath
from typing import Any, Mapping
from urllib.parse import urlparse

import yaml

from lidarslam_benchmark_tools.ntu_viral_acquisition import (
    CALIBRATION_ROLES,
    CONTRACT_VERSION,
    NtuAcquisitionError,
    ROOT,
    SEQUENCES,
    _archive_tree_hash,
    _atomic_write_new,
    _canonical_json,
    _iter_role_specs,
    _regular_nosymlink,
    _safe_relative,
    _validate_final_url,
    _validate_url_spec,
    canonical_sha256,
    inspect_safe_archive,
    load_context,
    sha256_file,
)


MANIFEST_KIND = "ntu_viral_immutable_pin_manifest"
MANIFEST_VERSION = 1


def _read_json(path: Path, label: str) -> Mapping[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise NtuAcquisitionError("READ_FAILED", f"cannot read {label}: {exc}") from exc
    if not isinstance(value, Mapping):
        raise NtuAcquisitionError("MALFORMED", f"{label} must be a JSON object")
    return value


def _verify_sidecar(path: Path) -> str:
    _regular_nosymlink(path, str(path))
    sidecar = path.with_name(path.name + ".sha256")
    _regular_nosymlink(sidecar, str(sidecar))
    fields = sidecar.read_text(encoding="ascii").strip().split()
    if len(fields) != 2 or fields[1] != path.name:
        raise NtuAcquisitionError("SIDECAR_MISMATCH", f"invalid sidecar for {path.name}")
    digest = sha256_file(path)
    if fields[0].lower() != digest:
        raise NtuAcquisitionError("SIDECAR_MISMATCH", f"sidecar SHA-256 mismatch for {path.name}")
    return digest


def _root_relative(root: Path, evidence_root: Path) -> str:
    resolved_root = root.resolve()
    resolved_evidence = evidence_root.resolve()
    try:
        relative = resolved_root.relative_to(resolved_evidence)
    except ValueError as exc:
        raise NtuAcquisitionError("ROOT_SCOPE", "candidate root is outside preregistered evidence root") from exc
    if not relative.parts or any(part in (".", "..") for part in relative.parts):
        raise NtuAcquisitionError("ROOT_SCOPE", "candidate root must be below evidence root")
    return relative.as_posix()


def _verify_redirects(artifact: Mapping[str, Any], spec: Mapping[str, Any], label: str) -> None:
    requested, _, _, _ = _validate_url_spec(spec, label)
    if artifact.get("requested_url") != requested:
        raise NtuAcquisitionError("SOURCE_DRIFT", f"{label} requested URL differs from selection")
    final = _validate_final_url(artifact, requested, list(spec["allowed_hosts"]), label)
    chain = artifact.get("redirect_chain")
    if not isinstance(chain, list) or not chain or str(chain[0]) != requested:
        raise NtuAcquisitionError("REDIRECT_METADATA", f"{label} redirect chain is not anchored to requested URL")
    if artifact.get("final_url") != final:
        raise NtuAcquisitionError("REDIRECT_METADATA", f"{label} final URL is inconsistent")
    for item in chain:
        parsed = urlparse(str(item))
        if parsed.username or parsed.password or parsed.fragment:
            raise NtuAcquisitionError("REDIRECT_POLICY", f"{label} redirect contains unsafe URL data")


def _verify_artifact(root: Path, label: str, artifact: Mapping[str, Any],
                     spec: Mapping[str, Any], expected_relative: str) -> dict[str, Any]:
    _, filename, archive, expected_members = _validate_url_spec(spec, label)
    relative = _safe_relative(artifact.get("relative_path"), f"{label}.relative_path")
    if relative != expected_relative or PurePosixPath(relative).name != filename:
        raise NtuAcquisitionError("ROLE_PATH", f"{label} path does not match the preregistered role")
    path = root / Path(relative)
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError as exc:
        raise NtuAcquisitionError("ROOT_SCOPE", f"{label} resolves outside candidate root") from exc
    info = _regular_nosymlink(path, label)
    size = artifact.get("size_bytes")
    digest = artifact.get("sha256")
    if not isinstance(size, int) or size <= 0 or size != info.st_size:
        raise NtuAcquisitionError("HASH_MISMATCH", f"{label} size does not match receipt")
    if not isinstance(digest, str) or len(digest) != 64 or sha256_file(path) != digest.lower():
        raise NtuAcquisitionError("HASH_MISMATCH", f"{label} SHA-256 does not match receipt")
    _verify_redirects(artifact, spec, label)
    result: dict[str, Any] = {
        "role": artifact.get("role"), "sequence_id": artifact.get("sequence_id"),
        "relative_path": relative, "sha256": digest.lower(), "size_bytes": size,
        "hash_kind": "sha256_file_v1",
    }
    if archive:
        inventory = inspect_safe_archive(path, expected_members)
        receipt_inventory = artifact.get("archive")
        if not isinstance(receipt_inventory, Mapping) or receipt_inventory.get("tree_sha256") != inventory["tree_sha256"]:
            raise NtuAcquisitionError("HASH_MISMATCH", f"{label} archive inventory differs from receipt")
        result["archive"] = inventory
    else:
        tree = canonical_sha256({"path": relative, "size_bytes": size, "sha256": digest.lower()})
        if artifact.get("tree_sha256") != tree:
            raise NtuAcquisitionError("HASH_MISMATCH", f"{label} tree SHA-256 differs from receipt")
        result["tree_sha256"] = tree
    return result


def _load_candidate_receipt(path: Path) -> tuple[Mapping[str, Any], str]:
    receipt_sha = _verify_sidecar(path)
    receipt = _read_json(path, "candidate receipt")
    if receipt.get("receipt_kind") != "ntu_viral_candidate_acquisition" or receipt.get("contract_version") != CONTRACT_VERSION:
        raise NtuAcquisitionError("RECEIPT_IDENTITY", "candidate receipt contract identity is invalid")
    if receipt.get("status") != "PASS":
        raise NtuAcquisitionError("CANDIDATE_NOT_PASS", "only a sealed PASS candidate can be proposed")
    seal = receipt.get("candidate_seal")
    if not isinstance(seal, Mapping) or seal.get("sealed") is not True or seal.get("review_status") != "CANDIDATE_ONLY" or seal.get("profile_edit_performed") is not False:
        raise NtuAcquisitionError("RECEIPT_SEAL", "candidate receipt is not an immutable candidate-only seal")
    retry = receipt.get("retry_policy")
    if not isinstance(retry, Mapping) or any(retry.get(key) is not False for key in (
            "candidate_root_reuse", "overwrite", "partial_download_reuse", "retry_after_failure")):
        raise NtuAcquisitionError("RETRY_POLICY", "candidate retry policy is not fail-closed")
    return receipt, receipt_sha


def _assert_runner_separation(receipt: Mapping[str, Any], artifacts: list[dict[str, Any]]) -> None:
    manifest = receipt.get("runner_input_manifest")
    if not isinstance(manifest, Mapping) or manifest.get("ground_truth_paths_exposed") is not False:
        raise NtuAcquisitionError("GT_SEPARATION", "candidate runner manifest exposes ground truth")
    if manifest.get("ground_truth_relative_paths") != []:
        raise NtuAcquisitionError("GT_SEPARATION", "candidate runner manifest contains ground-truth paths")
    expected_inputs = sorted(item["relative_path"] for item in artifacts if item["role"] == "sequence_archive")
    if manifest.get("input_relative_paths") != expected_inputs:
        raise NtuAcquisitionError("RUNNER_MANIFEST", "runner input manifest is not exactly the input role set")
    if any("ground_truth" in str(value).lower() or "calib" in str(value).lower()
           for value in manifest.get("input_relative_paths", [])):
        raise NtuAcquisitionError("GT_SEPARATION", "runner input manifest contains a non-input role path")


def propose_pin_manifest(selection_path: Path, profile_path: Path, candidate_root: Path,
                         output_path: Path, *, candidate_receipt: Path | None = None,
                         evidence_root: Path | None = None, root: Path = ROOT) -> dict[str, Any]:
    """Revalidate a candidate and write a deterministic review-only proposal."""
    context = load_context(selection_path, profile_path, root=root)
    evidence = evidence_root or Path(str(context.mount_spec["path"]))
    if candidate_root.is_symlink() or not candidate_root.is_dir():
        raise NtuAcquisitionError("ROOT_PATH", "candidate root must be a regular directory")
    candidate_relative = _root_relative(candidate_root, evidence)
    receipt_path = candidate_receipt or candidate_root / "candidate_receipt.json"
    if receipt_path.resolve() != (candidate_root / "candidate_receipt.json").resolve():
        raise NtuAcquisitionError("RECEIPT_SCOPE", "candidate receipt must be inside the candidate root")
    receipt, receipt_sha = _load_candidate_receipt(receipt_path)
    if receipt.get("selection_id") != context.selection_id or receipt.get("selection_sha256") != context.selection_sha256:
        raise NtuAcquisitionError("SELECTION_DRIFT", "candidate receipt selection identity differs from selection")
    if receipt.get("profile_sha256") != context.profile_sha256:
        raise NtuAcquisitionError("PROFILE_DRIFT", "candidate receipt profile identity differs from profile")
    if receipt.get("candidate_root_relative") != candidate_relative:
        raise NtuAcquisitionError("ROOT_DRIFT", "candidate receipt root identity differs from candidate root")
    mount = receipt.get("mount_identity")
    if not isinstance(mount, Mapping) or mount.get("verified") is not True:
        raise NtuAcquisitionError("MOUNT_IDENTITY", "candidate receipt lacks verified mount identity")
    receipt_artifacts = receipt.get("artifacts")
    if not isinstance(receipt_artifacts, list):
        raise NtuAcquisitionError("ARTIFACT_SET", "candidate receipt artifacts are missing")
    by_key: dict[tuple[str, str], Mapping[str, Any]] = {}
    for raw in receipt_artifacts:
        if not isinstance(raw, Mapping):
            raise NtuAcquisitionError("ARTIFACT_SET", "candidate artifact is malformed")
        key = (str(raw.get("sequence_id")), str(raw.get("role")))
        if key in by_key:
            raise NtuAcquisitionError("ARTIFACT_SET", f"duplicate candidate role: {key[0]}/{key[1]}")
        by_key[key] = raw
    expected_keys = {(sequence, role) for sequence in SEQUENCES for role in ("sequence_archive", "ground_truth_csv")}
    expected_keys.update({("calibration", role) for role in CALIBRATION_ROLES})
    if set(by_key) != expected_keys:
        raise NtuAcquisitionError("ARTIFACT_SET", "candidate does not contain exactly all selected sequences and calibration roles")

    checked: list[dict[str, Any]] = []
    for label, spec, sequence, role in _iter_role_specs(context):
        if sequence == "calibration":
            expected_relative = PurePosixPath("calibration", role, str(spec["expected_filename"])).as_posix()
        else:
            expected_relative = PurePosixPath("sequences", sequence, role, str(spec["expected_filename"])).as_posix()
        raw = by_key[(sequence, role)]
        if raw.get("role") != role or raw.get("sequence_id") != sequence:
            raise NtuAcquisitionError("ROLE_IDENTITY", f"{label} role identity is inconsistent")
        checked.append(_verify_artifact(candidate_root, label, raw, spec, expected_relative))
    _assert_runner_separation(receipt, checked)

    inputs = [item for item in checked if item["role"] == "sequence_archive"]
    ground_truth = [item for item in checked if item["role"] == "ground_truth_csv"]
    calibration = [item for item in checked if item["sequence_id"] == "calibration"]
    # The proposal deliberately keeps GT in its own role section.  It is never
    # copied to runner_input_manifest and contains no absolute host path.
    manifest: dict[str, Any] = {
        "schema_version": MANIFEST_VERSION,
        "manifest_kind": MANIFEST_KIND,
        "status": "PROPOSED_REVIEW_REQUIRED",
        "claim_eligible": False,
        "selection_id": context.selection_id,
        "selection_sha256": context.selection_sha256,
        "profile_sha256": context.profile_sha256,
        "candidate_receipt_sha256": receipt_sha,
        "candidate_root_relative": candidate_relative,
        "runner_gt_paths_exposed": False,
        "runner_input_manifest": {
            "ground_truth_paths_exposed": False,
            "input_relative_paths": sorted(item["relative_path"] for item in inputs),
            "ground_truth_relative_paths": [],
        },
        "input_artifacts": inputs,
        "ground_truth_artifacts": ground_truth,
        "calibration_artifacts": calibration,
        "profile_diff": {
            "selection_id": context.selection_id,
            "required_review_status": "REVIEWED",
            "required_profile_byte_identity_status": "REVALIDATED",
            "sequence_roles": sorted((
                {"sequence_id": item["sequence_id"], "input_sha256": next(x["sha256"] for x in inputs if x["sequence_id"] == item["sequence_id"]),
                 "ground_truth_sha256": next(x["sha256"] for x in ground_truth if x["sequence_id"] == item["sequence_id"]),
                 "calibration_sha256": canonical_sha256(sorted((x["relative_path"], x["sha256"]) for x in calibration))}
                for item in inputs), key=lambda item: item["sequence_id"]),
        },
        "review_requirements": [
            "independent reviewer verifies candidate receipt and all sidecars",
            "reviewer updates the immutable profile pin fields in a separate change",
            "reviewer does not expose ground-truth paths to any runner",
            "dataset preflight accepts only a reviewed profile pin, never this proposal",
        ],
    }
    payload = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
    digest = _atomic_write_new(output_path, payload)
    _atomic_write_new(output_path.with_name(output_path.name + ".sha256"),
                      (digest + "  " + output_path.name + "\n").encode("ascii"))
    os.chmod(output_path, 0o444, follow_symlinks=False)
    os.chmod(output_path.with_name(output_path.name + ".sha256"), 0o444, follow_symlinks=False)
    return {"status": manifest["status"], "claim_eligible": False,
            "manifest_path": str(output_path), "manifest_sha256": digest,
            "selection_id": context.selection_id, "candidate_receipt_sha256": receipt_sha}


# Stable descriptive alias for callers that use the stage terminology.
authorize_candidate = propose_pin_manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--candidate-root", type=Path, required=True)
    parser.add_argument("--candidate-receipt", type=Path)
    parser.add_argument("--evidence-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = propose_pin_manifest(args.selection, args.profile, args.candidate_root,
                                      args.output, candidate_receipt=args.candidate_receipt,
                                      evidence_root=args.evidence_root, root=ROOT)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except (NtuAcquisitionError, OSError, UnicodeError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "NOT_AUTHORIZED", "error": str(exc)}, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
