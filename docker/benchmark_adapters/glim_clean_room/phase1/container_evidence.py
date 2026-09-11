#!/usr/bin/env python3
"""Pure parsing and fail-closed validation for a Phase 1 container receipt."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Any, Dict, List, Mapping, Optional


PHASE1_ROOT = Path(__file__).resolve().parent
SCHEMA_PATH = PHASE1_ROOT / "container_receipt.schema.json"
EXPECTED_KIND = "glim_clean_room_phase1_container_attempt"
EXPECTED_CANDIDATE = "glim-clean-room-phase1-core-public-api-v2"
EXPECTED_REVISION = "r2"
HEX64 = re.compile(r"^[0-9a-f]{64}$")
LDD_FINAL = re.compile(r"^LDD_UNRESOLVED=([0-9]+)$")
LDD_NOT_FOUND = re.compile(r"\bnot found\b")


class EvidenceValidationError(ValueError):
    """A malformed, incomplete, or ineligible evidence artifact."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_ldd_report(text: str) -> Dict[str, Any]:
    """Parse only actual ldd lines between exact start/final markers.

    Shell tracing is intentionally outside the bounded body.  This prevents a
    command such as ``grep -c 'not found'`` after the final marker from being
    mistaken for an unresolved library.
    """
    lines = text.splitlines()
    starts = [index for index, line in enumerate(lines)
              if line.strip() == "LDD_REPORT"]
    if len(starts) != 1:
        raise EvidenceValidationError("expected exactly one LDD_REPORT marker")
    start = starts[0]
    end: Optional[int] = None
    declared: Optional[int] = None
    for index in range(start + 1, len(lines)):
        match = LDD_FINAL.fullmatch(lines[index].strip())
        if match:
            end = index
            declared = int(match.group(1))
            break
    if end is None or declared is None:
        raise EvidenceValidationError("missing final LDD_UNRESOLVED marker")
    raw_body = lines[start + 1:end]
    # ``set -x`` may trace the grep command before the final marker.  Actual
    # ldd output is indented or starts with ``###``; trace lines begin with
    # one or more plus signs and are excluded explicitly.
    body = [line for line in raw_body if not line.lstrip().startswith("+")]
    unresolved = [line for line in body if LDD_NOT_FOUND.search(line)]
    targets = [line[4:] for line in body if line.startswith("### ")]
    result = {
        "marker_bounded": True,
        "target_count": len(targets),
        "targets": targets,
        "shell_trace_lines_excluded": len(raw_body) - len(body),
        "unresolved_count": len(unresolved),
        "unresolved_lines": unresolved,
        "declared_unresolved": declared,
    }
    if declared != len(unresolved):
        raise EvidenceValidationError(
            "declared LDD count does not match bounded report: " + repr(result))
    return result


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise EvidenceValidationError(message)


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    _require(isinstance(value, Mapping), name + " must be an object")
    return value


def _exact_counts(value: Mapping[str, Any], name: str,
                  total: int, passed: int, failed: int) -> None:
    _require(value.get("total") == total, name + " total must be " + str(total))
    _require(value.get("passed") == passed, name + " passed count drift")
    _require(value.get("failed") == failed, name + " failed count drift")


def _validate_schema_metadata() -> None:
    try:
        schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise EvidenceValidationError("cannot read receipt schema: " + str(error))
    _require(schema.get("$schema") == "http://json-schema.org/draft-07/schema#",
             "receipt schema dialect drift")
    _require(schema.get("properties", {}).get("receipt_status", {}).get("enum")
             and "CORRECTED_SEALED" in schema["properties"]["receipt_status"]["enum"],
             "receipt schema omits CORRECTED_SEALED")
    corrected = schema.get("allOf", [])[0]
    _require("supersedes_receipt" in corrected.get("then", {}).get("required", []),
             "receipt schema does not require supersedes_receipt for corrections")


def validate_receipt(value: Mapping[str, Any], receipt_path: Optional[Path] = None) -> Dict[str, Any]:
    """Validate claim-ineligible container evidence and return its summary."""
    _validate_schema_metadata()
    required = {
        "schema_version", "receipt_kind", "candidate_id", "candidate_revision",
        "status", "benchmark_execution", "dataset_execution", "gt_execution",
        "scorer_execution", "attempt", "recipe", "base_image", "image", "postrun",
    }
    _require(required.issubset(value), "receipt is missing required fields")
    _require(value.get("schema_version") == 1, "receipt schema_version drift")
    _require(value.get("receipt_kind") == EXPECTED_KIND, "receipt kind drift")
    _require(value.get("candidate_id") == EXPECTED_CANDIDATE, "candidate identity drift")
    _require(value.get("candidate_revision") == EXPECTED_REVISION, "candidate revision drift")
    _require(value.get("status") == "PASS", "receipt is not a PASS")
    for role in ("benchmark_execution", "dataset_execution", "gt_execution", "scorer_execution"):
        _require(value.get(role) == "FORBIDDEN", role + " must be FORBIDDEN")

    attempt = _mapping(value["attempt"], "attempt")
    _require(attempt.get("build_exit") == 0, "build exit is not zero")
    image = _mapping(value["image"], "image")
    labels = _mapping(image.get("labels"), "image.labels")
    _require(labels.get("org.opencontainers.image.benchmark_eligible") == "false",
             "image is not benchmark-ineligible")
    postrun = _mapping(value["postrun"], "postrun")
    _require(postrun.get("exit") == 0, "postrun exit is not zero")
    _exact_counts(_mapping(postrun.get("ctest_all"), "postrun.ctest_all"),
                  "postrun.ctest_all", 3, 3, 0)
    _exact_counts(_mapping(postrun.get("ctest_link_smoke"), "postrun.ctest_link_smoke"),
                  "postrun.ctest_link_smoke", 1, 1, 0)
    _require(postrun.get("ldd_unresolved") == 0, "unresolved dynamic dependency exists")
    _require(postrun.get("installed_regular_file_count") == 16428,
             "installed-file count drift")
    receipt_status = value.get("receipt_status")
    if receipt_status == "CORRECTED_SEALED":
        supersedes = _mapping(value.get("supersedes_receipt"), "supersedes_receipt")
        path_value = supersedes.get("path")
        digest = supersedes.get("sha256")
        _require(isinstance(path_value, str) and HEX64.fullmatch(str(digest)) is not None,
                 "supersedes_receipt identity is malformed")
        if receipt_path is not None:
            superseded_path = Path(path_value)
            _require(superseded_path.is_file() and not superseded_path.is_symlink(),
                     "superseded receipt is unavailable")
            _require(_sha256(superseded_path) == digest,
                     "superseded receipt SHA-256 mismatch")
        _require(isinstance(supersedes.get("reason"), str) and supersedes["reason"],
                 "supersedes_receipt reason is missing")
    else:
        _require(receipt_status in (None, "FINALIZED_EXISTING", "NEW_ATTEMPT"),
                 "unknown receipt_status")
    return {
        "status": "PASS",
        "receipt_status": receipt_status,
        "candidate_id": value["candidate_id"],
        "ctest_all": dict(postrun["ctest_all"]),
        "ctest_link_smoke": dict(postrun["ctest_link_smoke"]),
        "installed_regular_file_count": postrun["installed_regular_file_count"],
        "ldd_unresolved": postrun["ldd_unresolved"],
    }


def validate_receipt_file(path: Path) -> Dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise EvidenceValidationError("receipt is not a regular file: " + str(path))
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise EvidenceValidationError("invalid receipt JSON: " + str(error))
    _require(isinstance(value, Mapping), "receipt root must be an object")
    return validate_receipt(value, path)


def _main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    validate = subparsers.add_parser("validate-receipt")
    validate.add_argument("path", type=Path)
    parse = subparsers.add_parser("parse-postrun")
    parse.add_argument("path", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "validate-receipt":
            result = validate_receipt_file(args.path)
        else:
            result = parse_ldd_report(args.path.read_text(encoding="utf-8", errors="replace"))
        print(json.dumps(result, sort_keys=True))
        return 0
    except (EvidenceValidationError, OSError, json.JSONDecodeError) as error:
        print("container evidence: FAIL_CLOSED: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(_main())
