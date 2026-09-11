#!/usr/bin/env python3
"""Create or verify one additive v13 exact-root formal authorization.

The v13 authorization reuses the tested v12 authorization mechanics but binds
the mount-corrected v13 candidate and its immutable persistence receipt.  It
never opens the bag, starts Docker/ROS, invokes scoring, or authorizes another
root.  The v12 feeder's underlying nonzero cause remains explicitly unknown.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.authorize_fast_livo2_m6a10_v12_formal as base  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v13_persistence_gate as persistence  # noqa: E402

BASE_WRITE_JSON = base._write_json
_extra_forbidden_processes = base._extra_forbidden_processes


PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v13_formal_candidate.yaml"
PROFILE_SHA256 = "53d49adf595ae1783f741e264351dca45e2b17b6c91e9259f3e5fa5d036060ab"
PERSISTENCE_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v13_persistence_20260823T192720Z_agentv13/"
    "persistence_gate.receipt.json"
)
PERSISTENCE_RECEIPT_SHA256 = "18998f6177418eaf7e98fe517691d0642d59ac48f2faeb9b45ef0bc8fef62c72"
PERSISTENCE_SIDECAR_SHA256 = "1988676efabdf1276ca3bb635f784941d2539aa15f58f94a8f249bcb730eabb7"
AUTHORIZED_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v13_formal_authorized.py"
BASE_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v13_formal.py"
BASE_LAUNCHER_SHA256 = "7402bef5a5ad13898461a4d9c53d95715e1a22631fe88d76af9f468ccfe0407b"
WRAPPER_PATH = ROOT / "scripts/fast_livo2_m6a10_v12_formal_container_run.sh"
WRAPPER_SHA256 = "af6fa54f834ae209758ee5a9903695674f4095c1063ac57b46c2774bfbc5c7a9"
IMAGE_TAG = "m6a10-v2c-v12-nonlidar-boundary-transport-20260824-fast-livo2-benchmark:ros1-pinned"
IMAGE_ID = "sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7"
PHASE_CONTRACT = "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
TRANSPORT_CONTRACT = "m6a10-v12-callback-ack-transport-outstanding-v1"
CONTRACT_VERSION = "m6a10-v13-formal-exact-root-authorization-v1"
RECEIPT_NAME = "formal_authorization.receipt.json"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _pin(path: Path, expected: str, label: str) -> str:
    base._regular(path, label)
    observed = _sha256(path)
    if observed != expected:
        raise base.AuthorizationError("SOURCE_DRIFT", "%s SHA drift" % label)
    return observed


def _verify_receipt(path: Path, expected: str, sidecar_sha: str, label: str) -> Dict[str, Any]:
    observed = _pin(path, expected, label)
    sidecar = path.with_name(path.name + ".sha256")
    _pin(sidecar, sidecar_sha, "%s sidecar" % label)
    if sidecar.read_text(encoding="ascii") != "%s  %s\n" % (observed, path.name):
        raise base.AuthorizationError("RECEIPT_SIDECAR", "%s sidecar content drift" % label)
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise base.AuthorizationError("RECEIPT_INVALID", "%s is not an object" % label)
    return value


def _verify_profile() -> Dict[str, Any]:
    _pin(PROFILE_PATH, PROFILE_SHA256, "v13 formal profile")
    document = yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1 or \
            document.get("status") != "V13_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            document.get("formal_replay_forbidden") is not True or \
            document.get("formal_replay_authorized") is not False or \
            document.get("replay_count") != 0:
        raise base.AuthorizationError("PROFILE_AUTHORITY", "v13 profile is not unauthorized")
    candidate = document.get("candidate")
    image = document.get("image")
    input_value = document.get("input")
    if not isinstance(candidate, Mapping) or candidate.get("evidence_persistence_candidate") is not True or \
            candidate.get("feeder_root_cause_fixed") is not False or \
            candidate.get("feeder_underlying_nonzero_cause") != "unknown" or \
            candidate.get("phase_contract") != PHASE_CONTRACT or \
            candidate.get("transport_contract") != TRANSPORT_CONTRACT:
        raise base.AuthorizationError("PROFILE_CONTRACT", "v13 candidate contract drift")
    if not isinstance(image, Mapping) or image.get("tag") != IMAGE_TAG or image.get("id") != IMAGE_ID:
        raise base.AuthorizationError("PROFILE_IMAGE", "v13 image drift")
    if not isinstance(input_value, Mapping) or input_value.get("opened") is not False or \
            input_value.get("bytes") != base.INPUT_BYTES or input_value.get("sha256") != base.INPUT_SHA256:
        raise base.AuthorizationError("PROFILE_INPUT", "v13 input identity drift")
    return document


def _verify_immutable_lineage(repo_root: Path = ROOT) -> Dict[str, Any]:
    """Bind v12 evidence, v13 persistence, and the current candidate sources."""
    lineage = persistence.verify_candidate_lineage(repo_root)
    profile = _verify_profile()
    _pin(repo_root / BASE_LAUNCHER_PATH.relative_to(ROOT), BASE_LAUNCHER_SHA256, "v13 base launcher")
    _pin(repo_root / WRAPPER_PATH.relative_to(ROOT), WRAPPER_SHA256, "v12 wrapper")
    authorized_launcher = repo_root / AUTHORIZED_LAUNCHER_PATH.relative_to(ROOT)
    if not authorized_launcher.is_file() or authorized_launcher.is_symlink():
        raise base.AuthorizationError("AUTHORIZED_LAUNCHER", "v13 authorized launcher is missing")
    persistence_receipt = _verify_receipt(
        PERSISTENCE_RECEIPT_PATH, PERSISTENCE_RECEIPT_SHA256,
        PERSISTENCE_SIDECAR_SHA256, "v13 persistence receipt",
    )
    if persistence_receipt.get("status") != "PASS" or \
            persistence_receipt.get("kind") != persistence.CONTRACT or \
            persistence_receipt.get("feeder_root_cause") != "unknown":
        raise base.AuthorizationError("PERSISTENCE_RECEIPT", "v13 persistence receipt is not admissible")
    return {
        "v13_profile": {"path": str(PROFILE_PATH.resolve()), "sha256": PROFILE_SHA256},
        "v13_base_launcher": {"path": str(BASE_LAUNCHER_PATH.resolve()), "sha256": BASE_LAUNCHER_SHA256},
        "v13_authorized_launcher": {"path": str(authorized_launcher.resolve()), "sha256": _sha256(authorized_launcher)},
        "v12_wrapper": {"path": str(WRAPPER_PATH.resolve()), "sha256": WRAPPER_SHA256},
        "v13_persistence_gate": {"path": str(PERSISTENCE_RECEIPT_PATH), "sha256": PERSISTENCE_RECEIPT_SHA256},
        "v12_lineage": lineage,
        "profile_document_sha256": _sha256(PROFILE_PATH),
        "feeder_root_cause_fixed": False,
        "feeder_underlying_nonzero_cause": "unknown",
    }


def _enriched_write_json(path: Path, value: Mapping[str, Any]) -> str:
    if path.name == RECEIPT_NAME:
        enriched = dict(value)
        enriched["candidate_version"] = "v13-mount-corrected-evidence-persistence"
        enriched["evidence_persistence_candidate"] = True
        enriched["feeder_root_cause_fixed"] = False
        enriched["feeder_underlying_nonzero_cause"] = "unknown"
        enriched["persistence_receipt_sha256"] = PERSISTENCE_RECEIPT_SHA256
        value = enriched
    return BASE_WRITE_JSON(path, value)


def _patch_base_globals() -> None:
    base.PROFILE_PATH = PROFILE_PATH
    base.PROFILE_SHA256 = PROFILE_SHA256
    base.IMAGE_TAG = IMAGE_TAG
    base.IMAGE_ID = IMAGE_ID
    base.PHASE_CONTRACT = PHASE_CONTRACT
    base.TRANSPORT_CONTRACT = TRANSPORT_CONTRACT
    base.CONTRACT_VERSION = CONTRACT_VERSION
    base.RECEIPT_NAME = RECEIPT_NAME
    base._verify_immutable_lineage = _verify_immutable_lineage
    base._write_json = _enriched_write_json


def authorize(*args: Any, **kwargs: Any) -> Dict[str, Any]:
    _patch_base_globals()
    return base.authorize(*args, **kwargs)


def verify_authorization(*args: Any, **kwargs: Any) -> Dict[str, Any]:
    _patch_base_globals()
    value = base.verify_authorization(*args, **kwargs)
    if value.get("candidate_version") != "v13-mount-corrected-evidence-persistence" or \
            value.get("evidence_persistence_candidate") is not True or \
            value.get("feeder_root_cause_fixed") is not False or \
            value.get("feeder_underlying_nonzero_cause") != "unknown" or \
            value.get("persistence_receipt_sha256") != PERSISTENCE_RECEIPT_SHA256:
        raise base.AuthorizationError("AUTHORIZATION_CANDIDATE", "v13 candidate binding drift")
    sources = value.get("lineage", {}).get("v13_authorized_launcher", {})
    launcher = Path(str(sources.get("path", "")))
    if launcher.resolve() != AUTHORIZED_LAUNCHER_PATH.resolve() or \
            _sha256(launcher) != sources.get("sha256"):
        raise base.AuthorizationError("AUTHORIZATION_LAUNCHER", "authorized launcher source drift")
    return value


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--authorization-root", type=Path, required=True)
    parser.add_argument("--attempt-root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--proc-root", type=Path, default=Path("/proc"))
    args = parser.parse_args(argv)
    try:
        result = authorize(
            args.authorization_root, args.attempt_root,
            repo_root=args.repo_root, proc_root=args.proc_root,
        )
    except base.AuthorizationError as error:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": error.kind, "failure_message": str(error)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "AUTHORIZED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
