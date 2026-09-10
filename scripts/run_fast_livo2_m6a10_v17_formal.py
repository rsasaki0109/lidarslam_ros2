#!/usr/bin/env python3
"""Unauthorized v17 formal candidate with an explicit v12 contract snapshot.

v17 is additive to the immutable v16 lineage and binds the corrected v17
image/build/no-input evidence.  The producer change is limited to empty
synchronization retry disposition; acquired records, discard, processing
failure, and conservation failures remain sticky invalid.

The command-line entry point remains unauthorized.  ``main(..., runtime=...)``
is an injected test seam for a fake authorization receipt, identity/bag
probes, monitor, Popen, raw artifacts, and compositor.  It never runs Docker,
ROS, bag input, GT, scoring, or map generation in tests.
"""

from __future__ import annotations

from contextlib import contextmanager
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from typing import Any, Callable, Dict, Iterator, List, Mapping, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v12_formal as v12  # noqa: E402
import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v15_formal as v15  # noqa: E402


CANDIDATE_VERSION = "v17-retryable-empty-synchronization-abort"
CONTRACT_VERSION = "m6a10-v17-formal-candidate-closure-v1"
PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_ready.yaml"
PROFILE_SHA256 = "10ddffb6eed647eb3b7a2ee7ede49505923755eb917d8b87737675853e35d9c5"
READY_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_candidate.yaml"
READY_PROFILE_SHA256 = "1803ba255adfac96a61e1f677be53e84b196ad70bb367a10ebdb9791cb94128b"
IMAGE_TAG = "m6a10-v2c-v17-retryable-abort-correction-20260823t224017z-fast-livo2-benchmark:ros1-pinned"
IMAGE_ID = "sha256:f1432426af64e76d9ad9c655a8727d35c2fb0a03753502ada3b21187638f747c"
BASE_IMAGE_ID = "sha256:f103a0f61f7ec9b19cbae02da6c6b8a28f17475dbc3f51fba3206b954ca4ca7a"
PHASE_CONTRACT = v12.PHASE_CONTRACT
TRANSPORT_CONTRACT = v12.TRANSPORT_CONTRACT
TERMINAL_CONTRACT = v15.TERMINAL_CONTRACT
PHASE_MODE = v15.PHASE_MODE
EXPECTED_COUNTS = dict(v15.EXPECTED_COUNTS)
EXPECTED_MESSAGES = v15.EXPECTED_MESSAGES
REQUIRED_END_TIMESTAMP_SECONDS = v15.REQUIRED_END_TIMESTAMP_SECONDS
MAX_END_GAP_SECONDS = v15.MAX_END_GAP_SECONDS
SENSOR_DURATION_SECONDS = v15.SENSOR_DURATION_SECONDS
WATCHDOG_SECONDS = v15.WATCHDOG_SECONDS
MONITOR_INTERVAL_SECONDS = v15.MONITOR_INTERVAL_SECONDS
INPUT_PATH = v15.INPUT_PATH
INPUT_BYTES = v15.INPUT_BYTES
INPUT_SHA256 = v15.INPUT_SHA256
FEEDER_SOURCE = Path("scripts/fast_livo2_m6a10_v15_feeder.py")
FEEDER_SHA256 = "6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7"
WRAPPER_SOURCE = Path("scripts/fast_livo2_m6a10_v17_formal_container_run.sh")
WRAPPER_SHA256 = "506c6a8c72bca9fa66cf49abaead79eabc5b177fa9b1b30f868383a98a5ab67e"
V12_PATCH_SOURCE = Path("docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch")
V12_PATCH_SHA256 = v15.V12_PATCH_SHA256
WRAPPER_PATH_IN_CONTAINER = "/runner/v17_runtime.sh"
FEEDER_PATH_IN_CONTAINER = v15.FEEDER_PATH_IN_CONTAINER
TIMING_CONTRACT = v15.TIMING_CONTRACT
MONITOR_CONTRACT = v15.MONITOR_CONTRACT
RECEIPT_NAME = "closure_receipt.json"

V16_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v16_formal.py"
V16_LAUNCHER_SHA256 = "77cf40effb3e578d0c2c95e48ea34b6dc5aa77d7d2fbc7bdd954936b63949bcd"
V15_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_candidate.yaml"
V15_PROFILE_SHA256 = "017e8583e1085b1f11da4117a1c53497cdcb1055c6c6224568df2f73ba751bc7"
V16_AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v16_formal.py"
V16_AUTHORIZER_SHA256 = "3f26dd7c1b530d7eb6828a39c49a9829f692c8528e3da12f7515ea76bd397b26"

BUILD_RECEIPT_PATH = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v17_build_correction_20260823T224017Z_agentv17/build_identity.receipt.json")
BUILD_RECEIPT_SHA256 = "23811d4200f9ed3ac464083ef339eb6d0437815ffdd82a2f37cc4180195bc2f7"
BUILD_RECEIPT_SIDECAR_SHA256 = "76e9859832412f0310e685b376324ff53ce4dbe033d2d440fce940421b1de15b"
NO_INPUT_PASS_RECEIPT_PATH = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v17_no_input_correction2_20260823T225359Z_agentv17/no_input.receipt.json")
NO_INPUT_PASS_RECEIPT_SHA256 = "ac1df6f390ad3e348e572ced668f4971bc05ac835c55cc83fef9ea4ad5fa3d74"
NO_INPUT_PASS_SIDECAR_SHA256 = "8dfa1520c1e2024af3f0710de4ee8c38c9cf4a1402389891ac77d423d1848164"
PRIOR_NO_INPUT_FAIL_RECEIPT_PATH = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v17_no_input_correction_20260823T224017Z_agentv17/no_input.receipt.json")
PRIOR_NO_INPUT_FAIL_RECEIPT_SHA256 = "93296e2d47e37aeea739daf3c44d4c7bf988c7a9bcfc61f11a4cbab31ec1203a"
PRIOR_NO_INPUT_FAIL_SIDECAR_SHA256 = "a0db9e7c7303974edc575033088bc92e0bc14ab8db7eda054ae16c1ec4d9c2f7"
V16_FAILURE_CLOSURE_SHA256 = "a7a5bfaaaf99a0851cb1b5b19759cdc568c3d4f681f8f2ac81c438944d1c1ccc"
V16_TERMINAL_RAW_SHA256 = "56523b593676961b83ca9f9340706e964086b6aa3916e47902219b81dc41d21a"
V16_FAILURE_CLOSURE_PATH = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v16_formal_replay_20260823T213533Z_agentv16formal/closure_receipt.json")
V16_TERMINAL_RAW_PATH = Path("/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v16_formal_replay_20260823T213533Z_agentv16formal/out/consumer_evidence.json")

# Capture the actual v12 callable before any v16 scope is entered.  This is
# deliberately a direct symbol reference, not getattr/fallback lookup.
ORIGINAL_V12_BUILD_DOCKER_ARGV = v12.build_safe_docker_argv
BASE_ATOMIC_CREATE_JSON = v12._atomic_create_json
BASE_PRODUCTION_COMPOSE = v12._production_compose

CandidateError = v12.CandidateError
AuthorizationError = v12.AuthorizationError
ProcessFactory = Callable[[Sequence[str], Path], Any]
Probe = Callable[[Any], Mapping[str, Any]]
Capture = Callable[[Any, Path], Mapping[str, Any]]
Composer = Callable[[Mapping[str, Any], Any], Mapping[str, Any]]


def _sha256_file(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise CandidateError("SOURCE_MISSING", "not a regular source: %s" % path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular(path: Path, label: str) -> None:
    current = Path(path.absolute().anchor)
    for component in path.absolute().parts[1:]:
        current /= component
        if current.is_symlink():
            raise CandidateError("SYMLINK_REJECTED", "%s contains a symlink" % label)
    if os.path.lexists(path) is False or path.is_symlink() or not path.is_file():
        raise CandidateError("NOT_REGULAR", "%s is not a regular file" % label)


def _verify_json_receipt(path: Path, expected_sha: str, sidecar_sha: str, label: str) -> Dict[str, Any]:
    observed = _sha256_file(path)
    if observed != expected_sha:
        raise CandidateError("RECEIPT_DRIFT", "%s SHA drift" % label)
    sidecar = path.with_name(path.name + ".sha256")
    observed_sidecar = _sha256_file(sidecar)
    if observed_sidecar != sidecar_sha:
        raise CandidateError("RECEIPT_SIDECAR_DRIFT", "%s sidecar SHA drift" % label)
    expected_line = ("%s  %s\n" % (expected_sha, path.name)).encode("ascii")
    if sidecar.name != path.name + ".sha256" or sidecar.read_bytes() != expected_line:
        raise CandidateError("RECEIPT_SIDECAR_CONTENT", "%s sidecar filename/content drift" % label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CandidateError("RECEIPT_INVALID", "%s is not JSON" % label) from exc
    if not isinstance(value, dict):
        raise CandidateError("RECEIPT_INVALID", "%s must be an object" % label)
    return value


def _mount_destination(spec: str) -> str:
    for field in spec.split(","):
        if field.startswith("dst="):
            return field[4:]
    raise CandidateError("DOCKER_ARGV", "mount has no destination")


def _mounts(argv: Sequence[str]) -> List[str]:
    return [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == "--mount"]


def _tmpfs_destinations(argv: Sequence[str]) -> List[str]:
    return [argv[index + 1].split(":", 1)[0]
            for index, item in enumerate(argv[:-1]) if item == "--tmpfs"]


def _replace_mount(spec: str, source: Path, destination: str, readonly: bool) -> str:
    mode = "readonly" if readonly else "readonly=false"
    return "type=bind,src=%s,dst=%s,%s" % (source.resolve(), destination, mode)


def _replace_env_value(argv: List[str], key: str, value: str) -> None:
    matches = [index + 1 for index, item in enumerate(argv[:-1])
               if item == "--env" and argv[index + 1].startswith(key + "=")]
    if len(matches) != 1:
        raise CandidateError("DOCKER_ENV", "expected one %s environment entry" % key)
    argv[matches[0]] = "%s=%s" % (key, value)


def build_safe_docker_argv(config: Any, output_dir: Path) -> List[str]:
    """Call the captured v12 builder once and apply the v17-safe transform."""
    parent_argv = ORIGINAL_V12_BUILD_DOCKER_ARGV(config, output_dir)
    argv: List[str] = []
    index = 0
    wrapper_source = Path(config.repo_root) / WRAPPER_SOURCE
    feeder_source = Path(config.repo_root) / FEEDER_SOURCE
    while index < len(parent_argv):
        item = parent_argv[index]
        if item == "--tmpfs" and index + 1 < len(parent_argv):
            if parent_argv[index + 1].startswith("/out:"):
                index += 2
                continue
        if item == "--mount" and index + 1 < len(parent_argv):
            spec = parent_argv[index + 1]
            destination = _mount_destination(spec)
            if destination == v12.WRAPPER_PATH_IN_CONTAINER:
                argv.extend(("--mount", _replace_mount(spec, wrapper_source, WRAPPER_PATH_IN_CONTAINER, True)))
                index += 2
                continue
            if destination == v12.FEEDER_PATH_IN_CONTAINER:
                argv.extend(("--mount", _replace_mount(spec, feeder_source, FEEDER_PATH_IN_CONTAINER, True)))
                index += 2
                continue
        if item == "--entrypoint" and index + 1 < len(parent_argv):
            argv.extend((item, WRAPPER_PATH_IN_CONTAINER))
            index += 2
            continue
        argv.append(IMAGE_ID if item == v12.IMAGE_ID else item)
        index += 1

    _replace_env_value(argv, "M6A10_PROFILE_PATH", "configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_candidate.yaml")
    _replace_env_value(argv, "M6A10_PROFILE_SHA256", READY_PROFILE_SHA256)
    _replace_env_value(argv, "M6A10_FAST_FEEDER_SHA256", FEEDER_SHA256)
    for key, value in v15.ROS_ENV.items():
        if any(item == "--env" and argv[index + 1].startswith(key + "=")
               for index, item in enumerate(argv[:-1])):
            _replace_env_value(argv, key, value)
        else:
            entrypoint_index = argv.index("--entrypoint")
            argv[entrypoint_index:entrypoint_index] = ["--env", "%s=%s" % (key, value)]

    # The mounted v17 wrapper has an explicit authorization gate.  Carry the
    # fixed, already-verified authorization state into the container only for
    # an authorized lifecycle; unauthorized candidate argv remains unable to
    # satisfy the wrapper gate.
    if config.authorization_path is not None and config.authorization_sha256:
        entrypoint_index = argv.index("--entrypoint")
        argv[entrypoint_index:entrypoint_index] = [
            "--env", "M6A10_V17_AUTHORIZATION_STATUS=AUTHORIZED_FOR_EXACT_ROOT",
        ]

    mounts = _mounts(argv)
    destinations = [_mount_destination(spec) for spec in mounts]
    expected_destinations = {"/input/ntu_viral.bag", "/out", WRAPPER_PATH_IN_CONTAINER, FEEDER_PATH_IN_CONTAINER}
    if len(destinations) != len(set(destinations)) or set(destinations) != expected_destinations:
        raise CandidateError("DOCKER_ARGV", "v17 mount destination allowlist drift")
    output_mounts = [spec for spec in mounts if _mount_destination(spec) == "/out"]
    expected_output = "type=bind,src=%s,dst=/out,readonly=false" % output_dir.resolve()
    if output_mounts != [expected_output]:
        raise CandidateError("DOCKER_ARGV", "v17 output bind contract drift")
    if _tmpfs_destinations(argv) != ["/tmp", "/root/.ros"] or "/out" in _tmpfs_destinations(argv):
        raise CandidateError("DOCKER_ARGV", "v17 tmpfs allowlist drift")
    if "--rm" in argv or any(item == "rw" for item in argv):
        raise CandidateError("DOCKER_ARGV", "unsafe cleanup or bare rw token")
    if argv[:3] != ["docker", "run", "--name"] or argv[-1] != IMAGE_ID:
        raise CandidateError("IMAGE_IDENTITY", "v17 image identity drift")
    if "--network" not in argv or argv[argv.index("--network") + 1] != "none":
        raise CandidateError("DOCKER_ARGV", "network is not none")
    if "--read-only" not in argv or "--init" not in argv or "--pull=never" not in argv:
        raise CandidateError("DOCKER_ARGV", "container safety flags drift")
    forbidden = {"--privileged", "--cap-add", "--device", "--pid=host", "--ipc=host", "--uts=host"}
    if any(item in forbidden for item in argv):
        raise CandidateError("DOCKER_ARGV", "unsafe capability or namespace flag")
    if argv[argv.index("--entrypoint") + 1] != WRAPPER_PATH_IN_CONTAINER:
        raise CandidateError("DOCKER_ARGV", "v17 entrypoint drift")
    if any(item in {"sh", "bash", "-c", "-lc", "--shell"} for item in argv):
        raise CandidateError("DOCKER_ARGV", "shell execution token is forbidden")
    return argv


def _snapshot_v12_state() -> Dict[str, Any]:
    """Explicitly capture only symbols that exist in the v12 module."""
    return {
        "PROFILE_PATH": v12.PROFILE_PATH,
        "PROFILE_SHA256": v12.PROFILE_SHA256,
        "READY_PROFILE_PATH": v12.READY_PROFILE_PATH,
        "READY_PROFILE_SHA256": v12.READY_PROFILE_SHA256,
        "IMAGE_TAG": v12.IMAGE_TAG,
        "IMAGE_ID": v12.IMAGE_ID,
        "PHASE_CONTRACT": v12.PHASE_CONTRACT,
        "TRANSPORT_CONTRACT": v12.TRANSPORT_CONTRACT,
        "verify_candidate_profile": v12.verify_candidate_profile,
        "build_safe_docker_argv": v12.build_safe_docker_argv,
        "_production_identity_probe": v12._production_identity_probe,
        "_production_capture": v12._production_capture,
        "_production_compose": v12._production_compose,
        "_continuous_quiescence": v12._continuous_quiescence,
        "_atomic_create_json": v12._atomic_create_json,
    }


def _restore_v12_state(snapshot: Mapping[str, Any]) -> None:
    for name, value in snapshot.items():
        setattr(v12, name, value)


def snapshot_v12_contracts() -> Mapping[str, str]:
    """Expose the exact phase/transport symbols used by v17."""
    return {"phase_contract": v12.PHASE_CONTRACT, "transport_contract": v12.TRANSPORT_CONTRACT}


def verify_candidate_profile(profile_path: Path = PROFILE_PATH,
                             repo_root: Path = ROOT) -> Mapping[str, Any]:
    """Verify the additive v17 profile and every immutable evidence binding."""
    if profile_path.resolve() != PROFILE_PATH.resolve():
        raise CandidateError("PROFILE_PATH", "v17 profile path differs")
    observed = _sha256_file(profile_path)
    if observed != PROFILE_SHA256:
        raise CandidateError("PROFILE_SHA256", "v17 profile SHA differs")
    import yaml
    try:
        document = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise CandidateError("PROFILE_INVALID", "v17 profile YAML invalid") from exc
    candidate = document.get("formal_candidate") if isinstance(document, Mapping) else None
    image = document.get("image") if isinstance(document, Mapping) else None
    phase = document.get("phase") if isinstance(document, Mapping) else None
    if document.get("status") != "V17_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            document.get("formal_replay_forbidden") is not True or \
            document.get("formal_replay_authorized") is not False or \
            document.get("replay_count") != 0 or \
            not isinstance(candidate, Mapping) or candidate.get("status") != "UNAUTHORIZED_NOT_RUN" or \
            candidate.get("authorization_status") != "UNAUTHORIZED_NOT_INSTALLED" or \
            candidate.get("predecessor_profile_sha256") != "a9ca34982cfb22fe7e3bb0e2a5437a10875990926fddb820fb6a29f6abc6461f" or \
            candidate.get("predecessor_launcher_sha256") != V16_LAUNCHER_SHA256 or \
            not isinstance(image, Mapping) or image.get("id") != IMAGE_ID or image.get("tag") != IMAGE_TAG or \
            not isinstance(phase, Mapping) or phase.get("contract_version") != PHASE_CONTRACT or \
            phase.get("transport", {}).get("contract_version") != TRANSPORT_CONTRACT:
        raise CandidateError("PROFILE", "v17 profile contract drift")
    source_section = document.get("source")
    if not isinstance(source_section, Mapping):
        raise CandidateError("PROFILE_SOURCES", "v17 source pins are missing")
    source_pins = {
        "v17_patch": (repo_root / "docker/patches/fast_livo2.m6a10-v2c-v17-retryable-abort.patch", "c9254497b21846b2de6a028aa5fd60927a80da641c2fb47943c49ff3636ca001"),
        "v17_production_selftest": (repo_root / "tools/m6a10_terminal_support_context_v17_production_selftest.cpp", "f81bde7cabe54ebfa702908afa9ef071f2436ce5fee2f50bf67e3c494972bc84"),
        "v17_wrapper": (repo_root / WRAPPER_SOURCE, WRAPPER_SHA256),
        "v17_no_input_payload": (repo_root / "scripts/fast_livo2_m6a10_v17_no_input_container_payload.sh", "a8043f751db0638f34e79254133ac5974527c609f7574c616ee219a685cd8ed5"),
        "v17_no_input_gate": (repo_root / "scripts/run_fast_livo2_m6a10_v17_no_input_gate.sh", "fcfa3da6908f17d5cf06c999345afd0a8eccabc83d95625b212314f6051406a9"),
        "v15_feeder": (repo_root / FEEDER_SOURCE, FEEDER_SHA256),
        "v12_patch": (repo_root / V12_PATCH_SOURCE, V12_PATCH_SHA256),
        "v16_launcher": (V16_LAUNCHER_PATH, V16_LAUNCHER_SHA256),
        "v16_authorizer": (V16_AUTHORIZER_PATH, _sha256_file(V16_AUTHORIZER_PATH)),
    }
    for label, (path, expected) in source_pins.items():
        entry_sha = source_section.get("%s_sha256" % label)
        if entry_sha != expected or _sha256_file(path) != expected:
            raise CandidateError("SOURCE_DRIFT", "%s SHA drift" % label)
    evidence = document.get("evidence")
    if not isinstance(evidence, Mapping):
        raise CandidateError("RECEIPT_PIN", "v17 evidence bindings are missing")
    build_entry = evidence.get("build_receipt")
    no_input_entry = evidence.get("no_input_gate")
    for label, entry, path, expected, sidecar in (
            ("build", build_entry, BUILD_RECEIPT_PATH, BUILD_RECEIPT_SHA256, BUILD_RECEIPT_SIDECAR_SHA256),
            ("no-input", no_input_entry, NO_INPUT_PASS_RECEIPT_PATH, NO_INPUT_PASS_RECEIPT_SHA256, NO_INPUT_PASS_SIDECAR_SHA256)):
        if not isinstance(entry, Mapping) or entry.get("path") != str(path) or entry.get("sha256") != expected or entry.get("sidecar_sha256") != sidecar:
            raise CandidateError("RECEIPT_PIN", "%s receipt pin drift" % label)
    build_value = _verify_json_receipt(BUILD_RECEIPT_PATH, BUILD_RECEIPT_SHA256, BUILD_RECEIPT_SIDECAR_SHA256, "v17 build")
    if build_value.get("status") != "PASS" or build_value.get("image", {}).get("id") != IMAGE_ID or \
            build_value.get("safety", {}).get("formal_replay_started") is not False:
        raise CandidateError("BUILD_RECEIPT", "v17 build lineage is not admissible")
    no_input_value = _verify_json_receipt(NO_INPUT_PASS_RECEIPT_PATH, NO_INPUT_PASS_RECEIPT_SHA256, NO_INPUT_PASS_SIDECAR_SHA256, "v17 no-input")
    runtime = no_input_value.get("runtime", {})
    if no_input_value.get("status") != "PASS" or no_input_value.get("image", {}).get("id") != IMAGE_ID or \
            runtime.get("start_count") != 1 or runtime.get("network") != "none" or runtime.get("rootfs") != "read_only" or \
            runtime.get("host_mount_count") != 0 or runtime.get("oom_killed") is not False or \
            runtime.get("cleanup") != "stopped_only_remove_success" or \
            no_input_value.get("safety", {}).get("formal_replay_started") is not False:
        raise CandidateError("NO_INPUT_LINEAGE", "v17 no-input lineage is not admissible")
    prior = _verify_json_receipt(PRIOR_NO_INPUT_FAIL_RECEIPT_PATH, PRIOR_NO_INPUT_FAIL_RECEIPT_SHA256, PRIOR_NO_INPUT_FAIL_SIDECAR_SHA256, "v17 prior no-input failure")
    if prior.get("status") != "FAIL_CLOSED":
        raise CandidateError("FAILURE_LINEAGE", "prior v17 failure was not preserved")
    _regular(V16_FAILURE_CLOSURE_PATH, "v16 closure")
    if _sha256_file(V16_FAILURE_CLOSURE_PATH) != V16_FAILURE_CLOSURE_SHA256:
        raise CandidateError("V16_LINEAGE", "v16 closure SHA drift")
    _regular(V16_TERMINAL_RAW_PATH, "v16 terminal raw")
    if _sha256_file(V16_TERMINAL_RAW_PATH) != V16_TERMINAL_RAW_SHA256:
        raise CandidateError("V16_LINEAGE", "v16 terminal raw SHA drift")
    return {"path": str(profile_path.resolve()), "sha256": observed,
            "phase_contract": PHASE_CONTRACT, "transport_contract": TRANSPORT_CONTRACT,
            "image_id": IMAGE_ID, "build_receipt_sha256": BUILD_RECEIPT_SHA256,
            "no_input_receipt_sha256": NO_INPUT_PASS_RECEIPT_SHA256}


def _validate_schema3_callback(raw: Mapping[str, Any]) -> None:
    documents = raw.get("documents") if isinstance(raw, Mapping) else None
    callback = documents.get("callback") if isinstance(documents, Mapping) else None
    if not isinstance(callback, Mapping) or callback.get("schema_version") != 3 or \
            callback.get("contract_version") != PHASE_CONTRACT or \
            callback.get("transport_contract_version") != TRANSPORT_CONTRACT or \
            callback.get("phase_mode") != PHASE_MODE or callback.get("status") != "pass":
        raise CandidateError("CALLBACK_SCHEMA3", "schema-3 v5 callback evidence is invalid")
    if callback.get("ground_truth_content_opened") is not False or \
            callback.get("scorer_invoked") is not False:
        raise CandidateError("CALLBACK_AUTHORITY", "callback evidence contains unsafe authority")
    consumer = callback.get("consumer")
    if not isinstance(consumer, Mapping) or consumer.get("ack_exact") is not True or \
            consumer.get("transport_outstanding_at_drain") != 0 or \
            consumer.get("maximum_transport_outstanding_messages") != 1 or \
            consumer.get("maximum_allowed_transport_outstanding_messages") != 1 or \
            consumer.get("dropped_messages") != 0 or consumer.get("queue_overflow") != 0 or \
            consumer.get("processing_failures") != 0 or consumer.get("drain_complete") is not True:
        raise CandidateError("CALLBACK_TRANSPORT", "v17 transport conservation is invalid")


def _validate_v17_terminal(raw: Mapping[str, Any]) -> None:
    if not isinstance(raw, Mapping):
        raise CandidateError("TERMINAL_EVIDENCE", "terminal evidence is not an object")
    if raw.get("status") not in {"pass", "PASS"} and raw.get("terminal_status") not in {"pass", "PASS"}:
        raise CandidateError("TERMINAL_STATUS", "terminal evidence is not PASS")
    contract = raw.get("contract_id", raw.get("contract_version"))
    if contract not in {TERMINAL_CONTRACT, PHASE_CONTRACT}:
        raise CandidateError("TERMINAL_CONTRACT", "terminal contract drift")
    # The v17 fix changes only producer retry disposition.  Host conservation
    # remains the v12 fail-closed invariant and explicitly permits the 78/3
    # non-LiDAR support tail while forbidding residual LiDAR.
    v12.validate_topic_conservation(raw)


def _capture_with_persistence_diagnostics(config: Any, root: Path) -> Mapping[str, Any]:
    result = v15._capture_with_persistence_diagnostics(config, root)
    _validate_schema3_callback(result)
    _validate_v17_terminal(result.get("terminal", {}))
    persistence = result.get("feeder_persistence", {})
    if any(persistence.get(key) is not True for key in ("receipt_present", "stderr_present", "exit_status_present")):
        raise CandidateError("FEEDER_PERSISTENCE", "v16 feeder persistence evidence is incomplete")
    return result


def _verify_authorization(config: Any) -> Mapping[str, Any]:
    if config.authorization_path is None or not config.authorization_sha256:
        raise AuthorizationError("V16_FORMAL_REPLAY_UNAUTHORIZED", "v16 formal authorization is required")
    raise AuthorizationError("V16_FORMAL_REPLAY_UNAUTHORIZED", "v16 authorization is not installed")


def _enriched_atomic_json(path: Path, value: Mapping[str, Any], mode: int = 0o444) -> str:
    if path.name == RECEIPT_NAME:
        enriched = dict(value)
        enriched.update({
            "candidate_version": CANDIDATE_VERSION,
            "v17_contract_version": CONTRACT_VERSION,
            "v16_predecessor_formal_failure": {
                "exit_code": 11,
                "failure_kind": "FORMAL_FAIL_CLOSED",
                "root_reserved": False,
                "docker_started": False,
            },
            "v17_v12_contract_snapshot": snapshot_v12_contracts(),
        })
        value = enriched
    return BASE_ATOMIC_CREATE_JSON(path, value, mode)


@contextmanager
def scoped_runtime() -> Iterator[None]:
    snapshot = _snapshot_v12_state()
    v12.PROFILE_PATH = PROFILE_PATH
    v12.PROFILE_SHA256 = PROFILE_SHA256
    v12.READY_PROFILE_PATH = READY_PROFILE_PATH
    v12.READY_PROFILE_SHA256 = READY_PROFILE_SHA256
    v12.IMAGE_TAG = IMAGE_TAG
    v12.IMAGE_ID = IMAGE_ID
    v12.PHASE_CONTRACT = PHASE_CONTRACT
    v12.TRANSPORT_CONTRACT = TRANSPORT_CONTRACT
    v12.verify_candidate_profile = verify_candidate_profile
    v12.build_safe_docker_argv = build_safe_docker_argv
    v12._production_identity_probe = v15._production_identity_probe
    v12._production_capture = _capture_with_persistence_diagnostics
    v12._production_compose = BASE_PRODUCTION_COMPOSE
    v12._atomic_create_json = _enriched_atomic_json
    try:
        yield
    finally:
        _restore_v12_state(snapshot)


def run_formal(config: Any, *, authorization_validator: Optional[Callable[[Any], Mapping[str, Any]]] = None,
               quiescence_probe: Optional[Callable[[Any], Mapping[str, Any]]] = None,
               **kwargs: Any) -> Mapping[str, Any]:
    """Run v12 lifecycle under v17 state; all runtime seams are injectable."""
    raw_capture = kwargs.pop("raw_capture", None)
    if raw_capture is not None:
        def checked_capture(value: Any, root: Path) -> Mapping[str, Any]:
            result = raw_capture(value, root)
            _validate_schema3_callback(result)
            _validate_v17_terminal(result.get("terminal", {}) if isinstance(result, Mapping) else {})
            return result
        kwargs["raw_capture"] = checked_capture
    with scoped_runtime():
        if quiescence_probe is not None:
            kwargs["_unused_v16_quiescence_probe"] = quiescence_probe
            # The v12 lifecycle calls this symbol directly; replace only for
            # this scope so injected tests never sleep or inspect host state.
            v12._continuous_quiescence = quiescence_probe
        validator = authorization_validator or _verify_authorization
        kwargs.pop("_unused_v16_quiescence_probe", None)
        return v12.run_formal(config, authorization_validator=validator, **kwargs)


def main(argv: Optional[Sequence[str]] = None, *, runtime: Optional[Mapping[str, Any]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--bag", default=INPUT_PATH)
    parser.add_argument("--authorization", type=Path)
    parser.add_argument("--authorization-sha256")
    parser.add_argument("--container-name", default="m6a10-v17-formal-candidate")
    args = parser.parse_args(argv)
    try:
        result = run_formal(
            v12.CandidateConfig(
                root=args.root, repo_root=args.repo_root, bag_path=args.bag,
                profile_path=PROFILE_PATH, container_name=args.container_name,
                authorization_path=args.authorization,
                authorization_sha256=args.authorization_sha256,
            ),
            **dict(runtime or {}),
        )
    except Exception as error:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(error, "kind", "V17_FORMAL_FAIL_CLOSED"),
                          "failure_message": str(error)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in
                      ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
