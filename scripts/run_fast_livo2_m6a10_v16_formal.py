#!/usr/bin/env python3
"""Unauthorized v16 formal candidate with an explicit v12 contract snapshot.

v16 is additive to the immutable v15 lineage.  The v15 formal start failed
before root reservation because its snapshot requested ``CONTRACT_VERSION``
from v12 even though v12 exposes only ``PHASE_CONTRACT`` and
``TRANSPORT_CONTRACT``.  This candidate constructs its snapshot from those
real v12 symbols explicitly; it has no attribute fallback and never mutates
v15 files.

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


CANDIDATE_VERSION = "v16-explicit-v12-contract-snapshot"
CONTRACT_VERSION = "m6a10-v16-formal-candidate-closure-v1"
PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v16_formal_candidate.yaml"
PROFILE_SHA256 = "a9ca34982cfb22fe7e3bb0e2a5437a10875990926fddb820fb6a29f6abc6461f"
READY_PROFILE_PATH = v15.READY_PROFILE_PATH
READY_PROFILE_SHA256 = v15.READY_PROFILE_SHA256
IMAGE_TAG = v15.IMAGE_TAG
IMAGE_ID = v15.IMAGE_ID
BASE_IMAGE_ID = v15.BASE_IMAGE_ID
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
FEEDER_SHA256 = v15.FEEDER_SHA256
WRAPPER_SOURCE = Path("scripts/fast_livo2_m6a10_v15_formal_container_run.sh")
WRAPPER_SHA256 = v15.WRAPPER_SHA256
V12_PATCH_SOURCE = Path("docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch")
V12_PATCH_SHA256 = v15.V12_PATCH_SHA256
WRAPPER_PATH_IN_CONTAINER = v15.WRAPPER_PATH_IN_CONTAINER
FEEDER_PATH_IN_CONTAINER = v15.FEEDER_PATH_IN_CONTAINER
TIMING_CONTRACT = v15.TIMING_CONTRACT
MONITOR_CONTRACT = v15.MONITOR_CONTRACT
RECEIPT_NAME = "closure_receipt.json"

V15_LAUNCHER_PATH = ROOT / "scripts/run_fast_livo2_m6a10_v15_formal.py"
V15_LAUNCHER_SHA256 = "a16e88158dd140e9d61ebb329e168514a6003b7d2e5564a149a48824e523130b"
V15_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_candidate.yaml"
V15_PROFILE_SHA256 = "017e8583e1085b1f11da4117a1c53497cdcb1055c6c6224568df2f73ba751bc7"
V15_AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v15_formal.py"
V15_AUTHORIZER_SHA256 = "2ca414d3dae1a492591038f677ccc046c27196e6150e1685fd3caf5c7c5db9fc"

BUILD_RECEIPT_PATH = v15.BUILD_RECEIPT_PATH
BUILD_RECEIPT_SHA256 = v15.BUILD_RECEIPT_SHA256
BUILD_RECEIPT_SIDECAR_SHA256 = "6940f5762e87bf9f08752b950f12de2ae8423c9b0217841e212af56f61a04c65"
V15_AUTHORIZATION_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v15_formal_authorization_20260823T210707Z_agentv15/"
    "formal_authorization.receipt.json"
)
V15_AUTHORIZATION_SHA256 = "60db8d6328481b4b3d6b9c85711ca02c60fa4803e3bb62005f5d10bb36714613"
V15_AUTHORIZATION_SIDECAR_SHA256 = "496e46ac106db3c8f9427156b623f6aed313e6e983ad581fc5ffea268a78fc22"
NO_INPUT_PASS_RECEIPT_PATH = v15.NO_INPUT_PASS_RECEIPT_PATH
NO_INPUT_PASS_RECEIPT_SHA256 = v15.NO_INPUT_PASS_RECEIPT_SHA256
NO_INPUT_PASS_SIDECAR_SHA256 = v15.NO_INPUT_PASS_SIDECAR_SHA256

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
    """Call the captured v12 builder once and apply the v15-safe transform."""
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

    _replace_env_value(argv, "M6A10_PROFILE_PATH", "configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_ready.yaml")
    _replace_env_value(argv, "M6A10_PROFILE_SHA256", READY_PROFILE_SHA256)
    _replace_env_value(argv, "M6A10_FAST_FEEDER_SHA256", FEEDER_SHA256)
    for key, value in v15.ROS_ENV.items():
        if any(item == "--env" and argv[index + 1].startswith(key + "=")
               for index, item in enumerate(argv[:-1])):
            _replace_env_value(argv, key, value)
        else:
            entrypoint_index = argv.index("--entrypoint")
            argv[entrypoint_index:entrypoint_index] = ["--env", "%s=%s" % (key, value)]

    mounts = _mounts(argv)
    destinations = [_mount_destination(spec) for spec in mounts]
    expected_destinations = {"/input/ntu_viral.bag", "/out", WRAPPER_PATH_IN_CONTAINER, FEEDER_PATH_IN_CONTAINER}
    if len(destinations) != len(set(destinations)) or set(destinations) != expected_destinations:
        raise CandidateError("DOCKER_ARGV", "v16 mount destination allowlist drift")
    output_mounts = [spec for spec in mounts if _mount_destination(spec) == "/out"]
    expected_output = "type=bind,src=%s,dst=/out,readonly=false" % output_dir.resolve()
    if output_mounts != [expected_output]:
        raise CandidateError("DOCKER_ARGV", "v16 output bind contract drift")
    if _tmpfs_destinations(argv) != ["/tmp", "/root/.ros"] or "/out" in _tmpfs_destinations(argv):
        raise CandidateError("DOCKER_ARGV", "v16 tmpfs allowlist drift")
    if "--rm" in argv or any(item == "rw" for item in argv):
        raise CandidateError("DOCKER_ARGV", "unsafe cleanup or bare rw token")
    if argv[:3] != ["docker", "run", "--name"] or argv[-1] != IMAGE_ID:
        raise CandidateError("IMAGE_IDENTITY", "v16 image identity drift")
    if "--network" not in argv or argv[argv.index("--network") + 1] != "none":
        raise CandidateError("DOCKER_ARGV", "network is not none")
    if "--read-only" not in argv or "--init" not in argv or "--pull=never" not in argv:
        raise CandidateError("DOCKER_ARGV", "container safety flags drift")
    forbidden = {"--privileged", "--cap-add", "--device", "--pid=host", "--ipc=host", "--uts=host"}
    if any(item in forbidden for item in argv):
        raise CandidateError("DOCKER_ARGV", "unsafe capability or namespace flag")
    if argv[argv.index("--entrypoint") + 1] != WRAPPER_PATH_IN_CONTAINER:
        raise CandidateError("DOCKER_ARGV", "v16 entrypoint drift")
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
    """Expose the exact phase/transport symbols used by v16."""
    return {"phase_contract": v12.PHASE_CONTRACT, "transport_contract": v12.TRANSPORT_CONTRACT}


def verify_candidate_profile(profile_path: Path = PROFILE_PATH,
                             repo_root: Path = ROOT) -> Mapping[str, Any]:
    if profile_path.resolve() != PROFILE_PATH.resolve():
        raise CandidateError("PROFILE_PATH", "v16 candidate profile path differs")
    observed = _sha256_file(profile_path)
    if observed != PROFILE_SHA256:
        raise CandidateError("PROFILE_SHA256", "v16 candidate profile SHA differs")
    import yaml
    try:
        document = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise CandidateError("PROFILE_INVALID", "v16 candidate profile YAML invalid") from exc
    candidate = document.get("formal_candidate") if isinstance(document, Mapping) else None
    image = document.get("image") if isinstance(document, Mapping) else None
    phase = document.get("phase") if isinstance(document, Mapping) else None
    predecessor_failure = candidate.get("predecessor_failure") if isinstance(candidate, Mapping) else None
    if document.get("status") != "V16_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            document.get("formal_replay_forbidden") is not True or \
            document.get("formal_replay_authorized") is not False or \
            document.get("replay_count") != 0 or \
            not isinstance(candidate, Mapping) or candidate.get("status") != "UNAUTHORIZED_NOT_RUN" or \
            candidate.get("predecessor_profile_sha256") != V15_PROFILE_SHA256 or \
            candidate.get("predecessor_launcher_sha256") != V15_LAUNCHER_SHA256 or \
            candidate.get("nonexistent_v12_attribute_rejected") != "CONTRACT_VERSION" or \
            not isinstance(predecessor_failure, Mapping) or predecessor_failure.get("exit_code") != 11 or \
            predecessor_failure.get("root_reserved") is not False or \
            not isinstance(image, Mapping) or image.get("id") != IMAGE_ID or image.get("tag") != IMAGE_TAG or \
            not isinstance(phase, Mapping) or phase.get("contract_version") != PHASE_CONTRACT or \
            phase.get("transport", {}).get("contract_version") != TRANSPORT_CONTRACT:
        raise CandidateError("PROFILE", "v16 candidate profile contract drift")
    source_pins = {
        "v15_launcher": (V15_LAUNCHER_PATH, V15_LAUNCHER_SHA256),
        "v15_profile": (V15_PROFILE_PATH, V15_PROFILE_SHA256),
        "v15_authorizer": (V15_AUTHORIZER_PATH, V15_AUTHORIZER_SHA256),
        "v15_feeder": (repo_root / FEEDER_SOURCE, FEEDER_SHA256),
        "v15_wrapper": (repo_root / WRAPPER_SOURCE, WRAPPER_SHA256),
        "v12_patch": (repo_root / V12_PATCH_SOURCE, V12_PATCH_SHA256),
    }
    source_section = document.get("source")
    if not isinstance(source_section, Mapping):
        raise CandidateError("PROFILE_SOURCES", "v16 source pins are missing")
    for label, (path, expected) in source_pins.items():
        entry = source_section.get("%s_path" % label)
        entry_sha = source_section.get("%s_sha256" % label)
        try:
            relative_path = str(path.relative_to(ROOT))
        except ValueError:
            relative_path = str(path)
        if entry is not None and entry != relative_path:
            raise CandidateError("SOURCE_PIN", "%s path drift" % label)
        if entry_sha != expected or _sha256_file(path) != expected:
            raise CandidateError("SOURCE_DRIFT", "%s SHA drift" % label)
    build = document.get("evidence", {}).get("build_receipt")
    auth = document.get("evidence", {}).get("v15_authorization")
    no_input = document.get("evidence", {}).get("v15_no_input_pass")
    for label, entry, path, expected, sidecar in (
            ("build", build, BUILD_RECEIPT_PATH, BUILD_RECEIPT_SHA256, BUILD_RECEIPT_SIDECAR_SHA256),
            ("authorization", auth, V15_AUTHORIZATION_PATH, V15_AUTHORIZATION_SHA256,
             V15_AUTHORIZATION_SIDECAR_SHA256),
            ("no-input", no_input, NO_INPUT_PASS_RECEIPT_PATH, NO_INPUT_PASS_RECEIPT_SHA256,
             NO_INPUT_PASS_SIDECAR_SHA256)):
        if not isinstance(entry, Mapping) or entry.get("path") != str(path) or entry.get("sha256") != expected or \
                entry.get("sidecar_sha256") != sidecar:
            raise CandidateError("RECEIPT_PIN", "%s receipt pin drift" % label)
    build_value = _verify_json_receipt(BUILD_RECEIPT_PATH, BUILD_RECEIPT_SHA256,
                                       BUILD_RECEIPT_SIDECAR_SHA256, "v15 build")
    if build_value.get("status") != "PASS" or build_value.get("image", {}).get("id") != IMAGE_ID or \
            build_value.get("execution", {}).get("formal_replay_started") is not False:
        raise CandidateError("BUILD_RECEIPT", "v15 build lineage is not admissible")
    auth_value = _verify_json_receipt(V15_AUTHORIZATION_PATH, V15_AUTHORIZATION_SHA256,
                                      V15_AUTHORIZATION_SIDECAR_SHA256, "v15 authorization")
    if auth_value.get("status") != "AUTHORIZED" or auth_value.get("formal_replay_started") is not False:
        raise CandidateError("AUTHORIZATION_LINEAGE", "v15 authorization lineage drift")
    no_input_value = _verify_json_receipt(NO_INPUT_PASS_RECEIPT_PATH, NO_INPUT_PASS_RECEIPT_SHA256,
                                          NO_INPUT_PASS_SIDECAR_SHA256, "v15 no-input")
    if no_input_value.get("status") != "PASS" or no_input_value.get("safety", {}).get("formal_replay_started") is not False:
        raise CandidateError("NO_INPUT_LINEAGE", "v15 no-input lineage drift")
    return {"path": str(profile_path.resolve()), "sha256": observed,
            "phase_contract": PHASE_CONTRACT, "transport_contract": TRANSPORT_CONTRACT}


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


def _capture_with_persistence_diagnostics(config: Any, root: Path) -> Mapping[str, Any]:
    result = v15._capture_with_persistence_diagnostics(config, root)
    _validate_schema3_callback(result)
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
            "v16_contract_version": CONTRACT_VERSION,
            "v15_predecessor_formal_failure": {
                "exit_code": 11,
                "failure_kind": "FORMAL_FAIL_CLOSED",
                "root_reserved": False,
                "docker_started": False,
            },
            "v16_v12_contract_snapshot": snapshot_v12_contracts(),
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
    """Run v12 lifecycle under v16 state; all runtime seams are injectable."""
    raw_capture = kwargs.pop("raw_capture", None)
    if raw_capture is not None:
        def checked_capture(value: Any, root: Path) -> Mapping[str, Any]:
            result = raw_capture(value, root)
            _validate_schema3_callback(result)
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
    parser.add_argument("--container-name", default="m6a10-v16-formal-candidate")
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
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": getattr(error, "kind", "V16_FORMAL_FAIL_CLOSED"),
                          "failure_message": str(error)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in
                      ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
