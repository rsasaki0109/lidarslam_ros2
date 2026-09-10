#!/usr/bin/env python3
"""Unauthorized v15 formal candidate with scoped original-builder state.

The v15 candidate is additive to the immutable v14 launcher and image
lineage.  It captures the original v12 Docker argv callable before applying a
scoped v15 transformation, replaces the output tmpfs with one explicit RW
``/out`` bind, mounts the v15 wrapper/feeder read-only, and supplies the ROS
environment explicitly.  Importing or running the default entry point never
opens the bag or starts Docker; only an exact future authorization may do so.
"""

from __future__ import annotations

from contextlib import contextmanager
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Callable, Dict, Iterator, List, Mapping, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import lidarslam_benchmark_tools.run_fast_livo2_m6a10_v12_formal as v12  # noqa: E402


CANDIDATE_VERSION = "v15-schema3-feeder-scoped-original-builder"
PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_candidate.yaml"
PROFILE_SHA256 = "017e8583e1085b1f11da4117a1c53497cdcb1055c6c6224568df2f73ba751bc7"
READY_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_ready.yaml"
READY_PROFILE_SHA256 = "070fb2b762881a7126caf021c7a98ab59c8fa55e10be220b6482d7f2ef97899f"
IMAGE_TAG = "m6a10-v2c-v15-schema3-feeder-20260823t203230z-correction-fast-livo2-benchmark:ros1-pinned"
IMAGE_ID = "sha256:f103a0f61f7ec9b19cbae02da6c6b8a28f17475dbc3f51fba3206b954ca4ca7a"
BASE_IMAGE_ID = "sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7"
PHASE_CONTRACT = "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
TRANSPORT_CONTRACT = "m6a10-v12-callback-ack-transport-outstanding-v1"
TERMINAL_CONTRACT = "m6a10-fast-livo2-consumer-terminal-v1"
PHASE_MODE = "unpaced_ack"
EXPECTED_COUNTS = {"lidar": 5793, "imu": 225102, "image": 5792}
EXPECTED_MESSAGES = 236687
REQUIRED_END_TIMESTAMP_SECONDS = 1623491515.148352
MAX_END_GAP_SECONDS = 0.25
SENSOR_DURATION_SECONDS = 579.278127298
WATCHDOG_SECONDS = 1200
MONITOR_INTERVAL_SECONDS = 4.0
INPUT_PATH = v12.INPUT_PATH
INPUT_BYTES = v12.INPUT_BYTES
INPUT_SHA256 = v12.INPUT_SHA256
FEEDER_SOURCE = Path("scripts/fast_livo2_m6a10_v15_feeder.py")
FEEDER_SHA256 = "6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7"
WRAPPER_SOURCE = Path("scripts/fast_livo2_m6a10_v15_formal_container_run.sh")
WRAPPER_SHA256 = "d3ab90871422100aa2d89e2469ac413c69346300e2766cc2cc88096e9b45e8f2"
V12_PATCH_SOURCE = Path("docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch")
V12_PATCH_SHA256 = "39c77535a7557365dac6b0f2c99849038b29670a57c3101be3a39138317c6333"
FEEDER_PATH_IN_CONTAINER = "/runner/scripts/fast_livo2_m6a10_feeder.py"
WRAPPER_PATH_IN_CONTAINER = "/runner/v15_runtime.sh"
TIMING_CONTRACT = "m6a10-online-compute-v3-timing-v1"
MONITOR_CONTRACT = "m6a10-v12-host-interference-gate-v1"
CONTRACT_VERSION = "m6a10-v15-formal-candidate-closure-v1"
RECEIPT_NAME = "closure_receipt.json"
AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v15_formal.py"
AUTHORIZER_SHA256 = "2ca414d3dae1a492591038f677ccc046c27196e6150e1685fd3caf5c7c5db9fc"

BUILD_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v15_build_20260823T203230Z_agentv15_correction/"
    "build_identity.receipt.json"
)
BUILD_RECEIPT_SHA256 = "5005c8d392a8591bdfcda8e1de31be3a550a39c61ed22a3ae48455e1a05bce40"
NO_INPUT_PASS_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v15_no_input_correction_20260823T204349Z_agentv15/"
    "no_input_correction.receipt.json"
)
NO_INPUT_PASS_RECEIPT_SHA256 = "e5e1703b6cfcd6b10c9703af06f7d89ede2c638519b285d3ae12152b3e63b5ff"
NO_INPUT_PASS_SIDECAR_SHA256 = "d2cf11c36fb203d3972daeef9f9c2f0fe462a82dfac19070a822c6672fb086a3"
PRIOR_NO_INPUT_FAIL_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v15_build_20260823T203230Z_agentv15_correction/"
    "no_input_identity.receipt.json"
)
PRIOR_NO_INPUT_FAIL_SHA256 = "36782a546e558a537619f183599112354d547f0fd0a70f06c4618ec76aa5a4de"
PRIOR_NO_INPUT_FAIL_SIDECAR_SHA256 = "197ee62c79b397d9f7ef6c6d6be40f1063584dea7cc69ef5c47558991b52320f"
PRIOR_VALIDATOR_FAIL_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v15_no_input_correction_20260823T203806Z_agentv15/"
    "no_input_correction.receipt.json"
)
PRIOR_VALIDATOR_FAIL_SHA256 = "804620192884f097de5fbc4e7946c789af2237c679f38d1ffbdecf616a28aca8"
PRIOR_VALIDATOR_FAIL_SIDECAR_SHA256 = "581f9835c23a200f2a651ded025e3fe783371c4e12ebb334b13b0529d4e0ed7a"

ROS_ENV = {
    "ROS_MASTER_URI": "http://127.0.0.1:11311",
    "ROS_IP": "127.0.0.1",
    "ROS_HOSTNAME": "127.0.0.1",
    "ROS_HOME": "/out/ros_home",
    "ROS_LOG_DIR": "/out/ros_logs",
}

# Capture this before scoped_runtime can change v12's shared symbol.  Tests
# replace it with a counting fake to prove exactly-one builder invocation.
ORIGINAL_V12_BUILD_DOCKER_ARGV = v12.build_safe_docker_argv
BASE_ATOMIC_CREATE_JSON = v12._atomic_create_json

CandidateError = v12.CandidateError
AuthorizationError = v12.AuthorizationError
ProcessFactory = Callable[[Sequence[str], Path], Any]


def _sha256_file(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise CandidateError("SOURCE_MISSING", "not a regular source: %s" % path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


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
    matches = []
    for index, item in enumerate(argv[:-1]):
        if item == "--env" and argv[index + 1].startswith(key + "="):
            matches.append(index + 1)
    if len(matches) != 1:
        raise CandidateError("DOCKER_ENV", "expected one %s environment entry" % key)
    argv[matches[0]] = "%s=%s" % (key, value)


def build_safe_docker_argv(config: Any, output_dir: Path) -> List[str]:
    """Call the captured v12 builder once, then apply the v15 transform."""
    parent_argv = ORIGINAL_V12_BUILD_DOCKER_ARGV(config, output_dir)
    argv: List[str] = []
    index = 0
    wrapper_source = Path(config.repo_root) / WRAPPER_SOURCE
    feeder_source = Path(config.repo_root) / FEEDER_SOURCE
    while index < len(parent_argv):
        item = parent_argv[index]
        if item == "--tmpfs" and index + 1 < len(parent_argv):
            tmpfs = parent_argv[index + 1]
            if tmpfs.startswith("/out:"):
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
        if item == v12.IMAGE_ID:
            argv.append(IMAGE_ID)
        else:
            argv.append(item)
        index += 1

    _replace_env_value(argv, "M6A10_PROFILE_PATH", "configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_ready.yaml")
    _replace_env_value(argv, "M6A10_PROFILE_SHA256", READY_PROFILE_SHA256)
    _replace_env_value(argv, "M6A10_FAST_FEEDER_SHA256", FEEDER_SHA256)
    for key, value in ROS_ENV.items():
        if any(item == "--env" and argv[index + 1].startswith(key + "=")
               for index, item in enumerate(argv[:-1])):
            _replace_env_value(argv, key, value)
        else:
            entrypoint_index = argv.index("--entrypoint")
            argv[entrypoint_index:entrypoint_index] = ["--env", "%s=%s" % (key, value)]

    mounts = _mounts(argv)
    destinations = [_mount_destination(spec) for spec in mounts]
    expected_destinations = {
        "/input/ntu_viral.bag", "/out", WRAPPER_PATH_IN_CONTAINER,
        FEEDER_PATH_IN_CONTAINER,
    }
    if len(destinations) != len(set(destinations)) or set(destinations) != expected_destinations:
        raise CandidateError("DOCKER_ARGV", "v15 mount destination allowlist drift")
    output_mounts = [spec for spec in mounts if _mount_destination(spec) == "/out"]
    expected_output = "type=bind,src=%s,dst=/out,readonly=false" % output_dir.resolve()
    if output_mounts != [expected_output]:
        raise CandidateError("DOCKER_ARGV", "v15 output bind contract drift")
    if _tmpfs_destinations(argv) != ["/tmp", "/root/.ros"]:
        raise CandidateError("DOCKER_ARGV", "v15 tmpfs allowlist drift")
    if "/out" in _tmpfs_destinations(argv) or "--rm" in argv or any(item == "rw" for item in argv):
        raise CandidateError("DOCKER_ARGV", "unsafe cleanup or output tmpfs token")
    if argv[:3] != ["docker", "run", "--name"] or argv[-1] != IMAGE_ID:
        raise CandidateError("IMAGE_IDENTITY", "v15 image identity drift")
    if "--network" not in argv or argv[argv.index("--network") + 1] != "none":
        raise CandidateError("DOCKER_ARGV", "network is not none")
    if "--read-only" not in argv or "--init" not in argv or "--pull=never" not in argv:
        raise CandidateError("DOCKER_ARGV", "container safety flags drift")
    forbidden = {"--privileged", "--cap-add", "--device", "--pid=host", "--ipc=host", "--uts=host"}
    if any(item in forbidden for item in argv):
        raise CandidateError("DOCKER_ARGV", "unsafe capability or namespace flag")
    if argv[argv.index("--entrypoint") + 1] != WRAPPER_PATH_IN_CONTAINER:
        raise CandidateError("DOCKER_ARGV", "v15 entrypoint drift")
    if any(item in {"sh", "bash", "-c", "-lc", "--shell"} for item in argv):
        raise CandidateError("DOCKER_ARGV", "shell execution token is forbidden")
    env_values = [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == "--env"]
    if any(value not in env_values for value in [
            "ROS_MASTER_URI=%s" % ROS_ENV["ROS_MASTER_URI"],
            "ROS_IP=%s" % ROS_ENV["ROS_IP"],
            "ROS_HOSTNAME=%s" % ROS_ENV["ROS_HOSTNAME"],
            "ROS_HOME=%s" % ROS_ENV["ROS_HOME"],
            "ROS_LOG_DIR=%s" % ROS_ENV["ROS_LOG_DIR"]]):
        raise CandidateError("DOCKER_ENV", "explicit ROS environment is incomplete")
    return argv


@contextmanager
def scoped_runtime() -> Iterator[None]:
    """Patch shared v12 runtime only inside this scope and always restore."""
    original = v12.build_safe_docker_argv
    v12.build_safe_docker_argv = build_safe_docker_argv
    try:
        yield
    finally:
        v12.build_safe_docker_argv = original


def verify_candidate_profile(profile_path: Path = PROFILE_PATH,
                             repo_root: Path = ROOT) -> Mapping[str, Any]:
    if profile_path.resolve() != PROFILE_PATH.resolve():
        raise CandidateError("PROFILE_PATH", "v15 candidate profile path differs")
    observed = _sha256_file(profile_path)
    if observed != PROFILE_SHA256:
        raise CandidateError("PROFILE_SHA256", "v15 candidate profile SHA differs")
    import yaml
    try:
        document = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise CandidateError("PROFILE_INVALID", "v15 candidate profile YAML invalid") from error
    candidate = document.get("formal_candidate") if isinstance(document, Mapping) else None
    image = document.get("image") if isinstance(document, Mapping) else None
    phase = document.get("phase") if isinstance(document, Mapping) else None
    if document.get("status") != "V15_FORMAL_CANDIDATE_UNAUTHORIZED" or \
            document.get("formal_replay_forbidden") is not True or \
            document.get("formal_replay_authorized") is not False or \
            document.get("replay_count") != 0 or \
            not isinstance(candidate, Mapping) or candidate.get("status") != "UNAUTHORIZED_NOT_RUN" or \
            not isinstance(image, Mapping) or image.get("id") != IMAGE_ID or image.get("tag") != IMAGE_TAG or \
            not isinstance(phase, Mapping) or phase.get("contract_version") != PHASE_CONTRACT:
        raise CandidateError("PROFILE", "v15 candidate profile contract drift")
    _sha256_file(READY_PROFILE_PATH)
    if _sha256_file(READY_PROFILE_PATH) != READY_PROFILE_SHA256:
        raise CandidateError("PROFILE_PREDECESSOR", "v15 ready profile drift")
    for relative, expected, label in ((FEEDER_SOURCE, FEEDER_SHA256, "v15 feeder"),
                                      (WRAPPER_SOURCE, WRAPPER_SHA256, "v15 wrapper"),
                                      (V12_PATCH_SOURCE, V12_PATCH_SHA256, "v12 patch")):
        if _sha256_file(repo_root / relative) != expected:
            raise CandidateError("SOURCE_DRIFT", "%s drift" % label)
    return {"path": str(profile_path.resolve()), "sha256": observed, "document": document}


def _capture_with_persistence_diagnostics(config: Any, root: Path) -> Mapping[str, Any]:
    paths = {
        "feeder": root / "out/feeder_receipt.json",
        "callback": root / "out/callback_consumer_evidence.json",
        "terminal": root / "out/consumer_evidence.json",
        "timing": root / "out/online_compute_timing.json",
        "feeder_log": root / "out/feeder.log",
        "feeder_exit_status": root / "out/feeder_exit_status.txt",
    }
    values: Dict[str, Any] = {}
    bindings: Dict[str, Dict[str, Any]] = {}
    missing: List[str] = []
    for label, path in paths.items():
        if path.is_symlink() or not path.is_file():
            missing.append(label)
            continue
        raw = path.read_bytes()
        bindings[label] = {"path": str(path), "bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest()}
        if label in {"feeder", "callback", "terminal", "timing"}:
            try:
                values[label] = json.loads(raw.decode("utf-8"))
            except (UnicodeError, json.JSONDecodeError):
                values[label] = {"_invalid_json": True}
        else:
            values[label] = raw.decode("utf-8", "replace").strip()
    return {
        "terminal": values.get("terminal", {}), "documents": values,
        "bindings": bindings, "missing": missing,
        "feeder_persistence": {
            "receipt_present": "feeder" not in missing,
            "stderr_present": "feeder_log" not in missing,
            "exit_status_present": "feeder_exit_status" not in missing,
        },
        "feeder_root_cause_fixed": False,
        "feeder_underlying_nonzero_cause": "unknown",
    }


def _production_identity_probe(config: Any) -> Mapping[str, Any]:
    result = subprocess.run(["docker", "image", "inspect", IMAGE_ID], cwd=str(config.repo_root),
                            shell=False, capture_output=True, check=False)
    if result.returncode != 0:
        raise CandidateError("IMAGE_IDENTITY_FAIL_CLOSED", "v15 image inspect failed")
    try:
        image = json.loads(result.stdout.decode("utf-8"))[0]
        labels = image.get("Config", {}).get("Labels", {}) or {}
        tags = image.get("RepoTags", []) or []
    except (UnicodeError, json.JSONDecodeError, IndexError, TypeError, AttributeError) as error:
        raise CandidateError("IMAGE_IDENTITY_INVALID", "v15 image inspect malformed") from error
    required = {
        "benchmark.fast_livo2.m6a10_v15_feeder_sha256": FEEDER_SHA256,
        "benchmark.fast_livo2.m6a10_v15_wrapper_sha256": WRAPPER_SHA256,
        "benchmark.fast_livo2.m6a10_v15_profile_sha256": READY_PROFILE_SHA256,
        "benchmark.fast_livo2.m6a10_phase_contract": PHASE_CONTRACT,
        "benchmark.fast_livo2.m6a10_transport_contract": TRANSPORT_CONTRACT,
        "benchmark.fast_livo2.m6a10_formal_replay_forbidden": "true",
        "benchmark.fast_livo2.m6a10_input_present": "false",
        "benchmark.fast_livo2.m6a10_ground_truth_present": "false",
        "benchmark.fast_livo2.m6a10_scorer_present": "false",
        "benchmark.fast_livo2.m6a10_map_present": "false",
    }
    if image.get("Id") != IMAGE_ID or IMAGE_TAG not in tags or any(labels.get(k) != v for k, v in required.items()):
        raise CandidateError("IMAGE_IDENTITY_DRIFT", "v15 image ID/tag/labels drift")
    return {"image_id": IMAGE_ID, "tag": IMAGE_TAG, "labels": labels, "opened": False}


def _verify_authorization(config: Any) -> Mapping[str, Any]:
    if config.authorization_path is None or not config.authorization_sha256:
        raise AuthorizationError("FORMAL_REPLAY_UNAUTHORIZED", "v15 authorization is required")
    if _sha256_file(AUTHORIZER_PATH) != AUTHORIZER_SHA256:
        raise AuthorizationError("AUTHORIZER_DRIFT", "v15 authorizer source drift")
    spec = importlib.util.spec_from_file_location("m6a10_v15_authorizer_runtime", AUTHORIZER_PATH)
    if spec is None or spec.loader is None:
        raise AuthorizationError("AUTHORIZER_LOAD", "v15 authorizer cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    value = module.verify_authorization(config.authorization_path, config.root,
                                        config.authorization_sha256, repo_root=config.repo_root)
    if value.get("authorized") is not True or value.get("formal_execution") is not True:
        raise AuthorizationError("AUTHORIZATION_STATUS", "v15 authorization is not executable")
    return value


def _enriched_atomic_json(path: Path, value: Mapping[str, Any], mode: int = 0o444) -> str:
    if path.name == RECEIPT_NAME:
        enriched = dict(value)
        enriched.update({
            "candidate_version": CANDIDATE_VERSION,
            "v15_builder_fix": "capture_original_before_patch_and_call_original_only",
            "feeder_root_cause_fixed": False,
            "feeder_underlying_nonzero_cause": "unknown",
            "v15_image_id": IMAGE_ID,
            "v15_build_receipt_sha256": BUILD_RECEIPT_SHA256,
            "v15_no_input_pass_receipt_sha256": NO_INPUT_PASS_RECEIPT_SHA256,
        })
        value = enriched
    return BASE_ATOMIC_CREATE_JSON(path, value, mode)


def _base_snapshot() -> Dict[str, Any]:
    return {name: getattr(v12, name) for name in (
        "PROFILE_PATH", "PROFILE_SHA256", "READY_PROFILE_PATH", "READY_PROFILE_SHA256",
        "IMAGE_TAG", "IMAGE_ID", "PHASE_CONTRACT", "TRANSPORT_CONTRACT",
        "CONTRACT_VERSION", "RECEIPT_NAME", "verify_candidate_profile",
        "build_safe_docker_argv", "_production_identity_probe", "_production_capture",
        "_production_compose", "_atomic_create_json",
    )}


def _restore_base(snapshot: Mapping[str, Any]) -> None:
    for name, value in snapshot.items():
        setattr(v12, name, value)


def run_injected_once(config: Any, process_factory: ProcessFactory) -> Mapping[str, Any]:
    output_dir = Path(config.root) / "out"
    output_dir.mkdir(parents=True, exist_ok=False)
    with scoped_runtime():
        argv = v12.build_safe_docker_argv(config, output_dir)
        process = process_factory(argv, Path(config.repo_root))
        pid = getattr(process, "pid", None)
        if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 0:
            raise CandidateError("RUNNER_PID", "injected process has no positive PID")
        returncode = int(process.wait())
    return {"argv": list(argv), "pid": pid, "returncode": returncode,
            "start_count": 1, "popen_count": 1}


def run_formal(config: Any, **kwargs: Any) -> Mapping[str, Any]:
    """Run the v12 lifecycle under scoped v15 identity and authorization."""
    snapshot = _base_snapshot()
    v12.PROFILE_PATH = PROFILE_PATH
    v12.PROFILE_SHA256 = PROFILE_SHA256
    v12.READY_PROFILE_PATH = READY_PROFILE_PATH
    v12.READY_PROFILE_SHA256 = READY_PROFILE_SHA256
    v12.IMAGE_TAG = IMAGE_TAG
    v12.IMAGE_ID = IMAGE_ID
    v12.PHASE_CONTRACT = PHASE_CONTRACT
    v12.TRANSPORT_CONTRACT = TRANSPORT_CONTRACT
    v12.CONTRACT_VERSION = CONTRACT_VERSION
    v12.RECEIPT_NAME = RECEIPT_NAME
    v12.verify_candidate_profile = verify_candidate_profile
    v12.build_safe_docker_argv = build_safe_docker_argv
    v12._production_identity_probe = _production_identity_probe
    v12._production_capture = _capture_with_persistence_diagnostics
    v12._atomic_create_json = _enriched_atomic_json
    try:
        return v12.run_formal(config, authorization_validator=_verify_authorization, **kwargs)
    finally:
        _restore_base(snapshot)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--bag", default=INPUT_PATH)
    parser.add_argument("--authorization", type=Path)
    parser.add_argument("--authorization-sha256")
    parser.add_argument("--container-name", default="m6a10-v15-formal-candidate")
    args = parser.parse_args(argv)
    try:
        result = run_formal(v12.CandidateConfig(
            root=args.root, repo_root=args.repo_root, bag_path=args.bag,
            profile_path=PROFILE_PATH, container_name=args.container_name,
            authorization_path=args.authorization,
            authorization_sha256=args.authorization_sha256,
        ))
    except Exception as error:
        print(json.dumps({"status": "FAIL_CLOSED",
                          "failure_kind": getattr(error, "kind", "FORMAL_FAIL_CLOSED"),
                          "failure_message": str(error)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in
                      ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
