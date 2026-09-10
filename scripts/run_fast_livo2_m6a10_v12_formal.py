#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""Unauthorized v12 formal-candidate lifecycle with injected test seams.

The v12 ready profile and all prior receipts remain immutable.  This additive
candidate deliberately refuses formal authorization by default, and therefore
does not inspect an input or start a process in its production entry point.
Tests may inject an authorization decision, identity probes, a process factory,
raw capture, and composition to exercise the lifecycle without Docker, ROS, or
benchmark data.  The one-start lifecycle is shell-free and observes host
interference without stopping the runner when contamination is detected.
"""

from __future__ import annotations

import argparse
import datetime as _datetime
from dataclasses import dataclass
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from lidarslam_benchmark_tools.monitor_m6a10_host_interference import (  # noqa: E402
    HostInterferenceMonitor,
    InterferenceError,
)
import lidarslam_benchmark_tools.check_m6a10_quiescence as _quiescence  # noqa: E402


PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_candidate.yaml"
PROFILE_SHA256 = "93bba16eb88d13a8f02de2178ae5c9a8e8466a4c5ad4fc2e730875e2ebd8c1ed"
READY_PROFILE_PATH = ROOT / "configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_ready.yaml"
READY_PROFILE_SHA256 = "675996a7a5fd81d59d752de4bbea1d4373487a17d03ee085aa08ae9493a0b28d"
PROFILE_KEY = "m6a10_fast_livo2_v2c_v12_formal_candidate"
PHASE_CONTRACT = "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
TRANSPORT_CONTRACT = "m6a10-v12-callback-ack-transport-outstanding-v1"
TERMINAL_CONTRACT = "m6a10-fast-livo2-consumer-terminal-v1"
PHASE_MODE = "unpaced_ack"
IMAGE_TAG = (
    "m6a10-v2c-v12-nonlidar-boundary-transport-20260824-"
    "fast-livo2-benchmark:ros1-pinned"
)
IMAGE_ID = "sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7"
BASE_IMAGE_ID = "sha256:729a7bba2127fc6517c106d59a12668294aeee1a0b31d04d31f9c25c762f6c3a"
EXPECTED_COUNTS = {"lidar": 5793, "imu": 225102, "image": 5792}
EXPECTED_MESSAGES = 236687
REQUIRED_END_TIMESTAMP_SECONDS = 1623491515.148352
MAX_END_GAP_SECONDS = 0.25
SENSOR_DURATION_SECONDS = 579.278127298
MONITOR_INTERVAL_SECONDS = 4.0
WATCHDOG_SECONDS = 1200
INPUT_PATH = (
    "/media/sasaki/aiueo1/datasets/ntu_viral_release/"
    "tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag"
)
INPUT_BYTES = 11290464091
INPUT_SHA256 = "5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310"
FEEDER_PATH_IN_CONTAINER = "/runner/scripts/fast_livo2_m6a10_feeder.py"
WRAPPER_PATH_IN_CONTAINER = "/runner/v12_runtime.sh"
TIMING_CONTRACT = "m6a10-online-compute-v3-timing-v1"
MONITOR_CONTRACT = "m6a10-v12-host-interference-gate-v1"
AUTHORIZER_PATH = ROOT / "scripts/authorize_fast_livo2_m6a10_v12_formal.py"
# This pin is intentionally one-way: the authorization receipt does not pin
# this launcher, avoiding a self-referential receipt/source hash cycle.
AUTHORIZER_SHA256 = "3b32a56050fb8ca8f432763493364b0430253331c9411195ec075c0972f39993"

BUILD_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v12_build_20260823T161850Z_agentv12/build_identity.receipt.json"
)
BUILD_RECEIPT_SHA256 = "e0e5c924025a24083661a6838bc5af28f07c34c419ce6e103c37890ed1382dda"
BUILD_RECEIPT_SIDECAR_SHA256 = "5e79d3ba1145b58ca07e78256fd55593a77e1b7ebacb8e6d349d644ee578efb7"
ATTEMPT6_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v12_dual_service_attempt6_20260823T175000Z_agentv12/"
    "no_input_dual_evidence.receipt.json"
)
ATTEMPT6_RECEIPT_SHA256 = "c5afa2452f96588bf46dc7f0d0b4504e816d0069bbd3a107009eb2c3beac0b45"
ATTEMPT6_SIDECAR_SHA256 = "076cd96e10c78d0bf73a6a7ed66d3549aaa2dd0f75c7503d5d88e19237d608e3"
ATTEMPT6_CALLBACK_SHA256 = "7b61b2299c3f9c79ddae33e9ae1bbf14855805b89e46a7e56d99ffba8cdfa84c"
ATTEMPT6_TERMINAL_SHA256 = "f843ce9bed5b16e4d44d6b7b7f9da945282ef0d3801ae94aa525717669b128b4"
ATTEMPT6_JOURNAL_SHA256 = "f6608931454694d39cbf0fa5fc6f9af6db791d5ca2d8ffa9fed02608e026c26a"
ATTEMPT6_RUNNER_SHA256 = "f3c41d6533475229a765fe92682d483758347053c6f146dd148fc85a6274648a"
HOST_GATE_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v12_host_evidence_gate_20260823T180718Z_agentv12/"
    "host_evidence_gate.receipt.json"
)
HOST_GATE_RECEIPT_SHA256 = "3a603bc4ece0e9167b0b3943df6be569f2e64bce196beb18da29ff09ad69f643"
HOST_GATE_SIDECAR_SHA256 = "ff50620daf076ed357032111d33197da1903b840605fa34259f3880ae0e49f66"

SOURCE_PINS = {
    "v12_ready_profile": ("configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_ready.yaml", "675996a7a5fd81d59d752de4bbea1d4373487a17d03ee085aa08ae9493a0b28d"),
    "v12_delta_patch": ("docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch", "39c77535a7557365dac6b0f2c99849038b29670a57c3101be3a39138317c6333"),
    "v12_build_script": ("scripts/build_fast_livo2_m6a10_v12_image.sh", "4a88bbfd68d5b021b5b3a194f42cfbae67816b31c92e1bb0036b10ed6b62e41b"),
    "v12_dockerfile": ("docker/fast_livo2_m6a10_v12.Dockerfile", "f579d6b09868a67fa33c626d23295de41c5517490f7849c2fafdb52203aaae26"),
    "v12_terminal_selftest": ("tools/m6a10_terminal_support_context_v12_selftest.cpp", "1b42ed713dbb2a8614902ae6214d5790ab9c73ac59792f4d68753320cef47a91"),
    "v12_consumer_selftest": ("tools/m6a10_consumer_evidence_v12_selftest.cpp", "02078640496daa28f0be471355c6e2ad2822a76199cda1b54e738eeff5fe48de"),
    "v12_validator": ("scripts/benchmark_phase_contract.py", "d601a89d1cdc6b6d90483005a65c71e6f9b0a6171bccab2085dcec83eac2dfe7"),
    "v12_wrapper": ("scripts/fast_livo2_m6a10_v12_formal_container_run.sh", "32fce2c054b92f0d695fd4d32c7737ab75dd3a467fdee1c579c82c63398894e6"),
    "v12_no_input_payload": ("scripts/fast_livo2_m6a10_v12_no_input_container_payload.sh", "2c1c9b86fadc0d1d83e7874542f92ff33b99b23421397374ecd9e880ad1b73aa"),
    "v12_feeder": ("scripts/fast_livo2_m6a10_feeder.py", "869ca54921c86310af5cefc4ef0c4f8626b5fdcc125dcd60e865fdf1e677ddbf"),
    "binder": ("scripts/bind_fast_livo2_v12_consumer_evidence.py", "3a708c87137f81330b479579dbcb184ac25853596546263c406473dbef106aab"),
    "compositor": ("scripts/compose_fast_livo2_v12_terminal_evidence.py", "e37c2f22ac6220f7cfc5739d27f7c24360c395bac7b2dd0f90ff80ca9e8db99f"),
    "monitor": ("scripts/monitor_m6a10_host_interference.py", "d19cac9b7755b30e7cf45adcdb3fe7869855c4eb9814e337a50b102048dd5e7d"),
    "host_evidence_gate_runner": ("scripts/run_fast_livo2_m6a10_v12_host_evidence_gate.py", "9b542e1de68846b25d7307633fb38fa7f2e104a495da6e1ea23569a633fe6324"),
    "no_input_runner": ("scripts/run_fast_livo2_m6a10_v12_no_input_gate.py", "f3c41d6533475229a765fe92682d483758347053c6f146dd148fc85a6274648a"),
}


class CandidateError(ValueError):
    """Fail-closed candidate error with a stable machine-readable kind."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


class AuthorizationError(CandidateError):
    pass


def _error(kind: str, message: str) -> CandidateError:
    return CandidateError(kind, message)


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _reject_symlink_components(path: Path, label: str) -> None:
    absolute = path.absolute()
    current = Path(absolute.anchor)
    for component in absolute.parts[1:]:
        current /= component
        if current.is_symlink():
            raise _error("SYMLINK_REJECTED", "%s contains a symlink" % label)


def _regular(path: Path, label: str) -> None:
    _reject_symlink_components(path, label)
    if os.path.lexists(path) is False or path.is_symlink() or not path.is_file():
        raise _error("NOT_REGULAR", "%s is not a regular file" % label)
    if path.name.endswith(".part"):
        raise _error("STAGING_FILE", "%s is a staging file" % label)


def _counts(value: Any, label: str) -> Dict[str, int]:
    if not isinstance(value, Mapping) or set(value) != set(EXPECTED_COUNTS):
        raise _error("COUNTS_INVALID", "%s must contain lidar/imu/image" % label)
    result: Dict[str, int] = {}
    for topic in EXPECTED_COUNTS:
        count = value.get(topic)
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise _error("COUNTS_INVALID", "%s.%s is invalid" % (label, topic))
        result[topic] = count
    return result


def _sha256_pin(path: Path, expected: str, label: str) -> str:
    _regular(path, label)
    observed = sha256_file(path)
    if observed != expected:
        raise _error("SOURCE_DRIFT", "%s SHA-256 differs from pin" % label)
    return observed


def _verify_sidecar(path: Path, expected_file_sha: str, expected_sidecar_sha: str, label: str) -> None:
    _regular(path, label)
    observed = sha256_file(path)
    if observed != expected_sidecar_sha:
        raise _error("RECEIPT_SIDECAR_DRIFT", "%s sidecar SHA differs from pin" % label)
    expected_line = "%s  %s\n" % (expected_file_sha, path.name[:-len(".sha256")])
    if path.read_text(encoding="ascii") != expected_line:
        raise _error("RECEIPT_SIDECAR_CONTENT", "%s sidecar content differs" % label)


def _verify_json_receipt(path: Path, expected_sha: str, sidecar_sha: str, label: str) -> Dict[str, Any]:
    observed = _sha256_pin(path, expected_sha, label)
    sidecar = path.with_name(path.name + ".sha256")
    _verify_sidecar(sidecar, observed, sidecar_sha, label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise _error("RECEIPT_INVALID", "%s is not valid JSON" % label) from exc
    if not isinstance(value, dict):
        raise _error("RECEIPT_INVALID", "%s must be an object" % label)
    return value


def verify_candidate_profile(
        profile_path: Path = PROFILE_PATH, repo_root: Path = ROOT) -> Dict[str, Any]:
    """Verify candidate YAML and every immutable lineage pin read-only."""
    observed_profile_sha = _sha256_pin(profile_path, PROFILE_SHA256, "candidate profile")
    try:
        document = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise _error("PROFILE_INVALID", "candidate profile YAML is invalid") from exc
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise _error("PROFILE_INVALID", "candidate profile schema is invalid")
    prereg = document.get("preregistration")
    if not isinstance(prereg, Mapping) or prereg.get("formal_replay_forbidden") is not True or \
            prereg.get("replay_authorized") is not False or prereg.get("replay_count") != 0:
        raise _error("PROFILE_AUTHORITY", "candidate profile is not unauthorized")
    if prereg.get("predecessor_profile") != "configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_ready.yaml" or \
            prereg.get("predecessor_profile_sha256") != READY_PROFILE_SHA256 or \
            prereg.get("predecessor_launcher") != "scripts/run_fast_livo2_m6a10_v11_formal.py":
        raise _error("PROFILE_PREDECESSOR", "candidate predecessor identity drift")
    _sha256_pin(READY_PROFILE_PATH, READY_PROFILE_SHA256, "v12 ready profile")
    predecessor_launcher = repo_root / "scripts/run_fast_livo2_m6a10_v11_formal.py"
    _sha256_pin(
        predecessor_launcher,
        prereg.get("predecessor_launcher_sha256", ""),
        "v11 predecessor launcher",
    )
    candidate = document.get("formal_candidate")
    if not isinstance(candidate, Mapping) or candidate.get("status") != "UNAUTHORIZED_NOT_RUN" or \
            candidate.get("authorization_required") is not True:
        raise _error("PROFILE_STATUS", "candidate status/authorization drift")
    phase = candidate.get("phase")
    if not isinstance(phase, Mapping) or phase.get("contract_version") != PHASE_CONTRACT or \
            phase.get("mode") != PHASE_MODE or phase.get("expected_messages") != EXPECTED_MESSAGES or \
            phase.get("expected_topic_counts") != EXPECTED_COUNTS or \
            phase.get("maximum_end_gap_seconds") != MAX_END_GAP_SECONDS or \
            phase.get("required_end_timestamp_seconds") != REQUIRED_END_TIMESTAMP_SECONDS:
        raise _error("PROFILE_PHASE", "candidate phase contract drift")
    transport = candidate.get("transport")
    if not isinstance(transport, Mapping) or transport.get("contract_version") != TRANSPORT_CONTRACT or \
            transport.get("callback_schema_version") != 3 or \
            transport.get("maximum_allowed_outstanding_messages") != 1 or \
            transport.get("mapper_internal_deque_is_diagnostic_only") is not True:
        raise _error("PROFILE_TRANSPORT", "candidate transport contract drift")
    host_interference = candidate.get("host_interference")
    if not isinstance(host_interference, Mapping) or \
            host_interference.get("contract_version") != MONITOR_CONTRACT or \
            host_interference.get("interval_seconds") != MONITOR_INTERVAL_SECONDS or \
            host_interference.get("maximum_allowed_gap_seconds") != 5.0 or \
            host_interference.get("detection_stops_process") is not False or \
            host_interference.get("performance_timing_admissible_on_detection") is not False:
        raise _error("PROFILE_INTERFERENCE", "candidate host-interference contract drift")
    image = candidate.get("image")
    if not isinstance(image, Mapping) or image.get("tag") != IMAGE_TAG or image.get("id") != IMAGE_ID or \
            image.get("base_id") != BASE_IMAGE_ID:
        raise _error("PROFILE_IMAGE", "candidate image identity drift")
    input_value = candidate.get("input")
    if not isinstance(input_value, Mapping) or input_value.get("path") != INPUT_PATH or \
            input_value.get("bytes") != INPUT_BYTES or input_value.get("sha256") != INPUT_SHA256:
        raise _error("PROFILE_INPUT", "candidate input identity drift")
    source_section = candidate.get("sources")
    if not isinstance(source_section, Mapping):
        raise _error("PROFILE_SOURCES", "candidate source pins are missing")
    observed_sources: Dict[str, Dict[str, str]] = {}
    for label, (relative, expected_sha) in SOURCE_PINS.items():
        entry = source_section.get(label)
        if not isinstance(entry, Mapping) or entry.get("path") != relative or entry.get("sha256") != expected_sha:
            raise _error("PROFILE_SOURCE_PIN", "%s profile source pin drift" % label)
        observed_sources[label] = {
            "path": str((repo_root / relative).resolve()),
            "sha256": _sha256_pin(repo_root / relative, expected_sha, label),
        }
    receipts = candidate.get("receipts")
    if not isinstance(receipts, Mapping):
        raise _error("PROFILE_RECEIPTS", "candidate receipt pins are missing")
    build = receipts.get("build")
    attempt = receipts.get("no_input_attempt6")
    host = receipts.get("host_evidence_gate")
    expected_receipt_pins = {
        "build": (str(BUILD_RECEIPT_PATH), BUILD_RECEIPT_SHA256, BUILD_RECEIPT_SIDECAR_SHA256),
        "no_input_attempt6": (str(ATTEMPT6_RECEIPT_PATH), ATTEMPT6_RECEIPT_SHA256, ATTEMPT6_SIDECAR_SHA256),
        "host_evidence_gate": (str(HOST_GATE_RECEIPT_PATH), HOST_GATE_RECEIPT_SHA256, HOST_GATE_SIDECAR_SHA256),
    }
    for name, entry in (("build", build), ("no_input_attempt6", attempt), ("host_evidence_gate", host)):
        if not isinstance(entry, Mapping) or entry.get("status") not in {"PASS", "pass"}:
            raise _error("PROFILE_RECEIPTS", "%s receipt status/pin is invalid" % name)
        if not isinstance(entry.get("path"), str) or not isinstance(entry.get("sha256"), str) or \
                not isinstance(entry.get("sidecar_sha256"), str):
            raise _error("PROFILE_RECEIPTS", "%s receipt pin is incomplete" % name)
        expected_path, expected_sha, expected_sidecar = expected_receipt_pins[name]
        if (entry.get("path"), entry.get("sha256"), entry.get("sidecar_sha256")) != \
                (expected_path, expected_sha, expected_sidecar):
            raise _error("PROFILE_RECEIPT_PIN", "%s receipt pin differs from immutable candidate" % name)
    if attempt.get("callback_sha256") != ATTEMPT6_CALLBACK_SHA256 or \
            attempt.get("terminal_sha256") != ATTEMPT6_TERMINAL_SHA256 or \
            attempt.get("service_journal_sha256") != ATTEMPT6_JOURNAL_SHA256 or \
            attempt.get("observed_runner_sha256") != ATTEMPT6_RUNNER_SHA256 or \
            host.get("pytest_command_sha256") != "8d305db19ba551f1a4f1ac884fe907bb651cb43e36e3152a3e31caa0828ba6f3":
        raise _error("PROFILE_RECEIPT_ARTIFACT", "attempt-6/host receipt artifact pin drift")
    build_receipt = _verify_json_receipt(BUILD_RECEIPT_PATH, BUILD_RECEIPT_SHA256, BUILD_RECEIPT_SIDECAR_SHA256, "build receipt")
    if build_receipt.get("status") != "PASS" or build_receipt.get("image_id") != IMAGE_ID or \
            build_receipt.get("image_tag") != IMAGE_TAG or build_receipt.get("formal_replay_forbidden") is not True or \
            build_receipt.get("ground_truth_content_opened") is not False or build_receipt.get("scorer_invoked") is not False:
        raise _error("BUILD_RECEIPT", "build receipt identity/safety drift")
    attempt_receipt = _verify_json_receipt(ATTEMPT6_RECEIPT_PATH, ATTEMPT6_RECEIPT_SHA256, ATTEMPT6_SIDECAR_SHA256, "attempt-6 receipt")
    attempt_safety = attempt_receipt.get("safety")
    if attempt_receipt.get("status") != "PASS" or attempt_receipt.get("attempt_index") != 6 or \
            not isinstance(attempt_safety, Mapping) or \
            any(attempt_safety.get(key) is not False for key in (
                "formal_replay_started", "ground_truth_content_opened", "input_opened",
                "map_saved", "scorer_invoked")):
            raise _error("ATTEMPT6_RECEIPT", "attempt-6 receipt status/authority drift")
    attempt_root = ATTEMPT6_RECEIPT_PATH.parent
    _sha256_pin(attempt_root / "out/callback.json", ATTEMPT6_CALLBACK_SHA256, "attempt-6 callback")
    _sha256_pin(attempt_root / "out/terminal.json", ATTEMPT6_TERMINAL_SHA256, "attempt-6 terminal")
    _sha256_pin(attempt_root / "service_responses.jsonl", ATTEMPT6_JOURNAL_SHA256, "attempt-6 service journal")
    observed_attempt_runner = attempt_receipt.get("sources", {}).get("runner_runtime_observed", {})
    if not isinstance(observed_attempt_runner, Mapping) or observed_attempt_runner.get("sha256") != ATTEMPT6_RUNNER_SHA256:
        raise _error("ATTEMPT6_RUNNER", "attempt-6 observed runner SHA drift")
    host_receipt = _verify_json_receipt(HOST_GATE_RECEIPT_PATH, HOST_GATE_RECEIPT_SHA256, HOST_GATE_SIDECAR_SHA256, "host evidence receipt")
    if host_receipt.get("status") != "PASS" or host_receipt.get("formal_replay_started") is not False:
        raise _error("HOST_GATE_RECEIPT", "host evidence receipt status/authority drift")
    execution = candidate.get("execution")
    if not isinstance(execution, Mapping) or execution.get("formal_replay_authorized") is not False or \
            execution.get("formal_replay_started") is not False or execution.get("replay_count") != 0 or \
            execution.get("monitor_interval_seconds") != MONITOR_INTERVAL_SECONDS or \
            execution.get("one_shell_free_popen") is not True or execution.get("retry") is not False:
        raise _error("PROFILE_EXECUTION", "candidate execution gate is not fail-closed")
    return {
        "path": str(profile_path.resolve()),
        "sha256": observed_profile_sha,
        "profile_key": PROFILE_KEY,
        "sources": observed_sources,
        "receipts": {
            "build": {"path": str(BUILD_RECEIPT_PATH), "sha256": BUILD_RECEIPT_SHA256},
            "no_input_attempt6": {"path": str(ATTEMPT6_RECEIPT_PATH), "sha256": ATTEMPT6_RECEIPT_SHA256},
            "host_evidence_gate": {"path": str(HOST_GATE_RECEIPT_PATH), "sha256": HOST_GATE_RECEIPT_SHA256},
        },
    }


def verify_formal_authorization(config: "CandidateConfig") -> Mapping[str, Any]:
    """Production authority gate: no v12 formal authorization is installed."""
    raise AuthorizationError(
        "FORMAL_REPLAY_UNAUTHORIZED",
        "v12 formal candidate has no additive authorization; input access is refused",
    )


@dataclass(frozen=True)
class CandidateConfig:
    root: Path
    repo_root: Path = ROOT
    bag_path: str = INPUT_PATH
    profile_path: Path = PROFILE_PATH
    container_name: str = "m6a10-v12-formal-candidate"
    watchdog_seconds: int = WATCHDOG_SECONDS
    authorization_path: Optional[Path] = None
    authorization_sha256: Optional[str] = None


def _canonical_mount_path(value: Path, label: str) -> str:
    if not value.is_absolute() or value.is_symlink():
        raise _error("PATH_INVALID", "%s must be an absolute non-symlink path" % label)
    return str(value)


def build_safe_docker_argv(config: CandidateConfig, output_dir: Path) -> List[str]:
    """Build the exact shell-free production argv; this function never runs it."""
    container_name = config.container_name
    if container_name == "m6a10-v12-formal-candidate":
        container_name = "m6a10-v12-formal-" + hashlib.sha256(str(config.root).encode("utf-8")).hexdigest()[:16]
    if not container_name or any(char not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-" for char in container_name):
        raise _error("CONTAINER_NAME", "container name is not a fixed safe token")
    if config.bag_path != INPUT_PATH:
        raise _error("INPUT_IDENTITY", "candidate bag path differs from pinned profile")
    output = _canonical_mount_path(output_dir.resolve(), "output")
    bag = _canonical_mount_path(Path(config.bag_path), "input")
    wrapper = _canonical_mount_path(config.repo_root / "scripts/fast_livo2_m6a10_v12_formal_container_run.sh", "wrapper")
    feeder = _canonical_mount_path(config.repo_root / "scripts/fast_livo2_m6a10_feeder.py", "feeder")
    mounts = [
        "type=bind,src=%s,dst=/input/ntu_viral.bag,readonly" % bag,
        "type=bind,src=%s,dst=/out,readonly=false" % output,
        "type=bind,src=%s,dst=%s,readonly" % (wrapper, WRAPPER_PATH_IN_CONTAINER),
        "type=bind,src=%s,dst=%s,readonly" % (feeder, FEEDER_PATH_IN_CONTAINER),
    ]
    env = [
        "M6A10_PHASE_CONTRACT_VERSION=%s" % PHASE_CONTRACT,
        "M6A10_PHASE_MODE=%s" % PHASE_MODE,
        "M6A10_PROFILE_PATH=configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_ready.yaml",
        "M6A10_PROFILE_SHA256=%s" % READY_PROFILE_SHA256,
        "M6A10_BAG_PATH=/input/ntu_viral.bag",
        "M6A10_BAG_BYTES=%d" % INPUT_BYTES,
        "M6A10_BAG_SHA256=%s" % INPUT_SHA256,
        "M6A10_FAST_EXPECTED_MESSAGES=%d" % EXPECTED_MESSAGES,
        "M6A10_FAST_EXPECTED_LIDAR_MESSAGES=%d" % EXPECTED_COUNTS["lidar"],
        "M6A10_FAST_EXPECTED_IMU_MESSAGES=%d" % EXPECTED_COUNTS["imu"],
        "M6A10_FAST_EXPECTED_IMAGE_MESSAGES=%d" % EXPECTED_COUNTS["image"],
        "M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS=%.6f" % REQUIRED_END_TIMESTAMP_SECONDS,
        "M6A10_FAST_MAX_END_GAP_SECONDS=%.2f" % MAX_END_GAP_SECONDS,
        "M6A10_FAST_MAX_BACKLOG_MESSAGES=1",
        "M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS=0.25",
        "M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS=0.05",
        "M6A10_SENSOR_DURATION_SECONDS=%.9f" % SENSOR_DURATION_SECONDS,
        "M6A10_TIMING_CONTRACT_VERSION=%s" % TIMING_CONTRACT,
        "M6A10_FAST_FEEDER_SHA256=%s" % SOURCE_PINS["v12_feeder"][1],
        "M6A10_CONSUMER_EVIDENCE=/out/callback_consumer_evidence.json",
        "M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE=/out/consumer_evidence.json",
        "M6A10_ONLINE_TIMING_EVIDENCE=/out/online_compute_timing.json",
    ]
    argv: List[str] = [
        "docker", "run", "--name", container_name,
        "--network", "none", "--read-only", "--init", "--pull=never",
        "--tmpfs", "/tmp:rw,noexec,nosuid,size=128m",
        "--tmpfs", "/root/.ros:rw,noexec,nosuid,size=32m",
        "--tmpfs", "/out:rw,noexec,nosuid,size=128m",
    ]
    for mount in mounts:
        argv.extend(["--mount", mount])
    for item in env:
        argv.extend(["--env", item])
    argv.extend(["--entrypoint", WRAPPER_PATH_IN_CONTAINER, IMAGE_ID])
    if "--rm" in argv or any(item == "rw" for item in argv):
        raise _error("DOCKER_ARGV", "unsafe cleanup or bare rw token")
    return argv


def _default_popen(argv: Sequence[str], cwd: Path) -> Any:
    return subprocess.Popen(  # noqa: S603 - argv is constructed above without a shell
        list(argv), cwd=str(cwd), shell=False, close_fds=True,
    )


def _default_monitor(root: Path) -> HostInterferenceMonitor:
    return HostInterferenceMonitor(
        root / "host_interference.samples.jsonl",
        interval_seconds=MONITOR_INTERVAL_SECONDS,
        launcher_pid=os.getpid(),
        owned_pids=(),
    )


def _atomic_create_bytes(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path):
        raise _error("OUTPUT_OVERWRITE", "refusing to overwrite %s" % path)
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + ".part")
    if os.path.lexists(part):
        raise _error("OUTPUT_STAGING", "staging file already exists: %s" % part)
    fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0), 0o600)
    try:
        with os.fdopen(fd, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(part, path, follow_symlinks=False)
    except FileExistsError as exc:
        raise _error("OUTPUT_OVERWRITE", "refusing to overwrite %s" % path) from exc
    finally:
        try:
            part.unlink()
        except FileNotFoundError:
            pass
    os.chmod(path, mode, follow_symlinks=False)
    return _sha256_bytes(payload)


def _atomic_create_json(path: Path, value: Mapping[str, Any], mode: int = 0o444) -> str:
    return _atomic_create_bytes(
        path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"), mode,
    )


def _record_counts(value: Any, label: str) -> Dict[str, int]:
    return _counts(value, label)


def validate_topic_conservation(raw: Mapping[str, Any], expected: Mapping[str, int] = EXPECTED_COUNTS) -> Dict[str, Any]:
    """Validate mapper conservation before host composition.

    The checks intentionally use backend counters and explicit record lists;
    support context is derived only from non-LiDAR buffers/records and is never
    copied from a host expected-count authority document.
    """
    if not isinstance(raw, Mapping):
        raise _error("CONSERVATION_INVALID", "raw mapper evidence must be an object")
    expected_counts = _counts(expected, "expected topic counts")
    received = _record_counts(raw.get("received_topic_counts"), "received_topic_counts")
    if received != expected_counts:
        raise _error("CONSERVATION_RECEIVED", "received counts differ from pinned expected counts")
    completed = _record_counts(raw.get("completed_counts"), "completed_counts")
    backend = raw.get("backend")
    if not isinstance(backend, Mapping):
        raise _error("CONSERVATION_BACKEND", "backend counters are missing")
    backend_completed = _record_counts(backend.get("completed_counts"), "backend.completed_counts")
    if completed != backend_completed:
        raise _error("CONSERVATION_COMPLETED", "completed counters differ from backend counters")
    buffers = raw.get("buffers")
    if not isinstance(buffers, Mapping):
        raise _error("CONSERVATION_BUFFERS", "per-topic buffers are missing")
    buffer_counts: Dict[str, int] = {}
    record_counts: Dict[str, int] = {}
    for topic in expected_counts:
        entry = buffers.get(topic)
        if not isinstance(entry, Mapping):
            raise _error("CONSERVATION_BUFFERS", "%s buffer is missing" % topic)
        count = entry.get("count")
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise _error("CONSERVATION_BUFFERS", "%s buffer count is invalid" % topic)
        records = entry.get("records")
        if not isinstance(records, list):
            raise _error("CONSERVATION_RECORDS", "%s buffer records are missing" % topic)
        buffer_counts[topic] = count
        record_counts[topic] = len(records)
        if count != len(records):
            raise _error("CONSERVATION_RECORDS", "%s buffer count differs from record count" % topic)
        if completed[topic] + count != received[topic]:
            raise _error("CONSERVATION_RECEIVED", "%s completed+buffer differs from received" % topic)
    if buffer_counts["lidar"] != 0:
        raise _error("CONSERVATION_RESIDUAL_LIDAR", "residual LiDAR buffer is forbidden")
    support = raw.get("terminal_support_context")
    if not isinstance(support, Mapping):
        raise _error("CONSERVATION_SUPPORT", "terminal support context is missing")
    support_by_topic = _record_counts(support.get("by_topic"), "support.by_topic")
    expected_support = {
        "lidar": 0,
        "imu": buffer_counts["imu"],
        "image": buffer_counts["image"],
    }
    if support_by_topic != expected_support:
        raise _error("CONSERVATION_SUPPORT", "support counts do not equal non-LiDAR buffers")
    if support.get("total_count") != sum(expected_support.values()):
        raise _error("CONSERVATION_SUPPORT", "support total differs from non-LiDAR buffers")
    support_records = support.get("records")
    if not isinstance(support_records, list):
        raise _error("CONSERVATION_SUPPORT", "support records are missing")
    support_record_counts = {topic: 0 for topic in expected_counts}
    for record in support_records:
        if not isinstance(record, Mapping) or record.get("topic") not in support_record_counts:
            raise _error("CONSERVATION_SUPPORT", "support record topic is invalid")
        if record.get("topic") == "lidar":
            raise _error("CONSERVATION_SUPPORT", "LiDAR support record is forbidden")
        support_record_counts[str(record["topic"])] += 1
    if support_record_counts != expected_support or len(support_records) != sum(expected_support.values()):
        raise _error("CONSERVATION_SUPPORT", "support records do not equal non-LiDAR buffers")
    return {
        "received": received,
        "completed": completed,
        "backend_completed": backend_completed,
        "buffer": buffer_counts,
        "record_count": record_counts,
        "support": support_by_topic,
        "support_record_count": support_record_counts,
    }


def _monitor_summary_bad(summary: Mapping[str, Any]) -> Optional[str]:
    if summary.get("contaminated") is True:
        return "HOST_INTERFERENCE_CONTAMINATED"
    if summary.get("invalid") is True:
        return "HOST_INTERFERENCE_INVALID"
    coverage = summary.get("coverage")
    if not isinstance(coverage, Mapping) or coverage.get("coverage_gap") is True:
        return "HOST_INTERFERENCE_COVERAGE_INVALID"
    if summary.get("status") != "PASS":
        return "HOST_INTERFERENCE_INVALID"
    return None


def _load_authorizer() -> Any:
    spec = importlib.util.spec_from_file_location("m6a10_v12_authorizer", AUTHORIZER_PATH)
    if spec is None or spec.loader is None:
        raise AuthorizationError("AUTHORIZER_LOAD", "authorization module cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _default_authorized(config: CandidateConfig) -> Mapping[str, Any]:
    if config.authorization_path is None or not config.authorization_sha256:
        return verify_formal_authorization(config)
    if AUTHORIZER_SHA256:
        _sha256_pin(AUTHORIZER_PATH, AUTHORIZER_SHA256, "authorization module")
    authorizer = _load_authorizer()
    receipt = authorizer.verify_authorization(
        config.authorization_path, config.root, config.authorization_sha256,
        repo_root=config.repo_root,
    )
    return {"authorized": True, "formal_execution": True, "receipt": receipt}


def _production_identity_probe(config: CandidateConfig) -> Mapping[str, Any]:
    """Read-only image inspect after authorization and quiescence only."""
    completed = subprocess.run(
        ["docker", "image", "inspect", IMAGE_ID],
        cwd=str(config.repo_root), shell=False, capture_output=True, check=False,
    )
    if completed.returncode != 0:
        raise CandidateError("IMAGE_IDENTITY_FAIL_CLOSED", "docker image inspect failed")
    try:
        documents = json.loads(completed.stdout.decode("utf-8"))
        document = documents[0]
        labels = document.get("Config", {}).get("Labels", {}) or {}
        repo_tags = document.get("RepoTags", []) or []
    except (UnicodeError, json.JSONDecodeError, IndexError, TypeError, AttributeError) as exc:
        raise CandidateError("IMAGE_IDENTITY_INVALID", "docker image inspect was malformed") from exc
    image_id = document.get("Id")
    if image_id != IMAGE_ID or IMAGE_TAG not in repo_tags or \
            labels.get("benchmark.fast_livo2.m6a10_variant") != "v12" or \
            labels.get("benchmark.fast_livo2.m6a10_terminal_contract") != PHASE_CONTRACT or \
            labels.get("benchmark.fast_livo2.m6a10_transport_contract") != TRANSPORT_CONTRACT or \
            labels.get("benchmark.fast_livo2.m6a10_formal_replay_forbidden") != "true" or \
            labels.get("benchmark.fast_livo2.m6a10_ground_truth_present") != "false" or \
            labels.get("benchmark.fast_livo2.m6a10_scorer_present") != "false":
        raise CandidateError("IMAGE_IDENTITY_DRIFT", "image ID/tag/labels do not match v12 pin")
    return {"image_id": image_id, "tag": IMAGE_TAG, "labels": labels, "opened": False}


def _production_bag_probe(config: CandidateConfig) -> Mapping[str, Any]:
    """Stat and stream the pinned bag only after auth/quiescence gates."""
    path = Path(config.bag_path)
    _reject_symlink_components(path, "input bag")
    if path.is_symlink() or not path.is_file():
        raise CandidateError("INPUT_IDENTITY_FAIL_CLOSED", "pinned input bag is not regular")
    try:
        size = path.stat().st_size
        digest = sha256_file(path)
    except OSError as exc:
        raise CandidateError("INPUT_PROBE_FAIL_CLOSED", "pinned input bag cannot be read") from exc
    if size != INPUT_BYTES or digest != INPUT_SHA256:
        raise CandidateError("INPUT_IDENTITY_DRIFT", "input bag bytes/SHA differ from authorization")
    return {"path": str(path), "bytes": size, "sha256": digest, "opened": True}


def _continuous_quiescence(config: CandidateConfig) -> Mapping[str, Any]:
    """Take three contiguous launcher-owned read-only pre-start windows."""
    windows: List[Dict[str, Any]] = []
    excluded = _quiescence.ancestor_pids(Path("/proc"), pid=os.getpid())
    authorizer = _load_authorizer()
    for index in range(1, 4):
        try:
            observation = _quiescence.collect_observation(
                proc_root=Path("/proc"), sample_seconds=5.0,
                max_busy_percent=5.0, max_load_per_cpu=0.50,
                excluded_pids=excluded,
            )
            extra_forbidden = authorizer._extra_forbidden_processes(Path("/proc"), excluded)
            if extra_forbidden:
                observation["forbidden_processes"] = list(observation.get("forbidden_processes", [])) + extra_forbidden
                observation["checks"]["no_forbidden_processes"] = False
            value = _quiescence.build_receipt(observation)
        except Exception as exc:
            value = {
                "schema_version": 1,
                "contract_version": "m6a10-quiescence-v1",
                "status": "FAIL_CLOSED",
                "runner_start_allowed": False,
                "error": "%s: %s" % (type(exc).__name__, exc),
            }
        value["launcher_owned_pid"] = os.getpid()
        value["window_index"] = index
        path = config.root / ("launcher_quiescence_window_%02d.receipt.json" % index)
        digest = _atomic_create_json(path, value)
        windows.append({
            "path": str(path), "sha256": digest,
            "status": value.get("status"),
            "runner_start_allowed": value.get("runner_start_allowed"),
            "forbidden_processes": value.get("observation", {}).get("forbidden_processes", []),
        })
    passed = len(windows) == 3 and all(
        item.get("status") == "PASS" and item.get("runner_start_allowed") is True and
        not item.get("forbidden_processes") for item in windows
    )
    return {
        "status": "PASS" if passed else "FAIL_CLOSED",
        "window_count": len(windows),
        "consecutive_passes": 3 if passed else 0,
        "windows": windows,
        "launcher_pid": os.getpid(),
    }


def _default_identity_probe(config: CandidateConfig) -> Mapping[str, Any]:
    raise CandidateError("IDENTITY_PROBE_NOT_INSTALLED", "production identity probe is not enabled for candidate")


def _default_bag_probe(config: CandidateConfig) -> Mapping[str, Any]:
    raise CandidateError("INPUT_PROBE_NOT_INSTALLED", "production input probe is not enabled for candidate")


def _default_capture(config: CandidateConfig, root: Path) -> Mapping[str, Any]:
    raise CandidateError("RAW_CAPTURE_NOT_INSTALLED", "raw capture requires a reviewed runtime adapter")


def _default_compose(raw: Mapping[str, Any], config: CandidateConfig) -> Mapping[str, Any]:
    raise CandidateError("COMPOSITOR_NOT_INSTALLED", "composition requires a reviewed host adapter")


def _load_host_module(relative: str, module_name: str) -> Any:
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise CandidateError("HOST_ADAPTER_LOAD", "host adapter cannot be loaded: %s" % relative)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _production_capture(config: CandidateConfig, root: Path) -> Mapping[str, Any]:
    """Load only regular raw outputs after the one container naturally exits."""
    paths = {
        "feeder": root / "out/feeder_receipt.json",
        "callback": root / "out/callback_consumer_evidence.json",
        "terminal": root / "out/consumer_evidence.json",
        "timing": root / "out/online_compute_timing.json",
    }
    values: Dict[str, Any] = {}
    bindings: Dict[str, Dict[str, Any]] = {}
    for label, path in paths.items():
        _regular(path, label)
        raw = path.read_bytes()
        try:
            values[label] = json.loads(raw.decode("utf-8"))
        except (UnicodeError, json.JSONDecodeError) as exc:
            raise CandidateError("RAW_EVIDENCE_INVALID", "%s raw evidence is invalid" % label) from exc
        bindings[label] = {"path": str(path), "sha256": _sha256_bytes(raw)}
    return {"terminal": values["terminal"], "documents": values, "bindings": bindings}


def _production_compose(raw: Mapping[str, Any], config: CandidateConfig) -> Mapping[str, Any]:
    root = config.root
    documents = raw.get("documents") if isinstance(raw, Mapping) else None
    if not isinstance(documents, Mapping):
        raise CandidateError("RAW_EVIDENCE_INVALID", "raw document collection is missing")
    feeder_path = root / "out/feeder_receipt.json"
    callback_path = root / "out/callback_consumer_evidence.json"
    terminal_path = root / "out/consumer_evidence.json"
    timing_path = root / "out/online_compute_timing.json"
    bound_path = root / "host_bound_consumer_evidence.json"
    composed_path = root / "host_composed_terminal_evidence.json"
    binder = _load_host_module("scripts/bind_fast_livo2_v12_consumer_evidence.py", "m6a10_v12_binder_runtime")
    compositor = _load_host_module("scripts/compose_fast_livo2_v12_terminal_evidence.py", "m6a10_v12_compositor_runtime")
    try:
        bound = binder.bind_consumer_evidence(
            feeder_path=feeder_path, callback_path=callback_path,
            terminal_path=terminal_path, timing_path=timing_path,
            output_path=bound_path, profile_path=READY_PROFILE_PATH,
            expected_profile_sha256=READY_PROFILE_SHA256,
            input_path=INPUT_PATH, input_bytes=INPUT_BYTES, input_sha256=INPUT_SHA256,
        )
        composed = compositor.compose(
            profile_path=READY_PROFILE_PATH, bound_path=bound_path,
            output_path=composed_path, expected_profile_sha256=READY_PROFILE_SHA256,
        )
    except Exception as exc:
        raise CandidateError("HOST_COMPOSITION_FAIL_CLOSED", str(exc)) from exc
    return {
        "status": composed.get("status"),
        "bound_path": str(bound_path),
        "bound_sha256": bound.get("output_sha256"),
        "composed_path": str(composed_path),
        "composed_sha256": sha256_file(composed_path),
        "contract_id": composed.get("contract_id"),
    }


def _reserve_root(root: Path) -> None:
    if os.path.lexists(root):
        raise _error("ROOT_NOT_FRESH", "formal candidate root already exists")
    root.parent.mkdir(parents=True, exist_ok=True)
    try:
        root.mkdir()
    except FileExistsError as exc:
        raise _error("ROOT_NOT_FRESH", "formal candidate root already exists") from exc


def _seal_preflight_failure(
        config: CandidateConfig, *, failure_kind: str, failure_message: str,
        authorization: Mapping[str, Any], profile: Optional[Mapping[str, Any]] = None,
        quiescence: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
    receipt: Dict[str, Any] = {
        "schema_version": 1,
        "contract_version": "m6a10-v12-formal-candidate-closure-v1",
        "status": "FAIL_CLOSED",
        "created_at_utc": _datetime.datetime.now(_datetime.timezone.utc).isoformat(),
        "profile": profile,
        "authorization": {
            "formal_replay_authorized": authorization.get("formal_execution") is True,
            "exact_root": str(config.root),
        },
        "execution": {
            "formal_replay_started": False,
            "candidate_process_started": False,
            "one_start": False,
            "popen_count": 0,
            "retry": False,
            "manual_stop": False,
            "returncode": None,
            "shell": False,
        },
        "monitor": None,
        "preflight_quiescence": dict(quiescence) if isinstance(quiescence, Mapping) else None,
        "conservation": None,
        "composition": None,
        "failure_kind": failure_kind,
        "failure_message": failure_message,
        "safety": {
            "formal_replay_started": False,
            "input_opened": False,
            "ground_truth_content_opened": False,
            "scorer_invoked": False,
            "map_saved": False,
        },
    }
    receipt_path = config.root / "closure_receipt.json"
    receipt_sha = _atomic_create_json(receipt_path, receipt)
    sidecar = receipt_path.with_name(receipt_path.name + ".sha256")
    sidecar_sha = _atomic_create_bytes(sidecar, ("%s  %s\n" % (receipt_sha, receipt_path.name)).encode("ascii"))
    receipt.update({
        "receipt_path": str(receipt_path), "receipt_sha256": receipt_sha,
        "sidecar_path": str(sidecar), "sidecar_sha256": sidecar_sha,
    })
    return receipt


ProcessFactory = Callable[[Sequence[str], Path], Any]
MonitorFactory = Callable[[Path], Any]
Probe = Callable[[CandidateConfig], Mapping[str, Any]]
Capture = Callable[[CandidateConfig, Path], Mapping[str, Any]]
Composer = Callable[[Mapping[str, Any], CandidateConfig], Mapping[str, Any]]


def run_formal(
        config: CandidateConfig,
        *,
        authorization_validator: Optional[Callable[[CandidateConfig], Mapping[str, Any]]] = None,
        identity_probe: Optional[Probe] = None,
        bag_probe: Optional[Probe] = None,
        process_factory: Optional[ProcessFactory] = None,
        monitor_factory: Optional[MonitorFactory] = None,
        raw_capture: Optional[Capture] = None,
        composer: Optional[Composer] = None,
        now: Optional[str] = None,
) -> Dict[str, Any]:
    """Run one exact-root lifecycle; the default path remains unauthorized."""
    if authorization_validator is None:
        authorization = _default_authorized(config)
    else:
        authorization = authorization_validator(config)
    if not isinstance(authorization, Mapping) or authorization.get("authorized") is not True:
        raise AuthorizationError("FORMAL_REPLAY_UNAUTHORIZED", "injected authority did not authorize this candidate")

    _reserve_root(config.root)
    profile: Optional[Mapping[str, Any]] = None
    preflight_quiescence: Optional[Mapping[str, Any]] = None
    try:
        profile = verify_candidate_profile(config.profile_path, config.repo_root)
        if authorization.get("formal_execution") is True:
            preflight_quiescence = _continuous_quiescence(config)
            if preflight_quiescence.get("status") != "PASS":
                return _seal_preflight_failure(
                    config, failure_kind="QUIESCENCE_FAIL_CLOSED",
                    failure_message="launcher-owned three-window quiescence did not PASS",
                    authorization=authorization, profile=profile,
                    quiescence=preflight_quiescence,
                )
        identity_fn = identity_probe or (
            _production_identity_probe if authorization.get("formal_execution") is True else _default_identity_probe)
        bag_fn = bag_probe or (
            _production_bag_probe if authorization.get("formal_execution") is True else _default_bag_probe)
        identity = identity_fn(config)
        bag_identity = bag_fn(config)
        if not isinstance(identity, Mapping) or not isinstance(bag_identity, Mapping):
            raise _error("IDENTITY_INVALID", "identity probe result is not an object")
    except CandidateError as exc:
        return _seal_preflight_failure(
            config, failure_kind=exc.kind, failure_message=str(exc),
            authorization=authorization, profile=profile,
            quiescence=preflight_quiescence,
        )
    output_dir = config.root / "out"
    output_dir.mkdir()
    argv = build_safe_docker_argv(config, output_dir)
    monitor = (monitor_factory or _default_monitor)(config.root)
    runner: Any = None
    returncode: Optional[int] = None
    started = False
    monitor_summary: Dict[str, Any] = {}
    failure_kind: Optional[str] = None
    failure_message: Optional[str] = None
    conservation: Optional[Dict[str, Any]] = None
    composed: Optional[Mapping[str, Any]] = None
    raw: Optional[Mapping[str, Any]] = None
    capture_fn = raw_capture or (
        _production_capture if authorization.get("formal_execution") is True else None)
    compose_fn = composer or (
        _production_compose if authorization.get("formal_execution") is True else None)

    try:
        # This is the start boundary: no shell, no retry, and no work between
        # monitor.start() and the single Popen/process-factory invocation.
        monitor.start()
        runner = (process_factory or _default_popen)(argv, config.repo_root)
        started = True
        pid = getattr(runner, "pid", None)
        if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 0:
            raise CandidateError("RUNNER_PID_INVALID", "single runner did not expose a positive PID")
        monitor.allow_owned_pid(pid)
        returncode = int(runner.wait())
    except CandidateError as exc:
        failure_kind = exc.kind
        failure_message = str(exc)
    except InterferenceError as exc:
        failure_kind = "HOST_INTERFERENCE_INVALID"
        failure_message = str(exc)
    except Exception as exc:  # preserve closure; never retry or control the runner
        failure_kind = "PROCESS_LIFECYCLE_FAILURE"
        failure_message = "%s: %s" % (type(exc).__name__, exc)
    finally:
        if getattr(monitor, "_started", True):
            try:
                monitor.stop()
                monitor_summary = dict(monitor.finalize(config.root / "host_interference.summary.json"))
            except Exception as exc:
                failure_kind = failure_kind or "HOST_INTERFERENCE_COVERAGE_INVALID"
                failure_message = failure_message or "%s: %s" % (type(exc).__name__, exc)

    # Raw observations are captured after natural process completion and
    # monitor finalization.  They are validated before any composition call.
    if started and capture_fn is not None:
        try:
            raw = capture_fn(config, config.root)
            terminal_raw = raw.get("terminal", raw) if isinstance(raw, Mapping) else raw
            conservation = validate_topic_conservation(terminal_raw)
        except CandidateError as exc:
            failure_kind = failure_kind or exc.kind
            failure_message = failure_message or str(exc)
        except Exception as exc:
            failure_kind = failure_kind or "CONSERVATION_INVALID"
            failure_message = failure_message or "%s: %s" % (type(exc).__name__, exc)
    elif started and capture_fn is None:
        failure_kind = failure_kind or "RAW_CAPTURE_NOT_INSTALLED"
        failure_message = failure_message or "raw capture adapter is required"

    monitor_failure = _monitor_summary_bad(monitor_summary) if monitor_summary else "HOST_INTERFERENCE_COVERAGE_INVALID"
    if monitor_failure:
        failure_kind = failure_kind or monitor_failure
        failure_message = failure_message or "host monitor did not produce admissible performance evidence"
    if returncode not in (None, 0):
        failure_kind = failure_kind or "PROCESS_FAILURE"
        failure_message = failure_message or "runner exited with code %d" % returncode
    if compose_fn is not None and raw is not None and conservation is not None:
        try:
            composed = compose_fn(raw, config)
            if not isinstance(composed, Mapping) or composed.get("status") != "PASS":
                failure_kind = failure_kind or "COMPOSITION_FAIL_CLOSED"
                failure_message = failure_message or "composition did not return PASS"
        except CandidateError as exc:
            failure_kind = failure_kind or exc.kind
            failure_message = failure_message or str(exc)
        except Exception as exc:
            failure_kind = failure_kind or "COMPOSITION_FAIL_CLOSED"
            failure_message = failure_message or "%s: %s" % (type(exc).__name__, exc)
    elif failure_kind is None:
        failure_kind = "COMPOSITOR_NOT_INSTALLED"
        failure_message = "composition adapter is required"

    status = "PASS" if failure_kind is None else "FAIL_CLOSED"
    formal_started = authorization.get("formal_execution") is True and started
    input_opened = authorization.get("formal_execution") is True and bag_identity.get("opened") is True
    receipt: Dict[str, Any] = {
        "schema_version": 1,
        "contract_version": "m6a10-v12-formal-candidate-closure-v1",
        "status": status,
        "created_at_utc": now or _datetime.datetime.now(_datetime.timezone.utc).isoformat(),
        "profile": profile,
        "authorization": {
            "formal_replay_authorized": authorization.get("formal_execution") is True,
            "receipt_path": str(config.authorization_path) if config.authorization_path else None,
            "receipt_sha256": config.authorization_sha256,
            "exact_root": str(config.root),
        },
        "identity": dict(identity),
        "bag_identity": dict(bag_identity),
        "execution": {
            "formal_replay_started": formal_started,
            "candidate_process_started": started,
            "one_start": started,
            "popen_count": 1 if started else 0,
            "retry": False,
            "manual_stop": False,
            "returncode": returncode,
            "argv": list(argv),
            "shell": False,
        },
        "monitor": monitor_summary,
        "preflight_quiescence": dict(preflight_quiescence) if isinstance(preflight_quiescence, Mapping) else None,
        "conservation": conservation,
        "raw_evidence": dict(raw.get("bindings", {})) if isinstance(raw, Mapping) else None,
        "composition": dict(composed) if isinstance(composed, Mapping) else None,
        "failure_kind": failure_kind,
        "failure_message": failure_message,
        "safety": {
            "formal_replay_started": formal_started,
            "input_opened": input_opened,
            "ground_truth_content_opened": False,
            "scorer_invoked": False,
            "map_saved": False,
        },
    }
    receipt_path = config.root / "closure_receipt.json"
    receipt_sha = _atomic_create_json(receipt_path, receipt)
    sidecar = receipt_path.with_name(receipt_path.name + ".sha256")
    sidecar_sha = _atomic_create_bytes(
        sidecar, ("%s  %s\n" % (receipt_sha, receipt_path.name)).encode("ascii"), 0o444,
    )
    receipt["receipt_path"] = str(receipt_path)
    receipt["receipt_sha256"] = receipt_sha
    receipt["sidecar_path"] = str(sidecar)
    receipt["sidecar_sha256"] = sidecar_sha
    return receipt


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--bag", default=INPUT_PATH)
    parser.add_argument("--authorization", type=Path)
    parser.add_argument("--authorization-sha256")
    parser.add_argument("--container-name", default="m6a10-v12-formal-candidate")
    args = parser.parse_args(argv)
    try:
        result = run_formal(CandidateConfig(
            root=args.root, repo_root=args.repo_root, bag_path=args.bag,
            authorization_path=args.authorization,
            authorization_sha256=args.authorization_sha256,
            container_name=args.container_name,
        ))
    except CandidateError as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "failure_kind": exc.kind, "failure_message": str(exc)}, sort_keys=True))
        return 11
    print(json.dumps({key: result.get(key) for key in ("status", "failure_kind", "receipt_path", "receipt_sha256")}, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
