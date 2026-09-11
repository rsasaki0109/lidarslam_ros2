#!/usr/bin/env python3
"""Host lifecycle gate for the v12 no-input dual-service handshake.

The existing v12 container payload is the only process-startup payload.  This
runner starts one detached, isolated container, feeds that payload through one
``docker exec -i ... bash -s``, and then drives the fixed service protocol with
argv-only commands.  Docker/ROS effects are injectable for unit tests.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from typing import Any, Callable, Mapping, MutableMapping, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
IMAGE_TAG = (
    "m6a10-v2c-v12-nonlidar-boundary-transport-20260824-"
    "fast-livo2-benchmark:ros1-pinned"
)
IMAGE_ID = "sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7"
PATCH_PATH = Path(
    "docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch"
)
PATCH_SHA256 = "39c77535a7557365dac6b0f2c99849038b29670a57c3101be3a39138317c6333"
WRAPPER_PATH = Path("scripts/fast_livo2_m6a10_v12_formal_container_run.sh")
WRAPPER_SHA256 = "af6fa54f834ae209758ee5a9903695674f4095c1063ac57b46c2774bfbc5c7a9"
PAYLOAD_PATH = Path(
    "scripts/fast_livo2_m6a10_v12_no_input_container_payload.sh"
)
PAYLOAD_SHA256 = "2c1c9b86fadc0d1d83e7874542f92ff33b99b23421397374ecd9e880ad1b73aa"
PROFILE_PATH = Path(
    "configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_ready.yaml"
)
PROFILE_SHA256 = "675996a7a5fd81d59d752de4bbea1d4373487a17d03ee085aa08ae9493a0b28d"
FOCUSED_TEST_PATH = Path(
    "graph_based_slam/test/test_fast_livo2_m6a10_v12_no_input_gate.py"
)
BUILD_RECEIPT_PATH = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v12_build_20260823T161850Z_agentv12/"
    "build_identity.receipt.json"
)
BUILD_RECEIPT_SHA256 = "e0e5c924025a24083661a6838bc5af28f07c34c419ce6e103c37890ed1382dda"

PHASE_CONTRACT = "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
TRANSPORT_CONTRACT = "m6a10-v12-callback-ack-transport-outstanding-v1"
TERMINAL_CONTRACT = "m6a10-fast-livo2-consumer-terminal-v1"
PHASE_MODE = "unpaced_ack"
CONTAINER_CONTRACT = "m6a10-v12-no-input-dual-service-gate-v1"
ROS_ENTRYPOINT = "/ros_entrypoint.sh"
READY_MARKER = "/out/services_ready.txt"
SERVICES_LIST = "/out/services.list"
CALLBACK_RAW = "/out/callback.json"
TERMINAL_RAW = "/out/terminal.json"
PINNED_DIAGNOSTIC_PATHS = (
    "/out/services_ready.txt",
    "/out/services.list",
    "/out/roscore.log",
    "/out/mapper.log",
)
PINNED_RAW_PATHS = (CALLBACK_RAW, TERMINAL_RAW)
ARTIFACT_MANIFEST_NAME = "artifact_manifest.json"
SERVICE_JOURNAL_NAME = "service_responses.jsonl"
EXPECTED_COUNTS = {"lidar": 1, "imu": 0, "image": 0}
EXPECTED_SERVICES = [
    "/m6a10/consumer_status", "/m6a10/consumer_ack",
    "/m6a10/consumer_eof", "/m6a10/consumer_finalize",
    "/m6a10/terminal_status", "/m6a10/terminal_eof",
    "/m6a10/terminal_finalize",
]
MAX_RUNTIME_SECONDS = 300.0
STOP_GRACE_SECONDS = 20.0
TERMINAL_POLL_GAP_SECONDS = 0.05

PRIOR_ATTEMPTS = [
    {
        "attempt_index": 1,
        "path": Path(
            "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
            "fast_livo2_v2c_v12_dual_service_20260823T163503Z_agentv12/"
            "no_input_dual_evidence.receipt.json"
        ),
        "sha256": "b5e70574156353d476697a9315c4285d81f7c84a24f0e5fa0ce212af1dec18ce",
        "cause": "missing_docker_interactive_stdin",
        "failure_kind": "NO_INPUT_GATE_NOT_READY",
        "exit_code": 0,
        "cause_evidence": {
            "argv_path": Path(
                "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
                "fast_livo2_v2c_v12_dual_service_20260823T163503Z_agentv12/"
                "docker_argv.json"
            ),
            "argv_sidecar_path": Path(
                "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
                "fast_livo2_v2c_v12_dual_service_20260823T163503Z_agentv12/"
                "docker_argv.sha256"
            ),
            "argv_sha256": "79c09830e4d896e25996b1cb5752e66d9197846690ba2e10fec18da38f17d694",
            "argv_canonical_sha256": "03af075918f4f6ca94e7f6cea41799dac7a8c1e46e39f25ae56feed90b21fc8b",
            "argv_sidecar_sha256": "033dac815af2b26da9035eab849c23e895d9cfe982e064e9c5162805d7c3e62a",
        },
    },
    {
        "attempt_index": 2,
        "path": Path(
            "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
            "fast_livo2_v2c_v12_dual_service_attempt2_20260823T163846Z_agentv12/"
            "no_input_dual_evidence_attempt2.receipt.json"
        ),
        "sha256": "c85f4252aeb19f05f9635e0fd3fefbf7e070c8bb2e1df2ce3db38d16afccf7fb",
        "cause": "mapper_pid_assignment_typo",
        "failure_kind": "NO_INPUT_GATE_NOT_READY",
        "exit_code": 143,
        "cause_evidence": {
            "argv_path": Path(
                "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
                "fast_livo2_v2c_v12_dual_service_attempt2_20260823T163846Z_agentv12/"
                "docker_exec_argv.json"
            ),
            "argv_sidecar_path": Path(
                "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
                "fast_livo2_v2c_v12_dual_service_attempt2_20260823T163846Z_agentv12/"
                "docker_exec_argv.sha256"
            ),
            "argv_sha256": "2f0cec1b55abe4f38b39505f64a77b18b8bd85943e24ebf835b7fe1fe4a53e6a",
            "argv_canonical_sha256": "46ddde7b79679087a5180fcfe5ea512e9a18e40bf67627bddecf3d3a31b748ff",
            "argv_sidecar_sha256": "c5caadec81eb66fc1540070dcafdbe905c8b646af5695a5ae931d7f7c25ba2c9",
            "log_path": Path(
                "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
                "fast_livo2_v2c_v12_dual_service_attempt2_20260823T163846Z_agentv12/"
                "docker_exec.log"
            ),
            "log_sha256": "139228f498382d8b37e8438638ccaa1fd114b86d76de471a4b62f5df7645d71c",
        },
    },
    {
        "attempt_index": 3,
        "path": Path(
            "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
            "fast_livo2_v2c_v12_dual_service_attempt3_20260823T172100Z_agentv12/"
            "no_input_dual_evidence.receipt.json"
        ),
        "sha256": "7579b046bab044ce6c45b9cdc851f2406a7ec850910d12e49d9a3eb67e2943a9",
        "receipt_sidecar_sha256": "6f192d3872cf6aa733bdf79a38f470ac2399f1159dd49532f937e0980e85b659",
        "cause": "synthetic_publisher_syntax_error",
        "failure_kind": "SYNTHETIC_PUBLISH_FAILURE",
        "exit_code": 143,
        "cause_evidence": {
            "failure_message_fragments": (
                'File "<string>", line 1',
                "while p.get_num_connections()<1 and time.monotonic()<d: time.sleep(.1);",
                "SyntaxError: invalid syntax",
            ),
            "services": tuple(EXPECTED_SERVICES),
            "start_count": 1,
            "publisher_failed_before_callback": True,
        },
    },
    {
        "attempt_index": 4,
        "path": Path(
            "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
            "fast_livo2_v2c_v12_dual_service_attempt4_20260823T173000Z_agentv12/"
            "no_input_dual_evidence.receipt.json"
        ),
        "sha256": "6f8ca7d34884420a805726100cd01f3d004d7fb63f50a28267bedaa1d04a0490",
        "receipt_sidecar_sha256": "2ebad3e9c01399f093352c9e7fdcbbe5ef187a68bab218f0df6aa8e0e824c215",
        "receipt_attempt_index": 3,
        "cause": "ros_entrypoint_not_used_for_ros_python",
        "failure_kind": "SYNTHETIC_PUBLISH_FAILURE",
        "exit_code": 143,
        "cause_evidence": {
            "failure_message_fragments": (
                'File "<string>", line 4',
                "ModuleNotFoundError: No module named 'rospy'",
            ),
            "services": tuple(EXPECTED_SERVICES),
            "start_count": 1,
            "publisher_failed_before_callback": True,
        },
    },
    {
        "attempt_index": 5,
        "path": Path(
            "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
            "fast_livo2_v2c_v12_dual_service_attempt5_20260823T174000Z_agentv12/"
            "no_input_dual_evidence.receipt.json"
        ),
        "sha256": "a6109457f554bc329f630987274266f68c65bf4c92dbc29737ee44ce0bedf687",
        "receipt_sidecar_sha256": "dba4a5e3e913e9f7dab99b575e3e8e80626c3ba99846482a8f6c16ab86595d1c",
        "cause": "artifact_copy_callback_raw_failure",
        "failure_kind": "ARTIFACT_COPY_FAILURE",
        "exit_code": 143,
        "cause_evidence": {
            "failure_message_fragments": ("/out/callback.json",),
            "services": tuple(EXPECTED_SERVICES),
            "start_count": 1,
            "artifact_path": CALLBACK_RAW,
        },
    },
]


class GateError(RuntimeError):
    """Fail-closed host gate error."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def sha256_file(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise GateError("SOURCE_MISSING", f"not a regular file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def reserve_output_root(root: Path) -> Path:
    root = Path(root)
    if root.exists() or root.is_symlink():
        raise GateError("ROOT_REUSE", f"fresh output root required: {root}")
    if not root.parent.is_dir() or root.parent.is_symlink():
        raise GateError("ROOT_PARENT_INVALID", f"invalid output parent: {root.parent}")
    try:
        root.mkdir(mode=0o755)
    except FileExistsError as error:
        raise GateError("ROOT_REUSE", f"output root appeared during reservation: {root}") from error
    marker = root / ".reserved-v12-no-input-host-gate"
    fd = os.open(marker, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o444)
    with os.fdopen(fd, "w", encoding="ascii") as stream:
        stream.write(CONTAINER_CONTRACT + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    marker.chmod(0o444)
    return root


def _container_name(root: Path) -> str:
    suffix = hashlib.sha256(str(root.resolve(strict=False)).encode()).hexdigest()[:20]
    name = f"m6a10-v12-no-input-{suffix}"
    if re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,127}", name) is None:
        raise GateError("CONTAINER_NAME_INVALID", name)
    return name


def verify_build_receipt(path: Path = BUILD_RECEIPT_PATH) -> dict[str, Any]:
    observed = sha256_file(path)
    if observed != BUILD_RECEIPT_SHA256:
        raise GateError("BUILD_RECEIPT_SHA_MISMATCH", str(path))
    sidecar = path.with_suffix(path.suffix + ".sha256")
    if sidecar.is_symlink() or not sidecar.is_file() or \
            not sidecar.read_text(encoding="ascii").startswith(f"{observed}  "):
        raise GateError("BUILD_RECEIPT_SIDECAR_INVALID", str(sidecar))
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise GateError("BUILD_RECEIPT_INVALID", str(path)) from error
    expected = {
        "schema_version": 1, "kind": "fast_livo2_m6a10_v12_build_candidate",
        "status": "PASS", "image_tag": IMAGE_TAG, "image_id": IMAGE_ID,
        "patch_sha256": PATCH_SHA256, "wrapper_sha256": WRAPPER_SHA256,
        "network": "none", "formal_replay_forbidden": True,
        "input_opened": False, "ground_truth_content_opened": False,
        "scorer_invoked": False,
    }
    if not isinstance(value, dict) or any(value.get(k) != v for k, v in expected.items()):
        raise GateError("BUILD_RECEIPT_BINDING_INVALID", "build identity/safety drift")
    return {"path": str(path), "sha256": observed, "status": value["status"]}


def verify_prior_attempts(prior: Sequence[Mapping[str, Any]] = PRIOR_ATTEMPTS) -> list[dict[str, Any]]:
    indices = [item.get("attempt_index") for item in prior]
    if indices != list(range(1, len(indices) + 1)):
        raise GateError("PRIOR_ATTEMPT_SEQUENCE_INVALID", f"expected contiguous prior indices, got {indices}")
    checked = []
    for item in prior:
        path = Path(item["path"])
        expected_sha = item.get("sha256")
        cause = item.get("cause")
        if not isinstance(expected_sha, str) or len(expected_sha) != 64 or not isinstance(cause, str) or not cause:
            raise GateError("PRIOR_RECEIPT_BINDING_INVALID", str(path))
        if sha256_file(path) != expected_sha:
            raise GateError("PRIOR_RECEIPT_SHA_MISMATCH", str(path))
        sidecar = path.with_suffix(path.suffix + ".sha256")
        if sidecar.is_symlink() or not sidecar.is_file() or \
                not sidecar.read_text(encoding="ascii").startswith(f"{expected_sha}  "):
            raise GateError("PRIOR_RECEIPT_SIDECAR_INVALID", str(sidecar))
        try:
            receipt = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise GateError("PRIOR_RECEIPT_INVALID", str(path)) from error
        if not isinstance(receipt, dict) or receipt.get("status") != "FAIL_CLOSED" or \
                receipt.get("failure_kind") != item.get("failure_kind"):
            raise GateError("PRIOR_RECEIPT_BINDING_INVALID", str(path))
        evidence = item.get("cause_evidence")
        if not isinstance(evidence, Mapping):
            raise GateError("PRIOR_CAUSE_EVIDENCE_MISSING", str(path))
        attempt_index = item.get("attempt_index")
        if attempt_index in (1, 2):
            argv_path = Path(evidence["argv_path"])
            argv_sidecar_path = Path(evidence["argv_sidecar_path"])
            argv_sha = evidence.get("argv_sha256")
            canonical_sha = evidence.get("argv_canonical_sha256")
            sidecar_sha = evidence.get("argv_sidecar_sha256")
            if not all(isinstance(value, str) and len(value) == 64 for value in (argv_sha, canonical_sha, sidecar_sha)):
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", str(argv_path))
            if sha256_file(argv_path) != argv_sha or sha256_file(argv_sidecar_path) != sidecar_sha:
                raise GateError("PRIOR_CAUSE_EVIDENCE_SHA_MISMATCH", str(argv_path))
            try:
                argv_value = json.loads(argv_path.read_text(encoding="utf-8"))
                canonical_bytes = json.dumps(argv_value, separators=(",", ":")).encode("utf-8")
            except (OSError, UnicodeError, json.JSONDecodeError, TypeError) as error:
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", str(argv_path)) from error
            if hashlib.sha256(canonical_bytes).hexdigest() != canonical_sha or \
                    argv_sidecar_path.read_text(encoding="ascii") != f"{canonical_sha}\n":
                raise GateError("PRIOR_CAUSE_EVIDENCE_SIDECAR_INVALID", str(argv_sidecar_path))
            if attempt_index == 1:
                if not isinstance(argv_value, list) or argv_value[:2] != ["docker", "run"] or \
                        "-i" in argv_value or argv_value[-2:] != ["bash", "-s"]:
                    raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt1 docker argv does not prove missing -i")
                verified_cause_evidence = {
                    "kind": "docker_run_argv_missing_interactive_stdin",
                    "argv_path": str(argv_path), "argv_sha256": argv_sha,
                    "argv_sidecar_path": str(argv_sidecar_path),
                    "argv_sidecar_sha256": sidecar_sha,
                    "argv_canonical_sha256": canonical_sha,
                    "stdin_payload": "bash -s",
                }
            else:
                log_path = Path(evidence.get("log_path", ""))
                log_sha = evidence.get("log_sha256")
                if not isinstance(log_sha, str) or len(log_sha) != 64 or sha256_file(log_path) != log_sha:
                    raise GateError("PRIOR_CAUSE_EVIDENCE_SHA_MISMATCH", str(log_path))
                if not isinstance(argv_value, list) or argv_value[:3] != ["docker", "exec", "-i"] or \
                        argv_value.count("-i") != 1 or argv_value[-2:] != ["bash", "-s"]:
                    raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt2 docker exec argv drift")
                log_text = log_path.read_text(encoding="utf-8")
                typo = "MAPPER_PID: command not found"
                if typo not in log_text:
                    raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt2 mapper PID typo absent")
                verified_cause_evidence = {
                    "kind": "mapper_pid_assignment_typo",
                    "argv_path": str(argv_path), "argv_sha256": argv_sha,
                    "argv_sidecar_path": str(argv_sidecar_path),
                    "argv_sidecar_sha256": sidecar_sha,
                    "argv_canonical_sha256": canonical_sha,
                    "stdin_payload": "bash -s", "log_path": str(log_path),
                    "log_sha256": log_sha, "log_exact_fragment": typo,
                }
        elif attempt_index == 3:
            receipt_sidecar = path.with_suffix(path.suffix + ".sha256")
            receipt_sidecar_sha = item.get("receipt_sidecar_sha256")
            if not isinstance(receipt_sidecar_sha, str) or len(receipt_sidecar_sha) != 64 or \
                    sha256_file(receipt_sidecar) != receipt_sidecar_sha or \
                    receipt_sidecar.read_text(encoding="ascii") != f"{expected_sha}  {path.name}\n":
                raise GateError("PRIOR_RECEIPT_SIDECAR_INVALID", str(receipt_sidecar))
            failure_message = receipt.get("failure_message")
            fragments = evidence.get("failure_message_fragments")
            if not isinstance(failure_message, str) or not isinstance(fragments, (tuple, list)) or \
                    not all(isinstance(fragment, str) and fragment in failure_message for fragment in fragments):
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt3 syntax-error evidence drift")
            execution = receipt.get("execution")
            if not isinstance(execution, Mapping) or execution.get("start_count") != 1 or \
                    execution.get("one_start") is not True or execution.get("retry_count") != 0 or \
                    execution.get("manual_stop") is not False:
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt3 execution cardinality drift")
            if receipt.get("services") != list(EXPECTED_SERVICES) or receipt.get("protocol") is not None or \
                    receipt.get("callback") is not None or receipt.get("terminal") is not None:
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt3 publisher failed after protocol start")
            safety = receipt.get("safety")
            if safety != {
                    "formal_replay_started": False,
                    "ground_truth_content_opened": False,
                    "host_mounts_exposed": False,
                    "input_opened": False,
                    "map_saved": False,
                    "scorer_invoked": False,
            }:
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt3 safety drift")
            cleanup = receipt.get("cleanup")
            diagnostics = receipt.get("diagnostics")
            after = diagnostics.get("inspect_after_stop") if isinstance(diagnostics, Mapping) else None
            if not isinstance(cleanup, Mapping) or cleanup.get("stopped_only") is not True or \
                    not isinstance(cleanup.get("remove"), Mapping) or cleanup["remove"].get("returncode") != 0 or \
                    not isinstance(after, Mapping) or after.get("status") != "exited" or \
                    after.get("oom_killed") is not False or receipt.get("status") != "FAIL_CLOSED":
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt3 cleanup evidence drift")
            verified_cause_evidence = {
                "kind": "synthetic_publisher_syntax_error_before_callback",
                "receipt_sidecar_path": str(receipt_sidecar),
                "receipt_sidecar_sha256": receipt_sidecar_sha,
                "failure_message_fragments": list(fragments),
                "services_reached": len(EXPECTED_SERVICES),
                "start_count": 1,
                "published_lidar_callbacks": 0,
                "container_exit_code": after.get("exit_code"),
                "publisher_failed_before_callback": True,
                "safety_all_false": True,
                "cleanup_stopped_only": True,
                "cleanup_remove_returncode": cleanup["remove"].get("returncode"),
                "cleanup_oom_killed": after.get("oom_killed"),
            }
        elif attempt_index == 4:
            receipt_sidecar = path.with_suffix(path.suffix + ".sha256")
            receipt_sidecar_sha = item.get("receipt_sidecar_sha256")
            if not isinstance(receipt_sidecar_sha, str) or len(receipt_sidecar_sha) != 64 or \
                    sha256_file(receipt_sidecar) != receipt_sidecar_sha or \
                    receipt_sidecar.read_text(encoding="ascii") != f"{expected_sha}  {path.name}\n":
                raise GateError("PRIOR_RECEIPT_SIDECAR_INVALID", str(receipt_sidecar))
            if receipt.get("attempt_index") != item.get("receipt_attempt_index"):
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt4 observed hardcoded attempt index drift")
            failure_message = receipt.get("failure_message")
            fragments = evidence.get("failure_message_fragments")
            if not isinstance(failure_message, str) or not isinstance(fragments, (tuple, list)) or \
                    not all(isinstance(fragment, str) and fragment in failure_message for fragment in fragments):
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt4 rospy import evidence drift")
            execution = receipt.get("execution")
            if not isinstance(execution, Mapping) or execution.get("start_count") != 1 or \
                    execution.get("one_start") is not True or execution.get("retry_count") != 0 or \
                    execution.get("manual_stop") is not False:
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt4 execution cardinality drift")
            if receipt.get("services") != list(EXPECTED_SERVICES) or receipt.get("protocol") is not None or \
                    receipt.get("callback") is not None or receipt.get("terminal") is not None:
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt4 publisher failed after protocol start")
            safety = receipt.get("safety")
            if safety != {
                    "formal_replay_started": False,
                    "ground_truth_content_opened": False,
                    "host_mounts_exposed": False,
                    "input_opened": False,
                    "map_saved": False,
                    "scorer_invoked": False,
            }:
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt4 safety drift")
            cleanup = receipt.get("cleanup")
            diagnostics = receipt.get("diagnostics")
            after = diagnostics.get("inspect_after_stop") if isinstance(diagnostics, Mapping) else None
            if not isinstance(cleanup, Mapping) or cleanup.get("stopped_only") is not True or \
                    not isinstance(cleanup.get("remove"), Mapping) or cleanup["remove"].get("returncode") != 0 or \
                    not isinstance(after, Mapping) or after.get("status") != "exited" or \
                    after.get("oom_killed") is not False or receipt.get("status") != "FAIL_CLOSED":
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt4 cleanup evidence drift")
            verified_cause_evidence = {
                "kind": "ros_entrypoint_not_used_for_rospy_import",
                "receipt_sidecar_path": str(receipt_sidecar),
                "receipt_sidecar_sha256": receipt_sidecar_sha,
                "receipt_attempt_index_observed": receipt.get("attempt_index"),
                "failure_message_fragments": list(fragments),
                "services_reached": len(EXPECTED_SERVICES),
                "start_count": 1,
                "published_lidar_callbacks": 0,
                "container_exit_code": after.get("exit_code"),
                "publisher_failed_before_callback": True,
                "safety_all_false": True,
                "cleanup_stopped_only": True,
                "cleanup_remove_returncode": cleanup["remove"].get("returncode"),
                "cleanup_oom_killed": after.get("oom_killed"),
            }
        elif attempt_index == 5:
            receipt_sidecar = path.with_suffix(path.suffix + ".sha256")
            receipt_sidecar_sha = item.get("receipt_sidecar_sha256")
            if not isinstance(receipt_sidecar_sha, str) or len(receipt_sidecar_sha) != 64 or \
                    sha256_file(receipt_sidecar) != receipt_sidecar_sha or \
                    receipt_sidecar.read_text(encoding="ascii") != f"{expected_sha}  {path.name}\n":
                raise GateError("PRIOR_RECEIPT_SIDECAR_INVALID", str(receipt_sidecar))
            if receipt.get("attempt_index") != 5:
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt5 receipt index drift")
            failure_message = receipt.get("failure_message")
            fragments = evidence.get("failure_message_fragments")
            artifact_path = evidence.get("artifact_path")
            if not isinstance(failure_message, str) or not isinstance(fragments, (tuple, list)) or \
                    not all(isinstance(fragment, str) and fragment in failure_message for fragment in fragments) or \
                    artifact_path != CALLBACK_RAW:
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt5 artifact failure evidence drift")
            execution = receipt.get("execution")
            if not isinstance(execution, Mapping) or execution.get("start_count") != 1 or \
                    execution.get("one_start") is not True or execution.get("retry_count") != 0 or \
                    execution.get("manual_stop") is not False:
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt5 execution cardinality drift")
            if receipt.get("services") != list(EXPECTED_SERVICES) or receipt.get("protocol") is not None or \
                    receipt.get("callback") is not None or receipt.get("terminal") is not None:
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt5 artifact failure occurred after protocol start")
            safety = receipt.get("safety")
            if safety != {
                    "formal_replay_started": False,
                    "ground_truth_content_opened": False,
                    "host_mounts_exposed": False,
                    "input_opened": False,
                    "map_saved": False,
                    "scorer_invoked": False,
            }:
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt5 safety drift")
            cleanup = receipt.get("cleanup")
            diagnostics = receipt.get("diagnostics")
            after = diagnostics.get("inspect_after_stop") if isinstance(diagnostics, Mapping) else None
            if not isinstance(cleanup, Mapping) or cleanup.get("stopped_only") is not True or \
                    not isinstance(cleanup.get("remove"), Mapping) or cleanup["remove"].get("returncode") != 0 or \
                    not isinstance(after, Mapping) or after.get("status") != "exited" or \
                    after.get("oom_killed") is not False or receipt.get("status") != "FAIL_CLOSED":
                raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", "attempt5 cleanup evidence drift")
            verified_cause_evidence = {
                "kind": "artifact_copy_callback_raw_failure",
                "receipt_sidecar_path": str(receipt_sidecar),
                "receipt_sidecar_sha256": receipt_sidecar_sha,
                "failure_message_fragments": list(fragments),
                "artifact_path": artifact_path,
                "services_reached": len(EXPECTED_SERVICES),
                "start_count": 1,
                "published_lidar_callbacks": 1,
                "container_exit_code": after.get("exit_code"),
                "safety_all_false": True,
                "cleanup_stopped_only": True,
                "cleanup_remove_returncode": cleanup["remove"].get("returncode"),
                "cleanup_oom_killed": after.get("oom_killed"),
            }
        else:
            raise GateError("PRIOR_CAUSE_EVIDENCE_INVALID", f"unsupported attempt index: {attempt_index}")
        bound_item = {**dict(item), "path": str(path), "receipt_status": receipt["status"],
                      "verified_cause_evidence": verified_cause_evidence}
        bound_item["cause_evidence"] = {
            key: str(value) if isinstance(value, Path) else value
            for key, value in evidence.items()
        }
        checked.append(bound_item)
    return checked


def verify_sources(repo_root: Path = ROOT) -> dict[str, Any]:
    pins = ((PATCH_PATH, PATCH_SHA256), (WRAPPER_PATH, WRAPPER_SHA256),
            (PAYLOAD_PATH, PAYLOAD_SHA256), (PROFILE_PATH, PROFILE_SHA256))
    result = {}
    for relative, expected in pins:
        path = repo_root / relative
        if sha256_file(path) != expected:
            raise GateError("SOURCE_PIN_MISMATCH", str(path))
        result[str(relative)] = {"path": str(path), "sha256": expected}
    runner_path = Path(__file__).resolve()
    focused_test_path = repo_root / FOCUSED_TEST_PATH
    result["runner_runtime_observed"] = {
        "path": str(runner_path),
        "sha256": sha256_file(runner_path),
        "binding": "observed_runtime_self_hash_not_source_pin",
    }
    result["focused_test_runtime_observed"] = {
        "path": str(focused_test_path),
        "sha256": sha256_file(focused_test_path),
        "binding": "observed_test_evidence_not_source_pin",
    }
    return result


def _image_probe_default(image_id: str) -> Mapping[str, Any]:
    completed = subprocess.run(
        ("docker", "image", "inspect", "--format", "{{json .}}", image_id),
        check=False, capture_output=True, text=True)
    if completed.returncode != 0:
        raise GateError("IMAGE_INSPECT_FAILURE", completed.stderr.strip())
    try:
        value = json.loads(completed.stdout.strip())
        if isinstance(value, list):
            value = value[0]
        labels = value.get("Config", {}).get("Labels", {})
        tags = value.get("RepoTags", [])
    except (IndexError, AttributeError, TypeError, json.JSONDecodeError) as error:
        raise GateError("IMAGE_INSPECT_INVALID", "invalid inspect JSON") from error
    if value.get("Id") != image_id or IMAGE_TAG not in tags:
        raise GateError("IMAGE_IDENTITY_MISMATCH", "image ID/tag drift")
    if value.get("Config", {}).get("Entrypoint") != [ROS_ENTRYPOINT]:
        raise GateError("IMAGE_ENTRYPOINT_MISMATCH", "image does not expose the pinned ROS entrypoint")
    if labels.get("benchmark.fast_livo2.m6a10_v12_patch_sha256") != PATCH_SHA256 or \
            labels.get("benchmark.fast_livo2.m6a10_v12_wrapper_sha256") != WRAPPER_SHA256:
        raise GateError("IMAGE_LABEL_MISMATCH", "image source labels drift")
    return {"id": image_id, "tag": IMAGE_TAG, "labels": dict(labels), "entrypoint": [ROS_ENTRYPOINT]}


def build_run_argv(container_name: str) -> list[str]:
    if re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,127}", container_name) is None:
        raise GateError("CONTAINER_NAME_INVALID", container_name)
    env = {
        "M6A10_PHASE_CONTRACT_VERSION": PHASE_CONTRACT,
        "M6A10_PHASE_MODE": PHASE_MODE,
        "M6A10_FAST_EXPECTED_MESSAGES": "1",
        "M6A10_FAST_EXPECTED_LIDAR_MESSAGES": "1",
        "M6A10_FAST_EXPECTED_IMU_MESSAGES": "0",
        "M6A10_FAST_EXPECTED_IMAGE_MESSAGES": "0",
        "M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS": "10.0",
        "M6A10_FAST_MAX_END_GAP_SECONDS": "0.25",
        "M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS": "0.05",
        "M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS": "0.25",
        "M6A10_FAST_MAX_BACKLOG_MESSAGES": "1",
        "M6A10_CONSUMER_EVIDENCE": CALLBACK_RAW,
        "M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE": TERMINAL_RAW,
        "ROS_MASTER_URI": "http://127.0.0.1:11311",
        "ROS_IP": "127.0.0.1", "ROS_HOSTNAME": "127.0.0.1",
        "ROS_HOME": "/root/.ros", "ROS_LOG_DIR": "/out/ros_logs",
    }
    argv = ["docker", "run", "-d", "-i", "--name", container_name,
            "--pull=never", "--init", "--network", "none", "--read-only",
            "--tmpfs", "/tmp:rw,noexec,nosuid,size=128m",
            "--tmpfs", "/root/.ros:rw,noexec,nosuid,size=32m",
            "--tmpfs", "/out:rw,noexec,nosuid,size=128m"]
    for key, value in env.items():
        argv.extend(("--env", f"{key}={value}"))
    argv.extend((IMAGE_ID, "tail", "-f", "/dev/null"))
    validate_run_argv(argv)
    return argv


def build_exec_argv(container_name: str) -> list[str]:
    if re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,127}", container_name) is None:
        raise GateError("CONTAINER_NAME_INVALID", container_name)
    return ["docker", "exec", "-i", container_name, "bash", "-s"]


SYNTHETIC_PUBLISHER_SOURCE = """\
import struct
import time

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


# This is the v9 Ouster/EIGEN-aligned layout verbatim.  The only intentional
# deltas from the v9 handshake source are this no-input node name, omission of
# its marker file, and the fixed required-end stamp of 10.0 seconds.
rospy.init_node(
    'm6a10_no_input_lidar_once',
    anonymous=False,
    disable_signals=True,
)
publisher = rospy.Publisher(
    '/os1_cloud_node1/points',
    PointCloud2,
    queue_size=1,
    latch=False,
)
deadline = time.monotonic() + 30.0
while publisher.get_num_connections() < 1 and time.monotonic() < deadline:
    time.sleep(0.1)
assert publisher.get_num_connections() >= 1

fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('intensity', 16, PointField.FLOAT32, 1),
    PointField('t', 20, PointField.UINT32, 1),
    PointField('reflectivity', 24, PointField.UINT16, 1),
    PointField('ring', 26, PointField.UINT8, 1),
    PointField('ambient', 28, PointField.UINT16, 1),
    PointField('range', 32, PointField.UINT32, 1),
]
payload = bytearray(36)
struct.pack_into('<f', payload, 0, 2.0)
struct.pack_into('<f', payload, 4, 0.0)
struct.pack_into('<f', payload, 8, 0.0)
struct.pack_into('<f', payload, 16, 1.0)
struct.pack_into('<I', payload, 20, 0)
struct.pack_into('<H', payload, 24, 1)
struct.pack_into('<B', payload, 26, 0)
struct.pack_into('<H', payload, 28, 0)
struct.pack_into('<I', payload, 32, 2000)
message = PointCloud2(
    header=Header(stamp=rospy.Time.from_sec(10.0), frame_id='os_sensor'),
    height=1, width=1, fields=fields, is_bigendian=False,
    point_step=36, row_step=36, data=bytes(payload), is_dense=True,
)
publisher.publish(message)
time.sleep(2.0)
""".strip()


def build_publish_argv(container_name: str) -> list[str]:
    return ["docker", "exec", container_name, ROS_ENTRYPOINT, "python3", "-c", SYNTHETIC_PUBLISHER_SOURCE]


def build_service_argv(container_name: str, service: str) -> list[str]:
    if service not in EXPECTED_SERVICES:
        raise GateError("SERVICE_NOT_PINNED", service)
    return ["docker", "exec", container_name, ROS_ENTRYPOINT, "rosservice", "call", service, "{}"]


def validate_run_argv(argv: Sequence[str]) -> None:
    rendered = " ".join(argv).lower()
    if argv[:2] != ["docker", "run"] or IMAGE_ID not in argv or IMAGE_TAG in argv:
        raise GateError("UNSAFE_DOCKER_ARG", "run argv is not digest-bound")
    if argv.count("-d") != 1 or argv.count("-i") != 1 or "--rm" in argv:
        raise GateError("UNSAFE_DOCKER_ARG", "detached/interactive/no-rm contract drift")
    if any(flag in argv for flag in ("--mount", "--volume", "-v", "--device", "--privileged", "--cap-add")):
        raise GateError("UNSAFE_DOCKER_ARG", "host mount or privilege leaked")
    if argv.count("--tmpfs") != 3 or "--read-only" not in argv or "none" not in argv:
        raise GateError("UNSAFE_DOCKER_ARG", "rootfs/network/tmpfs isolation drift")
    if any(item in rendered for item in ("/input/", "rosbag", "ground_truth", "scorer", "map_save")):
        raise GateError("UNSAFE_DOCKER_ARG", "input/evaluation surface leaked")
    if rendered.count(IMAGE_ID.lower()) != 1:
        raise GateError("UNSAFE_DOCKER_ARG", "image ID must occur once")


def _validate_container_path(path: Any) -> str:
    if not isinstance(path, str) or not path.startswith("/out/") or "\x00" in path:
        raise GateError("ARTIFACT_PATH_INVALID", str(path))
    parts = path.split("/")
    if len(parts) < 3 or parts[0] != "" or parts[1] != "out" or any(part in ("", ".", "..") for part in parts[2:]):
        raise GateError("ARTIFACT_PATH_INVALID", str(path))
    return path


def _validate_pinned_artifact_path(path: Any) -> str:
    path = _validate_container_path(path)
    if path not in PINNED_RAW_PATHS + PINNED_DIAGNOSTIC_PATHS:
        raise GateError("ARTIFACT_PATH_NOT_PINNED", path)
    return path


def build_manifest_argv(container_name: str) -> list[str]:
    if re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,127}", container_name) is None:
        raise GateError("CONTAINER_NAME_INVALID", container_name)
    return ["docker", "exec", container_name, "find", "/out", "-maxdepth", "2", "-type", "f", "-printf", "%p\\n"]


def build_artifact_cat_argv(container_name: str, path: str) -> list[str]:
    if re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,127}", container_name) is None:
        raise GateError("CONTAINER_NAME_INVALID", container_name)
    return ["docker", "exec", container_name, "cat", _validate_pinned_artifact_path(path)]


def _default_run(argv: Sequence[str], log_path: Path) -> str:
    completed = subprocess.run(argv, check=False, capture_output=True, text=True)
    log_path.write_text(completed.stdout + completed.stderr, encoding="utf-8")
    if completed.returncode != 0 or not completed.stdout.strip():
        raise GateError("CONTAINER_START_FAILURE", completed.stderr.strip())
    return completed.stdout.strip().splitlines()[-1]


def _default_exec_payload(name: str, payload: bytes, log_path: Path) -> Any:
    stream = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(build_exec_argv(name), stdin=subprocess.PIPE,
                               stdout=stream, stderr=subprocess.STDOUT)
    if process.stdin is None:
        raise GateError("PAYLOAD_EXEC_FAILURE", "exec stdin unavailable")
    process.stdin.write(payload)
    process.stdin.close()
    return process


def _default_wait_ready(name: str, _payload_process: Any) -> Mapping[str, Any]:
    deadline = time.monotonic() + MAX_RUNTIME_SECONDS
    while time.monotonic() < deadline:
        marker = subprocess.run(("docker", "exec", name, "test", "-f", READY_MARKER), check=False)
        if marker.returncode == 0:
            services = subprocess.run(
                ("docker", "exec", name, "cat", SERVICES_LIST),
                check=False, capture_output=True, text=True,
            )
            observed = services.stdout.splitlines() if services.returncode == 0 else []
            if observed == EXPECTED_SERVICES:
                return {"services": list(EXPECTED_SERVICES), "ready": True}
        time.sleep(0.2)
    raise GateError("SERVICES_TIMEOUT", "payload did not report seven services")


def _default_publish(name: str) -> Mapping[str, Any]:
    completed = subprocess.run(build_publish_argv(name), check=False, capture_output=True, text=True, timeout=45)
    if completed.returncode != 0:
        raise GateError("SYNTHETIC_PUBLISH_FAILURE", completed.stderr.strip())
    return {"published_lidar_callbacks": 1, "argv": build_publish_argv(name)}


def _default_service(name: str, service: str) -> Mapping[str, Any]:
    completed = subprocess.run(build_service_argv(name, service), check=False, capture_output=True, text=True, timeout=15)
    if completed.returncode != 0:
        raise GateError("SERVICE_CALL_FAILURE", service)
    try:
        import yaml
        value = yaml.safe_load(completed.stdout) or {}
    except (ImportError, ValueError):
        value = {"success": "success: true" in completed.stdout.lower(), "message": completed.stdout}
    if not isinstance(value, dict):
        value = {"success": False, "message": completed.stdout}
    value["service"] = service
    value["stdout"] = completed.stdout
    return value


def _default_manifest(name: str) -> Mapping[str, Any]:
    argv = build_manifest_argv(name)
    completed = subprocess.run(argv, check=False, capture_output=True, text=True, timeout=15)
    if completed.returncode != 0:
        raise GateError("ARTIFACT_MANIFEST_FAILURE", completed.stderr.strip())
    paths = []
    for line in completed.stdout.splitlines():
        if line:
            paths.append(_validate_container_path(line))
    return {"argv": argv, "returncode": completed.returncode, "paths": paths,
            "stdout": completed.stdout, "stderr": completed.stderr}


def _default_read_file(name: str, source: str) -> bytes:
    argv = build_artifact_cat_argv(name, source)
    completed = subprocess.run(argv, check=False, capture_output=True, timeout=15)
    if completed.returncode != 0:
        raise GateError("ARTIFACT_READ_FAILURE", source)
    return bytes(completed.stdout)


def _atomic_write_bytes(path: Path, payload: bytes) -> None:
    if path.exists() or path.is_symlink():
        raise GateError("ARTIFACT_OVERWRITE", str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.parent.is_symlink():
        raise GateError("ARTIFACT_PARENT_INVALID", str(path.parent))
    part = path.with_name(f".{path.name}.part")
    if part.exists() or part.is_symlink():
        raise GateError("ARTIFACT_OVERWRITE", str(part))
    fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(part, path)
    finally:
        if part.exists() and not part.is_symlink():
            part.unlink()
    path.chmod(0o444)


def _prepare_service_journal(path: Path) -> None:
    if path.exists() or path.is_symlink():
        raise GateError("SERVICE_JOURNAL_OVERWRITE", str(path))
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(fd, "wb") as stream:
        stream.flush()
        os.fsync(stream.fileno())


def _append_service_journal(path: Path, record: Mapping[str, Any]) -> None:
    payload = (json.dumps(dict(record), sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
    fd = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
    try:
        with os.fdopen(fd, "ab") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except OSError as error:
        raise GateError("SERVICE_JOURNAL_FAILURE", str(path)) from error


def _journal_value(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


def _call_service(name: str, service: str, hooks: "GateHooks",
                  state: MutableMapping[str, Any], journal_path: Path) -> Mapping[str, Any]:
    order = len(state.setdefault("service_responses", [])) + 1
    try:
        response = hooks.service(name, service)
    except Exception as error:
        record = {"order": order, "service": service, "success": False,
                  "message": str(error), "stdout": ""}
        _append_service_journal(journal_path, record)
        state["service_responses"].append(record)
        raise
    if not isinstance(response, Mapping):
        record = {"order": order, "service": service, "success": False,
                  "message": "non-mapping service response", "stdout": ""}
        _append_service_journal(journal_path, record)
        state["service_responses"].append(record)
        raise GateError("SERVICE_RESPONSE_INVALID", service)
    record = {
        "order": order,
        "service": service,
        "success": response.get("success") is True,
        "message": _journal_value(response.get("message")),
        "stdout": _journal_value(response.get("stdout", "")),
    }
    _append_service_journal(journal_path, record)
    state["service_responses"].append(record)
    return response


def _default_inspect(name: str) -> Mapping[str, Any]:
    completed = subprocess.run(("docker", "inspect", name), check=False, capture_output=True, text=True)
    if completed.returncode != 0:
        return {"status": "not_found", "returncode": completed.returncode}
    try:
        value = json.loads(completed.stdout)[0]
        state = value.get("State", {})
        return {"status": state.get("Status"), "exit_code": state.get("ExitCode"), "oom_killed": state.get("OOMKilled"), "raw": value}
    except (IndexError, TypeError, json.JSONDecodeError) as error:
        return {"status": "inspect_invalid", "error": str(error)}


def _default_stats(name: str) -> Mapping[str, Any]:
    result = subprocess.run(("docker", "stats", "--no-stream", "--no-trunc", name), check=False, capture_output=True, text=True)
    return {"returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr}


def _default_top(name: str) -> Mapping[str, Any]:
    result = subprocess.run(("docker", "top", name, "-eo", "pid,comm,stat,pcpu,pmem"), check=False, capture_output=True, text=True)
    return {"returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr}


def _default_stop(name: str) -> Mapping[str, Any]:
    result = subprocess.run(("docker", "stop", "--time", str(int(STOP_GRACE_SECONDS)), name), check=False, capture_output=True, text=True)
    return {"returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr}


def _default_wait_container(name: str, timeout: float) -> Mapping[str, Any]:
    deadline = time.monotonic() + timeout
    state: Mapping[str, Any] = {}
    while time.monotonic() < deadline:
        state = _default_inspect(name)
        if state.get("status") in ("exited", "dead", "not_found"):
            return state
        time.sleep(0.2)
    raise GateError("STOP_TIMEOUT", "container did not become stopped")


def _default_remove(name: str) -> Mapping[str, Any]:
    result = subprocess.run(("docker", "rm", name), check=False, capture_output=True, text=True)
    if result.returncode != 0:
        raise GateError("REMOVE_FAILURE", result.stderr.strip())
    return {"returncode": result.returncode, "stdout": result.stdout}


@dataclass
class GateHooks:
    image_probe: Callable[[str], Mapping[str, Any]] = _image_probe_default
    run_container: Callable[[Sequence[str], Path], str] = _default_run
    exec_payload: Callable[[str, bytes, Path], Any] = _default_exec_payload
    wait_ready: Callable[[str, Any], Mapping[str, Any]] = _default_wait_ready
    publish: Callable[[str], Mapping[str, Any]] = _default_publish
    service: Callable[[str, str], Mapping[str, Any]] = _default_service
    manifest: Callable[[str], Mapping[str, Any]] = _default_manifest
    read_file: Callable[[str, str], bytes] = _default_read_file
    inspect: Callable[[str], Mapping[str, Any]] = _default_inspect
    stats: Callable[[str], Mapping[str, Any]] = _default_stats
    top: Callable[[str], Mapping[str, Any]] = _default_top
    stop: Callable[[str], Mapping[str, Any]] = _default_stop
    wait_container: Callable[[str, float], Mapping[str, Any]] = _default_wait_container
    remove: Callable[[str], Mapping[str, Any]] = _default_remove
    sleep: Callable[[float], None] = time.sleep


def _response_success(response: Mapping[str, Any]) -> bool:
    return response.get("success") is True


def _status_counts(response: Mapping[str, Any]) -> Mapping[str, Any]:
    if isinstance(response.get("received_topic_counts"), Mapping):
        return response
    message = response.get("message")
    if isinstance(message, str):
        try:
            value = json.loads(message)
            consumer = value.get("consumer", {})
            return {"received_messages": consumer.get("received_messages"), "received_topic_counts": consumer.get("received_topic_counts")}
        except (json.JSONDecodeError, AttributeError):
            pass
    return {}


def _require_success(response: Mapping[str, Any], service: str) -> None:
    if not _response_success(response):
        raise GateError("SERVICE_RESPONSE_INVALID", service)


def _finite_number(value: Any, label: str, *, maximum: Optional[float] = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise GateError("EVIDENCE_NUMERIC_FIELD", f"{label} is not finite")
    number = float(value)
    if number < 0.0 or (maximum is not None and number > maximum):
        raise GateError("EVIDENCE_NUMERIC_FIELD", f"{label} is outside bounds")
    return number


def _exact_counts(value: Any, label: str, expected: Optional[Mapping[str, int]] = None) -> dict[str, int]:
    expected = EXPECTED_COUNTS if expected is None else expected
    if not isinstance(value, Mapping) or dict(value) != dict(expected):
        raise GateError("EVIDENCE_COUNTS", f"{label} differs from exact no-input counts")
    for topic, count in value.items():
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise GateError("EVIDENCE_COUNTS", f"{label}.{topic} is invalid")
    return dict(value)


def _exact_integer(value: Any, expected: int, label: str) -> bool:
    if isinstance(value, bool) or not isinstance(value, int) or value != expected:
        return False
    return True


def validate_callback(value: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise GateError("CALLBACK_INVALID", "callback evidence must be an object")
    if value.get("schema_version") != 3 or value.get("contract_version") != PHASE_CONTRACT or \
            value.get("transport_contract_version") != TRANSPORT_CONTRACT or \
            value.get("phase_mode") != PHASE_MODE or value.get("system") != "fast_livo2" or \
            value.get("benchmark_only") is not True or value.get("status") != "pass":
        raise GateError("CALLBACK_INVALID", "callback schema/contract/status mismatch")
    if any(key in value for key in (
            "input", "profile_sha256", "profile_path", "raw_path", "raw_sha256",
            "expected", "expected_messages", "expected_topic_counts", "published",
            "published_messages", "ack", "acked", "binding", "validation")):
        raise GateError("CALLBACK_AUTHORITY_CONTAMINATION", "callback raw contains host fields")
    consumer = value.get("consumer")
    if not isinstance(consumer, Mapping):
        raise GateError("CALLBACK_COUNTS", "callback consumer ledger missing")
    if _exact_counts(consumer.get("expected_topic_counts"), "callback.expected_topic_counts") != EXPECTED_COUNTS or \
            _exact_counts(consumer.get("received_topic_counts"), "callback.received_topic_counts") != EXPECTED_COUNTS or \
            not _exact_integer(consumer.get("expected_messages"), 1, "callback.expected_messages") or \
            not _exact_integer(consumer.get("received_messages"), 1, "callback.received_messages") or \
            not _exact_integer(consumer.get("processed_messages"), 1, "callback.processed_messages") or \
            not _exact_integer(consumer.get("acked_messages"), 1, "callback.acked_messages"):
        raise GateError("CALLBACK_COUNTS", "callback exact counts/ACK mismatch")
    if consumer.get("ack_source_kind") != "consumer_callback" or \
            consumer.get("ack_source") != "LIVMapper subscriber callback return" or \
            consumer.get("ack_semantics") != "callback_acceptance_not_backend_completion" or \
            consumer.get("publisher_count_used") is not False:
        raise GateError("CALLBACK_ACK_SEMANTICS", "callback ACK source/semantics mismatch")
    if consumer.get("ack_exact") is not True or \
            not _exact_integer(consumer.get("transport_outstanding_at_drain"), 0, "callback.transport_outstanding_at_drain") or \
            not _exact_integer(consumer.get("maximum_transport_outstanding_messages"), 1, "callback.maximum_transport_outstanding_messages") or \
            not _exact_integer(consumer.get("maximum_allowed_transport_outstanding_messages"), 1, "callback.maximum_allowed_transport_outstanding_messages"):
        raise GateError("CALLBACK_TRANSPORT", "callback transport ledger mismatch")
    if not _exact_integer(consumer.get("queue_capacity_messages"), 1, "callback.queue_capacity_messages") or \
            not _exact_integer(consumer.get("maximum_allowed_backlog_messages"), 1, "callback.maximum_allowed_backlog_messages") or \
            isinstance(consumer.get("maximum_backlog_messages"), bool) or \
            not isinstance(consumer.get("maximum_backlog_messages"), int) or \
            consumer.get("maximum_backlog_messages") < 0:
        raise GateError("CALLBACK_QUEUE_CAPACITY", "callback queue capacity evidence mismatch")
    for key in ("mapper_internal_deque_current_messages", "mapper_internal_deque_peak_messages"):
        if not isinstance(consumer.get(key), int) or isinstance(consumer.get(key), bool) or consumer[key] < 0:
            raise GateError("CALLBACK_DIAGNOSTIC", key)
    if consumer.get("queue_overflow_observable") is not True or \
            not _exact_integer(consumer.get("dropped_messages"), 0, "callback.dropped_messages") or \
            not _exact_integer(consumer.get("queue_overflow"), 0, "callback.queue_overflow") or \
            not _exact_integer(consumer.get("processing_failures"), 0, "callback.processing_failures"):
        raise GateError("CALLBACK_FAILURE_COUNTER", "callback failure counter nonzero")
    if consumer.get("eof_observed") is not True or consumer.get("eof_source") != "/m6a10/consumer_eof service" or \
            consumer.get("drain_complete") is not True or consumer.get("ack_backpressure_verified") is not True:
        raise GateError("CALLBACK_COMPLETION", "callback completion flag false")
    if _finite_number(consumer.get("maximum_callback_latency_seconds"), "callback latency", maximum=0.25) > 0.25 or \
            consumer.get("maximum_allowed_callback_latency_seconds") != 0.25:
        raise GateError("CALLBACK_LATENCY", "callback latency bound mismatch")
    for key in ("first_processed_timestamp_seconds", "last_processed_timestamp_seconds",
                "required_end_timestamp_seconds"):
        if _finite_number(consumer.get(key), key) != 10.0:
            raise GateError("CALLBACK_TIMESTAMPS", f"{key} must equal 10.0")
    if consumer.get("paced_input_rate") != 1.0 or consumer.get("paced_input_rate_verified") is not False or \
            consumer.get("acknowledgement_contract") != "one_publish_waits_for_callback_then_one_ack_service_call" or \
            consumer.get("queue_drop_detection") != "exact_counts_plus_single_inflight_ack" or \
            consumer.get("single_message_buffer_verified") is not False or \
            consumer.get("counter_evidence_path") != CALLBACK_RAW:
        raise GateError("CALLBACK_CONTRACT_FIELDS", "callback production contract fields mismatch")
    if value.get("ground_truth_content_opened") is not False or value.get("scorer_invoked") is not False:
        raise GateError("CALLBACK_SAFETY", "callback safety drift")
    return dict(value)


def validate_terminal(value: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise GateError("TERMINAL_EXPECTED_INVALID", "terminal evidence must be an object")
    if value.get("schema_version") != 1 or value.get("contract_id") != TERMINAL_CONTRACT or \
            value.get("phase_mode") != PHASE_MODE or value.get("system") != "fast_livo2" or \
            value.get("status") != "invalid":
        raise GateError("TERMINAL_EXPECTED_INVALID", "terminal must be schema1 invalid")
    if any(key in value for key in (
            "input", "profile_sha256", "profile_path", "raw_path", "raw_sha256",
            "expected", "expected_messages", "expected_topic_counts", "published",
            "published_messages", "ack", "acked", "binding", "validation")):
        raise GateError("TERMINAL_AUTHORITY_CONTAMINATION", "terminal raw contains host fields")
    if _exact_counts(value.get("received_topic_counts"), "terminal.received_topic_counts") != EXPECTED_COUNTS:
        raise GateError("TERMINAL_COUNTS", "terminal received counts mismatch")
    zero = {"lidar": 0, "imu": 0, "image": 0}
    if _exact_counts(value.get("completed_counts"), "terminal.completed_counts", zero) != zero or \
            _exact_counts(value.get("dropped_counts"), "terminal.dropped_counts", zero) != zero or \
            _exact_counts(value.get("overflow_counts"), "terminal.overflow_counts", zero) != zero or \
            not _exact_integer(value.get("processing_failures"), 0, "terminal.processing_failures"):
        raise GateError("TERMINAL_COUNTERS", "terminal top-level counters are not zero")
    backend = value.get("backend")
    if not isinstance(backend, Mapping) or backend.get("quiescent") is not False or \
            backend.get("quiescence_observed") is not False:
        raise GateError("TERMINAL_REASON", "invalid terminal is not nonquiescent")
    boundary = backend.get("completed_boundary")
    if not isinstance(boundary, Mapping) or boundary.get("observed") is not False or \
            boundary.get("timestamp_seconds") is not None or boundary.get("sequence") is not None or \
            boundary.get("source") != "":
        raise GateError("TERMINAL_REASON", "invalid terminal is not no-boundary")
    if backend.get("completed_counts") != {"lidar": None, "imu": None, "image": None} or \
            _exact_counts(backend.get("dropped_counts"), "terminal.backend.dropped_counts", zero) != zero or \
            _exact_counts(backend.get("overflow_counts"), "terminal.backend.overflow_counts", zero) != zero or \
            not _exact_integer(backend.get("processing_failures"), 0, "terminal.backend.processing_failures") or \
            backend.get("completed_synchronization_units") is not None:
        raise GateError("TERMINAL_BACKEND_COUNTERS", "terminal backend counters are not zero")
    in_flight = backend.get("in_flight")
    if not isinstance(in_flight, Mapping) or in_flight.get("active") is not True:
        raise GateError("TERMINAL_IN_FLIGHT", "no-boundary terminal must remain in flight")
    buffers = value.get("buffers")
    if not isinstance(buffers, Mapping) or set(buffers) != set(EXPECTED_COUNTS):
        raise GateError("TERMINAL_BUFFERS", "terminal buffer map is incomplete")
    for topic, expected_count in EXPECTED_COUNTS.items():
        buffer = buffers.get(topic)
        if not isinstance(buffer, Mapping) or not _exact_integer(buffer.get("count"), expected_count, f"terminal.buffer.{topic}.count") or \
                not isinstance(buffer.get("records"), list) or len(buffer["records"]) != expected_count:
            raise GateError("TERMINAL_BUFFERS", f"terminal {topic} buffer count mismatch")
        if expected_count == 0 and (buffer.get("oldest_timestamp_seconds") is not None or \
                                    buffer.get("newest_timestamp_seconds") is not None):
            raise GateError("TERMINAL_BUFFERS", f"terminal {topic} empty timestamps are not null")
    lidar_record = buffers["lidar"]["records"][0]
    lidar_buffer = buffers["lidar"]
    if lidar_buffer.get("oldest_timestamp_seconds") != 10.0 or \
            lidar_buffer.get("newest_timestamp_seconds") != 10.0:
        raise GateError("TERMINAL_BUFFERS", "no-input lidar buffer timestamps must equal 10.0")
    if not isinstance(lidar_record, Mapping) or lidar_record.get("topic") != "lidar" or \
            lidar_record.get("record_id") != "lidar-0" or \
            _finite_number(lidar_record.get("timestamp_seconds"), "terminal lidar record timestamp") < 0.0 or \
            lidar_record.get("support_context_proven") is not False or \
            lidar_record.get("post_boundary") is not False or \
            lidar_record.get("equal_to_completed_boundary") is not False or \
            lidar_record.get("sync_predicate_evaluated") is not True or \
            lidar_record.get("can_form_synchronization_unit") is not False or \
            lidar_record.get("reason_code") != "residual_lidar_rejected":
        raise GateError("TERMINAL_LIDAR_RECORD", "terminal lidar record is not rejected/nonformable")
    support = value.get("terminal_support_context")
    if not isinstance(support, Mapping) or \
            support.get("classification") != "nonlidar_at_or_after_boundary_support_context" or \
            not _exact_integer(support.get("total_count"), 0, "terminal support total_count") or _exact_counts(support.get("by_topic"), "terminal support by-topic", zero) != zero or \
            support.get("records") != []:
        raise GateError("TERMINAL_SUPPORT", "terminal support context is not empty")
    observation = value.get("terminal_observation")
    if not isinstance(observation, Mapping) or observation.get("eof_observed") is not True or \
            observation.get("stable") is not True or not _exact_integer(observation.get("poll_count"), 2, "terminal.poll_count") or \
            not _exact_integer(observation.get("stable_poll_count"), 2, "terminal.stable_poll_count") or observation.get("identical_snapshot") is not True or \
            observation.get("sync_predicate_evaluated") is not True or observation.get("minimum_poll_wall_seconds") != 0.05:
        raise GateError("TERMINAL_OBSERVATION", "terminal EOF/stability proof is incomplete")
    first_poll = _finite_number(observation.get("first_poll_wall_seconds"), "terminal first poll")
    stable_poll = _finite_number(observation.get("stable_poll_wall_seconds"), "terminal stable poll")
    if stable_poll - first_poll < 0.05:
        raise GateError("TERMINAL_OBSERVATION", "terminal polls are not separated by 0.05s")
    trajectory = value.get("trajectory")
    if not isinstance(trajectory, Mapping) or trajectory.get("coverage_verified") is not True or \
            trajectory.get("last_timestamp_seconds") is not None or trajectory.get("end_gap_seconds") is not None:
        raise GateError("TERMINAL_TRAJECTORY", "no-boundary trajectory coverage/null timestamps mismatch")
    if value.get("ground_truth_content_opened") is not False or value.get("scorer_invoked") is not False:
        raise GateError("TERMINAL_SAFETY", "terminal safety drift")
    return dict(value)


def _load_json(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise GateError("EVIDENCE_MISSING", str(path))
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise GateError("EVIDENCE_INVALID", str(path))
    return value


def _seal_receipt(path: Path, value: Mapping[str, Any]) -> str:
    sidecar = path.with_suffix(path.suffix + ".sha256")
    if path.exists() or path.is_symlink() or sidecar.exists() or sidecar.is_symlink():
        raise GateError("RECEIPT_OVERWRITE", str(path))
    payload = (json.dumps(value, indent=2, sort_keys=True) + "\n").encode()
    part = path.with_name(f".{path.name}.part")
    fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(fd, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.link(part, path)
    part.unlink()
    path.chmod(0o444)
    digest = hashlib.sha256(payload).hexdigest()
    side_part = sidecar.with_name(f".{sidecar.name}.part")
    fd = os.open(side_part, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(fd, "wb") as stream:
        stream.write(f"{digest}  {path.name}\n".encode())
        stream.flush()
        os.fsync(stream.fileno())
    os.link(side_part, sidecar)
    side_part.unlink()
    sidecar.chmod(0o444)
    return digest


def _capture_artifacts(name: str, root: Path, hooks: "GateHooks",
                       state: MutableMapping[str, Any]) -> tuple[Path, Path]:
    manifest = hooks.manifest(name)
    if not isinstance(manifest, Mapping):
        raise GateError("ARTIFACT_MANIFEST_INVALID", "manifest response is not an object")
    manifest_paths = manifest.get("paths")
    if not isinstance(manifest_paths, (list, tuple)):
        raise GateError("ARTIFACT_MANIFEST_INVALID", "manifest paths are missing")
    checked_paths = [_validate_container_path(path) for path in manifest_paths]
    manifest_value = dict(manifest)
    manifest_value["paths"] = checked_paths
    manifest_host = root / ARTIFACT_MANIFEST_NAME
    _atomic_write_bytes(manifest_host, (json.dumps(manifest_value, indent=2, sort_keys=True) + "\n").encode("utf-8"))
    state["artifact_manifest"] = {
        "path": str(manifest_host), "sha256": sha256_file(manifest_host),
        "container_paths": checked_paths,
    }

    out_dir = root / "out"
    raw_errors = []
    raw_hosts: dict[str, Path] = {}
    for source in PINNED_RAW_PATHS:
        destination = out_dir / Path(source).name
        raw_hosts[source] = destination
        try:
            payload = hooks.read_file(name, _validate_pinned_artifact_path(source))
            if not isinstance(payload, (bytes, bytearray)):
                raise GateError("ARTIFACT_READ_INVALID", source)
            _atomic_write_bytes(destination, bytes(payload))
            state.setdefault("artifacts", {})[source] = {
                "path": str(destination), "sha256": sha256_file(destination),
                "bytes": destination.stat().st_size,
            }
        except Exception as error:
            raw_errors.append(f"{source}: {error}")

    diagnostic_errors = []
    for source in PINNED_DIAGNOSTIC_PATHS:
        destination = root / "diagnostics" / Path(source).name
        try:
            payload = hooks.read_file(name, _validate_pinned_artifact_path(source))
            if not isinstance(payload, (bytes, bytearray)):
                raise GateError("ARTIFACT_READ_INVALID", source)
            _atomic_write_bytes(destination, bytes(payload))
            state.setdefault("artifact_diagnostics", {})[source] = {
                "path": str(destination), "sha256": sha256_file(destination),
                "bytes": destination.stat().st_size,
            }
        except Exception as error:
            diagnostic_errors.append(f"{source}: {error}")
    if diagnostic_errors:
        state["artifact_diagnostic_errors"] = diagnostic_errors
    if raw_errors:
        state["artifact_errors"] = raw_errors
        raise GateError("ARTIFACT_READ_FAILURE", "; ".join(raw_errors))
    return raw_hosts[CALLBACK_RAW], raw_hosts[TERMINAL_RAW]


def run_no_input_gate(root: Path, *, repo_root: Path = ROOT,
                      hooks: Optional[GateHooks] = None) -> dict[str, Any]:
    """Execute one host lifecycle attempt and seal PASS/FAIL_CLOSED evidence."""

    root = reserve_output_root(root)
    hooks = hooks or GateHooks()
    name = _container_name(root)
    receipt_path = root / "no_input_dual_evidence.receipt.json"
    state: MutableMapping[str, Any] = {
        "schema_version": 1, "kind": "fast_livo2_m6a10_v12_no_input_dual_evidence_gate",
        "contract_version": CONTAINER_CONTRACT, "status": "FAIL_CLOSED",
        "failure_kind": None, "attempt_index": None, "retry_of_same_root": False,
        "image": {"tag": IMAGE_TAG, "id": IMAGE_ID}, "container_name": name,
        "execution": {"one_start": True, "start_count": 0, "retry_count": 0,
                       "manual_stop": False, "network": "none", "rootfs": "read_only",
                       "host_mounts": [], "input_mounts": 0,
                       "tmpfs": ["/tmp", "/root/.ros", "/out"]},
        "safety": {"input_opened": False, "ground_truth_content_opened": False,
                   "scorer_invoked": False, "map_saved": False,
                   "formal_replay_started": False, "host_mounts_exposed": False},
        "prior_attempts": [], "diagnostics": {},
    }
    container_started = False
    payload_process: Any = None
    stop_requested = False
    service_journal_path = root / SERVICE_JOURNAL_NAME
    state["service_journal_path"] = str(service_journal_path)
    state["service_responses"] = []
    try:
        _prepare_service_journal(service_journal_path)
        state["build_receipt"] = verify_build_receipt()
        state["prior_attempts"] = verify_prior_attempts()
        state["attempt_index"] = len(state["prior_attempts"]) + 1
        state["sources"] = verify_sources(repo_root)
        image = hooks.image_probe(IMAGE_ID)
        if image.get("id") != IMAGE_ID or image.get("tag") != IMAGE_TAG or \
                image.get("entrypoint") != [ROS_ENTRYPOINT]:
            raise GateError("IMAGE_IDENTITY_MISMATCH", "image hook differs from pin")
        state["image"] = dict(image)
        run_argv = build_run_argv(name)
        state["run_argv_sha256"] = hashlib.sha256(json.dumps(run_argv, separators=(",", ":")).encode()).hexdigest()
        hooks.run_container(run_argv, root / "docker.run.log")
        container_started = True
        state["execution"]["start_count"] = 1
        payload = (repo_root / PAYLOAD_PATH).read_bytes()
        payload_process = hooks.exec_payload(name, payload, root / "payload.exec.log")
        state["payload_sha256"] = hashlib.sha256(payload).hexdigest()
        ready = hooks.wait_ready(name, payload_process)
        if ready.get("services") != EXPECTED_SERVICES:
            raise GateError("SERVICES_CONTRACT", "payload did not expose all seven services")
        state["services"] = list(EXPECTED_SERVICES)
        published = hooks.publish(name)
        if published.get("published_lidar_callbacks") != 1:
            raise GateError("SYNTHETIC_COUNT", "synthetic publisher was not exactly one callback")
        status_response: Mapping[str, Any] = {}
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            status_response = _call_service(name, "/m6a10/consumer_status", hooks, state, service_journal_path)
            counts = _status_counts(status_response)
            if counts.get("received_messages") == 1 and counts.get("received_topic_counts") == EXPECTED_COUNTS:
                break
            hooks.sleep(0.1)
        else:
            raise GateError("CALLBACK_STATUS_TIMEOUT", "received count one was not observed")
        ack = _call_service(name, "/m6a10/consumer_ack", hooks, state, service_journal_path)
        if not _response_success(ack):
            raise GateError("ACK_FAILURE", "first ACK was not accepted")
        duplicate = _call_service(name, "/m6a10/consumer_ack", hooks, state, service_journal_path)
        if _response_success(duplicate):
            raise GateError("DUPLICATE_ACK_ACCEPTED", "duplicate ACK was accepted")
        consumer_sequence = []
        for service in ("/m6a10/consumer_eof", "/m6a10/consumer_status", "/m6a10/consumer_finalize"):
            _require_success(_call_service(name, service, hooks, state, service_journal_path), service)
            consumer_sequence.append(service.rsplit("/", 1)[-1])
        terminal_sequence = []
        for service in ("/m6a10/terminal_eof", "/m6a10/terminal_status"):
            _require_success(_call_service(name, service, hooks, state, service_journal_path), service)
            terminal_sequence.append(service.rsplit("/", 1)[-1])
        hooks.sleep(TERMINAL_POLL_GAP_SECONDS)
        _require_success(_call_service(name, "/m6a10/terminal_status", hooks, state, service_journal_path), "/m6a10/terminal_status")
        terminal_sequence.append("terminal_status")
        _require_success(_call_service(name, "/m6a10/terminal_finalize", hooks, state, service_journal_path), "/m6a10/terminal_finalize")
        terminal_sequence.append("terminal_finalize")
        callback_path, terminal_path = _capture_artifacts(name, root, hooks, state)
        callback = validate_callback(_load_json(callback_path))
        terminal = validate_terminal(_load_json(terminal_path))
        state["protocol"] = {
            "services": list(EXPECTED_SERVICES), "published_lidar_callbacks": 1,
            "status_received_counts": dict(EXPECTED_COUNTS), "first_ack_success": True,
            "duplicate_ack_rejected": True, "consumer_sequence": consumer_sequence,
            "terminal_sequence": terminal_sequence,
            "terminal_expected_invalid_no_completed_boundary": True,
        }
        state["callback"] = {"path": str(callback_path), "sha256": sha256_file(callback_path), "status": callback["status"]}
        state["terminal"] = {"path": str(terminal_path), "sha256": sha256_file(terminal_path), "status": terminal["status"], "expected_reason": "no_completed_estimator_boundary"}
        state["status"] = "PASS"
    except GateError as error:
        state["failure_kind"] = error.kind
        state["failure_message"] = str(error)
    except Exception as error:  # pragma: no cover - final fail-closed guard
        state["failure_kind"] = "UNEXPECTED_FAILURE"
        state["failure_message"] = str(error)
    finally:
        if container_started:
            def cleanup_failure(kind: str, error: Any) -> None:
                state.setdefault("cleanup_errors", []).append(f"{kind}: {error}")
                state["failure_kind"] = state.get("failure_kind") or "CLEANUP_FAILURE"

            before: Mapping[str, Any] = {}
            try:
                before = dict(hooks.inspect(name))
                state["diagnostics"]["inspect_before_stop"] = dict(before)
            except Exception as error:  # inspect failure must still take the safety-stop path
                cleanup_failure("inspect_before_stop", error)
            try:
                state["diagnostics"]["stats_before_stop"] = dict(hooks.stats(name))
            except Exception as error:
                cleanup_failure("stats_before_stop", error)
            try:
                state["diagnostics"]["top_before_stop"] = dict(hooks.top(name))
            except Exception as error:
                cleanup_failure("top_before_stop", error)

            current = before.get("status")
            if current not in ("exited", "dead", "not_found"):
                try:
                    stop_result = hooks.stop(name)
                    if isinstance(stop_result, Mapping) and stop_result.get("returncode", 0) != 0:
                        cleanup_failure("stop", stop_result)
                    stop_requested = True
                except Exception as error:
                    cleanup_failure("stop", error)
            if payload_process is not None and hasattr(payload_process, "wait"):
                try:
                    payload_process.wait(timeout=STOP_GRACE_SECONDS + 5)
                except Exception as error:
                    state["diagnostics"]["payload_wait"] = "timeout"
                    cleanup_failure("payload_wait", error)

            after: Mapping[str, Any] = {}
            try:
                after = dict(hooks.wait_container(name, STOP_GRACE_SECONDS + 5))
                state["diagnostics"]["inspect_after_stop"] = dict(after)
            except Exception as error:
                cleanup_failure("wait_container", error)
            if after.get("status") in ("exited", "dead"):
                try:
                    removed = dict(hooks.remove(name))
                    state["cleanup"] = {
                        "diagnostics_before_stop": True,
                        "stop_requested": stop_requested,
                        "stopped_only": True,
                        "remove": removed,
                    }
                except Exception as error:
                    cleanup_failure("remove", error)
            elif after:
                cleanup_failure("stopped_only", f"container state={after.get('status')}")
            if state.get("status") == "PASS":
                cleanup = state.get("cleanup")
                remove_result = cleanup.get("remove") if isinstance(cleanup, Mapping) else None
                if not isinstance(cleanup, Mapping) or \
                        cleanup.get("stopped_only") is not True or \
                        after.get("status") not in ("exited", "dead") or \
                        after.get("oom_killed") is not False or \
                        not isinstance(remove_result, Mapping) or \
                        remove_result.get("returncode") != 0:
                    cleanup_failure("cleanup_pass_requirements", "PASS requires stopped/non-OOM container and successful remove")
                else:
                    cleanup["stopped_state"] = after.get("status")
                    cleanup["oom_killed"] = False
                    cleanup["remove_success"] = True
        if service_journal_path.exists() and not service_journal_path.is_symlink():
            try:
                service_journal_path.chmod(0o444)
            except OSError as error:
                state["failure_kind"] = state.get("failure_kind") or "SERVICE_JOURNAL_FAILURE"
                state["failure_message"] = str(error)
        state["execution"]["stop_requested"] = stop_requested
        state["execution"]["started"] = container_started
        state["receipt_path"] = str(receipt_path)
        if state.get("failure_kind") or state.get("status") != "PASS":
            state["status"] = "FAIL_CLOSED"
        state["created_at_utc"] = dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")
    state["receipt_sha256"] = _seal_receipt(receipt_path, state)
    return dict(state)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    args = parser.parse_args(argv)
    result = run_no_input_gate(args.root, repo_root=args.repo_root)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
