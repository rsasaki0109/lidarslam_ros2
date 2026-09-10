#!/usr/bin/env python3
"""Actual-container, input-free v22 wrapper persistence gate.

The gate deliberately uses a tiny fake ROS/feeder surface mounted read-only
into the immutable v17 image. It exercises the mounted v22 wrapper inside a
real Docker container, with one RW /out bind, no input bind, network none,
and a read-only rootfs. Success and failure are separate fresh-root
contracts; this module never opens a bag and never authorizes formal replay.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Callable, Dict, Mapping, Optional, Sequence

ROOT = Path(__file__).resolve().parents[1]
IMAGE_TAG = "m6a10-v2c-v17-retryable-abort-correction-20260823t224017z-fast-livo2-benchmark:ros1-pinned"
IMAGE_ID = "sha256:f1432426af64e76d9ad9c655a8727d35c2fb0a03753502ada3b21187638f747c"
V22_WRAPPER = ROOT / "scripts/fast_livo2_m6a10_v22_formal_container_run.sh"
V22_WRAPPER_SHA256 = "6e74246d2b7ee60086f3b316b22c6ef69ca3f16307ffdecea3eae9ad2d794f67"
PROFILE_SHA256 = "1803ba255adfac96a61e1f677be53e84b196ad70bb367a10ebdb9791cb94128b"
BUILD_RECEIPT = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v17_build_correction_20260823T224017Z_agentv17/"
    "build_identity.receipt.json"
)
BUILD_RECEIPT_SHA256 = "23811d4200f9ed3ac464083ef339eb6d0437815ffdd82a2f37cc4180195bc2f7"
BUILD_SIDECAR_SHA256 = "76e9859832412f0310e685b376324ff53ce4dbe033d2d440fce940421b1de15b"
NO_INPUT_RECEIPT = Path(
    "/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/"
    "fast_livo2_v2c_v17_no_input_correction2_20260823T225359Z_agentv17/"
    "no_input.receipt.json"
)
NO_INPUT_RECEIPT_SHA256 = "ac1df6f390ad3e348e572ced668f4971bc05ac835c55cc83fef9ea4ad5fa3d74"
NO_INPUT_SIDECAR_SHA256 = "8dfa1520c1e2024af3f0710de4ee8c38c9cf4a1402389891ac77d423d1848164"
PHASE_CONTRACT = "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
TRANSPORT_CONTRACT = "m6a10-v12-callback-ack-transport-outstanding-v1"
TERMINAL_CONTRACT = "m6a10-fast-livo2-consumer-terminal-v1"
EXPECTED_COUNTS = {"lidar": 5793, "imu": 225102, "image": 5792}
EXPECTED_MESSAGES = 236687
EXPECTED_END = 1623491515.148352
EXPECTED_SENSOR_DURATION = 579.278127298
EXPECTED_BAG_PATH = "/input/ntu_viral.bag"
EXPECTED_BAG_BYTES = 11290464091
EXPECTED_BAG_SHA256 = "5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310"
EXPECTED_SERVICES = (
    "/m6a10/consumer_status", "/m6a10/consumer_ack",
    "/m6a10/consumer_eof", "/m6a10/consumer_finalize",
    "/m6a10/terminal_status", "/m6a10/terminal_eof",
    "/m6a10/terminal_finalize",
)
CONTRACT = "m6a10-v22-synthetic-wrapper-persistence-v1"


class GateError(RuntimeError):
    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise GateError("SOURCE_NOT_REGULAR", str(path))
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _seal_bytes(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path) or os.path.lexists(path.with_name(path.name + ".part")):
        raise GateError("OUTPUT_OVERWRITE", str(path))
    if not path.parent.is_dir() or path.parent.is_symlink():
        raise GateError("OUTPUT_PARENT_INVALID", str(path.parent))
    staging = path.with_name(path.name + ".part")
    fd = os.open(staging, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                 getattr(os, "O_NOFOLLOW", 0), 0o600)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(staging, path, follow_symlinks=False)
        os.chmod(path, mode, follow_symlinks=False)
        dfd = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(dfd)
        finally:
            os.close(dfd)
    finally:
        try:
            staging.unlink()
        except FileNotFoundError:
            pass
    return sha256_bytes(payload)


def _seal_json(path: Path, value: Mapping[str, Any]) -> str:
    return _seal_bytes(path, (json.dumps(value, sort_keys=True, indent=2) + "\n").encode())


def _source_script(name: str, payload: str, root: Path, mode: int = 0o755) -> Dict[str, Any]:
    path = root / name
    digest = _seal_bytes(path, payload.encode("utf-8"), mode)
    return {"path": str(path), "sha256": digest, "mode": mode}


def _fake_sources(root: Path, mode: str) -> Dict[str, Dict[str, Any]]:
    fakebin = root / "fakebin"
    synthetic = root / "synthetic"
    fakebin.mkdir()
    synthetic.mkdir()
    loop = """#!/usr/bin/env python3
import signal
import time
def stop(_signum, _frame):
    raise SystemExit(0)
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
while True:
    time.sleep(1.0)
"""
    param = """#!/usr/bin/env python3
import sys
if sys.argv[1:] and sys.argv[1] in ("list", "set"):
    raise SystemExit(0)
raise SystemExit(0)
"""
    feeder = f"""#!/usr/bin/env python3
import json
from pathlib import Path
value = {{
    "schema_version": 3,
    "status": "pass" if {mode!r} == "success" else "fail",
    "single_inflight": True,
    "bag_path": {EXPECTED_BAG_PATH!r},
    "bag_bytes": {EXPECTED_BAG_BYTES},
    "bag_sha256": {EXPECTED_BAG_SHA256!r},
    "published_messages": {EXPECTED_MESSAGES},
    "expected_topic_counts": {EXPECTED_COUNTS!r},
    "published_topic_counts": {EXPECTED_COUNTS!r},
    "acked_topic_counts": {EXPECTED_COUNTS!r},
    "ack_backpressure_verified": True,
    "ground_truth_content_opened": False,
    "scorer_invoked": False,
}}
Path("/out/feeder_receipt.json").write_text(json.dumps(value, sort_keys=True), encoding="utf-8")
if {mode!r} != "success":
    print("synthetic feeder deliberate nonzero", file=__import__("sys").stderr)
    raise SystemExit(17)
"""
    service = f"""#!/usr/bin/env python3
import json
from pathlib import Path
import sys
services = {list(EXPECTED_SERVICES)!r}
if sys.argv[1:] and sys.argv[1] == "list":
    print("\\n".join(services))
    raise SystemExit(0)
if len(sys.argv) < 3 or sys.argv[1] != "call":
    raise SystemExit(2)
name = sys.argv[2]
if {mode!r} == "success" and name == "/m6a10/consumer_finalize":
    callback = {{
        "schema_version": 3, "contract_version": {PHASE_CONTRACT!r},
        "transport_contract_version": {TRANSPORT_CONTRACT!r}, "status": "pass",
        "consumer": {{
            "received_topic_counts": {EXPECTED_COUNTS!r},
            "received_messages": {EXPECTED_MESSAGES}, "acked_messages": {EXPECTED_MESSAGES},
            "ack_exact": True, "transport_outstanding_at_drain": 0,
            "maximum_transport_outstanding_messages": 1,
            "maximum_allowed_transport_outstanding_messages": 1,
            "eof_observed": True, "drain_complete": True,
            "dropped_messages": 0, "queue_overflow": 0, "processing_failures": 0,
        }},
        "ground_truth_content_opened": False, "scorer_invoked": False,
    }}
    Path("/out/callback_consumer_evidence.json").write_text(json.dumps(callback, sort_keys=True), encoding="utf-8")
if {mode!r} == "success" and name == "/m6a10/terminal_finalize":
    terminal = {{
        "schema_version": 1, "contract_id": {TERMINAL_CONTRACT!r}, "status": "pass",
        "received_topic_counts": {EXPECTED_COUNTS!r},
        "ground_truth_content_opened": False, "scorer_invoked": False,
        "terminal_support_context": {{"by_topic": {{"lidar": 0, "imu": 0, "image": 0}}}},
        "trajectory": {{
            "coverage_verified": True, "last_timestamp_seconds": {EXPECTED_END - 0.1!r},
            "end_gap_seconds": 0.1,
        }},
        "terminal_observation": {{
            "eof_observed": True, "stable": True, "identical_snapshot": True,
            "poll_count": 2, "stable_poll_count": 2,
        }},
    }}
    Path("/out/consumer_evidence.json").write_text(json.dumps(terminal, sort_keys=True), encoding="utf-8")
print("success: True")
"""
    return {
        "roscore": _source_script("fakebin/roscore", loop, root),
        "roslaunch": _source_script("fakebin/roslaunch", loop, root),
        "rosparam": _source_script("fakebin/rosparam", param, root),
        "rosservice": _source_script("fakebin/rosservice", service, root),
        "rostopic": _source_script("fakebin/rostopic", loop, root),
        "feeder": _source_script("synthetic/feeder.py", feeder, root),
    }


def build_docker_argv(root: Path, mode: str, container_name: str,
                      feeder_sha256: str) -> list[str]:
    if mode not in {"success", "failure"}:
        raise GateError("MODE_INVALID", mode)
    if re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,127}", container_name) is None:
        raise GateError("CONTAINER_NAME_INVALID", container_name)
    wrapper = V22_WRAPPER.resolve()
    fakebin = (root / "fakebin").resolve()
    feeder = (root / "synthetic/feeder.py").resolve()
    out = (root / "out").resolve()
    argv = [
        "docker", "run", "--name", container_name, "--init", "--pull=never",
        "--network", "none", "--read-only",
        "--tmpfs", "/tmp:rw,nosuid,nodev",
        "--tmpfs", "/root/.ros:rw,nosuid,nodev",
        "--mount", f"type=bind,src={wrapper},dst=/runner/v22_runtime.sh,readonly",
        "--mount", f"type=bind,src={fakebin},dst=/runner/fakebin,readonly",
        "--mount", f"type=bind,src={feeder},dst=/runner/synthetic/feeder.py,readonly",
        "--mount", f"type=bind,src={out},dst=/out,readonly=false",
    ]
    env = {
        "PATH": "/runner/fakebin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        "M6A10_V22_SYNTHETIC_MODE": "1",
        "M6A10_V22_SYNTHETIC_FEEDER_PATH": "/runner/synthetic/feeder.py",
        "M6A10_V22_SYNTHETIC_FEEDER_SHA256": feeder_sha256,
        "M6A10_PROFILE_PATH": "/runner/configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_candidate.yaml",
        "M6A10_PROFILE_SHA256": PROFILE_SHA256,
        "M6A10_PHASE_CONTRACT_VERSION": PHASE_CONTRACT,
        "M6A10_PHASE_MODE": "unpaced_ack",
        "M6A10_FAST_FEEDER_SHA256": feeder_sha256,
        "M6A10_BAG_PATH": EXPECTED_BAG_PATH, "M6A10_BAG_BYTES": str(EXPECTED_BAG_BYTES),
        "M6A10_BAG_SHA256": EXPECTED_BAG_SHA256,
        "M6A10_FAST_EXPECTED_MESSAGES": str(EXPECTED_MESSAGES),
        "M6A10_FAST_EXPECTED_LIDAR_MESSAGES": str(EXPECTED_COUNTS["lidar"]),
        "M6A10_FAST_EXPECTED_IMU_MESSAGES": str(EXPECTED_COUNTS["imu"]),
        "M6A10_FAST_EXPECTED_IMAGE_MESSAGES": str(EXPECTED_COUNTS["image"]),
        "M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS": str(EXPECTED_END),
        "M6A10_FAST_MAX_END_GAP_SECONDS": "0.25",
        "M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS": "0.05",
        "M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS": "0.25",
        "M6A10_FAST_MAX_BACKLOG_MESSAGES": "1",
        "M6A10_SENSOR_DURATION_SECONDS": str(EXPECTED_SENSOR_DURATION),
        "M6A10_TIMING_CONTRACT_VERSION": "m6a10-online-compute-v3-timing-v1",
        "M6A10_CONSUMER_EVIDENCE": "/out/callback_consumer_evidence.json",
        "M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE": "/out/consumer_evidence.json",
        "M6A10_ONLINE_TIMING_EVIDENCE": "/out/online_compute_timing.json",
        "ROS_MASTER_URI": "http://127.0.0.1:11311", "ROS_IP": "127.0.0.1",
        "ROS_HOSTNAME": "127.0.0.1", "ROS_HOME": "/root/.ros",
        "ROS_LOG_DIR": "/out/ros_logs",
    }
    for key, value in env.items():
        argv.extend(("--env", f"{key}={value}"))
    argv.extend(("--entrypoint", "/bin/bash", IMAGE_ID, "/runner/v22_runtime.sh"))
    validate_docker_argv(argv, container_name)
    return argv


def validate_docker_argv(argv: Sequence[str], container_name: str) -> None:
    rendered = " ".join(argv).lower()
    if argv[:2] != ["docker", "run"] or IMAGE_ID not in argv or IMAGE_TAG in argv:
        raise GateError("ARGV_IDENTITY", "digest-bound image argv required")
    if argv.count("--name") != 1 or container_name not in argv or "--rm" in argv:
        raise GateError("ARGV_LIFECYCLE", "single named no-rm lifecycle required")
    if argv.count("--network") != 1 or argv[argv.index("--network") + 1] != "none" or "--read-only" not in argv:
        raise GateError("ARGV_ISOLATION", "network/rootfs isolation drift")
    tmpfs = [argv[i + 1] for i, item in enumerate(argv[:-1]) if item == "--tmpfs"]
    if sorted(tmpfs) != ["/root/.ros:rw,nosuid,nodev", "/tmp:rw,nosuid,nodev"]:
        raise GateError("ARGV_TMPFS", "tmpfs contract drift")
    mounts = [argv[i + 1] for i, item in enumerate(argv[:-1]) if item == "--mount"]
    if len(mounts) != 4 or sum(spec.endswith("dst=/out,readonly=false") for spec in mounts) != 1:
        raise GateError("ARGV_MOUNTS", "exact single RW out bind required")
    if any("dst=/input" in spec or "bag" in spec.lower() for spec in mounts):
        raise GateError("ARGV_INPUT", "input mount leaked")
    if any(item in rendered for item in ("rosbag play", "ground_truth", "scorer", "map_save")):
        raise GateError("ARGV_FORBIDDEN", "evaluation surface leaked")


def _inspect_shape(records: Any, name: str, run_rc: int) -> Dict[str, Any]:
    if not isinstance(records, list) or len(records) != 1:
        raise GateError("INSPECT_SHAPE", "container inspect is not one record")
    value = records[0]
    state = value.get("State") or {}
    host = value.get("HostConfig") or {}
    if value.get("Name") != "/" + name or value.get("Image") != IMAGE_ID:
        raise GateError("IDENTITY_MISMATCH", "container name/image drift")
    if state.get("Status") != "exited" or state.get("OOMKilled") is not False:
        raise GateError("CONTAINER_NOT_STOPPED", "container did not exit without OOM")
    if host.get("NetworkMode") != "none" or host.get("ReadonlyRootfs") is not True:
        raise GateError("ISOLATION_MISMATCH", "network/rootfs drift")
    mounts = value.get("Mounts") or []
    rw = [m for m in mounts if m.get("Destination") == "/out" and m.get("RW") is True]
    if len(rw) != 1 or any(m.get("Destination", "").startswith("/input") for m in mounts):
        raise GateError("MOUNT_CONTRACT", "RW out or input mount contract drift")
    if any(m.get("Destination") != "/out" and m.get("RW") is True for m in mounts):
        raise GateError("MOUNT_CONTRACT", "unexpected RW source mount")
    if int(run_rc) not in (0, 1) or state.get("ExitCode") != int(run_rc):
        raise GateError("EXIT_MISMATCH", "container/run exit mismatch")
    return value


def _artifact(path: Path) -> Dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise GateError("ARTIFACT_MISSING", str(path))
    raw = path.read_bytes()
    return {"path": str(path), "bytes": len(raw), "sha256": sha256_bytes(raw)}


def _verify_mode(root: Path, mode: str, run_rc: int) -> Dict[str, Any]:
    out = root / "out"
    status_path = out / "feeder_exit_status.txt"
    if not status_path.is_file() or status_path.is_symlink():
        raise GateError("EXIT_STATUS_MISSING", "feeder exit status is not persisted")
    status_text = status_path.read_text(encoding="utf-8").strip()
    manifest_path = out / "artifact_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if mode == "success":
        if run_rc != 0 or status_text != "0" or manifest.get("status") != "PASS":
            raise GateError("SUCCESS_ARTIFACT_INVALID", "success persistence contract failed")
        required = ["feeder_receipt.json", "feeder.log", "callback_consumer_evidence.json",
                    "consumer_evidence.json", "online_compute_timing.json",
                    "container_run_status.txt"]
        artifacts = {name: _artifact(out / name) for name in required}
        if (out / "failure_reason.json").exists():
            raise GateError("UNEXPECTED_FAILURE_REASON", "success emitted failure reason")
        timing = json.loads((out / "online_compute_timing.json").read_text())
        if timing.get("status") != "PASS" or float(timing.get("online_compute_rtf", -1)) < 0:
            raise GateError("TIMING_INVALID", "synthetic timing is not PASS")
        return {"status": "PASS", "feeder_exit_status": status_text,
                "manifest": _artifact(manifest_path), "artifacts": artifacts,
                "timing": timing}
    if run_rc == 0 or status_text != "17" or manifest.get("status") != "FAIL_CLOSED":
        raise GateError("FAILURE_ARTIFACT_INVALID", "failure persistence contract failed")
    reason_path = out / "failure_reason.json"
    reason = json.loads(reason_path.read_text())
    if reason.get("status") != "FAIL_CLOSED" or reason.get("reason") != "feeder failed":
        raise GateError("FAILURE_REASON_INVALID", "explicit feeder failure reason missing")
    if (out / "online_compute_timing.json").exists():
        raise GateError("FAILURE_TIMING_PRESENT", "failure must not emit timing")
    return {"status": "FAIL_CLOSED", "failure_kind": "SYNTHETIC_FEEDER_NONZERO",
            "feeder_exit_status": status_text, "reason": _artifact(reason_path),
            "manifest": _artifact(manifest_path), "missing": manifest.get("missing", [])}


def run_gate(root: Path, mode: str, *, repo_root: Path = ROOT,
             run_hook: Optional[Callable[[Sequence[str], Path], Any]] = None,
             inspect_hook: Optional[Callable[[str], Any]] = None,
             remove_hook: Optional[Callable[[str], Any]] = None) -> Dict[str, Any]:
    if mode not in {"success", "failure"}:
        raise GateError("MODE_INVALID", mode)
    root = root.absolute()
    if os.path.lexists(root) or not root.parent.is_dir() or root.parent.is_symlink():
        raise GateError("ROOT_NOT_FRESH", str(root))
    root.mkdir()
    (root / "out").mkdir()
    sources = _fake_sources(root, mode)
    name = f"m6a10-v22-synthetic-{mode}"
    argv = build_docker_argv(root, mode, name, sources["feeder"]["sha256"])
    if run_hook is None:
        completed = subprocess.run(argv, check=False, capture_output=True, text=True)
        run_rc = completed.returncode
        _seal_bytes(root / "docker_run.log", (completed.stdout + completed.stderr).encode())
    else:
        result = run_hook(argv, root)
        run_rc = int(getattr(result, "returncode", result))
        _seal_bytes(root / "docker_run.log", b"injected runner\n")
    if inspect_hook is None:
        inspected = subprocess.run(["docker", "inspect", name], check=False,
                                    capture_output=True, text=True)
        if inspected.returncode != 0:
            raise GateError("INSPECT_MISSING", inspected.stderr.strip())
        records = json.loads(inspected.stdout)
    else:
        records = inspect_hook(name)
    inspect_value = _inspect_shape(records, name, run_rc)
    _seal_json(root / "container.inspect.json", inspect_value)
    if remove_hook is None:
        removed = subprocess.run(["docker", "rm", name], check=False,
                                 capture_output=True, text=True)
        if removed.returncode != 0:
            raise GateError("CLEANUP_FAILED", removed.stderr.strip())
        post = subprocess.run(["docker", "inspect", name], check=False,
                              capture_output=True, text=True)
        if post.returncode == 0:
            raise GateError("CLEANUP_NOT_ABSENT", "container remained after remove")
    else:
        remove_hook(name)
    evidence = _verify_mode(root, mode, run_rc)
    receipt = {
        "schema_version": 1, "contract_version": CONTRACT,
        "status": evidence["status"], "mode": mode,
        "image": {"tag": IMAGE_TAG, "id": IMAGE_ID},
        "source": {"v22_wrapper_path": str(V22_WRAPPER),
                   "v22_wrapper_sha256": V22_WRAPPER_SHA256,
                   "synthetic_sources": sources},
        "build_receipt": {"path": str(BUILD_RECEIPT), "sha256": BUILD_RECEIPT_SHA256,
                          "sidecar_sha256": BUILD_SIDECAR_SHA256},
        "prior_no_input": {"path": str(NO_INPUT_RECEIPT), "sha256": NO_INPUT_RECEIPT_SHA256,
                           "sidecar_sha256": NO_INPUT_SIDECAR_SHA256},
        "execution": {"container_name": name, "start_count": 1, "retry": False,
                      "manual_stop": False, "run_exit_code": run_rc,
                      "network": "none", "rootfs": "read_only",
                      "rw_mount_destination": "/out", "input_mount_count": 0},
        "evidence": evidence,
        "safety": {"input_opened": False, "ground_truth_content_opened": False,
                   "scorer_invoked": False, "map_saved": False,
                   "formal_replay_started": False},
        "forbidden": {"bag": False, "ground_truth": False, "scorer": False,
                      "map": False, "formal": False},
        "argv_sha256": sha256_bytes("\0".join(argv).encode()),
    }
    receipt_path = root / "synthetic_gate.receipt.json"
    receipt_sha = _seal_json(receipt_path, receipt)
    sidecar_sha = _seal_bytes(receipt_path.with_name(receipt_path.name + ".sha256"),
                              (receipt_sha + "  " + receipt_path.name + "\n").encode("ascii"))
    return dict(receipt, receipt_path=str(receipt_path), receipt_sha256=receipt_sha,
                sidecar_sha256=sidecar_sha)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--mode", choices=("success", "failure"), required=True)
    args = parser.parse_args(argv)
    try:
        result = run_gate(args.root, args.mode)
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED",
                          "failure_kind": getattr(exc, "kind", "V22_GATE_FAILURE"),
                          "failure_message": str(exc)}, sort_keys=True))
        return 1
    print(json.dumps({key: result.get(key) for
                      key in ("status", "mode", "receipt_path", "receipt_sha256",
                              "sidecar_sha256")}, sort_keys=True))
    if args.mode == "success":
        return 0 if result["status"] == "PASS" else 1
    return 0 if result["status"] == "FAIL_CLOSED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
