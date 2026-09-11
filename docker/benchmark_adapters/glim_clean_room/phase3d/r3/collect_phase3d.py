#!/usr/bin/env python3
"""Collect the ROS 2 Phase 3d outputs into one atomic candidate batch.

The collector is intentionally a small transport consumer, not a scorer.  It
does not open ground truth or select a result.  A successful batch is written
only after the node's completion diagnostic follows both publication calls;
otherwise exactly one terminal ``failure.json`` is sealed.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
from pathlib import Path
import resource
import subprocess
import time
from typing import Any


TIMEOUT_SECONDS = 1800


class CollectionError(RuntimeError):
    """A terminal collector failure."""


def _json_atomic(path: Path, value: Any) -> None:
    if path.exists() or path.is_symlink():
        raise CollectionError(f"output already exists: {path.name}")
    part = path.with_name(path.name + ".part")
    if part.exists() or part.is_symlink():
        raise CollectionError(f"output part already exists: {path.name}.part")
    part.write_text(json.dumps(value, sort_keys=True, indent=2) + "\n",
                    encoding="utf-8")
    os.replace(part, path)


def _stamp(message: Any) -> dict[str, int]:
    value = message.header.stamp
    return {"sec": int(value.sec), "nanosec": int(value.nanosec)}


def _finite(value: Any, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise CollectionError(f"non-finite {label}")
    return result


def _path_document(message: Any) -> dict[str, Any]:
    frame = str(message.header.frame_id)
    if not frame:
        raise CollectionError("trajectory frame is empty")
    poses = []
    previous = None
    for order, item in enumerate(message.poses):
        stamp = _stamp(item)
        key = (stamp["sec"], stamp["nanosec"])
        if previous is not None and key <= previous:
            raise CollectionError("trajectory stamps are not strictly increasing")
        previous = key
        pose = item.pose
        poses.append({
            "order": order,
            "stamp": stamp,
            "frame_id": str(item.header.frame_id),
            "position": {
                "x": _finite(pose.position.x, "position.x"),
                "y": _finite(pose.position.y, "position.y"),
                "z": _finite(pose.position.z, "position.z"),
            },
            "orientation": {
                "x": _finite(pose.orientation.x, "orientation.x"),
                "y": _finite(pose.orientation.y, "orientation.y"),
                "z": _finite(pose.orientation.z, "orientation.z"),
                "w": _finite(pose.orientation.w, "orientation.w"),
            },
        })
    if not poses:
        raise CollectionError("trajectory is empty")
    if any(item["frame_id"] != frame for item in poses):
        raise CollectionError("trajectory frame changes within one output")
    return {
        "schema_version": 1,
        "kind": "glim_clean_room_phase3d_trajectory_v1",
        "frame_id": frame,
        "samples": poses,
    }


def _map_document(message: Any) -> dict[str, Any]:
    frame = str(message.header.frame_id)
    if not frame or int(message.point_step) <= 0 or int(message.width) <= 0:
        raise CollectionError("map layout is empty or invalid")
    fields = []
    for field in message.fields:
        name = str(field.name)
        if not name or int(field.count) <= 0:
            raise CollectionError("map field is invalid")
        fields.append({
            "name": name,
            "offset": int(field.offset),
            "datatype": int(field.datatype),
            "count": int(field.count),
        })
    payload = bytes(message.data)
    if len(payload) != int(message.row_step) * int(message.height):
        raise CollectionError("map payload and row_step disagree")
    return {
        "schema_version": 1,
        "kind": "glim_clean_room_phase3d_map_v1",
        "frame_id": frame,
        "stamp": _stamp(message),
        "width": int(message.width),
        "height": int(message.height),
        "point_step": int(message.point_step),
        "row_step": int(message.row_step),
        "is_bigendian": bool(message.is_bigendian),
        "is_dense": bool(message.is_dense),
        "fields": fields,
        "data_base64": base64.b64encode(payload).decode("ascii"),
    }


def _resource_document(started: float, child_status: int) -> dict[str, Any]:
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    return {
        "schema_version": 1,
        "kind": "glim_clean_room_phase3d_resource_v1",
        "measurement_tool": "python-resource-getrusage",
        "measurement_revision": "phase3d-r3-resource-v1",
        "wall_time_ns": int((time.monotonic() - started) * 1_000_000_000),
        "user_time_ns": int(usage.ru_utime * 1_000_000_000),
        "system_time_ns": int(usage.ru_stime * 1_000_000_000),
        "peak_rss_bytes": int(usage.ru_maxrss) * 1024,
        "child_exit_status": int(child_status),
        "oom_killed": False,
        "network_used": False,
    }


def _failure_document(stage: str, error: BaseException) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "kind": "glim_clean_room_phase3d_failure_v1",
        "terminal": True,
        "stage": stage,
        "error_type": type(error).__name__,
        "error": str(error),
        "gt_blind": {
            "ground_truth_content_opened": False,
            "scorer_invoked": False,
        },
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--calibration-root", type=Path, required=True)
    parser.add_argument("--config-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--lidar-topic", required=True)
    parser.add_argument("--imu-topic", required=True)
    return parser


def _require_directory(path: Path, label: str) -> None:
    if path.is_symlink() or not path.is_dir():
        raise CollectionError(f"{label} is not a regular directory")


def run(args: argparse.Namespace) -> int:
    for path, label in ((args.input_root, "input"),
                        (args.calibration_root, "calibration"),
                        (args.config_root, "config")):
        _require_directory(path, label)
    if args.output_root.exists() or args.output_root.is_symlink():
        raise CollectionError("output root must be fresh")
    args.output_root.mkdir(parents=True)
    started = time.monotonic()
    node = None
    bag = None
    collector = None
    try:
        # ROS imports are intentionally deferred so host-side source audits and
        # schema tests do not require a ROS installation.
        import rclpy  # type: ignore
        from diagnostic_msgs.msg import DiagnosticArray  # type: ignore
        from nav_msgs.msg import Path as PathMessage  # type: ignore
        from rclpy.node import Node  # type: ignore
        from rclpy.qos import QoSProfile, ReliabilityPolicy  # type: ignore
        from sensor_msgs.msg import PointCloud2  # type: ignore
        from std_srvs.srv import Trigger  # type: ignore

        rclpy.init()

        class CollectorNode(Node):
            def __init__(self) -> None:
                super().__init__("glim_clean_room_phase3d_collector")
                reliable = QoSProfile(depth=10)
                reliable.reliability = ReliabilityPolicy.RELIABLE
                self.path_message = None
                self.map_message = None
                self.diagnostic_message = None
                self.finalize_future = None
                self.finalize_sent = False
                self.finished = False
                self.failure: BaseException | None = None
                self.create_subscription(PathMessage, "/glim_clean_room/trajectory",
                                         self.on_path, reliable)
                self.create_subscription(PointCloud2, "/glim_clean_room/map",
                                         self.on_map, reliable)
                self.create_subscription(DiagnosticArray, "/glim_clean_room/diagnostics",
                                         self.on_diagnostic, reliable)
                self.finalize_client = self.create_client(
                    Trigger, "/glim_clean_room/finalize")
                self.create_timer(0.05, self.poll)

            def on_path(self, message: Any) -> None:
                if self.path_message is None:
                    self.path_message = message

            def on_map(self, message: Any) -> None:
                if self.map_message is None:
                    self.map_message = message

            def on_diagnostic(self, message: Any) -> None:
                self.diagnostic_message = message

            def poll(self) -> None:
                if self.finished:
                    return
                if time.monotonic() - started > TIMEOUT_SECONDS:
                    self.failure = CollectionError("bounded collector timeout")
                    self.finished = True
                    return
                if bag is None or bag.poll() is None:
                    return
                if bag.returncode != 0:
                    self.failure = CollectionError(
                        f"ros2 bag play exited with {bag.returncode}")
                    self.finished = True
                    return
                if not self.finalize_sent:
                    if not self.finalize_client.service_is_ready():
                        return
                    self.finalize_future = self.finalize_client.call_async(
                        Trigger.Request())
                    self.finalize_sent = True
                    return
                if self.finalize_future is None or not self.finalize_future.done():
                    return
                response = self.finalize_future.result()
                if response is None or not response.success:
                    self.failure = CollectionError(
                        "phase3d finalize service reported failure")
                    self.finished = True
                    return
                if self.path_message is None or self.map_message is None:
                    self.failure = CollectionError(
                        "completion arrived without both required outputs")
                elif self.diagnostic_message is None:
                    self.failure = CollectionError(
                        "completion diagnostic is missing")
                self.finished = True

        collector = CollectorNode()
        node_cmd = [
            "ros2", "run", "glim_clean_room_phase3d",
            "glim_clean_room_phase3d_node_main", "--ros-args",
            "-p", f"config_directory:={args.config_root}",
            "-p", f"lidar_topic:={args.lidar_topic}",
            "-p", f"imu_topic:={args.imu_topic}",
        ]
        node = subprocess.Popen(node_cmd)
        bag = subprocess.Popen(["ros2", "bag", "play", str(args.input_root), "--clock"])
        while not collector.finished:
            rclpy.spin_once(collector, timeout_sec=0.1)
        if collector.failure is not None:
            raise collector.failure
        trajectory = _path_document(collector.path_message)
        mapping = _map_document(collector.map_message)
        _json_atomic(args.output_root / "trajectory.json", trajectory)
        _json_atomic(args.output_root / "map.json", mapping)
        _json_atomic(args.output_root / "resource.json",
                     _resource_document(started, bag.returncode or 0))
        return 0
    except BaseException as error:
        if args.output_root.is_dir() and not (args.output_root / "failure.json").exists():
            try:
                _json_atomic(args.output_root / "failure.json",
                             _failure_document("collection", error))
            except BaseException:
                pass
        return 1
    finally:
        for process in (bag, node):
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
        if collector is not None:
            try:
                collector.destroy_node()
            except BaseException:
                pass
        try:
            import rclpy  # type: ignore
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException:
            pass


def main() -> int:
    args = _parser().parse_args()
    try:
        return run(args)
    except BaseException as error:
        if args.output_root.exists() and args.output_root.is_dir() and \
                not (args.output_root / "failure.json").exists():
            _json_atomic(args.output_root / "failure.json",
                         _failure_document("preflight", error))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
