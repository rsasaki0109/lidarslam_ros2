#!/usr/bin/env python3
"""Synthetic contract tests for the opt-in Phase 3d r3 candidate."""

from __future__ import annotations

import copy
import base64
import importlib.util
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "run_glim_clean_room_r3_candidate.py"
SPEC = importlib.util.spec_from_file_location("phase3d_r3_candidate", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _tree_hash(path: Path) -> str:
    return MODULE.sha256_tree(path)


def _fixture(tmp_path: Path) -> tuple[dict, dict, dict, Path, Path, Path, Path, str]:
    input_root = tmp_path / "input"
    calibration_root = tmp_path / "calibration"
    config_root = tmp_path / "config"
    for root, name in ((input_root, "bag.db3"),
                       (calibration_root, "calibration.json"),
                       (config_root, "config.json")):
        root.mkdir()
        (root / name).write_text("fixture-" + name + "\n", encoding="utf-8")
    profile_path = tmp_path / "profile.yaml"
    selection_path = tmp_path / "selection.yaml"
    selector = "competitive_slam_profile/m6a10_glim_v2b"
    profile = {
        "competitive_slam_profile": {
            "profile_revision_id": "fixture-profile-r3",
            "evidence_gate_v2": {},
            "m6a10_glim_v2b": {
                "input": {
                    "path": str(input_root),
                    "tree_sha256": _tree_hash(input_root),
                },
                "config": {
                    "path": str(config_root),
                    "tree_sha256": _tree_hash(config_root),
                },
            },
        }
    }
    profile_path.write_text(yaml.safe_dump(profile, sort_keys=True), encoding="utf-8")
    selection = {
        "selection_id": "fixture-selection-r2",
        "profile_path": str(profile_path),
        "profile_revision_id": "fixture-profile-r3",
        "closure_id": "fixture-closure-r2",
        "closure_revision": 2,
        "closure_identity_sha256": "f" * 64,
    }
    selection_path.write_text(yaml.safe_dump(selection, sort_keys=True), encoding="utf-8")
    manifest = copy.deepcopy(MODULE._manifest())
    frozen = manifest["frozen_r2_binding"]
    frozen.update({
        "profile_path": str(profile_path),
        "profile_canonical_sha256": MODULE.canonical_profile_sha256(profile),
        "profile_revision_id": "fixture-profile-r3",
        "selection_path": str(selection_path),
        "selection_id": "fixture-selection-r2",
        "selection_file_sha256": MODULE.sha256_file(selection_path),
        "closure_id": "fixture-closure-r2",
        "closure_identity_sha256": "f" * 64,
    })
    sequence = "fixture-sequence"
    run_index = 1
    input_manifest = tmp_path / "input-manifest.json"
    input_document = {
        "manifest_kind": manifest["input_contract"]["manifest_kind"],
        "canonical_rosbag2_tree_sha256": _tree_hash(input_root),
        "calibration_tree_sha256": _tree_hash(calibration_root),
        "calibration_root": str(calibration_root),
        "config_tree_sha256": _tree_hash(config_root),
        "profile_path": str(profile_path),
        "profile_selector": selector,
        "sequence_id": sequence,
        "run_index": run_index,
        "lidar_topic": "/points",
        "imu_topic": "/imu",
    }
    input_manifest.write_text(json.dumps(input_document, sort_keys=True), encoding="utf-8")
    identity = MODULE.validate_frozen_bindings(
        manifest=manifest, profile_path=profile_path,
        selection_path=selection_path, input_root=input_root,
        calibration_root=calibration_root, config_root=config_root,
        input_manifest_path=input_manifest, sequence_id=sequence,
        run_index=run_index)
    return (manifest, identity, input_document, input_root, calibration_root,
            config_root, input_manifest, sequence)


def test_candidate_manifest_is_opt_in_and_recipe_is_bridge_free():
    manifest = MODULE._manifest()
    assert manifest["status"] == "OPT_IN_NOT_READY"
    assert manifest["benchmark_eligible"] is False
    MODULE._validate_recipe_surface()


def test_frozen_bindings_and_command_are_exact(tmp_path):
    (manifest, identity, _document, input_root, calibration_root, config_root,
     _input_manifest, _) = _fixture(tmp_path)
    argv, mounts = MODULE.build_container_argv(
        manifest=manifest, input_root=input_root,
        calibration_root=calibration_root, config_root=config_root,
        output_root=tmp_path / "output", identity=identity)
    assert argv[:8] == [
        "docker", "run", "--rm", "--init", "--network", "none",
        "--read-only", "--cap-drop=ALL"]
    assert "--privileged" not in argv
    assert "none" in argv
    assert len(mounts) == 4
    assert [item["read_only"] for item in mounts] == [True, True, True, False]


def test_input_path_or_calibration_mutation_fails_closed(tmp_path):
    (manifest, _identity, _document, input_root, calibration_root, config_root,
     input_manifest, sequence) = _fixture(tmp_path)
    with pytest.raises(MODULE.CandidateError, match="input/config path"):
        MODULE.validate_frozen_bindings(
            manifest=manifest, profile_path=Path(manifest["frozen_r2_binding"]["profile_path"]),
            selection_path=Path(manifest["frozen_r2_binding"]["selection_path"]),
            input_root=tmp_path / "wrong-input", calibration_root=calibration_root,
            config_root=config_root, input_manifest_path=input_manifest,
            sequence_id=sequence, run_index=1)
    calibration_root.joinpath("calibration.json").write_text("mutated\n", encoding="utf-8")
    with pytest.raises(MODULE.CandidateError, match="calibration identity"):
        MODULE.validate_frozen_bindings(
            manifest=manifest, profile_path=Path(manifest["frozen_r2_binding"]["profile_path"]),
            selection_path=Path(manifest["frozen_r2_binding"]["selection_path"]),
            input_root=input_root, calibration_root=calibration_root,
            config_root=config_root, input_manifest_path=input_manifest,
            sequence_id=sequence, run_index=1)


def _write_success(root: Path, frame: str = "world") -> None:
    root.mkdir()
    (root / "trajectory.json").write_text(json.dumps({
        "schema_version": 1,
        "kind": "glim_clean_room_phase3d_trajectory_v1",
        "frame_id": frame,
        "samples": [{
            "order": 0, "stamp": {"sec": 1, "nanosec": 0},
            "frame_id": frame,
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }],
    }, sort_keys=True), encoding="utf-8")
    (root / "map.json").write_text(json.dumps({
        "schema_version": 1,
        "kind": "glim_clean_room_phase3d_map_v1",
        "frame_id": frame,
        "stamp": {"sec": 1, "nanosec": 0},
        "width": 1, "height": 1, "point_step": 1, "row_step": 1,
        "is_bigendian": False, "is_dense": True, "fields": [],
        "data_base64": base64.b64encode(b"x").decode("ascii"),
    }, sort_keys=True), encoding="utf-8")
    (root / "resource.json").write_text(json.dumps({
        "schema_version": 1,
        "kind": "glim_clean_room_phase3d_resource_v1",
        "measurement_tool": "fixture",
        "measurement_revision": "phase3d-r3-resource-v1",
        "peak_rss_bytes": 1,
        "network_used": False,
    }, sort_keys=True), encoding="utf-8")


def test_success_artifacts_and_receipt_are_sealed(tmp_path):
    (manifest, identity, _document, input_root, calibration_root, config_root,
     _input_manifest, _) = _fixture(tmp_path)
    output = tmp_path / "output"
    _write_success(output)
    argv, mounts = MODULE.build_container_argv(
        manifest=manifest, input_root=input_root,
        calibration_root=calibration_root, config_root=config_root,
        output_root=output, identity=identity)
    receipt = MODULE.seal_attempt(
        output_root=output, manifest=manifest, identity=identity,
        argv=argv, mounts=mounts, exit_status=0, timed_out=False)
    assert receipt["completion"]["complete"] is True
    assert (output / "attempt.json").is_file()
    index = json.loads((output / "attempt.index.json").read_text(encoding="utf-8"))
    assert index["receipt_sha256"] == receipt["execution_receipt_sha256"]
    assert index["receipt_file_sha256"] == MODULE.sha256_file(output / "attempt.json")


def test_frame_drift_and_extra_artifact_are_rejected(tmp_path):
    (_manifest_value, _identity, _document, _input, _calibration, _config,
     _receipt, _sequence) = _fixture(tmp_path)
    output = tmp_path / "output"
    _write_success(output, frame="map")
    trajectory = json.loads((output / "trajectory.json").read_text(encoding="utf-8"))
    trajectory["samples"][0]["frame_id"] = "world"
    (output / "trajectory.json").write_text(json.dumps(trajectory), encoding="utf-8")
    with pytest.raises(MODULE.CandidateError):
        MODULE._validate_success_artifacts(output)
    (output / "trajectory.json").write_text(json.dumps({
        "kind": "glim_clean_room_phase3d_trajectory_v1", "frame_id": "map",
        "samples": [{"order": 0, "stamp": {"sec": 1, "nanosec": 0},
                     "frame_id": "map", "position": {"x": 0, "y": 0, "z": 0},
                     "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}]},
        sort_keys=True), encoding="utf-8")
    (output / "unexpected.bin").write_bytes(b"x")
    with pytest.raises(MODULE.CandidateError, match="exact"):
        MODULE._validate_success_artifacts(output)


def test_failure_is_terminal_and_cannot_mix_success_outputs(tmp_path):
    (manifest, identity, _document, input_root, calibration_root, config_root,
     _input_manifest, _) = _fixture(tmp_path)
    output = tmp_path / "failure"
    output.mkdir()
    (output / "failure.json").write_text(json.dumps({
        "schema_version": 1,
        "kind": "glim_clean_room_phase3d_failure_v1",
        "terminal": True,
        "stage": "fixture",
        "gt_blind": {"ground_truth_content_opened": False, "scorer_invoked": False},
    }), encoding="utf-8")
    argv, mounts = MODULE.build_container_argv(
        manifest=manifest, input_root=input_root,
        calibration_root=calibration_root, config_root=config_root,
        output_root=output, identity=identity)
    receipt = MODULE.seal_attempt(
        output_root=output, manifest=manifest, identity=identity,
        argv=argv, mounts=mounts, exit_status=1, timed_out=False)
    assert receipt["completion"]["complete"] is False
    assert not (output / "trajectory.json").exists()
    (output / "resource.json").write_text("{}", encoding="utf-8")
    with pytest.raises(MODULE.CandidateError):
        MODULE._validate_failure_artifact(output)
