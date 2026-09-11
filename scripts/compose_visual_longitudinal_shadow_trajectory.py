#!/usr/bin/env python3
"""Compose a deterministic output-only visual weak-axis trajectory.

The Voxel mapper trajectory and map remain immutable.  Accepted camera-derived
base-frame velocity vectors are projected against the timestamped weak axis of
the behavior-preserving runtime diagnostic, then integrated only in an output
shadow.  Strong and well-conditioned increments continue to come from the
baseline trajectory.  No reference or ground-truth trajectory is read.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml


@dataclass(frozen=True)
class Pose:
    stamp_text: str
    stamp: float
    position: np.ndarray
    quaternion: np.ndarray
    quaternion_text: tuple[str, str, str, str]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def quaternion_rotation(quaternion: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError('invalid quaternion')
    qx, qy, qz, qw = quaternion / norm
    return np.asarray([
        [1 - 2 * (qy * qy + qz * qz),
         2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),
         1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
         1 - 2 * (qx * qx + qy * qy)]], dtype=np.float64)


def load_tum(path: Path) -> list[Pose]:
    poses: list[Pose] = []
    previous = -math.inf
    for line_number, line in enumerate(
            path.read_text(encoding='utf-8', errors='replace').splitlines(), 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f'{path}:{line_number}: invalid TUM row')
        values = np.asarray([float(value) for value in fields[1:]],
                            dtype=np.float64)
        stamp = float(fields[0])
        if not math.isfinite(stamp) or not np.isfinite(values).all():
            raise ValueError(f'{path}:{line_number}: non-finite TUM value')
        if stamp <= previous:
            raise ValueError(f'{path}:{line_number}: timestamps not increasing')
        previous = stamp
        poses.append(Pose(
            fields[0], stamp, values[:3], values[3:], tuple(fields[4:8])))
    if not poses:
        raise ValueError(f'{path}: empty trajectory')
    return poses


def load_weak_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline='', encoding='utf-8', errors='replace') as stream:
        rows = list(csv.DictReader(stream))
    required = {
        't', 'evalue0', 'evalue1', 'weak_eigen_x', 'weak_eigen_y',
        'weak_eigen_z', 'weak_horizontal_norm', 'velocity_x', 'velocity_y',
        'velocity_z'}
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f'{path}: weak diagnostic schema is incomplete')
    return rows


def _nearest_index(values: list[float], target: float) -> int:
    insertion = bisect_left(values, target)
    candidates = [
        index for index in (insertion - 1, insertion)
        if 0 <= index < len(values)]
    if not candidates:
        raise ValueError('cannot join an empty timestamp sequence')
    return min(candidates, key=lambda index: abs(values[index] - target))


def weak_state(
        row: dict[str, str], gravity_axis: np.ndarray,
        config: dict[str, Any],
        ) -> tuple[bool, np.ndarray, float]:
    evalue0 = float(row['evalue0'])
    evalue1 = max(float(row['evalue1']), 1.0e-12)
    ratio = evalue0 / evalue1
    weak = np.asarray([
        float(row[f'weak_eigen_{axis}']) for axis in 'xyz'],
        dtype=np.float64)
    horizontal = weak - gravity_axis * float(gravity_axis @ weak)
    horizontal_length = float(np.linalg.norm(horizontal))
    if not np.isfinite(horizontal).all() or horizontal_length <= 1.0e-9:
        return False, np.zeros(3, dtype=np.float64), 0.0
    horizontal /= horizontal_length
    velocity = np.asarray([
        float(row[f'velocity_{axis}']) for axis in 'xyz'],
        dtype=np.float64)
    if not np.isfinite(velocity).all():
        return False, np.zeros(3, dtype=np.float64), 0.0
    if float(velocity @ horizontal) < 0.0:
        horizontal = -horizontal
    aligned_speed = float(velocity @ horizontal)
    weak_config = config['weak_axis']
    eligible = (
        ratio < float(weak_config['eigen_ratio_max']) and
        float(row['weak_horizontal_norm']) >=
        float(weak_config['horizontal_norm_min']) and
        abs(aligned_speed) >= float(weak_config['min_speed_mps']))
    return eligible, horizontal, aligned_speed


def validate_vector_report(document: dict[str, Any]) -> list[dict[str, Any]]:
    if document.get('accuracy_ground_truth_accessed') is not False:
        raise ValueError('vector report is not ground-truth-free')
    decision = document.get('decision')
    if decision == 'NO_GO_REPORT_ONLY_VECTOR_SHADOW_SOURCE':
        return []
    if decision != 'GO_REPORT_ONLY_VECTOR_SHADOW_SOURCE':
        raise ValueError(f'vector report did not pass: {decision}')
    contract = document.get('contract')
    if not isinstance(contract, dict) or (
            contract.get('velocity_frame') != 'base' or
            contract.get('axis_projection_at_consumer') is not True or
            contract.get('mapper_state_mutated') is not False):
        raise ValueError('vector report contract is incompatible')
    observations = document.get('observations')
    if not isinstance(observations, list):
        raise ValueError('vector report has no observations')
    previous = -math.inf
    validated: list[dict[str, Any]] = []
    max_speed = float(contract['max_speed_mps'])
    for observation in observations:
        stamp = float(observation['stamp_sec'])
        vector = np.asarray(observation['velocity_base_mps'], dtype=np.float64)
        confidence = float(observation['confidence'])
        if (not math.isfinite(stamp) or stamp < previous or
                vector.shape != (3,) or not np.isfinite(vector).all() or
                float(np.linalg.norm(vector)) > max_speed or
                not 0.0 <= confidence <= 1.0):
            raise ValueError('invalid vector observation')
        validated.append({
            'stamp_sec': stamp, 'velocity_base_mps': vector,
            'confidence': confidence})
        previous = stamp
    return validated


def trajectory_path_length(positions: list[np.ndarray]) -> float:
    return float(sum(np.linalg.norm(right - left)
                     for left, right in zip(positions, positions[1:])))


def compose_shadow_positions(
        poses: list[Pose], weak_rows: list[dict[str, str]],
        vector_document: dict[str, Any], config: dict[str, Any],
        ) -> tuple[list[np.ndarray], dict[str, Any]]:
    observations = validate_vector_report(vector_document)
    baseline_positions = [pose.position.copy() for pose in poses]
    if not observations:
        return baseline_positions, {
            'decision': 'NO_OP_OUTPUT_SHADOW', 'applied_observations': 0,
            'rejected_observations': 0, 'activation_stamp_sec': None,
            'max_shadow_speed_mps': 0.0}

    gravity_axis = np.asarray(
        vector_document.get('gravity_axis_world'), dtype=np.float64)
    gravity_norm = float(np.linalg.norm(gravity_axis))
    if gravity_axis.shape != (3,) or not math.isfinite(gravity_norm) or (
            gravity_norm <= 1.0e-9):
        raise ValueError('vector report has invalid gravity axis')
    gravity_axis /= gravity_norm

    weak_times = [float(row['t']) for row in weak_rows]
    if any(right <= left for left, right in zip(weak_times, weak_times[1:])):
        raise ValueError('weak diagnostic timestamps are not increasing')
    max_join = float(config['timing']['max_join_sec'])
    max_age = float(config['timing']['max_observation_age_sec'])
    max_dt = float(config['timing']['max_integration_dt_sec'])
    required_streak = int(config['weak_axis']['required_streak_scans'])
    gain = float(config['filter']['gain'])
    max_change = float(config['filter']['max_velocity_change_mps'])
    max_speed = float(config['filter']['max_speed_mps'])
    if (max_join <= 0.0 or max_age <= 0.0 or max_dt <= 0.0 or
            required_streak < 1 or not 0.0 <= gain <= 1.0 or
            max_change <= 0.0 or max_speed <= 0.0):
        raise ValueError('invalid output-shadow configuration')

    observations_by_pose: dict[int, list[dict[str, Any]]] = {}
    rejected_join = 0
    pose_times = [pose.stamp for pose in poses]
    for observation in observations:
        index = bisect_left(pose_times, observation['stamp_sec'])
        if index >= len(poses):
            rejected_join += 1
            continue
        age = pose_times[index] - observation['stamp_sec']
        if age < 0.0 or age > min(max_join, max_age):
            rejected_join += 1
            continue
        observations_by_pose.setdefault(index, []).append(observation)

    positions = [baseline_positions[0].copy()]
    shadow_position = baseline_positions[0].copy()
    shadow_speed: float | None = None
    weak_streak = 0
    applied = 0
    rejected_gate = 0
    activation_stamp: float | None = None
    max_shadow_speed = 0.0
    for index in range(1, len(poses)):
        pose = poses[index]
        weak_index = _nearest_index(weak_times, pose.stamp)
        weak_delta = abs(weak_times[weak_index] - pose.stamp)
        if weak_delta <= max_join:
            eligible, axis, baseline_speed = weak_state(
                weak_rows[weak_index], gravity_axis, config)
        else:
            eligible = False
            axis = np.zeros(3, dtype=np.float64)
            baseline_speed = 0.0
        weak_streak = weak_streak + 1 if eligible else 0
        gated = eligible and weak_streak >= required_streak

        for observation in observations_by_pose.get(index, []):
            if not gated:
                rejected_gate += 1
                continue
            world_velocity = quaternion_rotation(pose.quaternion) @ (
                observation['velocity_base_mps'])
            target_speed = float(axis @ world_velocity)
            if not math.isfinite(target_speed) or abs(target_speed) > max_speed:
                rejected_gate += 1
                continue
            if shadow_speed is None:
                shadow_speed = max(-max_speed, min(max_speed, baseline_speed))
            innovation = target_speed - shadow_speed
            correction = gain * observation['confidence'] * innovation
            correction = max(-max_change, min(max_change, correction))
            shadow_speed = max(
                -max_speed, min(max_speed, shadow_speed + correction))
            applied += 1
            max_shadow_speed = max(max_shadow_speed, abs(shadow_speed))
            if activation_stamp is None:
                activation_stamp = pose.stamp

        baseline_delta = baseline_positions[index] - baseline_positions[index - 1]
        if shadow_speed is not None and gated:
            dt = max(0.0, min(max_dt, pose.stamp - poses[index - 1].stamp))
            strong_delta = baseline_delta - axis * float(axis @ baseline_delta)
            shadow_position = (
                shadow_position + strong_delta + axis * shadow_speed * dt)
        else:
            shadow_position = shadow_position + baseline_delta
        positions.append(shadow_position.copy())

    return positions, {
        'decision': (
            'GO_OUTPUT_ONLY_SHADOW_SCREEN' if applied
            else 'NO_OP_OUTPUT_SHADOW'),
        'applied_observations': applied,
        'rejected_observations': rejected_join + rejected_gate,
        'rejected_join': rejected_join,
        'rejected_runtime_gate': rejected_gate,
        'activation_stamp_sec': activation_stamp,
        'max_shadow_speed_mps': max_shadow_speed,
    }


def write_tum(path: Path, poses: list[Pose], positions: list[np.ndarray]) -> None:
    if len(poses) != len(positions):
        raise ValueError('pose and position counts differ')
    lines = []
    for pose, position in zip(poses, positions):
        lines.append(
            f'{pose.stamp_text} '
            + ' '.join(f'{float(value):.12g}' for value in position)
            + ' ' + ' '.join(pose.quaternion_text) + '\n')
    path.write_text(''.join(lines), encoding='utf-8')


def translate_reference_positions(
        baseline_poses: list[Pose], reference_poses: list[Pose],
        shadow_positions: list[np.ndarray],
        ) -> list[np.ndarray]:
    """Apply the raw world-frame translation delta to a reference-point track."""
    if not (len(baseline_poses) == len(reference_poses) ==
            len(shadow_positions)):
        raise ValueError('raw and reference-point trajectory counts differ')
    translated: list[np.ndarray] = []
    for baseline, reference, shadow in zip(
            baseline_poses, reference_poses, shadow_positions):
        if abs(baseline.stamp - reference.stamp) > 1.0e-6:
            raise ValueError('raw and reference-point timestamps differ')
        translated.append(
            reference.position + (shadow - baseline.position))
    return translated


def build_report(
        baseline_path: Path, weak_path: Path, vector_path: Path,
        config_path: Path, output_path: Path, poses: list[Pose],
        positions: list[np.ndarray], runtime: dict[str, Any],
        baseline_reference_path: Path | None = None,
        output_reference_path: Path | None = None,
        ) -> dict[str, Any]:
    baseline_positions = [pose.position for pose in poses]
    position_deltas = [float(np.linalg.norm(candidate - baseline))
                       for candidate, baseline in zip(
                           positions, baseline_positions)]
    report = {
        'schema_version': 1,
        'status': 'ground_truth_free_output_only_visual_shadow',
        'accuracy_ground_truth_accessed': False,
        'decision': runtime['decision'],
        'contract': {
            'mapper_state_mutated': False,
            'map_state_mutated': False,
            'baseline_orientation_mutated': False,
            'observation_consumption': 'at_most_once',
            'axis_projection': 'timestamped_candidate_state',
        },
        'inputs': {
            'baseline_tum': str(baseline_path.resolve()),
            'baseline_sha256': sha256(baseline_path),
            'weak_csv': str(weak_path.resolve()),
            'weak_csv_sha256': sha256(weak_path),
            'vector_report': str(vector_path.resolve()),
            'vector_report_sha256': sha256(vector_path),
            'config': str(config_path.resolve()),
            'config_sha256': sha256(config_path),
        },
        'output': {
            'trajectory_tum': str(output_path.resolve()),
            'trajectory_sha256': sha256(output_path),
            'samples': len(poses),
            'baseline_path_length_m': trajectory_path_length(
                baseline_positions),
            'shadow_path_length_m': trajectory_path_length(positions),
            'max_position_delta_from_baseline_m': max(position_deltas),
            'final_position_delta_from_baseline_m': position_deltas[-1],
        },
        'runtime': runtime,
    }
    if baseline_reference_path is not None and output_reference_path is not None:
        report['inputs'].update({
            'baseline_reference_tum': str(baseline_reference_path.resolve()),
            'baseline_reference_sha256': sha256(baseline_reference_path),
        })
        report['output'].update({
            'reference_trajectory_tum': str(output_reference_path.resolve()),
            'reference_trajectory_sha256': sha256(output_reference_path),
        })
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-tum', required=True, type=Path)
    parser.add_argument('--weak-csv', required=True, type=Path)
    parser.add_argument('--vector-report', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output-tum', required=True, type=Path)
    parser.add_argument('--baseline-reference-tum', type=Path)
    parser.add_argument('--output-reference-tum', type=Path)
    parser.add_argument('--output-report', required=True, type=Path)
    args = parser.parse_args()
    if ((args.baseline_reference_tum is None) !=
            (args.output_reference_tum is None)):
        parser.error(
            '--baseline-reference-tum and --output-reference-tum are a pair')
    return args


def main() -> int:
    args = parse_args()
    outputs = [args.output_tum, args.output_report]
    if args.output_reference_tum is not None:
        outputs.append(args.output_reference_tum)
    for output in outputs:
        if output.exists():
            raise ValueError(f'refusing to overwrite: {output}')
    poses = load_tum(args.baseline_tum)
    reference_poses = (
        load_tum(args.baseline_reference_tum)
        if args.baseline_reference_tum is not None else None)
    vector_document = json.loads(args.vector_report.read_text(encoding='utf-8'))
    config = yaml.safe_load(args.config.read_text(encoding='utf-8'))
    observations = validate_vector_report(vector_document)
    if observations:
        weak_rows = load_weak_rows(args.weak_csv)
        positions, runtime = compose_shadow_positions(
            poses, weak_rows, vector_document, config)
    else:
        positions = [pose.position.copy() for pose in poses]
        runtime = {
            'decision': 'NO_OP_OUTPUT_SHADOW', 'applied_observations': 0,
            'rejected_observations': 0, 'activation_stamp_sec': None,
            'max_shadow_speed_mps': 0.0}
    args.output_tum.parent.mkdir(parents=True, exist_ok=True)
    if runtime['applied_observations'] == 0:
        args.output_tum.write_bytes(args.baseline_tum.read_bytes())
    else:
        write_tum(args.output_tum, poses, positions)
    if reference_poses is not None and args.output_reference_tum is not None:
        reference_positions = translate_reference_positions(
            poses, reference_poses, positions)
        args.output_reference_tum.parent.mkdir(parents=True, exist_ok=True)
        if runtime['applied_observations'] == 0:
            args.output_reference_tum.write_bytes(
                args.baseline_reference_tum.read_bytes())
        else:
            write_tum(
                args.output_reference_tum, reference_poses,
                reference_positions)
    report = build_report(
        args.baseline_tum, args.weak_csv, args.vector_report, args.config,
        args.output_tum, poses, positions, runtime,
        args.baseline_reference_tum, args.output_reference_tum)
    args.output_report.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(report, indent=2, sort_keys=True) + '\n'
    args.output_report.write_text(encoded, encoding='utf-8')
    print(encoded, end='')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, TypeError, KeyError,
            json.JSONDecodeError, yaml.YAMLError) as error:
        print(f'error: {error}', file=__import__('sys').stderr)
        raise SystemExit(2)
