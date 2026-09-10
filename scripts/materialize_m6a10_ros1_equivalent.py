#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Create and verify the pinned ROS1 equivalent of a fixed M6a10 ROS2 bag.

The ROS2 synchronized-tail materialization is the only input selection source.
This command validates that fixed10 receipt and the current profile still bind
the same materializer, converter, and semantic comparator before invoking a
subprocess.  Conversion is performed into
``<final>.staging/<final-basename>.bag``; semantic equivalence is checked while
the output is still staged, and only then is the file atomically published.
The receipt is external to both bag paths and never contains its own hash.
Ground truth and scoring are deliberately out of scope.
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
from typing import Any, Callable, Mapping, Sequence

try:
    import yaml
except ImportError:  # pragma: no cover - repository runtime dependency
    yaml = None  # type: ignore[assignment]


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from lidarslam_benchmark_tools.materialize_m6a10_synchronized_tail import (  # noqa: E402
    MaterializationError,
    sha256_file,
    sha256_tree,
)


SCHEMA_VERSION = 1
RECEIPT_KIND = 'm6a10_v2a_ros1_equivalence_generation'
CONTRACT_ID = 'm6a10-v2a-ros1-equivalence-v1'
MATERIALIZATION_CONTRACT_ID = 'm6a10-v2a-synchronized-tail-materialization-v1'
ANALYZER_CONTRACT_ID = 'm6a10-v2a-synchronized-tail-v1'
CONVERTER = 'rosbags-convert'
CONVERTER_VERSION = '0.11.0'
CONVERTER_PATH = Path('/home/sasaki/.local/bin/rosbags-convert')
COMPARATOR_PATH = SCRIPTS / 'compare_rosbag_semantic_inputs.py'
COMPARATOR_RELATIVE_PATH = 'scripts/compare_rosbag_semantic_inputs.py'
GENERATOR_RELATIVE_PATH = 'scripts/materialize_m6a10_synchronized_tail.py'
GENERATOR_PATH = ROOT / GENERATOR_RELATIVE_PATH
ORDER_DEFINITION = 'anyreader_chronological_playback_order_including_deterministic_ties'
EXPECTED_TOPICS = (
    '/os1_cloud_node1/points', '/imu/imu', '/left/image_raw')
EXPECTED_TOPIC_TYPES = {
    '/os1_cloud_node1/points': 'sensor_msgs/msg/PointCloud2',
    '/imu/imu': 'sensor_msgs/msg/Imu',
    '/left/image_raw': 'sensor_msgs/msg/Image',
}
EXPECTED_TOPIC_COUNTS = {
    '/imu/imu': 225102,
    '/left/image_raw': 5792,
    '/os1_cloud_node1/points': 5793,
}
EXPECTED_RECORD_COUNT = 236687
SHA256_RE = re.compile(r'^[0-9a-f]{64}$')
PROFILE_CONVERSION_ARGV = [
    'rosbags-convert',
    '--src', '<canonical_ros2>',
    '--dst', '<ros1_staging_container>/<ros1_final_basename>.bag',
    '--compress', 'none',
    '--src-typestore', 'ros2_humble',
    '--dst-typestore', 'ros1_noetic',
]


class Ros1EquivalenceError(MaterializationError):
    """Raised when ROS1 conversion cannot satisfy the fixed identity contract."""


CommandRunner = Callable[[Sequence[str]], subprocess.CompletedProcess[str]]


def _require_sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA256_RE.fullmatch(value) is None:
        raise Ros1EquivalenceError('invalid_sha256', f'{label} must be a 64-hex SHA-256')
    return value.lower()


def _regular(path: Path, label: str, *, directory: bool | None = None) -> Path:
    path = Path(path).expanduser()
    if path.is_symlink() or not path.exists():
        raise Ros1EquivalenceError('unsafe_path', f'{label} is missing or a symlink: {path}')
    resolved = path.resolve(strict=True)
    if directory is True and not resolved.is_dir():
        raise Ros1EquivalenceError('unsafe_path', f'{label} is not a directory: {path}')
    if directory is False and not resolved.is_file():
        raise Ros1EquivalenceError('unsafe_path', f'{label} is not a regular file: {path}')
    if directory is False and label == 'converter' and not os.access(resolved, os.X_OK):
        raise Ros1EquivalenceError('unsafe_path', f'converter is not executable: {path}')
    return resolved


def _resolve_new(path: Path, label: str) -> Path:
    path = Path(path).expanduser().resolve(strict=False)
    if path.exists() or path.is_symlink():
        raise Ros1EquivalenceError('path_exists', f'{label} already exists: {path}')
    return path


def _under(path: Path, root: Path) -> bool:
    try:
        path.resolve(strict=False).relative_to(root.resolve(strict=False))
    except ValueError:
        return False
    return True


def _assert_no_symlink_ancestors(path: Path, label: str) -> None:
    """Reject an output path whose existing parent chain contains a symlink.

    This check deliberately runs before creating any output parent.  In
    particular, an overlapping output path must not cause us to create a
    directory inside the read-only input tree before the overlap rejection.
    """
    current = path.parent
    while current != current.parent:
        if current.is_symlink():
            raise Ros1EquivalenceError(
                'unsafe_path', f'{label} has a symlink ancestor: {current}')
        current = current.parent


def _assert_not_overlapping(paths: Mapping[str, Path], roots: Mapping[str, Path]) -> None:
    for name, path in paths.items():
        for root_name, root in roots.items():
            if _under(path, root) or _under(root, path):
                raise Ros1EquivalenceError(
                    'path_overlap', f'{name} overlaps {root_name}: {path} / {root}')


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise Ros1EquivalenceError('json_invalid', f'{label} is not valid JSON') from exc
    if not isinstance(value, dict):
        raise Ros1EquivalenceError('json_invalid', f'{label} must be an object')
    return value


def _load_yaml(path: Path, label: str) -> dict[str, Any]:
    if yaml is None:
        raise Ros1EquivalenceError('yaml_missing', 'PyYAML is required for profile validation')
    try:
        value = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, ValueError) as exc:
        raise Ros1EquivalenceError('yaml_invalid', f'{label} is not valid YAML') from exc
    if not isinstance(value, dict):
        raise Ros1EquivalenceError('yaml_invalid', f'{label} must be an object')
    return value


def _profile_contract(profile: dict[str, Any]) -> dict[str, Any]:
    try:
        return profile['competitive_slam_profile']['runtime_policy'][
            'phase_contract_v2']['synchronized_tail_materialization']
    except (KeyError, TypeError) as exc:
        raise Ros1EquivalenceError(
            'profile_contract_missing',
            'current profile materialization contract is missing',
        ) from exc


def _repo_path(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise Ros1EquivalenceError('path_missing', f'{label} path is missing')
    path = Path(value)
    if not path.is_absolute():
        path = ROOT / path
    return path


def _converter_help_sha256(converter_path: Path) -> str:
    try:
        result = subprocess.run(
            [str(converter_path), '--help'],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=False,
            text=False,
        )
    except OSError as exc:
        raise Ros1EquivalenceError('converter_help', 'converter --help could not execute') from exc
    if result.returncode != 0 or result.stderr:
        raise Ros1EquivalenceError('converter_help', 'converter --help did not pass cleanly')
    return hashlib.sha256(result.stdout).hexdigest()


def _fixed10_counts(expected_topic_counts: Mapping[str, int] | None) -> dict[str, int]:
    counts = dict(expected_topic_counts or EXPECTED_TOPIC_COUNTS)
    if tuple(sorted(counts)) != tuple(sorted(EXPECTED_TOPICS)):
        raise Ros1EquivalenceError('topic_contract', 'exact NTU topic set is required')
    if any(not isinstance(value, int) or value <= 0 for value in counts.values()):
        raise Ros1EquivalenceError('topic_contract', 'topic counts must be positive integers')
    return counts


def validate_profile_and_tools(
    profile_path: Path,
    fixed_receipt_path: Path,
    final_path: Path,
    input_root: Path,
    *,
    expected_topic_counts: Mapping[str, int] | None = None,
) -> dict[str, Any]:
    """Validate current profile, fixed10 receipt, and executable identities."""
    profile_path = _regular(profile_path, 'profile', directory=False)
    fixed_receipt_path = _regular(fixed_receipt_path, 'fixed10 receipt', directory=False)
    profile = _load_yaml(profile_path, 'profile')
    contract = _profile_contract(profile)
    expected_counts = _fixed10_counts(expected_topic_counts)
    generated = contract.get('generated_receipt')
    output_identity = contract.get('generated_output')
    ros1 = contract.get('ros1_fast_livo2')
    if not isinstance(generated, dict) or not isinstance(output_identity, dict):
        raise Ros1EquivalenceError(
            'profile_contract_missing', 'generated materialization identity missing')
    if not isinstance(ros1, dict):
        raise Ros1EquivalenceError('profile_contract_missing', 'ROS1 converter identity missing')
    if Path(str(generated.get('path', ''))).expanduser().resolve(strict=False) \
            != fixed_receipt_path:
        raise Ros1EquivalenceError(
            'profile_receipt_path', 'fixed receipt differs from current profile')
    receipt_sha = _require_sha(generated.get('sha256'), 'profile generated receipt sha256')
    if sha256_file(fixed_receipt_path) != receipt_sha:
        raise Ros1EquivalenceError(
            'profile_receipt_sha', 'fixed receipt hash differs from profile')
    configured_output = _repo_path(
        output_identity.get('path'), 'profile ROS2 output').resolve(strict=False)
    input_root = Path(input_root).expanduser().resolve(strict=True)
    if configured_output != input_root:
        raise Ros1EquivalenceError(
            'profile_input_path', 'input ROS2 path differs from profile generated output')
    configured_ros1 = _repo_path(
        ros1.get('ros1_output_path'), 'profile ROS1 output').resolve(strict=False)
    if configured_ros1 != final_path:
        raise Ros1EquivalenceError('profile_ros1_path', 'final ROS1 path differs from profile')
    if contract.get('contract_id') != MATERIALIZATION_CONTRACT_ID:
        raise Ros1EquivalenceError('profile_contract', 'materialization contract ID is not pinned')
    if contract.get('status') != 'ros2_materialized_verified_ros1_pending':
        raise Ros1EquivalenceError(
            'profile_status', 'profile is not at ROS1-pending materialization status')
    if (ros1.get('conversion_executed_here') is not False
            or ros1.get('status') != 'ros1_pending'):
        raise Ros1EquivalenceError(
            'profile_ros1_status', 'profile must keep ROS1 conversion pending')
    if (ros1.get('converter') != CONVERTER
            or str(ros1.get('converter_version')) != CONVERTER_VERSION):
        raise Ros1EquivalenceError(
            'converter_identity', 'converter name/version differs from profile')
    converter_path = _regular(
        _repo_path(ros1.get('converter_executable_path'), 'converter'),
        'converter', directory=False)
    converter_sha = _require_sha(ros1.get('converter_executable_sha256'), 'converter sha256')
    if sha256_file(converter_path) != converter_sha:
        raise Ros1EquivalenceError(
            'converter_identity', 'converter executable hash differs from profile')
    help_sha = _require_sha(ros1.get('converter_help_sha256'), 'converter help sha256')
    observed_help_sha = _converter_help_sha256(converter_path)
    if observed_help_sha != help_sha:
        raise Ros1EquivalenceError('converter_help', 'converter help hash differs from profile')
    comparator_path = _regular(
        _repo_path(ros1.get('semantic_comparator', {}).get('path'), 'semantic comparator'),
        'semantic comparator', directory=False)
    comparator_sha = _require_sha(
        ros1.get('semantic_comparator', {}).get('sha256'),
        'semantic comparator sha256')
    if sha256_file(comparator_path) != comparator_sha \
            or comparator_path != COMPARATOR_PATH:
        raise Ros1EquivalenceError(
            'comparator_identity',
            'semantic comparator hash/path differs from current file')
    if ros1.get('semantic_comparator', {}).get('topics') != list(EXPECTED_TOPICS):
        raise Ros1EquivalenceError(
            'topic_contract', 'profile semantic topics are not exact NTU topics')
    semantic_contract = ros1.get('semantic_comparator', {})
    if semantic_contract.get('all_topics_equal_required') is not True \
            or semantic_contract.get('report_required_before_fast_run') is not True:
        raise Ros1EquivalenceError(
            'semantic_contract', 'profile semantic comparator requirements are not pinned')
    if ros1.get('conversion_argv') != PROFILE_CONVERSION_ARGV:
        raise Ros1EquivalenceError(
            'conversion_contract', 'profile conversion argv is not the pinned command')
    if ros1.get('destination_extension') != '.bag' or ros1.get(
            'staging_policy') != 'sibling_container_exact_final_basename':
        raise Ros1EquivalenceError('staging_contract', 'ROS1 .bag staging policy is not pinned')
    expected_stage_container = configured_ros1.with_suffix('.staging')
    expected_stage_output = expected_stage_container / configured_ros1.name
    configured_stage = _repo_path(
        ros1.get('ros1_staging_container_path'),
        'profile ROS1 staging container').resolve(strict=False)
    configured_stage_output = _repo_path(
        ros1.get('ros1_staging_output_path'),
        'profile ROS1 staging output').resolve(strict=False)
    if configured_stage != expected_stage_container \
            or configured_stage_output != expected_stage_output:
        raise Ros1EquivalenceError(
            'staging_contract', 'profile ROS1 staging paths do not match final basename')

    receipt = _load_json(fixed_receipt_path, 'fixed10 materialization receipt')
    if receipt.get('schema_version') != 1 or receipt.get('receipt_kind') != (
            'm6a10_v2a_synchronized_tail_generation'):
        raise Ros1EquivalenceError(
            'fixed_receipt_schema', 'fixed10 receipt kind/schema is unsupported')
    if (receipt.get('status') != 'PASS'
            or receipt.get('contract_id') != MATERIALIZATION_CONTRACT_ID):
        raise Ros1EquivalenceError(
            'fixed_receipt_status', 'fixed10 receipt is not a PASS materialization')
    if Path(str(receipt.get('receipt_path', ''))).expanduser().resolve(strict=False) \
            != fixed_receipt_path:
        raise Ros1EquivalenceError(
            'fixed_receipt_path', 'fixed10 receipt path does not identify its file')
    analyzer = receipt.get('analyzer')
    generator = receipt.get('generator')
    output = receipt.get('output')
    verification = receipt.get('verification')
    safety = receipt.get('safety')
    if not isinstance(analyzer, dict) or analyzer.get('contract_id') != ANALYZER_CONTRACT_ID:
        raise Ros1EquivalenceError('fixed_receipt_schema', 'nested analyzer contract is missing')
    if not isinstance(generator, dict) or generator.get('path') != GENERATOR_RELATIVE_PATH:
        raise Ros1EquivalenceError('generator_identity', 'fixed generator path is not pinned')
    generator_sha = _require_sha(generator.get('sha256'), 'fixed generator sha256')
    actual_generator_sha = sha256_file(GENERATOR_PATH)
    if generator_sha != actual_generator_sha or generator_sha != contract['producer']['sha256']:
        raise Ros1EquivalenceError(
            'generator_identity', 'fixed generator hash differs from current/profile')
    if (not isinstance(output, dict) or not isinstance(verification, dict)
            or not isinstance(safety, dict)):
        raise Ros1EquivalenceError(
            'fixed_receipt_schema',
            'fixed output/verification/safety blocks are missing')
    if output.get('path') != str(configured_output):
        raise Ros1EquivalenceError(
            'fixed_receipt_output', 'fixed output path differs from profile')
    if output.get('record_count') != sum(expected_counts.values()):
        raise Ros1EquivalenceError(
            'fixed_receipt_output',
            'fixed record count differs from exact topic counts')
    per_topic = output.get('per_topic')
    if not isinstance(per_topic, dict) or set(per_topic) != set(expected_counts):
        raise Ros1EquivalenceError('fixed_receipt_output', 'fixed per-topic identity is invalid')
    for topic, count in expected_counts.items():
        if per_topic.get(topic, {}).get('count') != count:
            raise Ros1EquivalenceError('fixed_receipt_output', f'fixed count mismatch for {topic}')
    if verification.get('status') != 'PASS' or verification.get(
            'connection_metadata_equal') is not True:
        raise Ros1EquivalenceError(
            'fixed_receipt_verification',
            'fixed independent verification is not PASS')
    if receipt.get('order_definition') != ORDER_DEFINITION:
        raise Ros1EquivalenceError('fixed_receipt_order', 'fixed order definition differs')
    if any(safety.get(field) is not expected for field, expected in (
            ('ground_truth_content_opened', False),
            ('scorer_invoked', False),
            ('input_modified', False),
            ('receipt_external_to_output', True))):
        raise Ros1EquivalenceError(
            'fixed_receipt_safety', 'fixed receipt safety flags are not blind')
    observed_tree = sha256_tree(configured_output)
    output_tree = output.get('tree')
    if not isinstance(output_tree, dict) or observed_tree != output_tree:
        raise Ros1EquivalenceError(
            'fixed_output_changed', 'fixed ROS2 output tree differs from receipt')
    if output_identity.get('tree_sha256') != observed_tree['sha256']:
        raise Ros1EquivalenceError(
            'profile_output_changed', 'profile ROS2 tree differs from receipt')
    if output_identity.get('output_record_count') != output.get('record_count'):
        raise Ros1EquivalenceError(
            'profile_output_changed', 'profile ROS2 record count differs from receipt')
    if (final_path.suffix != '.bag'
            or final_path.name != Path(str(ros1.get('ros1_output_path'))).name):
        raise Ros1EquivalenceError(
            'ros1_basename', 'final ROS1 output must retain the exact .bag basename')
    return {
        'profile': profile,
        'contract': contract,
        'receipt': receipt,
        'receipt_sha256': receipt_sha,
        'generator_sha256': generator_sha,
        'converter_path': converter_path,
        'converter_sha256': converter_sha,
        'converter_help_sha256': help_sha,
        'comparator_path': comparator_path,
        'comparator_sha256': comparator_sha,
        'expected_topic_counts': expected_counts,
    }


def _assert_paths(
    input_root: Path,
    final_path: Path,
    receipt_path: Path,
    semantic_report_path: Path,
) -> tuple[Path, Path]:
    input_root = _regular(input_root, 'ROS2 input', directory=True)
    # Inspect the caller-provided spelling before resolve(strict=False) can
    # erase an existing symlink from the path.  This also covers a symlink
    # ancestor whose target does not yet contain the requested output.
    for raw_path, label in (
            (Path(final_path).expanduser(), 'final'),
            (Path(receipt_path).expanduser(), 'receipt'),
            (Path(semantic_report_path).expanduser(), 'semantic')):
        _assert_no_symlink_ancestors(raw_path, label)
    final_path = _resolve_new(final_path, 'ROS1 final output')
    receipt_path = _resolve_new(receipt_path, 'external receipt')
    semantic_report_path = _resolve_new(semantic_report_path, 'semantic report')
    if final_path.suffix != '.bag':
        raise Ros1EquivalenceError(
            'ros1_basename', 'ROS1 final output must have .bag extension')
    stage_container = final_path.with_suffix('.staging')
    if stage_container.exists() or stage_container.is_symlink():
        raise Ros1EquivalenceError(
            'staging_exists', f'ROS1 staging container exists: {stage_container}')
    for legacy_part in (
            final_path.with_name(final_path.name + '.part'),
            final_path.with_suffix('.part')):
        if legacy_part.exists() or legacy_part.is_symlink():
            raise Ros1EquivalenceError(
                'staging_exists', f'legacy ROS1 staging artifact exists: {legacy_part}')
    if final_path.name != Path(final_path.name).name:
        raise Ros1EquivalenceError('ros1_basename', 'invalid ROS1 output basename')
    named_paths = {
        'final': final_path,
        'receipt': receipt_path,
        'semantic': semantic_report_path,
    }
    for name, path in named_paths.items():
        _assert_no_symlink_ancestors(path, name)
    _assert_not_overlapping(named_paths, {'input': input_root, 'stage': stage_container})
    names = list(named_paths.items())
    for index, (left_name, left_path) in enumerate(names):
        for right_name, right_path in names[index + 1:]:
            if _under(left_path, right_path) or _under(right_path, left_path):
                raise Ros1EquivalenceError(
                    'path_overlap', f'{left_name} overlaps {right_name}')
    final_path.parent.mkdir(parents=True, exist_ok=True)
    if final_path.parent.is_symlink():
        raise Ros1EquivalenceError('unsafe_path', 'ROS1 output parent is a symlink')
    return stage_container, final_path


def build_conversion_argv(
    converter_path: Path,
    ros2_input: Path,
    ros1_stage: Path,
) -> list[str]:
    """Build a shell-free argv with an immutable executable path."""
    if ros1_stage.suffix != '.bag':
        raise Ros1EquivalenceError(
            'ros1_basename', 'staging output must retain .bag extension')
    return [
        str(converter_path),
        '--src', str(ros2_input),
        '--dst', str(ros1_stage),
        '--compress', 'none',
        '--src-typestore', 'ros2_humble',
        '--dst-typestore', 'ros1_noetic',
    ]


def build_comparator_argv(
    comparator_path: Path,
    ros1_input: Path,
    ros2_input: Path,
    report_path: Path,
) -> list[str]:
    """Build the exact three-topic semantic comparator argv."""
    argv = [
        sys.executable, str(comparator_path),
        '--left', str(ros1_input), '--right', str(ros2_input),
    ]
    for topic in EXPECTED_TOPICS:
        argv.extend(('--topic', topic))
    argv.extend(('--output', str(report_path)))
    return argv


def _run(
    argv: Sequence[str],
    runner: CommandRunner | None,
) -> subprocess.CompletedProcess[str]:
    if runner is not None:
        return runner(argv)
    return subprocess.run(
        list(argv), check=False, capture_output=True, text=True, shell=False)


def _stream_sha256(value: str | bytes | None) -> dict[str, Any]:
    """Record subprocess stream identity without persisting its contents."""
    if value is None:
        payload = b''
    elif isinstance(value, bytes):
        payload = value
    elif isinstance(value, str):
        payload = value.encode('utf-8', errors='surrogateescape')
    else:
        raise Ros1EquivalenceError('subprocess_stream', 'subprocess stream has invalid type')
    return {'sha256': hashlib.sha256(payload).hexdigest(), 'bytes': len(payload)}


def _inspect_ros1_bag(
    path: Path,
    expected_counts: Mapping[str, int],
) -> dict[str, Any]:
    """Inspect the staged ROS1 stream before publication.

    This is intentionally a streaming count pass: no deserialized messages or
    payloads are retained.  The converter is not allowed to publish an output
    with an extra connection, an unexpected message type, or a count mismatch.
    """
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:  # pragma: no cover - runtime dependency
        raise Ros1EquivalenceError('ros1_inspection', 'rosbags AnyReader is unavailable') from exc
    try:
        with AnyReader([path]) as reader:
            connections = list(reader.connections)
            if len(connections) != len(expected_counts):
                raise Ros1EquivalenceError(
                    'ros1_connections', 'ROS1 output has an unexpected connection count')
            by_topic: dict[str, Any] = {}
            for connection in connections:
                topic = getattr(connection, 'topic', None)
                msgtype = getattr(connection, 'msgtype', None)
                if topic not in expected_counts or topic in by_topic:
                    raise Ros1EquivalenceError(
                        'ros1_connections', 'ROS1 output has an extra or duplicate topic')
                if msgtype != EXPECTED_TOPIC_TYPES.get(topic):
                    raise Ros1EquivalenceError(
                        'ros1_connections', f'ROS1 message type mismatch for {topic}')
                msgdef = getattr(getattr(connection, 'msgdef', None), 'data', None)
                msgdef_sha = None
                if isinstance(msgdef, str):
                    msgdef_sha = hashlib.sha256(msgdef.encode('utf-8')).hexdigest()
                by_topic[topic] = {
                    'connection_id': getattr(connection, 'id', None),
                    'topic': topic,
                    'msgtype': msgtype,
                    'serialization_format': getattr(connection, 'serialization_format', None),
                    'offered_qos_profiles': getattr(connection, 'offered_qos_profiles', None),
                    'msgdef_sha256': msgdef_sha,
                    'rihs01': getattr(connection, 'rihs01', None),
                }
            counts = {topic: 0 for topic in expected_counts}
            for connection, _timestamp, _rawdata in reader.messages():
                topic = getattr(connection, 'topic', None)
                if topic not in counts:
                    raise Ros1EquivalenceError(
                        'ros1_connections', 'ROS1 stream contains an unexpected topic')
                counts[topic] += 1
            if counts != dict(expected_counts):
                raise Ros1EquivalenceError(
                    'ros1_counts', f'ROS1 topic counts differ: {counts!r}')
            total = sum(counts.values())
            if total != sum(expected_counts.values()):
                raise Ros1EquivalenceError('ros1_counts', 'ROS1 total message count differs')
            return {
                'status': 'PASS',
                'format': 'rosbag1',
                'total_message_count': total,
                'topic_counts': counts,
                'connections': [by_topic[topic] for topic in EXPECTED_TOPICS],
            }
    except Ros1EquivalenceError:
        raise
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise Ros1EquivalenceError(
            'ros1_inspection', 'ROS1 output could not be inspected') from exc


def _validate_semantic_report(
    path: Path,
    expected_counts: Mapping[str, int],
) -> dict[str, Any]:
    report = _load_json(
        _regular(path, 'semantic report', directory=False), 'semantic report')
    if report.get('schema_version') != 1 or report.get('all_topics_equal') is not True:
        raise Ros1EquivalenceError('semantic_failed', 'semantic comparator did not pass')
    rows = report.get('topics')
    if not isinstance(rows, list):
        raise Ros1EquivalenceError('semantic_failed', 'semantic comparator topics are missing')
    if len(rows) != len(expected_counts):
        raise Ros1EquivalenceError(
            'semantic_failed', 'semantic topic set differs from fixed NTU topics')
    if any(not isinstance(row, dict) for row in rows):
        raise Ros1EquivalenceError('semantic_failed', 'semantic topic rows are malformed')
    by_topic = {row.get('topic'): row for row in rows}
    if len(by_topic) != len(rows):
        raise Ros1EquivalenceError('semantic_failed', 'semantic topic rows are duplicated')
    if set(by_topic) != set(expected_counts):
        raise Ros1EquivalenceError(
            'semantic_failed', 'semantic topic set differs from fixed NTU topics')
    for topic, expected_count in expected_counts.items():
        row = by_topic[topic]
        if row.get('equal') is not True or row.get('message_count_left') != expected_count \
                or row.get('message_count_right') != expected_count:
            raise Ros1EquivalenceError(
                'semantic_failed', f'semantic count/equality mismatch for {topic}')
        left_sha = _require_sha(
            row.get('aggregate_sha256_left'), f'{topic} semantic left digest')
        right_sha = _require_sha(
            row.get('aggregate_sha256_right'), f'{topic} semantic right digest')
        if left_sha != right_sha:
            raise Ros1EquivalenceError(
                'semantic_failed', f'{topic} semantic aggregate digests differ')
    return {
        'status': 'PASS',
        'schema_version': 1,
        'all_topics_equal': True,
        'topics': [by_topic[topic] for topic in EXPECTED_TOPICS],
    }


def _atomic_write_json(path: Path, document: dict[str, Any]) -> None:
    part = path.with_name(path.name + '.part')
    if path.exists() or path.is_symlink() or part.exists() or part.is_symlink():
        raise Ros1EquivalenceError('receipt_exists', f'receipt or staging exists: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (
        json.dumps(document, sort_keys=True, separators=(',', ':')) + '\n').encode()
    with part.open('xb') as stream:
        stream.write(encoded)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(part, path)
    directory_fd = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def materialize_ros1_equivalent(
    input_root: Path,
    fixed_receipt_path: Path,
    final_path: Path,
    receipt_path: Path,
    *,
    profile_path: Path | None = None,
    semantic_report_path: Path | None = None,
    runner: CommandRunner | None = None,
    expected_topic_counts: Mapping[str, int] | None = None,
) -> dict[str, Any]:
    """Convert, semantically verify, and atomically publish the ROS1 bag."""
    input_root = _regular(input_root, 'ROS2 input', directory=True)
    final_path = Path(final_path).expanduser().resolve(strict=False)
    receipt_path = Path(receipt_path).expanduser().resolve(strict=False)
    if semantic_report_path is None:
        semantic_report_path = receipt_path.with_name(receipt_path.name + '.semantic.json')
    else:
        semantic_report_path = Path(semantic_report_path).expanduser().resolve(strict=False)
    stage_container, final_path = _assert_paths(
        input_root, final_path, receipt_path, semantic_report_path)
    profile_path = profile_path or (
        ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml')
    identity = validate_profile_and_tools(
        profile_path, fixed_receipt_path, final_path, input_root,
        expected_topic_counts=expected_topic_counts)
    expected_counts = identity['expected_topic_counts']
    converter_path = identity['converter_path']
    comparator_path = identity['comparator_path']
    stage = stage_container / final_path.name
    conversion_argv = build_conversion_argv(converter_path, input_root, stage)
    stage_container.mkdir()
    try:
        conversion = _run(conversion_argv, runner)
        if conversion.returncode != 0:
            raise Ros1EquivalenceError(
                'conversion_failed', f'rosbags-convert exited {conversion.returncode}')
        stage = _regular(stage, 'ROS1 staged output', directory=False)
        semantic_argv = build_comparator_argv(
            comparator_path, stage, input_root, semantic_report_path)
        semantic = _run(semantic_argv, runner)
        if semantic.returncode != 0:
            raise Ros1EquivalenceError(
                'semantic_failed', f'semantic comparator exited {semantic.returncode}')
        semantic_identity = _validate_semantic_report(semantic_report_path, expected_counts)
        ros1_inspection = _inspect_ros1_bag(stage, expected_counts)
        ros1_size = stage.stat().st_size
        ros1_sha = sha256_file(stage)
        os.replace(stage, final_path)
        final_sha = sha256_file(final_path)
        final_size = final_path.stat().st_size
        if final_sha != ros1_sha or final_size != ros1_size:
            raise Ros1EquivalenceError(
                'published_output_changed', 'published ROS1 file differs from stage')
        os.rmdir(stage_container)
        receipt = {
            'schema_version': SCHEMA_VERSION,
            'receipt_kind': RECEIPT_KIND,
            'contract_id': CONTRACT_ID,
            'status': 'PASS',
            'receipt_path': str(receipt_path),
            'input': {
                'path': str(input_root),
                'tree_sha256': identity['receipt']['output']['tree']['sha256'],
                'materialization_receipt_sha256': identity['receipt_sha256'],
            },
            'ros2_materialization': {
                'contract_id': identity['receipt']['contract_id'],
                'analyzer_contract_id': identity['receipt']['analyzer']['contract_id'],
                'receipt_kind': identity['receipt']['receipt_kind'],
                'receipt_sha256': identity['receipt_sha256'],
                'output_tree_sha256': identity['receipt']['output']['tree']['sha256'],
                'output_record_count': identity['receipt']['output']['record_count'],
                'per_topic': identity['receipt']['output']['per_topic'],
                'verification': identity['receipt']['verification'],
                'order_definition': identity['receipt']['order_definition'],
                'generator': {
                    'path': GENERATOR_RELATIVE_PATH,
                    'sha256': identity['generator_sha256'],
                },
            },
            'conversion': {
                'argv': conversion_argv,
                'converter': CONVERTER,
                'version': CONVERTER_VERSION,
                'executable_path': str(converter_path),
                'executable_sha256': identity['converter_sha256'],
                'help_sha256': identity['converter_help_sha256'],
                'exit_code': conversion.returncode,
                'stdout': _stream_sha256(getattr(conversion, 'stdout', None)),
                'stderr': _stream_sha256(getattr(conversion, 'stderr', None)),
                'staging_policy': 'sibling_container_exact_final_basename',
            },
            'ros1_output': {
                'path': str(final_path),
                'sha256': final_sha,
                'bytes': final_size,
                'basename': final_path.name,
                'message_count': sum(expected_counts.values()),
                'topic_counts': dict(expected_counts),
                'inspection': ros1_inspection,
            },
            'semantic_equivalence': {
                'comparator_path': COMPARATOR_RELATIVE_PATH,
                'comparator_sha256': identity['comparator_sha256'],
                'argv': semantic_argv,
                'report_path': str(semantic_report_path),
                'report_sha256': sha256_file(semantic_report_path),
                'exit_code': semantic.returncode,
                'stdout': _stream_sha256(getattr(semantic, 'stdout', None)),
                'stderr': _stream_sha256(getattr(semantic, 'stderr', None)),
                **semantic_identity,
            },
            'safety': {
                'atomic_publish': True,
                'input_modified': False,
                'ground_truth_content_opened': False,
                'scorer_invoked': False,
                'receipt_external_to_output': True,
                'semantic_verified_before_publish': True,
                'staging_container_removed_after_publish': True,
            },
        }
        _atomic_write_json(receipt_path, receipt)
        return {
            'receipt': receipt,
            'receipt_sha256': sha256_file(receipt_path),
            'conversion_argv': conversion_argv,
            'semantic_argv': semantic_argv,
        }
    except Exception:
        # Do not remove a staging artifact after any failure.  It is the
        # immutable diagnostic needed to explain converter or semantic drift.
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--fixed-receipt', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--receipt', type=Path, required=True)
    parser.add_argument('--profile', type=Path)
    parser.add_argument('--semantic-report', type=Path)
    args = parser.parse_args(argv)
    try:
        result = materialize_ros1_equivalent(
            args.input, args.fixed_receipt, args.output, args.receipt,
            profile_path=args.profile, semantic_report_path=args.semantic_report)
    except (OSError, Ros1EquivalenceError, ValueError, TypeError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    print(json.dumps({
        'status': result['receipt']['status'],
        'receipt_path': result['receipt']['receipt_path'],
        'receipt_sha256': result['receipt_sha256'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
