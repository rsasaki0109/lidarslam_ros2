# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
Read-only v12 terminal and callback-transport production-header gates.

The test applies the real patch chain to a fresh source tree and compiles the
v12 selftest against the resulting production header.  It never starts ROS,
Docker, a bag, GT, or a scorer.
"""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
PATCH = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch'
SELFTEST = ROOT / 'tools/m6a10_terminal_support_context_v12_selftest.cpp'
CONSUMER_SELFTEST = ROOT / 'tools/m6a10_consumer_evidence_v12_selftest.cpp'
STUBS = ROOT / 'tools/m6a10_v12_test_stubs'
PROFILE = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_ready.yaml'
CONTRACT = 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary'
RAW_PATH = Path(
    '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/'
    'fast_livo2_v2c_v11_formal_replay_20260823T140331Z_agentv11formal/'
    'out/consumer_evidence.json'
)
RAW_SHA256 = '020de643f37e3cea269445b674c49e5c908b9632ca4c0981221df9fced606b6e'
SOURCE = Path('/tmp/fast-livo2-v10-compile-final3.vCnnqO')
SOURCE_SHAS = {
    'include/m6a10_consumer_evidence.h':
        '6004e7019b2a03d8f28d806a6b82c562557fc8a9acd88fb180fb60ca2c58fa84',
    'include/m6a10_terminal_support_context.h':
        '091c84db7ce1046e39a661332701be8e812de84d2cbe336821fe347344dc9430',
    'src/LIVMapper.cpp': '217f91e06bbb5abc2153d036fc5c1616f1f4f1edc95053a858c0a4d4e3c33a31',
}
CHAIN = [
    ROOT / 'docker/patches/fast_livo2.m6a10-v2c.patch',
    ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v8-service-queue.patch',
    ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v9-wallrate.patch',
    ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch',
    ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch',
    PATCH,
]


def _validator():
    spec = importlib.util.spec_from_file_location(
        'm6a10_phase_contract_v12_terminal', ROOT / 'scripts/benchmark_phase_contract.py'
    )
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def _assert_source_pin():
    assert SOURCE.is_dir()
    for relative, expected in SOURCE_SHAS.items():
        assert hashlib.sha256((SOURCE / relative).read_bytes()).hexdigest() == expected


def _fresh_chain(tmp_path: Path) -> Path:
    _assert_source_pin()
    repo = tmp_path / 'production'
    repo.mkdir()
    archive = subprocess.run(
        ['git', '-C', str(SOURCE), 'archive', 'HEAD'], check=True, capture_output=True
    ).stdout
    with tarfile.open(fileobj=io.BytesIO(archive), mode='r:') as stream:
        stream.extractall(repo)
    subprocess.run(['git', 'init', '--quiet', str(repo)], check=True)
    subprocess.run(['git', '-C', str(repo), 'add', '-A'], check=True)
    subprocess.run(
        [
            'git',
            '-C',
            str(repo),
            '-c',
            'user.name=v12-test',
            '-c',
            'user.email=v12-test@example.invalid',
            'commit',
            '--quiet',
            '-m',
            'baseline',
        ],
        check=True,
    )
    for patch in CHAIN:
        command = ['git', '-C', str(repo), 'apply', '--check', '--recount', str(patch)]
        checked = subprocess.run(command, check=False, capture_output=True, text=True)
        assert checked.returncode == 0, f'{patch}: {checked.stderr}'
        subprocess.run(['git', '-C', str(repo), 'apply', '--recount', str(patch)], check=True)
    return repo


def test_v12_patch_integrates_terminal_and_v5_transport_only_and_v11_immutable():
    text = PATCH.read_text(encoding='utf-8')
    assert 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary' in text
    assert 'equal_to_completed_boundary_nonlidar_support_context' in text
    assert 'equal_to_completed_boundary' in text
    assert 'valid_locked' in text and 'support_record' in text
    assert 'm6a10_consumer_evidence.h' in text
    assert 'v5_transport_contract_' in text
    assert 'transport_outstanding_at_drain' in text
    assert 'maximum_transport_outstanding_messages' in text
    assert 'mapper_internal_deque_peak_messages' in text
    assert 'observe_queue_overflow' in text
    assert 'LIVMapper.cpp' not in text
    assert 'm6a10_v12_terminal_policy' not in text
    assert 'm6a10_v12_transport_ledger' not in text
    result = subprocess.run(
        ['git', 'apply', '--stat', str(PATCH)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    v11 = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch'
    assert hashlib.sha256(v11.read_bytes()).hexdigest() == (
        '2cbca3a7bb465981cb5e3072efd1713302e002127d4ae5872413ed5749d80ba2'
    )


def test_exact_patch_chain_applies_to_fresh_production_source(tmp_path):
    repo = _fresh_chain(tmp_path)
    terminal = (repo / 'include/m6a10_terminal_support_context.h').read_text()
    assert 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary' in terminal
    assert 'equal_to_completed_boundary' in terminal
    assert 'nonlidar_at_or_after_boundary_support_context' in terminal
    assert 'timestamp_seconds < required_end_timestamp_seconds_' not in terminal
    assert 'transport_outstanding' not in terminal
    assert 'm6a10_v12_terminal_policy' not in terminal
    assert 'm6a10_v12_transport_ledger' not in terminal
    consumer = (repo / 'include/m6a10_consumer_evidence.h').read_text()
    assert 'v5_transport_contract_' in consumer
    assert 'transport_outstanding_at_drain' in consumer
    assert 'mapper_internal_deque_peak_messages' in consumer


def test_v12_selftest_compiles_actual_patched_header_with_werror(tmp_path):
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('g++ unavailable')
    repo = _fresh_chain(tmp_path)
    binary = tmp_path / 'v12_selftest'
    subprocess.run(
        [
            compiler,
            '-std=c++17',
            '-Wall',
            '-Wextra',
            '-Werror',
            '-O2',
            '-I',
            str(STUBS),
            '-I',
            str(repo / 'include'),
            str(SELFTEST),
            '-o',
            str(binary),
        ],
        cwd=ROOT,
        check=True,
    )
    result = subprocess.run(
        [str(binary)],
        cwd=ROOT,
        env={**__import__('os').environ, 'M6A10_SELFTEST_OUTPUT_DIR': str(tmp_path)},
        check=True,
        capture_output=True,
        text=True,
    )
    assert 'M6A10_SYNTHETIC_ALL PASS' in result.stdout
    for marker in (
        'equal_boundary_nonlidar',
        'strict_after_reason',
        'equal_boundary_lidar',
        'pre_boundary_tail',
        'forming_unit',
        'predicate_unproven',
        'wrong_reason',
        'bounded_end_gap',
        'excessive_end_gap',
        'future_boundary',
    ):
        assert f'M6A10_SYNTHETIC_CASE {marker}' in result.stdout


def test_v12_consumer_selftest_compiles_actual_header_with_werror(tmp_path):
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('g++ unavailable')
    repo = _fresh_chain(tmp_path)
    binary = tmp_path / 'consumer_v12_selftest'
    subprocess.run(
        [
            compiler,
            '-std=c++17',
            '-Wall',
            '-Wextra',
            '-Werror',
            '-O2',
            '-I',
            str(STUBS),
            '-I',
            str(repo / 'include'),
            str(CONSUMER_SELFTEST),
            '-o',
            str(binary),
        ],
        cwd=ROOT,
        check=True,
    )
    result = subprocess.run(
        [str(binary)],
        cwd=ROOT,
        env={**__import__('os').environ, 'M6A10_SELFTEST_OUTPUT_DIR': str(tmp_path)},
        check=True,
        capture_output=True,
        text=True,
    )
    assert 'M6A10_CONSUMER_ALL PASS' in result.stdout
    for marker in (
        'one_callback_tail81',
        'two_before_ack_peak2',
        'duplicate_ack_rejected',
        'outstanding_nonzero',
        'count_mismatch',
        'drop_rejected',
        'overflow_rejected',
        'callback_latency_rejected',
        'v4_legacy_tail81',
        'unknown_contract_disabled',
    ):
        assert f'M6A10_CONSUMER_CASE {marker}' in result.stdout


def _v5_from_attempt4_raw() -> dict:
    assert RAW_PATH.is_file()
    assert hashlib.sha256(RAW_PATH.read_bytes()).hexdigest() == RAW_SHA256
    value = json.loads(RAW_PATH.read_text(encoding='utf-8'))
    result = copy.deepcopy(value)
    boundary = 1623491515.03615
    required = 1623491515.148352
    received = result['received_topic_counts']
    result['schema_version'] = 3
    result['contract_version'] = CONTRACT
    result['status'] = 'pass'
    result['required_evaluation_end_timestamp_seconds'] = required
    result['maximum_end_gap_seconds'] = 0.25
    result['counts'] = {
        'expected': dict(received),
        'published': dict(received),
        'received': dict(received),
        'acknowledged': dict(received),
    }
    result['terminal_support_context'][
        'classification'
    ] = 'nonlidar_at_or_after_boundary_support_context'
    for topic in ('imu', 'image'):
        for record in result['buffers'][topic]['records']:
            record.setdefault('equal_to_completed_boundary', False)
    for record in result['terminal_support_context']['records']:
        record.setdefault('equal_to_completed_boundary', False)
    for record in result['buffers']['image']['records']:
        if record['timestamp_seconds'] == boundary:
            record['equal_to_completed_boundary'] = True
            record['reason_code'] = 'equal_to_completed_boundary_nonlidar_support_context'
            record['post_boundary'] = False
            record['support_context_proven'] = True
    for record in result['terminal_support_context']['records']:
        if record['timestamp_seconds'] == boundary:
            record['equal_to_completed_boundary'] = True
            record['reason_code'] = 'equal_to_completed_boundary_nonlidar_support_context'
            record['post_boundary'] = False
            record['support_context_proven'] = True
    return result


def test_attempt4_exact_raw_fixture_converts_to_v5_pass():
    validator = _validator()
    result = validator.validate_terminal_support_context_v5(
        _v5_from_attempt4_raw(), maximum_end_gap_seconds=0.25, require_pass=True
    )
    assert result['status'] == 'pass'
    assert result['validation']['support_context_total_count'] == 81
    assert result['validation']['residual_lidar_forbidden'] is True


@pytest.mark.parametrize(
    'field, value',
    [
        ('equal_to_completed_boundary', False),
        ('post_boundary', True),
    ],
)
def test_exact_boundary_record_negative_cases_are_fail_closed(field, value):
    validator = _validator()
    document = _v5_from_attempt4_raw()
    image = next(
        record
        for record in document['buffers']['image']['records']
        if record['timestamp_seconds'] == 1623491515.03615
    )
    image[field] = value
    with pytest.raises(validator.PhaseContractError):
        validator.validate_terminal_support_context_v5(document, require_pass=True)


def test_v12_profile_remains_implementation_only_and_safe():
    profile = yaml.safe_load(PROFILE.read_text(encoding='utf-8'))
    candidate = profile['competitive_slam_profile']['m6a10_fast_livo2_v2c_v12_formal_ready']
    assert candidate['phase']['contract_version'] == CONTRACT
    assert candidate['execution']['input_opened'] is False
    assert candidate['execution']['ground_truth_content_opened'] is False
    assert candidate['execution']['scorer_invoked'] is False
    assert candidate['execution']['formal_replay_started'] is False
