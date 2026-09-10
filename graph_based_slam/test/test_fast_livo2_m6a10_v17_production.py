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


"""Static and host-only gates for the additive v17 production patch."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]
PATCH = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v17-retryable-abort.patch'
SELFTEST = ROOT / 'tools/m6a10_terminal_support_context_v17_production_selftest.cpp'
WRAPPER = ROOT / 'scripts/fast_livo2_m6a10_v17_formal_container_run.sh'
PAYLOAD = ROOT / 'scripts/fast_livo2_m6a10_v17_no_input_container_payload.sh'
BUILD = ROOT / 'scripts/build_fast_livo2_m6a10_v17_image.sh'
NO_INPUT_GATE = ROOT / 'scripts/run_fast_livo2_m6a10_v17_no_input_gate.sh'
DOCKERFILE = ROOT / 'docker/fast_livo2_m6a10_v17.Dockerfile'
PROFILE = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_candidate.yaml'
SOURCE = Path(os.environ.get('M6A10_V17_PATCHED_SOURCE', '/tmp/m6a10-v17-patch-source.qaXCFc/src'))
PATCH_SHA = 'c9254497b21846b2de6a028aa5fd60927a80da641c2fb47943c49ff3636ca001'
SELFTEST_SHA = 'f81bde7cabe54ebfa702908afa9ef071f2436ce5fee2f50bf67e3c494972bc84'
WRAPPER_SHA = '506c6a8c72bca9fa66cf49abaead79eabc5b177fa9b1b30f868383a98a5ab67e'
PAYLOAD_SHA = 'a8043f751db0638f34e79254133ac5974527c609f7574c616ee219a685cd8ed5'
PROFILE_SHA = '1803ba255adfac96a61e1f677be53e84b196ad70bb367a10ebdb9791cb94128b'


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_v17_source_files_are_pinned_and_patch_is_narrow():
    assert _sha(PATCH) == PATCH_SHA
    assert _sha(SELFTEST) == SELFTEST_SHA
    assert _sha(WRAPPER) == WRAPPER_SHA
    assert _sha(PAYLOAD) == PAYLOAD_SHA
    assert _sha(PROFILE) == PROFILE_SHA
    patch = PATCH.read_text(encoding='utf-8')
    assert 'diff --git a/include/m6a10_terminal_support_context.h' in patch
    assert 'diff --git a/src/LIVMapper.cpp' in patch
    assert 'include/m6a10_consumer_evidence.h' not in patch
    assert 'transport_contract' not in patch
    assert 'schema_version' not in patch
    assert (
        patch.count('m6a10_terminal_support_context.abort_retryable_synchronization_unit();') == 3
    )
    assert 'completion_disposition_valid_ = false' in patch
    assert 'discard_unit_locked("retryable_abort_after_record")' in patch


def test_production_source_has_three_retry_sites_and_fatal_discard_sites():
    if not SOURCE.is_dir():
        pytest.skip('local post-v12 source extraction is not mounted')
    header = (SOURCE / 'include/m6a10_terminal_support_context.h').read_text(encoding='utf-8')
    mapper = (SOURCE / 'src/LIVMapper.cpp').read_text(encoding='utf-8')
    assert 'void abort_retryable_synchronization_unit()' in header
    assert '++retryable_synchronization_aborts_;' in header
    assert mapper.count('abort_retryable_synchronization_unit();') == 3
    assert mapper.count('abort_synchronization_unit();') >= 5
    discard = mapper.index(
        'observe_queue_pop(\n          M6A10TerminalSupportContext::Topic::Image, false)'
    )
    assert mapper.index('abort_synchronization_unit();', discard) < mapper.index(
        'return false;', discard
    )
    malformed = mapper.index('meas.lidar->points.size() <= 1')
    assert mapper.index('abort_synchronization_unit();', malformed) < mapper.index(
        'return false;', malformed
    )


def test_patch_apply_check_and_reapply_on_actual_post_v12_source(tmp_path: Path):
    if not SOURCE.is_dir() or not (SOURCE / '.git').is_dir():
        pytest.skip('local post-v12 source extraction is not mounted')
    copied = tmp_path / 'FAST-LIVO2'
    shutil.copytree(SOURCE, copied, symlinks=True)
    subprocess.run(['git', 'apply', '-R', '--recount', str(PATCH)], cwd=copied, check=True)
    subprocess.run(['git', 'apply', '--check', '--recount', str(PATCH)], cwd=copied, check=True)
    subprocess.run(['git', 'apply', '--recount', str(PATCH)], cwd=copied, check=True)
    assert 'void abort_retryable_synchronization_unit()' in (
        copied / 'include/m6a10_terminal_support_context.h'
    ).read_text(encoding='utf-8')


def test_production_selftest_compiles_with_werror_and_runs(tmp_path: Path):
    if not SOURCE.is_dir():
        pytest.skip('local post-v12 source extraction is not mounted')
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('g++ unavailable')
    binary = tmp_path / 'm6a10_v17_production_selftest'
    subprocess.run(
        [
            compiler,
            '-std=c++17',
            '-Wall',
            '-Wextra',
            '-Werror',
            '-I',
            str(SOURCE / 'include'),
            '-I',
            str(ROOT / 'tools/m6a10_v12_test_stubs'),
            str(SELFTEST),
            '-o',
            str(binary),
        ],
        cwd=ROOT,
        check=True,
    )
    output_dir = tmp_path / 'evidence'
    output_dir.mkdir()
    result = subprocess.run(
        [str(binary)],
        cwd=ROOT,
        env={**os.environ, 'M6A10_SELFTEST_OUTPUT_DIR': str(output_dir)},
        check=True,
        capture_output=True,
        text=True,
    )
    assert 'empty_retry_then_complete actual=pass' in result.stdout
    assert 'partial_retry_after_record actual=invalid' in result.stdout
    assert 'explicit_discard actual=invalid' in result.stdout
    assert 'processing_failure actual=invalid' in result.stdout
    assert 'M6A10_V17_PRODUCTION_ALL PASS' in result.stdout


def test_wrapper_payload_and_build_are_input_free_candidate_contracts():
    wrapper = WRAPPER.read_text(encoding='utf-8')
    payload = PAYLOAD.read_text(encoding='utf-8')
    build = BUILD.read_text(encoding='utf-8')
    dockerfile = DOCKERFILE.read_text(encoding='utf-8')
    profile = PROFILE.read_text(encoding='utf-8')
    assert 'roscore >"${OUT_DIR}/roscore.log" 2>&1 &' in wrapper
    assert 'ROSCORE_PID=$!' in wrapper
    assert 'roslaunch fast_livo mapping_ouster_ntu.launch rviz:=false' in wrapper
    assert 'MAPPER_PID=$!' in wrapper
    assert wrapper.count('call_trigger /m6a10/consumer_eof') == 1
    assert wrapper.count('call_trigger /m6a10/consumer_finalize') == 1
    assert wrapper.count('/m6a10/terminal_status') >= 2
    assert 'abort_retryable' not in wrapper
    assert 'M6A10_V17_NO_INPUT_PASS' in payload
    assert 'validate_consumer_status' in payload
    assert 'docker build --network none --pull=false' in build
    assert '--build-arg "FAST_LIVO2_V17_PATCH_SHA256=' in build
    mapper_sha = 'c2a62cf0a6943a8c68084e70585955212bbe944b8788d285e4ca41845492ba94'
    assert mapper_sha in build
    assert f'ARG FAST_LIVO2_V17_MAPPER_SHA256={mapper_sha}' in dockerfile
    assert f'v17_installed_mapper_sha256: {mapper_sha}' in profile
    assert (
        'prior_v17_build_failure_receipt_sha256: '
        '1eccad6bd1c714a7807a830520db202f0fd3843d3d0d55b3171a17c74f4d0343'
        in profile
    )
    assert 'formal_replay_forbidden: true' in profile
    assert 'retryable_empty_unit_non_sticky: true' in profile
    assert 'explicit_discard_sticky_invalid: true' in profile


def test_no_input_gate_is_single_isolated_correction_contract():
    gate = NO_INPUT_GATE.read_text(encoding='utf-8')
    assert NO_INPUT_GATE.stat().st_mode & 0o777 == 0o755
    assert (
        'docker run --name "${CONTAINER_NAME}" --init --pull=never --network none --read-only'
        in gate
    )
    assert gate.count('docker run --name') == 1
    assert '--env ROS_MASTER_URI=http://127.0.0.1:11311' in gate
    assert '--env ROS_IP=127.0.0.1' in gate
    assert '--env ROS_HOSTNAME=127.0.0.1' in gate
    assert '--tmpfs /tmp:rw,nosuid,nodev' in gate
    assert '--tmpfs /root/.ros:rw,nosuid,nodev' in gate
    assert '--tmpfs /out:rw,nosuid,nodev' in gate
    assert "value.get('Mounts')" in gate
    assert "host.get('Tmpfs')" in gate
    assert (
        '--mount' not in gate and 'docker run -v' not in gate and 'docker run --volume' not in gate
    )
    assert 'BAG_PATH' in gate and 'GROUND_TRUTH_PATH' in gate and 'SCORER_PATH' in gate
    assert 'M6A10_V17_PRODUCTION_ALL PASS' in gate
    assert 'M6A10_V17_NO_INPUT_SCHEMA3_PASS' in gate
    assert 'M6A10_V17_NO_INPUT_PASS' in gate
    assert 'empty_retry_then_complete' in gate
    assert 'partial_retry_after_record' in gate
    assert 'processing_failure' in gate
    assert 'stopped_only_remove_success' in gate
    assert 'v17_no_input_gate_sha256' in gate
    assert "formal_replay_started': False" in gate
