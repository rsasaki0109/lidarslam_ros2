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
Static/build-contract gates for the v11 image candidate.

These tests inspect only source files and shell syntax.  They never invoke
Docker, catkin, ROS, a bag, a feeder, a scorer, or a replay.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
DOCKERFILE = ROOT / 'docker/fast_livo2_m6a10_v11.Dockerfile'
BUILD_SCRIPT = ROOT / 'scripts/build_fast_livo2_m6a10_v11_image.sh'
PATCH = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch'
SELFTEST = ROOT / 'tools/m6a10_terminal_support_context_v11_selftest.cpp'

V10_FILES = {
    ROOT
    / 'docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch':
    '155bae4e37eac7d6220baedd861b9110b6139ee2b384d5b90db78ac24a6733c6',
    ROOT
    / 'tools/m6a10_terminal_support_context_selftest.cpp':
    '4e37d0aa67fa384e99cf633c0afab95fd718484a5ddf5758e3f6807a3a438941',
    ROOT
    / 'docker/fast_livo2_m6a10_v10.Dockerfile':
    '405a96bb37d543966bb49a9eae418d46e4809633f754685b2a722716750fb49a',
    ROOT
    / 'scripts/build_fast_livo2_m6a10_v10_image.sh':
    '05473e6dc0efd8f14f02b90e3a79938940446f03a6ced7029d3dd7a7e357b124',
}

V10_BASE_ID = 'sha256:3e087acc5ef116f03357a73927c18b2059068b093ae1cce6cb41c1baf1fbf759'
V10_BASE_TAG = 'm6a10-v2c-v10-callback-v3-20260823t094100z-fast-livo2-benchmark:ros1-pinned'
V11_PATCH_SHA256 = '2cbca3a7bb465981cb5e3072efd1713302e002127d4ae5872413ed5749d80ba2'
V11_SELFTEST_SHA256 = '7eaf8fbc5d522870d184857a77771ba12800816adf376ddb2554bc5649c85981'
V11_DOCKERFILE_SHA256 = '4cf6ebd26abf389e2deaf318576b0d467c67491e54224ea56b5198f1be0561e1'
V11_CONTRACT = 'm6a10-online-compute-v4-terminal-bounded-end-gap'


def test_v10_inputs_remain_byte_identical_and_v11_sources_are_pinned():
    for path, expected in V10_FILES.items():
        assert path.is_file()
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected
    assert hashlib.sha256(PATCH.read_bytes()).hexdigest() == V11_PATCH_SHA256
    assert hashlib.sha256(SELFTEST.read_bytes()).hexdigest() == V11_SELFTEST_SHA256
    assert hashlib.sha256(DOCKERFILE.read_bytes()).hexdigest() == V11_DOCKERFILE_SHA256


def test_v11_dockerfile_is_build_only_and_identity_bound():
    text = DOCKERFILE.read_text(encoding='utf-8')
    assert f'ARG FAST_LIVO2_V10_BASE_IMAGE={V10_BASE_TAG}' in text
    assert f'ARG FAST_LIVO2_V10_EXPECTED_BASE_ID={V10_BASE_ID}' in text
    assert 'FROM ${FAST_LIVO2_V10_BASE_IMAGE}' in text
    assert 'COPY docker/patches/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch' in text
    assert 'COPY tools/m6a10_terminal_support_context_v11_selftest.cpp' in text
    assert 'sha256sum /tmp/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch' in text
    assert 'git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --check --recount' in text
    assert 'git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --recount' in text
    assert V11_CONTRACT in text
    assert "! grep -q 'boundary_.timestamp_seconds < required_end_timestamp_seconds_ ||'" in text
    assert 'contract_value == "m6a10-online-compute-v4-terminal-bounded-end-gap"' in text
    assert 'catkin_make -DCMAKE_BUILD_TYPE=Release -j2' in text
    assert 'g++ -std=c++17 -O0' in text
    assert 'm6a10_terminal_support_context_v11_selftest.cpp' in text
    assert 'benchmark.fast_livo2.m6a10_variant="${FAST_LIVO2_M6A10_VARIANT}"' in text
    assert 'benchmark.fast_livo2.m6a10_v10_base_id="${FAST_LIVO2_V10_BASE_ID}"' in text
    assert (
        'benchmark.fast_livo2.m6a10_v11_patch_sha256="${FAST_LIVO2_M6A10_V11_PATCH_SHA256}"'
        in text
    )
    assert 'benchmark.fast_livo2.m6a10_ground_truth_present="false"' in text
    assert 'benchmark.fast_livo2.m6a10_scorer_present="false"' in text
    assert 'benchmark.fast_livo2.m6a10_formal_replay_forbidden="true"' in text
    assert 'docker run' not in text
    assert 'rosbag' not in text.lower()


def test_v11_build_script_has_one_network_isolated_build_and_immutable_receipts():
    text = BUILD_SCRIPT.read_text(encoding='utf-8')
    assert BUILD_SCRIPT.stat().st_mode & 0o111
    assert V10_BASE_TAG in text
    assert V10_BASE_ID in text
    assert V11_PATCH_SHA256 in text
    assert V11_SELFTEST_SHA256 in text
    assert V11_CONTRACT in text
    assert 'IMAGE_TAG="${FAST_LIVO2_V11_IMAGE_TAG:-m6a10-v2c-v11-bounded-end-gap-' in text
    assert text.count('docker build --network none --pull=false') == 1
    assert '--build-arg "FAST_LIVO2_V10_BASE_ID=${BASE_ID}"' in text
    assert '--build-arg "FAST_LIVO2_M6A10_V11_PATCH_SHA256=${EXPECTED_PATCH_SHA256}"' in text
    assert '--build-arg "FAST_LIVO2_M6A10_V11_SELFTEST_SHA256=${EXPECTED_SELFTEST_SHA256}"' in text
    docker_text = DOCKERFILE.read_text(encoding='utf-8')
    assert 'ARG FAST_LIVO2_M6A10_V11_PATCH_SHA256' in docker_text
    assert 'ARG FAST_LIVO2_M6A10_V11_SELFTEST_SHA256' in docker_text
    assert '--build-arg "FAST_LIVO2_M6A10_V11_PATCH_SHA256=${EXPECTED_PATCH_SHA256}"' in text
    assert '--build-arg "FAST_LIVO2_M6A10_V11_SELFTEST_SHA256=${EXPECTED_SELFTEST_SHA256}"' in text
    assert 'docker run' not in text
    assert 'rosbag' not in text.lower()
    assert 'test ! -e "${RECEIPT_PATH}"' in text
    assert 'test ! -e "${SIDECAR_PATH}"' in text
    assert '[[ "${IMAGE_TAG}" == *v11* ]]' in text
    assert '[[ "${IMAGE_TAG}" != "${BASE_IMAGE}" ]]' in text
    assert 'ln "${TMP_DIR}/build.receipt.json" "${RECEIPT_PATH}"' in text
    assert 'ln "${TMP_DIR}/build.receipt.json.sha256" "${SIDECAR_PATH}"' in text
    assert 'chmod 0444 "${RECEIPT_PATH}" "${SIDECAR_PATH}"' in text
    assert 'formal_replay_forbidden' in text
    assert 'ground_truth_content_opened' in text
    assert 'scorer_invoked' in text


def test_v11_production_selftest_keeps_v10_matrix_and_adds_gap_cases():
    text = SELFTEST.read_text(encoding='utf-8')
    assert '#include "m6a10_terminal_support_context.h"' in text
    assert V11_CONTRACT in text
    names = (
        'zero_backlog',
        'post_boundary_imu_image_tail',
        'residual_lidar',
        'pre_boundary_tail',
        'discard_clear',
        'active_inflight',
        'same_rpc_stability',
        'missing_eof',
        'bounded_end_gap',
        'excessive_end_gap',
        'future_boundary',
    )
    for name in names:
        assert f'"{name}"' in text
    assert 'M6A10_SYNTHETIC_CASE " << name' in text
    assert text.count('accept(context, Topic::Lidar, 9.90);') == 1
    assert text.count('complete(context, {Topic::Lidar}, 9.90);') == 1
    assert text.count('accept(context, Topic::Lidar, 9.70);') == 1
    assert text.count('complete(context, {Topic::Lidar}, 9.70);') == 1
    assert text.count('accept(context, Topic::Lidar, 10.10);') == 1
    assert text.count('complete(context, {Topic::Lidar}, 10.10);') == 1
    assert 'expected_pass = true;' in text[text.index('name == "bounded_end_gap"'):]
    assert 'M6A10_SYNTHETIC_ALL ' in text


def test_v11_shell_syntax_is_checked_without_running_the_build():
    result = subprocess.run(
        ['bash', '-n', str(BUILD_SCRIPT)],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
